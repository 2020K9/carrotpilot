#!/usr/bin/env python3
"""comma: K9 log keeper (always on, low priority). Keeps 3 days (1 GB cap) under /data/media/0/k9_logs:

  <time>_<boot id>/journal.log, kern.log, dmesg_start.txt   system journal / kernel (dmesg, kmsg) per boot
  swaglog/swaglog_<YYYYmmdd>.gz                              copies of rotated /data/log/swaglog.N (openpilot)
  crash/<name>                                               copies of /data/carrot/process_crashes, /data/tombstones

Only reads the openpilot folders (logmessaged keeps its own 2500-file rotation, rlog/qlog/realdata
and deleter are untouched); deletes only inside its own folder. See tools/jetlink/logkeep.py.
"""
import gzip
import json
import os
from pathlib import Path
import shutil
import time

from tools.jetlink import logkeep  # noqa: TID251 -- shared with the Jetson keeper

ROOT = Path(os.environ.get('K9_LOGKEEP_ROOT', '/data/media/0/k9_logs'))
SWAGLOG = Path('/data/log')
CRASH_DIRS = (Path('/data/carrot/process_crashes'), Path('/data/tombstones'))
SETTLE_S = 120.          # a swaglog file untouched this long has been rotated away
MAX_COPY_PER_TICK = 400


def _state(root):
  p = root / '.state.json'
  try:
    return json.loads(p.read_text())
  except (OSError, ValueError):
    return {}


def _save_state(root, state):
  p = root / '.state.json'
  tmp = p.with_name('.state.json.tmp')
  tmp.write_text(json.dumps(state))
  os.replace(tmp, p)


def archive_swaglog(keeper, src=SWAGLOG, now=None):
  """Append each finished swaglog.N (once, in order) to swaglog/swaglog_<day>.gz (one gzip member each)."""
  now = time.time() if now is None else now  # noqa: TID251 -- compares file mtimes
  root = keeper.root
  state = _state(root)
  last = int(state.get('swaglog', -1))
  todo = []
  try:
    for p in src.glob('swaglog.*'):
      try:
        n = int(p.name.split('.', 1)[1])
        st = p.stat()
      except (ValueError, OSError):
        continue
      if n > last and st.st_mtime < now - SETTLE_S and st.st_mtime >= now - keeper.keep_s:
        todo.append((n, p, st.st_mtime))
  except OSError:
    return 0
  todo.sort()
  out_dir = root / 'swaglog'
  done = 0
  for n, p, mtime in todo[:MAX_COPY_PER_TICK]:
    out_dir.mkdir(parents=True, exist_ok=True)
    day = time.strftime('%Y%m%d', time.localtime(mtime))
    try:
      data = p.read_bytes()
    except OSError:
      continue
    with gzip.open(out_dir / f'swaglog_{day}.gz', 'ab', compresslevel=6) as f:
      f.write(data)
    last = n
    done += 1
  if done:
    state['swaglog'] = last
    _save_state(root, state)
  return done


def copy_crashes(keeper, dirs=CRASH_DIRS, now=None):
  """Copy new crash / tombstone files (by name + size) into crash/, flat; keeps them for 3 days."""
  now = time.time() if now is None else now  # noqa: TID251 -- compares file mtimes
  out_dir = keeper.root / 'crash'
  copied = 0
  for d in dirs:
    try:
      entries = [p for p in d.rglob('*') if p.is_file()]
    except OSError:
      continue
    for p in entries:
      try:
        st = p.stat()
      except OSError:
        continue
      if st.st_mtime < now - keeper.keep_s:
        continue
      rel = '__'.join(p.relative_to(d.parent).parts)[-180:]
      dst = out_dir / rel
      try:
        if dst.exists() and dst.stat().st_size == st.st_size:
          continue
        out_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, dst)
        copied += 1
      except OSError:
        continue
  return copied


def keeper(root=ROOT):
  return logkeep.Keeper(root, logkeep.journal_streams(), snapshots={'dmesg_start': ['dmesg', '-T']},
                        hooks=[archive_swaglog, copy_crashes], prune_s=60., mode=0o644)


def main():
  try:
    os.nice(15)
  except OSError:
    pass
  keeper().run()


if __name__ == '__main__':
  main()
