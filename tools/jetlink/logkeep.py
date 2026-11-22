"""K9 log keeper, shared by the comma (openpilot/system/k9_logkeep.py) and the Jetson (host_logkeep.py).

Follows live system logs into plain text files on persistent storage, one folder per boot:

  <root>/<YYYYmmdd-HHMMSS>_<boot id 8>/journal.log   whole system journal (= syslog, service output)
                                       kern.log      kernel messages (journalctl -k = dmesg, with wall time)
                                       dmesg_start.txt  kernel ring buffer once at keeper start
                                       <extra>.log   further followers (Jetson: carrot-* service units)

Each stream is cut into parts of PART_BYTES; finished parts are gzip'd. Lines are flushed at once and
fsync'd every FSYNC_S, so a power cut loses at most that much. Retention: every file older than
KEEP_S (3 days) is deleted, and the oldest files go first while the folder is over MAX_BYTES or the disk
has less than MIN_FREE_BYTES free. The current boot's open files are never deleted. Only the keeper's
own folder is ever pruned. stdlib only (runs under /usr/bin/python3 on the Jetson).
"""
import gzip
import os
from pathlib import Path
import re
import shutil
import subprocess
import threading
import time

KEEP_S = 3 * 86400
MAX_BYTES = 1 << 30
MIN_FREE_BYTES = 2 << 30
PART_BYTES = 16 << 20
FSYNC_S = 2.
PRUNE_S = 600.
RESTART_S = 5.
# short-iso-precise: 2026-11-22T15:49:12.633705+09:00 (systemd >= 25x) or ...+0900 (older)
_STAMP = re.compile(rb'^(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d)(?:[.,]\d+)?(Z|[+-]\d\d:?\d\d)?\s')


def boot_id():
  try:
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip().replace('-', '') or 'unknown'
  except OSError:
    return 'unknown'


def boot_dir(root, bid=None, now=None):
  """This boot's folder (reused when the keeper restarts within the same boot)."""
  root = Path(root)
  bid = (bid or boot_id())[:8]
  try:
    for d in sorted(root.iterdir()):
      if d.is_dir() and d.name.endswith('_' + bid):
        return d
  except OSError:
    pass
  stamp = time.strftime('%Y%m%d-%H%M%S', time.localtime(now if now is not None else time.time()))  # noqa: TID251 -- names the folder only
  return root / f'{stamp}_{bid}'


def _compress(path):
  gz = path.with_name(path.name + '.gz')
  tmp = gz.with_name(gz.name + '.tmp')
  try:
    with open(path, 'rb') as src, gzip.open(tmp, 'wb', compresslevel=6) as dst:
      shutil.copyfileobj(src, dst, 1 << 20)
    os.replace(tmp, gz)
    path.unlink()
  except OSError:
    try:
      tmp.unlink()
    except OSError:
      pass


class Sink:
  """Append-only text stream split into parts: name.log, name.1.log, ... (finished parts -> .gz)."""

  def __init__(self, folder, name, part_bytes=PART_BYTES, mode=0o640):
    self.folder, self.name, self.part_bytes, self.mode = Path(folder), name, part_bytes, mode
    self.lock = threading.Lock()
    idx = 0
    while self._path(idx).exists() or self._path(idx, '.gz').exists():
      idx += 1
    if idx > 0 and self._path(idx - 1).exists():
      idx -= 1        # keeper restart: continue the last unfinished part
    self.index = idx
    self.f = None
    self.size = 0
    self.dirty = False
    self._open()

  def _path(self, index, suffix=''):
    return self.folder / ((f'{self.name}.log' if index == 0 else f'{self.name}.{index}.log') + suffix)

  @property
  def path(self):
    return self._path(self.index)

  def _open(self):
    self.f = open(self.path, 'ab')
    try:
      os.chmod(self.path, self.mode)
    except OSError:
      pass
    self.size = self.f.tell()

  def write(self, data):
    with self.lock:
      self.f.write(data)
      self.f.flush()
      self.size += len(data)
      self.dirty = True
      if self.size >= self.part_bytes:
        self._rotate()

  def _rotate(self):
    old = self.path
    self.f.flush()
    os.fsync(self.f.fileno())
    self.f.close()
    self.index += 1
    self._open()
    t = threading.Thread(target=_compress, args=(old,), daemon=True)
    self.compressing = [x for x in getattr(self, 'compressing', []) if x.is_alive()] + [t]
    t.start()

  def sync(self):
    with self.lock:
      if self.dirty and self.f is not None:
        try:
          os.fsync(self.f.fileno())
        except OSError:
          pass
        self.dirty = False

  def close(self):
    for t in getattr(self, 'compressing', []):
      t.join(60)
    with self.lock:
      if self.f is not None:
        try:
          self.f.flush()
          os.fsync(self.f.fileno())
        except OSError:
          pass
        self.f.close()
        self.f = None


def _last_stamp(path, tail=65536):
  """'YYYY-MM-DD HH:MM:SS' (local) of the last short-iso-precise line in path, or None."""
  import datetime
  try:
    with open(path, 'rb') as f:
      f.seek(0, os.SEEK_END)
      f.seek(max(0, f.tell() - tail))
      lines = f.read().splitlines()
  except OSError:
    return None
  for line in reversed(lines):
    m = _STAMP.match(line)
    if m is None:
      continue
    try:
      t = datetime.datetime.strptime(m.group(1).decode(), '%Y-%m-%dT%H:%M:%S')
    except ValueError:
      continue
    tz = m.group(2)
    if tz:
      tz = tz.decode()
      off = 0 if tz == 'Z' else (1 if tz[0] == '+' else -1) * (int(tz[1:3]) * 60 + int(tz[-2:]))
      t = t.replace(tzinfo=datetime.timezone(datetime.timedelta(minutes=off))).astimezone()
    return t.strftime('%Y-%m-%d %H:%M:%S')
  return None


def prune(root, keep_s=KEEP_S, max_bytes=MAX_BYTES, min_free=MIN_FREE_BYTES, protect=(), now=None):
  """Delete keeper files older than keep_s, then the oldest while over max_bytes / under min_free free.
  -> list of deleted paths. Never touches anything outside root, never deletes a path in protect."""
  root = Path(root)
  now = time.time() if now is None else now  # noqa: TID251 -- file ages are wall-clock mtimes
  protect = {Path(p) for p in protect}
  files = []
  for dirpath, _, names in os.walk(root):
    for n in names:
      p = Path(dirpath) / n
      try:
        st = p.lstat()
      except OSError:
        continue
      files.append((st.st_mtime, p, st.st_size))
  files.sort()
  total = sum(s for _, _, s in files)
  deleted = []

  def free():
    try:
      st = os.statvfs(root)
      return st.f_bavail * st.f_frsize
    except OSError:
      return None

  for mtime, p, size in files:
    if p in protect:
      continue
    fr = free() if min_free else None
    if mtime < now - keep_s or total > max_bytes or (fr is not None and fr < min_free):
      try:
        p.unlink()
        deleted.append(p)
        total -= size
      except OSError:
        pass
  for dirpath, dirs, names in sorted(os.walk(root), key=lambda x: -len(x[0])):
    d = Path(dirpath)
    if d != root and not dirs and not names and not any(d == pp.parent for pp in protect):
      try:
        d.rmdir()
      except OSError:
        pass
  return deleted


def previous_boot_kernel(root, current=None, limit=262144):
  """Tail of the newest earlier boot's kern.log (the Jetson journal is volatile: journalctl -b -1 is empty)."""
  root = Path(root)
  current = current or boot_dir(root)
  try:
    dirs = sorted(d for d in root.iterdir() if d.is_dir() and d != current)
  except OSError:
    return ''
  if not dirs:
    return ''
  d = dirs[-1]
  parts = sorted(d.glob('kern*.log'), key=lambda p: p.stat().st_mtime)
  if not parts:
    return ''
  data = parts[-1].read_bytes()[-limit:]
  return f'# {parts[-1]}\n' + data.decode('utf-8', 'replace')


class Keeper:
  def __init__(self, root, streams, *, snapshots=None, hooks=(), keep_s=KEEP_S, max_bytes=MAX_BYTES,
               min_free=MIN_FREE_BYTES, part_bytes=PART_BYTES, fsync_s=FSYNC_S, prune_s=PRUNE_S, mode=0o640,
               group=None):
    """streams: {name: argv} followed line by line (restarted if they exit). snapshots: {name: argv} run once.
    hooks: callables(keeper) run at every prune tick (e.g. archive swaglog / crash files into root)."""
    self.root = Path(root)
    self.streams, self.snapshots, self.hooks = dict(streams), dict(snapshots or {}), list(hooks)
    self.keep_s, self.max_bytes, self.min_free = keep_s, max_bytes, min_free
    self.part_bytes, self.fsync_s, self.prune_s, self.mode, self.group = part_bytes, fsync_s, prune_s, mode, group
    self.stop = threading.Event()
    self.procs = {}
    self.sinks = {}
    self.dir = None

  def _own(self, path, dmode=0o750):
    try:
      os.chmod(path, dmode)
      if self.group is not None and os.geteuid() == 0:
        os.chown(path, 0, self.group)
    except OSError:
      pass

  def open(self):
    self.root.mkdir(parents=True, exist_ok=True)
    self._own(self.root)
    self.dir = boot_dir(self.root)
    self.dir.mkdir(parents=True, exist_ok=True)
    self._own(self.dir)
    for name, argv in self.snapshots.items():
      try:
        r = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=20)
        text = r.stdout
      except (OSError, subprocess.SubprocessError) as exc:
        text = f'{name} unavailable: {type(exc).__name__}: {exc}\n'.encode()
      p = self.dir / f'{name}.txt'
      with open(p, 'wb') as f:
        f.write(text[-(8 << 20):])
        f.flush()
        os.fsync(f.fileno())
      os.chmod(p, self.mode)
    for name in self.streams:
      self.sinks[name] = Sink(self.dir, name, self.part_bytes, self.mode)
    return self

  def protected(self):
    return {s.path for s in self.sinks.values()}

  def _follow(self, name, argv):
    sink = self.sinks[name]
    while not self.stop.is_set():
      cmd = list(argv)
      since = _last_stamp(sink.path) if cmd and cmd[0] == 'journalctl' else None
      if since:
        # keeper restarted within this boot: continue from the last kept second (no re-dump of the boot)
        if '-n' in cmd:
          i = cmd.index('-n')
          del cmd[i:i + 2]
        cmd += ['--since', since]
      try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL)
      except OSError as exc:
        sink.write(f'# keeper: cannot start {cmd[0]}: {exc}\n'.encode())
        self.stop.wait(60)
        continue
      self.procs[name] = proc
      if self.stop.is_set():
        proc.terminate()
      for line in proc.stdout:
        sink.write(line)
        if self.stop.is_set():
          break
      try:
        proc.terminate()
        proc.wait(5)
      except (OSError, subprocess.SubprocessError):
        pass
      proc.stdout.close()
      if not self.stop.is_set():
        sink.write(f'# keeper: {cmd[0]} exited ({proc.returncode}), restarting\n'.encode())
        self.stop.wait(RESTART_S)

  def tick(self, now=None):
    for hook in self.hooks:
      try:
        hook(self)
      except Exception as exc:  # a hook never stops log keeping
        try:
          self.sinks[next(iter(self.sinks))].write(f'# keeper: hook {getattr(hook, "__name__", hook)} {type(exc).__name__}: {exc}\n'.encode())
        except Exception:
          pass
    return prune(self.root, self.keep_s, self.max_bytes, self.min_free, self.protected(), now)

  def run(self):
    self.open()
    self.threads = [threading.Thread(target=self._follow, args=(n, a), daemon=True, name=f'logkeep-{n}')
                    for n, a in self.streams.items()]
    for t in self.threads:
      t.start()
    last_prune = float('-inf')
    try:
      while not self.stop.is_set():
        for s in self.sinks.values():
          s.sync()
        if time.monotonic() - last_prune >= self.prune_s:
          last_prune = time.monotonic()
          self.tick()
        self.stop.wait(self.fsync_s)
    finally:
      self.close()

  def close(self):
    self.stop.set()
    for p in list(self.procs.values()):
      try:
        p.terminate()
      except OSError:
        pass
    for t in getattr(self, 'threads', []):
      if t is not threading.current_thread():
        t.join(10)
    for s in self.sinks.values():
      s.close()


def journal_streams(extra=None):
  """journalctl followers for this whole boot (a restarted keeper continues with --since, see Keeper._follow)."""
  base = ['journalctl', '--no-pager', '-b', '-f', '-n', 'all', '-o', 'short-iso-precise']
  streams = {'journal': base, 'kern': base + ['-k']}
  for name, units in (extra or {}).items():
    streams[name] = base + [a for u in units for a in ('-u', u)]
  return streams
