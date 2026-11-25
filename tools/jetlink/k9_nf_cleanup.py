"""Jetson: remove what the retired K9 NaN tracer / log keeper left behind (2026-11-25).

Started as root (sudo -n, detached) by server.py on every Jetson server start, i.e. after each update.
Removed, and nothing else:
  <cache>/nf_trace                      NOT_FINITE tracer events (cache = /opt/carrot-jetlink/cache,
                                        /mnt/data/jetlink or $JETLINK_CACHE); engines/models beside it stay
  /run/carrot-data/logs/<YYYYMMDD-HHMMSS>_<boot id 8 hex>/   per-boot logkeep folders (journal / kern /
                                        carrot .log(.gz), dmesg_start.txt); the logs folder itself only if empty
  transient unit carrot-jetlink-logkeep (stopped if still running)
Symlinks are never followed.
"""
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

CACHE_ROOTS = [Path('/opt/carrot-jetlink/cache'), Path('/mnt/data/jetlink')]
LOGS = Path('/run/carrot-data/logs')
UNIT = 'carrot-jetlink-logkeep'
BOOT_DIR = re.compile(r'^\d{8}-\d{6}_[0-9a-f]{8}$')
LOG_FILE = re.compile(r'^((journal|kern|carrot)(\.\d+)?\.log(\.gz)?(\.tmp)?|dmesg_start\.txt)$')


def _remove(path):
  if path.is_symlink():
    path.unlink()
  elif path.is_dir():
    shutil.rmtree(path)
  else:
    path.unlink()


def cache_roots(env=None):
  env = os.environ if env is None else env
  roots = list(CACHE_ROOTS)
  if env.get('JETLINK_CACHE'):
    roots.append(Path(env['JETLINK_CACHE']))
  return roots


def cleanup(cache=None, logs=LOGS, run=subprocess.run):
  removed = []
  for root in (cache if cache is not None else cache_roots()):
    path = Path(root) / 'nf_trace'
    try:
      if path.is_symlink() or path.exists():
        _remove(path)
        removed.append(str(path))
    except OSError:
      pass
  logs = Path(logs)
  try:
    if logs.is_dir() and not logs.is_symlink():
      for d in sorted(logs.iterdir()):
        # only logkeep's own per-boot folders holding only its own files
        if d.is_symlink() or not d.is_dir() or not BOOT_DIR.match(d.name):
          continue
        if all(f.is_file() and not f.is_symlink() and LOG_FILE.match(f.name) for f in d.iterdir()):
          try:
            shutil.rmtree(d)
            removed.append(str(d))
          except OSError:
            pass
      if not any(logs.iterdir()):
        logs.rmdir()
        removed.append(str(logs))
  except OSError:
    pass
  if run is not None:
    try:
      if run(['systemctl', 'is-active', '--quiet', UNIT], timeout=10).returncode == 0:
        run(['systemctl', 'stop', UNIT], timeout=20)
        removed.append(UNIT)
    except (OSError, subprocess.SubprocessError):
      pass
  return removed


def start_as_root(popen=subprocess.Popen):
  """server.py (user jetlink): run this file as root in the background; never delays or stops the server."""
  try:
    popen(['sudo', '-n', sys.executable, str(Path(__file__).resolve())], start_new_session=True,
          stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return True
  except (OSError, subprocess.SubprocessError):
    return False


if __name__ == '__main__':
  for p in cleanup():
    print('removed', p, flush=True)
  sys.exit(0)
