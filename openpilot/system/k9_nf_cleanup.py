"""comma: remove what the retired K9 NaN tracer / log keeper left behind (2026-11-25).

Only these two folders, made by K9 code alone (k9_logkeep and the nf_trace offroad pull), are removed:
  /data/media/0/k9_logs      (journal / kern / dmesg_start / swaglog / crash copies)
  /data/media/0/k9_nf_trace  (NOT_FINITE events pulled from the Jetson)
Symlinks are never followed; anything else (rlog/qlog, /data/log, params, models, keys) is not touched.
"""
import os
from pathlib import Path
import shutil
import threading

MEDIA = Path('/data/media/0')
NAMES = ('k9_logs', 'k9_nf_trace')


def targets(media=MEDIA):
  return [Path(media) / name for name in NAMES]


def cleanup(media=MEDIA):
  removed = []
  for path in targets(media):
    try:
      if path.is_symlink():
        path.unlink()            # the link only, never what it points to
      elif path.is_dir():
        shutil.rmtree(path, ignore_errors=True)
      elif path.exists():
        os.unlink(path)
      else:
        continue
      removed.append(str(path))
    except OSError:
      pass
  return removed


def cleanup_async(media=MEDIA):
  t = threading.Thread(target=cleanup, args=(media,), name='k9_nf_cleanup', daemon=True)
  t.start()
  return t
