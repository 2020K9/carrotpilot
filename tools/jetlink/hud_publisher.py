"""Low-priority read-only vehicle display publisher, independent of inference."""
import ctypes
import os
from pathlib import Path
import signal
import sys
import time

from hud_protocol import SnapshotBuilder, HEADER


STATUS = Path('/dev/shm/carrot-jetlink-hud-snapshot.json')
_next_status = [0.]


def write_status(now, report, path=None):
  """Comma side: last snapshot size, largest items, dropped items or the failure, once a second."""
  if now < _next_status[0] and 'error' not in report:
    return
  _next_status[0] = now + 1
  path = path or STATUS
  try:
    import json
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps({'updated': now, **report}))
    os.replace(temporary, path)
  except (OSError, TypeError, ValueError):
    pass


def main(path, nav_map=False):
  from openpilot.common.display_scheduling import DisplayScheduler
  from openpilot.common.params import Params
  ctypes.CDLL(None).prctl(1, signal.SIGTERM, 0, 0, 0)
  if os.getppid() == 1:
    return
  def stop(*_):
    raise SystemExit
  signal.signal(signal.SIGTERM, stop)
  scheduler = DisplayScheduler(7, enabled=True)
  params = Params()
  builder = SnapshotBuilder(nav_map=nav_map)
  next_warning = 0.
  try:
    while True:
      start = time.monotonic()
      scheduler.update(params.get_bool('IsOnroad'))
      try:
        packet = builder.packet()
        if packet is not None:
          temporary = path.with_suffix('.tmp')
          temporary.write_bytes(HEADER.pack(time.monotonic()) + packet)
          os.replace(temporary, path)
          write_status(start, builder.report)
          if builder.report.get('dropped') and start >= next_warning:
            print(f"Jetlink display snapshot items dropped: {builder.report['dropped']}", file=sys.stderr, flush=True)
            next_warning = start + 2
      except Exception as exc:
        # A transient display/size failure expires the last snapshot. Retry in
        # this low-priority worker without disturbing the active model session.
        write_status(start, {'error': f'{type(exc).__name__}: {exc}'[:200], **getattr(builder, 'report', {})})
        if start >= next_warning:
          print(f'Jetlink display snapshot unavailable: {exc}', file=sys.stderr, flush=True)
          next_warning = start + 2
      time.sleep(max(0, start + .1 - time.monotonic()))
  finally:
    builder.close()
    path.unlink(missing_ok=True)
    path.with_suffix('.tmp').unlink(missing_ok=True)


if __name__ == '__main__':
  main(Path(sys.argv[1]), nav_map='--nav-map' in sys.argv[2:])
