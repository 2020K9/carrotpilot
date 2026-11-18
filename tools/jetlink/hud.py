"""Run the existing Carrot renderer using read-only, Wi-Fi-forwarded cereal data."""
import base64
import json
import os
from pathlib import Path
import sys
import time
import types

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'openpilot/selfdrive/carrot/cluster'))
from hud_protocol import read_snapshot, SERVICES, SNAPSHOT_SCHEMA, SNAPSHOT_STATE
from hud_protocol import EVENT_LIMITS, EVENT_LIMIT, PARAM_LIMITS, PARAM_LIMIT

# K9HudNavdy 2 startup marks, Jetson monotonic (same clock as the server marks): first / last NEW valid carState
# (a snapshot arriving is not enough), first USB send call that returned (not the panel showing it). Display only.
MARKS = {}
# Items this Jetson refused or the comma did not send (name -> reason). A refused item keeps its last good value;
# it never takes the other items or the whole HUD down. Written to the HUD status file and logged on change.
REJECTED = {}
_logged = {}


def note_item(name, reason):
  if reason is None:
    REJECTED.pop(name, None)
    _logged.pop(name, None)
    return
  REJECTED[name] = reason
  if _logged.get(name) != reason:
    _logged[name] = reason
    print(f'HUD snapshot item {name} not applied: {reason}', file=sys.stderr, flush=True)


def snapshot_status(now=None):
  """Last good snapshot age, schema, size, last read error and refused items (for the status file / telemetry)."""
  now = time.monotonic() if now is None else now
  received = SNAPSHOT_STATE.get('received')
  age = now - received if received is not None else None
  return {'ok': age is not None and 0 <= age < 1. and SNAPSHOT_STATE.get('schema') == SNAPSHOT_SCHEMA,
          'age_s': round(age, 3) if age is not None else None, 'schema': SNAPSHOT_STATE.get('schema'),
          'bytes': SNAPSHOT_STATE.get('bytes', 0), 'error': SNAPSHOT_STATE.get('error'), 'rejected': dict(REJECTED)}


# K9HudNavdy 3 (11/18): the 3D board renders at 10 FPS like everything else (대감님 11/18 13:05) (libx264, 1 thread, Jetson CPU).
# The rate for the next start is kept in a small file so the restart after a menu change comes up at the new rate.
K9_FPS_BASE, K9_FPS_3D = 10, 10
K9_FPS_FILE = Path(os.environ.get('CARROT_JETLINK_HUD_FPS_FILE', '/dev/shm/carrot-jetlink-hud-fps'))


def k9_start_fps(path=None):
  try:
    value = int((path or K9_FPS_FILE).read_text().strip())
  except (OSError, ValueError):
    return K9_FPS_BASE
  return value if value in (K9_FPS_BASE, K9_FPS_3D) else K9_FPS_BASE


def k9_menu_fps_policy(menu, path=None, received=None):
  """wanted FPS for this K9HudNavdy value; None until the comma's Params really arrived (no restart loop at boot)"""
  if (SNAPSHOT_STATE.get('received') if received is None else received) is None:
    return None
  want = K9_FPS_3D if menu == 3 else K9_FPS_BASE
  try:
    target = path or K9_FPS_FILE
    if k9_start_fps(target) != want or not target.exists():
      tmp = target.with_suffix('.tmp')
      tmp.write_text(str(want))
      os.replace(tmp, target)
  except OSError:
    return None   # cannot remember the rate: never restart for it
  return want


def require_usb_display(scan, expected):
  product = scan(expected)
  if product is None:
    raise RuntimeError('Jetson USB display disappeared before opening; retrying')
  return product


def wait_for_usb_display(scan, expected, sleep=time.sleep):
  announced = False
  while scan(expected) is None:
    if not announced:
      print('Waiting for Jetson USB display', flush=True)
      announced = True
    sleep(1.)


class DisplayParams:
  """Only this renderer sees this adapter; no vehicle Params can be written."""
  def __init__(self, *args, **kwargs):
    self.next_read = 0.
    self.values = {}

  def get(self, key, *args, **kwargs):
    # This adapter exists only inside the Jetson renderer. The saved switch
    # controls the vehicle's USB panel, not a panel attached to this host.
    if key == 'ClusterHud':
      return b'1'
    now = time.monotonic()
    if now >= self.next_read:
      record = read_snapshot()
      # Missing telemetry is not an ignition-off command. Keep the last
      # settings until a fresh C4 snapshot explicitly changes them; SubMaster
      # still invalidates all vehicle data immediately on snapshot loss.
      if record is not None:
        self.values = self.merge(self.values, record[1])
      self.next_read = now + .1
    value = self.values.get(key)
    return base64.b64decode(value) if value is not None else None

  @staticmethod
  def merge(old, value):
    """New Params; a Param the comma dropped or that is malformed/oversized keeps its previous value."""
    params = value.get('params') if isinstance(value, dict) else None
    if not isinstance(params, dict):
      note_item('params', 'missing')
      return old
    note_item('params', None)
    values = {}
    for key, item in params.items():
      limit = PARAM_LIMITS.get(key, PARAM_LIMIT)
      if not isinstance(item, str) or len(item) > limit:
        note_item(f'params.{key}', f'{len(item)} B > {limit} B' if isinstance(item, str) else 'not text')
        item = old.get(key)
      else:
        note_item(f'params.{key}', None)
      if item is not None:
        values[key] = item
    dropped = value.get('dropped')
    for name, reason in (dropped.items() if isinstance(dropped, dict) else ()):
      if isinstance(name, str) and name.startswith('params.'):
        key = name[len('params.'):]
        note_item(name, f'comma: {reason}')
        if key not in values and key in old:
          values[key] = old[key]
    return values

  def get_bool(self, key, *args, **kwargs):
    return self.get(key) in (b'1', b'True')

  def external_compute_label(self):
    snapshot = read_snapshot()
    label = snapshot[1].get('external_compute_label', '') if snapshot else ''
    return label if label in ('jetSON', 'MAC', 'Jetlink') else ''

  def jetlink_roundtrip_ms(self):
    snapshot = read_snapshot()   # comma clock value carried as is; never mixed with Jetson times
    return snapshot[1].get('jetlink_roundtrip_ms') if snapshot else None

  def get_int(self, key, *args, **kwargs):
    try:
      return int(self.get(key) or 0)
    except ValueError:
      return 0

  def get_float(self, key, *args, **kwargs):
    try:
      return float(self.get(key) or 0)
    except ValueError:
      return 0.

  def put_bool_nonblocking(self, key, value):
    if key == 'ClusterHudConnected':
      Path('/dev/shm/carrot-jetlink-hud-connected').write_text(str(bool(value)))


class RemoteSubMaster:
  def __init__(self, services):
    from openpilot.cereal import log
    self.log = log
    self.services = services
    self.data = {}
    self.updated = dict.fromkeys(services, False)
    self.valid = dict.fromkeys(services, False)
    self.alive = dict.fromkeys(services, False)
    self.recv_time = dict.fromkeys(services, 0.)
    self.logMonoTime = dict.fromkeys(services, 0)
    self.generations = {}
    self.last_received = 0.
    for name in services:
      self.data[name] = log.Event.new_message(**{name: {}}) if name != 'liveTracks' else log.Event.new_message(liveTracks={})

  def __getitem__(self, key):
    return getattr(self.data[key], key)

  def update(self, timeout=0):
    self.updated = dict.fromkeys(self.services, False)
    snapshot = read_snapshot()
    if snapshot is None:
      self.valid = dict.fromkeys(self.services, False)
      self.alive = dict.fromkeys(self.services, False)
      return
    received, value = snapshot
    if received == self.last_received:
      return
    self.last_received = received
    SNAPSHOT_STATE['schema'] = value.get('schema')
    try:
      shift = received - float(value['sent'])
      valid, alive, recv, monos, events = (value[k] for k in ('valid', 'alive', 'received', 'mono', 'events'))
      if not all(isinstance(x, dict) for x in (valid, alive, recv, monos, events)):
        raise TypeError('snapshot tables')
    except (KeyError, TypeError, ValueError) as exc:
      # A malformed snapshot is one lost update, not a HUD stop: last values stay, marked not alive.
      note_item('snapshot', f'{type(exc).__name__}: {exc}'[:120])
      self.valid = dict.fromkeys(self.services, False)
      self.alive = dict.fromkeys(self.services, False)
      return
    note_item('snapshot', None)
    dropped = value.get('dropped') if isinstance(value.get('dropped'), dict) else {}
    for name in self.services:
      if name not in SERVICES:
        continue
      if f'events.{name}' in dropped:
        note_item(f'events.{name}', f"comma: {dropped[f'events.{name}']}")
      try:
        mono = int(monos.get(name, 0))
        self.valid[name] = bool(valid.get(name, False))
        self.alive[name] = bool(alive.get(name, False))
        self.recv_time[name] = float(recv.get(name, 0.)) + shift
        self.logMonoTime[name] = int(mono + shift * 1e9)
        if mono == self.generations.get(name) or name not in events:
          continue
        item = events[name]
        limit = EVENT_LIMITS.get(name, EVENT_LIMIT)
        if not isinstance(item, str) or len(item) > limit:
          raise ValueError(f'{len(item)} B > {limit} B' if isinstance(item, str) else 'not text')
        raw = base64.b64decode(item)
        with self.log.Event.from_bytes(raw) as reader:
          if reader.which() != name:
            raise ValueError('display service identity mismatch')
          data = reader.as_builder()
      except Exception as exc:
        note_item(f'events.{name}', f'{type(exc).__name__}: {exc}'[:120])
        continue
      if f'events.{name}' not in dropped:
        note_item(f'events.{name}', None)
      self.data[name] = data
      self.generations[name] = mono
      self.updated[name] = True
      if name == 'carState' and self.valid[name]:
        MARKS['car_last'] = received   # a new carState (new logMonoTime), valid
        MARKS.setdefault('car', received)


def main():
  MARKS.setdefault('hud_start', time.monotonic())
  if '--help' not in sys.argv:
    from boot_update import wait_for_runtime
    wait_for_runtime()
  if '--help' not in sys.argv:
    settings = DisplayParams()
    while not (settings.get_int('ClusterHud') == 1 and
               (settings.get_bool('IsOnroad') or settings.get_int('ClusterHudDebug') >= 1)):
      time.sleep(.2)
  from openpilot.cereal import log
  params_module = types.ModuleType('openpilot.common.params')
  params_module.Params = DisplayParams
  sys.modules[params_module.__name__] = params_module
  messaging = types.ModuleType('openpilot.cereal.messaging')
  messaging.SubMaster = RemoteSubMaster
  from hud_navi import RemoteMediaSocket, MediaPump
  def sub_sock(service, **kwargs):
    if service != 'carrotNaviMedia':
      raise ValueError(f'unsupported display subscription: {service}')
    return RemoteMediaSocket()
  messaging.sub_sock = sub_sock
  messaging.drain_sock = lambda sock: sock.drain()
  def from_bytes(data, schema=log.Event):
    with schema.from_bytes(data) as reader:
      return reader.as_builder()
  messaging.log_from_bytes = from_bytes
  sys.modules[messaging.__name__] = messaging
  from hud_signal import install as install_signal_display
  remote_signals = install_signal_display(read_snapshot)
  from blink_display import Remote
  from openpilot.common import indicator_display
  indicator_display.current = Remote(read_snapshot, time.monotonic).read
  import cluster_live_camera
  from hud_camera import RemoteRoadCamera
  cluster_live_camera.LiveRoadCamera = RemoteRoadCamera
  import cluster_system_monitor
  from hud_stats import VehicleSystemStats, VehicleCpuOverlay, VehicleNetworkAddress
  cluster_system_monitor.SystemStatsSampler = VehicleSystemStats
  cluster_system_monitor.ClusterProcessCoreUsageSampler = VehicleCpuOverlay
  cluster_system_monitor.NetworkAddressProvider = VehicleNetworkAddress
  import cluster_navi_source
  display_metrics = {}
  from openpilot.selfdrive.navigation import preview as nav_preview
  from hud_nav_map import RemoteNavPreview
  remote_nav_preview = RemoteNavPreview(read_snapshot, nav_preview.PREVIEW_PATH)
  nav_preview.preview_fresh = remote_nav_preview.fresh
  BaseNaviSource = cluster_navi_source.NaviIpcMediaSource
  class RemoteNaviSource:
    def __init__(self, *args, **kwargs):
      self.source = BaseNaviSource(*args, **kwargs)
      self.pump = MediaPump(self.source)

    def update(self, navi_live):
      dashboard = self.pump.update(navi_live)
      frame = next((f for f in dashboard.media if f.key == 'render:map_main' and f.present), None)
      display_metrics['map'] = {'sequence': frame.sequence if frame else None,
                                'width': frame.width if frame else None,
                                'height': frame.height if frame else None,
                                'age_ms': dashboard.map_frame_age_ms,
                                'stalled': dashboard.map_stream_stalled,
                                'decode_drops': self.source._h264_worker.dropped_requests,
                                'received': dashboard.received_count}
      return dashboard

    def close(self):
      self.pump.close()
  cluster_navi_source.NaviIpcMediaSource = RemoteNaviSource
  vehicle_stats = VehicleSystemStats()
  import main as cluster
  from hud_health import HealthRenderer
  cluster.ClusterUiRenderer = HealthRenderer
  HealthRenderer.startup_marks = MARKS
  if '--help' not in sys.argv:
    scan = cluster.find_supported_usb_product
    wait_for_usb_display(scan, cluster.product_id_for_hud_mode(1))
    # The common PC renderer falls back to an invisible window if the USB
    # panel is absent. A headless Jetson must wait/retry, including the race
    # where it disappears between the preflight scan and opening the device.
    cluster.find_supported_usb_product = lambda expected: require_usb_display(scan, expected)
  class Display(cluster.TuringUsbDisplay):
    next_status = 0.

    def send_h264_chunk(self, *args, **kwargs):
      result = super().send_h264_chunk(*args, **kwargs)
      self._record_send_status()
      return result

    def send_jpeg(self, *args, **kwargs):
      result = super().send_jpeg(*args, **kwargs)
      self._record_send_status()
      return result

    def _record_send_status(self):
      now = time.monotonic()
      MARKS.setdefault('frame', now)
      if now >= self.next_status:
        path = Path('/dev/shm/carrot-jetlink-hud-status.json')
        tmp = path.with_suffix('.tmp')
        stats = vehicle_stats.sample()
        remote_nav_preview.fresh()
        display_metrics['nav_model_map'] = remote_nav_preview.status
        signal_color, signal_held, signal_online = remote_signals.status()
        display_metrics['signal_display'] = dict(enabled=remote_signals.enabled, color=signal_color,
                                                held=signal_held, online=signal_online)
        tmp.write_text(json.dumps({'updated': now, **display_metrics, 'snapshot': snapshot_status(now),
                                  'vehicle_network_address': VehicleNetworkAddress().address(),
                                  'vehicle_cpu': stats.cpu_core_percents,
                                  'vehicle_memory_percent': stats.memory_used_percent}))
        os.replace(tmp, path)
        self.next_status = now + 1
  cluster.TuringUsbDisplay = Display
  # Jetson has no video encoder block. Keep inference CPU/GPU headroom and
  # use the existing software encoder at the normal 10 FPS cluster rate.
  # Bound conversion/encoding concurrency; an explicit trailing CLI value of
  # zero restores FFmpeg's automatic thread pools for comparisons.
  cluster.K9_MENU_FPS_POLICY = k9_menu_fps_policy
  sys.argv = [__file__, '--input', 'live', '--output', 'usb', '--fps', str(k9_start_fps()),
              '--usb-codec', 'jpeg', '--usb-h264-backend', 'ffmpeg',
              '--usb-h264-ffmpeg-encoder', 'libx264', '--usb-h264-ffmpeg-threads', '1',
              '--cluster-hud-mode', '1', *sys.argv[1:]]
  if '--help' in sys.argv:
    cluster.main()
  else:
    from display_owner import panel_owner
    with panel_owner(hud=True):
      cluster.main()


if __name__ == '__main__':
  main()
