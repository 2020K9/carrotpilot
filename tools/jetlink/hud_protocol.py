"""Read-only display snapshots over a negotiated Jetlink extension."""
import base64
import json
from pathlib import Path
import struct
import socket
import subprocess
import sys
import time

HUD_MESSAGE = 0x4000
HUD_CAPABILITY = 'carrot_hud_v1'
MAX_HUD_BYTES = 512 * 1024
HUD_PACKET = Path('/dev/shm/carrot-jetlink-hud.packet')
HEADER = struct.Struct('<d')
SERVICES = ('carState', 'carParams', 'modelV2', 'radarState', 'liveTracks', 'longitudinalPlan',
            'lateralPlan', 'controlsState', 'selfdriveState', 'carControl', 'carOutput', 'deviceState',
            'roadCameraState', 'cameraOdometry', 'liveCalibration', 'livePose', 'drivingModelData',
            'liveDelay', 'liveParameters', 'liveTorqueParameters', 'navInstruction', 'navInstructionCarrot',
            'navRoute', 'carrotMan', 'carrotNavi', 'wideRoadCameraState')
PARAMS = ('ClusterHud', 'ClusterHudDebug', 'ClusterHudBrightness', 'ClusterHudOrientation',
          'ClusterHudMirror', 'ClusterHudTheme', 'ClusterHudRadarInfo', 'ClusterHudRadarDisplay',
          'ClusterHudRadarSourceColor', 'ClusterHudCameraViewMode', 'ClusterHudPanelLayout',
          'ClusterHudScreenMode', 'ShowPlotMode', 'LanguageSetting', 'IsMetric', 'ShowDateTime',
          'IsOnroad', 'CarParams', 'CalibrationParams', 'CustomSteerRatio', 'SteerActuatorDelay',
          'LateralAccelerationFactor', 'UsbGpuActive',
          # The K9 HUD (대감님 HUD) reads these on the vehicle; without them a Jetson panel falls back
          # to defaults (K9 D cards/Navdy/season switches, the five lat lines above the TPMS car).
          'ClusterHudK9Style', 'K9HudNavdy', 'K9MeterSeason', 'CustomSR', 'SteerRatioRate',
          'LateralTorqueCustom', 'LateralTorqueAccelFactor', 'LateralTorqueFriction',
          'LiveDelay', 'LiveTorqueParameters', 'LiveParameters', 'LiveParametersV2', 'UsbGpuLoading',
          # 11/24 18:29: the comma repository/branch/commit, shown small at the HUD bottom-left (hud_health)
          'GitRemote', 'GitBranch', 'GitCommit')

# Snapshot schema 2 (2026-11-17): a snapshot carries only what the Jetson HUD draws. Every service and every
# capnp-valued Param is rebuilt from a per-field allow list instead of being forwarded whole - 98d17e01 forwarded
# LiveTorqueParameters whole and torqued's cached point cloud (12000 points, 512 KB base64) broke every snapshot.
# Field lists = the service's schema fields that the HUD code (cluster/*.py, ext_hud, cutin_alert, deceleration_source,
# indicator_display, tools/jetlink/hud*.py) names anywhere, minus fields known to be unused there
# (test_hud_payload.py re-derives this and fails when the HUD starts reading a field that is not sent).
SNAPSHOT_SCHEMA = 2
SERVICE_FIELDS = {
  'carState': ('vEgo', 'gas', 'brake', 'steeringAngleDeg', 'cruiseState', 'gearShifter', 'aEgo', 'brakeLights',
               'leftBlinker', 'rightBlinker', 'leftBlindspot', 'rightBlindspot', 'fuelGauge', 'charging', 'vEgoCluster',
               'vCruise', 'vCruiseCluster', 'pcmCruiseGap', 'speedLimit', 'gearStep', 'tpms', 'leftLatDist',
               'rightLatDist', 'leftLongDist', 'rightLongDist', 'leftLaneLine', 'rightLaneLine', 'datetime',
               'leftRearLongDist', 'rightRearLongDist', 'leftRearLatDist', 'rightRearLatDist', 'ureaGauge',
               'evModeActive', 'evModeValid', 'vehicleNaviAvailable'),
  'carParams': ('brand', 'carFingerprint', 'wheelbase', 'steerRatio', 'flags', 'maxLateralAccel', 'extFlags'),
  'modelV2': ('frameAge', 'frameDropPerc', 'position', 'orientation', 'velocity', 'orientationRate', 'laneLines',
              'laneLineProbs', 'roadEdges', 'leads', 'meta', 'laneLineStds', 'roadEdgeStds', 'modelExecutionTime',
              'leadsV3', 'acceleration', 'confidence', 'action'),
  # leadLeft/leadRight (11/23): side cars on the HUD road; +~190 B, the largest 11/18 copy stays < EVENT_LIMIT, and an
  # older Jetson reads the event as before (unknown-to-it fields are ignored, an older comma simply omits them)
  'radarState': ('leadOne', 'leadTwo', 'leadsCutIn', 'leadLeft', 'leadRight'),
  'liveTracks': ('points', 'errors'),
  'longitudinalPlan': ('fcw', 'longitudinalPlanSource', 'accels', 'speeds', 'jerks', 'shouldStop', 'allowThrottle',
                       'allowBrake', 'trafficState', 'events', 'vTargetNow', 'cruiseTarget', 'jTargetNow', 'tFollow',
                       'desiredDistance', 'myDrivingMode'),
  'lateralPlan': ('laneWidth', 'mpcSolutionValid', 'desire', 'laneChangeState', 'laneChangeDirection', 'curvatures',
                  'curvatureRates', 'useLaneLines', 'solverCost', 'latDebugText', 'position', 'distances'),
  'controlsState': ('curvature', 'lateralControlState', 'desiredCurvature', 'activeLaneLine', 'k9LaneState',
                    'k9LaneAccel'),
  'selfdriveState': ('state', 'enabled', 'active', 'alertText1', 'alertText2', 'alertStatus', 'alertSize', 'alertType',
                     'alertSound', 'personality'),
  'carControl': ('enabled', 'actuators', 'latActive', 'leftBlinker', 'rightBlinker'),
  'carOutput': ('actuatorsOutput',),
  'deviceState': ('freeSpacePercent', 'fanSpeedPercentDesired', 'started', 'thermalStatus', 'memoryUsagePercent',
                  'cpuTempC', 'gpuTempC', 'memoryTempC', 'gpuUsagePercent', 'cpuUsagePercent',
                  'screenBrightnessPercent', 'pmicTempC', 'powerDrawW', 'deviceType', 'dspTempC', 'bottomSocTempC'),
  'roadCameraState': ('transform', 'gain', 'sensor', 'exposureValPercent'),
  'cameraOdometry': ('trans', 'rot', 'transStd', 'rotStd', 'wideFromDeviceEuler', 'roadTransformTrans',
                     'roadTransformTransStd'),
  'liveCalibration': ('calPerc', 'rpyCalib', 'wideFromDeviceEuler', 'height'),
  'livePose': ('angularVelocityDevice', 'inputsOK', 'sensorsOK', 'timestamp'),
  'drivingModelData': ('action', 'laneLineMeta', 'meta', 'path', 'frameDropPerc', 'modelExecutionTime'),
  'liveDelay': ('lateralDelay', 'status', 'lateralDelayEstimate', 'calPerc'),     # not the lag points
  'liveParameters': ('valid', 'angleOffsetDeg', 'stiffnessFactor', 'steerRatio', 'sensorValid', 'posenetValid', 'roll'),
  'liveTorqueParameters': ('liveValid', 'latAccelFactorRaw', 'frictionCoefficientRaw', 'latAccelFactorFiltered',
                           'frictionCoefficientFiltered', 'version', 'calPerc'),   # not torqued's point cloud
  'navInstruction': ('lanes', 'speedLimit'),
  'navInstructionCarrot': ('lanes', 'speedLimit'),
  'navRoute': ('coordinates',),
  'carrotMan': ('remote', 'desiredSpeed', 'desiredSource', 'xPosLat', 'xPosLon', 'xPosAngle', 'trafficState',
                'vehicleNaviAvailable'),
  'carrotNavi': ('schemaVersion', 'generation', 'sessionId', 'connected', 'vehicle', 'guidanceCurrent', 'guidanceNext',
                 'laneCurrent', 'laneAhead', 'speed', 'trafficSignal', 'crossroad', 'route', 'navigationStatus'),
}
SERVICE_FIELDS['wideRoadCameraState'] = SERVICE_FIELDS['roadCameraState']
# capnp-valued Params: (Event service or None for car.CarParams, fields). LiveParameters is JSON: steerRatio only.
PARAM_FIELDS = {'CarParams': (None, SERVICE_FIELDS['carParams']),
                'CalibrationParams': ('liveCalibration', SERVICE_FIELDS['liveCalibration']),
                'LiveDelay': ('liveDelay', SERVICE_FIELDS['liveDelay']),
                'LiveTorqueParameters': ('liveTorqueParameters', SERVICE_FIELDS['liveTorqueParameters']),
                'LiveParametersV2': ('liveParameters', SERVICE_FIELDS['liveParameters'])}
LIVE_PARAMETERS_JSON = ('steerRatio',)
# Per-item ceilings (base64 bytes in the snapshot). Comma drops an item above its ceiling (the Jetson keeps the
# previous value) and the Jetson rejects one the same way, so one oversized item can never fail a whole snapshot.
# navRoute is the only item whose size follows the route; it also keeps its old last-resort drop in encode_snapshot.
# 11/13 rlog (3 segments) largest HUD copy: modelV2 12568, carrotNavi 2604, liveTracks 1996, lateralPlan 1240,
# every other service <= 672 (whole events were: radarState 3224, carrotMan 1248, ...).
EVENT_LIMITS = {'modelV2': 16 * 1024, 'navRoute': 40 * 1024, 'carrotNavi': 4 * 1024, 'liveTracks': 4 * 1024,
                'lateralPlan': 2 * 1024}
EVENT_LIMIT = 1024
PARAM_LIMITS = {'CarParams': 512, 'CalibrationParams': 512, 'LiveDelay': 256, 'LiveTorqueParameters': 256,
                'LiveParametersV2': 256}
PARAM_LIMIT = 128
SNAPSHOT_LIMIT = 96 * 1024


class Publisher:
  """Keep cereal decoding, JSON and preview work outside the USB owner's GIL."""
  def __init__(self, navi=False, nav_map=False):
    import os
    self.media_socket = self.media_process = None
    if navi:
      self.media_socket, child = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
      self.media_socket.setblocking(False)
      try:
        self.media_process = subprocess.Popen(['chrt', '--other', '0', sys.executable,
          str(Path(__file__).with_name('hud_navi.py')), str(child.fileno())], pass_fds=(child.fileno(),),
          stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
      finally:
        child.close()
    self.path = Path(f'/dev/shm/carrot-jetlink-display-{os.getpid()}.packet')
    self.last_sent = 0.
    self.process = subprocess.Popen(['chrt', '--other', '0', sys.executable,
                                    str(Path(__file__).with_name('hud_publisher.py')), str(self.path),
                                    *(['--nav-map'] if nav_map else [])],
                                    stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL)

  def media_packet(self):
    if self.media_socket is not None:
      from hud_navi import CHUNK, HEADER as MEDIA_HEADER
      try:
        return self.media_socket.recv(CHUNK + MEDIA_HEADER.size)
      except BlockingIOError:
        pass
    return None

  def packet(self):
    try:
      with self.path.open('rb') as f:
        data = f.read(SNAPSHOT_LIMIT + HEADER.size + 1)
      if not HEADER.size < len(data) <= SNAPSHOT_LIMIT + HEADER.size:
        return None
      stamp, = HEADER.unpack_from(data)
      if stamp == self.last_sent or not 0 <= time.monotonic() - stamp < .3:
        return None
      self.last_sent = stamp
      return data[HEADER.size:]
    except (OSError, struct.error):
      return None

  def close(self):
    if self.media_process is not None:
      self.media_process.terminate()
      try:
        self.media_process.wait(timeout=1)
      except subprocess.TimeoutExpired:
        self.media_process.kill()
        self.media_process.wait(timeout=1)
      self.media_socket.close()
    self.process.terminate()
    try:
      self.process.wait(timeout=2)
    except subprocess.TimeoutExpired:
      self.process.kill()
      self.process.wait(timeout=1)
    self.path.unlink(missing_ok=True)
    self.path.with_suffix('.tmp').unlink(missing_ok=True)


class SnapshotBuilder:
  def __init__(self, nav_map=False):
    from openpilot.cereal import messaging
    from openpilot.common.params import Params
    self.sm = messaging.SubMaster(list(SERVICES))
    self.params = Params()
    self.cached = {}
    self.settings = {}
    self.problems, self.dropped_params, self.report = {}, {}, {}
    self.next_params = self.next_send = 0.
    self.host_release = None
    release_file = Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/modeld/jetlink/host_release.json'
    from openpilot.common.jetson_model import selected_release
    self.host_release = selected_release(release_file)
    self.camera = None
    from hud_nav_map import NavPreviewPublisher
    self.nav_map = NavPreviewPublisher() if nav_map else None
    from openpilot.selfdrive.carrot.cluster.cluster_system_monitor import NetworkAddressProvider
    self.network = NetworkAddressProvider()

  def close(self):
    if self.camera is not None:
      self.camera.close()
    self.network.close()

  def camera_previews(self):
    from openpilot.common.jetlink_status import LINK_STATUS, _fresh, host_label
    now = time.monotonic()
    link = _fresh(LINK_STATUS, now)
    try:
      host_display = (host_label(link.get('peer')) == 'jetSON' and
                      0 <= now - float(link['telemetry_updated']) < 3 and
                      link['telemetry'].get('carrot_hud_connected') is True)
    except (KeyError, TypeError, ValueError):
      host_display = False
    # A host panel can start rendering without a camera preview. Only its
    # fresh output heartbeat starts this worker; no panel means no allocation.
    enabled = ((host_display or self.params.get_int('ClusterHud') == 1) and
               (self.params.get_bool('IsOnroad') or self.params.get_int('ClusterHudDebug') >= 1))
    if enabled and self.camera is None:
      from hud_camera import CameraPublisher
      self.camera = CameraPublisher()
    elif not enabled and self.camera is not None:
      self.camera.close()
      self.camera = None
    return self.camera.latest if self.camera is not None else {}

  def packet(self):
    from openpilot.common.jetlink_status import badge
    now = time.monotonic()
    if now < self.next_send:
      return None
    self.next_send = now + .1 if now - self.next_send > .2 else self.next_send + .1
    self.sm.update(0)
    for name in SERVICES:
      if self.sm.updated[name]:
        try:
          value = base64.b64encode(display_event(name, self.sm[name], self.sm.logMonoTime[name],
                                                 self.sm.valid[name])).decode()
        except Exception as exc:
          self.problems[name] = f'{type(exc).__name__}: {exc}'[:120]
          continue
        if len(value) > EVENT_LIMITS.get(name, EVENT_LIMIT):
          self.problems[name] = f'{len(value)} B > {EVENT_LIMITS.get(name, EVENT_LIMIT)} B'
          self.cached.pop(name, None)
          continue
        self.problems.pop(name, None)
        self.cached[name] = value
    if now >= self.next_params:
      self.settings, self.dropped_params = {}, {}
      for key in PARAMS:
        try:
          value = display_param(key, self.params.get(key))
        except Exception as exc:
          self.dropped_params[key] = f'{type(exc).__name__}: {exc}'[:120]
          continue
        if value is not None:
          if len(value) > PARAM_LIMITS.get(key, PARAM_LIMIT):
            self.dropped_params[key] = f'{len(value)} B > {PARAM_LIMITS.get(key, PARAM_LIMIT)} B'
          else:
            self.settings[key] = value
      self.next_params = now + 1
    device_badge = badge()
    # 'dropped' items were not sent this time; the Jetson keeps showing their last value.
    dropped = {**{f'events.{k}': v for k, v in self.problems.items()},
               **{f'params.{k}': v for k, v in self.dropped_params.items()}}
    record = {'version': 1, 'schema': SNAPSHOT_SCHEMA, 'sent': now, 'events': self.cached, 'params': self.settings,
              'dropped': dropped,
              **({'jetson_release': self.host_release} if self.host_release is not None else {}),
              'vehicle_network_address': self.network.address(),
              'external_compute_label': device_badge[0] if device_badge and device_badge[1] == 'active' else '',
              'received': self.sm.recv_time, 'mono': self.sm.logMonoTime,
              'valid': self.sm.valid, 'alive': self.sm.alive, 'cameras': self.camera_previews()}
    from hud_signal import signal_packet
    record['signal_display'] = signal_packet(now)
    record['jetlink_roundtrip_ms'] = jetlink_roundtrip_ms()   # K9HudNavdy 2, comma clock; None = comma's own model
    from openpilot.common.indicator_display import current
    record['indicator_display'] = current()
    if self.nav_map is not None:
      preview = self.nav_map.packet(self.settings.get('ClusterHudScreenMode') == 'Nw==')
      if preview is not None:
        record['nav_map_preview'] = preview
    data = encode_snapshot(record)
    self.report = snapshot_report(record, data)
    return data


def _pointer(field):
  return field.proto.which() == 'slot' and field.proto.slot.type.which() in (
    'text', 'data', 'list', 'struct', 'anyPointer', 'interface')


def copy_fields(src, dst, fields):
  """Copy the named fields of capnp struct src into builder dst (unset pointers stay unset; union: active only)."""
  schema = src.schema
  union = set(schema.union_fields)
  active = src.which() if union else None
  for name in fields:
    field = schema.fields.get(name)
    if field is None or (name in union and name != active):
      continue
    if field.proto.which() == 'group':
      group = getattr(src, name)
      copy_fields(group, dst.init(name) if name in union else getattr(dst, name), list(group.schema.fields))
    elif not _pointer(field) or src._has(name):
      setattr(dst, name, getattr(src, name))


def display_event(name, data, mono=0, valid=True):
  """Event bytes holding only SERVICE_FIELDS[name] of a service reader."""
  from openpilot.cereal import log
  event = log.Event.new_message(logMonoTime=mono, valid=bool(valid))
  copy_fields(data, event.init(name), SERVICE_FIELDS[name])
  return event.to_bytes()


def display_param(key, value):
  """base64 of the HUD's view of one Param, or None when unset. capnp Params keep only their PARAM_FIELDS."""
  if value is None:
    return None
  if isinstance(value, bool):
    value = b'1' if value else b'0'
  if key == 'LiveParameters':
    # JSON; the HUD reads steerRatio only (unparsable = the HUD has nothing to read, so nothing is sent)
    try:
      parsed = json.loads(value) if isinstance(value, (bytes, str)) else value
      value = json.dumps({k: parsed[k] for k in LIVE_PARAMETERS_JSON if k in parsed}).encode()
    except (TypeError, ValueError):
      return None
  value = value if isinstance(value, bytes) else str(value).encode()
  if key in PARAM_FIELDS:
    service, fields = PARAM_FIELDS[key]
    if service is None:
      from openpilot.cereal import car
      with car.CarParams.from_bytes(value) as reader:
        builder = car.CarParams.new_message()
        copy_fields(reader, builder, fields)
    else:
      from openpilot.cereal import log
      with log.Event.from_bytes(value) as reader:
        if reader.which() != service:
          raise ValueError(f'{key} is not {service}')
        builder = log.Event.new_message(logMonoTime=reader.logMonoTime, valid=reader.valid)
        copy_fields(getattr(reader, service), builder.init(service), fields)
    value = builder.to_bytes()
  return base64.b64encode(value).decode()


def without_torque_points(data):
  """Superseded by PARAM_FIELDS (kept for older callers): LiveTorqueParameters without the cached point cloud."""
  from openpilot.cereal import log
  try:
    with log.Event.from_bytes(data) as event:
      if event.which() != 'liveTorqueParameters' or len(event.liveTorqueParameters.points) == 0:
        return data
      fields = [f for f in event.liveTorqueParameters.schema.fields if f != 'points']
      builder = log.Event.new_message(logMonoTime=event.logMonoTime, valid=event.valid)
      copy_fields(event.liveTorqueParameters, builder.init('liveTorqueParameters'), fields)
    return builder.to_bytes()
  except Exception:
    return data


def snapshot_report(record, data):
  """Sizes for the comma-side status file: total, the largest items, what was dropped."""
  sizes = {f'events.{k}': len(v) for k, v in record.get('events', {}).items()}
  sizes.update({f'params.{k}': len(v) for k, v in record.get('params', {}).items()})
  sizes.update({k: len(json.dumps(record[k])) for k in ('cameras', 'nav_map_preview') if record.get(k)})
  largest = dict(sorted(sizes.items(), key=lambda kv: -kv[1])[:8])
  return {'bytes': len(data), 'limit': SNAPSHOT_LIMIT, 'largest': largest, 'dropped': record.get('dropped', {})}


def jetlink_roundtrip_ms():
  try:
    from openpilot.common.jetlink_status import MODEL_STATUS, _fresh
    status = _fresh(MODEL_STATUS)
    value = status.get('roundtrip_ms') if status.get('active') is True else None
    return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else None
  except Exception:
    return None


def encode_snapshot(record):
  indicator = record.get('indicator_display')
  if indicator and indicator.get('origin') == 'manual_turn':
    # Older HUD bundles know the driver-indicator origin and turning reason.
    record = dict(record, indicator_display=dict(indicator, origin='manual'))
  data = json.dumps(record, separators=(',', ':')).encode()
  if len(data) > SNAPSHOT_LIMIT:
    # Optional signal metadata must not evict an existing camera or NAV image.
    record.pop('indicator_display', None)
    record.pop('signal_display', None)
    data = json.dumps(record, separators=(',', ':')).encode()
  if len(data) > SNAPSHOT_LIMIT:
    record['cameras'] = {}
    data = json.dumps(record, separators=(',', ':')).encode()
  if len(data) > SNAPSHOT_LIMIT:
    record.pop('nav_map_preview', None)
    data = json.dumps(record, separators=(',', ':')).encode()
  if len(data) > SNAPSHOT_LIMIT:
    record['events'] = {k:v for k,v in record['events'].items() if k != 'navRoute'}
    data = json.dumps(record, separators=(',', ':')).encode()
  if len(data) > SNAPSHOT_LIMIT:
    raise ValueError('HUD snapshot exceeds bounded display allocation')
  return data


_cached_snapshot = None
_next_snapshot_read = 0.
# Jetson side: why the last read failed and how big the snapshot was (hud.py puts this in its status file).
SNAPSHOT_STATE = {'bytes': 0, 'error': 'no snapshot yet', 'received': None}


def read_snapshot():
  global _cached_snapshot, _next_snapshot_read
  now = time.monotonic()
  if now < _next_snapshot_read:
    return _cached_snapshot
  _next_snapshot_read = now + .02
  _cached_snapshot = None
  try:
    with HUD_PACKET.open('rb') as f:
      data = f.read(MAX_HUD_BYTES + HEADER.size + 1)
    SNAPSHOT_STATE['bytes'] = max(0, len(data) - HEADER.size)
    if len(data) > MAX_HUD_BYTES + HEADER.size:
      SNAPSHOT_STATE['error'] = f'oversized {len(data) - HEADER.size} B'
      return None
    received, = HEADER.unpack_from(data)
    if not 0 <= time.monotonic() - received < .5:
      SNAPSHOT_STATE['error'] = 'stale'
      return None
    value = json.loads(data[HEADER.size:])
    if value.get('version') == 1:
      _cached_snapshot = received, value
      SNAPSHOT_STATE.update(error=None, received=received)
      return _cached_snapshot
    SNAPSHOT_STATE['error'] = f"version {value.get('version')!r}"
  except (OSError, ValueError, struct.error, AttributeError) as exc:
    SNAPSHOT_STATE['error'] = f'{type(exc).__name__}: {exc}'[:120]
  return None
