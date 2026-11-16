"""Snapshot schema 2: the comma sends the Jetson HUD only the fields it draws, and each item has its own ceiling.

98d17e01 forwarded LiveTorqueParameters whole; torqued's cached point cloud (up to 12000 points, 512 KB base64)
pushed every snapshot past 96 KB and the Jetson HUD got nothing. These tests fail when
- the HUD code starts naming a field that is not sent (the allow list fell behind),
- an item grows past its ceiling (11/13 rlog copies in fixtures/hud_payload_20261113.json),
- the ceilings together could no longer fit the 96 KB snapshot,
- one oversized or broken item takes down anything but itself (comma and Jetson side).
"""
import base64
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
import hud_protocol as P
from openpilot.cereal import car, log

FIXTURE = Path(__file__).resolve().parent / 'fixtures/hud_payload_20261113.json'
# Fields the HUD code names (as a word) that are deliberately not sent: same word, other meaning/struct.
NOT_SENT = {
  'liveDelay': {'points'},              # liveTracks.points; the lag point cloud is never drawn
  'liveTorqueParameters': {'points'},   # torqued's fit cache (the 512 KB that broke the HUD)
  'roadCameraState': {'image'}, 'wideRoadCameraState': {'image'},   # 'image' = navi media kind
}


def hud_words():
  cluster = ROOT / 'openpilot/selfdrive/carrot'
  files = [f for f in (cluster / 'cluster').glob('*.py') if not f.name.startswith(('make_', 'test_'))]
  files += list(cluster.glob('ext_hud*.py')) + list(cluster.glob('ext_hud/*.py'))
  files += [cluster / 'deceleration_source.py', ROOT / 'openpilot/selfdrive/controls/lib/cutin_alert.py',
            ROOT / 'openpilot/common/indicator_display.py']
  files += [f for f in Path(__file__).resolve().parent.glob('hud*.py') if f.name != 'hud_protocol.py']
  words = set()
  for f in files:
    if f.is_file():
      words |= set(re.findall(r'[A-Za-z_]\w*', f.read_text()))
  return words


def test_every_service_has_an_allow_list_of_real_fields():
  assert set(P.SERVICE_FIELDS) == set(P.SERVICES)
  for name, fields in P.SERVICE_FIELDS.items():
    schema = log.Event.schema.fields[name].schema
    assert set(fields) <= set(schema.fields), (name, set(fields) - set(schema.fields))
    assert len(set(fields)) == len(fields), name
  for key, (service, fields) in P.PARAM_FIELDS.items():
    assert key in P.PARAMS
    schema = car.CarParams.schema if service is None else log.Event.schema.fields[service].schema
    assert set(fields) <= set(schema.fields)


def test_allow_list_covers_every_field_the_hud_names():
  words = hud_words()
  missing = {}
  for name, fields in P.SERVICE_FIELDS.items():
    schema = log.Event.schema.fields[name].schema
    named = {f for f in schema.fields if f in words and f != 'deprecated'}
    lost = named - set(fields) - NOT_SENT.get(name, set())
    if lost:
      missing[name] = sorted(lost)
  assert not missing, f'HUD code reads fields the Jetson never receives: {missing}'


def test_logged_items_fit_their_ceilings_and_the_snapshot():
  events = json.loads(FIXTURE.read_text())
  assert set(events) <= set(P.SERVICES) and len(events) >= 20
  record = {'version': 1, 'schema': P.SNAPSHOT_SCHEMA, 'sent': 1., 'events': {}, 'params': {}, 'cameras': {}}
  for name, item in events.items():
    with log.Event.from_bytes(base64.b64decode(item)) as event:
      again = base64.b64encode(P.display_event(name, getattr(event, name), event.logMonoTime, event.valid)).decode()
    assert again == item, f'{name}: allow list changed the logged copy'
    limit = P.EVENT_LIMITS.get(name, P.EVENT_LIMIT)
    assert len(item) <= limit * .85, f'{name} {len(item)} B is within 15 % of its {limit} B ceiling'
    record['events'][name] = item
  record['params']['LiveTorqueParameters'] = P.display_param('LiveTorqueParameters', torque(12000))
  data = P.encode_snapshot(record)
  assert len(data) <= 40 * 1024   # 11/13 measured 25 KB of events; leaves room for cameras and NAV preview
  assert json.loads(data)['events'].keys() == events.keys()


def test_ceilings_together_fit_96kb_without_navroute():
  # navRoute follows the route length and keeps its own last-resort drop in encode_snapshot.
  services = sum(P.EVENT_LIMITS.get(n, P.EVENT_LIMIT) for n in P.SERVICES if n != 'navRoute')
  params = sum(P.PARAM_LIMITS.get(k, P.PARAM_LIMIT) for k in P.PARAMS)
  per_service_tables = len(P.SERVICES) * 160   # valid/alive/mono/received/key text per service
  assert services + params + per_service_tables <= P.SNAPSHOT_LIMIT - 8 * 1024
  assert P.SNAPSHOT_LIMIT == 96 * 1024


def torque(points):
  event = log.Event.new_message(logMonoTime=7, valid=True)
  t = event.init('liveTorqueParameters')
  t.latAccelFactorFiltered, t.frictionCoefficientFiltered, t.latAccelFactorRaw = 2.75, .125, 3.1
  t.frictionCoefficientRaw, t.calPerc, t.liveValid, t.totalBucketPoints, t.version = .09, 61, True, float(points), 1
  t.points = [[.01 * (i % 90) - .45, .5, .1] for i in range(points)]
  return event.to_bytes()


def test_torque_param_is_the_hud_numbers_only():
  full = torque(12000)
  assert len(base64.b64encode(full)) > 500 * 1024
  shown = P.display_param('LiveTorqueParameters', full)
  assert len(shown) <= P.PARAM_LIMITS['LiveTorqueParameters']
  with log.Event.from_bytes(base64.b64decode(shown)) as e, log.Event.from_bytes(full) as o:
    for f in P.SERVICE_FIELDS['liveTorqueParameters']:
      assert getattr(e.liveTorqueParameters, f) == getattr(o.liveTorqueParameters, f), f
    assert (e.logMonoTime, e.valid) == (7, True)
    assert len(e.liveTorqueParameters.points) == 0 and e.liveTorqueParameters.totalBucketPoints == 0


def test_carparams_and_json_params_are_reduced():
  cp = car.CarParams.new_message(brand='hyundai', carFingerprint='KIA_K9', wheelbase=3.1, steerRatio=15.2,
                                 maxLateralAccel=2.4, flags=5)
  cp.carFw = [{'ecu': 'eps', 'fwVersion': b'x' * 200}] * 8
  shown = P.display_param('CarParams', cp.to_bytes())
  assert len(shown) <= P.PARAM_LIMITS['CarParams']
  with car.CarParams.from_bytes(base64.b64decode(shown)) as r:
    assert (r.brand, r.carFingerprint, r.flags, len(r.carFw)) == ('hyundai', 'KIA_K9', 5, 0)
    assert abs(r.maxLateralAccel - 2.4) < 1e-6
  live = {'steerRatio': 15.5, 'stiffnessFactor': 1., 'angleOffsetAverageDeg': .1, 'carFingerprint': 'KIA_K9'}
  assert json.loads(base64.b64decode(P.display_param('LiveParameters', live))) == {'steerRatio': 15.5}
  assert P.display_param('LiveParameters', b'not json') is None
  assert P.display_param('IsOnroad', True) == base64.b64encode(b'1').decode()
  assert P.display_param('ClusterHudBrightness', 42) == base64.b64encode(b'42').decode()
  assert P.display_param('K9HudNavdy', None) is None


class Params:
  def __init__(self, values):
    self.values = values

  def get(self, key):
    return self.values.get(key)


class Sub:
  def __init__(self, data):
    self.data = data
    self.updated = {n: n in data for n in P.SERVICES}
    self.valid = dict.fromkeys(P.SERVICES, True)
    self.logMonoTime = dict.fromkeys(P.SERVICES, 5)
    self.alive = dict.fromkeys(P.SERVICES, True)
    self.recv_time = dict.fromkeys(P.SERVICES, 1.)

  def update(self, timeout):
    pass

  def __getitem__(self, name):
    return self.data[name]


def builder(data, params):
  b = object.__new__(P.SnapshotBuilder)
  b.sm, b.params = Sub(data), Params(params)
  b.cached, b.settings, b.problems, b.dropped_params, b.report = {}, {}, {}, {}, {}
  b.next_params = b.next_send = 0.
  b.host_release, b.camera, b.nav_map = None, None, None
  b.network = type('N', (), {'address': lambda self: None})()
  b.camera_previews = dict
  return b


def test_oversized_item_is_dropped_alone_on_the_comma(monkeypatch):
  import hud_signal
  from openpilot.common import indicator_display, jetlink_status
  monkeypatch.setattr(hud_signal, 'signal_packet', lambda now: None)
  monkeypatch.setattr(indicator_display, 'current', lambda: None)
  monkeypatch.setattr(jetlink_status, 'badge', lambda: None)
  model_event = log.Event.new_message(modelV2={})
  model = model_event.modelV2
  model.laneLines = [{'x': [float(i)] * 33, 'y': [0.] * 33, 'z': [0.] * 33, 't': [0.] * 33} for i in range(60)]
  car_state = log.Event.new_message(carState={'vEgo': 12.5}).carState
  b = builder({'modelV2': model, 'carState': car_state}, {'IsOnroad': True, 'LiveTorqueParameters': torque(12000)})
  record = json.loads(b.packet())
  assert record['schema'] == P.SNAPSHOT_SCHEMA
  assert 'modelV2' not in record['events'] and 'events.modelV2' in record['dropped']
  with log.Event.from_bytes(base64.b64decode(record['events']['carState'])) as e:
    assert e.carState.vEgo == 12.5
  assert base64.b64decode(record['params']['IsOnroad']) == b'1'
  assert 'LiveTorqueParameters' in record['params']
  assert b.report['bytes'] == len(json.dumps(record, separators=(',', ':')))
  assert 'events.modelV2' in b.report['dropped']


def jetson_record(stamp, events, params, dropped=None):
  names = list(events)
  return (stamp, {'version': 1, 'schema': P.SNAPSHOT_SCHEMA, 'sent': stamp, 'events': events, 'params': params,
                  'dropped': dropped or {}, 'valid': dict.fromkeys(names, True), 'alive': dict.fromkeys(names, True),
                  'received': dict.fromkeys(names, stamp), 'mono': {n: int(stamp * 1e9) for n in names}})


def b64event(**kw):
  return base64.b64encode(log.Event.new_message(**kw).to_bytes()).decode()


def test_jetson_keeps_last_value_of_a_bad_item_and_shows_the_rest(monkeypatch):
  import hud
  feed = []
  monkeypatch.setattr(hud, 'read_snapshot', lambda: feed[-1])
  monkeypatch.setattr(hud, 'REJECTED', {})
  sm = hud.RemoteSubMaster(['carState', 'deviceState', 'liveTorqueParameters'])
  feed.append(jetson_record(10., {'carState': b64event(carState={'vEgo': 3.}),
                                  'deviceState': b64event(deviceState={'started': True})}, {}))
  sm.update(0)
  assert sm['carState'].vEgo == 3. and sm['deviceState'].started
  fat = 'A' * (P.EVENT_LIMITS.get('liveTorqueParameters', P.EVENT_LIMIT) + 4)
  feed.append(jetson_record(10.1, {'carState': b64event(carState={'vEgo': 4.}),
                                   'deviceState': b64event(carState={'vEgo': 9.}),   # wrong identity
                                   'liveTorqueParameters': fat}, {}))
  sm.update(0)
  assert sm['carState'].vEgo == 4. and sm.updated['carState']
  assert sm['deviceState'].started and not sm.updated['deviceState']   # last good value kept
  assert not sm.updated['liveTorqueParameters']
  assert 'identity' in hud.REJECTED['events.deviceState'] and 'B >' in hud.REJECTED['events.liveTorqueParameters']
  feed.append((10.2, {'version': 1, 'sent': 'x'}))   # malformed snapshot: one lost update, no exception
  sm.update(0)
  assert not sm.valid['carState'] and sm['carState'].vEgo == 4. and 'snapshot' in hud.REJECTED
  feed.append(jetson_record(10.3, {'deviceState': b64event(deviceState={'started': False})}, {}))
  sm.update(0)
  assert not sm['deviceState'].started and 'events.deviceState' not in hud.REJECTED and 'snapshot' not in hud.REJECTED


def test_jetson_params_keep_value_when_one_is_dropped_or_oversized(monkeypatch):
  import hud
  def enc(v):
    return base64.b64encode(v).decode()
  feed = [jetson_record(10., {}, {'IsOnroad': enc(b'1'), 'K9HudNavdy': enc(b'2'), 'CustomSR': enc(b'0')})]
  monkeypatch.setattr(hud, 'read_snapshot', lambda: feed[-1])
  monkeypatch.setattr(hud, 'REJECTED', {})
  params = hud.DisplayParams()
  assert params.get_bool('IsOnroad') and params.get_int('K9HudNavdy') == 2
  feed.append(jetson_record(10.1, {}, {'IsOnroad': 'B' * (P.PARAM_LIMIT + 1)}, {'params.K9HudNavdy': 'ValueError'}))
  params.next_read = 0.
  assert params.get_bool('IsOnroad') and params.get_int('K9HudNavdy') == 2
  assert params.get('CustomSR') is None   # absent and not dropped = unset on the comma, as before
  assert set(hud.REJECTED) >= {'params.IsOnroad', 'params.K9HudNavdy'}
  feed.append((10.2, {'version': 1}))   # no params table at all
  params.next_read = 0.
  assert params.get_bool('IsOnroad')


def test_connected_follows_snapshots_not_just_frames(tmp_path, monkeypatch):
  import hud
  monkeypatch.setattr(hud, 'REJECTED', {})
  state = hud.SNAPSHOT_STATE
  monkeypatch.setitem(state, 'received', 9.6)
  monkeypatch.setitem(state, 'schema', P.SNAPSHOT_SCHEMA)
  monkeypatch.setitem(state, 'error', None)
  assert hud.snapshot_status(10.)['ok']
  assert not hud.snapshot_status(11.)['ok']            # frames keep going, snapshots stopped 1.4 s ago
  monkeypatch.setitem(state, 'schema', None)
  assert not hud.snapshot_status(10.)['ok']            # an old comma that still sends whole services
  import server
  status = tmp_path / 'status.json'
  real = Path
  monkeypatch.setattr(server, 'Path', lambda p: status if p == '/dev/shm/carrot-jetlink-hud-status.json' else real(p))
  monkeypatch.setattr(server.time, 'monotonic', lambda: 20.)
  telemetry = object.__new__(server.DisplayTelemetry)
  telemetry.original = type('O', (), {'read': lambda self: {}})()
  telemetry.blinker = telemetry.health = None
  for snapshot, expected in (({'ok': True}, True), ({'ok': False, 'error': 'stale'}, False), (None, False)):
    status.write_text(json.dumps({'updated': 19.5, **({'snapshot': snapshot} if snapshot else {})}))
    assert telemetry.read()['carrot_hud_connected'] is expected


def test_read_snapshot_records_why_it_failed(tmp_path, monkeypatch):
  path = tmp_path / 'hud'
  monkeypatch.setattr(P, 'HUD_PACKET', path)
  monkeypatch.setattr(P.time, 'monotonic', lambda: 10.)
  for data, reason in ((P.HEADER.pack(9.) + b'{}', 'stale'), (P.HEADER.pack(9.9) + b'[1]', 'AttributeError'),
                       (P.HEADER.pack(9.9) + b'{"version":3}', 'version 3'), (b'x' * (P.MAX_HUD_BYTES + 20), 'oversized')):
    path.write_bytes(data)
    monkeypatch.setattr(P, '_next_snapshot_read', 0.)
    assert P.read_snapshot() is None and reason in P.SNAPSHOT_STATE['error']
  path.write_bytes(P.HEADER.pack(9.9) + b'{"version":1}')
  monkeypatch.setattr(P, '_next_snapshot_read', 0.)
  assert P.read_snapshot() is not None and P.SNAPSHOT_STATE['error'] is None and P.SNAPSHOT_STATE['bytes'] == 13


def test_comma_status_file(tmp_path):
  import hud_publisher
  path = tmp_path / 'snapshot.json'
  hud_publisher._next_status[0] = 0.
  hud_publisher.write_status(5., {'bytes': 100, 'dropped': {}}, path)
  hud_publisher.write_status(5.5, {'bytes': 200}, path)                 # once a second
  assert json.loads(path.read_text())['bytes'] == 100
  hud_publisher.write_status(5.6, {'error': 'ValueError: big'}, path)   # failures immediately
  assert json.loads(path.read_text())['error'] == 'ValueError: big'
