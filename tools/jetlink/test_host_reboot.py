"""Jetson reboot from comma Settings > Device (대감님 11/19 07:13): transfer and Jetson refusal rules."""
import base64
import json
from types import SimpleNamespace

import pytest

import host_reboot as R
from hud_protocol import HEADER


def _ev(name, **fields):
  from openpilot.cereal import log
  return base64.b64encode(log.Event.new_message(**{name: fields}).to_bytes()).decode()


def snapshot(gear='park', v=0., enabled=False, active=False, cc_enabled=False, lat=False, onroad=b'0',
             ages=None, alive=None, with_car=True, vehicle=None):
  ages = {'carState': .01, 'selfdriveState': .01, 'carControl': .01, **(ages or {})}
  names = ('carState', 'selfdriveState', 'carControl') if with_car else ()
  events = {}
  if with_car:
    events = {'carState': _ev('carState', gearShifter=gear, vEgo=v),
              'selfdriveState': _ev('selfdriveState', enabled=enabled, active=active),
              'carControl': _ev('carControl', enabled=cc_enabled, latActive=lat)}
  sent = 100.
  snap = {'version': 1, 'sent': sent, 'events': events,
          'params': {} if onroad is None else {'IsOnroad': base64.b64encode(onroad).decode()},
          'alive': dict.fromkeys(names, True) | (alive or {}),
          'received': {n: sent - ages[n] for n in names}}
  if vehicle is not None:
    snap['vehicle_state'] = vehicle
  return snap


def test_parked_stopped_disengaged_is_allowed():
  assert R.check(snapshot(), .05) is None


@pytest.mark.parametrize('snap_age', [None, .5, .9, -1.])
def test_old_or_missing_snapshot_rejected(snap_age):
  assert R.check(snapshot(), snap_age) == 'no vehicle signal'
  assert R.check(None, .01) == 'no vehicle signal'


def test_not_park_rejected():
  for gear in ('drive', 'reverse', 'neutral', 'unknown'):
    assert R.check(snapshot(gear=gear), .01) == 'not in P'


def test_moving_rejected():
  assert R.check(snapshot(v=.3), .01) == 'moving'
  assert R.check(snapshot(v=.001), .01) == 'moving'


@pytest.mark.parametrize('kw', [{"enabled": True}, {"active": True}, {"cc_enabled": True}, {"lat": True}])
def test_driver_assistance_rejected(kw):
  assert R.check(snapshot(**kw), .01) == 'driver assistance active'


def test_onroad_or_unknown_onroad_rejected():
  assert R.check(snapshot(onroad=b'1'), .01) == 'onroad'
  assert R.check(snapshot(onroad=None), .01) == 'no vehicle signal'


@pytest.mark.parametrize('name', ['carState', 'selfdriveState', 'carControl'])
def test_stale_signal_rejected(name):
  # carState stale: card running but old; selfdriveState/carControl stale with fresh carState.
  assert R.check(snapshot(ages={name: .5}), .01) == 'no vehicle signal'
  assert R.check(snapshot(ages={name: 2.}), .01) == 'no vehicle signal'


def test_not_alive_signal_rejected():
  assert R.check(snapshot(alive={'selfdriveState': False}), .01) == 'no vehicle signal'


def test_ignition_off_without_gear_speed_rejected():
  # 노비 11/19 07:50: the old 'card not running + fresh panda ignition off' allowance is gone.
  for vehicle in ({'ignition': False, 'age': .01}, {'ignition': False, 'age': .1}, None):
    assert R.check(snapshot(with_car=False, vehicle=vehicle), .01) == 'no vehicle signal'
  assert R.check(snapshot(with_car=False, vehicle={'ignition': False, 'age': .1}, onroad=b'1'), .01) == 'onroad'


@pytest.mark.parametrize('name', ['carState', 'selfdriveState', 'carControl'])
@pytest.mark.parametrize('how', ['missing', 'not_alive', 'old', 'no_event'])
def test_each_condition_missing_or_old_rejected(name, how):
  snap = snapshot(vehicle={'ignition': False, 'age': .01})   # a fresh ignition-off must not rescue anything
  if how == 'missing':
    del snap['received'][name], snap['alive'][name], snap['events'][name]
  elif how == 'not_alive':
    snap['alive'][name] = False
  elif how == 'old':
    snap['received'][name] = snap['sent'] - .5
  else:
    del snap['events'][name]
  assert R.check(snap, .01) == 'no vehicle signal'


def _packet(tmp_path, snap, age=0.):
  import time
  p = tmp_path / 'hud.packet'
  p.write_bytes(HEADER.pack(time.monotonic() - age) + json.dumps(snap).encode())
  return p


def test_host_request_runs_reboot_only_when_allowed(tmp_path):
  calls = []

  def run(cmd, **kw):
    calls.append(cmd)
    return SimpleNamespace(returncode=0)
  req = json.dumps({'reboot': True, 'id': 'a'}).encode()
  assert R.host_request(req, _packet(tmp_path, snapshot(gear='drive')), run) == {'id': 'a', 'ok': False, 'reason': 'not in P'}
  assert R.host_request(req, _packet(tmp_path, snapshot(), age=.6), run)['reason'] == 'no vehicle signal'
  assert R.host_request(req, tmp_path / 'missing', run)['reason'] == 'no vehicle signal'
  assert calls == []
  assert R.host_request(req, _packet(tmp_path, snapshot()), run) == {'id': 'a', 'ok': True}
  assert calls == [['sudo', '-n', 'systemctl', '--no-block', 'reboot']]

  def failing(cmd, **kw):
    return SimpleNamespace(returncode=1)
  assert R.host_request(req, _packet(tmp_path, snapshot()), failing)['reason'] == 'reboot command failed'
  assert R.host_request(b'{"reboot": false, "id": "a"}', _packet(tmp_path, snapshot()), run)['ok'] is False
  assert R.host_request(b'x' * 300, _packet(tmp_path, snapshot()), run)['ok'] is False
  assert len(calls) == 1


class FakeParams:
  def __init__(self, onroad=False):
    self.onroad = onroad

  def get_bool(self, key):
    return {'IsOnroad': self.onroad, 'IsOffroad': not self.onroad}[key]


class FakeClient:
  def __init__(self, reply):
    self.reply, self.sent = reply, []
    self.t = SimpleNamespace(send_json=lambda kind, seq, obj: self.sent.append((kind, seq, obj)))

  def _next_seq(self):
    return 7

  def _expect(self, kind, seq, timeout):
    assert (kind, seq) == (R.RESPONSE, 7)
    if isinstance(self.reply, Exception):
      raise self.reply
    return SimpleNamespace(payload=json.dumps(self.reply).encode())


@pytest.fixture
def files(tmp_path, monkeypatch):
  monkeypatch.setattr(R, 'REQUEST_FILE', tmp_path / 'req.json')
  monkeypatch.setattr(R, 'RESULT_FILE', tmp_path / 'res.json')
  return tmp_path


PEER = {'carrot_host': 'jetson', R.CAPABILITY: True}


def test_comma_sends_request_and_reports_sent(files):
  rid = R.submit(now=10.)
  client = FakeClient({'id': rid, 'ok': True})
  R.Client(PEER).idle(client, FakeParams(), now=10.2)
  assert client.sent == [(R.REQUEST, 7, {'reboot': True, 'id': rid})]
  assert R.status(rid, 10., now=10.3) == ('sent', '')
  assert not R.REQUEST_FILE.exists()
  R.Client(PEER).idle(client, FakeParams(), now=10.4)   # one request, one message
  assert len(client.sent) == 1


def test_comma_reports_jetson_rejection(files):
  rid = R.submit(now=10.)
  R.Client(PEER).idle(FakeClient({'id': rid, 'ok': False, 'reason': 'not in P'}), FakeParams(), now=10.1)
  assert R.status(rid, 10., now=10.2) == ('rejected', 'not in P')


def test_comma_never_sends_onroad(files):
  rid = R.submit(now=10.)
  client = FakeClient({'ok': True})
  R.Client(PEER).idle(client, FakeParams(onroad=True), now=10.1)
  assert client.sent == [] and R.status(rid, 10., now=10.2) == ('rejected', 'onroad')


def test_comma_old_bundle_and_disconnected(files):
  rid = R.submit(now=10.)
  client = FakeClient({'ok': True})
  R.Client({'carrot_host': 'jetson'}).idle(client, FakeParams(), now=10.1)
  assert client.sent == [] and R.status(rid, 10., now=10.2) == ('unsupported', 'old jetson bundle')
  rid = R.submit(now=20.)
  R.Client(PEER).idle(FakeClient(TimeoutError()), FakeParams(), now=20.1)
  assert R.status(rid, 20., now=20.2) == ('disconnected', '')
  rid = R.submit(now=30.)   # no jetlinkd session picks it up
  assert R.status(rid, 30., now=31.) == ('pending', '')
  assert R.status(rid, 30., now=34.) == ('disconnected', '')


def test_comma_drops_old_request(files):
  R.submit(now=10.)
  client = FakeClient({'ok': True})
  R.Client(PEER).idle(client, FakeParams(), now=14.)
  assert client.sent == []


def test_server_advertises_and_answers(tmp_path, monkeypatch):
  import server
  assert server.REBOOT_REQUEST == R.REQUEST and server.REBOOT_CAPABILITY == R.CAPABILITY
  sent = []
  session = server.CarrotSession.__new__(server.CarrotSession)
  session.last_seq = 0
  session._send_json = lambda kind, seq, obj: sent.append((kind, seq, obj))
  monkeypatch.setattr(server, 'reboot_request', lambda payload, packet: {'id': 'x', 'ok': False, 'reason': 'not in P'})
  monkeypatch.setattr(server.Path, 'is_file', lambda self: True)
  session.handle(SimpleNamespace(msg_type=R.REQUEST, seq=3, payload=b'{}'))
  assert sent == [(R.RESPONSE, 3, {'id': 'x', 'ok': False, 'reason': 'not in P'})]
  session.handle(SimpleNamespace(msg_type=R.REQUEST, seq=3, payload=b'{}'))   # replay ignored
  assert len(sent) == 1


def test_ignition_off_no_gear_speed_rejected_end_to_end(tmp_path, files):
  # Jetson refuses, never runs the reboot command, comma records the reason the UI shows as the fixed sentence.
  calls = []
  snap = snapshot(with_car=False, vehicle={'ignition': False, 'age': .01})
  packet = _packet(tmp_path, snap)
  rid = R.submit(now=10.)

  class Wire(FakeClient):
    def _expect(self, kind, seq, timeout):
      reply = R.host_request(json.dumps(self.sent[-1][2]).encode(), packet, lambda cmd, **kw: calls.append(cmd))
      return SimpleNamespace(payload=json.dumps(reply).encode())
  R.Client(PEER).idle(Wire(None), FakeParams(), now=10.1)
  assert calls == []
  assert R.status(rid, 10., now=10.2) == ('rejected', 'no vehicle signal')
