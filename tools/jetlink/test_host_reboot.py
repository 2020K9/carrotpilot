"""Jetson reboot from comma Settings > Device (대감님 11/19 07:13, 08:29 "어떤 조건이든 내가 누르면 재부팅되게"):
no vehicle condition anywhere; one reboot command per Jetson server process; transfer onroad too."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import host_reboot as R

COMMAND = ['sudo', '-n', 'systemctl', '--no-block', 'reboot']


@pytest.fixture(autouse=True)
def fresh_jetson(monkeypatch):
  monkeypatch.setitem(R._state, 'rebooting', False)


def _runner(returncode=0):
  calls = []

  def run(cmd, **kw):
    calls.append(cmd)
    return SimpleNamespace(returncode=returncode)
  return run, calls


def test_no_vehicle_condition_code_left():
  assert not hasattr(R, 'check') and not hasattr(R, 'latest_snapshot') and not hasattr(R, 'MAX_AGE')
  source = Path(R.__file__).read_text().replace(R.__doc__, '')
  for word in ('gearShifter', 'vEgo', 'IsOnroad', 'selfdriveState', 'carControl'):
    assert word not in source


def test_host_request_runs_reboot_once():
  run, calls = _runner()
  req = json.dumps({'reboot': True, 'id': 'a'}).encode()
  assert R.host_request(req, run) == {'id': 'a', 'ok': True}
  assert calls == [COMMAND]
  # Duplicate (same or new id) while the Jetson is already rebooting: answered, no second command.
  assert R.host_request(req, run) == {'id': 'a', 'ok': True, 'duplicate': True}
  assert R.host_request(json.dumps({'reboot': True, 'id': 'b'}).encode(), run)['ok'] is True
  assert calls == [COMMAND]


def test_host_request_failed_command_can_retry():
  failing, calls = _runner(returncode=1)
  req = json.dumps({'reboot': True, 'id': 'a'}).encode()
  assert R.host_request(req, failing)['reason'] == 'reboot command failed'
  run, calls2 = _runner()
  assert R.host_request(req, run) == {'id': 'a', 'ok': True} and calls2 == [COMMAND]


def test_host_request_invalid_never_runs():
  run, calls = _runner()
  assert R.host_request(b'{"reboot": false, "id": "a"}', run)['ok'] is False
  assert R.host_request(b'x' * 300, run)['ok'] is False
  assert R.host_request(b'not json', run)['ok'] is False
  assert calls == []


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


def test_comma_sends_onroad_too(files):
  rid = R.submit(now=10.)
  client = FakeClient({'id': rid, 'ok': True})
  R.Client(PEER).idle(client, FakeParams(onroad=True), now=10.1)
  assert len(client.sent) == 1 and R.status(rid, 10., now=10.2) == ('sent', '')


def test_comma_reports_jetson_failure(files):
  rid = R.submit(now=10.)
  R.Client(PEER).idle(FakeClient({'id': rid, 'ok': False, 'reason': 'reboot command failed'}), FakeParams(), now=10.1)
  assert R.status(rid, 10., now=10.2) == ('rejected', 'reboot command failed')


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
  rid = R.submit(now=40.)   # not a Jetson peer
  R.Client({'carrot_host': 'pc', R.CAPABILITY: True}).idle(client, FakeParams(), now=40.1)
  assert client.sent == [] and R.status(rid, 40., now=40.2) == ('disconnected', '')


def test_comma_drops_old_request(files):
  R.submit(now=10.)
  client = FakeClient({'ok': True})
  R.Client(PEER).idle(client, FakeParams(), now=14.)
  assert client.sent == []


def _session(monkeypatch, run):
  import server
  sent = []
  session = server.CarrotSession.__new__(server.CarrotSession)
  session.last_seq = 0
  session._send_json = lambda kind, seq, obj: sent.append((kind, seq, obj))
  monkeypatch.setattr(server, 'reboot_request', lambda payload: R.host_request(payload, run))
  monkeypatch.setattr(server.Path, 'is_file', lambda self: True)
  return session, sent


def test_server_advertises_and_answers(monkeypatch):
  import server
  assert server.REBOOT_REQUEST == R.REQUEST and server.REBOOT_CAPABILITY == R.CAPABILITY
  run, calls = _runner()
  session, sent = _session(monkeypatch, run)
  session.handle(SimpleNamespace(msg_type=R.REQUEST, seq=3, payload=b'{"reboot": true, "id": "x"}'))
  assert sent == [(R.RESPONSE, 3, {'id': 'x', 'ok': True})] and calls == [COMMAND]
  session.handle(SimpleNamespace(msg_type=R.REQUEST, seq=3, payload=b'{"reboot": true, "id": "x"}'))   # replay ignored
  assert len(sent) == 1 and calls == [COMMAND]


def test_no_hud_signal_does_not_matter(monkeypatch, files):
  # No comma HUD snapshot at all on the Jetson (신호 없음): still reboots exactly once.
  run, calls = _runner()
  session, sent = _session(monkeypatch, run)
  rid = R.submit(now=10.)

  class Wire(FakeClient):
    def _expect(self, kind, seq, timeout):
      kind_, seq_, obj = self.sent[-1]
      session.handle(SimpleNamespace(msg_type=kind_, seq=seq_, payload=json.dumps(obj).encode()))
      return SimpleNamespace(payload=json.dumps(sent[-1][2]).encode())
  R.Client(PEER).idle(Wire(None), FakeParams(onroad=True), now=10.1)
  assert calls == [COMMAND] and R.status(rid, 10., now=10.2) == ('sent', '')


def test_daemon_forwards_between_onroad_frames():
  # jetlinkd calls host_reboot.idle in the offroad idle loop AND inside the onroad inference loop
  # (receive timeout + after each reply), so a request made while driving reaches the Jetson.
  daemon = Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/modeld/jetlink/daemon.py'
  tree = ast.parse(daemon.read_text())
  serve = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == '_serve_local')
  inner = [n for n in ast.walk(serve) if isinstance(n, ast.While)][1]
  calls = [n for n in ast.walk(inner) if isinstance(n, ast.Call) and ast.unparse(n.func) == 'host_reboot.idle']
  assert len(calls) == 2
  outer = [n for n in ast.walk(serve) if isinstance(n, ast.Call) and ast.unparse(n.func) == 'host_reboot.idle']
  assert len(outer) == 3
  infer_end = next(n for n in ast.walk(inner) if isinstance(n, ast.Call) and ast.unparse(n.func) == 'client.infer_end')
  assert max(c.lineno for c in calls) > infer_end.lineno   # never between infer_begin and infer_end
