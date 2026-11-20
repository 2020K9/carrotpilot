"""Carrot web Settings > Device 'Jetson Reboot' (대감님 11/20 09:10): web endpoint -> host_reboot request file
-> jetlinkd Client -> Jetson server.py -> mock reboot command. Any vehicle state (engaged included)."""
import asyncio
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from openpilot.selfdrive.carrot.server.features import system
from tools.jetlink import host_reboot as R  # noqa: TID251

COMMAND = ['sudo', '-n', 'systemctl', '--no-block', 'reboot']
PEER = {'carrot_host': 'jetson', R.CAPABILITY: True}
WEB = Path(__file__).parents[2] / 'web'
JETLINK = Path(R.__file__).parent


def _jetson_server():
  # tools/jetlink/server.py; the name "server" is the carrot web package here, so load it by path.
  spec = importlib.util.spec_from_file_location('jetson_host_server_test', JETLINK / 'server.py')
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


class _Params:
  def get_bool(self, key):
    return {'IsOnroad': True, 'IsOffroad': False}[key]


class _Wire:
  """jetlinkd Client <-> Jetson server.py session in one process; reboot command recorded, never run."""
  def __init__(self, monkeypatch):
    server = _jetson_server()
    self.calls, self.replies, self.seq = [], [], 0
    self.session = server.CarrotSession.__new__(server.CarrotSession)
    self.session.last_seq = 0
    self.session._send_json = lambda kind, seq, obj: self.replies.append((kind, seq, obj))
    run = lambda cmd, **kw: self.calls.append(cmd) or SimpleNamespace(returncode=0)  # noqa: E731
    monkeypatch.setattr(server, 'reboot_request', lambda payload: R.host_request(payload, run))
    monkeypatch.setattr(server.Path, 'is_file', lambda self: True)
    self.t = SimpleNamespace(send_json=self._send)

  def _next_seq(self):
    self.seq += 1
    return self.seq

  def _send(self, kind, seq, obj):
    self.session.handle(SimpleNamespace(msg_type=kind, seq=seq, payload=json.dumps(obj).encode()))

  def _expect(self, kind, seq, timeout):
    reply = next(r for r in self.replies if r[:2] == (kind, seq))
    return SimpleNamespace(payload=json.dumps(reply[2]).encode())


@pytest.fixture
def files(tmp_path, monkeypatch):
  monkeypatch.setattr(R, 'REQUEST_FILE', tmp_path / 'req.json')
  monkeypatch.setattr(R, 'RESULT_FILE', tmp_path / 'res.json')
  monkeypatch.setitem(R._state, 'rebooting', False)
  # Driving and engaged: the comma reboot endpoint refuses, the Jetson one must not.
  monkeypatch.setattr(system, 'is_drive_engaged', lambda request: True)


def _run(steps):
  async def main():
    app = web.Application()
    system.register(app)
    client = TestClient(TestServer(app))
    await client.start_server()
    try:
      return await steps(client)
    finally:
      await client.close()
  return asyncio.run(main())


def test_web_endpoint_reaches_reboot_command_once(files, monkeypatch):
  wire = _Wire(monkeypatch)

  async def steps(client):
    r = await (await client.post('/api/jetson_reboot', json={})).json()
    assert r['ok'] and r['state'] == 'pending'
    assert json.loads(R.REQUEST_FILE.read_text())['id'] == r['id']
    s = await (await client.get('/api/jetson_reboot_status', params={'id': r['id']})).json()
    assert s['state'] == 'pending'
    R.Client(PEER).idle(wire, _Params())
    s = await (await client.get('/api/jetson_reboot_status', params={'id': r['id']})).json()
    assert (s['state'], s['reason']) == ('sent', '')
    blocked = await client.post('/api/reboot', json={})
    assert blocked.status == 409                         # comma reboot gate unchanged
  _run(steps)
  assert wire.calls == [COMMAND]


def test_web_not_connected_zero_commands(files, monkeypatch):
  wire = _Wire(monkeypatch)

  async def steps(client):
    r = await (await client.post('/api/jetson_reboot', json={})).json()
    R.Client({}).idle(wire, _Params())                   # no Jetson peer
    s = await (await client.get('/api/jetson_reboot_status', params={'id': r['id']})).json()
    assert s['state'] == 'disconnected'
    r = await (await client.post('/api/jetson_reboot', json={})).json()   # no jetlinkd at all: times out
    state = client.server.app[system.JETSON_REBOOT]
    state['request'] = (state['request'][0], state['request'][1] - 5.)   # 5 s later, nothing answered
    s = await (await client.get('/api/jetson_reboot_status', params={'id': r['id']})).json()
    assert s['state'] == 'disconnected'
  _run(steps)
  assert wire.calls == []


def test_web_duplicate_requests_one_command(files, monkeypatch):
  wire = _Wire(monkeypatch)

  async def steps(client):
    states = []
    for _ in range(3):
      r = await (await client.post('/api/jetson_reboot', json={})).json()
      R.Client(PEER).idle(wire, _Params())
      R.Client(PEER).idle(wire, _Params())               # same request file read once
      s = await (await client.get('/api/jetson_reboot_status', params={'id': r['id']})).json()
      states.append(s['state'])
    assert states == ['sent'] * 3
  _run(steps)
  assert wire.calls == [COMMAND] and len(wire.replies) == 3


def test_web_unsupported_and_failed_results(files):
  async def steps(client):
    out = []
    for state, reason in (('unsupported', 'old jetson bundle'), ('rejected', 'reboot command failed')):
      r = await (await client.post('/api/jetson_reboot', json={})).json()
      R._write(R.RESULT_FILE, {'id': r['id'], 'state': state, 'reason': reason})
      s = await (await client.get('/api/jetson_reboot_status', params={'id': r['id']})).json()
      out.append((s['state'], s['reason']))
    assert out == [('unsupported', 'old jetson bundle'), ('rejected', 'reboot command failed')]
    assert (await client.get('/api/jetson_reboot_status', params={'id': 'other'})).status == 404
  _run(steps)


def test_web_button_next_to_reboot_and_texts():
  device = (WEB / 'js/pages/setting_device.js').read_text()
  reboot = device.index('"btnDeviceReboot")')
  jetson = device.index('"btnDeviceJetsonReboot")')
  assert reboot < jetson < device.index('"btnDeviceRecalib")')
  actions = (WEB / 'js/pages/setting_device_actions.js').read_text()
  assert '"/api/jetson_reboot"' in actions and 'appConfirm(getUIText("jetson_reboot_confirm"' in actions
  ko = (WEB / 'js/translations/ko.js').read_text()
  for text in ('"젯슨 재부팅"', '"보내는 중..."', '"젯슨 재부팅 중"', '"젯슨 미연결"', '"젯슨 새 판 필요"', '"명령 실패"'):
    assert text in ko, text
