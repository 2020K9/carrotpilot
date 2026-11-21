"""Jetson reboot while the Jetson is still loading (대감님 11/21 15:21 "젯슨 재부팅 안된다").

Before: only the ready loop (_serve_local) forwarded the request. A Jetson stuck in resident-model wait,
App model selection or the boot-release check never got it and the button answered "Jetson not connected".
Now each of those wait loops forwards it between polls (no transaction in flight)."""
import json
from types import SimpleNamespace

import pytest

from jetlink.transport.base import LinkError
import host_reboot as R
from openpilot.selfdrive.modeld.jetlink import mac, startup
from openpilot.selfdrive.modeld.jetlink import daemon

PEER = {'carrot_host': 'jetson', R.CAPABILITY: True}


@pytest.fixture(autouse=True)
def files(tmp_path, monkeypatch):
  monkeypatch.setattr(R, 'REQUEST_FILE', tmp_path / 'req.json')
  monkeypatch.setattr(R, 'RESULT_FILE', tmp_path / 'res.json')
  monkeypatch.setitem(R._state, 'rebooting', False)


class Wire:
  """Comma client whose REQUEST goes straight into the Jetson server handler (mock reboot command)."""
  def __init__(self, states, monkeypatch):
    import server
    self.states, self.seq, self.calls, self.sent = list(states), 0, [], []
    session = server.CarrotSession.__new__(server.CarrotSession)
    session.last_seq = 0
    replies = []
    session._send_json = lambda kind, seq, obj: replies.append(obj)
    monkeypatch.setattr(server, 'reboot_request',
                        lambda payload: R.host_request(payload, lambda cmd, **kw: self.calls.append(cmd) or SimpleNamespace(returncode=0)))
    monkeypatch.setattr(server.Path, 'is_file', lambda self: True)

    def send_json(kind, seq, obj):
      self.sent.append(kind)
      if kind == R.REQUEST:
        session.handle(SimpleNamespace(msg_type=kind, seq=seq, payload=json.dumps(obj).encode()))
    self.t = SimpleNamespace(send_json=send_json)
    self._expect = lambda kind, seq, timeout: SimpleNamespace(payload=json.dumps(replies[-1]).encode())

  def _next_seq(self):
    self.seq += 1
    return self.seq

  def state(self):
    value = self.states.pop(0) if len(self.states) > 1 else self.states[0]
    if value == 'press':
      R.submit()
      value = {'loaded': None}
    return value


def _idle(client):
  forwarder = R.Client(PEER)
  return lambda: forwarder.idle(client, None)


def test_resident_wait_onroad_forwards_reboot(monkeypatch):
  monkeypatch.setattr(mac.time, 'sleep', lambda s: None)
  client = Wire(['press', {'loaded': None, 'engine_state': 'failed', 'detail': 'stuck'}], monkeypatch)
  with pytest.raises(LinkError):
    mac.prepare_jetson(client, {'loaded': None}, lambda: False, lambda: True, lambda *a: None, idle=_idle(client))
  assert client.calls == [list(R.REBOOT_COMMAND)]


def test_resident_wait_without_idle_unchanged(monkeypatch):
  monkeypatch.setattr(mac.time, 'sleep', lambda s: None)
  client = Wire(['press', {'loaded': None, 'engine_state': 'failed', 'detail': 'stuck'}], monkeypatch)
  with pytest.raises(LinkError):
    mac.prepare_jetson(client, {'loaded': None}, lambda: False, lambda: True, lambda *a: None)
  assert client.calls == [] and R.REQUEST_FILE.exists()


def test_app_selection_wait_forwards_reboot(monkeypatch):
  monkeypatch.setattr(daemon.time, 'sleep', lambda s: None)
  monkeypatch.setattr(daemon, 'may_provision', lambda peer, mode: True)
  monkeypatch.setattr(daemon, 'host_attached', lambda: True)
  R.submit()
  client = Wire([{'loaded': 'a' * 64}], monkeypatch)
  peer = daemon.wait_for_selection(client, dict(PEER, protocol=3, loaded=None), lambda *a: None, idle=_idle(client))
  assert peer['loaded'] == 'a' * 64 and client.calls == [list(R.REBOOT_COMMAND)]


def test_boot_release_wait_forwards_reboot(monkeypatch):
  monkeypatch.setattr(startup, 'selected_release', lambda *a, **k: {'delivery': 'nas'})
  states = [{'carrot_update': {'state': 'checking'}}, {'carrot_update': {'state': 'ready'}}]
  client = Wire([{}], monkeypatch)
  client.state = lambda: states.pop(0)
  R.submit()
  with pytest.raises(startup.RuntimeHandoff):
    startup.wait_for_boot_update(client, {'carrot_host': 'jetson', startup.CAPABILITY: True}, None,
                                 lambda: True, lambda *a, **k: None, sleep=lambda s: None, idle=_idle(client))
  assert client.calls == [list(R.REBOOT_COMMAND)]


def test_one_command_for_repeated_presses_while_loading(monkeypatch):
  monkeypatch.setattr(mac.time, 'sleep', lambda s: None)
  client = Wire(['press', 'press', 'press', {'loaded': None, 'engine_state': 'failed', 'detail': 'stuck'}], monkeypatch)
  with pytest.raises(LinkError):
    mac.prepare_jetson(client, {'loaded': None}, lambda: False, lambda: True, lambda *a: None, idle=_idle(client))
  assert client.calls == [list(R.REBOOT_COMMAND)]
