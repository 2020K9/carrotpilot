"""Clean Jetson reboot once after comma git pull / comma model change / Jetson model change (대감님 11/26 19:57)."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import host_reboot as R

PEER = {'carrot_host': 'jetson', R.CAPABILITY: True}
BASE = {'commit': 'aaa', 'comma_model': 'Cinque', 'jetson_model': 'n16'}


class Wire:
  def __init__(self):
    self.sent = []
    self.t = SimpleNamespace(send_json=lambda kind, seq, obj: self.sent.append(obj))

  def _next_seq(self):
    return 3

  def _expect(self, kind, seq, timeout):
    return SimpleNamespace(payload=json.dumps({'ok': True}).encode())


@pytest.fixture
def state(tmp_path, monkeypatch):
  monkeypatch.setattr(R, 'BOOT_STATE_FILE', tmp_path / 'boot.json')
  monkeypatch.setattr(R, 'RESULT_FILE', tmp_path / 'res.json')
  R.BOOT_STATE_FILE.write_text(json.dumps(BASE))
  return tmp_path


def _boot(fingerprint):
  wire = Wire()
  result = R.Client(PEER).boot_reset(wire, fingerprint)
  return wire.sent, result


@pytest.mark.parametrize('key,value', [('commit', 'bbb'), ('comma_model', 'Other'), ('jetson_model', 'ls2')])
def test_three_changes_reboot_once(state, key, value):
  new = dict(BASE, **{key: value})
  sent, result = _boot(new)
  assert len(sent) == 1 and sent[0]['reboot'] is True and result['state'] == 'sent'
  assert json.loads(R.BOOT_STATE_FILE.read_text()) == new
  assert _boot(new)[0] == []   # next comma boot / jetlinkd restart: no second reboot


def test_ordinary_start_does_nothing(state):
  for _ in range(3):
    assert _boot(dict(BASE)) == ([], None)


def test_one_check_per_client_and_jetson_only(state):
  client, wire = R.Client(PEER), Wire()
  client.boot_reset(wire, dict(BASE, commit='c1'))
  client.boot_reset(wire, dict(BASE, commit='c2'))
  assert len(wire.sent) == 1
  wire = Wire()
  assert R.Client({'carrot_host': 'pc'}).boot_reset(wire, dict(BASE, commit='c3')) is None and wire.sent == []


def test_old_bundle_marks_done_without_message(state):
  wire = Wire()
  result = R.Client({'carrot_host': 'jetson'}).boot_reset(wire, dict(BASE, commit='c4'))
  assert result['state'] == 'unsupported' and wire.sent == []
  assert _boot(dict(BASE, commit='c4'))[0] == []


def test_fallback_timer_is_20s_and_same_prepare_code(tmp_path):
  calls = []

  def run(cmd, **kw):
    calls.append(list(cmd))
    return SimpleNamespace(returncode=0)
  R.prepare_and_reboot(run, tmp_path, sleep=lambda s: None, log=lambda m: None)
  assert R.FALLBACK_SECONDS == 20 and calls[0][:2] == ['systemd-run', '--on-active=20']
  # button and boot reset both go through Client._send -> same REQUEST -> host_request -> PREPARE_COMMAND
  tree = ast.parse(Path(R.__file__).read_text())
  client = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Client')
  users = {f.name for f in client.body if isinstance(f, ast.FunctionDef)
           and any(isinstance(c, ast.Call) and ast.unparse(c.func) == 'self._send' for c in ast.walk(f))}
  assert users == {'idle', 'boot_reset'}


def test_daemon_calls_boot_reset_offroad_loop():
  daemon = Path(R.__file__).resolve().parents[2] / 'openpilot/selfdrive/modeld/jetlink/daemon.py'
  assert daemon.read_text().count('host_reboot.boot_reset(client)') == 1
