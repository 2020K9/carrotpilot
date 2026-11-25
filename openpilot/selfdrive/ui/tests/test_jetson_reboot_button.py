"""Settings > Device 'Jetson Reboot' (대감님 11/19 07:13, 08:29 "어떤 조건이든 내가 누르면 재부팅"):
placement, always enabled (onroad, D, moving, engaged, no signal), confirm -> request -> Jetson command once."""
import json
from types import SimpleNamespace

import pytest

from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import DialogResult
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.layouts.settings import jetson_reboot as JR
from tools.jetlink import host_reboot as R  # noqa: TID251

COMMAND = ['sudo', '-n', 'systemctl', '--no-block', 'reboot']


@pytest.fixture(scope='module', autouse=True)
def window():
  from openpilot.system.ui.lib.multilang import multilang
  previous = multilang.language
  multilang.change_language("en")   # English texts below; Korean is checked from app_ko.po
  gui_app.init_window("jetson reboot test")
  yield
  gui_app.close()
  multilang.change_language(previous)


@pytest.fixture
def files(tmp_path, monkeypatch):
  monkeypatch.setattr(R, 'REQUEST_FILE', tmp_path / 'req.json')
  monkeypatch.setattr(R, 'RESULT_FILE', tmp_path / 'res.json')
  monkeypatch.setitem(R._state, 'rebooting', False)
  monkeypatch.setattr(ui_state, 'started', False)
  ui_state.params.put_bool("IsOnroad", False)
  pushed = []
  monkeypatch.setattr(gui_app, 'push_widget', pushed.append)
  yield pushed
  ui_state.params.put_bool("IsOnroad", False)


def _title(item):
  return item.title if isinstance(item.title, str) else item.title()


# Vehicle states the old version refused. The comma side reads none of them now; each is set where the UI can see it.
CONDITIONS = {
  'onroad': {'started': True, 'IsOnroad': True},
  'drive_gear': {'started': True, 'IsOnroad': True, 'gear': 'drive'},
  'moving': {'started': True, 'IsOnroad': True, 'gear': 'drive', 'vEgo': 27.},
  'engaged': {'started': True, 'IsOnroad': True, 'gear': 'drive', 'vEgo': 27., 'engaged': True},
  'no_signal': {'started': True, 'IsOnroad': True, 'no_signal': True},
}


def _apply(monkeypatch, cond):
  monkeypatch.setattr(ui_state, 'started', cond['started'])
  ui_state.params.put_bool("IsOnroad", cond['IsOnroad'])
  monkeypatch.setattr(type(ui_state), 'engaged', property(lambda self: cond.get('engaged', False)))
  # Gear / speed / missing car signals: nothing on the comma or Jetson path reads them any more
  # (test_no_vehicle_condition_code_left); the Jetson side here has no HUD snapshot at all.


def _jetson_server(monkeypatch):
  """The Jetson tools/jetlink/server.py by file path: a bare `import server` picks whichever top-level
  `server` is first on sys.path (openpilot/selfdrive/carrot/server when the carrot tests ran first)."""
  import importlib.util
  import sys
  from pathlib import Path
  jetlink = Path(__file__).resolve().parents[4] / "tools/jetlink"
  monkeypatch.syspath_prepend(str(jetlink))   # its own sibling imports (host_reboot, jetlink, ...)
  name = "k9_test_jetson_server"
  if name not in sys.modules:
    spec = importlib.util.spec_from_file_location(name, jetlink / "server.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
  return sys.modules[name]


class _Wire:
  """jetlinkd Client <-> Jetson server.py session in one process; reboot command recorded, never run."""
  def __init__(self, monkeypatch):
    server = _jetson_server(monkeypatch)
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


PEER = {'carrot_host': 'jetson', R.CAPABILITY: True}


class _OnroadParams:
  def get_bool(self, key):
    return {'IsOnroad': True, 'IsOffroad': False}[key]


def test_big_ui_button_sits_above_reboot_power_off(files):
  from openpilot.selfdrive.ui.layouts.settings.device import DeviceLayout
  layout = DeviceLayout()
  items = layout._initialize_items()
  index = items.index(layout._jetson_reboot_btn)
  assert items[index + 1] is layout._power_off_btn and index == len(items) - 2
  assert _title(layout._jetson_reboot_btn) == "Jetson Reboot"


@pytest.mark.parametrize('name', list(CONDITIONS))
def test_big_ui_any_state_reaches_reboot_command_once(files, monkeypatch, name):
  from openpilot.selfdrive.ui.layouts.settings.device import DeviceLayout
  from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
  _apply(monkeypatch, CONDITIONS[name])
  layout = DeviceLayout()
  assert layout._jetson_reboot_btn.action_item.enabled is True
  layout._jetson_reboot_prompt()
  dialog = files[-1]
  assert isinstance(dialog, ConfirmDialog)            # confirm stays
  dialog._callback(DialogResult.CANCEL)
  assert not R.REQUEST_FILE.exists()
  dialog._callback(DialogResult.CONFIRM)
  assert json.loads(R.REQUEST_FILE.read_text())['id'] == layout._jetson_reboot.request_id
  assert _title(layout._jetson_reboot_btn) == "Jetson Reboot - sending..."
  wire = _Wire(monkeypatch)
  R.Client(PEER).idle(wire, _OnroadParams())
  assert wire.calls == [COMMAND]
  assert _title(layout._jetson_reboot_btn) == "Jetson Reboot - Jetson rebooting"


def test_reboot_and_power_off_conditions_unchanged(files, monkeypatch):
  from openpilot.selfdrive.ui.layouts.settings.device import DeviceLayout
  _apply(monkeypatch, CONDITIONS['engaged'])
  layout = DeviceLayout()
  layout._reboot_prompt()
  layout._power_off_prompt()
  # Comma reboot / power off keep their engaged gate: alerts only, no confirm callback.
  assert len(files) == 2 and all(w._callback is None for w in files)
  assert [w._label._text for w in files] == ["Disengage to Reboot", "Disengage to Power Off"]


def test_duplicate_requests_one_command(files, monkeypatch):
  from openpilot.selfdrive.ui.layouts.settings.device import DeviceLayout
  _apply(monkeypatch, CONDITIONS['moving'])
  layout = DeviceLayout()
  wire = _Wire(monkeypatch)
  for _ in range(3):
    layout._jetson_reboot_prompt()
    files[-1]._callback(DialogResult.CONFIRM)
    R.Client(PEER).idle(wire, _OnroadParams())
    R.Client(PEER).idle(wire, _OnroadParams())          # same request file read once
  assert wire.calls == [COMMAND] and len(wire.replies) == 3
  assert _title(layout._jetson_reboot_btn) == "Jetson Reboot - Jetson rebooting"


def test_not_connected_zero_commands(files, monkeypatch):
  from openpilot.selfdrive.ui.layouts.settings.device import DeviceLayout
  _apply(monkeypatch, CONDITIONS['onroad'])
  layout = DeviceLayout()
  wire = _Wire(monkeypatch)
  layout._jetson_reboot_prompt()
  files[-1]._callback(DialogResult.CONFIRM)
  R.Client({}).idle(wire, _OnroadParams())              # no Jetson peer
  assert wire.calls == []
  assert _title(layout._jetson_reboot_btn) == "Jetson Reboot - Jetson not connected"
  layout._jetson_reboot_prompt()                         # no jetlinkd at all: times out
  files[-1]._callback(DialogResult.CONFIRM)
  assert layout._jetson_reboot.text(layout._jetson_reboot.requested_at + 5.) == "Jetson not connected"
  assert wire.calls == []


def test_result_texts(files):
  state = JR.JetsonReboot()
  state.request(ui_state.params)
  t = state.requested_at
  assert state.text(t + .1) == "sending..."
  assert state.text(t + 5.) == "Jetson not connected"
  assert state.text(t + 5. + JR.SHOW_SECONDS + 1) == ""
  for result, text in ((('sent', ''), "Jetson rebooting"),
                       (('unsupported', 'old jetson bundle'), "Jetson needs the new bundle (next boot)"),
                       (('disconnected', ''), "Jetson not connected")):
    state.request(ui_state.params)
    R._write(R.RESULT_FILE, {'id': state.request_id, 'state': result[0], 'reason': result[1]})
    assert state.text() == text
  ui_state.params.put_bool("IsOnroad", True)           # onroad no longer refused on the comma
  state.request(ui_state.params)
  assert state.text() == "sending..." and R.REQUEST_FILE.exists()


def test_korean_strings():
  from openpilot.system.ui.lib.multilang import TRANSLATIONS_DIR, load_translations
  translations, _ = load_translations(TRANSLATIONS_DIR.joinpath('app_ko.po'))
  assert translations["Jetson Reboot"] == "젯슨 재부팅"
  assert translations["sending..."] == "보내는 중..."
  assert translations["Jetson rebooting"] == "젯슨 재부팅 중"
  assert translations["Jetson not connected"] == "젯슨 미연결"
  assert translations["Jetson needs the new bundle (next boot)"] == "젯슨 새 판 필요"
  for msgid in list(JR.REASONS.values()) + ["rejected: {}", "REBOOT", "Are you sure you want to reboot the Jetson?",
                                            "Reboot the Jetson that runs the driving model. Works in any state, including while driving."]:
    assert translations.get(msgid), msgid
  for gone in ("onroad", "not in P", "vehicle moving", "driver assistance active", "Go offroad to reboot the Jetson",
               "Vehicle stop state could not be confirmed, so the Jetson was not rebooted"):
    assert gone not in translations


@pytest.mark.parametrize('name', list(CONDITIONS))
def test_mici_any_state_reaches_reboot_command_once(files, monkeypatch, name):
  from openpilot.selfdrive.ui.mici.layouts.settings.device import DeviceLayoutMici, JetsonRebootBigButton
  from openpilot.selfdrive.ui.mici.widgets.dialog import BigConfirmationDialog
  _apply(monkeypatch, CONDITIONS[name])
  layout = DeviceLayoutMici()
  widgets = layout._scroller._items
  index = next(i for i, w in enumerate(widgets) if isinstance(w, JetsonRebootBigButton))
  button = widgets[index]
  assert widgets[-1] is layout._power_off_btn and index == len(widgets) - 3
  assert button.enabled is True
  button._click_callback()
  dialog = files[-1]
  assert isinstance(dialog, BigConfirmationDialog)     # slide confirm stays, no "Disengage to" dialog
  assert not R.REQUEST_FILE.exists()
  dialog._confirm_callback()
  assert json.loads(R.REQUEST_FILE.read_text())['id'] == button.jetson_reboot.request_id
  button._update_state()
  assert button.value == "sending..."
  wire = _Wire(monkeypatch)
  R.Client(PEER).idle(wire, _OnroadParams())
  assert wire.calls == [COMMAND]
  button._update_state()
  assert button.value == "Jetson rebooting"
