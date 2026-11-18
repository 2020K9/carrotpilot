"""Settings > Device 'Jetson Reboot' (대감님 11/19 07:13): placement, offroad-only, confirm -> request."""
import json

import pytest

from openpilot.system.ui.lib.application import gui_app
from openpilot.system.ui.widgets import DialogResult
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.selfdrive.ui.layouts.settings import jetson_reboot as JR
from tools.jetlink import host_reboot as R  # noqa: TID251


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
  monkeypatch.setattr(ui_state, 'started', False)
  ui_state.params.put_bool("IsOnroad", False)
  pushed = []
  monkeypatch.setattr(gui_app, 'push_widget', pushed.append)
  return pushed


def _title(item):
  return item.title if isinstance(item.title, str) else item.title()


def test_big_ui_button_sits_above_reboot_power_off(files):
  from openpilot.selfdrive.ui.layouts.settings.device import DeviceLayout
  layout = DeviceLayout()
  items = layout._initialize_items()
  index = items.index(layout._jetson_reboot_btn)
  assert items[index + 1] is layout._power_off_btn and index == len(items) - 2
  assert _title(layout._jetson_reboot_btn) == "Jetson Reboot"


def test_big_ui_enabled_only_offroad(files, monkeypatch):
  from openpilot.selfdrive.ui.layouts.settings.device import DeviceLayout
  layout = DeviceLayout()
  action = layout._jetson_reboot_btn.action_item
  assert action.enabled is True
  monkeypatch.setattr(ui_state, 'started', True)
  assert action.enabled is False
  layout._jetson_reboot_prompt()    # onroad: alert only, no confirm, no request
  assert len(files) == 1 and not R.REQUEST_FILE.exists()


def test_big_ui_confirm_writes_request_and_shows_result(files, monkeypatch):
  from openpilot.selfdrive.ui.layouts.settings.device import DeviceLayout
  from openpilot.system.ui.widgets.confirm_dialog import ConfirmDialog
  layout = DeviceLayout()
  layout._jetson_reboot_prompt()
  dialog = files[-1]
  assert isinstance(dialog, ConfirmDialog)
  dialog._callback(DialogResult.CANCEL)
  assert not R.REQUEST_FILE.exists()
  dialog._callback(DialogResult.CONFIRM)
  request = json.loads(R.REQUEST_FILE.read_text())
  assert request['id'] == layout._jetson_reboot.request_id
  assert _title(layout._jetson_reboot_btn) == "Jetson Reboot - sending..."
  R._write(R.RESULT_FILE, {'id': request['id'], 'state': 'rejected', 'reason': 'not in P'})
  assert _title(layout._jetson_reboot_btn) == "Jetson Reboot - rejected: not in P"


def test_big_ui_cancelled_when_onroad_during_dialog(files, monkeypatch):
  from openpilot.selfdrive.ui.layouts.settings.device import DeviceLayout
  layout = DeviceLayout()
  layout._jetson_reboot_prompt()
  monkeypatch.setattr(ui_state, 'started', True)
  files[-1]._callback(DialogResult.CONFIRM)
  assert not R.REQUEST_FILE.exists()


def test_result_texts(files):
  state = JR.JetsonReboot()
  state.request(ui_state.params)
  t = state.requested_at
  assert state.text(t + .1) == "sending..."
  assert state.text(t + 5.) == "Jetson not connected"
  assert state.text(t + 5. + JR.SHOW_SECONDS + 1) == ""
  for result, text in ((('sent', ''), "sent, Jetson rebooting"),
                       (('unsupported', 'old jetson bundle'), "Jetson needs the new bundle (next boot)"),
                       (('rejected', 'no vehicle signal'), "Vehicle stop state could not be confirmed, so the Jetson was not rebooted")):
    state.request(ui_state.params)
    R._write(R.RESULT_FILE, {'id': state.request_id, 'state': result[0], 'reason': result[1]})
    assert state.text() == text
  R.REQUEST_FILE.unlink()   # no jetlinkd in this test picks requests up
  ui_state.params.put_bool("IsOnroad", True)
  state.request(ui_state.params)
  assert state.text() == "rejected: onroad" and not R.REQUEST_FILE.exists()
  ui_state.params.put_bool("IsOnroad", False)


def test_korean_strings():
  from openpilot.system.ui.lib.multilang import TRANSLATIONS_DIR, load_translations
  translations, _ = load_translations(TRANSLATIONS_DIR.joinpath('app_ko.po'))
  assert translations["Jetson Reboot"] == "젯슨 재부팅"
  assert translations["Jetson not connected"] == "젯슨 미연결"
  assert translations["Vehicle stop state could not be confirmed, so the Jetson was not rebooted"] == "차량 정차 상태를 확인할 수 없어 재부팅하지 않았습니다"
  for msgid in list(JR.REASONS.values()) + ["sending...", "sent, Jetson rebooting", "rejected: {}", "REBOOT",
                                            "Are you sure you want to reboot the Jetson?", "Go offroad to reboot the Jetson",
                                            "Reboot the Jetson that runs the driving model. Only offroad; " +
                                            "the Jetson also refuses unless the car is in P, stopped and driver assistance is off."]:
    assert translations.get(msgid), msgid


def test_mici_button_before_reboot_offroad_only(files, monkeypatch):
  from openpilot.selfdrive.ui.mici.layouts.settings.device import DeviceLayoutMici, JetsonRebootBigButton
  layout = DeviceLayoutMici()
  widgets = layout._scroller._items
  index = next(i for i, w in enumerate(widgets) if isinstance(w, JetsonRebootBigButton))
  button = widgets[index]
  assert widgets[-1] is layout._power_off_btn and index == len(widgets) - 3
  assert button.enabled is True
  monkeypatch.setattr(ui_state, 'started', True)
  assert button.enabled is False
  button._request()
  assert not R.REQUEST_FILE.exists()
  monkeypatch.setattr(ui_state, 'started', False)
  button._request()
  assert json.loads(R.REQUEST_FILE.read_text())['id'] == button.jetson_reboot.request_id
  button._update_state()
  assert button.value == "sending..."


def test_unconfirmed_stop_shows_exact_korean_sentence(files):
  from openpilot.system.ui.lib.multilang import multilang
  state = JR.JetsonReboot()
  multilang.change_language("ko")
  try:
    state.request(ui_state.params)
    R._write(R.RESULT_FILE, {'id': state.request_id, 'state': 'rejected', 'reason': 'no vehicle signal'})
    assert state.text() == "차량 정차 상태를 확인할 수 없어 재부팅하지 않았습니다"
  finally:
    multilang.change_language("en")
    R.REQUEST_FILE.unlink(missing_ok=True)
