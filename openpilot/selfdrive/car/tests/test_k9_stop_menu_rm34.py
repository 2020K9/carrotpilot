"""K9StopMode 3 / 4 removed (2026-11-06 06:21, 대감님 "브레이크3.4.제거"). a082dc0d had 3 / 4 = 1 / 2 + StoppingAccel -1.00
while picked; the menu is 0 / 1 / 2 again and a marker-11 3 -> 1, 4 -> 2 (MERGE_V12, idempotent, no new marker).
Model-only harness (실차 미검증)."""
import json

from opendbc.car.hyundai import k9_creep as kc
from openpilot.selfdrive.car import k9_stop_mode_migration as mig
from openpilot.selfdrive.car.tests.k9_rig import ROOT
from openpilot.selfdrive.car.tests.test_k9_stop_v12 import FakeParams
from openpilot.selfdrive.controls.lib import longcontrol


def test_menu_is_0_1_2_again():
  assert kc.MENU == {1: 15, 2: 17} and kc.MODES == (1, 2)
  assert not hasattr(longcontrol, "K9_STOP_MODE_STOPPING_ACCEL")
  item = next(i for i in json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))["params"]
              if i["name"] == "K9StopMode")
  assert item["max"] == 2 and item["choices"] == [0, 1, 2]
  assert all(len(item["options"][lang]) == 3 for lang in ("ko", "en", "zh"))


def test_marker11_3_4_fold_into_1_2_once():
  for old, want in ((3, 1), (4, 2)):
    p = FakeParams(K9StopMode=old, K9StopModeVer=11)
    assert mig.migrate_k9_stop_mode(p) is True and p.v == {"K9StopMode": want, "K9StopModeVer": 11}
    assert mig.migrate_k9_stop_mode(p) is False and p.v == {"K9StopMode": want, "K9StopModeVer": 11}


def test_marker11_other_values_untouched():
  for v in (0, 1, 2, 5, 6, 9, -1, True, None, "x"):
    p = FakeParams(K9StopMode=v, K9StopModeVer=11)
    assert mig.migrate_k9_stop_mode(p) is False and p.v == {"K9StopMode": v, "K9StopModeVer": 11}


def test_marker10_never_gets_1_2_from_3_4_rule():  # old numbering: 3 -> 1 (kind 15) and 4 -> 0 by MOVE_V11, as before
  for old, want in ((3, 1), (4, 0), (5, 2)):
    p = FakeParams(K9StopMode=old, K9StopModeVer=10)
    mig.migrate_k9_stop_mode(p)
    mig.migrate_k9_stop_mode(p)
    assert p.v == {"K9StopMode": want, "K9StopModeVer": 11}


def test_write_failure_retries_next_start():
  class Fail(FakeParams):
    def put_int(self, k, x):
      raise OSError
  p = Fail(K9StopMode=3, K9StopModeVer=11)
  assert mig.migrate_k9_stop_mode(p) is False and p.v["K9StopMode"] == 3
  q = FakeParams(**p.v)
  assert mig.migrate_k9_stop_mode(q) is True and q.v["K9StopMode"] == 1


def test_settings_texts_without_3_4():
  item = next(i for i in json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))["params"]
              if i["name"] == "K9StopMode")
  for k in ("descr", "edescr", "cdescr"):
    assert "-100" not in item[k] and "0~4" not in item[k] and "0-4" not in item[k] and "0至4" not in item[k]


def test_older_markers_never_end_on_3_4():
  for marker in (None, 8, 9, 10):
    for stored in range(6):
      p = FakeParams(K9StopMode=stored) if marker is None else FakeParams(K9StopMode=stored, K9StopModeVer=marker)
      mig.migrate_k9_stop_mode(p)
      assert p.v["K9StopModeVer"] == 11 and p.v["K9StopMode"] in (0, 1, 2), (marker, stored, p.v)


class _LP:
  def __init__(self, sa, mode):
    self.v = {"StoppingAccel": sa, "K9StopMode": mode}

  def get_float(self, k):
    return self.v[k]

  def get_int(self, k):
    return self.v[k]


def test_stopping_accel_is_the_setting_for_every_mode():  # the -1.00 override of 3 / 4 is gone
  for mode in (0, 1, 2, 3, 4):
    for sa in (-50, -75, -100):
      c = object.__new__(longcontrol.LongControl)
      c.params = _LP(sa, mode)
      c._refresh_stopping_accel()
      assert abs(c.stopping_accel - sa * 0.01) < 1e-9, (mode, sa)
