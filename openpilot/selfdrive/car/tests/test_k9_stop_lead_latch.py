"""K9StopMode lead latch through the CarController (2026-10-22 17:06 K9 drive, rlog 000001ad): the kept kinds v9 and v10
(menu 1) latch StopReq on a lead at or within LEAD_LATCH_GAP (3.0 m) while still rolling.
Rewritten 2026-10-24 evening (보노보노 "브레이크 2.3.4 제거"): the old menu 2 (v9 + shudder fix, kind V9_SHUDDER_FIX, its
2.5 m lead latch LEAD_LATCH_GAP_2) and the old menu 3 (v10 + band 0.2, kind V10_BAND02) were removed with their code, so
their tests are deleted: test_mode2_withholds_stopreq_between_2_5_and_3_m_mode1_latches,
test_mode2_latches_at_or_below_2_5_m_like_v9, test_mode2_lead_zone_never_releases_never_weaker_than_v9,
test_mode2_lead_zone_then_standstill_latches, test_through_carcontroller_lead_2_9_m_slow_menu_3_is_menu_1,
test_through_carcontroller_menu_3_band_0_2_only_scc14, and the V9_SHUDDER_FIX / V10_SHUDDER_FIX cases of
test_through_carcontroller_lead_2_9_m (the V9 / V10 cases are unchanged); test_constants without LEAD_LATCH_GAP_2.
Checks source logic and SCC12 encoding, not K9 vehicle behaviour.
"""
import pytest

from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.k9_creep import V9, V10
from openpilot.selfdrive.car.tests.k9_rig import LCS, Rig


def test_constants():
  assert kc.LEAD_LATCH_GAP == 3.0 and not hasattr(kc, "LEAD_LATCH_GAP_2")


@pytest.mark.parametrize("mode,expect_sr_rolling", [(V9, True), (V10, True)])
def test_through_carcontroller_lead_2_9_m(monkeypatch, mode, expect_sr_rolling):
  # rewritten 2026-10-23 07:25: K9StopMode values moved, so the kind is mapped to menu 1 (test only); v10 keeps the
  # 3.0 m lead latch (safety addition of 1d0961da 2), v9 + shudder fix latches at 2.5 m; rewritten again in r2:
  # v10 + lead stop fix (menu 3) latches at 2.5 m too; rewritten again in r3: menu 3 also sends StopReq at once in the
  # 2.5-3.0 m zone when the predicted stop point is closer than 2.6 m - here 0.8 m/s at -0.3 m/s^2 would stop 1.1 m on,
  # so it sends StopReq while rolling like menu 1 (the slow case is test_through_carcontroller_lead_2_9_m_slow_menu_3)
  monkeypatch.setattr(carcontroller, "K9_CREEP_MENU", {1: mode})
  r = Rig(monkeypatch, K9StopMode=1)
  for i in range(100):
    r.step(-0.6, LCS.pid, 4.0 - 0.03 * i, a_target=-0.5, a_ego=-0.5, lead=(8.0 - 0.04 * i, -1.0))
  rolling_sr = []
  for i in range(150):
    v = max(0.8 - 0.005 * i, 0.06)
    _, sent = r.step(-0.7, LCS.stopping, v, a_target=-0.3, a_ego=-0.3, lead=(2.9, -v))
    if sent is not None:
      rolling_sr.append(sent["StopReq"])
  assert (max(rolling_sr) == 1) is expect_sr_rolling
  for i in range(60):
    _, sent = r.step(-0.7, LCS.stopping, 0.0, standstill=True, a_target=-0.3, a_ego=0.0, lead=(2.9, 0.0))
  assert sent is None or sent["StopReq"] == 1
