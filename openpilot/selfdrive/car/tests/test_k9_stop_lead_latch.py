"""K9StopMode 2 lead latch (2026-10-22 17:06 K9 drive, rlog 000001ad).

The K9 planner stops 2.2-3.0 m behind a lead, so the 3.0 m lead latch sent StopReq in 3 of 4 mode 2 stops while the
car still rolled (0.23 / 0.78 / 0.23 m/s) and the K9 ESC braked hard on it (LONG_ACCEL jerk 8.2 / 10.6 / 8.6 m/s^3).
Mode 2 latches on a lead only at or below LEAD_LATCH_GAP_2 (2.5 m); between 2.5 and 3.0 m StopReq stays withheld while
the sent value is at most the v9 latch value and never releases. Mode 1 (v9) keeps the 3.0 m latch.
New tests only; the existing K9 tests are unchanged. Checks source logic and SCC12 encoding, not K9 vehicle behaviour.
"""
import random

import numpy as np
import pytest

from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V9, V9_SHUDDER_FIX
from openpilot.selfdrive.car.tests.k9_rig import LCS, Rig


def upd(c, **kw):
  args = dict(accel=-0.7, a_target=-0.3, stopping=True, pid=False, v_ego=0.3, v_ego_raw=0.3, a_ego=-0.3, long_accel=-0.3,
              esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True,
              lead_visible=True, lead_distance=2.9)
  args.update(kw)
  return c.update(**args)


def test_constants():
  assert kc.LEAD_LATCH_GAP == 3.0 and kc.LEAD_LATCH_GAP_2 == 2.5


@pytest.mark.parametrize("gap", [2.51, 2.6, 2.8, 2.9, 3.0])
def test_mode2_withholds_stopreq_between_2_5_and_3_m_mode1_latches(gap):
  c1, c2 = K9Creep(V9), K9Creep(V9_SHUDDER_FIX)
  for _ in range(20):
    a1, s1 = upd(c1, lead_distance=gap)
    a2, s2 = upd(c2, lead_distance=gap)
    assert s1 is True and s2 is False and not c2.latched
    assert a2 <= a1 + 1e-12  # never weaker than the v9 latch value
    assert a2 <= min(-0.7, kc.LATCH_ACCEL) + 1e-12


@pytest.mark.parametrize("gap", [0.0, 1.0, 2.0, 2.49, 2.5])
@pytest.mark.parametrize("visible", [True, False])
def test_mode2_latches_at_or_below_2_5_m_like_v9(gap, visible):
  if gap == 0.0 and not visible:
    pytest.skip("no lead (distance 0, not visible)")
  c1, c2 = K9Creep(V9), K9Creep(V9_SHUDDER_FIX)
  a1, s1 = upd(c1, lead_distance=gap, lead_visible=visible)
  a2, s2 = upd(c2, lead_distance=gap, lead_visible=visible)
  assert s1 is True and s2 is True and a2 <= a1 + 1e-12


@pytest.mark.parametrize("seed", range(6))
def test_mode2_lead_zone_never_releases_never_weaker_than_v9(seed):
  """Approach like the 17:13:54 / 17:20:15 stops: stopping from 0.8 m/s, lead 3.7 -> 2.6 m, noisy feedback."""
  rng = random.Random(f"k9-lead-1706-{seed}")
  c1, c2 = K9Creep(V9), K9Creep(V9_SHUDDER_FIX)
  prev, in_zone = None, 0
  n = 300
  for i in range(n):
    v = max(0.8 - 0.75 * i / (n - 1), 0.06)
    gap = 3.7 - 1.1 * i / (n - 1)
    ae = rng.uniform(-0.5, 0.05)
    kw = dict(accel=rng.uniform(-0.9, -0.5), a_target=rng.uniform(-0.6, -0.05), v_ego=v, v_ego_raw=v, a_ego=ae,
              long_accel=min(rng.uniform(-0.6, 0.03), ae + 0.1), lead_distance=gap)
    a1, _ = upd(c1, **kw)
    a2, s2 = upd(c2, **kw)
    assert a2 <= a1 + 1e-12  # never weaker than v9 on any frame
    if gap > kc.LEAD_LATCH_GAP_2 and kw["a_ego"] <= 0.06:
      assert s2 is False  # rolling, lead 2.5-3.0 m: no StopReq
    if kc.LEAD_LATCH_GAP_2 < gap <= kc.LEAD_LATCH_GAP and prev is not None and not s2:
      assert a2 <= prev + 1e-12  # lead within 3 m: no release
      in_zone += 1
    prev = a2
  assert in_zone > 50


def test_mode2_lead_zone_then_standstill_latches():
  c2 = K9Creep(V9_SHUDDER_FIX)
  for v in np.linspace(0.6, 0.06, 100):
    a, s = upd(c2, v_ego=float(v), v_ego_raw=float(v), lead_distance=2.9)
    assert s is False
  before = c2.sent
  a, s = upd(c2, v_ego=0.04, v_ego_raw=0.04, lead_distance=2.9)
  assert s is True and a <= before + 1e-12  # standstill latch, no release at the latch


@pytest.mark.parametrize("mode,expect_sr_rolling", [(V9, True), (V9_SHUDDER_FIX, False)])
def test_through_carcontroller_lead_2_9_m(monkeypatch, mode, expect_sr_rolling):
  r = Rig(monkeypatch, K9StopMode=mode)
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
