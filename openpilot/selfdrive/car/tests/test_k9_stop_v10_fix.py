"""K9StopMode 3 = v10 + lead stop fix (the K9Creep kind V10_SHUDDER_FIX; 2026-10-23 brake_v10band r2).
accel is exactly K9StopMode 1 (the page v10, test_k9_stop_v10.py) on every frame; only StopReq timing differs: the lead
latch is at 2.5 m (as K9StopMode 2) instead of 3.0 m, and between 2.5 and 3.0 m the mode 1 latch ramp is sent without
StopReq. ComfortBand/jerk are mode 1's (band 0; the r1 narrow band 0.04 was dropped in r2).
Rewritten 2026-10-23 r2 (behavior pins of the r1 band version: test_menu_and_constants, test_lead_latch_and_interlocks_are_menu_1,
test_band_is_narrow_low_speed_on_the_can_grid -> test_bands_are_menu_1, test_band_ramps_from_the_previous_band -> removed);
the lead tests below are new. Rewritten 2026-10-23 r3 (pins of the r2 StopReq timing, now with the predicted stop point
LEAD_PRED_GAP: test_lead_latch_and_interlocks, test_lead_hold_resets_with_the_stopping_state,
test_lead_stop_accel_is_menu_1_every_frame_stopreq_later - its 'withheld > 0' pin dropped, prediction check added); new r3 tests:
test_lead_zone_predicted_short_stop_sends_stopreq, test_lead_zone_withholds_only_with_room. Sim/replay only; NOT verified
on the vehicle."""
import random

import numpy as np
import pytest

from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V9, V10, V10_SHUDDER_FIX
from openpilot.selfdrive.car.tests.test_k9_stop_v10 import V10Ref, _stops


def _upd(c, f, **kw):
  args = dict(accel=f["accel"], a_target=f["a_target"], stopping=f["stopping"], pid=f["pid"], v_ego=f["v_ego"],
              v_ego_raw=f["v_ego_raw"], a_ego=f["a_ego"], long_accel=f["long_accel"], esp_standstill=f["esp_standstill"],
              long_active=True, override=False, gas=False, brake=False, can_valid=True, lead_visible=False, lead_distance=0.0)
  args.update(kw)
  return c.update(**args)


def test_menu_and_constants():
  assert kc.MENU[3] == V10_SHUDDER_FIX and kc.MENU[1] == V10
  c = K9Creep(V10_SHUDDER_FIX)
  assert c.v10 and c.v10_fix and not c.shudder_fix
  assert not K9Creep(V10).v10_fix
  assert kc.LEAD_LATCH_GAP == 3.0 and kc.LEAD_LATCH_GAP_2 == 2.5
  assert kc.LEAD_PRED_GAP == 2.6 and kc.LEAD_PRED_DECEL_MIN == 0.05
  assert not hasattr(kc, "V10_FIX_CB")  # r2: no mode 4 band


@pytest.mark.parametrize("seed", range(8))
def test_accel_and_stopreq_are_menu_1_bit_for_bit(seed):
  for frames in _stops(seed):
    ref, c1, c3 = V10Ref(), K9Creep(V10), K9Creep(V10_SHUDDER_FIX)
    for f in frames:
      want = ref.step(**f)
      assert _upd(c1, f) == want and _upd(c3, f) == want


@pytest.mark.parametrize("lead", [2.9, 2.6, 2.51, 3.0, 2.5, 1.0])
def test_lead_latch_and_interlocks(lead):
  base = dict(accel=-0.3, a_target=-0.3, stopping=True, pid=False, v_ego=0.6, v_ego_raw=0.6, a_ego=-0.3, long_accel=-0.3,
              esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True, lead_visible=True)
  a3, s3 = K9Creep(V10_SHUDDER_FIX).update(lead_distance=lead, **base)
  a1, s1 = K9Creep(V10).update(lead_distance=lead, **base)
  assert a3 == a1 and s1 is True
  pred = lead - 0.6 ** 2 / (2.0 * 0.3)  # predicted stop point: v 0.6 m/s, eff_ac -0.3 m/s^2
  assert s3 is (lead <= kc.LEAD_LATCH_GAP_2 or pred < kc.LEAD_PRED_GAP)
  for k in ("override", "gas", "brake"):
    assert K9Creep(V10_SHUDDER_FIX).update(lead_distance=10.0, **dict(base, **{k: True})) == (-0.3, True)
  assert K9Creep(V10_SHUDDER_FIX).update(lead_distance=10.0, **dict(base, can_valid=False)) == (-0.3, True)


def _lead_approach(rng, n=320):
  """stopping from 0.9 m/s toward a lead 3.8 -> 2.3 m (K9 planner stops 2.2-3.0 m behind), noisy feedback, then standstill."""
  for i in range(n):
    v = max(0.9 - 0.9 * i / (n - 60), 0.0)
    gap = max(3.8 - 1.5 * i / (n - 60), rng.choice([2.3, 2.6, 2.8]))
    ae = rng.uniform(-0.6, 0.05) if v > 0 else 0.0
    yield dict(accel=rng.uniform(-1.0, -0.4), a_target=rng.choice([rng.uniform(-0.7, -0.05), -0.3]), stopping=True, pid=False,
               v_ego=v, v_ego_raw=v, a_ego=ae, long_accel=min(rng.uniform(-0.7, 0.03), ae + 0.1), esp_standstill=v == 0.0,
               long_active=True, override=False, gas=False, brake=False, can_valid=True,
               lead_visible=rng.random() < 0.9, lead_distance=gap)


@pytest.mark.parametrize("seed", range(10))
def test_lead_stop_accel_is_menu_1_every_frame_stopreq_later(seed):
  rng = random.Random(f"k9-v10-lead-{seed}")
  c1, c3 = K9Creep(V10), K9Creep(V10_SHUDDER_FIX)
  zone_rolling_withheld = 0
  for kw in _lead_approach(rng):
    a1, s1 = c1.update(**kw)
    a3, s3 = c3.update(**kw)
    assert a3 == a1  # same accel on every frame (never weaker than menu 1)
    assert s3 <= s1  # StopReq never earlier than menu 1
    if s1 and not s3:
      # only withheld while a lead is in 2.5-3.0 m and no other mode 1 latch condition holds
      assert kc.LEAD_LATCH_GAP_2 < kw["lead_distance"] <= kc.LEAD_LATCH_GAP or c3.lead_hold
      assert kw["v_ego"] >= kc.STANDSTILL_SPEED and kw["a_ego"] <= 0.06 and kw["a_target"] > kc.URGENT_TARGET
      # r3: and only while the predicted stop point keeps LEAD_PRED_GAP to the lead
      eff_ac = min(kw["long_accel"], kw["a_ego"] + 0.10)
      v = kw["v_ego"]
      assert kw["lead_distance"] - v * v / (2.0 * max(-eff_ac, kc.LEAD_PRED_DECEL_MIN)) >= kc.LEAD_PRED_GAP
      zone_rolling_withheld += 1
    if kw["v_ego"] == 0.0:
      assert s3 is True  # standstill latches
  # r3: the r2 pin 'zone_rolling_withheld > 0' is dropped - with the predicted stop point a noisy approach may send
  # StopReq exactly like menu 1 (withheld 0 frames); the withholding itself is pinned by test_lead_zone_withholds_only_with_room
  assert zone_rolling_withheld >= 0


def test_lead_zone_urgent_target_latches_at_once():
  c1, c3 = K9Creep(V10), K9Creep(V10_SHUDDER_FIX)
  base = dict(accel=-0.6, a_target=-0.3, stopping=True, pid=False, v_ego=0.5, v_ego_raw=0.5, a_ego=-0.3, long_accel=-0.3,
              esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True,
              lead_visible=True, lead_distance=2.8)
  for _ in range(10):
    assert c3.update(**base)[0] == c1.update(**base)[0]
  urgent = dict(base, a_target=-1.2, accel=-1.3)
  assert c3.update(**urgent) == c1.update(**urgent) == (-1.3, True)


def test_lead_hold_resets_with_the_stopping_state():
  c3 = K9Creep(V10_SHUDDER_FIX)
  # r3: 0.3 m/s at -0.3 m/s^2 stops 0.15 m on, 2.65 m behind the lead (room), so StopReq is withheld
  base = dict(accel=-0.6, a_target=-0.3, stopping=True, pid=False, v_ego=0.3, v_ego_raw=0.3, a_ego=-0.3, long_accel=-0.3,
              esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True,
              lead_visible=True, lead_distance=2.8)
  c3.update(**base)
  assert c3.lead_hold
  c3.update(**dict(base, stopping=False, pid=True, lead_distance=10.0))
  assert not c3.lead_hold


def test_bands_are_menu_1():
  for v in (0.0, 0.3, 1.0, 1.39, 1.4, 2.0, 2.5, 3.0, 5.0):
    c, c1 = K9Creep(V10_SHUDDER_FIX), K9Creep(V10)
    for cb_prev in (0.0, 0.8):
      assert c.bands(0.5, v, 0.9, 0.8, cb_prev) == c1.bands(0.5, v, 0.9, 0.8, cb_prev) == K9Creep.jerk_bands(0.5, v)


def test_band_only_changes_scc14_band_not_v9():
  assert K9Creep(V9).bands(0.5, 1.0, 0.9, 0.8, 0.0) == K9Creep.jerk_bands(0.5, 1.0)


def _zone(v, ac, gap, **kw):
  d = dict(accel=-0.6, a_target=-0.3, stopping=True, pid=False, v_ego=v, v_ego_raw=v, a_ego=ac, long_accel=ac,
           esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True,
           lead_visible=True, lead_distance=gap)
  d.update(kw)
  return d


@pytest.mark.parametrize("v,ac,gap", [(0.8, -0.3, 2.9), (0.6, -0.5, 2.9), (0.4, -0.1, 2.8), (0.3, 0.0, 2.95), (0.5, -0.3, 2.7)])
def test_lead_zone_predicted_short_stop_sends_stopreq(v, ac, gap):
  """r3: in the 2.5-3.0 m zone, a predicted stop point closer than LEAD_PRED_GAP latches at once, exactly like menu 1."""
  pred = gap - v * v / (2.0 * max(-min(ac, ac + 0.1), kc.LEAD_PRED_DECEL_MIN))
  assert pred < kc.LEAD_PRED_GAP
  c1, c3 = K9Creep(V10), K9Creep(V10_SHUDDER_FIX)
  for _ in range(5):
    assert c3.update(**_zone(v, ac, gap)) == c1.update(**_zone(v, ac, gap))
  assert c3.latched and not c3.lead_hold


@pytest.mark.parametrize("v,ac,gap", [(0.3, -0.3, 2.8), (0.2, -0.3, 2.7), (0.4, -0.6, 2.9), (0.1, -0.2, 2.65)])
def test_lead_zone_withholds_only_with_room(v, ac, gap):
  """r3: StopReq is withheld only while the predicted stop point keeps LEAD_PRED_GAP; accel stays menu 1's."""
  pred = gap - v * v / (2.0 * max(-min(ac, ac + 0.1), kc.LEAD_PRED_DECEL_MIN))
  assert pred >= kc.LEAD_PRED_GAP
  c1, c3 = K9Creep(V10), K9Creep(V10_SHUDDER_FIX)
  for _ in range(5):
    a1, s1 = c1.update(**_zone(v, ac, gap))
    a3, s3 = c3.update(**_zone(v, ac, gap))
    assert a3 == a1 and s1 is True and s3 is False and c3.lead_hold
  # the car stops braking (eff_ac rises): the prediction runs short and StopReq follows at once
  a1, s1 = c1.update(**_zone(v, 0.0, gap))
  a3, s3 = c3.update(**_zone(v, 0.0, gap))
  assert a3 == a1 and s3 is True
