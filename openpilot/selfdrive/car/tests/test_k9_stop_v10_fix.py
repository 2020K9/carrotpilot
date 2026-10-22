"""K9StopMode 3 = v10 + lead stop fix (the K9Creep kind V10_SHUDDER_FIX; 2026-10-23 brake_v10band r2).
accel is exactly K9StopMode 1 (the page v10, test_k9_stop_v10.py) on every frame; only StopReq timing differs: the lead
latch is at 2.5 m (as K9StopMode 2) instead of 3.0 m, and between 2.5 and 3.0 m the mode 1 latch ramp is sent without
StopReq. ComfortBand/jerk are mode 1's (band 0; the r1 narrow band 0.04 was dropped in r2).
Rewritten 2026-10-23 r2 (behavior pins of the r1 band version: test_menu_and_constants, test_lead_latch_and_interlocks_are_menu_1,
test_band_is_narrow_low_speed_on_the_can_grid -> test_bands_are_menu_1, test_band_ramps_from_the_previous_band -> removed);
the lead tests below are new. Sim/replay only; NOT verified on the vehicle."""
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
  assert s3 is (lead <= kc.LEAD_LATCH_GAP_2)
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
      zone_rolling_withheld += 1
    if kw["v_ego"] == 0.0:
      assert s3 is True  # standstill latches
  assert zone_rolling_withheld > 0


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
  base = dict(accel=-0.6, a_target=-0.3, stopping=True, pid=False, v_ego=0.5, v_ego_raw=0.5, a_ego=-0.3, long_accel=-0.3,
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
