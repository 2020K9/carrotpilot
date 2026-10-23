"""K9StopMode 3 = v10 + band 0.2 (the K9Creep kind V10_BAND02, first name V10_SHUDDER_FIX; 2026-10-23 brake_v10band r6).
update() (accel, StopReq, feather, latch ramp, 3.0 m lead latch, interlocks) is exactly K9StopMode 1 (the page v10,
test_k9_stop_v10.py) on every frame. Only SCC14 ComfortBand differs: where menu 1 sends 0/0, menu 3 sends 0.20/0.20
(the driver's spec 2026-10-23 17:40), moved at most 0.04 per make_jerk call from the band sent before.
Rewritten 2026-10-23 r6 (behavior pins of the r2-r5 lead stop fix / r5 band, removed from menu 3 on purpose):
test_menu_and_constants, test_lead_latch_and_interlocks -> test_lead_latch_and_interlocks_are_menu_1,
test_lead_stop_accel_is_menu_1_every_frame_stopreq_later -> test_lead_stop_is_menu_1_every_frame,
test_lead_hold_resets_with_the_stopping_state, test_lead_zone_predicted_short_stop_sends_stopreq and
test_lead_zone_withholds_only_with_room -> test_lead_zone_is_menu_1, test_bands_are_menu_1 -> test_bands_are_0_2_where_menu_1_sends_0.
Kept unchanged: test_accel_and_stopreq_are_menu_1_bit_for_bit, test_lead_zone_urgent_target_latches_at_once,
test_band_only_changes_scc14_band_not_v9. New r6: test_band_ramp_from_the_previous_band, test_band_never_changes_accel_or_stopreq,
test_band_on_the_can_grid. Sim/replay only; NOT verified on the vehicle."""
import math
import random

import pytest

from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V9, V10, V9_SHUDDER_FIX, V10_BAND02, V10_SHUDDER_FIX
from openpilot.selfdrive.car.tests.test_k9_stop_v10 import V10Ref, _stops


def _upd(c, f, **kw):
  args = dict(accel=f["accel"], a_target=f["a_target"], stopping=f["stopping"], pid=f["pid"], v_ego=f["v_ego"],
              v_ego_raw=f["v_ego_raw"], a_ego=f["a_ego"], long_accel=f["long_accel"], esp_standstill=f["esp_standstill"],
              long_active=True, override=False, gas=False, brake=False, can_valid=True, lead_visible=False, lead_distance=0.0)
  args.update(kw)
  return c.update(**args)


def test_menu_and_constants():
  assert kc.MENU[3] == V10_BAND02 == V10_SHUDDER_FIX == 4 and kc.MENU[1] == V10
  c = K9Creep(V10_BAND02)
  assert c.v10 and c.v10_band and not c.shudder_fix
  assert not K9Creep(V10).v10_band and not K9Creep(V9_SHUDDER_FIX).v10_band
  assert kc.LEAD_LATCH_GAP == 3.0 and kc.LEAD_LATCH_GAP_2 == 2.5
  assert kc.V10_BAND == 0.20 and kc.V10_BAND_STEP == 0.04 == kc.BAND_RAMP_STEP
  assert not hasattr(kc, "LEAD_PRED_GAP") and not hasattr(kc, "V10_FIX_CB")  # r6: no lead stop fix in menu 3


@pytest.mark.parametrize("seed", range(8))
def test_accel_and_stopreq_are_menu_1_bit_for_bit(seed):
  for frames in _stops(seed):
    ref, c1, c3 = V10Ref(), K9Creep(V10), K9Creep(V10_SHUDDER_FIX)
    for f in frames:
      want = ref.step(**f)
      assert _upd(c1, f) == want and _upd(c3, f) == want


@pytest.mark.parametrize("lead", [2.9, 2.6, 2.51, 3.0, 2.5, 1.0, 3.01, 0.0])
def test_lead_latch_and_interlocks_are_menu_1(lead):
  base = dict(accel=-0.3, a_target=-0.3, stopping=True, pid=False, v_ego=0.6, v_ego_raw=0.6, a_ego=-0.3, long_accel=-0.3,
              esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True, lead_visible=True)
  a3, s3 = K9Creep(V10_BAND02).update(lead_distance=lead, **base)
  a1, s1 = K9Creep(V10).update(lead_distance=lead, **base)
  assert (a3, s3) == (a1, s1)
  assert s3 is (lead <= kc.LEAD_LATCH_GAP)  # the 3.0 m lead latch of menu 1
  for k in ("override", "gas", "brake"):
    assert K9Creep(V10_BAND02).update(lead_distance=10.0, **dict(base, **{k: True})) == (-0.3, True)
  assert K9Creep(V10_BAND02).update(lead_distance=10.0, **dict(base, can_valid=False)) == (-0.3, True)


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
def test_lead_stop_is_menu_1_every_frame(seed):
  rng = random.Random(f"k9-v10-lead-{seed}")
  c1, c3 = K9Creep(V10), K9Creep(V10_BAND02)
  for kw in _lead_approach(rng):
    assert c3.update(**kw) == c1.update(**kw)


def test_lead_zone_urgent_target_latches_at_once():
  c1, c3 = K9Creep(V10), K9Creep(V10_SHUDDER_FIX)
  base = dict(accel=-0.6, a_target=-0.3, stopping=True, pid=False, v_ego=0.5, v_ego_raw=0.5, a_ego=-0.3, long_accel=-0.3,
              esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True,
              lead_visible=True, lead_distance=2.8)
  for _ in range(10):
    assert c3.update(**base)[0] == c1.update(**base)[0]
  urgent = dict(base, a_target=-1.2, accel=-1.3)
  assert c3.update(**urgent) == c1.update(**urgent) == (-1.3, True)


def _zone(v, ac, gap, **kw):
  d = dict(accel=-0.6, a_target=-0.3, stopping=True, pid=False, v_ego=v, v_ego_raw=v, a_ego=ac, long_accel=ac,
           esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True,
           lead_visible=True, lead_distance=gap)
  d.update(kw)
  return d


@pytest.mark.parametrize("v,ac,gap", [(0.8, -0.3, 2.9), (0.6, -0.5, 2.9), (0.4, -0.1, 2.8), (0.3, 0.0, 2.95), (0.5, -0.3, 2.7),
                                      (0.3, -0.3, 2.8), (0.2, -0.3, 2.7), (0.4, -0.6, 2.9), (0.1, -0.2, 2.65)])
def test_lead_zone_is_menu_1(v, ac, gap):
  c1, c3 = K9Creep(V10), K9Creep(V10_BAND02)
  for _ in range(5):
    assert c3.update(**_zone(v, ac, gap)) == c1.update(**_zone(v, ac, gap))
  assert c3.update(**_zone(v, 0.0, gap)) == c1.update(**_zone(v, 0.0, gap))
  assert c3.latched == c1.latched is True


def test_bands_are_0_2_where_menu_1_sends_0():
  for v in (0.0, 0.3, 1.0, 1.39, 1.4, 2.0, 2.5, 3.0, 5.0, 20.0):
    for cb_prev in (0.2, 0.0, 0.8, 1.2):
      c, c1 = K9Creep(V10_BAND02), K9Creep(V10)
      ju1, cu1, cl1 = c1.bands(0.5, v, 0.9, 0.8, cb_prev)
      assert (ju1, cu1, cl1) == K9Creep.jerk_bands(0.5, v) and cu1 == cl1 == 0.0  # menu 1: 0 on every applied frame
      for _ in range(40):
        ju, cu, cl = c.bands(0.5, v, 0.9, 0.8, cb_prev, cb_prev)
        assert ju == ju1  # jerk_u is menu 1's
      assert cu == cl == kc.V10_BAND  # settled: exactly 0.20, upper and lower


def test_band_ramp_from_the_previous_band():
  # r6 (new): each make_jerk call moves upper/lower at most 0.04 from the value sent on the previous call; lands on 0.20
  for start_u, start_l in ((0.0, 0.0), (0.83, 0.73), (1.2, 1.1), (0.21, 0.19), (0.2, 0.2), (0.5, 0.0)):
    c = K9Creep(V10_BAND02)
    pu, pl, n = start_u, start_l, 0
    while True:
      _, cu, cl = c.bands(0.5, 1.0, 0.75, 0.65, pu, pl)
      assert abs(cu - pu) <= kc.V10_BAND_STEP + 1e-12 and abs(cl - pl) <= kc.V10_BAND_STEP + 1e-12
      assert min(pu, kc.V10_BAND) - 1e-12 <= cu <= max(pu, kc.V10_BAND) + 1e-12  # monotonic toward 0.20, no overshoot
      pu, pl, n = cu, cl, n + 1
      if cu == cl == kc.V10_BAND:
        break
      assert n < 40
    assert n == max(1, math.ceil(round(max(abs(start_u - 0.2), abs(start_l - 0.2)) / 0.04, 9)))
  # lower without its own previous value follows the upper start (old callers)
  c = K9Creep(V10_BAND02)
  assert c.bands(0.5, 1.0, 0.75, 0.65, 0.0) == (K9Creep.jerk_bands(0.5, 1.0)[0], 0.04, 0.04)
  # non-finite previous band: 0.20 at once (fails toward the menu value, never NaN on the CAN)
  assert K9Creep(V10_BAND02).bands(0.5, 1.0, 0.75, 0.65, float("nan"), float("nan"))[1:] == (0.2, 0.2)
  # reset (interlock / mode change) starts again from the band sent before
  c = K9Creep(V10_BAND02)
  for _ in range(10):
    c.bands(0.5, 1.0, 0.75, 0.65, 0.2, 0.2)
  c.reset()
  assert c.bands(0.5, 1.0, 0.75, 0.65, 0.75, 0.65)[1:] == (0.71, 0.61)


def test_band_on_the_can_grid():
  # r6 (new): SCC14 ComfortBand is 6 bit x 0.02; the settled value encodes as exactly 10 (0.20) and decodes back to 0.20
  raw = int(math.floor(kc.V10_BAND / 0.02 + 0.5))
  assert raw == 10 and abs(raw * 0.02 - 0.20) < 1e-12
  c = K9Creep(V10_BAND02)
  pu = pl = 0.83
  for _ in range(30):
    _, pu, pl = c.bands(0.5, 0.5, 0.8, 0.7, pu, pl)
    assert 0 <= int(math.floor(pu / 0.02 + 0.5)) <= 63
  assert int(math.floor(pu / 0.02 + 0.5)) == int(math.floor(pl / 0.02 + 0.5)) == 10


@pytest.mark.parametrize("seed", range(4))
def test_band_never_changes_accel_or_stopreq(seed):
  # r6 (new): calling bands() between update() calls (as make_jerk does) leaves accel/StopReq menu 1's bit for bit
  for frames in _stops(seed):
    ref, c3 = V10Ref(), K9Creep(V10_BAND02)
    pu = pl = 0.0
    for f in frames:
      assert _upd(c3, f) == ref.step(**f)
      if c3.active:
        _, pu, pl = c3.bands(1.0, f["v_ego"], 0.8, 0.7, pu, pl)


def test_band_only_changes_scc14_band_not_v9():
  assert K9Creep(V9).bands(0.5, 1.0, 0.9, 0.8, 0.0) == K9Creep.jerk_bands(0.5, 1.0)
