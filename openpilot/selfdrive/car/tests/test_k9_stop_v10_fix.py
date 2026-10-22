"""K9StopMode 3 = v10 + shudder fix (the K9Creep kind V10_SHUDDER_FIX; 2026-10-23 brake_v10band, 07:25/07:29 menu).
accel and StopReq are exactly K9StopMode 1 (the page v10, test_k9_stop_v10.py); only ComfortBand differs: a narrow
low-speed band V10_FIX_CB (not the original 0.8-0.9), ramped, on the SCC14 0.02 grid.
New tests only. Sim/replay only; NOT verified on the vehicle."""
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


def _q(x):
  return round(x / kc.CB_CAN_RES) * kc.CB_CAN_RES


def test_menu_and_constants():
  assert kc.MENU[3] == V10_SHUDDER_FIX and kc.MENU[1] == V10
  c = K9Creep(V10_SHUDDER_FIX)
  assert c.v10 and c.v10_fix and not c.shudder_fix
  assert not K9Creep(V10).v10_fix
  assert kc.V10_FIX_CB == 0.04 and kc.CB_CAN_RES == 0.02
  assert (kc.V10_FIX_BAND_FULL_SPEED, kc.V10_FIX_BAND_OFF_SPEED, kc.V10_FIX_RAMP_STEP) == (2.0, 3.0, 0.04)
  # narrower than the v10 hold ramp's request change (LATCH_ACCEL - page feather at 0.05..0.11 m/s)
  assert kc.V10_FIX_CB < (kc.V10_FEATHER_V[0] - kc.LATCH_ACCEL) / 2


@pytest.mark.parametrize("seed", range(8))
def test_accel_and_stopreq_are_menu_1_bit_for_bit(seed):
  for frames in _stops(seed):
    ref, c1, c3 = V10Ref(), K9Creep(V10), K9Creep(V10_SHUDDER_FIX)
    for f in frames:
      want = ref.step(**f)
      assert _upd(c1, f) == want and _upd(c3, f) == want


@pytest.mark.parametrize("lead", [2.9, 1.0])
def test_lead_latch_and_interlocks_are_menu_1(lead):
  base = dict(accel=-0.3, a_target=-0.3, stopping=True, pid=False, v_ego=0.6, v_ego_raw=0.6, a_ego=-0.3, long_accel=-0.3,
              esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True, lead_visible=True)
  assert K9Creep(V10_SHUDDER_FIX).update(lead_distance=lead, **base) == K9Creep(V10).update(lead_distance=lead, **base)
  for k in ("override", "gas", "brake"):
    assert K9Creep(V10_SHUDDER_FIX).update(lead_distance=10.0, **dict(base, **{k: True})) == (-0.3, True)
  assert K9Creep(V10_SHUDDER_FIX).update(lead_distance=10.0, **dict(base, can_valid=False)) == (-0.3, True)


def test_band_is_narrow_low_speed_on_the_can_grid():
  for v in (0.0, 0.3, 1.0, 1.39, 1.4, 2.0, 2.5, 2.9, 3.0, 5.0):
    c, c1 = K9Creep(V10_SHUDDER_FIX), K9Creep(V10)
    for _ in range(60):
      got = c.bands(0.5, v, 0.9, 0.8, 0.0)
    ref1 = c1.bands(0.5, v, 0.9, 0.8, 0.0)
    assert got[0] == ref1[0] and ref1[1:] == (0.0, 0.0)  # jerk_u as v10; v10 sends band 0
    want = _q(kc.V10_FIX_CB * kc.v10_fix_band_weight(v))
    assert got[1] == pytest.approx(want) and got[2] == pytest.approx(want)
    assert abs(got[1] / kc.CB_CAN_RES - round(got[1] / kc.CB_CAN_RES)) < 1e-9  # on the 0.02 grid
  assert K9Creep(V10_SHUDDER_FIX).bands(0.5, 5.0, 0.9, 0.8, 0.0)[1:] == (0.0, 0.0)


def test_band_ramps_from_the_previous_band():
  c = K9Creep(V10_SHUDDER_FIX)
  ups = [c.bands(0.5, 1.0, 0.9, 0.8, 0.0)[1] for _ in range(int(1 / kc.V10_FIX_RAMP_STEP) + 5)]
  assert ups[0] == 0.0 and ups[-1] == pytest.approx(kc.V10_FIX_CB)
  assert all(0.0 <= b - a <= kc.CB_CAN_RES + 1e-9 for a, b in zip(ups, ups[1:]))  # one grid step at most, no jump
  c = K9Creep(V10_SHUDDER_FIX)  # switching in from 2 (original band about 0.8): starts at the full narrow band, ramps down
  assert c.bands(0.5, 5.0, 0.9, 0.8, 0.8)[1] == pytest.approx(_q(kc.V10_FIX_CB * (1 - kc.V10_FIX_RAMP_STEP)))


def test_band_only_changes_scc14_band_not_v9():
  assert K9Creep(V9).bands(0.5, 1.0, 0.9, 0.8, 0.0) == K9Creep.jerk_bands(0.5, 1.0)
