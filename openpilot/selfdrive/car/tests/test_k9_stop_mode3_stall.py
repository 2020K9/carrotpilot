"""Menu 3 stall ramp above 1.5 km/h (2026-11-12, log 00000227--1a9943ad32--13); synthetic K9Creep tests, not road validation."""
import pytest
from opendbc.car.hyundai import k9_creep as kc
from openpilot.selfdrive.car.tests.test_k9_creep_mode import upd

V = 0.45  # m/s, the floating speed of the logged stop (above 1.5 km/h)


def test_stall_ramp_starts_after_frames():
  c = kc.K9Creep(kc.V12K9OGR_HALF)
  for k in range(1, kc.B18_STALL_FRAMES):
    upd(c, v_ego=V, v_ego_raw=V, a_target=-0.01, a_ego=-0.05)
    assert c.b17o_ramp == 0, k
  upd(c, v_ego=V, v_ego_raw=V, a_target=-0.01, a_ego=-0.05)
  assert c.b17o_ramp == -kc.B17O_RAMP_STEP


@pytest.mark.parametrize("a_ego", [-0.16, -0.5])
def test_no_stall_ramp_while_slowing(a_ego):
  c = kc.K9Creep(kc.V12K9OGR_HALF)
  for _ in range(100):
    upd(c, v_ego=V, v_ego_raw=V, a_target=-0.01, a_ego=a_ego)
  assert c.b17o_ramp == 0


def test_no_stall_ramp_above_stall_speed():
  c = kc.K9Creep(kc.V12K9OGR_HALF)
  for _ in range(100):
    upd(c, v_ego=kc.B18_STALL_V + 0.01, v_ego_raw=kc.B18_STALL_V + 0.01, a_target=-0.01, a_ego=0.0)
  assert c.b17o_ramp == 0


def test_stall_counter_needs_consecutive_frames():
  c = kc.K9Creep(kc.V12K9OGR_HALF)
  for k in range(100):
    upd(c, v_ego=V, v_ego_raw=V, a_target=-0.01, a_ego=-0.05 if k % kc.B18_STALL_FRAMES else -0.5)
  assert c.b17o_ramp == 0


@pytest.mark.parametrize("mode", [kc.V12K9OH, kc.V12K9OGR])
def test_menus_1_2_have_no_stall_ramp(mode):
  c = kc.K9Creep(mode)
  for _ in range(100):
    upd(c, v_ego=V, v_ego_raw=V, a_target=-0.01, a_ego=-0.05)
  assert c.b17o_ramp == 0


def test_stall_ramp_cap_and_reset():
  c = kc.K9Creep(kc.V12K9OGR_HALF)
  for _ in range(400):
    upd(c, v_ego=V, v_ego_raw=V, a_target=-0.01, a_ego=0.0)
  assert c.b17o_ramp == -kc.B17O_RAMP_MAX
  upd(c, stopping=False)
  assert c.b17o_ramp == 0 and c.b17o_hold is None and c.b18_stall_k == 0
