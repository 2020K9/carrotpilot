# Written for the lanebias/corner v3 review; NOT executed (test runs await approval).
# Every numeric value below is synthetic test input, not an approved setting.
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import laneless_center as lc
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.laneless_center import (
  APPROVED_CONFIG, BLOCK_DISABLED, BLOCK_INACTIVE, BLOCK_LANE_CHANGE, BLOCK_LANE_MODE, BLOCK_LANES, BLOCK_MISMATCH,
  BLOCK_MISSING, BLOCK_MODEL_STALE, BLOCK_NO_INTERVAL, BLOCK_PATH_VERIFIER, BLOCK_STEERING_PRESSED,
  LanelessCenterConfig, LanelessCenterCorrection, center_bounded_target, consume_curvature,
  consumed_within_bounds, curvature_delta_interval, lane_center, reconstructed_offsets, uniform_shift_interval)

DT = 0.01
V = 20.0
X = np.linspace(0.0, 60.0, 33)
TEST_CFG = LanelessCenterConfig(min_lane_prob=0.5, max_lane_std=0.3, check_x_min=5.0, check_x_max=40.0,
                                eval_x=20.0, gain=0.5, max_path_direct_mismatch=0.01, max_model_age=0.2,
                                max_delta_rate=1.0, max_lane_mode_weight=0.0)
WIDTH = 3.6


def old_chain(weight, lane_curvature, laneless_curvature, prev, lat_smooth, v=V, roll=0.0, clip_fn=clip_curvature):
  # Exactly the pre-change controlsd expressions (controlsd.py:286-291 at 0e117698).
  def smooth_value(val, prev_val, tau):
    alpha = 1 - np.exp(-DT / tau) if tau > 0 else 1
    return alpha * val + (1 - alpha) * prev_val
  from openpilot.selfdrive.controls.lib.lat_mode_blend import blend_lat_mode
  lane_target = laneless_curvature if lane_curvature is None else lane_curvature
  curvature = blend_lat_mode(weight, lane_target, laneless_curvature)
  tau = blend_lat_mode(weight, lat_smooth, 0.1)
  return clip_fn(v, prev, smooth_value(curvature, prev, tau), roll)


def inputs(center_offset=0.5, plan_y=None, model_curvature=0.0, **overrides):
  plan_y = np.zeros_like(X) if plan_y is None else np.asarray(plan_y, dtype=float)
  center = np.full_like(X, center_offset) if np.isscalar(center_offset) else np.asarray(center_offset, dtype=float)
  kw = dict(dt=DT, lat_active=True, steering_pressed=False, lane_change_active=False, lane_mode_weight=0.0,
            model_age=0.05, model_valid=True, model_curvature=model_curvature, path_verifier_effect=0.0,
            plan_x=X, plan_y=plan_y, lane_x=X, lll_x=X, rll_x=X,
            lll_y=center - WIDTH / 2, rll_y=center + WIDTH / 2,
            lll_prob=0.9, rll_prob=0.9, lll_std=0.1, rll_std=0.1)
  kw.update(overrides)
  return kw


def run(cycles=200, **kw):
  corr = LanelessCenterCorrection(TEST_CFG)
  delta = 0.0
  for _ in range(cycles):
    delta = corr.update(**inputs(**kw))
  return corr, delta


# ------------------------------------------------------------ default off

def test_no_approved_config_and_default_is_disabled():
  assert APPROVED_CONFIG is None
  corr = LanelessCenterCorrection()
  assert not corr.enabled
  assert corr.update(**inputs()) == 0.0
  assert corr.trace["reason"] == BLOCK_DISABLED


def test_partial_config_is_not_approved():
  assert not LanelessCenterCorrection(LanelessCenterConfig(gain=0.5)).enabled


@pytest.mark.parametrize("weight", [0.0, 0.3, 1.0])
@pytest.mark.parametrize("lane_curvature", [None, 0.004])
def test_zero_delta_chain_is_bit_identical_to_original(weight, lane_curvature):
  for prev in (0.0, 0.002, -0.003):
    new, limited, _ = consume_curvature(weight, lane_curvature, -0.002, 0.0, prev, 0.2, V, 0.0, DT, clip_curvature)
    old, old_limited = old_chain(weight, lane_curvature, -0.002, prev, 0.2)
    assert new == old and limited == old_limited


def test_controlsd_default_branch_kept_and_new_path_gated():
  src = (Path(__file__).resolve().parents[1] / "controlsd.py").read_text()
  assert "self.laneless_center = LanelessCenterCorrection()" in src
  assert "if self.laneless_center.enabled:" in src
  # original consumption lines remain verbatim in the disabled branch
  for line in ("curvature = blend_lat_mode(self.lat_mode_blend, lane_target, laneless_curvature)",
               "new_desired_curvature = smooth_value(curvature, self.desired_curvature, tau)",
               "self.desired_curvature, curvature_limited = clip_curvature(CS.vEgo, self.desired_curvature, "
               "new_desired_curvature, lp.roll)",
               "CC.latActive = CC.latActive and lateral_ready",
               "self.desired_curvature = self.curvature"):
    assert line in src
  # correction goes after PathVerifier and into the same desired_curvature handed to LaC
  pv = src.index("laneless_curvature = self.path_verifier_curvature(")
  delta = src.index("laneless_center_delta = self.laneless_center_delta(")
  assert pv < delta
  assert "self.steer_limited_by_safety, self.desired_curvature," in src


# ------------------------------------------------------------ connection to consumption

def test_pure_laneless_correction_reaches_consumed_curvature():
  corr, delta = run()
  assert corr.trace["reason"] == ""
  assert delta > 0.0  # centre is to the right (y right positive), so curvature turns right
  prev = 0.0
  new, _, stages = consume_curvature(0.0, None, 0.0, delta, prev, 0.2, V, 0.0, DT, clip_curvature)
  old, _ = old_chain(0.0, None, 0.0, prev, 0.2)
  assert stages["corrected"] == delta
  assert new != old  # reaches the value later passed to LaC.update
  assert corr.check_consumed(new, old)


def test_plan_only_edit_does_not_reach_consumption():
  # Old defect: shifting the plan (as lane_planner_2's disabled adjust would) leaves
  # desiredCurvature, hence the consumed command, untouched in laneless mode.
  prev = 0.0
  laneless = 0.001
  new_plan_only, _ = old_chain(0.0, 0.004, laneless, prev, 0.2)  # lane plan changed, weight 0
  unchanged, _ = old_chain(0.0, 0.0, laneless, prev, 0.2)
  assert new_plan_only == unchanged
  _, delta = run()
  with_delta, _, _ = consume_curvature(0.0, 0.0, laneless, delta, prev, 0.2, V, 0.0, DT, clip_curvature)
  assert with_delta != unchanged


def test_lane_mode_weight_one_has_no_laneless_addition():
  for prev in (0.0, 0.001):
    a, _, _ = consume_curvature(1.0, 0.003, 0.0, 0.002, prev, 0.2, V, 0.0, DT, clip_curvature)
    b, _, _ = consume_curvature(1.0, 0.003, 0.0, 0.0, prev, 0.2, V, 0.0, DT, clip_curvature)
    assert a == b


def test_transition_adds_correction_once_scaled_by_laneless_weight():
  no_clip = lambda v, prev, new, roll: (new, False)  # noqa: E731
  w, d = 0.25, 0.002
  _, _, s1 = consume_curvature(w, 0.003, 0.0, d, 0.0, 0.2, V, 0.0, DT, no_clip)
  _, _, s0 = consume_curvature(w, 0.003, 0.0, 0.0, 0.0, 0.2, V, 0.0, DT, no_clip)
  assert s1["blended"] - s0["blended"] == pytest.approx((1.0 - w) * d)


def test_lane_mode_weight_above_limit_blocks():
  corr, delta = run(lane_mode_weight=0.5)
  assert delta == 0.0 and corr.trace["reason"] == BLOCK_LANE_MODE


def test_clip_saturation_is_distinguished_from_missing_connection():
  saturated = lambda v, prev, new, roll: (prev, True)  # noqa: E731
  _, delta = run()
  new, limited, stages = consume_curvature(0.0, None, 0.0, delta, 0.0, 0.2, V, 0.0, DT, saturated)
  assert new == 0.0 and limited
  assert stages["corrected"] == delta and stages["smoothed"] != 0.0  # connected, then limited


# ------------------------------------------------------------ gating / validity

@pytest.mark.parametrize("override,reason", [
  ({"steering_pressed": True}, BLOCK_STEERING_PRESSED),
  ({"lane_change_active": True}, BLOCK_LANE_CHANGE),
  ({"model_age": 1.0}, BLOCK_MODEL_STALE),
  ({"model_age": float("nan")}, BLOCK_MODEL_STALE),
  ({"model_valid": False}, BLOCK_MODEL_STALE),
  ({"path_verifier_effect": 1e-4}, BLOCK_PATH_VERIFIER),
  ({"model_curvature": 0.05}, BLOCK_MISMATCH),
  ({"model_curvature": float("inf")}, BLOCK_MISSING),
  ({"plan_x": ()}, BLOCK_MISSING),
  ({"rll_prob": 0.2}, BLOCK_LANES),
  ({"lll_std": 0.9}, BLOCK_LANES),
  ({"rll_x": X + 0.5}, BLOCK_LANES),
  ({"lane_x": X[:10]}, BLOCK_MISSING),
])
def test_blocked_inputs_never_request(override, reason):
  corr = LanelessCenterCorrection(TEST_CFG)
  corr.update(**inputs(**override))
  assert corr.trace["reason"] == reason
  assert corr.trace["request"] == 0.0
  assert corr.delta == 0.0


def test_inactive_resets_immediately_and_block_releases_at_rate():
  corr, delta = run()
  assert delta > 0.0
  released = corr.update(**inputs(steering_pressed=True))
  assert 0.0 <= released < delta
  assert delta - released == pytest.approx(min(delta, TEST_CFG.max_delta_rate * DT))
  assert corr.update(**inputs(lat_active=False)) == 0.0
  assert corr.trace["reason"] == BLOCK_INACTIVE


def test_left_right_symmetry():
  _, right = run(center_offset=0.5)
  _, left = run(center_offset=-0.5)
  assert left == pytest.approx(-right)


# ------------------------------------------------------------ per-point centre limit

def test_far_point_already_on_center_allows_nothing():
  # 0.5 s point has room, the far point is already centred -> intersection is {0}
  center = np.where(X < 25.0, 0.5, 0.0)
  corr, delta = run(center_offset=center)
  assert delta == 0.0 and corr.trace["reason"] == BLOCK_NO_INTERVAL


def test_far_point_center_in_opposite_direction_allows_nothing():
  center = np.where(X < 25.0, 0.5, -0.5)
  corr, delta = run(center_offset=center)
  assert delta == 0.0


def test_path_crossing_center_allows_nothing():
  plan = np.interp(X, [0.0, 60.0], [-0.6, 0.6])
  lo, hi = curvature_delta_interval(X[1:20], plan[1:20], np.zeros(19))
  assert lo == 0.0 and hi == 0.0


@pytest.mark.parametrize("case", ["straight", "left_gentle", "right_gentle", "sharp"])
def test_every_point_and_interpolated_point_stays_between_path_and_center(case):
  k = {"straight": 0.0, "left_gentle": -0.002, "right_gentle": 0.002, "sharp": 0.01}[case]
  plan = k * X * X / 2.0
  center = plan + 0.4
  corr, delta = run(center_offset=center, plan_y=plan, model_curvature=k)
  xm = X[(X >= TEST_CFG.check_x_min) & (X <= TEST_CFG.check_x_max)]
  bm, cm = np.interp(xm, X, plan), np.interp(xm, X, center)
  y = bm + reconstructed_offsets(xm, delta)
  assert np.all(y >= np.minimum(bm, cm)) and np.all(y <= np.maximum(bm, cm))
  assert consumed_within_bounds(xm, bm, cm, delta)


def test_filter_residual_beyond_new_interval_is_detected():
  corr, delta = run()
  assert delta > 0.0
  corr.update(**inputs(center_offset=0.0))  # centre now on the path: interval {0}
  assert not corr.check_consumed(delta, 0.0)  # residual consumed delta overshoots


def test_blocked_cycle_reports_any_nonzero_consumed_delta():
  corr = LanelessCenterCorrection(TEST_CFG)
  corr.update(**inputs(rll_prob=0.1))
  assert corr.check_consumed(0.0, 0.0)
  assert not corr.check_consumed(1e-6, 0.0)


def test_center_bounded_target_and_uniform_shift_interval():
  b = np.array([0.0, 0.2, -0.1])
  c = np.array([0.3, 0.2, 0.4])
  y = center_bounded_target(b, c, 2.0)  # gain clipped to 1
  assert np.all(y >= np.minimum(b, c)) and np.all(y <= np.maximum(b, c))
  assert np.array_equal(center_bounded_target(b, c, 0.0), b)
  assert uniform_shift_interval(b, c) == (0.0, 0.0)  # middle point already centred
  assert uniform_shift_interval(np.zeros(2), np.array([0.3, 0.5])) == (0.0, 0.3)


def test_lane_center_requires_both_lines_same_positions_and_order():
  ok_x, c, reason = lane_center(X, X, X, -np.ones_like(X), np.ones_like(X), 0.9, 0.9, 0.1, 0.1, TEST_CFG)
  assert reason == "" and np.all(c == 0.0)
  _, _, reason = lane_center(X, X, X, np.ones_like(X), -np.ones_like(X), 0.9, 0.9, 0.1, 0.1, TEST_CFG)
  assert reason == BLOCK_LANES  # swapped lines
  bad = np.ones_like(X)
  bad[3] = np.nan
  _, _, reason = lane_center(X, X, X, -bad, bad, 0.9, 0.9, 0.1, 0.1, TEST_CFG)
  assert reason == BLOCK_LANES


def test_test_config_not_leaked_into_module():
  assert lc.APPROVED_CONFIG is None
  assert all(v is None for v in vars(LanelessCenterConfig()).values())
