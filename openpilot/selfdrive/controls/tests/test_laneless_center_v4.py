# Written for the lanebias/corner v4 review; NOT executed (test runs await separate approval).
# Every numeric value below is an UNAPPROVED DEFAULT -- SYNTHETIC TEST ONLY, not a setting.
import dataclasses
import inspect
import math
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import laneless_center as lc
from openpilot.selfdrive.controls.lib.drive_helpers import clip_curvature
from openpilot.selfdrive.controls.lib.lat_mode_blend import blend_lat_mode
from openpilot.selfdrive.controls.lib.laneless_center import (
  APPROVED_CONFIG, APPROVED_SAFETY, BLOCK_CANDIDATE_CHECK, BLOCK_CONFIG_INVALID, BLOCK_UNRESOLVED,
  RESIDUAL_POLICY_APPROVED, UNRESOLVED_HANDOVER, UNRESOLVED_RECOMPUTE, UNRESOLVED_RELEASE, CandidateSafety, ClearanceObservation,
  LanelessCenterConfig, LanelessCenterCorrection, SideClearance, _densify, consumed_within_bounds,
  reconstructed_offsets)

DT = 0.01
V = 20.0
X = np.linspace(0.0, 60.0, 33)
WIDTH = 3.6
TEST_CFG = LanelessCenterConfig(min_lane_prob=0.5, max_lane_std=0.3, check_x_min=5.0, check_x_max=40.0,
                                eval_x=20.0, gain=0.5, max_path_direct_mismatch=0.01, max_model_age=0.2,
                                max_delta_rate=1.0, max_lane_mode_weight=0.0)


def side(**kw):
  base = dict(supported=True, valid=True, age=0.0, range_x=100.0, detected=False, approaching=False, free_lateral=2.0)
  base.update(kw)
  return SideClearance(**base)


CLEAR = ClearanceObservation(left=side(), right=side())


def safety(check=lambda x, y: True, **kw):
  base = dict(candidate_check=check, max_obs_age=0.2, required_range_x=40.0, lateral_margin=0.3)
  base.update(kw)
  return CandidateSafety(**base)


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


def old_chain(weight, lane_curvature, laneless_curvature, prev, lat_smooth):
  # Pre-change controlsd default-branch expressions.
  lane_target = laneless_curvature if lane_curvature is None else lane_curvature
  curvature = blend_lat_mode(weight, lane_target, laneless_curvature)
  tau = blend_lat_mode(weight, lat_smooth, 0.1)
  alpha = 1 - np.exp(-DT / tau) if tau > 0 else 1
  return clip_curvature(V, prev, alpha * curvature + (1 - alpha) * prev, 0.0)


def drive(corr, cycles, prev=0.0, clip_fn=clip_curvature, **kw):
  """update -> consume, carrying the consumed state like controlsd does."""
  for _ in range(cycles):
    corr.update(**inputs(**kw))
    prev, _ = corr.consume(0.0, None, 0.0, prev, 0.2, V, 0.0, DT, clip_fn)
  return prev


# ------------------------------------------------------------ §5 config numeric validity

FIELDS = [f.name for f in dataclasses.fields(LanelessCenterConfig)]
GENERIC_BAD = [None, "0.5", True, False, float("nan"), float("inf"), float("-inf"), -1.0, [0.5]]


@pytest.mark.parametrize("name", FIELDS)
@pytest.mark.parametrize("bad", GENERIC_BAD)
def test_every_field_rejects_unset_nonnumeric_bool_nonfinite_negative(name, bad):
  cfg = dataclasses.replace(TEST_CFG, **{name: bad})
  assert name in cfg.errors()
  assert not cfg.approved()
  assert not LanelessCenterCorrection(cfg).enabled


@pytest.mark.parametrize("name,bad", [
  ("min_lane_prob", 1.01), ("gain", 1.5), ("max_lane_mode_weight", 1.1),
  ("max_lane_std", 0.0), ("max_model_age", 0.0), ("max_delta_rate", 0.0), ("check_x_min", 0.0),
  ("check_x_max", 5.0), ("check_x_max", 4.0),  # equal / reversed order
  ("eval_x", 4.9), ("eval_x", 40.1),             # outside the checked range
])
def test_field_specific_ranges_rejected(name, bad):
  cfg = dataclasses.replace(TEST_CFG, **{name: bad})
  assert name in cfg.errors() and not cfg.approved()


@pytest.mark.parametrize("name,ok", [
  ("gain", 0.0), ("gain", 1.0), ("min_lane_prob", 0.0), ("min_lane_prob", 1.0),
  ("eval_x", 5.0), ("eval_x", 40.0), ("max_path_direct_mismatch", 0.0), ("max_lane_mode_weight", 1.0),
  ("gain", np.float64(0.5)), ("check_x_max", 40),
])
def test_mathematical_boundaries_are_numerically_valid(name, ok):
  assert dataclasses.replace(TEST_CFG, **{name: ok}).approved()


def test_valid_numbers_are_not_human_approval():
  assert APPROVED_CONFIG is None and APPROVED_SAFETY is None and RESIDUAL_POLICY_APPROVED is False
  corr = LanelessCenterCorrection(TEST_CFG, safety())
  assert corr.enabled and corr.safety is not None
  assert not corr.operational  # residual policy undecided
  assert not LanelessCenterCorrection().operational
  assert not LanelessCenterCorrection(TEST_CFG).operational  # no safety contract


def test_config_mutated_at_runtime_is_rejected_next_cycle():
  cfg = dataclasses.replace(TEST_CFG)
  corr = LanelessCenterCorrection(cfg)
  for _ in range(50):
    corr.update(**inputs())
  assert corr.delta > 0.0
  object.__setattr__(cfg, "gain", float("nan"))
  assert corr.update(**inputs()) == 0.0
  assert corr.trace["reason"] == BLOCK_CONFIG_INVALID and "gain" in corr.trace["errors"]
  assert corr.delta == 0.0 and not corr.enabled


def test_invalid_safety_contract_is_not_connected():
  for kw in ({"lateral_margin": float("nan")}, {"max_obs_age": 0.0}, {"required_range_x": -1.0},
             {"lateral_margin": True}, {"check": None}):
    assert LanelessCenterCorrection(TEST_CFG, safety(**kw)).safety is None


# ------------------------------------------------------------ §3 consumed state / recompute

def test_normal_intersection_passes_first_check_and_stays_in_bounds():
  corr = LanelessCenterCorrection(TEST_CFG)
  drive(corr, 200)
  t = corr.trace
  assert t["final_status"] == "candidate" and t["first_ok"] and "recomputed" not in t
  assert t["final"] == t["first"]
  assert t["residual"] > 0.0
  assert consumed_within_bounds(*corr.geometry, t["residual"])
  assert corr.unresolved is None


@pytest.mark.parametrize("center_offset", [0.0, -0.5])  # interval collapses to {0} / reverses
def test_residual_after_interval_change_is_rechecked_and_not_reported_valid(center_offset):
  corr = LanelessCenterCorrection(TEST_CFG)
  prev = drive(corr, 200)
  assert corr.last_residual > 0.0
  drive(corr, 1, prev=prev, center_offset=center_offset)
  t = corr.trace
  if center_offset == 0.0:
    assert t["first_delta"] == 0.0  # zero delta ...
  assert not t["first_ok"]          # ... is not "no correction"
  assert corr.delta == 0.0          # violation drops the delta before the recomputation
  assert "recomputed" in t and not t["recomputed_ok"]   # the recomputation is checked again
  assert t["final"] == t["recomputed"] and t["final_status"] == "unresolved"
  assert corr.unresolved == UNRESOLVED_RECOMPUTE and not corr.operational
  assert corr.update(**inputs()) == 0.0 and corr.trace["reason"] == BLOCK_UNRESOLVED


@pytest.mark.parametrize("override", [
  {"steering_pressed": True}, {"lane_change_active": True}, {"model_valid": False},
  {"rll_prob": 0.1}, {"lane_mode_weight": 0.5}, {"model_age": float("nan")},
])
def test_block_with_residual_in_state_is_flagged_not_silently_emitted(override):
  corr = LanelessCenterCorrection(TEST_CFG)
  prev = drive(corr, 200)
  drive(corr, 1, prev=prev, **override)
  t = corr.trace
  assert t["residual"] != 0.0
  assert not t["first_ok"] and not t["recomputed_ok"]
  assert t["final_status"] == "unresolved" and corr.unresolved == UNRESOLVED_RECOMPUTE


def test_setting_cancel_mid_run_is_flagged():
  cfg = dataclasses.replace(TEST_CFG)
  corr = LanelessCenterCorrection(cfg)
  prev = drive(corr, 200)
  assert corr.last_residual != 0.0
  object.__setattr__(cfg, "max_delta_rate", -1.0)
  assert corr.update(**inputs()) == 0.0
  assert corr.trace["reason"] == BLOCK_CONFIG_INVALID
  # The consumed state still holds the residual: mid-run stop is not "same as off".
  assert corr.unresolved == UNRESOLVED_HANDOVER and not corr.operational
  assert not corr.baseline_independent


def test_curvature_clip_saturation_is_limited_not_violation():
  saturated = lambda v, prev, new, roll: (prev, True)  # noqa: E731
  corr = LanelessCenterCorrection(TEST_CFG)
  out = drive(corr, 5, prev=0.001, clip_fn=saturated)
  assert out == 0.001 and corr.trace["curvature_limited"]
  assert corr.trace["first_ok"] and corr.trace["residual"] == 0.0


def test_reentry_without_baseline_marks_seed_not_independent():
  fresh = LanelessCenterCorrection(TEST_CFG)
  drive(fresh, 1)
  assert fresh.trace["baseline_seed"] == "consumed_prev" and fresh.trace["baseline_independent"]
  corr = LanelessCenterCorrection(TEST_CFG)
  prev = drive(corr, 200)
  corr.update(**inputs(lat_active=False))  # reset with a residual in controlsd's state
  assert corr.baseline_desired is None and not corr.baseline_independent
  drive(corr, 1, prev=prev)
  assert corr.trace["baseline_seed"] == "consumed_prev" and not corr.trace["baseline_independent"]


def test_never_requesting_chain_is_bit_identical_to_original():
  corr = LanelessCenterCorrection(TEST_CFG)
  prev_new = prev_old = 0.0015
  for _ in range(100):
    corr.update(**inputs(center_offset=0.0))  # centre on the path: no correction ever requested
    prev_new, lim_new = corr.consume(0.0, None, 0.0, prev_new, 0.2, V, 0.0, DT, clip_curvature)
    prev_old, lim_old = old_chain(0.0, None, 0.0, prev_old, 0.2)
    assert prev_new == prev_old and lim_new == lim_old
  assert corr.unresolved is None


def test_off_from_start_and_stop_mid_run_are_different_cases():
  # Off from start: identical (above).  Stop mid-run: controlsd's state still carries
  # the residual; the default branch then continues from it -> not reported as identical.
  corr = LanelessCenterCorrection(TEST_CFG)
  prev = drive(corr, 200)
  baseline = corr.baseline_desired
  assert prev != baseline


# ------------------------------------------------------------ §4 candidate safety / clearance

def test_candidate_path_not_raw_model_path_is_checked():
  seen = []
  corr = LanelessCenterCorrection(TEST_CFG, safety(check=lambda x, y: seen.append((x.copy(), y.copy())) or True))
  for _ in range(100):
    corr.update(**inputs(clearance=CLEAR))
  assert corr.delta > 0.0 and seen
  x, y = seen[-1]
  xs, bs = _densify(*corr.geometry[:2])
  assert np.array_equal(x, xs)
  assert np.allclose(y, bs + reconstructed_offsets(xs, corr.delta))
  assert np.any(y != 0.0)  # the raw plan is all zero here


def test_candidate_only_invades_barrier_never_committed():
  barrier = 0.3  # synthetic obstacle 0.3 m right of the raw path; raw path itself is clear
  corr = LanelessCenterCorrection(TEST_CFG, safety(check=lambda x, y: bool(np.all(y < barrier))))
  blocked = False
  for _ in range(200):
    corr.update(**inputs(clearance=CLEAR))
    blocked |= corr.trace["reason"] == BLOCK_CANDIDATE_CHECK
    xs, bs = _densify(*corr.geometry[:2]) if corr.geometry is not None else _densify(X[3:22], np.zeros(19))
    assert np.all(bs + reconstructed_offsets(xs, corr.delta) < barrier)
  assert blocked


@pytest.mark.parametrize("bad,why", [
  (side(supported=False), "unsupported"), (side(valid=False), "invalid"), (side(age=1.0), "stale"),
  (side(age=float("nan")), "stale"), (side(age=-0.1), "stale"), (side(range_x=10.0), "short_range"),
  (side(range_x=None), "short_range"), (side(detected=True), "detected"),
  (side(approaching=True), "approaching"), (side(free_lateral=None), "no_free_evidence"),
  (side(free_lateral=float("inf")), "no_free_evidence"), (side(free_lateral=0.5), "insufficient_space"),
])
def test_move_side_not_proven_free_blocks_even_with_good_lanes_and_zero_verifier(bad, why):
  corr = LanelessCenterCorrection(TEST_CFG, safety())
  for _ in range(20):
    d = corr.update(**inputs(clearance=ClearanceObservation(left=side(), right=bad), path_verifier_effect=0.0))
    assert d == 0.0
  assert corr.trace["reason"] == f"clearance:right:{why}"
  assert corr.unresolved is None  # nothing was ever built, so nothing to release


def test_unobserved_clearance_blocks():
  corr = LanelessCenterCorrection(TEST_CFG, safety())
  assert corr.update(**inputs()) == 0.0
  assert corr.trace["reason"] == "clearance:right:unsupported"


def test_both_sides_occupied_blocks_either_direction():
  both = ClearanceObservation(left=side(detected=True), right=side(detected=True))
  for offset in (0.5, -0.5):
    corr = LanelessCenterCorrection(TEST_CFG, safety())
    assert corr.update(**inputs(center_offset=offset, clearance=both)) == 0.0


def test_left_right_symmetry_of_side_selection():
  right_blocked = ClearanceObservation(left=side(), right=side(detected=True))
  left_blocked = ClearanceObservation(left=side(detected=True), right=side())
  a = LanelessCenterCorrection(TEST_CFG, safety())
  b = LanelessCenterCorrection(TEST_CFG, safety())
  for _ in range(100):
    a.update(**inputs(center_offset=-0.5, clearance=right_blocked))  # moves left: right side irrelevant
    b.update(**inputs(center_offset=0.5, clearance=left_blocked))    # moves right: left side irrelevant
  assert a.delta < 0.0 and b.delta > 0.0
  assert a.delta == pytest.approx(-b.delta)


@pytest.mark.parametrize("check", [lambda x, y: False, lambda x, y: 1, lambda x, y: np.True_,
                                   lambda x, y: 1 / 0])
def test_verifier_authority_zero_is_not_a_pass(check):
  corr = LanelessCenterCorrection(TEST_CFG, safety(check=check))
  assert corr.update(**inputs(clearance=CLEAR, path_verifier_effect=0.0)) == 0.0
  assert corr.trace["reason"] == BLOCK_CANDIDATE_CHECK


def test_release_without_geometry_is_unresolved():
  corr = LanelessCenterCorrection(TEST_CFG, safety())
  for _ in range(100):
    corr.update(**inputs(clearance=CLEAR))
  built = corr.delta
  assert built > 0.0
  released = corr.update(**inputs(clearance=CLEAR, rll_prob=0.1))
  assert released < built  # existing release ramp is emitted, unchanged
  assert corr.trace["safety"] and corr.unresolved == UNRESOLVED_RELEASE
  assert not corr.operational


def test_return_only_dangerous_is_unresolved():
  corr = LanelessCenterCorrection(TEST_CFG, safety())
  for _ in range(100):
    corr.update(**inputs(clearance=CLEAR))
  assert corr.delta > 0.0
  # interval shrinks (moves back left) while the LEFT side is occupied; the band itself is right
  corr.update(**inputs(center_offset=0.2, clearance=ClearanceObservation(left=side(detected=True), right=side())))
  assert corr.trace["reason"].startswith("clearance:left:")
  assert corr.unresolved == UNRESOLVED_RELEASE


def test_consumed_value_rechecked_against_clearance():
  corr = LanelessCenterCorrection(TEST_CFG, safety())
  prev = 0.0
  for _ in range(100):
    corr.update(**inputs(clearance=CLEAR))
    prev, _ = corr.consume(0.0, None, 0.0, prev, 0.2, V, 0.0, DT, clip_curvature)
  assert corr.trace["first_ok"] and corr.trace["residual"] > 0.0
  corr.clearance = ClearanceObservation(left=side(), right=side(detected=True))
  assert not corr.check_consumed(corr.trace["final"], corr.trace["baseline"])
  assert corr.trace["consumed_safety"] == "clearance:right:detected"


def test_nonfinite_consumed_value_fails():
  corr = LanelessCenterCorrection(TEST_CFG)
  corr.update(**inputs())
  assert not corr.check_consumed(float("nan"), 0.0)
  assert not corr.check_consumed(float("inf"), 0.0)


def test_library_never_advances_path_verifier_state():
  src = inspect.getsource(lc.LanelessCenterCorrection)
  assert "PathVerifier" not in src and ".update(" not in src.replace("self.trace.update(", "").replace(
    "t.update(", "").replace("trace.update(", "")
  ctl = (Path(__file__).resolve().parents[1] / "controlsd.py").read_text()
  assert ctl.count("self.path_verifier.update(") == 1


# ------------------------------------------------------------ controlsd wiring (source + order)

def test_controlsd_gates_on_operational_and_delegates_consumption():
  src = (Path(__file__).resolve().parents[1] / "controlsd.py").read_text()
  assert "if self.laneless_center.operational:" in src
  assert "return lcc.consume(self.lat_mode_blend, lane_curvature, laneless_curvature, self.desired_curvature," in src
  assert "model_curvature=float(model_v2.action.desiredCurvature)," in src
  assert "model_raw=float(model_v2.action.desiredCurvature), verified=laneless_curvature," in src
  assert "clearance=None," in src  # no observation source connected: stays blocked
  # order: verifier -> delta -> consume -> actuators.curvature -> LaC
  idx = [src.index(s) for s in ("laneless_curvature = self.path_verifier_curvature(",
                                "laneless_center_delta = self.laneless_center_delta(",
                                "self.desired_curvature, curvature_limited = self.consume_laneless_center(",
                                "actuators.curvature = float(self.desired_curvature)",
                                "steer, steeringAngleDeg, lac_log = self.LaC.update(")]
  assert idx == sorted(idx)


def test_test_values_not_leaked():
  assert lc.APPROVED_CONFIG is None and lc.APPROVED_SAFETY is None and not lc.RESIDUAL_POLICY_APPROVED
  assert math.isfinite(TEST_CFG.gain)  # synthetic only
