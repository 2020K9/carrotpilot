"""Laneless lane-centre correction routed into the CONSUMED curvature (disabled).

lane_planner_2 already has a laneless centre adjust (``use_laneless_center_adjust``)
but it only edits the lateral *plan*: in laneless driving controlsd consumes
``modelV2.action.desiredCurvature`` (controlsd.py ``laneless_curvature``), so a
plan-only edit never reaches the steering command.  This module instead produces
a curvature *delta* that controlsd adds to the laneless candidate after the
PathVerifier and before the lane/laneless blend, and checks that the curvature
actually consumed after smoothing and clip_curvature still keeps every checked
point between the original model path and the lane centre.

Nothing here is approved for driving.  Every threshold lives in
``LanelessCenterConfig`` with ``None`` defaults and ``APPROVED_CONFIG`` is None,
so ``LanelessCenterCorrection().enabled`` is False and controlsd keeps its
original code path bit for bit.  Values used by the tests are synthetic.

Geometry contract (model frame: x forward, y RIGHT positive, curvature > 0 ==
turning right, same as path_verifier):

  * b_i  = modelV2.position.y interpolated at lane-line x_i (same model frame,
           same longitudinal position x_i, same message)
  * c_i  = (laneLines[1].y + laneLines[2].y) / 2 at x_i, only when BOTH lines are
           trusted, share the same x samples and are ordered (left < right)
  * a curvature delta dk is reconstructed as dy_i = dk * x_i^2 / 2: small angle,
           constant dk over the checked horizon, same initial position and
           heading as the model path.  This is a reconstruction assumption, not a
           claim that one curvature determines the whole model path.
  * per point: min(b_i, c_i) <= b_i + dy_i <= max(b_i, c_i).  Each point allows
           dk in [min(d_i, 0), max(d_i, 0)] * 2 / x_i^2 with d_i = c_i - b_i; the
           usable dk is the intersection over all checked points.  Opposite
           directions or a point already on the centre leave only dk = 0.
           Between checked samples b and c are linearly interpolated on a
           densified grid and the same bound is applied there too; this is a
           check of the interpolation, not of the physical path.
  * the consumed check compares the final clipped curvature with a shadow chain
           run without the correction from its own state; a violation makes
           controlsd recompute that cycle without the correction.  It cannot
           remove correction already stored in the smoothing state.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Any, Callable, Dict, Optional, Sequence, Tuple

import numpy as np

from openpilot.selfdrive.controls.lib.lat_mode_blend import blend_lat_mode
from openpilot.selfdrive.controls.lib.path_verifier import _poly_curvature, _poly_fit

BLOCK_DISABLED = "disabled"
BLOCK_INACTIVE = "lat_inactive"
BLOCK_STEERING_PRESSED = "steering_pressed"
BLOCK_LANE_CHANGE = "lane_change"
BLOCK_LANE_MODE = "lane_mode_weight"
BLOCK_MODEL_STALE = "model_stale"
BLOCK_MISSING = "missing_input"
BLOCK_LANES = "lanes_untrusted"
BLOCK_MISMATCH = "path_direct_mismatch"
BLOCK_PATH_VERIFIER = "path_verifier_active"
BLOCK_NO_INTERVAL = "no_common_interval"
BLOCK_CONSUMED_VIOLATION = "consumed_violation"

DENSIFY = 4  # sub-samples per checked segment for the between-points check


@dataclass(frozen=True)
class LanelessCenterConfig:
  # All unapproved: None keeps the feature off.
  min_lane_prob: Optional[float] = None       # both lane lines must reach this probability
  max_lane_std: Optional[float] = None        # m, both lane line stds must stay below this
  check_x_min: Optional[float] = None         # m, nearest checked lane-line sample
  check_x_max: Optional[float] = None         # m, farthest checked lane-line sample
  eval_x: Optional[float] = None              # m, where the requested correction is measured
  gain: Optional[float] = None                # 0..1, fraction of (c - b) requested at eval_x
  max_path_direct_mismatch: Optional[float] = None  # 1/m, |plan curvature - desiredCurvature| allowed
  max_model_age: Optional[float] = None       # s, controlsd receive age of modelV2
  max_delta_rate: Optional[float] = None      # 1/m/s, rate limit of the correction (build and release)
  max_lane_mode_weight: Optional[float] = None  # lat_mode_blend above this blocks (lane mode is B1's job)

  def approved(self) -> bool:
    return all(getattr(self, f.name) is not None for f in fields(self))


# No value has been approved; replacing this is a separate approval, not a test.
APPROVED_CONFIG: Optional[LanelessCenterConfig] = None


def _arr(v: Optional[Sequence[float]]) -> np.ndarray:
  if v is None:
    return np.zeros(0)
  return np.asarray(v, dtype=np.float64).ravel()


def lane_center(lane_x: Sequence[float], lll_x: Sequence[float], rll_x: Sequence[float],
                lll_y: Sequence[float], rll_y: Sequence[float],
                lll_prob: float, rll_prob: float, lll_std: float, rll_std: float,
                cfg: LanelessCenterConfig) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], str]:
  """Both-line centre at shared x samples. lane_path_y in lane_planner_2 may be a
  single line or a probability weighted path, so it is deliberately not used."""
  x, lx, rx = _arr(lane_x), _arr(lll_x), _arr(rll_x)
  ly, ry = _arr(lll_y), _arr(rll_y)
  n = x.size
  if n < 2 or lx.size != n or rx.size != n or ly.size != n or ry.size != n:
    return None, None, BLOCK_MISSING
  if not (np.array_equal(lx, x) and np.array_equal(rx, x)):
    return None, None, BLOCK_LANES  # not the same longitudinal positions
  vals = (lll_prob, rll_prob, lll_std, rll_std)
  if not all(math.isfinite(v) for v in vals):
    return None, None, BLOCK_LANES
  if min(lll_prob, rll_prob) < cfg.min_lane_prob or max(lll_std, rll_std) > cfg.max_lane_std:
    return None, None, BLOCK_LANES
  if not (np.all(np.isfinite(x)) and np.all(np.isfinite(ly)) and np.all(np.isfinite(ry))):
    return None, None, BLOCK_LANES
  if np.any(np.diff(x) <= 0.0) or np.any(ry <= ly):
    return None, None, BLOCK_LANES
  return x, 0.5 * (ly + ry), ""


def center_bounded_target(b: np.ndarray, c: np.ndarray, gain: float) -> np.ndarray:
  """Per-point target y = b + g (c - b), g in [0, 1]: always between b and c."""
  g = float(min(max(gain, 0.0), 1.0))
  return b + g * (c - b)


def within_center_bounds(b: np.ndarray, c: np.ndarray, y: np.ndarray, eps: float = 0.0) -> bool:
  lo = np.minimum(b, c) - eps
  hi = np.maximum(b, c) + eps
  return bool(np.all(np.isfinite(y)) and np.all(y >= lo) and np.all(y <= hi))


def uniform_shift_interval(b: np.ndarray, c: np.ndarray) -> Tuple[float, float]:
  """Single lateral shift allowed by every point: intersection of [min(d,0), max(d,0)]."""
  d = c - b
  return float(np.max(np.minimum(d, 0.0))), float(np.min(np.maximum(d, 0.0)))


def _densify(x: np.ndarray, *ys: np.ndarray) -> Tuple[np.ndarray, ...]:
  if x.size < 2:
    return (x,) + ys
  xs = np.concatenate([np.linspace(x[i], x[i + 1], DENSIFY, endpoint=False) for i in range(x.size - 1)] + [x[-1:]])
  return (xs,) + tuple(np.interp(xs, x, y) for y in ys)


def curvature_delta_interval(x: np.ndarray, b: np.ndarray, c: np.ndarray) -> Tuple[float, float]:
  """dk allowed so that b + dk x^2/2 stays between b and c at every checked and
  interpolated point (see module docstring)."""
  xs, bs, cs = _densify(x, b, c)
  k = 2.0 / np.maximum(xs * xs, 1e-6)
  d = cs - bs
  return float(np.max(np.minimum(d, 0.0) * k)), float(np.min(np.maximum(d, 0.0) * k))


def reconstructed_offsets(x: np.ndarray, dk: float) -> np.ndarray:
  return dk * x * x / 2.0


def consumed_within_bounds(x: np.ndarray, b: np.ndarray, c: np.ndarray, dk: float) -> bool:
  xs, bs, cs = _densify(x, b, c)
  return within_center_bounds(bs, cs, bs + reconstructed_offsets(xs, dk))


def consume_curvature(weight: float, lane_curvature: Optional[float], laneless_curvature: float,
                      delta: float, prev_desired: float, lat_smooth_seconds: float,
                      v_ego: float, roll: float, dt: float,
                      clip_fn: Callable[[float, float, float, float], Tuple[float, bool]]) -> Tuple[float, bool, Dict[str, float]]:
  """controlsd's K9/general lateral chain (blend -> smooth -> clip) with the
  correction added to the laneless candidate only.  delta = 0 reproduces the
  original expressions exactly."""
  corrected = laneless_curvature + delta
  lane_target = laneless_curvature if lane_curvature is None else lane_curvature
  blended = blend_lat_mode(weight, lane_target, corrected)
  tau = blend_lat_mode(weight, lat_smooth_seconds, 0.1)
  alpha = 1 - np.exp(-dt / tau) if tau > 0 else 1
  smoothed = alpha * blended + (1 - alpha) * prev_desired
  desired, limited = clip_fn(v_ego, prev_desired, smoothed, roll)
  return desired, limited, {"corrected": float(corrected), "blended": float(blended),
                            "smoothed": float(smoothed), "clipped": float(desired)}


class LanelessCenterCorrection:
  """Stateful wrapper: requested delta, rate limit, and the consumed-value check."""

  def __init__(self, cfg: Optional[LanelessCenterConfig] = APPROVED_CONFIG) -> None:
    self.cfg = cfg
    self.enabled = cfg is not None and cfg.approved()
    self.reset()

  def reset(self) -> None:
    self.delta = 0.0
    self.shadow_desired: Optional[float] = None
    self.geometry: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]] = None
    self.trace: Dict[str, Any] = {"reason": BLOCK_DISABLED if not self.enabled else BLOCK_INACTIVE}

  def update(self, *, dt: float, lat_active: bool, steering_pressed: bool, lane_change_active: bool,
             lane_mode_weight: float, model_age: float, model_valid: bool, model_curvature: float,
             path_verifier_effect: float,
             plan_x: Sequence[float], plan_y: Sequence[float],
             lane_x: Sequence[float], lll_x: Sequence[float], rll_x: Sequence[float],
             lll_y: Sequence[float], rll_y: Sequence[float],
             lll_prob: float, rll_prob: float, lll_std: float, rll_std: float) -> float:
    """Returns the curvature delta (1/m) to add to the laneless candidate."""
    trace: Dict[str, Any] = {"reason": "", "request": 0.0, "interval": None, "delta": 0.0}
    self.trace = trace
    if not self.enabled:
      trace["reason"] = BLOCK_DISABLED
      self.reset()
      return 0.0
    cfg = self.cfg
    assert cfg is not None
    if not lat_active:
      trace["reason"] = BLOCK_INACTIVE
      self.reset()
      self.trace = trace
      return 0.0

    request, reason = 0.0, ""
    geometry = None
    if steering_pressed:
      reason = BLOCK_STEERING_PRESSED
    elif lane_change_active:
      reason = BLOCK_LANE_CHANGE
    elif not (math.isfinite(lane_mode_weight) and lane_mode_weight <= cfg.max_lane_mode_weight):
      reason = BLOCK_LANE_MODE
    elif not model_valid or not math.isfinite(model_age) or model_age < 0.0 or model_age > cfg.max_model_age:
      reason = BLOCK_MODEL_STALE
    elif not math.isfinite(model_curvature):
      reason = BLOCK_MISSING
    elif path_verifier_effect != 0.0:
      # The verifier is holding the model back; adding a correction on top could
      # undo its hold, so the candidate is blocked rather than layered over it.
      reason = BLOCK_PATH_VERIFIER
    else:
      x, c, reason = lane_center(lane_x, lll_x, rll_x, lll_y, rll_y, lll_prob, rll_prob, lll_std, rll_std, cfg)
      px, py = _arr(plan_x), _arr(plan_y)
      if not reason and (px.size < 2 or px.size != py.size or not np.all(np.isfinite(px))
                         or not np.all(np.isfinite(py)) or np.any(np.diff(px) <= 0.0)):
        reason = BLOCK_MISSING
      if not reason:
        m = (x >= cfg.check_x_min) & (x <= cfg.check_x_max) & (x >= px[0]) & (x <= px[-1])
        if int(m.sum()) < 2 or not (cfg.check_x_min <= cfg.eval_x <= cfg.check_x_max):
          reason = BLOCK_MISSING
      if not reason:
        xm, cm = x[m], c[m]
        bm = np.interp(xm, px, py)
        plan_k = _poly_curvature(_poly_fit(px, py, 0.0, cfg.check_x_max), cfg.eval_x)
        if plan_k is None or abs(plan_k - model_curvature) > cfg.max_path_direct_mismatch:
          reason = BLOCK_MISMATCH
        else:
          lo, hi = curvature_delta_interval(xm, bm, cm)
          trace["interval"] = (lo, hi)
          b_e = float(np.interp(cfg.eval_x, xm, bm))
          c_e = float(np.interp(cfg.eval_x, xm, cm))
          target = center_bounded_target(np.array([b_e]), np.array([c_e]), cfg.gain)[0]
          wanted = 2.0 * (target - b_e) / max(cfg.eval_x * cfg.eval_x, 1e-6)
          request = float(min(max(wanted, lo), hi))
          if lo == 0.0 and hi == 0.0 and wanted != 0.0:
            reason = BLOCK_NO_INTERVAL
          geometry = (xm, bm, cm)

    trace["request"] = request
    # A blocked cycle releases at the same (unapproved) rate; nothing is held.
    goal = 0.0 if reason else request
    max_step = cfg.max_delta_rate * dt
    step = float(min(max(goal - self.delta, -max_step), max_step))
    new_delta = self.delta + step
    # A ramp from an older interval must still satisfy the current one.
    if geometry is not None and not consumed_within_bounds(geometry[0], geometry[1], geometry[2], new_delta):
      new_delta = float(min(max(new_delta, trace["interval"][0]), trace["interval"][1]))
    self.delta = new_delta
    self.geometry = geometry if not reason else None
    trace["reason"] = reason
    trace["delta"] = self.delta
    return self.delta

  def check_consumed(self, desired: float, shadow_desired: float) -> bool:
    """True when the consumed delta (actual minus the no-correction shadow chain)
    keeps every checked point between b and c.  No geometry == no new correction
    claim, which only holds when the consumed delta is exactly 0."""
    dk = desired - shadow_desired
    self.trace["consumed_delta"] = dk
    if self.geometry is None:
      ok = dk == 0.0
    else:
      ok = consumed_within_bounds(self.geometry[0], self.geometry[1], self.geometry[2], dk)
    self.trace["consumed_ok"] = ok
    return ok
