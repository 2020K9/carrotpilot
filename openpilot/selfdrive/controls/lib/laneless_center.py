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
  * the consumed check compares the final clipped curvature with a baseline chain
           run without the correction from its own state; a violation makes
           ``consume`` recompute that cycle without the correction and CHECK THE
           RECOMPUTED VALUE AGAIN.  A zero delta is not "no correction": the
           smoothing/clip state still carries the residual.  When the recomputed
           value also fails, the emitted value is the pre-v4 recomputation, the
           cycle is marked unresolved and ``operational`` latches False; which
           value should be emitted then (hold / snap / baseline) is a human
           decision and is NOT chosen here.

State ownership (v4):

  * ``baseline_desired``: clean no-correction chain.  Seeded from controlsd's
           consumed ``desired_curvature`` on (re-)entry; if the previous run left a
           non-zero residual the seed is marked not independent.
  * ``delta``: requested correction; only committed after its checks.
  * controlsd ``desired_curvature``: the consumed state (owned by controlsd).

Activation (v4): ``enabled`` means "a complete, numerically valid config was
given" (computation possible), NOT human approval.  controlsd gates on
``operational``, which additionally needs a connected ``CandidateSafety``
contract and an approved residual/release policy; neither exists, so the
controlsd path stays off.
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
BLOCK_CONFIG_INVALID = "config_invalid"
BLOCK_UNRESOLVED = "unresolved_latched"
BLOCK_CANDIDATE_CHECK = "candidate_check_failed"
BLOCK_NO_GEOMETRY = "no_geometry_for_motion"
BLOCK_CLEARANCE = "clearance"  # prefix; the side/reason follow, e.g. "clearance:right:stale"

UNRESOLVED_RELEASE = "release_unverified"
UNRESOLVED_RECOMPUTE = "recompute_failed"
UNRESOLVED_HANDOVER = "handover_with_residual"

DENSIFY = 4  # sub-samples per checked segment for the between-points check

# Human decision required: what to emit when neither the candidate nor the
# zero-delta recomputation passes (hold / snap / baseline / ramp).  Until it is
# decided, ``operational`` is False.
RESIDUAL_POLICY_APPROVED = False


def _real(v: Any) -> bool:
  """Finite real number; bool, strings and None are not numbers here."""
  if isinstance(v, (bool, np.bool_)) or not isinstance(v, (int, float, np.integer, np.floating)):
    return False
  return math.isfinite(float(v))


# Numeric validity only (mathematical/physical meaning), not operational limits.
# field -> (check, reason).  Cross-field checks see the whole config.
_CONFIG_RULES: Dict[str, Tuple[Callable[[Any, Any], bool], str]] = {
  "min_lane_prob": (lambda v, c: 0.0 <= v <= 1.0, "probability outside [0, 1]"),
  "max_lane_std": (lambda v, c: v > 0.0, "std limit must be > 0"),
  "check_x_min": (lambda v, c: v > 0.0, "x_min must be > 0 (dk = 2 dy / x^2)"),
  "check_x_max": (lambda v, c: _real(c.check_x_min) and v > c.check_x_min, "x_max must exceed x_min"),
  "eval_x": (lambda v, c: _real(c.check_x_min) and _real(c.check_x_max) and c.check_x_min <= v <= c.check_x_max,
             "eval_x outside [x_min, x_max]"),
  "gain": (lambda v, c: 0.0 <= v <= 1.0, "gain outside [0, 1]"),
  "max_path_direct_mismatch": (lambda v, c: v >= 0.0, "mismatch limit must be >= 0"),
  "max_model_age": (lambda v, c: v > 0.0, "age limit must be > 0"),
  "max_delta_rate": (lambda v, c: v > 0.0, "rate must be > 0"),
  "max_lane_mode_weight": (lambda v, c: 0.0 <= v <= 1.0, "weight outside [0, 1]"),
}


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

  def complete(self) -> bool:
    return all(getattr(self, f.name) is not None for f in fields(self))

  def errors(self) -> Dict[str, str]:
    """field -> reason for every missing or numerically invalid value."""
    out: Dict[str, str] = {}
    for f in fields(self):
      v = getattr(self, f.name)
      if v is None:
        out[f.name] = "unset"
      elif not _real(v):
        out[f.name] = "not a finite real number"
      else:
        check, why = _CONFIG_RULES[f.name]
        if not check(float(v), self):
          out[f.name] = why
    return out

  def approved(self) -> bool:
    # Name kept for compatibility: complete AND numerically valid.  Human
    # approval is APPROVED_CONFIG, which is separate and still None.
    return not self.errors()


# No value has been approved; replacing this is a separate approval, not a test.
APPROVED_CONFIG: Optional[LanelessCenterConfig] = None


@dataclass(frozen=True)
class SideClearance:
  """Observation of one side (left = y < 0) supplied by the caller.  Nothing in
  controlsd produces this today; see REVIEW.md (unconnected)."""
  supported: bool = False         # the platform can observe this side at all
  valid: bool = False             # the source reports valid now
  age: Optional[float] = None     # s since the observation
  range_x: Optional[float] = None  # m ahead actually covered
  detected: bool = True           # vehicle / obstacle / BSD / road edge inside the band
  approaching: bool = True        # an object is entering the band during the motion
  free_lateral: Optional[float] = None  # m of OBSERVED free space beyond the current path; None = no evidence


@dataclass(frozen=True)
class ClearanceObservation:
  left: SideClearance
  right: SideClearance


@dataclass(frozen=True)
class CandidateSafety:
  """Contract the correction must satisfy before any delta is committed.

  candidate_check(x, y) receives the CANDIDATE path (model path plus the
  reconstructed correction) in the model frame and must be stateless: it must
  not advance PathVerifier's odometry/trails (that would run them twice per
  cycle).  Return exactly True to pass.  No implementation exists; values are
  unapproved and only synthetic tests construct one."""
  candidate_check: Callable[[np.ndarray, np.ndarray], bool]
  max_obs_age: float        # s
  required_range_x: float   # m ahead the observation must cover
  lateral_margin: float     # m added to the displacement (body corners + approved margin: human decision)

  def errors(self) -> Dict[str, str]:
    out: Dict[str, str] = {}
    if not callable(self.candidate_check):
      out["candidate_check"] = "not callable"
    for name, ok in (("max_obs_age", lambda v: v > 0.0), ("required_range_x", lambda v: v > 0.0),
                     ("lateral_margin", lambda v: v >= 0.0)):
      v = getattr(self, name)
      if not _real(v) or not ok(float(v)):
        out[name] = "missing, non-finite or out of range"
    return out


# No safety contract is connected in controlsd; replacing this is a separate approval.
APPROVED_SAFETY: Optional[CandidateSafety] = None


def side_clearance_reason(side: Optional[SideClearance], need_lateral: float, safety: CandidateSafety) -> str:
  """'' only with positive evidence of free space; everything unknown blocks."""
  if side is None or not side.supported:
    return "unsupported"
  if not side.valid:
    return "invalid"
  if not _real(side.age) or side.age < 0.0 or side.age > safety.max_obs_age:
    return "stale"
  if not _real(side.range_x) or side.range_x < safety.required_range_x:
    return "short_range"
  if side.detected:
    return "detected"
  if side.approaching:
    return "approaching"
  if not _real(side.free_lateral):
    return "no_free_evidence"
  if side.free_lateral < need_lateral + safety.lateral_margin:
    return "insufficient_space"
  return ""


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
  lo = float(np.max(np.minimum(d, 0.0) * k))
  hi = float(np.min(np.maximum(d, 0.0) * k))
  return _inside_bounds(xs, bs, cs, lo), _inside_bounds(xs, bs, cs, hi)


def _inside_bounds(xs: np.ndarray, bs: np.ndarray, cs: np.ndarray, dk: float) -> float:
  # d*2/x^2 reconstructed as dk*x^2/2 can round one ulp past c; step toward 0
  # until the exact reconstruction is inside, falling back to 0 (always inside).
  for _ in range(64):
    if dk == 0.0 or within_center_bounds(bs, cs, bs + reconstructed_offsets(xs, dk)):
      return dk
    dk = float(np.nextafter(dk, 0.0))
  return 0.0


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

  def __init__(self, cfg: Optional[LanelessCenterConfig] = APPROVED_CONFIG,
               safety: Optional[CandidateSafety] = APPROVED_SAFETY) -> None:
    self.cfg = cfg
    self.enabled = cfg is not None and cfg.approved()
    self.safety = safety if safety is not None and not safety.errors() else None
    self.unresolved: Optional[str] = None  # latched for the object's lifetime
    self.last_residual = 0.0
    self.baseline_independent = True
    self.reset()

  @property
  def operational(self) -> bool:
    """What controlsd gates on.  False until a safety contract is connected, the
    residual policy is approved and nothing unresolved has happened."""
    return (self.enabled and self.safety is not None and RESIDUAL_POLICY_APPROVED
            and self.unresolved is None and self.cfg is not None and self.cfg.approved())

  def reset(self) -> None:
    # A non-zero residual left in controlsd's consumed state makes the next
    # baseline seed (that state) not independent of the correction.
    # Conservative: once a seed was not independent it is never reported independent again.
    self.baseline_independent = (getattr(self, "baseline_independent", True) and self.last_residual == 0.0
                                 and getattr(self, "delta", 0.0) == 0.0)
    self.last_residual = 0.0  # the re-seeded baseline equals the consumed state
    self.delta = 0.0
    self.baseline_desired: Optional[float] = None
    self.geometry: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]] = None
    self.clearance: Optional[ClearanceObservation] = None
    self.trace: Dict[str, Any] = {"reason": BLOCK_DISABLED if not self.enabled else BLOCK_INACTIVE}

  # Pre-v4 name kept for callers/tests that read it.
  @property
  def shadow_desired(self) -> Optional[float]:
    return self.baseline_desired

  def _offsets_need(self, xs: np.ndarray, dk: float) -> float:
    reach = xs if self.safety is None else np.append(xs, self.safety.required_range_x)
    return float(np.max(np.abs(reconstructed_offsets(reach, dk)))) if reach.size else 0.0

  def safety_reason(self, dk_new: float, dk_old: float,
                    geometry: Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]],
                    clearance: Optional[ClearanceObservation]) -> str:
    """Checks the candidate path and the space for the occupied band AND for the
    motion from dk_old to dk_new (build, release and residual alike).  ''
    passes.  Without a safety contract this is spec/computation mode and returns
    '' (``operational`` is False then)."""
    if self.safety is None:
      return ""
    if not (math.isfinite(dk_new) and math.isfinite(dk_old)):
      return BLOCK_MISSING
    if dk_new == 0.0 and dk_old == 0.0:
      return ""
    if geometry is None:
      return BLOCK_NO_GEOMETRY
    xm, bm, _ = geometry
    xs, bs = _densify(xm, bm)
    try:
      ok = self.safety.candidate_check(xs, bs + reconstructed_offsets(xs, dk_new)) is True
    except Exception:
      ok = False
    if not ok:
      return BLOCK_CANDIDATE_CHECK
    need = {"left": 0.0, "right": 0.0}
    for dk in (dk_new, dk_new - dk_old):  # occupied band, then motion
      if dk != 0.0:
        side = "right" if dk > 0.0 else "left"  # y right positive, dk > 0 moves right
        need[side] = max(need[side], self._offsets_need(xs, dk))
    for side in ("left", "right"):
      if need[side] > 0.0:
        obs = None if clearance is None else getattr(clearance, side)
        why = side_clearance_reason(obs, need[side], self.safety)
        if why:
          return f"{BLOCK_CLEARANCE}:{side}:{why}"
    return ""

  def update(self, *, dt: float, lat_active: bool, steering_pressed: bool, lane_change_active: bool,
             lane_mode_weight: float, model_age: float, model_valid: bool, model_curvature: float,
             path_verifier_effect: float,
             plan_x: Sequence[float], plan_y: Sequence[float],
             lane_x: Sequence[float], lll_x: Sequence[float], rll_x: Sequence[float],
             lll_y: Sequence[float], rll_y: Sequence[float],
             lll_prob: float, rll_prob: float, lll_std: float, rll_std: float,
             clearance: Optional[ClearanceObservation] = None) -> float:
    """Returns the curvature delta (1/m) to add to the laneless candidate.
    model_curvature must be the RAW model desiredCurvature (path/direct check)."""
    trace: Dict[str, Any] = {"reason": "", "request": 0.0, "interval": None, "delta": 0.0}
    self.trace = trace
    if not self.enabled:
      trace["reason"] = BLOCK_DISABLED
      self.reset()
      return 0.0
    cfg = self.cfg
    assert cfg is not None
    if not cfg.approved():  # re-validated every cycle: a mutated/reused config is not trusted
      if self.delta != 0.0 or self.last_residual != 0.0:
        # Mid-run stop with correction still in the consumed state: the handover
        # to the default branch is not verified (off-from-start identity does not cover it).
        self.unresolved = UNRESOLVED_HANDOVER
      self.enabled = False
      self.reset()
      self.trace = {"reason": BLOCK_CONFIG_INVALID, "errors": cfg.errors(), "delta": 0.0}
      return 0.0
    if self.unresolved is not None:
      self.reset()
      self.trace = {"reason": BLOCK_UNRESOLVED, "unresolved": self.unresolved, "delta": 0.0}
      return 0.0
    self.clearance = clearance
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
        # The float midpoint (and lines built as centre -/+ w/2) can sit an ulp
        # beyond the real centre; pull it toward the path by that bound, never past.
        tol = 4.0 * np.finfo(float).eps * (np.abs(_arr(lll_y)[m]) + np.abs(_arr(rll_y)[m]))
        d = cm - bm
        cm = bm + np.sign(d) * np.maximum(np.abs(d) - tol, 0.0)
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
    max_step = cfg.max_delta_rate * dt

    def step_to(goal: float) -> float:
      step = float(min(max(goal - self.delta, -max_step), max_step))
      nd = self.delta + step
      # A ramp from an older interval must still satisfy the current one.
      if geometry is not None and not consumed_within_bounds(geometry[0], geometry[1], geometry[2], nd):
        nd = float(min(max(nd, trace["interval"][0]), trace["interval"][1]))
      return nd

    # A blocked cycle releases at the same (unapproved) rate; nothing is held.
    new_delta = step_to(0.0 if reason else request)
    safety = self.safety_reason(new_delta, self.delta, geometry, clearance)
    if safety and not reason:
      # The candidate itself failed: do not commit it; release instead.
      reason = safety
      new_delta = step_to(0.0)
      safety = self.safety_reason(new_delta, self.delta, geometry, clearance)
    trace["safety"] = safety
    if safety:
      # The release/residual motion cannot be shown safe either.  The existing
      # release ramp is emitted unchanged; choosing another policy is a human
      # decision, so the feature latches non-operational.
      self.unresolved = trace["unresolved"] = UNRESOLVED_RELEASE
    self.delta = new_delta
    self.geometry = geometry if not reason else None
    trace["reason"] = reason
    trace["delta"] = self.delta
    return self.delta

  def check_consumed(self, desired: float, shadow_desired: float) -> bool:
    """True when the consumed delta (actual minus the no-correction baseline
    chain) is finite, keeps every checked point between b and c and passes the
    safety contract for the band and the motion from the previous consumed
    delta.  No geometry == no new correction claim, which only holds when the
    consumed delta is exactly 0."""
    dk = desired - shadow_desired
    self.trace["consumed_delta"] = dk
    if not (math.isfinite(desired) and math.isfinite(dk)):
      ok = False
    elif self.geometry is None:
      ok = dk == 0.0
    else:
      ok = consumed_within_bounds(self.geometry[0], self.geometry[1], self.geometry[2], dk)
      if ok:
        why = self.safety_reason(dk, self.last_residual, self.geometry, self.clearance)
        self.trace["consumed_safety"] = why
        ok = not why
    self.trace["consumed_ok"] = ok
    return ok

  def consume(self, weight: float, lane_curvature: Optional[float], laneless_curvature: float,
              prev_desired: float, lat_smooth_seconds: float, v_ego: float, roll: float, dt: float,
              clip_fn: Callable[[float, float, float, float], Tuple[float, bool]]) -> Tuple[float, bool]:
    """Candidate -> check -> (recompute -> check again) -> commit.  prev_desired is
    controlsd's consumed state; the baseline chain is owned here."""
    t = self.trace
    if self.baseline_desired is None:
      self.baseline_desired = prev_desired
      t["baseline_seed"] = "consumed_prev"
    t["baseline_independent"] = self.baseline_independent
    tail = (lat_smooth_seconds, v_ego, roll, dt, clip_fn)
    base, _, _ = consume_curvature(weight, lane_curvature, laneless_curvature, 0.0, self.baseline_desired, *tail)
    desired, limited, stages = consume_curvature(weight, lane_curvature, laneless_curvature, self.delta,
                                                 prev_desired, *tail)
    first_ok = self.check_consumed(desired, base)
    t.update(first=desired, first_ok=first_ok, first_delta=self.delta)
    status = "candidate"
    if not first_ok:
      t["reason"] = BLOCK_CONSUMED_VIOLATION
      self.delta = 0.0
      desired, limited, stages = consume_curvature(weight, lane_curvature, laneless_curvature, 0.0,
                                                   prev_desired, *tail)
      re_ok = self.check_consumed(desired, base)  # delta 0 still carries the residual
      t.update(recomputed=desired, recomputed_ok=re_ok)
      status = "recomputed"
      if not re_ok:
        # Emitted value is the pre-v4 recomputation, unvalidated: human decision.
        status = "unresolved"
        self.unresolved = t["unresolved"] = UNRESOLVED_RECOMPUTE
    self.baseline_desired = base
    self.last_residual = float(desired - base) if math.isfinite(desired - base) else math.nan
    t.update(stages, baseline=base, final=desired, final_status=status, residual=self.last_residual,
             curvature_limited=limited)
    return desired, limited
