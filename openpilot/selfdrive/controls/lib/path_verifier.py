"""Plausibility check for the laneless desired curvature.

In laneless driving controlsd feeds ``modelV2.action.desiredCurvature`` straight
into the lateral controller.  Nothing downstream asks whether that number agrees
with the road: the ISO jerk limit in ``clip_curvature`` only bounds the *rate*,
the dynamic ``LatSmooth`` from the plan stds only bounds the *noise*, and the
lane prob / std / width checks in ``lane_planner_2`` are lane-mode only.  On
route 00000154--2c58732416 seg5 (2026-10-05) the lane line probs collapsed in a
left-hand curve and the model's 60 m lateral target flipped from -2.3 m (left)
to +3.8 m (right) for about 1.5 s.  openpilot unwound the wheel from 16 deg to
2 deg and the driver had to take over at 37 deg.

This module collects independent, *model-free* evidence of where the road goes
and only slows the model down when that evidence contradicts it.  The design is
deliberately ASYMMETRIC: the model value always passes straight through, and the
only thing the verifier owns is an additive *deviation* that is bounded in size
and in rate.  A correct curvature change therefore can never be delayed by more
than the (small) deviation already standing, and a wrong one is only slowed.

  * any confident evidence that supports the model  -> deviation decays to 0
  * no confident evidence at all                    -> deviation decays to 0
  * confident evidence contradicts, nothing supports-> deviation absorbs the change
  * contradicted value held by the model > 0.5 s    -> deviation collapses (the
                                                       model is never locked out)

Everything is continuous (requirement 7): there is no on/off switch anywhere in
the signal path.

  output = model + effect
  effect -> authority * deviation_state, rate limited by DEV_JERK_LIMIT

``authority`` is the same laneless weight the ``lat_mode_blend`` crossfade uses
(0 in lane mode, 1 fully laneless) multiplied by continuous ramps for lane-line
probability, speed, driver override, arming and sensor health.  ``effect`` is
rate limited, so every gate edge -- lanes appearing, steeringPressed, a lane
change starting -- produces a ramp instead of a step, and the verifier can never
add lateral jerk of its own.

Sign conventions (verified against the logs above, see
``tools/path_verifier/report.md``):

  * model frame: x forward, y RIGHT positive
  * ``livePose.angularVelocityDevice.z`` > 0  == turning right
  * ``carState.steeringAngleDeg``        > 0  == turning left
  * ``modelV2.action.desiredCurvature``  > 0  == turning right, i.e. the SAME
    sign as ``yaw_rate / v_ego`` (corr +0.96 over 5162 engaged samples of route
    00000154, median ratio 0.96) and the opposite sign of the steering angle.

Pure numpy, no openpilot imports, dt passed in explicitly, so this runs under
python3.9 for the offline harness in ``tools/path_verifier``.
"""
from __future__ import annotations

import math
from collections import deque
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# authority (requirement 7b): continuous, never a switch
# ---------------------------------------------------------------------------
# Lane-line probability ramp.  Only laneless driving is checked -- lane mode
# already has lane_planner_2's gates -- but the probs rattle across any fixed
# threshold frame by frame, so the transition is a ramp rather than a latch.
LANELESS_PROB_GATE = 0.3       # centre of the ramp: above it the model is "using lanes"
LANELESS_PROB_RAMP = 0.1       # +-this around the gate; prob gate-ramp -> authority 1
# The probs do not just cross the gate, they rattle (0.26 / 0.31 / 0.32 frame by
# frame in the seg5 scene).  A peak hold that falls linearly is the continuous form
# of the old enter/exit latch: authority rises at once and falls from 1 to 0 in
# this time.  Linear rather than exponential so that, once the lanes are back,
# authority really reaches 0 and the output is bit-exact passthrough again
# (an exponential tail lingered ~3 s on the logs).
PROB_WEIGHT_FALL_SECONDS = 0.5  # s
MIN_ACTIVE_SPEED = 5.0         # m/s, below this the curvature is dominated by geometry, not by the model
SPEED_RAMP = 1.0               # m/s of ramp above MIN_ACTIVE_SPEED
# The arming delay warms up the evidence after *engagement*, not after lane loss:
# in the seg5 scene the lanes collapse in the same frame as the bad flip, so an
# arming delay triggered by lane loss would sit in passthrough exactly through
# the failure.  Lane memory is already warm by then, so lane loss arms at once.
ARM_DELAY_SECONDS = 0.5

# ---------------------------------------------------------------------------
# intentional path changes (requirement 6b): never fight the driver or the planner
# ---------------------------------------------------------------------------
INTENT_RELEASE_SECONDS = 2.0   # stay in passthrough this long after the manoeuvre ends
# A blinker on its own is much weaker than laneChangeState/desire: on route
# 00000154 seg5 the driver had the left blinker on for the whole scene (they were
# taking a left turn), so treating any blinker as "full passthrough" would switch
# the verifier off through the exact failure it exists for.  A blinker therefore
# only exempts deviations *toward the signalled side*; an explicit lane change,
# turn desire or navigation turn exempts everything.
INTENT_BLINKER_DIRECTIONAL = True

# ---------------------------------------------------------------------------
# geometry / fitting
# ---------------------------------------------------------------------------
LOOKAHEAD_TIME = 1.0           # s, evidence and the model plan are compared at this time ahead
LOOKAHEAD_MIN_X = 8.0          # m, never compare closer than this (too noisy)
LOOKAHEAD_MAX_X = 40.0         # m, never compare further than this (model horizon gets soft)
FIT_MAX_X = 60.0               # m, longest usable part of a 33 point plan/lane/edge polyline
FIT_MIN_POINTS = 8             # a cubic fit needs a decent number of samples
POLY_DEGREE = 3                # cubic: curvature may vary along the look-ahead

# A curvature below this is "straight" for decision purposes (~670 m radius).
CURV_SIGNIFICANT = 0.0015      # 1/m

# Evidence agrees with the model if it is within this band.
EVIDENCE_TOL_BASE = 0.0015     # 1/m, absolute slack (fit noise)
EVIDENCE_TOL_FRAC = 0.5        # plus 50 % of the evidence magnitude (curvature estimates are relative)

# "the model is already changing toward this evidence" is judged against the plan
# curvature from this long ago, not the previous frame: frame-to-frame plan noise
# is the same size as the change we are looking for.
MOVING_BASELINE_SECONDS = 0.3
MOVING_CLOSER_MIN = 0.0002     # 1/m the gap must have closed by, to count as a real move
MODEL_REF_SMOOTH_TAU = 0.1     # s, light low pass used only for the "is it moving" test;
                               # the raw value is what gets compared against the tolerance
# When the model points the *opposite* way to significant evidence, "closing the
# gap" only counts as support if it is closing fast enough to actually agree
# within this horizon. A real S-curve / curve onset sweeps far faster than this,
# a drifting glitch does not.
CLOSE_HORIZON_SECONDS = 0.3

# ---------------------------------------------------------------------------
# (a) lane memory
# ---------------------------------------------------------------------------
LANE_MEM_PROB_MIN = 0.5        # store a lane-center profile only while both lines are this confident
LANE_MEM_STD_MAX = 0.3         # m, and only while both line stds are tight (same band lane_planner_2 uses)
LANE_MEM_WIDTH_MIN = 2.0       # m, sanity on the stored lane
LANE_MEM_WIDTH_MAX = 4.5       # m
LANE_MEM_MAX_AGE = 3.0         # s, memory is worthless after this
LANE_MEM_MAX_DIST = 60.0       # m, and after this much travel we have driven off the stored profile
LANE_MEM_CONF_MAX = 1.0        # a fresh lane fit is the best evidence available

# ---------------------------------------------------------------------------
# (b) road edges
# ---------------------------------------------------------------------------
EDGE_STD_MAX = 0.6             # m, above this the edge polyline is not trustworthy at all
EDGE_FIT_MIN_X = 10.0          # m, the first metres of a road edge are noisy
EDGE_FIT_MAX_X = 50.0          # m
EDGE_AGREE_TOL = 0.002         # 1/m, two edges must agree within this or neither counts
EDGE_CONF_MAX_BOTH = 0.9       # two agreeing edges
EDGE_CONF_MAX_ONE = 0.6        # a single edge could be a slip road / parked cars

# ---------------------------------------------------------------------------
# (c) trails of other road users (leadOne + other moving radar tracks)
# ---------------------------------------------------------------------------
TRAIL_MIN_DIST = 8.0           # m, closer than this the trail is too short to fit
TRAIL_MAX_DIST = 80.0          # m, further than this radar lateral is too coarse
TRAIL_SECONDS = 3.0            # s of world-frame trail to keep
TRAIL_MIN_POINTS = 10          # need this many samples to fit
TRAIL_MIN_SPAN = 10.0          # m of x span before a curvature fit means anything
TRAIL_RESID_MAX = 0.35         # m RMS, a kinked trail (lane change, mis-association) is rejected
TRAIL_LANE_CHANGE_LAT = 1.2    # m of target lateral motion over the trail with the ego going straight
TRAIL_LANE_CHANGE_YAW = 0.05   # rad, "ego going straight" threshold for the test above
LEAD_CONF_MAX = 0.7            # a lead follows its own line within the lane, so it is weaker evidence
OTHER_CONF_MAX = 0.5           # adjacent-lane traffic is weaker still (different lane, different line)
OTHER_TRAIL_MAX_TRACKS = 6     # bound the work and the memory: closest N moving tracks only
OTHER_TRAIL_MIN_TRACKS = 2     # a single adjacent car is not worth its own vote

# ---------------------------------------------------------------------------
# (c2) roadside stationary radar points (guardrails, poles, parked cars)
# ---------------------------------------------------------------------------
# liveTracks carries ~20-25 points at 20 Hz on these logs and almost all of them
# are ground-fixed, so a 3 s world-frame accumulation gives a dense roadside
# point cloud that is completely independent of the vision model.
TRACK_STATIONARY_V = 1.0       # m/s, |vLead| below this == ground fixed
TRACK_MIN_X = 5.0              # m
TRACK_MAX_X = 90.0             # m
TRACK_SIDE_MIN = 1.5           # m, |y| below this is in our own lane -> not a roadside
TRACK_SIDE_MAX = 25.0          # m, further out is another road / building
TRACK_MEM_SECONDS = 3.0        # s of accumulation
TRACK_MEM_MAX_POINTS = 900     # hard cap on the stored cloud (memory bound)
# At an intersection the roadside behind us belongs to the road we are leaving:
# in world coordinates it genuinely carries straight on while the new road bends,
# so an old cloud would "contradict" a perfectly correct turn (seen on seg5 at
# model index 265-274, where the model was recovering correctly).  Points recorded
# more than this much heading away from now are dropped.
TRACK_MAX_HEADING_CHANGE = 0.15  # rad (~8.6 deg)
TRACK_MIN_POINTS = 12          # per side
TRACK_MIN_SPAN = 20.0          # m, required x spread of one side
TRACK_RESID_MAX = 0.8          # m RMS, a roadside is not a straight line but it is not noise either
TRACK_AGREE_TOL = 0.0025       # 1/m, two sides must agree within this
TRACK_CONF_BOTH = 0.8          # both roadsides agreeing
TRACK_CONF_ONE = 0.45          # a single side could be a row of parked cars

# ---------------------------------------------------------------------------
# (d) ego continuity
# ---------------------------------------------------------------------------
EGO_FILTER_TAU = 0.2           # s, low pass on yaw_rate / v
EGO_TREND_SECONDS = 0.5        # s, window used for the curvature trend (reported, not extrapolated:
                               # during the seg5 unwind the trend is driven by the bad model itself,
                               # so extrapolating it flips the evidence to the wrong sign)
EGO_CONF_MAX = 0.7             # the ego state is the past, not the road ahead, so cap it

# ---------------------------------------------------------------------------
# (e) model self-consistency
# ---------------------------------------------------------------------------
# A single old plan is as noisy as the current one, so the median of every plan
# in this age window is used: "what the model has consistently been saying about
# this piece of road".
SELF_LAG_MIN_SECONDS = 0.5     # s, below this the plans are too correlated to be independent
SELF_LAG_MAX_SECONDS = 1.1     # s
SELF_HIST_SECONDS = 1.6        # s of plan history to keep
# Fitting every plan in the window costs ~12 cubic fits per evidence update, which
# dominated the whole module's run time.  These fixed target ages pick a stable
# subset instead: the chosen plans slide continuously with time, so the median
# does not jitter the way a changing subsample did.
SELF_AGE_TARGETS = (0.55, 0.70, 0.85, 1.00)  # s
SELF_AGE_TOL = 0.08           # s, a target with no plan this close is skipped
SELF_MIN_PLANS = 3             # need this many old plans in the window
SELF_CONF_MAX = 0.9            # the model's own earlier opinion about the same piece of road

# ---------------------------------------------------------------------------
# decision / action
# ---------------------------------------------------------------------------
# All evidence derives from 20 Hz model / 20 Hz radar data, so recomputing the
# fits on every 100 Hz control cycle is wasted work.  The value being judged
# (model_ref) is still refreshed every cycle, so support is never detected late.
EVIDENCE_PERIOD = 0.05         # s, minimum interval between evidence recomputations

CONTRADICT_CONF_MIN = 0.3      # below this contradiction weight nothing happens
# The asymmetry requirement, in three levels.  "The model is demonstrably moving
# toward this evidence" is the correct-change case and cancels everything.  Mere
# agreement within the (deliberately generous) tolerance band is much weaker: it
# fully excuses a magnitude objection, but a source saying the road bends the
# OTHER WAY is a qualitatively worse symptom and is only partly excused -- in the
# seg5 scene the measured ego curvature still "agreed" with the unwinding model
# inside the tolerance while the road clearly went the other way.
SUPPORT_GAIN_MAG = 1.5         # support vs a magnitude (undercut) objection
SUPPORT_GAIN_SIGN = 0.5        # support vs an outright sign conflict
# The hold weight engages at once but releases smoothly (7d).  Individual
# evidence sources drop in and out over 2-3 model frames as their fits lose
# confidence, which is a fit artifact rather than real agreement, so the release
# deliberately spans several model frames.
HOLD_WEIGHT_RELEASE_TAU = 0.3   # s
HOLD_TAU_SECONDS = 1.0         # s, decay of the deviation while evidence contradicts
DECAY_TAU_SECONDS = 0.3        # s, decay of the deviation with no contradiction (7c)
CONVERGE_TAU_SECONDS = 0.2     # s, once the model has persisted we collapse the deviation
CONTRADICT_PERSIST_SECONDS = 0.5  # s, a contradicted value held this long is accepted
CONVERGE_RAMP_SECONDS = 0.2    # s, blend into the converge tau instead of switching it
MAX_LAT_ACCEL_DEVIATION = 0.3  # m/s^2, hard cap on |output - model| * v^2 (requirement 4)
# Requirement 7e: the verifier may never add jerk.  clip_curvature uses the ISO
# MAX_LATERAL_JERK = 5.0 m/s^3 for the whole command; the verifier keeps its own
# contribution to 20 % of that, which also gives the ~0.3 s ramp asked for in 7c
# (0.3 m/s^2 of deviation at 1.0 m/s^3 takes 0.3 s to build or release).
DEV_JERK_LIMIT = 1.0           # m/s^3
# Every decay in here is exponential, so it approaches zero without ever reaching
# it.  Anything below this counts as zero, which is what makes the passthrough
# requirement ("output == model exactly in lane mode") actually hold: 1e-9 1/m is
# 4e-7 m/s^2 at 20 m/s, i.e. far below any physical relevance.
RESET_EPS = 1e-9               # 1/m

VOTE_SUPPORT = "support"
VOTE_CONTRADICT = "contradict"
VOTE_NEUTRAL = "neutral"

DECISION_PASS = "pass"
DECISION_HOLD = "hold"
DECISION_CONVERGE = "converge"

EVIDENCE_NAMES = ("lane_memory", "road_edges", "road_tracks", "lead_trail",
                  "other_trails", "ego_continuity", "model_self")

# "the model barely turns while this says the road bends" is only a contradiction
# for sources that describe the road *ahead*.  The measured ego curvature is the
# road *behind*: it legitimately disagrees in magnitude at every curve entry and
# exit, so it may only object to an outright sign conflict.
EVIDENCE_UNDERCUT = {
  "lane_memory": True,
  "road_edges": True,
  "road_tracks": True,
  "lead_trail": True,
  "other_trails": True,
  "ego_continuity": False,
  "model_self": True,
}
UNDERCUT_FRAC = 0.5           # model must turn at least this fraction of the evidence

# Which evidence depends on which sensor, for the fail-safe inputs (requirement 6e).
RADAR_EVIDENCE = ("road_tracks", "lead_trail", "other_trails")
MODEL_EVIDENCE = ("lane_memory", "model_self", "road_edges")

# Settings the validation sweep varies.  Everything else stays a module constant.
DEFAULT_CONFIG = {
  "hold_tau": HOLD_TAU_SECONDS,
  "converge_after": CONTRADICT_PERSIST_SECONDS,
  "max_deviation": MAX_LAT_ACCEL_DEVIATION,
  "laneless_prob_gate": LANELESS_PROB_GATE,
  "edge_std_max": EDGE_STD_MAX,
  "blinker_directional": INTENT_BLINKER_DIRECTIONAL,
  "hold_release_tau": HOLD_WEIGHT_RELEASE_TAU,
}


def _as_array(v: Optional[Sequence[float]]) -> np.ndarray:
  if v is None:
    return np.zeros(0)
  return np.asarray(v, dtype=np.float64).ravel()


def _finite_pair(x: np.ndarray, y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
  n = min(x.size, y.size)
  if n == 0:
    return x[:0], y[:0]
  x, y = x[:n], y[:n]
  m = np.isfinite(x) & np.isfinite(y)
  return x[m], y[m]


_FIT_SCALE = 30.0  # x is normalised by this before fitting to keep the normal equations well conditioned
_FIT_UNSCALE = np.array([_FIT_SCALE ** -3, _FIT_SCALE ** -2, _FIT_SCALE ** -1, 1.0])


def _poly_fit(x: np.ndarray, y: np.ndarray, x_min: float, x_max: float,
              min_points: int = FIT_MIN_POINTS) -> Optional[np.ndarray]:
  """Cubic fit of y(x) over [x_min, x_max]; None if there is not enough clean data.

  Solved via the 4x4 normal equations rather than ``np.polyfit``: the SVD in
  polyfit costs ~50us and this runs several times per control cycle at 100 Hz.
  The range, finiteness and scaling are done in one pass for the same reason.
  """
  n = min(x.size, y.size)
  if n < min_points:
    return None
  x, y = x[:n], y[:n]
  m = (x >= x_min) & (x <= x_max) & np.isfinite(x) & np.isfinite(y)
  cnt = int(m.sum())
  if cnt < min_points:
    return None
  xs = x[m] * (1.0 / _FIT_SCALE)
  ys = y[m]
  if xs.max() - xs.min() < 1.0 / _FIT_SCALE:  # less than 1 m of span
    return None
  x2 = xs * xs
  v = np.empty((cnt, POLY_DEGREE + 1))
  v[:, 0] = x2 * xs
  v[:, 1] = x2
  v[:, 2] = xs
  v[:, 3] = 1.0
  try:
    coeffs = np.linalg.solve(v.T @ v, v.T @ ys)
  except Exception:
    return None
  if not np.all(np.isfinite(coeffs)):
    return None
  return coeffs * _FIT_UNSCALE


def _poly_curvature(coeffs: Optional[np.ndarray], x_ref: float) -> Optional[float]:
  """Signed curvature y'' / (1 + y'^2)^(3/2) of a cubic at x_ref. y right positive,
  so a positive result means the path bends right -- same sign as desiredCurvature."""
  if coeffs is None:
    return None
  a3, a2, a1, _ = coeffs
  d1 = 3.0 * a3 * x_ref * x_ref + 2.0 * a2 * x_ref + a1
  d2 = 6.0 * a3 * x_ref + 2.0 * a2
  c = d2 / (1.0 + d1 * d1) ** 1.5
  if not math.isfinite(c):
    return None
  return float(c)


def _poly_resid_rms(coeffs: np.ndarray, x: np.ndarray, y: np.ndarray) -> float:
  if x.size == 0:
    return float("inf")
  return float(np.sqrt(np.mean((y - np.polyval(coeffs, x)) ** 2)))


def _ramp(v: float, lo: float, hi: float) -> float:
  """Linear 0 -> 1 ramp, robust to lo == hi and to non-finite input."""
  if not math.isfinite(v):
    return 0.0
  if hi <= lo:
    return 1.0 if v >= hi else 0.0
  return float(min(max((v - lo) / (hi - lo), 0.0), 1.0))


class _Odometry:
  """Dead-reckoned ego pose in an arbitrary fixed frame.

  Same axis convention as the model frame: x forward, y right, heading positive
  when turning right (so it integrates ``livePose.angularVelocityDevice.z``
  directly).  Everything the verifier remembers is stored in this frame, which
  makes "where is that old observation now" a single rigid transform.
  """

  def __init__(self) -> None:
    self.x = 0.0
    self.y = 0.0
    self.heading = 0.0
    self.odometer = 0.0  # travelled arc length

  def reset(self) -> None:
    self.x = 0.0
    self.y = 0.0
    self.heading = 0.0
    self.odometer = 0.0

  def step(self, v_ego: float, yaw_rate: float, dt: float) -> None:
    self.x += v_ego * math.cos(self.heading) * dt
    self.y += v_ego * math.sin(self.heading) * dt
    self.heading += yaw_rate * dt
    self.odometer += v_ego * dt

  def pose(self) -> Tuple[float, float, float, float]:
    return self.x, self.y, self.heading, self.odometer

  def to_world(self, pose: Tuple[float, float, float, float],
               px: np.ndarray, py: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    x0, y0, th, _ = pose
    c, s = math.cos(th), math.sin(th)
    return x0 + px * c - py * s, y0 + px * s + py * c

  def to_ego(self, wx: np.ndarray, wy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    dx = wx - self.x
    dy = wy - self.y
    c, s = math.cos(self.heading), math.sin(self.heading)
    return dx * c + dy * s, -dx * s + dy * c


class PathVerifier:
  """See module docstring. ``update()`` returns ``(curvature_out, debug)``."""

  def __init__(self, **config: Any) -> None:
    unknown = set(config) - set(DEFAULT_CONFIG)
    if unknown:
      raise TypeError("unknown PathVerifier config: %s" % sorted(unknown))
    self.cfg: Dict[str, Any] = dict(DEFAULT_CONFIG)
    self.cfg.update(config)
    self.odo = _Odometry()
    self.reset()

  # -- state ---------------------------------------------------------------
  def reset(self) -> None:
    self.odo.reset()

    # action state (requirement 7): a deviation and its rate limited effect
    self.deviation = 0.0        # 1/m, what the verifier wants to add to the model
    self.effect = 0.0           # 1/m, what it is actually adding right now
    self.authority = 0.0
    self.prob_w = 0.0
    self.hold_weight = 0.0
    self.contradict_sign = 0.0
    self.arm_timer = 0.0
    self.hold_timer = 0.0
    self.intent_timer = INTENT_RELEASE_SECONDS  # start exempt until we have seen a clean frame
    self.intent_side = 0        # -1 left, +1 right, 0 none -- for the directional blinker rule
    self.prev_model: Optional[float] = None
    self.prev_lat_active = False
    self.output = 0.0

    self.model_ref_hist: Deque[Tuple[float, float]] = deque()  # (t, smoothed plan curvature at x_ref)
    self.model_ref_smooth: Optional[float] = None
    self.model_ref: Optional[float] = None
    self.model_ref_t = 0.0

    # (a) lane memory: one snapshot of the lane-center polyline in world coords
    self.lane_mem: Optional[Dict[str, Any]] = None

    # (c) trails: key -> deque of (t, wx, wy, d_rel, y_rel, ego_heading).
    # key "lead" is radarState.leadOne, integer keys are liveTracks trackIds.
    self.trails: Dict[Any, Deque[Tuple[float, float, float, float, float, float]]] = {}
    self.trail_sig: Optional[Tuple[int, float, float]] = None

    # (c2) roadside stationary point cloud in world coords, as a fixed ring
    # buffer of (t, wx, wy, ego heading when recorded)
    self.cloud = np.zeros((TRACK_MEM_MAX_POINTS, 4))
    self.cloud_n = 0
    self.cloud_i = 0
    self.track_sig: Optional[Tuple[int, float, float]] = None

    # (d) ego continuity
    self.ego_curv = 0.0
    self.ego_curv_valid = False
    self.ego_hist: Deque[Tuple[float, float]] = deque()  # (t, filtered curvature)

    # (e) model self-consistency: plan polylines in world coords
    self.plan_hist: Deque[Dict[str, Any]] = deque()

    # evidence recomputed at model rate, not control rate (see EVIDENCE_PERIOD)
    self.ev_cache: Optional[Dict[str, Tuple[Optional[float], float]]] = None
    self.ev_cache_t = 0.0

    self.t = 0.0

  # -- evidence ------------------------------------------------------------
  def _update_lane_memory(self, lll_prob: float, rll_prob: float, lll_std: float, rll_std: float,
                          lane_x: np.ndarray, lll_y: np.ndarray, rll_y: np.ndarray) -> None:
    if not (lll_prob > LANE_MEM_PROB_MIN and rll_prob > LANE_MEM_PROB_MIN):
      return
    if not (lll_std < LANE_MEM_STD_MAX and rll_std < LANE_MEM_STD_MAX):
      return
    x, lll = _finite_pair(lane_x, lll_y)
    x2, rll = _finite_pair(lane_x, rll_y)
    if x.size < FIT_MIN_POINTS or x2.size != x.size:
      return
    width = float(np.median(rll - lll))
    if not (LANE_MEM_WIDTH_MIN < width < LANE_MEM_WIDTH_MAX):
      return
    center = 0.5 * (lll + rll)
    m = (x >= 0.0) & (x <= FIT_MAX_X)
    if int(m.sum()) < FIT_MIN_POINTS:
      return
    wx, wy = self.odo.to_world(self.odo.pose(), x[m], center[m])
    self.lane_mem = {"t": self.t, "s": self.odo.odometer, "wx": wx, "wy": wy}

  def _ev_lane_memory(self, x_ref: float) -> Tuple[Optional[float], float]:
    mem = self.lane_mem
    if mem is None:
      return None, 0.0
    age = self.t - mem["t"]
    dist = self.odo.odometer - mem["s"]
    if age > LANE_MEM_MAX_AGE or dist > LANE_MEM_MAX_DIST or age < 0.0 or dist < 0.0:
      return None, 0.0
    ex, ey = self.odo.to_ego(mem["wx"], mem["wy"])
    # the stored profile only covered FIT_MAX_X ahead of where it was taken
    c = _poly_curvature(_poly_fit(ex, ey, 0.0, FIT_MAX_X), x_ref)
    if c is None:
      return None, 0.0
    # The look-ahead point must still be inside the stored profile, no extrapolation.
    valid_x = ex[np.isfinite(ex)]
    if valid_x.size == 0 or valid_x.max() < x_ref:
      return None, 0.0
    conf = LANE_MEM_CONF_MAX * min(1.0 - age / LANE_MEM_MAX_AGE, 1.0 - dist / LANE_MEM_MAX_DIST)
    return c, max(conf, 0.0)

  def _ev_road_edges(self, edge_x: np.ndarray, le_y: np.ndarray, re_y: np.ndarray,
                     le_std: float, re_std: float, x_ref: float) -> Tuple[Optional[float], float]:
    std_max = float(self.cfg["edge_std_max"])
    xr = float(np.clip(x_ref, EDGE_FIT_MIN_X, EDGE_FIT_MAX_X))
    cands: List[Tuple[float, float]] = []  # (curvature, std)
    for y, std in ((le_y, le_std), (re_y, re_std)):
      if not math.isfinite(std) or std >= std_max:
        continue
      c = _poly_curvature(_poly_fit(edge_x, y, EDGE_FIT_MIN_X, EDGE_FIT_MAX_X), xr)
      if c is not None:
        cands.append((c, float(std)))
    if not cands:
      return None, 0.0
    lo = min(0.2, 0.5 * std_max)
    if len(cands) == 2:
      c0, c1 = cands[0][0], cands[1][0]
      if abs(c0 - c1) > EDGE_AGREE_TOL:
        return None, 0.0  # the two edges disagree -> neither is evidence
      c = 0.5 * (c0 + c1)
      conf = EDGE_CONF_MAX_BOTH * float(np.interp(max(cands[0][1], cands[1][1]), [lo, std_max], [1.0, 0.0]))
    else:
      c, std = cands[0]
      conf = EDGE_CONF_MAX_ONE * float(np.interp(std, [lo, std_max], [1.0, 0.0]))
    return c, max(conf, 0.0)

  # -- (c) trails ----------------------------------------------------------
  def _push_trail(self, key: Any, d_rel: float, y_rel: float) -> None:
    if not (math.isfinite(d_rel) and math.isfinite(y_rel)):
      self.trails.pop(key, None)
      return
    if not (TRAIL_MIN_DIST <= d_rel <= TRAIL_MAX_DIST):
      return
    tr = self.trails.get(key)
    if tr is None:
      tr = self.trails[key] = deque()
    elif tr and tr[-1][3] == d_rel and tr[-1][4] == y_rel:
      return  # source has not been refreshed, do not duplicate the sample
    wx, wy = self.odo.to_world(self.odo.pose(), np.array([d_rel]), np.array([y_rel]))
    tr.append((self.t, float(wx[0]), float(wy[0]), float(d_rel), float(y_rel), self.odo.heading))

  def _update_trails(self, lead_present: bool, lead_d_rel: float, lead_y_rel: float,
                     track_d: np.ndarray, track_y: np.ndarray, track_v: np.ndarray,
                     track_id: np.ndarray) -> None:
    # age out, and drop tracks we have not seen for a while
    for key in list(self.trails):
      tr = self.trails[key]
      while tr and self.t - tr[0][0] > TRAIL_SECONDS:
        tr.popleft()
      if not tr:
        del self.trails[key]

    if lead_present:
      self._push_trail("lead", float(lead_d_rel), float(lead_y_rel))
    else:
      self.trails.pop("lead", None)

    n = min(track_d.size, track_y.size, track_v.size, track_id.size)
    if n == 0:
      return  # without stable track ids a trail cannot be accumulated at all
    sig = (n, float(np.nansum(track_d[:n])), float(np.nansum(track_y[:n])))
    if sig == self.trail_sig:
      return  # same liveTracks message replayed at the control rate
    self.trail_sig = sig
    moving = np.isfinite(track_v)[:n] & (np.abs(track_v[:n]) >= TRACK_STATIONARY_V)
    moving &= (track_d[:n] >= TRAIL_MIN_DIST) & (track_d[:n] <= TRAIL_MAX_DIST)
    idx = np.nonzero(moving)[0]
    if idx.size == 0:
      return
    # closest OTHER_TRAIL_MAX_TRACKS only: bounds both the work and the memory
    idx = idx[np.argsort(track_d[idx])][:OTHER_TRAIL_MAX_TRACKS]
    for i in idx:
      self._push_trail(("trk", int(track_id[i])), float(track_d[i]), float(track_y[i]))

  def _trail_curvature(self, tr: Deque[Tuple[float, float, float, float, float, float]],
                       x_ref: float) -> Optional[Tuple[float, float]]:
    """(curvature, x span) of one target trail, or None if it is not usable."""
    if len(tr) < TRAIL_MIN_POINTS:
      return None
    # A target that moves sideways while we drive straight is changing lanes, not
    # tracing the road -- its trail is not evidence about road curvature.
    if (abs(tr[-1][4] - tr[0][4]) > TRAIL_LANE_CHANGE_LAT
        and abs(self.odo.heading - tr[0][5]) < TRAIL_LANE_CHANGE_YAW):
      return None
    wx = np.fromiter((p[1] for p in tr), dtype=np.float64, count=len(tr))
    wy = np.fromiter((p[2] for p in tr), dtype=np.float64, count=len(tr))
    ex, ey = self.odo.to_ego(wx, wy)
    span = float(ex.max() - ex.min())
    if span < TRAIL_MIN_SPAN:
      return None
    coeffs = _poly_fit(ex, ey, ex.min() - 1.0, ex.max() + 1.0)
    if coeffs is None or _poly_resid_rms(coeffs, ex, ey) > TRAIL_RESID_MAX:
      return None  # kinked trail: lane change or a mis-associated track
    c = _poly_curvature(coeffs, float(np.clip(x_ref, ex.min(), ex.max())))
    if c is None:
      return None
    return c, span

  def _ev_lead_trail(self, x_ref: float) -> Tuple[Optional[float], float]:
    tr = self.trails.get("lead")
    r = self._trail_curvature(tr, x_ref) if tr is not None else None
    if r is None:
      return None, 0.0
    c, span = r
    return c, LEAD_CONF_MAX * float(np.interp(span, [TRAIL_MIN_SPAN, 30.0], [0.3, 1.0]))

  def _ev_other_trails(self, x_ref: float) -> Tuple[Optional[float], float]:
    cs: List[float] = []
    spans: List[float] = []
    for key, tr in self.trails.items():
      if key == "lead":
        continue
      r = self._trail_curvature(tr, x_ref)
      if r is not None:
        cs.append(r[0])
        spans.append(r[1])
    if len(cs) < OTHER_TRAIL_MIN_TRACKS:
      return None, 0.0
    c = float(np.median(cs))
    conf = OTHER_CONF_MAX * float(np.interp(float(np.median(spans)), [TRAIL_MIN_SPAN, 30.0], [0.3, 1.0]))
    return c, conf

  # -- (c2) roadside stationary points -------------------------------------
  def _update_track_cloud(self, track_d: np.ndarray, track_y: np.ndarray, track_v: np.ndarray) -> None:
    n = min(track_d.size, track_y.size, track_v.size)
    if n == 0:
      return
    d, y, v = track_d[:n], track_y[:n], track_v[:n]
    # liveTracks arrives at 20 Hz but update() runs at 100 Hz: without this the
    # same ~24 points would be stored five times over, which both biases the fit
    # and costs five times the work.
    sig = (n, float(np.nansum(d)), float(np.nansum(y)))
    if sig == self.track_sig:
      return
    self.track_sig = sig
    m = (np.isfinite(d) & np.isfinite(y) & np.isfinite(v)
         & (np.abs(v) < TRACK_STATIONARY_V)
         & (d >= TRACK_MIN_X) & (d <= TRACK_MAX_X)
         & (np.abs(y) >= TRACK_SIDE_MIN) & (np.abs(y) <= TRACK_SIDE_MAX))
    if not m.any():
      return
    wx, wy = self.odo.to_world(self.odo.pose(), d[m], y[m])
    k = min(wx.size, TRACK_MEM_MAX_POINTS)
    idx = (self.cloud_i + np.arange(k)) % TRACK_MEM_MAX_POINTS
    self.cloud[idx, 0] = self.t
    self.cloud[idx, 1] = wx[:k]
    self.cloud[idx, 2] = wy[:k]
    self.cloud[idx, 3] = self.odo.heading
    self.cloud_i = int((self.cloud_i + k) % TRACK_MEM_MAX_POINTS)
    self.cloud_n = int(min(self.cloud_n + k, TRACK_MEM_MAX_POINTS))

  def _ev_road_tracks(self, x_ref: float) -> Tuple[Optional[float], float]:
    if self.cloud_n < 2 * TRACK_MIN_POINTS:
      return None, 0.0
    c = self.cloud[:self.cloud_n]
    fresh = ((self.t - c[:, 0]) <= TRACK_MEM_SECONDS) & \
            (np.abs(c[:, 3] - self.odo.heading) <= TRACK_MAX_HEADING_CHANGE)
    if int(fresh.sum()) < 2 * TRACK_MIN_POINTS:
      return None, 0.0
    ex, ey = self.odo.to_ego(c[fresh, 1], c[fresh, 2])
    keep = (ex >= TRACK_MIN_X) & (ex <= TRACK_MAX_X) & (np.abs(ey) <= TRACK_SIDE_MAX)
    ex, ey = ex[keep], ey[keep]
    cands: List[Tuple[float, int]] = []  # (curvature, n points)
    for side in (-1.0, 1.0):
      m = (ey * side >= TRACK_SIDE_MIN)
      if int(m.sum()) < TRACK_MIN_POINTS:
        continue
      sx, sy = ex[m], ey[m]
      if sx.max() - sx.min() < TRACK_MIN_SPAN:
        continue
      coeffs = _poly_fit(sx, sy, sx.min() - 1.0, sx.max() + 1.0, min_points=TRACK_MIN_POINTS)
      if coeffs is None or _poly_resid_rms(coeffs, sx, sy) > TRACK_RESID_MAX:
        continue
      c = _poly_curvature(coeffs, float(np.clip(x_ref, sx.min(), sx.max())))
      if c is not None:
        cands.append((c, int(m.sum())))
    if not cands:
      return None, 0.0
    if len(cands) == 2:
      if abs(cands[0][0] - cands[1][0]) > TRACK_AGREE_TOL:
        return None, 0.0
      c = 0.5 * (cands[0][0] + cands[1][0])
      pts = min(cands[0][1], cands[1][1])
      conf = TRACK_CONF_BOTH * _ramp(float(pts), TRACK_MIN_POINTS, 3.0 * TRACK_MIN_POINTS)
    else:
      c, pts = cands[0]
      conf = TRACK_CONF_ONE * _ramp(float(pts), TRACK_MIN_POINTS, 3.0 * TRACK_MIN_POINTS)
    return c, conf

  # -- (d) ego continuity --------------------------------------------------
  def _update_ego(self, v_ego: float, yaw_rate: float, dt: float) -> None:
    c = yaw_rate / max(v_ego, 1.0)
    if not math.isfinite(c):
      return
    alpha = 1.0 - math.exp(-dt / EGO_FILTER_TAU) if EGO_FILTER_TAU > 0 else 1.0
    if not self.ego_curv_valid:
      self.ego_curv = c
      self.ego_curv_valid = True
    else:
      self.ego_curv += alpha * (c - self.ego_curv)
    self.ego_hist.append((self.t, self.ego_curv))
    while self.ego_hist and self.t - self.ego_hist[0][0] > EGO_TREND_SECONDS:
      self.ego_hist.popleft()

  def _ego_trend(self) -> float:
    if len(self.ego_hist) < 3:
      return 0.0
    t0, c0 = self.ego_hist[0]
    span = self.t - t0
    return (self.ego_curv - c0) / span if span > 0.1 else 0.0

  def _ev_ego_continuity(self) -> Tuple[Optional[float], float]:
    if not self.ego_curv_valid:
      return None, 0.0
    c_now = self.ego_curv
    conf = EGO_CONF_MAX * float(np.interp(abs(c_now), [CURV_SIGNIFICANT, 3.0 * CURV_SIGNIFICANT], [0.0, 1.0]))
    return float(c_now), conf

  # -- (e) model self-consistency ------------------------------------------
  def _update_plan_hist(self, plan_x: np.ndarray, plan_y: np.ndarray) -> bool:
    """Store the current plan in world coordinates. Returns True for a new model frame."""
    while self.plan_hist and self.t - self.plan_hist[0]["t"] > SELF_HIST_SECONDS:
      self.plan_hist.popleft()
    x, y = _finite_pair(plan_x, plan_y)
    if x.size < FIT_MIN_POINTS:
      return False
    if self.plan_hist:
      prev = self.plan_hist[-1]
      if prev["n"] == x.size and prev["sig"] == float(y[-1]) and prev["sig2"] == float(y[x.size // 2]):
        return False  # same model frame replayed at the control rate
    m = (x >= 0.0) & (x <= FIT_MAX_X)
    if int(m.sum()) < FIT_MIN_POINTS:
      return False
    wx, wy = self.odo.to_world(self.odo.pose(), x[m], y[m])
    self.plan_hist.append({"t": self.t, "s": self.odo.odometer, "wx": wx, "wy": wy,
                           "n": x.size, "sig": float(y[-1]), "sig2": float(y[x.size // 2]),
                           "heading": self.odo.heading})
    return True

  def _ev_model_self(self, x_ref: float) -> Tuple[Optional[float], float]:
    """Curvature at the same world point as predicted by the model's own plans of
    0.5 - 1.1 s ago, ego-motion compensated. Disagreement here is a short flip."""
    picks: List[Dict[str, Any]] = []
    for target in SELF_AGE_TARGETS:
      best = None
      best_err = SELF_AGE_TOL
      for e in self.plan_hist:
        age = self.t - e["t"]
        if not (SELF_LAG_MIN_SECONDS <= age <= SELF_LAG_MAX_SECONDS):
          continue
        err = abs(age - target)
        if err <= best_err:
          best, best_err = e, err
      if best is not None and not any(p is best for p in picks):
        picks.append(best)

    cs: List[float] = []
    for e in picks:
      ex, ey = self.odo.to_ego(e["wx"], e["wy"])
      valid_x = ex[np.isfinite(ex)]
      if valid_x.size == 0 or valid_x.max() < x_ref:
        continue  # we have driven past the useful part of that plan
      c = _poly_curvature(_poly_fit(ex, ey, 0.0, FIT_MAX_X), x_ref)
      if c is not None:
        cs.append(c)
    if len(cs) < SELF_MIN_PLANS:
      return None, 0.0
    c = float(np.median(cs))
    conf = SELF_CONF_MAX * float(np.interp(abs(c), [CURV_SIGNIFICANT, 3.0 * CURV_SIGNIFICANT], [0.0, 1.0]))
    return c, conf

  def _baseline_model_ref(self) -> Optional[float]:
    """Plan curvature at x_ref MOVING_BASELINE_SECONDS ago, for the
    "is the model already moving toward this evidence" test."""
    best = None
    best_err = None
    for ts, c in self.model_ref_hist:
      err = abs((self.t - ts) - MOVING_BASELINE_SECONDS)
      if best_err is None or err < best_err:
        best, best_err = c, err
    if best is None or best_err is None or best_err > 0.5 * MOVING_BASELINE_SECONDS:
      return None
    return best

  # -- main ----------------------------------------------------------------
  def update(self,
             v_ego: float,
             model_curvature: float,
             plan_x: Optional[Sequence[float]] = None,
             plan_y: Optional[Sequence[float]] = None,
             lll_prob: float = 0.0,
             rll_prob: float = 0.0,
             lll_std: float = 1.0,
             rll_std: float = 1.0,
             lane_x: Optional[Sequence[float]] = None,
             lll_y: Optional[Sequence[float]] = None,
             rll_y: Optional[Sequence[float]] = None,
             edge_x: Optional[Sequence[float]] = None,
             le_y: Optional[Sequence[float]] = None,
             re_y: Optional[Sequence[float]] = None,
             le_std: float = 1.0,
             re_std: float = 1.0,
             lead_present: bool = False,
             lead_d_rel: float = 0.0,
             lead_y_rel: float = 0.0,
             lead_v_lead: float = 0.0,
             track_d_rel: Optional[Sequence[float]] = None,
             track_y_rel: Optional[Sequence[float]] = None,
             track_v_lead: Optional[Sequence[float]] = None,
             track_id: Optional[Sequence[float]] = None,
             yaw_rate: float = 0.0,
             steering_pressed: bool = False,
             lat_active: bool = False,
             dt: float = 0.01,
             lane_mode_weight: float = 0.0,
             lane_change_active: bool = False,
             desire_turn: bool = False,
             blinker_left: bool = False,
             blinker_right: bool = False,
             nav_turn: bool = False,
             pose_valid: bool = True,
             calibrated: bool = True,
             radar_valid: bool = True,
             model_ok: bool = True) -> Tuple[float, Dict[str, Any]]:
    """One control cycle. Returns (curvature to use, debug dict).

    ``lane_mode_weight`` is controlsd's ``lat_mode_blend`` weight: 1.0 = pure
    lane mode, 0.0 = pure laneless. The verifier's authority is its complement,
    so the verifier fades in and out with the crossfade the car already does.
    """
    debug: Dict[str, Any] = {
      "reason": "",
      "decision": DECISION_PASS,
      "model": 0.0,
      "model_ref": None,
      "x_ref": 0.0,
      "authority": 0.0,
      "hold_weight": 0.0,
      "deviation_state": 0.0,
      "deviation": 0.0,
      "deviation_lat_accel": 0.0,
      "hold_time": self.hold_timer,
      "support_conf": 0.0,
      "contradict_conf": 0.0,
      "intent": False,
      "armed": False,
      "evidence": {n: {"curvature": None, "confidence": 0.0, "vote": VOTE_NEUTRAL} for n in EVIDENCE_NAMES},
    }

    # --- requirement 4: anything non-finite is an immediate passthrough ----
    if not (math.isfinite(model_curvature) and math.isfinite(v_ego) and math.isfinite(dt)) or dt <= 0.0:
      self.arm_timer = 0.0
      self.hold_timer = 0.0
      self.deviation = 0.0
      self.effect = 0.0
      self.authority = 0.0
      debug["reason"] = "nonfinite_input"
      out = float(model_curvature) if math.isfinite(model_curvature) else 0.0
      self.output = out
      self.prev_model = out
      debug["model"] = out
      return out, debug

    model = float(model_curvature)
    debug["model"] = model
    if not math.isfinite(yaw_rate):
      yaw_rate = 0.0
      pose_valid = False

    self.t += dt

    # --- evidence bookkeeping runs every cycle, lanes or not (7a), so the
    # --- memory is already warm the moment authority rises.
    odo_ok = bool(pose_valid) and bool(calibrated)
    if odo_ok:
      self.odo.step(v_ego, yaw_rate, dt)
    self._update_ego(v_ego, yaw_rate, dt)

    plan_x_a, plan_y_a = _as_array(plan_x), _as_array(plan_y)
    lane_x_a, lll_y_a, rll_y_a = _as_array(lane_x), _as_array(lll_y), _as_array(rll_y)
    edge_x_a, le_y_a, re_y_a = _as_array(edge_x), _as_array(le_y), _as_array(re_y)
    tr_d, tr_y, tr_v = _as_array(track_d_rel), _as_array(track_y_rel), _as_array(track_v_lead)
    tr_id = _as_array(track_id)

    new_plan = self._update_plan_hist(plan_x_a, plan_y_a)
    if new_plan or self.lane_mem is None:
      # lane lines are 20 Hz data; storing them again on every control cycle
      # would only duplicate work
      self._update_lane_memory(lll_prob, rll_prob, lll_std, rll_std, lane_x_a, lll_y_a, rll_y_a)
    if radar_valid:
      self._update_trails(bool(lead_present), float(lead_d_rel), float(lead_y_rel), tr_d, tr_y, tr_v, tr_id)
      self._update_track_cloud(tr_d, tr_y, tr_v)
    else:
      self.trails.clear()
      self.cloud_n = 0
      self.cloud_i = 0
      self.track_sig = None
      self.trail_sig = None

    # --- common look-ahead point -------------------------------------------
    x_ref = float(np.clip(v_ego * LOOKAHEAD_TIME, LOOKAHEAD_MIN_X, LOOKAHEAD_MAX_X))
    debug["x_ref"] = x_ref

    # Every input the evidence and the vote use -- the plan, the lane lines, the
    # road edges, the radar -- arrives at 20 Hz, so all of it is recomputed at the
    # evidence rate and reused on the intervening control cycles.  Odometry, the
    # ego filter, the deviation dynamics and the rate limiter always run.
    ev_due = (self.model_ref is None or new_plan or self.t - self.model_ref_t >= EVIDENCE_PERIOD)

    # The model's own geometry at x_ref, not its lag-adjusted desiredCurvature:
    # evidence describes the road ahead, so it has to be compared like for like.
    if ev_due:
      model_ref = _poly_curvature(_poly_fit(plan_x_a, plan_y_a, 0.0, FIT_MAX_X), x_ref)
      if model_ref is None:
        model_ref = model
      d_ev = max(self.t - self.model_ref_t, 1e-6) if self.model_ref is not None else dt
      self.model_ref = model_ref
      self.model_ref_t = self.t
      if self.model_ref_smooth is None:
        self.model_ref_smooth = model_ref
      else:
        a = 1.0 - math.exp(-d_ev / MODEL_REF_SMOOTH_TAU) if MODEL_REF_SMOOTH_TAU > 0.0 else 1.0
        self.model_ref_smooth += a * (model_ref - self.model_ref_smooth)
      self.model_ref_hist.append((self.t, self.model_ref_smooth))
      while self.model_ref_hist and self.t - self.model_ref_hist[0][0] > 2.0 * MOVING_BASELINE_SECONDS:
        self.model_ref_hist.popleft()
    model_ref = self.model_ref
    debug["model_ref"] = model_ref

    # --- requirement 6a: the arming delay belongs to engagement, not to lanes
    engaged_edge = bool(lat_active) and not self.prev_lat_active
    self.prev_lat_active = bool(lat_active)
    if (not lat_active) or engaged_edge or steering_pressed or v_ego <= MIN_ACTIVE_SPEED:
      self.arm_timer = 0.0
    else:
      self.arm_timer += dt
    armed = self.arm_timer >= ARM_DELAY_SECONDS
    debug["armed"] = armed

    # --- requirement 6b: intentional path changes always pass through -------
    explicit_intent = bool(lane_change_active) or bool(desire_turn) or bool(nav_turn)
    blinker = bool(blinker_left) != bool(blinker_right)  # both on == hazards, not a manoeuvre
    if explicit_intent:
      self.intent_timer = 0.0
      self.intent_side = 0
    elif blinker and not self.cfg["blinker_directional"]:
      self.intent_timer = 0.0
      self.intent_side = 0
    elif blinker:
      self.intent_timer = 0.0
      self.intent_side = -1 if blinker_left else 1
    else:
      self.intent_timer += dt
    intent_active = self.intent_timer < INTENT_RELEASE_SECONDS
    debug["intent"] = intent_active

    # --- requirement 7b: authority is a continuous weight -------------------
    gate = float(self.cfg["laneless_prob_gate"])
    prob_w_raw = 1.0 - _ramp(max(lll_prob, rll_prob), gate - LANELESS_PROB_RAMP, gate + LANELESS_PROB_RAMP)
    # peak hold: rise at once, fall over PROB_WEIGHT_FALL_SECONDS, so a prob
    # rattling across the gate does not chatter the authority (7f)
    self.prob_w = max(prob_w_raw, self.prob_w - dt / PROB_WEIGHT_FALL_SECONDS, 0.0)
    prob_w = self.prob_w
    laneless_w = 1.0 - float(min(max(lane_mode_weight, 0.0), 1.0))
    speed_w = _ramp(v_ego, MIN_ACTIVE_SPEED, MIN_ACTIVE_SPEED + SPEED_RAMP)
    directional_intent = intent_active and self.intent_side != 0
    authority = (prob_w * laneless_w * speed_w
                 * (0.0 if steering_pressed else 1.0)
                 * (0.0 if not lat_active else 1.0)
                 * (0.0 if (intent_active and not directional_intent) else 1.0)
                 * (0.0 if not odo_ok else 1.0)
                 * (1.0 if armed else 0.0))
    self.authority = authority
    debug["authority"] = authority
    if authority <= 0.0 and not debug["reason"]:
      debug["reason"] = ("inactive" if not lat_active else
                         "steering_pressed" if steering_pressed else
                         "slow" if speed_w <= 0.0 else
                         "sensor_fault" if not odo_ok else
                         "intent" if intent_active else
                         "arming" if not armed else
                         "lanes_visible")

    # --- requirement 2: independent evidence --------------------------------
    # Computed while lat is active regardless of lanes (7a) so the decision is
    # continuous across the crossfade; skipped when stopped or disengaged.
    want_ev = bool(lat_active) and v_ego > 1.0
    if want_ev and (ev_due or self.ev_cache is None):
      ev_new: Dict[str, Tuple[Optional[float], float]] = {
        "lane_memory": self._ev_lane_memory(x_ref),
        "road_edges": self._ev_road_edges(edge_x_a, le_y_a, re_y_a, float(le_std), float(re_std), x_ref),
        "road_tracks": self._ev_road_tracks(x_ref),
        "lead_trail": self._ev_lead_trail(x_ref),
        "other_trails": self._ev_other_trails(x_ref),
        "ego_continuity": self._ev_ego_continuity(),
        "model_self": self._ev_model_self(x_ref),
      }
      # requirement 6e: a faulty source contributes no confidence at all
      if not odo_ok:
        ev_new = {k: (None, 0.0) for k in ev_new}
      else:
        if not radar_valid:
          for k in RADAR_EVIDENCE:
            ev_new[k] = (None, 0.0)
        if not model_ok:
          for k in MODEL_EVIDENCE:
            ev_new[k] = (None, 0.0)
      self.ev_cache = ev_new
      self.ev_cache_t = self.t
    ev = self.ev_cache if (want_ev and self.ev_cache is not None) else \
        {n: (None, 0.0) for n in EVIDENCE_NAMES}
    debug["ego_trend"] = self._ego_trend()

    # --- requirement 3: vote ------------------------------------------------
    base_ref = self._baseline_model_ref()
    support_strong = 0.0    # the model is demonstrably moving toward this evidence
    support_conf = 0.0      # the model agrees with it within tolerance
    contra_sign_conf = 0.0  # evidence says the road bends the other way
    contra_mag_conf = 0.0   # evidence says the road bends much more than the model does
    contra_sign_dir = 0.0
    contra_mag_dir = 0.0
    triggers: List[str] = []
    for name in EVIDENCE_NAMES:
      c_e, conf = ev.get(name, (None, 0.0))
      slot = debug["evidence"][name]
      slot["curvature"] = c_e
      slot["confidence"] = conf
      if c_e is None or conf < CONTRADICT_CONF_MIN:
        continue
      tol = EVIDENCE_TOL_BASE + EVIDENCE_TOL_FRAC * abs(c_e)
      err = model_ref - c_e
      significant = abs(c_e) > CURV_SIGNIFICANT
      # Both the plan's shape and the command actually being steered must point
      # the other way.  desiredCurvature also carries the correction back to the
      # path when the car is off-centre, so a command that already turns with the
      # evidence is not wrong even when the plan's shape is (mock drive 154 seg11).
      sign_conflict = significant and model_ref * c_e <= 0.0 and model * c_e <= 0.0

      smooth_err = (self.model_ref_smooth if self.model_ref_smooth is not None else model_ref) - c_e
      closed = 0.0 if base_ref is None else abs(base_ref - c_e) - abs(smooth_err)
      moving_closer = closed > MOVING_CLOSER_MIN
      if moving_closer and sign_conflict:
        # Closing the gap is not enough while still on the wrong side: require a
        # rate that actually reaches agreement within CLOSE_HORIZON_SECONDS.
        rate = closed / MOVING_BASELINE_SECONDS
        moving_closer = rate * CLOSE_HORIZON_SECONDS >= (abs(err) - tol)

      if moving_closer:
        # The model is genuinely changing toward this evidence: exactly the case
        # that must never be delayed, whatever else is saying.
        slot["vote"] = VOTE_SUPPORT
        support_strong = max(support_strong, conf)
      elif abs(err) <= tol:
        slot["vote"] = VOTE_SUPPORT
        support_conf = max(support_conf, conf)
      elif sign_conflict:
        # The model points the other way: the seg5 failure signature.
        slot["vote"] = VOTE_CONTRADICT
        if conf > contra_sign_conf:
          contra_sign_dir = math.copysign(1.0, c_e)
        contra_sign_conf = max(contra_sign_conf, conf)
        triggers.append(name)
      elif (significant and EVIDENCE_UNDERCUT[name] and abs(model_ref) < UNDERCUT_FRAC * abs(c_e)
            and abs(model) < UNDERCUT_FRAC * abs(c_e)):
        # The model barely turns while road geometry says the road clearly bends.
        slot["vote"] = VOTE_CONTRADICT
        if conf > contra_mag_conf:
          contra_mag_dir = math.copysign(1.0, c_e)
        contra_mag_conf = max(contra_mag_conf, conf)
        triggers.append(name)
      else:
        slot["vote"] = VOTE_NEUTRAL

    debug["support_conf"] = support_conf
    debug["support_strong_conf"] = support_strong
    debug["contradict_conf"] = max(contra_sign_conf, contra_mag_conf)
    debug["triggers"] = triggers

    # --- requirement 7d: pass/hold is a continuous weight -------------------
    w_sign = contra_sign_conf - SUPPORT_GAIN_SIGN * support_conf
    w_mag = contra_mag_conf - SUPPORT_GAIN_MAG * support_conf
    contradict_sign = contra_sign_dir if w_sign >= w_mag else contra_mag_dir
    w_raw = 0.0 if support_strong > 0.0 else max(w_sign, w_mag)
    w_raw = float(min(max(w_raw, 0.0), 1.0))
    if w_raw > 0.0:
      self.contradict_sign = contradict_sign
    if w_raw >= self.hold_weight:
      self.hold_weight = w_raw                      # engage at once (safety direction)
    else:
      a = 1.0 - math.exp(-dt / max(float(self.cfg["hold_release_tau"]), 1e-3))
      self.hold_weight += a * (w_raw - self.hold_weight)   # release smoothly, no per-frame flip
    hold_w = self.hold_weight
    debug["hold_weight"] = hold_w

    if hold_w > CONTRADICT_CONF_MIN:
      self.hold_timer += dt
    else:
      self.hold_timer = 0.0
    debug["hold_time"] = self.hold_timer

    # A directional blinker only exempts changes toward the signalled side.
    if directional_intent and self.intent_side * (model_ref - (base_ref if base_ref is not None else model_ref)) > 0.0:
      hold_w = 0.0

    # --- requirement 7c: the deviation state moves smoothly -----------------
    d_model = 0.0 if self.prev_model is None else model - self.prev_model
    self.prev_model = model

    # Holding == absorbing the part of the model's change that moves AWAY from the
    # contradicting evidence.  A change toward that evidence is the correct change
    # we must never delay, so it is passed through untouched.  Only contradiction
    # present *now* absorbs: the smoothly released weight keeps holding what was
    # already absorbed (slow decay below) but must not soak up a new change once
    # the evidence has stopped objecting -- in a mock drive of 154 seg11 that
    # residual delayed a correct reversal by ~0.27 m/s^2.
    absorb = min(hold_w, w_raw)
    if self.contradict_sign != 0.0 and d_model * self.contradict_sign > 0.0:
      absorb = 0.0
      # The model is coming back toward the evidence: a held output simply waits
      # for it, so the deviation unwinds by the model's step instead of staying
      # behind as an overshoot (noise faults: +2 % error without this).
      if self.deviation * self.contradict_sign > 0.0:
        self.deviation -= math.copysign(min(abs(self.deviation), abs(d_model)), self.deviation)
    self.deviation -= absorb * d_model

    conv = _ramp(self.hold_timer - float(self.cfg["converge_after"]), 0.0, CONVERGE_RAMP_SECONDS)
    tau = DECAY_TAU_SECONDS + hold_w * (float(self.cfg["hold_tau"]) - DECAY_TAU_SECONDS)
    tau = (1.0 - conv) * tau + conv * CONVERGE_TAU_SECONDS
    self.deviation *= math.exp(-dt / max(tau, 1e-3))

    lim = float(self.cfg["max_deviation"]) / max(v_ego * v_ego, 1.0)
    self.deviation = float(min(max(self.deviation, -lim), lim))
    if not math.isfinite(self.deviation) or abs(self.deviation) < RESET_EPS:
      self.deviation = 0.0
    debug["deviation_state"] = self.deviation

    # --- requirement 7e: rate limit the actual effect -----------------------
    # Every gate edge and every authority change goes through this limiter, so
    # the verifier can only ever ramp -- it cannot step and it cannot add jerk.
    effect_target = authority * self.deviation
    max_step = DEV_JERK_LIMIT / max(v_ego * v_ego, 1.0) * dt
    step = float(min(max(effect_target - self.effect, -max_step), max_step))
    self.effect = float(min(max(self.effect + step, -lim), lim))
    if not math.isfinite(self.effect) or abs(self.effect) < RESET_EPS:
      self.effect = 0.0

    out = model + self.effect
    if not math.isfinite(out):
      out = model
      self.effect = 0.0
    self.output = out

    debug["deviation"] = self.effect
    debug["deviation_lat_accel"] = self.effect * max(v_ego * v_ego, 1.0)
    if abs(self.effect) > 0.0:
      debug["decision"] = DECISION_CONVERGE if conv > 0.5 else (
        DECISION_HOLD if hold_w > CONTRADICT_CONF_MIN else DECISION_PASS)

    # --- requirement 7c: reset only when inactive AND already neutral -------
    if not lat_active and abs(self.effect) < RESET_EPS and abs(self.deviation) < RESET_EPS:
      self.effect = 0.0
      self.deviation = 0.0
      self.hold_weight = 0.0
      self.hold_timer = 0.0
      self.contradict_sign = 0.0

    return out, debug
