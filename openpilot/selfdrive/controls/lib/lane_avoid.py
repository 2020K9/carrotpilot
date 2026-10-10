"""Bounded lateral avoidance component for lane mode (lanemode_avoid_prompt_v3).

Lane mode replaces the model (laneless) path with
  d_prob * lane_path + (1 - d_prob) * model_path            (lane_planner_2.get_d_path)
so the share d_prob * (model_path - lane_path) of a model sidestep is removed.
This module decides, frame by frame, whether a bounded part of that removed share
may be added back as a uniform lateral offset of the lane-mode path.

Every limit, coverage, freshness and return/risk policy below is an approval-bound
setting. None of them is approved: LaneAvoidConfig() has every value unset, so
validate() reports blockers and the controller stays DISABLED (offset exactly 0).
Do not fill these values from other features' constants.

Coordinates: modelV2 path/laneLines/roadEdges y is positive to the RIGHT
(lane_planner_2: path_from_left_lane = lll_y + width/2). radarState yRel is
positive to the LEFT. A LEFT avoidance therefore has a negative model-y offset.

Observation states: "not detected" is never "clear". A side is CLEAR only when
every required source is CLEAR (supported, valid, fresh, no detection) and the
approved coverage of those sources spans the approved required region.

v3.1 (lanemode_avoid_prompt_v3_1):
- Safety checks cover the whole consumed path. The offset is a uniform shift of every
  path point (lateral_planner), so road-edge room and the side space are evaluated over
  all path points, the segments between them and the approved body extents, not only
  over candidate_x_m. The fixed required region stays as an additional constraint.
- Freshness comes from real message times: evaluation time t and the model message
  time model_t share one monotonic clock; repeated, reversed, future or missing times
  are invalid, never age 0.
- Every frame's applied offset is chosen inside the intersection of all constraints
  (edge room, max offset, entry/return rate, rate-change continuity, no growth without
  permission, no return without a clear return side, no side crossing). There is no
  forced-zero rate exception. An empty intersection is CONFLICT: its policy is not
  approved, so blockers() keeps the controller disabled.
- Approval is re-validated every frame; any change resets all avoidance state.
"""
import math
from dataclasses import dataclass, field

import numpy as np

LEFT = 'left'
RIGHT = 'right'

# model-y sign of a movement toward each side
SIDE_SIGN = {LEFT: -1.0, RIGHT: 1.0}

OCCUPIED = 'occupied'
CLEAR = 'clear'
UNKNOWN = 'unknown'

# required observation sources for a side; the radar side lists are built from the
# model path (radar_motion d_path) and are NOT independent model observations
SRC_BSD = 'bsd'
SRC_RADAR = 'radar'
SRC_MODEL = 'model'
REQUIRED_SOURCES = (SRC_BSD, SRC_RADAR, SRC_MODEL)

# controller states
DISABLED = 'disabled'
STANDBY = 'standby'        # permission pending
AVOID = 'avoid'
RETURN = 'return'          # new-avoidance target 0, residual offset remains
RETURN_RISK = 'return_risk'
CONFLICT = 'constraint_conflict'  # no offset satisfies every constraint this frame

# Return-risk policies (hold / decelerate / driver warning ...) are not approved.
# The empty tuple keeps validate() failing until a reviewed policy is added here.
APPROVED_RETURN_RISK_POLICIES: tuple = ()
APPROVED_CENTER_INVALID_POLICIES: tuple = ()
# v3.1: none of these are approved either; each empty tuple is a blocker.
# - occupancy prediction: objects moving into the required space during the manoeuvre
# - body geometry: the band model (half width + clearance at every path point, front/rear
#   extents along x, no heading-induced corner offset) used by required_space/road_edge_allowance
# - constraint conflict: what to command when CONFLICT occurs
APPROVED_OCCUPANCY_PREDICTION_POLICIES: tuple = ()
APPROVED_BODY_GEOMETRY_MODELS: tuple = ()
APPROVED_CONSTRAINT_CONFLICT_POLICIES: tuple = ()


def other_side(side):
  return RIGHT if side == LEFT else LEFT


def _finite(*values):
  return all(v is not None and math.isfinite(v) for v in values)


@dataclass(frozen=True)
class Coverage:
  """Approved observation area of one source on one side (ego-relative metres).

  x: longitudinal (negative = behind ego), lat: outward distance from the ego
  centreline on that side. Only evidence-backed, approved coverage belongs here.
  """
  x_min: float
  x_max: float
  lat_min: float
  lat_max: float


@dataclass(frozen=True)
class LaneAvoidConfig:
  enabled: bool = False
  # limits (unit, all unapproved)
  max_offset_m: float | None = None             # m, |applied offset| limit
  entry_rate_mps: float | None = None           # m/s, magnitude increase rate
  return_rate_mps: float | None = None          # m/s, magnitude decrease rate
  max_rate_change_mps2: float | None = None     # m/s^2, continuity limit on offset rate
  candidate_x_m: tuple | None = None            # (min, max) m, path x range used for the candidate
  candidate_deadband_m: float | None = None     # m, minimum |candidate| treated as a sidestep
  max_abs_curvature: float | None = None        # 1/m, above this avoidance is not separable from a curve
  vehicle_half_width_m: float | None = None     # m
  road_edge_margin_m: float | None = None       # m, clearance between body and road edge
  road_edge_std_max_m: float | None = None      # m, roadEdgeStds above this -> unknown
  required_region_x_m: tuple | None = None      # (min, max) m, side space that must be observed
  required_region_lat_m: tuple | None = None    # (min, max) m outward from centreline
  coverage: dict | None = None                  # {(source, side): Coverage}
  max_age_s: dict | None = None                 # {'bsd'|'radar'|'model'|'road_edge'|'model_path': s}
  max_input_gap_s: float | None = None          # s, larger dt revokes permission
  entry_confirm_frames: int | None = None       # consecutive permitted frames before AVOID
  reentry_wait_s: float | None = None           # s after a revocation before a new AVOID
  return_risk_policy: str | None = None
  center_invalid_policy: str | None = None
  # v3.1 (all unapproved)
  vehicle_front_m: float | None = None          # m, body extent ahead of a path point
  vehicle_rear_m: float | None = None           # m, body extent behind the current reference point
  side_clearance_m: float | None = None         # m, lateral clearance to side objects
  occupancy_prediction_policy: str | None = None
  body_geometry_model: str | None = None
  constraint_conflict_policy: str | None = None

  def blockers(self):
    out = []
    if not self.enabled:
      out.append('feature_disabled')
    positive = ('max_offset_m', 'entry_rate_mps', 'return_rate_mps', 'max_rate_change_mps2',
                'candidate_deadband_m', 'max_abs_curvature', 'vehicle_half_width_m',
                'road_edge_margin_m', 'road_edge_std_max_m', 'max_input_gap_s', 'reentry_wait_s',
                'vehicle_front_m', 'vehicle_rear_m', 'side_clearance_m')
    for name in positive:
      v = getattr(self, name)
      if not (isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) and v > 0.0):
        out.append(f'{name}_unapproved')
    for name in ('candidate_x_m', 'required_region_x_m', 'required_region_lat_m'):
      v = getattr(self, name)
      if not (isinstance(v, tuple) and len(v) == 2 and _finite(*v) and v[0] < v[1]):
        out.append(f'{name}_unapproved')
    if not (isinstance(self.entry_confirm_frames, int) and self.entry_confirm_frames >= 1):
      out.append('entry_confirm_frames_unapproved')
    if not isinstance(self.coverage, dict) or not all(
        isinstance(self.coverage.get((s, side)), Coverage) for s in REQUIRED_SOURCES for side in (LEFT, RIGHT)):
      out.append('coverage_unapproved')
    ages = self.max_age_s if isinstance(self.max_age_s, dict) else {}
    for key in REQUIRED_SOURCES + ('road_edge', 'model_path'):
      v = ages.get(key)
      if not (isinstance(v, (int, float)) and math.isfinite(v) and v > 0.0):
        out.append(f'max_age_{key}_unapproved')
    if self.return_risk_policy not in APPROVED_RETURN_RISK_POLICIES:
      out.append('return_risk_policy_unapproved')
    if self.center_invalid_policy not in APPROVED_CENTER_INVALID_POLICIES:
      out.append('center_invalid_policy_unapproved')
    if self.occupancy_prediction_policy not in APPROVED_OCCUPANCY_PREDICTION_POLICIES:
      out.append('occupancy_prediction_policy_unapproved')
    if self.body_geometry_model not in APPROVED_BODY_GEOMETRY_MODELS:
      out.append('body_geometry_model_unapproved')
    if self.constraint_conflict_policy not in APPROVED_CONSTRAINT_CONFLICT_POLICIES:
      out.append('constraint_conflict_policy_unapproved')
    return out


@dataclass(frozen=True)
class SourceReading:
  """One observation source on one side. None means unknown, never False."""
  supported: bool | None = None
  valid: bool | None = None
  age_s: float | None = None
  detected: bool | None = None


def evaluate_source(reading, max_age_s):
  # A positive detection is a veto whatever the validity/age/settings (BSD true included).
  if reading is None:
    return UNKNOWN
  if reading.detected is True:
    return OCCUPIED
  if (reading.detected is False and reading.supported is True and reading.valid is True and
      _finite(reading.age_s, max_age_s) and 0.0 <= reading.age_s <= max_age_s):
    return CLEAR
  return UNKNOWN


def region_covered(coverages, region_x, region_lat):
  """True when the union of the given coverages spans the required rectangle."""
  if region_x is None or region_lat is None:
    return False
  spans = sorted((c.x_min, c.x_max) for c in coverages
                 if c is not None and c.lat_min <= region_lat[0] and c.lat_max >= region_lat[1])
  reach = region_x[0]
  for x0, x1 in spans:
    if x0 > reach:
      return False
    reach = max(reach, x1)
    if reach >= region_x[1]:
      return True
  return False


@dataclass(frozen=True)
class RequiredSpace:
  """Ego-relative rectangle on one side (same axes as Coverage) that must be observed."""
  x_min: float
  x_max: float
  lat_min: float
  lat_max: float


def _path_arrays(path_x, base_y):
  """Validated (x, y) of the consumed path, or None. x must be strictly increasing so
  that segments between points are well defined."""
  try:
    x = np.asarray(path_x, dtype=float)
    y = np.asarray(base_y, dtype=float)
  except (TypeError, ValueError):
    return None
  if x.ndim != 1 or x.shape != y.shape or x.size < 2:
    return None
  if not (np.all(np.isfinite(x)) and np.all(np.isfinite(y))) or np.any(np.diff(x) <= 0.0):
    return None
  return x, y


def required_x_range(path_x, cfg):
  """Longitudinal span of the space the body sweeps along the whole consumed path:
  from the body rear at the current position to the body front at the last path point.
  It does not depend on the offset. None when not computable (never a smaller range)."""
  if not _finite(cfg.vehicle_front_m, cfg.vehicle_rear_m):
    return None
  try:
    x = np.asarray(path_x, dtype=float)
  except (TypeError, ValueError):
    return None
  if x.ndim != 1 or x.size < 2 or not np.all(np.isfinite(x)) or np.any(np.diff(x) <= 0.0):
    return None
  return min(float(x[0]), 0.0) - cfg.vehicle_rear_m, float(x[-1]) + cfg.vehicle_front_m


def required_space(path_x, base_y, offsets, side, cfg):
  """Space on `side` swept by the body while the consumed path (base_y + uniform offset)
  takes any of `offsets` (current applied, target, 0 for the return). Every path point
  and, since the rectangle bounds each segment's end points, every segment between them
  is inside. Moving objects need the (unapproved) occupancy prediction policy."""
  xr = required_x_range(path_x, cfg)
  arrays = _path_arrays(path_x, base_y)
  if xr is None or arrays is None or not _finite(cfg.vehicle_half_width_m, cfg.side_clearance_m):
    return None
  offs = np.asarray(offsets, dtype=float)
  if offs.ndim != 1 or offs.size == 0 or not np.all(np.isfinite(offs)):
    return None
  u = SIDE_SIGN[side] * (arrays[1][None, :] + offs[:, None])
  lat_max = max(float(np.max(u)) + cfg.vehicle_half_width_m + cfg.side_clearance_m, 0.0)
  return RequiredSpace(xr[0], xr[1], 0.0, lat_max)


def evaluate_side(readings, side, cfg, spaces=()):
  """readings: {source: SourceReading}. Returns (state, {source: state}).
  Coverage must span the fixed required region AND every space in `spaces`; a None
  space (not computable) is UNKNOWN. The controller always passes its dynamic space."""
  ages = cfg.max_age_s if isinstance(cfg.max_age_s, dict) else {}
  per = {s: evaluate_source(readings.get(s), ages.get(s)) for s in REQUIRED_SOURCES}
  if any(v == OCCUPIED for v in per.values()):
    return OCCUPIED, per
  if not all(v == CLEAR for v in per.values()):
    return UNKNOWN, per
  cov = cfg.coverage if isinstance(cfg.coverage, dict) else {}
  covs = [cov.get((s, side)) for s in REQUIRED_SOURCES]
  if not region_covered(covs, cfg.required_region_x_m, cfg.required_region_lat_m):
    return UNKNOWN, per
  for sp in spaces:
    if sp is None or not region_covered(covs, (sp.x_min, sp.x_max), (sp.lat_min, sp.lat_max)):
      return UNKNOWN, per
  return CLEAR, per


def radar_side_reading(leads, valid, age_s, region_x):
  """radarState side list (leadLeft/leadsLeft or right). Any status lead inside the
  required x range is a detection; non-finite values make the reading unknown.
  The lists are classified on the model path and are not model observations."""
  if leads is None or region_x is None:
    return SourceReading(supported=None, valid=valid, age_s=age_s, detected=None)
  detected = False
  for lead in leads:
    if not bool(getattr(lead, 'status', False)):
      continue
    d_rel = getattr(lead, 'dRel', None)
    if not _finite(d_rel):
      return SourceReading(supported=True, valid=valid, age_s=age_s, detected=None)
    if region_x[0] <= d_rel <= region_x[1]:
      detected = True
  return SourceReading(supported=True, valid=valid, age_s=age_s, detected=detected)


def avoid_candidate(path_x, model_y, lane_y, d_prob, cfg):
  """Signed sidestep removed by lane mode: d_prob * (model_y - lane_y) on the same
  samples. Returns (value, reason); value None when not a single-direction sidestep."""
  if cfg.candidate_x_m is None or cfg.candidate_deadband_m is None:
    return None, 'candidate_unapproved'
  try:
    x = np.asarray(path_x, dtype=float)
    my = np.asarray(model_y, dtype=float)
    ly = np.asarray(lane_y, dtype=float)
  except (TypeError, ValueError):
    return None, 'candidate_bad_input'
  if x.ndim != 1 or x.shape != my.shape or x.shape != ly.shape or x.size < 2:
    return None, 'candidate_shape'
  if not (_finite(d_prob) and 0.0 <= d_prob <= 1.0):
    return None, 'candidate_d_prob'
  if not (np.all(np.isfinite(x)) and np.all(np.isfinite(my)) and np.all(np.isfinite(ly))):
    return None, 'candidate_nonfinite'
  if np.any(np.diff(x) < 0.0):
    return None, 'candidate_x_not_monotonic'
  sel = (x >= cfg.candidate_x_m[0]) & (x <= cfg.candidate_x_m[1])
  if not np.any(sel):
    return None, 'candidate_empty_region'
  comp = d_prob * (my[sel] - ly[sel])
  peak = float(comp[np.argmax(np.abs(comp))])
  if abs(peak) < cfg.candidate_deadband_m:
    return None, 'candidate_below_deadband'
  significant = comp[np.abs(comp) >= cfg.candidate_deadband_m]
  if np.any(np.sign(significant) != np.sign(peak)):
    return None, 'candidate_mixed_direction'
  return peak, ''


def road_edge_allowance(path_x, base_y, edge_x, edge_y, edge_std, edge_age_s, side, cfg):
  """Lateral room (m) toward `side` beyond body half width and approved margin,
  measured from the lane-mode base path over the WHOLE consumed path (v3.1): every
  path point, every segment between points and the body front beyond the last point.
  Path and edge are piecewise linear, so their distance is minimal at a knot of either;
  all knots in range are checked. The edge must be observed over the whole span (no
  extrapolation). Returns (state, allowance). roadEdges is a model estimate, not a
  guardrail sensor."""
  ages = cfg.max_age_s if isinstance(cfg.max_age_s, dict) else {}
  max_age = ages.get('road_edge')
  if not _finite(cfg.vehicle_half_width_m, cfg.road_edge_margin_m, cfg.road_edge_std_max_m, max_age,
                 cfg.vehicle_front_m):
    return UNKNOWN, 0.0
  if not (_finite(edge_std, edge_age_s) and 0.0 <= edge_age_s <= max_age and edge_std <= cfg.road_edge_std_max_m):
    return UNKNOWN, 0.0
  arrays = _path_arrays(path_x, base_y)
  try:
    ex = np.asarray(edge_x, dtype=float)
    ey = np.asarray(edge_y, dtype=float)
  except (TypeError, ValueError):
    return UNKNOWN, 0.0
  if arrays is None or ex.shape != ey.shape or ex.ndim != 1 or ex.size < 2:
    return UNKNOWN, 0.0
  x, by = arrays
  if not (np.all(np.isfinite(ex)) and np.all(np.isfinite(ey))) or np.any(np.diff(ex) <= 0.0):
    return UNKNOWN, 0.0
  x_end = float(x[-1]) + cfg.vehicle_front_m
  # no extrapolation: the edge must be observed over the whole consumed span
  if x[0] < ex[0] or x_end > ex[-1]:
    return UNKNOWN, 0.0
  knots = np.union1d(np.append(x, x_end), ex[(ex >= x[0]) & (ex <= x_end)])
  # beyond the last point the body front is taken at the last path y (body geometry model)
  path_at = np.interp(knots, x, by)
  edge_at = np.interp(knots, ex, ey)
  dist = SIDE_SIGN[side] * (edge_at - path_at)
  allowance = float(np.min(dist)) - cfg.vehicle_half_width_m - cfg.road_edge_margin_m
  if allowance <= 0.0:
    return OCCUPIED, 0.0
  return CLEAR, allowance


@dataclass
class AvoidInputs:
  t: float | None                       # s, evaluation time (monotonic clock shared with model_t)
  center_valid: bool                    # lane-mode base path is in use and valid
  path_x: object = None
  model_y: object = None                # laneless path y before lane blending
  lane_y: object = None                 # lane path y on the same samples (None if not used)
  base_y: object = None                 # lane-mode path y before this offset (centre reference)
  d_prob: float | None = None
  model_valid: bool = False
  model_age_s: float | None = None      # s, builder's t - model_t; checked together with model_t
  lane_change_active: bool = True       # desire / lane change / ATC
  driver_steering: bool = True
  measured_curvature: float | None = None
  side_readings: dict = field(default_factory=dict)   # {side: {source: SourceReading}}
  road_edges: dict = field(default_factory=dict)      # {side: (edge_x, edge_y, std, age_s)}
  model_t: float | None = None          # s, modelV2 message time on the same clock as t
  clock_verified: bool = False          # t and every message time come from one verified clock


@dataclass
class AvoidOutput:
  state: str = DISABLED
  permitted: bool = False               # new avoidance start/hold/expand allowed this frame
  target: float = 0.0                   # new-avoidance target (model y, m)
  applied: float = 0.0                  # offset actually added to the consumed path (model y, m)
  rate: float = 0.0                     # m/s, (applied - previous applied) / dt of this frame
  return_move_permitted: bool = False
  avoid_side: str | None = None
  candidate: float | None = None
  edge_allowance: float | None = None
  edge_room: dict = field(default_factory=dict)       # {side: room m, 0.0 occupied, None unknown}
  required_space: dict = field(default_factory=dict)  # {side: RequiredSpace or None}
  conflict: tuple = ()
  reasons: tuple = ()
  side_state: dict = field(default_factory=dict)      # {side: state}
  source_state: dict = field(default_factory=dict)    # {side: {source: state}}
  edge_state: dict = field(default_factory=dict)      # {side: state}
  blockers: tuple = ()


class LaneAvoidController:
  """State ownership: applied, rate, motion_sign, avoid_side, confirm, last_t and state
  belong to one lane-mode episode. reset() clears them on any approval or config change
  (refresh) and whenever the lane-mode path is not consumed (mode_inactive). revoked_t
  (start of the re-entry wait) is kept across those resets so leaving and re-entering
  does not skip the wait. last_model_t belongs to the model message stream and survives
  resets, so an old message is never reused after one."""

  def __init__(self, cfg=None):
    self.cfg = cfg if cfg is not None else LaneAvoidConfig()
    self._cfg_seen = self.cfg
    self.blockers = tuple(self.cfg.blockers())
    self.active = not self.blockers
    self.last_model_t = None
    self.note = ()
    self.reset()

  def reset(self):
    self.state = STANDBY if self.active else DISABLED
    self.applied = 0.0
    self.rate = 0.0
    self.motion_sign = 0.0
    self.avoid_side = None
    self.last_t = None
    self.confirm = 0
    self.revoked_t = None

  def refresh(self):
    """Re-validate approval now instead of reusing the construction-time result. Any
    change of the config object or of its blockers resets all avoidance state, so a
    re-activated controller starts at zero offset and needs new permission."""
    blockers = tuple(self.cfg.blockers())
    if self.cfg is not self._cfg_seen or blockers != self.blockers:
      residual = self.applied != 0.0 or self.rate != 0.0
      engaged = self.state not in (STANDBY, DISABLED) or residual
      # the re-entry wait survives the reset (counted from the change if engaged)
      revoked = self.last_t if engaged and self.last_t is not None else self.revoked_t
      self._cfg_seen, self.blockers = self.cfg, blockers
      self.active = not blockers
      self.reset()
      self.revoked_t = revoked
      # Dropping a residual at runtime deactivation is not an approved safety action;
      # the reason is reported on the next active frame (and blocks that frame). Notes
      # accumulate so a later change (e.g. re-enable) cannot hide an earlier residual drop.
      new = ('approval_changed_reset',) + (('approval_change_with_residual_unverified',) if residual else ())
      self.note = self.note + tuple(n for n in new if n not in self.note)
    return self.active

  def radar_region_x(self, path_x):
    """x range for radar_side_reading: the fixed region united with the dynamic span."""
    fixed = self.cfg.required_region_x_m
    dyn = required_x_range(path_x, self.cfg)
    if not (isinstance(fixed, tuple) and len(fixed) == 2 and _finite(*fixed)) or dyn is None:
      return None
    return min(fixed[0], dyn[0]), max(fixed[1], dyn[1])

  def mode_inactive(self, t):
    """The lane-mode path is not consumed this frame, so the offset's episode ends.
    Re-entry starts at zero offset and needs new observations, confirmation and, after
    any engagement, the re-entry wait. This resets state; it is not a steering command,
    and the transition of a non-zero residual is not validated (reported reason)."""
    engaged = self.state not in (STANDBY, DISABLED) or self.applied != 0.0 or self.rate != 0.0
    residual = self.applied != 0.0 or self.rate != 0.0
    revoked = self.revoked_t
    self.reset()
    # keep the re-entry wait across repeated inactive frames (reset() clears revoked_t)
    self.revoked_t = t if engaged and _finite(t) else revoked
    reasons = ('lane_mode_inactive_reset',) + (('mode_exit_with_residual_unverified',) if residual else ())
    return AvoidOutput(state=self.state, blockers=self.blockers, reasons=reasons)

  def update(self, inp):
    if not self.refresh():
      return AvoidOutput(blockers=self.blockers)
    cfg = self.cfg
    out = AvoidOutput(blockers=self.blockers)
    reasons = list(self.note)
    self.note = ()

    # evaluation time: reversal, duplicate or gap revokes permission; no motion this frame
    dt = None
    if not _finite(inp.t):
      reasons.append('time_invalid')
    elif self.last_t is not None and inp.t <= self.last_t:
      reasons.append('time_not_increasing')
    elif self.last_t is not None and inp.t - self.last_t > cfg.max_input_gap_s:
      reasons.append('time_gap')
    elif self.last_t is not None:
      dt = inp.t - self.last_t
    if _finite(inp.t):
      self.last_t = inp.t if self.last_t is None else max(self.last_t, inp.t)

    # message time: a new model message on the verified clock, never from the future
    if not inp.clock_verified:
      reasons.append('clock_unverified')
    if not _finite(inp.model_t):
      reasons.append('model_time_invalid')
    else:
      if self.last_model_t is not None and inp.model_t <= self.last_model_t:
        reasons.append('model_not_new')
      if _finite(inp.t) and inp.model_t > inp.t:
        reasons.append('model_time_future')
      self.last_model_t = inp.model_t if self.last_model_t is None else max(self.last_model_t, inp.model_t)

    if not inp.center_valid:
      # No safe reference to return to; this is handed to the (unapproved) centre-invalid
      # policy. Lane mode not in use resets the episode instead (mode_inactive).
      reasons.append('center_invalid')

    candidate, why = (None, 'no_lane_path')
    if inp.lane_y is not None and inp.model_y is not None:
      candidate, why = avoid_candidate(inp.path_x, inp.model_y, inp.lane_y, inp.d_prob, cfg)
    out.candidate = candidate
    if why:
      reasons.append(why)
    want_side = None if candidate is None else (RIGHT if candidate > 0.0 else LEFT)

    # road edges over the whole consumed path; room is how far the uniform offset may go
    edges, room = {}, {}
    for side in (LEFT, RIGHT):
      e = inp.road_edges.get(side)
      if e is None or inp.base_y is None:
        edges[side], room[side] = UNKNOWN, 0.0
      else:
        edges[side], room[side] = road_edge_allowance(inp.path_x, inp.base_y, e[0], e[1], e[2], e[3], side, cfg)
    out.edge_state = edges
    out.edge_room = {s: room[s] if edges[s] == CLEAR else (0.0 if edges[s] == OCCUPIED else None) for s in room}
    cap = {s: min(cfg.max_offset_m, room[s]) if edges[s] == CLEAR else 0.0 for s in room}
    mag = min(abs(candidate), cfg.max_offset_m, room[want_side]) if want_side is not None else 0.0

    # side space for every offset this frame may command: 0, the current one, the target
    offsets = [0.0, self.applied] + ([SIDE_SIGN[want_side] * mag] if want_side is not None else [])
    spaces = {s: required_space(inp.path_x, inp.base_y, offsets, s, cfg) for s in (LEFT, RIGHT)}
    out.required_space = spaces
    if any(sp is None for sp in spaces.values()):
      reasons.append('required_space_unavailable')
    sides, sources = {}, {}
    for side in (LEFT, RIGHT):
      sides[side], sources[side] = evaluate_side(inp.side_readings.get(side, {}), side, cfg, (spaces[side],))
    out.side_state, out.source_state = sides, sources

    max_model = cfg.max_age_s['model_path']
    model_ok = (inp.model_valid and _finite(inp.model_age_s, inp.t, inp.model_t) and
                0.0 <= inp.model_age_s <= max_model and 0.0 <= inp.t - inp.model_t <= max_model)
    if not model_ok:
      reasons.append('model_invalid_or_stale')
    if inp.lane_change_active:
      reasons.append('lane_change_or_desire')
    if inp.driver_steering:
      reasons.append('driver_steering')
    if not (_finite(inp.measured_curvature) and abs(inp.measured_curvature) <= cfg.max_abs_curvature):
      reasons.append('curve_not_separable')

    # direction: a shift toward side S is avoidance only with positive evidence of an
    # obstacle on the other side and observed-clear space on S
    if want_side is not None:
      if self.avoid_side is not None and want_side != self.avoid_side:
        reasons.append('direction_reversal')
      if sides[other_side(want_side)] != OCCUPIED:
        reasons.append('no_obstacle_evidence_opposite')
      if sides[want_side] != CLEAR:
        reasons.append(f'avoid_side_{sides[want_side]}')
      if edges[want_side] != CLEAR:
        reasons.append(f'avoid_side_edge_{edges[want_side]}')
    if self.revoked_t is not None and self.state == STANDBY and (
        dt is None or self.last_t - self.revoked_t < cfg.reentry_wait_s):
      reasons.append('reentry_wait')

    gate_ok = not reasons
    self.confirm = self.confirm + 1 if gate_ok else 0

    permitted = False
    target = 0.0
    if gate_ok and (self.state == AVOID or self.confirm >= cfg.entry_confirm_frames) and \
       self.state in (STANDBY, AVOID):
      permitted = True
      target = SIDE_SIGN[want_side] * mag
      self.avoid_side = want_side
      self.state = AVOID
    elif self.state == AVOID:
      self.state = RETURN
      self.revoked_t = self.last_t

    # return side = where the original obstacle was; its space and edge must be
    # observed clear for any movement that reduces |applied|. The avoid side is
    # still evaluated and reported every frame.
    return_ok = False
    if self.avoid_side is not None:
      ret = other_side(self.avoid_side)
      return_ok = sides[ret] == CLEAR and edges[ret] == CLEAR and inp.center_valid and dt is not None

    s = self.motion_sign
    if s != 0.0 and max(s * target, 0.0) < s * self.applied and not return_ok:
      # Reducing |applied| would move toward the original obstacle side without
      # observed-clear space. The approved risk policy would act here; none is
      # approved, so blockers() keeps the controller disabled. Permission is revoked
      # and _step refuses the reduction; this is not an approved hold.
      if self.state == AVOID:
        self.revoked_t = self.last_t
      self.state = RETURN_RISK
      permitted, target = False, 0.0
      reasons.append('return_risk_policy_unapproved')
    elif self.state in (RETURN_RISK, CONFLICT):
      self.state = RETURN

    conflict = ()
    if dt is None:
      new_applied = self.applied
      if self.rate != 0.0:
        conflict = ('time_fault_during_motion',)
    else:
      new_applied, conflict = self._step(target, dt, permitted, return_ok, cap)
    if conflict:
      # No offset satisfies every constraint this frame. The response belongs to the
      # unapproved constraint_conflict_policy (blockers() keeps production disabled).
      # This placeholder only refuses movement; it is not an approved hold and breaks
      # the continuity limit whenever the rate was non-zero.
      if self.state == AVOID:
        self.revoked_t = self.last_t
      self.state = CONFLICT
      permitted, target = False, 0.0
      new_applied = self.applied
      reasons.append('constraint_conflict_policy_unapproved')
      reasons.extend(conflict)
    new_rate = (new_applied - self.applied) / dt if dt is not None else 0.0
    out.return_move_permitted = abs(new_applied) < abs(self.applied) and return_ok
    self.applied, self.rate = new_applied, new_rate
    if new_applied != 0.0:
      self.motion_sign = math.copysign(1.0, new_applied)
    elif new_rate == 0.0:
      self.motion_sign = 0.0

    if self.state in (RETURN, RETURN_RISK, CONFLICT) and self.applied == 0.0 and self.rate == 0.0:
      self.state = STANDBY
      self.avoid_side = None
      self.motion_sign = 0.0
      self.confirm = 0

    out.state = self.state
    out.permitted = permitted
    out.target = target
    out.applied = self.applied
    out.rate = self.rate
    out.avoid_side = self.avoid_side
    out.conflict = conflict
    out.edge_allowance = room.get(want_side) if want_side else None
    out.reasons = tuple(reasons)
    return out

  def _step(self, target, dt, permitted, return_ok, cap):
    """Choose this frame's applied offset inside the intersection of every constraint,
    on the magnitude m = s * applied along the motion side s (u = actual dm/dt):
      continuity   |u - u_prev| <= max_rate_change_mps2 * dt
      rates        -return_rate_mps <= u <= entry_rate_mps
      room         m + u*dt <= cap (min(max_offset, observed edge room); 0 if not observed)
      no crossing  m + u*dt >= 0
      permission   u <= 0 unless permitted; u >= 0 unless return_ok
    The desired rate follows a discrete braking profile, so at uniform dt a target is
    reached with a rate the next frame can bring back to 0. No constraint is skipped and
    no rate is forced to 0. Returns (applied, conflict); conflict names the binding
    lower/upper constraints when the intersection is empty."""
    cfg = self.cfg
    s = self.motion_sign
    if s == 0.0:
      if target == 0.0:
        return 0.0, ()
      s = math.copysign(1.0, target)
    m, u_prev = s * self.applied, s * self.rate
    m_t = max(s * target, 0.0)
    m_cap = cap[RIGHT if s > 0.0 else LEFT]
    d = cfg.max_rate_change_mps2 * dt
    lower = {'continuity': u_prev - d, 'return_rate': -cfg.return_rate_mps, 'no_side_crossing': -m / dt}
    upper = {'continuity': u_prev + d, 'entry_rate': cfg.entry_rate_mps, 'edge_room_or_max_offset': (m_cap - m) / dt}
    if not permitted:
      upper['no_growth_without_permission'] = 0.0
    if not return_ok:
      lower['no_return_without_clear_return_side'] = 0.0
    lo_name = max(lower, key=lower.get)
    hi_name = min(upper, key=upper.get)
    lo, hi = lower[lo_name], upper[hi_name]
    if lo > hi:
      return self.applied, (f'conflict_{lo_name}_vs_{hi_name}',)
    e = m_t - m
    land = abs(e) <= d * dt
    if land:
      u_des = e / dt  # final step: |u_des| <= d, so the next frame can return the rate to 0
    else:
      lim = cfg.entry_rate_mps if e > 0.0 else cfg.return_rate_mps
      # largest rate whose discrete stopping distance (steps of d) still fits in |e|;
      # d*dt/8 covers the gap between the discrete sum and its continuous bound
      brake = d * (math.sqrt(0.25 + 2.0 * (abs(e) - d * dt / 8.0) / (d * dt)) - 0.5)
      u_des = math.copysign(min(lim, brake), e)
    u = min(max(u_des, lo), hi)
    nxt = m_t if (land and u == u_des) else m + u * dt
    # float rounding only: the bounds above already keep 0 <= nxt <= m_cap
    nxt = min(max(nxt, 0.0), m_cap)
    return s * nxt, ()


