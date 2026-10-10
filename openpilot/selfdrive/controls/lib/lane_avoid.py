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

# Return-risk policies (hold / decelerate / driver warning ...) are not approved.
# The empty tuple keeps validate() failing until a reviewed policy is added here.
APPROVED_RETURN_RISK_POLICIES: tuple = ()
APPROVED_CENTER_INVALID_POLICIES: tuple = ()


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

  def blockers(self):
    out = []
    if not self.enabled:
      out.append('feature_disabled')
    positive = ('max_offset_m', 'entry_rate_mps', 'return_rate_mps', 'max_rate_change_mps2',
                'candidate_deadband_m', 'max_abs_curvature', 'vehicle_half_width_m',
                'road_edge_margin_m', 'road_edge_std_max_m', 'max_input_gap_s', 'reentry_wait_s')
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


def evaluate_side(readings, side, cfg):
  """readings: {source: SourceReading}. Returns (state, {source: state})."""
  ages = cfg.max_age_s if isinstance(cfg.max_age_s, dict) else {}
  per = {s: evaluate_source(readings.get(s), ages.get(s)) for s in REQUIRED_SOURCES}
  if any(v == OCCUPIED for v in per.values()):
    return OCCUPIED, per
  if not all(v == CLEAR for v in per.values()):
    return UNKNOWN, per
  cov = cfg.coverage if isinstance(cfg.coverage, dict) else {}
  if not region_covered([cov.get((s, side)) for s in REQUIRED_SOURCES], cfg.required_region_x_m,
                        cfg.required_region_lat_m):
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
  measured from the lane-mode base path over the candidate x range.
  Returns (state, allowance). roadEdges is a model estimate, not a guardrail sensor."""
  ages = cfg.max_age_s if isinstance(cfg.max_age_s, dict) else {}
  max_age = ages.get('road_edge')
  if not _finite(cfg.vehicle_half_width_m, cfg.road_edge_margin_m, cfg.road_edge_std_max_m, max_age) or \
     cfg.candidate_x_m is None:
    return UNKNOWN, 0.0
  if not (_finite(edge_std, edge_age_s) and 0.0 <= edge_age_s <= max_age and edge_std <= cfg.road_edge_std_max_m):
    return UNKNOWN, 0.0
  try:
    x = np.asarray(path_x, dtype=float)
    by = np.asarray(base_y, dtype=float)
    ex = np.asarray(edge_x, dtype=float)
    ey = np.asarray(edge_y, dtype=float)
  except (TypeError, ValueError):
    return UNKNOWN, 0.0
  if x.shape != by.shape or ex.shape != ey.shape or ex.ndim != 1 or ex.size < 2 or x.ndim != 1:
    return UNKNOWN, 0.0
  if not (np.all(np.isfinite(x)) and np.all(np.isfinite(by)) and np.all(np.isfinite(ex)) and np.all(np.isfinite(ey))):
    return UNKNOWN, 0.0
  if np.any(np.diff(ex) <= 0.0):
    return UNKNOWN, 0.0
  sel = (x >= cfg.candidate_x_m[0]) & (x <= cfg.candidate_x_m[1])
  # no extrapolation: the edge must be observed over the whole range
  if not np.any(sel) or x[sel][0] < ex[0] or x[sel][-1] > ex[-1]:
    return UNKNOWN, 0.0
  edge_at = np.interp(x[sel], ex, ey)
  dist = SIDE_SIGN[side] * (edge_at - by[sel])
  allowance = float(np.min(dist)) - cfg.vehicle_half_width_m - cfg.road_edge_margin_m
  if allowance <= 0.0:
    return OCCUPIED, 0.0
  return CLEAR, allowance


@dataclass
class AvoidInputs:
  t: float | None                       # s, model frame time (monotonic)
  center_valid: bool                    # lane-mode base path is in use and valid
  path_x: object = None
  model_y: object = None                # laneless path y before lane blending
  lane_y: object = None                 # lane path y on the same samples (None if not used)
  base_y: object = None                 # lane-mode path y before this offset (centre reference)
  d_prob: float | None = None
  model_valid: bool = False
  model_age_s: float | None = None
  lane_change_active: bool = True       # desire / lane change / ATC
  driver_steering: bool = True
  measured_curvature: float | None = None
  side_readings: dict = field(default_factory=dict)   # {side: {source: SourceReading}}
  road_edges: dict = field(default_factory=dict)      # {side: (edge_x, edge_y, std, age_s)}


@dataclass
class AvoidOutput:
  state: str = DISABLED
  permitted: bool = False               # new avoidance start/hold/expand allowed this frame
  target: float = 0.0                   # new-avoidance target (model y, m)
  applied: float = 0.0                  # rate-limited offset actually added (model y, m)
  return_move_permitted: bool = False
  avoid_side: str | None = None
  candidate: float | None = None
  edge_allowance: float | None = None
  reasons: tuple = ()
  side_state: dict = field(default_factory=dict)      # {side: state}
  source_state: dict = field(default_factory=dict)    # {side: {source: state}}
  edge_state: dict = field(default_factory=dict)      # {side: state}
  blockers: tuple = ()


class LaneAvoidController:
  def __init__(self, cfg=None):
    self.cfg = cfg if cfg is not None else LaneAvoidConfig()
    self.blockers = tuple(self.cfg.blockers())
    self.active = not self.blockers
    self.reset()

  def reset(self):
    self.state = STANDBY if self.active else DISABLED
    self.applied = 0.0
    self.rate = 0.0
    self.avoid_side = None
    self.last_t = None
    self.confirm = 0
    self.revoked_t = None

  def update(self, inp):
    if not self.active:
      return AvoidOutput(blockers=self.blockers)
    cfg = self.cfg
    out = AvoidOutput(blockers=self.blockers)
    reasons = []

    # time base: reversal, duplicate or gap revokes permission; no motion this frame
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

    sides, sources = {}, {}
    for side in (LEFT, RIGHT):
      sides[side], sources[side] = evaluate_side(inp.side_readings.get(side, {}), side, cfg)
    out.side_state, out.source_state = sides, sources

    edges = {}
    allowance = {}
    for side in (LEFT, RIGHT):
      e = inp.road_edges.get(side)
      if e is None or inp.base_y is None:
        edges[side], allowance[side] = UNKNOWN, 0.0
      else:
        edges[side], allowance[side] = road_edge_allowance(inp.path_x, inp.base_y, e[0], e[1], e[2], e[3], side, cfg)
    out.edge_state = edges

    if not inp.center_valid:
      # No safe reference to return to; this is handed to the (unapproved) centre-invalid
      # policy. Lane mode not in use also means the offset is not consumed (lateral_planner).
      reasons.append('center_invalid')

    candidate, why = (None, 'no_lane_path')
    if inp.lane_y is not None and inp.model_y is not None:
      candidate, why = avoid_candidate(inp.path_x, inp.model_y, inp.lane_y, inp.d_prob, cfg)
    out.candidate = candidate
    if why:
      reasons.append(why)

    ages = cfg.max_age_s
    model_ok = inp.model_valid and _finite(inp.model_age_s) and 0.0 <= inp.model_age_s <= ages['model_path']
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
    want_side = None
    if candidate is not None:
      want_side = RIGHT if candidate > 0.0 else LEFT
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
    if gate_ok:
      self.confirm += 1
    else:
      self.confirm = 0

    permitted = False
    target = 0.0
    if gate_ok and (self.state == AVOID or self.confirm >= cfg.entry_confirm_frames) and \
       self.state in (STANDBY, AVOID):
      permitted = True
      mag = min(abs(candidate), cfg.max_offset_m, allowance[want_side])
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
      return_ok = sides[ret] == CLEAR and edges[ret] != OCCUPIED and edges[ret] != UNKNOWN \
        and inp.center_valid and dt is not None

    new_applied = self.applied
    if dt is not None:
      new_applied = self._step(target, dt)
    if not permitted and abs(new_applied) > abs(self.applied):
      # rate momentum must never carry the offset further once permission is gone
      new_applied, self.rate = self.applied, 0.0
    shrinking = abs(new_applied) < abs(self.applied)
    if shrinking and not return_ok:
      # Moving toward the original obstacle side without observed-clear space.
      # The approved risk policy would act here; none is approved, so blockers()
      # keeps the controller disabled and this branch is not reachable. The
      # placeholder below only refuses the movement; it is not an approved hold.
      if self.state == AVOID:
        self.revoked_t = self.last_t
      self.state = RETURN_RISK
      permitted, target = False, 0.0
      self.rate = 0.0
      new_applied = self.applied
      reasons.append('return_risk_policy_unapproved')
    elif self.state == RETURN_RISK and return_ok:
      self.state = RETURN
    out.return_move_permitted = shrinking and return_ok
    self.applied = new_applied

    if self.state in (RETURN, RETURN_RISK) and self.applied == 0.0:
      self.state = STANDBY
      self.avoid_side = None
      self.rate = 0.0
      self.confirm = 0

    out.state = self.state
    out.permitted = permitted
    out.target = target
    out.applied = self.applied
    out.avoid_side = self.avoid_side
    out.edge_allowance = allowance.get(want_side) if want_side else None
    out.reasons = tuple(reasons)
    return out

  def _step(self, target, dt):
    """Move applied toward target: entry rate when |applied| grows, return rate when it
    shrinks, rate change bounded by max_rate_change_mps2 (braking profile toward the
    target), never past the target and never across zero."""
    cfg = self.cfg
    err = target - self.applied
    growing = abs(target) > abs(self.applied) and (self.applied == 0.0 or target * self.applied > 0.0)
    limit = cfg.entry_rate_mps if growing else cfg.return_rate_mps
    want = math.copysign(min(limit, math.sqrt(2.0 * cfg.max_rate_change_mps2 * abs(err))), err)
    dv = cfg.max_rate_change_mps2 * dt
    self.rate = float(np.clip(want, self.rate - dv, self.rate + dv))
    if self.rate * err < 0.0:
      # momentum away from the target (e.g. a reduced edge allowance) is cut, not
      # carried: safety takes precedence over the continuity limit here
      self.rate = 0.0
    nxt = self.applied + self.rate * dt
    if (target - nxt) * err <= 0.0:  # reached or passed the target (err == 0 included)
      nxt, self.rate = target, 0.0
    if self.applied != 0.0 and nxt * self.applied < 0.0:  # no crossing to the other side
      nxt, self.rate = 0.0, 0.0
    return float(np.clip(nxt, -cfg.max_offset_m, cfg.max_offset_m))
