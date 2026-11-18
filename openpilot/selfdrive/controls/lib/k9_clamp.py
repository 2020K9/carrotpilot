"""K9Clamp: minimum-distance clamp from lane lines / road edges (experimental menu, NOT vehicle-validated).

Ported from aabc88/openpilot ej (be252819 path clamp, 18c0344b settings, 399efba7 laneless steering corrector,
916a79f0 alert, 6a06a6ce lane-change holdoff + slew, 2c50e890 violation dead band) on 대감님 11/18 13:55 "클램프기능추가해".

K9Clamp (int, default 0):
  0 (or anything else / unreadable): off - lateral_planner, controlsd and selfdrived behave exactly as before.
  1: on.
     - lane mode (lateral_planner): the final path (after PathOffset) is pushed inward where it comes closer than
       MIN_LANE 1.35 m to a lane line or MIN_EDGE 1.15 m to a road edge (edge only on a side whose line is not seen).
     - laneless (controlsd): a curvature correction is added to the model desiredCurvature from the violation at the
       1 s look-ahead point (pure pursuit 2 y / L^2), capped at 1.5 m/s^2 lateral accel and 0.02 1/m, slew 0.01 1/m/s.
     - both modes share one ClampGate: 5 -> 10 km/h fade-in, violation dead band on 0.20 / off 0.10 m, off during lane
       change / blinker / ATC and for 2 s after, gain fixed 1.0 (original default).
     - "Clamp steering correction active" + ding (AudibleAlert.prompt, as ldw) while the laneless correction is large.
Fixes against the original (노비 11/18 13:53):
  1. switch off / lane change / blinker / ATC start / gain 0 / model input missing -> the correction is 0 in that same
     frame (the original slew limit kept it for ~0.3 s).
  2. settings unreadable -> off (the original defaulted to on).
  3. dead band, 2 s holdoff and lane-change block are the same ClampGate in both modes (the original path clamp had none
     of the dead band / holdoff and blocked only on desire / ATC).
Found in the closed-loop mock drives (sim_clamp, 11/18):
  4. SOFT_RELEASE (both modes): while on, push by (violation - 10 cm) instead of the whole violation, so the push is 0
     where the dead band releases (the original dropped ~2 y/L^2 at 10 cm and pulsed on / off every ~2.5 s while the model hugs a line).
  5. laneless: JERK_MAX 0.25 m/s^3 on top of the 0.01 1/m/s slew (that slew alone is 7.7 m/s^3 at 100 km/h).
Found in the K9 log replay (11/18, 12 segments): the original lane-mode clamp moved the far plan by up to 51 m on curves
(plan beyond the last line point -> flat extrapolation, far plan vs far lines). 6. lane mode clamps only the first
PATH_N 17 plan points (t <= 2.5 s); 7. no bound outside the x range of a line / edge.
"""
import math

import numpy as np

K9_CLAMP_PARAM = "K9Clamp"
# 대감님 15:16 "왜 다안넣음?": all 5 original ej settings (ClampEnable / ClampMinDistLane / ClampMinDistEdge / ClampSteerGain /
# ClampSteerAlert) as K9Clamp / K9ClampLaneCm / K9ClampEdgeCm / K9ClampGain / K9ClampAlert, 1 cm steps, original defaults
# (135 / 115 cm, gain 10 = 1.0x, alert on). K9Clamp itself stays default 0 and unreadable = off (fix 2).
K9_CLAMP_LANE_PARAM = "K9ClampLaneCm"
K9_CLAMP_EDGE_PARAM = "K9ClampEdgeCm"
K9_CLAMP_GAIN_PARAM = "K9ClampGain"
K9_CLAMP_ALERT_PARAM = "K9ClampAlert"
DIST_CM_MIN, DIST_CM_MAX = 0, 300
GAIN_RAW_MIN, GAIN_RAW_MAX, GAIN_RAW_DEFAULT = 0, 20, 10

MIN_LANE = 1.35            # m, from a lane line
MIN_EDGE = 1.15            # m, from a road edge
GAIN = 1.0                 # original ClampSteerGain default (10 = 1.0x)
PROB_ON, PROB_OFF = 0.5, 0.3
EDGE_STD_MAX = 0.5
V_FADE_KPH = (5.0, 10.0)
DEADBAND_ON, DEADBAND_OFF = 0.20, 0.10
HOLDOFF_SEC = 2.0
LOOKAHEAD_TIME, LOOKAHEAD_MIN, LOOKAHEAD_MAX = 1.0, 5.0, 30.0
MAX_LAT_ACCEL = 1.5
MAX_CURVATURE = 0.02
SLEW_RATE = 0.01           # 1/m/s
SOFT_RELEASE = 1.0         # 1: while on, push only by (violation - DEADBAND_OFF) -> 0 at the release point (K9 change)
PATH_N = 17                # lane-mode clamp only on the first 17 plan points (T_IDXS: t <= 2.5 s)
JERK_MAX = 0.25            # m/s^3: also cap the correction rate at this lateral jerk (K9 change, see update)
ALERT_ON, ALERT_OFF = 0.001, 0.0005   # 1/m
ALERT_COOLDOWN_SEC = 5.0
MIN_SPEED = 1.0


def k9_clamp_on(params) -> bool:
  """K9Clamp: 1 = on; anything else or unreadable = off (fix 2)."""
  try:
    return int(params.get_int(K9_CLAMP_PARAM)) == 1
  except Exception:
    return False


class ClampCfg:
  __slots__ = ("enabled", "min_lane", "min_edge", "gain", "alert")

  def __init__(self, enabled, min_lane=None, min_edge=None, gain=None, alert=True):
    self.enabled = bool(enabled)
    self.min_lane = MIN_LANE if min_lane is None else min_lane
    self.min_edge = MIN_EDGE if min_edge is None else min_edge
    self.gain = GAIN if gain is None else gain
    self.alert = bool(alert)


def as_cfg(c):
  """ClampCfg, or a bare bool (= on / off with the original defaults)."""
  return c if isinstance(c, ClampCfg) else ClampCfg(bool(c))


def _read_int(params, key, default, lo, hi):
  """param int, clipped; missing / unreadable -> default (the original fell back to its defaults too)."""
  try:
    v = params.get(key, return_default=True)
  except TypeError:
    try:
      v = params.get_int(key)
    except Exception:
      return default
  except Exception:
    return default
  try:
    if v is None or v == b"" or v == "":
      return default
    if isinstance(v, bytes):
      v = v.decode()
    return int(min(max(int(round(float(v))), lo), hi))
  except Exception:
    return default


def read_cfg(params) -> ClampCfg:
  """All 5 settings, read live. K9Clamp unreadable / not 1 -> off (fix 2); the other 4 fall back to the original defaults."""
  if not k9_clamp_on(params):
    return ClampCfg(False)
  lane = _read_int(params, K9_CLAMP_LANE_PARAM, int(round(MIN_LANE * 100)), DIST_CM_MIN, DIST_CM_MAX)
  edge = _read_int(params, K9_CLAMP_EDGE_PARAM, int(round(MIN_EDGE * 100)), DIST_CM_MIN, DIST_CM_MAX)
  gain = _read_int(params, K9_CLAMP_GAIN_PARAM, GAIN_RAW_DEFAULT, GAIN_RAW_MIN, GAIN_RAW_MAX)
  alert = _read_int(params, K9_CLAMP_ALERT_PARAM, 1, 0, 1)
  return ClampCfg(True, lane / 100.0, edge / 100.0, gain / 10.0, alert == 1)


def atc_active_from_carrot_man(cm) -> bool:
  """Same rule as carrot_functions._update_carrot_man (activeCarrot > 1 and 0 < xDistToTurn < 100)."""
  try:
    return int(cm.activeCarrot) > 1 and 0 < int(cm.xDistToTurn) < 100
  except Exception:
    return False


def lane_change_block(md, left_blinker, right_blinker, atc_active) -> bool:
  """Lane change / blinker / ATC: the clamp must not push (crossing the line is intended)."""
  try:
    from openpilot.cereal import log
    if md.meta.desire != log.Desire.none or md.meta.laneChangeState != log.LaneChangeState.off:
      return True
  except Exception:
    return True
  return bool(left_blinker or right_blinker or atc_active)


def speed_strength(v_ego) -> float:
  return float(np.interp(v_ego * 3.6, V_FADE_KPH, [0.0, 1.0]))


class ClampGate:
  """Shared gate (fix 3): enable, lane-change block + 2 s holdoff, dead band. Own clock = calls * dt."""

  def __init__(self, dt):
    self.dt = dt
    self.t = 0.0
    self.holdoff_until = -1.0
    self.db_active = False
    self.left_active = False
    self.right_active = False

  def reset_sides(self):
    self.db_active = False
    self.left_active = False
    self.right_active = False

  def allow(self, enabled, blocked) -> bool:
    """False -> output must be 0 this frame."""
    self.t += self.dt
    if not enabled:
      self.reset_sides()
      return False
    if blocked:
      self.reset_sides()
      self.holdoff_until = self.t + HOLDOFF_SEC
      return False
    if self.t < self.holdoff_until:
      self.db_active = False
      return False
    return True

  def update_sides(self, l_prob, r_prob):
    if l_prob > PROB_ON:
      self.left_active = True
    elif l_prob < PROB_OFF:
      self.left_active = False
    if r_prob > PROB_ON:
      self.right_active = True
    elif r_prob < PROB_OFF:
      self.right_active = False

  def deadband(self, violation) -> bool:
    if self.db_active:
      if violation < DEADBAND_OFF:
        self.db_active = False
    elif violation > DEADBAND_ON:
      self.db_active = True
    return self.db_active


def _interp_in(x, g):
  """y of line g at x; inf / nan outside the x range the model gave for that line (no flat extrapolation: the plan runs
  far past the lines on curves and the original clamp pushed it by metres there; K9 change, 11/18 log replay)."""
  y = np.interp(x, g[0], g[1])
  inside = (np.asarray(x) >= g[0][0]) & (np.asarray(x) <= g[0][-1])
  return np.where(inside, y, np.nan)


def _bounds(gate, x, line_l, line_r, edge_l, edge_r, min_lane=None, min_edge=None):
  """lo / hi limits for y at x (arrays or floats). line_* / edge_* = (xs, ys) or None (edge ys already std-shifted)."""
  min_lane = MIN_LANE if min_lane is None else min_lane
  min_edge = MIN_EDGE if min_edge is None else min_edge
  lo = -np.inf
  hi = np.inf
  if gate.left_active and line_l is not None:
    lo = _interp_in(x, line_l) + min_lane
  elif edge_l is not None:
    lo = _interp_in(x, edge_l) + min_edge
  if gate.right_active and line_r is not None:
    hi = _interp_in(x, line_r) - min_lane
  elif edge_r is not None:
    hi = _interp_in(x, edge_r) - min_edge
  lo = np.where(np.isfinite(lo), lo, -np.inf)
  hi = np.where(np.isfinite(hi), hi, np.inf)
  return lo, hi


def _target(y, lo, hi):
  t = np.clip(y, lo, hi)
  conflict = lo > hi
  return np.where(conflict, (lo + hi) / 2.0, t)


def _model_geometry(md):
  """(line_l, line_r, edge_l, edge_r, probs) from modelV2, or None when nothing usable."""
  ll = md.laneLines
  edges = md.roadEdges
  lines_ok = (len(ll) >= 4 and len(ll[1].y) >= 2 and len(ll[2].y) >= 2 and len(md.laneLineProbs) >= 3
              and len(ll[1].x) == len(ll[1].y) and len(ll[2].x) == len(ll[2].y))
  edges_ok = (len(edges) >= 2 and len(edges[0].y) >= 2 and len(edges[1].y) >= 2 and len(md.roadEdgeStds) >= 2
              and len(edges[0].x) == len(edges[0].y) and len(edges[1].x) == len(edges[1].y))
  if not lines_ok and not edges_ok:
    return None
  line_l = line_r = edge_l = edge_r = None
  probs = (0.0, 0.0)
  if lines_ok:
    line_l = (np.asarray(ll[1].x, dtype=float), np.asarray(ll[1].y, dtype=float))
    line_r = (np.asarray(ll[2].x, dtype=float), np.asarray(ll[2].y, dtype=float))
    probs = (float(md.laneLineProbs[1]), float(md.laneLineProbs[2]))
  if edges_ok:
    le_std, re_std = float(md.roadEdgeStds[0]), float(md.roadEdgeStds[1])
    if le_std < EDGE_STD_MAX:
      edge_l = (np.asarray(edges[0].x, dtype=float), np.asarray(edges[0].y, dtype=float) + le_std * 0.4)
    if re_std < EDGE_STD_MAX:
      edge_r = (np.asarray(edges[1].x, dtype=float), np.asarray(edges[1].y, dtype=float) - re_std * 0.4)
  return line_l, line_r, edge_l, edge_r, probs, lines_ok


class PathClamp:
  """Lane mode: clamp the final path. Runs in lateral_planner (DT_MDL)."""

  def __init__(self, dt):
    self.gate = ClampGate(dt)
    self.max_correction = 0.0
    self.sides = ""

  def apply(self, path_xyz, md, v_ego, enabled, blocked, lane_probs=None):
    """Returns (path_xyz, changed). path_xyz is modified in place only when changed."""
    self.max_correction = 0.0
    self.sides = ""
    cfg = as_cfg(enabled)
    if not self.gate.allow(cfg.enabled and cfg.gain > 0.0, blocked):
      return path_xyz, False
    strength = speed_strength(v_ego)
    try:
      geo = _model_geometry(md)
    except Exception:
      geo = None
    if geo is None or strength <= 0.0:
      self.gate.db_active = False
      return path_xyz, False
    line_l, line_r, edge_l, edge_r, probs, lines_ok = geo
    if lane_probs is not None:
      probs = lane_probs
    if lines_ok:
      self.gate.update_sides(*probs)
    else:
      self.gate.left_active = self.gate.right_active = False
    # only the near plan (first PATH_N points, t <= 2.5 s): the far plan (beyond ~75 m) often disagrees
    # with the far lines by metres on curves (11/18 log replay: original clamp moved it up to 51 m there).
    x = path_xyz[:PATH_N, 0]
    y = path_xyz[:PATH_N, 1]
    if not (np.all(np.isfinite(x)) and np.all(np.isfinite(y))):
      self.gate.db_active = False
      return path_xyz, False
    lo, hi = _bounds(self.gate, x, line_l, line_r, edge_l, edge_r, cfg.min_lane, cfg.min_edge)
    lo = np.broadcast_to(lo, y.shape)
    hi = np.broadcast_to(hi, y.shape)
    if not np.isfinite(lo).any() and not np.isfinite(hi).any():
      self.gate.db_active = False
      return path_xyz, False
    corr = _target(y, lo, hi) - y
    violation = float(np.max(np.abs(corr)))
    if not self.gate.deadband(violation):
      return path_xyz, False
    if SOFT_RELEASE:
      corr = np.sign(corr) * np.maximum(np.abs(corr) - DEADBAND_OFF, 0.0)   # same soft release as the corrector (4.)
    corr = corr * strength * cfg.gain
    self.max_correction = float(np.max(np.abs(corr)))
    if np.any(corr > 0.005):
      self.sides += "L"
    if np.any(corr < -0.005):
      self.sides += "R"
    path_xyz[:PATH_N, 1] = y + corr
    return path_xyz, True


class SteerCorrector:
  """Laneless: curvature added to the model desiredCurvature. Runs in controlsd (DT_CTRL)."""

  def __init__(self, dt):
    self.dt = dt
    self.gate = ClampGate(dt)
    self.kappa = 0.0
    self.alert_active = False
    self._alert_cooldown_until = -1.0

  def reset(self):
    """Not on the laneless branch: output 0 now, start from 0 next time."""
    self.kappa = 0.0
    self.gate.db_active = False
    self.alert_active = False

  def update(self, md, v_ego, enabled, blocked):
    cfg = as_cfg(enabled)
    target, hard_zero = self._target(md, v_ego, cfg, blocked)
    if hard_zero:
      self.kappa = 0.0      # fix 1: no slew tail
    else:
      # original: 0.01 1/m/s only (= 7.7 m/s^3 at 100 km/h: on/off chatter at the dead band and overshoot with actuator
      # delay in the mock drives); K9 also caps it at JERK_MAX lateral jerk. Below ~18 km/h the original 0.01 binds.
      d = min(SLEW_RATE, JERK_MAX / max(v_ego, MIN_SPEED) ** 2) * self.dt
      self.kappa += float(np.clip(target - self.kappa, -d, d))
    self._update_alert(cfg.enabled and cfg.alert)
    return self.kappa

  def _update_alert(self, enabled):
    t = self.gate.t
    if not enabled:
      self.alert_active = False
      return
    k = abs(self.kappa)
    if self.alert_active:
      if k < ALERT_OFF:
        self.alert_active = False
        self._alert_cooldown_until = t + ALERT_COOLDOWN_SEC
    elif k >= ALERT_ON and t >= self._alert_cooldown_until:
      self.alert_active = True

  def _target(self, md, v_ego, cfg, blocked):
    """(target kappa, hard_zero)."""
    if not self.gate.allow(cfg.enabled, blocked) or cfg.gain <= 0.0:
      return 0.0, True
    strength = speed_strength(v_ego)
    try:
      geo = _model_geometry(md)
      pos_ok = len(md.position.x) >= 2 and len(md.position.y) == len(md.position.x)
    except Exception:
      geo, pos_ok = None, False
    if geo is None or not pos_ok:
      self.gate.db_active = False
      return 0.0, True
    if strength <= 0.0:
      self.gate.db_active = False
      return 0.0, False
    line_l, line_r, edge_l, edge_r, probs, lines_ok = geo
    if lines_ok:
      self.gate.update_sides(*probs)
    else:
      self.gate.left_active = self.gate.right_active = False
    la = float(np.clip(v_ego * LOOKAHEAD_TIME, LOOKAHEAD_MIN, LOOKAHEAD_MAX))
    path_y = float(np.interp(la, np.asarray(md.position.x, dtype=float), np.asarray(md.position.y, dtype=float)))
    if not math.isfinite(path_y):
      self.gate.db_active = False
      return 0.0, True
    lo, hi = _bounds(self.gate, la, line_l, line_r, edge_l, edge_r, cfg.min_lane, cfg.min_edge)
    lo, hi = float(lo), float(hi)
    if lo == -np.inf and hi == np.inf:
      self.gate.db_active = False
      return 0.0, False
    tgt = float(_target(path_y, lo, hi))
    if not self.gate.deadband(abs(tgt - path_y)):
        return 0.0, False
    y_corr = tgt - path_y
    if SOFT_RELEASE:
      # original pushes by the whole violation and drops to 0 at 10 cm: on / off pulses every ~2.5 s at ~1 m/s^2 in the
      # mock drives while the model keeps hugging the line. Pushing by (violation - 10 cm) is continuous at the release.
      y_corr = float(np.sign(y_corr)) * max(abs(y_corr) - DEADBAND_OFF, 0.0)
    kappa = 2.0 * y_corr * strength / (la ** 2) * cfg.gain
    max_k = min(MAX_LAT_ACCEL / max(v_ego, MIN_SPEED) ** 2, MAX_CURVATURE)
    return float(np.clip(kappa, -max_k, max_k)), False
