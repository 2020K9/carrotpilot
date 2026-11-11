"""K9 wall hold: follow a stable wall / guard rail (model roadEdge) at a fixed gap while the lane lines are weak.

Part of K9LaneAssist 1 (no new menu; 0 / anything else = off, the laneless command is returned unchanged, no state kept).
대감님 2026-11-12 05:59 / 06:05 "가드레일을 벽으로 생각하고 붙어서 가야", "벽기준으로 붙어서 가게 해라".

Why (00000227--1a9943ad32--16, 05:13:08.6-21.6, construction zone, 43-45 km/h): lane line probs fell from 0.8-0.97 to
0.1-0.6, the car ran pure laneless (UseLaneLineSpeed 0, K9LaneAssist 0) and the model path swung left / right
(desiredCurvature -0.0027 -> +0.0063 -> -0.0053 1/m in 6 s) while the left wall stayed 2.2-3.1 m from the car centre.
The driver pulled the car toward the wall four times (+24 deg); the model then pushed it away from the wall again.

Gap (measured on that log, car width K9 1.915 m -> half width 0.958 m):
  - 05:13:00-08 (wall present, lines clear, no driver input): wall to car centre median 2.41 m;
  - 1 s after each driver correction ended: minimum 2.17-2.34 m (driver keeps it closer than the model);
  - window 08.6-21.6 as driven (edge visible): mean 2.70 m, std 0.36 m.
  -> target D_TGT 2.40 m wall to car centre = 1.44 m from the car side; never pulled closer than D_MIN 2.0 m
     (1.04 m from the car side); a weak line on the wall side is not crossed (car side >= LINE_MARGIN inside it).

How (controlsd laneless branch, every tick, after the K9 lane holds):
  - wall side: the model road edge with std <= STD_MAX, 1.6..D_SEEN m from the car centre, BOTH lane lines weak (a line counts
    as clear once its 1 s filtered probability is above 0.70 and until it drops below 0.55), opposite edge at least
    D_OPP m away (05:13 window: 4.6 m; narrow unmarked streets with both edges at 2.3-3.4 m are left alone).
    Normal roads with one clear line or a curb further than D_SEEN: nothing changes;
  - wall accepted only after it was seen stable for WALL_T s (>= 85 % good frames, 5-frame median at the car moving
    less than WALL_RANGE m over the window); lost after LOST_T s without a good frame (then ramp out);
  - reference: the filtered wall line (per-point low-pass TAU_EDGE, frame outliers > OUTLIER m rejected) shifted by
    D_TGT toward the road; pure pursuit at T_LOOK s (at least X_LOOK_MIN m): k_wall = 2 y / (x^2 + y^2);
  - output = model + w (k_wall - model), w ramps 0 -> W_MAX over RAMP_T s (RAMP_OUT out), the added part is capped at
    A_MAX lateral accel and rate limited at J_MAX lateral jerk. No push toward the wall when closer than D_MIN or when the
    car side is within LINE_MARGIN of a seen wall-side line; off in lane change / blinker / desire / steering pressed /
    lane mode / turns (model lateral accel A_TURN) / below V_ON.
"""
import numpy as np

DT = 0.01
HALF_W = 1.915 / 2.0
D_TGT = 2.40
D_MIN = 2.00
D_OPP = 3.8        # opposite road edge at least this far: a road with room beside the wall, not a narrow unmarked street
D_LO, D_SEEN = 1.6, 3.3
STD_MAX = 1.2
P_LINE_ON, P_LINE_OFF, TAU_P = 0.55, 0.70, 1.0   # clear line: stays clear above P_LINE_ON, becomes clear above P_LINE_OFF
P_LINE_SEEN = 0.3
LINE_MARGIN = 0.15
WALL_T, WALL_FRAC, WALL_RANGE = 1.0, 0.85, 0.8
LOST_T = 0.5
OUTLIER = 0.8
TAU_EDGE = 0.4
X_PTS = np.array([0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0])
T_LOOK, X_LOOK_MIN = 2.4, 8.0
W_MAX, RAMP_T, RAMP_OUT = 0.6, 1.0, 0.5
A_MAX, J_MAX = 0.8, 0.6
A_TURN = 1.2       # 05:13 window: model lateral accel at most 0.97 m/s^2
V_ON = 20.0 / 3.6
STALE_T = 0.3


class Side:
  def __init__(self):
    self.y = None          # filtered wall line at X_PTS (car frame, + right)
    self.hist = []         # (good, y0) for the last WALL_T s
    self.med = []          # last 5 y0
    self.p_f = None
    self.line_on = False   # this side's lane line counted as clear (hysteresis)
    self.lost = 0.0
    self.ok = False


class WallHold:
  def __init__(self):
    self.key = None
    self.frame_dt = 0.0
    self.sides = [Side(), Side()]
    self.side = -1
    self.w = 0.0
    self.k_add = 0.0
    self.active = False
    self.dist = 0.0
    self.line_y = [None, None]
    self.opp_dist = [9.9, 9.9]

  def _frame(self, md, dt):
    edges = md.roadEdges
    if len(edges) < 2 or len(edges[0].x) == 0 or len(md.roadEdgeStds) < 2:
      for sd in self.sides:
        sd.hist.append((False, 0.0))
      self._trim(dt)
      return
    ll = md.laneLines
    probs = list(md.laneLineProbs) if len(md.laneLineProbs) >= 4 else [0.0] * 4
    for i, sd in enumerate(self.sides):
      x = np.asarray(edges[i].x, dtype=float)
      y = np.asarray(edges[i].y, dtype=float)
      std = float(md.roadEdgeStds[i])
      ys = np.interp(X_PTS, x, y)
      y0 = float(ys[0])
      dist = -y0 if i == 0 else y0
      li = 1 if i == 0 else 2
      p = float(probs[li]) if len(ll) >= 4 else 0.0
      sd.p_f = p if sd.p_f is None else sd.p_f + dt / (TAU_P + dt) * (p - sd.p_f)
      sd.line_on = sd.p_f > (P_LINE_ON if sd.line_on else P_LINE_OFF)
      self.line_y[i] = float(ll[li].y[0]) if len(ll) >= 4 and len(ll[li].y) and p >= P_LINE_SEEN else None
      good = bool(np.all(np.isfinite(ys)) and std <= STD_MAX and D_LO <= dist <= D_SEEN)
      if good and sd.y is not None and abs(y0 - sd.y[0]) > OUTLIER:
        good = False
      if good:
        sd.y = ys.copy() if sd.y is None else sd.y + dt / (TAU_EDGE + dt) * (ys - sd.y)
        sd.med = (sd.med + [y0])[-5:]
        sd.lost = 0.0
      else:
        sd.lost += dt
        if sd.lost > LOST_T:
          sd.y = None
          sd.med = []
      sd.hist.append((good, float(np.median(sd.med)) if sd.med else 0.0))
      self.opp_dist[1 - i] = dist if np.isfinite(dist) else 9.9
    self._trim(dt)
    for sd in self.sides:
      n = len(sd.hist)
      need = int(round(WALL_T / max(dt, 1e-3)))
      goods = [m for g, m in sd.hist if g]
      stable = (n >= need and len(goods) >= WALL_FRAC * n and goods and max(goods) - min(goods) <= WALL_RANGE)
      if sd.ok:
        sd.ok = sd.y is not None and not sd.line_on
      else:
        sd.ok = bool(stable) and sd.y is not None and not sd.line_on

  def _trim(self, dt):
    need = int(round(WALL_T / max(dt, 1e-3)))
    for sd in self.sides:
      sd.hist = sd.hist[-need:]

  def update(self, md, v, laneless, off, model_curv):
    fid = getattr(md, 'frameId', None)
    self.frame_dt += DT
    if fid is None or fid != self.key:
      self.key = fid
      self._frame(md, float(np.clip(self.frame_dt, 0.01, 0.2)) if self.frame_dt > 0.0 else 0.05)
      self.frame_dt = 0.0
    v2 = max(v, 1.0) ** 2
    cand = []
    lines_weak = not (self.sides[0].line_on or self.sides[1].line_on)   # both lane lines weak (a clear line: lane holds)
    for i, sd in enumerate(self.sides):
      if not lines_weak:
        break
      if sd.ok and sd.y is not None:
        d = -float(sd.y[0]) if i == 0 else float(sd.y[0])
        if self.opp_dist[i] >= D_OPP:
          cand.append((d, i))
    ok = (laneless and not off and v >= V_ON and self.frame_dt <= STALE_T and abs(model_curv) * v2 < A_TURN and bool(cand))
    tgt = 0.0
    if ok:
      d, i = min(cand)
      self.side, self.dist = i, d
      sd = self.sides[i]
      sgn = 1.0 if i == 0 else -1.0          # left wall: reference = wall + D_TGT to the right
      ref = sd.y + sgn * D_TGT
      xl = max(v * T_LOOK, X_LOOK_MIN)
      yl = float(np.interp(xl, X_PTS, ref))
      k_wall = 2.0 * yl / (xl * xl + yl * yl)
      tgt = self.w_target() * (k_wall - model_curv)
      toward_wall = tgt * (-sgn) > 0.0
      if toward_wall:
        ly = self.line_y[i]
        near_line = ly is not None and abs(ly) - HALF_W < LINE_MARGIN
        if d <= D_MIN or near_line:
          tgt = 0.0
      self.w = min(self.w + DT / RAMP_T, 1.0)
    else:
      self.w = max(self.w - DT / RAMP_OUT, 0.0)
      tgt = 0.0      # ramping out: the added part only moves back to 0 (jerk limited)
    amax = A_MAX / v2
    tgt = float(np.clip(tgt, -amax, amax))
    dk = J_MAX / v2 * DT
    self.k_add = float(np.clip(tgt, self.k_add - dk, self.k_add + dk))
    if not ok and self.w <= 0.0 and abs(self.k_add) <= dk:
      self.k_add = 0.0
    self.active = ok
    return model_curv + self.k_add

  def w_target(self):
    g = self.w * self.w * (3.0 - 2.0 * self.w)
    return W_MAX * g



def k9_wall_hold_step(controls, CS, CC, model_v2, lane_change_off, dc_model):
  """controlsd laneless branch: returns dc_model with the wall hold applied. K9LaneAssist (read by k9_lane_center_step this
  tick as controls.k9_lc_mode) anything but 1: state dropped, dc_model returned unchanged."""
  if getattr(controls, 'k9_lc_mode', 0) != 1:
    controls.k9_wh = None
    return dc_model
  wh = getattr(controls, 'k9_wh', None)
  if wh is None:
    wh = controls.k9_wh = WallHold()
  laneless = bool(CC.latActive) and not controls.is_vw_meb and not controls.lanefull_mode_enabled
  off = (model_v2.meta.laneChangeState != lane_change_off or bool(CS.leftBlinker) or bool(CS.rightBlinker)
         or bool(CS.steeringPressed))
  desire = getattr(model_v2.meta, 'desire', None)
  if desire is not None and str(desire) not in ('none', '0'):
    off = True
  return float(wh.update(model_v2, float(CS.vEgo), laneless, off, float(dc_model)))
