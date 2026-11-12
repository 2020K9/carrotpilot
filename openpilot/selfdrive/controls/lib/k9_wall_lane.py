"""K9WallLane (실험중 맨 위, 0 끔 기본 / 1 켬): 차선이 흐리고 방호벽·가드레일(model roadEdge)이 안정적이면 벽을 그쪽 차선으로 삼아 차선 모드로 따라간다.

대감님 2026-11-12 08:42 / 08:43: "벽을 차선으로 인식하게 만들어서 가야 할 거 아냐", "실험중에 껏다 켤다 할 수 있게 하고 실험중 맨 위에 넣어".
2ed007c35 (k9_wall_hold, K9LaneAssist 1: model curvature + up to 0.6 x wall pursuit) is replaced by this; there is one switch only.

Why (00000227--1a9943ad32--16, 05:13:08.6-21.6): lane line probs 0.29 / 0.31 (mean), the model kept an imaginary 3.34 m lane centred on the
car (path - lane centre -0.05 m) while the wall drifted 1.44 m further out than with clear lines (wall to left line 0.92 -> 1.44-1.53 m).
After the driver pulled toward the wall (12.95-14.05 s) the model path at 20 m went to +0.84 m (away from the wall) and the
curvature to +5.3e-3 1/m: the model re-centred on its own lane, not on the wall.

How (lateral_planner, model rate, only while K9WallLane is 1; 0 -> this module is not called, lateralPlan unchanged):
  - wall side: road edge std <= STD_MAX, D_LO..D_SEEN m from the car centre, BOTH lane lines weak (1 s filtered prob below 0.50 to
    start; a line counts as clear again above 0.70 until below 0.55), opposite edge >= D_OPP, stable for WALL_T s (>= 85 % good
    frames, 5-frame median range <= WALL_RANGE), at least V_ON. The wall line is low-passed (TAU_EDGE) after carrying it with
    the car's own motion (EGO_PRED);
  - virtual lane (11/12 11:00 rework, real lateral MPC replay of 05:13): the wall counts as a straight line, a least-squares line
    through the filtered wall over FIT_X0..FIT_X1 m. The far wall shape is NOT copied any more (at 05:13 the 20 m wall point
    swung 1.1-4.3 m and the old wall-shaped lane doubled the steering swings). The lane path = the normal path shifted sideways
    by an offset c so that, C_LOOK s ahead on the normal path, the gap to the wall line is D_TGT (2.41 m wall to car centre =
    median 05:13:00-08 with clear lines). c = C_TAU low-pass of that, at most C_RATE_IN m/s toward the wall and C_RATE_OUT m/s
    away from it; the shift grows along the path over MERGE_T s of travel (no step at the car);
  - the published path = normal path + g (virtual lane path - normal path), g smoothstep 0 -> 1 in RAMP_T s, 1 -> 0 in RAMP_OUT s
    when the wall is lost, a line becomes clear, lane change / blinker / desire / below V_ON / side change; lateralPlan
    useLaneLines is True while g > 0 (so it works with UseLaneLineSpeed 0 and K9LaneAssist 0); steering pressed toward the wall
    keeps it, steering away turns it off;
  - turns: model lateral accel >= A_TURN (0.25 m/s^2) ends it and ramps out in RAMP_OUT_TURN s. Replay 05:13 14.4-16.6 s
    (model curve away from the wall, +5..+6e-3 1/m): the lane-mode MPC steered less than the model curvature and the car went
    0.5-0.9 m toward the wall (gap 1.5-1.8 m) whatever the offset settings; the model curvature kept 2.35 m there;
  - safety: no point of the lane path closer than D_MIN (2.30) to the wall line (car side 1.34 m).
"""
import numpy as np
from openpilot.common.realtime import DT_MDL

PARAM = "K9WallLane"
HALF_W = 1.915 / 2.0
D_TGT = 2.41
D_MIN = 2.30
D_OPP = 3.8
D_LO, D_SEEN = 1.6, 3.3
STD_MAX = 1.2
P_LINE_ON, P_LINE_OFF, TAU_P = 0.55, 0.70, 1.0
WALL_T, WALL_FRAC, WALL_RANGE = 1.0, 0.85, 0.8
LOST_T = 0.5
OUTLIER = 0.8
TAU_EDGE = 1.0
RAMP_T, RAMP_OUT, RAMP_OUT_TURN = 1.0, 1.0, 0.3
A_TURN = 0.25
V_ON = 30.0 / 3.6     # 05:13: 43-45 km/h; below 30 km/h (city curbs, 000001bb--21 / 0000020e--14 at 24-25 km/h) left alone
P_WEAK_IN = 0.5      # start only while both 1 s filtered line probs are below this
EGO_PRED = True
N_PTS = 33
FIT_X0, FIT_X1 = 0.0, 15.0   # near-field wall fit range (m)
MERGE_T = 1.5                # path shift reaches the full offset after this many seconds of travel
C_TAU = 0.3                  # offset low-pass (s)
C_LOOK = 1.0                 # gap to the wall line taken on the normal path this many seconds ahead
C_RATE_IN, C_RATE_OUT = 0.15, 0.50   # offset rate limit toward / away from the wall (m/s)


def read_mode(params):
  try:
    return 1 if params.get_int(PARAM) == 1 else 0
  except Exception:
    return 0


class Side:
  def __init__(self):
    self.y = None       # filtered edge y on the lane x grid
    self.hist = []
    self.med = []
    self.p_f = None
    self.line_on = False
    self.lost = 0.0
    self.ok = False


def _smooth(g):
  return g * g * (3.0 - 2.0 * g)


class WallLane:
  def __init__(self):
    self.sides = [Side(), Side()]
    self.opp_dist = [9.9, 9.9]
    self.u = 0.0           # ramp state 0..1 (g = smoothstep(u))
    self.delta = None      # last (virtual - normal) path y, for the ramp out
    self.side = -1
    self.dist = 0.0
    self.active = False
    self.cand = False
    self.c = None          # sideways offset of the published path (m, model frame +y = right)
    self.c_f = 0.0
    self.wall_fit = (0.0, 0.0)
    self.fast_out = False

  def _frame(self, md, x_grid, v_ego=0.0, curv=0.0):
    dt = DT_MDL
    sx = max(float(v_ego), 0.0) * dt
    edges = md.roadEdges
    ok_in = len(edges) >= 2 and len(edges[0].x) > 1 and len(md.roadEdgeStds) >= 2 and len(md.laneLineProbs) >= 4
    probs = list(md.laneLineProbs) if ok_in else [0.0] * 4
    for i, sd in enumerate(self.sides):
      p = float(probs[1 if i == 0 else 2])
      sd.p_f = p if sd.p_f is None else sd.p_f + dt / (TAU_P + dt) * (p - sd.p_f)
      sd.line_on = sd.p_f > (P_LINE_ON if sd.line_on else P_LINE_OFF)
      good, y0 = False, 0.0
      if ok_in:
        ys = np.interp(x_grid, np.asarray(edges[i].x, dtype=float), np.asarray(edges[i].y, dtype=float))
        y0 = float(ys[0])
        dist = -y0 if i == 0 else y0
        self.opp_dist[1 - i] = dist if np.isfinite(dist) else 9.9
        good = bool(np.all(np.isfinite(ys)) and float(md.roadEdgeStds[i]) <= STD_MAX and D_LO <= dist <= D_SEEN)
        if good and sd.y is not None and abs(y0 - sd.y[0]) > OUTLIER:
          good = False
      if sd.y is not None and EGO_PRED:
        # carry the filtered wall with the car's own motion (forward sx, heading change curv * sx), so the low-pass only
        # filters the model's frame-to-frame edge noise, not the car's movement. Sign checked on logs (lane line y(t) vs
        # y(t-1) moved this way: median error 0.034 m vs 0.041 m without the heading term and 0.049 m with the other sign,
        # 00000227--16; 0.0210 / 0.0215 / 0.0265 m on 000001bb--16)
        sd.y = np.interp(x_grid + sx, x_grid, sd.y) - float(curv) * sx * x_grid
      if good:
        sd.y = ys.copy() if sd.y is None else sd.y + dt / (TAU_EDGE + dt) * (ys - sd.y)
        sd.med = (sd.med + [y0])[-5:]
        sd.lost = 0.0
      else:
        sd.lost += dt
        if sd.lost > LOST_T:
          sd.y, sd.med = None, []
      sd.hist = (sd.hist + [(good, float(np.median(sd.med)) if sd.med else 0.0)])[-int(round(WALL_T / dt)):]
      n = len(sd.hist)
      goods = [m for g, m in sd.hist if g]
      stable = n >= int(round(WALL_T / dt)) and goods and len(goods) >= WALL_FRAC * n and max(goods) - min(goods) <= WALL_RANGE
      if sd.ok:
        sd.ok = sd.y is not None and not sd.line_on
      else:
        sd.ok = bool(stable) and sd.y is not None and not sd.line_on

  def update(self, md, CS, v_ego, LP, path_t, path_xyz, base_active, model_curv, laneless_only):
    """path_xyz / base_active: the normal lateral_planner result this frame. Returns (path_xyz, active)."""
    x_grid = np.asarray(LP.ll_x, dtype=float)
    if len(x_grid) != N_PTS or not np.all(np.isfinite(x_grid)):
      x_grid = np.asarray(path_xyz[:, 0], dtype=float)
    self._frame(md, x_grid, v_ego, float(getattr(LP, 'curvature', 0.0) or 0.0))
    lines_weak = not (self.sides[0].line_on or self.sides[1].line_on)
    desire = getattr(md.meta, 'desire', 0)
    off = (int(getattr(md.meta, 'laneChangeState', 0)) != 0 or (desire not in (0, 'none') and str(desire) not in ('none', '0'))
           or bool(CS.leftBlinker) or bool(CS.rightBlinker) or laneless_only
           or v_ego < V_ON)
    turn = abs(model_curv) * max(v_ego, 1.0) ** 2 >= A_TURN
    if turn:
      off = True
      if self.u > 0.0:
        self.fast_out = True
    if bool(CS.steeringPressed):
      # driver steering toward the wall (the 05:13 case: +200..+300 torque to the left wall) keeps the wall lane;
      # steering away from it (or no side yet) turns it off (ramp out). steeringTorque > 0 = left.
      tq = float(getattr(CS, 'steeringTorque', 0.0))
      toward = (self.side == 0 and tq > 0.0) or (self.side == 1 and tq < 0.0)
      if not (toward and self.u > 0.0):
        off = True
    cand = []
    if lines_weak and self.u <= 0.0 and max(sd.p_f or 0.0 for sd in self.sides) >= P_WEAK_IN:
      lines_weak = False
    if lines_weak and not off:
      for i, sd in enumerate(self.sides):
        if sd.ok and sd.y is not None and self.opp_dist[i] >= D_OPP:
          cand.append(((-float(sd.y[0])) if i == 0 else float(sd.y[0]), i))
    self.cand = bool(cand)
    virt = None
    if cand:
      d, i = min(cand)
      if i != self.side and self.u > 0.0:
        cand = []          # side switch: ramp out first
      else:
        self.side, self.dist = i, d
        virt = self._virtual_path(i, x_grid, LP, CS, v_ego, path_t, path_xyz)
    if virt is not None:
      self.delta = virt - path_xyz[:, 1]
      self.u = min(self.u + DT_MDL / RAMP_T, 1.0)
      self.fast_out = False
    else:
      self.u = max(self.u - DT_MDL / (RAMP_OUT_TURN if self.fast_out else RAMP_OUT), 0.0)
      if self.u <= 0.0:
        self.delta, self.side = None, -1
        self.c = None
        self.fast_out = False
    self.active = virt is not None
    if self.u <= 0.0 or self.delta is None:
      return path_xyz, base_active
    out = path_xyz.copy()
    out[:, 1] = path_xyz[:, 1] + _smooth(self.u) * self.delta
    if virt is not None:
      k, b = self.wall_fit      # fitted near-field wall line (the far wall shape is noise, see _virtual_path)
      wy = b + k * np.asarray(out[:, 0], dtype=float)
      out[:, 1] = np.maximum(out[:, 1], wy + D_MIN) if self.side == 0 else np.minimum(out[:, 1], wy - D_MIN)
    return out, True

  def _virtual_path(self, i, x_grid, LP, CS, v_ego, path_t, path_xyz):
    """Offset-only wall lane (11/12 fix of the 05:13 replay): the wall is reduced to ONE number, its near-field lateral position
    (straight-line fit of the filtered wall over FIT_X0..FIT_X1 m, evaluated at the car), and the published path is the normal
    path shifted sideways by a slow offset c so the car settles D_TGT from the wall. The path SHAPE stays the model's: the far
    wall shape (the 20 m wall point swung 1.1-4.3 m at 05:13) is no longer copied into the steering. c follows a C_TAU low-pass
    of its target and moves at most C_RATE_IN m/s toward the wall and C_RATE_OUT m/s away from it; the shift grows along the
    path over MERGE_T s of travel (no step at the car). Every path point stays D_MIN from the fitted wall line."""
    wall = self.sides[i].y
    sgn = 1.0 if i == 0 else -1.0                        # left wall (i 0, y < 0): the road is toward +y
    xs = np.asarray(x_grid, dtype=float)
    m = (xs >= FIT_X0) & (xs <= FIT_X1)
    if m.sum() >= 2:
      k, b = np.polyfit(xs[m], wall[m], 1)
    else:
      k, b = 0.0, float(wall[0])
    px = np.asarray(path_xyz[:, 0], dtype=float)
    py = np.asarray(path_xyz[:, 1], dtype=float)
    x_l = C_LOOK * max(v_ego, 0.0)                       # gap measured between the normal path and the wall line here
    gap_l = sgn * (float(np.interp(x_l, px, py)) - (b + k * x_l))
    c_tgt = sgn * (D_TGT - gap_l)                        # wanted sideways move (model frame +y = right)
    if self.c is None:
      self.c = self.c_f = 0.0
    self.c_f += DT_MDL / (C_TAU + DT_MDL) * (c_tgt - self.c_f)
    step = self.c_f - self.c
    lim = (C_RATE_IN if sgn * step < 0.0 else C_RATE_OUT) * DT_MDL     # sgn * step < 0: toward the wall
    self.c += float(np.clip(step, -lim, lim))
    ramp = np.clip(px / max(MERGE_T * max(v_ego, 1.0), 1.0), 0.0, 1.0)
    y = py + self.c * ramp * ramp * (3.0 - 2.0 * ramp)
    self.wall_fit = (float(k), float(b))
    wl_y = b + k * px
    return np.maximum(y, wl_y + D_MIN) if i == 0 else np.minimum(y, wl_y - D_MIN)


def k9_wall_lane_step(planner, md, CS, v_ego, path_t, path_xyz, base_active):
  """lateral_planner hook. planner.k9_wall_lane_mode != 1 -> state dropped, inputs returned unchanged."""
  if getattr(planner, 'k9_wall_lane_mode', 0) != 1:
    planner.k9_wl = None
    return path_xyz, base_active
  LP = getattr(planner, 'LP', None)
  if not all(hasattr(md, a) for a in ('roadEdges', 'roadEdgeStds', 'laneLineProbs', 'action', 'meta')) or \
     not all(hasattr(LP, a) for a in ('ll_x', 'll_t', 'lane_width')):
    planner.k9_wl = None       # incomplete inputs (stubs): nothing changed
    return path_xyz, base_active
  wl = getattr(planner, 'k9_wl', None)
  if wl is None:
    wl = planner.k9_wl = WallLane()
  return wl.update(md, CS, v_ego, planner.LP, path_t, path_xyz, base_active, float(md.action.desiredCurvature),
                   bool(getattr(planner, 'laneless_only', False)))
