"""K9OneLaneHold: laneless one-lane-line gap hold (experimental menu, sim only, NOT vehicle-validated).

Param K9OneLaneHold (int, default 0):
  0 (or anything else): off - controlsd.py behaves exactly as before (this module is not called).
  1: on. Only while controlsd follows the laneless model command (model desiredCurvature):
     1) While both lane lines are reliable, the gaps car edge <-> left line and car edge <-> right line are measured
        (car width K9 1.915 m, low-pass filtered). The centre target gap (lane width / 2 - half car width) and the
        minimum gap (smaller of the two filtered gaps, at least MIN_GAP_FLOOR, at most the target) are remembered.
     2) When only one lane line stays reliable, a bounded curvature is added to the model desiredCurvature so the gap
        to that line returns to the remembered centre target (PD: gap error + filtered time rate of the line position at
        the car, not the line slope - the near slope is biased in curves); closer than the remembered minimum gap the push is
        stronger. Two lines or no line: nothing is added (laneless as before).
     3) Off (with a ramp back to 0, memory dropped) when the visible line moves by itself: its position at the car is
        predicted every model frame from the previous frame (line fit at the distance travelled minus the model path there)
        and the leaky sum (TAU_C) of the unexplained motion exceeds C_OFF (bias per line learned while both lines are seen);
        the push already fades from C_FADE0 on, and nothing is pushed for the first T_PROBE s of a one-line episode.
        Also off when the remaining shift exceeds S_OFF or the visible line keeps receding for RECEDE_T s while the hold
        pushes fully (the line moved: widening lane, merge, split).
        Off (with a ramp back to 0) in lane change / blinker / steering pressed / turns / low speed, lane width jump or
        lane width changing ahead (merge, split), the visible line jumping, diverging from the model path or jittering.
     4) The memory is dropped after MEM_T s or MEM_D m without both lines. The correction is capped (lateral accel 0.25 m/s^2, 0.6 inside
        the minimum gap) and rate limited (lateral jerk 0.2 m/s^3), and ramped in / out over RAMP_T s.
"""
import numpy as np

K9_ONE_LANE_HOLD_PARAM = "K9OneLaneHold"
DT = 0.01                    # controlsd tick
HALF_W = 1.915 / 2.0         # Kia K9 (RJ) overall width 1,915 mm
P_ON, P_OFF, STD_MAX = 0.6, 0.45, 0.2
W_MIN, W_MAX = 2.6, 4.3      # plausible lane width [m]
W_JUMP, W_AHEAD = 0.35, 0.5  # lane width step vs filtered / width change 0 -> 30 m (merge, split)
DIVERGE = 0.5                # visible line vs model path lateral change over 30 m [m]
LINE_JUMP = 0.3              # visible line step in one model frame after ego motion [m]
JIT_MAX = 0.06               # filtered |line step residual| per frame [m]
MEM_T, MEM_D = 15.0, 300.0
V_MIN, K_TURN = 8.0, 0.006   # m/s, 1/m (|curvature| above = turning)
MIN_GAP_FLOOR = 0.3
OMEGA, ZETA, KI = 0.35, 0.9, 0.15   # closed-loop sweep 13:55 (0.6/0.45 cost more jerk, 0.45 worse p95 on R20)
S_MAX, I_MAX = 0.6, 0.4
S_OFF = 0.5                  # remaining shift above this = memory implausible -> dropped
RECEDE_RATE, RECEDE_T = 0.08, 1.0   # line keeps receding while the hold pushes fully for RECEDE_T s -> line moved, not the car
# r3 line-moved detector: per model frame the visible line position at the car is predicted from the previous frame
# (previous line fit at the distance travelled minus the model path there: the car moves along its heading / the laneless
# path; the line fit is in the car frame, so heading changes made by the hold itself are already in the line slope).
# Innovation bias per line is learned while both lines are reliable. Leaky sum of the unexplained motion over TAU_C;
# above C_OFF the line moved by itself (merge / split / branch line) -> memory dropped, hold ramps out.
C_OFF, TAU_C, TAU_B, INN_JUMP = 0.10, 1.0, 5.0, 0.3   # r3 sweep 15:35 (0.15/0.12 still followed MERGE or cost jerk)
T_PROBE = 0.6                # one-line episode age before the hold may push (the detector watches the line first) [s]
C_FADE0 = 0.03               # the push fades linearly from |unexplained motion| C_FADE0 to 0 at C_OFF (moving line = less trust)
A_MAX, A_MAX_CLOSE, J_MAX = 0.25, 0.6, 0.2
RAMP_T, BLOCK_T = 1.0, 1.0
BOTH_T, STALE_T = 1.0, 0.3      # both lines steady before remembering / model frame older than this = no measurement
TAU_GAP, TAU_W, TAU_RATE = 0.5, 1.0, 0.3
X_NEAR, X_FAR = 20.0, 30.0


def k9_one_lane_hold_mode(params) -> int:
  try:
    mode = int(params.get_int(K9_ONE_LANE_HOLD_PARAM))
  except Exception:
    return 0
  return 1 if mode == 1 else 0


def _near_coef(x, y):
  """quadratic least-squares fit over 0 <= x <= X_NEAR (highest power first); constant when too few points"""
  m = x <= X_NEAR
  if m.sum() < 4:
    return np.array([0.0, 0.0, float(y[0])])
  return np.polyfit(x[m], y[m], 2)


def _near_fit(x, y):
  """intercept and slope at x=0 of a quadratic least-squares fit over 0 <= x <= X_NEAR"""
  c = _near_coef(x, y)
  return float(c[2]), float(c[1])


def _lpf(prev, val, dt, tau):
  if prev is None:
    return val
  a = dt / (tau + dt)
  return prev + a * (val - prev)


class OneLaneHold:
  def __init__(self):
    self.reset()

  def reset(self):
    self.key = None
    self.frame_dt = 0.0
    self.rel = [False, False]       # left, right reliable (hysteresis)
    self.y0 = [None, None]
    self.jit = [0.0, 0.0]
    self.ydot = [0.0, 0.0]          # filtered d(line y at the car)/dt [m/s] = rate of the remaining shift
    self.g_f = [None, None]
    self.w_f = None
    self.both_t = 0.0
    self.mem = None                 # dict(target, min_gap, t, d)
    self.block_t = 0.0
    self.meas = None                # (side, gap, ds/dt, line30 - path30 divergence ok)
    self.integ = 0.0
    self.w = 0.0
    self.k_raw = 0.0
    self.k_out = 0.0
    self.active = False
    self.recede_t = 0.0
    self.prev = None                # previous frame (line fits, path x, path y) for the line-moved detector
    self.ib = [0.0, 0.0]            # learned innovation bias [m/s] per line
    self.cu = [0.0, 0.0]            # leaky sum of unexplained line motion [m]
    self.moved = False
    self.one_t = 0.0                # age of the current one-line episode [s]

  # ---- model frame (20 Hz) ----
  def _frame(self, md, v, dt):
    ll = md.laneLines
    if len(ll) < 4 or len(ll[1].y) == 0:
      self.rel = [False, False]; self.meas = None; return
    x = np.asarray(ll[1].x, dtype=float)
    ys = [np.asarray(ll[1].y, dtype=float), np.asarray(ll[2].y, dtype=float)]
    probs = [float(md.laneLineProbs[1]), float(md.laneLineProbs[2])]
    stds = [float(md.laneLineStds[1]), float(md.laneLineStds[2])]
    coefs = [_near_coef(x, y) for y in ys]
    fits = [(float(c_[2]), float(c_[1])) for c_ in coefs]
    px = np.asarray(md.position.x, dtype=float); py = np.asarray(md.position.y, dtype=float)
    inn = [None, None]
    if self.prev is not None and dt > 0.0:
      s_ = max(v, 0.0) * dt
      pth = float(np.interp(s_, self.prev[1], self.prev[2])) if len(self.prev[1]) > 1 else 0.0
      for i in (0, 1):
        e_ = fits[i][0] - (float(np.polyval(self.prev[0][i], s_)) - pth)
        inn[i] = e_ if abs(e_) <= INN_JUMP else None
    self.prev = (coefs, px, py)
    jump = False
    for i in (0, 1):
      y0, a = fits[i]
      if self.y0[i] is not None:
        step = y0 - self.y0[i]
        res = abs(step - self.ydot[i] * dt)
        self.jit[i] += (dt / (0.5 + dt)) * (min(res, 1.0) - self.jit[i])
        if res > LINE_JUMP and probs[i] >= P_OFF:
          jump = True; self.ydot[i] = 0.0
        else:
          self.ydot[i] += (dt / (TAU_RATE + dt)) * (float(np.clip(step / dt, -2.0, 2.0)) - self.ydot[i])
      self.y0[i] = y0
      ok = probs[i] >= (P_OFF if self.rel[i] else P_ON) and stds[i] <= STD_MAX and self.jit[i] <= JIT_MAX
      self.rel[i] = bool(ok)
    both = self.rel[0] and self.rel[1]
    for i in (0, 1):
      if inn[i] is None or not self.rel[i]:
        self.cu[i] = 0.0 if not self.rel[i] else self.cu[i]; continue
      if both:
        self.ib[i] += (dt / (TAU_B + dt)) * (inn[i] / dt - self.ib[i]); self.cu[i] = 0.0
      else:
        self.cu[i] = self.cu[i] * max(1.0 - dt / TAU_C, 0.0) + (inn[i] - self.ib[i] * dt)
        if abs(self.cu[i]) > C_OFF:
          self.moved = True
    gaps = [-fits[0][0] - HALF_W, fits[1][0] - HALF_W]
    for i in (0, 1):
      self.g_f[i] = _lpf(self.g_f[i], gaps[i], dt, TAU_GAP) if self.rel[i] else None
    if jump:
      self.block_t = BLOCK_T
    w0 = fits[1][0] - fits[0][0]
    w30 = float(np.interp(X_FAR, x, ys[1]) - np.interp(X_FAR, x, ys[0]))
    if self.rel[0] and self.rel[1]:
      wjump = self.w_f is not None and abs(w0 - self.w_f) > W_JUMP
      if wjump or not (W_MIN <= w0 <= W_MAX) or abs(w30 - w0) > W_AHEAD:
        self.mem = None; self.both_t = 0.0; self.w_f = None if wjump else self.w_f
      else:
        self.w_f = _lpf(self.w_f, w0, dt, TAU_W)
        self.both_t += dt
      if self.both_t >= BOTH_T:
        target = max(self.w_f / 2.0 - HALF_W, MIN_GAP_FLOOR)
        min_gap = float(np.clip(min(self.g_f[0], self.g_f[1]), MIN_GAP_FLOOR, target))
        self.mem = dict(target=target, min_gap=min_gap, t=0.0, d=0.0)
      self.meas = None
      return
    self.w_f = None; self.both_t = 0.0
    if self.rel[0] == self.rel[1]:
      self.meas = None; return
    i = 0 if self.rel[0] else 1
    line_d = float(np.interp(X_FAR, x, ys[i])) - fits[i][0]
    path_d = float(np.interp(X_FAR, px, py) - py[0]) if len(px) > 1 and px[-1] > X_FAR else line_d
    self.meas = (i, gaps[i], self.ydot[i], abs(line_d - path_d) <= DIVERGE)

  # ---- controlsd tick (100 Hz) ----
  def update(self, md, v, laneless, lane_change, pressed, model_curv):
    """returns the curvature [1/m] to add to the laneless model desiredCurvature (0.0 when not holding)"""
    fid = getattr(md, 'frameId', None)
    key = fid if fid is not None else (float(md.laneLineProbs[1]), float(md.laneLineProbs[2]), float(md.laneLines[1].y[0]), float(md.laneLines[2].y[0])) \
      if len(md.laneLines) >= 4 and len(md.laneLines[1].y) and len(md.laneLineProbs) >= 4 else None
    self.frame_dt += DT
    if key != self.key:
      self.key = key
      if key is not None:
        self._frame(md, v, min(self.frame_dt, 0.2))
      else:
        self.rel = [False, False]; self.meas = None
      self.frame_dt = 0.0
    self.block_t = max(self.block_t - DT, 0.0)
    if self.moved:
      # the visible line moved by itself relative to the laneless path: memory no longer valid
      self.moved = False; self.mem = None; self.recede_t = 0.0
    if self.mem is not None and not (self.rel[0] and self.rel[1]):
      self.mem["t"] += DT; self.mem["d"] += v * DT
      if self.mem["t"] > MEM_T or self.mem["d"] > MEM_D:
        self.mem = None
    m = self.meas if self.frame_dt <= STALE_T else None
    self.one_t = self.one_t + DT if m is not None else 0.0
    ok = (laneless and not lane_change and not pressed and v >= V_MIN and abs(model_curv) < K_TURN and self.block_t <= 0.0
          and self.one_t >= T_PROBE and self.mem is not None and m is not None and m[3] and -0.2 <= m[1] <= self.mem["target"] + S_MAX + 0.5)
    v2 = max(v, V_MIN) ** 2
    close = False
    if ok:
      i, gap, sd = m[0], m[1], m[2]
      tg, mg = self.mem["target"], self.mem["min_gap"]
      s = (tg - gap) if i == 0 else (gap - tg)
      pushing = (self.w >= 0.99 and abs(self.k_out) * v2 >= 0.4 * A_MAX and self.k_out * s > 0.0 and s * sd > 0.0
                 and abs(sd) > RECEDE_RATE)
      self.recede_t = self.recede_t + DT if pushing else 0.0
      if abs(s) > S_OFF or self.recede_t >= RECEDE_T:
        # the visible line moved away by itself (merge / split / widening lane) or the memory does not fit any more
        self.mem = None; self.recede_t = 0.0; ok = False
    if ok:
      if gap < mg:
        close = True
        s += (mg - gap) if i == 0 else -(mg - gap)
      s = float(np.clip(s, -S_MAX, S_MAX))
      if abs(s) < 0.4:
        self.integ = float(np.clip(self.integ + KI * s * DT, -I_MAX, I_MAX))
      # openpilot curvature and model y are both right-positive (controlsd: curvature = -VM.calc_curvature): s'' = -v^2 k
      self.k_raw = (OMEGA ** 2 * (s + self.integ) + 2.0 * ZETA * OMEGA * sd) / v2
      self.w = min(self.w + DT / RAMP_T, 1.0)
    else:
      self.integ *= 0.99
      self.w = max(self.w - DT / RAMP_T, 0.0)
    self.active = ok
    dk = J_MAX / v2 * DT
    if self.w <= 0.0 and not ok and abs(self.k_out) <= dk:
      self.k_out = 0.0; self.integ = 0.0
      return 0.0
    amax = (A_MAX_CLOSE if close else A_MAX) / v2
    g = self.w * self.w * (3.0 - 2.0 * self.w)
    fade = 1.0
    if self.meas is not None and C_OFF > C_FADE0:
      fade = float(np.clip((C_OFF - abs(self.cu[self.meas[0]])) / (C_OFF - C_FADE0), 0.0, 1.0))
    tgt = g * fade * float(np.clip(self.k_raw, -amax, amax))
    self.k_out = float(np.clip(tgt, self.k_out - dk, self.k_out + dk))
    return self.k_out


def k9_one_lane_hold_step(controls, CS, CC, model_v2, lane_change_off):
  """called from controlsd.state_control every tick; returns the curvature to add to the laneless model command.
  Param read every 10 ticks (0.1 s); anything but 1 = off: state dropped, 0.0 returned, nothing else touched."""
  n = getattr(controls, 'k9_olh_read_n', 0)
  if n <= 0:
    controls.k9_olh_mode = k9_one_lane_hold_mode(controls.params)
    n = 10
  controls.k9_olh_read_n = n - 1
  if controls.k9_olh_mode != 1:
    controls.k9_olh = None
    return 0.0
  if getattr(controls, 'k9_olh', None) is None:
    controls.k9_olh = OneLaneHold()
  laneless = bool(CC.latActive) and not controls.is_vw_meb and not controls.lanefull_mode_enabled
  lane_change = model_v2.meta.laneChangeState != lane_change_off or bool(CS.leftBlinker) or bool(CS.rightBlinker)
  return controls.k9_olh.update(model_v2, float(CS.vEgo), laneless, lane_change, bool(CS.steeringPressed),
                                float(model_v2.action.desiredCurvature))
