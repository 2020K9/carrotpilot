"""K9LaneCenter: laneless lane-centre hold while both lane lines are clear (experimental menu, NOT vehicle-validated).

Why (rightpull_1815, 2026-10-25 drives 1c9/1ca): on straights the torque controller follows the model desiredCurvature
(gyro curvature - command: mean -0.24e-4 1/m, no roll dependence), but the laneless model only pulls back to the lane centre
weakly (fit dc = road + c0 * offset + h * heading: c0 ~ -1.0e-3 1/m per m, h ~ -0.055). A small curvature bias anywhere in
the chain (model plan, angle-offset / roll term of the vehicle model while paramsd re-learns the fast offset) of 1e-4 1/m
therefore leaves the car 8-10 cm off centre, 2e-4 -> 15-20 cm (17:50-17:52 1ca, 18:17 1ca: +15..+22 cm right for minutes,
the model path at 1 s / 2 s stayed at the same offset). This module adds the missing centre pull from the lane lines.
Gain: K_C = 2e-3 1/m per m (2x the measured model pull), so P alone cuts a bias offset to ~1/3 and the slow integral removes the rest.

Param K9LaneCenter (int, default 0):
  0 (or anything else): off - controlsd.py behaves exactly as before (this module returns 0.0, no state kept).
  1: on. Only while controlsd follows the laneless model command (model desiredCurvature), never in lane mode:
     - both lane lines reliable (prob >= P_ON, hysteresis P_OFF, std <= STD_MAX, no line jump / jitter) for BOTH_T s,
       plausible lane width W_MIN..W_MAX, no width jump vs the filtered width, no width change ahead (merge / split)
       and no fast width change (one line moving away);
     - car offset from the lane centre e (right +, low-pass) beyond a dead band DB: a bounded curvature is added to the
       model desiredCurvature that pulls the car back: a = -(w^2 (s + I) + 2 ZETA w u_own), w = v sqrt(K_C), s = dead-banded e,
       I = slow integral (KI, |I| <= I_MAX) that cancels a steady bias, u_own = only the lateral velocity the hold itself
       added (it never brakes the model's own return, so a jumpy lane estimate cannot push the car away from the centre);
     - capped at A_MAX lateral accel, rate limited at J_MAX lateral jerk, ramped in / out over RAMP_T / RAMP_OUT s, and
       faded out in turns (model lateral accel A_TURN0 -> A_TURN1) and at low speed (V_MIN -> V_FULL);
     - off (ramp back to 0) in lane change / blinker / steering pressed / lane mode / lines unreliable.
"""
import numpy as np

K9_LANE_CENTER_PARAM = "K9LaneCenter"
DT = 0.01                    # controlsd tick
P_ON, P_OFF, STD_MAX = 0.7, 0.6, 0.2
W_MIN, W_MAX = 2.8, 4.2      # plausible lane width [m]
W_JUMP, W_AHEAD = 0.3, 0.4   # width step vs filtered / width change 0 -> X_FAR m (merge, split)
W_RATE = 0.15                # filtered |d width / dt| above = a line is moving (merge, split, branch) [m/s]
X_FAR = 30.0
LINE_JUMP, JIT_MAX = 0.3, 0.06
BOTH_T = 1.0
V_MIN, V_FULL = 3.0, 5.0     # m/s (11 -> 18 km/h): the 17:50 1ca right-hug was at 15-30 km/h
A_TURN0, A_TURN1, K_TURN = 0.8, 1.5, 0.006
DB = 0.05                    # dead band [m]: natural offsets below this are left to the model
E_OFF = 0.6                  # |offset| above = implausible (lane change, wrong lines) -> off
S_MAX = 0.4
K_C, ZETA, KI, I_MAX = 2.0e-3, 0.9, 0.10, 0.3   # centre gain in curvature per m of offset (2x the model's own c0 ~1e-3)
A_MAX, J_MAX = 0.20, 0.15
RAMP_T, RAMP_OUT = 1.0, 1.0
TAU_E, TAU_RATE, TAU_W, TAU_I = 0.5, 0.3, 1.0, 5.0
TAU_WR = 1.0                 # width-rate filter [s] (20 Hz width noise ~2 cm/frame)
STALE_T = 0.3


def k9_lane_center_mode(params) -> int:
  try:
    mode = int(params.get_int(K9_LANE_CENTER_PARAM))
  except Exception:
    return 0
  return 1 if mode == 1 else 0


def _lpf(prev, val, dt, tau):
  if prev is None:
    return val
  return prev + dt / (tau + dt) * (val - prev)


class LaneCenter:
  def __init__(self):
    self.reset()

  def reset(self):
    self.key = None
    self.frame_dt = 0.0
    self.rel = [False, False]
    self.y0 = [None, None]
    self.ydot = [0.0, 0.0]
    self.jit = [0.0, 0.0]
    self.e_f = None              # filtered offset of the car from the lane centre (right +) [m]
    self.u_f = 0.0               # filtered d e / dt [m/s]
    self.w_f = None
    self.wr_f = 0.0
    self.w_prev = None
    self.both_t = 0.0
    self.meas_ok = False
    self.integ = 0.0
    self.w = 0.0
    self.k_raw = 0.0
    self.k_out = 0.0
    self.hv = 0.0                # integral of the hold lateral accel = its own share of the lateral velocity [m/s]
    self.active = False
    self.why = 0                 # debug: 0 ok, 1 lines, 2 jump, 3 width, 4 width ahead, 5 width step, 6 width rate, 7 offset

  # ---- model frame (20 Hz) ----
  def _frame(self, md, dt):
    ll = md.laneLines
    if len(ll) < 4 or len(ll[1].y) == 0 or len(ll[2].y) == 0 or len(md.laneLineProbs) < 4:
      self.rel = [False, False]; self.meas_ok = False; self.both_t = 0.0; return
    x = np.asarray(ll[1].x, dtype=float)
    ys = [np.asarray(ll[1].y, dtype=float), np.asarray(ll[2].y, dtype=float)]
    probs = [float(md.laneLineProbs[1]), float(md.laneLineProbs[2])]
    stds = [float(md.laneLineStds[1]), float(md.laneLineStds[2])] if len(md.laneLineStds) >= 4 else [0.0, 0.0]
    jump = False
    for i in (0, 1):
      y0 = float(ys[i][0])
      if self.y0[i] is not None and dt > 0.0:
        step = y0 - self.y0[i]
        res = abs(step - self.ydot[i] * dt)
        self.jit[i] += (dt / (0.5 + dt)) * (min(res, 1.0) - self.jit[i])
        if res > LINE_JUMP:
          jump = True; self.ydot[i] = 0.0
        else:
          self.ydot[i] += (dt / (TAU_RATE + dt)) * (float(np.clip(step / dt, -2.0, 2.0)) - self.ydot[i])
      self.y0[i] = y0
      self.rel[i] = bool(probs[i] >= (P_OFF if self.rel[i] else P_ON) and stds[i] <= STD_MAX and self.jit[i] <= JIT_MAX)
    w0 = self.y0[1] - self.y0[0]
    w30 = float(np.interp(X_FAR, x, ys[1]) - np.interp(X_FAR, x, ys[0]))
    if self.w_prev is not None and dt > 0.0:
      self.wr_f += (dt / (TAU_WR + dt)) * (float(np.clip((w0 - self.w_prev) / dt, -2.0, 2.0)) - self.wr_f)
    self.w_prev = w0
    self.why = 0 if (self.rel[0] and self.rel[1]) else 1
    ok = self.rel[0] and self.rel[1] and not jump and W_MIN <= w0 <= W_MAX and abs(w30 - w0) <= W_AHEAD
    if self.why == 0 and not ok: self.why = 2 if jump else (3 if not (W_MIN <= w0 <= W_MAX) else 4)
    if ok and self.w_f is not None and abs(w0 - self.w_f) > W_JUMP:
      ok = False; self.w_f = None; self.why = 5
    if ok and abs(self.wr_f) > W_RATE:
      ok = False; self.why = 6
    e = -(self.y0[0] + self.y0[1]) / 2.0     # car right of the lane centre -> e > 0
    if ok and abs(e) > E_OFF:
      ok = False; self.why = 7
    if not ok:
      self.meas_ok = False; self.both_t = 0.0; self.e_f = None; self.u_f = 0.0
      return
    self.w_f = _lpf(self.w_f, w0, dt, TAU_W)
    e_prev = self.e_f
    self.e_f = _lpf(self.e_f, e, dt, TAU_E)
    if e_prev is not None and dt > 0.0:
      self.u_f += (dt / (TAU_RATE + dt)) * (float(np.clip((self.e_f - e_prev) / dt, -2.0, 2.0)) - self.u_f)
    self.both_t += dt
    self.meas_ok = self.both_t >= BOTH_T

  # ---- controlsd tick (100 Hz) ----
  def update(self, md, v, laneless, lane_change, pressed, model_curv):
    """returns the curvature [1/m] to add to the laneless model desiredCurvature (0.0 when not holding)"""
    fid = getattr(md, 'frameId', None)
    self.frame_dt += DT
    if fid is None or fid != self.key:
      self.key = fid
      self._frame(md, min(self.frame_dt, 0.2))
      self.frame_dt = 0.0
    v2 = max(v, V_MIN) ** 2
    alat = abs(model_curv) * v2
    gate = float(np.clip((v - V_MIN) / (V_FULL - V_MIN), 0.0, 1.0)) * float(np.clip((A_TURN1 - alat) / (A_TURN1 - A_TURN0), 0.0, 1.0))
    ok = (laneless and not lane_change and not pressed and self.meas_ok and self.frame_dt <= STALE_T and
          self.e_f is not None and abs(model_curv) < K_TURN and gate > 0.0)
    if ok:
      e = self.e_f
      s = float(np.sign(e)) * max(abs(e) - DB, 0.0)
      s = float(np.clip(s, -S_MAX, S_MAX))
      self.integ = float(np.clip(self.integ + KI * s * DT * gate, -I_MAX, I_MAX))
      u = self.u_f                 # car lateral velocity relative to the lane centre (right +)
      if u * e < 0.0:              # moving toward the centre: damp only the hold's own share of that velocity
        u = float(np.sign(u)) * min(abs(u), max(self.hv * float(np.sign(u)), 0.0))
      elif u * e > 0.0:            # moving away: the offset term already pulls back
        u = 0.0
      # openpilot curvature and model y are right-positive (controlsd: curvature = -VM.calc_curvature)
      omega = max(v, V_MIN) * np.sqrt(K_C)   # pull frequency [rad/s]: speed-independent curvature gain K_C
      self.k_raw = -(omega ** 2 * (s + self.integ) + 2.0 * ZETA * omega * u) / v2
      self.w = min(self.w + DT / RAMP_T, 1.0)
    else:
      self.integ *= max(1.0 - DT / TAU_I, 0.0) if not pressed else 1.0
      self.w = max(self.w - DT / RAMP_OUT, 0.0)
    self.active = ok
    dk = J_MAX / v2 * DT
    if self.w <= 0.0 and not ok and abs(self.k_out) <= dk:
      self.k_out = 0.0; self.hv = 0.0
      return 0.0
    g = self.w * self.w * (3.0 - 2.0 * self.w)
    amax = A_MAX / v2
    tgt = g * gate * float(np.clip(self.k_raw, -amax, amax)) if ok else g * float(np.clip(self.k_raw, -amax, amax))
    if not ok:
      # ramping out: the push only shrinks toward 0 (never grows, never changes sign)
      tgt = 0.0 if tgt * self.k_out <= 0.0 else float(np.sign(tgt)) * min(abs(tgt), abs(self.k_out))
    self.k_out = float(np.clip(tgt, self.k_out - dk, self.k_out + dk))
    self.hv += DT * self.k_out * v2
    self.hv *= max(1.0 - DT / 3.0, 0.0)
    return self.k_out


def k9_lane_center_step(controls, CS, CC, model_v2, lane_change_off):
  """called from controlsd.state_control every tick; returns the curvature to add to the laneless model command.
  Param read every 10 ticks (0.1 s); anything but 1 = off: state dropped, 0.0 returned, nothing else touched."""
  n = getattr(controls, 'k9_lc_read_n', 0)
  if n <= 0:
    controls.k9_lc_mode = k9_lane_center_mode(controls.params)
    n = 10
  controls.k9_lc_read_n = n - 1
  if controls.k9_lc_mode != 1:
    controls.k9_lc = None
    return 0.0
  if getattr(controls, 'k9_lc', None) is None:
    controls.k9_lc = LaneCenter()
  laneless = bool(CC.latActive) and not controls.is_vw_meb and not controls.lanefull_mode_enabled
  lane_change = model_v2.meta.laneChangeState != lane_change_off or bool(CS.leftBlinker) or bool(CS.rightBlinker)
  return controls.k9_lc.update(model_v2, float(CS.vEgo), laneless, lane_change, bool(CS.steeringPressed),
                               float(model_v2.action.desiredCurvature))
