"""K9 camera-SCC stop feathering ("creep fix" v9 and v9 + shudder fix), selected by the K9StopMode param.

K9StopMode: 0 = base braking (default; this module is never called), 1 = v9, 2 = v9 + shudder fix. Any other value
(3 or more, negative, unreadable) acts as 0. The two modes never stack.
K9StopMode replaces the deleted K9CreepMode (2026-10-22): its old 1 (v8) and 3/4/5 (shudder candidates) are gone,
new 1 is exactly the old 2; new 2 is the old 6 improved in place (2026-10-22 stop_ref, sent-value gate below).
The new key starts at 0, so a stored K9CreepMode is not reused (the driver picks 1 or 2 again in the menu).

Source of v9: https://leegrt.org/oppo/creep_fix/ (selectMode v9 carcontroller.py diff, written for Kia Niro EV CAN).
No license is stated on that page; this is an independent re-implementation of the described algorithm and its
published constants. It was designed for an EV with motor creep and is NOT verified on the K9.
Mode 2 comes from the 2026-10-22 K9 v9 log analysis (log replay and simulator only, NOT verified on the vehicle).

Mode 0 (or any other car) never calls update(), so accel, StopReq, jerk and ComfortBand are the base controller's.

Mode 1 (v9), while ON and active:
- stopping state, before the latch: StopReq is withheld (stopping=False sent to SCC12) and the sent accel is
  rate-limited toward a speed-scheduled feather target with eff_ac feedback (FEATHER/MIN_DECEL/BOOST tables);
- latch: once real standstill (TCS13 StandStill, vEgoRaw or vEgo < 0.05), aTarget <= -1.0, eff_ac > 0.04 above
  0.14 m/s or aEgo > 0.06 is seen, StopReq is sent as the original and accel = min(original, -0.50) until the
  stopping state ends;
- pid below 1.6 m/s with aTarget < -0.05: extra braking creep_comp, faded by aTarget and the accel itself;
- jerk: only in HyundaiJerk.make_jerk's non-CAN-FD branch with longControlState != off (pid, stopping or starting),
  on frames where update() applied (active), jerk_u is raised to at least the JERK_U table below 1.4 m/s and
  ComfortBand upper/lower are set to 0. Control-off, CAN-FD and interlocked frames keep the original jerk/ComfortBand.
eff_ac = min(ESP12 LONG_ACCEL [m/s^2], aEgo + 0.10); LONG_ACCEL missing or non-finite falls back to aEgo.

Mode 2 (v9 + shudder fix; StopReq hold/timing, latches, interlocks, jerk_u and the v9 feather state are v9's):
- ComfortBand: on the frames where v9 sends 0/0, the original band (0.9 + 0.2*accel / 0.8 + 0.2*accel, clipped
  0..1.2) times a weight w is sent. The w target is 1 at or below BAND_FULL_SPEED, 0 (= v9) at or above
  BAND_OFF_SPEED, linear between; w moves at most BAND_RAMP_STEP per make_jerk call (50 Hz, 0 -> 1 in 0.5 s), so the
  band never jumps. After construction (mode change) or any reset (interlock) w starts from the band ratio actually
  sent on the previous call. ComfortBand is an ESC-side tolerance; it does not change the sent aReq or StopReq.
- accel, only ever more braking than v9 (never weaker); the v9 feather state (self.last) keeps the v9 value:
  - stopping state before the latch: blend toward the original (mode 0) command with weight
    blend_weight(v) * BLEND_GAIN (1 at or below BLEND_FULL_SPEED, 0 at or above BLEND_OFF_SPEED), only when that
    brakes more: min(v9, (1 - w) * v9 + w * original);
  - then COMP_DECEL extra braking times the band weight target, the creep fade by aTarget and a speed fade to 0
    between COMP_FADE_BP (pid and stopping; none when aTarget >= -0.05 or the command is not braking).
  Reason (2026-10-22): K9 logs show the 5-9 Hz shudder at 0.6-2.0 m/s is 2-3x lower with the original band (mode 0)
  than with band 0 (v9) at the same pid state and StopReq 0, while aEgo - aReq is about +0.07 m/s^2 larger (weaker
  real braking) with the band; the compensation keeps the real deceleration near v9 (toy-model sweep).
- sent-value gate (the creep_fix page's own v8/v9 evaluation applied to K9 logs, 2026-10-22 stop_ref), on top of the
  above and still never weaker than v9 (min with the v9 value on every frame):
  - pid below GATE_SPEED (6.4 km/h) with aTarget < -0.05: the mode 2 extra braking may grow by at most DOWN_STEP per
    frame and only while the car decelerates less than the v9 min_decel; releases pass at once. So mode 2 adds no
    release->re-press reversal (the page's re-press count); a re-press of v9 itself still passes (min).
  - stopping before the latch: the sent value moves like the v9 feather (rises at most up, or up + CATCH_UP while
    braking well and below v9; falls only by DOWN_STEP and only while margin > GATE_DOWN_MARGIN above 0.14 m/s), never
    releases above FLOOR_V once there (K9 creep: v9 left -0.07..-0.31 m/s^2 net with aReq -0.38..-0.53 over the last 0.5 s of motion) and
    is held (no release) at or below HOLD_SPEED, where the K9 wheels have already stopped (last WHL_PUL11 pulse at
    vEgo 0.08-0.25 m/s; vEgo < 0.05 and TCS13 StandStill come 0.3-0.7 s later).
  - latch: the sent value is min(v9 latch value, previous sent value), so no release at the StopReq latch.
  - the COMP_DECEL extra braking rises at most BC_RATE per frame (falls at once).

Deviations from the source, all fail toward the original command (documented, not calibrated):
- disengaged/longActive off, override, gas, brake, invalid CAN, non-finite inputs: original accel/StopReq, reset;
- lead at <= LEAD_LATCH_GAP (plan leadVisible or non-zero radar distance, zero/negative counted) latches StopReq.
"""
import math

import numpy as np

from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags

V9, V9_SHUDDER_FIX = 1, 2  # K9StopMode values
MODES = (V9, V9_SHUDDER_FIX)  # K9StopMode values that run a K9Creep; anything else acts as 0

# v9 profile (creep_fix source constants)
FEATHER_BP, FEATHER_V = [0.05, 0.14, 0.25, 0.42], [-0.31, -0.35, -0.42, -0.52]
MIN_DECEL_BP, MIN_DECEL_V = [0.08, 0.14, 0.25, 0.40], [-0.05, -0.12, -0.20, -0.30]
CREEP_BP, CREEP_V = [0.25, 0.55, 1.0, 1.6], [-0.26, -0.26, -0.20, 0.0]
DOWN_EFF_AC = -0.02
UP_MARGIN_BP, UP_MARGIN_V = [-0.08, 0.02], [0.0032, 0.0]
UP_LOW = 0.0032  # release step at or below UP_LOW_SPEED
UP_LOW_SPEED = 0.10  # m/s
BOOST_BP, BOOST_V = [0.0, 0.15], [0.0, -0.18]
DOWN_STEP = 0.005
DOWN_UP_SPEED = 0.14  # m/s: down_step only above it, and the weak-decel latch
FADE_TARGET_BP, FADE_TARGET_V = [-0.20, -0.05], [1.0, 0.0]
FADE_ACCEL_BP, FADE_ACCEL_V = [-0.80, -0.30], [0.25, 1.0]
LATCH_ACCEL = -0.50
STANDSTILL_SPEED = 0.05
URGENT_TARGET = -1.0
JERK_U_BP, JERK_U_V = [0.5, 1.4], [2.0, 0.8]
LEAD_LATCH_GAP = 3.0  # m, deviation: lead protection is not in the source

# Mode 2 ComfortBand: the 2026-10-22 logs showed the 5-12 Hz shudder raised with v9 (band 0) from 0.3 up to 2.0 m/s
# and no difference at 3-12 m/s, so the original band is restored fully at or below 2.0 m/s and v9 is kept at or
# above 3.0 m/s, with a linear blend in between.
BAND_FULL_SPEED = 2.0  # m/s
BAND_OFF_SPEED = 3.0  # m/s
BAND_RAMP_STEP = 0.04  # band weight change per make_jerk call (50 Hz): 0 -> 1 in 0.5 s
# Mode 2 blend toward the original accel just before the stop (more-braking side only)
BLEND_FULL_SPEED = 0.6  # m/s, at or below: full blend weight
BLEND_OFF_SPEED = 0.8  # m/s, at or above: v9
BLEND_GAIN = 0.5  # share of the blend; chosen in the 2026-10-22 toy-model sweep (decel just before the stop near v9)
# Mode 2 extra braking while the band is restored, times the band weight target and the creep fade by aTarget.
# 0.07 m/s^2 = the K9 log tracking deficit with the original band at 1-2 m/s (aEgo - aReq median +0.31 with band vs
# +0.24 with band 0); 0.10 chosen in the same sweep.
COMP_DECEL = 0.10
COMP_FADE_BP = (0.3, 0.6)  # m/s: comp is 0 at or below [0], full at or above [1]
# Mode 2 stopping gate (creep_fix v8/v9 evaluation applied to K9 logs, 2026-10-22 stop_ref):
# - the sent value may only release (rise) while feathering; it may re-press (fall) only by DOWN_STEP per frame and only
#   while the car decelerates less than the v9 min_decel (creep winning = tail drag), so blend/comp never re-press;
# - K9 wheels stop (last WHL_PUL11 pulse) while vEgo still reads 0.08-0.25 m/s (vEgo < 0.05 comes 0.36-0.68 s later),
#   so at or below HOLD_SPEED the sent value is held (no release at the real stop, small step at the StopReq latch);
# - K9 torque-converter creep: v9 net decel over the last 0.5 s of real motion was only -0.07..-0.31 m/s^2 with aReq
#   -0.38..-0.53 (deficit about 0.33), so the feather releases no higher than FLOOR_V (more braking only; target net
#   decel about -0.15 m/s^2 = the page's v9 residual -0.12..-0.15).
HOLD_SPEED = 0.15  # m/s
GATE_SPEED = 1.78  # m/s (6.4 km/h, the page's re-press window): the pid-phase gate works below it
GATE_DOWN_MARGIN = 0.0  # re-press allowed only while eff_ac - min_decel > this
FLOOR_BP, FLOOR_V = [0.15, 0.45], [-0.48, -0.48]
CATCH_UP = 0.0016  # extra release per frame (100 Hz) while the sent value is below v9 and the car decelerates well
BC_RATE = 0.002  # mode 2 extra braking may rise at most this per frame (100 Hz): aTarget wiggles do not jerk it in


def k9_creep_applies(CP) -> bool:
  # KIA_K9, non-CAN-FD, camera SCC, openpilot longitudinal
  return (getattr(CP, "carFingerprint", None) == CAR.KIA_K9 and not CP.flags & HyundaiFlags.CANFD and
          bool(CP.flags & HyundaiFlags.CAMERA_SCC) and CP.openpilotLongitudinalControl)


def read_long_accel(cp):
  try:
    value = float(cp.vl["ESP12"]["LONG_ACCEL"])
  except (AttributeError, KeyError, TypeError, ValueError):
    return None
  return value if math.isfinite(value) else None


def band_weight(v_ego: float) -> float:
  """Mode 2 band weight target: 1 = original band, 0 = v9 band (0). Non-finite speed fails toward the original."""
  if not math.isfinite(v_ego):
    return 1.0
  return float(np.interp(v_ego, [BAND_FULL_SPEED, BAND_OFF_SPEED], [1.0, 0.0]))


def blend_weight(v_ego: float) -> float:
  """Mode 2 accel blend weight before BLEND_GAIN: 1 = original, 0 = v9. Non-finite speed fails toward the original."""
  if not math.isfinite(v_ego):
    return 1.0
  return float(np.interp(v_ego, [BLEND_FULL_SPEED, BLEND_OFF_SPEED], [1.0, 0.0]))


def _up_step(v_ego: float, margin: float) -> float:
  return float(np.interp(margin, UP_MARGIN_BP, UP_MARGIN_V)) if v_ego > UP_LOW_SPEED else UP_LOW


class K9Creep:
  def __init__(self, mode: int = V9):
    assert mode in MODES, mode
    self.mode = mode
    self.shudder_fix = mode == V9_SHUDDER_FIX
    self.reset()

  def reset(self):
    self.latched = False
    self.last = None  # previous v9 accel; None means "start from the current command"
    self.active = False  # True only on a frame where update() applied (jerk/band change follows it)
    self.cb_w = None  # mode 2 band weight; None = start from the band sent on the previous make_jerk call
    self.sent = None  # mode 2 last sent accel (stopping gate); None = start from the current value
    self.bc = None  # mode 2 rate-limited extra braking; None = start from the current value

  def update(self, *, accel: float, a_target: float, stopping: bool, pid: bool, v_ego: float, v_ego_raw: float,
             a_ego: float, long_accel, esp_standstill: bool, long_active: bool, override: bool, gas: bool,
             brake: bool, can_valid: bool, lead_visible: bool, lead_distance: float):
    """Returns (accel, stopping) to send."""
    values = (accel, a_target, v_ego, v_ego_raw, a_ego, lead_distance)
    if (not all(math.isfinite(x) for x in values) or not long_active or override or gas or brake or not can_valid):
      self.reset()
      return accel, stopping
    self.active = True
    orig_accel = float(accel)
    lon_ac = long_accel if long_accel is not None and math.isfinite(long_accel) else a_ego
    eff_ac = min(lon_ac, a_ego + 0.10)

    if not stopping:
      self.latched = False
      if pid and v_ego < CREEP_BP[-1] and a_target < FADE_TARGET_BP[-1]:
        comp = np.interp(v_ego, CREEP_BP, CREEP_V) * np.interp(a_target, FADE_TARGET_BP, FADE_TARGET_V)
        comp *= np.interp(accel, FADE_ACCEL_BP, FADE_ACCEL_V)
        accel = float(min(accel, max(accel + comp, CarControllerParams.ACCEL_MIN)))  # comp <= 0
      self.last = float(accel)
      bc = self._band_comp_limited(accel, a_target, v_ego)
      if bc > 0.0:  # mode 2 only; the v9 state (self.last) keeps the v9 value
        accel = float(max(accel - bc, CarControllerParams.ACCEL_MIN))
      if self.shudder_fix:
        if pid and v_ego < GATE_SPEED and a_target < FADE_TARGET_BP[-1] and self.sent is not None:
          # 6.4 -> 0.2 km/h: the mode 2 extra braking never re-presses; only v9 itself may (min => never weaker)
          margin = eff_ac - float(np.interp(v_ego, MIN_DECEL_BP, MIN_DECEL_V))
          down2 = DOWN_STEP if margin > GATE_DOWN_MARGIN else 0.0
          accel = float(min(self.last, max(accel, self.sent - down2)))
        self.sent = float(accel)
      return accel, stopping

    lead_close = (lead_visible or lead_distance != 0.0) and lead_distance <= LEAD_LATCH_GAP
    real_standstill = esp_standstill or v_ego_raw < STANDSTILL_SPEED or v_ego < STANDSTILL_SPEED
    if (real_standstill or a_target <= URGENT_TARGET or (v_ego > DOWN_UP_SPEED and eff_ac > 0.04) or a_ego > 0.06
        or lead_close):
      self.latched = True

    if self.latched:
      accel = float(min(accel, LATCH_ACCEL))
      self.last = accel
      if self.shudder_fix:  # mode 2: never release below the held feather value at the latch (min => never weaker)
        accel = float(accel if self.sent is None else min(accel, self.sent))
        self.sent = accel
      return accel, True

    margin = eff_ac - float(np.interp(v_ego, MIN_DECEL_BP, MIN_DECEL_V))
    target = float(np.interp(v_ego, FEATHER_BP, FEATHER_V)) + float(np.interp(margin, BOOST_BP, BOOST_V))
    prev = accel if self.last is None else self.last
    down = DOWN_STEP if (v_ego > DOWN_UP_SPEED and eff_ac > DOWN_EFF_AC) else 0.0
    up = _up_step(v_ego, margin)
    accel = float(min(max(target, prev - down), prev + up))
    self.last = accel  # v9 feather state; the mode 2 blend below changes only the sent value
    if self.shudder_fix:  # toward the original command only when that brakes more (min) => never weaker than v9
      w = blend_weight(v_ego) * BLEND_GAIN
      accel = float(min(accel, (1.0 - w) * accel + w * orig_accel))
    bc = self._band_comp_limited(accel, a_target, v_ego)
    if bc > 0.0:  # mode 2 only
      accel = float(max(accel - bc, CarControllerParams.ACCEL_MIN))
    if self.shudder_fix:
      v9_accel = self.last
      cand = float(min(accel, np.interp(v_ego, FLOOR_BP, FLOOR_V))) if math.isfinite(v_ego) else accel
      prev = cand if self.sent is None else self.sent
      down2 = DOWN_STEP if (v_ego > DOWN_UP_SPEED and margin > GATE_DOWN_MARGIN) else 0.0
      if v_ego <= HOLD_SPEED:
        up2 = 0.0
      else:  # catch up toward the K9 floor while braking well (margin < 0), at most up + CATCH_UP per frame
        up2 = up + (CATCH_UP if (margin < 0.0 and prev < v9_accel) else 0.0)
      accel = float(min(max(cand, prev - down2), prev + up2, v9_accel))  # never weaker than v9
      self.sent = accel
    return accel, False

  def _band_comp_limited(self, accel: float, a_target: float, v_ego: float) -> float:
    """Mode 2: _band_comp rate-limited by BC_RATE per frame (>= 0); 0 in mode 1."""
    if not self.shudder_fix:
      return 0.0
    raw = self._band_comp(accel, a_target, v_ego)
    if self.bc is not None and BC_RATE > 0.0:  # rises at most BC_RATE per frame; falls (less braking) at once
      raw = float(min(raw, self.bc + BC_RATE))
    self.bc = raw
    return raw if accel < 0.0 else 0.0

  def _band_comp(self, accel: float, a_target: float, v_ego: float) -> float:
    """Mode 2 extra braking (>= 0) while the band is restored; 0 in mode 1 or when not braking."""
    if not self.shudder_fix or accel >= 0.0:
      return 0.0
    lo, hi = COMP_FADE_BP
    fade_v = 1.0 if (hi <= lo or not math.isfinite(v_ego)) else float(np.interp(v_ego, [lo, hi], [0.0, 1.0]))
    return float(COMP_DECEL * fade_v * band_weight(v_ego) * np.interp(a_target, FADE_TARGET_BP, FADE_TARGET_V))

  @staticmethod
  def jerk_bands(jerk_u: float, v_ego: float):
    """v9 non-CAN-FD jerk/ComfortBand: (jerk_u, cb_upper, cb_lower)."""
    if v_ego < JERK_U_BP[-1]:
      jerk_u = max(jerk_u, float(np.interp(v_ego, JERK_U_BP, JERK_U_V)))
    return jerk_u, 0.0, 0.0

  def bands(self, jerk_u: float, v_ego: float, cb_upper: float, cb_lower: float, cb_prev: float):
    """Applied non-CAN-FD, non-off frames: (jerk_u, cb_upper, cb_lower) to send. cb_upper/lower are the original
    band of this frame, cb_prev the upper band sent on the previous make_jerk call."""
    jerk_u, cu, cl = K9Creep.jerk_bands(jerk_u, v_ego)
    if not self.shudder_fix:
      return jerk_u, cu, cl  # mode 1: exactly the v9 band (0)
    target = band_weight(v_ego)
    if self.cb_w is None:
      ok = math.isfinite(cb_prev) and math.isfinite(cb_upper) and cb_upper > 0.0
      self.cb_w = float(np.clip(cb_prev / cb_upper, 0.0, 1.0)) if ok else target
    self.cb_w = float(self.cb_w + np.clip(target - self.cb_w, -BAND_RAMP_STEP, BAND_RAMP_STEP))
    return jerk_u, float(cb_upper * self.cb_w), float(cb_lower * self.cb_w)
