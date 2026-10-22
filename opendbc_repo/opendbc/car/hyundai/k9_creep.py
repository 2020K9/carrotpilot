"""K9 camera-SCC "creep fix" stop feathering, selected by the K9CreepMode param.

K9CreepMode: 0 = base braking (this module is never called), 1 = v8, 2 = v9, 3 = v9 + shudder fix 1 (C2),
4 = v9 + shudder fix 2 (C2 + C3), 5 = v9 + shudder fix 3 (BL: below 0.8 m/s blend toward base braking),
6 = v9 + shudder fix 4 (C2 band of mode 3 + BL accel of mode 5). Any other value (7 or more, negative, unreadable)
acts as 0. Modes never stack.

Source of v8/v9: https://leegrt.org/oppo/creep_fix/ (selectMode v8/v9 carcontroller.py diffs, written for Kia Niro
EV CAN). No license is stated on that page; this is an independent re-implementation of the described algorithm and
its published constants. It was designed for an EV with motor creep and is NOT verified on the K9.
Modes 3/4/5/6 are research candidates from the 2026-10-22 K9 v9 log analysis (simulator/log replay only, NOT
verified on the vehicle).

Mode 0 (or any other car) never calls update(), so accel, StopReq, jerk and ComfortBand are the base controller's.
v8 and v9 differ only in the FEATHER, MIN_DECEL and CREEP tables and the down/up step rules (PROFILES); latch,
boost, fade and jerk are shared. Modes 3/4/5 use the v9 profile unchanged and differ from v9 only as listed below.

What it changes while ON and active:
- stopping state, before the latch: StopReq is withheld (stopping=False sent to SCC12) and the sent accel is
  rate-limited toward a speed-scheduled feather target with eff_ac feedback (FEATHER/MIN_DECEL/BOOST tables);
- latch: once real standstill (TCS13 StandStill, vEgoRaw or vEgo < 0.05), aTarget <= -1.0, eff_ac > 0.04 above
  0.14 m/s or aEgo > 0.06 is seen, StopReq is sent as the original and accel = min(original, -0.50) until the
  stopping state ends;
- pid below 1.6 m/s with aTarget < -0.05: extra braking creep_comp, faded by aTarget and the accel itself;
- jerk: only in HyundaiJerk.make_jerk's non-CAN-FD branch with longControlState != off (pid, stopping or starting),
  on frames where update() applied (active), jerk_u is raised to at least the JERK_U table below 1.4 m/s and
  ComfortBand upper/lower are set to 0 in modes 1/2 (modes 3/4: see C2). Control-off, CAN-FD and interlocked frames
  keep the original jerk/ComfortBand.
eff_ac = min(ESP12 LONG_ACCEL [m/s^2], aEgo + 0.10); LONG_ACCEL missing or non-finite falls back to aEgo.

Modes 3/4 (v9 + shudder fix; accel/StopReq/latches/interlocks/jerk_u are exactly v9's except C3):
- C2 (3 and 4): on the frames where v9 would send ComfortBand 0/0, the original band (0.9 + 0.2*accel /
  0.8 + 0.2*accel, clipped 0..1.2) times a weight w is sent. The w target is 1 at or below CB_FULL_SPEED,
  0 (= v9) at or above CB_OFF_SPEED, linear between; w moves at most CB_RAMP_STEP per make_jerk call (50 Hz, so
  0 -> 1 takes 0.5 s), so the band never jumps. After construction (mode change) or any reset (interlock) w starts
  from the band ratio actually sent on the previous call. ComfortBand is an ESC-side tolerance; it does not change
  the sent aReq or StopReq.
- C3 (4 only): in the stopping state before the latch, at or below C3_HOLD_SPEED the sent accel may rise (release)
  by at most C3_RELEASE_MAX per frame (0 = held). The result is equal to or more braking than v9 on that frame.

Mode 5 (v9 + BL; StopReq hold/timing, latches, interlocks, jerk_u and the v9 feather state are exactly v9's):
- with w = 1 at or below BL_FULL_SPEED, 0 at or above BL_OFF_SPEED, linear between (non-finite speed: 1):
  - accel: in the stopping state before the latch, the sent accel is min(v9, (1 - w) * v9 + w * original), i.e.
    blended toward the original (mode 0) command only when that brakes more; never less braking than v9. The
    feather state (self.last) keeps the v9 value, so the v9 profile itself is not changed by the blend;
  - ComfortBand: on applied frames w * the original band is sent instead of 0 (v9's 0 at or above BL_OFF_SPEED).
    The weight follows speed only (no time ramp), as in the simulated candidate BLcb_0.6.

Mode 6 (v9 + C2 band + BL accel; StopReq hold/timing, latches, interlocks, jerk_u and the v9 feather state are v9's):
- ComfortBand exactly as mode 3 (C2: original band at or below 2.0 m/s, blend to 3.0 m/s, 0.5 s ramp);
- accel: the mode-5 BL blend times C2BL_GAIN (below 0.8 m/s, more-braking side only), then C2BL_COMP extra braking
  times the C2 band weight target, the creep fade by aTarget and a speed fade to 0 between C2BL_COMP_FADE_BP;
  only ever more braking than v9 (never weaker), the v9 feather state (self.last) keeps the v9 value.
  Reason (2026-10-22 resume): K9 logs show the 5-9 Hz shudder at 0.6-2.0 m/s is 2-3x lower with the original band
  (mode 0) than with band 0 (v9) at the same pid state and StopReq 0, while aEgo - aReq is about +0.07 m/s^2 larger
  (weaker real braking) with the band; the compensation keeps the real deceleration near v9 (toy-model sweep).

Deviations from the source, all fail toward the original command (documented, not calibrated):
- disengaged/longActive off, override, gas, brake, invalid CAN, non-finite inputs: original accel/StopReq, reset;
- lead at <= LEAD_LATCH_GAP (plan leadVisible or non-zero radar distance, zero/negative counted) latches StopReq.
"""
import math

import numpy as np

from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags

V8, V9, V9_C2, V9_C2C3, V9_BL, V9_C2BL = 1, 2, 3, 4, 5, 6  # K9CreepMode values
MODES = (V8, V9, V9_C2, V9_C2C3, V9_BL, V9_C2BL)  # K9CreepMode values that run a K9Creep; anything else acts as 0


def _up_v8(v_ego, eff_ac, margin):
  return 0.0 if (v_ego > 0.14 and eff_ac > -0.06) else 0.0035


def _up_v9(v_ego, eff_ac, margin):
  return float(np.interp(margin, [-0.08, 0.02], [0.0032, 0.0])) if v_ego > 0.10 else 0.0032


PROFILES = {
  V8: dict(feather=([0.05, 0.13, 0.22, 0.38], [-0.26, -0.28, -0.35, -0.48]),
           min_decel=([0.08, 0.13, 0.25], [0.02, -0.06, -0.14]),
           creep=([0.18, 0.40, 1.0, 1.6], [-0.18, -0.24, -0.20, 0.0]),
           down_eff_ac=-0.03, up=_up_v8),
  V9: dict(feather=([0.05, 0.14, 0.25, 0.42], [-0.31, -0.35, -0.42, -0.52]),
           min_decel=([0.08, 0.14, 0.25, 0.40], [-0.05, -0.12, -0.20, -0.30]),
           creep=([0.25, 0.55, 1.0, 1.6], [-0.26, -0.26, -0.20, 0.0]),
           down_eff_ac=-0.02, up=_up_v9),
}
PROFILE_OF_MODE = {V8: V8, V9: V9, V9_C2: V9, V9_C2C3: V9, V9_BL: V9, V9_C2BL: V9}
BOOST_BP, BOOST_V = [0.0, 0.15], [0.0, -0.18]
DOWN_STEP = 0.005
DOWN_UP_SPEED = 0.14  # m/s, both versions: down_step only above it, and the weak-decel latch
FADE_TARGET_BP, FADE_TARGET_V = [-0.20, -0.05], [1.0, 0.0]
FADE_ACCEL_BP, FADE_ACCEL_V = [-0.80, -0.30], [0.25, 1.0]
LATCH_ACCEL = -0.50
STANDSTILL_SPEED = 0.05
URGENT_TARGET = -1.0
JERK_U_BP, JERK_U_V = [0.5, 1.4], [2.0, 0.8]
LEAD_LATCH_GAP = 3.0  # m, deviation: lead protection is not in the source

# C2 (modes 3/4): the 2026-10-22 logs showed the 5-12 Hz shudder raised with v9 (band 0) from 0.3 up to 2.0 m/s and
# no difference at 3-12 m/s, so the original band is restored fully at or below 2.0 m/s and v9 is kept at or above
# 3.0 m/s, with a linear blend in between.
CB_FULL_SPEED = 2.0  # m/s
CB_OFF_SPEED = 3.0  # m/s
CB_RAMP_STEP = 0.04  # band weight change per make_jerk call (50 Hz): 0 -> 1 in 0.5 s
# C3 (mode 4)
C3_HOLD_SPEED = 0.14  # m/s, the v9 DOWN_UP_SPEED: below it v9 only releases (down step 0)
C3_RELEASE_MAX = 0.0  # m/s^2 per frame, 0 = hold the decel request, never release further
# BL (mode 5): simulator candidate BLcb_0.6 (2026-10-22 alternatives search, ESC model A only; not vehicle-verified)
BL_FULL_SPEED = 0.6  # m/s, at or below: full original accel (more-braking side) and full original ComfortBand
BL_OFF_SPEED = 0.8  # m/s, at or above: v9
# Mode 6: share of the BL accel blend (1 = mode 5 blend). Chosen in the 2026-10-22 resume toy-model sweep so that the
# deceleration just before the stop stays near v9 with the C2 band restored (see k9_creep docstring, mode 6).
C2BL_GAIN = 0.5
# Mode 6: extra braking while the C2 band is restored, times the band weight target and the creep fade by aTarget
# (no extra braking when aTarget >= -0.05 or the command is not braking). 0.07 m/s^2 = the K9 log tracking deficit
# with the original band at 1-2 m/s (aEgo - aReq median +0.31 with band vs +0.24 with band 0).
C2BL_COMP = 0.10
C2BL_COMP_FADE_BP = (0.3, 0.6)  # m/s: comp is 0 at or below [0], full at or above [1] (last part of the stop = v9 + BL)


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


def cb_target_weight(v_ego: float) -> float:
  """C2 band weight target: 1 = original band, 0 = v9 band (0). Non-finite speed fails toward the original band."""
  if not math.isfinite(v_ego):
    return 1.0
  return float(np.interp(v_ego, [CB_FULL_SPEED, CB_OFF_SPEED], [1.0, 0.0]))


def bl_weight(v_ego: float) -> float:
  """Mode 5 weight of the original accel/band: 1 = original, 0 = v9. Non-finite speed fails toward the original."""
  if not math.isfinite(v_ego):
    return 1.0
  return float(np.interp(v_ego, [BL_FULL_SPEED, BL_OFF_SPEED], [1.0, 0.0]))


class K9Creep:
  def __init__(self, mode: int = V9):
    assert mode in MODES, mode
    self.profile = PROFILES[PROFILE_OF_MODE[mode]]
    self.mode = mode
    self.cb_restore = mode in (V9_C2, V9_C2C3, V9_C2BL)  # C2
    self.low_hold = mode == V9_C2C3  # C3
    self.low_blend = mode in (V9_BL, V9_C2BL)  # BL accel
    self.bl_band = mode == V9_BL  # BL band (mode 5 only; mode 6 uses the C2 band)
    self.reset()

  def reset(self):
    self.latched = False
    self.last = None  # previous sent accel; None means "start from the current command"
    self.active = False  # True only on a frame where update() applied (jerk/band change follows it)
    self.cb_w = None  # C2 band weight; None = start from the band sent on the previous make_jerk call

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

    p = self.profile
    if not stopping:
      self.latched = False
      if pid and v_ego < p["creep"][0][-1] and a_target < FADE_TARGET_BP[-1]:
        comp = np.interp(v_ego, *p["creep"]) * np.interp(a_target, FADE_TARGET_BP, FADE_TARGET_V)
        comp *= np.interp(accel, FADE_ACCEL_BP, FADE_ACCEL_V)
        accel = float(min(accel, max(accel + comp, CarControllerParams.ACCEL_MIN)))  # comp <= 0
      self.last = float(accel)
      bc = self._band_comp(accel, a_target, v_ego)
      if bc > 0.0:  # mode 6 only; the v9 state (self.last) keeps the v9 value
        accel = float(max(accel - bc, CarControllerParams.ACCEL_MIN))
      return accel, stopping

    lead_close = (lead_visible or lead_distance != 0.0) and lead_distance <= LEAD_LATCH_GAP
    real_standstill = esp_standstill or v_ego_raw < STANDSTILL_SPEED or v_ego < STANDSTILL_SPEED
    if (real_standstill or a_target <= URGENT_TARGET or (v_ego > DOWN_UP_SPEED and eff_ac > 0.04) or a_ego > 0.06
        or lead_close):
      self.latched = True

    if self.latched:
      accel = float(min(accel, LATCH_ACCEL))
      self.last = accel
      return accel, True

    margin = eff_ac - float(np.interp(v_ego, *p["min_decel"]))
    target = float(np.interp(v_ego, *p["feather"])) + float(np.interp(margin, BOOST_BP, BOOST_V))
    prev = accel if self.last is None else self.last
    down = DOWN_STEP if (v_ego > DOWN_UP_SPEED and eff_ac > p["down_eff_ac"]) else 0.0
    up = float(p["up"](v_ego, eff_ac, margin))
    if self.low_hold and v_ego <= C3_HOLD_SPEED:
      up = min(up, C3_RELEASE_MAX)  # C3: no further release at the very end of the stop
    accel = float(min(max(target, prev - down), prev + up))
    self.last = accel  # v9 feather state; the BL blend below changes only the sent value
    if self.low_blend:  # BL: toward the original command only when that brakes more (min) => never weaker than v9
      w = bl_weight(v_ego) * (C2BL_GAIN if self.mode == V9_C2BL else 1.0)
      accel = float(min(accel, (1.0 - w) * accel + w * orig_accel))
    bc = self._band_comp(accel, a_target, v_ego)
    if bc > 0.0:  # mode 6 only
      accel = float(max(accel - bc, CarControllerParams.ACCEL_MIN))
    return accel, False

  def _band_comp(self, accel: float, a_target: float, v_ego: float) -> float:
    """Mode 6 extra braking (>= 0) while the C2 band is restored; 0 for every other mode."""
    if self.mode != V9_C2BL or accel >= 0.0:
      return 0.0
    lo, hi = C2BL_COMP_FADE_BP
    fade_v = 1.0 if (hi <= lo or not math.isfinite(v_ego)) else float(np.interp(v_ego, [lo, hi], [0.0, 1.0]))
    return float(C2BL_COMP * fade_v * cb_target_weight(v_ego) * np.interp(a_target, FADE_TARGET_BP, FADE_TARGET_V))

  @staticmethod
  def jerk_bands(jerk_u: float, v_ego: float):
    """Modes 1/2 non-CAN-FD jerk/ComfortBand: (jerk_u, cb_upper, cb_lower)."""
    if v_ego < JERK_U_BP[-1]:
      jerk_u = max(jerk_u, float(np.interp(v_ego, JERK_U_BP, JERK_U_V)))
    return jerk_u, 0.0, 0.0

  def bands(self, jerk_u: float, v_ego: float, cb_upper: float, cb_lower: float, cb_prev: float):
    """Applied non-CAN-FD, non-off frames: (jerk_u, cb_upper, cb_lower) to send. cb_upper/lower are the original
    band of this frame, cb_prev the upper band sent on the previous make_jerk call."""
    jerk_u, cu, cl = K9Creep.jerk_bands(jerk_u, v_ego)
    if self.bl_band:  # BL (mode 5): w * original band below BL_OFF_SPEED, v9's 0 above
      w = bl_weight(v_ego)
      return jerk_u, float(cb_upper * w), float(cb_lower * w)
    if not self.cb_restore:
      return jerk_u, cu, cl  # modes 1/2: exactly the v8/v9 band (0)
    target = cb_target_weight(v_ego)
    if self.cb_w is None:
      ok = math.isfinite(cb_prev) and math.isfinite(cb_upper) and cb_upper > 0.0
      self.cb_w = float(np.clip(cb_prev / cb_upper, 0.0, 1.0)) if ok else target
    self.cb_w = float(self.cb_w + np.clip(target - self.cb_w, -CB_RAMP_STEP, CB_RAMP_STEP))
    return jerk_u, float(cb_upper * self.cb_w), float(cb_lower * self.cb_w)
