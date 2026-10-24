"""K9 camera-SCC stop feathering ("creep fix" v9, v10, v10 + light shudder fix, v11), selected by K9StopMode.

K9StopMode (menu since 2026-10-24 evening, the driver's order 18:58 "브레이크 2.3.4 제거"): 0 = base braking (default; this
module is never called), 1 = v10, 2 = v10 + light shudder fix, 3 = v11 (MENU below). Any other value (4 or more,
negative, unreadable) acts as 0. The modes never stack.
The 2026-10-24 menu 0-6 had 2 = v9 + shudder fix, 3 = v10 + band 0.2 and 4 = v10 + shudder fix; those three are removed with
their code (kinds 3, 4, 5 below are gone), and the old 5 / 6 moved down to 2 / 3 bit for bit (same kinds 6 / 7). Stored
values are migrated once at manager start (openpilot/selfdrive/car/k9_stop_mode_migration.py, marker K9StopModeVer):
old 5 -> 2, 6 -> 3, the removed 2 / 3 / 4 -> 1 (v10: old 3 / 4 ran v10's update() bit for bit, old 2 was v9-based and
v10 is the closest kept menu), 0 / 1 unchanged.
Earlier menu history (K9CreepMode -> K9StopMode 2026-10-22, the 1d0961da / 23:16 / 07:25 renumberings) is in git log.
Throughout this module "mode N" means the K9Creep kind N, NOT the menu value: kind 1 = v9 (not on the menu; the test
reference), 2 = v10 (menu 1), 6 = v10 + light shudder fix (menu 2; V10_LIGHT), 7 = v11 (menu 3; V11). The kind numbers are
kept from the 0-6 menu, so kinds 3, 4 and 5 do not exist any more.

Source of v9: https://leegrt.org/oppo/creep_fix/ (selectMode v9 carcontroller.py diff, written for Kia Niro EV CAN).
No license is stated on that page; this is an independent re-implementation of the described algorithm and its
published constants. It was designed for an EV with motor creep and is NOT verified on the K9.
Source of v10: the same page, "구형 CAN 롱컨 통합 패치 v10" (two-stage stop_feather + soft hold ramp, Niro EV log
00000305--1). Mode 2 changes against v9, all taken from the page code and nothing else:
- stop_feather V10_FEATHER_BP/V (-0.26 at 0.05 m/s .. -0.52 at 0.42 m/s), min_decel V10_MIN_DECEL_BP/V;
- the release step up_step is the margin-scheduled one above V10_UP_LOW_SPEED (0.12 m/s; v9: 0.10), UP_LOW below;
- latch hold: accel = max(min(accel, -0.50), prev - V10_HOLD_STEP) (0.35 m/s^3 soft press from the last feather value)
  while aTarget > -1.0, else min(accel, -0.50) at once; the held value is the new prev.
Everything else (latch conditions, pid creep_comp, jerk_u / ComfortBand 0, eff_ac) is v9's. Our safety additions of
mode 1 apply to mode 2 too: lead latch at LEAD_LATCH_GAP (3.0 m) and the original command on disengage, override,
pedal, invalid CAN or non-finite input. Mode 2 is the page's design and may brake less than v9 (by design: weaker
feather below 0.25 m/s and the hold ramp); it is NOT verified on the K9.

Mode 6 (v10 + light shudder fix, K9StopMode 2 (5 before the 2026-10-24 evening trim), 2026-10-24 brake_methods candidate L0.6_V9_f0; the driver's order 08:03
"하나 더 넣어봐"; log replay and simulator only, NOT verified on the vehicle):
- update() is mode 2's (v10) code path bit for bit (accel, StopReq, feather, latch ramp, 3.0 m lead latch, interlocks) and
  jerk_u is mode 2's, so aReq, StopReq timing and the jerk limits are exactly menu 1's on every frame.
- bands(): on the frames where mode 2 sends ComfortBand 0/0, upper/lower move toward LIGHT_UPPER / LIGHT_LOWER (0.60 / 0.50
  m/s^2) at or below LIGHT_FULL_SPEED (2.5 m/s, 9 km/h), linear down to 0 at LIGHT_OFF_SPEED (3.0 m/s, ~11 km/h) and 0
  above (= menu 1). Everything is on the exact 0.02 SCC14 CAN grid (integer CB_UNIT steps): the start (the band sent on the
  previous make_jerk call) and the speed-blended target are rounded to the grid, the sent value moves at most
  LIGHT_STEP_UNITS (0.04 = 2 units) per call and lands exactly on the target. Non-finite speed targets 0 (menu 1).
- Reason (K9 rlog, the same 20 op-long stops, 5-9 Hz LONG_ACCEL 2x envelope, estimate): below 2 m/s frames with band
  0.5-0.7 shook 0.024-0.033 m/s^2 and 0.7-0.9 0.025-0.037 (slope -0.04 +- 0.07, not different from 0), against
  0.07-0.12 with band 0; at 2.0-2.5 m/s band 0.5-0.7 still cut 0.063 -> 0.046; above 2.5 m/s the band showed no effect.
  So 0.6 gives the shake estimate of the removed full-band variant with a smaller band, i.e. a smaller cost if the band
  changes the real decel (closed-loop sim: stop distance vs menu 1 -0.18 / +0.10 m median at b -0.056 / +0.03).

Mode 7 (v11, K9StopMode 3 (6 before the 2026-10-24 evening trim), 2026-10-24; the driver's order 12:46 "v11 추가해라"; log replay and simulator only, NOT
verified on the vehicle). Source: the same page, "구형 CAN 롱컨 통합 패치 v11" (selectMode v11 carcontroller.py code and the
changelog v10 -> v11 diff, written for the Kia Niro EV). Changes against mode 2 (v10) taken from the page code (the two
page pid changes NOT taken on the K9 are listed after this list):
- StopReq latch on real standstill only after V11_STANDSTILL_FRAMES (12) consecutive stopping frames with standstill
  (TCS13 StandStill, vEgoRaw or vEgo < 0.05); the count restarts on any non-standstill or non-stopping frame. The other
  latch conditions (aTarget <= -1.0, eff_ac > 0.04 above 0.14 m/s, aEgo > 0.06, our 3.0 m lead latch) latch at once.
- stop_feather V11_FEATHER_V (-0.28 at 0.05 m/s; v10 -0.26), min_decel V11_MIN_DECEL_V (-0.04 at 0.06 m/s; v10 -0.02);
  the feather may also fall (down_step) while the previous value is above stop_feather + V11_DOWN_ABOVE (0.02); the
  release step below 0.12 m/s is V11_UP_LOW (0.0030; v10 0.0032).
- latch hold ramp V11_HOLD_STEP (0.0025 per frame = 0.25 m/s^3; v10 0.0035) toward min(accel, -0.50); aTarget <= -1.0: at once.
- pid (after v10's creep compensation, unchanged): below V11_GUARD_SPEED (0.95 m/s) with aTarget > V11_GUARD_MIN_TARGET
  (-0.80) and eff_ac < V11_GUARD_EFF_AC (-0.18): no further fall than the previous sent value (re-press guard).
- jerk: on applied frames jerk_u = max(jerk_u, interp(v, V11_JERK_U_BP, V11_JERK_U_V)) below 0.65 m/s only (0.7 at
  0.05 m/s, 1.8 at 0.14-0.45 m/s, 0.8 at 0.65 m/s); v10 raised it to 0.8-2.0 below 1.4 m/s. ComfortBand 0/0 as v10.
Page v11 pid changes NOT taken on the K9 (2026-10-24 closed-loop replay, 20 K9 stops x 72 conditions, sim only): the
page lowers creep_comp to [0.22, 0.45, 0.85, 1.45] -> [-0.26, -0.22, -0.08, 0] below 1.5 m/s with the accel fade
[-0.75, -0.28] -> [0.20, 1.0], and blends a PID over-braking command (below 1.05 m/s, aTarget > -0.80, accel < aTarget -
0.12) up to 55 % toward aTarget - 0.12. Both target the Niro EV regen -> hydraulic hand-over at 3.1 km/h (the K9 has a
torque converter, no regen) and both brake less: with them the sim stop distance vs menu 1 was +0.079 m median (p90
+0.208, max +0.395) and the min lead gap -0.083 m median (min -0.486), the creep table alone +0.074 / the blend alone a
tail of +0.229 m / -0.306 m, against a noise band (menu 1, noise variants vs nominal) of 0.005 m p90 / 0.029 m max. With
v10's pid creep compensation and no blend (this mode): distance median 0.000 (max +0.061, 1 of 1440 stops above 0.05),
gap median 0.000 (min -0.012).
K9 deviations (fail toward more braking, not in the page): the 3.0 m lead latch and the interlocks (original command on
disengage, override, pedal, invalid CAN, non-finite input) are mode 2's; with a lead at or within LEAD_LATCH_GAP (3.0 m)
the re-press guard is skipped (it only ever brakes less than the upstream command). The page's claims (Niro EV logs) are
not K9 results.

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

Deviations from the source, all fail toward the original command (documented, not calibrated):
- disengaged/longActive off, override, gas, brake, invalid CAN, non-finite inputs: original accel/StopReq, reset;
- lead at <= LEAD_LATCH_GAP (plan leadVisible or non-zero radar distance, zero/negative counted) latches StopReq.
"""
import math

import numpy as np

from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags

V9, V10, V10_LIGHT, V11 = 1, 2, 6, 7  # K9Creep kinds ("mode" here), not menu values; kinds 3, 4, 5 were removed 2026-10-24
KINDS = (V9, V10, V10_LIGHT, V11)
# K9StopMode value -> kind (2026-10-24 evening menu: the old 2/3/4 removed, the old 5/6 moved to 2/3)
MENU = {1: V10, 2: V10_LIGHT, 3: V11}
# Stored values of the 0-6 menu are migrated once at manager start: openpilot/selfdrive/car/k9_stop_mode_migration.py
MODES = tuple(MENU)  # K9StopMode values that run a K9Creep; anything else acts as 0

# v9 profile (creep_fix source constants)
FEATHER_BP, FEATHER_V = [0.05, 0.14, 0.25, 0.42], [-0.31, -0.35, -0.42, -0.52]
MIN_DECEL_BP, MIN_DECEL_V = [0.08, 0.14, 0.25, 0.40], [-0.05, -0.12, -0.20, -0.30]
CREEP_BP, CREEP_V = [0.25, 0.55, 1.0, 1.6], [-0.26, -0.26, -0.20, 0.0]
DOWN_EFF_AC = -0.02
UP_MARGIN_BP, UP_MARGIN_V = [-0.08, 0.02], [0.0032, 0.0]
UP_LOW = 0.0032  # release step at or below UP_LOW_SPEED
UP_LOW_SPEED = 0.10  # m/s
# v10 (mode 2) profile (creep_fix v10 page code)
V10_FEATHER_BP, V10_FEATHER_V = [0.05, 0.11, 0.16, 0.25, 0.42], [-0.26, -0.28, -0.34, -0.42, -0.52]
V10_MIN_DECEL_BP, V10_MIN_DECEL_V = [0.06, 0.12, 0.25, 0.40], [-0.02, -0.09, -0.20, -0.30]
V10_UP_LOW_SPEED = 0.12  # m/s
V10_HOLD_STEP = 0.0035  # per frame: latch hold ramp toward min(accel, LATCH_ACCEL)
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
# Mode 6 ComfortBand (menu 2, L0.6_V9_f0): integer units of the 0.02 SCC14 CAN grid where mode 2 sends 0
CB_UNIT = 0.02  # m/s^2 per SCC14 ComfortBand bit
LIGHT_UPPER_UNITS, LIGHT_LOWER_UNITS = 30, 25  # 0.60 / 0.50 m/s^2
LIGHT_UPPER, LIGHT_LOWER = LIGHT_UPPER_UNITS / 50.0, LIGHT_LOWER_UNITS / 50.0
LIGHT_FULL_SPEED = 2.5  # m/s (9 km/h): full light band at or below
LIGHT_OFF_SPEED = 3.0  # m/s (~11 km/h): 0 (menu 1) at or above, linear between
LIGHT_STEP_UNITS = 2  # 0.04 m/s^2 per make_jerk call
# v11 (mode 7, menu 3) profile (creep_fix v11 page code)
V11_STANDSTILL_FRAMES = 12  # consecutive stopping frames with standstill before the standstill latch (0.12 s at 100 Hz)
V11_FEATHER_BP, V11_FEATHER_V = [0.05, 0.11, 0.16, 0.25, 0.42], [-0.28, -0.29, -0.34, -0.42, -0.52]
V11_MIN_DECEL_BP, V11_MIN_DECEL_V = [0.06, 0.12, 0.25, 0.40], [-0.04, -0.09, -0.20, -0.30]
V11_DOWN_ABOVE = 0.02  # feather may fall while prev > stop_feather + this (late stopping entry)
V11_UP_LOW = 0.0030  # release step at or below V10_UP_LOW_SPEED
V11_HOLD_STEP = 0.0025  # per frame: latch hold ramp toward min(accel, LATCH_ACCEL)
# (the page's pid creep table / accel fade / 1.5 m/s gate and PID over-braking blend are not used on the K9: v10's are kept)
V11_GUARD_MIN_TARGET = -0.80  # re-press guard only while aTarget is above it
V11_GUARD_SPEED = 0.95  # m/s: re-press guard below it
V11_GUARD_EFF_AC = -0.18  # re-press guard while eff_ac is below it
V11_JERK_U_SPEED = 0.65  # m/s
V11_JERK_U_BP, V11_JERK_U_V = [0.05, 0.08, 0.14, 0.45, 0.65], [0.7, 0.9, 1.8, 1.8, 0.8]



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


def light_band_target_units(level_units: int, v_ego: float) -> int:
  """Mode 6 band target in 0.02 units: level at or below LIGHT_FULL_SPEED, linear to 0 at LIGHT_OFF_SPEED, rounded to the
  grid. Non-finite speed: 0 (menu 1's band)."""
  if not math.isfinite(v_ego):
    return 0
  return int(round(level_units * float(np.interp(v_ego, [LIGHT_FULL_SPEED, LIGHT_OFF_SPEED], [1.0, 0.0]))))


def _light_toward(cur_units, prev_sent, target_units: int) -> int:
  """Mode 6: one make_jerk step on the 0.02 grid (at most LIGHT_STEP_UNITS, lands exactly on target). cur None = start
  from prev_sent (the band sent on the previous call) rounded to the grid; non-finite or unreadable starts at 0."""
  if cur_units is None:
    try:
      prev = float(prev_sent)
    except (TypeError, ValueError):
      prev = 0.0
    prev = float(np.clip(prev, 0.0, 1.26)) if math.isfinite(prev) else 0.0
    cur_units = int(round(prev / CB_UNIT))
  if abs(target_units - cur_units) <= LIGHT_STEP_UNITS:
    return int(target_units)
  return int(cur_units + (LIGHT_STEP_UNITS if target_units > cur_units else -LIGHT_STEP_UNITS))


def _up_step(v_ego: float, margin: float, low_speed: float = UP_LOW_SPEED) -> float:
  return float(np.interp(margin, UP_MARGIN_BP, UP_MARGIN_V)) if v_ego > low_speed else UP_LOW


class K9Creep:
  def __init__(self, mode: int = V9):
    assert mode in KINDS, mode
    self.mode = mode
    self.v10 = mode in (V10, V10_LIGHT)  # page v10 update() (modes 2 and 6 share it bit for bit)
    self.v10_light = mode == V10_LIGHT  # mode 6: 0.60/0.50 band below 2.5 m/s, 0 above 3.0 m/s (bands() only)
    self.v11 = mode == V11  # mode 7: page v11 (own update path _update_v11 and jerk profile; band 0 as v10)
    self.reset()

  def reset(self):
    self.latched = False
    self.last = None  # previous v9 accel; None means "start from the current command"
    self.active = False  # True only on a frame where update() applied (jerk/band change follows it)
    self.lb_u = self.lb_l = None  # mode 6 sent upper/lower band in 0.02 units; None = start from the band sent before
    self.ss_count = 0  # mode 7 consecutive stopping frames with standstill

  def update(self, *, accel: float, a_target: float, stopping: bool, pid: bool, v_ego: float, v_ego_raw: float,
             a_ego: float, long_accel, esp_standstill: bool, long_active: bool, override: bool, gas: bool,
             brake: bool, can_valid: bool, lead_visible: bool, lead_distance: float):
    """Returns (accel, stopping) to send."""
    values = (accel, a_target, v_ego, v_ego_raw, a_ego, lead_distance)
    if (not all(math.isfinite(x) for x in values) or not long_active or override or gas or brake or not can_valid):
      self.reset()
      return accel, stopping
    self.active = True
    lon_ac = long_accel if long_accel is not None and math.isfinite(long_accel) else a_ego
    eff_ac = min(lon_ac, a_ego + 0.10)
    if self.v11:
      return self._update_v11(accel, a_target, stopping, pid, v_ego, v_ego_raw, a_ego, eff_ac, esp_standstill,
                              lead_visible, lead_distance)

    if not stopping:
      self.latched = False
      if pid and v_ego < CREEP_BP[-1] and a_target < FADE_TARGET_BP[-1]:
        comp = np.interp(v_ego, CREEP_BP, CREEP_V) * np.interp(a_target, FADE_TARGET_BP, FADE_TARGET_V)
        comp *= np.interp(accel, FADE_ACCEL_BP, FADE_ACCEL_V)
        accel = float(min(accel, max(accel + comp, CarControllerParams.ACCEL_MIN)))  # comp <= 0
      self.last = float(accel)
      return accel, stopping

    lead_seen = lead_visible or lead_distance != 0.0
    lead_near = lead_seen and lead_distance <= LEAD_LATCH_GAP
    real_standstill = esp_standstill or v_ego_raw < STANDSTILL_SPEED or v_ego < STANDSTILL_SPEED
    if (real_standstill or a_target <= URGENT_TARGET or (v_ego > DOWN_UP_SPEED and eff_ac > 0.04) or a_ego > 0.06
        or lead_near):
      self.latched = True

    if self.latched:
      if self.v10:  # v10 soft hold ramp from the last feather value; urgent target: at once
        prev = accel if self.last is None else self.last
        if a_target > URGENT_TARGET:
          accel = float(max(min(accel, LATCH_ACCEL), prev - V10_HOLD_STEP))
        else:
          accel = float(min(accel, LATCH_ACCEL))
        self.last = accel
        return accel, True
      accel = float(min(accel, LATCH_ACCEL))
      self.last = accel
      return accel, True

    if self.v10:
      margin = eff_ac - float(np.interp(v_ego, V10_MIN_DECEL_BP, V10_MIN_DECEL_V))
      target = float(np.interp(v_ego, V10_FEATHER_BP, V10_FEATHER_V)) + float(np.interp(margin, BOOST_BP, BOOST_V))
    else:
      margin = eff_ac - float(np.interp(v_ego, MIN_DECEL_BP, MIN_DECEL_V))
      target = float(np.interp(v_ego, FEATHER_BP, FEATHER_V)) + float(np.interp(margin, BOOST_BP, BOOST_V))
    prev = accel if self.last is None else self.last
    down = DOWN_STEP if (v_ego > DOWN_UP_SPEED and eff_ac > DOWN_EFF_AC) else 0.0
    up = _up_step(v_ego, margin, V10_UP_LOW_SPEED if self.v10 else UP_LOW_SPEED)
    accel = float(min(max(target, prev - down), prev + up))
    self.last = accel  # v9 / v10 feather state
    return accel, False

  def _update_v11(self, accel, a_target, stopping, pid, v_ego, v_ego_raw, a_ego, eff_ac, esp_standstill, lead_visible,
                  lead_distance):
    """Mode 7 (page v11) after the interlocks; returns (accel, stopping) to send."""
    lead_seen = lead_visible or lead_distance != 0.0
    lead_near = lead_seen and lead_distance <= LEAD_LATCH_GAP
    if not stopping:
      self.latched = False
      self.ss_count = 0
      if pid and v_ego < CREEP_BP[-1] and a_target < FADE_TARGET_BP[-1]:  # v10's creep compensation (K9)
        comp = np.interp(v_ego, CREEP_BP, CREEP_V) * np.interp(a_target, FADE_TARGET_BP, FADE_TARGET_V)
        comp *= np.interp(accel, FADE_ACCEL_BP, FADE_ACCEL_V)
        accel = float(min(accel, max(accel + comp, CarControllerParams.ACCEL_MIN)))  # comp <= 0
        prev = accel if self.last is None else self.last
        if v_ego < V11_GUARD_SPEED and a_target > V11_GUARD_MIN_TARGET and eff_ac < V11_GUARD_EFF_AC and not lead_near:
          accel = float(max(accel, prev))  # page v11 re-press guard
      self.last = float(accel)
      return accel, stopping

    raw_standstill = esp_standstill or v_ego_raw < STANDSTILL_SPEED or v_ego < STANDSTILL_SPEED
    self.ss_count = self.ss_count + 1 if raw_standstill else 0
    real_standstill = self.ss_count >= V11_STANDSTILL_FRAMES
    if (real_standstill or a_target <= URGENT_TARGET or (v_ego > DOWN_UP_SPEED and eff_ac > 0.04) or a_ego > 0.06
        or lead_near):
      self.latched = True
    prev = accel if self.last is None else self.last
    if self.latched:
      if a_target > URGENT_TARGET:
        accel = float(max(min(accel, LATCH_ACCEL), prev - V11_HOLD_STEP))
      else:
        accel = float(min(accel, LATCH_ACCEL))
      self.last = accel
      return accel, True
    margin = eff_ac - float(np.interp(v_ego, V11_MIN_DECEL_BP, V11_MIN_DECEL_V))
    feather = float(np.interp(v_ego, V11_FEATHER_BP, V11_FEATHER_V))
    target = feather + float(np.interp(margin, BOOST_BP, BOOST_V))
    down = DOWN_STEP if ((v_ego > DOWN_UP_SPEED and eff_ac > DOWN_EFF_AC) or prev > feather + V11_DOWN_ABOVE) else 0.0
    up = float(np.interp(margin, UP_MARGIN_BP, UP_MARGIN_V)) if v_ego > V10_UP_LOW_SPEED else V11_UP_LOW
    accel = float(min(max(target, prev - down), prev + up))
    self.last = accel
    return accel, False

  @staticmethod
  def jerk_bands(jerk_u: float, v_ego: float):
    """v9 non-CAN-FD jerk/ComfortBand: (jerk_u, cb_upper, cb_lower)."""
    if v_ego < JERK_U_BP[-1]:
      jerk_u = max(jerk_u, float(np.interp(v_ego, JERK_U_BP, JERK_U_V)))
    return jerk_u, 0.0, 0.0

  def bands(self, jerk_u: float, v_ego: float, cb_upper: float, cb_lower: float, cb_prev: float,
            cb_prev_lower=None):
    """Applied non-CAN-FD, non-off frames: (jerk_u, cb_upper, cb_lower) to send. cb_upper/lower are the original
    band of this frame, cb_prev / cb_prev_lower the upper / lower band sent on the previous make_jerk call
    (cb_prev_lower is used by mode 6 only; None = cb_prev)."""
    if self.v11:  # mode 7: page v11 jerk_u profile below 0.65 m/s, ComfortBand 0 (as v10)
      if v_ego < V11_JERK_U_SPEED:
        jerk_u = max(jerk_u, float(np.interp(v_ego, V11_JERK_U_BP, V11_JERK_U_V)))
      return jerk_u, 0.0, 0.0
    jerk_u, cu, cl = K9Creep.jerk_bands(jerk_u, v_ego)
    if self.v10_light:  # mode 6: 0.60/0.50 below 2.5 m/s, 0 above 3.0 m/s, where mode 2 sends 0; ramped on the 0.02 grid
      self.lb_u = _light_toward(self.lb_u, cb_prev, light_band_target_units(LIGHT_UPPER_UNITS, v_ego))
      self.lb_l = _light_toward(self.lb_l, cb_prev if cb_prev_lower is None else cb_prev_lower,
                                light_band_target_units(LIGHT_LOWER_UNITS, v_ego))
      return jerk_u, self.lb_u / 50.0, self.lb_l / 50.0
    return jerk_u, cu, cl  # modes 1 and 2: exactly the v9 band (0)
