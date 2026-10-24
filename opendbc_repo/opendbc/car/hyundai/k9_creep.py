"""K9 camera-SCC stop feathering ("creep fix"), selected by K9StopMode.

K9StopMode (menu since 2026-10-25, the driver's orders 00:36 "v12 원본 그대로 넣어놓기도 해보고, K9 맞춰 넣는 것도 v12k9으로
해서 넣어봐" / "브레이크 앞전꺼는 다 별로니 지워" and 00:39 "원본 그대로 수정하지 말고 올려라"):
- 0 = base braking (default; nothing here is called);
- 1 = v12 original: the creep_fix v12 page code for the Kia Niro EV, unchanged. It is NOT in this module: it is the page
  code itself, pasted into carcontroller.py / HyundaiJerk.make_jerk behind `if self.k9_v12_orig:` / `if v12_orig:`
  (V12_ORIG_MENU below; test_k9_stop_v12.py pins it line by line against the page);
- 2 = v12k9: the same page v12 fitted to the K9 the way v11 was (kind V12K9 below).
Any other value (3 or more, negative, unreadable) acts as 0. The modes never stack.
The 2026-10-24 menu (1 v10, 2 v10 + light shudder fix, 3 v11) is deleted with its code (kinds 2 v10, 6 v10 + light shudder
fix and 7 v11 are gone). Stored values are migrated once at manager start (openpilot/selfdrive/car/k9_stop_mode_migration.py,
marker K9StopModeVer 3): old 1 / 2 / 3 -> 2 (v12k9), 0 unchanged. Earlier menu history is in git log.
Throughout this module "mode N" / "kind N" means the K9Creep kind N, NOT the menu value: kind 1 = v9 (not on the menu; kept
only as the test reference of "never brake less than v9"), kind 8 = v12k9 (menu 2).

Sources: https://leegrt.org/oppo/creep_fix/ (selectMode carcontroller.py code and changelog, written for the Kia Niro EV CAN,
VEgoStopping 15). No license is stated on that page; kind 1 is an independent re-implementation of the v9 algorithm and its
published constants, kind 8 re-implements v12 with the K9 changes below. Designed for an EV with motor creep and regen; NOT
verified on the K9.

Kind 1 (v9, test reference), while ON and active:
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

Kind 8 (v12k9, K9StopMode 2; log replay and simulator only, NOT verified on the vehicle). It is bit for bit the K9 v12 of
2026-10-25 01:10 (out/brake_v12): the page v11 fitted to the K9 as in 04c151d5 (out/brake_v11_1247) plus the page v12 pid rate
limit and jerk profile. Against v9:
- stopping phase (page v11, kept for v12k9): StopReq latch on real standstill only after V12K9_STANDSTILL_FRAMES (12)
  consecutive stopping frames with standstill; the other latch conditions (aTarget <= -1.0, eff_ac > 0.04 above 0.14 m/s,
  aEgo > 0.06, our 3.0 m lead latch) latch at once. stop_feather V12K9_FEATHER (-0.28 at 0.05 .. -0.52 at 0.42 m/s),
  min_decel V12K9_MIN_DECEL, boost BOOST (down to -0.18), release step UP_MARGIN above V12K9_UP_LOW_SPEED (0.12 m/s) and
  V12K9_UP_LOW (0.0030) below; the feather may also fall while the previous value is above stop_feather + V12K9_DOWN_ABOVE
  (0.02); latch hold ramp V12K9_HOLD_STEP (0.0025 per frame) toward min(accel, -0.50), aTarget <= -1.0: at once.
- pid: v9's creep compensation with the v12 accel fade FADE_ACCEL ([-0.80, -0.30] -> [0.25, 1.0], = page v12), then below
  V12_RATE_SPEED (1.00 m/s) with aTarget > V12_RATE_MIN_TARGET (-0.80) the page v12 two-way rate limit around the previous
  sent value: the command may fall (brake more) by at most V12_DOWN_LIM (0.004 per frame), and not at all while
  eff_ac < min(aTarget - 0.04, -0.42); above V12_UP_LIM_SPEED (0.52 m/s) it may rise (release) by at most
  interp(eff_ac - aTarget, [-0.05, 0.10], [0.0035, 0.0008]) per frame (the page's fix of the 3.25 -> 1.6 km/h release).
- jerk (applied non-CAN-FD frames): below V12_JERK_U_SPEED (1.15 m/s) jerk_u = max(jerk_u, page v12 7-point table) and
  jerk_l = max(jerk_l, interp(v, [0.55, 0.75, 0.95, 1.15], [1.0, 1.3, 1.3, 1.0])) (jerk_l_floor()). ComfortBand 0/0.
Page v12 parts NOT taken on the K9 (2026-10-25 closed-loop replay, 20 K9 stops x 72 conditions, sim only):
- the page pid creep table [0.22, 0.50, 0.92, 1.45] -> [-0.25, -0.22, -0.18, 0.0] and the PID over-braking blend (up to 30 %
  toward aTarget - 0.14 at 1.05 m/s, 0 at 1.50 m/s): both brake less, target the Niro EV regen -> hydraulic hand-over at 3.25
  km/h (the K9 has a torque converter, no regen); the K9 creep table CREEP is kept as in v10 / v11 on the K9.
- the page stopping changes eff_ac = min(LONG_ACCEL, aEgo + 0.03), min_decel to -0.20, boost -0.10 and a release step of
  at least 0.0010: they brake less before the latch; in the sim the K9 stopped longer than v11 (p90 +0.20 m, max +0.51 m)
  and closer to the lead (min -0.45 m), 164 instead of 103 stops not stopped within 3 s. v11's stopping phase is kept.
- the pid release limit 0.0040 per frame at or below 0.52 m/s: in the sim it made the last 0.2 s harder (+0.020 m/s^2 mean)
  without changing the release / re-grab count.
K9 deviations (fail toward the original command or more braking, not in the page): the 3.0 m lead latch (LEAD_LATCH_GAP) in
the stopping state; with a lead at or within 3.0 m the pid fall limit is skipped (it only brakes less); the original command
on disengage / longActive off, override, gas, brake, invalid CAN or non-finite input (reset).
Sim result (kind 8 against the old v11, same 1440 stops): stop distance median -0.005 m (max +0.028), min lead gap median
+0.004 m (min -0.028; noise band of v11 itself 0.064 m max), release-then-regrab events 1641 -> 1508.
The page's claims (Niro EV logs) are not K9 results.
"""
import math

import numpy as np

from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags

V9, V12K9 = 1, 8  # K9Creep kinds ("mode" here), not menu values; kinds 2-7 were removed (2026-10-24 / 2026-10-25)
KINDS = (V9, V12K9)
V12_ORIG_MENU = 1  # K9StopMode 1: the creep_fix v12 page code unchanged, in carcontroller.py (no K9Creep)
# K9StopMode value -> K9Creep kind (2026-10-25: 0 base, 1 v12 original (V12_ORIG_MENU, not a kind), 2 v12k9)
MENU = {2: V12K9}
# Stored values of older menus are migrated once at manager start: openpilot/selfdrive/car/k9_stop_mode_migration.py
MODES = tuple(sorted((V12_ORIG_MENU, *MENU)))  # K9StopMode values that change braking; anything else acts as 0

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
# v12k9 (kind 8, menu 2) stopping phase: the page v11 values (the K9 keeps them, see the docstring)
V12K9_STANDSTILL_FRAMES = 12  # consecutive stopping frames with standstill before the standstill latch (0.12 s at 100 Hz)
V12K9_FEATHER_BP, V12K9_FEATHER_V = [0.05, 0.11, 0.16, 0.25, 0.42], [-0.28, -0.29, -0.34, -0.42, -0.52]
V12K9_MIN_DECEL_BP, V12K9_MIN_DECEL_V = [0.06, 0.12, 0.25, 0.40], [-0.04, -0.09, -0.20, -0.30]
V12K9_DOWN_ABOVE = 0.02  # feather may fall while prev > stop_feather + this (late stopping entry)
V12K9_UP_LOW_SPEED = 0.12  # m/s
V12K9_UP_LOW = 0.0030  # release step at or below V12K9_UP_LOW_SPEED
V12K9_HOLD_STEP = 0.0025  # per frame: latch hold ramp toward min(accel, LATCH_ACCEL)
# v12k9 pid rate limit and jerk (creep_fix v12 page code)
V12_RATE_SPEED = 1.00  # m/s: pid two-way rate limit below it
V12_RATE_MIN_TARGET = -0.80  # ... while aTarget is above it
V12_DOWN_LIM = 0.004  # per frame: largest fall (more braking) of the sent pid command
V12_DOWN_HOLD_MARGIN, V12_DOWN_HOLD_EFF_AC = 0.04, -0.42  # no fall while eff_ac < min(aTarget - margin, this)
V12_UP_LIM_BP, V12_UP_LIM_V = [-0.05, 0.10], [0.0035, 0.0008]  # largest rise (release) per frame over eff_ac - aTarget
V12_UP_LIM_SPEED = 0.52  # m/s: the scheduled release limit above it only (the page's 0.0040 below is not used on the K9)
V12_JERK_U_SPEED = 1.15  # m/s
V12_JERK_U_BP, V12_JERK_U_V = [0.05, 0.08, 0.14, 0.38, 0.60, 0.92, 1.15], [0.7, 0.9, 1.6, 1.6, 1.3, 1.2, 0.8]
V12_JERK_L_BP, V12_JERK_L_V = [0.55, 0.75, 0.95, 1.15], [1.0, 1.3, 1.3, 1.0]


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


def _up_step(v_ego: float, margin: float, low_speed: float = UP_LOW_SPEED) -> float:
  return float(np.interp(margin, UP_MARGIN_BP, UP_MARGIN_V)) if v_ego > low_speed else UP_LOW


class K9Creep:
  def __init__(self, mode: int = V9):
    assert mode in KINDS, mode
    self.mode = mode
    self.v12 = mode == V12K9  # kind 8: v12k9 (own update path _update_v12k9, jerk_u / jerk_l profile; band 0)
    self.reset()

  def reset(self):
    self.latched = False
    self.last = None  # previous sent accel; None means "start from the current command"
    self.active = False  # True only on a frame where update() applied (jerk/band change follows it)
    self.ss_count = 0  # kind 8: consecutive stopping frames with standstill

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
    if self.v12:
      return self._update_v12k9(accel, a_target, stopping, pid, v_ego, v_ego_raw, a_ego, eff_ac, esp_standstill,
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
      accel = float(min(accel, LATCH_ACCEL))
      self.last = accel
      return accel, True

    margin = eff_ac - float(np.interp(v_ego, MIN_DECEL_BP, MIN_DECEL_V))
    target = float(np.interp(v_ego, FEATHER_BP, FEATHER_V)) + float(np.interp(margin, BOOST_BP, BOOST_V))
    prev = accel if self.last is None else self.last
    down = DOWN_STEP if (v_ego > DOWN_UP_SPEED and eff_ac > DOWN_EFF_AC) else 0.0
    up = _up_step(v_ego, margin, UP_LOW_SPEED)
    accel = float(min(max(target, prev - down), prev + up))
    self.last = accel  # v9 feather state
    return accel, False

  def _update_v12k9(self, accel, a_target, stopping, pid, v_ego, v_ego_raw, a_ego, eff_ac, esp_standstill, lead_visible,
                    lead_distance):
    """Kind 8 (v12k9) after the interlocks; returns (accel, stopping) to send."""
    lead_seen = lead_visible or lead_distance != 0.0
    lead_near = lead_seen and lead_distance <= LEAD_LATCH_GAP
    if not stopping:
      self.latched = False
      self.ss_count = 0
      if pid and v_ego < CREEP_BP[-1] and a_target < FADE_TARGET_BP[-1]:  # the K9 creep compensation
        comp = np.interp(v_ego, CREEP_BP, CREEP_V) * np.interp(a_target, FADE_TARGET_BP, FADE_TARGET_V)
        comp *= np.interp(accel, FADE_ACCEL_BP, FADE_ACCEL_V)
        accel = float(min(accel, max(accel + comp, CarControllerParams.ACCEL_MIN)))  # comp <= 0
        prev = accel if self.last is None else self.last
        if v_ego < V12_RATE_SPEED and a_target > V12_RATE_MIN_TARGET:  # page v12 two-way rate limit
          if lead_near:  # K9: no fall limit with a lead within 3.0 m (it only brakes less)
            down = math.inf
          else:
            down = 0.0 if eff_ac < min(a_target - V12_DOWN_HOLD_MARGIN, V12_DOWN_HOLD_EFF_AC) else V12_DOWN_LIM
          accel = float(max(accel, prev - down))
          if v_ego > V12_UP_LIM_SPEED:
            accel = float(min(accel, prev + float(np.interp(eff_ac - a_target, V12_UP_LIM_BP, V12_UP_LIM_V))))
      self.last = float(accel)
      return accel, stopping

    raw_standstill = esp_standstill or v_ego_raw < STANDSTILL_SPEED or v_ego < STANDSTILL_SPEED
    self.ss_count = self.ss_count + 1 if raw_standstill else 0
    real_standstill = self.ss_count >= V12K9_STANDSTILL_FRAMES
    if (real_standstill or a_target <= URGENT_TARGET or (v_ego > DOWN_UP_SPEED and eff_ac > 0.04) or a_ego > 0.06
        or lead_near):
      self.latched = True
    prev = accel if self.last is None else self.last
    if self.latched:
      if a_target > URGENT_TARGET:
        accel = float(max(min(accel, LATCH_ACCEL), prev - V12K9_HOLD_STEP))
      else:
        accel = float(min(accel, LATCH_ACCEL))
      self.last = accel
      return accel, True
    margin = eff_ac - float(np.interp(v_ego, V12K9_MIN_DECEL_BP, V12K9_MIN_DECEL_V))
    feather = float(np.interp(v_ego, V12K9_FEATHER_BP, V12K9_FEATHER_V))
    target = feather + float(np.interp(margin, BOOST_BP, BOOST_V))
    down = DOWN_STEP if ((v_ego > DOWN_UP_SPEED and eff_ac > DOWN_EFF_AC) or prev > feather + V12K9_DOWN_ABOVE) else 0.0
    up = float(np.interp(margin, UP_MARGIN_BP, UP_MARGIN_V)) if v_ego > V12K9_UP_LOW_SPEED else V12K9_UP_LOW
    accel = float(min(max(target, prev - down), prev + up))
    self.last = accel
    return accel, False

  def jerk_l_floor(self, jerk_l, v_ego: float):
    """Applied non-CAN-FD, non-off frames, after bands(): the JerkLowerLimit to send. Kind 8 (v12k9) only raises it below
    V12_JERK_U_SPEED (page v12 hand-over boost, 1.3 at 0.75-0.95 m/s); kind 1 returns it unchanged."""
    if self.v12 and v_ego < V12_JERK_U_SPEED:
      return max(jerk_l, float(np.interp(v_ego, V12_JERK_L_BP, V12_JERK_L_V)))
    return jerk_l

  @staticmethod
  def jerk_bands(jerk_u: float, v_ego: float):
    """v9 non-CAN-FD jerk/ComfortBand: (jerk_u, cb_upper, cb_lower)."""
    if v_ego < JERK_U_BP[-1]:
      jerk_u = max(jerk_u, float(np.interp(v_ego, JERK_U_BP, JERK_U_V)))
    return jerk_u, 0.0, 0.0

  def bands(self, jerk_u: float, v_ego: float, cb_upper: float, cb_lower: float, cb_prev: float,
            cb_prev_lower=None):
    """Applied non-CAN-FD, non-off frames: (jerk_u, cb_upper, cb_lower) to send. cb_upper/lower are the original band of
    this frame, cb_prev / cb_prev_lower the band sent on the previous make_jerk call (unused since the light band of the
    deleted kind 6 is gone; kept in the signature for the caller)."""
    if self.v12:  # kind 8: page v12 jerk_u profile below 1.15 m/s, ComfortBand 0; jerk_l: jerk_l_floor()
      if v_ego < V12_JERK_U_SPEED:
        jerk_u = max(jerk_u, float(np.interp(v_ego, V12_JERK_U_BP, V12_JERK_U_V)))
      return jerk_u, 0.0, 0.0
    return K9Creep.jerk_bands(jerk_u, v_ego)  # kind 1: exactly the v9 band (0)
