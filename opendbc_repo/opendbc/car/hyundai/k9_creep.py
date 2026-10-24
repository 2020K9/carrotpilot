"""K9 camera-SCC stop feathering ("creep fix" v9, v10, v9 + shudder fix, v10 + band 0.2, v10 + shudder fix, v10 + light
shudder fix), selected by K9StopMode.

K9StopMode (menu, 2026-10-23 07:25; 3 changed in r2 and r6; 4 and 5 added 2026-10-24): 0 = base braking (default; this
module is never called), 1 = v10, 2 = v9 + shudder fix, 3 = v10 + band 0.2, 4 = v10 + shudder fix, 5 = v10 + light shudder
fix (MENU below). Any other value (6 or more, negative, unreadable) acts as 0. The modes never stack. The bare v9 menu entry was removed on 2026-10-23 07:25 (its code path stays: 2 is built on it); the menu numbers
moved down by one (1d0961da: 1 v9 / 2 v10 / 3 v9 + shudder fix), so the new 1 and 2 are bit-identical to the 1d0961da
2 and 3. Stored values are not migrated: a stored 1 (v9 before) now selects v10, a stored 2 selects v9 + shudder fix
and a stored 3 selects v10 + band 0.2 (r6; it selected v10 + lead stop fix on the earlier worker builds).
Throughout this module "mode N" means the K9Creep kind N, NOT the menu value: kind 1 = v9 (no longer on the menu; the
base of kind 3 and the test reference), 2 = v10 (menu 1), 3 = v9 + shudder fix (menu 2), 4 = v10 + band 0.2 (menu 3;
the constant V10_BAND02, also importable by its first name V10_SHUDDER_FIX), 5 = v10 + shudder fix (menu 4; V10_SMOOTH),
6 = v10 + light shudder fix (menu 5; V10_LIGHT).
2026-10-22 23:16 menu change (driver): 2 is now a faithful port of creep_fix v10; the former 2 (v9 + shudder fix) moved
to 3 unchanged. A stored 2 therefore now selects v10.
K9StopMode replaces the deleted K9CreepMode (2026-10-22): its old 1 (v8) and 3/4/5 (shudder candidates) are gone,
new 1 is exactly the old 2; the old 6 improved in place (2026-10-22 stop_ref, sent-value gate below) was 2 and is
3 since the 23:16 menu change. The key started at 0, so a stored K9CreepMode is not reused.

Source of v9: https://leegrt.org/oppo/creep_fix/ (selectMode v9 carcontroller.py diff, written for Kia Niro EV CAN).
No license is stated on that page; this is an independent re-implementation of the described algorithm and its
published constants. It was designed for an EV with motor creep and is NOT verified on the K9.
Mode 3 comes from the 2026-10-22 K9 v9 log analysis (log replay and simulator only, NOT verified on the vehicle).
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

Mode 4 (v10 + band 0.2, 2026-10-23 brake_v10band r6, the driver's spec 17:40 "v10 with 0.2 where it sends 0"; log
replay and simulator only, NOT verified on the vehicle):
- update() is mode 2's code path bit for bit (accel, StopReq, feather, latch ramp, 3.0 m lead latch, interlocks). The r2-r5
  lead stop fix (2.5 m latch, predicted stop point) is removed from this menu.
- bands(): on exactly the frames where mode 2 sends ComfortBand 0/0 (applied, non-CAN-FD, non-off), upper and lower are
  V10_BAND (0.20 m/s^2, 10 steps of the 0.02 CAN unit) instead. jerk_u is mode 2's. The sent value moves at most
  V10_BAND_STEP (0.04, = BAND_RAMP_STEP) per make_jerk call from the band sent on the previous call (upper and lower
  each; after construction or any reset), and lands exactly on V10_BAND, so the band never jumps.
- Reason (K9 rlog 2026-10-23, estimate): the two v10 stops (band 0) shook at 5-9 Hz with 0.145 / 0.094 m/s^2 (2x envelope,
  last 2 s), the two v9 + shudder fix stops (band 0.76 / 0.80) 0.027 / 0.013. No K9 log with band 0.2 exists.

Mode 5 (v10 + shudder fix, K9StopMode 4, 2026-10-24 brake smooth_next; the driver's order 07:53 "위아래로 안그러게 수정하고
다음번호에다가 추가"; log replay and simulator only, NOT verified on the vehicle):
- update() is mode 2's (v10) code path bit for bit (accel, StopReq, feather, latch ramp, 3.0 m lead latch, interlocks), so
  the sent aReq and StopReq timing are exactly menu 1's and never weaker.
- bands(): on the frames where mode 2 sends ComfortBand 0/0, upper/lower move toward
  max(V10_BAND, original band x band_weight(v)): the original band (0.9 + 0.2*accel / 0.8 + 0.2*accel, clipped 0..1.2,
  = what mode 0 sends) at or below BAND_FULL_SPEED (2.0 m/s), linear to V10_BAND (0.20, = mode 4) at BAND_OFF_SPEED
  (3.0 m/s) and above. The sent value moves at most V10_BAND_STEP (0.04) per make_jerk call from the band sent on the
  previous call (as mode 4), so it never jumps. jerk_u is mode 2's.
- Reason (K9 rlog, 20 op-long stops 10/22 and 10/23, 5-9 Hz LONG_ACCEL 2x envelope by speed, estimate): below 2.0 m/s the
  shake was 0.073-0.120 m/s^2 with band 0 (8 v9/v10 stops) and 0.024-0.037 with the original band (12 base / v9 + shudder
  fix stops); at 2.0-7.0 m/s there was no difference (0.045-0.082 vs 0.048-0.098). The 10/23 drive also showed the shake
  only while the ESC brakes by itself (23% of the time below 15 km/h, 1.4% with the pedal). Whether the band changes the
  real deceleration is not resolved by the logs (-0.056 +- 0.109 m/s^2, r5 fit), so the request is left at v10.

Mode 6 (v10 + light shudder fix, K9StopMode 5, 2026-10-24 brake_methods candidate L0.6_V9_f0; the driver's order 08:03
"하나 더 넣어봐"; log replay and simulator only, NOT verified on the vehicle):
- update() is mode 2's (v10) code path bit for bit (accel, StopReq, feather, latch ramp, 3.0 m lead latch, interlocks) and
  jerk_u is mode 2's, so aReq, StopReq timing and the jerk limits are exactly menu 1's on every frame.
- bands(): on the frames where mode 2 sends ComfortBand 0/0, upper/lower move toward LIGHT_UPPER / LIGHT_LOWER (0.60 / 0.50
  m/s^2) at or below LIGHT_FULL_SPEED (2.5 m/s, 9 km/h), linear down to 0 at LIGHT_OFF_SPEED (3.0 m/s, ~11 km/h) and 0
  above (= menu 1). Everything is on the exact 0.02 SCC14 CAN grid (integer CB_UNIT steps): the start (the band sent on the
  previous make_jerk call) and the speed-blended target are rounded to the grid, the sent value moves at most
  V10_BAND_STEP (0.04 = 2 units) per call and lands exactly on the target. Non-finite speed targets 0 (menu 1).
- Reason (K9 rlog, the same 20 op-long stops, 5-9 Hz LONG_ACCEL 2x envelope, estimate): below 2 m/s frames with band
  0.5-0.7 shook 0.024-0.033 m/s^2 and 0.7-0.9 0.025-0.037 (slope -0.04 +- 0.07, not different from 0), against
  0.07-0.12 with band 0; at 2.0-2.5 m/s band 0.5-0.7 still cut 0.063 -> 0.046; above 2.5 m/s the band showed no effect.
  So 0.6 gives the menu 4 shake estimate with a smaller band, i.e. a smaller cost if the band changes the real decel
  (closed-loop sim: stop distance vs menu 1 -0.18 / +0.10 m median at b -0.056 / +0.03, menu 4 -0.26 / +0.13).

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

Mode 3 (v9 + shudder fix; StopReq hold/timing, latches, interlocks, jerk_u and the v9 feather state are v9's, except
the lead latch distance below):
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
  - pid below GATE_SPEED (6.4 km/h) with aTarget < -0.05: the mode 3 extra braking may grow by at most DOWN_STEP per
    frame and only while the car decelerates less than the v9 min_decel; releases pass at once. So mode 3 adds no
    release->re-press reversal (the page's re-press count); a re-press of v9 itself still passes (min).
  - stopping before the latch: the sent value moves like the v9 feather (rises at most up, or up + CATCH_UP while
    braking well and below v9; falls only by DOWN_STEP and only while margin > GATE_DOWN_MARGIN above 0.14 m/s), never
    releases above FLOOR_V once there (K9 creep: v9 left -0.07..-0.31 m/s^2 net with aReq -0.38..-0.53 over the last 0.5 s of motion) and
    is held (no release) at or below HOLD_SPEED, where the K9 wheels have already stopped (last WHL_PUL11 pulse at
    vEgo 0.08-0.25 m/s; vEgo < 0.05 and TCS13 StandStill come 0.3-0.7 s later).
  - latch: the sent value is min(v9 latch value, previous sent value), so no release at the StopReq latch.
  - the COMP_DECEL extra braking rises at most BC_RATE per frame (falls at once).
- lead latch (2026-10-22 17:06 K9 drive, rlog): the K9 planner stops 2.2-3.0 m behind a lead, so the 3.0 m lead
  latch fired in 3 of 4 mode 3 stops while the car still rolled (0.23 / 0.78 / 0.23 m/s). The K9 ESC answers StopReq
  by braking toward the sent aReq at once (LONG_ACCEL -0.2 -> -0.6..-1.1 within 0.3 s) and the car rebounds at the
  wheel stop: LONG_ACCEL jerk 8.2 / 10.6 / 8.6 m/s^3 and dDevAx 1.01 / 1.51 / 0.97, against 1.7 and 0.23 in the
  mode 3 stop that latched at standstill. Mode 3 therefore latches on a lead only at or below LEAD_LATCH_GAP_2
  (2.5 m). Between that and LEAD_LATCH_GAP (3.0 m) StopReq stays withheld, but the sent value is at most the v9
  latch value min(original, LATCH_ACCEL) (still never weaker than v9 on any frame) and never releases (held, or
  pressed further by the normal gate). Mode 1, mode 2, mode 4, mode 5 and mode 6 keep the 3.0 m latch.

Deviations from the source, all fail toward the original command (documented, not calibrated):
- disengaged/longActive off, override, gas, brake, invalid CAN, non-finite inputs: original accel/StopReq, reset;
- lead at <= LEAD_LATCH_GAP (plan leadVisible or non-zero radar distance, zero/negative counted) latches StopReq.
"""
import math

import numpy as np

from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags

V9, V10, V9_SHUDDER_FIX, V10_BAND02, V10_SMOOTH, V10_LIGHT = 1, 2, 3, 4, 5, 6  # K9Creep kinds ("mode" here), not menu values
V10_SHUDDER_FIX = V10_BAND02  # first name of kind 4 (menu 3), kept for imports (kind 5 is V10_SMOOTH, menu 4)
KINDS = (V9, V10, V9_SHUDDER_FIX, V10_BAND02, V10_SMOOTH, V10_LIGHT)
# K9StopMode value -> kind (2026-10-23 07:25 menu, 4 and 5 on 10-24)
MENU = {1: V10, 2: V9_SHUDDER_FIX, 3: V10_BAND02, 4: V10_SMOOTH, 5: V10_LIGHT}
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
LEAD_LATCH_GAP_2 = 2.5  # m, mode 3 lead latch (K9 planner stops 2.2-3.0 m behind a lead; see the docstring)
# Mode 4 ComfortBand (r6): upper = lower = V10_BAND where mode 2 sends 0, ramped by V10_BAND_STEP per make_jerk call
V10_BAND = 0.20  # m/s^2, the driver's value; 10 x the 0.02 SCC14 ComfortBand unit
V10_BAND_STEP = 0.04  # per make_jerk call (= BAND_RAMP_STEP)
# Mode 6 ComfortBand (menu 5, L0.6_V9_f0): integer units of the 0.02 SCC14 CAN grid where mode 2 sends 0
CB_UNIT = 0.02  # m/s^2 per SCC14 ComfortBand bit
LIGHT_UPPER_UNITS, LIGHT_LOWER_UNITS = 30, 25  # 0.60 / 0.50 m/s^2
LIGHT_UPPER, LIGHT_LOWER = LIGHT_UPPER_UNITS / 50.0, LIGHT_LOWER_UNITS / 50.0
LIGHT_FULL_SPEED = 2.5  # m/s (9 km/h): full light band at or below
LIGHT_OFF_SPEED = 3.0  # m/s (~11 km/h): 0 (menu 1) at or above, linear between
LIGHT_STEP_UNITS = 2  # = V10_BAND_STEP (0.04) per make_jerk call

# Mode 3 ComfortBand: the 2026-10-22 logs showed the 5-12 Hz shudder raised with v9 (band 0) from 0.3 up to 2.0 m/s
# and no difference at 3-12 m/s, so the original band is restored fully at or below 2.0 m/s and v9 is kept at or
# above 3.0 m/s, with a linear blend in between.
BAND_FULL_SPEED = 2.0  # m/s
BAND_OFF_SPEED = 3.0  # m/s
BAND_RAMP_STEP = 0.04  # band weight change per make_jerk call (50 Hz): 0 -> 1 in 0.5 s
# Mode 3 blend toward the original accel just before the stop (more-braking side only)
BLEND_FULL_SPEED = 0.6  # m/s, at or below: full blend weight
BLEND_OFF_SPEED = 0.8  # m/s, at or above: v9
BLEND_GAIN = 0.5  # share of the blend; chosen in the 2026-10-22 toy-model sweep (decel just before the stop near v9)
# Mode 3 extra braking while the band is restored, times the band weight target and the creep fade by aTarget.
# 0.07 m/s^2 = the K9 log tracking deficit with the original band at 1-2 m/s (aEgo - aReq median +0.31 with band vs
# +0.24 with band 0); 0.10 chosen in the same sweep.
COMP_DECEL = 0.10
COMP_FADE_BP = (0.3, 0.6)  # m/s: comp is 0 at or below [0], full at or above [1]
# Mode 3 stopping gate (creep_fix v8/v9 evaluation applied to K9 logs, 2026-10-22 stop_ref):
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
BC_RATE = 0.002  # mode 3 extra braking may rise at most this per frame (100 Hz): aTarget wiggles do not jerk it in


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
  """Mode 3 band weight target: 1 = original band, 0 = v9 band (0). Non-finite speed fails toward the original."""
  if not math.isfinite(v_ego):
    return 1.0
  return float(np.interp(v_ego, [BAND_FULL_SPEED, BAND_OFF_SPEED], [1.0, 0.0]))


def blend_weight(v_ego: float) -> float:
  """Mode 3 accel blend weight before BLEND_GAIN: 1 = original, 0 = v9. Non-finite speed fails toward the original."""
  if not math.isfinite(v_ego):
    return 1.0
  return float(np.interp(v_ego, [BLEND_FULL_SPEED, BLEND_OFF_SPEED], [1.0, 0.0]))


def _band_toward(cur, prev_sent, target=V10_BAND) -> float:
  """Modes 4/5: one make_jerk step of the sent band toward target (mode 4: V10_BAND; at most V10_BAND_STEP, lands exactly
  on target). cur None = start from prev_sent (the band sent on the previous call); non-finite starts at V10_BAND."""
  if cur is None:
    try:
      cur = float(prev_sent)
    except (TypeError, ValueError):
      cur = V10_BAND
    cur = float(np.clip(cur, 0.0, 1.26)) if math.isfinite(cur) else V10_BAND
  if abs(target - cur) <= V10_BAND_STEP + 1e-9:  # float residue of the 0.04 steps
    return target
  return float(cur + math.copysign(V10_BAND_STEP, target - cur))


def smooth_band_target(band: float, v_ego: float) -> float:
  """Mode 5 band target: max(V10_BAND, original band x band_weight(v)) (original at or below 2.0 m/s, V10_BAND at or
  above 3.0 m/s). Non-finite input fails toward V10_BAND (the mode 4 value)."""
  try:
    band = float(band)
  except (TypeError, ValueError):
    return V10_BAND
  if not (math.isfinite(band) and math.isfinite(v_ego)):
    return V10_BAND
  return float(max(V10_BAND, float(np.clip(band, 0.0, 1.2)) * band_weight(v_ego)))


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
    self.shudder_fix = mode == V9_SHUDDER_FIX
    self.v10 = mode in (V10, V10_BAND02, V10_SMOOTH, V10_LIGHT)  # page v10 update() (modes 2, 4, 5, 6 share it bit for bit)
    self.v10_band = mode in (V10_BAND02, V10_SMOOTH)  # modes 4/5: ComfortBand instead of 0 (bands() only)
    self.v10_smooth = mode == V10_SMOOTH  # mode 5: original band at low speed, V10_BAND above (bands() only)
    self.v10_light = mode == V10_LIGHT  # mode 6: 0.60/0.50 band below 2.5 m/s, 0 above 3.0 m/s (bands() only)
    self.reset()

  def reset(self):
    self.latched = False
    self.last = None  # previous v9 accel; None means "start from the current command"
    self.active = False  # True only on a frame where update() applied (jerk/band change follows it)
    self.cb_w = None  # mode 3 band weight; None = start from the band sent on the previous make_jerk call
    self.cb_u = self.cb_l = None  # mode 4/5 sent upper/lower band; None = start from the band sent on the previous call
    self.lb_u = self.lb_l = None  # mode 6 sent upper/lower band in 0.02 units; None = start from the band sent before
    self.sent = None  # mode 3 last sent accel (stopping gate); None = start from the current value
    self.bc = None  # mode 3 rate-limited extra braking; None = start from the current value

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
      if bc > 0.0:  # mode 3 only; the v9 state (self.last) keeps the v9 value
        accel = float(max(accel - bc, CarControllerParams.ACCEL_MIN))
      if self.shudder_fix:
        if pid and v_ego < GATE_SPEED and a_target < FADE_TARGET_BP[-1] and self.sent is not None:
          # 6.4 -> 0.2 km/h: the mode 3 extra braking never re-presses; only v9 itself may (min => never weaker)
          margin = eff_ac - float(np.interp(v_ego, MIN_DECEL_BP, MIN_DECEL_V))
          down2 = DOWN_STEP if margin > GATE_DOWN_MARGIN else 0.0
          accel = float(min(self.last, max(accel, self.sent - down2)))
        self.sent = float(accel)
      return accel, stopping

    lead_seen = lead_visible or lead_distance != 0.0
    lead_near = lead_seen and lead_distance <= LEAD_LATCH_GAP
    lead_close = lead_seen and lead_distance <= (LEAD_LATCH_GAP_2 if self.shudder_fix else LEAD_LATCH_GAP)
    real_standstill = esp_standstill or v_ego_raw < STANDSTILL_SPEED or v_ego < STANDSTILL_SPEED
    if (real_standstill or a_target <= URGENT_TARGET or (v_ego > DOWN_UP_SPEED and eff_ac > 0.04) or a_ego > 0.06
        or lead_close):
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
      if self.shudder_fix:  # mode 3: never release below the held feather value at the latch (min => never weaker)
        accel = float(accel if self.sent is None else min(accel, self.sent))
        self.sent = accel
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
    self.last = accel  # v9 feather state; the mode 3 blend below changes only the sent value
    if self.shudder_fix:  # toward the original command only when that brakes more (min) => never weaker than v9
      w = blend_weight(v_ego) * BLEND_GAIN
      accel = float(min(accel, (1.0 - w) * accel + w * orig_accel))
    bc = self._band_comp_limited(accel, a_target, v_ego)
    if bc > 0.0:  # mode 3 only
      accel = float(max(accel - bc, CarControllerParams.ACCEL_MIN))
    if self.shudder_fix:
      v9_accel = self.last
      cand = float(min(accel, np.interp(v_ego, FLOOR_BP, FLOOR_V))) if math.isfinite(v_ego) else accel
      prev = cand if self.sent is None else self.sent
      down2 = DOWN_STEP if (v_ego > DOWN_UP_SPEED and margin > GATE_DOWN_MARGIN) else 0.0
      if v_ego <= HOLD_SPEED or lead_near:  # wheels stopped, or a lead within 3 m: no release
        up2 = 0.0
      else:  # catch up toward the K9 floor while braking well (margin < 0), at most up + CATCH_UP per frame
        up2 = up + (CATCH_UP if (margin < 0.0 and prev < v9_accel) else 0.0)
      accel = float(min(max(cand, prev - down2), prev + up2, v9_accel))  # never weaker than v9
      if lead_near:  # 2.5-3.0 m: v9 has latched here; keep its latch braking without StopReq (never weaker than v9)
        accel = float(min(accel, orig_accel, LATCH_ACCEL))
      self.sent = accel
    return accel, False

  def _band_comp_limited(self, accel: float, a_target: float, v_ego: float) -> float:
    """Mode 3: _band_comp rate-limited by BC_RATE per frame (>= 0); 0 in mode 1."""
    if not self.shudder_fix:
      return 0.0
    raw = self._band_comp(accel, a_target, v_ego)
    if self.bc is not None and BC_RATE > 0.0:  # rises at most BC_RATE per frame; falls (less braking) at once
      raw = float(min(raw, self.bc + BC_RATE))
    self.bc = raw
    return raw if accel < 0.0 else 0.0

  def _band_comp(self, accel: float, a_target: float, v_ego: float) -> float:
    """Mode 3 extra braking (>= 0) while the band is restored; 0 in mode 1 or when not braking."""
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

  def bands(self, jerk_u: float, v_ego: float, cb_upper: float, cb_lower: float, cb_prev: float,
            cb_prev_lower=None):
    """Applied non-CAN-FD, non-off frames: (jerk_u, cb_upper, cb_lower) to send. cb_upper/lower are the original
    band of this frame, cb_prev / cb_prev_lower the upper / lower band sent on the previous make_jerk call
    (cb_prev_lower is used by modes 4/5/6 only; None = cb_prev)."""
    jerk_u, cu, cl = K9Creep.jerk_bands(jerk_u, v_ego)
    if self.v10_light:  # mode 6: 0.60/0.50 below 2.5 m/s, 0 above 3.0 m/s, where mode 2 sends 0; ramped on the 0.02 grid
      self.lb_u = _light_toward(self.lb_u, cb_prev, light_band_target_units(LIGHT_UPPER_UNITS, v_ego))
      self.lb_l = _light_toward(self.lb_l, cb_prev if cb_prev_lower is None else cb_prev_lower,
                                light_band_target_units(LIGHT_LOWER_UNITS, v_ego))
      return jerk_u, self.lb_u / 50.0, self.lb_l / 50.0
    if self.v10_smooth:  # mode 5: original band at low speed / V10_BAND above, where mode 2 sends 0; ramped
      self.cb_u = _band_toward(self.cb_u, cb_prev, smooth_band_target(cb_upper, v_ego))
      self.cb_l = _band_toward(self.cb_l, cb_prev if cb_prev_lower is None else cb_prev_lower,
                               smooth_band_target(cb_lower, v_ego))
      return jerk_u, self.cb_u, self.cb_l
    if self.v10_band:  # mode 4: V10_BAND where mode 2 sends 0, ramped from the band sent before
      self.cb_u = _band_toward(self.cb_u, cb_prev)
      self.cb_l = _band_toward(self.cb_l, cb_prev if cb_prev_lower is None else cb_prev_lower)
      return jerk_u, self.cb_u, self.cb_l
    if not self.shudder_fix:
      return jerk_u, cu, cl  # modes 1 and 2: exactly the v9 band (0)
    target = band_weight(v_ego)
    if self.cb_w is None:
      ok = math.isfinite(cb_prev) and math.isfinite(cb_upper) and cb_upper > 0.0
      self.cb_w = float(np.clip(cb_prev / cb_upper, 0.0, 1.0)) if ok else target
    self.cb_w = float(self.cb_w + np.clip(target - self.cb_w, -BAND_RAMP_STEP, BAND_RAMP_STEP))
    return jerk_u, float(cb_upper * self.cb_w), float(cb_lower * self.cb_w)
