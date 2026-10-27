"""K9 camera-SCC stop feathering ("creep fix"), selected by K9StopMode.

K9StopMode (menu since 2026-10-25, the driver's orders 00:36 "v12 원본 그대로 넣어놓기도 해보고, K9 맞춰 넣는 것도 v12k9으로
해서 넣어봐" / "브레이크 앞전꺼는 다 별로니 지워" and 00:39 "원본 그대로 수정하지 말고 올려라"):
- 0 = base braking (default; nothing here is called);
- 1 = v12 original: the creep_fix v12 page code for the Kia Niro EV, unchanged. It is NOT in this module: it is the page
  code itself, pasted into carcontroller.py / HyundaiJerk.make_jerk behind `if self.k9_v12_orig:` / `if v12_orig:`
  (V12_ORIG_MENU below; test_k9_stop_v12.py pins it line by line against the page);
- 2 = v12k9: the same page v12 fitted to the K9 the way v11 was (kind V12K9 below);
- 3 = v13 original (2026-10-26, the driver's orders 23:13/23:14 "v13나왔다" / "v12그냥놓아두고 v13추가하는걸로"): the creep_fix
  v13 page code, unchanged, pasted into carcontroller.py / HyundaiJerk.make_jerk behind `if self.k9_v13_orig:` /
  `if v13_orig:` (V13_ORIG_MENU below; test_k9_stop_v13.py pins it line by line against the page). Not a K9Creep kind.
- 4 = C1 (2026-10-27, the driver's orders 15:53 / 16:14 "c1.c1+ 3번 4번추가해" and 16:45 "그럼 4.5에넣어야지"): kind V12K9_C1 below,
  kind 8 with the C1 stop end profile (k9band 2026-10-27): end-profile integral V12K9_C1_END_KI 0.5 (kind 8: 1.5) against the
  target lagged by V12K9_C1_END_REF_TAU, no more pressing once the predicted time to stand is below V12K9_C1_END_PRE_T, the end
  profile starts at the last sent request (bumpless), no go StopReq while rolling (V12K9_C1_END_GO_SPEED 0; kind 8: 0.15);
- 5 = C1+ (same orders): kind V12K9_C1P below, C1 plus the 05:15 lead filter (K9Creep.filter_lead: leadVisible 1 with
  leadDistance 0.0 right after a far radar lead keeps that lead), the end-profile hand-over without a request step
  (V12K9_C1P_END_HANDOFF) and, with a lead in the end profile, the lead target V12K9_C1P_END_LEAD_GAP behind it, the
  predicted-gap gate V12K9_C1P_SG_* and the integral V12K9_C1P_END_KI_SAFE.
Menus 4 and 5 are separate kinds: kind 8 (menu 2) and the v13 original (menu 3) are not changed by them (bit for bit 26daac1a).
A value 4 or 5 stored before 2026-10-27 (it acted as 0 then) now selects C1 / C1+; there is no migration for it.
Any other value (6 or more, negative, unreadable) acts as 0. The modes never stack.
The 2026-10-24 menu (1 v10, 2 v10 + light shudder fix, 3 v11) is deleted with its code (kinds 2 v10, 6 v10 + light shudder
fix and 7 v11 are gone). Stored values are migrated once at manager start (openpilot/selfdrive/car/k9_stop_mode_migration.py,
marker K9StopModeVer 3): old 1 / 2 / 3 -> 2 (v12k9), 0 unchanged. Earlier menu history is in git log.
Throughout this module "mode N" / "kind N" means the K9Creep kind N, NOT the menu value: kind 1 = v9 (not on the menu; kept
only as the test reference of "never brake less than v9"), kind 8 = v12k9 (menu 2), kind 9 = C1 (menu 4), kind 10 = C1+ (menu 5).
Kinds 9 and 10 run the kind-8 code (self.v12) and differ from it only where self.c1 / self.c1p is checked.

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
  K9 fix (2026-10-25 brake_v12k9_fix): the fall limit only inside the old v11 re-press guard region (below
  V12K9_FALL_GUARD_SPEED 0.95 m/s, eff_ac < V12K9_FALL_GUARD_EFF_AC -0.18, no lead within 3.0 m), elsewhere the command
  falls at once as in v11, so for the same inputs kind 8 never sends a weaker command than v11; the release limit only while
  eff_ac >= aTarget - V12K9_UP_LIM_MARGIN (0.02): when the car already brakes harder than planned it releases as in v11.
  (f9cd0960 limited the fall everywhere below 1.0 m/s and held the release always; in the closed-loop sim the held brake
  slowed the car early and the stop phase then braked lighter: weaker-than-v9 frames 46367 vs v11 43370.)
- jerk (applied non-CAN-FD frames): below V12_JERK_U_SPEED (1.15 m/s) jerk_u = max(jerk_u, page v12 7-point table) and
  jerk_l = max(jerk_l, interp(v, [0.55, 0.75, 0.95, 1.15], [1.0, 1.3, 1.3, 1.0])) (jerk_l_floor()).
- ComfortBand (K9 fix 2026-10-25 brake_v12k9_live_fix): the original band of the frame (0.9 / 0.8 + 0.2 * accel, the same
  as menu 0) at or below V12K9_BAND_FULL_SPEED (2.5 m/s), scaled linearly to 0 at V12K9_BAND_ZERO_SPEED (3.0 m/s), 0 at or
  above it as before (the page sends 0 everywhere). Once StopReq is latched (standstill, lead within V12K9_LEAD_HARD_GAP,
  aTarget <= -1.0) it ramps back to 0 by V12K9_BAND_LATCH_STEP (0.04) per call, so the latch phase is the old menu 2 again.
  Why: on the car (2026-10-25) the three feathered band-0 stops of menus 1 and 2 shuddered below 7 km/h (17:32:41 env
  0.087, 6-7 Hz share 0.295; 17:39:05 env 0.059; 17:43:10 env 0.092, 0.285), the band-on stops of menu 0 did not (env
  0.019-0.040); in the 10/21-23 logs the same v9 logic shuddered 0.086 with band 0 and 0.019 with the band (aBasis 5-9 Hz),
  and the band only mattered below 9 km/h. The band changes no accel / StopReq / jerk value; the stop jolts of those logs
  followed a StopReq sent while rolling (lead latch, base stopping), not the band. Whether the band itself changes the
  decel is not separable from 0 in the logs (-0.056 +- 0.128 m/s^2 per unit band, 28 stops). Closed-loop sim (1440): with
  b = 0 identical stops; b = -0.056 stop distance -0.19 m median, lead overlaps 33 -> 26; b = +0.03 +0.10 m median (max
  +0.48 m), overlaps 5 -> 9, not stopped within 3 s 96 -> 132 (without the latch ramp: 5 -> 18 and 141). Log estimate and
  sim only, not car-verified.
- band compensation and stall latch (K9 fix 2026-10-26 brake_shudder_1025): over 48 logged K9 stops the band made the car
  brake weaker than asked, +0.146 +- 0.054 m/s^2 per unit band (the 28-stop estimate above was -0.056 +- 0.128). With that
  the band alone (62eb5798) stopped later in the closed-loop sim (beta 0.14: stop distance +0.39 m median, not stopped
  within 3 s 2 -> 37, lead gap min -1.39 m). So while the band is sent the request is V12K9_BAND_COMP (0.20) stronger, times
  band_w (the sent share of the original band, from the previous bands() call) and faded in over accel 0 .. -0.3 (none while
  accelerating, none at or above 3.0 m/s); self.last stays the uncompensated feather / rate-limit state. A feathered stop
  that still hangs with the band (stopping, below V12K9_STALL_SPEED 0.3 m/s, low-passed eff_ac above -0.10 for 15 frames)
  latches StopReq (the band then ramps out as on every latch). Closed-loop sim (48 stops x 210 conditions, band effect beta
  -0.06..0.25, fitted ESC model): 5-9 Hz shudder runs 9288 -> 723 (menu 0: 853), not stopped within 3 s 46 -> 9, lead gap
  min -0.02 -> 1.19 m, stop-line overrun > 0.3 m 8 / 10080 (max +0.66 m, only at beta 0.25); at beta 0.14 jmax / rebound
  paired with 6f518e35 median -0.14 / +0.006, at beta 0.25 jmax median +0.42. Log estimate and sim only, not car-verified.
- stop end profile (K9 fix 2026-10-26 brake_0241, replaces the stall latch, the weak-decel latch and the 3.0 m lead latch of
  kind 8; menus 0 / 1 / other values unchanged). Logged K9 stops: every big stop jolt (jmax 9-12) came with StopReq sent
  while still rolling (lead 3.0 m latch, weak-decel latch; 0.1-0.3 m/s still gave jmax 3.5-8.5); the quiet ones stood still
  before StopReq with a low decel at the stop. Model hypothesis (fitted, not a confirmed ESC fact): with StopReq the ESC runs
  its own hold profile instead of the request. So in the stopping state below V12K9_END_SPEED the request follows a decel
  target over speed, d(v) = sqrt(V12K9_END_DECEL^2 + 2 V12K9_END_JERK vp), vp = v - dec * V12K9_END_LAG (constant release
  jerk ending at V12K9_END_DECEL when the car stands; dec = low-passed aEgo, V12K9_END_ALPHA), closed on aEgo: request = -d +
  offset, the offset starts bumpless at (last sent request - aEgo) and integrates V12K9_END_KI * (aEgo + d) (clipped
  V12K9_END_OFF_MIN..MAX); no band compensation while it runs. With a lead (0 < gap < 12 m) d is at least V12K9_END_LEAD_K v^2
  / (2 max(gap - V12K9_END_LEAD_GAP, 0.5)) (capped 1.0), and within 3.0 m the request is never weaker than v^2 / (2 max(gap -
  V12K9_LEAD_MARGIN, 0.3)). The end profile starts only below V12K9_END_MAX_SPEED (1.5 m/s; a stopping state entered faster
  keeps the feather, whose fast release V12K9_UP_FAST acts only below V12K9_UP_FAST_SPEED 1.0 m/s); it releases the request by
  at most V12K9_END_REL per frame (2 m/s^3, the integral holds meanwhile). A car that still rolls after V12K9_END_TMAX (3.0 s) below
  V12K9_END_TMAX_SPEED (0.4 m/s) in the end profile, or crawls (below V12K9_END_CRAWL_SPEED with low-passed aEgo above V12K9_END_CRAWL_AEGO for
  V12K9_END_CRAWL_FRAMES), gets StopReq. Outside the end profile a brake-only slow trim (V12K9_TRIM_*) adds braking when the car
  brakes less than asked with the band on. StopReq while rolling only at a lead gap at or within V12K9_LEAD_HARD_GAP (1.5 m) or aTarget <= -1.0;
  otherwise after standstill as before. Closed-loop sim only (fitted ESC model with the residual held after the logged stop,
  225 conditions x 3 seeds x 40 TRAIN stops), NOT verified on the vehicle.
Page v12 parts NOT taken on the K9 (2026-10-25 closed-loop replay, 20 K9 stops x 72 conditions, sim only):
- the page pid creep table [0.22, 0.50, 0.92, 1.45] -> [-0.25, -0.22, -0.18, 0.0] and the PID over-braking blend (up to 30 %
  toward aTarget - 0.14 at 1.05 m/s, 0 at 1.50 m/s): both brake less, target the Niro EV regen -> hydraulic hand-over at 3.25
  km/h (the K9 has a torque converter, no regen); the K9 creep table CREEP is kept as in v10 / v11 on the K9.
- the page stopping changes eff_ac = min(LONG_ACCEL, aEgo + 0.03), min_decel to -0.20, boost -0.10 and a release step of
  at least 0.0010: they brake less before the latch; in the sim the K9 stopped longer than v11 (p90 +0.20 m, max +0.51 m)
  and closer to the lead (min -0.45 m), 164 instead of 103 stops not stopped within 3 s. v11's stopping phase is kept.
- the pid release limit 0.0040 per frame at or below 0.52 m/s: in the sim it made the last 0.2 s harder (+0.020 m/s^2 mean)
  without changing the release / re-grab count.
K9 deviations (fail toward the original command or more braking, not in the page): the lead latch in the stopping state
(kind 1: LEAD_LATCH_GAP 3.0 m; kind 8: V12K9_LEAD_HARD_GAP 1.5 m plus the lead terms of the end profile); with a lead at or within 3.0 m the pid fall limit is skipped (it only brakes less); the original command
on disengage / longActive off, override, gas, brake, invalid CAN or non-finite input (reset).
Sim result (kind 8 after brake_v12k9_fix against the old v11, same 1440 stops): stop distance max +0.003 m, min lead gap
min -0.004 m (noise band of v11 itself 0.064 m max), release-then-regrab events 1641 -> 1548 (f9cd0960: 1508),
weaker-than-v9 frames 43370 -> 39946 (834 instead of 1030 stops).
The page's claims (Niro EV logs) are not K9 results.
"""
import math

import numpy as np

from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags

V9, V12K9 = 1, 8  # K9Creep kinds ("mode" here), not menu values; kinds 2-7 were removed (2026-10-24 / 2026-10-25)
V12K9_C1, V12K9_C1P = 9, 10  # (2026-10-27) kind 8 with the C1 / C1+ stop end (menus 4 / 5)
KINDS = (V9, V12K9, V12K9_C1, V12K9_C1P)
V12_ORIG_MENU = 1  # K9StopMode 1: the creep_fix v12 page code unchanged, in carcontroller.py (no K9Creep)
V13_ORIG_MENU = 3  # K9StopMode 3 (2026-10-26): the creep_fix v13 page code unchanged, in carcontroller.py (no K9Creep)
# K9StopMode value -> K9Creep kind (2026-10-25: 0 base, 1 v12 original (V12_ORIG_MENU, not a kind), 2 v12k9;
# 2026-10-26: 3 v13 original (V13_ORIG_MENU, not a kind); 2026-10-27: 4 C1, 5 C1+)
MENU = {2: V12K9, 4: V12K9_C1, 5: V12K9_C1P}
# Stored values of older menus are migrated once at manager start: openpilot/selfdrive/car/k9_stop_mode_migration.py
MODES = tuple(sorted((V12_ORIG_MENU, V13_ORIG_MENU, *MENU)))  # K9StopMode values that change braking; anything else acts as 0

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
# K9 (brake_v12k9_fix): the fall limit applies only inside the old v11 re-press guard region (same speed / eff_ac values)
V12K9_FALL_GUARD_SPEED = 0.95  # m/s
V12K9_FALL_GUARD_EFF_AC = -0.18  # m/s^2: only while already decelerating at least this much
# K9 (brake_v12k9_fix): the release limit only while eff_ac >= aTarget - this (not while the car already brakes harder than
# planned: holding the brake there only made the sim car slow early and then brake lighter than v9 / v11 near the stop)
V12K9_UP_LIM_MARGIN = 0.02  # m/s^2
V12_UP_LIM_BP, V12_UP_LIM_V = [-0.05, 0.10], [0.0035, 0.0008]  # largest rise (release) per frame over eff_ac - aTarget
V12_UP_LIM_SPEED = 0.52  # m/s: the scheduled release limit above it only (the page's 0.0040 below is not used on the K9)
V12_JERK_U_SPEED = 1.15  # m/s
V12_JERK_U_BP, V12_JERK_U_V = [0.05, 0.08, 0.14, 0.38, 0.60, 0.92, 1.15], [0.7, 0.9, 1.6, 1.6, 1.3, 1.2, 0.8]
V12_JERK_L_BP, V12_JERK_L_V = [0.55, 0.75, 0.95, 1.15], [1.0, 1.3, 1.3, 1.0]
# v12k9 ComfortBand (brake_v12k9_live_fix): original band at or below FULL, linear to 0 at ZERO, 0 above (as the page)
V12K9_BAND_FULL_SPEED = 2.5  # m/s (9 km/h: the logged band effect on the 5-9 Hz shudder was below it)
V12K9_BAND_ZERO_SPEED = 3.0  # m/s
V12K9_BAND_LATCH_STEP = 0.04  # per make_jerk call (50 Hz): band ramp to 0 once StopReq is latched (0.8 -> 0 in 0.4 s)
# v12k9 band compensation (brake_shudder_1025): with the band the logged K9 stops braked weaker than asked (+0.146 +- 0.054
# m/s^2 per unit band, 48 stops); while the band is sent the decel request is this much stronger (times the sent band share)
V12K9_BAND_COMP = 0.20  # m/s^2 at the full band
V12K9_BAND_COMP_FADE = 0.3  # m/s^2: full compensation at accel <= -0.3, linear to 0 at accel 0 (none while accelerating)
# v12k9 stall latch (brake_shudder_1025): with the band the ESC does not trim a decel shortfall, so a feathered stop can hang at
# 0.06-0.15 m/s (sim); stopping, band on, below STALL_SPEED and the low-passed eff_ac above STALL_EFF_AC for STALL_FRAMES -> StopReq
V12K9_STALL_SPEED = 0.3  # m/s
V12K9_STALL_EFF_AC = -0.10  # m/s^2 (low-passed eff_ac, V12K9_STALL_ALPHA per frame)
V12K9_STALL_FRAMES = 15  # 0.15 s at 100 Hz
V12K9_STALL_ALPHA = 0.05  # low-passed eff_ac per frame (the crawl press below; the stall latch itself is gone since brake_0241)
# v12k9 stop end profile and lead handling (brake_0241, see the docstring); stopping state, before the latch
V12K9_LEAD_HARD_GAP = 1.5  # m: StopReq while rolling only at or within this lead gap (kind 8; kind 1 keeps LEAD_LATCH_GAP)
V12K9_LEAD_MARGIN = 1.0  # m: within LEAD_LATCH_GAP the request is at least v^2 / (2 max(gap - margin, V12K9_LEAD_NEED_MIN))
V12K9_LEAD_NEED_MIN = 0.3  # m
V12K9_UP_FAST, V12K9_UP_FAST_GAP = 0.008, 0.10  # feather release step at least this while prev < feather - gap
V12K9_PRESS, V12K9_PRESS_MIN = 0.005, -0.8  # weak decel (was a StopReq latch): press the request this much per frame instead
V12K9_CRAWL_SPEED, V12K9_CRAWL_EFF_AC = 0.3, -0.10  # crawl (below the speed, low-passed eff_ac above the decel): press ...
V12K9_CRAWL_STEP, V12K9_CRAWL_MIN = 0.005, -1.0  # ... this much per frame, down to this
V12K9_END_SPEED = 3.0  # m/s: the end profile runs in the stopping state below it
V12K9_END_DECEL = 0.08  # m/s^2: decel target when the car stands (brake_0241 k2: 0.10 -> 0.08)
V12K9_END_JERK = 0.35  # m/s^3: release jerk of the target
V12K9_END_LAG = 0.45  # s: speed prediction for the ESC lag
V12K9_END_DEC_MIN = 0.08  # m/s^2: floor of the measured decel in the prediction
V12K9_END_ALPHA = 0.2  # low-pass of aEgo per frame
V12K9_END_KI = 1.5  # 1/s: integral on aEgo + target
V12K9_END_OFF_MIN, V12K9_END_OFF_MAX = -1.5, 0.5  # m/s^2
V12K9_END_LEAD_GAP, V12K9_END_LEAD_K = 3.0, 1.333  # m, -: lead target: stop this far behind it (k = 4/3: release profile)
V12K9_END_LEAD_SMIN, V12K9_END_LEAD_CAP, V12K9_END_LEAD_MAX = 0.5, 1.0, 12.0  # m, m/s^2, m
V12K9_END_MAX_SPEED = 1.5  # m/s: the end profile starts only below it (stopping entered faster: the feather)
V12K9_UP_FAST_SPEED = 1.0  # m/s: the fast feather release V12K9_UP_FAST only below it
V12K9_END_REL = 0.02  # per frame: largest release (rise) of the end-profile request (2 m/s^3)
V12K9_END_TMAX, V12K9_END_TMAX_SPEED = 3.0, 0.4  # s, m/s: still rolling (above 0.02 m/s) after this long below the speed -> StopReq (k2: 2.0 -> 3.0)
V12K9_END_CRAWL_SPEED, V12K9_END_CRAWL_AEGO, V12K9_END_CRAWL_FRAMES = 0.15, -0.02, 60  # m/s, m/s^2, frames: crawl -> StopReq (k2: -0.05 -> -0.02)
# (brake_0241 k2, sim only) end profile: the predicted speed is V12K9_END_VS lower (the profile reaches V12K9_END_DECEL that much above 0;
# the car stands at about 0.05 m/s); the offset integral never releases during the first V12K9_END_HOLD_T after the start;
# aTarget above V12K9_END_GO_TARGET (the planner is about to go) below V12K9_END_GO_SPEED -> StopReq from the next frame;
# a virtual stop line v0^2 / (2 V12K9_END_LINE_DECEL) ahead of the start: d is at least V12K9_END_LINE_K v^2 / (2 max(left, V12K9_END_LINE_SMIN))
# (capped V12K9_END_LINE_CAP)
V12K9_END_VS, V12K9_END_HOLD_T = 0.03, 1.0  # m/s, s
V12K9_END_GO_TARGET, V12K9_END_GO_SPEED = 0.0, 0.15  # m/s^2, m/s
V12K9_END_LINE_DECEL, V12K9_END_LINE_K, V12K9_END_LINE_SMIN, V12K9_END_LINE_CAP = 0.4, 1.333, 0.2, 1.0  # m/s^2, -, m, m/s^2
# C1 (kinds 9 and 10, menus 4 / 5; k9band 2026-10-27, sim only, kind 8 keeps the constants above): the end profile starts at the
# last sent request (offset = last - target, not last - aEgo); its integral V12K9_C1_END_KI compares aEgo with the target passed
# through V12K9_C1_END_REF_TAU (the decel the car can show by now, no windup while the brake builds up) and stops pressing once
# the predicted time to stand v / decel is below V12K9_C1_END_PRE_T; no go StopReq while rolling (V12K9_C1_END_GO_SPEED 0)
V12K9_C1_END_KI = 0.5  # 1/s (kind 8: V12K9_END_KI 1.5, it wound up against the ESC lag)
V12K9_C1_END_GO_SPEED = 0.0  # m/s (kind 8: V12K9_END_GO_SPEED 0.15)
V12K9_C1_END_REF_TAU, V12K9_C1_END_PRE_T = 0.35, 0.5  # s, s
# C1+ (kind 10, menu 5; k9-stop-0515c 2026-10-27, sim only), on top of C1:
# - the end-profile lead target V12K9_C1P_END_LEAD_GAP behind the lead (kind 8 / C1: V12K9_END_LEAD_GAP 3.0)
# - while the lead term of the end profile asks more than the profile (or the gate below is on) the integral presses with
#   V12K9_C1P_END_KI_SAFE instead of V12K9_C1_END_KI
# - predicted-gap gate (k9band X9): lead in the end profile and v SG_LAG + v^2 / (2 max(-aEgo_lp, 0.05)) > lead - SG_GAP -> the
#   integral presses toward the decel that stops SG_GAP behind the lead; SG_REACH: the gate starts only while SG_GAP is still
#   reachable (left - v SG_LAG > SG_SMIN), then stays on for the stop; SG_K: gate decel gain
V12K9_C1P_END_LEAD_GAP = 4.0  # m
V12K9_C1P_END_KI_SAFE = 1.25  # 1/s
V12K9_C1P_SG_GAP, V12K9_C1P_SG_LAG, V12K9_C1P_SG_SMIN = 4.0, 0.45, 0.3  # m, s, m
V12K9_C1P_SG_REACH, V12K9_C1P_SG_K = True, 1.0
# - lead input (2026-10-27 05:15:17 drive: radarState lost a 40 m lead pulling away; for 2 carControl frames the HUD said
#   leadVisible 1 (longitudinalPlan.hasLead of the frame before) with leadDistance 0.0 (radarState leadOne status 0 -> dRel 0),
#   kind 8 took that for a lead at 0 m -> StopReq while rolling at 0.139 m/s, aReq -0.06 -> -0.59 -> 꾹), K9Creep.filter_lead():
#   a positive distance is used as sent, at once; only leadVisible 1 with leadDistance 0.0 changes, and only when the last positive
#   distance came at most V12K9_C1P_LF_HOLD frames ago (all of them visible without a distance) and that distance, shortened by the
#   own travel since, is still beyond V12K9_C1P_LF_FAR (the 3.0 m lead latch gap): kind 10 then uses that shortened distance.
#   The zone reference (_V12K9Ref) keeps the raw input.
V12K9_C1P_LF_HOLD, V12K9_C1P_LF_FAR = 50, 3.0  # frames, m
# - once the end profile ran in this stop and it ends (standstill, any latch) in the zone with no lead within
#   V12K9_END_LEAD_MAX (a visible lead without a distance counts as near), the sent request falls from the last sent one by at
#   most V12K9_HOLD_STEP per frame (no step from the end-profile request to the feather / latch hold request)
V12K9_C1P_END_HANDOFF = True
# zone (2026-10-26, the driver's approval 08:54): only in the stopping state below V12K9_ZONE_SPEED with no lead within
# V12K9_ZONE_LEAD may kind 8 send a weaker request / move StopReq against the reference kind 8 (_V12K9Ref); it is never more
# than V12K9_ZONE_PRESS_V (0.2 above 0.6 m/s, 0.4 below 0.3 m/s) stronger there, and outside the zone it stays within [reference - V12K9_BAND_COMP, reference]
V12K9_ZONE_SPEED, V12K9_ZONE_LEAD = 1.5, 1.5  # m/s, m
V12K9_ZONE_PRESS_BP, V12K9_ZONE_PRESS_V = [0.3, 0.6], [0.4, 0.2]  # m/s, m/s^2: 0.4 below 0.3 m/s (crawl), 0.2 above 0.6 m/s
# (brake_0241 MX, sim only) inside the zone the press limit is measured from the reference plus the band compensation (ref_c);
# outside the zone below V12K9_OFF_SPEED the band compensation share is w = clip((s_meas - s_target - V12K9_OFF_DD0) / V12K9_OFF_DDW, 0, 1),
# s_meas = v^2 / (2 max(-low-passed eff_ac, 0.05)), s_target = v^2 / (2 max(-aTarget, 0.05)): the request is in [ref - w (ref - ref_c), ref]
V12K9_OFF_SPEED, V12K9_OFF_DD0, V12K9_OFF_DDW, V12K9_OFF_ALPHA = 1.0, 0.0, 0.05, 0.03  # m/s, m, m, per frame (k2: 0.9 -> 1.0)
# (brake_0241 k2) the removal of the band compensation fades in over V12K9_OFF_FADE below V12K9_OFF_SPEED:
# w = max(w, clip((v - (V12K9_OFF_SPEED - V12K9_OFF_FADE)) / V12K9_OFF_FADE, 0, 1)) (no step at V12K9_OFF_SPEED)
V12K9_OFF_FADE = 0.15  # m/s
# (brake_0241 MX) stopping, lead at or within V12K9_MX_GAP and V12K9_MX_SPEED_LO..HI: StopReq from the next frame (where the
# reference latches on its 3.0 m lead latch); the end-profile lead gain is V12K9_END_LEAD_KB between V12K9_END_LEAD_BAND
V12K9_MX_GAP, V12K9_MX_SPEED_LO, V12K9_MX_SPEED_HI = 3.0, 0.5, 1.0  # m, m/s, m/s
V12K9_END_LEAD_KB, V12K9_END_LEAD_BAND = 3.0, (1.5, 3.5)  # -, m
# v12k9 brake-only slow trim (brake_0241): band on, below TRIM_SPEED, decelerating pid (aTarget < -0.05) or stopping before the
# latch and the end profile: the request gets trim = integral of TRIM_KI * (request - low-passed eff_ac), learned at or above
# TRIM_LEARN_SPEED, clipped to -TRIM_MAX .. 0 (only ever more braking); it decays 0.95 per frame while off
V12K9_TRIM_KI, V12K9_TRIM_ALPHA, V12K9_TRIM_MAX = 2.0, 0.1, 0.4  # 1/s, per frame, m/s^2
V12K9_TRIM_SPEED, V12K9_TRIM_LEARN_SPEED = 3.0, 0.3  # m/s


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


class _V12K9Ref:
  """Reference kind 8 for the zone clamp: the 66ca01d5 kind 8 without the band (band_w 0: no band compensation, no stall
  latch), i.e. the 6f518e35 menu 2 command; the K9 creep, v12 pid rate limit, page v11 feather and latches as there."""
  def __init__(self):
    self.reset()

  def reset(self):
    self.latched = False
    self.last = None
    self.ss_count = 0

  def update(self, accel, a_target, stopping, pid, v_ego, v_ego_raw, a_ego, long_accel, esp_standstill, long_active, override,
             gas, brake, can_valid, lead_visible, lead_distance):
    values = (accel, a_target, v_ego, v_ego_raw, a_ego, lead_distance)
    if (not all(math.isfinite(x) for x in values) or not long_active or override or gas or brake or not can_valid):
      self.reset()
      return accel, stopping
    lon_ac = long_accel if long_accel is not None and math.isfinite(long_accel) else a_ego
    eff_ac = min(lon_ac, a_ego + 0.10)
    lead_seen = lead_visible or lead_distance != 0.0
    lead_near = lead_seen and lead_distance <= LEAD_LATCH_GAP
    if not stopping:
      self.latched = False
      self.ss_count = 0
      if pid and v_ego < CREEP_BP[-1] and a_target < FADE_TARGET_BP[-1]:
        comp = np.interp(v_ego, CREEP_BP, CREEP_V) * np.interp(a_target, FADE_TARGET_BP, FADE_TARGET_V)
        comp *= np.interp(accel, FADE_ACCEL_BP, FADE_ACCEL_V)
        accel = float(min(accel, max(accel + comp, CarControllerParams.ACCEL_MIN)))
        prev = accel if self.last is None else self.last
        if v_ego < V12_RATE_SPEED and a_target > V12_RATE_MIN_TARGET:
          if (v_ego < V12K9_FALL_GUARD_SPEED and eff_ac < V12K9_FALL_GUARD_EFF_AC and not lead_near):
            down = 0.0 if eff_ac < min(a_target - V12_DOWN_HOLD_MARGIN, V12_DOWN_HOLD_EFF_AC) else V12_DOWN_LIM
            accel = float(max(accel, prev - down))
          if v_ego > V12_UP_LIM_SPEED and eff_ac >= a_target - V12K9_UP_LIM_MARGIN:
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


class K9Creep:
  def __init__(self, mode: int = V9):
    assert mode in KINDS, mode
    self.mode = mode
    self.v12 = mode in (V12K9, V12K9_C1, V12K9_C1P)  # kinds 8 / 9 / 10: v12k9 (own update path _update_v12k9, jerk_u / jerk_l profile; band below 3.0 m/s)
    self.c1 = mode in (V12K9_C1, V12K9_C1P)  # kinds 9 / 10 (menus 4 / 5): the C1 stop end
    self.c1p = mode == V12K9_C1P  # kind 10 (menu 5): C1+ (lead filter, hand-over, lead gate)
    if self.c1p:
      self.lead_f = (False, 0.0)  # kind 10: the lead of the last update() as kind 10 used it (after filter_lead)
    self.ref = _V12K9Ref() if self.v12 else None  # kinds 8-10: reference for the zone clamp
    self.reset()

  @property
  def last(self):
    """Previous kind-8 / kind-1 command before the band compensation (feather / rate-limit state); None = start from the
    current command."""
    return self._last

  @last.setter
  def last(self, value):
    # set from outside (test harness: "the previous sent command was value"): the zone reference starts from it too
    self._last = value
    if self.ref is not None:
      self.ref.last = value

  def reset(self):
    if self.ref is not None:
      self.ref.reset()
    self.latched = False
    self._last = None  # previous sent accel; None means "start from the current command"
    self.active = False  # True only on a frame where update() applied (jerk/band change follows it)
    self.ss_count = 0  # kind 8: consecutive stopping frames with standstill
    self.band_w = 0.0  # kind 8: share of the original ComfortBand sent on the last bands() call (0..1)
    self.eff_f = None  # kind 8: low-passed eff_ac
    self.end_on = False  # kind 8: end profile running in this stop
    self.end_ae = None  # kind 8: low-passed aEgo
    self.end_off = 0.0  # kind 8: end profile offset (request - aEgo)
    self.end_out = None  # kind 8: last sent accel
    self.end_n = 0  # kind 8: end-profile frames below V12K9_END_TMAX_SPEED
    self.end_fb = 0  # kind 8: crawl frames
    self.end_k = 0  # kind 8: end-profile frames since its start
    self.end_sref, self.end_s = 0.0, 0.0  # kind 8: end-profile virtual stop line (m ahead of the start) and distance since the start
    self.trim = 0.0  # kind 8: brake-only slow trim
    self.trim_lp = None  # kind 8: low-passed eff_ac for the trim
    self.off_lp = None  # kind 8: low-passed eff_ac for the band compensation share outside the zone (V12K9_OFF_ALPHA)
    if self.c1:
      self.end_ref = None  # kinds 9 / 10: end-profile target through V12K9_C1_END_REF_TAU
      self.end_bl = False  # kinds 9 / 10: bumpless start pending
    if self.c1p:
      self.sg_on = False  # kind 10: predicted-gap gate on (V12K9_C1P_SG_REACH)
      self.lf_d, self.lf_n, self.lf_s = None, 0, 0.0  # kind 10: filter_lead() state (last positive distance, frames since, travel since)

  def filter_lead(self, lead_visible: bool, lead_distance: float, v_ego: float):
    """Kind 10 (C1+): the HUD lead (leadVisible, leadDistance) as kind 10 uses it (see V12K9_C1P_LF_HOLD); other kinds: unchanged."""
    if not self.c1p:
      return lead_visible, lead_distance
    if not (math.isfinite(lead_distance) and math.isfinite(v_ego)):  # the interlock in update() resets on it as before
      self.lf_d, self.lf_n, self.lf_s = None, 0, 0.0
      return lead_visible, lead_distance
    if lead_distance > 0.0:  # a radar distance: as sent
      self.lf_d, self.lf_n, self.lf_s = float(lead_distance), 0, 0.0
      return lead_visible, lead_distance
    if not (lead_visible and lead_distance == 0.0) or self.lf_d is None:  # no lead (or a negative distance): as sent
      self.lf_d, self.lf_n, self.lf_s = None, 0, 0.0
      return lead_visible, lead_distance
    self.lf_n += 1  # visible without a distance right after a radar distance
    self.lf_s += max(v_ego, 0.0) * 0.01
    d = self.lf_d - self.lf_s
    if self.lf_n <= V12K9_C1P_LF_HOLD and d > V12K9_C1P_LF_FAR:
      return True, float(d)
    return lead_visible, lead_distance

  def update(self, *, accel: float, a_target: float, stopping: bool, pid: bool, v_ego: float, v_ego_raw: float,
             a_ego: float, long_accel, esp_standstill: bool, long_active: bool, override: bool, gas: bool,
             brake: bool, can_valid: bool, lead_visible: bool, lead_distance: float):
    """Returns (accel, stopping) to send."""
    ref = None
    if self.ref is not None:  # kind 8: the reference runs on every frame (its own interlock reset), on the raw lead
      ref = self.ref.update(accel, a_target, stopping, pid, v_ego, v_ego_raw, a_ego, long_accel, esp_standstill, long_active,
                            override, gas, brake, can_valid, lead_visible, lead_distance)[0]
    if self.c1p:  # kind 10: the HUD lead as kind 10 itself uses it (zone, latches, end profile)
      lead_visible, lead_distance = self.filter_lead(lead_visible, lead_distance, v_ego)
      self.lead_f = (lead_visible, lead_distance)  # kind 10: the lead of this update() after filter_lead
    self.ref_out = ref  # kind 8: reference command of this frame (None for kind 1)
    values = (accel, a_target, v_ego, v_ego_raw, a_ego, lead_distance)
    if (not all(math.isfinite(x) for x in values) or not long_active or override or gas or brake or not can_valid):
      self.reset()
      return accel, stopping
    self.active = True
    lon_ac = long_accel if long_accel is not None and math.isfinite(long_accel) else a_ego
    eff_ac = min(lon_ac, a_ego + 0.10)
    if self.v12:
      self.eff_f = eff_ac if self.eff_f is None else self.eff_f + V12K9_STALL_ALPHA * (eff_ac - self.eff_f)
      stopping_in = stopping
      accel, stopping = self._update_v12k9(accel, a_target, stopping, pid, v_ego, v_ego_raw, a_ego, eff_ac, esp_standstill,
                                           lead_visible, lead_distance)
      if (stopping_in and not self.latched and (lead_visible or lead_distance != 0.0) and 0.0 < lead_distance <= V12K9_MX_GAP
          and V12K9_MX_SPEED_LO <= v_ego <= V12K9_MX_SPEED_HI):
        self.latched = True  # MX: StopReq from the next frame (no end profile on this frame)
      end = self._end_profile(accel, a_target, stopping_in, stopping, v_ego, a_ego, esp_standstill, lead_visible, lead_distance)
      if end is not None:
        accel = end
      else:
        acc0 = accel
        if self.band_w > 0.0 and accel < 0.0 and v_ego < V12K9_BAND_ZERO_SPEED:  # K9 band compensation (self._last uncompensated)
          accel = float(accel - V12K9_BAND_COMP * self.band_w * min(1.0, -accel / V12K9_BAND_COMP_FADE))
        self.trim_lp = eff_ac if self.trim_lp is None else self.trim_lp + V12K9_TRIM_ALPHA * (eff_ac - self.trim_lp)
        on = (self.band_w > 0.0 and acc0 < 0.0 and v_ego < V12K9_TRIM_SPEED and not self.latched and not esp_standstill
              and (stopping_in or (pid and a_target < -0.05)))
        if on:
          if v_ego >= V12K9_TRIM_LEARN_SPEED:
            self.trim = float(np.clip(self.trim + V12K9_TRIM_KI * (acc0 - self.trim_lp) * 0.01, -V12K9_TRIM_MAX, 0.0))
          accel = float(accel + self.trim)
        else:
          self.trim *= 0.95
      if ref is not None and math.isfinite(ref):  # zone clamp against the reference kind 8
        ref_c = ref  # reference plus the band compensation of this frame
        if self.band_w > 0.0 and ref < 0.0 and v_ego < V12K9_BAND_ZERO_SPEED:
          ref_c = float(ref - V12K9_BAND_COMP * self.band_w * min(1.0, -ref / V12K9_BAND_COMP_FADE))
        self.off_lp = eff_ac if self.off_lp is None else self.off_lp + V12K9_OFF_ALPHA * (eff_ac - self.off_lp)
        lead_in = (lead_visible or lead_distance != 0.0) and lead_distance <= V12K9_ZONE_LEAD
        if not (stopping_in and v_ego < V12K9_ZONE_SPEED and not lead_in):
          if v_ego < V12K9_OFF_SPEED:
            dd = v_ego ** 2 / (2.0 * max(-self.off_lp, 0.05)) - v_ego ** 2 / (2.0 * max(-a_target, 0.05))
            w = float(min(max((dd - V12K9_OFF_DD0) / V12K9_OFF_DDW, 0.0), 1.0))
            w = float(max(w, min(max((v_ego - (V12K9_OFF_SPEED - V12K9_OFF_FADE)) / V12K9_OFF_FADE, 0.0), 1.0)))
            accel = float(min(max(accel, ref - w * (ref - ref_c)), ref))
          else:
            accel = float(min(max(accel, ref - V12K9_BAND_COMP), ref))
        else:
          accel = float(max(accel, ref_c - float(np.interp(v_ego, V12K9_ZONE_PRESS_BP, V12K9_ZONE_PRESS_V))))
          if (self.c1p and V12K9_C1P_END_HANDOFF and end is None and self.end_on and self.end_out is not None
              and a_target > URGENT_TARGET and math.isfinite(accel)
              and not ((lead_visible or lead_distance != 0.0) and lead_distance < V12K9_END_LEAD_MAX)):
            accel = float(max(accel, self.end_out - V12K9_HOLD_STEP))  # kind 10: end-profile hand-over, no step
      self.end_out = accel
      return accel, stopping

    if not stopping:
      self.latched = False
      if pid and v_ego < CREEP_BP[-1] and a_target < FADE_TARGET_BP[-1]:
        comp = np.interp(v_ego, CREEP_BP, CREEP_V) * np.interp(a_target, FADE_TARGET_BP, FADE_TARGET_V)
        comp *= np.interp(accel, FADE_ACCEL_BP, FADE_ACCEL_V)
        accel = float(min(accel, max(accel + comp, CarControllerParams.ACCEL_MIN)))  # comp <= 0
      self._last = float(accel)
      return accel, stopping

    lead_seen = lead_visible or lead_distance != 0.0
    lead_near = lead_seen and lead_distance <= LEAD_LATCH_GAP
    real_standstill = esp_standstill or v_ego_raw < STANDSTILL_SPEED or v_ego < STANDSTILL_SPEED
    if (real_standstill or a_target <= URGENT_TARGET or (v_ego > DOWN_UP_SPEED and eff_ac > 0.04) or a_ego > 0.06
        or lead_near):
      self.latched = True

    if self.latched:
      accel = float(min(accel, LATCH_ACCEL))
      self._last = accel
      return accel, True

    margin = eff_ac - float(np.interp(v_ego, MIN_DECEL_BP, MIN_DECEL_V))
    target = float(np.interp(v_ego, FEATHER_BP, FEATHER_V)) + float(np.interp(margin, BOOST_BP, BOOST_V))
    prev = accel if self._last is None else self._last
    down = DOWN_STEP if (v_ego > DOWN_UP_SPEED and eff_ac > DOWN_EFF_AC) else 0.0
    up = _up_step(v_ego, margin, UP_LOW_SPEED)
    accel = float(min(max(target, prev - down), prev + up))
    self._last = accel  # v9 feather state
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
        prev = accel if self._last is None else self._last
        if v_ego < V12_RATE_SPEED and a_target > V12_RATE_MIN_TARGET:  # page v12 two-way rate limit
          # K9 (2026-10-25 brake_v12k9_fix): the fall limit only where the old v11 re-press guard held (below 0.95 m/s,
          # eff_ac < -0.18, no lead within 3.0 m), so v12k9 never brakes later than v11 (min of both commands)
          if (v_ego < V12K9_FALL_GUARD_SPEED and eff_ac < V12K9_FALL_GUARD_EFF_AC and not lead_near):
            down = 0.0 if eff_ac < min(a_target - V12_DOWN_HOLD_MARGIN, V12_DOWN_HOLD_EFF_AC) else V12_DOWN_LIM
            accel = float(max(accel, prev - down))
          # K9 (brake_v12k9_fix): the release limit only while the car is not braking harder than planned
          if v_ego > V12_UP_LIM_SPEED and eff_ac >= a_target - V12K9_UP_LIM_MARGIN:
            accel = float(min(accel, prev + float(np.interp(eff_ac - a_target, V12_UP_LIM_BP, V12_UP_LIM_V))))
      self._last = float(accel)
      return accel, stopping

    raw_standstill = esp_standstill or v_ego_raw < STANDSTILL_SPEED or v_ego < STANDSTILL_SPEED
    self.ss_count = self.ss_count + 1 if raw_standstill else 0
    real_standstill = self.ss_count >= V12K9_STANDSTILL_FRAMES
    weak = (v_ego > DOWN_UP_SPEED and eff_ac > 0.04) or a_ego > 0.06  # brake_0241: no StopReq for it (pressed below)
    if real_standstill or a_target <= URGENT_TARGET or (lead_seen and lead_distance <= V12K9_LEAD_HARD_GAP):
      self.latched = True
    prev = accel if self._last is None else self._last
    if self.latched:
      if a_target > URGENT_TARGET:
        accel = float(max(min(accel, LATCH_ACCEL), prev - V12K9_HOLD_STEP))
      else:
        accel = float(min(accel, LATCH_ACCEL))
      self._last = accel
      return accel, True
    margin = eff_ac - float(np.interp(v_ego, V12K9_MIN_DECEL_BP, V12K9_MIN_DECEL_V))
    feather = float(np.interp(v_ego, V12K9_FEATHER_BP, V12K9_FEATHER_V))
    target = feather + float(np.interp(margin, BOOST_BP, BOOST_V))
    down = DOWN_STEP if ((v_ego > DOWN_UP_SPEED and eff_ac > DOWN_EFF_AC) or prev > feather + V12K9_DOWN_ABOVE) else 0.0
    up = float(np.interp(margin, UP_MARGIN_BP, UP_MARGIN_V)) if v_ego > V12K9_UP_LOW_SPEED else V12K9_UP_LOW
    if prev < feather - V12K9_UP_FAST_GAP and v_ego < V12K9_UP_FAST_SPEED:
      up = max(up, V12K9_UP_FAST)
    accel = float(min(max(target, prev - down), prev + up))
    if weak:
      accel = float(min(accel, max(prev - V12K9_PRESS, V12K9_PRESS_MIN)))
    if v_ego < V12K9_CRAWL_SPEED and self.eff_f is not None and self.eff_f > V12K9_CRAWL_EFF_AC:
      accel = float(min(accel, max(prev - V12K9_CRAWL_STEP, V12K9_CRAWL_MIN)))
    if lead_near:
      a_need = -(v_ego ** 2) / (2.0 * max(lead_distance - V12K9_LEAD_MARGIN, V12K9_LEAD_NEED_MIN)) - 0.0
      if a_need < accel:
        accel = float(min(accel, max(a_need, prev - DOWN_STEP)))
    self._last = accel
    return accel, False

  def _end_profile(self, accel, a_target, stopping_in, stopping, v_ego, a_ego, esp_standstill, lead_visible, lead_distance):
    """Kind 8 stop end profile (brake_0241): the accel to send instead of accel (the feather / rate-limit output), or None."""
    self.end_ae = a_ego if self.end_ae is None else self.end_ae + V12K9_END_ALPHA * (a_ego - self.end_ae)
    if not stopping_in or self.latched or esp_standstill or stopping:
      if not stopping_in:
        self.end_on, self.end_ae, self.end_n, self.end_fb, self.end_k = False, None, 0, 0, 0
        if self.c1:
          self.end_ref = None
      return None
    dec = max(-self.end_ae, V12K9_END_DEC_MIN)
    vp = max(v_ego - dec * V12K9_END_LAG - V12K9_END_VS, 0.0)
    d = math.sqrt(V12K9_END_DECEL ** 2 + 2.0 * V12K9_END_JERK * vp)
    d_prof, d_lead = d, 0.0
    lead_seen = lead_visible or lead_distance != 0.0
    if lead_seen and 0.0 < lead_distance < V12K9_END_LEAD_MAX:
      k_l = V12K9_END_LEAD_KB if V12K9_END_LEAD_BAND[0] <= lead_distance <= V12K9_END_LEAD_BAND[1] else V12K9_END_LEAD_K
      gap = V12K9_C1P_END_LEAD_GAP if self.c1p else V12K9_END_LEAD_GAP
      d_lead = min(k_l * v_ego ** 2 / (2.0 * max(lead_distance - gap, V12K9_END_LEAD_SMIN)), V12K9_END_LEAD_CAP)
      d = max(d, d_lead)
    if not self.end_on and (d <= dec or v_ego < V12K9_END_SPEED) and v_ego < V12K9_END_MAX_SPEED:
      self.end_on = True
      self.end_off = float((accel if self.end_out is None else self.end_out) - self.end_ae)
      self.end_k, self.end_sref, self.end_s = 0, v_ego ** 2 / (2.0 * V12K9_END_LINE_DECEL), 0.0
      if self.c1:
        self.end_ref, self.end_bl = self.end_ae, True
      if self.c1p:
        self.sg_on = False
    if not self.end_on:
      return None
    self.end_k += 1
    self.end_n += 1 if v_ego < V12K9_END_TMAX_SPEED else 0
    self.end_fb = self.end_fb + 1 if (v_ego < V12K9_END_CRAWL_SPEED and self.end_ae > V12K9_END_CRAWL_AEGO) else 0
    go_speed = V12K9_C1_END_GO_SPEED if self.c1 else V12K9_END_GO_SPEED
    if (self.end_fb >= V12K9_END_CRAWL_FRAMES or (self.end_n > V12K9_END_TMAX * 100.0 and v_ego > 0.02)
        or (a_target > V12K9_END_GO_TARGET and v_ego < go_speed)):
      self.latched = True  # StopReq from the next frame (the band ramps out as on any latch)
    self.end_s += v_ego * 0.01
    d_line = min(V12K9_END_LINE_K * v_ego ** 2 / (2.0 * max(self.end_sref - self.end_s, V12K9_END_LINE_SMIN)), V12K9_END_LINE_CAP)
    d = max(d, d_line)
    cmd = -d
    if self.c1:
      off0 = self._end_c1_integral(accel, cmd, v_ego, lead_seen, lead_distance, d_lead > d_prof)
    else:
      off0 = self.end_off
      self.end_off = float(np.clip(self.end_off - V12K9_END_KI * 0.01 * (self.end_ae - cmd), V12K9_END_OFF_MIN, V12K9_END_OFF_MAX))
    if self.end_k <= V12K9_END_HOLD_T * 100.0 and self.end_off > off0:  # no release right after the start
      self.end_off = off0
    cmd = float(cmd + self.end_off)
    if self.end_out is not None and cmd > self.end_out + V12K9_END_REL:  # release rate limit, integral held
      cmd, self.end_off = float(self.end_out + V12K9_END_REL), off0
    cmd = float(cmd + 0.0)
    if lead_seen and lead_distance <= LEAD_LATCH_GAP:
      cmd = float(min(cmd, -(v_ego ** 2) / (2.0 * max(lead_distance - V12K9_LEAD_MARGIN, V12K9_LEAD_NEED_MIN)) - 0.0))
    return float(max(cmd, -3.5))

  def _end_c1_integral(self, accel, cmd, v_ego, lead_seen, lead_distance, safe):
    """Kinds 9 / 10 (C1 / C1+): end-profile offset update for the target cmd (-d); returns the offset before it (off0).
    safe: the lead term of the end profile asks more than the profile itself (kind 10 only uses it)."""
    if self.end_bl:  # bumpless start: the request continues from the last sent one
      self.end_bl = False
      self.end_off = float(np.clip((accel if self.end_out is None else self.end_out) - cmd, V12K9_END_OFF_MIN, V12K9_END_OFF_MAX))
    off0 = self.end_off
    self.end_ref = float(self.end_ref + 0.01 / (V12K9_C1_END_REF_TAU + 0.01) * (cmd - self.end_ref))
    err = self.end_ae - self.end_ref
    sg = False
    if self.c1p:  # kind 10: predicted-gap gate
      d_need = 0.0
      if V12K9_C1P_SG_GAP > 0.0 and lead_seen and 0.0 < lead_distance < V12K9_END_LEAD_MAX:
        left = lead_distance - V12K9_C1P_SG_GAP
        over = v_ego * V12K9_C1P_SG_LAG + v_ego ** 2 / (2.0 * max(-self.end_ae, 0.05)) > left
        if V12K9_C1P_SG_REACH:
          if over and left - v_ego * V12K9_C1P_SG_LAG > V12K9_C1P_SG_SMIN:
            self.sg_on = True
          over = self.sg_on
        if over:
          d_need = min(V12K9_C1P_SG_K * v_ego ** 2 / (2.0 * max(left - v_ego * V12K9_C1P_SG_LAG, V12K9_C1P_SG_SMIN)), V12K9_END_LEAD_CAP)
      sg = d_need > 0.0
      if sg:
        err = max(err, self.end_ae + d_need)
    if err > 0.0 and v_ego < V12K9_C1_END_PRE_T * max(-self.end_ae, 0.05):
      err = 0.0  # about to stand: no more pressing
    ki = V12K9_C1_END_KI
    if self.c1p and V12K9_C1P_END_KI_SAFE > 0.0 and (safe or sg) and err > 0.0:
      ki = V12K9_C1P_END_KI_SAFE
    self.end_off = float(np.clip(self.end_off - ki * 0.01 * err, V12K9_END_OFF_MIN, V12K9_END_OFF_MAX))
    return off0

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
    this frame, cb_prev / cb_prev_lower the band sent on the previous make_jerk call (kind 8: the latch ramp; cb_prev_lower
    None = cb_prev - 0.1)."""
    if self.v12:  # kind 8: page v12 jerk_u profile below 1.15 m/s; jerk_l: jerk_l_floor()
      if v_ego < V12_JERK_U_SPEED:
        jerk_u = max(jerk_u, float(np.interp(v_ego, V12_JERK_U_BP, V12_JERK_U_V)))
      if not v_ego < V12K9_BAND_ZERO_SPEED:  # as the page: band 0 (bit for bit the old kind 8 here; NaN too)
        self.band_w = 0.0
        return jerk_u, 0.0, 0.0
      # K9 (brake_v12k9_live_fix): the original band below 3.0 m/s (full at or below 2.5 m/s) against the 6-7 Hz shudder
      w = float(np.interp(v_ego, [V12K9_BAND_FULL_SPEED, V12K9_BAND_ZERO_SPEED], [1.0, 0.0]))
      cu, cl = float(cb_upper) * w, float(cb_lower) * w
      if self.latched:  # StopReq latched: back to band 0 as before, ramped down V12K9_BAND_LATCH_STEP per call
        pu = float(cb_prev)
        pl = max(pu - 0.1, 0.0) if cb_prev_lower is None else float(cb_prev_lower)
        cu, cl = max(0.0, min(cu, pu - V12K9_BAND_LATCH_STEP)), max(0.0, min(cl, pl - V12K9_BAND_LATCH_STEP))
      self.band_w = float(min(1.0, max(0.0, cu / float(cb_upper)))) if cb_upper > 0.0 else 0.0
      return jerk_u, cu, cl
    return K9Creep.jerk_bands(jerk_u, v_ego)  # kind 1: exactly the v9 band (0)
