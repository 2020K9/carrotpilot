"""SR / SAD text above the external HUD TPMS car (보노보노 2026-10-24 23:39).

Shows the steer ratio and the steer actuator delay that lateral control actually
uses, worked out the same way the control processes do. Display only: nothing
here is read back by controls.

- SR: controlsd.state_control -> VM.update_params(x, resolve_vehicle_model_steer_ratio(
  lp.steerRatio, SteerRatioRate, CustomSR)). CustomSR*0.1 > 1 wins (fixed), otherwise
  the learned liveParameters.steerRatio scaled by SteerRatioRate %. While liveParameters
  is not usable yet controlsd uses NominalLateralParameters(CP.steerRatio).
- SAD: modeld (laneless) uses SteerActuatorDelay*0.01 when > 0, otherwise
  liveDelay.lateralDelay (learned); controlsd lane mode does the same (== 0 -> liveDelay).

Lane line above it (보노보노 2026-10-25 20:16 "라이브값 따로 못보나, 좌우치우침"; the first 'OFS / LC' text of 7ba6432e is
replaced by the Korean line below since 6fc96445):
- offset: car (comma device, modelV2 y = 0) from the lane centre [m] (+ right, - left), modelV2 inner lane lines at x = 0,
  the same measurement K9LaneAssist's centre hold uses ( -(y1 + y2) / 2 ), with a 0.5 s low-pass. None ('--') while either
  inner line is weak (prob < LANE_P_MIN, std > LANE_STD_MAX) or the width is implausible (outside LANE_W_MIN..LANE_W_MAX).
- state: controlsState.k9LaneState from controlsd (0 switch off, 1 on but not holding, 2 / 3 holding). controlsState.k9LaneAccel
  (lateral accel the hold adds, m/s^2, + = toward the right) is logged but not shown.

2026-10-25 23:49 (보노보노 "hud는 수정 다 했니"): one Korean line, shown whatever K9LaneAssist is set to:
  '치우침 우 12cm · 좌 1.87m 우 1.63m · 보정 켬'
  치우침 = offset above (좌 / 우 + cm, '가운데' under 2 cm, '--' while the lines are weak); 보정 = 꺼짐 (switch 0) / 대기 (on, not
  pushing) / 켬 (pushing now).
  좌 / 우 (보노보노 2026-10-26 01:20 "콤마 기준"): distance from the car centre (comma device, modelV2 y = 0) to the middle of that
  inner lane line at x = 0, i.e. -y1 / y2 ('--' for a weak line, or outside GAP_MIN_M..GAP_MAX_M). So 좌 + 우 = lane width and
  (좌 - 우) / 2 = 치우침 to the right (before the 0.5 s low-pass): 우 12cm -> 좌 is 24 cm more than 우. Until then it was the gap
  from the car side (minus K9 half width 0.9575 m).

2026-10-26 (보노보노 2026-10-25 20:28 / 20:38 / 20:43 / 20:53, branch k9-hud-ld-lt): five Korean lines above the TPMS car, all
labels in Korean, no 적용 / 미적용 text, no 보정 state:
  1 '조향비 12.90  지연설정 0.20초'         (SR / SAD as above)
  2 '지연학습 63%  0.24초'                  (liveDelay.calPerc, learned lag: lateralDelayEstimate while calPerc < 100, lateralDelay at 100)
  3 '토크학습 61%  횡가속 3.16  마찰 0.09'  (liveTorqueParameters.calPerc, latAccelFactor / frictionCoefficient: Raw while calPerc < 100,
                                          Filtered at 100; '-' while nothing is learned yet: calPerc 0 and raw 0)
  4 '치우침 오른쪽 12cm'                     (offset above, 오른쪽 / 왼쪽, '가운데' under 2 cm, '--' weak lines)
  5 '좌 차선 1.87m   우 차선 1.63m'          (comma-centre gaps above, unchanged since 98c9e407)
  Lines 2 / 3 show the learners' current numbers whether or not lateral control uses them (SteerActuatorDelay > 0 and
  LateralTorqueCustom 1 mean the fixed values are used); the HUD does not say which.

2026-10-26 17:24 (보노보노 "외부uhd에 넣은거 콤마화면에서는 빼라", 17:23 "횡가속은 3.3~4 왔다갔다하는데 뭘 집어넣어 놓은 거니?"):
  - the comma (mici) screen no longer shows the two K9 lane lines (6fc96445); the external HUD keeps all five lines.
  - line 3 numbers = what latcontrol_torque applies (applied_torque), never Raw: LateralTorqueCustom > 0 -> the menu values
    LateralTorqueAccelFactor / LateralTorqueFriction x 0.001, else liveTorqueParameters Filtered (CarParams values until learned).
    Raw is the instant fit of whatever points torqued has (rlog 1ca / 1cc at calPerc 14..61 %: 2.78..3.48, while the full fits
    of 1c8 / 1c9 at 100 % were 2.59..2.97 with Filtered 2.62..2.75), it jumps every update and lateral control never uses it.
    The % stays liveTorqueParameters.calPerc.
"""
from __future__ import annotations

import math
from typing import Any

try:
    from openpilot.selfdrive.controls.lib.steer_ratio import resolve_vehicle_model_steer_ratio
except Exception:  # pragma: no cover - standalone HUD tools without the openpilot tree
    resolve_vehicle_model_steer_ratio = None


def _finite(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def live_parameters_usable(lp: Any | None) -> bool:
    """controls/lib/lateral_readiness.live_parameters_ready without the SubMaster checks."""
    if lp is None:
        return False
    try:
        if not (lp.valid and lp.sensorValid and lp.posenetValid):
            return False
        values = (float(lp.steerRatio), float(lp.stiffnessFactor), float(lp.angleOffsetDeg), float(lp.roll))
    except Exception:
        return False
    return all(math.isfinite(v) for v in values) and values[0] > 1.0 and values[1] > 0.0


def applied_steer_ratio(live_steer_ratio: Any, live_ok: bool, nominal_steer_ratio: Any,
                        steer_ratio_rate_param: Any, custom_sr_param: Any) -> float | None:
    """Steer ratio fed to the vehicle model. Params are the raw stored numbers
    (CustomSR in 0.1 units, SteerRatioRate in %); missing params read as 0 like Params.get_float."""
    if resolve_vehicle_model_steer_ratio is None:
        return None
    custom = _finite(custom_sr_param)
    custom = 0.0 if custom is None else custom
    rate = _finite(steer_ratio_rate_param)
    rate = 0.0 if rate is None else rate
    base = _finite(live_steer_ratio) if live_ok else _finite(nominal_steer_ratio)
    if base is None:
        if not custom * 0.1 > 1.0:
            return None
        base = 0.0
    return _finite(resolve_vehicle_model_steer_ratio(base, rate, custom))


def applied_steer_delay_s(steer_actuator_delay_param: Any, live_lateral_delay: Any) -> float | None:
    """Steer actuator delay in seconds (SteerActuatorDelay param in 0.01 s)."""
    raw = _finite(steer_actuator_delay_param)
    fixed = 0.0 if raw is None else raw * 0.01
    if fixed > 0.0:
        return fixed
    return _finite(live_lateral_delay)


def _fmt(value: Any, spec: str, unit: str = "") -> str:
    v = _finite(value)
    return "-" if v is None else f"{v:{spec}}{unit}"


def lat_values_text(steer_ratio: float | None, steer_delay_s: float | None) -> str:
    """HUD line 1: '조향비 12.90  지연설정 0.20초'."""
    return f"조향비 {_fmt(steer_ratio, '.2f')}  지연설정 {_fmt(steer_delay_s, '.2f', '초')}"


def _perc(value: Any) -> int | None:
    v = _finite(value)
    return None if v is None else int(round(min(max(v, 0.0), 100.0)))


def learned_delay(live_delay: Any | None) -> tuple[int | None, float | None]:
    """(calPerc %, learned lag s) of liveDelay: lateralDelayEstimate while learning (< 100 %), lateralDelay once done."""
    if live_delay is None:
        return None, None
    try:
        perc = _perc(live_delay.calPerc)
        value = live_delay.lateralDelay if perc == 100 else live_delay.lateralDelayEstimate
    except Exception:
        return None, None
    value = _finite(value)
    return perc, value if value is not None and value > 0.0 else None


def learned_torque(live_torque: Any | None) -> tuple[int | None, float | None, float | None]:
    """(calPerc %, latAccelFactor, friction) of liveTorqueParameters: Raw while learning (< 100 %), Filtered once done;
    None for both values while nothing is learned yet (calPerc 0 with raw 0)."""
    if live_torque is None:
        return None, None, None
    try:
        perc = _perc(live_torque.calPerc)
        if perc == 100:
            af, fr = live_torque.latAccelFactorFiltered, live_torque.frictionCoefficientFiltered
        else:
            af, fr = live_torque.latAccelFactorRaw, live_torque.frictionCoefficientRaw
    except Exception:
        return None, None, None
    af, fr = _finite(af), _finite(fr)
    if perc in (0, None) and not af and not fr:
        return perc, None, None
    return perc, af, fr


def _custom_on(custom: Any) -> bool | None:
    c = _finite(custom)
    return None if c is None else c > 0


def applied_torque(live_torque: Any | None, custom: Any = None, custom_laf: Any = None,
                   custom_fr: Any = None) -> tuple[int | None, float | None, float | None]:
    """(calPerc %, latAccelFactor, friction) for HUD line 3 = the values latcontrol_torque applies:
    LateralTorqueCustom > 0 -> LateralTorqueAccelFactor / LateralTorqueFriction x 0.001 (menu values, set every 10 frames),
    LateralTorqueCustom 0 -> liveTorqueParameters Filtered (controlsd update_live_torque_params; the CarParams values until
    torqued has learned). Raw is never shown. Values None ('-') when unknown (menu not read yet, no message)."""
    perc = None
    if live_torque is not None:
        try:
            perc = _perc(live_torque.calPerc)
        except Exception:
            perc = None
    on = _custom_on(custom)
    if on is None:
        return perc, None, None
    if on:
        af, fr = _finite(custom_laf), _finite(custom_fr)
        return (perc, af * 0.001 if af is not None and af > 0.0 else None,
                fr * 0.001 if fr is not None and fr >= 0.0 else None)
    if live_torque is None:
        return perc, None, None
    try:
        af, fr = _finite(live_torque.latAccelFactorFiltered), _finite(live_torque.frictionCoefficientFiltered)
    except Exception:
        return perc, None, None
    return perc, af if af is not None and af > 0.0 else None, fr if fr is not None and fr >= 0.0 else None


def delay_learn_text(perc: int | None, delay_s: float | None) -> str:
    """HUD line 2: '지연학습 63%  0.24초'."""
    return f"지연학습 {'-' if perc is None else f'{perc}%'}  {_fmt(delay_s, '.2f', '초')}"


def torque_learn_text(perc: int | None, lat_accel_factor: float | None, friction: float | None) -> str:
    """HUD line 3: '토크학습 61%  횡가속 2.50  마찰 0.13'."""
    return (f"토크학습 {'-' if perc is None else f'{perc}%'}  횡가속 {_fmt(lat_accel_factor, '.2f')}  "
            f"마찰 {_fmt(friction, '.2f')}")


LANE_P_MIN = 0.6
LANE_STD_MAX = 0.2
LANE_W_MIN, LANE_W_MAX = 2.6, 4.4
LANE_TAU_S = 0.5


def lane_offset_m(model: Any | None) -> float | None:
    """Car offset from the lane centre at the car [m], + right; None while the inner lines are weak."""
    if model is None:
        return None
    try:
        lines = model.laneLines
        probs = model.laneLineProbs
        stds = model.laneLineStds
        if len(lines) < 4 or len(probs) < 4 or len(lines[1].y) == 0 or len(lines[2].y) == 0:
            return None
        p1, p2 = float(probs[1]), float(probs[2])
        s1, s2 = (float(stds[1]), float(stds[2])) if len(stds) >= 4 else (0.0, 0.0)
        y1, y2 = float(lines[1].y[0]), float(lines[2].y[0])
    except Exception:
        return None
    if not all(math.isfinite(v) for v in (p1, p2, s1, s2, y1, y2)):
        return None
    if min(p1, p2) < LANE_P_MIN or max(s1, s2) > LANE_STD_MAX or not LANE_W_MIN <= y2 - y1 <= LANE_W_MAX:
        return None
    return -(y1 + y2) / 2.0


class LaneOffsetFilter:
    """0.5 s low-pass of lane_offset_m; restarts after a weak / missing frame (shows '--' meanwhile)."""

    def __init__(self, tau_s: float = LANE_TAU_S):
        self.tau_s = tau_s
        self.value: float | None = None
        self.t: float | None = None

    def update(self, offset_m: float | None, now: float) -> float | None:
        if offset_m is None:
            self.value, self.t = None, None
            return None
        if self.value is None or self.t is None:
            self.value = offset_m
        else:
            dt = min(max(now - self.t, 0.0), 1.0)
            self.value += dt / (self.tau_s + dt) * (offset_m - self.value)
        self.t = now
        return self.value


CENTER_BAND_M = 0.02     # |offset| below -> '치우침 가운데' (보노보노 2026-10-25 23:52)
# car centre <-> inner line [m]; real 1c9 / 1ca clear lines 0.25..2.70 m (p0.1 0.76, p99.9 2.31): keeps them all, '--' for a
# line under the car centre or farther than one wide lane
GAP_MIN_M, GAP_MAX_M = 0.2, 3.5


def _line_ok(model: Any, i: int) -> float | None:
    """y [m, + right] of lane line i at the car when that line is clear (prob >= LANE_P_MIN, std <= LANE_STD_MAX)."""
    try:
        line = model.laneLines[i]
        p = float(model.laneLineProbs[i])
        stds = model.laneLineStds
        s = float(stds[i]) if len(stds) > i else 0.0
        y = float(line.y[0])
    except Exception:
        return None
    if not all(math.isfinite(v) for v in (p, s, y)) or p < LANE_P_MIN or s > LANE_STD_MAX:
        return None
    return y


def lane_gaps_m(model: Any | None) -> tuple[float | None, float | None]:
    """(left, right) distance [m] from the car centre (comma, y = 0) to the middle of the inner lane line at the car
    (-y1, y2; left + right = lane width); None per weak line or outside GAP_MIN_M..GAP_MAX_M."""
    if model is None:
        return None, None
    try:
        if len(model.laneLines) < 4 or len(model.laneLineProbs) < 4:
            return None, None
    except Exception:
        return None, None
    y1, y2 = _line_ok(model, 1), _line_ok(model, 2)
    left = None if y1 is None else -y1
    right = None if y2 is None else y2
    left = left if left is not None and GAP_MIN_M <= left <= GAP_MAX_M else None
    right = right if right is not None and GAP_MIN_M <= right <= GAP_MAX_M else None
    return left, right


def lane_offset_text(offset_m: float | None) -> str:
    """HUD line 4: '치우침 오른쪽 12cm' / '치우침 왼쪽 8cm' / '치우침 가운데' (|offset| < CENTER_BAND_M) / '치우침 --' (lines weak).
    offset = car from the lane centre in modelV2 y (+ = right)."""
    if offset_m is None or not math.isfinite(offset_m):
        return "치우침 --"
    if abs(offset_m) < CENTER_BAND_M:
        return "치우침 가운데"
    return f"치우침 {'오른쪽' if offset_m > 0 else '왼쪽'} {int(round(abs(offset_m) * 100.0))}cm"


def lane_assist_text(lane_state: Any) -> str:
    """'보정 꺼짐' (K9LaneAssist 0) / '보정 대기' (on, not pushing) / '보정 켬' (pushing now) / '보정 --' (unknown)."""
    try:
        state = int(lane_state)
    except (TypeError, ValueError):
        return "보정 --"
    return {0: "보정 꺼짐", 1: "보정 대기", 2: "보정 켬", 3: "보정 켬"}.get(state, "보정 --")


def lane_gap_text(left_m: float | None, right_m: float | None) -> str:
    """HUD line 5: '좌 차선 1.87m   우 차선 1.63m' (car centre / comma <-> lane line); '--' for a weak / missing line."""
    f = lambda v: "--" if v is None or not math.isfinite(v) else f"{v:.2f}m"
    return f"좌 차선 {f(left_m)}   우 차선 {f(right_m)}"


def lane_values_text(offset_m: float | None, lane_state: Any = None, gap_left_m: float | None = None,
                     gap_right_m: float | None = None) -> str:
    """HUD lines 4 and 5 joined by ' / ' (manual / tests): '치우침 오른쪽 12cm / 좌 차선 1.87m   우 차선 1.63m'. Shown whatever
    K9LaneAssist is; lane_state (보정) is no longer shown on the external HUD (2026-10-26), the argument is kept for callers."""
    return f"{lane_offset_text(offset_m)} / {lane_gap_text(gap_left_m, gap_right_m)}"
