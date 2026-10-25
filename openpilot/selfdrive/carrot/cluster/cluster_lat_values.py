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

Lane line above it (보노보노 2026-10-25 20:16 "라이브값 따로 못보나, 좌우치우침"):
- OFS: car offset from the lane centre in cm (+ right, - left), from modelV2 inner lane lines at
  the car (x = 0), the same measurement K9LaneAssist's centre hold uses ( -(y1 + y2) / 2 ), with a
  0.5 s low-pass. '--' while either inner line is weak (prob < LANE_P_MIN, std > LANE_STD_MAX) or the
  width is implausible (outside LANE_W_MIN..LANE_W_MAX) - shown whatever K9LaneAssist is set to.
- LC: controlsState.k9LaneState / k9LaneAccel from controlsd: 'off' switch off, '--' on but not
  holding, otherwise the lateral accel the hold adds now (m/s^2, + = toward the right).
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


def lat_values_text(steer_ratio: float | None, steer_delay_s: float | None) -> str:
    sr = "-" if steer_ratio is None or not math.isfinite(steer_ratio) else f"{steer_ratio:.2f}"
    sad = "-" if steer_delay_s is None or not math.isfinite(steer_delay_s) else f"{steer_delay_s:.2f}s"
    return f"SR {sr}  SAD {sad}"


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


def lane_values_text(offset_m: float | None, lane_state: Any, lane_accel: Any) -> str:
    """'OFS +12cm  LC -0.04' ; '--' when unknown, LC 'off' when K9LaneAssist is off."""
    ofs = "--" if offset_m is None or not math.isfinite(offset_m) else f"{round(offset_m * 100.0):+d}cm"
    try:
        state = int(lane_state)
    except (TypeError, ValueError):
        state = None
    accel = _finite(lane_accel)
    if state is None:
        lc = "--"
    elif state == 0:
        lc = "off"
    elif state in (2, 3) and accel is not None:
        lc = f"{accel:+.2f}"
    else:
        lc = "--"
    return f"OFS {ofs}  LC {lc}"
