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
