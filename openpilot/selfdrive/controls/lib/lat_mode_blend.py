"""Pure math for crossfading between the lane-based and laneless curvature sources.

Switching the desired curvature source in a single 10ms step produced a steering
jerk at every lane <-> laneless transition, so the two candidates are mixed with a
weight that ramps linearly over LAT_MODE_BLEND_SECONDS. No heavy imports here so
this stays unit-testable without the compiled modules.
"""

LAT_MODE_BLEND_SECONDS = 1.0


def lat_mode_blend_target(lanefull_mode_enabled: bool, lane_available: bool) -> float:
  # 1.0 = pure lane mode, 0.0 = pure laneless
  return 1.0 if (lanefull_mode_enabled and lane_available) else 0.0


def update_lat_mode_blend(weight: float, lanefull_mode_enabled: bool, lane_available: bool, dt: float,
                          blend_seconds: float = LAT_MODE_BLEND_SECONDS) -> float:
  # Without a lane candidate there is nothing to fade towards, so drop to laneless at once.
  if not lane_available:
    return 0.0

  target = lat_mode_blend_target(lanefull_mode_enabled, lane_available)
  if blend_seconds <= 0.0:
    return target

  step = dt / blend_seconds
  if weight < target:
    weight = min(weight + step, target)
  elif weight > target:
    weight = max(weight - step, target)
  return min(max(weight, 0.0), 1.0)


def blend_lat_mode(weight: float, lane_value: float, laneless_value: float) -> float:
  # Exact endpoints: a finished blend has to match the un-blended values bit for bit.
  if weight >= 1.0:
    return lane_value
  if weight <= 0.0:
    return laneless_value
  return weight * lane_value + (1.0 - weight) * laneless_value
