"""Carrot Web settings -> LaneAvoidConfig / LanelessCenterConfig (both default OFF).

Every setting is a persistent INT. For every field 0 means "unset" (the config's
None), never a numeric zero: a range that should start at exactly 0 m must be
entered as the smallest step instead (e.g. 1 cm). Negative values are allowed only
where the field is a signed coordinate (x behind the ego, lateral range ends).

The UI min/max in carrot_settings.json are REPRESENTATION ranges, not approved
limits. Where the config has no finite upper bound the UI maximum only limits what
can be entered; it is not checked here and the config's own validation
(LaneAvoidConfig.blockers / LanelessCenterConfig.errors) stays the only authority.

Fail closed: a read exception, a missing value (None), a bool, a non-int, an enable
value other than 0/1 or a policy choice without a known name turns the whole
feature off (LaneAvoidConfig() / None). Valid numbers are NOT approval: the lane
avoid policy tuples/implementations and the laneless safety contract and
RESIDUAL_POLICY_APPROVED are separate human decisions and are not touched here.
"""
from openpilot.selfdrive.controls.lib.lane_avoid import (BODY_MODEL_RIGID_HEADING, LEFT, REQUIRED_SOURCES, RIGHT,
                                                        Coverage, LaneAvoidConfig)
from openpilot.selfdrive.controls.lib.laneless_center import LanelessCenterConfig, LanelessCenterCorrection

# stored int / divisor = config value (division keeps e.g. 95 cm -> exactly 0.95 m)
CM = 100       # cm -> m (also cm/s -> m/s, cm/s^2 -> m/s^2)
MS = 1000      # ms -> s
PCT = 100      # % -> fraction
CURV = 100000  # 1e-5 1/m (or 1/m/s) -> 1/m

# (key, field, divisor, ui_min, ui_max, unit text). ui_min < 0 only for signed coordinates.
LANE_AVOID_ENABLE_KEY = "LaneAvoidEnabled"
LANE_AVOID_SCALARS = (
  ("LaneAvoidMaxOffsetCm", "max_offset_m", CM, 0, 300, "cm"),
  ("LaneAvoidEntryRateCmps", "entry_rate_mps", CM, 0, 200, "cm/s"),
  ("LaneAvoidReturnRateCmps", "return_rate_mps", CM, 0, 200, "cm/s"),
  ("LaneAvoidRateChangeCmps2", "max_rate_change_mps2", CM, 0, 500, "cm/s^2"),
  ("LaneAvoidDeadbandCm", "candidate_deadband_m", CM, 0, 300, "cm"),
  ("LaneAvoidMaxCurvature", "max_abs_curvature", CURV, 0, 10000, "1e-5 1/m"),
  ("LaneAvoidHalfWidthCm", "vehicle_half_width_m", CM, 0, 300, "cm"),
  ("LaneAvoidEdgeMarginCm", "road_edge_margin_m", CM, 0, 300, "cm"),
  ("LaneAvoidEdgeStdMaxCm", "road_edge_std_max_m", CM, 0, 500, "cm"),
  ("LaneAvoidInputGapMs", "max_input_gap_s", MS, 0, 5000, "ms"),
  ("LaneAvoidReentryWaitMs", "reentry_wait_s", MS, 0, 60000, "ms"),
  ("LaneAvoidFrontCm", "vehicle_front_m", CM, 0, 1000, "cm"),
  ("LaneAvoidRearCm", "vehicle_rear_m", CM, 0, 1000, "cm"),
  ("LaneAvoidSideClearanceCm", "side_clearance_m", CM, 0, 300, "cm"),
  ("LaneAvoidPredictionMs", "prediction_horizon_s", MS, 0, 10000, "ms"),
)
LANE_AVOID_CONFIRM = ("LaneAvoidConfirmFrames", "entry_confirm_frames", 1, 0, 100, "frames")  # kept int
# tuple field -> ((min key, ...), (max key, ...))
LANE_AVOID_RANGES = (
  ("candidate_x_m", ("LaneAvoidCandXMinCm", "LaneAvoidCandXMaxCm"), CM, -5000, 20000, "cm"),
  ("required_region_x_m", ("LaneAvoidRegionXMinCm", "LaneAvoidRegionXMaxCm"), CM, -10000, 20000, "cm"),
  ("required_region_lat_m", ("LaneAvoidRegionLatMinCm", "LaneAvoidRegionLatMaxCm"), CM, -1000, 1000, "cm"),
)
SOURCE_KEY = {"bsd": "Bsd", "radar": "Radar", "model": "Model"}
SIDE_KEY = {LEFT: "Left", RIGHT: "Right"}
COVERAGE_PARTS = (("XMin", -10000, 20000), ("XMax", -10000, 20000), ("LatMin", -1000, 1000), ("LatMax", -1000, 1000))
LANE_AVOID_COVERAGE = tuple(
  ((src, side), tuple((f"LaneAvoidCov{SOURCE_KEY[src]}{SIDE_KEY[side]}{part}Cm", lo, hi) for part, lo, hi in COVERAGE_PARTS))
  for src in REQUIRED_SOURCES for side in (LEFT, RIGHT))
AGE_KEY = {"bsd": "Bsd", "radar": "Radar", "model": "Model", "road_edge": "RoadEdge", "model_path": "ModelPath"}
LANE_AVOID_AGES = tuple((name, f"LaneAvoidAge{AGE_KEY[name]}Ms", MS, 0, 5000, "ms")
                        for name in REQUIRED_SOURCES + ("road_edge", "model_path"))
# policy field -> {UI value: policy name}. Only names that exist in lane_avoid are selectable;
# a name being selectable is not approval (the APPROVED_* tuples stay empty).
LANE_AVOID_POLICIES = (
  ("LaneAvoidReturnRiskPolicy", "return_risk_policy", {}),
  ("LaneAvoidCenterInvalidPolicy", "center_invalid_policy", {}),
  ("LaneAvoidOccupancyPolicy", "occupancy_prediction_policy", {}),
  ("LaneAvoidBodyModel", "body_geometry_model", {1: BODY_MODEL_RIGID_HEADING}),
  ("LaneAvoidConflictPolicy", "constraint_conflict_policy", {}),
)

LANELESS_CENTER_ENABLE_KEY = "LanelessCenterEnabled"
LANELESS_CENTER_SCALARS = (
  ("LanelessCenterMinLaneProbPct", "min_lane_prob", PCT, 0, 100, "%"),
  ("LanelessCenterMaxLaneStdCm", "max_lane_std", CM, 0, 500, "cm"),
  ("LanelessCenterCheckXMinCm", "check_x_min", CM, 0, 20000, "cm"),
  ("LanelessCenterCheckXMaxCm", "check_x_max", CM, 0, 20000, "cm"),
  ("LanelessCenterEvalXCm", "eval_x", CM, 0, 20000, "cm"),
  ("LanelessCenterGainPct", "gain", PCT, 0, 100, "%"),
  ("LanelessCenterMaxMismatch", "max_path_direct_mismatch", CURV, 0, 10000, "1e-5 1/m"),
  ("LanelessCenterMaxModelAgeMs", "max_model_age", MS, 0, 5000, "ms"),
  ("LanelessCenterMaxDeltaRate", "max_delta_rate", CURV, 0, 100000, "1e-5 1/m/s"),
  ("LanelessCenterMaxLaneWeightPct", "max_lane_mode_weight", PCT, 0, 100, "%"),
)


class _Invalid(Exception):
  pass


def _stored_int(params, key):
  try:
    v = params.get(key)
  except Exception as e:
    raise _Invalid(key) from e
  if isinstance(v, bool) or not isinstance(v, int):  # None (missing), str, float, NaN, ...
    raise _Invalid(key)
  return v


def _enabled(params, key):
  v = _stored_int(params, key)
  if v not in (0, 1):
    raise _Invalid(key)
  return v == 1


def _scaled(params, key, divisor):
  v = _stored_int(params, key)
  return None if v == 0 else v / divisor


def lane_avoid_config_from_params(params):
  """LaneAvoidConfig from the stored settings; LaneAvoidConfig() (disabled) when off or unreadable."""
  try:
    if not _enabled(params, LANE_AVOID_ENABLE_KEY):
      return LaneAvoidConfig()
    kw = {field: _scaled(params, key, scale) for key, field, scale, *_ in LANE_AVOID_SCALARS}
    key, field = LANE_AVOID_CONFIRM[:2]
    frames = _stored_int(params, key)
    kw[field] = None if frames == 0 else frames
    for field, keys, scale, *_ in LANE_AVOID_RANGES:
      lo, hi = (_scaled(params, k, scale) for k in keys)
      kw[field] = None if lo is None or hi is None else (lo, hi)
    coverage = {}
    for src_side, parts in LANE_AVOID_COVERAGE:
      values = [_scaled(params, k, CM) for k, *_ in parts]
      if all(v is not None for v in values):  # a partly set source/side stays absent (blocker)
        coverage[src_side] = Coverage(*values)
    kw["coverage"] = coverage
    ages = {}
    for name, k, scale, *_ in LANE_AVOID_AGES:
      v = _scaled(params, k, scale)
      if v is not None:
        ages[name] = v
    kw["max_age_s"] = ages
    for k, field, choices in LANE_AVOID_POLICIES:
      v = _stored_int(params, k)
      if v != 0 and v not in choices:
        raise _Invalid(k)
      kw[field] = choices.get(v)
    return LaneAvoidConfig(enabled=True, **kw)
  except _Invalid:
    return LaneAvoidConfig()


def laneless_center_config_from_params(params):
  """LanelessCenterConfig from the stored settings; None (disabled) when off or unreadable."""
  try:
    if not _enabled(params, LANELESS_CENTER_ENABLE_KEY):
      return None
    return LanelessCenterConfig(**{field: _scaled(params, key, scale) for key, field, scale, *_ in LANELESS_CENTER_SCALARS})
  except _Invalid:
    return None


def updated_laneless_center(current, cfg):
  """Same object when the config is unchanged, otherwise a fresh correction (no delta,
  baseline or trace reuse). Latches that outlive one object are carried over: an
  unresolved cycle stays latched and a non-independent baseline seed is never reported
  independent again."""
  if cfg == current.cfg:
    return current
  new = LanelessCenterCorrection(cfg)
  new.unresolved = current.unresolved
  new.baseline_independent = (current.baseline_independent and current.last_residual == 0.0
                              and current.delta == 0.0)
  return new


def all_setting_specs():
  """(key, divisor, ui_min, ui_max, unit) for every exposed setting, in UI order."""
  out = [(LANE_AVOID_ENABLE_KEY, 1, 0, 1, "")]
  out += [(k, s, lo, hi, u) for k, _f, s, lo, hi, u in LANE_AVOID_SCALARS + (LANE_AVOID_CONFIRM,)]
  out += [(k, s, lo, hi, u) for _f, keys, s, lo, hi, u in LANE_AVOID_RANGES for k in keys]
  out += [(k, CM, lo, hi, "cm") for _ss, parts in LANE_AVOID_COVERAGE for k, lo, hi in parts]
  out += [(k, s, lo, hi, u) for _n, k, s, lo, hi, u in LANE_AVOID_AGES]
  out += [(k, 1, 0, max(choices, default=0), "") for k, _f, choices in LANE_AVOID_POLICIES]
  out.append((LANELESS_CENTER_ENABLE_KEY, 1, 0, 1, ""))
  out += [(k, s, lo, hi, u) for k, _f, s, lo, hi, u in LANELESS_CENTER_SCALARS]
  return out
