"""Carrot Web settings for lane-mode avoidance and laneless centring (default OFF).

Valid numbers are not approval: the policy tuples/implementations, the laneless safety
contract and RESIDUAL_POLICY_APPROVED stay unapproved in production. Tests that reach an
active state inject the existing synthetic fixtures only.
"""
import ast
import dataclasses
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.selfdrive.controls.lib import lane_avoid as la
from openpilot.selfdrive.controls.lib import laneless_center as lc
from openpilot.selfdrive.controls.lib import lateral_feature_settings as fs
from openpilot.selfdrive.controls.lib.lane_avoid import LEFT, Coverage, LaneAvoidConfig, LaneAvoidController
from openpilot.selfdrive.controls.lib.laneless_center import LanelessCenterConfig, LanelessCenterCorrection
from openpilot.selfdrive.controls.tests import test_lane_avoid as lat
from openpilot.selfdrive.controls.tests.test_lane_avoid import TEST_POLICY, approved_policies, avoid_then  # noqa: F401
from openpilot.selfdrive.controls.tests import test_laneless_center_v4 as lcv4

ROOT = Path(__file__).resolve().parents[2]
SETTINGS = ROOT / "carrot_settings.json"
PARAMS_KEYS = ROOT.parent / "common" / "params_keys.h"
KEYS = [k for k, *_ in fs.all_setting_specs()]


class FakeParams:
  def __init__(self, values=None, fail=None):
    self.values = dict(values or {})
    self.fail = fail

  def get(self, key):
    if key == self.fail:
      raise RuntimeError("read failure")
    return self.values.get(key)


def zeros():
  return {k: 0 for k in KEYS}


# UI values (ints) equivalent to the synthetic lane_avoid TEST_CFG. 0 means unset, so the
# fixture's lateral 0.0 m starts cannot be entered: the coverage uses -1 cm (signed, still
# covering the centreline) and the required region starts at the smallest step (1 cm).
COV_UI = {"XMin": -1500, "XMax": 11500, "LatMin": -1, "LatMax": 500}
LANE_AVOID_UI = {
  "LaneAvoidEnabled": 1, "LaneAvoidMaxOffsetCm": 50, "LaneAvoidEntryRateCmps": 30, "LaneAvoidReturnRateCmps": 20,
  "LaneAvoidRateChangeCmps2": 200, "LaneAvoidCandXMinCm": 500, "LaneAvoidCandXMaxCm": 6000,
  "LaneAvoidDeadbandCm": 5, "LaneAvoidMaxCurvature": 1000, "LaneAvoidHalfWidthCm": 95,
  "LaneAvoidEdgeMarginCm": 50, "LaneAvoidEdgeStdMaxCm": 100, "LaneAvoidRegionXMinCm": -1000,
  "LaneAvoidRegionXMaxCm": 3000, "LaneAvoidRegionLatMinCm": 1, "LaneAvoidRegionLatMaxCm": 400,
  "LaneAvoidInputGapMs": 200, "LaneAvoidConfirmFrames": 2, "LaneAvoidReentryWaitMs": 1000,
  "LaneAvoidFrontCm": 400, "LaneAvoidRearCm": 100, "LaneAvoidSideClearanceCm": 30, "LaneAvoidPredictionMs": 3000,
  **{f"LaneAvoidAge{n}Ms": 200 for n in ("Bsd", "Radar", "Model", "RoadEdge", "ModelPath")},
  **{k: COV_UI[k[len("LaneAvoidCov"):-2].removeprefix(s).removeprefix(d)]
     for (src, side), parts in fs.LANE_AVOID_COVERAGE for k, *_ in parts
     for s in [fs.SOURCE_KEY[src]] for d in [fs.SIDE_KEY[side]]},
}
POLICY_KEYS = [k for k, *_ in fs.LANE_AVOID_POLICIES]
EXPECTED_LANE_AVOID = dataclasses.replace(
  lat.TEST_CFG, required_region_lat_m=(0.01, 4.0),
  coverage={ss: Coverage(-15.0, 115.0, -0.01, 5.0) for ss, _ in fs.LANE_AVOID_COVERAGE},
  return_risk_policy=None, center_invalid_policy=None, occupancy_prediction_policy=None,
  body_geometry_model=None, constraint_conflict_policy=None)

LANELESS_UI = {
  "LanelessCenterEnabled": 1, "LanelessCenterMinLaneProbPct": 50, "LanelessCenterMaxLaneStdCm": 30,
  "LanelessCenterCheckXMinCm": 500, "LanelessCenterCheckXMaxCm": 4000, "LanelessCenterEvalXCm": 2000,
  "LanelessCenterGainPct": 50, "LanelessCenterMaxMismatch": 1000, "LanelessCenterMaxModelAgeMs": 200,
  "LanelessCenterMaxDeltaRate": 100000, "LanelessCenterMaxLaneWeightPct": 1,
}
EXPECTED_LANELESS = dataclasses.replace(lcv4.TEST_CFG, max_lane_mode_weight=0.01)


def valid_params(**overrides):
  values = {**zeros(), **LANE_AVOID_UI, **LANELESS_UI}
  values.update(overrides)
  return FakeParams(values)


def assert_close_config(cfg, expected):
  for f in dataclasses.fields(expected):
    a, b = getattr(cfg, f.name), getattr(expected, f.name)
    if isinstance(b, float):
      assert a == pytest.approx(b, rel=1e-12), f.name
    elif isinstance(b, tuple):
      assert a == pytest.approx(b, rel=1e-12), f.name
    elif isinstance(b, dict):
      assert a.keys() == b.keys(), f.name
      for key in b:
        va, vb = a[key], b[key]
        if isinstance(vb, Coverage):
          assert dataclasses.astuple(va) == pytest.approx(dataclasses.astuple(vb), rel=1e-12), (f.name, key)
        else:
          assert va == pytest.approx(vb, rel=1e-12), (f.name, key)
    else:
      assert a == b, f.name


# ---------------------------------------------------------------- defaults: both off, offset 0

@pytest.mark.parametrize("params", [FakeParams(), FakeParams(zeros())], ids=["never_saved", "all_zero"])
def test_default_settings_disable_both_features(params):
  cfg = fs.lane_avoid_config_from_params(params)
  assert cfg == LaneAvoidConfig()
  assert "feature_disabled" in cfg.blockers()
  ctrl = LaneAvoidController(cfg)
  assert not ctrl.refresh() and ctrl.state == la.DISABLED and ctrl.applied == 0.0
  assert fs.laneless_center_config_from_params(params) is None
  corr = fs.updated_laneless_center(LanelessCenterCorrection(), fs.laneless_center_config_from_params(params))
  assert not corr.enabled and not corr.operational
  assert corr.update(**lcv4.inputs()) == 0.0


def test_default_settings_keep_planner_offset_exactly_zero():
  from openpilot.selfdrive.controls.tests.test_lane_model_speed_planner import inputs, make_planner
  planner = make_planner()
  planner.params = SimpleNamespace(get=FakeParams(zeros()).get, get_int=lambda _k: 0, get_float=lambda _k: 1.0)
  for _ in range(25):
    planner.update(inputs(), SimpleNamespace(atc_active=False))
    assert planner.lane_avoid_out is None and not planner.lane_avoid.active
  assert planner.lanelines_active  # lane mode consumed, yet no avoidance output/offset
  assert planner.lane_avoid_settings_cfg == LaneAvoidConfig()


def test_actual_planner_replaces_config_only_when_settings_change():
  from openpilot.selfdrive.controls.tests.test_lane_model_speed_planner import inputs, make_planner
  planner = make_planner()
  store = FakeParams(zeros())
  planner.params = SimpleNamespace(get=store.get, get_int=lambda _k: 0, get_float=lambda _k: 1.0)
  sm, carrot = inputs(lane_speed=0), SimpleNamespace(atc_active=False)
  default = planner.lane_avoid.cfg
  for _ in range(20):
    planner.update(sm, carrot)
  assert planner.lane_avoid.cfg is default  # equal default read: no replacement
  store.values.update(LANE_AVOID_UI)
  for _ in range(10):  # the existing 10-frame (0.5 s) parameter block
    planner.update(sm, carrot)
  enabled = planner.lane_avoid.cfg
  assert enabled is not default and enabled.enabled and not planner.lane_avoid.active  # policies unapproved
  for _ in range(20):
    planner.update(sm, carrot)
  assert planner.lane_avoid.cfg is enabled  # same values again: same object, no reset
  store.values["LaneAvoidMaxOffsetCm"] = "bad"
  for _ in range(10):
    planner.update(sm, carrot)
  assert planner.lane_avoid.cfg == LaneAvoidConfig()  # unreadable -> disabled


# ---------------------------------------------------------------- invalid values fail closed

BAD_VALUES = [None, True, False, 1.5, float("nan"), float("inf"), "50", b"50"]


@pytest.mark.parametrize("bad", BAD_VALUES)
@pytest.mark.parametrize("key", ["LaneAvoidEnabled", "LaneAvoidMaxOffsetCm", "LaneAvoidCovRadarLeftLatMinCm",
                                 "LaneAvoidAgeModelMs", "LaneAvoidConfirmFrames", "LaneAvoidBodyModel"])
def test_lane_avoid_bad_stored_value_disables_whole_feature(key, bad):
  assert fs.lane_avoid_config_from_params(valid_params(**{key: bad})) == LaneAvoidConfig()


@pytest.mark.parametrize("bad", BAD_VALUES)
@pytest.mark.parametrize("key", ["LanelessCenterEnabled", "LanelessCenterGainPct", "LanelessCenterEvalXCm"])
def test_laneless_bad_stored_value_disables_whole_feature(key, bad):
  assert fs.laneless_center_config_from_params(valid_params(**{key: bad})) is None


@pytest.mark.parametrize("key", ["LaneAvoidEnabled", "LaneAvoidRearCm", "LanelessCenterEnabled", "LanelessCenterGainPct"])
def test_read_exception_disables(key):
  params = valid_params()
  params.fail = key
  if key.startswith("LaneAvoid"):
    assert fs.lane_avoid_config_from_params(params) == LaneAvoidConfig()
    assert fs.laneless_center_config_from_params(params) is not None  # the other feature is independent
  else:
    assert fs.laneless_center_config_from_params(params) is None
    assert fs.lane_avoid_config_from_params(params).enabled


@pytest.mark.parametrize("enable", [2, -1, 100])
def test_enable_must_be_exactly_zero_or_one(enable):
  assert fs.lane_avoid_config_from_params(valid_params(LaneAvoidEnabled=enable)) == LaneAvoidConfig()
  assert fs.laneless_center_config_from_params(valid_params(LanelessCenterEnabled=enable)) is None


@pytest.mark.parametrize("key", [k for k in POLICY_KEYS if k != "LaneAvoidBodyModel"])
def test_unsupported_policy_choice_disables(key):
  assert fs.lane_avoid_config_from_params(valid_params(**{key: 1})) == LaneAvoidConfig()


def test_enabled_with_unset_values_is_blocked():
  cfg = fs.lane_avoid_config_from_params(FakeParams({**zeros(), "LaneAvoidEnabled": 1}))
  assert cfg.enabled and cfg.max_offset_m is None and cfg.blockers()
  assert not LaneAvoidController(cfg).active
  ll = fs.laneless_center_config_from_params(FakeParams({**zeros(), "LanelessCenterEnabled": 1}))
  assert ll is not None and set(ll.errors()) == {f.name for f in dataclasses.fields(ll)}
  assert not LanelessCenterCorrection(ll).enabled


@pytest.mark.parametrize("key", sorted(set(LANE_AVOID_UI) - {"LaneAvoidEnabled"}))
def test_lane_avoid_each_unset_value_blocks(approved_policies, monkeypatch, key):
  inject_policy_choices(monkeypatch)
  params = valid_params(**{k: 1 for k in POLICY_KEYS}, **{key: 0})
  ctrl = LaneAvoidController(fs.lane_avoid_config_from_params(params))
  assert ctrl.blockers and not ctrl.active


@pytest.mark.parametrize("key", sorted(set(LANELESS_UI) - {"LanelessCenterEnabled"}))
def test_laneless_each_unset_value_blocks(key):
  cfg = fs.laneless_center_config_from_params(valid_params(**{key: 0}))
  assert cfg.errors() and not LanelessCenterCorrection(cfg).enabled


@pytest.mark.parametrize("overrides", [
  dict(LaneAvoidCandXMinCm=6000, LaneAvoidCandXMaxCm=500),        # min >= max
  dict(LaneAvoidRegionLatMinCm=400, LaneAvoidRegionLatMaxCm=400),
  dict(LaneAvoidMaxOffsetCm=-5),                                    # not > 0
  dict(LaneAvoidConfirmFrames=-1),
])
def test_lane_avoid_invalid_numbers_keep_code_validation(approved_policies, monkeypatch, overrides):
  inject_policy_choices(monkeypatch)
  cfg = fs.lane_avoid_config_from_params(valid_params(**{k: 1 for k in POLICY_KEYS}, **overrides))
  assert cfg.blockers() and not LaneAvoidController(cfg).active


@pytest.mark.parametrize("overrides", [
  dict(LanelessCenterGainPct=101), dict(LanelessCenterMinLaneProbPct=-1),
  dict(LanelessCenterCheckXMaxCm=500), dict(LanelessCenterEvalXCm=4500), dict(LanelessCenterMaxModelAgeMs=-1),
])
def test_laneless_invalid_numbers_keep_code_validation(overrides):
  cfg = fs.laneless_center_config_from_params(valid_params(**overrides))
  assert cfg.errors() and not LanelessCenterCorrection(cfg).enabled


# ---------------------------------------------------------------- valid values

def inject_policy_choices(monkeypatch):
  # Synthetic only: lets UI value 1 select the test fixture's (patched-approved) policy name.
  monkeypatch.setattr(fs, "LANE_AVOID_POLICIES", tuple((k, f, {1: TEST_POLICY}) for k, f, _ in fs.LANE_AVOID_POLICIES))


def test_lane_avoid_valid_numbers_are_not_approval():
  cfg = fs.lane_avoid_config_from_params(valid_params())
  assert_close_config(cfg, EXPECTED_LANE_AVOID)
  # every remaining blocker is a human policy decision, never a numeric field
  assert set(cfg.blockers()) == ({f"{f}_unapproved" for _, f, _ in la.POLICY_FIELDS} |
                                 {f"{f}_unimplemented" for _, f, _ in la.POLICY_FIELDS})
  body = fs.lane_avoid_config_from_params(valid_params(LaneAvoidBodyModel=1))
  assert body.body_geometry_model == la.BODY_MODEL_RIGID_HEADING
  assert "body_geometry_model_unapproved" in body.blockers()  # implemented, still not approved
  assert not LaneAvoidController(body).active
  assert la.APPROVED_BODY_GEOMETRY_MODELS == () and la.APPROVED_RETURN_RISK_POLICIES == ()


def test_lane_avoid_valid_settings_activate_only_with_synthetic_policies(approved_policies, monkeypatch):
  inject_policy_choices(monkeypatch)
  cfg = fs.lane_avoid_config_from_params(valid_params(**{k: 1 for k in POLICY_KEYS}))
  assert_close_config(cfg, dataclasses.replace(EXPECTED_LANE_AVOID, **{f: TEST_POLICY for _, f, _ in la.POLICY_FIELDS}))
  assert cfg.blockers() == []
  ctrl = LaneAvoidController(cfg)
  assert ctrl.active
  _, _, _, outs, _ = avoid_then(ctrl, LEFT, n=30)
  assert outs[-1].valid and outs[-1].applied < 0.0  # LEFT avoidance: negative model-y offset


def test_laneless_valid_numbers_enable_computation_but_not_operation():
  cfg = fs.laneless_center_config_from_params(valid_params())
  assert_close_config(cfg, EXPECTED_LANELESS)
  assert cfg.approved()  # numerically valid only (name kept by laneless_center.py)
  corr = LanelessCenterCorrection(cfg)
  assert corr.enabled and not corr.operational
  assert lc.APPROVED_SAFETY is None and lc.RESIDUAL_POLICY_APPROVED is False
  delta = 0.0
  for _ in range(100):
    delta = corr.update(**lcv4.inputs())
  assert delta > 0.0  # synthetic computation effect; controlsd still gates on operational


def test_laneless_operational_only_with_synthetic_safety_and_residual_policy(monkeypatch):
  cfg = fs.laneless_center_config_from_params(valid_params())
  assert not LanelessCenterCorrection(cfg, lcv4.safety()).operational
  monkeypatch.setattr(lc, "RESIDUAL_POLICY_APPROVED", True)
  assert LanelessCenterCorrection(cfg, lcv4.safety()).operational


# ---------------------------------------------------------------- changes

def test_unchanged_laneless_config_keeps_object_and_state():
  params = valid_params()
  corr = fs.updated_laneless_center(LanelessCenterCorrection(), fs.laneless_center_config_from_params(params))
  for _ in range(50):
    corr.update(**lcv4.inputs())
  assert corr.delta != 0.0
  same = fs.updated_laneless_center(corr, fs.laneless_center_config_from_params(params))
  assert same is corr and same.delta == corr.delta


@pytest.mark.parametrize("change", [dict(LanelessCenterEnabled=0), dict(LanelessCenterGainPct=0),
                                    dict(LanelessCenterGainPct=40), dict(LanelessCenterGainPct="x")])
def test_laneless_change_never_reuses_residual_state(change):
  corr = fs.updated_laneless_center(LanelessCenterCorrection(), fs.laneless_center_config_from_params(valid_params()))
  for _ in range(50):
    corr.update(**lcv4.inputs())
  corr.unresolved = lc.UNRESOLVED_RECOMPUTE
  new = fs.updated_laneless_center(corr, fs.laneless_center_config_from_params(valid_params(**change)))
  assert new is not corr
  assert new.delta == 0.0 and new.baseline_desired is None and new.last_residual == 0.0
  assert new.unresolved == lc.UNRESOLVED_RECOMPUTE  # latch outlives the settings change
  assert not new.baseline_independent  # a residual delta was dropped by the change
  assert not new.operational


def read_planner_settings(planner, params):
  # Execute only the planner's settings read (the readParams block) against real objects.
  planner.params = params
  cfg = fs.lane_avoid_config_from_params(planner.params)
  if cfg != planner.lane_avoid_settings_cfg:
    planner.lane_avoid_settings_cfg = planner.lane_avoid.cfg = cfg


def test_planner_source_reads_settings_in_existing_param_block():
  src = (Path(__file__).resolve().parents[1] / "lib" / "lateral_planner.py").read_text()
  block = src[src.index("if self.readParams <= 0:"):src.index("# clip speed")]
  assert "lane_avoid_cfg = lane_avoid_config_from_params(self.params)" in block
  assert "if lane_avoid_cfg != self.lane_avoid_settings_cfg:" in block
  assert "self.lane_avoid_settings_cfg = self.lane_avoid.cfg = lane_avoid_cfg" in block


@pytest.mark.parametrize("change", [dict(LaneAvoidEnabled=0), dict(LaneAvoidMaxOffsetCm=0),
                                    dict(LaneAvoidMaxOffsetCm=40), dict(LaneAvoidRearCm=None)])
def test_lane_avoid_change_resets_residual_and_unchanged_keeps_it(approved_policies, monkeypatch, change):
  inject_policy_choices(monkeypatch)
  planner = SimpleNamespace(lane_avoid=LaneAvoidController(), params=None)
  planner.lane_avoid_settings_cfg = planner.lane_avoid.cfg
  on = valid_params(**{k: 1 for k in POLICY_KEYS})
  read_planner_settings(planner, on)
  ctrl = planner.lane_avoid
  assert ctrl.refresh()
  avoid_then(ctrl, LEFT, n=30)
  applied = ctrl.applied
  assert applied != 0.0
  read_planner_settings(planner, valid_params(**{k: 1 for k in POLICY_KEYS}))  # equal values, new read
  ctrl.refresh()
  assert ctrl.applied == applied and ctrl.state == la.AVOID  # no reset without a change
  read_planner_settings(planner, valid_params(**{k: 1 for k in POLICY_KEYS}, **change))
  ctrl.refresh()
  assert ctrl.applied == 0.0 and ctrl.rate == 0.0 and ctrl.avoid_side is None
  assert ctrl.dropped is not None and "approval_change_with_residual_unverified" in ctrl.note


def test_controlsd_reads_laneless_settings_at_init_and_1hz_only():
  path = Path(__file__).resolve().parents[1] / "controlsd.py"
  tree = ast.parse(path.read_text(encoding="utf-8"))
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "Controls")
  update = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "update")
  ns = {"Pose": SimpleNamespace(from_live_pose=lambda _p: None),
        "laneless_center_config_from_params": fs.laneless_center_config_from_params,
        "updated_laneless_center": fs.updated_laneless_center}
  exec(compile(ast.Module(body=[update], type_ignores=[]), str(path), "exec"), ns)
  sm = SimpleNamespace(frame=0, updated={"liveCalibration": False, "livePose": False})
  sm.update = lambda _t: setattr(sm, "frame", sm.frame + 1)
  reads = []

  class CountingParams(FakeParams):
    def get(self, key):
      reads.append(key)
      return super().get(key)
  params = CountingParams(zeros())
  ctl = SimpleNamespace(sm=sm, params=params, laneless_center=LanelessCenterCorrection())
  first = ctl.laneless_center
  for _ in range(99):
    ns["update"](ctl)
  assert reads == [] and ctl.laneless_center is first
  params.values.update(LANELESS_UI)
  ns["update"](ctl)  # frame 100
  assert reads and ctl.laneless_center is not first and ctl.laneless_center.enabled
  assert not ctl.laneless_center.operational
  enabled = ctl.laneless_center
  for _ in range(100):
    ns["update"](ctl)  # frame 200: same values -> same object
  assert ctl.laneless_center is enabled
  src = path.read_text(encoding="utf-8")
  assert ("self.laneless_center = updated_laneless_center(self.laneless_center, "
          "laneless_center_config_from_params(self.params))") in src


# ---------------------------------------------------------------- catalogue, registry, ranges

@pytest.fixture(scope="module")
def settings():
  return json.loads(SETTINGS.read_text(encoding="utf-8"))


def test_new_sections_are_first_in_vehicle_steering(settings):
  driving = next(c for c in settings["menu"] if c["id"] == "DRIVING")
  steer = next(g for g in driving["groups"] if g["id"] == "STEER")
  assert steer["ko"] == "차량 조향"
  first, second = steer["groups"][:2]
  assert first["id"] == "STEER_LANE_AVOID" and first["params"] == [k for k in KEYS if k.startswith("LaneAvoid")]
  assert second["id"] == "STEER_LANELESS_CENTER" and second["params"] == [k for k in KEYS if k.startswith("Laneless")]
  assert first["params"][0] == fs.LANE_AVOID_ENABLE_KEY and second["params"][0] == fs.LANELESS_CENTER_ENABLE_KEY


def test_every_new_key_is_declared_off_by_default(settings):
  by_name = {p["name"]: p for p in settings["params"]}
  registry = PARAMS_KEYS.read_text(encoding="utf-8")
  for key, _scale, lo, hi, unit in fs.all_setting_specs():
    p = by_name[key]
    assert (p["min"], p["max"], p["default"], p["unit"]) == (lo, hi, 0, 1), key
    assert f'{{"{key}", {{PERSISTENT, INT, "0"}}}}' in registry, key
    assert "기본 꺼짐" in p["descr"] and p["edescr"] and p["cdescr"], key
    assert "Default off" in p["edescr"], key
    if unit in ("cm", "ms", "%"):
      assert p["display_unit"] == {"cm": "distanceCm", "ms": "timeMs", "%": "percent"}[unit], key
      assert "0=미설정" in p["descr"] and "안전 승인값이 아님" in p["descr"], key
  for key in (fs.LANE_AVOID_ENABLE_KEY, fs.LANELESS_CENTER_ENABLE_KEY):
    assert by_name[key]["control"] == "toggle" and by_name[key]["risk"] == "high"


def test_existing_registry_order_preserved():
  registry = PARAMS_KEYS.read_text(encoding="utf-8")
  pv = registry.index('{"PathVerifier", {PERSISTENT, INT, "0"}},')
  nxt = registry.index('{"LatSuspendAngleDeg", {PERSISTENT, INT, "300"}},')
  between = registry[pv:nxt]
  assert [k for k in KEYS if f'"{k}"' in between] == KEYS


def _single_field_ok(cfg, field):
  if isinstance(cfg, LaneAvoidConfig):
    return not any(b.startswith(field) for b in cfg.blockers())
  return field not in cfg.errors()


def _with_value(key, field, v):
  values = {**valid_params().values, key: v}
  # the laneless x fields are cross-checked (0 < x_min < x_max, x_min <= eval_x <= x_max)
  if field == "check_x_min":
    values.update(LanelessCenterCheckXMaxCm=v + 1, LanelessCenterEvalXCm=v)
  elif field == "check_x_max":
    values.update(LanelessCenterCheckXMinCm=1, LanelessCenterEvalXCm=1)
  elif field == "eval_x":
    values.update(LanelessCenterCheckXMinCm=1, LanelessCenterCheckXMaxCm=max(v, 2))
  return FakeParams(values)


@pytest.mark.parametrize("spec", fs.LANE_AVOID_SCALARS + fs.LANELESS_CENTER_SCALARS + (fs.LANE_AVOID_CONFIRM,),
                         ids=lambda s: s[0])
def test_ui_range_matches_code_validation(spec):
  """Every non-zero UI value is inside the field's own code rule. Where the code has an
  upper bound (fractions) the UI maximum is exactly that bound; elsewhere the UI maximum is
  a representation limit only and the code accepts values beyond it too."""
  key, field, divisor, lo, hi, _unit = spec
  assert lo == 0
  build = fs.laneless_center_config_from_params if key.startswith("Laneless") else fs.lane_avoid_config_from_params
  # check_x_max must exceed check_x_min (>= 1 cm), so its smallest valid entry is 2
  smallest = 2 if field == "check_x_max" else 1
  for ui in (smallest, hi):
    assert _single_field_ok(build(_with_value(key, field, ui)), field), (key, ui)
  if field in ("min_lane_prob", "gain", "max_lane_mode_weight"):
    assert hi / divisor == 1.0  # code range [0, 1]; 0 itself is "unset"
    assert not _single_field_ok(build(_with_value(key, field, hi + 1)), field)
  else:
    # no finite code upper bound: one step above the UI maximum is still code-valid
    assert _single_field_ok(build(_with_value(key, field, hi + 1)), field), key
  assert not _single_field_ok(build(_with_value(key, field, 0)), field)  # 0 = unset blocks


def test_signed_ranges_accept_negative_rear_x():
  cfg = fs.lane_avoid_config_from_params(valid_params(LaneAvoidRegionXMinCm=-10000, LaneAvoidCandXMinCm=-5000))
  assert cfg.required_region_x_m == pytest.approx((-100.0, 30.0)) and cfg.candidate_x_m == pytest.approx((-50.0, 60.0))
  assert not any(b.startswith(("required_region_x_m", "candidate_x_m")) for b in cfg.blockers())


def test_partial_coverage_or_age_is_absent_not_zero():
  cfg = fs.lane_avoid_config_from_params(valid_params(LaneAvoidCovBsdLeftLatMinCm=0, LaneAvoidAgeRadarMs=0))
  assert ("bsd", LEFT) not in cfg.coverage and "radar" not in cfg.max_age_s
  assert "coverage_unapproved" in cfg.blockers() and "max_age_radar_unapproved" in cfg.blockers()


def test_policy_selectors_offer_only_existing_names(settings):
  by_name = {p["name"]: p for p in settings["params"]}
  for key, field, choices in fs.LANE_AVOID_POLICIES:
    p = by_name[key]
    assert p["control"] == "select" and p["max"] == max(choices, default=0)
    for name in choices.values():
      kind = next(k for k, f, _ in la.POLICY_FIELDS if f == field)
      assert la.policy_impl(kind, name) is not None  # implemented, but approval is separate


def test_lane_avoid_config_fields_are_all_exposed():
  exposed = ({f for _, f, *_ in fs.LANE_AVOID_SCALARS} | {fs.LANE_AVOID_CONFIRM[1]} |
             {f for f, *_ in fs.LANE_AVOID_RANGES} | {f for _, f, _ in fs.LANE_AVOID_POLICIES} |
             {"enabled", "coverage", "max_age_s"})
  assert exposed == {f.name for f in dataclasses.fields(LaneAvoidConfig)}
  assert {ss for ss, _ in fs.LANE_AVOID_COVERAGE} == {(s, side) for s in la.REQUIRED_SOURCES for side in (la.LEFT, la.RIGHT)}
  assert {n for n, *_ in fs.LANE_AVOID_AGES} == set(la.REQUIRED_SOURCES) | {"road_edge", "model_path"}
  assert {f for _, f, *_ in fs.LANELESS_CENTER_SCALARS} == {f.name for f in dataclasses.fields(LanelessCenterConfig)}


@pytest.mark.parametrize("lanefull", [True, False])
def test_valid_laneless_settings_leave_controlsd_output_bit_identical(lanefull):
  # Enabled (numerically valid) but not operational: state_control must consume exactly
  # what it consumes with the production default (disabled) correction.
  from openpilot.selfdrive.controls.tests.test_lat_mode_blend import make_controls, run_cycles
  default, default_out = make_controls(lanefull=lanefull)
  enabled, enabled_out = make_controls(lanefull=lanefull)
  enabled.laneless_center = fs.updated_laneless_center(enabled.laneless_center,
                                                       fs.laneless_center_config_from_params(valid_params()))
  assert enabled.laneless_center.enabled and not enabled.laneless_center.operational
  assert run_cycles(default, default_out, 150) == run_cycles(enabled, enabled_out, 150)
  assert default.desired_curvature == enabled.desired_curvature
