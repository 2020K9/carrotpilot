import ast
import math
from numbers import Number
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from openpilot.cereal import car, log
from openpilot.selfdrive.controls.lib.laneless_center import LanelessCenterCorrection
from openpilot.selfdrive.controls.lib.lat_mode_blend import (LAT_MODE_BLEND_SECONDS, blend_lat_mode,
                                                             lat_mode_blend_target, update_lat_mode_blend)

DT = 0.01
CYCLES = int(round(LAT_MODE_BLEND_SECONDS / DT))
LANE_CURVATURE = 0.03
LANELESS_CURVATURE = -0.03
LAT_SMOOTH_SECONDS = 0.2  # LatSmoothSec param is in centiseconds


def old_smooth(val, prev_val, tau):
  alpha = 1 - np.exp(-DT / tau) if tau > 0 else 1
  return alpha * val + (1 - alpha) * prev_val


# ---------------------------------------------------------------- pure helper


def test_weight_ramps_linearly_and_reaches_target_after_one_second():
  weight = 0.0
  for _ in range(CYCLES - 1):
    weight = update_lat_mode_blend(weight, True, True, DT)
  assert weight < 1.0
  weight = update_lat_mode_blend(weight, True, True, DT)
  assert weight == pytest.approx(1.0)

  for _ in range(CYCLES - 1):
    weight = update_lat_mode_blend(weight, False, True, DT)
  assert weight > 0.0
  assert update_lat_mode_blend(weight, False, True, DT) == pytest.approx(0.0)


def test_blend_endpoints_are_exact():
  assert blend_lat_mode(1.0, LANE_CURVATURE, LANELESS_CURVATURE) == LANE_CURVATURE
  assert blend_lat_mode(0.0, LANE_CURVATURE, LANELESS_CURVATURE) == LANELESS_CURVATURE
  assert blend_lat_mode(0.5, 1.0, 0.0) == 0.5


def test_missing_lane_candidate_drops_to_laneless_at_once():
  assert update_lat_mode_blend(1.0, True, False, DT) == 0.0
  assert lat_mode_blend_target(True, False) == 0.0
  assert lat_mode_blend_target(True, True) == 1.0
  assert lat_mode_blend_target(False, True) == 0.0


def test_blended_target_moves_slower_than_an_instant_switch():
  weight, prev, max_step = 1.0, LANE_CURVATURE, 0.0
  for _ in range(CYCLES + 10):
    weight = update_lat_mode_blend(weight, False, True, DT)
    target = blend_lat_mode(weight, LANE_CURVATURE, LANELESS_CURVATURE)
    max_step = max(max_step, abs(target - prev))
    prev = target
  assert prev == LANELESS_CURVATURE
  instant_step = abs(LANELESS_CURVATURE - LANE_CURVATURE)
  assert max_step < instant_step
  assert max_step == pytest.approx(instant_step * DT / LAT_MODE_BLEND_SECONDS)


# ------------------------------------------------------------- controlsd path


class FakeSubMaster(dict):
  frame = 0
  recv_frame = {"longitudinalPlan": 0}

  def all_checks(self, _keys=None):
    return True


def make_controls(*, lanefull=True, has_curvatures=True, lat_active=True):
  # Execute the production state_control lateral path without hardware, IPC or model inference.
  path = Path(__file__).parents[1] / "controlsd.py"
  tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
  tree.body = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.ClassDef))]

  captured = []

  def fake_clip_curvature(_v_ego, _prev, new_curvature, _roll):
    captured.append(float(new_curvature))
    return float(new_curvature), False

  from openpilot.selfdrive.controls.lib import lat_mode_blend as blend_mod

  namespace = {
    "math": math, "np": np, "car": car, "Number": Number, "CV": SimpleNamespace(KPH_TO_MS=1 / 3.6),
    "DT_CTRL": DT, "MIN_LATERAL_CONTROL_SPEED": 0.3,
    "LaneChangeState": log.LaneChangeState, "LaneChangeDirection": log.LaneChangeDirection,
    "ACTUATOR_FIELDS": tuple(car.CarControl.Actuators.schema.fields.keys()),
    "cloudlog": SimpleNamespace(error=lambda *_a: None, info=lambda *_a: None),
    "resolve_vehicle_model_steer_ratio": lambda *_args: 15.0,
    "get_lag_adjusted_curvature": lambda *_args: LANE_CURVATURE,
    "clip_curvature": fake_clip_curvature,
    "LAT_MODE_BLEND_SECONDS": blend_mod.LAT_MODE_BLEND_SECONDS,
    "blend_lat_mode": blend_mod.blend_lat_mode,
    "lat_mode_blend_target": blend_mod.lat_mode_blend_target,
    "update_lat_mode_blend": blend_mod.update_lat_mode_blend,
  }
  exec(compile(tree, str(path), "exec"), namespace)

  controls = namespace["Controls"].__new__(namespace["Controls"])
  controls.CP = SimpleNamespace(brand="hyundai", steerAtStandstill=False, minSteerSpeed=0.0,
                                openpilotLongitudinalControl=False,
                                lateralTuning=SimpleNamespace(which=lambda: "angle"))
  float_params = {"LatSmoothSec": LAT_SMOOTH_SECONDS * 100.0, "SteerActuatorDelay": 20.0}
  bool_params = {"AlwaysLateral": True, "ImpactDashcamReboot": False}
  controls.params = SimpleNamespace(get_float=lambda key: float_params.get(key, 0.0),
                                    get_bool=lambda key: bool_params[key],
                                    get_int=lambda _key: 0)
  controls.is_vw_meb = False
  controls.steer_limited_by_safety = False
  controls.desired_curvature = LANE_CURVATURE
  controls.lat_mode_blend = 1.0
  controls.calibrated_pose = None
  controls.VM = SimpleNamespace(update_params=lambda *_args: None, calc_curvature=lambda *_args: 0.0)
  controls.LoC = SimpleNamespace(reset=lambda: None, long_control_state="off",
                                 update=lambda *_args: (0.0, 0.0, 0.0))
  controls.LaC = SimpleNamespace(reset=lambda: None, update=lambda *_a, **_k: (0.0, 0.0, None))
  controls.CI = SimpleNamespace(get_pid_accel_limits=lambda *_args: (-1.0, 1.0))
  controls.carrot_controls = SimpleNamespace(lat_suspend_control=lambda _cs, active: active)
  # Startup gate already passed and first activation already seeded, so only the blend is under test.
  controls.lateral_startup = SimpleNamespace(update=lambda *_args: True)
  controls.lateral_started = True
  # Production default: no LanelessCenterConfig -> disabled, never operational.
  controls.laneless_center = LanelessCenterCorrection(None)

  curvatures = [LANE_CURVATURE] * 17 if has_curvatures else []
  controls.sm = FakeSubMaster({
    "carState": car.CarState.new_message(vEgo=20.0, standstill=False, gearShifter="drive",
                                         latEnabled=lat_active, vCruise=100.0),
    "liveParameters": SimpleNamespace(stiffnessFactor=1.0, steerRatio=15.0, angleOffsetDeg=0.0, roll=0.0),
    "liveDelay": SimpleNamespace(lateralDelay=0.2),
    "longitudinalPlan": SimpleNamespace(), "radarState": SimpleNamespace(),
    "modelV2": SimpleNamespace(action=SimpleNamespace(desiredCurvature=LANELESS_CURVATURE),
                               meta=SimpleNamespace(laneChangeState=log.LaneChangeState.off)),
    "lateralPlan": SimpleNamespace(useLaneLines=lanefull, curvatures=curvatures,
                                   psis=[0.0] * 17, distances=[1.0] * 17),
    "carrotMan": SimpleNamespace(vTurnSpeed=100.0),
    "selfdriveState": SimpleNamespace(enabled=True, active=True),
    "onroadEvents": [],
  })
  return controls, captured


def run_cycles(controls, captured, count):
  out = []
  for _ in range(count):
    controls.state_control()
    out.append(captured[-1])
  return out


def test_lane_to_laneless_switch_is_rate_limited_and_completes_in_one_second():
  controls, captured = make_controls()
  run_cycles(controls, captured, 1)  # settled in lane mode
  assert controls.lat_mode_blend == 1.0

  controls.sm["lateralPlan"].useLaneLines = False
  outputs = [captured[-1], *run_cycles(controls, captured, CYCLES)]
  assert controls.lat_mode_blend == pytest.approx(0.0)

  steps = [abs(b - a) for a, b in zip(outputs[:-1], outputs[1:])]

  # Old behavior: the source switched in one cycle, so the first step was a full
  # jump from the lane-smoothed value to the laneless-smoothed value.
  old_first_step = abs(old_smooth(LANELESS_CURVATURE, outputs[0], 0.1) - outputs[0])
  assert max(steps) < old_first_step


def test_output_matches_old_behavior_once_the_blend_is_done():
  controls, captured = make_controls()
  expected = old_smooth(LANE_CURVATURE, controls.desired_curvature, LAT_SMOOTH_SECONDS)
  assert run_cycles(controls, captured, 1) == [expected]
  assert controls.lat_mode_blend == 1.0

  controls.sm["lateralPlan"].useLaneLines = False
  run_cycles(controls, captured, CYCLES)
  assert controls.lat_mode_blend == pytest.approx(0.0)
  prev = controls.desired_curvature
  assert run_cycles(controls, captured, 1) == [old_smooth(LANELESS_CURVATURE, prev, 0.1)]


def test_missing_lane_candidate_is_immediately_laneless():
  controls, captured = make_controls(has_curvatures=False)
  prev = controls.desired_curvature
  assert run_cycles(controls, captured, 1) == [old_smooth(LANELESS_CURVATURE, prev, 0.1)]
  assert controls.lat_mode_blend == 0.0


@pytest.mark.parametrize("lanefull,has_curvatures,expected", [(True, True, 1.0), (False, True, 0.0),
                                                              (True, False, 0.0)])
def test_weight_snaps_while_lateral_is_inactive(lanefull, has_curvatures, expected):
  controls, captured = make_controls(lanefull=lanefull, has_curvatures=has_curvatures, lat_active=False)
  controls.lat_mode_blend = 1.0 - expected
  assert run_cycles(controls, captured, 1) == [controls.curvature]
  assert controls.lat_mode_blend == expected
