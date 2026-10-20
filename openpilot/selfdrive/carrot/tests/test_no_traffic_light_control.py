"""Regression: traffic-light assistance is removed; model stop, lead and driver behavior remain.

Executes the real CarrotPlanner source with recorded-input style doubles. This is a
state-machine check, not MPC or vehicle validation.
"""
import ast
from enum import Enum
import importlib.util
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[4]
CARROT = REPO / 'openpilot/selfdrive/carrot'


def load_file(path, name):
  spec = importlib.util.spec_from_file_location(name, path)
  mod = importlib.util.module_from_spec(spec)
  sys.modules[name] = mod
  spec.loader.exec_module(mod)
  return mod


class EventNames:
  def __getattr__(self, name):
    return name


class Events:
  def __init__(self):
    self.events = []

  def add(self, name):
    self.events.append(name)


def planner(detect_mode=1, carrot_man=None, suspended=None):
  helpers = load_file(CARROT / 't_follow.py', 'nosignal_tf')
  stop = load_file(CARROT / 'traffic_stop.py', 'nosignal_stop')
  filters = load_file(REPO / 'openpilot/common/filter_simple.py', 'nosignal_filters')
  driving = Enum('DrivingMode', {'Eco': 1, 'Safe': 2, 'Normal': 3, 'High': 4})
  ns = dict(time=time, np=np, Enum=Enum, DT_MDL=.05, CV=NS(MS_TO_KPH=3.6, KPH_TO_MS=1/3.6),
            DrivingMode=driving, DrivingModeDetector=NS, LaneChangeGapPlan=NS, LaneChangeGapTracker=NS,
            Params=lambda: NS(get_int=lambda name: 2 if name == 'MyDrivingMode' else 0),
            Events=Events, EventName=EventNames(), log=NS(LongitudinalPersonality=NS(standard=1)),
            get_carrot_man=lambda sm: carrot_man, get_mode_lead_response=lambda requested, mode: requested,
            RADAR_TO_CAMERA=1.52, MyMovingAverage=filters.MyMovingAverage,
            TrafficStopModelLeadMatcher=stop.TrafficStopModelLeadMatcher,
            is_traffic_stop_entry_allowed=stop.is_traffic_stop_entry_allowed,
            model_stop_suspended=lambda sm: bool(suspended and suspended[0]))
  ns.update({n: getattr(helpers, n) for n in dir(helpers) if not n.startswith('_')})
  source = (CARROT / 'carrot_functions.py').read_text(encoding='utf-8')
  nodes = [n for n in ast.parse(source).body if isinstance(n, (ast.ClassDef, ast.FunctionDef))]
  exec(compile(ast.Module(nodes, type_ignores=[]), 'carrot_no_signal', 'exec'), ns)
  cls = ns['CarrotPlanner']
  cls._params_update = lambda self: None
  cls._update_driving_mode = lambda self, sm: None
  cls._update_model_desire = lambda self, sm: None
  p = cls()
  p.modelStopPrediction = detect_mode
  return p


def inputs(speed=0., distance=100., lead=False, d_rel=10., model_v=15.):
  return dict(carState=NS(vCluRatio=1., softHoldActive=0, vEgo=speed, aEgo=0., vEgoCluster=speed,
                         gasPressed=False, brakePressed=False, steeringAngleDeg=0., leftBlinker=False,
                         rightBlinker=False, standstill=speed < .1),
              selfdriveState=NS(personality=1),
              radarState=NS(leadOne=NS(status=lead, dRel=d_rel), leadTwo=NS(status=False)),
              modelV2=NS(position=NS(x=[distance*i/32 for i in range(33)], y=[0.]*33),
                         velocity=NS(x=[speed]+[model_v]*32), leadsV3=[]))


def run(p, sm, count=20):
  events = []
  for _ in range(count):
    p.update(sm, 80., 'acc')
    events += p.events.events
  return events


def test_planner_has_no_signal_inputs_or_helpers():
  src = (CARROT / 'carrot_functions.py').read_text(encoding='utf-8')
  for token in ('signal_assist', 'start_guard', 'stop_prepare', 'signal_observation', 'trafficSignGreen',
                'trafficSignChanged', 'trafficState_carrot', 'carrotMan.trafficState', 'e2eStopPrepare'):
    assert token not in src
  for name in ('signal_assist', 'signal_assist_runtime', 'signal_start_guard', 'signal_stop_prepare',
               'signal_stop_prepare_runtime', 'signal_mode', 'signal_display', 'signal_road_policy',
               'lead_brake_lights'):
    assert not (CARROT / f'{name}.py').exists()
  modeld = REPO / 'openpilot/selfdrive/modeld'
  assert not list(modeld.glob('signal_*.py'))
  assert 'signalcolord' not in (REPO / 'openpilot/system/manager/process_config.py').read_text(encoding='utf-8')
  assert 'signal_shadow' not in (modeld / 'modeld.py').read_text(encoding='utf-8')


def test_model_stop_still_enters_and_stops_without_lead():
  p = planner()
  sm = inputs(speed=10., distance=40., model_v=0.)
  events = run(p, sm)
  assert p.xState.name == 'e2eStop'
  assert 0 < p.stop_dist < 40. and p.v_cruise < 80/3.6
  assert 'trafficStopping' in events  # generic model-stop notice is retained


def test_no_automatic_departure_after_model_stop_even_when_model_goes():
  p = planner()
  sm = inputs(distance=3., model_v=0.)
  run(p, sm)
  assert p.xState.name == 'e2eStopped'
  sm['modelV2'].velocity.x = [0.] + [15.]*32  # model predicts departure ("green" in old code)
  events = run(p, sm, 60)
  assert p.trafficState.name == 'green'
  assert p.xState.name == 'e2eStopped' and p.v_cruise == 0
  assert 'trafficSignGreen' not in events and 'trafficSignChanged' not in events


def test_driver_gas_resumes_from_model_stop():
  p = planner()
  sm = inputs(distance=3., model_v=0.)
  run(p, sm)
  sm['carState'].gasPressed = True
  run(p, sm, 1)
  assert p.xState.name == 'e2eCruise'


def test_lead_at_stop_point_hands_over_to_lead_following_and_lead_loss_returns_to_cruise():
  p = planner()
  sm = inputs(distance=3., model_v=0.)
  run(p, sm)
  sm['radarState'].leadOne = NS(status=True, dRel=4.)
  run(p, sm, 1)
  assert p.xState.name == 'lead'
  sm['radarState'].leadOne = NS(status=False, dRel=0.)
  sm['modelV2'] = inputs(distance=80.)['modelV2']  # path clear: no model stop
  run(p, sm, 5)
  assert p.xState.name == 'e2eCruise'


def test_lead_following_while_moving_is_unchanged():
  p = planner()
  sm = inputs(speed=15., distance=80., lead=True, d_rel=30.)
  run(p, sm)
  assert p.xState.name == 'lead' and p.stop_dist >= 0


def test_soft_hold_still_holds():
  p = planner()
  sm = inputs(distance=80.)
  sm['carState'].softHoldActive = 1
  events = run(p, sm, 5)
  assert p.xState.name == 'e2eStopped'
  assert 'trafficSignChanged' not in events


@pytest.mark.parametrize('nav_state', [1, 2, 3])
def test_navigation_traffic_light_state_has_no_control_effect(nav_state):
  carrot_man = NS(trafficState=nav_state, atcType='', activeCarrot=0, xDistToTurn=0, desiredSpeed=250)
  p = planner(carrot_man=carrot_man)
  sm = inputs(distance=3., model_v=0.)
  run(p, sm)
  assert p.xState.name == 'e2eStopped'
  carrot_man.trafficState = 2  # red -> green from navigation used to trigger a start
  events = run(p, sm, 20)
  assert p.xState.name == 'e2eStopped'
  assert 'trafficSignChanged' not in events and 'trafficSignGreen' not in events


def test_model_stop_prediction_zero_disables_model_stop():
  p = planner(detect_mode=0)
  sm = inputs(speed=10., distance=40., model_v=0.)
  run(p, sm)
  assert p.xState.name == 'e2eCruise'


def test_stop_notice_is_muted_by_lead():
  p = planner()
  p.stop_lead_present = True
  p.events = Events()
  p.last_event_time = 0
  p.add_stop_event('trafficStopping')
  assert p.events.events == []


def test_motorway_suspension_cancels_model_stop_but_keeps_soft_hold_and_user_stop():
  flag = [False]
  p = planner(suspended=flag)
  sm = inputs(speed=10., distance=40., model_v=0.)
  run(p, sm)
  assert p.xState.name == 'e2eStop'
  flag[0] = True  # fresh motorway class: generic model stop is suspended
  run(p, sm, 5)
  assert p.xState.name == 'e2eCruise' and p.actual_stop_distance == 0.
  flag[0] = False  # back to ordinary road: model stop may enter again
  run(p, sm)
  assert p.xState.name == 'e2eStop'

  p = planner(suspended=[True])
  sm = inputs(distance=80.)
  sm['carState'].softHoldActive = 1
  run(p, sm, 5)
  assert p.xState.name == 'e2eStopped'  # driver/vehicle hold is not cancelled

  p = planner(suspended=[True])
  sm = inputs(speed=5., distance=80.)
  p.user_stop_distance = 20.
  run(p, sm, 3)
  assert p.xState.name == 'e2eStop' and 0 < p.stop_dist <= 20.  # explicit user stop survives suspension


def test_gas_from_moving_model_stop_blocks_reentry_for_ten_seconds():
  p = planner()
  sm = inputs(speed=10., distance=40., model_v=0.)
  run(p, sm)
  assert p.xState.name == 'e2eStop'
  sm['carState'].gasPressed = True
  run(p, sm, 1)
  sm['carState'].gasPressed = False
  run(p, sm, 20)
  assert p.xState.name == 'e2eCruise'  # still within the 10 s cooldown
