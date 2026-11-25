"""joongyu01 b0daa949: a red seen under held gas waits for the release; a red stop keeps its target while creeping."""
from types import SimpleNamespace as NS

from tools.signal_analysis.replay_signal_assist import REPO, planner_class
from tools.signal_analysis.test_signal_assist_planner import inputs


def planner():
  cls, _ = planner_class((REPO/'openpilot/selfdrive/carrot/carrot_functions.py').read_text(encoding='utf-8'))
  return cls()


def no_lead_red(speed=10.):
  sm = inputs(speed=speed, distance=40.)
  sm['modelV2'].velocity.x = [speed] + [0.] * 32  # model plans a stop at 40 m
  sm['radarState'].leadOne = NS(status=False, dRel=0., vLead=0.)
  return sm


def run(p, sm, v_cruise_kph, n=20):
  for _ in range(n):
    p.update(sm, v_cruise_kph, 'acc')


def test_red_stop_keeps_its_target_through_a_go_plan_while_creeping_to_the_line():
  p, sm = planner(), no_lead_red()
  run(p, sm, 80.)
  assert p.xState.name == 'e2eStop'
  sm['carState'].vEgo = sm['carState'].vEgoCluster = 2.
  sm['modelV2'].velocity.x = [2.] + [15.] * 32
  run(p, sm, 80.)
  assert p.xState.name == 'e2eStop' and p.stop_dist < 300.


def test_red_seen_under_gas_waits_for_release_instead_of_arming_the_red_ignore():
  p, sm = planner(), no_lead_red()
  sm['carState'].gasPressed = True
  run(p, sm, 80.)
  assert p.xState.name == 'e2eCruise' and p.traffic_starting_count == 0
  sm['carState'].gasPressed = False
  run(p, sm, 80., 1)
  assert p.xState.name == 'e2eStop'
