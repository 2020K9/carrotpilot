from types import SimpleNamespace
import pytest

from openpilot.cereal import log
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.desire_lib.constants import BLINKER_LEFT, BLINKER_RIGHT, TURN_RELEASE_TIME
from openpilot.selfdrive.controls.lib.desire_lib.maneuver_classifier import classify_maneuver_type


def make_car_state(left_blinker=False):
  return SimpleNamespace(
    canValid=True,
    leftBlinker=left_blinker,
    rightBlinker=False,
    vEgo=20.0,
    aEgo=0.0,
    trailerConnected=False,
    steeringTorque=0.0,
    steeringPressed=False,
  )


class TestDesireHelperDriverIntent:
  def setup_method(self):
    self.helper = DesireHelper()
    self.helper._update_params_periodic = lambda: None
    self.helper._process_sides = lambda car, model, radar: None
    self.helper._check_desire_state = lambda model, car, maneuver: None
    self.helper.laneChangeNeedTorque = 0
    self.helper.laneChangeBsd = 0
    self.helper.laneLineCheck = 0

    side = self.helper.left
    side.lane_available = True
    side.edge_available = False
    side.dist_to_edge_far = 2.0
    side.lane_change_available_geom = True
    side.lane_change_available = True
    side.side_object_detected = False
    side.bsd_hold_counter = 0
    side.lane_line_info_mod = 0
    side.lane_line_info_edge_detect = False
    side.lane_change_available_released = False
    side.lane_available_trigger = False
    side.lane_appeared = False
    side.lane_exist_count.counter = 10

    self.carrot_man = SimpleNamespace(
      atcType="fork left",
      carrotCmdIndex=0,
      carrotCmd="",
      carrotArg="",
    )

  def update(self, left_blinker=False, lateral_active=True):
    self.helper.update(
      make_car_state(left_blinker),
      SimpleNamespace(),
      lateral_active,
      0.1,
      self.carrot_man,
      SimpleNamespace(),
    )

  def test_driver_blinker_rearms_lane_change_while_atc_desire_is_already_active(self):
    # The first ATC frame is intentionally ignored when its type changes.
    self.update()

    # ATC keeps the combined desire high while lateral control leaves the FSM off.
    self.update(lateral_active=False)
    assert self.helper.prev_desire_enabled
    assert self.helper.lane_change_state == log.LaneChangeState.off

    # A new physical blinker request must be treated as fresh driver intent even
    # though the combined ATC/driver desire has no rising edge.
    self.update(left_blinker=True)
    assert self.helper.lane_change_state == log.LaneChangeState.preLaneChange

    self.update(left_blinker=True)
    assert self.helper.lane_change_state == log.LaneChangeState.laneChangeStarting

  def test_atc_desire_alone_does_not_rearm_lane_change(self):
    self.update()
    self.update(lateral_active=False)

    self.update()

    assert self.helper.lane_change_state == log.LaneChangeState.off

  @pytest.mark.parametrize('block', ['inactive', 'can', 'speed', 'trailer'])
  def test_bluetooth_lane_request_rejected_by_existing_gates(self, block):
    state = make_car_state()
    state.canValid = block != 'can'
    state.vEgo = 0 if block == 'speed' else 20
    state.trailerConnected = block == 'trailer'
    self.carrot_man.atcType = ''
    seen = []
    self.helper.bluetooth_commands = SimpleNamespace(read=lambda allowed: seen.append(allowed))
    self.helper.update(state, SimpleNamespace(), block != 'inactive', .1, self.carrot_man, SimpleNamespace())
    assert seen == [False]
    assert self.helper.lane_change_state == log.LaneChangeState.off

  def test_bluetooth_lane_request_does_not_skip_blindspot(self):
    self.carrot_man.atcType = ''
    self.helper.laneChangeBsd = 1
    self.helper.left.bsd_hold_counter = 10
    self.helper.left.lane_change_available = False
    self.helper.bluetooth_commands = SimpleNamespace(read=lambda allowed: 'laneLeft' if allowed else None)
    self.update()
    self.update()
    assert self.helper.lane_change_state != log.LaneChangeState.laneChangeStarting


def make_turn_car_state(v_kph=24.0, steering_angle=0.0, left_blinker=False, right_blinker=False):
  return SimpleNamespace(
    canValid=True,
    leftBlinker=left_blinker,
    rightBlinker=right_blinker,
    vEgo=v_kph / 3.6,
    aEgo=0.0,
    steeringAngleDeg=steering_angle,
    trailerConnected=False,
    steeringTorque=0.0,
    steeringPressed=False,
  )


def make_model_data(yaw_rate=0.0, turn_desire=False):
  desire_state = [0.0] * 8
  if turn_desire:
    desire_state[1] = 1.0
  return SimpleNamespace(
    meta=SimpleNamespace(desireState=desire_state),
    orientationRate=SimpleNamespace(z=[yaw_rate] * 33),
  )


def make_side(dist_to_edge_far=2.0):
  return SimpleNamespace(dist_to_edge_far=dist_to_edge_far)


class TestManeuverClassifier:
  def classify(self, blinker_state, carstate, modeldata=None, side=None, atc_type=""):
    return classify_maneuver_type(
      blinker_state=blinker_state,
      carstate=carstate,
      side=side if side is not None else make_side(),
      turn_desire_state=False,
      atc_type=atc_type,
      old_type="none",
      modeldata=modeldata,
    )

  def test_low_speed_sharp_right_steer_is_a_turn(self):
    carstate = make_turn_car_state(v_kph=24.0, steering_angle=-93.0, right_blinker=True)
    assert self.classify(BLINKER_RIGHT, carstate, make_model_data()) == "turn"

  def test_above_classification_speed_is_always_a_lane_change(self):
    carstate = make_turn_car_state(v_kph=55.0, steering_angle=-93.0, right_blinker=True)
    assert self.classify(BLINKER_RIGHT, carstate, make_model_data()) == "lane_change"

  def test_straight_driving_below_classification_speed_is_a_lane_change(self):
    carstate = make_turn_car_state(v_kph=45.0, steering_angle=3.0, left_blinker=True)
    assert self.classify(BLINKER_LEFT, carstate, make_model_data(yaw_rate=0.01)) == "lane_change"


class TestTurnOnlyDesires:
  def setup_method(self):
    self.helper = DesireHelper()
    self.helper._update_params_periodic = lambda: None
    self.helper._process_sides = lambda car, model, radar: None
    self.helper.laneChangeNeedTorque = -1
    self.helper.laneChangeBsd = 0
    self.helper.laneLineCheck = 0

    for side in (self.helper.left, self.helper.right):
      side.lane_available = True
      side.edge_available = True
      side.dist_to_edge_far = 10.0
      side.lane_change_available_geom = True
      side.lane_change_available = True
      side.side_object_detected = False
      side.bsd_hold_counter = 0
      side.lane_line_info_mod = 0
      side.lane_line_info_edge_detect = False
      side.lane_change_available_released = False
      side.lane_available_trigger = False
      side.lane_appeared = False
      side.lane_exist_count.counter = 10

    self.carrot_man = SimpleNamespace(
      atcType="",
      carrotCmdIndex=0,
      carrotCmd="",
      carrotArg="",
    )

  def update(self, carstate, yaw_rate=0.0, turn_desire=False):
    return self.helper.update(
      carstate,
      make_model_data(yaw_rate, turn_desire),
      True,
      0.1,
      self.carrot_man,
      SimpleNamespace(),
    )

  def test_straight_blinker_never_requests_a_lane_change(self):
    for _ in range(100):
      desire = self.update(make_turn_car_state(v_kph=45.0, steering_angle=3.0, left_blinker=True))
      assert desire not in (log.Desire.laneChangeLeft, log.Desire.laneChangeRight)
      assert self.helper.lane_change_state == log.LaneChangeState.off

  def test_turn_desire_is_released_after_the_wheel_comes_back(self):
    # sharp left turn: turn desire is active
    for _ in range(5):
      desire = self.update(make_turn_car_state(v_kph=24.0, steering_angle=90.0, left_blinker=True), turn_desire=True)
    assert desire == log.Desire.turnLeft

    # wheel back under the release angle, but not long enough yet
    for _ in range(int(TURN_RELEASE_TIME / DT_MDL) - 1):
      desire = self.update(make_turn_car_state(v_kph=24.0, steering_angle=10.0, left_blinker=True), turn_desire=True)
    assert desire == log.Desire.turnLeft

    # one more frame latches the release
    desire = self.update(make_turn_car_state(v_kph=24.0, steering_angle=10.0, left_blinker=True), turn_desire=True)
    assert desire == log.Desire.none
    assert self.helper.turn_disable_count > 0

    # the latch holds while the blinker stays on, even if the driver steers again
    desire = self.update(make_turn_car_state(v_kph=24.0, steering_angle=90.0, left_blinker=True), turn_desire=True)
    assert desire == log.Desire.none
    assert self.helper.turn_disable_count > 0

    # blinker off clears the latch
    self.update(make_turn_car_state(v_kph=24.0, steering_angle=0.0), turn_desire=True)
    assert self.helper.turn_disable_count == 0
    assert not self.helper.turn_released
