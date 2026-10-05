from openpilot.cereal import log
from openpilot.common.constants import CV

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection
TurnDirection = log.Desire

LANE_CHANGE_SPEED_MIN = 30 * CV.KPH_TO_MS
LANE_CHANGE_TIME_MAX = 10.0

BLINKER_NONE = 0
BLINKER_LEFT = 1
BLINKER_RIGHT = 2
BLINKER_BOTH = 3

# turn / lane change classification (maneuver_classifier)
TURN_CLASSIFY_SPEED_MAX_KPH = 50.0
TURN_ANGLE_LOW_DEG = 40.0
TURN_ANGLE_HIGH_DEG = 90.0
TURN_CURVATURE_MIN = 0.012
TURN_LOW_SPEED_KPH = 30.0
TURN_EDGE_FAR_MIN = 4.0
TURN_YAW_RATE_IDX = (8, 16)

# turn desire release latch
TURN_RELEASE_PEAK_ANGLE_DEG = 60.0
TURN_RELEASE_ANGLE_DEG = 30.0
TURN_RELEASE_TIME = 1.0

DESIRES = {
  LaneChangeDirection.none: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.none,
    LaneChangeState.laneChangeFinishing: log.Desire.none,
  },
  LaneChangeDirection.left: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeLeft,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeLeft,
  },
  LaneChangeDirection.right: {
    LaneChangeState.off: log.Desire.none,
    LaneChangeState.preLaneChange: log.Desire.none,
    LaneChangeState.laneChangeStarting: log.Desire.laneChangeRight,
    LaneChangeState.laneChangeFinishing: log.Desire.laneChangeRight,
  },
}

TURN_DESIRES = {
  TurnDirection.none: log.Desire.none,
  TurnDirection.turnLeft: log.Desire.turnLeft,
  TurnDirection.turnRight: log.Desire.turnRight,
}
