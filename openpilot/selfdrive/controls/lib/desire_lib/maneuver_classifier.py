from openpilot.common.constants import CV
from .constants import (
  BLINKER_LEFT,
  TURN_ANGLE_HIGH_DEG, TURN_ANGLE_LOW_DEG, TURN_CLASSIFY_SPEED_MAX_KPH,
  TURN_CURVATURE_MIN, TURN_EDGE_FAR_MIN, TURN_LOW_SPEED_KPH, TURN_YAW_RATE_IDX,
)

def classify_maneuver_type(blinker_state: int,
                           carstate,
                           side,                 # SideState
                           turn_desire_state: bool,
                           atc_type: str,
                           old_type: str,
                           modeldata=None):
  if blinker_state == 0:
    return "none"

  v_kph = carstate.vEgo * CV.MS_TO_KPH
  if v_kph >= TURN_CLASSIFY_SPEED_MAX_KPH:
    return "lane_change"

  # this fork: steeringAngleDeg is positive to the left, negative to the right
  sgn = 1.0 if blinker_state == BLINKER_LEFT else -1.0
  ang = carstate.steeringAngleDeg * sgn

  # geometry score: steering angle + model yaw rate into the blinker direction
  geo = 0
  if ang > TURN_ANGLE_LOW_DEG:
    geo += 1
  if ang > TURN_ANGLE_HIGH_DEG:
    geo += 1
  if modeldata is not None:
    i0, i1 = TURN_YAW_RATE_IDX
    yaw_rate = sum(modeldata.orientationRate.z[i] for i in range(i0, i1)) / (i1 - i0) * sgn
    if yaw_rate / max(carstate.vEgo, 3.0) > TURN_CURVATURE_MIN:
      geo += 1

  score = geo
  if v_kph < TURN_LOW_SPEED_KPH:
    score += 1
  if turn_desire_state:
    score += 1

  if atc_type in ("turn left", "turn right"):
    score += 2
  elif atc_type in ("fork left", "fork right", "atc left", "atc right"):
    score -= 2

  if score >= 2 and (geo >= 1 or side.dist_to_edge_far > TURN_EDGE_FAR_MIN):
    return "turn"
  return "lane_change"
