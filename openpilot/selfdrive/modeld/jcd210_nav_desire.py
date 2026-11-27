"""Jcd210 only: navigation guidance as a desire pulse repeated every second.

Jcd210 has no navigation input. Carrot's ATC type (already limited by the
existing ATC distance settings) is turned into a Jcd210 desire. A desire that
stays on reverses the Jcd210 path, so the desire is sent as a one-frame pulse
and sent again every second while it lasts. DesireHelper's own desire (ATC
turn, lane change) has priority, so one desire path feeds the model.
"""
from openpilot.cereal import log
from openpilot.selfdrive.modeld.constants import ModelConstants

Desire = log.Desire
PULSE_FRAMES = ModelConstants.MODEL_RUN_FREQ  # one pulse per second

NAV_DESIRES = {
  'turn left': Desire.turnLeft,
  'turn right': Desire.turnRight,
  'atc left': Desire.keepLeft,
  'atc right': Desire.keepRight,
  'fork left': Desire.keepLeft,
  'fork right': Desire.keepRight,
}
LEFT_DESIRES = (Desire.turnLeft, Desire.keepLeft, Desire.laneChangeLeft)

_JCD210_SHA256: list[str | None] = []


def jcd210_sha256():
  if not _JCD210_SHA256:
    try:
      from openpilot.common.big_model_catalog import model_spec
      _JCD210_SHA256.append(model_spec('jcd210')['sha256'])
    except Exception:
      _JCD210_SHA256.append(None)
  return _JCD210_SHA256[0]


def is_jcd210(model):
  """True only while a connected Jetlink client runs the Jcd210 graph."""
  spec = getattr(model, 'spec', None)
  sha = jcd210_sha256()
  return getattr(model, 'client', None) is not None and sha is not None and getattr(spec, 'sha256', None) == sha


def nav_desire(dh, car_state, atc_type):
  desire = NAV_DESIRES.get(atc_type, Desire.none)
  # DesireHelper owns lane changes (including BSD waits) and driver conflicts.
  if desire == Desire.none or dh.lane_change_state != log.LaneChangeState.off or dh.atc_active == 2:
    return Desire.none
  if car_state.leftBlindspot if desire in LEFT_DESIRES else car_state.rightBlindspot:
    return Desire.none
  return desire


class Jcd210Desire:
  def __init__(self):
    self.reset()

  def reset(self):
    self.current = Desire.none
    self.frames = 0

  def update(self, dh, car_state, atc_type):
    desire = dh.desire if dh.desire != Desire.none else nav_desire(dh, car_state, atc_type)
    if desire == Desire.none:
      self.reset()
      return Desire.none
    if desire != self.current:
      self.current = desire
      self.frames = 0
    pulse = self.frames % PULSE_FRAMES == 0
    self.frames += 1
    return desire if pulse else Desire.none
