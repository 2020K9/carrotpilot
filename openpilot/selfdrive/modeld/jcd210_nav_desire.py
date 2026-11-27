"""Jcd210 only: navigation guidance as a desire pulse repeated every second.

Jcd210 has no navigation input. It reads the same navigation command N16
gets (navInstructionCarrot.allManeuvers through navigation_input, fresh for
2 s, distance advanced by speed since receipt), so it does not depend on the
auto turn control setting. It is on exactly while N16 would get that command
(no distance limit of its own, like N16). Turns become turn desires; slight turns, ramps and forks become
keep desires (never a lane change desire). A desire that stays on reverses
the Jcd210 path, so it is sent as a one-frame pulse and again every second.
DesireHelper's own desire has priority, so one desire path feeds the model.
"""
import time

from openpilot.cereal import log
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.navigation.helpers import navigation_input

Desire = log.Desire
PULSE_FRAMES = ModelConstants.MODEL_RUN_FREQ  # one pulse per second
NAV_FRESH_S = 2.  # same freshness as N16 (MapInput)

# Valhalla command index (navigation_input) -> desire
NAV_DESIRES = {
  9: Desire.keepRight, 10: Desire.turnRight, 11: Desire.turnRight,  # slight / right / sharp right
  14: Desire.turnLeft, 15: Desire.turnLeft, 16: Desire.keepLeft,  # sharp left / left / slight left
  18: Desire.keepRight, 19: Desire.keepLeft,  # on ramp
  20: Desire.keepRight, 21: Desire.keepLeft,  # off ramp
  23: Desire.keepRight, 24: Desire.keepLeft,  # fork
}

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


def nav_desire(sm, speed, now=None):
  """Desire from the N16 navigation command, or none."""
  now = time.monotonic() if now is None else now
  if not (sm.valid['navInstructionCarrot'] and 0 <= now - sm.recv_time['navInstructionCarrot'] < NAV_FRESH_S):
    return Desire.none
  nav = navigation_input(sm['navInstructionCarrot'].allManeuvers,
                         max(0., speed) * (now - sm.recv_time['navInstructionCarrot']))
  commands = nav[:-1].nonzero()[0]
  if len(commands) == 0:
    return Desire.none
  return NAV_DESIRES.get(int(commands[0]), Desire.none)


class Jcd210Desire:
  def __init__(self):
    self.reset()

  def reset(self):
    self.current = Desire.none
    self.frames = 0

  def update(self, dh, sm, speed, now=None):
    desire = dh.desire if dh.desire != Desire.none else nav_desire(sm, speed, now)
    if desire == Desire.none:
      self.reset()
      return Desire.none
    if desire != self.current:
      self.current = desire
      self.frames = 0
    pulse = self.frames % PULSE_FRAMES == 0
    self.frames += 1
    return desire if pulse else Desire.none
