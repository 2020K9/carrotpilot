"""K9 camera-SCC terminal stop taper (candidate B), opt-in via the K9StopTaper param (default OFF).

OFF (or any other car) never calls update(): the sent accel and CAN are the original. CarController reads the
param every 0.5 s; turning it OFF calls reset(), so an active taper or restore returns to the original accel at
once, and turning it ON mid-stop does nothing until the car has moved at REARM_SPEED again.

Not established by this module or its tests (effect and safety are not vehicle-validated):
- the nod mechanism is a hypothesis: the taper assumes the rebound at wheel stop grows with the deceleration at
  that instant. In recorded K9 stops the sent aReq at StopReq ranged about -0.1 to -0.96 m/s^2;
- four recorded K9 op-long stops (observational, not a paired test) show braking falling off and returning after
  StopReq=1 with similar measured LONG_ACCEL rebounds for sent aReq -0.1 and -0.96. That suggests little influence
  of aReq there, but does not prove an ESC-internal sequence, that the ESC ignores aReq, or that this taper has
  zero effect. Simulated acceleration swings are not a measured pitch or comfort result.

What it changes: only the SENT accel (aReq), never StopReq or the LongControl state.
- taper: in the final TAPER_SPEED of an ordinary stop, braking is weakened at <= RELEASE_RATE toward a
  speed-scheduled floor; the output is never stronger than the original;
- restore: right after standstill the sent hold accel is still weaker than the original and is moved back to it
  at REAPPLY_RATE (|original - output| / REAPPLY_RATE, about 0.5 s or less for the floor). The standstill hold
  command IS modified for that time; StopReq and hold/soft-hold timing are not.

Precedence, checked every frame in BOTH taper and restore; any hit sends the original at once and ends the
episode (constants are model-derived, NOT K9-calibrated, not a safety proof):
- strong braking: original accel or planner aTarget <= STRONG_ACCEL;
- the original strengthening by more than STRENGTHEN_ABORT since taper entry;
- pedal/override/brake, inactive control, a long control state other than stopping/pid, invalid CAN or
  non-finite inputs;
- a lead at <= LEAD_MIN_GAP (including zero/negative distance) is treated as too close or invalid. A lead counts when
  the plan reports one (leadVisible) OR its radar distance is non-zero: controlsd takes leadVisible from the plan and
  the distance from the current radarState, which recorded K9 drives show disagreeing for up to ~60 ms;
- restore only: motion (not standstill, or |vEgo| > MOVING_AFTER_STOP) or an original no stronger than the output.
Taper only:
- lead floor: never weaker than closing^2 / (2 (d - LEAD_MIN_GAP)), a constant-decel kinematic heuristic from the
  HUD lead. It ignores actuator lag and lead deceleration and does NOT guarantee stopping LEAD_MIN_GAP short;
- progress: measured aEgo above -MIN_PROGRESS_DECEL for PROGRESS_TIME (e.g. a downhill grade cancelling the weaker
  floor), or TAPER_MAX_TIME, returns the original. v1 without this never stopped on 5-8 % grades in the simulator;
- distance budget: measured travel (integrated vEgo) since entry may exceed the entry command's flat-road stop
  distance v^2 / (2 |a|) by at most DISTANCE_MARGIN, then the original returns;
- leaving the window (vEgo >= TAPER_SPEED or an original accel >= 0) ends the episode; a new episode needs the
  vehicle to have moved at REARM_SPEED or faster, so the progress/time/distance limits cannot be restarted by
  oscillating around the window edge.
No stop-line distance exists in this fork, so the stop position is not corrected. MAX_EXTRA_DISTANCE is only the
bound asserted by the simplified simulator tests (flat and 5/8 % up/downhill; ideal actuator and first-order lag
0.05-0.7 s with 0-0.3 s dead time, not K9-measured); it is NOT a guaranteed real-vehicle stopping distance increase.
On a 12 % downhill the same model adds up to about 0.33 m.
"""
import math
from enum import IntEnum

from opendbc.car.hyundai.values import CAR, HyundaiFlags

DT = 0.01  # CarController.update period
TAPER_SPEED = 1.0  # m/s
END_DECEL = 0.3  # m/s^2, floor magnitude at v -> 0
FLOOR_SLOPE = 1.0  # 1/s, floor = -(END_DECEL + FLOOR_SLOPE * v)
STRONG_ACCEL = -1.5  # m/s^2
LEAD_MIN_GAP = 2.0  # m
RELEASE_RATE = 2.0  # m/s^3, weakening rate limit
REAPPLY_RATE = 2.0  # m/s^3, back to the original after standstill
REARM_SPEED = 1.5  # m/s
STRENGTHEN_ABORT = 0.3  # m/s^2
MIN_PROGRESS_DECEL = 0.15  # m/s^2, measured aEgo must stay below -this while tapering
PROGRESS_TIME = 0.2  # s
TAPER_MAX_TIME = 2.0  # s
DISTANCE_MARGIN = 0.12  # m
MOVING_AFTER_STOP = 0.1  # m/s, motion during restore returns the original immediately
MAX_EXTRA_DISTANCE = 0.25  # m, simulator test bound only, not a real-vehicle guarantee


class TaperPhase(IntEnum):
  idle = 0  # armed or waiting to be re-armed; original
  taper = 1
  restore = 2  # standstill reached, ramping back to the original
  done = 3  # episode finished or aborted; original until re-armed


def k9_stop_taper_applies(CP) -> bool:
  return (getattr(CP, "carFingerprint", None) == CAR.KIA_K9 and not CP.flags & HyundaiFlags.CANFD and
          bool(CP.flags & HyundaiFlags.CAMERA_SCC) and CP.openpilotLongitudinalControl)


def taper_floor(v_ego: float, lead_distance: float, lead_rel_speed: float, lead_visible: bool) -> float:
  floor = -(END_DECEL + FLOOR_SLOPE * max(v_ego, 0.0))
  if lead_visible and lead_distance > 0.0:
    closing = max(0.0, -lead_rel_speed)
    need = closing ** 2 / (2.0 * max(lead_distance - LEAD_MIN_GAP, 0.05))
    floor = min(floor, -need)
  return floor


class K9StopTaper:
  def __init__(self):
    self.reset()

  def reset(self):
    self.phase = TaperPhase.idle
    self.armed = False
    self.out = 0.0
    self.entry_accel = 0.0
    self.taper_time = 0.0
    self.no_progress_time = 0.0
    self.travel = 0.0
    self.budget = 0.0
    self.reason = "init"

  def _finish(self, accel, reason):
    self.phase = TaperPhase.done
    self.reason = reason
    self.out = accel
    return accel

  def update(self, *, accel: float, a_target: float, v_ego: float, a_ego: float, standstill: bool, long_active: bool,
             override: bool, gas: bool, brake: bool, can_valid: bool, stopping_or_pid: bool,
             lead_visible: bool, lead_distance: float, lead_rel_speed: float) -> float:
    values = (accel, a_target, v_ego, a_ego, lead_distance, lead_rel_speed)
    if not all(math.isfinite(x) for x in values):
      self.armed = False
      return self._finish(accel, "invalid")  # caller already clipped a finite accel; NaN stays original
    if not long_active or override or gas or brake or not can_valid or not stopping_or_pid:
      self.armed = False
      return self._finish(accel, "interlock")
    lead_visible = lead_visible or lead_distance != 0.0  # plan and radarState can disagree for a few frames
    lead_too_close = lead_visible and lead_distance <= LEAD_MIN_GAP

    if v_ego >= REARM_SPEED and not standstill:
      self.armed = True
      if self.phase == TaperPhase.done:
        self.phase = TaperPhase.idle

    if self.phase in (TaperPhase.taper, TaperPhase.restore):
      # shared precedence: the restore ramp must not delay any of these either
      reason = ("strong_braking" if accel <= STRONG_ACCEL or a_target <= STRONG_ACCEL else
                "strengthened" if accel < self.entry_accel - STRENGTHEN_ABORT else
                "lead_too_close" if lead_too_close else None)
      if reason is not None:
        self.armed = False
        return self._finish(accel, reason)

    if self.phase == TaperPhase.idle:
      if (self.armed and not standstill and 0.0 < v_ego < TAPER_SPEED and accel < 0.0 and
          accel > STRONG_ACCEL and a_target > STRONG_ACCEL and not lead_too_close):
        self.phase, self.reason, self.out, self.entry_accel = TaperPhase.taper, "taper", accel, accel
        self.taper_time = self.no_progress_time = self.travel = 0.0
        self.budget = v_ego * v_ego / (2.0 * -accel) + DISTANCE_MARGIN
      else:
        self.out = accel
        return accel

    if self.phase == TaperPhase.taper:
      if standstill or v_ego <= 0.0:
        self.phase, self.reason = TaperPhase.restore, "restore"
      elif v_ego >= TAPER_SPEED or accel >= 0.0:
        self.armed = False
        return self._finish(accel, "left_window")
      else:
        self.taper_time += DT
        self.travel += v_ego * DT
        if self.travel >= self.budget:
          self.armed = False
          return self._finish(accel, "distance_budget")
        self.no_progress_time = self.no_progress_time + DT if a_ego > -MIN_PROGRESS_DECEL else 0.0
        if self.no_progress_time >= PROGRESS_TIME - 1e-9 or self.taper_time >= TAPER_MAX_TIME - 1e-9:
          self.armed = False
          return self._finish(accel, "no_progress")
        target = max(accel, taper_floor(v_ego, lead_distance, lead_rel_speed, lead_visible))
        out = target if target <= self.out else min(target, self.out + RELEASE_RATE * DT)
        self.out = max(out, accel)  # never stronger than the original
        return self.out

    if self.phase == TaperPhase.restore:
      self.armed = False
      if not standstill or abs(v_ego) > MOVING_AFTER_STOP:
        return self._finish(accel, "moving")
      if accel >= self.out:
        return self._finish(accel, "release")
      self.out = max(accel, self.out - REAPPLY_RATE * DT)
      if self.out <= accel:
        return self._finish(accel, "restored")
      return self.out

    self.out = accel
    return accel
