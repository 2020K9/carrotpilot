"""K9 camera-SCC stop soft brake trial (K9StopSoftBrakeTime / Accel / Speed), default off.

Below the K9StopSoftBrakeSpeed setting in the stopping state, the sent accel becomes the strongest of -strength,
the planned aTarget and, with a detected lead, a closing-speed required decel, still clipped to the existing
vehicle accel limits. Continuous car standstill frames are counted from the car's
standstill signal; after the set time this trial requests the existing soft hold. Unapproved trial values,
not verified on a vehicle (including whether the weak deceleration keeps the car from rolling).
"""
import math

from opendbc.car.hyundai.values import CAR, HyundaiFlags

# Closing-speed required decel, numbers taken from the removed StopEase trial (reference, not a safety proof).
K9_SOFT_BRAKE_LEAD_GAP = 1.5  # m subtracted from the lead distance
K9_SOFT_BRAKE_NEED_GAIN = 2.0
K9_SOFT_BRAKE_MIN_GAP = 0.05  # m, lower bound of the remaining distance in the denominator
# Params: (name, default, min, max).
# Time x0.1 s in 0.5 s steps (0 = whole trial off).
K9_SOFT_BRAKE_TIME = ("K9StopSoftBrakeTime", 0, 0, 50)
# Strength x0.01 m/s^2. Provisional value without supporting evidence (근거 없는 임시값): default 0.2 m/s^2.
K9_SOFT_BRAKE_ACCEL = ("K9StopSoftBrakeAccel", 20, 5, 50)
# Entry speed in 1 km/h steps: 1-5 compare "v_ego < value / 3.6 m/s"; 0 means only after the car's standstill signal
# (no speed comparison). This is not the time setting 0, which turns the whole trial off.
# 간격 1km/h·최소 0·최대 5km/h·기본 2km/h는 형님 지정. 실차 안전성·효과 미검증.
K9_SOFT_BRAKE_SPEED = ("K9StopSoftBrakeSpeed", 2, 0, 5)
K9_SOFT_BRAKE_DT = 0.01  # card loop period


def k9_stop_soft_brake_applies(CP) -> bool:
  # getattr: some cruise-helper tests pass a partial CarParams without carFingerprint.
  return (getattr(CP, "carFingerprint", None) == CAR.KIA_K9 and not CP.flags & HyundaiFlags.CANFD and
          bool(CP.flags & HyundaiFlags.CAMERA_SCC) and CP.openpilotLongitudinalControl)


def k9_lead_need_accel(lead) -> float | None:
  """Required decel (<= 0) for a detected lead: -2 * closing^2 / (2 * max(dRel - 1.5, 0.05)), closing = max(0, -vRel).
  0.0 without a detected lead; None if a lead is reported but its distance or relative speed is invalid."""
  if not lead.status:
    return 0.0
  if not (math.isfinite(lead.dRel) and math.isfinite(lead.vRel) and lead.dRel >= 0.0):
    return None
  closing = max(0.0, -lead.vRel)
  return -K9_SOFT_BRAKE_NEED_GAIN * closing ** 2 / (2.0 * max(lead.dRel - K9_SOFT_BRAKE_LEAD_GAP, K9_SOFT_BRAKE_MIN_GAP))


def read_k9_int_param(params, name, default, lo, hi) -> int:
  """An integer is clamped to [lo, hi]; missing, unparsable or non-finite values give the default.
  Params.get() (not get_int, which aborts on non-numeric text) returns an int or None for INT keys."""
  value = params.get(name)
  if isinstance(value, (bytes, str)):
    try:
      value = int(value)
    except ValueError:
      return default
  if isinstance(value, float) and math.isfinite(value) and value.is_integer():
    value = int(value)
  if isinstance(value, bool) or not isinstance(value, int):
    return default
  return min(hi, max(lo, value))


def read_k9_stop_soft_brake(params) -> tuple[int, float]:
  """Returns (hold frames, 0 when off; soft brake accel in m/s^2, negative)."""
  time_steps = round(read_k9_int_param(params, *K9_SOFT_BRAKE_TIME) / 5) * 5  # 0.5 s steps
  return round(time_steps * 0.1 / K9_SOFT_BRAKE_DT), -read_k9_int_param(params, *K9_SOFT_BRAKE_ACCEL) * 0.01


def read_k9_stop_soft_brake_speed(params) -> float:
  """Entry speed in m/s: setting / 3.6 (0.0 = standstill only)."""
  return read_k9_int_param(params, *K9_SOFT_BRAKE_SPEED) / 3.6


class K9StopSoftBrake:
  """Per-frame decision. The caller owns the soft hold state; this only says when to set or clear it.

  - Soft brake only in the stopping state, below the entry speed or at standstill (entry speed 0: at standstill only), enabled, no gas/brake/override, valid
    speed/lead/plan inputs (a reported lead with invalid distance/relative speed is invalid, not "no lead"), no
    other soft hold, and while not latched off. The caller sends min(-strength, aTarget, lead need decel).
  - Hold frames count only continuous eligible standstill frames; any other frame resets them to 0.
  - Standstill ending after it was reported: original control until a new standstill or the entry speed or more
    (entry speed 0: until a new standstill).
  - The entry speed setting is fixed when a stop episode starts (first soft brake frame) and kept through the weak
    decel, the standstill wait and the hold; a changed setting applies from the next episode. The episode ends when
    no own hold is active and the car moves at or above the fixed entry speed without standstill (entry speed 0: as
    soon as standstill ends).
  - Pedal/override/disengage: off for this stop until the car moves (standstill ends). It also ends this
    trial's own hold request; a hold that was not requested here is never touched.
  """
  def __init__(self):
    self.frames = 0
    self.standstill_prev = False
    self.moved = False
    self.blocked = False
    self.hold = False
    self.engaged = False
    self.speed = None  # fixed entry speed (m/s) of the current episode

  def reset(self):
    self.__init__()

  def update(self, hold_frames, *, speed, enabled, driver, standstill, v_ego, inputs_valid, stopping,
             soft_hold_active, pedal=False):
    """speed: entry speed setting (m/s), used only when no episode is in progress. driver: gas, brake or override;
    pedal: gas or brake only. Returns (soft_brake, request_hold, clear_hold)."""
    if hold_frames <= 0:
      clear = self.hold and soft_hold_active > 0
      self.reset()
      return False, False, clear

    if not self.engaged:
      self.speed = speed
    clear = False
    if self.hold and (driver or not enabled or soft_hold_active == 0):
      # Gas/brake already reset the soft hold in the existing pedal logic (a long brake press may arm the
      # existing manual hold, which is kept); an override or disengage without pedal input is cleared here.
      clear = not pedal and soft_hold_active > 0
      self.hold = False
      self.blocked = True

    if self.standstill_prev and not standstill:
      self.moved = True
    # Entry speed 0 is decided by the standstill signal alone, never by a speed comparison.
    standstill_only = self.speed == 0.0
    above_entry = not standstill_only and v_ego >= self.speed
    below_entry = not standstill_only and v_ego < self.speed
    if standstill or above_entry:
      self.moved = False
    self.standstill_prev = standstill
    if not standstill:
      self.blocked = False
    if driver or not enabled:
      self.blocked = True

    eligible = (not self.hold and not self.blocked and not self.moved and enabled and not driver and inputs_valid and
                stopping and soft_hold_active == 0 and (standstill or below_entry))
    self.frames = self.frames + 1 if eligible and standstill else 0
    request = eligible and self.frames >= hold_frames
    if request:
      self.hold = True
      self.frames = 0
    if eligible or self.hold:
      self.engaged = True
    elif not standstill and (standstill_only or above_entry):
      self.engaged = False
    return eligible and not request, request, clear
