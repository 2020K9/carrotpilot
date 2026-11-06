"""K9 torque handback after the driver holds the wheel against openpilot (2026-11-06, 17:54 rotary).

Log 0000020e--7939eaee37 seg 13~16 (carrot-wip a082dc0d9): in a rotary at 4~6 m/s (lane prob 0~0.05) the driver
held the wheel against openpilot. The driver-torque limit in the car code cut the output to 0 while he held
(driver torque 254+ on a full command), and as soon as he eased off that limit opened 2 units per driver unit,
so the output went 0 -> full within ~0.75 s and the wheel unwound at 240 deg/s (17:54:05.6) and 288 deg/s
(17:54:10.95).

This puts a slow-rise / fast-fall envelope on the magnitude of the torque request (actuators.torque, -1..1).
It never raises the request, never changes its sign and never slows a decrease; the car code still applies its
driver, rate and max limits (and panda its safety model) to whatever comes out, so those limits are unchanged.

  - armed when the driver pushes against the request (steeringPressed, or a driver torque the driver limit cuts
    by more than CLAMP_MIN units)
  - while the driver holds against, the envelope follows the torque the car actually put out (carOutput, one
    frame old) down at once and does not rise
  - once he lets go it rises only along an ease-in curve from its last low point:
    CAP = low + (1 - low) * (t / T)^2, T from model uncertainty like the angle-control recovery in the car
    code (0.5~3.0 s by position yStd at 1 s), never below T_MIN
  - disarmed once the envelope reaches 1, the driver turns the way of the request (he asked for it), lateral
    control is off, or speed is above V_MAX
"""
import numpy as np

DT = 0.01
CLAMP_MIN = 40       # car torque units the driver limit cuts that count as the driver holding against
V_MAX = 15.0         # m/s, not used above (handback at speed stays as before)
T_MIN = 2.0          # s, shortest handback
HELP_TQ = 60.0       # driver torque in the request direction that counts as asking for it


def handback_time(y_std_1s):
  if y_std_1s is None or not np.isfinite(y_std_1s) or y_std_1s < 0.0:
    y_std_1s = 0.3
  return max(T_MIN, float(np.interp(y_std_1s, [0.1, 0.2, 0.3, 0.4], [0.5, 0.8, 1.5, 3.0])))


def driver_cut(request, driver_torque, limits):
  """Car torque units the driver-torque limit alone removes from `request` (-1..1); same formula as
  apply_driver_steer_torque_limits without the rate limits."""
  steer_max = limits.STEER_MAX
  new = request * steer_max
  driver_max = steer_max + (limits.STEER_DRIVER_ALLOWANCE + driver_torque * limits.STEER_DRIVER_FACTOR) * limits.STEER_DRIVER_MULTIPLIER
  driver_min = -steer_max + (-limits.STEER_DRIVER_ALLOWANCE + driver_torque * limits.STEER_DRIVER_FACTOR) * limits.STEER_DRIVER_MULTIPLIER
  hi = max(min(steer_max, driver_max), 0)
  lo = min(max(-steer_max, driver_min), 0)
  return abs(new) - abs(min(max(new, lo), hi))


class K9TorqueRecover:
  def __init__(self, limits):
    self.limits = limits  # CarControllerParams of the car (STEER_MAX and the driver-limit constants)
    self.reset()

  def reset(self):
    self.armed = False
    self.cap = 1.0
    self.low = 1.0
    self.t = 0.0
    self.T = T_MIN
    self.limiting = False

  def update(self, request, applied_prev, driver_torque, steering_pressed, v_ego, lat_active, y_std_1s=None):
    """request: actuators.torque (-1..1) before this stage; applied_prev: carOutput.actuatorsOutput.torque of the
    last frame; driver_torque: carState.steeringTorque. Returns the request to send."""
    self.limiting = False
    if not lat_active or v_ego > V_MAX:
      self.reset()
      return request

    against = request * driver_torque < 0.0 and (steering_pressed or driver_cut(request, driver_torque, self.limits) > CLAMP_MIN)
    helping = request * driver_torque > 0.0 and abs(driver_torque) > HELP_TQ
    if helping:
      self.reset()
      return request
    if against and not self.armed:
      self.armed = True
      self.cap = min(1.0, abs(applied_prev))
      self.low = self.cap
      self.t = 0.0
    if not self.armed:
      return request

    self.T = handback_time(y_std_1s)
    mag = abs(request)
    if against:
      # the driver still holds against: follow what the car put out down at once, no rise until he lets go
      self.cap = min(self.cap, abs(applied_prev))
      self.low = self.cap
      self.t = 0.0
    else:
      # he let go: rise along the ease-in curve whatever openpilot's own request does
      self.t += DT
      self.cap = self.low + (1.0 - self.low) * min(1.0, self.t / self.T) ** 2
      if self.cap >= 1.0:
        self.reset()
        return request
    if mag <= self.cap:
      return request
    self.limiting = True
    return float(np.sign(request) * self.cap)


def k9_torque_recover_for(CP):
  """K9TorqueRecover for KIA_K9 torque steering, else None (every other car unchanged)."""
  if str(CP.carFingerprint) != "KIA_K9" or str(CP.steerControlType) != "torque":
    return None
  from opendbc.car.hyundai.values import CarControllerParams
  return K9TorqueRecover(CarControllerParams(CP))
