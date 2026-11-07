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
  - with the wheel speed known (controlsd passes carState.steeringRateDeg, 2026-11-07 fix): while he holds, the
    envelope may rise HOLD_RISE/s up to HOLD_MAX so openpilot still pushes back a little (rotary tracking); after he
    lets go it rises over T_FAST but only while the wheel is not already running the request's way (RATE_OK..
    RATE_HOLD deg/s slows and stops the rise), and a request in the other direction is never limited
  - 2026-11-08: carState.steeringRateDeg is unsigned on the K9 (SAS11 SAS_Speed), so controlsd also passes the
    steering angle (set_angle) and the rate gets its direction from it. While he holds, the push-back only grows
    while the wheel is still (HOLD_RUN0..HOLD_RUN1 deg/s the request's way); once the wheel runs the request's way
    faster than HOLD_RUN1 he is giving way and the push-back the hold allowance added falls (HOLD_FALL/s) back to
    the lowest output of the hold, so the wheel is not shoved when he lets go (17:54:05.3, 10.9, 13.0)
  - disarmed once the envelope reaches 1, the driver turns the way of the request (he asked for it), lateral
    control is off, or speed is above V_MAX
"""
import numpy as np

DT = 0.01
CLAMP_MIN = 40       # car torque units the driver limit cuts that count as the driver holding against
V_MAX = 15.0         # m/s, not used above (handback at speed stays as before)
T_MIN = 2.0          # s, shortest handback
HELP_TQ = 60.0       # driver torque in the request direction that counts as asking for it
RATE_OK = 60.0       # deg/s, wheel speed in the request direction below which the handback may go fast
RATE_HOLD = 120.0    # deg/s, wheel speed in the request direction above which the envelope stops rising
T_FAST = 1.0         # s, handback time while the wheel is not running away (rate known)
HOLD_MAX = 0.4       # highest envelope while the driver still holds against (rate known)
HOLD_RISE = 2.0      # 1/s, how fast the envelope may rise while the driver still holds against (rate known, wheel slow)
HOLD_RUN0 = 4.0      # deg/s, wheel speed in the request direction while he holds above which the push-back stops growing
HOLD_RUN1 = 40.0     # deg/s, ... above which he is giving way: the push-back falls to what the driver limit lets through
HOLD_FALL = 1.0      # 1/s, how fast the push-back falls once he gives way
ANGLE_EPS = 0.05     # deg, steering angle change that sets the wheel direction (carState.steeringRateDeg is unsigned)


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
    self.angle_ref = None
    self.angle_dir = 0.0

  def reset(self):
    self.armed = False
    self.cap = 1.0
    self.low = 1.0
    self.t = 0.0
    self.T = T_MIN
    self.limiting = False
    self.sign = 0.0
    self.hold_cap = 1.0
    self.follow = 1.0

  def set_angle(self, steering_angle_deg):
    """carState.steeringAngleDeg of this frame. On the K9 (SAS11 SAS_Speed, unsigned in the DBC) carState.steeringRateDeg
    is only the wheel speed, never negative (11-06 logs: 0..288, sign matches the angle motion 49 % of frames), so the
    direction is taken from the angle and steering_rate_deg is used as a magnitude once an angle has been given."""
    if steering_angle_deg is None or not np.isfinite(steering_angle_deg):
      return
    a = float(steering_angle_deg)
    if self.angle_ref is None:
      self.angle_ref = a
    elif abs(a - self.angle_ref) >= ANGLE_EPS:
      self.angle_dir = 1.0 if a > self.angle_ref else -1.0
      self.angle_ref = a

  def signed_rate(self, steering_rate_deg):
    if steering_rate_deg is None or not np.isfinite(steering_rate_deg):
      return None
    if self.angle_ref is None:
      return float(steering_rate_deg)  # caller gave a signed rate
    return abs(float(steering_rate_deg)) * self.angle_dir

  def update(self, request, applied_prev, driver_torque, steering_pressed, v_ego, lat_active, y_std_1s=None,
             steering_rate_deg=None):
    """request: actuators.torque (-1..1) before this stage; applied_prev: carOutput.actuatorsOutput.torque of the
    last frame; driver_torque: carState.steeringTorque. Returns the request to send."""
    steering_rate_deg = self.signed_rate(steering_rate_deg)
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
      self.sign = float(np.sign(request))
      self.hold_cap = self.cap
      self.follow = self.cap
    if not self.armed:
      return request

    self.T = handback_time(y_std_1s)
    mag = abs(request)
    if against:
      self.sign = float(np.sign(request))
      # the driver still holds against: follow what the car put out down at once, no rise until he lets go
      self.cap = min(self.cap, abs(applied_prev))
      self.follow = min(self.follow, abs(applied_prev))
      if HOLD_RISE > 0.0 and steering_rate_deg is not None and np.isfinite(steering_rate_deg):
        run = float(steering_rate_deg) * self.sign
        if run > HOLD_RUN1:
          # he is giving way while still on the wheel (17:54:04.7~05.4, 10.3~10.9, 12.8~13.0): the wheel already runs
          # the request's way, so pushing back now only adds to the spring when he lets go -> let the push-back fall
          # only the push-back the hold allowance added falls; what the car put out at the lowest point stays
          self.hold_cap = max(self.follow, min(self.hold_cap, self.cap) - HOLD_FALL * DT)
        else:
          w = float(np.interp(run, [HOLD_RUN0, HOLD_RUN1], [1.0, 0.0]))
          top = max(HOLD_MAX, self.cap)
          self.hold_cap = min(top, max(self.hold_cap, self.cap) + HOLD_RISE * DT * w) if w > 0.0 else min(self.hold_cap, top)
        self.cap = self.hold_cap
      else:
        self.hold_cap = self.cap
      self.low = self.cap
      self.t = 0.0
    elif steering_rate_deg is None or not np.isfinite(steering_rate_deg):
      # he let go: rise along the ease-in curve whatever openpilot's own request does
      self.t += DT
      self.cap = self.low + (1.0 - self.low) * min(1.0, self.t / self.T) ** 2
      if self.cap >= 1.0:
        self.reset()
        return request
    else:
      if self.t == 0.0:
        # just let go (or the wheel has been running away since): start from what the car actually put out, not
        # from the hold allowance
        self.low = min(self.low, abs(applied_prev))
      # he let go and the wheel speed is known: the spring is the wheel running the request's way, so rise fast while
      # the wheel stays slow, slow down as it speeds up and stop rising above RATE_HOLD (time does not count then)
      run = float(steering_rate_deg) * self.sign
      w = float(np.interp(run, [RATE_OK, RATE_HOLD], [1.0, 0.0]))
      T = min(self.T, T_FAST)
      self.t += DT * w
      self.cap = self.low + (1.0 - self.low) * min(1.0, self.t / T) ** 2
      if self.cap >= 1.0:
        self.reset()
        return request
    if self.sign != 0.0 and np.sign(request) != self.sign:
      # openpilot now asks the other way (the way the driver pushed): that is not the unwind, leave it alone
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
