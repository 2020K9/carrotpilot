"""K9 camera-SCC stop soft brake trial (K9StopSoftBrakeTime / Accel / Speed), default off.

Below the K9StopSoftBrakeSpeed setting in the stopping state, the sent accel becomes -strength alone, clipped to the
existing vehicle accel limits: a stronger planned aTarget or lead closing-speed decel is NOT applied (user's choice,
accepting a higher collision risk while closing on a lead). The lead decel is computed only to reject invalid radar. Continuous car standstill frames are counted from the car's standstill signal.
Unapproved trial values, not verified on a vehicle (including whether the weak deceleration keeps the car from rolling).

시간 설정(K9StopSoftBrakeTime)은 연속 정지 상태에서 약제동을 유지하는 시간이다. 시간이 지나면 약제동을
끝내고(만료) 원래 정차 제어로 돌아가며, 소프트홀드를 요청하지 않는다. 같은 정차 회차에서는 다시 켜지지 않는다.
"""
import math

from opendbc.car.hyundai.values import CAR, HyundaiFlags

# Closing-speed required decel, numbers taken from the removed StopEase trial (reference, not a safety proof).
# Only its None (invalid lead) result is used now; the value itself is no longer sent.
K9_SOFT_BRAKE_LEAD_GAP = 1.5  # m subtracted from the lead distance
K9_SOFT_BRAKE_NEED_GAIN = 2.0
K9_SOFT_BRAKE_MIN_GAP = 0.05  # m, lower bound of the remaining distance in the denominator
# Params: (name, default, min, max).
# Time x0.1 s in 0.5 s steps (0 = whole trial off).
K9_SOFT_BRAKE_TIME = ("K9StopSoftBrakeTime", 0, 0, 150)
# Strength x0.01 m/s^2. Provisional value without supporting evidence (근거 없는 임시값): default 0.2 m/s^2.
K9_SOFT_BRAKE_ACCEL = ("K9StopSoftBrakeAccel", 20, 0, 20)  # 0~20(칸 1), 0은 설정 제동만 0이고 기능 끔이 아님
# Entry speed in 1 km/h steps: 1-5 compare "v_ego < value / 3.6 m/s"; 0 means only after the car's standstill signal
# (no speed comparison). This is not the time setting 0, which turns the whole trial off.
# 간격 1km/h·최소 0·최대 5km/h·기본 2km/h는 형님 지정. 실차 안전성·효과 미검증.
K9_SOFT_BRAKE_SPEED = ("K9StopSoftBrakeSpeed", 2, 0, 5)
# 속도 설정 0의 회차 종료 속도(m/s): 기본 진입 속도 2km/h를 다시 쓴 오프라인 검증용 잠정 기준. 진입에는 쓰지 않으며,
# 실차 출발 검출이 검증됐다는 뜻은 아니다.
K9_SOFT_BRAKE_END_SPEED_ZERO = K9_SOFT_BRAKE_SPEED[1] / 3.6
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
  """Per-frame decision. 소프트홀드 상태는 호출한 쪽 것이며, 여기서는 읽기만 하고 켜거나 끄지 않는다.

  - Soft brake only in the stopping state, below the entry speed or at standstill (entry speed 0: at standstill only), enabled, no gas/brake/override, valid
    speed/lead/plan inputs (a reported lead with invalid distance/relative speed is invalid, not "no lead"), no
    other soft hold, and while not latched off. The caller sends -strength only.
  - Hold frames count only continuous eligible standstill frames; any other frame resets them to 0.
  - 만료: 연속 정지 프레임이 설정 시간에 이르는 그 프레임부터 약제동을 끝내고 원래 정차 제어로 돌아간다(소프트홀드
    요청 없음). 만료 표시는 이 정차 회차가 끝날 때까지 남아, 같은 회차에서는 다시 멈춰도 약제동이 켜지지 않는다.
  - Standstill ending after it was reported: original control until a new standstill or the entry speed or more
    (entry speed 0: until a new standstill).
  - The entry speed setting is fixed when a stop episode starts (first soft brake frame) and kept through the weak
    decel, the standstill wait and the expiry; a changed setting applies from the next episode.
  - 회차 종료: 멈춤 신호가 꺼지고, 유한하고 유효한 차속(inputs_valid)이 회차 종료 속도 이상일 때. 회차 종료 속도는
    회차에 고정한 진입 속도(속도 설정 1~5), 속도 설정 0이면 K9_SOFT_BRAKE_END_SPEED_ZERO(2km/h, 회차 종료에만 씀).
    회차 종료에서만 만료가 풀려 다음 회차가 기존 진입 조건으로 시작한다. 멈춤 신호 꺼짐만으로, 종료 속도보다 느린
    이동으로, 잘못된 입력으로는 회차가 끝나지 않고 만료가 남는다.
  - 페달·오버라이드·해제: 이 정차에서는 차가 움직일 때(멈춤 신호 꺼짐)까지 끔.
  - 시간 설정 0: 전부 끄고 초기화.
  """
  def __init__(self):
    self.frames = 0
    self.standstill_prev = False
    self.moved = False
    self.blocked = False
    self.expired = False  # 이 정차 회차의 약제동 시간이 지남
    self.engaged = False
    self.speed = None  # fixed entry speed (m/s) of the current episode

  def reset(self):
    self.__init__()

  def update(self, hold_frames, *, speed, enabled, driver, standstill, v_ego, inputs_valid, stopping, soft_hold_active):
    """speed: 진입 속도 설정(m/s), 진행 중인 회차가 없을 때만 쓴다. driver: 가속·브레이크 페달 또는 오버라이드.
    약제동을 거는 동안 True를 돌려준다."""
    if hold_frames <= 0:
      self.reset()
      return False

    if not self.engaged:
      self.speed = speed

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
    end_speed = K9_SOFT_BRAKE_END_SPEED_ZERO if standstill_only else self.speed
    episode_end = not standstill and inputs_valid and math.isfinite(v_ego) and v_ego >= end_speed
    if episode_end:
      self.expired = False

    eligible = (not self.expired and not self.blocked and not self.moved and enabled and not driver and inputs_valid and
                stopping and soft_hold_active == 0 and (standstill or below_entry))
    self.frames = self.frames + 1 if eligible and standstill else 0
    expire = eligible and self.frames >= hold_frames
    if expire:
      self.expired = True
      self.frames = 0
    if eligible or self.expired:
      self.engaged = True
    elif episode_end:
      self.engaged = False
    return eligible and not expire
