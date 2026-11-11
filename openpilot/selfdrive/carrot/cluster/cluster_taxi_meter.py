"""K9ExtHud=1 taxi meter panel (right information panel of the external HUD).

Two parts, kept apart on purpose:
  * TaxiMeter       pure fare/distance/time logic (no raylib), unit-tested.
  * TaxiMeterPanel  draws the 보노보노 모범택시 design into the 792x478 information
                    panel. All artwork comes from assets/taxi_meter/*.png (baked by
                    make_taxi_meter_assets.py from taxi_meter_design.html); swapping the
                    design = replace those PNGs / the layout constants below.
  * MeterAnimation  speed-linked animation phase (00:52 지시): characters and the sea move
                    on phase += dt * rate(speed) (0.35 stopped / 1.0 @30 / 2.2 @>=100 km/h,
                    0.5 s speed low-pass); stars and eye blinks stay on the wall clock.
                    In the time-fare zone (below 14.81 km/h onroad) 보노보노 taps the 조개 (waiting).

Fares: one table, TAXI_FARE_SIHEUNG. 시흥시 = 경기도 표준형 요금 (2023-07-01 04:00 적용,
2026-08 보도 기준 그대로 유지):
  - https://gnews.gg.go.kr/briefing/brief_gongbo_view.do?number=57519  (경기도 공식 보도: 모범·대형
    기본 7,000원/3km, 144m당 200원, 35초당 200원, 2023-07-01 04시부터)
  - https://m.mk.co.kr/amp/10745360  (매일경제 2023-05-25, 경기 택시요금 인상)
  - https://www.daehannoin.co.kr/news_view.jsp?ncd=3942
  - https://www.kihoilbo.co.kr/news/articleView.html?idxno=3033050  (2026-08, 경기 중형 4,800원 유지)
  - https://copyking.tistory.com/entry/전국-택시-할증-시간-및-요금-총정리
  - https://outfox.tistory.com/6
Night surcharge numbers come from a blog (https://m.blog.naver.com/simsimba89/223618430491):
출처 블로그, 확인 필요 (not confirmed against an official 경기도 notice). The 30% in the 경기도 notice
is the 일반(중형) rate; 공식 모범 할증률 미확인 -> 모범 20% (blog; 서울·인천 모범도 20%).
시계외 할증: none for 경기 모범 (not modelled).

Counting = real meter 시간·거리 부분 동시병산: 기준속도 = distance_unit_m / time_unit_s
(144 m / 35 s = 4.114 m/s = 14.81 km/h, computed from the two values). v >= 기준속도: driven
distance only; v < 기준속도 (stopped included): time only, converted to distance as dt * 144 / 35 m.
Both go into ONE 환산거리 counter: the base fare covers the first 3 km of it, then every 144 m of it
adds one unit (200원; night 240원). Speed = carState.vEgo (wheel speed), integrated sample by sample
at the received rate (live HUD: every carState, not the draw rate); gaps > 0.5 s are dropped.
Shown fare: rounded to 10원 (서울 요금표 rule; 240 / 8,400 are already round).
Night surcharge is applied on the device clock in Asia/Seoul (UTC+9, no DST; the device itself
may run on UTC): base fare with the rate at the drive start, every +step with the rate at the
moment it is charged (23:00 / 04:00 switch the unit price from then on, nothing already charged
is re-priced). A clock that is not synced yet (year < 2024) = no surcharge.
"""
from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import math
from pathlib import Path
import time

TAXI_METER_ASSET_DIR = Path(__file__).resolve().parent / "assets" / "taxi_meter"


@dataclass(frozen=True)
class TaxiFare:
  name: str
  base_fare_won: int
  base_distance_m: float
  distance_unit_m: float
  distance_unit_won: int
  time_unit_s: float
  time_unit_won: int
  # (start_hour, end_hour, rate) in local time; end exclusive; wraps midnight. 확인 필요.
  night_surcharges: tuple[tuple[int, int, float], ...]
  # 0 = 기준속도 from the two units (distance_unit_m / time_unit_s); > 0 = the notice's own speed, time fare at or
  # below it (경기 중형: "15km/h이하 주행 시", 경기도 공고 제2019-5429호, unchanged by 제2023-5572호)
  time_below_kph: float = 0.0
  out_of_area_rate: float = 0.0   # 시계외(사업구역외) 할증 - listed, not applied (the meter has no area boundary)

  @property
  def time_fare_below_mps(self) -> float:
    """기준속도: below it the meter counts time, at or above it distance (144 m / 35 s)."""
    if self.time_below_kph > 0.0:
      return self.time_below_kph / 3.6
    return self.distance_unit_m / self.time_unit_s

  @property
  def time_fare_below_kph(self) -> float:
    if self.time_below_kph > 0.0:
      return self.time_below_kph
    return self.time_fare_below_mps * 3.6


# 심야 할증 (모범): 공식 모범 할증률 미확인 -- 경기도 공식 보도의 30%는 일반(중형) 기준. 모범 20% =
# 출처 블로그, 확인 필요 (서울·인천 모범도 20%). Change it here only.
TAXI_NIGHT_RATE_DELUXE = 0.20
TAXI_NIGHT_HOURS = (23, 4)   # 23:00 ~ 04:00 (Asia/Seoul)

TAXI_FARE_SIHEUNG = {
  # 모범·대형 (meter default) -- 경기도 공식 보도 gnews.gg.go.kr number=57519 (2023-07-01 04시~)
  "deluxe": TaxiFare(
    name="보노 모범택시 요금",
    base_fare_won=7000, base_distance_m=3000.0,
    distance_unit_m=144.0, distance_unit_won=200,
    time_unit_s=35.0, time_unit_won=200,
    night_surcharges=((TAXI_NIGHT_HOURS[0], TAXI_NIGHT_HOURS[1], TAXI_NIGHT_RATE_DELUXE),),
  ),
  # 일반(중형), 표준형 15개 시 (시흥 포함) -- 경기도 공고 제2023-5572호 (2023-05-26 공고, 2023-07-01 04:00 시행)
  # 원문 PDF https://www.gg.go.kr/cmmn/download.do?idx=812608 : "기본요금(1.6㎞) : 4,800원", "거리요금 : 131m당 100원",
  # "시간요금 : 30초당 100원", "23:00～04:00 사이에 운행한 요금에 대하여 30% 할증", 시계외 "20% 할증".
  # 시간요금 속도: "15km/h이하 주행 시" (제2019-5429호; 2023 공고는 속도를 다시 적지 않음). (res 10/24 17:52)
  "midsize": TaxiFare(
    name="보노 일반택시 요금",
    base_fare_won=4800, base_distance_m=1600.0,
    distance_unit_m=131.0, distance_unit_won=100,
    time_unit_s=30.0, time_unit_won=100,
    night_surcharges=((23, 4, 0.30),),
    time_below_kph=15.0, out_of_area_rate=0.20,
  ),
}
# K9ExtHud=1 C1 meter = 일반(중형) (보노보노 10/24 17:52 '미터기 금액 일반택시로 변경'); the old bd09c076 panel keeps deluxe.
TAXI_METER_C1_FARE = TAXI_FARE_SIHEUNG["midsize"]
TAXI_METER_C1_FOOTER_TITLE = "보노 일반택시 요금"
TAXI_METER_C1_FOOTER_TEXT = "기본 4,800원 (1.6km) · 거리 131m당 100원 · 시간 30초당 100원 (15km/h 이하) · 심야 23~04시 30%"
TAXI_METER_DEFAULT_FARE = TAXI_FARE_SIHEUNG["deluxe"]
TAXI_METER_FOOTER_TITLE = "보노 모범택시 요금"
TAXI_METER_FOOTER_TEXT = (
  f"기본 7,000원(3km) · 144m당 200원 · 35초당 200원 · "
  f"시간·거리 동시병산({TAXI_FARE_SIHEUNG['deluxe'].time_fare_below_kph:.1f}km/h 미만 시간) · 심야 23~04시 20%"
)
TAXI_METER_MAX_DT_S = 0.5    # a longer gap between speed samples (frame / message stall) is dropped


SEOUL_TZ = timezone(timedelta(hours=9), "Asia/Seoul")   # KST, no daylight saving
TAXI_METER_CLOCK_MIN_YEAR = 2024                          # earlier = clock not synced -> no surcharge


def seoul_hour(epoch_s: float | None = None) -> int | None:
  """Asia/Seoul hour (0..23) for the night surcharge, None when the clock is not synced."""
  try:
    t = time.time() if epoch_s is None else float(epoch_s)
    local = datetime.fromtimestamp(t, SEOUL_TZ)
  except (OverflowError, OSError, ValueError, TypeError):
    return None
  if local.year < TAXI_METER_CLOCK_MIN_YEAR:
    return None
  return local.hour


def night_surcharge_rate(fare: TaxiFare, hour: int | None) -> float:
  """Night rate for a local (Asia/Seoul) hour; None (clock not synced) = 0."""
  if hour is None:
    return 0.0
  for start, end, rate in fare.night_surcharges:
    inside = start <= hour < end if start < end else (hour >= start or hour < end)
    if inside:
      return rate
  return 0.0


class TaxiMeter:
  """부분 동시 병산 meter.

  At or above 기준속도 (fare.time_fare_below_mps = 144 m / 35 s) the meter counts driven
  distance; below it (stopped included) it counts time, converted to the equivalent
  distance (time_unit_s <-> distance_unit_m). The base fare covers base_distance_m of that
  combined 환산거리; after it every distance_unit_m adds distance_unit_won. Each step is
  charged with the night rate active when it happens (the base fare with the rate at the
  start). Shown fare is rounded to 10 won.
  """

  def __init__(self, fare: TaxiFare = TAXI_METER_DEFAULT_FARE) -> None:
    self.fare = fare
    self.reset()

  def reset(self, hour: int | None = None) -> None:
    rate = night_surcharge_rate(self.fare, hour) if hour is not None else 0.0
    self.distance_m = 0.0
    self.elapsed_s = 0.0
    self.metered_m = 0.0  # distance + low-speed time as equivalent distance
    self.units = 0
    self.fare_won_exact = self.fare.base_fare_won * (1.0 + rate)
    self.started = False
    self.last_tick_s: float | None = None
    self._last_now: float | None = None
    self._last_sample_t: float | None = None
    self.mode: str | None = None   # "distance" / "time": what the last counted interval charged
    self.dropped_gaps = 0

  @property
  def fare_won(self) -> int:
    """Shown fare, rounded (half up) to 10 won."""
    return int(math.floor(self.fare_won_exact / 10.0 + 0.5 + 1e-9)) * 10

  @property
  def next_step_remaining_s(self) -> float:
    """Remaining 환산거리 to the next unit as waiting time (time fare)."""
    return self.next_step_remaining_m * self.fare.time_unit_s / self.fare.distance_unit_m

  def _units_for(self, metered_m: float) -> int:
    """Completed distance_unit_m steps after the base distance (charged when completed)."""
    beyond = metered_m - self.fare.base_distance_m
    return max(0, int(math.floor(beyond / self.fare.distance_unit_m + 1e-9)))

  @property
  def next_step_remaining_m(self) -> float:
    next_at = self.fare.base_distance_m + (self.units + 1) * self.fare.distance_unit_m
    return max(0.0, next_at - self.metered_m)

  @property
  def step_progress(self) -> float:
    """0..1 progress toward the next +step (the base part is part of the first step)."""
    unit = self.fare.distance_unit_m
    if self.units == 0:
      span = self.fare.base_distance_m + unit
      return min(1.0, max(0.0, self.metered_m / span))
    done = self.metered_m - self.fare.base_distance_m - self.units * unit
    return min(1.0, max(0.0, done / unit))

  def step(self, dt_s: float, speed_kph: float, hour: int | None = 12, now_s: float | None = None) -> int:
    """Advance by dt_s at speed_kph. A gap > TAXI_METER_MAX_DT_S (0.5 s) is dropped (not counted).
    Returns how many +steps were charged."""
    try:
      dt = float(dt_s)
    except (TypeError, ValueError):
      dt = 0.0
    if not math.isfinite(dt) or dt < 0.0:
      dt = 0.0
    if dt > TAXI_METER_MAX_DT_S + 1e-9:
      self.dropped_gaps += 1
      return 0
    self.started = True
    try:
      speed = float(speed_kph)
    except (TypeError, ValueError):
      speed = 0.0
    speed = max(0.0, speed) if math.isfinite(speed) else 0.0
    v = speed / 3.6
    self.elapsed_s += dt
    driven = v * dt
    self.distance_m += driven
    thr = self.fare.time_fare_below_kph
    if speed < thr or (self.fare.time_below_kph > 0.0 and speed <= thr):   # notice speed: "이하" = inclusive
      self.metered_m += dt * self.fare.distance_unit_m / self.fare.time_unit_s
      self.mode = "time"
    else:
      self.metered_m += driven
      self.mode = "distance"
    new_units = self._units_for(self.metered_m)
    added = new_units - self.units
    if added > 0:
      rate = night_surcharge_rate(self.fare, hour)
      self.fare_won_exact += added * self.fare.distance_unit_won * (1.0 + rate)
      self.last_tick_s = now_s
      self.units = new_units
      return added
    return 0

  def _start_if_rising(self, onroad: bool, speed_kph: float, hour: int | None, now_s) -> bool:
    """Onroad rising edge = a new drive from the base fare. Returns False while offroad."""
    if not onroad:
      self._last_now = None
      self._last_sample_t = None
      self._was_onroad = False
      return False
    if not getattr(self, "_was_onroad", False):
      self.reset(hour)
      self._was_onroad = True
      self.step(0.0, speed_kph, hour, now_s)
    return True

  def update(self, now_s: float, speed_kph: float, onroad: bool, hour: int | None) -> int:
    """Frame-rate driver (simulator / replay without speed samples): starts at the base fare when
    a drive starts (onroad rising edge), counts while onroad, holds the last value when offroad."""
    was = getattr(self, "_was_onroad", False)
    if not self._start_if_rising(onroad, speed_kph, hour, now_s):
      return 0
    if not was:
      self._last_now = now_s
      return 0
    dt = 0.0 if self._last_now is None else now_s - self._last_now
    self._last_now = now_s
    return self.step(dt, speed_kph, hour, now_s)

  def feed_sample(self, t_s: float, v_mps: float, hour: int | None, now_s: float | None = None) -> int:
    """One wheel-speed sample (carState.vEgo at its logMonoTime): counts the interval since the
    previous sample at this speed; the first sample and gaps > 0.5 s only set the time base."""
    prev, self._last_sample_t = self._last_sample_t, float(t_s)
    if prev is None:
      return 0
    try:
      v = float(v_mps)
    except (TypeError, ValueError):
      v = 0.0
    return self.step(float(t_s) - prev, v * 3.6, hour, now_s)

  def update_samples(self, samples, onroad: bool, hour: int | None, now_s: float | None = None) -> int:
    """Live driver: every carState sample since the last call, [(t_s, vEgo_mps), ...]."""
    first_v = samples[-1][1] * 3.6 if samples else 0.0
    if not self._start_if_rising(onroad, first_v, hour, now_s):
      return 0
    added = 0
    for t_s, v in samples:
      added += self.feed_sample(t_s, v, hour, now_s)
    return added


def format_won(value: int) -> str:
  return f"{int(value):,}"


def format_hms(seconds: float) -> str:
  s = int(max(0.0, seconds))
  return f"{s // 3600:02d}:{(s // 60) % 60:02d}:{s % 60:02d}"


# ---- animation (speed-linked phase; no raylib) ----------------------------------------------
# Character / sea motion runs on an accumulated phase, not on the wall clock:
#   phase += dt * rate(speed),  rate 0.35 (stopped: slow breathing only) -> 1.0 at 30 km/h
#   -> 2.2 at >= 100 km/h (cap), linear in between; speed low-passed with a 0.5 s time constant.
# Stars twinkle and eye blinks stay on the wall clock; the +200원 pop stays on the fare tick.
ANIM_RATE_POINTS = ((0.0, 0.35), (30.0, 1.0), (100.0, 2.2))
ANIM_SPEED_TAU_S = 0.5
ANIM_MAX_DT_S = 0.25          # longer frame gaps (stall, resume) count as 0.25 s
ANIM_MOVE_FULL_KPH = 12.0     # 포로리 runs fully from here; 0 km/h = breathing only
ANIM_MOVE_TAU_S = 0.5         # run <-> stand fade (on top of the speed filter: no pop when starting off)
ANIM_WAIT_TAU_S = 0.5         # fade between "driving" and "waiting" (time fare) poses
ANIM_WAIT_TAP_PERIOD = 0.8    # phase units per 조개 tap while waiting (2.3 s stopped, 1.2 s at 14.8 km/h)
ANIM_WAIT_TAP_LEN = 0.3       # phase units the tap stroke lasts
ANIM_WAVE_PERIOD = 10.0       # phase units per sea cycle (= the design's 10 s loop at rate 1.0)
PORORI_RUN_HZ = 0.8           # strides per phase unit: 0.8 /s at 30 km/h .. 1.76 /s at 100 km/h
                              # (the 60 fps browser design ran 2.4; the HUD draws at 5..30 fps)
ANIM_FPS_DEFAULT = 20.0       # frame-rate estimate before any frame interval was seen
ANIM_FPS_ALPHA = 0.1          # EMA weight of one frame interval
PORORI_BLINKS = (1.3, 6.1, 8.4)   # wall clock, 10 s loop, 0.14 s each (design)
BONO_BLINKS = (2.2, 6.8)          # 0.15 s each


def anim_rate(speed_kph: float) -> float:
  """Phase rate for a (filtered) speed: 0.35 / 1.0 @30 / 2.2 @>=100 km/h, linear between."""
  try:
    v = abs(float(speed_kph))
  except (TypeError, ValueError):
    return ANIM_RATE_POINTS[0][1]
  if not math.isfinite(v):
    return ANIM_RATE_POINTS[0][1]
  pts = ANIM_RATE_POINTS
  if v <= pts[0][0]:
    return pts[0][1]
  for (v0, r0), (v1, r1) in zip(pts, pts[1:]):
    if v <= v1:
      return r0 + (r1 - r0) * (v - v0) / (v1 - v0)
  return pts[-1][1]


class MeterAnimation:
  """Accumulated animation phase driven by the low-passed vehicle speed."""

  def __init__(self, wait_below_kph: float = TAXI_METER_DEFAULT_FARE.time_fare_below_kph) -> None:
    self.wait_below_kph = float(wait_below_kph)   # = 기준속도: below it the meter counts time
    self.reset()

  def reset(self) -> None:
    self.phase = 0.0
    self.speed_kph = 0.0      # low-passed |speed|
    self.wait = 0.0           # 0 driving .. 1 waiting (time fare zone while onroad), low-passed
    self.move_w = 0.0         # 0 standing .. 1 running, low-passed
    self.rate = anim_rate(0.0)
    self.fps = ANIM_FPS_DEFAULT  # measured draw rate (EMA), for anti-aliasing fast motion
    self._last_now: float | None = None

  def step(self, dt_s: float, speed_kph: float, onroad: bool = True) -> float:
    dt = float(dt_s)
    dt = min(max(dt, 0.0), ANIM_MAX_DT_S) if math.isfinite(dt) else 0.0
    try:
      v = abs(float(speed_kph))
    except (TypeError, ValueError):
      v = 0.0
    if not math.isfinite(v):
      v = self.speed_kph
    if dt > 0.0:
      self.fps += (min(60.0, max(1.0, 1.0 / dt)) - self.fps) * ANIM_FPS_ALPHA
      self.speed_kph += (v - self.speed_kph) * (1.0 - math.exp(-dt / ANIM_SPEED_TAU_S))
      target = 1.0 if (onroad and self.speed_kph < self.wait_below_kph) else 0.0
      self.wait += (target - self.wait) * (1.0 - math.exp(-dt / ANIM_WAIT_TAU_S))
      move_target = min(1.0, max(0.0, self.speed_kph / ANIM_MOVE_FULL_KPH))
      self.move_w += (move_target - self.move_w) * (1.0 - math.exp(-dt / ANIM_MOVE_TAU_S))
    self.rate = anim_rate(self.speed_kph)
    self.phase += dt * self.rate
    return self.phase

  def update(self, now_s: float, speed_kph: float, onroad: bool) -> float:
    dt = 0.0 if self._last_now is None else now_s - self._last_now
    self._last_now = now_s
    return self.step(dt, speed_kph, onroad)

  @property
  def move(self) -> float:
    """0 stopped (breathing only) .. 1 running (>= ANIM_MOVE_FULL_KPH), low-passed (continuous)."""
    return min(1.0, max(0.0, self.move_w))


@dataclass(frozen=True)
class TaxiMeterPose:
  porori_hop: float        # px up
  porori_rot: float        # deg
  porori_tail: float       # deg
  porori_leg: float        # px, + = left foot up, - = right foot up
  porori_arm: float        # deg
  porori_blink: bool
  bono_bob: float          # px down
  bono_rot: float          # deg
  bono_tap: float          # 0..1 arm/조개 stroke
  bono_wait_tap: float     # 0..1 part of the stroke that is the "waiting" tap
  bono_blink: bool
  wave_ph: float           # rad


def motion_visibility(freq_hz: float, fps: float) -> float:
  """Anti-alias for a periodic motion drawn at fps: full amplitude up to fps/4, fades to 0 at
  the Nyquist rate fps/2 (a faster motion sampled at fps would only flicker / run backwards)."""
  q = max(1e-6, float(fps)) / 4.0
  return min(1.0, max(0.0, 2.0 - float(freq_hz) / q))


def _blinking(t: float, starts: tuple[float, ...], length: float) -> bool:
  tt = t % 10.0
  return any(b < tt < b + length for b in starts)


def taxi_meter_pose(anim: MeterAnimation, t: float, tk: float) -> TaxiMeterPose:
  """Pure pose for one frame. anim = speed-linked phase, t = wall clock (blinks),
  tk = seconds since the last fare tick (hop / 딱!). Continuous in phase, speed and wait."""
  ph = anim.phase
  m = anim.move
  run = ph * 2 * math.pi * PORORI_RUN_HZ
  stride_hz = PORORI_RUN_HZ * anim.rate
  m_step = m * motion_visibility(stride_hz, anim.fps)        # legs / sway / arms (1 x stride)
  m_hop = m * motion_visibility(2.0 * stride_hz, anim.fps)   # |sin| hop (2 x stride)
  m_tail = m * motion_visibility(0.5 * stride_hz, anim.fps)
  breathe = math.sin(ph * 2 * math.pi / 1.6) * 1.2 * (1.0 - m)
  b = math.sin(tk / 0.8 * math.pi) if 0.0 <= tk < 0.8 else 0.0   # fare tick hop envelope
  run_hop = abs(math.sin(run)) * 9.0 * m_hop + breathe
  hop = run_hop * (1.0 - b) + 30.0 * b
  rot = math.sin(run) * 5.0 * m_step * (1.0 - b)
  arm = math.sin(run) * 18.0 * m_step * (1.0 - b) - 30.0 * b
  tail = math.sin(run * 0.5) * 8.0 * m_tail
  leg = math.sin(run) * 4.0 * m_step
  bob = math.sin(ph * 2 * math.pi / 2.5) * 4.5
  brot = math.sin(ph * 2 * math.pi / 5.0) * 3.2
  tick_tap = math.sin(tk / 0.45 * math.pi) if 0.0 <= tk < 0.45 else 0.0
  u = ph % ANIM_WAIT_TAP_PERIOD
  wait_tap = anim.wait * (math.sin(u / ANIM_WAIT_TAP_LEN * math.pi) if u < ANIM_WAIT_TAP_LEN else 0.0)
  return TaxiMeterPose(
    porori_hop=hop, porori_rot=rot, porori_tail=tail, porori_leg=leg, porori_arm=arm,
    porori_blink=_blinking(t, PORORI_BLINKS, 0.14),
    bono_bob=bob, bono_rot=brot, bono_tap=max(tick_tap, wait_tap), bono_wait_tap=wait_tap,
    bono_blink=_blinking(t, BONO_BLINKS, 0.15),
    wave_ph=ph * 2 * math.pi / ANIM_WAVE_PERIOD,
  )


# ---- drawing -------------------------------------------------------------------------------
# Card layout in card pixels (776x456, the design's coordinates).
CARD_W = 776.0
CARD_H = 456.0
CARD_OFFSET_X = 8.0   # inside the 792 px information panel
CARD_OFFSET_Y = 12.0
TRACK_X0 = 28.0
TRACK_X1 = 500.0
TRACK_Y = 326.0
GOLD = (233, 201, 128)
GOLD_SOFT = (191, 162, 101)
GOLD_LABEL = (168, 145, 95)
GOLD_BRIGHT = (255, 224, 138)
INK = (242, 234, 216)
INK_SOFT = (185, 167, 127)
FOOT_TITLE = (201, 164, 87)
FOOT_TEXT = (232, 220, 192)
STARS = tuple(
  (x, y, (i % 3) * 0.4 + 0.8, i)
  for i in range(34)
  for x, y in [((i * 137.5) % 760 + 8, (i * 71.3) % 250 + 70)]
  if not ((x > 540 and y < 250) or (20 < x < 520 and 90 < y < 215))
)
# sprite boxes (design units) and scales, must match make_taxi_meter_assets.py
PORORI_SCALE = 0.86
PORORI_BOX = (-86.0, -80.0)          # body sprite top-left (local 0,0 = anchor)
PORORI_TAIL_BOX = (-82.0, -66.0)
PORORI_TAIL_PIVOT = (-10.0, 14.0)
BONO_SCALE = 0.82
BONO_BOX = (-66.0, -80.0)
BONO_ARM_BOX = (4.0, 60.0)
BONO_ARM_PIVOT = (40.0, 80.0)
PORORI_SKIN = (236, 107, 125)
PORORI_LIGHT = (253, 233, 238)
PORORI_LINE = (91, 36, 48)
BONO_SKIN = (63, 158, 232)
EYE_DARK = (20, 20, 20)
# sea: (base y, amplitude, wavelength, phase multiplier, phase offset, top colour, bottom colour)
SEA_WAVES = (
  (398.0, 4.0, 260.0, 2.0, 0.0, (19, 48, 90, 242), (19, 48, 90, 242)),
  (406.0, 5.0, 190.0, -3.0, 1.0, (22, 50, 94, 255), (10, 23, 48, 255)),
  (428.0, 4.0, 150.0, 4.0, 2.0, (13, 34, 70, 230), (13, 34, 70, 230)),
)
SEA_STEP_PX = 8
NIGHT_LAMP_X = 300.0   # top row, between the brand text and the moon
NIGHT_LAMP_Y = 26.0
NIGHT_LAMP_W = 150.0
NIGHT_LAMP_H = 30.0
NIGHT_LAMP_RED = (240, 104, 76)
FARE_MODE_DISTANCE = (240, 214, 144)   # gold: 거리요금
FARE_MODE_TIME = (130, 200, 255)       # sky blue: 시간요금 (waiting / slow)
FOOTER_TEXT_SIZE = 14.0
FOOTER_TEXT_MAX_W = 776.0 - 24 - 20
SEA_CORNER_R = 18.0


def sea_wave_y(base: float, amp: float, length: float, ph: float, x: float) -> float:
  return base + amp * math.sin(x / length * 2 * math.pi + ph) + amp * 0.35 * math.sin(x / (length * 0.47) * 2 * math.pi - ph * 1.7)


def _sea_bottom(x: float) -> float:
  r = SEA_CORNER_R
  if x < r:
    return CARD_H - r + math.sqrt(max(0.0, r * r - (r - x) ** 2))
  if x > CARD_W - r:
    return CARD_H - r + math.sqrt(max(0.0, r * r - (x - (CARD_W - r)) ** 2))
  return CARD_H


SEA_XS = tuple(float(x) for x in range(0, int(CARD_W) + 1, SEA_STEP_PX))
SEA_BOTTOMS = tuple(_sea_bottom(x) for x in SEA_XS)


def _rot(px: float, py: float, deg: float) -> tuple[float, float]:
  a = math.radians(deg)
  c, s = math.cos(a), math.sin(a)
  return px * c - py * s, px * s + py * c


class TaxiMeterPanel:
  """Draws the meter. Static parts (card art + fixed labels + fare footer) are
  cached once in a render texture; per frame only numbers, the progress fill,
  stars, the two characters and the sea are drawn."""

  def __init__(self, meter: TaxiMeter | None = None, clock=time.monotonic, local_hour=None) -> None:
    self.meter = meter or TaxiMeter()
    self.anim = MeterAnimation(self.meter.fare.time_fare_below_kph)
    self._speed_samples: list[tuple[float, float]] = []
    self._speed_samples_live = False   # True once a live carState feed exists (frame path off)
    self._clock = clock
    self._local_hour = local_hour or seoul_hour   # Asia/Seoul hour, None = clock not synced
    self.night_rate = 0.0                          # surcharge in force now (lamp, +step amount)
    self._textures: dict[str, object] = {}
    self._textures_loaded = False
    self._static_target = None
    self._static_key = None
    self.freeze_animation_t: float | None = None  # PNG review only (wall clock + no fare tick)
    self._sea_points = None
    self._footer_size: float | None = None

  # -- resources
  def _load_textures(self, rl) -> None:
    if self._textures_loaded:
      return
    self._textures_loaded = True
    for name in ("bg", "porori", "porori_tail", "bonobono", "bonobono_arm"):
      path = TAXI_METER_ASSET_DIR / f"taxi_meter_{name}.png"
      if not path.exists():
        continue
      try:
        texture = rl.load_texture(str(path))
        if texture.id > 0:
          rl.set_texture_filter(texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
          # clamp: rotated sprites must not sample the opposite edge (REPEAT is raylib's default)
          rl.set_texture_wrap(texture, rl.TextureWrap.TEXTURE_WRAP_CLAMP)
          self._textures[name] = texture
      except Exception as exc:
        print(f"taxi meter asset load failed for {path.name}: {exc}", flush=True)

  def close(self, rl) -> None:
    for texture in self._textures.values():
      try:
        rl.unload_texture(texture)
      except Exception:
        pass
    self._textures.clear()
    self._textures_loaded = False
    if self._static_target is not None:
      try:
        rl.unload_render_texture(self._static_target)
      except Exception:
        pass
    self._static_target = None
    self._static_key = None

  def restart(self) -> None:
    self._speed_samples = []
    """K9ExtHud changed (non-1 -> 1, and 1 -> non-1 so nothing stale is kept): fare, distance and
    time back to 0 / the base fare; the next onroad frame starts a fresh drive from the base fare."""
    self.meter.reset()
    self.meter._was_onroad = False
    self.anim.reset()

  # -- per frame state
  def update(self, state) -> None:
    speed = float(getattr(state, "speed_kph", 0.0) or 0.0)
    onroad = bool(getattr(state, "onroad", False))
    now = self._clock()
    hour = self._local_hour()
    self.night_rate = night_surcharge_rate(self.meter.fare, hour)
    if self._speed_samples_live:
      samples, self._speed_samples = self._speed_samples, []
      self.meter.update_samples(samples, onroad, hour, now)
    else:
      self.meter.update(now, speed, onroad, hour)
    self.anim.update(now, speed, onroad)

  def feed_speed_samples(self, samples) -> None:
    """Live HUD: every carState since the last frame as [(logMonoTime_s, vEgo_mps), ...].
    From the first call on, the meter integrates these instead of the frame-rate speed."""
    self._speed_samples_live = True
    self._speed_samples.extend(samples)
    if len(self._speed_samples) > 2000:          # ~20 s of carState without a frame: keep the tail
      self._speed_samples = self._speed_samples[-2000:]

  @property
  def step_won(self) -> int:
    """Amount of the next +step at the rate in force now (200 day / 240 night)."""
    return int(round(self.meter.fare.distance_unit_won * (1.0 + self.night_rate)))

  def prepare_static_layer(self, renderer, rl) -> None:
    """Call outside any active render target (like the trip report cache)."""
    self._load_textures(rl)
    key = (id(renderer._font), id(getattr(renderer, "_korean_font", None)))
    if self._static_target is not None and self._static_key == key:
      return
    if self._static_target is None:
      self._static_target = rl.load_render_texture(int(CARD_W), int(CARD_H))
      rl.set_texture_filter(self._static_target.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
    rl.begin_texture_mode(self._static_target)
    try:
      rl.clear_background(rl.Color(0, 0, 0, 0))
      self._draw_static(renderer, rl, 0.0, 0.0)
    finally:
      rl.end_texture_mode()
    self._static_key = key

  def _draw_static(self, renderer, rl, x: float, y: float) -> None:
    bg = self._textures.get("bg")
    if bg is not None:
      rl.draw_texture(bg, int(x), int(y), rl.WHITE)
    else:
      rl.draw_rectangle_rounded(rl.Rectangle(x, y, CARD_W, CARD_H), 0.08, 12, rl.Color(9, 12, 22, 255))
      rl.draw_rectangle_rounded_lines_ex(rl.Rectangle(x, y, CARD_W, CARD_H), 0.08, 12, 1.0, rl.Color(109, 90, 46, 255))
    text = renderer._draw_text
    # brand
    rl.draw_circle(int(x + 47), int(y + 41), 21.0, rl.Color(26, 22, 14, 255))
    rl.draw_circle_lines(int(x + 47), int(y + 41), 21.0, rl.Color(201, 164, 87, 255))
    text("모범", x + 47, y + 41, 14, (227, 194, 122), anchor="center")
    text("D E L U X E   T A X I", x + 80, y + 30, 12, GOLD_SOFT)
    text("보노보노 모범택시", x + 80, y + 53, 19, (239, 230, 210))
    # labels
    text("운 행  요 금", x + 28, y + 92, 14, GOLD_LABEL)
    text("운 행  거 리", x + 560, y + 92, 12, GOLD_LABEL)
    rl.draw_line_ex(rl.Vector2(x + 560, y + 152), rl.Vector2(x + 750, y + 152), 1.0, rl.Color(42, 38, 24, 255))
    text("운 행  시 간", x + 560, y + 168, 12, GOLD_LABEL)

  def _draw_static_cached(self, renderer, rl, x: float, y: float) -> None:
    target = self._static_target
    if target is None:
      self._draw_static(renderer, rl, x, y)
      return
    source = rl.Rectangle(0.0, 0.0, float(target.texture.width), -float(target.texture.height))
    rl.draw_texture_pro(target.texture, source, rl.Rectangle(x, y, CARD_W, CARD_H), rl.Vector2(0, 0), 0.0, rl.WHITE)

  def _part(self, rl, name: str, anchor: tuple[float, float], body_rot: float, scale: float,
            box: tuple[float, float], pivot: tuple[float, float], offset: tuple[float, float] = (0.0, 0.0),
            part_rot: float = 0.0) -> None:
    """Sprite part of a character: character transform = translate(anchor) rotate(body_rot) scale(scale);
    the part itself is offset in local units and rotated by part_rot around its local pivot."""
    texture = self._textures.get(name)
    if texture is None:
      return
    lx, ly = _rot((pivot[0] + offset[0]) * scale, (pivot[1] + offset[1]) * scale, body_rot)
    w, h = float(texture.width), float(texture.height)
    rl.draw_texture_pro(texture, rl.Rectangle(0, 0, w, h), rl.Rectangle(anchor[0] + lx, anchor[1] + ly, w, h),
                        rl.Vector2((pivot[0] - box[0]) * scale, (pivot[1] - box[1]) * scale), body_rot + part_rot, rl.WHITE)

  @staticmethod
  def _local(anchor: tuple[float, float], rot: float, scale: float, px: float, py: float) -> tuple[float, float]:
    lx, ly = _rot(px * scale, py * scale, rot)
    return anchor[0] + lx, anchor[1] + ly

  def _closed_eye(self, rl, anchor, rot, scale, ex, ey, r, skin) -> None:
    cx, cy = self._local(anchor, rot, scale, ex, ey)
    rl.draw_circle_v(rl.Vector2(cx, cy), (r + 0.9) * scale, rl.Color(*skin, 255))
    a = self._local(anchor, rot, scale, ex - r, ey)
    b = self._local(anchor, rot, scale, ex + r, ey)
    rl.draw_line_ex(rl.Vector2(*a), rl.Vector2(*b), max(1.2, 0.34 * r * scale), rl.Color(*EYE_DARK, 255))

  def _draw_porori(self, rl, anchor: tuple[float, float], pose: TaxiMeterPose) -> None:
    s, rot = PORORI_SCALE, pose.porori_rot
    self._part(rl, "porori_tail", anchor, rot, s, PORORI_TAIL_BOX, PORORI_TAIL_PIVOT, part_rot=pose.porori_tail)
    for fx, lift in ((-9.0, max(0.0, pose.porori_leg)), (9.0, max(0.0, -pose.porori_leg))):
      cx, cy = self._local(anchor, rot, s, fx, 46.0 - lift)
      rl.draw_ellipse(int(round(cx)), int(round(cy)), 8.9 * s, 4.9 * s, rl.Color(*PORORI_LINE, 255))
      rl.draw_ellipse(int(round(cx)), int(round(cy)), 7.1 * s, 3.1 * s, rl.Color(*PORORI_LIGHT, 255))
    self._part(rl, "porori", anchor, rot, s, PORORI_BOX, (0.0, 0.0))
    for sx, sign in ((-14.0, 1.0), (14.0, -1.0)):
      ex, ey = _rot(sx * 1.5 - sx, 14.0, sign * pose.porori_arm)   # shoulder (sx, 6) -> hand (sx*1.5, 20)
      a = self._local(anchor, rot, s, sx, 6.0)
      b = self._local(anchor, rot, s, sx + ex, 6.0 + ey)
      rl.draw_line_ex(rl.Vector2(*a), rl.Vector2(*b), 2.0 * s, rl.Color(*PORORI_LINE, 255))
      rl.draw_circle_v(rl.Vector2(*b), 1.0 * s, rl.Color(*PORORI_LINE, 255))
    if pose.porori_blink:
      for ex in (-14.0, 14.0):
        self._closed_eye(rl, anchor, rot, s, ex, -34.0, 2.6, PORORI_SKIN)

  def _draw_bonobono(self, rl, anchor: tuple[float, float], pose: TaxiMeterPose) -> None:
    s, rot = BONO_SCALE, pose.bono_rot
    self._part(rl, "bonobono", anchor, rot, s, BONO_BOX, (0.0, 0.0))
    self._part(rl, "bonobono_arm", anchor, rot, s, BONO_ARM_BOX, BONO_ARM_PIVOT,
               offset=(0.0, -pose.bono_tap * 16.0), part_rot=-pose.bono_tap * 14.0)
    if pose.bono_blink:
      for ex in (-42.0, 42.0):
        self._closed_eye(rl, anchor, rot, s, ex, -22.0, 4.6, BONO_SKIN)

  def _draw_sea(self, rl, x: float, y: float, wave_ph: float) -> None:
    """Three live waves (phase-driven). One triangle strip per wave (top/bottom vertex pairs);
    the middle wave's vertical gradient is a band below its lowest crest (w3 covers the rest)."""
    xs, bottoms = SEA_XS, SEA_BOTTOMS
    n = len(xs)
    strip = getattr(getattr(rl, "rl", None), "DrawTriangleStrip", None)
    if strip is not None and self._sea_points is None:
      self._sea_points = rl.ffi.new("struct Vector2[]", 2 * n)
    pts = self._sea_points
    for base, amp, length, mul, off, top_c, bot_c in SEA_WAVES:
      ph = wave_ph * mul + off
      ys = [sea_wave_y(base, amp, length, ph, wx) for wx in xs]
      gradient = top_c != bot_c
      top = base - amp * 1.35
      span = CARD_H - top

      def col(py, top_c=top_c, bot_c=bot_c, top=top, span=span):
        f = min(1.0, max(0.0, (py - top) / span))
        return tuple(int(round(a + (b - a) * f)) for a, b in zip(top_c, bot_c))

      solid = col(base + amp * 1.35 + 0.5) if gradient else top_c   # = the band's top colour (no seam)
      if strip is not None and pts is not None:
        for k in range(n):
          p0 = pts[2 * k]
          p0.x = x + xs[k]
          p0.y = y + ys[k]
          p1 = pts[2 * k + 1]
          p1.x = x + xs[k]
          p1.y = y + bottoms[k]
        strip(pts, 2 * n, rl.Color(*solid))   # T0,B0,T1,B1,... = the quad winding raylib draws
      else:
        rl.rl_check_render_batch_limit(6 * n)
        rl.rl_begin(rl.RL_TRIANGLES)
        rl.rl_color4ub(*solid)
        for k in range(n - 1):
          x0, x1 = x + xs[k], x + xs[k + 1]
          t0, t1 = y + ys[k], y + ys[k + 1]
          b0, b1 = y + bottoms[k], y + bottoms[k + 1]
          for vx, vy in ((x0, t0), (x0, b0), (x1, b1), (x0, t0), (x1, b1), (x1, t1)):
            rl.rl_vertex2f(vx, vy)
        rl.rl_end()
      if gradient:
        g0 = base + amp * 1.35 + 0.5           # below every crest of this wave
        g1 = CARD_H - SEA_CORNER_R              # above the rounded corners
        rl.draw_rectangle_gradient_v(int(round(x)), int(round(y + g0)), int(CARD_W), int(round(g1 - g0)),
                                     rl.Color(*col(g0)), rl.Color(*col(g1)))

  def _draw_fare_mode_badge(self, renderer, rl, bx: float, cy: float, time_mode: bool) -> float:
    """'거리요금' / '시간요금' badge: which counter runs now. Returns the x after it."""
    label = "시간요금" if time_mode else "거리요금"
    color = FARE_MODE_TIME if time_mode else FARE_MODE_DISTANCE
    tw, _ = renderer._measure_text(label, 13)
    box = rl.Rectangle(bx, cy - 11, tw + 18, 22)
    rl.draw_rectangle_rounded(box, 1.0, 12, rl.Color(*color, 46))
    rl.draw_rectangle_rounded_lines_ex(box, 1.0, 12, 1.2, rl.Color(*color, 255))
    renderer._draw_text(label, bx + 9, cy, 13, color)
    return bx + tw + 18 + 10

  def _draw_night_lamp(self, renderer, rl, lx: float, ly: float, t: float) -> None:
    """'심야 할증 20%' lamp (lit while the night surcharge is in force)."""
    box = rl.Rectangle(lx, ly, NIGHT_LAMP_W, NIGHT_LAMP_H)
    rl.draw_rectangle_rounded(box, 1.0, 16, rl.Color(58, 16, 12, 235))
    rl.draw_rectangle_rounded_lines_ex(box, 1.0, 16, 1.4, rl.Color(*NIGHT_LAMP_RED, 255))
    glow = int(70 + 40 * (0.5 + 0.5 * math.sin(t * 2.2)))
    cx, cy = lx + 20, ly + NIGHT_LAMP_H / 2
    rl.draw_circle_v(rl.Vector2(cx, cy), 9.0, rl.Color(*NIGHT_LAMP_RED, glow))
    rl.draw_circle_v(rl.Vector2(cx, cy), 4.8, rl.Color(255, 138, 96, 255))
    pct = int(round(self.night_rate * 100))
    renderer._draw_text(f"심야 할증 {pct}%", lx + 34, cy, 15, (255, 196, 160))

  def draw(self, renderer, rl, panel_x: float, panel_y: float) -> None:
    self._load_textures(rl)
    x = panel_x + CARD_OFFSET_X
    y = panel_y + CARD_OFFSET_Y
    meter = self.meter
    t = self.freeze_animation_t if self.freeze_animation_t is not None else self._clock()
    tk = 9.0 if meter.last_tick_s is None or self.freeze_animation_t is not None else max(0.0, t - meter.last_tick_s)
    pose = taxi_meter_pose(self.anim, t, tk)
    text = renderer._draw_text
    self._draw_static_cached(renderer, rl, x, y)

    # stars (twinkle, wall clock)
    for sx, sy, r, i in STARS:
      alpha = int(255 * (0.35 + 0.6 * abs(math.sin(t * 1.3 + i * 1.7))))
      rl.draw_circle_v(rl.Vector2(x + sx, y + sy), r, rl.Color(255, 244, 207, alpha))

    # status pill
    onroad = bool(getattr(meter, "_was_onroad", False))
    pill = rl.Rectangle(x + 776 - 26 - 92, y + 24, 92, 34)
    rl.draw_rectangle_rounded_lines_ex(pill, 1.0, 16, 1.2, rl.Color(201, 164, 87, 255))
    rl.draw_circle_v(rl.Vector2(pill.x + 22, pill.y + 17), 4.5, rl.Color(243, 207, 107, 255) if onroad else rl.Color(110, 98, 70, 255))
    text("주행" if onroad else "대기", pill.x + 36, pill.y + 17, 16, (240, 214, 144))
    if self.night_rate > 0.0:
      self._draw_night_lamp(renderer, rl, x + NIGHT_LAMP_X, y + NIGHT_LAMP_Y, t)

    # fare
    fare_text = format_won(meter.fare_won)
    pop = 1.0 + 0.09 * math.sin(tk / 0.5 * math.pi) if tk < 0.5 else 1.0
    size = 98.0 * pop
    fw, _ = renderer._measure_text(fare_text, size)
    text(fare_text, x + 24, y + 152, size, GOLD)
    text("원", x + 24 + fw + 10, y + 176, 30, (217, 189, 124))
    if tk < 1.2:
      alpha = int(255 * (1.0 - tk / 1.2))
      text(f"+{format_won(self.step_won)}원", x + max(372.0, 24 + fw + 60), y + 120 - tk * 40, 30, (*GOLD_BRIGHT, alpha))

    # stats
    km = f"{meter.distance_m / 1000.0:.2f}"
    kw, _ = renderer._measure_text(km, 27)
    text(km, x + 560, y + 124, 27, INK)
    text("km", x + 560 + kw + 6, y + 128, 15, INK_SOFT)
    text(format_hms(meter.elapsed_s), x + 560, y + 200, 27, INK)

    # progress track + 포로리
    p = meter.step_progress
    px = TRACK_X0 + (TRACK_X1 - TRACK_X0) * p
    if px > TRACK_X0 + 1:
      rl.draw_rectangle_gradient_h(int(x + TRACK_X0), int(y + TRACK_Y - 2), int(px - TRACK_X0), 4,
                                   rl.Color(122, 95, 39, 255), rl.Color(241, 214, 143, 255))
    pxp = max(TRACK_X0 + 50, px)
    self._draw_porori(rl, (x + pxp, y + 284 - pose.porori_hop), pose)
    nx = x + 28
    time_mode = onroad and meter.mode == "time"
    if onroad and meter.mode is not None:
      nx = self._draw_fare_mode_badge(renderer, rl, nx, y + 350, time_mode)
    if time_mode:
      remaining, unit = max(1, int(math.ceil(meter.next_step_remaining_s - 1e-6))), "초"
    else:
      remaining, unit = max(1, int(round(meter.next_step_remaining_m))), " m"
    for part, color, sz in (("다음 ", GOLD_LABEL, 14), (f"+{format_won(self.step_won)}원", (240, 214, 144), 14),
                            ("까지 ", GOLD_LABEL, 14), (f"{remaining}", (240, 214, 144), 14), (unit, GOLD_LABEL, 14)):
      text(part, nx, y + 350, sz, color)
      nx += renderer._measure_text(part, sz)[0]

    # 보노보노 (floats on the phase; taps the 조개 on a fare tick and while waiting in the time-fare zone)
    bono = (x + 668, y + 306 + pose.bono_bob)
    self._draw_bonobono(rl, bono, pose)
    if tk < 0.7:
      text("딱!", x + 716, y + 330 - tk * 30, 22, (*GOLD_BRIGHT, int(255 * (1 - tk / 0.7))))
    elif pose.bono_wait_tap > 0.3:
      a = int(255 * min(1.0, (pose.bono_wait_tap - 0.3) / 0.7) * 0.85)
      text("톡", x + 726, y + 344 - pose.bono_wait_tap * 8, 15, (*GOLD_BRIGHT, a))

    # sea over the characters (flows on the phase), then the fare footer
    self._draw_sea(rl, x, y, pose.wave_ph)
    text(TAXI_METER_FOOTER_TITLE, x + 24, y + 410, 12, FOOT_TITLE)
    if self._footer_size is None:                 # fit the fare line into the card once
      w, _ = renderer._measure_text(TAXI_METER_FOOTER_TEXT, FOOTER_TEXT_SIZE)
      self._footer_size = FOOTER_TEXT_SIZE if w <= FOOTER_TEXT_MAX_W else max(11.0, FOOTER_TEXT_SIZE * FOOTER_TEXT_MAX_W / w)
    text(TAXI_METER_FOOTER_TEXT, x + 24, y + 432, self._footer_size, FOOT_TEXT)
