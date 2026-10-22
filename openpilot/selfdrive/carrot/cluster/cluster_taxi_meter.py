"""K9ExtHud=1 taxi meter panel (right information panel of the external HUD).

Two parts, kept apart on purpose:
  * TaxiMeter       pure fare/distance/time logic (no raylib), unit-tested.
  * TaxiMeterPanel  draws the 보노보노 모범택시 design into the 792x478 information
                    panel. All artwork comes from assets/taxi_meter/*.png (baked by
                    make_taxi_meter_assets.py from taxi_meter_design.html); swapping the
                    design = replace those PNGs / the layout constants below.

Fares: one table, TAXI_FARE_SIHEUNG. 시흥시 = 경기도 표준형 요금 (2023-07-01 04:00 적용,
2026-08 보도 기준 그대로 유지):
  - https://m.mk.co.kr/amp/10745360  (매일경제 2023-05-25, 경기 택시요금 인상)
  - https://www.daehannoin.co.kr/news_view.jsp?ncd=3942
  - https://www.kihoilbo.co.kr/news/articleView.html?idxno=3033050  (2026-08, 경기 중형 4,800원 유지)
  - https://copyking.tistory.com/entry/전국-택시-할증-시간-및-요금-총정리
  - https://outfox.tistory.com/6
Night surcharge numbers come from a blog (https://m.blog.naver.com/simsimba89/223618430491):
확인 필요 (not confirmed against an official 경기도 notice). 시계외 할증 is not modelled.
"""
from __future__ import annotations

from dataclasses import dataclass
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
  time_fare_max_speed_kph: float
  # (start_hour, end_hour, rate) in local time; end exclusive; wraps midnight. 확인 필요.
  night_surcharges: tuple[tuple[int, int, float], ...]


TAXI_FARE_SIHEUNG = {
  # 모범·대형 (meter default)
  "deluxe": TaxiFare(
    name="시흥시 모범택시 (경기 표준형)",
    base_fare_won=7000, base_distance_m=3000.0,
    distance_unit_m=144.0, distance_unit_won=200,
    time_unit_s=35.0, time_unit_won=200,
    time_fare_max_speed_kph=15.0,
    night_surcharges=((23, 4, 0.20),),  # 확인 필요 (blog source)
  ),
  # 중형
  "midsize": TaxiFare(
    name="시흥시 중형택시 (경기 표준형)",
    base_fare_won=4800, base_distance_m=1600.0,
    distance_unit_m=131.0, distance_unit_won=100,
    time_unit_s=30.0, time_unit_won=100,
    time_fare_max_speed_kph=15.0,
    night_surcharges=((0, 2, 0.30), (23, 4, 0.20)),  # first match wins; 확인 필요 (blog source)
  ),
}
TAXI_METER_DEFAULT_FARE = TAXI_FARE_SIHEUNG["deluxe"]
TAXI_METER_FOOTER_TITLE = "시흥시 모범택시 요금 (경기 표준형)"
TAXI_METER_FOOTER_TEXT = "기본 7,000원(3km) · 거리 144m당 200원 · 시간 35초당 200원(15km/h 이하) · 심야 23~04시 20%"
TAXI_METER_MAX_DT_S = 1.0


def night_surcharge_rate(fare: TaxiFare, hour: int) -> float:
  for start, end, rate in fare.night_surcharges:
    inside = start <= hour < end if start < end else (hour >= start or hour < end)
    if inside:
      return rate
  return 0.0


class TaxiMeter:
  """부분 동시 병산 meter.

  Above time_fare_max_speed_kph the meter counts driven distance; at or below it
  counts time, converted to the equivalent distance (time_unit_s <-> distance_unit_m).
  The base fare covers base_distance_m of that combined distance; after it every
  distance_unit_m adds distance_unit_won. Each step is charged with the night rate
  active when it happens (the base fare with the rate at the start). Shown fare is
  rounded down to 10 won.
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

  @property
  def fare_won(self) -> int:
    return int(math.floor(self.fare_won_exact / 10.0 + 1e-9)) * 10

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

  def step(self, dt_s: float, speed_kph: float, hour: int = 12, now_s: float | None = None) -> int:
    """Advance by dt_s (clamped to 0..1 s). Returns how many +steps were charged."""
    dt = min(max(float(dt_s), 0.0), TAXI_METER_MAX_DT_S)
    self.started = True
    speed = max(0.0, float(speed_kph)) if math.isfinite(speed_kph) else 0.0
    self.elapsed_s += dt
    driven = speed / 3.6 * dt
    self.distance_m += driven
    if speed <= self.fare.time_fare_max_speed_kph:
      self.metered_m += dt * self.fare.distance_unit_m / self.fare.time_unit_s
    else:
      self.metered_m += driven
    new_units = self._units_for(self.metered_m)
    added = new_units - self.units
    if added > 0:
      rate = night_surcharge_rate(self.fare, hour)
      self.fare_won_exact += added * self.fare.distance_unit_won * (1.0 + rate)
      self.last_tick_s = now_s
      self.units = new_units
      return added
    return 0

  def update(self, now_s: float, speed_kph: float, onroad: bool, hour: int) -> int:
    """Real-time driver: starts at the base fare when a drive starts (onroad
    rising edge), counts while onroad, holds the last value when offroad."""
    if not onroad:
      self._last_now = None
      self._was_onroad = False
      return 0
    if not getattr(self, "_was_onroad", False):
      self.reset(hour)
      self._was_onroad = True
      self._last_now = now_s
      self.step(0.0, speed_kph, hour, now_s)
      return 0
    dt = 0.0 if self._last_now is None else now_s - self._last_now
    self._last_now = now_s
    return self.step(dt, speed_kph, hour, now_s)


def format_won(value: int) -> str:
  return f"{int(value):,}"


def format_hms(seconds: float) -> str:
  s = int(max(0.0, seconds))
  return f"{s // 3600:02d}:{(s // 60) % 60:02d}:{s % 60:02d}"


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


class TaxiMeterPanel:
  """Draws the meter. Static parts (card art + fixed labels + fare footer) are
  cached once in a render texture; per frame only numbers, the progress fill,
  stars and the two characters are drawn."""

  def __init__(self, meter: TaxiMeter | None = None, clock=time.monotonic, local_hour=None) -> None:
    self.meter = meter or TaxiMeter()
    self._clock = clock
    self._local_hour = local_hour or (lambda: time.localtime().tm_hour)
    self._textures: dict[str, object] = {}
    self._textures_loaded = False
    self._static_target = None
    self._static_key = None
    self.freeze_animation_t: float | None = None  # PNG review only

  # -- resources
  def _load_textures(self, rl) -> None:
    if self._textures_loaded:
      return
    self._textures_loaded = True
    for name in ("bg", "sea", "porori", "bonobono"):
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

  # -- per frame state
  def update(self, state) -> None:
    speed = float(getattr(state, "speed_kph", 0.0) or 0.0)
    self.meter.update(self._clock(), speed, bool(getattr(state, "onroad", False)), self._local_hour())

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

  def _sprite(self, rl, name: str, anchor_x: float, anchor_y: float, origin: tuple[float, float], rotation: float = 0.0) -> None:
    texture = self._textures.get(name)
    if texture is None:
      return
    w, h = float(texture.width), float(texture.height)
    rl.draw_texture_pro(texture, rl.Rectangle(0, 0, w, h), rl.Rectangle(anchor_x, anchor_y, w, h),
                        rl.Vector2(origin[0], origin[1]), rotation, rl.WHITE)

  def draw(self, renderer, rl, panel_x: float, panel_y: float) -> None:
    self._load_textures(rl)
    x = panel_x + CARD_OFFSET_X
    y = panel_y + CARD_OFFSET_Y
    meter = self.meter
    t = self.freeze_animation_t if self.freeze_animation_t is not None else self._clock()
    tk = 9.0 if meter.last_tick_s is None or self.freeze_animation_t is not None else max(0.0, t - meter.last_tick_s)
    text = renderer._draw_text
    self._draw_static_cached(renderer, rl, x, y)

    # stars (twinkle)
    for sx, sy, r, i in STARS:
      alpha = int(255 * (0.35 + 0.6 * abs(math.sin(t * 1.3 + i * 1.7))))
      rl.draw_circle_v(rl.Vector2(x + sx, y + sy), r, rl.Color(255, 244, 207, alpha))

    # status pill
    onroad = bool(getattr(meter, "_was_onroad", False))
    pill = rl.Rectangle(x + 776 - 26 - 92, y + 24, 92, 34)
    rl.draw_rectangle_rounded_lines_ex(pill, 1.0, 16, 1.2, rl.Color(201, 164, 87, 255))
    rl.draw_circle_v(rl.Vector2(pill.x + 22, pill.y + 17), 4.5, rl.Color(243, 207, 107, 255) if onroad else rl.Color(110, 98, 70, 255))
    text("주행" if onroad else "대기", pill.x + 36, pill.y + 17, 16, (240, 214, 144))

    # fare
    fare_text = format_won(meter.fare_won)
    pop = 1.0 + 0.09 * math.sin(tk / 0.5 * math.pi) if tk < 0.5 else 1.0
    size = 98.0 * pop
    fw, _ = renderer._measure_text(fare_text, size)
    text(fare_text, x + 24, y + 152, size, GOLD)
    text("원", x + 24 + fw + 10, y + 176, 30, (217, 189, 124))
    if tk < 1.2:
      alpha = int(255 * (1.0 - tk / 1.2))
      step_won = int(round(meter.fare.distance_unit_won * (1.0 + night_surcharge_rate(meter.fare, self._local_hour()))))
      text(f"+{format_won(step_won)}원", x + max(372.0, 24 + fw + 60), y + 120 - tk * 40, 30, (*GOLD_BRIGHT, alpha))

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
    run = t * 2 * math.pi * 2.4
    hop = math.sin(tk / 0.8 * math.pi) * 30 if tk < 0.8 else abs(math.sin(run)) * 9
    pxp = max(TRACK_X0 + 50, px)
    self._sprite(rl, "porori", x + pxp, y + 284 - hop, (86 * 0.86, 80 * 0.86), 0.0 if tk < 0.8 else math.sin(run) * 5)
    remaining = max(1, int(round(meter.next_step_remaining_m)))
    nx = x + 28
    for part, color, sz in (("다음 ", GOLD_LABEL, 14), (f"+{meter.fare.distance_unit_won}원", (240, 214, 144), 14),
                            ("까지 ", GOLD_LABEL, 14), (f"{remaining}", (240, 214, 144), 14), (" m", GOLD_LABEL, 14)):
      text(part, nx, y + 350, sz, color)
      nx += renderer._measure_text(part, sz)[0]

    # 보노보노 (bob + tap) and 딱!
    bob = math.sin(t * 2 * math.pi / 2.5) * 4.5
    rot = math.sin(t * 2 * math.pi / 5) * 3.2
    self._sprite(rl, "bonobono", x + 668, y + 306 + bob, (66 * 0.82, 80 * 0.82), rot)
    if tk < 0.7:
      text("딱!", x + 716, y + 330 - tk * 30, 22, (*GOLD_BRIGHT, int(255 * (1 - tk / 0.7))))

    # sea over the characters, then the fare footer
    sea = self._textures.get("sea")
    if sea is not None:
      rl.draw_texture(sea, int(x), int(y + 380), rl.WHITE)
    text(TAXI_METER_FOOTER_TITLE, x + 24, y + 410, 12, FOOT_TITLE)
    text(TAXI_METER_FOOTER_TEXT, x + 24, y + 432, 14, FOOT_TEXT)
