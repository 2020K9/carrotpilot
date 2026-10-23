"""K9ExtHud=1 taxi meter, C1 look (치비 보노보노·포로리, 조개 핸들), drawn live with raylib.

C1 replaces the look of K9ExtHud=1 (no new menu number). The fare logic is TaxiMeter /
TAXI_FARE_SIHEUNG from cluster_taxi_meter.py, unchanged.

Live inputs (every frame):
  * carState.steeringAngleDeg -> 조개 (clam) rotation on screen = -steeringAngleDeg, 1:1, no filter,
    no dead band, no clamp: the same formula as the carrot wheel (hud_renderer._draw_steering_wheel
    `rotation = -steeringAngleDeg`, cluster_renderer._draw_lfa_status_icon). Left turn (+) = counter-
    clockwise on screen. On this screen the carrot wheel is not drawn; the clam is the only steering
    display (보노보노 11:48 "당근스티어링은 빼자").
  * carState.vEgo -> fare (TaxiMeter, only while 주행/DRIVE), stop/start state changes, animation.
  * 승객감지 CAN 0x500 bus0 bits 2-3 (ACU14 CF_TTL_Ind, 1 = passenger) -> C1Fsm (10:15/10:35 rules):
    0->1 held 5 s while stopped = 손님탐; 1->0 held 5 s at any speed = 지불, amount = fare at the start
    of the last stop before the 1->0 (none: fare when it counts). OCCUPANT_ENABLED = True. The screen
    toggle (tap) always wins.
Season: K9MeterSeason 0 auto (month) / 1 봄 / 2 여름 / 3 가을 / 4 겨울; other values = 0.
Time of day (낮 05:00 / 노을 16:00 / 밤 19:00, 3 s cross-fade) follows the Asia/Seoul clock.

Artwork: assets/taxi_meter/c1/*.png (baked from out/taximeter/variants/C1.html by bake_c1.js).
Backgrounds are read from files: back_{season}_{tod}.png (behind the characters) and over_{season}_{tod}.png
(sea in front of them + frame/brand/stats/footer, = front_ + ui_ composited by make_c1_over.py), 776x456;
replacing those files swaps the backgrounds (tod = day / dusk / night). Only the current season is loaded.
"""
from __future__ import annotations

import json
import math
from datetime import datetime
from pathlib import Path
import time

from cluster_taxi_meter import (
  NIGHT_LAMP_H, NIGHT_LAMP_RED, NIGHT_LAMP_W, NIGHT_LAMP_X, NIGHT_LAMP_Y, SEOUL_TZ, TAXI_METER_CLOCK_MIN_YEAR, TAXI_METER_FOOTER_TEXT, TaxiMeter, MeterAnimation, format_hms,
  format_won, night_surcharge_rate, seoul_hour, taxi_meter_pose,
)

C1_ASSET_DIR = Path(__file__).resolve().parent / "assets" / "taxi_meter" / "c1"

# ---- occupant / state machine (port of variants/c1state.js, 09:26 + 09:30 + 10:35 FINAL) ----------------
OCC_SIGNAL = {"addr": 0x500, "bus": 0, "start_bit": 2, "len": 2, "present_value": 1}
OCCUPANT_ENABLED = True
OCC_STABLE_S = 5.0
OCC_STALE_S = 2.0          # no 0x500 frame for this long = no signal (the FSM keeps its last raw value)
C1_LOGIC = {"stop_kph": 1.0, "move_kph": 3.0, "occ_stable_s": OCC_STABLE_S, "empty_after_s": 10.0}
STATES = ("EMPTY", "BOARD", "DRIVE", "PAY")


def occupant_from_acu14(dat) -> bool | None:
  """0x500 bus0 byte 0 bits 2-3 (CF_TTL_Ind): 1 -> True, other values -> False, no data -> None."""
  try:
    b = dat[0]
  except (IndexError, TypeError, KeyError):
    return None
  if isinstance(b, (bytes, bytearray)):
    b = b[0]
  return ((int(b) >> OCC_SIGNAL["start_bit"]) & 0x3) == OCC_SIGNAL["present_value"]


class C1Fsm:
  """EMPTY(빈차) -> BOARD(손님탐) -> DRIVE(주행) -> PAY(지불) -> EMPTY. step(sig, dt):
  sig = dict(v_kph, occ (True/False/None = no signal), fare (current meter fare), tap (screen toggle))."""

  def __init__(self, cfg=C1_LOGIC, occ_on: bool = OCCUPANT_ENABLED) -> None:
    self.cfg = dict(cfg)
    self.occ_on = bool(occ_on)
    self.state = "EMPTY"
    self.since = 0.0
    self.t = 0.0
    self.ev = ""
    self.ev_t = -9.0
    self.occ: bool | None = None
    self.occ_raw: bool | None = None
    self.occ_t = 0.0
    self.rise_ok = False
    self.was_stopped = True
    self.last_stop_fare = None
    self.fall_stop_fare = None
    self.pay_fare = None
    self.pay_stopped = False

  def _set(self, s: str, ev: str, sig) -> None:
    self.state, self.since, self.ev, self.ev_t = s, self.t, ev, self.t
    if s == "DRIVE":
      self.last_stop_fare = None
    if s == "PAY":
      self.pay_stopped = float(sig.get("v_kph") or 0.0) <= self.cfg["stop_kph"]
      if self.pay_fare is None:
        self.pay_fare = sig.get("fare")
    if s != "PAY":
      self.pay_fare = None

  def step(self, sig, dt: float) -> "C1Fsm":
    cfg = self.cfg
    self.t += max(0.0, float(dt))
    v = float(sig.get("v_kph") or 0.0)
    stopped, moving = v <= cfg["stop_kph"], v > cfg["move_kph"]
    if self.state == "DRIVE" and stopped and not self.was_stopped:
      self.last_stop_fare = sig.get("fare")        # fare at the moment a stop starts while driving
    self.was_stopped = True if stopped else (False if moving else self.was_stopped)
    occ_edge = None
    occ = sig.get("occ")
    if self.occ_on and occ is not None:
      occ = bool(occ)
      if occ != self.occ_raw:
        self.occ_raw, self.occ_t, self.rise_ok = occ, self.t, stopped
        if not occ:
          self.fall_stop_fare = self.last_stop_fare if self.state == "DRIVE" else None
      if self.occ_raw is True and not stopped:
        self.rise_ok = False                         # moved while the 0->1 was being held: void
      if self.occ != self.occ_raw and self.t - self.occ_t > cfg["occ_stable_s"]:
        if self.occ_raw is True:
          if self.rise_ok:
            occ_edge, self.occ = True, True
        else:
          if self.occ is True:
            occ_edge = False                         # only a confirmed passenger can get out
          self.occ = False
    if sig.get("tap"):                               # screen toggle, always first
      nxt = {"EMPTY": "BOARD", "BOARD": "EMPTY", "DRIVE": "PAY", "PAY": "EMPTY"}[self.state]
      self._set(nxt, "tap", sig)
      return self
    if self.state == "PAY" and stopped:
      self.pay_stopped = True
    if self.state == "EMPTY":
      if occ_edge is True:
        self._set("BOARD", "occ", sig)
    elif self.state == "BOARD":
      if moving:
        self._set("DRIVE", "move", sig)
      elif occ_edge is False:
        self._set("EMPTY", "occ", sig)
    elif self.state == "DRIVE":
      if occ_edge is False:
        self.pay_fare = self.fall_stop_fare if self.fall_stop_fare is not None else sig.get("fare")
        self._set("PAY", "occ", sig)
    elif self.state == "PAY":
      if self.t - self.since >= cfg["empty_after_s"]:
        self._set("EMPTY", "timer", sig)
      elif moving and self.pay_stopped:
        self._set("EMPTY", "move", sig)
      elif occ_edge is True:
        self._set("BOARD", "occ", sig)
    return self


# ---- background / season -------------------------------------------------------------------------------
C1_BG_TIMES = (("day", 5 * 3600), ("dusk", 16 * 3600), ("night", 19 * 3600))
C1_BG_FADE_S = 3.0
C1_SEASON_BY_MONTH = {3: "spring", 4: "spring", 5: "spring", 6: "summer", 7: "summer", 8: "summer",
                      9: "autumn", 10: "autumn", 11: "autumn", 12: "winter", 1: "winter", 2: "winter"}
K9_METER_SEASON_PARAM = "K9MeterSeason"
K9_METER_SEASONS = {1: "spring", 2: "summer", 3: "autumn", 4: "winter"}   # 0 = auto (month)


def normalize_meter_season(value) -> int:
  """K9MeterSeason: 0..4; anything else (out of range, not a number, missing) = 0 (auto)."""
  try:
    v = int(value)
  except (TypeError, ValueError):
    return 0
  return v if 0 <= v <= 4 else 0


def c1_season(month: int | None, setting: int = 0) -> str:
  s = normalize_meter_season(setting)
  if s:
    return K9_METER_SEASONS[s]
  return C1_SEASON_BY_MONTH.get(month or 0, "autumn")


def c1_bg_weights(sec: float) -> dict:
  """Day/dusk/night weights for a second of the day (car clock): 3 s smoothstep at 05:00/16:00/19:00."""
  sec = ((float(sec) % 86400.0) + 86400.0) % 86400.0
  pts = sorted(C1_BG_TIMES, key=lambda p: p[1])
  i = len(pts) - 1
  for j, (_, start) in enumerate(pts):
    if start <= sec:
      i = j
  cur, prev = pts[i][0], pts[(i - 1) % len(pts)][0]
  el = (sec - pts[i][1] + 86400.0) % 86400.0
  x = min(1.0, el / C1_BG_FADE_S)
  w = x * x * (3 - 2 * x)
  o = {"day": 0.0, "dusk": 0.0, "night": 0.0, "cur": cur, "prev": prev, "w": w}
  o[cur] += w
  o[prev] += 1.0 - w
  return o


def seoul_clock(epoch_s: float | None = None) -> tuple[float, int] | None:
  """(second of the day, month) in Asia/Seoul, None when the clock is not synced."""
  try:
    local = datetime.fromtimestamp(time.time() if epoch_s is None else float(epoch_s), SEOUL_TZ)
  except (OverflowError, OSError, ValueError, TypeError):
    return None
  if local.year < TAXI_METER_CLOCK_MIN_YEAR:
    return None
  return local.hour * 3600 + local.minute * 60 + local.second + local.microsecond * 1e-6, local.month


# ---- steering (same formula as the carrot wheel) -------------------------------------------------------
def clam_rotation_deg(steering_angle_deg) -> float:
  """Screen rotation of the 조개 = -steeringAngleDeg (raylib degrees, + = clockwise), 1:1 every frame."""
  try:
    a = float(steering_angle_deg or 0.0)
  except (TypeError, ValueError):
    return 0.0
  return -a if math.isfinite(a) else 0.0


HAND_GRIP_MAX_DEG = 90.0   # hands follow the rim up to +-90 deg, beyond that the clam turns under the hands
BODY_LEAN_K = 0.06         # body lean = clamp(rotation, +-90) * 0.06 (max 5.4 deg, same as the HTML)


def hand_grip_deg(rot: float) -> float:
  return max(-HAND_GRIP_MAX_DEG, min(HAND_GRIP_MAX_DEG, rot))


# ---- theme (C1_THEME of c1bg.js) ----------------------------------------------------------------------
def _hex(c: str) -> tuple[int, int, int, int]:
  c = c.lstrip("#")
  if len(c) == 6:
    c += "ff"
  return tuple(int(c[i:i + 2], 16) for i in (0, 2, 4, 6))


C1_THEME = {
  "night": {"g2": "#d8b25e", "dsh": "#07060f", "lb": "#a8915f", "tx": "#f2ead8", "sub": "#b9a77f", "unit": "#d9bd7c",
            "acc": "#f0d690", "nx": "#a8915f", "pillB": "#c9a457", "pillT": "#f0d690", "pillBg": "#00000033",
            "plus": "#ffe08a", "trk0": "#2b2618", "tk0": "#7a5f27", "tk1": "#f1d68f"},
  "dusk": {"g2": "#dcb35c", "dsh": "#0c0610", "lb": "#b4955e", "tx": "#f6ead6", "sub": "#c4a77a", "unit": "#e0bf7a",
           "acc": "#f4d48a", "nx": "#c19b66", "pillB": "#c9a457", "pillT": "#f4d690", "pillBg": "#00000033",
           "plus": "#ffe08a", "trk0": "#2b2618", "tk0": "#7a5f27", "tk1": "#f1d68f"},
  "day": {"g2": "#1d3a5f", "dsh": "#ffffffcc", "lb": "#3d6a92", "tx": "#1d3a5f", "sub": "#4f7ea8", "unit": "#2b5c8a",
          "acc": "#d2552a", "nx": "#6b4f2a", "pillB": "#2b5c8a", "pillT": "#2b5c8a", "pillBg": "#fffdf4cc",
          "plus": "#ff7a45", "trk0": "#e0b97f", "tk0": "#ffb07a", "tk1": "#ff6a3d"},
}
_THEME_RGBA = {k: {kk: _hex(vv) for kk, vv in v.items()} for k, v in C1_THEME.items()}


def theme_weights(bw: dict) -> dict:
  """Text colours switch fast in the middle of the background fade (weights .4..6), like c1bg.js."""
  tx = min(1.0, max(0.0, (bw["w"] - 0.4) / 0.2))
  tw = tx * tx * (3 - 2 * tx)
  o = {"day": 0.0, "dusk": 0.0, "night": 0.0}
  o[bw["cur"]] += tw
  o[bw["prev"]] += 1.0 - tw
  return o


def theme_color(key: str, tw: dict, alpha_mul: float = 1.0) -> tuple[int, int, int, int]:
  c = [0.0, 0.0, 0.0, 0.0]
  for tod, w in tw.items():
    if w <= 0.0:
      continue
    for i, v in enumerate(_THEME_RGBA[tod][key]):
      c[i] += v * w
  return (int(round(c[0])), int(round(c[1])), int(round(c[2])), int(round(min(255.0, c[3]) * alpha_mul)))


PILL = {"EMPTY": ("빈차", (255, 90, 74)), "BOARD": ("손님탐", (126, 224, 160)), "DRIVE": ("주행", (243, 207, 107)),
        "PAY": ("지불", (111, 216, 255))}
LABEL = {"EMPTY": "기 본  요 금", "BOARD": "기 본  요 금", "DRIVE": "운 행  요 금", "PAY": "결 제  금 액"}
NEXT_MSG = {"EMPTY": "손님을 기다리는 중 · 기본 {base}원 (3km)", "BOARD": "어서 오세요 · 출발하면 요금이 올라갑니다",
            "PAY": "감사합니다 · 카드·현금 모두 됩니다 · 안녕히 가세요"}
SAY = {"BOARD": ("어서 오세요!", 0.1, 2.6), "DRIVE": ("출발~!", 0.1, 2.2), "PAY": ("감사합니다!", 0.1, 3.6),
       "EMPTY": ("또 만나요!", 0.1, 2.6)}

# ---- layout (card px, = C1.html) ----------------------------------------------------------------------
CARD_W, CARD_H = 776.0, 456.0
CARD_OFFSET_X, CARD_OFFSET_Y = 8.0, 12.0
SPRITE_ANCHOR = 120.0          # sprites are 240x240, local (0,0) at (120,120)
X0, X1, TRACK_Y = 28.0, 500.0, 326.0
PORO_Y, PORO_S, PORO_MIN = 284.2, 0.72, 70.0
PORO_JUMP_MAX = 14.0
OPAQUE_BACK = True
BONO_X, BONO_Y, BONO_S = 668.0, 306.0, 0.82
BONO_LEAN_PIVOT = (0.0, 150.0)
SHELL_C = (0.0, 77.0)          # steering clam centre (bono local)
SHELL_SC = 1.32
GRIP = (19.5, -1.0)            # hand points on the clam (clam local, x +-)
SHOULDER = (33.0, 88.0)
GOAL = (516.0, 318.0)
FARE_BOX = (14.0, 84.0, 410.0, 222.0)   # particles are not drawn over the fare digits
NO_PARTICLE_BOXES = (FARE_BOX, (NIGHT_LAMP_X - 10, NIGHT_LAMP_Y - 10, NIGHT_LAMP_X + NIGHT_LAMP_W + 10,
                                NIGHT_LAMP_Y + NIGHT_LAMP_H + 10),   # ... nor over the night lamp,
                     (546.0, 72.0, 758.0, 236.0))   # ... nor over the distance / time numbers (C1.html had only the fare)
BUBBLE_FILL = (255, 253, 246)
BUBBLE_TEXT = (43, 52, 72)
BONO_PAL = {"f": (92, 188, 255), "o": (18, 48, 79)}
PARTICLES = {"spring": ("petal", ((255, 192, 214), (255, 214, 228), (255, 158, 192))),
             "autumn": ("leaf", ((232, 100, 44), (240, 160, 48), (200, 64, 42))),
             "winter": ("snow", ((255, 255, 255),)), "summer": (None, ())}


def _rot(px: float, py: float, deg: float) -> tuple[float, float]:
  a = math.radians(deg)
  c, s = math.cos(a), math.sin(a)
  return px * c - py * s, px * s + py * c


def _sm(x: float) -> float:
  x = min(1.0, max(0.0, x))
  return x * x * (3 - 2 * x)


def _win(t: float, a: float, b: float, f: float = 0.25) -> float:
  return min(1.0, max(0.0, min((t - a) / f, (b - t) / f)))


BUBBLE_H = 34.0
BUBBLE_Y = 236.0
BUBBLE_MAX_RIGHT = 600.0   # 보노보노's head starts at x ~612 (bono_body.png bbox, bob and sway included)


def bubble_rect(text_w: float, x_char: float):
  """포로리 speech bubble (card px): (x, y, w, h, tail_on_right). Right of 포로리 like C1.html, flipped to its left
  when it would reach 보노보노 (C1.html flipped at a fixed x 460, which let a wide bubble touch 보노보노)."""
  w = text_w + 28
  X = x_char + 10
  right = X + 32 + w > BUBBLE_MAX_RIGHT
  bx = (X - 32 - w) if right else (X + 32)
  return bx, BUBBLE_Y, w, BUBBLE_H, right


class BonoTransform:
  """bono local -> card px: translate(bx, by+bob) rotate(rot) scale(s), then lean about local (0,150)."""

  def __init__(self, ox: float, oy: float, rot: float, lean: float, scale: float = BONO_S) -> None:
    self.ox, self.oy, self.rot, self.lean, self.s = ox, oy, rot, lean, scale

  def __call__(self, px: float, py: float) -> tuple[float, float]:
    cx, cy = BONO_LEAN_PIVOT
    lx, ly = _rot(px - cx, py - cy, self.lean)
    lx, ly = (lx + cx) * self.s, (ly + cy) * self.s
    rx, ry = _rot(lx, ly, self.rot)
    return self.ox + rx, self.oy + ry

  def inverse(self, qx: float, qy: float) -> tuple[float, float]:
    rx, ry = _rot(qx - self.ox, qy - self.oy, -self.rot)
    cx, cy = BONO_LEAN_PIVOT
    lx, ly = _rot(rx / self.s - cx, ry / self.s - cy, -self.lean)
    return lx + cx, ly + cy

  @property
  def angle(self) -> float:
    return self.rot + self.lean


def clam_geometry(steering_angle_deg, bob: float = 0.0, brot: float = 0.0, tap: float = 0.0,
                  origin=(BONO_X, BONO_Y)) -> dict:
  """Pure geometry of the steering clam for one frame (card px). rotation = -steeringAngleDeg exactly."""
  rot = clam_rotation_deg(steering_angle_deg)
  grip = hand_grip_deg(rot)
  lean = BODY_LEAN_K * grip
  tf = BonoTransform(origin[0], origin[1] + bob, brot, lean)
  center = tf(SHELL_C[0], SHELL_C[1] - tap * 5.0)
  hands = []
  for sg in (-1.0, 1.0):
    hx, hy = _rot(sg * GRIP[0] * SHELL_SC * BONO_S, GRIP[1] * SHELL_SC * BONO_S, grip)
    hands.append((center[0] + hx, center[1] + hy))
  shoulders = [tf(sg * SHOULDER[0], SHOULDER[1]) for sg in (-1.0, 1.0)]
  return {"rotation": rot, "grip": grip, "lean": lean, "center": center, "hands": hands, "shoulders": shoulders,
          "tf": tf}


class C1MeterPanel:
  """Same interface as TaxiMeterPanel (renderer: update / feed_speed_samples / restart / prepare_static_layer /
  draw / close), C1 look, live state machine."""

  def __init__(self, meter: TaxiMeter | None = None, clock=time.monotonic, local_hour=None, wall_clock=None) -> None:
    self.meter = meter or TaxiMeter()
    self.anim = MeterAnimation(self.meter.fare.time_fare_below_kph)
    self.fsm = C1Fsm()
    self.fsm_mode = False   # False until a 0x500 frame (or the toggle) is seen: then the C1 state machine
    self._speed_samples: list[tuple[float, float]] = []
    self._speed_samples_live = False
    self._clock = clock
    self._local_hour = local_hour or seoul_hour
    self._wall_clock = wall_clock   # -> (sec of day, month) | None; None = Asia/Seoul clock (or the injected hour)
    self.season_setting = 0
    self.night_rate = 0.0
    self.steering_angle_deg = 0.0
    self.speed_kph = 0.0
    self.occupant: bool | None = None
    self.occupant_t: float | None = None
    self._tap = False
    self._last_t: float | None = None
    self.bg_weights = c1_bg_weights(21 * 3600)
    self.season = "autumn"
    self._textures: dict[str, object] = {}
    self._textures_loaded = False
    self._loaded_season = None
    self.freeze_animation_t: float | None = None
    self._footer_size: float | None = None
    self._static_target = None   # kept for API parity (the C1 static layers are textures)
    self.last_clam_rotation = 0.0
    self._last_center = (0.0, 0.0)
    self.draw_count = 0

  def _clock_from_hour(self):
    h = self._local_hour()
    if h is None:
      return None
    wc = seoul_clock()
    return h * 3600.0 + 1800.0, (wc[1] if wc is not None else 10)

  # -- inputs
  def feed_speed_samples(self, samples) -> None:
    self._speed_samples_live = True
    self._speed_samples.extend(samples)
    if len(self._speed_samples) > 2000:
      self._speed_samples = self._speed_samples[-2000:]

  def feed_occupant(self, present: bool | None, t_s: float | None = None) -> None:
    """Latest 0x500 bus0 CF_TTL_Ind (True/False), None = no frame. Stale after OCC_STALE_S."""
    if present is None:
      return
    self.occupant = bool(present)
    self.occupant_t = self._clock() if t_s is None else float(t_s)

  def toggle(self) -> None:
    """Screen toggle (always wins): 빈차->손님탐, 손님탐->빈차, 주행->지불, 지불->빈차."""
    self._tap = True

  def set_season_setting(self, value) -> None:
    self.season_setting = normalize_meter_season(value)

  def restart(self) -> None:
    self._speed_samples = []
    self.meter.reset()
    self.meter._was_onroad = False
    self.anim.reset()
    self.fsm = C1Fsm()
    self.fsm_mode = False
    self._last_t = None
    self._tap = False

  @property
  def step_won(self) -> int:
    return int(round(self.meter.fare.distance_unit_won * (1.0 + self.night_rate)))

  @property
  def base_fare_won(self) -> int:
    return int(math.floor(self.meter.fare.base_fare_won * (1.0 + self.night_rate) / 10.0 + 0.5)) * 10

  # -- per frame
  def _occ_signal(self, now: float):
    if self.occupant is None or self.occupant_t is None:
      return None
    return self.occupant if now - self.occupant_t <= OCC_STALE_S else None

  def update(self, state) -> None:
    speed = float(getattr(state, "speed_kph", 0.0) or 0.0)
    onroad = bool(getattr(state, "onroad", False))
    steer = getattr(state, "steering_angle_deg", None)
    self.steering_angle_deg = float(steer) if steer is not None and math.isfinite(float(steer)) else 0.0
    self.speed_kph = speed
    now = self._clock()
    hour = self._local_hour()
    self.night_rate = night_surcharge_rate(self.meter.fare, hour)
    if self._wall_clock is not None:
      wc = self._wall_clock()
    elif self._local_hour is seoul_hour:
      wc = seoul_clock()
    else:
      wc = self._clock_from_hour()   # tests / replays that inject the hour: pictures follow that hour
    sec, month = (wc if wc is not None else (21 * 3600.0, 10))
    self.bg_weights = c1_bg_weights(sec)
    self.season = c1_season(month, self.season_setting)
    dt = 0.0 if self._last_t is None else max(0.0, min(0.25, now - self._last_t))
    self._last_t = now
    tap, self._tap = self._tap, False
    occ = self._occ_signal(now)
    if occ is not None or tap:
      self.fsm_mode = True       # occupant sensor seen (or the toggle used): C1 state machine from now on
    if not self.fsm_mode:
      # no 0x500 frame ever (other car / no CAN yet): bd09c076 meter as before - onroad rising edge = new drive
      if self._speed_samples_live:
        samples, self._speed_samples = self._speed_samples, []
        self.meter.update_samples(samples, onroad, hour, now)
      else:
        self.meter.update(now, speed, onroad, hour)
      self.anim.update(now, speed, onroad)
      return
    prev_state = self.fsm.state
    self.fsm.step({"v_kph": speed, "occ": occ, "fare": self.meter.fare_won, "tap": tap}, dt)
    st = self.fsm.state
    if st in ("BOARD", "DRIVE") and prev_state not in ("BOARD", "DRIVE"):
      self.meter.reset(hour)                    # new passenger: base fare, 0 km, 0 s
      self.meter._was_onroad = True
    samples, self._speed_samples = self._speed_samples, []
    if st == "DRIVE":                           # 주행 = moving with a passenger: counts (onroad flag not needed)
      if self._speed_samples_live:
        for t_s, v in samples:
          self.meter.feed_sample(t_s, v, hour, now)
      else:
        dtm = 0.0 if self.meter._last_now is None else now - self.meter._last_now
        self.meter._last_now = now
        self.meter.step(dtm, speed, hour, now)
    else:
      self.meter._last_sample_t = None          # next DRIVE sample only sets the time base
      self.meter._last_now = None
    self.anim.update(now, speed, onroad)

  @property
  def display_state(self) -> str:
    if self.fsm_mode:
      return self.fsm.state
    return "DRIVE" if getattr(self.meter, "_was_onroad", False) else "EMPTY_LEGACY"

  def shown_fare(self) -> int:
    if not self.fsm_mode:
      return self.meter.fare_won
    st = self.fsm.state
    if st == "DRIVE":
      return self.meter.fare_won
    if st == "PAY":
      return int(self.fsm.pay_fare if self.fsm.pay_fare is not None else self.meter.fare_won)
    return self.base_fare_won

  # -- resources
  def _tex(self, rl, name: str):
    if name in self._textures:
      return self._textures[name]
    path = C1_ASSET_DIR / f"{name}.png"
    tex = None
    if path.exists():
      try:
        t = rl.load_texture(str(path))
        if t.id > 0:
          rl.set_texture_filter(t, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
          rl.set_texture_wrap(t, rl.TextureWrap.TEXTURE_WRAP_CLAMP)
          tex = t
      except Exception as exc:
        print(f"C1 asset load failed for {path.name}: {exc}", flush=True)
    self._textures[name] = tex
    return tex

  def _drop_season(self, rl, keep: str) -> None:
    for name in list(self._textures):
      if (name.startswith("back_") or name.startswith("over_")) and f"_{keep}_" not in name:
        tex = self._textures.pop(name)
        if tex is not None:
          try:
            rl.unload_texture(tex)
          except Exception:
            pass

  def prepare_static_layer(self, renderer, rl) -> None:
    if self._loaded_season != self.season:
      self._drop_season(rl, self.season)   # only the current season's 6 background layers stay in memory
      self._loaded_season = self.season
    for tod in ("day", "dusk", "night"):
      if self.bg_weights.get(tod, 0.0) > 0.0:
        self._tex(rl, f"back_{self.season}_{tod}")
        self._tex(rl, f"over_{self.season}_{tod}")
    for name in ("porori_body", "porori_tail", "porori_fL", "porori_fR", "porori_aL", "porori_aR", "bono_body", "shell"):
      self._tex(rl, name)

  def close(self, rl) -> None:
    for tex in self._textures.values():
      if tex is not None:
        try:
          rl.unload_texture(tex)
        except Exception:
          pass
    self._textures.clear()
    self._loaded_season = None

  # -- drawing helpers
  def _layer(self, rl, name: str, x: float, y: float, alpha: float) -> None:
    tex = self._textures.get(name)
    if tex is None or alpha <= 0.002:
      return
    w, h = float(tex.width), float(tex.height)
    rl.draw_texture_pro(tex, rl.Rectangle(0, 0, w, h), rl.Rectangle(x, y, w, h), rl.Vector2(0, 0), 0.0,
                        rl.Color(255, 255, 255, int(round(255 * min(1.0, alpha)))))

  def _layers(self, rl, kind: str, x: float, y: float) -> None:
    """Cross-fade: previous layer opaque underneath, current on top with the fade weight."""
    bw = self.bg_weights
    cur, prev, w = bw["cur"], bw["prev"], bw["w"]
    name = lambda tod: f"{kind}_{self.season}_{tod}"   # noqa: E731
    if kind == "back" and w >= 1.0 and OPAQUE_BACK:
      # the back layer is opaque inside the card: draw it without blending (cheaper per pixel); the rounded
      # corners write transparent black, the same colour as the HUD background around the card
      flush = getattr(rl, "rl_draw_render_batch_active", None)
      off, on = getattr(rl, "rl_disable_color_blend", None), getattr(rl, "rl_enable_color_blend", None)
      if flush is not None and off is not None and on is not None:
        flush()
        off()
        self._layer(rl, name(cur), x, y, 1.0)
        flush()
        on()
        return
    if w < 1.0:
      self._layer(rl, name(prev), x, y, 1.0 if kind == "back" else 1.0 - w)
    self._layer(rl, name(cur), x, y, w)

  def _sprite(self, rl, name: str, pos, angle: float, pivot_local=(0.0, 0.0), scale: float = 1.0,
              local_scale: float = 1.0, tint=(255, 255, 255, 255)) -> None:
    """Sprite baked with local (0,0) at (120,120); pivot_local in baked px (already scaled)."""
    tex = self._textures.get(name)
    if tex is None:
      return
    w, h = float(tex.width), float(tex.height)
    ox = (SPRITE_ANCHOR + pivot_local[0]) * scale
    oy = (SPRITE_ANCHOR + pivot_local[1]) * scale
    rl.draw_texture_pro(tex, rl.Rectangle(0, 0, w, h), rl.Rectangle(pos[0], pos[1], w * scale, h * scale),
                        rl.Vector2(ox, oy), angle, rl.Color(*tint))

  def _part(self, rl, name, anchor, body_rot, scale, pivot, part_rot=0.0, offset=(0.0, 0.0)) -> None:
    """Character part: pivot in character local units; drawn rotated body_rot + part_rot about it."""
    px, py = _rot((pivot[0] + offset[0]) * scale, (pivot[1] + offset[1]) * scale, body_rot)
    self._sprite(rl, name, (anchor[0] + px, anchor[1] + py), body_rot + part_rot,
                 (pivot[0] * scale, pivot[1] * scale))

  @staticmethod
  def _limb(rl, pts, w: float, fill, outline) -> None:
    for color, width in ((outline, w + 3.2 * BONO_S), (fill, w)):
      c = rl.Color(*color, 255)
      for a, b in zip(pts, pts[1:]):
        rl.draw_line_ex(rl.Vector2(*a), rl.Vector2(*b), width, c)
      for p in (pts[0], pts[-1]) if color is fill else pts:
        rl.draw_circle_v(rl.Vector2(*p), width * 0.5, c)

  @staticmethod
  def _quad(p0, p1, p2, n: int = 8):
    out = []
    for i in range(n + 1):
      t = i / n
      a, b, c = (1 - t) ** 2, 2 * (1 - t) * t, t * t
      out.append((a * p0[0] + b * p1[0] + c * p2[0], a * p0[1] + b * p1[1] + c * p2[1]))
    return out

  def _draw_bono(self, rl, x: float, y: float, pose, tap: float) -> dict:
    g = clam_geometry(self.steering_angle_deg, pose.bono_bob, pose.bono_rot, tap, (x + BONO_X, y + BONO_Y))
    tf = g["tf"]
    origin = tf(0.0, 0.0)
    self._sprite(rl, "bono_body", origin, tf.angle)
    # arms: shoulder -> elbow bent outward -> hand (as steer.js)
    fill, outline = BONO_PAL["f"], BONO_PAL["o"]
    for i, sg in enumerate((-1.0, 1.0)):
      sx, sy = g["shoulders"][i]
      hx, hy = g["hands"][i]
      _, hy_local = tf.inverse(hx, hy)
      ex, ey = tf(sg * (SHOULDER[0] + 11.0), max(SHOULDER[1], hy_local) + 2.0)   # elbow bent outward (steer.js)
      self._limb(rl, self._quad((sx, sy), (ex, ey), (hx, hy)), 9.0 * BONO_S, fill, outline)
    cx, cy = g["center"]
    self._last_center = (cx, cy)
    self._sprite(rl, "shell", (cx, cy), g["rotation"])
    self.last_clam_rotation = g["rotation"]
    for hx, hy in g["hands"]:
      rl.draw_circle_v(rl.Vector2(hx, hy), (6.4 + 0.8) * BONO_S, rl.Color(*outline, 255))
      rl.draw_circle_v(rl.Vector2(hx, hy), 6.4 * BONO_S - 0.5, rl.Color(*fill, 255))
    return g

  def _draw_porori(self, rl, anchor, pose, wave: float, bow: float) -> None:
    s, rot = PORO_S, pose.porori_rot
    self._part(rl, "porori_tail", anchor, rot, s, (-12.0, 56.0), part_rot=pose.porori_tail)
    for name, lift in (("porori_fL", max(0.0, pose.porori_leg)), ("porori_fR", max(0.0, -pose.porori_leg))):
      self._part(rl, name, anchor, rot, s, (0.0, 0.0), offset=(0.0, -lift))
    self._part(rl, "porori_body", anchor, rot, s, (0.0, 0.0))
    arm = pose.porori_arm
    self._part(rl, "porori_aL", anchor, rot, s, (-10.0, 8.0), part_rot=arm * (1 - bow) - bow * 28)
    self._part(rl, "porori_aR", anchor, rot, s, (10.0, 8.0), part_rot=-arm * (1 - wave) * (1 - bow) - wave * 130 + bow * 28)

  def _draw_bubble(self, renderer, rl, text: str, x_char: float, x: float, y: float, alpha: float) -> None:
    if alpha <= 0.01 or not text:
      return
    a = int(255 * alpha)
    tw, _ = renderer._measure_text(text, 17)
    bx, by, w, h, right = bubble_rect(tw, x_char)
    rl.draw_rectangle_rounded(rl.Rectangle(x + bx, y + by, w, h), 0.9, 12, rl.Color(*BUBBLE_FILL, a))
    mid = y + by + h / 2
    if right:   # tail on the right edge, pointing at 포로리
      rl.draw_triangle(rl.Vector2(x + bx + w - 1, mid - 7), rl.Vector2(x + bx + w - 1, mid + 7), rl.Vector2(x + bx + w + 11, mid), rl.Color(*BUBBLE_FILL, a))
    else:
      rl.draw_triangle(rl.Vector2(x + bx - 11, mid), rl.Vector2(x + bx + 1, mid + 7), rl.Vector2(x + bx + 1, mid - 7), rl.Color(*BUBBLE_FILL, a))
    renderer._draw_text(text, x + bx + 14, mid, 17, (*BUBBLE_TEXT, a))

  def _draw_particles(self, rl, x: float, y: float, t: float) -> None:
    kind, cols = PARTICLES.get(self.season, (None, ()))
    if not kind:
      return
    for i in range(10):
      sp = 22 + (i % 4) * 5 if kind == "snow" else (26 + (i % 3) * 6 if kind == "leaf" else 20 + (i % 3) * 5)
      py = ((i * 53 + t * sp) % 500) - 22
      px = (i * 83.7 + math.sin(t * 0.9 + i * 1.7) * (10 if kind == "snow" else 22) + t * (3 if kind == "snow" else 9)) % 800 - 12
      if any(b[0] < px < b[2] and b[1] < py < b[3] for b in NO_PARTICLE_BOXES):
        continue
      if not (0 <= px <= CARD_W and 0 <= py <= CARD_H):
        continue
      c = rl.Color(*cols[i % len(cols)], 217)
      if kind == "snow":
        rl.draw_circle_v(rl.Vector2(x + px, y + py), 2.0, c)
      elif kind == "petal":
        rl.draw_ellipse(int(x + px), int(y + py), 3.4, 2.0, c)
      else:
        rl.draw_poly(rl.Vector2(x + px, y + py), 5, 4.6, (t * (60 + i * 13) + i * 40) % 360, c)

  def _draw_night_lamp(self, renderer, rl, lx: float, ly: float, t: float) -> None:
    """'심야 할증 20%' lamp, same lamp and place as the bd09c076 meter (top row, between brand and moon)."""
    box = rl.Rectangle(lx, ly, NIGHT_LAMP_W, NIGHT_LAMP_H)
    rl.draw_rectangle_rounded(box, 1.0, 16, rl.Color(58, 16, 12, 235))
    rl.draw_rectangle_rounded_lines_ex(box, 1.0, 16, 1.4, rl.Color(*NIGHT_LAMP_RED, 255))
    glow = int(70 + 40 * (0.5 + 0.5 * math.sin(t * 2.2)))
    cx, cy = lx + 20, ly + NIGHT_LAMP_H / 2
    rl.draw_circle_v(rl.Vector2(cx, cy), 9.0, rl.Color(*NIGHT_LAMP_RED, glow))
    rl.draw_circle_v(rl.Vector2(cx, cy), 4.8, rl.Color(255, 138, 96, 255))
    renderer._draw_text(f"심야 할증 {int(round(self.night_rate * 100))}%", lx + 34, cy, 15, (255, 196, 160))

  # -- frame
  def draw(self, renderer, rl, panel_x: float, panel_y: float) -> None:
    self.prepare_static_layer(renderer, rl)
    self.draw_count += 1
    x, y = panel_x + CARD_OFFSET_X, panel_y + CARD_OFFSET_Y
    meter, fsm = self.meter, self.fsm
    st = self.display_state
    legacy = st == "EMPTY_LEGACY"           # no occupant sensor, offroad: old '대기' look
    if legacy:
      st = "EMPTY"
    t = self.freeze_animation_t if self.freeze_animation_t is not None else self._clock()
    since = (fsm.t - fsm.since) if self.fsm_mode else 99.0
    tk = 9.0 if (st != "DRIVE" or meter.last_tick_s is None or self.freeze_animation_t is not None) else max(0.0, t - meter.last_tick_s)
    pose = taxi_meter_pose(self.anim, t, tk)
    tw = theme_weights(self.bg_weights)
    text = renderer._draw_text

    self._layers(rl, "back", x, y)
    self._draw_particles(rl, x, y, t)
    # progress track
    p = meter.step_progress if st == "DRIVE" else 0.0
    rl.draw_line_ex(rl.Vector2(x + X0, y + TRACK_Y), rl.Vector2(x + X1, y + TRACK_Y), 3.0, rl.Color(*theme_color("trk0", tw)))
    px_fill = X0 + (X1 - X0) * p
    if px_fill > X0 + 1:
      rl.draw_rectangle_gradient_h(int(x + X0), int(y + TRACK_Y - 2), int(px_fill - X0), 4,
                                   rl.Color(*theme_color("tk0", tw)), rl.Color(*theme_color("tk1", tw)))
    # goal clam (static, scale 1 of the steering clam's 1.32*0.82)
    goal_scale = 1.0 / (SHELL_SC * BONO_S)
    self._sprite(rl, "shell", (x + GOAL[0], y + GOAL[1]), 0.0, scale=goal_scale)
    # 포로리: start point while 빈차/손님탐, moves with the fare step while 주행
    pxp = max(X0 + PORO_MIN, px_fill if st == "DRIVE" else X0)
    greet = _win(since, 0.0, 2.6, 0.3) if st == "BOARD" else 0.0
    bow = (max(0.0, math.sin(min(since, 2.4) / 1.2 * math.pi)) if since < 2.4 else 0.0) if st == "PAY" else 0.0
    wave = greet + (_win(since, 0.2, 2.2, 0.3) if (st == "EMPTY" and fsm.ev) else 0.0)
    hop = min(pose.porori_hop, PORO_JUMP_MAX)   # C1.html jump 14 px (the bd09c076 30 px hop would touch the digits)
    self._draw_porori(rl, (x + pxp, y + PORO_Y - hop + bow * 6.0), pose, min(1.0, wave), bow)
    # 보노보노 + steering clam (the only steering display on this screen)
    tap = pose.bono_tap if st != "PAY" else 0.0
    self._draw_bono(rl, x, y, pose, tap)
    self._layers(rl, "over", x, y)            # sea in front of the characters + frame/brand/stats/footer (one layer)

    # status pill (text drawn live; frame colours from the theme)
    label, dot = ("대기", (110, 98, 70)) if legacy else PILL[st]
    tw_px, _ = renderer._measure_text(label, 16)
    pw = 17 + 9 + 9 + tw_px + 17
    pill = rl.Rectangle(x + 750 - pw, y + 24, pw, 34)
    rl.draw_rectangle_rounded(pill, 1.0, 16, rl.Color(*theme_color("pillBg", tw)))
    rl.draw_rectangle_rounded_lines_ex(pill, 1.0, 16, 1.0, rl.Color(*theme_color("pillB", tw)))
    if st == "EMPTY" and not legacy:
      k = 0.5 - 0.5 * math.cos(t * math.pi)
      dot = tuple(int(round(a + (b - a) * k)) for a, b in zip(dot, (122, 42, 36)))
    rl.draw_circle_v(rl.Vector2(pill.x + 17 + 4.5, pill.y + 17), 9.0, rl.Color(*dot, 60))
    rl.draw_circle_v(rl.Vector2(pill.x + 17 + 4.5, pill.y + 17), 4.5, rl.Color(*dot, 255))
    text(label, pill.x + 17 + 9 + 9, pill.y + 17, 16, theme_color("pillT", tw))
    if self.night_rate > 0.0:
      self._draw_night_lamp(renderer, rl, x + NIGHT_LAMP_X, y + NIGHT_LAMP_Y, t)

    # label + fare
    text(LABEL[st], x + 28, y + 83, 13, theme_color("lb", tw))
    fare_alpha = 0.62 if (st == "EMPTY" and not legacy) else 1.0
    fare_text = format_won(self.shown_fare())
    pop = 1.0 + 0.09 * math.sin(tk / 0.5 * math.pi) if tk < 0.5 else 1.0
    size = 98.0 * pop
    fw, _ = renderer._measure_text(fare_text, size)
    text(fare_text, x + 24, y + 143 + 1.5, size, theme_color("dsh", tw, fare_alpha))
    text(fare_text, x + 24, y + 143, size, theme_color("g2", tw, fare_alpha))
    text("원", x + 24 + fw + 10, y + 166, 30, theme_color("unit", tw, fare_alpha))
    if st == "DRIVE" and tk < 1.2:
      text(f"+{format_won(self.step_won)}원", x + max(372.0, 24 + fw + 60), y + 118 - tk * 40, 30,
           theme_color("plus", tw, 1.0 - tk / 1.2))
    # stats
    km = f"{meter.distance_m / 1000.0:.2f}"
    kw, _ = renderer._measure_text(km, 27)
    text(km, x + 566, y + 127, 27, theme_color("tx", tw))
    text("km", x + 566 + kw + 5, y + 131, 15, theme_color("sub", tw))
    text(format_hms(meter.elapsed_s), x + 566, y + 199, 27, theme_color("tx", tw))
    # next line
    nx, ny = x + 28, y + 350
    if st == "DRIVE":
      time_mode = meter.mode == "time"
      if time_mode:
        remaining, unit = max(1, int(math.ceil(meter.next_step_remaining_s - 1e-6))), "초"
      else:
        remaining, unit = max(1, int(round(meter.next_step_remaining_m))), " m"
      parts = (("다음 ", "nx"), (f"+{format_won(self.step_won)}원", "acc"), ("까지 ", "nx"), (f"{remaining}", "acc"),
               (unit, "nx"), (f"  ·  현재 속도 ", "nx"), (f"{int(round(self.speed_kph))}", "acc"), (" km/h", "nx"))
    else:
      msg = "주행을 시작하면 요금이 올라갑니다" if legacy else NEXT_MSG[st].format(base=format_won(self.base_fare_won))
      parts = ((msg, "nx"),)
    for part, key in parts:
      text(part, nx, ny, 14, theme_color(key, tw))
      nx += renderer._measure_text(part, 14)[0]
    # 딱! on a fare tick
    if st == "DRIVE" and tk < 0.7:
      text("딱!", x + 732, y + 312 - tk * 30, 22, theme_color("plus", tw, 1.0 - tk / 0.7))
    # speech bubble (one at a time)
    say, a0, a1 = SAY.get(st, ("", 0, 0))
    op = _win(since, a0, a1) if (st != "DRIVE" or since < 2.4) and (st != "EMPTY" or fsm.ev) else 0.0
    self._draw_bubble(renderer, rl, say, pxp, x, y, op)
