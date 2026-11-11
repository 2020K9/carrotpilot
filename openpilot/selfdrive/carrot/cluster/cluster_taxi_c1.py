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
    0->1 held 5 s while stopped = 손님탐 (1->0 again before moving = 빈차). meter_improve (보노보노 20:43 "운행끝나고
    승객연동되잇는거 지워"): the end of a drive is no longer tied to the passenger - a 1->0 while 주행 does not make
    지불 any more (it did: 1->0 held 5 s = 지불, amount = the fare at the start of the last stop), nor does a 0->1
    while 지불 make 손님탐. meter_empty2 (보노보노 21:59 "손님탐도없애"): OCCUPANT_ENABLED = False - the sensor changes no state
    at all and its reader is not started (main.py); the meter starts in 빈차 (own screen) and only the app's 시작 /
    일시정지 / 정산 / 빈차 (or the screen toggle = the next of those) change it. The sensor code stays for True.
  * 0x500 signal loss (r2): only NEW frames are handed on; 2 s without one = 모름 (OCC_STALE_S). A 5 s hold in
    progress is dropped, never completed, and restarts from 0 when frames return.
  * LFA (r2, the carrot wheel stays hidden): badge under the clam = lane icon (solid: lane lines in use / faint:
    laneless, carrot rule; green on / grey off). meter_improve (보노보노 20:43): no "LFA" text on it any more, and the
    clam keeps its own colours whatever LFA / steering do (it was dimmed with LFA off).
Season: K9MeterSeason 0 auto (month) / 1 봄 / 2 여름 / 3 가을 / 4 겨울; other values = 0.
Backgrounds (r2): bg2 scenes (out/taximeter/variants/bg2), baked by bake_c1.js; moving layer = ptcl_params.json +
ptcl_white.png (petal / leaf / ginkgo / snow / firefly).
Moving scene bits (r3): what c1bg.js draw() animates in the HTML (first 14 data-tw elements per time of day =
twinkle opacity, every data-dr element = slow drift in x: lanterns, windows, stars, light paths on the sea, clouds,
summer front waves) is left out of back_/over_ and drawn live from small baked sprites (anim_{season}.png +
anim.json, same formulas). Particles fade out near the card edge and are clipped to the card (scissor).
CPU (r2): sprites are drawn from their alpha bounding box, the 'over' layer as runs of non-empty 16 px tiles.
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
  NIGHT_LAMP_H, NIGHT_LAMP_RED, NIGHT_LAMP_W, NIGHT_LAMP_X, NIGHT_LAMP_Y, SEOUL_TZ, TAXI_METER_C1_FARE, TAXI_METER_CLOCK_MIN_YEAR,
  TAXI_METER_FOOTER_TEXT, TaxiMeter, MeterAnimation, format_hms,
  format_won, night_surcharge_rate, seoul_hour, taxi_meter_pose,
)

C1_ASSET_DIR = Path(__file__).resolve().parent / "assets" / "taxi_meter" / "c1"

# ---- occupant / state machine (port of variants/c1state.js, 09:26 + 09:30 + 10:35 FINAL) ----------------
OCC_SIGNAL = {"addr": 0x500, "bus": 0, "start_bit": 2, "len": 2, "present_value": 1}
OCCUPANT_ENABLED = False   # 보노보노 2026-10-24 21:59 "손님탐도없애": no passenger-sensor transitions at all (app/screen only)
OCC_STABLE_S = 5.0
OCC_STALE_S = 2.0          # no new 0x500 frame for this long = no signal (모름): the confirmed value stays; a 5 s
                           # hold in progress is dropped, never completed, and restarts from 0 when frames return
C1_LOGIC = {"stop_kph": 1.0, "move_kph": 3.0, "occ_stable_s": OCC_STABLE_S, "empty_after_s": 10.0}
STATES = ("EMPTY", "BOARD", "DRIVE", "PAY")
# meter_improve (보노보노 20:46 "(초기화)시작.일시정지.정산(정산화면따로만들기) 버튼만들어", 21:20 "...누르면되는 어플하나만들어"):
# a phone / Carlinkit-box app with three buttons talks UDP 7720 to carrot_man (selfdrive/carrot/k9_meter_link.py, only
# while K9ExtHud=1), which queues the press in the memory param K9MeterCmd ("token:cmd;..."); the HUD applies new tokens
# (MeterButtonLink) and reports K9MeterState (JSON) back, which the app shows. 시작 = reset to 0 (base fare, 0 km, 0 s) and count (주행), from any state; 일시정지 = stop counting /
# go on (주행 only); 정산 = stop counting and show the separate 정산 screen (SETTLE) until the next 시작.
METER_CMD_PARAM = "K9MeterCmd"
METER_STATE_PARAM = "K9MeterState"
# meter_empty2 (보노보노 21:59 "손님탐도없애", 22:01 "빈차창도 따로만들고 어플에 빈차도 넣어바"): with the passenger sensor off
# (OCCUPANT_ENABLED False) the meter is the app's alone - it starts in 빈차 (its own screen, _draw_empty) and nothing but
# the buttons changes the state: 시작 = 주행 at once (no 손님탐), 정산 = the 정산 screen, 빈차 = back to 빈차 (reset: base fare,
# 0 km, 0 s) from any state. The onroad-driven legacy meter (no 0x500 frame ever) is only used with the sensor on.
METER_COMMANDS = ("start", "pause", "settle", "empty")
SETTLE = "SETTLE"


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
    self.was_stopped = True if stopped else (False if moving else self.was_stopped)
    occ_edge = None
    occ = sig.get("occ")
    if self.occ_on and occ is None and self.occ_raw != self.occ:
      self.occ_raw, self.rise_ok = self.occ, False   # signal lost (모름) during a 5 s hold: drop it, never complete it
    if self.occ_on and occ is not None:
      occ = bool(occ)
      if occ != self.occ_raw:
        self.occ_raw, self.occ_t, self.rise_ok = occ, self.t, stopped
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
      # meter_empty2: no 손님탐 any more when the sensor is off - 빈차 -> 주행 at once (as the app's 시작)
      nxt = {"EMPTY": "BOARD" if self.occ_on else "DRIVE", "BOARD": "EMPTY", "DRIVE": "PAY", "PAY": "EMPTY",
             SETTLE: "EMPTY"}[self.state]
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
      pass                                           # meter_improve 20:43: the passenger no longer ends a drive
    elif self.state == "PAY":                        # (지불 only from the toggle) ... and no 0->1 -> 손님탐 from 지불
      if self.t - self.since >= cfg["empty_after_s"]:
        self._set("EMPTY", "timer", sig)
      elif moving and self.pay_stopped:
        self._set("EMPTY", "move", sig)
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


def _c4(c) -> tuple[int, int, int, int]:
  c = tuple(int(v) for v in c)
  return c if len(c) == 4 else (*c[:3], 255)


C1_THEME = {
  "night": {"g2": "#d8b25e", "dsh": "#07060f", "lb": "#a8915f", "tx": "#f2ead8", "sub": "#b9a77f", "unit": "#d9bd7c",
            "acc": "#f0d690", "nx": "#a8915f", "pillB": "#c9a457", "pillT": "#f0d690", "pillBg": "#00000033",
            "plus": "#ffe08a", "trk0": "#2b2618", "tk0": "#7a5f27", "tk1": "#f1d68f", "stBg": "#0d0b1ce0", "stRow": "#ffffff14"},
  "dusk": {"g2": "#dcb35c", "dsh": "#0c0610", "lb": "#b4955e", "tx": "#f6ead6", "sub": "#c4a77a", "unit": "#e0bf7a",
           "acc": "#f4d48a", "nx": "#c19b66", "pillB": "#c9a457", "pillT": "#f4d690", "pillBg": "#00000033",
           "plus": "#ffe08a", "trk0": "#2b2618", "tk0": "#7a5f27", "tk1": "#f1d68f", "stBg": "#160b16e0", "stRow": "#ffffff14"},
  "day": {"g2": "#1d3a5f", "dsh": "#ffffffcc", "lb": "#3d6a92", "tx": "#1d3a5f", "sub": "#4f7ea8", "unit": "#2b5c8a",
          "acc": "#d2552a", "nx": "#6b4f2a", "pillB": "#2b5c8a", "pillT": "#2b5c8a", "pillBg": "#fffdf4cc",
          "plus": "#ff7a45", "trk0": "#e0b97f", "tk0": "#ffb07a", "tk1": "#ff6a3d", "stBg": "#fffdf4e6", "stRow": "#2b5c8a14"},
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
NEXT_MSG = {"EMPTY": "손님을 기다리는 중 · 기본 {base}원 ({km}km)", "BOARD": "어서 오세요 · 출발하면 요금이 올라갑니다",
            "PAY": "감사합니다 · 카드·현금 모두 됩니다 · 안녕히 가세요"}
SAY = {"BOARD": ("어서 오세요!", 0.1, 2.6), "DRIVE": ("출발~!", 0.1, 2.2), "PAY": ("감사합니다!", 0.1, 3.6),
       "EMPTY": ("또 만나요!", 0.1, 2.6)}
# res (10/24 17:47 '말풍선 다시 넣어라'): the C1.html main-loop lines of 포로리 (core.js bubbles(): psay), live.
# '괴롭힐 거야?' = 0.15..2.4 s after every fare step (HTML: win(t, T1+.15, T1+2.4), T1 = the +200원 moment), and
# '또 멈췄네?' when the car comes to a stop while 주행 (HTML: the end of the loop, when the car stops). 보노보노's
# thought bubble (bth) is back since the bono round (18:37), see WAIT_THINK / TICK_THINK below.
TICK_SAY = ("괴롭힐 거야?", 0.15, 2.4)
STOP_SAY = ("또 멈췄네?", 0.15, 2.6)
# bono (10/24 18:37 '보노보노 말풍선도 넣어라', supersedes 10:08 for these lines): C1.html's only 보노보노 lines are
# the thought bubble bth of core.js bubbles(): show = max(win(t,.2,3.0), win(t,T1+2.3,T1+4.1)), text t<T1 '언제 가는
# 걸까…' else '또 올라가는 걸까…' (10 s loop: the car waits stopped at 0-2 s right after 포로리's '또 멈췄네?', T1 = the
# +200원 moment). Live, one bubble at a time with 포로리 (bubble_now() keeps priority and is unchanged):
#  WAIT_THINK while 주행 and stopped, right after STOP_SAY: 2.75..5.55 s after the stop starts (HTML length 2.8 s), once
#   per stop, only if the car is still stopped at 2.75 s; it runs to its end if the car pulls away (HTML: 1 s into it);
#  TICK_THINK 2.4..4.2 s after each fare step, right after TICK_SAY (HTML 2.3..4.1, moved 0.1 s so they never overlap).
# No bubble on steering is added (the 10:08 핸들 말풍선 '끼익' stays out).
WAIT_THINK = ("언제 가는 걸까…", 2.75, 5.55)
TICK_THINK = ("또 올라가는 걸까…", 2.4, 4.2)

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
GOAL_SPRITE = "acorn"         # res (10/24 17:47 '도토리로 바꿔'): the goal is a 도토리 (acorn.png, baked at scale 1)
FARE_BOX = (14.0, 84.0, 410.0, 222.0)   # particles are not drawn over the fare digits
PLUS_RIGHT = 538.0                      # '+100원' ends left of the stats glass (x 546) with an 8 px gap (meter_improve)
SWITCH_FADE_S = 0.1                     # meter_improve: a bubble cut by a state change fades out in this time (the
                                        # next state's line starts 0.1 s after the change) instead of vanishing
NO_PARTICLE_BOXES = (FARE_BOX, (NIGHT_LAMP_X - 10, NIGHT_LAMP_Y - 10, NIGHT_LAMP_X + NIGHT_LAMP_W + 10,
                                NIGHT_LAMP_Y + NIGHT_LAMP_H + 10),   # ... nor over the night lamp,
                     (546.0, 72.0, 758.0, 236.0))   # ... nor over the distance / time numbers (C1.html had only the fare)
BUBBLE_FILL = (255, 253, 246)
BUBBLE_TEXT = (43, 52, 72)
BONO_PAL = {"f": (92, 188, 255), "o": (18, 48, 79)}
# moving layer of the bg2 backgrounds (13:34): out/taximeter/variants/bg2/ptcl_params.json, copied to the assets.
# Shapes = ptcl_white.png (white version of bg2/ptcl_sprites.png: petal leaf ginkgo snow fly, 64 px cells, 4x).
PTCL_KINDS = ("petal", "leaf", "ginkgo", "snow", "fly")
PTCL_CELL, PTCL_DRAW = 64.0, 16.0
PTCL_TOD = {"day": "day", "dusk": "sunset", "night": "night"}
PTCL_EDGE_FADE = 24.0   # r3: alpha 1 -> 0 over the last 24 px before the card edge (sprite centre), scissor at the card
PTCL_BOX_FADE = 12.0    # ... and over the last 12 px before a no-particle box (inside the box: 0, as in C1.html)


def _smooth01(v: float) -> float:
  v = 0.0 if v <= 0.0 else (1.0 if v >= 1.0 else v)
  return v * v * (3.0 - 2.0 * v)


def particle_alpha(px: float, py: float) -> float:
  """Opacity multiplier of a particle centred at card (px, py): 0 at / beyond the card edge and inside a
  no-particle box, smooth ramp to 1 over PTCL_EDGE_FADE px from the edge / PTCL_BOX_FADE px from a box."""
  a = _smooth01(min(px, py, CARD_W - px, CARD_H - py) / PTCL_EDGE_FADE)
  for b in NO_PARTICLE_BOXES:
    if a <= 0.0:
      break
    d = max(b[0] - px, px - b[2], b[1] - py, py - b[3])   # > 0 outside the box (Chebyshev distance)
    a *= _smooth01(d / PTCL_BOX_FADE)
  return a


def _load_ptcl_params() -> dict:
  try:
    d = json.loads((C1_ASSET_DIR / "ptcl_params.json").read_text())["seasons"]
  except Exception:
    return {}
  out = {}
  for season, tods in d.items():
    for tod, q in tods.items():
      n = int(q.get("n") or 0)
      if n <= 0:
        continue
      kinds = q["k"] if isinstance(q["k"], list) else [q["k"]]
      cols = [_hex(c if len(c) != 4 else "#" + "".join(ch * 2 for ch in c[1:]))[:3] for c in q["c"]]
      out[(season, tod)] = {"n": n, "k": kinds, "c": cols, "x0": q.get("x0", 420), "xw": q.get("xw", 330),
                            "y0": q.get("y0", 250), "yh": q.get("yh", 120)}
  return out


PTCL_PARAMS = _load_ptcl_params()

# r3: animated scene elements of the bg2 HTML. anim.json[season][tod] = [{kind tw|dr, i, v, part back|front,
# x, y, w, h (card px of the sprite), sx, sy (atlas px)}], atlas anim_{season}.png (2 px transparent gutter).
ANIM_TW_MAX = 14   # c1bg.js: only the first 14 data-tw of a time of day twinkle (the rest stay as drawn)


def _load_anim() -> dict:
  try:
    return json.loads((C1_ASSET_DIR / "anim.json").read_text())["seasons"]
  except Exception:
    return {}


ANIM = _load_anim()


def anim_elements(season: str, tod: str, t: float, part: str | None = None):
  """[(entry, dx, alpha)] for the time of day = c1bg.js draw(): twinkle opacity v*(.6+.4 sin(t(1.1+.37i)+2.1i)),
  drift x sin(.12t+1.7i)*v (i = index in its list, back then front)."""
  out = []
  for e in ANIM.get(season, {}).get(PTCL_TOD.get(tod, tod), ()):
    if part is not None and e["part"] != part:
      continue
    i = e["i"]
    if e["kind"] == "tw":
      out.append((e, 0.0, e["v"] * (.6 + .4 * math.sin(t * (1.1 + i * .37) + i * 2.1))))
    else:
      out.append((e, math.sin(t * .12 + i * 1.7) * e["v"], 1.0))
  return out


def particle_positions(season: str, tod: str, t: float):
  """[(kind, x, y, rot_deg, color, opacity)] in card px for the current tod, = c1bg.js draw() (bg2)."""
  q = PTCL_PARAMS.get((season, PTCL_TOD.get(tod, tod)))
  if not q:
    return []
  out = []
  for i in range(q["n"]):
    k = q["k"][i % len(q["k"])]
    col = q["c"][i % len(q["c"])]
    if k == "fly":
      x = q["x0"] + ((i * 97) % q["xw"]) + math.sin(t * .5 + i * 2.3) * 26
      y = q["y0"] + ((i * 37) % q["yh"]) + math.sin(t * .7 + i * 1.3) * 14
      out.append((k, x, y, 0.0, col, .25 + .75 * max(0.0, math.sin(t * 1.6 + i * 1.9))))
      continue
    sp = 22 + (i % 4) * 5 if k == "snow" else (18 + (i % 3) * 5 if k == "petal" else 24 + (i % 3) * 6)
    y = ((i * 53 + t * sp) % 500) - 22
    x = (i * 83.7 + math.sin(t * .9 + i * 1.7) * (10 if k == "snow" else 22) + t * (3 if k == "snow" else 9)) % 800 - 12
    rot = 0.0 if k == "snow" else (t * (60 + i * 13) + i * 40) % 360
    out.append((k, x, y, rot, col, 0.9))
  return out


# LFA state on the clam (13:20 지시 2; the carrot wheel is not drawn on this screen): LFA off = clam dimmed,
# small badge in the sea under 보노보노 = lane icon (solid green: lane lines in use, faint: laneless) + "LFA".
CLAM_OFF_TINT = (190, 190, 190, 120)       # no longer used (meter_improve: the clam always keeps its own colours)
LFA_BADGE = (664.0, 412.0, 48.0, 24.0)      # card px (x, y, w, h): right of the footer (ends x 645), under the clam;
                                            # meter_improve: 'LFA' text removed, the pill now fits the lane icon (was 92 wide)
LFA_ON_COLOR = (64, 200, 112)
LFA_OFF_COLOR = (150, 150, 150)

# CPU (13:20 지시 4): only the pixels that can change are sent to the rasterizer. Sprites are drawn from their
# alpha bounding box (+2 px for the bilinear edge) instead of the whole 240x240 quad; the 'over' layer (45 %
# opaque) is drawn as runs of non-empty 16 px tiles. Same texels -> same picture (pixel-compared).
C1_CROP_PAD = 2
C1_RUN_TILE = 16

# ---- resolution (res, 10/24 17:45 '글씨 같은 게 잘 안 보인다') -----------------------------------------------
# The HUD is laid out on a 1920x480 design canvas and drawn with rl_scalef(width/1920, height/480). The car's
# TURZX 9.2 panel is 1920x462 (lcd_comm_turing_usb PRODUCT_ID 0x0092 = (462, 1920), landscape), so the card used
# to be squashed to 776x438.9 px (sy = 0.9625): every baked 776x456 layer and every glyph was resampled
# between pixel rows. The card is now drawn in device pixels: 1:1 (k = 1) whenever the 456 px card fits the
# frame height, at an integer device origin, centred on its design slot; otherwise uniform k = min(sx, sy).
C1_NATIVE_MIN_SCALE = 0.90      # sx/sy down to this: snap the card to 1:1 device pixels (fits 1920x462)
# Text: the shared fonts are atlases of 32 px (Korean, 11k glyphs) and 160 px (Latin) drawn with bilinear
# filtering and no mipmaps, so 13-17 px text was minified 0.4-0.5x and the 27 px digits 0.17x (only a few
# texels of each glyph sampled = thin, broken strokes). C1 now loads its own small atlases, one per pixel size
# it uses, with only the characters it draws (stb_truetype rasterises them at that size), and puts every
# glyph run on an integer device pixel.
C1_TEXT_SHADOW_MAX = 17.0       # live text up to this size on the scene gets a 1 px theme shadow (contrast)


def c1_card_device_rect(width: float, height: float, x: float, y: float, design_w: float = 1920.0,
                        design_h: float = 480.0):
  """(dev_x, dev_y, k) for a card whose design top-left is (x, y): integer origin, 1:1 when it fits."""
  sx, sy = float(width) / design_w, float(height) / design_h
  k = min(sx, sy)
  if k < 1.0 and k >= C1_NATIVE_MIN_SCALE and CARD_H <= float(height) and CARD_W <= float(width):
    k = 1.0
  cx, cy = (x + CARD_W * 0.5) * sx, (y + CARD_H * 0.5) * sy
  dx, dy = round(cx - CARD_W * k * 0.5), round(cy - CARD_H * k * 0.5)
  if k == 1.0:
    dy = max(0, min(int(height) - int(CARD_H), dy))
    dx = max(0, min(int(width) - int(CARD_W), dx))
  return float(dx), float(dy), k


PAUSE_PILL = ("일시정지", (255, 168, 64))
PAUSE_MSG = "일시정지 · 요금·거리·시간이 멈춰 있어요"
SETTLE_STRINGS = ("정 산", "총 요 금", "보노보노 택시 · 오늘도 고마워요", "기본요금", "거리요금", "시간요금", "심야 할증", "없음",
                  "회", "×", "초", "운 행  거 리", "운 행  시 간", "시작을 누르면 새 운행을 시작합니다", "감사합니다!", "원", "km")
EMPTY_BRAND = "보노보노 택시 · 어서 오세요"
EMPTY_WAIT = "손님을 기다리는 중"
EMPTY_HINT = "시작을 누르면 바로 주행을 시작합니다"
EMPTY_CALL = ("어디로 모실까요?", 6.0, 8.6, 14.0)   # 포로리 on the 빈차 screen: (text, from s, to s, period s)
EMPTY_STRINGS = ("빈 차", EMPTY_BRAND, EMPTY_WAIT, EMPTY_HINT, EMPTY_CALL[0], "당", "시", "~", "km/h 이하", " · 심야)",
                 " · 지금 적용", "없음")


def c1_text_strings():
  """Every string the C1 panel draws live (the font atlases hold only these characters + ASCII)."""
  out = list(LABEL.values()) + [m for m in NEXT_MSG.values()] + [v[0] for v in SAY.values()] + \
    [v[0] for v in PILL.values()] + [TICK_SAY[0], STOP_SAY[0], WAIT_THINK[0], TICK_THINK[0], "대기", "주행을 시작하면 요금이 올라갑니다", "원", "딱!",
                                    "다음 까지 초 현재 속도 km/h", "심야 할증 %", "·", "LFA"] + list(SETTLE_STRINGS) + [PAUSE_PILL[0], PAUSE_MSG] + \
    list(EMPTY_STRINGS)
  return out


def c1_codepoints() -> list[int]:
  cps = set(range(0x20, 0x7F))
  for s in c1_text_strings():
    cps.update(ord(ch) for ch in s)
  return sorted(cps)


def _png_alpha(path):
  try:
    import numpy as np
    from PIL import Image
    with Image.open(path) as im:
      return np.asarray(im.convert("RGBA"))[:, :, 3]
  except Exception:
    return None


def alpha_bbox(alpha, pad: int = C1_CROP_PAD):
  """(x, y, w, h) of the pixels with alpha > 0, padded; None = keep the whole texture."""
  if alpha is None:
    return None
  import numpy as np
  ys, xs = np.nonzero(alpha)
  h, w = alpha.shape
  if not len(xs):
    return (0, 0, 1, 1)
  x0, y0 = max(0, int(xs.min()) - pad), max(0, int(ys.min()) - pad)
  x1, y1 = min(w, int(xs.max()) + 1 + pad), min(h, int(ys.max()) + 1 + pad)
  return (x0, y0, x1 - x0, y1 - y0)


def alpha_runs(alpha, tile: int = C1_RUN_TILE, pad: int = C1_CROP_PAD):
  """Rectangles (x, y, w, h) covering every pixel with alpha > 0 (dilated by pad): non-empty tiles, merged
  along a tile row, equal consecutive rows merged. None = draw the whole texture."""
  if alpha is None:
    return None
  import numpy as np
  h, w = alpha.shape
  m = alpha > 0
  if pad:
    d = m.copy()
    for k in range(1, pad + 1):
      d[k:, :] |= m[:-k, :]
      d[:-k, :] |= m[k:, :]
    m = d.copy()
    for k in range(1, pad + 1):
      d[:, k:] |= m[:, :-k]
      d[:, :-k] |= m[:, k:]
    m = d
  nty, ntx = (h + tile - 1) // tile, (w + tile - 1) // tile
  rows = []
  for ty in range(nty):
    band = m[ty * tile:(ty + 1) * tile]
    occ = [bool(band[:, tx * tile:(tx + 1) * tile].any()) for tx in range(ntx)]
    spans, tx = [], 0
    while tx < ntx:
      if occ[tx]:
        s = tx
        while tx < ntx and occ[tx]:
          tx += 1
        spans.append((s * tile, min(w, tx * tile)))
      else:
        tx += 1
    rows.append(tuple(spans))
  out, ty = [], 0
  while ty < nty:
    s = ty
    while ty + 1 < nty and rows[ty + 1] == rows[s]:
      ty += 1
    y0, y1 = s * tile, min(h, (ty + 1) * tile)
    out.extend((x0, y0, x1 - x0, y1 - y0) for x0, x1 in rows[s])
    ty += 1
  return out


def _rot(px: float, py: float, deg: float) -> tuple[float, float]:
  a = math.radians(deg)
  c, s = math.cos(a), math.sin(a)
  return px * c - py * s, px * s + py * c


def _sm(x: float) -> float:
  x = min(1.0, max(0.0, x))
  return x * x * (3 - 2 * x)


def _win(t: float, a: float, b: float, f: float = 0.25) -> float:
  return min(1.0, max(0.0, min((t - a) / f, (b - t) / f)))


# 정산 screen layout (card px)
SETTLE_PANEL = (16.0, 16.0, CARD_W - 32.0, 312.0)     # panel: title, total, lines, distance / time (above the characters)
SETTLE_ROWS = (440.0, 744.0, 104.0)                    # lines: x from, x to (amount right edge), first row centre y
SETTLE_ROW_H = 34.0
SETTLE_HINT = (210.0, 424.0)                           # "시작을 누르면 ..." right of 포로리, under its bubble
SETTLE_PORO = (120.0, 380.0)                           # 포로리 anchor (bows), its bubble to the right
SETTLE_BUBBLE_DY = 380.0 - 236.0 - 46.0                # the bubble at 포로리's head height
# 빈차 screen layout (card px, meter_empty2): the 정산 screen's panel / 포로리 / hint places, a red sign left, fare rows right
EMPTY_PANEL = SETTLE_PANEL
EMPTY_SIGN = (36.0, 82.0, 352.0, 196.0)               # the '빈 차' roof-lamp sign (x, y, w, h)
EMPTY_SIGN_TEXT_Y = 82.0                               # '빈 차' 98 px centre, from the sign top
EMPTY_SIGN_SUB_Y = 160.0                               # '손님을 기다리는 중' centre, from the sign top
EMPTY_ROWS = SETTLE_ROWS
EMPTY_RED = (255, 90, 74)                              # PILL['EMPTY'] red
EMPTY_SIGN_BG = (22, 8, 10, 240)
EMPTY_GLOW = 5.0                                       # glow ring around the sign (card px)

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


# 보노보노's thought bubble (C1.html .bub.th): left of 보노보노's head, its two dots going down-right to the head.
THINK_RIGHT = 600.0   # right edge (C1.html right:162 = x 614; kept at the 포로리 limit: the head starts at x ~612)
THINK_Y = 238.0       # top (C1.html 228 - bob*0.3; the stats glass ends at y 229, its no-draw box at 236)
THINK_BOB = 0.3       # follows 보노보노's bob (+-4.5 px) x 0.3, as C1.html
# dots (dx from the right edge, dy from the bottom, r): C1.html <i> 12 px right:14 bottom:-14, 7 px right:2 bottom:-24
THINK_DOTS = ((-20.0, 8.0, 6.0), (-5.5, 20.5, 3.5))
PORO_REACH = (-66.0, 30.0)   # 포로리's x extent around its anchor (tail .. waving arm), card px
THINK_PORO_FADE = 24.0       # the thought bubble fades out over the last 24 px before 포로리 would walk under it


def think_rect(text_w: float, bob: float = 0.0):
  """보노보노 thought bubble (card px): (x, y, w, h)."""
  w = text_w + 28
  return THINK_RIGHT - w, THINK_Y - bob * THINK_BOB, w, BUBBLE_H


def think_parts(text_w: float, bob: float = 0.0):
  """(x0, y0, x1, y1) boxes of what the thought bubble draws: the box with its drop shadow (3 px out, 0.5 px below
  the top), then each dot with its shadow (r + 1, 1 px down)."""
  bx, by, w, h = think_rect(text_w, bob)
  out = [(bx - 3.0, by, bx + w + 3.0, by + h + 6.5)]
  for ddx, ddy, r in THINK_DOTS:
    cx, cy = bx + w + ddx, by + h + ddy
    out.append((cx - r - 1.0, cy - r, cx + r + 1.0, cy + r + 2.0))
  return out


def think_extent(text_w: float, bob: float = 0.0):
  """(x0, y0, x1, y1) around everything the thought bubble draws."""
  ps = think_parts(text_w, bob)
  return min(p[0] for p in ps), min(p[1] for p in ps), max(p[2] for p in ps), max(p[3] for p in ps)


def think_poro_fade(text_w: float, x_char: float) -> float:
  """1 = 포로리 is well left of the thought bubble, 0 = it would be under it."""
  gap = think_rect(text_w)[0] - 6.0 - (x_char + PORO_REACH[1])
  return min(1.0, max(0.0, gap / THINK_PORO_FADE))


# meter_improve (10/24 19:04 '미터기 더 고쳐'): when 포로리 would walk under the thought bubble (high progress) the bubble
# used to fade away. It now takes a second place instead: below the track, left of 보노보노's chest (between the
# 도토리 / next line and the footer), its two dots going UP-right to the head (the C1.html dots mirrored). The place is
# chosen once when a thought starts (from where 포로리 will be when it ends) and kept for that thought.
THINK_LOW_Y = 346.0          # top of the lower place (도토리 ends y ~340, the 다음 line is x <= ~380, footer from y ~405)
THINK_LOW_DOTS = ((-20.0, -8.0, 6.0), (-5.5, -20.5, 3.5))   # (dx from the right edge, dy from the TOP, r)
THINK_LOW_TEXT_GAP = 14.0    # the lower place needs this much room right of the 다음 line
# fare steps close together (meter_improve item 2; ~110 km/h and up the 131 m steps come < 4.2 s apart): the
# '또 올라가는 걸까…' window is ended THINK_STEP_MARGIN s before the next step the meter predicts (its fade-out done
# by then), and left out for that step when less than THINK_MIN_S would remain; a thought cut by a step anyway (the
# car sped up) fades out in the 0.15 s before 포로리's line instead of vanishing.
THINK_STEP_MARGIN = 0.05
THINK_MIN_S = 1.0


def think_rect_low(text_w: float, bob: float = 0.0):
  """보노보노 thought bubble, lower place (card px): (x, y, w, h)."""
  w = text_w + 28
  return THINK_RIGHT - w, THINK_LOW_Y - bob * THINK_BOB, w, BUBBLE_H


def think_parts_low(text_w: float, bob: float = 0.0):
  """(x0, y0, x1, y1) boxes of the lower place: the box with its drop shadow, then each dot with its shadow."""
  bx, by, w, h = think_rect_low(text_w, bob)
  out = [(bx - 3.0, by, bx + w + 3.0, by + h + 6.5)]
  for ddx, ddy, r in THINK_LOW_DOTS:
    cx, cy = bx + w + ddx, by + ddy
    out.append((cx - r - 1.0, cy - r, cx + r + 1.0, cy + r + 2.0))
  return out


def think_low_fits(text_w: float, next_line_right: float) -> bool:
  """The lower place is free: right of the 다음 line (card px of its right end) by THINK_LOW_TEXT_GAP."""
  return think_rect_low(text_w)[0] - 3.0 >= next_line_right + THINK_LOW_TEXT_GAP


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

  def __init__(self, meter: TaxiMeter | None = None, clock=time.monotonic, local_hour=None, wall_clock=None,
               occupant: bool | None = None) -> None:
    self.meter = meter or TaxiMeter(TAXI_METER_C1_FARE)   # res: 경기 일반(중형) 요금 (공고 제2023-5572호)
    self.anim = MeterAnimation(self.meter.fare.time_fare_below_kph)
    self._occ_on = OCCUPANT_ENABLED if occupant is None else bool(occupant)   # None = OCCUPANT_ENABLED (car: off)
    self.fsm = C1Fsm(C1_LOGIC, self._occ_on)
    self.fsm_mode = not self._occ_on   # sensor on: False until a 0x500 frame (or the toggle) is seen; off: button meter
    self._speed_samples: list[tuple[float, float]] = []
    self._speed_samples_live = False
    self._clock = clock
    self._local_hour = local_hour or seoul_hour
    self._wall_clock = wall_clock   # -> (sec of day, month) | None; None = Asia/Seoul clock (or the injected hour)
    self.season_setting = 0
    self.night_rate = 0.0
    self.steering_angle_deg = 0.0
    self.speed_kph = 0.0
    self.lfa_active = False
    self.lane_line = False
    self._crop: dict[str, tuple] = {}
    self._runs: dict[str, list] = {}
    self._anim_runs: dict = {}
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
    self._fonts: dict[int, object] = {}
    self._measure_cache: dict = {}
    self._font_path = None
    self._font_failed = False
    self._dev = None              # (dev_x, dev_y, k, card_x, card_y) of the frame being drawn
    self._stop_since: float | None = None   # wall time the car came to a stop while 주행 (STOP_SAY)
    self.last_bubble = ""
    self._think_stop: float | None = None    # the stop (its _stop_since) that got WAIT_THINK
    self.last_think = ""
    self.last_think_slot = ""
    self.paused = False                      # 일시정지 (button): the meter does not count while 주행 is paused
    self._cmds: list[str] = []
    self.settle_breakdown: dict | None = None   # 정산 screen numbers, frozen at the 정산 press
    self._bd = self._new_breakdown(0.0)
    self._think_key = None                   # the thought on screen: ("tick", step time) / ("wait", stop time)
    self._think_slot_key = None
    self._think_slot = "up"
    self._tick_skip = None                   # step time of a fare step whose thought is left out (too short)
    self._tick_decided = None                # step time whose thought window was looked at
    self._think_last = None                  # (text, opacity, t, key) of the last frame that showed a thought
    self._think_carry = None                 # (text, opacity, step time): a thought cut by a fare step, fading out
    self._think_t = None                     # time of this / the previous think_now() call
    self._say_last = None                    # (text, opacity, t) of 포로리's bubble in the last frame (None: none)
    self._say_st = None
    self._say_t = None
    self._say_switch = None                  # (text, opacity, t of the state change): 포로리's line fading out
    self._think_switch = None                # the same for a thought cut by leaving 주행
    self._think_prev_t = None

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
    """Latest 0x500 bus0 CF_TTL_Ind (True/False), None = no frame. Stale after OCC_STALE_S. Sensor off: ignored."""
    if present is None or not self._occ_on:
      return
    self.occupant = bool(present)
    self.occupant_t = self._clock() if t_s is None else float(t_s)

  def toggle(self) -> None:
    """Screen toggle (always wins). Sensor on: 빈차->손님탐, 손님탐->빈차, 주행->지불, 지불->빈차. Sensor off (meter_empty2):
    the app's next button - 빈차 -> 시작, 주행 / 일시정지 -> 정산, 정산 -> 빈차."""
    self._tap = True

  def command(self, cmd: str) -> None:
    """A meter button (K9MeterCmd): 'start' / 'pause' / 'settle' / 'empty', applied on the next update()."""
    if cmd in METER_COMMANDS:
      self._cmds.append(cmd)

  def meter_status(self) -> dict:
    """K9MeterState for the remote app: state + shown fare, km, s, night rate."""
    m = self.meter
    return {"state": self.meter_state, "fare": int(self.shown_fare()), "km": round(m.distance_m / 1000.0, 2),
            "sec": int(m.elapsed_s), "night": round(self.night_rate, 2)}

  @property
  def meter_state(self) -> str:
    """K9MeterState: EMPTY / DRIVE / PAUSE / SETTLE (sensor on also BOARD / PAY; legacy: DRIVE or EMPTY)."""
    st = self.display_state
    if st == "EMPTY_LEGACY":
      return "EMPTY"
    return "PAUSE" if st == "DRIVE" and self.paused else st

  def _new_breakdown(self, rate: float) -> dict:
    return {"dist_units": 0, "time_units": 0, "base_rate": rate}

  def _apply_command(self, cmd: str, hour) -> None:
    f = self.fsm
    if cmd == "start":                          # 초기화 후 시작: 0 km, 0 s, base fare, counting
      self.fsm_mode = True
      self.meter.reset(hour)
      self.meter._was_onroad = True
      self._bd = self._new_breakdown(night_surcharge_rate(self.meter.fare, hour))
      self.paused = False
      self.settle_breakdown = None
      f._set("DRIVE", "start", {"v_kph": self.speed_kph, "fare": self.meter.fare_won})
    elif cmd == "pause":
      if self.fsm_mode and f.state == "DRIVE":
        self.paused = not self.paused
        self.meter._last_sample_t = None        # no time base across the pause: nothing is counted for it
        self.meter._last_now = None
    elif cmd == "settle":
      if self.fsm_mode and f.state == "DRIVE":
        self.paused = False
        self.settle_breakdown = self.breakdown()
        f._set(SETTLE, "settle", {"v_kph": self.speed_kph, "fare": self.meter.fare_won})
        self.meter._last_sample_t = None
        self.meter._last_now = None
    elif cmd == "empty":                        # meter_empty2 빈차: back to 빈차 from any state (reset: base fare, 0 km, 0 s)
      self.fsm_mode = True
      self.meter.reset(hour)
      self.meter._was_onroad = False
      self._bd = self._new_breakdown(night_surcharge_rate(self.meter.fare, hour))
      self.paused = False
      self.settle_breakdown = None
      self.meter._last_sample_t = None
      self.meter._last_now = None
      f._set("EMPTY", "empty", {"v_kph": self.speed_kph, "fare": self.meter.fare_won})

  def breakdown(self) -> dict:
    """정산 numbers of the current drive: base, distance / time steps, night surcharge, total (shown fare), km, s."""
    m, fare, bd = self.meter, self.meter.fare, self._bd
    base = fare.base_fare_won
    dist = bd["dist_units"] * fare.distance_unit_won
    tim = bd["time_units"] * fare.time_unit_won
    total = m.fare_won
    night = max(0, total - base - dist - tim)   # the rest of the shown (10 won rounded) total = 심야 할증
    return {"total": int(total), "base": int(base), "base_km": fare.base_distance_m / 1000.0,
            "dist_units": int(bd["dist_units"]), "dist_won": int(dist), "dist_unit_m": fare.distance_unit_m,
            "time_units": int(bd["time_units"]), "time_won": int(tim), "time_unit_s": fare.time_unit_s,
            "unit_won": int(fare.distance_unit_won), "night_won": int(night), "night_rate": bd["base_rate"],
            "km": m.distance_m / 1000.0, "elapsed_s": m.elapsed_s}

  def _count(self, fn, *args) -> None:
    """One meter call while 주행; its new steps go to the 정산 breakdown (by the mode that charged them)."""
    added = fn(*args) or 0
    if added > 0:
      key = "time_units" if self.meter.mode == "time" else "dist_units"
      self._bd[key] += added

  def set_season_setting(self, value) -> None:
    self.season_setting = normalize_meter_season(value)

  def restart(self) -> None:
    self._speed_samples = []
    self.meter.reset()
    self.meter._was_onroad = False
    self.anim.reset()
    self.fsm = C1Fsm(C1_LOGIC, self._occ_on)
    self.fsm_mode = not self._occ_on
    self._last_t = None
    self._tap = False
    self.paused = False
    self._cmds = []
    self.settle_breakdown = None
    self._bd = self._new_breakdown(0.0)

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
    self.lfa_active = bool(getattr(state, "lfa_active", False))       # same rule as the carrot wheel icon
    self.lane_line = bool(getattr(state, "active_lane_line", False))
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
    cmds, self._cmds = self._cmds, []
    if tap and not self.fsm.occ_on:   # meter_empty2: the toggle = the app's next button (no 손님탐 / 지불)
      tap = False
      st0 = self.fsm.state if self.fsm_mode else "EMPTY"
      cmds.append({"EMPTY": "start", "DRIVE": "settle"}.get(st0, "empty"))
    for cmd in cmds:
      self._apply_command(cmd, hour)
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
      self._bd = self._new_breakdown(night_surcharge_rate(self.meter.fare, hour))
    if st != "DRIVE":
      self.paused = False
    samples, self._speed_samples = self._speed_samples, []
    if st == "DRIVE" and not self.paused:       # 주행 = moving with a passenger: counts (onroad flag not needed)
      if self._speed_samples_live:
        for t_s, v in samples:
          self._count(self.meter.feed_sample, t_s, v, hour, now)
      else:
        dtm = 0.0 if self.meter._last_now is None else now - self.meter._last_now
        self.meter._last_now = now
        self._count(self.meter.step, dtm, speed, hour, now)
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
    if st in ("DRIVE", SETTLE):
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
    if tex is not None and not name.startswith("back_") and not name.startswith("anim_"):
      alpha = _png_alpha(path)
      if alpha is not None and alpha.shape == (tex.height, tex.width):
        if name.startswith("over_"):
          self._runs[name] = alpha_runs(alpha)
        else:
          self._crop[name] = alpha_bbox(alpha)
    self._textures[name] = tex
    return tex

  def _drop_season(self, rl, keep: str) -> None:
    for name in list(self._textures):
      if ((name.startswith("back_") or name.startswith("over_")) and f"_{keep}_" not in name) or \
         (name.startswith("anim_") and name != f"anim_{keep}"):
        tex = self._textures.pop(name)
        self._runs.pop(name, None)
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
    for name in ("porori_body", "porori_tail", "porori_fL", "porori_fR", "porori_aL", "porori_aR", "bono_body", "shell",
                 "ptcl_white", GOAL_SPRITE):
      self._tex(rl, name)
    self._load_fonts(renderer, rl)
    if ANIM.get(self.season):
      self._anim_atlas(rl)

  def _anim_atlas(self, rl):
    name = f"anim_{self.season}"
    if name in self._textures:
      return self._textures[name]
    tex = self._tex(rl, name)
    runs = {}
    if tex is not None:
      alpha = _png_alpha(C1_ASSET_DIR / f"{name}.png")
      if alpha is not None and alpha.shape == (tex.height, tex.width):
        for tod, lst in ANIM[self.season].items():
          for e in lst:
            sx, sy, w, h = int(e["sx"]), int(e["sy"]), int(e["w"]), int(e["h"])
            runs[(tod, e["kind"], e["i"])] = alpha_runs(alpha[sy:sy + h, sx:sx + w])
    self._anim_runs = runs
    return tex

  # -- crisp text (res)
  C1_FONT_SIZES = (13, 14, 15, 16, 17, 22, 27, 30, 98)

  def _load_fonts(self, renderer, rl) -> None:
    if self._fonts or self._font_failed:
      return
    load = getattr(rl, "load_font_ex", None)
    cands = getattr(renderer, "_font_candidates", None)
    if load is None or cands is None or not hasattr(rl, "ffi"):
      self._font_failed = True
      return
    path = next((p for p in cands() if Path(p).exists()), None)
    if path is None:
      self._font_failed = True
      return
    cps = c1_codepoints()
    buf = rl.ffi.new("int[]", cps)
    quiet = getattr(rl, "set_trace_log_level", None)
    try:
      if quiet is not None:   # glyph-bounds FONT warnings, as in ClusterUiRenderer._load_font
        quiet(rl.TraceLogLevel.LOG_ERROR)
      for size in self.C1_FONT_SIZES:
        f = load(str(path), size, rl.ffi.cast("int *", buf), len(cps))
        if f.texture.id <= 0:
          raise RuntimeError("font atlas")
        rl.set_texture_filter(f.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
        self._fonts[size] = f
      self._font_path = str(path)
      if quiet is not None:
        quiet(rl.TraceLogLevel.LOG_WARNING)
    except Exception as exc:
      if quiet is not None:
        quiet(rl.TraceLogLevel.LOG_WARNING)
      print(f"C1 font load failed ({exc}); shared fonts used", flush=True)
      for f in self._fonts.values():
        try:
          rl.unload_font(f)
        except Exception:
          pass
      self._fonts.clear()
      self._font_failed = True

  def _font_for(self, size: float):
    if not self._fonts:
      return None
    s = int(round(size))
    if s in self._fonts and abs(size - s) < 1e-6:
      return self._fonts[s]
    # non-integer sizes (fare pop): nearest atlas at or above the size, scaled down
    bigger = [k for k in self._fonts if k >= size]
    return self._fonts[min(bigger)] if bigger else self._fonts[max(self._fonts)]

  def _measure(self, renderer, rl, text: str, size: float) -> tuple[float, float]:
    f = self._font_for(size)
    if f is None:
      return renderer._measure_text(text, size)
    return self._measure_ex(rl, f, text, size, max(1.0, size * 0.02))

  C1_MEASURE_CACHE_MAX = 512

  def _measure_ex(self, rl, f, text: str, size: float, sp: float) -> tuple[float, float]:
    """rl.measure_text_ex of a C1 atlas, cached (meter_improve CPU: ~28 measures per frame, mostly the same strings;
    the atlases never change while loaded, the cache is dropped with them)."""
    key = (int(f.texture.id), text, size, sp)
    v = self._measure_cache.get(key)
    if v is None:
      if len(self._measure_cache) >= self.C1_MEASURE_CACHE_MAX:
        self._measure_cache.clear()
      m = rl.measure_text_ex(f, text, size, sp)
      v = self._measure_cache[key] = (float(m.x), float(m.y))
    return v

  def _snap(self, v: float, axis: int) -> float:
    """Design coordinate -> the nearest one that lands on a whole device pixel (card drawn at k = 1)."""
    d = self._dev
    if d is None or d[2] != 1.0:
      return v
    o = d[3 + axis]
    return o + round(v - o)

  def _text(self, renderer, rl, text: str, x: float, y: float, size: float, color, shadow=None) -> None:
    """renderer._draw_text(anchor='left') look: y = vertical centre; own atlas at this pixel size, pixel-snapped."""
    if not text:
      return
    f = self._font_for(size)
    if f is None:
      if shadow is not None:
        renderer._draw_text(text, x, y + 1.0, size, shadow)
      renderer._draw_text(text, x, y, size, color)
      return
    sp = max(1.0, size * 0.02)
    _, vy = self._measure_ex(rl, f, text, size, sp)
    integral = abs(size - round(size)) < 1e-6
    dx, dy = x, y - vy * 0.5
    if integral:
      dx, dy = self._snap(dx, 0), self._snap(dy, 1)
    if shadow is not None:
      rl.draw_text_ex(f, text, rl.Vector2(dx, dy + 1.0), size, sp, rl.Color(*_c4(shadow)))
    rl.draw_text_ex(f, text, rl.Vector2(dx, dy), size, sp, rl.Color(*_c4(color)))

  def close(self, rl) -> None:
    self._measure_cache.clear()
    for f in self._fonts.values():
      try:
        rl.unload_font(f)
      except Exception:
        pass
    self._fonts.clear()
    for tex in self._textures.values():
      if tex is not None:
        try:
          rl.unload_texture(tex)
        except Exception:
          pass
    self._textures.clear()
    self._crop.clear()
    self._runs.clear()
    self._loaded_season = None

  # -- drawing helpers
  def _layer(self, rl, name: str, x: float, y: float, alpha: float) -> None:
    tex = self._textures.get(name)
    if tex is None or alpha <= 0.002:
      return
    color = rl.Color(255, 255, 255, int(round(255 * min(1.0, alpha))))
    runs = self._runs.get(name)
    if runs is not None:
      for rx, ry, rw, rh in runs:
        rl.draw_texture_pro(tex, rl.Rectangle(rx, ry, rw, rh), rl.Rectangle(x + rx, y + ry, rw, rh), rl.Vector2(0, 0),
                            0.0, color)
      return
    w, h = float(tex.width), float(tex.height)
    rl.draw_texture_pro(tex, rl.Rectangle(0, 0, w, h), rl.Rectangle(x, y, w, h), rl.Vector2(0, 0), 0.0, color)

  def _draw_anim(self, rl, tod: str, part: str, x: float, y: float, t: float, mul: float) -> None:
    """Animated scene elements of one time of day (r3), atlas sprites, alpha / x-offset per frame."""
    tex = self._textures.get(f"anim_{self.season}")
    if tex is None or mul <= 0.002:
      return
    key = PTCL_TOD.get(tod, tod)
    for e, dx, a in anim_elements(self.season, tod, t, part):
      a *= mul
      if a <= 0.004:
        continue
      col = rl.Color(255, 255, 255, int(round(255 * min(1.0, a))))
      sx, sy, ex, ey = e["sx"], e["sy"], x + e["x"] + dx, y + e["y"]
      for rx, ry, rw, rh in self._anim_runs.get((key, e["kind"], e["i"])) or ((0, 0, e["w"], e["h"]),):
        rl.draw_texture_pro(tex, rl.Rectangle(sx + rx, sy + ry, rw, rh), rl.Rectangle(ex + rx, ey + ry, rw, rh),
                            rl.Vector2(0, 0), 0.0, col)
      # a drifting sprite cut at the card edge: repeat its edge strip into the gap it leaves (HTML draws past the card)
      if dx < 0 and e["w"] >= 12 and e["x"] + e["w"] >= CARD_W - 0.5:
        g = min(-dx + 1.0, 12.0)
        rl.draw_texture_pro(tex, rl.Rectangle(sx + e["w"] - 12, sy, 12, e["h"]),
                            rl.Rectangle(ex + e["w"] - 1, ey, g, e["h"]), rl.Vector2(0, 0), 0.0, col)
      elif dx > 0 and e["w"] >= 12 and e["x"] <= 0.5:
        g = min(dx + 1.0, 12.0)
        rl.draw_texture_pro(tex, rl.Rectangle(sx, sy, 12, e["h"]), rl.Rectangle(ex - g + 1, ey, g, e["h"]),
                            rl.Vector2(0, 0), 0.0, col)

  def _scissor(self, rl, x: float, y: float, on: bool) -> None:
    f = getattr(rl, "begin_scissor_mode" if on else "end_scissor_mode", None)
    if f is None:
      return
    if on:
      d = self._dev
      if d is not None:   # scissor works in framebuffer pixels: the card's device rectangle
        f(int(d[0]), int(d[1]), int(round(CARD_W * d[2])), int(round(CARD_H * d[2])))
      else:
        f(int(round(x)), int(round(y)), int(CARD_W), int(CARD_H))
    else:
      f()

  def _back_and_moving(self, rl, x: float, y: float, t: float) -> None:
    """back layer(s), the animated back elements of each visible time of day, then the particles; the moving
    bits are clipped to the card."""
    bw = self.bg_weights
    cur, prev, w = bw["cur"], bw["prev"], bw["w"]
    has_anim = bool(ANIM.get(self.season)) and f"anim_{self.season}" in self._textures
    if w < 1.0:
      self._layer(rl, f"back_{self.season}_{prev}", x, y, 1.0)
      if has_anim:
        self._scissor(rl, x, y, True)
        self._draw_anim(rl, prev, "back", x, y, t, 1.0)
        self._scissor(rl, x, y, False)
      self._layer(rl, f"back_{self.season}_{cur}", x, y, w)
    else:
      self._layers(rl, "back", x, y)
    self._scissor(rl, x, y, True)
    if has_anim:
      self._draw_anim(rl, cur, "back", x, y, t, w)
    self._draw_particles(rl, x, y, t)
    self._scissor(rl, x, y, False)

  def _front_moving(self, rl, x: float, y: float, t: float) -> None:
    """animated front elements (summer waves: the first front elements, so under the rest of the over layer)."""
    lst = ANIM.get(self.season, {})
    if f"anim_{self.season}" not in self._textures or not any(e["part"] == "front" for v in lst.values() for e in v):
      return
    bw = self.bg_weights
    self._scissor(rl, x, y, True)
    if bw["w"] < 1.0:
      self._draw_anim(rl, bw["prev"], "front", x, y, t, 1.0 - bw["w"])
    self._draw_anim(rl, bw["cur"], "front", x, y, t, bw["w"])
    self._scissor(rl, x, y, False)

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
    cx0, cy0, w, h = self._crop.get(name) or (0, 0, tex.width, tex.height)
    ox = (SPRITE_ANCHOR + pivot_local[0] - cx0) * scale
    oy = (SPRITE_ANCHOR + pivot_local[1] - cy0) * scale
    rl.draw_texture_pro(tex, rl.Rectangle(cx0, cy0, w, h), rl.Rectangle(pos[0], pos[1], w * scale, h * scale),
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
    self._sprite(rl, "shell", (cx, cy), g["rotation"])   # own colours always (meter_improve 20:43, no LFA dimming)
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
    tw, _ = self._measure(renderer, rl, text, 17)
    bx, by, w, h, right = bubble_rect(tw, x_char)
    bx, by = self._snap(x + bx, 0) - x, self._snap(y + by, 1) - y
    # soft drop shadow (C1.html .bub box-shadow 0 2px 10px #0006), 2 widening layers
    for grow, sa in ((3.0, 0.16), (1.5, 0.22)):
      rl.draw_rectangle_rounded(rl.Rectangle(x + bx - grow, y + by + 2.0 - grow * 0.5, w + 2 * grow, h + 2 * grow), 0.9, 12,
                                rl.Color(0, 0, 0, int(a * sa)))
    rl.draw_rectangle_rounded(rl.Rectangle(x + bx, y + by, w, h), 0.9, 12, rl.Color(*BUBBLE_FILL, a))
    mid = y + by + h / 2
    if right:   # tail on the right edge, pointing at 포로리
      rl.draw_triangle(rl.Vector2(x + bx + w - 1, mid - 7), rl.Vector2(x + bx + w - 1, mid + 7), rl.Vector2(x + bx + w + 11, mid), rl.Color(*BUBBLE_FILL, a))
    else:
      rl.draw_triangle(rl.Vector2(x + bx - 11, mid), rl.Vector2(x + bx + 1, mid + 7), rl.Vector2(x + bx + 1, mid - 7), rl.Color(*BUBBLE_FILL, a))
    self._text(renderer, rl, text, x + bx + 14, mid, 17, (*BUBBLE_TEXT, a))
    self.last_bubble = text

  def _draw_think(self, renderer, rl, text: str, x: float, y: float, bob: float, alpha: float, slot: str = "up") -> None:
    """보노보노's thought bubble (C1.html .bub.th): the 포로리 bubble look, no tail, two dots to the head (slot 'low':
    the lower place, dots going up)."""
    if alpha <= 0.01 or not text:
      return
    a = int(255 * alpha)
    tw, _ = self._measure(renderer, rl, text, 17)
    low = slot == "low"
    bx, by, w, h = think_rect_low(tw, bob) if low else think_rect(tw, bob)
    bx, by = self._snap(x + bx, 0) - x, self._snap(y + by, 1) - y
    for grow, sa in ((3.0, 0.16), (1.5, 0.22)):
      rl.draw_rectangle_rounded(rl.Rectangle(x + bx - grow, y + by + 2.0 - grow * 0.5, w + 2 * grow, h + 2 * grow), 0.9, 12,
                                rl.Color(0, 0, 0, int(a * sa)))
    rl.draw_rectangle_rounded(rl.Rectangle(x + bx, y + by, w, h), 0.9, 12, rl.Color(*BUBBLE_FILL, a))
    for ddx, ddy, r in (THINK_LOW_DOTS if low else THINK_DOTS):
      c = rl.Vector2(x + bx + w + ddx, y + by + (0.0 if low else h) + ddy)
      rl.draw_circle_v(rl.Vector2(c.x, c.y + 1.0), r + 1.0, rl.Color(0, 0, 0, int(a * 0.18)))
      rl.draw_circle_v(c, r, rl.Color(*BUBBLE_FILL, a))
    self._text(renderer, rl, text, x + bx + 14, y + by + h / 2, 17, (*BUBBLE_TEXT, a))
    self.last_think = text
    self.last_think_slot = slot

  def _draw_particles(self, rl, x: float, y: float, t: float) -> None:
    bw = self.bg_weights
    cur = max(("day", "dusk", "night"), key=lambda k: bw.get(k, 0.0))   # c1bg.js: the strongest tod's particles
    tex = self._textures.get("ptcl_white")
    if tex is None:
      return
    h = PTCL_DRAW * 0.5
    for kind, px, py, rot, col, op in particle_positions(self.season, cur, t):
      op *= particle_alpha(px, py)          # r3: fade near the card edge / no-particle boxes (scissor clips the rest)
      if op <= 0.004:
        continue
      cell = PTCL_KINDS.index(kind) * PTCL_CELL
      rl.draw_texture_pro(tex, rl.Rectangle(cell, 0, PTCL_CELL, PTCL_CELL), rl.Rectangle(x + px, y + py, PTCL_DRAW, PTCL_DRAW),
                          rl.Vector2(h, h), rot, rl.Color(*col, int(round(255 * op))))

  def _draw_lfa_badge(self, renderer, rl, x: float, y: float, tw: dict) -> None:
    """LFA on/off + lane lines in use (the carrot wheel's lane icon rule; icon colour = LFA), small, in the sea under
    the clam. No 'LFA' text (보노보노 20:43)."""
    bx, by, bw, bh = LFA_BADGE
    box = rl.Rectangle(x + bx, y + by, bw, bh)
    rl.draw_rectangle_rounded(box, 1.0, 12, rl.Color(*theme_color("pillBg", tw)))
    rl.draw_rectangle_rounded_lines_ex(box, 1.0, 12, 1.0, rl.Color(*theme_color("pillB", tw)))
    col = LFA_ON_COLOR if self.lfa_active else LFA_OFF_COLOR
    la = 255 if self.lane_line else 70
    ix, iy, iw, ih = x + bx + 11, y + by + 5, 26.0, bh - 10.0
    for a, b in (((ix + 6, iy), (ix, iy + ih)), ((ix + iw - 6, iy), (ix + iw, iy + ih))):
      rl.draw_line_ex(rl.Vector2(*a), rl.Vector2(*b), 3.0, rl.Color(*col, la))

  def _draw_night_lamp(self, renderer, rl, lx: float, ly: float, t: float) -> None:
    """'심야 할증 20%' lamp, same lamp and place as the bd09c076 meter (top row, between brand and moon)."""
    box = rl.Rectangle(lx, ly, NIGHT_LAMP_W, NIGHT_LAMP_H)
    rl.draw_rectangle_rounded(box, 1.0, 16, rl.Color(58, 16, 12, 235))
    rl.draw_rectangle_rounded_lines_ex(box, 1.0, 16, 1.4, rl.Color(*NIGHT_LAMP_RED, 255))
    glow = int(70 + 40 * (0.5 + 0.5 * math.sin(t * 2.2)))
    cx, cy = lx + 20, ly + NIGHT_LAMP_H / 2
    rl.draw_circle_v(rl.Vector2(cx, cy), 9.0, rl.Color(*NIGHT_LAMP_RED, glow))
    rl.draw_circle_v(rl.Vector2(cx, cy), 4.8, rl.Color(255, 138, 96, 255))
    self._text(renderer, rl, f"심야 할증 {int(round(self.night_rate * 100))}%", lx + 34, cy, 15, (255, 196, 160))

  # -- frame
  def draw(self, renderer, rl, panel_x: float, panel_y: float) -> None:
    """res: the card is drawn in device pixels (see c1_card_device_rect): the HUD's rl_scalef(sx, sy) is undone
    and replaced by translate(dev) * scale(k) * translate(-card), so all card coordinates stay as they were."""
    x, y = panel_x + CARD_OFFSET_X, panel_y + CARD_OFFSET_Y
    W, H = getattr(renderer, "width", 1920), getattr(renderer, "height", 480)
    push, pop = getattr(rl, "rl_push_matrix", None), getattr(rl, "rl_pop_matrix", None)
    scl, trn = getattr(rl, "rl_scalef", None), getattr(rl, "rl_translatef", None)
    sx, sy = float(W) / 1920.0, float(H) / 480.0
    if None in (push, pop, scl, trn) or not (sx > 0 and sy > 0):
      self._dev = None
      self._draw_card(renderer, rl, panel_x, panel_y)
      return
    dx, dy, k = c1_card_device_rect(W, H, x, y)
    self._dev = (dx, dy, k, x, y)
    flush = getattr(rl, "rl_draw_render_batch_active", None)
    if flush is not None:
      flush()
    push()
    scl(1.0 / sx, 1.0 / sy, 1.0)
    trn(dx, dy, 0.0)
    scl(k, k, 1.0)
    trn(-x, -y, 0.0)
    try:
      self._draw_card(renderer, rl, panel_x, panel_y)
    finally:
      if flush is not None:
        flush()
      pop()

  def _draw_card(self, renderer, rl, panel_x: float, panel_y: float) -> None:
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
    text = lambda s_, x_, y_, z_, c_, sh_=None: self._text(renderer, rl, s_, x_, y_, z_, c_, sh_)   # noqa: E731
    measure = lambda s_, z_: self._measure(renderer, rl, s_, z_)   # noqa: E731
    shadow = theme_color("dsh", tw, 0.75)

    if st == SETTLE:
      self._draw_settle(renderer, rl, x, y, t, tw, since)
      return
    if st == "EMPTY" and not legacy:
      self._draw_empty(renderer, rl, x, y, t, tw, since)
      return
    self._back_and_moving(rl, x, y, t)
    # progress track
    p = meter.step_progress if st == "DRIVE" else 0.0
    rl.draw_line_ex(rl.Vector2(x + X0, y + TRACK_Y), rl.Vector2(x + X1, y + TRACK_Y), 3.0, rl.Color(*theme_color("trk0", tw)))
    px_fill = X0 + (X1 - X0) * p
    if px_fill > X0 + 1:
      rl.draw_rectangle_gradient_h(int(x + X0), int(y + TRACK_Y - 2), int(px_fill - X0), 4,
                                   rl.Color(*theme_color("tk0", tw)), rl.Color(*theme_color("tk1", tw)))
    # goal: 도토리 (res 17:47; acorn.png baked at scale 1), the old clam if the file is missing
    if self._textures.get(GOAL_SPRITE) is not None:
      self._sprite(rl, GOAL_SPRITE, (self._snap(x + GOAL[0], 0), self._snap(y + GOAL[1], 1)), 0.0)
    else:
      self._sprite(rl, "shell", (x + GOAL[0], y + GOAL[1]), 0.0, scale=1.0 / (SHELL_SC * BONO_S))
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
    self._front_moving(rl, x, y, t)
    self._layers(rl, "over", x, y)            # sea in front of the characters + frame/brand/stats/footer (one layer)
    self._draw_lfa_badge(renderer, rl, x, y, tw)

    # status pill (text drawn live; frame colours from the theme)
    label, dot = ("대기", (110, 98, 70)) if legacy else (PAUSE_PILL if (st == "DRIVE" and self.paused) else PILL[st])
    tw_px, _ = measure(label, 16)
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
    text(LABEL[st], x + 28, y + 83, 13, theme_color("lb", tw), shadow)
    fare_alpha = 0.62 if (st == "EMPTY" and not legacy) else 1.0
    fare_text = format_won(self.shown_fare())
    pop = 1.0 + 0.09 * math.sin(tk / 0.5 * math.pi) if tk < 0.5 else 1.0
    size = 98.0 * pop
    fw, _ = measure(fare_text, size)
    text(fare_text, x + 24, y + 143 + 1.5, size, theme_color("dsh", tw, fare_alpha))
    text(fare_text, x + 24, y + 143, size, theme_color("g2", tw, fare_alpha))
    text("원", x + 24 + fw + 10, y + 166, 30, theme_color("unit", tw, fare_alpha))
    if st == "DRIVE" and tk < 1.2:
      plus = f"+{format_won(self.step_won)}원"
      # meter_improve: a 6-digit fare (100,000원 and up) pushed '+100원' onto the stats glass: keep it left of the glass
      px_plus = min(max(372.0, 24 + fw + 60), PLUS_RIGHT - measure(plus, 30)[0])
      text(plus, x + px_plus, y + 118 - tk * 40, 30, theme_color("plus", tw, 1.0 - tk / 1.2))
    # stats
    km = f"{meter.distance_m / 1000.0:.2f}"
    kw, _ = measure(km, 27)
    text(km, x + 566, y + 127, 27, theme_color("tx", tw))
    text("km", x + 566 + kw + 5, y + 131, 15, theme_color("sub", tw))
    text(format_hms(meter.elapsed_s), x + 566, y + 199, 27, theme_color("tx", tw))
    # next line
    nx, ny = x + 28, y + 350
    if st == "DRIVE" and self.paused:
      parts = ((PAUSE_MSG, "acc"),)
    elif st == "DRIVE":
      time_mode = meter.mode == "time"
      if time_mode:
        remaining, unit = max(1, int(math.ceil(meter.next_step_remaining_s - 1e-6))), "초"
      else:
        remaining, unit = max(1, int(round(meter.next_step_remaining_m))), " m"
      parts = (("다음 ", "nx"), (f"+{format_won(self.step_won)}원", "acc"), ("까지 ", "nx"), (f"{remaining}", "acc"),
               (unit, "nx"), (f"  ·  현재 속도 ", "nx"), (f"{int(round(self.speed_kph))}", "acc"), (" km/h", "nx"))
    else:
      msg = "주행을 시작하면 요금이 올라갑니다" if legacy else NEXT_MSG[st].format(base=format_won(self.base_fare_won), km=f"{meter.fare.base_distance_m / 1000.0:g}")
      parts = ((msg, "nx"),)
    for part, key in parts:
      text(part, nx, ny, 14, theme_color(key, tw), shadow)
      nx += measure(part, 14)[0]
    # 딱! on a fare tick
    if st == "DRIVE" and tk < 0.7:
      text("딱!", x + 732, y + 312 - tk * 30, 22, theme_color("plus", tw, 1.0 - tk / 0.7))
    # speech bubble (one at a time)
    say, op = self.bubble_now(st, since, tk, t)
    self.last_bubble = ""
    self._draw_bubble(renderer, rl, say, pxp, x, y, op)
    # 보노보노's thought bubble (only while 포로리 says nothing)
    think, top = self.think_now(st, tk, t, op > 0.0)
    self.last_think = ""
    self.last_think_slot = ""
    slot = "up"
    if top > 0.0:
      tw17 = measure(think, 17)[0]
      slot = self.think_slot(tw17, pxp, nx - x, tk, t)
      if slot == "up":
        top *= think_poro_fade(tw17, pxp)
      if self._think_last is not None and self._think_last[2] == t:
        self._think_last = (think, top, t, self._think_key)   # the opacity as drawn (a cut thought fades from it)
    self._draw_think(renderer, rl, think, x, y, pose.bono_bob, top, slot)

  def _draw_settle(self, renderer, rl, x: float, y: float, t: float, tw: dict, since: float) -> None:
    """정산 screen (button 정산): the scene behind a panel, 총 요금 large, 기본/거리/시간/심야 할증 lines, 운행 거리·시간,
    포로리 bowing with '감사합니다!' and 보노보노 with the clam (C1 chibi look, same sprites, theme colours)."""
    text = lambda s_, x_, y_, z_, c_, sh_=None: self._text(renderer, rl, s_, x_, y_, z_, c_, sh_)   # noqa: E731
    measure = lambda s_, z_: self._measure(renderer, rl, s_, z_)[0]   # noqa: E731
    b = self.settle_breakdown or self.breakdown()
    self.last_settle = b
    self._back_and_moving(rl, x, y, t)
    panel = rl.Rectangle(x + SETTLE_PANEL[0], y + SETTLE_PANEL[1], SETTLE_PANEL[2], SETTLE_PANEL[3])
    rl.draw_rectangle_rounded(panel, 0.08, 16, rl.Color(*theme_color("stBg", tw)))
    rl.draw_rectangle_rounded_lines_ex(panel, 0.08, 16, 1.5, rl.Color(*theme_color("pillB", tw)))
    # title: 정 산 pill + brand line
    pill = rl.Rectangle(x + 36, y + 32, 76, 32)
    rl.draw_rectangle_rounded(pill, 1.0, 16, rl.Color(*theme_color("pillBg", tw)))
    rl.draw_rectangle_rounded_lines_ex(pill, 1.0, 16, 1.0, rl.Color(*theme_color("pillB", tw)))
    rl.draw_circle_v(rl.Vector2(pill.x + 17, pill.y + 16), 4.5, rl.Color(111, 216, 255, 255))
    text("정 산", pill.x + 28, pill.y + 16, 16, theme_color("pillT", tw))
    text("보노보노 택시 · 오늘도 고마워요", x + 126, y + 48, 16, theme_color("sub", tw))
    # total
    shadow = theme_color("dsh", tw, 0.75)
    text("총 요 금", x + 40, y + 88, 13, theme_color("lb", tw), shadow)
    total = format_won(b["total"])
    fw = measure(total, 98)
    text(total, x + 36, y + 148 + 1.5, 98, theme_color("dsh", tw))
    text(total, x + 36, y + 148, 98, theme_color("g2", tw))
    text("원", x + 36 + fw + 10, y + 171, 30, theme_color("unit", tw))
    # lines (right column)
    rows = [("기본요금", f"({b['base_km']:g}km)", f"{format_won(b['base'])}원"),
            ("거리요금", f"{b['dist_unit_m']:g}m × {b['dist_units']}회", f"{format_won(b['dist_won'])}원"),
            ("시간요금", f"{b['time_unit_s']:g}초 × {b['time_units']}회", f"{format_won(b['time_won'])}원"),
            ("심야 할증", f"{int(round(b['night_rate'] * 100))}%" if b["night_won"] > 0 else "",
             f"{format_won(b['night_won'])}원" if b["night_won"] > 0 else "없음")]
    rx0, rx1, ry = x + SETTLE_ROWS[0], x + SETTLE_ROWS[1], y + SETTLE_ROWS[2]
    for i, (name, detail, amount) in enumerate(rows):
      cy = ry + i * SETTLE_ROW_H
      if i % 2 == 0:
        rl.draw_rectangle_rounded(rl.Rectangle(rx0 - 10, cy - SETTLE_ROW_H / 2 + 2, rx1 - rx0 + 20, SETTLE_ROW_H - 4), 0.5, 8,
                                  rl.Color(*theme_color("stRow", tw)))
      text(name, rx0, cy, 16, theme_color("tx", tw))
      if detail:
        text(detail, rx0 + 82, cy, 14, theme_color("sub", tw))
      text(amount, rx1 - measure(amount, 17), cy, 17, theme_color("acc" if i == 3 and b["night_won"] > 0 else "tx", tw))
    # distance / time
    for i, (lab, val, unit) in enumerate((("운 행  거 리", f"{b['km']:.2f}", "km"), ("운 행  시 간", format_hms(b["elapsed_s"]), ""))):
      sx = x + 44 + i * 190
      text(lab, sx, y + 246, 13, theme_color("lb", tw))
      text(val, sx, y + 276, 27, theme_color("tx", tw))
      if unit:
        text(unit, sx + measure(val, 27) + 5, y + 280, 15, theme_color("sub", tw))
    text("시작을 누르면 새 운행을 시작합니다", x + SETTLE_HINT[0], y + SETTLE_HINT[1], 14, theme_color("nx", tw), shadow)
    # characters: 포로리 bows (thanks), 보노보노 holds the clam
    pose = taxi_meter_pose(self.anim, t, 9.0)
    bow = max(0.0, math.sin(min(since, 2.4) / 1.2 * math.pi)) if since < 2.4 else 0.0
    self._draw_porori(rl, (x + SETTLE_PORO[0], y + SETTLE_PORO[1] + bow * 6.0), pose, 0.0, bow)
    self._draw_bono(rl, x, y, pose, 0.0)
    op = _win(since, 0.1, 9e9)
    if op > 0.0:
      self._draw_bubble(renderer, rl, "감사합니다!", SETTLE_PORO[0], x, y + SETTLE_BUBBLE_DY, op)

  def _draw_empty(self, renderer, rl, x: float, y: float, t: float, tw: dict, since: float) -> None:
    """빈차 screen (meter_empty2, 보노보노 22:01 "빈차창도 따로"): the scene behind the 정산 screen's panel, a red '빈 차' sign
    (a taxi's roof lamp, breathing like the old 빈차 dot), the fare guide (기본요금 now incl. a night surcharge, 거리 /
    시간 / 심야 할증), 포로리 waving with '어디로 모실까요?' now and then, 보노보노 with the steering clam (still the only
    steering display) and the LFA badge. Same C1 sprites / theme colours as the 정산 screen."""
    text = lambda s_, x_, y_, z_, c_, sh_=None: self._text(renderer, rl, s_, x_, y_, z_, c_, sh_)   # noqa: E731
    measure = lambda s_, z_: self._measure(renderer, rl, s_, z_)[0]   # noqa: E731
    fare = self.meter.fare
    self._back_and_moving(rl, x, y, t)
    panel = rl.Rectangle(x + EMPTY_PANEL[0], y + EMPTY_PANEL[1], EMPTY_PANEL[2], EMPTY_PANEL[3])
    rl.draw_rectangle_rounded(panel, 0.08, 16, rl.Color(*theme_color("stBg", tw)))
    rl.draw_rectangle_rounded_lines_ex(panel, 0.08, 16, 1.5, rl.Color(*theme_color("pillB", tw)))
    breath = 0.5 - 0.5 * math.cos(t * math.pi)       # 0.5 Hz, as the 빈차 dot of the meter card
    red = tuple(int(round(a + (b - a) * breath)) for a, b in zip(EMPTY_RED, (122, 42, 36), strict=True))
    # title: 빈 차 pill + brand line
    pw = 17 + 9 + 9 + measure("빈 차", 16) + 17
    pill = rl.Rectangle(x + 36, y + 32, pw, 32)
    rl.draw_rectangle_rounded(pill, 1.0, 16, rl.Color(*theme_color("pillBg", tw)))
    rl.draw_rectangle_rounded_lines_ex(pill, 1.0, 16, 1.0, rl.Color(*theme_color("pillB", tw)))
    rl.draw_circle_v(rl.Vector2(pill.x + 17 + 4.5, pill.y + 16), 9.0, rl.Color(*red, 60))
    rl.draw_circle_v(rl.Vector2(pill.x + 17 + 4.5, pill.y + 16), 4.5, rl.Color(*red, 255))
    text("빈 차", pill.x + 17 + 9 + 9, pill.y + 16, 16, theme_color("pillT", tw))
    text(EMPTY_BRAND, pill.x + pw + 14, y + 48, 16, theme_color("sub", tw))
    # the sign: dark LED box, red outline + soft glow, '빈 차' 98 px, '손님을 기다리는 중' under it
    sx, sy, sw, sh = EMPTY_SIGN
    glow = 0.55 + 0.45 * (1.0 - breath)
    grow = EMPTY_GLOW                                 # one soft glow ring (fill kept small: one layer, no text shadow)
    rl.draw_rectangle_rounded(rl.Rectangle(x + sx - grow, y + sy - grow, sw + 2 * grow, sh + 2 * grow), 0.2, 16,
                              rl.Color(*EMPTY_RED, int(255 * 0.2 * glow)))
    sign = rl.Rectangle(x + sx, y + sy, sw, sh)
    rl.draw_rectangle_rounded(sign, 0.16, 16, rl.Color(*EMPTY_SIGN_BG))
    rl.draw_rectangle_rounded_lines_ex(sign, 0.16, 16, 2.0, rl.Color(*EMPTY_RED, int(150 + 105 * glow)))
    lit = tuple(int(round(a + (b - a) * (1.0 - breath) * 0.6)) for a, b in zip(EMPTY_RED, (255, 150, 120), strict=True))
    big = "빈 차"
    bw = measure(big, 98)
    text(big, x + sx + (sw - bw) / 2, y + sy + EMPTY_SIGN_TEXT_Y, 98, (*lit, 255))
    wait = EMPTY_WAIT
    text(wait, x + sx + (sw - measure(wait, 17)) / 2, y + sy + EMPTY_SIGN_SUB_Y, 17, (255, 176, 150, 230))
    # fare guide (right column, the 정산 rows' look)
    night_now = self.night_rate > 0.0
    n0, n1, nr = (fare.night_surcharges[0] if fare.night_surcharges else (0, 0, 0.0))
    rows = [("기본요금", f"({fare.base_distance_m / 1000.0:g}km" + (" · 심야)" if night_now else ")"),
             f"{format_won(self.base_fare_won)}원"),
            ("거리요금", f"{fare.distance_unit_m:g}m당", f"{format_won(fare.distance_unit_won)}원"),
            ("시간요금", f"{fare.time_unit_s:g}초당 · {fare.time_fare_below_kph:g}km/h 이하", f"{format_won(fare.time_unit_won)}원"),
            ("심야 할증", f"{n0:02d}~{n1:02d}시" + (" · 지금 적용" if night_now else ""),
             f"{int(round(nr * 100))}%" if nr > 0 else "없음")]
    rx0, rx1, ry = x + EMPTY_ROWS[0], x + EMPTY_ROWS[1], y + EMPTY_ROWS[2]
    for i, (name, detail, amount) in enumerate(rows):
      cy = ry + i * SETTLE_ROW_H
      if i % 2 == 0:
        rl.draw_rectangle_rounded(rl.Rectangle(rx0 - 10, cy - SETTLE_ROW_H / 2 + 2, rx1 - rx0 + 20, SETTLE_ROW_H - 4), 0.5, 8,
                                  rl.Color(*theme_color("stRow", tw)))
      hot = night_now and i in (0, 3)
      text(name, rx0, cy, 16, theme_color("tx", tw))
      text(detail, rx0 + 82, cy, 14, theme_color("acc" if hot else "sub", tw))
      text(amount, rx1 - measure(amount, 17), cy, 17, theme_color("acc" if hot else "tx", tw))
    shadow = theme_color("dsh", tw, 0.75)
    text(EMPTY_HINT, x + SETTLE_HINT[0], y + SETTLE_HINT[1], 14, theme_color("nx", tw), shadow)
    # characters: 포로리 waits at the start point (waves while it calls), 보노보노 holds the clam (steering)
    pose = taxi_meter_pose(self.anim, t, 9.0)
    say, op = self.empty_bubble(since)
    self._draw_porori(rl, (x + SETTLE_PORO[0], y + SETTLE_PORO[1]), pose, min(1.0, op * 1.4), 0.0)
    self._draw_bono(rl, x, y, pose, 0.0)
    self._draw_lfa_badge(renderer, rl, x, y, tw)
    self.last_bubble = ""
    self._draw_bubble(renderer, rl, say, SETTLE_PORO[0], x, y + SETTLE_BUBBLE_DY, op)

  def empty_bubble(self, since: float) -> tuple[str, float]:
    """포로리 on the 빈차 screen: '또 만나요!' right after the 빈차 button (0.1-2.6 s), then '어디로 모실까요?' every
    EMPTY_CALL[3] s (from EMPTY_CALL[1] s into 빈차, EMPTY_CALL[2] - EMPTY_CALL[1] s long). One bubble at a time."""
    if self.fsm.ev == "empty":
      op = _win(since, SAY["EMPTY"][1], SAY["EMPTY"][2])
      if op > 0.0:
        return SAY["EMPTY"][0], op
    text, a0, a1, period = EMPTY_CALL
    if since < a0:
      return "", 0.0
    op = _win(a0 + (since - a0) % period, a0, a1)
    return (text, op) if op > 0.0 else ("", 0.0)

  def bubble_now(self, st: str, since: float, tk: float, t: float) -> tuple[str, float]:
    """One bubble at a time: the state-change line (SAY) first, then the C1.html main-loop lines of 포로리:
    TICK_SAY after every fare step while 주행, STOP_SAY when the car comes to a stop while 주행."""
    out = self._bubble_now(st, since, tk, t)
    last = self._say_last
    if st != self._say_st:
      self._say_switch = (last[0], last[1], t) if (last is not None and last[1] > 0.0 and last[2] == self._say_t) else None
      self._say_st = st
    self._say_t = t
    sw = self._say_switch
    if out[1] <= 0.0 and sw is not None and 0.0 <= t - sw[2] < SWITCH_FADE_S and sw[0] != out[0]:
      out = (sw[0], sw[1] * (1.0 - (t - sw[2]) / SWITCH_FADE_S))   # the line of the state before, fading out
    self._say_last = (out[0], out[1], t) if out[1] > 0.0 else None
    return out

  def _bubble_now(self, st: str, since: float, tk: float, t: float) -> tuple[str, float]:
    fsm = self.fsm
    stopped = st == "DRIVE" and self.speed_kph <= C1_LOGIC["stop_kph"]
    if not stopped:
      self._stop_since = None
    elif self._stop_since is None:
      self._stop_since = t
    say, a0, a1 = SAY.get(st, ("", 0, 0))
    op = _win(since, a0, a1) if (st != "DRIVE" or since < 2.4) and (st != "EMPTY" or fsm.ev) else 0.0
    if op > 0.0:
      return say, op
    if st != "DRIVE":
      return "", 0.0
    op = _win(tk, TICK_SAY[1], TICK_SAY[2])
    if op > 0.0:
      return TICK_SAY[0], op
    if self._stop_since is not None:
      op = _win(t - self._stop_since, STOP_SAY[1], STOP_SAY[2])
      if op > 0.0:
        return STOP_SAY[0], op
    return "", 0.0

  def think_now(self, st: str, tk: float, t: float, poro_says: bool = False) -> tuple[str, float]:
    """보노보노's thought bubble (C1.html bth), after bubble_now() of the same frame: WAIT_THINK after STOP_SAY while
    stopped, TICK_THINK after TICK_SAY. Never while 포로리 says something; it fades out in the 0.15 s before a
    포로리 line starts (TICK_SAY / STOP_SAY begin 0.15 s after their event), so the two never show together."""
    self._think_prev_t, self._think_t = self._think_t, t
    out = self._think_now(st, tk, t, poro_says)
    if out[1] > 0.0:
      self._think_last = (out[0], out[1], t, self._think_key)
    return out

  def _next_step_in_s(self) -> float:
    """Seconds to the next fare step at the current speed (meter_improve item 2), inf when unknown."""
    m = self.meter
    rem = m.next_step_remaining_m
    v = max(0.0, self.speed_kph) / 3.6
    if self.speed_kph < m.fare.time_fare_below_kph:   # time fare (stopped / slow): the 30 s clock
      v = m.fare.distance_unit_m / m.fare.time_unit_s
    return rem / v if v > 1e-6 else float("inf")

  def _progress_rate(self) -> float:
    """step_progress per second at the current speed (0..1 per step)."""
    m = self.meter
    span = m.fare.distance_unit_m + (m.fare.base_distance_m if m.units == 0 else 0.0)
    v = max(0.0, self.speed_kph) / 3.6
    if self.speed_kph < m.fare.time_fare_below_kph:
      v = m.fare.distance_unit_m / m.fare.time_unit_s
    return v / span if span > 0.0 else 0.0

  def _tick_think_end(self, tk: float) -> float:
    """End (tk) of '또 올라가는 걸까…' for this step: TICK_THINK[2], or earlier when the next step comes first."""
    return min(TICK_THINK[2], tk + self._next_step_in_s() - THINK_STEP_MARGIN)

  def think_slot(self, text_w: float, x_char: float, next_line_right: float, tk: float, t: float) -> str:
    """'up' (C1.html place) or 'low' for the thought on screen, chosen once per thought: 'low' when 포로리 will be
    under the upper place by the end of the thought and the lower place is free."""
    key = self._think_key
    if key is not None and key == self._think_slot_key:
      return self._think_slot
    self._think_slot_key = key
    if key is not None and key[0] == "tick":
      left = max(0.0, self._tick_think_end(tk) - tk)
    elif key is not None and key[0] == "wait":
      left = max(0.0, key[1] + WAIT_THINK[2] - t)
    else:
      left = 0.0
    p_end = min(1.0, self.meter.step_progress + self._progress_rate() * left) if key is not None and key[0] in ("tick", "wait") else 0.0
    x_end = max(x_char, max(X0 + PORO_MIN, X0 + (X1 - X0) * p_end))
    up_ok = think_poro_fade(text_w, x_end) >= 1.0
    self._think_slot = "low" if (not up_ok and think_low_fits(text_w, next_line_right)) else "up"
    if key is not None and key[0] in ("carry", "leave"):
      self._think_slot = key[2]
    return self._think_slot

  def _think_now(self, st: str, tk: float, t: float, poro_says: bool) -> tuple[str, float]:
    if st != "DRIVE":
      self._think_stop = None
      last = self._think_last
      if self._think_key is not None and self._think_key[0] != "leave":
        ok = last is not None and last[1] > 0.0 and last[2] == self._think_prev_t
        self._think_switch = (last[0], last[1], t, self._think_slot) if ok else None
      self._think_key = None
      sw = self._think_switch
      if sw is not None and 0.0 <= t - sw[2] < SWITCH_FADE_S and not poro_says:
        self._think_key = ("leave", sw[2], sw[3])
        return sw[0], sw[1] * (1.0 - (t - sw[2]) / SWITCH_FADE_S)   # left 주행: fade out before the next line
      self._think_switch = None
      return "", 0.0
    if poro_says:
      self._think_key = None
      return "", 0.0
    stop = self._stop_since
    if stop is not None and self._think_stop != stop and t - stop >= WAIT_THINK[1]:
      self._think_stop = stop      # this stop lasted long enough: think once (also after the car pulls away)
    best, bop, key = "", 0.0, None
    if self._think_stop is not None:
      op = _win(t - self._think_stop, WAIT_THINK[1], WAIT_THINK[2])
      if op > bop:
        best, bop, key = WAIT_THINK[0], op, ("wait", self._think_stop)
    step_t = round(t - tk, 3)      # the fare step this tk counts from (= meter.last_tick_s on the live card)
    if tk >= TICK_THINK[1] - 1e-9 and tk < TICK_THINK[2]:
      end = self._tick_think_end(tk)
      if self._tick_decided != step_t:   # decided once, on the first frame of the window
        self._tick_decided = step_t
        if end - max(tk, TICK_THINK[1]) < THINK_MIN_S:
          self._tick_skip = step_t       # the next step comes too soon: no thought for this step
      op = 0.0 if self._tick_skip == step_t else _win(tk, TICK_THINK[1], end)
      if op > bop:
        best, bop, key = TICK_THINK[0], op, ("tick", step_t)
    lead = TICK_SAY[1]   # = STOP_SAY[1]
    if bop <= 0.0:
      # a thought that was on screen when this fare step came (the car sped up): fade it out before 포로리 speaks
      last = self._think_last
      if tk < lead:
        if self._think_carry is None or self._think_carry[2] != step_t:
          # only a thought still on in the frame right before the step (not one that had already faded out)
          ok = last is not None and last[3] is not None and last[3][0] in ("tick", "carry") and last[1] > 0.0 \
            and last[2] == self._think_prev_t and last[2] < t - tk + 1e-6
          self._think_carry = (last[0], last[1], step_t, self._think_slot) if ok else (None, 0.0, step_t, "up")
        text, op0 = self._think_carry[0], self._think_carry[1]
        if text:
          self._think_key = ("carry", step_t, self._think_carry[3])
          return text, op0 * max(0.0, (lead - tk) / lead)
      self._think_key = None
      return "", 0.0
    if tk < TICK_SAY[2]:                                   # a fare step just happened: 포로리 is about to speak
      bop *= max(0.0, (lead - tk) / lead)
    if stop is not None and t - stop < STOP_SAY[2]:        # the car just stopped: '또 멈췄네?' is about to come
      bop *= max(0.0, (lead - (t - stop)) / lead)
    self._think_key = key if bop > 0.0 else None
    return (best, bop) if bop > 0.0 else ("", 0.0)


class MeterButtonLink:
  """K9ExtHud=1 meter remote <-> HUD (meter_improve 20:46 / 21:20: the 시작·일시정지·정산 app, carrot k9_meter_link.py).
  poll(now) reads the memory param K9MeterCmd at most every POLL_S: a queue "token:cmd;token:cmd;..." of the last few
  presses (one "token:cmd" also works); it returns the commands of tokens not seen before, in order. What is already
  there when the HUD starts is not a press, so a restart never repeats an old one - except the presses after the last
  token the previous HUD took (K9MeterState "tok"): the HUD exits offroad, and a 시작 pressed meanwhile (e.g. on the
  정산 screen) is applied when it comes back instead of being lost (meter_reset 2026-10-31 23:04 "정산하고 시작 누르면
  초기화 안 된다"). publish(status) writes K9MeterState (JSON: state, fare, km, sec, night, tok, ts) when it changes,
  at most every PUBLISH_S, and at least every HEARTBEAT_S."""
  POLL_S = 0.2
  PUBLISH_S = 0.5
  HEARTBEAT_S = 2.0
  SEEN_MAX = 64

  def __init__(self, params, wall=time.time) -> None:
    self._params = params
    self._wall = wall
    self._seen: list[str] = []
    self._primed = False
    self._next = 0.0
    self._last: dict | None = None
    self._last_t = -1e9
    self._tok: str | None = None   # last token taken (published as "tok")

  def _read(self, key: str = METER_CMD_PARAM) -> str:
    try:
      v = self._params.get(key)
    except Exception:
      return ""
    if isinstance(v, bytes):
      v = v.decode("utf-8", "replace")
    return str(v or "")

  def poll(self, now: float) -> list[str]:
    if now < self._next:
      return []
    self._next = now + self.POLL_S
    entries = []
    for part in self._read().split(";"):
      tok, _, cmd = part.strip().rpartition(":")
      if tok:
        entries.append((tok, cmd.strip()))
    if not self._primed:
      self._primed = True
      toks = [t for t, _ in entries]
      try:
        prev = json.loads(self._read(METER_STATE_PARAM)).get("tok")
      except (ValueError, AttributeError):
        prev = None
      if prev not in toks:
        self._seen = toks[-self.SEEN_MAX:]
        return []
      self._seen = toks[:toks.index(prev) + 1][-self.SEEN_MAX:]
      self._tok = prev
    out = []
    for tok, cmd in entries:
      if tok in self._seen:
        continue
      self._seen = (self._seen + [tok])[-self.SEEN_MAX:]
      self._tok = tok
      if cmd in METER_COMMANDS:
        out.append(cmd)
    return out

  def publish(self, status, now: float | None = None) -> None:
    if not status:
      return
    if isinstance(status, str):
      status = {"state": status}
    if self._tok is not None:
      status = {**status, "tok": self._tok}
    now = time.monotonic() if now is None else now
    changed = status != self._last
    urgent = self._last is None or status.get("state") != self._last.get("state") or status.get("fare") != self._last.get("fare") \
      or status.get("tok") != self._last.get("tok")
    if not ((changed and (urgent or now - self._last_t >= self.PUBLISH_S)) or now - self._last_t >= self.HEARTBEAT_S):
      return
    try:
      put = getattr(self._params, "put_nonblocking", None) or self._params.put
      put(METER_STATE_PARAM, json.dumps({**status, "ts": round(self._wall(), 2)}, ensure_ascii=False))
      self._last, self._last_t = dict(status), now
    except Exception:
      pass
