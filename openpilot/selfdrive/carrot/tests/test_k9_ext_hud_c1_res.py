"""K9ExtHud=1 C1 meter, res round (2026-10-24 17:45-17:52): device-pixel card (1:1 on the 1920x462 TURZX 9.2),
own small font atlases, 포로리's main-loop bubbles back, 도토리 goal, 경기 일반(중형) fare (공고 제2023-5572호).
New tests only; no existing test input or threshold is changed here."""
import json
import sys
from pathlib import Path

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

import cluster_taxi_c1 as c1  # noqa: E402
from cluster_taxi_meter import TAXI_FARE_SIHEUNG, TAXI_METER_C1_FARE, TAXI_METER_C1_FOOTER_TEXT, TaxiMeter, night_surcharge_rate  # noqa: E402

TURZX_92 = (1920, 462)   # .vendor lcd_comm_turing_usb PRODUCT_ID 0x0092 = (462, 1920), landscape


# ---- resolution -------------------------------------------------------------------------------------------
def test_card_is_one_to_one_on_the_turzx_panel():
  x, y = 1916 - 784 + 8.0, 1 + 12.0
  dx, dy, k = c1.c1_card_device_rect(*TURZX_92, x, y)
  assert k == 1.0
  assert dx == int(dx) and dy == int(dy)
  assert 0 <= dy and dy + c1.CARD_H <= TURZX_92[1]
  assert 0 <= dx and dx + c1.CARD_W <= TURZX_92[0]


def test_card_on_the_design_canvas_is_where_it_was():
  for x in (1132.0, 1140.0):
    dx, dy, k = c1.c1_card_device_rect(1920, 480, x, 13.0)
    assert (dx, dy, k) == (x, 13.0, 1.0)


def test_card_scales_uniformly_when_the_panel_is_much_smaller():
  dx, dy, k = c1.c1_card_device_rect(960, 240, 1140.0, 13.0)
  assert abs(k - 0.5) < 1e-9


def test_font_atlas_holds_every_drawn_character():
  cps = set(c1.c1_codepoints())
  for s in c1.c1_text_strings():
    assert all(ord(ch) in cps for ch in s), s
  assert len(cps) < 400                              # a small atlas, not the 11k-glyph navi font
  assert set(c1.C1MeterPanel.C1_FONT_SIZES) >= {13, 14, 15, 16, 17, 22, 27, 30, 98}


# ---- bubbles -------------------------------------------------------------------------------------------
def _panel():
  p = c1.C1MeterPanel(clock=lambda: 0.0, local_hour=lambda: 12)
  p.fsm_mode = True
  return p


def test_state_change_bubbles_come_first():
  p = _panel()
  for st, txt in (("BOARD", "어서 오세요!"), ("PAY", "감사합니다!"), ("DRIVE", "출발~!")):
    p.speed_kph = 30.0
    say, op = p.bubble_now(st, 1.0, 1.0, 10.0)
    assert say == txt and op > 0.9


def test_tick_bubble_after_every_fare_step_while_driving():
  p = _panel()
  p.speed_kph = 40.0
  for tk, shown in ((0.05, False), (0.5, True), (2.0, True), (2.6, False), (8.0, False)):
    say, op = p.bubble_now("DRIVE", 30.0, tk, 100.0 + tk)
    assert (say == "괴롭힐 거야?" and op > 0) == shown, (tk, say, op)


def test_stop_bubble_when_the_car_stops_while_driving():
  p = _panel()
  p.speed_kph = 40.0
  assert p.bubble_now("DRIVE", 30.0, 9.0, 100.0)[0] == ""
  p.speed_kph = 0.0
  p.bubble_now("DRIVE", 31.0, 9.0, 101.0)          # stop starts
  say, op = p.bubble_now("DRIVE", 32.0, 9.0, 102.0)
  assert say == "또 멈췄네?" and op > 0.9
  assert p.bubble_now("DRIVE", 34.0, 9.0, 104.0)[0] == ""   # once per stop
  p.speed_kph = 20.0
  assert p.bubble_now("DRIVE", 35.0, 9.0, 105.0)[0] == ""


def test_no_handle_thought_bubble():
  # 보노보노 10:08: the 핸들 말풍선 (bth '언제 가는 걸까…') stays out
  p = _panel()
  texts = set()
  for st in c1.STATES:
    for sp in (0.0, 30.0):
      p.speed_kph = sp
      for s in (0.5, 1.5, 3.0, 9.0):
        for tk in (0.5, 1.5, 9.0):
          texts.add(p.bubble_now(st, s, tk, 50.0 + s)[0])
  assert not any("걸까" in t for t in texts)
  assert {"어서 오세요!", "출발~!", "감사합니다!", "괴롭힐 거야?", "또 멈췄네?"} <= texts


def _boxes():
  from cluster_taxi_meter import NIGHT_LAMP_H, NIGHT_LAMP_W, NIGHT_LAMP_X, NIGHT_LAMP_Y
  bx, by, bw, bh = c1.LFA_BADGE
  return {"fare": c1.FARE_BOX, "stats": c1.NO_PARTICLE_BOXES[2], "lfa": (bx, by, bx + bw, by + bh),
          "lamp": (NIGHT_LAMP_X, NIGHT_LAMP_Y, NIGHT_LAMP_X + NIGHT_LAMP_W, NIGHT_LAMP_Y + NIGHT_LAMP_H)}


def _w17(text):
  return sum(17.5 if ord(ch) > 0x7F else 10.0 for ch in text)   # >= KaiGen Bold 17 px advances


def test_bubbles_never_overlap_fare_stats_badge_or_lamp():
  texts = [v[0] for v in c1.SAY.values()] + [c1.TICK_SAY[0], c1.STOP_SAY[0]]
  boxes = _boxes()
  for text in texts:
    for px in range(int(c1.X0 + c1.PORO_MIN), int(c1.X1) + 1):
      bx, by, w, h, right = c1.bubble_rect(_w17(text), float(px))
      r = (bx - 12, by, bx + w + 12, by + h + 5)   # tail (12 px) + drop shadow (3 px out, 0.5 px below the top)
      for name, b in boxes.items():
        hit = r[0] < b[2] and b[0] < r[2] and r[1] < b[3] and b[1] < r[3]
        assert not hit, (text, px, name, r, b)


# ---- 도토리 goal ---------------------------------------------------------------------------------------------
def test_goal_is_an_acorn_sprite_of_clam_size():
  import numpy as np
  from PIL import Image
  a = np.asarray(Image.open(c1.C1_ASSET_DIR / "acorn.png").convert("RGBA"))
  s = np.asarray(Image.open(c1.C1_ASSET_DIR / "shell.png").convert("RGBA"))
  assert a.shape == s.shape == (240, 240, 4)
  ys, xs = np.nonzero(a[:, :, 3] > 8)
  w, h = xs.max() - xs.min() + 1, ys.max() - ys.min() + 1
  sc = 1.0 / (c1.SHELL_SC * c1.BONO_S)               # the old goal clam was drawn at this scale
  ys2, xs2 = np.nonzero(s[:, :, 3] > 8)
  sw, sh = (xs2.max() - xs2.min() + 1) * sc, (ys2.max() - ys2.min() + 1) * sc
  assert 0.7 * sw <= w <= 1.3 * sw and 0.8 * sh <= h <= 1.6 * sh
  rgb = a[:, :, :3][a[:, :, 3] > 200].astype(int)
  assert (rgb[:, 0] > rgb[:, 2] + 20).mean() > 0.6   # brown / tan, not the pink clam
  assert c1.GOAL_SPRITE == "acorn" and c1.GOAL == (516.0, 318.0)


# ---- 경기 일반(중형) fare ----------------------------------------------------------------------------------------
def test_c1_fare_is_gyeonggi_midsize_notice_2023_5572():
  f = TAXI_METER_C1_FARE
  assert f is TAXI_FARE_SIHEUNG["midsize"]
  assert (f.base_fare_won, f.base_distance_m) == (4800, 1600.0)
  assert (f.distance_unit_m, f.distance_unit_won, f.time_unit_s, f.time_unit_won) == (131.0, 100, 30.0, 100)
  assert f.time_fare_below_kph == 15.0
  assert [night_surcharge_rate(f, h) for h in range(24)] == [0.3] * 4 + [0.0] * 19 + [0.3]
  assert f.out_of_area_rate == 0.20
  assert c1.C1MeterPanel().meter.fare is f
  assert "4,800" in TAXI_METER_C1_FOOTER_TEXT and "131m" in TAXI_METER_C1_FOOTER_TEXT and "30%" in TAXI_METER_C1_FOOTER_TEXT


def _meter(hour=12):
  m = TaxiMeter(TAXI_METER_C1_FARE)
  m.reset(hour)
  return m


def test_midsize_base_then_131m_steps():
  m = _meter()
  assert m.fare_won == 4800
  for _ in range(int(1600 / (60 / 3.6) / 0.05)):
    m.step(0.05, 60.0, 12, 0.0)
  assert m.fare_won == 4800
  for _ in range(int(140 / (60 / 3.6) / 0.05) + 1):
    m.step(0.05, 60.0, 12, 0.0)
  assert m.fare_won == 4900


def test_midsize_time_fare_at_and_below_15kph():
  for v in (0.0, 15.0):
    m = _meter()
    m.metered_m = 1600.0
    for _ in range(int(30.0 / 0.05) + 2):
      m.step(0.05, v, 12, 0.0)
    assert m.mode == "time" and m.fare_won == 4900, v
  m = _meter()
  m.step(0.05, 15.5, 12, 0.0)
  assert m.mode == "distance"


def test_midsize_night_30_percent():
  m = _meter(23)
  assert m.fare_won == 6240
  m.metered_m = 1600.0
  for _ in range(int(30.0 / 0.05) + 2):
    m.step(0.05, 0.0, 23, 0.0)
  assert m.fare_won == 6370


def test_next_line_shows_the_new_base():
  p = c1.C1MeterPanel(clock=lambda: 0.0, local_hour=lambda: 12)
  msg = c1.NEXT_MSG["EMPTY"].format(base=c1.format_won(p.base_fare_won), km=f"{p.meter.fare.base_distance_m / 1000.0:g}")
  assert msg == "손님을 기다리는 중 · 기본 4,800원 (1.6km)"
  assert p.shown_fare() == 4800


def test_baked_footer_was_rebaked():
  # over_ layers carry the footer; the bake of this round writes res_bake.json next to them
  meta = json.loads((c1.C1_ASSET_DIR / "res_bake.json").read_text())
  assert meta["footer"] == TAXI_METER_C1_FOOTER_TEXT and meta["goal"] == "acorn"
