"""K9ExtHud=1 C1 meter, bono round (2026-10-24 18:37 '보노보노 말풍선도 넣어라'): 보노보노's thought bubble (C1.html
core.js bubbles() bth: '언제 가는 걸까…' / '또 올라가는 걸까…') back on the live card, one bubble at a time with 포로리.
New tests only; no existing test input or threshold is changed here."""
import sys
from pathlib import Path

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

import cluster_taxi_c1 as c1  # noqa: E402

WAIT, TICK = c1.WAIT_THINK[0], c1.TICK_THINK[0]


def _panel():
  p = c1.C1MeterPanel(clock=lambda: 0.0, local_hour=lambda: 12)
  p.fsm_mode = True
  return p


def _frame(p, st, since, tk, t):
  say, op = p.bubble_now(st, since, tk, t)
  think, top = p.think_now(st, tk, t, op > 0.0)
  return say, op, think, top


def _w17(text):
  return sum(17.5 if ord(ch) > 0x7F else 10.0 for ch in text)   # >= KaiGen Bold 17 px advances


# ---- lines and conditions --------------------------------------------------------------------------------
def test_lines_are_the_c1_html_bth_lines():
  assert WAIT == "언제 가는 걸까…" and TICK == "또 올라가는 걸까…"
  assert {WAIT, TICK} <= set(c1.c1_text_strings())
  cps = set(c1.c1_codepoints())
  assert all(ord(ch) in cps for ch in WAIT + TICK)   # '…' (U+2026) is in the atlas


def test_wait_think_after_stop_say_while_stopped():
  p = _panel()
  p.speed_kph = 40.0
  _frame(p, "DRIVE", 30.0, 9.0, 100.0)
  p.speed_kph = 0.0
  _frame(p, "DRIVE", 30.0, 9.0, 101.0)                       # the stop starts
  say, _, think, _ = _frame(p, "DRIVE", 30.0, 9.0, 102.0)
  assert say == "또 멈췄네?" and think == ""                  # 포로리 first
  assert _frame(p, "DRIVE", 30.0, 9.0, 101.0 + 2.65)[2] == ""   # gap between the two
  say, _, think, top = _frame(p, "DRIVE", 30.0, 9.0, 101.0 + 4.0)
  assert say == "" and think == WAIT and top > 0.9
  assert _frame(p, "DRIVE", 30.0, 9.0, 101.0 + 5.7)[2] == ""
  assert _frame(p, "DRIVE", 30.0, 9.0, 101.0 + 20.0)[2] == ""   # once per stop


def test_wait_think_runs_on_when_the_car_pulls_away():
  p = _panel()
  p.speed_kph = 0.0
  _frame(p, "DRIVE", 30.0, 9.0, 200.0)
  assert _frame(p, "DRIVE", 30.0, 9.0, 203.0)[2] == WAIT     # 3.0 s stopped -> thinking
  p.speed_kph = 20.0
  assert _frame(p, "DRIVE", 30.0, 9.0, 204.5)[2] == WAIT     # still on after the start (HTML: 1 s into it)
  assert _frame(p, "DRIVE", 30.0, 9.0, 206.0)[2] == ""


def test_no_wait_think_after_a_short_stop():
  p = _panel()
  p.speed_kph = 0.0
  _frame(p, "DRIVE", 30.0, 9.0, 300.0)
  _frame(p, "DRIVE", 30.0, 9.0, 302.0)
  p.speed_kph = 20.0                                          # moved after 2 s
  for dt in (2.5, 3.0, 4.0, 5.0):
    assert _frame(p, "DRIVE", 30.0, 9.0, 300.0 + dt)[2] == "", dt


def test_tick_think_after_tick_say():
  p = _panel()
  p.speed_kph = 40.0
  for tk, shown in ((0.1, False), (1.0, False), (2.3, False), (2.6, True), (3.3, True), (4.1, True), (4.3, False), (8.0, False)):
    say, op, think, top = _frame(p, "DRIVE", 30.0, tk, 400.0 + tk)
    assert (think == TICK and top > 0) == shown, (tk, say, think, top)
    if 0.15 < tk < 2.4:
      assert say == "괴롭힐 거야?"


def test_no_thought_outside_drive():
  p = _panel()
  for st in ("EMPTY", "BOARD", "PAY"):
    for sp in (0.0, 30.0):
      p.speed_kph = sp
      for s in (0.5, 3.0, 9.0, 99.0):
        for tk in (3.0, 9.0):
          assert _frame(p, st, s, tk, 500.0 + s)[2] == "", (st, sp, s, tk)


def test_never_together_with_poro_over_a_drive():
  """Frame by frame (20 fps) over a stop-and-go drive with fare steps: 포로리 and 보노보노 never show at once."""
  p = _panel()
  dt, t, last_tick, seen = 0.05, 1000.0, 1000.0, set()
  prof = [(0.0, 6.0), (40.0, 9.0), (0.0, 1.5), (60.0, 7.0), (0.0, 12.0), (100.0, 9.0), (0.0, 3.0), (20.0, 4.0)]
  step_every = {0.0: 30.0, 20.0: 23.6, 40.0: 11.8, 60.0: 7.86, 100.0: 4.72}   # 131 m / 30 s
  for v, dur in prof:
    p.speed_kph = v
    for _ in range(int(dur / dt)):
      t += dt
      if t - last_tick >= step_every[v]:
        last_tick = t
      say, op, think, top = _frame(p, "DRIVE", 99.0, t - last_tick, t)
      assert not (op > 0.0 and top > 0.0), (t, say, think)
      if top > 0.0:
        seen.add(think)
  assert seen == {WAIT, TICK}


def test_poro_line_cuts_in_by_fading_the_thought_out_first():
  p = _panel()
  p.speed_kph = 0.0
  _frame(p, "DRIVE", 99.0, 9.0, 600.0)
  assert _frame(p, "DRIVE", 99.0, 9.0, 603.5)[3] > 0.9       # waiting thought on
  # a time-fare step at 603.6: the thought fades over 0.15 s, then '괴롭힐 거야?'
  tops = [_frame(p, "DRIVE", 99.0, tk, 603.6 + tk)[3] for tk in (0.0, 0.05, 0.1, 0.15)]
  assert tops[0] > tops[1] > tops[2] > tops[3] == 0.0


# ---- placement ----------------------------------------------------------------------------------------------
def _boxes():
  from cluster_taxi_meter import NIGHT_LAMP_H, NIGHT_LAMP_W, NIGHT_LAMP_X, NIGHT_LAMP_Y
  bx, by, bw, bh = c1.LFA_BADGE
  return {"fare": c1.FARE_BOX, "stats": c1.NO_PARTICLE_BOXES[2], "lfa": (bx, by, bx + bw, by + bh),
          "lamp": (NIGHT_LAMP_X, NIGHT_LAMP_Y, NIGHT_LAMP_X + NIGHT_LAMP_W, NIGHT_LAMP_Y + NIGHT_LAMP_H),
          "next": (28.0, 340.0, 760.0, 360.0), "track": (c1.X0, c1.TRACK_Y - 2, c1.X1, c1.TRACK_Y + 2),
          "goal": (c1.GOAL[0] - 20, c1.GOAL[1] - 24, c1.GOAL[0] + 20, c1.GOAL[1] + 24)}


def _hit(a, b):
  return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def test_thought_clear_of_fare_stats_badge_lamp_next_line_goal_and_card_edge():
  for text in (WAIT, TICK):
    for bob in (-4.5, -2.0, 0.0, 2.0, 4.5):
      for r in c1.think_parts(_w17(text), bob):
        for name, b in _boxes().items():
          assert not _hit(r, b), (text, bob, name, r, b)
      r = c1.think_extent(_w17(text), bob)
      assert 8.0 <= r[0] and r[2] <= c1.CARD_W - 8 and 8.0 <= r[1] and r[3] <= c1.CARD_H - 8


def test_thought_left_of_bono_head():
  # bono_body.png left edge = x -50 from the anchor (668); head rows (y <= 330) with the +-6 deg body rock stay >= 612
  for text in (WAIT, TICK):
    for bob in (-4.5, 4.5):
      assert c1.think_extent(_w17(text), bob)[2] <= 606.0


def test_thought_never_drawn_over_poro():
  for text in (WAIT, TICK):
    w = _w17(text)
    x0 = c1.think_extent(w)[0]
    for px in range(int(c1.X0 + c1.PORO_MIN), int(c1.X1) + 1):
      f = c1.think_poro_fade(w, float(px))
      if f > 0.0:
        assert px + c1.PORO_REACH[1] < x0, (text, px, f)
    assert c1.think_poro_fade(w, c1.X0 + c1.PORO_MIN) == 1.0   # 포로리 at the start: full
