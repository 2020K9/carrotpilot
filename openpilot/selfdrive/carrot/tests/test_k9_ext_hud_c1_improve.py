"""K9ExtHud=1 C1 meter, meter_improve round (2026-10-24 19:04 '미터기 고칠거 더 고쳐 개선해'), no GL needed:
  * fare steps close together (~110 km/h and up): '또 올라가는 걸까…' ends before the next step (fade done), is left out
    when < 1 s would remain, and a thought cut by a step anyway fades out before 포로리 speaks;
  * 포로리 under the thought's place: the thought takes the lower place (below the track, left of 보노보노) instead of
    fading away; the place is chosen once per thought and is clear of everything else on the card;
  * a bubble cut by a state change fades out (0.1 s) instead of vanishing, before the next state's line;
  * the JPEG encoder reuses the previous JPEG for an unchanged frame (same bytes) and only then.
New tests only; no existing test input or threshold is changed here."""
import sys
from pathlib import Path

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

import cluster_taxi_c1 as c1  # noqa: E402

WAIT, TICK = c1.WAIT_THINK[0], c1.TICK_THINK[0]


def _w17(text):
  return sum(17.5 if ord(ch) > 0x7F else 10.0 for ch in text)   # >= KaiGen Bold 17 px advances


def _drive_sim(v_of_t, secs=60.0, dt=0.05):
  """The panel's own meter (C1 fare) driven at v_of_t(t) km/h, frame by frame like _draw_card: bubble_now, think_now,
  think_slot (+ the upper place's 포로리 fade). Returns per-frame records."""
  clock = [1000.0]
  p = c1.C1MeterPanel(clock=lambda: clock[0], local_hour=lambda: 12)
  p.fsm_mode = True
  p.fsm.state, p.fsm.ev = "DRIVE", "move"
  p.meter.reset(12)
  p.meter._was_onroad = True
  p.meter.units = 3
  p.meter.metered_m = p.meter.fare.base_distance_m + 3 * p.meter.fare.distance_unit_m
  st = type("S", (), {})()
  st.onroad, st.steering_angle_deg, st.lfa_active, st.active_lane_line = True, 0.0, True, True
  out = []
  for i in range(int(secs / dt)):
    clock[0] += dt
    t = clock[0]
    st.speed_kph = v_of_t(i * dt)
    p.update(st)
    tk = 9.0 if p.meter.last_tick_s is None else max(0.0, t - p.meter.last_tick_s)
    say, op = p.bubble_now("DRIVE", 99.0, tk, t)
    think, top = p.think_now("DRIVE", tk, t, op > 0.0)
    pxp = max(c1.X0 + c1.PORO_MIN, c1.X0 + (c1.X1 - c1.X0) * p.meter.step_progress)
    slot = ""
    if top > 0.0:
      slot = p.think_slot(_w17(think), pxp, 28.0 + 300.0, tk, t)
      if slot == "up":
        top *= c1.think_poro_fade(_w17(think), pxp)
      p._think_last = (think, top, t, p._think_key)
    out.append({"t": t, "tk": tk, "say": say, "op": op, "think": think if top > 0.0 else "", "top": top, "slot": slot,
                "pxp": pxp, "step": p.meter.last_tick_s, "v": st.speed_kph})
  return out


def _max_drop_ok(frames, dt):
  """No thought vanishes faster than its fades (0.25 s window fade, 0.15 s / 0.1 s cut fades) allow at this rate."""
  lim = dt / 0.15 + 0.02
  for a, b in zip(frames, frames[1:]):
    assert a["top"] - b["top"] <= lim, (a, b)


@pytest.mark.parametrize("dt", [0.05, 0.1])
@pytest.mark.parametrize("v", [90.0, 110.0, 115.0, 120.0, 125.0, 130.0])
def test_fast_steps_thought_ends_before_the_next_step(v, dt):
  fr = _drive_sim(lambda s: v, dt=dt)
  _max_drop_ok(fr, dt)
  assert not any(f["op"] > 0.0 and f["top"] > 0.0 for f in fr)
  steps = sorted({f["step"] for f in fr if f["step"] is not None})
  assert len(steps) >= 8
  shown = sum(dt for f in fr if f["top"] > 0.0)
  assert shown / (len(steps) - 1) >= 0.9, shown          # every step gets its thought (>= 1 s window, 10 fps edges)
  # at the frame of each new step the thought is already gone (it ended THINK_STEP_MARGIN s before, faded)
  for a, b in zip(fr, fr[1:]):
    if b["step"] != a["step"] and a["step"] is not None:
      assert b["top"] == 0.0 or b["think"] == a["think"], (a, b)
      assert a["top"] <= dt / 0.25 + 0.05 or a["tk"] < c1.TICK_THINK[1], (a, b)   # already in its fade-out


@pytest.mark.parametrize("v", [140.0, 160.0])
def test_very_fast_steps_no_thought(v):
  fr = _drive_sim(lambda s: v)
  assert sum(f["top"] > 0.0 for f in fr) == 0   # < 1 s would remain: left out, 포로리's line every step stays
  assert sum(f["say"] == c1.TICK_SAY[0] for f in fr) > 0


@pytest.mark.parametrize("dt", [0.05, 0.1])
@pytest.mark.parametrize("ramp", [(90.0, 150.0), (150.0, 90.0), (0.0, 140.0), (140.0, 0.0)])
def test_speed_ramps_never_cut_or_overlap(ramp, dt):
  fr = _drive_sim(lambda s: ramp[0] + (ramp[1] - ramp[0]) * s / 90.0, secs=90.0, dt=dt)
  _max_drop_ok(fr, dt)
  assert not any(f["op"] > 0.0 and f["top"] > 0.0 for f in fr)
  assert any(f["top"] > 0.0 for f in fr)


def test_thought_cut_by_an_early_step_fades_out_before_poro():
  p = c1.C1MeterPanel(clock=lambda: 0.0, local_hour=lambda: 12)
  p.fsm_mode = True
  p.speed_kph = 40.0
  ops = []
  for tk in (2.6, 3.0, 3.05):
    say, op = p.bubble_now("DRIVE", 99.0, tk, 100.0 + tk)
    ops.append(p.think_now("DRIVE", tk, 100.0 + tk, op > 0.0))
  assert ops[-1][0] == TICK and ops[-1][1] > 0.9
  # the next step comes at 103.1 (earlier than predicted): the thought fades over 0.15 s, 포로리 waits for it
  t0 = 103.1
  tops = []
  for tk in (0.0, 0.05, 0.1, 0.15, 0.2):
    say, op = p.bubble_now("DRIVE", 99.0, tk, t0 + tk)
    think, top = p.think_now("DRIVE", tk, t0 + tk, op > 0.0)
    assert not (op > 0.0 and top > 0.0)
    tops.append(top)
    if top > 0.0:
      assert think == TICK
  assert tops[0] > tops[1] > tops[2] > 0.0 and tops[3] == 0.0 and tops[4] == 0.0


def test_no_ghost_after_a_thought_that_already_faded():
  p = c1.C1MeterPanel(clock=lambda: 0.0, local_hour=lambda: 12)
  p.fsm_mode = True
  p.speed_kph = 40.0
  for tk in (2.6, 3.5, 4.1, 4.25, 4.3):
    say, op = p.bubble_now("DRIVE", 99.0, tk, 100.0 + tk)
    p.think_now("DRIVE", tk, 100.0 + tk, op > 0.0)
  for tk in (0.0, 0.05, 0.1):          # next step at 104.4: the thought was already gone -> nothing comes back
    say, op = p.bubble_now("DRIVE", 99.0, tk, 104.4 + tk)
    assert p.think_now("DRIVE", tk, 104.4 + tk, op > 0.0) == ("", 0.0)


# ---- lower place --------------------------------------------------------------------------------------------
def _boxes():
  from cluster_taxi_meter import NIGHT_LAMP_H, NIGHT_LAMP_W, NIGHT_LAMP_X, NIGHT_LAMP_Y
  bx, by, bw, bh = c1.LFA_BADGE
  return {"fare": c1.FARE_BOX, "stats": c1.NO_PARTICLE_BOXES[2], "lfa": (bx, by, bx + bw, by + bh),
          "lamp": (NIGHT_LAMP_X, NIGHT_LAMP_Y, NIGHT_LAMP_X + NIGHT_LAMP_W, NIGHT_LAMP_Y + NIGHT_LAMP_H),
          "next": (28.0, 340.0, 28.0 + 330.0, 360.0),           # longest 다음 line (worst case 300 px) + margin
          "track": (c1.X0, c1.TRACK_Y - 2, c1.X1, c1.TRACK_Y + 2),
          "goal": (c1.GOAL[0] - 20, c1.GOAL[1] - 24, c1.GOAL[0] + 20, c1.GOAL[1] + 24),
          "footer": (14.0, 400.0, 660.0, 456.0),
          "poro": (c1.X1 + c1.PORO_REACH[0], 200.0, c1.X1 + c1.PORO_REACH[1], 336.0)}   # 포로리 at the far right


def _hit(a, b):
  return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def test_lower_place_clear_of_everything():
  for text in (WAIT, TICK):
    for bob in (-4.5, -2.0, 0.0, 2.0, 4.5):
      parts = c1.think_parts_low(_w17(text), bob)
      for r in parts:
        for name, b in _boxes().items():
          assert not _hit(r, b), (text, bob, name, r, b)
        assert r[2] <= 606.0                                      # left of 보노보노 (head / arms from x ~612)
        assert 8.0 <= r[0] and r[2] <= c1.CARD_W - 8 and r[1] >= 8.0 and r[3] <= c1.CARD_H - 8
      assert c1.think_low_fits(_w17(text), 28.0 + 300.0)


def test_lower_place_when_poro_would_be_under_the_upper_one():
  p = c1.C1MeterPanel(clock=lambda: 0.0, local_hour=lambda: 12)
  p.fsm_mode = True
  p.speed_kph = 0.0
  w = _w17(TICK)
  for px, want in ((100.0, "up"), (300.0, "up"), (450.0, "low"), (500.0, "low")):
    p._think_key = ("wait", 100.0 - c1.WAIT_THINK[1] + px * 1e-6)   # a new thought each time, just starting
    p.meter.units, p.meter.metered_m = 3, p.meter.fare.base_distance_m + 3 * p.meter.fare.distance_unit_m
    assert p.think_slot(w, px, 28.0 + 300.0, 9.0, 100.0) == want, px
  p._think_key = ("wait", 100.0 - c1.WAIT_THINK[1])
  assert p.think_slot(w, 500.0, 600.0, 9.0, 100.0) == "up"   # lower place taken (a long 다음 line): fade as before


def test_place_kept_for_the_whole_thought():
  fr = _drive_sim(lambda s: 100.0)
  by_step = {}
  for f in fr:
    if f["top"] > 0.0:
      by_step.setdefault(f["step"], set()).add(f["slot"])
  assert by_step and all(len(s) == 1 for s in by_step.values()), by_step
  assert {"low"} <= set().union(*by_step.values())
  for f in fr:   # never in the upper place with 포로리 under it
    if f["top"] > 0.0 and f["slot"] == "up":
      assert c1.think_poro_fade(_w17(f["think"]), f["pxp"]) >= 1.0


def test_highway_thought_visible_instead_of_faded_away():
  fr = _drive_sim(lambda s: 100.0)
  steps = len({f["step"] for f in fr if f["step"] is not None})
  full = sum(0.05 for f in fr if f["top"] > 0.95)
  assert full / steps >= 1.2, full   # bono round: ~0.6 s per step left at 100 km/h (faded under 포로리)


# ---- state change ---------------------------------------------------------------------------------------------
def test_poro_line_cut_by_a_state_change_fades_before_the_next_line():
  p = c1.C1MeterPanel(clock=lambda: 0.0, local_hour=lambda: 12)
  p.fsm_mode = True
  p.fsm.ev = "occ"
  p.speed_kph = 30.0
  assert p.bubble_now("DRIVE", 99.0, 1.0, 200.0) == (c1.TICK_SAY[0], 1.0)
  ops = []
  for i, s in enumerate((0.0, 0.05, 0.1, 0.15, 0.3)):   # 주행 -> 지불 at 200.05
    say, op = p.bubble_now("PAY", s, 9.0, 200.05 + s)
    ops.append((say, round(op, 3)))
  assert ops[0][0] == c1.TICK_SAY[0] and 0.9 < ops[0][1] <= 1.0
  assert ops[1][0] == c1.TICK_SAY[0] and 0.4 < ops[1][1] < 0.6
  assert ops[2] == ("감사합니다!", 0.0) or ops[2][1] == 0.0
  assert ops[3][0] == "감사합니다!" and ops[4][0] == "감사합니다!"


def test_thought_cut_by_leaving_drive_fades_and_never_meets_the_next_line():
  p = c1.C1MeterPanel(clock=lambda: 0.0, local_hour=lambda: 12)
  p.fsm_mode = True
  p.fsm.ev = "occ"
  p.speed_kph = 40.0
  for tk in (2.6, 3.0):
    say, op = p.bubble_now("DRIVE", 99.0, tk, 300.0 + tk)
    think, top = p.think_now("DRIVE", tk, 300.0 + tk, op > 0.0)
  assert think == TICK and top > 0.9
  seen = []
  for s in (0.0, 0.03, 0.06, 0.09, 0.1, 0.12, 0.5):
    say, op = p.bubble_now("PAY", s, 9.0, 303.05 + s)
    think, top = p.think_now("PAY", 9.0, 303.05 + s, op > 0.0)
    assert not (op > 0.0 and top > 0.0), (s, say, think)
    seen.append(top)
  assert seen[0] > seen[1] > seen[2] > seen[3] > 0.0 and seen[4:] == [0.0, 0.0, 0.0]


# ---- JPEG reuse (cluster_usb_display) ----------------------------------------------------------------------
def test_jpeg_reused_only_for_an_unchanged_frame(monkeypatch):
  from cluster_usb_display import TuringUsbDisplay
  d = TuringUsbDisplay(jpeg_quality=68, jpeg_encoder="pillow")
  calls = []
  real = d._encode_jpeg_pillow

  def spy(rgba, w, h):
    calls.append((w, h))
    return real(rgba, w, h)
  monkeypatch.setattr(d, "_encode_jpeg_pillow", spy)
  w, h = 64, 32
  a = bytes(range(256)) * (w * h * 4 // 256)
  b = bytes(reversed(a))
  j1 = d.encode_jpeg(a, w, h, unchanged=True)       # nothing cached yet: encodes
  j2 = d.encode_jpeg(a, w, h, unchanged=True)
  assert j2 is j1 and len(calls) == 1 and d.jpeg_reused_frames == 1
  j3 = d.encode_jpeg(b, w, h)                       # changed: encodes
  assert len(calls) == 2 and j3 != j1
  assert d.encode_jpeg(b, w, h, unchanged=True) is j3
  d.jpeg_quality = 70                               # same pixels, other quality: encodes
  d.encode_jpeg(b, w, h, unchanged=True)
  assert len(calls) == 3
  assert d.encode_jpeg(a[: 32 * 32 * 4], 32, 32, unchanged=True) is not None and len(calls) == 4   # other size
  d2 = TuringUsbDisplay(jpeg_quality=70, jpeg_encoder="pillow")
  assert d2.encode_jpeg(b, w, h) == d.encode_jpeg(b, w, h, unchanged=True)   # reused bytes = a fresh encode


def test_frame_compare_is_exactly_byte_equality():
  """renderer._rgba_frames_equal (memcmp with a moving 64 KB first-look window) == (a == b), over a sequence of frames
  that change in different places (and sometimes not at all)."""
  import random
  from cluster_renderer import ClusterUiRenderer, _buffer_address
  r = ClusterUiRenderer(width=1920, height=462, language="ko", theme_mode="dark")
  n = 462 * 1920 * 4
  rng = random.Random(7)
  prev = bytearray(rng.randbytes(n))
  for i in range(60):
    cur = bytearray(prev)
    kind = i % 4
    if kind == 1:
      cur[rng.randrange(n)] ^= 0x01                  # one byte anywhere
    elif kind == 2:
      off = rng.randrange(n - 5000)
      cur[off:off + 5000] = rng.randbytes(5000)
    elif kind == 3:
      cur[-1] ^= 0x80                                # the last byte
    want = bytes(cur) == bytes(prev)
    assert r._rgba_frames_equal(_buffer_address(cur), _buffer_address(prev), n) == want, (i, kind)
    prev = cur
