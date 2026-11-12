"""K9 D drawn lines smoothing (11/13 04:59 "D단 차선 흐물흐물") and steady 치우침 / 좌·우 text (11/13 05:12)."""
import random
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "cluster"))

import cluster_lat_values as lv
from cluster_k9_lanesm import K9LaneSmoother


def _line(lat, curv=0.0, noise=0.0, rng=None):
  rng = rng or random.Random(0)
  return [(f, lat + curv * f * f + (rng.gauss(0.0, noise * (1.0 + f / 30.0)) if noise else 0.0))
          for f in [0.0, 2.0, 5.0, 9.0, 14.0, 20.0, 27.0, 35.0, 45.0, 57.0, 70.0, 85.0, 100.0, 120.0, 140.0]]


def test_noise_is_reduced_and_clean_line_kept():
  rng = random.Random(1)
  sm = K9LaneSmoother()
  d_raw, d_sm, prev_r, prev_s = [], [], None, None
  for i in range(200):
    raw = _line(-1.8, 0.0002, 0.08, rng)
    out = sm.smooth("egoL", raw, i * 0.05)
    at40 = [lat for f, lat in out if f == 40.0][0]
    r40 = -1.8 + 0.0002 * 1600
    if prev_s is not None:
      d_sm.append(abs(at40 - prev_s))
      d_raw.append(abs(raw[8][1] - prev_r))
    prev_s, prev_r = at40, raw[8][1]
  assert sum(d_sm) < 0.5 * sum(d_raw)
  assert abs(at40 - r40) < 0.15


def test_lane_change_followed_without_lag():
  sm = K9LaneSmoother()
  t = 0.0
  for _ in range(120):  # 0.6 m/s sideways (lane change speed)
    lat = -1.8 + 0.6 * max(0.0, t - 1.0)
    out = sm.smooth("path", _line(lat), t)
    t += 0.05
  assert abs(out[0][1] - lat) < 0.1


def test_slot_swap_and_far_end():
  sm = K9LaneSmoother()
  sm.smooth("egoL", _line(-1.8), 0.0)
  out = sm.smooth("egoL", _line(1.8), 0.05)  # another line took the slot: no blend
  assert abs(out[0][1] - 1.8) < 1e-6
  assert out[-1][0] <= 140.0 + 1e-6


def _model(y1, y2, p=0.9, s=0.1):
  return SimpleNamespace(laneLines=[SimpleNamespace(y=[-5.4]), SimpleNamespace(y=[y1]), SimpleNamespace(y=[y2]), SimpleNamespace(y=[5.4])],
                         laneLineProbs=[0.5, p, p, 0.5], laneLineStds=[0.3, s, s, 0.3])


def test_text_held_through_short_drop_then_waits():
  h = lv.LaneTextHold()
  t = 0.0
  for _ in range(20):
    off, gl, gr = h.update(_model(-1.7, 1.9), t)
    t += 0.05
  assert off is not None and off < 0 and gl is not None and gr is not None
  for _ in range(10):  # 0.5 s of weak lines: still shown
    off, gl, gr = h.update(_model(-1.7, 1.9, p=0.3, s=0.4), t)
    t += 0.05
  assert off is not None and gl is not None
  for _ in range(40):  # 2 s: 대기
    off, gl, gr = h.update(None, t)
    t += 0.05
  assert off is None and gl is None and gr is None


def test_no_side_flicker_near_zero():
  h = lv.LaneTextHold()
  rng = random.Random(3)
  texts = []
  for i in range(400):
    c = rng.uniform(-0.03, 0.03)
    off, _, _ = h.update(_model(-1.8 - c, 1.8 - c), i * 0.05)
    texts.append(lv.lane_offset_text(off))
  flips = sum(1 for a, b in zip(texts, texts[1:], strict=False) if ("→" in a and "←" in b) or ("←" in a and "→" in b))
  assert flips == 0
  assert sum(1 for a, b in zip(texts, texts[1:], strict=False) if a != b) < 20
