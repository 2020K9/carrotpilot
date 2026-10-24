"""K9ExtHud=1 C1 meter, bono round render checks (GL display needed, e.g. xvfb-run): 보노보노's thought bubble is drawn
on the car's 1920x462 frame with the 17 px C1 atlas on whole pixels, only inside its own box (never on 보노보노),
and the live state machine puts it up after a fare step / while waiting stopped. New tests only."""
import io
import math
import os
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

pytestmark = pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")

T = 5000.0


class _Spy:
  def __init__(self, rl):
    self._rl = rl
    self.tex, self.txt = [], []

  def __getattr__(self, name):
    return getattr(self._rl, name)

  def draw_texture_pro(self, tex, src, dst, origin, rot, tint):
    self.tex.append((int(tex.id), (src.x, src.y, src.width, src.height), (dst.x, dst.y, dst.width, dst.height),
                     (origin.x, origin.y), float(rot)))
    return self._rl.draw_texture_pro(tex, src, dst, origin, rot, tint)

  def draw_text_ex(self, font, text, pos, size, spacing, tint):
    self.txt.append((int(font.baseSize), text, float(pos.x), float(pos.y), float(size)))
    return self._rl.draw_text_ex(font, text, pos, size, spacing, tint)


def _state(v=0.0):
  from cluster_models import SimulatorInput
  from cluster_simulator import ClusterSimulator
  base = ClusterSimulator().update(SimulatorInput(), 0.05)
  return replace(base, speed_kph=v, onroad=True, gear_text="D", steering_angle_deg=12.0, lfa_active=True,
                 active_lane_line=True)


def _open(season=3, hour=21):
  from cluster_renderer import ClusterUiRenderer
  r = ClusterUiRenderer(width=1920, height=462, language="ko", theme_mode="dark")
  r.open(hidden=True)
  r.set_taxi_meter_enabled(True)
  r.set_taxi_meter_season(season)
  p = r._taxi_meter_panel
  p._local_hour = lambda: hour
  return r, p


def _drive(p, tick_ago, stop_ago=None, progress=0.4):
  """주행, fare step `tick_ago` s before now, stopped since `stop_ago` s (None = moving)."""
  f, m = p.fsm, p.meter
  p.fsm_mode = True
  p._clock = lambda: T
  p.freeze_animation_t = None
  f.t = 100.0
  m.reset(12)
  m.metered_m = m.fare.base_distance_m + (5 + progress) * m.fare.distance_unit_m
  m.distance_m, m.elapsed_s, m.units = 2306.0, 412.0, 5
  m.fare_won_exact = m.fare.base_fare_won + 5 * m.fare.distance_unit_won
  m._was_onroad = True
  f.state, f.ev, f.since = "DRIVE", "move", 30.0
  m.last_tick_s = T - tick_ago
  p._think_stop = None
  if stop_ago is not None:
    p._stop_since = T - stop_ago
    return 0.0
  p._stop_since = None
  return 38.0


def _png(b):
  from PIL import Image
  return np.asarray(Image.open(io.BytesIO(b)).convert("RGB")).astype(int)


def test_live_state_puts_up_the_thoughts():
  import cluster_taxi_c1 as c1
  r, p = _open()
  try:
    for _ in range(2):
      v = _drive(p, 3.0)
      r.render_to_png_bytes(_state(v))
    assert p.last_think == c1.TICK_THINK[0] and p.last_bubble == ""
    for _ in range(2):
      v = _drive(p, 9.0, stop_ago=4.0)
      r.render_to_png_bytes(_state(v))
    assert p.last_think == c1.WAIT_THINK[0] and p.last_bubble == ""
    v = _drive(p, 1.0)                                    # '괴롭힐 거야?' -> no thought
    r.render_to_png_bytes(_state(v))
    assert p.last_bubble == c1.TICK_SAY[0] and p.last_think == ""
    v = _drive(p, 3.0, progress=0.95)                     # 포로리 under the thought's place -> not drawn there
    r.render_to_png_bytes(_state(v))
    assert p.last_think == "" or p.last_think_slot == "low"   # meter_improve: the lower place (item 3)
  finally:
    r.close()


def test_ellipsis_glyph_is_in_the_17px_atlas():
  import cluster_renderer
  r, p = _open()
  try:
    _drive(p, 3.0)
    r.render_to_png_bytes(_state(38.0))
    f = p._fonts[17]
    rl = cluster_renderer.rl
    for ch in "…걸까올":
      i = rl.get_glyph_index(f, ord(ch))
      assert f.glyphs[i].value == ord(ch), ch
  finally:
    r.close()


def test_thought_drawn_with_the_17px_atlas_on_whole_pixels(monkeypatch):
  import cluster_renderer
  import cluster_taxi_c1 as c1
  r, p = _open()
  try:
    v = _drive(p, 3.0)
    r.render_to_png_bytes(_state(v))
    spy = _Spy(cluster_renderer.rl)
    monkeypatch.setattr(cluster_renderer, "rl", spy)
    v = _drive(p, 3.0)
    r.render_to_png_bytes(_state(v))
    cx, cy = p._dev[3], p._dev[4]
    mine = [t for t in spy.txt if t[1] == c1.TICK_THINK[0]]
    assert len(mine) == 1
    base, _, x, y, size = mine[0]
    assert base == 17 and size == 17.0
    assert abs((x - cx) - round(x - cx)) < 1e-6 and abs((y - cy) - round(y - cy)) < 1e-6
  finally:
    r.close()


def _sprite_mask(alpha, call, shape, off):
  """device pixels covered (alpha > 8) by one draw_texture_pro(src, dst, origin, rot) call; off = card - device."""
  (sx, sy, sw, sh), (dx, dy, dw, dh), (ox, oy), rot = call[1], call[2], call[3], call[4]
  H, W = shape
  ys, xs = np.mgrid[0:H, 0:W]
  a = math.radians(rot)
  c, s = math.cos(a), math.sin(a)
  px, py = xs + 0.5 + off[0] - dx, ys + 0.5 + off[1] - dy
  lx, ly = px * c + py * s + ox, -px * s + py * c + oy   # inverse rotation about dst, then + origin
  u, v = sx + lx * sw / dw, sy + ly * sh / dh
  ok = (lx >= 0) & (ly >= 0) & (lx < dw) & (ly < dh)
  ui, vi = np.clip(u.astype(int), 0, alpha.shape[1] - 1), np.clip(v.astype(int), 0, alpha.shape[0] - 1)
  return ok & (alpha[vi, ui] > 8)


@pytest.mark.parametrize("season,hour", [(3, 21), (1, 12)])
def test_thought_pixels_stay_in_its_box_and_off_bono(monkeypatch, season, hour):
  import cluster_renderer
  import cluster_taxi_c1 as c1
  from PIL import Image
  r, p = _open(season, hour)
  try:
    v = _drive(p, 3.0)
    r.render_to_png_bytes(_state(v))
    p.freeze_animation_t = None
    spy = _Spy(cluster_renderer.rl)
    monkeypatch.setattr(cluster_renderer, "rl", spy)
    v = _drive(p, 3.0)
    on = _png(r.render_to_png_bytes(_state(v)))
    calls = list(spy.tex)
    assert p.last_think == c1.TICK_THINK[0]
    monkeypatch.setattr(p, "think_now", lambda *a, **k: ("", 0.0))
    v = _drive(p, 3.0)
    off = _png(r.render_to_png_bytes(_state(v)))
    dx, dy, k, cx, cy = p._dev
    assert k == 1.0
    diff = np.abs(on - off).max(axis=2) > 2
    assert diff.sum() > 2000
    tw, _ = p._measure(r, cluster_renderer.rl, c1.TICK_THINK[0], 17)
    bob = 0.0
    x0, y0, x1, y1 = c1.think_extent(tw, 4.5)
    x0b, y0b, x1b, y1b = c1.think_extent(tw, -4.5)
    box = (dx + min(x0, x0b) - 1.5, dy + min(y0, y0b) - 1.5, dx + max(x1, x1b) + 1.5, dy + max(y1, y1b) + 1.5)
    ys, xs = np.nonzero(diff)
    assert xs.min() >= box[0] and xs.max() <= box[2] and ys.min() >= box[1] and ys.max() <= box[3], (xs.min(), xs.max(), ys.min(), ys.max(), box)
    # the fill colour is there (the box is opaque at alpha 1)
    bx, by, w, h = c1.think_rect(tw, bob)
    mid = on[int(round(dy + by + 3)), int(round(dx + bx + w / 2))]
    assert np.abs(mid - np.array(c1.BUBBLE_FILL)).max() <= 3, mid
    # never on 보노보노's body
    bono_id = int(p._textures["bono_body"].id)
    bc = [c for c in calls if c[0] == bono_id]
    assert len(bc) == 1
    alpha = np.asarray(Image.open(c1.C1_ASSET_DIR / "bono_body.png").convert("RGBA"))[:, :, 3]
    bono = _sprite_mask(alpha, bc[0], diff.shape, (cx - dx, cy - dy))
    assert bono.sum() > 3000
    assert not (bono & diff).any()
    gap = np.nonzero(bono[int(box[1]):int(box[3])].any(axis=0))[0].min() - xs.max()
    assert gap >= 6, gap
  finally:
    r.close()
