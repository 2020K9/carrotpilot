"""K9ExtHud=1 C1 meter, res round render checks (GL display needed, e.g. xvfb-run): on the car's 1920x462 frame
the card is drawn 1:1 in device pixels (baked layers land texel-exact), C1 text uses its own exact-size atlases
on whole pixels, the goal is the 도토리 sprite. New tests only."""
import io
import os
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

pytestmark = pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")


class _Spy:
  def __init__(self, rl):
    self._rl = rl
    self.tex, self.txt = [], []

  def __getattr__(self, name):
    return getattr(self._rl, name)

  def draw_texture_pro(self, tex, src, dst, origin, rot, tint):
    self.tex.append((int(tex.id), float(dst.x), float(dst.y), float(dst.width), float(src.width)))
    return self._rl.draw_texture_pro(tex, src, dst, origin, rot, tint)

  def draw_text_ex(self, font, text, pos, size, spacing, tint):
    self.txt.append((int(font.baseSize), text, float(pos.x), float(pos.y), float(size)))
    return self._rl.draw_text_ex(font, text, pos, size, spacing, tint)


def _state():
  from cluster_models import SimulatorInput
  from cluster_simulator import ClusterSimulator
  base = ClusterSimulator().update(SimulatorInput(), 0.05)
  return replace(base, speed_kph=0.0, onroad=True, gear_text="D", steering_angle_deg=0.0, lfa_active=True,
                 active_lane_line=True)


def _open(w, h, season=3, hour=21):
  from cluster_renderer import ClusterUiRenderer
  r = ClusterUiRenderer(width=w, height=h, language="ko", theme_mode="dark")
  r.open(hidden=True)
  r.set_taxi_meter_enabled(True)
  r.taxi_meter_command("start")   # meter_empty2: 빈차 has its own screen now; these check the meter card (주행)
  r.set_taxi_meter_season(season)
  p = r._taxi_meter_panel
  p._local_hour = lambda: hour
  p.freeze_animation_t = 2.0
  return r, p


def _png(b):
  from PIL import Image
  return np.asarray(Image.open(io.BytesIO(b)).convert("RGB")).astype(int)


def test_card_layers_are_texel_exact_on_the_1920x462_frame():
  import cluster_taxi_c1 as c1
  from PIL import Image
  r, p = _open(1920, 462)
  try:
    img = _png(r.render_to_png_bytes(_state()))
    assert img.shape[:2] == (462, 1920)
    dx, dy, k = p._dev[:3]
    assert k == 1.0 and dx == int(dx) and dy == int(dy)
    over = np.asarray(Image.open(c1.C1_ASSET_DIR / "over_autumn_night.png").convert("RGBA")).astype(int)
    card = img[int(dy):int(dy) + 456, int(dx):int(dx) + 776]
    # the frame ring on the left / right edges (opaque in the over layer, nothing drawn live there)
    m = np.zeros((456, 776), bool)
    m[90:380, 1:5] = True
    m[250:380, 772:775] = True
    m &= over[:, :, 3] == 255
    assert m.sum() > 400
    diff = np.abs(card[m] - over[:, :, :3][m]).max()
    assert diff <= 1, diff
  finally:
    r.close()


def test_c1_text_uses_exact_size_atlases_on_whole_pixels(monkeypatch):
  import cluster_renderer
  r, p = _open(1920, 462)
  try:
    r.render_to_png_bytes(_state())
    spy = _Spy(cluster_renderer.rl)
    monkeypatch.setattr(cluster_renderer, "rl", spy)
    r.render_to_png_bytes(_state())
    cx, cy = p._dev[3], p._dev[4]
    mine = [t for t in spy.txt if t[0] in p.C1_FONT_SIZES and t[0] == round(t[4])]
    assert len(mine) >= 6                              # label, fare, 원, km, km unit, time, next line, pill, LFA
    for base, text, x, y, size in mine:
      assert abs(size - base) < 1e-6, text
      assert abs((x - cx) - round(x - cx)) < 1e-6 and abs((y - cy) - round(y - cy)) < 1e-6, (text, x, y)
  finally:
    r.close()


def test_goal_is_the_acorn(monkeypatch):
  import cluster_renderer
  import cluster_taxi_c1 as c1
  r, p = _open(1920, 462)
  try:
    r.render_to_png_bytes(_state())
    spy = _Spy(cluster_renderer.rl)
    monkeypatch.setattr(cluster_renderer, "rl", spy)
    r.render_to_png_bytes(_state())
    acorn = int(p._textures["acorn"].id)
    shell = int(p._textures["shell"].id)
    cx, cy = p._dev[3], p._dev[4]
    hits = [t for t in spy.tex if t[0] == acorn]
    assert len(hits) == 1 and abs(hits[0][1] - (cx + c1.GOAL[0])) < 1.0 and hits[0][3] == hits[0][4]   # scale 1
    assert len([t for t in spy.tex if t[0] == shell]) == 1   # only the steering clam is a clam now
  finally:
    r.close()


def test_design_canvas_frame_unchanged_in_place():
  r, p = _open(1920, 480)
  try:
    r.render_to_png_bytes(_state())
    dx, dy, k, cx, cy = p._dev
    assert (dx, dy, k) == (cx, cy, 1.0)
  finally:
    r.close()
