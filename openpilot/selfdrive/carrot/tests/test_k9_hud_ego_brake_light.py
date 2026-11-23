"""HUD ego car brake lamps (대감님 11/23 19:47 "내 차 브레이크등 쪽 이미지를 빨갛게 들어오게 해봐").

carState.brakeLights on -> the K9 ego photo's own tail lamps (and the high-mount stop lamp) glow red: overlays
assets/images/k9_ego_car_brake.png / k9_navdy_ego_brake.png (make_k9_ego_brake.py), drawn additively on the sprite's
own rectangle. Brake lights off -> nothing extra drawn, pixels as before. Render checks need a GL display (xvfb-run).
"""
import os
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cluster_renderer as cr

PAIRS = ((cr.EGO_CAR_ICON_PATH, cr.EGO_CAR_BRAKE_PATH), (cr.K9_NAVDY_EGO_PATH, cr.K9_NAVDY_EGO_BRAKE_PATH))


@pytest.mark.parametrize("sprite,overlay", PAIRS)
def test_overlay_sits_on_the_photo_tail_lamps(sprite, overlay):
  s = np.asarray(Image.open(sprite).convert("RGBA"), dtype=float)
  o = np.asarray(Image.open(overlay).convert("RGBA"), dtype=float)
  assert o.shape == s.shape  # same rectangle as the sprite
  core = o[..., 3] >= 200
  assert core.sum() > 300
  assert not (core & (s[..., 3] < 64)).any()  # the lit lamp is on the car, not in the air
  # red only
  lit = o[..., 3] > 0
  assert (o[..., 0][lit] >= 250).all() and (o[..., 1][lit] <= 60).all() and (o[..., 2][lit] <= 40).all()
  h, w = core.shape
  ys, xs = np.nonzero(core)
  left, right = xs < w * 0.3, xs > w * 0.7
  assert left.sum() > 100 and right.sum() > 100  # both tail lamps
  assert abs(left.sum() - right.sum()) < 0.35 * max(left.sum(), right.sum())
  for side in (left, right):  # in the lamp band of the rear, not on the roof / bumper bottom
    assert 0.35 * h < ys[side].mean() < 0.8 * h
  # the core is the photo's red lamp area: mostly red-dominant source pixels
  r, g, b = s[..., 0], s[..., 1], s[..., 2]
  red = (r > 1.3 * g) & (r > 1.2 * b) & (r >= 40)
  assert red[core].mean() > 0.6
  # high-mount stop lamp in the middle of the rear window
  mid = core & (np.arange(w)[None, :] > w * 0.35) & (np.arange(w)[None, :] < w * 0.65)
  assert 10 < mid.sum() < 400
  # nothing lit outside lamp areas (licence plate, badge, glass): only the lamps and their soft rim
  assert (o[..., 3] >= 16).mean() < 0.12


def _renderer(monkeypatch):
  from openpilot.common import indicator_display
  from openpilot.selfdrive.carrot import signal_mode
  monkeypatch.setattr(indicator_display, "current", lambda: None)
  monkeypatch.setattr(signal_mode, "signal_mode", lambda: 0)
  monkeypatch.setattr(cr.EV6_WEATHER, "snapshot", lambda: None)
  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  return r


def _png(r, st, nav):
  import io
  r.set_k9_hud_style(1)
  r.set_k9_hud_navdy(nav)
  r.set_k9_hud_res(True)
  r.set_k9_hud_menu(1)
  r.set_k9_hud_smooth(False)
  for _ in range(6):
    png = r.render_to_png_bytes(st)
  return np.asarray(Image.open(io.BytesIO(png)).convert("RGB"), dtype=int)


@pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")
@pytest.mark.parametrize("nav", (0, 1))
def test_brake_lights_light_the_ego_lamps_red(monkeypatch, nav):
  import render_gear_screens as rg
  from k9_hud_road_state import road_state
  r = _renderer(monkeypatch)
  try:
    st = road_state(replace(rg.synthetic_state("D"), car_fingerprint="KIA_K9", comma_resources=(23.0, 41.0, 57.0)))
    off = _png(r, replace(st, brake_lights=False), nav)
    on = _png(r, replace(st, brake_lights=True), nav)
    # brake off: same pixels as a renderer that has no brake-lamp drawing at all
    monkeypatch.setattr(r, "_draw_ego_brake_light", lambda *a, **k: None)
    plain = _png(r, replace(st, brake_lights=True), nav)
    assert np.array_equal(off, plain)
  finally:
    r.close()
  d = on - off
  changed = np.abs(d).sum(axis=2) > 0
  assert changed.sum() > 150, changed.sum()
  # what changed got redder: red up, green / blue far less
  assert (d[..., 0][changed] >= 0).all()
  assert d[..., 0][changed].mean() > 3 * max(1.0, d[..., 1][changed].mean())
  lit = (on[..., 0] >= 200) & (on[..., 0] > on[..., 1] + 110) & changed
  assert lit.sum() > 40, lit.sum()  # clearly lit red pixels
  ys, xs = np.nonzero(changed)
  h, w = changed.shape
  assert ys.min() > h * 0.3  # only around the ego car at the bottom, nothing on the rest of the HUD
