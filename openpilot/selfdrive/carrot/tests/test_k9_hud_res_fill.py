"""대감님 11/23 16:03 "가운데 화면 맨 위에 공간 있는데 왜 거기까지 안 채워?": the K9HudNavdy 1 res scene (scaled
K9R_SCENE_S about its left edge) zooms its camera so the road fills the screen top to bottom; K9HudNavdy 0 keeps its
old clip. Render checks need a GL display (xvfb-run)."""
import io
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


def _setup(res: bool, ego_bottom: bool = False):
  r = cr.ClusterUiRenderer.__new__(cr.ClusterUiRenderer)
  r.set_k9_hud_navdy(1)
  r.set_k9_hud_menu(1)
  r.set_k9_hud_res(res)
  mode = cr._k9_cs.CLUSTER_CAMERA_VIEW_MODE_EGO_BOTTOM if ego_bottom else None
  r._k9_exp1_setup(type("S", (), {"camera_view_mode": mode})())
  return r._k9_exp1_cam


def _screen_y(y: float) -> float:
  """k-frame (scene drawn K9R_SCENE_S about its left edge, moved down K9R_SCENE_DY) -> screen design y"""
  return cr.K9R_SCENE_DY + cr.K9R_SCENE_S * y


def test_res_off_camera_unchanged():
  assert _setup(False) == cr.k9_exp1_camera(False)
  assert _setup(False, True) == cr.k9_exp1_camera(True)


@pytest.mark.parametrize("ego_bottom", (False, True))
def test_res_zoom_keeps_camera_and_reaches_bottom(ego_bottom):
  base, cam = cr.k9_exp1_camera(ego_bottom), _setup(True, ego_bottom)
  assert cam[:4] == base[:4]  # same position / direction: only the focal changes
  assert cam[4] >= base[4] / cr.K9R_SCENE_S - 1e-9
  # zoom about the scene middle: what sat on the old scene bottom edge (y 480) now lands at / below the screen bottom
  half = cr.DESIGN_HEIGHT * 0.5
  assert _screen_y(half + half * cam[4] / base[4]) >= cr.DESIGN_HEIGHT - 0.5
  # my car keeps its place in the same projection (only farther from the middle by the zoom)
  y0 = cr.k9_exp1_project(base, cr.K9_EXP1_EGO_REAR_M, 0.0, cr.K9L_CX)[1]
  y1 = cr.k9_exp1_project(cam, cr.K9_EXP1_EGO_REAR_M, 0.0, cr.K9L_CX)[1]
  assert y1 - half == pytest.approx((y0 - half) * cam[4] / base[4])


def test_res_drive_horizon_at_screen_top():
  cam = _setup(True)
  assert _screen_y(cr.k9_exp1_horizon_y(cam)) <= 0.0
  # and not more zoom than needed: the plain 1 / K9R_SCENE_S zoom would leave the horizon a few px below the top
  assert cam[4] <= cr.k9_exp1_camera(False)[4] / cr.K9R_SCENE_S * 1.05


@pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")
def test_render_res_scene_fills_top_and_bottom():
  import render_gear_screens as rg
  from openpilot.common import indicator_display
  from openpilot.selfdrive.carrot import signal_mode
  from k9_hud_road_state import road_state
  indicator_display.current = lambda: None
  signal_mode.signal_mode = lambda: 0
  cr.EV6_WEATHER.snapshot = lambda: None
  st = road_state(replace(rg.synthetic_state("D"), car_fingerprint="KIA_K9", comma_resources=(23.0, 41.0, 57.0)))
  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  try:
    def dark_rows(navdy):
      r.set_k9_hud_style(1)
      r.set_k9_hud_navdy(navdy)
      r.set_k9_hud_res(True)
      r.set_k9_hud_menu(1)
      for _ in range(6):
        png = r.render_to_png_bytes(st)
      img = np.asarray(Image.open(io.BytesIO(png)).convert("RGB")).astype(float)
      sx = img.shape[1] / cr.DESIGN_WIDTH
      col = img[:, int(480 * sx):int(1280 * sx)].mean(axis=(1, 2))  # middle of the scene, off the side fades
      top = next(i for i, v in enumerate(col) if v >= 14.0)
      bot = next(i for i, v in enumerate(col[::-1]) if v >= 14.0)
      return top, bot, img.shape[0]
    top, bot, h = dark_rows(1)
    assert top <= 1 and bot <= 1, (top, bot, h)
  finally:
    r.close()
