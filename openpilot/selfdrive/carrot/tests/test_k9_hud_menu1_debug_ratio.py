"""K9HudNavdy 1 = debug 1 projection (대감님 11/19 06:05 "실험중1번을 ClusterHudScreenMode 디버그 1 주행화면처럼 차선이고 뭐고
비율 맞춰라, 예상경로 등등", 06:10 "디자인은 그대로"). New tests only; no existing test changed."""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "cluster"))

import cluster_renderer as cr
import cluster_scene as cs
from cluster_config import CLUSTER_CAMERA_VIEW_MODE_DEFAULT, CLUSTER_CAMERA_VIEW_MODE_EGO_BOTTOM
import render_gear_screens as rg
from dataclasses import replace


def _state(mode=CLUSTER_CAMERA_VIEW_MODE_DEFAULT):
  return replace(rg.synthetic_state("D"), camera_view_mode=mode, surround_view_active=False)


def _raylib_world_to_screen(camera, p, w, h):
  """raylib GetWorldToScreenEx (MatrixLookAt + MatrixPerspective, up = +z), independent of the renderer code"""
  eye, tgt = np.array([camera.position.x, camera.position.y, camera.position.z]), np.array([camera.target.x, camera.target.y, camera.target.z])
  z = eye - tgt
  z /= np.linalg.norm(z)
  x = np.cross(np.array([0.0, 0.0, 1.0]), z)
  x /= np.linalg.norm(x)
  y = np.cross(z, x)
  v = np.array(p) - eye
  cam = np.array([v @ x, v @ y, v @ z])
  f = 1.0 / math.tan(math.radians(camera.fovy_deg) * 0.5)
  ndc_x = f / (w / h) * cam[0] / -cam[2]
  ndc_y = f * cam[1] / -cam[2]
  return (ndc_x + 1.0) * 0.5 * w, (1.0 - ndc_y) * 0.5 * h


def _renderer(menu, view=CLUSTER_CAMERA_VIEW_MODE_DEFAULT):
  r = cr.ClusterUiRenderer()
  r.set_k9_hud_style(1)
  r.set_k9_hud_navdy(1)
  r.set_k9_hud_menu(menu)
  r._k9_geo, r._k9_cx = cr.K9L_ROAD_GEO, cr.K9L_CX
  r._k9_exp1_setup(_state(view))
  return r


@pytest.mark.parametrize("view", [CLUSTER_CAMERA_VIEW_MODE_DEFAULT, CLUSTER_CAMERA_VIEW_MODE_EGO_BOTTOM])
def test_menu1_points_land_where_debug1_draws_them(view):
  st = _state(view)
  camera = cs.scene_camera(st, 3.6)
  r = _renderer(1, view)
  ref0 = _raylib_world_to_screen(camera, (0.0, cs.render_scene_forward_m(0.0, st), 0.0), cr.DESIGN_WIDTH, cr.DESIGN_HEIGHT)
  worst = 0.0
  for f in (-8.0, 0.0, 3.0, 5.0, 10.0, 20.0, 40.0, 60.0, 90.0, 140.0):
    for lat in (-5.4, -1.8, 0.0, 1.8, 5.4):
      rx, ry = _raylib_world_to_screen(camera, (lat, cs.render_scene_forward_m(f, st), 0.0), cr.DESIGN_WIDTH, cr.DESIGN_HEIGHT)
      x, y, k = r.k9_turzx_road_project(f, lat)
      worst = max(worst, abs((x - cr.K9L_CX) - (rx - ref0[0])), abs(y - ry))
      if lat == 1.8:  # px per m = the debug 1 lane width at that row
        lx, _ = _raylib_world_to_screen(camera, (lat - 1.0, cs.render_scene_forward_m(f, st), 0.0), cr.DESIGN_WIDTH, cr.DESIGN_HEIGHT)
        assert k == pytest.approx(rx - lx, rel=1e-6)
  assert worst < 1e-6


def test_menu1_horizon_ego_and_back_from_debug1():
  r = _renderer(1)
  cam = r._k9_exp1_cam
  assert cr.k9_exp1_horizon_y(cam) == pytest.approx(r.k9_turzx_road_project(1e7, 0.0)[1], abs=0.05)
  assert cr.k9_exp1_horizon_y(cam) < 10.0                      # debug 1 horizon near the top (old 170)
  _, rear_y, _ = r.k9_turzx_road_project(cr.K9_EXP1_EGO_REAR_M, 0.0)
  assert 440.0 < rear_y < 470.0                                  # debug 1 ego rear bumper row
  assert cr._K9_BACK[0] == pytest.approx(10.0)                  # lines reach the debug 1 rear edge
  assert r.k9_road_clear_of_ego(4.0, 0.0) == 4.0                 # cars at their own distance, like debug 1


@pytest.mark.parametrize("menu", [None])   # 11/21: 0 / 2 removed
def test_other_menus_keep_the_old_projection(menu):
  r = _renderer(menu)
  assert r._k9_exp1_cam is None and cr._K9_BACK[0] == 0.0
  hy, ego_y, kx, _ = cr.K9L_ROAD_GEO
  for f, lat in ((0.0, 0.0), (20.0, 1.8), (80.0, -5.4)):
    z = f + cr.K9_TURZX_ROAD_Z0
    assert r.k9_turzx_road_project(f, lat) == pytest.approx((cr.K9L_CX + lat * kx / z, hy + (ego_y - hy) * cr.K9_TURZX_ROAD_Z0 / z, kx / z))
