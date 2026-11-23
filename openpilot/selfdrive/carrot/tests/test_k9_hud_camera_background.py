"""External HUD K9HudNavdy 1 middle road on the camera picture (대감님 11/23 18:15 "주행화면에 일반카메라·광각카메라도
선택할 수 있게"): ClusterHudCameraViewMode 2 road / 3 wide / 4 by speed draws the comma->Jetson preview frame under the
path band, lane lines and cars (same calibration / intrinsics as the comma camera overlay); 0 / 1 and a missing or
stale frame keep the 3D road. Render checks need a GL display (xvfb-run)."""
import base64
import io
import os
import sys
import time
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
JETLINK = Path(__file__).resolve().parents[4] / "tools" / "jetlink"
sys.path.insert(0, str(CLUSTER))
sys.path.insert(0, str(JETLINK))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cluster_renderer as cr  # noqa: E402
from cluster_scene import Vec3, EGO_FORWARD_M  # noqa: E402


def _state(mode):
  import render_gear_screens as rg
  from k9_hud_road_state import road_state
  st = road_state(replace(rg.synthetic_state("D"), car_fingerprint="KIA_K9"))
  return replace(st, camera_view_mode=mode, onroad=True, camera_calibration_euler=(0.0, 0.01, 0.003),
                 wide_camera_from_device_euler=(0.0, -0.003, -0.01), road_transform_trans=(0.0, 0.0, 1.25))


@pytest.mark.parametrize("wide", [False, True])
def test_projection_matches_comma_camera_overlay(wide):
  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r._camera_overlay_wide = wide
  st = _state(3 if wide else 2)
  proj = r._camera_overlay_projection(st, r.k9_cam_scene_rect())
  for f, lat in ((6.0, 0.0), (15.0, -1.8), (30.0, 1.7), (60.0, 3.5)):
    ref = r._camera_overlay_screen_xy(Vec3(lat, EGO_FORWARD_M + f, 0.0), proj)
    assert ref is not None
    x, y, k = r.k9_cam_project(proj, f, lat)
    assert abs(x - ref[0]) < 1e-6 and abs(y - ref[1]) < 1e-6 and k > 0.0
  # the camera fills the middle road area (scene coordinates of the res scene)
  d = proj.dest
  assert (d.x, d.width) == (cr.K9L_SCENE_X0, cr.K9L_SCENE_X1 - cr.K9L_SCENE_X0)
  v = proj.video_dest
  assert v.x <= d.x + 1e-6 and v.x + v.width >= d.x + d.width - 1e-6


class _Cam:
  is_wide = False
  display_frame = None

  def __init__(self, ok):
    self.ok, self.calls, self.closed = ok, [], 0

  def select_stream(self, prefer_wide):
    self.is_wide = prefer_wide
    return prefer_wide

  def draw(self, dest):
    self.calls.append((dest.x, dest.y, dest.width, dest.height))
    return self.ok

  def close(self):
    self.closed += 1


@pytest.mark.parametrize("mode", [0, 1])
def test_mode_0_1_never_touch_the_camera(mode):
  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.k9_hud_res = True
  r._k9_exp1_cam = cr.k9_exp1_camera(mode == 1)
  cam = r._live_road_camera = _Cam(True)
  assert r._k9_cam_background(_state(mode)) is None
  assert cam.calls == [] and cam.closed == 1 and r._live_road_camera is None


def test_no_frame_falls_back_to_3d_road():
  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.k9_hud_res = True
  r._k9_exp1_cam = cr.k9_exp1_camera(False)
  cam = r._live_road_camera = _Cam(False)
  for mode, wide in ((2, False), (3, True)):
    assert r._k9_cam_background(_state(mode)) is None
    assert cam.is_wide is wide
  assert len(cam.calls) == 2


def test_setting_text_names_k9_screen():
  import json
  root = Path(__file__).resolve().parents[2]
  for path in (root / "carrot_settings.json", root.parents[1] / "tools" / "carrot_assistant" / "catalog.json"):
    items = json.loads(path.read_text())
    stack, found = [items], None
    while stack:
      o = stack.pop()
      if isinstance(o, dict):
        if o.get("name") == "ClusterHudCameraViewMode":
          found = o
        stack.extend(o.values())
      elif isinstance(o, list):
        stack.extend(o)
    assert found is not None and found["max"] == 4  # no new menu numbers
    assert "K9 D 화면" in found["descr"] and "3D" in found["descr"]


def _jpeg(rgb):
  import hud_camera
  ycc = np.asarray(Image.fromarray(rgb).convert("YCbCr"), np.uint8)
  uv = np.empty((hud_camera.HEIGHT // 2, hud_camera.WIDTH), np.uint8)
  uv[:, 0::2] = ycc[0::2, 0::2, 1]
  uv[:, 1::2] = ycc[0::2, 0::2, 2]
  return hud_camera.jpeg_preview(ycc[:, :, 0].tobytes() + uv.tobytes())


@pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")
def test_render_camera_modes_and_fallback(monkeypatch):
  import hud_camera
  import hud_protocol
  import cluster_live_camera
  from openpilot.common import indicator_display
  from openpilot.selfdrive.carrot import signal_mode
  monkeypatch.setattr(indicator_display, "current", lambda: None)
  monkeypatch.setattr(signal_mode, "signal_mode", lambda: 0)
  monkeypatch.setattr(cr.EV6_WEATHER, "snapshot", lambda: None)
  monkeypatch.setattr(cluster_live_camera, "LiveRoadCamera", hud_camera.RemoteRoadCamera)
  packet = {}
  monkeypatch.setattr(hud_protocol, "read_snapshot", lambda: (time.monotonic(), packet) if packet else None)
  green = np.zeros((hud_camera.HEIGHT, hud_camera.WIDTH, 3), np.uint8)
  green[:, :, 1] = 200
  red = np.zeros_like(green)
  red[:, :, 0] = 200

  def feed(fid):
    now = time.monotonic()
    packet.clear()
    packet.update(sent=now, cameras={name: {"width": hud_camera.WIDTH, "height": hud_camera.HEIGHT, "frame": fid,
                                            "time": now, "jpeg": base64.b64encode(_jpeg(img)).decode()}
                                     for name, img in (("road", green), ("wide", red))})

  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  try:
    r.set_k9_hud_style(1)
    r.set_k9_hud_navdy(1)
    r.set_k9_hud_res(True)
    r.set_k9_hud_menu(1)
    r.set_k9_hud_smooth(False)

    def frame(mode, live):
      for k in range(4):
        if live:
          feed(k)
        else:
          packet.clear()
        png = r.render_to_png_bytes(_state(mode))
      return np.array(Image.open(io.BytesIO(png)).convert("RGB")).astype(int)
    base = frame(0, True)
    road, wide = frame(2, True), frame(3, True)
    gone = frame(2, False)
    sx = r.width / cr.DESIGN_WIDTH
    x0, x1 = int(cr.K9L_SCENE_X0 * sx) + 120, int((cr.K9L_SCENE_X0 + 300) * sx)
    patch = (slice(int(r.height * 0.05), int(r.height * 0.12)), slice(x0, x1))  # sky, beside the speed digits
    g, rr = road[patch].mean(axis=(0, 1)), wide[patch].mean(axis=(0, 1))
    assert g[1] > g[0] + 60 and rr[0] > rr[1] + 60
    assert np.array_equal(gone, frame(2, False))
    assert not np.array_equal(gone, road)
    # text panels outside the middle road are unchanged
    left = (slice(None), slice(0, int(cr.K9L_SCENE_X0 * sx) - 2))
    assert np.array_equal(base[left], road[left]) and np.array_equal(base[left], wide[left])
  finally:
    r.close()
