"""K9HudNavdy 3 (대감님 11/18 11:49~11:56): smooth 3D board. Display only.
Render checks need a GL display (xvfb-run like the other cluster render tests)."""
import io
import json
import os
import sys
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image

ROOT = Path(__file__).resolve().parents[4]
CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
JETLINK = ROOT / "tools" / "jetlink"
sys.path.insert(0, str(CLUSTER))

import cluster_k9_3d as k3
import cluster_renderer as cr
from cluster_models import DetectedVehicle, LaneMarking, ModelPathPoint


def line(off, curv=0.0015):
  return tuple(ModelPathPoint(float(f), off + curv * f * f) for f in range(0, 120, 3))


def road_state(base, lead_accel=-0.2, lead_d=22.0, left=False):
  lanes = tuple(LaneMarking(o, model_points=line(o)) for o in (-5.4, -1.8, 1.8, 5.4))
  vs = (DetectedVehicle("L1", lead_d, 0.2, "radar", relative_speed_mps=-0.6, acceleration_mps2=lead_accel, primary=True),
        DetectedVehicle("R2", 14.0, 3.9, "radar", radar_track_id=7))
  return replace(base, car_fingerprint="KIA_K9", lanes=lanes, left_road_edge_points=line(-6.6),
                 right_road_edge_points=line(6.6), detected_vehicles=vs, left_signal=left, speed_kph=56.0,
                 display_speed_kph=56.0)


# ---- pure ---------------------------------------------------------------------------------------------------------

def test_projection_ego_low_horizon_high_and_behind_camera_clipped():
  ego = k3.project(-2.45, 0.0)
  far = k3.project(110.0, 0.0)
  assert 400 < ego[1] < 480 and 0 < far[1] < 120 and abs(ego[0] - k3.CX) < 1e-6
  assert k3.project(-k3.CAM_BACK_M - 30.0, 0.0) is None and k3.project(-k3.CAM_BACK_M, 0.0, k3.CAM_H_M) is None
  left, right = k3.project(10.0, -1.8), k3.project(10.0, 1.8)
  assert left[0] < k3.CX < right[0]


def _sample(lat, f, speed):
  return k3.Sample([(lat, [lat] * len(k3.STATIONS))], (None, None), {"lead": (f, 0.0, True, False)}, speed)


def test_interpolation_one_interval_behind_never_ahead():
  it = k3.Interpolator()
  it.push(_sample(-1.8, 20.0, 50.0), 0.0)
  assert it.at(0.0).cars["lead"][0] == 20.0
  it.push(_sample(-1.6, 22.0, 60.0), 0.1)
  mid = it.at(0.15)
  assert abs(mid.cars["lead"][0] - 21.0) < 1e-6 and abs(mid.lines[0][0] + 1.7) < 1e-6 and abs(mid.speed - 55.0) < 1e-6
  assert it.at(0.1).cars["lead"][0] == 20.0          # new sample starts from the old one
  assert it.at(0.5).cars["lead"][0] == 22.0          # never past the newest sample (no extrapolation)


def test_same_data_is_not_a_new_sample_and_a_stall_snaps():
  it = k3.Interpolator()
  it.push(_sample(-1.8, 20.0, 50.0), 0.0)
  it.push(_sample(-1.6, 22.0, 60.0), 0.1)
  it.push(_sample(-1.6, 22.0, 60.0), 0.15)           # same content again: still gliding from 0.1
  assert abs(it.at(0.15).cars["lead"][0] - 21.0) < 1e-6
  it.push(_sample(-1.0, 40.0, 60.0), 1.0)            # after a stall: drawn as is
  assert it.at(1.0).cars["lead"][0] == 40.0


def test_new_or_jumping_car_and_changed_line_count_are_not_blended():
  a = k3.Sample([(-1.8, [-1.8] * len(k3.STATIONS))], (None, None), {"lead": (20.0, 0.0, True, False)}, 50.0)
  b = k3.Sample([(-1.8, [-1.8] * len(k3.STATIONS)), (1.8, [1.8] * len(k3.STATIONS))], (None, None),
                {"lead": (60.0, 0.0, True, True), "r7": (10.0, 3.6, False, False)}, 50.0)
  m = k3.mix(a, b, 0.5)
  assert m.lines == b.lines and m.cars["lead"] == (60.0, 0.0, True, True) and m.cars["r7"] == (10.0, 3.6, False, False)


def test_lead_brake_flag_from_lead_acceleration():
  r = cr.ClusterUiRenderer.__new__(cr.ClusterUiRenderer)
  base = SimpleNamespace(lanes=(), left_road_edge_points=(), right_road_edge_points=(), speed_kph=50.0)
  for accel, want in ((-0.2, False), (-1.0, True), (-2.5, True), (None, False)):
    st = SimpleNamespace(**vars(base), detected_vehicles=(DetectedVehicle("L1", 20.0, 0.0, "radar", acceleration_mps2=accel,
                                                                           primary=True),))
    s = k3.sample_from_state(st, r._k9_is_lead, lambda _s: [])
    assert s.cars["lead"][3] is want


def test_dash_phase_moves_with_speed():
  scene = k3.K9Scene3D(clock=iter([0.0, 0.1, 0.2]).__next__)
  r = cr.ClusterUiRenderer.__new__(cr.ClusterUiRenderer)
  st = SimpleNamespace(lanes=(), left_road_edge_points=(), right_road_edge_points=(), speed_kph=72.0, detected_vehicles=())
  for _ in range(3):
    scene.update(st, r._k9_is_lead, lambda _s: [])
  assert abs(scene.odo_m - 4.0) < 1e-6   # 20 m/s x 0.2 s


def test_settings_and_main_wiring():
  data = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text())
  item = next(p for p in data["params"] if p["name"] == "K9HudNavdy")
  assert item["max"] == 3 and item["options"]["ko"][3] == "3 부드러운 3D 판"
  assert "3D" in item["descr"] and "20 FPS" in item["descr"] and "20 FPS" in item["edescr"] and "20 FPS" in item["cdescr"]
  assert "차선 평활" not in item["descr"] and "smoothed lanes" not in item["edescr"]   # 11/18 revert: no line smoothing
  src = (CLUSTER / "main.py").read_text()
  assert src.count("renderer.set_k9_hud_3d(k9_hud_menu == 3)") == 2
  assert cr.ClusterUiRenderer.k9_hud_3d is False


def test_jetson_fps_policy(tmp_path):
  sys.path.insert(0, str(JETLINK))
  import hud
  f = tmp_path / "fps"
  assert hud.k9_start_fps(f) == 10
  assert hud.k9_menu_fps_policy(3, f, received=None) is None and not f.exists()   # no Params yet: never restart
  assert hud.k9_menu_fps_policy(3, f, received=1.0) == 20 and hud.k9_start_fps(f) == 20
  assert hud.k9_menu_fps_policy(1, f, received=1.0) == 10 and hud.k9_start_fps(f) == 10
  f.write_text("55")
  assert hud.k9_start_fps(f) == 10
  text = (JETLINK / "hud.py").read_text()
  assert "'--fps', str(k9_start_fps())" in text and "'--usb-h264-ffmpeg-threads', '1'" in text


# ---- render (GL) -------------------------------------------------------------------------------------------------

def _frames(fn):
  import render_gear_screens as rg
  from openpilot.common import indicator_display
  from openpilot.selfdrive.carrot import signal_mode
  old = (indicator_display.current, signal_mode.signal_mode, cr.EV6_WEATHER.snapshot, cr.time)
  indicator_display.current, signal_mode.signal_mode, cr.EV6_WEATHER.snapshot = (lambda: None), (lambda: 0), (lambda: None)
  fixed = time.struct_time((2026, 11, 18, 12, 0, 0, 2, 322, -1))
  cr.time = SimpleNamespace(monotonic=lambda: 10000.0, perf_counter=time.perf_counter, localtime=lambda *a: fixed,
                            strftime=time.strftime, time=lambda: 1794970800.0, sleep=time.sleep, thread_time=time.thread_time)
  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  try:
    def png(st):
      r.render_to_png_bytes(st)
      return np.array(Image.open(io.BytesIO(r.render_to_png_bytes(st))).convert("RGB"))
    return fn(r, rg, png)
  finally:
    r.close()
    indicator_display.current, signal_mode.signal_mode, cr.EV6_WEATHER.snapshot, cr.time = old


@pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")
def test_render_3_only_k9_d_and_0_1_2_unchanged():
  def body(r, rg, png):
    out = {}
    for car in ("KIA_K9", "KIA_EV6"):
      for gear in ("D", "P", "R"):
        st = road_state(rg.synthetic_state(gear))   # no blinker: its phase follows the real clock
        st = replace(st, car_fingerprint=car, gear_text=gear)
        for menu in (0, 1, 2, 3):
          r.set_k9_hud_style(min(menu, 1))
          r.set_k9_hud_navdy(1)
          r.set_k9_hud_res(menu == 2)
          r.set_k9_hud_3d(menu == 3)
          out[(car, gear, menu)] = png(st)
    r.set_k9_hud_3d(True)
    r.set_k9_hud_style(1)
    base = rg.synthetic_state("D")
    out["blue"] = png(road_state(base, lead_accel=-0.2))
    out["red"] = png(road_state(base, lead_accel=-2.5))
    r.set_k9_hud_3d(False)
    for menu in (0, 1, 2):
      r.set_k9_hud_style(min(menu, 1))
      r.set_k9_hud_res(menu == 2)
      out[("again", menu)] = png(replace(road_state(rg.synthetic_state("D")), gear_text="D"))
    return out
  f = _frames(body)
  for car in ("KIA_K9", "KIA_EV6"):
    for gear in ("D", "P", "R"):
      if car == "KIA_EV6" or gear != "D":
        assert np.array_equal(f[(car, gear, 1)], f[(car, gear, 3)]), (car, gear)   # no K9 D scene: 3 == 1
  k9 = f[("KIA_K9", "D", 3)]
  for menu in (0, 1, 2):
    assert np.array_equal(f[("KIA_K9", "D", menu)], f[("again", menu)])           # leaving 3 leaves nothing behind
    assert np.any(f[("KIA_K9", "D", menu)] != k9)
  h, w = k9.shape[:2]
  assert k9[: h // 20, w // 3: 2 * w // 3].max() < 40                               # black field above the road
  def has(img, rgb, tol=40):
    return np.any(np.all(np.abs(img.astype(int) - rgb) < tol, axis=2))
  assert has(f["blue"], k3.LEAD_BLUE) and not has(f["blue"], k3.LEAD_RED, 30)
  assert has(f["red"], k3.LEAD_RED)
