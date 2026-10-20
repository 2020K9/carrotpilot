from dataclasses import replace
from pathlib import Path
import sys

from PIL import Image
import pytest

CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

import cluster_renderer
from cluster_models import DetectedVehicle
from cluster_renderer import EGO_CAR_ICON_PATH, ClusterUiRenderer
from cluster_scene import LIGHT_CLUSTER_THEME, build_cluster_scene
from test_cluster_scene import _cluster_state


def test_ego_car_icon_is_a_transparent_cutout():
  assert EGO_CAR_ICON_PATH.parent == CLUSTER_DIR / "assets" / "images"
  with Image.open(EGO_CAR_ICON_PATH) as image:
    assert image.mode == "RGBA"
    width, height = image.size
    assert 256 <= width <= 1024
    assert 0.9 <= height / width <= 1.1
    alpha = image.getchannel("A")
    corners = [alpha.getpixel(point) for point in ((0, 0), (width - 1, 0), (0, height - 1), (width - 1, height - 1))]
    assert corners == [0, 0, 0, 0]
    assert alpha.getpixel((width // 2, height // 2)) == 255
    histogram = alpha.histogram()
    assert 0 < sum(histogram[1:255]) < width * height * 0.05  # anti-aliased edge only


def test_only_the_ego_box_is_flagged():
  lead = DetectedVehicle("L1", 30.0, 0.0, source="radarState", primary=True, absolute_speed_kph=80.0)
  scene = build_cluster_scene(_cluster_state(detected_vehicles=(lead,)))
  egos = [vehicle for vehicle in scene.vehicles if vehicle.ego]
  assert len(egos) == 1
  assert all(not vehicle.ego for vehicle in scene.vehicles if vehicle.source)
  assert egos[0].label == ""
  assert all(not vehicle.ego for vehicle in scene.vehicles if vehicle.label == "L1")


def test_ego_texture_loads_once_with_the_vehicle_model(monkeypatch):
  renderer = object.__new__(ClusterUiRenderer)
  renderer._vehicle_model_load_attempted = False
  renderer._vehicle_model = None
  renderer._ego_car_texture = None
  loads = []
  monkeypatch.setattr(renderer, "_load_icon_texture", lambda path, label: loads.append(path) or object(), raising=False)
  monkeypatch.setattr(cluster_renderer, "VEHICLE_MODEL_PATH", Path("/nonexistent/model.obj"))
  renderer._load_vehicle_model()
  renderer._load_vehicle_model()
  assert loads == [EGO_CAR_ICON_PATH]
  assert renderer._ego_car_texture is not None


def test_sprite_rect_keeps_texture_aspect_and_sits_on_rear_edge(monkeypatch):
  class Screen:
    def __init__(self, x, y):
      self.x, self.y = x, y

  # Simple oblique projection: x right, forward goes up the screen, z goes up.
  monkeypatch.setattr(cluster_renderer, "world_to_screen_label_anchor",
                      lambda point, *_: Screen(500 + point.x * 60, 400 - point.y * 10 - point.z * 40))
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width, renderer.height = 1920, 480
  scene = build_cluster_scene(_cluster_state())
  ego = next(vehicle for vehicle in scene.vehicles if vehicle.ego)
  anchor_x, anchor_y, width, height, roll = renderer._ego_car_sprite_rect(ego, None, 0.0, 512, 519)
  assert width == pytest.approx(ego.width_m * 60, rel=0.05)
  assert height / width == pytest.approx(519 / 512)
  rear_bottom_y = 400 - (ego.center.y - ego.length_m * 0.5) * 10
  assert anchor_y == pytest.approx(rear_bottom_y, abs=1.0)
  assert abs(roll) <= cluster_renderer.EGO_CAR_ICON_MAX_ROLL_DEG


@pytest.mark.parametrize(("mode", "expect_photo"), (
  (0, True), ("0", True), ("default", True), ("mode0", True), (99, True),  # unknown -> normalized to 0
  (1, False), ("ego-bottom", False), (2, False), (3, False), (4, False),
))
def test_photo_ego_only_in_camera_view_mode_zero(mode, expect_photo):
  scene = build_cluster_scene(_cluster_state(camera_view_mode=mode))
  assert sum(vehicle.ego for vehicle in scene.vehicles) == (1 if expect_photo else 0)


def test_surround_orbit_in_mode_zero_keeps_model():
  scene = build_cluster_scene(_cluster_state(camera_view_mode=0, surround_view_active=True))
  assert not any(vehicle.ego for vehicle in scene.vehicles)


class _FakeTexture:
  width, height = 512, 519


def _scene_renderer(monkeypatch, texture):
  renderer = object.__new__(ClusterUiRenderer)
  renderer._scene_cache = None
  renderer._scene_cache_key = None
  renderer.profile_enabled = False
  renderer._ego_car_texture = texture
  renderer._vehicle_model = object()
  calls = []
  monkeypatch.setattr(renderer, "_draw_vehicle", lambda vehicle: calls.append(("model", vehicle.ego)), raising=False)
  monkeypatch.setattr(renderer, "_draw_vehicle_shadow", lambda vehicle: calls.append(("shadow", vehicle.ego)), raising=False)
  monkeypatch.setattr(renderer, "_draw_ego_car_sprite", lambda vehicle, *_: calls.append(("sprite", vehicle.ego)), raising=False)
  return renderer, calls


def _ego_draws(renderer, calls, state):
  calls.clear()
  scene = renderer._scene_for_state(state, False, LIGHT_CLUSTER_THEME)
  ego_sprite_vehicle = None
  # Mirrors the vehicle loop in ClusterUiRenderer._draw_scene.
  for vehicle in scene.vehicles:
    if vehicle.ego and getattr(renderer, "_ego_car_texture", None) is not None:
      renderer._draw_vehicle_shadow(vehicle)
      ego_sprite_vehicle = vehicle
      continue
    renderer._draw_vehicle(vehicle)
  if ego_sprite_vehicle is not None:
    renderer._draw_ego_car_sprite(ego_sprite_vehicle, None, 0.0)
  return list(calls)


def test_draw_scene_loop_matches_helper():
  source = (CLUSTER_DIR / "cluster_renderer.py").read_text(encoding="utf-8")
  assert 'if vehicle.ego and getattr(self, "_ego_car_texture", None) is not None:' in source
  assert "self._draw_ego_car_sprite(ego_sprite_vehicle, camera, scene.scene_shift_x_m)" in source


def test_live_mode_switch_toggles_photo_and_model(monkeypatch):
  renderer, calls = _scene_renderer(monkeypatch, _FakeTexture())
  base = _cluster_state(camera_view_mode=0)
  assert ("sprite", True) in _ego_draws(renderer, calls, base)
  mode1 = _ego_draws(renderer, calls, replace(base, camera_view_mode=1))
  assert ("model", False) in mode1 and not any(kind == "sprite" for kind, _ in mode1)
  assert ("sprite", True) in _ego_draws(renderer, calls, replace(base, camera_view_mode=0))
  orbit = _ego_draws(renderer, calls, replace(base, surround_view_active=True))
  assert not any(kind == "sprite" for kind, _ in orbit)


def test_mode_zero_falls_back_to_model_without_texture(monkeypatch):
  renderer, calls = _scene_renderer(monkeypatch, None)
  draws = _ego_draws(renderer, calls, _cluster_state(camera_view_mode=0))
  assert ("model", True) in draws
  assert not any(kind == "sprite" for kind, _ in draws)
