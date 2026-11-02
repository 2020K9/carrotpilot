"""Live cluster method contract: every self.<name> used by OpenpilotLiveSource exists, and the
navigation route / calibration helpers behave as before the traffic-light removal."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from dataclasses import replace  # noqa: E402

from cluster_live import OpenpilotLiveSource, standby_state  # noqa: E402
from cluster_models import NaviDebugInfo  # noqa: E402

SOURCE = CLUSTER_DIR / "cluster_live.py"


def _class_node():
  tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
  return next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "OpenpilotLiveSource")


def test_every_self_reference_resolves_to_a_method_or_assigned_attribute():
  cls = _class_node()
  methods = {n.name for n in cls.body if isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef))}
  assigned = set()
  used = set()
  for node in ast.walk(cls):
    if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name) and node.value.id == "self":
      if isinstance(node.ctx, ast.Store):
        assigned.add(node.attr)
      else:
        used.add(node.attr)
  missing = sorted(a for a in used - methods - assigned if not hasattr(OpenpilotLiveSource, a))
  assert missing == []


@pytest.mark.parametrize("name", [
  "_update_nav_route", "_with_external_nav_route", "_with_navi_speed_limit", "_external_nav_route_model_path",
  "_nav_route_anchor_from_carrot_man", "_read_cached_live_debug", "_event_service_from_param",
  "_legacy_live_parameters_steer_ratio", "_effective_steer_actuator_delay",
])
def test_navigation_and_calibration_helpers_exist(name):
  assert callable(getattr(OpenpilotLiveSource, name, None))


def _source(carrot_man=None, params=None):
  source = object.__new__(OpenpilotLiveSource)
  source.sm = {"carrotMan": carrot_man}
  source.params = params
  source.log = None
  source.messaging = None
  source._nav_route_coords = ()
  source._nav_route_model_path = ()
  source._nav_route_anchor = None
  source._steer_actuator_delay_param_s = None
  return source


def test_nav_route_update_projects_external_route_onto_model_path():
  lat0, lon0 = 37.5, 127.0
  source = _source(SimpleNamespace(xPosLat=lat0, xPosLon=lon0, xPosAngle=0.0))
  coords = [SimpleNamespace(latitude=lat0 + i * 0.0001, longitude=lon0) for i in range(6)]
  coords.append(SimpleNamespace(latitude=float("nan"), longitude=lon0))
  coords.append(SimpleNamespace(latitude=95.0, longitude=lon0))
  source._update_nav_route(SimpleNamespace(coordinates=coords))
  assert len(source._nav_route_coords) == 6

  state = standby_state()
  routed = source._with_external_nav_route(state)
  assert len(routed.model_path) >= 2
  assert routed.model_path[0].forward_m == 0.0
  assert all(abs(p.lateral_m) < 0.5 for p in routed.model_path)
  assert routed.model_path[-1].forward_m == pytest.approx(5 * 0.0001 * 40008000.0 / 360.0, rel=1e-3)
  assert source._external_nav_route_model_path() is source._nav_route_model_path  # cached for same anchor

  source._update_nav_route(SimpleNamespace(coordinates=[]))
  assert source._nav_route_coords == () and source._with_external_nav_route(state) is state


def test_nav_route_needs_a_valid_vehicle_anchor():
  source = _source(SimpleNamespace(xPosLat=0.0, xPosLon=0.0, xPosAngle=0.0))
  assert source._nav_route_anchor_from_carrot_man() is None
  source = _source(SimpleNamespace(xPosLat=37.5, xPosLon=127.0, xPosAngle=-90.0))
  assert source._nav_route_anchor_from_carrot_man() == (37.5, 127.0, 270.0)


def test_navi_speed_limit_fills_only_missing_limit():
  source = _source()
  state = replace(standby_state(), speed_limit_kph=None)
  navi = NaviDebugInfo(title="navi", speed_limit_kph=60)
  assert source._with_navi_speed_limit(state, navi).speed_limit_kph == 60
  assert source._with_navi_speed_limit(state, navi).speed_limit_source == "n"
  assert source._with_navi_speed_limit(state, None) is state
  limited = replace(state, speed_limit_kph=80)
  assert source._with_navi_speed_limit(limited, navi) is limited


def test_cached_live_debug_uses_real_cereal_parameters_and_legacy_ratio():
  from openpilot.cereal import log
  import openpilot.cereal.messaging as messaging
  delay = log.Event.new_message()
  ld = delay.init("liveDelay")
  ld.lateralDelay = 0.21
  ld.calPerc = 80
  values = {"LiveDelay": delay.to_bytes(), "LiveParameters": json.dumps({"steerRatio": 15.5}).encode()}
  source = _source(params=SimpleNamespace(get=values.get))
  source.log, source.messaging = log, messaging
  info = source._read_cached_live_debug()
  assert info.live_delay_lateral_s == pytest.approx(0.21)
  assert info.live_delay_calibration_percent == 80
  assert info.live_steer_ratio == pytest.approx(15.5)
  assert info.live_torque_valid is None

  source.params = SimpleNamespace(get=lambda key: b"not-json" if key == "LiveParameters" else None)
  assert source._read_cached_live_debug() is None
  source.params = None
  assert source._read_cached_live_debug() is None


def test_effective_steer_actuator_delay_prefers_positive_param():
  source = _source()
  assert source._effective_steer_actuator_delay(0.2) == 0.2
  source._steer_actuator_delay_param_s = 0.35
  assert source._effective_steer_actuator_delay(0.2) == 0.35
  source._steer_actuator_delay_param_s = 0.0
  assert source._effective_steer_actuator_delay(None) == 0.0
