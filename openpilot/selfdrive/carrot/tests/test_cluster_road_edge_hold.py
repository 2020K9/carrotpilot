from dataclasses import replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_config import (
  CLUSTER_CAMERA_VIEW_MODE_DEFAULT,
  CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
  CLUSTER_SCREEN_MODE_DEBUG,
  CLUSTER_SCREEN_MODE_DEFAULT,
  CLUSTER_SCREEN_MODE_FULLSCREEN_3D,
)
from cluster_live import standby_state
from cluster_models import ModelPathPoint, RadarPoint
from cluster_renderer import LANE_CONFIDENCE_HOLD_MAX_S, ClusterUiRenderer
from cluster_route_replay import RouteLogParser, frame_to_state, road_edge_geometry_is_current, road_edge_outside_lane_line
from cluster_scene import build_cluster_scene

START = 100.0
EDGE = tuple(ModelPathPoint(float(x), -2.0, 0.0) for x in range(0, 60, 10))
REDGE = tuple(ModelPathPoint(float(x), 2.0, 0.0) for x in range(0, 60, 10))
RADAR = (RadarPoint(30.0, 0.0, 0.0, "radar", 1),)


def edge_state(left_drawn=True, right_drawn=True, left_hold=(), right_hold=(), left_conf=0.9, right_conf=0.9, **kw):
  return replace(
    standby_state(),
    radar_points=RADAR,
    left_road_edge_offset=-1.0 if left_drawn else None,
    right_road_edge_offset=1.0 if right_drawn else None,
    left_road_edge_points=EDGE if left_drawn else (),
    right_road_edge_points=REDGE if right_drawn else (),
    left_road_edge_hold_points=left_hold,
    right_road_edge_hold_points=right_hold,
    left_lane_line_confidence=left_conf,
    right_lane_line_confidence=right_conf,
    **{"camera_view_mode": CLUSTER_CAMERA_VIEW_MODE_DEFAULT, **kw},
  )


def strips_multiset(strips):
  return sorted(repr(strip) for strip in strips)


def left_gap(**kw):
  return edge_state(left_drawn=False, left_hold=EDGE, **kw)


def test_hold_boundary_and_expiry_without_rearm():
  r = ClusterUiRenderer()
  assert r._road_edge_hold_sides(edge_state(), now=START) == frozenset()
  gap = left_gap()
  assert r._road_edge_hold_sides(gap, now=START + 0.1) == {"left"}
  assert r._road_edge_hold_sides(gap, now=START + LANE_CONFIDENCE_HOLD_MAX_S) == {"left"}
  assert r._road_edge_hold_sides(gap, now=START + LANE_CONFIDENCE_HOLD_MAX_S + 0.001) == frozenset()
  # Expired: the same gap never resumes, and holding never moved the start.
  assert r._road_edge_hold_sides(gap, now=START + 0.05) == frozenset()


def test_repeated_gap_does_not_refresh_expiry():
  r = ClusterUiRenderer()
  r._road_edge_hold_sides(edge_state(), now=START)
  for step in range(1, 4):
    assert r._road_edge_hold_sides(left_gap(), now=START + 0.1 * step) == {"left"}
  assert r._road_edge_hold_sides(left_gap(), now=START + 0.31) == frozenset()


@pytest.mark.parametrize("gap", [
  edge_state(left_drawn=False),                       # no current points: invalid/missing geometry
  left_gap(left_conf=None),                           # stale/invalid modelV2
  left_gap(right_conf=None),
])
def test_missing_invalid_or_stale_hides_immediately(gap):
  r = ClusterUiRenderer()
  r._road_edge_hold_sides(edge_state(), now=START)
  assert r._road_edge_hold_sides(gap, now=START + 0.01) == frozenset()
  # And a later valid gap cannot use the dropped start.
  assert r._road_edge_hold_sides(left_gap(), now=START + 0.02) == frozenset()


def test_no_hold_without_previous_drawn_edge():
  r = ClusterUiRenderer()
  assert r._road_edge_hold_sides(left_gap(), now=START) == frozenset()


def test_sides_are_independent():
  r = ClusterUiRenderer()
  r._road_edge_hold_sides(edge_state(), now=START)
  both = edge_state(left_drawn=False, right_drawn=False, left_hold=EDGE, right_hold=REDGE)
  assert r._road_edge_hold_sides(both, now=START + 0.1) == {"left", "right"}
  right_only = edge_state(left_drawn=False, right_drawn=False, right_hold=REDGE)
  assert r._road_edge_hold_sides(right_only, now=START + 0.2) == {"right"}
  assert r._road_edge_hold_sides(both, now=START + 0.25) == {"right"}


@pytest.mark.parametrize("screen_mode,camera_view", [
  (CLUSTER_SCREEN_MODE_DEBUG, CLUSTER_CAMERA_VIEW_MODE_DEFAULT),
  (CLUSTER_SCREEN_MODE_FULLSCREEN_3D, CLUSTER_CAMERA_VIEW_MODE_DEFAULT),
  (CLUSTER_SCREEN_MODE_DEFAULT, CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA),
])
def test_other_modes_never_hold_and_switch_clears(screen_mode, camera_view):
  r = ClusterUiRenderer()
  r._road_edge_hold_sides(edge_state(), now=START)
  r.screen_mode = screen_mode
  assert r._road_edge_hold_sides(left_gap(camera_view_mode=camera_view) if camera_view != CLUSTER_CAMERA_VIEW_MODE_DEFAULT
                                 else replace(left_gap()), now=START + 0.05) == frozenset()
  r.screen_mode = CLUSTER_SCREEN_MODE_DEFAULT
  assert r._road_edge_hold_sides(left_gap(), now=START + 0.06) == frozenset()


def test_scene_draws_hold_only_for_held_side_and_keeps_everything_else():
  gap = left_gap()
  plain = build_cluster_scene(gap)
  held = build_cluster_scene(gap, held_road_edge_sides=frozenset({"left"}))
  assert held.road_edges != plain.road_edges
  # Held edge is drawn exactly like the same current points drawn by baseline rules.
  drawn = build_cluster_scene(replace(gap, left_road_edge_points=EDGE))
  assert held.road_edges == drawn.road_edges
  for field in ("planned_path", "lane_markings", "highlight_lanes", "radar_points", "vehicles", "road_surface", "camera"):
    assert getattr(held, field) == getattr(plain, field), field
  # Listed side without hold points draws nothing extra.
  assert build_cluster_scene(left_gap(), held_road_edge_sides=frozenset({"right"})).road_edges == plain.road_edges


def test_render_passes_sides_to_cache_without_mutating_state(monkeypatch):
  r = ClusterUiRenderer()
  theme = r._current_theme()
  gap = left_gap()
  plain = r._scene_for_state(gap, True, theme)
  held = r._scene_for_state(gap, True, theme, frozenset(), frozenset({"left"}))
  assert held is not plain and held.road_edges != plain.road_edges
  assert held.planned_path == plain.planned_path
  assert r._scene_for_state(gap, True, theme).road_edges == plain.road_edges
  before = replace(gap)
  r._road_edge_hold_sides(edge_state(), now=START)
  r._road_edge_hold_sides(gap, now=START + 0.1)
  assert gap == before


# --- parser gate ---------------------------------------------------------------

def flat(y, xs=range(0, 50, 10)):
  return tuple(ModelPathPoint(float(x), y) for x in xs)


def parser_with(left_edge=-3.0, right_edge=3.0, edges_current=True, lanes_current=True, left_edge_points=None):
  p = RouteLogParser(recompute_cutins=False)
  # Model lines match the first-point values (flat), as a parsed message would.
  p.model_lane_lines = (flat(-5.4), flat(-1.8), flat(1.8), flat(5.4))
  p.model_road_edges = (
    left_edge_points if left_edge_points is not None else (flat(left_edge) if left_edge is not None else ()),
    flat(right_edge) if right_edge is not None else (),
  )
  p.lane_width_m = 3.6
  p.left_lane_y_m = -1.8
  p.right_lane_y_m = 1.8
  p.left_road_edge_y_m = left_edge
  p.right_road_edge_y_m = right_edge
  p.inner_lane_geometry_current = lanes_current
  p.road_edge_geometry_current = edges_current
  return p


def test_parser_flags_only_near_lane_gate_drops_with_current_geometry():
  v = parser_with()._lane_values()
  assert v["left_road_edge_offset"] is not None and not v["left_road_edge_gate_dip"]
  # Edge 2.2 m left of centre with 3.6 m lanes: offset -0.61 > -0.68 -> hidden by gate.
  v = parser_with(left_edge=-2.2)._lane_values()
  assert v["left_road_edge_offset"] is None and v["left_road_edge_gate_dip"]
  assert v["right_road_edge_offset"] is not None and not v["right_road_edge_gate_dip"]
  for kw in ({"edges_current": False}, {"lanes_current": False}, {"left_edge": None}, {"left_edge": 0.5}):
    kw.setdefault("left_edge", -2.2)
    v = parser_with(**kw)._lane_values()
    assert v["left_road_edge_offset"] is None and not v["left_road_edge_gate_dip"], kw
  # Edge inside the ego lane or on the inner lane line: never held (no wrong-side geometry).
  for left_edge in (-1.8, -1.0, -0.1):
    v = parser_with(left_edge=left_edge)._lane_values()
    assert v["left_road_edge_offset"] is None and not v["left_road_edge_gate_dip"], left_edge
  # Baseline bounds/visibility unchanged by the new flag.
  base = parser_with(left_edge=-2.2)
  base.road_edge_geometry_current = False
  a, b = base._lane_values(), parser_with(left_edge=-2.2)._lane_values()
  for key in a:
    if not key.endswith("gate_dip"):
      assert a[key] == b[key], key


def edge(xs, ys):
  return SimpleNamespace(x=xs, y=ys)


@pytest.mark.parametrize("edges,expected", [
  ([edge([0.0, 10.0], [-2.0, -2.0]), edge([0.0, 10.0], [2.0, 2.0])], True),
  (None, False),
  ([edge([0.0, 10.0], [-2.0, -2.0])], False),
  ([edge([0.0, math.nan], [-2.0, -2.0]), edge([0.0, 10.0], [2.0, 2.0])], False),
  ([edge([0.0, 10.0], [-2.0, math.inf]), edge([0.0, 10.0], [2.0, 2.0])], False),
  ([edge([0.0], [-2.0]), edge([0.0, 10.0], [2.0, 2.0])], False),
  ([edge([0.0, 10.0], [-2.0]), edge([0.0, 10.0], [2.0, 2.0])], False),
])
def test_road_edge_geometry_gate(edges, expected):
  assert road_edge_geometry_is_current(edges) is expected


XS = [float(x) for x in range(0, 50, 5)]


def real_model(left_edge_y, finite=True, left_edge_ys=None):
  from cereal import log as cereal_log
  msg = cereal_log.Event.new_message().init("modelV2")
  lane_lines = msg.init("laneLines", 4)
  for target, y in zip(lane_lines, (-5.4, -1.8, 1.8, 5.4)):
    target.x = XS
    target.y = [y] * 10
    target.z = [0.0] * 10
  msg.laneLineProbs = [0.5, 0.9, 0.9, 0.5]
  edges = msg.init("roadEdges", 2)
  for target, y in zip(edges, (left_edge_y, 3.6)):
    target.x = XS
    target.y = [y] * 10 if finite else [y] * 9 + [math.nan]
  if left_edge_ys is not None:
    edges[0].y = left_edge_ys
    target.z = [0.0] * 10
  position = msg.init("position")
  position.x = XS
  position.y = [0.0] * 10
  position.z = [0.0] * 10
  return msg.as_reader()


def real_state(parser, model, t):
  from cereal import log as cereal_log
  parser._update_model_v2(model, t)
  car = cereal_log.Event.new_message().init("carState")
  car.vEgo = 20.0
  return frame_to_state(parser._frame_from_car_state(car.as_reader(), t))


def test_real_parser_pipeline_hold_points_and_planned_path_unchanged():
  pytest.importorskip("cereal")
  parser = RouteLogParser(recompute_cutins=False)
  drawn = real_state(parser, real_model(-3.6), START)
  assert drawn.left_road_edge_points and drawn.left_road_edge_hold_points == ()
  gated = real_state(parser, real_model(-2.2), START + 0.05)
  # Baseline still hides the edge everywhere radar/objects look.
  assert gated.left_road_edge_offset is None and gated.left_road_edge_points == ()
  assert [p.lateral_m for p in gated.left_road_edge_hold_points] == pytest.approx([-2.2] * 10)
  assert gated.right_road_edge_hold_points == ()
  plain = build_cluster_scene(gated)
  held = build_cluster_scene(gated, held_road_edge_sides=frozenset({"left"}))
  assert held.road_edges != plain.road_edges
  assert held.planned_path == plain.planned_path and held.lane_markings == plain.lane_markings
  # Non-finite current edge: no hold geometry at all (immediate hide).
  invalid = real_state(parser, real_model(-2.2, finite=False), START + 0.1)
  assert invalid.left_road_edge_hold_points == () and invalid.right_road_edge_hold_points == ()


# Constructed unit inputs (not log data): edge starts outside its lane line, then crosses it ahead.
FAR_CROSS = tuple(ModelPathPoint(float(x), y) for x, y in zip(range(0, 50, 10), (-2.2, -2.2, -2.0, -1.6, -1.2)))


def test_edge_must_stay_outside_own_lane_over_whole_horizon():
  lane_l, lane_r = flat(-1.8), flat(1.8)
  assert road_edge_outside_lane_line(flat(-2.2), lane_l, left=True)
  assert road_edge_outside_lane_line(flat(2.2), lane_r, left=False)
  assert not road_edge_outside_lane_line(FAR_CROSS, lane_l, left=True)
  mirrored = tuple(ModelPathPoint(p.forward_m, -p.lateral_m) for p in FAR_CROSS)
  assert not road_edge_outside_lane_line(mirrored, lane_r, left=False)
  # Touching the lane line ahead, missing points, or beyond the lane's range: not checkable -> no hold.
  assert not road_edge_outside_lane_line(flat(-2.2)[:4] + (ModelPathPoint(40.0, -1.8),), lane_l, left=True)
  assert not road_edge_outside_lane_line((), lane_l, left=True)
  assert not road_edge_outside_lane_line(flat(-2.2)[:1], lane_l, left=True)
  assert not road_edge_outside_lane_line(flat(-2.2), lane_l[:1], left=True)
  assert not road_edge_outside_lane_line(flat(-2.2, range(0, 60, 10)), lane_l, left=True)
  # Curved lane line: interpolated between samples, edge parallel outside it stays valid.
  curved = tuple(ModelPathPoint(float(x), -1.8 + 0.02 * x) for x in range(0, 50, 10))
  edge_out = tuple(ModelPathPoint(float(x), -2.2 + 0.02 * x) for x in range(0, 45, 5))
  assert road_edge_outside_lane_line(edge_out, curved, left=True)


def test_parser_far_crossing_suppresses_hold_only():
  near_ok = parser_with(left_edge=-2.2)._lane_values()
  crossed = parser_with(left_edge=-2.2, left_edge_points=FAR_CROSS)._lane_values()
  assert near_ok["left_road_edge_gate_dip"] and not crossed["left_road_edge_gate_dip"]
  missing = parser_with(left_edge=-2.2, left_edge_points=())._lane_values()
  assert not missing["left_road_edge_gate_dip"]
  # Display bounds/visibility (radar, lanes) identical; only the hold flag changes.
  for key in near_ok:
    if key != "left_road_edge_gate_dip":
      assert near_ok[key] == crossed[key] == missing[key], key


def test_real_parser_far_crossing_edge_gets_no_hold_points():
  pytest.importorskip("cereal")
  parser = RouteLogParser(recompute_cutins=False)
  real_state(parser, real_model(-3.6), START)
  crossing = [-2.2] * 6 + [-2.0, -1.7, -1.4, -1.2]
  gated = real_state(parser, real_model(-2.2, left_edge_ys=crossing), START + 0.05)
  assert gated.left_road_edge_offset is None and gated.left_road_edge_points == ()
  assert gated.left_road_edge_hold_points == ()
