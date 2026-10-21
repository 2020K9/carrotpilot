from dataclasses import fields, replace
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

import cluster_renderer
from cluster_config import (
  CLUSTER_CAMERA_VIEW_MODE_DEFAULT,
  CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
  CLUSTER_SCREEN_MODE_DEBUG,
  CLUSTER_SCREEN_MODE_DEFAULT,
  CLUSTER_SCREEN_MODE_FULLSCREEN_3D,
)
from cluster_live import OpenpilotLiveSource, standby_state
from cluster_models import LaneMarking, ModelPathPoint, RadarPoint
from cluster_renderer import LANE_CONFIDENCE_HOLD_MAX_S, ClusterUiRenderer
from cluster_route_replay import (
  LANE_LINE_PROBABILITY_MIN,
  RouteLogParser,
  frame_to_state,
  inner_lane_geometry_is_current,
  lanes_for_frame,
)
from cluster_scene import ClusterScene, build_cluster_scene, radar_point_is_outside_outer_lane

START = 100.0
POINTS = (ModelPathPoint(0.0, 0.0, 0.0), ModelPathPoint(10.0, 0.0, 0.0))


def inner(side, visible, dip=False, points=POINTS):
  return LaneMarking(
    -0.5 if side == "left" else 0.5,
    visible=visible,
    width=7,
    model_points=points,
    confidence_hold_side=side,
    confidence_dip=dip,
  )


def lane_state(left=True, right=True, left_dip=False, right_dip=False, left_conf=0.9, right_conf=0.9, lanes=None, **kw):
  if lanes is None:
    lanes = (inner("left", left, left_dip), inner("right", right, right_dip))
  return replace(
    standby_state(),
    lanes=lanes,
    left_lane_line_confidence=left_conf,
    right_lane_line_confidence=right_conf,
    camera_view_mode=CLUSTER_CAMERA_VIEW_MODE_DEFAULT,
    **kw,
  )


def held_sides(r, state, now):
  indices = r._lane_confidence_hold_indices(state, now=now)
  return {state.lanes[i].confidence_hold_side for i in indices}


def dip_state(**kw):
  kw.setdefault("left_conf", 0.3)
  return lane_state(left=False, left_dip=True, **kw)


def test_hold_within_threshold_then_expires_by_render_time():
  r = ClusterUiRenderer()
  assert held_sides(r, lane_state(), START) == set()
  dip = dip_state()
  assert held_sides(r, dip, START + 0.1) == {"left"}
  assert held_sides(r, dip, START + LANE_CONFIDENCE_HOLD_MAX_S) == {"left"}
  assert held_sides(r, dip, START + LANE_CONFIDENCE_HOLD_MAX_S + 0.01) == set()
  # Once expired it never comes back from the same dip.
  assert held_sides(r, dip, START + 0.05) == set()


def test_repeated_hidden_input_does_not_refresh_hold():
  r = ClusterUiRenderer()
  held_sides(r, lane_state(), START)
  dip = dip_state()
  times = [0.02 * i for i in range(1, 40)]
  shown = ["left" in held_sides(r, dip, START + t) for t in times]
  assert all(s for s, t in zip(shown, times) if t <= LANE_CONFIDENCE_HOLD_MAX_S)
  assert not any(s for s, t in zip(shown, times) if t > LANE_CONFIDENCE_HOLD_MAX_S + 1e-9)


def test_repeated_visible_frame_rearms_start_baseline_behavior():
  # Documented limit: the window starts at the last render where the line was
  # drawn by the existing rules. A repeated (possibly up to 0.5 s old) visible
  # frame keeps re-arming it; the hold adds no tighter freshness rule.
  r = ClusterUiRenderer()
  visible = lane_state()
  for i in range(5):
    held_sides(r, visible, START + 0.1 * i)
  assert r._lane_confidence_last_visible_at["left"] == pytest.approx(START + 0.4)
  assert held_sides(r, dip_state(), START + 0.4 + LANE_CONFIDENCE_HOLD_MAX_S) == {"left"}


def test_hold_time_comes_from_monotonic_clock(monkeypatch):
  clock = {"now": START}
  monkeypatch.setattr(cluster_renderer.time, "monotonic", lambda: clock["now"])
  r = ClusterUiRenderer()
  r._lane_confidence_hold_indices(lane_state())
  clock["now"] = START + 0.2
  assert r._lane_confidence_hold_indices(dip_state()) == frozenset({0})
  clock["now"] = START + 0.31
  assert r._lane_confidence_hold_indices(dip_state()) == frozenset()


@pytest.mark.parametrize("state", [
  dip_state(left_conf=None),
  lane_state(left=False, left_dip=False, left_conf=0.3),
  dip_state(left_conf=0.9),
  lane_state(lanes=(inner("right", True),), left_conf=0.3),
])
def test_missing_invalid_or_edge_cases_drop_immediately(state):
  r = ClusterUiRenderer()
  held_sides(r, lane_state(), START)
  assert "left" not in held_sides(r, state, START + 0.05)
  # And they disarm it: a valid dip right after cannot resurrect the old line.
  assert "left" not in held_sides(r, dip_state(), START + 0.1)


def test_confidence_none_keeps_baseline_visible_line_and_adds_nothing():
  # Unknown confidence never hides a line the existing rules draw; it only
  # prevents new held lines.
  r = ClusterUiRenderer()
  stale_visible = lane_state(left_conf=None, right_conf=None)
  assert r._lane_confidence_hold_indices(stale_visible, now=START) == frozenset()
  held = build_cluster_scene(stale_visible, held_lane_indices=frozenset())
  assert held.lane_markings == build_cluster_scene(stale_visible).lane_markings
  assert len(held.lane_markings) > 0
  # A later dip after that stale period has nothing armed.
  assert held_sides(r, dip_state(), START + 0.05) == set()


def test_sides_are_independent():
  r = ClusterUiRenderer()
  held_sides(r, lane_state(), START)
  held_sides(r, lane_state(left=False, left_dip=True, left_conf=0.3), START + 0.2)
  both = lane_state(left=False, right=False, left_dip=True, right_dip=True, left_conf=0.3, right_conf=0.3)
  assert held_sides(r, both, START + 0.35) == {"right"}
  assert held_sides(r, both, START + 0.51) == set()


@pytest.mark.parametrize("screen_mode, camera_view", [
  (CLUSTER_SCREEN_MODE_DEBUG, CLUSTER_CAMERA_VIEW_MODE_DEFAULT),
  (CLUSTER_SCREEN_MODE_FULLSCREEN_3D, CLUSTER_CAMERA_VIEW_MODE_DEFAULT),
  (CLUSTER_SCREEN_MODE_DEFAULT, CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA),
])
def test_other_modes_unchanged_and_transition_clears_hold(screen_mode, camera_view):
  r = ClusterUiRenderer(screen_mode=screen_mode)
  r._lane_confidence_hold_indices(replace(lane_state(), camera_view_mode=camera_view), now=START)
  dip = replace(dip_state(), camera_view_mode=camera_view)
  assert r._lane_confidence_hold_indices(dip, now=START + 0.1) == frozenset()

  r = ClusterUiRenderer()
  held_sides(r, lane_state(), START)
  r.set_screen_mode(screen_mode)
  r._lane_confidence_hold_indices(replace(dip_state(), camera_view_mode=camera_view), now=START + 0.05)
  r.set_screen_mode(CLUSTER_SCREEN_MODE_DEFAULT)
  assert held_sides(r, dip_state(), START + 0.1) == set()
  held_sides(r, lane_state(), START + 0.2)
  assert held_sides(r, dip_state(), START + 0.3) == {"left"}


def render_spy(r, monkeypatch):
  seen = []

  def fake_world(state, lights):
    seen.append((state, r._lane_confidence_held_indices))

  monkeypatch.setattr(r, "_render_world", fake_world)
  monkeypatch.setattr(r, "_draw_hud", lambda state, lights: None)
  monkeypatch.setattr(r, "_draw_alert_overlay", lambda alert: None)
  return seen


def test_render_never_replaces_or_mutates_incoming_state(monkeypatch):
  r = ClusterUiRenderer()
  seen = render_spy(r, monkeypatch)
  clock = {"now": START}
  monkeypatch.setattr(cluster_renderer.time, "monotonic", lambda: clock["now"])
  first = lane_state()
  r.render(first, (False, False))
  clock["now"] = START + 0.1
  dip = dip_state()
  dip_lanes = dip.lanes
  r.render(dip, (False, False))
  clock["now"] = START + 0.4
  r.render(dip, (False, False))
  assert [state is given for (state, _), given in zip(seen, (first, dip, dip))] == [True, True, True]
  assert dip.lanes is dip_lanes and not dip.lanes[0].visible
  assert [held for _, held in seen] == [frozenset(), frozenset({0}), frozenset()]


# --- held indices only change thin lane drawing --------------------------------

def radar_state(left_visible, left_dip):
  # Inner left/right plus outer right. A radar point between the hidden inner left
  # and the inner right changes its outer-lane classification when the left line
  # counts as visible, so a leaked hold would change radar/object decisions.
  left_points = (ModelPathPoint(0.0, -4.0, 0.0), ModelPathPoint(80.0, -4.0, 0.0))
  right_points = (ModelPathPoint(0.0, 1.8, 0.0), ModelPathPoint(80.0, 1.8, 0.0))
  lanes = (
    inner("left", left_visible, left_dip, left_points),
    inner("right", True, points=right_points),
    LaneMarking(1.5, visible=True, width=5, model_points=()),
  )
  points = (
    # Between the wide hidden left line (-4.0 m) and the right line: an outer-lane
    # reject now, but a vehicle sprite if the held line counted as visible.
    RadarPoint("R1", 30.0, -3.0, "radar", relative_speed_mps=0.0, probability=1.0, valid=1),
    RadarPoint("R2", 40.0, 1.0, "radar", relative_speed_mps=0.0, valid_count=20),
    RadarPoint("R3", 25.0, 6.0, "radar", relative_speed_mps=0.0, valid_count=20),
  )
  return lane_state(lanes=lanes, left_conf=0.3, radar_points=points)


def test_held_lines_do_not_change_radar_or_object_decisions():
  base = radar_state(left_visible=False, left_dip=True)
  leaked = radar_state(left_visible=True, left_dip=False)
  lane_width_m = base.lane_width_m or 3.6
  # Sensitivity: the classification really depends on that line's visibility.
  assert any(
    radar_point_is_outside_outer_lane(p, base, lane_width_m)
    != radar_point_is_outside_outer_lane(p, leaked, lane_width_m)
    for p in base.radar_points
  )
  plain = build_cluster_scene(base)
  # A leaked hold (state with the line visible) adds a vehicle sprite.
  assert len(build_cluster_scene(leaked).vehicles) == len(plain.vehicles) + 1
  held = build_cluster_scene(base, held_lane_indices=frozenset({0}))
  assert held.radar_points == plain.radar_points
  assert held.vehicles == plain.vehicles
  for field in fields(ClusterScene):
    if field.name != "lane_markings":
      assert getattr(held, field.name) == getattr(plain, field.name), field.name


def test_held_scene_differs_from_baseline_only_in_that_thin_line():
  base = radar_state(left_visible=False, left_dip=True)
  plain = build_cluster_scene(base)
  held = build_cluster_scene(base, held_lane_indices=frozenset({0}))
  assert held.lane_markings != plain.lane_markings
  # Lane markings equal the ones drawn for a state with only that line visible,
  # and nothing other than lane drawing comes from that state.
  only_line = build_cluster_scene(replace(base, lanes=(replace(base.lanes[0], visible=True),) + base.lanes[1:]))
  assert held.lane_markings == only_line.lane_markings


def test_renderer_scene_cache_keys_on_held_indices():
  r = ClusterUiRenderer()
  base = radar_state(left_visible=False, left_dip=True)
  theme = r._current_theme()
  plain = r._scene_for_state(base, True, theme)
  held = r._scene_for_state(base, True, theme, frozenset({0}))
  assert held is not plain and held.lane_markings != plain.lane_markings
  assert r._scene_for_state(base, True, theme).lane_markings == plain.lane_markings


# --- parser geometry gate --------------------------------------------------------

def parser_with(left_prob, right_prob, left_y=-1.8, right_y=1.8, left_edge=-3.0, right_edge=3.0, current=True):
  p = RouteLogParser(recompute_cutins=False)
  p.lane_width_m = 3.6
  p.left_lane_y_m = left_y
  p.right_lane_y_m = right_y
  p.left_lane_prob = left_prob
  p.right_lane_prob = right_prob
  p.left_road_edge_y_m = left_edge
  p.right_road_edge_y_m = right_edge
  p.inner_lane_geometry_current = current
  return p


def test_parser_flags_only_confidence_dips_with_current_geometry():
  low = LANE_LINE_PROBABILITY_MIN - 0.01
  v = parser_with(low, LANE_LINE_PROBABILITY_MIN)._lane_values()
  assert (v["left_visible"], v["left_confidence_dip"]) == (False, True)
  assert (v["right_visible"], v["right_confidence_dip"]) == (True, False)
  for left_y, right_y in ((None, 1.8), (1.8, -1.8)):
    v = parser_with(low, low, left_y=left_y, right_y=right_y)._lane_values()
    assert not v["left_confidence_dip"] and not v["right_confidence_dip"]
  # Retained y from an older message while the latest geometry is not current.
  v = parser_with(low, low, current=False)._lane_values()
  assert not v["left_confidence_dip"] and not v["right_confidence_dip"]
  # Baseline visibility does not depend on the new gate.
  assert v["left_visible"] == parser_with(low, low)._lane_values()["left_visible"]
  p = parser_with(low, low, left_edge=-1.7)
  p.lane_width_m = 2.4
  v = p._lane_values()
  assert v["left_road_edge_offset"] is not None and v["left_offset"] < v["left_road_edge_offset"]
  assert not v["left_confidence_dip"] and not v["left_visible"]


def line(xs, ys):
  return SimpleNamespace(x=list(xs), y=list(ys))


XS = [float(i) * 5.0 for i in range(10)]


def lines(left_ys=None, right_ys=None, left_xs=None, right_xs=None):
  left_ys = [-1.8] * 10 if left_ys is None else left_ys
  right_ys = [1.8] * 10 if right_ys is None else right_ys
  return [
    line(XS, [-5.4] * 10),
    line(XS if left_xs is None else left_xs, left_ys),
    line(XS if right_xs is None else right_xs, right_ys),
    line(XS, [5.4] * 10),
  ]


@pytest.mark.parametrize("lane_lines, expected", [
  (lines(), True),
  (None, False),
  (lines()[:2], False),
  (lines(left_ys=[-1.8] * 9 + [math.nan]), False),
  (lines(right_ys=[math.inf] + [1.8] * 9), False),
  (lines(left_xs=XS[:5] + [math.nan] + XS[6:]), False),
  (lines(left_ys=[-1.8] * 9), False),
  (lines(left_ys=[], right_ys=[]), False),
  (lines(left_ys=[-1.8] * 5 + [2.0] * 5), False),
  (lines(left_ys=[1.8] * 10, right_ys=[-1.8] * 10), False),
  (lines(left_xs=[0.0] * 10), False),
])
def test_inner_geometry_gate_rejects_nonfinite_missing_and_crossed(lane_lines, expected):
  assert inner_lane_geometry_is_current(lane_lines) is expected


def test_lanes_for_frame_marks_inner_lines_only():
  f = SimpleNamespace()
  f.active_lane_line = True
  f.left_lane_color = None
  f.right_lane_color = None
  f.lane_change = None
  f.left_lane_offset = -0.5
  f.right_lane_offset = 0.5
  f.left_lane_visible = False
  f.right_lane_visible = True
  f.left_lane_confidence_dip = True
  f.right_lane_confidence_dip = False
  f.left_road_edge_offset = None
  f.right_road_edge_offset = None
  f.left_lane_style = "solid"
  f.right_lane_style = "solid"
  f.extra_left_lane_visible = False
  f.extra_right_lane_visible = False
  f.model_lane_lines = (POINTS, POINTS, POINTS, POINTS)
  f.lane_width_m = 3.6
  f.lane_center_offset_m = 0.0
  markings = lanes_for_frame(f)
  by_side = {m.confidence_hold_side: m for m in markings if m.confidence_hold_side}
  assert set(by_side) == {"left", "right"}
  assert by_side["left"].confidence_dip and not by_side["left"].visible
  assert not by_side["right"].confidence_dip
  assert sum(1 for m in markings if m.confidence_hold_side is None) == 2
  animated = lanes_for_frame(f, 0.0, True)
  assert not any(m.confidence_dip for m in animated)


# --- real parser + lanes_for_frame + live confidence + renderer pipeline ---------

cereal_log = pytest.importorskip("openpilot.cereal").log


def model_v2(left_prob, right_prob=0.9, lane_lines=None, probs=None):
  msg = cereal_log.Event.new_message().init("modelV2")
  if lane_lines != "missing":
    source = lines() if lane_lines is None else lane_lines
    out = msg.init("laneLines", len(source))
    for target, src in zip(out, source):
      target.x = src.x
      target.y = src.y
      target.z = [0.0] * len(src.x)
  msg.laneLineProbs = [0.5, left_prob, right_prob, 0.5] if probs is None else probs
  edges = msg.init("roadEdges", 2)
  for target, y in zip(edges, (-3.6, 3.6)):
    target.x = XS
    target.y = [y] * 10
    target.z = [0.0] * 10
  return msg.as_reader()


def car_state():
  msg = cereal_log.Event.new_message().init("carState")
  msg.vEgo = 20.0
  return msg.as_reader()


class LivePipeline:
  """Live source wiring: parser updates, carState frame, frame_to_state, and the
  production modelV2 confidence (validity + 0.5 s freshness) from cluster_live."""

  def __init__(self, renderer, monkeypatch):
    self.parser = RouteLogParser(recompute_cutins=False)
    self.renderer = renderer
    self.model = None
    self.model_valid = True
    self.model_receive_t = None
    self.clock = {"now": START}
    monkeypatch.setattr(cluster_renderer.time, "monotonic", lambda: self.clock["now"])
    self.seen = render_spy(renderer, monkeypatch)

  def _service_valid(self, _service):
    return self.model_valid

  def _service_receive_time(self, _service):
    return self.model_receive_t

  def _service_data(self, _service):
    return self.model

  def step(self, now, model=None):
    self.clock["now"] = now
    if model is not None:
      self.parser._update_model_v2(model, now)
      self.model = model
      self.model_receive_t = now
    state = frame_to_state(self.parser._frame_from_car_state(car_state(), now))
    left, right = OpenpilotLiveSource._live_lane_line_confidence(self, now)
    state = replace(
      state,
      left_lane_line_confidence=left,
      right_lane_line_confidence=right,
      camera_view_mode=CLUSTER_CAMERA_VIEW_MODE_DEFAULT,
    )
    self.renderer.render(state, (False, False))
    rendered, held = self.seen[-1]
    assert rendered is state
    return state, {state.lanes[i].confidence_hold_side for i in held}


def baseline_left_visible(state):
  return next(m.visible for m in state.lanes if m.confidence_hold_side == "left")


def test_pipeline_valid_dip_is_held_then_expires(monkeypatch):
  p = LivePipeline(ClusterUiRenderer(), monkeypatch)
  state, held = p.step(START, model_v2(0.9))
  assert baseline_left_visible(state) and held == set()
  state, held = p.step(START + 0.05, model_v2(0.3))
  assert not baseline_left_visible(state) and held == {"left"}
  state, held = p.step(START + LANE_CONFIDENCE_HOLD_MAX_S, model_v2(0.3))
  assert held == {"left"}
  state, held = p.step(START + LANE_CONFIDENCE_HOLD_MAX_S + 0.05, model_v2(0.3))
  assert held == set()


@pytest.mark.parametrize("dip_lines", [
  lines(left_ys=[-1.8] * 9 + [math.nan]),
  lines(right_ys=[math.nan] * 10),
  lines(left_xs=XS[:3] + [math.inf] + XS[4:]),
  lines(left_ys=[-1.8] * 5 + [2.5] * 5),
  lines(left_ys=[1.8] * 10, right_ys=[-1.8] * 10),
  "missing",
])
def test_pipeline_invalid_or_missing_geometry_is_never_held(monkeypatch, dip_lines):
  p = LivePipeline(ClusterUiRenderer(), monkeypatch)
  p.step(START, model_v2(0.9))
  state, held = p.step(START + 0.05, model_v2(0.3, lane_lines=dip_lines))
  assert held == set()
  assert not p.parser.inner_lane_geometry_current


def test_pipeline_missing_model_input_after_freshness_limit(monkeypatch):
  p = LivePipeline(ClusterUiRenderer(), monkeypatch)
  p.step(START, model_v2(0.9))
  # No new modelV2: the existing frame keeps being drawn (baseline) while fresh.
  state, held = p.step(START + 0.4)
  assert baseline_left_visible(state) and held == set()
  # Past the existing 0.5 s rule the confidence is unknown: no hold arms.
  state, held = p.step(START + 0.6)
  assert state.left_lane_line_confidence is None and held == set()
  assert baseline_left_visible(state)  # stale baseline line is not hidden
  state, held = p.step(START + 0.65, model_v2(0.3))
  assert held == set()


def test_pipeline_invalid_model_service_or_nonfinite_probability(monkeypatch):
  p = LivePipeline(ClusterUiRenderer(), monkeypatch)
  p.step(START, model_v2(0.9))
  p.model_valid = False
  _, held = p.step(START + 0.05, model_v2(0.3))
  assert held == set()

  p = LivePipeline(ClusterUiRenderer(), monkeypatch)
  p.step(START, model_v2(0.9))
  state, held = p.step(START + 0.05, model_v2(0.3, probs=[0.5, math.nan, 0.9, 0.5]))
  assert state.left_lane_line_confidence is None and held == set()


def test_pipeline_held_frame_keeps_radar_and_objects_identical(monkeypatch):
  r = ClusterUiRenderer()
  p = LivePipeline(r, monkeypatch)
  p.step(START, model_v2(0.9))
  state, held = p.step(START + 0.05, model_v2(0.3))
  assert held == {"left"}
  state = replace(state, radar_points=radar_state(False, True).radar_points)
  theme = r._current_theme()
  plain = build_cluster_scene(state, theme=theme)
  held_scene = build_cluster_scene(state, theme=theme, held_lane_indices=r._lane_confidence_held_indices)
  assert held_scene.lane_markings != plain.lane_markings
  for field in fields(ClusterScene):
    if field.name != "lane_markings":
      assert getattr(held_scene, field.name) == getattr(plain, field.name), field.name
