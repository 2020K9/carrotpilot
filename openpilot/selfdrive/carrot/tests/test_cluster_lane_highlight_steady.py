from dataclasses import replace
from pathlib import Path
import sys

import pytest

CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

import cluster_renderer
from cluster_config import (
  CLUSTER_CAMERA_VIEW_MODE_AUTO_CAMERA,
  CLUSTER_CAMERA_VIEW_MODE_DEFAULT,
  CLUSTER_CAMERA_VIEW_MODE_EGO_BOTTOM,
  CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
  CLUSTER_CAMERA_VIEW_MODE_WIDE_CAMERA,
  CLUSTER_SCREEN_MODE_DEBUG,
  CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
  CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
  CLUSTER_SCREEN_MODE_DEFAULT,
  CLUSTER_SCREEN_MODE_FULLSCREEN_3D,
  CLUSTER_SCREEN_MODE_TRIP_REPORT,
)
from cluster_live import standby_state
from cluster_models import LaneMarking
from cluster_renderer import ClusterUiRenderer

START = 100.0
# Blink period is 1.0s with 0.5s on: these sample both phases several times.
PHASES = (0.0, 0.25, 0.49, 0.5, 0.75, 0.99, 1.0, 1.3, 1.6, 2.2, 2.7)


def lane_change_state(side="left", signal=True, **overrides):
  offset = -1.0 if side == "left" else 1.0
  state = replace(
    standby_state(),
    lane_change=side,
    lane_change_phase="changing",
    highlight_lane=side,
    highlight_lane_offset=offset,
    left_signal=signal and side == "left",
    right_signal=signal and side == "right",
  )
  return replace(state, **overrides)


def frames(renderer, monkeypatch, state, phases=PHASES):
  """Run the renderer's own per-frame signal/highlight/scene path at each time phase."""
  clock = {"now": START}
  monkeypatch.setattr(cluster_renderer.time, "perf_counter", lambda: clock["now"])
  theme = renderer._current_theme()
  out = []
  for phase in phases:
    clock["now"] = START + phase
    signal_lights = renderer._turn_signal_lights(state)
    lit = renderer._highlight_lane_lit(state, signal_lights)
    scene = renderer._scene_for_state(state, lit, theme)
    out.append((phase, signal_lights, lit, scene))
  return out


@pytest.mark.parametrize("side", ("left", "right"))
def test_default_hud_lane_highlight_stays_drawn_while_blinker_arrow_blinks(monkeypatch, side):
  renderer = ClusterUiRenderer()
  results = frames(renderer, monkeypatch, lane_change_state(side))

  assert all(lit for _, _, lit, _ in results)
  assert all(len(scene.highlight_lanes) == 1 for *_, scene in results)
  index = 0 if side == "left" else 1
  arrow = [lights[index] for _, lights, _, _ in results]
  other = [lights[1 - index] for _, lights, _, _ in results]
  assert True in arrow and False in arrow
  assert arrow == [(p % 1.0) < 0.5 for p in PHASES]
  assert not any(other)


def test_default_hud_highlight_side_follows_lane_change_side(monkeypatch):
  renderer = ClusterUiRenderer()
  left = frames(renderer, monkeypatch, lane_change_state("left"), (0.75,))[0][3]
  right = frames(renderer, monkeypatch, lane_change_state("right"), (0.75,))[0][3]
  left_x = sum(v.x for v in left.highlight_lanes[0].left) / len(left.highlight_lanes[0].left)
  right_x = sum(v.x for v in right.highlight_lanes[0].left) / len(right.highlight_lanes[0].left)
  assert left_x < right_x


def test_default_hud_lane_highlight_steady_without_signal(monkeypatch):
  renderer = ClusterUiRenderer()
  results = frames(renderer, monkeypatch, lane_change_state("left", signal=False))

  assert all(lit for _, _, lit, _ in results)
  assert all(len(scene.highlight_lanes) == 1 for *_, scene in results)
  assert all(lights == (False, False) for _, lights, _, _ in results)


def test_default_hud_hazard_arrows_blink_but_lane_highlight_steady(monkeypatch):
  renderer = ClusterUiRenderer()
  state = lane_change_state("left", left_signal=True, right_signal=True)
  results = frames(renderer, monkeypatch, state)

  assert all(lit for _, _, lit, _ in results)
  assert all(len(scene.highlight_lanes) == 1 for *_, scene in results)
  assert [lights for _, lights, _, _ in results] == [((p % 1.0) < 0.5,) * 2 for p in PHASES]


def test_default_hud_hides_highlight_when_lane_change_source_disappears(monkeypatch):
  renderer = ClusterUiRenderer()
  frames(renderer, monkeypatch, lane_change_state("left"), (0.0, 0.75))
  gone = replace(
    lane_change_state("left"),
    lane_change=None,
    lane_change_phase="idle",
    highlight_lane=None,
    highlight_lane_offset=None,
  )
  results = frames(renderer, monkeypatch, gone, (1.0, 1.25, 1.75))

  assert all(scene.highlight_lanes == () for *_, scene in results)
  assert renderer._lane_highlight_side is None
  assert renderer._lane_highlight_started_at is None
  # Offset alone vanishing (side still named) also hides it.
  no_offset = replace(lane_change_state("left"), highlight_lane_offset=None)
  assert all(scene.highlight_lanes == () for *_, scene in frames(renderer, monkeypatch, no_offset, (2.0, 2.75)))


def test_default_hud_does_not_force_undetected_lane_lines(monkeypatch):
  renderer = ClusterUiRenderer()
  hidden = replace(
    lane_change_state("left"),
    lanes=(LaneMarking(-0.5, visible=False, width=7), LaneMarking(0.5, visible=False, width=7)),
  )
  results = frames(renderer, monkeypatch, hidden, (0.0, 0.75))
  assert all(scene.lane_markings == () for *_, scene in results)

  # Lane-line meshes are identical whatever the highlight lit value is.
  visible = lane_change_state("left")
  theme = renderer._current_theme()
  on = cluster_renderer.build_cluster_scene(visible, highlight_lane_lit=True, theme=theme)
  off = cluster_renderer.build_cluster_scene(visible, highlight_lane_lit=False, theme=theme)
  assert on.lane_markings == off.lane_markings
  assert on.vehicles == off.vehicles


@pytest.mark.parametrize(
  ("screen_mode", "camera_view_mode"),
  (
    (CLUSTER_SCREEN_MODE_DEFAULT, CLUSTER_CAMERA_VIEW_MODE_EGO_BOTTOM),
    (CLUSTER_SCREEN_MODE_DEFAULT, CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA),
    (CLUSTER_SCREEN_MODE_DEFAULT, CLUSTER_CAMERA_VIEW_MODE_WIDE_CAMERA),
    (CLUSTER_SCREEN_MODE_DEFAULT, CLUSTER_CAMERA_VIEW_MODE_AUTO_CAMERA),
    (CLUSTER_SCREEN_MODE_FULLSCREEN_3D, CLUSTER_CAMERA_VIEW_MODE_DEFAULT),
    (CLUSTER_SCREEN_MODE_DEBUG, CLUSTER_CAMERA_VIEW_MODE_DEFAULT),
    (CLUSTER_SCREEN_MODE_DEBUG_SYSTEM, CLUSTER_CAMERA_VIEW_MODE_DEFAULT),
    (CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT, CLUSTER_CAMERA_VIEW_MODE_DEFAULT),
    (CLUSTER_SCREEN_MODE_TRIP_REPORT, CLUSTER_CAMERA_VIEW_MODE_DEFAULT),
  ),
)
@pytest.mark.parametrize("signal", (True, False))
def test_other_modes_keep_lane_highlight_blinking(monkeypatch, screen_mode, camera_view_mode, signal):
  renderer = ClusterUiRenderer(screen_mode=screen_mode)
  state = lane_change_state("left", signal=signal, camera_view_mode=camera_view_mode)
  results = frames(renderer, monkeypatch, state)

  expected = [(p % 1.0) < 0.5 for p in PHASES]
  assert [lit for _, _, lit, _ in results] == expected
  assert [len(scene.highlight_lanes) for *_, scene in results] == [int(e) for e in expected]
  if signal:
    assert [lights[0] for _, lights, _, _ in results] == expected


def test_switching_from_default_hud_resumes_signal_synchronised_blink(monkeypatch):
  renderer = ClusterUiRenderer()
  state = lane_change_state("left")
  frames(renderer, monkeypatch, state, (0.0, 0.3))
  renderer.screen_mode = CLUSTER_SCREEN_MODE_DEBUG
  results = frames(renderer, monkeypatch, state, (0.6, 1.2))
  assert [lit for _, _, lit, _ in results] == [lights[0] for _, lights, _, _ in results] == [False, True]
