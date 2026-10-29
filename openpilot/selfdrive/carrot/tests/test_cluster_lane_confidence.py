from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import math
import sys

import pytest

CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

import cluster_live
import cluster_renderer
from cluster_config import (
  CLUSTER_CAMERA_VIEW_MODE_AUTO_CAMERA,
  CLUSTER_CAMERA_VIEW_MODE_DEFAULT,
  CLUSTER_CAMERA_VIEW_MODE_EGO_BOTTOM,
  CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
  CLUSTER_CAMERA_VIEW_MODE_WIDE_CAMERA,
  CLUSTER_PANEL_LAYOUT_DRIVING_LEFT,
  CLUSTER_SCREEN_MODE_DEBUG,
  CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
  CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
  CLUSTER_SCREEN_MODE_DEFAULT,
  CLUSTER_SCREEN_MODE_FULLSCREEN_3D,
  CLUSTER_SCREEN_MODE_TRIP_REPORT,
  DESIGN_HEIGHT,
  DESIGN_WIDTH,
)
from cluster_live import LANE_LINE_CONFIDENCE_MAX_AGE_S, OpenpilotLiveSource, standby_state
from cluster_renderer import (
  LANE_CONFIDENCE_BOX_H,
  LANE_CONFIDENCE_BOX_PAD_X,
  LANE_CONFIDENCE_CENTER_Y,
  LANE_CONFIDENCE_EGO_CAR_GAP_X,
  LANE_CONFIDENCE_EGO_CAR_HALF_W,
  LANE_CONFIDENCE_FONT_SIZE,
  LANE_TURN_SIGNAL_CENTER_Y,
  LANE_TURN_SIGNAL_LEFT_CENTER_X,
  LANE_TURN_SIGNAL_RIGHT_CENTER_X,
  NAVI_WORLD_VIEW_SHIFT_X,
  CRUISE_OVERRIDE_LABEL_CENTER_Y,
  CRUISE_OVERRIDE_SPEED_CENTER_X,
  CRUISE_OVERRIDE_SPEED_CENTER_Y,
  CRUISE_OVERRIDE_SPEED_FONT_SIZE,
  SPEED_GEAR_CENTER_X,
  SPEED_GEAR_CENTER_Y,
  SPEED_GEAR_H,
  SPEED_GEAR_W,
  SPEED_PANEL_H,
  SPEED_PANEL_W,
  SPEED_PANEL_X,
  SPEED_PANEL_Y,
  TPMS_STATUS_CENTER_X,
  TPMS_STATUS_ICON_H,
  TPMS_STATUS_ICON_W,
  TPMS_STATUS_VALUE_CENTER_Y,
  TURN_SIGNAL_HEAD_HALF_HEIGHT,
  ClusterUiRenderer,
)

NOW = 100.0


def _live_source(lane_probs, *, receive_t=NOW - 0.05, valid=True, model_present=True):
  source = object.__new__(OpenpilotLiveSource)
  source._max_lateral_accel = 3.0
  source._energy_gauge_label = "fuel"
  source._carrot_navi_media = None
  source._current_carrot_navi = lambda _now: None
  model = SimpleNamespace(laneLineProbs=lane_probs) if model_present else None
  source._service_data = lambda service: model if service == "modelV2" else None
  source._service_alive = lambda _service: False
  source.sm = SimpleNamespace(
    recv_time={} if receive_t is None else {"modelV2": receive_t},
    valid={"modelV2": valid},
  )
  return source


# --- live data path -------------------------------------------------------

def test_inner_lane_probabilities_are_reported_independently_per_side():
  # Schema: [outer left, inner left, inner right, outer right].
  source = _live_source([0.99, 0.83, 0.12, 0.98])
  assert source._live_lane_line_confidence(NOW) == pytest.approx((0.83, 0.12))
  source = _live_source([0.0, 0.12, 0.83, 0.0])
  assert source._live_lane_line_confidence(NOW) == pytest.approx((0.12, 0.83))


def test_valid_zero_is_preserved_and_not_unknown():
  left, right = _live_source([0.5, 0.0, 1.0, 0.5])._live_lane_line_confidence(NOW)
  assert left == 0.0 and left is not None
  assert right == 1.0


@pytest.mark.parametrize(
  ("lane_probs", "expected"),
  (
    ([0.1, math.nan, 0.7, 0.1], (None, 0.7)),
    ([0.1, 0.6, math.inf, 0.1], (0.6, None)),
    ([0.1, -math.inf, math.nan, 0.1], (None, None)),
    ([0.1, 1.2, 0.4, 0.1], (None, 0.4)),
    ([0.1, 0.4, -0.01, 0.1], (0.4, None)),
    ([0.1, "bad", 0.4, 0.1], (None, 0.4)),
    ([0.1, None, 0.4, 0.1], (None, 0.4)),
  ),
)
def test_nonfinite_or_out_of_range_side_is_unknown_without_affecting_other_side(lane_probs, expected):
  assert _live_source(lane_probs)._live_lane_line_confidence(NOW) == pytest.approx(expected)


@pytest.mark.parametrize("lane_probs", (None, [], [0.9], [0.9, 0.8]))
def test_absent_or_short_lane_probabilities_are_unknown(lane_probs):
  assert _live_source(lane_probs)._live_lane_line_confidence(NOW) == (None, None)


def test_missing_model_message_is_unknown():
  assert _live_source(None, model_present=False)._live_lane_line_confidence(NOW) == (None, None)
  assert _live_source([0.9, 0.8, 0.7, 0.6], receive_t=None)._live_lane_line_confidence(NOW) == (None, None)


def test_invalid_model_message_is_unknown():
  assert _live_source([0.9, 0.8, 0.7, 0.6], valid=False)._live_lane_line_confidence(NOW) == (None, None)


def test_stale_model_message_is_unknown_not_last_percentage():
  source = _live_source([0.9, 0.8, 0.7, 0.6], receive_t=NOW)
  assert source._live_lane_line_confidence(NOW) == pytest.approx((0.8, 0.7))
  assert source._live_lane_line_confidence(NOW + LANE_LINE_CONFIDENCE_MAX_AGE_S) == pytest.approx((0.8, 0.7))
  # Same retained SubMaster data, no newer receipt: must not keep old values.
  assert source._live_lane_line_confidence(NOW + LANE_LINE_CONFIDENCE_MAX_AGE_S + 0.01) == (None, None)
  # Receipt time in the future (clock inconsistency) is not trusted either.
  assert source._live_lane_line_confidence(NOW - 1.0) == (None, None)


def test_live_hud_state_carries_confidence_and_drops_it_when_stale(monkeypatch):
  source = _live_source([0.2, 0.77, 0.0, 0.3], receive_t=NOW - 0.1)
  monkeypatch.setattr(cluster_live.time, "monotonic", lambda: NOW)
  state = source._with_live_hud_state(standby_state())
  assert state.left_lane_line_confidence == pytest.approx(0.77)
  assert state.right_lane_line_confidence == 0.0

  monkeypatch.setattr(cluster_live.time, "monotonic", lambda: NOW + 2.0)
  state = source._with_live_hud_state(state)
  assert state.left_lane_line_confidence is None
  assert state.right_lane_line_confidence is None


def test_standby_state_has_unknown_confidence():
  state = standby_state()
  assert state.left_lane_line_confidence is None
  assert state.right_lane_line_confidence is None


# --- renderer -------------------------------------------------------------

@pytest.mark.parametrize(
  ("confidence", "text"),
  (
    (0.0, "0%"),
    (1.0, "100%"),
    (0.834, "83%"),
    (0.006, "1%"),
    (None, "--%"),
    (math.nan, "--%"),
    (math.inf, "--%"),
    (1.5, "--%"),
    (-0.2, "--%"),
  ),
)
def test_confidence_text(confidence, text):
  assert ClusterUiRenderer._lane_confidence_text(confidence) == text


def _renderer(monkeypatch, screen_mode=CLUSTER_SCREEN_MODE_DEFAULT):
  renderer = object.__new__(ClusterUiRenderer)
  renderer.width = 1920
  renderer.height = 480
  renderer.screen_mode = screen_mode
  renderer.panel_layout = CLUSTER_PANEL_LAYOUT_DRIVING_LEFT
  draws = {"signals": [], "texts": [], "boxes": []}
  monkeypatch.setattr(cluster_renderer.rl, "rl_push_matrix", lambda: None)
  monkeypatch.setattr(cluster_renderer.rl, "rl_pop_matrix", lambda: None)
  monkeypatch.setattr(cluster_renderer.rl, "rl_translatef", lambda *_args: None)
  monkeypatch.setattr(
    cluster_renderer.rl, "draw_rectangle_rounded",
    lambda rect, *_args: draws["boxes"].append((rect.x, rect.y, rect.width, rect.height)),
  )
  monkeypatch.setattr(renderer, "_font_for_text", lambda _text: None)
  # Deterministic stand-in for the font measurement: width depends on the text.
  monkeypatch.setattr(renderer, "_measure_text", lambda text, size, *_args, **_kwargs: (len(text) * size * 0.55, size))
  monkeypatch.setattr(renderer, "_profile_start", lambda: 0.0)
  monkeypatch.setattr(renderer, "_profile_add", lambda *_args: None)
  monkeypatch.setattr(renderer, "_draw_speed_block", lambda _state, **_kwargs: None)
  monkeypatch.setattr(renderer, "_draw_accel_block", lambda _state: None)
  monkeypatch.setattr(renderer, "_draw_steering_output_block", lambda _state: None)
  monkeypatch.setattr(renderer, "_draw_drive_status", lambda _state: None)
  monkeypatch.setattr(renderer, "_draw_center_clock", lambda _state, **_kwargs: None)
  monkeypatch.setattr(
    renderer,
    "_draw_turn_signal",
    lambda side, lit, show_inactive=False, center_x_offset=0.0: draws["signals"].append(
      (side, lit, show_inactive, center_x_offset)
    ),
  )
  monkeypatch.setattr(
    renderer,
    "_draw_text_with_stroke",
    lambda text, x, y, size, *_args, anchor="left", **_kwargs: draws["texts"].append((text, x, y, size, anchor)),
  )
  return renderer, draws


# Measured ego-car sprite in design px (out/ego_base.json, 2026-10-29, 11 render cases): 133 x 134,
# centred in the 3D viewport - x 960 unshifted, 562 once a panel shifts the driving view.
EGO_CAR_H = 134.0


def _expected_label_x(side, text, ego_car_center_x=DESIGN_WIDTH * 0.5):
  """Where a label's centre must land: against that side of the car, outside its own box."""
  # Same width stand-in the _renderer patch gives _measure_text.
  text_width = len(text) * LANE_CONFIDENCE_FONT_SIZE * 0.55
  reach = (
    LANE_CONFIDENCE_EGO_CAR_HALF_W + LANE_CONFIDENCE_EGO_CAR_GAP_X
    + LANE_CONFIDENCE_BOX_PAD_X + text_width * 0.5
  )
  return ego_car_center_x + (-reach if side == "left" else reach)


def _hud_state(**overrides):
  fields = {
    "gear_text": "D",
    "external_nav_active": True,
    "camera_view_mode": CLUSTER_CAMERA_VIEW_MODE_DEFAULT,
    "left_lane_line_confidence": 0.91,
    "right_lane_line_confidence": None,
  }
  fields.update(overrides)
  return replace(standby_state(), **fields)


@pytest.mark.parametrize(
  ("left_signal", "right_signal", "left_lit", "right_lit"),
  (
    (False, False, False, False),  # no blinker: labels still shown
    (True, False, True, False),
    (False, True, False, True),
    (True, True, True, True),  # hazard
    (True, True, False, False),  # hazard off-phase
  ),
)
def test_labels_shown_regardless_of_blinker_and_blinkers_unchanged(
  monkeypatch, left_signal, right_signal, left_lit, right_lit,
):
  renderer, draws = _renderer(monkeypatch)
  state = _hud_state(left_signal=left_signal, right_signal=right_signal)
  renderer._draw_driving_hud_content(state, CLUSTER_SCREEN_MODE_DEFAULT, left_lit, right_lit)

  # Turn arrows receive exactly the lit state and offsets they did before.
  assert [(side, lit) for side, lit, _show, _offset in draws["signals"]] == [("left", left_lit), ("right", right_lit)]
  left_offset = renderer._turn_signal_center_x_offset(state, "left")
  right_offset = renderer._turn_signal_center_x_offset(state, "right")
  assert [offset for *_rest, offset in draws["signals"]] == [left_offset, right_offset]

  # 보노보노 2026-10-29 18:12: each label hugs its own side of the centre ego car, at the car's height.
  assert draws["texts"] == [
    ("91%", _expected_label_x("left", "91%"), LANE_CONFIDENCE_CENTER_Y, LANE_CONFIDENCE_FONT_SIZE, "center"),
    ("--%", _expected_label_x("right", "--%"), LANE_CONFIDENCE_CENTER_Y, LANE_CONFIDENCE_FONT_SIZE, "center"),
  ]


def test_left_and_right_values_are_independent_in_display(monkeypatch):
  renderer, draws = _renderer(monkeypatch)
  state = _hud_state(left_lane_line_confidence=0.0, right_lane_line_confidence=0.64)
  renderer._draw_driving_hud_content(state, CLUSTER_SCREEN_MODE_DEFAULT, False, False)
  assert [text for text, *_rest in draws["texts"]] == ["0%", "64%"]
  left_x = draws["texts"][0][1]
  right_x = draws["texts"][1][1]
  assert left_x < right_x


def test_labels_follow_navigation_panel_shift_with_the_car(monkeypatch):
  renderer, draws = _renderer(monkeypatch)
  state = _hud_state(external_nav_active=False, navi_dashboard=None)
  renderer._draw_driving_hud_content(state, CLUSTER_SCREEN_MODE_TRIP_REPORT, False, False)
  # Automatic trip-report panel shifts the world view; the car moves with it and the labels stay on the car.
  left_offset = renderer._turn_signal_center_x_offset(state, "left")
  assert left_offset == -NAVI_WORLD_VIEW_SHIFT_X
  ego_car_center_x = DESIGN_WIDTH * 0.5 - NAVI_WORLD_VIEW_SHIFT_X
  assert draws["texts"][0][1] == _expected_label_x("left", "91%", ego_car_center_x)
  assert draws["texts"][1][1] == _expected_label_x("right", "--%", ego_car_center_x)


@pytest.mark.parametrize("center_x_offset_case", ("shifted", "unshifted"))
def test_opaque_box_is_sized_from_measured_text_and_centered_under_label(monkeypatch, center_x_offset_case):
  renderer, draws = _renderer(monkeypatch)
  state = _hud_state(
    external_nav_active=center_x_offset_case == "unshifted", left_lane_line_confidence=1.0, right_lane_line_confidence=0.05,
  )
  renderer._draw_driving_hud_content(state, CLUSTER_SCREEN_MODE_DEFAULT, False, False)
  assert len(draws["boxes"]) == 2
  for (text, x, y, size, _anchor), (bx, by, bw, bh) in zip(draws["texts"], draws["boxes"], strict=True):
    measured_w = len(text) * size * 0.55
    assert bw == pytest.approx(measured_w + LANE_CONFIDENCE_BOX_PAD_X * 2.0)
    assert bh == LANE_CONFIDENCE_BOX_H
    assert bx + bw * 0.5 == pytest.approx(x)
    assert by + bh * 0.5 == pytest.approx(y)
  # Wider "100%" gets a wider box than "5%": width follows the text, not a fixed slot.
  assert draws["boxes"][0][2] > draws["boxes"][1][2]


@pytest.mark.parametrize(
  "camera_view_mode",
  (
    CLUSTER_CAMERA_VIEW_MODE_EGO_BOTTOM,
    CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
    CLUSTER_CAMERA_VIEW_MODE_WIDE_CAMERA,
    CLUSTER_CAMERA_VIEW_MODE_AUTO_CAMERA,
  ),
)
def test_nondefault_camera_view_modes_do_not_show_labels(monkeypatch, camera_view_mode):
  renderer, draws = _renderer(monkeypatch)
  state = _hud_state(camera_view_mode=camera_view_mode, left_signal=True)
  renderer._draw_driving_hud_content(state, CLUSTER_SCREEN_MODE_DEFAULT, True, False)
  assert draws["texts"] == []
  assert [(side, lit) for side, lit, *_rest in draws["signals"]] == [("left", True), ("right", False)]


@pytest.mark.parametrize(
  "screen_mode",
  (
    CLUSTER_SCREEN_MODE_FULLSCREEN_3D,
    CLUSTER_SCREEN_MODE_DEBUG,
    CLUSTER_SCREEN_MODE_DEBUG_SYSTEM,
    CLUSTER_SCREEN_MODE_DEBUG_GRAPH_RIGHT,
    CLUSTER_SCREEN_MODE_TRIP_REPORT,
  ),
)
def test_nondefault_requested_screen_modes_show_labels_too(monkeypatch, screen_mode):
  # 보노보노 2026-10-26 17:42 "외부hud 차선 % 안보이는데?" (car: ClusterHudScreenMode 2): labels in every screen mode
  # (default camera view), each on its own arrow; arrows unchanged; lane-line hold / steady highlight stay default-mode only.
  renderer, draws = _renderer(monkeypatch, screen_mode=screen_mode)
  state = _hud_state()
  renderer._draw_driving_hud_content(state, screen_mode, False, True)
  # Fullscreen 3D leaves the driving view unshifted; the debug / trip panels shift it, car and labels together.
  ego_car_center_x = DESIGN_WIDTH * 0.5 + (
    0.0 if screen_mode == CLUSTER_SCREEN_MODE_FULLSCREEN_3D else -NAVI_WORLD_VIEW_SHIFT_X
  )
  assert draws["texts"] == [
    ("91%", _expected_label_x("left", "91%", ego_car_center_x), LANE_CONFIDENCE_CENTER_Y,
     LANE_CONFIDENCE_FONT_SIZE, "center"),
    ("--%", _expected_label_x("right", "--%", ego_car_center_x), LANE_CONFIDENCE_CENTER_Y,
     LANE_CONFIDENCE_FONT_SIZE, "center"),
  ]
  assert [(side, lit) for side, lit, *_rest in draws["signals"]] == [("left", False), ("right", True)]
  assert not renderer._lane_confidence_visible(state)


def _label_rect(side, ego_car_center_x):
  # Centered box sized for the widest text "100%": measured 79.125 px with the real HUD
  # font at this size (Xvfb, 2026-10-21); +3 px margin. Fonts aren't loaded in unit tests.
  x, y = ClusterUiRenderer._lane_confidence_anchor(side, ego_car_center_x, 82.0)
  bx, by, bw, bh = ClusterUiRenderer._lane_confidence_box(x, y, 82.0)
  return bx, by, bx + bw, by + bh


def _arrow_rects(side, center_x_offset):
  """The drawn arrow, not one bounding box: a 48 px tail (half height 16) then the head out to 60 px
  at TURN_SIGNAL_HEAD_HALF_HEIGHT (_draw_turn_signal). One box would also claim the room beside the
  tail, where the % labels now sit."""
  cx = (LANE_TURN_SIGNAL_LEFT_CENTER_X if side == "left" else LANE_TURN_SIGNAL_RIGHT_CENTER_X) + center_x_offset
  direction = -1 if side == "left" else 1
  tail = sorted((cx + direction * -36, cx + direction * 12))
  head = sorted((cx + direction * 12, cx + direction * 60))
  return (
    (tail[0], LANE_TURN_SIGNAL_CENTER_Y - 16, tail[1], LANE_TURN_SIGNAL_CENTER_Y + 16),
    (head[0], LANE_TURN_SIGNAL_CENTER_Y - TURN_SIGNAL_HEAD_HALF_HEIGHT,
     head[1], LANE_TURN_SIGNAL_CENTER_Y + TURN_SIGNAL_HEAD_HALF_HEIGHT),
  )


def _overlaps(a, b):
  return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def _center_rect(cx, cy, w, h):
  return cx - w * 0.5, cy - h * 0.5, cx + w * 0.5, cy + h * 0.5


# Fixed HUD widgets near the arrows (not translated by the world-view shift).
FIXED_WIDGETS = {
  "tpms": _center_rect(TPMS_STATUS_CENTER_X, TPMS_STATUS_VALUE_CENTER_Y, TPMS_STATUS_ICON_W, TPMS_STATUS_ICON_H),
  "speed_panel": (SPEED_PANEL_X, SPEED_PANEL_Y, SPEED_PANEL_X + SPEED_PANEL_W, SPEED_PANEL_Y + SPEED_PANEL_H),
  "gear": _center_rect(SPEED_GEAR_CENTER_X, SPEED_GEAR_CENTER_Y, SPEED_GEAR_W, SPEED_GEAR_H),
  # Override speed "100" plus its source label above it, conservative 3 em width.
  "cruise_override": (
    CRUISE_OVERRIDE_SPEED_CENTER_X - CRUISE_OVERRIDE_SPEED_FONT_SIZE * 1.5,
    CRUISE_OVERRIDE_LABEL_CENTER_Y - 12.0,
    CRUISE_OVERRIDE_SPEED_CENTER_X + CRUISE_OVERRIDE_SPEED_FONT_SIZE * 1.5,
    CRUISE_OVERRIDE_SPEED_CENTER_Y + CRUISE_OVERRIDE_SPEED_FONT_SIZE * 0.5,
  ),
}


@pytest.mark.parametrize(
  ("ego_car_center_x", "center_x_offset"),
  ((DESIGN_WIDTH * 0.5, 0.0), (DESIGN_WIDTH * 0.5 - NAVI_WORLD_VIEW_SHIFT_X, -NAVI_WORLD_VIEW_SHIFT_X)),
)
def test_labels_hug_the_car_without_overlapping_it_arrows_or_fixed_widgets(ego_car_center_x, center_x_offset):
  car = _center_rect(ego_car_center_x, LANE_CONFIDENCE_CENTER_Y, LANE_CONFIDENCE_EGO_CAR_HALF_W * 2.0, EGO_CAR_H)
  rects = {side: _label_rect(side, ego_car_center_x) for side in ("left", "right")}
  for side, label in rects.items():
    assert not _overlaps(label, car)
    # Against the car's own side, the configured gap away, within the car's height.
    inner_edge = label[2] if side == "left" else label[0]
    assert abs(inner_edge - ego_car_center_x) == pytest.approx(
      LANE_CONFIDENCE_EGO_CAR_HALF_W + LANE_CONFIDENCE_EGO_CAR_GAP_X
    )
    assert car[1] <= label[1] and label[3] <= car[3]
    assert label[3] <= DESIGN_HEIGHT
    for name, widget in FIXED_WIDGETS.items():
      # Widgets already stacked over the centre car itself (unshifted full-width driving view) are
      # pre-existing crowding, not something a label hugging the car can avoid.
      if _overlaps(widget, car):
        continue
      assert not _overlaps(label, widget), name
    for arrow_side in ("left", "right"):
      for arrow in _arrow_rects(arrow_side, center_x_offset):
        assert not _overlaps(label, arrow)
  assert not _overlaps(rects["left"], rects["right"])
  assert rects["left"][1] == rects["right"][1] and rects["left"][3] == rects["right"][3]  # same height
