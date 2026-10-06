from pathlib import Path
from types import SimpleNamespace
import sys


CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_config import BLUE, WHITE, YELLOW
from cluster_route_replay import (
  LANE_LINE_PROBABILITY_MIN,
  LANE_LINE_PROBABILITY_OFF,
  RouteLogParser,
  lane_color_from_code,
  lane_style_from_code,
)


def lane_parser() -> RouteLogParser:
  return RouteLogParser(recompute_cutins=False)


def inner_visibility(parser: RouteLogParser, left_prob: float, right_prob: float) -> tuple[bool, bool]:
  parser.left_lane_prob = left_prob
  parser.right_lane_prob = right_prob
  values = parser._lane_values()
  return values["left_visible"], values["right_visible"]


def outer_visibility(parser: RouteLogParser, left_prob: float, right_prob: float) -> tuple[bool, bool]:
  parser.left_lane_y_m = -1.8
  parser.right_lane_y_m = 1.8
  parser.outer_left_lane_y_m = -5.4
  parser.outer_right_lane_y_m = 5.4
  parser.outer_left_lane_prob = left_prob
  parser.outer_right_lane_prob = right_prob
  values = parser._lane_values()
  return values["extra_left_visible"], values["extra_right_visible"]


def test_lane_visibility_thresholds_are_separated():
  assert LANE_LINE_PROBABILITY_OFF < LANE_LINE_PROBABILITY_MIN


def test_lane_line_starts_hidden_until_the_on_threshold():
  parser = lane_parser()
  parser.left_lane_prob = 0.0
  parser.right_lane_prob = 0.0

  assert inner_visibility(parser, 0.35, 0.35) == (False, False)
  assert inner_visibility(parser, 0.45, 0.45) == (True, True)


def test_mid_band_probability_keeps_the_previous_lane_state():
  parser = lane_parser()
  parser.left_lane_prob = 0.0
  parser.right_lane_prob = 0.0
  parser._lane_values()

  assert inner_visibility(parser, 0.35, 0.35) == (False, False)
  assert inner_visibility(parser, 0.45, 0.45) == (True, True)
  assert inner_visibility(parser, 0.35, 0.35) == (True, True)
  assert inner_visibility(parser, 0.2, 0.2) == (False, False)
  assert inner_visibility(parser, 0.35, 0.35) == (False, False)


def test_alternating_probabilities_do_not_flicker_after_the_first_show():
  parser = lane_parser()
  parser.left_lane_prob = 0.0
  parser.right_lane_prob = 0.0
  parser._lane_values()

  assert inner_visibility(parser, 0.42, 0.42) == (True, True)
  for probability in (0.38, 0.42) * 10:
    assert inner_visibility(parser, probability, probability) == (True, True)


def test_each_lane_line_latches_independently():
  parser = lane_parser()
  parser.left_lane_prob = 0.0
  parser.right_lane_prob = 0.0
  parser._lane_values()

  assert inner_visibility(parser, 0.45, 0.35) == (True, False)
  assert inner_visibility(parser, 0.35, 0.45) == (True, True)
  assert inner_visibility(parser, 0.2, 0.35) == (False, True)


def test_outer_lane_lines_use_the_same_hysteresis():
  parser = lane_parser()

  assert outer_visibility(parser, 0.35, 0.35) == (False, False)
  assert outer_visibility(parser, 0.45, 0.45) == (True, True)
  assert outer_visibility(parser, 0.35, 0.35) == (True, True)
  assert outer_visibility(parser, 0.2, 0.2) == (False, False)


def test_road_edge_rejection_still_hides_a_latched_lane_line():
  parser = lane_parser()
  assert outer_visibility(parser, 0.9, 0.9) == (True, True)

  parser.left_road_edge_y_m = -3.6
  parser.right_road_edge_y_m = 3.6
  assert outer_visibility(parser, 0.9, 0.9) == (False, False)


def test_negative_lane_code_hides_the_lane_line_immediately():
  parser = lane_parser()
  parser.left_lane_prob = 0.9
  parser.right_lane_prob = 0.9
  assert inner_visibility(parser, 0.9, 0.9) == (True, True)

  parser._update_lane_styles_from_car_state(SimpleNamespace(leftLaneLine=-1, rightLaneLine=-1))

  values = parser._lane_values()
  assert (values["left_visible"], values["right_visible"]) == (False, False)


def test_colorless_lane_code_keeps_the_probability():
  parser = lane_parser()
  parser.left_lane_prob = 0.9
  parser.right_lane_prob = 0.9
  parser._update_lane_styles_from_car_state(SimpleNamespace(leftLaneLine=0, rightLaneLine=0))

  assert (parser.left_lane_prob, parser.right_lane_prob) == (0.9, 0.9)
  assert parser._lane_values()["left_visible"] is True


def test_new_parser_starts_with_hidden_lane_lines():
  parser = lane_parser()
  parser.left_lane_prob = 0.3
  parser.right_lane_prob = 0.3
  assert inner_visibility(parser, 0.45, 0.45) == (True, True)

  fresh = lane_parser()
  assert fresh.lane_line_visible_state == {
    "left": False,
    "right": False,
    "outer_left": False,
    "outer_right": False,
  }
  assert inner_visibility(fresh, 0.35, 0.35) == (False, False)


def test_colorless_lane_codes_are_drawn_as_unknown_lines():
  parser = lane_parser()
  parser._update_lane_styles_from_car_state(SimpleNamespace(leftLaneLine=0, rightLaneLine=5))

  assert parser.left_lane_style == "solid"
  assert parser.right_lane_style == "solid"
  assert parser.left_lane_color is None
  assert parser.right_lane_color is None

  for code in range(10):
    assert lane_style_from_code(code) == "solid"
    assert lane_color_from_code(code) is None
    assert (lane_color_from_code(code) or BLUE) == BLUE


def test_colored_lane_codes_keep_their_existing_style():
  assert (lane_style_from_code(-1), lane_color_from_code(-1)) == ("solid", None)
  assert (lane_style_from_code(10), lane_color_from_code(10)) == ("dashed", WHITE)
  assert (lane_style_from_code(11), lane_color_from_code(11)) == ("solid", WHITE)
  assert (lane_style_from_code(20), lane_color_from_code(20)) == ("dashed", YELLOW)
  assert (lane_style_from_code(21), lane_color_from_code(21)) == ("solid", YELLOW)
