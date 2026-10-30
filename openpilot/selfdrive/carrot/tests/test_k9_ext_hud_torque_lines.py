"""External HUD torque lines (보노보노 2026-10-30 18:10 "마찰학습값이 0.11이면 웹에서설정하는것처럼 110", 18:24 "헌재 학습값
띄우게하고 100%안찻으면 아래에 퍼센트 띄워", 18:27 "학습값과 적용되고잇는값 위아래로 띄워"). Display only.
  '학습 횡가속 2745  마찰 105'  liveTorqueParameters Filtered, whatever LateralTorqueCustom is
  '적용 횡가속 2500  마찰 130'  applied_torque (what latcontrol_torque uses)
  '토크학습 61%'                only while calPerc < 100 ('-' unknown), its slot stays empty at 100
Numbers are the web menu units of LateralTorqueAccelFactor / LateralTorqueFriction: x 1000, round half up.
GL checks need xvfb-run."""
import os
import sys
from dataclasses import fields, replace
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

from cluster_lat_values import (
  applied_torque, learned_torque_filtered, torque_applied_text, torque_learn_text, torque_learned_text,
)


def _lt(perc, af_f=2.745, fr_f=0.1046, af_raw=3.163, fr_raw=0.087):
  return NS(calPerc=perc, latAccelFactorRaw=af_raw, frictionCoefficientRaw=fr_raw,
            latAccelFactorFiltered=af_f, frictionCoefficientFiltered=fr_f)


def test_learned_line_is_filtered_x1000_whatever_custom():
  assert learned_torque_filtered(_lt(61)) == (pytest.approx(2.745), pytest.approx(0.1046))
  assert torque_learned_text(*learned_torque_filtered(_lt(61))) == "학습 횡가속 2745  마찰 105"
  assert torque_learned_text(*learned_torque_filtered(_lt(100))) == "학습 횡가속 2745  마찰 105"
  # LateralTorqueCustom 1: the applied line shows the menu values, the learned line still the Filtered ones (never Raw 3163 / 87)
  _, af, fr = applied_torque(_lt(61), 1, 2500, 130)
  assert torque_applied_text(af, fr) == "적용 횡가속 2500  마찰 130"
  assert torque_learned_text(*learned_torque_filtered(_lt(61))) == "학습 횡가속 2745  마찰 105"


def test_applied_line_is_applied_torque_x1000():
  assert torque_applied_text(*applied_torque(_lt(61), 0)[1:]) == "적용 횡가속 2745  마찰 105"      # custom 0 -> Filtered
  assert torque_applied_text(*applied_torque(_lt(61), 1, 2500, 131)[1:]) == "적용 횡가속 2500  마찰 131"
  assert torque_applied_text(*applied_torque(_lt(100), 1, 3300, 95)[1:]) == "적용 횡가속 3300  마찰 95"
  assert torque_applied_text(*applied_torque(_lt(61), None, 2500, 131)[1:]) == "적용 횡가속 -  마찰 -"   # menu not read yet


def test_web_units_round_half_up():
  # x 1000, then round half up (not Python's round-half-even): 0.1105 -> 110.5 -> 111
  assert torque_learned_text(2.4495, 0.1105) == "학습 횡가속 2450  마찰 111"
  assert torque_learned_text(2.4494, 0.1104) == "학습 횡가속 2449  마찰 110"
  assert torque_applied_text(2.5, 0.0005) == "적용 횡가속 2500  마찰 1"
  assert torque_applied_text(2.0, 0.0) == "적용 횡가속 2000  마찰 0"


def test_unknown_values_dash():
  assert torque_learned_text(None, None) == "학습 횡가속 -  마찰 -"
  assert torque_applied_text(float("nan"), float("inf")) == "적용 횡가속 -  마찰 -"
  assert learned_torque_filtered(None) == (None, None)
  assert learned_torque_filtered(NS(calPerc=50)) == (None, None)                       # broken message
  assert learned_torque_filtered(_lt(61, float("nan"), -0.1)) == (None, None)          # non-finite / impossible
  assert learned_torque_filtered(_lt(61, 0.0, 0.0)) == (None, pytest.approx(0.0))


def test_percent_line_only_while_learning():
  assert torque_learn_text(99) == "토크학습 99%"
  assert torque_learn_text(0) == "토크학습 0%"
  assert torque_learn_text(100) is None
  assert torque_learn_text(None) == "토크학습 -"


def test_new_state_fields_default_unknown():
  from cluster_models import ClusterUiState
  defaults = {f.name: f.default for f in fields(ClusterUiState)}
  assert defaults["torque_filtered_lat_accel_factor"] is None and defaults["torque_filtered_friction"] is None


def test_live_source_learned_reads_filtered():
  from cluster_live import OpenpilotLiveSource
  src = object.__new__(OpenpilotLiveSource)
  sm = {"liveTorqueParameters": _lt(61)}
  src._service_data = lambda n: sm.get(n)
  src._service_alive = lambda n: sm.get(n) is not None
  assert src._live_torque_learned() == (pytest.approx(2.745), pytest.approx(0.1046))
  sm.clear()
  assert src._live_torque_learned() == (None, None)


def _lat_state(perc=61, unknown=False):
  if unknown:
    return NS(applied_steer_ratio=None, applied_steer_delay_s=None, delay_learn_perc=None, delay_learn_s=None,
              torque_learn_perc=None, torque_learn_lat_accel_factor=None, torque_learn_friction=None,
              torque_filtered_lat_accel_factor=None, torque_filtered_friction=None)
  return NS(applied_steer_ratio=12.9, applied_steer_delay_s=0.2, delay_learn_perc=63, delay_learn_s=0.24,
            torque_learn_perc=perc, torque_learn_lat_accel_factor=2.5, torque_learn_friction=0.13,
            torque_filtered_lat_accel_factor=2.745, torque_filtered_friction=0.1046)


def test_renderer_fixed_slots():
  import cluster_renderer as cr
  calls = []
  r = object.__new__(cr.ClusterUiRenderer)
  r._draw_text_with_stroke = lambda text, x, y, *a, **k: calls.append((text, y))
  cr.ClusterUiRenderer._draw_lat_values(r, _lat_state(61))
  assert calls == list(zip(("조향비 12.90  지연설정 0.20초", "지연학습 63%  0.24초", "학습 횡가속 2745  마찰 105",
                            "적용 횡가속 2500  마찰 130", "토크학습 61%"), cr.LAT_VALUES_SLOT_Y, strict=True))
  first = calls[:4]
  calls.clear()
  cr.ClusterUiRenderer._draw_lat_values(r, _lat_state(100))
  assert calls == first                                   # % line gone at 100 %, the other four stay in their slots
  calls.clear()
  cr.ClusterUiRenderer._draw_lat_values(r, _lat_state(unknown=True))
  assert [t for t, _ in calls][2:] == ["학습 횡가속 -  마찰 -", "적용 횡가속 -  마찰 -", "토크학습 -"]
  ys = list(cr.LAT_VALUES_SLOT_Y)
  assert ys == sorted(ys) and ys[-1] < cr.TPMS_STATUS_CAR_CENTER_Y - cr.TPMS_STATUS_ICON_H * 0.5
  assert cr.LAT_VALUES_SLOT_Y[:3] == cr.LAT_VALUES_LINE_Y and cr.LAT_VALUES_SLOT_Y[3:] == cr.TORQUE_LINE_Y


# ---------------------------------------------------------------- GL (xvfb-run)
gl = pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")


def _state(**kw):
  from cluster_models import DetectedVehicle, SimulatorInput
  from cluster_simulator import ClusterSimulator
  base = ClusterSimulator().update(SimulatorInput(), 0.05)
  return replace(base, speed_kph=62.0, cruise_kph=80, speed_limit_kph=80, onroad=True, gear_text="D",
                 left_lane_line_confidence=1.0, right_lane_line_confidence=1.0,
                 detected_vehicles=(DetectedVehicle("L1", 32.0, 0.0, source="radarState", primary=True,
                                                    absolute_speed_kph=58.0),), **kw)


def _rgb(png):
  import io
  from PIL import Image
  return Image.open(io.BytesIO(png)).convert("RGB")


@gl
def test_torque_lines_fixed_slots_no_overlap():
  from PIL import ImageChops
  from cluster_config import CLUSTER_PANEL_LAYOUT_DRIVING_LEFT, CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT
  import cluster_renderer as cr
  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  try:
    for layout in (CLUSTER_PANEL_LAYOUT_DRIVING_LEFT, CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT):
      r.set_panel_layout(layout)
      sx, sy = r.width / cr.DESIGN_WIDTH, r.height / cr.DESIGN_HEIGHT
      shift = 0.0 if layout == CLUSTER_PANEL_LAYOUT_DRIVING_LEFT else cr.NAVI_LIVE_PANEL_W
      drive = (int(shift * sx), 0, int((shift + cr.NAVI_LIVE_PANEL_X) * sx), r.height)
      shots = {}
      for name, kw in (("61", vars(_lat_state(61))), ("100", vars(_lat_state(100))), ("none", vars(_lat_state(unknown=True)))):
        s = _state(**kw)
        r.render_to_png_bytes(s)
        with_line = _rgb(r.render_to_png_bytes(s))
        orig = r._draw_lat_values
        r._draw_lat_values = lambda _s: None
        try:
          r.render_to_png_bytes(s)
          without = _rgb(r.render_to_png_bytes(s))
        finally:
          r._draw_lat_values = orig
        shots[name] = with_line
        diff = ImageChops.difference(with_line, without)
        half = cr.HUD_LINE_STEP * 0.5
        prev_bottom = None
        for i, y in enumerate(cr.LAT_VALUES_SLOT_Y):
          band = (drive[0], int(round((y - half) * sy)), drive[2], int(round((y + half) * sy)))
          b = diff.crop(band).getbbox()
          if name == "100" and i == 4:
            assert b is None, (layout, name, i, b)          # no % line at 100 %
            continue
          assert b is not None, (layout, name, i)
          box = (b[0] + band[0], b[1] + band[1], b[2] + band[0], b[3] + band[1])
          # at least one empty row between neighbouring lines (a line running into the next slot would close it)
          assert prev_bottom is None or box[1] >= prev_bottom + 1, (layout, name, i, box, prev_bottom)
          prev_bottom = box[3]
          assert box[3] <= (cr.TPMS_STATUS_CAR_CENTER_Y - cr.TPMS_STATUS_ICON_H * 0.5) * sy   # above the TPMS car
          # nothing else drawn under the line (lane-probability badge, lane lines, car, other text)
          under = without.crop(box).convert("L")
          assert sum(under.histogram()[91:]) <= 0.01 * under.width * under.height, (layout, name, i, box)
      # fixed slots: lines 1..4 are the same pixels with (61 %) and without (100 %) the % line
      top = int(round((cr.LAT_VALUES_SLOT_Y[0] - cr.HUD_LINE_STEP * 0.5) * sy))
      bottom = int(round((cr.LAT_VALUES_SLOT_Y[3] + cr.HUD_LINE_STEP * 0.5) * sy))
      region = (drive[0], top, drive[2], bottom)
      assert ImageChops.difference(shots["61"].crop(region), shots["100"].crop(region)).getbbox() is None
  finally:
    r.close()
