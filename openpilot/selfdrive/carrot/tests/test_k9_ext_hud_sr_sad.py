"""SR / SAD line above the external HUD TPMS car (보노보노 2026-10-24 23:39).

Unit checks: the shown values follow the same rules as controlsd / modeld (CustomSR fixed vs
learned liveParameters x SteerRatioRate, SteerActuatorDelay fixed vs liveDelay learned), '-' when
unknown. GL checks (xvfb-run): the line is drawn inside its own box above the TPMS car, never on
the TPMS badges or the right panel, for both panel layouts and with the C1 taxi meter on.
"""
import math
import os
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

from cluster_lat_values import (  # noqa: E402
  applied_steer_delay_s,
  applied_steer_ratio,
  lat_values_text,
  live_parameters_usable,
)
from openpilot.selfdrive.controls.lib.steer_ratio import resolve_vehicle_model_steer_ratio  # noqa: E402


def _lp(sr=14.2, valid=True, stiffness=1.0):
  return SimpleNamespace(valid=valid, sensorValid=True, posenetValid=True, steerRatio=sr,
                         stiffnessFactor=stiffness, angleOffsetDeg=0.1, roll=0.0)


def test_custom_sr_is_fixed_value():
  # 보노보노 car 10/24: CustomSR=129, SteerRatioRate=100
  assert applied_steer_ratio(16.5, True, 15.0, 100, 129) == pytest.approx(12.9)
  assert applied_steer_ratio(None, False, None, 100, 129) == pytest.approx(12.9)


@pytest.mark.parametrize("live_sr, rate, custom", [(14.2, 100, 0), (14.2, 90, 0), (14.2, 0, 0), (14.2, 250, 10), (0.05, 100, 0),
                                                   (16.0, 120, 129), (16.0, 120, 11)])
def test_matches_controlsd_function(live_sr, rate, custom):
  assert applied_steer_ratio(live_sr, True, 15.0, rate, custom) == pytest.approx(
    resolve_vehicle_model_steer_ratio(live_sr, rate, custom))


def test_nominal_used_until_live_parameters_usable():
  assert applied_steer_ratio(None, False, 15.0, 100, 0) == pytest.approx(15.0)
  assert applied_steer_ratio(None, False, None, 100, 0) is None
  assert live_parameters_usable(_lp())
  assert not live_parameters_usable(_lp(valid=False))
  assert not live_parameters_usable(_lp(sr=0.9))
  assert not live_parameters_usable(_lp(stiffness=0.0))
  assert not live_parameters_usable(_lp(sr=float("nan")))
  assert not live_parameters_usable(None)


def test_sad_fixed_or_learned():
  assert applied_steer_delay_s(23, 0.31) == pytest.approx(0.23)
  assert applied_steer_delay_s(0, 0.31) == pytest.approx(0.31)
  assert applied_steer_delay_s(None, 0.31) == pytest.approx(0.31)
  assert applied_steer_delay_s(0, None) is None
  assert applied_steer_delay_s(0, float("nan")) is None


def test_text():
  assert lat_values_text(12.9, 0.23) == "조향비 12.90  지연설정 0.23초"   # 2026-10-26 Korean labels (k9-hud-ld-lt)
  assert lat_values_text(14.7342, 0.2871) == "조향비 14.73  지연설정 0.29초"
  assert lat_values_text(None, None) == "조향비 -  지연설정 -"
  assert lat_values_text(float("nan"), 0.2) == "조향비 -  지연설정 0.20초"


class _Params:
  def __init__(self, values):
    self.values = values
    self.reads = 0

  def get_float(self, key):
    self.reads += 1
    return float(self.values.get(key, 0.0))


def _source(params, lp=None, ld=None):
  from cluster_live import OpenpilotLiveSource
  src = object.__new__(OpenpilotLiveSource)
  src.params = params
  src.parser = SimpleNamespace(steer_ratio=15.5, car_brand="hyundai")
  sm = {"liveParameters": lp, "liveDelay": ld}
  src._service_data = lambda name: sm.get(name)
  src._service_alive = lambda name: sm.get(name) is not None
  return src


def test_live_source_values_and_param_cache():
  p = _Params({"CustomSR": 129, "SteerRatioRate": 100, "SteerActuatorDelay": 23})
  src = _source(p, _lp(16.1), SimpleNamespace(lateralDelay=0.3))
  assert src._live_lat_values(100.0) == (pytest.approx(12.9), pytest.approx(0.23))
  reads = p.reads
  src._live_lat_values(100.5)          # params re-read at most once per second
  assert p.reads == reads
  p.values.update(CustomSR=0, SteerActuatorDelay=0)
  sr, sad = src._live_lat_values(101.1)
  assert sr == pytest.approx(16.1) and sad == pytest.approx(0.3)
  src2 = _source(_Params({"SteerRatioRate": 100}))  # nothing alive yet -> CarParams nominal, SAD unknown
  assert src2._live_lat_values(0.0) == (pytest.approx(15.5), None)
  src3 = _source(None)
  src3.parser = SimpleNamespace(steer_ratio=15.0, car_brand="")
  assert src3._live_lat_values(0.0) == (None, None)


# ---------------------------------------------------------------- GL (xvfb-run)
gl = pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")


def _state(**kw):
  from cluster_models import DetectedVehicle, SimulatorInput
  from cluster_simulator import ClusterSimulator
  base = ClusterSimulator().update(SimulatorInput(), 0.05)
  return replace(base, speed_kph=62.0, cruise_kph=80, speed_limit_kph=80, onroad=True, gear_text="D",
                 detected_vehicles=(DetectedVehicle("L1", 32.0, 0.0, source="radarState", primary=True,
                                                    absolute_speed_kph=58.0),), **kw)


def _rgb(png):
  import io
  from PIL import Image
  return Image.open(io.BytesIO(png)).convert("RGB")


@gl
def test_line_stays_in_its_box_above_tpms():
  from PIL import ImageChops
  from cluster_config import CLUSTER_PANEL_LAYOUT_DRIVING_LEFT, CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT
  import cluster_renderer as cr
  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  try:
    for meter in (False, True):
      r.set_taxi_meter_enabled(meter)
      for layout in (CLUSTER_PANEL_LAYOUT_DRIVING_LEFT, CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT):
        r.set_panel_layout(layout)
        for sr, sad in ((12.9, 0.23), (14.7342, 0.2871), (None, None), (88.88, 8.88)):
          s = _state(applied_steer_ratio=sr, applied_steer_delay_s=sad)
          r.render_to_png_bytes(s)
          with_line = _rgb(r.render_to_png_bytes(s))
          orig = r._draw_lat_values
          r._draw_lat_values = lambda _s: None
          try:
            r.render_to_png_bytes(s)
            without = _rgb(r.render_to_png_bytes(s))
          finally:
            r._draw_lat_values = orig
          sx = r.width / cr.DESIGN_WIDTH
          sy = r.height / cr.DESIGN_HEIGHT
          shift = 0.0 if layout == CLUSTER_PANEL_LAYOUT_DRIVING_LEFT else cr.NAVI_LIVE_PANEL_W
          panel_edge = (shift + cr.NAVI_LIVE_PANEL_X) * sx
          # the info panel has live (clock / animation) content, so compare the driving view only;
          # a line running into the panel would reach the driving-view edge (checked below)
          drive = (int(shift * sx), 0, int(panel_edge), r.height)
          box = ImageChops.difference(with_line.crop(drive), without.crop(drive)).getbbox()
          assert box is not None, "SR/SAD line not drawn"
          box = (box[0] + drive[0], box[1], box[2] + drive[0], box[3])
          assert box[2] <= panel_edge - 4, (box, panel_edge)
          icon_top = (cr.TPMS_STATUS_CAR_CENTER_Y - cr.TPMS_STATUS_ICON_H * 0.5) * sy
          x0, y0, x1, y1 = box
          assert x1 <= panel_edge, (box, panel_edge)          # never into the right/left info panel
          assert y1 <= icon_top + 1, (box, icon_top)           # never on the TPMS car / badges
          # 2026-10-26 (k9-hud-ld-lt): _draw_lat_values draws lines 1..3 of five (조향비 / 지연학습 / 토크학습), the top one
          # 80 px above the old SR / SAD line, the 토크학습 line is the widest
          assert y0 >= (cr.TPMS_STATUS_CAR_CENTER_Y - 150.0) * sy, box
          assert x0 >= (shift + cr.TPMS_STATUS_CENTER_X - 430.0) * sx, box
          # nothing else is drawn under the line (no other text / icon there)
          under = without.crop(box).convert("L")
          bright = sum(under.histogram()[91:])
          assert bright <= 0.01 * under.width * under.height, (box, bright)
  finally:
    r.close()
