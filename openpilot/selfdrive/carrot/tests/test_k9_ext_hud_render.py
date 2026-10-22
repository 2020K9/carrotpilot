"""K9ExtHud render checks with the production raylib renderer (needs a GL display, e.g. xvfb-run).

(a) taxi meter off (default) renders byte-identically frame to frame, and switching the meter
    on and off again changes nothing outside the information panel.
(f) the taxi meter panel (K9ExtHud=1) never changes the driving screen of the K9ExtHud=0 screen,
    for both panel layouts (driving left / driving right).
"""
import os
import sys
from dataclasses import replace
from pathlib import Path

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

pytestmark = pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")


def _state():
  from cluster_models import DetectedVehicle, SimulatorInput
  from cluster_simulator import ClusterSimulator
  base = ClusterSimulator().update(SimulatorInput(), 0.05)
  return replace(base, speed_kph=62.0, cruise_kph=80, speed_limit_kph=80, onroad=True, gear_text="D",
                 detected_vehicles=(DetectedVehicle("L1", 32.0, 0.0, source="radarState", primary=True, absolute_speed_kph=58.0),))


def _rgb(png):
  import io
  from PIL import Image
  return Image.open(io.BytesIO(png)).convert("RGB")


def test_taxi_panel_only_touches_the_information_panel():
  from PIL import ImageChops
  from cluster_config import CLUSTER_PANEL_LAYOUT_DRIVING_LEFT, CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT
  from cluster_renderer import NAVI_LIVE_PANEL_W, NAVI_LIVE_PANEL_X, ClusterUiRenderer
  r = ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  try:
    s = _state()
    for layout in (CLUSTER_PANEL_LAYOUT_DRIVING_LEFT, CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT):
      r.set_panel_layout(layout)
      r.render_to_png_bytes(s)
      off_a = r.render_to_png_bytes(s)
      off_b = r.render_to_png_bytes(s)
      assert off_a == off_b  # default path deterministic
      r.set_taxi_meter_enabled(True)
      r.render_to_png_bytes(s)
      on = r.render_to_png_bytes(s)
      r.set_taxi_meter_enabled(False)
      off_c = r.render_to_png_bytes(s)
      if layout == CLUSTER_PANEL_LAYOUT_DRIVING_LEFT:
        drive_box = (0, 0, NAVI_LIVE_PANEL_X, r.height)
        panel_x0 = NAVI_LIVE_PANEL_X
      else:
        drive_box = (NAVI_LIVE_PANEL_W, 0, r.width, r.height)
        panel_x0 = 0
      base = _rgb(off_a)
      for other in (on, off_c):
        diff = ImageChops.difference(base.crop(drive_box), _rgb(other).crop(drive_box))
        assert diff.getbbox() is None, (layout, diff.getbbox())
      full = ImageChops.difference(base, _rgb(on)).getbbox()
      assert full is not None and full[0] >= panel_x0 and full[2] <= panel_x0 + NAVI_LIVE_PANEL_W, (layout, full)
  finally:
    r.close()
