"""K9LaneAssist OFS / LC line above the external HUD SR / SAD line (보노보노 2026-10-25 20:16 "라이브값 따로 못보나").

GL checks (xvfb-run): the line is drawn in its own empty box right above the SR / SAD line, never on that line, the TPMS
car or the info panel, for both panel layouts and with the C1 taxi meter on. Text / source checks are in
controls/tests/test_k9_lane_assist.py.
"""
import os
import sys
from dataclasses import replace
from pathlib import Path

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

gl = pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")


def _state(**kw):
  from cluster_models import DetectedVehicle, SimulatorInput
  from cluster_simulator import ClusterSimulator
  base = ClusterSimulator().update(SimulatorInput(), 0.05)
  return replace(base, speed_kph=62.0, cruise_kph=80, speed_limit_kph=80, onroad=True, gear_text="D",
                 applied_steer_ratio=12.9, applied_steer_delay_s=0.2,
                 detected_vehicles=(DetectedVehicle("L1", 32.0, 0.0, source="radarState", primary=True,
                                                    absolute_speed_kph=58.0),), **kw)


def _rgb(png):
  import io
  from PIL import Image
  return Image.open(io.BytesIO(png)).convert("RGB")


def test_state_fields_default_unknown():
  from cluster_models import ClusterUiState
  fields = ClusterUiState.__dataclass_fields__
  assert fields["k9_lane_offset_m"].default is None and fields["k9_lane_state"].default is None
  assert fields["k9_lane_accel"].default is None


@gl
def test_lane_line_in_its_own_box_above_sr_sad():
  from PIL import ImageChops
  from cluster_config import CLUSTER_PANEL_LAYOUT_DRIVING_LEFT, CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT
  import cluster_renderer as cr
  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)

  def render(s, skip=()):
    saved = {}
    for name in skip:
      saved[name] = getattr(r, name)
      setattr(r, name, lambda _s: None)
    try:
      r.render_to_png_bytes(s)
      return _rgb(r.render_to_png_bytes(s))
    finally:
      for name, fn in saved.items():
        setattr(r, name, fn)
  try:
    for meter in (False, True):
      r.set_taxi_meter_enabled(meter)
      for layout in (CLUSTER_PANEL_LAYOUT_DRIVING_LEFT, CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT):
        r.set_panel_layout(layout)
        for off, st, acc in ((0.123, 2, -0.0412), (None, 1, 0.0), (-0.3, 3, 0.2), (0.0, 0, 0.0), (-9.99, 2, -9.99)):
          s = _state(k9_lane_offset_m=off, k9_lane_state=st, k9_lane_accel=acc)
          full = render(s)
          no_lane = render(s, ("_draw_lane_values",))
          no_both = render(s, ("_draw_lane_values", "_draw_lat_values"))
          sx = r.width / cr.DESIGN_WIDTH
          sy = r.height / cr.DESIGN_HEIGHT
          shift = 0.0 if layout == CLUSTER_PANEL_LAYOUT_DRIVING_LEFT else cr.NAVI_LIVE_PANEL_W
          panel_edge = (shift + cr.NAVI_LIVE_PANEL_X) * sx
          drive = (int(shift * sx), 0, int(panel_edge), r.height)
          box = ImageChops.difference(full.crop(drive), no_lane.crop(drive)).getbbox()
          assert box is not None, "lane line not drawn"
          box = (box[0] + drive[0], box[1], box[2] + drive[0], box[3])
          sr = ImageChops.difference(no_lane.crop(drive), no_both.crop(drive)).getbbox()
          assert sr is not None
          sr = (sr[0] + drive[0], sr[1], sr[2] + drive[0], sr[3])
          x0, y0, x1, y1 = box
          assert x1 <= panel_edge - 4, (box, panel_edge)       # never into the info panel
          assert y1 <= sr[1] + 1, (box, sr)                     # above the SR / SAD line, not on it
          icon_top = (cr.TPMS_STATUS_CAR_CENTER_Y - cr.TPMS_STATUS_ICON_H * 0.5) * sy
          assert y1 <= icon_top + 1
          assert y0 >= (cr.TPMS_STATUS_CAR_CENTER_Y - 110.0) * sy, box
          assert x0 >= (shift + cr.TPMS_STATUS_CENTER_X - 300.0) * sx, box
          # nothing else is drawn under the line
          under = no_lane.crop(box).convert("L")
          bright = sum(under.histogram()[91:])
          assert bright <= 0.01 * under.width * under.height, (box, bright)
  finally:
    r.close()
