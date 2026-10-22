"""K9ExtHud=1 night lamp render check (01:03 지시; needs a GL display, e.g. xvfb-run).

Same meter, same frame: hour 12 vs hour 23. Only the information panel changes, the
'심야 할증 20%' lamp is lit (red) at night and absent by day, and '다음 +240원' replaces +200원.
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
  from cluster_models import SimulatorInput
  from cluster_simulator import ClusterSimulator
  base = ClusterSimulator().update(SimulatorInput(), 0.05)
  return replace(base, speed_kph=62.0, cruise_kph=80, speed_limit_kph=80, onroad=True, gear_text="D")


def _rgb(png):
  import io
  from PIL import Image
  return Image.open(io.BytesIO(png)).convert("RGB")


def _red_pixels(img, box):
  import numpy as np
  a = np.asarray(img.crop(box)).astype(int)
  return int(((a[..., 0] > 200) & (a[..., 1] < 150) & (a[..., 2] < 130)).sum())


def test_night_lamp_lit_only_at_night_and_only_in_the_panel():
  from PIL import ImageChops
  from cluster_renderer import NAVI_LIVE_PANEL_W, NAVI_LIVE_PANEL_X, NAVI_LIVE_PANEL_Y, ClusterUiRenderer
  from cluster_taxi_meter import CARD_OFFSET_X, CARD_OFFSET_Y, NIGHT_LAMP_H, NIGHT_LAMP_W, NIGHT_LAMP_X, NIGHT_LAMP_Y
  r = ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  try:
    s = _state()
    r.set_taxi_meter_enabled(True)
    panel = r._taxi_meter_panel
    clock = [100.0]
    panel._clock = lambda: clock[0]
    panel.freeze_animation_t = 1.05
    hour = [12]
    panel._local_hour = lambda: hour[0]
    r.render_to_png_bytes(s)
    panel.meter.metered_m = 3000.0 + 144.0 * 3 + 50.0   # same meter for both frames
    day = _rgb(r.render_to_png_bytes(s))
    hour[0] = 23
    night = _rgb(r.render_to_png_bytes(s))
    assert panel.night_rate == pytest.approx(0.2) and panel.step_won == 240
    px = r._information_panel_x(NAVI_LIVE_PANEL_X)
    lamp = (int(px + CARD_OFFSET_X + NIGHT_LAMP_X), int(NAVI_LIVE_PANEL_Y + CARD_OFFSET_Y + NIGHT_LAMP_Y),
            int(px + CARD_OFFSET_X + NIGHT_LAMP_X + NIGHT_LAMP_W) + 1, int(NAVI_LIVE_PANEL_Y + CARD_OFFSET_Y + NIGHT_LAMP_Y + NIGHT_LAMP_H) + 1)
    assert _red_pixels(night, lamp) > 40
    assert _red_pixels(day, lamp) == 0
    diff = ImageChops.difference(day, night).getbbox()
    assert diff is not None and diff[0] >= px and diff[2] <= px + NAVI_LIVE_PANEL_W, diff
    hour[0] = None                                   # clock not synced -> lamp off
    unsynced = _rgb(r.render_to_png_bytes(s))
    assert _red_pixels(unsynced, lamp) == 0 and panel.night_rate == 0.0
  finally:
    r.close()
