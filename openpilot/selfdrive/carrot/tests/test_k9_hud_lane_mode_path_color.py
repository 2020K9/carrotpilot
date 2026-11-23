"""External HUD (K9HudNavdy 1) path band: lime while lane mode is active (대감님 11/23 16:06
"예상경로추종 파란색 장판 깔리는 거 레인모드 활성화 시 연두색으로"), the deep blue band otherwise.
Render check needs a GL display (xvfb-run)."""
import io
import os
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cluster_renderer as cr  # noqa: E402


def test_lane_mode_band_colour_is_light_lime_and_blue_kept():
  assert cr.K9_NAVDY_EGO_LANE == (34, 78, 196)
  assert cr.K9_NAVDY_EGO_LANE_LANEMODE == (0x9b, 0xe1, 0x5d)


@pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")
@pytest.mark.parametrize("menu", [0, 1])
def test_path_band_follows_active_lane_line(monkeypatch, menu):
  import render_gear_screens as rg
  from k9_hud_road_state import line, road_state
  from openpilot.common import indicator_display
  from openpilot.selfdrive.carrot import signal_mode
  monkeypatch.setattr(indicator_display, "current", lambda: None)
  monkeypatch.setattr(signal_mode, "signal_mode", lambda: 0)
  monkeypatch.setattr(cr.EV6_WEATHER, "snapshot", lambda: None)
  bands = []
  orig = cr.ClusterUiRenderer._k9_turzx_band

  def spy(self, left, right, color, fade=False, end_m=None):
    bands.append((tuple(color), fade))
    return orig(self, left, right, color, fade, end_m)

  monkeypatch.setattr(cr.ClusterUiRenderer, "_k9_turzx_band", spy)
  st = replace(road_state(replace(rg.synthetic_state("D"), car_fingerprint="KIA_K9")), model_path=line(0.0))
  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  try:
    def frame(lane):
      r.set_k9_hud_style(1)
      r.set_k9_hud_navdy(1)
      r.set_k9_hud_res(True)
      r.set_k9_hud_menu(1)
      r.set_k9_hud_smooth(menu == 0)
      for _ in range(5):
        bands.clear()
        png = r.render_to_png_bytes(replace(st, active_lane_line=lane))
      return np.array(Image.open(io.BytesIO(png)).convert("RGB")).astype(int), [b for b in bands if b[1]]
    off, off_bands = frame(False)
    on, on_bands = frame(True)
    _, none_bands = frame(None)
  finally:
    r.close()
  assert off_bands == [((*cr.K9_NAVDY_EGO_LANE, 215), True)]
  assert on_bands == [((*cr.K9_NAVDY_EGO_LANE_LANEMODE, 215), True)]
  assert none_bands == off_bands  # unknown lane state keeps the blue band
  diff = np.abs(on - off).sum(axis=2) > 30
  assert diff.sum() > 2000  # the band itself changes colour
  ys, xs = np.nonzero(diff)
  px = on[ys, xs]
  assert (px[:, 1] > px[:, 2]).mean() > 0.9  # green over blue where it changed (lime)
  pb = off[ys, xs]
  assert (pb[:, 2] > pb[:, 1]).mean() > 0.9  # was blue
