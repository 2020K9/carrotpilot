"""K9HudNavdy 0 and 1 drive screen the same size (대감님 11/22 16:05 "외부 HUD 0번 1번 주행화면 크기 똑바로 안 맞추냐"):
menu 0 (흔들림 줄임) used to lose the road part behind data forward 0, so its road stopped short of my car."""
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

import cluster_renderer as cr


def test_smoothed_line_keeps_the_part_behind_forward_0():
  r = cr.ClusterUiRenderer.__new__(cr.ClusterUiRenderer)
  r.set_k9_hud_smooth(True)
  r.k9_lanesm_clock = lambda: 0.0
  pts = [(-9.0, 1.8)] + [(float(f), 1.8) for f in range(0, 60, 3)]
  out = r._k9_lanesm("egoR", pts)
  assert out[0][0] == -9.0 and abs(out[0][1] - 1.8) < 0.05
  r.set_k9_hud_smooth(False)
  assert r._k9_lanesm("egoR", pts) == pts


@pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")
def test_menu0_and_menu1_road_fill_the_same_area(monkeypatch):
  import render_gear_screens as rg
  from k9_hud_road_state import road_state
  from openpilot.common import indicator_display
  from openpilot.selfdrive.carrot import signal_mode
  monkeypatch.setattr(indicator_display, "current", lambda: None)
  monkeypatch.setattr(signal_mode, "signal_mode", lambda: 0)
  monkeypatch.setattr(cr.EV6_WEATHER, "snapshot", lambda: None)
  st = road_state(replace(rg.synthetic_state("D"), car_fingerprint="KIA_K9"))
  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  try:
    def frame(menu):
      r.set_k9_hud_style(1)
      r.set_k9_hud_navdy(1)
      r.set_k9_hud_res(True)
      r.set_k9_hud_menu(1)
      r.set_k9_hud_smooth(menu == 0)
      for _ in range(5):
        png = r.render_to_png_bytes(st)
      return np.array(Image.open(io.BytesIO(png)).convert("RGB")).astype(int)
    zero, one = frame(0), frame(1)
  finally:
    r.close()
  # drawn road width per row (pixels that differ from the empty scene background), near half of the scene
  bg = one[2, 1395]   # scene background (dark, above the scene box)
  def widths(img):
    return (np.abs(img[240:430, 400:1400] - bg).sum(axis=2) > 30).sum(axis=1).astype(float)
  w0, w1 = widths(zero), widths(one)
  assert w1.min() > 300   # menu 1 road reaches the bottom of the scene
  assert np.max(np.abs(w0 - w1) / w1) < 0.05
