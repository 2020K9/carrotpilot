"""K9HudNavdy 2 (대감님 11/16 00:46/00:55): menu 1 one scene + comma / Jetson CPU GPU memory in the screen-right block.
0 and 1 stay as before. Render checks need a GL display (xvfb-run)."""
import io
import json
import os
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from PIL import Image

ROOT = Path(__file__).resolve().parents[4]
CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

import cluster_renderer as cr
from cluster_live import live_comma_resources


def test_settings_menu_has_three_values():
  data = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text())
  item = next(p for p in data["params"] if p["name"] == "K9HudNavdy")
  assert (item["min"], item["max"], item["default"]) == (0, 2, 0)
  assert all(len(v) == 3 for v in item["options"].values())
  assert "2" in item["descr"] and "2" in item["edescr"] and "2" in item["cdescr"]


def test_reader_passes_2_and_main_maps_it_to_scene_plus_res():
  import main
  reader = main.K9HudNavdyParamReader()
  for raw, want in ((0, 0), (1, 1), (2, 2), (3, 0), (-1, 0)):
    reader._params = SimpleNamespace(get_int=lambda _k, v=raw: v)
    assert reader.read() == want
  src = (CLUSTER / "main.py").read_text()
  assert src.count("renderer.set_k9_hud_style(min(k9_hud_menu, 1))") == 2
  assert src.count("renderer.set_k9_hud_res(k9_hud_menu == 2)") == 2


def test_renderer_res_off_by_default_and_no_jetson_values():
  r = cr.ClusterUiRenderer.__new__(cr.ClusterUiRenderer)
  assert r.k9_hud_res is False
  assert r.k9_jetson_resources() == (None, None, None)
  r.set_k9_hud_res(1)
  assert r.k9_hud_res is True
  assert cr.ClusterUiRenderer.k9_res_text(None) == "--" and cr.ClusterUiRenderer.k9_res_text(41.4) == "41%"


def test_comma_resources_from_device_state_only():
  ds = SimpleNamespace(cpuUsagePercent=[10, 20, 30, 200], gpuUsagePercent=41, memoryUsagePercent=57.0)
  assert live_comma_resources(ds) == (20.0, 41.0, 57.0)
  assert live_comma_resources(SimpleNamespace()) == (None, None, None)
  assert live_comma_resources(None) is None


def test_jetson_resources_from_host_performance_or_none(monkeypatch):
  monkeypatch.syspath_prepend(str(ROOT / "tools" / "jetlink"))
  hud_health = pytest.importorskip("hud_health")
  perf = {"version": 1, "cpu": [{"online": True, "busy_pct": 10.0}, {"online": True, "busy_pct": 30.0},
                                 {"online": False, "busy_pct": None}],
          "gpu": {"17000000.gpu": {"load_pct": 35.0}}, "memory": {"MemTotal_kb": 8000, "MemAvailable_kb": 6000}}
  assert hud_health.jetson_resources({"performance": perf}) == (20.0, 35.0, 25.0)
  assert hud_health.jetson_resources({}) == (None, None, None)
  assert hud_health.jetson_resources({"performance": {"status": "stale"}}) == (None, None, None)


@pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")
def test_menu_2_only_changes_the_screen_right_block(monkeypatch):
  import render_gear_screens as rg
  from openpilot.common import indicator_display
  from openpilot.selfdrive.carrot import signal_mode
  monkeypatch.setattr(indicator_display, "current", lambda: None)
  monkeypatch.setattr(signal_mode, "signal_mode", lambda: 0)
  monkeypatch.setattr(cr.EV6_WEATHER, "snapshot", lambda: None)
  st = replace(rg.synthetic_state("D"), car_fingerprint="KIA_K9", comma_resources=(23.0, 41.0, 57.0))
  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  try:
    def frame(res):
      r.set_k9_hud_style(1)
      r.set_k9_hud_navdy(1)
      r.set_k9_hud_res(res)
      r.render_to_png_bytes(st)
      return np.array(Image.open(io.BytesIO(r.render_to_png_bytes(st))).convert("RGB"))
    one, two, one_again = frame(False), frame(True), frame(False)
  finally:
    r.close()
  assert np.array_equal(one, one_again)
  diff = np.any(one != two, axis=2)
  ys, xs = np.nonzero(diff)
  assert len(xs) > 1000
  scale = one.shape[1] / cr.DESIGN_WIDTH
  assert xs.min() >= (cr.K9L_SCENE_X0 - 2) * scale   # 11/18: 2 also shrinks the scene (aspect kept) for the 480 px block
  assert ys.max() >= cr.K9L_TPMS_HAIR * scale   # 11/16 01:33 "tpms말고도 전체삭제": the TPMS area is replaced too
