"""R -> P on the Jetson (대감님 11/19 06:18 / 06:49 "R단에서 P단이나 D단 넘어갈 때 HUD 반응이 왜 이렇게 느림?"):
while R is shown the renderer already loads the P screen images, so the first P frame after R does no PNG decode /
mipmap work (on the Jetson CPU that work delayed the new screen). GL display needed (xvfb-run)."""
import os
import sys
import time
from dataclasses import replace
from pathlib import Path

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

pytestmark = pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")


def _state(gear):
  from cluster_models import SimulatorInput
  from cluster_simulator import ClusterSimulator
  base = ClusterSimulator().update(SimulatorInput(), 0.05)
  return replace(base, onroad=True, car_fingerprint="KIA_K9", speed_kph=0.0, gear_text=gear)


def test_p_images_ready_before_the_first_p_frame(monkeypatch, tmp_path):
  import cluster_renderer as R
  monkeypatch.setattr(R, "EV6_PARK_SCREEN_OFF_FLAG", str(tmp_path / "park-off"))
  monkeypatch.setattr(R, "EV6_REVERSE_SCREEN_OFF_FLAG", str(tmp_path / "reverse-off"))
  r = R.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  try:
    for _ in range(2):
      r.render_to_rgba_bytes(_state("D"))
    r.render_to_rgba_bytes(_state("R"))
    assert getattr(r, "_k9_park_sprite", None), "P car image not loaded while in R"
    assert getattr(r, "_k9_park_floor", None), "P floor image not loaded while in R"
    loads = []
    real = R.rl.load_image
    monkeypatch.setattr(R.rl, "load_image", lambda *a, **k: loads.append(a) or real(*a, **k))
    t = time.perf_counter()
    r.render_to_rgba_bytes(_state("P"))
    first_p = time.perf_counter() - t
    assert loads == []                          # nothing decoded on the gear change frame
    t = time.perf_counter()
    r.render_to_rgba_bytes(_state("P"))
    assert first_p < max(3.0 * (time.perf_counter() - t), 0.05)
    r.render_to_rgba_bytes(_state("D"))         # and straight on to D: no load either
    assert loads == []
  finally:
    r.close()
