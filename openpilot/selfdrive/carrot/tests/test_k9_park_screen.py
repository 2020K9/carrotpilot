"""K9 parked screen (대감님 11/11 18:24 / 18:33): 2020 K9 RJ car model in P, fuel level in the battery's place
(carState.fuelGauge, "--" when the car does not report it), same text sizes as the EV6 screen. New tests only."""
import io
import os
import subprocess
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

import cluster_renderer as renderer_module
from cluster_renderer import ClusterUiRenderer
from cluster_vehicle_models import ego_model_name

K9_OBJ = CLUSTER / "assets" / "models" / "k9" / "k9_cluster.obj"


def test_k9_fingerprint_selects_the_k9_park_model():
  assert ego_model_name("KIA_K9") == "k9"
  assert ego_model_name("KIA_EV6") == "ev6"


def test_k9_model_file_is_original_normalised_and_reproducible(tmp_path):
  text = K9_OBJ.read_text()
  assert "no third-party mesh or photo" in text
  v = np.array([[float(x) for x in line.split()[1:4]] for line in text.splitlines() if line.startswith("v ")])
  assert len(v) > 1000
  # mirrors stick out past the +-0.5 body width on purpose (like the EV6 model)
  assert v[:, 0].min() > -0.6 and v[:, 0].max() < 0.6
  assert v[:, 1].min() >= -0.51 and v[:, 1].max() <= 0.51
  assert v[:, 2].min() >= 0.0 and v[:, 2].max() <= 1.001
  mats = {line.split()[1] for line in text.splitlines() if line.startswith("usemtl ")}
  assert all(m.startswith("rgb_") and len(m) == 10 for m in mats)
  out = tmp_path / "k9.obj"
  subprocess.check_call([sys.executable, str(CLUSTER / "make_k9_park_model.py"), str(out)], stdout=subprocess.DEVNULL)
  assert out.read_text() == text


def test_k9_model_is_not_a_lead_or_side_vehicle():
  others = renderer_module.OTHER_VEHICLE_MODELS_DIR
  assert renderer_module.PARK_K9_MODEL_PATH.parent != others
  assert not list(others.glob("k9*"))


def test_k9_ego_model_falls_back_to_ev6_until_loaded():
  r = ClusterUiRenderer()
  r._park_vehicle_model = "ev6-model"
  assert r._ego_model("k9") == ("ev6-model", "ev6")
  r._park_k9_model = "k9-model"
  assert r._ego_model("k9") == ("k9-model", "k9")
  assert r._ego_model("ev6") == ("ev6-model", "ev6")


@pytest.mark.parametrize("fraction,expected", [
  (None, "--"), (0.0, "--"), (-0.1, "--"), (1.2, "--"), (float("nan"), "--"),
  (0.004, "0%"), (0.1, "10%"), (0.62, "62%"), (1.0, "100%"),
])
def test_k9_fuel_text(fraction, expected):
  assert ClusterUiRenderer.k9_fuel_text(fraction) == expected


@pytest.mark.parametrize("fraction,expected", [(None, "--"), (0.62, "62%")])
def test_k9_fuel_uses_the_same_34px_value_text_as_the_battery(monkeypatch, fraction, expected):
  r = ClusterUiRenderer()
  calls = []
  monkeypatch.setattr(r, "_rounded_rect", lambda *_: None)
  for name in ("draw_rectangle_rounded", "draw_line_ex"):
    monkeypatch.setattr(renderer_module.rl, name, lambda *_: None)
  r._k9_draw_fuel(fraction, 236.0, 402.0, renderer_module.EV6_PARK_COLORS_DARK,
                  lambda value, x, y, size, color, *a: calls.append((value, size, color)))
  assert calls == [(expected, 34, "text")]


def _park_draw_calls(monkeypatch, label, gauge):
  r = ClusterUiRenderer()
  seen = []
  monkeypatch.setattr(r, "_ev6_draw_battery", lambda soc, *a: seen.append(("battery", soc)))
  monkeypatch.setattr(r, "_k9_draw_fuel", lambda f, *a: seen.append(("fuel", f)))

  real_rl = renderer_module.rl

  class _Rl:
    def __getattr__(self, name):
      return getattr(real_rl, name) if name[0].isupper() or name in ("Rectangle", "Vector2", "Color") \
        else (lambda *a, **k: None)
  monkeypatch.setattr(renderer_module, "rl", _Rl())
  monkeypatch.setattr(r, "_draw_text", lambda *a, **k: None)
  monkeypatch.setattr(r, "_rounded_rect", lambda *a, **k: None)
  monkeypatch.setattr(renderer_module.EV6_WEATHER, "snapshot", lambda: None)
  state = SimpleNamespace(fuel_gauge=gauge, energy_gauge_label=label, car_fingerprint="KIA_K9", tpms=None,
                          trip_report=None)
  r._park_model_texture = None
  r._ev6_draw_park_screen(state)
  return seen


def test_park_screen_draws_fuel_for_fuel_cars_and_battery_for_evs(monkeypatch):
  assert _park_draw_calls(monkeypatch, "fuel", None) == [("fuel", None)]
  assert _park_draw_calls(monkeypatch, "fuel", 0.62) == [("fuel", 0.62)]
  assert _park_draw_calls(monkeypatch, "battery", 0.76) == [("battery", 0.76)]


@pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")
@pytest.mark.parametrize("theme", ["dark", "light"])
def test_k9_park_screen_renders_the_k9_model(monkeypatch, tmp_path, theme):
  from PIL import Image
  from cluster_live import standby_state
  monkeypatch.setattr(renderer_module, "EV6_PARK_SCREEN_OFF_FLAG", str(tmp_path / "park-off"))
  monkeypatch.setattr(renderer_module.EV6_WEATHER, "snapshot", lambda: None)
  r = ClusterUiRenderer(width=1920, height=462, language="ko", theme_mode=theme)
  r.open(hidden=True)
  try:
    base = replace(standby_state(), onroad=True, gear_text="P", car_fingerprint="KIA_K9", energy_gauge_label="fuel")
    frames = {}
    for name, gauge in (("none", None), ("62", 0.62)):
      state = replace(base, fuel_gauge=gauge)
      r.render_to_png_bytes(state)
      with Image.open(io.BytesIO(r.render_to_png_bytes(state))) as img:
        frames[name] = np.array(img.convert("RGB")).astype(int)
    assert r._park_k9_model is not None
    assert r._park_model_key == "k9"
    assert r._park_model_texture is not None
    assert "k9" not in r._other_vehicle_names
    # the car area (centre) is drawn, the fuel value differs only in the left column
    centre = frames["none"][60:420, 560:1020]
    assert centre.std() > 20
    diff = np.any(frames["none"] != frames["62"], axis=2)
    ys, xs = np.nonzero(diff)
    assert len(xs) > 0 and xs.max() < 560
  finally:
    r.close()


def test_reverse_path_geometry_is_per_car_k9_uses_its_carparams():
  import math
  from opendbc.car.hyundai.values import CAR
  k9 = CAR.KIA_K9.config.specs
  assert renderer_module.reverse_geometry("KIA_K9") == (k9.wheelbase, k9.steerRatio) == (3.105, 14.5)
  assert renderer_module.reverse_geometry("KIA_EV6") == (2.90, 16.0)
  assert renderer_module.reverse_geometry("") == (2.90, 16.0)
  assert renderer_module.REVERSE_GEOMETRY_FALLBACK["KIA_K9"] == (k9.wheelbase, k9.steerRatio)
  ev6 = ClusterUiRenderer.ev6_reverse_path(160.0, "KIA_EV6")
  assert ev6 == ClusterUiRenderer.ev6_reverse_path(160.0)
  k9p = ClusterUiRenderer.ev6_reverse_path(160.0, "KIA_K9")
  assert math.isclose(k9p[1][2], 5.0 / 60 * math.tan(math.radians(160.0 / 14.5)) / 3.105)
  assert math.isclose(ev6[1][2], 5.0 / 60 * math.tan(math.radians(160.0 / 16.0)) / 2.90)
  assert k9p != ev6
