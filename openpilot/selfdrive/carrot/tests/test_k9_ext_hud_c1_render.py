"""K9ExtHud=1 C1 meter render checks (needs a GL display, e.g. xvfb-run).

* 보노보노 11:48 "당근스티어링은 빼자": with the C1 meter on screen (K9ExtHud=1) the carrot wheel (LFA
  carrot_wheel icon, rotated by -steeringAngleDeg) is not drawn; with the meter off (K9ExtHud 0/-1) it is
  drawn exactly as before.
* The clam is drawn every frame at -steeringAngleDeg (the rotation argument of its draw call).
* K9MeterSeason picks the background files of that season.
"""
import os
import sys
from dataclasses import replace
from pathlib import Path

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

pytestmark = pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")


def _state(steer=0.0):
  from cluster_models import SimulatorInput
  from cluster_simulator import ClusterSimulator
  base = ClusterSimulator().update(SimulatorInput(), 0.05)
  return replace(base, speed_kph=40.0, cruise_kph=80, speed_limit_kph=80, onroad=True, gear_text="D",
                 steering_angle_deg=steer, lfa_active=True)


class _Spy:
  """Forwards to pyray; records draw_texture_pro calls."""

  def __init__(self, rl):
    self._rl = rl
    self.calls = []

  def __getattr__(self, name):
    return getattr(self._rl, name)

  def draw_texture_pro(self, tex, src, dst, origin, rot, tint):
    self.calls.append((int(tex.id), float(dst.x), float(dst.y), float(rot)))
    return self._rl.draw_texture_pro(tex, src, dst, origin, rot, tint)


@pytest.fixture(scope="module")
def renderer():
  from cluster_renderer import ClusterUiRenderer
  r = ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  yield r
  r.close()


def _wheel_ids(r):
  return {int(t.id) for t in (r._lfa_texture, getattr(r, "_lfa_active_texture", None)) if t is not None}


def test_carrot_wheel_not_drawn_with_c1_meter_drawn_without(renderer, monkeypatch):
  import cluster_renderer
  r = renderer
  spy = _Spy(cluster_renderer.rl)
  monkeypatch.setattr(cluster_renderer, "rl", spy)
  s = _state(35.0)
  r.render_to_png_bytes(s)                               # K9ExtHud 0/-1: meter off
  spy.calls.clear()
  r.render_to_png_bytes(s)
  ids = _wheel_ids(r)
  assert ids, "carrot wheel texture not loaded"
  wheel_off = [c for c in spy.calls if c[0] in ids]
  assert wheel_off and all(c[3] == pytest.approx(-35.0) for c in wheel_off)   # drawn, -steeringAngleDeg
  r.set_taxi_meter_enabled(True)                          # K9ExtHud = 1
  r.render_to_png_bytes(s)
  spy.calls.clear()
  r.render_to_png_bytes(s)
  assert [c for c in spy.calls if c[0] in ids] == []      # no carrot wheel
  r.set_taxi_meter_enabled(False)                         # back to 0: drawn again
  spy.calls.clear()
  r.render_to_png_bytes(s)
  assert [c for c in spy.calls if c[0] in ids]


def test_clam_draw_call_rotation_is_minus_steering(renderer, monkeypatch):
  import cluster_renderer
  r = renderer
  spy = _Spy(cluster_renderer.rl)
  monkeypatch.setattr(cluster_renderer, "rl", spy)
  r.set_taxi_meter_enabled(True)
  r.render_to_png_bytes(_state())
  panel = r._taxi_meter_panel
  shell_id = int(panel._textures["shell"].id)
  for deg in (0.0, 7.3, -42.0, 180.0, -275.5, 540.0):
    spy.calls.clear()
    r.render_to_png_bytes(_state(deg))
    steering = [c for c in spy.calls if c[0] == shell_id and abs(c[1] - (panel._last_center[0])) < 1e-3]
    assert steering and all(c[3] == pytest.approx(-deg, abs=1e-9) for c in steering), (deg, steering)


def test_meter_season_setting_loads_that_season(renderer):
  r = renderer
  r.set_taxi_meter_enabled(True)
  for value, season in ((1, "spring"), (2, "summer"), (3, "autumn"), (4, "winter")):
    r.set_taxi_meter_season(value)
    r.render_to_png_bytes(_state())
    names = {n for n, t in r._taxi_meter_panel._textures.items() if t is not None and n.startswith("back_")}
    assert names and all(f"_{season}_" in n for n in names), (value, names)
  r.set_taxi_meter_season(9)                              # out of range -> 0 (auto)
  assert r._taxi_meter_season == 0
