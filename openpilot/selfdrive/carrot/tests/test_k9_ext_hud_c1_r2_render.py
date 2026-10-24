"""K9ExtHud=1 C1 meter r2 render checks (GL display needed, e.g. xvfb-run).
LFA off -> the steering clam is drawn dimmed; the badge is drawn every frame; the carrot wheel stays hidden.
"""
import os
import sys
from dataclasses import replace
from pathlib import Path

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

pytestmark = pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")


class _Spy:
  def __init__(self, rl):
    self._rl = rl
    self.tex = []

  def __getattr__(self, name):
    return getattr(self._rl, name)

  def draw_texture_pro(self, tex, src, dst, origin, rot, tint):
    self.tex.append((int(tex.id), float(dst.x), int(tint.a)))
    return self._rl.draw_texture_pro(tex, src, dst, origin, rot, tint)


def _state(lfa, lane=True):
  from cluster_models import SimulatorInput
  from cluster_simulator import ClusterSimulator
  base = ClusterSimulator().update(SimulatorInput(), 0.05)
  return replace(base, speed_kph=40.0, onroad=True, gear_text="D", steering_angle_deg=20.0, lfa_active=lfa,
                 active_lane_line=lane)


def test_clam_dimmed_when_lfa_off_and_wheel_still_hidden(monkeypatch):
  import cluster_renderer
  import cluster_taxi_c1 as c1
  from cluster_renderer import ClusterUiRenderer
  r = ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  try:
    spy = _Spy(cluster_renderer.rl)
    monkeypatch.setattr(cluster_renderer, "rl", spy)
    r.set_taxi_meter_enabled(True)
    r.render_to_png_bytes(_state(True))
    p = r._taxi_meter_panel
    shell = int(p._textures["shell"].id)
    wheel = {int(t.id) for t in (r._lfa_texture, getattr(r, "_lfa_active_texture", None)) if t is not None}
    # meter_improve 20:43 (보노보노 '조개색은 원래색 유지 조향상관없이'): the clam keeps its own colours, LFA on or off
    for lfa, alpha in ((True, 255), (False, 255), (None, 255)):
      spy.tex.clear()
      r.render_to_png_bytes(_state(lfa))
      steering = [a for i, x, a in spy.tex if i == shell and abs(x - p._last_center[0]) < 1e-3]
      assert steering == [alpha], (lfa, steering)
      assert not [1 for i, _, _ in spy.tex if i in wheel]
      assert p.lfa_active is bool(lfa)
  finally:
    r.close()


def test_particles_drawn_from_the_bg2_sheet(monkeypatch):
  import cluster_renderer
  from cluster_renderer import ClusterUiRenderer
  r = ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  try:
    spy = _Spy(cluster_renderer.rl)
    monkeypatch.setattr(cluster_renderer, "rl", spy)
    r.set_taxi_meter_enabled(True)
    r.set_taxi_meter_season(4)                           # winter: snow at every time of day
    r.render_to_png_bytes(_state(True))
    r.render_to_png_bytes(_state(True))
    tex = r._taxi_meter_panel._textures.get("ptcl_white")
    assert tex is not None
    spy.tex.clear()
    r.render_to_png_bytes(_state(True))
    assert [1 for i, _, _ in spy.tex if i == int(tex.id)]
  finally:
    r.close()
