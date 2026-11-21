"""K9HudNavdy 0 = menu 1 picture + 흔들림 줄임 (대감님 11/21 17:55); 1 = d2c46ecbb, nothing smoothed or held."""
import sys
from pathlib import Path

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

import cluster_renderer as cr


def _r(smooth):
  r = cr.ClusterUiRenderer.__new__(cr.ClusterUiRenderer)
  r.set_k9_hud_smooth(smooth)
  t = [0.0]
  r.k9_lanesm_clock = lambda: t[0]
  return r, t


def test_default_is_off():
  assert cr.ClusterUiRenderer.k9_hud_smooth is False


def test_menu1_lines_drawn_as_given():
  r, t = _r(False)
  for k in range(20):
    t[0] = k * 0.1
    pts = [(float(f), 0.3 * (-1) ** k) for f in range(0, 60, 10)]
    assert r._k9_lanesm("egoL", pts) == pts
  assert getattr(r, "_k9_lanesm_state", None) is None


def test_menu0_wobble_reduced():
  r, t = _r(True)
  out = []
  for k in range(20):
    t[0] = k * 0.1
    pts = [(float(f), -1.8 + 0.3 * (-1) ** k) for f in range(0, 60, 10)]
    out.append(r._k9_lanesm("egoL", pts)[2][1])
  wob_in = 0.6
  wob_out = max(abs(a - b) for a, b in zip(out[10:], out[11:]))
  assert wob_out < wob_in


def test_switch_resets_state():
  r, t = _r(True)
  r._k9_lanesm("egoL", [(0.0, 1.0), (10.0, 1.0)])
  assert r._k9_lanesm_state is not None
  r.set_k9_hud_smooth(False)
  assert r._k9_lanesm_state is None and r._k9_road_hold is None and r._k9_alpha == {}
