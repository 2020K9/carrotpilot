"""K9 wall hold (controls/lib/k9_wall_hold.py) behaviour tests. Synthetic model frames, no vehicle data."""
from types import SimpleNamespace as NS

import numpy as np

from openpilot.selfdrive.controls.lib import k9_wall_hold as W

X = 192.0 * (np.arange(33) / 32.0) ** 2


def frame(left_edge, right_edge=6.0, p_left=0.2, p_right=0.2, edge_std=0.6, line_left=-1.6, line_right=1.6):
  frame.n += 1
  n = frame.n
  lines = [NS(x=X.tolist(), y=np.full(33, y).tolist()) for y in (line_left - 3.5, line_left, line_right, line_right + 3.5)]
  edges = [NS(x=X.tolist(), y=np.full(33, left_edge).tolist()), NS(x=X.tolist(), y=np.full(33, right_edge).tolist())]
  return NS(frameId=n, laneLines=lines, laneLineProbs=[0.1, p_left, p_right, 0.1], roadEdges=edges,
            roadEdgeStds=[edge_std, 0.6], meta=NS(laneChangeState=0, desire='none'))


frame.n = 0


def run(h, fn, seconds, v=12.4, laneless=True, off=False, model=0.0):
  out, md = [], None
  for k in range(int(round(seconds / W.DT))):
    if k % 5 == 0:
      md = fn(k)
    out.append(h.update(md, v, laneless, off, model))
  return np.array(out)


def test_stable_wall_far_pulls_toward_wall():
  h = W.WallHold()
  out = run(h, lambda k: frame(-2.9), 4.0)
  assert h.active and h.side == 0
  assert out[-1] < -1e-4            # left wall 0.5 m further than D_TGT: curvature to the left (negative)


def test_wall_too_close_never_toward_wall():
  h = W.WallHold()
  out = run(h, lambda k: frame(-1.9), 4.0)
  assert np.all(out >= 0.0)


def test_clear_lane_line_no_change():
  h = W.WallHold()
  out = run(h, lambda k: frame(-2.9, p_left=0.95, p_right=0.95), 4.0, model=1e-3)
  assert np.all(out == 1e-3) and not h.active


def test_jumping_edge_never_active():
  h = W.WallHold()
  out = run(h, lambda k: frame(-2.2 if (k // 5) % 2 else -3.4), 4.0, model=5e-4)
  assert np.all(out == 5e-4)


def test_off_ramps_back_to_model_and_limits():
  h = W.WallHold()
  a = run(h, lambda k: frame(-3.3), 4.0)
  b = run(h, lambda k: frame(-3.3), 3.0, off=True)
  assert abs(b[-1]) == 0.0
  k = np.concatenate([a, b])
  v2 = 12.4 ** 2
  assert np.max(np.abs(k)) * v2 <= W.A_MAX + 1e-9
  assert np.max(np.abs(np.diff(k))) * v2 / W.DT <= W.J_MAX + 1e-6


def test_switch_off_step_unchanged():
  c = NS(k9_lc_mode=0, k9_wh=W.WallHold())
  out = W.k9_wall_hold_step(c, None, None, None, 0, 1.23e-3)
  assert out == 1.23e-3 and c.k9_wh is None
