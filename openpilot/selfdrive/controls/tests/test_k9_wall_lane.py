"""K9WallLane (controls/lib/k9_wall_lane.py): wall as the lane line while lane lines are weak. Synthetic model frames only."""
import json
import os
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import lane_planner_2 as LPM
from openpilot.selfdrive.controls.lib import k9_wall_lane as W

T = np.array([10.0 * (i / 32.0) ** 2 for i in range(33)])
ROOT = os.path.join(os.path.dirname(__file__), "..", "..", "..")


class _P:
  def __init__(self, *a, **k): pass
  def get_int(self, k, *a, **kw): return 4 if k == "LatMpcInputOffset" else 0
  def get_float(self, k, *a, **kw): return 4.0 if k == "LatMpcInputOffset" else 0.0


@pytest.fixture(autouse=True)
def _params(monkeypatch):
  monkeypatch.setattr(LPM, "Params", _P)


V = 12.4
X = V * T


def frame(n, left_edge=-2.9, right_edge=6.0, p=0.2, edge_std=0.6, ll=-1.6, lr=1.6, model_y=0.0):
  lines = [NS(x=X.tolist(), y=np.full(33, y).tolist(), t=T.tolist()) for y in (ll - 3.5, ll, lr, lr + 3.5)]
  edges = [NS(x=X.tolist(), y=np.full(33, left_edge).tolist(), t=T.tolist()), NS(x=X.tolist(), y=np.full(33, right_edge).tolist(), t=T.tolist())]
  return NS(frameId=n, laneLines=lines, laneLineProbs=[0.1, p, p, 0.1], laneLineStds=[0.1] * 4, roadEdges=edges,
            roadEdgeStds=[edge_std, 0.6], action=NS(desiredCurvature=0.0),
            meta=NS(desireState=[], laneChangeState=0, desire=0, laneWidthLeft=0.0, laneWidthRight=0.0)), \
         np.column_stack([X, np.full(33, model_y), np.zeros(33)])


def cs(pressed=False, tq=0.0):
  return NS(vEgo=V, leftBlinker=False, rightBlinker=False, steeringPressed=pressed, steeringTorque=tq)


def run(planner, seconds, start=0, **kw):
  outs = []
  CS = kw.pop("CS", cs())
  for k in range(int(round(seconds / 0.05))):
    md, path = frame(start + k, **kw)
    planner.LP.parse_model(md)
    planner.LP.lanefull_mode = False
    base, act = planner.LP.get_d_path(CS, V, T, path, 250.0)
    outs.append(W.k9_wall_lane_step(planner, md, CS, V, T, base.copy(), act) + (base.copy(),))
  return outs


def planner(mode=1):
  return NS(LP=LPM.LanePlanner(), k9_wall_lane_mode=mode, laneless_only=False)


def y20(p):
  return float(np.interp(20.0, p[:, 0], p[:, 1]))


def test_mode0_unchanged_no_state():
  pl = planner(0)
  for out, act, base in run(pl, 4.0):
    assert act is False and np.array_equal(out, base)
  assert pl.k9_wl is None


def test_stable_wall_becomes_lane_line():
  pl = planner()
  out, act, base = run(pl, 5.0)[-1]
  assert act and pl.k9_wl.active and pl.k9_wl.side == 0
  assert abs(y20(out) - (-2.9 + W.D_TGT)) < 0.03      # lane centre 2.41 m from the wall, toward the wall vs the model path


def test_never_closer_than_d_min():
  pl = planner()
  outs = run(pl, 5.0, left_edge=-1.7)
  for out, _act, base in outs:   # never moved toward the wall inside D_MIN (the wall lane pushes the path away instead)
    assert np.all(out[:, 1] >= np.minimum(base[:, 1], -1.7 + W.D_MIN) - 1e-6)
  assert pl.k9_wl.active and y20(outs[-1][0]) > -1.7 + W.D_MIN


def test_clear_lines_nothing_changes():
  pl = planner()
  for out, _act, base in run(pl, 5.0, p=0.9):
    assert np.array_equal(out[:, 1], base[:, 1])
  assert not pl.k9_wl.active


def test_narrow_street_left_alone():
  pl = planner()
  for out, _act, base in run(pl, 5.0, right_edge=3.0):
    assert np.array_equal(out[:, 1], base[:, 1])


def test_lines_clear_again_smooth_return():
  pl = planner()
  run(pl, 5.0)
  outs = run(pl, 6.0, start=1000, p=0.95)
  ys = np.array([y20(o) for o, a, b in outs])
  assert abs(ys[-1] - y20(outs[-1][2])) < 1e-9 and outs[-1][1] is False   # back to the normal (laneless) path
  assert np.max(np.abs(np.diff(ys))) < 0.05        # <= 5 cm per model frame at 20 m


def test_steering_away_turns_off_toward_keeps():
  pl = planner()
  run(pl, 5.0)
  run(pl, 0.5, start=500, CS=cs(True, 250.0))       # toward the left wall
  assert pl.k9_wl.active
  run(pl, 0.5, start=600, CS=cs(True, -250.0))      # away from it
  assert not pl.k9_wl.active


def test_registered_top_of_experimental():
  with open(os.path.join(ROOT, "selfdrive", "carrot_settings.json"), encoding="utf-8") as f:
    cat = json.load(f)
  def walk(n):
    if isinstance(n, dict):
      yield n
      for v in n.values():
        yield from walk(v)
    elif isinstance(n, list):
      for v in n:
        yield from walk(v)
  exp = [g for g in walk(cat["menu"]) if g.get("id") == "EXPERIMENTAL"]
  assert len(exp) == 1 and exp[0]["params"][0] == "K9WallLane"
  p = [x for x in cat["params"] if x["name"] == "K9WallLane"][0]
  assert p["min"] == 0 and p["max"] == 1 and p["default"] == 0
  with open(os.path.join(ROOT, "common", "params_keys.h"), encoding="utf-8") as f:
    assert '{"K9WallLane", {PERSISTENT, INT, "0"}}' in f.read()


def test_lateral_planner_calls_hook_only_when_on():
  with open(os.path.join(ROOT, "selfdrive", "controls", "lib", "lateral_planner.py"), encoding="utf-8") as f:
    s = f.read()
  assert "if self.k9_wall_lane_mode == 1:" in s and s.count("k9_wall_lane_step(") == 1
  with open(os.path.join(ROOT, "selfdrive", "controls", "controlsd.py"), encoding="utf-8") as f:
    assert "k9_wall_hold" not in f.read()
