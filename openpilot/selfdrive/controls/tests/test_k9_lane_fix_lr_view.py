"""K9LaneFix + K9LRView together (both experimental, NOT vehicle-validated).

Order on the device: modeld applies K9LRView to the parsed model outputs BEFORE modelV2 is filled and published;
the lateral planner reads that modelV2 (LanePlanner.parse_model) and only then K9LaneFix shapes the lane-mode
reference in LanePlanner.get_d_path (and controlsd caps the hand-over). The two features share no state and
neither module imports the other, so "both on" must be exactly "K9LaneFix fed the K9LRView-filtered modelV2":
- K9LaneFix=0 on filtered input == the frozen parent lane_planner_2 on the same input (bit for bit),
- K9LaneFix=1 on filtered input == the frozen F1 lane_planner_2 on the same input (bit for bit),
for K9LRView 0 and 1; K9LRView 0 leaves modelV2 byte-identical, and the published lane-line stds stay raw.
Inputs: synthetic model outputs (several seeds) with lanes, offsets, prob drops and a curve; synthetic plan path.
"""
import types
from pathlib import Path

import numpy as np
import pytest

from cereal import messaging
from openpilot.selfdrive.modeld.constants import ModelConstants, Plan
from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, PublishState
from openpilot.selfdrive.modeld.k9_lr_view import K9LRView
from openpilot.selfdrive.modeld.parse_model_outputs import Parser
import openpilot.selfdrive.controls.lib.lane_planner_2 as LP_NEW

HERE = Path(__file__).parent
REF = HERE / "k9_lane_fix_ref"
ROOT = HERE.parents[3]
N = LP_NEW.TRAJECTORY_SIZE
PATH_T = (np.linspace(0.0, 1.0, N) ** 2) * 10.0
X = np.asarray(ModelConstants.X_IDXS, dtype=np.float64)
OUTPUT_SLICES = {'meta': slice(0, 55), 'desire_pred': slice(55, 87), 'pose': slice(87, 99), 'wide_from_device_euler': slice(99, 105),
                 'road_transform': slice(105, 117), 'lane_lines': slice(117, 645), 'lane_lines_prob': slice(645, 653),
                 'road_edges': slice(653, 917), 'lead': slice(917, 1061), 'lead_prob': slice(1061, 1064),
                 'hidden_state': slice(1064, 1576), 'plan': slice(1576, 2566), 'desire_state': slice(2566, 2574), 'pad': slice(-2, None)}


class MemParams:
  def __init__(self, **values):
    self.values = dict(values)

  def get_int(self, k, *a, **kw):
    return int(self.values.get(k, 0))

  def get_float(self, k, *a, **kw):
    return float(self.values.get(k, 0.0))

  def get_bool(self, k, *a, **kw):
    return bool(self.values.get(k, False))


def load_module(path, name):
  mod = types.ModuleType(name)
  mod.__file__ = str(path)
  exec(compile(path.read_text(encoding="utf8"), str(path), "exec"), mod.__dict__)
  return mod


LP_PARENT = load_module(REF / "lane_planner_2.parent.py.ref", "k9_ref_lp_parent_lrv")
LP_F1 = load_module(REF / "lane_planner_2.F1.py.ref", "k9_ref_lp_F1_lrv")


def make_lp(mod, params):
  saved = mod.Params
  mod.Params = lambda: params
  try:
    return mod.LanePlanner()
  finally:
    mod.Params = saved


def model_frames(seed, n=400):
  """Parsed modeld outputs with plausible lane lines (4 lines, noisy, drifting offset, a curve, prob drops)."""
  rng = np.random.default_rng(seed)
  raw = rng.normal(0, 1, 2576).astype(np.float32)
  base = Parser().parse_outputs({k: raw[np.newaxis, v] for k, v in OUTPUT_SLICES.items()})
  v = 20.0; off = 0.0; curv = 0.0; probs = np.array([0.3, 0.95, 0.95, 0.3]); out = []
  for k in range(n):
    v = float(np.clip(v + rng.normal(0, 0.3), 5.0, 33.0))
    off = float(np.clip(off + rng.normal(0, 0.04) + (rng.normal(0, 0.5) if rng.random() < 0.01 else 0.0), -1.0, 1.0))
    curv = float(np.clip(curv + rng.normal(0, 1.5e-4), -0.01, 0.01))
    probs = np.clip(probs + rng.normal(0, 0.03, 4) + np.where(rng.random(4) < 0.01, -0.7, 0.0), 0.0, 1.0)
    if rng.random() < 0.1:
      probs[1:3] = np.minimum(probs[1:3] + 0.4, 1.0)
    o = {kk: np.array(vv, copy=True) for kk, vv in base.items()}
    centre = off + 0.5 * curv * X ** 2
    for i, c in enumerate((-5.25, -1.75, 1.75, 5.25)):
      o['lane_lines'][0, i, :, 0] = (centre + c + rng.normal(0, 0.06, X.shape)).astype(o['lane_lines'].dtype)
      o['lane_lines_stds'][0, i, :, 0] = rng.uniform(0.03, 0.2 if rng.random() < 0.02 else 0.12, X.shape).astype(o['lane_lines_stds'].dtype)
    o['lane_lines_prob'][0, 1::2] = probs.astype(o['lane_lines_prob'].dtype)
    o['lane_lines_prob'][0, 0::2] = (1.0 - probs).astype(o['lane_lines_prob'].dtype)
    o['plan'][0, :, Plan.VELOCITY][:, 0] = v
    o['plan'][0, :, Plan.ORIENTATION_RATE][:, 2] = v * curv
    x = v * PATH_T
    path_y = 0.5 * (curv + rng.normal(0, 3e-4)) * x ** 2 + rng.normal(0, 0.02) * PATH_T
    path_y[0] = 0.0
    out.append(dict(outputs=o, v=v, path=np.stack([x, path_y, np.zeros(N)], axis=1),
                    lanefull_mode=bool(rng.random() > 0.03), lane_change_multiplier=0.0 if rng.random() < 0.02 else 1.0,
                    lane_width_left=float(rng.uniform(1.5, 3.6)), lane_width_right=float(rng.uniform(1.5, 3.6)),
                    curve_speed=float(rng.uniform(-250, 250))))
  return out


def modelv2_bytes(outputs):
  msg = messaging.new_message('modelV2')
  msg.logMonoTime = 1
  fill_model_msg(msg, outputs, msg.modelV2.action, PublishState(), 10, 10, 10, 0.0, 123456789, 0.02, True)
  return msg.to_bytes()


def published(frames, lr_view_mode):
  """What modeld publishes for this K9LRView number (same call order as modeld.main)."""
  view = K9LRView()
  view.set_mode(lr_view_mode)
  return [modelv2_bytes(view.apply(f['outputs'])) for f in frames]


def run_planner(planner, frames, msgs):
  out = []
  for f, b in zip(frames, msgs, strict=True):
    planner.parse_model(messaging.log_from_bytes(b).modelV2)
    planner.lanefull_mode = f['lanefull_mode']
    planner.lane_change_multiplier = f['lane_change_multiplier']
    planner.lane_width_left = f['lane_width_left']
    planner.lane_width_right = f['lane_width_right']
    path, active = planner.get_d_path(None, f['v'], PATH_T, f['path'].copy(), f['curve_speed'])
    out.append((path, active))
  return out


def same(a, b):
  return all(np.array_equal(x[0], y[0]) and x[1] == y[1] for x, y in zip(a, b, strict=True))


def params(**kw):
  return MemParams(AdjustLaneOffset=0, LatMpcInputOffset=6, **kw)


def test_order_and_no_shared_state_in_code():
  modeld = (ROOT / "openpilot/selfdrive/modeld/modeld.py").read_text(encoding="utf-8")
  loop = modeld[modeld.index("def main("):]
  assert loop.index("model_output = k9_lr_view.apply(model_output)") < loop.index("fill_model_msg(")
  lat = (ROOT / "openpilot/selfdrive/controls/lib/lateral_planner.py").read_text(encoding="utf-8")
  assert lat.index("self.LP.parse_model(md)") < lat.index("self.LP.get_d_path(")
  lf = (ROOT / "openpilot/selfdrive/controls/lib/k9_lane_fix.py").read_text(encoding="utf-8")
  lp = (ROOT / "openpilot/selfdrive/controls/lib/lane_planner_2.py").read_text(encoding="utf-8")
  cd = (ROOT / "openpilot/selfdrive/controls/controlsd.py").read_text(encoding="utf-8")
  lrv = (ROOT / "openpilot/selfdrive/modeld/k9_lr_view.py").read_text(encoding="utf-8")
  assert all("k9_lr_view" not in s and "K9LRView" not in s for s in (lf, lp, cd))
  assert "k9_lane_fix" not in lrv and "K9LaneFix" not in lrv and "K9LaneFix" not in modeld


@pytest.mark.parametrize("seed", [0, 1, 2])
@pytest.mark.parametrize("lr_view", [0, 1])
def test_lane_fix_on_lr_view_output_equals_frozen_references(seed, lr_view):
  frames = model_frames(seed)
  msgs = published(frames, lr_view)
  raw = published(frames, 0)
  if lr_view == 0:
    assert msgs == [modelv2_bytes(f['outputs']) for f in frames]  # K9LRView 0: modelV2 byte-identical
  else:
    assert msgs != raw  # the view really changed the lanes
  off = run_planner(make_lp(LP_NEW, params(K9LaneFix=0)), frames, msgs)
  on = run_planner(make_lp(LP_NEW, params(K9LaneFix=1)), frames, msgs)
  assert same(off, run_planner(make_lp(LP_PARENT, params()), frames, msgs))
  assert same(on, run_planner(make_lp(LP_F1, params()), frames, msgs))
  assert sum(a for _, a in on) > 50  # lane mode was really used
  assert not same(off, on)  # K9LaneFix really acts on the filtered lanes
  assert all(np.all(np.isfinite(p)) for p, _ in on)


@pytest.mark.parametrize("lr_view", [1])
def test_both_on_differs_from_each_alone_and_keeps_raw_stds(lr_view):
  frames = model_frames(7)
  msgs, raw = published(frames, lr_view), published(frames, 0)
  both = run_planner(make_lp(LP_NEW, params(K9LaneFix=1)), frames, msgs)
  fix_only = run_planner(make_lp(LP_NEW, params(K9LaneFix=1)), frames, raw)
  view_only = run_planner(make_lp(LP_NEW, params(K9LaneFix=0)), frames, msgs)
  assert not same(both, fix_only) and not same(both, view_only)
  # K9LRView changes lane-line y / probs only; the stds lane_planner_2 uses for lane weighting stay the raw ones
  for a, b in zip(msgs, raw, strict=True):
    ea, eb = messaging.log_from_bytes(a), messaging.log_from_bytes(b)
    assert list(ea.modelV2.laneLineStds) == list(eb.modelV2.laneLineStds)
    assert list(ea.modelV2.position.y) == list(eb.modelV2.position.y)
