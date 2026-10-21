"""K9LRView (experimental menu number, NOT vehicle-validated).

(a) 0 / unknown numbers: model outputs pass through as the same object and the serialized modelV2 is
    byte-identical to the parent path (no K9LRView call).
(b) 1 (P1g): Kalman lane-line tracker reduces frame-to-frame noise, keeps prob in [0, 1], resets on
    lane jumps > 1 m, falls back to raw outputs on non-finite input, and switching the number resets state.
(c) menu / params registration (default 0).
"""
import json
from pathlib import Path

import numpy as np

from cereal import messaging
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, PublishState
from openpilot.selfdrive.modeld.k9_lr_view import K9LRView, LaneLineTracker, read_mode
from openpilot.selfdrive.modeld.parse_model_outputs import Parser

ROOT = Path(__file__).resolve().parents[4]
OUTPUT_SLICES = {'meta': slice(0, 55), 'desire_pred': slice(55, 87), 'pose': slice(87, 99), 'wide_from_device_euler': slice(99, 105),
                 'road_transform': slice(105, 117), 'lane_lines': slice(117, 645), 'lane_lines_prob': slice(645, 653),
                 'road_edges': slice(653, 917), 'lead': slice(917, 1061), 'lead_prob': slice(1061, 1064),
                 'hidden_state': slice(1064, 1576), 'plan': slice(1576, 2566), 'desire_state': slice(2566, 2574), 'pad': slice(-2, None)}


def parsed_outputs(seed):
  raw = np.random.default_rng(seed).normal(0, 1, 2576).astype(np.float32)
  return Parser().parse_outputs({k: raw[np.newaxis, v] for k, v in OUTPUT_SLICES.items()})


def snapshot(outputs):
  return {k: np.array(v, copy=True) for k, v in outputs.items()}


def modelv2_bytes(outputs):
  msg = messaging.new_message('modelV2')
  msg.logMonoTime = 1
  action = msg.modelV2.action
  fill_model_msg(msg, outputs, action, PublishState(), 10, 10, 10, 0.0, 123456789, 0.02, True)
  return msg.to_bytes()


def test_off_is_same_object_and_untouched():
  view = K9LRView()
  for k in range(30):
    out = parsed_outputs(k)
    before = snapshot(out)
    assert view.apply(out) is out
    for name, arr in before.items():
      assert arr.tobytes() == np.asarray(out[name]).tobytes(), name


def test_unknown_numbers_are_off():
  view = K9LRView()
  for mode in (0, 2, 7, -1, None, 'x', '0'):
    view.set_mode(mode)
    assert view.mode == 0
    out = parsed_outputs(1)
    assert view.apply(out) is out


def test_off_modelv2_bit_identical_over_sequence():
  view = K9LRView()
  view.set_mode(0)
  for k in range(20):
    a = parsed_outputs(100 + k)
    b = parsed_outputs(100 + k)
    assert modelv2_bytes(a) == modelv2_bytes(view.apply(b))


def test_off_after_on_is_bit_identical_again():
  view = K9LRView()
  view.set_mode(1)
  for k in range(10):
    view.apply(parsed_outputs(k))
  view.set_mode(0)
  for k in range(10):
    a, b = parsed_outputs(200 + k), parsed_outputs(200 + k)
    assert modelv2_bytes(a) == modelv2_bytes(view.apply(b))


def _lane_seq(n, rng, noise=0.15, prob_noise=0.25):
  base = np.array([-5.4, -1.8, 1.8, 5.4])[:, None] * np.ones((4, ModelConstants.IDX_N))
  for _ in range(n):
    y = base + rng.normal(0, noise, base.shape)
    std = np.full(base.shape, 0.4)
    p = np.clip(0.55 + rng.normal(0, prob_noise, 4), 0, 1)
    yield y, std, p


def test_on_reduces_jitter_and_keeps_prob_range():
  rng = np.random.default_rng(0)
  tr = LaneLineTracker(q=0.05, prob_alpha=0.3, gate_m=1.0)
  raw, filt, probs = [], [], []
  for y, s, p in _lane_seq(400, rng):
    fy, fs, fp = tr.update(y, s, p)
    raw.append(y); filt.append(fy); probs.append(fp)
  raw, filt, probs = np.array(raw), np.array(filt), np.array(probs)
  assert np.abs(np.diff(filt, axis=0)).mean() < 0.6 * np.abs(np.diff(raw, axis=0)).mean()
  assert probs.min() >= 0 and probs.max() <= 1
  assert np.all(np.isfinite(filt))


def test_on_resets_lane_on_jump():
  tr = LaneLineTracker(q=0.05, prob_alpha=0.3, gate_m=1.0)
  y = np.array([-5.4, -1.8, 1.8, 5.4])[:, None] * np.ones((4, 33)); s = np.full((4, 33), 0.4); p = np.full(4, 0.9)
  for _ in range(20):
    tr.update(y, s, p)
  y2 = y.copy(); y2[1] += 3.6  # lane index swap during a lane change
  fy, _, _ = tr.update(y2, s, p)
  np.testing.assert_allclose(fy[1], y2[1])
  assert np.abs(fy[2] - y[2]).max() < 1e-9


def test_on_changes_only_lane_lines():
  view = K9LRView()
  view.set_mode(1)
  view.apply(parsed_outputs(5))
  out = parsed_outputs(6)
  before = snapshot(out)
  new = view.apply(out)
  assert new is not out
  for name, arr in before.items():
    assert arr.tobytes() == np.asarray(out[name]).tobytes(), name  # input dict untouched
    if name not in ('lane_lines', 'lane_lines_stds', 'lane_lines_prob'):
      assert new[name] is out[name]
  np.testing.assert_array_equal(new['lane_lines'][..., 1], out['lane_lines'][..., 1])  # z untouched
  np.testing.assert_array_equal(new['lane_lines_prob'][0, 0::2], out['lane_lines_prob'][0, 0::2])
  modelv2_bytes(new)


def test_non_finite_falls_back_to_raw():
  view = K9LRView()
  view.set_mode(1)
  view.apply(parsed_outputs(1))
  bad = parsed_outputs(2)
  bad['lane_lines'][0, 1, 3, 0] = np.nan
  assert view.apply(bad) is bad
  good = parsed_outputs(3)
  assert np.all(np.isfinite(view.apply(good)['lane_lines']))


def test_switching_resets_state():
  view = K9LRView()
  view.set_mode(1)
  for k in range(5):
    view.apply(parsed_outputs(k))
  view.set_mode(0)
  view.set_mode(1)
  out = parsed_outputs(42)
  new = view.apply(out)
  # first frame after a fresh start equals the raw model output
  np.testing.assert_allclose(new['lane_lines'][0, :, :, 0], out['lane_lines'][0, :, :, 0])


def test_menu_and_param_registered_default_off():
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  driving = next(m for m in catalog["menu"] if m["id"] == "DRIVING")
  experimental = next(g for g in driving["groups"] if g["id"] == "EXPERIMENTAL")
  assert "K9LRView" in experimental["params"]
  entry = next(p for p in catalog["params"] if p["name"] == "K9LRView")
  assert (entry["min"], entry["max"], entry["default"]) == (0, 1, 0)
  assert entry["options"]["ko"][0].startswith("0")
  keys = (ROOT / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
  assert '{"K9LRView", {PERSISTENT, INT, "0"}}' in keys


def test_read_mode_never_raises():
  class Bad:
    def get_int(self, key):
      raise KeyError(key)
  class Good:
    def get_int(self, key):
      assert key == "K9LRView"
      return 1
  assert read_mode(Bad()) == 0
  assert read_mode(Good()) == 1
