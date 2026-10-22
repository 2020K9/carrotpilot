"""K9LRView (experimental, NOT vehicle-validated). One finished version: 0 off / 1 on (lane-line tracker).

(a) 0 / invalid numbers: model outputs pass through as the same object and the serialized modelV2 /
    drivingModelData are byte-identical to the parent path (no K9LRView call).
(b) 1: Kalman lane-line tracker with ego-motion prediction; bit-identical to a frozen written-out reference of the
    update; reduces frame-to-frame noise, keeps prob in [0, 1], resets on lane jumps > 1 m, follows a curve entry
    with less lag than the same filter without ego motion, falls back to raw outputs on non-finite input, and
    switching off and on restarts the state.
(c) stored values: 1, 2, 3 (earlier trial numbers) -> 1; everything else -> 0.
(d) menu / params registration: min 0, max 1, exactly two options, default 0.
"""
import json
from pathlib import Path

import numpy as np
import pytest

from cereal import messaging
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.modeld.fill_model_msg import fill_model_msg, fill_driving_model_data, PublishState
from openpilot.selfdrive.modeld import k9_lr_view as M
from openpilot.selfdrive.modeld.k9_lr_view import K9LRView, LaneLineTracker, normalize_mode, read_mode
from openpilot.selfdrive.modeld.parse_model_outputs import Parser

ROOT = Path(__file__).resolve().parents[4]
X = np.asarray(ModelConstants.X_IDXS, dtype=np.float64)
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
  fill_model_msg(msg, outputs, msg.modelV2.action, PublishState(), 10, 10, 10, 0.0, 123456789, 0.02, True)
  return msg.to_bytes()


def drivingdata_bytes(outputs):
  mv = messaging.new_message('modelV2')
  fill_model_msg(mv, outputs, mv.modelV2.action, PublishState(), 10, 10, 10, 0.0, 123456789, 0.02, True)
  dd = messaging.new_message('drivingModelData')
  dd.logMonoTime = 1
  fill_driving_model_data(dd, mv)
  return dd.to_bytes()


# ---------------------------------------------------------------- (a) 0 = off, bit-identical
def test_off_is_same_object_and_untouched():
  view = K9LRView()
  for k in range(30):
    out = parsed_outputs(k)
    before = snapshot(out)
    assert view.apply(out) is out
    for name, arr in before.items():
      assert arr.tobytes() == np.asarray(out[name]).tobytes(), name


def test_off_modelv2_bit_identical_over_sequence():
  view = K9LRView()
  view.set_mode(0)
  for k in range(20):
    a, b = parsed_outputs(100 + k), parsed_outputs(100 + k)
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
    assert drivingdata_bytes(a) == drivingdata_bytes(view.apply(b))


# ---------------------------------------------------------------- (c) stored values
@pytest.mark.parametrize("stored, expect", [(0, 0), (1, 1), (2, 1), (3, 1), (4, 0), (7, 0), (-1, 0), (None, 0), ('x', 0),
                                            ('0', 0), ('1', 1), ('3', 1), (2.0, 1), (2.5, 0), (float('nan'), 0), (True, 0)])
def test_stored_values_map_to_0_or_1(stored, expect):
  assert normalize_mode(stored) == expect
  view = K9LRView()
  view.set_mode(stored)
  assert view.mode == expect and (view.tracker is None) == (expect == 0)
  out = parsed_outputs(1)
  assert (view.apply(out) is out) == (expect == 0)


def test_read_mode_never_raises_and_maps_old_numbers():
  class Bad:
    def get_int(self, key):
      raise KeyError(key)
  class Stored:
    def __init__(self, v):
      self.v = v
    def get_int(self, key):
      assert key == "K9LRView"
      return self.v
  assert read_mode(Bad()) == 0
  assert [read_mode(Stored(v)) for v in (0, 1, 2, 3, 4, -1)] == [0, 1, 1, 1, 0, 0]


def test_only_two_values_in_code():
  assert (M.K9_LR_VIEW_OFF, M.K9_LR_VIEW_ON) == (0, 1)
  assert not hasattr(M, "PROFILES")
  src = (ROOT / "openpilot/selfdrive/modeld/k9_lr_view.py").read_text(encoding="utf-8")
  code = src.split('"""', 2)[2]  # history mention only in the module docstring
  assert "P1g" not in code and "P2m" not in code and "P3" not in code


# ---------------------------------------------------------------- (b) 1 = lane-line tracker
def _reference(frames):
  """1 written out (frozen 2026-10-22 lane_improve 'P3' update: ego-motion predict, q(x, yaw), innovation inflate,
  prob EMA 0.3, 1 m jump reset). Must stay bit-identical."""
  qx = 0.01 * (1.0 + 3.0 * X / 50.0)
  ye = P = pe = prev0 = None; outs = []
  for y, s, p, v, w in frames:
    y = np.asarray(y, dtype=np.float64); s = np.maximum(np.asarray(s, dtype=np.float64), 1e-3); p = np.asarray(p, dtype=np.float64)
    if ye is None:
      ye, P, pe = y.copy(), s ** 2, p.copy(); prev0 = ye[:, 0].copy(); outs.append((ye.copy(), pe.copy())); continue
    R = s ** 2
    vv = float(v) if np.isfinite(v) else 0.0
    ww = float(w) if np.isfinite(w) else 0.0
    xs = X + max(vv, 0.0) * 0.05
    ye = np.stack([np.interp(xs, X, ye[i]) for i in range(4)]) - (ww * 0.05) * xs[None, :]
    q = qx * (1.0 + 3.0 * abs(ww) / 0.1)
    Pp = P + q
    Pp = np.where(np.abs(y - ye) > 2.0 * np.sqrt(Pp + R), Pp + q * (10.0 - 1.0), Pp)
    K = Pp / (Pp + R)
    ye = ye + K * (y - ye); P = (1 - K) * Pp
    pe = 0.3 * p + (1 - 0.3) * pe
    j = np.abs(y[:, 0] - prev0) > 1.0; ye[j] = y[j]; P[j] = R[j]; pe[j] = p[j]
    prev0 = ye[:, 0].copy(); outs.append((ye.copy(), np.clip(pe, 0.0, 1.0)))
  return outs


def _lane_seq(n, rng, noise=0.15, prob_noise=0.25):
  base = np.array([-5.4, -1.8, 1.8, 5.4])[:, None] * np.ones((4, ModelConstants.IDX_N))
  for _ in range(n):
    y = base + rng.normal(0, noise, base.shape)
    std = np.full(base.shape, 0.4)
    p = np.clip(0.55 + rng.normal(0, prob_noise, 4), 0, 1)
    yield y, std, p


def _curve_entry(n, rng, v=20.0, noise=0.1):
  """ego drives along the lane centre into a curve whose curvature ramps 0 -> 1/250 m over 40 m (small-angle
  geometry). returns frames of (y, std, prob, v, yaw_rate, truth)"""
  off = np.array([-5.4, -1.8, 1.8, 5.4])[:, None]
  ds = 0.05
  s_grid = np.arange(0.0, v * 0.05 * n + 250.0, ds)
  kappa = np.clip((s_grid - 30.0) / 40.0, 0.0, 1.0) / 250.0
  theta = np.concatenate([[0.0], np.cumsum(kappa[:-1] * ds)])
  for t in range(n):
    i0 = int(round(v * 0.05 * t / ds))
    u = np.arange(0.0, 200.0, ds)
    dth = theta[i0 + (u / ds).astype(int)] - theta[i0]
    yc = np.concatenate([[0.0], np.cumsum(dth[:-1] * ds)])
    truth = off + np.interp(X, u, yc)[None, :]
    yield truth + rng.normal(0, noise, truth.shape), np.full(truth.shape, 0.4), np.full(4, 0.9), v, v * kappa[i0], truth


def test_on_bit_identical_to_frozen_reference():
  rng = np.random.default_rng(3)
  frames = [(y, s, rng.uniform(0.2, 0.95, 4), v, w) for y, s, _, v, w, _ in _curve_entry(140, rng)]
  frames += [(y, s, p, (np.nan if i % 9 == 4 else 25.0), (np.inf if i % 13 == 5 else 0.01)) for i, (y, s, p) in enumerate(_lane_seq(60, rng))]
  y2 = frames[-1][0].copy(); y2[1] += 3.6
  frames.append((y2, frames[-1][1], frames[-1][2], 20.0, 0.0))  # lane jump reset
  tr = LaneLineTracker()
  for f, (ry, rp) in zip(frames, _reference(frames), strict=True):
    fy, fs, fp = tr.update(*f)
    assert fy.tobytes() == ry.tobytes() and fp.tobytes() == rp.tobytes()
    assert fs.tobytes() == np.maximum(f[1], 1e-3).tobytes()  # raw std passed through


def test_on_reduces_jitter_and_keeps_prob_range():
  rng = np.random.default_rng(0)
  tr = LaneLineTracker()
  raw, filt, praw, probs = [], [], [], []
  for y, s, p in _lane_seq(400, rng):
    fy, fs, fp = tr.update(y, s, p, 25.0, 0.0)
    raw.append(y); filt.append(fy); praw.append(p); probs.append(fp)
  raw, filt, praw, probs = map(np.array, (raw, filt, praw, probs))
  assert np.abs(np.diff(filt[:, :, :12], axis=0)).mean() < 0.6 * np.abs(np.diff(raw[:, :, :12], axis=0)).mean()
  assert np.abs(np.diff(probs, axis=0)).mean() < 0.6 * np.abs(np.diff(praw, axis=0)).mean()
  assert probs.min() >= 0 and probs.max() <= 1
  assert np.all(np.isfinite(filt))


def test_on_resets_lane_on_jump():
  tr = LaneLineTracker()
  y = np.array([-5.4, -1.8, 1.8, 5.4])[:, None] * np.ones((4, 33)); s = np.full((4, 33), 0.4); p = np.full(4, 0.9)
  for _ in range(20):
    tr.update(y, s, p, 0.0, 0.0)
  y2 = y.copy(); y2[1] += 3.6  # lane index swap during a lane change
  p2 = p.copy(); p2[1] = 0.2
  fy, _, fp = tr.update(y2, s, p2, 0.0, 0.0)
  np.testing.assert_allclose(fy[1], y2[1])
  assert fp[1] == 0.2
  assert np.abs(fy[2] - y[2]).max() < 1e-9


def test_on_follows_curve_entry_with_less_lag_than_without_ego_motion():
  rng = np.random.default_rng(1)
  frames = list(_curve_entry(140, rng))
  tr = LaneLineTracker()
  e_on, e_static, ye, P = [], [], None, None
  for y, s, p, v, w, truth in frames:
    fy, _, _ = tr.update(y, s, p, v, w)
    e_on.append(np.abs(fy[1:3, 8:20] - truth[1:3, 8:20]).mean())
    # the same Kalman filter without the ego-motion step (plain per-point smoothing)
    if ye is None:
      ye, P = y.copy(), s ** 2
    else:
      Pp = P + 0.05; K = Pp / (Pp + s ** 2); ye = ye + K * (y - ye); P = (1 - K) * Pp
    e_static.append(np.abs(ye[1:3, 8:20] - truth[1:3, 8:20]).mean())
  assert np.mean(e_on[20:120]) < 0.8 * np.mean(e_static[20:120])


def test_on_handles_bad_motion_inputs():
  rng = np.random.default_rng(2)
  tr = LaneLineTracker()
  for i, (y, s, p) in enumerate(_lane_seq(300, rng)):
    v, w = (np.nan, np.inf) if i % 50 == 7 else (25.0, 0.0)
    fy, fs, fp = tr.update(y, s, p, v, w)
    assert np.all(np.isfinite(fy)) and np.all(np.isfinite(fs)) and 0 <= fp.min() and fp.max() <= 1


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
    if name not in ('lane_lines', 'lane_lines_prob'):
      assert new[name] is out[name]
  np.testing.assert_array_equal(new['lane_lines'][..., 1], out['lane_lines'][..., 1])  # z untouched
  assert new['lane_lines_stds'] is out['lane_lines_stds']  # stds passed through (planner weighting unchanged)
  np.testing.assert_array_equal(new['lane_lines_prob'][0, 0::2], out['lane_lines_prob'][0, 0::2])
  modelv2_bytes(new); drivingdata_bytes(new)


def test_non_finite_falls_back_to_raw():
  view = K9LRView()
  view.set_mode(1)
  view.apply(parsed_outputs(1))
  bad = parsed_outputs(2)
  bad['lane_lines'][0, 1, 3, 0] = np.nan
  assert view.apply(bad) is bad
  good = parsed_outputs(3)
  assert np.all(np.isfinite(view.apply(good)['lane_lines']))


def test_switching_off_and_on_restarts_state_and_old_numbers_do_not():
  view = K9LRView()
  view.set_mode(1)
  for k in range(5):
    view.apply(parsed_outputs(k))
  tracker = view.tracker
  view.set_mode(3)  # stored old number: still 1, same tracker
  assert view.mode == 1 and view.tracker is tracker
  view.set_mode(0)
  view.set_mode(1)
  out = parsed_outputs(42)
  new = view.apply(out)
  np.testing.assert_allclose(new['lane_lines'][0, :, :, 0], out['lane_lines'][0, :, :, 0])  # fresh start = raw


# ---------------------------------------------------------------- (d) menu / params
def test_menu_and_param_registered_0_1_default_off():
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  driving = next(m for m in catalog["menu"] if m["id"] == "DRIVING")
  experimental = next(g for g in driving["groups"] if g["id"] == "EXPERIMENTAL")
  assert "K9LRView" in experimental["params"]
  entry = next(p for p in catalog["params"] if p["name"] == "K9LRView")
  assert (entry["min"], entry["max"], entry["default"]) == (0, 1, 0)
  assert [len(entry["options"][lang]) for lang in ("ko", "en", "zh")] == [2, 2, 2]
  for lang in ("ko", "en", "zh"):
    assert [o.split(" ", 1)[0] for o in entry["options"][lang]] == ["0", "1"]
  for key in ("descr", "edescr", "cdescr"):
    text = entry[key]
    assert "0" in text and "1" in text
    for gone in ("P1g", "P2m", "P3", "2(", "2 (", "2（", "3(", "3 (", "3（", "2:", "3:", "2：", "3："):
      assert gone not in text, (key, gone)
  for lang in ("ko", "en"):
    doc = (ROOT / f"docs/user/{lang}/settings.md").read_text(encoding="utf-8")
    row = next(line for line in doc.splitlines() if "| `K9LRView` |" in line)
    assert "`0`" in row and "`1`" in row and "`2`" not in row and "`3`" not in row and "P1g" not in row and "P2m" not in row
  keys = (ROOT / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
  assert '{"K9LRView", {PERSISTENT, INT, "0"}}' in keys
