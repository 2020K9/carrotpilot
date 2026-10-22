"""K9LRView (experimental menu number, NOT vehicle-validated).

(a) 0 / unknown numbers: model outputs pass through as the same object and the serialized modelV2 is
    byte-identical to the parent path (no K9LRView call).
(b) 1 (P1g): Kalman lane-line tracker reduces frame-to-frame noise, keeps prob in [0, 1], resets on
    lane jumps > 1 m, falls back to raw outputs on non-finite input, and switching the number resets state.
(c) menu / params registration (default 0).
(d) 2 (P2m): ego-motion predicted tracker; 1 stays numerically unchanged; OFF drivingModelData byte-identical.
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
  for mode in (0, 3, 7, -1, None, 'x', '0'):
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
  assert new['lane_lines_stds'] is out['lane_lines_stds']  # stds passed through (planner weighting unchanged)
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
  assert (entry["min"], entry["max"], entry["default"]) == (0, 2, 0)
  assert [len(entry["options"][lang]) for lang in ("ko", "en", "zh")] == [3, 3, 3]
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


# ---------------------------------------------------------------- 2 (P2m): ego-motion predicted tracker
def test_p2m_profile_and_p1g_unchanged():
  from openpilot.selfdrive.modeld.k9_lr_view import PROFILES
  assert PROFILES[1] == dict(q=0.05, prob_alpha=0.3, gate_m=1.0)
  assert PROFILES[2]['ego_predict'] is True


def _p1g_reference(seq):
  """P1g update written out (0dc9b766 y filter + prob restart on a jump), to check 1 is unchanged by the P2m refactor"""
  ye = P = pe = prev0 = None; outs = []
  for y, s, p in seq:
    s = np.maximum(s, 1e-3)
    if ye is None:
      ye, P, pe = y.copy(), s ** 2, p.copy(); prev0 = ye[:, 0].copy(); outs.append(ye.copy()); continue
    R = s ** 2; Pp = P + 0.05; K = Pp / (Pp + R)
    ye = ye + K * (y - ye); P = (1 - K) * Pp
    pe = 0.3 * p + 0.7 * pe
    j = np.abs(y[:, 0] - prev0) > 1.0; ye[j] = y[j]; P[j] = R[j]; pe[j] = p[j]
    prev0 = ye[:, 0].copy(); outs.append(ye.copy())
  return outs


def test_p1g_numerically_unchanged():
  seq = list(_lane_seq(200, np.random.default_rng(3)))
  tr = LaneLineTracker(**__import__('openpilot.selfdrive.modeld.k9_lr_view', fromlist=['PROFILES']).PROFILES[1])
  ref = _p1g_reference(seq)
  for (y, s, p), r in zip(seq, ref):
    fy, _, _ = tr.update(y, s, p, 30.0, 0.2)  # v / yaw rate must be ignored by 1
    assert fy.tobytes() == r.tobytes()


def _curve_entry(n, rng, v=20.0, noise=0.1):
  """ego drives along the lane centre into a curve whose curvature ramps 0 -> 1/250 m over 40 m (small-angle
  geometry). In the ego frame the lane lines bend more each frame and the yaw rate is v * curvature at the car.
  returns frames of (y, std, prob, v, yaw_rate, truth)"""
  x = np.asarray(ModelConstants.X_IDXS)
  off = np.array([-5.4, -1.8, 1.8, 5.4])[:, None]
  ds = 0.05
  s_grid = np.arange(0.0, v * 0.05 * n + 250.0, ds)
  kappa = np.clip((s_grid - 30.0) / 40.0, 0.0, 1.0) / 250.0
  theta = np.concatenate([[0.0], np.cumsum(kappa[:-1] * ds)])
  for t in range(n):
    se = v * 0.05 * t
    i0 = int(round(se / ds))
    u = np.arange(0.0, 200.0, ds)
    dth = theta[i0 + (u / ds).astype(int)] - theta[i0]
    yc = np.concatenate([[0.0], np.cumsum(dth[:-1] * ds)])
    truth = off + np.interp(x, u, yc)[None, :]
    yield truth + rng.normal(0, noise, truth.shape), np.full(truth.shape, 0.4), np.full(4, 0.9), v, v * kappa[i0], truth


def test_p2m_follows_curve_entry_with_less_lag_than_p1g():
  rng = np.random.default_rng(1)
  frames = list(_curve_entry(140, rng))
  err = {}
  for mode in (1, 2):
    from openpilot.selfdrive.modeld.k9_lr_view import PROFILES
    tr = LaneLineTracker(**PROFILES[mode]); e = []
    for y, s, p, v, w, truth in frames:
      fy, _, _ = tr.update(y, s, p, v, w)
      e.append(np.abs(fy[1:3, 8:20] - truth[1:3, 8:20]).mean())
    err[mode] = np.mean(e[20:120])
  assert err[2] < 0.8 * err[1], err


def test_p2m_reduces_jitter_and_handles_bad_motion_inputs():
  from openpilot.selfdrive.modeld.k9_lr_view import PROFILES
  rng = np.random.default_rng(2)
  tr = LaneLineTracker(**PROFILES[2])
  raw, filt = [], []
  for i, (y, s, p) in enumerate(_lane_seq(300, rng)):
    v, w = (np.nan, np.inf) if i % 50 == 7 else (25.0, 0.0)
    fy, fs, fp = tr.update(y, s, p, v, w)
    assert np.all(np.isfinite(fy)) and np.all(np.isfinite(fs)) and 0 <= fp.min() and fp.max() <= 1
    raw.append(y); filt.append(fy)
  raw, filt = np.array(raw), np.array(filt)
  assert np.abs(np.diff(filt[:, :, :12], axis=0)).mean() < 0.6 * np.abs(np.diff(raw[:, :, :12], axis=0)).mean()


def test_p2m_view_uses_plan_motion_and_changes_only_lane_lines():
  view = K9LRView()
  view.set_mode(2)
  assert view.mode == 2
  view.apply(parsed_outputs(5))
  out = parsed_outputs(6)
  before = snapshot(out)
  new = view.apply(out)
  assert new is not out
  for name, arr in before.items():
    assert arr.tobytes() == np.asarray(out[name]).tobytes(), name
    if name not in ('lane_lines', 'lane_lines_prob'):
      assert new[name] is out[name]
  modelv2_bytes(new)
  view.set_mode(0)
  a, b = parsed_outputs(300), parsed_outputs(300)
  assert modelv2_bytes(a) == modelv2_bytes(view.apply(b))


def drivingdata_bytes(outputs):
  from openpilot.selfdrive.modeld.fill_model_msg import fill_driving_model_data
  mv = messaging.new_message('modelV2')
  fill_model_msg(mv, outputs, mv.modelV2.action, PublishState(), 10, 10, 10, 0.0, 123456789, 0.02, True)
  dd = messaging.new_message('drivingModelData')
  dd.logMonoTime = 1
  fill_driving_model_data(dd, mv)
  return dd.to_bytes()


def test_off_driving_model_data_bit_identical():
  view = K9LRView()
  for mode in (1, 2, 0):
    view.set_mode(mode)
    for k in range(5):
      view.apply(parsed_outputs(400 + k))
  assert view.mode == 0
  for k in range(10):
    a, b = parsed_outputs(500 + k), parsed_outputs(500 + k)
    assert drivingdata_bytes(a) == drivingdata_bytes(view.apply(b))
