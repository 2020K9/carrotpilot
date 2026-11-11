"""K9LRView (experimental, NOT vehicle-validated). One finished version: 0 off / 1 on (lane-line tracker).

(a) 0 / invalid numbers: model outputs pass through as the same object and the serialized modelV2 /
    drivingModelData are byte-identical to the parent path (no K9LRView call).
(b) 1: Kalman lane-line tracker with ego-motion prediction and long-dropout follow (raw prob < 0.5 for more than
    5 frames -> 10 x process noise for that line); bit-identical to a frozen written-out reference of the
    update and to the previous tracker (P3) when there is no long dropout; follows the model in long dropouts; reduces frame-to-frame noise, keeps prob in [0, 1], resets on lane jumps > 1 m, follows a curve entry
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
from openpilot.selfdrive.modeld.k9_lr_view import K9LRView, LaneLineTracker, normalize_mode, read_mode, HANDBACK_N
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
      assert key == "K9LaneAssist"   # 2026-10-25 lane menu merge (old K9LRView 1/2/3 -> K9LaneAssist 1 by k9_lane.migrate_k9_lane)
      return self.v
  assert read_mode(Bad()) == 0
  assert [read_mode(Stored(v)) for v in (0, 1, 2, 3, 4, -1)] == [0, 1, 0, 0, 0, 0]
  assert [normalize_mode(v) for v in (0, 1, 2, 3, 4, -1)] == [0, 1, 1, 1, 0, 0]


def test_only_two_values_in_code():
  assert (M.K9_LR_VIEW_OFF, M.K9_LR_VIEW_ON) == (0, 1)
  assert not hasattr(M, "PROFILES")
  src = (ROOT / "openpilot/selfdrive/modeld/k9_lr_view.py").read_text(encoding="utf-8")
  code = src.split('"""', 2)[2]  # history mention only in the module docstring
  assert "P1g" not in code and "P2m" not in code and "P3" not in code


# ---------------------------------------------------------------- (b) 1 = lane-line tracker
def _reference_y(frames, low_n, low_m):
  """y filter written out: ego-motion predict, q(x, yaw), innovation inflate, 1 m jump reset; low_n > 0: a line whose
  raw prob has been < 0.5 for more than low_n frames in a row gets low_m x process noise. Returns (y, jump) per frame."""
  qx = 0.01 * (1.0 + 3.0 * X / 50.0)
  ye = P = prev0 = None; outs = []; lowc = np.zeros(4, dtype=np.int64)
  res = _reference_res(frames)  # pin rewritten 2026-10-23 lrview_r4: + the 3 cm residual gate
  mark = np.zeros(4, dtype=bool)  # pin rewritten 2026-10-23 lrview_r6: + the hand-back to the rule-free filter
  rmp = np.zeros(4, dtype=np.int64)  # pin rewritten 2026-10-23 lrview_r7: hand-back spread over 6 frames
  for t, (y, s, p, v, w) in enumerate(frames):
    y = np.asarray(y, dtype=np.float64); s = np.maximum(np.asarray(s, dtype=np.float64), 1e-3); p = np.asarray(p, dtype=np.float64)
    if ye is None:
      ye, P = y.copy(), s ** 2; prev0 = ye[:, 0].copy(); outs.append((ye.copy(), None))
      y3, P3 = y.copy(), s ** 2; prev3 = y3[:, 0].copy(); continue
    R = s ** 2
    vv = float(v) if np.isfinite(v) else 0.0
    ww = float(w) if np.isfinite(w) else 0.0
    xs = X + max(vv, 0.0) * 0.05
    q0 = np.repeat((qx * (1.0 + 3.0 * abs(ww) / 0.1))[None, :], 4, axis=0)
    y3 = np.stack([np.interp(xs, X, y3[i]) for i in range(4)]) - (ww * 0.05) * xs[None, :]   # rule-free filter
    Pq = P3 + q0
    Pq = np.where(np.abs(y - y3) > 2.0 * np.sqrt(Pq + R), Pq + q0 * (10.0 - 1.0), Pq)
    K3 = Pq / (Pq + R); y3 = y3 + K3 * (y - y3); P3 = (1 - K3) * Pq
    j3 = np.abs(y[:, 0] - prev3) > 1.0; y3[j3] = y[j3]; P3[j3] = R[j3]; prev3 = y3[:, 0].copy()
    ye = np.stack([np.interp(xs, X, ye[i]) for i in range(4)]) - (ww * 0.05) * xs[None, :]
    q = q0.copy()
    if low_n:
      lowc = np.where(p < 0.5, lowc + 1, 0)
      low = (lowc > low_n) & (np.abs(res[t]) > 0.03)
      q[low] *= low_m; mark |= low
    Pp = P + q
    Pp = np.where(np.abs(y - ye) > 2.0 * np.sqrt(Pp + R), Pp + q * (10.0 - 1.0), Pp)
    K = Pp / (Pp + R)
    ye = ye + K * (y - ye); P = (1 - K) * Pp
    j = np.abs(y[:, 0] - prev0) > 1.0; ye[j] = y[j]; P[j] = R[j]
    if low_n:
      rmp[(rmp > 0) & low] = 0
    st = mark & (p >= 0.5) & (rmp == 0); rmp[st] = 6; mark[st] = False
    a = rmp > 0
    ye[a] = y3[a] + (ye[a] - y3[a]) * ((rmp[a] - 1) / rmp[a])[:, None]; P[a] = P3[a]; rmp[a] -= 1
    prev0 = ye[:, 0].copy(); outs.append((ye.copy(), j))
  return outs


def _reference_res(frames):
  """new 2026-10-23 lrview_r4: per frame EMA 0.3 of (raw x=0 - the previous (99eb53f7) filter x=0), starting at 0; the
  long-dropout rule acts on a line only while |this| > 0.03 m (the raw line really keeps moving away)."""
  r = np.zeros(4); out = [r.copy()]
  for (y, *_), (py, _) in list(zip(frames, _reference_p3(frames), strict=True))[1:]:
    r = 0.3 * (np.asarray(y, dtype=np.float64)[:, 0] - py[:, 0]) + 0.7 * r; out.append(r.copy())
  return out


def _reference(frames):
  """1 written out (frozen 2026-10-23 lrview_r2): y = the y filter with the long-dropout rule (raw prob < 0.5 for more
  than 3 frames in a row -> 5 x process noise); prob = EMA 0.3 that restarts on the jumps of the y filter WITHOUT
  that rule, i.e. exactly the previous (99eb53f7) probabilities. Must stay bit-identical."""
  ys = _reference_y(frames, 10, 5.0)  # pin rewritten 2026-10-23 lrview_r10: the long-dropout rule starts after 10 frames (was 3)
  return [(y, rp) for (y, _), (_, rp) in zip(ys, _reference_p3(frames), strict=True)]


def _reference_p3(frames):
  """The previous 1 (99eb53f7, lane_improve 'P3': ego-motion predict, q(x, yaw), innovation inflate, prob EMA 0.3,
  1 m jump reset), written out. The current 1 must equal it whenever no raw prob stays < 0.5 for more than 5 frames."""
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


def _dropout_frames(rng, n=260, drop=((40, 43), (60, 65), (100, 200)), line=2):
  """straight lane, model noise 5 cm; line `line` has raw prob 0.2 and model std 1.0 inside each (start, end) frame
  window (what the model outputs for a faint line). From frame 100 on that line really moves 1 cm per frame for 60
  frames (the car drifts sideways; the ego-motion prediction only has forward speed and yaw rate)."""
  base = np.array([-5.4, -1.8, 1.8, 5.4])[:, None] * np.ones((4, ModelConstants.IDX_N))
  out = []
  for t in range(n):
    truth = base.copy(); truth[line] += 0.01 * min(max(t - 100, 0), 60)
    y = truth + rng.normal(0, 0.05, base.shape)
    s = np.full(base.shape, 0.4); p = np.full(4, 0.9)
    if any(a <= t < b for a, b in drop):
      p[line] = 0.2; s[line] = 1.0
    out.append((y, s, p, 20.0, 0.0))
  return out


def test_on_long_dropout_bit_identical_to_frozen_reference():
  # new pin (2026-10-22 lrview_night_r1): the long-dropout path written out, incl. a dropout on every line
  rng = np.random.default_rng(11)
  frames = _dropout_frames(rng)
  frames += [(y, s, np.where(np.arange(4) == (i // 20) % 4, 0.1, p), v, w) for i, (y, s, p, v, w) in enumerate(_dropout_frames(rng, 120, ()))]
  tr = LaneLineTracker()
  for f, (ry, rp) in zip(frames, _reference(frames), strict=True):
    fy, _, fp = tr.update(*f)
    assert fy.tobytes() == ry.tobytes() and fp.tobytes() == rp.tobytes()


def test_on_equals_previous_tracker_without_long_dropouts():
  # dropouts of up to 3 frames (and none) are filtered exactly like the previous 1 (P3, 99eb53f7)
  # (pin rewritten 2026-10-23 lrview_r2: the long-dropout rule now starts after 3 frames instead of 5)
  rng = np.random.default_rng(12)
  frames = _dropout_frames(rng, drop=((40, 43), (60, 63), (100, 103), (200, 203)))
  tr = LaneLineTracker()
  for f, (ry, rp) in zip(frames, _reference_p3(frames), strict=True):
    fy, _, fp = tr.update(*f)
    assert fy.tobytes() == ry.tobytes() and fp.tobytes() == rp.tobytes()


def test_on_long_dropout_follows_model_and_short_dropout_keeps_smoothing():
  rng = np.random.default_rng(13)
  frames = _dropout_frames(rng)
  tr = LaneLineTracker()
  new = np.array([tr.update(*f)[0] for f in frames])
  old = np.array([ry for ry, _ in _reference_p3(frames)])
  raw = np.array([f[0] for f in frames])
  truth = np.array([1.8 + 0.01 * min(max(t - 100, 0), 60) for t in range(len(frames))])
  pts = slice(4, 14)  # x = 3 .. 32 m
  # 1.5-5 s into the long dropout: much closer to where the line really is than the previous tracker
  e_new = np.abs(new[130:200, 2, pts] - truth[130:200, None]).mean()
  e_old = np.abs(old[130:200, 2, pts] - truth[130:200, None]).mean()
  assert e_new < 0.7 * e_old
  # still filtered inside the long dropout (not a pass-through of the raw model)
  j_new = np.abs(np.diff(new[130:200, 2, pts], axis=0)).mean()
  j_raw = np.abs(np.diff(raw[130:200, 2, pts], axis=0)).mean()
  assert j_new < 0.5 * j_raw
  # short dropouts are bridged exactly as before: the 3-frame one and the first 3 frames of the 5-frame one
  # (pin rewritten 2026-10-23 lrview_r2: was new[:100], the rule now starts after 3 dropout frames instead of 5)
  assert new[:63].tobytes() == old[:63].tobytes()


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
  # 2026-10-25 lane menu merge (pin rewritten deliberately): the tracker is part of the single 0/1 switch K9LaneAssist;
  # the old K9LRView key stays registered (migration only, no effect) but is no longer in the menu
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  driving = next(m for m in catalog["menu"] if m["id"] == "DRIVING")
  experimental = next(g for g in driving["groups"] if g["id"] == "EXPERIMENTAL")
  assert "K9LaneAssist" in experimental["params"] and "K9LRView" not in experimental["params"]
  assert not any(p["name"] == "K9LRView" for p in catalog["params"])
  entry = next(p for p in catalog["params"] if p["name"] == "K9LaneAssist")
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
    row = next(line for line in doc.splitlines() if "| `K9LaneAssist` |" in line)
    assert "`0`" in row and "`1`" in row and "`2`" not in row and "`3`" not in row and "P1g" not in row and "P2m" not in row
  keys = (ROOT / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
  assert '{"K9LRView", {PERSISTENT, INT, "0"}}' in keys and '{"K9LaneAssist", {PERSISTENT, INT, "0"}}' in keys


def test_on_prob_always_bit_identical_to_previous_tracker():
  # new 2026-10-23 lrview_r2: the long-dropout rule never moves a published probability, also across jump resets
  rng = np.random.default_rng(21)
  frames = _dropout_frames(rng)
  frames += [(y, s, np.where(np.arange(4) == (i // 20) % 4, 0.1, p), v, w) for i, (y, s, p, v, w) in enumerate(_dropout_frames(rng, 120, ()))]
  y2 = frames[-1][0].copy(); y2[2] += 3.6; p2 = frames[-1][2].copy(); p2[2] = 0.2
  frames += [(y2 + 0.01 * k, frames[-1][1], p2, 20.0, 0.0) for k in range(30)]  # jump while the line is in a dropout
  tr = LaneLineTracker()
  for f, (_, rp) in zip(frames, _reference_p3(frames), strict=True):
    assert tr.update(*f)[2].tobytes() == rp.tobytes()


def test_on_long_dropout_starts_after_ten_frames():
  # new 2026-10-23 lrview_r2: a dropout differs from the previous tracker only after LOW_N frames
  # (pin rewritten 2026-10-23 lrview_r4: the dropout lies where the line really moves (the line moves 1 cm/frame from
  # frame 100), since the rule only acts while the raw line keeps leaving the previous filter)
  # (pin rewritten 2026-10-23 lrview_r10: was test_on_long_dropout_starts_after_three_frames with a 4-frame dropout at
  # 130-134; the rule now starts after 10 frames, so an 11-frame dropout at 130-141 differs from its 11th frame)
  rng = np.random.default_rng(22)
  frames = _dropout_frames(rng, n=200, drop=((130, 141),))
  tr = LaneLineTracker()
  new = np.array([tr.update(*f)[0] for f in frames])
  old = np.array([ry for ry, _ in _reference_p3(frames)])
  assert new[:140].tobytes() == old[:140].tobytes()
  assert not np.array_equal(new[140, 2], old[140, 2])



# ---------------------------------------------------------------- new 2026-10-23 lrview_r3: near-car and 4.4 m width pins
def _reference_full(frames):
  """1 written out in full (frozen 2026-10-23 lrview_r3), incl. the two rules the pins above never reach:
  - near-car: a line whose PREDICTED (after ego motion) x=0 point is within 0.5 m of the car gets 2 x process noise;
  - width: the long-dropout rule (raw prob < 0.5 for more than 10 frames (3 before lrview_r10) -> 5 x) is off for every line in a frame whose
    predicted near-lane width y[2, 0] - y[1, 0] is above 4.4 m.
  prob = the previous (99eb53f7) probabilities. Must stay bit-identical. Returns (y, prob) per frame."""
  qx = 0.01 * (1.0 + 3.0 * X / 50.0)
  ye = P = prev0 = None
  ys = []
  lowc = np.zeros(4, dtype=np.int64)
  res = _reference_res(frames)  # pin rewritten 2026-10-23 lrview_r4: + the 3 cm residual gate
  mark = np.zeros(4, dtype=bool)  # pin rewritten 2026-10-23 lrview_r6: near-car rule also behind the 3 cm residual gate, + hand-back
  rmp = np.zeros(4, dtype=np.int64)  # pin rewritten 2026-10-23 lrview_r7: hand-back spread over 6 frames
  for t, (y, s, p, v, w) in enumerate(frames):
    y = np.asarray(y, dtype=np.float64); s = np.maximum(np.asarray(s, dtype=np.float64), 1e-3); p = np.asarray(p, dtype=np.float64)
    if ye is None:
      ye, P = y.copy(), s ** 2; prev0 = ye[:, 0].copy(); ys.append(ye.copy())
      y3, P3 = y.copy(), s ** 2; prev3 = y3[:, 0].copy(); continue
    R = s ** 2
    vv = float(v) if np.isfinite(v) else 0.0
    ww = float(w) if np.isfinite(w) else 0.0
    xs = X + max(vv, 0.0) * 0.05
    q0 = np.repeat((qx * (1.0 + 3.0 * abs(ww) / 0.1))[None, :], 4, axis=0)
    y3 = np.stack([np.interp(xs, X, y3[i]) for i in range(4)]) - (ww * 0.05) * xs[None, :]   # rule-free filter
    Pq = P3 + q0
    Pq = np.where(np.abs(y - y3) > 2.0 * np.sqrt(Pq + R), Pq + q0 * (10.0 - 1.0), Pq)
    K3 = Pq / (Pq + R); y3 = y3 + K3 * (y - y3); P3 = (1 - K3) * Pq
    j3 = np.abs(y[:, 0] - prev3) > 1.0; y3[j3] = y[j3]; P3[j3] = R[j3]; prev3 = y3[:, 0].copy()
    ye = np.stack([np.interp(xs, X, ye[i]) for i in range(4)]) - (ww * 0.05) * xs[None, :]
    q = q0.copy()
    lowc = np.where(p < 0.5, lowc + 1, 0)
    low = (lowc > 10) & (np.abs(res[t]) > 0.03)  # pin rewritten 2026-10-23 lrview_r10: after 10 frames (was 3)
    if (ye[2, 0] - ye[1, 0]) > 4.4:
      low = np.zeros_like(low)
    q[low] *= 5.0
    near = (np.abs(ye[:, 0]) < 0.5) & (np.abs(res[t]) > 0.03)
    q[near] *= 2.0
    mark |= low | near
    Pp = P + q
    Pp = np.where(np.abs(y - ye) > 2.0 * np.sqrt(Pp + R), Pp + q * (10.0 - 1.0), Pp)
    K = Pp / (Pp + R)
    ye = ye + K * (y - ye); P = (1 - K) * Pp
    j = np.abs(y[:, 0] - prev0) > 1.0; ye[j] = y[j]; P[j] = R[j]
    rmp[(rmp > 0) & (low | near)] = 0
    st = mark & (p >= 0.5) & ~near & (rmp == 0); rmp[st] = 6; mark[st] = False
    a = rmp > 0
    ye[a] = y3[a] + (ye[a] - y3[a]) * ((rmp[a] - 1) / rmp[a])[:, None]; P[a] = P3[a]; rmp[a] -= 1
    prev0 = ye[:, 0].copy(); ys.append(ye.copy())
  return [(y, rp) for y, (_, rp) in zip(ys, _reference_p3(frames), strict=True)]


def _crossing_frames(rng, n=200):
  """the car drifts left across line 1 and back (all lines move +2 cm/frame for 90 frames, then -2 cm/frame), so line 1
  (from -1.8 m) passes the x=0 point of the car; with a long dropout on line 2 and a short one on line 1."""
  base = np.array([-5.4, -1.8, 1.8, 5.4])[:, None] * np.ones((4, ModelConstants.IDX_N))
  out = []
  for t in range(n):
    sh = 0.02 * (t if t < 90 else max(180 - t, 0))
    y = base + sh + rng.normal(0, 0.05, base.shape)
    s = np.full(base.shape, 0.4); p = np.full(4, 0.9)
    if 30 <= t < 70:
      p[2] = 0.2; s[2] = 1.0
    if 80 <= t < 83:
      p[1] = 0.3
    out.append((y, s, p, 20.0, 0.0))
  return out


def _wide_frames(rng, n=200, half=2.4, drop=((30, 120),), line=2):
  """straight lane of width 2*half m (merge / wide lane) with a long dropout on one line that also moves 0.5 cm/frame."""
  base = np.array([-3 * half, -half, half, 3 * half])[:, None] * np.ones((4, ModelConstants.IDX_N))
  out = []
  for t in range(n):
    y = base + rng.normal(0, 0.05, base.shape)
    y[line] += 0.005 * min(max(t - 40, 0), 60)
    s = np.full(base.shape, 0.4); p = np.full(4, 0.9)
    if any(a <= t < b for a, b in drop):
      p[line] = 0.2; s[line] = 1.0
    out.append((y, s, p, 20.0, 0.0))
  return out


def test_on_near_car_rule_bit_identical_to_frozen_reference():
  rng = np.random.default_rng(31)
  frames = _crossing_frames(rng)
  tr = LaneLineTracker()
  got = []
  for f, (ry, rp) in zip(frames, _reference_full(frames), strict=True):
    fy, _, fp = tr.update(*f)
    assert fy.tobytes() == ry.tobytes() and fp.tobytes() == rp.tobytes()
    got.append(fy)
  # the near-car rule really acts in this scene: the r2 reference without it differs only on line 1 (the one under the car)
  no_near = np.array([ry for ry, _ in _reference(frames)])
  got = np.array(got)
  assert np.array_equal(got[:, 0], no_near[:, 0]) and np.array_equal(got[:, 2], no_near[:, 2]) and np.array_equal(got[:, 3], no_near[:, 3])
  assert not np.array_equal(got[:, 1], no_near[:, 1])


def test_on_wide_lane_rule_bit_identical_to_frozen_reference():
  rng = np.random.default_rng(32)
  frames = _wide_frames(rng)                    # 4.8 m: the long-dropout rule stays off
  frames += _wide_frames(rng, half=2.1)[1:]     # 4.2 m: it acts again
  tr = LaneLineTracker()
  got = []
  for f, (ry, rp) in zip(frames, _reference_full(frames), strict=True):
    fy, _, fp = tr.update(*f)
    assert fy.tobytes() == ry.tobytes() and fp.tobytes() == rp.tobytes()
    got.append(fy)
  got = np.array(got)
  p3 = np.array([ry for ry, _ in _reference_p3(frames)])
  assert got[:200].tobytes() == p3[:200].tobytes()      # wide lane: exactly the previous tracker, even in a 4.5 s dropout
  assert not np.array_equal(got[200:, 2], p3[200:, 2])  # normal width: the long-dropout rule follows the model again


def test_on_full_reference_matches_the_earlier_pin_scenes():
  # the full written-out reference also reproduces the earlier frozen scenes (where neither new rule acts)
  rng = np.random.default_rng(11)
  frames = _dropout_frames(rng)
  for (ay, ap), (by, bp) in zip(_reference_full(frames), _reference(frames), strict=True):
    assert ay.tobytes() == by.tobytes() and ap.tobytes() == bp.tobytes()


# ---------------------------------------------------------------- new 2026-10-23 lrview_r6: hand-back and near-car residual gate
def test_on_hand_back_equals_previous_tracker_after_a_long_dropout():
  # once the raw prob is back after a long dropout the line is exactly the previous tracker's line again
  # (and the dropout itself still follows the model: it differs from the previous tracker inside the dropout)
  # (pin rewritten 2026-10-23 lrview_r7: the hand-back takes HANDBACK_N frames, exact from the last one on)
  rng = np.random.default_rng(41)
  frames = _dropout_frames(rng, n=260, drop=((100, 200),))
  tr = LaneLineTracker()
  new = np.array([tr.update(*f)[0] for f in frames])
  old = np.array([ry for ry, _ in _reference_p3(frames)])
  assert not np.array_equal(new[130:200, 2], old[130:200, 2])
  n = HANDBACK_N
  assert new[200 + n - 1:].tobytes() == old[200 + n - 1:].tobytes()
  assert not np.array_equal(new[200:200 + n - 1, 2], old[200:200 + n - 1, 2])


def test_on_near_car_rule_off_for_a_line_that_stays_put():
  # a line that sits under the car but does not move away from the rule-free filter (residual EMA <= 3 cm) gets no
  # near-car rule: the tracker is bit-identical to the previous one
  rng = np.random.default_rng(42)
  base = np.array([-3.6, -0.3, 3.3, 6.9])[:, None] * np.ones((4, ModelConstants.IDX_N))
  frames = [(base + rng.normal(0, 0.01, base.shape), np.full(base.shape, 0.4), np.full(4, 0.9), 20.0, 0.0) for _ in range(150)]
  tr = LaneLineTracker()
  new = np.array([tr.update(*f)[0] for f in frames])
  old = np.array([ry for ry, _ in _reference_p3(frames)])
  assert new.tobytes() == old.tobytes()


# ---------------------------------------------------------------- new 2026-10-23 lrview_r7: hand-back over several frames
def test_on_hand_back_ramp_shrinks_the_difference_step_by_step():
  # after a long dropout (line still moving when the raw prob comes back) the gap to the previous tracker closes over
  # HANDBACK_N frames instead of in one jump: frame k of the hand-back keeps at most (n - k) / n of the first gap
  # (+ 5 mm for the innovation inflation of the filter), it is not closed before the last frame, and exact on it
  rng = np.random.default_rng(43)
  frames = _dropout_frames(rng, n=260, drop=((100, 130),))
  tr = LaneLineTracker()
  new = np.array([tr.update(*f)[0] for f in frames])
  old = np.array([ry for ry, _ in _reference_p3(frames)])
  n = HANDBACK_N
  assert n > 1
  gap = np.abs(new[129:130 + n, 2, 0] - old[129:130 + n, 2, 0])
  assert gap[0] > 0.02 and gap[-1] == 0.0 and np.all(gap[1:-1] > 0.0)
  for k in range(1, n + 1):
    assert gap[k] <= gap[0] * (n - k) / n + 0.005
  assert new[130 + n - 1:].tobytes() == old[130 + n - 1:].tobytes()


# ---------------------------------------------------------------- new 2026-10-23 lrview_r9: no rule before / outside engagement
def test_on_not_engaged_is_bit_identical_to_previous_tracker():
  # lateral control not active (carControl.latActive False): a long dropout and a line under the car get no rule,
  # the tracker is exactly the previous one (99eb53f7 P3), y and prob
  rng = np.random.default_rng(51)
  frames = _dropout_frames(rng, n=260, drop=((40, 43), (100, 200))) + _crossing_frames(rng)
  tr = LaneLineTracker()
  for f, (ry, rp) in zip(frames, _reference_p3(frames), strict=True):
    fy, _, fp = tr.update(*f, engaged=False)
    assert fy.tobytes() == ry.tobytes() and fp.tobytes() == rp.tobytes()


def test_on_dropout_counter_starts_at_engagement():
  # a dropout that began before engagement counts from the engagement frame: up to LOW_N engaged dropout frames the
  # tracker is still the previous one, the rule only starts after them
  rng = np.random.default_rng(52)
  frames = _dropout_frames(rng, n=260, drop=((60, 200),))
  eng0 = 100
  tr = LaneLineTracker()
  new = np.array([tr.update(*f, engaged=(t >= eng0))[0] for t, f in enumerate(frames)])
  old = np.array([ry for ry, _ in _reference_p3(frames)])
  assert new[:eng0 + M.LOW_N].tobytes() == old[:eng0 + M.LOW_N].tobytes()
  assert not np.array_equal(new[130:200, 2], old[130:200, 2])


def test_apply_passes_engaged_to_the_tracker():
  # apply(engaged=False): no rule ever touches a line; apply() without the argument = engaged (as before)
  a = K9LRView(); a.set_mode(1); b = K9LRView(); b.set_mode(1); c = K9LRView(); c.set_mode(1)
  for t in range(80):
    o = parsed_outputs(1000 + t)
    o["lane_lines_prob"][0, 5] = 0.02 if 10 <= t < 70 else 0.98   # line 2 prob
    ra = a.apply(o, engaged=False); rb = b.apply(o); rc = c.apply(o, engaged=True)
    assert not a.tracker.yf.rule.any() and not a.tracker.yf.low_count.any()
    assert rb["lane_lines"].tobytes() == rc["lane_lines"].tobytes()
    assert ra["lane_lines_prob"].tobytes() == rb["lane_lines_prob"].tobytes()


def test_on_disengage_returns_to_previous_tracker_at_once():
  # a line moved by the long-dropout rule while engaged is exactly the previous tracker's line on the first
  # not-engaged frame (no ramp), and stays so while not engaged
  rng = np.random.default_rng(54)
  frames = _dropout_frames(rng, n=260, drop=((100, 200),))
  tr = LaneLineTracker()
  new = np.array([tr.update(*f, engaged=(t < 170))[0] for t, f in enumerate(frames)])
  old = np.array([ry for ry, _ in _reference_p3(frames)])
  assert not np.array_equal(new[130:170, 2], old[130:170, 2])
  assert new[170:].tobytes() == old[170:].tobytes()


# ---------------------------------------------------------------- new 2026-10-23 lrview_r10: minimum dropout length 10 frames
def test_on_dropouts_up_to_ten_frames_are_the_previous_tracker():
  # dropouts of 4 to 10 frames (0.2-0.5 s, which the rule followed before lrview_r10) are filtered exactly like the
  # previous 1 (P3, 99eb53f7), also where the line really moves (from frame 100 on, 1 cm/frame)
  assert M.LOW_N == 10
  rng = np.random.default_rng(61)
  frames = _dropout_frames(rng, n=260, drop=((40, 44), (60, 66), (105, 115), (130, 140), (170, 178)))
  tr = LaneLineTracker()
  for f, (ry, rp) in zip(frames, _reference_p3(frames), strict=True):
    fy, _, fp = tr.update(*f)
    assert fy.tobytes() == ry.tobytes() and fp.tobytes() == rp.tobytes()
    assert not tr.yf.rule.any()


# ---------------------------------------------------------------- new 2026-10-23 lrview_r11: hand-back ends where lane_planner_2 reads the lane width
def _confident_after(frames, t0, std_x0=0.1):
  """same frames, but from frame t0 on every line has model std std_x0 at x=0 (lane_planner_2 std weight 1)"""
  out = []
  for t, (y, s, p, v, w) in enumerate(frames):
    s = np.array(s, dtype=np.float64)
    if t >= t0:
      s[:, 0] = std_x0
    out.append((y, s, p, v, w))
  return out


def test_on_hand_back_ends_at_once_where_the_lane_width_filter_may_update():
  # after a long dropout of line 2 (raw prob back at frame 130): with model stds 0.4 (std weight 0) the hand-back takes
  # HANDBACK_N frames as before; with confident lines (std 0.1 at x=0, weight 1) it ends on the first frame where both
  # near lines are published above 0.5, so lane_planner_2's lane-width filter never sees a rule-shifted width
  rng = np.random.default_rng(71)
  base = _dropout_frames(rng, n=200, drop=((100, 130),))
  for frames, quick in ((base, False), (_confident_after(base, 130), True)):
    tr = LaneLineTracker()
    out = [tr.update(*f) for f in frames]
    new = np.array([o[0] for o in out]); pub = np.array([o[2] for o in out])
    old = np.array([ry for ry, _ in _reference_p3(frames)])
    both = [t for t in range(130, 200) if pub[t, 1] > 0.5 and pub[t, 2] > 0.5]
    assert both and both[0] < 130 + HANDBACK_N - 1
    tb = both[0]
    assert abs(new[129, 2, 0] - old[129, 2, 0]) > 0.02
    if quick:
      assert new[tb:].tobytes() == old[tb:].tobytes()
      assert not np.array_equal(new[tb - 1, 2], old[tb - 1, 2])
    else:
      assert not np.array_equal(new[tb, 2], old[tb, 2])
      assert new[130 + HANDBACK_N - 1:].tobytes() == old[130 + HANDBACK_N - 1:].tobytes()
    for (_, rp), p in zip(_reference_p3(frames), pub, strict=True):
      assert rp.tobytes() == p.tobytes()   # probabilities stay the previous ones


def test_on_one_lane_hold_near_lines_are_the_previous_tracker():
  # lrview_r13: K9OneLaneHold 1 reads the near lines (1, 2) in every frame, so with it on they are exactly the previous
  # tracker (99eb53f7 P3, y and prob) through a long dropout of a near line and a line under the car; lines 0 / 3 keep
  # the long-dropout rule (a dropout of line 3 still follows the model)
  rng = np.random.default_rng(81)
  frames = _dropout_frames(rng, n=260, drop=((40, 43), (100, 200))) + _crossing_frames(rng) + _dropout_frames(rng, n=260, drop=((100, 200),), line=3)
  tr = LaneLineTracker(); ref = LaneLineTracker()
  out = [tr.update(*f, one_lane_hold=True) for f in frames]
  r = [ref.update(*f) for f in frames]
  p3 = _reference_p3(frames)
  for (fy, _, fp), (ry, rp) in zip(out, p3, strict=True):
    assert fy[1:3].tobytes() == ry[1:3].tobytes() and fp.tobytes() == rp.tobytes()
  new = np.array([o[0] for o in out]); rule = np.array([o[0] for o in r]); old = np.array([ry for ry, _ in p3])
  assert not np.array_equal(rule[100:200, 2], old[100:200, 2])   # without the hold the rule does move line 2 there
  k = len(frames) - 160
  assert not np.array_equal(new[k:k + 100, 3], old[k:k + 100, 3])   # line 3 still has the rule with the hold on


def test_on_one_lane_hold_off_is_unchanged_and_switching_hands_over():
  # one_lane_hold False (default) = the tracker without the argument, bit for bit; switching the hold on mid-drive
  # makes the near lines the previous tracker from that frame on
  rng = np.random.default_rng(82)
  frames = _dropout_frames(rng, n=260, drop=((40, 43), (100, 200))) + _crossing_frames(rng)
  a = LaneLineTracker(); b = LaneLineTracker(); c = LaneLineTracker()
  p3 = _reference_p3(frames)
  for t, f in enumerate(frames):
    ra = a.update(*f); rb = b.update(*f, one_lane_hold=False); rc = c.update(*f, one_lane_hold=t >= 150)
    assert all(x.tobytes() == y.tobytes() for x, y in zip(ra, rb, strict=True))
    if t >= 150:
      assert rc[0][1:3].tobytes() == p3[t][0][1:3].tobytes()


def test_apply_and_read_one_lane_hold():
  # modeld: K9LRView.set_one_lane_hold(read_one_lane_hold(params)); read like controlsd (1 = on, else / error = off);
  # with K9LRView 0 the outputs still pass through as the same object
  class P:
    def __init__(self, v): self.v = v
    def get_int(self, k):
      assert k == "K9LaneAssist"   # 2026-10-25 lane menu merge
      if isinstance(self.v, Exception):
        raise self.v
      return self.v
  assert M.read_one_lane_hold(P(1)) is True
  for v in (0, 2, -1, 3, KeyError("x")):
    assert M.read_one_lane_hold(P(v)) is False
  off = K9LRView(); off.set_one_lane_hold(True); o = parsed_outputs(5)
  assert off.apply(o) is o
  a = K9LRView(); a.set_mode(1); a.set_one_lane_hold(True); b = K9LRView(); b.set_mode(1)
  for t in range(80):
    o = parsed_outputs(2000 + t)
    o["lane_lines_prob"][0, 5] = 0.02 if 10 <= t < 70 else 0.98   # line 2 prob
    ra = a.apply(o); rb = b.apply(o)
    assert not a.tracker.yf.rule[1:3].any()
    assert ra["lane_lines_prob"].tobytes() == rb["lane_lines_prob"].tobytes()
    assert ra["lane_lines"][0, 1:3, :, 0].tobytes() == np.asarray(a.tracker.yp3.y[1:3], dtype=ra["lane_lines"].dtype).tobytes()
