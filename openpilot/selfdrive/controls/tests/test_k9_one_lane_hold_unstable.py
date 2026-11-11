"""K9OneLaneHold r7 (K9LaneAssist 1): no hold while the lane estimate jumps (1c9 17:30 lane estimate +-30 cm in 1 s).
Synthetic model frames, no vehicle data."""
import numpy as np

from openpilot.selfdrive.controls.lib import k9_one_lane_hold as H
from openpilot.selfdrive.controls.tests.test_k9_one_lane_hold import frame, run, warm, DT


def test_r7_steady_one_line_still_pushes():
  h = warm()
  out, _ = run(h, lambda y, p: frame(-1.8 - y, 1.8 - y, p_right=0.05, slope=-p), 3.0, y0=-0.4)
  assert out.max() > 1e-4 and h.unst_t == 0.0


def test_r7_swinging_visible_line_is_not_followed():
  h = warm()
  t = {"t": 0.0}
  def fr(y, p):   # right line lost, the visible left line estimate swings +-0.2 m at 1 Hz (lane estimate wander)
    t["t"] += 0.05
    return frame(-1.8 - y + 0.2 * np.sin(2 * np.pi * t["t"]), 1.8 - y, p_right=0.05, slope=-p)
  out, _ = run(h, fr, 4.0, y0=-0.4)
  assert np.max(np.abs(out)) == 0.0 and h.mem is None


def test_r7_width_swing_before_the_loss_blocks_the_hold():
  h = warm()
  t = {"t": 0.0}
  def fr(y, p):   # right line (prob 0.4 >= UNST_P, not reliable) swings 0.4 m in 0.4 s, then lost at 1 s; left line steady
    t["t"] += 0.05
    if t["t"] < 1.0:
      return frame(-1.8 - y, 1.8 - y + 0.4 * min(t["t"] / 0.4, 1.0), p_right=0.4, slope=-p)
    return frame(-1.8 - y, 1.8 - y, p_right=0.05, slope=-p)
  out, _ = run(h, fr, 1.0 + H.UNST_HOLD - 0.1, y0=-0.4)
  assert np.max(np.abs(out)) == 0.0


def test_r7_lost_line_garbage_below_unst_p_is_ignored():
  h = warm()
  t = {"t": 0.0}
  def fr(y, p):   # lost right line (prob 0.05 < UNST_P) has garbage y; the steady left line is still held
    t["t"] += 0.05
    return frame(-1.8 - y, 1.8 - y + 0.8 * np.sin(2 * np.pi * t["t"]), p_right=0.05, slope=-p)
  out, _ = run(h, fr, 3.0, y0=-0.4)
  assert out.max() > 1e-4


def test_r7_swing_mid_hold_cuts_the_push_at_the_jerk_limit():
  h = warm()
  out1, _ = run(h, lambda y, p: frame(-1.8 - y, 1.8 - y, p_right=0.05, slope=-p), 3.0, y0=-0.4)
  k0 = out1[-1]; assert k0 > 1e-4
  t = {"t": 0.0}
  def fr(y, p):   # visible line jumps 0.35 m over 0.5 s
    t["t"] += 0.05
    return frame(-1.8 - y - 0.35 * min(t["t"] / 0.5, 1.0), 1.8 - y, p_right=0.05, slope=-p)
  out2, _ = run(h, fr, 4.0, y0=-0.4)
  assert h.unst_t > 0.0 or out2[-1] == 0.0
  k = np.concatenate([[k0], out2])
  assert np.max(np.abs(np.diff(k))) <= H.J_MAX / 400 * DT + 1e-12   # never a step
  assert out2[-1] == 0.0 and np.all(out2 >= -1e-12)                  # falls to exactly 0, never changes sign
