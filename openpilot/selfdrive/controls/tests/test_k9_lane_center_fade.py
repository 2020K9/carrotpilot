"""K9LaneCenter drop-out / re-entry / centre crossing (11/03: BOTH_T 0.5, RAMP_OUT 0.3, push from min(filtered, latest) offset).
Synthetic model frames, no vehicle data."""
import numpy as np

from openpilot.selfdrive.controls.lib import k9_lane_center as H
from openpilot.selfdrive.controls.tests.test_k9_lane_center import frame

DT = H.DT


def drive(h, segs, v=20.0):
  """segs: list of (seconds, y_car, lines_ok); a model frame every 5 ticks, car does not move (open loop)"""
  out = []
  for sec, y, ok in segs:
    for k in range(int(round(sec / DT))):
      if k % 5 == 0:
        md = frame(-1.75 - y, 1.75 - y, p_left=0.9 if ok else 0.3, p_right=0.9 if ok else 0.3)
      out.append(h.update(md, v, True, False, False, 0.0))
  return np.array(out)


def test_waits_both_t_then_pushes_toward_centre():
  out = drive(H.LaneCenter(), [(3.0, 0.3, True)])
  n = int(round(H.BOTH_T / DT))
  assert np.all(out[:n - 5] == 0.0)
  assert out[-1] < 0.0 and np.all(out <= 0.0)          # car right of centre -> pulled left only


def test_line_dropout_fades_to_exact_zero_without_sign_change():
  out = drive(H.LaneCenter(), [(4.0, 0.3, True), (2.0, 0.3, False)])
  i0 = int(round(4.0 / DT))
  k0 = out[i0 - 1]
  assert k0 < 0.0
  tail = out[i0:]
  assert np.all(tail <= 0.0) and np.all(np.abs(tail) <= abs(k0) + 1e-12)   # only shrinks, never flips
  assert np.all(np.diff(np.abs(tail)) <= 1e-12)
  dk = min(H.J_MAX / 20.0 ** 2, H.KR_MAX) * DT
  n_end = int(np.ceil(abs(k0) / dk)) + int(round(H.RAMP_OUT / DT)) + 5
  assert np.all(tail[n_end:] == 0.0)


def test_reentry_after_dropout_rewatches():
  out = drive(H.LaneCenter(), [(4.0, 0.3, True), (2.0, 0.3, False), (3.0, 0.3, True)])
  i1 = int(round(6.0 / DT))
  assert np.all(out[i1 - 10:i1] == 0.0)
  n = int(round(H.BOTH_T / DT))
  assert np.all(out[i1:i1 + n - 5] == 0.0)             # watches the lines again before pushing
  assert out[-1] < 0.0 and np.all(out[i1:] <= 0.0)


def test_short_flicker_dropouts_never_push_away():
  segs = []
  for _ in range(6):
    segs += [(0.7, 0.25, True), (0.15, 0.25, False)]
  out = drive(H.LaneCenter(), segs)
  assert np.all(out <= 0.0)


def test_crossing_centre_does_not_keep_pushing_past():
  # car right of centre held, then the lines show it 0.1 m left of centre: the lagging filter must not keep pulling it left
  h = H.LaneCenter()
  out = drive(h, [(4.0, 0.15, True)])
  k0 = out[-1]
  assert k0 < 0.0
  out2 = drive(h, [(0.3, -0.1, True)])
  assert np.all(out2 >= k0 - 1e-12)                    # never more push to the left than before the crossing
  assert out2[-1] > k0
