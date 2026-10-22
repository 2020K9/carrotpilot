"""K9LRView (experimental, NOT vehicle-validated): CPU lane-line tracker in modeld. One finished version: 0 off / 1 on.

0 = off (default): model outputs are passed through untouched (same object), so modelV2 / drivingModelData are
    bit-identical to the parent.
1 = on: the lane-line tracker. Per lane line and per X_IDXS point a 1D Kalman filter on y (measurement variance =
    model std^2). Before each update the previous estimate is moved with the car's own motion taken from this
    frame's model plan (forward speed and yaw rate at t=0): shifted forward by v*dt along x and rotated by
    yaw_rate*dt. Process noise is smaller near the car and grows with distance and yaw rate
    (q(x) = Q * (1 + D_DIST*x/50) * (1 + C_YAW*|yaw_rate|/0.1)), and a point whose innovation exceeds K_INNOV sigma
    gets INFLATE x the process noise for that frame. Lane-line probabilities get an EMA (alpha 0.3, ~0.15 s).
    A lane line whose x=0 point jumps by more than 1.0 m in one frame (lane change / lane-index swap) is reset to
    the raw model output (its probability EMA restarts too). Lane-line stds, z, road edges, path/plan and every
    other output are unchanged (raw stds keep lane_planner_2's lane weighting as is; the smaller Kalman posterior
    std is NOT published). No new inputs or subscriptions: v and yaw rate are read from the same parsed output.
Stored values: 1, 2 and 3 (numbers of the earlier, never-pushed trial menu) all mean 1 now; anything else
(negative, 4 or more, non-int, unreadable) is 0.

History (simulation only, qcamera replay, no ground truth): the earlier trial numbers 1 "P1g" (Kalman + prob EMA,
no ego motion) and 2 "P2m" (ego-motion prediction, prob EMA 0.5) were merged into this one tracker
(lane_improve_20261022 "P3": the P2m y filter with the P1g prob EMA 0.3); they are no longer selectable.
No extra GPU work; cost is a few numpy ops on 4x33 arrays per frame.
"""
import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants, Plan

K9_LR_VIEW_OFF = 0
K9_LR_VIEW_ON = 1
K9_LR_VIEW_LEGACY_ON = (1, 2, 3)  # stored values that mean 1

DT = 0.05  # modeld runs at 20 Hz
X_IDXS = np.asarray(ModelConstants.X_IDXS, dtype=np.float64)

Q = 0.01         # process noise per frame at x=0, straight driving [m^2]
PROB_ALPHA = 0.3 # lane-line probability EMA weight of the new frame
GATE_M = 1.0     # x=0 jump that resets a lane line to the raw model output [m]
K_INNOV = 2.0    # innovation (sigma) above which the process noise is inflated
INFLATE = 10.0
C_YAW = 3.0
D_DIST = 3.0


def normalize_mode(mode):
  """Stored param value -> 0 or 1 (old 1/2/3 -> 1, everything else -> 0)."""
  try:
    if isinstance(mode, bool) or mode is None:
      return K9_LR_VIEW_OFF
    if isinstance(mode, float) and not mode.is_integer():
      return K9_LR_VIEW_OFF
    mode = int(mode)
  except Exception:
    return K9_LR_VIEW_OFF
  return K9_LR_VIEW_ON if mode in K9_LR_VIEW_LEGACY_ON else K9_LR_VIEW_OFF


def read_mode(params):
  """Param read that can never stop modeld (e.g. a build whose params keys predate K9LRView) -> 0 (off)."""
  try:
    return normalize_mode(params.get_int("K9LRView"))
  except Exception:
    return K9_LR_VIEW_OFF


class LaneLineTracker:
  def __init__(self):
    self.q_x = Q * (1.0 + D_DIST * X_IDXS / 50.0)
    self.reset()

  def reset(self):
    self.y = None    # [4, 33] filtered y
    self.P = None    # [4, 33] variance
    self.prob = None # [4] filtered probability (the 1::2 entries of lane_lines_prob)
    self.prev_out_y0 = None

  def _predict(self, v_ego, yaw_rate):
    v = float(v_ego) if np.isfinite(v_ego) else 0.0
    w = float(yaw_rate) if np.isfinite(yaw_rate) else 0.0
    xs = X_IDXS + max(v, 0.0) * DT
    return np.stack([np.interp(xs, X_IDXS, self.y[i]) for i in range(self.y.shape[0])]) - (w * DT) * xs[None, :]

  def update(self, y, std, prob, v_ego=0.0, yaw_rate=0.0):
    """y, std: [4, 33]; prob: [4]. Returns new arrays: filtered y, the RAW model std (passed through on purpose,
    see module doc), filtered prob."""
    y = np.asarray(y, dtype=np.float64); std = np.maximum(np.asarray(std, dtype=np.float64), 1e-3)
    prob = np.asarray(prob, dtype=np.float64)
    if self.y is None or not (np.all(np.isfinite(self.y)) and np.all(np.isfinite(self.P))):
      self.y, self.P, self.prob = y.copy(), std ** 2, prob.copy()
      self.prev_out_y0 = self.y[:, 0].copy()
      return self.y.copy(), std.copy(), self.prob.copy()
    R = std ** 2
    self.y = self._predict(v_ego, yaw_rate)
    w = float(yaw_rate) if np.isfinite(yaw_rate) else 0.0
    q = self.q_x * (1.0 + C_YAW * abs(w) / 0.1)
    P = self.P + q
    P = np.where(np.abs(y - self.y) > K_INNOV * np.sqrt(P + R), P + q * (INFLATE - 1.0), P)
    K = P / (P + R)
    self.y = self.y + K * (y - self.y)
    self.P = (1 - K) * P
    self.prob = PROB_ALPHA * prob + (1 - PROB_ALPHA) * self.prob
    jump = np.abs(y[:, 0] - self.prev_out_y0) > GATE_M
    self.y[jump] = y[jump]; self.P[jump] = R[jump]; self.prob[jump] = prob[jump]
    self.prev_out_y0 = self.y[:, 0].copy()
    return self.y.copy(), std.copy(), np.clip(self.prob, 0.0, 1.0)


class K9LRView:
  """Holds the selected number (0/1) and the tracker state; switching off and on again restarts the tracker."""
  def __init__(self):
    self.mode = K9_LR_VIEW_OFF
    self.tracker = None

  def set_mode(self, mode):
    mode = normalize_mode(mode)
    if mode != self.mode:
      self.mode = mode
      self.tracker = LaneLineTracker() if mode == K9_LR_VIEW_ON else None

  def apply(self, outputs):
    """outputs: parsed model output dict (modeld). OFF -> the same dict object, untouched."""
    if self.mode == K9_LR_VIEW_OFF or self.tracker is None or outputs is None:
      return outputs
    try:
      ll = outputs['lane_lines']; ls = outputs['lane_lines_stds']; lp = outputs['lane_lines_prob']
      plan = outputs['plan']
      v_ego = float(plan[0, 0, Plan.VELOCITY][0]); yaw_rate = float(plan[0, 0, Plan.ORIENTATION_RATE][2])
      y, s, p = self.tracker.update(ll[0, :, :, 0], ls[0, :, :, 0], lp[0, 1::2], v_ego, yaw_rate)
      if not (np.all(np.isfinite(y)) and np.all(np.isfinite(p))):
        self.tracker.reset()
        return outputs
      new = dict(outputs)
      new['lane_lines'] = ll.copy(); new['lane_lines'][0, :, :, 0] = y
      new['lane_lines_prob'] = lp.copy(); new['lane_lines_prob'][0, 1::2] = p
      return new
    except Exception:
      # never let the experiment break the model publish: fall back to the raw outputs
      try:
        self.tracker.reset()
      except Exception:
        pass
      return outputs
