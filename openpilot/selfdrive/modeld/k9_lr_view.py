"""K9LRView (experimental menu number, NOT vehicle-validated): CPU lane-line post-processing in modeld.

0 (and any unknown number) = off: model outputs are passed through untouched (same object), so
modelV2 / drivingModelData are bit-identical to the parent.
1 = "P1g" from the lr_split_branch2_20261022 reference simulation (qcamera replay, no ground truth):
    per lane line and per X_IDXS point a 1D Kalman filter on y (measurement variance = model std^2,
    process noise 0.05 m^2 per 20 Hz frame), lane-line probability EMA (alpha 0.3), and a reset of a
    lane line to the raw model output when its x=0 point jumps by more than 1.0 m in one frame
    (lane change / lane-index swap; its prob EMA restarts too). Lane-line stds, z, road edges, path/plan and
    every other output are unchanged (raw stds keep lane_planner_2's lane weighting as is; the smaller Kalman
    posterior std is NOT published).
2 = "P2m" from the lr_track_main_20261022 simulation (train/validation split by drive):
    the same per-point Kalman filter, but before each update the previous estimate is moved with the
    car's own motion taken from this frame's model plan (forward speed and yaw rate at t=0): shifted
    forward by v*dt along x and rotated by yaw_rate*dt. Process noise is smaller near the car and grows
    with distance and yaw rate (q(x) = Q * (1 + D_DIST*x/50) * (1 + C_YAW*|yaw_rate|/0.1)), and a point
    whose innovation exceeds K_INNOV sigma gets INFLATE x the process noise for that frame.
    Probability EMA / jump reset as listed in PROFILES. No new inputs or subscriptions: v and yaw rate
    are read from the same parsed model output.
No extra GPU work; cost is a few numpy ops on 4x33 arrays per frame.
"""
import numpy as np

from openpilot.selfdrive.modeld.constants import ModelConstants, Plan

K9_LR_VIEW_OFF = 0
K9_LR_VIEW_P1G = 1
K9_LR_VIEW_P2M = 2

DT = 0.05  # modeld runs at 20 Hz
X_IDXS = np.asarray(ModelConstants.X_IDXS, dtype=np.float64)

PROFILES = {
  K9_LR_VIEW_P1G: dict(q=0.05, prob_alpha=0.3, gate_m=1.0),
  K9_LR_VIEW_P2M: dict(q=0.01, prob_alpha=0.5, gate_m=1.0, k_innov=2.0, inflate=10.0, c_yaw=3.0, d_dist=3.0, ego_predict=True),
}


def read_mode(params):
  """Param read that can never stop modeld (e.g. a build whose params keys predate K9LRView) -> 0 (off)."""
  try:
    return int(params.get_int("K9LRView"))
  except Exception:
    return K9_LR_VIEW_OFF


class LaneLineTracker:
  def __init__(self, q=0.05, prob_alpha=0.3, gate_m=1.0, k_innov=0.0, inflate=10.0, c_yaw=0.0, d_dist=0.0, ego_predict=False):
    self.q, self.prob_alpha, self.gate_m = q, prob_alpha, gate_m
    self.k_innov, self.inflate, self.c_yaw, self.d_dist, self.ego_predict = k_innov, inflate, c_yaw, d_dist, ego_predict
    self.q_x = q * (1.0 + d_dist * X_IDXS / 50.0) if d_dist else None
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
    if self.ego_predict:
      self.y = self._predict(v_ego, yaw_rate)
    q = self.q if self.q_x is None else self.q_x
    if self.c_yaw:
      w = float(yaw_rate) if np.isfinite(yaw_rate) else 0.0
      q = q * (1.0 + self.c_yaw * abs(w) / 0.1)
    P = self.P + q
    if self.k_innov:
      P = np.where(np.abs(y - self.y) > self.k_innov * np.sqrt(P + R), P + q * (self.inflate - 1.0), P)
    K = P / (P + R)
    self.y = self.y + K * (y - self.y)
    self.P = (1 - K) * P
    self.prob = self.prob_alpha * prob + (1 - self.prob_alpha) * self.prob
    jump = np.abs(y[:, 0] - self.prev_out_y0) > self.gate_m
    self.y[jump] = y[jump]; self.P[jump] = R[jump]; self.prob[jump] = prob[jump]
    self.prev_out_y0 = self.y[:, 0].copy()
    return self.y.copy(), std.copy(), np.clip(self.prob, 0.0, 1.0)


class K9LRView:
  """Holds the selected number and the tracker state; switching the number resets the state."""
  def __init__(self):
    self.mode = K9_LR_VIEW_OFF
    self.tracker = None

  def set_mode(self, mode):
    try:
      mode = int(mode)
    except Exception:
      mode = K9_LR_VIEW_OFF
    if mode not in PROFILES:
      mode = K9_LR_VIEW_OFF
    if mode != self.mode:
      self.mode = mode
      self.tracker = LaneLineTracker(**PROFILES[mode]) if mode in PROFILES else None

  def apply(self, outputs):
    """outputs: parsed model output dict (modeld). OFF -> the same dict object, untouched."""
    if self.mode == K9_LR_VIEW_OFF or self.tracker is None or outputs is None:
      return outputs
    try:
      ll = outputs['lane_lines']; ls = outputs['lane_lines_stds']; lp = outputs['lane_lines_prob']
      v_ego = yaw_rate = 0.0
      if self.tracker.ego_predict or self.tracker.c_yaw:
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
