"""K9LRView (experimental menu number, NOT vehicle-validated): CPU lane-line post-processing in modeld.

0 (and any unknown number) = off: model outputs are passed through untouched (same object), so
modelV2 / drivingModelData are bit-identical to the parent.
1 = "P1g" from the lr_split_branch2_20261022 reference simulation (qcamera replay, no ground truth):
    per lane line and per X_IDXS point a 1D Kalman filter on y (measurement variance = model std^2,
    process noise 0.05 m^2 per 20 Hz frame), lane-line probability EMA (alpha 0.3), and a reset of a
    lane line to the raw model output when its x=0 point jumps by more than 1.0 m in one frame
    (lane change / lane-index swap). z, road edges, path/plan and every other output are unchanged.
No extra GPU work; cost is a few numpy ops on 4x33 arrays per frame.
"""
import numpy as np

K9_LR_VIEW_OFF = 0
K9_LR_VIEW_P1G = 1

PROFILES = {
  K9_LR_VIEW_P1G: dict(q=0.05, prob_alpha=0.3, gate_m=1.0),
}


def read_mode(params):
  """Param read that can never stop modeld (e.g. a build whose params keys predate K9LRView) -> 0 (off)."""
  try:
    return int(params.get_int("K9LRView"))
  except Exception:
    return K9_LR_VIEW_OFF


class LaneLineTracker:
  def __init__(self, q=0.05, prob_alpha=0.3, gate_m=1.0):
    self.q, self.prob_alpha, self.gate_m = q, prob_alpha, gate_m
    self.reset()

  def reset(self):
    self.y = None    # [4, 33] filtered y
    self.P = None    # [4, 33] variance
    self.prob = None # [4] filtered probability (the 1::2 entries of lane_lines_prob)
    self.prev_out_y0 = None

  def update(self, y, std, prob):
    """y, std: [4, 33]; prob: [4]. Returns (y, std, prob) filtered (new arrays)."""
    y = np.asarray(y, dtype=np.float64); std = np.maximum(np.asarray(std, dtype=np.float64), 1e-3)
    prob = np.asarray(prob, dtype=np.float64)
    if self.y is None or not (np.all(np.isfinite(self.y)) and np.all(np.isfinite(self.P))):
      self.y, self.P, self.prob = y.copy(), std ** 2, prob.copy()
      self.prev_out_y0 = self.y[:, 0].copy()
      return self.y.copy(), np.sqrt(self.P), self.prob.copy()
    R = std ** 2
    P = self.P + self.q
    K = P / (P + R)
    self.y = self.y + K * (y - self.y)
    self.P = (1 - K) * P
    jump = np.abs(y[:, 0] - self.prev_out_y0) > self.gate_m
    self.y[jump] = y[jump]; self.P[jump] = R[jump]
    self.prob = self.prob_alpha * prob + (1 - self.prob_alpha) * self.prob
    self.prev_out_y0 = self.y[:, 0].copy()
    return self.y.copy(), np.sqrt(self.P), np.clip(self.prob, 0.0, 1.0)


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
      y, s, p = self.tracker.update(ll[0, :, :, 0], ls[0, :, :, 0], lp[0, 1::2])
      if not (np.all(np.isfinite(y)) and np.all(np.isfinite(s)) and np.all(np.isfinite(p))):
        self.tracker.reset()
        return outputs
      new = dict(outputs)
      new['lane_lines'] = ll.copy(); new['lane_lines'][0, :, :, 0] = y
      new['lane_lines_stds'] = ls.copy(); new['lane_lines_stds'][0, :, :, 0] = s
      new['lane_lines_prob'] = lp.copy(); new['lane_lines_prob'][0, 1::2] = p
      return new
    except Exception:
      # never let the experiment break the model publish: fall back to the raw outputs
      try:
        self.tracker.reset()
      except Exception:
        pass
      return outputs
