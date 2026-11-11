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
    the raw model output (its probability EMA restarts too).
    Long dropouts (2026-10-22 lrview_night_r1, retuned 2026-10-23 lrview_r2): once a line's RAW probability has
    stayed below LOW_PROB for more than LOW_N frames (0.5 s; 0.15 s before 2026-10-23 lrview_r10), that line gets LOW_INFLATE x the process noise every
    frame until the raw probability is back (not while the near-lane width is above LOW_W_MAX 4.4 m, and since
    2026-10-23 lrview_r4 only while |EMA 0.3 of (raw line - rule-free filter) at x=0| > LOW_RES_MIN 3 cm, i.e. while the
    raw line really keeps moving away from the coasting estimate; a faded line that stays put is filtered as before), so the estimate follows the model again instead of coasting on the
    ego-motion prediction (the old tracker drifted 2-2.5 cm further from the model than the raw line in dropouts
    > 3 s). Dropouts up to 0.5 s are filtered exactly as before (lrview_r10: in the closed-loop sim a 0.2-0.45 s
    dropout in lane mode let the rule follow a transient wobble of the faded raw line by 4.5 cm and the car followed it;
    the qcamera-replay gain of the rule is in the dropouts > 1 s). A line whose predicted x=0 point is within
    NEAR_M (0.5 m) of the car (the car is crossing it) gets NEAR_INFLATE (2) x the process noise (since 2026-10-23
    lrview_r6 only while |that residual EMA| > NEAR_RES_MIN 3 cm, like the long-dropout rule). Hand-back
    (2026-10-23 lrview_r6): a line that either rule touched is set back to the state (y and variance) of the
    rule-free filter once its raw probability is back (>= LOW_PROB) and the near-car rule is off for
    it (since 2026-10-23 lrview_r7 spread over HANDBACK_N frames: the difference shrinks linearly and is 0 on the last
    one; a rule that comes back on stops it; since 2026-10-23 lrview_r11 it ends at once on a frame where lane_planner_2 may update its lane-width
    filter from the near lines (both published probabilities x std weight above 0.5), so that filter never takes a
    rule-shifted width), so after a dropout the tracker is exactly the previous one again instead of carrying the difference for
    seconds. Not engaged (2026-10-23 lrview_r9; carControl.latActive, already subscribed by modeld): neither rule runs,
    the dropout counter is held at 0 and the published lines are the rule-free filter's (y and variance, no
    hand-back ramp), so while not engaged and at engagement the tracker is exactly the previous one (in the
    MetaDrive sim a pre-engagement dropout otherwise left a 1e-7 curvature difference at engagement that grew
    chaotically). K9OneLaneHold 1 (2026-10-24 lrview_r13; modeld reads the same param as controlsd): the near lines
    (1, 2) are published from the rule-free filter (y and variance, no rule, no hand-back), because the one-lane hold
    reads their y in every frame (jitter, unexplained-motion and reliability state): in the closed-loop sim a rule-shifted
    faded line changed that state at 219 s and the hold pushed differently at 293 s (1e-6 m). Lines 0 and 3 keep the tracker.
    The published probabilities are exactly the
    previous ones: the probability EMA restarts on the 1.0 m jumps of a second, unmodified y filter (the y
    filter of 99eb53f7), so the long-dropout rule never moves a probability. Lane-line stds, z, road edges, path/plan and every
    other output are unchanged (raw stds keep lane_planner_2's lane weighting as is; the smaller Kalman posterior
    std is NOT published). No new inputs or subscriptions: v and yaw rate are read from the same parsed output.
    No lane-acceptance change: lane_planner_2 gates and blends the lanes exactly as with 0 (the lrview_night_r1
    verified-pair boost was dropped 2026-10-23 lrview_r2: in the MetaDrive full-stack sim it put lane mode on in the
    first curve at 60 km/h and the car left the lane there, while 0 and the previous 1 stayed in lane until the
    second curve).
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
LOW_PROB = 0.5     # raw lane-line probability below which a frame counts as a dropout frame
LOW_N = 10         # dropout frames in a row after which the line follows the model again (lrview_r10: 3 -> 10)
LOW_INFLATE = 5.0  # process-noise factor for such a line
LOW_W_MAX = 4.4    # ... but not while the near-lane width (predicted, x=0) is above this [m] (wide lane / merge)
LOW_RES_MIN = 0.03 # ... and only while the raw line keeps leaving the rule-free filter: |EMA 0.3 of (raw - rule-free
                   # filter) at x=0| above this [m] (a real lateral move, not a faded line that stays put)
RES_ALPHA = 0.3
NEAR_RES_MIN = 0.03 # near-car rule (below) only while |that residual EMA| is above this [m]
NEAR_M = 0.5       # a line whose predicted x=0 point is closer than this to the car (crossing it) ...
NEAR_INFLATE = 2.0 # ... gets this x the process noise (the ego lateral motion is not in the prediction)
HANDBACK_N = 6     # hand-back to the rule-free filter spread over this many frames (linear, the last one exact)


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


def read_one_lane_hold(params):
  """The one-lane hold as controlsd reads it (k9_one_lane_hold_mode: K9LaneAssist 1 = on, anything else / unreadable = off).
  2026-10-25 menu merge (k9_lane.py): K9LRView and K9OneLaneHold are both the single switch K9LaneAssist now."""
  try:
    return int(params.get_int("K9LaneAssist")) == 1
  except Exception:
    return False


def read_mode(params):
  """Param read that can never stop modeld (e.g. a build whose params keys predate K9LaneAssist) -> 0 (off).
  2026-10-25 menu merge (k9_lane.py): the tracker is on with K9LaneAssist 1 (only 1; the old K9LRView 2 / 3 are migrated)."""
  try:
    return K9_LR_VIEW_ON if int(params.get_int("K9LaneAssist")) == 1 else K9_LR_VIEW_OFF
  except Exception:
    return K9_LR_VIEW_OFF


class _YFilter:
  """Per-point 1D Kalman y filter (ego-motion prediction, innovation inflation, 1.0 m jump reset).
  low_n = 0 and near_m = 0: no long-dropout / near-car rule (= the previous 99eb53f7 y filter)."""
  def __init__(self, q_x, low_n, low_inflate, near_m=0.0, near_inflate=1.0, low_w_max=0.0):
    self.q_x, self.low_n, self.low_inflate, self.low_w_max = q_x, low_n, low_inflate, low_w_max
    self.near_m, self.near_inflate = near_m, near_inflate
    self.y = None; self.P = None; self.prev_out_y0 = None
    self.low_count = np.zeros(4, dtype=np.int64)
    self.rule = np.zeros(4, dtype=bool)       # lines a rule touched since their last hand-back
    self.near_now = np.zeros(4, dtype=bool)   # near-car rule on this frame
    self.low_now = np.zeros(4, dtype=bool)    # long-dropout rule on this frame

  def init(self, y, R):
    self.y, self.P = y.copy(), R.copy()
    self.prev_out_y0 = self.y[:, 0].copy()
    self.low_count = np.zeros(self.y.shape[0], dtype=np.int64)
    self.rule = np.zeros(self.y.shape[0], dtype=bool); self.near_now = np.zeros(self.y.shape[0], dtype=bool)
    self.low_now = np.zeros(self.y.shape[0], dtype=bool)

  def ok(self):
    return self.y is not None and bool(np.all(np.isfinite(self.y)) and np.all(np.isfinite(self.P)))

  def step(self, y, R, prob, v_ego, yaw_rate, res_ema=None, engaged=True):
    """returns the boolean jump mask (lines reset to the raw output this frame).
    engaged False (lateral control not active): neither rule runs and the dropout counter is held at 0."""
    v = float(v_ego) if np.isfinite(v_ego) else 0.0
    w = float(yaw_rate) if np.isfinite(yaw_rate) else 0.0
    xs = X_IDXS + max(v, 0.0) * DT
    self.y = np.stack([np.interp(xs, X_IDXS, self.y[i]) for i in range(self.y.shape[0])]) - (w * DT) * xs[None, :]
    q = np.repeat((self.q_x * (1.0 + C_YAW * abs(w) / 0.1))[None, :], self.y.shape[0], axis=0)
    self.near_now = np.zeros(self.y.shape[0], dtype=bool); self.low_now = np.zeros(self.y.shape[0], dtype=bool)
    if self.low_n:
      self.low_count = np.where(prob < LOW_PROB, self.low_count + 1, 0) if engaged else np.zeros_like(self.low_count)
      low = self.low_count > self.low_n
      if self.low_w_max and (self.y[2, 0] - self.y[1, 0]) > self.low_w_max:
        low = np.zeros_like(low)
      if res_ema is not None:
        low = low & (np.abs(res_ema) > LOW_RES_MIN)
      q[low] *= self.low_inflate
      self.rule |= low; self.low_now = low
    if self.near_m and engaged:
      near = np.abs(self.y[:, 0]) < self.near_m
      if res_ema is not None:
        near = near & (np.abs(res_ema) > NEAR_RES_MIN)
      q[near] *= self.near_inflate
      self.rule |= near; self.near_now = near
    P = self.P + q
    P = np.where(np.abs(y - self.y) > K_INNOV * np.sqrt(P + R), P + q * (INFLATE - 1.0), P)
    K = P / (P + R)
    self.y = self.y + K * (y - self.y)
    self.P = (1 - K) * P
    jump = np.abs(y[:, 0] - self.prev_out_y0) > GATE_M
    self.y[jump] = y[jump]; self.P[jump] = R[jump]
    self.prev_out_y0 = self.y[:, 0].copy()
    return jump


class LaneLineTracker:
  def __init__(self):
    self.q_x = Q * (1.0 + D_DIST * X_IDXS / 50.0)
    self.reset()

  def reset(self):
    self.yf = _YFilter(self.q_x, LOW_N, LOW_INFLATE, NEAR_M, NEAR_INFLATE, LOW_W_MAX)   # published y
    self.yp3 = _YFilter(self.q_x, 0, 1.0)             # previous y filter: decides only the probability resets
    self.prob = None # [4] filtered probability (the 1::2 entries of lane_lines_prob)

  @property
  def y(self):
    return self.yf.y

  @property
  def low_count(self):
    return self.yf.low_count

  def update(self, y, std, prob, v_ego=0.0, yaw_rate=0.0, engaged=True, one_lane_hold=False):
    """y, std: [4, 33]; prob: [4]. Returns new arrays: filtered y, the RAW model std (passed through on purpose,
    see module doc), filtered prob. engaged False: no rule this frame (see _YFilter.step).
    one_lane_hold True (K9OneLaneHold 1): the near lines 1, 2 are the rule-free filter's (see module doc)."""
    y = np.asarray(y, dtype=np.float64); std = np.maximum(np.asarray(std, dtype=np.float64), 1e-3)
    prob = np.asarray(prob, dtype=np.float64)
    R = std ** 2
    if not (self.yf.ok() and self.yp3.ok() and self.prob is not None):
      self.yf.init(y, R); self.yp3.init(y, R); self.prob = prob.copy(); self.res_ema = np.zeros(y.shape[0])
      self.ramp = np.zeros(y.shape[0], dtype=np.int64)
      return self.yf.y.copy(), std.copy(), self.prob.copy()
    jump_p3 = self.yp3.step(y, R, prob, v_ego, yaw_rate)
    self.res_ema = RES_ALPHA * (y[:, 0] - self.yp3.y[:, 0]) + (1 - RES_ALPHA) * self.res_ema
    self.yf.step(y, R, prob, v_ego, yaw_rate, self.res_ema, bool(engaged))
    back = self.yf.rule & (prob >= LOW_PROB) & ~self.yf.near_now   # hand-back to the rule-free filter
    cancel = (self.ramp > 0) & (self.yf.low_now | self.yf.near_now)   # a rule is on again: hand-back stops
    self.ramp[cancel] = 0
    start = back & (self.ramp == 0)
    self.ramp[start] = HANDBACK_N; self.yf.rule[start] = False
    self.prob = PROB_ALPHA * prob + (1 - PROB_ALPHA) * self.prob
    self.prob[jump_p3] = prob[jump_p3]
    if self.prob.shape[0] > 2 and np.any(self.ramp > 0):
      # lrview_r11: lane_planner_2 updates its lane-width filter from the x=0 points of the near lines (1, 2) when
      # both of their probabilities x std weight (np.interp(std, [.15, .3], [1, 0]); its width check can only lower
      # them) are above 0.5. On such a frame any hand-back ends at once, so that filter never takes a rule-shifted
      # width (closed-loop sim: it kept a 1e-5 m width difference for seconds and moved the lane-mode path).
      pw = np.clip(self.prob[1:3], 0.0, 1.0) * np.interp(std[1:3, 0], [0.15, 0.3], [1.0, 0.0])
      if pw[0] > LOW_PROB and pw[1] > LOW_PROB:
        self.ramp[self.ramp > 0] = 1
    act = self.ramp > 0
    if act.any():   # the difference to the rule-free filter shrinks linearly over HANDBACK_N frames (last frame exact)
      f = (self.ramp[act] - 1).astype(np.float64) / self.ramp[act]
      self.yf.y[act] = self.yp3.y[act] + (self.yf.y[act] - self.yp3.y[act]) * f[:, None]; self.yf.P[act] = self.yp3.P[act]
      self.yf.prev_out_y0 = self.yf.y[:, 0].copy(); self.ramp[act] -= 1
    if not engaged:   # not engaged: exactly the rule-free (previous) filter, so engagement starts from its state
      self.yf.y = self.yp3.y.copy(); self.yf.P = self.yp3.P.copy(); self.yf.prev_out_y0 = self.yp3.prev_out_y0.copy()
      self.yf.rule[:] = False; self.ramp[:] = 0
    if one_lane_hold and self.yf.y.shape[0] > 2:   # lrview_r13: the one-lane hold reads lines 1, 2 -> rule-free filter there
      n = slice(1, 3)
      self.yf.y[n] = self.yp3.y[n]; self.yf.P[n] = self.yp3.P[n]; self.yf.prev_out_y0[n] = self.yp3.prev_out_y0[n]
      self.yf.rule[n] = False; self.ramp[n] = 0
    return self.yf.y.copy(), std.copy(), np.clip(self.prob, 0.0, 1.0)


class K9LRView:
  """Holds the selected number (0/1) and the tracker state; switching off and on again restarts the tracker."""
  def __init__(self):
    self.mode = K9_LR_VIEW_OFF
    self.tracker = None
    self.one_lane_hold = False   # lrview_r13: K9OneLaneHold 1 -> near lines from the rule-free filter

  def set_one_lane_hold(self, on):
    self.one_lane_hold = bool(on)

  def set_mode(self, mode):
    mode = normalize_mode(mode)
    if mode != self.mode:
      self.mode = mode
      self.tracker = LaneLineTracker() if mode == K9_LR_VIEW_ON else None

  def apply(self, outputs, engaged=True):
    """outputs: parsed model output dict (modeld). OFF -> the same dict object, untouched.
    engaged: lateral control active (modeld: carControl.latActive, already subscribed); False -> no rule this frame."""
    if self.mode == K9_LR_VIEW_OFF or self.tracker is None or outputs is None:
      return outputs
    try:
      ll = outputs['lane_lines']; ls = outputs['lane_lines_stds']; lp = outputs['lane_lines_prob']
      plan = outputs['plan']
      v_ego = float(plan[0, 0, Plan.VELOCITY][0]); yaw_rate = float(plan[0, 0, Plan.ORIENTATION_RATE][2])
      y, s, p = self.tracker.update(ll[0, :, :, 0], ls[0, :, :, 0], lp[0, 1::2], v_ego, yaw_rate, engaged, self.one_lane_hold)
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
