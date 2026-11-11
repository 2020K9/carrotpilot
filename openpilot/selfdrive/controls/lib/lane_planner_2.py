import math
import numpy as np
from openpilot.cereal import log
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
# from openpilot.common.swaglog import cloudlog
# from openpilot.common.logger import sLogger
from openpilot.common.params import Params
from openpilot.selfdrive.controls.lib.k9_lane_fix import k9_lane_fix_mode, k9_lane_fix_profile

TRAJECTORY_SIZE = 33
# positive numbers go right
CAMERA_OFFSET = 0 #0.08
MIN_LANE_DISTANCE = 2.6
MAX_LANE_DISTANCE = 3.7
MAX_LANE_CENTERING_AWAY = 1.85
KEEP_MIN_DISTANCE_FROM_LANE = 1.35
KEEP_MIN_DISTANCE_FROM_EDGELANE = 1.15

def clamp(num, min_value, max_value):
  # weird broken case, do something reasonable
  if min_value > num > max_value:
    return (min_value + max_value) * 0.5
  # ok, basic min/max below
  if num < min_value:
    return min_value
  if num > max_value:
    return max_value
  return num

def sigmoid(x, scale=1, offset=0):
  return (1 / (1 + math.exp(x*scale))) + offset

def lerp(start, end, t):
  t = clamp(t, 0.0, 1.0)
  return (start * (1.0 - t)) + (end * t)

def max_abs(a, b):
  return a if abs(a) > abs(b) else b

def lane_converge_weight(t, mode, T):
  # K9LaneFix: s(t) = 1 at t=0 -> 0 at the end; fraction of the initial lane-path offset e0 still removed at time t.
  t = np.asarray(t, dtype=float)
  if mode == "smoothstep":
    u = np.clip(t / T, 0.0, 1.0)
    return 1.0 - u * u * (3.0 - 2.0 * u)
  if mode == "smootherstep":
    u = np.clip(t / T, 0.0, 1.0)
    return 1.0 - u * u * u * (u * (6.0 * u - 15.0) + 10.0)
  if mode == "exp":
    return np.exp(-np.maximum(t, 0.0) / T)
  return np.zeros_like(t)

# K9LaneFix F1 merge latch (2026-10-26 recheck_0208): at lane-mode entry the merge also takes over the car's current lateral
# rate / accel relative to the lane path (from the model path vs the lane path over the first LANE_MERGE_FIT_T s), so the
# reference continues the car's motion instead of first stopping it and then pulling hard (quintic, smootherstep only).
LANE_MERGE_MATCH = 2          # 0 position only, 1 + rate, 2 + rate and accel
LANE_MERGE_FIT_T = 1.0        # s
LANE_MERGE_R_MAX = 1.0        # |rate| cap [m/s]
LANE_MERGE_A_MAX = 1.5        # |accel| cap [m/s^2]
# Only |e0| <= t_eref * (t_max / converge_t)^2 is merged (the largest offset t_max can merge at the t_eref peak lateral accel);
# the rest is left to the MPC as with K9LaneAssist 0 (a 1 m offset at lane-mode start no longer asks for ~1.4 m/s^2).
LANE_MERGE_J_REF = 1.0        # m/s^3: merge time at least (60 |e0| / J_REF)^(1/3) (peak lateral jerk of the merge), within t_max;
                              # converge_t 1 s stays for |e0| <= 1.7 cm, 9 cm -> 1.75 s, 13 cm or more -> t_max 2 s


def lane_merge_rates(path_t, d):
  """rate / accel of d(t) = lane path - model path at t = 0 (quadratic least squares over t <= LANE_MERGE_FIT_T)"""
  if LANE_MERGE_MATCH <= 0:
    return 0.0, 0.0
  t = np.asarray(path_t, dtype=float); d = np.asarray(d, dtype=float)
  m = np.isfinite(t) & np.isfinite(d) & (t <= LANE_MERGE_FIT_T)
  if m.sum() < 4:
    return 0.0, 0.0
  c2, c1, _ = np.polyfit(t[m], d[m], 2)
  r0 = float(np.clip(c1, -LANE_MERGE_R_MAX, LANE_MERGE_R_MAX))
  a0 = float(np.clip(2.0 * c2, -LANE_MERGE_A_MAX, LANE_MERGE_A_MAX)) if LANE_MERGE_MATCH >= 2 else 0.0
  return r0, a0


def lane_merge_offset(t, mode, T, e0, r0=0.0, a0=0.0):
  """F1 merge offset E(t) subtracted from the lane path: E(0) = e0, E'(0) = r0, E''(0) = a0, E = E' = E'' = 0 from T on."""
  out = e0 * lane_converge_weight(t, mode, T)
  if mode != "smootherstep" or (r0 == 0.0 and a0 == 0.0):
    return out
  u = np.clip(np.asarray(t, dtype=float) / T, 0.0, 1.0)
  h1 = u - 6.0 * u ** 3 + 8.0 * u ** 4 - 3.0 * u ** 5
  h2 = 0.5 * (u ** 2 - 3.0 * u ** 3 + 3.0 * u ** 4 - u ** 5)
  return out + r0 * T * h1 + a0 * T * T * h2


def lane_ramp_step(u):
  u = min(max(u, 0.0), 1.0)
  return u * u * (3.0 - 2.0 * u)

class LanePlanner:
  def __init__(self):
    self.ll_t = np.zeros((TRAJECTORY_SIZE,))
    self.ll_x = np.zeros((TRAJECTORY_SIZE,))
    self.lll_y = np.zeros((TRAJECTORY_SIZE,))
    self.rll_y = np.zeros((TRAJECTORY_SIZE,))
    self.le_y = np.zeros((TRAJECTORY_SIZE,))
    self.re_y = np.zeros((TRAJECTORY_SIZE,))
    #self.lane_width_estimate = FirstOrderFilter(3.2, 9.95, DT_MDL)
    self.lane_width_estimate = FirstOrderFilter(3.2, 3.0, DT_MDL)
    self.lane_width = 3.2
    self.lane_width_last = self.lane_width
    self.lane_change_multiplier = 1
    #self.lane_width_updated_count = 0

    self.lll_prob = 0.
    self.rll_prob = 0.
    self.d_prob = 0.

    self.lll_std = 0.
    self.rll_std = 0.

    self.l_lane_change_prob = 0.
    self.r_lane_change_prob = 0.

    self.debugText = ""
    self.lane_width_left = 0.0
    self.lane_width_right = 0.0
    self.lane_width_left_filtered = FirstOrderFilter(1.0, 1.0, DT_MDL)
    self.lane_width_right_filtered = FirstOrderFilter(1.0, 1.0, DT_MDL)
    self.lane_offset_filtered = FirstOrderFilter(0.0, 2.0, DT_MDL)

    self.lanefull_mode = False
    self.d_prob_count = 0

    # K9LaneFix (experimental, see k9_lane_fix.py). Mode 0 = off: none of the state below is used.
    self.k9_lane_fix_mode = None
    self.k9_lane_fix_reset()

    self.params = Params()

  def k9_lane_fix_reset(self):
    self.lane_converge_e0 = 0.0
    self.lane_merge_k = -1      # F1 merge latch: -1 idle, else model frames since the lane-mode merge was latched
    self.lane_merge_e0 = 0.0    # latched start offset (lane path start - model path start) [m]
    self.lane_merge_t = 0.0     # latched merge time [s]
    self.lane_merge_r0 = 0.0    # latched rate / accel of (lane path - model path) at entry [m/s, m/s^2]
    self.lane_merge_a0 = 0.0
    self.lane_ramp_g = 0.0
    self.lane_ramp_from = 0.0
    self.lane_ramp_k = 0
    self.lane_ramp_phase = 0   # 0 idle, 1 ramp in, 2 on

  def k9_lane_fix_update(self):
    # read every model frame (same as LatMpcInputOffset); a changed number restarts the transition state
    mode = k9_lane_fix_mode(self.params)
    if mode != self.k9_lane_fix_mode:
      self.k9_lane_fix_mode = mode
      self.k9_lane_fix_reset()
    return k9_lane_fix_profile(mode)

  def parse_model(self, md):

    lane_lines = md.laneLines
    edges = md.roadEdges

    if len(lane_lines) >= 4 and len(lane_lines[0].t) == TRAJECTORY_SIZE:
      self.ll_t = (np.array(lane_lines[1].t) + np.array(lane_lines[2].t))/2
      # left and right ll x is the same
      self.ll_x = lane_lines[1].x
      self.lll_y = np.array(lane_lines[1].y)
      self.rll_y = np.array(lane_lines[2].y)
      self.lll_prob = md.laneLineProbs[1]
      self.rll_prob = md.laneLineProbs[2]
      self.lll_std = md.laneLineStds[1]
      self.rll_std = md.laneLineStds[2]

    if len(edges[0].t) == TRAJECTORY_SIZE:
      self.le_y = np.array(edges[0].y) + md.roadEdgeStds[0] * 0.4
      self.re_y = np.array(edges[1].y) - md.roadEdgeStds[1] * 0.4
    else:
      self.le_y = self.lll_y
      self.re_y = self.rll_y

    desire_state = md.meta.desireState
    if len(desire_state):
      self.l_lane_change_prob = desire_state[log.Desire.laneChangeLeft]
      self.r_lane_change_prob = desire_state[log.Desire.laneChangeRight]

  def get_d_path(self, CS, v_ego, path_t, path_xyz, curve_speed):
    #if v_ego > 0.1:
    #  self.lane_width_updated_count = max(0, self.lane_width_updated_count - 1)
    # Reduce reliance on lanelines that are too far apart or
    # will be in a few seconds
    l_prob, r_prob = self.lll_prob, self.rll_prob
    width_pts = self.rll_y - self.lll_y
    prob_mods = []
    for t_check in (0.0, 1.5, 3.0):
      width_at_t = np.interp(t_check * (v_ego + 7), self.ll_x, width_pts)
      #prob_mods.append(np.interp(width_at_t, [4.0, 5.0], [1.0, 0.0]))
      prob_mods.append(np.interp(width_at_t, [4.5, 6.0], [1.0, 0.0]))
    mod = min(prob_mods)
    l_prob *= mod
    r_prob *= mod

    # Reduce reliance on uncertain lanelines
    l_std_mod = np.interp(self.lll_std, [.15, .3], [1.0, 0.0])
    r_std_mod = np.interp(self.rll_std, [.15, .3], [1.0, 0.0])
    l_prob *= l_std_mod
    r_prob *= r_std_mod

    self.l_prob, self.r_prob = l_prob, r_prob

    # Find current lanewidth
    current_lane_width = abs(self.rll_y[0] - self.lll_y[0])

    max_updated_count = 10.0 * DT_MDL
    both_lane_available = False
    #speed_lane_width = np.interp(v_ego*3.6, [0., 60.], [2.8, 3.5])
    if l_prob > 0.5 and r_prob > 0.5 and self.lane_change_multiplier > 0.5:
      both_lane_available = True
      #self.lane_width_updated_count = max_updated_count
      self.lane_width_estimate.update(current_lane_width)
      self.lane_width_last = self.lane_width_estimate.x
    #elif self.lane_width_updated_count <= 0 and v_ego > 0.1:   # 양쪽차선이 없을때.... 일정시간후(10초)부터 speed차선폭 적용함.
    #  self.lane_width_estimate.update(speed_lane_width)
    else:
      self.lane_width_estimate.update(self.lane_width_last)

    self.lane_width =  self.lane_width_estimate.x
    clipped_lane_width = min(4.0, self.lane_width)
    path_from_left_lane = self.lll_y + clipped_lane_width / 2.0
    path_from_right_lane = self.rll_y - clipped_lane_width / 2.0

    # 가장 차선이 진한쪽으로 골라서..
    self.d_prob = max(l_prob, r_prob) if not both_lane_available else 1.0

    # 좌/우의 차선폭을 필터링.
    if self.lane_width_left > 0:
      self.lane_width_left_filtered.update(self.lane_width_left)
      #self.lane_width_left_filtered.x = self.lane_width_left #바로적용
    if self.lane_width_right > 0:
      self.lane_width_right_filtered.update(self.lane_width_right)
      #self.lane_width_right_filtered.x = self.lane_width_right #바로적용

    self.adjustLaneOffset = float(self.params.get_int("AdjustLaneOffset")) * 0.01
    self.adjustCurveOffset = self.adjustLaneOffset #float(self.params.get_int("AdjustCurveOffset")) * 0.01
    ADJUST_OFFSET_LIMIT = 0.4 #max(self.adjustLaneOffset, self.adjustCurveOffset)
    offset_curve = 0.0
    ## curve offset
    offset_curve = np.interp(abs(curve_speed), [50, 200], [self.adjustCurveOffset, 0.0]) * np.sign(curve_speed)

    offset_lane = 0.0
    if self.lane_width_left_filtered.x > 2.2 and self.lane_width_right_filtered.x > 2.2: #양쪽에 차로가 여유 있는경우
      offset_lane = 0.0
    elif self.lane_width_left_filtered.x < 2.0 and self.lane_width_right_filtered.x < 2.0: #양쪽에 차로가 여유 없는경우
      offset_lane = 0.0
    elif self.lane_width_left_filtered.x > self.lane_width_right_filtered.x:
      offset_lane = np.interp(self.lane_width, [2.5, 2.9], [0.0, self.adjustLaneOffset]) # 차선이 좁으면 안함..
    else:
      offset_lane = np.interp(self.lane_width, [2.5, 2.9], [0.0, -self.adjustLaneOffset]) # 차선이 좁으면 안함..

    #select lane path
    # 차선이 좁아지면, 도로경계쪽에 있는 차선 위주로 따라가도록함.
    if self.lane_width < 2.5:
      if r_prob > 0.5 and self.lane_width_right_filtered.x < self.lane_width_left_filtered.x:
        lane_path_y = path_from_right_lane
      elif l_prob > 0.5 and self.lane_width_left_filtered.x < 2.0:
        lane_path_y = path_from_left_lane
      else:
        lane_path_y = path_from_left_lane if l_prob > 0.5 or l_prob > r_prob else path_from_right_lane
    elif l_prob > 0.7 and r_prob > 0.7:
      lane_path_y = (path_from_left_lane + path_from_right_lane) / 2.
      # lane_width filtering에 의해서, 점점 줄어들때, 중앙선으로 붙어가는 현상이 생김.. 
      #if self.lane_width > 3.2:
      #  lane_path_y = path_from_right_lane
      #else:
      #  lane_path_y = (path_from_left_lane + path_from_right_lane) / 2.
    # 그외 진한차선을 따라가도록함.
    else:
      lane_path_y = (l_prob * path_from_left_lane + r_prob * path_from_right_lane) / (l_prob + r_prob + 0.0001)

    use_laneless_center_adjust = False
    if use_laneless_center_adjust:
      ## 0.5초 앞의 중심을 보도록함.
      lane_path_y_center = np.interp(0.5, path_t, lane_path_y)
      path_xyz_y_center = np.interp(0.5, path_t, path_xyz[:,1])
      #lane_path_y_center = lane_path_y[0]
      #path_xyz_y_center = path_xyz[:,1][0]
      diff_center = (lane_path_y_center - path_xyz_y_center) if not self.lanefull_mode else 0.0
    else:
      diff_center = 0.0
    #print("center = {:.2f}={:.2f}-{:.2f}, lanefull={}".format(diff_center, lane_path_y_center, path_xyz_y_center, self.lanefull_mode))
    #diff_center = lane_path_y[5] - path_xyz[:,1][5] if not self.lanefull_mode else 0.0
    if offset_curve * offset_lane < 0:
      offset_total = np.clip(offset_curve + offset_lane + diff_center, - ADJUST_OFFSET_LIMIT, ADJUST_OFFSET_LIMIT)
    else:
      offset_total = np.clip(max(offset_curve, offset_lane, key=abs) + diff_center, - ADJUST_OFFSET_LIMIT, ADJUST_OFFSET_LIMIT)

    ## self.d_prob = 0 if lane_changing
    self.d_prob *= self.lane_change_multiplier  ## 차선변경중에는 꺼버림.
    if self.lane_change_multiplier < 0.5:
      #self.lane_offset_filtered.x = 0.0
      pass
    else:
      self.lane_offset_filtered.update(np.interp(self.d_prob, [0, 0.3], [0, offset_total]))

    ## laneless at lowspeed
    self.d_prob *= np.interp(v_ego*3.6, [5., 10.], [0.0, 1.0])

    #self.debugText = "OFFSET({:.2f}={:.2f}+{:.2f}+{:.2f}),Vc:{:.2f},dp:{:.1f},lf:{},lrw={:.1f}|{:.1f}|{:.1f}".format(
    #  self.lane_offset_filtered.x,
    #  diff_center, offset_lane, offset_curve,
    #  curve_speed,
    #  self.d_prob, self.lanefull_mode,
    #  self.lane_width_left_filtered.x, self.lane_width, self.lane_width_right_filtered.x)

    adjustLaneTime = self.params.get_float("LatMpcInputOffset") * 0.01 # 0.06 
    laneline_active = False
    fix = self.k9_lane_fix_update()
    lane_blended = False
    if fix is not None:
      model_path_y0 = float(path_xyz[0, 1])
      model_path_y = path_xyz[:, 1].copy() if fix["ramp_in_t"] > 0.0 else None
      self.k9_model_path_y_raw = path_xyz[:, 1].copy()   # merge latch: car's lateral motion (model path) at lane-mode entry
      self.lane_converge_e0 = 0.0
    self.d_prob_count = self.d_prob_count + 1 if self.d_prob > 0.3 else 0
    if self.lanefull_mode and self.d_prob_count > int(1 / DT_MDL):
      laneline_active = True
      use_dist_mode = False  ## 아무리생각해봐도.. 같은 방법인듯...
      if use_dist_mode:
        lane_path_y_interp = np.interp(path_xyz[:,0] + v_ego * adjustLaneTime, self.ll_x, lane_path_y)
        path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]
      else:
        safe_idxs = np.isfinite(self.ll_t)
        if safe_idxs[0]:
          lane_path_y_interp = np.interp(path_t * (1.0 + adjustLaneTime), self.ll_t[safe_idxs], lane_path_y[safe_idxs])
          path_xyz[:,1] = self.d_prob * lane_path_y_interp + (1.0 - self.d_prob) * path_xyz[:,1]
          lane_blended = True


    path_xyz[:, 1] += (CAMERA_OFFSET + self.lane_offset_filtered.x)

    self.k9_path_offset_applied = False
    if fix is not None:
      # K9LaneAssist 1 (2026-10-26 conflict audit A1): PathOffset is added here, before the F1 merge, so the merge starts at the
      # car (model path start, without the offset) and ends on lane path + PathOffset; lateral_planner then does not add it
      # again (k9_path_offset_applied). Before, PathOffset was added after F1 to the whole path, so the reference started
      # PathOffset beside the car every frame and the car crept past the offset. The laneless reference (model_path_y, used
      # for the ramp-in) keeps the offset, as the laneless branch output has it.
      po = float(getattr(self, 'k9_path_offset', 0.0))
      if po != 0.0:
        path_xyz[:, 1] += po
        if model_path_y is not None:
          model_path_y = model_path_y + po
      self.k9_path_offset_applied = True
      self.k9_lane_fix_apply(fix, v_ego, path_t, path_xyz, lane_blended, model_path_y0, model_path_y)

    self.offset_total = self.lane_offset_filtered.x

    return path_xyz, laneline_active

  def k9_lane_fix_apply(self, fix, v_ego, path_t, path_xyz, lane_blended, model_path_y0, model_path_y):
    # K9LaneFix reference shaping, only when the lane path was blended into path_xyz (lane-mode branch above).
    # 2026-10-26 recheck_0208 (보노보노 02:35): the merge is latched ONCE when lane mode starts (start = model path start ~ the
    # car, target = lane path, merge time) and then only advances in time; it is not measured again every model frame. Before,
    # e0 was taken again each frame, so the reference always started at the car and the 1 s merge acted as a 1 s feedback
    # loop (closed-loop sim, new K9 spec: 3.5 s period +-0.6 m in lane mode, and a weaker centre pull -> 1-2 cm right with
    # roll). The merge starts with the car's lateral rate / accel relative to the lane path (lane_merge_rates). After the
    # merge time the lane-mode path is exactly the K9LaneAssist 0 path.
    if lane_blended:
      if self.lane_merge_k < 0:
        e0 = float(path_xyz[0, 1]) - model_path_y0
        T = fix["converge_t"]
        if fix["t_eref"] > 0.0:
          cap = fix["t_eref"] * (max(fix["t_max"], T) / T) ** 2
          e0 = float(np.clip(e0, -cap, cap))
        converge_t = T
        if fix["t_eref"] > 0.0:
          # longer merge for a large offset (same peak lateral accel as |e0| = t_eref at T), limited to the time at which
          # the merge curvature stays under t_kref (high speed keeps T), and to t_max
          t_acc = T * np.sqrt(abs(e0) / fix["t_eref"])
          if t_acc >= T:
            t_ext = t_acc
            if fix["t_kref"] > 0.0:
              t_ext = min(t_acc, np.sqrt(5.7735 * abs(e0) / (fix["t_kref"] * max(v_ego, 1.0) ** 2)))
            converge_t = float(np.clip(t_ext, T, max(fix["t_max"], T)))
        if LANE_MERGE_J_REF > 0.0:
          converge_t = max(converge_t, min(float(np.cbrt(60.0 * abs(e0) / LANE_MERGE_J_REF)), max(fix["t_max"], T)))
        self.lane_merge_k, self.lane_merge_e0, self.lane_merge_t = 0, e0, converge_t
        self.lane_merge_r0, self.lane_merge_a0 = lane_merge_rates(path_t, path_xyz[:, 1] - self.k9_model_path_y_raw)
      else:
        self.lane_merge_k += 1
      el = self.lane_merge_k * DT_MDL
      if el < self.lane_merge_t:
        self.lane_converge_e0 = self.lane_merge_e0
        path_xyz[:, 1] -= lane_merge_offset(path_t + el, fix["converge_mode"], self.lane_merge_t, self.lane_merge_e0,
                                            self.lane_merge_r0, self.lane_merge_a0)
    else:
      self.lane_merge_k = -1
    if model_path_y is not None:
      # lane-mode start: ramp the lane-mode correction in over ramp_in_t (laneless_ref = what the laneless branch outputs)
      laneless_ref = model_path_y + (CAMERA_OFFSET + self.lane_offset_filtered.x)
      if lane_blended:
        if self.lane_ramp_phase == 0:
          self.lane_ramp_phase, self.lane_ramp_from, self.lane_ramp_k = 1, self.lane_ramp_g, 0
        delta = path_xyz[:, 1] - laneless_ref
        if self.lane_ramp_phase == 1:
          self.lane_ramp_k += 1
          u = self.lane_ramp_k * DT_MDL / fix["ramp_in_t"]
          self.lane_ramp_g = self.lane_ramp_from + (1.0 - self.lane_ramp_from) * lane_ramp_step(u)
          if u >= 1.0:
            self.lane_ramp_phase, self.lane_ramp_g = 2, 1.0
          path_xyz[:, 1] = laneless_ref + self.lane_ramp_g * delta
      else:
        self.lane_ramp_phase, self.lane_ramp_g = 0, 0.0

  def calculate_plan_yaw_and_yaw_rate(self, path_xyz):
    if path_xyz.shape[0] < 3:
        # 너무 짧으면 직진 가정
        N = path_xyz.shape[0]
        return np.zeros(N), np.zeros(N)

    # x, y 추출
    x = path_xyz[:, 0]
    y = path_xyz[:, 1]

    # 모두 동일한 점인지 확인
    if np.allclose(x, x[0]) and np.allclose(y, y[0]):
        return np.zeros(len(x)), np.zeros(len(x))

    # 안전한 diff 계산
    dx = np.diff(x)
    dy = np.diff(y)
    mask = (dx == 0) & (dy == 0)
    dx[mask] = 1e-4
    dy[mask] = 0.0

    yaw = np.arctan2(dy, dx)
    yaw = np.append(yaw, yaw[-1])  # N-1 → N
    yaw = np.unwrap(yaw)

    dx_full = np.clip(np.diff(x), 1e-4, None)
    yaw_rate = np.diff(yaw) / dx_full
    yaw_rate = np.append(yaw_rate, yaw_rate[-1])
    yaw_rate = np.append(yaw_rate, 0.0)

    # NaN/Inf 방어
    if np.any(np.isnan(yaw_rate)) or np.any(np.isinf(yaw_rate)):
        yaw_rate = np.zeros_like(yaw_rate)
    if np.any(np.isnan(yaw)) or np.any(np.isinf(yaw)):
        yaw = np.zeros_like(yaw)

    return yaw, yaw_rate
