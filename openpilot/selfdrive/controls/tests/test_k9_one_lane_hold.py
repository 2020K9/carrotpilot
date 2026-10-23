"""K9OneLaneHold (controls/lib/k9_one_lane_hold.py) behaviour tests. Synthetic model frames, no vehicle data."""
from types import SimpleNamespace as NS

import numpy as np

from openpilot.selfdrive.controls.lib import k9_one_lane_hold as H

X = 192.0 * (np.arange(33) / 32.0) ** 2
DT = H.DT


def frame(y_left, y_right, p_left=0.9, p_right=0.9, std=0.05, left_curv=0.0, path_y=None, slope=0.0):
  """model frame: lines at lateral y (right +) relative to the car, straight unless left_curv (1/m, bends the left line)"""
  yl = y_left + 0.5 * left_curv * X ** 2 + slope * X
  yr = y_right + slope * X
  lines = [NS(x=X.tolist(), y=(yl - 3.6).tolist(), t=X.tolist()), NS(x=X.tolist(), y=yl.tolist(), t=X.tolist()),
           NS(x=X.tolist(), y=yr.tolist(), t=X.tolist()), NS(x=X.tolist(), y=(yr + 3.6).tolist(), t=X.tolist())]
  py = np.zeros(33) if path_y is None else path_y
  frame.n += 1
  return NS(frameId=frame.n, laneLines=lines, laneLineProbs=[0.5, p_left, p_right, 0.5], laneLineStds=[0.1, std, std, 0.1],
            position=NS(x=X.tolist(), y=py.tolist()), meta=NS(laneChangeState=0), action=NS(desiredCurvature=0.0))


frame.n = 0


def run(h, frames_fn, seconds, v=20.0, laneless=True, lane_change=False, y0=0.0, plant=False, pull=False):
  """drive for seconds at 100 Hz, a new model frame every 5 ticks; plant=True integrates the added curvature (model command 0).
  Plant sign check: positive curvature (right +, as controlsd) moves the car right."""
  y, psi, out, ys, md = y0, 0.0, [], [], None
  for k in range(int(round(seconds / DT))):
    if k % 5 == 0:
      md = frames_fn(y, psi)
    kk = h.update(md, v, laneless, lane_change, False, 0.0)
    out.append(kk)
    if plant:
      if pull:   # laneless model recentring on its own path (y = 0), as the closed-loop harness (TL 2.5 s)
        kk = kk - 2.0 * y / (v * 2.5) ** 2 - 2.0 * psi / (v * 2.5)
      psi += v * kk * DT          # openpilot convention: curvature, heading and y all right-positive
      y += v * psi * DT
    ys.append(y)
  return np.array(out), np.array(ys)


def world(p_right=0.9, half_r=1.8, **kw):
  """world-fixed straight lines at -1.8 / +half_r m, seen from the car at lateral y (right +) with heading p (right +)"""
  return lambda y, p: frame(-1.8 - y, half_r - y, p_right=p_right, slope=-p, **kw)


def test_mode_read():
  class P:
    def __init__(self, v): self.v = v
    def get_int(self, k):
      if isinstance(self.v, Exception): raise self.v
      return self.v
  assert H.k9_one_lane_hold_mode(P(0)) == 0 and H.k9_one_lane_hold_mode(P(1)) == 1
  assert H.k9_one_lane_hold_mode(P(2)) == 0 and H.k9_one_lane_hold_mode(P(-1)) == 0 and H.k9_one_lane_hold_mode(P(KeyError())) == 0


def warm(y0=-0.4):
  h = H.OneLaneHold()
  out, _ = run(h, world(), 3.0, y0=y0)
  assert np.all(out == 0.0)
  return h


def test_both_lines_measure_only_and_remember_centre():
  h = warm(-0.3)
  assert abs(h.mem["target"] - (1.8 - H.HALF_W)) < 0.02
  assert abs(h.mem["min_gap"] - (1.5 - H.HALF_W)) < 0.02


def test_one_line_pushes_back_to_centre_sign_and_caps():
  h = warm()
  out, _ = run(h, world(p_right=0.05), 3.0, y0=-0.4)   # right line gone, car 0.4 m left: push right = positive curvature (right +)
  assert out.max() > 1e-4 and out.min() >= -1e-12
  assert np.max(np.abs(out)) * 400 <= H.A_MAX_CLOSE + 1e-9
  assert np.max(np.abs(np.diff(out))) <= H.J_MAX / 400 * DT + 1e-12


def test_closed_loop_one_line_converges_without_crossing_min_gap():
  h = warm(-0.35)
  out, ys = run(h, world(p_right=0.05), 25.0, y0=-0.35, plant=True)
  assert abs(ys[-1]) < 0.08                       # back near the centre
  assert ys.max() < 0.15 and ys.min() >= -0.36    # no big overshoot, never closer to the left line than at the start
  assert np.max(np.abs(out)) * 400 <= H.A_MAX_CLOSE + 1e-9


def test_off_in_lane_change_and_lane_mode():
  for kw in (dict(lane_change=True), dict(laneless=False)):
    h = warm()
    out, _ = run(h, world(p_right=0.05), 2.0, y0=-0.4, **kw)
    assert np.all(out == 0.0)


def test_ramp_out_is_smooth_and_ends_at_exact_zero():
  h = warm()
  out1, _ = run(h, world(p_right=0.05), 3.0, y0=-0.4)
  out2, _ = run(h, world(p_right=0.05), 3.0, y0=-0.4, lane_change=True)
  seq = np.concatenate([out1, out2])
  assert np.max(np.abs(np.diff(seq))) <= H.J_MAX / 400 * DT + 1e-12
  assert out2[-1] == 0.0 and h.integ == 0.0


def test_memory_expires():
  h = warm()
  out, _ = run(h, world(p_right=0.05), H.MEM_D / 20.0 + 2.5, y0=-0.4)
  assert h.mem is None and out[-1] == 0.0


def test_lane_width_jump_drops_memory():
  h = warm()
  run(h, world(half_r=2.6), 0.3, y0=-0.4)          # merge: width 3.6 -> 4.4 in one frame
  out, _ = run(h, world(p_right=0.05, half_r=2.6), 2.0, y0=-0.4)
  assert h.mem is None and np.all(out == 0.0)


def test_diverging_line_is_not_followed():
  h = warm()
  out, _ = run(h, world(p_right=0.05, left_curv=-0.004), 2.0, y0=-0.4)   # left line bends away (split)
  assert np.all(out == 0.0)


def test_jittery_line_is_not_followed():
  h = warm()
  rng = np.random.default_rng(1)
  out, _ = run(h, lambda y, p: frame(-1.4 + rng.normal(0, 0.15), 2.2, p_right=0.05), 3.0, y0=-0.4)
  assert np.max(np.abs(out)) * 400 < 0.05


def test_controlsd_off_path_is_the_old_expression():
  # mode 0: k9_one_lane_hold_step returns 0.0 and the laneless branch is the 1d0961da expression
  import openpilot.selfdrive.controls.controlsd as C
  with open(C.__file__) as f:
    src = f.read()
  assert "      if k9_olh_add != 0.0:\n        dc_model = dc_model + k9_olh_add" in src
  assert "new_desired_curvature = smooth_value(dc_model, self.desired_curvature, 0.1)" in src
  assert "self.curvature = -self.VM.calc_curvature(" in src   # right-positive curvature, the sign the hold is written for
  c = C.Controls.__new__(C.Controls)
  c.params = NS(get_int=lambda k: 0)
  assert H.k9_one_lane_hold_step(c, None, None, None, 0) == 0.0 and c.k9_olh is None
  assert "k9_olh_add = k9_one_lane_hold_step(self, CS, CC, model_v2, LaneChangeState.off)" in src


def test_receding_line_is_released():
  # left line moves away 0.25 m/s (lane widening / split, parallel shift so the shape guards do not see it)
  h = warm(0.0)
  n = {"k": 0}
  def fr(y, p):
    n["k"] += 1
    return frame(-1.8 - y - 0.25 * min(n["k"] * 0.05, 3.0), 1.8 - y, p_right=0.05, slope=-p)
  out, ys = run(h, fr, 14.0, y0=0.0, plant=True, pull=True)
  assert h.mem is None and out[-1] == 0.0
  assert ys.min() > -0.3          # followed the receding line (0.75 m) by less than 0.3 m
  assert abs(ys[-1]) < 0.1        # and back on the model path


def test_line_moving_by_itself_is_dropped_fast():
  # r3: right line gone and at the same moment the left line moves away 0.233 m/s for 3 s (branch line, parallel shift)
  h = warm(0.0)
  n = {"k": 0}
  def fr(y, p):
    n["k"] += 1
    return frame(-1.8 - y - 0.7 * min(n["k"] * 0.05 / 3.0, 1.0), 1.8 - y, p_right=0.05, slope=-p)
  out, ys = run(h, fr, 8.0, y0=0.0, plant=True, pull=True)
  assert h.mem is None and out[-1] == 0.0
  t_drop = None
  h2 = warm(0.0); n["k"] = 0
  for k in range(400):
    run(h2, fr, 0.05, y0=0.0)
    if h2.mem is None:
      t_drop = (k + 1) * 0.05; break
  assert t_drop is not None and t_drop <= 1.5      # unexplained line motion > C_OFF within 1.5 s (0.35 m moved)
  assert ys.min() > -0.08                          # followed the moving line by less than 8 cm


def test_static_line_with_car_offset_is_not_released():
  # r3: the hold's own motion (car moves toward the centre) must not look like a moving line
  h = warm(-0.35)
  out, ys = run(h, world(p_right=0.05), 8.0, y0=-0.35, plant=True, pull=True)
  assert h.mem is not None and h.active
  assert max(abs(h.cu[0]), abs(h.cu[1])) < H.C_OFF
  assert abs(ys[-1]) < 0.2



def test_r4_hold_adds_no_jerk_while_the_model_command_jerks():
  # r4: the model command (laneless desiredCurvature) jerks +0.5 m/s^3 for 2 s: the hold may not add jerk in that direction
  h = warm(-0.4)
  v = 20.0
  out = []
  md = None
  for k in range(800):
    if k % 5 == 0:
      md = world(p_right=0.05)(-0.4, 0.0)
    t = k * DT
    m_k = (0.5 * min(max(t - 3.0, 0.0), 2.0)) / v ** 2     # model lateral accel ramps 0 -> 1 m/s^2 between 3 s and 5 s
    out.append(h.update(md, v, True, False, False, m_k))
  out = np.array(out)
  d = np.diff(out)
  tk = np.arange(1, 800) * DT
  sel = (tk > 3.4) & (tk < 5.0)                            # filtered model command jerk above J_Q1
  assert np.max(np.abs(out)) > 1e-4                      # the hold is pushing (right +, car 0.4 m left)
  assert np.all(d[sel] <= 1e-12)                          # no added jerk in the model command direction
  assert np.max(np.abs(d)) <= H.J_MAX / v ** 2 * DT + 1e-12


def test_r4_short_measurement_gap_does_not_restart_the_probe():
  # r4: the visible line drops out for 0.2 s (prob 0.3): the hold resumes at once instead of waiting T_PROBE again
  h = warm(-0.4)
  run(h, world(p_right=0.05), 2.0, y0=-0.4)
  assert h.active
  run(h, lambda y, p: frame(-1.8 - y, 1.8 - y, p_left=0.3, p_right=0.05), 0.2, y0=-0.4)
  assert not h.active
  run(h, world(p_right=0.05), 0.1, y0=-0.4)
  assert h.active                                         # within 0.1 s, not T_PROBE (0.6 s)


def test_r4_moved_line_cuts_the_push_even_when_the_line_is_lost_next():
  # r4: the visible line moves 0.2 m by itself in one frame (watch fires) and is lost right after: the push falls at the
  # full jerk limit from the next tick (no 1 s ramp-out with the old target)
  h = warm(-0.3)
  run(h, world(p_right=0.05), 2.0, y0=-0.3)                # one line, still: the hold pushes
  k0 = abs(h.k_out)
  assert h.active and k0 * 400 > 0.02
  run(h, lambda y, p: frame(-2.0 - y, 1.8 - y, p_right=0.05), 0.01, y0=-0.3)
  assert h.mem is None
  out2, _ = run(h, lambda y, p: frame(-2.0 - y, 1.8 - y, p_left=0.3, p_right=0.05), 2.0, y0=-0.3)
  dk = H.J_MAX / 400 * DT
  steps = np.arange(1, len(out2) + 1)
  assert np.all(np.abs(out2) <= np.maximum(k0 - steps * dk, 0.0) + dk + 1e-12)
  assert out2[-1] == 0.0


def test_r5_never_pushes_away_from_the_centre_target():
  # r5: car 0.4 m left of centre, right line gone, the car already moves back toward the centre at 0.3 m/s on its own (the
  # laneless model recentres it) after the hold pushed for 2 s. The line-rate damping may only take back the lateral velocity the hold itself added:
  # summed over time the hold never pushes away from the centre target (r4 braked the model's own recentring)
  h = warm(-0.4)
  out, _ = run(h, world(p_right=0.05), 2.0, y0=-0.4)       # one line, car still 0.4 m left: the hold pushes right
  out = list(out)
  md = None
  for k in range(130):
    y = -0.4 + 0.3 * k * DT
    if k % 5 == 0:
      md = world(p_right=0.05)(y, 0.3 / 20.0)              # heading matches the lateral motion (line slope)
    out.append(h.update(md, 20.0, True, False, False, 0.0))
  out = np.array(out)
  assert np.min(np.cumsum(out)) * 400 * DT >= -0.002     # the hold's own lateral velocity share never points away from the centre


def test_r5_jerk_aware_does_not_block_the_push_from_shrinking():
  # r5: the hold pushes left (car right of centre), then the visible line moves 0.2 m by itself (watch fires, push cut)
  # while the model command jerks right (+0.25 m/s^3). The cut push must still fall toward 0 at the jerk limit (r4 froze it:
  # rising toward 0 counted as 'same direction' as the model jerk)
  h = warm(0.4)
  run(h, world(p_right=0.05), 2.0, y0=0.4)
  v = 20.0
  k0 = h.k_out
  assert h.active and k0 * v ** 2 < -0.02
  out = []
  md = None
  for k in range(100):
    if k % 5 == 0:
      md = frame(-2.0 - 0.4, 1.8 - 0.4, p_right=0.05)
    m_k = 0.25 * k * DT / v ** 2                         # model lateral accel ramps +0.25 m/s^3 (r4 held the fall to 1/4)
    out.append(h.update(md, v, True, False, False, m_k))
  out = np.array(out)
  assert h.mem is None
  dk = H.J_MAX / v ** 2 * DT
  assert out[39] >= min(k0 + 30 * dk, 0.0) - 1e-12       # fell at least 30 jerk-limit steps in the first 0.4 s (or reached 0)
  assert np.max(np.abs(np.diff(out))) <= dk + 1e-12
