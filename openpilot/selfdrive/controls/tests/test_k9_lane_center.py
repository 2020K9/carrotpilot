"""K9LaneCenter (controls/lib/k9_lane_center.py) behaviour tests. Synthetic model frames, no vehicle data."""
from types import SimpleNamespace as NS

import numpy as np

from openpilot.selfdrive.controls.lib import k9_lane_center as H

X = 192.0 * (np.arange(33) / 32.0) ** 2
DT = H.DT


def frame(y_left, y_right, p_left=0.9, p_right=0.9, std=0.05, slope=0.0, w_far_add=0.0):
  """model frame: straight lines at lateral y (right +) relative to the car; w_far_add widens the right line ahead (split)"""
  yl = y_left + slope * X
  yr = y_right + slope * X + w_far_add * np.clip(X / 30.0, 0.0, 1.0)
  lines = [NS(x=X.tolist(), y=(yl - 3.6).tolist()), NS(x=X.tolist(), y=yl.tolist()),
           NS(x=X.tolist(), y=yr.tolist()), NS(x=X.tolist(), y=(yr + 3.6).tolist())]
  frame.n += 1
  return NS(frameId=frame.n, laneLines=lines, laneLineProbs=[0.5, p_left, p_right, 0.5], laneLineStds=[0.1, std, std, 0.1],
            position=NS(x=X.tolist(), y=np.zeros(33).tolist()), meta=NS(laneChangeState=0), action=NS(desiredCurvature=0.0))


frame.n = 0


def world(half=1.75, **kw):
  """world-fixed straight lane (centre 0, width 2*half) seen from the car at lateral y (right +) with heading p (right +)"""
  return lambda y, p: frame(-half - y, half - y, slope=-p, **kw)


def run(h, frames_fn, seconds, v=20.0, laneless=True, lane_change=False, pressed=False, y0=0.0, plant=False, bias=0.0, pull=True,
        model_curv=0.0):
  """100 Hz, a model frame every 5 ticks. plant=True: the car follows the command; the laneless model is the weak recentring
  measured on 2026-10-25 (dc = c0 * y + h * psi, c0 = -1.0e-3 1/m per m, h = -0.055) plus a constant curvature bias."""
  y, psi, out, ys, md = y0, 0.0, [], [], None
  for k in range(int(round(seconds / DT))):
    if k % 5 == 0:
      md = frames_fn(y, psi)
    kk = h.update(md, v, laneless, lane_change, pressed, model_curv)
    out.append(kk)
    if plant:
      cmd = kk + bias + ((-1.0e-3 * y - 0.055 * psi) if pull else 0.0)
      psi += v * cmd * DT
      y += v * psi * DT
    ys.append(y)
  return np.array(out), np.array(ys)


def test_mode_read():
  class P:
    def __init__(self, v): self.v = v
    def get_int(self, k):
      if isinstance(self.v, Exception): raise self.v
      return self.v
  assert H.k9_lane_center_mode(P(0)) == 0 and H.k9_lane_center_mode(P(1)) == 1
  assert H.k9_lane_center_mode(P(2)) == 0 and H.k9_lane_center_mode(P(-1)) == 0 and H.k9_lane_center_mode(P(KeyError())) == 0


def test_controlsd_off_path_is_the_old_expression():
  import openpilot.selfdrive.controls.controlsd as C
  with open(C.__file__) as f:
    src = f.read()
  assert "      if k9_lane_add != 0.0:\n        dc_model = dc_model + k9_lane_add" in src   # 23:38: one mixed add (k9_lane_mix)
  assert "k9_lane_mix_step(self, k9_lc_add, k9_olh_add, CS.vEgo)" in src
  assert "new_desired_curvature = smooth_value(dc_model, self.desired_curvature, 0.1)" in src
  assert "self.curvature = -self.VM.calc_curvature(" in src   # right-positive curvature, the sign the hold is written for
  assert "k9_lc_add = k9_lane_center_step(self, CS, CC, model_v2, LaneChangeState.off)" in src
  c = C.Controls.__new__(C.Controls)
  c.params = NS(get_int=lambda k: 0)
  assert H.k9_lane_center_step(c, None, None, None, 0) == 0.0 and c.k9_lc is None


def test_centred_car_gets_nothing():
  out, _ = run(H.LaneCenter(), world(), 5.0, y0=0.0)
  assert np.all(out == 0.0)
  out, _ = run(H.LaneCenter(), world(), 5.0, y0=H.DB * 0.8)   # inside the dead band
  assert np.max(np.abs(out)) < 1e-9


def test_car_right_of_centre_is_pulled_left_sign_and_caps():
  out, _ = run(H.LaneCenter(), world(), 4.0, y0=0.25)
  assert out.min() < -1e-5 and out.max() <= 1e-12                # right of centre -> negative (left) curvature
  assert np.max(np.abs(out)) * 400 <= H.A_MAX + 1e-9
  assert np.max(np.abs(np.diff(out))) <= H.J_MAX / 400 * DT + 1e-12
  assert np.all(out[:int(H.BOTH_T / DT)] == 0.0)               # lines watched first
  out_l, _ = run(H.LaneCenter(), world(), 4.0, y0=-0.25)
  assert out_l.max() > 1e-5 and out_l.min() >= -1e-12


def test_closed_loop_bias_offset_is_removed():
  # 2e-4 1/m right bias: with the weak model recentring alone the car settles ~20 cm right (the 1ca 17:50 case)
  _, ys0 = run(H.LaneCenter(), world(), 60.0, v=8.0, plant=True, bias=2e-4, laneless=False)
  _, ys1 = run(H.LaneCenter(), world(), 60.0, v=8.0, plant=True, bias=2e-4)
  assert ys0[-1] > 0.15
  assert ys1[-1] < 0.08 and np.mean(ys1[-1000:]) < 0.08
  _, ys2 = run(H.LaneCenter(), world(), 90.0, v=22.0, plant=True, bias=1e-4)
  assert ys2[-1] < 0.08


def test_closed_loop_no_overshoot_from_offset():
  out, ys = run(H.LaneCenter(), world(), 40.0, v=15.0, plant=True, y0=0.3)
  assert abs(ys[-1]) < H.DB + 0.03
  assert ys.min() > -0.10                                       # does not swing through to the other side
  assert np.max(np.abs(out)) * 225 <= H.A_MAX + 1e-9


def test_off_in_lane_change_pressed_lane_mode_low_speed_turn():
  for kw in (dict(lane_change=True), dict(laneless=False), dict(pressed=True), dict(v=0.2), dict(model_curv=0.01),
             dict(model_curv=1.6 / 400.0)):
    out, _ = run(H.LaneCenter(), world(), 4.0, y0=0.3, **kw)
    assert np.max(np.abs(out)) < 1e-12, kw


def test_weak_or_missing_line_and_bad_width_do_nothing():
  for fn in (lambda y, p: frame(-1.75 - y, 1.75 - y, p_right=0.3), lambda y, p: frame(-1.75 - y, 1.75 - y, std=0.4),
             world(half=1.2), world(half=2.3), world(w_far_add=0.8)):
    out, _ = run(H.LaneCenter(), fn, 4.0, y0=0.3)
    assert np.max(np.abs(out)) < 1e-12


def test_ramp_out_is_smooth_and_ends_at_exact_zero():
  h = H.LaneCenter()
  out1, _ = run(h, world(), 4.0, y0=0.3)
  assert out1[-1] < -1e-5
  out2, _ = run(h, world(), 3.0, y0=0.3, lane_change=True)
  assert np.all(np.sign(out2[out2 != 0.0]) == np.sign(out1[-1]))   # only shrinks, never changes sign
  assert np.all(np.abs(np.diff(np.concatenate([[out1[-1]], out2]))) <= H.J_MAX / 400 * DT + 1e-12)
  assert out2[-1] == 0.0


def test_width_jump_stops_and_rewatches():
  h = H.LaneCenter()
  run(h, world(), 3.0, y0=0.3)
  out, _ = run(h, world(half=2.05), 0.3, y0=0.3)   # the right line jumps 0.6 m (branch): measurement dropped at once
  assert not h.meas_ok
  assert np.all(np.diff(np.abs(out)) <= 1e-12)


def test_jumpy_lane_estimate_never_pushes_away_from_centre():
  rng = np.random.default_rng(3)
  fn = lambda y, p: frame(-1.75 - y + rng.normal(0, 0.02), 1.75 - y + rng.normal(0, 0.02), slope=-p)
  out, ys = run(H.LaneCenter(), fn, 40.0, v=20.0, plant=True, y0=0.25)
  # pushes are toward the centre side of the car's offset (allow tiny damping terms near the dead band)
  bad = (out * np.sign(ys) > 0.0) & (np.abs(ys) > H.DB + 0.05)
  assert not bad.any()
  assert abs(ys[-1]) < 0.12
