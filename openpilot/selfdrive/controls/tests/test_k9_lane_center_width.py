"""K9LaneCenter lane-width admission (W_MIN 2.6) and preserved gates. Synthetic model frames, no vehicle data."""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import k9_lane_center as H

X = 192.0 * (np.arange(33) / 32.0) ** 2
_N = [0]


def frame(w, e, p=0.9, std=0.05):
  """straight lane of width w, car e m right of the centre (right +)"""
  yl = np.full(33, -w / 2.0 - e)
  yr = yl + w        # exact width at every point
  _N[0] += 1
  lines = [NS(x=X.tolist(), y=(yl - 3.6).tolist()), NS(x=X.tolist(), y=yl.tolist()),
           NS(x=X.tolist(), y=yr.tolist()), NS(x=X.tolist(), y=(yr + 3.6).tolist())]
  return NS(frameId=_N[0], laneLines=lines, laneLineProbs=[0.5, p, p, 0.5], laneLineStds=[0.1, std, std, 0.1])


def run(h, w, e, seconds=4.0, v=20.0, laneless=True, lane_change=False, pressed=False, p=0.9, std=0.05, stale_after=None):
  out, md = [], None
  for k in range(int(round(seconds / H.DT))):
    if k % 5 == 0 and (stale_after is None or k * H.DT < stale_after):
      md = frame(w, e, p=p, std=std)
    out.append(h.update(md, v, laneless, lane_change, pressed, 0.0))
  return np.array(out)


@pytest.mark.parametrize("w", [2.6, 2.7, 2.8, 3.5, 4.2])
@pytest.mark.parametrize("e", [0.3, -0.3])
def test_supported_width_engages(w, e):
  h = H.LaneCenter()
  out = run(h, w, e)
  assert h.active and h.why == 0
  assert abs(out[-1]) > 1e-6
  assert np.sign(out[-1]) == -np.sign(e)   # pulls back toward the centre


@pytest.mark.parametrize("w", [2.59, 4.21])
@pytest.mark.parametrize("e", [0.3, -0.3])
def test_out_of_range_width_rejected(w, e):
  h = H.LaneCenter()
  out = run(h, w, e)
  assert not h.active and h.why == 3
  assert np.all(out == 0.0)


@pytest.mark.parametrize("p,std", [(0.65, 0.05), (0.9, 0.25)])
def test_line_prob_std_gates_on(p, std):
  h = H.LaneCenter()
  assert np.all(run(h, 3.5, 0.3, p=p, std=std) == 0.0) and h.why == 1   # P_ON 0.7 not reached / std > STD_MAX


def test_prob_hysteresis_off():
  h = H.LaneCenter()
  run(h, 3.5, 0.3)
  assert h.active
  run(h, 3.5, 0.3, seconds=0.5, p=0.65)     # above P_OFF: still reliable
  assert h.active
  run(h, 3.5, 0.3, seconds=0.5, p=0.55)     # below P_OFF: off
  assert not h.active and h.why == 1


def test_stale_model_input_releases():
  h = H.LaneCenter()
  run(h, 3.0, 0.3, seconds=4.0, stale_after=3.0)
  assert not h.active


@pytest.mark.parametrize("kw", [dict(lane_change=True), dict(pressed=True), dict(laneless=False)])
def test_lane_change_blinker_pressed_lane_mode_release(kw):
  h = H.LaneCenter()
  run(h, 3.0, 0.3)
  assert h.active
  out = run(h, 3.0, 0.3, seconds=3.0, **kw)
  assert not h.active and out[-1] == 0.0
  assert np.all(np.abs(np.diff(out)) <= 1e-12 + min(H.J_MAX / 20.0 ** 2, H.KR_MAX) * H.DT)


@pytest.mark.parametrize("v", [3.0, 10.0, 25.0])
def test_output_cap_and_rate_limit(v):
  h = H.LaneCenter()
  out = run(h, 3.0, 0.55, seconds=8.0, v=v)
  v2 = max(v, H.V_MIN) ** 2
  assert np.max(np.abs(out)) <= min(H.A_MAX / v2, H.K_MAX) + 1e-12
  assert np.max(np.abs(np.diff(out))) <= min(H.J_MAX / v2, H.KR_MAX) * H.DT + 1e-12


def test_large_offset_is_off():
  h = H.LaneCenter()
  assert np.all(run(h, 3.0, H.E_OFF + 0.05) == 0.0) and h.why == 7


def test_off_param_returns_zero():
  c = NS(params=NS(get_int=lambda k: 0))
  assert H.k9_lane_center_step(c, None, None, None, 0) == 0.0 and c.k9_lc is None
