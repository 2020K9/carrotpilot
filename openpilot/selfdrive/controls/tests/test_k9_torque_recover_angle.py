"""K9 torque handback, 2026-11-08: carState.steeringRateDeg is unsigned on the K9 (SAS11 SAS_Speed) and the driver
giving way while still on the wheel (11-06 17:54 rotary replay). k9_torque_recover.py set_angle / HOLD_RUN0..1 / HOLD_FALL."""
from types import SimpleNamespace as NS

import numpy as np

from opendbc.car.lateral import apply_driver_steer_torque_limits
from openpilot.selfdrive.controls.lib import k9_torque_recover as ktr
from openpilot.selfdrive.controls.lib.k9_torque_recover import K9TorqueRecover

LIM = NS(STEER_MAX=409, STEER_DELTA_UP=5, STEER_DELTA_DOWN=10, STEER_DRIVER_ALLOWANCE=50,
         STEER_DRIVER_MULTIPLIER=2, STEER_DRIVER_FACTOR=1)


def run(seq, rec=None, v=4.5, y_std=0.3):
  """seq: (request, driver torque, pressed, angle deg, unsigned rate deg/s) -> applied (base, new)"""
  rec = rec or K9TorqueRecover(LIM)
  last_b = last_n = 0
  base, new = [], []
  for req, tq, p, ang, rate in seq:
    b = apply_driver_steer_torque_limits(int(round(req * 409)), last_b, tq, LIM)
    rec.set_angle(ang)
    r = rec.update(req, last_n / 409., tq, p, v, True, y_std, rate)
    n = apply_driver_steer_torque_limits(int(round(r * 409)), last_n, tq, LIM)
    base.append(b)
    new.append(n)
    last_b, last_n = b, n
  return np.array(base), np.array(new)


def test_unsigned_rate_gets_the_angle_direction():
  rec = K9TorqueRecover(LIM)
  assert rec.signed_rate(-30.0) == -30.0  # no angle given: the caller's sign is kept
  rec.set_angle(10.0)
  rec.set_angle(9.0)
  assert rec.signed_rate(120.0) == -120.0
  rec.set_angle(9.0)  # same angle (slow CAN update): keeps the last direction
  assert rec.signed_rate(120.0) == -120.0
  rec.set_angle(9.5)
  assert rec.signed_rate(120.0) == 120.0
  assert rec.signed_rate(None) is None


def release_17_54_10(n_hold=150):
  """17:54:09.9~11.7 shape: request -1 (right), driver holds left 270 while the wheel slowly gives way to the right,
  then lets go and the wheel runs right at 240 deg/s. Rate unsigned as on the car."""
  seq, ang = [], 190.0
  for _ in range(100):
    seq.append((-1.0, 270.0, True, ang, 0.0))
  for i in range(n_hold):  # giving way: wheel turns right 20..90 deg/s while he still pushes left
    r = 20.0 + 70.0 * i / n_hold
    ang -= r * 0.01
    seq.append((-1.0, float(np.interp(i, [0, n_hold], [250, 160])), True, ang, r))
  for i in range(80):  # lets go: the wheel spins right
    ang -= 240.0 * 0.01
    seq.append((-1.0, float(np.interp(i, [0, 30], [140, -30])), False, ang, 240.0))
  return seq


def test_giving_way_then_release_stays_low():
  seq = release_17_54_10()
  base, new = run(seq)
  rel = next(i for i, s in enumerate(seq) if not s[2])
  assert np.abs(base[rel:rel + 50]).max() > 0.5 * 409  # without it the wheel is pushed on the way out
  assert np.abs(new[rel:rel + 50]).max() <= 0.05 * 409


def test_signed_rate_would_hide_the_unwind_without_angle():
  # the same drive with the unsigned rate and no angle (the 4dd24e56 call): the wheel running right reads as left,
  # so the handback is not stopped; with the angle it is (the fix)
  seq = release_17_54_10()
  rec = K9TorqueRecover(LIM)
  last = 0
  out = []
  for req, tq, p, _ang, rate in seq:
    r = rec.update(req, last / 409., tq, p, 4.5, True, 0.3, rate)
    last = apply_driver_steer_torque_limits(int(round(r * 409)), last, tq, LIM)
    out.append(last)
  rel = next(i for i, s in enumerate(seq) if not s[2])
  _, new = run(seq)
  assert np.abs(new[rel:rel + 50]).max() < np.abs(np.array(out[rel:rel + 50])).max()


def test_still_wheel_hold_keeps_push_back():
  # he holds with the wheel still (rotary tracking): push-back still grows to HOLD_MAX
  seq = [(-1.0, 120.0, True, 30.0, 0.0)] * 300
  rec = K9TorqueRecover(LIM)
  run(seq, rec=rec)
  assert rec.hold_cap == ktr.HOLD_MAX


def test_hold_against_wheel_moving_his_way_keeps_push_back():
  # wheel turning the driver's way (left, against a right request) is him overpowering: not giving way
  seq, ang = [], 0.0
  for _ in range(300):
    ang += 0.5
    seq.append((-1.0, 200.0, True, ang, 50.0))
  rec = K9TorqueRecover(LIM)
  run(seq, rec=rec)
  assert rec.hold_cap == ktr.HOLD_MAX


def test_controlsd_passes_angle():
  from pathlib import Path
  src = (Path(__file__).resolve().parents[1] / "controlsd.py").read_text(encoding="utf-8")
  assert "self.k9_torque_recover.set_angle(CS.steeringAngleDeg)" in src
