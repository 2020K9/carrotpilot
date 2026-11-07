"""K9 torque handback with the wheel speed known (k9_torque_recover.py, 2026-11-07 fix of the 11-06 17:54 rotary).

controlsd passes carState.steeringRateDeg. With it the handback is governed by the wheel itself: while the driver
holds against, openpilot may still push back up to HOLD_MAX; after he lets go the envelope rises over T_FAST only while
the wheel is not already running the request's way, so the wheel cannot spring back fast either way."""
from types import SimpleNamespace as NS

import numpy as np

from opendbc.car.lateral import apply_driver_steer_torque_limits
from openpilot.selfdrive.controls.lib import k9_torque_recover as ktr
from openpilot.selfdrive.controls.lib.k9_torque_recover import K9TorqueRecover, driver_cut

LIM = NS(STEER_MAX=409, STEER_DELTA_UP=5, STEER_DELTA_DOWN=10, STEER_DRIVER_ALLOWANCE=50,
         STEER_DRIVER_MULTIPLIER=2, STEER_DRIVER_FACTOR=1)


def run(seq, v=4.5, y_std=0.3, rec=None):
  """seq: (request, driver torque, pressed, wheel rate deg/s). Returns applied car torque base/new and requests."""
  rec = rec or K9TorqueRecover(LIM)
  last_b = last_n = 0
  base, new, out = [], [], []
  for req, tq, pressed, rate in seq:
    b = apply_driver_steer_torque_limits(int(round(req * 409)), last_b, tq, LIM)
    r = rec.update(req, last_n / 409., tq, pressed, v, True, y_std, rate)
    n = apply_driver_steer_torque_limits(int(round(r * 409)), last_n, tq, LIM)
    base.append(b)
    new.append(n)
    out.append(r)
    last_b, last_n = b, n
  return np.array(base), np.array(new), np.array(out)


def rotary_release(rate_after):
  seq = [(-1.0, 310.0, True, 0.0)] * 180
  seq += [(-1.0, float(t), t > 150, rate_after) for t in np.linspace(310, 224, 25)]
  seq += [(-1.0, float(t), t > 150, rate_after) for t in np.linspace(224, -38, 50)]
  seq += [(-1.0, 0.0, False, rate_after)] * 400
  return seq


def test_no_driver_is_unchanged_with_rate():
  rng = np.random.default_rng(0)
  seq = [(float(r), float(t), False, float(w)) for r, t, w in zip(rng.uniform(-1, 1, 3000), rng.normal(0, 15, 3000),
                                                                    rng.normal(0, 200, 3000), strict=True)]
  base, new, _ = run(seq)
  assert (base == new).all()


def test_request_never_larger_never_flips_with_rate():
  rng = np.random.default_rng(1)
  rec = K9TorqueRecover(LIM)
  last = 0
  for _ in range(60):
    req = float(rng.choice([-1.0, 1.0]))
    tq = float(-np.sign(req) * rng.uniform(150, 400))
    seq = [(req, tq, True)] * int(rng.integers(30, 200))
    seq += [(float(rng.uniform(-1, 1)), float(rng.normal(0, 20)), False)] * int(rng.integers(20, 300))
    for r, t, p in seq:
      out = rec.update(r, last / 409., t, p, 5.0, True, 0.3, float(rng.normal(0, 150)))
      assert abs(out) <= abs(r) + 1e-12
      assert out == 0 or np.sign(out) == np.sign(r)
      last = apply_driver_steer_torque_limits(int(round(out * 409)), last, t, LIM)


def test_holding_pushes_back_at_most_hold_max():
  # held long against with the wheel still: openpilot keeps pushing back, but never above HOLD_MAX of full
  seq = [(-1.0, 120.0, True, 0.0)] * 300
  base, new, out = run(seq)
  assert np.abs(out[-1]) == ktr.HOLD_MAX
  assert np.abs(out).max() <= ktr.HOLD_MAX + 1e-9
  assert np.abs(new).max() <= np.abs(base).max()


def test_wheel_running_away_stops_the_rise():
  # after he lets go the wheel is already turning the request's way faster than RATE_HOLD: envelope does not rise
  _, new, out = run(rotary_release(-200.0))
  off = next(i for i, (r, t, p, _w) in enumerate(rotary_release(-200.0)) if i >= 180 and not p and driver_cut(r, t, LIM) <= ktr.CLAMP_MIN)
  assert np.abs(out[off:]).max() <= np.abs(out[off]) + 1e-9
  assert np.abs(new[off:]).max() <= 409 * ktr.HOLD_MAX + 6


def test_still_wheel_hands_back_within_t_fast():
  seq = rotary_release(0.0)
  base, new, _ = run(seq)
  off = next(i for i, (r, t, p, _w) in enumerate(seq) if i >= 180 and not p and driver_cut(r, t, LIM) <= ktr.CLAMP_MIN)
  assert abs(new[off]) <= 409 * ktr.HOLD_MAX + 6
  end = off + int(ktr.T_FAST * 100) + 100
  assert new[end] == base[end] == -409


def test_other_way_request_is_not_limited():
  # armed against a -1 request, then openpilot asks +0.8 (the driver's way): passed through untouched
  seq = [(-1.0, 310.0, True, 0.0)] * 100 + [(0.8, 0.0, False, 0.0)] * 50
  _, _, out = run(seq)
  assert (out[100:] == 0.8).all()


def test_without_rate_is_the_old_behaviour():
  # rate None gives exactly the 11-06 envelope (rate-less callers unchanged)
  rec_a, rec_b = K9TorqueRecover(LIM), K9TorqueRecover(LIM)
  last = 0
  for req, tq, p, _w in rotary_release(0.0):
    a = rec_a.update(req, last / 409., tq, p, 4.5, True, 0.3)
    b = rec_b.update(req, last / 409., tq, p, 4.5, True, 0.3, None)
    assert a == b
    last = apply_driver_steer_torque_limits(int(round(a * 409)), last, tq, LIM)


def test_controlsd_passes_wheel_rate():
  from pathlib import Path
  src = (Path(__file__).resolve().parents[1] / "controlsd.py").read_text(encoding="utf-8")
  assert "y_std_1s, CS.steeringRateDeg))" in src
