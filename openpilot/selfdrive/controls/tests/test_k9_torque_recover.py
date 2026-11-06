"""K9 torque handback after the driver holds against openpilot (k9_torque_recover.py, 17:54 rotary 2026-11-06)."""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from opendbc.car.lateral import apply_driver_steer_torque_limits
from openpilot.cereal import car
from openpilot.selfdrive.controls.lib import k9_torque_recover as ktr
from openpilot.selfdrive.controls.lib.k9_torque_recover import K9TorqueRecover, driver_cut, handback_time, k9_torque_recover_for

LIM = NS(STEER_MAX=409, STEER_DELTA_UP=5, STEER_DELTA_DOWN=10, STEER_DRIVER_ALLOWANCE=50,
         STEER_DRIVER_MULTIPLIER=2, STEER_DRIVER_FACTOR=1)


def run(seq, v=4.5, y_std=0.3, rec=None):
  """seq: (request -1..1, driver torque, pressed). Car side = apply_driver_steer_torque_limits as in the K9 car code.
  Returns applied car torque (base, new) and the new requests."""
  rec = rec or K9TorqueRecover(LIM)
  last_b = last_n = 0
  base, new, req_out = [], [], []
  for req, tq, pressed in seq:
    b = apply_driver_steer_torque_limits(int(round(req * 409)), last_b, tq, LIM)
    r = rec.update(req, last_n / 409., tq, pressed, v, True, y_std)
    n = apply_driver_steer_torque_limits(int(round(r * 409)), last_n, tq, LIM)
    base.append(b)
    new.append(n)
    req_out.append(r)
    last_b, last_n = b, n
  return np.array(base), np.array(new), np.array(req_out)


def test_no_driver_is_unchanged():
  rng = np.random.default_rng(0)
  seq = [(float(r), float(t), False) for r, t in zip(rng.uniform(-1, 1, 3000), rng.normal(0, 15, 3000), strict=True)]
  base, new, _ = run(seq)
  assert (base == new).all()


def test_rate_limit_alone_does_not_arm():
  assert driver_cut(-1.0, 5.0, LIM) == 0
  base, new, _ = run([(-1.0, 5.0, False)] * 200)
  assert (base == new).all()


def test_request_never_larger_never_flips():
  rng = np.random.default_rng(1)
  rec = K9TorqueRecover(LIM)
  last = 0
  for _ in range(40):
    req = float(rng.choice([-1.0, 1.0]))
    tq = float(-np.sign(req) * rng.uniform(150, 400))
    seq = [(req, tq, True)] * int(rng.integers(30, 200))
    seq += [(float(rng.uniform(-1, 1)), float(rng.normal(0, 20)), False)] * int(rng.integers(20, 300))
    for r, t, p in seq:
      out = rec.update(r, last / 409., t, p, 5.0, True, 0.3)
      assert abs(out) <= abs(r) + 1e-12
      assert out == 0 or np.sign(out) == np.sign(r)
      last = apply_driver_steer_torque_limits(int(round(out * 409)), last, t, LIM)


def rotary_release():
  # 17:54:08.9~11.7 shape: request -1, driver holds left 310 -> eases 224 -> 98 -> lets go
  seq = [(-1.0, 310.0, True)] * 180
  seq += [(-1.0, float(t), t > 150) for t in np.linspace(310, 224, 25)]
  seq += [(-1.0, float(t), t > 150) for t in np.linspace(224, -38, 50)]
  seq += [(-1.0, 0.0, False)] * 400
  return seq


def test_rotary_release_is_eased():
  seq = rotary_release()
  base, new, _ = run(seq, y_std=0.3)
  assert (base[:180] == 0).all() and (new[:180] == 0).all()  # held: the driver limit gives 0 either way
  off = next(i for i, (r, t, p) in enumerate(seq) if i >= 180 and not p and driver_cut(r, t, LIM) <= ktr.CLAMP_MIN)
  T = handback_time(0.3)
  assert T == pytest.approx(ktr.T_MIN)
  assert abs(base[off + 50]) >= 300  # base: driver limit opens with his torque, near full within 0.5 s
  assert abs(new[off]) <= 5
  assert abs(new[off + 50]) <= 409 * (0.5 / T) ** 2 + 6
  assert abs(new[off + 100]) <= 409 * (1.0 / T) ** 2 + 6
  end = off + int(T * 100) + 100
  assert new[end] == base[end] == -409  # and the same full torque afterwards


def test_holding_does_not_rise():
  # driver hovering around the limit (base output switches 0 <-> -100 every 0.15 s)
  seq = [(-1.0, 310.0, True)] * 50
  for k in range(20):
    seq += [(-1.0, 250.0 if k % 2 else 200.0, True)] * 15
  base, new, _ = run(seq)
  assert np.abs(np.diff(base)).max() >= 5
  assert np.abs(new[50:]).max() <= 2


def test_driver_turning_the_same_way_hands_back_at_once():
  seq = [(-1.0, 310.0, True)] * 100 + [(-1.0, 0.0, False)] * 20 + [(-1.0, -200.0, True)] * 200
  rec = K9TorqueRecover(LIM)
  base, new, _ = run(seq, rec=rec)
  assert not rec.armed
  assert new[-1] == base[-1] == -409


@pytest.mark.parametrize("v", [15.5, 25.0])
def test_above_speed_unchanged(v):
  base, new, _ = run(rotary_release(), v=v)
  assert (base == new).all()


def test_lat_inactive_resets():
  rec = K9TorqueRecover(LIM)
  run(rotary_release()[:200], rec=rec)
  assert rec.armed
  rec.update(0.0, 0.0, 0.0, False, 4.0, False, 0.3)
  assert not rec.armed and rec.cap == 1.0 and not rec.limiting


def test_limiting_flag_only_when_capped():
  rec = K9TorqueRecover(LIM)
  _, _, req = run(rotary_release(), rec=rec)
  assert (np.abs(req) < 1.0).any()
  assert not rec.limiting  # disarmed at the end


@pytest.mark.parametrize("y,T", [(None, 2.0), (float("nan"), 2.0), (0.1, 2.0), (0.4, 3.0), (0.6, 3.0)])
def test_handback_time(y, T):
  assert handback_time(y) == pytest.approx(T)


def test_only_k9_torque():
  k9 = car.CarParams.new_message(carFingerprint="KIA_K9", flags=262536, steerControlType="torque").as_reader()
  rec = k9_torque_recover_for(k9)
  assert isinstance(rec, K9TorqueRecover) and rec.limits.STEER_MAX == 409
  for fp, ctl in (("HYUNDAI_SONATA", "torque"), ("KIA_K7", "torque"), ("KIA_K9", "angle")):
    assert k9_torque_recover_for(car.CarParams.new_message(carFingerprint=fp, steerControlType=ctl).as_reader()) is None


def test_controlsd_wiring():
  from pathlib import Path
  src = (Path(__file__).resolve().parents[1] / "controlsd.py").read_text(encoding="utf-8")
  assert "self.k9_torque_recover = k9_torque_recover_for(self.CP)" in src
  assert "self.k9_torque_recover.update(actuators.torque, self.sm['carOutput'].actuatorsOutput.torque," in src
  assert "self.k9_torque_recover.limiting" in src
