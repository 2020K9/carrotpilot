"""LateralTorqueCustom 1: menu torque values until torqued is valid, then cross-fade to the learned Filtered values
(보노보노 2026-10-26 17:29, controls/lib/k9_torque_learn.py)."""
import math
from types import SimpleNamespace as NS

import pytest

from openpilot.cereal import car, log
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.vehicle_model import VehicleModel
from openpilot.common.realtime import DT_CTRL
import openpilot.selfdrive.controls.lib.latcontrol_torque as LCT
from openpilot.selfdrive.controls.lib import k9_torque_learn as K
from openpilot.selfdrive.locationd import torqued


class _Params:
  def __init__(self, vals):
    self.v = vals

  def get_int(self, k, *a, **kw):
    return int(float(self.v.get(k, 0)))

  def get_float(self, k, *a, **kw):
    return float(self.v.get(k, 0.0))

  def get_bool(self, k, *a, **kw):
    return bool(int(float(self.v.get(k, 0))))


MENU = {"LateralTorqueCustom": 1, "LateralTorqueAccelFactor": 2500, "LateralTorqueFriction": 131, "LateralTorqueKpV": 140,
        "LateralTorqueKiV": 20, "LateralTorqueKf": 85, "LateralTorqueKd": 0, "SteerActuatorDelay": 20}


def _k9_cp():
  from opendbc.car.hyundai.interface import CarInterface
  return CarInterface.get_non_essential_params("KIA_K9")


def _lac(monkeypatch, vals=None):
  p = _Params(dict(MENU if vals is None else vals))
  monkeypatch.setattr(LCT, "Params", lambda: p)
  CP = _k9_cp()
  tfl = lambda *aa, **kk: CarInterfaceBase.torque_from_lateral_accel_linear(None, *aa, **kk)
  ci = NS(use_nnff=False, use_nnff_lite=False, torque_from_lateral_accel=lambda: tfl)
  return LCT.LatControlTorque(CP.as_reader(), ci), VehicleModel(CP), p


def _step(lac, vm, n=1, active=False, curv=0.0, v=25.0):
  cs = car.CarState.new_message()
  cs.vEgo = v
  lp = log.LiveParametersData.new_message()
  lp.roll = 0.0
  cc = car.CarControl.new_message()
  out = None
  for _ in range(n):
    out = lac.update(active, cs, vm, lp, False, curv, cc, False)
  return out


def _tp(lac):
  return float(lac.torque_params.latAccelFactor), float(lac.torque_params.friction)


def test_constants_match_torqued():
  assert K.TORQUED_MIN_DECAY == torqued.MIN_FILTER_DECAY and K.TORQUED_MAX_DECAY == torqued.MAX_FILTER_DECAY


def test_switch_unit_rules():
  sw = K.TorqueLearnSwitch()
  assert sw.apply(2.5, 0.131) == (2.5, 0.131)
  sw.update_learned(False, 2.71, 0.102, 250.0)             # learning (< 100 %): menu
  for _ in range(5000):
    sw.step(0.01)
  assert sw.w == 0.0 and sw.apply(2.5, 0.131) == (2.5, 0.131)
  sw.update_learned(True, 2.30, 0.10, 50.0)               # just valid: Filtered = CarParams seed, decay 50 -> still menu
  for _ in range(100):
    sw.step(0.01)
  assert sw.w == 0.0
  sw.update_learned(True, 2.62, 0.103, 125.0)             # 1c8 seg 42: decay 113..125 -> w target 0.375
  for _ in range(1000):                                    # 10 s: rate limit 1/30 per s -> 0.333
    sw.step(0.01)
    sw.update_learned(True, 2.62, 0.103, 125.0)
  assert sw.w == pytest.approx(1.0 / 3.0)
  for _ in range(500):
    sw.step(0.01)
    sw.update_learned(True, 2.62, 0.103, 125.0)
  assert sw.w == pytest.approx(0.375)
  laf, fr = sw.apply(2.5, 0.131)
  assert laf == pytest.approx(0.625 * 2.5 + 0.375 * 2.62) and fr == pytest.approx(0.625 * 0.131 + 0.375 * 0.103)
  for bad in ((True, float("nan"), 0.1, 250.0), (True, 0.0, 0.1, 250.0), (True, 2.7, -0.1, 250.0), (True, 2.7, 0.1, None)):
    s2 = K.TorqueLearnSwitch()
    s2.update_learned(*bad)
    s2.step(1.0)
    assert s2.w == 0.0, bad


def test_menu_values_bit_identical_until_valid(monkeypatch):
  new, vm, _ = _lac(monkeypatch)
  old, vm2, _ = _lac(monkeypatch)
  old.update_live_torque_params = lambda *a, **kw: None     # = before: Custom 1 ignores liveTorqueParameters
  for i in range(3000):
    for lac in (new, old):
      lac.update_live_torque_params(2.30, 0.0, 0.10, live_valid=False, decay=50.0)
    a = _step(new, vm, active=True, curv=1e-3 * math.sin(i * 0.01))
    b = _step(old, vm2, active=True, curv=1e-3 * math.sin(i * 0.01))
    assert a[0] == b[0] and _tp(new) == _tp(old)
  assert _tp(new) == pytest.approx((2.5, 0.131))
  assert new.k9_torque_learn.w == 0.0


def test_fade_in_to_learned_no_step_and_back(monkeypatch):
  lac, vm, _ = _lac(monkeypatch)
  _step(lac, vm, 20)
  assert _tp(lac) == pytest.approx((2.5, 0.131))
  hist = []
  for _ in range(int(40 / DT_CTRL)):                       # 1c9: restored valid cache, decay 250, Filtered 2.708 / 0.102
    lac.update_live_torque_params(2.708, -0.036, 0.102, live_valid=True, decay=250.0)
    _step(lac, vm)
    hist.append(_tp(lac))
  laf = [h[0] for h in hist]
  assert laf[-1] == pytest.approx(2.708, abs=1e-6) and hist[-1][1] == pytest.approx(0.102, abs=1e-6)
  steps = [abs(b - a) for a, b in zip(laf, laf[1:])]
  assert max(steps) <= (2.708 - 2.5) * K.RATE_PER_S * DT_CTRL * 1.01 + 1e-6   # no step: <= gap / 30 s per frame
  t_full = next(i for i, x in enumerate(laf) if abs(x - 2.708) < 1e-6) * DT_CTRL
  assert 29.0 <= t_full <= 31.0
  assert float(lac.torque_params.latAccelOffset) == pytest.approx(0.0)      # offset stays CarParams default
  # liveValid drops (torqued reset) -> back to the menu values over 30 s, ends exactly there
  hist = []
  for _ in range(int(35 / DT_CTRL)):
    lac.update_live_torque_params(2.30, 0.0, 0.10, live_valid=False, decay=50.0)
    _step(lac, vm)
    hist.append(_tp(lac))
  laf = [h[0] for h in hist]
  assert max(abs(b - a) for a, b in zip(laf, laf[1:])) <= (2.708 - 2.5) * K.RATE_PER_S * DT_CTRL * 1.01 + 1e-6
  assert hist[-1] == pytest.approx((2.5, 0.131), abs=1e-7) and lac.k9_torque_learn.w == 0.0


def test_stale_message_goes_back_to_menu(monkeypatch):
  lac, vm, _ = _lac(monkeypatch)
  for _ in range(int(40 / DT_CTRL)):
    lac.update_live_torque_params(2.708, 0.0, 0.102, live_valid=True, decay=250.0)
    _step(lac, vm)
  assert lac.k9_torque_learn.w == 1.0
  _step(lac, vm, int(0.9 / DT_CTRL))                       # controlsd stops calling (checks fail): held for STALE_S
  assert lac.k9_torque_learn.w == 1.0
  _step(lac, vm, int(32 / DT_CTRL))
  assert lac.k9_torque_learn.w == 0.0 and _tp(lac) == pytest.approx((2.5, 0.131), abs=1e-7)


def test_custom_off_is_upstream(monkeypatch):
  vals = dict(MENU, LateralTorqueCustom=0)
  lac, vm, _ = _lac(monkeypatch, vals)
  lac.update_live_torque_params(2.708, -0.036, 0.102, live_valid=False, decay=50.0)
  _step(lac, vm, 50)
  assert _tp(lac) == pytest.approx((2.708, 0.102)) and float(lac.torque_params.latAccelOffset) == pytest.approx(-0.036)
  assert lac.k9_torque_learn.w == 0.0
