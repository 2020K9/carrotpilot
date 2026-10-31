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


def test_custom_on_menu_values_even_when_learned_100(monkeypatch):
  # 보노보노 2026-10-31 16:48 "돌려놔": Custom on = web values fixed, never replaced by the learned values
  lac, vm, _ = _lac(monkeypatch)
  lp = log.LiveTorqueParametersData.new_message()
  lp.useParams, lp.liveValid, lp.calPerc, lp.decay = True, True, 100, float(torqued.MAX_FILTER_DECAY)
  lp.latAccelFactorFiltered, lp.latAccelOffsetFiltered, lp.frictionCoefficientFiltered = 2.708, -0.036, 0.102
  for _ in range(int(60 / DT_CTRL)):
    lac.update_live_torque_params(lp.latAccelFactorFiltered, lp.latAccelOffsetFiltered, lp.frictionCoefficientFiltered)
    _step(lac, vm)
    assert _tp(lac) == pytest.approx((2.5, 0.131)) or lac.frame < 10
  assert _tp(lac) == pytest.approx((2.5, 0.131)) and float(lac.torque_params.latAccelOffset) == pytest.approx(0.0)


def test_custom_off_is_upstream(monkeypatch):
  vals = dict(MENU, LateralTorqueCustom=0)
  lac, vm, _ = _lac(monkeypatch, vals)
  lac.update_live_torque_params(2.708, -0.036, 0.102)
  _step(lac, vm, 50)
  assert _tp(lac) == pytest.approx((2.708, 0.102)) and float(lac.torque_params.latAccelOffset) == pytest.approx(-0.036)


def test_controlsd_feeds_only_filtered():
  import inspect
  from openpilot.selfdrive.controls import controlsd
  src = inspect.getsource(controlsd)
  assert "live_valid=" not in src and "decay=torque_params" not in src
