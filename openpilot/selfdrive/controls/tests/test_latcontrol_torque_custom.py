"""LateralTorqueCustom (0/1, carrot_settings.json) switching at runtime in LatControlTorque.update.

Runs without the compiled params_pyx: a dict-backed Params stand-in is installed only if the real module is missing.
"""
import functools
import sys
import types

import pytest

_PARAMS = {}


class _FakeParams:
  def __init__(self, *a, **k):
    pass

  def get_int(self, key, *a, **k):
    return int(float(_PARAMS.get(key, 0)))

  def get_float(self, key, *a, **k):
    return float(_PARAMS.get(key, 0))

  def get_bool(self, key, *a, **k):
    return bool(int(float(_PARAMS.get(key, 0))))

  def get(self, key, *a, **k):
    return _PARAMS.get(key)

  def __getattr__(self, name):  # put*, remove, ... : no-ops for the car interface constructors
    if name.startswith(('put', 'remove')):
      return lambda *a, **k: None
    raise AttributeError(name)


try:
  import openpilot.common.params_pyx  # noqa: F401
except Exception:
  m = types.ModuleType('openpilot.common.params_pyx')
  m.Params = _FakeParams; m.ParamKeyFlag = object; m.ParamKeyType = object; m.UnknownKeyName = KeyError
  sys.modules['openpilot.common.params_pyx'] = m

from openpilot.cereal import car, log  # noqa: E402
from opendbc.car.car_helpers import interfaces  # noqa: E402
from opendbc.car.interfaces import CarInterfaceBase  # noqa: E402
from opendbc.car.hyundai.values import CAR as HYUNDAI  # noqa: E402
from opendbc.car.vehicle_model import VehicleModel  # noqa: E402
import openpilot.selfdrive.controls.lib.latcontrol_torque as LT  # noqa: E402

# the drive of 2026-10-08 (route 00000160, initData)
CUSTOM = {'LateralTorqueAccelFactor': 2450, 'LateralTorqueFriction': 131, 'LateralTorqueKpV': 140, 'LateralTorqueKiV': 20,
          'LateralTorqueKf': 85, 'LateralTorqueKd': 0}


@pytest.fixture(autouse=True)
def fake_params(monkeypatch):
  monkeypatch.setattr(LT, 'Params', _FakeParams)
  _PARAMS.clear(); _PARAMS.update(CUSTOM)
  yield


def make(custom):
  _PARAMS['LateralTorqueCustom'] = custom
  CI = interfaces[HYUNDAI.KIA_K9]
  CP = CI.get_non_essential_params(HYUNDAI.KIA_K9)
  # LatControlTorque only uses these CI members; the real Hyundai constructor needs CAN fingerprint params
  ci = types.SimpleNamespace(use_nnff=False, use_nnff_lite=False,
                             torque_from_lateral_accel=lambda: functools.partial(CarInterfaceBase.torque_from_lateral_accel_linear, None))
  ctl = LT.LatControlTorque(CP.as_reader(), ci)
  return ctl, VehicleModel(CP), CP


def run(ctl, VM, frames):
  CS = car.CarState.new_message(); CS.vEgo = 15.0
  params = log.LiveParametersData.new_message()
  CC = car.CarControl.new_message()
  for _ in range(frames):
    ctl.update(True, CS, VM, params, False, 0.001, CC, False)


def gains(ctl):
  return ctl.pid.k_p, ctl.pid.k_i, ctl.pid.k_f, ctl.pid.k_d


def test_custom_on_applies_custom_values():
  ctl, VM, CP = make(1)
  run(ctl, VM, 10)
  assert gains(ctl) == pytest.approx((1.40, 0.20, 0.85, 0.0))
  assert ctl.torque_params.latAccelFactor == pytest.approx(2.45)
  assert ctl.torque_params.friction == pytest.approx(0.131)


def test_custom_1_to_0_restores_carparams_defaults():
  ctl, VM, CP = make(1)
  run(ctl, VM, 10)
  _PARAMS['LateralTorqueCustom'] = 0
  run(ctl, VM, 10)
  t = CP.lateralTuning.torque
  assert gains(ctl) == pytest.approx((t.kp, t.ki, t.kf, 0.0))
  assert ctl.torque_params.latAccelFactor == pytest.approx(t.latAccelFactor)
  assert ctl.torque_params.friction == pytest.approx(t.friction)
  assert ctl.torque_params.latAccelOffset == pytest.approx(t.latAccelOffset)


def test_custom_0_to_1_to_0_to_1_round_trip():
  ctl, VM, CP = make(0)
  run(ctl, VM, 10)
  t = CP.lateralTuning.torque
  assert gains(ctl) == pytest.approx((t.kp, t.ki, t.kf, 0.0))
  for custom, want in ((1, (1.40, 0.20, 0.85, 0.0)), (0, (t.kp, t.ki, t.kf, 0.0)), (1, (1.40, 0.20, 0.85, 0.0))):
    _PARAMS['LateralTorqueCustom'] = custom
    run(ctl, VM, 10)
    assert gains(ctl) == pytest.approx(want), custom


def test_live_torque_params_used_after_custom_off():
  ctl, VM, CP = make(1)
  run(ctl, VM, 10)
  ctl.update_live_torque_params(3.0, 0.0, 0.2)
  assert ctl.torque_params.latAccelFactor == pytest.approx(2.45)  # custom on: live values ignored
  _PARAMS['LateralTorqueCustom'] = 0
  run(ctl, VM, 10)
  ctl.update_live_torque_params(3.0, 0.0, 0.2)
  assert ctl.torque_params.latAccelFactor == pytest.approx(3.0)
  assert ctl.torque_params.friction == pytest.approx(0.2)
