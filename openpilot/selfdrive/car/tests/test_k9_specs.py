"""K9 등록 제원 (recheck_0208, 보노보노 01:54 '축거 3105' / 02:20 2020 K9 RJ 3.8 AWD 제원 / 02:24 '전량 수정').
제원 고정 시험: 숫자만 확인한다(성능·안전 기준 아님)."""
import math
import pytest
from opendbc.car import STD_CARGO_KG, gen_empty_fingerprint
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.car.vehicle_model import VehicleModel


@pytest.fixture(scope="module")
def cp():
  return CarInterface.get_params(CAR.KIA_K9, gen_empty_fingerprint(), [], False, False, False)


def test_k9_registered_specs(cp):
  assert cp.wheelbase == pytest.approx(3.105, abs=1e-6)              # 대감님 제공 / kia.com 축거 3,105 mm
  assert cp.mass == pytest.approx(2015 + STD_CARGO_KG, abs=1e-3)       # 공차 약 2,015 kg (대감님 제공) + opendbc 적재 136 kg
  assert cp.centerToFront == pytest.approx(0.4 * 3.105, abs=1e-6)     # 0.4 비율은 opendbc 현대 공통 가정값 (실측 없음)
  assert cp.steerRatio == pytest.approx(14.5)                          # paramsd 초기값 (그대로)


def test_k9_slip_factor_scales_with_wheelbase_only(cp):
  # opendbc tire-stiffness scaling: slip factor = (kF - kR) / (L kF kR), mass and centerToFront ratio cancel
  vm = VehicleModel(cp)
  sf = (1.0 - 1.0 / (vm.curvature_factor(30.0) * cp.wheelbase)) / 30.0 ** 2
  assert sf == pytest.approx(-7.6067e-4 * 3.15 / 3.105, rel=2e-3)
  # standstill: curvature per road-wheel angle = 1 / L
  assert vm.curvature_factor(0.0) == pytest.approx(1.0 / 3.105, rel=1e-6)
  assert math.isfinite(vm.calc_curvature(math.radians(10.0), 10.0, 0.0))
