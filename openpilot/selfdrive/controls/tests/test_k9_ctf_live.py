"""K9CenterToFront live update (selfdrive/controls/lib/k9_ctf.py): untouched setting = no change at all,
a change reaches controlsd's VehicleModel and the lateral planner factors exactly as a car started with that value."""
from types import SimpleNamespace

import pytest

from opendbc.car import scale_tire_stiffness
from opendbc.car.vehicle_model import VehicleModel
from openpilot.selfdrive.controls.lib.k9_ctf import LiveCenterToFront, ctf_ratio

WB, MASS, TSF = 3.105, 2100.0, 0.65


def _cp(ratio=0.4, brand="hyundai"):
  ctf = WB * ratio
  tf, tr = scale_tire_stiffness(MASS, WB, ctf, TSF)
  return SimpleNamespace(brand=brand, wheelbase=WB, centerToFront=ctf, mass=MASS, rotationalInertia=4000.0,
                         steerRatioRear=0.0, tireStiffnessFront=tf, tireStiffnessRear=tr, tireStiffnessFactor=TSF,
                         steerRatio=15.0)


@pytest.mark.parametrize("raw,ratio", [(40, 0.4), (None, 0.4), ("x", 0.4), (29, 0.4), (81, 0.4), (0, 0.4), (-60, 0.4),
                                       (30, 0.30), (41, 0.41), (60, 0.60), (80, 0.80)])
def test_ratio(raw, ratio):
  assert ctf_ratio(raw) == ratio


@pytest.mark.parametrize("start", [40, 0, 60])
def test_unchanged_value_never_touches_anything(start):
  live = LiveCenterToFront(_cp(), start)
  for _ in range(5):
    assert live.poll(start) is None
  assert live.poll(None) is None  # unreadable -> keep


def test_other_brands_ignored():
  live = LiveCenterToFront(_cp(brand="toyota"), 40)
  assert not live.enabled and live.poll(60) is None


@pytest.mark.parametrize("v", [30, 41, 60, 80])
def test_change_matches_a_fresh_start(v):
  CP = _cp()
  VM = VehicleModel(CP)
  live = LiveCenterToFront(CP, 40)
  ctf = live.poll(v)
  assert ctf == WB * (v / 100.0)
  live.apply_vehicle_model(VM, ctf)
  VM.update_params(1.0, 15.0)
  ref = VehicleModel(_cp(v / 100.0))
  for k in ("aF", "aR", "cF", "cR", "cF_orig", "cR_orig"):
    assert getattr(VM, k) == pytest.approx(getattr(ref, k), rel=1e-12), k
  for u in (5.0, 15.0, 30.0):
    assert VM.calc_curvature(0.1, u, 0.0) == pytest.approx(ref.calc_curvature(0.1, u, 0.0), rel=1e-9)
  f1, f2 = live.planner_factors(ctf)
  CPv = _cp(v / 100.0)
  assert f1 == pytest.approx(CPv.wheelbase - CPv.centerToFront, rel=1e-12)
  assert f2 == pytest.approx((CPv.centerToFront * CPv.mass) / (CPv.wheelbase * CPv.tireStiffnessRear), rel=1e-12)


def test_back_to_default_restores_0_4():
  CP = _cp()
  live = LiveCenterToFront(CP, 40)
  assert live.poll(60) == WB * 0.6
  assert live.poll(60) is None
  assert live.poll(40) == WB * 0.4
  assert live.poll(99) == WB * 0.4  # out of range -> 0.4


def test_controlsd_and_planner_hooks_present():
  import inspect
  from openpilot.selfdrive.controls import controlsd
  from openpilot.selfdrive.controls.lib import lateral_planner
  src_c, src_p = inspect.getsource(controlsd), inspect.getsource(lateral_planner)
  assert "LiveCenterToFront.from_params(self.CP, self.params, every=100)" in src_c
  assert "k9_ctf.apply_vehicle_model(self.VM, ctf)" in src_c
  assert src_c.index("k9_ctf.apply_vehicle_model(self.VM, ctf)") < src_c.index("self.VM.update_params(x, sr)")
  assert "self.factor1, self.factor2 = k9_ctf.planner_factors(ctf)" in src_p


def test_due_every_n_calls():
  live = LiveCenterToFront(_cp(), 40, every=100)
  hits = [live.due() for _ in range(300)]
  assert sum(hits) == 3 and hits[99] and hits[199] and hits[299]
  assert not any(LiveCenterToFront(_cp(brand="toyota"), 40, every=1).due() for _ in range(5))


class _P:
  def __init__(self, v):
    self.v = v

  def get_int(self, k):
    assert k == "K9CenterToFront"
    if isinstance(self.v, Exception):
      raise self.v
    return self.v


def test_check_reads_params():
  live = LiveCenterToFront.from_params(_cp(), _P(40))
  assert live.check(_P(40)) is None
  assert live.check(_P(RuntimeError("no param"))) is None
  assert live.check(_P(55)) == WB * 0.55
