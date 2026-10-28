"""K9CenterToFront (실험중 menu, 2026-10-28): centerToFront = wheelbase * value/100, 40 = the original `* 0.4`.

Default / unset / out of range must give exactly the old value (bit for bit); 60 gives wheelbase * 0.6.
"""
import numpy as np
import pytest

from opendbc.car.hyundai import interface
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR


class _P:
  def __init__(self, vals):
    self.vals = vals

  def get_int(self, key, *a, **k):
    return int(self.vals.get(key, 0))

  def get_bool(self, key, *a, **k):
    return bool(self.vals.get(key, 0))

  def get(self, key, *a, **k):
    return None

  def put_int(self, *a, **k):
    pass

  def put_bool(self, *a, **k):
    pass

  def put(self, *a, **k):
    pass


def _f32(x):
  return float(np.float32(x))  # CarParams fields are Float32


def _cp(monkeypatch, car, vals):
  monkeypatch.setattr(interface, "Params", lambda: _P(vals))
  fingerprint = {i: {} for i in range(8)}
  return CarInterface.get_params(car, fingerprint, [], False, False, False)


@pytest.mark.parametrize("car", [CAR.KIA_K9, CAR.HYUNDAI_SONATA, CAR.KIA_EV6])
@pytest.mark.parametrize("vals", [{}, {"K9CenterToFront": 40}, {"K9CenterToFront": 0}, {"K9CenterToFront": 29},
                                  {"K9CenterToFront": 81}, {"K9CenterToFront": -40}, {"K9CenterToFront": 400}])
def test_default_and_out_of_range_are_original(monkeypatch, car, vals):
  cp = _cp(monkeypatch, car, vals)
  assert cp.centerToFront == _f32(cp.wheelbase * 0.4)  # exact, same expression as before


@pytest.mark.parametrize("v", [30, 41, 50, 60, 80])
def test_value_scales_wheelbase(monkeypatch, v):
  cp = _cp(monkeypatch, CAR.KIA_K9, {"K9CenterToFront": v})
  assert cp.centerToFront == _f32(cp.wheelbase * (v / 100.0))


def test_k9_numbers(monkeypatch):
  base = _cp(monkeypatch, CAR.KIA_K9, {})
  six = _cp(monkeypatch, CAR.KIA_K9, {"K9CenterToFront": 60})
  assert base.wheelbase == pytest.approx(3.105)
  assert base.centerToFront == pytest.approx(1.242)
  assert six.centerToFront == pytest.approx(1.863)
