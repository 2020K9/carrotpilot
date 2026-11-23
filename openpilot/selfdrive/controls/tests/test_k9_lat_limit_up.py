"""K9LatLimitUp (실험중 맨 위, 대감님 11/23 17:47): 0 = clip_curvature limits unchanged, 1 = carrot-dev limits."""
import json
import pathlib

import numpy as np
import pytest

from opendbc.car.vehicle_model import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.realtime import DT_CTRL
import openpilot.selfdrive.controls.lib.drive_helpers as DH

ROOT = pathlib.Path(__file__).resolve().parents[4]


def _ref(v, prev, new, roll, jerk, lo, hi, maxc):
  v = max(v, DH.MIN_SPEED)
  r = jerk / v ** 2
  new = float(np.clip(new, prev - r * DT_CTRL, prev + r * DT_CTRL))
  a = float(np.interp(v, [80 / 3.6, 120 / 3.6], [lo, hi]))
  rc = roll * ACCELERATION_DUE_TO_GRAVITY
  lim = False
  lo_c, hi_c = (-a + rc) / v ** 2, (a + rc) / v ** 2
  if new < lo_c or new > hi_c:
    lim = True
  new = min(max(new, lo_c), hi_c)
  if abs(new) > maxc:
    lim = True
  new = min(max(new, -maxc), maxc)
  return new, lim


CASES = [(v, prev, new, roll) for v in (0.0, 0.5, 3.0, 8.0, 15.0, 22.2, 27.0, 33.3, 40.0)
         for prev in (-0.3, -0.05, 0.0, 0.01, 0.25) for new in (-0.5, -0.1, 0.0, 0.003, 0.15, 0.45) for roll in (-0.05, 0.0, 0.03)]


def test_default_off_is_old_limits_exactly():
  assert (DH.MAX_LATERAL_JERK, DH.MAX_LATERAL_ACCEL_NO_ROLL_LOW_SPEED, DH.MAX_LATERAL_ACCEL_NO_ROLL, DH.MAX_CURVATURE) == (5.0, 4.5, 3.0, 0.2)
  for c in CASES:
    a = DH.clip_curvature(*c)
    assert a == DH.clip_curvature(*c, lat_limit_up=False)
    r = _ref(*c, 5.0, 4.5, 3.0, 0.2)
    assert a[0] == pytest.approx(r[0], abs=1e-12) and a[1] == r[1], c


def test_on_uses_carrot_dev_limits():
  assert (DH.K9_UP_MAX_LATERAL_JERK, DH.K9_UP_MAX_LATERAL_ACCEL_NO_ROLL_LOW_SPEED, DH.K9_UP_MAX_LATERAL_ACCEL_NO_ROLL,
          DH.K9_UP_MAX_CURVATURE) == (7.0, 6.5, 4.5, 0.4)
  for c in CASES:
    a = DH.clip_curvature(*c, lat_limit_up=True)
    r = _ref(*c, 7.0, 6.5, 4.5, 0.4)
    assert a[0] == pytest.approx(r[0], abs=1e-12) and a[1] == r[1], c


def test_on_never_tighter_than_off():
  for c in CASES:
    v, prev, new, roll = c
    if not DH.lat_limit_fits_normal(v, prev, roll):
      continue   # prev outside the old limits: not a like-for-like start
    off, _ = DH.clip_curvature(*c)
    on, _ = DH.clip_curvature(*c, lat_limit_up=True)
    assert abs(on - new) <= abs(off - new) + 1e-12, c   # on is at least as close to the request


def test_accel_and_jerk_numbers():
  v = 20.0  # 72 km/h -> low-speed accel
  assert DH.clip_curvature(v, 0.0, 1.0, 0.0)[0] == pytest.approx(5.0 * DT_CTRL / v ** 2)
  assert DH.clip_curvature(v, 0.0, 1.0, 0.0, lat_limit_up=True)[0] == pytest.approx(7.0 * DT_CTRL / v ** 2)
  assert DH.clip_curvature(v, 0.2, 1.0, 0.0)[0] == pytest.approx(4.5 / v ** 2)
  assert DH.clip_curvature(v, 0.2, 1.0, 0.0, lat_limit_up=True)[0] == pytest.approx(6.5 / v ** 2)
  v = 40.0  # 144 km/h -> high-speed accel
  assert DH.clip_curvature(v, 0.01, 1.0, 0.0)[0] == pytest.approx(3.0 / v ** 2)
  assert DH.clip_curvature(v, 0.01, 1.0, 0.0, lat_limit_up=True)[0] == pytest.approx(4.5 / v ** 2)
  v = 2.0
  assert DH.clip_curvature(v, 0.39, 1.0, 0.0)[0] == pytest.approx(0.2)
  assert DH.clip_curvature(v, 0.39, 1.0, 0.0, lat_limit_up=True)[0] == pytest.approx(0.4)


class _P:
  def __init__(self, v):
    self.v = v

  def get_int(self, k):
    assert k == "K9LatLimitUp"
    if isinstance(self.v, Exception):
      raise self.v
    return self.v


@pytest.mark.parametrize("v,exp", [(0, False), (1, True), (2, False), (-1, False), (None, False), (RuntimeError(), False)])
def test_read_param(v, exp):
  assert DH.read_lat_limit_up(_P(v)) is exp


def test_turn_off_waits_until_fits_normal():
  v = 20.0
  assert not DH.lat_limit_fits_normal(v, 6.0 / v ** 2, 0.0)
  assert DH.lat_limit_fits_normal(v, 4.0 / v ** 2, 0.0)
  assert not DH.lat_limit_fits_normal(2.0, 0.3, 0.0)
  assert DH.lat_limit_fits_normal(2.0, 0.19, 0.0)


class _Live:
  def __init__(self):
    self.v = 0
    self.reads = 0

  def get_int(self, k):
    self.reads += 1
    return self.v


def _toggle_sim(req_seq, v, curv_seq):
  p = _Live()
  s = DH.LatLimitUp(p)
  out = []
  for frame, (r, k) in enumerate(zip(req_seq, curv_seq)):
    p.v = 1 if r else 0
    out.append(s.step(frame, v, k, 0.0))
  assert p.reads == 1 + (len(req_seq) + 99) // 100   # __init__ + 1 Hz only
  return out


def test_toggle_live_within_1s_and_soft_off():
  v = 20.0
  n = 400
  req = [False] * 50 + [True] * 150 + [False] * 200
  out = _toggle_sim(req, v, [0.0] * n)
  assert out.index(True) == 100 and out.index(True) - 50 <= 100   # on within 1 s of the change
  assert out[200] is False and out[199] is True                     # off at the next 1 Hz read when curvature fits
  big = [6.0 / v ** 2] * 350 + [0.0] * 50
  out = _toggle_sim(req, v, big)
  assert out[349] is True and out[350] is False                     # off waits until the command fits the normal limits


def test_controlsd_wiring():
  src = (ROOT / "openpilot/selfdrive/controls/controlsd.py").read_text(encoding="utf-8")
  assert "lat_limit_up=lat_limit_up)" in src and "self.k9_lat_limit = LatLimitUp(self.params)" in src
  assert "k9_lat_limit.step(self.sm.frame, CS.vEgo, self.desired_curvature, lp.roll)" in src
  assert "K9LatLimitUp" not in src.replace("# K9LatLimitUp", "")   # no direct param read in controlsd


def test_menu_top_of_experimental_0_1():
  d = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  exp = [g for t in d["menu"] for g in t.get("groups", []) if g.get("id") == "EXPERIMENTAL"]
  assert len(exp) == 1 and exp[0]["params"][0] == "K9LatLimitUp"
  p = next(x for x in d["params"] if x["name"] == "K9LatLimitUp")
  assert (p["min"], p["max"], p["default"], p["control"]) == (0, 1, 0, "select")
  assert [len(p["options"][k]) for k in ("ko", "en", "zh")] == [2, 2, 2]
  for k in ("title", "etitle", "ctitle", "descr", "edescr", "cdescr"):
    assert p[k]
  assert "급커브 더 꺾음" in p["descr"]
  keys = (ROOT / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
  assert '{"K9LatLimitUp", {PERSISTENT, INT, "0"}}' in keys
