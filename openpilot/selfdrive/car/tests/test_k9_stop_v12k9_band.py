"""brake_v12k9_live_fix (2026-10-25 evening, 보노보노 17:46 "브레이크 좀 어떻게 해줘라.." / 18:02 absolute criteria): K9StopMode 2
(v12k9) ComfortBand. On the car the feathered band-0 stops of menus 1 and 2 shuddered at 6-7 Hz below 7 km/h (17:32:41, 17:39:05,
17:43:10), the band-on stops of menu 0 did not; kind 8 now sends the original ComfortBand (0.9 / 0.8 + 0.2 * accel) at or below
2.5 m/s, linear to 0 at 3.0 m/s, 0 at or above 3.0 m/s (as before); once StopReq is latched it ramps back to 0 by 0.04 per
make_jerk call (the latch phase is the old menu 2 again). Accel, StopReq and the jerk limits are unchanged.
Menus 0 / 1 / other values and other cars: CAN identical to 6f518e35 (the brake code of the car's 5ab624e8).
Model-only harness with synthetic inputs: source logic and SCC12/SCC14 encoding, not K9 vehicle behaviour (실차 미검증).
"""
import math
import random

import numpy as np
import pytest

from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V12K9
from opendbc.car.hyundai.values import CAR
from openpilot.selfdrive.car.tests.k9_rig import LCS, Rig, edge_sequence, gentle_sequence, load_frozen_cc, make_cp, mixed_sequence
from openpilot.selfdrive.car.tests.test_k9_stop_v12 import _random_inputs

FROZEN = "6f518e35"  # carrot-wip brake code before this fix (the car runs 5ab624e8, brake code identical)
SEQS = (mixed_sequence, edge_sequence, gentle_sequence)


@pytest.fixture(scope="module")
def frozen_cc(tmp_path_factory):
  return load_frozen_cc(FROZEN, tmp_path_factory.mktemp("band_6f518e35"), "band_6f518e35")


def test_band_constants():
  assert (kc.V12K9_BAND_FULL_SPEED, kc.V12K9_BAND_ZERO_SPEED, kc.V12K9_BAND_LATCH_STEP) == (2.5, 3.0, 0.04)


def test_latched_band_ramps_to_zero():
  c = K9Creep(V12K9)
  c.latched = True
  prev_u, prev_l, seen = 0.84, 0.74, []
  for _ in range(40):
    _, cu, cl = c.bands(0.5, 0.3, 0.84, 0.74, prev_u, prev_l)
    assert cu == pytest.approx(max(0.0, prev_u - 0.04)) and cl == pytest.approx(max(0.0, prev_l - 0.04))
    prev_u, prev_l = cu, cl
    seen.append(cu)
  assert seen[-1] == 0.0 and seen.index(0.0) == 20  # 0.84 / 0.04 = 21 calls
  # never above the original band of the frame, no lower previous value given: previous upper - 0.1
  assert c.bands(0.5, 0.3, 0.50, 0.40, 0.84, None)[1:] == pytest.approx((0.50, 0.40))
  assert c.bands(0.5, 0.3, 0.84, 0.74, 0.30, None)[1:] == pytest.approx((0.26, 0.16))
  c.latched = False
  assert c.bands(0.5, 0.3, 0.84, 0.74, 0.0, 0.0)[1:] == (0.84, 0.74)  # unlatched: the original band at once


def test_through_carcontroller_band_on_then_ramp_at_standstill_latch(monkeypatch):
  r = Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), K9StopMode=2)
  for i in range(120):
    r.step(-0.5, LCS.pid, max(2.0 - 0.012 * i, 0.4), a_target=-0.4, a_ego=-0.4)
  for i in range(60):
    r.step(-0.5, LCS.stopping, max(0.4 - 0.01 * i, 0.06), a_target=-0.3, a_ego=-0.3)
  n0 = len(r.scc14)
  assert all(s["ComfortBandUpper"] > 0.6 for _, v, s in r.scc14[20:n0])  # band on while feathering, not latched
  rows = []
  for _ in range(80):
    _, sent = r.step(-0.5, LCS.stopping, 0.0, standstill=True, a_target=-0.3, a_ego=0.0)
    rows.append(None if sent is None else sent["StopReq"])
  bands = [s["ComfortBandUpper"] for _, _, s in r.scc14[n0:]]
  assert max(x for x in rows if x is not None) == 1 and bands[-1] == 0.0
  assert all(b - a <= 1e-9 for a, b in zip(bands, bands[1:], strict=False))  # only falls
  assert all(a - b <= 0.04 + 0.011 for a, b in zip(bands, bands[1:], strict=False))  # by the ramp step (0.02 CAN grid)


def test_band_object_grid(frozen_cc):
  old, new = frozen_cc.K9Creep(V12K9), K9Creep(V12K9)
  for v in np.linspace(0.0, 6.0, 241):
    for cu in (0.0, 0.3, 0.744, 0.9, 1.2):
      cl = max(cu - 0.1, 0.0)
      ro, rn = old.bands(0.5, float(v), cu, cl, 0.0, 0.0), new.bands(0.5, float(v), cu, cl, 0.0, 0.0)
      assert rn[0] == ro[0] and ro[1:] == (0.0, 0.0)  # jerk unchanged; the old band was 0
      w = float(np.interp(v, [2.5, 3.0], [1.0, 0.0]))
      assert rn[1] == pytest.approx(cu * w, abs=1e-12) and rn[2] == pytest.approx(cl * w, abs=1e-12)
      if v >= 3.0:
        assert rn == ro  # bit for bit the old kind 8
      if v <= 2.5:
        assert (rn[1], rn[2]) == (cu, cl)  # exactly the original band
      assert new.jerk_l_floor(1.0, float(v)) == old.jerk_l_floor(1.0, float(v))
  assert new.bands(0.5, math.nan, 0.9, 0.8, 0.0, 0.0)[1:] == (0.0, 0.0)


@pytest.mark.parametrize("seed", range(6))
def test_update_identical_to_frozen(frozen_cc, seed):
  """accel / StopReq / latch state are bit for bit the frozen kind 8 for the same inputs."""
  old, new = frozen_cc.K9Creep(V12K9), K9Creep(V12K9)
  rng = random.Random(f"k9-v12k9-band-{seed}")
  for kw, i in _random_inputs(rng, 3000):
    ro, rn = old.update(**kw), new.update(**kw)
    assert ro[1] == rn[1], (i, kw)
    assert (math.isnan(ro[0]) and math.isnan(rn[0])) or ro[0] == rn[0], (i, kw, ro, rn)
    assert (old.latched, old.active, old.ss_count) == (new.latched, new.active, new.ss_count), i
    assert old.last == new.last or (math.isnan(old.last) and math.isnan(new.last)), i


def _kw(mode):
  return {} if mode is None else {"K9StopMode": mode}


@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0, 1, 3, 99, -1])
def test_menus_other_than_2_can_identical_to_frozen(monkeypatch, frozen_cc, mode, fingerprint):
  for seq in SEQS:
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), module=frozen_cc, **_kw(mode)))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), **_kw(mode)))
    monkeypatch.undo()
    assert len(new.can) > 400 and ref.can == new.can and ref.scc14 == new.scc14


def test_menu2_other_cars_can_identical_to_frozen(monkeypatch, frozen_cc):
  for seq in SEQS:
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), module=frozen_cc, K9StopMode=2))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), K9StopMode=2))
    monkeypatch.undo()
    assert ref.can == new.can


def test_menu2_only_the_band_changes(monkeypatch, frozen_cc):
  """K9 menu 2: every message other than SCC14 is byte-identical to the frozen menu 2; in SCC14 only ComfortBandUpper / Lower
  change, only below 3.0 m/s, and never down."""
  changed = 0
  for seq in SEQS:
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), module=frozen_cc, K9StopMode=2))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), K9StopMode=2))
    monkeypatch.undo()
    assert len(ref.can) == len(new.can) > 400
    for a, b in zip(ref.can, new.can, strict=True):
      assert [m for m in a if m[0] != 905] == [m for m in b if m[0] != 905]
    assert len(ref.scc14) == len(new.scc14)
    for (fo, vo, so), (fn, vn, sn) in zip(ref.scc14, new.scc14, strict=True):
      assert (fo, vo) == (fn, vn)
      for sig in so:
        if sig in ("ComfortBandUpper", "ComfortBandLower"):
          continue
        assert so[sig] == sn[sig], sig
      if vo >= 3.0:
        assert so == sn
      assert sn["ComfortBandUpper"] >= so["ComfortBandUpper"] and sn["ComfortBandLower"] >= so["ComfortBandLower"]
      changed += sn["ComfortBandUpper"] != so["ComfortBandUpper"]
  assert changed > 50


def test_menu2_low_speed_band_is_the_original_band(monkeypatch):
  r = Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), K9StopMode=2)
  rows = []
  for i in range(240):
    v = max(2.4 - 0.01 * i, 0.06)
    acc, _ = r.step(-0.6, LCS.pid if v > 0.3 else LCS.stopping, v, a_target=-0.5, a_ego=-0.5)
    rows.append(acc)
  n = 0
  for frame, v, s in r.scc14:
    if v <= 2.5 and frame >= 60:
      acc = rows[frame]
      assert s["ComfortBandUpper"] == pytest.approx(float(np.clip(0.9 + 0.2 * acc, 0, 1.2)), abs=0.011)
      assert s["ComfortBandLower"] == pytest.approx(float(np.clip(0.8 + 0.2 * acc, 0, 1.2)), abs=0.011)
      n += 1
  assert n > 50
