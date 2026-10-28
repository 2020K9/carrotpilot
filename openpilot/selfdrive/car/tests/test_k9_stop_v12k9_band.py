"""brake_v12k9_live_fix (2026-10-25 evening, 보노보노 17:46 "브레이크 좀 어떻게 해줘라.." / 18:02 absolute criteria): K9StopMode 2
(v12k9) ComfortBand. On the car the feathered band-0 stops of menus 1 and 2 shuddered at 6-7 Hz below 7 km/h (17:32:41, 17:39:05,
17:43:10), the band-on stops of menu 0 did not; kind 8 now sends the original ComfortBand (0.9 / 0.8 + 0.2 * accel) at or below
2.5 m/s, linear to 0 at 3.0 m/s, 0 at or above 3.0 m/s (as before); once StopReq is latched it ramps back to 0 by 0.04 per
make_jerk call (the latch phase is the old menu 2 again). Jerk limits unchanged.
brake_shudder_1025 (2026-10-26, 보노보노 23:54 / 00:50): with the band the logged K9 stops braked weaker than asked (+0.146 +- 0.054
m/s^2 per unit band); while the band is sent the decel request is V12K9_BAND_COMP stronger (times the sent band share, faded in
over accel 0 .. -0.3, none while accelerating), and a feathered stop that hangs below 0.3 m/s with the band (low-passed eff_ac
above -0.10 for 0.15 s) latches StopReq. Above 3.0 m/s and with the band off everything is the frozen kind 8.
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
  assert 0.0 < kc.V12K9_BAND_COMP <= 0.30 and kc.V12K9_BAND_COMP_FADE == 0.3
  assert (kc.V12K9_STALL_SPEED, kc.V12K9_STALL_EFF_AC, kc.V12K9_STALL_FRAMES) == (0.3, -0.10, 15)


def _kw1(v, accel=-0.5, stopping=False, a_ego=-0.5, long_accel=None, a_target=-0.5):
  return dict(accel=accel, a_target=a_target, stopping=stopping, pid=not stopping, v_ego=v, v_ego_raw=v, a_ego=a_ego,
              long_accel=long_accel, esp_standstill=False, long_active=True, override=False, gas=False, brake=False,
              can_valid=True, lead_visible=False, lead_distance=0.0)


def test_band_compensation_object():
  """the sent accel is the frozen command minus COMP x sent band share x fade(accel); none without the band or while accelerating"""
  c = K9Creep(V12K9)
  assert c.update(**_kw1(2.0, accel=-0.5))[0] == -0.5  # no band sent yet
  c.bands(0.5, 2.0, 0.8, 0.7, 0.0, 0.0)  # full band at 2.0 m/s
  assert c.band_w == 1.0
  assert c.update(**_kw1(2.0, accel=-0.5))[0] == pytest.approx(-0.5 - kc.V12K9_BAND_COMP)
  assert c.update(**_kw1(2.0, accel=-0.15))[0] == pytest.approx(-0.15 - kc.V12K9_BAND_COMP * 0.5)
  assert c.update(**_kw1(2.0, accel=0.3))[0] == 0.3
  c.bands(0.5, 2.75, 0.8, 0.7, 0.8, 0.7)  # half band
  assert c.band_w == pytest.approx(0.5)
  # 2026-10-26 (brake_0241): the brake-only slow trim (c.trim <= 0) is added on top of the compensation
  assert c.update(**_kw1(2.75, accel=-0.6))[0] == pytest.approx(-0.6 - kc.V12K9_BAND_COMP * 0.5 + c.trim) and c.trim <= 0.0
  c.bands(0.5, 3.2, 0.8, 0.7, 0.4, 0.35)  # band 0 above 3.0 m/s: frozen kind 8
  assert c.band_w == 0.0 and c.update(**_kw1(3.2, accel=-0.6))[0] == -0.6
  assert c.last == -0.6  # the feather / rate-limit state never carries the compensation


def test_stall_latch_object():
  """stopping with the band below 0.3 m/s and no decel for 0.15 s -> StopReq; never without the band or while decelerating.
  2026-10-26 (brake_0241): the stall latch is gone; the crawl (below V12K9_END_CRAWL_SPEED, no decel) gets StopReq from the end
  profile's crawl fallback after V12K9_END_CRAWL_FRAMES, with or without the band; never while decelerating."""
  def run(band, a_ego, n=90):
    c = K9Creep(V12K9)
    c.bands(0.5, 0.2, 0.84, 0.74, 0.0, 0.0) if band else None
    out = []
    for _ in range(n):
      out.append(c.update(**_kw1(0.12, accel=-0.4, stopping=True, a_ego=a_ego, long_accel=a_ego, a_target=-0.3))[1])
      if band and not c.latched:
        c.bands(0.5, 0.12, 0.84, 0.74, 0.84, 0.74)
    return out
  n = kc.V12K9_END_CRAWL_FRAMES
  for band in (True, False):
    hang = run(band, 0.0)
    assert hang[:n] == [False] * n and all(hang[n + 1:])
  assert not any(run(True, -0.3))


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
  r = Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), K9StopMode=1)  # menu 1 since the 04:48 swap
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
  """accel / StopReq / latch state are bit for bit the frozen kind 8 for the same inputs.
  2026-10-26 (brake_0241, zone approved 08:54): bit for bit on the zone reference (c.ref / c.ref_out); kind 8 itself outside the
  zone: same StopReq / latch, accel within [frozen - V12K9_BAND_COMP, frozen]."""
  old, new = frozen_cc.K9Creep(V12K9), K9Creep(V12K9)
  rng = random.Random(f"k9-v12k9-band-{seed}")
  for kw, i in _random_inputs(rng, 3000):
    ro, rn = old.update(**kw), new.update(**kw)
    rr = new.ref_out
    assert (math.isnan(ro[0]) and math.isnan(rr)) or ro[0] == rr, (i, kw, ro, rr)
    assert (old.latched, old.ss_count) == (new.ref.latched, new.ref.ss_count), i
    assert old.last == new.ref.last or (math.isnan(old.last) and math.isnan(new.ref.last)), i
    lead_in = (kw["lead_visible"] or kw["lead_distance"] != 0.0) and kw["lead_distance"] <= kc.V12K9_ZONE_LEAD
    if kw["stopping"] and kw["v_ego"] < kc.V12K9_ZONE_SPEED and not lead_in:
      continue
    assert ro[1] == rn[1] and (old.latched, old.active) == (new.latched, new.active), (i, kw)
    if not (math.isnan(ro[0]) or math.isnan(rn[0])):
      assert ro[0] - kc.V12K9_BAND_COMP - 1e-12 <= rn[0] <= ro[0] + 1e-12, (i, kw, ro, rn)


def _kw(mode):
  return {} if mode is None else {"K9StopMode": mode}


@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0, 5, 99, -1])  # 3 -> 4 on 2026-10-26: 3 = v13 original (new), 4 acts as 0; 2026-10-28:
# 3 = v14 original, 4 still acts as 0; 2026-10-29 04:48: 1 and 2 swapped methods, so both are pinned across numbers below
def test_menus_other_than_the_k9_menu_can_identical_to_frozen(monkeypatch, frozen_cc, mode, fingerprint):
  for seq in SEQS:
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), module=frozen_cc, **_kw(mode)))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), **_kw(mode)))
    monkeypatch.undo()
    assert len(new.can) > 400 and ref.can == new.can and ref.scc14 == new.scc14


def test_menu1_other_cars_can_identical_to_frozen(monkeypatch, frozen_cc):
  for seq in SEQS:
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), module=frozen_cc, K9StopMode=2))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), K9StopMode=1))  # same method, menu 1 since 04:48
    monkeypatch.undo()
    assert ref.can == new.can


def _zone(inp):
  state, v, lead, lead_visible = inp
  lead_in = lead is not None and (lead_visible or lead[0] != 0.0) and lead[0] <= kc.V12K9_ZONE_LEAD
  return state == LCS.stopping and v < kc.V12K9_ZONE_SPEED and not lead_in


def test_menu1_only_band_compensation_and_stall_change(monkeypatch, frozen_cc):
  """K9 menu 2 against the frozen menu 2 (6f518e35): every message other than SCC12 / SCC14 is byte-identical; at or above
  3.0 m/s SCC12 and SCC14 are identical; below it the band is never lower, aReqValue never higher (more decel by at most
  V12K9_BAND_COMP) and StopReq never earlier-released (only earlier set, by the stall latch). Was 62eb5798's pin "only the band
  changes", updated for the band compensation. 2026-10-26 (the driver's approval 08:54): aReqValue / StopReq are not checked in
  the zone (stopping, v < kc.V12K9_ZONE_SPEED, no lead within kc.V12K9_ZONE_LEAD; _zone below)."""
  changed = 0
  for seq in SEQS:
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), module=frozen_cc, K9StopMode=2))  # frozen: menu 2
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), K9StopMode=1))  # tree: menu 1 since 04:48
    monkeypatch.undo()
    assert len(ref.can) == len(new.can) > 400
    for a, b in zip(ref.can, new.can, strict=True):
      assert [m for m in a if m[0] not in (905, 1057)] == [m for m in b if m[0] not in (905, 1057)]
    assert len(ref.scc12) == len(new.scc12) and len(ref.scc14) == len(new.scc14)
    for (fo, vo, so), (fn, vn, sn) in zip(ref.scc12, new.scc12, strict=True):
      assert (fo, vo) == (fn, vn)
      if vo >= 3.0:
        assert so == sn
        continue
      if _zone(new.inputs[fn]):
        continue
      assert sn["aReqValue"] <= so["aReqValue"] + 0.011 and so["aReqValue"] - sn["aReqValue"] <= kc.V12K9_BAND_COMP + 0.011
      assert sn["StopReq"] >= so["StopReq"]
      changed += sn["aReqValue"] != so["aReqValue"]
    for (fo, vo, so), (fn, vn, sn) in zip(ref.scc14, new.scc14, strict=True):
      assert (fo, vo) == (fn, vn)
      if vo >= 3.0:
        assert so == sn
      for sig in so:
        if sig not in ("ComfortBandUpper", "ComfortBandLower"):
          assert so[sig] == sn[sig], sig
      assert sn["ComfortBandUpper"] >= so["ComfortBandUpper"] and sn["ComfortBandLower"] >= so["ComfortBandLower"]
  assert changed > 50


def test_menu1_low_speed_band_is_the_original_band(monkeypatch):
  r = Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), K9StopMode=1)  # menu 1 since the 04:48 swap
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
