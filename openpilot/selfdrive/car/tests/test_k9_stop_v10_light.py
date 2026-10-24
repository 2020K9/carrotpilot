"""K9StopMode 5 = v10 + light shudder fix (the K9Creep kind V10_LIGHT = 6; 2026-10-24 brake_methods candidate L0.6_V9_f0,
the driver's order 08:03). update() (accel, StopReq, feather, latch ramp, 3.0 m lead latch, interlocks) and jerk_u are
exactly K9StopMode 1 (the page v10, test_k9_stop_v10.py) on every frame. Only SCC14 ComfortBand differs: where menu 1
sends 0/0, menu 5 sends 0.60 / 0.50 at or below 2.5 m/s (9 km/h), linear down to 0 at 3.0 m/s and 0 above (= menu 1), on
the exact 0.02 CAN grid, moved at most 0.04 per make_jerk call from the band sent before. Menus 0-4 are byte-identical to
the frozen c89f726e (carrot-wip before menu 5) controller. All new (2026-10-24). Sim/replay only; NOT verified on the
vehicle."""
import math
import random

import numpy as np
import pytest

from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V9, V10, V9_SHUDDER_FIX, V10_BAND02, V10_SMOOTH, V10_LIGHT
from opendbc.car.hyundai.values import CAR
from openpilot.selfdrive.car.tests.k9_rig import LCS, Rig, load_frozen_cc
from openpilot.selfdrive.car.tests.test_k9_creep_mode import SEQS, _random_inputs
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import make_cp
from openpilot.selfdrive.car.tests.test_k9_stop_v10 import V10Ref, _stops

FROZEN = "c89f726e"  # carrot-wip head before menu 5 (K9StopMode 0 base, 1 v10, 2 v9 + shudder fix, 3 v10 + band 0.2, 4 v10 + shudder fix)


def orig_bands(accel):
  return float(np.clip(0.9 + accel * 0.2, 0, 1.2)), float(np.clip(0.8 + accel * 0.2, 0, 1.2))


def on_grid(x):
  return abs(x * 50.0 - round(x * 50.0)) < 1e-9 and x == round(x * 50.0) / 50.0


def want_light(v):
  w = float(np.interp(v, [2.5, 3.0], [1.0, 0.0]))
  return round(30 * w) / 50.0, round(25 * w) / 50.0


def _upd(c, f, **kw):
  args = dict(accel=f["accel"], a_target=f["a_target"], stopping=f["stopping"], pid=f["pid"], v_ego=f["v_ego"],
              v_ego_raw=f["v_ego_raw"], a_ego=f["a_ego"], long_accel=f["long_accel"], esp_standstill=f["esp_standstill"],
              long_active=True, override=False, gas=False, brake=False, can_valid=True, lead_visible=False, lead_distance=0.0)
  args.update(kw)
  return c.update(**args)


@pytest.fixture(scope="module")
def frozen_cc(tmp_path_factory):
  return load_frozen_cc(FROZEN, tmp_path_factory.mktemp("cc_c89f726e"), "cc_c89f726e")


def test_menu_and_constants():
  assert kc.MENU[5] == V10_LIGHT == 6 and kc.MENU[4] == V10_SMOOTH and kc.MENU[1] == V10 and 5 in kc.MODES
  assert carcontroller.K9_CREEP_MENU[5] == V10_LIGHT
  c = K9Creep(V10_LIGHT)
  assert c.v10 and c.v10_light and not c.v10_band and not c.v10_smooth and not c.shudder_fix
  assert not any(K9Creep(k).v10_light for k in (V9, V10, V9_SHUDDER_FIX, V10_BAND02, V10_SMOOTH))
  assert (kc.LIGHT_UPPER, kc.LIGHT_LOWER, kc.LIGHT_FULL_SPEED, kc.LIGHT_OFF_SPEED) == (0.6, 0.5, 2.5, 3.0)
  assert (kc.LIGHT_UPPER_UNITS, kc.LIGHT_LOWER_UNITS, kc.LIGHT_STEP_UNITS, kc.CB_UNIT) == (30, 25, 2, 0.02)
  assert kc.LIGHT_STEP_UNITS * kc.CB_UNIT == pytest.approx(kc.V10_BAND_STEP)


@pytest.mark.parametrize("seed", range(8))
def test_accel_stopreq_and_jerk_are_menu_1_bit_for_bit(seed):
  # bands() called between update() calls (as make_jerk does) never changes accel/StopReq; jerk_u is menu 1's
  for frames in _stops(seed):
    ref, c1, c5 = V10Ref(), K9Creep(V10), K9Creep(V10_LIGHT)
    pu = pl = 0.0
    for f in frames:
      r5 = _upd(c5, f)
      assert r5 == ref.step(**f) == _upd(c1, f)
      if c5.active:
        cu, cl = orig_bands(f["accel"])
        ju = 0.5 + 0.1 * (f["v_ego"] % 3)
        j1 = c1.bands(ju, f["v_ego"], cu, cl, 0.0, 0.0)[0]
        j5, pu, pl = c5.bands(ju, f["v_ego"], cu, cl, pu, pl)
        assert j5 == j1


@pytest.mark.parametrize("seed", range(6))
def test_update_is_menu_1_on_random_inputs(seed):
  rng = random.Random(f"k9-v10-light-{seed}")
  c1, c5 = K9Creep(V10), K9Creep(V10_LIGHT)
  pu = pl = 0.0
  for kw, i in _random_inputs(rng, 4000):
    r1, r5 = c1.update(**kw), c5.update(**kw)
    assert r1 == r5 or (math.isnan(r1[0]) and math.isnan(r5[0]) and r1[1] == r5[1]), i
    assert (c1.latched, c1.active) == (c5.latched, c5.active)
    if i % 2 == 0 and c5.active:
      cu, cl = orig_bands(r5[0] if math.isfinite(r5[0]) else 0.0)
      ju = rng.uniform(0.5, 5.0)
      assert c5.bands(ju, kw["v_ego"], cu, cl, pu, pl)[0] == c1.bands(ju, kw["v_ego"], cu, cl, pu, pl)[0]
      _, pu, pl = c5.bands(ju, kw["v_ego"], cu, cl, pu, pl)
      assert on_grid(pu) and on_grid(pl)


@pytest.mark.parametrize("lead", [2.9, 2.5, 3.0, 1.0, 3.01, 0.0])
def test_lead_latch_and_interlocks_are_menu_1(lead):
  base = dict(accel=-0.3, a_target=-0.3, stopping=True, pid=False, v_ego=0.6, v_ego_raw=0.6, a_ego=-0.3, long_accel=-0.3,
              esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True, lead_visible=True)
  assert K9Creep(V10_LIGHT).update(lead_distance=lead, **base) == K9Creep(V10).update(lead_distance=lead, **base)
  for k in ("override", "gas", "brake"):
    assert K9Creep(V10_LIGHT).update(lead_distance=10.0, **dict(base, **{k: True})) == (-0.3, True)


def test_band_targets_by_speed_on_the_grid():
  for accel in (-0.3, -0.5, -0.8, -1.5, 0.2):
    cu, cl = orig_bands(accel)
    for v in (0.0, 0.03, 0.3, 1.0, 2.0, 2.49, 2.5, 2.6, 2.75, 2.9, 2.99, 3.0, 3.05, 4.0, 20.0):
      c = K9Creep(V10_LIGHT)
      pu, pl = cu, cl  # the original band was sent before the stop feathering applied
      for _ in range(60):
        ju, pu, pl = c.bands(0.5, v, cu, cl, pu, pl)
      assert ju == K9Creep.jerk_bands(0.5, v)[0]  # jerk_u is menu 1's
      assert (pu, pl) == want_light(v) and on_grid(pu) and on_grid(pl)
      if v <= 2.5:
        assert (pu, pl) == (0.6, 0.5)
      if v >= 3.0:
        assert pu == pl == 0.0  # exactly menu 1's band


def test_band_ramp_never_jumps_and_stays_on_the_grid():
  # a stop profile 6 -> 0 m/s with changing accel, started from every plausible previous band: <= 0.04 per call
  for start in (0.0, 0.2, 0.37, 0.8, 1.2):
    c = K9Creep(V10_LIGHT)
    pu = pl = start
    for i in range(400):
      v = max(6.0 - 0.02 * i, 0.0)
      accel = -0.4 - 0.4 * math.sin(i / 17.0)
      cu, cl = orig_bands(accel)
      _, nu, nl = c.bands(0.5, v, cu, cl, pu, pl)
      assert abs(nu - pu) <= 0.04 + 0.01 + 1e-9 and abs(nl - pl) <= 0.04 + 0.01 + 1e-9  # first call: grid rounding
      if i > 0:
        assert abs(nu - pu) <= 0.04 + 1e-9 and abs(nl - pl) <= 0.04 + 1e-9
      assert on_grid(nu) and on_grid(nl) and 0.0 <= nl <= nu <= max(0.6, start + 0.01)
      pu, pl = nu, nl
    assert (pu, pl) == (0.6, 0.5)


def test_band_non_finite_and_reset():
  assert K9Creep(V10_LIGHT).bands(0.5, float("nan"), 0.8, 0.7, 0.0, 0.0)[1:] == (0.0, 0.0)  # non-finite speed: menu 1
  assert K9Creep(V10_LIGHT).bands(0.5, 1.0, float("nan"), float("nan"), 0.0, 0.0)[1:] == (0.04, 0.04)  # level is fixed
  assert K9Creep(V10_LIGHT).bands(0.5, 1.0, 0.8, 0.7, float("nan"), float("nan"))[1:] == (0.04, 0.04)  # from 0
  assert K9Creep(V10_LIGHT).bands(0.5, 1.0, 0.8, 0.7, "x", None)[1:] == (0.04, 0.04)
  assert K9Creep(V10_LIGHT).bands(0.5, 1.0, 0.8, 0.7, 0.761, 0.659)[1:] == (0.72, 0.62)  # 0.76/0.66 grid, one 0.04 step
  c = K9Creep(V10_LIGHT)
  for _ in range(30):
    c.bands(0.5, 1.0, 0.8, 0.7, 0.8, 0.7)
  assert (c.lb_u, c.lb_l) == (30, 25)
  c.reset()  # interlock / mode change: starts again from the band sent before
  assert (c.lb_u, c.lb_l) == (None, None)
  assert c.bands(0.5, 1.0, 0.8, 0.7, 0.0, 0.0)[1:] == (0.04, 0.04)


def _stop_run(r):
  out = [r.step(-0.6, LCS.pid, 5.0, a_target=-0.5, a_ego=-0.5) for _ in range(60)]
  for i in range(160):
    out.append(r.step(-0.6, LCS.pid, 5.0 - 0.025 * i, a_target=-0.5, a_ego=-0.5))
  for i in range(150):
    out.append(r.step(-0.7, LCS.stopping, max(1.0 - 0.006 * i, 0.06), a_target=-0.3, a_ego=-0.3))
  for i in range(40):
    out.append(r.step(-0.7, LCS.stopping, 0.0, standstill=True, a_target=-0.3, a_ego=0.0))
  return out


def test_through_carcontroller_menu_5_only_scc14_band(monkeypatch):
  """menu 5 vs menu 1 through the CarController: accel and SCC12 identical on every frame, SCC14 differs only in
  ComfortBand (JerkUpper/Lower identical); settled at or below 2.5 m/s menu 5 sends exactly 0.60/0.50, at or above
  3.0 m/s exactly 0 (menu 1); <= 0.04 per frame; every sent band is on the 0.02 grid."""
  runs = {}
  for mode in (1, 5):
    r = Rig(monkeypatch, K9StopMode=mode)
    out = _stop_run(r)
    assert r.cc.k9_creep.mode == kc.MENU[mode]
    runs[mode] = (out, r.scc14)
    monkeypatch.undo()
  (o1, b1), (o5, b5) = runs[1], runs[5]
  assert o1 == o5
  low = high = 0
  prev = None
  hist = []
  for (f1, v1, d1), (f5, v5, d5) in zip(b1, b5, strict=True):
    assert f1 == f5 and v1 == v5
    for key in d1:
      if key not in ("ComfortBandUpper", "ComfortBandLower"):
        assert d5[key] == d1[key], key
    u, lo = d5["ComfortBandUpper"], d5["ComfortBandLower"]
    assert on_grid(round(u, 9)) and on_grid(round(lo, 9))
    if prev is not None:
      assert abs(u - prev[0]) <= 0.04 + 1e-6 and abs(lo - prev[1]) <= 0.04 + 1e-6
    prev = (u, lo)
    hist.append(v5)
    if d1["ComfortBandUpper"] == 0.0 and len(hist) > 30:
      if all(x >= 3.0 for x in hist[-30:]):
        assert u == 0.0 and lo == 0.0
        high += 1
      if all(x <= 2.5 for x in hist[-30:]):
        assert abs(u - 0.6) < 1e-6 and abs(lo - 0.5) < 1e-6
        low += 1
  assert low > 20 and high > 20


@pytest.mark.parametrize("seq", list(SEQS))
def test_through_carcontroller_menu_5_scc12_and_accel_equal_menu_1_on_all_sequences(monkeypatch, seq):
  runs = {}
  for mode in (1, 5):
    r = Rig(monkeypatch, K9StopMode=mode)
    acc = []
    orig_step = r.step

    def step(*a, _s=orig_step, **k):
      res = _s(*a, **k)
      acc.append(res)
      return res
    r.step = step
    SEQS[seq](r)
    runs[mode] = (acc, r.scc14, [[m for m in f if m[0] != 905] for f in r.can])
    monkeypatch.undo()
  assert runs[1][0] == runs[5][0]  # accel + decoded SCC12 every frame
  assert runs[1][2] == runs[5][2]  # every other CAN message byte-identical
  for (_, _, d1), (_, _, d5) in zip(runs[1][1], runs[5][1], strict=True):
    assert {k: v for k, v in d1.items() if "ComfortBand" not in k} == {k: v for k, v in d5.items() if "ComfortBand" not in k}


@pytest.mark.parametrize("seq", list(SEQS))
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0, 1, 2, 3, 4])
def test_menus_0_to_4_identical_can_to_c89f726e(monkeypatch, frozen_cc, mode, fingerprint, seq):
  kw = {} if mode is None else {"K9StopMode": mode}
  ref = SEQS[seq](Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), module=frozen_cc, **kw))
  monkeypatch.undo()
  new = SEQS[seq](Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), **kw))
  assert len(new.can) > 400 and ref.can == new.can
  assert ref.scc14 == new.scc14


@pytest.mark.parametrize("seq", list(SEQS))
def test_menu_5_on_k7_is_menu_0_of_c89f726e(monkeypatch, frozen_cc, seq):
  ref = SEQS[seq](Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), module=frozen_cc, K9StopMode=0))
  monkeypatch.undo()
  new = SEQS[seq](Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), K9StopMode=5))
  assert ref.can == new.can and new.cc.k9_creep is None


@pytest.mark.parametrize("seed", range(6))
@pytest.mark.parametrize("kind", (V9, V10, V9_SHUDDER_FIX, V10_BAND02, V10_SMOOTH))
def test_kinds_1_to_5_object_identical_to_c89f726e(frozen_cc, kind, seed):
  rng = random.Random(f"k9-light-frozen-{seed}")
  old, new = frozen_cc.K9Creep(kind), K9Creep(kind)
  assert old.__class__ is not K9Creep
  pu_o = pl_o = pu_n = pl_n = 0.0
  for kw, i in _random_inputs(rng, 4000):
    ro, rn = old.update(**kw), new.update(**kw)
    assert ro == rn or (math.isnan(ro[0]) and math.isnan(rn[0]) and ro[1] == rn[1]), (i, kw)
    if i % 2 == 0 and old.active:
      cu, cl = orig_bands(rn[0] if math.isfinite(rn[0]) else 0.0)
      ju = rng.uniform(0.5, 5.0)
      bo = old.bands(ju, kw["v_ego"], cu, cl, pu_o, pl_o)
      bn = new.bands(ju, kw["v_ego"], cu, cl, pu_n, pl_n)
      assert bo == bn, i
      (_, pu_o, pl_o), (_, pu_n, pl_n) = bo, bn


def test_menu_7_and_up_act_as_0(monkeypatch):
  # 2026-10-24: 6 is now v11 (test_k9_stop_v11.py) (menu-set pin rewritten)
  for value in (7, 8, 99, -1):
    r = Rig(monkeypatch, K9StopMode=value)
    r.step(-0.5, LCS.pid, 3.0)
    assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0
    monkeypatch.undo()


@pytest.mark.parametrize("start", [1, 3, 4])
def test_live_switch_to_5_band_does_not_jump(monkeypatch, start):
  r = Rig(monkeypatch, K9StopMode=start)
  for _ in range(60):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
  b0 = float(r.cc.hyundai_jerk.cb_upper)
  r.params.values["K9StopMode"] = 5
  bands = []
  for _ in range(80):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
    bands.append(float(r.cc.hyundai_jerk.cb_upper))
  assert r.cc.k9_creep.mode == V10_LIGHT
  d = np.abs(np.diff([b0] + bands))
  assert d[0] <= 0.04 + 0.01 + 1e-9 and np.all(d <= 0.04 + 0.01 + 1e-9) and np.sum(d > 0.04 + 1e-9) <= 1
  assert bands[-1] == 0.6 and float(r.cc.hyundai_jerk.cb_lower) == 0.5
