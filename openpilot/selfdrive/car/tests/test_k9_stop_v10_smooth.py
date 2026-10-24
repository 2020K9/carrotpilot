"""K9StopMode 4 = v10 + shudder fix (the K9Creep kind V10_SMOOTH = 5; 2026-10-24 brake smooth_next, the driver's order 07:53).
update() (accel, StopReq, feather, latch ramp, 3.0 m lead latch, interlocks) is exactly K9StopMode 1 (the page v10,
test_k9_stop_v10.py) on every frame. Only SCC14 ComfortBand differs: where menu 1 sends 0/0, menu 4 sends the original band
(0.9 + 0.2*accel / 0.8 + 0.2*accel, = menu 0) at or below 2.0 m/s and 0.20 (= menu 3) at or above 3.0 m/s, blended between,
moved at most 0.04 per make_jerk call from the band sent before. Menus 0-3 are byte-identical to the frozen 30d35895
(carrot-wip before menu 4) controller. All new (2026-10-24). Sim/replay only; NOT verified on the vehicle."""
import math
import random

import numpy as np
import pytest

from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V9, V10, V9_SHUDDER_FIX, V10_BAND02, V10_SMOOTH
from opendbc.car.hyundai.values import CAR
from openpilot.selfdrive.car.tests.k9_rig import LCS, Rig, load_frozen_cc
from openpilot.selfdrive.car.tests.test_k9_creep_mode import SEQS, _random_inputs
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import make_cp
from openpilot.selfdrive.car.tests.test_k9_stop_v10 import V10Ref, _stops

FROZEN = "30d35895"  # carrot-wip head before menu 4 (K9StopMode 0 base, 1 v10, 2 v9 + shudder fix, 3 v10 + band 0.2)


def orig_bands(accel):
  return float(np.clip(0.9 + accel * 0.2, 0, 1.2)), float(np.clip(0.8 + accel * 0.2, 0, 1.2))


def _upd(c, f, **kw):
  args = dict(accel=f["accel"], a_target=f["a_target"], stopping=f["stopping"], pid=f["pid"], v_ego=f["v_ego"],
              v_ego_raw=f["v_ego_raw"], a_ego=f["a_ego"], long_accel=f["long_accel"], esp_standstill=f["esp_standstill"],
              long_active=True, override=False, gas=False, brake=False, can_valid=True, lead_visible=False, lead_distance=0.0)
  args.update(kw)
  return c.update(**args)


@pytest.fixture(scope="module")
def frozen_cc(tmp_path_factory):
  return load_frozen_cc(FROZEN, tmp_path_factory.mktemp("cc_30d35895"), "cc_30d35895")


def test_menu_and_constants():
  assert kc.MENU[4] == V10_SMOOTH == 5 and kc.MENU[3] == V10_BAND02 and kc.MENU[1] == V10 and 4 in kc.MODES
  c = K9Creep(V10_SMOOTH)
  assert c.v10 and c.v10_band and c.v10_smooth and not c.shudder_fix
  assert not any(K9Creep(k).v10_smooth for k in (V9, V10, V9_SHUDDER_FIX, V10_BAND02))
  assert (kc.BAND_FULL_SPEED, kc.BAND_OFF_SPEED, kc.V10_BAND, kc.V10_BAND_STEP) == (2.0, 3.0, 0.20, 0.04)


@pytest.mark.parametrize("seed", range(8))
def test_accel_and_stopreq_are_menu_1_bit_for_bit(seed):
  # bands() called between update() calls (as make_jerk does) never changes accel/StopReq: menu 1 bit for bit
  for frames in _stops(seed):
    ref, c4 = V10Ref(), K9Creep(V10_SMOOTH)
    pu = pl = 0.0
    for f in frames:
      assert _upd(c4, f) == ref.step(**f)
      if c4.active:
        cu, cl = orig_bands(f["accel"])
        _, pu, pl = c4.bands(1.0, f["v_ego"], cu, cl, pu, pl)


@pytest.mark.parametrize("seed", range(6))
def test_update_is_menu_1_on_random_inputs(seed):
  rng = random.Random(f"k9-v10-smooth-{seed}")
  c1, c4 = K9Creep(V10), K9Creep(V10_SMOOTH)
  for kw, i in _random_inputs(rng, 4000):
    r1, r4 = c1.update(**kw), c4.update(**kw)
    assert r1 == r4 or (math.isnan(r1[0]) and math.isnan(r4[0]) and r1[1] == r4[1]), i
    assert (c1.latched, c1.active) == (c4.latched, c4.active)


@pytest.mark.parametrize("lead", [2.9, 2.5, 3.0, 1.0, 3.01, 0.0])
def test_lead_latch_and_interlocks_are_menu_1(lead):
  base = dict(accel=-0.3, a_target=-0.3, stopping=True, pid=False, v_ego=0.6, v_ego_raw=0.6, a_ego=-0.3, long_accel=-0.3,
              esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True, lead_visible=True)
  assert K9Creep(V10_SMOOTH).update(lead_distance=lead, **base) == K9Creep(V10).update(lead_distance=lead, **base)
  for k in ("override", "gas", "brake"):
    assert K9Creep(V10_SMOOTH).update(lead_distance=10.0, **dict(base, **{k: True})) == (-0.3, True)


def test_band_targets_by_speed():
  for accel in (-0.3, -0.5, -0.8, -1.5, 0.2):
    cu, cl = orig_bands(accel)
    for v in (0.0, 0.3, 1.0, 1.99, 2.0, 2.25, 2.5, 2.75, 3.0, 4.0, 20.0):
      w = float(np.interp(v, [2.0, 3.0], [1.0, 0.0]))
      want_u, want_l = max(0.2, cu * w), max(0.2, cl * w)
      c = K9Creep(V10_SMOOTH)
      pu, pl = 0.0, 0.0
      for _ in range(60):
        ju, pu, pl = c.bands(0.5, v, cu, cl, pu, pl)
      assert ju == K9Creep.jerk_bands(0.5, v)[0]  # jerk_u is menu 1's
      assert pu == pytest.approx(want_u, abs=1e-12) and pl == pytest.approx(want_l, abs=1e-12)
      if v <= 2.0:
        assert (pu, pl) == (cu, cl)  # exactly the menu 0 band
      if v >= 3.0:
        assert pu == pl == kc.V10_BAND  # exactly menu 3's 0.20


def test_band_ramp_never_jumps():
  # a stop profile 6 -> 0 m/s with changing accel, started from every plausible previous band: <= 0.04 per call
  for start in (0.0, 0.2, 0.8, 1.2):
    c = K9Creep(V10_SMOOTH)
    pu = pl = start
    for i in range(400):
      v = max(6.0 - 0.02 * i, 0.0)
      accel = -0.4 - 0.4 * math.sin(i / 17.0)
      cu, cl = orig_bands(accel)
      _, nu, nl = c.bands(0.5, v, cu, cl, pu, pl)
      assert abs(nu - pu) <= 0.04 + 1e-9 and abs(nl - pl) <= 0.04 + 1e-9
      assert 0.0 <= nu <= 1.2 and 0.0 <= nl <= 1.2
      pu, pl = nu, nl


def test_band_non_finite_and_reset():
  c = K9Creep(V10_SMOOTH)
  assert c.bands(0.5, float("nan"), 0.8, 0.7, 0.2, 0.2)[1:] == (0.2, 0.2)  # non-finite speed: menu 3 value
  assert K9Creep(V10_SMOOTH).bands(0.5, 1.0, float("nan"), float("nan"), 0.2, 0.2)[1:] == (0.2, 0.2)
  assert K9Creep(V10_SMOOTH).bands(0.5, 1.0, 0.8, 0.7, float("nan"), float("nan"))[1:] == pytest.approx((0.24, 0.24))  # from 0.2
  c = K9Creep(V10_SMOOTH)
  for _ in range(30):
    c.bands(0.5, 1.0, 0.8, 0.7, 0.8, 0.7)
  c.reset()  # interlock / mode change: starts again from the band sent before
  assert c.bands(0.5, 1.0, 0.8, 0.7, 0.0, 0.0)[1:] == (0.04, 0.04)


def test_through_carcontroller_menu_4_only_scc14_band(monkeypatch):
  """menu 4 vs menu 1 through the CarController: accel and SCC12 identical on every frame, SCC14 differs only in
  ComfortBand; at or below 2.0 m/s (settled) menu 4 sends the original band, at or above 3.0 m/s 0.20; <= 0.04 per frame."""
  runs = {}
  for mode in (1, 4):
    r = Rig(monkeypatch, K9StopMode=mode)
    out = [r.step(-0.6, LCS.pid, 5.0, a_target=-0.5, a_ego=-0.5) for _ in range(60)]
    for i in range(160):
      out.append(r.step(-0.6, LCS.pid, 5.0 - 0.025 * i, a_target=-0.5, a_ego=-0.5))
    for i in range(150):
      out.append(r.step(-0.7, LCS.stopping, max(1.0 - 0.006 * i, 0.06), a_target=-0.3, a_ego=-0.3))
    for i in range(40):
      out.append(r.step(-0.7, LCS.stopping, 0.0, standstill=True, a_target=-0.3, a_ego=0.0))
    assert r.cc.k9_creep.mode == kc.MENU[mode]
    runs[mode] = (out, r.scc14)
    monkeypatch.undo()
  (o1, b1), (o4, b4) = runs[1], runs[4]
  assert o1 == o4
  low = high = 0
  prev = None
  hist = []
  for (f1, v1, d1), (f4, v4, d4) in zip(b1, b4, strict=True):
    assert f1 == f4 and v1 == v4
    for key in d1:
      if key not in ("ComfortBandUpper", "ComfortBandLower"):
        assert d4[key] == d1[key], key
    u, lo = d4["ComfortBandUpper"], d4["ComfortBandLower"]
    if prev is not None:
      assert abs(u - prev[0]) <= 0.04 + 1e-9 and abs(lo - prev[1]) <= 0.04 + 1e-9
    prev = (u, lo)
    hist.append(v4)
    if d1["ComfortBandUpper"] == 0.0 and len(hist) > 30:
      if all(x >= 3.0 for x in hist[-30:]):
        assert abs(u - 0.2) < 1e-9 and abs(lo - 0.2) < 1e-9
        high += 1
      if all(x <= 2.0 for x in hist[-30:]):
        assert u > 0.6 and lo > 0.5 and abs((u - lo) - 0.1) < 0.021  # original band (0.02 CAN grid)
        low += 1
  assert low > 20 and high > 20


@pytest.mark.parametrize("seq", list(SEQS))
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0, 1, 2, 3])
def test_menus_0_to_3_identical_can_to_30d35895(monkeypatch, frozen_cc, mode, fingerprint, seq):
  kw = {} if mode is None else {"K9StopMode": mode}
  ref = SEQS[seq](Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), module=frozen_cc, **kw))
  monkeypatch.undo()
  new = SEQS[seq](Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), **kw))
  assert len(new.can) > 400 and ref.can == new.can
  assert ref.scc14 == new.scc14


@pytest.mark.parametrize("seed", range(6))
@pytest.mark.parametrize("kind", (V9, V10, V9_SHUDDER_FIX, V10_BAND02))
def test_kinds_1_to_4_object_identical_to_30d35895(frozen_cc, kind, seed):
  rng = random.Random(f"k9-smooth-frozen-{seed}")
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


def test_menu_6_and_up_act_as_0(monkeypatch):
  # 2026-10-24: 5 is now v10 + light shudder fix (menu-set pin rewritten; test_k9_stop_v10_light.py)
  for value in (6, 7, -1):
    r = Rig(monkeypatch, K9StopMode=value)
    r.step(-0.5, LCS.pid, 3.0)
    assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0
    monkeypatch.undo()


def test_live_switch_3_to_4_band_does_not_jump(monkeypatch):
  r = Rig(monkeypatch, K9StopMode=3)
  for _ in range(60):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
  assert abs(r.cc.hyundai_jerk.cb_upper - 0.2) < 1e-9
  r.params.values["K9StopMode"] = 4
  bands = []
  for _ in range(80):
    sent, _ = r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
    bands.append(float(r.cc.hyundai_jerk.cb_upper))
  assert r.cc.k9_creep.mode == V10_SMOOTH
  assert np.all(np.abs(np.diff([0.2] + bands)) <= 0.04 + 1e-9)
  assert bands[-1] == pytest.approx(orig_bands(sent)[0])  # the menu 0 band of the sent (v10) accel
  assert carcontroller.K9_CREEP_MENU[4] == V10_SMOOTH
