"""K9CreepMode single selector: 0 base braking, 1 creep v8, 2 creep v9, 3 v9 + shudder fix 1 (C2: low-speed
ComfortBand restore), 4 v9 + shudder fix 2 (C2 + C3: low-speed hold), 5 v9 + shudder fix 3 (BL: below 0.8 m/s
blend toward base braking, more-braking side only, ComfortBand restored there), 6 v9 + shudder fix 4 (mode-3 band +
braking compensation, never weaker than v9). Other values act as 0.

0/1/2 are checked byte-for-byte against the real frozen 685223c8 controller (the previous menu), with and without a
stale K9StopTaper left in Params; 0-4 against the frozen 55a8e096 controller (the menu before mode 5); 0-5 against the
frozen 5d345a08 controller (the menu before mode 6).
Checks source logic and SCC12/SCC14 encoding, not K9 vehicle behaviour.
"""
import math
import random

import numpy as np
import pytest

from opendbc.car import structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V8, V9, V9_BL, V9_C2, V9_C2BL, V9_C2C3
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.selfdrive.car.tests import k9_rig
from openpilot.selfdrive.car.tests.k9_rig import (CC_REL, LCS, ROOT, Rig, edge_sequence, gentle_sequence, load_frozen_cc,
                                                  mixed_sequence)
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import K9_FLAGS, make_cp

PREV_MENU = "685223c8"  # 0 base, 1 v8, 2 v9, 3 the deleted taper
MENU_0_4 = "55a8e096"  # 0 base, 1 v8, 2 v9, 3 v9+C2, 4 v9+C2+C3 (before mode 5)
MENU_0_5 = "5d345a08"  # 0-4 as above, 5 v9+BL (before mode 6)
ALL = (V8, V9, V9_C2, V9_C2C3, V9_BL, V9_C2BL)
SEQS = {"mixed": mixed_sequence, "edge": edge_sequence, "gentle": gentle_sequence}


def upd(c, **kw):
  args = dict(accel=-1.0, a_target=-0.5, stopping=True, pid=False, v_ego=0.3, v_ego_raw=0.3, a_ego=-0.3, long_accel=-0.3,
              esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True,
              lead_visible=False, lead_distance=0.0)
  args.update(kw)
  return c.update(**args)


@pytest.fixture(scope="module")
def prev_cc(tmp_path_factory):
  return load_frozen_cc(PREV_MENU, tmp_path_factory.mktemp("prev_cc"), "cc_685223c8")


@pytest.fixture(scope="module")
def menu4_cc(tmp_path_factory):
  return load_frozen_cc(MENU_0_4, tmp_path_factory.mktemp("menu4_cc"), "cc_55a8e096")


@pytest.fixture(scope="module")
def menu5_cc(tmp_path_factory):
  return load_frozen_cc(MENU_0_5, tmp_path_factory.mktemp("menu5_cc"), "cc_5d345a08")


def rig(monkeypatch, mode, stale_taper=False, cp=None, module=None):
  kw = {} if mode is None else {"K9CreepMode": mode}
  return Rig(monkeypatch, cp=cp, stale_taper=stale_taper, **({"module": module} if module else {}), **kw)


# --- unit -----------------------------------------------------------------------------------------------------

def test_applies_only_k9_camera_scc_openpilot_long():
  assert kc.k9_creep_applies(make_cp())
  assert not kc.k9_creep_applies(make_cp(fingerprint=CAR.KIA_K7))
  assert not kc.k9_creep_applies(make_cp(flags=int(K9_FLAGS | HyundaiFlags.CANFD)))
  assert not kc.k9_creep_applies(make_cp(flags=int(K9_FLAGS & ~HyundaiFlags.CAMERA_SCC)))
  cp = make_cp()
  cp.openpilotLongitudinalControl = False
  assert not kc.k9_creep_applies(cp)


def test_profile_constants_match_source():
  v8, v9 = kc.PROFILES[V8], kc.PROFILES[V9]
  assert v8["feather"] == ([0.05, 0.13, 0.22, 0.38], [-0.26, -0.28, -0.35, -0.48])
  assert v8["min_decel"] == ([0.08, 0.13, 0.25], [0.02, -0.06, -0.14])
  assert v8["creep"] == ([0.18, 0.40, 1.0, 1.6], [-0.18, -0.24, -0.20, 0.0])
  assert v9["feather"] == ([0.05, 0.14, 0.25, 0.42], [-0.31, -0.35, -0.42, -0.52])
  assert v9["creep"] == ([0.25, 0.55, 1.0, 1.6], [-0.26, -0.26, -0.20, 0.0])
  assert (v8["down_eff_ac"], v9["down_eff_ac"]) == (-0.03, -0.02)
  assert v8["up"](0.2, -0.05, 0.0) == 0.0 and v8["up"](0.2, -0.07, 0.0) == 0.0035 and v8["up"](0.1, 0.0, 0.0) == 0.0035
  assert v9["up"](0.05, 0.0, 0.0) == 0.0032 and v9["up"](0.2, 0.0, 0.02) == 0.0
  assert kc.MODES == (1, 2, 3, 4, 5, 6) and K9Creep(V9_C2).profile is v9 and K9Creep(V9_C2C3).profile is v9
  assert K9Creep(V9_BL).profile is v9 and K9Creep(V9_C2BL).profile is v9
  assert (kc.C2BL_GAIN, kc.C2BL_COMP, kc.C2BL_COMP_FADE_BP) == (0.5, 0.10, (0.3, 0.6))
  assert (kc.CB_FULL_SPEED, kc.CB_OFF_SPEED, kc.CB_RAMP_STEP, kc.C3_HOLD_SPEED, kc.C3_RELEASE_MAX) == (2.0, 3.0, 0.04, 0.14, 0.0)
  assert (kc.BL_FULL_SPEED, kc.BL_OFF_SPEED) == (0.6, 0.8)
  flags = [(K9Creep(m).cb_restore, K9Creep(m).low_hold, K9Creep(m).low_blend) for m in kc.MODES]
  assert flags == [(False, False, False), (False, False, False), (True, False, False), (True, True, False),
                   (False, False, True), (True, False, True)]
  assert [K9Creep(m).bl_band for m in kc.MODES] == [False, False, False, False, True, False]


@pytest.mark.parametrize("mode", ALL)
def test_withholds_stopreq_then_latches_immediately(mode):
  c = K9Creep(mode)
  a, s = upd(c)
  assert s is False and (a > -1.0 or mode == V9_BL)  # 5 at 0.3 m/s sends the stronger original (-1.0)
  for kw in ({"esp_standstill": True}, {"v_ego_raw": 0.01}, {"v_ego": 0.04}, {"a_target": -1.0}, {"a_ego": 0.07},
             {"lead_visible": True, "lead_distance": 2.0}, {"lead_distance": 2.5}, {"long_accel": 0.2, "a_ego": 0.0},
             {"v_ego": 0.12, "v_ego_raw": 0.12, "a_target": -1.0}):
    c, ref = K9Creep(mode), K9Creep(V9)
    upd(c)
    upd(ref)
    a, s = upd(c, **kw)
    assert s is True and a <= -0.5  # same frame
    if mode != V8:
      assert (a, s) == upd(ref, **kw)  # 3/4/5 latch exactly like v9


@pytest.mark.parametrize("mode", ALL)
@pytest.mark.parametrize("kw", [{"gas": True}, {"brake": True}, {"override": True}, {"long_active": False},
                                {"can_valid": False}, {"v_ego": float("nan")}, {"accel": float("nan")}])
def test_interlocks_return_original(mode, kw):
  c = K9Creep(mode)
  upd(c)
  orig = (kw.get("accel", -1.0), True)
  out = upd(c, **kw)
  assert (out[0] == orig[0] or (math.isnan(out[0]) and math.isnan(orig[0]))) and out[1] is True and not c.active
  assert c.cb_w is None and c.last is None


def test_v8_v9_differ():
  outs = []
  for mode in (V8, V9):
    c = K9Creep(mode)
    outs.append([upd(c, v_ego=v, v_ego_raw=v)[0] for v in (0.4, 0.38, 0.35, 0.3, 0.25, 0.2)])
  assert outs[0] != outs[1]


def _feather(mode, speeds, **kw):
  c = K9Creep(mode)
  return [upd(c, v_ego=v, v_ego_raw=v, **kw) for v in speeds]


@pytest.mark.parametrize("margin_kw", [{}, {"long_accel": -0.05, "a_ego": -0.1}, {"long_accel": -0.6, "a_ego": -0.6},
                                       {"long_accel": None, "a_ego": -0.2}])
def test_c3_hold_never_releases_and_equals_v9_above_hold_speed(margin_kw):
  speeds = list(np.linspace(0.40, 0.06, 60))
  v9 = _feather(V9, speeds, **margin_kw)
  c2 = _feather(V9_C2, speeds, **margin_kw)
  c3 = _feather(V9_C2C3, speeds, **margin_kw)
  assert c2 == v9  # C2 does not touch accel/StopReq
  prev = None
  for v, (a9, s9), (a4, s4) in zip(speeds, v9, c3, strict=True):
    assert s4 == s9 is False
    if v > kc.C3_HOLD_SPEED and (prev is None or prev[0] > kc.C3_HOLD_SPEED):
      assert a4 == a9
    if v <= kc.C3_HOLD_SPEED:
      assert a4 <= a9 + 1e-12  # equal or more braking than v9
      assert a4 - prev[1] <= 0.0  # RELEASE_MAX = 0: delta <= 0
    prev = (v, a4)


@pytest.mark.parametrize("release_max", [0.0, 0.001])
def test_c3_release_max_bounds_delta(monkeypatch, release_max):
  monkeypatch.setattr(kc, "C3_RELEASE_MAX", release_max)
  out = _feather(V9_C2C3, list(np.linspace(0.14, 0.06, 40)), long_accel=0.02, a_ego=-0.05)
  deltas = np.diff([a for a, _ in out])
  assert np.all(deltas <= release_max + 1e-12)
  if release_max == 0.0:
    assert np.all(deltas <= 0.0)


def test_c2_band_weight_target_and_ramp_unit():
  assert kc.cb_target_weight(0.0) == 1.0 and kc.cb_target_weight(2.0) == 1.0 and kc.cb_target_weight(2.5) == 0.5
  assert kc.cb_target_weight(3.0) == 0.0 and kc.cb_target_weight(10.0) == 0.0 and kc.cb_target_weight(float("nan")) == 1.0
  for mode in (V8, V9):  # modes 1/2: exactly the v8/v9 band
    assert K9Creep(mode).bands(0.5, 0.3, 0.8, 0.7, 0.8) == K9Creep.jerk_bands(0.5, 0.3)
  c = K9Creep(V9_C2)
  ws = []
  for _ in range(30):  # prev band 0 (v9 at speed) at 0.3 m/s: ramps 0 -> 1 in 25 calls
    ju, cu, cl = c.bands(0.5, 0.3, 0.8, 0.7, 0.0 if not ws else ws[-1] * 0.8)
    assert ju == K9Creep.jerk_bands(0.5, 0.3)[0]
    ws.append(c.cb_w)
    assert cu == pytest.approx(0.8 * c.cb_w) and cl == pytest.approx(0.7 * c.cb_w)
  assert ws[0] == pytest.approx(0.04) and ws[24] == pytest.approx(1.0) and ws[-1] == 1.0
  assert np.all(np.abs(np.diff([0.0] + ws)) <= kc.CB_RAMP_STEP + 1e-12)
  c.reset()
  c.bands(0.5, 0.3, 0.8, 0.7, 0.8)  # after a reset it starts from the band actually sent (original): no jump
  assert c.cb_w == 1.0


# --- CarController: 0/1/2 byte-identical to 685223c8 ------------------------------------------------------------

@pytest.mark.parametrize("seq", list(SEQS))
@pytest.mark.parametrize("stale", [False, True], ids=["no_stale_taper", "stale_taper_on"])
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0, 1, 2])
def test_modes_012_identical_can_to_previous_menu(monkeypatch, prev_cc, mode, fingerprint, stale, seq):
  # 685223c8 mode 0 already ignored K9StopTaper on the K9; its K7 never had the K9 logic.
  ref = SEQS[seq](rig(monkeypatch, mode, stale_taper=stale, cp=make_cp(fingerprint=fingerprint), module=prev_cc))
  monkeypatch.undo()
  new = SEQS[seq](rig(monkeypatch, mode, stale_taper=stale, cp=make_cp(fingerprint=fingerprint)))
  assert len(new.can) > 1000 and ref.can == new.can
  assert "K9StopTaper" not in new.params.reads


@pytest.mark.parametrize("value", [7, 8, 99, -1, -4, float("nan"), "nan", float("inf")])
def test_out_of_range_values_act_as_0(monkeypatch, prev_cc, menu4_cc, menu5_cc, value):
  ref = mixed_sequence(rig(monkeypatch, 0, module=prev_cc))
  monkeypatch.undo()
  ref4 = mixed_sequence(rig(monkeypatch, 0, module=menu4_cc))
  monkeypatch.undo()
  ref5 = mixed_sequence(rig(monkeypatch, 0, module=menu5_cc))
  monkeypatch.undo()
  r = mixed_sequence(rig(monkeypatch, value))
  assert r.can == ref.can == ref4.can == ref5.can and r.cc.k9_creep is None and r.cc.k9_creep_mode == 0


# --- CarController: 0-5 byte-identical to 5d345a08 (the menu before mode 6) --------------------------------------

@pytest.mark.parametrize("seq", list(SEQS) + ["blstop"])
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0, 1, 2, 3, 4, 5])
def test_modes_012345_identical_can_to_menu_before_mode6(monkeypatch, menu5_cc, mode, fingerprint, seq):
  run = SEQS.get(seq, bl_stop_sequence)
  ref = run(rig(monkeypatch, mode, cp=make_cp(fingerprint=fingerprint), module=menu5_cc))
  monkeypatch.undo()
  new = run(rig(monkeypatch, mode, cp=make_cp(fingerprint=fingerprint)))
  assert len(new.can) > 400 and ref.can == new.can
  assert ref.scc14 == new.scc14


# --- CarController: 0-4 byte-identical to 55a8e096 (the menu before mode 5) --------------------------------------

@pytest.mark.parametrize("seq", list(SEQS) + ["blstop"])
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0, 1, 2, 3, 4])
def test_modes_01234_identical_can_to_menu_before_mode5(monkeypatch, menu4_cc, mode, fingerprint, seq):
  run = SEQS.get(seq, bl_stop_sequence)
  ref = run(rig(monkeypatch, mode, cp=make_cp(fingerprint=fingerprint), module=menu4_cc))
  monkeypatch.undo()
  new = run(rig(monkeypatch, mode, cp=make_cp(fingerprint=fingerprint)))
  assert len(new.can) > 400 and ref.can == new.can
  assert ref.scc14 == new.scc14


def test_stored_3_is_new_mode_3(monkeypatch):
  r = rig(monkeypatch, 3, stale_taper=True)
  mixed_sequence(r)
  assert isinstance(r.cc.k9_creep, K9Creep) and r.cc.k9_creep.mode == 3 and r.cc.k9_creep.cb_restore
  assert not r.cc.k9_creep.low_hold and "K9StopTaper" not in r.params.reads
  assert not hasattr(r.cc, "k9_stop_taper") and "StopTaper" not in (ROOT / CC_REL).read_text(encoding="utf-8")


@pytest.mark.parametrize("mode", ALL)
def test_no_camera_scc_k9_identical_to_previous_menu(monkeypatch, prev_cc, mode):
  flags = int(K9_FLAGS & ~HyundaiFlags.CAMERA_SCC)
  ref = mixed_sequence(rig(monkeypatch, 0, cp=make_cp(flags=flags), module=prev_cc))
  monkeypatch.undo()
  r = rig(monkeypatch, mode, stale_taper=True, cp=make_cp(flags=flags))
  assert mixed_sequence(r).can == ref.can and r.cc.k9_creep is None and r.cc.k9_creep_mode == 0


@pytest.mark.parametrize("mode", ALL)
def test_canfd_k9_guard_false(monkeypatch, mode):
  r = rig(monkeypatch, mode, stale_taper=True, cp=make_cp(flags=int(K9_FLAGS | HyundaiFlags.CANFD)))
  assert r.cc.k9_creep_applies is False and r.cc.k9_creep is None


def test_canfd_jerk_branch_ignores_creep_flag():
  def run(creep, obj=None):
    j = carcontroller.HyundaiJerk()
    cp = make_cp(flags=int(K9_FLAGS | HyundaiFlags.CANFD))
    cs = k9_rig.make_ci_state()
    cs.out = structs.CarState(vEgo=0.5, aEgo=-0.3)
    act = structs.CarControl().actuators
    act.longControlState, act.accel = LCS.stopping, -0.6
    j.make_jerk(cp, cs, -0.6, act, structs.CarControl().hudControl, creep=creep, creep_obj=obj)
    return j.jerk_u, j.jerk_l, j.cb_upper, j.cb_lower
  assert run(True) == run(False) == run(True, K9Creep(V9_C2)) == run(True, K9Creep(V9_C2C3)) == run(True, K9Creep(V9_BL))


class FakeCp:
  def __init__(self, vl):
    self.vl = vl


def patch_state(monkeypatch, **attrs):
  orig = k9_rig.make_ci_state

  def make():
    cs = orig()
    for k, v in attrs.items():
      setattr(cs, k, v)
    return cs
  monkeypatch.setattr(k9_rig, "make_ci_state", make)


@pytest.mark.parametrize("mode", ALL)
def test_soft_hold_mode2_keeps_stopreq(monkeypatch, prev_cc, mode):
  rows = []
  patch_state(monkeypatch, softHoldActive=1)
  r = rig(monkeypatch, mode)
  for _ in range(100):
    rows.append(r.step(-0.6, LCS.stopping, 0.3, a_target=-0.4, a_ego=-0.3))
  assert all(c is None or c["StopReq"] == 1 for _, c in rows)


@pytest.mark.parametrize("mode", ALL)
def test_tcs13_standstill_latches_stopreq(monkeypatch, mode):
  patch_state(monkeypatch, cp=FakeCp({"TCS13": {"StandStill": 1}, "ESP12": {"LONG_ACCEL": -0.3}}))
  r = rig(monkeypatch, mode)
  rows = [r.step(-0.6, LCS.stopping, 0.4, a_target=-0.4, a_ego=-0.3) for _ in range(40)]
  assert all(c is None or (c["StopReq"] == 1 and c["aReqValue"] <= -0.5 + 0.006) for _, c in rows)


# --- 3/4 versus v9 through the CarController -------------------------------------------------------------------

def bl_stop_sequence(rig, seed="k9-bl-stop"):
  """Stops whose stopping state starts at 1.0 m/s (so 0.8 -> 0.6 m/s is crossed before the latch): original accel
  stronger than v9 (-0.9), weaker than v9 (-0.25) and noisy around it, then standstill and restart."""
  rng = random.Random(seed)
  for orig in (-0.9, -0.25, None):
    for i in range(120):
      rig.step(-0.5, LCS.pid, max(4.0 - 0.025 * i, 1.05), a_target=-0.5, a_ego=-0.5 + rng.uniform(-0.05, 0.05))
    for i in range(190):
      v = max(1.0 - 0.005 * i, 0.06)
      a = rng.uniform(-0.8, -0.3) if orig is None else orig
      rig.step(a, LCS.stopping, v, a_target=-0.4, a_ego=-0.3 + rng.uniform(-0.05, 0.05))
    for _ in range(80):
      rig.step(-0.6, LCS.stopping, 0.0, standstill=True, a_target=-0.4, a_ego=0.0)
    for i in range(40):
      rig.step(0.8, LCS.pid, 0.05 * i, standstill=i == 0)
  return rig


# --- 3/4 versus v9 through the CarController -------------------------------------------------------------------

def _split(r):
  """(SCC12+others bytes per frame, SCC14 bytes per frame)"""
  other = [[m for m in f if m[0] != 905] for f in r.can]
  scc14 = [[m for m in f if m[0] == 905] for f in r.can]
  return other, scc14


@pytest.mark.parametrize("seq", list(SEQS))
def test_mode3_changes_only_comfortband_versus_v9(monkeypatch, seq):
  v9 = SEQS[seq](rig(monkeypatch, 2))
  monkeypatch.undo()
  m3 = SEQS[seq](rig(monkeypatch, 3))
  o9, s9 = _split(v9)
  o3, s3 = _split(m3)
  assert o9 == o3  # SCC12 (aReq, StopReq) and every other message identical
  differ, last_low = 0, -10**6
  for (f9, v, d9), (f3, _, d3) in zip(v9.scc14, m3.scc14, strict=True):
    assert f9 == f3
    if v < kc.CB_OFF_SPEED:
      last_low = f9
    for key in d9:
      if key not in ("ComfortBandUpper", "ComfortBandLower"):
        assert d9[key] == d3[key], key
    if (d9["ComfortBandUpper"], d9["ComfortBandLower"]) != (d3["ComfortBandUpper"], d3["ComfortBandLower"]):
      differ += 1
      assert v < kc.CB_OFF_SPEED or f9 - last_low <= 52  # above 3 m/s only while ramping back to v9 (0.5 s)
  assert differ > 0 or seq == "edge"


@pytest.mark.parametrize("seq", list(SEQS))
def test_mode4_stopreq_like_v9_accel_equal_or_stronger(monkeypatch, seq):
  rows = {}
  for mode in (2, 3, 4):
    r = rig(monkeypatch, mode)
    out = []
    orig_step = r.step

    def step(*a, **k):
      res = orig_step(*a, **k)
      out.append((a[2] if len(a) > 2 else k["v"], res))
      return res
    r.step = step
    SEQS[seq](r)
    rows[mode] = out
    monkeypatch.undo()
  for (v, (a2, s2)), (_, (a3, s3)), (_, (a4, s4)) in zip(rows[2], rows[3], rows[4], strict=True):
    assert a3 == a2
    assert a4 <= a2 + 1e-9  # never weaker than v9
    if s2 is not None:
      assert s4["StopReq"] == s2["StopReq"] == s3["StopReq"]  # StopReq timing unchanged
    if a4 != a2:
      assert v <= kc.C3_HOLD_SPEED


def band_trace(monkeypatch, mode, profile):
  r = rig(monkeypatch, mode)
  out = []
  for state, v in profile:
    r.step(-0.6, state, v, a_target=-0.4, a_ego=-0.3)
    j, c = r.cc.hyundai_jerk, r.cc.k9_creep
    out.append((v, float(j.cb_upper), float(j.cb_lower), None if c is None or c.cb_w is None else c.cb_w))
  return out


DECEL = ([(LCS.pid, 5.0)] * 100 + [(LCS.pid, 5.0 - 0.02 * i) for i in range(200)] +
         [(LCS.stopping, max(1.0 - 0.006 * i, 0.06)) for i in range(150)] + [(LCS.off, 0.0)] * 30)


@pytest.mark.parametrize("mode", [3, 4])
def test_c2_comfortband_values_and_ramp(monkeypatch, mode):
  tr = band_trace(monkeypatch, mode, DECEL)
  ws = [w for _, _, _, w in tr[:450]]
  assert all(w is not None for w in ws[2:])
  steps = np.abs(np.diff([w for w in ws[2:]]))
  assert np.all(steps <= kc.CB_RAMP_STEP + 1e-9)  # no jump (per make_jerk call; it runs every 2nd frame)
  for i, (v, cu, cl, w) in enumerate(tr[:450]):
    if w is None:
      continue
    if i >= 60 and i < 100:  # steady 5 m/s: v9 band (0)
      assert w == 0.0 and cu == 0.0 and cl == 0.0
    if v <= kc.CB_FULL_SPEED and all(x[0] <= kc.CB_FULL_SPEED for x in tr[max(0, i - 50):i]):
      assert w == 1.0 and cu > 0.0 and cu - cl == pytest.approx(0.1, abs=1e-6)  # original band, upper-lower 0.1
  v9 = band_trace(monkeypatch, 2, DECEL)
  assert all(cu == 0.0 and cl == 0.0 for _, cu, cl, _ in v9[2:450])  # v9: 0 everywhere while applied
  assert tr[-1][1:3] == v9[-1][1:3]  # control off: original (0) in both


def test_live_mode_switch_within_half_second_and_band_does_not_jump(monkeypatch):
  r = rig(monkeypatch, 2)
  for _ in range(60):
    r.step(-0.5, LCS.pid, 3.5, a_target=-0.5)
  for _ in range(60):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
  assert r.cc.k9_creep.mode == 2 and r.cc.hyundai_jerk.cb_upper == 0.0
  r.params.values["K9CreepMode"] = 3
  seen, bands = None, []
  for i in range(80):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
    bands.append(float(r.cc.hyundai_jerk.cb_upper))
    if seen is None and r.cc.k9_creep.mode == 3:
      seen = i
  assert seen is not None and seen <= 50  # read every 0.5 s
  assert np.all(np.abs(np.diff(bands)) <= kc.CB_RAMP_STEP * 0.8 + 0.01)  # ramp, no jump to the original band
  assert bands[-1] > 0.0
  r.params.values["K9CreepMode"] = float("nan")
  for i in range(51):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
  assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0
  r.params.values["K9CreepMode"] = 4
  for i in range(51):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
  assert r.cc.k9_creep.mode == 4 and r.cc.k9_creep.low_hold


def test_live_mode_change_resets_profiles(monkeypatch):
  r = rig(monkeypatch, 2)
  for _ in range(60):
    r.step(-1.0, LCS.pid, 3.0)
  for i in range(60):
    r.step(-1.2, LCS.stopping, max(0.5 - 0.005 * i, 0.1))
  assert r.cc.k9_creep is not None and r.cc.k9_creep.mode == 2 and r.cc.k9_creep.last is not None
  r.params.values["K9CreepMode"] = 1
  for _ in range(60):
    r.step(-1.2, LCS.stopping, 0.1)
  assert r.cc.k9_creep.mode == 1
  r.params.values["K9CreepMode"] = 0
  out = [r.step(-1.2, LCS.stopping, 0.1) for _ in range(60)]
  assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0
  sent = [o for o in out[-10:] if o[1] is not None]
  assert sent and all(o[1]["StopReq"] == 1 and o[0] == pytest.approx(-1.2) for o in sent)


def test_setting_registered_single_selector_and_taper_deleted():
  import json
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  entry = next(p for p in catalog["params"] if p["name"] == "K9CreepMode")
  assert (entry["min"], entry["max"], entry["default"], entry["unit"]) == (0, 6, 0, 1)
  assert not any(p["name"] in ("K9CreepV9", "K9CreepV8", "K9StopTaper") for p in catalog["params"])
  keys = (ROOT / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
  assert '{"K9CreepMode", {PERSISTENT, INT, "0"}},' in keys and "K9CreepV9" not in keys and "K9StopTaper" not in keys
  assert not (ROOT / "opendbc_repo/opendbc/car/hyundai/k9_stop_taper.py").exists()


# --- 5 (BL) ------------------------------------------------------------------------------------------------------

def test_bl_weight_unit():
  assert kc.bl_weight(0.0) == 1.0 and kc.bl_weight(0.6) == 1.0 and kc.bl_weight(0.7) == pytest.approx(0.5)
  assert kc.bl_weight(0.8) == 0.0 and kc.bl_weight(5.0) == 0.0 and kc.bl_weight(float("nan")) == 1.0
  c = K9Creep(V9_BL)
  assert c.bands(0.5, 1.0, 0.8, 0.7, 0.0) == K9Creep.jerk_bands(0.5, 1.0)  # >= 0.8 m/s: v9 band (0)
  ju, cu, cl = c.bands(0.5, 0.7, 0.8, 0.7, 0.0)
  assert ju == K9Creep.jerk_bands(0.5, 0.7)[0] and (cu, cl) == pytest.approx((0.4, 0.35))
  ju, cu, cl = c.bands(0.5, 0.3, 0.8, 0.7, 0.0)
  assert ju == K9Creep.jerk_bands(0.5, 0.3)[0] and (cu, cl) == (0.8, 0.7)  # <= 0.6 m/s: original band, no ramp
  assert c.cb_w is None  # C2 ramp state is not used by 5


@pytest.mark.parametrize("orig", [-1.2, -0.9, -0.6, -0.45, -0.3, -0.1, 0.1])
@pytest.mark.parametrize("margin_kw", [{}, {"long_accel": -0.05, "a_ego": -0.1}, {"long_accel": -0.6, "a_ego": -0.6},
                                       {"long_accel": None, "a_ego": -0.2}])
def test_bl_accel_never_weaker_than_v9_and_feather_state_is_v9(orig, margin_kw):
  speeds = list(np.linspace(1.0, 0.06, 120))
  c9, c5 = K9Creep(V9), K9Creep(V9_BL)
  differ = 0
  for v in speeds:
    a9, s9 = upd(c9, accel=orig, v_ego=v, v_ego_raw=v, **margin_kw)
    a5, s5 = upd(c5, accel=orig, v_ego=v, v_ego_raw=v, **margin_kw)
    assert s5 == s9 is False  # StopReq withheld exactly like v9
    assert c5.last == c9.last  # the v9 feather state is not changed by the blend
    w = kc.bl_weight(v)
    assert a5 == pytest.approx(min(a9, (1.0 - w) * a9 + w * orig), abs=1e-12)
    assert a5 <= a9 + 1e-12  # never less braking than v9
    if v >= kc.BL_OFF_SPEED:
      assert a5 == a9
    if v <= kc.BL_FULL_SPEED:
      assert a5 == pytest.approx(min(a9, orig), abs=1e-12)
    differ += a5 != a9
  assert (differ > 0) == (orig < -0.31)  # weaker original (above the v9 feather values): exactly v9


def test_bl_pid_and_latch_are_v9():
  for kw in ({"stopping": False, "pid": True, "v_ego": 0.5, "v_ego_raw": 0.5, "a_target": -0.3},
             {"stopping": False, "pid": True, "v_ego": 0.3, "v_ego_raw": 0.3, "accel": -0.2, "a_target": -0.3},
             {"esp_standstill": True}, {"a_target": -1.0}, {"a_ego": 0.07}, {"lead_distance": 2.5}):
    c9, c5 = K9Creep(V9), K9Creep(V9_BL)
    for _ in range(3):
      assert upd(c5, **kw) == upd(c9, **kw)


@pytest.mark.parametrize("seq", list(SEQS) + ["blstop"])
def test_mode5_versus_v9_through_carcontroller(monkeypatch, seq):
  run = SEQS.get(seq, bl_stop_sequence)
  rows = {}
  for mode in (2, 5):
    r = rig(monkeypatch, mode)
    out = []
    orig_step = r.step

    def step(*a, _out=out, _orig=orig_step, **k):
      res = _orig(*a, **k)
      _out.append((a[2] if len(a) > 2 else k["v"], res))
      return res
    r.step = step
    run(r)
    rows[mode] = (out, r)
    monkeypatch.undo()
  (o2, r2), (o5, r5) = rows[2], rows[5]
  stronger = 0
  for (v, (a2, s2)), (_, (a5, s5)) in zip(o2, o5, strict=True):
    assert a5 <= a2 + 1e-9  # never weaker than v9
    if s2 is not None:
      assert s5["StopReq"] == s2["StopReq"]  # StopReq timing unchanged
      assert s5["aReqValue"] <= s2["aReqValue"] + 1e-9
    if a5 != a2:
      assert v < kc.BL_OFF_SPEED
      stronger += 1
  for f, ((v, _), c2, c5) in enumerate(zip(o2, r2.can, r5.can, strict=True)):
    if v >= kc.BL_OFF_SPEED and all(x[0] >= kc.BL_OFF_SPEED for x in o2[max(0, f - 100):f]):
      assert c2 == c5, f  # well above 0.8 m/s (1 s after leaving the BL range): every message identical to v9
  band_differ = 0
  for (f2, v, d2), (f5, _, d5) in zip(r2.scc14, r5.scc14, strict=True):
    assert f2 == f5
    for key in d2:
      if key not in ("ComfortBandUpper", "ComfortBandLower"):
        assert d2[key] == d5[key], key
    if (d2["ComfortBandUpper"], d2["ComfortBandLower"]) != (d5["ComfortBandUpper"], d5["ComfortBandLower"]):
      assert v < kc.BL_OFF_SPEED
      band_differ += 1
  assert band_differ > 0 and (stronger > 0 or seq in ("mixed", "edge"))


def test_mode5_comfortband_follows_speed_weight(monkeypatch):
  profile = ([(LCS.pid, 3.0)] * 60 + [(LCS.stopping, max(1.0 - 0.004 * i, 0.06)) for i in range(250)] +
             [(LCS.off, 0.0)] * 20)
  tr = band_trace(monkeypatch, 5, profile)
  nonzero = 0
  for (vp, _, _, _), (v, cu, cl, _) in zip(tr[1:309], tr[2:310], strict=True):
    # make_jerk runs every 2nd frame, so the band on a frame is from this frames or the previous frames speed
    ws = [kc.bl_weight(x) for x in (v, vp)]
    assert any((cu, cl) == (0.0, 0.0) if w == 0.0 else
               (cu > 0.0 and cl > 0.0 and cu - cl == pytest.approx(0.1 * w, abs=1e-6)) for w in ws), (v, cu, cl)
    nonzero += cu > 0.0
  assert nonzero > 100
  v9 = band_trace(monkeypatch, 2, profile)
  assert tr[-1][1:3] == v9[-1][1:3]  # control off: original (0) in both


def test_live_mode_switch_to_5_and_out(monkeypatch):
  r = rig(monkeypatch, 4)
  for _ in range(60):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
  r.params.values["K9CreepMode"] = 5
  seen = None
  for i in range(60):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
    if seen is None and r.cc.k9_creep.mode == 5:
      seen = i
  assert seen is not None and seen <= 50 and r.cc.k9_creep.low_blend and not r.cc.k9_creep.low_hold
  for value in (7, float("nan"), -1):
    r.params.values["K9CreepMode"] = 5
    for _ in range(51):
      r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
    assert r.cc.k9_creep.mode == 5
    r.params.values["K9CreepMode"] = value
    for _ in range(51):
      r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
    assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0


def test_stored_5_is_mode_5(monkeypatch):
  r = rig(monkeypatch, 5, stale_taper=True)
  mixed_sequence(r)
  assert isinstance(r.cc.k9_creep, K9Creep) and r.cc.k9_creep.mode == 5 and r.cc.k9_creep.low_blend
  assert not r.cc.k9_creep.cb_restore and not r.cc.k9_creep.low_hold and "K9StopTaper" not in r.params.reads


# --- 6 (C2 band + braking compensation) ------------------------------------------------------------------------

def test_mode6_band_comp_unit():
  c = K9Creep(V9_C2BL)
  comp = kc.C2BL_COMP
  assert c._band_comp(-0.5, -0.5, 1.0) == pytest.approx(comp) and c._band_comp(-0.5, -0.5, 2.0) == pytest.approx(comp)
  assert c._band_comp(-0.5, -0.5, 2.5) == pytest.approx(comp * 0.5) and c._band_comp(-0.5, -0.5, 3.0) == 0.0
  assert c._band_comp(-0.5, -0.5, 0.6) == pytest.approx(comp) and c._band_comp(-0.5, -0.5, 0.45) == pytest.approx(comp / 2)
  assert c._band_comp(-0.5, -0.5, 0.3) == 0.0 and c._band_comp(-0.5, -0.5, 0.1) == 0.0
  assert c._band_comp(0.0, -0.5, 1.0) == 0.0 and c._band_comp(0.2, -0.5, 1.0) == 0.0  # not braking: no extra braking
  assert c._band_comp(-0.5, -0.05, 1.0) == 0.0 and c._band_comp(-0.5, -0.125, 1.0) == pytest.approx(comp / 2)
  assert c._band_comp(-0.5, -0.5, float("nan")) == pytest.approx(comp)  # non-finite speed: toward the original band
  for m in (V8, V9, V9_C2, V9_C2C3, V9_BL):
    assert K9Creep(m)._band_comp(-0.5, -0.5, 1.0) == 0.0
  # band: exactly the mode-3 (C2) band, not the mode-5 band
  c3, c6 = K9Creep(V9_C2), K9Creep(V9_C2BL)
  for v in (3.5, 2.6, 1.5, 0.7, 0.3):
    assert c6.bands(0.5, v, 0.8, 0.7, 0.0) == c3.bands(0.5, v, 0.8, 0.7, 0.0)


def _comp(v, a_target, accel):
  return K9Creep(V9_C2BL)._band_comp(accel, a_target, v)


@pytest.mark.parametrize("orig", [-1.2, -0.9, -0.6, -0.45, -0.3, -0.1, 0.1])
@pytest.mark.parametrize("margin_kw", [{}, {"long_accel": -0.05, "a_ego": -0.1}, {"long_accel": -0.6, "a_ego": -0.6},
                                       {"long_accel": None, "a_ego": -0.2}])
def test_mode6_never_weaker_than_v9_stopreq_and_state_are_v9(orig, margin_kw):
  # pid approach 3.5 -> 1.0 m/s, then stopping 1.0 -> 0.06 m/s
  c9, c6 = K9Creep(V9), K9Creep(V9_C2BL)
  for v in np.linspace(3.5, 1.0, 120):
    kw = dict(accel=orig, stopping=False, pid=True, v_ego=float(v), v_ego_raw=float(v), a_target=-0.5, **margin_kw)
    a9, s9 = upd(c9, **kw)
    a6, s6 = upd(c6, **kw)
    assert s6 == s9 is False and c6.last == c9.last
    assert a6 == pytest.approx(max(a9 - _comp(v, -0.5, a9), kc.CarControllerParams.ACCEL_MIN), abs=1e-12) and a6 <= a9
  differ = 0
  for v in np.linspace(1.0, 0.06, 120):
    a9, s9 = upd(c9, accel=orig, v_ego=float(v), v_ego_raw=float(v), **margin_kw)
    a6, s6 = upd(c6, accel=orig, v_ego=float(v), v_ego_raw=float(v), **margin_kw)
    assert s6 == s9 is False  # StopReq withheld exactly like v9
    assert c6.last == c9.last  # the v9 feather state is not changed
    w = kc.bl_weight(v) * kc.C2BL_GAIN
    blend = min(a9, (1.0 - w) * a9 + w * orig)
    assert a6 == pytest.approx(blend - _comp(v, -0.5, blend), abs=1e-12)
    assert a6 <= a9 + 1e-12  # never less braking than v9
    differ += a6 != a9
  assert differ > 0


def test_mode6_latches_like_v9():
  for kw in ({"esp_standstill": True}, {"a_target": -1.0}, {"a_ego": 0.07}, {"lead_distance": 2.5},
             {"v_ego": 0.04, "v_ego_raw": 0.04}, {"long_accel": 0.2, "a_ego": 0.0}):
    c9, c6 = K9Creep(V9), K9Creep(V9_C2BL)
    for _ in range(3):
      r9, r6 = upd(c9, **kw), upd(c6, **kw)
      assert r6 == r9 and r6[1] is True
  for kw in ({"gas": True}, {"brake": True}, {"override": True}, {"long_active": False}, {"can_valid": False}):
    assert upd(K9Creep(V9_C2BL), **kw) == (-1.0, True)  # interlock: the original command at once


@pytest.mark.parametrize("seq", list(SEQS) + ["blstop"])
def test_mode6_versus_v9_and_3_through_carcontroller(monkeypatch, seq):
  run = SEQS.get(seq, bl_stop_sequence)
  rows = {}
  for mode in (2, 3, 6):
    r = rig(monkeypatch, mode)
    out = []
    orig_step = r.step

    def step(*a, _out=out, _orig=orig_step, **k):
      res = _orig(*a, **k)
      _out.append((a[2] if len(a) > 2 else k["v"], res))
      return res
    r.step = step
    run(r)
    rows[mode] = (out, r)
    monkeypatch.undo()
  (o2, r2), (_, r3), (o6, r6) = rows[2], rows[3], rows[6]
  stronger = 0
  for (v, (a2, s2)), (_, (a6, s6)) in zip(o2, o6, strict=True):
    assert a6 <= a2 + 1e-9  # never weaker than v9
    if s2 is not None:
      assert s6["StopReq"] == s2["StopReq"]  # StopReq timing unchanged
      assert s6["aReqValue"] <= s2["aReqValue"] + 1e-9
    if a6 != a2:
      assert v < kc.CB_OFF_SPEED
      stronger += 1
  for f, ((v, _), c2, c6) in enumerate(zip(o2, r2.can, r6.can, strict=True)):
    if v >= kc.CB_OFF_SPEED and all(x[0] >= kc.CB_OFF_SPEED for x in o2[max(0, f - 100):f]):
      assert c2 == c6, f  # 1 s above 3 m/s: every message identical to v9
  for (f3, _, d3), (f6, _, d6), (f2, _, d2) in zip(r3.scc14, r6.scc14, r2.scc14, strict=True):
    assert f3 == f6 == f2
    for key in d2:
      if key not in ("ComfortBandUpper", "ComfortBandLower"):
        assert d6[key] == d2[key], key
    assert (d6["ComfortBandUpper"] == 0.0) == (d3["ComfortBandUpper"] == 0.0)  # the band follows mode 3's weight
  assert stronger > 0 or seq == "edge"


def test_live_mode_switch_to_6_and_out(monkeypatch):
  r = rig(monkeypatch, 5)
  for _ in range(60):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
  r.params.values["K9CreepMode"] = 6
  seen = None
  for i in range(60):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
    if seen is None and r.cc.k9_creep.mode == 6:
      seen = i
  assert seen is not None and seen <= 50
  assert r.cc.k9_creep.cb_restore and r.cc.k9_creep.low_blend and not r.cc.k9_creep.bl_band and not r.cc.k9_creep.low_hold
  for value in (7, float("nan"), -1):
    r.params.values["K9CreepMode"] = 6
    for _ in range(51):
      r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
    assert r.cc.k9_creep.mode == 6
    r.params.values["K9CreepMode"] = value
    for _ in range(51):
      r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
    assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0


def test_stored_6_is_mode_6(monkeypatch):
  r = rig(monkeypatch, 6, stale_taper=True)
  mixed_sequence(r)
  assert isinstance(r.cc.k9_creep, K9Creep) and r.cc.k9_creep.mode == 6 and "K9StopTaper" not in r.params.reads
