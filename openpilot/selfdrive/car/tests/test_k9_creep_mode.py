"""K9StopMode single selector (2026-10-25 menu): 0 base braking, 1 v12 original (the creep_fix v12 page code unchanged), 2 v12k9
(kind V12K9); both are tested in test_k9_stop_v12.py. Other values act as 0.
2026-10-26 (보노보노 "v12그냥놓아두고 v13추가하는걸로"): 3 = v13 original (test_k9_stop_v13.py). Menu-set pins only: MODES (1, 2, 3),
INVALID and the live-switch invalid list without 3, settings max 3 / 4 options.
2026-10-28 (보노보노 01:06 "원본 그대로 푸쉬해라 v14"; 2026-10-27 19:48 / 19:51 C1 / C1+ / v13 removed): 3 = v14 original
(test_k9_stop_v14.py) instead of the v13 original. Menu-set pins only: V13_ORIG_MENU -> V14_ORIG_MENU (same value 3).
Pin updated 2026-10-25 evening (brake_v12k9_live_fix, menu 2 ComfortBand): test_live_mode_switch_within_half_second expects the
original band (0.9 + 0.2 * accel) after the switch to 2 at 1.0 m/s instead of 0.
Rewritten 2026-10-25 (보노보노 00:36 "브레이크 앞전꺼는 다 별로니 지워" / "v12 원본 ... v12k9"; v10, v10 + light shudder fix and v11
removed with their kinds 2, 6, 7 and their own test files test_k9_stop_v10*.py / test_k9_stop_v11.py / test_k9_stop_menu_trim.py):
menu-set pins only (MODES/MENU/KINDS/V12_ORIG_MENU, bad kinds, INVALID with 3, max 2 / 3 options); ALL_KINDS = (V9,) (kind 8
waits 12 standstill frames like the old kind 7: its TCS13 / soft-hold checks are in test_k9_stop_v12.py); the 1d0961da
comparison only for 0 (its 2 = v10 is deleted) and the kind-2 object comparison deleted (frozen pin of a deleted kind);
test_controller_never_writes_the_menu_value, the CAN-FD jerk check and the two live-switch tests use the new 1 / 2 (the
light-band ramp they pinned belonged to the deleted menu 2); the removed-names list gains the deleted kinds' names.
Rewritten 2026-10-24 evening (보노보노 "브레이크 2.3.4 제거", the old 2 v9 + shudder fix, 3 v10 + band 0.2 and 4 v10 + shudder
fix removed with their kinds 3, 4, 5): deleted the kind-3-only tests (band weight / blend / band comp / feather gate / pid
gate / latch hold / v9-vs-mode-2 / ComfortBand ramp), ALL = (V9,), ALL_KINDS without kinds 3-5, the 1d0961da comparison
only for the new 0/1 (its 3 was kind 3), INVALID with 4, 5, 6, test_stored_values_are_not_migrated ->
test_controller_never_writes_the_menu_value (new menu), live switch tests use menu 2 (v10 + light) instead of the old 2,
_state/cb_w/sent read with getattr (the attributes of the removed kinds are gone; always None for the kept kinds),
menu-set pins (MODES/MENU/KINDS, bad kinds, max 3 / 4 options, K9StopModeVer key). Bit identity of the new menu against
the frozen 407bedaa (0/1 = 0/1, 2 = 5, 3 = 6) and the stored-value migration: test_k9_stop_menu_trim.py.
Rewritten 2026-10-24 for menu 6 (menu-set pins only): INVALID without 6, test_constants_match_source_and_mode2 (MODES/MENU/
KINDS, the bad-kind list 0/8/9/-1), test_live_mode_switch_within_half_second_and_band_does_not_jump (invalid 7 instead of
6), test_setting_registered_single_selector_old_keys_deleted (max 6, 7 options). Added: menu 6 in the stored-value and
CAN-FD jerk checks. Kind 7 (v11) is NOT in ALL_KINDS: its StopReq waits 12 standstill frames by design, so the TCS13 and
soft-hold checks for it are in test_k9_stop_v11.py (the ALL_KINDS tests are unchanged for kinds 1-6).
Rewritten 2026-10-24 for menu 5 (menu-set pins only): INVALID without 5, test_constants_match_source_and_mode2 (MODES/MENU/
KINDS, the bad-kind list 0/7/8/-1), test_live_mode_switch_within_half_second_and_band_does_not_jump (invalid 6 instead of
5), test_setting_registered_single_selector_old_keys_deleted (max 5, 6 options). Added: menu 5 in the stored-value and
CAN-FD jerk checks (kind 6 also runs every ALL_KINDS test).
Rewritten 2026-10-24 for menu 4 (menu-set pins only): INVALID without 4, test_constants_match_source_and_mode2 (MODES/MENU/
KINDS), test_live_mode_switch_within_half_second_and_band_does_not_jump (invalid 5 instead of 4),
test_setting_registered_single_selector_old_keys_deleted (max 4, 5 options). Added: menu 4 in the stored-value and CAN-FD
jerk checks (kind 5 also runs every ALL_KINDS test). The bare v9 menu entry was removed; v9 stays as the K9Creep kind V9 (the
base of 2) and is driven through the CarController here by mapping menu 1 to it in the test only (rig_kind).
New 1 and 2 are checked byte-for-byte against the frozen 1d0961da controller's 2 and 3 (the previous menu).

K9StopMode replaced K9CreepMode (0-6) on 2026-10-22. 0 and 1 are checked byte-for-byte against the real frozen
aeef67c6 controller (the 0-6 menu): new 0 / unset / invalid = old 0, new 1 = old 2 (v9), and a stale K9CreepMode left
in Params is never read. Mode 3 (the former 2, renumbered 2026-10-22 23:16) was improved in place (2026-10-22 stop_ref: no re-press, hold at the K9 wheel stop,
K9 creep floor, no release at the latch) and is checked against v9 (StopReq identical, aReq never weaker).
Checks source logic and SCC12/SCC14 encoding, not K9 vehicle behaviour.
"""
import math
import random

import numpy as np
import pytest

from opendbc.car import structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V9, V12K9
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.selfdrive.car.tests import k9_rig
from openpilot.selfdrive.car.tests.k9_rig import (CC_REL, LCS, ROOT, Rig, edge_sequence, gentle_sequence, load_frozen_cc,
                                                  mixed_sequence)
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import K9_FLAGS, make_cp

OLD_MENU = "aeef67c6"  # K9CreepMode 0 base, 1 v8, 2 v9, 3/4/5 candidates, 6 v9 + shudder fix 4
OLD_OF_NEW = {None: 0, 0: 0}  # aeef67c6 K9CreepMode of the new menu value (v9 = old 2 is checked as the kind V9 below)
OLD_KIND = {V9: 2}  # aeef67c6 K9CreepMode 2 = v9 (the K9Creep kind V9 of this tree)
PREV_MENU = "1d0961da"  # K9StopMode 0 base, 1 v9, 2 v10, 3 v9 + shudder fix
PREV_OF_NEW = {None: None, 0: 0}  # 1d0961da K9StopMode of the new menu value (new 1 / 2 are new)
ALL = (V9,)
ALL_KINDS = (V9,)
INVALID = (4, 5, 6, 7, 8, 99, -1, -4, float("nan"), "nan", float("inf"))  # 3 = v13 original 2026-10-26, v14 original since 2026-10-28


def upd(c, **kw):
  args = dict(accel=-1.0, a_target=-0.5, stopping=True, pid=False, v_ego=0.3, v_ego_raw=0.3, a_ego=-0.3, long_accel=-0.3,
              esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True,
              lead_visible=False, lead_distance=0.0)
  args.update(kw)
  return c.update(**args)


@pytest.fixture(scope="module")
def old_cc(tmp_path_factory):
  return load_frozen_cc(OLD_MENU, tmp_path_factory.mktemp("old_cc"), "cc_aeef67c6")


def rig(monkeypatch, mode, cp=None, module=None, **extra):
  kw = {} if mode is None else {"K9StopMode": mode}
  return Rig(monkeypatch, cp=cp, **({"module": module} if module else {}), **kw, **extra)


def rig_kind(monkeypatch, kind, cp=None, **extra):
  """CarController with K9StopMode 1 mapped to the K9Creep kind `kind` (test only; v9 is no longer a menu value)."""
  monkeypatch.setattr(carcontroller, "K9_CREEP_MENU", {1: kind})
  return rig(monkeypatch, 1, cp=cp, **extra)


@pytest.fixture(scope="module")
def prev_cc(tmp_path_factory):
  return load_frozen_cc(PREV_MENU, tmp_path_factory.mktemp("prev_cc"), "cc_1d0961da")


def old_rig(monkeypatch, old_cc, old_mode, cp=None):
  return Rig(monkeypatch, cp=cp, module=old_cc, K9CreepMode=old_mode)


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


SEQS = {"mixed": mixed_sequence, "edge": edge_sequence, "gentle": gentle_sequence, "blstop": bl_stop_sequence}


# --- unit -----------------------------------------------------------------------------------------------------

def test_applies_only_k9_camera_scc_openpilot_long():
  assert kc.k9_creep_applies(make_cp())
  assert not kc.k9_creep_applies(make_cp(fingerprint=CAR.KIA_K7))
  assert not kc.k9_creep_applies(make_cp(flags=int(K9_FLAGS | HyundaiFlags.CANFD)))
  assert not kc.k9_creep_applies(make_cp(flags=int(K9_FLAGS & ~HyundaiFlags.CAMERA_SCC)))
  cp = make_cp()
  cp.openpilotLongitudinalControl = False
  assert not kc.k9_creep_applies(cp)


def test_constants_match_source_and_mode2():
  assert (kc.FEATHER_BP, kc.FEATHER_V) == ([0.05, 0.14, 0.25, 0.42], [-0.31, -0.35, -0.42, -0.52])
  assert (kc.MIN_DECEL_BP, kc.MIN_DECEL_V) == ([0.08, 0.14, 0.25, 0.40], [-0.05, -0.12, -0.20, -0.30])
  assert (kc.CREEP_BP, kc.CREEP_V) == ([0.25, 0.55, 1.0, 1.6], [-0.26, -0.26, -0.20, 0.0])
  assert kc.DOWN_EFF_AC == -0.02
  assert kc._up_step(0.05, 0.0) == 0.0032 and kc._up_step(0.2, 0.02) == 0.0 and kc._up_step(0.2, -0.08) == 0.0032
  assert kc.MODES == (1, 2, 3) and (V9, V12K9) == (1, 8) and kc.V12_ORIG_MENU == 1 and kc.V14_ORIG_MENU == 3  # V13_ORIG_MENU until 2026-10-28
  assert not hasattr(kc, "V13_ORIG_MENU")
  assert kc.MENU == {2: V12K9} and kc.KINDS == ALL_KINDS + (V12K9,)
  assert not any(hasattr(K9Creep(m), "shudder_fix") for m in kc.KINDS)
  for bad in (0, 2, 3, 4, 5, 6, 7, 9, -1):
    with pytest.raises(AssertionError):
      K9Creep(bad)


def test_removed_modes_are_gone_from_source():
  src = (ROOT / "opendbc_repo/opendbc/car/hyundai/k9_creep.py").read_text(encoding="utf-8")
  for word in ("V8", "_up_v8", "PROFILES", "C3_", "V9_C2", "V9_BL", "low_hold", "bl_band", "C2BL", "K9CreepMode = ",
               "V9_SHUDDER_FIX", "V10_BAND02", "V10_SMOOTH", "shudder_fix", "_band_comp", "band_weight", "blend_weight",
               "smooth_band_target", "_band_toward", "LEAD_LATCH_GAP_2", "V10_BAND", "V10_LIGHT", "LIGHT_MIN_DECEL",
               "light_band_target_units", "_light_toward", "V10_HOLD_STEP", "V10_FEATHER", "_update_v11", "V11_",
               "v10_light", "self.v11"):
    assert word not in src, word
  cc_src = (ROOT / CC_REL).read_text(encoding="utf-8")
  assert 'params.get_int("K9StopMode")' in cc_src and "K9CreepMode" not in cc_src


@pytest.mark.parametrize("mode", ALL)
def test_withholds_stopreq_then_latches_immediately(mode):
  c = K9Creep(mode)
  a, s = upd(c)
  assert s is False and a > -1.0
  for kw in ({"esp_standstill": True}, {"v_ego_raw": 0.01}, {"v_ego": 0.04}, {"a_target": -1.0}, {"a_ego": 0.07},
             {"lead_visible": True, "lead_distance": 2.0}, {"lead_distance": 2.5}, {"long_accel": 0.2, "a_ego": 0.0},
             {"v_ego": 0.12, "v_ego_raw": 0.12, "a_target": -1.0}):
    c, ref = K9Creep(mode), K9Creep(V9)
    upd(c)
    upd(ref)
    a, s = upd(c, **kw)
    assert s is True and a <= -0.5  # same frame
    assert (a, s) == upd(ref, **kw)  # 2 latches exactly like v9


@pytest.mark.parametrize("mode", ALL)
@pytest.mark.parametrize("kw", [{"gas": True}, {"brake": True}, {"override": True}, {"long_active": False},
                                {"can_valid": False}, {"v_ego": float("nan")}, {"accel": float("nan")}])
def test_interlocks_return_original(mode, kw):
  c = K9Creep(mode)
  upd(c)
  orig = (kw.get("accel", -1.0), True)
  out = upd(c, **kw)
  assert (out[0] == orig[0] or (math.isnan(out[0]) and math.isnan(orig[0]))) and out[1] is True and not c.active
  assert c.last is None


def _state(c):
  return (c.latched, c.last, c.active, getattr(c, "cb_w", None))


def _random_inputs(rng, n):
  v = 3.5
  for i in range(n):
    if rng.random() < 0.01:
      v = rng.uniform(0.0, 4.0)
    v = max(0.0, v - rng.uniform(0.0, 0.02))
    acc = rng.choice([rng.uniform(-2.0, 0.5), -0.5, -1.0, -0.31, 0.0])
    yield {"accel": acc, "a_target": rng.choice([acc, rng.uniform(-1.2, 0.2), -0.05, -0.2, -1.0]),
           "stopping": v < 1.0 and rng.random() < 0.9, "pid": rng.random() < 0.8, "v_ego": v,
           "v_ego_raw": v if rng.random() < 0.95 else rng.choice([0.0, 0.04, float("nan")]),
           "a_ego": rng.uniform(-1.2, 0.1), "long_accel": rng.choice([None, float("nan"), rng.uniform(-1.3, 0.2)]),
           "esp_standstill": rng.random() < 0.02, "long_active": rng.random() < 0.99,
           "override": rng.random() < 0.005, "gas": rng.random() < 0.005, "brake": rng.random() < 0.005,
           "can_valid": rng.random() < 0.998, "lead_visible": rng.random() < 0.3,
           "lead_distance": rng.choice([0.0, rng.uniform(0.0, 12.0), 3.0])}, i


@pytest.mark.parametrize("seed", range(6))
@pytest.mark.parametrize("new_mode", (V9,))
def test_k9creep_object_identical_to_old_menu(old_cc, new_mode, seed):
  rng = random.Random(f"k9-stopmode-{seed}")
  old, new = old_cc.K9Creep(OLD_KIND[new_mode]), K9Creep(new_mode)
  cb_prev_o = cb_prev_n = 0.0
  for kw, i in _random_inputs(rng, 4000):
    ro, rn = old.update(**kw), new.update(**kw)
    assert ro == rn or (math.isnan(ro[0]) and math.isnan(rn[0]) and ro[1] == rn[1]), (i, kw)
    assert _state(old) == _state(new) or (math.isnan(old.last or 0.0) and math.isnan(new.last or 0.0))
    if i % 2 == 0 and old.active:
      cu, cl = float(np.clip(0.9 + rn[0] * 0.2, 0, 1.2)), float(np.clip(0.8 + rn[0] * 0.2, 0, 1.2))
      ju = rng.uniform(0.5, 5.0)
      bo = old.bands(ju, kw["v_ego"], cu, cl, cb_prev_o)
      bn = new.bands(ju, kw["v_ego"], cu, cl, cb_prev_n)
      assert bo == bn, i
      cb_prev_o, cb_prev_n = bo[1], bn[1]
      assert getattr(old, "cb_w", None) == getattr(new, "cb_w", None)


@pytest.mark.parametrize("new_mode", (V9,))
def test_k9creep_object_identical_to_old_menu_on_grids(old_cc, new_mode):
  for orig in (-1.2, -0.9, -0.6, -0.45, -0.3, -0.1, 0.1):
    for margin_kw in ({}, {"long_accel": -0.05, "a_ego": -0.1}, {"long_accel": -0.6, "a_ego": -0.6},
                      {"long_accel": None, "a_ego": -0.2}, {"long_accel": 0.02, "a_ego": -0.05}):
      old, new = old_cc.K9Creep(OLD_KIND[new_mode]), K9Creep(new_mode)
      for v in list(np.linspace(3.5, 1.0, 60)) + list(np.linspace(1.0, 0.0, 120)):
        kw = dict(accel=orig, v_ego=float(v), v_ego_raw=float(v), **margin_kw)
        if v > 1.0:
          kw.update(stopping=False, pid=True)
        assert upd(old, **kw) == upd(new, **kw) and _state(old) == _state(new)
        for v_b, prev in ((float(v), 0.0), (float(v), 0.8), (float(v) + 1.5, 0.3)):
          assert old.bands(0.5, v_b, 0.8, 0.7, prev) == new.bands(0.5, v_b, 0.8, 0.7, prev)


# --- CarController: byte-identical to the frozen aeef67c6 controller ------------------------------------------------

@pytest.mark.parametrize("seq", list(SEQS))
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0])
def test_new_modes_identical_can_to_old_menu(monkeypatch, old_cc, mode, fingerprint, seq):
  ref = SEQS[seq](old_rig(monkeypatch, old_cc, OLD_OF_NEW[mode], cp=make_cp(fingerprint=fingerprint)))
  monkeypatch.undo()
  new = SEQS[seq](rig(monkeypatch, mode, cp=make_cp(fingerprint=fingerprint)))
  assert len(new.can) > 400 and ref.can == new.can
  assert ref.scc14 == new.scc14
  assert "K9CreepMode" not in new.params.reads and "K9StopTaper" not in new.params.reads
  if fingerprint == CAR.KIA_K9:
    assert "K9StopMode" in new.params.reads
    assert (new.cc.k9_creep is None) == (mode in (None, 0))
    assert new.cc.k9_creep is None


# --- CarController: new 1 byte-identical to the frozen 1d0961da controller's 2 (2026-10-23 07:25 renumbering) --------

@pytest.mark.parametrize("seq", list(SEQS))
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0])
def test_new_1_2_identical_can_to_1d0961da_2_3(monkeypatch, prev_cc, mode, fingerprint, seq):
  prev = PREV_OF_NEW[mode]
  ref = SEQS[seq](Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), module=prev_cc,
                      **({} if prev is None else {"K9StopMode": prev})))
  monkeypatch.undo()
  new = SEQS[seq](rig(monkeypatch, mode, cp=make_cp(fingerprint=fingerprint)))
  assert len(new.can) > 400 and ref.can == new.can
  assert ref.scc14 == new.scc14


def test_controller_never_writes_the_menu_value(monkeypatch):
  # rewritten 2026-10-24 evening (was test_stored_values_are_not_migrated): the CarController maps the stored value through
  # MENU and never writes it; the one-time stored-value migration runs in manager_init (test_k9_stop_menu_trim.py).
  for stored, kind in ((1, None), (2, V12K9)):
    r = rig(monkeypatch, stored)
    r.step(-0.5, LCS.pid, 3.0)
    assert r.cc.k9_creep_mode == stored and (r.cc.k9_creep is None if kind is None else r.cc.k9_creep.mode == kind)
    assert r.cc.k9_v12_orig is (stored == 1)
    assert r.params.values["K9StopMode"] == stored  # the controller never writes the menu value
    monkeypatch.undo()


@pytest.mark.parametrize("value", INVALID)
def test_invalid_values_act_as_0(monkeypatch, old_cc, value):
  ref = mixed_sequence(old_rig(monkeypatch, old_cc, 0))
  monkeypatch.undo()
  r = mixed_sequence(rig(monkeypatch, value))
  assert r.can == ref.can and r.cc.k9_creep is None and r.cc.k9_creep_mode == 0


@pytest.mark.parametrize("stale", [1, 2, 3, 4, 5, 6])
@pytest.mark.parametrize("mode", [None, 1, 2])  # 2026-10-25: 3 is no longer a menu value
def test_stale_old_key_is_never_read(monkeypatch, old_cc, stale, mode):
  if mode:  # no aeef67c6 reference for 1/2/3: same bytes as a fresh K9StopMode rig of that value
    ref = gentle_sequence(rig(monkeypatch, mode))
    monkeypatch.undo()
    r = gentle_sequence(rig(monkeypatch, mode, K9CreepMode=stale))
    assert r.can == ref.can and "K9CreepMode" not in r.params.reads and r.cc.k9_creep_mode == mode
    return
  # a device that still has K9CreepMode stored: only K9StopMode decides (unset = 0)
  ref = gentle_sequence(old_rig(monkeypatch, old_cc, OLD_OF_NEW[mode]))
  monkeypatch.undo()
  r = gentle_sequence(rig(monkeypatch, mode, K9CreepMode=stale))
  assert r.can == ref.can and "K9CreepMode" not in r.params.reads
  assert r.cc.k9_creep_mode == (mode or 0)


@pytest.mark.parametrize("mode", kc.MODES)
def test_no_camera_scc_k9_identical_to_old_0(monkeypatch, old_cc, mode):
  flags = int(K9_FLAGS & ~HyundaiFlags.CAMERA_SCC)
  ref = mixed_sequence(old_rig(monkeypatch, old_cc, 0, cp=make_cp(flags=flags)))
  monkeypatch.undo()
  r = rig(monkeypatch, mode, cp=make_cp(flags=flags))
  assert mixed_sequence(r).can == ref.can and r.cc.k9_creep is None and r.cc.k9_creep_mode == 0


@pytest.mark.parametrize("mode", kc.MODES)
def test_canfd_k9_guard_false(monkeypatch, mode):
  r = rig(monkeypatch, mode, cp=make_cp(flags=int(K9_FLAGS | HyundaiFlags.CANFD)))
  assert r.cc.k9_creep_applies is False and r.cc.k9_creep is None


def test_canfd_jerk_branch_ignores_creep_flag():
  def run(creep, obj=None):
    j = carcontroller.HyundaiJerk()
    cp = make_cp(flags=int(K9_FLAGS | HyundaiFlags.CANFD))
    cs = k9_rig.make_ci_state()
    cs.out = structs.CarState(vEgo=0.5, aEgo=-0.3)
    act = structs.CarControl().actuators
    act.longControlState, act.accel = LCS.stopping, -0.6
    j.make_jerk(cp, cs, -0.6, act, structs.CarControl().hudControl, creep=creep, creep_obj=obj, v12_orig=orig)
    return j.jerk_u, j.jerk_l, j.cb_upper, j.cb_lower
  orig = False
  assert run(True) == run(False) == run(True, K9Creep(V9))
  assert run(True) == run(True, K9Creep(V12K9))
  orig = True  # the v12 original jerk block is in the non-CAN-FD branch only
  assert run(False) == run(True, K9Creep(V12K9))
  orig = False
  assert run(False) == run(True)


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


@pytest.mark.parametrize("mode", ALL_KINDS)
def test_soft_hold_mode2_keeps_stopreq(monkeypatch, mode):
  rows = []
  patch_state(monkeypatch, softHoldActive=1)
  r = rig_kind(monkeypatch, mode)
  for _ in range(100):
    rows.append(r.step(-0.6, LCS.stopping, 0.3, a_target=-0.4, a_ego=-0.3))
  assert all(c is None or c["StopReq"] == 1 for _, c in rows)


@pytest.mark.parametrize("mode", ALL_KINDS)
def test_tcs13_standstill_latches_stopreq(monkeypatch, mode):
  patch_state(monkeypatch, cp=FakeCp({"TCS13": {"StandStill": 1}, "ESP12": {"LONG_ACCEL": -0.3}}))
  r = rig_kind(monkeypatch, mode)
  rows = [r.step(-0.6, LCS.stopping, 0.4, a_target=-0.4, a_ego=-0.3) for _ in range(40)]
  assert all(c is None or (c["StopReq"] == 1 and c["aReqValue"] <= -0.5 + 0.006) for _, c in rows)


def test_live_mode_switch_within_half_second(monkeypatch):
  # rewritten 2026-10-25: the light-band ramp this pinned belonged to the deleted menu 2; now 1 (v12 original) <-> 2 (v12k9)
  r = rig(monkeypatch, 1)
  for _ in range(60):
    r.step(-0.5, LCS.pid, 3.5, a_target=-0.5)
  for _ in range(60):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
  assert r.cc.k9_v12_orig and r.cc.k9_creep is None and r.cc.hyundai_jerk.cb_upper == 0.0
  r.params.values["K9StopMode"] = 2
  seen = None
  for i in range(80):
    acc, _ = r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
    if seen is None and r.cc.k9_creep is not None and r.cc.k9_creep.mode == V12K9:
      seen = i
  assert seen is not None and seen <= 50 and not r.cc.k9_v12_orig  # read every 0.5 s
  # pin updated 2026-10-25 brake_v12k9_live_fix: menu 2 now sends the original ComfortBand below 2.5 m/s (was 0)
  assert r.cc.hyundai_jerk.cb_upper == pytest.approx(float(np.clip(0.9 + 0.2 * acc, 0, 1.2)))
  for value in (4, 7, 8, float("nan"), -1):  # 3 = v13 original 2026-10-26, v14 original since 2026-10-28
    for back in (1, 2):
      r.params.values["K9StopMode"] = back
      for _ in range(51):
        r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
      assert r.cc.k9_creep_mode == back
      r.params.values["K9StopMode"] = value
      for _ in range(51):
        r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
      assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0 and not r.cc.k9_v12_orig


def test_live_mode_change_resets(monkeypatch):
  r = rig(monkeypatch, 2)
  for _ in range(60):
    r.step(-1.0, LCS.pid, 3.0)
  for i in range(60):
    r.step(-1.2, LCS.stopping, max(0.5 - 0.005 * i, 0.1))
  assert r.cc.k9_creep is not None and r.cc.k9_creep.mode == V12K9 and r.cc.k9_creep.last is not None
  r.params.values["K9StopMode"] = 1
  for _ in range(60):
    r.step(-1.2, LCS.stopping, 0.1)
  assert r.cc.k9_creep is None and r.cc.k9_v12_orig and hasattr(r.cc, "accel_feather_last")
  r.params.values["K9StopMode"] = 0
  out = [r.step(-1.2, LCS.stopping, 0.1) for _ in range(60)]
  assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0 and not r.cc.k9_v12_orig
  assert not hasattr(r.cc, "accel_feather_last") and r.cc.stop_req_latched is False  # the page state starts fresh
  sent = [o for o in out[-10:] if o[1] is not None]
  assert sent and all(o[1]["StopReq"] == 1 and o[0] == pytest.approx(-1.2) for o in sent)


def test_setting_registered_single_selector_old_keys_deleted():
  import json
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  entry = next(p for p in catalog["params"] if p["name"] == "K9StopMode")
  assert (entry["min"], entry["max"], entry["default"], entry["unit"]) == (0, 3, 0, 1)  # 3 v13 original 2026-10-26, v14 original since 2026-10-28
  assert [len(entry["options"][loc]) for loc in ("ko", "en", "zh")] == [4, 4, 4]
  assert not any(p["name"] in ("K9CreepMode", "K9CreepV9", "K9CreepV8", "K9StopTaper") for p in catalog["params"])
  keys = (ROOT / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
  assert '{"K9StopMode", {PERSISTENT, INT, "0"}},' in keys and '{"K9StopModeVer", {PERSISTENT, INT}},' in keys
  assert "K9CreepMode" not in keys and "K9CreepV9" not in keys and "K9StopTaper" not in keys
  assert not (ROOT / "opendbc_repo/opendbc/car/hyundai/k9_stop_taper.py").exists()
