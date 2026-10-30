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
from opendbc.car.hyundai.k9_creep import K9Creep, V9, V12K9, V12K9O
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
INVALID = (4, 5, 6, 7, 8, 99, -1, -4, float("nan"), "nan", float("inf"))  # 3 = v14 original since 2026-10-28;
# 2026-10-29 05:40: 4 is a menu value (kind 15, 원본2 + the 1 s hold), so it left this list; 07:10: the v14 original
# left the menu with its code and kind 15 moved to 3, so 4 is back here


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
  # 2026-10-29 05:40: the v12 original left the menu with its code, so V12_ORIG_MENU is gone and 4 is a menu value;
  # 07:10 the v14 original left the same way, so V14_ORIG_MENU is gone too and kind 15 moved from menu 4 to menu 3
  assert kc.MODES == (1, 2, 3) and (V9, V12K9) == (1, 8) and not hasattr(kc, "V12_ORIG_MENU")
  assert not hasattr(kc, "V13_ORIG_MENU") and not hasattr(kc, "V14_ORIG_MENU")
  # 2026-10-29 bono1739 원본: menu 2 = kind 13 (kind 8 + the driver's own 17:39:14 stop table); kind 8 stays a kind (its own pins)
  assert kc.MENU == {1: V12K9O, 2: kc.V12K9OS, 3: kc.V12K9OH}  # 1 원본2 (04:48), 2 끝 구간 개선 (05:40) / 3 선 뒤 1초 잠금 (07:10)
  assert kc.KINDS == ALL_KINDS + (V12K9, V12K9O, kc.V12K9OS, kc.V12K9OH) and (kc.V12K9OS, kc.V12K9OH) == (14, 15)
  assert not any(hasattr(K9Creep(m), "shudder_fix") for m in kc.KINDS)
  for bad in (0, 2, 3, 4, 5, 6, 7, 9, 12, 16, -1):  # kinds 14 / 15 are real since 2026-10-29 05:40
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
  # 2026-10-29 07:10: 1 = kind 13 (bono1739 원본2), 2 = kind 14 (끝 구간 개선), 3 = kind 15 (원본2 + the 1 s hold); the v14
  # original, the last menu value without a K9Creep object, left the menu with its page code
  for stored, kind in ((1, V12K9O), (2, kc.V12K9OS), (3, kc.V12K9OH)):
    r = rig(monkeypatch, stored)
    r.step(-0.5, LCS.pid, 3.0)
    assert r.cc.k9_creep_mode == stored and (r.cc.k9_creep is None if kind is None else r.cc.k9_creep.mode == kind)
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
    j.make_jerk(cp, cs, -0.6, act, structs.CarControl().hudControl, creep=creep, creep_obj=obj)
    return j.jerk_u, j.jerk_l, j.cb_upper, j.cb_lower
  assert run(True) == run(False) == run(True, K9Creep(V9))
  assert run(True) == run(True, K9Creep(V12K9))
  assert run(False) == run(True)
  # the v12_orig jerk case this also pinned went with the v12 original (2026-10-29 05:40)


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
  # rewritten 2026-10-25: the light-band ramp this pinned belonged to the deleted menu 2; now 2 (v12 original) <-> 1
  # (his own stop table, kind 13) after the 2026-10-29 04:48 slot swap; since 05:40 the page-code menu is 3 (v14
  # original), the v12 original having left the menu, so the "no K9Creep, band 0" side of the switch is menu 3
  # 2026-10-29 07:10: the v14 original left the menu with its page code as well, so no menu value runs pasted page code
  # any more and the "no K9Creep" side of the switch is menu 0 (base braking). The band-0 half of the pin went with it.
  r = rig(monkeypatch, 0)
  for _ in range(60):
    r.step(-0.5, LCS.pid, 3.5, a_target=-0.5)
  for _ in range(60):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
  assert r.cc.k9_creep is None
  r.params.values["K9StopMode"] = 1
  seen = None
  for i in range(80):
    acc, _ = r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
    if seen is None and r.cc.k9_creep is not None and r.cc.k9_creep.mode == V12K9O:  # menu 1 = kind 13 since 2026-10-29 04:48
      seen = i
  assert seen is not None and seen <= 50  # read every 0.5 s
  # pin updated 2026-10-25 brake_v12k9_live_fix: menu 2 now sends the original ComfortBand below 2.5 m/s (was 0)
  assert r.cc.hyundai_jerk.cb_upper == pytest.approx(float(np.clip(0.9 + 0.2 * acc, 0, 1.2)))
  for value in (4, 5, 7, 8, float("nan"), -1):  # 4 is back off the menu since 2026-10-29 07:10 (kind 15 moved to 3)
    for back in (1, 2):
      r.params.values["K9StopMode"] = back
      for _ in range(51):
        r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
      assert r.cc.k9_creep_mode == back
      r.params.values["K9StopMode"] = value
      for _ in range(51):
        r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
      assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0


def test_live_mode_change_resets(monkeypatch):
  r = rig(monkeypatch, 1)  # his own stop table: menu 1 since 2026-10-29 04:48
  for _ in range(60):
    r.step(-1.0, LCS.pid, 3.0)
  for i in range(60):
    r.step(-1.2, LCS.stopping, max(0.5 - 0.005 * i, 0.1))
  assert r.cc.k9_creep is not None and r.cc.k9_creep.mode == V12K9O and r.cc.k9_creep.last is not None  # kind 13 since 2026-10-29
  # 2026-10-29 07:10: the v14 original left the menu with its page code, so the mid-step of this pin is menu 3 (kind 15)
  # and the controller page state it used to check (accel_feather_last / stop_req_latched) no longer exists. A mode change
  # still has to build a fresh K9Creep object, which is what is checked instead.
  first = r.cc.k9_creep
  r.params.values["K9StopMode"] = 3  # kind 15 (원본2 + the 1 s hold): menu 3 since 07:10, menu 4 before
  for _ in range(60):
    r.step(-1.2, LCS.stopping, 0.1)
  assert r.cc.k9_creep is not None and r.cc.k9_creep.mode == kc.V12K9OH and r.cc.k9_creep is not first
  r.params.values["K9StopMode"] = 0
  out = [r.step(-1.2, LCS.stopping, 0.1) for _ in range(60)]
  assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0
  sent = [o for o in out[-10:] if o[1] is not None]
  assert sent and all(o[1]["StopReq"] == 1 and o[0] == pytest.approx(-1.2) for o in sent)


# --- 3 = menu 1 + the extra braking 1 s after the stop (kind 15; menu 4 until 2026-10-29 07:10) ------------------------------------------------------
def _stop_and_stand(monkeypatch, mode, frames=400):
  """Roll to a stop in the stopping state and keep standing; returns the sent accel of every standing frame."""
  r = rig(monkeypatch, mode)
  for i in range(120):  # approach below B17O_SPEED so the driver's table runs, as in menus 1 / 3
    r.step(-0.5, LCS.stopping, max(2.5 - 0.025 * i, 0.06), a_target=-0.4, a_ego=-0.4)
  out = []
  for _ in range(frames):
    acc, _ = r.step(-0.3, LCS.stopping, 0.0, standstill=True, a_target=-0.3, a_ego=0.0)
    out.append(acc)
  return r, out


def test_menu3_is_menu1_for_one_second_then_presses_harder(monkeypatch):
  assert (kc.B17O_HOLD2_T, kc.B17O_HOLD2_ACCEL, kc.B17O_HOLD2_STEP) == (1.0, -1.00, 0.0060)
  _, one = _stop_and_stand(monkeypatch, 1)
  monkeypatch.undo()
  _, three = _stop_and_stand(monkeypatch, 3)
  monkeypatch.undo()
  n = int(kc.B17O_HOLD2_T * 100.0)
  assert one[:n] == three[:n]  # the first second after the stop is menu 1 bit for bit
  assert all(b <= a + 1e-9 for a, b in zip(one, three))  # never weaker than menu 1 on any frame
  extra = abs(kc.B17O_HOLD2_ACCEL - kc.LATCH_ACCEL)
  gain = one[-1] - three[-1]
  # The realized extra can be less than |B17O_HOLD2_ACCEL - LATCH_ACCEL|: the existing zone clamp against the kind 8
  # reference (V12K9_ZONE_PRESS) limits how far past that reference any request may go, whatever asks for it. In this
  # sequence it settles 0.40 of the 0.50 asked for. The clamp is not touched here.
  assert 0.3 <= gain <= extra + 1e-6
  reach = next(i for i, (a, b) in enumerate(zip(one, three)) if a - b >= gain - 1e-6)
  assert n < reach <= n + int(extra / kc.B17O_HOLD2_STEP) + 3  # and gets there on the B17O_HOLD2_STEP ramp


def test_menu3_difference_is_exactly_the_one_constant(monkeypatch):
  """B17O_HOLD2_ACCEL is the single number to turn: set it to the standstill hold and menu 3 is menu 1 again."""
  _, one = _stop_and_stand(monkeypatch, 1)
  monkeypatch.undo()
  monkeypatch.setattr(kc, "B17O_HOLD2_ACCEL", kc.LATCH_ACCEL)
  _, three = _stop_and_stand(monkeypatch, 3)
  monkeypatch.undo()
  assert one == three


def test_menu3_rolling_again_clears_the_second(monkeypatch):
  r = rig(monkeypatch, 3)
  for i in range(120):
    r.step(-0.5, LCS.stopping, max(2.5 - 0.025 * i, 0.06), a_target=-0.4, a_ego=-0.4)
  for _ in range(160):  # past B17O_HOLD2_T: the stronger hold is running
    acc, _ = r.step(-0.3, LCS.stopping, 0.0, standstill=True, a_target=-0.3, a_ego=0.0)
  assert acc < kc.LATCH_ACCEL
  assert r.cc.k9_creep.hold2_k > int(kc.B17O_HOLD2_T * 100.0)
  r.step(-0.3, LCS.stopping, 0.4, a_target=-0.3, a_ego=0.0)  # rolling again
  assert r.cc.k9_creep.hold2_k == 0
  acc, _ = r.step(-0.3, LCS.stopping, 0.0, standstill=True, a_target=-0.3, a_ego=0.0)
  assert acc >= kc.LATCH_ACCEL - 1e-9  # back to the menu 1 hold at once


def _stop_go_stop(monkeypatch, mode, gas, frames=400):
  """Stop and stand 3 s, start again (the plan alone = an openpilot start with no pedal, so no interlock reset(); `gas`: the
  driver's pedal for the first 0.2 s), drive at 3 m/s and stop again as _stop_and_stand. Returns the sent accel of the last
  standing frame of the first stop and of every standing frame of the second stop."""
  r = rig(monkeypatch, mode)
  for i in range(120):
    r.step(-0.5, LCS.stopping, max(2.5 - 0.025 * i, 0.06), a_target=-0.4, a_ego=-0.4)
  for _ in range(300):
    held, _ = r.step(-0.3, LCS.stopping, 0.0, standstill=True, a_target=-0.3, a_ego=0.0)
  for i in range(300):  # the start: pid up to 3 m/s
    r.step(0.5, LCS.pid, min(0.01 * i, 3.0), standstill=i == 0, a_target=0.5, a_ego=0.5, gas=gas and i < 20)
  for _ in range(100):
    r.step(0.0, LCS.pid, 3.0, a_target=0.0, a_ego=0.0)
  for i in range(140):
    r.step(-0.5, LCS.stopping, max(3.0 - 0.025 * i, 0.06), a_target=-0.4, a_ego=-0.4)
  out = []
  for _ in range(frames):
    acc, _ = r.step(-0.3, LCS.stopping, 0.0, standstill=True, a_target=-0.3, a_ego=0.0)
    out.append(acc)
  return held, out


@pytest.mark.parametrize("gas", [False, True])
def test_menu3_next_stop_after_a_start_counts_its_second_again(monkeypatch, gas):
  """2026-10-30 (m3hold2fix_1030): stop, 3 s standing, a start with no pedal (or with the gas pedal), 3 m/s, stop again: the
  second stop is menu 1 bit for bit for its first B17O_HOLD2_T and only then presses harder, like the first one."""
  held1, one = _stop_go_stop(monkeypatch, 1, gas)
  monkeypatch.undo()
  held3, three = _stop_go_stop(monkeypatch, 3, gas)
  monkeypatch.undo()
  assert held3 < held1 - 0.3  # the stronger hold of the first stop ran up to the start
  n = int(kc.B17O_HOLD2_T * 100.0)
  assert one[:n] == three[:n]  # the first second of the second stop is menu 1 bit for bit
  assert all(b <= a + 1e-9 for a, b in zip(one, three, strict=True))  # never weaker than menu 1 on any frame
  assert one[-1] - three[-1] >= 0.3  # then the stronger hold again


@pytest.mark.parametrize("mode", [1, 2])
def test_menu3_extra_hold_runs_in_menu_3_only(monkeypatch, mode):
  """Turning B17O_HOLD2_ACCEL changes nothing in menus 1 and 2: the extra hold is kind 15 (menu 3) only."""
  _, ref = _stop_and_stand(monkeypatch, mode, frames=300)
  monkeypatch.undo()
  monkeypatch.setattr(kc, "B17O_HOLD2_ACCEL", -2.5)
  _, out = _stop_and_stand(monkeypatch, mode, frames=300)
  monkeypatch.undo()
  assert ref == out


# --- 3: below 1.5 km/h in the stopping state the request presses harder frame by frame (kind 15 only; m3ramp_1030) ---------
def _slow_entry(monkeypatch, mode, stand=250, hover=200):
  """2026-10-30 (m3ramp_1030, the 05:02:53.9 menu 3 stop): pid down to 0.43 m/s, then the stopping state from 0.43 m/s with the
  plan's decel fading from -0.35 to 0 while the car hovers `hover` frames down from 0.43 m/s (0.33 m/s for 200) with almost no
  decel (aEgo -0.05), the roll to the stop and `stand` standing frames. Returns the rig and (v, standstill, sent accel, StopReq)
  of every frame."""
  r = rig(monkeypatch, mode)
  out = []

  def step(state, v, a_target, a_ego, standstill=False):
    acc, sent = r.step(a_target, state, v, standstill=standstill, a_target=a_target, a_ego=a_ego)
    out.append((v, standstill, acc, None if sent is None else sent["StopReq"]))

  for i in range(100):  # pid 1.2 -> 0.43 m/s
    step(LCS.pid, 1.2 - 0.0077 * i, -0.4, -0.4)
  for i in range(hover):  # the stopping state from 0.43 m/s: 27 frames above 1.5 km/h, then the hover
    step(LCS.stopping, 0.43 - 0.0005 * i, -0.35 * max(1.0 - i / 150.0, 0.0), -0.05)
  v0 = round(0.43 - 0.0005 * hover, 4)
  for i in range(round(v0 / 0.01) + 1):  # the roll to the stop
    step(LCS.stopping, max(v0 - 0.01 * i, 0.0), 0.0, -0.3)
  for _ in range(stand):
    step(LCS.stopping, 0.0, 0.0, 0.0, standstill=True)
  return r, out


def test_menu3_presses_harder_below_1_5_kmh_never_weaker_than_menu1(monkeypatch):
  """Above B17O_RAMP_V menu 3 is menu 1 bit for bit (CAN too). At or below it, in the stopping state before the stop, the
  request goes past menu 1's table value while aEgo brakes less than that value: by B17O_RAMP_STEP per frame up to
  B17O_RAMP_KNEE, then by B17O_RAMP_STEP2 per frame up to B17O_RAMP_MAX. StopReq is menu 1's on every frame and no frame is
  weaker than menu 1."""
  assert (kc.B17O_RAMP_V, kc.B17O_RAMP_STEP, kc.B17O_RAMP_KNEE, kc.B17O_RAMP_STEP2, kc.B17O_RAMP_MAX) == (0.4167, 0.015, 0.45, 0.002, 0.60)
  r1, one = _slow_entry(monkeypatch, 1)
  monkeypatch.undo()
  r3, three = _slow_entry(monkeypatch, 3)
  monkeypatch.undo()
  n = next(i for i, o in enumerate(one) if o[0] <= kc.B17O_RAMP_V)
  assert n == 127 and all(o[0] > kc.B17O_RAMP_V for o in one[:n])
  assert three[:n] == one[:n] and r3.can[:n] == r1.can[:n]  # above 1.5 km/h: menu 1 bit for bit
  assert [o[3] for o in three] == [o[3] for o in one]  # StopReq: menu 1's on every frame
  gap = [a[2] - b[2] for a, b in zip(one, three, strict=True)]
  assert min(gap) >= -1e-9  # never weaker than menu 1 on any frame
  low = [i for i in range(n, len(one)) if one[i][0] >= kc.STANDSTILL_SPEED]  # the table frames at or below 1.5 km/h (latch below)
  assert gap[low[0]] == pytest.approx(kc.B17O_RAMP_STEP, abs=1e-6)  # from the table value, one step per frame (float32 sent)
  assert all(-1e-6 <= gap[i] - gap[i - 1] <= kc.B17O_RAMP_STEP + 1e-6 for i in low[1:])  # step by step, never back
  assert max(gap[i] for i in low) == pytest.approx(kc.B17O_RAMP_MAX, abs=1e-6)  # up to B17O_RAMP_MAX past the table


@pytest.mark.parametrize("hover, lo, hi", [(200, 0.60, 0.60), (60, 0.45, 0.50)])
def test_menu3_ramp_hands_over_at_the_stop_without_weakening(monkeypatch, hover, lo, hi):
  """At the stop the brake does not drop back from the ramped request at the hand-over to the latch path, also while the band
  compensation of the first latch frames (-0.20 at the full band, 0.4 s) decays: the floor is the strongest request sent since
  the last ramp frame until the latch request without the band compensation passes it. Fully ramped (hover 200) and past
  B17O_RAMP_KNEE (hover 60) the latch request starts weaker than the ramp. After that the standing hold is menu 3's as before
  (B17O_HOLD2_* unchanged)."""
  _, one = _slow_entry(monkeypatch, 1, hover=hover)
  monkeypatch.undo()
  _, three = _slow_entry(monkeypatch, 3, hover=hover)
  monkeypatch.undo()
  monkeypatch.setattr(kc, "B17O_RAMP_MAX", 0.0)  # no ramp: menu 3 as before it
  _, before = _slow_entry(monkeypatch, 3, hover=hover)
  monkeypatch.undo()
  h = next(i for i, o in enumerate(three) if o[0] < kc.STANDSTILL_SPEED) - 1  # the last table frame (the latch from the next)
  acc = [o[2] for o in three]
  assert lo - 1e-6 <= one[h][2] - acc[h] <= hi + 1e-6  # how far past the table it comes to the stop
  assert max(b - a for a, b in zip(acc[h:], acc[h + 1:])) <= 1e-6  # from the last table frame on: no frame weaker than the one before it
  assert all(b[2] <= a[2] + 1e-9 for a, b in zip(one, three, strict=True))  # never weaker than menu 1
  assert all(b[2] <= a[2] + 1e-9 for a, b in zip(before, three, strict=True))  # never weaker than menu 3 before the ramp
  assert three[-1][2] == before[-1][2] < one[-1][2] - 0.3  # then the extra hold of menu 3 as before


def test_menu3_ramp_restarts_after_leaving_the_stopping_state(monkeypatch):
  """Leaving the stopping state clears the ramp and its floor (the 2026-10-30 00:53 hold2_k lesson): entering it again starts
  from the table value."""
  def run(mode):
    r = rig(monkeypatch, mode)
    out = [r.step(-0.35, LCS.stopping, 0.41 - 0.0005 * i, a_target=-0.35, a_ego=-0.05)[0] for i in range(100)]
    st = (r.cc.k9_creep.b17o_ramp, r.cc.k9_creep.b17o_hold)
    out.append(r.step(0.3, LCS.pid, 0.36, a_target=0.3, a_ego=0.0)[0])  # the plan lets go for one frame
    st += (r.cc.k9_creep.b17o_ramp, r.cc.k9_creep.b17o_hold)
    out += [r.step(0.0, LCS.stopping, 0.36, a_target=0.0, a_ego=-0.05)[0] for _ in range(20)]  # stops again, plan 0: the table
    return out, st
  one, _ = run(1)
  monkeypatch.undo()
  three, st = run(3)
  monkeypatch.undo()
  assert st[0] == pytest.approx(-(kc.B17O_RAMP_KNEE + 70 * kc.B17O_RAMP_STEP2)) and st[1] == pytest.approx(three[99], abs=1e-6)  # the ramp and its stop floor are armed
  assert st[2] == 0.0 and st[3] is None  # cleared by the pid frame
  gap = [a - b for a, b in zip(one, three, strict=True)]
  assert gap[100] == 0.0  # the pid frame is menu 1's
  assert gap[101] == pytest.approx(kc.B17O_RAMP_STEP, abs=1e-6)  # the next stop starts from the table value again
  assert gap[120] == pytest.approx(20 * kc.B17O_RAMP_STEP, abs=1e-6)


@pytest.mark.parametrize("mode", [1, 2])
def test_menu3_ramp_runs_in_menu_3_only(monkeypatch, mode):
  """Turning the B17O_RAMP_* constants changes nothing in menus 1 and 2: the ramp is kind 15 (menu 3) only."""
  _, ref = _slow_entry(monkeypatch, mode)
  monkeypatch.undo()
  monkeypatch.setattr(kc, "B17O_RAMP_STEP", 0.05)
  monkeypatch.setattr(kc, "B17O_RAMP_MAX", 2.0)
  _, out = _slow_entry(monkeypatch, mode)
  monkeypatch.undo()
  assert ref == out


def test_setting_registered_single_selector_old_keys_deleted():
  import json
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  entry = next(p for p in catalog["params"] if p["name"] == "K9StopMode")
  assert (entry["min"], entry["max"], entry["default"], entry["unit"]) == (0, 3, 0, 1)  # max 3 since 2026-10-29 07:10 (kind 15 = 3)
  assert [len(entry["options"][loc]) for loc in ("ko", "en", "zh")] == [4, 4, 4]
  assert not any(p["name"] in ("K9CreepMode", "K9CreepV9", "K9CreepV8", "K9StopTaper") for p in catalog["params"])
  keys = (ROOT / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
  assert '{"K9StopMode", {PERSISTENT, INT, "0"}},' in keys and '{"K9StopModeVer", {PERSISTENT, INT}},' in keys
  assert "K9CreepMode" not in keys and "K9CreepV9" not in keys and "K9StopTaper" not in keys
  assert not (ROOT / "opendbc_repo/opendbc/car/hyundai/k9_stop_taper.py").exists()
