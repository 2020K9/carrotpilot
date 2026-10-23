"""K9StopMode single selector (2026-10-23 07:25 menu): 0 base braking, 1 v10 (test_k9_stop_v10.py), 2 v9 + shudder fix (low-speed
ComfortBand restore + braking compensation + creep_fix-method sent-value gate, never weaker than v9), 3 v10 + band 0.2 (r6)
(test_k9_stop_v10_fix.py), 4 v10 + shudder fix (2026-10-24, test_k9_stop_v10_smooth.py). Other values act as 0.
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
from opendbc.car.hyundai.k9_creep import K9Creep, V9, V10, V9_SHUDDER_FIX, V10_SHUDDER_FIX, V10_SMOOTH
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.selfdrive.car.tests import k9_rig
from openpilot.selfdrive.car.tests.k9_rig import (CC_REL, LCS, ROOT, Rig, edge_sequence, gentle_sequence, load_frozen_cc,
                                                  mixed_sequence)
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import K9_FLAGS, make_cp

OLD_MENU = "aeef67c6"  # K9CreepMode 0 base, 1 v8, 2 v9, 3/4/5 candidates, 6 v9 + shudder fix 4
OLD_OF_NEW = {None: 0, 0: 0}  # aeef67c6 K9CreepMode of the new menu value (v9 = old 2 is checked as the kind V9 below)
OLD_KIND = {V9: 2}  # aeef67c6 K9CreepMode 2 = v9 (the K9Creep kind V9 of this tree)
PREV_MENU = "1d0961da"  # K9StopMode 0 base, 1 v9, 2 v10, 3 v9 + shudder fix
PREV_OF_NEW = {None: None, 0: 0, 1: 2, 2: 3}  # 1d0961da K9StopMode of the new menu value (new 3 is new)
KIND_OF_PREV = {2: V10, 3: V9_SHUDDER_FIX}
ALL = (V9, V9_SHUDDER_FIX)
ALL_KINDS = (V9, V10, V9_SHUDDER_FIX, V10_SHUDDER_FIX, V10_SMOOTH)
INVALID = (5, 6, 7, 99, -1, -4, float("nan"), "nan", float("inf"))


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
  assert kc.MODES == (1, 2, 3, 4) and (V9, V10, V9_SHUDDER_FIX, V10_SHUDDER_FIX, V10_SMOOTH) == (1, 2, 3, 4, 5)
  assert kc.MENU == {1: V10, 2: V9_SHUDDER_FIX, 3: V10_SHUDDER_FIX, 4: V10_SMOOTH} and kc.KINDS == ALL_KINDS
  assert (kc.BAND_FULL_SPEED, kc.BAND_OFF_SPEED, kc.BAND_RAMP_STEP) == (2.0, 3.0, 0.04)
  assert (kc.BLEND_FULL_SPEED, kc.BLEND_OFF_SPEED, kc.BLEND_GAIN) == (0.6, 0.8, 0.5)
  assert (kc.COMP_DECEL, kc.COMP_FADE_BP) == (0.10, (0.3, 0.6))
  assert [K9Creep(m).shudder_fix for m in kc.KINDS] == [False, False, True, False, False]
  for bad in (0, 6, 7, -1):
    with pytest.raises(AssertionError):
      K9Creep(bad)


def test_removed_modes_are_gone_from_source():
  src = (ROOT / "opendbc_repo/opendbc/car/hyundai/k9_creep.py").read_text(encoding="utf-8")
  for word in ("V8", "_up_v8", "PROFILES", "C3_", "V9_C2", "V9_BL", "low_hold", "bl_band", "C2BL", "K9CreepMode = "):
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
  assert c.cb_w is None and c.last is None


def test_band_weight_target_and_ramp_unit():
  assert kc.band_weight(0.0) == 1.0 and kc.band_weight(2.0) == 1.0 and kc.band_weight(2.5) == 0.5
  assert kc.band_weight(3.0) == 0.0 and kc.band_weight(10.0) == 0.0 and kc.band_weight(float("nan")) == 1.0
  assert K9Creep(V9).bands(0.5, 0.3, 0.8, 0.7, 0.8) == K9Creep.jerk_bands(0.5, 0.3)  # 1: exactly the v9 band
  c = K9Creep(V9_SHUDDER_FIX)
  ws = []
  for _ in range(30):  # prev band 0 (v9 at speed) at 0.3 m/s: ramps 0 -> 1 in 25 calls
    ju, cu, cl = c.bands(0.5, 0.3, 0.8, 0.7, 0.0 if not ws else ws[-1] * 0.8)
    assert ju == K9Creep.jerk_bands(0.5, 0.3)[0]
    ws.append(c.cb_w)
    assert cu == pytest.approx(0.8 * c.cb_w) and cl == pytest.approx(0.7 * c.cb_w)
  assert ws[0] == pytest.approx(0.04) and ws[24] == pytest.approx(1.0) and ws[-1] == 1.0
  assert np.all(np.abs(np.diff([0.0] + ws)) <= kc.BAND_RAMP_STEP + 1e-12)
  c.reset()
  c.bands(0.5, 0.3, 0.8, 0.7, 0.8)  # after a reset it starts from the band actually sent (original): no jump
  assert c.cb_w == 1.0


def test_blend_weight_unit():
  assert kc.blend_weight(0.0) == 1.0 and kc.blend_weight(0.6) == 1.0 and kc.blend_weight(0.7) == pytest.approx(0.5)
  assert kc.blend_weight(0.8) == 0.0 and kc.blend_weight(5.0) == 0.0 and kc.blend_weight(float("nan")) == 1.0


def test_mode2_band_comp_unit():
  c = K9Creep(V9_SHUDDER_FIX)
  comp = kc.COMP_DECEL
  assert c._band_comp(-0.5, -0.5, 1.0) == pytest.approx(comp) and c._band_comp(-0.5, -0.5, 2.0) == pytest.approx(comp)
  assert c._band_comp(-0.5, -0.5, 2.5) == pytest.approx(comp * 0.5) and c._band_comp(-0.5, -0.5, 3.0) == 0.0
  assert c._band_comp(-0.5, -0.5, 0.6) == pytest.approx(comp) and c._band_comp(-0.5, -0.5, 0.45) == pytest.approx(comp / 2)
  assert c._band_comp(-0.5, -0.5, 0.3) == 0.0 and c._band_comp(-0.5, -0.5, 0.1) == 0.0
  assert c._band_comp(0.0, -0.5, 1.0) == 0.0 and c._band_comp(0.2, -0.5, 1.0) == 0.0  # not braking: no extra braking
  assert c._band_comp(-0.5, -0.05, 1.0) == 0.0 and c._band_comp(-0.5, -0.125, 1.0) == pytest.approx(comp / 2)
  assert c._band_comp(-0.5, -0.5, float("nan")) == pytest.approx(comp)  # non-finite speed: toward the original band
  assert K9Creep(V9)._band_comp(-0.5, -0.5, 1.0) == 0.0


def _comp(v, a_target, accel):
  return K9Creep(V9_SHUDDER_FIX)._band_comp(accel, a_target, v)


@pytest.mark.parametrize("orig", [-1.2, -0.9, -0.6, -0.45, -0.3, -0.1, 0.1])
@pytest.mark.parametrize("margin_kw", [{}, {"long_accel": -0.05, "a_ego": -0.1}, {"long_accel": -0.6, "a_ego": -0.6},
                                       {"long_accel": None, "a_ego": -0.2}])
def test_mode2_never_weaker_than_v9_stopreq_and_state_are_v9(orig, margin_kw):
  # pid approach 3.5 -> 1.0 m/s, then stopping 1.0 -> 0.06 m/s
  c9, c2 = K9Creep(V9), K9Creep(V9_SHUDDER_FIX)
  differ = 0
  for v in np.linspace(3.5, 1.0, 120):
    kw = dict(accel=orig, stopping=False, pid=True, v_ego=float(v), v_ego_raw=float(v), a_target=-0.5, **margin_kw)
    a9, s9 = upd(c9, **kw)
    a2, s2 = upd(c2, **kw)
    assert s2 == s9 is False and c2.last == c9.last
    assert a2 <= a9 + 1e-12 and a2 >= a9 - _comp(v, -0.5, a9) - 1e-12  # extra braking at most the band comp
    differ += a2 != a9
  for v in np.linspace(1.0, 0.06, 120):
    a9, s9 = upd(c9, accel=orig, v_ego=float(v), v_ego_raw=float(v), **margin_kw)
    a2, s2 = upd(c2, accel=orig, v_ego=float(v), v_ego_raw=float(v), **margin_kw)
    assert s2 == s9 is False  # StopReq withheld exactly like v9
    assert c2.last == c9.last  # the v9 feather state is not changed
    assert a2 <= a9 + 1e-12  # never less braking than v9
    differ += a2 != a9
  assert differ > 0 or orig >= 0.0  # not braking: no extra braking at all


def test_mode2_latches_like_v9():
  for kw in ({"esp_standstill": True}, {"a_target": -1.0}, {"a_ego": 0.07}, {"lead_distance": 2.5},
             {"v_ego": 0.04, "v_ego_raw": 0.04}, {"long_accel": 0.2, "a_ego": 0.0}):
    c9, c2 = K9Creep(V9), K9Creep(V9_SHUDDER_FIX)
    for _ in range(3):
      r9, r2 = upd(c9, **kw), upd(c2, **kw)
      assert r2 == r9 and r2[1] is True
  for kw in ({"gas": True}, {"brake": True}, {"override": True}, {"long_active": False}, {"can_valid": False}):
    assert upd(K9Creep(V9_SHUDDER_FIX), **kw) == (-1.0, True)  # interlock: the original command at once


def _margin(v, la, ae):
  return min(la, ae + 0.10) - float(np.interp(v, kc.MIN_DECEL_BP, kc.MIN_DECEL_V))


def _feather_inputs(rng, n=400, v0=1.0):
  """stopping state before any latch (a_ego <= 0.05, eff_ac <= 0.03, v >= 0.06): noisy original command/feedback."""
  for i in range(n):
    v = max(v0 - (v0 - 0.06) * i / (n - 1), 0.06)
    ae = rng.uniform(-0.8, 0.05)
    la = min(rng.uniform(-0.9, 0.2), ae + 0.10, 0.03)
    yield {"accel": rng.uniform(-1.3, -0.1), "a_target": rng.uniform(-0.9, -0.1), "v_ego": v, "v_ego_raw": v,
           "a_ego": ae, "long_accel": la}


@pytest.mark.parametrize("seed", range(8))
def test_mode2_feather_gate_no_repress_hold_and_floor(seed):
  rng = random.Random(f"k9-gate-{seed}")
  c9, c2 = K9Creep(V9), K9Creep(V9_SHUDDER_FIX)
  for v in np.linspace(3.0, 1.0, 80):  # pid approach
    kw = {"accel": -0.7, "stopping": False, "pid": True, "v_ego": float(v), "v_ego_raw": float(v), "a_target": -0.5}
    upd(c9, **kw), upd(c2, **kw)
  prev = c2.sent
  for kw in _feather_inputs(rng):
    a9, s9 = upd(c9, **kw)
    a2, s2 = upd(c2, **kw)
    v = kw["v_ego"]
    assert s9 is False and s2 is False and c2.last == c9.last and not c2.latched  # StopReq/state exactly v9
    floor = float(np.interp(v, kc.FLOOR_BP, kc.FLOOR_V))
    assert a2 <= a9 + 1e-12 and (prev > floor or a2 <= floor + 1e-12)  # once at the K9 creep floor, never above it
    if a2 < prev - 1e-12 and a2 != a9:  # mode 2 itself pressed more: only by DOWN_STEP, only when decel is weak
      assert prev - a2 <= kc.DOWN_STEP + 1e-12
      assert v > kc.DOWN_UP_SPEED and _margin(v, kw["long_accel"], kw["a_ego"]) > kc.GATE_DOWN_MARGIN
    if v <= kc.HOLD_SPEED:
      assert a2 <= prev + 1e-12  # K9 wheels already stopped: no release
    else:
      assert a2 <= prev + kc.UP_LOW + kc.CATCH_UP + 1e-12  # release rate
    prev = a2


def test_mode2_latch_never_releases_and_stopreq_is_v9():
  for latch_kw in ({"esp_standstill": True}, {"v_ego": 0.04, "v_ego_raw": 0.04}, {"a_target": -1.2}, {"a_ego": 0.07},
                   {"lead_distance": 2.0}):
    c9, c2 = K9Creep(V9), K9Creep(V9_SHUDDER_FIX)
    for v in np.linspace(0.6, 0.12, 120):
      kw = {"accel": -0.5, "v_ego": float(v), "v_ego_raw": float(v), "a_ego": -0.5, "long_accel": -0.5}
      upd(c9, **kw), upd(c2, **kw)
    before = c2.sent
    assert before is not None and before <= float(np.interp(0.12, kc.FLOOR_BP, kc.FLOOR_V)) + 1e-12
    for accel in (-0.5, -0.45, -0.8, -1.0):
      kw = {"accel": accel, "v_ego": 0.1, "v_ego_raw": 0.1, "a_ego": -0.3, "long_accel": -0.3}
      kw.update(latch_kw)
      a9, s9 = upd(c9, **kw)
      a2, s2 = upd(c2, **kw)
      assert s9 is True and s2 is True and c2.latched and c2.last == c9.last
      assert a2 == pytest.approx(min(a9, before)) and a2 <= a9
      before = a2


@pytest.mark.parametrize("seed", range(6))
def test_mode2_pid_gate_below_6_4_kmh(seed):
  rng = random.Random(f"k9-pid-gate-{seed}")
  c9, c2 = K9Creep(V9), K9Creep(V9_SHUDDER_FIX)
  prev = None
  for i in range(600):
    v = 2.5 - 2.0 * i / 599
    ae = rng.uniform(-0.9, 0.0)
    kw = {"accel": rng.uniform(-1.2, -0.05), "a_target": rng.choice([rng.uniform(-0.9, -0.06), -0.3]), "stopping": False,
          "pid": True, "v_ego": v, "v_ego_raw": v, "a_ego": ae, "long_accel": rng.uniform(-1.0, 0.1)}
    a9, _ = upd(c9, **kw)
    a2, s2 = upd(c2, **kw)
    assert s2 is False and a2 <= a9 + 1e-12 and c2.last == c9.last
    if prev is not None and v < kc.GATE_SPEED and a2 < prev - 1e-12 and a2 != a9:
      assert prev - a2 <= kc.DOWN_STEP + 1e-12 and _margin(v, kw["long_accel"], ae) > kc.GATE_DOWN_MARGIN
    prev = a2


def test_mode2_band_comp_rises_rate_limited():
  c = K9Creep(V9_SHUDDER_FIX)
  out = [c._band_comp_limited(-0.6, a_t, 1.0) for a_t in [-0.05] * 5 + [-0.5] * 80 + [-0.05] * 3]
  assert out[:5] == [0.0] * 5
  assert np.all(np.diff(out[4:85]) <= kc.BC_RATE + 1e-12) and out[84] == pytest.approx(kc.COMP_DECEL)
  assert out[85] == 0.0  # falls (less braking) at once
  assert K9Creep(V9)._band_comp_limited(-0.6, -0.5, 1.0) == 0.0


# --- K9Creep object: exactly the frozen aeef67c6 one (new 1 = old 2) ------------------------------

def _state(c):
  return (c.latched, c.last, c.active, c.cb_w)


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
      assert old.cb_w == new.cb_w


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


# --- CarController: new 1/2 byte-identical to the frozen 1d0961da controller's 2/3 (2026-10-23 07:25 renumbering) ----

@pytest.mark.parametrize("seq", list(SEQS))
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0, 1, 2])
def test_new_1_2_identical_can_to_1d0961da_2_3(monkeypatch, prev_cc, mode, fingerprint, seq):
  prev = PREV_OF_NEW[mode]
  ref = SEQS[seq](Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), module=prev_cc,
                      **({} if prev is None else {"K9StopMode": prev})))
  monkeypatch.undo()
  new = SEQS[seq](rig(monkeypatch, mode, cp=make_cp(fingerprint=fingerprint)))
  assert len(new.can) > 400 and ref.can == new.can
  assert ref.scc14 == new.scc14
  if fingerprint == CAR.KIA_K9 and mode:
    assert new.cc.k9_creep.mode == KIND_OF_PREV[prev] and new.cc.k9_creep_mode == mode


@pytest.mark.parametrize("seed", range(6))
@pytest.mark.parametrize("new_menu", (1, 2))
def test_k9creep_object_identical_to_1d0961da(prev_cc, new_menu, seed):
  rng = random.Random(f"k9-stopmode-0725-{seed}")
  old, new = prev_cc.K9Creep(PREV_OF_NEW[new_menu]), K9Creep(kc.MENU[new_menu])
  cb_prev_o = cb_prev_n = 0.0
  for kw, i in _random_inputs(rng, 4000):
    ro, rn = old.update(**kw), new.update(**kw)
    assert ro == rn or (math.isnan(ro[0]) and math.isnan(rn[0]) and ro[1] == rn[1]), (i, kw)
    assert _state(old) == _state(new) or (math.isnan(old.last or 0.0) and math.isnan(new.last or 0.0))
    assert old.sent == new.sent or (math.isnan(old.sent or 0.0) and math.isnan(new.sent or 0.0))
    if i % 2 == 0 and old.active:
      cu, cl = float(np.clip(0.9 + rn[0] * 0.2, 0, 1.2)), float(np.clip(0.8 + rn[0] * 0.2, 0, 1.2))
      ju = rng.uniform(0.5, 5.0)
      bo = old.bands(ju, kw["v_ego"], cu, cl, cb_prev_o)
      bn = new.bands(ju, kw["v_ego"], cu, cl, cb_prev_n)
      assert bo == bn, i
      cb_prev_o, cb_prev_n = bo[1], bn[1]


def test_stored_values_are_not_migrated(monkeypatch):
  # 2026-10-23 07:25: no value migration. A stored 1 (v9 on every earlier menu) now runs v10, 2 runs v9 + shudder fix
  # (its meaning on the 99eb53f7 menu; v10 on 1d0961da), 3 runs v10 + band 0.2 (v9 + shudder fix on 1d0961da).
  for stored, kind in ((1, V10), (2, V9_SHUDDER_FIX), (3, V10_SHUDDER_FIX), (4, V10_SMOOTH)):
    r = rig(monkeypatch, stored)
    r.step(-0.5, LCS.pid, 3.0)
    assert r.cc.k9_creep_mode == stored and r.cc.k9_creep.mode == kind
    assert r.params.values["K9StopMode"] == stored  # the controller never writes the menu value
    monkeypatch.undo()


@pytest.mark.parametrize("value", INVALID)
def test_invalid_values_act_as_0(monkeypatch, old_cc, value):
  ref = mixed_sequence(old_rig(monkeypatch, old_cc, 0))
  monkeypatch.undo()
  r = mixed_sequence(rig(monkeypatch, value))
  assert r.can == ref.can and r.cc.k9_creep is None and r.cc.k9_creep_mode == 0


@pytest.mark.parametrize("stale", [1, 2, 3, 4, 5, 6])
@pytest.mark.parametrize("mode", [None, 1, 2, 3])
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
  assert run(True) == run(False) == run(True, K9Creep(V9)) == run(True, K9Creep(V9_SHUDDER_FIX))
  assert run(True) == run(True, K9Creep(V10)) == run(True, K9Creep(V10_SHUDDER_FIX)) == run(True, K9Creep(V10_SMOOTH))


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


# --- 2 versus 1 (v9) through the CarController -------------------------------------------------------------------

@pytest.mark.parametrize("seq", list(SEQS))
def test_mode2_versus_v9_through_carcontroller(monkeypatch, seq):
  run = SEQS[seq]
  rows = {}
  for mode in ALL:
    r = rig_kind(monkeypatch, mode)
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
  (o1, r1), (o2, r2) = rows[V9], rows[V9_SHUDDER_FIX]
  stronger = band_differ = 0
  for (v, (a1, s1)), (_, (a2, s2)) in zip(o1, o2, strict=True):
    assert a2 <= a1 + 1e-9  # never weaker than v9
    if s1 is not None:
      assert s2["StopReq"] == s1["StopReq"]  # StopReq timing unchanged
      assert s2["aReqValue"] <= s1["aReqValue"] + 1e-9
    if a2 != a1:
      assert v < kc.BAND_OFF_SPEED
      stronger += 1
  for f, ((v, _), c1, c2) in enumerate(zip(o1, r1.can, r2.can, strict=True)):
    if v >= kc.BAND_OFF_SPEED and all(x[0] >= kc.BAND_OFF_SPEED for x in o1[max(0, f - 100):f]):
      assert c1 == c2, f  # 1 s above 3 m/s: every message identical to v9
  for (f1, _, d1), (f2, _, d2) in zip(r1.scc14, r2.scc14, strict=True):
    assert f1 == f2
    for key in d1:
      if key not in ("ComfortBandUpper", "ComfortBandLower"):
        assert d2[key] == d1[key], key
    band_differ += (d1["ComfortBandUpper"], d1["ComfortBandLower"]) != (d2["ComfortBandUpper"], d2["ComfortBandLower"])
  assert band_differ > 0 or seq == "edge"
  assert stronger > 0 or seq == "edge"


def band_trace(monkeypatch, mode, profile):
  r = rig_kind(monkeypatch, mode)
  out = []
  for state, v in profile:
    r.step(-0.6, state, v, a_target=-0.4, a_ego=-0.3)
    j, c = r.cc.hyundai_jerk, r.cc.k9_creep
    out.append((v, float(j.cb_upper), float(j.cb_lower), None if c is None or c.cb_w is None else c.cb_w))
  return out


DECEL = ([(LCS.pid, 5.0)] * 100 + [(LCS.pid, 5.0 - 0.02 * i) for i in range(200)] +
         [(LCS.stopping, max(1.0 - 0.006 * i, 0.06)) for i in range(150)] + [(LCS.off, 0.0)] * 30)


def test_mode2_comfortband_values_and_ramp(monkeypatch):
  tr = band_trace(monkeypatch, V9_SHUDDER_FIX, DECEL)
  ws = [w for _, _, _, w in tr[:450]]
  assert all(w is not None for w in ws[2:])
  steps = np.abs(np.diff([w for w in ws[2:]]))
  assert np.all(steps <= kc.BAND_RAMP_STEP + 1e-9)  # no jump (per make_jerk call; it runs every 2nd frame)
  for i, (v, cu, cl, w) in enumerate(tr[:450]):
    if w is None:
      continue
    if 60 <= i < 100:  # steady 5 m/s: v9 band (0)
      assert w == 0.0 and cu == 0.0 and cl == 0.0
    if v <= kc.BAND_FULL_SPEED and all(x[0] <= kc.BAND_FULL_SPEED for x in tr[max(0, i - 50):i]):
      assert w == 1.0 and cu > 0.0 and cu - cl == pytest.approx(0.1, abs=1e-6)  # original band, upper-lower 0.1
  v9 = band_trace(monkeypatch, V9, DECEL)
  assert all(cu == 0.0 and cl == 0.0 for _, cu, cl, _ in v9[2:450])  # v9: 0 everywhere while applied
  assert tr[-1][1:3] == v9[-1][1:3]  # control off: original (0) in both


def test_live_mode_switch_within_half_second_and_band_does_not_jump(monkeypatch):
  r = rig(monkeypatch, 1)  # v10: band 0
  for _ in range(60):
    r.step(-0.5, LCS.pid, 3.5, a_target=-0.5)
  for _ in range(60):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
  assert r.cc.k9_creep.mode == V10 and r.cc.hyundai_jerk.cb_upper == 0.0
  r.params.values["K9StopMode"] = 2  # v9 + shudder fix: band restored
  seen, bands = None, []
  for i in range(80):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
    bands.append(float(r.cc.hyundai_jerk.cb_upper))
    if seen is None and r.cc.k9_creep.mode == V9_SHUDDER_FIX:
      seen = i
  assert seen is not None and seen <= 50  # read every 0.5 s
  assert np.all(np.abs(np.diff(bands)) <= kc.BAND_RAMP_STEP * 0.8 + 0.01)  # ramp, no jump to the original band
  assert bands[-1] > 0.0
  for value in (5, 6, float("nan"), -1):
    r.params.values["K9StopMode"] = 2
    for _ in range(51):
      r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
    assert r.cc.k9_creep.mode == V9_SHUDDER_FIX
    r.params.values["K9StopMode"] = value
    for _ in range(51):
      r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
    assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0


def test_live_mode_change_resets(monkeypatch):
  r = rig(monkeypatch, 2)
  for _ in range(60):
    r.step(-1.0, LCS.pid, 3.0)
  for i in range(60):
    r.step(-1.2, LCS.stopping, max(0.5 - 0.005 * i, 0.1))
  assert r.cc.k9_creep is not None and r.cc.k9_creep.mode == V9_SHUDDER_FIX and r.cc.k9_creep.last is not None
  r.params.values["K9StopMode"] = 1
  for _ in range(60):
    r.step(-1.2, LCS.stopping, 0.1)
  assert r.cc.k9_creep.mode == V10 and not r.cc.k9_creep.shudder_fix
  r.params.values["K9StopMode"] = 0
  out = [r.step(-1.2, LCS.stopping, 0.1) for _ in range(60)]
  assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0
  sent = [o for o in out[-10:] if o[1] is not None]
  assert sent and all(o[1]["StopReq"] == 1 and o[0] == pytest.approx(-1.2) for o in sent)


def test_setting_registered_single_selector_old_keys_deleted():
  import json
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  entry = next(p for p in catalog["params"] if p["name"] == "K9StopMode")
  assert (entry["min"], entry["max"], entry["default"], entry["unit"]) == (0, 4, 0, 1)
  assert [len(entry["options"][loc]) for loc in ("ko", "en", "zh")] == [5, 5, 5]
  assert not any(p["name"] in ("K9CreepMode", "K9CreepV9", "K9CreepV8", "K9StopTaper") for p in catalog["params"])
  keys = (ROOT / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
  assert '{"K9StopMode", {PERSISTENT, INT, "0"}},' in keys
  assert "K9CreepMode" not in keys and "K9CreepV9" not in keys and "K9StopTaper" not in keys
  assert not (ROOT / "opendbc_repo/opendbc/car/hyundai/k9_stop_taper.py").exists()
