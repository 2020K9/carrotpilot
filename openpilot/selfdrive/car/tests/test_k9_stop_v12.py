"""K9StopMode menu of 2026-10-25 (보노보노 00:36 "v12 원본 그대로 넣어놓기도 해보고, K9 맞춰 넣는 것도 v12k9으로" /
"브레이크 앞전꺼는 다 별로니 지워" / 00:39 "원본 그대로 수정하지 말고 올려라"): 0 base, 1 v12 original (the creep_fix v12 page
code unchanged in carcontroller.py), 2 v12k9 (k9_creep kind V12K9). The old 1 v10 / 2 v10 + light / 3 v11 are deleted.
Model-only harness with synthetic inputs: source logic and SCC12/SCC14 encoding, not K9 vehicle behaviour (실차 미검증).
2026-10-26 (보노보노 23:14 "v12그냥놓아두고 v13추가하는걸로"): 3 is now the v13 original (test_k9_stop_v13.py). Menu-set pins only:
MODES (1, 2, 3), settings max 3, 3 left the "other values act as 0" list (4, 7, 99, -1 stay), and the diff-vs-b6696b37
check also accepts the v13 page lines (their own pin is in test_k9_stop_v13.py).
2026-10-29 05:40 (보노보노 "지금 2번인 v12 원본은 메뉴에서 빼라"): the v12 original left the menu with its two page blocks, so the
page sha pin, the page-block-unchanged pin and the two v12-original behaviour tests are deleted with it (as the v13 / C1
pins were on 2026-10-28). Menu 2 is kind 14 and menu 4 is kind 15 (own pins: test_k9_stop_v12k9o.py / the replay);
MODES (1, 2, 3, 4), settings max 4 / 5 options, and the stored-value chain gets K9StopModeVer 6 (a stored 2 -> 0).
2026-10-28 (보노보노 01:06 "원본 그대로 푸쉬해라 v14"): 3 = v14 original instead of v13 (test_k9_stop_v13.py deleted, pin in
test_k9_stop_v14.py); 4 C1 / 5 C1+ (2026-10-27) gone. Menu-set pins only: the diff check accepts the v14 page lines instead of
the v13 ones, the stored-value migration pins for K9StopModeVer 4 (3 / 4 / 5 / 6 of the K9StopModeVer 3 menu -> 0).
2026-10-29 07:10 (보노보노 "3.v14제거후 4번을 3번으로"): the v14 original left the menu with its two page blocks as well, so
test_k9_stop_v14.py went with it; its two tests that are not about the v14 page code (the removed-code pin and the
K9StopModeVer 3 migration rule) moved here. Menu-set pins only: MODES (1, 2, 3), settings max 3 / 4 options, 4 is back in
the "acts as 0" lists, and the stored-value chain gets K9StopModeVer 7 (a stored 4 -> 3, a stored 3 -> 0).
Pin updated 2026-10-30 (보노보노 17:29 / 17:31): 2 = kind 16 격차수정, 4 = kind 14 (moved from 2), 5 = kind 17. Menu-set pins only:
MODES 1-5, settings max 5, 4 / 5 left the "acts as 0" lists (6 joined), and the stored-value chain gets K9StopModeVer 8
(a marker-7 2 -> 4, a 4 / 5 -> 0).
Pin updated 2026-10-25 evening (brake_v12k9_live_fix): test_v12k9_jerk_and_band expects the menu 2 ComfortBand (original band at
or below 2.5 m/s, linear to 0 at 3.0 m/s, 0 above) instead of 0 everywhere.
"""
import hashlib
import math
import random
import subprocess

import numpy as np
import pytest

from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V9, V12K9
from opendbc.car.hyundai.values import CAR
from openpilot.selfdrive.car import k9_stop_mode_migration as mig
from openpilot.selfdrive.car.tests.k9_rig import LCS, ROOT, CC_REL, Rig, edge_sequence, gentle_sequence, load_frozen_cc, make_cp, mixed_sequence

PREV = "b6696b37"  # carrot-wip before this change: K9StopMode 0 base, 1 v10, 2 v10 + light shudder fix, 3 v11 (kind 7)
PREV_V11 = 7
PAGE_BLOCK_1 = """\
      if self.CP.openpilotLongitudinalControl:
        lon_ac = float(CS.cp.vl["ESP12"]["LONG_ACCEL"]) if (CS.cp is not None and "ESP12" in CS.cp.vl and "LONG_ACCEL" in CS.cp.vl["ESP12"]) else float(CS.out.aEgo)
        eff_ac = float(min(lon_ac, CS.out.aEgo + 0.03))
        if stopping:
          esp_standstill = bool(CS.cp is not None and "TCS13" in CS.cp.vl and CS.cp.vl["TCS13"]["StandStill"] == 1)
          raw_standstill = bool(esp_standstill or CS.out.vEgoRaw < 0.05 or CS.out.vEgo < 0.05)
          self.standstill_count = getattr(self, "standstill_count", 0) + 1 if raw_standstill else 0
          real_standstill = bool(self.standstill_count >= 12)
          if real_standstill or actuators.aTarget <= -1.0 or (CS.out.vEgo > 0.14 and eff_ac > 0.04) or CS.out.aEgo > 0.06:
            self.stop_req_latched = True
          if not getattr(self, "stop_req_latched", False):
            stopping = False
            stop_feather = float(np.interp(CS.out.vEgo, [0.05, 0.11, 0.16, 0.25, 0.42], [-0.28, -0.29, -0.34, -0.42, -0.52]))
            min_decel = float(np.interp(CS.out.vEgo, [0.06, 0.12, 0.25, 0.42], [-0.04, -0.08, -0.14, -0.20]))
            a_boost = float(np.interp(eff_ac - min_decel, [0.0, 0.15], [0.0, -0.10]))
            target_accel = float(stop_feather + a_boost)
            prev_accel = float(getattr(self, "accel_feather_last", accel))
            down_step = 0.005 if ((CS.out.vEgo > 0.14 and eff_ac > -0.02) or prev_accel > stop_feather + 0.02) else 0.0
            up_step = float(np.interp(eff_ac - min_decel, [-0.06, 0.04], [0.0032, 0.0010])) if CS.out.vEgo > 0.12 else 0.0030
            accel = float(np.clip(target_accel, prev_accel - down_step, prev_accel + up_step))
            self.accel_feather_last = float(accel)
          else:
            prev_accel = float(getattr(self, "accel_feather_last", accel))
            accel = float(max(min(accel, -0.50), prev_accel - 0.0025)) if actuators.aTarget > -1.0 else float(min(accel, -0.50))
            self.accel_feather_last = float(accel)
        else:
          self.stop_req_latched = False
          self.standstill_count = 0
          if actuators.longControlState == LongCtrlState.pid and CS.out.vEgo < 1.5 and actuators.aTarget < -0.05:
            if actuators.aTarget > -0.80 and accel < actuators.aTarget - 0.14:
              blend = float(np.interp(CS.out.vEgo, [1.05, 1.50], [0.30, 0.0]))
              accel = float(accel * (1.0 - blend) + (actuators.aTarget - 0.14) * blend)
            creep_comp = float(np.interp(CS.out.vEgo, [0.22, 0.50, 0.92, 1.45], [-0.25, -0.22, -0.18, 0.0]))
            fade_by_target = float(np.interp(actuators.aTarget, [-0.20, -0.05], [1.0, 0.0]))
            fade_by_accel = float(np.interp(accel, [-0.80, -0.30], [0.25, 1.0]))
            accel = float(np.clip(accel + creep_comp * fade_by_target * fade_by_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
            prev_accel = float(getattr(self, "accel_feather_last", accel))
            if CS.out.vEgo < 1.00 and actuators.aTarget > -0.80:
              down_lim = 0.0 if eff_ac < min(actuators.aTarget - 0.04, -0.42) else 0.004
              up_lim = float(np.interp(eff_ac - actuators.aTarget, [-0.05, 0.10], [0.0035, 0.0008])) if CS.out.vEgo > 0.52 else 0.0040
              accel = float(np.clip(accel, prev_accel - down_lim, prev_accel + up_lim))
          self.accel_feather_last = float(accel)
"""
PAGE_BLOCK_2 = """\
        if CS.out.vEgo < 1.15:
          self.jerk_u = max(self.jerk_u, float(np.interp(CS.out.vEgo, [0.05, 0.08, 0.14, 0.38, 0.60, 0.92, 1.15], [0.7, 0.9, 1.6, 1.6, 1.3, 1.2, 0.8])))
          self.jerk_l = max(self.jerk_l, float(np.interp(CS.out.vEgo, [0.55, 0.75, 0.95, 1.15], [1.0, 1.3, 1.3, 1.0])))
        self.cb_upper = 0.0
        self.cb_lower = 0.0
"""


def _kw(mode):
  return {} if mode is None else {"K9StopMode": mode}


@pytest.fixture(scope="module")
def prev_cc(tmp_path_factory):
  return load_frozen_cc(PREV, tmp_path_factory.mktemp("cc_b6696b37"), "cc_b6696b37")


# --- menu ------------------------------------------------------------------------------------------------------------
def test_menu():
  # 2026-10-29 04:48 slot swap: kind 13 (bono1739, his own stop table) is menu 1. 05:40: the v12 original left the menu
  # with its code, 2 = kind 14 (his table improved), 4 = kind 15 (his table + the 1 s hold); 3 = v14 original.
  # 07:10 (보노보노 "3.v14제거후 4번을 3번으로"): the v14 original left the menu with its code too and kind 15 moved 4 -> 3,
  # so every menu value is a K9Creep kind now and 4 is no longer a menu value.
  # 2026-10-30 (17:29 / 17:31): 2 = kind 16 (격차수정), 4 = kind 14 (moved from 2), 5 = kind 17 (격차수정 + harder below 1 km/h).
  assert kc.MENU == {1: kc.V12K9O, 2: kc.V12K9OG, 3: kc.V12K9OH, 4: kc.V12K9OS, 5: kc.V12K9OGR} and kc.MODES == (1, 2, 3, 4, 5)
  assert kc.KINDS == (V9, V12K9, kc.V12K9O, kc.V12K9OS, kc.V12K9OH, kc.V12K9OG, kc.V12K9OGR) == (1, 8, 13, 14, 15, 16, 17)
  assert carcontroller.K9_CREEP_MENU is kc.MENU
  for gone in ("V10", "V10_LIGHT", "V11", "LIGHT_MIN_DECEL", "V10_FEATHER_V", "V11_FEATHER_V", "V12_ORIG_MENU"):
    assert not hasattr(kc, gone), gone
  for gone in ("V14_ORIG_MENU", "V13_ORIG_MENU"):  # 2026-10-29 07:10 / 2026-10-28
    assert not hasattr(kc, gone) and not hasattr(carcontroller, gone), gone


def test_carcontroller_diff_is_menu_wiring_only():
  """Every line added to carcontroller.py since PREV is one of the listed menu-wiring lines. Both creep_fix page blocks
  left the tree with their menu entry (the v12 original 2026-10-29 05:40, the v14 original 07:10), so there is no pasted
  page code left to allow here - the pin is only tighter than the page-code version it replaces."""
  d = subprocess.run(["git", "diff", "-U0", PREV, "--", CC_REL], cwd=ROOT, capture_output=True, text=True, check=True).stdout
  added = [l[1:] for l in d.splitlines() if l.startswith("+") and not l.startswith("+++") and l[1:].strip()]
  assert len(added) > 4
  for l in added:  # wiring only: comments, the K9Creep band call arguments
    s = l.strip()
    assert s.startswith("#") or "cb_prev" in s or "creep_obj" in s, l


def test_v13_and_v14_page_code_gone():
  """Moved here from test_k9_stop_v14.py when that file went with the v14 page code (2026-10-29 07:10, 보노보노
  "3.v14제거후 4번을 3번으로"). The v13 original had gone on 2026-10-28 and the v12 original on 2026-10-29 05:40; none of
  the three page blocks, nor the page state they kept on the controller, is in the tree."""
  src = (ROOT / CC_REL).read_text(encoding="utf-8")
  for gone in ("v13_orig", "stop_latch_frames", "v12_orig", "v14_orig", "stop_req_latched", "standstill_count",
               "accel_feather_last", "low_v_crawl_count", "stop_hold_count"):
    assert gone not in src, gone
  for f in ("test_k9_stop_v13.py", "test_k9_stop_c1_menu45.py", "test_k9_stop_v14.py"):
    assert not (ROOT / "openpilot/selfdrive/car/tests" / f).exists(), f


# --- 0 / other values: unchanged -------------------------------------------------------------------------------------
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0, 6, 7, 99, -1])  # 3 left 2026-10-26 (v13 original); 4 / 5 are menu values since 2026-10-30
def test_base_and_other_values_identical_to_prev_0(monkeypatch, prev_cc, mode, fingerprint):
  for seq in (mixed_sequence, edge_sequence, gentle_sequence):
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), module=prev_cc, K9StopMode=0))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), **_kw(mode)))
    monkeypatch.undo()
    assert len(new.can) > 400 and ref.can == new.can and ref.scc14 == new.scc14
    assert new.cc.k9_creep is None


@pytest.mark.parametrize("mode", [1, 2, 3])
def test_other_cars_never_run_the_menu(monkeypatch, prev_cc, mode):
  for seq in (mixed_sequence, edge_sequence):
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), module=prev_cc, K9StopMode=0))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), K9StopMode=mode))
    monkeypatch.undo()
    assert ref.can == new.can and new.cc.k9_creep is None


# --- 2 v12k9 ---------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("seed", range(6))
def test_v12k9_stop_phase_is_the_old_v11(prev_cc, monkeypatch, seed):
  """v12k9 = the old v11 with the v11 pid re-press guard replaced by the v12 two-way rate limit: with both switched off,
  kind 8 is float-identical to b6696b37 kind 7 (v11), state too (stop phase, latch, creep compensation).
  2026-10-26 (brake_0241, zone approved 08:54): kind 8 now differs from v11 inside the zone by design, so the identity is pinned
  on the zone reference (c.ref / c.ref_out, the 66ca01d5 kind 8 without the band); outside the zone kind 8 sends the same StopReq
  and an accel within [reference - V12K9_BAND_COMP, reference]."""
  monkeypatch.setattr(kc, "V12_RATE_SPEED", -1.0)
  monkeypatch.setitem(prev_cc.K9Creep._update_v11.__globals__, "V11_GUARD_SPEED", -1.0)
  rng = random.Random(f"k9-v12k9-{seed}")
  old, new = prev_cc.K9Creep(PREV_V11), K9Creep(V12K9)
  assert old.__class__ is not K9Creep
  for kw, i in _random_inputs(rng, 4000):
    ro, rn = old.update(**kw), new.update(**kw)
    rr = (new.ref_out, new.ref.latched)
    assert ro[0] == rr[0] or (math.isnan(ro[0]) and math.isnan(rr[0])), (i, kw)
    assert old.latched == rr[1], i
    lead_in = (kw["lead_visible"] or kw["lead_distance"] != 0.0) and kw["lead_distance"] <= kc.V12K9_ZONE_LEAD
    if kw["stopping"] and kw["v_ego"] < kc.V12K9_ZONE_SPEED and not lead_in:
      continue
    assert ro[1] == rn[1] and (old.latched, old.active) == (new.latched, new.active), (i, kw)
    if not (math.isnan(ro[0]) or math.isnan(rn[0])):
      assert ro[0] - kc.V12K9_BAND_COMP - 1e-12 <= rn[0] <= ro[0] + 1e-12, (i, kw, ro, rn)


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


def _pid(c, prev_sent, accel, v, a_target, a_ego, lead=(False, 0.0)):
  c.last = prev_sent
  return c.update(accel=accel, a_target=a_target, stopping=False, pid=True, v_ego=v, v_ego_raw=v, a_ego=a_ego,
                  long_accel=a_ego, esp_standstill=False, long_active=True, override=False, gas=False, brake=False,
                  can_valid=True, lead_visible=lead[0], lead_distance=lead[1])[0]


def test_v12k9_pid_rate_limit():
  c = K9Creep(V12K9)
  _pid(c, None, -0.4, 0.9, -0.4, -0.4)
  assert c.last is not None
  # brake more: at most 0.004 per frame
  a = _pid(c, -0.30, -0.60, 0.9, -0.5, -0.30)
  assert a == pytest.approx(-0.304, abs=1e-9)
  # already decelerating hard enough: no fall at all
  assert _pid(c, -0.30, -0.60, 0.9, -0.3, -0.50) == pytest.approx(-0.30, abs=1e-9)
  # lead within 3 m: no fall limit
  assert _pid(c, -0.30, -0.60, 0.9, -0.5, -0.30, lead=(True, 2.0)) <= -0.59
  # release above 0.52 m/s: limited by the page table, below it (page 0.0040) not used on the K9
  # (brake_v12k9_fix: only while eff_ac >= aTarget - 0.02, so the car here brakes less than planned: aEgo -0.25)
  assert _pid(c, -0.50, 0.0, 0.9, -0.30, -0.25) <= -0.50 + 0.0035 + 1e-9
  assert _pid(c, -0.50, 0.0, 0.4, -0.30, -0.35) > -0.49
  # at or above 1.0 m/s, or a plan below -0.80: not limited
  assert _pid(c, -0.30, -0.60, 1.0, -0.5, -0.30) <= -0.59
  assert _pid(c, -0.30, -1.20, 0.9, -0.9, -0.30) <= -1.0


def test_v12k9_jerk_and_band():
  # pin updated 2026-10-25 brake_v12k9_live_fix: the band is the original one at or below 2.5 m/s, linear to 0 at 3.0 m/s,
  # 0 at or above 3.0 m/s (was 0 everywhere); the jerk values are unchanged
  c = K9Creep(V12K9)
  for v in (0.05, 0.1, 0.3, 0.6, 0.9, 1.1, 1.2, 2.0, 2.5, 2.75, 3.0, 3.5, 8.0):
    ju, cu, cl = c.bands(0.5, v, 0.9, 0.8, 0.0, 0.0)
    want = max(0.5, float(np.interp(v, kc.V12_JERK_U_BP, kc.V12_JERK_U_V))) if v < 1.15 else 0.5
    w = float(np.interp(v, [2.5, 3.0], [1.0, 0.0]))
    assert ju == pytest.approx(want) and cu == pytest.approx(0.9 * w) and cl == pytest.approx(0.8 * w)
    jl = c.jerk_l_floor(1.0, v)
    assert jl == pytest.approx(max(1.0, float(np.interp(v, kc.V12_JERK_L_BP, kc.V12_JERK_L_V))) if v < 1.15 else 1.0)
  assert kc.V12_JERK_U_V == [0.7, 0.9, 1.6, 1.6, 1.3, 1.2, 0.8] and kc.V12_JERK_L_V == [1.0, 1.3, 1.3, 1.0]
  assert K9Creep(V9).jerk_l_floor(0.7, 0.8) == 0.7


@pytest.mark.parametrize("seq", [mixed_sequence, edge_sequence, gentle_sequence])
@pytest.mark.parametrize("mode", [1, 2, 3])
def test_menu_sequences_run(monkeypatch, seq, mode):
  r = seq(Rig(monkeypatch, K9StopMode=mode))
  assert len(r.can) > 400
  assert r.cc.k9_creep is not None and r.cc.k9_creep.mode == kc.MENU[mode]  # 1 / 2 / 3 are all K9Creep kinds since 07:10


def test_live_switch_resets(monkeypatch):
  r = Rig(monkeypatch, K9StopMode=1)
  for value in (1, 2, 0, 1, 2, 4, 3, 4):
    r.params.values["K9StopMode"] = value
    for i in range(80):
      r.step(-0.6, LCS.pid, max(2.0 - 0.02 * i, 0.4), a_target=-0.4, a_ego=-0.3)
    # 2026-10-29 07:10: 1 / 2 / 3 are K9Creep kinds 13 / 14 / 15 and 0 is base; 4 left the menu with the v14 original,
    # so it is read as 0 again (the same inputs as before, one of them now an off-menu value)
    # 2026-10-30: 1 / 2 / 3 / 4 / 5 are kinds 13 / 16 / 15 / 14 / 17, so the 4 is a menu value again (kind 14)
    want = value if value in kc.MENU else 0
    assert r.cc.k9_creep_mode == want
    assert (r.cc.k9_creep is not None) is (want in kc.MENU)
    if want in kc.MENU:
      assert r.cc.k9_creep.mode == kc.MENU[want]


# --- stored-value migration ------------------------------------------------------------------------------------------
class FakeParams:
  def __init__(self, **v):
    self.v = dict(v)

  def get(self, k):
    return self.v.get(k)

  def put_int(self, k, x):
    self.v[k] = int(x)


def test_migration_tables():
  assert mig.STOP_MODE_VER == 8  # 7 until 2026-10-30, 6 until 2026-10-29 07:10, 5 until 05:40, 4 until the 04:48 slot swap, 3 until 2026-10-28
  assert mig.MENU_MIGRATION_V4 == {0: 0, 1: 1, 2: 2, 3: 0, 4: 0, 5: 0, 6: 0}
  assert mig.MENU_MIGRATION_V3 == {0: 0, 1: 2, 2: 2, 3: 2}
  assert mig.MENU_MIGRATION_FROM_V1 == {0: 0, 1: 2, 2: 2, 3: 2, 4: 2, 5: 2, 6: 2}
  assert mig.SWAP_V5 == {0: 0, 1: 2, 2: 1, 3: 3} and mig.SWAP_PENDING_BASE == 50  # 2026-10-29 04:48: 1 <-> 2
  assert mig.DROP_V6 == {0: 0, 1: 1, 2: 0, 3: 3}  # 2026-10-29 05:40: the v12 original (2) is off the menu -> 0
  assert all(mig.DROP_V6.get(v, v) == v for v in mig.DROP_V6.values())  # idempotent: no pending marker needed
  # 2026-10-29 07:10: the v14 original (3) is off the menu and menu 4 moved into its slot
  assert mig.MOVE_V7 == {0: 0, 1: 1, 2: 2, 3: 0, 4: 3} and mig.MOVE_PENDING_BASE == 60
  assert not all(mig.MOVE_V7.get(v, v) == v for v in mig.MOVE_V7.values())  # NOT idempotent -> it needs the pending marker
  assert set(mig.MOVE_V7.values()) <= {0, *kc.MENU}  # only base or a menu that exists
  assert not (set(range(mig.SWAP_PENDING_BASE, mig.SWAP_PENDING_BASE + 4))
              & set(range(mig.MOVE_PENDING_BASE, mig.MOVE_PENDING_BASE + 5)))  # a marker names one pending rule only
  assert 4 not in set(mig.DROP_V6.values()) | set(mig.MOVE_V7.values())  # 4 was off the 07:10 menu (kind 14 since 2026-10-30, only via MOVE_V8)
  old_menu = set(mig.MENU_MIGRATION_V3.values()) | set(mig.MENU_MIGRATION_FROM_V1.values())
  assert old_menu <= {0, 2}  # the tables above are in K9StopModeVer 4 numbering, before the swap
  assert {mig.MOVE_V7[mig.DROP_V6[mig.SWAP_V5[v]]] for v in old_menu} <= {0, *kc.MENU}  # only base or a menu that still exists
  assert set(mig.MENU_MIGRATION_V4.values()) <= {0, 1, 2}  # never the then-new 3 (the v14 original was picked by hand)
  # the v14 original kept its number 3 through the 04:48 swap and the 05:40 drop, and leaves the menu as 0 on 07:10
  assert mig.SWAP_V5[3] == 3 and mig.DROP_V6[3] == 3 and mig.MOVE_V7[3] == 0
  for t in (mig.MENU_MIGRATION_V4, mig.MENU_MIGRATION_V3, mig.MENU_MIGRATION_FROM_V1):
    assert all(t.get(v, v) == v for v in t.values())  # repeating an older table changes nothing (power loss before the swap)
  # 2026-10-30 (보노보노 17:29 "이전에하던개선은 4번 / 2번은 격차만큼조정한거", 17:31 "...그걸 5번"): the end-window improvement
  # (kind 14) moves 2 -> 4; a 4 / 5 that acted as 0 on the 0-3 menu becomes 0, so 격차수정 (2) and its ramp (5) only by hand
  assert mig.MOVE_V8 == {0: 0, 1: 1, 2: 4, 3: 3, 4: 0, 5: 0} and mig.MOVE8_PENDING_BASE == 70
  assert not all(mig.MOVE_V8.get(v, v) == v for v in mig.MOVE_V8.values())  # NOT idempotent -> pending marker 70 + value
  assert set(mig.MOVE_V8.values()) <= {0, *kc.MENU} and not ({2, 5} & set(mig.MOVE_V8.values()))
  assert not (set(range(mig.MOVE8_PENDING_BASE, mig.MOVE8_PENDING_BASE + 6))
              & (set(range(mig.SWAP_PENDING_BASE, mig.SWAP_PENDING_BASE + 4)) | set(range(mig.MOVE_PENDING_BASE, mig.MOVE_PENDING_BASE + 5))))
  assert {mig.MOVE_V8[mig.MOVE_V7[v]] for v in mig.MOVE_V7} <= {0, *kc.MENU}


@pytest.mark.parametrize("ver", [7, 6, 5, 4, 3, 2, None, 1, 9])  # 6 added 2026-10-29 07:10, 7 on 2026-10-30; 9 = unknown marker (8 is current)
@pytest.mark.parametrize("old", [None, 0, 1, 2, 3, 4, 5, 6, 7, 99, -1, "x"])
def test_migration(ver, old):
  table = {} if ver in (4, 5, 6, 7) else (mig.MENU_MIGRATION_V4 if ver == 3 else (mig.MENU_MIGRATION_V3 if ver == 2 else mig.MENU_MIGRATION_FROM_V1))
  p = FakeParams(**({} if old is None else {"K9StopMode": old}), **({} if ver is None else {"K9StopModeVer": ver}))
  changed = mig.migrate_k9_stop_mode(p)
  if isinstance(old, int):
    mid = table.get(old, old)  # the older menu first, then the 04:48 slot swap, the 05:40 drop and the 07:10 move
    if ver == 7:  # 2026-10-30: a marker-7 device is past every older step, only the MOVE_V8 move below
      sw = dr = want = mid
    elif ver in (5, 6) or mid in mig.SWAP_V5:  # the marker-4 gate: a value outside the swap table stops the chain there
      sw = mid if ver in (5, 6) else mig.SWAP_V5.get(mid, mid)  # a marker-5 / 6 device is already past the 04:48 swap
      dr = sw if ver == 6 else mig.DROP_V6.get(sw, sw)  # a marker-6 device is already past the 05:40 drop
      want = dr if (dr == 4 and ver != 6) else mig.MOVE_V7.get(dr, dr)  # 4 was a menu value on the ver-6 numbering only
    else:
      sw = dr = want = mid
    w8 = mig.MOVE_V8.get(want, want)  # 2026-10-30: 2 -> 4 (kind 14) and a 4 / 5 that acted as 0 -> 0, every marker
  else:
    mid = sw = dr = want = w8 = old
  wrote = isinstance(old, int) and (mid != old or sw != mid or dr != sw or want != dr or w8 != want)  # the chain can write more than once
  assert p.v.get("K9StopMode") == w8 and p.v["K9StopModeVer"] == mig.STOP_MODE_VER and changed == wrote
  assert mig.migrate_k9_stop_mode(p) is False and p.v.get("K9StopMode") == w8  # once only


def test_migration_after_ver8_never_touches():  # marker 8 since 2026-10-30 (marker 7 since 2026-10-29 07:10; older devices migrate once more)
  for v in (0, 1, 2, 3, 4, 5, 9):
    p = FakeParams(K9StopMode=v, K9StopModeVer=mig.STOP_MODE_VER)
    assert mig.migrate_k9_stop_mode(p) is False and p.v["K9StopMode"] == v


def test_migration_swap_1_2_is_crash_safe():
  """2026-10-29 04:48 (the driver: his own stop table must be menu 1): a marker-4 device swaps 1 <-> 2 once. The swap
  undoes itself if it runs twice, so the pre-swap value is written into the marker first (SWAP_PENDING_BASE + value):
  a power loss before or after the value write ends with the same value on the next start."""
  ver = mig.STOP_MODE_VER
  for old, swapped in ((0, 0), (1, 2), (2, 1), (3, 3)):
    want = mig.MOVE_V8[mig.MOVE_V7[mig.DROP_V6[swapped]]]  # the 05:40 drop, the 07:10 and 2026-10-30 moves run right after the swap
    p = FakeParams(K9StopMode=old, K9StopModeVer=4)
    assert mig.migrate_k9_stop_mode(p) is (want != old) and p.v["K9StopMode"] == want and p.v["K9StopModeVer"] == ver
    assert mig.migrate_k9_stop_mode(p) is False and p.v["K9StopMode"] == want  # once only
    q = FakeParams(K9StopMode=old, K9StopModeVer=mig.SWAP_PENDING_BASE + old)  # power loss before the value was rewritten
    assert mig.migrate_k9_stop_mode(q) is (want != old) and q.v["K9StopMode"] == want and q.v["K9StopModeVer"] == ver
    r = FakeParams(K9StopMode=swapped, K9StopModeVer=mig.SWAP_PENDING_BASE + old)  # power loss after the value write
    mig.migrate_k9_stop_mode(r)
    assert r.v["K9StopMode"] == want and r.v["K9StopModeVer"] == ver


def test_migration_drop_v12_original_from_ver5():
  """2026-10-29 05:40 (the driver: take the v12 original off the menu): a marker-5 device keeps 0 / 1 / 3 and moves a
  stored 2 (the v12 original, now gone) to 0. The table is idempotent, so it needs no pending marker: running it twice
  lands on the same value either way. Renumbered 2026-10-29 07:10: the same pass now also runs the 4 -> 3 / 3 -> 0 move,
  so the stored values below (unchanged) end on MOVE_V7 of the 05:40 result; DROP_V6 itself is untouched."""
  for old in (0, 1, 2, 3, 4, 9):
    mid = mig.DROP_V6.get(old, old)  # after the 05:40 drop
    # after the 07:10 move; a marker-5 device's stored 4 was out of range on its menu (0-3), so the move leaves it alone
    want = mid if mid == 4 else mig.MOVE_V7.get(mid, mid)
    want = mig.MOVE_V8.get(want, want)  # 2026-10-30: then 2 -> 4 and the out-of-range 4 -> 0
    p = FakeParams(K9StopMode=old, K9StopModeVer=5)
    assert mig.migrate_k9_stop_mode(p) is (want != old) and p.v["K9StopMode"] == want
    assert p.v["K9StopModeVer"] == mig.STOP_MODE_VER
    assert mig.migrate_k9_stop_mode(p) is False and p.v["K9StopMode"] == want  # once only
    q = FakeParams(K9StopMode=mid, K9StopModeVer=5)  # power loss after the drop wrote, before the marker: same value again
    mig.migrate_k9_stop_mode(q)
    assert q.v["K9StopMode"] == want and q.v["K9StopModeVer"] == mig.STOP_MODE_VER


def test_migration_move_menu4_to_3_and_drop_v14_is_crash_safe():
  """2026-10-29 07:10 (보노보노 "3.v14제거후 4번을 3번으로"): a marker-6 device moves a stored 4 to 3 (the same method, the
  free number) and a stored 3 (the v14 original, gone from the tree) to 0. Running the table twice would take the new 3
  to 0, so unlike the 05:40 drop it is NOT idempotent: the pre-move value is written into the marker first
  (MOVE_PENDING_BASE + value), and a power loss before or after the value write ends on the same value next start."""
  ver = mig.STOP_MODE_VER
  for old, want7 in ((0, 0), (1, 1), (2, 2), (3, 0), (4, 3), (9, 9)):
    assert mig.MOVE_V7.get(old, old) == want7
    want = mig.MOVE_V8.get(want7, want7)  # 2026-10-30: the same pass then moves 2 -> 4 (kind 14, the same method)
    p = FakeParams(K9StopMode=old, K9StopModeVer=6)
    assert mig.migrate_k9_stop_mode(p) is (want != old) and p.v["K9StopMode"] == want and p.v["K9StopModeVer"] == ver
    assert mig.migrate_k9_stop_mode(p) is False and p.v["K9StopMode"] == want  # once only
    if old not in mig.MOVE_V7:  # outside the table: left as stored (it acted as 0 and still acts as 0), no pending marker
      continue
    q = FakeParams(K9StopMode=old, K9StopModeVer=mig.MOVE_PENDING_BASE + old)  # power loss before the value was rewritten
    assert mig.migrate_k9_stop_mode(q) is (want != old) and q.v["K9StopMode"] == want and q.v["K9StopModeVer"] == ver
    r = FakeParams(K9StopMode=want7, K9StopModeVer=mig.MOVE_PENDING_BASE + old)  # power loss after the value write
    mig.migrate_k9_stop_mode(r)
    assert r.v["K9StopMode"] == want and r.v["K9StopModeVer"] == ver
  # 4 was a menu value on the ver-6 numbering only: on an older marker a stored 4 was out of range and acted as 0, so the
  # 07:10 move leaves it alone; since 2026-10-30 4 is kind 14, so the same pass writes the 0 it acted as (MOVE_V8).
  # (Marker 3 and no marker are excluded: their own tables already map a stored 4, C1 / v9.)
  for mk in (2, 4, 5):
    p = FakeParams(K9StopMode=4, K9StopModeVer=mk)
    assert mig.migrate_k9_stop_mode(p) is True and p.v["K9StopMode"] == 0 and p.v["K9StopModeVer"] == ver
  # the marker alone decides the result: repeating the pass any number of times lands on the same value
  for old in (2, 3, 4):
    p = FakeParams(K9StopMode=old, K9StopModeVer=6)
    for _ in range(4):
      mig.migrate_k9_stop_mode(p)
    assert p.v["K9StopMode"] == mig.MOVE_V8[mig.MOVE_V7[old]] and p.v["K9StopModeVer"] == ver


def test_migration_old_v13_c1_values_go_to_0():
  """Moved here from test_k9_stop_v14.py (2026-10-29 07:10, numbers updated only). 2026-10-28: a K9StopModeVer 3 device
  (the 0-5 menu of d76f7ecc) with 3 v13 / 4 C1 / 5 C1+ (6 = the never-pushed v13k9) goes to 0 once; an old 3 v13 was never
  carried to the 3 v14 that followed, and the v14 original is itself off the menu since 2026-10-29 07:10. 0 / 1 / 2 stay."""
  assert mig.STOP_MODE_VER == 8  # 7 until 2026-10-30, 6 until 2026-10-29 07:10, 5 until 05:40, 4 until the 04:48 slot swap
  # table first, then the 1 <-> 2 swap, the 05:40 drop of the v12 original and the 07:10 move: a ver-3 stored 1 was the
  # v12 original, so it ends on 0 (base) instead of the slot it used to own.
  for old, want in ((0, 0), (1, 0), (2, 1), (3, 0), (4, 0), (5, 0), (6, 0)):
    p = FakeParams(K9StopMode=old, K9StopModeVer=3)
    assert mig.migrate_k9_stop_mode(p) is (want != old) and p.v["K9StopMode"] == want and p.v["K9StopModeVer"] == 8
    assert mig.migrate_k9_stop_mode(p) is False and p.v["K9StopMode"] == want  # once only
  p = FakeParams(K9StopMode=3, K9StopModeVer=4)  # the v14 original kept its number until 07:10 took it off the menu
  assert mig.migrate_k9_stop_mode(p) is True and p.v["K9StopMode"] == 0 and p.v["K9StopModeVer"] == 8
  p = FakeParams(K9StopMode=3, K9StopModeVer=5)  # picked on the 04:48 menu: v14, now base
  assert mig.migrate_k9_stop_mode(p) is True and p.v["K9StopMode"] == 0
  p = FakeParams(K9StopMode=3, K9StopModeVer=2)  # the 2026-10-24 evening menu: 3 was v11 -> 2 -> 1 after the swap, never v14
  assert mig.migrate_k9_stop_mode(p) is True and p.v["K9StopMode"] == 1
  p = FakeParams(K9StopMode=3)  # the 0-6 menu
  assert mig.migrate_k9_stop_mode(p) is True and p.v["K9StopMode"] == 1
  tables = set(mig.MENU_MIGRATION_V4.values()) | set(mig.MENU_MIGRATION_V3.values()) | set(mig.MENU_MIGRATION_FROM_V1.values())
  assert 3 not in tables  # the new 3 (kind 15) only ever comes from a stored 4 through MOVE_V7


def test_migration_menu5_move_from_ver7_is_crash_safe():
  """2026-10-30 (보노보노 17:29 "이전에하던개선은 4번 / 2번은 격차만큼조정한거", 17:31 "...그걸 5번"): a marker-7 device moves a
  stored 2 (the end-window improvement, kind 14) to 4 and a 4 / 5 that acted as 0 on the 0-3 menu to 0; 0 / 1 / 3 stay.
  Running the table twice would take the new 4 to 0, so the pre-move value is written into the marker first
  (MOVE8_PENDING_BASE + value), and a power loss before or after the value write ends on the same value next start."""
  ver = mig.STOP_MODE_VER
  assert kc.MENU[mig.MOVE_V8[2]] == kc.V12K9OS == 14  # the old 2 runs the same kind 14 under its new number
  for old, want in ((0, 0), (1, 1), (2, 4), (3, 3), (4, 0), (5, 0), (6, 6), (9, 9)):
    assert mig.MOVE_V8.get(old, old) == want
    p = FakeParams(K9StopMode=old, K9StopModeVer=7)
    assert mig.migrate_k9_stop_mode(p) is (want != old) and p.v["K9StopMode"] == want and p.v["K9StopModeVer"] == ver
    assert mig.migrate_k9_stop_mode(p) is False and p.v["K9StopMode"] == want  # once only
    if old not in mig.MOVE_V8:  # outside the table: left as stored (it acted as 0 and still acts as 0), no pending marker
      continue
    q = FakeParams(K9StopMode=old, K9StopModeVer=mig.MOVE8_PENDING_BASE + old)  # power loss before the value was rewritten
    assert mig.migrate_k9_stop_mode(q) is (want != old) and q.v["K9StopMode"] == want and q.v["K9StopModeVer"] == ver
    r = FakeParams(K9StopMode=want, K9StopModeVer=mig.MOVE8_PENDING_BASE + old)  # power loss after the value write
    for _ in range(4):  # the marker alone decides the result: repeating the pass lands on the same value
      mig.migrate_k9_stop_mode(r)
    assert r.v["K9StopMode"] == want and r.v["K9StopModeVer"] == ver


def test_migration_in_manager_init():
  src = (ROOT / "openpilot/system/manager/manager.py").read_text(encoding="utf-8")
  assert "migrate_k9_stop_mode" in src


# --- settings json ---------------------------------------------------------------------------------------------------
def test_settings_entry():
  import json
  d = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  ents = [e for e in d["params"] if e.get("name") == "K9StopMode"]
  assert len(ents) == 1
  e = ents[0]
  assert e["min"] == 0 and e["max"] == 5 and e["default"] == 0  # max 5 since 2026-10-30 (2 / 4 / 5 = kinds 16 / 14 / 17)
