"""K9StopMode 3 = creep_fix v13 original (2026-10-26, 보노보노 23:13 "v13나왔다 ..." / 23:14 "v12그냥놓아두고 v13추가하는걸로").

The page https://leegrt.org/oppo/creep_fix/ (copy of 2026-10-26 23:17, cf.html sha256
50342d40453617d90dba9fec3687272e58a3ca6f9ba9547e5df7b7444ce36839) publishes v13 as a python one-liner: pat1 replaces
everything from the casper_ev line to the `if self.frame % 2 == 0 ...` line with PAGE_NEW1 ([변경 1]), pat2 replaces the
make_jerk jerk_l line through the cb_lower line with PAGE_NEW2 ([변경 2]). PAGE_NEW1 / PAGE_NEW2 below are the page's
new1 / rep2 strings, decoded from the page's <pre> commands, its JS new1Code / new2Code and its diff preview (all five
copies identical). Menu 3 = those lines unchanged (two more leading spaces, under `if self.k9_v13_orig:` / `if v13_orig:`);
1 v12 original and 2 v12k9 unchanged. Reference: the page one-liner applied to the f01c98fc carcontroller.py (the tree
before this change), run as menu 0 = the page patch exactly as a page user would get it on this fork.
Model-only harness with synthetic inputs: source logic and SCC12/SCC14 encoding, not K9 vehicle behaviour (실차 미검증).
2026-10-27 (보노보노 "그럼 4.5에넣어야지"): 4 = C1, 5 = C1+ added; the v13 original stays 3. Menu-number pins only: MODES (1-5),
the "acts as 0" value 4 -> 6, settings max 5.
"""
import hashlib
import re
import subprocess

import pytest

from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.values import CAR
from openpilot.selfdrive.car import k9_stop_mode_migration as mig
from openpilot.selfdrive.car.tests import k9_rig
from openpilot.selfdrive.car.tests.k9_rig import (LCS, ROOT, CC_REL, Rig, edge_sequence, gentle_sequence, load_frozen_cc, make_cp,
                                                  mixed_sequence)

PREV = "f01c98fc"  # carrot-wip before this change: 0 base, 1 v12 original, 2 v12k9 (3 acted as 0)
PAGE_NEW1 = """\
      casper_ev = self.CP.carFingerprint == CAR.HYUNDAI_CASPER_EV
      if self.CP.openpilotLongitudinalControl:
        lon_ac = float(CS.cp.vl["ESP12"]["LONG_ACCEL"]) if (CS.cp is not None and "ESP12" in CS.cp.vl and "LONG_ACCEL" in CS.cp.vl["ESP12"]) else float(CS.out.aEgo)
        eff_ac = float(min(lon_ac, CS.out.aEgo + 0.03))
        if stopping:
          esp_standstill = bool(CS.cp is not None and "TCS13" in CS.cp.vl and CS.cp.vl["TCS13"]["StandStill"] == 1)
          whl_standstill = bool(max(CS.out.wheelSpeeds.fl, CS.out.wheelSpeeds.fr, CS.out.wheelSpeeds.rl, CS.out.wheelSpeeds.rr) < 0.03)
          raw_standstill = bool(esp_standstill or CS.out.vEgoRaw < 0.05 or CS.out.vEgo < 0.05)
          self.standstill_count = getattr(self, "standstill_count", 0) + 1 if raw_standstill else 0
          self.whl_standstill_count = getattr(self, "whl_standstill_count", 0) + 1 if whl_standstill else 0
          self.low_v_crawl_count = getattr(self, "low_v_crawl_count", 0) + 1 if (CS.out.vEgo < 0.28 and not raw_standstill and not whl_standstill) else 0
          real_standstill = bool(self.standstill_count >= 12 or self.whl_standstill_count >= 28)
          if real_standstill or actuators.aTarget <= -1.0 or (CS.out.vEgo > 0.14 and eff_ac > 0.04) or CS.out.aEgo > 0.06:
            self.stop_req_latched = True
          if not getattr(self, "stop_req_latched", False):
            self.stop_latch_frames = 0
            stopping = False
            stop_feather = float(np.interp(CS.out.vEgo, [0.05, 0.14, 0.20, 0.28, 0.42], [-0.28, -0.29, -0.34, -0.39, -0.44]))
            min_decel = float(np.interp(CS.out.vEgo, [0.06, 0.14, 0.25, 0.42], [-0.03, -0.07, -0.13, -0.18]))
            a_boost = float(np.interp(eff_ac - min_decel, [0.02, 0.15], [0.0, -0.08]))
            crawl_guard = float(np.clip((self.low_v_crawl_count - 120) * 0.0020, 0.0, 0.22))
            target_accel = float(stop_feather + a_boost - crawl_guard)
            prev_accel = float(getattr(self, "accel_feather_last", accel))
            down_step = 0.005 if ((CS.out.vEgo > 0.14 and eff_ac > -0.02) or prev_accel > -0.28 or crawl_guard > 0.0) else 0.0
            up_step = float(np.interp(eff_ac - min_decel, [-0.06, 0.04], [0.0036, 0.0016])) if CS.out.vEgo > 0.14 else 0.0036
            accel = float(np.clip(target_accel, prev_accel - down_step, prev_accel + up_step))
            self.accel_feather_last = float(accel)
          else:
            self.stop_latch_frames = getattr(self, "stop_latch_frames", 0) + 1
            prev_accel = float(getattr(self, "accel_feather_last", accel))
            hold_step = 0.0 if self.stop_latch_frames <= 25 else 0.0018
            accel = float(max(min(accel, -0.50), prev_accel - hold_step)) if actuators.aTarget > -1.0 else float(min(accel, -0.50))
            self.accel_feather_last = float(accel)
        else:
          self.stop_req_latched = False
          self.standstill_count = 0
          self.whl_standstill_count = 0
          self.low_v_crawl_count = 0
          self.stop_latch_frames = 0
          if actuators.longControlState == LongCtrlState.pid and CS.out.vEgo < 1.5 and actuators.aTarget < -0.05:
            if actuators.aTarget > -0.80 and accel < actuators.aTarget - 0.10:
              blend = float(np.interp(CS.out.vEgo, [0.50, 1.25], [0.65, 0.0]))
              accel = float(accel * (1.0 - blend) + (actuators.aTarget - 0.10) * blend)
            creep_comp = float(np.interp(CS.out.vEgo, [0.22, 0.50, 0.92, 1.45], [-0.24, -0.20, -0.18, 0.0]))
            fade_by_target = float(np.interp(actuators.aTarget, [-0.20, -0.05], [1.0, 0.0]))
            fade_by_accel = float(np.interp(accel, [-0.80, -0.30], [0.25, 1.0]))
            accel = float(np.clip(accel + creep_comp * fade_by_target * fade_by_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
            if CS.out.vEgo < 0.95 and actuators.aTarget > -0.75:
              hyd_env = float(np.interp(CS.out.vEgo, [0.14, 0.25, 0.42, 0.65, 0.92], [-0.29, -0.38, -0.44, -0.52, -0.64]))
              env_weight = float(np.interp(CS.out.aEgo, [-0.26, -0.12], [1.0, 0.0]) * np.interp(eff_ac, [-0.28, -0.15], [1.0, 0.0]))
              accel = float(accel * (1.0 - env_weight) + max(accel, hyd_env) * env_weight)
            prev_accel = float(getattr(self, "accel_feather_last", accel))
            if CS.out.vEgo < 1.00 and actuators.aTarget > -0.80:
              down_lim = 0.0 if eff_ac < min(actuators.aTarget - 0.04, -0.42) else 0.004
              up_lim = float(np.interp(eff_ac - actuators.aTarget, [-0.06, 0.08], [0.0038, 0.0022])) if CS.out.vEgo > 0.52 else 0.0038
              accel = float(np.clip(accel, prev_accel - down_lim, prev_accel + up_lim))
          self.accel_feather_last = float(accel)
      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
"""
PAGE_NEW2 = """\
        decel_boost = float(np.interp(-accel, [0.25, 0.80, 1.80, 2.80], [0.0, 0.6, 2.0, 4.0])) if CS.out.vEgo > 1.15 else 0.0
        self.jerk_l = min(max(1.0, -self.jerk * 4.0) + decel_boost, jerk_max_l)
        if CS.out.vEgo < 1.15:
          self.jerk_u = max(self.jerk_u, float(np.interp(CS.out.vEgo, [0.05, 0.08, 0.14, 0.38, 0.65, 0.92, 1.15], [0.7, 0.9, 1.8, 1.8, 1.4, 1.2, 0.8])))
          self.jerk_l = max(self.jerk_l, float(np.interp(CS.out.vEgo, [0.65, 0.85, 1.00, 1.15], [1.0, 1.2, 1.2, 1.0])))
        self.cb_upper = 0.0
        self.cb_lower = 0.0
"""
# the page one-liner's two patterns (raw strings, as on the page)
PAGE_PAT1 = r"      casper_ev = self\.CP\.carFingerprint == CAR\.HYUNDAI_CASPER_EV\n.*?      if self\.frame % 2 == 0 and self\.CP\.openpilotLongitudinalControl:"
PAGE_PAT2 = r"        self\.jerk_l = min\(max\(1\.0, -self\.jerk \* 4\.0\), jerk_max_l\)\n.*?        self\.cb_lower = [^\n]+"
V13_BLOCK_1 = "\n".join(PAGE_NEW1.splitlines()[1:-1]) + "\n"  # between the casper_ev line and the frame % 2 line
V13_BLOCK_2 = PAGE_NEW2
SEQS = (mixed_sequence, edge_sequence, gentle_sequence)


def _kw(mode):
  return {} if mode is None else {"K9StopMode": mode}


def page_patched(source):
  """The page one-liner, step by step: CRLF -> LF, pat1 -> new1, pat2 -> rep2 (count=1, re.S), ast.parse."""
  import ast
  s = source.replace("\r\n", "\n")
  assert re.search(PAGE_PAT1, s, flags=re.S), "pos1 not found"
  s = re.sub(PAGE_PAT1, PAGE_NEW1.rstrip("\n"), s, count=1, flags=re.S)
  assert re.search(PAGE_PAT2, s, flags=re.S), "pos2 not found"
  s = re.sub(PAGE_PAT2, PAGE_NEW2.rstrip("\n"), s, count=1, flags=re.S)
  ast.parse(s)
  return s


@pytest.fixture(scope="module")
def prev_cc(tmp_path_factory):
  return load_frozen_cc(PREV, tmp_path_factory.mktemp("cc_f01c98fc"), "cc_f01c98fc")


@pytest.fixture(scope="module")
def page_cc(tmp_path_factory):
  orig = k9_rig._git_show

  def show(commit, rel):
    s = orig(commit, rel)
    return page_patched(s) if rel == CC_REL else s
  with pytest.MonkeyPatch.context() as mp:
    mp.setattr(k9_rig, "_git_show", show)
    return load_frozen_cc(PREV, tmp_path_factory.mktemp("cc_page_v13"), "cc_page_v13")


# --- the page code ---------------------------------------------------------------------------------------------------
def test_page_sha_pinned():
  assert hashlib.sha256((PAGE_NEW1 + PAGE_NEW2).encode()).hexdigest() == "51d9a6437af605fc3c4b6889c98c4fcf7cbc1088f78d1819758da123bb6f0902"
  assert PAGE_NEW1.startswith("      casper_ev = self.CP.carFingerprint == CAR.HYUNDAI_CASPER_EV\n")
  assert PAGE_NEW1.endswith("      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:\n")
  assert "\\" not in PAGE_NEW1 + PAGE_NEW2  # re.sub would expand backslashes in the replacement; the page has none


def test_menu():
  assert kc.V13_ORIG_MENU == 3 and kc.V12_ORIG_MENU == 1 and kc.MODES == (1, 2, 3, 4, 5) and 3 not in kc.MENU  # 4 / 5 since 2026-10-27
  assert carcontroller.V13_ORIG_MENU == 3 and carcontroller.V12_ORIG_MENU == 1


@pytest.mark.parametrize("block, guard", [(V13_BLOCK_1, "      if self.k9_v13_orig:"), (V13_BLOCK_2, "        if v13_orig:")])
def test_menu3_is_the_page_code_unchanged(block, guard):
  src = (ROOT / CC_REL).read_text(encoding="utf-8").splitlines()
  want = ["  " + l if l else l for l in block.splitlines()]
  starts = [i for i, l in enumerate(src) if l.startswith(guard)]
  assert len(starts) == 1
  got = src[starts[0] + 1: starts[0] + 1 + len(want)]
  assert got == want
  nxt = src[starts[0] + 1 + len(want)]
  ind = len(guard) - len(guard.lstrip())
  assert nxt.strip() == "" or len(nxt) - len(nxt.lstrip()) <= ind  # the block ends right there


def test_carcontroller_diff_is_the_page_code_plus_menu_wiring():
  d = subprocess.run(["git", "diff", "-U0", PREV, "--", CC_REL], cwd=ROOT, capture_output=True, text=True, check=True).stdout
  page = {"  " + l for l in (V13_BLOCK_1 + V13_BLOCK_2).splitlines() if l}
  added = [l[1:] for l in d.splitlines() if l.startswith("+") and not l.startswith("+++")]
  removed = [l[1:] for l in d.splitlines() if l.startswith("-") and not l.startswith("---")]
  assert page <= set(added)
  for l in [l for l in added if l.strip() and l not in page]:
    s = l.strip()
    assert s.startswith("#") or "v13_orig" in s or "V13_ORIG_MENU" in s or "whl_standstill_count" in s or "standstill_count" in s, l
  for l in removed:  # only the touched wiring lines (and comments) go
    s = l.strip()
    assert (s.startswith("#") or "V12_ORIG_MENU" in s or "v12_orig" in s or "standstill_count" in s), l
  assert len(removed) <= 8


# --- every other value: byte-identical to f01c98fc -------------------------------------------------------------------
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0, 1, 2, 6, 7, 99, -1, float("nan")])  # 4 -> 6 on 2026-10-27 (4 = C1)
def test_other_values_identical_to_prev(monkeypatch, prev_cc, mode, fingerprint):
  if mode == 2 and fingerprint == CAR.KIA_K9:
    pytest.skip("2026-10-26 23:27 (보노보노 \"2에 k2넣어\"): K9 menu 2 is k2 now (04c88723), pinned in test_k9_stop_v12k9_band.py")
  for seq in SEQS:
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), module=prev_cc, **_kw(mode)))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), **_kw(mode)))
    monkeypatch.undo()
    assert len(new.can) > 400 and ref.can == new.can and ref.scc14 == new.scc14
    assert new.cc.k9_v13_orig is False


def test_other_cars_never_run_menu3(monkeypatch, prev_cc):
  for seq in SEQS:
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), module=prev_cc, K9StopMode=0))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), K9StopMode=3))
    monkeypatch.undo()
    assert ref.can == new.can and new.cc.k9_v13_orig is False and new.cc.k9_creep is None


# --- 3 = the page patch ----------------------------------------------------------------------------------------------
def test_page_patch_applies_to_prev():
  s = page_patched(subprocess.run(["git", "show", f"{PREV}:{CC_REL}"], cwd=ROOT, capture_output=True, text=True,
                                  check=True).stdout)
  assert PAGE_NEW1 in s and PAGE_NEW2 in s


@pytest.mark.parametrize("seq", SEQS)
def test_menu3_identical_to_the_page_patch(monkeypatch, page_cc, seq):
  """K9: menu 3 of this tree == f01c98fc + the page one-liner, menu 0 (every frame, every CAN message, SCC14 too)."""
  ref = seq(Rig(monkeypatch, module=page_cc, K9StopMode=0))
  monkeypatch.undo()
  new = seq(Rig(monkeypatch, K9StopMode=3))
  assert len(new.can) > 400 and ref.can == new.can and ref.scc14 == new.scc14
  assert new.cc.k9_v13_orig is True and new.cc.k9_v12_orig is False and new.cc.k9_creep is None


@pytest.mark.parametrize("seq", SEQS)
def test_menu3_is_not_menu1_or_0(monkeypatch, seq):
  outs = {}
  for mode in (0, 1, 3):
    outs[mode] = seq(Rig(monkeypatch, K9StopMode=mode)).can
    monkeypatch.undo()
  assert outs[3] != outs[1] and outs[3] != outs[0]


def test_menu3_jerk_page_values(monkeypatch):
  import numpy as np
  r = Rig(monkeypatch, K9StopMode=3)
  for _ in range(150):
    r.step(-0.4, LCS.pid, 0.9, a_target=-0.4, a_ego=-0.4)
  _, _, scc14 = r.scc14[-1]
  assert scc14["ComfortBandUpper"] == 0.0 and scc14["ComfortBandLower"] == 0.0
  assert scc14["JerkUpperLimit"] >= float(np.interp(0.9, [0.05, 0.08, 0.14, 0.38, 0.65, 0.92, 1.15],
                                                   [0.7, 0.9, 1.8, 1.8, 1.4, 1.2, 0.8])) - 0.1
  r2 = Rig(monkeypatch, K9StopMode=3)
  for _ in range(60):
    r2.step(-2.0, LCS.pid, 10.0, a_target=-2.0, a_ego=-2.0)
  _, _, hi = r2.scc14[-1]
  assert hi["ComfortBandUpper"] == 0.0 and hi["JerkLowerLimit"] >= 1.0 + float(np.interp(2.0, [0.25, 0.80, 1.80, 2.80], [0.0, 0.6, 2.0, 4.0])) - 0.1


def test_live_switch_resets_v13_state(monkeypatch):
  r = Rig(monkeypatch, K9StopMode=3)
  for value in (3, 1, 3, 0, 2, 3, 6, 3):  # "acts as 0": 4 -> 6 on 2026-10-27 (4 = C1)
    r.params.values["K9StopMode"] = value
    for i in range(60):
      r.step(-0.6, LCS.pid, max(1.0 - 0.02 * i, 0.1), a_target=-0.4, a_ego=-0.3)
    for i in range(40):
      r.step(-0.6, LCS.stopping, 0.2, a_target=-0.4, a_ego=-0.3)
    want = value if value in (1, 2, 3) else 0
    assert r.cc.k9_creep_mode == want
    assert r.cc.k9_v13_orig is (want == 3) and r.cc.k9_v12_orig is (want == 1) and (r.cc.k9_creep is not None) is (want == 2)
  # a change starts the page state fresh (the v13 counters too)
  assert r.cc.low_v_crawl_count > 0 or r.cc.stop_latch_frames > 0 or r.cc.whl_standstill_count > 0
  r.params.values["K9StopMode"] = 1
  for _ in range(50):
    r.step(-0.6, LCS.pid, 3.0)
  assert (r.cc.whl_standstill_count, r.cc.low_v_crawl_count, r.cc.stop_latch_frames, r.cc.standstill_count) == (0, 0, 0, 0)
  assert r.cc.stop_req_latched is False


# --- settings / stored value -----------------------------------------------------------------------------------------
def test_settings_entry():
  import json
  d = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  e = next(p for p in d["params"] if p.get("name") == "K9StopMode")
  assert (e["min"], e["max"], e["default"]) == (0, 5, 0)  # 4 C1 / 5 C1+ since 2026-10-27
  assert e["options"]["ko"][3] == "3 v13 원본" and e["options"]["en"][3] == "3 v13 original" and e["options"]["zh"][3] == "3 v13原版"


class FakeParams:
  def __init__(self, **v):
    self.v = dict(v)

  def get(self, k):
    return self.v.get(k)

  def put_int(self, k, x):
    self.v[k] = int(x)


def test_migration_keeps_3_on_the_current_menu_and_maps_old_3():
  assert mig.STOP_MODE_VER == 3
  p = FakeParams(K9StopMode=3, K9StopModeVer=3)
  assert mig.migrate_k9_stop_mode(p) is False and p.v["K9StopMode"] == 3
  p = FakeParams(K9StopMode=3, K9StopModeVer=2)  # the 2026-10-24 evening menu: 3 was v11 -> 2 (v12k9), never v13
  assert mig.migrate_k9_stop_mode(p) is True and p.v["K9StopMode"] == 2
  p = FakeParams(K9StopMode=3)  # the 0-6 menu
  assert mig.migrate_k9_stop_mode(p) is True and p.v["K9StopMode"] == 2
  assert kc.V13_ORIG_MENU not in set(mig.MENU_MIGRATION_V3.values()) | set(mig.MENU_MIGRATION_FROM_V1.values())
