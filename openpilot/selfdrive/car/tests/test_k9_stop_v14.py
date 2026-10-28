"""K9StopMode 3 = creep_fix v14 original (2026-10-28, 보노보노 01:06 "원본 그대로 푸쉬해라 v14 https://leegrt.org/oppo/creep_fix/").

It replaces 3 v13 original (2026-10-26, test_k9_stop_v13.py, deleted with its code); 4 C1 / 5 C1+ (2026-10-27) are removed
(보노보노 2026-10-27 19:48 "c1 c1+제거하고 말해라", 19:51 "v13.v13k9만든거 제거하시고").
The page https://leegrt.org/oppo/creep_fix/ (copy of 2026-10-28 01:06, index.html sha256
7b5857add051574314cd8d58a61df360897da29ab814846fa71c48ecfaedbc3e) publishes v14 (selectMode(1), "v14 5.2~2.0km/h 선제 감압 +
유압 재결합 이질감 원천 억제 완성본") as a python one-liner: pat1 replaces everything from the casper_ev line to the
`if self.frame % 2 == 0 ...` line with PAGE_NEW1 ([변경 1]), pat2 replaces the make_jerk jerk_l line through the cb_lower line
with PAGE_NEW2 ([변경 2]). PAGE_NEW1 / PAGE_NEW2 below are the page's new1 / rep2 strings, decoded from the two <pre> one-liners
and the JS selectMode(1) new1Code / new2Code (all copies identical). Menu 3 = those lines unchanged (two more leading spaces,
under `if self.k9_v14_orig:` / `if v14_orig:`); 1 v12 original and 2 v12k9 unchanged. The K9 has every signal the page reads
(ESP12 LONG_ACCEL, TCS13 StandStill / DCEnable, WHL_SPD11 in km/h: the hyundai_kia_generic pt parser), so nothing was mapped.
Reference: the page one-liner applied to the d76f7ecc carcontroller.py (the tree before this change), run as menu 0 = the page
patch exactly as a page user would get it on this fork.
Model-only harness with synthetic inputs: source logic and SCC12/SCC14 encoding, not K9 vehicle behaviour (실차 미검증).
"""
import hashlib
import random
import re
import subprocess
from types import SimpleNamespace

import pytest

from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.values import CAR
from openpilot.selfdrive.car import k9_stop_mode_migration as mig
from openpilot.selfdrive.car.tests import k9_rig
from openpilot.selfdrive.car.tests.k9_rig import (LCS, ROOT, CC_REL, Rig, edge_sequence, gentle_sequence, load_frozen_cc, make_cp,
                                                  mixed_sequence)

PREV = "d76f7ecc"  # carrot-wip before this change: 0 base, 1 v12 original, 2 v12k9, 3 v13 original, 4 C1, 5 C1+
PAGE_NEW1 = """\
      casper_ev = self.CP.carFingerprint == CAR.HYUNDAI_CASPER_EV
      if self.CP.openpilotLongitudinalControl:
        lon_ac = float(CS.cp.vl["ESP12"]["LONG_ACCEL"]) if (CS.cp is not None and "ESP12" in CS.cp.vl and "LONG_ACCEL" in CS.cp.vl["ESP12"]) else float(CS.out.aEgo)
        eff_ac = float(min(lon_ac, CS.out.aEgo + 0.03))
        whl_standstill = bool(CS.cp is not None and "WHL_SPD11" in CS.cp.vl and max(CS.cp.vl["WHL_SPD11"]["WHL_SPD_FL"], CS.cp.vl["WHL_SPD11"]["WHL_SPD_FR"], CS.cp.vl["WHL_SPD11"]["WHL_SPD_RL"], CS.cp.vl["WHL_SPD11"]["WHL_SPD_RR"]) < 0.1)
        self.whl_standstill_count = getattr(self, "whl_standstill_count", 0) + 1 if whl_standstill else 0
        if stopping:
          esp_standstill = bool(CS.cp is not None and "TCS13" in CS.cp.vl and CS.cp.vl["TCS13"]["StandStill"] == 1)
          raw_standstill = bool(esp_standstill or CS.out.vEgoRaw < 0.05 or CS.out.vEgo < 0.05)
          self.standstill_count = getattr(self, "standstill_count", 0) + 1 if raw_standstill else 0
          real_standstill = bool(self.standstill_count >= 12 or self.whl_standstill_count >= 28)
          if real_standstill or actuators.aTarget <= -1.0 or (CS.out.vEgo > 0.14 and eff_ac > 0.04) or CS.out.aEgo > 0.06:
            self.stop_req_latched = True
          if not getattr(self, "stop_req_latched", False):
            stopping = False
            self.stop_hold_count = 0
            self.low_v_crawl_count = getattr(self, "low_v_crawl_count", 0) + 1 if (CS.out.vEgo < 0.28 and not raw_standstill and not whl_standstill) else 0
            crawl_guard = float(np.clip((self.low_v_crawl_count - 120) * 0.0020, 0.0, 0.22))
            stop_feather = float(np.interp(CS.out.vEgo, [0.05, 0.14, 0.20, 0.28, 0.42], [-0.28, -0.29, -0.34, -0.39, -0.43]) - crawl_guard)
            min_decel = float(np.interp(CS.out.vEgo, [0.06, 0.14, 0.25, 0.42], [-0.04, -0.08, -0.14, -0.20]))
            a_boost = float(np.interp(eff_ac - min_decel, [0.02, 0.16], [0.0, -0.10]))
            target_accel = float(stop_feather + a_boost)
            prev_accel = float(getattr(self, "accel_feather_last", accel))
            down_step = 0.005 if ((CS.out.vEgo > 0.14 and eff_ac > min_decel + 0.04) or prev_accel > -0.28 or crawl_guard > 0.0) else 0.0
            up_step = float(np.interp(eff_ac - min_decel, [-0.06, 0.04], [0.0038, 0.0015])) if CS.out.vEgo > 0.12 else 0.0035
            accel = float(np.clip(target_accel, prev_accel - down_step, prev_accel + up_step))
            self.accel_feather_last = float(accel)
          else:
            self.stop_hold_count = getattr(self, "stop_hold_count", 0) + 1
            prev_accel = float(getattr(self, "accel_feather_last", accel))
            hold_step = 0.0 if self.stop_hold_count <= 25 else 0.0018
            accel = float(max(min(accel, -0.50), prev_accel - hold_step)) if actuators.aTarget > -1.0 else float(min(accel, -0.50))
            self.accel_feather_last = float(accel)
        else:
          self.stop_req_latched = False
          self.standstill_count = 0
          self.stop_hold_count = 0
          self.low_v_crawl_count = 0
          if actuators.longControlState == LongCtrlState.pid and CS.out.vEgo < 1.5 and actuators.aTarget < -0.05:
            if actuators.aTarget > -1.05 and accel < actuators.aTarget - 0.06:
              blend = float(np.interp(CS.out.vEgo, [0.50, 1.45], [0.80, 0.0]))
              accel = float(accel * (1.0 - blend) + (actuators.aTarget - 0.06) * blend)
            creep_comp = float(np.interp(CS.out.vEgo, [0.14, 0.28, 0.50, 0.92, 1.45], [-0.17, -0.22, -0.22, -0.18, 0.0]))
            fade_by_target = float(np.interp(actuators.aTarget, [-0.20, -0.05], [1.0, 0.0]))
            fade_by_accel = float(np.interp(accel, [-0.52, -0.26], [0.0, 1.0]))
            accel = float(np.clip(accel + creep_comp * fade_by_target * fade_by_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
            prev_accel = float(getattr(self, "accel_feather_last", accel))
            if actuators.aTarget > -1.05:
              hyd_env = float(np.interp(CS.out.vEgo, [0.14, 0.25, 0.42, 0.65, 0.92, 1.15, 1.45], [-0.29, -0.37, -0.43, -0.48, -0.53, -0.58, -0.72]))
              min_env_ac = -0.10 if CS.out.vEgo > 0.55 else -0.15
              env_weight = float(np.interp(CS.out.aEgo, [-0.24, -0.10], [1.0, 0.0]) * np.interp(eff_ac, [min_env_ac - 0.12, min_env_ac], [1.0, 0.0]))
              accel = float(accel * (1.0 - env_weight) + max(accel, hyd_env) * env_weight)
              in_handoff = bool(0.55 < CS.out.vEgo < 1.15 and CS.out.aEgo < -0.14 and prev_accel <= -0.44)
              down_lim = 0.0 if (in_handoff or eff_ac < min(actuators.aTarget - 0.04, -0.36)) else 0.004
              base_up_lim = float(np.interp(eff_ac - actuators.aTarget, [-0.05, 0.12], [0.0038, 0.0022])) if CS.out.vEgo > 0.52 else 0.0038
              up_lim = max(base_up_lim, 0.0110) if (env_weight > 0.5 and prev_accel < hyd_env - 0.01) else base_up_lim
              accel = float(np.clip(accel, prev_accel - down_lim, prev_accel + up_lim))
          self.accel_feather_last = float(accel)
      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
"""
PAGE_NEW2 = """\
        decel_boost = float(np.interp(-accel, [0.25, 0.80, 1.80, 2.80], [0.0, 0.6, 2.0, 4.0]) * np.interp(CS.out.vEgo, [1.10, 1.45], [0.0, 1.0]))
        self.jerk_l = min(max(1.0, -self.jerk * 4.0) + decel_boost, jerk_max_l)
        if CS.out.vEgo < 1.45:
          self.jerk_u = max(self.jerk_u, float(np.interp(CS.out.vEgo, [0.05, 0.08, 0.14, 0.38, 0.65, 1.10, 1.45], [0.7, 0.9, 1.8, 1.8, 1.5, 1.5, 0.9])))
          dc_active = bool(CS.cp is not None and "TCS13" in CS.cp.vl and CS.cp.vl["TCS13"]["DCEnable"] == 1)
          if dc_active and 0.50 < CS.out.vEgo < 1.10 and actuators.aTarget > -0.95 and CS.out.aEgo < -0.10:
            self.jerk_l = float(np.interp(CS.out.vEgo, [0.50, 0.65, 0.95, 1.10], [1.0, 0.7, 0.7, 1.0]))
          else:
            self.jerk_l = max(self.jerk_l, float(np.interp(CS.out.vEgo, [0.65, 0.85, 1.00, 1.15], [1.0, 1.2, 1.2, 1.0])))
        self.cb_upper = 0.0
        self.cb_lower = 0.0
"""
# the page one-liner's two patterns (raw strings, as on the page)
PAGE_PAT1 = r"      casper_ev = self\.CP\.carFingerprint == CAR\.HYUNDAI_CASPER_EV\n.*?      if self\.frame % 2 == 0 and self\.CP\.openpilotLongitudinalControl:"
PAGE_PAT2 = r"        self\.jerk_l = min\(max\(1\.0, -self\.jerk \* 4\.0\), jerk_max_l\)\n.*?        self\.cb_lower = [^\n]+"
V14_BLOCK_1 = "\n".join(PAGE_NEW1.splitlines()[1:-1]) + "\n"  # between the casper_ev line and the frame % 2 line
V14_BLOCK_2 = PAGE_NEW2
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


def _prev_block_lines(guard):
  """The lines of the d76f7ecc `if self.k9_v13_orig:` / `if v13_orig:` block (the removed v13 page code)."""
  src = subprocess.run(["git", "show", f"{PREV}:{CC_REL}"], cwd=ROOT, capture_output=True, text=True, check=True).stdout.splitlines()
  i = next(i for i, l in enumerate(src) if l.startswith(guard))
  ind = len(guard) - len(guard.lstrip())
  out = []
  for l in src[i + 1:]:
    if l.strip() and len(l) - len(l.lstrip()) <= ind:
      break
    out.append(l)
  return out


@pytest.fixture(scope="module")
def prev_cc(tmp_path_factory):
  return load_frozen_cc(PREV, tmp_path_factory.mktemp("cc_d76f7ecc"), "cc_d76f7ecc")


@pytest.fixture(scope="module")
def page_cc(tmp_path_factory):
  orig = k9_rig._git_show

  def show(commit, rel):
    s = orig(commit, rel)
    return page_patched(s) if rel == CC_REL else s
  with pytest.MonkeyPatch.context() as mp:
    mp.setattr(k9_rig, "_git_show", show)
    return load_frozen_cc(PREV, tmp_path_factory.mktemp("cc_page_v14"), "cc_page_v14")


# --- the page code ---------------------------------------------------------------------------------------------------
def test_page_sha_pinned():
  assert hashlib.sha256((PAGE_NEW1 + PAGE_NEW2).encode()).hexdigest() == "48d53167327502b49b850dadcebe4cf8fdfb4585de87a89b41cf66140fae77f8"
  assert PAGE_NEW1.startswith("      casper_ev = self.CP.carFingerprint == CAR.HYUNDAI_CASPER_EV\n")
  assert PAGE_NEW1.endswith("      if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:\n")
  assert "\\" not in PAGE_NEW1 + PAGE_NEW2  # re.sub would expand backslashes in the replacement; the page has none
  # the v14 markers of the page's own check command (cmd-verify: "in_handoff =", "dc_active =", "up_lim = max(base_up_lim, 0.0110)")
  assert "in_handoff =" in PAGE_NEW1 and "up_lim = max(base_up_lim, 0.0110)" in PAGE_NEW1 and "dc_active =" in PAGE_NEW2


def test_menu():
  assert kc.V14_ORIG_MENU == 3 and kc.V12_ORIG_MENU == 2 and kc.MODES == (1, 2, 3) and 3 not in kc.MENU
  assert carcontroller.V14_ORIG_MENU == 3 and carcontroller.V12_ORIG_MENU == 2  # 1 until 2026-10-29 04:48
  assert kc.MENU == {1: kc.V12K9O} and kc.KINDS == (kc.V9, kc.V12K9, kc.V12K9O)  # C1 / C1+ (kinds 9 / 10) gone;
  # kind 13 is menu 1 since the 2026-10-29 04:48 slot swap (menu 2 before that)
  for gone in ("V13_ORIG_MENU", "V12K9_C1", "V12K9_C1P"):
    assert not hasattr(kc, gone) and not hasattr(carcontroller, gone), gone


def test_v13_code_gone():
  src = (ROOT / CC_REL).read_text(encoding="utf-8")
  assert "v13_orig" not in src and "stop_latch_frames" not in src
  assert not (ROOT / "openpilot/selfdrive/car/tests/test_k9_stop_v13.py").exists()
  assert not (ROOT / "openpilot/selfdrive/car/tests/test_k9_stop_c1_menu45.py").exists()


@pytest.mark.parametrize("block, guard", [(V14_BLOCK_1, "      if self.k9_v14_orig:"), (V14_BLOCK_2, "        if v14_orig:")])
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
  page = {"  " + l for l in (V14_BLOCK_1 + V14_BLOCK_2).splitlines() if l}
  old_v13 = set(_prev_block_lines("      if self.k9_v13_orig:") + _prev_block_lines("        if v13_orig:"))
  added = [l[1:] for l in d.splitlines() if l.startswith("+") and not l.startswith("+++")]
  removed = [l[1:] for l in d.splitlines() if l.startswith("-") and not l.startswith("---")]
  assert page <= set(added) | set(src for src in (ROOT / CC_REL).read_text(encoding="utf-8").splitlines())
  for l in [l for l in added if l.strip() and l not in page]:
    s = l.strip()
    assert s.startswith("#") or "v14_orig" in s or "V14_ORIG_MENU" in s or "standstill_count" in s, l
  for l in [l for l in removed if l not in old_v13]:  # besides the old v13 page lines: only the touched wiring lines / comments
    s = l.strip()
    assert (s.startswith("#") or "v13_orig" in s or "V13_ORIG_MENU" in s or "standstill_count" in s), l
  assert len([l for l in removed if l not in old_v13]) <= 14  # 12 wiring lines on 2026-10-28


# --- every other value: byte-identical to d76f7ecc -------------------------------------------------------------------
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0, 1, 2, 6, 7, 99, -1, float("nan")])
def test_other_values_identical_to_prev(monkeypatch, prev_cc, mode, fingerprint):
  ref_mode = {1: 2, 2: 1}.get(mode, mode)  # 2026-10-29 04:48 slot swap: the same method, the other number on d76f7ecc
  for seq in SEQS:
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), module=prev_cc, **_kw(ref_mode)))
    monkeypatch.undo()
    if mode == 1 and fingerprint == CAR.KIA_K9:
      # 2026-10-29 bono1739 원본 (pin narrowed): K9 menu 1 is kind 13 = kind 8 + the driver's stop table (stopping state below
      # B17O_SPEED; test_k9_stop_v12k9o.py). With the table switched off it must still be byte-identical
      # to d76f7ecc menu 2 (its own number before the 04:48 swap); every other value and other cars stay pinned.
      monkeypatch.setattr(kc, "B17O_SPEED", 0.0)
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), **_kw(mode)))
    monkeypatch.undo()
    assert len(new.can) > 400 and ref.can == new.can and ref.scc14 == new.scc14
    assert new.cc.k9_v14_orig is False


@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [4, 5])
def test_old_c1_values_act_as_0(monkeypatch, prev_cc, mode, fingerprint):
  """4 C1 / 5 C1+ are gone: a stored 4 / 5 (only by hand: the migration maps them to 0) now acts as d76f7ecc menu 0."""
  for seq in SEQS:
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), module=prev_cc, K9StopMode=0))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), K9StopMode=mode))
    monkeypatch.undo()
    assert len(new.can) > 400 and ref.can == new.can and ref.scc14 == new.scc14
    assert new.cc.k9_creep is None and new.cc.k9_v14_orig is False and new.cc.k9_creep_mode == 0


def test_other_cars_never_run_menu3(monkeypatch, prev_cc):
  for seq in SEQS:
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), module=prev_cc, K9StopMode=0))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), K9StopMode=3))
    monkeypatch.undo()
    assert ref.can == new.can and new.cc.k9_v14_orig is False and new.cc.k9_creep is None


# --- 3 = the page patch ----------------------------------------------------------------------------------------------
def test_page_patch_applies_to_prev_and_this_tree():
  s = page_patched(subprocess.run(["git", "show", f"{PREV}:{CC_REL}"], cwd=ROOT, capture_output=True, text=True,
                                  check=True).stdout)
  assert PAGE_NEW1 in s and PAGE_NEW2 in s
  s = page_patched((ROOT / CC_REL).read_text(encoding="utf-8"))
  assert PAGE_NEW1 in s and PAGE_NEW2 in s


@pytest.mark.parametrize("seq", SEQS)
def test_menu3_identical_to_the_page_patch(monkeypatch, page_cc, seq):
  """K9: menu 3 of this tree == d76f7ecc + the page one-liner, menu 0 (every frame, every CAN message, SCC14 too)."""
  ref = seq(Rig(monkeypatch, module=page_cc, K9StopMode=0))
  monkeypatch.undo()
  new = seq(Rig(monkeypatch, K9StopMode=3))
  assert len(new.can) > 400 and ref.can == new.can and ref.scc14 == new.scc14
  assert new.cc.k9_v14_orig is True and new.cc.k9_v12_orig is False and new.cc.k9_creep is None


class _CS:
  """CS.cp for the page code: the K9 pt-parser messages it reads (ESP12 LONG_ACCEL, TCS13 StandStill / DCEnable,
  WHL_SPD11 km/h), values set per frame by the sequence (the default rig leaves CS.cp None = the page's fallbacks)."""
  def __init__(self):
    self.vl = {"ESP12": {"LONG_ACCEL": 0.0}, "TCS13": {"StandStill": 0, "DCEnable": 0},
               "WHL_SPD11": {"WHL_SPD_FL": 0.0, "WHL_SPD_FR": 0.0, "WHL_SPD_RL": 0.0, "WHL_SPD_RR": 0.0}}


def _signal_sequence(rig, cp, seed):
  """Random stops with the four page signals moving: LONG_ACCEL around aEgo, ESC standstill, DCEnable on / off,
  wheel speeds around vEgo (some frames below the page's 0.1 km/h)."""
  rng = random.Random(seed)
  for cycle in range(6):
    v0 = rng.uniform(1.0, 6.0)
    for i in range(420):
      v = max(v0 - 0.006 * i * rng.uniform(0.8, 1.2), 0.0)
      st = LCS.stopping if (v < 0.7 and rng.random() < 0.9) else LCS.pid
      a = rng.uniform(-1.3, -0.1) if rng.random() < 0.9 else rng.uniform(-0.2, 0.3)
      a_t = a + rng.uniform(-0.2, 0.2)
      ae = rng.uniform(-1.0, 0.15)
      cp.vl["ESP12"]["LONG_ACCEL"] = ae + rng.uniform(-0.15, 0.15)
      cp.vl["TCS13"]["StandStill"] = int(v < 0.03 and rng.random() < 0.7)
      cp.vl["TCS13"]["DCEnable"] = int(rng.random() < (0.8 if a < -0.2 else 0.2))
      w = max(v * 3.6 + rng.uniform(-0.15, 0.15), 0.0)
      for k in cp.vl["WHL_SPD11"]:
        cp.vl["WHL_SPD11"][k] = w if rng.random() < 0.95 else 0.0
      rig.step(a, st, v, standstill=v < 0.01, a_target=a_t, a_ego=ae, brake=cycle == 4 and 300 <= i < 305,
               lead=(rng.uniform(2.0, 9.0), -v) if cycle % 2 else None)
    for i in range(80):
      cp.vl["TCS13"]["StandStill"] = 1
      for k in cp.vl["WHL_SPD11"]:
        cp.vl["WHL_SPD11"][k] = 0.0
      rig.step(-0.8, LCS.stopping, 0.0, standstill=True, a_target=-0.5, a_ego=0.0)
    for i in range(40):
      cp.vl["TCS13"]["StandStill"] = int(i < 3)
      rig.step(0.8, LCS.pid, 0.05 * i, standstill=i == 0)
  return rig


@pytest.mark.parametrize("seed", ["v14-a", "v14-b", "v14-c", "v14-d"])
def test_menu3_identical_to_the_page_patch_with_page_signals(monkeypatch, page_cc, seed):
  """Same as above with CS.cp carrying the four signals the page reads (every branch of the page code reachable):
  same inputs -> same accel, every CAN message and SCC14 frame identical to the page patch."""
  outs = []
  for module, mode in ((page_cc, 0), (carcontroller, 3)):
    cp = _CS()

    def mk(cp=cp):
      cs = k9_rig._base_ci_state()
      cs.cp = cp
      return cs
    monkeypatch.setattr(k9_rig, "make_ci_state", mk)
    r = _signal_sequence(Rig(monkeypatch, module=module, K9StopMode=mode), cp, seed)
    outs.append(r)
    monkeypatch.undo()
  ref, new = outs
  assert len(new.can) > 2000 and ref.can == new.can and ref.scc14 == new.scc14 and ref.scc12 == new.scc12
  sr = [f for f, _, m in new.scc12 if m["StopReq"]]
  assert sr  # the latch is reached
  assert new.cc.k9_v14_orig is True


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
  assert scc14["JerkUpperLimit"] >= float(np.interp(0.9, [0.05, 0.08, 0.14, 0.38, 0.65, 1.10, 1.45],
                                                   [0.7, 0.9, 1.8, 1.8, 1.5, 1.5, 0.9])) - 0.1
  r2 = Rig(monkeypatch, K9StopMode=3)
  for _ in range(60):
    r2.step(-2.0, LCS.pid, 10.0, a_target=-2.0, a_ego=-2.0)
  _, _, hi = r2.scc14[-1]
  assert hi["ComfortBandUpper"] == 0.0 and hi["JerkLowerLimit"] >= 1.0 + float(np.interp(2.0, [0.25, 0.80, 1.80, 2.80], [0.0, 0.6, 2.0, 4.0])) - 0.1


def test_live_switch_resets_v14_state(monkeypatch):
  r = Rig(monkeypatch, K9StopMode=3)
  for value in (3, 1, 3, 0, 2, 3, 4, 5, 3):
    r.params.values["K9StopMode"] = value
    for i in range(60):
      r.step(-0.6, LCS.pid, max(1.0 - 0.02 * i, 0.1), a_target=-0.4, a_ego=-0.3)
    for i in range(40):
      r.step(-0.6, LCS.stopping, 0.2, a_target=-0.4, a_ego=-0.3)
    want = value if value in (1, 2, 3) else 0
    assert r.cc.k9_creep_mode == want
    assert r.cc.k9_v14_orig is (want == 3) and r.cc.k9_v12_orig is (want == 2) and (r.cc.k9_creep is not None) is (want == 1)
  # a change starts the page state fresh (the v14 counters too)
  assert r.cc.low_v_crawl_count > 0 or r.cc.stop_hold_count > 0
  r.params.values["K9StopMode"] = 2  # the v12 original since 04:48
  for _ in range(50):
    r.step(-0.6, LCS.pid, 3.0)
  assert (r.cc.whl_standstill_count, r.cc.low_v_crawl_count, r.cc.stop_hold_count, r.cc.standstill_count) == (0, 0, 0, 0)
  assert r.cc.stop_req_latched is False


# --- settings / stored value -----------------------------------------------------------------------------------------
def test_settings_entry():
  import json
  d = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  e = next(p for p in d["params"] if p.get("name") == "K9StopMode")
  assert (e["min"], e["max"], e["default"]) == (0, 3, 0)
  assert e["options"]["ko"][3] == "3 v14 원본" and e["options"]["en"][3] == "3 v14 original" and e["options"]["zh"][3] == "3 v14原版"
  assert all(len(e["options"][k]) == 4 for k in ("ko", "en", "zh"))
  assert "v13" not in json.dumps(e, ensure_ascii=False) and "C1" not in json.dumps(e, ensure_ascii=False)


class FakeParams:
  def __init__(self, **v):
    self.v = dict(v)

  def get(self, k):
    return self.v.get(k)

  def put_int(self, k, x):
    self.v[k] = int(x)


def test_migration_old_v13_c1_values_go_to_0_never_to_v14():
  """2026-10-28: a K9StopModeVer 3 device (the 0-5 menu of d76f7ecc) with 3 v13 / 4 C1 / 5 C1+ (6 = the never-pushed v13k9)
  goes to 0 once; an old 3 v13 is NOT carried to the new 3 v14 (v14 is picked by hand). 0 / 1 / 2 stay."""
  assert mig.STOP_MODE_VER == 5  # 4 until the 2026-10-29 04:48 slot swap
  for old, want in ((0, 0), (1, 2), (2, 1), (3, 0), (4, 0), (5, 0), (6, 0)):  # table first, then the 1 <-> 2 swap
    p = FakeParams(K9StopMode=old, K9StopModeVer=3)
    assert mig.migrate_k9_stop_mode(p) is (want != old) and p.v["K9StopMode"] == want and p.v["K9StopModeVer"] == 5
    assert mig.migrate_k9_stop_mode(p) is False and p.v["K9StopMode"] == want  # once only
  p = FakeParams(K9StopMode=3, K9StopModeVer=4)  # the v14 original keeps its number through the 2026-10-29 swap
  assert mig.migrate_k9_stop_mode(p) is False and p.v["K9StopMode"] == 3 and p.v["K9StopModeVer"] == 5
  p = FakeParams(K9StopMode=3, K9StopModeVer=5)  # picked on the new menu: v14, never touched
  assert mig.migrate_k9_stop_mode(p) is False and p.v["K9StopMode"] == 3
  p = FakeParams(K9StopMode=3, K9StopModeVer=2)  # the 2026-10-24 evening menu: 3 was v11 -> 2 -> 1 after the swap, never v14
  assert mig.migrate_k9_stop_mode(p) is True and p.v["K9StopMode"] == 1
  p = FakeParams(K9StopMode=3)  # the 0-6 menu
  assert mig.migrate_k9_stop_mode(p) is True and p.v["K9StopMode"] == 1
  assert kc.V14_ORIG_MENU not in set(mig.MENU_MIGRATION_V4.values()) | set(mig.MENU_MIGRATION_V3.values()) | set(mig.MENU_MIGRATION_FROM_V1.values())
