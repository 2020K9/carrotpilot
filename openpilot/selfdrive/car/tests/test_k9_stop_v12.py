"""K9StopMode menu of 2026-10-25 (보노보노 00:36 "v12 원본 그대로 넣어놓기도 해보고, K9 맞춰 넣는 것도 v12k9으로" /
"브레이크 앞전꺼는 다 별로니 지워" / 00:39 "원본 그대로 수정하지 말고 올려라"): 0 base, 1 v12 original (the creep_fix v12 page
code unchanged in carcontroller.py), 2 v12k9 (k9_creep kind V12K9). The old 1 v10 / 2 v10 + light / 3 v11 are deleted.
Model-only harness with synthetic inputs: source logic and SCC12/SCC14 encoding, not K9 vehicle behaviour (실차 미검증).
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
  assert kc.V12_ORIG_MENU == 1 and kc.MENU == {2: V12K9} and kc.MODES == (1, 2)
  assert kc.KINDS == (V9, V12K9) == (1, 8) and carcontroller.K9_CREEP_MENU is kc.MENU
  assert carcontroller.V12_ORIG_MENU == 1
  for gone in ("V10", "V10_LIGHT", "V11", "LIGHT_MIN_DECEL", "V10_FEATHER_V", "V11_FEATHER_V"):
    assert not hasattr(kc, gone), gone


def test_page_sha_pinned():
  # the two code blocks of https://leegrt.org/oppo/creep_fix/changelog.html (v12, 2026-10-25 00:36 copy)
  assert hashlib.sha256((PAGE_BLOCK_1 + PAGE_BLOCK_2).encode()).hexdigest() == \
    "0c2ee08d63db2a03b14501e4b757e35d78740560c7899a58577005a4c8e7cc89"


@pytest.mark.parametrize("block, guard", [(PAGE_BLOCK_1, "      if self.k9_v12_orig:"), (PAGE_BLOCK_2, "        if v12_orig:")])
def test_menu1_is_the_page_code_unchanged(block, guard):
  """Menu 1 = the page block, every line the same (only two more leading spaces: it sits under the menu-1 if)."""
  src = (ROOT / CC_REL).read_text(encoding="utf-8").splitlines()
  want = ["  " + l if l else l for l in block.splitlines()]
  starts = [i for i, l in enumerate(src) if l.startswith(guard)]
  assert len(starts) == 1
  got = src[starts[0] + 1: starts[0] + 1 + len(want)]
  assert got == want
  nxt = src[starts[0] + 1 + len(want)] if starts[0] + 1 + len(want) < len(src) else ""
  ind = len(guard) - len(guard.lstrip())
  assert nxt.strip() == "" or len(nxt) - len(nxt.lstrip()) <= ind  # the block ends right there


def test_carcontroller_diff_is_the_page_code_plus_menu_wiring():
  """Every line added to carcontroller.py since PREV is a page line or one of the listed menu-wiring lines."""
  d = subprocess.run(["git", "diff", "-U0", PREV, "--", CC_REL], cwd=ROOT, capture_output=True, text=True, check=True).stdout
  page = {"  " + l for l in (PAGE_BLOCK_1 + PAGE_BLOCK_2).splitlines() if l}
  added = [l[1:] for l in d.splitlines() if l.startswith("+") and not l.startswith("+++")]
  other = [l for l in added if l.strip() and l not in page]
  assert len(added) - len(other) >= len(page)
  for l in other:  # wiring only: comments, the menu switch, the call arguments
    s = l.strip()
    assert (s.startswith("#") or "v12_orig" in s or "V12_ORIG_MENU" in s or "K9_CREEP_MENU" in s or "standstill_count" in s
            or "accel_feather_last" in s or "cb_prev" in s or "jerk_u_cap" in s or "creep_obj" in s), l


# --- 0 / other values: unchanged -------------------------------------------------------------------------------------
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0, 3, 4, 7, 99, -1])
def test_base_and_other_values_identical_to_prev_0(monkeypatch, prev_cc, mode, fingerprint):
  for seq in (mixed_sequence, edge_sequence, gentle_sequence):
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), module=prev_cc, K9StopMode=0))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), **_kw(mode)))
    monkeypatch.undo()
    assert len(new.can) > 400 and ref.can == new.can and ref.scc14 == new.scc14
    assert new.cc.k9_creep is None and new.cc.k9_v12_orig is False


@pytest.mark.parametrize("mode", [1, 2])
def test_other_cars_never_run_the_menu(monkeypatch, prev_cc, mode):
  for seq in (mixed_sequence, edge_sequence):
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), module=prev_cc, K9StopMode=0))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), K9StopMode=mode))
    monkeypatch.undo()
    assert ref.can == new.can and new.cc.k9_v12_orig is False and new.cc.k9_creep is None


# --- 2 v12k9 ---------------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("seed", range(6))
def test_v12k9_stop_phase_is_the_old_v11(prev_cc, monkeypatch, seed):
  """v12k9 = the old v11 with the v11 pid re-press guard replaced by the v12 two-way rate limit: with both switched off,
  kind 8 is float-identical to b6696b37 kind 7 (v11), state too (stop phase, latch, creep compensation)."""
  monkeypatch.setattr(kc, "V12_RATE_SPEED", -1.0)
  monkeypatch.setitem(prev_cc.K9Creep._update_v11.__globals__, "V11_GUARD_SPEED", -1.0)
  rng = random.Random(f"k9-v12k9-{seed}")
  old, new = prev_cc.K9Creep(PREV_V11), K9Creep(V12K9)
  assert old.__class__ is not K9Creep
  for kw, i in _random_inputs(rng, 4000):
    ro, rn = old.update(**kw), new.update(**kw)
    assert ro == rn or (math.isnan(ro[0]) and math.isnan(rn[0]) and ro[1] == rn[1]), (i, kw)
    assert (old.latched, old.active) == (new.latched, new.active), i


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
  assert _pid(c, -0.50, 0.0, 0.9, -0.30, -0.35) <= -0.50 + 0.0035 + 1e-9
  assert _pid(c, -0.50, 0.0, 0.4, -0.30, -0.35) > -0.49
  # at or above 1.0 m/s, or a plan below -0.80: not limited
  assert _pid(c, -0.30, -0.60, 1.0, -0.5, -0.30) <= -0.59
  assert _pid(c, -0.30, -1.20, 0.9, -0.9, -0.30) <= -1.0


def test_v12k9_jerk_and_band():
  c = K9Creep(V12K9)
  for v in (0.05, 0.1, 0.3, 0.6, 0.9, 1.1, 1.2, 2.0):
    ju, cu, cl = c.bands(0.5, v, 0.9, 0.8, 0.0, 0.0)
    want = max(0.5, float(np.interp(v, kc.V12_JERK_U_BP, kc.V12_JERK_U_V))) if v < 1.15 else 0.5
    assert ju == pytest.approx(want) and cu == 0.0 and cl == 0.0
    jl = c.jerk_l_floor(1.0, v)
    assert jl == pytest.approx(max(1.0, float(np.interp(v, kc.V12_JERK_L_BP, kc.V12_JERK_L_V))) if v < 1.15 else 1.0)
  assert kc.V12_JERK_U_V == [0.7, 0.9, 1.6, 1.6, 1.3, 1.2, 0.8] and kc.V12_JERK_L_V == [1.0, 1.3, 1.3, 1.0]
  assert K9Creep(V9).jerk_l_floor(0.7, 0.8) == 0.7


# --- 1 v12 original through the real CarController -------------------------------------------------------------------
def test_v12_orig_runs_and_waits_12_frames(monkeypatch):
  r = Rig(monkeypatch, K9StopMode=1)
  for i in range(200):
    r.step(-0.8, LCS.pid, max(3.0 - 0.015 * i, 0.3), a_target=-0.6, a_ego=-0.6)
  assert r.cc.k9_v12_orig is True and r.cc.k9_creep is None
  stops = []
  for i in range(40):
    _, sent = r.step(-0.5, LCS.stopping, 0.0, standstill=True, a_target=-0.3, a_ego=-0.05)
    if sent is not None:
      stops.append(int(sent["StopReq"]))
  assert stops[:5] == [0] * 5 and stops[-1] == 1  # StopReq only after a held standstill
  assert len(r.can) == 240


def test_v12_orig_jerk_page_values(monkeypatch):
  r = Rig(monkeypatch, K9StopMode=1)
  for i in range(150):
    r.step(-0.4, LCS.pid, 0.9, a_target=-0.4, a_ego=-0.4)
  _, _, scc14 = r.scc14[-1]
  assert scc14["ComfortBandUpper"] == 0.0 and scc14["ComfortBandLower"] == 0.0
  assert scc14["JerkUpperLimit"] >= float(np.interp(0.9, [0.05, 0.08, 0.14, 0.38, 0.60, 0.92, 1.15],
                                                   [0.7, 0.9, 1.6, 1.6, 1.3, 1.2, 0.8])) - 0.1


@pytest.mark.parametrize("seq", [mixed_sequence, edge_sequence, gentle_sequence])
@pytest.mark.parametrize("mode", [1, 2])
def test_menu_sequences_run(monkeypatch, seq, mode):
  r = seq(Rig(monkeypatch, K9StopMode=mode))
  assert len(r.can) > 400
  assert (r.cc.k9_v12_orig, r.cc.k9_creep is not None) == ((True, False) if mode == 1 else (False, True))


def test_live_switch_resets(monkeypatch):
  r = Rig(monkeypatch, K9StopMode=1)
  for value in (1, 2, 0, 1, 2):
    r.params.values["K9StopMode"] = value
    for i in range(80):
      r.step(-0.6, LCS.pid, max(2.0 - 0.02 * i, 0.4), a_target=-0.4, a_ego=-0.3)
    assert r.cc.k9_creep_mode == value
    assert r.cc.k9_v12_orig is (value == 1) and (r.cc.k9_creep is not None) is (value == 2)


# --- stored-value migration ------------------------------------------------------------------------------------------
class FakeParams:
  def __init__(self, **v):
    self.v = dict(v)

  def get(self, k):
    return self.v.get(k)

  def put_int(self, k, x):
    self.v[k] = int(x)


def test_migration_tables():
  assert mig.STOP_MODE_VER == 3
  assert mig.MENU_MIGRATION_V3 == {0: 0, 1: 2, 2: 2, 3: 2}
  assert mig.MENU_MIGRATION_FROM_V1 == {0: 0, 1: 2, 2: 2, 3: 2, 4: 2, 5: 2, 6: 2}
  assert set(mig.MENU_MIGRATION_V3.values()) | set(mig.MENU_MIGRATION_FROM_V1.values()) <= {0, *kc.MENU}  # never menu 1


@pytest.mark.parametrize("ver", [2, None, 1, 5])
@pytest.mark.parametrize("old", [None, 0, 1, 2, 3, 4, 5, 6, 7, 99, -1, "x"])
def test_migration(ver, old):
  table = mig.MENU_MIGRATION_V3 if ver == 2 else mig.MENU_MIGRATION_FROM_V1
  p = FakeParams(**({} if old is None else {"K9StopMode": old}), **({} if ver is None else {"K9StopModeVer": ver}))
  changed = mig.migrate_k9_stop_mode(p)
  want = table.get(old, old) if isinstance(old, int) else old
  assert p.v.get("K9StopMode") == want and p.v["K9StopModeVer"] == 3 and changed == (want != old)
  assert mig.migrate_k9_stop_mode(p) is False and p.v.get("K9StopMode") == want  # once only


def test_migration_after_ver3_never_touches():
  for v in (0, 1, 2, 3, 9):
    p = FakeParams(K9StopMode=v, K9StopModeVer=3)
    assert mig.migrate_k9_stop_mode(p) is False and p.v["K9StopMode"] == v


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
  assert e["min"] == 0 and e["max"] == 2 and e["default"] == 0
