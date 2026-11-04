"""공통환경 비교 (2026-11-04 22:xx, k9cmp2200): a separate path next to the frozen-controller check, which stays as it is.

k9_rig.load_frozen_cc() refuses to load a past controller when any non-K9 file under opendbc_repo changed in a way it does not
whitelist (k9_rig.py:166: hyundaicanfd.py outside create_acc_control_scc2). That check is kept unchanged. This file instead
answers a narrower, separately named question:

  (a) "Common environment": the K9 controller of the reference commit (its carcontroller.py and every k9_*.py it has) and the
      K9 controller of this tree are driven with the same inputs while sharing this tree's common modules (hyundaicanfd.py,
      hyundaican.py, carstate.py, values.py, dbc ...). Their CAN output must match frame for frame. One test per case of
      test_k9_stop_menu_035's two frozen comparisons (20 + 12 = 32 cases), same sequences, same assertions.
      This is NOT the historical byte-equality check: the common files are this tree's, not the reference commit's.
  (b) What the hyundaicanfd.py difference does, per changed function: each changed function is put back to its reference
      version alone (and all of them together) under this tree's K9 controller, and the K9 CAN output is compared; the number
      of calls of each function in the K9 run is reported. Each changed function is also compared directly, old vs new, on a
      grid of CAN-FD inputs; every differing case is reported (not hidden) and its condition is asserted.

Model-only harness with synthetic inputs (실차 미검증). Results also go to $K9CMP_OUT/results.jsonl when that is set.
"""
import ast
import importlib.util
import itertools
import json
import os
import subprocess
import sys
from types import SimpleNamespace

import pytest

from opendbc.can import CANPacker
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.values import HyundaiFlags
from openpilot.selfdrive.car.tests.k9_rig import CC_REL, HY, ROOT, Rig, _git_show
from openpilot.selfdrive.car.tests.test_k9_creep_mode import SEQS
from openpilot.selfdrive.car.tests.test_k9_stop_gap import RUNS
from openpilot.selfdrive.car.tests.test_k9_stop_menu_035 import REF

K9_OWN = ("k9_stop_taper", "k9_creep", "k9_hard_brake")
CANFD_REL = f"{HY}/hyundaicanfd.py"


def _record(kind, **data):
  out = os.environ.get("K9CMP_OUT")
  if out:
    with open(os.path.join(out, "results.jsonl"), "a", encoding="utf-8") as f:
      f.write(json.dumps({"kind": kind, **data}, ensure_ascii=False, default=str) + "\n")


def _load_source(source, path, modname):
  path.write_text(source, encoding="utf-8")
  spec = importlib.util.spec_from_file_location(modname, path)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


def common_env_files(commit):
  """Non-test files under opendbc_repo that differ from `commit` and are NOT the K9 controller: this tree's version of
  each is what both controllers use here."""
  changed = subprocess.run(["git", "diff", "--name-only", commit, "--", "opendbc_repo"], cwd=ROOT, check=True,
                           capture_output=True, text=True).stdout.split()
  own = {CC_REL} | {f"{HY}/{n}.py" for n in K9_OWN}
  return sorted(f for f in changed if "/tests/" not in f and f not in own)


def load_cc_common_env(commit, tmp_dir, tag):
  """carcontroller.py and the k9_*.py of `commit`; every other module is this tree's (no equality assertion)."""
  saved, injected, used = {}, [], []
  try:
    for name in K9_OWN:
      rel = f"{HY}/{name}.py"
      try:
        source = _git_show(commit, rel)
      except subprocess.CalledProcessError:
        continue
      full = f"opendbc.car.hyundai.{name}"
      module = _load_source(source, tmp_dir / f"{tag}_{name}.py", f"{tag}_{name}")
      saved[full] = sys.modules.get(full)
      sys.modules[full] = module
      injected.append(full)
      used.append(rel)
    cc = _load_source(_git_show(commit, CC_REL), tmp_dir / f"{tag}_carcontroller.py", f"{tag}_carcontroller")
  finally:
    for full in injected:
      if saved[full] is None:
        sys.modules.pop(full, None)
      else:
        sys.modules[full] = saved[full]
  from opendbc.car.hyundai import carcontroller
  assert cc.__file__ != carcontroller.__file__
  return cc, [CC_REL] + used


@pytest.fixture(scope="module")
def ref_env(tmp_path_factory):
  cc, own = load_cc_common_env(REF, tmp_path_factory.mktemp("cmp_ref"), "cmp_9df95178c")
  head = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, check=True, capture_output=True, text=True).stdout.strip()
  _record("env", ref=REF, head=head, ref_controller_files=own, common_env_files=common_env_files(REF))
  return cc


def _kw(mode):
  return {} if mode is None else {"K9StopMode": mode}


# 2026-11-05 07:16 (대감님 "0.3.5 ... 0.1.2로 바꿔"): this tree numbers the old 0 / 3 / 5 as 0 / 1 / 2 (number pin only)
NEW_NUM = {None: None, 0: 0, 1: 3, 2: 4, 3: 1, 4: 5, 5: 2}


def _kw_new(mode):
  return _kw(NEW_NUM[mode])


def _run(seqs, seq, module, values):
  mp = pytest.MonkeyPatch()
  try:
    return seqs[seq](Rig(mp, **({} if module is None else {"module": module}), **values))
  finally:
    mp.undo()


def _first_diff(a, b):
  return next((i for i, (x, y) in enumerate(zip(a, b, strict=False)) if x != y), None if len(a) == len(b) else min(len(a), len(b)))


# --- (a) 32 cases: reference K9 controller vs this tree's, same common environment ------------------------------------
@pytest.mark.parametrize("seq", list(RUNS))
@pytest.mark.parametrize("mode", [None, 0, 3, 5])
def test_common_env_0_3_5_send_the_9df95178c_bytes(ref_env, mode, seq):
  """Counterpart of test_k9_stop_menu_035::test_0_3_5_send_the_9df95178c_bytes[mode-seq]."""
  ref = _run(RUNS, seq, ref_env, _kw(mode))
  new = _run(RUNS, seq, None, _kw_new(mode))
  same = new.can == ref.can and new.scc14 == ref.scc14
  _record("case", test=f"test_0_3_5_send_the_9df95178c_bytes[{mode}-{seq}]", frames=len(new.can), same=same,
          first_diff=_first_diff(new.can, ref.can))
  assert len(new.can) > 400 and same
  if mode in (3, 5):
    assert new.cc.k9_creep.mode == ref_env.K9_CREEP_MENU[mode]
    # k9cmp2235: the actual 3/5 value is also checked against this tree's menu table (as the original test does)
    assert new.cc.k9_creep.mode == kc.MENU[NEW_NUM[mode]]
    _record("menu", mode=mode, seq=seq, actual=new.cc.k9_creep.mode, ref_menu=ref_env.K9_CREEP_MENU[mode], cur_menu=kc.MENU[NEW_NUM[mode]])


@pytest.mark.parametrize("seq", list(SEQS))
@pytest.mark.parametrize("mode", [1, 2, 4])
def test_common_env_removed_numbers_send_the_9df95178c_0_bytes(ref_env, mode, seq):
  """Counterpart of test_k9_stop_menu_035::test_removed_numbers_send_the_9df95178c_0_bytes[mode-seq]."""
  ref = _run(SEQS, seq, ref_env, {"K9StopMode": 0})
  new = _run(SEQS, seq, None, {"K9StopMode": NEW_NUM[mode]})
  same = new.can == ref.can and new.scc14 == ref.scc14
  _record("case", test=f"test_removed_numbers_send_the_9df95178c_0_bytes[{mode}-{seq}]", frames=len(new.can), same=same,
          first_diff=_first_diff(new.can, ref.can))
  assert same and new.cc.k9_creep is None and new.cc.k9_creep_mode == 0


# --- (b) hyundaicanfd.py: per function ---------------------------------------------------------------------------------
def _defs(source):
  tree = ast.parse(source)
  return {n.name: ast.dump(n) for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))}


def changed_canfd_functions():
  old, new = _defs(_git_show(REF, CANFD_REL)), _defs((ROOT / CANFD_REL).read_text(encoding="utf-8"))
  return sorted(n for n in set(old) | set(new) if old.get(n) != new.get(n))


CHANGED = changed_canfd_functions()


@pytest.fixture(scope="module")
def old_canfd(tmp_path_factory):
  return _load_source(_git_show(REF, CANFD_REL), tmp_path_factory.mktemp("cmp_canfd") / "old_hyundaicanfd.py",
                      "cmp_old_hyundaicanfd")


def test_changed_canfd_functions_listed():
  """Top-level defs of hyundaicanfd.py that differ from the reference (AST): recorded; a module-level statement change
  outside defs is checked too, so nothing outside the listed functions is hidden."""
  old_src, new_src = _git_show(REF, CANFD_REL), (ROOT / CANFD_REL).read_text(encoding="utf-8")

  def rest(src):
    return [ast.dump(n) for n in ast.parse(src).body if not isinstance(n, (ast.FunctionDef, ast.ClassDef))]
  _record("canfd_changed", functions=CHANGED, module_level_same=rest(old_src) == rest(new_src))
  assert rest(old_src) == rest(new_src)


@pytest.mark.parametrize("swap", CHANGED + ["ALL"])
@pytest.mark.parametrize("seq", list(RUNS))
@pytest.mark.parametrize("mode", [None, 0, 3, 5])
def test_canfd_function_effect_on_k9(old_canfd, swap, mode, seq):
  """This tree's K9 controller with one changed hyundaicanfd function (or all) put back to the reference version."""
  names = CHANGED if swap == "ALL" else [swap]
  calls = dict.fromkeys(CHANGED, 0)

  def counting(name, fn):
    def wrapper(*a, **k):
      calls[name] += 1
      return fn(*a, **k)
    return wrapper

  mp = pytest.MonkeyPatch()
  try:
    for n in CHANGED:
      if hasattr(hyundaicanfd, n):
        mp.setattr(hyundaicanfd, n, counting(n, getattr(hyundaicanfd, n)))
    new = RUNS[seq](Rig(mp, **_kw_new(mode)))
  finally:
    mp.undo()
  mp = pytest.MonkeyPatch()
  try:
    for n in names:
      mp.setattr(hyundaicanfd, n, getattr(old_canfd, n), raising=False)
    old = RUNS[seq](Rig(mp, **_kw_new(mode)))
  finally:
    mp.undo()
  same = new.can == old.can and new.scc14 == old.scc14
  _record("canfd_k9", swap=swap, mode=mode, seq=seq, calls=calls, frames=len(new.can), same=same)
  assert same


def _alt2_state(main, mode, info, sym, v, hold, driver):
  return SimpleNamespace(cruise_buttons_alt2={"LFA_BTN": 0, "CRUISE_BUTTONS": driver},
                         lfahda_cluster={"HDA_LFA_SymSta": sym, "HDA_CntrlModSta": 0}, MainMode_ACC=main, ACCMode=mode,
                         scc_control={"InfoDisplay": info},
                         out=SimpleNamespace(vEgo=v, brakeHoldActive=hold, parkingBrake=False))


def test_alt2_button_request_old_vs_new(old_canfd):
  """create_alt2_adas_button_request: new(keep_stock_scc_main=False) == old on the whole grid; new(True) differs from old
  only with CC.enabled False and MAIN off, where it adds the SCC MAIN pulse (CRUISE_BUTTONS 8)."""
  # one packer per variant: CANPacker keeps a per-message counter, a shared one would differ by call order only
  pk = {k: CANPacker("hyundai_canfd_generated") for k in ("old", "new", "new_false", "new_true")}
  can = SimpleNamespace(CAM=2)
  diff_keep, n = [], 0
  for frame, enabled, main, mode, info, sym, v, hold, driver, stopping in itertools.product(
      range(0, 400, 2), (False, True), (False, True), (0, 1, 4), (0, 4), (0, 1), (2., 20.), (False, True), (0, 2),
      (False, True)):
    cs, cc = _alt2_state(main, mode, info, sym, v, hold, driver), SimpleNamespace(enabled=enabled)
    old = old_canfd.create_alt2_adas_button_request(pk["old"], can, frame, cc, cs, stopping)
    assert hyundaicanfd.create_alt2_adas_button_request(pk["new"], can, frame, cc, cs, stopping) == old
    assert hyundaicanfd.create_alt2_adas_button_request(pk["new_false"], can, frame, cc, cs, stopping,
                                                         keep_stock_scc_main=False) == old
    new_keep = hyundaicanfd.create_alt2_adas_button_request(pk["new_true"], can, frame, cc, cs, stopping,
                                                            keep_stock_scc_main=True)
    n += 1
    if new_keep != old:
      diff_keep.append({"frame": frame, "enabled": enabled, "main": main, "mode": mode, "info": info, "sym": sym, "v": v,
                        "hold": hold, "driver": driver, "stopping": stopping, "old": old[1][10], "new": new_keep[1][10]})
  _record("alt2_grid", cases=n, differ_keep_true=len(diff_keep), sample=diff_keep[:5],
          conditions=sorted({(d["enabled"], d["main"], d["old"], d["new"]) for d in diff_keep}))
  assert all(not d["enabled"] and not d["main"] and d["old"] == 0 and d["new"] == 8 and 20 < d["frame"] % 200 <= 26
             for d in diff_keep)


def _ccnc_state(cs_out, main, mode, alt2, driver):
  from opendbc.car import structs
  cs = _alt2_state(main, mode, 4, 0, cs_out["vEgo"], False, 0)
  cs.out = structs.CarState(**cs_out)
  cs.modelV2 = cs.adrv_0x161 = cs.adrv_0x200 = cs.adrv_0x1ea = cs.ccnc_0x162 = None
  cs.cruise_buttons_msg = {"NORMAL_CRUISE_MAIN_BTN": 0, "ADAPTIVE_CRUISE_MAIN_BTN": 0, "LFA_BTN": 0, "CRUISE_BUTTONS": 0}
  cs.cruise_btns_msg_canfd = "CRUISE_BUTTONS_ALT"
  if driver:
    cs.cruise_buttons_msg["CRUISE_BUTTONS"] = 1
    cs.cruise_buttons_alt2["CRUISE_BUTTONS"] = 1
  if not alt2:
    cs.cruise_buttons_alt2 = None
  return cs


def test_ccnc_messages_old_vs_new(monkeypatch, old_canfd):
  """create_ccnc_messages, old vs new, all its messages compared: every differing input combination is recorded and must
  satisfy the new keep_stock_scc_main condition (CAMERA_SCC, openpilot long, corner radar > 0, drive, vEgo > 3, no brake)
  with CC.enabled False and MAIN off."""
  from opendbc.car import structs
  for m in (hyundaicanfd, old_canfd):
    monkeypatch.setattr(m, "Params", lambda: SimpleNamespace(get_int=lambda key: 0))
  can = SimpleNamespace(CAM=2, ECAN=0)
  diffs, n, addrs = [], 0, set()
  for alt2, longi, corner, camera, main, mode, enabled, v, brake, gear, driver in itertools.product(
      (False, True), (False, True), (0, 2), (False, True), (False, True), (0, 1), (False, True), (2., 20.), (False, True),
      ("drive", "park"), (False, True)):
    cp = structs.CarParams(flags=int(HyundaiFlags.CAMERA_SCC) if camera else 0, openpilotLongitudinalControl=longi)
    out = {"canValid": True, "gearShifter": gear, "vEgo": v, "brakePressed": brake}
    cc = SimpleNamespace(enabled=enabled, latActive=enabled)
    differ = []
    pk_new, pk_old = CANPacker("hyundai_canfd_generated"), CANPacker("hyundai_canfd_generated")  # own counters each
    for frame in range(0, 400, 2):
      tail = (SimpleNamespace(), 0, False, False, corner, False, 0, 0)
      for f in (hyundaicanfd.create_ccnc_messages, old_canfd.create_ccnc_messages):
        f.__dict__.pop("_lane_line_check", None)
      new = hyundaicanfd.create_ccnc_messages(cp, pk_new, can, frame, cc, _ccnc_state(out, main, mode, alt2, driver), *tail)
      old = old_canfd.create_ccnc_messages(cp, pk_old, can, frame, cc, _ccnc_state(out, main, mode, alt2, driver), *tail)
      if new != old:
        differ.append(frame)
        addrs.update(hex(x[0]) for x, y in itertools.zip_longest(new, old, fillvalue=(None,)) if x != y and x[0] is not None)
    n += 1
    if differ:
      diffs.append({"alt2": alt2, "longi": longi, "corner": corner, "camera": camera, "main": main, "mode": mode,
                    "enabled": enabled, "v": v, "brake": brake, "gear": gear, "driver": driver, "frames": differ})
  _record("ccnc_grid", configs=n, differ=len(diffs), addresses=sorted(addrs), sample=diffs[:4],
          frame_sets=sorted({tuple(sorted({f % 200 for f in d["frames"]})) for d in diffs}))
  for d in diffs:
    assert d["camera"] and d["longi"] and d["corner"] > 0 and d["gear"] == "drive" and d["v"] > 3 and not d["brake"]
    assert not d["enabled"] and not d["main"] and not d["driver"]
