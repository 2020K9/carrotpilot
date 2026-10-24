"""K9StopMode menu trim (2026-10-24 evening, 보노보노 "브레이크 2.3.4 제거"): the 0-6 menu of 407bedaa (0 base, 1 v10,
2 v9 + shudder fix, 3 v10 + band 0.2, 4 v10 + shudder fix, 5 v10 + light shudder fix, 6 v11) became 0 base, 1 v10,
2 v10 + light shudder fix (old 5), 3 v11 (old 6). New tests (all new 2026-10-24 evening):
- through the real frozen 407bedaa CarController: new 0 / unset / 1 / 2 / 3 send byte-identical CAN (all messages, SCC12
  and SCC14 incl. accel, StopReq, jerk, ComfortBand) to old 0 / unset / 1 / 5 / 6 on the K9 and the K7, every sequence;
- the kept K9Creep kinds (1 v9, 2 v10, 6 v10 + light, 7 v11) are float-identical to 407bedaa's on random inputs (update and
  bands, state included);
- new 4, 5, 6 and other values act as 407bedaa 0;
- the one-time stored-value migration (openpilot/selfdrive/car/k9_stop_mode_migration.py) and its place in manager_init.
Checks source logic and CAN encoding; NOT verified on the vehicle."""
import math
import random

import numpy as np
import pytest

from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V9, V10, V10_LIGHT, V11
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.selfdrive.car import k9_stop_mode_migration as mig
from openpilot.selfdrive.car.tests.k9_rig import LCS, ROOT, Rig, load_frozen_cc
from openpilot.selfdrive.car.tests.test_k9_creep_mode import SEQS, _random_inputs
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import K9_FLAGS, make_cp

FROZEN = "407bedaa"  # carrot-wip head before the trim (K9StopMode 0-6)
OLD_OF_NEW = {None: None, 0: 0, 1: 1, 2: 5, 3: 6}


@pytest.fixture(scope="module")
def frozen_cc(tmp_path_factory):
  return load_frozen_cc(FROZEN, tmp_path_factory.mktemp("cc_407bedaa"), "cc_407bedaa")


def _kw(mode):
  return {} if mode is None else {"K9StopMode": mode}


def test_menu():
  assert kc.MENU == {1: V10, 2: V10_LIGHT, 3: V11} and kc.MODES == (1, 2, 3) and carcontroller.K9_CREEP_MENU is kc.MENU
  assert kc.KINDS == (V9, V10, V10_LIGHT, V11) == (1, 2, 6, 7)


def test_new_menu_runs_the_old_kinds(frozen_cc):
  old_menu = frozen_cc.K9_CREEP_MENU
  for new, old in OLD_OF_NEW.items():
    if new:
      assert kc.MENU[new] == old_menu[old]
  assert set(old_menu) == {1, 2, 3, 4, 5, 6}


@pytest.mark.parametrize("seq", list(SEQS))
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0, 1, 2, 3])
def test_new_menu_identical_can_to_407bedaa(monkeypatch, frozen_cc, mode, fingerprint, seq):
  ref = SEQS[seq](Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), module=frozen_cc, **_kw(OLD_OF_NEW[mode])))
  monkeypatch.undo()
  new = SEQS[seq](Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), **_kw(mode)))
  assert len(new.can) > 400 and ref.can == new.can
  assert ref.scc14 == new.scc14
  if fingerprint == CAR.KIA_K9 and mode:
    assert new.cc.k9_creep.mode == ref.cc.k9_creep.mode == kc.MENU[mode]


@pytest.mark.parametrize("seq", list(SEQS))
@pytest.mark.parametrize("mode", [1, 2, 3])
def test_new_menu_identical_can_to_407bedaa_off_k9_paths(monkeypatch, frozen_cc, mode, seq):
  cp = make_cp(flags=int(K9_FLAGS & ~HyundaiFlags.CAMERA_SCC))  # K9 without camera SCC: K9StopMode never applies
  ref = SEQS[seq](Rig(monkeypatch, cp=cp, module=frozen_cc, K9StopMode=OLD_OF_NEW[mode]))
  monkeypatch.undo()
  new = SEQS[seq](Rig(monkeypatch, cp=cp, K9StopMode=mode))
  assert ref.can == new.can and ref.scc14 == new.scc14 and new.cc.k9_creep is None
  monkeypatch.undo()
  r = Rig(monkeypatch, cp=make_cp(flags=int(K9_FLAGS | HyundaiFlags.CANFD)), K9StopMode=mode)  # CAN-FD: guard off
  assert r.cc.k9_creep_applies is False and r.cc.k9_creep is None


@pytest.mark.parametrize("value", [4, 5, 6, 7, 99, -1])
def test_removed_and_invalid_values_act_as_old_0(monkeypatch, frozen_cc, value):
  ref = SEQS["mixed"](Rig(monkeypatch, module=frozen_cc, K9StopMode=0))
  monkeypatch.undo()
  new = SEQS["mixed"](Rig(monkeypatch, K9StopMode=value))
  assert ref.can == new.can and new.cc.k9_creep is None and new.cc.k9_creep_mode == 0


def _st(c):
  return (c.latched, c.last, c.active, c.lb_u, c.lb_l, c.ss_count)


@pytest.mark.parametrize("seed", range(6))
@pytest.mark.parametrize("kind", (V9, V10, V10_LIGHT, V11))
def test_kept_kinds_object_identical_to_407bedaa(frozen_cc, kind, seed, monkeypatch):
  # 2026-10-24 20:26: kind 6 got the LIGHT_MIN_DECEL stop floor on purpose; this pin checks everything else (floor off)
  monkeypatch.setattr(kc, "LIGHT_MIN_DECEL", math.inf)
  rng = random.Random(f"k9-trim-frozen-{seed}")
  old, new = frozen_cc.K9Creep(kind), K9Creep(kind)
  assert old.__class__ is not K9Creep
  pu_o = pl_o = pu_n = pl_n = 0.0
  for kw, i in _random_inputs(rng, 4000):
    ro, rn = old.update(**kw), new.update(**kw)
    assert ro == rn or (math.isnan(ro[0]) and math.isnan(rn[0]) and ro[1] == rn[1]), (i, kw)
    so, sn = _st(old), _st(new)
    assert so == sn or all(a == b or (isinstance(a, float) and math.isnan(a) and math.isnan(b)) for a, b in zip(so, sn)), i
    if i % 2 == 0 and old.active:
      a0 = rn[0] if math.isfinite(rn[0]) else 0.0
      cu, cl = float(np.clip(0.9 + a0 * 0.2, 0, 1.2)), float(np.clip(0.8 + a0 * 0.2, 0, 1.2))
      ju = rng.uniform(0.5, 5.0)
      bo = old.bands(ju, kw["v_ego"], cu, cl, pu_o, pl_o)
      bn = new.bands(ju, kw["v_ego"], cu, cl, pu_n, pl_n)
      assert bo == bn, i
      (_, pu_o, pl_o), (_, pu_n, pl_n) = bo, bn


def test_live_switch_between_new_menus(monkeypatch, frozen_cc):
  """1 -> 2 -> 3 -> 0 live (every 0.5 s) sends the same CAN as 407bedaa switching 1 -> 5 -> 6 -> 0."""
  def drive(r, values, set_value):
    for value in values:
      set_value(r, value)
      for i in range(120):
        r.step(-0.6, LCS.pid, max(3.0 - 0.02 * i, 0.5), a_target=-0.4, a_ego=-0.3)
      for i in range(120):
        r.step(-0.7, LCS.stopping, max(0.5 - 0.004 * i, 0.0), standstill=i > 110, a_target=-0.3, a_ego=-0.3)
    return r

  def set_value(r, v):
    r.params.values["K9StopMode"] = v
  ref = drive(Rig(monkeypatch, module=frozen_cc, K9StopMode=1), (1, 5, 6, 0), set_value)
  monkeypatch.undo()
  new = drive(Rig(monkeypatch, K9StopMode=1), (1, 2, 3, 0), set_value)
  assert ref.can == new.can and ref.scc14 == new.scc14


# --- stored-value migration ---------------------------------------------------------------------------------------

class FakeParams:
  def __init__(self, **values):
    self.values = dict(values)
    self.writes = []

  def get(self, key):
    v = self.values.get(key)
    if isinstance(v, Exception):
      raise v
    return v

  def put_int(self, key, value):
    self.writes.append((key, value))
    self.values[key] = int(value)


def test_migration_table():
  assert mig.MENU_MIGRATION_V2 == {0: 0, 1: 1, 2: 1, 3: 1, 4: 1, 5: 2, 6: 3}
  assert mig.STOP_MODE_VER == 2 and mig.STOP_MODE_KEY == "K9StopMode" and mig.STOP_MODE_VER_KEY == "K9StopModeVer"
  for new in mig.MENU_MIGRATION_V2.values():
    assert new == 0 or new in kc.MENU


def test_migration_keeps_the_driven_kind(frozen_cc):
  old_menu = frozen_cc.K9_CREEP_MENU
  for old in (5, 6, 1):
    assert kc.MENU[mig.MENU_MIGRATION_V2[old]] == old_menu[old]  # same K9Creep kind = bit-identical behavior
  for old in (2, 3, 4):  # removed menus -> v10
    assert kc.MENU[mig.MENU_MIGRATION_V2[old]] == V10


@pytest.mark.parametrize("old,new", [(0, 0), (1, 1), (2, 1), (3, 1), (4, 1), (5, 2), (6, 3)])
def test_migrates_once(old, new):
  p = FakeParams(K9StopMode=old)
  assert mig.migrate_k9_stop_mode(p) is (old != new)
  assert p.values == {"K9StopMode": new, "K9StopModeVer": 2}
  # second start: marker present, never touched again (a new-menu 2 or 3 stays 2 or 3)
  p.writes.clear()
  assert mig.migrate_k9_stop_mode(p) is False and p.writes == [] and p.values["K9StopMode"] == new


def test_car_value_5_becomes_2_and_runs_the_same_kind(monkeypatch):
  """carrot-mici had K9StopMode 5 (v10 + light shudder fix) on 2026-10-24 18:58."""
  p = FakeParams(K9StopMode=5)
  mig.migrate_k9_stop_mode(p)
  assert p.values["K9StopMode"] == 2
  r = Rig(monkeypatch, K9StopMode=p.values["K9StopMode"])
  r.step(-0.5, LCS.pid, 3.0)
  assert r.cc.k9_creep.mode == V10_LIGHT


@pytest.mark.parametrize("value", [7, 8, 99, -1, -4, None])
def test_out_of_range_or_absent_left_as_stored(value):
  p = FakeParams(K9StopMode=value) if value is not None else FakeParams()
  assert mig.migrate_k9_stop_mode(p) is False
  assert p.values.get("K9StopMode") == value and p.values["K9StopModeVer"] == 2
  assert p.writes == [("K9StopModeVer", 2)]


@pytest.mark.parametrize("value", [True, False, 2.0, "5", b"5", ValueError("bad"), TypeError("bad")])
def test_unreadable_or_non_int_left_as_stored_and_never_raises(value):
  p = FakeParams(K9StopMode=value)
  assert mig.migrate_k9_stop_mode(p) is False
  assert ("K9StopMode", 1) not in p.writes and all(k == "K9StopModeVer" for k, _ in p.writes)


@pytest.mark.parametrize("ver", [2])
def test_marker_present_never_migrates(ver):
  for value in range(0, 7):
    p = FakeParams(K9StopMode=value, K9StopModeVer=ver)
    assert mig.migrate_k9_stop_mode(p) is False and p.writes == []


@pytest.mark.parametrize("ver", [None, 0, 1, 3, ValueError("bad")])
def test_other_marker_values_migrate(ver):
  p = FakeParams(K9StopMode=6, K9StopModeVer=ver)
  assert mig.migrate_k9_stop_mode(p) is True and p.values["K9StopMode"] == 3 and p.values["K9StopModeVer"] == 2


def test_write_failure_leaves_no_marker():
  class Failing(FakeParams):
    def put_int(self, key, value):
      if key == "K9StopMode":
        raise OSError("ro")
      super().put_int(key, value)
  p = Failing(K9StopMode=5)
  assert mig.migrate_k9_stop_mode(p) is False and p.values == {"K9StopMode": 5}  # next start tries again


def test_runs_in_manager_init_before_defaults_and_processes():
  src = (ROOT / "openpilot/system/manager/manager.py").read_text(encoding="utf-8")
  init = src[src.index("def manager_init()"):src.index("def manager_cleanup()")]
  assert "migrate_k9_stop_mode(params)" in init
  assert init.index("migrate_k9_stop_mode(params)") < init.index("for k in params.all_keys():")
  assert "from openpilot.selfdrive.car.k9_stop_mode_migration import migrate_k9_stop_mode" in src
  main = src[src.index("def main()"):]
  assert main.index("manager_init()") < main.index("manager_thread(")  # processes (card) start in manager_thread
  keys = (ROOT / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
  assert '{"K9StopModeVer", {PERSISTENT, INT}},' in keys  # no default: absent until the migration writes it
  mig_src = (ROOT / "openpilot/selfdrive/car/k9_stop_mode_migration.py").read_text(encoding="utf-8")
  assert "import opendbc" not in mig_src and "from opendbc" not in mig_src  # manager_init does not import car code for it


def test_controller_reads_k9stopmode_live_and_never_writes_it(monkeypatch):
  r = Rig(monkeypatch, K9StopMode=5)  # an unmigrated value on the new code acts as 0 (never a removed mode)
  r.step(-0.5, LCS.pid, 3.0)
  assert r.cc.k9_creep is None and r.params.values["K9StopMode"] == 5
  assert "K9StopModeVer" not in r.params.reads
