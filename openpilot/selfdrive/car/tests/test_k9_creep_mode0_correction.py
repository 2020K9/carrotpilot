"""Owner correction 2026-10-22: K9CreepMode 0 = base braking without the existing K9 pre-stop brake release (taper),
1 = creep v8, 2 = creep v9, 3 = the existing taper. The old K9StopTaper param is ignored on eligible K9s.

References are the actual frozen controllers: e04c05e4 (before K9CreepMode; K9StopTaper param only) and 8390d111
(K9CreepMode before this correction). Checks source logic and sent CAN, not K9 vehicle behaviour.
"""
import importlib.util
import subprocess

import pytest

from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.k9_stop_taper import TaperPhase
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.selfdrive.car.tests.test_k9_creep_mode import gentle_stop, jerk_trace
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import K9_FLAGS, make_cp
from openpilot.selfdrive.car.tests.test_k9_stop_taper import (
  CC_REL, LCS, ROOT, Rig, _taper_until_active, edge_sequence, mixed_sequence, stop_sequence,
)

PRE_SELECTOR = "e04c05e4"  # parent of the K9CreepMode commits
PRE_CORRECTION = "8390d111"  # K9CreepMode before this correction (mode 0 still honoured K9StopTaper)
CREEP_REL = "opendbc_repo/opendbc/car/hyundai/k9_creep.py"
OLD_PARAM = [{}, {"K9StopTaper": False}, {"K9StopTaper": True}]
OLD_IDS = ["old_missing", "old_false", "old_true"]
SEQUENCES = [mixed_sequence, edge_sequence]


def _git(*args):
  return subprocess.run(["git", *args], cwd=ROOT, check=True, capture_output=True, text=True, encoding="utf-8").stdout


def _load(commit, tmp_path_factory):
  source = _git("show", f"{commit}:{CC_REL}")
  path = tmp_path_factory.mktemp(f"cc_{commit}") / f"carcontroller_{commit}.py"
  path.write_text(source, encoding="utf-8")
  spec = importlib.util.spec_from_file_location(f"carcontroller_frozen_{commit}", path)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  assert module.__file__ == str(path) and module.__file__ != carcontroller.__file__
  return module, source


def _deps_unchanged_since(commit):
  # frozen controllers import the working tree's helpers: only carcontroller.py and k9_creep.py may differ, and
  # k9_creep.py only in its module docstring
  changed = set(_git("diff", "--name-only", commit, "--", "opendbc_repo").split())
  assert changed <= {CC_REL, CREEP_REL}, changed
  if CREEP_REL in _git("ls-tree", "-r", "--name-only", commit, "--", CREEP_REL).split():
    old_code = _git("show", f"{commit}:{CREEP_REL}").split('"""', 2)[2]
    assert (ROOT / CREEP_REL).read_text(encoding="utf-8").split('"""', 2)[2] == old_code


@pytest.fixture(scope="module")
def pre_selector(tmp_path_factory):
  _deps_unchanged_since(PRE_SELECTOR)
  module, source = _load(PRE_SELECTOR, tmp_path_factory)
  assert "K9CreepMode" not in source and 'get_bool("K9StopTaper")' in source
  return module


@pytest.fixture(scope="module")
def pre_correction(tmp_path_factory):
  _deps_unchanged_since(PRE_CORRECTION)
  module, source = _load(PRE_CORRECTION, tmp_path_factory)
  assert "K9CreepMode" in source and "self.k9_creep_mode == 0 and self.k9_stop_taper_enabled" in source
  return module


def _run(monkeypatch, seq, values, cp=None, module=carcontroller, taper=False):
  rig = Rig(monkeypatch, cp=cp, taper=taper, module=module, **values)
  seq(rig)  # mixed/edge return the rig, stop_sequence returns per-frame rows; the CAN log lives on the rig
  monkeypatch.undo()
  return rig


def test_candidate_never_reads_old_param(monkeypatch):
  rig = Rig(monkeypatch, taper=True)
  stop_sequence(rig)
  assert "K9CreepMode" in rig.params.reads and "K9StopTaper" not in rig.params.reads
  assert 'get_bool("K9StopTaper")' not in (ROOT / CC_REL).read_text(encoding="utf-8")


@pytest.mark.parametrize("seq", SEQUENCES, ids=["mixed", "edge"])
@pytest.mark.parametrize("old", OLD_PARAM, ids=OLD_IDS)
@pytest.mark.parametrize("mode", [None, 0, 4, -1], ids=["mode_missing", "mode0", "mode4", "mode_neg1"])
def test_mode0_matches_pre_selector_taper_off_byte_for_byte(monkeypatch, pre_selector, seq, old, mode):
  ref = _run(monkeypatch, seq, {}, module=pre_selector)  # actual e04c05e4, K9StopTaper missing = taper OFF
  values = {**old, **({} if mode is None else {"K9CreepMode": mode})}
  new = _run(monkeypatch, seq, values)
  assert len(new.can) in (1800, 1280) and new.can == ref.can
  assert new.cc.k9_stop_taper is not None and not new.cc.k9_stop_taper_enabled


@pytest.mark.parametrize("old", OLD_PARAM, ids=OLD_IDS)
def test_mode0_matches_pre_correction_mode0_with_taper_off(monkeypatch, pre_correction, old):
  ref = _run(monkeypatch, mixed_sequence, {"K9CreepMode": 0, "K9StopTaper": False}, module=pre_correction)
  assert _run(monkeypatch, mixed_sequence, {"K9CreepMode": 0, **old}).can == ref.can


@pytest.mark.parametrize("seq", SEQUENCES + [stop_sequence], ids=["mixed", "edge", "stop"])
@pytest.mark.parametrize("old", OLD_PARAM, ids=OLD_IDS)
def test_mode3_matches_pre_selector_taper_on_regardless_of_old_param(monkeypatch, pre_selector, pre_correction, seq,
                                                                        old):
  ref = _run(monkeypatch, seq, {"K9StopTaper": True}, module=pre_selector)  # actual e04c05e4 taper ON
  ref8390 = _run(monkeypatch, seq, {"K9CreepMode": 3, **old}, module=pre_correction)
  new = _run(monkeypatch, seq, {"K9CreepMode": 3, **old})
  assert new.can == ref.can == ref8390.can and new.cc.k9_stop_taper_enabled


def _scc12(rig):
  from opendbc.can import CANParser
  parser = CANParser("hyundai_kia_generic", [("SCC12", 0)], 0)
  out = []
  for i, frame in enumerate(rig.can):
    msgs = [m for m in frame if m[0] == 1057]
    if msgs:
      parser.update([i * 10_000_000, msgs])
      out.append((parser.vl["SCC12"]["StopReq"], parser.vl["SCC12"]["aReqValue"]))
  return out


@pytest.mark.parametrize("old", OLD_PARAM, ids=OLD_IDS)
def test_mode0_differs_from_mode3_only_in_triggered_taper_accel(monkeypatch, old):
  m0 = _run(monkeypatch, stop_sequence, {"K9CreepMode": 0, **old})
  m3 = _run(monkeypatch, stop_sequence, {"K9CreepMode": 3, **old})
  a0, a3 = _scc12(m0), _scc12(m3)
  assert len(a0) == len(a3) and a0 != a3
  diffs = [(s0, v0, s3, v3) for (s0, v0), (s3, v3) in zip(a0, a3, strict=True) if (s0, v0) != (s3, v3)]
  assert diffs and all(s0 == s3 for s0, _, s3, _ in diffs)  # StopReq identical; only aReq differs
  assert all(v3 > v0 for _, v0, _, v3 in diffs)  # the taper only weakens the request
  # the taper actually triggered in mode 3 and never in mode 0
  r0 = Rig(monkeypatch, taper=bool(old.get("K9StopTaper")), K9CreepMode=0)
  with pytest.raises(AssertionError, match="no taper entry"):
    _taper_until_active(r0)
  monkeypatch.undo()
  r3 = Rig(monkeypatch, taper=bool(old.get("K9StopTaper")), K9CreepMode=3)
  _taper_until_active(r3)


@pytest.mark.parametrize("mode", [0, 1, 2, 3])
@pytest.mark.parametrize("old", [False, True])
@pytest.mark.parametrize("case", ["k7", "no_camera_scc", "no_op_long"])
def test_nonapplicable_cars_unchanged_vs_pre_selector(monkeypatch, pre_selector, mode, old, case):
  if case == "k7":
    cp = lambda: make_cp(fingerprint=CAR.KIA_K7)
  elif case == "no_camera_scc":
    cp = lambda: make_cp(flags=int(K9_FLAGS & ~HyundaiFlags.CAMERA_SCC))
  else:
    def cp():
      c = make_cp()
      c.openpilotLongitudinalControl = False
      return c
  ref = _run(monkeypatch, mixed_sequence, {"K9StopTaper": old}, cp=cp(), module=pre_selector)
  new = _run(monkeypatch, mixed_sequence, {"K9StopTaper": old, "K9CreepMode": mode}, cp=cp())
  assert new.can == ref.can and new.cc.k9_stop_taper is None and new.cc.k9_creep is None


@pytest.mark.parametrize("mode", [1, 2])
@pytest.mark.parametrize("old", OLD_PARAM, ids=OLD_IDS)
def test_modes12_identical_to_pre_correction(monkeypatch, pre_correction, mode, old):
  ref = gentle_stop(Rig(monkeypatch, taper=False, module=pre_correction, K9CreepMode=mode, **old))
  monkeypatch.undo()
  assert gentle_stop(Rig(monkeypatch, taper=False, K9CreepMode=mode, **old)) == ref  # accel + StopReq per frame
  monkeypatch.undo()
  ref = _run(monkeypatch, mixed_sequence, {"K9CreepMode": mode, **old}, module=pre_correction)
  assert _run(monkeypatch, mixed_sequence, {"K9CreepMode": mode, **old}).can == ref.can


@pytest.mark.parametrize("mode", [0, 1, 2, 3])
def test_jerk_comfortband_identical_to_pre_correction(monkeypatch, pre_correction, mode):
  # pre-correction mode 0 with K9StopTaper missing; ComfortBand 0 only appears in 1/2 (checked in test_k9_creep_mode)
  ref = jerk_trace(monkeypatch, mode, module=pre_correction)
  monkeypatch.undo()
  cand = jerk_trace(monkeypatch, mode)
  assert cand == ref


def _drive_to_taper(rig):
  _taper_until_active(rig)
  assert rig.cc.k9_stop_taper.phase == TaperPhase.taper


def _settle(rig, n=60):  # >= one 0.5 s param read period
  return [rig.step(-1.2, LCS.stopping, 0.3) for _ in range(n)]


@pytest.mark.parametrize("to_mode", [0, 1, 2])
def test_leaving_mode3_mid_taper_resets_and_returns_original(monkeypatch, to_mode):
  rig = Rig(monkeypatch, taper=True, K9CreepMode=3)
  _drive_to_taper(rig)
  rig.params.values["K9CreepMode"] = to_mode
  _settle(rig)
  t = rig.cc.k9_stop_taper
  assert rig.cc.k9_creep_mode == to_mode and not rig.cc.k9_stop_taper_enabled
  assert t.phase == TaperPhase.idle and not t.armed
  if to_mode == 0:
    out = _settle(rig, 10)
    assert all(a == pytest.approx(-1.2, abs=1e-6) for a, _ in out)
  else:
    assert rig.cc.k9_creep is not None and rig.cc.k9_creep.mode == to_mode


@pytest.mark.parametrize("from_mode", [0, 1, 2])
def test_entering_mode3_mid_stop_starts_fresh(monkeypatch, from_mode):
  rig = Rig(monkeypatch, taper=True, K9CreepMode=from_mode)
  for _ in range(100):
    rig.step(-1.0, LCS.pid, 5.0)
  for i in range(60):
    rig.step(-1.2, LCS.stopping, max(1.4 - 0.01 * i, 0.2))
  rig.params.values["K9CreepMode"] = 3
  out = _settle(rig)
  t = rig.cc.k9_stop_taper
  assert rig.cc.k9_creep is None and rig.cc.k9_stop_taper_enabled
  assert t.phase == TaperPhase.idle and not t.armed  # needs a fresh run above REARM_SPEED, no mid-stop entry
  assert all(a == pytest.approx(-1.2, abs=1e-6) for a, _ in out[-10:])


def test_mode_cycle_0_1_2_3_0_resets_every_time(monkeypatch):
  rig = Rig(monkeypatch, taper=True, K9CreepMode=0)
  seen = []
  for mode in (1, 2, 3, 0, 3):
    rig.params.values["K9CreepMode"] = mode
    _settle(rig)
    seen.append((rig.cc.k9_creep_mode, rig.cc.k9_creep.mode if rig.cc.k9_creep else None, rig.cc.k9_stop_taper_enabled,
                 rig.cc.k9_stop_taper.phase))
  assert seen == [(1, 1, False, TaperPhase.idle), (2, 2, False, TaperPhase.idle), (3, None, True, TaperPhase.idle),
                  (0, None, False, TaperPhase.idle), (3, None, True, TaperPhase.idle)]


@pytest.mark.parametrize("mode", [0, 1, 2, 3])
@pytest.mark.parametrize("kw", [{"gas": True}, {"brake": True}], ids=["gas", "brake"])
def test_interlock_returns_original_at_once_in_every_mode(monkeypatch, mode, kw):
  rig = Rig(monkeypatch, taper=True, K9CreepMode=mode)
  for _ in range(100):
    rig.step(-1.0, LCS.pid, 5.0)
  for i in range(110):
    rig.step(-0.6, LCS.stopping, max(1.4 - 0.01 * i, 0.3), a_target=-0.4, a_ego=-0.3)
  accel, sent = rig.step(-1.2, LCS.stopping, 0.3, **kw)
  assert accel == pytest.approx(-1.2, abs=1e-6)
  if sent is not None:
    assert sent["StopReq"] == 1
