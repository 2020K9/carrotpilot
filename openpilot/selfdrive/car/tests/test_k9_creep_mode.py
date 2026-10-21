"""K9CreepMode single selector: 0 base braking (no taper), 1 creep v8, 2 creep v9, 3 the existing K9StopTaper taper.

Checks source logic and SCC12 encoding through the real CarController, not K9 vehicle behaviour.
"""
import importlib.util
import subprocess

import pytest

from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V8, V9
from opendbc.car import structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.selfdrive.car.tests import test_k9_stop_taper as tst
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import K9_FLAGS, make_cp
from openpilot.selfdrive.car.tests.test_k9_stop_taper import CC_REL, LCS, ROOT, Rig, mixed_sequence


def upd(c, **kw):
  args = dict(accel=-1.0, a_target=-0.5, stopping=True, pid=False, v_ego=0.3, v_ego_raw=0.3, a_ego=-0.3, long_accel=-0.3,
              esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True,
              lead_visible=False, lead_distance=0.0)
  args.update(kw)
  return c.update(**args)


def test_applies_same_guard_as_taper():
  assert kc.k9_creep_applies(make_cp())
  assert not kc.k9_creep_applies(make_cp(fingerprint=CAR.KIA_K7))
  cp = make_cp()
  cp.openpilotLongitudinalControl = False
  assert not kc.k9_creep_applies(cp)


def test_profile_constants_match_source():
  v8, v9 = kc.PROFILES[V8], kc.PROFILES[V9]
  assert v8["feather"] == ([0.05, 0.13, 0.22, 0.38], [-0.26, -0.28, -0.35, -0.48])
  assert v8["min_decel"] == ([0.08, 0.13, 0.25], [0.02, -0.06, -0.14])
  assert v8["creep"] == ([0.18, 0.40, 1.0, 1.6], [-0.18, -0.24, -0.20, 0.0])
  assert v9["feather"] == ([0.05, 0.14, 0.25, 0.42], [-0.31, -0.35, -0.42, -0.52])
  assert v9["creep"] == ([0.25, 0.55, 1.0, 1.6], [-0.26, -0.26, -0.20, 0.0])
  assert (v8["down_eff_ac"], v9["down_eff_ac"]) == (-0.03, -0.02)
  assert v8["up"](0.2, -0.05, 0.0) == 0.0 and v8["up"](0.2, -0.07, 0.0) == 0.0035 and v8["up"](0.1, 0.0, 0.0) == 0.0035
  assert v9["up"](0.05, 0.0, 0.0) == 0.0032 and v9["up"](0.2, 0.0, 0.02) == 0.0


@pytest.mark.parametrize("mode", [V8, V9])
def test_withholds_stopreq_then_latches(mode):
  c = K9Creep(mode)
  a, s = upd(c)
  assert s is False and a > -1.0
  for kw in ({"esp_standstill": True}, {"v_ego_raw": 0.01}, {"a_target": -1.0}, {"a_ego": 0.07},
             {"lead_visible": True, "lead_distance": 2.0}, {"long_accel": 0.2, "a_ego": 0.0}):
    c = K9Creep(mode)
    upd(c)
    a, s = upd(c, **kw)
    assert s is True and a <= -0.5


@pytest.mark.parametrize("mode", [V8, V9])
@pytest.mark.parametrize("kw", [{"gas": True}, {"brake": True}, {"override": True}, {"long_active": False},
                                {"can_valid": False}, {"v_ego": float("nan")}])
def test_interlocks_return_original(mode, kw):
  c = K9Creep(mode)
  upd(c)
  assert upd(c, **kw) == (-1.0, True) and not c.active


def test_v8_v9_differ():
  outs = []
  for mode in (V8, V9):
    c = K9Creep(mode)
    outs.append([upd(c, v_ego=v, v_ego_raw=v)[0] for v in (0.4, 0.38, 0.35, 0.3, 0.25, 0.2)])
  assert outs[0] != outs[1]


def gentle_stop(rig):
  """Stop with planned decel above the -1.0 urgent latch, so the creep feathering is exercised."""
  rows = []
  for _ in range(100):
    rows.append(rig.step(-0.5, LCS.pid, 3.0, a_target=-0.5, a_ego=-0.5))
  for i in range(150):
    v = max(1.0 - 0.006 * i, 0.06)
    rows.append(rig.step(-0.6, LCS.pid if v > 0.5 else LCS.stopping, v, a_target=-0.4, a_ego=-0.3))
  for _ in range(200):
    rows.append(rig.step(-0.6, LCS.stopping, 0.0, standstill=True, a_target=-0.4, a_ego=0.0))
  return rows


def rig(monkeypatch, mode, taper=False, cp=None, module=None):
  kw = {} if mode is None else {"K9CreepMode": mode}
  return Rig(monkeypatch, cp=cp, taper=taper, **({"module": module} if module else {}), **kw)


PARENT_COMMIT = "e04c05e4"  # actual parent of the K9CreepMode commit (K9StopTaper param only, no selector)


@pytest.fixture(scope="module")
def parent_cc(tmp_path_factory):
  """Frozen parent CarController. k9_stop_taper.py, hyundaican.py etc. are byte-identical at HEAD (only
  carcontroller.py/k9_creep.py changed under opendbc_repo), so the parent module runs against the same deps."""
  changed = subprocess.run(["git", "diff", "--name-only", PARENT_COMMIT, "HEAD", "--", "opendbc_repo"], cwd=ROOT,
                           check=True, capture_output=True, text=True).stdout.split()
  assert set(changed) <= {CC_REL, "opendbc_repo/opendbc/car/hyundai/k9_creep.py"}, changed
  source = subprocess.run(["git", "show", f"{PARENT_COMMIT}:{CC_REL}"], cwd=ROOT, check=True, capture_output=True,
                          text=True, encoding="utf-8").stdout
  assert "k9_stop_taper_enabled" in source and "K9CreepMode" not in source and "k9_creep" not in source
  path = tmp_path_factory.mktemp("parent_cc") / "carcontroller_parent.py"
  path.write_text(source, encoding="utf-8")
  spec = importlib.util.spec_from_file_location("carcontroller_k9_creep_parent", path)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  assert module.__file__ == str(path) and module.__file__ != carcontroller.__file__
  return module


@pytest.mark.parametrize("values", [None, 0, 4, 7, -1])
@pytest.mark.parametrize("taper", [False, True])
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
def test_mode0_identical_can_to_actual_parent(monkeypatch, parent_cc, values, taper, fingerprint):
  # reference: the real parent e04c05e4 controller (it never reads K9CreepMode). On K9 mode 0 is the parent with the
  # taper OFF whatever the old K9StopTaper value; other cars keep the parent under the same old value.
  ref_taper = False if fingerprint == CAR.KIA_K9 else taper
  ref = mixed_sequence(rig(monkeypatch, None, taper=ref_taper, cp=make_cp(fingerprint=fingerprint), module=parent_cc))
  monkeypatch.undo()
  new = mixed_sequence(rig(monkeypatch, values, taper=taper, cp=make_cp(fingerprint=fingerprint)))
  assert len(new.can) == 1800 and ref.can == new.can


@pytest.mark.parametrize("taper", [False, True])
def test_mode3_identical_can_to_actual_parent_taper_on(monkeypatch, parent_cc, taper):
  ref = mixed_sequence(rig(monkeypatch, None, taper=True, module=parent_cc))
  monkeypatch.undo()
  assert mixed_sequence(rig(monkeypatch, 3, taper=taper)).can == ref.can


@pytest.mark.parametrize("mode", [1, 2, 3])
def test_no_camera_scc_k9_identical_to_parent(monkeypatch, parent_cc, mode):
  flags = int(K9_FLAGS & ~HyundaiFlags.CAMERA_SCC)
  assert not kc.k9_creep_applies(make_cp(flags=flags))
  ref = mixed_sequence(rig(monkeypatch, None, taper=True, cp=make_cp(flags=flags), module=parent_cc))
  monkeypatch.undo()
  r = rig(monkeypatch, mode, taper=True, cp=make_cp(flags=flags))
  assert mixed_sequence(r).can == ref.can and r.cc.k9_creep is None and r.cc.k9_creep_mode == 0


@pytest.mark.parametrize("mode", [1, 2, 3])
def test_canfd_k9_guard_false(monkeypatch, mode):
  # the shared Rig cannot drive a full CAN-FD update (no CAN-FD CarState fixture); check the guard at construction
  cp = make_cp(flags=int(K9_FLAGS | HyundaiFlags.CANFD))
  assert not kc.k9_creep_applies(cp)
  r = rig(monkeypatch, mode, taper=True, cp=cp)
  assert r.cc.k9_creep_applies is False and r.cc.k9_creep is None and r.cc.k9_stop_taper is None


def test_canfd_jerk_branch_ignores_creep_flag():
  # even if creep=True reached make_jerk, the CAN-FD branch never applies jerk_bands
  def run(creep):
    j = carcontroller.HyundaiJerk()
    cp = make_cp(flags=int(K9_FLAGS | HyundaiFlags.CANFD))
    cs = tst.make_ci_state()
    cs.out = structs.CarState(vEgo=0.5, aEgo=-0.3)
    act = structs.CarControl().actuators
    act.longControlState, act.accel = LCS.stopping, -0.6
    j.make_jerk(cp, cs, -0.6, act, structs.CarControl().hudControl, creep=creep)
    return j.jerk_u, j.jerk_l, j.cb_upper, j.cb_lower
  assert run(True) == run(False)


class FakeCp:
  def __init__(self, vl):
    self.vl = vl


def patch_state(monkeypatch, **attrs):
  orig = tst.make_ci_state

  def make():
    cs = orig()
    for k, v in attrs.items():
      setattr(cs, k, v)
    return cs
  monkeypatch.setattr(tst, "make_ci_state", make)


@pytest.mark.parametrize("mode", [1, 2])
def test_soft_hold_mode2_keeps_stopreq_like_parent(monkeypatch, parent_cc, mode):
  # hyundaican: StopReq = stopping or (softHoldActive > 0 and soft_hold_mode == 2); the creep must not defeat it
  patch_state(monkeypatch, softHoldActive=1)
  ref = gentle_stop(rig(monkeypatch, None, module=parent_cc))
  cand = gentle_stop(rig(monkeypatch, mode))
  assert all(c is None or c["StopReq"] == 1 for _, c in cand)
  assert [r[1]["StopReq"] for r in ref if r[1]] == [c[1]["StopReq"] for c in cand if c[1]]


@pytest.mark.parametrize("vl", [{}, {"ESP12": {}}, {"ESP12": {"LONG_ACCEL": float("nan")}, "TCS13": {"StandStill": 0}},
                                {"ESP12": {"LONG_ACCEL": float("inf")}}], ids=["absent", "no_signal", "nan", "inf"])
@pytest.mark.parametrize("mode", [1, 2])
def test_missing_or_invalid_esp12_tcs13_falls_back_to_aego(monkeypatch, mode, vl):
  assert kc.read_long_accel(FakeCp(vl)) is None and kc.read_long_accel(None) is None
  ref = gentle_stop(rig(monkeypatch, mode))  # make_ci_state has no cp: same aEgo fallback, no TCS13 standstill
  monkeypatch.undo()
  patch_state(monkeypatch, cp=FakeCp(vl))
  assert gentle_stop(rig(monkeypatch, mode)) == ref


@pytest.mark.parametrize("mode", [1, 2])
def test_tcs13_standstill_latches_stopreq(monkeypatch, mode):
  patch_state(monkeypatch, cp=FakeCp({"TCS13": {"StandStill": 1}, "ESP12": {"LONG_ACCEL": -0.3}}))
  rows = gentle_stop(rig(monkeypatch, mode))
  assert all(c is None or c["StopReq"] == 1 for _, c in rows[184:])  # first stopping frame (v <= 0.5) onwards


def jerk_trace(monkeypatch, mode, module=None):
  r = rig(monkeypatch, mode, module=module)
  out = []
  for state, n in ((LCS.pid, 100), (LCS.stopping, 150), (LCS.off, 50)):
    for i in range(n):
      v = max(1.0 - 0.006 * i, 0.06) if state != LCS.pid else 3.0
      r.step(-0.6, state, v, a_target=-0.4, a_ego=-0.3)
      j = r.cc.hyundai_jerk
      out.append((state, float(j.jerk_u), float(j.jerk_l), float(j.cb_upper), float(j.cb_lower)))
  return out


@pytest.mark.parametrize("mode", [1, 2])
def test_jerk_comfortband_override_only_on_applied_non_off_frames(monkeypatch, parent_cc, mode):
  ref = jerk_trace(monkeypatch, None, module=parent_cc)
  monkeypatch.undo()
  cand = jerk_trace(monkeypatch, mode)
  monkeypatch.undo()
  assert jerk_trace(monkeypatch, 0) == ref  # mode 0 jerk/ComfortBand identical to the actual parent
  for (st, *r), (_, ju, jl, cu, cl) in zip(ref, cand, strict=True):
    assert jl == r[1]  # jerk_l never touched
    if st == LCS.off:
      assert (ju, cu, cl) == (r[0], r[2], r[3])
    else:
      assert cu == 0.0 and cl == 0.0 and ju >= r[0]
  assert any(r[2] > 0 for r in ref)  # parent really had a non-zero band there


@pytest.mark.parametrize("mode", [1, 2])
def test_jerk_band_original_on_interlocked_frames(monkeypatch, parent_cc, mode):
  def trace(m, module=None):
    r = rig(monkeypatch, m, module=module)
    out = []
    for i in range(200):
      r.step(-0.6, LCS.stopping, 0.4, a_target=-0.4, a_ego=-0.3, gas=i >= 100)
      j = r.cc.hyundai_jerk
      out.append((float(j.jerk_u), float(j.cb_upper), float(j.cb_lower)))
    return out
  ref = trace(None, parent_cc)
  monkeypatch.undo()
  cand = trace(mode)
  assert cand[102:] == ref[102:]  # gas interlock: creep reset, original jerk/ComfortBand (make_jerk runs every 2nd frame)


def test_mode3_equals_taper_param_on_regardless_of_param(monkeypatch, parent_cc):
  ref = mixed_sequence(rig(monkeypatch, None, taper=True, module=parent_cc))  # parent with the taper ON
  monkeypatch.undo()
  for taper in (False, True):
    assert mixed_sequence(rig(monkeypatch, 3, taper=taper)).can == ref.can
    monkeypatch.undo()


@pytest.mark.parametrize("mode", [1, 2])
def test_creep_modes_change_k9_scc12_only_and_ignore_taper(monkeypatch, mode):
  base = gentle_stop(rig(monkeypatch, 0))
  monkeypatch.undo()
  cand = gentle_stop(rig(monkeypatch, mode))
  monkeypatch.undo()
  cand_t = gentle_stop(rig(monkeypatch, mode, taper=True))
  assert cand == cand_t  # never stacked with K9StopTaper
  withheld = [i for i, ((_, b), (_, c)) in enumerate(zip(base, cand, strict=True))
              if b is not None and b["StopReq"] == 1 and c["StopReq"] == 0]
  assert withheld and all(100 <= i < 250 for i in withheld)
  assert all(c is None or c["StopReq"] == 1 for _, c in cand[260:450])  # latched at standstill
  assert all(c is None or c["aReqValue"] <= -0.5 + 0.006 for _, c in cand[260:450])
  monkeypatch.undo()
  k7 = make_cp(fingerprint=CAR.KIA_K7)
  ref7 = gentle_stop(rig(monkeypatch, 0, cp=k7))
  monkeypatch.undo()
  assert gentle_stop(rig(monkeypatch, mode, cp=make_cp(fingerprint=CAR.KIA_K7))) == ref7


def test_v8_and_v9_controller_outputs_differ(monkeypatch):
  a = gentle_stop(rig(monkeypatch, 1))
  monkeypatch.undo()
  assert gentle_stop(rig(monkeypatch, 2)) != a


def test_live_mode_change_resets_profiles(monkeypatch):
  r = rig(monkeypatch, 2)
  for _ in range(60):
    r.step(-1.0, LCS.pid, 3.0)
  for i in range(60):
    r.step(-1.2, LCS.stopping, max(0.5 - 0.005 * i, 0.1))
  assert r.cc.k9_creep is not None and r.cc.k9_creep.mode == 2 and r.cc.k9_creep.last is not None
  r.params.values["K9CreepMode"] = 1
  for _ in range(60):  # >= one 0.5 s read period
    r.step(-1.2, LCS.stopping, 0.1)
  assert r.cc.k9_creep.mode == 1
  r.params.values["K9CreepMode"] = 0
  out = [r.step(-1.2, LCS.stopping, 0.1) for _ in range(60)]
  assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0
  sent = [o for o in out[-10:] if o[1] is not None]
  assert sent and all(o[1]["StopReq"] == 1 and o[0] == pytest.approx(-1.2) for o in sent)


def test_setting_registered_single_selector():
  import json
  from openpilot.selfdrive.car.tests.test_k9_stop_taper import ROOT
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  entry = next(p for p in catalog["params"] if p["name"] == "K9CreepMode")
  assert (entry["min"], entry["max"], entry["default"], entry["unit"]) == (0, 3, 0, 1)
  assert not any(p["name"] in ("K9CreepV9", "K9CreepV8") for p in catalog["params"])
  keys = (ROOT / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
  assert '{"K9CreepMode", {PERSISTENT, INT, "0"}},' in keys and "K9CreepV9" not in keys
