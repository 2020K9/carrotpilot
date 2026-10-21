"""K9CreepMode single selector: 0 existing behaviour, 1 creep v8, 2 creep v9, 3 K9StopTaper explicitly.

Checks source logic and SCC12 encoding through the real CarController, not K9 vehicle behaviour.
"""
import pytest

from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V8, V9
from opendbc.car.hyundai.values import CAR
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import make_cp
from openpilot.selfdrive.car.tests.test_k9_stop_taper import LCS, Rig, base_cc, mixed_sequence  # noqa: F401


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


@pytest.mark.parametrize("values", [None, 0, 7, -1])
@pytest.mark.parametrize("taper", [False, True])
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
def test_mode0_identical_can_to_head_controller(monkeypatch, base_cc, values, taper, fingerprint):  # noqa: F811
  # base_cc is the pre-taper controller; with taper=True compare against the HEAD path (K9StopTaper only, mode absent)
  if taper:
    ref = mixed_sequence(rig(monkeypatch, None, taper=True, cp=make_cp(fingerprint=fingerprint)))
  else:
    ref = mixed_sequence(rig(monkeypatch, None, cp=make_cp(fingerprint=fingerprint), module=base_cc))
  monkeypatch.undo()
  new = mixed_sequence(rig(monkeypatch, values, taper=taper, cp=make_cp(fingerprint=fingerprint)))
  assert len(new.can) == 1800 and ref.can == new.can


def test_mode3_equals_taper_param_on_regardless_of_param(monkeypatch):
  ref = mixed_sequence(rig(monkeypatch, 0, taper=True))
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
