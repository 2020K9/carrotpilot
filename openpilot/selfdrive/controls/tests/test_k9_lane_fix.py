"""K9LaneFix (experimental menu number, NOT vehicle-validated): OFF/ON identity, switching reset and menu schema.

(a) K9LaneFix=0 (and unknown numbers) gives exactly the parent (685223c8) outputs: lane_planner_2.get_d_path and the
    controlsd.state_control lateral command are compared bit for bit with frozen copies of the parent files.
(b) K9LaneFix=1 gives exactly the LANEFIX4 "F1" outputs (frozen copy of the replay / simulation tree_F1 files,
    lane_planner_2.py sha256 9d7bbe26..., controlsd.py sha256 1a2dbb64...). Tolerance: 0 (np.array_equal / ==).
(c) changing the number restarts the K9LaneFix transition state.
(d) the param is registered, in the 'EXPERIMENTAL' (실험중) menu right after K9StopMode, and described per number.
Inputs are synthetic random sequences (several seeds) built to cross lane-mode on/off, lane change, lateral off and
large lane offsets. Frozen references live in k9_lane_fix_ref/*.py.ref (not imported by production code).
"""
import ast
import json
import math
import re
import types
from numbers import Number
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.cereal import car, log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.selfdrive.controls.lib import drive_helpers as DH
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_lag_adjusted_curvature
from openpilot.selfdrive.controls.lib import k9_lane_fix as K9
import openpilot.selfdrive.controls.lib.lane_planner_2 as LP_NEW

HERE = Path(__file__).parent
REF = HERE / "k9_lane_fix_ref"
CONTROLS = HERE.parent
ROOT = CONTROLS.parents[2]
SEEDS = [0, 1, 2, 3, 4, 5]


class MemParams:
  def __init__(self, **values):
    self.values = dict(values)

  def get_int(self, k, *a, **kw):
    return int(self.values.get(k, 0))

  def get_float(self, k, *a, **kw):
    return float(self.values.get(k, 0.0))

  def get_bool(self, k, *a, **kw):
    return bool(self.values.get(k, False))


def load_module(path, name):
  mod = types.ModuleType(name)
  mod.__file__ = str(path)
  exec(compile(path.read_text(encoding="utf8"), str(path), "exec"), mod.__dict__)
  return mod


# ---------------------------------------------------------------- lane planner
LP_PARENT = load_module(REF / "lane_planner_2.parent.py.ref", "k9_ref_lp_parent")
LP_F1 = load_module(REF / "lane_planner_2.F1.py.ref", "k9_ref_lp_F1")
N = LP_NEW.TRAJECTORY_SIZE
PATH_T = (np.linspace(0.0, 1.0, N) ** 2) * 10.0


def make_lp(mod, params):
  saved = mod.Params
  mod.Params = lambda: params
  try:
    return mod.LanePlanner()
  finally:
    mod.Params = saved


def lp_frames(seed, n=600):
  rng = np.random.default_rng(seed)
  v = 15.0; off = 0.0; curv = 0.0; pl = 0.9; pr = 0.9; frames = []
  for k in range(n):
    v = float(np.clip(v + rng.normal(0, 0.4), 2.0, 36.0))
    off = float(np.clip(off + rng.normal(0, 0.05) + (rng.normal(0, 0.6) if rng.random() < 0.01 else 0.0), -1.2, 1.2))
    curv = float(np.clip(curv + rng.normal(0, 2e-4), -0.02, 0.02))
    pl = float(np.clip(pl + rng.normal(0, 0.05) + (-0.8 if rng.random() < 0.006 else 0.0), 0.0, 1.0))
    pr = float(np.clip(pr + rng.normal(0, 0.05) + (-0.8 if rng.random() < 0.006 else 0.0), 0.0, 1.0))
    if rng.random() < 0.08:
      pl, pr = min(pl + 0.5, 1.0), min(pr + 0.5, 1.0)
    x = v * PATH_T
    lane_c = off + 0.5 * curv * x ** 2
    ll_t = PATH_T.copy()
    if rng.random() < 0.1:
      ll_t[-int(rng.integers(1, 10)):] = np.nan
    model_y = 0.5 * (curv + rng.normal(0, 3e-4)) * x ** 2 + rng.normal(0, 0.02) * PATH_T
    model_y[0] = 0.0
    path = np.stack([x, model_y, np.zeros(N)], axis=1)
    w = 3.3 + rng.normal(0, 0.15)
    frames.append(dict(v=v, ll_t=ll_t, ll_x=x.copy(), lll_y=lane_c - w / 2 + rng.normal(0, 0.01, N), rll_y=lane_c + w / 2 + rng.normal(0, 0.01, N),
                       lll_prob=pl, rll_prob=pr, lll_std=float(rng.uniform(0.25, 0.35) if rng.random() < 0.01 else rng.uniform(0.03, 0.14)), rll_std=float(rng.uniform(0.03, 0.14)),
                       lanefull_mode=bool(rng.random() > 0.03), lane_change_multiplier=0.0 if rng.random() < 0.02 else 1.0,
                       lane_width_left=float(rng.uniform(1.5, 3.6)), lane_width_right=float(rng.uniform(1.5, 3.6)),
                       path=path, curve_speed=float(rng.uniform(-250, 250))))
  return frames


def lp_step(planner, f):
  for key in ("ll_t", "ll_x", "lll_y", "rll_y", "lll_prob", "rll_prob", "lll_std", "rll_std", "lanefull_mode",
              "lane_change_multiplier", "lane_width_left", "lane_width_right"):
    setattr(planner, key, f[key].copy() if isinstance(f[key], np.ndarray) else f[key])
  path, active = planner.get_d_path(None, f["v"], PATH_T, f["path"].copy(), f["curve_speed"])
  return path, active


def run_lp(planner, frames, modes=None, params=None):
  out = []
  for k, f in enumerate(frames):
    if modes is not None:
      params.values["K9LaneFix"] = modes[k]
    out.append(lp_step(planner, f))
  return out


def same(a, b):
  return all(np.array_equal(x[0], y[0]) and x[1] == y[1] for x, y in zip(a, b, strict=True))


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("off_value", [0, -1, 2, 7])
def test_lane_planner_off_equals_parent(seed, off_value):
  frames = lp_frames(seed)
  base = MemParams(AdjustLaneOffset=0, LatMpcInputOffset=6)
  new = MemParams(AdjustLaneOffset=0, LatMpcInputOffset=6, K9LaneFix=off_value)
  ref = run_lp(make_lp(LP_PARENT, base), frames)
  got = run_lp(make_lp(LP_NEW, new), frames)
  assert same(ref, got)
  assert sum(a for _, a in ref) > 50  # the sequence really used lane mode


@pytest.mark.parametrize("seed", SEEDS)
def test_lane_planner_on_equals_f1(seed):
  frames = lp_frames(seed)
  ref = run_lp(make_lp(LP_F1, MemParams(AdjustLaneOffset=0, LatMpcInputOffset=6)), frames)
  got = run_lp(make_lp(LP_NEW, MemParams(AdjustLaneOffset=0, LatMpcInputOffset=6, K9LaneFix=1)), frames)
  assert same(ref, got)
  off = run_lp(make_lp(LP_PARENT, MemParams(AdjustLaneOffset=0, LatMpcInputOffset=6)), frames)
  assert not same(off, got)  # F1 really changes the lane-mode reference


@pytest.mark.parametrize("seed", SEEDS[:3])
def test_lane_planner_switch_resets_and_returns_to_parent(seed):
  frames = lp_frames(seed)
  modes = [1] * 200 + [0] * 150 + [1] * 250
  params = MemParams(AdjustLaneOffset=0, LatMpcInputOffset=6, K9LaneFix=1)
  planner = make_lp(LP_NEW, params)
  got = []; states = []
  for k, f in enumerate(frames):
    params.values["K9LaneFix"] = modes[k]
    got.append(lp_step(planner, f))
    states.append((planner.lane_ramp_phase, planner.lane_ramp_g, planner.k9_lane_fix_mode))
  ref_off = run_lp(make_lp(LP_PARENT, MemParams(AdjustLaneOffset=0, LatMpcInputOffset=6)), frames)
  # OFF stretch is bit-identical to the parent fed the same whole sequence (F1 keeps no state that leaks into OFF)
  assert same(ref_off[200:350], got[200:350])
  # every change of the number restarted the K9LaneFix state: idle ramp state while OFF
  assert all(s[0] == 0 and s[1] == 0.0 and s[2] == 0 for s in states[200:350])
  # back ON: identical to a fresh F1 planner whose base filters saw the same history -> compare against F1 fed the
  # same whole sequence with its K9 state reset at frame 350 (base filters are shared behaviour)
  f1 = make_lp(LP_F1, MemParams(AdjustLaneOffset=0, LatMpcInputOffset=6))
  ref_on = []
  for k, f in enumerate(frames):
    if k == 350:
      f1.lane_ramp_g, f1.lane_ramp_from, f1.lane_ramp_k, f1.lane_ramp_phase, f1.lane_ramp_delta = 0.0, 0.0, 0, 0, None
    ref_on.append(lp_step(f1, f))
  assert same(ref_on[350:], got[350:])


def test_lane_planner_unreadable_param_is_off():
  class Broken(MemParams):
    def get_int(self, k, *a, **kw):
      if k == "K9LaneFix":
        raise KeyError(k)
      return super().get_int(k)
  frames = lp_frames(9, 200)
  ref = run_lp(make_lp(LP_PARENT, MemParams(AdjustLaneOffset=0, LatMpcInputOffset=6)), frames)
  got = run_lp(make_lp(LP_NEW, Broken(AdjustLaneOffset=0, LatMpcInputOffset=6)), frames)
  assert same(ref, got)


# ---------------------------------------------------------------- controlsd lateral command
class Cap(Exception):
  pass


def make_controls(path, params):
  tree = ast.parse(path.read_text(encoding="utf8"))
  tree.body = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))]
  ns = dict(math=math, np=np, car=car, log=log, Number=Number, CV=CV, LaneChangeState=log.LaneChangeState,
            LaneChangeDirection=log.LaneChangeDirection, DT_CTRL=DT_CTRL, DT_MDL=DT_MDL, CONTROL_N=CONTROL_N,
            MIN_LATERAL_CONTROL_SPEED=0.3, clip_curvature=DH.clip_curvature, get_lag_adjusted_curvature=get_lag_adjusted_curvature,
            resolve_vehicle_model_steer_ratio=lambda *a: 15.0, lateral_vehicle_parameters=lambda sm, cp: sm["liveParameters"])
  exec(compile(tree, str(path), "exec"), ns)
  c = ns["Controls"].__new__(ns["Controls"])
  c.params = params
  c.CP = NS(brand="hyundai", steerAtStandstill=False, minSteerSpeed=0.0, openpilotLongitudinalControl=True,
            lateralTuning=NS(which=lambda: "angle"), steerActuatorDelay=0.1)
  c.CI = NS(get_pid_accel_limits=lambda *a: (-1.0, 1.0)); c.is_vw_meb = False
  c.lateral_startup = NS(update=lambda *a: True); c.lateral_started = True
  c.laneless_blend_weight = 0.0; c.steer_limited_by_safety = False; c.desired_curvature = 0.0
  c.cur = {"k": 0.0, "lat": False}
  c.VM = NS(update_params=lambda *a: None, calc_curvature=lambda *a: -c.cur["k"], roll_compensation=lambda *a: 0.0)
  c.LoC = NS(reset=lambda: None, update=lambda *a: (0.0, 0.0, 0.0), long_control_state="off")
  c.carrot_controls = NS(lat_suspend_control=lambda _cs, la: c.cur["lat"])

  def lac_update(*a, **kw):
    raise Cap
  c.LaC = NS(update=lac_update, reset=lambda: None)
  return c


class SM(dict):
  pass


def cd_frames(seed, n=4000):
  rng = np.random.default_rng(100 + seed)
  v = 14.0; dc = 0.0; kl = 0.0; use = False; lat = True; frames = []
  for k in range(n):
    v = float(np.clip(v + rng.normal(0, 0.05), 1.0, 35.0))
    dc = float(np.clip(dc + rng.normal(0, 2e-4), -0.03, 0.03))
    kl = float(np.clip(kl + rng.normal(0, 2e-4), -0.03, 0.03))
    if rng.random() < 0.01:
      use = not use
    if rng.random() < 0.002:
      lat = not lat
    jump = rng.normal(0, 4e-3) if rng.random() < 0.01 else 0.0
    curvs = [kl + jump + 1e-4 * i for i in range(N)]
    frames.append(dict(v=v, dc=dc + jump * 0.5, use=use, lat=lat, curvs=curvs, psis=[0.0] * N, dist=list(np.linspace(0, 80, N)),
                       lcs=log.LaneChangeState.laneChangeStarting if rng.random() < 0.01 else log.LaneChangeState.off,
                       meas=dc + rng.normal(0, 1e-4), vturn=float(rng.uniform(0, 250)), roll=float(rng.normal(0, 0.02))))
  return frames


def cd_step(c, f, k):
  c.cur["k"] = f["meas"]; c.cur["lat"] = f["lat"]
  CS = car.CarState.new_message(vEgo=f["v"], gearShifter="drive", latEnabled=True)
  plan = NS(laneBlendWeight=0.0, useLaneLines=f["use"], mpcSolutionValid=True, laneChangeState=f["lcs"], modelMonoTime=k,
            psis=f["psis"], curvatures=f["curvs"], distances=f["dist"])
  sm = SM(carState=CS, modelV2=NS(meta=NS(laneChangeState=f["lcs"], laneChangeDirection=0), action=NS(desiredCurvature=f["dc"])),
          lateralPlan=plan, liveParameters=NS(stiffnessFactor=1.0, steerRatio=15.0, angleOffsetDeg=0.0, roll=f["roll"]),
          longitudinalPlan=NS(), selfdriveState=NS(enabled=True, active=True), onroadEvents=[], radarState=NS(),
          carrotMan=NS(vTurnSpeed=f["vturn"]), liveDelay=NS(lateralDelay=0.2))
  sm.frame = k; sm.recv_frame = {"lateralPlan": k, "longitudinalPlan": k}; sm.logMonoTime = {"modelV2": k}
  c.sm = sm
  try:
    c.state_control()
  except Cap:
    pass
  return float(c.desired_curvature)


def run_cd(c, frames, params=None, modes=None, sync_from=None):
  out = []
  for k, f in enumerate(frames):
    if modes is not None:
      params.values["K9LaneFix"] = modes[k]
    if sync_from is not None:
      sync_from(k, c)
    out.append(cd_step(c, f, k))
  return np.array(out)


CD_SETTINGS = dict(LatSmoothSec=13, SteerActuatorDelay=23, UseLaneLineCurveSpeed=0, AlwaysLateral=True)


@pytest.mark.parametrize("seed", SEEDS)
@pytest.mark.parametrize("off_value", [0, -3, 5])
def test_controls_off_equals_parent(seed, off_value):
  frames = cd_frames(seed)
  ref = run_cd(make_controls(REF / "controlsd.parent.py.ref", MemParams(**CD_SETTINGS)), frames)
  got = run_cd(make_controls(CONTROLS / "controlsd.py", MemParams(**CD_SETTINGS, K9LaneFix=off_value)), frames)
  assert np.array_equal(ref, got)


@pytest.mark.parametrize("seed", SEEDS)
def test_controls_on_equals_f1(seed):
  frames = cd_frames(seed)
  ref = run_cd(make_controls(REF / "controlsd.F1.py.ref", MemParams(**CD_SETTINGS)), frames)
  got = run_cd(make_controls(CONTROLS / "controlsd.py", MemParams(**CD_SETTINGS, K9LaneFix=1)), frames)
  assert np.array_equal(ref, got)
  off = run_cd(make_controls(REF / "controlsd.parent.py.ref", MemParams(**CD_SETTINGS)), frames)
  assert not np.array_equal(off, got)  # the hand-over cap really acts on this sequence


@pytest.mark.parametrize("seed", SEEDS[:3])
def test_controls_switch_resets(seed):
  frames = cd_frames(seed)
  params = MemParams(**CD_SETTINGS, K9LaneFix=1)
  c = make_controls(CONTROLS / "controlsd.py", params)
  modes = [1] * 1500 + [0] * 1000 + [1] * 1500
  parent = make_controls(REF / "controlsd.parent.py.ref", MemParams(**CD_SETTINGS))
  got = []; ref = []; ks = []
  for k, f in enumerate(frames):
    params.values["K9LaneFix"] = modes[k]
    prev = c.desired_curvature  # parent steps from the same command level each tick
    got.append(cd_step(c, f, k))
    if k >= 1500:
      parent.desired_curvature = prev
      ref.append(cd_step(parent, f, k))
    ks.append((getattr(c, "k9_handover_k", None), getattr(c, "k9_lane_fix_mode", None)))
  # at the switch to 0 the hand-over state is cleared and from then on the command equals the parent step for step
  assert ks[1500] == (-1, 0)
  assert np.array_equal(np.array(got[1500:2500]), np.array(ref[:1000]))
  # at the switch back to 1 the state starts fresh: no cap on the switch tick itself
  assert ks[2500][1] == 1 and ks[2500][0] in (-1, 0)
  assert got[2500] == ref[1000]


# ---------------------------------------------------------------- profile table / param / menu
def test_profile_table_and_mode_reader():
  assert set(K9.K9_LANE_FIX_PROFILES) == {1}
  f1 = K9.K9_LANE_FIX_PROFILES[1]
  assert f1 == {"name": "F1", "converge_mode": "smootherstep", "converge_t": 2.0, "ramp_in_t": 0.5, "t_eref": 0.3, "t_max": 4.0,
                "t_kref": 0.003, "entry_jerk": 2.5, "entry_t": 2.0}
  # the same constants as the frozen F1 files
  assert (LP_F1.LANE_CONVERGE_MODE, LP_F1.LANE_CONVERGE_T, LP_F1.LANE_RAMP_IN_T, LP_F1.LANE_CONVERGE_T_EREF,
          LP_F1.LANE_CONVERGE_T_MAX, LP_F1.LANE_CONVERGE_T_KREF) == ("smootherstep", 2.0, 0.5, 0.3, 4.0, 0.003)
  cd_src = (REF / "controlsd.F1.py.ref").read_text(encoding="utf8")
  assert "LANELESS_ENTRY_JERK = 2.5" in cd_src and "LANELESS_ENTRY_T = 2.0" in cd_src and "LANELESS_LANE_ENTRY_JERK = 0.0" in cd_src
  for value, expect in ((0, 0), (1, 1), (2, 0), (-1, 0), (99, 0)):
    assert K9.k9_lane_fix_mode(MemParams(K9LaneFix=value)) == expect
  assert K9.k9_lane_fix_profile(0) is None and K9.k9_lane_fix_profile(2) is None


def test_param_registered_default_off():
  keys = (ROOT / "openpilot" / "common" / "params_keys.h").read_text(encoding="utf8")
  assert '{"K9LaneFix", {PERSISTENT, INT, "0"}},' in keys


def test_menu_schema():
  settings = json.loads((ROOT / "openpilot" / "selfdrive" / "carrot_settings.json").read_text(encoding="utf8"))
  text = json.dumps(settings["menu"])
  assert "K9LaneFix" in text

  def groups(node):
    if isinstance(node, dict):
      if node.get("id") == "EXPERIMENTAL":
        yield node
      for v in node.values():
        yield from groups(v)
    elif isinstance(node, list):
      for v in node:
        yield from groups(v)
  exp = list(groups(settings["menu"]))
  assert len(exp) == 1 and exp[0]["params"] == ["K9StopMode", "K9LaneFix", "K9LRView", "K9ExtHud", "K9MeterSeason"]
  params = settings["params"]
  p = next(x for x in params if x["name"] == "K9LaneFix")
  assert (p["min"], p["max"], p["default"], p["unit"], p["control"]) == (0, 1, 0, 1, "select")
  for locale in ("ko", "en", "zh"):
    assert [o.split(" ", 1)[0] for o in p["options"][locale]] == ["0", "1"]
  assert "실차 미검증" in p["descr"] and "not vehicle-validated" in p["edescr"] and "未经实车验证" in p["cdescr"]
  for d in (p["descr"], p["edescr"]):
    assert "2.5" in d and re.search(r"2\s?(초|s)", d) and re.search(r"4\s?(초|s)", d) and "0.5" in d
  docs_ko = (ROOT / "docs" / "user" / "ko" / "settings.md").read_text(encoding="utf8")
  docs_en = (ROOT / "docs" / "user" / "en" / "settings.md").read_text(encoding="utf8")
  assert "`K9LaneFix`" in docs_ko and "실차 미검증" in docs_ko.split("`K9LaneFix`", 1)[1].split("\n", 1)[0]
  assert "`K9LaneFix`" in docs_en and "not vehicle-validated" in docs_en.split("`K9LaneFix`", 1)[1].split("\n", 1)[0]
