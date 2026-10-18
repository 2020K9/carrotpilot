"""LongMpcJerkCost removal: acc MPC jerk weight is again the fixed jerk_factor * clip(response, 0, 1) * J_EGO_COST(20.0).

A recording solver captures the weights sent to acados; this is wiring, not a solver or vehicle test.
Leftover stored values on a device must not be read or change any solver write.
"""
import importlib.util
import inspect
import json
import subprocess
import sys
import types
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.params import Params
import openpilot.selfdrive.carrot.carrot_functions as carrot_functions
from openpilot.selfdrive.carrot.carrot_functions import CarrotPlanner
from openpilot.selfdrive.carrot.driving_mode import DrivingMode, DrivingModeDetector


class RecordingSolver:
  def __init__(self, *args): self.writes = []
  def reset(self): pass
  def set(self, *args): pass
  def cost_set(self, i, key, value): self.writes.append(("cost", i, key, np.array(value, dtype=float)))
  def constraints_set(self, i, key, value): self.writes.append(("constr", i, key, np.array(value, dtype=float)))


def _import_long_mpc():
  # Every MPC here uses RecordingSolver. Without a built acados solver, stub it only for this import and
  # drop both modules from sys.modules again so other test files still see the real import result.
  solver = "openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx"
  name = "openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc"
  try:
    if importlib.util.find_spec(solver) is not None:
      return importlib.import_module(name)
  except ModuleNotFoundError:
    pass
  assert solver not in sys.modules and name not in sys.modules
  sys.modules[solver] = types.SimpleNamespace(AcadosOcpSolverCython=RecordingSolver)
  try:
    return importlib.import_module(name)
  finally:
    sys.modules.pop(solver, None)
    sys.modules.pop(name, None)


long_mpc = _import_long_mpc()
ROOT = Path(__file__).resolve().parents[4]
NAME = "LongMpcJerkCost"
# Last revision before LongMpcJerkCost was added (e8ded9e1).
BASE_COMMIT = "e8ded9e171b03c62391239479f5ca378cb4d1ec0^"
LONG_MPC = "openpilot/selfdrive/controls/lib/longitudinal_mpc_lib/long_mpc.py"
Personality = log.LongitudinalPersonality
JERK_FACTORS = [(Personality.moreRelaxed, 1.0), (Personality.relaxed, 1.0), (Personality.standard, 0.7), (Personality.aggressive, 0.5)]
RESPONSE_FACTORS = [0.0, 0.37, 1.0, 1.5, -0.2]
LEFTOVERS = [None, "20", "5", "60", "40", "20.5", "abc"]


@pytest.fixture
def params(tmp_path):
  p = Params(str(tmp_path))
  p.put("MyDrivingMode", 3)
  p.put("StopDistanceCarrot", 600)
  return p


def store_leftover(params, raw):
  # An old device may still hold the removed key's file; write it directly next to the other params.
  path = Path(params.get_param_path("MyDrivingMode")).with_name(NAME)
  if raw is None:
    path.unlink(missing_ok=True)
  else:
    path.write_bytes(raw.encode())


def make_planner(params):
  planner = CarrotPlanner.__new__(CarrotPlanner)
  planner.params = params
  planner.frame = 0
  planner.params_count = 39
  planner.myDrivingMode_last = planner.myDrivingMode = DrivingMode.Normal
  planner.myDrivingMode_disable_auto = False
  planner.drivingModeDetector = DrivingModeDetector()
  planner.tFollowGap1 = planner.tFollowGap2 = planner.tFollowGap3 = planner.tFollowGap4 = 1.45
  planner.jerk_factor = 1.0
  planner.get_T_FOLLOW = lambda personality=Personality.standard, v_ego=0., a_ego=0.: planner._get_base_t_follow(personality, v_ego)
  planner.leadAccelResponse = 2
  planner.comfort_brake = 2.4
  planner.v_cruise = 30.
  planner.stop_dist = 1000.
  planner.lane_change_active = False
  return planner


def read_settings(planner):
  planner._params_update()  # params_count 40 group (AChangeCostStarting etc.)
  assert planner.params_count == 40
  return planner


def run_mpc(monkeypatch, planner, mode, personality, response_factor, prev=True):
  monkeypatch.setattr(long_mpc, "AcadosOcpSolverCython", RecordingSolver)
  mpc = long_mpc.LongitudinalMpc(mode=mode)
  mpc.run = lambda: None
  mpc.lead_response_state = NS(update=lambda req, dt, tid: NS(active=True, level=2, a_change_cost_factor=0.4,
                                                               jerk_cost_factor=response_factor))
  planner.mode = mode
  lead = NS(status=True, radar=True, radarTrackId=49, dRel=40., vLead=18., vRel=-2., aLeadK=-.3, aLeadTau=1.5, modelProb=1.)
  absent = NS(status=False, radar=False, radarTrackId=-1, dRel=0., vLead=0., vRel=0., aLeadK=0., aLeadTau=1.5, modelProb=0.)
  mpc.set_cur_state(20., 0.)
  mpc.set_accel_limits(-1.2, 2.)
  mpc.solver.writes.clear()
  mpc.update(planner, False, NS(leadOne=lead, leadTwo=absent), 30., *(np.zeros(13) for _ in range(4)),
             personality=personality, prev_accel_constraint=prev, a_change_cost_starting=planner.aChangeCostStarting,
             lead_accel_response_enabled=True, lead_gap_enabled=True, lead_track_frames=(10, 10))
  return mpc


def assert_same_writes(a, b):
  assert len(a.solver.writes) == len(b.solver.writes)
  for x, y in zip(a.solver.writes, b.solver.writes, strict=True):
    assert x[:-1] == y[:-1]
    np.testing.assert_array_equal(x[-1], y[-1])


def test_fixed_base_cost_and_no_configurable_argument():
  assert long_mpc.J_EGO_COST == 20.0
  assert "jerk_cost" not in inspect.signature(long_mpc.LongitudinalMpc.set_weights).parameters
  mpc = NS(mode="acc", set_cost_weights=lambda cost, constraint: setattr(mpc, "cost", cost))
  long_mpc.LongitudinalMpc.set_weights(mpc, jerk_factor=0.7, jerk_cost_factor=0.37)
  assert mpc.cost[5] == 0.7 * 0.37 * 20.0
  long_mpc.LongitudinalMpc.set_weights(mpc)
  assert mpc.cost[5] == 20.0


def test_long_mpc_matches_revision_before_setting():
  try:
    base = subprocess.run(["git", "show", f"{BASE_COMMIT}:{LONG_MPC}"], cwd=ROOT, check=True,
                          capture_output=True, text=True, encoding="utf-8").stdout
  except (OSError, subprocess.CalledProcessError) as e:
    pytest.skip(f"git history unavailable: {e}")
  assert (ROOT / LONG_MPC).read_text(encoding="utf-8") == base


@pytest.mark.parametrize("personality,jerk_factor", JERK_FACTORS)
@pytest.mark.parametrize("response_factor", RESPONSE_FACTORS)
@pytest.mark.parametrize("prev", [True, False])
def test_acc_jerk_weight_is_fixed_formula_even_with_leftover(monkeypatch, params, personality, jerk_factor, response_factor, prev):
  expected = jerk_factor * float(np.clip(response_factor, 0.0, 1.0)) * 20.0
  store_leftover(params, None)
  base = run_mpc(monkeypatch, read_settings(make_planner(params)), "acc", personality, response_factor, prev)
  for raw in LEFTOVERS:
    store_leftover(params, raw)
    planner = read_settings(make_planner(params))
    assert not hasattr(planner, "longMpcJerkCost")
    mpc = run_mpc(monkeypatch, planner, "acc", personality, response_factor, prev)
    assert planner.jerk_factor == jerk_factor
    jerk_weights = [w[5, 5] for kind, _, key, w in mpc.solver.writes if kind == "cost" and key == "W" and w.shape[0] > 5]
    assert len(jerk_weights) == long_mpc.N and all(w == expected for w in jerk_weights)
    assert_same_writes(base, mpc)


@pytest.mark.parametrize("personality,jerk_factor", JERK_FACTORS)
def test_blended_costs_unchanged_with_leftover(monkeypatch, params, personality, jerk_factor):
  store_leftover(params, None)
  base = run_mpc(monkeypatch, read_settings(make_planner(params)), "blended", personality, 0.37)
  for raw in LEFTOVERS:
    store_leftover(params, raw)
    assert_same_writes(base, run_mpc(monkeypatch, read_settings(make_planner(params)), "blended", personality, 0.37))


def test_setting_code_catalog_and_docs_are_gone():
  assert not hasattr(carrot_functions, "parse_long_mpc_jerk_cost")
  assert not hasattr(CarrotPlanner, "_read_param_text")
  for rel in ("openpilot/selfdrive/carrot/carrot_functions.py", LONG_MPC, "openpilot/common/params_keys.h",
              "docs/user/ko/settings.md", "docs/user/en/settings.md"):
    text = (ROOT / rel).read_text(encoding="utf-8")
    assert NAME not in text and "longMpcJerkCost" not in text and "jerk_cost=" not in text, rel
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  assert NAME not in json.dumps(catalog, ensure_ascii=False)
  assert all(p.get("name") != NAME for p in catalog["params"])
  assert not (ROOT / "openpilot/selfdrive/controls/tests/test_long_mpc_jerk_cost.py").exists()
