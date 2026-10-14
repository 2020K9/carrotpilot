"""LongMpcJerkCost: stored text -> CarrotPlanner -> LongitudinalMpc.update -> solver cost weights.

A recording solver captures the weights sent to acados; this is wiring, not a solver or vehicle test.
"""
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.cereal import log
from openpilot.common.params import Params
from openpilot.selfdrive.carrot.carrot_functions import CarrotPlanner, parse_long_mpc_jerk_cost
from openpilot.selfdrive.carrot.driving_mode import DrivingMode, DrivingModeDetector
import openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc as long_mpc

Personality = log.LongitudinalPersonality
JERK_FACTORS = [(Personality.moreRelaxed, 1.0), (Personality.relaxed, 1.0), (Personality.standard, 0.7), (Personality.aggressive, 0.5)]
RESPONSE_FACTORS = [0.0, 0.37, 1.0, 1.5, -0.2]


class RecordingSolver:
  def __init__(self, *args): self.writes = []
  def reset(self): pass
  def set(self, *args): pass
  def cost_set(self, i, key, value): self.writes.append(("cost", i, key, np.array(value, dtype=float)))
  def constraints_set(self, i, key, value): self.writes.append(("constr", i, key, np.array(value, dtype=float)))


@pytest.fixture
def params(tmp_path):
  p = Params(str(tmp_path))
  p.put("MyDrivingMode", 3)
  p.put("StopDistanceCarrot", 600)
  return p


def store(params, raw):
  path = params.get_param_path("LongMpcJerkCost")
  if raw is None:
    params.remove("LongMpcJerkCost")
  else:
    with open(path, "wb") as f:
      f.write(raw if isinstance(raw, bytes) else raw.encode())


def make_planner(params):
  planner = CarrotPlanner.__new__(CarrotPlanner)
  planner.params = params
  planner.frame = 0
  planner.params_count = 39
  planner.myDrivingMode_last = planner.myDrivingMode = DrivingMode.Normal
  planner.myDrivingMode_disable_auto = False
  planner.drivingModeDetector = DrivingModeDetector()
  planner.longMpcJerkCost = 20.0
  planner.tFollowGap1 = planner.tFollowGap2 = planner.tFollowGap3 = planner.tFollowGap4 = 1.45
  planner.jerk_factor = 1.0
  # Keep the real gap-step -> jerk_factor mapping, with a fixed following time.
  planner.get_T_FOLLOW = lambda personality=Personality.standard, v_ego=0., a_ego=0.: planner._get_base_t_follow(personality, v_ego)
  planner.leadAccelResponse = 2
  planner.comfort_brake = 2.4
  planner.v_cruise = 30.
  planner.stop_dist = 1000.
  planner.lane_change_active = False
  return planner


def read_settings(planner):
  planner._params_update()  # params_count 40 group: AChangeCostStarting and LongMpcJerkCost
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


def weights(mpc):
  return [w for kind, _, key, w in mpc.solver.writes if kind == "cost" and key == "W"]


def everything_but_jerk(mpc):
  out = []
  for kind, i, key, value in mpc.solver.writes:
    value = value.copy()
    if key == "W" and value.shape[0] > 5:
      value[5, 5] = np.nan
    out.append((kind, i, key, value))
  return out + [("params", mpc.params.copy()), ("yref", mpc.yref.copy()), ("a_change_cost", mpc.a_change_cost)]


def assert_same(a, b):
  assert len(a) == len(b)
  for x, y in zip(a, b, strict=True):
    assert x[:-1] == y[:-1]
    np.testing.assert_array_equal(x[-1], y[-1])


@pytest.mark.parametrize("raw,expected", [
  ("5", 5.0), ("60", 60.0), ("20", 20.0), ("10", 10.0), ("40", 40.0),
  (None, 20.0), ("", 20.0), ("4", 20.0), ("61", 20.0), ("0", 20.0), ("-10", 20.0),
  ("20.5", 20.0), ("10.0", 20.0), ("59.9", 20.0), ("nan", 20.0), ("NaN", 20.0), ("inf", 20.0), ("-inf", 20.0),
  ("abc", 20.0), ("1e1", 20.0), ("0x10", 20.0), ("1_0", 20.0), ("４０", 20.0), (b"\xff\xfe", 20.0),
])
def test_stored_text_reaches_carrot_attribute(params, raw, expected):
  store(params, raw)
  planner = read_settings(make_planner(params))
  assert planner.longMpcJerkCost == expected
  assert type(planner.longMpcJerkCost) is float
  assert parse_long_mpc_jerk_cost(None) == parse_long_mpc_jerk_cost(1.5) == 20.0


def test_registered_default_is_20(params):
  assert params.get("LongMpcJerkCost", return_default=True) == 20
  params.put("LongMpcJerkCost", 40)
  assert read_settings(make_planner(params)).longMpcJerkCost == 40.0


@pytest.mark.parametrize("personality,jerk_factor", JERK_FACTORS)
@pytest.mark.parametrize("response_factor", RESPONSE_FACTORS)
@pytest.mark.parametrize("prev", [True, False])
def test_default_20_matches_previous_weights_exactly(monkeypatch, params, personality, jerk_factor, response_factor, prev):
  for raw in (None, "20"):
    store(params, raw)
    planner = read_settings(make_planner(params))
    mpc = run_mpc(monkeypatch, planner, "acc", personality, response_factor, prev)
    assert planner.jerk_factor == jerk_factor
    # Previous formula: jerk_factor * clip(response, 0, 1) * J_EGO_COST(20.0)
    previous = jerk_factor * float(np.clip(response_factor, 0.0, 1.0)) * 20.0
    for w in weights(mpc):
      if w.shape[0] > 5:
        assert w[5, 5] == previous
    assert mpc.jerk_cost_factor == float(np.clip(response_factor, 0.0, 1.0))


@pytest.mark.parametrize("personality,jerk_factor", JERK_FACTORS)
@pytest.mark.parametrize("response_factor", RESPONSE_FACTORS)
@pytest.mark.parametrize("value", ["10", "40", "5", "60"])
def test_setting_scales_only_acc_jerk_weight(monkeypatch, params, personality, jerk_factor, response_factor, value):
  store(params, "20")
  base = run_mpc(monkeypatch, read_settings(make_planner(params)), "acc", personality, response_factor)
  store(params, value)
  planner = read_settings(make_planner(params))
  changed = run_mpc(monkeypatch, planner, "acc", personality, response_factor)
  assert planner.longMpcJerkCost == float(value)

  clipped = float(np.clip(response_factor, 0.0, 1.0))
  base_w, changed_w = weights(base), weights(changed)
  assert len(base_w) == len(changed_w) == long_mpc.N + 1
  for b, c in zip(base_w, changed_w, strict=True):
    if b.shape[0] > 5:
      assert c[5, 5] == jerk_factor * clipped * float(value)
      assert c[5, 5] == pytest.approx(b[5, 5] * float(value) / 20.0, rel=1e-12, abs=0.0)
  assert_same(everything_but_jerk(base), everything_but_jerk(changed))


@pytest.mark.parametrize("personality,jerk_factor", JERK_FACTORS)
@pytest.mark.parametrize("value", ["10", "40", "5", "60", "abc"])
def test_blended_costs_ignore_setting(monkeypatch, params, personality, jerk_factor, value):
  store(params, "20")
  base = run_mpc(monkeypatch, read_settings(make_planner(params)), "blended", personality, 0.37)
  store(params, value)
  changed = run_mpc(monkeypatch, read_settings(make_planner(params)), "blended", personality, 0.37)
  for w in weights(changed):
    if w.shape[0] > 5:
      assert w[5, 5] == 1.0
  assert len(base.solver.writes) == len(changed.solver.writes)
  for x, y in zip(base.solver.writes, changed.solver.writes, strict=True):
    assert x[:-1] == y[:-1]
    np.testing.assert_array_equal(x[-1], y[-1])


def test_set_weights_default_argument_keeps_existing_callers():
  mpc = NS(mode="acc", set_cost_weights=lambda cost, constraint: setattr(mpc, "cost", cost))
  long_mpc.LongitudinalMpc.set_weights(mpc, jerk_factor=0.7, jerk_cost_factor=0.37)
  assert mpc.cost[5] == 0.7 * 0.37 * 20.0
  long_mpc.LongitudinalMpc.set_weights(mpc)
  assert mpc.cost[5] == 20.0
