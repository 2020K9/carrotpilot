"""lane_avoid gate/return/audit tests (prompt v3 §5). Written for review; NOT executed.

TEST_CFG values are synthetic fixtures for exercising the gate logic only. They are
not approved settings and must not be copied into LaneAvoidConfig defaults. The
return-risk/centre-invalid policy tuples are monkeypatched with a test sentinel;
production keeps them empty, so the production controller stays disabled.
"""
import ast
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

import openpilot.selfdrive.controls.lib.lane_avoid as la
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.lane_avoid import (
  AVOID, CLEAR, DISABLED, LEFT, OCCUPIED, RETURN, RETURN_RISK, RIGHT, STANDBY, UNKNOWN,
  AvoidInputs, Coverage, LaneAvoidConfig, LaneAvoidController, SourceReading,
  avoid_candidate, evaluate_side, evaluate_source, radar_side_reading, region_covered, road_edge_allowance)
from openpilot.selfdrive.controls.lib.lane_avoid_audit import audit_frames
from openpilot.selfdrive.controls.tests.test_lane_model_speed_planner import inputs as planner_inputs
from openpilot.selfdrive.controls.tests.test_lane_model_speed_planner import make_planner

DT = 0.05
N = 33
X = np.linspace(0.0, 100.0, N)
TEST_POLICY = 'test_only_not_approved'

# v3.1 fixture update: the required space spans the whole consumed path (body rear ..
# last point + body front = -1 .. 104 m here), so the synthetic coverage and road-edge
# samples must span it too. Previously (-15, 40) and X; see REVIEW.md "fixture conflicts".
FULL_COV = Coverage(-15.0, 115.0, 0.0, 5.0)
# v3.2 fixture update: road edges must be observed from the body rear (-1 m) too
# (previously np.linspace(0.0, 110.0, N); see REVIEW.md v3.2 fixture changes).
EDGE_X = np.linspace(-10.0, 110.0, N)
TEST_CFG = LaneAvoidConfig(
  enabled=True, max_offset_m=0.5, entry_rate_mps=0.3, return_rate_mps=0.2, max_rate_change_mps2=2.0,
  candidate_x_m=(5.0, 60.0), candidate_deadband_m=0.05, max_abs_curvature=0.01, vehicle_half_width_m=0.95,
  road_edge_margin_m=0.5, road_edge_std_max_m=1.0, required_region_x_m=(-10.0, 30.0),
  required_region_lat_m=(0.0, 4.0),
  coverage={(s, side): FULL_COV for s in la.REQUIRED_SOURCES for side in (LEFT, RIGHT)},
  max_age_s={'bsd': 0.2, 'radar': 0.2, 'model': 0.2, 'road_edge': 0.2, 'model_path': 0.2},
  max_input_gap_s=0.2, entry_confirm_frames=2, reentry_wait_s=1.0,
  return_risk_policy=TEST_POLICY, center_invalid_policy=TEST_POLICY,
  vehicle_front_m=4.0, vehicle_rear_m=1.0, side_clearance_m=0.3,
  occupancy_prediction_policy=TEST_POLICY, body_geometry_model=TEST_POLICY, constraint_conflict_policy=TEST_POLICY,
  prediction_horizon_s=3.0)

POLICY_TUPLES = ('APPROVED_RETURN_RISK_POLICIES', 'APPROVED_CENTER_INVALID_POLICIES',
                 'APPROVED_OCCUPANCY_PREDICTION_POLICIES', 'APPROVED_BODY_GEOMETRY_MODELS',
                 'APPROVED_CONSTRAINT_CONFLICT_POLICIES')


@pytest.fixture
def approved_policies(monkeypatch):
  for name in POLICY_TUPLES:
    monkeypatch.setattr(la, name, (TEST_POLICY,))
  # v3.2: approval alone no longer activates; register test-only implementations
  for kind, impl in TEST_IMPLEMENTATIONS.items():
    monkeypatch.setitem(la.POLICY_IMPLEMENTATIONS[kind], TEST_POLICY, impl)


def cv_predictor(objects, side, space, horizon_s, t, step=0.1):
  """Test-only constant-velocity predictor (synthetic, not an approved policy): any object
  rectangle intersecting the space at any sampled time in [t, t + horizon] -> OCCUPIED."""
  for o in objects:
    for k in range(int(round(horizon_s / step)) + 1):
      tau = (t - o.t) + k * step
      x, lat = o.x + o.vx * tau, o.lat + o.vlat * tau
      if (x + o.length / 2 >= space.x_min and x - o.length / 2 <= space.x_max and
          lat + o.width / 2 >= space.lat_min and lat - o.width / 2 <= space.lat_max):
        return OCCUPIED
  return CLEAR


# None = no proposal: the constraint-only step is used (test stand-in, not an approved policy)
TEST_IMPLEMENTATIONS = {
  'return_risk': lambda ctx: None,
  'center_invalid': lambda ctx: None,
  'occupancy_prediction': cv_predictor,
  'body_geometry': la.body_footprint,
  'constraint_conflict': lambda ctx: 'test_conflict_action',
}


def clear():
  return SourceReading(supported=True, valid=True, age_s=0.0, detected=False)


def occ():
  return SourceReading(supported=True, valid=True, age_s=0.0, detected=True)


def side(bsd=None, radar=None, model=None):
  return {'bsd': bsd or clear(), 'radar': radar or clear(), 'model': model or clear()}


def edges(left_y=-5.0, right_y=5.0, std=0.1, age=0.0):
  return {LEFT: (EDGE_X, np.full(N, left_y), std, age), RIGHT: (EDGE_X, np.full(N, right_y), std, age)}


def inp(t, shift, left=None, right=None, road=None, **kw):
  lane_y = np.zeros(N)
  # v3.1: a new model message stamped at t on the verified clock (age 0) each frame
  args = dict(t=t, center_valid=True, path_x=X, model_y=lane_y + shift, lane_y=lane_y, base_y=np.zeros(N),
              d_prob=1.0, model_valid=True, model_age_s=0.0, lane_change_active=False, driver_steering=False,
              measured_curvature=0.0, side_readings={LEFT: left or side(), RIGHT: right or side()},
              road_edges=road if road is not None else edges(), model_t=t, clock_verified=True,
              # v3.2: an observed, empty object list per side (missing -> UNKNOWN)
              side_objects={LEFT: [], RIGHT: []})
  args.update(kw)
  return AvoidInputs(**args)


def mirror(direction):
  """direction LEFT: obstacle right, model shifts left (negative y). RIGHT: mirror."""
  if direction == LEFT:
    return -0.3, side(), side(radar=occ())
  return 0.3, side(radar=occ()), side()


def frame(out, t):
  return {'t': t, 'state': out.state, 'permitted': out.permitted, 'target': out.target, 'applied': out.applied,
          'avoid_side': out.avoid_side, 'side_state': dict(out.side_state), 'edge_state': dict(out.edge_state),
          # v3.1: the controller's rate seeds the first interval; edge room bounds every applied value
          'rate': out.rate, 'edge_room': dict(out.edge_room),
          # v3.2: invalid outputs are audit failures
          'valid': out.valid, 'invalid_reasons': out.invalid_reasons}


def run(ctrl, seq, t0=0.0):
  outs, frames = [], []
  for i, kw in enumerate(seq):
    t = t0 + i * DT
    o = ctrl.update(inp(t, **kw))
    outs.append(o)
    frames.append(frame(o, t))
  return outs, frames


# ---------------------------------------------------------------- configuration / disabled


def test_default_config_is_disabled_with_every_blocker():
  cfg = LaneAvoidConfig()
  b = cfg.blockers()
  for name in ('feature_disabled', 'max_offset_m_unapproved', 'entry_rate_mps_unapproved', 'return_rate_mps_unapproved',
               'road_edge_margin_m_unapproved', 'required_region_x_m_unapproved', 'coverage_unapproved',
               'max_age_bsd_unapproved', 'max_age_road_edge_unapproved', 'return_risk_policy_unapproved',
               'center_invalid_policy_unapproved', 'entry_confirm_frames_unapproved'):
    assert name in b
  ctrl = LaneAvoidController()
  assert not ctrl.active and ctrl.state == DISABLED
  shift, left, right = mirror(LEFT)
  for i in range(40):
    o = ctrl.update(inp(i * DT, shift, left, right))
    assert o.applied == 0.0 and o.target == 0.0 and not o.permitted and o.state == DISABLED


def test_numeric_values_without_approved_return_policy_stay_disabled():
  # production tuples are empty: even a fully numeric config cannot activate
  assert la.APPROVED_RETURN_RISK_POLICIES == ()
  assert la.APPROVED_CENTER_INVALID_POLICIES == ()
  b = TEST_CFG.blockers()
  assert 'return_risk_policy_unapproved' in b and 'center_invalid_policy_unapproved' in b
  assert not LaneAvoidController(TEST_CFG).active


@pytest.mark.parametrize('field,value', [
  ('max_offset_m', float('nan')), ('max_offset_m', float('inf')), ('max_offset_m', -0.1), ('max_offset_m', 0.0),
  ('max_offset_m', True), ('entry_rate_mps', None), ('candidate_x_m', (60.0, 5.0)), ('candidate_x_m', (5.0,)),
  ('required_region_lat_m', (0.0, float('nan'))), ('entry_confirm_frames', 0), ('entry_confirm_frames', 1.5),
  ('coverage', {}), ('max_age_s', {'bsd': 0.2}), ('enabled', False),
])
def test_invalid_or_missing_settings_block(approved_policies, field, value):
  cfg = LaneAvoidConfig(**{**TEST_CFG.__dict__, field: value})
  assert cfg.blockers()
  assert not LaneAvoidController(cfg).active


def test_fixture_config_activates_only_with_patched_policies(approved_policies):
  assert TEST_CFG.blockers() == []
  assert LaneAvoidController(TEST_CFG).active


# ---------------------------------------------------------------- start gate, symmetry


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_symmetric_avoidance_moves_away_from_obstacle(approved_policies, direction):
  shift, left, right = mirror(direction)
  ctrl = LaneAvoidController(TEST_CFG)
  outs, frames = run(ctrl, [dict(shift=shift, left=left, right=right)] * 40)
  assert not outs[0].permitted  # entry confirmation
  assert outs[-1].state == AVOID and outs[-1].permitted and outs[-1].avoid_side == direction
  sign = la.SIDE_SIGN[direction]
  assert all(o.applied * sign >= 0.0 for o in outs)
  assert outs[-1].applied * sign > 0.0
  assert abs(outs[-1].applied) <= 0.3 + 1e-12
  assert audit_frames(frames, TEST_CFG)['result'] == 'no_violation_found'


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_shift_toward_obstacle_side_is_not_avoidance(approved_policies, direction):
  shift, left, right = mirror(direction)
  ctrl = LaneAvoidController(TEST_CFG)
  outs, _ = run(ctrl, [dict(shift=-shift, left=left, right=right)] * 30)
  assert not any(o.permitted for o in outs)
  assert all(o.applied == 0.0 for o in outs)


def test_model_bias_without_obstacle_evidence_is_not_avoidance(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  outs, _ = run(ctrl, [dict(shift=-0.3)] * 30)
  assert not any(o.permitted for o in outs)
  assert 'no_obstacle_evidence_opposite' in outs[-1].reasons


AVOID_SIDE_CASES = {
  'bsd_only': dict(bsd=occ()),
  'radar_only': dict(radar=occ()),
  'model_only': dict(model=occ()),
  'bsd_and_radar': dict(bsd=occ(), radar=occ()),
  'all': dict(bsd=occ(), radar=occ(), model=occ()),
}


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
@pytest.mark.parametrize('case', sorted(AVOID_SIDE_CASES))
def test_any_occupied_source_on_avoid_side_blocks_start(approved_policies, direction, case):
  shift, left, right = mirror(direction)
  blocked = side(**AVOID_SIDE_CASES[case])
  if direction == LEFT:
    left = blocked
  else:
    right = blocked
  ctrl = LaneAvoidController(TEST_CFG)
  outs, frames = run(ctrl, [dict(shift=shift, left=left, right=right)] * 30)
  assert sum(o.permitted for o in outs) == 0
  assert all(o.applied == 0.0 for o in outs)
  assert audit_frames(frames, TEST_CFG)['violations'] == []


def test_bsd_only_on_obstacle_side_is_evidence_not_veto_of_avoid_side(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  outs, _ = run(ctrl, [dict(shift=-0.3, left=side(), right=side(bsd=occ()))] * 30)
  assert outs[-1].permitted and outs[-1].avoid_side == LEFT


def test_bsd_true_is_veto_whatever_its_metadata():
  for r in (SourceReading(supported=False, valid=False, age_s=None, detected=True),
            SourceReading(supported=None, valid=None, age_s=99.0, detected=True)):
    assert evaluate_source(r, 0.2) == OCCUPIED


# ---------------------------------------------------------------- unknown is not clear


UNKNOWN_READINGS = {
  'bool_false_unsupported': SourceReading(supported=None, valid=None, age_s=None, detected=False),
  'sensor_off': SourceReading(supported=False, valid=True, age_s=0.0, detected=False),
  'invalid_vision_false': SourceReading(supported=True, valid=False, age_s=0.0, detected=False),
  'age_unknown': SourceReading(supported=True, valid=True, age_s=None, detected=False),
  'stale': SourceReading(supported=True, valid=True, age_s=0.5, detected=False),
  'negative_age': SourceReading(supported=True, valid=True, age_s=-0.01, detected=False),
  'nan_age': SourceReading(supported=True, valid=True, age_s=float('nan'), detected=False),
  'detection_unknown': SourceReading(supported=True, valid=True, age_s=0.0, detected=None),
}


@pytest.mark.parametrize('src', la.REQUIRED_SOURCES)
@pytest.mark.parametrize('case', sorted(UNKNOWN_READINGS))
def test_unknown_reading_on_avoid_side_blocks(approved_policies, src, case):
  left = side(**{src: UNKNOWN_READINGS[case]})
  state, per = evaluate_side(left, LEFT, TEST_CFG)
  assert state == UNKNOWN and per[src] == UNKNOWN
  ctrl = LaneAvoidController(TEST_CFG)
  outs, _ = run(ctrl, [dict(shift=-0.3, left=left, right=side(radar=occ()))] * 30)
  assert not any(o.permitted for o in outs)


def test_missing_source_entry_is_unknown(approved_policies):
  left = side()
  del left['model']
  assert evaluate_side(left, LEFT, TEST_CFG)[0] == UNKNOWN
  assert evaluate_side({}, LEFT, TEST_CFG)[0] == UNKNOWN


def test_partial_coverage_rear_blind_zone_is_unknown(approved_policies):
  front_only = Coverage(0.0, 40.0, 0.0, 5.0)
  cov = {(s, sd): front_only for s in la.REQUIRED_SOURCES for sd in (LEFT, RIGHT)}
  cfg = LaneAvoidConfig(**{**TEST_CFG.__dict__, 'coverage': cov})
  assert evaluate_side(side(), LEFT, cfg)[0] == UNKNOWN
  # union of sources may cover: BSD rear + radar front
  cov2 = dict(cov)
  cov2[('bsd', LEFT)] = Coverage(-12.0, 1.0, 0.0, 5.0)
  cfg2 = LaneAvoidConfig(**{**TEST_CFG.__dict__, 'coverage': cov2})
  assert evaluate_side(side(), LEFT, cfg2)[0] == CLEAR


def test_region_coverage_requires_lateral_span_and_no_gaps():
  assert region_covered([Coverage(-10, 0, 0, 4), Coverage(0, 30, 0, 4)], (-10, 30), (0, 4))
  assert not region_covered([Coverage(-10, -1, 0, 4), Coverage(0, 30, 0, 4)], (-10, 30), (0, 4))
  assert not region_covered([Coverage(-10, 30, 0, 3)], (-10, 30), (0, 4))
  assert not region_covered([], (-10, 30), (0, 4))
  assert not region_covered([Coverage(-10, 30, 0, 4)], None, (0, 4))


def test_radar_lists_are_not_counted_as_model_evidence(approved_policies):
  # planner builder: radar side list may be clear, model source is unsupported
  readings = {'bsd': clear(), 'radar': clear(), 'model': SourceReading(supported=False)}
  state, per = evaluate_side(readings, LEFT, TEST_CFG)
  assert state == UNKNOWN and per['model'] == UNKNOWN


def test_radar_reading_detection_and_invalid_values():
  lead = lambda status, d: SimpleNamespace(status=status, dRel=d)  # noqa: E731
  assert radar_side_reading([lead(True, 5.0)], True, 0.0, (-10, 30)).detected is True
  assert radar_side_reading([lead(True, 80.0)], True, 0.0, (-10, 30)).detected is False
  assert radar_side_reading([lead(False, 5.0)], True, 0.0, (-10, 30)).detected is False
  assert radar_side_reading([lead(True, float('nan'))], True, 0.0, (-10, 30)).detected is None
  assert radar_side_reading(None, True, 0.0, (-10, 30)).detected is None
  assert radar_side_reading([], True, 0.0, None).detected is None
  # an empty list is "no detection", still unknown unless valid/fresh/covered
  assert evaluate_source(radar_side_reading([], None, 0.0, (-10, 30)), 0.2) == UNKNOWN


# ---------------------------------------------------------------- road edge (v3)


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_close_road_edge_on_avoid_side_blocks(approved_policies, direction):
  shift, left, right = mirror(direction)
  road = edges(left_y=-1.4) if direction == LEFT else edges(right_y=1.4)  # 1.4 < 0.95 + 0.5
  ctrl = LaneAvoidController(TEST_CFG)
  outs, frames = run(ctrl, [dict(shift=shift, left=left, right=right, road=road)] * 30)
  assert not any(o.permitted for o in outs)
  assert outs[-1].edge_state[direction] == OCCUPIED
  assert audit_frames(frames, TEST_CFG)['violations'] == []


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
@pytest.mark.parametrize('road_kw', [dict(std=2.0), dict(age=None), dict(age=1.0), dict(std=float('nan'))])
def test_uncertain_or_stale_road_edge_is_unknown(approved_policies, direction, road_kw):
  shift, left, right = mirror(direction)
  ctrl = LaneAvoidController(TEST_CFG)
  outs, _ = run(ctrl, [dict(shift=shift, left=left, right=right, road=edges(**road_kw))] * 30)
  assert not any(o.permitted for o in outs)
  assert outs[-1].edge_state[direction] == UNKNOWN


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_missing_road_edges_block(approved_policies, direction):
  shift, left, right = mirror(direction)
  ctrl = LaneAvoidController(TEST_CFG)
  outs, _ = run(ctrl, [dict(shift=shift, left=left, right=right, road={})] * 30)
  assert not any(o.permitted for o in outs)


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_avoid_amount_is_clipped_to_road_edge_room(approved_policies, direction):
  shift, left, right = mirror(direction)
  road = edges(left_y=-1.6) if direction == LEFT else edges(right_y=1.6)  # room 1.6 - 1.45 = 0.15
  ctrl = LaneAvoidController(TEST_CFG)
  outs, _ = run(ctrl, [dict(shift=shift, left=left, right=right, road=road)] * 80)
  assert outs[-1].permitted
  assert abs(outs[-1].target) == pytest.approx(0.15)
  assert max(abs(o.applied) for o in outs) <= 0.15 + 1e-12


def test_road_edge_not_extrapolated_and_shape_checked(approved_policies):
  short_x = np.linspace(0.0, 20.0, N)
  assert road_edge_allowance(X, np.zeros(N), short_x, np.full(N, -5.0), 0.1, 0.0, LEFT, TEST_CFG)[0] == UNKNOWN
  assert road_edge_allowance(X, np.zeros(N), X[:10], np.full(N, -5.0), 0.1, 0.0, LEFT, TEST_CFG)[0] == UNKNOWN
  bad = np.full(N, -5.0)
  bad[3] = np.nan
  assert road_edge_allowance(X, np.zeros(N), X, bad, 0.1, 0.0, LEFT, TEST_CFG)[0] == UNKNOWN
  assert road_edge_allowance(X, np.zeros(N), X, np.full(N, -5.0), 0.1, 0.0, LEFT, LaneAvoidConfig())[0] == UNKNOWN


# ---------------------------------------------------------------- in-progress block and return


def avoid_then(ctrl, direction, n=30):
  shift, left, right = mirror(direction)
  outs, frames = run(ctrl, [dict(shift=shift, left=left, right=right)] * n)
  assert outs[-1].state == AVOID
  return shift, left, right, outs, frames


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_new_avoid_side_detection_revokes_and_never_grows(approved_policies, direction):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, frames = avoid_then(ctrl, direction, n=12)  # still growing, rate > 0
  assert ctrl.rate != 0.0
  blocked = side(bsd=occ())
  seq_l, seq_r = (blocked, right) if direction == LEFT else (left, blocked)
  before = outs[-1].applied
  outs2, frames2 = run(ctrl, [dict(shift=shift, left=seq_l, right=seq_r)] * 20, t0=12 * DT)
  assert not outs2[0].permitted and outs2[0].target == 0.0
  # the original obstacle is still there -> no return movement, never further out
  assert all(abs(o.applied) <= abs(before) for o in outs2)
  assert all(o.applied == before for o in outs2)  # no movement toward the remaining obstacle either
  assert all(o.state in (RETURN, RETURN_RISK) and not o.permitted for o in outs2)
  assert outs2[-1].state == RETURN_RISK
  assert audit_frames(frames + frames2, TEST_CFG)['violations'] == []


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_return_only_when_return_side_clear_at_bounded_rate(approved_policies, direction):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, frames = avoid_then(ctrl, direction, n=40)
  peak = outs[-1].applied
  # obstacle passed: both sides clear, model back to centre
  outs2, frames2 = run(ctrl, [dict(shift=0.0)] * 120, t0=40 * DT)
  assert outs2[0].state in (RETURN, STANDBY) and not outs2[0].permitted
  steps = [abs(b.applied - a.applied) for a, b in zip([outs[-1]] + outs2[:-1], outs2)]
  assert max(steps) <= TEST_CFG.return_rate_mps * DT + 1e-12
  assert all(o.applied * peak >= 0.0 for o in outs2)  # never crosses to the other side
  assert outs2[-1].applied == 0.0 and outs2[-1].state == STANDBY
  assert audit_frames(frames + frames2, TEST_CFG)['result'] == 'no_violation_found'


RETURN_RISKS = {
  'obstacle_remains': lambda: side(radar=occ()),
  'return_side_bsd': lambda: side(bsd=occ()),
  'new_vehicle_model': lambda: side(model=occ()),
  'return_side_stale': lambda: side(radar=UNKNOWN_READINGS['stale']),
}


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
@pytest.mark.parametrize('case', sorted(RETURN_RISKS))
def test_return_risk_refuses_movement_toward_centre(approved_policies, direction, case):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, frames = avoid_then(ctrl, direction, n=40)
  risky = RETURN_RISKS[case]()
  seq = dict(shift=0.0, left=risky, right=side()) if direction == RIGHT else dict(shift=0.0, left=side(), right=risky)
  outs2, frames2 = run(ctrl, [seq] * 20, t0=40 * DT)
  assert all(o.applied == outs[-1].applied for o in outs2)
  assert all(o.state == RETURN_RISK and not o.permitted and o.target == 0.0 for o in outs2)
  assert 'return_risk_policy_unapproved' in outs2[-1].reasons
  assert audit_frames(frames + frames2, TEST_CFG)['violations'] == []


def test_both_sides_occupied_during_return(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  _, _, _, outs, _ = avoid_then(ctrl, LEFT, n=40)
  outs2, _ = run(ctrl, [dict(shift=-0.3, left=side(radar=occ()), right=side(radar=occ()))] * 20, t0=40 * DT)
  assert all(o.applied == outs[-1].applied and not o.permitted for o in outs2)


def test_return_side_road_edge_blocks_return(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  _, _, _, outs, _ = avoid_then(ctrl, LEFT, n=40)
  outs2, _ = run(ctrl, [dict(shift=0.0, road=edges(right_y=1.2))] * 10, t0=40 * DT)
  assert all(o.applied == outs[-1].applied for o in outs2)


def test_centre_reference_invalid_does_not_move_or_permit(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  _, _, _, outs, _ = avoid_then(ctrl, LEFT, n=40)
  outs2, _ = run(ctrl, [dict(shift=-0.3, left=side(), right=side(radar=occ()), center_valid=False)] * 10, t0=40 * DT)
  assert all(not o.permitted and o.applied == outs[-1].applied for o in outs2)
  assert 'center_invalid' in outs2[-1].reasons


def test_no_automatic_opposite_reavoid_and_reentry_wait(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  _, _, _, outs, _ = avoid_then(ctrl, LEFT, n=40)
  # obstacle switches sides at once: model shifts right with left occupied
  outs2, _ = run(ctrl, [dict(shift=0.3, left=side(radar=occ()), right=side())] * 200, t0=40 * DT)
  assert not outs2[0].permitted and 'direction_reversal' in outs2[0].reasons
  first_permit = next((i for i, o in enumerate(outs2) if o.permitted), None)
  end = len(outs2) if first_permit is None else first_permit
  # until the left offset has fully returned, nothing toward the right is permitted
  assert all(o.applied <= 0.0 for o in outs2[:end])
  if first_permit is not None:
    prev = outs2[first_permit - 1]
    assert prev.state == STANDBY and prev.applied == 0.0
    # revoked at outs2[0]; re-entry needs the full wait from that frame
    assert first_permit * DT >= TEST_CFG.reentry_wait_s - 1e-9


def test_alternating_sides_never_permit_toward_detection(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  seq = []
  for i in range(200):
    if (i // 7) % 2:
      seq.append(dict(shift=-0.3, left=side(bsd=occ()), right=side(radar=occ())))
    else:
      seq.append(dict(shift=0.3, left=side(radar=occ()), right=side(bsd=occ())))
  outs, frames = run(ctrl, seq)
  assert not any(o.permitted for o in outs)
  assert audit_frames(frames, TEST_CFG)['violations'] == []


# ---------------------------------------------------------------- time base and inputs


@pytest.mark.parametrize('bad_t', [float('nan'), None, 'reverse', 'repeat', 'gap'])
def test_time_faults_revoke_and_freeze(approved_policies, bad_t):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, _ = avoid_then(ctrl, LEFT, n=12)
  last_t = 11 * DT
  t = {'reverse': last_t - 1.0, 'repeat': last_t, 'gap': last_t + 1.0}.get(bad_t, bad_t)
  o = ctrl.update(inp(t, shift, left, right))
  assert not o.permitted and o.target == 0.0
  assert o.applied == outs[-1].applied


@pytest.mark.parametrize('kw,reason', [
  (dict(lane_change_active=True), 'lane_change_or_desire'),
  (dict(driver_steering=True), 'driver_steering'),
  (dict(measured_curvature=0.02), 'curve_not_separable'),
  (dict(measured_curvature=float('nan')), 'curve_not_separable'),
  (dict(model_valid=False), 'model_invalid_or_stale'),
  (dict(model_age_s=1.0), 'model_invalid_or_stale'),
  (dict(model_age_s=None), 'model_invalid_or_stale'),
  (dict(lane_y=None), 'no_lane_path'),
  (dict(d_prob=float('nan')), 'candidate_d_prob'),
  (dict(d_prob=1.5), 'candidate_d_prob'),
])
def test_input_conditions_block_start(approved_policies, kw, reason):
  shift, left, right = mirror(LEFT)
  ctrl = LaneAvoidController(TEST_CFG)
  outs = [ctrl.update(inp(i * DT, shift, left, right, **kw)) for i in range(30)]
  assert not any(o.permitted for o in outs)
  assert reason in outs[-1].reasons


def test_candidate_rejections():
  y0 = np.zeros(N)
  assert avoid_candidate(X, y0 - 0.3, y0, 1.0, TEST_CFG)[0] == pytest.approx(-0.3)
  assert avoid_candidate(X, y0 - 0.3, y0, 0.5, TEST_CFG)[0] == pytest.approx(-0.15)  # (1-d_prob) part not re-added
  assert avoid_candidate(X, y0 + 0.01, y0, 1.0, TEST_CFG)[1] == 'candidate_below_deadband'
  mixed = y0.copy()
  mixed[5], mixed[10] = -0.3, 0.2
  assert avoid_candidate(X, mixed, y0, 1.0, TEST_CFG)[1] == 'candidate_mixed_direction'
  nan = y0.copy()
  nan[4] = np.inf
  assert avoid_candidate(X, nan, y0, 1.0, TEST_CFG)[1] == 'candidate_nonfinite'
  assert avoid_candidate(X, y0[:10], y0, 1.0, TEST_CFG)[1] == 'candidate_shape'
  assert avoid_candidate(X[::-1], y0, y0, 1.0, TEST_CFG)[1] == 'candidate_x_not_monotonic'
  assert avoid_candidate(X, y0, y0, 1.0, LaneAvoidConfig())[1] == 'candidate_unapproved'
  far = LaneAvoidConfig(**{**TEST_CFG.__dict__, 'candidate_x_m': (200.0, 300.0)})
  assert avoid_candidate(X, y0 - 0.3, y0, 1.0, far)[1] == 'candidate_empty_region'


def test_normal_curve_common_to_model_and_lane_is_not_a_candidate():
  curve = 0.002 * X ** 2
  assert avoid_candidate(X, curve, curve, 1.0, TEST_CFG)[0] is None


def test_candidate_does_not_mutate_inputs():
  my, ly = np.full(N, -0.3), np.zeros(N)
  my0, ly0 = my.copy(), ly.copy()
  avoid_candidate(X, my, ly, 1.0, TEST_CFG)
  assert np.array_equal(my, my0) and np.array_equal(ly, ly0)


def test_offset_never_exceeds_max_or_rate_limits(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  outs, _ = run(ctrl, [dict(shift=-2.0, left=side(), right=side(radar=occ()))] * 200)
  assert max(abs(o.applied) for o in outs) <= TEST_CFG.max_offset_m + 1e-12
  steps = [abs(b.applied - a.applied) for a, b in zip(outs, outs[1:])]
  assert max(steps) <= TEST_CFG.entry_rate_mps * DT + 1e-12
  rates = [(b.applied - a.applied) / DT for a, b in zip(outs, outs[1:])]
  # continuity holds except where the offset snaps onto its target (documented exception)
  jumps = [abs(rates[k] - rates[k - 1]) for k in range(1, len(rates))
           if outs[k].applied != outs[k].target and outs[k + 1].applied != outs[k + 1].target]
  assert jumps and max(jumps) <= TEST_CFG.max_rate_change_mps2 * DT + 1e-9
  assert outs[-1].applied == pytest.approx(-TEST_CFG.max_offset_m)


# ---------------------------------------------------------------- audit (strict failure)


def af(t, state, applied, permitted=False, target=0.0, avoid_side=LEFT, left=CLEAR, right=CLEAR, el=CLEAR, er=CLEAR):
  return {'t': t, 'state': state, 'permitted': permitted, 'target': target, 'applied': applied,
          'avoid_side': avoid_side, 'side_state': {LEFT: left, RIGHT: right}, 'edge_state': {LEFT: el, RIGHT: er}}


def test_audit_counts_return_frames_and_exclusion_gives_false_pass(approved_policies):
  frames = [af(0.0, AVOID, -0.20, True, -0.2, right=OCCUPIED),
            af(0.05, RETURN, -0.19, right=OCCUPIED)]  # residual moves toward the occupied side
  full = audit_frames(frames, TEST_CFG)
  assert full['result'] == 'fail'
  assert full['violations'][0]['rule'] == 'return_move_without_clear_return_side'
  filtered = audit_frames([f for f in frames if f['state'] not in (RETURN, RETURN_RISK)], TEST_CFG)
  assert filtered['result'] != 'fail'  # demonstrates why return frames must never be excluded


def test_audit_single_frame_violations(approved_policies):
  base = af(0.0, AVOID, -0.20, True, -0.2, right=OCCUPIED)
  cases = {
    'growth_without_permission': af(0.05, RETURN, -0.21, right=OCCUPIED),
    'permitted_avoid_side_not_clear': af(0.05, AVOID, -0.21, True, -0.3, left=UNKNOWN, right=OCCUPIED),
    'permitted_avoid_edge_not_clear': af(0.05, AVOID, -0.21, True, -0.3, el=UNKNOWN, right=OCCUPIED),
    'rate_above_limit': af(0.05, AVOID, -0.30, True, -0.3, right=OCCUPIED),
    'offset_crossed_sides': af(0.05, RETURN, 0.001),
    'offset_changed_without_time': af(0.0, AVOID, -0.21, True, -0.3, right=OCCUPIED),
    'target_without_permission': af(0.05, RETURN, -0.20, target=-0.2, right=OCCUPIED),
    'offset_above_limit': af(0.05, RETURN_RISK, -0.6, right=OCCUPIED),
  }
  for rule, f in cases.items():
    res = audit_frames([base, f], TEST_CFG)
    assert res['result'] == 'fail', rule
    assert rule in [v['rule'] for v in res['violations']], rule


def test_audit_not_evaluable_without_approval_or_frames():
  frames = [af(0.0, STANDBY, 0.0, avoid_side=None)]
  assert audit_frames(frames, LaneAvoidConfig())['result'] == 'not_evaluable'
  assert audit_frames([], TEST_CFG)['result'] == 'not_evaluable'


def test_audit_invalid_frames_are_counted_not_zero_filled(approved_policies):
  frames = [af(0.0, STANDBY, 0.0, avoid_side=None), {'t': 0.05}, af(0.1, STANDBY, float('nan'), avoid_side=None)]
  res = audit_frames(frames, TEST_CFG)
  assert res['invalid_frames'] == [1, 2]
  assert res['result'] == 'not_evaluable'


def test_audit_violation_is_kept_even_if_other_frames_are_fine(approved_policies):
  ok = [af(i * DT, STANDBY, 0.0, avoid_side=None) for i in range(50)]
  bad = ok + [af(50 * DT, RETURN, 0.01, avoid_side=RIGHT)]
  assert audit_frames(bad, TEST_CFG)['result'] == 'fail'


# ---------------------------------------------------------------- planner / lane planner wiring


def test_planner_default_disabled_preserves_path_exactly():
  planner = make_planner()
  assert not planner.lane_avoid.active
  for _ in range(30):
    sm = planner_inputs()
    planner.update(sm, SimpleNamespace(atc_active=False))
    assert planner.lane_avoid_out is None
    # stub lane planner returns the model path; only PathOffset (stub param 1 -> 0.01 m) is added
    assert np.array_equal(planner.path_xyz[:, 1], np.asarray(sm['modelV2'].position.y) + 0.01)


class SM(dict):
  pass


def lead(status=False, d_rel=0.0):
  return SimpleNamespace(status=status, dRel=d_rel)


def planner_sm(frame, left_bsd=False):
  sm = SM(planner_inputs())
  t_ns = int(1e9 + frame * DT * 1e9)
  sm.logMonoTime = {'modelV2': t_ns, 'radarState': t_ns}
  sm.valid = {'modelV2': True, 'radarState': True}
  sm['carState'].leftBlindspot, sm['carState'].rightBlindspot = left_bsd, False
  sm['carState'].steeringPressed = False
  sm['radarState'] = SimpleNamespace(
    radarErrors=SimpleNamespace(canError=False, radarFault=False, wrongConfig=False, radarUnavailableTemporary=False),
    leadLeft=lead(), leadsLeft=[], leadRight=lead(), leadsRight=[lead(True, 3.0)])
  md = sm['modelV2']
  md.roadEdges = [SimpleNamespace(x=X, y=np.full(N, -5.0)), SimpleNamespace(x=X, y=np.full(N, 5.0))]
  md.roadEdgeStds = [0.1, 0.1]
  md.meta.laneChangeState = 0
  return sm


@pytest.mark.parametrize('left_bsd', [False, True])
def test_planner_builder_cannot_clear_with_current_signals(approved_policies, left_bsd):
  planner = make_planner()
  g = type(planner).update.__globals__  # exec namespace of the compiled production class
  g.update({'AvoidInputs': AvoidInputs, 'SourceReading': SourceReading, 'LEFT': LEFT, 'RIGHT': RIGHT,
            'radar_side_reading': radar_side_reading,
            'log': SimpleNamespace(Desire=SimpleNamespace(none=0), LaneChangeState=SimpleNamespace(off=0))})
  planner.lane_avoid = LaneAvoidController(TEST_CFG)
  planner.LP.model_path_y = np.full(N, -0.3)  # model sidesteps left of a right-side radar object
  planner.LP.lane_path_y = np.zeros(N)
  planner.LP.blend_d_prob = 1.0
  for i in range(40):
    sm = planner_sm(i, left_bsd)
    planner.update(sm, SimpleNamespace(atc_active=False))
    o = planner.lane_avoid_out
    assert not o.permitted and o.applied == 0.0
    assert np.array_equal(planner.path_xyz[:, 1], np.asarray(sm['modelV2'].position.y) + 0.01)
  assert o.side_state[RIGHT] == OCCUPIED
  assert o.source_state[LEFT]['model'] == UNKNOWN  # no independent model side observation
  assert o.source_state[LEFT]['bsd'] == (OCCUPIED if left_bsd else UNKNOWN)  # BSD False is not clear
  assert o.side_state[LEFT] == (OCCUPIED if left_bsd else UNKNOWN)


def make_lane_planner():
  source = Path(__file__).parents[1] / 'lib/lane_planner_2.py'
  tree = ast.parse(source.read_text(encoding='utf8'))
  params = SimpleNamespace(get_int=lambda _: 0, get_float=lambda _: 0.0)
  namespace = {'np': np, 'math': math, 'FirstOrderFilter': FirstOrderFilter, 'DT_MDL': 0.05,
               'TRAJECTORY_SIZE': N, 'CAMERA_OFFSET': 0, 'Params': lambda: params}
  exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'LanePlanner'],
                          type_ignores=[]), str(source), 'exec'), namespace)
  lp = namespace['LanePlanner']()
  t = np.linspace(0.0, 10.0, N)
  lp.ll_t, lp.ll_x = t, X
  lp.lll_y, lp.rll_y = np.full(N, -1.75), np.full(N, 1.75)
  lp.lll_prob = lp.rll_prob = 0.9
  lp.lll_std = lp.rll_std = 0.1
  return lp, t


@pytest.mark.parametrize('lanefull', [True, False])
def test_lane_planner_copies_do_not_change_blend(lanefull):
  lp, t = make_lane_planner()
  lp.lanefull_mode = lanefull
  for _ in range(30):
    model = np.column_stack([X, np.full(N, 0.4), np.zeros(N)])
    original = model[:, 1].copy()
    out, active = lp.get_d_path(None, 25.0, t, model, 0.0)
    assert np.array_equal(lp.model_path_y, original)
    assert not np.shares_memory(lp.model_path_y, out)
    offset = 0 + lp.lane_offset_filtered.x
    if active:
      expect = lp.blend_d_prob * lp.lane_path_y + (1.0 - lp.blend_d_prob) * original + offset
    else:
      assert lp.lane_path_y is None
      expect = original + offset
    assert np.array_equal(out[:, 1], expect)
  assert active == lanefull


# ================================================================ v3.1 (lanemode_avoid_prompt_v3_1)
# Written for review; NOT executed. All numbers are synthetic fixture inputs.


def run_t(ctrl, times, make):
  outs, frames = [], []
  for i, t in enumerate(times):
    o = ctrl.update(make(i, t))
    outs.append(o)
    frames.append(frame(o, t))
  return outs, frames


def rules(res):
  return [v['rule'] for v in res['violations']]


def cfg_with(**kw):
  return LaneAvoidConfig(**{**TEST_CFG.__dict__, **kw})


def road_on(direction, x, y_out):
  """road edge on `direction` at outward distance y_out (array over x); the other side at 5 m."""
  road = edges()
  road[direction] = (x, la.SIDE_SIGN[direction] * np.asarray(y_out, dtype=float), 0.1, 0.0)
  return road


def patch_planner_globals(planner):
  g = type(planner).update.__globals__  # exec namespace of the compiled production class
  g.update({'AvoidInputs': AvoidInputs, 'SourceReading': SourceReading, 'LEFT': LEFT, 'RIGHT': RIGHT,
            'radar_side_reading': radar_side_reading,
            'log': SimpleNamespace(Desire=SimpleNamespace(none=0), LaneChangeState=SimpleNamespace(off=0))})


# ---------------------------------------------------------------- hold 1: whole consumed path


def test_required_span_covers_body_and_whole_path():
  assert la.required_x_range(X, TEST_CFG) == (-1.0, 104.0)  # rear at ego .. front beyond the last point
  assert la.required_x_range(X, LaneAvoidConfig()) is None
  assert la.required_x_range(X[::-1], TEST_CFG) is None
  sp = la.required_space(X, np.zeros(N), [0.0, -0.3], LEFT, TEST_CFG)
  assert (sp.x_min, sp.x_max, sp.lat_min) == (-1.0, 104.0, 0.0)
  assert sp.lat_max == pytest.approx(0.3 + 0.95 + 0.3)  # offset + half width + clearance
  # the base path's own lateral excursion counts too (not only the offset)
  bend = np.where(X > 70.0, -0.8, 0.0)
  # repair 2 (human decision 2026-10-10): the wider, more conservative rotated-corner model
  # replaces the band expectation 0.8 + 1.25 = 2.05. Independent derivation: X spacing
  # h = 100/32 = 3.125; the step lies between X[22] = 68.75 and X[23] = 71.875, so the
  # central-difference slope at X[23] is k = -0.8 / (2h) = -0.128 and its heading has
  # sin = -0.128/sqrt(1+k^2), cos = 1/sqrt(1+k^2). The outermost LEFT (-y) corner of that
  # pose is the body front (4.0 m) at the left half width (0.95 m):
  #   lat_max = 0.8 + (4.0 * 0.128 + 0.95) / sqrt(1 + 0.128^2) + 0.3 (side clearance)
  #           = 2.550168484768921 (band alone: 0.8 + 0.95 + 0.3 = 2.05 is smaller)
  k = 0.8 / (2.0 * (100.0 / 32.0))
  assert la.required_space(X, bend, [0.0], LEFT, TEST_CFG).lat_max == pytest.approx(
    0.8 + (4.0 * k + 0.95) / math.sqrt(1.0 + k * k) + 0.3)
  assert la.required_space(X, np.zeros(N), [float('nan')], LEFT, TEST_CFG) is None


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_far_path_point_edge_intrusion_outside_candidate_blocks(approved_policies, direction):
  # candidate range (5, 60) is wide; the edge closes in only at x > 80 on the consumed path
  road = road_on(direction, EDGE_X, np.where(EDGE_X > 80.0, 1.4, 5.0))
  state, _ = road_edge_allowance(X, np.zeros(N), *road[direction], direction, TEST_CFG)
  assert state == OCCUPIED
  shift, left, right = mirror(direction)
  outs, frames = run(LaneAvoidController(TEST_CFG), [dict(shift=shift, left=left, right=right, road=road)] * 30)
  assert not any(o.permitted for o in outs) and all(o.applied == 0.0 for o in outs)
  assert outs[-1].edge_state[direction] == OCCUPIED
  assert audit_frames(frames, TEST_CFG)['violations'] == []


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_edge_narrowing_between_path_points_and_beyond_last_point(direction):
  # v3.2 fixture: starts at -5 m so the body rear is observed (was np.linspace(0.0, 110.0, 221))
  dense = np.linspace(-5.0, 110.0, 231)  # 0.5 m knots; 51.5 m lies between path points 50.0 and 53.125
  y = np.full(dense.size, 5.0)
  y[np.isclose(dense, 51.5)] = 1.0
  road = road_on(direction, dense, y)
  assert road_edge_allowance(X, np.zeros(N), *road[direction], direction, TEST_CFG)[0] == OCCUPIED
  # narrowing ahead of the last path point, inside the body front (100 .. 104 m)
  y2 = np.where(dense > 102.0, 1.0, 5.0)
  road2 = road_on(direction, dense, y2)
  assert road_edge_allowance(X, np.zeros(N), *road2[direction], direction, TEST_CFG)[0] == OCCUPIED


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_edge_partial_observation_is_unknown_never_extrapolated(direction):
  for ex in (np.linspace(1.0, 110.0, N),     # starts after the current position
             np.linspace(0.0, 102.0, N)):    # ends before the body front at 104 m
    road = road_on(direction, ex, np.full(N, 5.0))
    assert road_edge_allowance(X, np.zeros(N), *road[direction], direction, TEST_CFG)[0] == UNKNOWN
  full = road_on(direction, EDGE_X, np.full(N, 5.0))
  state, room = road_edge_allowance(X, np.zeros(N), *full[direction], direction, TEST_CFG)
  assert state == CLEAR and room == pytest.approx(5.0 - 0.95 - 0.5)


def test_body_width_and_clearance_alone_make_coverage_insufficient(approved_policies):
  cfg = cfg_with(required_region_lat_m=(0.0, 1.0))  # fixed region is satisfied by 1.5 m coverage
  sp = la.required_space(X, np.zeros(N), [0.0, -0.3], LEFT, cfg)  # needs 1.55 m
  narrow = {k: Coverage(-15.0, 115.0, 0.0, 1.5) for k in cfg.coverage}
  wide = {k: Coverage(-15.0, 115.0, 0.0, 1.6) for k in cfg.coverage}
  assert evaluate_side(side(), LEFT, cfg_with(required_region_lat_m=(0.0, 1.0), coverage=narrow))[0] == CLEAR
  assert evaluate_side(side(), LEFT, cfg_with(required_region_lat_m=(0.0, 1.0), coverage=narrow), (sp,))[0] == UNKNOWN
  assert evaluate_side(side(), LEFT, cfg_with(required_region_lat_m=(0.0, 1.0), coverage=wide), (sp,))[0] == CLEAR
  assert evaluate_side(side(), LEFT, TEST_CFG, (None,))[0] == UNKNOWN  # not computable is never clear


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_coverage_of_fixed_region_only_is_unknown_for_the_path(approved_policies, direction):
  short = {k: Coverage(-15.0, 40.0, 0.0, 5.0) for k in TEST_CFG.coverage}  # spans the fixed region only
  cfg = cfg_with(coverage=short)
  assert evaluate_side(side(), direction, cfg)[0] == CLEAR
  shift, left, right = mirror(direction)
  outs, _ = run(LaneAvoidController(cfg), [dict(shift=shift, left=left, right=right)] * 30)
  assert not any(o.permitted for o in outs)
  assert outs[-1].side_state[direction] == UNKNOWN and f'avoid_side_{UNKNOWN}' in outs[-1].reasons


def test_radar_object_outside_fixed_region_but_on_path_is_detected(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  region = ctrl.radar_region_x(X)
  assert region == (-10.0, 104.0)  # fixed (-10, 30) united with the dynamic span
  far = [SimpleNamespace(status=True, dRel=80.0)]
  assert radar_side_reading(far, True, 0.0, TEST_CFG.required_region_x_m).detected is False  # the v3 gap
  assert radar_side_reading(far, True, 0.0, region).detected is True
  assert ctrl.radar_region_x(X[::-1]) is None
  assert LaneAvoidController(LaneAvoidConfig()).radar_region_x(X) is None


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_return_space_is_part_of_every_frame(approved_policies, direction):
  ctrl = LaneAvoidController(TEST_CFG)
  _, _, _, outs, _ = avoid_then(ctrl, direction, n=40)
  ret = la.other_side(direction)
  sp = outs[-1].required_space[ret]
  # the return side's space includes the body at offset 0 over the whole path
  assert (sp.x_min, sp.x_max) == (-1.0, 104.0) and sp.lat_max == pytest.approx(0.95 + 0.3)
  # only the return space occupied (avoid side clear): no return movement, no growth
  risky = side(model=occ())
  seq = dict(shift=0.0, left=risky, right=side()) if ret == LEFT else dict(shift=0.0, left=side(), right=risky)
  outs2, frames2 = run(ctrl, [seq] * 10, t0=40 * DT)
  assert all(o.applied == outs[-1].applied and not o.permitted for o in outs2)


def test_planner_consumes_exactly_the_controller_offset(approved_policies):
  # final consumption: the published path is the base path plus out.applied, nothing else
  planner = make_planner()
  planner.lane_avoid = LaneAvoidController(TEST_CFG)
  shift, left, right = mirror(LEFT)
  n = [0]

  def fake_inputs(sm, carrot, md, model_active):
    n[0] += 1
    return inp(1.0 + n[0] * DT, shift, left, right)
  planner.lane_avoid_inputs = fake_inputs
  # repair 2 (human decision 2026-10-10): LaneModelSpeedGuard's readiness time is intended;
  # advance planner frames until lane mode is active, then check the unchanged asserts
  for _ in range(200):
    planner.update(planner_inputs(), SimpleNamespace(atc_active=False))
    if planner.lanelines_active:
      break
  assert planner.lanelines_active
  seen = []
  for _ in range(40):
    sm = planner_inputs()
    planner.update(sm, SimpleNamespace(atc_active=False))
    o = planner.lane_avoid_out
    assert planner.lanelines_active and o is not None
    assert np.array_equal(planner.path_xyz[:, 1], np.asarray(sm['modelV2'].position.y) + 0.01 + o.applied)
    seen.append(o.applied)
  assert min(seen) < 0.0


# ---------------------------------------------------------------- hold 2: message-time freshness


def test_same_message_with_advancing_evaluation_time_goes_stale(approved_policies):
  shift, left, right = mirror(LEFT)
  ctrl = LaneAvoidController(TEST_CFG)
  outs = [ctrl.update(inp(i * DT, shift, left, right, model_t=0.0, model_age_s=i * DT)) for i in range(30)]
  assert not any(o.permitted for o in outs) and all(o.applied == 0.0 for o in outs)
  assert 'model_not_new' in outs[1].reasons
  assert 'model_invalid_or_stale' in outs[-1].reasons


def test_builder_reported_age_zero_cannot_hide_old_message_time(approved_policies):
  shift, left, right = mirror(LEFT)
  ctrl = LaneAvoidController(TEST_CFG)
  # new message each frame but stamped 1 s before evaluation; builder claims age 0
  outs = [ctrl.update(inp(1.0 + i * DT, shift, left, right, model_t=i * DT, model_age_s=0.0)) for i in range(30)]
  assert not any(o.permitted for o in outs)
  assert 'model_invalid_or_stale' in outs[-1].reasons


MSG_TIME_FAULTS = {
  'missing': (lambda t: dict(model_t=None), 'model_time_invalid'),
  'nan': (lambda t: dict(model_t=float('nan')), 'model_time_invalid'),
  'future': (lambda t: dict(model_t=t + 0.01), 'model_time_future'),
  'reversed': (lambda t: dict(model_t=10.0 - t), 'model_not_new'),
  'repeated': (lambda t: dict(model_t=0.0), 'model_not_new'),
  'clock_unverified': (lambda t: dict(clock_verified=False), 'clock_unverified'),
}


@pytest.mark.parametrize('case', sorted(MSG_TIME_FAULTS))
def test_message_time_faults_block_start(approved_policies, case):
  make, reason = MSG_TIME_FAULTS[case]
  shift, left, right = mirror(LEFT)
  ctrl = LaneAvoidController(TEST_CFG)
  outs = [ctrl.update(inp(i * DT, shift, left, right, **make(i * DT))) for i in range(30)]
  assert not any(o.permitted for o in outs)
  assert reason in outs[-1].reasons


@pytest.mark.parametrize('case', sorted(MSG_TIME_FAULTS))
def test_message_time_faults_revoke_in_progress_without_reusing_permission(approved_policies, case):
  make, reason = MSG_TIME_FAULTS[case]
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, _ = avoid_then(ctrl, LEFT, n=40)
  before = outs[-1].applied
  outs2 = []
  for i in range(10):
    t = (40 + i) * DT
    kw = make(t) if case != 'repeated' else dict(model_t=39 * DT)  # the last accepted message again
    outs2.append(ctrl.update(inp(t, shift, left, right, **kw)))
  assert all(not o.permitted and o.target == 0.0 for o in outs2)
  assert all(abs(o.applied) <= abs(before) for o in outs2)
  assert any(reason in o.reasons for o in outs2)  # 'reversed' shows as future first, then not new


def test_freshness_boundary_both_sides(approved_policies):
  shift, left, right = mirror(LEFT)
  ok = LaneAvoidController(TEST_CFG).update(inp(1.0, shift, left, right, model_t=0.8125, model_age_s=0.1875))
  assert 'model_invalid_or_stale' not in ok.reasons
  late = LaneAvoidController(TEST_CFG).update(inp(1.0, shift, left, right, model_t=0.75, model_age_s=0.25))
  assert 'model_invalid_or_stale' in late.reasons
  # an unapproved freshness limit is a blocker, not age 0
  assert 'max_age_model_path_unapproved' in LaneAvoidConfig(max_age_s={'bsd': 0.2}).blockers()


def test_builder_ages_come_from_message_times(approved_policies):
  planner = make_planner()
  patch_planner_globals(planner)
  planner.lane_avoid = LaneAvoidController(TEST_CFG)
  planner.path_xyz = np.column_stack([X, np.zeros(N), np.zeros(N)])
  planner.lanelines_active = True
  planner.LP.model_path_y, planner.LP.lane_path_y, planner.LP.blend_d_prob = np.full(N, -0.3), np.zeros(N), 1.0
  carrot = SimpleNamespace(atc_active=False)
  sm = planner_sm(0)
  sm.logMonoTime = {'modelV2': int(2.9e9), 'radarState': int(1.0e9)}
  planner.lane_avoid_clock = lambda: 3.0
  a = planner.lane_avoid_inputs(sm, carrot, sm['modelV2'], True)
  assert a.t == 3.0 and a.model_t == pytest.approx(2.9) and a.model_age_s == pytest.approx(0.1)
  assert a.road_edges[LEFT][3] == a.model_age_s  # edges inherit the age of the model message carrying them
  assert a.side_readings[LEFT]['radar'].age_s == pytest.approx(2.0)  # only the radar is old
  assert evaluate_source(a.side_readings[LEFT]['radar'], 0.2) == UNKNOWN
  assert evaluate_source(a.side_readings[RIGHT]['radar'], 0.2) == OCCUPIED  # detection stays a veto
  # unchanged messages, later evaluation: the age grows, it is never reset to 0
  planner.lane_avoid_clock = lambda: 3.5
  b = planner.lane_avoid_inputs(sm, carrot, sm['modelV2'], True)
  assert b.model_t == a.model_t and b.model_age_s == pytest.approx(0.6)
  # a message stamped after the evaluation time gives a negative age (rejected downstream)
  sm.logMonoTime = {'modelV2': int(3.6e9), 'radarState': int(3.6e9)}
  assert planner.lane_avoid_inputs(sm, carrot, sm['modelV2'], True).model_age_s < 0.0
  # never received: no time at all, not age 0
  sm.logMonoTime = {'modelV2': 0, 'radarState': 0}
  c = planner.lane_avoid_inputs(sm, carrot, sm['modelV2'], True)
  assert c.model_t is None and c.model_age_s is None and c.side_readings[LEFT]['radar'].age_s is None
  # model output not active: edges get no age
  assert planner.lane_avoid_inputs(sm, carrot, sm['modelV2'], False).road_edges[LEFT][3] is None


# ---------------------------------------------------------------- hold 3: consumed offset inside edge room


def test_edge_shrink_inside_continuity_keeps_applied_within_room(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  _, _, _, outs, frames = avoid_then(ctrl, LEFT, n=40)
  assert outs[-1].applied == pytest.approx(-0.3) and ctrl.rate == 0.0
  # obstacle passed (return side clear) while the left room shrinks to ~0.298 < |applied|
  outs2, frames2 = run(ctrl, [dict(shift=0.0, road=edges(left_y=-1.748))] * 60, t0=40 * DT)
  for o in outs2:
    assert o.state != la.CONFLICT and not o.conflict
    assert abs(o.applied) <= o.edge_room[LEFT] + 1e-12
  assert outs2[-1].applied == 0.0
  assert audit_frames(frames + frames2, TEST_CFG)['violations'] == []


EDGE_SHRINK = {
  'room_small': lambda: edges(left_y=-1.6),     # room 0.15 < 0.3, cannot shrink 0.15 m in one frame
  'room_zero': lambda: edges(left_y=-1.45),
  'edge_unknown': lambda: edges(std=2.0),
  'edge_direction': lambda: edges(left_y=1.0),  # "left" edge reported right of the path
}


@pytest.mark.parametrize('case', sorted(EDGE_SHRINK))
@pytest.mark.parametrize('return_clear', [False, True])
def test_edge_shrink_without_feasible_offset_is_conflict_and_blocked(approved_policies, case, return_clear):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, frames = avoid_then(ctrl, LEFT, n=40)
  seq = dict(shift=0.0) if return_clear else dict(shift=shift, left=left, right=right)
  outs2, frames2 = run(ctrl, [dict(seq, road=EDGE_SHRINK[case]())] * 5, t0=40 * DT)
  o = outs2[0]
  assert o.state == la.CONFLICT and o.conflict and not o.permitted and o.target == 0.0
  assert 'constraint_conflict_policy_unapproved' in o.reasons
  # the refusal is visible: the audit fails on the out-of-room frame (never hidden as a residual)
  assert 'applied_exceeds_edge_room' in rules(audit_frames(frames + frames2, TEST_CFG))
  # production: the conflict policy is unapproved, so the controller cannot be active at all
  assert 'constraint_conflict_policy_unapproved' in LaneAvoidConfig().blockers()


def test_production_policy_tuples_stay_empty_and_block():
  for name in POLICY_TUPLES:
    assert getattr(la, name) == ()
  b = TEST_CFG.blockers()
  for k in ('return_risk_policy_unapproved', 'center_invalid_policy_unapproved',
            'occupancy_prediction_policy_unapproved', 'body_geometry_model_unapproved',
            'constraint_conflict_policy_unapproved'):
    assert k in b
  assert not LaneAvoidController(TEST_CFG).active
  d = LaneAvoidConfig().blockers()
  for k in ('vehicle_front_m_unapproved', 'vehicle_rear_m_unapproved', 'side_clearance_m_unapproved',
            'max_age_model_path_unapproved', 'max_input_gap_s_unapproved'):
    assert k in d


# ---------------------------------------------------------------- hold 4: continuity on every sample


def test_continuity_holds_on_every_sample_including_target_reached(approved_policies):
  # stronger companion of test_offset_never_exceeds_max_or_rate_limits: no sample is excluded
  ctrl = LaneAvoidController(TEST_CFG)
  outs, frames = run(ctrl, [dict(shift=-2.0, left=side(), right=side(radar=occ()))] * 200)
  applied = [0.0] + [o.applied for o in outs]  # starts at 0 with rate 0
  rates = [0.0] + [(b - a) / DT for a, b in zip(applied, applied[1:])]
  assert max(abs(b - a) for a, b in zip(rates, rates[1:])) <= TEST_CFG.max_rate_change_mps2 * DT + 1e-9
  assert all(abs(o.rate - r) <= 1e-9 for o, r in zip(outs, rates[1:]))
  assert any(o.applied == o.target != 0.0 for o in outs)  # reached frames are in the sample
  res = audit_frames(frames, TEST_CFG)
  assert res['result'] == 'no_violation_found' and res['non_differentiable'] == []


def test_continuity_with_non_uniform_intervals_and_transitions(approved_policies):
  steps = [0.04, 0.06, 0.05, 0.03, 0.07]
  times = np.cumsum([0.0] + [steps[i % 5] for i in range(299)])
  shift, left, right = mirror(LEFT)

  def make(i, t):
    if i < 60:
      return inp(t, shift, left, right)                    # avoid left, reach target, hold
    if i < 70:
      return inp(t, shift, side(bsd=occ()), right)         # detection on the avoid side revokes
    if i < 200:
      return inp(t, 0.0)                                    # obstacle passed: return to 0
    return inp(t, -shift, side(radar=occ()), side())       # opposite avoidance (sign change via 0)
  ctrl = LaneAvoidController(TEST_CFG)
  outs, frames = run_t(ctrl, times, make)
  res = audit_frames(frames, TEST_CFG)
  assert res['frames'] == len(outs) and res['non_differentiable'] == []
  assert any(o.applied < 0.0 for o in outs) and any(o.applied > 0.0 for o in outs)
  # any violation may only appear on a CONFLICT frame, which production cannot reach
  assert all(outs[v['frame']].state == la.CONFLICT for v in res['violations'])
  assert all(o.state != la.CONFLICT or 'constraint_conflict_policy_unapproved' in o.reasons for o in outs)


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_return_risk_while_moving_is_reported_conflict_not_hidden(approved_policies, direction):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, frames = avoid_then(ctrl, direction, n=12)  # still growing
  rate0 = ctrl.rate
  assert abs(rate0) > TEST_CFG.max_rate_change_mps2 * DT
  blocked = side(bsd=occ())
  l2, r2 = (blocked, right) if direction == LEFT else (left, blocked)
  outs2, frames2 = run(ctrl, [dict(shift=shift, left=l2, right=r2)] * 10, t0=12 * DT)
  # neither growth (avoid side occupied) nor return (obstacle remains) is allowed.
  # repair 2 (human decision 2026-10-10): blocking movement has priority and continuity is
  # relaxed only toward stopping the growth, so the rate drops to 0 at once and this is no
  # longer an empty intersection; the risk is still reported (not hidden). Previously:
  #   assert outs2[0].state == la.CONFLICT and 'constraint_conflict_policy_unapproved' in outs2[0].reasons
  #   assert any(c.startswith('conflict_') for c in outs2[0].conflict)
  #   assert res['violations'] and all(all_outs[v['frame']].state == la.CONFLICT for v in res['violations'])
  assert outs2[0].state == la.RETURN_RISK and 'return_risk_policy_unapproved' in outs2[0].reasons
  assert outs2[0].conflict == () and all(o.valid for o in outs2)
  res = audit_frames(frames + frames2, TEST_CFG)
  assert res['violations'] == []


def test_audit_checks_rate_change_on_target_reached_frame(approved_policies):
  frames = [dict(af(0.0, AVOID, -0.2000, True, -0.2125, right=OCCUPIED), rate=-0.25),
            dict(af(0.05, AVOID, -0.2125, True, -0.2125, right=OCCUPIED), rate=-0.25),
            dict(af(0.10, AVOID, -0.2125, True, -0.2125, right=OCCUPIED), rate=0.0)]  # reached: -0.25 -> 0
  res = audit_frames(frames, TEST_CFG)
  assert [v['frame'] for v in res['violations'] if v['rule'] == 'rate_change_above_limit'] == [2]
  assert res['result'] == 'fail'


def test_audit_first_sample_gap_and_mismatch_are_listed_not_dropped(approved_policies):
  a = [af(0.0, STANDBY, 0.0, avoid_side=None), af(0.05, STANDBY, 0.0, avoid_side=None)]
  res = audit_frames(a, TEST_CFG)
  assert res['result'] == 'not_evaluable' and res['non_differentiable'][0]['why'] == 'no_previous_rate'
  b = [dict(f, rate=0.0) for f in a] + [dict(af(1.0, STANDBY, 0.0, avoid_side=None), rate=0.0)]
  res = audit_frames(b, TEST_CFG)
  assert [d['why'] for d in res['non_differentiable']] == ['time_gap'] and res['result'] == 'not_evaluable'
  c = [dict(a[0], rate=0.0), dict(a[1], rate=0.3)]
  assert 'reported_rate_mismatch' in rules(audit_frames(c, TEST_CFG))
  d = [dict(a[0], rate=0.0), dict(a[1], rate=0.0, consumed=0.01)]
  assert 'consumed_differs_from_applied' in rules(audit_frames(d, TEST_CFG))


def test_audit_names_final_command_signals_as_unverified():
  res = audit_frames([], TEST_CFG)
  assert res['final_command_signals_unverified'] and res['result'] == 'not_evaluable'


# ---------------------------------------------------------------- hold 5: approval and re-entry


def test_mode_exit_and_reentry_start_from_zero_with_new_permission(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, _ = avoid_then(ctrl, LEFT, n=40)
  assert outs[-1].applied != 0.0
  t0 = 40 * DT
  o = ctrl.mode_inactive(t0)
  assert o.applied == 0.0 and o.rate == 0.0 and not o.permitted
  assert 'mode_exit_with_residual_unverified' in o.reasons
  for k in range(1, 10):  # several inactive frames
    o = ctrl.mode_inactive(t0 + k * DT)
    assert o.applied == 0.0 and 'mode_exit_with_residual_unverified' not in o.reasons
  assert (ctrl.applied, ctrl.rate, ctrl.motion_sign, ctrl.avoid_side, ctrl.confirm, ctrl.state) == \
         (0.0, 0.0, 0.0, None, 0, STANDBY)
  assert ctrl.revoked_t == t0  # re-entry wait survives the repeated resets
  times = [t0 + (10 + i) * DT for i in range(60)]
  outs2 = [ctrl.update(inp(t, shift, left, right)) for t in times]
  assert outs2[0].applied == 0.0 and not outs2[0].permitted
  first = next(i for i, o in enumerate(outs2) if o.permitted)
  assert all(o.applied == 0.0 for o in outs2[:first])
  # the first permitted frame grows from 0 inside the continuity limit (rate 0 -> <= d)
  assert abs(outs2[first].applied) <= TEST_CFG.max_rate_change_mps2 * DT * DT + 1e-12
  assert times[first] - t0 >= TEST_CFG.reentry_wait_s - 1e-9


def test_repeated_mode_toggles_never_carry_offset(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right = mirror(LEFT)
  t = 0.0
  for _cycle in range(4):
    outs = []
    for _ in range(40):
      outs.append(ctrl.update(inp(t, shift, left, right)))
      t += DT
    assert outs[0].applied == 0.0
    assert ctrl.mode_inactive(t).applied == 0.0
    t += 3 * DT


def test_planner_mode_toggle_drops_residual_from_published_path(approved_policies):
  planner = make_planner()
  planner.lane_avoid = LaneAvoidController(TEST_CFG)
  shift, left, right = mirror(LEFT)
  n = [0]

  def fake_inputs(sm, carrot, md, model_active):
    n[0] += 1
    return inp(1.0 + n[0] * DT, shift, left, right)
  planner.lane_avoid_inputs = fake_inputs
  lane = [True]
  planner.LP.get_d_path = lambda cs, speed, t, path, curve_speed: (path, lane[0])
  planner.lane_avoid_clock = lambda: 1.0 + n[0] * DT
  for _ in range(40):
    planner.update(planner_inputs(), SimpleNamespace(atc_active=False))
  assert planner.lane_avoid_out.applied != 0.0
  lane[0] = False
  for _ in range(5):
    sm = planner_inputs()
    planner.update(sm, SimpleNamespace(atc_active=False))
    n[0] += 1
    assert planner.lane_avoid_out.applied == 0.0 and planner.lane_avoid.applied == 0.0
    assert np.array_equal(planner.path_xyz[:, 1], np.asarray(sm['modelV2'].position.y) + 0.01)
  lane[0] = True
  sm = planner_inputs()
  planner.update(sm, SimpleNamespace(atc_active=False))
  assert planner.lane_avoid_out.applied == 0.0 and not planner.lane_avoid_out.permitted
  assert np.array_equal(planner.path_xyz[:, 1], np.asarray(sm['modelV2'].position.y) + 0.01)


def test_config_disable_and_reenable_revalidates_and_starts_from_zero(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, _ = avoid_then(ctrl, LEFT, n=40)
  ctrl.cfg = cfg_with(enabled=False)
  o = ctrl.update(inp(40 * DT, shift, left, right))
  assert not ctrl.active and o.state == DISABLED and o.applied == 0.0 and not o.permitted
  ctrl.cfg = TEST_CFG
  times = [(41 + i) * DT for i in range(60)]
  outs2 = [ctrl.update(inp(t, shift, left, right)) for t in times]
  assert outs2[0].applied == 0.0 and not outs2[0].permitted
  # the residual dropped at the disable is still reported after the re-enable
  assert 'approval_change_with_residual_unverified' in outs2[0].reasons
  first = next(i for i, o in enumerate(outs2) if o.permitted)
  assert all(o.applied == 0.0 for o in outs2[:first])
  assert abs(outs2[first].applied) <= TEST_CFG.max_rate_change_mps2 * DT * DT + 1e-12
  assert times[first] - 39 * DT >= TEST_CFG.reentry_wait_s - 1e-9


def test_policy_withdrawn_at_runtime_deactivates(approved_policies, monkeypatch):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, _, _ = avoid_then(ctrl, LEFT, n=40)
  monkeypatch.setattr(la, 'APPROVED_CONSTRAINT_CONFLICT_POLICIES', ())
  o = ctrl.update(inp(40 * DT, shift, left, right))
  assert not ctrl.active and o.state == DISABLED and o.applied == 0.0
  assert 'constraint_conflict_policy_unapproved' in o.blockers
  monkeypatch.setattr(la, 'APPROVED_CONSTRAINT_CONFLICT_POLICIES', (TEST_POLICY,))
  o = ctrl.update(inp(41 * DT, shift, left, right))
  assert ctrl.active and o.applied == 0.0 and not o.permitted


def test_time_gap_recovery_does_not_reuse_old_permission(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, _ = avoid_then(ctrl, LEFT, n=40)
  before = outs[-1].applied
  t = 39 * DT + 1.0  # gap
  outs2 = [ctrl.update(inp(t + i * DT, shift, left, right)) for i in range(30)]
  assert 'time_gap' in outs2[0].reasons
  assert not any(o.permitted for o in outs2)  # residual must return to 0 before any new permission
  assert all(abs(o.applied) <= abs(before) for o in outs2)


def test_numbers_filled_but_policies_unapproved_stay_disabled():
  planner = make_planner()
  planner.lane_avoid = LaneAvoidController(TEST_CFG)  # production tuples empty
  for _ in range(10):
    sm = planner_inputs()
    planner.update(sm, SimpleNamespace(atc_active=False))
    assert planner.lane_avoid_out is None
    assert np.array_equal(planner.path_xyz[:, 1], np.asarray(sm['modelV2'].position.y) + 0.01)
