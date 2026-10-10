"""lane_avoid v3.2 hold 1-5 tests (lanemode_avoid_prompt_v3_2). Written for review; NOT executed.

All values are synthetic fixtures (see test_lane_avoid.TEST_CFG); none is an approved
setting. Tests that detect a defect are named *_detected; they passing is not a safety
pass. No test fixes "hold" or "instant 0" as the correct response to a conflict.
"""
from types import SimpleNamespace

import numpy as np
import pytest

import openpilot.selfdrive.controls.lib.lane_avoid as la
from openpilot.selfdrive.controls.lib.lane_avoid import (
  CLEAR, LEFT, OCCUPIED, RIGHT, UNKNOWN, LaneAvoidConfig, LaneAvoidController, SideObject,
  road_edge_allowance)
from openpilot.selfdrive.controls.lib.lane_avoid_audit import audit_frames, audit_signal_chain
from openpilot.selfdrive.controls.tests.test_lane_avoid import (  # noqa: F401  (fixture import)
  DT, EDGE_X, N, TEST_CFG, TEST_POLICY, X, approved_policies, avoid_then, edges, inp, mirror, occ, road_on, run,
  side)
from openpilot.selfdrive.controls.tests.test_lane_model_speed_planner import inputs as planner_inputs
from openpilot.selfdrive.controls.tests.test_lane_model_speed_planner import make_planner


def rules(res):
  return {v['rule'] for v in res['violations']}


def shrink_left(room):
  # left edge placed so that the observed room is `room` (half width 0.95 + margin 0.5)
  return edges(left_y=-(room + 0.95 + 0.5))


# ---------------------------------------------------------------- hold 1: conflict output is invalid


@pytest.mark.parametrize('return_clear', [False, True])
def test_conflict_output_is_invalid_with_reason_not_a_command(approved_policies, return_clear):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, frames = avoid_then(ctrl, LEFT, n=40)
  seq = dict(shift=0.0) if return_clear else dict(shift=shift, left=left, right=right)
  outs2, frames2 = run(ctrl, [dict(seq, road=edges(left_y=-1.2))] * 5, t0=40 * DT)  # room < 0
  o = outs2[0]
  assert o.state == la.CONFLICT and not o.valid and not o.permitted and o.target == 0.0
  assert o.invalid_reasons[0] == 'constraint_conflict' and any(r.startswith('conflict_') for r in o.invalid_reasons)
  assert o.conflict_action == 'test_conflict_action'  # handler result is handed over, not executed
  assert 'output_invalid' in rules(audit_frames(frames + frames2, TEST_CFG))
  # every valid frame before the conflict was consumable
  assert all(x.valid for x in outs)


def test_conflict_with_zero_room_direction_reversal_and_unobserved_edge(approved_policies):
  for road in (shrink_left(0.0), edges(left_y=1.0), edges(std=5.0)):
    ctrl = LaneAvoidController(TEST_CFG)
    avoid_then(ctrl, LEFT, n=40)
    shift, left, right = mirror(LEFT)
    o = ctrl.update(inp(40 * DT, shift, left, right, road=road))
    assert not o.valid and o.state == la.CONFLICT


def test_feasible_shrink_keeps_every_published_value_inside_all_constraints(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  _, _, _, outs, frames = avoid_then(ctrl, LEFT, n=40)
  assert abs(outs[-1].applied) == pytest.approx(0.3) and outs[-1].rate == 0.0
  outs2, frames2 = run(ctrl, [dict(shift=0.0, road=shrink_left(0.299))] * 60, t0=40 * DT)
  assert all(o.valid and o.invalid_reasons == () for o in outs2)
  assert all(abs(o.applied) <= o.edge_room[LEFT] + 1e-12 for o in outs2 if o.applied != 0.0)
  res = audit_frames(frames + frames2, TEST_CFG)
  assert res['violations'] == []


def test_final_check_rejects_out_of_room_value_on_time_fault_frame(approved_policies):
  # dt unknown (repeated time) and the room shrank: no motion is possible and the held
  # value exceeds the room; the final re-check must mark it invalid
  ctrl = LaneAvoidController(TEST_CFG)
  _, _, _, outs, _ = avoid_then(ctrl, LEFT, n=40)
  assert outs[-1].rate == 0.0
  shift, left, right = mirror(LEFT)
  o = ctrl.update(inp(39 * DT, shift, left, right, road=shrink_left(0.1)))  # not increasing
  assert 'time_not_increasing' in o.reasons
  assert not o.valid and 'final_check_edge_room_or_max_offset' in o.invalid_reasons


def test_violations_helper_names_each_constraint(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  ctrl.applied, ctrl.rate, ctrl.motion_sign = -0.3, 0.0, -1.0
  cap = {LEFT: 0.5, RIGHT: 0.5}
  assert ctrl._violations(-0.3, DT, False, False, cap) == []
  assert 'no_growth_without_permission' in ctrl._violations(-0.31, DT, False, False, cap)
  assert 'no_return_without_clear_return_side' in ctrl._violations(-0.295, DT, False, False, cap)
  assert 'continuity' in ctrl._violations(-0.2, DT, True, True, cap)
  assert 'edge_room_or_max_offset' in ctrl._violations(-0.3, DT, False, False, {LEFT: 0.2, RIGHT: 0.5})
  assert 'no_side_crossing' in ctrl._violations(0.01, DT, True, True, cap)


def test_planner_refuses_to_consume_invalid_output(approved_policies):
  planner = make_planner()
  planner.lane_avoid = LaneAvoidController(TEST_CFG)
  shift, left, right = mirror(LEFT)
  n = [0]

  def fake_inputs(sm, carrot, md, model_active):
    n[0] += 1
    return inp(1.0 + n[0] * DT, shift, left, right, road=edges(left_y=-1.2) if n[0] > 40 else None)
  planner.lane_avoid_inputs = fake_inputs
  for _ in range(41):
    sm = planner_inputs()
    planner.update(sm, SimpleNamespace(atc_active=False))
  o = planner.lane_avoid_out
  assert not o.valid and o.applied != 0.0
  # the invalid value is not consumed as an avoidance command. What is commanded
  # instead is undecided (human decision); this test does not assert it is safe.
  assert not np.array_equal(planner.path_xyz[:, 1], np.asarray(sm['modelV2'].position.y) + 0.01 + o.applied)


def test_production_conflict_policy_cannot_activate():
  b = LaneAvoidConfig().blockers()
  assert 'constraint_conflict_policy_unapproved' in b and 'constraint_conflict_policy_unimplemented' in b
  assert la.POLICY_IMPLEMENTATIONS['constraint_conflict'] == {}


# ---------------------------------------------------------------- hold 2: body rear on road edges


DENSE = np.linspace(-10.0, 110.0, 481)  # 0.25 m knots: -1 .. 0 m is sampled


def rear_narrow(direction, y_rear):
  return road_on(direction, DENSE, np.where(DENSE < 0.0, y_rear, 5.0))


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_rear_only_edge_intrusion_is_detected(direction):
  # front, middle and body front are clear; only -1 .. 0 m (body rear) is narrow
  road = rear_narrow(direction, 1.2)
  assert road_edge_allowance(X, np.zeros(N), *road[direction], direction, TEST_CFG)[0] == OCCUPIED


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_rear_unobserved_edge_is_unknown_not_extended(direction):
  for ex in (np.linspace(0.0, 110.0, N),      # v3.1 fixture: starts at the reference point
             np.linspace(-0.5, 110.0, N)):    # covers part of the body rear only
    road = road_on(direction, ex, np.full(N, 5.0))
    assert road_edge_allowance(X, np.zeros(N), *road[direction], direction, TEST_CFG)[0] == UNKNOWN
  full = road_on(direction, EDGE_X, np.full(N, 5.0))
  state, room = road_edge_allowance(X, np.zeros(N), *full[direction], direction, TEST_CFG)
  assert state == CLEAR and room == pytest.approx(5.0 - 0.95 - 0.5)  # normal case is not always blocked


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_path_starting_ahead_of_ego_is_unknown(direction):
  ahead = np.linspace(2.0, 100.0, N)
  full = road_on(direction, EDGE_X, np.full(N, 5.0))
  assert road_edge_allowance(ahead, np.zeros(N), *full[direction], direction, TEST_CFG)[0] == UNKNOWN
  assert la.body_footprint(ahead, np.zeros(N), TEST_CFG) is None
  assert la.required_space(ahead, np.zeros(N), [0.0], direction, TEST_CFG) is None


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_mid_path_narrowing_still_detected_with_rear_observed(direction):
  y = np.where((EDGE_X > 40.0) & (EDGE_X < 45.0), 1.2, 5.0)
  road = road_on(direction, EDGE_X, y)
  assert road_edge_allowance(X, np.zeros(N), *road[direction], direction, TEST_CFG)[0] == OCCUPIED


@pytest.mark.parametrize('sign', [1.0, -1.0])
def test_heading_rotated_corner_intrusion_detected(sign):
  # strongly curving synthetic path; the edge follows the curve at distance D. The
  # centreline band alone has room, the rotated body corners do not.
  direction = RIGHT if sign > 0 else LEFT
  px = np.linspace(0.0, 20.0, N)
  py = sign * -0.02 * px ** 2
  ex = np.linspace(-10.0, 30.0, 401)
  D = 1.8
  ey = sign * (-0.02 * ex ** 2 + D)
  fp = la.body_footprint(px, py, TEST_CFG)
  cx, cy, lx, ly = fp
  knots = np.union1d(lx, ex[(ex >= lx[0]) & (ex <= lx[-1])])
  band_room = float(np.min(sign * (np.interp(knots, ex, ey) - np.interp(knots, lx, ly)))) - 0.95 - 0.5
  assert band_room > 0.0
  state, _ = road_edge_allowance(px, py, ex, ey, 0.1, 0.0, direction, TEST_CFG)
  assert state == OCCUPIED


def test_side_space_and_edges_use_same_footprint_including_rear():
  sp = la.required_space(X, np.zeros(N), [0.0], LEFT, TEST_CFG)
  cx, _, lx, _ = la.body_footprint(X, np.zeros(N), TEST_CFG)
  assert sp.x_min <= min(float(np.min(cx)), float(lx[0])) and sp.x_min == -1.0
  assert sp.x_max >= max(float(np.max(cx)), float(lx[-1]))


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_rear_intrusion_blocks_avoid_and_return_in_controller(approved_policies, direction):
  shift, left, right = mirror(direction)
  road = rear_narrow(direction, 1.2)
  outs, _ = run(LaneAvoidController(TEST_CFG), [dict(shift=shift, left=left, right=right, road=road)] * 30)
  assert not any(o.permitted for o in outs) and outs[-1].edge_state[direction] == OCCUPIED
  # return side: rear narrowing on the return side keeps the return blocked
  ctrl = LaneAvoidController(TEST_CFG)
  _, _, _, outs, _ = avoid_then(ctrl, direction, n=40)
  ret = la.other_side(direction)
  road_r = rear_narrow(ret, 0.5)
  outs2, _ = run(ctrl, [dict(shift=0.0, road=road_r)] * 10, t0=40 * DT)
  assert all(abs(o.applied) >= abs(outs[-1].applied) - 1e-12 for o in outs2)


# ---------------------------------------------------------------- hold 3: policy name vs implementation


def test_approved_name_without_implementation_blocks(monkeypatch, approved_policies):
  assert LaneAvoidController(TEST_CFG).active
  for kind, fld, _ in la.POLICY_FIELDS:
    monkeypatch.delitem(la.POLICY_IMPLEMENTATIONS[kind], TEST_POLICY)
    b = TEST_CFG.blockers()
    assert f'{fld}_unimplemented' in b and f'{fld}_unapproved' not in b
    assert not LaneAvoidController(TEST_CFG).active
    monkeypatch.setitem(la.POLICY_IMPLEMENTATIONS[kind], TEST_POLICY, lambda *a: None)


def test_production_registry_has_no_operational_policy():
  for kind in ('return_risk', 'center_invalid', 'occupancy_prediction', 'constraint_conflict'):
    assert la.POLICY_IMPLEMENTATIONS[kind] == {}
  # the built-in body model is implemented but not approved
  assert la.BODY_MODEL_RIGID_HEADING not in la.APPROVED_BODY_GEOMETRY_MODELS
  b = LaneAvoidConfig(body_geometry_model=la.BODY_MODEL_RIGID_HEADING).blockers()
  assert 'body_geometry_model_unapproved' in b and 'body_geometry_model_unimplemented' not in b


def obj(side_, x, lat, vx=0.0, vlat=0.0, t=0.0, length=4.5, width=1.8):
  return SideObject(side=side_, x=x, lat=lat, vx=vx, vlat=vlat, length=length, width=width, t=t)


def run_objects(ctrl, direction, objs_of, n=30, t0=0.0):
  shift, left, right = mirror(direction)
  outs = []
  for i in range(n):
    t = t0 + i * DT
    o = ctrl.update(inp(t, shift, left, right, side_objects={direction: objs_of(t), la.other_side(direction): []}))
    outs.append(o)
  return outs


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_object_moving_into_space_blocks_and_static_outside_permits(approved_policies, direction):
  entering = run_objects(LaneAvoidController(TEST_CFG), direction, lambda t: [obj(direction, 20.0, 4.0, vlat=-1.5, t=t)])
  assert not any(o.permitted for o in entering) and entering[-1].prediction_state[direction] == OCCUPIED
  rear = run_objects(LaneAvoidController(TEST_CFG), direction, lambda t: [obj(direction, -25.0, 1.0, vx=12.0, t=t)])
  assert not any(o.permitted for o in rear)
  static = run_objects(LaneAvoidController(TEST_CFG), direction, lambda t: [obj(direction, 20.0, 4.5, t=t)])
  assert static[-1].prediction_state[direction] == CLEAR and static[-1].permitted  # prediction reaches permission


@pytest.mark.parametrize('case,reason', [
  ('stale', 'prediction_input_stale'), ('future', 'prediction_input_stale'), ('mismatch', 'prediction_input_mismatch'),
  ('nan', 'prediction_input_invalid'), ('missing', 'prediction_input_missing'),
])
def test_prediction_input_faults_are_unknown(approved_policies, case, reason):
  direction = LEFT
  objs = {
    'stale': lambda t: [obj(LEFT, 20.0, 4.5, t=t - 1.0)],
    'future': lambda t: [obj(LEFT, 20.0, 4.5, t=t + 0.5)],
    'mismatch': lambda t: [obj(RIGHT, 20.0, 4.5, t=t)],
    'nan': lambda t: [obj(LEFT, float('nan'), 4.5, t=t)],
    'missing': lambda t: None,
  }[case]
  outs = run_objects(LaneAvoidController(TEST_CFG), direction, objs)
  assert not any(o.permitted for o in outs)
  assert outs[-1].prediction_state[direction] == UNKNOWN and f'{direction}_{reason}' in outs[-1].reasons


@pytest.mark.parametrize('impl,reason', [
  (lambda *a: 'free', 'prediction_bad_return'), (lambda *a: None, 'prediction_bad_return'),
  (lambda *a: 1 / 0, 'prediction_failed'),
])
def test_bad_predictor_results_are_unknown(approved_policies, monkeypatch, impl, reason):
  monkeypatch.setitem(la.POLICY_IMPLEMENTATIONS['occupancy_prediction'], TEST_POLICY, impl)
  outs = run_objects(LaneAvoidController(TEST_CFG), LEFT, lambda t: [])
  assert not any(o.permitted for o in outs) and f'left_{reason}' in outs[-1].reasons


def test_new_object_predicted_into_return_space_blocks_return(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  _, _, _, outs, _ = avoid_then(ctrl, LEFT, n=40)
  before = outs[-1].applied
  for i in range(10):
    t = (40 + i) * DT
    o = ctrl.update(inp(t, 0.0, side_objects={LEFT: [], RIGHT: [obj(RIGHT, 10.0, 3.5, vlat=-1.0, t=t)]}))
    assert o.prediction_state[RIGHT] == OCCUPIED and abs(o.applied) >= abs(before) - 1e-12


def test_risk_handler_proposal_is_checked_against_constraints(approved_policies, monkeypatch):
  monkeypatch.setitem(la.POLICY_IMPLEMENTATIONS['return_risk'], TEST_POLICY, lambda ctx: 0.0)  # jump to 0
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, _, _ = avoid_then(ctrl, LEFT, n=40)
  o = ctrl.update(inp(40 * DT, 0.0, left=side(), right=side(radar=occ())))  # obstacle still on return side
  assert not o.valid and 'return_risk_policy_result_rejected' in o.invalid_reasons
  monkeypatch.setitem(la.POLICY_IMPLEMENTATIONS['return_risk'], TEST_POLICY, lambda ctx: 1 / 0)
  ctrl2 = LaneAvoidController(TEST_CFG)
  avoid_then(ctrl2, LEFT, n=40)
  o2 = ctrl2.update(inp(40 * DT, 0.0, left=side(), right=side(radar=occ())))
  assert not o2.valid and 'return_risk_policy_failed' in o2.invalid_reasons


# ---------------------------------------------------------------- hold 4: residual / final continuity


def test_mode_exit_reports_dropped_residual_and_chain_step_is_detected(approved_policies):
  ctrl = LaneAvoidController(TEST_CFG)
  _, _, _, outs, _ = avoid_then(ctrl, LEFT, n=12)  # non-zero applied and rate
  a, r = ctrl.applied, ctrl.rate
  assert a != 0.0 and r != 0.0
  ex = ctrl.mode_inactive(12 * DT)
  assert ex.residual_dropped == (a, r) and 'mode_exit_with_residual_unverified' in ex.reasons
  later = [ctrl.mode_inactive((12 + k) * DT) for k in range(1, 4)]
  assert all(x.residual_dropped is None for x in later)
  # consumed offset across the exit: the reset drops the residual in one frame
  recs = [{'t': i * DT, 'consumed': o.applied} for i, o in enumerate(outs)] + \
         [{'t': (12 + k) * DT, 'consumed': 0.0} for k in range(4)]
  lim = {'consumed': (TEST_CFG.entry_rate_mps, TEST_CFG.max_rate_change_mps2)}
  res = audit_signal_chain(recs, lim)['consumed']
  assert res['result'] == 'fail' and any(v['frame'] == 12 for v in res['violations'])
  # without approved limits the chain is not evaluable, never passed
  assert audit_signal_chain(recs, {'consumed': None})['consumed']['result'] == 'not_evaluable'


def test_approval_change_reports_dropped_residual_once(approved_policies, monkeypatch):
  ctrl = LaneAvoidController(TEST_CFG)
  avoid_then(ctrl, LEFT, n=12)
  a, r = ctrl.applied, ctrl.rate
  monkeypatch.setattr(la, 'APPROVED_RETURN_RISK_POLICIES', ())
  shift, left, right = mirror(LEFT)
  o = ctrl.update(inp(12 * DT, shift, left, right))
  assert o.residual_dropped == (a, r) and o.applied == 0.0
  o2 = ctrl.update(inp(13 * DT, shift, left, right))
  assert o2.residual_dropped is None


@pytest.mark.parametrize('cause', ['model_invalid', 'driver', 'gap'])
def test_reentry_first_frame_after_fault_is_zero_and_unpermitted(approved_policies, cause):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, _ = avoid_then(ctrl, LEFT, n=12)
  kw = {'model_invalid': dict(model_valid=False), 'driver': dict(driver_steering=True), 'gap': {}}[cause]
  t = 12 * DT if cause != 'gap' else 12 * DT + 1.0
  o = ctrl.update(inp(t, shift, left, right, **kw))
  assert not o.permitted
  for k in range(4):
    ctrl.mode_inactive(t + (k + 1) * DT)
  o2 = ctrl.update(inp(t + 5 * DT, shift, left, right))
  assert o2.applied == 0.0 and not o2.permitted and 'reentry_wait' in o2.reasons


def test_signal_chain_checks_every_signal_and_lists_gaps():
  recs = [{'t': 0.0, 'a': 0.0, 'b': 0.0}, {'t': 0.05, 'a': 0.0, 'b': 0.5}, {'t': 0.05, 'a': 0.0, 'b': 0.5},
          {'t': 0.10, 'a': float('nan'), 'b': 0.5}]
  res = audit_signal_chain(recs, {'a': (1.0, 1.0), 'b': (1.0, 1.0)})
  assert res['b']['result'] == 'fail'  # 10/s step
  assert res['a']['result'] == 'not_evaluable' and res['a']['non_differentiable']


# ---------------------------------------------------------------- hold 5: in-progress cancel


@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_in_progress_cancel_conflict_is_detected_and_invalid(approved_policies, direction):
  # defect-detection test: growing (rate 0.3) when the avoid side becomes occupied and the
  # obstacle remains -> no offset satisfies no-growth, no-return and continuity. The
  # output is invalid; this does NOT resolve the kept v3 assert in
  # test_new_avoid_side_detection_revokes_and_never_grows (human decision, REVIEW.md).
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, _, frames = avoid_then(ctrl, direction, n=12)
  blocked = side(bsd=occ())
  l2, r2 = (blocked, right) if direction == LEFT else (left, blocked)
  outs2, frames2 = run(ctrl, [dict(shift=shift, left=l2, right=r2)] * 3, t0=12 * DT)
  assert outs2[0].state == la.CONFLICT and not outs2[0].valid
  assert 'output_invalid' in rules(audit_frames(frames + frames2, TEST_CFG))


def test_cancel_after_hold_with_return_side_clear_is_valid_on_every_frame(approved_policies):
  # acceptance (held, rate 0): avoid side becomes occupied, return side clear -> a feasible
  # return exists on every frame, uniform and non-uniform intervals. The GROWING case is
  # infeasible by construction (no growth + continuity, see the test above): human decision.
  for steps in ([DT], [0.04, 0.06, 0.05]):
    ctrl = LaneAvoidController(TEST_CFG)
    shift, left, right = mirror(LEFT)
    t, outs, frames = 0.0, [], []
    for i in range(100):
      if i < 40:
        o = ctrl.update(inp(t, shift, left, right))
      else:
        o = ctrl.update(inp(t, 0.0, left=side(bsd=occ()), right=side()))
      outs.append(o)
      frames.append({'t': t, 'state': o.state, 'permitted': o.permitted, 'target': o.target, 'applied': o.applied,
                     'avoid_side': o.avoid_side, 'side_state': dict(o.side_state), 'edge_state': dict(o.edge_state),
                     'rate': o.rate, 'edge_room': dict(o.edge_room), 'valid': o.valid})
      t += steps[i % len(steps)]
    assert outs[39].rate == 0.0 and outs[39].applied != 0.0
    assert all(o.valid for o in outs)
    assert outs[-1].applied == 0.0
    assert audit_frames(frames, TEST_CFG)['violations'] == []


def test_nonfinite_edge_alone_flips_full_range_observation():
  # discriminating companion of test_road_edge_not_extrapolated_and_shape_checked:333
  good = np.full(N, -5.0)
  assert road_edge_allowance(X, np.zeros(N), EDGE_X, good, 0.1, 0.0, LEFT, TEST_CFG)[0] == CLEAR
  bad = good.copy()
  bad[N // 2] = np.nan
  assert road_edge_allowance(X, np.zeros(N), EDGE_X, bad, 0.1, 0.0, LEFT, TEST_CFG)[0] == UNKNOWN
  inf = good.copy()
  inf[0] = -np.inf
  assert road_edge_allowance(X, np.zeros(N), EDGE_X, inf, 0.1, 0.0, LEFT, TEST_CFG)[0] == UNKNOWN
