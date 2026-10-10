"""Frame-level strict safety audit for lane_avoid traces (prompt v3 §3, §6.2-5/5-1).

Every frame is audited, including RETURN and RETURN_RISK frames; a residual offset
tag never exempts a frame. One violation makes the result 'fail'. Without an
approved configuration, or with frames missing required fields, the result is
'not_evaluable' (never a pass). There is no 'pass'/'improved' outcome here:
improvement needs the separately approved replay evaluation.

v3.1: the offset rate is the actual difference of consecutive applied values over the
actual interval, and its change is checked on every frame (target reached, zero,
revocation, mode change, edge shrink included). An optional 'rate' field (the
controller's own value) must match that difference and supplies the initial rate of
the first interval only. Intervals that cannot be differentiated (first sample without
'rate', time gap, invalid neighbour) are listed, never dropped, and keep the result
not_evaluable. 'max_rate_change_mps2' is the second time derivative of the offset; it
is NOT a steering-command, curvature or lateral-acceleration jerk, which this trace
does not contain (FINAL_COMMAND_SIGNALS_UNVERIFIED).
"""
import math

from openpilot.selfdrive.controls.lib.lane_avoid import CLEAR, LEFT, RIGHT, other_side

REQUIRED_FIELDS = ('t', 'state', 'permitted', 'target', 'applied', 'avoid_side', 'side_state', 'edge_state')
RATE_TOL = 1e-9

# Consumed downstream of the offset; each needs its own definition, unit, clock, consumer
# and approved limit before its rate/rate-change can be judged. Not evaluated here.
FINAL_COMMAND_SIGNALS_UNVERIFIED = (
  'lateralPlan.curvatures (1/m, plannerd, per model frame; lateral_planner.publish)',
  'lateralPlan.curvatureRates (1/(m s), plannerd)',
  'controlsd desired_curvature after clip_curvature (1/m, DT_CTRL; controlsd.py:291)',
  'carControl.actuators.curvature (1/m, DT_CTRL; controlsd.py:293)',
  'lateral acceleration v^2 * curvature (m/s^2)',
  'steering angle/torque command (carcontroller, brand-specific)',
)


def _side_of(value):
  if value > 0.0:
    return RIGHT
  if value < 0.0:
    return LEFT
  return None


def _num(v):
  return isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)


def audit_frames(frames, cfg):
  violations = []
  missing = []
  non_diff = []
  counts = {}
  prev = None
  prev_rate = None  # actual rate of the previous interval (or the first frame's 'rate')
  gap = cfg.max_input_gap_s if _num(cfg.max_input_gap_s) else None
  jmax = cfg.max_rate_change_mps2 if _num(cfg.max_rate_change_mps2) else None
  for i, f in enumerate(frames):
    if not all(k in f for k in REQUIRED_FIELDS) or not all(
        isinstance(v, (int, float)) and math.isfinite(v) for v in (f.get('t'), f.get('applied'), f.get('target'))):
      missing.append(i)
      if i > 0:
        non_diff.append({'frame': i, 'why': 'invalid_frame'})
      prev = None
      prev_rate = None
      continue
    key = (f['state'], f['avoid_side'])
    counts[key] = counts.get(key, 0) + 1

    def fail(rule, **raw):
      violations.append({'frame': i, 'rule': rule, 'state': f['state'], 'avoid_side': f['avoid_side'], **raw})

    applied, target = f['applied'], f['target']
    move_side = _side_of(target) or _side_of(applied) or f['avoid_side']
    if f['permitted']:
      if move_side is None:
        fail('permitted_without_direction')
      else:
        if f['side_state'].get(move_side) != CLEAR:
          fail('permitted_avoid_side_not_clear', side=move_side, side_state=f['side_state'].get(move_side))
        if f['edge_state'].get(move_side) != CLEAR:
          fail('permitted_avoid_edge_not_clear', side=move_side, edge_state=f['edge_state'].get(move_side))
    if target != 0.0 and not f['permitted']:
      fail('target_without_permission', target=target)
    if isinstance(cfg.max_offset_m, (int, float)) and abs(applied) > cfg.max_offset_m:
      fail('offset_above_limit', applied=applied, limit=cfg.max_offset_m)
    # v3.1: every frame's applied offset (residual included) must fit the observed edge room
    side_now = _side_of(applied)
    if side_now is not None:
      room = f.get('edge_room')
      if isinstance(room, dict):
        r = room.get(side_now)
        if not _num(r) or abs(applied) > r + 1e-12:
          fail('applied_exceeds_edge_room', side=side_now, applied=applied, room=r)
      elif f['edge_state'].get(side_now) != CLEAR:
        fail('applied_toward_unobserved_edge', side=side_now, applied=applied,
             edge_state=f['edge_state'].get(side_now))
    if 'consumed' in f and not (_num(f['consumed']) and abs(f['consumed'] - applied) <= RATE_TOL):
      fail('consumed_differs_from_applied', applied=applied, consumed=f['consumed'])

    if prev is None:
      # first sample (or after an invalid frame): only the frame's own rate can seed continuity
      prev_rate = f['rate'] if _num(f.get('rate')) else None
    else:
      dt = f['t'] - prev['t']
      a0 = prev['applied']
      if dt <= 0.0:
        if applied != a0:
          fail('offset_changed_without_time', dt=dt, before=a0, after=applied)
        non_diff.append({'frame': i, 'why': 'time_not_increasing', 'dt': dt})
        prev_rate = None
      else:
        r = (applied - a0) / dt
        if _num(f.get('rate')) and abs(f['rate'] - r) > RATE_TOL:
          fail('reported_rate_mismatch', reported=f['rate'], actual=r)
        if gap is not None and dt > gap:
          non_diff.append({'frame': i, 'why': 'time_gap', 'dt': dt})
          r_next = None
        else:
          r_next = r
          if prev_rate is None:
            non_diff.append({'frame': i, 'why': 'no_previous_rate'})
          elif jmax is None:
            if r != prev_rate:
              fail('rate_change_limit_unapproved', rate_before=prev_rate, rate_after=r)
          elif abs(r - prev_rate) > jmax * dt + RATE_TOL:
            fail('rate_change_above_limit', rate_before=prev_rate, rate_after=r, change=abs(r - prev_rate),
                 limit=jmax * dt, dt=dt)
        prev_rate = r_next
        if a0 * applied < 0.0:
          fail('offset_crossed_sides', before=a0, after=applied)
        grew = abs(applied) > abs(a0)
        shrank = abs(applied) < abs(a0)
        side = _side_of(applied) or _side_of(a0)
        if grew:
          if not f['permitted']:
            fail('growth_without_permission', before=a0, after=applied)
          if f['side_state'].get(side) != CLEAR or f['edge_state'].get(side) != CLEAR:
            fail('growth_toward_unclear_side', side=side, side_state=f['side_state'].get(side),
                 edge_state=f['edge_state'].get(side))
          rate = cfg.entry_rate_mps
        else:
          rate = cfg.return_rate_mps
        if shrank:
          ret = other_side(side)
          if f['side_state'].get(ret) != CLEAR or f['edge_state'].get(ret) != CLEAR:
            fail('return_move_without_clear_return_side', side=ret, side_state=f['side_state'].get(ret),
                 edge_state=f['edge_state'].get(ret), before=a0, after=applied)
        if isinstance(rate, (int, float)) and abs(applied - a0) > rate * dt + 1e-12:
          fail('rate_above_limit', rate=abs(applied - a0) / dt, limit=rate)
        elif not isinstance(rate, (int, float)) and (grew or shrank):
          fail('rate_limit_unapproved', before=a0, after=applied)
    prev = f

  blockers = list(cfg.blockers())
  if violations:
    result = 'fail'
  elif blockers or missing or not frames or non_diff:
    result = 'not_evaluable'
  else:
    # still not a pass: return-side proximity, risk-policy and coverage evidence are
    # evaluated by the approved replay tool, not by this trace check
    result = 'no_violation_found'
  return {
    'result': result,
    'violations': violations,
    'frames': len(frames),
    'invalid_frames': missing,
    'non_differentiable': non_diff,
    'counts_by_state_side': counts,
    'blockers': blockers,
    'final_command_signals_unverified': FINAL_COMMAND_SIGNALS_UNVERIFIED,
  }
