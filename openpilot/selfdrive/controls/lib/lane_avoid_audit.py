"""Frame-level strict safety audit for lane_avoid traces (prompt v3 §3, §6.2-5/5-1).

Every frame is audited, including RETURN and RETURN_RISK frames; a residual offset
tag never exempts a frame. One violation makes the result 'fail'. Without an
approved configuration, or with frames missing required fields, the result is
'not_evaluable' (never a pass). There is no 'pass'/'improved' outcome here:
improvement needs the separately approved replay evaluation.
"""
import math

from openpilot.selfdrive.controls.lib.lane_avoid import CLEAR, LEFT, RIGHT, other_side

REQUIRED_FIELDS = ('t', 'state', 'permitted', 'target', 'applied', 'avoid_side', 'side_state', 'edge_state')


def _side_of(value):
  if value > 0.0:
    return RIGHT
  if value < 0.0:
    return LEFT
  return None


def audit_frames(frames, cfg):
  violations = []
  missing = []
  counts = {}
  prev = None
  for i, f in enumerate(frames):
    if not all(k in f for k in REQUIRED_FIELDS) or not all(
        isinstance(v, (int, float)) and math.isfinite(v) for v in (f.get('t'), f.get('applied'), f.get('target'))):
      missing.append(i)
      prev = None
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

    if prev is not None:
      dt = f['t'] - prev['t']
      a0 = prev['applied']
      if dt <= 0.0:
        if applied != a0:
          fail('offset_changed_without_time', dt=dt, before=a0, after=applied)
      else:
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
  elif blockers or missing or not frames:
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
    'counts_by_state_side': counts,
    'blockers': blockers,
  }
