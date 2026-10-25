"""K9 lane menu (experimental, NOT vehicle-validated): one switch + two values for every K9 lane / steering-path experiment.

보노보노 2026-10-25 20:17 "실험중 차선관련항목 통합좀해서 정리해라, 너무많다". Before this the 실험중 menu had four lane switches
(K9LaneFix, K9LRView, K9OneLaneHold, K9LaneCenter); K9LRView and K9OneLaneHold were already coupled (modeld publishes the
near lines from the rule-free filter while the one-lane hold is on) and the one-lane / both-lane holds are the two halves
of the same laneless lane-centre hold. They are one switch now:

K9LaneAssist (int, default 0):
  0 (or anything else): off - modeld, lane_planner_2 and controlsd behave exactly as with all four old switches at 0.
  1: on = everything the four old switches did at 1, together:
     - modeld: lane-line tracker (k9_lr_view; near lines from the rule-free filter, as the old K9OneLaneHold 1 made it);
     - lane mode only (UseLaneLineSpeed > 0): lane-path entry / hand-over shaping (k9_lane_fix "F1");
     - laneless: lane-centre hold, both lines (k9_lane_center, from 1 km/h) and one line (k9_one_lane_hold, from 29 km/h).
Values (read live, every 0.1 s; they only matter while K9LaneAssist is 1):
  K9LaneCenterGain (int %, 0..200, default 100): strength of both lane-centre holds (100 = as validated, 0 = no push).
  PathOffset (existing carrot param, cm, + = right): the lane-centre holds keep the car this far right of the lane centre,
    clipped to +-OFFSET_MAX_CM (lane mode keeps using the full PathOffset range for its path, as before).

Migration (manager_init, once, marker K9LaneVer): any old switch on (K9LaneFix 1, K9LRView 1/2/3, K9OneLaneHold 1,
K9LaneCenter 1) -> K9LaneAssist 1; all old switches 0 / absent -> K9LaneAssist untouched (0). The old keys stay defined (no
effect) so a downgrade still reads them.
"""

K9_LANE_PARAM = "K9LaneAssist"
K9_LANE_GAIN_PARAM = "K9LaneCenterGain"
K9_LANE_OFFSET_PARAM = "PathOffset"
K9_LANE_VER_KEY = "K9LaneVer"
K9_LANE_VER = 1
GAIN_MIN, GAIN_MAX, GAIN_DEFAULT = 0, 200, 100
OFFSET_MAX_CM = 30
# old key -> stored values that meant "on" (K9LRView 1, 2, 3: numbers of its earlier trial menu, all mean 1)
OLD_ON = {"K9LaneFix": (1,), "K9LRView": (1, 2, 3), "K9OneLaneHold": (1,), "K9LaneCenter": (1,)}


def k9_lane_on(params) -> bool:
  """K9LaneAssist: 1 = on, anything else / unreadable = off."""
  try:
    return int(params.get_int(K9_LANE_PARAM)) == 1
  except Exception:
    return False


def k9_lane_gain(params) -> float:
  """K9LaneCenterGain % -> factor (0.0 .. 2.0); unreadable -> 1.0. Integer % / 100 is exact for 100 (1.0)."""
  try:
    g = int(params.get_int(K9_LANE_GAIN_PARAM))
  except Exception:
    return 1.0
  return min(max(g, GAIN_MIN), GAIN_MAX) / 100.0


def k9_lane_offset_m(params) -> float:
  """PathOffset cm (+ right) clipped to +-OFFSET_MAX_CM -> m; unreadable -> 0.0."""
  try:
    o = int(params.get_int(K9_LANE_OFFSET_PARAM))
  except Exception:
    return 0.0
  return min(max(o, -OFFSET_MAX_CM), OFFSET_MAX_CM) / 100.0


def migrate_k9_lane(params) -> bool:
  """Once (marker K9LaneVer): any old lane switch on -> K9LaneAssist 1. Returns True if K9LaneAssist was written.
  Never raises (manager start must not fail on a bad stored value)."""
  try:
    if params.get(K9_LANE_VER_KEY) == K9_LANE_VER:
      return False
  except Exception:
    pass
  any_on = False
  for key, on in OLD_ON.items():
    try:
      val = params.get(key)
    except Exception:
      val = None
    if isinstance(val, int) and not isinstance(val, bool) and val in on:
      any_on = True
  changed = False
  if any_on:
    try:
      cur = params.get(K9_LANE_PARAM)
    except Exception:
      cur = None
    if cur != 1:
      try:
        params.put_int(K9_LANE_PARAM, 1)
        changed = True
      except Exception:
        return False   # not written: no marker either, the next start tries again
  try:
    params.put_int(K9_LANE_VER_KEY, K9_LANE_VER)
  except Exception:
    pass
  return changed
