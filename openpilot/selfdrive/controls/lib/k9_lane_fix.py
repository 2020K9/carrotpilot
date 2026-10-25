"""K9LaneFix: numbered lane-mode reference / hand-over profiles (experimental menu, NOT vehicle-validated).

Switch (2026-10-25 menu merge, see k9_lane.py): K9LaneAssist (int, default 0) - the old K9LaneFix menu item is gone.
  K9LaneAssist 0 (or anything else): mode 0 = off - lane_planner_2.py and controlsd.py behave exactly as before.
  K9LaneAssist 1: mode 1 = "F1" (LANEFIX4, replay / closed-loop simulation only):
     lane_planner_2.py - when the lane path is blended into the plan, the reference starts at the model path start and merges
       into the lane path over converge_t s (quintic smootherstep); the merge time is extended for large offsets
       (T * sqrt(|e0| / t_eref), at most t_max s) but only as far as the merge curvature would exceed t_kref 1/m, so at high
       speed it stays converge_t; lane-mode start ramps the correction in over ramp_in_t s.
     controlsd.py - from the first tick after controlsd stops following the lane-mode plan and goes back to the model
       desiredCurvature, the per-tick command change is capped at entry_jerk m/s^3 lateral jerk until the normal step fits
       under the cap or entry_t s have passed. Not used in lane change / when lateral is inactive.
     clip_curvature, jerk/accel limits, LatMpc*Cost, the laneless path and the lane-mode on/off conditions are unchanged.
Add a new number only for a candidate that passed the same checks; never edit an existing number's constants.
"""

from openpilot.selfdrive.controls.lib.k9_lane import K9_LANE_PARAM

K9_LANE_FIX_PARAM = K9_LANE_PARAM   # was "K9LaneFix" (migrated by k9_lane.migrate_k9_lane)

K9_LANE_FIX_PROFILES = {
  1: {
    "name": "F1",
    # lane_planner_2.py (reference shaping)
    "converge_mode": "smootherstep",
    "converge_t": 2.0,
    "ramp_in_t": 0.5,
    "t_eref": 0.3,
    "t_max": 4.0,
    "t_kref": 0.003,
    # controlsd.py (lane-mode -> laneless command hand-over)
    "entry_jerk": 2.5,
    "entry_t": 2.0,
  },
}


def k9_lane_fix_mode(params) -> int:
  """Read K9LaneAssist; 1 -> profile 1 (F1), anything else / unreadable is 0 (off)."""
  try:
    mode = int(params.get_int(K9_LANE_FIX_PARAM))
  except Exception:
    return 0
  return mode if mode in K9_LANE_FIX_PROFILES else 0


def k9_lane_fix_profile(mode):
  return K9_LANE_FIX_PROFILES.get(mode)
