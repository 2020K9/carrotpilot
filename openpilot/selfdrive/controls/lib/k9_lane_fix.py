"""K9LaneFix: numbered lane-mode reference / hand-over profiles (experimental menu, NOT vehicle-validated).

Param K9LaneFix (int, default 0):
  0 (or any value not in K9_LANE_FIX_PROFILES): off - lane_planner_2.py and controlsd.py behave exactly as before.
  1: "F1" (LANEFIX4, replay / closed-loop simulation only):
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

K9_LANE_FIX_PARAM = "K9LaneFix"

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
  """Read K9LaneFix; anything unknown / unreadable is 0 (off)."""
  try:
    mode = int(params.get_int(K9_LANE_FIX_PARAM))
  except Exception:
    return 0
  return mode if mode in K9_LANE_FIX_PROFILES else 0


def k9_lane_fix_profile(mode):
  return K9_LANE_FIX_PROFILES.get(mode)
