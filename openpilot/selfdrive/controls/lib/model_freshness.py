"""Release lateral control when modelV2 stops arriving.

controlsd runs at 100 Hz and reuses the last modelV2 / lateralPlan. If modelV2 stops
(model switch gap, modeld stall) the last desiredCurvature was held unchanged for the
whole gap, and with AlwaysLateral there is no selfdrived disengage to end it.
0.5 s matches the modelV2 alive window selfdrived already uses for commIssue.
"""
from openpilot.common.realtime import DT_CTRL

MODEL_STALE_LAT_S = 0.5


def model_age_s(sm_frame: int, recv_frame: int) -> float:
  return (sm_frame - recv_frame) * DT_CTRL


def lateral_model_fresh(seen: bool, sm_frame: int, recv_frame: int) -> bool:
  return bool(seen) and model_age_s(sm_frame, recv_frame) <= MODEL_STALE_LAT_S
