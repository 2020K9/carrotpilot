"""K9LaneAssist 1: one push at a time for the two laneless lane holds (experimental, NOT vehicle-validated).

보노보노 2026-10-25 23:38: controlsd used to add K9LaneCenter (both lines, k9_lane_center) and K9OneLaneHold (one line,
k9_one_lane_hold) straight onto the model desiredCurvature. Going from both lines to one line the centre hold ramps out over
1 s while the one-line hold watches 0.6 s and then ramps in over 1 s, so both could push at the same time (opposite ways too),
and their own caps added up (0.20 + 0.25 m/s^2 lateral accel, jerk 0.15 + 0.15 m/s^3).

LaneMix (called once per controlsd tick, after both holds ran - they keep their own state and limits):
  - owner: the one-line hold while it is holding (active), else the centre hold while it is holding, else whichever of the two
    still has the larger output (the one that is ramping out). Only the owner's output is used; the other one is ignored.
  - hand-over without a step: the combined output moves from the old owner's value to the new owner's value at the lateral-jerk
    limit J_MAX (and the curvature-rate cap KR_MAX below ~21 km/h), so it is continuous and never jumps when the owner changes.
  - combined caps: |lateral accel| <= A_MAX (A_MAX_CLOSE while the one-line hold owns it and the car is inside the remembered
    minimum gap), |curvature| <= K_MAX while the centre hold owns it, |jerk| <= J_MAX - the same numbers as the holds' own
    caps, never their sum.
  - PathOffset: both holds read the same k9_lane.k9_lane_offset_m value (+-0.30 m, + right) every 0.1 s.
With only one hold pushing and no hand-over the output is exactly that hold's output (its own steps are already inside these
limits, so the clip below returns it unchanged). K9LaneAssist 0: both holds return 0.0 and so does this (no state).
"""
import numpy as np

DT = 0.01
A_MAX, A_MAX_CLOSE, J_MAX = 0.25, 0.6, 0.15   # m/s^2, m/s^2, m/s^3 (one-line hold caps; the centre hold's own A_MAX is 0.20)
K_MAX, KR_MAX = 2.0e-3, 4.0e-3               # 1/m, 1/m/s (k9_lane_center low-speed caps)
V_MIN = 0.28                                  # m/s

OWNER_NONE, OWNER_LC, OWNER_OLH = 0, 2, 3      # same numbers as controlsState.k9LaneState (2 both-line hold, 3 one-line hold)


class LaneMix:
  def __init__(self):
    self.k = 0.0
    self.owner = OWNER_NONE

  def update(self, k_lc, k_olh, lc_active, olh_active, olh_close, v):
    if k_lc == 0.0 and k_olh == 0.0 and self.k == 0.0:
      self.owner = OWNER_NONE
      return 0.0
    if olh_active:
      owner = OWNER_OLH
    elif lc_active:
      owner = OWNER_LC
    elif k_lc != 0.0 or k_olh != 0.0:
      owner = OWNER_OLH if abs(k_olh) > abs(k_lc) else OWNER_LC
    else:
      owner = self.owner
    self.owner = owner
    tgt = k_olh if owner == OWNER_OLH else k_lc
    v2 = max(v, V_MIN) ** 2
    if owner == OWNER_OLH:
      amax = (A_MAX_CLOSE if olh_close else A_MAX) / v2
    else:
      amax = min(A_MAX / v2, K_MAX)     # centre hold: its low-speed curvature cap stays
    tgt = float(np.clip(tgt, -amax, amax))
    dk = min(J_MAX / v2, KR_MAX) * DT
    self.k = float(np.clip(tgt, self.k - dk, self.k + dk))
    if tgt == 0.0 and abs(self.k) <= 1e-12:
      self.k = 0.0
    return self.k


def k9_lane_mix_step(controls, k_lc, k_olh, v):
  """controlsd: the one curvature the K9 lane holds add to the laneless model command this tick."""
  if k_lc == 0.0 and k_olh == 0.0 and getattr(controls, 'k9_mix', None) is None:
    return 0.0
  mix = getattr(controls, 'k9_mix', None)
  if mix is None:
    mix = controls.k9_mix = LaneMix()
  lc = getattr(controls, 'k9_lc', None)
  olh = getattr(controls, 'k9_olh', None)
  out = mix.update(k_lc, k_olh, bool(lc is not None and lc.active), bool(olh is not None and olh.active),
                   bool(olh is not None and getattr(olh, 'close', False)), float(v))
  if out == 0.0 and mix.owner == OWNER_NONE:
    controls.k9_mix = None
  return out
