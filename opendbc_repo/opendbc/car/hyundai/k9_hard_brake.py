"""K9 hard-brake ease-off: keep the ESC from over-releasing while the planner eases off a hard brake for a closing lead
(2026-10-24 evening, the driver's report 19:12 "거리 미리 확보 못해서 쎄게 잡을 때 놨다 잡았다 하는 거 같은데 이때 드드득거리기도
하고 좀 일정하게 잡게 못함?"). Log analysis and simulator only, NOT verified on the vehicle.

KIA_K9 (CAN, camera SCC, openpilot longitudinal), every K9StopMode value (0-3); a shared hard-braking path before the
K9StopMode stop logic (k9_creep.py). It never changes the sent aReq, StopReq or JerkLowerLimit; it only lowers the SCC14
JerkUpperLimit (the ESC's allowed rate of releasing) to its floor HB_JERK_U (0.5 m/s^3) on the frames where:
- active: longControlState pid, longActive, no override/gas/brake, valid CAN, vEgo above HB_MIN_SPEED (3.0 m/s), a lead
  (hudControl leadVisible or a non-zero lead distance) that is closing (leadRelSpeed < 0); it starts when the command is
  at or below HB_ENTER (-1.0 m/s^2) and ends when the command is above HB_EXIT (-0.5), the lead stops closing, the speed
  falls to 3.0 m/s or any of the conditions above drops;
- and the planner jerk (actuators.jerk = plan jTargetNow) is in (0, HB_EASE_JERK): a slow, partial ease-off (the base
  limit there is max(0.5, 2 x jerk) = 0.5..2.0). A fast planned release (jerk >= 1.0 m/s^3) keeps the base limit, so the
  end of a hard brake is released as before.

Reason (K9 rlog, hard-brake clusters aReq or aEgo < -1.5 m/s^2, 2026-10-10..10-24, worker brake_hard_pulse; estimate, 추정):
the release-then-regrab cycles of the sent aReq were 0.87 per 10 s of hard braking with ComfortBand 0 (K9StopMode 1-3 send
band 0 at every speed) against 0.23 with the base band, of the measured decel 1.20 against 0.39; the ESC followed aReq with
~0.04-0.06 s lag (0.22 s with the band) and overshot the command peak (1.18-1.23 x, 1.04 x with the band). The planner eased
off at 0.1-0.8 m/s^3 while the lead still closed (11 of 14 releases), and each such release came back as a measured release
about 1.9 times as large (median, 1.2-3.4) when the planner jerk had raised JerkUpperLimit to 1.0 or more (0-1.1 times, 3 releases, with 0.5).
A hold of the sent aReq itself was tried and dropped: in log replay it removed few cycles (19 -> 15 of 69 clusters) and could
brake up to 3.1 m/s^2 more than the planner for seconds after a cut-in. The ComfortBand is not changed here (restoring it at
speed delayed the brake onset by ~0.2-0.3 s, p90 0.6-1.3 s, in the same logs).
"""
import math

HB_ENTER = -1.0  # m/s^2: command at or below this (closing lead, above HB_MIN_SPEED) starts it
HB_EXIT = -0.5  # m/s^2: ends once the command is above this
HB_MIN_SPEED = 3.0  # m/s (10.8 km/h)
HB_EASE_JERK = 1.0  # m/s^3: planner jerk below this (and above 0) = slow partial ease-off
HB_JERK_U = 0.5  # m/s^3 SCC14 JerkUpperLimit floor (= HyundaiJerk.jerk_u_min)


class K9HardBrake:
  def __init__(self):
    self.active = False
    self.jerk = 0.0

  def reset(self):
    self.active = False
    self.jerk = 0.0

  def update(self, *, accel: float, jerk: float, pid: bool, v_ego: float, long_active: bool, override: bool, gas: bool,
             brake: bool, can_valid: bool, lead_visible: bool, lead_distance: float, lead_rel_speed: float) -> None:
    if (not all(math.isfinite(x) for x in (accel, jerk, v_ego, lead_distance, lead_rel_speed)) or not pid or not long_active
        or override or gas or brake or not can_valid):
      self.reset()
      return
    lead_seen = lead_visible or lead_distance != 0.0
    closing = lead_seen and lead_distance > 0.0 and lead_rel_speed < 0.0
    if not closing or v_ego <= HB_MIN_SPEED or accel > HB_EXIT:
      self.reset()
      return
    if not self.active and accel > HB_ENTER:
      return
    self.active = True
    self.jerk = float(jerk)

  def jerk_u_cap(self):
    """SCC14 JerkUpperLimit cap (None = base limit)."""
    return HB_JERK_U if self.active and 0.0 < self.jerk < HB_EASE_JERK else None
