"""K9 camera-SCC "creep fix" v8/v9 stop feathering, selected by the K9CreepMode param (1 = v8, 2 = v9).

Source: https://leegrt.org/oppo/creep_fix/ (selectMode v8/v9 carcontroller.py diffs, written for Kia Niro EV CAN).
No license is stated on that page; this is an independent re-implementation of the described algorithm and its
published constants. It was designed for an EV with motor creep and is NOT verified on the K9.

Modes 0/3 (or any other car) never call update(): accel, StopReq, jerk and ComfortBand are the original. In modes
1/2 it replaces K9StopTaper (the caller resets the taper; modes never stack). v8 and v9 differ only in the FEATHER,
MIN_DECEL and CREEP tables and the down/up step rules (PROFILES); latch, boost, fade and jerk are shared.

What it changes while ON and active:
- stopping state, before the latch: StopReq is withheld (stopping=False sent to SCC12) and the sent accel is
  rate-limited toward a speed-scheduled feather target with eff_ac feedback (FEATHER/MIN_DECEL/BOOST tables);
- latch: once real standstill (TCS13 StandStill, vEgoRaw or vEgo < 0.05), aTarget <= -1.0, eff_ac > 0.04 above
  0.14 m/s or aEgo > 0.06 is seen, StopReq is sent as the original and accel = min(original, -0.50) until the
  stopping state ends;
- pid below 1.6 m/s with aTarget < -0.05: extra braking creep_comp, faded by aTarget and the accel itself;
- jerk (non-CAN-FD): jerk_u is raised below 1.4 m/s and ComfortBand upper/lower are 0 at every speed.
eff_ac = min(ESP12 LONG_ACCEL [m/s^2], aEgo + 0.10); LONG_ACCEL missing or non-finite falls back to aEgo.

Deviations from the source, all fail toward the original command (documented, not calibrated):
- disengaged/longActive off, override, gas, brake, invalid CAN, non-finite inputs: original accel/StopReq, reset;
- lead at <= LEAD_LATCH_GAP (plan leadVisible or non-zero radar distance, zero/negative counted) latches StopReq.
"""
import math

import numpy as np

from opendbc.car.hyundai.k9_stop_taper import k9_stop_taper_applies

V8, V9 = 1, 2  # K9CreepMode values


def _up_v8(v_ego, eff_ac, margin):
  return 0.0 if (v_ego > 0.14 and eff_ac > -0.06) else 0.0035


def _up_v9(v_ego, eff_ac, margin):
  return float(np.interp(margin, [-0.08, 0.02], [0.0032, 0.0])) if v_ego > 0.10 else 0.0032


PROFILES = {
  V8: dict(feather=([0.05, 0.13, 0.22, 0.38], [-0.26, -0.28, -0.35, -0.48]),
           min_decel=([0.08, 0.13, 0.25], [0.02, -0.06, -0.14]),
           creep=([0.18, 0.40, 1.0, 1.6], [-0.18, -0.24, -0.20, 0.0]),
           down_eff_ac=-0.03, up=_up_v8),
  V9: dict(feather=([0.05, 0.14, 0.25, 0.42], [-0.31, -0.35, -0.42, -0.52]),
           min_decel=([0.08, 0.14, 0.25, 0.40], [-0.05, -0.12, -0.20, -0.30]),
           creep=([0.25, 0.55, 1.0, 1.6], [-0.26, -0.26, -0.20, 0.0]),
           down_eff_ac=-0.02, up=_up_v9),
}
BOOST_BP, BOOST_V = [0.0, 0.15], [0.0, -0.18]
DOWN_STEP = 0.005
DOWN_UP_SPEED = 0.14  # m/s, both versions: down_step only above it, and the weak-decel latch
FADE_TARGET_BP, FADE_TARGET_V = [-0.20, -0.05], [1.0, 0.0]
FADE_ACCEL_BP, FADE_ACCEL_V = [-0.80, -0.30], [0.25, 1.0]
LATCH_ACCEL = -0.50
STANDSTILL_SPEED = 0.05
URGENT_TARGET = -1.0
JERK_U_BP, JERK_U_V = [0.5, 1.4], [2.0, 0.8]
LEAD_LATCH_GAP = 3.0  # m, deviation: lead protection is not in the source


def k9_creep_applies(CP) -> bool:
  return k9_stop_taper_applies(CP)  # KIA_K9, non-CAN-FD, camera SCC, openpilot longitudinal


def read_long_accel(cp):
  try:
    value = float(cp.vl["ESP12"]["LONG_ACCEL"])
  except (AttributeError, KeyError, TypeError, ValueError):
    return None
  return value if math.isfinite(value) else None


class K9Creep:
  def __init__(self, mode: int = V9):
    self.profile = PROFILES[mode]
    self.mode = mode
    self.reset()

  def reset(self):
    self.latched = False
    self.last = None  # previous sent accel; None means "start from the current command"
    self.active = False  # True only on a frame where update() applied v9 (jerk/band change follows it)

  def update(self, *, accel: float, a_target: float, stopping: bool, pid: bool, v_ego: float, v_ego_raw: float,
             a_ego: float, long_accel, esp_standstill: bool, long_active: bool, override: bool, gas: bool,
             brake: bool, can_valid: bool, lead_visible: bool, lead_distance: float):
    """Returns (accel, stopping) to send."""
    values = (accel, a_target, v_ego, v_ego_raw, a_ego, lead_distance)
    if (not all(math.isfinite(x) for x in values) or not long_active or override or gas or brake or not can_valid):
      self.reset()
      return accel, stopping
    self.active = True
    lon_ac = long_accel if long_accel is not None and math.isfinite(long_accel) else a_ego
    eff_ac = min(lon_ac, a_ego + 0.10)

    p = self.profile
    if not stopping:
      self.latched = False
      if pid and v_ego < p["creep"][0][-1] and a_target < FADE_TARGET_BP[-1]:
        comp = np.interp(v_ego, *p["creep"]) * np.interp(a_target, FADE_TARGET_BP, FADE_TARGET_V)
        comp *= np.interp(accel, FADE_ACCEL_BP, FADE_ACCEL_V)
        accel = float(min(accel, max(accel + comp, -4.0)))  # comp <= 0; -4.0 = CarControllerParams.ACCEL_MIN
      self.last = float(accel)
      return accel, stopping

    lead_close = (lead_visible or lead_distance != 0.0) and lead_distance <= LEAD_LATCH_GAP
    real_standstill = esp_standstill or v_ego_raw < STANDSTILL_SPEED or v_ego < STANDSTILL_SPEED
    if (real_standstill or a_target <= URGENT_TARGET or (v_ego > DOWN_UP_SPEED and eff_ac > 0.04) or a_ego > 0.06
        or lead_close):
      self.latched = True

    if self.latched:
      accel = float(min(accel, LATCH_ACCEL))
      self.last = accel
      return accel, True

    margin = eff_ac - float(np.interp(v_ego, *p["min_decel"]))
    target = float(np.interp(v_ego, *p["feather"])) + float(np.interp(margin, BOOST_BP, BOOST_V))
    prev = accel if self.last is None else self.last
    down = DOWN_STEP if (v_ego > DOWN_UP_SPEED and eff_ac > p["down_eff_ac"]) else 0.0
    up = float(p["up"](v_ego, eff_ac, margin))
    accel = float(min(max(target, prev - down), prev + up))
    self.last = accel
    return accel, False

  @staticmethod
  def jerk_bands(jerk_u: float, v_ego: float):
    """Modes 1/2 only, non-CAN-FD jerk/ComfortBand: (jerk_u, cb_upper, cb_lower)."""
    if v_ego < JERK_U_BP[-1]:
      jerk_u = max(jerk_u, float(np.interp(v_ego, JERK_U_BP, JERK_U_V)))
    return jerk_u, 0.0, 0.0
