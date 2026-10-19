"""K9 terminal stop taper (candidate B): state transitions, precedence, standstill hold and CarController CAN.

The K9StopTaper param (default OFF) is checked against the pre-candidate CarController for identical CAN while OFF,
and for the live ON/OFF switch. Model-only constants; these tests check source logic, not K9 vehicle behaviour.
"""
import importlib.util
import json
import math
import random
import subprocess
from pathlib import Path

import pytest

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai import k9_stop_taper as kt
from opendbc.car.hyundai.k9_stop_taper import K9StopTaper, TaperPhase
from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import (
  BASE_PARAMS, K9_FLAGS, MemoryParams, make_ci_state, make_cp,
)

LCS = structs.CarControl.Actuators.LongControlState


def step(t, accel=-1.0, a_target=-1.0, v=0.5, a_ego=-0.5, standstill=False, long_active=True, override=False, gas=False,
         brake=False, can_valid=True, stopping_or_pid=True, lead=None, lead_visible=None):
  visible, d, vrel = (False, 0.0, 0.0) if lead is None else (True, lead[0], lead[1])
  lead_visible = visible if lead_visible is None else lead_visible  # override: plan hasLead vs radarState distance
  return t.update(accel=accel, a_target=a_target, v_ego=v, a_ego=a_ego, standstill=standstill, long_active=long_active,
                  override=override, gas=gas, brake=brake, can_valid=can_valid, stopping_or_pid=stopping_or_pid,
                  lead_visible=lead_visible, lead_distance=d, lead_rel_speed=vrel)


def armed():
  t = K9StopTaper()
  assert step(t, v=3.0) == -1.0 and t.armed and t.phase == TaperPhase.idle
  return t


def test_applies_only_to_k9_camera_scc_openpilot_long():
  assert kt.k9_stop_taper_applies(make_cp())
  assert not kt.k9_stop_taper_applies(make_cp(fingerprint=CAR.KIA_K7))
  assert not kt.k9_stop_taper_applies(make_cp(flags=int(HyundaiFlags.LEGACY)))
  assert not kt.k9_stop_taper_applies(make_cp(flags=K9_FLAGS | int(HyundaiFlags.CANFD)))
  cp = make_cp()
  cp.openpilotLongitudinalControl = False
  assert not kt.k9_stop_taper_applies(cp)


def test_not_armed_without_prior_motion():
  t = K9StopTaper()
  for _ in range(50):
    assert step(t, v=0.5) == -1.0
  assert t.phase == TaperPhase.idle


def test_taper_weakens_rate_limited_never_stronger_and_floor():
  t = armed()
  outs = [step(t, v=0.8)]
  for _ in range(100):
    outs.append(step(t, v=0.2))
  assert t.phase == TaperPhase.taper
  diffs = [b - a for a, b in zip(outs, outs[1:], strict=False)]
  assert max(diffs) <= kt.RELEASE_RATE * kt.DT + 1e-9
  assert all(o >= -1.0 - 1e-12 for o in outs)  # never stronger than the original
  assert outs[-1] == pytest.approx(kt.taper_floor(0.2, 0, 0, False))  # -0.5


def test_original_strengthening_is_immediate_and_weaker_original_wins():
  t = armed()
  for _ in range(30):
    step(t, v=0.3)
  assert step(t, accel=-1.2, a_target=-1.0, v=0.3) > -1.2  # small increase stays within the taper
  assert step(t, accel=-0.1, a_target=-0.1, v=0.3) == -0.1  # weaker original (e.g. plan release)
  t = armed()
  for _ in range(30):
    step(t, v=0.3)
  assert step(t, accel=-1.31, a_target=-1.0, v=0.3) == -1.31 and t.phase == TaperPhase.done


@pytest.mark.parametrize("entry", [True, False])
def test_strong_braking_precedence(entry):
  t = armed()
  if entry:
    assert step(t, accel=-1.6, v=0.5) == -1.6
    assert t.phase == TaperPhase.idle
    assert step(t, accel=-1.0, a_target=-1.5, v=0.5) == -1.0 and t.phase == TaperPhase.idle
  else:
    for _ in range(20):
      step(t, v=0.5)
    assert t.phase == TaperPhase.taper
    assert step(t, accel=-1.0, a_target=-2.0, v=0.5) == -1.0
    assert t.phase == TaperPhase.done
    for _ in range(20):  # no re-entry in the same episode
      assert step(t, v=0.4) == -1.0


def test_lead_kinematic_need_bounds_floor():
  # closing 0.8 m/s, 2.2 m to the lead: need 0.8^2 / (2 * 0.2) = 1.6 m/s^2, stronger than the nominal floor
  floor = kt.taper_floor(0.8, 2.2, -0.8, True)
  assert floor == pytest.approx(-(0.8 ** 2) / (2 * 0.2))
  t = armed()
  for _ in range(200):
    out = step(t, accel=-1.2, v=0.8, lead=(2.2, -0.8))
  assert out == -1.2  # need 1.6 > original: original kept
  # far, stationary-relative lead: nominal floor
  assert kt.taper_floor(0.5, 20.0, -0.5, True) == pytest.approx(-(kt.END_DECEL + 0.5))


def test_complete_standstill_restores_original_hold_then_done():
  t = armed()
  for i in range(80):
    step(t, v=max(0.9 - 0.01 * i, 0.05))
  tapered = t.out
  assert tapered > -1.0
  outs = []
  for _ in range(100):
    outs.append(step(t, v=0.0, standstill=True))
  assert outs[0] == pytest.approx(tapered - kt.REAPPLY_RATE * kt.DT)
  assert all(b <= a + 1e-12 for a, b in zip(outs, outs[1:], strict=False))
  assert outs[-1] == -1.0 and t.phase == TaperPhase.done
  for _ in range(300):  # long hold: original only, no re-taper at standstill or slow creep
    assert step(t, v=0.0, standstill=True) == -1.0
  assert step(t, v=0.3) == -1.0


def test_motion_or_departure_during_restore_returns_original_immediately():
  t = armed()
  for _ in range(40):
    step(t, v=0.3)
  step(t, v=0.0, standstill=True)
  assert t.phase == TaperPhase.restore
  assert step(t, v=0.2, standstill=False) == -1.0 and t.phase == TaperPhase.done
  t = armed()
  for _ in range(40):
    step(t, v=0.3)
  step(t, v=0.0, standstill=True)
  assert step(t, accel=1.0, a_target=0.5, v=0.0, standstill=True) == 1.0  # restart not delayed


def _in_restore(entry_accel=-1.0):
  t = armed()
  for _ in range(30):  # 0.09 m travelled, inside the 0.165 m budget
    step(t, accel=entry_accel, a_target=entry_accel, v=0.3)
  assert t.phase == TaperPhase.taper and t.out > entry_accel + 0.3
  out = step(t, accel=entry_accel, a_target=entry_accel, v=0.0, standstill=True)
  assert t.phase == TaperPhase.restore and out > entry_accel
  return t


# Every precedence guard must pass the original through at once during restore too, not after the reapply ramp.
@pytest.mark.parametrize("kw,expected,reason", [
  ({"accel": -1.2, "a_target": -1.5}, -1.2, "strong_braking"),  # planner aTarget exactly STRONG_ACCEL
  ({"accel": -1.2, "a_target": -2.5}, -1.2, "strong_braking"),
  ({"accel": -1.5, "a_target": -1.0}, -1.5, "strong_braking"),  # original exactly STRONG_ACCEL
  ({"accel": -1.4, "a_target": -1.0}, -1.4, "strengthened"),  # > STRENGTHEN_ABORT stronger than entry -1.0
  ({"gas": True}, -1.0, "interlock"),
  ({"brake": True}, -1.0, "interlock"),
  ({"override": True}, -1.0, "interlock"),
  ({"long_active": False}, -1.0, "interlock"),
  ({"can_valid": False}, -1.0, "interlock"),
  ({"stopping_or_pid": False}, -1.0, "interlock"),
  ({"lead": (1.5, 0.0)}, -1.0, "lead_too_close"),
  ({"lead": (0.0, 0.0)}, -1.0, "lead_too_close"),  # zero/negative range: invalid lead data
  ({"lead": (-3.0, 0.0)}, -1.0, "lead_too_close"),
  ({"lead": (10.0, float("nan"))}, -1.0, "invalid"),
  ({"a_ego": float("inf")}, -1.0, "invalid"),
  ({"v": 0.0, "standstill": False}, -1.0, "moving"),  # standstill flag dropped: forward motion possible
  ({"v": 0.2, "standstill": True}, -1.0, "moving"),  # forward motion despite a standstill flag
])
def test_restore_precedence_passes_original_immediately(kw, expected, reason):
  t = _in_restore()
  args = {"accel": -1.0, "a_target": -1.0, "v": 0.0, "standstill": True} | kw
  assert step(t, **args) == expected and t.phase == TaperPhase.done and t.reason == reason
  assert not t.armed
  for _ in range(50):  # stays original for the rest of the stop
    assert step(t, v=0.0, standstill=True) == -1.0


@pytest.mark.parametrize("lead", [(2.0, 0.0), (1.0, -0.3), (0.0, 0.0), (-1.0, 0.0)])
def test_too_close_or_invalid_lead_passes_original_in_taper_and_blocks_entry(lead):
  t = armed()
  for _ in range(30):
    step(t, v=0.5)
  assert t.phase == TaperPhase.taper and t.out > -1.0
  assert step(t, v=0.5, lead=lead) == -1.0 and t.reason == "lead_too_close" and t.phase == TaperPhase.done
  t = armed()
  for _ in range(30):
    assert step(t, v=0.5, lead=lead) == -1.0
  assert t.phase == TaperPhase.idle


def test_restore_hold_accel_is_modified_only_transiently():
  # Documented fact: restore DOES send a weaker-than-original hold accel right after standstill, for a bounded time.
  t = _in_restore()
  outs = [t.out]
  while t.phase == TaperPhase.restore:
    outs.append(step(t, v=0.0, standstill=True))
  assert outs[0] > -1.0 and outs[-1] == -1.0
  assert len(outs) - 1 <= math.ceil((outs[0] + 1.0) / (kt.REAPPLY_RATE * kt.DT)) + 1


@pytest.mark.parametrize("kw", [{"gas": True}, {"brake": True}, {"override": True}, {"long_active": False},
                                {"can_valid": False}, {"stopping_or_pid": False}])
def test_interlocks_pass_original_and_disarm(kw):
  t = armed()
  for _ in range(30):
    step(t, v=0.4)
  assert t.phase == TaperPhase.taper
  assert step(t, v=0.4, **kw) == -1.0 and t.phase == TaperPhase.done and not t.armed
  for _ in range(30):  # pedal return while still slow: original (needs re-arm)
    assert step(t, v=0.4) == -1.0


@pytest.mark.parametrize("field", ["accel", "a_target", "v", "a_ego", "lead_d", "lead_v"])
def test_non_finite_inputs_pass_original(field):
  t = armed()
  for _ in range(30):
    step(t, v=0.4)
  kw = {"accel": -1.0, "a_target": -1.0, "v": 0.4, "a_ego": -0.5, "lead": (10.0, -0.4)}
  bad = float("nan")
  if field == "lead_d":
    kw["lead"] = (bad, -0.4)
  elif field == "lead_v":
    kw["lead"] = (10.0, bad)
  else:
    kw[field] = bad
  out = step(t, **kw)
  assert (math.isnan(out) if field == "accel" else out == -1.0) and t.phase == TaperPhase.done


def test_rearm_after_moving_again():
  t = armed()
  for _ in range(40):
    step(t, v=0.3)
  for _ in range(100):
    step(t, v=0.0, standstill=True)
  assert t.phase == TaperPhase.done
  step(t, accel=0.5, a_target=0.5, v=2.0)
  assert t.phase == TaperPhase.idle and t.armed
  step(t, v=0.6)
  assert t.phase == TaperPhase.taper


def _stop_distance(taper, a_cmd, v0=kt.TAPER_SPEED - 1e-6, tau=0.0, g=0.0, dead=0.0):
  t = armed()
  v, x, a_act = v0, 0.0, a_cmd
  queue = [a_cmd] * round(dead / kt.DT)  # pure transport delay before the first-order lag
  for _ in range(3000):
    out = step(t, accel=a_cmd, a_target=a_cmd, v=v, a_ego=a_act + g) if taper else a_cmd
    queue.append(out)
    applied = queue.pop(0)
    a_act += (applied - a_act) * (1.0 if tau == 0 else min(1.0, kt.DT / tau))
    nv = v + (a_act + g) * kt.DT
    if nv <= 0:
      x += v * v / (2 * -(a_act + g))
      return x
    x += 0.5 * (v + nv) * kt.DT
    v = nv
  raise AssertionError("did not stop")


@pytest.mark.parametrize("g", [0.0, 0.49, 0.78])  # flat, 5 %, 8 % downhill (v1 never stopped on the last two)
@pytest.mark.parametrize("a_cmd", [-1.0, -1.2, -1.49])
def test_added_stop_distance_bound_without_lead(a_cmd, g):
  extra = _stop_distance(True, a_cmd, g=g) - _stop_distance(False, a_cmd, g=g)
  assert -1e-6 <= extra <= kt.MAX_EXTRA_DISTANCE


def test_no_progress_and_max_time_return_original():
  t = armed()
  for _ in range(10):
    step(t, v=0.5)
  for _ in range(19):
    assert step(t, v=0.5, a_ego=-0.1) > -1.0
  assert step(t, v=0.5, a_ego=-0.1) == -1.0 and t.reason == "no_progress" and t.phase == TaperPhase.done
  t = armed()
  outs = [step(t, v=0.05) for _ in range(round(kt.TAPER_MAX_TIME / kt.DT) + 5)]
  assert outs[-1] == -1.0 and t.reason == "no_progress"  # max time
  t = armed()
  outs = [step(t, v=0.5) for _ in range(150)]  # constant speed: travel exceeds 0.5^2/2 + margin
  assert outs[-1] == -1.0 and t.reason == "distance_budget"



# Simplified-plant sensitivity only (first-order lag tau + dead time, no ESC hold/friction/pitch): NOT a vehicle
# validation and not a K9 lag calibration. g > 0 is downhill, g < 0 uphill (m/s^2 along the road).
@pytest.mark.parametrize("dead", [0.0, 0.1, 0.3])
@pytest.mark.parametrize("tau", [0.05, 0.15, 0.3, 0.5, 0.7])
@pytest.mark.parametrize("g", [0.0, 0.49, 0.78, -0.49, -0.78])
@pytest.mark.parametrize("a_cmd", [-1.0, -1.49])
def test_added_stop_distance_bound_with_actuator_lag_and_delay(a_cmd, g, tau, dead):
  base = _stop_distance(False, a_cmd, tau=tau, g=g, dead=dead)
  extra = _stop_distance(True, a_cmd, tau=tau, g=g, dead=dead) - base
  assert -1e-6 <= extra <= kt.MAX_EXTRA_DISTANCE


@pytest.mark.parametrize("tau,dead", [(0.05, 0.0), (0.3, 0.1), (0.7, 0.3)])
def test_steep_downhill_with_lag_stops_whenever_original_stops(tau, dead):
  # 12 % downhill: the original -1.49 still stops; the candidate must too. Its added distance (about 0.26-0.33 m in
  # this model) exceeds the flat/5/8 % MAX_EXTRA_DISTANCE bound, which is therefore not a steep-grade bound.
  base = _stop_distance(False, -1.49, tau=tau, g=1.18, dead=dead)
  assert _stop_distance(True, -1.49, tau=tau, g=1.18, dead=dead) >= base - 1e-6


@pytest.mark.parametrize("v", [-0.4, -0.11])
def test_backward_motion_during_restore_returns_original_despite_stale_standstill(v):
  # K9 wheel speeds are unsigned, so this is a finite-invalid-input guard: |vEgo| above MOVING_AFTER_STOP is motion.
  t = _in_restore()
  assert step(t, v=v, standstill=True) == -1.0 and t.phase == TaperPhase.done and t.reason == "moving"
  t = _in_restore()
  assert step(t, v=-0.05, standstill=True) > -1.0 and t.phase == TaperPhase.restore  # small KF undershoot: no change


@pytest.mark.parametrize("lead", [(1.5, -0.5), (2.0, 0.0), (-1.0, 0.0)])
def test_radar_lead_without_plan_has_lead_is_respected(lead):
  # controlsd: leadVisible = plan.hasLead, leadDistance = current radarState leadOne.dRel. Recorded K9 drives show
  # hasLead False with a non-zero distance (down to ~1.5 m) for up to ~60 ms.
  t = armed()
  for _ in range(30):
    step(t, v=0.5)
  assert t.phase == TaperPhase.taper and t.out > -1.0
  assert step(t, v=0.5, lead=lead, lead_visible=False) == -1.0 and t.reason == "lead_too_close"
  t = armed()
  for _ in range(30):
    assert step(t, v=0.5, lead=lead, lead_visible=False) == -1.0
  assert t.phase == TaperPhase.idle


def test_radar_lead_without_plan_has_lead_bounds_floor():
  t = armed()
  outs = [step(t, accel=-1.2, v=0.8, lead=(2.2, -0.8), lead_visible=False) for _ in range(30)]  # inside the budget
  assert t.phase == TaperPhase.taper and all(o == -1.2 for o in outs)  # need 1.6 m/s^2 > original: kept


@pytest.mark.parametrize("leave", [{"accel": 0.0, "a_target": 0.0}, {"accel": 0.2, "a_target": 0.2}, {"v": 1.0}])
def test_leaving_window_ends_episode_until_rearm(leave):
  t = armed()
  for _ in range(30):
    step(t, v=0.6)
  assert t.phase == TaperPhase.taper and t.out > -1.0
  args = {"v": 0.6} | leave
  assert step(t, **args) == args.get("accel", -1.0) and t.reason == "left_window" and t.phase == TaperPhase.done
  for _ in range(300):  # back inside the window in the same stop: original, no fresh budget/time
    assert step(t, v=0.6) == -1.0
  step(t, v=kt.REARM_SPEED)
  assert t.armed and t.phase == TaperPhase.idle
  step(t, v=0.6)
  assert t.phase == TaperPhase.taper


def test_oscillating_original_cannot_restart_distance_budget():
  # A one-frame accel >= 0 every 0.4 s at constant 0.6 m/s used to start a new episode with a fresh budget each time.
  t = armed()
  tapered_travel = 0.0
  for i in range(400):
    a = 0.0 if i % 40 == 39 else -1.0
    out = step(t, accel=a, a_target=a, v=0.6)
    if out != a:
      tapered_travel += 0.6 * kt.DT
  assert tapered_travel <= 0.6 ** 2 / 2.0 + kt.DISTANCE_MARGIN


@pytest.mark.parametrize("seq", [
  [(0.0, True), (0.0, False), (0.0, True)],  # standstill flag drops for one frame at rest
  [(0.105, True)],  # recorded K9: vEgo 0.10-0.109 m/s at the standstill edge in 17 of 73 stops
])
def test_standstill_edge_flicker_and_lagging_vego_pass_original(seq):
  t = armed()
  for _ in range(30):
    step(t, v=0.3)
  assert t.phase == TaperPhase.taper
  outs = [step(t, v=v, standstill=ss) for v, ss in seq]
  assert t.phase == TaperPhase.done and t.reason == "moving" and outs[-1] == -1.0
  for _ in range(50):
    assert step(t, v=0.0, standstill=True) == -1.0


def test_sub_1kmh_creep_ends_by_budget_or_time():
  t = armed()
  outs = [step(t, v=0.25, a_ego=-0.3) for _ in range(round(kt.TAPER_MAX_TIME / kt.DT) + 10)]
  assert t.phase == TaperPhase.done and t.reason in ("distance_budget", "no_progress") and outs[-1] == -1.0

# --- CarController integration: SCC12 aReq/StopReq on K9 with valid CAN -------------------------------------------

BASE_COMMIT = "38df0d7c5abaae4c677951ce3b6dc0dfd94c739d"  # carrot-wip before the K9StopTaper candidate
ROOT = Path(__file__).resolve().parents[4]
CC_REL = "opendbc_repo/opendbc/car/hyundai/carcontroller.py"


class Rig:
  def __init__(self, monkeypatch, cp=None, taper=True, module=carcontroller, **values):
    self.params = MemoryParams({**BASE_PARAMS, **({"K9StopTaper": True} if taper else {}), **values})
    monkeypatch.setattr(module, "Params", lambda *a, **k: self.params)
    self.cc = module.CarController({Bus.pt: "hyundai_kia_generic"}, make_cp() if cp is None else cp)
    self.parser = CANParser("hyundai_kia_generic", [("SCC12", 0)], 0)
    self.frame = 0
    self.can = []

  def step(self, accel, state, v, standstill=False, a_target=None, gas=False, brake=False, a_ego=None, lead=None,
           lead_visible=True):
    cc = structs.CarControl(enabled=True, longActive=True)
    cc.actuators.accel, cc.actuators.aTarget = accel, accel if a_target is None else a_target
    cc.actuators.longControlState = state
    if lead is not None:
      cc.hudControl.leadVisible, cc.hudControl.leadDistance, cc.hudControl.leadRelSpeed = lead_visible, lead[0], lead[1]
    cs = make_ci_state()
    cs.out = structs.CarState(vEgo=v, vEgoRaw=v, standstill=standstill, aEgo=accel if a_ego is None else a_ego,
                              canValid=True, gasPressed=gas, brakePressed=brake)
    out, sends = self.cc.update(cc.as_reader(), cs, self.frame * 10_000_000)
    self.can.append(sorted((addr, bytes(dat), bus) for addr, dat, bus in sends))
    scc12 = [m for m in sends if m[0] == 1057]
    sent = None
    if scc12:
      self.parser.update([self.frame * 10_000_000, scc12])
      sent = dict(self.parser.vl["SCC12"])
    self.frame += 1
    return out.accel, sent


def stop_sequence(rig):
  rows = []
  for _ in range(100):
    rows.append(rig.step(-1.0, LCS.pid, 5.0))
  for i in range(150):
    rows.append(rig.step(-1.2, LCS.stopping, max(1.4 - 0.01 * i, 0.02)))
  for _ in range(200):
    rows.append(rig.step(-1.2, LCS.stopping, 0.0, standstill=True))
  for _ in range(50):
    rows.append(rig.step(1.0, LCS.pid, 0.0, standstill=True))
  return rows


def test_carcontroller_k9_taper_changes_only_terminal_accel_and_keeps_stopreq(monkeypatch):
  base = stop_sequence(Rig(monkeypatch, taper=False))
  monkeypatch.undo()
  cand = stop_sequence(Rig(monkeypatch))
  assert len(base) == len(cand)
  changed = 0
  for i, ((ba, bs), (ca, cs)) in enumerate(zip(base, cand, strict=True)):
    assert ca >= ba - 1e-9  # never stronger than the original
    if bs is not None:
      assert cs["StopReq"] == bs["StopReq"] and cs["ACCMode"] == bs["ACCMode"]
      assert cs["aReqValue"] == pytest.approx(ca, abs=0.006)
    if ca != ba:
      changed += 1
      assert 140 <= i < 100 + 150 + 200  # only below 1 m/s and during the standstill restore
  assert changed > 0
  assert cand[100 + 150 + 199][0] == pytest.approx(-1.2)  # full original hold before the end of standstill
  assert all(c[0] == 1.0 for c in cand[-50:])  # restart unchanged
  assert (CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX) == (-4.0, 2.5)


def test_carcontroller_other_hyundai_unchanged(monkeypatch):
  cp = make_cp(fingerprint=CAR.KIA_K7)
  base = stop_sequence(Rig(monkeypatch, cp=cp, taper=False))
  monkeypatch.undo()
  cand = stop_sequence(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7)))
  assert base == cand


# --- OFF equivalence with the pre-candidate CarController, live switch, settings registration ----------------------

@pytest.fixture(scope="module")
def base_cc(tmp_path_factory):
  try:
    source = subprocess.run(["git", "show", f"{BASE_COMMIT}:{CC_REL}"], cwd=ROOT, check=True, capture_output=True,
                            text=True, encoding="utf-8").stdout
  except (OSError, subprocess.CalledProcessError) as e:
    pytest.fail(f"cannot read {CC_REL} at {BASE_COMMIT}: {e}")
  assert "k9_stop_taper" not in source
  path = tmp_path_factory.mktemp("base_cc") / "carcontroller_base.py"
  path.write_text(source, encoding="utf-8")
  spec = importlib.util.spec_from_file_location("carcontroller_k9_taper_base", path)
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


def mixed_sequence(rig, seed="k9-stop-taper-off"):
  """Repeated stops (no lead/closing lead), pedals, standstill hold, hard braking and restarts with noisy aEgo."""
  rng = random.Random(seed)
  for cycle in range(4):
    lead = (6.0, -0.5) if cycle % 2 else None
    for _ in range(80):
      rig.step(rng.uniform(-1.0, 0.5), LCS.pid, 5.0 + rng.uniform(-0.5, 0.5))
    for i in range(160):
      a = -2.0 if cycle == 3 and 60 <= i < 80 else rng.uniform(-1.4, -0.4)
      rig.step(a, LCS.stopping if i > 40 else LCS.pid, max(1.6 - 0.01 * i, 0.02), a_ego=rng.uniform(-1.2, -0.2),
               brake=cycle == 2 and 100 <= i < 110, lead=lead)
    for i in range(150):
      rig.step(-1.2, LCS.stopping, 0.0, standstill=True, gas=cycle == 1 and 50 <= i < 55)
    for i in range(60):
      rig.step(0.8, LCS.pid, 0.05 * i, standstill=i == 0)
  return rig


@pytest.mark.parametrize("values", [{}, {"K9StopTaper": False}, {"K9StopTaper": 0}], ids=["missing", "false", "zero"])
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
def test_off_sends_identical_can_to_pre_candidate_controller(monkeypatch, base_cc, values, fingerprint):
  ref = mixed_sequence(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), taper=False, module=base_cc))
  monkeypatch.undo()
  new = mixed_sequence(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), taper=False, **values))
  assert len(ref.can) == len(new.can) == 1800 and ref.can == new.can


def test_on_k7_identical_and_on_k9_differs_from_pre_candidate(monkeypatch, base_cc):
  ref = mixed_sequence(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), taper=False, module=base_cc))
  monkeypatch.undo()
  assert mixed_sequence(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7))).can == ref.can
  monkeypatch.undo()
  ref9 = mixed_sequence(Rig(monkeypatch, taper=False, module=base_cc))
  monkeypatch.undo()
  assert mixed_sequence(Rig(monkeypatch)).can != ref9.can  # the scenario really exercises the taper


def edge_sequence(rig, seed="k9-stop-taper-edges"):
  """Plan/radar lead disagreement, standstill flicker and lagging vEgo, accel sign oscillation, sub-1 km/h creep."""
  rng = random.Random(seed)
  for cycle in range(4):
    for _ in range(60):
      rig.step(-0.8, LCS.pid, 4.0)
    for i in range(140):
      v = max(1.6 - 0.011 * i, 0.03)
      a = 0.05 if cycle == 1 and i % 30 == 29 else rng.uniform(-1.3, -0.6)
      lead = (rng.uniform(1.2, 3.0), -v) if cycle == 2 and i % 7 < 3 else None
      rig.step(a, LCS.stopping if i > 30 else LCS.pid, 0.25 if cycle == 3 and i > 100 else v,
               a_ego=rng.uniform(-1.0, 0.1), lead=lead, lead_visible=i % 2 == 0)
    for i in range(80):
      rig.step(-1.1, LCS.stopping, 0.105 if i < 3 else 0.0, standstill=i not in (5, 6))
    for i in range(40):
      rig.step(0.8, LCS.pid, 0.05 * i, standstill=i == 0)
  return rig


@pytest.mark.parametrize("values", [{}, {"K9StopTaper": False}], ids=["missing", "false"])
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
def test_off_edge_sequence_identical_to_pre_candidate_controller(monkeypatch, base_cc, values, fingerprint):
  ref = edge_sequence(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), taper=False, module=base_cc))
  monkeypatch.undo()
  new = edge_sequence(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), taper=False, **values))
  assert len(ref.can) == len(new.can) == 1280 and ref.can == new.can


def test_on_edge_sequence_k7_identical_k9_never_stronger_and_stopreq_kept(monkeypatch, base_cc):
  ref7 = edge_sequence(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), taper=False, module=base_cc))
  monkeypatch.undo()
  assert edge_sequence(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7))).can == ref7.can
  monkeypatch.undo()
  parser = CANParser("hyundai_kia_generic", [("SCC12", 0)], 0)

  def scc12(can):
    out = []
    for i, frame in enumerate(can):
      msgs = [(a, d, b) for a, d, b in frame if a == 1057]
      if msgs:
        parser.update([i * 10_000_000, msgs])
        out.append((parser.vl["SCC12"]["StopReq"], parser.vl["SCC12"]["ACCMode"], parser.vl["SCC12"]["aReqValue"]))
    return out
  base, cand = scc12(edge_sequence(Rig(monkeypatch, taper=False, module=base_cc)).can), None
  monkeypatch.undo()
  cand = scc12(edge_sequence(Rig(monkeypatch)).can)
  assert len(base) == len(cand) and base != cand
  for (bs, bm, ba), (cs, cm, ca) in zip(base, cand, strict=True):
    assert (cs, cm) == (bs, bm) and ca >= ba - 0.011  # StopReq/ACCMode kept; aReq never stronger (2 LSB)


ORIG = pytest.approx(-1.2, abs=1e-6)  # original command after float32 CarControl storage


def _taper_until_active(rig):
  for _ in range(100):
    rig.step(-1.0, LCS.pid, 5.0)
  for i in range(60):
    accel, _ = rig.step(-1.2, LCS.stopping, max(1.4 - 0.01 * i, 0.3))
  assert rig.cc.k9_stop_taper.phase == TaperPhase.taper and accel > -1.2


def test_live_switch_off_mid_taper_returns_original_within_one_read(monkeypatch):
  rig = Rig(monkeypatch)
  _taper_until_active(rig)
  rig.params.values["K9StopTaper"] = False
  outs = [rig.step(-1.2, LCS.stopping, 0.3) for _ in range(51)]
  first_original = next(i for i, (a, _) in enumerate(outs) if a == ORIG)
  assert 0 < first_original <= 50 and all(a == ORIG for a, _ in outs[first_original:])
  assert all(s is None or s["aReqValue"] == pytest.approx(a, abs=0.006) for a, s in outs)
  assert rig.cc.k9_stop_taper.phase == TaperPhase.idle and not rig.cc.k9_stop_taper.armed
  for _ in range(200):  # stays original while OFF, including standstill and restart
    assert rig.step(-1.2, LCS.stopping, 0.0, standstill=True)[0] == ORIG
  assert rig.step(1.0, LCS.pid, 0.0, standstill=True)[0] == pytest.approx(1.0)


def test_live_switch_on_mid_stop_waits_for_new_run(monkeypatch):
  rig = Rig(monkeypatch, taper=False)
  for _ in range(100):
    rig.step(-1.0, LCS.pid, 5.0)
  for i in range(20):
    rig.step(-1.2, LCS.stopping, 1.4 - 0.01 * i)
  rig.params.values["K9StopTaper"] = True
  for i in range(100):  # already below REARM_SPEED when switched on: original for the rest of this stop
    assert rig.step(-1.2, LCS.stopping, max(1.2 - 0.01 * i, 0.05))[0] == ORIG
  for _ in range(100):
    assert rig.step(-1.2, LCS.stopping, 0.0, standstill=True)[0] == ORIG
  _taper_until_active(rig)  # next stop after a fresh run: active


def test_setting_registered_in_experimental_menu_default_off():
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  driving = next(m for m in catalog["menu"] if m["id"] == "DRIVING")
  experimental = next(g for g in driving["groups"] if g["id"] == "EXPERIMENTAL")
  # user wrote "메뉴실행중 안에넣을것": the visible Korean group label is 실행중; the internal id stays EXPERIMENTAL
  assert experimental["ko"] == "실행중" and experimental["params"] == ["K9StopTaper"]
  assert driving["ko"] == "주행 제어"
  entry = next(p for p in catalog["params"] if p["name"] == "K9StopTaper")
  assert (entry["min"], entry["max"], entry["default"], entry["unit"]) == (0, 1, 0, 1)
  keys = (ROOT / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
  assert '{"K9StopTaper", {PERSISTENT, BOOL, "0"}},' in keys
