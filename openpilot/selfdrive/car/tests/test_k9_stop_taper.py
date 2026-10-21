"""K9 terminal stop taper (candidate B): state transitions, precedence, standstill hold and CarController CAN.

The taper is selected only by K9CreepMode 3 (the old K9StopTaper param is kept but no longer read). It is checked
against the pre-candidate CarController for identical CAN while not selected, and for the live 3 <-> 0 switch.
Model-only constants; these tests check source logic, not K9 vehicle behaviour.
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
  ctrl = armed()
  assert step(ctrl, v=0.2) > -1.0 and ctrl.phase == TaperPhase.taper  # valid-entry control at the same speed
  t = K9StopTaper()
  for _ in range(50):
    assert step(t, v=0.2) == -1.0
  assert t.phase == TaperPhase.idle


def test_taper_weakens_rate_limited_never_stronger_and_floor():
  t = armed()
  outs = [step(t, v=0.25)]
  assert t.phase == TaperPhase.taper  # genuine entry inside the < 1 km/h window
  floor = kt.taper_floor(0.2, 0, 0, False)
  n_floor = math.ceil((floor - -1.0) / (kt.RELEASE_RATE * kt.DT))  # frames the release-rate limit needs to the floor
  n = 2 * n_floor + 10  # observe well past the floor
  assert 0.25 * kt.DT + n * 0.2 * kt.DT < t.budget  # examined interval fits the actual distance budget
  for _ in range(n):
    outs.append(step(t, v=0.2))
    assert t.phase == TaperPhase.taper  # no distance_budget expiry during the examined interval
  diffs = [b - a for a, b in zip(outs, outs[1:], strict=False)]
  assert max(diffs) <= kt.RELEASE_RATE * kt.DT + 1e-9
  assert sum(d == pytest.approx(kt.RELEASE_RATE * kt.DT) for d in diffs) >= 1  # the rate limit actually binds
  assert all(o >= -1.0 - 1e-12 for o in outs)  # never stronger than the original
  assert outs[-1] == pytest.approx(kt.taper_floor(0.2, 0, 0, False))  # -0.5


def test_original_strengthening_is_immediate_and_weaker_original_wins():
  t = armed()
  for _ in range(30):
    step(t, v=0.2)
  assert step(t, accel=-1.2, a_target=-1.0, v=0.2) > -1.2  # small increase stays within the taper
  assert step(t, accel=-0.1, a_target=-0.1, v=0.2) == -0.1  # weaker original (e.g. plan release)
  t = armed()
  for _ in range(30):
    step(t, v=0.2)
  assert step(t, accel=-1.31, a_target=-1.0, v=0.2) == -1.31 and t.phase == TaperPhase.done


@pytest.mark.parametrize("entry", [True, False])
def test_strong_braking_precedence(entry):
  t = armed()
  if entry:
    ctrl = armed()
    assert step(ctrl, v=0.2) > -1.0 and ctrl.phase == TaperPhase.taper  # valid-entry control at the same speed
    assert step(t, accel=-1.6, v=0.2) == -1.6
    assert t.phase == TaperPhase.idle
    assert step(t, accel=-1.0, a_target=-1.5, v=0.2) == -1.0 and t.phase == TaperPhase.idle
  else:
    for _ in range(20):
      step(t, v=0.2)
    assert t.phase == TaperPhase.taper
    assert step(t, accel=-1.0, a_target=-2.0, v=0.2) == -1.0
    assert t.phase == TaperPhase.done
    for _ in range(20):  # no re-entry in the same episode
      assert step(t, v=0.15) == -1.0


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
    step(t, v=0.2)
  step(t, v=0.0, standstill=True)
  assert t.phase == TaperPhase.restore
  assert step(t, v=0.2, standstill=False) == -1.0 and t.phase == TaperPhase.done
  t = armed()
  for _ in range(40):
    step(t, v=0.2)
  step(t, v=0.0, standstill=True)
  assert step(t, accel=1.0, a_target=0.5, v=0.0, standstill=True) == 1.0  # restart not delayed


def _in_restore(entry_accel=-1.0):
  t = armed()
  for _ in range(30):  # 0.06 m travelled, inside the 0.14 m budget
    step(t, accel=entry_accel, a_target=entry_accel, v=0.2)
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
    step(t, v=0.2)
  assert t.phase == TaperPhase.taper and t.out > -1.0
  assert step(t, v=0.2, lead=lead) == -1.0 and t.reason == "lead_too_close" and t.phase == TaperPhase.done
  t = armed()
  for _ in range(30):
    assert step(t, v=0.2, lead=lead) == -1.0
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
    step(t, v=0.2)
  assert t.phase == TaperPhase.taper
  assert step(t, v=0.2, **kw) == -1.0 and t.phase == TaperPhase.done and not t.armed
  for _ in range(30):  # pedal return while still slow: original (needs re-arm)
    assert step(t, v=0.2) == -1.0


@pytest.mark.parametrize("field", ["accel", "a_target", "v", "a_ego", "lead_d", "lead_v"])
def test_non_finite_inputs_pass_original(field):
  t = armed()
  for _ in range(30):
    step(t, v=0.2)
  assert t.phase == TaperPhase.taper and t.out > -1.0  # anti-vacuity: active and modified before the hazard
  kw = {"accel": -1.0, "a_target": -1.0, "v": 0.2, "a_ego": -0.5, "lead": (10.0, -0.2)}
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
    step(t, v=0.2)
  for _ in range(100):
    step(t, v=0.0, standstill=True)
  assert t.phase == TaperPhase.done
  step(t, accel=0.5, a_target=0.5, v=2.0)
  assert t.phase == TaperPhase.idle and t.armed
  step(t, v=0.2)
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
    step(t, v=0.2)
  for _ in range(19):
    assert step(t, v=0.2, a_ego=-0.1) > -1.0
  assert step(t, v=0.2, a_ego=-0.1) == -1.0 and t.reason == "no_progress" and t.phase == TaperPhase.done
  t = armed()
  outs = [step(t, v=0.05) for _ in range(round(kt.TAPER_MAX_TIME / kt.DT) + 5)]
  assert outs[-1] == -1.0 and t.reason == "no_progress"  # max time
  t = armed()
  outs = [step(t, v=0.2) for _ in range(150)]  # constant speed: travel exceeds 0.2^2/2 + margin
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
    step(t, v=0.2)
  assert t.phase == TaperPhase.taper and t.out > -1.0
  assert step(t, v=0.2, lead=lead, lead_visible=False) == -1.0 and t.reason == "lead_too_close"
  t = armed()
  for _ in range(30):
    assert step(t, v=0.2, lead=lead, lead_visible=False) == -1.0
  assert t.phase == TaperPhase.idle


def test_radar_lead_without_plan_has_lead_bounds_floor():
  t = armed()
  # Physically stationary lead (vRel = -vEgo) near the window top. Gap clamp 0.05 m caps the below-1 km/h need at
  # ~0.772 m/s^2, so the old "need 1.6 > original -1.2: kept" input is impossible here; the expectation is recomputed
  # from the same formula: output never weaker than the lead-required decel, and that floor binds below no-lead.
  v, lead = 0.27, (2.05, -0.27)
  need = -(0.27 ** 2) / (2.0 * (2.05 - kt.LEAD_MIN_GAP))
  floor = kt.taper_floor(v, *lead, True)
  assert floor == pytest.approx(need) and floor < kt.taper_floor(v, 0, 0, False)  # lead floor binds
  outs = [step(t, accel=-1.2, v=v, lead=lead, lead_visible=False) for _ in range(30)]  # 0.081 m, inside the budget
  assert t.phase == TaperPhase.taper and outs[0] > -1.2  # genuine entry, output modified
  assert all(-1.2 - 1e-12 <= o <= floor + 1e-9 for o in outs)  # never weaker than the lead need, never stronger
  assert outs[-1] == pytest.approx(floor)


@pytest.mark.parametrize("leave", [{"accel": 0.0, "a_target": 0.0}, {"accel": 0.2, "a_target": 0.2}, {"v": 0.3}])
def test_leaving_window_ends_episode_until_rearm(leave):
  t = armed()
  for _ in range(30):
    step(t, v=0.2)
  assert t.phase == TaperPhase.taper and t.out > -1.0
  args = {"v": 0.2} | leave
  assert step(t, **args) == args.get("accel", -1.0) and t.reason == "left_window" and t.phase == TaperPhase.done
  for _ in range(300):  # back inside the window in the same stop: original, no fresh budget/time
    assert step(t, v=0.2) == -1.0
  step(t, v=kt.REARM_SPEED)
  assert t.armed and t.phase == TaperPhase.idle
  step(t, v=0.2)
  assert t.phase == TaperPhase.taper


def test_window_upper_boundary_exact_and_nextafter():
  # exact TAPER_SPEED is outside (strict <), the next float below is inside
  t = armed()
  assert step(t, v=kt.TAPER_SPEED) == -1.0 and t.phase == TaperPhase.idle
  t = armed()
  assert step(t, v=math.nextafter(kt.TAPER_SPEED, 0.0)) > -1.0 and t.phase == TaperPhase.taper
  t = armed()
  step(t, v=0.2)
  assert t.phase == TaperPhase.taper
  assert step(t, v=kt.TAPER_SPEED) == -1.0 and t.reason == "left_window" and t.phase == TaperPhase.done


def test_oscillating_original_cannot_restart_distance_budget():
  # A one-frame accel >= 0 every 0.4 s at constant 0.6 m/s used to start a new episode with a fresh budget each time.
  t = armed()
  tapered_travel = 0.0
  for i in range(400):
    a = 0.0 if i % 40 == 39 else -1.0
    out = step(t, accel=a, a_target=a, v=0.2)
    if out != a:
      tapered_travel += 0.2 * kt.DT
  assert tapered_travel > 0.0  # anti-vacuity: the taper did modify output
  assert tapered_travel <= 0.2 ** 2 / 2.0 + kt.DISTANCE_MARGIN


@pytest.mark.parametrize("seq", [
  [(0.0, True), (0.0, False), (0.0, True)],  # standstill flag drops for one frame at rest
  [(0.105, True)],  # recorded K9: vEgo 0.10-0.109 m/s at the standstill edge in 17 of 73 stops
])
def test_standstill_edge_flicker_and_lagging_vego_pass_original(seq):
  t = armed()
  for _ in range(30):
    step(t, v=0.2)
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
  base = stop_sequence(Rig(monkeypatch, taper=False, K9CreepMode=0))
  monkeypatch.undo()
  cand = stop_sequence(Rig(monkeypatch, K9CreepMode=3))
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
  assert mixed_sequence(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), K9CreepMode=3)).can == ref.can
  monkeypatch.undo()
  ref9 = mixed_sequence(Rig(monkeypatch, taper=False, module=base_cc))
  monkeypatch.undo()
  assert mixed_sequence(Rig(monkeypatch, K9CreepMode=3)).can != ref9.can  # the scenario really exercises the taper


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
  assert edge_sequence(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), K9CreepMode=3)).can == ref7.can
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
  cand = scc12(edge_sequence(Rig(monkeypatch, K9CreepMode=3)).can)
  assert len(base) == len(cand) and base != cand
  for (bs, bm, ba), (cs, cm, ca) in zip(base, cand, strict=True):
    assert (cs, cm) == (bs, bm) and ca >= ba - 0.011  # StopReq/ACCMode kept; aReq never stronger (2 LSB)


ORIG = pytest.approx(-1.2, abs=1e-6)  # original command after float32 CarControl storage


def _taper_until_active(rig):
  for _ in range(100):
    rig.step(-1.0, LCS.pid, 5.0)
  for i in range(_ENTRY_MAX_FRAMES):  # clean 1 m/s^2 deceleration until the actual taper entry
    accel, _ = rig.step(-1.2, LCS.stopping, max(1.4 - 0.01 * i, 0.2))
    if rig.cc.k9_stop_taper.phase == TaperPhase.taper and accel > -1.2:
      return i
  raise AssertionError("no taper entry within bounded frames")


_ENTRY_MAX_FRAMES = 130


class ReadSpy:
  """Counts CarController reads of K9CreepMode per step: the read frame is observed, not assumed."""
  def __init__(self, rig):
    self.rig, self.read_frames = rig, []
    get_int = rig.params.get_int

    def spy(key):
      if key == "K9CreepMode":
        self.read_frames.append(rig.cc.frame)
      return get_int(key)
    rig.params.get_int = spy

  def step(self, *args, **kw):
    n = len(self.read_frames)
    frame = self.rig.cc.frame
    accel, sent = self.rig.step(*args, **kw)
    return frame, len(self.read_frames) > n, accel, sent


def _stop_profile(n=400, pad=0, dv=0.01):
  """(1 s + pad frames) cruise at 5 m/s, then a clean stop from 1.4 m/s at a measured dv / DT m/s^2 (aEgo matches the
  speed slope) to standstill and hold (frame-indexed inputs)."""
  rows = [(-1.0, LCS.pid, 5.0, False, -1.0)] * (100 + pad)
  for i in range(n):
    v = max(1.4 - dv * i, 0.0)
    rows.append((-1.2, LCS.stopping, v, v == 0.0, -dv / kt.DT if v > 0.0 else 0.0))
  return rows


def _run(spy, rows, flip=None, value=None, when=None):
  """Step rows; set K9CreepMode to value immediately BEFORE or AFTER the controller step of frame flip."""
  out = []
  for row in rows:
    accel, state, v, standstill, a_ego = row
    frame = spy.rig.cc.frame
    if when == "before" and frame == flip:
      spy.rig.params.values["K9CreepMode"] = value
    out.append((*spy.step(accel, state, v, standstill=standstill, a_ego=a_ego),
                spy.rig.cc.k9_stop_taper_enabled, spy.rig.cc.k9_stop_taper.phase, spy.rig.cc.k9_stop_taper.armed))
    if when == "after" and frame == flip:
      spy.rig.params.values["K9CreepMode"] = value
  return out


def _reference_off(monkeypatch, rows):
  ref = Rig(monkeypatch, taper=False, K9CreepMode=0)
  for accel, state, v, standstill, a_ego in rows:
    ref.step(accel, state, v, standstill=standstill, a_ego=a_ego)
  return ref


def _assert_safe_vs_off(rig, ref):
  ka = [i for i, f in enumerate(ref.can) if any(a == 1057 for a, _, _ in f)]
  parser_r, parser_c = CANParser("hyundai_kia_generic", [("SCC12", 0)], 0), CANParser("hyundai_kia_generic", [("SCC12", 0)], 0)
  for i in ka:
    parser_r.update([i * 10_000_000, [m for m in ref.can[i] if m[0] == 1057]])
    parser_c.update([i * 10_000_000, [m for m in rig.can[i] if m[0] == 1057]])
    r, c = parser_r.vl["SCC12"], parser_c.vl["SCC12"]
    assert (c["StopReq"], c["ACCMode"]) == (r["StopReq"], r["ACCMode"]) and c["aReqValue"] >= r["aReqValue"] - 0.011


# Production reads K9CreepMode when CarController.frame % 50 == 0 (asserted from the observed reads). With a measured
# 0.4 m/s^2 final deceleration this stop naturally tapers on frames 381..449 and restores on 450..493 (unpadded), long
# enough that both a read and the next read can fall inside one episode. Each case is padded so that the frame where
# OFF actually takes effect (the flip read itself BEFORE, the next read AFTER) is inside the stays-ON control's
# taper/restore with a modified output, so natural completion cannot be mistaken for the switch effect.
OFF_CASES = {  # (phase, when): (cruise pad, frame of the read the flip is adjacent to)
  (TaperPhase.taper, "before"): (10, 400),  # taper 391..459: flip read 400, effect 400
  (TaperPhase.taper, "after"): (10, 400),  # flip right after read 400 (taper), effect at read 450 (taper)
  (TaperPhase.restore, "before"): (30, 500),  # restore 480..523: flip read 500, effect 500
  (TaperPhase.restore, "after"): (30, 450),  # flip right after read 450 (taper 411..479), effect at read 500 (restore)
}
OFF_DV = 0.004


@pytest.mark.parametrize("when", ["before", "after"])
@pytest.mark.parametrize("phase", [TaperPhase.taper, TaperPhase.restore], ids=["taper", "restore"])
def test_live_switch_off_at_exact_read_frame(monkeypatch, when, phase):
  pad, flip = OFF_CASES[(phase, when)]
  rows = _stop_profile(n=500, pad=pad, dv=OFF_DV)
  assert all(r[4] == pytest.approx(-(r_prev[2] - r[2]) / kt.DT) for r_prev, r in zip(rows[100 + pad:], rows[101 + pad:])
             if r[2] > 0.0)  # measured aEgo is the actual speed slope, no held/unrealizable state
  ref = Rig(monkeypatch, taper=False, K9CreepMode=0)
  ref_accel = [ref.step(a, s, v, standstill=ss, a_ego=ae)[0] for a, s, v, ss, ae in rows]
  monkeypatch.undo()

  def run(flip_frame):
    rig = Rig(monkeypatch, K9CreepMode=3)
    spy = ReadSpy(rig)
    trace = []
    for frame, row in enumerate(rows):
      accel, state, v, standstill, a_ego = row
      if when == "before" and frame == flip_frame:
        rig.params.values["K9CreepMode"] = 0
      f, did, out, sent = spy.step(accel, state, v, standstill=standstill, a_ego=a_ego)
      t = rig.cc.k9_stop_taper
      trace.append({"frame": f, "read": did, "accel": out, "stop_req": None if sent is None else sent["StopReq"],
                    "enabled": rig.cc.k9_stop_taper_enabled, "phase": t.phase, "reason": t.reason})
      if when == "after" and frame == flip_frame:
        rig.params.values["K9CreepMode"] = 0
    monkeypatch.undo()
    return rig, spy, trace

  ctl, ctl_spy, ctl_tr = run(None)  # matched control: same inputs, switch never touched (stays ON)
  rig, spy, tr = run(flip)
  reads = list(range(0, len(rows), 50))
  assert [e["frame"] for e in tr if e["read"]] == reads == spy.read_frames == ctl_spy.read_frames  # every 50 frames
  assert tr[flip]["read"]  # the flip is adjacent to an actual read: before its step, or right after it
  effective = flip if when == "before" else flip + 50
  assert tr[effective]["read"] and not any(tr[f]["read"] for f in range(flip + 1, effective))

  # counterfactual: at the effect frame the stays-ON control is still in the requested phase, not finished by
  # distance_budget/no_progress or any other end reason, and still sends a modified (weaker) accel
  c = ctl_tr[effective]
  assert c["enabled"] and c["phase"] == phase and c["reason"] == phase.name
  assert c["accel"] > ref_accel[effective] + 1e-3
  assert c["reason"] not in ("distance_budget", "no_progress")
  if when == "after":  # the flip read itself is inside the episode too (taper), and the control stays active between
    assert ctl_tr[flip]["phase"] == TaperPhase.taper and ctl_tr[flip]["accel"] > ref_accel[flip] + 1e-3
    assert all(ctl_tr[f]["phase"] in (TaperPhase.taper, TaperPhase.restore) for f in range(flip, effective + 1))

  # before the effect frame the switched rig is exactly the control (AFTER: stale cached ON, value already False)
  for f in range(effective):
    assert (tr[f]["accel"], tr[f]["phase"], tr[f]["enabled"]) == (ctl_tr[f]["accel"], ctl_tr[f]["phase"], True)
    assert rig.can[f] == ctl.can[f]
    if when == "after" and f > flip:
      assert rig.params.values["K9CreepMode"] == 0
  # first restored output: exactly the effect read, caused by the switch (reset), not a natural end reason
  first = next(f for f in range(len(rows)) if tr[f]["accel"] != ctl_tr[f]["accel"])
  assert first == effective
  e = tr[effective]
  assert not e["enabled"] and e["phase"] == TaperPhase.idle and e["reason"] == "init"
  assert e["accel"] == ORIG and e["accel"] == ref_accel[effective]
  # from the effect frame on: original accel, CAN identical to the untouched OFF reference (StopReq included)
  for f in range(effective, len(rows)):
    assert tr[f]["accel"] == ref_accel[f] and rig.can[f] == ref.can[f]
  sent = [f for f in range(effective, len(rows)) if tr[f]["stop_req"] is not None]
  assert sent and all(tr[f]["stop_req"] == ctl_tr[f]["stop_req"] for f in sent)
  assert any(ctl_tr[f]["accel"] > ref_accel[f] + 1e-3 for f in sent)  # control still modifies while switched is OFF
  _assert_safe_vs_off(rig, ref)
  _assert_safe_vs_off(ctl, ref)
  assert rig.step(1.0, LCS.pid, 0.0, standstill=True)[0] == pytest.approx(1.0)


STALE_ON_RESTORE = (45, 500)  # (cruise pad, read): natural restore starts before read 500 and finishes before 550


def test_live_switch_off_after_read_stale_on_restore_completes_naturally_before_next_read(monkeypatch):
  pad, read = STALE_ON_RESTORE
  rows = _stop_profile(n=500, pad=pad, dv=OFF_DV)
  ref = Rig(monkeypatch, taper=False, K9CreepMode=0)
  ref_accel = [ref.step(a, s, v, standstill=ss, a_ego=ae)[0] for a, s, v, ss, ae in rows]
  monkeypatch.undo()

  def run(flip):
    rig = Rig(monkeypatch, K9CreepMode=3)
    spy = ReadSpy(rig)
    tr = []
    for frame, (accel, state, v, standstill, a_ego) in enumerate(rows):
      f, did, out, sent = spy.step(accel, state, v, standstill=standstill, a_ego=a_ego)
      t = rig.cc.k9_stop_taper
      tr.append({"read": did, "accel": out, "stop_req": None if sent is None else sent["StopReq"],
                 "enabled": rig.cc.k9_stop_taper_enabled, "phase": t.phase, "reason": t.reason})
      if frame == flip:
        rig.params.values["K9CreepMode"] = 0  # right AFTER the actual read: cached value stays ON until read + 50
    monkeypatch.undo()
    return rig, spy, tr

  ctl, ctl_spy, ctl_tr = run(None)  # matched control: same inputs, switch stays ON for the whole stop
  rig, spy, tr = run(read)
  assert spy.read_frames == ctl_spy.read_frames == list(range(0, len(rows), 50)) and tr[read]["read"]
  # until the next read the switched rig is frame-for-frame the stays-ON control (accel, phase, enabled, CAN)
  for f in range(read + 50):
    assert (tr[f]["accel"], tr[f]["phase"], tr[f]["reason"], tr[f]["enabled"]) == \
           (ctl_tr[f]["accel"], ctl_tr[f]["phase"], ctl_tr[f]["reason"], True) and rig.can[f] == ctl.can[f]
  restore = [f for f, e in enumerate(tr) if e["phase"] == TaperPhase.restore]
  start, end = restore[0], restore[-1]
  assert restore == list(range(start, end + 1)) and start < read <= end < read + 50  # flip lands inside restore
  assert tr[start - 1]["phase"] == TaperPhase.taper
  ctl_restore = [f for f, e in enumerate(ctl_tr) if e["phase"] == TaperPhase.restore]
  assert ctl_restore == restore and ctl_tr[end + 1]["phase"] == TaperPhase.done and ctl_tr[end + 1]["reason"] == "restored"
  # restore length derived from code constants for THIS scenario's actual restore delta (not a universal bound):
  # it must be < 50 frames (one read period) so that a flip anywhere in it still ends naturally before the next read
  n_restore = math.ceil((tr[start]["accel"] - ref_accel[start]) / (kt.REAPPLY_RATE * kt.DT) - 1e-6)
  assert n_restore == end - start + 1 < 50
  # derived duration: first restore output above the original, reapplied at REAPPLY_RATE * DT per frame
  assert end - start + 1 == math.ceil((tr[start]["accel"] - ref_accel[start]) / (kt.REAPPLY_RATE * kt.DT) - 1e-6)
  for f in range(start + 1, end + 1):
    assert tr[f]["accel"] == pytest.approx(max(tr[f - 1]["accel"] - kt.REAPPLY_RATE * kt.DT, ref_accel[f]), abs=1e-6)
  # natural completion by the stale cached ON before the next read, not the switch
  assert all(tr[f]["enabled"] for f in range(read + 1, read + 50)) and rig.params.values["K9CreepMode"] == 0
  assert tr[end + 1]["phase"] == TaperPhase.done and tr[end + 1]["reason"] == "restored"
  assert not any(tr[f]["read"] for f in range(read + 1, read + 50))
  for f in range(end + 1, read + 50):
    assert tr[f]["accel"] == ref_accel[f] == ORIG and tr[f]["phase"] == TaperPhase.done
  # next actual read: true OFF, full original, CAN identical to the OFF reference from natural completion on
  e = tr[read + 50]
  assert e["read"] and not e["enabled"] and e["phase"] == TaperPhase.idle and e["accel"] == ref_accel[read + 50]
  for f in range(end + 1, len(rows)):
    assert tr[f]["accel"] == ref_accel[f] and rig.can[f] == ref.can[f]
  for f in range(len(rows)):
    assert tr[f]["accel"] >= ref_accel[f] - 1e-6  # never stronger
  sent = [f for f in range(len(rows)) if tr[f]["stop_req"] is not None]
  assert sent and any(start <= f <= end for f in sent)
  _assert_safe_vs_off(rig, ref)  # StopReq/ACCMode invariant over the whole stop
  _assert_safe_vs_off(ctl, ref)
  for f in range(len(rows)):
    assert ctl_tr[f]["accel"] >= ref_accel[f] - 1e-6

def test_live_switch_off_mid_taper_original_no_later_than_next_read_natural_or_switch(monkeypatch):
  # Restored former short stale-ON taper check, renamed honestly: in this scene the taper may end naturally (stale
  # cached ON) before the switch takes effect, so it does NOT prove the switch caused the restore; it proves that,
  # either way, output is original by the next read and never stronger, matched to a stays-ON control until then.
  def run(flip):
    rig = Rig(monkeypatch, K9CreepMode=3)
    spy = ReadSpy(rig)
    _taper_until_active(rig)
    if flip:
      rig.params.values["K9CreepMode"] = 0
    start = rig.cc.frame
    outs = [spy.step(-1.2, LCS.stopping, 0.3) for _ in range(51)]
    monkeypatch.undo()
    return rig, start, outs

  ctl, c_start, c_outs = run(False)
  rig, start, outs = run(True)
  assert start == c_start
  next_read = next(i for i, (_, did, _, _) in enumerate(outs) if did)
  assert 0 <= next_read <= 50 and (start + next_read) % 50 == 0
  for i in range(next_read):  # stale cached ON: identical to the stays-ON control until the next read
    assert outs[i][2] == c_outs[i][2] and rig.can[start + i] == ctl.can[start + i]
  first_original = next(i for i, o in enumerate(outs) if o[2] == ORIG)
  assert first_original <= next_read and all(o[2] == ORIG for o in outs[first_original:])
  assert all(o[2] >= -1.2 - 1e-6 and (o[3] is None or o[3]["aReqValue"] == pytest.approx(o[2], abs=0.006)) for o in outs)
  assert rig.cc.k9_stop_taper.phase == TaperPhase.idle and not rig.cc.k9_stop_taper_enabled


ON_READ = 150  # v = 0.9 m/s: below REARM_SPEED, above TAPER_SPEED, before standstill


@pytest.mark.parametrize("when", ["before", "after"])
def test_live_switch_on_at_exact_read_frame_waits_for_true_rearm(monkeypatch, when):
  rows = _stop_profile()
  ref = _reference_off(monkeypatch, rows)
  monkeypatch.undo()
  rig = Rig(monkeypatch, taper=False, K9StopTaper=False, K9CreepMode=0)
  spy = ReadSpy(rig)
  out = _run(spy, rows, flip=ON_READ, value=3, when=when)
  by = {f: o for f, *o in out}
  assert [f for f, did, *_ in out if did] == list(range(0, len(rows), 50))
  assert rows[ON_READ][2] == pytest.approx(0.9) and kt.TAPER_SPEED < rows[ON_READ][2] < kt.REARM_SPEED
  effective = ON_READ if when == "before" else ON_READ + 50
  assert by[effective - 1][3] is False and by[effective][3] is True  # stale OFF until exactly this read
  for f in range(len(rows)):  # rest of this stop (taper window, standstill): original, never armed, CAN as OFF
    assert by[f][1] == ORIG or f < 100
    assert not by[f][5] and by[f][4] == TaperPhase.idle
  assert rig.can == ref.can[:len(rig.can)] and len(rig.can) == len(rows)
  _assert_safe_vs_off(rig, ref)
  # true re-arm: depart and move at >= REARM_SPEED, then the next stop actually tapers
  out2 = _run(spy, _stop_profile())
  assert any(o[5] == TaperPhase.taper and o[2] > -1.2 + 1e-3 for o in out2)


def test_setting_registered_in_experimental_menu_default_off():
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  driving = next(m for m in catalog["menu"] if m["id"] == "DRIVING")
  experimental = next(g for g in driving["groups"] if g["id"] == "EXPERIMENTAL")
  # owner 2026-10-21: the visible Korean group label is 실험중; the internal id stays EXPERIMENTAL
  assert experimental["ko"] == "실험중" and experimental["params"] == ["K9CreepMode", "K9LaneFix"]
  assert driving["ko"] == "주행 제어"
  entry = next(p for p in catalog["params"] if p["name"] == "K9StopTaper")
  assert (entry["min"], entry["max"], entry["default"], entry["unit"]) == (0, 1, 0, 1)
  # owner 2026-10-22: the unread K9StopTaper toggle is out of every menu (search_only), only the param name is kept
  assert "K9StopTaper" not in json.dumps(catalog["menu"]) and entry["search_only"] is True
  keys = (ROOT / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
  assert '{"K9StopTaper", {PERSISTENT, BOOL, "0"}},' in keys
