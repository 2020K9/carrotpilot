"""K9StopSoftBrake synthetic closed loop: LongControl -> cruise helper (card) -> K9 CarController -> decoded SCC12.

Params and the bluetooth reader are in memory. Checks transmitted aReq/StopReq/ACCMode only; this is not a
vehicle-response, rolling or hill-holding result.
"""
import math
import time
from types import SimpleNamespace

import pytest

from openpilot.cereal import car, log
import openpilot.selfdrive.car.cruise as cruise
import openpilot.selfdrive.controls.lib.longcontrol as longcontrol
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.k9_stop_soft_brake import (
  K9_SOFT_BRAKE_SPEED as SPEED_PARAM, K9StopSoftBrake, k9_stop_soft_brake_applies, read_k9_stop_soft_brake,
  read_k9_stop_soft_brake_speed,
)
from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags

KPH = 1 / 3.6
K9_SOFT_BRAKE_SPEED = SPEED_PARAM[1] / 3.6  # default entry speed, m/s
# Stored values of the removed StopEase trial (StopEaseAccel=-20 as set on the vehicle): must have no effect.
LEFTOVER = {"StopEaseEnable": 1, "StopEaseDist": 80, "StopEaseSpeed": 30, "StopEaseAccel": -20}
K9_FLAGS = int(HyundaiFlags.CAMERA_SCC | HyundaiFlags.LEGACY)
LCS_NAMES = {v: k for k, v in car.CarControl.Actuators.LongControlState.schema.enumerants.items()}
TOL = 0.006
ACCEL_MIN, ACCEL_MAX = CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX  # aReqValue is sent in 0.01 m/s^2 steps


class MemoryParams:
  def __init__(self, values):
    self.values = values

  def get(self, key):
    return self.values.get(key)

  def get_int(self, key):
    return int(self.values.get(key, 0))

  def get_float(self, key):
    return float(self.values.get(key, 0))

  def get_bool(self, key):
    return bool(self.values.get(key, False))

  def put_int_nonblocking(self, key, value):
    self.values[key] = int(value)

  def put_bool_nonblocking(self, key, value):
    self.values[key] = bool(value)

  put_bool = put_bool_nonblocking


class NullReader:
  def __init__(self, *args, **kwargs):
    self.is_repeat = False

  def read(self, **kwargs):
    return None


class SubMaster(dict):
  def __init__(self, radar_valid=True, **services):
    super().__init__(**services)
    self.alive = {"carrotMan": False, "longitudinalPlan": False, "radarState": True, "drivingModelData": False}
    self.valid = {"carrotMan": False, "radarState": radar_valid}
    self.seen = {"carrotMan": False}
    self.recv_time = {"carrotMan": time.monotonic()}


def make_cp(fingerprint=CAR.KIA_K9, flags=K9_FLAGS, long_control=True):
  return structs.CarParams(brand="hyundai", carFingerprint=fingerprint, flags=flags, openpilotLongitudinalControl=long_control,
                           pcmCruise=False, wheelbase=3.15, steerRatio=14.5, stopAccel=-2.0, stoppingDecelRate=0.8,
                           vEgoStarting=0.1, startingState=False,
                           longitudinalTuning=structs.CarParams.LongitudinalPIDTuning(kpBP=[0.], kpV=[1.], kiBP=[0.], kiV=[0.], kf=1.))


def make_ci_state():
  parser = CANParser("hyundai_kia_generic", [("SCC11", 0), ("SCC12", 0), ("SCC13", 0), ("SCC14", 0)], 2)
  return SimpleNamespace(out=structs.CarState(), modelV2=None, is_metric=True, lkas11=None, clu11=None, mdps12=None, fca11=None,
                         paddle_button_prev=0, softHoldActive=0,
                         scc11=dict(parser.vl["SCC11"]), scc12=dict(parser.vl["SCC12"]), scc13=dict(parser.vl["SCC13"]),
                         scc14=dict(parser.vl["SCC14"]))


class Loop:
  """controlsd LongControl output (previous frame) -> card cruise helper and CarController, like card.step()."""
  def __init__(self, monkeypatch, hold_time=20, strength=20, cp=None, **extra):
    self.params = MemoryParams({"K9StopSoftBrakeTime": hold_time, "K9StopSoftBrakeAccel": strength, "AutoCruiseControl": 1,
                                "HyundaiCameraSCC": 1, "StoppingAccel": -50, "CruiseSpeedUnit": 10, "CruiseSpeedUnitBasic": 1,
                                "CruiseButtonLongDelay": 50, "AutoGasTokSpeed": 30, "AutoGasCancelSpeed": 30, **extra})
    for module in (cruise, longcontrol, carcontroller):
      monkeypatch.setattr(module, "Params", lambda *args, **kwargs: self.params)
    monkeypatch.setattr(cruise, "CommandReader", NullReader)
    self.CP = make_cp() if cp is None else cp
    self.helper = cruise.VCruiseCarrot(self.CP)
    self.helper.cruise_state_available_last = True
    self.helper._gas_pressed_count = self.helper._brake_pressed_count = -2
    self.controller = carcontroller.CarController({Bus.pt: "hyundai_kia_generic"}, self.CP)
    self.lc = longcontrol.LongControl(self.CP)
    self.parser = CANParser("hyundai_kia_generic", [("SCC12", 0)], 0)
    self.cc = None
    self.frame = 0
    self.rows = []

  def make_cc(self, enabled, override, accel, a_target, state):
    state = state if isinstance(state, str) else LCS_NAMES[int(state)]
    cc = car.CarControl.new_message(enabled=enabled, longActive=enabled and not override)
    cc.cruiseControl.override = override
    cc.actuators.accel, cc.actuators.aTarget, cc.actuators.longControlState = float(accel), float(a_target), state
    st = structs.CarControl(enabled=enabled, longActive=enabled and not override)
    st.cruiseControl.override = override
    st.actuators.accel, st.actuators.aTarget = float(accel), float(a_target)
    st.actuators.longControlState = state
    return cc.as_reader(), st.as_reader()

  def step(self, v=0., standstill=False, a_target=0., should_stop=True, lead=None, radar_valid=True, can_valid=True,
           gas=False, brake=False, enabled=True, override=False):
    if self.cc is None:
      self.cc = self.make_cc(False, False, 0., 0., "off")
    cs = car.CarState.new_message(vEgo=v, vEgoCluster=v, vEgoRaw=v, standstill=standstill, gasPressed=gas, brakePressed=brake,
                                  canValid=can_valid, gearShifter="drive",
                                  cruiseState={"available": True, "enabled": enabled, "standstill": False})
    radar = log.RadarState.new_message()
    if lead is not None:  # (dRel, vRel)
      radar.leadOne.status, radar.leadOne.dRel, radar.leadOne.vRel = True, lead[0], lead[1]
    radar = radar.as_reader()
    cc_capnp, cc_structs = self.cc

    # card: carState and cruise helper, then CarController with the latest carControl
    self.helper.update_v_cruise(cs, SubMaster(radar_valid, carControl=cc_capnp, radarState=radar), True)
    cs.softHoldActive = self.helper._soft_hold_active
    ci_cs = make_ci_state()
    ci_cs.out = structs.CarState(vEgo=v, vEgoRaw=v, standstill=standstill, gasPressed=gas, brakePressed=brake)
    ci_cs.softHoldActive = cs.softHoldActive
    if self.helper.k9_stop_soft_brake is not None:
      ci_cs.k9_stop_soft_brake_accel = self.helper.k9_stop_soft_brake_accel
    _, sends = self.controller.update(cc_structs, ci_cs, self.frame * 10_000_000)
    sent = None
    scc12 = [m for m in sends if m[0] == 1057]
    if scc12:
      self.parser.update([self.frame * 10_000_000, scc12])
      sent = dict(self.parser.vl["SCC12"])
    sb = self.helper.k9_stop_soft_brake
    self.rows.append(dict(frame=self.frame, v=v, standstill=standstill, sent=sent, lead=lead, cc_accel=cc_structs.actuators.accel,
                          a_target=cc_structs.actuators.aTarget, lcs=str(cc_structs.actuators.longControlState),
                          soft=self.helper.k9_stop_soft_brake_accel, soft_hold=cs.softHoldActive,
                          frames=sb.frames if sb is not None else 0, own_hold=sb.hold if sb is not None else False))

    # controlsd: next carControl
    plan = log.LongitudinalPlan.new_message(aTarget=a_target, vTargetNow=0. if should_stop else v + 1., shouldStop=should_stop)
    accel, a_t, _ = self.lc.update(enabled and not override, cs.as_reader(), plan.as_reader(), (-3.5, 2.0), 0., radar)
    self.cc = self.make_cc(enabled, override, accel, a_t, self.lc.long_control_state)
    self.frame += 1
    return self.rows[-1]

  def run(self, n, **kwargs):
    return [self.step(**kwargs) for _ in range(n)]

  def approach(self, lead=None, start=3.):
    """Moving stop: start (default 3) -> 0.3 km/h without standstill (2 s), as the stopping state is entered."""
    for i in range(200):
      self.step(v=max(0.3, start - start / 200 * i) * KPH, a_target=-0.1, lead=lead)


def need_accel(lead):
  """Independent copy of the required formula: -2 * closing^2 / (2 * max(dRel - 1.5, 0.05)); 0 without a lead."""
  if lead is None:
    return 0.0
  closing = max(0.0, -lead[1])
  return -2.0 * closing ** 2 / (2.0 * max(lead[0] - 1.5, 0.05))


def selected(r, strength):
  """Strongest of -strength, plan and lead need, within the existing CarController accel limits."""
  return max(ACCEL_MIN, min(ACCEL_MAX, min(-strength, r["a_target"], need_accel(r["lead"]))))


def strong_plan_weakened(rows, strength=0.2):
  """Frames where this trial was active and the sent aReq (Value or Raw) is weaker than the strongest selected decel."""
  return [r for r in rows if r["soft"] is not None and r["sent"] is not None and
          max(r["sent"]["aReqValue"], r["sent"]["aReqRaw"]) > selected(r, strength) + TOL]


def sent(rows):
  return [r for r in rows if r["sent"] is not None]


def check_common(loop, strength=0.2):
  assert strong_plan_weakened(loop.rows, strength) == []
  for r in sent(loop.rows):
    if r["soft"] is not None:
      assert r["sent"]["aReqValue"] == pytest.approx(selected(r, strength), abs=TOL)
    else:
      assert r["sent"]["aReqValue"] == pytest.approx(max(ACCEL_MIN, min(ACCEL_MAX, r["cc_accel"])), abs=TOL)


def test_gating_and_params():
  assert k9_stop_soft_brake_applies(make_cp())
  assert not k9_stop_soft_brake_applies(make_cp(long_control=False))
  assert not k9_stop_soft_brake_applies(make_cp(flags=int(HyundaiFlags.LEGACY)))
  assert not k9_stop_soft_brake_applies(make_cp(flags=K9_FLAGS | int(HyundaiFlags.CANFD)))
  assert not k9_stop_soft_brake_applies(make_cp(fingerprint=CAR.KIA_K7))
  read = lambda t, a: read_k9_stop_soft_brake(MemoryParams({"K9StopSoftBrakeTime": t, "K9StopSoftBrakeAccel": a}))  # noqa: E731
  assert read(0, 20) == (0, pytest.approx(-0.2))
  assert read(5, 5) == (50, pytest.approx(-0.05))
  assert read(50, 50) == (500, pytest.approx(-0.5))
  assert read(7, 99) == (50, pytest.approx(-0.5))  # 0.5 s steps, clamped strength
  assert read(99, -3) == (500, pytest.approx(-0.05))
  assert read(-5, 20)[0] == 0


@pytest.mark.parametrize("cp", [make_cp(fingerprint=CAR.KIA_K7), make_cp(flags=int(HyundaiFlags.LEGACY)), make_cp(long_control=False)])
def test_other_cars_never_use_it(monkeypatch, cp):
  loop = Loop(monkeypatch, cp=cp)
  assert loop.helper.k9_stop_soft_brake is None and not loop.controller.stop_soft_brake
  loop.approach()
  loop.run(500, standstill=True)
  assert all(r["soft"] is None and r["soft_hold"] == 0 for r in loop.rows)


def test_time_zero_is_off(monkeypatch):
  loop = Loop(monkeypatch, hold_time=0)
  loop.approach()
  loop.run(800, standstill=True)
  assert all(r["soft"] is None and r["soft_hold"] == 0 and r["frames"] == 0 for r in loop.rows)
  check_common(loop)


def test_sequence_weak_decel_then_existing_hold(monkeypatch):
  loop = Loop(monkeypatch, hold_time=20)
  loop.approach()
  # Before the stop the timer never runs, however long the car crawls below 2 km/h.
  loop.run(1000, v=0.5 * KPH, a_target=-0.1)
  moving = loop.rows[:1200]
  assert all(r["frames"] == 0 and r["soft_hold"] == 0 for r in moving)
  assert all((r["soft"] is not None) == (r["v"] < K9_SOFT_BRAKE_SPEED and r["lcs"] == "stopping") for r in moving[1:])
  assert any(r["soft"] is not None for r in moving)
  assert all(r["sent"]["aReqValue"] == pytest.approx(-0.2, abs=TOL) for r in sent(moving[300:]))
  # At or above 2 km/h the original command is sent.
  assert all(r["soft"] is None for r in moving if r["v"] >= K9_SOFT_BRAKE_SPEED)

  wait = loop.run(200, standstill=True, a_target=-0.1)
  assert [r["frames"] for r in wait[:199]] == list(range(1, 200))
  assert all(r["soft"] == pytest.approx(-0.2) and r["soft_hold"] == 0 for r in wait[:199])
  assert all(r["sent"]["aReqValue"] == pytest.approx(-0.2, abs=TOL) and r["sent"]["StopReq"] == 1 for r in sent(wait[:199]))
  # At exactly 2 s of standstill: the existing soft hold, no weak decel any more.
  assert wait[199]["soft_hold"] == 2 and wait[199]["soft"] is None and wait[199]["own_hold"]
  hold = loop.run(300, standstill=True, a_target=-0.1)
  assert all(r["soft_hold"] == 2 and r["soft"] is None for r in hold)
  # Existing soft hold path: LongControl uses stopAccel and its ramp, StopReq=1 with long enabled.
  assert all(r["sent"]["StopReq"] == 1 and r["sent"]["ACCMode"] == 1 for r in sent(hold))
  assert hold[-1]["cc_accel"] == pytest.approx(-2.0) and sent(hold)[-1]["sent"]["aReqValue"] == pytest.approx(-2.0, abs=TOL)
  check_common(loop)


@pytest.mark.parametrize("kph", [0.4, 1.5])
def test_moving_while_waiting_resets_and_restarts(monkeypatch, kph):
  loop = Loop(monkeypatch, hold_time=20)
  loop.approach()
  loop.run(150, standstill=True, a_target=-0.1)
  moved = loop.run(50, v=kph * KPH, a_target=-0.1)  # standstill off, still below 2 km/h
  assert all(r["frames"] == 0 and r["soft"] is None and r["soft_hold"] == 0 for r in moved)
  for r in sent(moved):
    assert r["sent"]["aReqValue"] == pytest.approx(r["cc_accel"], abs=TOL)  # original control
  again = loop.run(250, standstill=True, a_target=-0.1)
  # 150 + 50 frames would already exceed 2 s; the new standstill needs a full 2 s from zero.
  assert [r["frames"] for r in again[:199]] == list(range(1, 200))
  assert all(r["soft_hold"] == 0 for r in again[:199]) and again[199]["soft_hold"] == 2
  check_common(loop)


@pytest.mark.parametrize("pedal", ["gas", "brake"])
@pytest.mark.parametrize("during", ["wait", "hold"])
def test_pedal_clears_and_never_rearms(monkeypatch, pedal, during):
  loop = Loop(monkeypatch, hold_time=20)
  loop.approach()
  loop.run(100 if during == "wait" else 300, standstill=True, a_target=-0.1)
  assert loop.rows[-1]["soft_hold"] == (0 if during == "wait" else 2)
  # A short press (0.3 s); the controls disengage on a pedal at standstill.
  pressed = loop.run(30, standstill=True, enabled=False, **{pedal: True})
  assert all(r["soft_hold"] == 0 and r["soft"] is None and r["frames"] == 0 and not r["own_hold"] for r in pressed)
  # Released, re-enabled and still stopped: no weak decel, no new request from this trial.
  after = loop.run(600, standstill=True, a_target=-0.1)
  assert all(r["soft_hold"] == 0 and r["soft"] is None and r["frames"] == 0 for r in after)
  # It works again after the car has moved.
  loop.run(100, v=3 * KPH, a_target=-0.1)
  loop.run(50, v=0.5 * KPH, a_target=-0.1)
  assert loop.run(200, standstill=True, a_target=-0.1)[-1]["soft_hold"] == 2
  check_common(loop)


def test_cruise_off_clears_own_hold(monkeypatch):
  loop = Loop(monkeypatch, hold_time=10)
  loop.approach()
  loop.run(200, standstill=True, a_target=-0.1)
  assert loop.rows[-1]["soft_hold"] == 2
  off = loop.run(100, standstill=True, enabled=False)
  # off[0] still carries the previous (enabled) carControl, as in card.
  assert all(r["soft_hold"] == 0 and r["soft"] is None and not r["own_hold"] for r in off[1:])
  # No long enable / StopReq from a remaining hold request.
  assert all(r["sent"]["StopReq"] == 0 and r["sent"]["ACCMode"] == 0 for r in sent(off[2:]))
  after = loop.run(400, standstill=True, a_target=-0.1)
  assert all(r["soft_hold"] == 0 and r["soft"] is None for r in after)
  check_common(loop)


def test_override_during_wait(monkeypatch):
  loop = Loop(monkeypatch, hold_time=20)
  loop.approach()
  loop.run(100, standstill=True, a_target=-0.1)
  ov = loop.run(20, standstill=True, override=True, a_target=-0.1)
  assert all(r["soft"] is None and r["frames"] == 0 for r in ov[1:])
  after = loop.run(400, standstill=True, a_target=-0.1)
  assert all(r["soft_hold"] == 0 and r["soft"] is None for r in after)
  check_common(loop)


def override_release_cruise_cycle(loop):
  """At standstill: 30 frames override without a pedal, 50 released, 100 cruise off, 400 cruise on again."""
  return (loop.run(30, standstill=True, override=True, a_target=-0.1), loop.run(50, standstill=True, a_target=-0.1),
          loop.run(100, standstill=True, enabled=False), loop.run(400, standstill=True, a_target=-0.1))


def test_override_without_pedal_during_hold_clears_own_hold(monkeypatch):
  """Own hold, then override without gas/brake, release, cruise off and on: the own hold is really cleared, never
  requested again, and the final sends match the path without this trial's hold (time 0)."""
  finals = []
  for hold_time in (0, 10):
    loop = Loop(monkeypatch, hold_time=hold_time)
    loop.approach()
    loop.run(200, standstill=True, a_target=-0.1)
    assert loop.rows[-1]["soft_hold"] == (2 if hold_time else 0)
    ov, released, off, on = override_release_cruise_cycle(loop)
    # ov[0] still carries the previous carControl (no override yet), as in card.
    assert all(r["soft_hold"] == 0 and r["soft"] is None and r["frames"] == 0 and not r["own_hold"]
               for r in ov[1:] + released + off + on)
    # No long enable / StopReq from a remaining hold request while cruise is off.
    assert all(r["sent"]["StopReq"] == 0 and r["sent"]["ACCMode"] == 0 for r in sent(off[2:]))
    check_common(loop)
    finals.append([(r["soft_hold"], r["soft"], r["cc_accel"], r["sent"] and (r["sent"]["StopReq"], r["sent"]["ACCMode"],
                    r["sent"]["aReqValue"], r["sent"]["aReqRaw"])) for r in off[2:] + on])
  assert finals[0] == finals[1]


def test_manual_soft_hold_unchanged_by_override_without_pedal(monkeypatch):
  """Existing manual soft hold, then override without a pedal and the cruise cycle: identical with this trial on/off."""
  traces = []
  for hold_time in (0, 20):
    loop = Loop(monkeypatch, hold_time=hold_time)
    loop.approach()
    loop.run(100, standstill=True, a_target=-0.1)
    loop.run(150, standstill=True, brake=True, enabled=False)
    loop.run(1, standstill=True, enabled=False)
    loop.run(100, standstill=True, a_target=-0.1)
    assert loop.rows[-1]["soft_hold"] == 2 and not loop.rows[-1]["own_hold"]
    override_release_cruise_cycle(loop)
    assert not any(r["own_hold"] for r in loop.rows)
    traces.append([(r["soft_hold"], r["sent"] and (r["sent"]["StopReq"], r["sent"]["ACCMode"], r["sent"]["aReqValue"]))
                   for r in loop.rows[300:]])
  assert traces[0] == traces[1]


def test_manual_soft_hold_unchanged(monkeypatch):
  """Existing manual soft hold (brake held >0.6 s at standstill, then released) with this trial on vs off."""
  traces = []
  for hold_time in (0, 20):
    loop = Loop(monkeypatch, hold_time=hold_time)
    loop.approach()
    loop.run(100, standstill=True, a_target=-0.1)
    loop.run(150, standstill=True, brake=True, enabled=False)
    loop.run(1, standstill=True, enabled=False)
    loop.run(400, standstill=True, a_target=-0.1)
    traces.append([(r["soft_hold"], r["sent"] and (r["sent"]["StopReq"], r["sent"]["ACCMode"], r["sent"]["aReqValue"]))
                   for r in loop.rows[300:]])
    assert loop.rows[-1]["soft_hold"] == 2 and not loop.rows[-1]["own_hold"]
  assert traces[0] == traces[1]


@pytest.mark.parametrize("leftover", [False, True])
def test_strong_plan_decel_always_wins(monkeypatch, leftover):
  loop = Loop(monkeypatch, hold_time=15, **(LEFTOVER if leftover else {}))
  # A steady lead at 6 m: its need decel is 0.
  for i in range(200):
    loop.step(v=max(0.3, 3. - 0.015 * i) * KPH, a_target=-0.1, lead=(6., 0.))
  plan = [-0.1, -0.25, -0.8, -1.5, -0.19, -0.6]
  for k in range(300):
    loop.step(v=0.5 * KPH if k < 100 else 0., standstill=k >= 100, a_target=plan[(k // 25) % len(plan)], lead=(6., 0.))
  strong = [r for r in sent(loop.rows) if r["soft"] is not None and r["a_target"] < -0.2]
  assert len(strong) > 50
  assert all(r["sent"]["aReqValue"] == pytest.approx(r["a_target"], abs=TOL) for r in strong)
  assert loop.rows[-1]["soft_hold"] == 2
  check_common(loop)


NAN, INF = float("nan"), float("inf")


@pytest.mark.parametrize("bad", [dict(radar_valid=False), dict(can_valid=False), dict(lead=(NAN, 0.)), dict(lead=(3., NAN)),
                                 dict(lead=(INF, -0.5)), dict(lead=(-1., 0.))])
def test_invalid_inputs_use_original_control(monkeypatch, bad):
  """Invalid radar/speed input, or a reported lead whose distance/relative speed is invalid (not "no lead")."""
  loop = Loop(monkeypatch, hold_time=20)
  loop.approach()
  loop.run(150, standstill=True, a_target=-0.1)
  rows = loop.run(300, standstill=True, a_target=-0.1, **bad)
  assert all(r["soft"] is None and r["frames"] == 0 and r["soft_hold"] == 0 for r in rows)
  for r in sent(rows[1:]):
    assert r["sent"]["aReqValue"] == pytest.approx(r["cc_accel"], abs=TOL)
  # Once the condition clears, timing restarts from zero.
  ok = loop.run(200, standstill=True, a_target=-0.1)
  assert [r["frames"] for r in ok[:199]] == list(range(1, 200)) and ok[199]["soft_hold"] == 2
  check_common(loop)


def test_departure_state_is_not_braked(monkeypatch):
  loop = Loop(monkeypatch, hold_time=50)
  loop.approach()
  loop.run(100, standstill=True, a_target=-0.1)
  go = loop.run(100, standstill=True, a_target=0.5, should_stop=False)
  assert all(r["soft"] is None for r in go[1:])
  assert all(r["lcs"] != "stopping" for r in go[1:])
  assert math.isclose(go[-1]["sent"]["aReqValue"] if go[-1]["sent"] else go[-2]["sent"]["aReqValue"], go[-1]["cc_accel"], abs_tol=TOL)
  check_common(loop)


@pytest.mark.parametrize("leftover", [False, True])
@pytest.mark.parametrize("plan", [-0.1, -1.0])
@pytest.mark.parametrize("d_rel", [1.4, 1.5, 1.6, 2.5, 3.9])
@pytest.mark.parametrize("v_rel", [0.0, -0.3, -1.0, 0.3])  # steady, approaching (slow/fast), receding
def test_close_lead_uses_strongest_of_strength_plan_and_need(monkeypatch, leftover, plan, d_rel, v_rel):
  loop = Loop(monkeypatch, hold_time=20, **(LEFTOVER if leftover else {}))
  loop.approach()
  lead = (d_rel, v_rel)
  near = loop.run(100, v=0.5 * KPH, a_target=plan, lead=lead) + loop.run(200, standstill=True, a_target=plan, lead=lead)
  # A lead within 4 m no longer disables the trial: weak decel throughout and the hold after 2 s of standstill.
  assert all(r["soft"] is not None for r in near[1:-1]) and near[-1]["soft_hold"] == 2
  need = need_accel(lead)
  expected = max(ACCEL_MIN, min(-0.2, plan, need))
  for r in sent(near[2:-1]):
    assert r["sent"]["aReqValue"] == pytest.approx(expected, abs=TOL)
    assert r["sent"]["aReqRaw"] == pytest.approx(expected, abs=TOL)
  if v_rel >= 0.0:
    assert need == 0.0  # steady or receding lead: strength or plan only
  if need < min(-0.2, plan):
    assert expected < plan  # the need decel overrides a weaker plan
  if plan < min(-0.2, need):
    assert expected == pytest.approx(plan)
  check_common(loop)


def test_need_formula_examples():
  assert need_accel((3.0, -1.0)) == pytest.approx(-2 * 1.0 / 3.0)
  assert need_accel((1.5, -0.3)) == pytest.approx(-1.8) == need_accel((1.4, -0.3))  # max(dRel - 1.5, 0.05)
  assert need_accel((1.6, -0.3)) == pytest.approx(-0.9)
  assert need_accel((2.0, 0.5)) == 0.0 and need_accel(None) == 0.0
  from opendbc.car.hyundai.k9_stop_soft_brake import (K9_SOFT_BRAKE_LEAD_GAP, K9_SOFT_BRAKE_MIN_GAP, K9_SOFT_BRAKE_NEED_GAIN,
                                                       k9_lead_need_accel)
  assert (K9_SOFT_BRAKE_LEAD_GAP, K9_SOFT_BRAKE_NEED_GAIN, K9_SOFT_BRAKE_MIN_GAP) == (1.5, 2.0, 0.05)
  for lead in [(3.0, -1.0), (1.5, -0.3), (1.4, -0.3), (1.6, -0.3), (2.0, 0.5), (0.0, -2.0)]:
    assert k9_lead_need_accel(SimpleNamespace(status=True, dRel=lead[0], vRel=lead[1])) == pytest.approx(need_accel(lead))
  assert k9_lead_need_accel(SimpleNamespace(status=False, dRel=NAN, vRel=NAN)) == 0.0
  assert k9_lead_need_accel(SimpleNamespace(status=True, dRel=2.0, vRel=NAN)) is None


def test_no_lead_skips_need(monkeypatch):
  loop = Loop(monkeypatch, hold_time=20)
  loop.approach()
  rows = loop.run(150, standstill=True, a_target=-0.1)
  assert all(r["sent"]["aReqValue"] == pytest.approx(-0.2, abs=TOL) for r in sent(rows[2:]))
  check_common(loop)


EPS = 1e-3 / 3.6  # 0.001 km/h


def test_entry_speed_default_is_2kph():
  assert SPEED_PARAM == ("K9StopSoftBrakeSpeed", 2, 0, 5) and K9_SOFT_BRAKE_SPEED == 2.0 / 3.6
  assert read_k9_stop_soft_brake_speed(MemoryParams({})) == 2.0 / 3.6


@pytest.mark.parametrize(("kph_offset", "active"), [(-EPS, True), (0.0, False), (EPS, False)])
def test_2kph_boundary(monkeypatch, kph_offset, active):
  """Just below / at / just above 2 km/h in the stopping state, without standstill ('below' is strict).
  carState.vEgo is float32 here; the exact float64 boundary is checked in test_2kph_boundary_exact."""
  loop = Loop(monkeypatch, hold_time=20)
  loop.approach()
  loop.run(20, v=3 * KPH, a_target=-0.1)
  rows = loop.run(300, v=K9_SOFT_BRAKE_SPEED + kph_offset, a_target=-0.1)
  assert all(r["lcs"] == "stopping" for r in rows)
  assert all((r["soft"] is not None) == active and r["frames"] == 0 and r["soft_hold"] == 0 for r in rows)
  for r in sent(rows[1:]):
    expected = -0.2 if active else max(ACCEL_MIN, min(ACCEL_MAX, r["cc_accel"]))
    assert r["sent"]["aReqValue"] == pytest.approx(expected, abs=TOL)
  check_common(loop)


@pytest.mark.parametrize(("kph_offset", "rearmed"), [(-EPS, False), (0.0, True), (EPS, True)])
def test_2kph_boundary_rearms_after_standstill_ends(monkeypatch, kph_offset, rearmed):
  """After standstill ends, reaching 2 km/h (>=) rearms; just below keeps the original control."""
  loop = Loop(monkeypatch, hold_time=20)
  loop.approach()
  loop.run(100, standstill=True, a_target=-0.1)
  loop.run(20, v=K9_SOFT_BRAKE_SPEED + kph_offset, a_target=-0.1)
  rows = loop.run(100, v=1.5 * KPH, a_target=-0.1)
  assert all((r["soft"] is not None) == rearmed for r in rows)
  check_common(loop)


@pytest.mark.parametrize(("plan", "lead"), [(-1.0, None), (-0.1, (2.5, -1.0)), (-1.0, (2.5, -1.0)), (-0.1, (1.5, -0.3)),
                                            (-0.1, (3.0, 0.3)), (-0.1, (2.0, 0.0)), (-0.1, None)])
@pytest.mark.parametrize("kph", [1.0, 1.5, 1.99])
def test_1_to_2kph_strongest_decel(monkeypatch, plan, lead, kph):
  """The newly included 1-2 km/h band: strongest of -strength, plan and lead need decel is sent."""
  loop = Loop(monkeypatch, hold_time=20, **LEFTOVER)
  loop.approach(lead=lead)
  rows = loop.run(200, v=kph * KPH, a_target=plan, lead=lead)
  assert all(r["soft"] is not None and r["frames"] == 0 and r["soft_hold"] == 0 for r in rows)
  expected = max(ACCEL_MIN, min(-0.2, plan, need_accel(lead)))
  for r in sent(rows[2:]):
    assert r["sent"]["aReqValue"] == pytest.approx(expected, abs=TOL)
    assert r["sent"]["aReqRaw"] == pytest.approx(expected, abs=TOL)
  check_common(loop)


@pytest.mark.parametrize("cut", [dict(gas=True, enabled=False), dict(brake=True, enabled=False), dict(enabled=False),
                                 dict(override=True)])
def test_1_to_2kph_pedal_or_cruise_off(monkeypatch, cut):
  """Pedal/override/cruise off while crawling at 1.5 km/h: original control, no timing and no hold from that time."""
  loop = Loop(monkeypatch, hold_time=20)
  loop.approach()
  loop.run(50, v=1.5 * KPH, a_target=-0.1)
  rows = loop.run(50, v=1.5 * KPH, a_target=-0.1, **cut)
  assert all(r["soft"] is None and r["frames"] == 0 and r["soft_hold"] == 0 and not r["own_hold"] for r in rows[1:])
  for r in sent(rows[2:]):
    assert r["sent"]["aReqValue"] == pytest.approx(max(ACCEL_MIN, min(ACCEL_MAX, r["cc_accel"])), abs=TOL)
  # Released while still moving: the stop counts from its own standstill only (full 2 s from zero).
  loop.run(50, v=1.5 * KPH, a_target=-0.1)
  stop = loop.run(200, standstill=True, a_target=-0.1)
  assert [r["frames"] for r in stop[:199]] == list(range(1, 200)) and stop[199]["soft_hold"] == 2
  check_common(loop)


def test_2kph_boundary_exact():
  for v, active in ((math.nextafter(K9_SOFT_BRAKE_SPEED, 0.), True), (K9_SOFT_BRAKE_SPEED, False),
                    (math.nextafter(K9_SOFT_BRAKE_SPEED, 1.), False)):
    soft, request, clear = K9StopSoftBrake().update(200, speed=K9_SOFT_BRAKE_SPEED, enabled=True, driver=False, standstill=False, v_ego=v,
                                                    inputs_valid=True, stopping=True, soft_hold_active=0)
    assert (soft, request, clear) == (active, False, False)
  for v, rearmed in ((math.nextafter(K9_SOFT_BRAKE_SPEED, 0.), False), (K9_SOFT_BRAKE_SPEED, True)):
    sb = K9StopSoftBrake()
    kw = dict(speed=K9_SOFT_BRAKE_SPEED, enabled=True, driver=False, inputs_valid=True, stopping=True, soft_hold_active=0)
    sb.update(200, standstill=True, v_ego=0., **kw)
    sb.update(200, standstill=False, v_ego=v, **kw)
    assert sb.update(200, standstill=False, v_ego=1.5 / 3.6, **kw)[0] == rearmed


# K9StopSoftBrakeSpeed: 1 km/h steps, 0-5, default 2. 1-5 compare v < setting / 3.6; 0 = standstill signal only.

def f32(x):
  import numpy as np
  return float(np.float32(x))


@pytest.mark.parametrize("setting", [1, 2, 5])
@pytest.mark.parametrize("offset", [-EPS, 0.0, EPS])
def test_speed_setting_boundary(monkeypatch, setting, offset):
  """Just below / equal / just above each setting, stopping state without standstill. carState.vEgo is float32, so the
  'equal' case is decided on the float32 value; the exact float64 boundary is in test_speed_setting_boundary_exact."""
  loop = Loop(monkeypatch, hold_time=20, K9StopSoftBrakeSpeed=setting)
  loop.run(20, v=7 * KPH, a_target=-0.1)
  v = setting / 3.6 + offset
  rows = loop.run(300, v=v, a_target=-0.1)
  active = f32(v) < setting / 3.6  # e.g. float32(5 / 3.6) rounds below 5 / 3.6
  if offset:
    assert active == (offset < 0)  # 0.001 km/h is far beyond float32 rounding
  assert all(r["lcs"] == "stopping" for r in rows)
  assert all((r["soft"] is not None) == active and r["frames"] == 0 and r["soft_hold"] == 0 for r in rows)
  for r in sent(rows[1:]):
    expected = -0.2 if active else max(ACCEL_MIN, min(ACCEL_MAX, r["cc_accel"]))
    assert r["sent"]["aReqValue"] == pytest.approx(expected, abs=TOL)
  check_common(loop)


@pytest.mark.parametrize("setting", [1, 2, 5])
def test_speed_setting_boundary_exact(setting):
  speed = setting / 3.6
  assert read_k9_stop_soft_brake_speed(MemoryParams({"K9StopSoftBrakeSpeed": setting})) == speed
  kw = dict(enabled=True, driver=False, inputs_valid=True, stopping=True, soft_hold_active=0)
  for v, active in ((math.nextafter(speed, 0.), True), (speed, False), (math.nextafter(speed, 9.), False)):
    assert K9StopSoftBrake().update(200, speed=speed, standstill=False, v_ego=v, **kw) == (active, False, False)
  for v, rearmed in ((math.nextafter(speed, 0.), False), (speed, True), (math.nextafter(speed, 9.), True)):
    sb = K9StopSoftBrake()
    sb.update(200, speed=speed, standstill=True, v_ego=0., **kw)
    sb.update(200, speed=speed, standstill=False, v_ego=v, **kw)
    assert sb.update(200, speed=speed, standstill=False, v_ego=speed / 2, **kw)[0] == rearmed


@pytest.mark.parametrize(("stored", "expected"), [
  (None, 2), ("abc", 2), (b"x", 2), (b"", 2), (b"2.5", 2), (float("nan"), 2), (float("inf"), 2), (-float("inf"), 2),
  (2.5, 2), (True, 2), (0, 0), (-3, 0), (1, 1), (3, 3), (5, 5), (9, 5), (b"4", 4), ("5", 5), (4.0, 4),
])
def test_speed_setting_invalid_values(stored, expected):
  values = {} if stored is None else {"K9StopSoftBrakeSpeed": stored}
  assert read_k9_stop_soft_brake_speed(MemoryParams(values)) == expected / 3.6


def test_time_and_strength_missing_or_unparsable_use_defaults():
  assert read_k9_stop_soft_brake(MemoryParams({})) == (0, pytest.approx(-0.2))
  assert read_k9_stop_soft_brake(MemoryParams({"K9StopSoftBrakeTime": "abc", "K9StopSoftBrakeAccel": float("nan")})) == \
    (0, pytest.approx(-0.2))
  assert read_k9_stop_soft_brake(MemoryParams({"K9StopSoftBrakeTime": 20, "K9StopSoftBrakeAccel": b"7"})) == \
    (200, pytest.approx(-0.07))


def test_real_params_unparsable_text_does_not_abort(tmp_path):
  """Real Params: get_int aborts the process on non-numeric text; the reader uses get() and falls back."""
  import os
  import subprocess
  import sys
  root = tmp_path / "params"
  (root / "d").mkdir(parents=True)
  for name, value in (("K9StopSoftBrakeSpeed", b"abc"), ("K9StopSoftBrakeTime", b"nan"), ("K9StopSoftBrakeAccel", b"99")):
    (root / "d" / name).write_bytes(value)
  code = ("from openpilot.common.params import Params\n"
          "from opendbc.car.hyundai.k9_stop_soft_brake import read_k9_stop_soft_brake, read_k9_stop_soft_brake_speed\n"
          "p = Params(); print(read_k9_stop_soft_brake_speed(p) * 3.6, *read_k9_stop_soft_brake(p))\n")
  env = {k: v for k, v in os.environ.items() if k != "OPENPILOT_PREFIX"}  # pytest's per-test params prefix
  out = subprocess.run([sys.executable, "-c", code], env={**env, "PARAMS_ROOT": str(root)}, cwd=str(tmp_path),
                       capture_output=True, text=True, timeout=60)
  assert out.returncode == 0, out.stderr
  speed, frames, accel = map(float, out.stdout.split()[-3:])
  assert speed == pytest.approx(2.0) and frames == 0 and accel == pytest.approx(-0.5)


def test_speed_setting_is_read_every_second(monkeypatch):
  loop = Loop(monkeypatch, hold_time=20)
  loop.run(30, v=7 * KPH, should_stop=False)
  loop.params.values["K9StopSoftBrakeSpeed"] = 5
  waited = 0
  while loop.helper.k9_stop_soft_brake_speed != 5 / 3.6:
    loop.step(v=7 * KPH, should_stop=False)
    waited += 1
  assert 0 < waited <= 100


def set_speed_and_wait_read(loop, setting, **step):
  """Change K9StopSoftBrakeSpeed and keep stepping until the 1 s read has happened."""
  loop.params.values["K9StopSoftBrakeSpeed"] = setting
  rows = []
  while loop.helper.frame % 100 != 1:
    rows.append(loop.step(**step))
  return rows


def test_speed_change_during_episode_keeps_entry_value(monkeypatch):
  loop = Loop(monkeypatch, hold_time=20)  # entry speed 2 km/h
  loop.approach()
  crawl = dict(v=1.5 * KPH, a_target=-0.1)
  rows = set_speed_and_wait_read(loop, 1, **crawl) + loop.run(200, **crawl)
  assert loop.helper.k9_stop_soft_brake_speed == 1 / 3.6
  assert all(r["soft"] is not None for r in rows)  # 1.5 km/h is above the new 1 km/h, but the episode keeps 2 km/h
  wait = loop.run(80, standstill=True, a_target=-0.1)
  wait += set_speed_and_wait_read(loop, 5, standstill=True, a_target=-0.1)
  n = 200 - len(wait)
  wait += loop.run(n, standstill=True, a_target=-0.1)
  assert [r["frames"] for r in wait[:199]] == list(range(1, 200)) and wait[199]["soft_hold"] == 2  # timer not reset
  hold = set_speed_and_wait_read(loop, 1, standstill=True, a_target=-0.1) + loop.run(200, standstill=True, a_target=-0.1)
  assert all(r["soft_hold"] == 2 and r["own_hold"] for r in hold)  # hold not released by the change
  # Driver releases with gas and drives off above every setting: the next stop uses the new 1 km/h.
  loop.run(10, standstill=True, gas=True, enabled=False)
  loop.run(100, v=7 * KPH, a_target=-0.1)
  nxt = [loop.step(v=max(0.3, 7. - 0.035 * i) * KPH, a_target=-0.1) for i in range(200)]
  assert all((r["soft"] is not None) == (r["v"] < 1 / 3.6) for r in nxt[1:]) and any(r["soft"] is not None for r in nxt)
  check_common(loop)


def test_speed_change_without_episode_applies_at_once(monkeypatch):
  loop = Loop(monkeypatch, hold_time=20, K9StopSoftBrakeSpeed=1)
  loop.run(20, v=7 * KPH, a_target=-0.1)
  before = loop.run(150, v=3 * KPH, a_target=-0.1)
  assert all(r["soft"] is None for r in before)  # 3 km/h >= 1 km/h, no episode
  rows = set_speed_and_wait_read(loop, 5, v=3 * KPH, a_target=-0.1)
  after = loop.run(100, v=3 * KPH, a_target=-0.1)
  assert all(r["soft"] is None for r in rows[:-1]) and all(r["soft"] is not None for r in after)
  # 5 -> 1 now that the episode runs at 3 km/h: stays active.
  later = set_speed_and_wait_read(loop, 1, v=3 * KPH, a_target=-0.1) + loop.run(200, v=3 * KPH, a_target=-0.1)
  assert all(r["soft"] is not None for r in later)
  check_common(loop)


@pytest.mark.parametrize(("plan", "lead"), [(-1.0, None), (-0.1, (2.5, -1.0)), (-1.0, (2.5, -1.0)), (-0.1, (1.5, -0.3)),
                                            (-0.1, (3.0, 0.3)), (-0.1, (2.0, 0.0)), (-0.1, None)])
@pytest.mark.parametrize("kph", [2.0, 2.5, 3.5, 4.99])
def test_2_to_5kph_strongest_decel(monkeypatch, plan, lead, kph):
  """Setting 5: the newly allowed 2-5 km/h band sends the strongest of -strength, plan and lead need decel."""
  loop = Loop(monkeypatch, hold_time=20, K9StopSoftBrakeSpeed=5, **LEFTOVER)
  loop.approach(lead=lead, start=7.)
  rows = loop.run(200, v=kph * KPH, a_target=plan, lead=lead)
  assert all(r["soft"] is not None and r["frames"] == 0 and r["soft_hold"] == 0 for r in rows)
  expected = max(ACCEL_MIN, min(-0.2, plan, need_accel(lead)))
  for r in sent(rows[2:]):
    assert r["sent"]["aReqValue"] == pytest.approx(expected, abs=TOL)
    assert r["sent"]["aReqRaw"] == pytest.approx(expected, abs=TOL)
  check_common(loop)


@pytest.mark.parametrize("cut", [dict(gas=True, enabled=False), dict(brake=True, enabled=False), dict(enabled=False),
                                 dict(override=True)])
def test_2_to_5kph_pedal_or_cruise_off(monkeypatch, cut):
  loop = Loop(monkeypatch, hold_time=20, K9StopSoftBrakeSpeed=5)
  loop.approach(start=7.)
  loop.run(50, v=3.5 * KPH, a_target=-0.1)
  rows = loop.run(50, v=3.5 * KPH, a_target=-0.1, **cut)
  assert all(r["soft"] is None and r["frames"] == 0 and r["soft_hold"] == 0 and not r["own_hold"] for r in rows[1:])
  for r in sent(rows[2:]):
    assert r["sent"]["aReqValue"] == pytest.approx(max(ACCEL_MIN, min(ACCEL_MAX, r["cc_accel"])), abs=TOL)
  loop.run(50, v=3.5 * KPH, a_target=-0.1)
  stop = loop.run(200, standstill=True, a_target=-0.1)
  assert [r["frames"] for r in stop[:199]] == list(range(1, 200)) and stop[199]["soft_hold"] == 2
  check_common(loop)


@pytest.mark.parametrize("end", ["invalid_lead", "hold"])
def test_soft_brake_never_feeds_back_into_longcontrol(monkeypatch, end):
  """LongControl's own output (the next cycle's base) is the same with the trial on or off, so when the weak decel
  ends the stronger original command is sent at once, with no delayed recovery."""
  runs = []
  for hold_time in (0, 20):
    loop = Loop(monkeypatch, hold_time=hold_time, K9StopSoftBrakeSpeed=5)
    loop.approach(start=7.)
    loop.run(150, v=1.0 * KPH, a_target=-0.1)
    loop.run(150 if end == "invalid_lead" else 199, standstill=True, a_target=-0.1)
    tail = loop.run(20, standstill=True, a_target=-0.1, lead=(3., NAN) if end == "invalid_lead" else None)
    runs.append((loop, tail))
  (off, _), (on, tail) = runs
  # Until the hold request (which then uses the existing soft hold path), LongControl never sees the weak decel.
  n = len(on.rows) if end == "invalid_lead" else len(on.rows) - len(tail) + 1
  assert [r["cc_accel"] for r in on.rows[:n]] == [r["cc_accel"] for r in off.rows[:n]]
  assert any(r["soft"] is not None for r in on.rows[:-len(tail)]) and tail[0]["soft"] is None
  if end == "hold":
    assert tail[0]["soft_hold"] == 2 and tail[0]["own_hold"]
  first = next(r for r in tail if r["sent"] is not None)
  assert first["sent"]["aReqValue"] == pytest.approx(first["cc_accel"], abs=TOL) and first["cc_accel"] <= -0.5
  check_common(on)


# Speed setting 0: the weak decel starts only from the car's standstill signal (time 0 is still "whole trial off").

def test_speed_zero_never_moving_then_standstill_then_existing_hold(monkeypatch):
  loop = Loop(monkeypatch, hold_time=20, K9StopSoftBrakeSpeed=0)
  loop.approach(start=7.)  # down to 0.3 km/h, below every positive setting
  moving = loop.rows + loop.run(300, v=0.1 * KPH, a_target=-0.1)  # crawling, standstill signal off
  assert all(r["soft"] is None and r["frames"] == 0 and r["soft_hold"] == 0 for r in moving)
  assert any(r["lcs"] == "stopping" for r in moving)
  for r in sent(moving[1:]):
    assert r["sent"]["aReqValue"] == pytest.approx(max(ACCEL_MIN, min(ACCEL_MAX, r["cc_accel"])), abs=TOL)
  wait = loop.run(200, standstill=True, a_target=-0.1)
  assert [r["frames"] for r in wait[:199]] == list(range(1, 200))  # counted from the first standstill frame
  assert all(r["soft"] == pytest.approx(-0.2) and r["soft_hold"] == 0 for r in wait[:199])
  assert all(r["sent"]["aReqValue"] == pytest.approx(-0.2, abs=TOL) and r["sent"]["StopReq"] == 1 for r in sent(wait[1:199]))
  assert wait[199]["soft_hold"] == 2 and wait[199]["own_hold"] and wait[199]["soft"] is None
  hold = loop.run(300, standstill=True, a_target=-0.1)
  assert all(r["soft_hold"] == 2 and r["soft"] is None for r in hold)
  assert all(r["sent"]["StopReq"] == 1 and r["sent"]["ACCMode"] == 1 for r in sent(hold))
  assert hold[-1]["cc_accel"] == pytest.approx(-2.0)
  check_common(loop)


def test_speed_zero_moving_while_waiting_resets_and_restarts(monkeypatch):
  loop = Loop(monkeypatch, hold_time=20, K9StopSoftBrakeSpeed=0)
  loop.approach()
  loop.run(150, standstill=True, a_target=-0.1)
  moved = loop.run(50, v=0.2 * KPH, a_target=-0.1)
  assert all(r["frames"] == 0 and r["soft"] is None and r["soft_hold"] == 0 for r in moved)
  for r in sent(moved):
    assert r["sent"]["aReqValue"] == pytest.approx(r["cc_accel"], abs=TOL)  # original control
  again = loop.run(250, standstill=True, a_target=-0.1)
  assert [r["frames"] for r in again[:199]] == list(range(1, 200))
  assert all(r["soft_hold"] == 0 for r in again[:199]) and again[199]["soft_hold"] == 2
  check_common(loop)


def test_speed_zero_and_time_zero_is_all_off(monkeypatch):
  loop = Loop(monkeypatch, hold_time=0, K9StopSoftBrakeSpeed=0)
  loop.approach(start=7.)
  loop.run(100, v=0.1 * KPH, a_target=-0.1)
  loop.run(800, standstill=True, a_target=-0.1)
  assert all(r["soft"] is None and r["soft_hold"] == 0 and r["frames"] == 0 and not r["own_hold"] for r in loop.rows)
  for r in sent(loop.rows[1:]):
    assert r["sent"]["aReqValue"] == pytest.approx(max(ACCEL_MIN, min(ACCEL_MAX, r["cc_accel"])), abs=TOL)


def test_speed_zero_uses_standstill_not_a_speed_comparison():
  kw = dict(enabled=True, driver=False, inputs_valid=True, stopping=True, soft_hold_active=0)
  # A slightly negative or zero measured speed without the standstill signal does not start it ("v < 0" is not used).
  for v in (-0.05, -1e-6, 0.0, 1e-6):
    assert K9StopSoftBrake().update(200, speed=0.0, standstill=False, v_ego=v, **kw) == (False, False, False)
  # With the standstill signal it starts even if the measured speed is not exactly zero.
  for v in (0.0, 0.05):
    assert K9StopSoftBrake().update(200, speed=0.0, standstill=True, v_ego=v, **kw) == (True, False, False)
  # After standstill ends, only a new standstill rearms it, whatever the speed.
  sb = K9StopSoftBrake()
  sb.update(200, speed=0.0, standstill=True, v_ego=0., **kw)
  assert sb.update(200, speed=0.0, standstill=False, v_ego=3 / 3.6, **kw)[0] is False
  assert sb.update(200, speed=0.0, standstill=False, v_ego=0.0, **kw)[0] is False
  assert sb.update(200, speed=0.0, standstill=True, v_ego=0.0, **kw)[0] is True
  assert sb.frames == 1
  assert read_k9_stop_soft_brake_speed(MemoryParams({"K9StopSoftBrakeSpeed": 0})) == 0.0


def test_speed_zero_change_is_applied_per_episode(monkeypatch):
  loop = Loop(monkeypatch, hold_time=20, K9StopSoftBrakeSpeed=0)
  loop.approach()
  wait = loop.run(80, standstill=True, a_target=-0.1)
  wait += set_speed_and_wait_read(loop, 5, standstill=True, a_target=-0.1)
  wait += loop.run(200 - len(wait), standstill=True, a_target=-0.1)
  assert [r["frames"] for r in wait[:199]] == list(range(1, 200)) and wait[199]["soft_hold"] == 2
  # Released and driven off; the next stop uses 5 km/h (active while moving at 3 km/h).
  loop.run(10, standstill=True, gas=True, enabled=False)
  loop.run(100, v=7 * KPH, a_target=-0.1)
  nxt = loop.run(100, v=3 * KPH, a_target=-0.1)
  assert all(r["soft"] is not None for r in nxt[1:])
  # 5 -> 0 during that moving episode keeps 5 km/h; after driving off again, 0 applies (no weak decel while moving).
  keep = set_speed_and_wait_read(loop, 0, v=3 * KPH, a_target=-0.1) + loop.run(100, v=3 * KPH, a_target=-0.1)
  assert all(r["soft"] is not None for r in keep)
  loop.run(100, v=7 * KPH, a_target=-0.1)
  later = loop.run(100, v=1 * KPH, a_target=-0.1)
  assert all(r["soft"] is None for r in later)
  check_common(loop)
