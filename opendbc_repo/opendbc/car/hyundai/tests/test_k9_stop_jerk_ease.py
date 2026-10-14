from types import SimpleNamespace

import pytest

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.carcontroller import K9_STOP_JERK_EASE_DT, K9_STOP_JERK_EASE_TIME, K9StopJerkEase, LongCtrlState
from opendbc.car.hyundai.values import CAR, HyundaiFlags

RAMP_FRAMES = round(K9_STOP_JERK_EASE_TIME / K9_STOP_JERK_EASE_DT)
K9_FLAGS = int(HyundaiFlags.CAMERA_SCC | HyundaiFlags.LEGACY)


def make_controller(monkeypatch, mode, *, fingerprint=CAR.KIA_K9, flags=K9_FLAGS, long_control=True, auto_cruise=0):
  values = {"K9StopJerkEase": mode, "HyundaiCameraSCC": 1, "AutoCruiseControl": auto_cruise}
  params = SimpleNamespace(get_int=lambda key: values.get(key, 0), get_bool=lambda key: False, get_float=lambda key: 0.0)
  monkeypatch.setattr(carcontroller, "Params", lambda: params)
  cp = structs.CarParams(carFingerprint=fingerprint, wheelbase=3.15, steerRatio=14.5, flags=flags,
                         openpilotLongitudinalControl=long_control)
  return carcontroller.CarController({Bus.pt: "hyundai_kia_generic"}, cp)


def make_state():
  parser = CANParser("hyundai_kia_generic", [("SCC11", 0), ("SCC12", 0), ("SCC13", 0), ("SCC14", 0)], 2)
  return SimpleNamespace(out=structs.CarState(vEgo=1.0, vEgoRaw=1.0, aEgo=-0.9), modelV2=None, is_metric=True, lkas11=None, clu11=None,
                         mdps12=None, fca11=None, paddle_button_prev=0, softHoldActive=0,
                         scc11=dict(parser.vl["SCC11"]), scc12=dict(parser.vl["SCC12"]), scc13=dict(parser.vl["SCC13"]),
                         scc14=dict(parser.vl["SCC14"]))


def decode(messages):
  parser = CANParser("hyundai_kia_generic", [("SCC12", 0), ("SCC14", 0)], 0)
  parser.update([0, [m for m in messages if m[0] in (1057, 905)]])
  return dict(parser.vl["SCC12"]), dict(parser.vl["SCC14"])


def drive(controller, steps, before=None):
  """steps: dicts of stopping/aEgo/enabled/gas/brake/override/soft_hold. Returns every sent frame (100 Hz)."""
  CS = make_state()
  out = []
  for frame, s in enumerate(steps):
    if before is not None:
      before(frame)
    CS.out = structs.CarState(vEgo=1.0, vEgoRaw=1.0, aEgo=s.get("aEgo", -0.9), gasPressed=s.get("gas", False),
                              brakePressed=s.get("brake", False))
    CS.softHoldActive = s.get("soft_hold", 0)
    CC = structs.CarControl(enabled=s.get("enabled", True), longActive=s.get("enabled", True))
    CC.actuators.accel = -0.9
    CC.actuators.longControlState = LongCtrlState.stopping if s.get("stopping") else LongCtrlState.pid
    CC.actuators.jerk = 0.3  # pid Upper 0.6
    CC.cruiseControl.override = s.get("override", False)
    _, sends = controller.update(CC.as_reader(), CS, frame * 10_000_000)
    out.append(sends)
  return out


def uppers(sent):
  return [decode(s)[1]["JerkUpperLimit"] for s in sent if any(m[0] == 905 for m in s)]


def stop_sequence(pre=40, stop=200, **stop_kwargs):
  return [dict(stopping=False)] * pre + [dict(stopping=True, **stop_kwargs)] * stop


@pytest.mark.parametrize(("fingerprint", "flags", "long_control", "active"), [
  (CAR.KIA_K9, K9_FLAGS, True, True),
  (CAR.KIA_K9, K9_FLAGS, False, False),
  (CAR.KIA_K9, int(HyundaiFlags.LEGACY), True, False),
  (CAR.KIA_K9, K9_FLAGS | int(HyundaiFlags.CANFD), True, False),
  (CAR.KIA_K7, K9_FLAGS, True, False),
  (CAR.GENESIS_G90, K9_FLAGS, True, False),
])
def test_scope_is_k9_conventional_can_camera_scc_openpilot_long(monkeypatch, fingerprint, flags, long_control, active):
  controller = make_controller(monkeypatch, 2, fingerprint=fingerprint, flags=flags, long_control=long_control)
  assert (controller.stop_jerk_ease is not None) == active


@pytest.mark.parametrize("value", [-1, 3, 99])
def test_invalid_setting_is_off(monkeypatch, value):
  controller = make_controller(monkeypatch, value)
  drive(controller, [dict()])
  assert controller.stop_jerk_ease_mode == 0


@pytest.mark.parametrize("mode", [0, 1, 2])
def test_off_matches_unscoped_controller_and_on_changes_only_upper(monkeypatch, mode):
  steps = stop_sequence(stop=60) + [dict(stopping=False, gas=True, override=True)] * 20 + stop_sequence(pre=20, stop=160)
  base = make_controller(monkeypatch, 0)
  base.stop_jerk_ease = None  # the pre-change code path
  reference = drive(base, steps)
  sent = drive(make_controller(monkeypatch, mode), steps)
  for a, b in zip(reference, sent, strict=True):
    if mode == 0:
      assert a == b
    else:
      (r12, r14), (s12, s14) = decode(a), decode(b)
      assert r12 == s12
      assert {k: v for k, v in r14.items() if k != "JerkUpperLimit"} == {k: v for k, v in s14.items() if k != "JerkUpperLimit"}
      assert s14["JerkUpperLimit"] <= r14["JerkUpperLimit"]
      assert [m for m in a if m[0] != 905] == [m for m in b if m[0] != 905]


def test_mode1_limits_only_the_first_sent_stop_req_frame(monkeypatch):
  ju = uppers(drive(make_controller(monkeypatch, 1), stop_sequence()))
  assert ju[19] == pytest.approx(0.6)  # last pid frame
  assert ju[20] == pytest.approx(0.6)  # first StopReq frame: base would send 2.3
  assert ju[21] == pytest.approx(2.3)  # rise is only deferred by one send


def test_mode2_ramps_rise_only_then_follows_computed(monkeypatch):
  ju = uppers(drive(make_controller(monkeypatch, 2), stop_sequence()))
  for k in range(RAMP_FRAMES + 5):
    expected = min(2.3, 0.6 + (2.3 - 0.6) * k / RAMP_FRAMES) if k < RAMP_FRAMES else 2.3
    assert ju[20 + k] == pytest.approx(round(expected, 1), abs=0.051)


def test_mode2_follows_a_falling_computed_value_immediately(monkeypatch):
  steps = stop_sequence(stop=10) + [dict(stopping=True, aEgo=0.0)] * 40  # Upper drops to 0.5
  ju = uppers(drive(make_controller(monkeypatch, 2), steps))
  assert ju[25:] == [pytest.approx(0.5)] * len(ju[25:])


@pytest.mark.parametrize("release", [dict(stopping=False), dict(stopping=True, gas=True), dict(stopping=True, brake=True),
                                     dict(stopping=True, override=True), dict(stopping=True, enabled=False)])
def test_release_ends_limit_and_reentry_restarts(monkeypatch, release):
  steps = stop_sequence(stop=20) + [release] * 10 + [dict(stopping=True)] * 120
  base = make_controller(monkeypatch, 0)
  ju_base = uppers(drive(base, steps))
  ju = uppers(drive(make_controller(monkeypatch, 2), steps))
  assert ju[20] < ju_base[20]
  assert ju[30:35] == ju_base[30:35]  # released: computed value, no residue
  if release.get("stopping") is False:
    # StopReq went 0 -> 1 again: a fresh ramp from the last sent (pid) Upper.
    assert ju[35] == pytest.approx(0.6) and ju_base[35] == pytest.approx(2.3)
    assert ju[35 + RAMP_FRAMES:] == ju_base[35 + RAMP_FRAMES:]
  else:
    # StopReq stayed 1 through the intervention: no new onset, limit stays off.
    assert ju[35:] == ju_base[35:]


def test_mode_change_to_off_releases(monkeypatch):
  steps = stop_sequence(stop=100)
  ju_base = uppers(drive(make_controller(monkeypatch, 0), steps))
  controller = make_controller(monkeypatch, 2)
  values = {"K9StopJerkEase": 2, "HyundaiCameraSCC": 1}
  params = SimpleNamespace(get_int=lambda key: values.get(key, 0), get_bool=lambda key: False, get_float=lambda key: 0.0)
  monkeypatch.setattr(carcontroller, "Params", lambda: params)

  def before(frame):
    if frame == 50:  # read at frame % 50 == 0, mid-ramp
      values["K9StopJerkEase"] = 0
  ju = uppers(drive(controller, steps, before))
  assert ju[24] < ju_base[24]
  assert ju[25:] == ju_base[25:]


@pytest.mark.parametrize(("soft_hold", "auto_cruise", "stopping", "expected"), [
  (0, 0, False, 0), (0, 0, True, 1), (1, 0, False, 1), (2, 0, False, 1), (1, 2, False, 0), (1, 2, True, 1),
])
def test_stop_req_source_matches_transmitted_frame(monkeypatch, soft_hold, auto_cruise, stopping, expected):
  controller = make_controller(monkeypatch, 1, auto_cruise=auto_cruise)
  sent = drive(controller, [dict(stopping=stopping, soft_hold=soft_hold)])
  assert decode(sent[0])[0]["StopReq"] == expected
  assert controller.stop_jerk_ease.last_stop_req == expected


def test_soft_hold_onset_is_a_sent_stop_req_onset(monkeypatch):
  ju = uppers(drive(make_controller(monkeypatch, 1), [dict()] * 40 + [dict(soft_hold=1)] * 4))
  assert ju[20] == pytest.approx(0.6)


def test_no_sent_record_or_missing_scc_starts_with_computed_value():
  ease = K9StopJerkEase()
  assert ease.update(2, 1, 2.3, True) == 2.3  # first frame ever: no history
  ease.record(None, None)  # SCC12/SCC14 not sent
  assert ease.update(2, 1, 2.3, True) == 2.3
  ease.record(0, 0.6)
  assert ease.update(2, None, 2.3, True) == 2.3  # StopReq not transmitted this frame
