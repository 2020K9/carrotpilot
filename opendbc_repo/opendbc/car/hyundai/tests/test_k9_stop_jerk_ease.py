import importlib.util
import os
import pickle
import subprocess
import sys
from pathlib import Path
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


def drive(controller, steps, before=None, after=None):
  """steps: dicts of stopping/aEgo/jerk/enabled/gas/brake/override/soft_hold. Returns every sent frame (100 Hz)."""
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
    CC.actuators.jerk = s.get("jerk", 0.3)  # pid Upper = 2 * jerk, default 0.6
    CC.cruiseControl.override = s.get("override", False)
    _, sends = controller.update(CC.as_reader(), CS, frame * 10_000_000)
    out.append(sends)
    if after is not None:
      after(controller, sends)
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


def test_mode2_follows_a_fall_between_start_and_previous_output(monkeypatch):
  # Computed 2.3 ramps from 0.6; at k=25 the previous sent Upper is 1.4 (0.6 + 1.7 * 24 / 50 = 1.416) and the
  # computed value falls to 1.3 (aEgo -0.4): above the start, below the previous output.
  fall, rise = 25, 40
  steps = stop_sequence(stop=2 * fall) + [dict(stopping=True, aEgo=-0.4)] * (2 * (rise - fall)) + [dict(stopping=True)] * 60
  ju_base = uppers(drive(make_controller(monkeypatch, 0), steps))
  ju = uppers(drive(make_controller(monkeypatch, 2), steps))
  assert ju_base[20 + fall - 1] == pytest.approx(2.3) and ju_base[20 + fall] == pytest.approx(1.3)
  assert ju[20 + fall - 1] == pytest.approx(1.4)  # previous output, higher than the new computed 1.3
  # 211a8734 sent 0.6 + 0.7 * 25 / 50 = 0.95 here and kept re-lowering toward the start on later frames.
  assert ju[20 + fall:20 + rise] == [pytest.approx(1.3)] * (rise - fall)
  # Computed rises again: limited from the previous output upward, never below it, then follows the base.
  resumed = ju[20 + rise:20 + RAMP_FRAMES]
  assert resumed[0] == pytest.approx(round(0.6 + 1.7 * rise / RAMP_FRAMES, 1))
  assert all(b >= a - 1e-6 for a, b in zip([1.3] + resumed, resumed, strict=False))
  assert all(x <= 2.3 + 1e-6 for x in resumed)
  assert ju[20 + RAMP_FRAMES:] == ju_base[20 + RAMP_FRAMES:]


def test_mode2_unit_fall_is_followed_and_not_relowered():
  ease = K9StopJerkEase()
  ease.record(0, 0.6)
  out = []
  for k in range(RAMP_FRAMES):
    computed = 2.3 if k < 25 else 1.3
    out.append(ease.update(2, 1, computed, True))
    ease.record(1, out[-1])
  assert out[24] == pytest.approx(0.6 + 1.7 * 24 / RAMP_FRAMES)  # 1.416 > 1.3
  assert out[25:] == [1.3] * (RAMP_FRAMES - 25)


# pid Upper = 2 * jerk. Values around the 0.05 rounding boundaries of the 0.1 m/s^3 JerkUpperLimit step.
BOUNDARY_PID_UPPERS = [0.64, 0.649999, 0.65, 0.650001, 0.66, 0.74, 0.75, 0.76, 1.05, 1.149999, 1.15]


@pytest.mark.parametrize("pid_upper", BOUNDARY_PID_UPPERS)
@pytest.mark.parametrize("mode", [1, 2])
def test_record_is_decoded_transmitted_upper_at_decimal_boundaries(monkeypatch, mode, pid_upper):
  steps = [dict(stopping=False, jerk=pid_upper / 2)] * 40 + [dict(stopping=True)] * 140
  recorded = []

  def after(controller, sends):
    if any(m[0] == 905 for m in sends):
      recorded.append(controller.stop_jerk_ease.last_jerk_u)
  base_sent = drive(make_controller(monkeypatch, 0), steps)
  sent = drive(make_controller(monkeypatch, mode), steps, after=after)
  ju_base, ju = uppers(base_sent), uppers(sent)
  assert recorded == ju  # exactly the value decoded from the packed SCC14 bytes
  assert all(abs(x * 10 - round(x * 10)) < 1e-6 for x in recorded)
  prev = ju_base[19]  # the transmitted 0.1-step pid Upper, not the internal float
  assert abs(prev - pid_upper) <= 0.05 + 1e-6
  assert ju[20] == prev  # onset never exceeds the last transmitted value
  if mode == 1:
    # Mode 1 is unchanged: only the onset frame differs from the base.
    assert ju[:20] == ju_base[:20] and ju[21:] == ju_base[21:]
  else:
    ramp = ju[20:20 + RAMP_FRAMES]
    assert all(b >= a for a, b in zip(ramp, ramp[1:], strict=False)) and all(x <= 2.3 + 1e-6 for x in ramp)
    assert ju[20 + RAMP_FRAMES:] == ju_base[20 + RAMP_FRAMES:]


@pytest.mark.parametrize("pid_upper", BOUNDARY_PID_UPPERS)
def test_mode1_transmitted_values_match_float_record(monkeypatch, pid_upper):
  """Recording the decoded value instead of the internal float (211a8734) leaves mode 1's bytes unchanged."""
  steps = [dict(stopping=False, jerk=pid_upper / 2)] * 40 + [dict(stopping=True, aEgo=a) for a in (-0.9, -0.3, -0.2)] * 40

  class FloatRecord(K9StopJerkEase):
    def update(self, mode, stop_req, jerk_u, allowed):
      out = super().update(mode, stop_req, jerk_u, allowed)
      self.pending = out
      return out

    def record(self, stop_req, jerk_u):
      super().record(stop_req, self.pending if jerk_u is not None else None)

  float_controller = make_controller(monkeypatch, 1)
  float_controller.stop_jerk_ease = FloatRecord()
  assert drive(float_controller, steps) == drive(make_controller(monkeypatch, 1), steps)


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


REPO_ROOT = Path(__file__).resolve().parents[5]


def load_tool(name):
  spec = importlib.util.spec_from_file_location(name, REPO_ROOT / "tools" / f"{name}.py")
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


def snapshot(path):
  return {str(p.relative_to(path)): p.read_bytes() for p in sorted(Path(path).rglob("*")) if p.is_file()}


def run_replay_tool(params_root, tmp_path):
  env = {**os.environ, "PARAMS_ROOT": str(params_root), "TMPDIR": str(tmp_path)}
  return subprocess.run([sys.executable, str(REPO_ROOT / "tools" / "k9_stop_jerk_ease_replay.py"), "--schema", str(tmp_path / "none"),
                         "--out", str(tmp_path / "out.pkl"), "--start", "0", "--end", "1", str(tmp_path / "none")],
                        env=env, capture_output=True, text=True, timeout=120)


@pytest.mark.parametrize("kind", ["dir", "symlink_to_dir", "dangling_symlink", "file"])
def test_replay_tool_refuses_existing_params_root_and_keeps_it(tmp_path, kind):
  victim = tmp_path / "victim"
  (victim / "d").mkdir(parents=True)
  (victim / "d" / "K9StopJerkEase").write_bytes(b"1")
  (victim / "d" / "Other").write_bytes(b"keep me")
  (victim / "note.txt").write_bytes(b"unrelated")
  target = {"dir": victim, "symlink_to_dir": tmp_path / "link", "dangling_symlink": tmp_path / "dangling",
            "file": tmp_path / "file"}[kind]
  if kind == "symlink_to_dir":
    target.symlink_to(victim, target_is_directory=True)
  elif kind == "dangling_symlink":
    target.symlink_to(tmp_path / "missing")
  elif kind == "file":
    target.write_bytes(b"a file")
  before = snapshot(victim)
  result = run_replay_tool(target, tmp_path)
  assert result.returncode != 0 and "refusing PARAMS_ROOT" in result.stderr
  assert snapshot(victim) == before
  assert target.is_symlink() == (kind in ("symlink_to_dir", "dangling_symlink"))
  if kind == "file":
    assert target.read_bytes() == b"a file"
  assert not (tmp_path / "missing").exists() and not (tmp_path / "out.pkl").exists()


def test_replay_tool_uses_a_new_directory_per_run_and_mode(tmp_path):
  replay_tool = load_tool("k9_stop_jerk_ease_replay")
  root = replay_tool.make_params_root(str(tmp_path / "root"))
  dirs = [replay_tool.write_params(root, m, {"A": b"x"}, {"K9StopJerkEase": m.encode()}) for m in ("0", "1", "2", "2")]
  assert len(set(dirs)) == 4 and all(Path(d).parent == Path(root) for d in dirs)
  assert [snapshot(d) for d in dirs] == [{"d/A": b"x", "d/K9StopJerkEase": m.encode()} for m in ("0", "1", "2", "2")]
  with pytest.raises(SystemExit):
    replay_tool.make_params_root(root)  # a second run cannot reuse the first run's root
  assert len(os.listdir(root)) == 4


SCC_STEP_NS = 20_000_000


def fake_rows(stop_req, areq, upper):
  rows = []
  for k, (s, a, u) in enumerate(zip(stop_req, areq, upper, strict=True)):
    scc14 = (round(u * 10) << 12).to_bytes(8, "little")
    rows.append(dict(t=k * SCC_STEP_NS, sends=[(1057, f"{s}/{a}".encode(), 0), (905, scc14, 0)], rec={},
                     dec={"SCC12": dict(ACCMode=1, StopReq=s, aReqRaw=a, aReqValue=a),
                          "SCC14": dict(ComfortBandUpper=0.7, ComfortBandLower=0.6, JerkUpperLimit=u, JerkLowerLimit=1.0, ACCMode=1)},
                     vEgo=1.0, aEgo=-0.5, gas=False, brake=False, override=False, enabled=True, lcs="pid", softHold=0))
  return rows


def run_compare(tmp_path, base_rows, mode_rows):
  compare_tool = load_tool("k9_stop_jerk_ease_compare")
  with open(tmp_path / "base.pkl", "wb") as fh:
    pickle.dump({"base": {"rows": base_rows}, "git": "test"}, fh)
  with open(tmp_path / "new.pkl", "wb") as fh:
    pickle.dump({**{m: {"rows": r} for m, r in mode_rows.items()}, "git": "test"}, fh)
  return compare_tool.main(["--out-dir", str(tmp_path), f"X={tmp_path / 'base.pkl'},{tmp_path / 'new.pkl'},0"])


STOP = [0] * 5 + [1] * 15
AREQ = [-0.5] * 20
UPPER = [0.6] * 5 + [2.3] * 15


def eased(upper, frames):
  return upper[:5] + [0.6] * frames + upper[5 + frames:]


def test_compare_passes_valid_case(tmp_path):
  modes = {"0": fake_rows(STOP, AREQ, UPPER), "1": fake_rows(STOP, AREQ, eased(UPPER, 1)), "2": fake_rows(STOP, AREQ, eased(UPPER, 5))}
  assert run_compare(tmp_path, fake_rows(STOP, AREQ, UPPER), modes) == 0


@pytest.mark.parametrize("case", ["empty", "no_onset_all_zero", "no_onset_all_one", "mismatched_steps"])
def test_compare_unevaluable_exits_nonzero(tmp_path, case):
  stop = {"no_onset_all_zero": [0] * 20, "no_onset_all_one": [1] * 20}.get(case, STOP)
  base = [] if case == "empty" else fake_rows(stop, AREQ, UPPER)
  modes = {m: list(base) for m in ("0", "1", "2")}
  if case == "mismatched_steps":
    modes["2"] = base[:-1]
  assert run_compare(tmp_path, base, modes) == 2


@pytest.mark.parametrize("case", ["off_differs", "stronger_decel", "weaker_areq", "stop_req_changed", "upper_higher",
                                  "comfort_band_changed", "jerk_lower_changed", "residue_before_onset"])
def test_compare_invariant_violation_exits_nonzero(tmp_path, case):
  base = fake_rows(STOP, AREQ, UPPER)
  modes = {"0": fake_rows(STOP, AREQ, UPPER), "1": fake_rows(STOP, AREQ, eased(UPPER, 1)), "2": fake_rows(STOP, AREQ, eased(UPPER, 5))}
  bad = modes["2"][10]
  if case == "off_differs":
    modes["0"] = fake_rows(STOP, AREQ, eased(UPPER, 1))
  elif case in ("stronger_decel", "weaker_areq"):
    modes["2"] = fake_rows(STOP, AREQ[:10] + [-0.6 if case == "stronger_decel" else -0.4] + AREQ[11:], eased(UPPER, 5))
  elif case == "stop_req_changed":
    modes["2"] = fake_rows(STOP[:10] + [0] + STOP[11:], AREQ, eased(UPPER, 5))
  elif case == "upper_higher":
    modes["1"] = fake_rows(STOP, AREQ, UPPER[:3] + [0.7] + UPPER[4:])
  elif case == "comfort_band_changed":
    bad["dec"]["SCC14"]["ComfortBandLower"] = 0.5
  elif case == "jerk_lower_changed":
    bad["dec"]["SCC14"]["JerkLowerLimit"] = 1.2
  elif case == "residue_before_onset":
    modes["2"] = fake_rows(STOP, AREQ, [0.5] + eased(UPPER, 5)[1:])
  assert run_compare(tmp_path, base, modes) == 1
