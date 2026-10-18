"""K9StopSoftBrake(K9 정차 약제동) 삭제 확인.

K9 일반 CAN 카메라 SCC 합성 폐루프(LongControl -> card 크루즈 도우미 -> K9 CarController -> 송신 SCC12)에서
삭제판의 송신 CAN·크루즈 상태·소프트홀드·LongControl 출력이, 삭제 직전판 52cd846c의 cruise.py·carcontroller.py에서
기능 끔(K9StopSoftBrakeTime=0)과 프레임마다 같은지 본다. 기기에 남은 옛 저장값이 있어도 같고, 읽지도 않는다.
기준판 두 파일과 k9_stop_soft_brake.py는 git에서 읽기 전용으로 꺼내 임시 파일로 불러오고, 나머지 모듈은 지금 저장소
것을 쓴다. 메모리 Params·합성 입력 시험이며 실차 응답 검증이 아니다.
"""
import importlib.util
import json
import random
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from openpilot.cereal import car, log
import openpilot.selfdrive.car.cruise as cruise
import openpilot.selfdrive.controls.lib.longcontrol as longcontrol
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai.values import CAR, CarControllerParams, HyundaiFlags

ROOT = Path(__file__).resolve().parents[4]
BASE_COMMIT = "52cd846cdb38acfd02258dd32dc6d0600dabd567"
K9_MODULE = "opendbc.car.hyundai.k9_stop_soft_brake"
BASE_FILES = {"k9": "opendbc_repo/opendbc/car/hyundai/k9_stop_soft_brake.py",
              "cruise": "openpilot/selfdrive/car/cruise.py",
              "carcontroller": "opendbc_repo/opendbc/car/hyundai/carcontroller.py"}
NAMES = ("K9StopSoftBrakeTime", "K9StopSoftBrakeAccel", "K9StopSoftBrakeSpeed")
KPH = 1 / 3.6
K9_FLAGS = int(HyundaiFlags.CAMERA_SCC | HyundaiFlags.LEGACY)
LCS_NAMES = {v: k for k, v in car.CarControl.Actuators.LongControlState.schema.enumerants.items()}
TOL = 0.006
BASE_PARAMS = {"AutoCruiseControl": 1, "HyundaiCameraSCC": 1, "StoppingAccel": -50, "CruiseSpeedUnit": 10,
               "CruiseSpeedUnitBasic": 1, "CruiseButtonLongDelay": 50, "AutoGasTokSpeed": 30, "AutoGasCancelSpeed": 30}
OFF = {"K9StopSoftBrakeTime": 0, "K9StopSoftBrakeAccel": 20, "K9StopSoftBrakeSpeed": 2}
ON = {"K9StopSoftBrakeTime": 20, "K9StopSoftBrakeAccel": 20, "K9StopSoftBrakeSpeed": 2}
LEFTOVERS = {"none": {}, "default": OFF, "on": ON,
             "max": {"K9StopSoftBrakeTime": 150, "K9StopSoftBrakeAccel": 0, "K9StopSoftBrakeSpeed": 5},
             "speed0": {"K9StopSoftBrakeTime": 50, "K9StopSoftBrakeAccel": 5, "K9StopSoftBrakeSpeed": 0}}


class MemoryParams:
  def __init__(self, values):
    self.values = values
    self.reads = []

  def get(self, key):
    self.reads.append(key)
    return self.values.get(key)

  def get_int(self, key):
    self.reads.append(key)
    return int(self.values.get(key, 0))

  def get_float(self, key):
    self.reads.append(key)
    return float(self.values.get(key, 0))

  def get_bool(self, key):
    self.reads.append(key)
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
  def __init__(self, **services):
    super().__init__(**services)
    self.alive = {"carrotMan": False, "longitudinalPlan": False, "radarState": True, "drivingModelData": False}
    self.valid = {"carrotMan": False, "radarState": True}
    self.seen = {"carrotMan": False}
    self.recv_time = {"carrotMan": time.monotonic()}


def make_cp(fingerprint=CAR.KIA_K9, flags=K9_FLAGS):
  return structs.CarParams(brand="hyundai", carFingerprint=fingerprint, flags=flags, openpilotLongitudinalControl=True,
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
  def __init__(self, monkeypatch, cruise_module=cruise, cc_module=carcontroller, cp=None, **values):
    self.params = MemoryParams({**BASE_PARAMS, **values})
    for module in (cruise_module, longcontrol, cc_module):
      monkeypatch.setattr(module, "Params", lambda *args, **kwargs: self.params)
    monkeypatch.setattr(cruise_module, "CommandReader", NullReader)
    self.CP = make_cp() if cp is None else cp
    self.helper = cruise_module.VCruiseCarrot(self.CP)
    self.helper.cruise_state_available_last = True
    self.helper._gas_pressed_count = self.helper._brake_pressed_count = -2
    self.controller = cc_module.CarController({Bus.pt: "hyundai_kia_generic"}, self.CP)
    self.lc = longcontrol.LongControl(self.CP)
    self.parser = CANParser("hyundai_kia_generic", [("SCC12", 0)], 0)
    self.a_ego = 0.
    self.cc = None
    self.frame = 0
    self.rows = []

  @staticmethod
  def make_cc(enabled, override, accel, a_target, state):
    state = state if isinstance(state, str) else LCS_NAMES[int(state)]
    cc = car.CarControl.new_message(enabled=enabled, longActive=enabled and not override)
    cc.cruiseControl.override = override
    cc.actuators.accel, cc.actuators.aTarget, cc.actuators.longControlState = float(accel), float(a_target), state
    st = structs.CarControl(enabled=enabled, longActive=enabled and not override)
    st.cruiseControl.override = override
    st.actuators.accel, st.actuators.aTarget = float(accel), float(a_target)
    st.actuators.longControlState = state
    return cc.as_reader(), st.as_reader()

  def step(self, v=0., standstill=False, a_target=0., should_stop=True, lead=None, gas=False, brake=False, enabled=True,
           override=False):
    if self.cc is None:
      self.cc = self.make_cc(False, False, 0., 0., "off")
    cs = car.CarState.new_message(vEgo=v, vEgoCluster=v, vEgoRaw=v, standstill=standstill, gasPressed=gas, brakePressed=brake,
                                  canValid=True, gearShifter="drive",
                                  cruiseState={"available": True, "enabled": enabled, "standstill": False})
    radar = log.RadarState.new_message()
    if lead is not None:  # (dRel, vRel)
      radar.leadOne.status, radar.leadOne.dRel, radar.leadOne.vRel = True, lead[0], lead[1]
    radar = radar.as_reader()
    cc_capnp, cc_structs = self.cc

    # card: carState and cruise helper, then CarController with the latest carControl
    v_cruise = self.helper.update_v_cruise(cs, SubMaster(carControl=cc_capnp, radarState=radar), True)
    cs.softHoldActive = self.helper._soft_hold_active
    ci_cs = make_ci_state()
    ci_cs.out = structs.CarState(vEgo=v, vEgoRaw=v, standstill=standstill, gasPressed=gas, brakePressed=brake, aEgo=self.a_ego)
    ci_cs.softHoldActive = cs.softHoldActive
    if getattr(self.helper, "k9_stop_soft_brake", None) is not None:  # 기준판 card.py와 같은 전달
      ci_cs.k9_stop_soft_brake_accel = self.helper.k9_stop_soft_brake_accel
    out, sends = self.controller.update(cc_structs, ci_cs, self.frame * 10_000_000)
    sent = None
    scc12 = [m for m in sends if m[0] == 1057]
    if scc12:
      self.parser.update([self.frame * 10_000_000, scc12])
      sent = dict(self.parser.vl["SCC12"])
    self.rows.append(dict(sent=sent, can=sorted((addr, bytes(dat), bus) for addr, dat, bus in sends), out_accel=out.accel,
                          v_cruise=v_cruise, soft_hold=cs.softHoldActive, cc_accel=cc_structs.actuators.accel,
                          lcs=str(cc_structs.actuators.longControlState),
                          soft=getattr(self.helper, "k9_stop_soft_brake_accel", None)))

    # controlsd: next carControl
    plan = log.LongitudinalPlan.new_message(aTarget=a_target, vTargetNow=0. if should_stop else v + 1., shouldStop=should_stop)
    accel, a_t, _ = self.lc.update(enabled and not override, cs.as_reader(), plan.as_reader(), (-3.5, 2.0), 0., radar)
    self.cc = self.make_cc(enabled, override, accel, a_t, self.lc.long_control_state)
    self.frame += 1
    return self.rows[-1]

  def run(self, n, **kwargs):
    return [self.step(**kwargs) for _ in range(n)]


def scenario(loop):
  """정차·출발 되풀이: 앞차 없음/접근 앞차, 기다림 중 브레이크·가속, 긴 브레이크로 거는 원래 소프트홀드와 그 해제."""
  rng = random.Random("k9-stop-soft-brake-removed")
  for cycle in range(4):
    lead = (6., -0.5) if cycle % 2 else None
    loop.a_ego = 0.
    loop.run(150, v=10 * KPH, should_stop=False, a_target=rng.uniform(-1., .5))
    for i in range(200):
      loop.a_ego = rng.uniform(-1.2, -.3)
      loop.step(v=max(0.3, 10 - 10 / 200 * i) * KPH, a_target=rng.uniform(-1.5, -.1), lead=lead)
    for i in range(300):
      loop.a_ego = -.9 if i < 100 else 0.
      loop.step(standstill=True, a_target=-.1, brake=cycle == 2 and 100 <= i < 120, gas=cycle == 3 and 200 <= i < 210)
    if cycle == 1:  # 원래 수동 소프트홀드: 정지 중 긴 브레이크(>0.6초) 뒤 놓음
      loop.run(150, standstill=True, brake=True, enabled=False)
      loop.run(1, standstill=True, enabled=False)
      loop.run(300, standstill=True, a_target=-.1)
      assert loop.rows[-1]["soft_hold"] == 2
      loop.run(30, standstill=True, gas=True, should_stop=False, a_target=.5)
    loop.a_ego = .5
    for i in range(100):
      loop.step(v=i * .1 * KPH, should_stop=False, a_target=.8)
  return loop


def trace(loop):
  return [(r["can"], r["out_accel"], r["v_cruise"], r["soft_hold"], r["cc_accel"], r["lcs"]) for r in loop.rows]


def load_base(name, rel, tmp_path):
  try:
    source = subprocess.run(["git", "show", f"{BASE_COMMIT}:{rel}"], cwd=ROOT, check=True, capture_output=True,
                            text=True, encoding="utf-8").stdout
  except (OSError, subprocess.CalledProcessError) as e:
    pytest.fail(f"기준판 {rel}을 읽지 못함: {e}")
  path = tmp_path / Path(rel).name
  path.write_text(source, encoding="utf-8")
  spec = importlib.util.spec_from_file_location(name, path)
  module = importlib.util.module_from_spec(spec)
  sys.modules[name] = module
  spec.loader.exec_module(module)
  return module


@pytest.fixture(scope="module")
def base(tmp_path_factory):
  """52cd846c의 k9_stop_soft_brake.py(원래 이름으로 잠시 등록)·cruise.py·carcontroller.py."""
  tmp = tmp_path_factory.mktemp("base_52cd846c")
  saved = sys.modules.get(K9_MODULE)
  load_base(K9_MODULE, BASE_FILES["k9"], tmp)
  try:
    modules = SimpleNamespace(cruise=load_base("cruise_k9_soft_brake_base_52cd846c", BASE_FILES["cruise"], tmp),
                              carcontroller=load_base("carcontroller_k9_soft_brake_base_52cd846c", BASE_FILES["carcontroller"], tmp))
  finally:
    if saved is None:
      sys.modules.pop(K9_MODULE, None)
    else:
      sys.modules[K9_MODULE] = saved
  assert hasattr(modules.cruise.VCruiseCarrot, "_update_k9_stop_soft_brake")  # 정말 그 기능이 있는 기준판
  return modules


def base_loop(monkeypatch, base, values):
  return Loop(monkeypatch, cruise_module=base.cruise, cc_module=base.carcontroller, **values)


def test_base_scenario_really_uses_soft_brake(monkeypatch, base):
  """비교 장면이 기준판의 약제동을 실제로 건드리는지(켜면 송신값이 달라지는지) 먼저 확인."""
  off = scenario(base_loop(monkeypatch, base, OFF))
  on = scenario(base_loop(monkeypatch, base, ON))
  assert all(r["soft"] is None for r in off.rows)
  assert any(r["soft"] is not None for r in on.rows)
  assert len(trace(on)) == len(trace(off)) and trace(on) != trace(off)


@pytest.mark.parametrize("leftover", LEFTOVERS)
def test_k9_matches_base_soft_brake_off_even_with_leftover(monkeypatch, base, leftover):
  """삭제판은 기준판 K9StopSoftBrakeTime=0과 송신 CAN·크루즈·소프트홀드·종방향 출력이 프레임마다 같다.
  옛 저장값이 남아 있어도 같고, 그 이름을 읽지 않는다."""
  ref = scenario(base_loop(monkeypatch, base, OFF))
  new = scenario(Loop(monkeypatch, **LEFTOVERS[leftover]))
  assert len(new.rows) == len(ref.rows) and trace(new) == trace(ref)
  assert not set(NAMES) & set(new.params.reads)
  assert any(r["soft_hold"] == 2 for r in new.rows)  # 원래 소프트홀드는 그대로 걸림
  # 보통 제동: 송신 감속은 LongControl 출력을 기존 제한(-4.0~2.5, 판다와 같음)으로 자른 값.
  assert (CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX) == (-4.0, 2.5)
  for r in new.rows:
    if r["sent"] is not None:
      expected = max(CarControllerParams.ACCEL_MIN, min(CarControllerParams.ACCEL_MAX, r["cc_accel"]))
      assert r["sent"]["aReqValue"] == pytest.approx(expected, abs=TOL)


def test_soft_brake_code_settings_docs_and_tools_are_gone():
  """운영 코드·설정·메뉴·키·현행 설명서·전용 시험·도구에서 이름이 사라짐."""
  assert not hasattr(cruise.VCruiseCarrot, "_update_k9_stop_soft_brake")
  helper_src = (ROOT / BASE_FILES["cruise"]).read_text(encoding="utf-8")
  for rel in (BASE_FILES["cruise"], BASE_FILES["carcontroller"], "openpilot/selfdrive/car/card.py",
              "openpilot/common/params_keys.h", "openpilot/selfdrive/carrot_settings.json",
              "docs/user/ko/settings.md", "docs/user/en/settings.md"):
    text = (ROOT / rel).read_text(encoding="utf-8")
    assert "K9StopSoftBrake" not in text and "k9_stop_soft_brake" not in text and "약제동" not in text, rel
  assert "stop_soft_brake" not in helper_src
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  assert not set(NAMES) & {x.get("name") for x in catalog["params"]}
  for rel in (BASE_FILES["k9"], "openpilot/selfdrive/car/tests/test_k9_stop_soft_brake.py",
              "openpilot/selfdrive/car/tests/test_k9_stop_soft_brake_replay_tool.py",
              "tools/k9_stop_soft_brake_replay.py", "tools/k9_stop_soft_brake_compare.py"):
    assert not (ROOT / rel).exists(), rel
  with pytest.raises(ModuleNotFoundError):
    importlib.import_module(K9_MODULE)
