"""StopEase (close-lead stop easing) removal: LongControl must match 131f8b1c with StopEaseEnable=0 bit for bit,
for every former target (Hyundai/Kia conventional CAN incl. K9) and other platforms, even with leftover stored
StopEase values (StopEaseAccel=-20 included). StoppingAccel is a separate setting and stays.
"""
import importlib.util
import json
import os
import random
import subprocess
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

# Also provides the Windows-only Params/hardware import shims.
from openpilot.selfdrive.controls.tests.test_longcontrol_hyundai_tuning import DictParams, make_cp
from opendbc.car.hyundai.values import HyundaiFlags
import openpilot.selfdrive.controls.lib.longcontrol as lc

ROOT = Path(__file__).resolve().parents[4]
BASE_COMMIT = "131f8b1cbf6caf35f2be68fde5223c871a7a9c8a"
LONGCONTROL = "openpilot/selfdrive/controls/lib/longcontrol.py"
STOP_EASE_NAMES = ("StopEaseEnable", "StopEaseDist", "StopEaseSpeed", "StopEaseAccel")
BASE_OFF = {"StopEaseEnable": 0, "StopEaseDist": 40, "StopEaseSpeed": 5, "StopEaseAccel": -50}
LEFTOVER = {"StopEaseEnable": 1, "StopEaseDist": 80, "StopEaseSpeed": 30, "StopEaseAccel": -20}
K9_FLAGS = int(HyundaiFlags.CAMERA_SCC | HyundaiFlags.LEGACY)
CARS = {"hyundai_can": ("hyundai", 0), "k9_camera_scc": ("hyundai", K9_FLAGS), "hyundai_canfd": ("hyundai", int(HyundaiFlags.CANFD)),
        "toyota": ("toyota", 0)}


@pytest.fixture(scope="module")
def base_lc(tmp_path_factory):
  path = os.environ.get("STOP_EASE_BASE_LONGCONTROL")
  if not path:
    try:
      source = subprocess.run(["git", "show", f"{BASE_COMMIT}:{LONGCONTROL}"], cwd=ROOT, check=True,
                              capture_output=True, text=True, encoding="utf-8").stdout
    except (OSError, subprocess.CalledProcessError):
      pytest.skip("base longcontrol.py is unavailable (set STOP_EASE_BASE_LONGCONTROL)")
    path = tmp_path_factory.mktemp("base") / "longcontrol_131f8b1c.py"
    path.write_text(source, encoding="utf-8")
  spec = importlib.util.spec_from_file_location("longcontrol_stop_ease_base_131f8b1c", path)
  module = importlib.util.module_from_spec(spec)
  sys.modules[spec.name] = module
  spec.loader.exec_module(module)
  assert hasattr(module, "STOP_EASE_PARAMS")  # really the version with stop easing
  return module


class RecordingParams(DictParams):
  def __init__(self, values):
    super().__init__(values)
    self.reads = []

  def get_float(self, name):
    self.reads.append(name)
    return super().get_float(name)


def frame(*, should_stop=True, v=.3, a_ego=0., d_rel=3., v_rel=0., status=True, brake=False, gas=False,
          fcw=False, soft_hold=0, standstill=False):
  cs = SimpleNamespace(softHoldActive=soft_hold, vEgo=v, aEgo=a_ego, brakePressed=brake, gasPressed=gas,
                       cruiseState=SimpleNamespace(standstill=standstill))
  plan = SimpleNamespace(aTarget=-.5, vTargetNow=v, jTargetNow=0., shouldStop=should_stop, fcw=fcw)
  radar = SimpleNamespace(leadOne=SimpleNamespace(status=status, dRel=d_rel, vRel=v_rel))
  return cs, plan, radar


def make(module, monkeypatch, values, brand, flags):
  params = RecordingParams(dict(values))
  monkeypatch.setattr(module, "Params", lambda: params)
  cp = make_cp(brand)
  cp.flags = flags
  return module.LongControl(cp), params


def random_frames(rng, n):
  frames = []
  for i in range(n):
    frames.append(frame(should_stop=(i // 40) % 3 != 2, v=rng.uniform(0., 2.), a_ego=rng.uniform(-1.5, .5),
                        d_rel=rng.uniform(.5, 10.), v_rel=rng.uniform(-2., 1.), status=rng.random() > .1,
                        brake=rng.random() < .05, gas=rng.random() < .05, fcw=rng.random() < .02,
                        soft_hold=int(rng.random() < .03), standstill=rng.random() < .3))
  return frames


def close_lead_stop(n=600):
  """Former easing case: pid -> stopping with a lead 1-4 m ahead at <0.5 m/s, then a soft hold and a restart."""
  frames = [frame(should_stop=False, v=2., d_rel=8.)] * 150
  frames += [frame(v=max(0., .5 - .002 * k), d_rel=max(1.6, 4. - .01 * k), v_rel=-.3 * (k < 200), standstill=k > 250)
             for k in range(n)]
  frames += [frame(v=0., d_rel=1.6, soft_hold=1, standstill=True)] * 100
  frames += [frame(should_stop=False, v=.2, d_rel=3.)] * 100
  return frames


@pytest.mark.parametrize("car", CARS)
@pytest.mark.parametrize("stopping_accel", [-50, -70, -100])
@pytest.mark.parametrize("sequence", ["random", "close_lead_stop"])
def test_matches_base_with_stop_ease_off_even_with_leftover_values(monkeypatch, base_lc, car, stopping_accel, sequence):
  brand, flags = CARS[car]
  new, new_params = make(lc, monkeypatch, {"StoppingAccel": stopping_accel, **LEFTOVER}, brand, flags)
  base, _ = make(base_lc, monkeypatch, {"StoppingAccel": stopping_accel, **BASE_OFF}, brand, flags)
  rng = random.Random(f"{car}{stopping_accel}{sequence}")
  frames = random_frames(rng, 1500) if sequence == "random" else close_lead_stop()
  for i, (cs, plan, radar) in enumerate(frames):
    limits = (rng.uniform(-3.5, -.2), rng.uniform(-.6, 2.)) if sequence == "random" else (-3.5, 2.)
    active = i % 97 != 50
    assert new.update(active, cs, plan, limits, 0., radar) == base.update(active, cs, plan, limits, 0., radar)
    assert new.long_control_state == base.long_control_state
    assert new.last_output_accel == base.last_output_accel
  # StoppingAccel is still read and applied; no StopEase key is ever read.
  assert "StoppingAccel" in new_params.reads and not set(STOP_EASE_NAMES) & set(new_params.reads)
  assert new.stopping_accel == pytest.approx(stopping_accel * .01)


def test_stop_ease_code_and_state_are_gone():
  for name in ("STOP_EASE_PARAMS", "STOP_EASE_GAP", "STOP_EASE_NEED_GAIN", "STOP_EASE_RATE"):
    assert not hasattr(lc, name)
  for name in ("_refresh_stop_ease", "_stop_ease_accel"):
    assert not hasattr(lc.LongControl, name)
  source = (ROOT / LONGCONTROL).read_text(encoding="utf-8")
  assert "stop_ease" not in source and "StopEase" not in source


def test_catalog_param_keys_and_experimental_menu_removed():
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  keys = (ROOT / "openpilot/common/params_keys.h").read_text(encoding="utf-8")
  text = json.dumps(catalog, ensure_ascii=False)
  for name in STOP_EASE_NAMES:
    assert name not in text and name not in keys

  def objects(value):
    if isinstance(value, dict):
      yield value
      for child in value.values():
        yield from objects(child)
    elif isinstance(value, list):
      for child in value:
        yield from objects(child)
  entries = list(objects(catalog))
  stopping = next(x for x in entries if x.get("name") == "StoppingAccel")
  assert (stopping["min"], stopping["max"], stopping["default"]) == (-100, -50, -50)
  assert '{"StoppingAccel", {PERSISTENT, INT, "-50"}}' in keys
  # EXPERIMENTAL 묶음의 옛 항목(K9 약제동 3개·LongMpcJerkCost)은 지워졌고, 지금은 화면 이름 "실행중" 아래
  # K9StopMode(옛 K9CreepMode, 2026-10-22 교체)와 K9LaneFix·K9LRView가 있다(옛 K9StopTaper 항목은 2026-10-22 삭제됨).
  experimental = [x for x in entries if x.get("id") == "EXPERIMENTAL"]
  assert len(experimental) == 1 and experimental[0]["params"] == ["K9StopMode", "K9LaneFix", "K9LRView", "K9ExtHud", "K9MeterSeason"]
  assert "LongMpcJerkCost" not in text


# K9StopJerkEase(K9 정차 변화 한도 완화) 삭제 확인.
# 기준판 43d443e4의 carcontroller.py를 git에서 읽기 전용으로 꺼내 임시 파일로 불러오고(K9_JERK_EASE_BASE_CARCONTROLLER로
# 다른 파일을 줄 수 있음), 기능 끔(0) 기준판과 삭제판의 실제 송신 CAN을 같은 입력·같은 다른 설정으로 비교한다.
# 기준판 carcontroller.py 말고 나머지 모듈은 지금 저장소 것을 쓴다. 기준판이 가져오는 지운 k9_stop_soft_brake.py도
# 같은 기준판에서 꺼내 불러올 때만 잠시 등록하며, 약제동은 끔(시간 0)으로만 비교한다(약제동 삭제는
# openpilot/selfdrive/car/tests/test_k9_stop_soft_brake_removed.py에서 따로 확인).
JERK_EASE_BASE_COMMIT = "43d443e48f903bcd617612d7c9faefe8e9834910"
CARCONTROLLER = "opendbc_repo/opendbc/car/hyundai/carcontroller.py"
JERK_EASE = "K9StopJerkEase"
K9_SOFT_BRAKE_MODULE = "opendbc.car.hyundai.k9_stop_soft_brake"
K9_SOFT_BRAKE_FILE = "opendbc_repo/opendbc/car/hyundai/k9_stop_soft_brake.py"


@pytest.fixture(scope="module")
def base_cc(tmp_path_factory):
  def base_source(rel):
    try:
      return subprocess.run(["git", "show", f"{JERK_EASE_BASE_COMMIT}:{rel}"], cwd=ROOT, check=True,
                            capture_output=True, text=True, encoding="utf-8").stdout
    except (OSError, subprocess.CalledProcessError) as e:
      pytest.fail(f"기준판 {rel}을 읽지 못함(carcontroller.py는 K9_JERK_EASE_BASE_CARCONTROLLER로 지정): {e}")

  def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module

  tmp = tmp_path_factory.mktemp("base_cc")
  path = os.environ.get("K9_JERK_EASE_BASE_CARCONTROLLER")
  source = base_source(CARCONTROLLER) if not path else Path(path).read_text(encoding="utf-8")
  path = tmp / "carcontroller_43d443e4.py"
  path.write_text(source, encoding="utf-8")
  soft_brake = tmp / "k9_stop_soft_brake_43d443e4.py"
  soft_brake.write_text(base_source(K9_SOFT_BRAKE_FILE), encoding="utf-8")
  saved = sys.modules.get(K9_SOFT_BRAKE_MODULE)
  load(K9_SOFT_BRAKE_MODULE, soft_brake)
  try:
    module = load("carcontroller_jerk_ease_base_43d443e4", path)
  finally:
    if saved is None:
      sys.modules.pop(K9_SOFT_BRAKE_MODULE, None)
    else:
      sys.modules[K9_SOFT_BRAKE_MODULE] = saved
  assert hasattr(module, JERK_EASE)  # 정말 그 기능이 있는 기준판
  return module


def k9_run(monkeypatch, module, values):
  """K9 카메라 SCC 합성 폐루프(test_k9_stop_soft_brake_removed.Loop)에서 module의 CarController로 정차·출발을 되풀이하고
  매 프레임 송신 CAN(주소·바이트·버스)과 읽은 설정 이름을 돌려준다."""
  from opendbc.car import Bus
  from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import KPH, Loop
  loop = Loop(monkeypatch, **values)
  monkeypatch.setattr(module, "Params", lambda *args, **kwargs: loop.params)
  loop.controller = module.CarController({Bus.pt: "hyundai_kia_generic"}, loop.CP)
  reads, sends, update = [], [], loop.controller.update
  a_ego = [0.]  # Loop은 aEgo를 넣지 않으므로 여기서 넣는다(감속 중 aEgo로 정차 때 변화 한도 위쪽 값이 오른다)
  get_int, get = loop.params.get_int, loop.params.get
  loop.params.get_int = lambda key: reads.append(key) or get_int(key)
  loop.params.get = lambda key: reads.append(key) or get(key)

  def recorded(cc, cs, now):
    cs.out.aEgo = a_ego[0]
    out = update(cc, cs, now)
    sends.append((out[0].accel, sorted((addr, bytes(dat), bus) for addr, dat, bus in out[1])))
    return out
  loop.controller.update = recorded

  rng = random.Random("k9-stop-jerk-ease-removed")
  for cycle in range(4):
    lead = (6., -0.5) if cycle % 2 else None
    a_ego[0] = 0.
    for _ in range(150):
      loop.step(v=10 * KPH, should_stop=False, a_target=rng.uniform(-1., .5))
    for i in range(200):
      a_ego[0] = rng.uniform(-1.2, -.3)
      loop.step(v=max(0.3, 10 - 10 / 200 * i) * KPH, a_target=rng.uniform(-1.5, -.1), lead=lead)
    for i in range(300):
      a_ego[0] = -.9 if i < 100 else 0.
      loop.step(standstill=True, a_target=-.1, brake=cycle == 2 and 100 <= i < 120, gas=cycle == 3 and 200 <= i < 210)
    a_ego[0] = .5
    for i in range(100):
      loop.step(v=i * .1 * KPH, should_stop=False, a_target=.8)
  return loop, sends, reads




def test_k9_base_scenario_really_uses_jerk_ease(monkeypatch, base_cc):
  """비교 장면이 기준판의 이 기능을 실제로 건드리는지(켜면 송신값이 달라지는지) 먼저 확인."""
  _, off, _ = k9_run(monkeypatch, base_cc, {JERK_EASE: 0})
  for mode in (1, 2):
    _, on, _ = k9_run(monkeypatch, base_cc, {JERK_EASE: mode})
    assert len(on) == len(off) and on != off


@pytest.mark.parametrize("leftover", [None, 0, 1, 2])
def test_k9_sends_match_base_jerk_ease_off_even_with_leftover(monkeypatch, base_cc, leftover):
  """삭제판의 송신 CAN은 기준판 K9StopJerkEase=0과 프레임마다 같다. 옛 저장값 1·2가 남아 있어도 같고, 읽지도 않는다."""
  from opendbc.car.hyundai import carcontroller
  from opendbc.car.hyundai.values import CarControllerParams
  _, base, _ = k9_run(monkeypatch, base_cc, {JERK_EASE: 0})
  extra = {} if leftover is None else {JERK_EASE: leftover}
  loop, new, reads = k9_run(monkeypatch, carcontroller, extra)
  assert len(new) == len(base) == len(loop.rows) and new == base
  assert JERK_EASE not in reads and "StoppingAccel" in loop.params.reads
  # 차로 보내는 감속은 LongControl 출력을 기존 제한(-4.0~2.5, 판다와 같음)으로 자른 값.
  for r in loop.rows:
    if r["sent"] is not None:
      expected = max(CarControllerParams.ACCEL_MIN, min(CarControllerParams.ACCEL_MAX, r["cc_accel"]))
      assert r["sent"]["aReqValue"] == pytest.approx(expected, abs=.006)
  assert (CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX) == (-4.0, 2.5)
  assert all(CarControllerParams.ACCEL_MIN <= r["sent"]["aReqValue"] <= CarControllerParams.ACCEL_MAX
             for r in loop.rows if r["sent"] is not None)


def test_k9_panda_accel_limits_unchanged():
  safety = (ROOT / "opendbc_repo/opendbc/safety/safety/safety_hyundai.h").read_text(encoding="utf-8")
  assert ".max_accel = 250," in safety and ".min_accel = -400," in safety


def test_k9_stop_jerk_ease_code_setting_and_tools_are_gone():
  """운영 코드·설정·메뉴·현행 설명·전용 시험·도구에서 이름이 사라짐(이 시험 파일의 확인용 문자열만 남음)."""
  from opendbc.car.hyundai import carcontroller
  for name in (JERK_EASE, "K9_STOP_JERK_EASE_TIME", "K9_STOP_JERK_EASE_DT"):
    assert not hasattr(carcontroller, name)
  for name in ("stop_jerk_ease", "stop_jerk_ease_mode"):
    assert not hasattr(carcontroller.CarController, name)
  for rel in (CARCONTROLLER, "openpilot/common/params_keys.h", "openpilot/selfdrive/carrot_settings.json", "docs/user/ko/settings.md"):
    text = (ROOT / rel).read_text(encoding="utf-8")
    assert JERK_EASE not in text and "stop_jerk_ease" not in text and "변화한도완화" not in text and "변화 한도 완화" not in text
  for rel in ("opendbc_repo/opendbc/car/hyundai/tests/test_k9_stop_jerk_ease.py", "tools/k9_stop_jerk_ease_replay.py",
              "tools/k9_stop_jerk_ease_compare.py"):
    assert not (ROOT / rel).exists()
