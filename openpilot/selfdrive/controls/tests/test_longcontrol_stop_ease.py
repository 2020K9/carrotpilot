import importlib.util
import json
import math
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
BASE_COMMIT = "4a75945c3316994e75c363269781e76827d38810"
LONGCONTROL = "openpilot/selfdrive/controls/lib/longcontrol.py"
STEP = lc.STOP_EASE_RATE * lc.DT_CTRL
ON = {"StoppingAccel": -50, "StopEaseEnable": 1, "StopEaseDist": 40, "StopEaseSpeed": 5, "StopEaseAccel": -50}
LIMITS = (-3.5, 2.)


@pytest.fixture(scope="module")
def base_lc(tmp_path_factory):
  # The unmodified controller before stop easing, for bit-for-bit comparison.
  path = os.environ.get("STOP_EASE_BASE_LONGCONTROL")
  if not path:
    try:
      source = subprocess.run(["git", "show", f"{BASE_COMMIT}:{LONGCONTROL}"], cwd=ROOT, check=True,
                              capture_output=True, text=True, encoding="utf-8").stdout
    except (OSError, subprocess.CalledProcessError):
      pytest.skip("base longcontrol.py is unavailable (set STOP_EASE_BASE_LONGCONTROL)")
    path = tmp_path_factory.mktemp("base") / "longcontrol_base.py"
    path.write_text(source, encoding="utf-8")
  spec = importlib.util.spec_from_file_location("longcontrol_stop_ease_base", path)
  module = importlib.util.module_from_spec(spec)
  sys.modules[spec.name] = module
  spec.loader.exec_module(module)
  assert not hasattr(module, "STOP_EASE_PARAMS")
  return module


def frame(*, should_stop=True, v=.3, a_ego=0., d_rel=3., v_rel=0., status=True, brake=False, gas=False,
          fcw=False, soft_hold=0, standstill=False):
  cs = SimpleNamespace(softHoldActive=soft_hold, vEgo=v, aEgo=a_ego, brakePressed=brake, gasPressed=gas,
                       cruiseState=SimpleNamespace(standstill=standstill))
  plan = SimpleNamespace(aTarget=-.5, vTargetNow=v, jTargetNow=0., shouldStop=should_stop, fcw=fcw)
  radar = SimpleNamespace(leadOne=SimpleNamespace(status=status, dRel=d_rel, vRel=v_rel))
  return cs, plan, radar


def make(module, monkeypatch, values, brand="hyundai", flags=0):
  params = DictParams(dict(values))
  monkeypatch.setattr(module, "Params", lambda: params)
  cp = make_cp(brand)
  cp.flags = flags
  return module.LongControl(cp), params


def step(control, f, limits=LIMITS, active=True):
  accel, _, _ = control.update(active, f[0], f[1], limits, 0., f[2])
  return accel


def warm_up(control, n=100):
  # Crosses the first ~1 s settings read while driving, before any stop.
  for _ in range(n):
    step(control, frame(should_stop=False, v=2.))


def enter_from(control, accel):
  control.long_control_state = lc.LongCtrlState.pid
  control.last_output_accel = accel


def random_frames(rng, n, **fixed):
  frames = []
  for i in range(n):
    kw = dict(should_stop=(i // 40) % 3 != 2, v=rng.uniform(0., 2.), a_ego=rng.uniform(-1.5, .5),
              d_rel=rng.uniform(.5, 10.), v_rel=rng.uniform(-2., 1.), status=rng.random() > .1,
              brake=rng.random() < .05, gas=rng.random() < .05, fcw=rng.random() < .02,
              soft_hold=int(rng.random() < .03), standstill=rng.random() < .3)
    kw.update(fixed)
    frames.append(frame(**kw))
  return frames


@pytest.mark.parametrize("case", ["default_off", "far_lead", "fast", "no_lead", "soft_hold", "canfd", "toyota"])
def test_off_or_unmet_conditions_match_base_bit_for_bit(monkeypatch, base_lc, case):
  values, fixed, brand, flags = ON, {}, "hyundai", 0
  if case == "default_off":
    values = {"StoppingAccel": -50}
  elif case == "far_lead":
    fixed = {"d_rel": 4.01}
  elif case == "fast":
    fixed = {"v": .51}
  elif case == "no_lead":
    fixed = {"status": False}
  elif case == "soft_hold":
    fixed = {"soft_hold": 1}
  elif case == "canfd":
    flags = HyundaiFlags.CANFD
  else:
    brand = "toyota"
  new, _ = make(lc, monkeypatch, values, brand, flags)
  base, _ = make(base_lc, monkeypatch, values, brand, flags)
  rng = random.Random(case)
  frames = random_frames(rng, 400, **fixed)
  for i, f in enumerate(frames):
    limits = (rng.uniform(-3.5, -.2), rng.uniform(-.6, 2.))
    active = i % 97 != 50
    assert step(new, f, limits, active) == step(base, f, limits, active)
    assert new.long_control_state == base.long_control_state


@pytest.mark.parametrize("d_rel", [4.01, 4.0, 3.0])
@pytest.mark.parametrize("v", [.51, .5, .3])
def test_distance_speed_boundaries_start_on_the_first_true_update(monkeypatch, d_rel, v):
  control, _ = make(lc, monkeypatch, ON)
  warm_up(control)
  assert control.stop_ease_pending == pytest.approx((4.0, .5, -.5))
  enter_from(control, -.93)
  accel = step(control, frame(d_rel=d_rel, v=v))
  assert control.long_control_state == lc.LongCtrlState.stopping
  assert control.stop_ease_shadow == -.93
  if d_rel <= 4.0 and v <= .5:
    assert accel == pytest.approx(-.93 + STEP)
  else:
    assert accel == -.93


def test_easing_curve_rate_bounds_and_weaker_original(monkeypatch, base_lc):
  control, _ = make(lc, monkeypatch, ON)
  warm_up(control)
  enter_from(control, -.93)
  previous = -.93
  for _ in range(200):
    accel = step(control, frame(v=0.))
    shadow = control.stop_ease_shadow
    assert 0. <= accel - previous <= STEP + 1e-9
    assert shadow <= accel <= max(shadow, -.5) and accel < 0.
    previous = accel
  assert accel == -.5 and control.stop_ease_shadow == -.93

  # The original ramp from -0.3 ends at StoppingAccel (-0.5, possibly one ulp stronger),
  # which stays weaker than A=-0.6 for the whole stop: output is the original.
  new, _ = make(lc, monkeypatch, {**ON, "StopEaseAccel": -60})
  base, _ = make(base_lc, monkeypatch, ON)
  for control in (new, base):
    warm_up(control)
    enter_from(control, -.3)
  for _ in range(200):
    f = frame(v=0.)
    assert step(new, f) == step(base, f)


@pytest.mark.parametrize("trigger", [{"brake": True}, {"gas": True}, {"fcw": True}, {"v_rel": -3.}])
def test_safety_return_matches_original_in_that_frame(monkeypatch, base_lc, trigger):
  new, _ = make(lc, monkeypatch, ON)
  base, _ = make(base_lc, monkeypatch, ON)
  for control in (new, base):
    warm_up(control)
    enter_from(control, -.93)
  for _ in range(20):
    f = frame()
    assert step(new, f) > step(base, f)
  f = frame(**trigger)
  assert step(new, f) == step(base, f) == new.stop_ease_shadow
  f = frame()
  assert step(new, f) == pytest.approx(new.stop_ease_shadow + STEP)


def test_continuous_lifecycle_and_pending_settings(monkeypatch, base_lc):
  new, params = make(lc, monkeypatch, ON)
  base, _ = make(base_lc, monkeypatch, ON)
  def both(f):
    return step(new, f), step(base, f)
  for control in (new, base):
    warm_up(control)
    enter_from(control, -.93)

  # 1) easing starts
  for i in range(10):
    accel, original = both(frame())
    assert accel == pytest.approx(-.93 + STEP * (i + 1)) and original == -.93
  # 2) lead lost: original output in that frame
  accel, original = both(frame(status=False))
  assert accel == original == new.stop_ease_shadow == -.93
  # 3) lead reacquired: easing restarts from the original output at the bounded rate
  accel, original = both(frame())
  assert accel == pytest.approx(-.93 + STEP)
  # 4) safety return (closing speed)
  accel, original = both(frame(v_rel=-3.))
  assert accel == original == -.93
  # A value change during this stop is held for the next stop.
  params.values["StopEaseAccel"] = -30
  for _ in range(150):
    accel, original = both(frame(v=0.))
  assert new.stop_ease_pending == pytest.approx((4.0, .5, -.3))
  assert accel == -.5 and original == -.93

  # 5) leave stopping, drive briefly, then 6) re-enter: shadow starts from the entry's last output
  for _ in range(30):
    accel, original = both(frame(should_stop=False, v=1.))
    assert new.long_control_state == lc.LongCtrlState.pid and accel == original
    assert new.stop_ease_shadow is None
  entry = new.last_output_accel
  assert entry == base.last_output_accel
  accel, original = both(frame(d_rel=3.5, a_ego=0.))
  expected = entry if entry <= new.stopping_accel else min(entry, 0.) - STEP
  assert new.long_control_state == lc.LongCtrlState.stopping
  assert new.stop_ease_shadow == original == pytest.approx(expected)
  assert new.stop_ease == pytest.approx((4.0, .5, -.3))
  # Disable during this stop: easing continues with the applied values.
  params.values["StopEaseEnable"] = 0
  for _ in range(250):
    accel, original = both(frame(v=0.))
  assert new.stop_ease_pending is None and accel == -.3 and original == -.5

  for _ in range(30):
    both(frame(should_stop=False, v=1.))
  # Next stop is fully original.
  for i in range(150):
    accel, original = both(frame(v=0., d_rel=2.))
    assert accel == original
  assert new.stop_ease is None


def test_final_accel_limits_keep_priority(monkeypatch, base_lc):
  rng = random.Random(7)
  weakest = -.2
  new, _ = make(lc, monkeypatch, {**ON, "StopEaseAccel": -20})
  base, _ = make(base_lc, monkeypatch, ON)
  off, _ = make(lc, monkeypatch, {"StoppingAccel": -50})
  for control in (new, base, off):
    warm_up(control)
  eased = safety = 0
  for i in range(1500):
    lower = rng.choice([-3.5, rng.uniform(-3.5, -.05)])
    upper = rng.choice([2., rng.uniform(max(lower, -1.2), 2.)])
    kw = dict(should_stop=(i // 300) % 4 != 3, v=rng.uniform(0., .5), d_rel=rng.uniform(2., 4.),
              v_rel=rng.uniform(-.6, .2), a_ego=0.)
    special = rng.random()
    if special < .03:
      kw["brake"] = True
    elif special < .06:
      kw["soft_hold"] = 1
    active = special < .08 or special >= .1
    f = frame(**kw)
    accel = step(new, f, (lower, upper), active)
    original = step(base, f, (lower, upper), active)
    assert step(off, f, (lower, upper), active) == original
    if new.long_control_state != lc.LongCtrlState.stopping or kw.get("soft_hold"):
      assert accel == original
      continue
    shadow = new.stop_ease_shadow
    assert shadow == original
    assert shadow <= accel <= max(shadow, min(max(weakest, lower), upper))
    if accel != shadow:
      eased += 1
      assert accel < 0.
    if kw.get("brake"):
      safety += 1
      assert accel == shadow
  assert eased > 100 and safety > 5


@pytest.mark.parametrize("name, stored, index, expected", [
  ("StopEaseDist", 9, 0, 1.0), ("StopEaseDist", 81, 0, 8.0),
  ("StopEaseSpeed", 2, 1, .3), ("StopEaseSpeed", 31, 1, 3.0),
  ("StopEaseAccel", -101, 2, -1.0), ("StopEaseAccel", -19, 2, -.2),
  ("StopEaseDist", float("nan"), 0, 4.0), ("StopEaseSpeed", float("inf"), 1, .5),
  ("StopEaseAccel", "invalid", 2, -.5), ("StopEaseEnable", 2, None, None),
])
def test_setting_bounds_and_invalid_values(monkeypatch, name, stored, index, expected):
  control, _ = make(lc, monkeypatch, {**ON, name: stored})
  warm_up(control)
  if index is None:
    assert control.stop_ease_pending == pytest.approx((4.0, .5, -.5))
  else:
    assert control.stop_ease_pending[index] == pytest.approx(expected)


@pytest.mark.parametrize("enable", [-1, 0, float("nan"), "bad", None])
def test_missing_or_invalid_enable_is_off(monkeypatch, enable):
  values = {k: v for k, v in ON.items() if k != "StopEaseEnable"}
  if enable is not None:
    values["StopEaseEnable"] = enable
  control, _ = make(lc, monkeypatch, values)
  warm_up(control)
  assert control.stop_ease_pending is None


class CountingParams(DictParams):
  def __init__(self, values):
    super().__init__(values)
    self.reads = []

  def get_float(self, name):
    self.reads.append(name)
    return super().get_float(name)


@pytest.mark.parametrize("brand, flags, long_control, reads", [
  ("hyundai", 0, True, 4), ("hyundai", HyundaiFlags.CANFD, True, 0), ("toyota", 0, True, 0),
  ("hyundai", 0, False, 0),
])
def test_settings_read_only_every_100_frames_and_only_for_conventional_hyundai(monkeypatch, brand, flags,
                                                                                long_control, reads):
  params = CountingParams(dict(ON))
  monkeypatch.setattr(lc, "Params", lambda: params)
  cp = make_cp(brand)
  cp.flags = flags
  cp.openpilotLongitudinalControl = long_control
  control = lc.LongControl(cp)
  stop_ease = lambda: [name for name in params.reads if name.startswith("StopEase")]
  assert stop_ease() == []
  warm_up(control, 99)
  assert stop_ease() == [] and control.stop_ease_pending is None
  warm_up(control, 1)
  assert len(stop_ease()) == reads
  warm_up(control, 150)
  assert len(stop_ease()) == reads * 2


def test_stop_ease_catalog_menu_and_param_keys():
  catalog = json.loads((ROOT / 'openpilot/selfdrive/carrot_settings.json').read_text(encoding='utf-8'))
  def objects(value):
    if isinstance(value, dict):
      yield value
      for child in value.values():
        yield from objects(child)
    elif isinstance(value, list):
      for child in value:
        yield from objects(child)
  entries = list(objects(catalog))
  menu = next(x for x in entries if x.get('id') == 'CRUISE_STOPGO')
  keys = (ROOT / 'openpilot/common/params_keys.h').read_text(encoding='utf-8')
  for name, default, lo, hi in lc.STOP_EASE_PARAMS:
    setting = next(x for x in entries if x.get('name') == name)
    unit = 10 if name == "StopEaseAccel" else 1
    assert (setting['min'], setting['max'], setting['default'], setting['unit']) == (lo, hi, default, unit)
    assert setting['risk'] == 'high'
    assert name in menu['params']
    assert f'{{"{name}", {{PERSISTENT, INT, "{default}"}}}}' in keys
  params = menu['params']
  assert params[params.index('StoppingAccel') + 1:params.index('StoppingAccel') + 5] == [
    name for name, *_ in lc.STOP_EASE_PARAMS]
  assert math.isclose(lc.STOP_EASE_RATE, .8) and lc.STOP_EASE_GAP == 1.5 and lc.STOP_EASE_NEED_GAIN == 2.
