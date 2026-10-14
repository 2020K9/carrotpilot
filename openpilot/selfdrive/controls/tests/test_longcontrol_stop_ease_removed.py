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


def test_catalog_param_keys_and_experimental_menu():
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
  menu = next(x for x in entries if x.get("id") == "EXPERIMENTAL")
  driving = next(x for x in catalog["menu"] if x.get("id") == "DRIVING")
  assert driving["groups"][-1] is menu and menu["ko"] == menu["en"] == menu["zh"] == "실험중"
  assert menu["params"] == ["K9StopJerkEase", "K9StopSoftBrakeTime", "K9StopSoftBrakeAccel", "K9StopSoftBrakeSpeed", "LongMpcJerkCost"]
  leaf_params = [name for x in entries if isinstance(x.get("params"), list) for name in x["params"] if isinstance(name, str)]
  for name in menu["params"]:
    assert leaf_params.count(name) == 1
