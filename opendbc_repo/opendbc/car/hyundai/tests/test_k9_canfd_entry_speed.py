"""실험중 > CAN-FD 정차 진입 속도 (K9CanfdEntrySpeed, x0.01 m/s, default 70 = ENTRY_SPEED 0.7 m/s).

New tests only; no existing test input or threshold changed. 대감님 2026-11-09 18:42.
"""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from opendbc.car.hyundai import stopping as stopping_mod
from opendbc.car.hyundai.stopping import (CanfdStopping, StopPhase, ENTRY_SPEED, ENTRY_SPEED_PARAM,
                                          ENTRY_SPEED_REFRESH_FRAMES, read_entry_speed)
from opendbc.car.hyundai.tests.test_stopping import make_cs, send

ROOT = Path(__file__).resolve().parents[5]


class FakeParams:
  def __init__(self, value):
    self.value = value
    self.calls = 0

  def get(self, key):
    assert key == ENTRY_SPEED_PARAM
    self.calls += 1
    if isinstance(self.value, Exception):
      raise self.value
    return self.value


def step(controller, **overrides):
  args = dict(active=True, requested=True, speed=0.3, held=False, accel=-0.5,
              previous_value=-0.5, jerk_u=2.0, jerk_l=1.0)
  args.update(overrides)
  return controller.update(**args)


def test_original_constant_unchanged():
  assert ENTRY_SPEED == 0.7
  assert ENTRY_SPEED_PARAM == "K9CanfdEntrySpeed"


@pytest.mark.parametrize("raw,expected", [
  (None, 0.7), (70, 0.7), ("70", 0.7), (b"70", 0.7),
  (100, 1.0), (b"120", 1.2), (20, 0.2), (200, 2.0),
  (19, 0.7), (201, 0.7), (0, 0.7), (-70, 0.7), (b"abc", 0.7), ("", 0.7), (7.5, 0.7),
  (RuntimeError("no params"), 0.7), (KeyError("unknown key"), 0.7),
])
def test_read_entry_speed_falls_back_to_original(raw, expected):
  assert read_entry_speed(FakeParams(raw)) == expected  # 7.5 -> int 7 out of range -> original


def test_default_value_is_bit_identical_to_constant():
  assert read_entry_speed(FakeParams(70)) == ENTRY_SPEED
  assert (70 / 100.0).hex() == ENTRY_SPEED.hex()


def test_real_params_failure_uses_original(monkeypatch):
  import openpilot.common.params as params_mod

  def broken():
    raise OSError("params unavailable")
  monkeypatch.setattr(params_mod, "Params", broken)
  assert read_entry_speed() == ENTRY_SPEED
  assert CanfdStopping().entry_speed == ENTRY_SPEED


@pytest.mark.parametrize("speed,default_phase,raised_phase", [
  (0.9, StopPhase.approach, StopPhase.request),
  (0.69, StopPhase.request, StopPhase.request),
  (1.2, StopPhase.approach, StopPhase.approach),
])
def test_entry_speed_moves_request_threshold(speed, default_phase, raised_phase):
  default = CanfdStopping(params=FakeParams(70))
  raised = CanfdStopping(params=FakeParams(100))
  assert raised.entry_speed == 1.0
  step(default, speed=speed)
  step(raised, speed=speed)
  assert default.phase == default_phase
  assert raised.phase == raised_phase


def test_lower_entry_speed_keeps_approach_braking_longer():
  c = CanfdStopping(params=FakeParams(40))
  cmd = step(c, speed=0.6)
  assert c.phase == StopPhase.approach and cmd.stop_req == 0
  cmd = step(c, speed=0.4)
  assert c.phase == StopPhase.request and c.reason == "entry_speed" and cmd.stop_req == 1


def test_value_refreshes_only_while_idle():
  p = FakeParams(70)
  c = CanfdStopping(params=p)
  step(c, speed=0.9)
  assert c.phase == StopPhase.approach
  p.value = 100
  for _ in range(3 * ENTRY_SPEED_REFRESH_FRAMES):
    step(c, speed=0.9)
  assert c.entry_speed == 0.7 and c.phase == StopPhase.approach  # no change inside an episode
  for _ in range(ENTRY_SPEED_REFRESH_FRAMES):
    c.update(active=False, requested=False, speed=0.9, held=False, accel=0.0, previous_value=0.0, jerk_u=2.0, jerk_l=1.0)
  assert c.entry_speed == 1.0
  step(c, speed=0.9)
  assert c.phase == StopPhase.request


def test_param_read_rate_is_limited():
  p = FakeParams(70)
  c = CanfdStopping(params=p)
  for _ in range(10 * ENTRY_SPEED_REFRESH_FRAMES):
    c.update(active=False, requested=False, speed=0.0, held=False, accel=0.0, previous_value=0.0, jerk_u=2.0, jerk_l=1.0)
  assert p.calls == 1 + 10


SPEEDS = [3.0, 2.0, 1.5, 1.0, 0.9, 0.8, 0.71, 0.7, 0.69, 0.5, 0.3, 0.1, 0.04, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.08, 0.12, 0.0]


def run_packets(camera, controller):
  CS = make_cs()
  out = []
  for v in SPEEDS:
    CS.out.vEgo = CS.out.vEgoRaw = v
    CS.out.wheelSpeeds = SimpleNamespace(fl=v, fr=v, rl=v, rr=v)
    out.append(dict(send(camera, controller, CS)))
  return out


@pytest.mark.parametrize("camera", [True, False])
def test_default_packets_identical_to_constant_path(camera):
  # Reference: the original module constant path (a controller whose param read always fails).
  reference = run_packets(camera, CanfdStopping(params=FakeParams(RuntimeError("x"))))
  default = run_packets(camera, CanfdStopping(params=FakeParams(70)))
  missing = run_packets(camera, CanfdStopping(params=FakeParams(None)))
  assert default == reference == missing
  assert any(p["StopReq"] == 1 for p in default) and any(p["StopReq"] == 0 for p in default)


@pytest.mark.parametrize("camera", [True, False])
def test_raised_entry_speed_changes_only_stop_entry(camera):
  default = run_packets(camera, CanfdStopping(params=FakeParams(70)))
  raised = run_packets(camera, CanfdStopping(params=FakeParams(150)))
  first_default = next(i for i, p in enumerate(default) if p["StopReq"] == 1)
  first_raised = next(i for i, p in enumerate(raised) if p["StopReq"] == 1)
  assert SPEEDS[first_default] <= 0.7 < SPEEDS[first_default - 1]
  assert SPEEDS[first_raised] <= 1.5 < SPEEDS[first_raised - 1]
  assert first_raised < first_default


def test_stop_object_uses_controller_entry_speed():
  from opendbc.car.hyundai import stop_object
  assert stop_object.ENTRY_SPEED is stopping_mod.ENTRY_SPEED
  src = (ROOT / "opendbc_repo/opendbc/car/hyundai/stop_object.py").read_text()
  assert 'getattr(stopping_controller, "entry_speed", ENTRY_SPEED)' in src


def test_param_registered_and_in_experimental_menu():
  keys = (ROOT / "openpilot/common/params_keys.h").read_text()
  assert '{"K9CanfdEntrySpeed", {PERSISTENT, INT, "70"}}' in keys
  d = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text())
  text = (ROOT / "openpilot/selfdrive/carrot_settings.json").read_text()
  exp = _find_group(d, "EXPERIMENTAL")
  assert exp is not None
  assert exp["params"][:2] == ["K9StopMode", "K9CanfdEntrySpeed"]
  items = [p for p in d["params"] if p["name"] == "K9CanfdEntrySpeed"]
  assert len(items) == 1
  p = items[0]
  assert "CAN-FD 정차 진입 속도" in p["title"]
  assert p["default"] == 70 and p["min"] == 20 and p["max"] == 200 and p["unit"] == 5
  for k in ("title", "descr", "etitle", "edescr", "ctitle", "cdescr", "egroup", "cgroup", "group"):
    assert p[k].strip()
  assert "CAN-FD" in p["etitle"] and "CAN-FD" in p["ctitle"]
  assert text.count('"name": "K9CanfdEntrySpeed"') == 1


def _find_group(node, gid):
  if isinstance(node, dict):
    if node.get("id") == gid and "params" in node:
      return node
    for v in node.values():
      r = _find_group(v, gid)
      if r is not None:
        return r
  elif isinstance(node, list):
    for v in node:
      r = _find_group(v, gid)
      if r is not None:
        return r
  return None


def test_k9_stop_mode_ramp_speed_untouched():
  # K9 brake menu ramp start (1.5 km/h) is a different value and code; it must not reference this param.
  for path in (ROOT / "openpilot").rglob("k9_creep*.py"):
    assert "K9CanfdEntrySpeed" not in path.read_text()
