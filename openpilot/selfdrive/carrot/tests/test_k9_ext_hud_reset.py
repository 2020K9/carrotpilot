"""K9ExtHud=1 taxi meter reset (00:58 지시). New tests, no GL.

- K9ExtHud non-1 -> 1 (seen by the periodic param read) always restarts the meter:
  fare 7,000 (base), distance 0, time 0, then counts again from the base fare.
- While K9ExtHud stays 1 the count is kept (current design: a new drive = onroad rising edge
  still starts from the base fare, unchanged).
- The HUD process restarting while K9ExtHud is 0 (or -1) starts with no meter; nothing is stored
  on disk, so there is nothing stale to carry over.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[4]
CLUSTER_DIR = ROOT / "openpilot" / "selfdrive" / "carrot" / "cluster"
if str(CLUSTER_DIR) not in sys.path:
  sys.path.insert(0, str(CLUSTER_DIR))

from cluster_renderer import ClusterUiRenderer  # noqa: E402
from main import ClusterHudModeParamReader, sync_taxi_meter_panel  # noqa: E402

BASE_FARE = 7000


class FakeParams:
  def __init__(self, **values):
    self.values = dict(values)

  def get_int(self, key, block=False):
    if key not in self.values:
      raise KeyError(key)
    return int(self.values[key])

  def put_int(self, key, value):
    self.values[key] = int(value)

  def get(self, key, block=False):
    return str(self.values[key]).encode() if key in self.values else None


class State:
  def __init__(self, speed_kph=60.0, onroad=True):
    self.speed_kph = speed_kph
    self.onroad = onroad


class Rig:
  """Renderer + periodic K9ExtHud read, like the cluster main loop (no GL)."""

  def __init__(self, ext_hud: int):
    self.params = FakeParams(ClusterHud=0, K9ExtHud=ext_hud)
    self.reader = ClusterHudModeParamReader()
    self.reader._params = self.params
    self.renderer = ClusterUiRenderer(language="ko", theme_mode="dark")
    self.clock = [1000.0]
    if self.reader.read_taxi_meter():           # main(): "K9ExtHud initial: 1"
      self.renderer.set_taxi_meter_enabled(True)
    self._fix_clock()

  def _fix_clock(self):
    panel = self.renderer._taxi_meter_panel
    if panel is not None:
      panel._clock = lambda: self.clock[0]
      panel._local_hour = lambda: 12

  def poll(self):
    self.reader.read()
    changed = sync_taxi_meter_panel(self.renderer, self.reader.read_taxi_meter())
    self._fix_clock()
    return changed

  def drive(self, seconds, speed_kph=60.0, onroad=True, dt=0.5):
    for _ in range(int(round(seconds / dt))):
      self.clock[0] += dt
      if self.renderer.taxi_meter_enabled and self.renderer._taxi_meter_panel is not None:
        self.renderer._taxi_meter_panel.update(State(speed_kph, onroad))   # = _prepare_taxi_meter_cache

  def set(self, value):
    self.params.values["K9ExtHud"] = value

  @property
  def meter(self):
    return self.renderer._taxi_meter_panel.meter


def _assert_fresh(meter):
  assert meter.fare_won == BASE_FARE
  assert meter.distance_m == 0.0
  assert meter.elapsed_s == 0.0
  assert meter.units == 0


def _assert_counted(meter):
  assert meter.fare_won > BASE_FARE and meter.distance_m > 3000.0 and meter.elapsed_s > 100.0


@pytest.mark.parametrize("off_value", [0, -1])
def test_1_off_1_resets_the_meter(off_value):
  rig = Rig(1)
  rig.drive(300.0)                       # 5 min at 60 km/h = 5 km
  _assert_counted(rig.meter)
  rig.set(off_value)
  assert rig.poll() is True
  rig.drive(30.0)                        # meter not counting while off
  rig.set(1)
  assert rig.poll() is True
  _assert_fresh(rig.meter)               # 1 -> 0 -> 1: fare 7000, distance 0, time 0
  assert rig.renderer._taxi_meter_panel.anim.phase == 0.0
  rig.drive(60.0)                        # counts again from the base fare
  m = rig.meter
  assert m.distance_m == pytest.approx(60.0 / 3.6 * 59.5, rel=1e-6)   # first onroad frame starts the drive
  assert m.elapsed_s == pytest.approx(59.5)
  assert m.fare_won == BASE_FARE         # 1 km < the 3 km base distance


def test_reset_shows_base_fare_even_when_offroad():
  rig = Rig(1)
  rig.drive(300.0)
  rig.drive(5.0, onroad=False)           # parked: meter holds the last value (current design)
  _assert_counted(rig.meter)
  rig.set(0)
  rig.poll()
  rig.set(1)
  rig.poll()
  _assert_fresh(rig.meter)
  rig.drive(10.0, onroad=False)          # still parked: stays at the base fare, no counting
  _assert_fresh(rig.meter)
  rig.drive(10.0, speed_kph=0.0)         # drive starts (onroad), stopped: time fare from 0
  assert rig.meter.elapsed_s == pytest.approx(9.5) and rig.meter.fare_won == BASE_FARE


def test_value_held_at_1_keeps_counting():
  rig = Rig(1)
  checkpoints = []
  for _ in range(10):                    # many periodic reads, value stays 1
    rig.drive(60.0)
    assert rig.poll() is False
    checkpoints.append((rig.meter.fare_won, rig.meter.distance_m, rig.meter.elapsed_s))
  for a, b in zip(checkpoints, checkpoints[1:]):
    assert b[0] >= a[0] and b[1] > a[1] and b[2] > a[2]
  assert rig.meter.distance_m == pytest.approx(60.0 / 3.6 * (600.0 - 0.5), rel=1e-6)
  rig.renderer.set_taxi_meter_enabled(True)   # a repeated "1" never resets
  assert rig.meter.distance_m == pytest.approx(60.0 / 3.6 * (600.0 - 0.5), rel=1e-6)


def test_value_held_at_1_new_drive_still_starts_from_base_fare():
  """Unchanged design: with K9ExtHud kept at 1, each drive (onroad rising edge) starts at the base fare."""
  rig = Rig(1)
  rig.drive(300.0)
  _assert_counted(rig.meter)
  rig.drive(10.0, onroad=False)
  _assert_counted(rig.meter)             # parked: last value held
  rig.drive(10.0)
  assert rig.meter.elapsed_s == pytest.approx(9.5) and rig.meter.fare_won == BASE_FARE


def test_0_to_1_starts_fresh_and_hud_restart_while_0_keeps_nothing():
  rig = Rig(0)
  assert rig.renderer._taxi_meter_panel is None
  rig.set(1)
  assert rig.poll() is True
  _assert_fresh(rig.meter)
  rig.drive(300.0)
  _assert_counted(rig.meter)
  rig.set(0)
  rig.poll()
  _assert_fresh(rig.meter)               # count dropped as soon as 0 is seen
  # HUD process dies and comes back while 0: new process, no meter at all, then 1 -> fresh
  rig2 = Rig(0)
  rig2.params = rig.params
  rig2.reader._params = rig.params
  assert rig2.renderer._taxi_meter_panel is None
  rig2.set(1)
  rig2.poll()
  _assert_fresh(rig2.meter)


def test_minus1_to_1_process_start_is_fresh():
  rig = Rig(1)                           # -1 -> 1 starts the HUD process: initial read = 1
  _assert_fresh(rig.meter)
  rig.drive(120.0)
  assert rig.meter.distance_m > 0


def test_menu_text_mentions_the_reset():
  import json
  data = json.loads((ROOT / "openpilot" / "selfdrive" / "carrot_settings.json").read_text(encoding="utf-8"))
  entry = next(p for p in data["params"] if p["name"] == "K9ExtHud")
  assert "0으로 바꿨다 1로 하면 미터기 초기화" in entry["descr"]
  assert "Switching to 0 and back to 1 resets the meter" in entry["edescr"]
  assert "改为 0 再改回 1 即重置计价器" in entry["cdescr"]
