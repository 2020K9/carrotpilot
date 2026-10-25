"""K9ExtHud (실험중 > 외부 HUD 화면 선택) -- new tests, no existing test changed in behaviour.

(a) -1 (default) = old behaviour: effective HUD mode == ClusterHud, process gate == ClusterHud==1,
    reconciler never writes, renderer default never touches the taxi panel.
(b) 0/1 -> ClusterHud=0 (web save path and process-side reconciler)
(c) ClusterHud=1 -> K9ExtHud=-1
(d) process_config gate (one carrot_cluster process)
(e) fare unit tests (distance / time / low-speed time fare / night / rounding)
(f) render check lives in test_k9_ext_hud_render.py (needs a GL display)
"""
import json
import math
from pathlib import Path

import pytest

from openpilot.selfdrive.carrot import ext_hud
from openpilot.selfdrive.carrot.ext_hud import (
  ExtHudReconciler,
  apply_exclusive_after_set,
  effective_cluster_hud_mode,
  normalize_ext_hud,
  read_effective_cluster_hud_mode,
)

ROOT = Path(__file__).resolve().parents[4]
OP = ROOT / "openpilot"


class FakeParams:
  def __init__(self, **values):
    self.values = dict(values)
    self.writes = []

  def get_int(self, key, block=False):
    if key not in self.values:
      raise KeyError(key)
    return int(self.values[key])

  def put_int(self, key, value):
    self.writes.append((key, int(value)))
    self.values[key] = int(value)


# ---------- (a) default -1 is the old behaviour
@pytest.mark.parametrize("cluster", [-3, 0, 1, 2, 7])
def test_default_off_effective_mode_is_cluster_hud(cluster):
  for ext in (-1, None, "x", 2, 5, -7):
    assert effective_cluster_hud_mode(cluster, ext) == cluster
  p = FakeParams(ClusterHud=cluster, K9ExtHud=-1)
  assert read_effective_cluster_hud_mode(p) == cluster
  # params build without the K9ExtHud key -> also old behaviour
  assert read_effective_cluster_hud_mode(FakeParams(ClusterHud=cluster)) == cluster


def test_default_off_reconciler_never_writes():
  rec = ExtHudReconciler()
  p = FakeParams(ClusterHud=0, K9ExtHud=-1)
  for cluster in (0, 1, 0, 1, 2, 1):
    p.values["ClusterHud"] = cluster
    assert rec.step(p) is None
  assert p.writes == []
  assert apply_exclusive_after_set(p, "ClusterHud", 1) is None  # K9ExtHud already -1
  assert p.writes == []


def test_unreadable_cluster_hud_still_raises_like_get_int():
  with pytest.raises(KeyError):
    read_effective_cluster_hud_mode(FakeParams(K9ExtHud=1))


def test_only_minus1_0_1_values():
  assert [normalize_ext_hud(v) for v in (-1, 0, 1, 2, -2, "1", None, "a", 1.0)] == [-1, 0, 1, -1, -1, 1, -1, -1, 1]
  assert ext_hud.EXT_HUD_VALUES == (-1, 0, 1)


# ---------- (b)/(c) exclusivity, web save path helper
@pytest.mark.parametrize("ext", [0, 1])
def test_selecting_ext_hud_turns_cluster_hud_off(ext):
  p = FakeParams(ClusterHud=1, K9ExtHud=ext)
  assert apply_exclusive_after_set(p, "K9ExtHud", ext) == ("ClusterHud", 1, 0)
  assert p.values == {"ClusterHud": 0, "K9ExtHud": ext}
  assert read_effective_cluster_hud_mode(p) == 1  # the HUD keeps running (no restart)


@pytest.mark.parametrize("ext", [0, 1])
def test_cluster_hud_on_returns_ext_hud_to_minus1(ext):
  p = FakeParams(ClusterHud=1, K9ExtHud=ext)
  assert apply_exclusive_after_set(p, "ClusterHud", 1) == ("K9ExtHud", ext, -1)
  assert p.values == {"ClusterHud": 1, "K9ExtHud": -1}


def test_exclusive_noops():
  p = FakeParams(ClusterHud=0, K9ExtHud=1)
  assert apply_exclusive_after_set(p, "K9ExtHud", 1) is None  # ClusterHud already off
  assert apply_exclusive_after_set(p, "ClusterHud", 0) is None
  assert apply_exclusive_after_set(p, "K9ExtHud", -1) is None
  assert apply_exclusive_after_set(p, "ShowDebugUI", 1) is None
  assert p.writes == []


def test_reconciler_last_switched_on_wins():
  rec = ExtHudReconciler()
  p = FakeParams(ClusterHud=1, K9ExtHud=-1)
  assert rec.step(p) is None
  p.values["K9ExtHud"] = 1  # another writer (restore) turned K9ExtHud on
  assert rec.step(p) == "K9ExtHud=1 -> ClusterHud=0"
  assert p.values == {"ClusterHud": 0, "K9ExtHud": 1}
  p.values["ClusterHud"] = 1  # now ClusterHud is switched on
  assert rec.step(p) == "ClusterHud=1 turned on -> K9ExtHud=-1"
  assert p.values == {"ClusterHud": 1, "K9ExtHud": -1}
  # both on with no history (e.g. restored backup) -> K9ExtHud wins, same screen
  rec2 = ExtHudReconciler()
  q = FakeParams(ClusterHud=1, K9ExtHud=0)
  assert rec2.step(q) == "K9ExtHud=0 -> ClusterHud=0"
  assert q.values == {"ClusterHud": 0, "K9ExtHud": 0}


def test_never_both_on_random_sequence():
  import random
  rng = random.Random(7)
  rec = ExtHudReconciler()
  p = FakeParams(ClusterHud=0, K9ExtHud=-1)
  for _ in range(2000):
    key = rng.choice(["ClusterHud", "K9ExtHud"])
    value = rng.choice([0, 1]) if key == "ClusterHud" else rng.choice([-1, 0, 1])
    p.values[key] = value
    if rng.random() < 0.5:
      apply_exclusive_after_set(p, key, value)  # web path
    rec.step(p)  # process path
    assert not (p.values["ClusterHud"] == 1 and p.values["K9ExtHud"] in (0, 1))


def test_web_param_set_applies_exclusive(monkeypatch):
  from openpilot.selfdrive.carrot.server.features import params as feature
  calls = []
  monkeypatch.setattr(feature, "HAS_PARAMS", True)
  fake = FakeParams(ClusterHud=1, K9ExtHud=1)
  import openpilot.common.params as op_params
  monkeypatch.setattr(op_params, "Params", lambda: fake)
  monkeypatch.setattr(feature, "append_param_change", lambda *a, **k: calls.append((a, k)))
  monkeypatch.setattr(feature, "is_drive_engaged", lambda request: False)
  assert feature._apply_ext_hud_exclusive("K9ExtHud", 1, None) == ("ClusterHud", 1, 0)
  assert fake.values["ClusterHud"] == 0
  assert calls and calls[0][0][:3] == ("ClusterHud", 1, 0)
  assert feature._apply_ext_hud_exclusive("LongPitch", 1, None) is None


# ---------- (d) manager gate
def test_process_config_gate():
  from openpilot.system.manager.process_config import cluster_hud_active, managed_processes
  for cluster, ext, expected in [(0, -1, False), (1, -1, True), (2, -1, False), (0, 0, True), (0, 1, True),
                                 (1, 1, True), (0, 5, False), (2, 1, True)]:
    p = FakeParams(ClusterHud=cluster, K9ExtHud=ext)
    assert cluster_hud_active(p) is expected, (cluster, ext)
  assert cluster_hud_active(FakeParams(ClusterHud=1)) is True  # no K9ExtHud key
  assert cluster_hud_active(FakeParams()) is False
  names = [name for name in managed_processes if name == "carrot_cluster"]
  assert names == ["carrot_cluster"]  # one process for both settings


def test_hud_mode_reader_uses_effective_mode_and_taxi_flag():
  import sys
  sys.path.insert(0, str(OP / "selfdrive" / "carrot" / "cluster"))
  from main import ClusterHudModeParamReader
  reader = ClusterHudModeParamReader()
  reader._params = FakeParams(ClusterHud=1, K9ExtHud=-1)
  assert reader.read() == 1 and reader.read_taxi_meter() is False
  reader._params.values.update(ClusterHud=0, K9ExtHud=1)
  assert reader.read() == 1 and reader.read_taxi_meter() is True  # 1 -> no restart
  reader._params.values.update(K9ExtHud=0)
  assert reader.read() == 1 and reader.read_taxi_meter() is False
  reader._params.values.update(K9ExtHud=-1)
  assert reader.read() == 0  # HUD stops like ClusterHud 1 -> 0 did


def test_autorun_read_hud_mode():
  from openpilot.selfdrive.carrot import cluster_autorun
  p = FakeParams(ClusterHud=1, K9ExtHud=-1)
  assert cluster_autorun._read_hud_mode(p) == 1 and p.writes == []
  p.values.update(ClusterHud=0)
  assert cluster_autorun._read_hud_mode(p) == 0
  p.values.update(K9ExtHud=1)
  assert cluster_autorun._read_hud_mode(p) == 1


# ---------- settings catalog / keys / docs
def test_menu_and_keys():
  data = json.loads((OP / "selfdrive" / "carrot_settings.json").read_text(encoding="utf-8"))
  exp = [g for g in _walk(data["menu"]) if g.get("id") == "EXPERIMENTAL"]
  # 11:51 K9MeterSeason is the sub-option right under K9ExtHud (registration pin updated deliberately);
  # 2026-10-25 lane menu merge: the lane switches K9OneLaneHold / K9LaneCenter are K9LaneAssist (+ K9LaneCenterGain) before it
  assert len(exp) == 1 and exp[0]["params"][-4:] == ["K9LaneAssist", "K9LaneCenterGain", "K9ExtHud", "K9MeterSeason"]
  p = next(x for x in data["params"] if x["name"] == "K9ExtHud")
  assert (p["min"], p["max"], p["default"], p["control"]) == (-1, 1, -1, "select")
  assert all(len(p["options"][loc]) == 3 for loc in ("ko", "en", "zh"))
  for key in ("descr", "edescr", "cdescr"):
    assert "ClusterHud" in p[key] or "외부 HUD 활성화" in p[key]
  keys = (OP / "common" / "params_keys.h").read_text(encoding="utf-8")
  assert '{"K9ExtHud", {PERSISTENT, INT, "-1"}}' in keys
  for loc in ("ko", "en"):
    assert "K9ExtHud" in (ROOT / "docs" / "user" / loc / "settings.md").read_text(encoding="utf-8")


def _walk(node):
  if isinstance(node, dict):
    yield node
    for v in node.values():
      yield from _walk(v)
  elif isinstance(node, list):
    for v in node:
      yield from _walk(v)


# ---------- (e) fares
import sys  # noqa: E402
sys.path.insert(0, str(OP / "selfdrive" / "carrot" / "cluster"))
from cluster_taxi_meter import (  # noqa: E402
  TAXI_FARE_SIHEUNG, TaxiMeter, format_hms, format_won, night_surcharge_rate)


def drive(meter, seconds, kph, hour=12, dt=0.05):
  for _ in range(int(round(seconds / dt))):
    meter.step(dt, kph, hour)


def test_fare_table_values():
  d = TAXI_FARE_SIHEUNG["deluxe"]
  # 01:06: the old fixed 15 km/h threshold is gone; 기준속도 = 144 m / 35 s (= 14.81 km/h)
  assert (d.base_fare_won, d.base_distance_m, d.distance_unit_m, d.distance_unit_won, d.time_unit_s,
          d.time_unit_won) == (7000, 3000.0, 144.0, 200, 35.0, 200)
  assert d.time_fare_below_mps == pytest.approx(144.0 / 35.0)
  m = TAXI_FARE_SIHEUNG["midsize"]
  assert (m.base_fare_won, m.base_distance_m, m.distance_unit_m, m.distance_unit_won, m.time_unit_s, m.time_unit_won) == (
    4800, 1600.0, 131.0, 100, 30.0, 100)


def test_base_fare_then_distance_steps():
  m = TaxiMeter()
  assert m.fare_won == 7000
  drive(m, 3000 / (36 / 3.6), 36)  # exactly 3 km at 36 km/h
  assert m.fare_won == 7000
  drive(m, 144 / 10.0 + 0.05, 36)  # +144 m
  assert m.fare_won == 7200 and m.units == 1
  drive(m, 144 * 9 / 10.0, 36)
  assert m.fare_won == 9000 and m.units == 10
  assert abs(m.distance_m - (3000 + 144 * 10)) < 2.0


def test_low_speed_time_fare():
  m = TaxiMeter()
  drive(m, 3000 / (60 / 3.6), 60)  # base used up by distance
  start = m.fare_won
  drive(m, 35.0 * 4 + 0.05, 10.0)  # 10 km/h = time fare: 4 x 35 s
  assert m.fare_won == start + 800
  drive(m, 35.0 * 2 + 0.05, 0.0)  # stopped also counts time
  assert m.fare_won == start + 1200
  assert m.distance_m == pytest.approx(3000 + 10 / 3.6 * 140.05, abs=2.0)


def test_partial_simultaneous_not_double():
  # above 15 km/h only distance counts: 30 s at 50 km/h = 416.7 m -> 2 steps after base, not 2 + time
  m = TaxiMeter()
  m.metered_m = 3000.0
  drive(m, 30.0, 50.0)
  assert m.units == 2 and m.fare_won == 7400
  # 01:06: 15 km/h is above 기준속도 14.81 km/h -> distance-charged (35.05 s = 146 m = 1 step either way)
  m2 = TaxiMeter(); m2.metered_m = 3000.0
  drive(m2, 35.0 + 0.05, 15.0)
  assert m2.units == 1


def test_low_speed_time_counts_inside_base():
  m = TaxiMeter()
  drive(m, 35.0 * (3000 / 144) + 35.0 + 0.05, 0.0)  # waiting only: base equivalent then one step
  assert m.fare_won == 7200


def test_night_surcharge_and_rounding():
  d = TAXI_FARE_SIHEUNG["deluxe"]
  assert [night_surcharge_rate(d, h) for h in (22, 23, 0, 3, 4, 12)] == [0.0, 0.2, 0.2, 0.2, 0.0, 0.0]
  mid = TAXI_FARE_SIHEUNG["midsize"]
  # res 10/24: 경기도 공고 제2023-5572호 원문 = 23:00~04:00 30% (one band; the old pin was the blog table 0.2/0.3/0.2)
  assert [night_surcharge_rate(mid, h) for h in (23, 0, 1, 2, 3, 4)] == [0.3, 0.3, 0.3, 0.3, 0.3, 0.0]
  m = TaxiMeter()
  m.reset(hour=23)
  assert m.fare_won == 8400
  m.metered_m = 3000.0
  drive(m, 14.45, 36.0, hour=23)
  assert m.fare_won == 8640
  mm = TaxiMeter(mid)
  mm.reset(hour=1)
  assert mm.fare_won == 6240  # 4800 * 1.3
  mm.metered_m = 1600.0
  drive(mm, 13.15, 36.0, hour=1)
  assert mm.fare_won == 6370  # +130 (10 won display)


def test_realtime_update_restarts_each_drive_and_clamps_dt():
  m = TaxiMeter()
  assert m.update(0.0, 50.0, True, 12) == 0 and m.fare_won == 7000
  m.update(100.0, 50.0, True, 12)  # 100 s gap (frame stall) -> 01:06: dropped (gaps > 0.5 s are not counted)
  assert m.elapsed_s == pytest.approx(0.0)
  m.update(100.5, 50.0, False, 12)  # offroad holds the value
  held = (m.fare_won, m.distance_m)
  m.update(200.0, 50.0, False, 12)
  assert (m.fare_won, m.distance_m) == held
  m.update(300.0, 0.0, True, 12)  # new drive -> base fare again
  assert m.fare_won == 7000 and m.distance_m == 0.0
  m.update(300.5, float("nan"), True, 12)
  assert math.isfinite(m.metered_m)


def test_formatting():
  assert format_won(14400) == "14,400"
  assert format_hms(18 * 60 + 32) == "00:18:32"
  assert format_hms(3 * 3600 + 5) == "03:00:05"


def test_renderer_default_has_no_taxi_panel():
  import sys
  sys.path.insert(0, str(OP / "selfdrive" / "carrot" / "cluster"))
  from cluster_renderer import ClusterUiRenderer
  r = ClusterUiRenderer(language="ko", theme_mode="dark")
  assert r.taxi_meter_enabled is False and r._taxi_meter_panel is None
  for mode in (-1, 0, 1, 2, 3, 4, 5, 6):
    assert r._taxi_meter_panel_active(mode) is False
  r.set_taxi_meter_enabled(True)
  assert [r._taxi_meter_panel_active(mode) for mode in (-1, 0, 1, 2, 3, 4, 5, 6)] == [False, True, True, True, False, True, True, False]
