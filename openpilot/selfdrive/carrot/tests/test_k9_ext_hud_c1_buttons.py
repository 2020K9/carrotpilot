"""K9ExtHud=1 C1 meter remote (보노보노 2026-10-24 20:46 "(초기화)시작.일시정지.정산(정산화면따로만들기) 버튼만들어", 21:20 app) and
the passenger link removed at the end of a drive (20:43 "운행끝나고 승객연동되잇는거 지워"). No GL needed. New tests only."""
import re
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

import cluster_taxi_c1 as c1  # noqa: E402
from cluster_taxi_c1 import C1Fsm, C1MeterPanel, MeterButtonLink  # noqa: E402


class _St:
  def __init__(self, v):
    self.speed_kph, self.onroad, self.steering_angle_deg, self.lfa_active, self.active_lane_line = v, True, 0.0, True, True


def _panel(hour=12):
  clock = [1000.0]
  p = C1MeterPanel(clock=lambda: clock[0], local_hour=lambda: hour)
  return p, clock


def _run(p, clock, secs, v, dt=0.05):
  for _ in range(int(round(secs / dt))):
    clock[0] += dt
    p.feed_occupant(False, clock[0])   # the K9 car: 0x500 there, nobody in the passenger seat
    p.update(_St(v))


def _snap(p):
  m = p.meter
  return (p.shown_fare(), round(m.distance_m, 6), round(m.elapsed_s, 6), m.units)


def test_start_pause_resume_settle_start():
  p, clock = _panel()
  _run(p, clock, 2, 0.0)
  p.command("start")
  _run(p, clock, 0.05, 0.0)
  assert p.meter_state == "DRIVE" and p.shown_fare() == 4800 and p.meter.distance_m == 0.0
  _run(p, clock, 120, 60.0)                         # 2 km
  a = _snap(p)
  assert a[0] > 4800 and a[1] == pytest.approx(2000, rel=0.01)
  p.command("pause")
  _run(p, clock, 0.05, 60.0)
  paused = _snap(p)
  assert p.meter_state == "PAUSE"
  _run(p, clock, 90, 60.0)                          # driving on and standing while paused: nothing counts
  _run(p, clock, 60, 0.0)
  assert _snap(p) == paused
  p.command("pause")                                # resume
  _run(p, clock, 60, 60.0)
  b = _snap(p)
  assert p.meter_state == "DRIVE" and b[1] == pytest.approx(paused[1] + 1000, rel=0.01) and b[0] > paused[0]
  p.command("settle")
  _run(p, clock, 0.05, 60.0)
  assert p.meter_state == "SETTLE"
  s = _snap(p)
  _run(p, clock, 120, 60.0)                         # 정산 screen: frozen
  assert _snap(p) == s and p.settle_breakdown["total"] == s[0]
  p.command("start")                                # new drive from 0
  _run(p, clock, 0.05, 0.0)
  assert p.meter_state == "DRIVE" and p.shown_fare() == 4800 and p.meter.distance_m == 0.0 and p.meter.elapsed_s < 0.2
  assert p.settle_breakdown is None


def test_start_resets_a_running_drive_and_from_pause():
  p, clock = _panel()
  p.command("start")
  _run(p, clock, 60, 60.0)
  p.command("pause")
  _run(p, clock, 1, 60.0)
  p.command("start")
  _run(p, clock, 0.05, 0.0)
  assert p.meter_state == "DRIVE" and not p.paused and p.meter.distance_m == 0.0


def test_pause_and_settle_only_while_driving():
  p, clock = _panel()
  for cmd in ("pause", "settle"):
    p.command(cmd)
    _run(p, clock, 0.1, 0.0)
    assert p.meter_state == "EMPTY" and not p.paused
  p.command("bogus")
  _run(p, clock, 0.1, 0.0)
  assert p.meter_state == "EMPTY"


def test_breakdown_adds_up_day_and_night():
  for hour in (12, 23):
    p, clock = _panel(hour)
    p.command("start")
    _run(p, clock, 300, 60.0)                       # 5 km
    _run(p, clock, 150, 0.0)                        # 150 s standing = 5 time steps
    p.command("settle")
    _run(p, clock, 0.05, 0.0)
    b = p.settle_breakdown
    assert b["time_units"] >= 4 and b["dist_units"] >= 25
    assert b["time_units"] + b["dist_units"] == p.meter.units
    assert b["base"] + b["dist_won"] + b["time_won"] + b["night_won"] == b["total"] == p.shown_fare()
    if hour == 23:
      assert b["night_rate"] == pytest.approx(0.3) and b["night_won"] > 0
    else:
      assert b["night_won"] == 0


def test_passenger_no_longer_ends_or_restarts_a_drive():
  F = C1Fsm()
  sig = lambda v, occ: {"v_kph": v, "occ": occ, "fare": 5000, "tap": False}   # noqa: E731
  for _ in range(110):
    F.step(sig(0, True), 0.05)
  for _ in range(60):
    F.step(sig(30, True), 0.05)
  assert F.state == "DRIVE"
  for _ in range(400):                              # passenger out for 20 s, moving and stopped
    F.step(sig(30 if _ < 200 else 0, False), 0.05)
  assert F.state == "DRIVE" and F.pay_fare is None
  F.step({"v_kph": 0, "occ": False, "fare": 5000, "tap": True}, 0.05)   # 지불 only by hand now
  assert F.state == "PAY"
  for _ in range(150):                              # a passenger gets in while 지불: no 손님탐
    F.step(sig(0, True), 0.05)
  assert F.state in ("PAY", "EMPTY") and F.state != "BOARD"


def test_boarding_side_still_works_and_button_mode_ignores_it():
  F = C1Fsm()
  for _ in range(110):
    F.step({"v_kph": 0, "occ": True, "fare": 4800, "tap": False}, 0.05)
  assert F.state == "BOARD"


# ---- memory params link (HUD side) -------------------------------------------------------------------
class _Mem:
  def __init__(self, **kv):
    self.kv = dict(kv)
    self.puts = []

  def get(self, k):
    v = self.kv.get(k)
    return v.encode() if isinstance(v, str) else v

  def put(self, k, v):
    self.kv[k] = v
    self.puts.append((k, v))


def test_link_applies_new_tokens_in_order_and_ignores_the_start_value():
  mem = _Mem(K9MeterCmd="a000001:start;a000002:pause")   # left from before a restart
  link = MeterButtonLink(mem)
  assert link.poll(0.0) == []
  assert link.poll(1.0) == []
  mem.kv["K9MeterCmd"] = "a000001:start;a000002:pause;a000003:pause;a000004:settle"   # two presses between polls
  assert link.poll(1.05) == []                      # polled at most every 0.2 s
  assert link.poll(1.3) == ["pause", "settle"]
  assert link.poll(1.6) == []                       # same presses, not again
  mem.kv["K9MeterCmd"] = "a000003:pause;a000004:settle;a000005:explode;a000006:start"
  assert link.poll(2.0) == ["start"]
  mem.kv["K9MeterCmd"] = "x1:pause"                 # the single "token:cmd" form
  assert link.poll(2.5) == ["pause"]


def test_link_publishes_json_state_on_change_with_heartbeat():
  import json
  mem = _Mem()
  link = MeterButtonLink(mem, wall=lambda: 5000.0)
  link.publish({"state": "DRIVE", "fare": 4800, "km": 0.0}, now=0.0)
  link.publish({"state": "DRIVE", "fare": 4800, "km": 0.0}, now=0.1)      # same: nothing
  link.publish({"state": "DRIVE", "fare": 4800, "km": 0.01}, now=0.2)     # km only: waits for 0.5 s
  link.publish({"state": "DRIVE", "fare": 4900, "km": 0.13}, now=0.3)     # fare: at once
  link.publish({"state": "DRIVE", "fare": 4900, "km": 0.14}, now=0.9)     # km after 0.5 s
  link.publish({"state": "DRIVE", "fare": 4900, "km": 0.14}, now=3.0)     # heartbeat
  vals = [json.loads(v) for k, v in mem.puts]
  assert [(v["fare"], v["km"]) for v in vals] == [(4800, 0.0), (4900, 0.13), (4900, 0.14), (4900, 0.14)]
  assert all(v["ts"] == 5000.0 and k == "K9MeterState" for (k, _), v in zip(mem.puts, vals))


def test_after_a_reboot_the_meter_starts_empty():
  import json
  keys = (Path(__file__).resolve().parents[3] / "common" / "params_keys.h").read_text()
  for k in ("K9MeterCmd", "K9MeterState"):           # memory only: cleared on every manager start
    assert re.search(r'\{"%s", \{CLEAR_ON_MANAGER_START, STRING\}\}' % k, keys), k
  # even if a press were still there, the new HUD process does not apply it
  mem = _Mem(K9MeterCmd="999:start", K9MeterState='{"state": "DRIVE"}')
  p, clock = _panel()
  link = MeterButtonLink(mem)
  r = SimpleNamespace(taxi_meter_command=p.command, taxi_meter_state=p.meter_status)
  src = SimpleNamespace(params_memory=mem, _k9_meter_button_link=link)
  import main as cluster_main
  for i in range(10):
    cluster_main.feed_taxi_meter_buttons(r, src, now=float(i))
    _run(p, clock, 0.05, 0.0)
  assert p.meter_state == "EMPTY" and p.shown_fare() == 4800
  assert json.loads(mem.kv["K9MeterState"])["state"] == "EMPTY"
  mem.kv["K9MeterCmd"] = "999:start;1000:start"
  cluster_main.feed_taxi_meter_buttons(r, src, now=20.0)
  _run(p, clock, 0.05, 0.0)
  cluster_main.feed_taxi_meter_buttons(r, src, now=21.0)
  st = json.loads(mem.kv["K9MeterState"])
  assert p.meter_state == "DRIVE" and st["state"] == "DRIVE" and st["fare"] == 4800 and st["km"] == 0.0


def test_feed_skips_sources_without_memory_params():
  import main as cluster_main
  cluster_main.feed_taxi_meter_buttons(SimpleNamespace(), SimpleNamespace())                 # no crash, nothing
  cluster_main.feed_taxi_meter_buttons(SimpleNamespace(taxi_meter_command=lambda c: 1 / 0), SimpleNamespace())


def test_settle_and_pause_strings_are_in_the_atlas():
  cps = set(c1.c1_codepoints())
  for s in c1.SETTLE_STRINGS + (c1.PAUSE_PILL[0], c1.PAUSE_MSG):
    assert all(ord(ch) in cps for ch in s), s
