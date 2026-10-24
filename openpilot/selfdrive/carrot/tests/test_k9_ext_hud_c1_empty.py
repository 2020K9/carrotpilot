"""K9ExtHud=1 C1 meter, meter_empty2 (보노보노 2026-10-24 21:59 "손님탐도없애", 22:01 "빈차창도 따로만들고 어플에 빈차도
넣어바"): no passenger-sensor link at all (no 손님탐), the meter starts in 빈차 and only the app's buttons change it, and
the new 빈차 button (back to 빈차 = reset) through K9MeterCmd / UDP 7720. No GL needed. New tests only."""
import json
import sys
from pathlib import Path

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

import cluster_taxi_c1 as c1  # noqa: E402
from cluster_taxi_c1 import C1Fsm, C1MeterPanel, MeterButtonLink  # noqa: E402


class _St:
  def __init__(self, v, onroad=True):
    self.speed_kph, self.onroad, self.steering_angle_deg, self.lfa_active, self.active_lane_line = v, onroad, 0.0, True, True


def _panel(hour=12, **kw):
  clock = [1000.0]
  p = C1MeterPanel(clock=lambda: clock[0], local_hour=lambda: hour, **kw)
  return p, clock


def _run(p, clock, secs, v, occ=None, dt=0.05, states=None):
  for _ in range(int(round(secs / dt))):
    clock[0] += dt
    if occ is not None:
      p.feed_occupant(occ, clock[0])
    p.update(_St(v))
    if states is not None:
      states.add(p.meter_state)


def _fresh(p):
  m = p.meter
  return m.distance_m == 0.0 and m.elapsed_s == 0.0 and m.units == 0 and p.shown_fare() == p.base_fare_won


def test_sensor_off_and_the_meter_starts_in_empty():
  assert c1.OCCUPANT_ENABLED is False and C1Fsm().occ_on is False
  p, clock = _panel()
  assert p.fsm_mode and p.display_state == "EMPTY" and p.meter_state == "EMPTY"
  seen = set()
  _run(p, clock, 6.0, 0.0, occ=True, states=seen)      # passenger in, held > 5 s while stopped
  _run(p, clock, 60.0, 60.0, occ=True, states=seen)    # and driving away with them
  _run(p, clock, 8.0, 0.0, occ=False, states=seen)     # out again
  _run(p, clock, 30.0, 40.0, states=seen)              # onroad, driving, no 0x500 at all
  assert seen == {"EMPTY"} and _fresh(p) and p.shown_fare() == 4800
  assert p.occupant is None                            # the sensor value is not even kept


def test_start_is_drive_at_once_and_never_board():
  p, clock = _panel()
  seen = set()
  p.command("start")
  _run(p, clock, 0.05, 0.0, states=seen)
  assert p.meter_state == "DRIVE" and p.fsm.ev == "start"
  _run(p, clock, 120.0, 50.0, occ=True, states=seen)
  _run(p, clock, 10.0, 0.0, occ=False, states=seen)
  assert seen == {"DRIVE"} and p.meter.distance_m > 1500


@pytest.mark.parametrize("before", ["DRIVE", "PAUSE", "SETTLE", "EMPTY"])
def test_empty_button_goes_back_to_empty_and_resets(before):
  p, clock = _panel(hour=23)
  if before != "EMPTY":
    p.command("start")
    _run(p, clock, 200.0, 60.0)
    assert p.meter.distance_m > 3000 and p.shown_fare() > p.base_fare_won
    if before == "PAUSE":
      p.command("pause")
    if before == "SETTLE":
      p.command("settle")
    _run(p, clock, 0.05, 60.0)
    assert p.meter_state == before
  p.command("empty")
  _run(p, clock, 0.05, 60.0)
  assert p.meter_state == "EMPTY" and p.fsm.ev == "empty" and not p.paused and p.settle_breakdown is None
  assert _fresh(p) and p.shown_fare() == 6240        # 23 h: base 4,800 + 심야 30 %
  _run(p, clock, 60.0, 60.0)                           # nothing counts in 빈차
  assert _fresh(p) and p.meter_state == "EMPTY"
  p.command("start")                                   # 시작 from 빈차: from 0
  _run(p, clock, 10.0, 0.0)
  assert p.meter_state == "DRIVE" and p.meter.elapsed_s == pytest.approx(10.0 - 0.05, abs=0.06)


def test_pause_and_settle_do_nothing_in_empty():
  p, clock = _panel()
  for cmd in ("pause", "settle", "pause"):
    p.command(cmd)
    _run(p, clock, 1.0, 30.0)
    assert p.meter_state == "EMPTY" and _fresh(p) and p.settle_breakdown is None


def test_toggle_is_the_apps_next_button():
  p, clock = _panel()
  order = []
  for _ in range(4):
    p.toggle()
    _run(p, clock, 30.0, 40.0)
    order.append(p.meter_state)
  assert order == ["DRIVE", "SETTLE", "EMPTY", "DRIVE"]
  assert p.meter.distance_m == pytest.approx(40 / 3.6 * (30.0 - 0.05), rel=1e-3)   # the last 시작 counts from 0


def test_sensor_on_option_keeps_the_old_boarding_logic():
  p, clock = _panel(occupant=True)
  assert not p.fsm_mode
  _run(p, clock, 5.3, 0.0, occ=True)
  assert p.fsm.state == "BOARD"
  p.restart()
  assert p.fsm.occ_on and not p.fsm_mode


def test_restart_keeps_the_sensor_off_meter():
  p, clock = _panel()
  p.command("start")
  _run(p, clock, 30.0, 40.0)
  p.restart()
  assert p.fsm_mode and p.meter_state == "EMPTY" and _fresh(p) and not p.fsm.occ_on


def test_empty_bubbles():
  p, _ = _panel()
  p.fsm.ev = "empty"
  assert p.empty_bubble(1.0) == ("또 만나요!", 1.0)
  assert p.empty_bubble(4.0)[1] == 0.0 and p.empty_bubble(12.0)[1] == 0.0
  for since in (7.0, 21.0, 35.0):
    assert p.empty_bubble(since) == (c1.EMPTY_CALL[0], 1.0)
  p.fsm.ev = ""                                        # HUD start (no button yet): no '또 만나요!'
  assert p.empty_bubble(1.0)[1] == 0.0 and p.empty_bubble(7.0)[0] == c1.EMPTY_CALL[0]
  # one bubble at a time: the two windows never overlap
  p.fsm.ev = "empty"
  for i in range(600):
    s = i * 0.05
    a = c1._win(s, c1.SAY["EMPTY"][1], c1.SAY["EMPTY"][2])
    b = c1._win(c1.EMPTY_CALL[1] + (s - c1.EMPTY_CALL[1]) % c1.EMPTY_CALL[3], c1.EMPTY_CALL[1], c1.EMPTY_CALL[2]) \
      if s >= c1.EMPTY_CALL[1] else 0.0
    assert not (a > 0 and b > 0), s


def test_every_empty_screen_glyph_is_in_the_atlas():
  cps = set(c1.c1_codepoints())
  for s in c1.EMPTY_STRINGS + ("기본요금", "거리요금", "시간요금", "심야 할증"):
    assert all(ord(ch) in cps for ch in s), s


def test_link_and_receiver_know_the_empty_command():
  from openpilot.selfdrive.carrot import k9_meter_link as kml
  assert "empty" in c1.METER_COMMANDS and "empty" in kml.COMMANDS

  class _Mem:
    def __init__(self):
      self.kv = {}

    def get(self, k):
      v = self.kv.get(k)
      return v.encode() if isinstance(v, str) else v

    def put(self, k, v):
      self.kv[k] = v

  class _P:
    def get_int(self, k):
      return 1

  mem = _Mem()
  s = kml.K9MeterServer(_P(), mem, clock=lambda: 10.0)
  link = MeterButtonLink(mem)
  assert link.poll(0.0) == []
  r = json.loads(s.handle(json.dumps({"k9meter": 1, "cmd": "empty", "id": 3, "cid": "a"}).encode()))
  assert r["ok"] and r["applied"] is True
  r = json.loads(s.handle(json.dumps({"k9meter": 1, "cmd": "empty", "id": 3, "cid": "a"}).encode()))
  assert r["applied"] is False                         # a resend is not applied twice
  assert link.poll(1.0) == ["empty"]


def test_main_does_not_start_the_occupant_reader():
  import main as cluster_main

  class Src:
    polled = 0

    def drain_taxi_speed_samples(self):
      return []

    def poll_taxi_occupant(self):
      Src.polled += 1
      return (1.0, True)

  class R:
    taxi_meter_enabled = True

    def feed_taxi_meter_speed_samples(self, s):
      pass

    def feed_taxi_meter_occupant(self, present, t_s=None):
      raise AssertionError("fed")

  for _ in range(5):
    cluster_main.feed_taxi_meter_speed(R(), Src())
  assert Src.polled == 0 and cluster_main.taxi_occupant_enabled() is False
