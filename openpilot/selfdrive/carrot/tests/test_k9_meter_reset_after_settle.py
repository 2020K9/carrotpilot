"""K9ExtHud=1 C1 meter, 보노보노 2026-10-31 23:04 "정산하고 시작 누르면 초기화 안 된다": 정산 -> 시작 = a new drive from 0
(base fare, 0 km, 0 s, empty breakdown), 일시정지 -> 재개 keeps the drive; and a 시작 pressed while the HUD is down
(it exits offroad) is applied when the HUD comes back, never an old press again. New tests only."""
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "cluster"))

import main as cluster_main
from cluster_taxi_c1 import C1MeterPanel, MeterButtonLink


class _St:
  def __init__(self, v):
    self.speed_kph, self.onroad, self.steering_angle_deg, self.lfa_active, self.active_lane_line = v, True, 0.0, True, True


class _Mem:
  def __init__(self, **kv):
    self.kv = dict(kv)

  def get(self, k):
    v = self.kv.get(k)
    return v.encode() if isinstance(v, str) else v

  def put(self, k, v):
    self.kv[k] = v


class _Hud:
  """One HUD process: a fresh C1 panel + MeterButtonLink on the memory params, 20 fps, live speed samples."""

  def __init__(self, mem, hour=23):
    self.clock = [1000.0]
    self.p = C1MeterPanel(clock=lambda: self.clock[0], local_hour=lambda: hour)
    self.r = SimpleNamespace(taxi_meter_command=self.p.command, taxi_meter_state=self.p.meter_status)
    self.src = SimpleNamespace(params_memory=mem)

  def run(self, secs, v, dt=0.05):
    for _ in range(int(round(secs / dt))):
      self.clock[0] += dt
      cluster_main.feed_taxi_meter_buttons(self.r, self.src, now=self.clock[0])
      self.p.feed_speed_samples([(self.clock[0], v / 3.6)])
      self.p.update(_St(v))


class _App:
  def __init__(self, mem):
    self.mem, self.n, self.q = mem, 0, []

  def press(self, cmd):
    self.n += 1
    self.q = (self.q + [f"ab{self.n:06d}:{cmd}"])[-8:]
    self.mem.put("K9MeterCmd", ";".join(self.q))


def _zero(p):
  m, b = p.meter, p.breakdown()
  return (p.meter_state, p.shown_fare(), m.distance_m, m.elapsed_s, m.units, b["dist_units"], b["time_units"])


def test_settle_then_start_is_a_new_drive_and_pause_resume_keeps_it():
  mem = _Mem()
  hud, app = _Hud(mem), _App(mem)
  hud.run(1, 0.0)
  app.press("start")
  hud.run(0.3, 0.0)
  base = hud.p.shown_fare()
  assert base == 6240                                 # 23 h: 4800 + 30 %
  hud.run(200, 60.0)
  app.press("pause")
  hud.run(0.3, 60.0)
  paused = (hud.p.shown_fare(), hud.p.meter.distance_m, hud.p.meter.elapsed_s)
  hud.run(30, 60.0)
  app.press("pause")                                  # 재개: the drive goes on from where it was
  hud.run(0.3, 60.0)
  assert hud.p.meter_state == "DRIVE" and hud.p.meter.distance_m >= paused[1] and hud.p.shown_fare() >= paused[0]
  hud.run(60, 60.0)
  assert hud.p.meter.distance_m == pytest.approx(paused[1] + 1000, rel=0.02)
  app.press("settle")
  hud.run(0.3, 60.0)
  s = hud.p.settle_breakdown
  assert hud.p.meter_state == "SETTLE" and s["total"] > base and s["night_won"] > 0 and s["dist_units"] > 0
  hud.run(5, 0.0)
  app.press("start")
  hud.run(0.25, 0.0)                                  # standing: nothing more than the base fare yet
  st, fare, km, sec, units, du, tu = _zero(hud.p)
  assert (st, fare, units, du, tu) == ("DRIVE", base, 0, 0, 0) and km == 0.0 and sec < 0.3
  assert hud.p.settle_breakdown is None
  st = json.loads(mem.kv["K9MeterState"])
  assert st["state"] == "DRIVE" and st["fare"] == base and st["km"] == 0.0 and st["sec"] == 0


def test_start_pressed_while_the_hud_is_down_is_applied_when_it_comes_back():
  mem = _Mem()
  hud, app = _Hud(mem), _App(mem)
  hud.run(1, 0.0)
  app.press("start")
  hud.run(100, 60.0)
  app.press("settle")
  hud.run(1, 0.0)
  assert hud.p.meter_state == "SETTLE"
  # offroad: the HUD exits; K9MeterState stays as it was (정산, the old fare) and the app presses 시작
  app.press("start")
  hud2 = _Hud(mem)
  hud2.run(0.5, 0.0)
  st, fare, km, sec, units, du, tu = _zero(hud2.p)
  assert (st, fare, units, du, tu) == ("DRIVE", 6240, 0, 0, 0) and km == 0.0 and sec < 0.5
  assert json.loads(mem.kv["K9MeterState"])["state"] == "DRIVE"
  # the HUD comes back again with nothing new pressed: no old press is replayed (빈차, as before)
  hud2.run(30, 60.0)
  hud3 = _Hud(mem)
  hud3.run(1, 0.0)
  assert hud3.p.meter_state == "EMPTY"


def test_link_takes_only_the_presses_after_the_published_token():
  mem = _Mem(K9MeterCmd="a1:start;a2:settle;a3:start;a4:pause", K9MeterState=json.dumps({"state": "SETTLE", "tok": "a2"}))
  link = MeterButtonLink(mem)
  assert link.poll(0.0) == ["start", "pause"]
  assert link.poll(1.0) == []
  mem.kv["K9MeterState"] = json.dumps({"state": "SETTLE", "tok": "zz"})  # token no longer in the queue: nothing replayed
  assert MeterButtonLink(mem).poll(0.0) == []
  mem.kv["K9MeterState"] = "not json"
  assert MeterButtonLink(mem).poll(0.0) == []
