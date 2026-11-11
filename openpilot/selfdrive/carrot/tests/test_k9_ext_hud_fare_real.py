"""K9ExtHud=1 taxi meter = real meter counting (01:06 지시). New tests, no GL.

경기 모범·대형 (경기도 공식 보도 gnews.gg.go.kr number=57519, 2023-07-01 04시~): 기본 7,000원/3km,
144m당 200원, 35초당 200원. 시간·거리 부분 동시병산: 기준속도 = 144 m / 35 s (4.114 m/s, 14.81 km/h);
v >= 기준속도 -> distance only, v < 기준속도 (stopped included) -> time only (dt * 144/35 m), one 환산거리
counter, base 3 km consumed by it, then one unit per 144 m of it. Speed samples = carState.vEgo at its
received rate (100 Hz here), gaps > 0.5 s dropped. Night 23~04 = 모범 20% (공식 모범 할증률 미확인).
Shown fare rounded to 10 won.
"""
from __future__ import annotations

from datetime import datetime
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
if str(CLUSTER_DIR) not in sys.path:
  sys.path.insert(0, str(CLUSTER_DIR))

from cluster_taxi_meter import (  # noqa: E402
  SEOUL_TZ, TAXI_FARE_SIHEUNG, TAXI_METER_FOOTER_TEXT, TAXI_METER_MAX_DT_S, TAXI_NIGHT_RATE_DELUXE, TaxiFare, TaxiMeter,
  TaxiMeterPanel, seoul_hour)

D = TAXI_FARE_SIHEUNG["deluxe"]
THR = 144.0 / 35.0          # m/s
HZ = 100                    # carState rate


def kst(y, mo, d, h, mi, s=0):
  return datetime(y, mo, d, h, mi, s, tzinfo=SEOUL_TZ).timestamp()


class Feed:
  """carState-like samples into a meter: t advances by exactly 1/HZ (integer sample index)."""

  def __init__(self, meter=None, hour_at=lambda t: 12, t0=0.0):
    self.m = meter or TaxiMeter()
    self.k = 0
    self.t0 = t0
    self.hour_at = hour_at
    self.ticks = []
    self.m.reset(self.hour_at(self.t))
    self.m._was_onroad = True
    self.m.feed_sample(self.t, 0.0, self.hour_at(self.t))   # time base

  @property
  def t(self):
    return self.t0 + self.k / HZ

  def run(self, n, v_mps):
    for _ in range(int(n)):
      self.k += 1
      before = self.m.fare_won_exact
      self.m.feed_sample(self.t, v_mps, self.hour_at(self.t), self.t)
      if self.m.fare_won_exact != before:
        self.ticks.append((self.t, round(self.m.fare_won_exact - before, 6)))


# ---------- 기준속도 from the two values
def test_threshold_is_computed_from_unit_distance_and_time():
  assert D.time_fare_below_mps == pytest.approx(4.114285714)
  assert D.time_fare_below_kph == pytest.approx(14.8114286)
  other = TaxiFare("x", 1000, 1000.0, 131.0, 100, 30.0, 100, ())
  assert other.time_fare_below_mps == pytest.approx(131.0 / 30.0)
  assert "시간·거리 동시병산(14.8km/h 미만 시간)" in TAXI_METER_FOOTER_TEXT


# ---------- (a) 60 km/h: exactly 3 km = 7,000, +144 m = 7,200
def test_a_60kph_3km_exact_then_144m():
  f = Feed()
  v = 60.0 / 3.6
  f.run(180 * HZ, v)                         # 180 s = 3,000 m
  assert f.m.distance_m == pytest.approx(3000.0, abs=1e-6)
  assert f.m.fare_won == 7000 and f.m.units == 0 and f.m.mode == "distance"
  f.run(864 - 1, v)                          # 8.63 s = 143.83 m
  assert f.m.fare_won == 7000
  f.run(1, v)                                # 8.64 s = 144 m
  assert f.m.metered_m == pytest.approx(3144.0, abs=1e-6)
  assert f.m.fare_won == 7200 and f.m.units == 1


# ---------- (b) stopped 35 s = 1 unit
def test_b_stopped_35s_is_one_unit():
  f = Feed()
  f.run(180 * HZ, 60.0 / 3.6)                # base used up by distance
  f.run(35 * HZ - 1, 0.0)
  assert f.m.fare_won == 7000 and f.m.mode == "time"
  f.run(1, 0.0)
  assert f.m.fare_won == 7200 and f.m.units == 1
  assert f.m.distance_m == pytest.approx(3000.0, abs=1e-6)   # 운행 거리 = real distance only
  assert f.m.elapsed_s == pytest.approx(215.0)


# ---------- (c) both sides of 14.81 km/h
@pytest.mark.parametrize("factor,mode", [(0.999, "time"), (1.0, "distance"), (1.001, "distance"), (0.5, "time"),
                                          (14.8 / 3.6 / THR, "time"), (14.82 / 3.6 / THR, "distance")])
def test_c_threshold_both_sides(factor, mode):
  v = THR * factor
  f = Feed()
  f.m.metered_m = 3000.0
  f.run(10 * HZ, v)
  assert f.m.mode == mode
  per_s = (f.m.metered_m - 3000.0) / 10.0
  if mode == "time":
    assert per_s == pytest.approx(THR)        # time counts 144/35 m per s whatever the speed
  else:
    assert per_s == pytest.approx(v)          # distance counts the driven metres
  assert f.m.distance_m == pytest.approx(v * 10.0)


def test_c_no_double_count_and_continuous_at_the_threshold():
  # 35 s just below / at the threshold: both give exactly one unit (no 시간+거리 double charge)
  for v in (THR * 0.9999, THR):
    f = Feed()
    f.m.metered_m = 3000.0
    f.run(35 * HZ, v)
    assert f.m.units == 1 and f.m.fare_won == 7200, v
  # 20 km/h for 35 s = 194 m -> one unit by distance, not two
  f = Feed()
  f.m.metered_m = 3000.0
  f.run(35 * HZ, 20.0 / 3.6)
  assert f.m.units == 1 and f.m.mode == "distance"


# ---------- (d) waiting inside the base part consumes the base distance
def test_d_stop_time_inside_base_consumes_base_distance():
  base_wait_s = 3000.0 / THR                 # 729.17 s of waiting = 3 km 환산거리
  f = Feed()
  f.run(int(base_wait_s * HZ), 0.0)
  assert f.m.fare_won == 7000 and f.m.distance_m == 0.0
  f.run(int(35 * HZ) + 2, 0.0)
  assert f.m.fare_won == 7200
  # mixed: 1.5 km driving + 1.5 km worth of waiting = base used, then one more 35 s
  g = Feed()
  g.run(90 * HZ, 60.0 / 3.6)                 # 1,500 m
  g.run(int(1500.0 / THR * HZ) + 1, 0.0)     # 364.6 s
  assert g.m.fare_won == 7000 and g.m.metered_m == pytest.approx(3000.0, abs=0.05)
  g.run(35 * HZ, 0.0)
  assert g.m.fare_won == 7200


# ---------- (e) 22:59 -> 23:00 and 03:59 -> 04:00: the unit price at the moment the unit is added
@pytest.mark.parametrize("start,switch,before,after,base", [
  ((2026, 10, 23, 22, 50), (2026, 10, 23, 23, 0), 200.0, 240.0, 7000),
  ((2026, 10, 24, 3, 50), (2026, 10, 24, 4, 0), 240.0, 200.0, 8400)])
def test_e_unit_price_switches_at_the_moment(start, switch, before, after, base):
  t0 = kst(*start)
  f = Feed(hour_at=seoul_hour, t0=t0)
  assert f.m.fare_won == base                 # base fare = rate at the drive start
  f.run(20 * 60 * HZ, 50.0 / 3.6)             # 20 min at 50 km/h, crosses the switch at 10 min
  s = kst(*switch)
  pre = [w for t, w in f.ticks if t < s]
  post = [w for t, w in f.ticks if t >= s]
  assert len(pre) > 20 and len(post) > 20
  assert set(pre) == {before} and set(post) == {after}
  assert f.m.fare_won_exact == pytest.approx(base + before * len(pre) + after * len(post))
  assert f.m.fare_won % 10 == 0


# ---------- (f) night start = 8,400 (and the rate lives in one constant)
def test_f_night_start_8400():
  assert TAXI_NIGHT_RATE_DELUXE == 0.20 and D.night_surcharges == ((23, 4, 0.20),)
  # start, fare after one 35 s unit: the unit is priced at its own moment
  for hms, after in (((23, 0, 0), 8640), ((0, 30, 0), 8640), ((3, 59, 0), 8640), ((3, 59, 59), 8600)):
    f = Feed(hour_at=seoul_hour, t0=kst(2026, 10, 24 if hms[0] < 12 else 23, *hms))
    assert f.m.fare_won == 8400               # night start = 8,400
    f.m.metered_m = 3000.0
    f.run(35 * HZ + 1, 0.0)                   # (+1 sample: epoch-sized timestamps round the dt sum)
    assert f.m.fare_won == after, hms         # 240 at night; 03:59:59 start -> unit at 04:00:35 = 200
  for hms in ((22, 59, 59), (4, 0, 0)):
    f = Feed(hour_at=seoul_hour, t0=kst(2026, 10, 24 if hms[0] < 12 else 23, *hms))
    assert f.m.fare_won == 7000


# ---------- (g) dt stalls are ignored
def test_g_gaps_over_half_a_second_are_dropped():
  assert TAXI_METER_MAX_DT_S == 0.5
  f = Feed()
  f.m.metered_m = 3000.0
  f.run(10 * HZ, 60.0 / 3.6)
  d0, e0, m0 = f.m.distance_m, f.m.elapsed_s, f.m.metered_m
  f.k += int(0.6 * HZ) - 1                    # next sample 0.6 s later (message stall)
  f.run(1, 60.0 / 3.6)
  assert (f.m.distance_m, f.m.elapsed_s, f.m.metered_m) == (d0, e0, m0)
  assert f.m.dropped_gaps == 1
  f.k += int(0.5 * HZ) - 1                    # 0.5 s gap is still counted
  f.run(1, 60.0 / 3.6)
  assert f.m.elapsed_s == pytest.approx(e0 + 0.5)
  # same on the frame path and on step()
  m = TaxiMeter()
  assert m.step(0.51, 60.0) == 0 and m.elapsed_s == 0.0
  m.step(0.5, 60.0)
  assert m.elapsed_s == pytest.approx(0.5)
  m2 = TaxiMeter()
  m2.update(0.0, 30.0, True, 12)
  m2.update(0.7, 30.0, True, 12)              # 0.7 s frame gap: dropped
  m2.update(1.0, 30.0, True, 12)              # 0.3 s: counted
  assert m2.elapsed_s == pytest.approx(0.3)


def test_shown_fare_rounds_to_10_won():
  m = TaxiMeter()
  for exact, shown in ((7004.9, 7000), (7005.0, 7010), (7015.0, 7020), (8400.0, 8400), (8640.0, 8640), (7239.9, 7240)):
    m.fare_won_exact = exact
    assert m.fare_won == shown, exact


# ---------- live feed: every carState goes to the meter, not the frame-rate speed
class FakeRenderer:
  def __init__(self):
    self.taxi_meter_enabled = True
    self.fed = []

  def feed_taxi_meter_speed_samples(self, samples):
    self.fed.append(list(samples))


class FakeSource:
  def __init__(self, samples):
    self.samples = list(samples)
    self.stopped = 0

  def drain_taxi_speed_samples(self):
    out, self.samples = self.samples, []
    return out

  def stop_taxi_speed_samples(self):
    self.stopped += 1


def test_live_feed_helper_and_panel_integrates_every_sample():
  from main import feed_taxi_meter_speed
  r = FakeRenderer()
  src = FakeSource([(1.0 + k / HZ, 10.0) for k in range(11)])
  assert feed_taxi_meter_speed(r, src) == 11 and len(r.fed[0]) == 11
  r.taxi_meter_enabled = False
  assert feed_taxi_meter_speed(r, src) == 0 and src.stopped == 1   # meter off: socket closed
  assert feed_taxi_meter_speed(r, SimpleNamespace()) == 0          # simulator source: no feed

  clock = [100.0]
  panel = TaxiMeterPanel(clock=lambda: clock[0], local_hour=lambda: 12)
  state = SimpleNamespace(speed_kph=99.0, onroad=True)              # frame speed must NOT be used
  panel.feed_speed_samples([])
  panel.update(state)                                               # drive start
  for frame in range(50):                                           # 10 fps frames, 10 samples each
    panel.feed_speed_samples([(10.0 + (frame * 10 + k) / HZ, 10.0) for k in range(10)])
    clock[0] += 0.1
    panel.update(state)
  # 500 samples at 100 Hz -> 499 intervals of 0.01 s at 10 m/s
  assert panel.meter.elapsed_s == pytest.approx(4.99)
  assert panel.meter.distance_m == pytest.approx(49.9)


def test_live_reader_drains_every_carstate_and_closes():
  from cluster_live import TaxiSpeedSampleReader
  queue = []

  class Msg:
    def __init__(self, t_ns, v):
      self.logMonoTime = t_ns
      self.carState = SimpleNamespace(vEgo=v)

  class FakeMessaging:
    opened = []

    def sub_sock(self, name, conflate=True):
      FakeMessaging.opened.append((name, conflate))
      return object()

    def drain_sock(self, sock):
      out = list(queue)
      queue.clear()
      return out

  reader = TaxiSpeedSampleReader(FakeMessaging())
  assert reader.drain() == []                                       # first call opens the socket
  assert FakeMessaging.opened == [("carState", False)]              # not conflated: every message
  queue.extend(Msg(1_000_000_000 + k * 10_000_000, 5.0 + k) for k in range(5))
  got = reader.drain()
  assert [round(t, 3) for t, _ in got] == [1.0, 1.01, 1.02, 1.03, 1.04] and [v for _, v in got] == [5, 6, 7, 8, 9]
  reader.stop()
  assert reader._sock is None


def test_fare_mode_for_display():
  f = Feed()
  f.run(HZ, 30.0 / 3.6)
  assert f.m.mode == "distance"
  f.run(HZ, 3.0)
  assert f.m.mode == "time"
  f.m.metered_m = 3000.0 + 72.0
  f.m.units = 0
  assert f.m.next_step_remaining_s == pytest.approx(72.0 * 35.0 / 144.0)
