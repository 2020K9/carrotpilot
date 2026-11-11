"""K9ExtHud=1 taxi meter: night surcharge really applied (01:03 지시). New tests, no GL.

- Asia/Seoul device clock: 23:00~04:00 = base fare and every distance / time step +20%
  (deluxe: base 8,400, one step 240); the rate switches at 23:00 / 04:00 from that moment on,
  nothing already charged is re-priced.
- clock not synced (year < 2024) = no surcharge; the device time zone does not matter.
- lamp state ('심야 할증 20%') = panel.night_rate > 0; next-step amount 200 / 240.
Night numbers: 출처 블로그, 확인 필요 (see cluster_taxi_meter.py).
"""
from __future__ import annotations

from datetime import datetime
import os
import sys
import time
from pathlib import Path

import pytest

CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
if str(CLUSTER_DIR) not in sys.path:
  sys.path.insert(0, str(CLUSTER_DIR))

from cluster_taxi_meter import (  # noqa: E402
  SEOUL_TZ, TAXI_FARE_SIHEUNG, TaxiMeter, TaxiMeterPanel, night_surcharge_rate, seoul_hour)

DELUXE = TAXI_FARE_SIHEUNG["deluxe"]


def kst(y, mo, d, h, mi, s=0):
  return datetime(y, mo, d, h, mi, s, tzinfo=SEOUL_TZ).timestamp()


class State:
  def __init__(self, speed_kph=36.0, onroad=True):
    self.speed_kph = speed_kph
    self.onroad = onroad


class Drive:
  """Panel driven like the HUD: monotonic frame clock + wall clock (epoch) moving together."""

  def __init__(self, wall_start):
    self.mono = [500.0]
    self.wall0 = wall_start - self.mono[0]
    self.panel = TaxiMeterPanel(clock=lambda: self.mono[0], local_hour=lambda: seoul_hour(self.wall0 + self.mono[0]))
    self.ticks = []   # (wall epoch, won added)
    self.rates = []   # (wall epoch of the update, night rate in force)

  @property
  def wall(self):
    return self.wall0 + self.mono[0]

  def run(self, seconds, speed_kph=36.0, dt=0.5):
    m = self.panel.meter
    for _ in range(int(round(seconds / dt))):
      before = m.fare_won_exact
      self.panel.update(State(speed_kph))
      self.rates.append((self.wall, self.panel.night_rate))
      if m.fare_won_exact != before and m.started and self.ticks is not None:
        self.ticks.append((self.wall, round(m.fare_won_exact - before, 6)))
      self.mono[0] += dt

  def start(self, speed_kph=36.0):
    self.panel.update(State(speed_kph))   # onroad rising edge: base fare with the rate now
    self.ticks = []
    self.mono[0] += 0.5


def test_seoul_hour_boundaries_and_unsynced_clock():
  assert seoul_hour(kst(2026, 10, 23, 22, 59, 59)) == 22
  assert seoul_hour(kst(2026, 10, 23, 23, 0, 0)) == 23
  assert seoul_hour(kst(2026, 10, 24, 3, 59, 59)) == 3
  assert seoul_hour(kst(2026, 10, 24, 4, 0, 0)) == 4
  assert seoul_hour(kst(2024, 1, 1, 0, 0, 0)) == 0          # synced from 2024
  assert seoul_hour(kst(2023, 12, 31, 23, 59, 59)) is None   # year < 2024 = not synced
  assert seoul_hour(0.0) is None                             # 1970 (no RTC / no GPS time yet)
  assert seoul_hour(float("nan")) is None
  assert night_surcharge_rate(DELUXE, None) == 0.0


def test_device_time_zone_does_not_matter(monkeypatch):
  monkeypatch.setenv("TZ", "UTC")
  time.tzset()
  try:
    t = kst(2026, 10, 23, 23, 0, 0)          # = 14:00 UTC
    assert time.localtime(t).tm_hour == 14
    assert seoul_hour(t) == 23
  finally:
    monkeypatch.delenv("TZ", raising=False)
    time.tzset()


@pytest.mark.parametrize("start,base", [
  ((22, 59, 59), 7000), ((23, 0, 0), 8400), ((1, 30, 0), 8400), ((3, 59, 59), 8400), ((4, 0, 0), 7000), ((12, 0, 0), 7000)])
def test_base_fare_uses_the_rate_at_drive_start(start, base):
  d = Drive(kst(2026, 10, 23 if start[0] >= 12 else 24, *start))
  d.start()
  assert d.panel.meter.fare_won == base
  assert d.panel.night_rate == (0.2 if base == 8400 else 0.0)


def test_night_base_fare_is_8400_and_step_240():
  m = TaxiMeter()
  m.reset(hour=23)
  assert m.fare_won == 8400
  d = Drive(kst(2026, 10, 23, 23, 30))
  d.start()
  assert d.panel.meter.fare_won == 8400 and d.panel.step_won == 240
  d.run(450.0)                                # 3 km in base, then ~10 steps
  assert d.ticks and all(won == pytest.approx(240.0) for _, won in d.ticks)
  assert d.panel.meter.fare_won == 8400 + 240 * len(d.ticks)


def _crossing(start_hms, start_day, switch_epoch, before_won, after_won, base):
  d = Drive(kst(2026, 10, start_day, *start_hms))
  d.start()
  assert d.panel.meter.fare_won == base
  d.run(switch_epoch - d.wall - 1.0)           # up to 1 s before the switch
  fare_before = d.panel.meter.fare_won_exact
  rate_before = d.panel.night_rate
  d.run(2.0)                                   # cross it
  assert round(d.panel.meter.fare_won_exact - fare_before, 6) in (0.0, before_won, after_won)
  d.run(25 * 60.0)
  pre = [won for t, won in d.ticks if t < switch_epoch]
  post = [won for t, won in d.ticks if t >= switch_epoch]
  assert len(pre) >= 10 and len(post) >= 10
  assert all(w == pytest.approx(before_won) for w in pre), set(pre)
  assert all(w == pytest.approx(after_won) for w in post), set(post)
  # nothing already charged is re-priced: total = base + sum of the per-step prices of their moment
  assert d.panel.meter.fare_won_exact == pytest.approx(base + before_won * len(pre) + after_won * len(post))
  return rate_before, d.panel.night_rate, d.panel.step_won


def test_22_59_to_23_00_switches_to_night_price_from_then_on():
  r0, r1, step = _crossing((22, 40, 0), 23, kst(2026, 10, 23, 23, 0, 0), 200.0, 240.0, 7000)
  assert (r0, r1, step) == (0.0, 0.2, 240)   # lamp off before, on after


def test_03_59_to_04_00_switches_back_to_day_price():
  r0, r1, step = _crossing((3, 40, 0), 24, kst(2026, 10, 24, 4, 0, 0), 240.0, 200.0, 8400)
  assert (r0, r1, step) == (0.2, 0.0, 200)   # lamp on before, off after


def test_switch_happens_at_the_exact_second():
  switch = kst(2026, 10, 23, 23, 0, 0)
  d = Drive(switch - 2.0)
  d.start()
  d.run(4.0)
  assert [w - switch for w, _ in d.rates] == pytest.approx([-1.5, -1.0, -0.5, 0.0, 0.5, 1.0, 1.5, 2.0])
  assert [r for _, r in d.rates] == [0.0, 0.0, 0.0, 0.2, 0.2, 0.2, 0.2, 0.2]   # 23:00:00 itself is night
  d2 = Drive(kst(2026, 10, 24, 4, 0, 0) - 1.0)
  d2.start()
  d2.run(2.0)
  assert [r for _, r in d2.rates] == [0.2, 0.0, 0.0, 0.0]                      # 04:00:00 itself is day


@pytest.mark.parametrize("wall", [kst(2000, 1, 1, 23, 30), kst(2023, 6, 1, 1, 0), 0.0])
def test_unsynced_clock_never_surcharges(wall):
  d = Drive(wall)
  d.start()
  assert d.panel.meter.fare_won == 7000 and d.panel.night_rate == 0.0 and d.panel.step_won == 200
  d.run(600.0)
  assert d.ticks and all(won == pytest.approx(200.0) for _, won in d.ticks)


def test_footer_and_constant_comment_keep_the_source_note():
  src = (CLUSTER_DIR / "cluster_taxi_meter.py").read_text(encoding="utf-8")
  assert "출처 블로그, 확인 필요" in src
  assert DELUXE.night_surcharges == ((23, 4, 0.20),)
