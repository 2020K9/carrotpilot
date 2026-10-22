"""K9ExtHud=1 taxi meter: speed-linked animation phase (00:52 지시). New tests, pure python (no GL).

- phase rate = 0.35 stopped / 1.0 at 30 km/h / 2.2 at >= 100 km/h (cap), linear between
- speed is low-passed (0.5 s): a speed step never makes the characters / sea jump
- stopped = slow breathing + blinks only; time-fare zone (below 14.81 km/h onroad; was <= 15 before 01:06) = 보노보노 taps the 조개
- stars / blinks stay on the wall clock, the +200원 hop stays on the fare tick
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
if str(CLUSTER_DIR) not in sys.path:
  sys.path.insert(0, str(CLUSTER_DIR))

from cluster_taxi_meter import (  # noqa: E402
  ANIM_MAX_DT_S, ANIM_RATE_POINTS, ANIM_SPEED_TAU_S, PORORI_RUN_HZ, MeterAnimation, TaxiMeterPanel, anim_rate,
  motion_visibility, taxi_meter_pose)

FPS = (20.0, 10.0, 30.0)
POSE_CHANNELS = ("porori_hop", "porori_rot", "porori_tail", "porori_leg", "porori_arm",
                 "bono_bob", "bono_rot", "bono_tap", "wave_ph")


def _run(anim, speed, secs, fps=20.0, onroad=True):
  out = []
  for _ in range(int(round(secs * fps))):
    anim.step(1.0 / fps, speed, onroad)
    out.append(taxi_meter_pose(anim, 3.5, 9.0))
  return out


def _max_frame_delta(poses, channel):
  vals = [getattr(p, channel) for p in poses]
  return max(abs(b - a) for a, b in zip(vals, vals[1:]))


def _steady_max_delta(speeds, channel, fps, onroad=True):
  """Fastest per-frame change of a pose channel while driving steadily at any of these speeds."""
  out = 0.0
  for v in speeds:
    anim = MeterAnimation()
    _run(anim, v, 6.0, fps, onroad)
    out = max(out, _max_frame_delta(_run(anim, v, 6.0, fps, onroad), channel))
  return out


# amplitude of each pose channel that fades in/out with the 0.5 s run<->stand weight
FADE_AMPLITUDE = {"porori_hop": 9.0 + 1.2, "porori_rot": 5.0, "porori_tail": 8.0, "porori_leg": 4.0,
                  "porori_arm": 18.0, "bono_bob": 0.0, "bono_rot": 0.0, "bono_tap": 0.0, "wave_ph": 0.0}
MAX_RATE_SLOPE = max((r1 - r0) / (v1 - v0) for (v0, r0), (v1, r1) in zip(ANIM_RATE_POINTS, ANIM_RATE_POINTS[1:]))


@pytest.mark.parametrize("speed,rate", [(0.0, 0.35), (30.0, 1.0), (100.0, 2.2), (130.0, 2.2), (250.0, 2.2),
                                        (15.0, 0.675), (65.0, 1.6), (-30.0, 1.0)])
def test_rate_table(speed, rate):
  assert anim_rate(speed) == pytest.approx(rate, abs=1e-9)


def test_rate_bad_input_is_stopped_rate():
  for bad in (float("nan"), float("inf"), None, "x"):
    assert anim_rate(bad) == pytest.approx(0.35)


@pytest.mark.parametrize("fps", FPS)
@pytest.mark.parametrize("speed,rate", [(0.0, 0.35), (30.0, 1.0), (100.0, 2.2)])
def test_phase_grows_at_the_rate_for_a_steady_speed(speed, rate, fps):
  anim = MeterAnimation()
  _run(anim, speed, 8.0, fps)                 # filter settled (16 tau)
  p0 = anim.phase
  _run(anim, speed, 4.0, fps)
  assert (anim.phase - p0) / 4.0 == pytest.approx(rate, rel=1e-4)
  assert anim.speed_kph == pytest.approx(speed, abs=0.01)


def test_speed_low_pass_time_constant_is_half_a_second():
  anim = MeterAnimation()
  _run(anim, 0.0, 2.0)
  _run(anim, 100.0, ANIM_SPEED_TAU_S)
  assert anim.speed_kph == pytest.approx(100.0 * (1 - math.exp(-1)), abs=0.5)  # 63.2 after one tau
  anim2 = MeterAnimation()
  _run(anim2, 100.0, 0.05)
  assert anim2.speed_kph < 10.0               # one 20 fps frame after the step: < 10 % of it


@pytest.mark.parametrize("fps", FPS)
@pytest.mark.parametrize("v0,v1", [(0.0, 100.0), (100.0, 0.0), (0.0, 30.0), (30.0, 100.0), (100.0, 30.0), (10.0, 60.0)])
def test_speed_step_never_jumps(v0, v1, fps):
  """Across a speed step every pose channel moves no faster per frame than it does while
  driving steadily at some speed inside the step's range (plus the 0.5 s run<->stand fade of
  its amplitude: at most amplitude * (1-exp(-dt/0.5)) per frame), and the phase rate changes
  gradually (one frame: at most slope * |step| * (1-exp(-dt/0.5)))."""
  lo, hi = min(v0, v1), max(v0, v1)
  speeds = [lo + (hi - lo) * k / 20.0 for k in range(21)]
  anim = MeterAnimation()
  before = _run(anim, v0, 6.0, fps)
  rates = [anim.rate]
  after = []
  for _ in range(int(4.0 * fps)):
    anim.step(1.0 / fps, v1, True)
    rates.append(anim.rate)
    after.append(taxi_meter_pose(anim, 3.5, 9.0))
  poses = before[-3:] + after
  for ch in POSE_CHANNELS:
    if ch == "bono_tap":
      continue  # the waiting tap fades in/out with its own 0.5 s filter, checked below
    fade = FADE_AMPLITUDE[ch] * (1 - math.exp(-(1.0 / fps) / 0.5))
    limit = _steady_max_delta(speeds, ch, fps) * 1.05 + fade + 1e-6
    assert _max_frame_delta(poses, ch) <= limit, ch
  # rate: one frame moves at most slope * |step| * (1-exp(-dt/tau)) -- never the whole step at once
  max_rate_step = max(abs(b - a) for a, b in zip(rates, rates[1:]))
  assert max_rate_step <= MAX_RATE_SLOPE * (hi - lo) * (1 - math.exp(-(1.0 / fps) / ANIM_SPEED_TAU_S)) * 1.01 + 1e-9


@pytest.mark.parametrize("fps", FPS)
def test_wait_tap_fades_in_and_out_without_jumps(fps):
  """Entering / leaving the time-fare zone (and onroad toggling) fades the 조개 tap in and out."""
  anim = MeterAnimation()
  # fastest steady waiting tap = at the top of the time-fare zone (just below 14.81 km/h, rate ~0.67)
  limit = _steady_max_delta([0.0, 5.0, 10.0, 14.8], "bono_tap", fps) * 1.05 + 1e-6
  seq = (_run(anim, 50.0, 5.0, fps) + _run(anim, 0.0, 6.0, fps) + _run(anim, 0.0, 3.0, fps, onroad=False)
         + _run(anim, 0.0, 3.0, fps) + _run(anim, 60.0, 4.0, fps))
  assert _max_frame_delta(seq, "bono_tap") <= limit


def test_waiting_tap_only_in_the_time_fare_zone():
  # 01:06: time-fare zone = below 기준속도 144 m / 35 s = 14.81 km/h (was <= 15 km/h)
  for speed, onroad, tapping in ((0.0, True, True), (10.0, True, True), (14.5, True, True), (15.0, True, False),
                                 (20.0, True, False), (30.0, True, False), (100.0, True, False), (0.0, False, False)):
    anim = MeterAnimation()
    _run(anim, speed, 5.0, onroad=onroad)
    poses = _run(anim, speed, 6.0, onroad=onroad)
    peak = max(p.bono_wait_tap for p in poses)
    assert (peak > 0.9) is tapping, (speed, onroad, peak)
    if not tapping:
      assert peak < 0.02, (speed, onroad, peak)


def test_stopped_is_breathing_only_and_running_moves():
  anim = MeterAnimation()
  _run(anim, 0.0, 5.0)
  stop = _run(anim, 0.0, 6.0)
  for ch in ("porori_rot", "porori_tail", "porori_leg", "porori_arm"):
    assert max(abs(getattr(p, ch)) for p in stop) < 1e-6, ch
  assert 0.5 < max(p.porori_hop for p in stop) - min(p.porori_hop for p in stop) < 3.0   # slow breathing
  assert max(abs(p.bono_bob) for p in stop) > 3.0                                         # slow float
  anim = MeterAnimation()
  _run(anim, 30.0, 5.0)
  run = _run(anim, 30.0, 4.0)
  assert max(abs(p.porori_leg) for p in run) > 3.5 and max(abs(p.porori_tail) for p in run) > 7.0
  assert max(p.porori_hop for p in run) > 8.0


def test_motion_cycles_scale_with_speed():
  """Same wall time: 100 km/h runs 2.2/0.35 x the cycles of stopped, 30 km/h = 1.0/0.35 x."""
  phases = {}
  for v in (0.0, 30.0, 100.0):
    anim = MeterAnimation()
    _run(anim, v, 6.0)
    p0 = anim.phase
    _run(anim, v, 10.0)
    phases[v] = anim.phase - p0
  assert phases[30.0] / phases[0.0] == pytest.approx(1.0 / 0.35, rel=1e-3)
  assert phases[100.0] / phases[0.0] == pytest.approx(2.2 / 0.35, rel=1e-3)


@pytest.mark.parametrize("fps", (5.0, 10.0, 20.0, 30.0))
@pytest.mark.parametrize("speed", (0.0, 30.0, 60.0, 100.0, 140.0))
def test_fast_motion_is_never_drawn_above_the_frame_rate(speed, fps):
  """No aliasing: a motion whose frequency is >= fps/2 is not drawn (it would flicker or run
  backwards at the HUD's 5..30 fps); motions <= fps/4 keep full amplitude."""
  anim = MeterAnimation()
  _run(anim, speed, 8.0, fps)
  assert anim.fps == pytest.approx(fps, rel=0.05)
  stride = PORORI_RUN_HZ * anim.rate
  poses = _run(anim, speed, 6.0, fps)
  hop_amp = max(p.porori_hop for p in poses) - min(p.porori_hop for p in poses)
  leg_amp = max(abs(p.porori_leg) for p in poses)
  if speed > 0 and 2 * stride >= fps / 2:
    assert hop_amp < 1e-6
  if speed > 0 and stride >= fps / 2:
    assert leg_amp < 1e-6
  if speed >= 12.0 and 2 * stride <= fps / 4:
    assert hop_amp > 8.0
  assert motion_visibility(fps / 4, fps) == 1.0 and motion_visibility(fps / 2, fps) == 0.0


def test_frame_gap_is_clamped():
  anim = MeterAnimation()
  _run(anim, 100.0, 6.0)
  p0 = anim.phase
  anim.step(5.0, 100.0)                         # 5 s stall
  assert anim.phase - p0 == pytest.approx(ANIM_MAX_DT_S * 2.2, rel=1e-4)
  anim.step(-3.0, 100.0)                        # clock going backwards
  anim.step(float("nan"), float("nan"))
  assert anim.phase - p0 == pytest.approx(ANIM_MAX_DT_S * 2.2, rel=1e-4)


def test_update_uses_the_clock_delta():
  anim = MeterAnimation()
  anim.update(100.0, 30.0, True)
  assert anim.phase == 0.0                      # first call only starts the clock
  for k in range(1, 201):
    anim.update(100.0 + k * 0.05, 30.0, True)
  assert anim.phase == pytest.approx(sum(1 for _ in range(200)) * 0.05 * 1.0, rel=0.2)


def test_blinks_and_stars_stay_on_the_wall_clock():
  a, b = MeterAnimation(), MeterAnimation()
  _run(a, 0.0, 3.0)
  _run(b, 100.0, 9.0)
  for t in (1.35, 2.25, 6.15, 6.85, 8.45, 4.0, 0.5):
    pa, pb = taxi_meter_pose(a, t, 9.0), taxi_meter_pose(b, t, 9.0)
    assert pa.porori_blink == pb.porori_blink and pa.bono_blink == pb.bono_blink
  assert taxi_meter_pose(a, 1.35, 9.0).porori_blink and taxi_meter_pose(a, 2.25, 9.0).bono_blink
  assert not taxi_meter_pose(a, 4.0, 9.0).porori_blink and not taxi_meter_pose(a, 4.0, 9.0).bono_blink


@pytest.mark.parametrize("speed", [0.0, 30.0, 100.0])
def test_fare_tick_hop_and_tap_stay_on_the_tick(speed):
  anim = MeterAnimation()
  _run(anim, speed, 5.0)
  assert taxi_meter_pose(anim, 3.5, 0.4).porori_hop > 25.0        # +200원 hop mid-way
  assert taxi_meter_pose(anim, 3.5, 0.225).bono_tap > 0.99         # 딱!
  # the tick envelope starts and ends at the running pose (no jump into / out of the hop)
  for tk0, tk1 in ((0.0, 0.0), (0.8, 9.0)):
    p0, p1 = taxi_meter_pose(anim, 3.5, tk0), taxi_meter_pose(anim, 3.5, tk1)
    assert abs(p0.porori_hop - taxi_meter_pose(anim, 3.5, 9.0).porori_hop) < 1e-6
    assert abs(p1.porori_rot - taxi_meter_pose(anim, 3.5, 9.0).porori_rot) < 1e-6


def test_panel_update_drives_meter_and_animation_from_one_clock():
  clock = [10.0]

  class State:
    speed_kph = 100.0
    onroad = True

  panel = TaxiMeterPanel(clock=lambda: clock[0], local_hour=lambda: 12)
  for _ in range(200):
    clock[0] += 0.05
    panel.update(State)
  assert panel.anim.speed_kph == pytest.approx(100.0, abs=0.1)
  assert panel.anim.rate == pytest.approx(2.2, abs=1e-3)
  assert panel.meter.distance_m == pytest.approx(100.0 / 3.6 * 0.05 * 199, rel=1e-6)
