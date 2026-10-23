"""K9ExtHud=1 C1 meter, r2 (13:20 지시 1): CAN 0x500 signal loss.

The reader keeps its last value; only NEW 0x500 frames are handed to the panel, so 2 s without a new frame
= 모름 (no signal). A 5 s hold (0->1 or 1->0) in progress when the signal is lost is dropped, never completed;
it restarts from 0 when frames come back. Short gaps (< 2 s) do not drop a hold.
"""
import sys
from pathlib import Path

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

from cluster_taxi_c1 import OCC_STALE_S, C1Fsm, C1MeterPanel  # noqa: E402


class _Can:
  def __init__(self, address, src, dat):
    self.address, self.src, self.dat = address, src, dat


class _Msg:
  def __init__(self, t_ns, frames):
    self.logMonoTime, self.can = t_ns, frames


class _Sock:
  def __init__(self, q):
    self.q = q

  def receive(self, non_blocking=True):
    return self.q.pop(0) if self.q else None


class _Messaging:
  def __init__(self, q):
    self.q = q

  def sub_sock(self, name, conflate=False):
    return _Sock(self.q)

  def log_from_bytes(self, dat):
    return dat


def S(v=0.0, occ=None, fare=None, tap=False):
  return {"v_kph": v, "occ": occ, "fare": fare, "tap": tap}


def run(F, seq):
  for dur, sig in seq:
    for _ in range(round(dur / 0.05)):
      F.step(sig, 0.05)
  return F.state


class _State:
  def __init__(self, v):
    self.speed_kph, self.steering_angle_deg, self.onroad = v, 0.0, True


def _panel():
  clock = [100.0]
  return C1MeterPanel(clock=lambda: clock[0], local_hour=lambda: 12), clock


def _frames(p, clock, secs, v, occ, every=0.1):
  """HUD at 20 fps; a 0x500 frame every `every` s (occ None = no frame at all)."""
  n_every = max(1, round(every / 0.05))
  for i in range(round(secs / 0.05)):
    clock[0] += 0.05
    if occ is not None and i % n_every == 0:
      p.feed_occupant(occ, clock[0])
    p.update(_State(v))


# ---- reader / source: only new frames ------------------------------------------------------------------
def test_reader_poll_new_only_returns_new_frames():
  from cluster_live import TaxiOccupantReader
  q = []
  r = TaxiOccupantReader(_Messaging(q))
  assert r.poll_new(0.0) is None                                   # opens the socket
  q.append(_Msg(1_000_000_000, [_Can(0x500, 0, b"\x05")]))
  assert r.poll_new(1.0) == (1.0, True)
  q.append(_Msg(1_100_000_000, [_Can(0x123, 0, b"\x05")]))         # other CAN traffic only
  assert r.poll_new(1.3) is None and r.poll(1.6) == (1.0, True)     # poll() still keeps the last value
  assert r.poll_new(2.0) is None
  q.append(_Msg(2_500_000_000, [_Can(0x500, 0, b"\x01")]))
  assert r.poll_new(2.6) == (2.5, False)
  r.stop()
  assert r._new_t is None


def test_live_source_hands_on_only_new_frames():
  from cluster_live import OpenpilotLiveSource
  q = []
  src = OpenpilotLiveSource.__new__(OpenpilotLiveSource)
  src.messaging = _Messaging(q)
  assert src.poll_taxi_occupant() is None
  q.append(_Msg(5_000_000_000, [_Can(0x500, 0, b"\x05")]))
  assert src.poll_taxi_occupant() == (5.0, True)
  for _ in range(20):
    assert src.poll_taxi_occupant() is None


def test_main_feeds_the_panel_only_on_new_frames():
  import main as cluster_main

  class R:
    taxi_meter_enabled = True

    def __init__(self):
      self.fed = []

    def feed_taxi_meter_speed_samples(self, s):
      pass

    def feed_taxi_meter_occupant(self, present, t_s=None):
      self.fed.append(present)

  class Src:
    def __init__(self, seq):
      self.seq = seq

    def drain_taxi_speed_samples(self):
      return []

    def poll_taxi_occupant(self):
      return self.seq.pop(0) if self.seq else None

  r = R()
  src = Src([(1.0, True), None, None, (1.2, False), None])
  for _ in range(6):
    cluster_main.feed_taxi_meter_speed(r, src)
  assert r.fed == [True, False]


# ---- panel: 2 s without a new frame = 모름 ---------------------------------------------------------------
def test_panel_signal_goes_unknown_after_2s_without_new_frames():
  p, clock = _panel()
  _frames(p, clock, 1.0, 0.0, False)
  assert p._occ_signal(clock[0]) is False
  _frames(p, clock, OCC_STALE_S - 0.2, 0.0, None)
  assert p._occ_signal(clock[0]) is False
  _frames(p, clock, 0.4, 0.0, None)
  assert p._occ_signal(clock[0]) is None


def test_board_hold_interrupted_by_signal_loss_is_not_completed():
  p, clock = _panel()
  _frames(p, clock, 2.0, 0.0, False)
  _frames(p, clock, 3.0, 0.0, True)               # 0->1 held 3 s
  _frames(p, clock, 10.0, 0.0, None)              # 0x500 lost
  assert p.fsm.state == "EMPTY"
  _frames(p, clock, 4.0, 0.0, True)               # back: the hold restarts from 0 (3 + 4 s must not count)
  assert p.fsm.state == "EMPTY"
  _frames(p, clock, 1.3, 0.0, True)
  assert p.fsm.state == "BOARD"


def test_pay_hold_interrupted_by_signal_loss_is_not_completed():
  p, clock = _panel()
  _frames(p, clock, 5.3, 0.0, True)
  _frames(p, clock, 60.0, 40.0, True)
  assert p.fsm.state == "DRIVE"
  _frames(p, clock, 3.0, 40.0, False)             # 1->0 held 3 s
  _frames(p, clock, 10.0, 40.0, None)             # lost
  assert p.fsm.state == "DRIVE"
  _frames(p, clock, 4.0, 40.0, False)
  assert p.fsm.state == "DRIVE"
  _frames(p, clock, 1.3, 40.0, False)
  assert p.fsm.state == "PAY"


def test_short_gap_below_2s_keeps_the_hold():
  p, clock = _panel()
  _frames(p, clock, 2.0, 0.0, False)
  _frames(p, clock, 2.0, 0.0, True)
  _frames(p, clock, 1.5, 0.0, None)               # gap 1.5 s < 2 s: still the last value
  _frames(p, clock, 1.8, 0.0, True)
  assert p.fsm.state == "BOARD"


def test_last_value_repeated_every_frame_would_never_go_stale():
  # the a34c6106 bug: feeding the reader's last value every frame refreshes the stale clock
  p, clock = _panel()
  _frames(p, clock, 3.0, 0.0, True, every=0.05)
  assert p._occ_signal(clock[0]) is True
  _frames(p, clock, 3.0, 0.0, None)
  assert p._occ_signal(clock[0]) is None


def test_fsm_unknown_drops_hold_keeps_confirmed_value():
  F = C1Fsm()
  run(F, [[5.2, S(0, True)], [5, S(30, True)]])
  assert F.state == "DRIVE" and F.occ is True
  run(F, [[3, S(30, False)], [0.05, S(30, None)]])
  assert F.occ_raw is True and F.occ is True and F.state == "DRIVE"
  assert run(F, [[30, S(30, None)]]) == "DRIVE"
  assert run(F, [[4.9, S(30, False)]]) == "DRIVE"
  assert run(F, [[0.3, S(30, False)]]) == "PAY"
