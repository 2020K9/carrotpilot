"""K9ExtHud=1 C1 meter (11:50 지시): live logic, no GL.

Ports every check of out/taximeter/variants/c1test.js (44, same meaning; the JS steering filter check is
replaced by the new carrot-wheel rule: clam = -steeringAngleDeg 1:1, no filter/dead band/clamp), the
occupant rules a-e (10:15/10:35), CAN 0x500 decoding / reader, K9MeterSeason (0..4, read only).
"""
import math
import sys
from pathlib import Path

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

import cluster_taxi_c1 as c1  # noqa: E402
from cluster_taxi_c1 import (  # noqa: E402
  C1_LOGIC, C1Fsm, C1MeterPanel, OCC_SIGNAL, OCC_STABLE_S, OCCUPANT_ENABLED, c1_bg_weights, c1_season,
  clam_geometry, clam_rotation_deg, normalize_meter_season, occupant_from_acu14,
)


def W(h, mi, s):
  return c1_bg_weights(h * 3600 + mi * 60 + s)


def run(F, seq):
  for dur, sig in seq:
    for _ in range(round(dur / 0.05)):
      F.step(sig, 0.05)
  return F.state


def S(v=0.0, occ=None, fare=None, tap=False):
  return {"v_kph": v, "occ": occ, "fare": fare, "tap": tap}


# ---- c1test.js: background weights / season / constants -----------------------------------------------
def test_bg_weights_points():
  assert W(12, 0, 0)["day"] == 1 and W(17, 0, 0)["dusk"] == 1 and W(21, 0, 0)["night"] == 1
  assert W(3, 0, 0)["night"] == 1 and W(4, 59, 59)["night"] == 1 and W(5, 0, 3)["day"] == 1


def test_bg_weights_fade_midpoints():
  assert W(5, 0, 1.5)["day"] == pytest.approx(0.5) and W(16, 0, 1.5)["dusk"] == pytest.approx(0.5)
  assert W(19, 0, 1.5)["night"] == pytest.approx(0.5)


def test_bg_fade_monotonic():
  prev = 0.0
  for i in range(61):
    w = W(16, 0, i * 0.05)["dusk"]
    assert w >= prev - 1e-12
    prev = w


@pytest.mark.parametrize("sec", [0, 3600, 40000, 86399])
def test_bg_weights_sum_1(sec):
  w = c1_bg_weights(sec)
  assert w["day"] + w["dusk"] + w["night"] == pytest.approx(1.0)


def test_season_by_month():
  assert c1_season(10) == "autumn" and c1_season(1) == "winter" and c1_season(4) == "spring" and c1_season(7) == "summer"


def test_occupant_constants():
  # meter_empty2 (보노보노 21:59 "손님탐도없애"): the passenger sensor is off - it changes no meter state at all
  assert OCCUPANT_ENABLED is False and OCC_STABLE_S == 5.0
  assert OCC_SIGNAL == {"addr": 0x500, "bus": 0, "start_bit": 2, "len": 2, "present_value": 1}


# ---- c1test.js: state machine -------------------------------------------------------------------------
def test_occupant_ignored_when_disabled_toggle_works():
  F = C1Fsm(C1_LOGIC, False)
  assert run(F, [[10, S(0, True)]]) == "EMPTY"
  F.step(S(0, tap=True), 0.05)
  assert F.state == "DRIVE"                        # meter_empty2: no 손님탐 with the sensor off, 빈차 -> 주행 at once
  assert run(F, [[1, S(20)]]) == "DRIVE"
  F.step(S(0, tap=True), 0.05)
  assert F.state == "PAY"
  assert run(F, [[10.1, S(0)]]) == "EMPTY"


def test_tap_cancel():
  F = C1Fsm(C1_LOGIC, True)                        # sensor on (kept logic): 빈차 -> 손님탐 -> 빈차
  F.step(S(0, tap=True), 0.05)
  F.step(S(0, tap=True), 0.05)
  assert F.state == "EMPTY"
  F = C1Fsm()                                      # meter_empty2 default (sensor off): 빈차 -> 주행 -> 지불
  F.step(S(0, tap=True), 0.05)
  assert F.state == "DRIVE"
  F.step(S(0, tap=True), 0.05)
  assert F.state == "PAY"


def test_occupant_board_drive_pay_empty():
  F = C1Fsm(C1_LOGIC, True)
  assert run(F, [[4.9, S(0, True)]]) == "EMPTY"
  assert run(F, [[0.3, S(0, True)]]) == "BOARD"
  assert run(F, [[2, S(30, True)]]) == "DRIVE"
  a1 = run(F, [[5.2, S(30, False)]])
  a2 = run(F, [[14.8, S(30, False)]])
  # meter_improve 20:43 (보노보노 '운행끝나고 승객연동되잇는거 지워'): the passenger no longer ends a drive
  assert a1 == "DRIVE" and a2 == "DRIVE"


def test_flicker_while_driving_stays_drive_then_stop_pay_move_empty():
  F = C1Fsm(C1_LOGIC, True)
  run(F, [[5.2, S(0, True)], [2, S(30, True)]])
  seq = [[0.15, S(40, i % 2 == 1)] for i in range(60)] + [[1, S(40, True)]]
  assert run(F, seq) == "DRIVE"
  assert run(F, [[4.9, S(0, False)]]) == "DRIVE"
  # meter_improve 20:43 (보노보노 '운행끝나고 승객연동되잇는거 지워'): the passenger no longer ends a drive
  assert run(F, [[0.3, S(0, False)]]) == "DRIVE"
  assert run(F, [[1, S(10, False)]]) == "DRIVE"


def test_unstable_occupant_while_stopped_no_board():
  F = C1Fsm(C1_LOGIC, True)
  assert run(F, [[1, S(0, i % 2 == 0)] for i in range(40)]) == "EMPTY"


def test_steering_rule_is_the_carrot_wheel_rule():
  # replaces the JS F.steer filter check (x0.5, +-90, tau): 11:50 rule = -steeringAngleDeg 1:1
  for deg in (100.0, -100.0, 1.0, 2.9, -0.4, 216.0, -540.0, 0.0):
    assert clam_rotation_deg(deg) == -deg
  assert clam_rotation_deg(float("nan")) == 0.0 and clam_rotation_deg(None) == 0.0


def test_no_door_or_seatbelt_code():
  import re
  src = Path(c1.__file__).read_text()
  code = "\n".join(line.split("#")[0] for line in src.splitlines())
  assert not re.search(r"DrSw|doors|seatbelt|CF_SBR|ODS11|0x5FA", code)


# ---- c1test.js (a)-(e): 10:35 occupant rules ----------------------------------------------------------
def test_a_154_replay_pay_at_stop_start_fare():
  F = C1Fsm(C1_LOGIC, True)   # meter_empty2: the sensor logic is kept (OCCUPANT_ENABLED False on the car)
  run(F, [[5.2, S(0, True, 7000)], [3, S(30, True, 7000)], [3, S(30, True, 7400)]])
  run(F, [[2, S(0, True, 7600)]])
  run(F, [[0.5, S(2, True, 7600)], [2.8, S(15, True, 7600)]])
  s154 = run(F, [[2, S(15, False, 7800)], [3.1, S(15, False, 7800)]])
  # meter_improve 20:43 (보노보노 '운행끝나고 승객연동되잇는거 지워'): the passenger no longer ends a drive
  assert s154 == "DRIVE" and F.pay_fare is None
  assert run(F, [[2, S(15, False, 8000)]]) == "DRIVE" and F.pay_fare is None


def test_a3_no_stop_fare_at_count_moment():
  F = C1Fsm(C1_LOGIC, True)
  run(F, [[5.2, S(0, True, 7000)], [3, S(30, True, 7200)]])
  run(F, [[5.2, S(30, False, 7400)]])
  # meter_improve 20:43 (보노보노 '운행끝나고 승객연동되잇는거 지워'): the passenger no longer ends a drive
  assert F.state == "DRIVE" and F.pay_fare is None


def test_b_rise_while_driving_ignored():
  F = C1Fsm(C1_LOGIC, True)
  run(F, [[2, S(0, False)], [1, S(30, False)]])
  assert run(F, [[6, S(30, True)]]) == "EMPTY"
  assert run(F, [[1, S(0, True)]]) == "EMPTY"


def test_c_fall_shorter_than_5s_ignored():
  F = C1Fsm(C1_LOGIC, True)
  run(F, [[5.2, S(0, True)], [2, S(30, True)]])
  assert run(F, [[4.9, S(30, False)], [1, S(30, True)]]) == "DRIVE"


def test_d_toggle_wins():
  F = C1Fsm(C1_LOGIC, True)
  run(F, [[4.95, S(0, True)]])
  F.step(S(0, True, tap=True), 0.1)
  assert F.state == "BOARD" and F.ev == "tap"
  F.step(S(0, True, tap=True), 0.05)
  assert F.state == "EMPTY" and F.ev == "tap"
  F = C1Fsm(C1_LOGIC, True)
  run(F, [[5.2, S(0, True, 7000)], [2, S(30, True, 7200)]])
  F.step(S(30, True, 7200, tap=True), 0.05)
  assert F.state == "PAY" and F.pay_fare == 7200


def test_e_enabled_by_default():
  # meter_empty2 (보노보노 21:59 "손님탐도없애"): off by default now - a passenger held 5 s changes nothing
  F = C1Fsm()
  assert OCCUPANT_ENABLED is False and run(F, [[5.2, S(0, True)], [60, S(30, True)], [10, S(0, False)]]) == "EMPTY"
  assert run(C1Fsm(C1_LOGIC, True), [[5.2, S(0, True)]]) == "BOARD"   # the kept logic, sensor on


def test_no_signal_keeps_state():
  F = C1Fsm(C1_LOGIC, True)
  run(F, [[5.2, S(0, True)], [2, S(30, True)]])
  assert run(F, [[30, S(30, None)]]) == "DRIVE"     # 0x500 missing != passenger got out


# ---- CAN 0x500 --------------------------------------------------------------------------------------------
def test_acu14_decode():
  assert occupant_from_acu14(b"\x05") is True and occupant_from_acu14(b"\x01") is False
  assert occupant_from_acu14(b"\x0d") is False and occupant_from_acu14(b"\x04") is True
  assert occupant_from_acu14(b"") is None and occupant_from_acu14(None) is None


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
    self.q, self.opened = q, []

  def sub_sock(self, name, conflate=False):
    self.opened.append((name, conflate))
    return _Sock(self.q)

  def log_from_bytes(self, dat):
    return dat


def test_occupant_reader_bus0_only_newest_and_read_only():
  from cluster_live import TaxiOccupantReader
  q = []
  m = _Messaging(q)
  r = TaxiOccupantReader(m)
  assert r.poll(0.0) is None and m.opened == [("can", False)]
  q.extend([_Msg(1_000_000_000, [_Can(0x500, 0, b"\x01")]),
            _Msg(2_000_000_000, [_Can(0x500, 1, b"\x05"), _Can(0x123, 0, b"\x05")])])  # bus1 / other ids ignored
  assert r.poll(1.0) == (1.0, False)
  q.append(_Msg(3_000_000_000, [_Can(0x500, 0, b"\x05")]))
  assert r.poll(1.1) == (1.0, False)               # parse at most every 0.25 s
  assert r.poll(1.3) == (3.0, True)
  assert not hasattr(m, "send") and all(name == "can" for name, _ in m.opened)


# ---- panel logic (meter only in 주행, frozen pay amount) ------------------------------------------------
class _State:
  def __init__(self, v, steer=0.0, onroad=True):
    self.speed_kph, self.steering_angle_deg, self.onroad = v, steer, onroad


def _panel():
  clock = [100.0]
  p = C1MeterPanel(clock=lambda: clock[0], local_hour=lambda: 12)
  return p, clock


def _drive(p, clock, secs, v, occ, steer=0.0):
  for _ in range(round(secs / 0.05)):
    clock[0] += 0.05
    p.feed_occupant(occ, clock[0])
    p.update(_State(v, steer))


def test_panel_empty_board_drive_pay_with_meter():
  p, clock = _panel()
  _drive(p, clock, 3, 0.0, False)
  # res 10/24 17:52: C1 meter = 경기 일반(중형) 4,800원/1.6km, 131m당 100원 (was 모범 7,000/3km, 144m당 200)
  assert p.fsm.state == "EMPTY" and p.shown_fare() == 4800 and p.meter.distance_m == 0.0
  _drive(p, clock, 5.3, 0.0, True)
  # meter_empty2 (보노보노 21:59 "손님탐도없애"): a passenger no longer makes 손님탐, nor does driving off make 주행
  assert p.fsm.state == "EMPTY" and p.shown_fare() == 4800
  _drive(p, clock, 30, 60.0, True)
  assert p.fsm.state == "EMPTY" and p.meter.distance_m == 0.0 and p.shown_fare() == 4800
  p.command("start")                                # the app's 시작: 주행 at once, from 0
  _drive(p, clock, 300, 60.0, True)                 # 5 km at 60 km/h
  assert p.fsm.state == "DRIVE" and p.meter.distance_m == pytest.approx(5000, rel=0.01)
  assert p.shown_fare() == 4800 + 100 * int((p.meter.metered_m - 1600) // 131)
  _drive(p, clock, 3, 0.0, True)                    # stop starts at this fare
  stop_fare = p.fsm.last_stop_fare
  _drive(p, clock, 2, 20.0, True)
  _drive(p, clock, 5.3, 20.0, False)
  # meter_improve 20:43 (보노보노 '운행끝나고 승객연동되잇는거 지워'): the passenger no longer ends a drive
  assert p.fsm.state == "DRIVE" and p.shown_fare() == p.meter.fare_won
  _drive(p, clock, 2, 20.0, False)
  assert p.fsm.state == "DRIVE" and p.shown_fare() == p.meter.fare_won


def test_panel_occupant_stale_is_no_signal():
  p, clock = _panel()
  p.feed_occupant(True, clock[0])
  for _ in range(200):
    clock[0] += 0.05
    p.update(_State(0.0))
  assert p.fsm.state == "EMPTY"                      # 1 seen once, then no frames for 10 s: not counted


def test_panel_restart_resets_state():
  p, clock = _panel()
  p.toggle()
  _drive(p, clock, 1, 0.0, None)
  assert p.fsm.state == "DRIVE"                     # meter_empty2: the toggle = 시작 (no 손님탐)
  p.restart()
  assert p.fsm.state == "EMPTY" and p.meter.distance_m == 0.0


def test_clam_geometry_rotation_exact_and_hands_bounded():
  for deg in (-540.0, -216.0, -45.0, 0.0, 3.0, 89.0, 160.0, 720.0):
    g = clam_geometry(deg, bob=3.0, brot=-2.0, tap=0.4)
    assert g["rotation"] == -deg
    assert abs(g["grip"]) <= 90.0 and abs(g["lean"]) <= 5.4 + 1e-9
    for (hx, hy) in g["hands"]:
      cx, cy = g["center"]
      assert math.hypot(hx - cx, hy - cy) == pytest.approx(math.hypot(19.5, 1.0) * 1.32 * 0.82, rel=1e-6)


def test_panel_follows_steering_every_update():
  p, clock = _panel()
  for deg in (0.0, 12.5, -300.0, 0.1):
    clock[0] += 0.05
    p.update(_State(30.0, deg))
    assert p.steering_angle_deg == deg            # no filter: the very value of this frame


# ---- K9MeterSeason ----------------------------------------------------------------------------------------
@pytest.mark.parametrize("value,expect", [(0, None), (1, "spring"), (2, "summer"), (3, "autumn"), (4, "winter"),
                                          (5, None), (-1, None), (99, None), (None, None), ("x", None)])
def test_meter_season_values(value, expect):
  for month, auto in ((4, "spring"), (7, "summer"), (10, "autumn"), (1, "winter")):
    assert c1_season(month, value) == (expect or auto)
  assert normalize_meter_season(value) == ({None: 0}.get(expect, 0) if expect is None else value)


def test_panel_season_setting_picks_background_files():
  p, clock = _panel()
  for setting, season in ((1, "spring"), (2, "summer"), (3, "autumn"), (4, "winter"), (7, None)):
    p.set_season_setting(setting)
    clock[0] += 0.05
    p.update(_State(0.0))
    assert p.season == (season or c1_season(p._clock_from_hour()[1]))
    for tod in ("day", "dusk", "night"):
      for kind in ("back", "over"):
        assert (c1.C1_ASSET_DIR / f"{kind}_{p.season}_{tod}.png").exists()


def test_meter_season_param_is_only_read():
  sys.path.insert(0, str(CLUSTER))
  import main as cluster_main

  class P:
    def __init__(self, v):
      self.v, self.calls = v, []

    def get_int(self, k):
      self.calls.append(("get_int", k))
      return self.v

    def __getattr__(self, name):
      raise AssertionError(f"unexpected Params.{name}")

  for v, expect in ((0, 0), (3, 3), (4, 4), (5, 0), (-2, 0)):
    reader = cluster_main.ClusterHudModeParamReader.__new__(cluster_main.ClusterHudModeParamReader)
    reader._params = P(v)
    assert reader.read_meter_season() == expect
    assert reader._params.calls == [("get_int", "K9MeterSeason")]


def test_meter_season_registered():
  import json
  root = Path(__file__).resolve().parents[3]
  d = json.loads((root / "selfdrive" / "carrot_settings.json").read_text())
  p = [x for x in d["params"] if x["name"] == "K9MeterSeason"][0]
  assert (p["min"], p["max"], p["default"], p["control"]) == (0, 4, 0, "select")
  assert all(len(p["options"][k]) == 5 for k in ("ko", "en", "zh"))
  assert '{"K9MeterSeason", {PERSISTENT, INT, "0"}}' in (root / "common" / "params_keys.h").read_text()


def test_no_occupant_signal_ever_keeps_the_bd09c076_meter():
  # meter_empty2: with the sensor off (default) the meter is the buttons' alone - onroad / driving start nothing
  p, clock = _panel()
  for _ in range(600):
    clock[0] += 0.05
    p.update(_State(60.0))
  assert p.fsm_mode and p.display_state == "EMPTY" and p.meter.distance_m == 0.0 and p.shown_fare() == 4800
  p.feed_occupant(True, clock[0])
  for _ in range(200):
    clock[0] += 0.05
    p.feed_occupant(True, clock[0])
    p.update(_State(0.0))
  assert p.display_state == "EMPTY"
  # sensor on (kept code): no 0x500 frame (other car / CAN not up yet) counts from the onroad rising edge as before
  clock = [100.0]
  p = C1MeterPanel(clock=lambda: clock[0], local_hour=lambda: 12, occupant=True)
  for _ in range(600):
    clock[0] += 0.05
    p.update(_State(60.0))
  assert not p.fsm_mode and p.display_state == "DRIVE" and p.meter.distance_m == pytest.approx(60 / 3.6 * 0.05 * 599, rel=1e-6)
  assert p.shown_fare() == p.meter.fare_won
  p.feed_occupant(False, clock[0])                 # first frame -> C1 state machine (seat empty = 빈차)
  clock[0] += 0.05
  p.update(_State(60.0))
  assert p.fsm_mode and p.display_state == "EMPTY" and p.shown_fare() == 4800   # res: 일반 기본요금


def test_bubble_never_overlaps_digits_stats_or_bonobono():
  # 겹침 없음: 포로리 at every x it can take, the widest bubble text (~17 px font, <= 8 Korean glyphs ~ 140 px)
  from cluster_taxi_c1 import FARE_BOX, PORO_MIN, X0, X1, bubble_rect
  stats = (552.0, 78.0, 752.0, 230.0)     # glass box of ui_*.png (bottom ~229)
  bono = (612.0, 242.0, 726.0, 404.0)    # bono_body.png bbox at (668, 306), bob +-4.5, sway +-3.2 deg
  def hit(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]
  for tw in (60.0, 100.0, 140.0, 180.0):
    for px in range(int(X0 + PORO_MIN), int(X1) + 1):
      bx, by, w, h, right = bubble_rect(tw, float(px))
      box = (bx - 11, by, bx + w + 11, by + h)
      assert not hit(box, FARE_BOX) and not hit(box, stats) and not hit(box, bono), (tw, px, box)
      assert box[0] >= 0 and box[2] <= 776


def test_porori_jump_stays_below_the_digits():
  # porori_body.png top row (bbox) + highest jump must stay under the fare digits (98 px from y 84 -> 182)
  from PIL import Image
  from cluster_taxi_c1 import C1_ASSET_DIR, PORO_JUMP_MAX, PORO_Y, SPRITE_ANCHOR
  top = Image.open(C1_ASSET_DIR / "porori_body.png").getbbox()[1] - SPRITE_ANCHOR
  assert PORO_Y + top - PORO_JUMP_MAX >= 182 + 8


def test_drive_counts_from_carstate_samples_even_without_the_onroad_flag():
  p, clock = _panel()
  p.toggle()
  for i in range(200):                              # 10 s at 36 km/h, every carState (100 Hz) fed
    clock[0] += 0.05
    p.feed_speed_samples([(clock[0] - 0.05 + k * 0.01, 10.0) for k in range(1, 6)])
    p.update(_State(36.0, onroad=False))
  assert p.fsm.state == "DRIVE" and p.meter.distance_m == pytest.approx(10.0 * 0.05 * 199, rel=0.02)
