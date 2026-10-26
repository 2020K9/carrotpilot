"""K9 hard-brake ease-off (opendbc/car/hyundai/k9_hard_brake.py, 2026-10-24 evening): the module alone and through the real
CarController against the frozen badf1ea7 controller (every K9StopMode value). Model-only harness with synthetic inputs: it
checks the source logic and the SCC12/SCC14 encoding, not K9 vehicle behaviour (실차 미검증)."""
import random

import pytest

from opendbc.can import CANParser
from opendbc.car import DT_CTRL, structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai import k9_hard_brake as hb
from opendbc.car.hyundai.k9_hard_brake import K9HardBrake
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.selfdrive.car.tests import k9_rig
from openpilot.selfdrive.car.tests.k9_rig import LCS, Rig, edge_sequence, gentle_sequence, load_frozen_cc, mixed_sequence
from openpilot.selfdrive.car.tests.test_k9_creep_mode import bl_stop_sequence

BASE = "badf1ea7bf1e72ccb5d512bfc2aa7f0ae311e7ed"
MENUS = (None, 0, 1, 2, 3, 4)
# 2026-10-25 menu 0 base / 1 v12 original / 2 v12k9 (3 acts as 0): None / 0 / 3 against badf1ea7 menu 0 (unchanged base
# braking); 1 and 2 are new stop methods, so against this tree with the hard-brake cap switched off (same inputs, same
# checks: the ease-off changes JerkUpperLimit only and nothing outside it).
# 2026-10-26: 3 = v13 original, a new stop method like 1 / 2 (this tree, cap off); 4 now stands for "other values act as 0".
BASE_MENU = {None: None, 0: 0, 4: 0}


def base_and_new(monkeypatch, tmp_path, menu, tag):
  kw = {} if menu is None else {"K9StopMode": menu}
  if menu in BASE_MENU:
    bm = BASE_MENU[menu]
    r0 = Rig(monkeypatch, module=load_frozen_cc(BASE, tmp_path, tag), **({} if bm is None else {"K9StopMode": bm}))
  else:
    r0 = Rig(monkeypatch, **kw)
    r0.cc.k9_hard_brake.jerk_u_cap = lambda: None  # the same controller without the ease-off
  return r0, Rig(monkeypatch, **kw)
CLOSING = (20.0, -2.0)


def upd(h, accel, jerk=0.5, v=15.0, pid=True, lead=CLOSING, visible=True, **kw):
  d = dict(accel=accel, jerk=jerk, pid=pid, v_ego=v, long_active=True, override=False, gas=False, brake=False, can_valid=True,
           lead_visible=visible and lead is not None, lead_distance=lead[0] if lead else 0.0,
           lead_rel_speed=lead[1] if lead else 0.0)
  d.update(kw)
  h.update(**d)
  return h.jerk_u_cap()


def test_constants():
  assert (hb.HB_ENTER, hb.HB_EXIT, hb.HB_MIN_SPEED, hb.HB_EASE_JERK, hb.HB_JERK_U) == (-1.0, -0.5, 3.0, 1.0, 0.5)
  assert carcontroller.HyundaiJerk().jerk_u_min == hb.HB_JERK_U


def test_starts_at_enter_holds_through_ease_off_and_ends_above_exit():
  h = K9HardBrake()
  assert upd(h, -0.99) is None and not h.active  # not hard yet
  assert upd(h, -1.0) == 0.5 and h.active
  assert [upd(h, a) for a in (-0.9, -0.7, -0.51)] == [0.5] * 3  # eased off above -1.0: still on until -0.5
  assert upd(h, -0.49) is None and not h.active
  assert upd(h, -0.9) is None  # needs -1.0 again to start


@pytest.mark.parametrize("jerk,cap", [(0.01, 0.5), (0.5, 0.5), (0.99, 0.5), (1.0, None), (2.5, None), (0.0, None), (-1.0, None)])
def test_only_slow_partial_ease_off_is_capped(jerk, cap):
  h = K9HardBrake()
  upd(h, -2.0)
  assert upd(h, -1.8, jerk=jerk) == cap
  assert h.active  # a fast release or more braking keeps it active, uncapped
  assert upd(h, -1.7, jerk=0.4) == 0.5


@pytest.mark.parametrize("case", ["slow", "no_lead", "lead_steady", "lead_opening", "not_pid", "not_long_active", "override",
                                  "gas", "brake", "can_invalid", "zero_distance", "nan"])
def test_every_condition_starts_nothing_and_ends_it(case):
  kw = {"slow": dict(v=hb.HB_MIN_SPEED), "no_lead": dict(lead=None), "lead_steady": dict(lead=(20.0, 0.0)),
        "lead_opening": dict(lead=(20.0, 1.0)), "not_pid": dict(pid=False), "not_long_active": dict(long_active=False),
        "override": dict(override=True), "gas": dict(gas=True), "brake": dict(brake=True), "can_invalid": dict(can_valid=False),
        "zero_distance": dict(lead=(0.0, -2.0)), "nan": dict(jerk=float("nan"))}[case]
  h = K9HardBrake()
  assert upd(h, -2.0, **kw) is None and not h.active
  h = K9HardBrake()
  assert upd(h, -2.0) == 0.5
  assert upd(h, -2.0, **kw) is None and not h.active
  if case not in ("nan",):
    assert upd(h, -0.9) is None  # does not resume above -1.0


@pytest.mark.parametrize("seed", range(4))
def test_fuzz_cap_only_in_its_window(seed):
  rng = random.Random(seed)
  h = K9HardBrake()
  for _ in range(30000):
    a = rng.uniform(-4.0, 1.0)
    jerk = rng.uniform(-3.0, 3.0)
    lead = rng.choice([None, (rng.uniform(0.0, 80.0), rng.uniform(-6.0, 3.0))])
    v = rng.uniform(0.0, 30.0)
    cap = upd(h, a, jerk=jerk, v=v, lead=lead, pid=rng.random() > 0.01, gas=rng.random() < 0.002)
    if cap is not None:
      assert cap == 0.5 and 0.0 < jerk < 1.0 and a <= -0.5 and v > 3.0 and lead is not None and lead[0] > 0 and lead[1] < 0


def step_j(rig, accel, v, jerk=0.0, lead=CLOSING, state=LCS.pid, a_target=None):
  """k9_rig.Rig.step with actuators.jerk (the planner jerk that sets JerkUpper/LowerLimit)."""
  cc = structs.CarControl(enabled=True, longActive=True)
  cc.actuators.accel, cc.actuators.aTarget, cc.actuators.jerk = accel, accel if a_target is None else a_target, jerk
  cc.actuators.longControlState = state
  if lead is not None:
    cc.hudControl.leadVisible, cc.hudControl.leadDistance, cc.hudControl.leadRelSpeed = True, lead[0], lead[1]
  cs = k9_rig.make_ci_state()
  cs.out = structs.CarState(vEgo=v, vEgoRaw=v, aEgo=accel, canValid=True)
  out, sends = rig.cc.update(cc.as_reader(), cs, rig.frame * 10_000_000)
  rig.can.append(sorted((addr, bytes(dat), bus) for addr, dat, bus in sends))
  rig.frame += 1
  return out.accel, sends


PROFILE = ([-0.3 - 0.022 * i for i in range(100)] + [-2.5] * 50 + [-2.5 + 0.005 * i for i in range(100)] +  # ease-off 0.5 m/s^3
           [-2.0 - 0.008 * i for i in range(60)] + [-2.48 + 0.025 * i for i in range(100)] + [0.02] * 30)  # regrab, fast release


def hard_sequence(rig):
  """Highway hard brake for a closing lead: build-up, slow planner ease-off, regrab, fast end release."""
  out = []
  v = 20.0
  for i, a in enumerate(PROFILE):
    jerk = 0.0 if i == 0 else (a - PROFILE[i - 1]) / DT_CTRL
    v = max(v + min(a, 0.0) * DT_CTRL, 4.0)
    out.append((a, jerk) + step_j(rig, a, v, jerk=jerk, lead=(max(30.0 - 0.1 * i, 6.0), -2.0)))
  return out


def decode(rig_can, addr, names):
  p = CANParser("hyundai_kia_generic", [("SCC12", 0), ("SCC14", 0)], 0)
  rows = []
  for f, frame in enumerate(rig_can):
    msgs = [(a, d, b) for a, d, b in frame if a == addr]
    if msgs:
      p.update([f * 10_000_000, msgs])
      rows.append((f, {n: p.vl["SCC12" if addr == 1057 else "SCC14"][n] for n in names}))
  return rows


@pytest.mark.parametrize("menu", MENUS)
def test_through_carcontroller_only_jerk_upper_changes(monkeypatch, tmp_path, menu):
  r0, r1 = base_and_new(monkeypatch, tmp_path, menu, f"hb_{menu}")
  o0, o1 = hard_sequence(r0), hard_sequence(r1)
  assert [x[2] for x in o0] == [x[2] for x in o1]  # sent accel identical
  for f0, f1 in zip(r0.can, r1.can):
    assert [m for m in f0 if m[0] != 905] == [m for m in f1 if m[0] != 905]  # SCC12 and every other message identical
  keep14 = ["JerkLowerLimit", "ComfortBandUpper", "ComfortBandLower", "ACCMode", "ObjGap", "ObjDistStat"]
  assert [r for _, r in decode(r0.can, 905, keep14)] == [r for _, r in decode(r1.can, 905, keep14)]
  ju0 = decode(r0.can, 905, ["JerkUpperLimit"])
  ju1 = decode(r1.can, 905, ["JerkUpperLimit"])
  for (f, a), (_, b) in zip(ju0, ju1):
    jerk = o1[f][1]
    if 150 <= f < 250 and 0.0 < jerk < 1.0:  # the slow ease-off of the closing-lead hard brake
      assert a["JerkUpperLimit"] == pytest.approx(1.0) and b["JerkUpperLimit"] == pytest.approx(0.5)
    elif f >= 310:  # the fast end release (planner jerk 2.5): base limit
      assert b["JerkUpperLimit"] == a["JerkUpperLimit"]
    assert b["JerkUpperLimit"] <= a["JerkUpperLimit"] + 1e-9


def highway_no_cap(rig):
  rng = random.Random("k9-hard-brake-no-cap")
  for _ in range(150):  # no lead, hard
    step_j(rig, rng.uniform(-2.5, -0.2), 20.0, jerk=rng.uniform(-2, 2), lead=None)
  for _ in range(150):  # opening or steady lead, hard
    step_j(rig, rng.uniform(-2.5, -0.2), 20.0, jerk=rng.uniform(-2, 2), lead=(25.0, rng.uniform(0.0, 3.0)))
  for _ in range(150):  # closing lead, never at or below -1.0
    step_j(rig, rng.uniform(-0.99, 0.5), 20.0, jerk=rng.uniform(-2, 2), lead=(20.0, -2.0))
  for _ in range(150):  # closing lead, hard, at or below 3.0 m/s
    step_j(rig, rng.uniform(-2.5, -0.2), rng.uniform(0.5, 3.0), jerk=rng.uniform(-2, 2), lead=(8.0, -2.0))
  for _ in range(150):  # closing lead, hard, stopping state
    step_j(rig, rng.uniform(-2.5, -0.2), 10.0, jerk=rng.uniform(-2, 2), lead=(20.0, -2.0), state=LCS.stopping)
  for _ in range(150):  # closing lead, hard, planner jerk outside (0, 1)
    step_j(rig, rng.uniform(-2.5, -1.0), 20.0, jerk=rng.choice([rng.uniform(-3, 0), rng.uniform(1.0, 3.0)]), lead=(20.0, -2.0))
  return rig


@pytest.mark.parametrize("seq", [mixed_sequence, edge_sequence, gentle_sequence, bl_stop_sequence, highway_no_cap])
@pytest.mark.parametrize("menu", MENUS)
def test_identical_to_badf1ea7_outside_the_ease_off(monkeypatch, tmp_path, seq, menu):
  # 2026-10-25: the old menu 2 stop floor (LIGHT_MIN_DECEL) was deleted with the old menu
  r0, r1 = base_and_new(monkeypatch, tmp_path, menu, f"nh_{menu}_{seq.__name__}")
  seq(r0)
  seq(r1)
  assert r0.can == r1.can


def test_only_the_k9_camera_scc_can_long_car_gets_it(monkeypatch):
  assert Rig(monkeypatch).cc.k9_hard_brake is not None
  assert Rig(monkeypatch, cp=k9_rig.make_cp(flags=int(HyundaiFlags.LEGACY))).cc.k9_hard_brake is None
  assert Rig(monkeypatch, cp=k9_rig.make_cp(fingerprint=CAR.KIA_K7)).cc.k9_hard_brake is None
