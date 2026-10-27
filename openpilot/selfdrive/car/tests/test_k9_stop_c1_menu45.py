"""K9StopMode 5 = C1+ (kind 10) and 4 = C1 (kind 9), 2026-10-27 (보노보노 15:53 / 16:14 "c1.c1+ 3번 4번추가해", 16:45 "그럼 4.5에넣어야지"):
menu 2 (kind 8) and menu 3 (v13 original) are 26daac1a unchanged; C1 / C1+ are separate kinds. This file = the C1+ candidate tests
of brake_0515_1027 (k9-stop-0515c, written for kind 8 there) moved to menu 5 / kind 10 and its constant names (V12K9_C1_* /
V12K9_C1P_*); the 26daac1a reference stays menu 2. Plus C1 pins (menu 4). C1+: lead input at the 05:15 dropout, end-profile hand-over, no go StopReq while rolling.
The 05:15:17 drive (rlog 000001d3--b1714d3f32--3): radarState lost a 40 m radar lead pulling away; for 2 carControl frames the HUD
said leadVisible 1 (longitudinalPlan.hasLead of the frame before) with leadDistance 0.0 (no radar lead), kind 8 took it for a lead
at 0 m and sent StopReq while rolling at 0.139 m/s (aReq -0.06 -> -0.38 -> -0.59) -> 꾹.
08:26 order: a positive distance is used as sent, at once; only leadVisible 1 + leadDistance 0.0 right after a far lead changes.
Lead cases against the frozen 26daac1a CarController: (a) a real 40 m -> 1 m cut-in (positive distance): StopReq / aReq frame by
frame the same or stronger, (b) 40 m -> 0 m for 2 frames: no StopReq while rolling, (c) a near lead (0-1.5 m) from the start:
CAN identical. Checks source logic and SCC12/SCC14 encoding, not K9 vehicle behaviour (실차 미검증)."""
import copy
import struct

import pytest

from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V9, V12K9, V12K9_C1, V12K9_C1P
from opendbc.car.hyundai.values import CAR
from openpilot.selfdrive.car.tests.k9_rig import LCS, Rig, edge_sequence, gentle_sequence, load_frozen_cc, mixed_sequence
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import make_cp

BASE = "26daac1a"


@pytest.fixture(scope="module")
def base_cc(tmp_path_factory):
  return load_frozen_cc(BASE, tmp_path_factory.mktemp("cc_26daac1a_0515"), "cc_26daac1a_0515")



# --- menu45_1027: every menu other than 4 / 5 is 26daac1a bit for bit (3 = v13 original as there; 6 acts as 0 as 4 did there)
def _kw(mode):
  return {} if mode is None else {"K9StopMode": mode}


@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("new_menu, base_menu", [(None, None), (0, 0), (1, 1), (2, 2), (3, 3), (6, 6), (6, 4), (99, 99), (-1, -1),
                                                 (float("nan"), float("nan"))])
def test_menus_identical_to_26daac1a(monkeypatch, base_cc, new_menu, base_menu, fingerprint):
  for seq in (mixed_sequence, edge_sequence, gentle_sequence):
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), module=base_cc, **_kw(base_menu)))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), **_kw(new_menu)))
    monkeypatch.undo()
    assert len(new.can) > 400 and ref.can == new.can and ref.scc14 == new.scc14


@pytest.mark.parametrize("menu", [4, 5])
def test_other_cars_never_run_menus_4_5(monkeypatch, base_cc, menu):
  for seq in (mixed_sequence, edge_sequence, gentle_sequence):
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), module=base_cc, K9StopMode=0))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), K9StopMode=menu))
    monkeypatch.undo()
    assert ref.can == new.can and new.cc.k9_creep is None


@pytest.mark.parametrize("menu, kind", [(4, V12K9_C1), (5, V12K9_C1P)])
def test_menus_4_5_select_their_kind_and_differ_from_2(monkeypatch, menu, kind):
  outs = {}
  for m in (2, menu):
    r = mixed_sequence(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), K9StopMode=m))
    outs[m] = r.can
    if m == menu:
      assert r.cc.k9_creep is not None and r.cc.k9_creep.mode == kind and not r.cc.k9_v13_orig and not r.cc.k9_v12_orig
    monkeypatch.undo()
  assert outs[menu] != outs[2]


def test_filter_constants():
  assert (kc.V12K9_C1P_LF_HOLD, kc.V12K9_C1P_LF_FAR) == (50, 3.0)
  assert kc.V12K9_C1P_LF_FAR >= kc.LEAD_LATCH_GAP >= kc.V12K9_ZONE_LEAD
  assert kc.V12K9_C1P_END_HANDOFF
  assert not any(hasattr(kc, n) for n in ("V12K9_LF_JUMP", "V12K9_LF_NEAR", "V12K9_LF_FRAMES"))  # no delay on a positive distance
  assert kc.V12K9_C1_END_GO_SPEED == 0.0 and kc.V12K9_END_GO_SPEED == 0.15  # no go StopReq while rolling (menus 4 / 5; menu 2 = 26daac1a: 0.15)


def test_filter_object():
  c = K9Creep(V12K9_C1P)
  assert c.filter_lead(True, 40.37, 0.14) == (True, 40.37)
  lv, ld = c.filter_lead(True, 0.0, 0.14)  # visible, no distance, right after 40 m: that distance minus the own travel
  assert lv and ld == pytest.approx(40.37 - 0.0014)
  assert c.filter_lead(False, 0.0, 0.14) == (False, 0.0)  # gone
  assert c.filter_lead(True, 0.0, 0.14) == (True, 0.0)  # no distance seen before: as sent (a lead at 0 m, as 26daac1a)
  c = K9Creep(V12K9_C1P)
  c.filter_lead(True, 40.0, 1.0)
  assert c.filter_lead(True, 1.0, 1.0) == (True, 1.0)  # a positive drop 40 -> 1 m counts at once
  assert c.filter_lead(True, 0.0, 1.0) == (True, 0.0)  # a dropout next to a near lead (<= LF_FAR): as sent
  c = K9Creep(V12K9_C1P)
  c.filter_lead(True, 3.5, 10.0)
  out = [c.filter_lead(True, 0.0, 10.0) for _ in range(6)]
  assert [d for _, d in out[:4]] == pytest.approx([3.4, 3.3, 3.2, 3.1]) and out[5] == (True, 0.0)  # shortened into LF_FAR: as sent
  c = K9Creep(V12K9_C1P)
  c.filter_lead(True, 40.0, 0.1)
  out = [c.filter_lead(True, 0.0, 0.1) for _ in range(kc.V12K9_C1P_LF_HOLD + 1)]
  assert all(d > 39.0 for _, d in out[:-1]) and out[-1] == (True, 0.0)  # at most LF_HOLD frames
  for kind in (V9, V12K9, V12K9_C1):  # other kinds: unchanged (menu 2 = 26daac1a, menu 4 = C1 without the filter)
    c = K9Creep(kind)
    c.filter_lead(True, 40.0, 0.1)
    assert c.filter_lead(True, 0.0, 0.1) == (True, 0.0)


def _drive(r, v_trig, case, far=40.4):
  """pid 3.0 -> 0.25 m/s then stopping down to standstill at -0.28 m/s^2, a far radar lead pulling away; from the first frame
  below v_trig: case a = a real lead 1.0 m ahead (closing at the own speed), b = 2 frames visible with no distance then no lead,
  c0 = no change. Returns [(frame, v, StopReq, aReqValue)] of the frames that sent SCC12 from the trigger on."""
  rows, trig, x, gap = [], None, 0.0, None
  for i in range(150):
    v = max(3.0 - 0.02 * i, 0.25)
    if trig is None and v < v_trig:
      trig = 0
    lead, lv = ((far, 4.7), True)
    if trig is not None:
      if case == "a":
        gap = 1.0 - x if gap is None else gap - v * 0.01
        lead = (max(gap, 0.05), -v)
      elif case == "b":
        lead, lv = ((0.0, 0.0), True) if trig < 2 else (None, False)
      trig += 1
    _, sent = r.step(-0.8, LCS.pid, v, a_target=-0.6, a_ego=-0.6, lead=lead, lead_visible=lv)
    if trig is not None and sent is not None:
      rows.append((r.frame, v, sent["StopReq"], sent["aReqValue"]))
  v = 0.25
  for i in range(260):
    if trig is None and v < v_trig:
      trig = 0
    lead, lv = ((far, 4.7), True)
    if trig is not None:
      if case == "a":
        gap = 1.0 if gap is None else gap - v * 0.01
        lead = (max(gap, 0.05), -v)
      elif case == "b":
        lead, lv = ((0.0, 0.0), True) if trig < 2 else (None, False)
      trig += 1
    _, sent = r.step(-0.4, LCS.stopping, v, standstill=v <= 0.0, a_target=-0.28, a_ego=-0.28 if v > 0 else 0.0, lead=lead,
                     lead_visible=lv)
    if trig is not None and sent is not None:
      rows.append((r.frame, v, sent["StopReq"], sent["aReqValue"]))
    v = max(v - 0.0028, 0.0)
  return rows


@pytest.mark.parametrize("v_trig", [2.0, 1.0, 0.5, 0.139])
def test_a_real_cut_in_as_strong_as_26daac1a(monkeypatch, base_cc, v_trig):
  ref = _drive(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), module=base_cc, K9StopMode=2), v_trig, "a")
  monkeypatch.undo()
  new = _drive(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), K9StopMode=5), v_trig, "a")
  assert len(ref) == len(new) > 50 and max(sr for *_, sr, _ in ref) == 1
  for (f0, v0, s0, a0), (f1, v1, s1, a1) in zip(ref, new, strict=True):
    assert (f0, v0) == (f1, v1)
    assert s1 >= s0 and a1 <= a0 + 1e-9, (f0, v0, (s0, a0), (s1, a1))


@pytest.mark.parametrize("v_trig", [1.0, 0.5, 0.2, 0.139, 0.1])
def test_b_signal_error_no_rolling_stopreq(monkeypatch, base_cc, v_trig):
  ref = _drive(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), module=base_cc, K9StopMode=2), v_trig, "b")
  monkeypatch.undo()
  new = _drive(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), K9StopMode=5), v_trig, "b")
  if v_trig <= 0.25:  # the dropout in the stopping state (from 0.25 m/s here; 05:15: from 0.21 m/s)
    assert any(sr for _, v, sr, _ in ref if v > 0.05)  # 26daac1a: StopReq while rolling on the 2 frames
  assert not any(sr for _, v, sr, _ in new if v > 0.05)  # no StopReq while rolling
  assert any(sr for _, v, sr, _ in new if v == 0.0)  # StopReq at standstill
  monkeypatch.undo()
  same = _drive(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), K9StopMode=5), v_trig, "c0")
  assert new == same  # the 2 frames change nothing at all (no StopReq, no request step: 05:15 drive -0.06 -> -0.38 -> -0.59)


@pytest.mark.parametrize("lead", [(0.0, 0.0), (0.5, -0.3), (1.0, -0.3), (1.5, -0.3)])
def test_c_near_lead_from_the_start_identical_to_26daac1a(monkeypatch, base_cc, lead):
  def seq(r):
    for i in range(150):
      r.step(-0.8, LCS.pid, max(2.0 - 0.012 * i, 0.3), a_target=-0.6, a_ego=-0.6, lead=lead)
    v = 0.3
    for i in range(200):
      r.step(-0.5, LCS.stopping, v, standstill=v <= 0.0, a_target=-0.3, a_ego=-0.3 if v > 0 else 0.0, lead=lead)
      v = max(v - 0.003, 0.0)
    return r
  ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), module=base_cc, K9StopMode=2))
  monkeypatch.undo()
  new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), K9StopMode=5))
  assert len(new.can) > 300 and ref.can == new.can and ref.scc14 == new.scc14


def test_through_carcontroller_real_near_lead_still_latches(monkeypatch):
  """a lead closing to 1.4 m (small steps) still gets StopReq while rolling; and a dropout at 1.4 m does not lose it"""
  for drop in (False, True):
    r = Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), K9StopMode=5)
    for i in range(100):
      r.step(-0.6, LCS.pid, 4.0 - 0.03 * i, a_target=-0.5, a_ego=-0.5, lead=(8.0 - 0.06 * i, -1.0))
    sr = []
    for i in range(150):
      v = max(0.8 - 0.005 * i, 0.06)
      lead = (0.0, 0.0) if (drop and i % 5 == 0) else (1.4, -v)
      _, sent = r.step(-0.7, LCS.stopping, v, a_target=-0.3, a_ego=-0.3, lead=lead)
      if sent is not None:
        sr.append(sent["StopReq"])
    assert max(sr) == 1
    monkeypatch.undo()


def test_end_profile_hand_over_no_step(monkeypatch):
  """end profile -> standstill latch: the sent request falls by at most V12K9_HOLD_STEP per frame (+ the 0.01 CAN grid)"""
  r = Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), K9StopMode=5)
  for i in range(120):
    r.step(-0.5, LCS.pid, max(2.0 - 0.012 * i, 0.4), a_target=-0.4, a_ego=-0.4)
  for i in range(60):
    r.step(-0.5, LCS.stopping, max(0.4 - 0.01 * i, 0.06), a_target=-0.3, a_ego=-0.3)
  n0 = len(r.scc12)
  for _ in range(150):
    r.step(-0.5, LCS.stopping, 0.0, standstill=True, a_target=-0.3, a_ego=0.0)
  a = [s["aReqValue"] for _, _, s in r.scc12[n0 - 1:]]
  assert max(x - y for x, y in zip(a, a[1:], strict=False)) <= 2 * kc.V12K9_HOLD_STEP + 0.011
  assert r.scc12[-1][2]["StopReq"] == 1


@pytest.mark.parametrize("menu", [4, 5])
def test_through_carcontroller_go_target_while_rolling_no_stopreq(monkeypatch, menu):
  """26daac1a (menu 2) sent StopReq when the plan asked to go (aTarget > 0) below 0.15 m/s while still rolling; C1 / C1+ do not"""
  r = Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), K9StopMode=menu)
  for i in range(120):
    r.step(-0.5, LCS.pid, max(2.0 - 0.012 * i, 0.4), a_target=-0.4, a_ego=-0.4)
  for i in range(30):
    r.step(-0.4, LCS.stopping, max(0.4 - 0.01 * i, 0.12), a_target=-0.3, a_ego=-0.3)
  sr = []
  for i in range(40):
    _, sent = r.step(-0.2, LCS.stopping, 0.12, a_target=0.1, a_ego=-0.1)
    if sent is not None:
      sr.append(sent["StopReq"])
  assert sr and max(sr) == 0


def test_c1p_constants():
  """C1 (menu 4, kind 9) / C1+ (menu 5, kind 10): the C1 end profile (k9band) + for C1+ the predicted-gap gate with a lead;
  MX (3 m lead StopReq) as 26daac1a; menu 2 (kind 8) keeps the 26daac1a end-profile constants"""
  assert kc.MENU[4] == V12K9_C1 and kc.MENU[5] == V12K9_C1P and kc.MENU[2] == V12K9 and 3 not in kc.MENU and kc.V13_ORIG_MENU == 3
  assert kc.V12K9_C1_END_KI == 0.5 and (kc.V12K9_C1_END_REF_TAU, kc.V12K9_C1_END_PRE_T) == (0.35, 0.5)
  assert (kc.V12K9_C1P_SG_GAP, kc.V12K9_C1P_SG_LAG, kc.V12K9_C1P_SG_SMIN, kc.V12K9_C1P_SG_REACH, kc.V12K9_C1P_SG_K) == (4.0, 0.45, 0.3, True, 1.0)
  assert kc.V12K9_C1P_END_KI_SAFE == 1.25
  assert kc.V12K9_C1P_END_LEAD_GAP == 4.0 and kc.V12K9_END_LEAD_GAP == 3.0
  assert (kc.V12K9_END_KI, kc.V12K9_END_GO_SPEED) == (1.5, 0.15)  # kind 8 (menu 2) = 26daac1a
  assert (kc.V12K9_MX_GAP, kc.V12K9_MX_SPEED_LO, kc.V12K9_MX_SPEED_HI) == (3.0, 0.5, 1.0)
  for k, c1, c1p in ((V12K9, False, False), (V12K9_C1, True, False), (V12K9_C1P, True, True)):
    c = K9Creep(k)
    assert (c.v12, c.c1, c.c1p) == (True, c1, c1p)


@pytest.mark.parametrize("seed", range(3))
def test_c1p_lead_knobs_do_nothing_without_a_lead(monkeypatch, seed):
  """the gate / KI_SAFE / lead target act only with a lead in the end profile: no lead, bit for bit the same with them off"""
  import random
  from openpilot.selfdrive.car.tests.test_k9_stop_v12 import _random_inputs
  def run():
    c = K9Creep(V12K9_C1P)
    out = []
    for kw, _ in _random_inputs(random.Random(f"k9-c1p-{seed}"), 3000):
      kw = dict(kw, lead_visible=False, lead_distance=0.0)
      out.append(c.update(**kw))
    return out
  a = run()
  monkeypatch.setattr(kc, "V12K9_C1P_SG_GAP", 0.0)
  monkeypatch.setattr(kc, "V12K9_C1P_END_KI_SAFE", 0.0)
  monkeypatch.setattr(kc, "V12K9_C1P_END_LEAD_GAP", kc.V12K9_END_LEAD_GAP)
  b = run()
  assert len(a) == len(b) and all((x[1] == y[1]) and (x[0] == y[0] or (x[0] != x[0] and y[0] != y[0])) for x, y in zip(a, b, strict=True))


# --- ORDERS 8 (09:25 / 09:26): no exemption after the dropout frames ---------------------------------------------------
# 노비's three transitions from a stopping state past the interlocks, on the same raw inputs for 26daac1a and the candidate:
# a far radar lead (40 m, pulling away), then K frames visible with leadDistance 0.0 (raw dropout), then
#   pos1: a positive distance 1.0 m (a real lead, closing at the own speed)
#   gone: lead_visible 0 (no lead)
#   zero: the 0.0 goes on (no switch; raw frame HOLD + 1 = 51 is the first frame of 0.0 the dropout rule does not cover)
# K = 1 (40 m -> 0 m for 1 frame) and the HOLD boundary 49 / 50 / 51. Speeds: standstill, rolling 0.139 m/s, 0.7 m/s.
# The comparison starts on the first frame the raw dropout condition is false (fixed by the scenario: zero frame
# min(K, HOLD) + 1) and runs on EVERY frame after it, no exemption: StopReq 1 at 26daac1a -> 1 at the candidate,
# aReqValue <= 26daac1a, the lead kind 8 consumes (after filter_lead) == the raw input. Failing frames stay failures
# (no xfail / skip). Model-only, 실차 미검증; a test gap check, not evidence of weaker braking or a crash on the car.
# 2026-10-27 menu 5 (menu45_1027): the candidate is C1+ on menu 5. The body of the test is the brake_0515_1027 one unchanged
# (no xfail / skip / exemption): the 8 'gone' cases that failed on the C1+ tree (brake_0515_1027 8항) fail here too.
HOLD = 50  # = kc.V12K9_C1P_LF_HOLD (pinned in test_filter_constants); fixed here so the scenario does not read the candidate
FAR = 40.37
TRANSITIONS = [("pos1", 1), ("pos1", 49), ("pos1", 50), ("pos1", 51), ("gone", 1), ("gone", 49), ("gone", 50), ("gone", 51),
               ("zero", 0)]  # zero: the 0.0 never ends (its frames 49 / 50 / 51 are all in the drive; compared from frame 51)
SPEEDS = (0.0, 0.139, 0.7)
_SKIP_STATE = ("ref", "mode", "v12", "c1", "c1p", "lead_filter", "lead_f", "ref_out", "update")


def _f32(x):
  return struct.unpack("f", struct.pack("f", x))[0]


def creep_state(c):
  """every K9Creep attribute (and its zone reference's), for the residual-state comparison"""
  if c is None:
    return {}
  d = {k: copy.deepcopy(v) for k, v in vars(c).items() if k not in _SKIP_STATE}
  if getattr(c, "ref", None) is not None:
    d.update({f"ref.{k}": copy.deepcopy(v) for k, v in vars(c.ref).items()})
  return d


def transition_run(monkeypatch, module, v0, kind, k, dropout=True):
  """kind in pos1 / gone / zero (zero: the 0.0 goes on to the end, k unused); dropout=False: the
  control drive with the 40 m lead instead of the 0.0 frames (and the same switch after them). Returns (rows, first): one row
  per frame, first = index of the first frame the raw dropout condition is false."""
  kw = {} if module is None else {"module": module}
  r = Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K9), K9StopMode=2 if module is not None else 5, **kw)  # 26daac1a menu 2 vs C1+ menu 5
  rows, outs = [], []
  wrapped = [False]

  def step(i, accel, state, v, standstill, a_target, a_ego, lead, lv):
    _, sent = r.step(accel, state, v, standstill=standstill, a_target=a_target, a_ego=a_ego, lead=lead, lead_visible=lv)
    c = r.cc.k9_creep
    if c is not None and not wrapped[0]:
      orig = c.update
      def upd(**a):
        out = orig(**a)
        outs.append(out)
        return out
      c.update = upd
      wrapped[0] = True
      outs.append(None)  # the first frame ran before the wrap
    raw = (bool(lv) and lead is not None, 0.0 if lead is None else _f32(lead[0]))  # as the controller reads it (cereal float32)
    rows.append({"i": i, "v": v, "raw": raw, "used": tuple(c.lead_f) if hasattr(c, "lead_f") else raw,
                 "out": outs[-1] if outs else None, "StopReq": None if sent is None else sent["StopReq"],
                 "aReq": None if sent is None else sent["aReqValue"], "state": creep_state(c)})

  i = 0
  for j in range(100):  # pid approach 3.0 -> 1.0 m/s, the far lead pulling away
    step(i, -0.6, LCS.pid, max(3.0 - 0.02 * j, 1.0), False, -0.5, -0.5, (FAR, 4.7), True)
    i += 1
  v = 1.0
  while v > v0:  # stopping down to v0 at -0.5 m/s^2
    step(i, -0.5, LCS.stopping, v, False, -0.3, -0.3, (FAR, 4.7), True)
    i += 1
    v = max(round(v - 0.005, 6), v0)
  for _ in range(5):  # 5 frames at v0 (standstill: below the 12-frame standstill latch)
    step(i, -0.4, LCS.stopping, v0, v0 == 0.0, -0.28, -0.28 if v0 > 0 else 0.0, (FAR, 4.7), True)
    i += 1
  t0 = i
  n_zero = 10 ** 6 if kind == "zero" else k
  first = t0 + (HOLD if kind == "zero" else min(k, HOLD))
  gap, v = 1.0, v0
  for j in range(500):  # j < 400: v0 for 150 frames then -0.28 m/s^2 to standstill; j >= 400: standstill hold
    if j >= 150:
      v = max(round(v - 0.0028, 6), 0.0)
    if j < n_zero:
      lead, lv = ((0.0, 0.0), True) if dropout else ((FAR, 4.7), True)
    elif kind == "pos1":
      lead, lv = (max(gap, 0.05), -v), True
      gap -= v * 0.01
    else:  # gone
      lead, lv = None, False
    step(i, -0.4, LCS.stopping, v, v == 0.0, -0.28, -0.28 if v > 0 else 0.0, lead, lv)
    i += 1
  return rows, first


def compare_rows(ref, new, first):
  """frame-by-frame failures from `first` on (no exemption): StopReq, aReq, consumed lead; and the residual state diffs"""
  bad, state_diff = [], []
  assert len(ref) == len(new)
  for r0, r1 in zip(ref, new, strict=True):
    assert (r0["i"], r0["v"], r0["raw"]) == (r1["i"], r1["v"], r1["raw"])
    assert (r0["StopReq"] is None) == (r1["StopReq"] is None), r0["i"]
    common = sorted(set(r0["state"]) & set(r1["state"]))
    diff = [n for n in common if repr(r0["state"][n]) != repr(r1["state"][n])]
    state_diff.append(diff)
    if r0["i"] < first:
      continue
    if r1["used"] != r1["raw"]:
      bad.append((r0["i"], "used", r1["raw"], r1["used"]))
    if r0["StopReq"] is None:
      continue
    if r0["StopReq"] == 1 and r1["StopReq"] != 1:
      bad.append((r0["i"], "StopReq", r0["StopReq"], r1["StopReq"]))
    if r1["aReq"] > r0["aReq"] + 1e-9:
      bad.append((r0["i"], "aReq", r0["aReq"], r1["aReq"]))
  return bad, state_diff


@pytest.mark.parametrize("v0", SPEEDS)
@pytest.mark.parametrize("kind,k", TRANSITIONS)
def test_after_dropout_no_weaker_than_26daac1a(monkeypatch, base_cc, v0, kind, k):
  """ORDERS 8: from the first frame the raw dropout condition is false, on every frame: StopReq 1 at 26daac1a -> 1 at the
  candidate, aReq not weaker than 26daac1a, consumed lead == raw input; at the end the latch state equals 26daac1a"""
  ref, first = transition_run(monkeypatch, base_cc, v0, kind, k)
  monkeypatch.undo()
  new, first1 = transition_run(monkeypatch, None, v0, kind, k)
  assert first == first1 and sum(r["StopReq"] is not None for r in new[first:]) > 100
  assert any(r["raw"] == (True, 0.0) for r in new)  # the dropout frames are in the drive
  bad, state_diff = compare_rows(ref, new, first)
  assert not bad, (len(bad), bad[:6])
  last0, last1 = ref[-1]["state"], new[-1]["state"]
  assert (last0["latched"], last0["active"]) == (last1["latched"], last1["active"])
