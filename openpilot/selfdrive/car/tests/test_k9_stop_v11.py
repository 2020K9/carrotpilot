"""K9StopMode 3 (6 before the 2026-10-24 evening trim) = creep_fix v11 (the K9Creep kind V11 = 7; 2026-10-24, the driver's order 12:46 "v11 추가해라").
update() is bit-identical to a literal reference port of the page v11 code (selectMode v11 carcontroller.py block, the
changelog v10 -> v11 diff) with the K9 adaption (V11Ref(k9=True)): the page's pid creep table / accel fade / 1.5 m/s gate
and PID over-braking blend are replaced by v10's pid creep compensation (closed-loop sim: they made the K9 stop longer and
closer to the lead); the stopping phase, the re-press guard and the jerk_u profile below 0.65 m/s are the page's; ComfortBand
is 0 (as v10). The pure page code (k9=False) is kept as the reference of what was not taken. K9 deviations (not in the page,
fail toward more braking): 3.0 m lead latch, original command on disengage/override/pedal/invalid CAN/non-finite input, and
no re-press guard with a lead within 3.0 m.
Menus 0-5 are byte-identical to the frozen 6ec4a36d (carrot-wip before menu 6) controller. All new (2026-10-24).
Rewritten 2026-10-24 evening (보노보노 "브레이크 2.3.4 제거"; this menu moved 6 -> 3, kinds 3-5 removed), number references
only: menu 6 -> 3 (constants, CarController runs, K7/CAN-FD, live switch), the frozen 6ec4a36d CAN comparison for new 0/1
and new 2 = 6ec4a36d 5 (its 2/3/4 are removed kinds), the kind comparison for kinds 1/2/6, removed-kind flags dropped,
4 and up act as 0, live switch starts 1/2 (old 4/5 -> 2). Sim/replay only; NOT verified on the vehicle."""
import math
import random

import numpy as np
import pytest

from opendbc.car import structs
from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V9, V10, V10_LIGHT, V11
from opendbc.car.hyundai.values import CAR, HyundaiFlags, CarControllerParams
from openpilot.selfdrive.car.tests import k9_rig
from openpilot.selfdrive.car.tests.k9_rig import LCS, Rig, load_frozen_cc
from openpilot.selfdrive.car.tests.test_k9_creep_mode import SEQS, FakeCp, _random_inputs, patch_state, rig_kind
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import K9_FLAGS, make_cp
from openpilot.selfdrive.car.tests.test_k9_stop_v10 import _stops

FROZEN = "6ec4a36d"  # carrot-wip head before menu 6 (0 base, 1 v10, 2 v9 + shudder fix, 3 v10 + band 0.2, 4/5 v10 + (light) shudder fix)


class V11Ref:
  """Literal port of the creep_fix v11 page code (carcontroller.py [변경 1], CS/actuators as plain arguments).
  k9=True: the K9 adaption, the page's pid lines 'v_ego < 1.5', the blend, creep_comp and fade_by_accel replaced by the v10
  page lines (v_ego < 1.6, creep_comp [0.25, 0.55, 1.0, 1.6] -> [-0.26, -0.26, -0.20, 0.0], fade_by_accel [-0.80, -0.30] ->
  [0.25, 1.0]); the re-press guard stays."""
  def __init__(self, k9=True):
    self.k9 = k9

  def step(self, accel, stopping, a_target, pid, v_ego, v_ego_raw, a_ego, long_accel, esp_standstill):
    lon_ac = float(long_accel) if long_accel is not None else float(a_ego)
    eff_ac = float(min(lon_ac, a_ego + 0.10))
    if stopping:
      raw_standstill = bool(esp_standstill or v_ego_raw < 0.05 or v_ego < 0.05)
      self.standstill_count = getattr(self, "standstill_count", 0) + 1 if raw_standstill else 0
      real_standstill = bool(self.standstill_count >= 12)
      if real_standstill or a_target <= -1.0 or (v_ego > 0.14 and eff_ac > 0.04) or a_ego > 0.06:
        self.stop_req_latched = True
      if not getattr(self, "stop_req_latched", False):
        stopping = False
        stop_feather = float(np.interp(v_ego, [0.05, 0.11, 0.16, 0.25, 0.42], [-0.28, -0.29, -0.34, -0.42, -0.52]))
        min_decel = float(np.interp(v_ego, [0.06, 0.12, 0.25, 0.40], [-0.04, -0.09, -0.20, -0.30]))
        a_boost = float(np.interp(eff_ac - min_decel, [0.0, 0.15], [0.0, -0.18]))
        target_accel = float(stop_feather + a_boost)
        prev_accel = float(getattr(self, "accel_feather_last", accel))
        down_step = 0.005 if ((v_ego > 0.14 and eff_ac > -0.02) or prev_accel > stop_feather + 0.02) else 0.0
        up_step = float(np.interp(eff_ac - min_decel, [-0.08, 0.02], [0.0032, 0.0])) if v_ego > 0.12 else 0.0030
        accel = float(np.clip(target_accel, prev_accel - down_step, prev_accel + up_step))
        self.accel_feather_last = float(accel)
      else:
        prev_accel = float(getattr(self, "accel_feather_last", accel))
        accel = float(max(min(accel, -0.50), prev_accel - 0.0025)) if a_target > -1.0 else float(min(accel, -0.50))
        self.accel_feather_last = float(accel)
    else:
      self.stop_req_latched = False
      self.standstill_count = 0
      if pid and v_ego < (1.6 if self.k9 else 1.5) and a_target < -0.05:
        if not self.k9 and v_ego < 1.05 and a_target > -0.80 and accel < a_target - 0.12:
          blend = float(np.interp(v_ego, [0.50, 1.05], [0.55, 0.0]))
          accel = float(accel * (1.0 - blend) + (a_target - 0.12) * blend)
        if self.k9:
          creep_comp = float(np.interp(v_ego, [0.25, 0.55, 1.0, 1.6], [-0.26, -0.26, -0.20, 0.0]))
        else:
          creep_comp = float(np.interp(v_ego, [0.22, 0.45, 0.85, 1.45], [-0.26, -0.22, -0.08, 0.0]))
        fade_by_target = float(np.interp(a_target, [-0.20, -0.05], [1.0, 0.0]))
        if self.k9:
          fade_by_accel = float(np.interp(accel, [-0.80, -0.30], [0.25, 1.0]))
        else:
          fade_by_accel = float(np.interp(accel, [-0.75, -0.28], [0.20, 1.0]))
        accel = float(np.clip(accel + creep_comp * fade_by_target * fade_by_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
        prev_accel = float(getattr(self, "accel_feather_last", accel))
        if v_ego < 0.95 and a_target > -0.80 and eff_ac < -0.18:
          accel = float(max(accel, prev_accel))
      self.accel_feather_last = float(accel)
    return accel, stopping


def page_jerk_u(jerk_u, v):
  if v < 0.65:
    jerk_u = max(jerk_u, float(np.interp(v, [0.05, 0.08, 0.14, 0.45, 0.65], [0.7, 0.9, 1.8, 1.8, 0.8])))
  return jerk_u


def _upd(c, f, **kw):
  args = dict(accel=f["accel"], a_target=f["a_target"], stopping=f["stopping"], pid=f["pid"], v_ego=f["v_ego"],
              v_ego_raw=f["v_ego_raw"], a_ego=f["a_ego"], long_accel=f["long_accel"], esp_standstill=f["esp_standstill"],
              long_active=True, override=False, gas=False, brake=False, can_valid=True, lead_visible=False, lead_distance=0.0)
  args.update(kw)
  return c.update(**args)


def upd(c, **kw):
  args = dict(accel=-0.6, a_target=-0.4, stopping=True, pid=False, v_ego=0.3, v_ego_raw=0.3, a_ego=-0.3, long_accel=-0.3,
              esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True,
              lead_visible=False, lead_distance=0.0)
  args.update(kw)
  return c.update(**args)


@pytest.fixture(scope="module")
def frozen_cc(tmp_path_factory):
  return load_frozen_cc(FROZEN, tmp_path_factory.mktemp("cc_6ec4a36d"), "cc_6ec4a36d")


def test_menu_and_constants():
  assert kc.MENU[3] == V11 == 7 and kc.MENU[2] == V10_LIGHT and kc.MENU[1] == V10 and 3 in kc.MODES
  assert carcontroller.K9_CREEP_MENU[3] == V11 and kc.KINDS[-1] == V11
  c = K9Creep(V11)
  assert c.v11 and not c.v10 and not c.v10_light
  assert not any(K9Creep(k).v11 for k in (V9, V10, V10_LIGHT))
  assert kc.V11_STANDSTILL_FRAMES == 12 and kc.V11_HOLD_STEP == 0.0025 and kc.V11_UP_LOW == 0.0030
  assert (kc.V11_FEATHER_BP, kc.V11_FEATHER_V) == ([0.05, 0.11, 0.16, 0.25, 0.42], [-0.28, -0.29, -0.34, -0.42, -0.52])
  assert (kc.V11_MIN_DECEL_BP, kc.V11_MIN_DECEL_V) == ([0.06, 0.12, 0.25, 0.40], [-0.04, -0.09, -0.20, -0.30])
  assert (kc.V11_GUARD_SPEED, kc.V11_GUARD_EFF_AC, kc.V11_GUARD_MIN_TARGET, kc.V11_DOWN_ABOVE) == (0.95, -0.18, -0.80, 0.02)
  assert not any(hasattr(kc, k) for k in ("V11_CREEP_BP", "V11_BLEND_SPEED", "V11_FADE_ACCEL_BP", "V11_PID_SPEED"))  # not taken (K9)
  assert (kc.V11_JERK_U_SPEED, kc.V11_JERK_U_BP, kc.V11_JERK_U_V) == (0.65, [0.05, 0.08, 0.14, 0.45, 0.65], [0.7, 0.9, 1.8, 1.8, 0.8])
  assert kc.LEAD_LATCH_GAP == 3.0 and kc.LATCH_ACCEL == -0.50 and kc.URGENT_TARGET == -1.0


@pytest.mark.parametrize("seed", range(8))
def test_mode7_is_bit_identical_to_the_v11_page_code_k9_adaption(seed):
  for frames in _stops(seed):
    ref, c = V11Ref(k9=True), K9Creep(V11)
    for f in frames:
      assert _upd(c, f) == ref.step(**f)  # exact float equality, accel and StopReq


@pytest.mark.parametrize("seed", range(6))
def test_mode7_is_the_page_code_k9_adaption_on_pid_heavy_sequences(seed):
  # low-speed pid decel with PID over-braking (blend), strong decel (guard) and late stopping entry (down_step)
  rng = random.Random(f"k9-v11-pid-{seed}")
  for _ in range(10):
    ref, c = V11Ref(k9=True), K9Creep(V11)
    v, a_ego = rng.uniform(0.4, 1.6), -0.3
    stop_at = rng.randint(100, 300)
    for i in range(stop_at + rng.randint(50, 200)):
      stopping = i >= stop_at
      a_target = rng.uniform(-0.9, -0.03)
      accel = float(np.clip(a_target - rng.uniform(-0.1, 0.6), CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      a_ego = float(np.clip(a_ego + rng.gauss(0.0, 0.05), -1.2, 0.2))
      la = None if rng.random() < 0.05 else a_ego + rng.gauss(0.0, 0.1)
      v = max(0.0, v + min(a_ego, -0.05) * 0.01)
      f = dict(accel=accel, stopping=stopping, a_target=a_target, pid=not stopping, v_ego=v, v_ego_raw=v, a_ego=a_ego,
               long_accel=la, esp_standstill=v <= 0.0)
      assert _upd(c, f) == ref.step(**f)


def test_standstill_latch_after_12_frames_and_count_restarts():
  c = K9Creep(V11)
  sr = [upd(c, v_ego=0.0, v_ego_raw=0.0, a_ego=0.0, long_accel=0.0)[1] for _ in range(14)]
  assert sr == [False] * 11 + [True] * 3
  c = K9Creep(V11)
  for _ in range(8):
    assert upd(c, v_ego=0.03, v_ego_raw=0.03, a_ego=-0.05, long_accel=-0.05)[1] is False
  upd(c, v_ego=0.06, v_ego_raw=0.06, a_ego=-0.05, long_accel=-0.05)  # one moving frame: count restarts
  sr = [upd(c, v_ego=0.03, v_ego_raw=0.03, a_ego=-0.05, long_accel=-0.05)[1] for _ in range(13)]
  assert sr == [False] * 11 + [True] * 2
  c = K9Creep(V11)  # TCS13 StandStill counts like the speed
  sr = [upd(c, v_ego=0.2, v_ego_raw=0.2, esp_standstill=True)[1] for _ in range(12)]
  assert sr == [False] * 11 + [True]
  c = K9Creep(V11)  # a pid frame in between restarts the count
  for _ in range(10):
    upd(c, v_ego=0.0, v_ego_raw=0.0, a_ego=0.0, long_accel=0.0)
  upd(c, stopping=False, pid=True, v_ego=0.0, v_ego_raw=0.0)
  sr = [upd(c, v_ego=0.0, v_ego_raw=0.0, a_ego=0.0, long_accel=0.0)[1] for _ in range(12)]
  assert sr == [False] * 11 + [True] and c.ss_count == 12


@pytest.mark.parametrize("kw", [{"a_target": -1.0}, {"a_target": -1.3}, {"a_ego": 0.07}, {"long_accel": 0.05, "a_ego": 0.0},
                                {"lead_visible": True, "lead_distance": 2.9}, {"lead_visible": True, "lead_distance": 3.0},
                                {"lead_visible": False, "lead_distance": 1.0}])
def test_other_latches_fire_at_once(kw):
  c = K9Creep(V11)
  assert upd(c, **kw)[1] is True and c.latched


@pytest.mark.parametrize("lead", [3.01, 5.0, 0.0])
def test_far_or_no_lead_does_not_latch(lead):
  c = K9Creep(V11)
  assert upd(c, lead_visible=lead != 0.0, lead_distance=lead)[1] is False


def test_hold_ramp_0_0025_per_frame_and_urgent_at_once():
  c = K9Creep(V11)
  upd(c, stopping=False, pid=True, accel=-0.3, a_target=-0.04, v_ego=0.2, v_ego_raw=0.2)
  for _ in range(30):
    upd(c, v_ego=0.2, v_ego_raw=0.2, a_ego=-0.25, long_accel=-0.25)
  start = c.last
  assert -0.45 < start < -0.2
  out = [upd(c, v_ego=0.0, v_ego_raw=0.0, a_ego=0.0, long_accel=0.0, accel=-0.7)[0] for _ in range(220)]
  held = out[11:]  # frames 0-10: standstill but not latched yet (feather continues)
  assert held[0] == pytest.approx(out[10] - 0.0025)
  assert all(b - a == pytest.approx(-0.0025) or b == -0.7 for a, b in zip(held, held[1:]) if a > -0.7 + 0.0025)
  assert held[-1] == -0.7 and min(held) == -0.7
  c2 = K9Creep(V11)
  for _ in range(30):
    upd(c2, v_ego=0.2, v_ego_raw=0.2, a_ego=-0.25, long_accel=-0.25)
  assert upd(c2, a_target=-1.2, accel=-0.9) == (-0.9, True)  # urgent target: at once
  assert upd(c2, a_target=-1.2, accel=-0.3) == (-0.5, True)


def test_late_entry_feather_falls_toward_stop_feather():
  c = K9Creep(V11)
  upd(c, stopping=False, pid=True, accel=-0.15, a_target=-0.04, v_ego=0.14, v_ego_raw=0.14)  # last = -0.15
  out = [upd(c, v_ego=0.14, v_ego_raw=0.14, a_ego=-0.10, long_accel=-0.10, accel=-0.15)[0] for _ in range(40)]
  feather = float(np.interp(0.14, kc.V11_FEATHER_BP, kc.V11_FEATHER_V))
  assert out[0] == pytest.approx(-0.155) and out[1] == pytest.approx(-0.16)
  assert min(out) >= feather - 0.2 and out[-1] < -0.3
  c10 = K9Creep(V10)  # v10 stays stuck near the entry value here (the page's -0.19 trap)
  upd(c10, stopping=False, pid=True, accel=-0.15, a_target=-0.04, v_ego=0.14, v_ego_raw=0.14)
  out10 = [upd(c10, v_ego=0.14, v_ego_raw=0.14, a_ego=-0.10, long_accel=-0.10, accel=-0.15)[0] for _ in range(40)]
  assert out10[-1] > -0.16


def test_pid_phase_is_v10_without_the_guard():
  # K9 adaption: no page blend, v10's creep compensation; while the re-press guard is off (eff_ac >= -0.18 or >= 0.95 m/s
  # or aTarget <= -0.80 or a lead within 3 m) the pid phase is exactly menu 1's (kind 2)
  rng = random.Random("k9-v11-pid-v10")
  for _ in range(4000):
    kw = dict(stopping=False, pid=True, v_ego=rng.uniform(0.0, 2.0), accel=rng.uniform(-2.0, 0.3), a_target=rng.uniform(-1.2, 0.1),
              a_ego=rng.uniform(-0.17, 0.1), lead_visible=rng.random() < 0.3, lead_distance=rng.choice([0.0, 2.0, 5.0]))
    kw["v_ego_raw"] = kw["v_ego"]; kw["long_accel"] = kw["a_ego"]
    assert upd(K9Creep(V11), **kw) == upd(K9Creep(V10), **kw)
  for v in (0.3, 0.5, 0.8, 1.0):  # PID over-braking (the page would blend): same as v10
    kw = dict(stopping=False, pid=True, v_ego=v, v_ego_raw=v, a_ego=-0.1, long_accel=-0.1, accel=-1.0, a_target=-0.4)
    assert upd(K9Creep(V11), **kw) == upd(K9Creep(V10), **kw)
    assert upd(K9Creep(V11), **kw)[0] < -1.0  # v10 creep comp adds braking; no release toward aTarget - 0.12
  ref, page = V11Ref(k9=True), V11Ref(k9=False)
  f = dict(accel=-1.0, stopping=False, a_target=-0.4, pid=True, v_ego=0.5, v_ego_raw=0.5, a_ego=-0.1, long_accel=-0.1, esp_standstill=False)
  assert page.step(**f)[0] > ref.step(**f)[0]  # what was not taken: the page brakes less here


def test_pid_repress_guard_and_lead_skip():
  def run(lead=0.0, a_target=-0.4, eff=-0.3):
    c = K9Creep(V11)
    out = []
    for i in range(20):
      out.append(upd(c, stopping=False, pid=True, v_ego=0.8, v_ego_raw=0.8, a_ego=eff, long_accel=eff,
                     accel=-0.4 - 0.02 * i, a_target=a_target, lead_visible=lead != 0.0, lead_distance=lead)[0])
    return out
  g = run()
  assert all(b >= a for a, b in zip(g, g[1:]))  # no further fall while decelerating harder than -0.18
  assert all(b < a for a, b in zip(run(eff=-0.1), run(eff=-0.1)[1:]))  # weak decel: the command may fall
  assert all(b < a for a, b in zip(run(lead=2.5), run(lead=2.5)[1:]))  # K9: lead within 3 m, no guard
  assert all(b < a for a, b in zip(run(a_target=-0.85), run(a_target=-0.85)[1:]))  # strong plan, no guard


@pytest.mark.parametrize("v", [0.0, 0.03, 0.05, 0.07, 0.1, 0.14, 0.3, 0.45, 0.55, 0.64, 0.65, 0.9, 1.4, 3.0, 20.0])
@pytest.mark.parametrize("ju", [0.5, 1.0, 2.5, 5.0])
def test_jerk_profile_and_band_0(v, ju):
  c = K9Creep(V11)
  assert c.bands(ju, v, 0.8, 0.7, 0.6, 0.5) == (page_jerk_u(ju, v), 0.0, 0.0)
  if v >= 0.65:
    assert c.bands(ju, v, 0.8, 0.7, 0.6, 0.5)[0] == ju


@pytest.mark.parametrize("kw", [{"gas": True}, {"brake": True}, {"override": True}, {"long_active": False}, {"can_valid": False},
                                {"v_ego": float("nan")}, {"accel": float("inf")}])
def test_interlocks_return_original(kw):
  c = K9Creep(V11)
  for _ in range(5):
    upd(c)
  args = dict(accel=-0.6, stopping=True)
  args.update(kw)
  got = upd(c, **args)
  assert got[1] is True and (got[0] == args["accel"] or (math.isnan(got[0]) and math.isnan(args["accel"])))
  assert c.last is None and not c.latched and c.ss_count == 0 and not c.active


def test_random_inputs_finite_and_bounded():
  rng = random.Random("k9-v11-random")
  c = K9Creep(V11)
  for kw, i in _random_inputs(rng, 20000):
    a, sr = c.update(**kw)
    if c.active:
      assert math.isfinite(a) and CarControllerParams.ACCEL_MIN <= a <= CarControllerParams.ACCEL_MAX, (i, kw)
      if not kw["stopping"]:
        assert sr is False
      if kw["stopping"] and kw["a_target"] <= -1.0:
        assert sr is True and a <= -0.5


def _stop_run(r, lead=None):
  out = [r.step(-0.6, LCS.pid, 5.0, a_target=-0.5, a_ego=-0.5, lead=lead) for _ in range(60)]
  for i in range(160):
    out.append(r.step(-0.6, LCS.pid, 5.0 - 0.028 * i, a_target=-0.5, a_ego=-0.5, lead=lead))
  for i in range(150):
    out.append(r.step(-0.7, LCS.stopping, max(0.5 - 0.004 * i, 0.06), a_target=-0.3, a_ego=-0.3, lead=lead))
  for i in range(40):
    out.append(r.step(-0.7, LCS.stopping, 0.0, standstill=True, a_target=-0.3, a_ego=0.0, lead=lead))
  return out


def test_through_carcontroller_menu_6(monkeypatch):
  """menu 6 runs kind 7 in the CarController: ComfortBand 0 wherever applied, StopReq only from the 12th standstill
  frame, the hold ramps 0.0025 per frame, JerkUpper follows the page profile below 0.65 m/s."""
  r = Rig(monkeypatch, K9StopMode=3)
  out = _stop_run(r)
  assert r.cc.k9_creep.mode == V11 and r.cc.k9_creep_mode == 3
  sent = [s for _, s in out]
  sr = [s["StopReq"] for s in sent if s is not None]
  st = [i for i, s in enumerate(sent) if s is not None]
  first = next(i for i, s in zip(st, sr) if s == 1)
  assert first >= 370 + 11  # standstill (vEgo 0) from frame 370; latch on the 12th frame
  assert all(s == 0 for i, s in zip(st, sr) if i < 370 + 11)
  held = [o[0] for o in out[first:]]
  assert all(b - a >= -0.0025 - 1e-6 for a, b in zip(held, held[1:])) and held[-1] < held[0]
  for f, v, d in r.scc14[5:]:
    assert d["ComfortBandUpper"] == 0.0 and d["ComfortBandLower"] == 0.0


def test_through_carcontroller_menu_6_lead_latch(monkeypatch):
  r = Rig(monkeypatch, K9StopMode=3)
  out = _stop_run(r, lead=(2.9, 0.0))
  sent = [s for _, s in out]
  first_stop = 220  # stopping starts
  srs = [s["StopReq"] for i, s in enumerate(sent) if s is not None and i >= first_stop + 2]
  assert srs and all(s == 1 for s in srs)  # lead within 3 m: StopReq at once (K9 deviation kept)


def test_soft_hold_keeps_stopreq(monkeypatch):
  rows = []
  patch_state(monkeypatch, softHoldActive=1)
  r = rig_kind(monkeypatch, V11)
  for _ in range(100):
    rows.append(r.step(-0.6, LCS.stopping, 0.3, a_target=-0.4, a_ego=-0.3))
  assert all(c is None or c["StopReq"] == 1 for _, c in rows)


def test_tcs13_standstill_latches_stopreq_after_12_frames(monkeypatch):
  patch_state(monkeypatch, cp=FakeCp({"TCS13": {"StandStill": 1}, "ESP12": {"LONG_ACCEL": -0.3}}))
  r = rig_kind(monkeypatch, V11)
  rows = [r.step(-0.6, LCS.stopping, 0.4, a_target=-0.4, a_ego=-0.3) for _ in range(80)]
  sr = [(i, c["StopReq"], c["aReqValue"]) for i, (_, c) in enumerate(rows) if c is not None]
  assert all(s == 0 for i, s, _ in sr if i < 11) and all(s == 1 for i, s, _ in sr if i >= 11)
  assert all(a <= -0.2 for _, _, a in sr)


@pytest.mark.parametrize("seq", list(SEQS))
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0, 1, 2])
def test_menus_0_to_5_identical_can_to_6ec4a36d(monkeypatch, frozen_cc, mode, fingerprint, seq):
  kw = {} if mode is None else {"K9StopMode": mode}
  old = {} if mode is None else {"K9StopMode": {2: 5}.get(mode, mode)}  # new 2 = 6ec4a36d 5 (evening trim)
  ref = SEQS[seq](Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), module=frozen_cc, **old))
  monkeypatch.undo()
  new = SEQS[seq](Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), **kw))
  assert len(new.can) > 400 and ref.can == new.can
  assert ref.scc14 == new.scc14


@pytest.mark.parametrize("seq", list(SEQS))
def test_menu_6_on_k7_and_canfd_is_menu_0_of_6ec4a36d(monkeypatch, frozen_cc, seq):
  for cp in (make_cp(fingerprint=CAR.KIA_K7), make_cp(flags=int(K9_FLAGS & ~HyundaiFlags.CAMERA_SCC))):
    ref = SEQS[seq](Rig(monkeypatch, cp=cp, module=frozen_cc, K9StopMode=0))
    monkeypatch.undo()
    new = SEQS[seq](Rig(monkeypatch, cp=cp, K9StopMode=3))
    assert ref.can == new.can and new.cc.k9_creep is None
    monkeypatch.undo()
  r = Rig(monkeypatch, cp=make_cp(flags=int(K9_FLAGS | HyundaiFlags.CANFD)), K9StopMode=3)
  assert r.cc.k9_creep_applies is False and r.cc.k9_creep is None


@pytest.mark.parametrize("seed", range(6))
@pytest.mark.parametrize("kind", (V9, V10, V10_LIGHT))
def test_kinds_1_to_6_object_identical_to_6ec4a36d(frozen_cc, kind, seed):
  rng = random.Random(f"k9-v11-frozen-{seed}")
  old, new = frozen_cc.K9Creep(kind), K9Creep(kind)
  assert old.__class__ is not K9Creep
  pu_o = pl_o = pu_n = pl_n = 0.0
  for kw, i in _random_inputs(rng, 4000):
    ro, rn = old.update(**kw), new.update(**kw)
    assert ro == rn or (math.isnan(ro[0]) and math.isnan(rn[0]) and ro[1] == rn[1]), (i, kw)
    if i % 2 == 0 and old.active:
      a0 = rn[0] if math.isfinite(rn[0]) else 0.0
      cu, cl = float(np.clip(0.9 + a0 * 0.2, 0, 1.2)), float(np.clip(0.8 + a0 * 0.2, 0, 1.2))
      ju = rng.uniform(0.5, 5.0)
      bo = old.bands(ju, kw["v_ego"], cu, cl, pu_o, pl_o)
      bn = new.bands(ju, kw["v_ego"], cu, cl, pu_n, pl_n)
      assert bo == bn, i
      (_, pu_o, pl_o), (_, pu_n, pl_n) = bo, bn


def test_menu_7_and_up_act_as_0(monkeypatch):
  for value in (4, 5, 6, 7, 8, 99, -1):
    r = Rig(monkeypatch, K9StopMode=value)
    r.step(-0.5, LCS.pid, 3.0)
    assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0
    monkeypatch.undo()


@pytest.mark.parametrize("start", [1, 2])
def test_live_switch_to_6_starts_fresh(monkeypatch, start):
  r = Rig(monkeypatch, K9StopMode=start)
  for _ in range(60):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
  r.params.values["K9StopMode"] = 3
  for _ in range(60):
    r.step(-0.6, LCS.pid, 1.0, a_target=-0.4, a_ego=-0.3)
  assert r.cc.k9_creep.mode == V11 and r.cc.k9_creep_mode == 3
  assert float(r.cc.hyundai_jerk.cb_upper) == 0.0 and float(r.cc.hyundai_jerk.cb_lower) == 0.0


def test_canfd_jerk_branch_ignores_mode7():
  def run(obj=None):
    j = carcontroller.HyundaiJerk()
    cp = make_cp(flags=int(K9_FLAGS | HyundaiFlags.CANFD))
    cs = k9_rig.make_ci_state()
    cs.out = structs.CarState(vEgo=0.3, aEgo=-0.3)
    act = structs.CarControl().actuators
    act.longControlState, act.accel = LCS.stopping, -0.6
    j.make_jerk(cp, cs, -0.6, act, structs.CarControl().hudControl, creep=obj is not None, creep_obj=obj)
    return j.jerk_u, j.jerk_l, j.cb_upper, j.cb_lower
  assert run() == run(K9Creep(V11))
