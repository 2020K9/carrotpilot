"""K9StopTaper property/regression tests over seeded, diverse input streams (no plant, no vehicle model).

Each frame of a deterministic random stream (stops, standstill flicker, pedals/interlocks, lead disagreement,
non-finite and huge finite values, strong braking, sign flips) is checked against the module contract written in
k9_stop_taper.py. CarController streams check that OFF (missing/false) sends CAN identical to the pre-candidate
controller and that ON never strengthens aReq or changes StopReq/ACCMode. These check source logic only; they say
nothing about K9 stopping distance, nod, comfort or safety (see docs/k9_alt11_overnight_20261020.md).
"""
import math
import random

import pytest

from opendbc.can import CANParser
from opendbc.car import structs
from opendbc.car.hyundai import k9_stop_taper as kt
from opendbc.car.hyundai.k9_stop_taper import K9StopTaper, TaperPhase
from opendbc.car.hyundai.values import CAR
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import make_ci_state, make_cp
from openpilot.selfdrive.car.tests.test_k9_stop_taper import Rig, base_cc  # noqa: F401  (module fixture)

LCS = structs.CarControl.Actuators.LongControlState
EPS = 1e-9
SEEDS = range(32)
INTERLOCKS = ("long_active", "override", "gas", "brake", "can_valid", "stopping_or_pid")
FINITE_KEYS = ("accel", "a_target", "v_ego", "a_ego", "lead_distance", "lead_rel_speed")


def clean_stop_prefix():
  """Deterministic fault-free stop (cruise 3 m/s -> -0.6 m/s^2 to standstill -> hold -> depart to 3 m/s).

  Added for the 1 km/h window: the speed is observed input only (no internal state seeding) and the prefix draws no
  random numbers, so the seeded random stream that follows is frame-for-frame the original one."""
  base = {"standstill": False, "long_active": True, "override": False, "gas": False, "brake": False,
          "can_valid": True, "stopping_or_pid": True, "lead_visible": False, "lead_distance": 0.0, "lead_rel_speed": 0.0}
  v = 3.0
  for _ in range(30):
    yield base | {"accel": -0.6, "a_target": -0.6, "v_ego": v, "a_ego": 0.0}
  while v > 0.0:
    v = max(v - 0.6 * kt.DT, 0.0)
    yield base | {"accel": -0.6, "a_target": -0.6, "v_ego": v, "a_ego": -0.6, "standstill": v == 0.0}
  for _ in range(150):
    yield base | {"accel": -0.6, "a_target": -0.6, "v_ego": 0.0, "a_ego": 0.0, "standstill": True}
  while v < 3.0:
    v = min(v + 0.04, 3.0)
    yield base | {"accel": 0.3, "a_target": 0.3, "v_ego": v, "a_ego": 0.4}


def coverage_prelude():
  """Deterministic coverage prelude (no random draws): one distance_budget episode (hold 0.27 m/s with decel reported)
  and one interlock episode (gas pressed in taper), each after a real rearm run at 3 m/s, then clean_stop_prefix().
  Reasons from here are deterministic by construction and are counted separately from the seeded random frames."""
  base = {"accel": -1.0, "a_target": -1.0, "a_ego": -0.5, "standstill": False, "long_active": True, "override": False,
          "gas": False, "brake": False, "can_valid": True, "stopping_or_pid": True, "lead_visible": False,
          "lead_distance": 0.0, "lead_rel_speed": 0.0}
  for kind in ("distance_budget", "interlock"):
    for _ in range(30):
      yield base | {"v_ego": 3.0, "a_ego": 0.0}
    v = 3.0
    while v > 0.27:
      v = max(v - 1.0 * kt.DT, 0.27)
      yield base | {"v_ego": v}
    for i in range(200):
      yield base | {"v_ego": 0.27, "gas": kind == "interlock" and i >= 20}
  yield from clean_stop_prefix()


PRELUDE_LEN = sum(1 for _ in coverage_prelude())
# Seeded random frames per coverage stream. 6000 left 7/32 seeds (3,7,11,13,14,19,24) with no random-frame episode:
# an invalid/interlock draw below REARM_SPEED during a slow approach disarms until the next >=1.5 m/s run, which never
# came before the stream ended. 12000 continues the same RNG sequence (first 6000 random frames unchanged, same fault
# draws/distribution and decel ranges) so later stops can rearm; no seed selection.
RANDOM_N = 12000


def frames(seed, n=6000, prefix=True):
  """Regime chain cruise -> approach -> standstill -> depart with sparse fault/edge events; yields update kwargs.

  With prefix=True the deterministic clean_stop_prefix() runs first, then the unchanged n seeded random frames."""
  if prefix:
    yield from coverage_prelude()
  r = random.Random(seed)
  v, regime, left, a_cmd, lead, grip = 4.0, "cruise", 0, -1.0, None, 1.0
  for _ in range(n):
    if left <= 0 or (regime == "approach" and v == 0.0):
      regime = {"cruise": "approach", "approach": "stand", "stand": "depart", "depart": "cruise"}[regime]
      left, a_cmd = r.randint(40, 160) if regime != "approach" else 10_000, r.uniform(-1.45, -0.3)
      grip = r.choice([1.0, 1.0, 1.0, 0.4, 0.1])  # weak response (downhill-like): budget/no-progress guards
      # per-stop lead: none, far, near the 2 m gap, plan/radar disagreement (radar-only range), closing
      lead = r.choice([None, None, None, (r.uniform(4.0, 30.0), r.uniform(-1.2, 0.3), True),
                       (r.uniform(2.0, 3.0), r.uniform(-0.3, 0.0), True), (r.uniform(3.0, 12.0), -0.2, False),
                       (r.uniform(2.3, 5.0), r.uniform(-1.4, -0.6), True),  # closing: lead need binds the floor
                       (r.uniform(0.5, 2.0), r.uniform(-0.3, 0.0), r.random() < 0.5)])  # too close, plan or radar-only
    left -= 1
    a_ego = r.uniform(-0.3, 0.3)
    if regime == "cruise":
      v = min(max(v + r.uniform(-0.05, 0.05), 1.6), 6.0)
    elif regime == "approach":  # kinematically consistent deceleration so the taper can run to standstill
      a_ego = a_cmd * grip * r.uniform(0.85, 1.3)
      v = max(v + a_ego * kt.DT, 0.0)
    elif regime == "stand":
      v = 0.0
    else:
      v = min(v + r.uniform(0.0, 0.04), 3.0)
    accel = a_cmd + r.uniform(-0.05, 0.05) if regime in ("approach", "stand") else r.uniform(-0.8, 0.8)
    kw = {"accel": accel, "a_target": accel + r.uniform(-0.1, 0.1),
          "v_ego": max(v + r.gauss(0.0, 0.005), 0.0) if v > 0.0 else 0.0, "a_ego": a_ego, "standstill": v == 0.0,
          "long_active": True, "override": False, "gas": False, "brake": False, "can_valid": True,
          "stopping_or_pid": True, "lead_visible": False, "lead_distance": 0.0, "lead_rel_speed": 0.0}
    if lead is not None:
      kw.update(lead_distance=lead[0], lead_rel_speed=lead[1], lead_visible=lead[2])
    e = r.random() * (0.5 if seed % 2 else 3.0)  # odd seeds: six times the fault/edge density of even ones
    if e < 0.002:
      k = r.choice(INTERLOCKS)
      kw[k] = not kw[k]
    elif e < 0.003:
      kw[r.choice(FINITE_KEYS)] = r.choice([math.nan, math.inf, -math.inf])
    elif e < 0.0035:
      kw[r.choice(("v_ego", "a_ego", "lead_distance"))] = r.choice([1e9, -1e9, 3.4e38])  # finite outliers
    elif e < 0.012:
      kw["standstill"] = not kw["standstill"]  # flicker at either edge
    elif e < 0.017:
      if r.random() < 0.3:  # original strengthens past STRENGTHEN_ABORT while staying above STRONG_ACCEL
        kw["accel"] = kw["a_target"] = max(kw["accel"] - 0.4, -1.49)
      else:
        kw["accel" if r.random() < 0.5 else "a_target"] = r.choice([-1.5, -2.5, 0.0, 0.3])
    elif e < 0.022:  # single-frame lead glitch: close, zero/negative range, or hasLead without range
      d = r.choice([r.uniform(-1.0, 2.0), 0.0, r.uniform(2.05, 30.0)])
      kw.update(lead_visible=r.random() < 0.7, lead_distance=d, lead_rel_speed=r.uniform(-1.5, 0.5))
    yield kw


def spec_floor(v, kw):
  """Independent restatement of the documented floor: -(END_DECEL + FLOOR_SLOPE v), lead need closing^2/(2(d-gap))."""
  floor = -(0.3 + 1.0 * max(v, 0.0))
  if (kw["lead_visible"] or kw["lead_distance"] != 0.0) and kw["lead_distance"] > 0.0:
    floor = min(floor, -max(0.0, -kw["lead_rel_speed"]) ** 2 / (2.0 * max(kw["lead_distance"] - 2.0, 0.05)))
  return floor


def _interlocked(kw):
  return (not kw["long_active"] or kw["override"] or kw["gas"] or kw["brake"] or not kw["can_valid"] or
          not kw["stopping_or_pid"])


def _same(a, b):
  return a == b or (math.isnan(a) and math.isnan(b))


def check_stream(seed):
  """Asserts the frame contract over every frame (prelude included); returns (episodes entered, modified frames)
  counted ONLY on seeded random frames i >= PRELUDE_LEN, so the deterministic prelude earns no coverage credit."""
  t = K9StopTaper()
  armed_ref, entries, changed, prev_out = False, 0, 0, 0.0
  for i, kw in enumerate(frames(seed, RANDOM_N)):
    before = t.phase
    out = t.update(**kw)
    accel, v = kw["accel"], kw["v_ego"]
    finite = all(math.isfinite(kw[k]) for k in FINITE_KEYS)
    if not finite or _interlocked(kw):
      assert _same(out, accel) and t.phase == TaperPhase.done and not t.armed, (seed, i)
      armed_ref = False
      prev_out = out
      continue
    lead_too_close = (kw["lead_visible"] or kw["lead_distance"] != 0.0) and kw["lead_distance"] <= kt.LEAD_MIN_GAP
    if v >= kt.REARM_SPEED and not kw["standstill"]:
      armed_ref = True  # also true for a finite 1e9 spike: open owner question Q1, not changed here
    if before == TaperPhase.idle and t.phase in (TaperPhase.taper, TaperPhase.restore):
      assert armed_ref, (seed, i)  # every episode needs a run at REARM_SPEED since the last end/disarm
      entries += i >= PRELUDE_LEN
    if out != accel:
      changed += i >= PRELUDE_LEN
      assert accel < out < 0.0, (seed, i)  # only weaker, never released to zero or positive
      assert accel > kt.STRONG_ACCEL and kw["a_target"] > kt.STRONG_ACCEL and not lead_too_close, (seed, i)
      if t.phase == TaperPhase.taper:
        assert 0.0 < v < kt.TAPER_SPEED and not kw["standstill"], (seed, i)
        ref = prev_out if before == TaperPhase.taper else accel
        assert out <= ref + kt.RELEASE_RATE * kt.DT + EPS, (seed, i)
        assert out <= max(accel, spec_floor(v, kw)) + EPS, (seed, i)
      else:
        assert t.phase == TaperPhase.restore and kw["standstill"] and abs(v) <= kt.MOVING_AFTER_STOP, (seed, i)
        assert prev_out - kt.REAPPLY_RATE * kt.DT - EPS <= out <= prev_out + EPS, (seed, i)
    else:
      assert t.phase in (TaperPhase.idle, TaperPhase.done) or t.out == accel, (seed, i)
    if t.phase == TaperPhase.done and before in (TaperPhase.taper, TaperPhase.restore):
      armed_ref = False
    prev_out = out
  return entries, changed


@pytest.mark.parametrize("seed", SEEDS)
def test_seeded_stream_frame_contract(seed):
  check_stream(seed)


def test_seeded_streams_reach_every_exit_reason():
  """Seeded stream WITH a deterministic coverage prelude: reasons are tallied per source (prelude vs seeded random)."""
  reasons, prelude_counts, random_counts = set(), {}, {}
  for seed in SEEDS:
    t = K9StopTaper()
    for i, kw in enumerate(frames(seed, RANDOM_N)):
      before = t.phase
      t.update(**kw)
      if before in (TaperPhase.taper, TaperPhase.restore) and t.phase == TaperPhase.done:
        reasons.add(t.reason)
        counts = prelude_counts if i < PRELUDE_LEN else random_counts
        counts[t.reason] = counts.get(t.reason, 0) + 1
  print("prelude", sorted(prelude_counts.items()), "random", sorted(random_counts.items()))
  assert prelude_counts.get("distance_budget", 0) == len(SEEDS) and prelude_counts.get("interlock", 0) == len(SEEDS)
  # required random-frame reasons (subset, not a freeze of observed outcomes: extra valid random reasons are allowed;
  # distance_budget/interlock/restored also have separate deterministic scenes below)
  assert set(random_counts) >= {"strong_braking", "strengthened", "lead_too_close", "invalid", "left_window",
                                "no_progress", "moving", "release"}
  stats = [check_stream(seed) for seed in SEEDS]
  print("random-only (entries, changed) per seed", stats)
  assert sum(e > 0 for e, _ in stats) >= 28 and sum(c > 0 for _, c in stats) >= 22  # random frames only, no prelude credit
  assert reasons == {"strong_braking", "strengthened", "lead_too_close", "interlock", "invalid", "left_window",
                     "distance_budget", "no_progress", "moving", "release", "restored"}


def _taper_base(**kw):
  return {"accel": -1.0, "a_target": -1.0, "a_ego": -0.5, "standstill": False, "long_active": True, "override": False,
          "gas": False, "brake": False, "can_valid": True, "stopping_or_pid": True, "lead_visible": False,
          "lead_distance": 0.0, "lead_rel_speed": 0.0} | kw


def _enter_taper(t, v=0.27):
  t.update(**_taper_base(v_ego=3.0))
  outs = [t.update(**_taper_base(v_ego=v)) for _ in range(10)]
  assert t.phase == TaperPhase.taper and outs[-1] > -1.0 and outs[-1] < 0.0  # real taper, modified output
  return outs[-1]


@pytest.mark.parametrize("reason", ["distance_budget", "strengthened", "interlock", "restored"])
def test_deterministic_exit_reason_scenes(reason):
  """Deterministic coverage, separate from the random streams above."""
  t = K9StopTaper()
  _enter_taper(t)
  if reason == "distance_budget":  # observed 0.27 m/s with decel reported: budget ends the episode (~0.5 s)
    for i in range(200):
      out = t.update(**_taper_base(v_ego=0.27))
      if t.phase == TaperPhase.done:
        break
    assert 0.3 <= i * kt.DT <= 1.0 and out == -1.0
  elif reason == "strengthened":
    assert t.update(**_taper_base(v_ego=0.25, accel=-1.4, a_target=-1.4)) == -1.4
  elif reason == "interlock":
    assert t.update(**_taper_base(v_ego=0.25, gas=True)) == -1.0
  else:
    v = 0.27
    while v > 0.0:
      v = max(v - 0.006, 0.0)
      t.update(**_taper_base(v_ego=v, standstill=v == 0.0))
    assert t.phase == TaperPhase.restore and t.out > -1.0
    for _ in range(300):
      out = t.update(**_taper_base(v_ego=0.0, standstill=True, a_ego=0.0))
      if t.phase == TaperPhase.done:
        break
    assert out == -1.0
  assert t.phase == TaperPhase.done and t.reason == reason


@pytest.mark.parametrize("seed", SEEDS[:8])
def test_reset_equals_fresh_object_and_streams_are_deterministic(seed):
  stream = list(frames(seed, 1500))
  cut = random.Random(seed).randint(200, 1300)
  a, b = K9StopTaper(), K9StopTaper()
  for kw in stream[:cut]:
    assert _same(a.update(**kw), b.update(**kw))  # deterministic, no hidden global state
  a.reset()
  fresh = K9StopTaper()
  assert vars(a) == vars(fresh)
  for kw in stream[cut:]:
    assert _same(a.update(**kw), fresh.update(**kw))


def test_q1_single_finite_speed_spike_rearms_after_abort_characterization():
  # Documents current behaviour behind open owner question Q1 (overnight finding). The module rejects only
  # non-finite values, so one finite 1e9 vEgo frame counts as a REARM_SPEED run and a new episode/budget can start.
  # If the owner chooses a plausibility/consecutive-frame rule this test must change with it; not changed here.
  t = K9StopTaper()
  base = {"a_target": -1.0, "a_ego": -0.5, "standstill": False, "long_active": True, "override": False, "gas": False,
          "brake": False, "can_valid": True, "stopping_or_pid": True, "lead_visible": False, "lead_distance": 0.0,
          "lead_rel_speed": 0.0}
  t.update(accel=-1.0, v_ego=3.0, **base)
  for _ in range(20):
    t.update(accel=-1.0, v_ego=0.2, **base)
  assert t.phase == TaperPhase.taper
  assert t.update(accel=-1.0, v_ego=0.2, **(base | {"a_target": -2.0})) == -1.0 and t.phase == TaperPhase.done
  assert t.update(accel=-1.0, v_ego=0.2, **base) == -1.0  # same stop: original
  t.update(accel=-1.0, v_ego=1e9, **(base | {"standstill": True}))  # with the standstill flag: no re-arm
  assert not t.armed and t.phase == TaperPhase.done
  t.update(accel=-1.0, v_ego=1e9, **base)  # finite outlier
  assert t.armed and t.phase == TaperPhase.idle
  assert t.update(accel=-1.0, v_ego=0.2, **base) > -1.0 and t.phase == TaperPhase.taper


# --- CarController: diverse seeded streams -------------------------------------------------------------------------

# CarController random frames. 1200 gave seed 21 zero random-frame output changes: all 60 of its in-window random frames
# came while the taper was done/disarmed (an interlock/invalid draw below REARM_SPEED), so `base != cand` was carried by
# the deterministic prelude only. 3600 continues the same frames()/cc_stream RNG sequences (first 1200 unchanged).
CC_RANDOM_N = 3600


def cc_stream(rig, seed):
  """CarController-level stream: longActive drops, gas/brake, override, lead disagreement, standstill flicker."""
  r = random.Random(f"k9-taper-props-{seed}")
  parser = CANParser("hyundai_kia_generic", [("SCC12", 0)], 0)
  rows = []
  for kw in frames(seed, CC_RANDOM_N):
    cc = structs.CarControl(enabled=True, longActive=kw["long_active"])
    accel = kw["accel"] if math.isfinite(kw["accel"]) else -1.0  # carControl accel comes from LongControl: finite
    cc.actuators.accel, cc.actuators.aTarget = accel, kw["a_target"] if math.isfinite(kw["a_target"]) else accel
    cc.actuators.longControlState = LCS.stopping if kw["v_ego"] < 1.5 and accel < 0 else LCS.pid
    if not kw["stopping_or_pid"]:
      cc.actuators.longControlState = r.choice([LCS.off, LCS.starting])
    cc.cruiseControl.override = kw["override"]
    lead_d = kw["lead_distance"] if math.isfinite(kw["lead_distance"]) else 0.0
    cc.hudControl.leadVisible, cc.hudControl.leadDistance = kw["lead_visible"], min(lead_d, 1e3)
    cc.hudControl.leadRelSpeed = kw["lead_rel_speed"] if math.isfinite(kw["lead_rel_speed"]) else 0.0
    cs = make_ci_state()
    v = kw["v_ego"] if math.isfinite(kw["v_ego"]) else 0.0
    cs.out = structs.CarState(vEgo=min(v, 1e3), vEgoRaw=min(v, 1e3), standstill=kw["standstill"],
                              aEgo=kw["a_ego"] if math.isfinite(kw["a_ego"]) else 0.0, canValid=kw["can_valid"],
                              gasPressed=kw["gas"], brakePressed=kw["brake"])
    _, sends = rig.cc.update(cc.as_reader(), cs, rig.frame * 10_000_000)
    rig.can.append(sorted((addr, bytes(dat), bus) for addr, dat, bus in sends))
    scc12 = [m for m in sends if m[0] == 1057]
    if scc12:
      parser.update([rig.frame * 10_000_000, scc12])
      rows.append((parser.vl["SCC12"]["StopReq"], parser.vl["SCC12"]["ACCMode"], parser.vl["SCC12"]["aReqValue"]))
    rig.frame += 1
  return rig.can, rows


@pytest.mark.parametrize("values", [{}, {"K9StopTaper": False}], ids=["missing", "false"])
@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("seed", [4, 21, 28])
def test_off_diverse_stream_can_identical_to_pre_candidate(monkeypatch, base_cc, values, fingerprint, seed):  # noqa: F811
  ref, _ = cc_stream(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), taper=False, module=base_cc), seed)
  monkeypatch.undo()
  new, _ = cc_stream(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), taper=False, **values), seed)
  assert len(ref) == len(new) == len(list(frames(seed, CC_RANDOM_N))) and ref == new


@pytest.mark.parametrize("seed", [4, 21, 28])
def test_on_diverse_stream_keeps_stopreq_and_never_strengthens(monkeypatch, base_cc, seed):  # noqa: F811
  ref7, _ = cc_stream(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), taper=False, module=base_cc), seed)
  monkeypatch.undo()
  assert cc_stream(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), K9CreepMode=3), seed)[0] == ref7  # other cars untouched
  monkeypatch.undo()
  _, base = cc_stream(Rig(monkeypatch, taper=False, module=base_cc), seed)
  monkeypatch.undo()
  old = list(frames(seed, 1200))
  assert list(frames(seed, CC_RANDOM_N))[:len(old)] == old  # extension keeps the original 1200 random frames
  rig, update, changed = Rig(monkeypatch, K9CreepMode=3), K9StopTaper.update, [0]

  def counted(self, **kw):  # random-frame output changes only; the prelude earns no credit
    out = update(self, **kw)
    changed[0] += rig.frame >= PRELUDE_LEN and out != kw["accel"]
    return out
  monkeypatch.setattr(K9StopTaper, "update", counted)
  _, cand = cc_stream(rig, seed)
  assert changed[0] > 0, seed
  assert len(base) == len(cand) and base != cand
  for (bs, bm, ba), (cs, cm, ca) in zip(base, cand, strict=True):
    assert (cs, cm) == (bs, bm) and ca >= ba - 0.011  # StopReq/ACCMode kept; aReq never stronger (2 LSB)


def _to_restore(t):
  _enter_taper(t)
  t.update(**_taper_base(v_ego=0.0, standstill=True))
  assert t.phase == TaperPhase.restore and -1.0 < t.out < 0.0  # real restore from a changed output


@pytest.mark.parametrize("reason", ["strong_braking", "lead_too_close", "invalid", "left_window", "no_progress",
                                    "moving", "release"])
def test_deterministic_other_exit_reason_scenes(reason):
  """The remaining 7 reasons, each after an actual entry with a changed output; separate from random credit."""
  t = K9StopTaper()
  if reason in ("moving", "release"):
    _to_restore(t)
    kw = {"moving": _taper_base(v_ego=0.2, standstill=False), "release": _taper_base(v_ego=0.0, standstill=True, accel=-0.1)}
    assert t.update(**kw[reason]) == kw[reason]["accel"]
  else:
    _enter_taper(t)
    if reason == "strong_braking":
      assert t.update(**_taper_base(v_ego=0.25, accel=kt.STRONG_ACCEL, a_target=kt.STRONG_ACCEL)) == kt.STRONG_ACCEL
    elif reason == "lead_too_close":
      assert t.update(**_taper_base(v_ego=0.25, lead_visible=True, lead_distance=kt.LEAD_MIN_GAP)) == -1.0
    elif reason == "invalid":
      assert t.update(**_taper_base(v_ego=0.25, a_ego=float("nan"))) == -1.0
    elif reason == "left_window":
      assert t.update(**_taper_base(v_ego=kt.TAPER_SPEED)) == -1.0
    else:
      for i in range(100):
        out = t.update(**_taper_base(v_ego=0.27, a_ego=0.0))
        if t.phase == TaperPhase.done:
          break
      assert out == -1.0 and abs((i + 1) * kt.DT - kt.PROGRESS_TIME) < 1e-9
  assert t.phase == TaperPhase.done and t.reason == reason
