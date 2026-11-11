"""brake_v12k9_fix (2026-10-25 03:40): K9StopMode 2 (v12k9) after f9cd0960.
In the K9 closed-loop sim (1440 stops x conditions) the pushed v12k9 sent more weaker-than-v9 frames than the old v11
(46367 vs 43370). Frame breakdown: the page v12 release limit (0.0035..0.0008 per frame above 0.52 m/s) kept braking while
the car already braked harder than planned, the car got slow early and the stop phase then braked lighter; the v12 fall limit
(0.004 per frame) also acted outside the old v11 re-press guard region (0.95-1.0 m/s, eff_ac >= -0.18), where v11 followed at
once. Fix, inside kind 8 only: the fall limit only where the v11 guard held (so never later than v11), the release limit only
while eff_ac >= aTarget - V12K9_UP_LIM_MARGIN. Menus 0 / 1 / other values: CAN identical to f9cd0960.
Model-only harness with synthetic inputs (실차 미검증).
"""
import math
import random

import numpy as np
import pytest

from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V12K9
from opendbc.car.hyundai.values import CAR
from openpilot.selfdrive.car.tests.k9_rig import Rig, edge_sequence, gentle_sequence, load_frozen_cc, make_cp, mixed_sequence
from openpilot.selfdrive.car.tests.test_k9_stop_v12 import PREV, PREV_V11, _pid, _random_inputs

PUSHED = "f9cd0960"  # carrot-wip before this fix: 0 base, 1 v12 original, 2 v12k9 (kind 8)


@pytest.fixture(scope="module")
def prev_cc(tmp_path_factory):
  return load_frozen_cc(PREV, tmp_path_factory.mktemp("fix_b6696b37"), "fix_b6696b37")


@pytest.fixture(scope="module")
def pushed_cc(tmp_path_factory):
  return load_frozen_cc(PUSHED, tmp_path_factory.mktemp("fix_f9cd0960"), "fix_f9cd0960")


def test_fix_constants_are_the_old_v11_guard(prev_cc):
  g = prev_cc.K9Creep._update_v11.__globals__
  assert kc.V12K9_FALL_GUARD_SPEED == g["V11_GUARD_SPEED"] == 0.95
  assert kc.V12K9_FALL_GUARD_EFF_AC == g["V11_GUARD_EFF_AC"] == -0.18
  assert kc.V12_RATE_MIN_TARGET == g["V11_GUARD_MIN_TARGET"] == -0.80
  assert kc.V12K9_UP_LIM_MARGIN == 0.02
  # unchanged v12 parts
  assert (kc.V12_DOWN_LIM, kc.V12_UP_LIM_BP, kc.V12_UP_LIM_V, kc.V12_UP_LIM_SPEED) == (0.004, [-0.05, 0.10], [0.0035, 0.0008], 0.52)


def _in_zone(kw):
  """2026-10-26 (the driver's approval 08:54): the zone where menu 2 may move StopReq and change the decel against v11 -
  stopping input below kc.V12K9_ZONE_SPEED with no lead within kc.V12K9_ZONE_LEAD."""
  lead_in = (kw["lead_visible"] or kw["lead_distance"] != 0.0) and kw["lead_distance"] <= kc.V12K9_ZONE_LEAD
  return bool(kw["stopping"] and kw["v_ego"] < kc.V12K9_ZONE_SPEED and not lead_in)


@pytest.mark.parametrize("seed", range(8))
def test_never_brakes_less_than_v11_for_the_same_inputs(prev_cc, seed):
  """Open loop, same inputs every frame: v12k9 sends a command at or below (as much or more braking than) the old v11 and
  the same StopReq on every frame outside the zone (_in_zone, 2026-10-26)."""
  old, new = prev_cc.K9Creep(PREV_V11), K9Creep(V12K9)
  rng = random.Random(f"k9-v12k9-fix-{seed}")
  for kw, i in _random_inputs(rng, 4000):
    ro, rn = old.update(**kw), new.update(**kw)
    if _in_zone(kw):
      continue
    assert ro[1] == rn[1], (i, kw)
    if math.isnan(ro[0]) or math.isnan(rn[0]):
      assert math.isnan(ro[0]) and math.isnan(rn[0]), (i, kw)
      continue
    assert rn[0] <= ro[0] + 1e-12, (i, kw, ro, rn)
    assert (old.latched, old.active) == (new.latched, new.active), i


def test_v12_parts_still_act(prev_cc):
  """... and the v12 parts still brake more than v11 somewhere on the same inputs."""
  more = 0
  for seed in range(8):
    old, new = prev_cc.K9Creep(PREV_V11), K9Creep(V12K9)
    rng = random.Random(f"k9-v12k9-fix-{seed}")
    for kw, _ in _random_inputs(rng, 4000):
      ro, rn = old.update(**kw), new.update(**kw)
      more += (not math.isnan(ro[0])) and rn[0] < ro[0] - 1e-9
  assert more > 0


def test_pushed_v12k9_braked_less_than_v11_somewhere(prev_cc, pushed_cc):
  """The f9cd0960 v12k9 (before the fix) does send weaker commands than v11 on these inputs (what this fix removes)."""
  weaker = 0
  for seed in range(8):
    old, pushed = prev_cc.K9Creep(PREV_V11), pushed_cc.K9Creep(V12K9)
    rng = random.Random(f"k9-v12k9-fix-{seed}")
    for kw, _ in _random_inputs(rng, 4000):
      ro, rp = old.update(**kw), pushed.update(**kw)
      weaker += (not math.isnan(ro[0])) and rp[0] > ro[0] + 0.005
  assert weaker > 0


def test_fall_limit_only_in_the_v11_guard_region():
  c = K9Creep(V12K9)
  # inside (below 0.95 m/s, already decelerating, eff_ac < -0.18): at most 0.004 per frame, as before
  assert _pid(c, -0.30, -0.60, 0.9, -0.5, -0.30) == pytest.approx(-0.304, abs=1e-9)
  # already decelerating hard enough: no fall at all, as before
  assert _pid(c, -0.30, -0.60, 0.9, -0.3, -0.50) == pytest.approx(-0.30, abs=1e-9)
  # not decelerating yet (eff_ac -0.10 >= -0.18): follows at once (v11 did; f9cd0960 limited it to 0.004 per frame)
  assert _pid(c, -0.30, -0.60, 0.9, -0.5, -0.10) <= -0.59
  # 0.95 .. 1.0 m/s: v11 had no guard there, the fall is not limited any more
  assert _pid(c, -0.30, -0.60, 0.97, -0.5, -0.30) <= -0.59
  # lead within 3 m: not limited (as before)
  assert _pid(c, -0.30, -0.60, 0.9, -0.5, -0.30, lead=(True, 2.0)) <= -0.59


def test_release_limit_only_while_not_braking_harder_than_planned(prev_cc):
  c = K9Creep(V12K9)
  # car brakes less than planned (eff_ac -0.25 >= aTarget -0.30 - 0.02): the page release limit, as before
  up = float(np.interp(-0.25 - -0.30, kc.V12_UP_LIM_BP, kc.V12_UP_LIM_V))
  assert _pid(c, -0.50, 0.0, 0.9, -0.30, -0.25) == pytest.approx(-0.50 + up, abs=1e-9)
  # inside the margin (eff_ac -0.31, aTarget -0.30): still limited
  up = float(np.interp(-0.31 - -0.30, kc.V12_UP_LIM_BP, kc.V12_UP_LIM_V))
  assert _pid(c, -0.50, 0.0, 0.9, -0.30, -0.31) == pytest.approx(-0.50 + up, abs=1e-9)
  # car already brakes harder than planned (eff_ac -0.35 < -0.32): released like v11 (f9cd0960 held it)
  old = prev_cc.K9Creep(PREV_V11)
  got = _pid(c, -0.50, 0.0, 0.9, -0.30, -0.35)
  assert got > -0.49 and got == _pid(old, -0.50, 0.0, 0.9, -0.30, -0.35)
  # at or below 0.52 m/s: never (as before)
  assert _pid(c, -0.50, 0.0, 0.4, -0.30, -0.25) > -0.49


def _kw(mode):
  return {} if mode is None else {"K9StopMode": mode}


@pytest.mark.parametrize("fingerprint", [CAR.KIA_K9, CAR.KIA_K7])
@pytest.mark.parametrize("mode", [None, 0, 6, 99, -1])  # 3 -> 4 on 2026-10-26: 3 = v13 original (new), 4 acts as 0; 2026-10-28:
# 3 = v14 original, 4 still acts as 0; 2026-10-29 04:48: 1 and 2 swapped methods, pinned across numbers below;
# 2026-10-29 07:10: the v14 original left the menu and kind 15 moved 4 -> 3, so 4 acts as 0 again and is back in this list
# 2026-10-30: 4 (kind 14) and 5 (kind 17) are menu values again, so 6 stands for "acts as 0"
def test_menus_other_than_the_k9_menu_can_identical_to_f9cd0960(monkeypatch, pushed_cc, mode, fingerprint):
  for seq in (mixed_sequence, edge_sequence, gentle_sequence):
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), module=pushed_cc, **_kw(mode)))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=fingerprint), **_kw(mode)))
    monkeypatch.undo()
    assert len(new.can) > 400 and ref.can == new.can and ref.scc14 == new.scc14


def test_menu1_other_cars_can_identical_to_f9cd0960(monkeypatch, pushed_cc):
  for seq in (mixed_sequence, edge_sequence, gentle_sequence):
    ref = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), module=pushed_cc, K9StopMode=2))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, cp=make_cp(fingerprint=CAR.KIA_K7), K9StopMode=1))
    monkeypatch.undo()
    assert ref.can == new.can
