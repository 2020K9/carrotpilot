"""K9StopMode 2 (kind 6, v10 + light shudder fix) stop floor LIGHT_MIN_DECEL (2026-10-24 20:26, the driver's order "떨림구간
브레이크 아예 안떼게 말고 감속신호 브레이크 제일 약할 수 있는 수치로"): before the latch the sent accel is at most -0.26 while
moving in the stopping state, and in pid below 3.0 m/s while the plan decelerates and the command brakes. Model-only checks with synthetic inputs, not vehicle behaviour (실차 미검증)."""
import math
import random

import pytest

from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V10, V10_LIGHT, V11
from openpilot.selfdrive.car.tests.k9_rig import LCS, Rig
from openpilot.selfdrive.car.tests.test_k9_creep_mode import _random_inputs
from openpilot.selfdrive.car.tests.test_k9_stop_v10 import _stops


def test_floor_constant_is_the_v10_feather_end():
  assert kc.LIGHT_MIN_DECEL == -0.26 == kc.V10_FEATHER_V[0]


def _upd(c, f):
  args = dict(accel=f["accel"], a_target=f["a_target"], stopping=f["stopping"], pid=f["pid"], v_ego=f["v_ego"],
              v_ego_raw=f["v_ego_raw"], a_ego=f["a_ego"], long_accel=f["long_accel"], esp_standstill=f["esp_standstill"],
              long_active=True, override=False, gas=False, brake=False, can_valid=True, lead_visible=False, lead_distance=0.0)
  return c.update(**args)


def _floor_zone(c, kw):
  """frames where kind 6 must not send above the floor (c = the kind 6 object after its update)"""
  if not c.active:
    return False
  if kw["stopping"]:
    return not c.latched
  return kw["pid"] and kw["v_ego"] < kc.LIGHT_OFF_SPEED and kw["a_target"] < kc.FADE_TARGET_BP[-1]


@pytest.mark.parametrize("seed", range(8))
def test_kind_6_never_weaker_than_menu_1_and_same_stopreq_on_stops(seed):
  for frames in _stops(seed):
    c1, c6 = K9Creep(V10), K9Creep(V10_LIGHT)
    for f in frames:
      (a1, s1), (a6, s6) = _upd(c1, f), _upd(c6, f)
      assert s6 == s1 and c6.latched == c1.latched
      assert a6 <= a1 + 1e-12
      if _floor_zone(c6, f) and (f["stopping"] or a1 < 0.0):
        assert a6 <= kc.LIGHT_MIN_DECEL


@pytest.mark.parametrize("seed", range(6))
def test_kind_6_floor_on_random_inputs(seed):
  rng = random.Random(f"k9-light-floor-{seed}")
  c1, c6 = K9Creep(V10), K9Creep(V10_LIGHT)
  for kw, i in _random_inputs(rng, 4000):
    (a1, s1), (a6, s6) = c1.update(**kw), c6.update(**kw)
    assert s1 == s6 and (c1.latched, c1.active) == (c6.latched, c6.active), i
    if math.isnan(a1):
      assert math.isnan(a6)
      continue
    assert a6 <= a1 + 1e-12, i
    if _floor_zone(c6, kw) and (kw["stopping"] or a1 < 0.0):
      assert a6 <= kc.LIGHT_MIN_DECEL, i


def test_kind_6_equals_menu_1_when_the_feather_never_goes_above_the_floor():
  for seed in range(8):
    for frames in _stops(seed):
      c1, c6, ok = K9Creep(V10), K9Creep(V10_LIGHT), True
      out1, out6 = [], []
      for f in frames:
        out1.append(_upd(c1, f)); out6.append(_upd(c6, f))
        if _floor_zone(c1, f) and out1[-1][0] > kc.LIGHT_MIN_DECEL and (f["stopping"] or out1[-1][0] < 0.0):
          ok = False
      if ok:
        assert out1 == out6


def test_weak_entry_is_held_at_the_floor_instead_of_released():
  c1, c6 = K9Creep(V10), K9Creep(V10_LIGHT)
  base = dict(a_target=-0.06, pid=True, stopping=False, v_ego=0.8, v_ego_raw=0.8, a_ego=-0.2, long_accel=-0.2, esp_standstill=False,
              long_active=True, override=False, gas=False, brake=False, can_valid=True, lead_visible=False, lead_distance=0.0)
  p1, p6 = c1.update(accel=-0.10, **base)[0], c6.update(accel=-0.10, **base)[0]  # coasting command before the stopping state
  assert p1 > -0.26 and p6 == kc.LIGHT_MIN_DECEL  # pid below 3 m/s with a decelerating plan: floored
  assert c6.update(accel=0.3, **base)[0] == K9Creep(V10).update(accel=0.3, **base)[0] > 0.0  # a go command is never floored
  assert K9Creep(V10_LIGHT).update(accel=-0.1, **dict(base, v_ego=3.0, v_ego_raw=3.0))[0] == -0.1  # 3.0 m/s and up: menu 1
  assert K9Creep(V10_LIGHT).update(accel=-0.1, **dict(base, a_target=-0.04))[0] == -0.1  # plan not decelerating
  c1, c6 = K9Creep(V10), K9Creep(V10_LIGHT)
  for c in (c1, c6):
    c.update(accel=-0.10, **dict(base, pid=False))  # (not pid: no floor) weak command before the stopping state
  stop = dict(base, stopping=True, pid=False, v_ego=0.5, v_ego_raw=0.5)
  a1 = [c1.update(accel=-0.10, **stop)[0] for _ in range(20)]
  a6 = [c6.update(accel=-0.10, **stop)[0] for _ in range(20)]
  assert max(a1) > -0.2  # menu 1 starts the feather from the weak command
  assert all(a <= kc.LIGHT_MIN_DECEL for a in a6)
  assert all(b <= a for a, b in zip(a1, a6))


def test_menu_3_and_menu_1_have_no_floor():
  c = K9Creep(V11)
  assert not c.v10_light and not K9Creep(V10).v10_light


@pytest.mark.parametrize("menu", [1, 2, 3])
def test_through_carcontroller_menu_2_holds_the_floor_menus_1_3_unchanged(monkeypatch, menu):
  r = Rig(monkeypatch, K9StopMode=menu)
  sent, pid_sent = [], []
  for i in range(100):
    _, s = r.step(-0.12, LCS.pid, 1.2 - 0.004 * i, a_target=-0.06, a_ego=-0.15)
    if s is not None and i > 2:
      pid_sent.append(s["aReqValue"])
  for i in range(120):
    _, s = r.step(-0.12, LCS.stopping, max(0.8 - 0.006 * i, 0.06), a_target=-0.06, a_ego=-0.15)
    if s is not None and r.cc.k9_creep is not None and not r.cc.k9_creep.latched:
      sent.append(s["aReqValue"])
  assert sent
  if menu == 2:
    assert max(sent) <= kc.LIGHT_MIN_DECEL + 0.006  # SCC12 0.01 grid
    assert max(pid_sent) <= kc.LIGHT_MIN_DECEL + 0.006
  else:
    assert max(sent) > -0.2  # unchanged: those menus still start from the weak command
