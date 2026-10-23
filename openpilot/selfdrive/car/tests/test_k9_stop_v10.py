"""K9StopMode 1 (since the 2026-10-23 07:25 menu; 2 on 1d0961da) = the K9Creep kind V10 = creep_fix v10: bit-identical to a literal reference port of the page
code ("구형 CAN 롱컨 통합 패치 v10", https://leegrt.org/oppo/creep_fix/). The reference below is the page's carcontroller
block transcribed line by line (CS/actuators replaced by plain arguments, getattr defaults kept). Our safety additions
(lead latch 3.0 m, original command on disengage/override/pedal/invalid CAN) are not in the page and are checked
separately. Sim/replay only; NOT verified on the vehicle."""
import random

import numpy as np
import pytest

from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V9, V10
from opendbc.car.hyundai.values import CarControllerParams


class V10Ref:
  """Literal port of the creep_fix v10 page code (carcontroller.py [변경 1])."""
  def step(self, accel, stopping, a_target, pid, v_ego, v_ego_raw, a_ego, long_accel, esp_standstill):
    if stopping:
      real_standstill = bool(esp_standstill or v_ego_raw < 0.05 or v_ego < 0.05)
      lon_ac = float(long_accel) if long_accel is not None else float(a_ego)
      eff_ac = float(min(lon_ac, a_ego + 0.10))
      if real_standstill or a_target <= -1.0 or (v_ego > 0.14 and eff_ac > 0.04) or a_ego > 0.06:
        self.stop_req_latched = True
      if not getattr(self, "stop_req_latched", False):
        stopping = False
        stop_feather = float(np.interp(v_ego, [0.05, 0.11, 0.16, 0.25, 0.42], [-0.26, -0.28, -0.34, -0.42, -0.52]))
        min_decel = float(np.interp(v_ego, [0.06, 0.12, 0.25, 0.40], [-0.02, -0.09, -0.20, -0.30]))
        a_boost = float(np.interp(eff_ac - min_decel, [0.0, 0.15], [0.0, -0.18]))
        target_accel = float(stop_feather + a_boost)
        prev_accel = float(getattr(self, "accel_feather_last", accel))
        down_step = 0.005 if (v_ego > 0.14 and eff_ac > -0.02) else 0.0
        up_step = float(np.interp(eff_ac - min_decel, [-0.08, 0.02], [0.0032, 0.0])) if v_ego > 0.12 else 0.0032
        accel = float(np.clip(target_accel, prev_accel - down_step, prev_accel + up_step))
        self.accel_feather_last = float(accel)
      else:
        prev_accel = float(getattr(self, "accel_feather_last", accel))
        accel = float(max(min(accel, -0.50), prev_accel - 0.0035)) if a_target > -1.0 else float(min(accel, -0.50))
        self.accel_feather_last = float(accel)
    else:
      self.stop_req_latched = False
      if pid and v_ego < 1.6 and a_target < -0.05:
        creep_comp = float(np.interp(v_ego, [0.25, 0.55, 1.0, 1.6], [-0.26, -0.26, -0.20, 0.0]))
        fade_by_target = float(np.interp(a_target, [-0.20, -0.05], [1.0, 0.0]))
        fade_by_accel = float(np.interp(accel, [-0.80, -0.30], [0.25, 1.0]))
        accel = float(np.clip(accel + creep_comp * fade_by_target * fade_by_accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      self.accel_feather_last = float(accel)
    return accel, stopping


def _stops(seed, n=12):
  """random approach-and-stop sequences: pid decel -> stopping -> standstill (with urgent targets, creep wins, noise)."""
  rng = random.Random(seed)
  for _ in range(n):
    v = rng.uniform(0.3, 3.0); a_ego = -0.3; frames = []
    stop_at = rng.randint(80, 400)
    for i in range(rng.randint(300, 700)):
      stopping = i >= stop_at
      a_target = rng.choice([rng.uniform(-0.6, 0.0), rng.uniform(-1.4, -0.9)]) if rng.random() < 0.03 else rng.uniform(-0.5, -0.02)
      accel = float(np.clip(rng.uniform(-1.2, 0.2), CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      la = None if rng.random() < 0.05 else a_ego + rng.gauss(0.0, 0.08)
      a_ego = float(np.clip(a_ego + rng.gauss(0.0, 0.04), -1.5, 0.3))
      v = max(0.0, v + a_ego * 0.01 + (0.002 if rng.random() < 0.02 else 0.0))
      frames.append(dict(accel=accel, stopping=stopping, a_target=a_target, pid=not stopping, v_ego=v,
                         v_ego_raw=max(0.0, v + rng.gauss(0.0, 0.01)), a_ego=a_ego, long_accel=la,
                         esp_standstill=v <= 0.0 and rng.random() < 0.5))
    yield frames


@pytest.mark.parametrize("seed", range(8))
def test_mode2_is_bit_identical_to_the_v10_page_code(seed):
  for frames in _stops(seed):
    ref, c = V10Ref(), K9Creep(V10)
    for f in frames:
      want = ref.step(**f)
      got = c.update(accel=f["accel"], a_target=f["a_target"], stopping=f["stopping"], pid=f["pid"], v_ego=f["v_ego"],
                     v_ego_raw=f["v_ego_raw"], a_ego=f["a_ego"], long_accel=f["long_accel"], esp_standstill=f["esp_standstill"],
                     long_active=True, override=False, gas=False, brake=False, can_valid=True, lead_visible=False, lead_distance=0.0)
      assert got == want  # exact float equality, accel and StopReq


def test_v10_constants_are_the_page_values():
  assert kc.MODES == (1, 2, 3, 4) and V10 == 2  # menu 4 added 2026-10-24 (pin rewritten)
  assert (kc.V10_FEATHER_BP, kc.V10_FEATHER_V) == ([0.05, 0.11, 0.16, 0.25, 0.42], [-0.26, -0.28, -0.34, -0.42, -0.52])
  assert (kc.V10_MIN_DECEL_BP, kc.V10_MIN_DECEL_V) == ([0.06, 0.12, 0.25, 0.40], [-0.02, -0.09, -0.20, -0.30])
  assert kc.V10_UP_LOW_SPEED == 0.12 and kc.V10_HOLD_STEP == 0.0035 and kc.LATCH_ACCEL == -0.50
  c = K9Creep(V10)
  assert c.v10 and not c.shudder_fix and not K9Creep(V9).v10


def test_v10_hold_ramp_and_urgent():
  c = K9Creep(V10)
  kw = dict(stopping=True, pid=False, v_ego=0.3, v_ego_raw=0.3, a_ego=-0.3, long_accel=-0.3, esp_standstill=False, long_active=True,
            override=False, gas=False, brake=False, can_valid=True, lead_visible=False, lead_distance=0.0)
  a, sr = c.update(accel=-0.3, a_target=-0.3, **kw)
  assert not sr
  a0 = a
  kw.update(v_ego=0.0, v_ego_raw=0.0)
  a1, sr = c.update(accel=-0.3, a_target=-0.3, **kw)
  assert sr and a1 == pytest.approx(max(-0.50, a0 - 0.0035))  # soft press, no step to -0.50
  a2, _ = c.update(accel=-0.3, a_target=-1.2, **kw)
  assert a2 == -0.50  # urgent: at once


def test_v10_keeps_our_safety_additions():
  base = dict(accel=-0.3, a_target=-0.3, stopping=True, pid=False, v_ego=0.6, v_ego_raw=0.6, a_ego=-0.3, long_accel=-0.3,
              esp_standstill=False, long_active=True, override=False, gas=False, brake=False, can_valid=True, lead_visible=True)
  a, sr = K9Creep(V10).update(lead_distance=2.9, **base)  # lead within 3.0 m: latch like mode 1
  a1, sr1 = K9Creep(V9).update(lead_distance=2.9, **base)
  assert sr and sr1 and a1 == -0.50 and a == max(-0.50, -0.3 - 0.0035)  # StopReq at once; v10 soft hold ramp (page) also here
  for k in ("override", "gas", "brake"):
    assert K9Creep(V10).update(lead_distance=10.0, **dict(base, **{k: True})) == (-0.3, True)
  assert K9Creep(V10).update(lead_distance=10.0, **dict(base, long_active=False)) == (-0.3, True)
  assert K9Creep(V10).update(lead_distance=10.0, **dict(base, can_valid=False)) == (-0.3, True)


def test_v10_jerk_and_band_are_v9():
  for v in (0.0, 0.3, 1.0, 1.39, 1.4, 3.0):
    assert K9Creep(V10).bands(0.5, v, 0.9, 0.8, 0.9) == K9Creep(V9).bands(0.5, v, 0.9, 0.8, 0.9)
