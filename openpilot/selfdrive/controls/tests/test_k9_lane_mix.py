"""K9LaneAssist 1: k9_lane_mix - one lane hold pushes at a time (보노보노 2026-10-25 23:38). Synthetic values only, NOT
vehicle-validated. Random stress tests run with 5 seeds."""
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import k9_lane_mix as M

ROOT = Path(__file__).resolve().parents[3]
DT = M.DT
SEEDS = [0, 1, 2, 3, 4]


def _limited_walk(rng, n, v, a_cap, j_cap, k_cap=None):
  """a jerk-limited curvature sequence inside the given caps (what one hold alone emits)."""
  v2 = max(v, M.V_MIN) ** 2
  amax = a_cap / v2 if k_cap is None else min(a_cap / v2, k_cap)
  dk = (j_cap / v2 if k_cap is None else min(j_cap / v2, M.KR_MAX)) * DT
  k = 0.0; out = []
  tgt = 0.0
  for i in range(n):
    if i % 150 == 0:
      tgt = float(rng.uniform(-1.2, 1.2)) * amax
    k = float(np.clip(np.clip(tgt, -amax, amax), k - dk, k + dk))
    out.append(k)
  return out


def test_both_zero_is_zero_and_no_state():
  c = type("C", (), {})()
  for _ in range(100):
    assert M.k9_lane_mix_step(c, 0.0, 0.0, 20.0) == 0.0
  assert getattr(c, "k9_mix", None) is None


@pytest.mark.parametrize("seed", SEEDS)
def test_centre_hold_alone_is_bit_identical(seed):
  rng = np.random.default_rng(seed)
  for v in (0.5, 2.0, 8.0, 20.0, 30.0):
    ks = _limited_walk(rng, 1500, v, 0.20, 0.15, k_cap=M.K_MAX)
    mix = M.LaneMix()
    got = [mix.update(k, 0.0, True, False, False, v) for k in ks]
    assert np.asarray(got).tobytes() == np.asarray(ks).tobytes()


@pytest.mark.parametrize("seed", SEEDS)
def test_one_line_hold_alone_is_bit_identical(seed):
  rng = np.random.default_rng(seed)
  for v in (8.0, 15.0, 30.0):
    for close in (False, True):
      ks = _limited_walk(rng, 1500, v, 0.6 if close else 0.25, 0.15)
      mix = M.LaneMix()
      got = [mix.update(0.0, k, False, True, close, v) for k in ks]
      assert np.asarray(got).tobytes() == np.asarray(ks).tobytes()


def _check(out, v, caps):
  out = np.asarray(out); v2 = max(v, M.V_MIN) ** 2
  assert np.all(np.abs(out) * v2 <= np.asarray(caps) + 1e-9)
  jerk = np.abs(np.diff(np.concatenate([[0.0], out]))) * v2 / DT
  assert jerk.max() <= M.J_MAX + 1e-9
  return out


def test_both_to_one_line_overlap_never_adds_up():
  # centre hold ramping out at +0.20 m/s^2 while the one-line hold ramps in at +0.25 (same way) and at -0.25 (opposite)
  v = 20.0; v2 = v * v
  for sign in (1.0, -1.0):
    mix = M.LaneMix(); out = []
    k_lc = 0.20 / v2
    for i in range(600):
      lc_active = i < 100
      if not lc_active:
        k_lc = max(k_lc - 0.15 / v2 * DT, 0.0)            # its own 1 s ramp-out at the jerk limit
      olh_active = 160 <= i < 500                          # 0.6 s watch, then on
      k_olh = sign * min(max(i - 160, 0) * 0.15 / v2 * DT, 0.25 / v2) if olh_active else 0.0
      out.append(mix.update(k_lc, k_olh, lc_active, olh_active, False, v))
      if olh_active:
        assert mix.owner == M.OWNER_OLH
    o = _check(out, v, [0.25] * len(out))
    old = np.array([max(0.20 / v2 - max(i - 100, 0) * 0.15 / v2 * DT, 0.0) if i >= 0 else 0 for i in range(600)])
    # the old plain sum would have pushed both at once; the mix follows one owner and moves at the jerk limit
    assert np.max(np.abs(o)) * v2 <= 0.25 + 1e-9
    if sign < 0:
      # opposite pushes: the output goes through zero once (a hand-over), never back and forth
      s = np.sign(o[np.abs(o) > 1e-12])
      assert np.count_nonzero(np.diff(s)) <= 1
    assert old.max() > 0


def test_switch_off_while_pushing_ramps_out():
  v = 15.0; v2 = v * v
  c = type("C", (), {})()
  c.k9_lc = type("L", (), {"active": True})(); c.k9_olh = None
  k = 0.0
  for _ in range(200):
    k = min(k + 0.15 / v2 * DT, 0.2 / v2)
    M.k9_lane_mix_step(c, k, 0.0, v)
  c.k9_lc = None
  out = [M.k9_lane_mix_step(c, 0.0, 0.0, v) for _ in range(400)]
  assert out[0] > 0.0 and out[-1] == 0.0
  assert np.max(np.abs(np.diff(out))) * v2 / DT <= M.J_MAX + 1e-9
  assert getattr(c, "k9_mix", None) is None


@pytest.mark.parametrize("seed", SEEDS)
def test_random_stress_caps_hold(seed):
  rng = np.random.default_rng(100 + seed)
  for _ in range(20):
    v = float(rng.uniform(0.3, 35.0)); v2 = max(v, M.V_MIN) ** 2
    mix = M.LaneMix(); out = []; caps = []
    for i in range(800):
      lc_a, olh_a, close = bool(rng.random() < 0.5), bool(rng.random() < 0.4), bool(rng.random() < 0.3)
      k_lc = float(rng.normal(0, 3e-3)) if rng.random() < 0.8 else 0.0      # far over the caps on purpose
      k_olh = float(rng.normal(0, 3e-3)) if rng.random() < 0.8 else 0.0
      out.append(mix.update(k_lc, k_olh, lc_a, olh_a, close, v))
      owner_olh = mix.owner == M.OWNER_OLH
      cap = (0.6 if (owner_olh and close) else 0.25) if owner_olh else min(0.25, M.K_MAX * v2)
      caps.append(cap)
    out = np.asarray(out)
    assert np.all(np.abs(out) * v2 <= np.asarray(caps) + 1e-9)
    dk = min(M.J_MAX / v2, M.KR_MAX) * DT
    assert np.max(np.abs(np.diff(np.concatenate([[0.0], out])))) <= dk + 1e-15


def test_controlsd_adds_only_the_mix():
  src = (ROOT / "selfdrive/controls/controlsd.py").read_text()
  assert "k9_lane_mix_step(self, k9_lc_add, k9_olh_add, CS.vEgo)" in src
  assert "dc_model + k9_lane_add" in src
  assert "dc_model + k9_olh_add" not in src and "dc_model + k9_lc_add" not in src


def test_same_path_offset_for_both_holds():
  # both holds take the target offset from the one reader (PathOffset, +-30 cm)
  lc = (ROOT / "selfdrive/controls/lib/k9_lane_center.py").read_text()
  olh = (ROOT / "selfdrive/controls/lib/k9_one_lane_hold.py").read_text()
  assert "k9_lane_offset_m(controls.params)" in lc and "k9_lane_offset_m(controls.params)" in olh
