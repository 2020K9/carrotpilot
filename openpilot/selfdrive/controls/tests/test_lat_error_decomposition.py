# Written for the lanebias/corner v3 review; NOT executed (test runs await approval).
# All values are synthetic; nothing here is a reference result.
import math

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib.lat_error_decomposition import (
  OK, UNEVALUABLE, center_frame_at, decompose, reconstruct_from_curvature, safe_ratio)

S = np.linspace(0.0, 100.0, 101)
STRAIGHT = (S, np.zeros_like(S))


def arc(k, n=401, length=100.0):
  s = np.linspace(0.0, length, n)
  if k == 0.0:
    return s, np.zeros_like(s)
  return np.sin(k * s) / k, (1.0 - np.cos(k * s)) / k


def test_sum_is_preserved_at_same_progress():
  r = decompose(*STRAIGHT, 30.0, (30.0, 0.4), (30.0, 0.1), 0.5)
  assert r["status"] == OK
  assert r["path"] + r["tracking"] == pytest.approx(r["total"])
  assert r["total"] == pytest.approx(0.4) and r["path"] == pytest.approx(0.1)


@pytest.mark.parametrize("k", [0.0, 0.005, -0.005])
def test_straight_and_left_right_curves(k):
  cx, cy = arc(k)
  frame = center_frame_at(cx, cy, 50.0)
  c, _, nrm = frame
  r = decompose(cx, cy, 50.0, c + 0.3 * nrm, c + 0.2 * nrm, 0.5)
  assert r["status"] == OK
  assert r["total"] == pytest.approx(0.3, abs=1e-6)
  assert r["path"] == pytest.approx(0.2, abs=1e-6)
  assert r["tracking"] == pytest.approx(0.1, abs=1e-6)


def test_frame_change_is_consistent():
  # rotate + translate every input together: offsets must not change
  th, tx, ty = 0.7, 12.0, -3.0
  rot = lambda x, y: (np.cos(th) * x - np.sin(th) * y + tx, np.sin(th) * x + np.cos(th) * y + ty)  # noqa: E731
  base = decompose(*STRAIGHT, 40.0, (40.0, 0.5), (40.0, 0.2), 0.5)
  cx, cy = rot(*STRAIGHT)
  e, p = rot(40.0, 0.5), rot(40.0, 0.2)
  moved = decompose(cx, cy, 40.0, e, p, 0.5)
  for key in ("total", "path", "tracking"):
    assert moved[key] == pytest.approx(base[key])


def test_sign_flip():
  a = decompose(*STRAIGHT, 40.0, (40.0, 0.5), (40.0, 0.2), 0.5)
  b = decompose(*STRAIGHT, 40.0, (40.0, -0.5), (40.0, -0.2), 0.5)
  for key in ("total", "path", "tracking"):
    assert b[key] == pytest.approx(-a[key])


def test_longitudinal_mismatch_is_unevaluable_not_rematched():
  r = decompose(*STRAIGHT, 40.0, (40.0, 0.5), (43.0, 0.2), 0.5)
  assert r["status"] == UNEVALUABLE and r["target_tangential"] == pytest.approx(3.0)
  assert r["path"] is None  # no separate nearest point is chosen for P


def test_no_tolerance_configured_is_unevaluable():
  assert decompose(*STRAIGHT, 40.0, (40.0, 0.5), (40.0, 0.2), None)["status"] == UNEVALUABLE


@pytest.mark.parametrize("bad", [(float("nan"), 0.0), (40.0,), ()])
def test_missing_or_nonfinite_is_unevaluable(bad):
  assert decompose(*STRAIGHT, 40.0, bad, (40.0, 0.2), 0.5)["status"] == UNEVALUABLE


def test_center_outside_range_is_not_extrapolated():
  assert center_frame_at(*STRAIGHT, 150.0) is None
  assert decompose(*STRAIGHT, 150.0, (150.0, 0.0), (150.0, 0.0), 0.5)["status"] == UNEVALUABLE


@pytest.mark.parametrize("t", [[0.0, 0.1, 0.1, 0.2], [0.0, 0.2, 0.1, 0.3], [0.0, float("nan"), 0.2, 0.3]])
def test_duplicate_reversed_nonfinite_time_rejected(t):
  assert reconstruct_from_curvature(t, [0.0] * 4, [20.0] * 4, 0.0, 0.0, 0.0) is None


def test_reconstruction_of_constant_curvature_matches_arc():
  k, v = 0.01, 20.0
  t = np.linspace(0.0, 2.0, 2001)
  x, y, psi = reconstruct_from_curvature(t, np.full_like(t, k), np.full_like(t, v), 0.0, 0.0, 0.0)
  s = v * t[-1]
  assert psi[-1] == pytest.approx(k * s, rel=1e-6)
  assert x[-1] == pytest.approx(math.sin(k * s) / k, rel=1e-4)
  assert y[-1] == pytest.approx((1 - math.cos(k * s)) / k, rel=1e-3)


def test_known_time_delay_shows_up_as_tracking_not_hidden():
  # E follows P's curvature history 0.3 s late: at the same t the residual is non-zero.
  v = 20.0
  t = np.linspace(0.0, 3.0, 301)
  k = np.where(t > 1.0, 0.005, 0.0)
  k_late = np.where(t > 1.3, 0.005, 0.0)
  px, py, _ = reconstruct_from_curvature(t, k, np.full_like(t, v), 0.0, 0.0, 0.0)
  ex, ey, _ = reconstruct_from_curvature(t, k_late, np.full_like(t, v), 0.0, 0.0, 0.0)
  r = decompose(*STRAIGHT, float(v * t[-1]), (ex[-1], ey[-1]), (px[-1], py[-1]), 1.0)
  assert r["status"] == OK
  assert abs(r["tracking"]) > 0.01
  assert r["path"] + r["tracking"] == pytest.approx(r["total"])


def test_same_plan_different_consumed_curvature_changes_path_share():
  v, t = 20.0, np.linspace(0.0, 2.0, 201)
  p1 = reconstruct_from_curvature(t, np.zeros_like(t), np.full_like(t, v), 0.0, 0.0, 0.0)
  p2 = reconstruct_from_curvature(t, np.full_like(t, 0.001), np.full_like(t, v), 0.0, 0.0, 0.0)
  s = float(v * t[-1])
  ego = (s, 0.3)
  r1 = decompose(*STRAIGHT, s, ego, (p1[0][-1], p1[1][-1]), 1.0)
  r2 = decompose(*STRAIGHT, s, ego, (p2[0][-1], p2[1][-1]), 1.0)
  assert r1["path"] != r2["path"] and r1["total"] == r2["total"]


def test_different_plan_same_consumed_curvature_same_path_share():
  # The plan is not an input at all: only the consumed curvature history defines P.
  v, t = 20.0, np.linspace(0.0, 2.0, 201)
  p = reconstruct_from_curvature(t, np.full_like(t, 0.001), np.full_like(t, v), 0.0, 0.0, 0.0)
  s = float(v * t[-1])
  a = decompose(*STRAIGHT, s, (s, 0.3), (p[0][-1], p[1][-1]), 1.0)
  b = decompose(*STRAIGHT, s, (s, 0.3), (p[0][-1], p[1][-1]), 1.0)
  assert a == b


def test_command_limit_seen_as_consumed_difference():
  # A clipped command is what gets integrated, so the limit appears in P, not as tracking.
  v, t = 20.0, np.linspace(0.0, 2.0, 201)
  wanted = np.full_like(t, 0.004)
  clipped = np.minimum(wanted, 0.002)
  pw = reconstruct_from_curvature(t, wanted, np.full_like(t, v), 0.0, 0.0, 0.0)
  pc = reconstruct_from_curvature(t, clipped, np.full_like(t, v), 0.0, 0.0, 0.0)
  assert pw[1][-1] != pc[1][-1]


def test_vehicle_model_error_lands_in_tracking_residual():
  # E simulated with a 10 % wrong curvature gain: the split books it as tracking,
  # which is why tracking must not be attributed to controller gains alone.
  v, t = 20.0, np.linspace(0.0, 2.0, 201)
  k = np.full_like(t, 0.002)
  p = reconstruct_from_curvature(t, k, np.full_like(t, v), 0.0, 0.0, 0.0)
  e = reconstruct_from_curvature(t, 0.9 * k, np.full_like(t, v), 0.0, 0.0, 0.0)
  cx, cy = arc(0.002)
  s = float(v * t[-1])
  r = decompose(cx, cy, s, (e[0][-1], e[1][-1]), (p[0][-1], p[1][-1]), 1.0)
  assert r["status"] == OK and abs(r["path"]) < 1e-3 and abs(r["tracking"]) > 0.01


def test_ratio_near_zero_target_is_not_filled():
  assert safe_ratio(0.1, 1e-6, 1e-3) is None
  assert safe_ratio(0.1, 0.2, None) is None
  assert safe_ratio(0.1, 0.2, 1e-3) == pytest.approx(0.5)
  assert safe_ratio(float("nan"), 0.2, 1e-3) is None


# ------------------------------------------------------------ v4: tolerance / ratio numeric validity
# Written for the v4 review; NOT executed.  Values are UNAPPROVED DEFAULTS -- SYNTHETIC TEST ONLY.

@pytest.mark.parametrize("tol", [float("nan"), float("inf"), float("-inf"), -0.1, "0.5", True, [0.5]])
def test_invalid_tangential_tolerance_is_unevaluable_with_reason(tol):
  # 3 m longitudinal mismatch: an infinite/NaN tolerance must not let it through.
  r = decompose(*STRAIGHT, 40.0, (40.0, 0.5), (43.0, 0.2), tol)
  assert r["status"] == UNEVALUABLE and r["reason"] == "tolerance_invalid"
  assert r["total"] is None and r["path"] is None and r["tracking"] is None


def test_unset_tolerance_reason():
  r = decompose(*STRAIGHT, 40.0, (40.0, 0.5), (40.0, 0.2), None)
  assert r["status"] == UNEVALUABLE and r["reason"] == "tolerance_unset"


def test_zero_tolerance_accepts_only_exact_progress():
  assert decompose(*STRAIGHT, 40.0, (40.0, 0.5), (40.0, 0.2), 0.0)["status"] == OK
  r = decompose(*STRAIGHT, 40.0, (40.0, 0.5), (40.01, 0.2), 0.0)
  assert r["status"] == UNEVALUABLE and r["reason"] == "longitudinal_mismatch"


@pytest.mark.parametrize("dx,status", [(0.25, OK), (0.5, OK), (0.75, UNEVALUABLE)])
def test_tolerance_boundary_is_inclusive(dx, status):
  # dx values are exact in binary; |tangential| == tol is accepted.
  assert decompose(*STRAIGHT, 40.0, (40.0, 0.5), (40.0 + dx, 0.2), 0.5)["status"] == status


def test_large_finite_inputs_with_large_mismatch_not_ok():
  r = decompose(*STRAIGHT, 40.0, (40.0, 1e300), (1e300, 0.2), 1.0)
  assert r["status"] == UNEVALUABLE


@pytest.mark.parametrize("min_den", [0.0, -0.0, -1e-3, float("nan"), float("inf"), True, "1e-3", None])
def test_ratio_invalid_lower_bound(min_den):
  assert safe_ratio(0.1, 0.5, min_den) is None


@pytest.mark.parametrize("den", [0.0, -0.0])
@pytest.mark.parametrize("min_den", [1e-3, 1e-300])
def test_ratio_signed_zero_denominator_rejected_before_division(den, min_den):
  assert safe_ratio(0.1, den, min_den) is None


def test_ratio_bound_inclusive_and_negative_denominator():
  m = 0.5  # exact in binary
  assert safe_ratio(0.25, 0.5, m) == 0.5           # |den| == bound: accepted
  assert safe_ratio(0.25, -0.5, m) == -0.5         # negative denominator keeps its sign
  assert safe_ratio(0.25, np.nextafter(0.5, 0.0), m) is None
  assert safe_ratio(0.25, np.nextafter(0.5, 1.0), m) is not None


def test_ratio_nonfinite_result_is_not_a_number():
  assert safe_ratio(1e308, 1e-300, 1e-310) is None   # overflows to inf
  assert safe_ratio(-1e308, 1e-300, 1e-310) is None


@pytest.mark.parametrize("num", [None, "0.1", float("nan"), float("inf"), True])
def test_ratio_missing_or_nonnumeric_numerator_no_exception(num):
  assert safe_ratio(num, 0.5, 1e-3) is None
