"""Lateral error decomposition against the CONSUMED curvature (offline spec code).

Not used by any onboard process.  Contract:

  * one fixed planar frame for everything (origin, axes given by the caller);
  * C(s): lane-centre polyline in that frame, parameterised by arc length s;
  * E(t): ego position in the same frame (measured or simulated -- the caller
    labels the source; this module never calls it a vehicle measurement);
  * P(t): position reconstructed from the curvature history that was actually
    consumed (lateral controller input), by integrating
        psi' = v k,  x' = v cos psi,  y' = v sin psi   (trapezoid on psi, then x/y)
    from a stated initial state.  P is a reconstruction under these kinematic
    assumptions, not the model position array and not a trajectory measurement;
  * every comparison uses ONE progress value s for C and measures E and P along
    the centre normal at C(s).  Nearest points are never chosen separately for E
    and P.  P's tangential distance from C(s) is reported; if it exceeds the
    caller's limit (or no limit is given) the sample is unevaluable;
  * total = e_lat, path = p_lat, tracking = e_lat - p_lat (C's own lateral is 0
    on its normal), so path + tracking == total by construction.  The sum holding
    is bookkeeping, not proof that the split is causal.

Missing or non-finite inputs and non-increasing time stamps make a sample
unevaluable; nothing is zero-filled.
"""
from __future__ import annotations

import math
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

UNEVALUABLE = "unevaluable"
OK = "ok"


def _finite(*arrs: np.ndarray) -> bool:
  return all(a.size > 0 and bool(np.all(np.isfinite(a))) for a in arrs)


def reconstruct_from_curvature(t: Sequence[float], curvature: Sequence[float], v: Sequence[float],
                               x0: float, y0: float, psi0: float) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
  """P(t) from the consumed curvature history. None if time is not strictly
  increasing or any input is missing/non-finite."""
  ta, ka, va = (np.asarray(a, dtype=np.float64).ravel() for a in (t, curvature, v))
  if ta.size < 2 or ka.size != ta.size or va.size != ta.size:
    return None
  if not _finite(ta, ka, va) or not all(math.isfinite(z) for z in (x0, y0, psi0)):
    return None
  dt = np.diff(ta)
  if np.any(dt <= 0.0):
    return None  # duplicated or reversed time stamps
  yaw_rate = va * ka
  psi = psi0 + np.concatenate([[0.0], np.cumsum(0.5 * (yaw_rate[1:] + yaw_rate[:-1]) * dt)])
  vx, vy = va * np.cos(psi), va * np.sin(psi)
  x = x0 + np.concatenate([[0.0], np.cumsum(0.5 * (vx[1:] + vx[:-1]) * dt)])
  y = y0 + np.concatenate([[0.0], np.cumsum(0.5 * (vy[1:] + vy[:-1]) * dt)])
  return x, y, psi


def center_frame_at(cx: Sequence[float], cy: Sequence[float], s: float) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
  """(point, unit tangent, unit normal) of the centre polyline at arc length s.
  Normal = tangent rotated +90 deg in the caller's frame (the caller's y axis
  sense defines the sign)."""
  x, y = np.asarray(cx, dtype=np.float64).ravel(), np.asarray(cy, dtype=np.float64).ravel()
  if x.size < 2 or y.size != x.size or not _finite(x, y) or not math.isfinite(s):
    return None
  seg = np.hypot(np.diff(x), np.diff(y))
  if np.any(seg <= 0.0):
    return None
  arc = np.concatenate([[0.0], np.cumsum(seg)])
  if s < 0.0 or s > arc[-1]:
    return None  # no extrapolation of the centre line
  i = int(min(np.searchsorted(arc, s, side="right") - 1, x.size - 2))
  f = (s - arc[i]) / seg[i]
  p = np.array([x[i] + f * (x[i + 1] - x[i]), y[i] + f * (y[i + 1] - y[i])])
  tan = np.array([x[i + 1] - x[i], y[i + 1] - y[i]]) / seg[i]
  nrm = np.array([-tan[1], tan[0]])
  return p, tan, nrm


def decompose(cx: Sequence[float], cy: Sequence[float], s: float,
              ego_xy: Sequence[float], target_xy: Sequence[float],
              max_tangential: Optional[float]) -> Dict[str, object]:
  """One sample at one time t: C(s), E(t), P(t) all in the same frame."""
  out: Dict[str, object] = {"status": UNEVALUABLE, "total": None, "path": None, "tracking": None,
                            "ego_tangential": None, "target_tangential": None}
  frame = center_frame_at(cx, cy, s)
  e, p = np.asarray(ego_xy, dtype=np.float64).ravel(), np.asarray(target_xy, dtype=np.float64).ravel()
  if frame is None or e.size != 2 or p.size != 2 or not _finite(e, p) or max_tangential is None:
    return out
  c, tan, nrm = frame
  e_tan, p_tan = float((e - c) @ tan), float((p - c) @ tan)
  out["ego_tangential"], out["target_tangential"] = e_tan, p_tan
  if abs(e_tan) > max_tangential or abs(p_tan) > max_tangential:
    return out  # longitudinal mismatch: not the same progress position
  e_lat, p_lat = float((e - c) @ nrm), float((p - c) @ nrm)
  out.update(status=OK, total=e_lat, path=p_lat, tracking=e_lat - p_lat)
  return out


def safe_ratio(numerator: float, denominator: float, min_abs_denominator: Optional[float]) -> Optional[float]:
  """Target/actual style ratio; None (not a normal value) near zero or unconfigured."""
  if min_abs_denominator is None or not (math.isfinite(numerator) and math.isfinite(denominator)):
    return None
  if abs(denominator) < min_abs_denominator:
    return None
  return numerator / denominator
