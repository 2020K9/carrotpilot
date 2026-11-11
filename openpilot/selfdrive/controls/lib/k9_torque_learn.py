"""LateralTorqueCustom 1: menu torque values until torqued has learned, then the learned values
(보노보노 2026-10-26 17:29 "토크학습100%차면 현재학습값으로 변하게만들어").

latcontrol_torque with LateralTorqueCustom > 0 used LateralTorqueAccelFactor / LateralTorqueFriction x 0.001 always and threw
liveTorqueParameters away. Now:

- until torqued says its fit is valid (liveTorqueParameters.liveValid, i.e. calPerc 100 %: every steer bucket and the total
  point count full) the menu values are used exactly as before (bit-identical, w == 0);
- from then on the applied latAccelFactor / friction cross-fade to liveTorqueParameters Filtered (the values upstream
  controlsd gives the torque controller, sanity-clipped by torqued):  applied = (1 - w) * menu + w * Filtered.
- w follows torqued's own filter progress, target = (decay - 50) / (250 - 50) (torqued MIN_FILTER_DECAY / MAX_FILTER_DECAY):
  torqued starts Filtered at the CarParams seed (K9 2.30 / 0.10) with decay 50 when it first gets valid, and decay grows by
  DT_MDL per valid update up to 250 (about 17 min of valid data). At decay d Filtered has covered about 1 - 50 / d of the way
  from the seed to the fit (1c8 rlog seg 42: decay 113..125, Filtered 2.62..2.65 from 2.30 toward fits 2.75..2.97), so the
  menu value is not swapped for the bare seed (recheck_0208 cl3c sim, 6 scenes 1a5 / 1a3 / 1ca / 1c9, document criteria: seed
  2.30 / 0.100 worse than the menu 2.50 / 0.131 in 9 conditions on 6 / 6 scenes; mid-fade 2.585 / 0.116 and learned
  2.708 / 0.102 worse nowhere) and the learned value takes over as it is learned (1c8 replay: w 0.25 / 0.5 / 1 at 245 / 490 /
  980 s after calPerc 100, applied 2.52 / 2.59 / 2.71). A drive that restores a valid cache (1c9: decay 250, Filtered 2.71)
  fades in fully within 30 s.
- w moves at most RATE_PER_S per second toward the target (30 s for 0 -> 1): no step at the switch, nor at restarts.
- back to the menu values (same 30 s fade, from the last learned values) when liveValid drops (torqued reset after a bad fit:
  points thrown away, Filtered no longer a learned value), when the message stops arriving / fails the checks for STALE_S,
  or when the learned numbers are not finite / not positive. The latAccelOffset stays the CarParams default as before.
"""
from __future__ import annotations

import math

TORQUED_MIN_DECAY = 50.0    # == locationd/torqued.py MIN_FILTER_DECAY (test pins it)
TORQUED_MAX_DECAY = 250.0   # == locationd/torqued.py MAX_FILTER_DECAY
RATE_PER_S = 1.0 / 30.0     # w change per second at most
STALE_S = 1.0               # no fresh valid liveTorqueParameters for this long -> back to the menu values


def _finite(x) -> float | None:
  try:
    v = float(x)
  except (TypeError, ValueError):
    return None
  return v if math.isfinite(v) else None


class TorqueLearnSwitch:
  def __init__(self) -> None:
    self.w = 0.0
    self.target = 0.0
    self.laf: float | None = None
    self.friction: float | None = None
    self.age = math.inf

  def update_learned(self, live_valid, lat_accel_factor, friction, decay) -> None:
    """One liveTorqueParameters reading (controlsd: only when the message passes all_checks and useParams)."""
    laf, fr, d = _finite(lat_accel_factor), _finite(friction), _finite(decay)
    if bool(live_valid) and laf is not None and laf > 0.0 and fr is not None and fr >= 0.0 and d is not None:
      self.laf, self.friction = laf, fr
      self.target = min(max((d - TORQUED_MIN_DECAY) / (TORQUED_MAX_DECAY - TORQUED_MIN_DECAY), 0.0), 1.0)
      self.age = 0.0
    else:
      self.target = 0.0

  def step(self, dt: float) -> None:
    self.age += dt
    target = self.target if self.age <= STALE_S and self.laf is not None else 0.0
    dw = RATE_PER_S * dt
    if self.w < target:
      self.w = min(target, self.w + dw)
    elif self.w > target:
      self.w = max(target, self.w - dw)

  def apply(self, menu_laf: float, menu_friction: float) -> tuple[float, float]:
    """(latAccelFactor, friction) to use; exactly the menu values while w == 0."""
    if self.w <= 0.0 or self.laf is None:
      return menu_laf, menu_friction
    w = self.w
    return (1.0 - w) * menu_laf + w * self.laf, (1.0 - w) * menu_friction + w * self.friction
