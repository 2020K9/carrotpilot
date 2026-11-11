"""K9 (2020 RJ, LEGACY CAN) fuel level for carState.fuelGauge (P-gear park screen), read in card, not in opendbc.

Logs 10/22..11/06: CLU13 0x50C on bus 0 at 10 Hz. CF_Clu_FuelDispLvl (bits 18..22) is always 0 on the K9, so the
fraction comes from CF_Clu_DTE (bits 24..33, km to empty; 27 km with the low-fuel lamp on 10/22, 405 km on 11/06)
over an assumed full-tank range. Estimate, not a tank sensor. Raw bytes only: no parser message, no CAN validity
check, nothing sent, other cars untouched.
"""

CLU13_ADDR = 0x50C
CLU13_BUS = 0
DTE_FULL_KM = 600.0
DTE_INVALID = 1023


def clu13_fuel_gauge(dat: bytes) -> float | None:
  if len(dat) < 5:
    return None
  x = int.from_bytes(dat[:5], "little")
  lvl = (x >> 18) & 0x1F
  dte = (x >> 24) & 0x3FF
  if lvl > 0:
    return min(lvl / 31.0, 1.0)
  if 0 < dte < DTE_INVALID:
    return min(dte / DTE_FULL_KM, 1.0)
  return None


class K9Fuel:
  def __init__(self, CP):
    self.enabled = str(CP.carFingerprint) == "KIA_K9"
    self.gauge: float | None = None

  def observe(self, can_list) -> None:
    if not self.enabled:
      return
    for _, frames in can_list:
      for f in frames:
        if f[0] == CLU13_ADDR and f[2] == CLU13_BUS:
          g = clu13_fuel_gauge(bytes(f[1]))
          if g is not None:
            self.gauge = g

  def apply(self, CS) -> None:
    if self.enabled and self.gauge is not None:
      CS.fuelGauge = self.gauge
