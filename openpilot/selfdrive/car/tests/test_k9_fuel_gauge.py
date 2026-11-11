"""K9 fuel gauge from CLU13 0x50C (대감님 11/11 19:27). New tests only."""
import math
from pathlib import Path
from types import SimpleNamespace

from openpilot.selfdrive.car.k9_fuel import DTE_FULL_KM, K9Fuel, clu13_fuel_gauge

# real 11/06 seg16 bus-0 payload start (DTE 0x94 + (1 << 8) = 404 km, level 0)
K9_1106 = bytes([0x00, 0x00, 0x00, 0x94, 0x01, 0x00, 0x00, 0x00])
# real 10/22 seg9 payload start (DTE 27 km, low-fuel bits = 1, level 0)
K9_1022 = bytes([0x01, 0x00, 0x00, 0x1B, 0x00, 0x00, 0x00, 0x00])


def test_real_k9_frames_give_dte_fraction():
  assert math.isclose(clu13_fuel_gauge(K9_1106), 404 / DTE_FULL_KM)
  assert math.isclose(clu13_fuel_gauge(K9_1022), 27 / DTE_FULL_KM)


def test_fuel_level_wins_when_a_car_fills_it():
  lvl16 = (16 << 18 | 404 << 24).to_bytes(8, "little")
  assert math.isclose(clu13_fuel_gauge(lvl16), 16 / 31)


def test_unknown_dte_is_ignored():
  assert clu13_fuel_gauge(bytes(8)) is None
  assert clu13_fuel_gauge((1023 << 24).to_bytes(8, "little")) is None
  assert clu13_fuel_gauge(b"\x00\x00") is None


def _cs():
  return SimpleNamespace(fuelGauge=0.0)


def test_only_k9_bus0_clu13_sets_fuel_gauge():
  can = [(1, [(0x50C, K9_1106, 1), (0x50C, K9_1106, 130), (0x50B, K9_1106, 0)])]
  k9 = K9Fuel(SimpleNamespace(carFingerprint="KIA_K9"))
  k9.observe(can)
  cs = _cs()
  k9.apply(cs)
  assert cs.fuelGauge == 0.0  # radar 0x50C on bus 1 and echoes are not fuel
  k9.observe([(2, [(0x50C, K9_1106, 0)])])
  k9.apply(cs)
  assert math.isclose(cs.fuelGauge, 404 / DTE_FULL_KM)
  other = K9Fuel(SimpleNamespace(carFingerprint="KIA_K7"))
  other.observe([(2, [(0x50C, K9_1106, 0)])])
  cs2 = _cs()
  other.apply(cs2)
  assert cs2.fuelGauge == 0.0


def test_card_feeds_fuel_after_carstate_update():
  src = (Path(__file__).parents[1] / "card.py").read_text(encoding="utf-8")
  i = src.index("CS = self.CI.update(can_list)")
  assert src.index("self.k9_fuel.observe(can_list)", i) < src.index("self.k9_fuel.apply(CS)", i)
