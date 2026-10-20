"""1 km/h entry boundary for K9StopTaper (user-selected 2026-10-20). Source-logic only, not vehicle comfort."""
import pytest

from opendbc.car.hyundai import k9_stop_taper as kt
from opendbc.car.hyundai.k9_stop_taper import TaperPhase
from openpilot.selfdrive.car.tests.test_k9_stop_taper import armed, step


def test_taper_speed_is_1kph():
  assert kt.TAPER_SPEED == pytest.approx(1.0 / 3.6)


@pytest.mark.parametrize("kph", [1.0, 1.01, 2.0, 3.0, 3.5])
def test_no_entry_at_or_above_1kph(kph):
  t = armed()
  for _ in range(100):
    assert step(t, v=kph / 3.6) == -1.0
  assert t.phase == TaperPhase.idle


@pytest.mark.parametrize("kph", [0.99, 0.5, 0.2])
def test_entry_below_1kph_weakens_never_stronger(kph):
  t = armed()
  outs = [step(t, v=kph / 3.6) for _ in range(30)]
  assert t.phase == TaperPhase.taper
  assert all(-1.0 <= o <= 0.0 for o in outs) and outs[-1] > -1.0


def test_rising_to_1kph_ends_episode_passes_original():
  t = armed()
  for _ in range(10):
    step(t, v=0.5 / 3.6)
  assert t.phase == TaperPhase.taper
  assert step(t, accel=-1.0, a_target=-1.0, v=1.0 / 3.6) == -1.0
  assert t.phase != TaperPhase.taper


@pytest.mark.parametrize("kw", [{"brake": True}, {"gas": True}, {"override": True}, {"long_active": False}, {"can_valid": False}])
def test_hard_interlocks_below_1kph_pass_original(kw):
  t = armed()
  for _ in range(10):
    step(t, v=0.5 / 3.6)
  assert step(t, v=0.5 / 3.6, **kw) == -1.0 and t.phase == TaperPhase.done
