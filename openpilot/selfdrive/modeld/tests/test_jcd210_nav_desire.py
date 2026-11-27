from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.selfdrive.modeld import jcd210_nav_desire as jnd
from openpilot.selfdrive.modeld.jcd210_nav_desire import PULSE_FRAMES, Jcd210Desire, is_jcd210, jcd210_sha256

D = log.Desire
OFF = log.LaneChangeState.off


def dh(desire=D.none, state=OFF, atc_active=0):
  return SimpleNamespace(desire=desire, lane_change_state=state, atc_active=atc_active)


def cs(left=False, right=False):
  return SimpleNamespace(leftBlindspot=left, rightBlindspot=right)


def run(helper, frames, atc, d=None, car=None):
  d = d or dh()
  car = car or cs()
  return [helper.update(d, car, atc) for _ in range(frames)]


def model(sha, connected=True):
  return SimpleNamespace(client=object() if connected else None, spec=SimpleNamespace(sha256=sha))


def test_catalog_sha_is_jcd210():
  assert jcd210_sha256() == 'f73a9e535523d5e9acb9e642c64e33d631825dc8ba74123757d107cedd047bb5'


def test_only_connected_jcd210_is_jcd210():
  assert is_jcd210(model(jcd210_sha256()))
  assert not is_jcd210(model(jcd210_sha256(), connected=False))
  from openpilot.common.big_model_catalog import available, model_spec
  for key in available():
    if key != 'jcd210':
      assert not is_jcd210(model(model_spec(key)['sha256'])), key
  # Local ModelState / eGPU model has no Jetlink client or spec.
  assert not is_jcd210(SimpleNamespace(prev_desire=None))


def test_mapping_pulses_every_second():
  for atc, want in (('turn left', D.turnLeft), ('turn right', D.turnRight), ('atc left', D.keepLeft),
                    ('atc right', D.keepRight), ('fork left', D.keepLeft), ('fork right', D.keepRight)):
    out = run(Jcd210Desire(), 3 * PULSE_FRAMES, atc)
    assert [i for i, v in enumerate(out) if v != D.none] == [0, PULSE_FRAMES, 2 * PULSE_FRAMES], atc
    assert {v for v in out if v != D.none} == {want}


def test_pulse_passes_rising_edge_each_second():
  out = run(Jcd210Desire(), 2 * PULSE_FRAMES + 1, 'turn left')
  prev, edges = D.none, 0
  for v in out:
    edges += v == D.turnLeft and prev != D.turnLeft
    prev = v
  assert edges == 3


def test_no_desire_for_prepare_none_canceled_straight():
  for atc in ('', 'none', 'turn left prepare', 'fork left canceled', 'straight', 'fork right prepare'):
    assert set(run(Jcd210Desire(), PULSE_FRAMES, atc)) == {D.none}, atc


def test_desire_helper_desire_has_priority_and_repulses():
  out = run(Jcd210Desire(), 2 * PULSE_FRAMES, 'turn right', d=dh(D.laneChangeLeft, log.LaneChangeState.laneChangeStarting))
  assert out[0] == D.laneChangeLeft and out[PULSE_FRAMES] == D.laneChangeLeft
  assert D.turnRight not in out and D.keepRight not in out


def test_desire_helper_owns_lane_change_wait_and_conflict():
  assert set(run(Jcd210Desire(), PULSE_FRAMES, 'fork left', d=dh(state=log.LaneChangeState.preLaneChange))) == {D.none}
  assert set(run(Jcd210Desire(), PULSE_FRAMES, 'turn left', d=dh(atc_active=2))) == {D.none}


def test_blindspot_blocks_that_side_only():
  assert set(run(Jcd210Desire(), PULSE_FRAMES, 'fork left', car=cs(left=True))) == {D.none}
  assert run(Jcd210Desire(), 1, 'fork right', car=cs(left=True)) == [D.keepRight]


def test_new_desire_pulses_immediately():
  h = Jcd210Desire()
  run(h, 5, 'atc left')
  assert run(h, 1, 'turn left') == [D.turnLeft]
  assert run(h, 1, 'none') == [D.none]
  assert run(h, 1, 'turn left') == [D.turnLeft]


def test_missing_catalog_is_not_jcd210(monkeypatch):
  monkeypatch.setattr(jnd, '_JCD210_SHA256', [None])
  assert not is_jcd210(model('f73a9e535523d5e9acb9e642c64e33d631825dc8ba74123757d107cedd047bb5'))


def test_modeld_other_models_keep_desire_helper_desire():
  import inspect
  from openpilot.selfdrive.modeld import modeld
  src = inspect.getsource(modeld.main)
  block = ("    desire = DH.desire\n"
           "    if is_jcd210(model):\n"
           "      desire = jcd210_desire.update(DH, sm['carState'], sm['carrotMan'].atcType)\n"
           "    else:\n"
           "      jcd210_desire.reset()\n")
  assert src.count(block) == 1
  # The model input is still built only from this one desire value.
  assert src.count('vec_desire[desire] = 1') == 1
