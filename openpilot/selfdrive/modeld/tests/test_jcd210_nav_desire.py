from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.selfdrive.modeld import jcd210_nav_desire as jnd
from openpilot.selfdrive.modeld.jcd210_nav_desire import PULSE_FRAMES, Jcd210Desire, is_jcd210, jcd210_sha256, nav_desire

D = log.Desire
NOW = 1000.


def dh(desire=D.none):
  return SimpleNamespace(desire=desire)


def man(kind, modifier, distance):
  return SimpleNamespace(type=kind, modifier=modifier, distance=distance)


def sm(*maneuvers, valid=True, age=0.):
  return SimpleNamespace(valid={'navInstructionCarrot': valid}, recv_time={'navInstructionCarrot': NOW - age},
                         navInstructionCarrot=SimpleNamespace(allManeuvers=list(maneuvers)))


class SM(SimpleNamespace):
  def __getitem__(self, key):
    return getattr(self, key)


def nsm(*maneuvers, valid=True, age=0.):
  s = sm(*maneuvers, valid=valid, age=age)
  return SM(valid=s.valid, recv_time=s.recv_time, navInstructionCarrot=s.navInstructionCarrot)


def run(helper, frames, s, d=None, speed=0.):
  d = d or dh()
  return [helper.update(d, s, speed, NOW) for _ in range(frames)]


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
  assert not is_jcd210(SimpleNamespace(prev_desire=None))


def test_mapping_from_n16_navigation_command():
  for kind, modifier, want in (('turn', 'left', D.turnLeft), ('turn', 'sharp left', D.turnLeft),
                               ('turn', 'right', D.turnRight), ('turn', 'sharp right', D.turnRight),
                               ('turn', 'slight left', D.keepLeft), ('turn', 'slight right', D.keepRight),
                               ('fork', 'left', D.keepLeft), ('fork', 'right', D.keepRight),
                               ('off ramp', 'left', D.keepLeft), ('off ramp', 'right', D.keepRight),
                               ('on ramp', 'left', D.keepLeft), ('on ramp', 'right', D.keepRight)):
    assert nav_desire(nsm(man(kind, modifier, 50.)), 0., NOW) == want, (kind, modifier)


def test_never_lane_change_desire():
  values = {nav_desire(nsm(man(k, m, 10.)), 0., NOW) for k in ('turn', 'fork', 'on ramp', 'off ramp', 'continue')
            for m in ('left', 'right', 'slight left', 'slight right', 'sharp left', 'sharp right', 'uturn', '')}
  assert D.laneChangeLeft not in values and D.laneChangeRight not in values


def test_no_desire_for_straight_uturn_roundabout_merge_arrive():
  for kind, modifier in (('turn', 'straight'), ('continue', ''), ('turn', 'uturn'), ('roundabout', 'left'),
                         ('merge', 'left'), ('arrive', 'left'), ('fork', 'straight'), ('notification', '')):
    assert nav_desire(nsm(man(kind, modifier, 10.)), 0., NOW) == D.none, (kind, modifier)


def test_distance_window_and_travel_compensation():
  assert nav_desire(nsm(man('turn', 'left', 99.)), 0., NOW) == D.turnLeft
  assert nav_desire(nsm(man('turn', 'left', 100.)), 0., NOW) == D.none
  assert nav_desire(nsm(man('turn', 'left', 120.), age=1.), 30., NOW) == D.turnLeft  # 120 - 30 m
  assert nav_desire(nsm(man('turn', 'left', 5.), age=1.), 10., NOW) == D.none  # already passed


def test_freshness_and_validity_like_n16():
  assert nav_desire(nsm(man('turn', 'left', 50.), valid=False), 0., NOW) == D.none
  assert nav_desire(nsm(man('turn', 'left', 50.), age=2.), 0., NOW) == D.none
  assert nav_desire(nsm(man('turn', 'left', 50.), age=1.9), 0., NOW) == D.turnLeft


def test_first_n16_command_is_used():
  assert nav_desire(nsm(man('turn', 'straight', 10.), man('turn', 'right', 50.)), 0., NOW) == D.none
  assert nav_desire(nsm(man('depart', 'left', 10.), man('turn', 'right', 50.)), 0., NOW) == D.turnRight


def test_pulses_every_second():
  out = run(Jcd210Desire(), 3 * PULSE_FRAMES, nsm(man('turn', 'left', 50.)))
  assert [i for i, v in enumerate(out) if v != D.none] == [0, PULSE_FRAMES, 2 * PULSE_FRAMES]
  assert {v for v in out if v != D.none} == {D.turnLeft}


def test_desire_helper_desire_has_priority_and_repulses():
  out = run(Jcd210Desire(), 2 * PULSE_FRAMES, nsm(man('turn', 'right', 50.)), d=dh(D.laneChangeLeft))
  assert out[0] == D.laneChangeLeft and out[PULSE_FRAMES] == D.laneChangeLeft
  assert D.turnRight not in out


def test_new_desire_pulses_immediately():
  h = Jcd210Desire()
  run(h, 5, nsm(man('fork', 'left', 50.)))
  assert run(h, 1, nsm(man('turn', 'left', 50.))) == [D.turnLeft]
  assert run(h, 1, nsm()) == [D.none]
  assert run(h, 1, nsm(man('turn', 'left', 50.))) == [D.turnLeft]


def test_missing_catalog_is_not_jcd210(monkeypatch):
  monkeypatch.setattr(jnd, '_JCD210_SHA256', [None])
  assert not is_jcd210(model('f73a9e535523d5e9acb9e642c64e33d631825dc8ba74123757d107cedd047bb5'))


def test_does_not_read_atc():
  import inspect
  assert 'atc' not in inspect.getsource(jnd).lower()


def test_modeld_other_models_keep_desire_helper_desire():
  import inspect
  from openpilot.selfdrive.modeld import modeld
  src = inspect.getsource(modeld.main)
  block = "".join(line + "\n" for line in (
    "    desire = DH.desire",
    "    if is_jcd210(model):",
    "      desire = jcd210_desire.update(DH, sm, sm['carState'].vEgo)",
    "    else:",
    "      jcd210_desire.reset()"))
  assert src.count(block) == 1
  assert src.count('vec_desire[desire] = 1') == 1
