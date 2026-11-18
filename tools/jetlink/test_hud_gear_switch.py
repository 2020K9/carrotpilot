"""R -> P / D: the HUD shows the new screen on the very first snapshot that carries the new gear (대감님 11/19 06:18
"R단에서 P단이나 D단 넘어갈 때 HUD 반응이 왜 이렇게 느림?").

Measured 11/19 on route 0000025b--e0edab79f5 (segments 23-24, 4 R exits/entries): carState.gearShifter follows the
lever (LVR12 CF_Lvr_Gear) within 4 ms. These tests pin every later step to "no hold": the Jetson RemoteSubMaster,
the live carState parser, and the renderer's screen choice must all switch on the same update, with no debounce,
fade or minimum time on the R screen. Pure python, no GL / USB.
"""
import base64
import sys
from dataclasses import replace
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
CLUSTER = HERE.parents[1] / 'openpilot/selfdrive/carrot/cluster'
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(CLUSTER))
import hud_protocol as P
from openpilot.cereal import log


def record(stamp, gear, started=True):
  events = {'carState': base64.b64encode(log.Event.new_message(carState={'gearShifter': gear, 'vEgo': 0.5},
                                                               logMonoTime=int(stamp * 1e9)).to_bytes()).decode(),
            'deviceState': base64.b64encode(log.Event.new_message(deviceState={'started': started}).to_bytes()).decode()}
  names = list(events)
  return (stamp, {'version': 1, 'schema': P.SNAPSHOT_SCHEMA, 'sent': stamp, 'events': events, 'params': {},
                  'dropped': {}, 'valid': dict.fromkeys(names, True), 'alive': dict.fromkeys(names, True),
                  'received': dict.fromkeys(names, stamp), 'mono': {n: int(stamp * 1e9) for n in names}})


@pytest.fixture
def screens(monkeypatch, tmp_path):
  import cluster_renderer as R
  monkeypatch.setattr(R, 'EV6_PARK_SCREEN_OFF_FLAG', str(tmp_path / 'park-off'))
  monkeypatch.setattr(R, 'EV6_REVERSE_SCREEN_OFF_FLAG', str(tmp_path / 'reverse-off'))
  renderer = R.ClusterUiRenderer()

  def screen(state):
    if renderer._ev6_park_screen_active(state):
      return 'P'
    if renderer._ev6_reverse_screen_active(state):
      return 'R'
    return 'D' if renderer._k9_turzx_active(state) else 'other'
  return screen


def k9_state(car_state):
  from cluster_live import standby_state
  from cluster_route_replay import RouteLogParser, frame_to_state
  parser = RouteLogParser(recompute_cutins=False)
  frame = parser._frame_from_car_state(car_state, 1.0)
  return replace(frame_to_state(frame), onroad=True, car_fingerprint='KIA_K9') if frame else standby_state()


@pytest.mark.parametrize('after,screen_after', [('drive', 'D'), ('park', 'P'), ('neutral', 'D')])
def test_first_snapshot_with_new_gear_switches_the_screen(monkeypatch, screens, after, screen_after):
  import hud
  feed = []
  monkeypatch.setattr(hud, 'read_snapshot', lambda: feed[-1])
  monkeypatch.setattr(hud, 'REJECTED', {})
  sm = hud.RemoteSubMaster(['carState', 'deviceState'])
  for i in range(5):   # a while in R first: no R time may carry into the next gear
    feed.append(record(100. + i * .1, 'reverse'))
    sm.update(0)
    assert screens(k9_state(sm['carState'])) == 'R'
  feed.append(record(100.5, after))
  sm.update(0)
  assert sm.updated['carState'] and str(sm['carState'].gearShifter) == after
  assert screens(k9_state(sm['carState'])) == screen_after   # same update, no hold
  feed.append(record(100.6, 'reverse'))   # and straight back: R again on the next update
  sm.update(0)
  assert screens(k9_state(sm['carState'])) == 'R'


@pytest.mark.parametrize('before', ['park', 'drive'])
def test_entering_reverse_is_immediate_too(monkeypatch, screens, before):
  import hud
  feed = [record(50., before)]
  monkeypatch.setattr(hud, 'read_snapshot', lambda: feed[-1])
  monkeypatch.setattr(hud, 'REJECTED', {})
  sm = hud.RemoteSubMaster(['carState', 'deviceState'])
  sm.update(0)
  assert screens(k9_state(sm['carState'])) in ('P', 'D')
  feed.append(record(50.1, 'reverse'))
  sm.update(0)
  assert screens(k9_state(sm['carState'])) == 'R'


def test_comma_snapshot_period_is_100ms():
  """The comma writes a new HUD snapshot every 0.1 s (SnapshotBuilder.packet); a slower cadence would delay gears."""
  src = (HERE / 'hud_protocol.py').read_text()
  assert 'self.next_send = now + .1 if now - self.next_send > .2 else self.next_send + .1' in src
  assert 'time.sleep(max(0, start + .1 - time.monotonic()))' in (HERE / 'hud_publisher.py').read_text()
