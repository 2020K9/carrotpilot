"""The cached LiveTorqueParameters point cloud must not push a HUD snapshot past 96 KB."""
import base64
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hud_protocol as protocol
from openpilot.cereal import log


def cached_torque(points):
  event = log.Event.new_message(logMonoTime=7, valid=True)
  torque = event.init('liveTorqueParameters')
  torque.latAccelFactorFiltered, torque.frictionCoefficientFiltered = 2.75, .125
  torque.calPerc, torque.liveValid, torque.totalBucketPoints = 100, True, float(points)
  torque.points = [[.01 * (i % 90) - .45, .5, .1] for i in range(points)]
  return event.to_bytes()


def test_calibrated_cache_fits_with_every_hud_scalar():
  data = cached_torque(12000)   # torqued: 8 buckets x POINTS_PER_BUCKET 1500
  assert len(base64.b64encode(data)) > 96 * 1024
  shown = protocol.without_torque_points(data)
  with log.Event.from_bytes(shown) as event, log.Event.from_bytes(data) as original:
    a, b = event.liveTorqueParameters, original.liveTorqueParameters
    assert (event.logMonoTime, event.valid) == (original.logMonoTime, original.valid)
    for field in ('latAccelFactorFiltered', 'frictionCoefficientFiltered', 'calPerc', 'liveValid', 'totalBucketPoints'):
      assert getattr(a, field) == getattr(b, field)
    assert len(a.points) == 0
  record = {'version': 1, 'events': {'modelV2': 'x' * 30000}, 'cameras': {},
            'params': {'LiveTorqueParameters': base64.b64encode(shown).decode()}}
  assert len(protocol.encode_snapshot(record)) <= 96 * 1024
  assert json.loads(protocol.encode_snapshot(record))['events']['modelV2'] == 'x' * 30000


def test_other_values_pass_unchanged():
  empty = cached_torque(0)
  assert protocol.without_torque_points(empty) == empty
  assert protocol.without_torque_points(b'not capnp') == b'not capnp'
  other = log.Event.new_message(liveDelay={'lateralDelay': .2}).to_bytes()
  assert protocol.without_torque_points(other) == other
