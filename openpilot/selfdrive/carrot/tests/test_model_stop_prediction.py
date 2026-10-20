"""Native Params migration TrafficLightDetectMode -> ModelStopPrediction, and the
generic motorway suspension of the model-native stop (formerly signal_road_policy)."""
from types import SimpleNamespace as NS

import pytest

from openpilot.selfdrive.carrot.traffic_stop import (LEGACY_MODEL_STOP_KEY, MODEL_STOP_PREDICTION_KEY,
                                                     migrate_model_stop_prediction, model_stop_suspended)


@pytest.fixture
def params(tmp_path):
  from openpilot.common.params import Params
  return Params(str(tmp_path / 'params'))


def populate_defaults(params):
  # Same order and rule as manager_init(): migrate, then fill unset defaults.
  migrate_model_stop_prediction(params)
  for k in params.all_keys():
    default_value = params.get_default_value(k)
    if default_value is not None and params.get(k) is None:
      params.put(k, default_value)


def test_registered_keys_and_defaults(params):
  assert params.get_default_value(MODEL_STOP_PREDICTION_KEY) == 1
  # Retired key stays registered for migration only and is never default-populated.
  assert params.get_default_value(LEGACY_MODEL_STOP_KEY) is None
  for retired in ('SignalAssistMode', 'LeadBrakeLights', 'TrafficLight'):
    with pytest.raises(Exception):
      params.check_key(retired)


@pytest.mark.parametrize('legacy,expected', [(0, 0), (1, 1), (2, 1)])
def test_one_time_legacy_migration(params, legacy, expected):
  params.put_int(LEGACY_MODEL_STOP_KEY, legacy)
  populate_defaults(params)
  assert params.get_int(MODEL_STOP_PREDICTION_KEY) == expected
  # A later user change survives every manager restart; the old key is not reapplied.
  params.put_int(MODEL_STOP_PREDICTION_KEY, 1 - expected)
  populate_defaults(params)
  populate_defaults(params)
  assert params.get_int(MODEL_STOP_PREDICTION_KEY) == 1 - expected


def test_explicit_new_key_is_never_overwritten(params):
  params.put_int(MODEL_STOP_PREDICTION_KEY, 0)
  params.put_int(LEGACY_MODEL_STOP_KEY, 2)
  populate_defaults(params)
  assert params.get_int(MODEL_STOP_PREDICTION_KEY) == 0


def test_absent_legacy_uses_default(params):
  populate_defaults(params)
  assert params.get_int(MODEL_STOP_PREDICTION_KEY) == 1
  assert params.get(LEGACY_MODEL_STOP_KEY) is None


@pytest.mark.parametrize('raw', [b'abc', b'', b'-1'])
def test_malformed_or_negative_legacy_falls_back_to_default(params, raw):
  path = params.get_param_path(LEGACY_MODEL_STOP_KEY) if hasattr(params, 'get_param_path') else None
  if path is None:
    pytest.skip('Params binding has no raw path access')
  with open(path, 'wb') as f:
    f.write(raw)
  populate_defaults(params)
  assert params.get_int(MODEL_STOP_PREDICTION_KEY) == 1


def test_migration_ignores_non_int_values():
  class Fake(dict):
    def get(self, key):
      return dict.get(self, key)
    def put_int(self, key, value):
      self[key] = value
  for bad in ('2', 1.0, True, -3):
    p = Fake({LEGACY_MODEL_STOP_KEY: bad})
    migrate_model_stop_prediction(p)
    assert MODEL_STOP_PREDICTION_KEY not in p

  class Raising(Fake):
    def get(self, key):
      if key == LEGACY_MODEL_STOP_KEY:
        raise ValueError('malformed')
      return None
  p = Raising()
  migrate_model_stop_prediction(p)
  assert MODEL_STOP_PREDICTION_KEY not in p


def test_planner_reads_only_new_key():
  from pathlib import Path
  src = (Path(__file__).parents[1] / 'carrot_functions.py').read_text(encoding='utf-8')
  assert '"ModelStopPrediction"' in src and 'TrafficLightDetectMode' not in src


# --- motorway suspension of the generic model stop (formerly signal_road_policy) ---

class SM(dict):
  def __init__(self, category=0, *, seen=True, valid=True, alive=True, recv=100., received_ns=100_000_000_000,
               connected=True, session='test', schema=1, off_route=False, category_valid=True, lane_present=True):
    super().__init__()
    self.seen = {'carrotNavi': seen}
    self.valid = {'carrotNavi': valid}
    self.alive = {'carrotNavi': alive}
    self.recv_time = {'carrotNavi': recv}
    self['carrotNavi'] = NS(schemaVersion=schema, connected=connected, sessionId=session,
      navigationStatus=NS(meta=NS(present=True), offRoute=off_route),
      laneCurrent=NS(meta=NS(present=lane_present, receivedMonoTimeNanos=received_ns),
                     roadCategory=category, roadCategoryValid=category_valid))


@pytest.mark.parametrize('category,expected', [(0, True), (1, True), (2, False), (8, False), (-1, False), (12, False)])
def test_explicit_motorway_class_suspends(category, expected):
  assert model_stop_suspended(SM(category), 100.) is expected


@pytest.mark.parametrize('kwargs', [
  {'seen': False}, {'valid': False}, {'alive': False}, {'recv': 97.9}, {'recv': 100.1},
  {'connected': False}, {'session': ''}, {'schema': 2}, {'off_route': True},
  {'category_valid': False}, {'lane_present': False}, {'received_ns': 0}, {'received_ns': 89_000_000_000},
])
def test_stale_invalid_or_offroute_navigation_never_suspends(kwargs):
  assert model_stop_suspended(SM(0, **kwargs), 100.) is False


def test_malformed_navigation_never_suspends():
  sm = SM(0)
  sm['carrotNavi'] = NS()
  assert model_stop_suspended(sm, 100.) is False
  assert model_stop_suspended({}, 100.) is False
