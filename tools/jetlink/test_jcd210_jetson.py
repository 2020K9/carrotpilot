"""Jcd210 젯슨 전용: 카탈로그 사양 = 콤마 내장 CD210 ONNX, action 없는 큐 방식 출력이 내장 경로와 똑같이 파싱된다."""
import sys

import numpy as np
import pytest

from openpilot.common import big_model_catalog as C

sys.path.insert(0, str(C.ROOT / 'third_party/jetlink'))
from jetlink.spec import ModelSpec, spec_from_onnx  # noqa: E402

ONNX = C.ROOT / 'openpilot/selfdrive/modeld/models/driving_supercombo.onnx'


@pytest.fixture(scope='module')
def spec():
  return ModelSpec.from_dict(C.model_spec('jcd210'))


def test_jcd210_is_jetson_only_queued_internal_model(spec):
  entry = C.catalog()['jcd210']
  assert 'jcd210' in C.available() and 'egpu' not in entry and 'manifest_url' not in entry
  assert not C.noo('jcd210')
  onnx = spec_from_onnx(str(ONNX))
  assert onnx.to_dict() == spec.to_dict()
  assert not spec.stateful and 'action' not in spec.output_slices
  assert spec.output_shapes == {'outputs': (1, 2576)}
  from openpilot.selfdrive.modeld.jetlink.contracts import validate_model_spec
  assert validate_model_spec(spec) is spec


def test_jcd210_jetson_outputs_parse_like_internal_path(spec):
  from openpilot.selfdrive.modeld.get_model_metadata import make_metadata_dict
  from openpilot.selfdrive.modeld.jetlink.contracts import parse_outputs
  from openpilot.selfdrive.modeld.modeld import get_action_from_model
  from openpilot.selfdrive.modeld.parse_model_outputs import Parser
  from openpilot.cereal import log

  internal = make_metadata_dict(ONNX)['output_slices']
  assert {k: (v.start, v.stop) for k, v in internal.items()} == {k: (v.start, v.stop) for k, v in spec.output_slices.items()}
  rng = np.random.default_rng(210)
  for _ in range(5):
    raw = rng.normal(0, 1, spec.output_nelem).astype(np.float16).astype(np.float32)
    jetson = parse_outputs(Parser(), spec, raw.copy())
    comma = Parser().parse_outputs({k: raw.copy()[np.newaxis, v] for k, v in internal.items()})
    assert set(jetson) == set(comma) and 'action' not in jetson
    for k in comma:
      np.testing.assert_array_equal(jetson[k], comma[k])
    assert jetson['plan'].shape == (1, 33, 15) and jetson['hidden_state'].shape == (1, 512)
    action = get_action_from_model(jetson, log.ModelDataV2.Action(), 0.2, 0.2, 10.0, 0.0, 0.5)
    assert np.isfinite([action.desiredCurvature, action.desiredAcceleration]).all()
