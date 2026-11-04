"""Restored from deleted test_generic_model_runtime.py / test_precompiled_artifact.py (4562d6a3).

model_metadata moved unchanged to artifact_metadata.py; bodies, inputs and thresholds are verbatim.
"""
import copy
import json
from pathlib import Path

import numpy as np
import pytest

from openpilot.selfdrive.modeld.artifact_metadata import model_metadata
from openpilot.selfdrive.modeld.parse_model_outputs import Parser


@pytest.fixture
def artifact():
  # Metadata extracted without executing the official PR #38932 pickle.
  return json.loads((Path(__file__).parent / 'fixtures/cinque_v3_metadata.json').read_text())


def test_official_v3_output_contract_and_internal_history(artifact):
  checkpoint, slices, pairs, count = model_metadata(artifact)
  assert '/f78ed37d-afad-4dbc-8050-40ea885eedde/12864' in checkpoint
  assert pairs == {name: f'next_{name}' for name in ('state_img_q', 'state_desire_q', 'state_feat_q')}
  assert count == 18452
  raw = np.zeros((1, count), np.float32)
  outputs = Parser().parse_outputs({name: raw[:, section].copy() for name, section in slices.items()})
  assert outputs['plan'].shape == (1, 33, 15)
  assert np.isfinite(outputs['action']).all()


@pytest.mark.parametrize('change', ['state_shape', 'state_dtype', 'image_dtype', 'unknown_input', 'device'])
def test_reject_incompatible_generic_artifact(artifact, change):
  if change == 'state_shape':
    artifact['output_specs']['next_state_feat_q'][0][0] += 1
  elif change == 'state_dtype':
    artifact['output_specs']['next_state_feat_q'][1] = 'float16'
  elif change == 'image_dtype':
    artifact['input_specs']['new_img'][1] = 'float32'
  elif change == 'unknown_input':
    artifact['input_specs']['unknown'] = copy.deepcopy(artifact['input_specs']['desire'])
  else:
    artifact['input_specs']['new_img'][2] = 'QCOM'
  with pytest.raises(ValueError):
    model_metadata(artifact)


@pytest.mark.parametrize('fixture,checkpoint_id', [('mdm2_metadata.json', '870a4823-d2ec-4b9a-8015-b947e9901fc7'),
                                                  ('mdm_retrained_metadata.json', '8d736683-fed4-4e48-96fd-eae5d9d5bc56'),
                                                  ('md_v3_metadata.json', '1365126a-e71e-4778-be00-8ab92dc82c09')])
@pytest.mark.parametrize('previous_fixture', ['cinque_v3_metadata.json', 'mdm_v1_metadata.json'])
def test_mdm_contract_matches_parser_and_preserves_history(previous_fixture, fixture, checkpoint_id):
  fixtures = Path(__file__).parent / 'fixtures'
  mdm = json.loads((fixtures / fixture).read_text())
  previous = json.loads((fixtures / previous_fixture).read_text())
  checkpoint, slices, pairs, count = model_metadata(mdm)
  assert f'/{checkpoint_id}/12864' in checkpoint
  def normalize(specs):
    return {name: (shape, np.dtype(dtype), device) for name, (shape, dtype, device) in specs.items()}
  for kind in ('input_specs', 'output_specs'):
    assert normalize(mdm[kind]) == normalize(previous[kind])
  assert slices == model_metadata(previous)[1]
  assert set(pairs) == {'state_img_q', 'state_desire_q', 'state_feat_q'}
  raw = np.zeros((1, count), np.float32)
  parsed = Parser().parse_outputs({name: raw[:, section].copy() for name, section in slices.items()})
  assert parsed['plan'].shape == (1, 33, 15)
  assert np.isfinite(parsed['action']).all()
