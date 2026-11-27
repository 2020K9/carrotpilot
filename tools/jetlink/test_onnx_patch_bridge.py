"""onnx_patch: an Identity bridge between an image input and its head Cast
(Jcd210 f73a9e53 export) is patched the same way as a direct Cast."""
from pathlib import Path
import sys

import numpy as np
import onnx
from onnx import TensorProto as T, helper as H, numpy_helper
from onnx.reference import ReferenceEvaluator
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'third_party/jetlink')]
from jetlink.onnx_patch import needs_patch, patch_uint8_inputs  # noqa: E402

SHAPE = [1, 2, 2, 2]


def _model(bridge: bool, chain: int = 1) -> onnx.ModelProto:
  nodes = []
  heads = {}
  for name in ('img', 'big_img'):
    src = name
    if bridge:
      for i in range(chain):
        out = f'vision/{name}' if i == chain - 1 else f'bridge{i}/{name}'
        nodes.append(H.make_node('Identity', [src], [out], name=f'bridge{i}_{name}'))
        src = out
    heads[name] = f'{name}__fp16'
    nodes.append(H.make_node('Cast', [src], [heads[name]], to=T.FLOAT16, name=f'cast_{name}'))
  nodes.append(H.make_node('Concat', [heads['img'], heads['big_img']], ['cat'], axis=1))
  nodes.append(H.make_node('Mul', ['cat', 'scale'], ['outputs']))
  scale = numpy_helper.from_array(np.array(0.5, dtype=np.float16), 'scale')
  inputs = [H.make_tensor_value_info(n, T.UINT8, SHAPE) for n in ('img', 'big_img')]
  value_info = [H.make_tensor_value_info(n.output[0], T.UINT8, SHAPE) for n in nodes if n.op_type == 'Identity']
  g = H.make_graph(nodes, 'm', inputs, [H.make_tensor_value_info('outputs', T.FLOAT16, [1, 4, 2, 2])],
                   initializer=[scale], value_info=value_info)
  return H.make_model(g, opset_imports=[H.make_opsetid('', 20)])


def _feeds(dtype):
  rng = np.random.default_rng(0)
  return {n: rng.integers(0, 256, SHAPE).astype(np.uint8).astype(dtype) for n in ('img', 'big_img')}


@pytest.mark.parametrize('chain', [1, 2])
def test_identity_bridged_head_cast_is_dropped(chain):
  original = _model(bridge=True, chain=chain)
  want = ReferenceEvaluator(original).run(None, _feeds(np.uint8))[0]
  patched = _model(bridge=True, chain=chain)
  assert needs_patch(patched)
  patch_uint8_inputs(patched)
  onnx.checker.check_model(patched, full_check=True)
  assert not any(n.op_type == 'Cast' for n in patched.graph.node)
  assert all(vi.type.tensor_type.elem_type == T.FLOAT16 for vi in [*patched.graph.input, *patched.graph.value_info])
  got = ReferenceEvaluator(patched).run(None, _feeds(np.float16))[0]
  np.testing.assert_array_equal(got, want)


def test_direct_cast_unchanged_by_bridge_support():
  """A model without bridges patches to the same bytes as before."""
  m = _model(bridge=False)
  patch_uint8_inputs(m)
  assert [n.op_type for n in m.graph.node] == ['Concat', 'Mul']
  assert list(m.graph.node[0].input) == ['img', 'big_img']


def test_identity_not_from_image_input_is_not_a_head():
  m = _model(bridge=False)
  for n in list(m.graph.node):
    if n.op_type == 'Cast':
      m.graph.node.remove(n)
  m.graph.node[0].input[0] = 'img'
  m.graph.node[0].input[1] = 'other'
  m.graph.node.insert(0, H.make_node('Identity', ['scale'], ['other_u8']))
  m.graph.node.insert(1, H.make_node('Cast', ['other_u8'], ['other'], to=T.FLOAT16))
  with pytest.raises(ValueError, match='head Cast'):
    patch_uint8_inputs(m)
