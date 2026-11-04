import numpy as np
from tinygrad import Device, Tensor, dtypes
from tinygrad.engine.jit import TinyJit

from openpilot.selfdrive.modeld.compile_modeld import (POLICY_INPUTS, is_stateful, make_input_queues, make_run_policy,
                                                       make_warp_input_queues, policy_input_keys)

SHAPES = {
  'new_img': (2, 6, 4, 8),
  'desire': (8,),
  'traffic_convention': (1, 2),
  'action_t': (1, 2),
  'state_img_q': (2, 3, 6, 4, 8),
  'state_desire_q': (5, 1, 8),
  'state_feat_q': (4, 1, 3),
}


def fake_runner(inputs):
  # shifts each state by one step and returns a sum that depends on every input
  img_q = inputs['state_img_q'][:, 1:].cat(inputs['new_img'].unsqueeze(1), dim=1)
  desire_q = inputs['state_desire_q'][1:].cat(inputs['desire'].reshape(1, 1, 8), dim=0)
  feat_q = inputs['state_feat_q'] + 1
  total = img_q.cast('float32').sum() + desire_q.sum() + inputs['traffic_convention'].sum() + inputs['action_t'].sum() + feat_q.sum()
  return {'outputs': total.reshape(1, 1).cast('float16'), 'next_state_img_q': img_q,
          'next_state_desire_q': desire_q, 'next_state_feat_q': feat_q}


def run(jit, queues, warped):
  return jit(**{k: queues[k] for k in policy_input_keys(SHAPES)}, warped=warped)


def test_stock_keys_unchanged():
  assert not is_stateful({'img': (1, 12, 128, 256)})
  assert policy_input_keys({'img': (1, 12, 128, 256)}) == POLICY_INPUTS


def test_stateful_queues():
  queues, npy = make_input_queues(SHAPES, 4, device=Device.DEFAULT)
  assert set(queues) == {'tfm', 'big_tfm', 'state_img_q', 'state_desire_q', 'state_feat_q', 'packed_npy_inputs'}
  assert queues['state_img_q'].dtype == dtypes.uint8 and queues['state_feat_q'].dtype == dtypes.float32
  assert set(npy) == {'tfm', 'big_tfm', 'desire', 'traffic_convention', 'action_t'}
  assert queues['packed_npy_inputs'].shape == (12,)
  warp_queues, _ = make_warp_input_queues(SHAPES, 4, device=Device.DEFAULT)
  assert set(warp_queues) == {'tfm', 'big_tfm'}


def test_state_is_carried_between_runs():
  queues, npy = make_input_queues(SHAPES, 4, device=Device.DEFAULT)
  jit = TinyJit(make_run_policy(fake_runner, {'input_shapes': SHAPES}, 4), prune=True)
  ref_img = np.zeros(SHAPES['state_img_q'], dtype=np.uint8)
  ref_feat = np.zeros(SHAPES['state_feat_q'], dtype=np.float32)
  for i in range(5):
    npy['desire'][:] = i
    npy['action_t'][:] = 0.5
    frame = np.full(SHAPES['new_img'], i + 1, dtype=np.uint8)
    out = run(jit, queues, Tensor(frame).realize())[0].numpy()
    ref_img = np.concatenate([ref_img[:, 1:], frame[:, None]], axis=1)
    ref_feat = ref_feat + 1
    np.testing.assert_array_equal(queues['state_img_q'].numpy(), ref_img)
    np.testing.assert_array_equal(queues['state_feat_q'].numpy(), ref_feat)
    assert queues['state_desire_q'].numpy()[-1, 0, 0] == i
    assert out.dtype == np.float32 and out.shape == (1, 1)
