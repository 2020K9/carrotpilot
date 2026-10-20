import numpy as np
import pytest

from openpilot.common.transformations.orientation import euler2quat, quat2euler, euler2rot, rot2euler, \
                                               rot2quat, quat2rot, ned_euler_from_ecef
from openpilot.common.transformations.transformations import euler2quat_single, euler2rot_single, \
                                                    rot2quat_single, quat2rot_single, \
                                                    ned_euler_from_ecef_single

rng = np.random.default_rng(0)
EULERS = rng.uniform(-np.pi, np.pi, (6, 3))
QUATS = euler2quat(EULERS)
ROTS = euler2rot(EULERS)

CASES = [
  (euler2quat, euler2quat_single, EULERS, (4,)),
  (euler2rot, euler2rot_single, EULERS, (3, 3)),
  (quat2rot, quat2rot_single, QUATS, (3, 3)),
  (rot2quat, rot2quat_single, ROTS, (4,)),
]


@pytest.mark.parametrize("wrapped,single,batch,out_shape", CASES)
def test_single_input_matches_single_function_and_shape(wrapped, single, batch, out_shape):
  inp = batch[0].copy()
  before = inp.copy()
  out = wrapped(inp)
  assert out.shape == out_shape
  np.testing.assert_array_equal(out, np.asarray(single(before)))
  assert inp.shape == before.shape
  np.testing.assert_array_equal(inp, before)


@pytest.mark.parametrize("wrapped,single,batch,out_shape", CASES)
def test_batch_list_and_noncontiguous_inputs(wrapped, single, batch, out_shape):
  expected = np.stack([np.asarray(single(x)) for x in batch])
  for inp in (batch, batch.tolist(), np.asfortranarray(batch), np.repeat(batch, 2, axis=0)[::2]):
    out = wrapped(inp)
    assert out.shape == (len(batch),) + out_shape
    np.testing.assert_array_equal(out, expected)
  np.testing.assert_array_equal(wrapped(batch[0].tolist()), expected[0])


def test_extra_args_and_empty_batch():
  ecef = np.array([[-2711076.55270557, -4259167.14692758, 3884579.87669935]])
  eul = np.array([[0.1, -0.2, 0.3]])
  np.testing.assert_array_equal(ned_euler_from_ecef(ecef[0], eul[0]), ned_euler_from_ecef_single(ecef[0], eul[0]))
  assert ned_euler_from_ecef(ecef[0], eul).shape == (1, 3)
  assert euler2quat(np.zeros((0, 3))).shape == (0, 4)
  assert quat2euler(QUATS).shape == rot2euler(ROTS).shape == (6, 3)
