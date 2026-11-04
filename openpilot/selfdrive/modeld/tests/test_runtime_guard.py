import numpy as np
import pytest

from openpilot.selfdrive.modeld.helpers import LIGHTSPEED_STUDENT_PKL, make_internal_model
from openpilot.selfdrive.modeld.runtime_guard import ModelOutputError, RuntimeOutputGuard, nonfinite_count


class FakeParams:
  def __init__(self):
    self.store = {}

  def put(self, k, v):
    self.store[k] = v


class FakeModel:
  def __init__(self, cam_w, cam_h, pkl_path=None, **kw):
    self.pkl_path = pkl_path
    self.state = np.zeros(4, np.float32)  # stands in for recurrent / feature buffers
    self.bad = False

  def run(self, frame):
    self.state += 1
    v = np.float32('nan') if self.bad else np.float32(frame)
    return {'plan': np.full((1, 3), v, np.float32), 'hidden': self.state.copy()}


def _bad():
  f = FakeModel(10, 10)
  f.bad = True
  return f


def build(student=True):
  p = FakeParams()
  m = make_internal_model(FakeModel, 10, 10, student, p, use_direct=False)
  g = RuntimeOutputGuard(lambda: FakeModel(10, 10), params=p)
  return m, g, p


def test_nonfinite_count_nan_inf_and_ignores_private():
  assert nonfinite_count({'a': np.array([1., np.nan, np.inf, -np.inf]), '_x': np.array([np.nan]), 'i': np.array([1])}) == 3
  assert nonfinite_count({'a': np.zeros(3)}) == 0


@pytest.mark.parametrize('bad', [np.nan, np.inf, -np.inf])
def test_student_nonfinite_switches_to_fresh_stock_and_drops_frame(bad):
  m, g, p = build(True)
  assert m.lightspeed_status['model'] == 'lightspeed_student_b' and m.pkl_path == LIGHTSPEED_STUDENT_PKL
  m.run(0)
  m.run(1)
  out = m.run(2)
  out['plan'][0, 0] = bad
  m2, res = g.check(m, out)
  assert res is None                                   # not published
  assert m2 is not m and m2.pkl_path is None           # new stock instance
  assert np.all(m2.state == 0)                         # old recurrent state not carried over
  assert p.store['LightspeedModelStatus'] == {'model': 'stock', 'requested_student': True, 'fallback_reason': 'runtime nonfinite'}
  m3, res = g.check(m2, m2.run(3))
  assert m3 is m2 and res is not None and nonfinite_count(res) == 0 and res['plan'][0, 0] == 3
  assert g.switches == 1 and g.dropped == 1


def test_no_republish_of_previous_output():
  m, g, _ = build(True)
  published = []
  for f in range(8):
    out = m.run(f)
    if f == 4:
      out['plan'][:] = np.nan
    m, res = g.check(m, out)
    if res is not None:
      published.append((f, float(res['plan'][0, 0])))
  assert [f for f, _ in published] == [0, 1, 2, 3, 5, 6, 7]
  assert all(v == f for f, v in published)              # each published value belongs to its own frame


def test_stock_failure_retries_limited_then_raises():
  m, g, p = build(False)
  m.bad = True
  rebuilt = []
  g.rebuild_stock = lambda: rebuilt.append(1) or _bad()
  for _ in range(3):
    m, res = g.check(m, m.run(0))
    assert res is None
  with pytest.raises(ModelOutputError):
    g.check(m, m.run(0))
  assert len(rebuilt) == 3
  assert p.store['LightspeedModelStatus']['model'] == 'stock'


def test_stock_counter_resets_after_finite_frame():
  m, g, _ = build(False)
  for _ in range(10):
    m.bad = True
    m, res = g.check(m, m.run(0))
    assert res is None
    m, res = g.check(m, m.run(1))
    assert res is not None
  assert g.stock_failures == 0


def test_student_then_stock_failure_still_limited():
  m, g, _ = build(True)
  g.rebuild_stock = _bad
  m, res = g.check(m, {'plan': np.array([np.nan])})    # student -> stock (no retry consumed)
  for _ in range(3):
    m, res = g.check(m, m.run(0))
    assert res is None
  with pytest.raises(ModelOutputError):
    g.check(m, m.run(0))


def test_joining_wrapper_internal_model_replaced():
  class J:
    active = False

    def __init__(self, small):
      self.small = small
  inner, g, _ = build(True)
  j = J(inner)
  j2, res = g.check(j, {'plan': np.array([np.inf])})
  assert res is None and j2 is j and j.small is not inner and j.small.lightspeed_status['model'] == 'stock'


def test_finite_output_passthrough_identity():
  m, g, _ = build(True)
  out = m.run(1)
  m2, res = g.check(m, out)
  assert m2 is m and res is out
  assert g.check(m, None) == (m, None)


def test_load_failure_still_falls_back():
  class Failing(FakeModel):
    def __init__(self, w, h, pkl_path=None, **kw):
      if pkl_path == LIGHTSPEED_STUDENT_PKL:
        raise OSError('bad pkl')
      super().__init__(w, h, pkl_path)
  p = FakeParams()
  m = make_internal_model(Failing, 1, 1, True, p, use_direct=False)
  assert m.lightspeed_status['model'] == 'stock' and 'bad pkl' in m.lightspeed_status['fallback_reason']
