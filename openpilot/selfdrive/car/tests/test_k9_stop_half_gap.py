"""Mode 3: half of mode 2's gap addition, not half of total braking. No road validation."""
import importlib.util
import json
import random

import numpy as np
import pytest

from opendbc.car.hyundai import k9_creep as kc
from openpilot.selfdrive.car import k9_stop_mode_migration as mig
from openpilot.selfdrive.car.tests.k9_rig import ROOT, Rig, LCS, _git_show, load_frozen_cc
from openpilot.selfdrive.car.tests.test_k9_stop_gap import RUNS
from openpilot.selfdrive.car.tests.test_k9_creep_mode import upd
from openpilot.selfdrive.car.tests.test_k9_stop_v12 import FakeParams, _random_inputs

BASE = 'f8532bed73e7ee914e4bd6197a396f887ab69ee5'


@pytest.fixture(scope='module')
def baseline(tmp_path_factory):
  return load_frozen_cc(BASE, tmp_path_factory.mktemp('half_gap_base'), 'half_gap_base')


@pytest.mark.parametrize('seq', list(RUNS))
@pytest.mark.parametrize('mode', [None, 0, 1, 2])
def test_existing_modes_exact_can_bytes(monkeypatch, baseline, mode, seq):
  kw = {} if mode is None else {'K9StopMode': mode}
  old = RUNS[seq](Rig(monkeypatch, module=baseline, **kw))
  monkeypatch.undo()
  new = RUNS[seq](Rig(monkeypatch, **kw))
  assert len(new.can) > 400
  assert old.can == new.can and old.scc12 == new.scc12 and old.scc14 == new.scc14


@pytest.mark.parametrize('v', sorted(set(kc.B17O_GAP_BP + [0.15, 0.3, 0.4, kc.B17O_RAMP_V - 1e-6, kc.B17O_RAMP_V, kc.B17O_RAMP_V + 1e-6, 0.5, 0.65, 0.8])))
def test_exact_half_gap_at_bins_and_interpolation(v):
  half = kc.K9Creep(kc.V12K9OGR_HALF)
  out = half._b17o(-0.01, True, v, v, False, False, 0.0)
  base = -float(np.interp(v, kc.B17O_TAB_BP, kc.B17O_TAB_V))
  gap = float(np.interp(v, kc.B17O_GAP_BP, kc.B17O_GAP_A))
  expected_gap = 0.5 * gap if v <= kc.B17O_RAMP_V else 0.0
  assert out == pytest.approx(base - expected_gap, abs=1e-12)
  assert half.gap_scale == 0.5 and kc.K9Creep(kc.V12K9OGR).gap_scale == 1.0


def test_only_gap_scale_differs_from_mode2():
  one, two = kc.K9Creep(kc.V12K9OGR), kc.K9Creep(kc.V12K9OGR_HALF)
  for key in one.__dict__:
    if key not in ('mode', 'gap_scale', 'ref', 'sh'):
      assert one.__dict__[key] == two.__dict__[key], key
  assert type(one.ref) is type(two.ref) and one.sh.mode == two.sh.mode == kc.V12K9O


@pytest.mark.parametrize('guard', [dict(gas=True), dict(brake=True), dict(override=True), dict(can_valid=False), dict(long_active=False)])
def test_pedals_fault_and_disengage_bypass_and_reset(guard):
  c = kc.K9Creep(kc.V12K9OGR_HALF)
  for _ in range(25):
    upd(c, v_ego=0.2, v_ego_raw=0.2, a_target=-0.05)
  assert c.b17g_f > 0
  assert upd(c, accel=-1.7, stopping=False, **guard) == (-1.7, False)
  assert c.b17g_f == 0 and c.b17o_ramp == 0 and not c.active


@pytest.mark.parametrize('kw', [dict(a_target=-2.0), dict(lead_visible=True, lead_distance=1.0), dict(esp_standstill=True, v_ego=0.0, v_ego_raw=0.0)])
def test_emergency_lead_and_standstill_latch(kw):
  c = kc.K9Creep(kc.V12K9OGR_HALF)
  for _ in range(20):
    out = upd(c, **kw)
  assert out[1] is True and c.latched
  assert out[0] <= -0.5


def test_ramp_threshold_and_steps_retained():
  c = kc.K9Creep(kc.V12K9OGR_HALF)
  upd(c, v_ego=0.3, v_ego_raw=0.3, a_target=-0.01, a_ego=0.0)
  assert c.b17o_ramp == 0
  upd(c, v_ego=0.27, v_ego_raw=0.27, a_target=-0.01, a_ego=0.0)
  assert c.b17o_ramp == -kc.B17O_RAMP_STEP
  assert c.b17o_hold is not None
  upd(c, stopping=False)
  assert c.b17o_ramp == 0 and c.b17g_f == 0 and c.b17o_hold is None


def test_random_inputs_no_weaker_than_original_table():
  c, ref = kc.K9Creep(kc.V12K9OGR_HALF), kc.K9Creep(kc.V12K9O)
  for kw, _ in _random_inputs(random.Random(1108), 3000):
    a, stop = c.update(**kw)
    r, _ = ref.update(**kw)
    assert a <= r + 1e-12
    assert isinstance(stop, bool)


def test_mode3_live_switch_and_web_menu(monkeypatch):
  r = Rig(monkeypatch, K9StopMode=2)
  for _ in range(51):
    r.step(-0.3, LCS.stopping, 0.5)
  r.params.values['K9StopMode'] = 3
  for _ in range(51):
    r.step(-0.3, LCS.stopping, 0.5)
  assert r.cc.k9_creep.mode == kc.V12K9OGR_HALF
  item = next(x for x in json.loads((ROOT/'openpilot/selfdrive/carrot_settings.json').read_text())['params'] if x['name'] == 'K9StopMode')
  assert item['choices'] == [0, 1, 2, 3] and item['max'] == 3 and item['default'] == 0
  assert all(len(item['options'][loc]) == 4 for loc in ('ko', 'en', 'zh'))
  assert '50%' in item['descr'] and '50%' in item['edescr']


@pytest.mark.parametrize('old,want', [(0, 0), (1, 1), (2, 2), (3, 1), (4, 2)])
def test_marker11_migration_and_write_failure_recovery(old, want):
  class Cut(FakeParams):
    def __init__(self, n, **v):
      super().__init__(**v)
      self.n = n

    def put_int(self, key, value):
      if self.n == 0:
        raise OSError('power loss')
      self.n -= 1
      super().put_int(key, value)

  for n in range(4):
    p = Cut(n, K9StopMode=old, K9StopModeVer=11)
    mig.migrate_k9_stop_mode(p)
    if p.v['K9StopModeVer'] == 12:
      assert p.v['K9StopMode'] == want
    q = FakeParams(**p.v)
    mig.migrate_k9_stop_mode(q)
    assert q.v == {'K9StopMode': want, 'K9StopModeVer': 12}
    assert mig.migrate_k9_stop_mode(q) is False
  q = FakeParams(K9StopMode=3, K9StopModeVer=12)
  assert mig.migrate_k9_stop_mode(q) is False
  assert q.v == {'K9StopMode': 3, 'K9StopModeVer': 12}


def test_all_legacy_selections_match_baseline(tmp_path):
  p = tmp_path/'old_mig.py'
  p.write_text(_git_show(BASE, 'openpilot/selfdrive/car/k9_stop_mode_migration.py'))
  spec = importlib.util.spec_from_file_location('old_half_gap_mig', p)
  old = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(old)
  for marker in [None, *range(2, 12), *range(50, 54), *range(60, 65), *range(70, 76), *range(80, 86), *range(90, 96), *range(100, 106)]:
    for value in [None, *range(10), -1, 'x', True]:
      data = {'K9StopMode': value, 'K9StopModeVer': marker}
      a, b = FakeParams(**data), FakeParams(**data)
      for _ in range(2):
        old.migrate_k9_stop_mode(a)
        mig.migrate_k9_stop_mode(b)
      assert a.v['K9StopMode'] == b.v['K9StopMode'], (marker, value, a.v, b.v)
      assert b.v['K9StopModeVer'] == 12


@pytest.mark.parametrize('camera_scc', [False, True])
def test_new_mode_does_not_apply_to_other_vehicle_configuration(monkeypatch, camera_scc):
  from opendbc.car.hyundai.values import CAR, HyundaiFlags
  from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import K9_FLAGS, make_cp
  from openpilot.selfdrive.car.tests.k9_rig import gentle_sequence
  cp = make_cp(fingerprint=CAR.KIA_K7) if camera_scc else make_cp(flags=int(K9_FLAGS & ~HyundaiFlags.CAMERA_SCC))
  zero = gentle_sequence(Rig(monkeypatch, cp=cp, K9StopMode=0))
  monkeypatch.undo()
  three = gentle_sequence(Rig(monkeypatch, cp=cp, K9StopMode=3))
  assert three.can == zero.can and three.cc.k9_creep is None


def test_half_gap_speed_crossings_do_not_change_mode2():
  half, full = kc.K9Creep(kc.V12K9OGR_HALF), kc.K9Creep(kc.V12K9OGR)
  for v in (0.8, 0.5, kc.B17O_RAMP_V, 0.3, 0.5, 0.4):
    base = -float(np.interp(v, kc.B17O_TAB_BP, kc.B17O_TAB_V))
    gap = float(np.interp(v, kc.B17O_GAP_BP, kc.B17O_GAP_A))
    h = half._b17o(-0.01, True, v, v, False, False, 0.0)
    f = full._b17o(-0.01, True, v, v, False, False, 0.0)
    assert h == pytest.approx(base - (0.5 * gap if v <= kc.B17O_RAMP_V else 0.0), abs=1e-12)
    assert f == pytest.approx(base - gap, abs=1e-12)
