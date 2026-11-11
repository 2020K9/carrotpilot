"""Mode 3 ramp starts at 1.5 km/h; synthetic controller tests, not road validation."""
import pytest
from opendbc.car.hyundai import k9_creep as kc
from openpilot.selfdrive.car.tests.k9_rig import Rig, load_frozen_cc
from openpilot.selfdrive.car.tests.test_k9_stop_gap import RUNS
from openpilot.selfdrive.car.tests.test_k9_creep_mode import upd

BASE = "f7e77539c955ca29f0c337b2f4b6e21d852731f9"

@pytest.fixture(scope="module")
def baseline(tmp_path_factory):
  return load_frozen_cc(BASE, tmp_path_factory.mktemp("ramp15_base"), "ramp15_base")

@pytest.mark.parametrize("mode", [None, 0, 1, 2])
@pytest.mark.parametrize("seq", list(RUNS))
def test_other_modes_exact_can_bytes(monkeypatch, baseline, mode, seq):
  kw = {} if mode is None else {"K9StopMode": mode}
  old = RUNS[seq](Rig(monkeypatch, module=baseline, **kw))
  monkeypatch.undo()
  new = RUNS[seq](Rig(monkeypatch, **kw))
  assert len(new.can) > 400
  assert old.can == new.can and old.scc12 == new.scc12 and old.scc14 == new.scc14

@pytest.mark.parametrize("delta", [-1e-6, 0, 1e-6])
def test_mode3_boundary_inclusive_mode2_unchanged(delta):
  v = 1.5 / 3.6 + delta
  for mode, expected in [(kc.V12K9OGR_HALF, -kc.B17O_RAMP_STEP if delta <= 0 else 0), (kc.V12K9OGR, 0)]:
    c = kc.K9Creep(mode)
    upd(c, v_ego=v, v_ego_raw=v, a_target=-0.01, a_ego=0.0)
    assert c.b17o_ramp == expected


def test_mode3_ramp_cap_and_reset_at_new_speed():
  c = kc.K9Creep(kc.V12K9OGR_HALF)
  prev = 0.0
  for _ in range(400):
    upd(c, v_ego=0.4, v_ego_raw=0.4, a_target=-0.01, a_ego=0.0)
    assert -kc.B17O_RAMP_MAX <= c.b17o_ramp <= prev
    prev = c.b17o_ramp
  assert c.b17o_ramp == -kc.B17O_RAMP_MAX
  upd(c, stopping=False)
  assert c.b17o_ramp == 0 and c.b17o_hold is None


def test_mode3_no_extra_ramp_when_deceleration_sufficient():
  c = kc.K9Creep(kc.V12K9OGR_HALF)
  for _ in range(50):
    upd(c, v_ego=0.4, v_ego_raw=0.4, a_target=-0.01, a_ego=-2.0)
  assert c.b17o_ramp == 0
