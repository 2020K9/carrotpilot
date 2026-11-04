import json
from pathlib import Path

from openpilot.selfdrive.modeld.helpers import LIGHTSPEED_STUDENT_PKL, MODELS_DIR, make_internal_model, modeld_pkl_path

OPENPILOT = Path(__file__).resolve().parents[3]


class FakeModel:
  created: list = []

  def __init__(self, cam_w, cam_h, pkl_path=None, *, enable_shadow=True):
    if pkl_path == LIGHTSPEED_STUDENT_PKL and FakeModel.fail_student:
      raise RuntimeError("student load failed")
    self.pkl_path = pkl_path or modeld_pkl_path()
    self.state = []
    FakeModel.created.append(self)


def make(use_student, fail_student=False):
  FakeModel.created, FakeModel.fail_student = [], fail_student
  return make_internal_model(FakeModel, 1928, 1208, use_student, enable_shadow=False)


def test_off_uses_stock_model():
  model = make(False)
  assert model.pkl_path == MODELS_DIR / 'driving_tinygrad.pkl'
  assert len(FakeModel.created) == 1


def test_on_uses_student_model():
  model = make(True)
  assert model.pkl_path == LIGHTSPEED_STUDENT_PKL
  assert len(FakeModel.created) == 1


def test_on_student_failure_falls_back_with_new_state():
  model = make(True, fail_student=True)
  assert model.pkl_path == MODELS_DIR / 'driving_tinygrad.pkl'
  assert len(FakeModel.created) == 1 and model.state == []


def test_stock_path_does_not_depend_on_student_files():
  assert modeld_pkl_path() == MODELS_DIR / 'driving_tinygrad.pkl'


def test_param_default_off_and_exposed():
  keys = (OPENPILOT / 'common/params_keys.h').read_text()
  assert '{"LightspeedStudentModel", {PERSISTENT, BOOL, "0"}}' in keys
  settings = json.loads((OPENPILOT / 'selfdrive/carrot_settings.json').read_text())
  p = next(p for p in settings['params'] if p['name'] == 'LightspeedStudentModel')
  assert (p['min'], p['max'], p['default'], p['control']) == (0, 1, 0, 'toggle')
  system = next(c for c in settings['menu'] if c['id'] == 'SYSTEM')
  assert any('LightspeedStudentModel' in g.get("params", []) for g in system['groups'])
