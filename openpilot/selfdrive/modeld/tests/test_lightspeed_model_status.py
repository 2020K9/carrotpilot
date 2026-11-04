from openpilot.selfdrive.modeld.helpers import LIGHTSPEED_STUDENT_PKL, MODELS_DIR, make_internal_model
from openpilot.selfdrive.ui.ui_state import lightspeed_badge


class FakeParams:
  def __init__(self):
    self.store = {}

  def put(self, k, v):
    self.store[k] = v


class FakeModel:
  fail_student = False

  def __init__(self, cam_w, cam_h, pkl_path=None, *, enable_shadow=True):
    if pkl_path == LIGHTSPEED_STUDENT_PKL and FakeModel.fail_student:
      raise FileNotFoundError("no student pkl")
    self.pkl_path = pkl_path or MODELS_DIR / 'driving_tinygrad.pkl'


def status(use_student, fail_student=False):
  FakeModel.fail_student = fail_student
  params = FakeParams()
  make_internal_model(FakeModel, 1928, 1208, use_student, params, enable_shadow=False)
  return params.store["LightspeedModelStatus"]


def test_status_student_loaded():
  s = status(True)
  assert s == {"model": "lightspeed_student_b", "requested_student": True, "fallback_reason": None}
  assert lightspeed_badge(s) == ("LS-B", "active")


def test_status_fallback_records_reason():
  s = status(True, fail_student=True)
  assert s["model"] == "stock" and s["requested_student"] and "FileNotFoundError: no student pkl" == s["fallback_reason"]
  assert lightspeed_badge(s) == ("LS>STOCK", "error")


def test_status_off_is_stock_without_badge():
  s = status(False)
  assert s == {"model": "stock", "requested_student": False, "fallback_reason": None}
  assert lightspeed_badge(s) is None and lightspeed_badge(None) is None
