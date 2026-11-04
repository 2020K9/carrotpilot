import pytest

import openpilot.selfdrive.modeld.modeld as md
from openpilot.selfdrive.modeld.runtime_guard import ModelOutputError


class FakeParams:
  def __init__(self, student):
    self.store, self.student = {}, student

  def put(self, k, v):
    self.store[k] = v

  def get_bool(self, k):
    assert k == "LightspeedStudentModel"
    return self.student


class _Stop(Exception):
  pass


@pytest.mark.parametrize("student", [False, True])
def test_stock_failure_halts_without_exit(monkeypatch, student):
  sleeps = []

  def fake_sleep(s):
    sleeps.append(s)
    if len(sleeps) >= 3:
      raise _Stop
  monkeypatch.setattr(md.time, "sleep", fake_sleep)
  p = FakeParams(student)
  with pytest.raises(_Stop):  # only the test's sleep ends it: no return, no ModelOutputError, no SystemExit
    md.halt_after_model_failure(p, ModelOutputError("x"))
  assert sleeps == [1., 1., 1.]
  st = p.store["LightspeedModelStatus"]
  assert st["model"] == "stock" and st["requested_student"] is student and "halted" in st["fallback_reason"]
