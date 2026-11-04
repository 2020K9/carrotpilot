from types import SimpleNamespace
import sys

import pytest

from openpilot.selfdrive.modeld import signal_neural_shadow as worker


# Separate from the original test_signal_neural_shadow.py::test_auto_switch_releases_previous_process_and_final_child,
# which is kept verbatim from base and fails here because integ3 removed eGPU QCOM selection
# (automatic_backend always returns tracking_cpu). This variant injects the base selection rule
# (eGPU active and DM not needed -> qcom), still read from the mocked Params, so it only checks the
# run_auto switch / terminate / close lifecycle, not the product's mode selection.
@pytest.mark.parametrize('selection', ['auto', 'qcom'])
@pytest.mark.parametrize('change_dm', [False, True])
def test_run_auto_lifecycle_with_injected_selection(monkeypatch, tmp_path, selection, change_dm):
  import multiprocessing
  from openpilot.common import params
  modes = [False, False, True, True, False]
  position = [0]
  children = []
  class Child:
    exitcode = None
    pid = 123
    def __init__(self, *, target, args, daemon):
      self.args, self.alive, self.closed = args, False, False
      assert target == worker.run_selected and daemon
      children.append(self)
    def start(self):
      assert all(not c.alive for c in children if c is not self)
      self.alive = True
    def is_alive(self): return self.alive
    def terminate(self): self.alive = False
    def join(self, timeout): pass
    def close(self): self.closed = True
  def value(key):
    if key == 'UsbGpuActive': return True if change_dm else modes[position[0]]
    if key == 'DriverMonitoringEnabled': return not modes[position[0]] if change_dm else False
    return False
  monkeypatch.setattr(params, 'Params', lambda: SimpleNamespace(get_bool=value))
  monkeypatch.setattr(worker, 'requested', lambda _: position[0] < len(modes))
  monkeypatch.setattr(worker, 'neural_requested', lambda _: True)
  monkeypatch.setattr(worker, 'backend_requested', lambda _: selection)
  monkeypatch.setattr(worker.time, 'sleep', lambda _: position.__setitem__(0, position[0]+1))
  monkeypatch.setattr(multiprocessing, 'get_context', lambda _: SimpleNamespace(Process=Child))
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', SimpleNamespace(cloudlog=SimpleNamespace(event=lambda *a, **k: None)))
  monkeypatch.setattr(worker, 'automatic_backend', lambda p: 'qcom' if p.get_bool('UsbGpuActive') and not p.get_bool('DriverMonitoringEnabled') else 'tracking_cpu')
  worker.run_auto(tmp_path)
  assert [c.args[1] for c in children] == ['tracking_cpu', 'qcom', 'tracking_cpu']
  assert all(c.closed and not c.alive for c in children)
