"""gap0005: at a student -> stock switch modeld reruns the switch frame on the stock model (modeld.py), so the
seed leaves out the newest history frame (that frame is queued by the stock run itself)."""
import inspect
from collections import deque
from types import SimpleNamespace

from openpilot.selfdrive.modeld import modeld
from openpilot.selfdrive.modeld.stock_standby import StockStandby
from openpilot.selfdrive.modeld.tests.test_stock_standby import _frames, _stock_like


def test_drop_current_seeds_all_but_newest_frame():
  sb = StockStandby(_stock_like)
  src = SimpleNamespace(warp_history=deque(_frames(5), maxlen=5))
  m = sb.take(src, drop_current=True)
  q = m.input_queues['img_q'].numpy()
  assert sb.seeded == 4
  assert [int(q[i, 0, 0, 0]) for i in range(5)] == [0, 10, 20, 30, 40]
  assert len(src.warp_history) == 5  # source history itself untouched


def test_default_take_unchanged():
  sb = StockStandby(_stock_like)
  m = sb.take(SimpleNamespace(warp_history=deque(_frames(5), maxlen=5)))
  assert sb.seeded == 5 and int(m.input_queues['img_q'].numpy()[-1, 0, 0, 0]) == 50


def test_modeld_reruns_switch_frame_only_after_a_student_switch():
  src = inspect.getsource(modeld.main)
  assert "drop_current=True" in src
  i = src.index("switches_before = output_guard.switches")
  block = src[i:src.index("except ModelOutputError", i)]
  assert "output_guard.switches != switches_before" in block
  assert block.count("output_guard.check(model, model_output)") == 2
