"""Stock model prepared before a runtime student -> stock switch, seeded with the last warped frames (stock_standby.py)."""
from collections import deque
from types import SimpleNamespace

import numpy as np

from openpilot.selfdrive.modeld.compile_modeld import make_warp_input_queues
from openpilot.selfdrive.modeld.modeld import ModelState
from openpilot.selfdrive.modeld.stock_standby import StockStandby

STOCK_SHAPES = {'img': (1, 12, 128, 256), 'big_img': (1, 12, 128, 256)}
FRAME_SKIP = 4


def _stock_like():
  q, npy = make_warp_input_queues(STOCK_SHAPES, FRAME_SKIP, 'CPU')
  m = SimpleNamespace(input_queues=q, npy=npy, warp_history=None)
  m.seed_image_history = lambda frames: ModelState.seed_image_history(m, frames)
  return m


def _frames(n, start=1):
  # warped output of one run: (2, 6, 128, 256) uint8, [0] road, [1] wide; value encodes (frame, camera)
  return [np.stack([np.full((6, 128, 256), 10 * i, np.uint8), np.full((6, 128, 256), 10 * i + 1, np.uint8)]) for i in range(start, start + n)]


def test_queue_length_is_frame_skip_plus_one():
  assert _stock_like().input_queues['img_q'].shape[0] == FRAME_SKIP + 1


def test_seed_fills_both_queues_oldest_first():
  m = _stock_like()
  assert m.seed_image_history(_frames(7)) == 5  # only the last 5 fit
  img, big = m.input_queues['img_q'].numpy(), m.input_queues['big_img_q'].numpy()
  assert [int(img[k, 0, 0, 0]) for k in range(5)] == [30, 40, 50, 60, 70]
  assert [int(big[k, 0, 0, 0]) for k in range(5)] == [31, 41, 51, 61, 71]


def test_seed_with_short_history_keeps_older_rows_zero():
  m = _stock_like()
  assert m.seed_image_history(_frames(2)) == 2
  img = m.input_queues['img_q'].numpy()
  assert not img[:3].any() and int(img[3, 0, 0, 0]) == 10 and int(img[4, 0, 0, 0]) == 20


def test_seed_refuses_mismatched_frames():
  m = _stock_like()
  bad = [np.zeros((2, 6, 64, 128), np.uint8)]
  assert m.seed_image_history(bad) == 0
  assert m.seed_image_history([f.astype(np.float32) for f in _frames(5)]) == 0
  assert not m.input_queues['img_q'].numpy().any()


def test_stateful_model_is_not_seeded():
  m = SimpleNamespace(input_queues={'packed_npy_inputs': None})
  assert ModelState.seed_image_history(m, _frames(5)) == 0


def test_take_uses_prepared_model_once_then_builds():
  built = []
  sb = StockStandby(lambda: built.append(_stock_like()) or built[-1])
  first = sb.prepare()
  assert len(built) == 1
  assert sb.take() is first and len(built) == 1
  second = sb.take()
  assert second is not first and len(built) == 2


def test_take_seeds_from_source_history():
  sb = StockStandby(_stock_like)
  sb.prepare()
  src = SimpleNamespace(warp_history=deque(_frames(5), maxlen=5))
  m = sb.take(src)
  assert sb.seeded == 5
  assert int(m.input_queues['img_q'].numpy()[-1, 0, 0, 0]) == 50


def test_take_from_stock_source_without_history_is_fresh():
  sb = StockStandby(_stock_like)
  m = sb.take(SimpleNamespace(warp_history=None))
  assert sb.seeded == 0 and not m.input_queues['img_q'].numpy().any()


def test_history_len_matches_stock_queue():
  assert StockStandby(_stock_like).history_len == FRAME_SKIP + 1


def test_keep_warp_history_and_reset_clears():
  m = SimpleNamespace(warp_history=None)
  ModelState.keep_warp_history(m, 5)
  assert isinstance(m.warp_history, deque) and m.warp_history.maxlen == 5
  m.warp_history.extend(_frames(3))
  m.prev_desire = np.ones(8, np.float32)
  m.npy = {}
  m.input_queues = {}
  ModelState.reset(m)
  assert len(m.warp_history) == 0 and not m.prev_desire.any()
  ModelState.keep_warp_history(m, 0)
  assert m.warp_history is None
