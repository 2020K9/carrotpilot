"""K9HudNavdy 2 diagnostics: frames whose USB send call returned success, counted in the sending code itself.

Display only. A frame counts as sent only after the USB display send function returned without an error
(PNG/JPEG: one call per frame; H264: every chunk of one picture). Failed sends and frames the H264 queue
dropped are counted separately. time.monotonic of the sending process (comma or Jetson, whichever runs the HUD)."""
from __future__ import annotations

from collections import deque
import threading
import time

WINDOW_S = 2.0


class TxStats:
    def __init__(self):
        self._lock = threading.Lock()
        self.enabled = False      # set by main.py once a real USB output exists; tests / tools never count by accident
        self._sent: deque[float] = deque()
        self.started: float | None = None
        self.failed = 0

    def reset(self) -> None:
        with self._lock:
            self._sent.clear()
            self.started = None
            self.failed = 0

    def sent(self, count: int = 1, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        with self._lock:
            if self.started is None:
                self.started = now
            for _ in range(max(0, int(count))):
                self._sent.append(now)
            while self._sent and now - self._sent[0] > WINDOW_S:
                self._sent.popleft()

    def fail(self, count: int = 1, now: float | None = None) -> None:
        now = time.monotonic() if now is None else now
        with self._lock:
            if self.started is None:
                self.started = now
            self.failed += max(0, int(count))

    def fps(self, now: float | None = None) -> float | None:
        """frames per second over the last WINDOW_S s; None before anything was tried, 0.0 when sending stopped"""
        now = time.monotonic() if now is None else now
        with self._lock:
            if self.started is None:
                return None
            while self._sent and now - self._sent[0] > WINDOW_S:
                self._sent.popleft()
            return len(self._sent) / min(WINDOW_S, max(now - self.started, 0.5))

    def snapshot(self, now: float | None = None) -> tuple[float | None, int | None]:
        if not self.enabled:
            return None, None
        fps = self.fps(now)
        return fps, (self.failed if self.started is not None else None)


TX = TxStats()


def count_h264_pictures(data: bytes, tail: bytes = b"") -> tuple[int, bytes]:
    """number of pictures starting in data (VCL NAL 1/5 with first_mb_in_slice == 0), plus the tail to carry over
    so a start code split between two pipe reads is still seen once."""
    buf = tail + data
    count, i = 0, 0
    while True:
        i = buf.find(b"\x00\x00\x01", i)
        if i < 0:
            return count, buf[-2:]          # a start code may begin in these bytes
        if i + 4 >= len(buf):
            return count, buf[i:]           # header not complete yet: decide with the next read
        nal_type = buf[i + 3] & 0x1F
        if nal_type in (1, 5) and buf[i + 4] & 0x80:
            count += 1
        i += 3
