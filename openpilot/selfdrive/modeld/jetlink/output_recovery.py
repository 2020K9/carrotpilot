"""K9JetErrDelay (실험중 '젯슨 에러지연'): survive ONE non-finite Jetson frame, Carrot eGPU rules on the Jetlink path.

대감님 2026-11-21 11:09 "강제변환식말고 ... 이전기록으로 유지". Same rule set as Carrot's eGPU OutputRecovery
(dabe4f1d, docs/egpu_single_output_recovery_20261014.md, MAX_HOLD_SECONDS 100 ms):

- One frame whose driving output or next recurrent state is NaN/Inf (Jetson status 4) is dropped. modeld publishes
  nothing for it and never relabels an old prediction; consumers keep the previous timestamped modelV2 for one more
  cycle (one missing publication, ~100 ms between results). The model is not changed.
- The Jetson restores the recurrent state of the previous valid frame (protocol Flag.HOLD_STATE / STATE_HELD), so a
  broken state is never fed back. A Jetson bundle without that capability ('carrot_hold_state_v1' in its HELLO) is
  never held.
- Everything else is exactly the existing path (link closed, FAULT, commIssue, rejoin after 5 s): the setting off,
  two status 4 in a row, a second status 4 within 20 frames (1 s), no previous valid result on this connection or
  one older than 100 ms, a Jetson that did not restore. Device / time-out / protocol / shape failures are never
  swallowed. "Never silently switch under control": switching to the comma model still raises commIssue.
- The fault history (recent status-4 times and frames, the in-a-row count) is recorded for every status 4 whatever
  the setting says, and survives setting changes and reconnects for the life of the process. The 20-frame rule
  compares frame ids of the same connection (modeld's frame counter stops while the comma model drives); the 1 s
  rule compares times across connections too.

jetlinkd (owner of the USB link) makes the decision; modeld enforces the same bounds again on its side (Carrot's
"parent and worker independently enforce one skipped result and the age bound").
"""
from collections import deque
import time

PARAM = 'K9JetErrDelay'
MAX_HOLD_SECONDS = .100      # Carrot eGPU egpu_output_recovery.MAX_HOLD_SECONDS
WINDOW_FRAMES = 20           # a second status 4 this many frames after the previous one falls back (20 = 1 s)
WINDOW_SECONDS = 1.0
HISTORY = 16                 # recent status-4 (time, frame) pairs kept
PARAM_READ_SECONDS = .5      # the setting is re-read this often: on / off reaches the next status 4 within 0.5 s
# modeld side of the local IPC (jetlinkd <-> modeld): offered in jetlinkd's hello only when the Jetson can restore its
# state; request flag bit 3 (bits 0..2 = reset / compact reply / deferred map) = "the setting is on for this frame";
# a held frame is answered with the REPLY header alone (frame id + timings, no outputs).
IPC_CAPABILITY = 'carrot_err_delay_ipc_v1'
IPC_FLAG = 8


_PROCESS = None


def process_recovery():
  """jetlinkd's one history for the life of the process (setting changes and reconnects keep it)."""
  global _PROCESS
  if _PROCESS is None:
    _PROCESS = JetlinkOutputRecovery()
  return _PROCESS


def frames_between(a: int, b: int) -> int:
  """frame ids are 32-bit counters (modeld JoiningModel.frame)"""
  return (int(b) - int(a)) & 0xFFFFFFFF


class JetlinkOutputRecovery:
  def __init__(self, clock=time.monotonic, max_hold=MAX_HOLD_SECONDS):
    self.clock = clock
    self.max_hold = max_hold
    self.last_good = None           # monotonic time of the last valid Jetson result on this connection
    self.last_good_frame = None
    self.faults = deque(maxlen=HISTORY)   # (time, frame, connection) of every status 4, held or not
    self.in_a_row = 0               # status 4 since the last valid result
    self.held = 0                   # frames held since this process started
    self.connection = 0
    self.last_reason = ''

  def connected(self) -> None:
    """A new connection: no previous valid result yet. The fault history is kept."""
    self.last_good = self.last_good_frame = None
    self.connection += 1

  def ok(self, frame: int, now: float | None = None) -> None:
    self.last_good = self.clock() if now is None else now
    self.last_good_frame = frame
    self.in_a_row = 0

  def fault(self, frame: int, enabled: bool, restored: bool, now: float | None = None) -> bool:
    """Record one status 4 and decide: True = hold this frame, False = the existing fallback.
    self.last_reason says why (for the one log line per frame)."""
    now = self.clock() if now is None else now
    previous = self.faults[-1] if self.faults else None
    self.faults.append((now, frame, self.connection))
    self.in_a_row += 1
    if not enabled:
      reason = 'setting off'
    elif not restored:
      reason = 'Jetson did not restore the previous state'
    elif self.in_a_row > 1:
      reason = f'{self.in_a_row} in a row'
    elif previous is not None and ((previous[2] == self.connection and 0 < frames_between(previous[1], frame) <= WINDOW_FRAMES)
                                   or 0 <= now - previous[0] < WINDOW_SECONDS):
      reason = f'second status 4 {frames_between(previous[1], frame)} frames / {now - previous[0]:.2f} s after the last'
    elif self.last_good is None:
      reason = 'no valid result yet on this connection'
    elif not 0 <= now - self.last_good <= self.max_hold:
      reason = f'previous result {1000 * (now - self.last_good):.0f} ms old'
    else:
      self.held += 1
      self.last_reason = 'held'
      return True
    self.last_reason = reason
    return False
