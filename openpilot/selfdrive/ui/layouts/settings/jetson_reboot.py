"""Settings > Device 'Jetson Reboot': request + short on-screen result (tools/jetlink/host_reboot.py)."""
import time

from openpilot.system.ui.lib.multilang import tr, tr_noop

SHOW_SECONDS = 10.
REASONS = {
  'onroad': tr_noop("onroad"),
  'not in P': tr_noop("not in P"),
  'moving': tr_noop("vehicle moving"),
  'driver assistance active': tr_noop("driver assistance active"),
  # Stop state (P, vEgo 0, assistance off) not confirmed by fresh signals: full sentence, no 'rejected:' prefix.
  'no vehicle signal': tr_noop("Vehicle stop state could not be confirmed, so the Jetson was not rebooted"),
  'old jetson bundle': tr_noop("Jetson needs the new bundle (next boot)"),
  'reboot command failed': tr_noop("reboot command failed"),
}


def _protocol():
  from tools.jetlink import host_reboot  # noqa: TID251 -- shared standalone host protocol
  return host_reboot


class JetsonReboot:
  def __init__(self):
    self.request_id = None
    self.requested_at = 0.
    self.final = None        # (state, reason, time)

  def request(self, params):
    # Same gate as the button: offroad only (IsOnroad=0).
    if params.get_bool("IsOnroad"):
      self.request_id, self.final = None, ('rejected', 'onroad', time.monotonic())
      return
    self.requested_at = time.monotonic()
    self.final = None
    self.request_id = _protocol().submit(self.requested_at)

  def state(self, now=None):
    now = time.monotonic() if now is None else now
    if self.request_id is not None and self.final is None:
      state, reason = _protocol().status(self.request_id, self.requested_at, now)
      if state != 'pending':
        self.final = (state, reason, now)
      else:
        return 'pending', ''
    if self.final is not None and now - self.final[2] < SHOW_SECONDS:
      return self.final[0], self.final[1]
    return None, ''

  def text(self, now=None):
    state, reason = self.state(now)
    if state is None:
      return ""
    if state == 'pending':
      return tr("sending...")
    if state == 'sent':
      return tr("sent, Jetson rebooting")
    if state == 'disconnected':
      return tr("Jetson not connected")
    if state == 'unsupported':
      return tr(REASONS['old jetson bundle'])
    if reason == 'no vehicle signal':
      return tr(REASONS[reason])
    return tr("rejected: {}").format(tr(REASONS[reason]) if reason in REASONS else reason)
