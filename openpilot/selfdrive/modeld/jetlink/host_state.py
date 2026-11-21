"""K9JetHostState (실험중 "젯슨 상태값 CPU", 대감님 11/21 17:13~17:18): keep the Jetson model recurrent state on
the host (CPU) as before Carrot 9cfe86696 (10/21 "enable validated GPU recurrence"), instead of on the GPU.

0 = as now (GPU recurrence, the Jetson default). 1 = the Jetson loads the same engine with host-staged state.
The comma only ever sends the last COMMITTED value. A changed menu value is committed only offroad, or onroad
while stopped with driver assistance off (fresh carState standstill / vEgo 0, carControl.enabled off,
selfdriveState.active off). A reconnect while driving uses the last committed value, never the menu value.
No reboot of the comma or the Jetson: a committed change reloads the cached engine (a few seconds; the comma
uses its own model meanwhile, which is why it is limited to stopped / assistance off).
An older Jetson bundle without carrot_host_state_v1 is never asked (always as now).
"""
import time

PARAM = "K9JetHostState"
COMMITTED = "K9JetHostStateSet"
CAPABILITY = "carrot_host_state_v1"
POLL_SECONDS = 1.0


def supported(peer) -> bool:
  return isinstance(peer, dict) and peer.get(CAPABILITY) is True


def _read(params, key, fallback: bool) -> bool:
  try:
    value = params.get_int(key)
  except Exception:
    return fallback
  return value == 1


def offroad(params) -> bool:
  try:
    return params.get_bool("IsOffroad") and not params.get_bool("IsOnroad")
  except Exception:
    return False


class StoppedInactive:
  """Fresh stopped + assistance-off check; any missing / stale / error answer is "not safe"."""
  def __init__(self, sub_master=None):
    self._sm = sub_master

  def __call__(self) -> bool:
    try:
      if self._sm is None:
        from openpilot.cereal import messaging
        self._sm = messaging.SubMaster(["carState", "carControl", "selfdriveState"])
      sm = self._sm
      sm.update(0)
      if not (all(sm.valid.values()) and all(sm.alive.values())):
        return False
      cs = sm["carState"]
      return bool(cs.standstill and abs(cs.vEgo) < 0.01 and not sm["carControl"].enabled
                  and not sm["selfdriveState"].active)
    except Exception:
      return False


class HostStateLatch:
  def __init__(self, params, safe=None, clock=time.monotonic):
    self.params = params
    self.safe = safe if safe is not None else StoppedInactive()
    self.clock = clock
    self._next_poll = 0.0

  def committed(self) -> bool:
    return _read(self.params, COMMITTED, False)

  def wanted(self, committed: bool) -> bool:
    # A read error keeps the committed value (no switch on a bad read).
    return _read(self.params, PARAM, committed)

  def resolve(self) -> bool:
    """The value for this (re)connection: commit a changed menu value only when it is safe."""
    have = self.committed()
    want = self.wanted(have)
    if want != have and (offroad(self.params) or self.safe()):
      try:
        self.params.put_int(COMMITTED, int(want))
      except Exception:
        return have
      have = want
    return have

  def poll(self, current: bool) -> bool:
    """At most once a second while serving: True when a new value was committed (caller reconnects)."""
    now = self.clock()
    if now < self._next_poll:
      return False
    self._next_poll = now + POLL_SECONDS
    return self.resolve() != current


def apply(client, peer, latch) -> bool:
  value = latch.resolve() if supported(peer) else False
  client.host_state = bool(value)
  return client.host_state
