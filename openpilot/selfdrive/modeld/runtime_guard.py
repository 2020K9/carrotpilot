"""Runtime NaN/Inf guard for modeld outputs (checked right after model.run, before publish).

Policy:
  * A non-finite output frame is never published (modeld publishes nothing for that frame;
    diagnostics records published=0). The previous output is never re-sent as a new one.
  * Student / direct model: replaced by a freshly built stock model (new ModelState: the model's
    internal state - recurrent/feature buffers, input queues - is reset). LightspeedModelStatus -> stock,
    reason "runtime nonfinite".
  * Not reset: modeld's prev_action (and other main-loop state outside the model) is kept, so the
    first published actions after a switch are smoothed from the last finite action of the old model
    (desiredAcceleration / desiredVelocity, tau LONG_SMOOTH_SECONDS 0.3 s; desiredCurvature is held
    from prev_action below MIN_LAT_CONTROL_SPEED).
  * Stock model: rebuilt fresh at most MAX_STOCK_RETRIES times in a row. One more non-finite frame
    after that raises ModelOutputError, so modeld exits and the existing process-not-running /
    commIssue path reports it. A finite frame resets the counter.
"""
import numpy as np

MAX_STOCK_RETRIES = 3


class ModelOutputError(RuntimeError):
  pass


def nonfinite_count(outputs) -> int:
  n = 0
  for k, v in outputs.items():
    if k.startswith('_'):
      continue
    a = np.asarray(v)
    if a.dtype.kind in 'fc':
      n += int(a.size - np.count_nonzero(np.isfinite(a)))
  return n


class RuntimeOutputGuard:
  def __init__(self, rebuild_stock, params=None, log=None, max_stock_retries=MAX_STOCK_RETRIES):
    self.rebuild_stock = rebuild_stock  # () -> fresh stock model
    self.params = params
    self.log = log
    self.max_stock_retries = max_stock_retries
    self.stock_failures = 0
    self.switches = 0
    self.dropped = 0

  @staticmethod
  def _internal(model):
    # JoiningModel keeps the internal model in .small
    return model.small if hasattr(model, 'small') else model

  def _is_stock(self, model):
    st = getattr(self._internal(model), 'lightspeed_status', None)
    return st is None or st.get('model') == 'stock'

  def _replace(self, model, fresh):
    if hasattr(model, 'small'):
      model.small = fresh
      return model
    return fresh

  def check(self, model, outputs):
    """Returns (model, outputs_or_None). None means: do not publish this frame."""
    if outputs is None:
      return model, None
    bad = nonfinite_count(outputs)
    if bad == 0:
      self.stock_failures = 0
      return model, outputs
    self.dropped += 1
    remote = bool(getattr(model, 'active', False))
    student = not remote and not self._is_stock(model)
    if self.log:
      self.log(f"modeld nonfinite output: {bad} values, student={student}, remote={remote}, stock_failures={self.stock_failures}")
    if not student:
      if self.stock_failures >= self.max_stock_retries:
        raise ModelOutputError(f"model output non-finite ({bad} values) after {self.stock_failures} fresh stock rebuilds")
      self.stock_failures += 1
    prev = getattr(self._internal(model), 'lightspeed_status', None) or {}
    fresh = self.rebuild_stock()
    status = {"model": "stock", "requested_student": bool(prev.get("requested_student", False)),
              "fallback_reason": "runtime nonfinite" if student else "runtime nonfinite stock rebuild"}
    if prev.get("requested_direct"):
      status["requested_direct"] = True
    fresh.lightspeed_status = status
    if student:
      self.switches += 1
    if self.params is not None:
      self.params.put("LightspeedModelStatus", status)
    return self._replace(model, fresh), None
