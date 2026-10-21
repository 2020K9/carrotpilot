"""Diagnostics-only GPU (kgsl) sampling and run-interval capture for runtimeTiming.

Nothing here may influence model inputs/outputs or publishing: every public
method swallows all exceptions and the sysfs reads are tiny, rate limited to
1 Hz and performed on the caller's thread (no background thread).
"""
from collections import deque
import glob
import time

KGSL_DIR = '/sys/class/kgsl/kgsl-3d0'


def _read(path):
  try:
    with open(path) as f:
      return f.read(256)
  except Exception:
    return None


def parse_gpubusy(text):
  """'  used  total' raw counters -> (used, total) ints, or (None, None)."""
  try:
    used, total = text.split()[:2]
    return int(used), int(total)
  except Exception:
    return None, None


def parse_leading_int(text):
  """'37 %' / '37%' / '37' -> 37; anything unparsable -> None."""
  try:
    s = text.strip()
    digits = ''
    for ch in s:
      if ch.isdigit() or (ch == '-' and not digits):
        digits += ch
      else:
        break
    return int(digits)
  except Exception:
    return None


class GpuSampler:
  def __init__(self, kgsl_dir=KGSL_DIR, interval=1.0, clock=time.monotonic):
    self.kgsl_dir, self.interval, self.clock = kgsl_dir, interval, clock
    self.last_sample = None
    self._freq_path = None

  def _cur_freq_path(self):
    if self._freq_path is None:
      candidates = [f'{self.kgsl_dir}/devfreq/cur_freq']
      try:
        candidates += sorted(glob.glob(f'{self.kgsl_dir}/devfreq/*/cur_freq'))
      except Exception:
        pass
      candidates.append(f'{self.kgsl_dir}/gpuclk')
      for path in candidates:
        if _read(path) is not None:
          self._freq_path = path
          break
    return self._freq_path

  def read(self):
    try:
      used, total = parse_gpubusy(_read(f'{self.kgsl_dir}/gpubusy'))
      pct = parse_leading_int(_read(f'{self.kgsl_dir}/gpu_busy_percentage'))
      freq_path = self._cur_freq_path()
      freq = parse_leading_int(_read(freq_path)) if freq_path else None
      return {'gpubusy_used': used, 'gpubusy_total': total, 'gpu_busy_pct': pct, 'gpu_cur_freq': freq}
    except Exception:
      return None

  def maybe_sample(self):
    """Return a sample dict at most once per `interval` seconds, else None. Never raises."""
    try:
      now = self.clock()
      if self.last_sample is not None and now - self.last_sample < self.interval:
        return None
      self.last_sample = now
      sample = self.read()
      if sample is not None:
        sample['gpu_sample_mono'] = round(now, 4)
      return sample
    except Exception:
      return None


class GpuTimingDiag:
  """Collects per-run monotonic intervals and the latest kgsl sample until the
  next runtimeTiming emission (RuntimeDiagnostics emits about once per second)."""

  def __init__(self, sampler=None, max_intervals=64):
    self.sampler = sampler if sampler is not None else GpuSampler()
    self.intervals = deque(maxlen=max_intervals)
    self.pending = {}

  def add_run(self, start_mono, end_mono):
    try:
      self.intervals.append([round(start_mono, 4), round(end_mono, 4)])
    except Exception:
      pass

  def poll(self):
    try:
      sample = self.sampler.maybe_sample()
      if sample is not None:
        self.pending = sample
    except Exception:
      pass

  def context(self, base=None):
    """Context dict for RuntimeDiagnostics.record (never raises)."""
    ctx = dict(base or {})
    try:
      ctx['run_intervals'] = list(self.intervals)
      ctx.update(self.pending)
    except Exception:
      pass
    return ctx

  def after_record(self, diagnostics):
    # RuntimeDiagnostics resets frames to 0 exactly when it emitted.
    try:
      if diagnostics.frames == 0:
        self.intervals.clear()
        self.pending = {}
    except Exception:
      pass
