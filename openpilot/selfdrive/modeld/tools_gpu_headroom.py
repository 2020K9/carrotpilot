#!/usr/bin/env python3
"""Offline GPU headroom report from runtimeTiming swaglog events in an rlog/qlog.

Usage: tools_gpu_headroom.py <rlog[.zst]|qlog[.zst]> [more logs...]

Reads modeld / dmonitoringmodeld runtimeTiming records (JSON inside
logMessage/errorLogMessage) and reports inference_ms, gpu_sync_wait_ms, overlap
of modeld vs dmonitoringmodeld run intervals, kgsl GPU busy and frequency, and an
estimated GPU headroom per 50 ms model cycle. Logs recorded before these fields
existed are handled gracefully (the missing sections are reported as absent).
"""
import json
import sys

MODELD = 'modeld'
DM = 'dmonitoringmodeld'
CYCLE_MS = 50.0


# ---------------------------------------------------------------- pure math

def percentile(values, q):
  vals = sorted(v for v in values if v is not None)
  if not vals:
    return None
  if len(vals) == 1:
    return vals[0]
  pos = (len(vals) - 1) * q / 100.0
  lo = int(pos)
  hi = min(lo + 1, len(vals) - 1)
  return vals[lo] + (vals[hi] - vals[lo]) * (pos - lo)


def stats(values):
  vals = [v for v in values if v is not None]
  if not vals:
    return None
  return {'n': len(vals), 'p50': percentile(vals, 50), 'p95': percentile(vals, 95),
          'mean': sum(vals) / len(vals), 'max': max(vals)}


def merge_intervals(intervals):
  out = []
  for s, e in sorted((float(s), float(e)) for s, e in intervals if e > s):
    if out and s <= out[-1][1]:
      out[-1][1] = max(out[-1][1], e)
    else:
      out.append([s, e])
  return out


def interval_overlap(a, b):
  """Overlap length (same units) between two (start, end) intervals."""
  return max(0.0, min(a[1], b[1]) - max(a[0], b[0]))


def per_run_overlap(runs, others):
  """For each run in `runs`, (overlap_seconds, fraction_of_run) against the merged `others`."""
  merged = merge_intervals(others)
  result = []
  for s, e in runs:
    dur = e - s
    ov = sum(interval_overlap((s, e), o) for o in merged)
    result.append((ov, ov / dur if dur > 0 else 0.0))
  return result


def busy_from_gpubusy(samples):
  """samples: [(used, total), ...] cumulative-or-windowed raw kgsl counters in time order.

  kgsl gpubusy reports (busy, total) for the window since the previous read on
  most kernels; on some it is cumulative. Use deltas when both counters grow
  monotonically, else treat each pair as its own window. Returns list of busy fractions.
  """
  out = []
  prev = None
  for used, total in samples:
    if used is None or total is None:
      prev = None
      continue
    if prev is not None and used >= prev[0] and total > prev[1]:
      du, dt = used - prev[0], total - prev[1]
      frac = du / dt
      if 0.0 <= frac <= 1.0:
        out.append(frac)
      elif total > 0 and 0 <= used <= total:
        out.append(used / total)
    elif total > 0 and 0 <= used <= total:
      out.append(used / total)
    prev = (used, total)
  return out


def headroom_ms(busy_frac, cycle_ms=CYCLE_MS):
  if busy_frac is None:
    return None
  return cycle_ms * (1.0 - min(max(busy_frac, 0.0), 1.0))


def distribution(values):
  counts = {}
  for v in values:
    if v is not None:
      counts[v] = counts.get(v, 0) + 1
  total = sum(counts.values())
  return {k: (c, c / total) for k, c in sorted(counts.items())} if total else {}


# ---------------------------------------------------------------- log parsing

def iter_messages(paths):
  try:
    from openpilot.tools.lib.logreader import LogReader
  except Exception:
    LogReader = None
  for path in paths:
    if LogReader is not None:
      reader = LogReader(path)
    else:
      reader = _capnp_reader(path)
    yield from reader


def _capnp_reader(path):
  from openpilot.cereal import log
  with open(path, 'rb') as f:
    data = f.read()
  if path.endswith('.zst'):
    import zstandard
    data = zstandard.ZstdDecompressor().stream_reader(__import__('io').BytesIO(data)).read()
  return log.Event.read_multiple_bytes(data, traversal_limit_in_words=2**63 - 1)


def collect(messages):
  """Return dict component -> list of runtimeTiming msg dicts, plus per-frame fallbacks."""
  rt = {MODELD: [], DM: []}
  frame = {'modelV2_exec_ms': [], 'dm_exec_ms': [], 'dm_gpu_exec_ms': [], 'deviceState_gpu_pct': []}
  for m in messages:
    try:
      w = m.which()
      if w in ('logMessage', 'errorLogMessage'):
        s = getattr(m, w)
        if 'runtimeTiming' not in s:
          continue
        d = json.loads(s).get('msg')
        if isinstance(d, dict) and d.get('event') == 'runtimeTiming' and d.get('component') in rt:
          rt[d['component']].append(d)
      elif w == 'modelV2':
        frame['modelV2_exec_ms'].append(m.modelV2.modelExecutionTime * 1000)
      elif w == 'driverStateV2':
        frame['dm_exec_ms'].append(m.driverStateV2.modelExecutionTime * 1000)
        frame['dm_gpu_exec_ms'].append(m.driverStateV2.gpuExecutionTime * 1000)
      elif w == 'deviceState':
        frame['deviceState_gpu_pct'].append(m.deviceState.gpuUsagePercent)
    except Exception:
      continue
  return rt, frame


def metric(records, name, key='mean'):
  return [r['metrics'][name][key] for r in records if name in r.get('metrics', {})]


def fmt(st, unit='ms'):
  if st is None:
    return 'absent'
  return f"n={st['n']} p50={st['p50']:.2f}{unit} p95={st['p95']:.2f}{unit} mean={st['mean']:.2f}{unit} max={st['max']:.2f}{unit}"


def report(rt, frame):
  lines = []
  notes = []
  for comp in (MODELD, DM):
    recs = rt[comp]
    lines.append(f'[{comp}] runtimeTiming records: {len(recs)}')
    lines.append(f'  inference_ms (per-1s mean) : {fmt(stats(metric(recs, "inference_ms")))}')
    lines.append(f'  inference_ms (per-1s max)  : {fmt(stats(metric(recs, "inference_ms", "max")))}')
    sync = metric(recs, 'gpu_sync_wait_ms')
    lines.append(f'  gpu_sync_wait_ms (mean)    : {fmt(stats(sync))}')
    lines.append(f'  gpu_sync_wait_ms (max)     : {fmt(stats(metric(recs, "gpu_sync_wait_ms", "max")))}')
    if recs and not sync:
      notes.append(f'{comp}: gpu_sync_wait_ms absent (log predates GPU timing diagnostics)')
  lines.append(f'[frames] modelV2.modelExecutionTime : {fmt(stats(frame["modelV2_exec_ms"]))}')
  lines.append(f'[frames] driverStateV2.modelExecutionTime : {fmt(stats(frame["dm_exec_ms"]))}')
  lines.append(f'[frames] driverStateV2.gpuExecutionTime : {fmt(stats(frame["dm_gpu_exec_ms"]))}')

  # overlap
  m_runs = [tuple(iv) for r in rt[MODELD] for iv in r.get('run_intervals', [])]
  d_runs = [tuple(iv) for r in rt[DM] for iv in r.get('run_intervals', [])]
  if m_runs and d_runs:
    ov = per_run_overlap(m_runs, d_runs)
    ov_ms = [o * 1000 for o, _ in ov]
    frac = [f for _, f in ov]
    lines.append(f'[overlap] modeld runs={len(m_runs)} dm runs={len(d_runs)}')
    lines.append(f'  overlap ms per modeld run : {fmt(stats(ov_ms))}')
    lines.append(f'  overlap fraction of modeld run : {fmt(stats(frac), "")}')
    lines.append(f'  modeld runs with any overlap : {sum(1 for o in ov_ms if o > 0) / len(ov_ms):.1%}')
  else:
    lines.append('[overlap] absent')
    notes.append('run_intervals absent (log predates GPU timing diagnostics); overlap not computable')

  # gpu busy / freq: one kgsl sampler per process; prefer modeld's samples, else dm's
  samples = [r for r in rt[MODELD] if 'gpubusy_used' in r] or [r for r in rt[DM] if 'gpubusy_used' in r]
  samples.sort(key=lambda r: r.get('gpu_sample_mono', r.get('mono_time', 0)))
  busy = busy_from_gpubusy([(r.get('gpubusy_used'), r.get('gpubusy_total')) for r in samples])
  pct = [r.get('gpu_busy_pct') for r in samples if r.get('gpu_busy_pct') is not None]
  freq = [r.get('gpu_cur_freq') for r in samples]
  if samples:
    lines.append(f'[gpu] kgsl samples: {len(samples)}')
    lines.append(f'  busy from gpubusy deltas : {fmt(stats([b * 100 for b in busy]), "%")}')
    lines.append(f'  gpu_busy_percentage      : {fmt(stats(pct), "%")}')
    dist = distribution(freq)
    lines.append('  cur_freq distribution    : ' + (', '.join(f'{k}:{c} ({p:.0%})' for k, (c, p) in dist.items()) or 'absent'))
  else:
    lines.append('[gpu] kgsl samples absent')
    notes.append('gpubusy/gpu_busy_percentage/cur_freq absent (log predates kgsl sampler)')
  dev = stats(frame['deviceState_gpu_pct'])
  lines.append(f'[gpu] deviceState.gpuUsagePercent : {fmt(dev, "%")}')

  # headroom
  src, b50, b95 = None, None, None
  if busy:
    src, b50, b95 = 'gpubusy deltas', percentile(busy, 50), percentile(busy, 95)
  elif pct:
    src, b50, b95 = 'gpu_busy_percentage', percentile(pct, 50) / 100, percentile(pct, 95) / 100
  if src:
    lines.append(f'[headroom] per {CYCLE_MS:.0f}ms cycle from {src}: ' +
                 f'p50 busy -> {headroom_ms(b50):.1f}ms free, p95 busy -> {headroom_ms(b95):.1f}ms free')
  else:
    lines.append('[headroom] absent (no kgsl busy source; deviceState.gpuUsagePercent is not used, it is unreliable on some devices)')
  m_inf = stats(metric(rt[MODELD], 'inference_ms')) or stats(frame['modelV2_exec_ms'])
  d_inf = stats(metric(rt[DM], 'inference_ms')) or stats(frame['dm_exec_ms'])
  if m_inf and d_inf:
    tot = m_inf['p50'] + d_inf['p50']
    lines.append(f'[wall-sum] modeld p50 + dm p50 inference wall time = {tot:.1f}ms per {CYCLE_MS:.0f}ms cycle ' +
                 f'({CYCLE_MS - tot:+.1f}ms); wall times include GPU contention, so this is not GPU busy time')
  if notes:
    lines.append('NOTES:')
    lines += [f'  - {n}' for n in notes]
  return '\n'.join(lines)


def main(argv):
  if not argv:
    print(__doc__)
    return 1
  rt, frame = collect(iter_messages(argv))
  print(f'logs: {len(argv)}')
  print(report(rt, frame))
  return 0


if __name__ == '__main__':
  sys.exit(main(sys.argv[1:]))
