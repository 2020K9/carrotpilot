"""K9 NOT_FINITE tracer (대감님 2026-11-21 20:57 "추적프로그램 만들면 되지 않니?").

Always on, records only around a NOT_FINITE frame (the JL_NF line from session._infer). Never changes a frame's
inputs, outputs, status, recovery or error handling; every hook swallows its own errors.

Per frame (hot path, preallocated after the first frame of an engine):
  * the frame's non-state inputs (as staged into the engine inputs) and, after the reply, its full `outputs`, in a
    ring of PREV+1 slots;
  * the frame's input recurrent state: on the TensorRT device-state path one async device-to-device copy before the
    graph (DeviceState.trace_snapshot); on the host-state path a host copy.
On NOT_FINITE (after the reply was sent):
  * the failed frame's inputs + input state + all outputs, and the previous normal frames, go to a writer thread;
  * when the link has been idle IDLE_S (the comma reconnects after every NOT_FINITE), the same inputs and state are
    run REPLAYS times on the same loaded engine under host.lock, with the live state saved and restored around it:
    NaN again = input/model (numerics), clean = GPU/power/kernel (transient);
  * Jetson health (GPU clock, rails mV/mA, temperatures, nvpmodel, cooling/throttle; from host_performance's 1 Hz
    status file) kept in a 1 s ring: 60 s before is written at once, then every second is appended + fsynced until
    60 s after the last error (so a following hang/reboot keeps everything up to it), plus filtered dmesg (nvgpu etc.).
Storage: <cache root>/nf_trace/<event id>/ (persistent engine cache disk), bounded by KEEP events / MAX_BYTES.
"""
from __future__ import annotations

import json
import logging
import os
import queue
import re
import shutil
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

log = logging.getLogger('jetlink.nf_trace')

VERSION = 1
PREV = 4                 # previous normal frames kept with an event
KEEP = 8                 # events kept on disk
MAX_BYTES = 400 << 20    # all events together
FULL_MIN_INTERVAL = 15.  # s between two events with tensors; others keep meta + health only
PRE_S = POST_S = 60
IDLE_S = 2.0             # no frame for this long before a replay may take the engine
REPLAY_WAIT_S = 600.
REPLAYS = 3
DMESG_RE = re.compile(r'nvgpu|nvrm|gk20a|ga10b|gr:|mmu|xid|sm |fault|ecc|tegra.*(err|fail)|throttl|oc[0-9]* |'
                      + r'over.?current|under.?volt|brown|soctherm|thermal|pcie|usb|dwc|ffs|watchdog|panic|oops|bug:',
                      re.IGNORECASE)


def _boot_id():
  try:
    return Path('/proc/sys/kernel/random/boot_id').read_text().strip()
  except OSError:
    return 'unknown'


def _count_bad(value):
  value = np.asarray(value)
  if value.dtype.kind != 'f':
    return 0
  return int(value.size - np.count_nonzero(np.isfinite(value)))


def _stats(value):
  """Compact numbers for a tensor: size, non-finite count, max |x| of the finite part (overflow headroom)."""
  value = np.asarray(value)
  bad = _count_bad(value)
  finite = value[np.isfinite(value)] if value.dtype.kind == 'f' and bad else value
  peak = float(np.max(np.abs(finite.astype(np.float32)))) if finite.size else None
  return {'dtype': str(value.dtype), 'shape': list(value.shape), 'nonfinite': bad, 'max_abs': peak}


def _fsync_write(path: Path, data: bytes):
  tmp = path.with_name('.' + path.name + '.tmp')
  with open(tmp, 'wb') as f:
    f.write(data)
    f.flush()
    os.fsync(f.fileno())
  os.replace(tmp, path)


def _fsync_dir(path: Path):
  try:
    fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
    try:
      os.fsync(fd)
    finally:
      os.close(fd)
  except OSError:
    pass


class Slot:
  __slots__ = ('frame', 'flags', 'mono', 'inputs', 'outputs', 'filled')

  def __init__(self, inputs, outputs):
    self.inputs, self.outputs = inputs, outputs
    self.frame = self.flags = 0
    self.mono = 0.
    self.filled = False


class NfTrace:
  def __init__(self, root, sampler=None, *, keep=KEEP, max_bytes=MAX_BYTES, prev=PREV, pre_s=PRE_S, post_s=POST_S,
               idle_s=IDLE_S, replays=REPLAYS, replay_wait_s=REPLAY_WAIT_S, full_min_interval=FULL_MIN_INTERVAL,
               dmesg=None, prev_boot_kernel=None, start=True, clock=time.monotonic, wall=time.time):  # noqa: TID251 -- wall clock only names events
    self.root = Path(root)
    self.sampler = sampler
    self.keep, self.max_bytes, self.prev = keep, max_bytes, prev
    self.pre_s, self.post_s, self.idle_s, self.replays = pre_s, post_s, idle_s, replays
    self.replay_wait_s, self.full_min_interval = replay_wait_s, full_min_interval
    self.dmesg = dmesg if dmesg is not None else self._dmesg
    self.prev_boot_kernel = prev_boot_kernel if prev_boot_kernel is not None else self._prev_boot_kernel
    self.clock, self.wall = clock, wall
    self.host = None
    self.boot = _boot_id()
    self.lock = threading.Lock()          # events / summary / armed files (never taken on the frame path)
    self.health = deque(maxlen=max(4, int(pre_s) + 5))
    self.armed = {}                       # event dir -> (until mono, open file)
    self.events = []                      # summaries, newest last
    self.jobs = queue.Queue(maxsize=4)
    self.seq = 0
    self.last_frame = float('-inf')
    self.last_full = float('-inf')
    self.errors = 0
    self._engine = None
    self._ring = None
    self._head = 0
    self._state_host = None
    self._stop = threading.Event()
    self._threads = []
    try:
      self.root.mkdir(parents=True, exist_ok=True)
      self._recover()
    except Exception:
      log.exception('nf_trace: storage unavailable')
    if start:
      for target, name in ((self._sample_loop, 'nf-trace-health'), (self._job_loop, 'nf-trace-writer')):
        t = threading.Thread(target=target, name=name, daemon=True)
        t.start()
        self._threads.append(t)

  def bind(self, host):
    self.host = host
    return self

  def close(self):
    self._stop.set()
    try:
      self.jobs.put_nowait(None)
    except queue.Full:
      pass
    for t in self._threads:
      t.join(timeout=5)
    with self.lock:
      for _, f in self.armed.values():
        try:
          f.close()
        except OSError:
          pass
      self.armed.clear()

  # -- frame path -----------------------------------------------------------

  def _prepare(self, loaded):
    engine = loaded.engine
    spec = loaded.spec
    state_names = set(spec.state_pairs) if spec.stateful else set()
    shapes = {n: np.asarray(v) for n, v in loaded.host_inputs.items() if n not in state_names}
    self._ring = [Slot({n: np.empty_like(v) for n, v in shapes.items()},
                       np.empty(int(spec.output_nelem), np.float32)) for _ in range(self.prev + 1)]
    self._head = 0
    self._state_host = None
    ds = getattr(engine, 'device_state', None)
    if state_names and ds is not None and hasattr(ds, 'trace_enable'):
      ds.trace_enable()
    elif state_names:
      state = getattr(loaded.queues, 'state', None) or {n: loaded.host_inputs[n] for n in state_names}
      self._state_host = {n: (np.asarray(v), np.empty_like(v)) for n, v in state.items()}
    self._engine = engine

  def before_run(self, loaded, frame_id, flags):
    """After staging, before engine.run(): copy this frame's inputs into the ring head."""
    try:
      if self._engine is not loaded.engine:
        self._prepare(loaded)
      slot = self._ring[self._head]
      for name, dest in slot.inputs.items():
        np.copyto(dest, loaded.host_inputs[name], casting='no')
      if self._state_host is not None:
        for live, copy in self._state_host.values():
          np.copyto(copy, live, casting='no')
      slot.frame, slot.flags, slot.mono, slot.filled = int(frame_id), int(flags), self.clock(), False
      self.last_frame = slot.mono
    except Exception:
      self._engine = None
      self.errors += 1

  def after_frame(self, loaded, frame_id, status, out32, outputs=None, detail='', timings=(0, 0, 0)):
    """After the reply is on the wire. OK: keep the outputs, advance the ring. NOT_FINITE: hand an event over."""
    try:
      if self._engine is not loaded.engine or self._ring is None:
        return
      slot = self._ring[self._head]
      if slot.frame != int(frame_id):
        return
      if out32.size == slot.outputs.size:
        np.copyto(slot.outputs, out32.reshape(-1), casting='same_kind')
      slot.filled = True
      self.last_frame = self.clock()
      if status:
        self._capture(loaded, slot, detail, outputs, timings)
      self._head = (self._head + 1) % len(self._ring)
    except Exception:
      self.errors += 1
      log.exception('nf_trace: capture failed')

  def _capture(self, loaded, slot, detail, outputs, timings):
    now = self.clock()
    self.seq += 1
    full = now - self.last_full >= self.full_min_interval
    event = {'seq': self.seq, 'mono': now, 'wall': self.wall(), 'boot': self.boot, 'frame': slot.frame,
             'flags': slot.flags, 'detail': str(detail)[:400], 'model': loaded.spec.sha256,
             'frame_skip': getattr(loaded.spec, 'frame_skip', None), 'timings_us': list(timings),
             'tensors': full, 'client': getattr(getattr(self.host, 'session', None), 'client', '')}
    if full:
      self.last_full = now
      inputs = {n: v.copy() for n, v in slot.inputs.items()}
      ds = getattr(loaded.engine, 'device_state', None)
      if self._state_host is not None:
        state = {n: copy.copy() for n, (_, copy) in self._state_host.items()}
        event['state_source'] = 'host'
      elif ds is not None and getattr(ds, 'trace', None) is not None:
        state = ds.trace_read_state()
        event['state_source'] = 'device_snapshot'
      else:
        state = {}
        event['state_source'] = 'none'
      outs = {'outputs': slot.outputs.copy()}
      for name, value in (outputs or {}).items():
        if name != 'outputs':
          outs[name] = np.array(value, copy=True)
      if ds is not None and getattr(ds, 'trace', None) is not None:
        try:
          outs.update(ds.trace_read_outputs())
        except Exception:
          event['state_outputs'] = 'unreadable'
      prev = []
      n = len(self._ring)
      for k in range(1, n):
        s = self._ring[(self._head - k) % n]
        if s.filled and s.frame != slot.frame:
          prev.append({'frame': s.frame, 'flags': s.flags, 'age_s': round(now - s.mono, 3),
                       'inputs': {a: b.copy() for a, b in s.inputs.items()}, 'outputs': s.outputs.copy()})
      event['_tensors'] = {'inputs': inputs, 'state': state, 'outputs': outs, 'prev': prev}
      event['_loaded'] = loaded
    try:
      self.jobs.put_nowait(event)
    except queue.Full:
      self.errors += 1

  # -- writer thread --------------------------------------------------------

  def _event_id(self, event):
    stamp = time.strftime('%Y%m%d-%H%M%S', time.gmtime(event['wall']))
    return f"{stamp}-b{event['boot'][:8]}-f{event['frame']}-{event['seq']}"

  def _job_loop(self):
    while not self._stop.is_set():
      try:
        job = self.jobs.get(timeout=1)
      except queue.Empty:
        continue
      if job is None:
        return
      try:
        self.process(job)
      except Exception:
        self.errors += 1
        log.exception('nf_trace: event write failed')

  def process(self, event):
    """Write one event, run its replay when the link goes idle, keep the post window armed."""
    tensors = event.pop('_tensors', None)
    loaded = event.pop('_loaded', None)
    eid = self._event_id(event)
    path = self.root / eid
    path.mkdir(parents=True, exist_ok=True)
    meta = dict(event, id=eid, version=VERSION, replay={'state': 'pending' if tensors else 'no_tensors'},
                post_complete=False)
    if tensors:
      meta['stats'] = {f'{g}.{n}': _stats(v) for g in ('inputs', 'state', 'outputs') for n, v in tensors[g].items()}
      meta['prev'] = [{'frame': p['frame'], 'flags': p['flags'], 'age_s': p['age_s'],
                       'outputs': _stats(p['outputs'])} for p in tensors['prev']]
      arrays = {f'{g}__{n}': v for g in ('inputs', 'state', 'outputs') for n, v in tensors[g].items()}
      for i, p in enumerate(tensors['prev']):
        arrays.update({f'prev{i}__inputs__{n}': v for n, v in p['inputs'].items()})
        arrays[f'prev{i}__outputs__outputs'] = p['outputs']
      tmp = path / '.tensors.npz.tmp'
      with open(tmp, 'wb') as f:
        np.savez_compressed(f, **arrays)
        f.flush()
        os.fsync(f.fileno())
      os.replace(tmp, path / 'tensors.npz')
    with self.lock:
      pre = list(self.health)
    _fsync_write(path / 'health_pre.jsonl', ''.join(json.dumps(h) + '\n' for h in pre).encode())
    self._write_dmesg(path, 'dmesg_at_error.txt')
    self._write_meta(path, meta)
    self._arm(path)
    self._summarize(meta)
    _fsync_dir(self.root)
    self._rotate(keep_path=path)
    if tensors and loaded is not None:
      meta['replay'] = self.replay(loaded, tensors['inputs'], tensors['state'])
      self._write_meta(path, meta)
      self._summarize(meta)

  def _write_meta(self, path, meta):
    _fsync_write(path / 'meta.json', json.dumps(meta, indent=1, default=str).encode())

  def _write_dmesg(self, path, name):
    try:
      text = self.dmesg()
    except Exception as exc:
      text = f'dmesg unavailable: {type(exc).__name__}: {exc}\n'
    _fsync_write(path / name, text.encode('utf-8', 'replace')[-262144:])

  @staticmethod
  def _dmesg():
    # jetlink has kernel.dmesg_restrict=1 -> plain dmesg fails; the image grants it sudo NOPASSWD (-n: never prompts)
    for cmd in (['sudo', '-n', 'dmesg', '--time-format', 'iso'], ['dmesg', '--time-format', 'iso'], ['dmesg'],
                ['journalctl', '-k', '-b', '0', '-n', '3000', '--no-pager']):
      try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
      except (OSError, subprocess.SubprocessError):
        continue
      if r.returncode == 0 and r.stdout:
        lines = r.stdout.splitlines()
        hits = [x for x in lines if DMESG_RE.search(x)]
        return (f'# {" ".join(cmd)}: {len(lines)} lines, {len(hits)} matching\n' + '\n'.join(hits[-400:]) +
                '\n# last 60 lines\n' + '\n'.join(lines[-60:]) + '\n')
    return 'dmesg unavailable (no permission or tool)\n'

  @staticmethod
  def _prev_boot_kernel():
    try:
      r = subprocess.run(['journalctl', '-k', '-b', '-1', '-n', '4000', '--no-pager'], capture_output=True, text=True,
                         timeout=10)
    except (OSError, subprocess.SubprocessError) as exc:
      return f'journalctl unavailable: {exc}\n'
    lines = r.stdout.splitlines()
    hits = [x for x in lines if DMESG_RE.search(x)]
    text = (f'# journalctl -k -b -1 rc={r.returncode}: {len(lines)} lines\n' + '\n'.join(hits[-400:]) +
            '\n# last 120 lines\n' + '\n'.join(lines[-120:]) + '\n' + r.stderr[-400:])
    # The protected Jetson journal is RAM-only, so -b -1 is empty there: add the K9 logkeep copy (DATA).
    try:
      from host_logkeep import persistent_root
      from logkeep import previous_boot_kernel
      root = persistent_root()
      kept = previous_boot_kernel(root) if root is not None else ''
    except Exception as exc:  # tools/jetlink not importable (tests, other hosts)
      kept = f'# logkeep unavailable: {type(exc).__name__}\n'
    if kept:
      kl = kept.splitlines()
      text += (f'# logkeep previous boot kern.log: {len(kl)} lines\n' + '\n'.join(x for x in kl if DMESG_RE.search(x))[-65536:] +
               '\n# last 120 lines\n' + '\n'.join(kl[-120:]) + '\n')
    return text

  # -- replay ---------------------------------------------------------------

  def replay(self, loaded, inputs, state):
    """Same inputs + input state, same engine, while nothing else uses it. Live state and buffers are restored."""
    started = self.clock()
    host = self.host
    if host is None:
      return {'state': 'skipped', 'reason': 'no host'}
    results = []
    while len(results) < self.replays:
      if self._stop.is_set():
        return {'state': 'skipped', 'reason': 'stopping', 'runs': results}
      if self.clock() - started >= self.replay_wait_s:
        return {'state': 'skipped', 'reason': 'never idle', 'runs': results}
      if self.clock() - self.last_frame < self.idle_s:
        time.sleep(0.2)
        continue
      busy = False
      with host.lock:
        if host.loaded is not loaded:
          return {'state': 'skipped', 'reason': 'engine changed', 'runs': results}
        if self.clock() - self.last_frame < self.idle_s:
          busy = True
        else:
          results.append(self._replay_once(loaded, inputs, state))
      if busy:
        time.sleep(0.2)
    bad = sum(1 for r in results if r.get('nonfinite_total', 0) > 0)
    errors = sum(1 for r in results if 'error' in r)
    verdict = ('error' if errors == len(results) else 'nan_again' if bad == len(results) else
               'clean' if bad == 0 else 'mixed')
    return {'state': 'done', 'verdict': verdict, 'nan_runs': bad, 'runs': results,
            'waited_s': round(self.clock() - started, 3),
            'meaning': {'nan_again': 'same input+state gives NaN again: input/model numerics (e.g. fp16 overflow)',
                        'clean': 'same input+state is finite: transient (GPU clock/power/kernel/memory)',
                        'mixed': 'nondeterministic: suspect GPU/power', 'error': 'replay could not run'}[verdict]}

  def _replay_once(self, loaded, inputs, state):
    engine = loaded.engine
    ds = getattr(engine, 'device_state', None)
    hi = loaded.host_inputs
    saved = {n: np.array(v, copy=True) for n, v in hi.items()}
    gpu_us = getattr(engine, 'last_gpu_us', 0)
    kept = None
    t0 = time.perf_counter()
    try:
      for n, v in inputs.items():
        np.copyto(hi[n], v, casting='no')
      if ds is not None and getattr(ds, 'trace', None) is not None:
        kept = ds.trace_begin_replay(state)
      else:
        for n, v in state.items():
          np.copyto(hi[n], v, casting='no')
      out = engine.run()
      result = {'gpu_us': int(getattr(engine, 'last_gpu_us', 0))}
      counts = {n: _count_bad(v) for n, v in out.items()}
      if ds is not None and kept is not None:
        result['device_flag'] = int(ds.bad[0])
        counts.update({n: _count_bad(v) for n, v in ds.trace_read_outputs().items()})
      result['nonfinite'] = counts
      result['nonfinite_total'] = sum(counts.values())
      o = np.asarray(out.get('outputs')).reshape(-1)
      result['outputs_max_abs'] = float(np.max(np.abs(o[np.isfinite(o)]))) if np.isfinite(o).any() else None
    except Exception as exc:
      result = {'error': f'{type(exc).__name__}: {exc}'[:200]}
    finally:
      try:
        if kept is not None:
          ds.trace_end_replay(kept)
      finally:
        for n, v in saved.items():
          np.copyto(hi[n], v, casting='no')
        if hasattr(engine, 'last_gpu_us'):
          engine.last_gpu_us = gpu_us
    result['wall_ms'] = round((time.perf_counter() - t0) * 1e3, 2)
    return result

  # -- health ring ----------------------------------------------------------

  def _sample_loop(self):
    while not self._stop.wait(1.0):
      self.sample_once()

  def sample_once(self):
    try:
      value = self.sampler() if self.sampler is not None else {}
    except Exception as exc:
      value = {'error': type(exc).__name__}
    row = {'mono': round(self.clock(), 3), 'wall': round(self.wall(), 3), **value}
    line = json.dumps(row, default=str) + '\n'
    now = self.clock()
    with self.lock:
      self.health.append(row)
      done = []
      for path, (until, f) in self.armed.items():
        try:
          f.write(line)
          f.flush()
          os.fsync(f.fileno())
        except OSError:
          done.append(path)
          continue
        if now >= until:
          done.append(path)
      finished = [(p, self.armed.pop(p)) for p in done]
    for path, (_, f) in finished:
      try:
        f.close()
        self._write_dmesg(path, 'dmesg_after.txt')
        meta = json.loads((path / 'meta.json').read_text())
        meta['post_complete'] = True
        self._write_meta(path, meta)
      except (OSError, ValueError):
        pass

  def _arm(self, path):
    until = self.clock() + self.post_s
    with self.lock:
      # A new error extends every open window: the escalation before a hang stays recorded up to the end.
      for p, (_, f) in list(self.armed.items()):
        self.armed[p] = (until, f)
      if path not in self.armed:
        self.armed[path] = (until, open(path / 'health_post.jsonl', 'a'))

  # -- index ----------------------------------------------------------------

  def _summarize(self, meta):
    replay = meta.get('replay') or {}
    entry = {'id': meta['id'], 'frame': meta['frame'], 'model': str(meta.get('model', ''))[:12],
             'detail': str(meta.get('detail', ''))[:140], 'tensors': bool(meta.get('tensors')),
             'replay': replay.get('verdict') or replay.get('state', '?'), 'boot': str(meta.get('boot', ''))[:8],
             'post_complete': bool(meta.get('post_complete'))}
    if meta.get('interrupted'):
      entry['interrupted'] = True
    with self.lock:
      self.events = [e for e in self.events if e['id'] != entry['id']] + [entry]
      self.events = self.events[-KEEP:]

  def summary(self):
    with self.lock:
      if not self.events:
        return None
      return {'v': VERSION, 'events': [dict(e) for e in self.events[-3:]]}

  def _recover(self):
    """Startup: index kept events; one whose post window never closed was cut by a crash/reboot."""
    metas = []
    for d in sorted(self.root.iterdir()):
      try:
        if d.is_dir():
          metas.append((d, json.loads((d / 'meta.json').read_text())))
      except (OSError, ValueError):
        continue
    metas.sort(key=lambda x: x[1].get('wall', 0))
    for d, meta in metas:
      if not meta.get('post_complete') and meta.get('boot') != self.boot:
        meta['post_complete'] = True
        meta['interrupted'] = 'boot changed before the 60 s window closed (reboot / power loss)'
        if meta.get('replay', {}).get('state') == 'pending':
          meta['replay'] = {'state': 'skipped', 'reason': 'reboot before replay'}
        if not (d / 'prev_boot_kernel.txt').exists():
          try:
            _fsync_write(d / 'prev_boot_kernel.txt', self.prev_boot_kernel().encode('utf-8', 'replace')[-262144:])
          except Exception:
            pass
        try:
          self._write_meta(d, meta)
        except OSError:
          pass
      self._summarize(meta)

  def _rotate(self, keep_path=None):
    dirs = []
    for d in self.root.iterdir():
      if d.is_dir():
        try:
          size = sum(f.stat().st_size for f in d.iterdir() if f.is_file())
          dirs.append((d.stat().st_mtime, d, size))
        except OSError:
          continue
    dirs.sort()
    total = sum(s for _, _, s in dirs)
    with self.lock:
      armed = set(self.armed)
    while dirs and (len(dirs) > self.keep or total > self.max_bytes):
      _, d, size = dirs.pop(0)
      if d == keep_path or d in armed:
        continue
      shutil.rmtree(d, ignore_errors=True)
      total -= size
      with self.lock:
        self.events = [e for e in self.events if e['id'] != d.name]

  # -- pulling (tools/jetlink/nf_trace_link.py) -----------------------------

  def listing(self):
    out = []
    with self.lock:
      armed = set(self.armed)
    for d in sorted(self.root.iterdir()):
      if not d.is_dir():
        continue
      files = {f.name: f.stat().st_size for f in sorted(d.iterdir()) if f.is_file() and not f.name.startswith('.')}
      try:
        meta = json.loads((d / 'meta.json').read_text())
      except (OSError, ValueError):
        continue
      done = (meta.get('post_complete') and d not in armed and
              (meta.get('replay') or {}).get('state') != 'pending')
      out.append({'id': d.name, 'files': files, 'complete': bool(done)})
    return out

  def read_file(self, eid, name, offset, size):
    if not re.fullmatch(r'[A-Za-z0-9._-]{1,120}', eid or '') or not re.fullmatch(r'[A-Za-z0-9_]+\.(json|jsonl|txt|npz)', name or ''):
      raise ValueError('bad name')
    path = self.root / eid / name
    if path.is_symlink() or not path.resolve().is_relative_to(self.root.resolve()):
      raise ValueError('bad path')
    with open(path, 'rb') as f:
      f.seek(max(0, int(offset)))
      return f.read(max(0, min(int(size), 262144)))


def _read_small(path):
  fd = os.open(path, os.O_RDONLY)
  try:
    data = os.read(fd, 256)
  finally:
    os.close(fd)
  if not isinstance(data, bytes):
    raise TypeError('no data')
  return data.decode('ascii', 'replace')


def carrot_health_sample(status=Path('/run/carrot-jetlink-performance/status.json'), thermal=Path('/sys/class/thermal')):
  """One compact health row: what host_performance samples at 1 Hz (no extra sysfs load) + thermal zones."""
  row = {}
  try:
    with status.open() as f:
      p = json.loads(f.read(65536))
    row['perf_age_s'] = round(time.monotonic() - float(p.get('sample_mono', 0)), 3)
    for key in ('gpu', 'emc', 'power_mode', 'power_rails', 'memory', 'status', 'error'):
      if key in p:
        row[key] = p[key]
    row['cooling'] = [c for c in p.get('cooling') or [] if c.get('state')]
    row['cpu_pct'] = (p.get('cpu') or {}).get('usage_pct') if isinstance(p.get('cpu'), dict) else None
  except (OSError, ValueError, TypeError, AttributeError) as exc:
    row['perf'] = type(exc).__name__
  temps = {}
  for zone in sorted(thermal.glob('thermal_zone*')):
    try:
      temps[_read_small(zone / 'type').strip()] = int(_read_small(zone / 'temp')) / 1000
    except (OSError, ValueError, TypeError):
      continue  # some Tegra zones fail their read (Path.read_text raised TypeError every second)
  row['temp_c'] = temps
  return row
