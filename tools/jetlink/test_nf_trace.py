"""K9 NOT_FINITE tracer (jetlink/server/nf_trace.py, nf_trace_link.py; 대감님 2026-11-21 20:57).

Host-state path through the real Session + JetlinkClient loopback (test_err_delay_hold), device-state path on
test_device_state's deferred-CUDA fake. Checks: replies unchanged with the tracer on, the saved inputs/state are the
failed frame's, previous normal frames kept, replay verdicts, live state restored after a replay, health window,
reboot recovery, rotation, the comma pull/rlog side. Not GPU validation."""
import json
import threading
from types import SimpleNamespace as NS

import numpy as np
import pytest

from jetlink.server.nf_trace import NfTrace, carrot_health_sample
from jetlink.client import NonFiniteOutput
import nf_trace_link as L
from test_err_delay_hold import pair, infer
from test_device_state import make_engine, runtime  # noqa: F401


class Clock:
  def __init__(self):
    self.t = 1000.

  def __call__(self):
    return self.t


MADE = []


@pytest.fixture(autouse=True)
def _close_tracers():
  yield
  while MADE:
    MADE.pop().close()


def tracer(tmp_path, clock=None, **kw):
  kw.setdefault('dmesg', lambda: 'nvgpu: test line\n')
  kw.setdefault('prev_boot_kernel', lambda: 'prev boot kernel\n')
  t = NfTrace(tmp_path / 'nf', lambda: {'gpu': {'g': {'clock_mhz': 1020}}}, start=False,
              clock=clock or Clock(), **kw)
  MADE.append(t)
  return t


def run(poison_at, with_tracer, tmp_path, frames=8, hold=False):
  client, session, loaded, transport = pair()
  host = session.host
  host.lock, host.loaded = threading.Lock(), loaded
  tr = None
  if with_tracer:
    tr = tracer(tmp_path).bind(host)
    host.nf_trace = tr
  replies, states = [], []
  for i in range(1, frames + 1):
    loaded.engine.poison = 'outputs' if i == poison_at else None
    states.append({k: v.copy() for k, v in loaded.queues.state.items()})
    try:
      infer(client, i, hold, reset=i == 1)
    except Exception as exc:
      replies.append(type(exc).__name__)
      client.dead = False
      continue
    replies.append(client.last_timings is not None)
  return replies, transport, tr, loaded, states


def test_replies_are_identical_with_and_without_the_tracer(tmp_path):
  a = run(5, False, tmp_path / 'a')
  b = run(5, True, tmp_path / 'b')
  assert a[0] == b[0]
  assert a[1].flags == b[1].flags
  ra = [bytes(m.payload) for m in a[1].replies]
  rb = [bytes(m.payload) for m in b[1].replies]
  assert ra == rb


@pytest.mark.parametrize('stays_bad', [True, False])
def test_failed_frame_is_saved_with_previous_frames_and_replayed(tmp_path, stays_bad):
  replies, transport, tr, loaded, states = run(6, True, tmp_path, frames=6)
  event = tr.jobs.get_nowait()
  assert event['frame'] == 6 and 'JL_NF frame=6' in event['detail'] and event['tensors']
  t = event['_tensors']
  # the failed frame's input state is the state before the run, not the reset one
  for k, v in states[-1].items():
    np.testing.assert_array_equal(t['state'][k], v)
  assert np.isnan(t['outputs']['outputs']).any()
  assert [p['frame'] for p in t['prev']] == [5, 4, 3, 2]
  live_state = {k: v.copy() for k, v in loaded.queues.state.items()}
  live_inputs = {k: v.copy() for k, v in loaded.host_inputs.items()}
  loaded.engine.poison = 'outputs' if stays_bad else None
  tr.clock.t += 10
  tr.process(event)
  d = next((tmp_path / 'nf').iterdir())
  meta = json.loads((d / 'meta.json').read_text())
  assert meta['replay']['verdict'] == ('nan_again' if stays_bad else 'clean')
  assert len(meta['replay']['runs']) == 3
  for run_ in loaded.engine.seen[-3:]:   # the replay ran on the saved state
    np.testing.assert_array_equal(run_[0], states[-1]['state_feat_q'])
  for k, v in live_state.items():
    np.testing.assert_array_equal(loaded.queues.state[k], v)
  for k, v in live_inputs.items():
    np.testing.assert_array_equal(loaded.host_inputs[k], v)
  with np.load(d / 'tensors.npz') as z:
    assert {'inputs__new_img', 'state__state_feat_q', 'outputs__outputs', 'prev0__inputs__new_img'} <= set(z.files)
  assert (d / 'dmesg_at_error.txt').read_text().startswith('nvgpu')
  assert tr.summary()['events'][-1]['replay'] == meta['replay']['verdict']


def test_replay_waits_for_idle_and_gives_up(tmp_path):
  replies, transport, tr, loaded, states = run(3, True, tmp_path, frames=3)
  event = tr.jobs.get_nowait()
  tr.replay_wait_s = 0.
  tr.process(event)   # clock never advanced: never idle
  meta = json.loads(next((tmp_path / 'nf').iterdir()).joinpath('meta.json').read_text())
  assert meta['replay'] == {'state': 'skipped', 'reason': 'never idle', 'runs': []}


def test_rate_limit_keeps_meta_only(tmp_path):
  client, session, loaded, transport = pair()
  host = session.host
  host.lock, host.loaded = threading.Lock(), loaded
  host.nf_trace = tr = tracer(tmp_path).bind(host)
  for i in (1, 2):
    loaded.engine.poison = 'outputs'
    with pytest.raises(NonFiniteOutput):
      infer(client, i, False, reset=True)
    client.dead = False
  first, second = tr.jobs.get_nowait(), tr.jobs.get_nowait()
  assert first['tensors'] and not second['tensors'] and '_tensors' not in second


def test_device_state_snapshot_and_replay_restore(runtime, monkeypatch, tmp_path):  # noqa: F811
  e, q, host = make_engine(runtime, True, monkeypatch)
  spec = NS(sha256='d' * 64, stateful=True, state_pairs=q.spec.state_pairs, output_nelem=2, frame_skip=4)
  loaded = NS(spec=spec, engine=e, host_inputs=host, queues=q)
  h = NS(lock=threading.Lock(), loaded=loaded)
  tr = tracer(tmp_path).bind(h)
  outs = []
  for i in range(4):
    q.step_into(np.full((2, 2), i + 1, np.uint8), np.array([i / 2, 0], np.float32), host)
    tr.before_run(loaded, i, 0)
    if i == 3:
      runtime.cuda.stream_sync(1)   # the previous frame's trailing feedback D2D is this frame's input state
      before = {n: np.frombuffer(runtime.cuda.memory[e.inputs[n].device_ptr], dtype=e.inputs[n].dtype).copy()
                for n in q.spec.state_pairs}
      e.bad_name = 'next_state_a'
    out = e.run()
    try:
      q.feedback(out)
      status = 0
    except FloatingPointError:
      status = 1
    outs.append(out['outputs'].copy())
    tr.after_frame(loaded, i, status, out['outputs'], out, 'JL_NF test' if status else '')
  event = tr.jobs.get_nowait()
  for n, v in before.items():
    np.testing.assert_array_equal(event['_tensors']['state'][n].reshape(-1), v)
  assert np.isnan(event['_tensors']['outputs']['next_state_a']).any()
  runtime.cuda.stream_sync(1)
  live = {n: np.frombuffer(runtime.cuda.memory[e.inputs[n].device_ptr], dtype=e.inputs[n].dtype).copy()
          for n in q.spec.state_pairs}
  flags = (e.device_state.needs_reset, e.device_state.needs_restore, int(e.device_state.bad[0]))
  e.bad_name = None
  tr.clock.t += 10
  tr.process(event)
  meta = json.loads(next((tmp_path / 'nf').iterdir()).joinpath('meta.json').read_text())
  assert meta['replay']['verdict'] == 'clean'
  runtime.cuda.stream_sync(1)
  for n, v in live.items():
    np.testing.assert_array_equal(np.frombuffer(runtime.cuda.memory[e.inputs[n].device_ptr], dtype=v.dtype), v)
  assert (e.device_state.needs_reset, e.device_state.needs_restore, int(e.device_state.bad[0])) == flags
  e.close()


def test_health_window_and_reboot_recovery(tmp_path):
  clock = Clock()
  tr = tracer(tmp_path, clock, post_s=3)
  for _ in range(5):
    tr.sample_once()
    clock.t += 1
  tr.process({'seq': 1, 'mono': clock.t, 'wall': 1.7e9, 'boot': tr.boot, 'frame': 9, 'flags': 0,
              'detail': 'JL_NF frame=9', 'model': 'm' * 64, 'tensors': False})
  d = next((tmp_path / 'nf').iterdir())
  assert len((d / 'health_pre.jsonl').read_text().splitlines()) == 5
  for _ in range(4):
    tr.sample_once()
    clock.t += 1
  assert len((d / 'health_post.jsonl').read_text().splitlines()) == 4
  assert json.loads((d / 'meta.json').read_text())['post_complete']
  assert (d / 'dmesg_after.txt').exists()
  # an event cut by a reboot: other boot id, window never closed
  meta = json.loads((d / 'meta.json').read_text())
  meta.update(post_complete=False, boot='other', replay={'state': 'pending'})
  (d / 'meta.json').write_text(json.dumps(meta))
  tr2 = tracer(tmp_path)
  meta = json.loads((d / 'meta.json').read_text())
  assert meta['interrupted'] and meta['replay']['state'] == 'skipped'
  assert (d / 'prev_boot_kernel.txt').read_text() == 'prev boot kernel\n'
  assert tr2.summary()['events'][0]['interrupted']


def test_rotation_keeps_newest(tmp_path):
  tr = tracer(tmp_path, keep=2, post_s=0)
  for i in range(4):
    tr.process({'seq': i, 'mono': 0, 'wall': 1.7e9 + i, 'boot': tr.boot, 'frame': i, 'flags': 0, 'detail': '',
                'model': 'm', 'tensors': False})
    tr.sample_once()
  names = sorted(d.name for d in (tmp_path / 'nf').iterdir())
  assert len(names) == 2 and names[-1].endswith('-f3-3')


def test_link_list_get_pull_and_rlog_note(tmp_path, monkeypatch):
  tr = tracer(tmp_path, post_s=0)
  tr.process({'seq': 1, 'mono': 0, 'wall': 1.7e9, 'boot': tr.boot, 'frame': 7, 'flags': 0, 'detail': 'JL_NF frame=7',
              'model': 'm', 'tensors': False})
  tr.sample_once()
  head, _ = L.split(b''.join(L.host_request(tr, json.dumps({'op': 'list'}).encode())))
  (event,) = head['events']
  assert event['complete'] and 'meta.json' in event['files']
  bad, _ = L.split(b''.join(L.host_request(tr, json.dumps({'op': 'get', 'id': '../x', 'name': 'meta.json'}).encode())))
  assert not bad['ok']

  class Client:
    def __init__(self):
      self.seq, self.reply = 0, None
      self.t = NS(send_json=lambda kind, seq, obj: setattr(self, 'reply', b''.join(L.host_request(tr, json.dumps(obj).encode()))))

    def _next_seq(self):
      self.seq += 1
      return self.seq

    def _expect(self, kind, seq, timeout):
      assert kind == L.RESPONSE
      return NS(payload=memoryview(self.reply))

  emitted = []
  c = L.Client({L.CAPABILITY: True}, lambda name, **kw: emitted.append((name, kw)), local=tmp_path / 'comma')
  c.pull(Client(), budget=1 << 20)
  local = tmp_path / 'comma' / event['id']
  assert (local / '.complete').exists()
  assert (local / 'meta.json').read_bytes() == (tmp_path / 'nf' / event['id'] / 'meta.json').read_bytes()
  state = L.telemetry(tr)
  c.note(state)
  c.last_note = float('-inf')
  c.note(state)
  lines = [kw['line'] for name, kw in emitted if name == 'jetsonNfTrace']
  assert len(lines) == 1 and lines[0].startswith(f"JL_NF_TRACE id={event['id']} frame=7") and 'JL_NF frame=7' in lines[0]
  assert L.telemetry(None) == {}


def test_offroad_only_pull(tmp_path):
  c = L.Client({L.CAPABILITY: True}, local=tmp_path)
  called = []
  c.pull = lambda client: called.append(1)
  c.idle(None, NS(get_bool=lambda k: k == 'IsOnroad'))
  assert not called
  c.last_pull = float('-inf')
  c.idle(None, NS(get_bool=lambda k: k == 'IsOffroad'))
  assert called
  assert not L.Client({}).enabled


def test_health_sample_reads_performance_status(tmp_path):
  status = tmp_path / 'status.json'
  status.write_text(json.dumps({'version': 1, 'sample_mono': 0, 'gpu': {'g': {'clock_mhz': 1020}},
                                'power_rails': [{'name': 'VDD_IN', 'mv': 5000, 'ma': 2000}],
                                'power_mode': {'name': 'MAXN_SUPER'}, 'cooling': [{'name': 'fan', 'state': 0},
                                                                                {'name': 'gpu-throttle', 'state': 2}]}))
  zone = tmp_path / 'thermal' / 'thermal_zone0'
  zone.mkdir(parents=True)
  (zone / 'type').write_text('gpu-thermal')
  (zone / 'temp').write_text('61500')
  row = carrot_health_sample(status, tmp_path / 'thermal')
  assert row['gpu']['g']['clock_mhz'] == 1020 and row['temp_c'] == {'gpu-thermal': 61.5}
  assert row['cooling'] == [{'name': 'gpu-throttle', 'state': 2}] and row['power_mode']['name'] == 'MAXN_SUPER'


def test_analyzer_reads_an_event(tmp_path, capsys):
  replies, transport, tr, loaded, states = run(4, True, tmp_path, frames=4)
  event = tr.jobs.get_nowait()
  tr.clock.t += 10
  tr.process(event)
  import nf_trace_analyze
  assert nf_trace_analyze.main([str(tmp_path / 'nf')]) == 0
  out = capsys.readouterr().out
  assert 'nan_again' in out and 'outputs.outputs' in out


def test_tracer_errors_never_reach_the_frame(tmp_path):
  tr = tracer(tmp_path)
  tr.before_run(NS(engine=object(), spec=None, host_inputs={}), 1, 0)   # broken loaded: swallowed
  tr.after_frame(NS(engine=object()), 1, 1, np.zeros(1, np.float32))
  assert tr.errors >= 1
