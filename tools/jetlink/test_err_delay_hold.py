"""K9JetErrDelay (실험중 '젯슨 에러지연', 대감님 2026-11-21 11:09): one non-finite Jetson frame restores the previous valid
recurrent state instead of clearing it (Flag.HOLD_STATE / Flag.STATE_HELD, HELLO 'carrot_hold_state_v1').

Real Session + JetlinkClient over an in-process loopback with a deterministic CPU engine (host state path,
CARROT_TRT_DEVICE_STATE=0), and the TensorRT device-state copy on test_device_state's deferred-CUDA fake (GPU state
path, CARROT_TRT_DEVICE_STATE=1). Ordering / values only: not GPU validation, no Jetson timing."""
from collections import deque
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from jetlink import protocol as P
from jetlink.client import JetlinkClient, HeldOutput, NonFiniteOutput
from jetlink.queues import StatefulQueues
from jetlink.server.session import Session
from jetlink.spec import ModelSpec
from jetlink.transport.base import LinkError, Message, Transport

from test_device_state import make_engine, runtime  # noqa: F401  (deferred-CUDA TensorRT fake)

SPEC = ModelSpec(
  sha256='e' * 64, nbytes=1, frame_skip=4,
  input_shapes={'new_img': (2, 6, 4, 8), 'desire': (8,), 'traffic_convention': (1, 2), 'action_t': (1, 2),
                'state_feat_q': (4, 1, 16), 'state_desire_q': (5, 1, 8)},
  output_shapes={'outputs': (1, 24), 'next_state_feat_q': (4, 1, 16), 'next_state_desire_q': (5, 1, 8)},
  output_slices={'plan': slice(0, 24)}, checkpoint=None)
POISON = ('outputs', 'state', 'both')


class Engine:
  """next state = f(input state, frame image, desire); output = f(next state). poison breaks this frame."""
  def __init__(self, inputs):
    self.inputs = inputs
    self.outputs = {k: np.zeros(v, np.float32) for k, v in SPEC.output_shapes.items()}
    self.poison = None
    self.seen = []        # the recurrent state each run actually received
    self.last_gpu_us = 1000

  def run(self):
    feat, des = self.inputs['state_feat_q'].copy(), self.inputs['state_desire_q'].copy()
    self.seen.append((feat, des))
    img = float(self.inputs['new_img'].astype(np.float64).mean())
    self.outputs['next_state_feat_q'][...] = feat * .5 + img + 1.
    self.outputs['next_state_desire_q'][...] = des * .25 + self.inputs['desire']
    self.outputs['outputs'][...] = self.outputs['next_state_feat_q'].sum() + np.arange(24, dtype=np.float32)
    if self.poison in ('outputs', 'both'):
      self.outputs['outputs'].flat[3] = np.nan
    if self.poison in ('state', 'both'):
      self.outputs['next_state_feat_q'].flat[5] = np.inf
    return self.outputs


class Loopback(Transport):
  protocol_version = 2

  def __init__(self, handler):
    self.replies = deque()
    self.handler = handler
    self.flags = []

  def send(self, kind, seq, parts=(), flags=0, timeout=None):
    payload = bytearray(b''.join(memoryview(p).cast('B') for p in parts))
    message = Message(kind, seq, flags, memoryview(payload))
    if kind in (P.Msg.HELLO_REQ, P.Msg.INFER_REQ):
      self.handler(message)
    else:
      if kind == P.Msg.INFER_RESP:
        self.flags.append(flags)
      self.replies.append(message)

  def recv(self, timeout=None):
    return self.replies.popleft()

  def close(self):
    pass


def pair(spec=SPEC, queues=None, engine=None, host_inputs=None, capability=True):
  host = NS(telemetry=NS(read=dict), backend=NS(describe=dict), status=lambda *a: {'state': 'ready'},
            loaded_sha=lambda: spec.sha256, cache=NS(inventory=list), sleep_after=0,
            frame_stats=NS(record=lambda *a: None))
  holder = {}

  def handle(message):
    if message.msg_type == P.Msg.HELLO_REQ:
      session._greet(message)
      session.on_hello(message)
    else:
      session._infer(holder['loaded'], message)

  transport = Loopback(handle)
  session = Session(transport, host)
  if queues is None:
    host_inputs = {k: np.zeros(v, np.float32) for k, v in spec.input_shapes.items()}
    queues = StatefulQueues(spec, host_inputs)
    engine = Engine(host_inputs)
  holder['loaded'] = NS(spec=spec, host_inputs=host_inputs, queues=queues, engine=engine)
  if not capability:   # an older Jetson bundle: the same server without the hello key
    original = session._send_json

    def old_hello(msg_type, seq, obj, flags=0):
      if msg_type == P.Msg.HELLO_RESP:
        obj = {k: v for k, v in obj.items() if k != P.HOLD_STATE_CAPABILITY}
      return original(msg_type, seq, obj, flags)
    session._send_json = old_hello
  client = JetlinkClient(transport)
  client.hello()
  client.spec = spec
  return client, session, holder['loaded'], transport


def frame_inputs(i, spec=SPEC):
  warped = np.full(spec.warped_shape, (i * 7) % 251, np.uint8)
  packed = np.zeros(spec.packed_nelem, np.float32)
  packed[i % 8] = 1.   # desire one-hot follows the frame
  return warped, packed


def infer(client, i, hold, reset=False):
  return client.infer_end(client.infer_begin(*frame_inputs(i), frame_id=i, reset=reset, **({'hold': True} if hold else {})))


def test_hello_offers_hold_and_an_older_bundle_does_not():
  client, *_ = pair()
  assert client.hold_state_available
  old, *_ = pair(capability=False)
  assert not old.hold_state_available
  with pytest.raises(LinkError, match='not negotiated'):
    old.infer_begin(*frame_inputs(1), frame_id=1, hold=True)


@pytest.mark.parametrize('poison', POISON)
def test_one_nonfinite_frame_restores_the_last_valid_state(poison):
  client, _, loaded, transport = pair()
  engine = loaded.engine
  kept = None
  for i in range(1, 4):
    infer(client, i, True, reset=i == 1)
    kept = [v.copy() for v in loaded.queues.state.values()]   # the state after frame 3 = frame 4's input
  engine.poison = poison
  with pytest.raises(HeldOutput, match=r'NOT_FINITE \(frame 4\)'):
    infer(client, 4, True)
  assert not client.dead and transport.flags[-1] == P.Flag.STATE_HELD
  broken = [engine.outputs[k].copy() for k in ('next_state_feat_q', 'next_state_desire_q')]
  engine.poison = None
  out = infer(client, 5, True)
  fed = engine.seen[-1]
  for a, b in zip(fed, kept, strict=True):
    assert a.tobytes() == b.tobytes()                       # frame 5 ran on the state after frame 3
  assert all(np.isfinite(a).all() for a in fed)            # nothing of the broken frame reached frame 5
  if poison != 'outputs':
    assert fed[0].tobytes() != broken[0].tobytes()
  assert np.isfinite(out).all() and transport.flags[-1] == 0


@pytest.mark.parametrize('poison', POISON)
def test_two_in_a_row_or_no_flag_clear_the_state_like_before(poison):
  client, _, loaded, transport = pair()
  for i in range(1, 4):
    infer(client, i, True, reset=i == 1)
  loaded.engine.poison = poison
  with pytest.raises(HeldOutput):
    infer(client, 4, True)
  with pytest.raises(NonFiniteOutput, match=r'^inference failed: NOT_FINITE \(frame 5\) JL_NF frame=5 ') as second:
    infer(client, 5, True)
  assert not isinstance(second.value, HeldOutput) and client.dead and transport.flags[-1] == 0
  assert not any(np.any(v) for v in loaded.queues.state.values())   # cleared, as before

  client, _, loaded, transport = pair()
  for i in range(1, 4):
    infer(client, i, False, reset=i == 1)
  loaded.engine.poison = poison
  with pytest.raises(NonFiniteOutput, match=r'^inference failed: NOT_FINITE \(frame 4\) JL_NF frame=4 ') as plain:
    infer(client, 4, False)
  assert not isinstance(plain.value, HeldOutput) and client.dead and transport.flags[-1] == 0
  assert not any(np.any(v) for v in loaded.queues.state.values())


def test_valid_frames_are_bitwise_the_same_with_and_without_the_flag():
  runs = []
  for hold in (False, True):
    client, _, loaded, _ = pair()
    rows = []
    for i in range(1, 40):
      out = infer(client, i, hold, reset=i in (1, 17))
      rows.append((out.tobytes(), *(v.tobytes() for v in loaded.queues.state.values()),
                   *(a.tobytes() for a in loaded.engine.seen[-1])))
    runs.append(rows)
  assert runs[0] == runs[1]


def test_alternating_frames_each_restore_on_the_server_the_comma_decides():
  client, _, loaded, transport = pair()
  for i in range(1, 4):
    infer(client, i, True, reset=i == 1)
  for i, bad in ((4, True), (5, False), (6, True), (7, False)):
    loaded.engine.poison = 'both' if bad else None
    if bad:
      with pytest.raises(HeldOutput):   # the server keeps rolling back; 20-frame / 1 s rule = output_recovery
        infer(client, i, True)
    else:
      infer(client, i, True)
  assert transport.flags[-4:] == [P.Flag.STATE_HELD, 0, P.Flag.STATE_HELD, 0]


@pytest.mark.parametrize('case', ['reset', 'gap', 'hello', 'unflagged'])
def test_never_restores_a_state_from_another_history(case):
  client, _, loaded, transport = pair()
  for i in range(1, 4):
    infer(client, i, True, reset=i == 1)
  nxt = 4
  if case == 'gap':
    nxt = 5                       # frame 4 never came: the kept state is not the previous frame's
  elif case == 'hello':
    client.hello()                # a new client process: its own history
  elif case == 'unflagged':
    infer(client, 4, False)       # setting off for one frame: nothing kept for frame 5
    nxt = 5
  loaded.engine.poison = 'state'
  with pytest.raises(NonFiniteOutput) as exc:
    infer(client, nxt, True, reset=case == 'reset')
  assert not isinstance(exc.value, HeldOutput) and transport.flags[-1] == 0
  assert not any(np.any(v) for v in loaded.queues.state.values())


# ---- GPU state path (CARROT_TRT_DEVICE_STATE=1): the device copy, on the deferred-CUDA fake ------------------------

def device_pair(runtime, monkeypatch, graph):  # noqa: F811
  e, q, host = make_engine(runtime, True, monkeypatch)   # DeviceStateQueues from the server's make_queues
  if graph:
    q.step_into(np.zeros((2, 2), np.uint8), np.zeros(2, np.float32), host)
    e.warm()
    q.reset()
  spec = NS(sha256='d' * 64, stateful=True, state_pairs={'state_a': 'next_state_a', 'state_b': 'next_state_b'},
            input_shapes=e.input_shapes, output_shapes=e.output_shapes, warped_shape=(2, 2), warped_nbytes=4,
            packed_shapes={'desire': (1, 2)}, packed_sizes=[2], packed_nelem=2, packed_nbytes=8, output_nelem=2,
            frame_skip=4, output_slices={})
  client, session, loaded, transport = pair(spec=spec, queues=q, engine=e, host_inputs=host)
  return client, loaded, transport, runtime


def dev_state(runtime, e, name):  # noqa: F811
  b = e.bindings[name]
  return np.frombuffer(runtime.cuda.memory[b.device_ptr], dtype=b.dtype).reshape(b.shape).copy()


def dev_infer(client, i, hold, reset=False):
  warped = np.full((2, 2), i, np.uint8)
  packed = np.array([i / 4, 0], np.float32)
  return client.infer_end(client.infer_begin(warped, packed, frame_id=i, reset=reset, **({'hold': True} if hold else {})))


@pytest.mark.parametrize('graph', [False, True])
@pytest.mark.parametrize('bad', ['outputs', 'next_state_a', 'next_state_b'])
def test_gpu_state_restore_feeds_the_last_valid_state(runtime, monkeypatch, graph, bad):  # noqa: F811
  client, loaded, transport, rt = device_pair(runtime, monkeypatch, graph)
  e = loaded.engine
  for i in range(1, 4):
    dev_infer(client, i, True, reset=i == 1)
  rt.cuda.stream_sync(e.stream)
  kept = {n: dev_state(rt, e, n) for n in ('state_a', 'state_b')}   # frame 4's input = after frame 3
  e.bad_name = bad
  with pytest.raises(HeldOutput):
    dev_infer(client, 4, True)
  assert transport.flags[-1] == P.Flag.STATE_HELD and e.device_state.needs_restore and not e.device_state.needs_reset
  e.bad_name = None
  calls = len(rt.cuda.calls)
  dev_infer(client, 5, True)
  restore = [c for c in rt.cuda.calls[calls:] if c[0] == 'd2d'][:2]
  assert [c[1] for c in restore] == [e.bindings['state_a'].device_ptr, e.bindings['state_b'].device_ptr]
  # frame 5 ran on frame 3's next state: recompute it from the kept values with the fake's own rule
  rt.cuda.stream_sync(e.stream)
  delta = np.float32(5.0) + np.float32(5 / 4)   # the fake engine: next = state + mean(image) + desire[0]
  for name in ('state_a', 'state_b'):
    expected = (kept[name].astype(np.float32) + delta).astype(kept[name].dtype)
    np.testing.assert_array_equal(dev_state(rt, e, name), expected)
  assert np.isfinite(dev_state(rt, e, 'state_a')).all() and np.isfinite(dev_state(rt, e, 'state_b')).all()
  e.close()
  assert not rt.cuda.memory and not rt.cuda.pending


@pytest.mark.parametrize('graph', [False, True])
def test_gpu_valid_frames_identical_and_copy_cost_only_when_requested(runtime, monkeypatch, graph):  # noqa: F811
  results, copies = [], []
  for hold in (False, True):
    client, loaded, _, rt = device_pair(runtime, monkeypatch, graph)
    e = loaded.engine
    start = len(rt.cuda.calls)
    values = [dev_infer(client, i, hold, reset=i == 1).tobytes() for i in range(1, 9)]
    d2d = [c for c in rt.cuda.calls[start:] if c[0] == 'd2d']
    copies.append(sum(c[3] for c in d2d))
    results.append(values)
    state_bytes = sum(e.bindings[n].nbytes for n in ('state_a', 'state_b'))
    e.close()
  assert results[0] == results[1]
  feedback = 0 if graph else 8 * state_bytes   # in a captured graph the per-frame feedback D2D is inside the graph
  assert copies[0] == feedback and copies[1] == feedback + 8 * state_bytes   # one extra copy per valid frame, only with the flag


def test_gpu_two_in_a_row_reset_like_before(runtime, monkeypatch):  # noqa: F811
  client, loaded, transport, rt = device_pair(runtime, monkeypatch, True)
  e = loaded.engine
  for i in range(1, 4):
    dev_infer(client, i, True, reset=i == 1)
  e.bad_name = 'next_state_a'
  with pytest.raises(HeldOutput):
    dev_infer(client, 4, True)
  with pytest.raises(NonFiniteOutput) as exc:
    dev_infer(client, 5, True)
  assert not isinstance(exc.value, HeldOutput) and e.device_state.needs_reset and not e.device_state.needs_restore
  e.close()


def test_session_logs_one_line_per_held_frame(caplog):
  client, _, loaded, _ = pair()
  for i in range(1, 4):
    infer(client, i, True, reset=i == 1)
  loaded.engine.poison = 'outputs'
  with caplog.at_level('WARNING', logger='jetlink.server'), pytest.raises(HeldOutput):
    infer(client, 4, True)
  lines = [r.getMessage() for r in caplog.records if 'K9JetErrDelay' in r.getMessage()]
  assert lines == ['frame 4 non-finite: recurrent state held at frame 3 (K9JetErrDelay)']
  assert json.dumps(lines)  # plain text, one line
