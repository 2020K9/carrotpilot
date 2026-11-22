"""Real jetlink Session + JetlinkClient over an in-process loopback with a deterministic CPU engine (host state
path), shared by the NOT_FINITE tracer tests (test_nf_trace). Ordering / values only: not GPU validation."""
from collections import deque
from types import SimpleNamespace as NS

import numpy as np

from jetlink import protocol as P
from jetlink.client import JetlinkClient
from jetlink.queues import StatefulQueues
from jetlink.server.session import Session
from jetlink.spec import ModelSpec
from jetlink.transport.base import Message, Transport

SPEC = ModelSpec(
  sha256='e' * 64, nbytes=1, frame_skip=4,
  input_shapes={'new_img': (2, 6, 4, 8), 'desire': (8,), 'traffic_convention': (1, 2), 'action_t': (1, 2),
                'state_feat_q': (4, 1, 16), 'state_desire_q': (5, 1, 8)},
  output_shapes={'outputs': (1, 24), 'next_state_feat_q': (4, 1, 16), 'next_state_desire_q': (5, 1, 8)},
  output_slices={'plan': slice(0, 24)}, checkpoint=None)


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


def pair(spec=SPEC, queues=None, engine=None, host_inputs=None):
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
  client = JetlinkClient(transport)
  client.hello()
  client.spec = spec
  return client, session, holder['loaded'], transport


def frame_inputs(i, spec=SPEC):
  warped = np.full(spec.warped_shape, (i * 7) % 251, np.uint8)
  packed = np.zeros(spec.packed_nelem, np.float32)
  packed[i % 8] = 1.   # desire one-hot follows the frame
  return warped, packed


def infer(client, i, reset=False):
  return client.infer_end(client.infer_begin(*frame_inputs(i), frame_id=i, reset=reset))
