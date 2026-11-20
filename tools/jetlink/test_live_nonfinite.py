"""Injected errors through the actual server reply and Comma infer_end."""
import time
from types import SimpleNamespace as NS
import numpy as np
import pytest
from test_stateful_protocol import _stateful_spec
from jetlink import protocol as P
from jetlink.client import JetlinkClient
from jetlink.queues import StatefulQueues
from jetlink.server.session import Session
from jetlink.transport.base import LinkError, Message


@pytest.mark.parametrize('peer', [False, True])
@pytest.mark.parametrize('name,x,kind', [('outputs', np.inf, '+Inf'), ('outputs', -np.inf, '-Inf'), ('next_state_features', np.nan, 'NaN')])
def test_failure_visible_in_host_and_opt_in_comma_logs(caplog, peer, name, x, kind):
  spec = _stateful_spec()
  buffers = {n: np.zeros(s, np.float16) for n, s in spec.input_shapes.items()}
  outputs = {n: np.zeros(s, np.float16) for n, s in spec.output_shapes.items()}
  outputs[name].flat[2100] = x
  replies = []
  t = NS(protocol_version=2, send=lambda k, seq, parts, flags: replies.append(Message(k, seq, flags, memoryview(b''.join(parts)))))
  session = Session(t, NS(telemetry=NS(read=dict), frame_stats=NS(record=lambda *a: None)))
  session._finite_detail_peer = peer
  q = StatefulQueues(spec, buffers)
  loaded = NS(spec=spec, host_inputs=buffers, queues=q, engine=NS(run=lambda: outputs, last_gpu_us=3))
  session._infer(loaded, NS(seq=11, payload=memoryview(P.pack_infer_req(42,0) + bytes(spec.warped_nbytes+spec.packed_nbytes))))
  assert P.unpack_infer_resp(replies[0].payload)[:2] == (42, P.Status.NOT_FINITE)
  stage = 'output' if name == 'outputs' else 'feedback'
  detail = f'JL_NF frame=42 model={spec.sha256[:12]} stage={stage} tensor={name} index=2100 value={kind} count=1'
  assert detail in caplog.text
  c = JetlinkClient(t); c.spec = spec; c._infer_started=time.monotonic(); c._expect=lambda *a: replies[0]
  with pytest.raises(LinkError) as error: c.infer_end(11)
  assert c.dead and 'NOT_FINITE' in str(error.value)
  assert (detail in str(error.value)) == peer
  if not peer: assert len(replies[0].payload) == P.INFER_RESP_SIZE + spec.output_nbytes
