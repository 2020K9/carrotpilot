"""K9 NOT_FINITE tracer: Jetson <-> comma side (see third_party/jetlink/jetlink/server/nf_trace.py, NF_TRACE.md).

Jetson: attach() puts one NfTrace on the EngineHost (events under <cache>/nf_trace), telemetry() adds a small
'carrot_nf_trace' summary to the state the comma already receives (only once an event exists), host_request()
answers list / file-chunk requests.
Comma (jetlinkd): Client.note() writes one 'jetsonNfTrace' rlog event per new event/replay result (onroad too, no
extra USB traffic); Client.idle() pulls the event files over the jetlink link only while offroad, a bounded amount
per call, into /data/media/0/k9_nf_trace/<event id>/.
"""
import json
import os
from pathlib import Path
import shutil
import time

CAPABILITY = 'carrot_nf_trace_v1'
REQUEST, RESPONSE = 0x4060, 0x4061
CHUNK = 256 * 1024
LOCAL = Path(os.environ.get('CARROT_NF_TRACE_LOCAL', '/data/media/0/k9_nf_trace'))
LOCAL_MAX_BYTES = 1500 << 20
PULL_BUDGET = 8 << 20     # bytes per idle call
PULL_INTERVAL = 5.


def attach(host):
  """Jetson server: the process-wide tracer (created once, on the first session)."""
  tracer = getattr(host, 'nf_trace', None)
  if tracer is None:
    try:
      from jetlink.server.nf_trace import NfTrace, carrot_health_sample
      tracer = NfTrace(Path(host.cache.root) / 'nf_trace', carrot_health_sample).bind(host)
      host.nf_trace = tracer
    except Exception:
      import logging
      logging.getLogger('jetlink.nf_trace').exception('nf_trace unavailable')
      return None
  return tracer


def telemetry(tracer):
  try:
    value = tracer.summary() if tracer is not None else None
  except Exception:
    value = None
  return {'carrot_nf_trace': value} if value else {}


def host_request(tracer, payload):
  """-> reply parts: JSON header line, then raw file bytes for 'get'."""
  try:
    if tracer is None:
      raise ValueError('tracer unavailable')
    if len(payload) > 512:
      raise ValueError('oversized request')
    request = json.loads(bytes(payload))
    if request.get('op') == 'list':
      return (json.dumps({'ok': True, 'events': tracer.listing()}).encode() + b'\n',)
    if request.get('op') == 'get':
      data = tracer.read_file(request.get('id'), request.get('name'), request.get('offset', 0), request.get('size', CHUNK))
      return (json.dumps({'ok': True, 'n': len(data)}).encode() + b'\n', data)
    raise ValueError('unknown op')
  except (OSError, ValueError, TypeError, AttributeError) as exc:
    return (json.dumps({'ok': False, 'error': f'{type(exc).__name__}: {exc}'[:160]}).encode() + b'\n',)


def split(payload):
  raw = bytes(payload)
  head, _, data = raw.partition(b'\n')
  return json.loads(head), data


def summary_line(event):
  return (f"JL_NF_TRACE id={event.get('id')} frame={event.get('frame')} model={event.get('model')} "
          + f"replay={event.get('replay')} tensors={int(bool(event.get('tensors')))} "
          + f"post={int(bool(event.get('post_complete')))}{' interrupted=1' if event.get('interrupted') else ''} "
          + f"{event.get('detail', '')}")[:400]


class Client:
  def __init__(self, peer, emit=None, local=None):
    self.enabled = peer.get(CAPABILITY) is True
    self.emit = emit
    self.local = Path(local) if local is not None else LOCAL
    self.seen = set()
    self.last_note = float('-inf')
    self.last_pull = float('-inf')
    self.done = set()

  def note(self, state):
    """One rlog line per (event, replay result). Cheap: dict lookups at most once a second."""
    now = time.monotonic()
    if self.emit is None or now - self.last_note < 1:
      return
    self.last_note = now
    try:
      value = (state or {}).get('carrot_nf_trace') or {}
      for event in value.get('events') or []:
        key = (event.get('id'), event.get('replay'), bool(event.get('post_complete')))
        if key in self.seen:
          continue
        self.seen.add(key)
        self.emit('jetsonNfTrace', line=summary_line(event), **{k: event.get(k) for k in (
          'id', 'frame', 'model', 'replay', 'tensors', 'boot', 'post_complete', 'interrupted', 'detail')})
    except Exception:
      pass  # diagnostics never interrupt jetlinkd

  def idle(self, client, params):
    if not self.enabled or time.monotonic() - self.last_pull < PULL_INTERVAL:
      return
    self.last_pull = time.monotonic()
    try:
      if not params.get_bool('IsOffroad') or params.get_bool('IsOnroad'):
        return
      self.pull(client)
    except Exception:
      pass

  def _ask(self, client, request, timeout=3.):
    seq = client._next_seq()
    client.t.send_json(REQUEST, seq, request)
    head, data = split(client._expect(RESPONSE, seq, timeout).payload)
    if not head.get('ok'):
      raise ValueError(head.get('error', 'failed'))
    return head, data

  def pull(self, client, budget=PULL_BUDGET):
    head, _ = self._ask(client, {'op': 'list'})
    moved = 0
    for event in head.get('events', []):
      eid = event['id']
      if eid in self.done or (self.local / eid / '.complete').exists():
        self.done.add(eid)
        continue
      target = self.local / eid
      target.mkdir(parents=True, exist_ok=True)
      for name, size in event['files'].items():
        path = target / name
        if path.exists() and path.stat().st_size == size and name != 'meta.json' and not name.endswith('.jsonl'):
          continue
        part = target / (name + '.part')
        offset = part.stat().st_size if part.exists() else 0
        if offset > size:
          part.unlink()
          offset = 0
        with open(part, 'ab') as f:
          while offset < size:
            if moved >= budget:
              return moved
            _, data = self._ask(client, {'op': 'get', 'id': eid, 'name': name, 'offset': offset, 'size': CHUNK})
            if not data:
              break
            f.write(data)
            offset += len(data)
            moved += len(data)
        os.replace(part, path)
      if event.get('complete'):
        (target / '.complete').write_text(json.dumps({'pulled': time.time()}))  # noqa: TID251 -- file stamp only
        self.done.add(eid)
        if self.emit is not None:
          self.emit('jetsonNfTracePulled', id=eid, path=str(target))
    self._cap()
    return moved

  def _cap(self):
    dirs = sorted((d for d in self.local.iterdir() if d.is_dir()), key=lambda d: d.stat().st_mtime)
    total = sum(f.stat().st_size for d in dirs for f in d.iterdir() if f.is_file())
    while dirs and total > LOCAL_MAX_BYTES:
      d = dirs.pop(0)
      total -= sum(f.stat().st_size for f in d.iterdir() if f.is_file())
      shutil.rmtree(d, ignore_errors=True)
