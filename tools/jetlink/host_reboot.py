"""Jetson OS reboot requested from comma Settings > Device (대감님 11/19 07:13).

Comma UI -> REQUEST_FILE -> jetlinkd (existing authenticated Jetlink session, offroad idle loop only)
-> REQUEST message -> Jetson server.py -> host_request() -> 'sudo -n systemctl --no-block reboot'.

The Jetson accepts only when, at receipt, its newest comma HUD snapshot (Jetson clock) is < MAX_AGE old and in it:
  IsOnroad is 0, and carState, selfdriveState and carControl are all fresh (< MAX_AGE, comma clock) with
  gear P, vEgo 0 and driver assistance not enabled/active.
Any missing or old signal rejects (no vehicle signal); no ignition-off shortcut (노비 11/19 07:50).
A Jetson without this file never advertises CAPABILITY, so comma reports 'old bundle'.
"""
import base64
import json
from pathlib import Path
import subprocess
import time
import uuid

CAPABILITY = 'carrot_host_reboot_v1'
REQUEST, RESPONSE = 0x4050, 0x4051
MAX_AGE = .5
REQUEST_FILE = Path('/dev/shm/carrot-jetson-reboot-request.json')
RESULT_FILE = Path('/dev/shm/carrot-jetson-reboot-result.json')
REQUEST_TIMEOUT = 3.   # comma side: a request older than this is dropped, the UI reports "not connected"
REBOOT_COMMAND = ('sudo', '-n', 'systemctl', '--no-block', 'reboot')


# ---------------------------------------------------------------- Jetson side

def _event(snapshot, name):
  from openpilot.cereal import log
  raw = snapshot.get('events', {}).get(name)
  if not isinstance(raw, str):
    return None
  with log.Event.from_bytes(base64.b64decode(raw)) as event:
    value = getattr(event, name)
    return {field: getattr(value, field) for field in FIELDS[name]}


FIELDS = {'carState': ('gearShifter', 'vEgo'), 'selfdriveState': ('enabled', 'active'),
          'carControl': ('enabled', 'latActive')}


def _age(snapshot, name):
  try:
    if snapshot['alive'].get(name) is not True:
      return None
    return float(snapshot['sent']) - float(snapshot['received'][name])
  except (KeyError, TypeError, ValueError, AttributeError):
    return None


def _fresh(snapshot, name):
  age = _age(snapshot, name)
  return age is not None and 0 <= age < MAX_AGE


def check(snapshot, snapshot_age):
  """None when a reboot is allowed, else the reason (short English, comma UI translates)."""
  if snapshot is None or snapshot_age is None or not 0 <= snapshot_age < MAX_AGE:
    return 'no vehicle signal'
  try:
    onroad = base64.b64decode(snapshot.get('params', {}).get('IsOnroad', '')).strip()
  except (ValueError, TypeError):
    onroad = b''
  if onroad != b'0':
    return 'onroad' if onroad == b'1' else 'no vehicle signal'
  try:
    if not all(_fresh(snapshot, n) for n in ('carState', 'selfdriveState', 'carControl')):
      return 'no vehicle signal'
    cs, ss, cc = (_event(snapshot, n) for n in ('carState', 'selfdriveState', 'carControl'))
    if ss['enabled'] or ss['active'] or cc['enabled'] or cc['latActive']:
      return 'driver assistance active'
    if str(cs['gearShifter']) != 'park':
      return 'not in P'
    if cs['vEgo'] != 0:
      return 'moving'
    return None
  except (KeyError, TypeError, ValueError, AttributeError):
    return 'no vehicle signal'


def latest_snapshot(packet):
  """(snapshot, age on the Jetson clock) of the HUD packet server.py writes, or (None, None)."""
  from hud_protocol import HEADER, MAX_HUD_BYTES
  try:
    with Path(packet).open('rb') as f:
      data = f.read(MAX_HUD_BYTES + HEADER.size + 1)
    received, = HEADER.unpack_from(data)
    return json.loads(data[HEADER.size:]), time.monotonic() - received
  except Exception:  # unreadable/partial packet = no signal
    return None, None


def host_request(payload, packet, run=subprocess.run):
  try:
    if len(payload) > 256:
      raise ValueError
    request = json.loads(bytes(payload))
    if not isinstance(request, dict) or request.get('reboot') is not True or not isinstance(request.get('id'), str):
      raise ValueError
  except (ValueError, TypeError):
    return {'ok': False, 'reason': 'invalid request'}
  snapshot, age = latest_snapshot(packet)
  reason = check(snapshot, age)
  if reason is not None:
    return {'id': request['id'], 'ok': False, 'reason': reason}
  try:
    result = run(list(REBOOT_COMMAND), capture_output=True, text=True, timeout=5, check=False)
    if result.returncode != 0:
      return {'id': request['id'], 'ok': False, 'reason': 'reboot command failed'}
  except (OSError, subprocess.TimeoutExpired):
    return {'id': request['id'], 'ok': False, 'reason': 'reboot command failed'}
  return {'id': request['id'], 'ok': True}


# ---------------------------------------------------------------- comma side

def _write(path, value):
  tmp = path.with_suffix('.tmp')
  tmp.write_text(json.dumps(value))
  tmp.replace(path)


def _read(path):
  try:
    value = json.loads(path.read_text())
    return value if isinstance(value, dict) else None
  except (OSError, ValueError):
    return None


def submit(now=None):
  """Comma UI: queue one request, return its id."""
  request_id = uuid.uuid4().hex
  RESULT_FILE.unlink(missing_ok=True)
  _write(REQUEST_FILE, {'id': request_id, 'time': time.monotonic() if now is None else now})
  return request_id


def status(request_id, requested_at, now=None):
  """Comma UI: (state, reason). state: pending / sent / rejected / unsupported / disconnected."""
  now = time.monotonic() if now is None else now
  result = _read(RESULT_FILE)
  if result is not None and result.get('id') == request_id:
    return result.get('state', 'rejected'), result.get('reason', '')
  if now - requested_at > REQUEST_TIMEOUT + .5:
    return 'disconnected', ''
  return 'pending', ''


class Client:
  """jetlinkd: forwards a queued request in the offroad idle loop."""
  def __init__(self, peer):
    self.enabled = isinstance(peer, dict) and peer.get(CAPABILITY) is True
    self.is_jetson = isinstance(peer, dict) and peer.get('carrot_host') == 'jetson'

  def idle(self, client, params, now=None):
    request = _read(REQUEST_FILE)
    if request is None:
      return
    REQUEST_FILE.unlink(missing_ok=True)
    now = time.monotonic() if now is None else now
    try:
      if not 0 <= now - float(request['time']) < REQUEST_TIMEOUT or not isinstance(request['id'], str):
        return
    except (KeyError, TypeError, ValueError):
      return
    result = {'id': request['id']}
    if not self.is_jetson:
      result.update(state='disconnected', reason='')
    elif not self.enabled:
      result.update(state='unsupported', reason='old jetson bundle')
    elif params.get_bool('IsOnroad') or not params.get_bool('IsOffroad'):
      result.update(state='rejected', reason='onroad')
    else:
      try:
        seq = client._next_seq()
        client.t.send_json(REQUEST, seq, {'reboot': True, 'id': request['id']})
        reply = json.loads(bytes(client._expect(RESPONSE, seq, 3.).payload))
        if reply.get('ok') is True:
          result.update(state='sent', reason='')
        else:
          result.update(state='rejected', reason=str(reply.get('reason', ''))[:60])
      except Exception:
        result.update(state='disconnected', reason='')
    _write(RESULT_FILE, result)
