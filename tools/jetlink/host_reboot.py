"""Jetson OS reboot requested from comma Settings > Device (대감님 11/19 07:13, 08:29 "어떤 조건이든 내가 누르면 재부팅").

Comma UI (always enabled, confirm dialog) -> REQUEST_FILE -> jetlinkd (existing authenticated Jetlink session,
offroad idle loop and between onroad frames) -> REQUEST message -> Jetson server.py -> host_request()
-> 'sudo -n systemctl --no-block reboot'. No vehicle condition (gear, speed, assistance, IsOnroad, signal age).
One reboot command per Jetson server process; later requests answer 'rebooting' without a second command.
A Jetson without this file never advertises CAPABILITY, so comma reports 'old bundle'.
"""
import json
from pathlib import Path
import subprocess
import time
import uuid

CAPABILITY = 'carrot_host_reboot_v1'
REQUEST, RESPONSE = 0x4050, 0x4051
REQUEST_FILE = Path('/dev/shm/carrot-jetson-reboot-request.json')
RESULT_FILE = Path('/dev/shm/carrot-jetson-reboot-result.json')
REQUEST_TIMEOUT = 3.   # comma side: a request older than this is dropped, the UI reports "not connected"
REBOOT_COMMAND = ('sudo', '-n', 'systemctl', '--no-block', 'reboot')


# ---------------------------------------------------------------- Jetson side

_state = {'rebooting': False}


def host_request(payload, run=subprocess.run):
  try:
    if len(payload) > 256:
      raise ValueError
    request = json.loads(bytes(payload))
    if not isinstance(request, dict) or request.get('reboot') is not True or not isinstance(request.get('id'), str):
      raise ValueError
  except (ValueError, TypeError):
    return {'ok': False, 'reason': 'invalid request'}
  if _state['rebooting']:
    return {'id': request['id'], 'ok': True, 'duplicate': True}
  try:
    result = run(list(REBOOT_COMMAND), capture_output=True, text=True, timeout=5, check=False)
    if result.returncode != 0:
      return {'id': request['id'], 'ok': False, 'reason': 'reboot command failed'}
  except (OSError, subprocess.TimeoutExpired):
    return {'id': request['id'], 'ok': False, 'reason': 'reboot command failed'}
  _state['rebooting'] = True
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
  """jetlinkd: forwards a queued request (offroad idle loop and between onroad frames, any vehicle state)."""
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
