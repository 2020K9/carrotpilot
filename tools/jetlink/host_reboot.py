"""Jetson OS reboot requested from comma Settings > Device (대감님 11/19 07:13, 08:29 "어떤 조건이든 내가 누르면 재부팅").

Comma UI (always enabled, confirm dialog) -> REQUEST_FILE -> jetlinkd (existing authenticated Jetlink session,
offroad idle loop and between onroad frames) -> REQUEST message -> Jetson server.py -> host_request()
-> 'sudo -n systemctl --no-block reboot'. No vehicle condition (gear, speed, assistance, IsOnroad, signal age).
One reboot command per Jetson server process; later requests answer 'rebooting' without a second command.
A Jetson without this file never advertises CAPABILITY, so comma reports 'old bundle'.

Clean reboot (대감님 11/26 18:50 "gpu.usb먼저 초기화하고 재부팅되게"): on a Jetson the request starts one root
transient unit (PREPARE_COMMAND) that, after the reply is sent, stops the inference/HUD services, idles the nvgpu
GPU, de-authorizes USB devices and switches USB port power off, then reboots. Every step is bounded and may fail;
the reboot always follows (plus a 5 s fallback reboot timer, 대감님 11/26 19:57 / 22:19). If the unit cannot be started, the plain
REBOOT_COMMAND runs as before.

Boot reset (대감님 11/26 19:57): after the comma boots with a new git commit, a new comma model (DrivingModelName)
or a new Jetson model (frozen boot selection), jetlinkd sends the same REQUEST once as soon as the Jetson is
connected (no parked / drive-waiting condition, 대감님 11/26 21:51), so the Jetson runs the same clean reboot as the button. The fingerprint is stored in BOOT_STATE_FILE; an ordinary start (same fingerprint)
does nothing, and one request per comma boot at most.
"""
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

CAPABILITY = 'carrot_host_reboot_v1'
REQUEST, RESPONSE = 0x4050, 0x4051
REQUEST_FILE = Path('/dev/shm/carrot-jetson-reboot-request.json')
RESULT_FILE = Path('/dev/shm/carrot-jetson-reboot-result.json')
REQUEST_TIMEOUT = 3.   # comma side: a request older than this is dropped, the UI reports "not connected"
REBOOT_COMMAND = ('sudo', '-n', 'systemctl', '--no-block', 'reboot')
PREPARE_UNIT = 'carrot-jetson-reboot-prepare'
PREPARE_COMMAND = ('sudo', '-n', 'systemd-run', '--no-block', '--collect', f'--unit={PREPARE_UNIT}',
                   sys.executable, str(Path(__file__).resolve()), '--prepare-and-reboot')
FALLBACK_SECONDS = 5   # 대감님 11/26 22:19 (button and automatic reboot)
BOOT_STATE_FILE = Path('/data/carrot_jetson_boot_reset.json')
JETSON_MARKER = Path('/etc/nv_tegra_release')
STOP_UNITS = ('carrot-jetlink-hud.service', 'carrot-jetlink.service')


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
  if JETSON_MARKER.exists() and _ok(run, PREPARE_COMMAND):
    _state['rebooting'] = True
    return {'id': request['id'], 'ok': True}
  if not _ok(run, REBOOT_COMMAND):
    return {'id': request['id'], 'ok': False, 'reason': 'reboot command failed'}
  _state['rebooting'] = True
  return {'id': request['id'], 'ok': True}


def _ok(run, command, timeout=5):
  try:
    return run(list(command), capture_output=True, text=True, timeout=timeout, check=False).returncode == 0
  except (OSError, subprocess.SubprocessError):
    return False


def _sysfs_write(path, value):
  try:
    path.write_text(value)
    return True
  except OSError:
    return False


def reset_gpu(sysfs=Path('/sys')):
  """nvgpu (Jetson iGPU) has no nvidia-smi reset; force_idle is the driver's own idle/reset hook."""
  done = 0
  for attr in sorted(sysfs.glob('devices/platform/**/force_idle')):
    if (attr.parent / 'railgate_enable').exists() or 'gpu' in attr.parent.name:
      done += _sysfs_write(attr, '1')
  return done


def reset_usb(sysfs=Path('/sys')):
  """De-authorize every non-root USB device (deepest first), then switch port power off where possible."""
  devices = sysfs / 'bus/usb/devices'
  names = [p.name for p in devices.glob('*') if ':' not in p.name and not p.name.startswith('usb')]
  deauth = sum(_sysfs_write(devices / n / 'authorized', '0') for n in sorted(names, key=len, reverse=True))
  ports = sum(_sysfs_write(p, '1') for p in sorted(devices.glob('*/*-port*/disable')))
  return deauth, ports


def prepare_and_reboot(run=subprocess.run, sysfs=Path('/sys'), sleep=time.sleep, log=print):
  """Root transient unit: 1 stop inference, 2 GPU, 3 USB, 4 reboot. The reboot is never skipped."""
  steps = []
  try:
    steps.append(('fallback', _ok(run, ('systemd-run', f'--on-active={FALLBACK_SECONDS}', '--collect',
                                        f'--unit={PREPARE_UNIT}-fallback', 'systemctl', '--no-block', 'reboot'))))
    sleep(1.)   # the RESPONSE reaches the comma before the server stops
    steps.append(('stop', _ok(run, ('systemctl', 'stop', *STOP_UNITS), timeout=30)))
    steps.append(('gpu', reset_gpu(sysfs)))
    steps.append(('usb', reset_usb(sysfs)))
    os.sync()
  except Exception as exc:   # noqa: BLE001 -- any failure still reboots
    steps.append(('error', repr(exc)[:120]))
  finally:
    rebooted = _ok(run, ('systemctl', '--no-block', 'reboot'))
    if not rebooted:
      rebooted = _ok(run, ('systemctl', '--force', 'reboot'))
    steps.append(('reboot', rebooted))
    log(json.dumps({'carrot_reboot_prepare': steps}))
  return steps


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
    self.boot_checked = False

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
    _write(RESULT_FILE, self._send(client, request['id']))

  def boot_reset(self, client, fingerprint=None):
    """Once per comma boot: same clean Jetson reboot after git pull / comma model / Jetson model change."""
    if self.boot_checked or not self.is_jetson:
      return None
    self.boot_checked = True
    try:
      current = boot_fingerprint() if fingerprint is None else fingerprint
      if _read(BOOT_STATE_FILE) == current:
        return None
      _write(BOOT_STATE_FILE, current)   # marked first: never a second automatic reboot for this state
    except Exception:
      return None
    return self._send(client, 'boot-' + uuid.uuid4().hex)

  def _send(self, client, request_id):
    result = {'id': request_id}
    if not self.is_jetson:
      result.update(state='disconnected', reason='')
    elif not self.enabled:
      result.update(state='unsupported', reason='old jetson bundle')
    else:
      try:
        seq = client._next_seq()
        client.t.send_json(REQUEST, seq, {'reboot': True, 'id': request_id})
        reply = json.loads(bytes(client._expect(RESPONSE, seq, 3.).payload))
        if reply.get('ok') is True:
          result.update(state='sent', reason='')
        else:
          result.update(state='rejected', reason=str(reply.get('reason', ''))[:60])
      except Exception:
        result.update(state='disconnected', reason='')
    return result


def boot_fingerprint():
  """(comma git commit, comma model, Jetson model) of this comma boot."""
  root = Path(__file__).resolve().parents[2]
  try:
    commit = subprocess.run(['git', '-C', str(root), 'rev-parse', 'HEAD'], capture_output=True, text=True,
                            timeout=5, check=False).stdout.strip()
  except (OSError, subprocess.SubprocessError):
    commit = ''
  from openpilot.common.params import Params
  from openpilot.common.big_model_catalog import effective_id
  comma_model = Params().get('DrivingModelName') or ''
  if isinstance(comma_model, bytes):
    comma_model = comma_model.decode(errors='replace')
  return {'commit': commit, 'comma_model': str(comma_model), 'jetson_model': effective_id()}


if __name__ == '__main__' and sys.argv[1:] == ['--prepare-and-reboot']:
  prepare_and_reboot()
