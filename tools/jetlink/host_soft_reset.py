"""Jetson soft reset button below 'Jetson Reboot' (대감님 11/26 23:21 "젤슨 재부팅 버튼 밑에 만들어놔").

Recover a Jetson error without an OS reboot. Web button -> SOFT_REQUEST_FILE -> jetlinkd (host_reboot.Client.idle,
same places as the reboot button) -> REQUEST {'soft_reset': true} -> Jetson server.py -> one root transient unit
(UNIT) that runs, in this order:
  1 stop the Jetson services (inference + HUD)
  2 GPU: the reboot button's force_idle (host_reboot.reset_gpu), then check that no process still holds the GPU.
    GPU not released -> full reboot at once.
  3 USB: de-authorize / port power off (host_reboot.reset_usb), then attach again
  4 receive the comma's selected release again (the boot gate session, boot_update.Gate) and compare the
    installed bundle and model sha256 with it
  5 start the services, wait for: comma link, map video and model output all alive
Not healthy within DEADLINE_SECONDS of the press -> the existing full reboot (host_reboot.prepare_and_reboot).
A systemd timer (FALLBACK_SECONDS) runs that full reboot too if this unit itself hangs; it is cancelled on success.
Progress (step 1..5, running / done / reboot) is in PROGRESS; the comma reads it from the server HELLO and from
{'soft_status': true} requests and shows the step on the web. A boot gate HELLO during a soft reset = step 4
(boot_update.py belongs to the installed updater and is not changed).
"""
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import uuid

try:
  import host_reboot as R
except ImportError:
  from tools.jetlink import host_reboot as R  # noqa: TID251 -- comma side

CAPABILITY = 'carrot_host_soft_reset_v1'
UNIT = 'carrot-jetson-soft-reset'
DEADLINE_SECONDS = 30
FALLBACK_SECONDS = DEADLINE_SECONDS + 10
PROGRESS = Path('/run/carrot-jetson-soft-reset.json')
START_UNITS = ('carrot-jetlink.service', 'carrot-jetlink-hud.service')
HUD_STATUS = Path('/dev/shm/carrot-jetlink-hud-status.json')
HOST_HEALTH = Path('/dev/shm/carrot-jetlink-health.json')
GPU_RELEASE_SECONDS = 5.
SOFT_REQUEST_FILE = Path('/dev/shm/carrot-jetson-soft-reset-request.json')
SOFT_RESULT_FILE = Path('/dev/shm/carrot-jetson-soft-reset-result.json')
STATUS_POLL_SECONDS = 1.
_ID = re.compile(r'^[0-9a-f]{1,64}$')


def command(request_id):
  return ('sudo', '-n', 'systemd-run', '--no-block', '--collect', f'--unit={UNIT}',
          sys.executable, str(Path(__file__).resolve()), '--soft-reset', request_id)


# ---------------------------------------------------------------- Jetson side (server.py, jetlink user)

def host_request(request, run=subprocess.run):
  """{'soft_reset': true, 'id'} starts the unit; {'soft_status': true, 'id'} returns PROGRESS."""
  request_id = request.get('id')
  if not isinstance(request_id, str) or not _ID.match(request_id):
    return {'ok': False, 'reason': 'invalid request'}
  if request.get('soft_status') is True:
    return {'id': request_id, 'ok': True, 'progress': progress()}
  if R._state['rebooting']:
    return {'id': request_id, 'ok': False, 'reason': 'rebooting'}
  if not R.JETSON_MARKER.exists():
    return {'id': request_id, 'ok': False, 'reason': 'not a jetson'}
  if not R._ok(run, command(request_id)):
    return {'id': request_id, 'ok': False, 'reason': 'soft reset command failed'}
  return {'id': request_id, 'ok': True}


def progress(path=None):
  value = R._read(PROGRESS if path is None else path)
  return value if value is not None else {}


# ---------------------------------------------------------------- Jetson side (root unit)

def _report(request_id, started, clock):
  def report(step, state, detail='', health=None):
    value = {'id': request_id, 'step': step, 'state': state, 'detail': detail[:80],
             'elapsed': round(clock() - started, 1)}
    if health is not None:
      value['health'] = health
    try:
      R._write(PROGRESS, value)
      PROGRESS.chmod(0o644)   # server.py (jetlink user) reads it
    except OSError:
      pass
  return report


def gpu_holders(proc=Path('/proc')):
  """PIDs that still have an nvgpu device node open (other than this process)."""
  holders = []
  for pid in proc.glob('[0-9]*'):
    if pid.name == str(os.getpid()):
      continue
    try:
      for fd in (pid / 'fd').iterdir():
        target = os.readlink(fd)
        if target.startswith('/dev/nvgpu') or (target.startswith('/dev/nvhost-') and target.endswith('gpu')):
          holders.append(int(pid.name))
          break
    except OSError:
      continue
  return holders


def release_gpu(sysfs, proc, sleep, clock):
  """Step 2: force_idle (same as the reboot button) and no GPU user left. (idled attrs, holders)."""
  idled = R.reset_gpu(sysfs)
  until = clock() + GPU_RELEASE_SECONDS
  holders = gpu_holders(proc)
  while holders and clock() < until:
    sleep(.2)
    holders = gpu_holders(proc)
  return idled, holders


def reattach_usb(sysfs, sleep):
  """Step 3: the reboot button's USB detach, then port power on and authorize again (shallowest first)."""
  devices = sysfs / 'bus/usb/devices'
  names = [p.name for p in devices.glob('*') if ':' not in p.name and not p.name.startswith('usb')]
  detached = R.reset_usb(sysfs)
  sleep(1.)
  ports = sum(R._sysfs_write(p, '0') for p in sorted(devices.glob('*/*-port*/disable')))
  sleep(.5)
  authorized = sum(R._sysfs_write(devices / n / 'authorized', '1') for n in sorted(names, key=len))
  return detached, (ports, authorized)


def verify_release(manifest, root):
  """Step 4 hash check: installed bundle + model against the comma's selected release. '' = same."""
  import update_host as update
  delivery = manifest.get('delivery')
  if delivery == 'usb':
    from usb_release import registered
    return '' if registered(root, manifest) else 'usb package hash differs'
  if delivery == 'local':
    from local_release import resolve, verify_assets
    verify_assets(resolve(root, manifest))
    return ''
  current = (root / 'current').resolve()
  update.check_bundle(current, manifest)
  if (current / '.bundle-sha256').read_text().strip() != manifest['bundle']['sha256']:
    return 'bundle sha256 differs'
  if update.digest(update.model_path(manifest['model'])) != manifest['model']['sha256']:
    return 'model sha256 differs'
  return ''


def release_from_comma(timeout, clock=time.monotonic):
  """Step 4: the boot gate session again (USB, comma sends its pin), install if different, then hash check."""
  import fcntl
  import boot_update
  import update_host as update
  sys.path.insert(0, str(update.ROOT / 'current/third_party/jetlink'))
  from jetlink.transport.usbbulk import UsbBulkTransport
  from jetlink.transport.base import LinkTimeout
  until = clock() + timeout
  with Path('/run/carrot-jetlink-update.lock').open('w') as lock:
    try:
      fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
      return 'update busy'
    boot_update.READY.unlink(missing_ok=True)
    gate = boot_update.Gate()
    transport = session = None
    try:
      while not gate.done:
        if clock() >= until:
          return 'no release from comma'
        try:
          if transport is None:
            transport = UsbBulkTransport.open(timeout_ms=1000)
            session = boot_update.BootstrapSession(transport, gate)
          try:
            session.handle(transport.recv(timeout=.25))
          except LinkTimeout:
            pass
          gate.tick()
        except Exception:
          if session is not None:
            session.close()
          if transport is not None:
            transport.close()
          transport = session = None
          gate.received = 0.
          time.sleep(.5)
      if gate.worker is not None:
        gate.worker.join(max(0., until - clock()))
      return verify_release(gate.selected, update.ROOT)
    finally:
      if session is not None:
        session.close()
      if transport is not None:
        transport.close()


def _fresh(value, now):
  try:
    return 0 <= now - float(value['updated']) < 3
  except (KeyError, TypeError, ValueError):
    return False


def health(now=None):
  """Comma link (fresh HUD snapshot), map video (current, not stalled), model output (recent inference)."""
  now = time.monotonic() if now is None else now
  hud = R._read(HUD_STATUS) or {}
  host = R._read(HOST_HEALTH) or {}
  hud_ok = _fresh(hud, now)
  snapshot = hud.get('snapshot') if isinstance(hud.get('snapshot'), dict) else {}
  map_state = hud.get('map') if isinstance(hud.get('map'), dict) else {}
  diag = host.get('hud_diag') if isinstance(host.get('hud_diag'), dict) else {}
  return {'comma': hud_ok and snapshot.get('ok') is True,
          'map': hud_ok and map_state.get('sequence') is not None and map_state.get('stalled') is False,
          'model': _fresh(host, now) and isinstance(diag.get('inference'), dict)}


def soft_reset(request_id, run=subprocess.run, sysfs=Path('/sys'), proc=Path('/proc'), sleep=time.sleep,
               clock=time.monotonic, release=None, check=None, reboot=None, report=None, log=print):
  """Root unit. Returns [(step, result), ...]; ends 'done' or in the full reboot."""
  started = clock()
  deadline = started + DEADLINE_SECONDS
  report = report or _report(request_id, started, clock)
  release = release or (lambda left: release_from_comma(left, clock))
  check = check or health
  reboot = reboot or (lambda: R.prepare_and_reboot(run, sysfs, sleep, log))
  steps = []
  step = 0

  def full_reboot(why):
    report(step, 'reboot', why)
    steps.append(('reboot', why))
    log(json.dumps({'carrot_soft_reset': steps}))
    reboot()
    return steps

  try:
    steps.append(('fallback', R._ok(run, ('systemd-run', f'--on-active={FALLBACK_SECONDS}', '--collect',
                                          f'--unit={UNIT}-fallback', sys.executable, str(Path(R.__file__).resolve()),
                                          '--prepare-and-reboot'))))
    step = 1
    report(1, 'running')
    sleep(1.)   # the RESPONSE reaches the comma before the server stops
    steps.append(('stop', R._ok(run, ('systemctl', 'stop', *R.STOP_UNITS), timeout=20)))
    step = 2
    report(2, 'running')
    idled, holders = release_gpu(sysfs, proc, sleep, clock)
    steps.append(('gpu', idled, len(holders)))
    if not idled or holders:
      return full_reboot('gpu not released')
    step = 3
    report(3, 'running')
    steps.append(('usb', *reattach_usb(sysfs, sleep)))
    step = 4
    report(4, 'running')
    why = release(max(0., deadline - clock()))
    steps.append(('release', why))
    if why:
      return full_reboot(why)
    step = 5
    report(5, 'running')
    steps.append(('start', R._ok(run, ('systemctl', 'start', *START_UNITS), timeout=20)))
    state = {}
    while clock() < deadline:
      state = check()
      if all(state.values()):
        R._ok(run, ('systemctl', 'stop', f'{UNIT}-fallback.timer'))
        steps.append(('done', state))
        report(5, 'done', '', state)
        log(json.dumps({'carrot_soft_reset': steps}))
        return steps
      report(5, 'running', '', state)
      sleep(.5)
    return full_reboot('not healthy in %d s: %s' % (DEADLINE_SECONDS, ','.join(k for k, v in state.items() if not v)))
  except Exception as exc:   # noqa: BLE001 -- any failure ends in the full reboot
    return full_reboot('error ' + repr(exc)[:60])


# ---------------------------------------------------------------- comma side

def submit(now=None):
  """Web: queue one soft reset request, return its id."""
  request_id = uuid.uuid4().hex
  now = time.monotonic() if now is None else now
  R._write(SOFT_RESULT_FILE, {'id': request_id, 'state': 'pending', 'step': 0})
  R._write(SOFT_REQUEST_FILE, {'id': request_id, 'time': now})
  return request_id


def status(request_id, requested_at, now=None):
  """Web: {'state', 'step', 'reason'}. state: pending / running / done / reboot / rejected / unsupported /
  disconnected. Past DEADLINE_SECONDS (+ margin) without 'done' the Jetson has gone to the full reboot."""
  now = time.monotonic() if now is None else now
  value = R._read(SOFT_RESULT_FILE) or {}
  if value.get('id') != request_id:
    value = {'state': 'pending', 'step': 0}
  state, step, reason = value.get('state', 'pending'), value.get('step', 0), value.get('reason', '')
  if state == 'pending' and now - requested_at > R.REQUEST_TIMEOUT + .5:
    state = 'disconnected'
  elif state == 'running' and now - requested_at > DEADLINE_SECONDS + 5:
    state, reason = 'reboot', 'not healthy in %d s' % DEADLINE_SECONDS
  return {'state': state, 'step': step, 'reason': reason}


def _merge(remote):
  """Jetson progress (HELLO or soft_status) into the web result, same request id only."""
  current = R._read(SOFT_RESULT_FILE)
  if not isinstance(remote, dict) or current is None or remote.get('id') != current.get('id'):
    return
  if current.get('state') not in ('pending', 'running'):
    return
  state = remote.get('state')
  if state in ('running', 'done', 'reboot') and isinstance(remote.get('step'), int):
    R._write(SOFT_RESULT_FILE, {'id': current['id'], 'state': state, 'step': remote['step'],
                                'reason': str(remote.get('detail', ''))[:80]})


class Client:
  """jetlinkd, inside host_reboot.Client: forward the web request, then poll the Jetson progress."""
  def __init__(self, peer):
    self.enabled = isinstance(peer, dict) and peer.get(CAPABILITY) is True
    self.is_jetson = isinstance(peer, dict) and peer.get('carrot_host') == 'jetson'
    self.last_poll = 0.
    if isinstance(peer, dict):
      _merge(peer.get('carrot_soft_reset'))
      if peer.get('carrot_boot_update_v1') is True and not self.enabled:
        # Only step 4 runs the boot gate session (its HELLO has no progress; the updater is not changed).
        current = R._read(SOFT_RESULT_FILE)
        if current is not None and current.get('state') == 'running' and current.get('step', 0) < 4:
          R._write(SOFT_RESULT_FILE, dict(current, step=4))

  def idle(self, client, now=None):
    now = time.monotonic() if now is None else now
    request = R._read(SOFT_REQUEST_FILE)
    if request is not None:
      SOFT_REQUEST_FILE.unlink(missing_ok=True)
      try:
        fresh = 0 <= now - float(request['time']) < R.REQUEST_TIMEOUT and isinstance(request['id'], str)
      except (KeyError, TypeError, ValueError):
        fresh = False
      if fresh:
        R._write(SOFT_RESULT_FILE, dict(self._start(client, request['id']), id=request['id']))
      return
    current = R._read(SOFT_RESULT_FILE)
    if (self.enabled and current is not None and current.get('state') == 'running'
        and now - self.last_poll >= STATUS_POLL_SECONDS):
      self.last_poll = now
      reply = self._request(client, {'soft_status': True, 'id': current['id']})
      if reply is not None and reply.get('ok') is True:
        _merge(reply.get('progress'))

  def _request(self, client, body):
    try:
      seq = client._next_seq()
      client.t.send_json(R.REQUEST, seq, body)
      return json.loads(bytes(client._expect(R.RESPONSE, seq, 3.).payload))
    except Exception:
      return None

  def _start(self, client, request_id):
    if not self.is_jetson:
      return {'state': 'disconnected', 'step': 0, 'reason': ''}
    if not self.enabled:
      return {'state': 'unsupported', 'step': 0, 'reason': 'old jetson bundle'}
    reply = self._request(client, {'soft_reset': True, 'id': request_id})
    if reply is None:
      return {'state': 'disconnected', 'step': 0, 'reason': ''}
    if reply.get('ok') is True:
      return {'state': 'running', 'step': 1, 'reason': ''}
    return {'state': 'rejected', 'step': 0, 'reason': str(reply.get('reason', ''))[:60]}


if __name__ == '__main__' and len(sys.argv) == 3 and sys.argv[1] == '--soft-reset' and _ID.match(sys.argv[2]):
  soft_reset(sys.argv[2])
