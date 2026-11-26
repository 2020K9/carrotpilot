"""'Jetson soft reset' button (대감님 11/26 23:21): step order, GPU not released -> reboot at once,
not healthy in 30 s -> the existing full reboot, web step text."""
import json
from pathlib import Path
from types import SimpleNamespace

import pytest

import host_reboot as R
import host_soft_reset as S


@pytest.fixture(autouse=True)
def files(monkeypatch, tmp_path):
  monkeypatch.setitem(R._state, 'rebooting', False)
  for name in ('PROGRESS', 'SOFT_REQUEST_FILE', 'SOFT_RESULT_FILE', 'HUD_STATUS', 'HOST_HEALTH'):
    monkeypatch.setattr(S, name, tmp_path / (name.lower() + '.json'))


def _runner(fail=()):
  calls = []

  def run(cmd, **kw):
    calls.append(list(cmd))
    return SimpleNamespace(returncode=1 if any(w in cmd for w in fail) else 0)
  return run, calls


def _sysfs(tmp_path):
  gpu = tmp_path / 'sys/devices/platform/bus@0/17000000.gpu'
  gpu.mkdir(parents=True)
  (gpu / 'force_idle').write_text('0')
  (gpu / 'railgate_enable').write_text('1')
  usb = tmp_path / 'sys/bus/usb/devices'
  for name in ('usb1', '1-1', '1-1.2', '1-1:1.0'):
    (usb / name).mkdir(parents=True)
    (usb / name / 'authorized').write_text('1')
  (usb / '1-0:1.0/usb1-port1').mkdir(parents=True)
  (usb / '1-0:1.0/usb1-port1/disable').write_text('0')
  (tmp_path / 'proc').mkdir()
  return tmp_path / 'sys', tmp_path / 'proc', gpu, usb


class Clock:
  def __init__(self):
    self.t = 100.

  def __call__(self):
    return self.t

  def sleep(self, s):
    self.t += s


def _run_reset(tmp_path, release='', check=None, gpu=True):
  sysfs, proc, gpu_dir, usb = _sysfs(tmp_path)
  if not gpu:
    (gpu_dir / 'force_idle').unlink()
  run, calls = _runner()
  clock = Clock()
  reports, reboots = [], []
  steps = S.soft_reset('ab', run, sysfs, proc, clock.sleep, clock, release=lambda left: release,
                       check=check or (lambda: {'comma': True, 'map': True, 'model': True}),
                       reboot=lambda: reboots.append(clock()), report=lambda *a, **k: reports.append(a[:2]), log=lambda m: None)
  return steps, calls, reports, reboots, clock, (gpu_dir, usb)


def test_order_stop_gpu_usb_release_start_done(tmp_path):
  steps, calls, reports, reboots, clock, (gpu, usb) = _run_reset(tmp_path)
  assert [s[0] for s in steps] == ['fallback', 'stop', 'gpu', 'usb', 'release', 'start', 'done']
  assert [r[0] for r in reports] == [1, 2, 3, 4, 5, 5] and reports[-1] == (5, 'done')
  assert calls[0][:2] == ['systemd-run', f'--on-active={S.FALLBACK_SECONDS}'] and calls[0][-1] == '--prepare-and-reboot'
  assert calls[1] == ['systemctl', 'stop', *R.STOP_UNITS]
  assert calls[2] == ['systemctl', 'start', *S.START_UNITS]
  assert calls[3] == ['systemctl', 'stop', f'{S.UNIT}-fallback.timer']
  assert (gpu / 'force_idle').read_text() == '1'
  # USB detached (reboot button's reset_usb) then attached again
  assert (usb / '1-1/authorized').read_text() == '1' and (usb / '1-1.2/authorized').read_text() == '1'
  assert (usb / '1-0:1.0/usb1-port1/disable').read_text() == '0'
  assert reboots == []


def test_gpu_not_released_reboots_at_once(tmp_path):
  steps, calls, reports, reboots, clock, _ = _run_reset(tmp_path, gpu=False)
  assert [s[0] for s in steps] == ['fallback', 'stop', 'gpu', 'reboot'] and reports[-1] == (2, 'reboot')
  assert len(reboots) == 1 and ['systemctl', 'start', *S.START_UNITS] not in calls


def test_gpu_holder_left_reboots(tmp_path, monkeypatch):
  monkeypatch.setattr(S, 'gpu_holders', lambda proc: [1234])
  steps, calls, reports, reboots, clock, _ = _run_reset(tmp_path)
  assert steps[-1] == ('reboot', 'gpu not released') and len(reboots) == 1


def test_hash_mismatch_reboots(tmp_path):
  steps, calls, reports, reboots, clock, _ = _run_reset(tmp_path, release='model sha256 differs')
  assert steps[-1] == ('reboot', 'model sha256 differs') and reports[-1] == (4, 'reboot') and len(reboots) == 1


def test_not_healthy_in_30s_goes_to_full_reboot(tmp_path):
  steps, calls, reports, reboots, clock, _ = _run_reset(tmp_path, check=lambda: {'comma': True, 'map': False, 'model': True})
  assert steps[-1][0] == 'reboot' and 'map' in steps[-1][1] and reports[-1] == (5, 'reboot')
  assert len(reboots) == 1 and reboots[0] >= 100 + S.DEADLINE_SECONDS
  assert ['systemctl', 'stop', f'{S.UNIT}-fallback.timer'] not in calls


def test_default_reboot_is_the_reboot_button_path(tmp_path, monkeypatch):
  seen = []
  monkeypatch.setattr(R, 'prepare_and_reboot', lambda *a, **k: seen.append('full'))
  sysfs, proc, gpu, usb = _sysfs(tmp_path)
  (gpu / 'force_idle').unlink()
  run, calls = _runner()
  clock = Clock()
  S.soft_reset('ab', run, sysfs, proc, clock.sleep, clock, report=lambda *a, **k: None, log=lambda m: None)
  assert seen == ['full']


def test_health_needs_comma_map_and_model():
  S.HUD_STATUS.write_text(json.dumps({'updated': 10., 'snapshot': {'ok': True}, 'map': {'sequence': 3, 'stalled': False}}))
  S.HOST_HEALTH.write_text(json.dumps({'updated': 10., 'hud_diag': {'inference': {'gpu_ms': 5}}}))
  assert S.health(11.) == {'comma': True, 'map': True, 'model': True}
  assert S.health(20.) == {'comma': False, 'map': False, 'model': False}
  S.HOST_HEALTH.write_text(json.dumps({'updated': 10., 'hud_diag': {}}))
  assert S.health(11.)['model'] is False


def test_jetson_request_starts_unit(monkeypatch, tmp_path):
  marker = tmp_path / 'nv'
  marker.write_text('R36')
  monkeypatch.setattr(R, 'JETSON_MARKER', marker)
  run, calls = _runner()
  assert R.host_request(json.dumps({'soft_reset': True, 'id': 'ab'}).encode(), run) == {'id': 'ab', 'ok': True}
  assert calls == [list(S.command('ab'))] and calls[0][-2:] == ['--soft-reset', 'ab']
  S.PROGRESS.write_text(json.dumps({'id': 'ab', 'step': 3, 'state': 'running'}))
  assert R.host_request(b'{"soft_status": true, "id": "ab"}', run)['progress']['step'] == 3
  assert R.host_request(b'{"soft_reset": true, "id": "a;rm"}', run)['ok'] is False


class Link:
  def __init__(self, replies):
    self.replies, self.sent, self.seq = list(replies), [], 0
    self.t = self

  def _next_seq(self):
    self.seq += 1
    return self.seq

  def send_json(self, kind, seq, body):
    self.sent.append(body)

  def _expect(self, kind, seq, timeout):
    return SimpleNamespace(payload=json.dumps(self.replies.pop(0)).encode())


def test_comma_steps_shown_on_web():
  peer = {'carrot_host': 'jetson', S.CAPABILITY: True, R.CAPABILITY: True}
  rid = S.submit(now=0.)
  link = Link([{'ok': True}, {'ok': True, 'progress': {'id': rid, 'step': 2, 'state': 'running'}}])
  client = R.Client(peer)
  client.idle(link, None, now=.1)
  assert link.sent == [{'soft_reset': True, 'id': rid}] and S.status(rid, 0., now=.2)['step'] == 1
  client.idle(link, None, now=2.)
  assert S.status(rid, 0., now=2.) == {'state': 'running', 'step': 2, 'reason': ''}
  # reconnect during step 4: the boot gate session HELLO
  S.Client({'carrot_host': 'jetson', 'carrot_boot_update_v1': True})
  assert S.status(rid, 0., now=10.)['step'] == 4
  S.Client({'carrot_host': 'jetson', S.CAPABILITY: True, 'carrot_soft_reset': {'id': rid, 'step': 5, 'state': 'done'}})
  assert S.status(rid, 0., now=12.)['state'] == 'done'


def test_comma_no_done_in_time_shows_reboot():
  rid = S.submit(now=0.)
  S.Client({'carrot_host': 'jetson', 'carrot_soft_reset': {'id': rid, 'step': 5, 'state': 'running'}})
  assert S.status(rid, 0., now=20.)['state'] == 'running'
  assert S.status(rid, 0., now=S.DEADLINE_SECONDS + 6)['state'] == 'reboot'


def test_old_bundle_unsupported():
  rid = S.submit(now=0.)
  R.Client({'carrot_host': 'jetson', R.CAPABILITY: True}).idle(Link([]), None, now=.1)
  assert S.status(rid, 0., now=.2)['state'] == 'unsupported'


def test_web_button_below_reboot():
  web = Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/carrot/web/js'
  page = (web / 'pages/setting_device.js').read_text()
  assert page.index('"btnDeviceJetsonReboot"') < page.index('"btnDeviceJetsonSoftReset"') < page.index('"btnDeviceRecalib"')
  ko = (web / 'translations/ko.js').read_text()
  assert 'jetson_soft_reset: "젯슨 초기화"' in ko
  for n in range(1, 6):
    assert f'jetson_soft_step{n}:' in ko
