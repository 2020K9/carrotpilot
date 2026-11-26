"""Jetson reboot button: stop inference -> GPU -> USB -> reboot (대감님 11/26 18:50); reboot never skipped."""
import json
from types import SimpleNamespace

import pytest

import host_reboot as R


@pytest.fixture(autouse=True)
def fresh(monkeypatch):
  monkeypatch.setitem(R._state, 'rebooting', False)


def _runner(fail=()):
  calls = []

  def run(cmd, **kw):
    calls.append(list(cmd))
    return SimpleNamespace(returncode=1 if any(w in cmd for w in fail) else 0)
  return run, calls


def _sysfs(tmp_path):
  gpu = tmp_path / 'devices/platform/bus@0/17000000.gpu'
  gpu.mkdir(parents=True)
  (gpu / 'force_idle').write_text('0')
  (gpu / 'railgate_enable').write_text('1')
  usb = tmp_path / 'bus/usb/devices'
  for name in ('usb1', '1-1', '1-1.2', '1-1:1.0'):
    (usb / name).mkdir(parents=True)
    (usb / name / 'authorized').write_text('1')
  (usb / '1-0:1.0/usb1-port1').mkdir(parents=True)
  (usb / '1-0:1.0/usb1-port1/disable').write_text('0')
  return tmp_path, gpu, usb


def test_jetson_starts_prepare_unit_once(monkeypatch, tmp_path):
  marker = tmp_path / 'nv_tegra_release'
  marker.write_text('R36')
  monkeypatch.setattr(R, 'JETSON_MARKER', marker)
  run, calls = _runner()
  req = json.dumps({'reboot': True, 'id': 'a'}).encode()
  assert R.host_request(req, run) == {'id': 'a', 'ok': True}
  assert calls == [list(R.PREPARE_COMMAND)]
  assert R.host_request(req, run)['duplicate'] is True and len(calls) == 1


def test_prepare_unit_failure_falls_back_to_plain_reboot(monkeypatch, tmp_path):
  marker = tmp_path / 'nv_tegra_release'
  marker.write_text('R36')
  monkeypatch.setattr(R, 'JETSON_MARKER', marker)
  run, calls = _runner(fail=('systemd-run',))
  assert R.host_request(b'{"reboot": true, "id": "a"}', run) == {'id': 'a', 'ok': True}
  assert calls == [list(R.PREPARE_COMMAND), list(R.REBOOT_COMMAND)]


def test_stop_reboot_preserves_gpu_usb(tmp_path):
  root, gpu, usb = _sysfs(tmp_path)
  run, calls = _runner()
  seen = []
  steps = R.prepare_and_reboot(run, root, sleep=lambda s: seen.append(('sleep', len(calls))), log=lambda m: None)
  assert [s[0] for s in steps] == ['fallback', 'stop', 'reboot']
  assert calls[0][:2] == ['systemd-run', '--on-active=5']
  assert calls[1] == ['systemctl', 'stop', *R.STOP_UNITS] and calls[2] == ['systemctl', '--no-block', 'reboot']
  assert seen == [('sleep', 1)]
  assert (gpu / 'force_idle').read_text() == '0'
  assert (usb / '1-1/authorized').read_text() == '1' and (usb / '1-1.2/authorized').read_text() == '1'
  assert (usb / 'usb1/authorized').read_text() == '1' and (usb / '1-1:1.0/authorized').read_text() == '1'
  assert (usb / '1-0:1.0/usb1-port1/disable').read_text() == '0'
  assert dict(steps)['reboot'] is True


def test_reboot_even_when_every_step_fails(tmp_path, monkeypatch):
  run, calls = _runner(fail=('stop', 'systemd-run'))
  steps = R.prepare_and_reboot(run, tmp_path / 'missing', sleep=lambda s: None, log=lambda m: None)
  assert calls[-1] == ['systemctl', '--no-block', 'reboot'] and dict(steps)['reboot'] is True
  assert dict(steps)['stop'] is False


def test_forced_reboot_when_orderly_reboot_fails(tmp_path):
  calls = []

  def run(cmd, **kw):
    calls.append(list(cmd))
    return SimpleNamespace(returncode=1 if cmd == ['systemctl', '--no-block', 'reboot'] else 0)
  steps = R.prepare_and_reboot(run, tmp_path, sleep=lambda s: None, log=lambda m: None)
  assert calls[-1] == ['systemctl', '--force', 'reboot'] and dict(steps)['reboot'] is True


def test_reboot_when_systemctl_raises(tmp_path):
  def run(cmd, **kw):
    raise OSError('no systemctl')
  steps = R.prepare_and_reboot(run, tmp_path, sleep=lambda s: None, log=lambda m: None)
  assert dict(steps)['reboot'] is False   # nothing else possible; the 5 s timer is already gone too
