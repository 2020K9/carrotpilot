"""대감님 11/18 15:49: the Jetson pre-downloads the comma's pin while onroad; only the next boot applies it."""
import json
import time
from pathlib import Path

import pytest

import update_host as update


def test_onroad_snapshot_stages_selected_release_without_activation(monkeypatch):
  import hud_protocol
  selected = {'source_commit': 'a' * 40}
  monkeypatch.setattr(hud_protocol, 'read_snapshot', lambda: (1., {'params': {'IsOnroad': 'MQ=='}, 'jetson_release': selected}))
  calls = []
  monkeypatch.setattr(update, 'stage_manifest', lambda value: calls.append(value))
  monkeypatch.setattr(update, 'activate', lambda: pytest.fail('staging must never activate'))
  monkeypatch.setattr(update, 'stage', lambda *a: pytest.fail('no unpinned channel'))
  update.prestage()
  assert calls == [selected]


def test_onroad_without_pin_does_not_use_unpinned_channel(monkeypatch):
  import hud_protocol
  monkeypatch.setattr(hud_protocol, 'read_snapshot', lambda: (1., {'params': {'IsOnroad': 'MQ=='}}))
  monkeypatch.setattr(update, 'stage', lambda *a: pytest.fail('legacy channel only offroad'))
  monkeypatch.setattr(update, 'stage_manifest', lambda *a: pytest.fail('no pin, nothing to stage'))
  update.prestage()


def test_onroad_bootstrap_control_stages(tmp_path, monkeypatch):
  control = {'received': time.monotonic(), 'onroad': True, 'jetson_release': {'source_commit': 'b' * 40}}
  real = Path
  path = tmp_path / 'c.json'
  path.write_text(json.dumps(control))
  monkeypatch.setattr(update, 'Path', lambda p: path if p == '/dev/shm/carrot-jetlink-bootstrap.json' else real(p))
  calls = []
  monkeypatch.setattr(update, 'stage_manifest', lambda value: calls.append(value))
  import hud_protocol
  monkeypatch.setattr(hud_protocol, 'read_snapshot', lambda: pytest.fail('fresh bootstrap must win'))
  update.prestage()
  assert calls == [control['jetson_release']]


def test_legacy_automatic_policy_unchanged_onroad(tmp_path, monkeypatch):
  control = {'received': time.monotonic(), 'onroad': True, 'jetson_release': {'source_commit': 'b' * 40}}
  real = Path
  path = tmp_path / 'c.json'
  path.write_text(json.dumps(control))
  monkeypatch.setattr(update, 'Path', lambda p: path if p == '/dev/shm/carrot-jetlink-bootstrap.json' else real(p))
  monkeypatch.setattr(update, 'stage_manifest', lambda value: pytest.fail('legacy staging stays offroad-only'))
  update.automatic_stage()


def _main(monkeypatch, ready):
  import sys
  import boot_update
  monkeypatch.setattr(sys, 'argv', ['update_host.py', 'automatic'])
  monkeypatch.setattr(update.os, 'geteuid', lambda: 0)
  monkeypatch.setattr(boot_update, 'enabled', lambda: True)
  monkeypatch.setattr(boot_update, 'runtime_ready', lambda: ready)
  monkeypatch.setattr(update, 'PROTECTED_MARKER', Path('/nonexistent-k9'))
  calls = []
  monkeypatch.setattr(update, 'prestage', lambda: calls.append('stage'))
  monkeypatch.setattr(update, 'automatic_stage', lambda: pytest.fail('gate hosts use prestage'))
  monkeypatch.setattr(update, 'activate', lambda: pytest.fail('timer never activates'))
  real = Path
  monkeypatch.setattr(update, 'Path', lambda p: real('/tmp/k9-test-update.lock') if p == '/run/carrot-jetlink-update.lock' else real(p))
  update.main()
  return calls


def test_boot_gate_host_prestages_only_after_this_boot_was_approved(monkeypatch):
  assert _main(monkeypatch, ready=False) == []    # the boot gate itself owns the pre-model phase
  assert _main(monkeypatch, ready=True) == ['stage']


def test_jetlink_status_peer_has_no_stale_hello_telemetry(tmp_path, monkeypatch):
  daemon = pytest.importorskip('openpilot.selfdrive.modeld.jetlink.daemon')
  monkeypatch.setattr(daemon, 'STATUS', tmp_path / 'status.json')
  monkeypatch.setattr(daemon, 'LAST_STATUS_EVENT', None)
  import sys
  import types
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog',
                      types.SimpleNamespace(cloudlog=types.SimpleNamespace(event=lambda *a, **k: None)))
  peer = {'carrot_host': 'jetson', 'telemetry': {'carrot_hud_connected': False}}
  daemon.publish('ready', peer=peer, telemetry={'carrot_hud_connected': True})
  record = json.loads((tmp_path / 'status.json').read_text())
  assert 'telemetry' not in record['peer'] and record['peer']['carrot_host'] == 'jetson'
  assert record['telemetry']['carrot_hud_connected'] is True
  assert 'telemetry' in peer   # caller's HELLO record is not modified
