"""Comma commit changes while the Jetson already has the model (대감님 11/27 05:43).

Same model hash: no download, no install, no error. Different model hash: the install path runs.
"""
import hashlib
import json

import pytest

import boot_update as boot
import host_soft_reset as soft
import update_host as update
from test_boot_update import env  # noqa: F401 -- shared filesystem fixture
from test_update_host import manifest

MODEL = b'model-bytes'


def installed_model(root, data=MODEL):
  """Current release d*40 activated with a model whose file lives under its hash."""
  sha = hashlib.sha256(data).hexdigest()
  identity = {'sha256': sha, 'size': len(data)}
  pin = root / 'current' / update.PIN
  pin.parent.mkdir(parents=True)
  pin.write_text(json.dumps({'sha256': sha, 'nbytes': len(data)}))
  (root / 'cache/models').mkdir(parents=True)
  update.model_path(identity).write_bytes(data)
  (root / 'updates').mkdir()
  (root / 'updates/active.json').write_text(json.dumps({'source_commit': 'd' * 40, 'model': identity}))
  return identity


def pin(identity):
  value = manifest()  # source_commit a*40, bundle c*64: a new comma commit and bundle
  value['model'] = dict(value['model'], **identity)
  return value


def test_commit_change_same_model_hash_keeps_current_without_download(env, monkeypatch):
  value = pin(installed_model(env))
  monkeypatch.setattr(update, 'stage_manifest', lambda *a, **k: pytest.fail('downloaded again'))
  monkeypatch.setattr(update, 'fetch', lambda *a, **k: pytest.fail('downloaded again'))
  monkeypatch.setattr(update, 'activate', lambda: pytest.fail('activated again'))
  monkeypatch.setattr(boot.subprocess, 'run', lambda *a, **k: pytest.fail('services touched'))
  gate = boot.Gate()
  gate.select(value)
  gate.tick()
  assert gate.worker is None and gate.done and gate.state == 'ready' and gate.error == ''
  assert boot.runtime_ready()
  assert json.loads(boot.READY.read_text())['source_commit'] == 'd' * 40
  assert (env / 'current/SOURCE_COMMIT').read_text() == 'd' * 40
  # Jetson soft reset step 4 accepts the kept release instead of a full reboot.
  assert soft.verify_release(value, env) == ''


def test_model_hash_change_downloads_new_model(env, monkeypatch):
  installed_model(env)
  value = pin({'sha256': hashlib.sha256(b'other-model').hexdigest(), 'size': len(b'other-model')})
  calls = []
  def stage(selected, progress):
    calls.append(('download', selected['model']['sha256']))
  def activate():
    calls.append('activate')
    (env / 'current/SOURCE_COMMIT').write_text('a' * 40)
  monkeypatch.setattr(update, 'stage_manifest', stage)
  monkeypatch.setattr(update, 'activate', activate)
  monkeypatch.setattr(boot.subprocess, 'run', lambda command, **k: calls.append(command[:2]))
  assert not update.keeps_current(value)
  gate = boot.Gate()
  gate.select(value)
  gate.tick()
  gate.worker.join(1)
  assert calls[0] == ('download', value['model']['sha256']) and 'activate' in calls
  assert not boot.runtime_ready()
  gate.tick()
  assert gate.done and boot.runtime_ready()
