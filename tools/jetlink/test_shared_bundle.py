"""One shared runtime bundle for every catalog model; old releases/models pruned.

Simulates model switches A -> B -> A on one Jetson runtime folder (no GPU, no
network): each switch must stage/probe the selected model, keep exactly one
release folder per commit and one cache file per model, and leave only the
active and the previous release/model on disk.
"""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import update_host as update
import boot_update

PIN = 'openpilot/selfdrive/modeld/jetlink/jetson_model.json'
CATALOG = 'openpilot/selfdrive/modeld/model_catalog'
A = {'sha256': 'a1' * 32, 'nbytes': 11, 'frame_skip': 4}
B = {'sha256': 'b2' * 32, 'nbytes': 22, 'frame_skip': 4}
OLD = {'sha256': 'c3' * 32, 'nbytes': 33, 'frame_skip': 4}


def make_release(root, commit, pin, catalog=()):
  release = root / 'releases' / commit
  (release / PIN).parent.mkdir(parents=True)
  (release / PIN).write_text(json.dumps(pin))
  (release / CATALOG).mkdir(parents=True)
  for name, spec in catalog:
    (release / CATALOG / name).write_text(json.dumps(spec))
  (release / CATALOG / 'catalog.json').write_text('{}')
  (release / 'SOURCE_COMMIT').write_text(commit + '\n')
  (release / '.bundle-sha256').write_text('e' * 64)
  return release


def manifest(commit, spec):
  return {'format': 1, 'source_commit': commit,
          'runtime': {'arch': 'aarch64', 'l4t': '36.4.7', 'tensorrt': '10.3.0'},
          'model': {'url': 'https://github.com/2020K9/carrotpilot/releases/download/k9-m/m.onnx',
                    'sha256': spec['sha256'], 'size': spec['nbytes']},
          'bundle': {'url': f'https://github.com/2020K9/carrotpilot/releases/download/k9-jetlink-host-{commit}/precompiled-runtime.tar.gz',
                     'sha256': 'e' * 64, 'size': 5}}


@pytest.fixture
def host(tmp_path, monkeypatch):
  monkeypatch.setattr(update, 'ROOT', tmp_path)
  monkeypatch.setattr(update, 'PROTECTED_MARKER', tmp_path / 'no-marker')
  monkeypatch.setattr(update, 'verify_signature', lambda _: None)
  monkeypatch.setattr(update, 'check_runtime', lambda _: None)
  monkeypatch.setattr(update.subprocess, 'run', lambda *a, **k: SimpleNamespace(returncode=3))
  monkeypatch.setattr(update.os, 'chown', lambda *a: None)
  import pwd
  monkeypatch.setattr(pwd, 'getpwnam', lambda _: SimpleNamespace(pw_uid=0, pw_gid=0))
  fetched = []

  def fetch(url, path, sha256, size, progress=None):
    fetched.append(path.name)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(b'x' * size)
  monkeypatch.setattr(update, 'fetch', fetch)
  probes = []
  monkeypatch.setattr(update, 'probe_release', lambda release, model=None: probes.append((release.name, model)))
  (tmp_path / 'cache/models').mkdir(parents=True)
  (tmp_path / 'updates').mkdir()
  return SimpleNamespace(root=tmp_path, fetched=fetched, probes=probes)


def switch(host, value):
  update.stage_manifest(value)
  update.activate()
  return value


def folders(root):
  return sorted(p.name for p in (root / 'releases').iterdir() if not p.name.startswith('update-'))


def models(root):
  return sorted(p.name for p in (root / 'cache/models').glob('*.onnx'))


def test_release_spec_selects_catalog_model_by_exact_identity(tmp_path):
  release = make_release(tmp_path, '1' * 40, A, [('b.json', B), ('b-release.json', B)])
  assert update.release_spec(release, {'sha256': A['sha256'], 'size': 11}) == A
  assert update.release_spec(release, {'sha256': B['sha256'], 'size': 22}) == B
  assert update.release_spec(release, {'sha256': B['sha256'], 'size': 23}) is None
  update.check_bundle(release, manifest('1' * 40, B))
  with pytest.raises(ValueError, match='model contract'):
    update.check_bundle(release, manifest('1' * 40, OLD))


def test_model_switch_a_b_a_one_folder_per_commit_and_prunes_old(host, monkeypatch):
  root = host.root
  shared = '5' * 40
  # Host as left by per-model bundles: two old folders and an old model.
  older, old = make_release(root, '8' * 40, OLD), make_release(root, '9' * 40, A)
  (root / 'current').symlink_to(old)
  for spec in (OLD, A):
    (root / 'cache/models' / (spec['sha256'][:16] + '.onnx')).write_bytes(b'x' * spec['nbytes'])
  (root / 'updates' / ('8' * 40 + '.tar.gz')).write_bytes(b'old')
  # The shared bundle carries the pin (A) and every catalog spec.
  import finalize_sd_image

  def extract(bundle, target):
    made = make_release(root / 'scratch-a', shared, A, [('a.json', A), ('b.json', B)])
    for path in made.iterdir():
      path.rename(target / path.name)
  monkeypatch.setattr(finalize_sd_image, 'extract_bundle', extract)

  switch(host, manifest(shared, A))  # same model, new shared runtime
  assert (root / 'current').resolve().name == shared
  assert folders(root) == sorted({'9' * 40, shared})  # older folder pruned, previous kept
  assert models(root) == [A['sha256'][:16] + '.onnx']  # OLD model pruned (not active/previous)
  assert not (root / 'updates' / ('8' * 40 + '.tar.gz')).exists()

  gate_value = manifest(shared, B)
  assert not update.is_active(gate_value)  # same commit, other model: gate must not say ready
  switch(host, gate_value)
  assert (root / 'current').resolve().name == shared
  assert host.probes[-1] == (shared, {'sha256': B['sha256'], 'size': 22})
  assert update.is_active(gate_value)
  assert folders(root) == sorted({'9' * 40, shared})
  assert models(root) == sorted(m['sha256'][:16] + '.onnx' for m in (A, B))

  switch(host, manifest(shared, A))  # back to A: nothing new downloaded or duplicated
  assert update.is_active(manifest(shared, A))
  assert host.probes[-1] == (shared, None)  # pin model uses the original probe call
  assert folders(root).count(shared) == 1
  assert sorted(models(root)) == sorted(m['sha256'][:16] + '.onnx' for m in (A, B))
  assert len(list((root / 'releases').iterdir())) <= 2

  # A later runtime commit drops the folder before the previous one.
  newer = '6' * 40
  monkeypatch.setattr(finalize_sd_image, 'extract_bundle',
                      lambda bundle, target: [p.rename(target / p.name) for p in
                                              make_release(root / 'scratch-b', newer, A, [('b.json', B)]).iterdir()])
  switch(host, manifest(newer, B))
  assert folders(root) == sorted({shared, newer})
  assert update.is_active(manifest(newer, B))


def test_same_commit_new_model_is_staged_even_when_release_exists(host):
  root = host.root
  shared = '7' * 40
  release = make_release(root, shared, A, [('b.json', B)])
  (root / 'current').symlink_to(release)
  update.atomic_json(root / 'updates/active.json', {'source_commit': shared, 'model': {'sha256': A['sha256'], 'size': 11}})
  (root / 'cache/models' / (A['sha256'][:16] + '.onnx')).write_bytes(b'x' * 11)
  update.stage_manifest(manifest(shared, A))
  assert host.fetched == []  # active and cached: nothing to do
  update.stage_manifest(manifest(shared, B))
  assert B['sha256'][:16] + '.onnx' in host.fetched
  assert json.loads((root / 'updates/pending.json').read_text())['model']['sha256'] == B['sha256']


def test_prune_keeps_transaction_pending_and_protected_preload(host, monkeypatch):
  root = host.root
  cur, prev, staged, other = (make_release(root, c * 40, A) for c in '1234')
  (root / 'current').symlink_to(cur)
  update.atomic_json(root / 'updates/previous.json', {'release': str(prev), 'last_loaded': json.dumps({'sha256': B['sha256'], 'frame_skip': 4})})
  update.atomic_json(root / 'updates/pending.json', manifest('3' * 40, OLD))
  marker = root / 'protected.json'
  preload = {'sha256': 'd4' * 32, 'bytes': 4}
  marker.write_text(json.dumps({'format': 1, 'preloaded_model': preload}))
  monkeypatch.setattr(update, 'PROTECTED_MARKER', marker)
  for sha in (A['sha256'], B['sha256'], OLD['sha256'], preload['sha256'], 'e5' * 32):
    (root / 'cache/models' / (sha[:16] + '.onnx')).write_bytes(b'x')
  (root / 'releases' / 'update-tmp').mkdir()
  update.prune()
  assert folders(root) == sorted(['1' * 40, '2' * 40, '3' * 40])
  assert (root / 'releases' / 'update-tmp').is_dir()
  assert models(root) == sorted(s[:16] + '.onnx' for s in (A['sha256'], B['sha256'], OLD['sha256'], preload['sha256']))
  assert other.name not in folders(root)


def test_gate_waits_for_model_on_same_commit(host, monkeypatch):
  root = host.root
  shared = '7' * 40
  release = make_release(root, shared, A, [('b.json', B)])
  (root / 'current').symlink_to(release)
  update.atomic_json(root / 'updates/active.json', {'source_commit': shared, 'model': {'sha256': A['sha256'], 'size': 11}})
  started = []
  monkeypatch.setattr(boot_update.threading, 'Thread', lambda target, args, daemon: SimpleNamespace(
    start=lambda: started.append(args[0]['model']['sha256']), is_alive=lambda: True))
  monkeypatch.setattr(boot_update, 'READY', root / 'ready.json')
  monkeypatch.setattr(boot_update, 'BOOT_ID', root / 'boot_id')
  (root / 'boot_id').write_text('boot')
  gate = boot_update.Gate()
  gate.select(manifest(shared, B))
  assert gate.source_commit != shared  # same runtime, other model: not active yet
  gate.tick()
  assert started == [B['sha256']] and not gate.done
  gate = boot_update.Gate()
  gate.select(manifest(shared, A))
  assert gate.source_commit == shared
  gate.tick()
  assert gate.done


def test_refresh_only_for_updater_without_model_selection(tmp_path):
  import install_updates
  (tmp_path / 'updater').mkdir()
  (tmp_path / 'current/tools/jetlink').mkdir(parents=True)
  (tmp_path / 'current/tools/jetlink/update_host.py').write_bytes(Path(update.__file__).read_bytes())
  (tmp_path / 'updater/update_host.py').write_text('"""old"""\n')
  assert install_updates.updater_needs_refresh(tmp_path)
  (tmp_path / 'updater/update_host.py').write_bytes(Path(update.__file__).read_bytes())
  assert not install_updates.updater_needs_refresh(tmp_path)
  assert set(install_updates.UPDATER_NAMES) >= {'update_host.py', 'boot_update.py', 'release-signing-public.pem'}
