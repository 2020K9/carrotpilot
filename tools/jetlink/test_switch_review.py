"""Review follow-up for switch_updater.py (fake root only, never /opt).

* legacy (0cb67d46c) boot order after an interrupted switch, over two reboots
* injected I/O errors at every recovery-relevant point: never "old updater +
  marker written by the tool", marker restored to its original state
* next boot with the switched updater: download, sha check, unpack, activate,
  current link (local fake download, test signing key)

crash_point() uses os._exit: a forced process kill, NOT a power cut.  It does
not prove what a real power loss keeps on disk.
"""
import base64
import hashlib
import io
import json
import os
from pathlib import Path
import subprocess
import tarfile

import pytest

from test_switch_updater import (MANIFEST, NEW_COMMIT, PY, assert_whole, migrate_calls, next_boot,  # noqa: F401
                                 old_root, run_switch, runtime)
import switch_updater as sw

HERE = Path(__file__).resolve().parent
ENV = dict(os.environ, PYTHONDONTWRITEBYTECODE='1')


def legacy_boot(root, boot_id, manifest=MANIFEST):
  """One boot of a legacy image, in its real order:
  1. carrot-jetlink-update-apply.service (Before=carrot-jetlink.service):
     <installed updater>/update_host.py activate -> boot gate if enabled()
  2. carrot-jetlink.service: current/tools/jetlink/server.py ->
     migrate_running_release() (--bootstrap-only runs the real old bootstrap()
     into the fake root) -> wait_for_runtime() decision.
  Returns what happened."""
  (root.parent / 'boot_id').write_text(boot_id + '\n')
  rt = runtime(root)
  out = {'boot': boot_id, 'updater_key_at_apply': sw.boot_state(root)['key']}
  # 1. apply service with the installed updater.
  gate = next_boot(root, manifest, boot_id=boot_id)
  out['apply'] = gate
  # 2. legacy server start: migrate_running_release with real bootstrap-only.
  src = rt / 'current/tools/jetlink'
  code = r'''
import sys, json, subprocess
from pathlib import Path as P
src, rt, root, bid = map(P, sys.argv[1:5])
sys.path.insert(0, str(src))
import install_updates as i
calls = []
def run(c, **k):
  calls.append(c[-1])
  if c[-1] == '--bootstrap-only':
    def path(x, *a):
      return bid if str(x) == '/proc/sys/kernel/random/boot_id' else P(x, *a)
    i.Path = path
    i.bootstrap(root, src, defer_this_boot=True)
    i.Path = P
i.subprocess.run = run
i.migrate_running_release(rt)
import boot_update as b, update_host as u
u.ROOT = rt
b.BOOT_ID = bid
print(json.dumps({'calls': calls, 'wait_for_runtime_blocks': b.enabled()}))
'''
  r = subprocess.run([PY, '-c', code, str(src), str(rt), str(root), str(root.parent / 'boot_id')],
                     capture_output=True, text=True, timeout=60, env=ENV)
  assert r.returncode == 0, r.stderr
  out['server'] = json.loads(r.stdout)
  out['after'] = sw.boot_state(root)
  out['marker'] = (rt / 'boot-update-required').read_text() if (rt / 'boot-update-required').exists() else None
  return out


def model_runs(boot):
  # Gate on: does the gate pick the comma's pinned commit (NEW_COMMIT)?  Gate
  # off: does wait_for_runtime() not block?  Only that decision is judged; the
  # real download, TensorRT and model start are not (download/activate: below).
  if boot['apply']['gate']:
    return boot['apply'].get('selected') == NEW_COMMIT
  return not boot['server']['wait_for_runtime_blocks']


# ---- item 2: legacy boot before a rerun of the tool -------------------------

def test_measure_kill_after_exchange_then_legacy_boots(old_root):
  """Residual window (exchange done, marker not yet renamed)."""
  assert run_switch(old_root, env={'JET_SWITCH_CRASH_AT': 'after_exchange'}).returncode == 91
  assert sw.boot_state(old_root) == {'key': 'new', 'complete': True, 'marker': False}
  b1 = legacy_boot(old_root, 'boot-B')
  print('MEASURE after_exchange boot-B', json.dumps(b1))
  b2 = legacy_boot(old_root, 'boot-C')
  print('MEASURE after_exchange boot-C', json.dumps(b2))
  # Measured: boot-B runs normally but the legacy server reinstalls the old
  # updater and arms the gate; boot-C is blocked (old updater rejects GitHub).
  assert b1['server']['calls'][-1] == '--bootstrap-only'
  assert b1['after'] == {'key': 'old', 'complete': True, 'marker': True} and model_runs(b1)
  assert b2['apply']['gate'] is True and 'NAS' in b2['apply']['error'] and not model_runs(b2)
  # Recovery measured: the tool run again (any time before or after boot-C)
  # switches the reverted state, keeping the legacy-written marker.
  again = run_switch(old_root, stamp='S2')
  assert again.returncode == 0, again.stderr
  assert json.loads(again.stdout)['result'] == 'switched'
  b3 = legacy_boot(old_root, 'boot-D')
  assert b3['apply'] == {'gate': True, 'selected': NEW_COMMIT} and b3['after']['key'] == 'new'


@pytest.mark.parametrize('point', ['after_marker_tmp'])
def test_measure_kill_before_marker_rename_then_legacy_boots(old_root, point):
  assert run_switch(old_root, env={'JET_SWITCH_CRASH_AT': point}).returncode == 91
  b1 = legacy_boot(old_root, 'boot-B')
  print('MEASURE', point, json.dumps(b1))
  assert b1['after']['key'] == 'old'  # same residual window as after_exchange


@pytest.mark.parametrize('point', ['after_marker_rename', 'after_marker', 'after_rename_old', 'after_verify'])
def test_kill_after_marker_survives_two_legacy_boots(old_root, point):
  """Fixed order: from the marker rename on, legacy boots keep the switch."""
  assert run_switch(old_root, env={'JET_SWITCH_CRASH_AT': point}).returncode == 91
  assert sw.boot_state(old_root) == {'key': 'new', 'complete': True, 'marker': True}
  b1 = legacy_boot(old_root, 'boot-B')
  b2 = legacy_boot(old_root, 'boot-C')
  print('MEASURE', point, json.dumps([b1, b2]))
  assert '--bootstrap-only' not in b1['server']['calls'] + b2['server']['calls']
  assert b1['after']['key'] == b2['after']['key'] == 'new'
  assert b1['apply'] == {'gate': True, 'selected': NEW_COMMIT} == b2['apply']
  assert_whole(old_root)


def test_normal_switch_survives_two_legacy_boots(old_root):
  assert run_switch(old_root).returncode == 0
  for boot in ('boot-B', 'boot-C'):
    b = legacy_boot(old_root, boot)
    assert b['apply'] == {'gate': True, 'selected': NEW_COMMIT} and b['after']['key'] == 'new'
    assert '--bootstrap-only' not in b['server']['calls']


def test_same_boot_legacy_server_restart_after_switch(old_root):
  """Server restarted in the switch boot (boot-A): gate off, no reinstall."""
  assert run_switch(old_root).returncode == 0
  b = legacy_boot(old_root, 'boot-A')
  assert b['apply']['gate'] is False and not b['server']['wait_for_runtime_blocks']
  assert b['after']['key'] == 'new' and '--bootstrap-only' not in b['server']['calls']


# ---- items 4/5: injected I/O errors ----------------------------------------

FAIL_POINTS = ['fsync_after_exchange', 'marker_rename', 'marker_fsync', 'rename_old', 'verify']


def tool_markers(rt):
  return sorted(p.name for p in rt.iterdir() if p.name.startswith('boot-update-required'))


@pytest.mark.parametrize('point', FAIL_POINTS)
@pytest.mark.parametrize('marker', [None, 'next-boot\n'])
def test_injected_error_restores_updater_and_marker(old_root, point, marker):
  rt = runtime(old_root)
  if marker is not None:
    (rt / 'boot-update-required').write_text(marker)
  before = sw.tree_hashes(rt / 'updater')
  r = run_switch(old_root, env={'JET_SWITCH_FAIL_AT': point})
  if marker is not None and point in ('marker_rename', 'marker_fsync'):
    # Existing marker is kept, write_marker never runs: no injection reached.
    assert r.returncode == 0, r.stderr
    assert (rt / 'boot-update-required').read_text() == marker
    return
  assert r.returncode != 0 and 'old updater restored' in r.stderr and 'injected' in r.stderr, r.stderr
  state = sw.boot_state(old_root)
  assert sw.tree_hashes(rt / 'updater') == before and state['key'] == 'old'
  # Never "old updater + marker written by the tool".
  if marker is None:
    assert state['marker'] is False
    assert all('rolledback-S1' in n for n in tool_markers(rt) if n != 'boot-update-required')
    moved = rt / 'boot-update-required.rolledback-S1'
    if point in ('marker_fsync', 'rename_old', 'verify'):
      assert moved.read_text() == 'boot-A\n'  # moved aside, not deleted
  else:
    assert (rt / 'boot-update-required').read_text() == marker
  sw.check_old_updater(PY, rt / 'updater', rt / 'updater-switch-backup/S1')
  # Old behaviour intact, and a later run still completes.
  b = legacy_boot(old_root, 'boot-B')
  assert b['after']['key'] == 'old'
  again = run_switch(old_root, stamp='S2')
  assert again.returncode == 0, again.stderr
  assert assert_whole(old_root) == {'key': 'new', 'complete': True, 'marker': True}


def test_marker_changed_by_someone_else_is_restored_to_original(old_root, monkeypatch):
  """Original marker content comes back even if it was altered before failure."""
  rt = runtime(old_root)
  (rt / 'boot-update-required').write_text('orig\n')
  sw.restore_marker(rt / 'boot-update-required', b'orig\n', 'X')
  assert (rt / 'boot-update-required').read_text() == 'orig\n'
  (rt / 'boot-update-required').write_text('changed\n')
  sw.restore_marker(rt / 'boot-update-required', b'orig\n', 'X')
  assert (rt / 'boot-update-required').read_text() == 'orig\n'
  (rt / 'boot-update-required').unlink()
  sw.restore_marker(rt / 'boot-update-required', b'orig\n', 'X')
  assert (rt / 'boot-update-required').read_text() == 'orig\n'


def test_exchange_has_no_fsync_outside_recovery(old_root, monkeypatch):
  """exchange() only swaps; its fsync now runs inside the recovery scope."""
  import inspect
  assert 'fsync_dir' not in inspect.getsource(sw.exchange).split('"""')[-1].split('# No fsync')[0]


# ---- item 3: full next boot with the switched updater ----------------------

def make_release(tmp, commit, model_bytes):
  spec = {'sha256': hashlib.sha256(model_bytes).hexdigest(), 'nbytes': len(model_bytes)}
  buf = io.BytesIO()
  with tarfile.open(fileobj=buf, mode='w:gz') as tar:
    for name, data in [('SOURCE_COMMIT', (commit + '\n').encode()),
                       ('openpilot/selfdrive/modeld/jetlink/jetson_model.json', json.dumps(spec).encode()),
                       ('tools/jetlink/README', b'fake release\n')]:
      info = tarfile.TarInfo(name)
      info.size = len(data)
      tar.addfile(info, io.BytesIO(data))
  bundle = buf.getvalue()
  return bundle


def signed_manifest(tmp, commit, bundle, model_bytes):
  from Crypto.PublicKey import ECC
  from Crypto.Signature import eddsa
  key = ECC.generate(curve='ed25519')
  (tmp / 'test-signing-public.pem').write_text(key.public_key().export_key(format='PEM'))
  base = 'https://github.com/2020K9/carrotpilot/releases/download/'
  m = {'format': 1, 'source_commit': commit,
       'runtime': {'arch': 'aarch64', 'l4t': '36.4.7', 'tensorrt': '10.3.0'},
       'bundle': {'url': base + 'k9-jetlink-host-' + commit + '/precompiled-runtime.tar.gz',
                  'sha256': hashlib.sha256(bundle).hexdigest(), 'size': len(bundle)},
       'model': {'url': base + 'k9-test-model/big_driving_supercombo.onnx',
                 'sha256': hashlib.sha256(model_bytes).hexdigest(), 'size': len(model_bytes)},
       'storage_format': 1}
  payload = json.dumps(m, sort_keys=True, separators=(',', ':')).encode()
  m['signature'] = base64.b64encode(eddsa.new(key, 'rfc8032').sign(payload)).decode()
  (tmp / 'manifest.json').write_text(json.dumps(m))
  return m


GATE_RUN = r'''
import sys, json, io, threading
from pathlib import Path as P
updater, rt, bid, man, files, pem, corrupt = sys.argv[1:8]
sys.path.insert(0, updater)
import update_host as u, boot_update as b
u.ROOT = P(rt); u.PROTECTED_MARKER = P(rt) / 'no-protected-marker'
b.BOOT_ID = P(bid); b.READY = P(rt) / 'run-ready.json'; b.STATUS = P(rt) / 'run-status.json'
real_verify = u.verify_signature
def verify(manifest):  # same code path, test key instead of the release key
  from Crypto.PublicKey import ECC
  from Crypto.Signature import eddsa
  import base64
  value = {k: v for k, v in manifest.items() if k != 'signature'}
  payload = json.dumps(value, sort_keys=True, separators=(',', ':')).encode()
  eddsa.new(ECC.import_key(P(pem).read_text()), 'rfc8032').verify(payload, base64.b64decode(manifest['signature'], validate=True))
u.verify_signature = verify
log = []
u.check_runtime = lambda m: log.append('check_runtime(skipped: dpkg/tensorrt not on worker)')
u.probe_release = lambda r: log.append('probe_release(skipped: GPU probe)')
import pwd, os
class PW: pw_uid = os.getuid(); pw_gid = os.getgid()
pwd.getpwnam = lambda n: PW
class Resp(io.BytesIO):
  def __init__(self, data, url): super().__init__(data); self.url = url
  def __enter__(self): return self
  def __exit__(self, *a): return False
def urlopen(url, timeout=None):
  name = url.rsplit('/', 1)[-1]
  data = (P(files) / name).read_bytes()
  if corrupt == '1' and name.endswith('.tar.gz'):
    data = data[:-1] + bytes([data[-1] ^ 1])
  log.append('download ' + name)
  return Resp(data, 'https://release-assets.githubusercontent.com/github-production-release-asset/1400258213/' + name)
u.urllib.request.urlopen = urlopen
def run(c, **k):
  log.append(' '.join(c[:2] + c[2:3]))
  class R: returncode = 1  # systemctl is-active: services stopped
  return R()
u.subprocess.run = run; b.subprocess.run = run
m = json.loads(P(man).read_text())
g = b.Gate()
enabled = b.enabled()
g.select(m)
g.tick(); g.worker.join(60)
first = g.state
g.received = __import__('time').monotonic()
g.tick()
print(json.dumps({'gate': enabled, 'after_install': first, 'state': g.state, 'error': g.error, 'done': g.done, 'log': log}))
'''


@pytest.mark.parametrize('corrupt', [False, True])
def test_next_boot_downloads_verifies_unpacks_activates(old_root, tmp_path, corrupt):
  assert run_switch(old_root).returncode == 0
  rt = runtime(old_root)
  commit = 'a' * 40
  model = b'fake-onnx-model' * 1000
  files = tmp_path / 'files'
  files.mkdir()
  bundle = make_release(tmp_path, commit, model)
  (files / 'precompiled-runtime.tar.gz').write_bytes(bundle)
  (files / 'big_driving_supercombo.onnx').write_bytes(model)
  signed_manifest(tmp_path, commit, bundle, model)
  (rt / 'cache/models').mkdir(parents=True)
  (old_root.parent / 'boot_id_next').write_text('boot-B\n')
  before = os.readlink(rt / 'current')
  r = subprocess.run([PY, '-c', GATE_RUN, str(rt / 'updater'), str(rt), str(old_root.parent / 'boot_id_next'),
                      str(tmp_path / 'manifest.json'), str(files), str(tmp_path / 'test-signing-public.pem'),
                      '1' if corrupt else '0'], capture_output=True, text=True, timeout=120, env=ENV)
  assert r.returncode == 0, r.stderr
  out = json.loads(r.stdout)
  print('MEASURE next boot', json.dumps(out))
  assert out['gate'] is True
  if corrupt:
    assert out['state'] == 'failed' and 'checksum' in out['error'] and not out['done']
    assert os.readlink(rt / 'current') == before
    assert not (rt / 'releases' / commit).exists()
    return
  assert out['done'] and out['state'] == 'ready', out
  assert (rt / 'current').resolve() == (rt / 'releases' / commit).resolve()
  assert (rt / 'current/SOURCE_COMMIT').read_text().strip() == commit
  assert (rt / 'releases' / commit / '.bundle-sha256').read_text() == hashlib.sha256(bundle).hexdigest()
  assert (rt / 'cache/models' / (hashlib.sha256(model).hexdigest()[:16] + '.onnx')).read_bytes() == model
  assert json.loads((rt / 'updates/status.json').read_text())['state'] == 'applied'
  assert not (rt / 'updates/transaction.json').exists() and not (rt / 'updates/pending.json').exists()
  assert json.loads((rt / 'run-ready.json').read_text()) == {'boot_id': 'boot-B', 'source_commit': commit}
  assert 'systemctl stop' in ' '.join(out['log'])
