"""One-time NAS->GitHub updater switch, in a fake root (never /opt)."""
import base64
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import switch_updater as sw  # noqa: E402

OLD_COMMIT = '0cb67d46c'
MANIFEST = HERE / 'switch_fixtures/host_release_cdc5ef408.json'
NEW_COMMIT = '063613a1650b525564f852cecf14c2c8104e74d7'
PY = sys.executable
OLD_NAMES = ['finalize_sd_image.py', 'hud_protocol.py', 'release-signing-public.pem', 'wifi_protocol.py',
             'local_release.py', 'usb_release.py', 'boot_update.py', 'update_host.py', 'install_updates.py']


def git_show(rev, path):
  return subprocess.run(['git', 'show', f'{rev}:{path}'], cwd=HERE, check=True, capture_output=True).stdout


@pytest.fixture
def old_root(tmp_path):
  """Legacy Jetson: updater installed from 0cb67d46c, old key, NAS host."""
  root = tmp_path / 'root'
  runtime = root / 'opt/carrot-jetlink'
  updater = runtime / 'updater'
  updater.mkdir(parents=True)
  for name in OLD_NAMES[:-1]:
    (updater / name).write_bytes(git_show(OLD_COMMIT, 'tools/jetlink/' + name))
  (updater / 'extra_old_only.txt').write_text('kept\n')
  # Legacy runtime release with its own install_updates (migrate_running_release).
  old_src = runtime / 'releases/old/tools/jetlink'
  old_src.mkdir(parents=True)
  for name in OLD_NAMES:
    (old_src / name).write_bytes(git_show(OLD_COMMIT, 'tools/jetlink/' + name))
  (runtime / 'releases/old/SOURCE_COMMIT').write_text('0' * 40 + '\n')
  (runtime / 'current').symlink_to(runtime / 'releases/old')
  units = root / 'etc/systemd/system'
  units.mkdir(parents=True)
  cmd = '/opt/carrot-jetlink/venv/bin/python /opt/carrot-jetlink/updater/update_host.py'
  (units / 'carrot-jetlink-update-apply.service').write_text(f'[Service]\nExecStart={cmd} activate\n')
  (units / 'carrot-jetlink-update-stage.service').write_text(f'[Service]\nExecStart={cmd} automatic\n')
  (units / 'carrot-jetlink-update-stage.timer').write_text('[Timer]\nOnBootSec=2min\n')
  (tmp_path / 'boot_id').write_text('boot-A\n')
  return root


def run_switch(root, env=None, manifest=MANIFEST, stamp='S1'):
  e = dict(os.environ)
  e.update(env or {})
  e['PYTHONDONTWRITEBYTECODE'] = '1'
  code = ('import sys,json;sys.path.insert(0,sys.argv[1]);import switch_updater as s;'
          'print(json.dumps(s.switch(sys.argv[2],sys.argv[1],sys.argv[3],sys.executable,sys.argv[4],stamp=sys.argv[5])))')
  return subprocess.run([PY, '-c', code, str(HERE), str(root), str(manifest), str(root.parent / 'boot_id'), stamp],
                        capture_output=True, text=True, env=e, timeout=120)


def runtime(root):
  return root / 'opt/carrot-jetlink'


def new_hashes():
  return {n: sw.sha(HERE / n) for n in sw.NAMES}


def assert_whole(root):
  """Next boot sees exactly one complete updater, never a mix."""
  updater = runtime(root) / 'updater'
  have = sw.tree_hashes(updater)
  state = sw.boot_state(root)
  assert state['complete']
  if state['key'] == 'new':
    assert all(have[n] == h for n, h in new_hashes().items())
  else:
    assert state['key'] == 'old'
    for n in OLD_NAMES[:-1]:
      assert have[n] == __import__('hashlib').sha256(git_show(OLD_COMMIT, 'tools/jetlink/' + n)).hexdigest()
  return state


def test_real_github_manifest_signature_verifies_with_new_key_only(old_root):
  assert sw.verify_updater(PY, HERE, old_root, MANIFEST, sw.NEW_KEY) == NEW_COMMIT
  with pytest.raises(sw.SwitchError):
    sw.verify_updater(PY, runtime(old_root) / 'updater', old_root, MANIFEST, sw.OLD_KEY)


def test_normal_switch(old_root):
  before = sw.tree_hashes(runtime(old_root) / 'updater')
  result = run_switch(old_root)
  assert result.returncode == 0, result.stderr
  report = json.loads(result.stdout)
  assert report['result'] == 'switched' and report['selected_commit'] == NEW_COMMIT
  state = assert_whole(old_root)
  assert state == {'key': 'new', 'complete': True, 'marker': True}
  assert (runtime(old_root) / 'boot-update-required').read_text() == 'boot-A\n'
  assert (runtime(old_root) / 'updater/extra_old_only.txt').read_text() == 'kept\n'
  backup = runtime(old_root) / 'updater-switch-backup/S1'
  assert sw.tree_hashes(backup / 'updater') == before
  assert sorted(p.name for p in (backup / 'units').iterdir()) == sorted(sw.UNITS)
  assert json.loads((backup / 'BACKUP.json').read_text())['marker_present'] is False
  assert sw.tree_hashes(runtime(old_root) / 'updater.replaced-S1') == before


def test_backup_never_overwritten(old_root):
  (runtime(old_root) / 'updater-switch-backup/S1').mkdir(parents=True)
  result = run_switch(old_root)
  assert result.returncode != 0 and 'refusing to overwrite' in result.stderr
  assert sw.boot_state(old_root) == {'key': 'old', 'complete': True, 'marker': False}


def next_boot(root, manifest_path, boot_id='boot-B'):
  """Model the services after reboot: ExecStart path -> installed updater."""
  unit = (root / 'etc/systemd/system/carrot-jetlink-update-apply.service').read_text()
  script = unit.split('ExecStart=')[1].split()[1]
  updater = root / script.lstrip('/')
  updater = updater.parent
  (root.parent / 'boot_id_next').write_text(boot_id + '\n')
  code = ('import sys,json;from pathlib import Path;sys.path.insert(0,sys.argv[1]);'
          'import update_host as u, boot_update as b;'
          'u.ROOT=Path(sys.argv[2]);u.PROTECTED_MARKER=Path(sys.argv[2])/"no-marker";b.BOOT_ID=Path(sys.argv[3]);'
          'g=b.Gate();m=json.loads(Path(sys.argv[4]).read_text());'
          'en=b.enabled()\n'
          'try:\n g.select(m);print(json.dumps({"gate":en,"selected":g.source_commit}))\n'
          'except Exception as e:\n print(json.dumps({"gate":en,"error":type(e).__name__+": "+str(e)}))')
  out = subprocess.run([PY, '-c', code, str(updater), str(runtime(root)), str(root.parent / 'boot_id_next'),
                        str(manifest_path)], capture_output=True, text=True, timeout=60,
                       env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
  assert out.returncode == 0, out.stderr
  return json.loads(out.stdout)


def test_next_boot_uses_new_updater_and_accepts_github(old_root):
  assert run_switch(old_root).returncode == 0
  assert next_boot(old_root, MANIFEST) == {'gate': True, 'selected': NEW_COMMIT}
  # Same boot as the switch: gate stays off (no interruption of this boot).
  assert next_boot(old_root, MANIFEST, boot_id='boot-A')['gate'] is False


def test_before_switch_old_updater_rejects_github(old_root):
  out = next_boot(old_root, MANIFEST)
  assert out['gate'] is False and 'NAS' in out['error']


def nas_manifest(tmp_path):
  m = json.loads(MANIFEST.read_text())
  for k in ('bundle', 'model'):
    m[k]['url'] = 'https://upload.shind0.synology.me/models/jetlink-host-stable/' + k
  m['signature'] = base64.b64encode(b'\0' * 64).decode()
  path = tmp_path / 'nas.json'
  path.write_text(json.dumps(m))
  return path


def test_old_nas_manifest_after_switch_is_rejected_without_change(old_root, tmp_path):
  assert run_switch(old_root).returncode == 0
  before = sw.tree_hashes(runtime(old_root) / 'updater')
  out = next_boot(old_root, nas_manifest(tmp_path))
  assert out['gate'] is True and 'GitHub' in out['error']
  assert sw.tree_hashes(runtime(old_root) / 'updater') == before
  assert os.readlink(runtime(old_root) / 'current').endswith('releases/old')


def test_verification_failure_restores_old(old_root):
  before = sw.tree_hashes(runtime(old_root) / 'updater')
  result = run_switch(old_root, env={'JET_SWITCH_FORCE_VERIFY_FAIL': '1'})
  assert result.returncode != 0 and 'old updater restored' in result.stderr
  assert sw.tree_hashes(runtime(old_root) / 'updater') == before
  assert sw.boot_state(old_root) == {'key': 'old', 'complete': True, 'marker': False}
  sw.check_old_updater(PY, runtime(old_root) / 'updater', runtime(old_root) / 'updater-switch-backup/S1')
  assert 'NAS' in next_boot(old_root, MANIFEST)['error']  # old behaviour intact


def test_bad_signature_stops_before_any_visible_change(old_root, tmp_path):
  m = json.loads(MANIFEST.read_text())
  m['bundle']['size'] += 1
  bad = tmp_path / 'bad.json'
  bad.write_text(json.dumps(m))
  before = sw.tree_hashes(runtime(old_root) / 'updater')
  result = run_switch(old_root, manifest=bad)
  assert result.returncode != 0 and 'manifest rejected' in result.stderr
  assert sw.tree_hashes(runtime(old_root) / 'updater') == before
  assert not (runtime(old_root) / 'boot-update-required').exists()


CRASH_POINTS = ['after_backup', 'mid_stage', 'after_stage', 'after_exchange', 'after_rename_old',
                'after_marker_tmp', 'after_marker']


@pytest.mark.parametrize('point', CRASH_POINTS)
@pytest.mark.parametrize('marker', [None, 'next-boot\n'])
def test_power_cut_leaves_whole_old_or_new(old_root, point, marker):
  if marker is not None:
    (runtime(old_root) / 'boot-update-required').write_text(marker)
  result = run_switch(old_root, env={'JET_SWITCH_CRASH_AT': point})
  if marker is not None and point == 'after_marker_tmp':
    assert result.returncode == 0, result.stderr  # existing marker is never rewritten
  else:
    assert result.returncode == 91, result.stderr
  state = assert_whole(old_root)
  if marker is None and state['key'] == 'old':
    assert not state['marker'], 'gate must never be enabled with the old updater'
  if marker is not None:
    assert (runtime(old_root) / 'boot-update-required').read_text() == marker
  # Next boot picks something consistent; a rerun finishes the switch.
  again = run_switch(old_root, stamp='S2')
  assert again.returncode == 0, again.stderr
  assert json.loads(again.stdout)['result'] in ('switched', 'already-new')
  assert assert_whole(old_root) == {'key': 'new', 'complete': True, 'marker': True}
  assert next_boot(old_root, MANIFEST) == {'gate': True, 'selected': NEW_COMMIT}


# after_rename_old dropped: since 9bdee0692 the marker is written right after the exchange, so that
# point already has new updater + marker; its expectation (no reinstall) is in test_switch_review.py.
@pytest.mark.parametrize('point', ['after_exchange', 'after_marker_tmp'])
def test_power_cut_new_without_marker_then_legacy_start(old_root, point):
  # If the legacy server starts before a rerun, its own bootstrap reinstalls
  # the old updater (whole, upstream code) and turns the gate on.
  assert run_switch(old_root, env={'JET_SWITCH_CRASH_AT': point}).returncode == 91
  state = sw.boot_state(old_root)
  assert state['key'] == 'new' and not state['marker']
  assert '--bootstrap-only' in migrate_calls(old_root)


def migrate_calls(root):
  """Run the legacy runtime's migrate_running_release() with sudo recorded."""
  src = runtime(root) / 'current/tools/jetlink'
  code = ('import sys,json;from pathlib import Path;sys.path.insert(0,sys.argv[1]);import subprocess,install_updates as i;'
          'calls=[];subprocess.run=lambda c,**k:calls.append(c[-1]);'
          'i.Path=Path;i.migrate_running_release(Path(sys.argv[2]));print(json.dumps(calls))')
  out = subprocess.run([PY, '-c', code, str(src), str(runtime(root))], capture_output=True, text=True, timeout=60,
                       env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
  assert out.returncode == 0, out.stderr
  return json.loads(out.stdout)


def test_marker_absent_legacy_runtime_would_reinstall_old_updater(old_root):
  assert '--bootstrap-only' in migrate_calls(old_root)


def test_after_switch_legacy_runtime_does_not_reinstall(old_root):
  assert run_switch(old_root).returncode == 0
  assert '--bootstrap-only' not in migrate_calls(old_root)


def test_marker_present_before_is_preserved(old_root):
  (runtime(old_root) / 'boot-update-required').write_text('next-boot\n')
  result = run_switch(old_root)
  assert result.returncode == 0, result.stderr
  assert (runtime(old_root) / 'boot-update-required').read_text() == 'next-boot\n'
  assert (runtime(old_root) / 'updater-switch-backup/S1/boot-update-required').read_text() == 'next-boot\n'
  assert next_boot(old_root, MANIFEST) == {'gate': True, 'selected': NEW_COMMIT}


def test_rerun_after_success_is_noop(old_root):
  assert run_switch(old_root).returncode == 0
  before = sw.tree_hashes(runtime(old_root) / 'updater')
  again = run_switch(old_root, stamp='S2')
  assert json.loads(again.stdout)['result'] == 'already-new'
  assert sw.tree_hashes(runtime(old_root) / 'updater') == before
  assert not (runtime(old_root) / 'updater-switch-backup/S2').exists()


def test_unknown_key_refuses(old_root):
  (runtime(old_root) / 'updater/release-signing-public.pem').write_text(
    '-----BEGIN PUBLIC KEY-----\nAAAA\n-----END PUBLIC KEY-----\n')
  result = run_switch(old_root)
  assert result.returncode != 0 and 'neither' in result.stderr
