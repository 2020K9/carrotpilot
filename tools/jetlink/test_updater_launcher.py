"""updater_launcher.py / install_launcher.py on fake roots (no device, no root).

Stub updaters stand in for update_host.py (they take the same flock the real one
takes); the installer, the legacy bytes, the legacy bootstrap()/configure() and
the new updater's signature check use the real files.  crash points are os._exit
(process kill), NOT a power cut.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import pytest

HERE = Path(__file__).resolve().parent
PY = sys.executable
OLD_COMMIT = '0cb67d46c'
sys.path.insert(0, str(HERE))
import install_launcher
import updater_launcher as ul
from switch_updater import NAMES, REQUIRED

STUB = r'''
import fcntl, json, os, sys, time, subprocess
from pathlib import Path
ROLE = %r
if ROLE == 'new' and os.environ.get('STUB_NEW') == 'import_error':
  import module_that_does_not_exist  # noqa
root = Path(os.environ['STUB_ROOT'])
rt = root / 'opt/carrot-jetlink'
mode = os.environ.get('STUB_' + ROLE.upper(), 'ok')
log = root / 'log.jsonl'
def note(**kv):
  kv.update(role=ROLE, pid=os.getpid(), t=time.time())
  with log.open('a') as s: s.write(json.dumps(kv) + '\n')
with (root / 'run/carrot-jetlink-update.lock').open('w') as lock:
  fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)   # same call as update_host.main()
  # Past the lock: every earlier updater process must be gone.
  for other in root.glob('pids.*'):
    for pid in other.read_text().split():
      if pid != str(os.getpid()) and Path('/proc/' + pid).exists() and Path('/proc/%%s/stat' %% pid).read_text().rsplit(')', 1)[1].split()[0] != 'Z':
        note(event='OVERLAP', alive=int(pid))
  with (root / ('pids.' + ROLE)).open('a') as s: s.write('%%d\n' %% os.getpid())
  note(event='start', action=sys.argv[1], mode=mode)
  tx = rt / 'updates/transaction.json'
  if mode == 'ok':
    if tx.exists():  # what update_host.activate() does with a leftover transaction
      saved = json.loads(tx.read_text())
      link = rt / 'current.recovery'
      if link.is_symlink(): link.unlink()
      link.symlink_to(saved['release']); os.replace(link, rt / 'current'); tx.unlink()
      note(event='recovered')
  elif mode == 'exit':
    sys.exit(3)
  elif mode == 'connect':
    import urllib.error; raise urllib.error.URLError('connection refused')
  elif mode == 'verify':
    raise ValueError('Signature is not authentic')
  elif mode == 'sha':
    raise ValueError('Update checksum/length mismatch')
  elif mode == 'rejected':
    (rt / 'updates/status.json').write_text(json.dumps({'state': 'rejected', 'error': 'probe', 'updated': time.time()}))
  elif mode == 'applied':
    (rt / 'updates/status.json').write_text(json.dumps({'state': 'applied', 'updated': time.time()}))
  elif mode in ('hang', 'apply_hang', 'apply_crash'):
    if mode != 'hang':
      tx.write_text(json.dumps({'release': str((rt / 'current').resolve()), 'last_loaded': None}))
      (rt / 'updates/pending.json').write_text('{}')
      link = rt / 'current.next'; link.symlink_to(rt / 'releases/B'); os.replace(link, rt / 'current')
      note(event='switched_to_B')
    if mode == 'apply_crash':
      os._exit(1)
    kid = subprocess.Popen([sys.executable, '-c', 'import signal,time;signal.signal(signal.SIGTERM,signal.SIG_IGN);time.sleep(600)'])
    with (root / ('pids.' + ROLE)).open('a') as s: s.write('%%d\n' %% kid.pid)
    signal_ignore = __import__('signal'); signal_ignore.signal(signal_ignore.SIGTERM, signal_ignore.SIG_IGN)
    time.sleep(600)
  elif mode == 'slow':
    time.sleep(float(os.environ.get('STUB_SLOW', '2')))
  elif mode == 'fail':
    raise RuntimeError('legacy failed')
  note(event='end')
'''


def old_file(name):
  return subprocess.run(['git', 'show', f'{OLD_COMMIT}:tools/jetlink/{name}'], cwd=HERE, check=True,
                        capture_output=True).stdout


def make_root(tmp_path, new_files=True, legacy_stub=True):
  root = tmp_path / 'root'
  rt = root / 'opt/carrot-jetlink'
  for name in ('A', 'B'):
    (rt / 'releases' / name).mkdir(parents=True)
  (rt / 'current').symlink_to(rt / 'releases/A')
  (rt / 'updates').mkdir()
  boots = root / 'boots'
  boots.mkdir(parents=True)
  set_boot(root, 1)
  # legacy source: real old public key, stub entry point
  src = tmp_path / 'legacy_src'
  src.mkdir()
  for name in REQUIRED:
    data = old_file(name) if name == 'release-signing-public.pem' else b'# stub\n'
    (src / name).write_bytes(data)
  if legacy_stub:
    (src / 'update_host.py').write_text(STUB % 'legacy')
  slot = rt / 'updater'
  slot.mkdir()
  if new_files:
    for name in REQUIRED:
      (slot / name).write_bytes(b'# new stub ' + name.encode() + b'\n')
    (slot / 'update_host.py').write_text(STUB % 'new')
  install_launcher.install(root, src, HERE / 'updater_launcher.py')
  return root


def set_boot(root, n):
  """A reboot: new boot id and an empty /run (the old /run is kept, never deleted)."""
  run = root / 'boots' / (f'run{n}')
  run.mkdir()
  (root / 'proc/sys/kernel/random').mkdir(parents=True, exist_ok=True)
  (root / 'proc/sys/kernel/random/boot_id').write_text(f'boot-{n}\n')
  link = root / 'run.new'
  link.symlink_to(run)
  os.replace(link, root / 'run')


def launch(root, action='activate', env=None, new_timeout=20, legacy_timeout=20, background=False):
  e = dict(os.environ, STUB_ROOT=str(root), PYTHONDONTWRITEBYTECODE='1')
  e.update(env or {})
  args = [PY, str(root / 'opt/carrot-jetlink/launcher/updater_launcher.py'), action, '--root', str(root),
          '--new-timeout', str(new_timeout), '--legacy-timeout', str(legacy_timeout)]
  if background:
    return subprocess.Popen(args, env=e, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
  return subprocess.run(args, env=e, capture_output=True, text=True, timeout=120)


def events(root):
  path = root / 'log.jsonl'
  return [json.loads(x) for x in path.read_text().splitlines()] if path.exists() else []


def roles(root):
  return [(e['role'], e['event']) for e in events(root) if e['event'] in ('start', 'end')]


def state(root):
  return json.loads((root / 'opt/carrot-jetlink/launcher/state.json').read_text())


def current(root):
  return (root / 'opt/carrot-jetlink/current').resolve().name


def no_overlap(root):
  assert not [e for e in events(root) if e['event'] == 'OVERLAP'], events(root)


def test_new_ok_runs_only_new(tmp_path):
  root = make_root(tmp_path)
  r = launch(root)
  assert r.returncode == ul.EXIT_OK, r.stderr
  assert roles(root) == [('new', 'start'), ('new', 'end')]
  assert state(root)['phase'] == 'done' and state(root)['new']['kind'] == 'ok'


def test_new_applied_counts_as_ok(tmp_path):
  root = make_root(tmp_path)
  r = launch(root, env={'STUB_NEW': 'applied'})
  assert r.returncode == ul.EXIT_OK and state(root)['new']['kind'] == 'applied'
  assert roles(root) == [('new', 'start'), ('new', 'end')]


@pytest.mark.parametrize('mode,kind', [('missing', 'unrunnable'), ('import_error', 'unrunnable'),
                                       ('exit', 'unrunnable'), ('connect', 'connect')])
def test_new_unrunnable_or_offline_falls_back_to_legacy(tmp_path, mode, kind):
  root = make_root(tmp_path, new_files=(mode != 'missing'))
  r = launch(root, env={'STUB_NEW': mode})
  assert r.returncode == ul.EXIT_LEGACY_OK, (r.stdout, r.stderr)
  s = state(root)
  assert s['new']['kind'] == kind and s['legacy']['kind'] == 'ok'
  assert ('legacy', 'start') in roles(root) and ('legacy', 'end') in roles(root)
  assert 'ran != updated' in s['result']
  no_overlap(root)


def test_new_timeout_kills_group_then_legacy(tmp_path):
  root = make_root(tmp_path)
  t = time.monotonic()
  r = launch(root, env={'STUB_NEW': 'hang'}, new_timeout=2)
  assert r.returncode == ul.EXIT_LEGACY_OK, r.stderr
  s = state(root)
  assert s['new'] == {'kind': 'timeout', 'group_stopped': True}
  for pid in (root / 'pids.new').read_text().split():  # stub and its SIGTERM-ignoring child
    p = Path('/proc') / pid
    assert not p.exists() or (p / 'stat').read_text().rsplit(')', 1)[1].split()[0] == 'Z'
  no_overlap(root)
  assert time.monotonic() - t < 60


@pytest.mark.parametrize('mode', ['verify', 'sha', 'rejected'])
def test_verification_failure_blocks_and_never_runs_legacy(tmp_path, mode):
  root = make_root(tmp_path)
  r = launch(root, env={'STUB_NEW': mode})
  assert r.returncode == ul.EXIT_BLOCKED
  assert [x for x in roles(root) if x[0] == 'legacy'] == []
  assert 'legacy not run' in state(root)['result']


def test_real_new_updater_signature_failure_is_classified_verify(tmp_path):
  """Real 063613a16-line update_host.verify_signature on a tampered manifest."""
  import Crypto  # noqa: F401
  root = make_root(tmp_path)
  slot = root / 'opt/carrot-jetlink/updater'
  for name in REQUIRED:
    (slot / name).write_bytes((HERE / name).read_bytes())
  (slot / 'update_host.py').write_bytes((HERE / 'update_host.py').read_bytes())
  # entry that loads the real module and hands it a tampered manifest
  real = slot / 'update_host.py'
  wrapper = slot / 'entry_check.py'
  wrapper.write_text(f'import sys,json;sys.path.insert(0,{str(slot)!r});import update_host as u\n'
                     + 'u.verify_signature({"source_commit":"x","signature":"AAAA"})\n')
  result = tmp_path / 'res.json'
  with (root / 'run/carrot-jetlink-update.lock').open('a') as lock:
    code = subprocess.run([PY, str(HERE / 'updater_launcher.py'), '--child', str(wrapper), 'activate', str(result),
                           str(lock.fileno()), str(root / 'opt/carrot-jetlink')], pass_fds=(lock.fileno(),),
                          capture_output=True, text=True, timeout=60)
  assert code.returncode == 1 and json.loads(result.read_text())['kind'] == 'verify', code.stderr
  assert real.read_bytes() == (HERE / 'update_host.py').read_bytes()


def test_apply_interrupted_restores_current_then_legacy(tmp_path):
  root = make_root(tmp_path)
  r = launch(root, env={'STUB_NEW': 'apply_crash'})
  assert r.returncode == ul.EXIT_LEGACY_OK, r.stderr
  rt = root / 'opt/carrot-jetlink'
  s = state(root)
  assert s['new']['kind'] == 'crash' and s['restore'] == 'restored A'
  assert current(root) == 'A' and not (rt / 'updates/transaction.json').exists()
  assert (rt / 'updates/interrupted.json').exists()
  assert json.loads((rt / 'updates/status.json').read_text())['state'] == 'recovered'
  # legacy started only after current was back on A
  assert [e['event'] for e in events(root)] [-2:] == ['start', 'end']


def test_apply_hang_timeout_restores_then_legacy(tmp_path):
  root = make_root(tmp_path)
  r = launch(root, env={'STUB_NEW': 'apply_hang'}, new_timeout=2)
  assert r.returncode == ul.EXIT_LEGACY_OK, r.stderr
  assert current(root) == 'A' and state(root)['restore'] == 'restored A'
  no_overlap(root)


def test_invalid_transaction_blocks_legacy(tmp_path):
  root = make_root(tmp_path)
  rt = root / 'opt/carrot-jetlink'
  (rt / 'updates/transaction.json').write_text(json.dumps({'release': '/elsewhere', 'last_loaded': None}))
  r = launch(root, env={'STUB_NEW': 'connect'})
  assert r.returncode == ul.EXIT_RESTORE_FAILED
  assert [x for x in roles(root) if x[0] == 'legacy'] == []


def test_changed_legacy_bytes_are_not_run(tmp_path):
  root = make_root(tmp_path)
  with (root / 'opt/carrot-jetlink/updater-legacy/update_host.py').open('a') as s:
    s.write('# changed\n')
  r = launch(root, env={'STUB_NEW': 'connect'})
  assert r.returncode == ul.EXIT_LEGACY_MISSING
  assert [x for x in roles(root) if x[0] == 'legacy'] == []


def test_legacy_failure_is_reported_as_failure(tmp_path):
  root = make_root(tmp_path)
  r = launch(root, env={'STUB_NEW': 'connect', 'STUB_LEGACY': 'fail'})
  assert r.returncode == ul.EXIT_LEGACY_FAILED


CRASH_POINTS = ['after_start', 'new_running', 'new_result', 'restore_before_replace', 'restore_before_unlink',
                'before_legacy', 'legacy_running']


@pytest.mark.parametrize('point', CRASH_POINTS)
def test_launcher_killed_then_two_reboots(tmp_path, point):
  root = make_root(tmp_path)
  rt = root / 'opt/carrot-jetlink'
  r = launch(root, env={'STUB_NEW': 'apply_crash', 'JET_LAUNCHER_CRASH_AT': point})
  assert r.returncode == 137
  assert state(root)['phase'] != 'done'
  # never both: at most one updater started in the killed run, legacy never with B current
  for boot in (2, 3):
    set_boot(root, boot)
    r = launch(root)  # new updater healthy after the reboot
    assert r.returncode == ul.EXIT_OK, (point, boot, r.stdout, r.stderr)
    s = state(root)
    assert s['phase'] == 'done' and s['boot_id'] == f'boot-{boot}'
    if boot == 2:
      assert s['previous_interrupted']['boot_id'] == 'boot-1'
    else:
      assert 'previous_interrupted' not in s
    assert current(root) == 'A' and not (rt / 'updates/transaction.json').exists()
  no_overlap(root)


def test_launcher_killed_while_new_hangs_keeps_lock(tmp_path):
  """Parent killed: the orphaned new updater keeps the inherited lock, so a second
  start program and a direct legacy timer run both stop on the lock."""
  root = make_root(tmp_path)
  p = launch(root, env={'STUB_NEW': 'slow', 'STUB_SLOW': '6'}, background=True)
  deadline = time.monotonic() + 20
  while ('new', 'start') not in roles(root) and time.monotonic() < deadline:
    time.sleep(0.05)
  p.kill()
  p.wait()
  r = launch(root)
  assert r.returncode == ul.EXIT_BUSY
  direct = subprocess.run([PY, str(root / 'opt/carrot-jetlink/updater-legacy/update_host.py'), 'automatic'],
                          env=dict(os.environ, STUB_ROOT=str(root)), capture_output=True, text=True, timeout=30)
  assert direct.returncode != 0 and 'BlockingIOError' in direct.stderr
  time.sleep(7)
  r = launch(root)
  assert r.returncode == ul.EXIT_OK
  no_overlap(root)


def test_legacy_timer_running_blocks_launcher(tmp_path):
  root = make_root(tmp_path)
  timer = subprocess.Popen([PY, str(root / 'opt/carrot-jetlink/updater-legacy/update_host.py'), 'automatic'],
                           env=dict(os.environ, STUB_ROOT=str(root), STUB_LEGACY='slow', STUB_SLOW='4'))
  deadline = time.monotonic() + 20
  while ('legacy', 'start') not in roles(root) and time.monotonic() < deadline:
    time.sleep(0.05)
  r = launch(root)
  assert r.returncode == ul.EXIT_BUSY
  assert ('new', 'start') not in roles(root)
  timer.wait(timeout=30)


def test_legacy_timer_during_launcher_fails_on_lock(tmp_path):
  root = make_root(tmp_path)
  p = launch(root, env={'STUB_NEW': 'slow', 'STUB_SLOW': '3'}, background=True)
  deadline = time.monotonic() + 20
  while ('new', 'start') not in roles(root) and time.monotonic() < deadline:
    time.sleep(0.05)
  direct = subprocess.run([PY, str(root / 'opt/carrot-jetlink/updater-legacy/update_host.py'), 'automatic'],
                          env=dict(os.environ, STUB_ROOT=str(root)), capture_output=True, text=True, timeout=30)
  assert direct.returncode != 0 and 'BlockingIOError' in direct.stderr
  assert p.wait(timeout=60) == ul.EXIT_OK
  no_overlap(root)


def test_two_launchers_never_run_together(tmp_path):
  root = make_root(tmp_path)
  a = launch(root, env={'STUB_NEW': 'slow', 'STUB_SLOW': '3'}, background=True)
  deadline = time.monotonic() + 20
  while ('new', 'start') not in roles(root) and time.monotonic() < deadline:
    time.sleep(0.05)
  b = launch(root)
  assert b.returncode == ul.EXIT_BUSY
  assert a.wait(timeout=60) == ul.EXIT_OK
  assert roles(root) == [('new', 'start'), ('new', 'end')]


# ------------------------------------------------ real 0cb67d46c bytes and start code
def real_legacy_source(tmp_path):
  src = tmp_path / 'old_tools'
  src.mkdir()
  for name in NAMES:
    (src / name).write_bytes(old_file(name))
  return src


def test_installer_keeps_legacy_bytes_exact(tmp_path):
  src = real_legacy_source(tmp_path)
  root = tmp_path / 'root'
  info = install_launcher.install(root, src, HERE / 'updater_launcher.py')
  legacy = root / 'opt/carrot-jetlink/updater-legacy'
  for name in NAMES:
    want = hashlib.sha256(old_file(name)).hexdigest()
    assert info['legacy_sha256'][name] == want == hashlib.sha256((legacy / name).read_bytes()).hexdigest()
  assert ul.legacy_intact(legacy)
  # second install with the same bytes is fine, different bytes refused
  install_launcher.install(root, src, HERE / 'updater_launcher.py')
  (src / 'update_host.py').write_bytes(old_file('update_host.py') + b'#x\n')
  with pytest.raises(SystemExit):
    install_launcher.install(root, src, HERE / 'updater_launcher.py')
  # a source with the new key is refused as "legacy"
  (src / 'update_host.py').write_bytes(old_file('update_host.py'))
  (src / 'release-signing-public.pem').write_bytes((HERE / 'release-signing-public.pem').read_bytes())
  with pytest.raises(SystemExit):
    install_launcher.install(tmp_path / 'root2', src, HERE / 'updater_launcher.py')
  for service, action in install_launcher.SERVICES.items():
    text = (root / 'etc/systemd/system' / (service + '.d') / install_launcher.DROPIN).read_text()
    assert 'ExecStart=\n' in text and text.rstrip().endswith('SuccessExitStatus=10 30') and action in text


def load_old_install_updates(tmp_path, src):
  path = tmp_path / 'old_install_updates.py'
  path.write_bytes(old_file('install_updates.py'))
  sys.path.insert(0, str(src))
  try:
    spec = importlib.util.spec_from_file_location('old_install_updates', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
  finally:
    sys.path.remove(str(src))
  return module


def snapshot(root):
  out = {}
  for base in ('opt/carrot-jetlink/launcher', 'opt/carrot-jetlink/updater-legacy', 'etc/systemd/system'):
    for p in sorted((root / base).rglob('*')):
      if p.is_file() and p.parent.name.endswith('.d'):
        out[str(p.relative_to(root))] = p.read_bytes()
      elif p.is_file() and 'etc/systemd' not in base:
        out[str(p.relative_to(root))] = p.read_bytes()
  return out


def test_legacy_start_code_keeps_launcher_and_dropins(tmp_path):
  """0cb67d46c bootstrap() and configure() (unit rewrite) leave launcher, legacy
  copy and drop-ins byte-identical; bootstrap rewrites only updater/ + marker."""
  src = real_legacy_source(tmp_path)
  root = make_root(tmp_path)  # new stub in updater/, stub legacy installed
  before = snapshot(root)
  old = load_old_install_updates(tmp_path, src)
  old.bootstrap(root, src)  # what migrate_running_release(--bootstrap-only) does when marker is absent
  assert snapshot(root) == before
  slot = root / 'opt/carrot-jetlink/updater'
  assert all((slot / n).read_bytes() == old_file(n) for n in NAMES)
  assert (root / 'opt/carrot-jetlink/boot-update-required').read_text() == 'next-boot\n'
  old.configure(root, src)  # also rewrites the three unit files (no power_setup in src)
  assert snapshot(root) == before
  unit = (root / 'etc/systemd/system/carrot-jetlink-update-apply.service').read_text()
  assert 'updater/update_host.py activate' in unit  # base unit, overridden by the drop-in


def test_slot_rewritten_with_legacy_bytes_runs_legacy_once(tmp_path):
  root = make_root(tmp_path)
  legacy = root / 'opt/carrot-jetlink/updater-legacy'
  slot = root / 'opt/carrot-jetlink/updater'
  for name in ul.read_sums(legacy):
    (slot / name).write_bytes((legacy / name).read_bytes())
  r = launch(root)
  assert r.returncode == ul.EXIT_LEGACY_OK
  assert roles(root) == [('legacy', 'start'), ('legacy', 'end')]
  assert state(root)['legacy_reason'] == 'new slot holds legacy bytes'


def test_linked_marker_switch_tool_and_launcher_together(tmp_path):
  """31372d20c leaves boot-update-required as a file -> legacy migrate_running_release
  returns before bootstrap, so the new slot stays new and the launcher runs it."""
  root = make_root(tmp_path)
  rt = root / 'opt/carrot-jetlink'
  (rt / 'updater/switch-boot-marker').write_text('boot-1\n')
  (rt / 'boot-update-required').symlink_to('updater/switch-boot-marker')
  assert (rt / 'boot-update-required').is_file()  # what legacy code checks
  r = launch(root)
  assert r.returncode == ul.EXIT_OK and roles(root) == [('new', 'start'), ('new', 'end')]
