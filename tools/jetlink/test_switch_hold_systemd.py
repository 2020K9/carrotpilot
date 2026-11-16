"""Server hold with the unit in /etc (review 11/16 07:49), fake root only.

The legacy install_server.sh installs carrot-jetlink.service in
/etc/systemd/system.  Measured with real systemd 252 (user manager, same
lookup order: ~/.config = /etc role before $XDG_RUNTIME_DIR = /run role;
jet_switch_review_out/hold_0750/measure_a.log, measure_b.log):

* mask --runtime with the unit in /etc: rc=0, /run link made, unit still
  loaded (UnitFileState=enabled-runtime); Restart=always and start still run it.
* runtime drop-in ConditionPathExists=!/dev/null + reload: start rc=0 but
  inactive (ConditionResult=no), Restart=always does not restart (NRestarts=0).

The fake systemctl here models that order: unit file /etc > /run > /lib (the
first found wins; /dev/null = masked), drop-ins of every dir are merged, and
only what was loaded at the last reload (daemon-reload, mask, unmask) counts.
So a /run mask alone does NOT hold a unit in /etc.  This is still a model:
real systemd on the Jetson is NOT tested here.
"""
import json
import os
from pathlib import Path
import subprocess

import pytest

from test_switch_updater import MANIFEST, PY
from test_switch_race import POINTS, fixture_root
import switch_updater_linked as sl

HERE = Path(__file__).resolve().parent
UNIT = 'carrot-jetlink.service'

SYSTEMCTL2 = r'''
import os, sys, json
from pathlib import Path as P
root = P(os.environ['FAKE_ROOT'])
st = root / 'run/fake-systemd2'
st.mkdir(parents=True, exist_ok=True)
args = sys.argv[1:]
with (st / 'log').open('a') as f:
  f.write(' '.join(args) + '\n')
cmd = args[0]
unit = 'carrot-jetlink.service'  # the only unit modelled (daemon-reload has no unit arg)
DIRS = ['etc/systemd/system', 'run/systemd/system', 'lib/systemd/system']
if cmd in os.environ.get('FAKE_FAIL', '').split(','):
  print('Failed (fake)', file=sys.stderr)
  sys.exit(1)
def scan():
  frag = None
  for d in DIRS:
    f = root / d / unit
    if os.path.lexists(f):
      frag = 'masked' if f.is_symlink() and os.readlink(f) == '/dev/null' else d
      break
  cond = False
  for d in DIRS:
    dd = root / d / (unit + '.d')
    if dd.is_dir():
      for c in sorted(dd.glob('*.conf')):
        if 'ConditionPathExists=!/dev/null' in c.read_text():
          cond = True
  return {'fragment': frag, 'condition_false': cond}
def loaded():
  p = st / 'loaded.json'
  return json.loads(p.read_text()) if p.exists() else scan()
def reload():
  (st / 'loaded.json').write_text(json.dumps(scan()))
def server(v=None):
  p = st / 'server'
  if v is not None:
    p.write_text(v)
  return p.read_text() if p.exists() else os.environ.get('FAKE_INITIAL', 'active')
if cmd == 'show':  # proposal 08:50: tool reads LoadState and DropInPaths (systemd order)
  prop = sys.argv[sys.argv.index('-p') + 1]
  u = sys.argv[-1]
  if prop == 'LoadState':
    f = [root / d / u for d in ('etc/systemd/system', 'run/systemd/system', 'lib/systemd/system') if os.path.lexists(root / d / u)]
    print('LoadState=' + (('masked' if f[0].is_symlink() and os.readlink(f[0]) == '/dev/null' else 'loaded') if f else 'not-found'))
  elif prop == 'DropInPaths':
    seen = {}
    for d in ('etc/systemd/system', 'run/systemd/system', 'lib/systemd/system'):
      dd = root / d / (u + '.d')
      if dd.is_dir():
        for c in dd.iterdir():
          if c.name.endswith('.conf') and c.name not in seen:
            seen[c.name] = str(c)
    print('DropInPaths=' + ' '.join(seen[n] for n in sorted(seen)))
  else:
    print('ActiveState=x')
  sys.exit(0)
if cmd == 'is-active':
  s = server()
  print(s)
  sys.exit(0 if s == 'active' else 3)
if cmd == 'daemon-reload':
  reload()
elif cmd == 'mask':
  m = root / 'run/systemd/system' / unit
  m.parent.mkdir(parents=True, exist_ok=True)
  if os.path.lexists(root / 'etc/systemd/system' / unit) and os.environ.get('FAKE_MASK_REFUSE'):
    print('Failed to mask unit: File exists (fake)', file=sys.stderr)
    sys.exit(1)
  if not os.path.lexists(m):
    os.symlink('/dev/null', m)
  reload()
elif cmd == 'unmask':
  m = root / 'run/systemd/system' / unit
  if m.is_symlink():
    m.unlink()
  reload()
elif cmd == 'stop':
  server('inactive')
elif cmd == 'start':
  cfg = loaded()
  if cfg['fragment'] in (None, 'masked'):
    print('Unit is masked' if cfg['fragment'] else 'Unit not found', file=sys.stderr)
    sys.exit(1 if cfg['fragment'] else 5)
  if cfg['condition_false']:
    sys.exit(0)  # systemd: start rc 0, condition failed, unit stays inactive
  with (st / 'starts').open('a') as f:
    f.write('1\n')
  server('failed' if os.environ.get('FAKE_START_DIES') else 'active')
'''


def fake(tmp):
  shim = Path(tmp) / 'fake_systemctl2.py'
  if not shim.exists():
    shim.write_text(SYSTEMCTL2)
  return [PY, str(shim)]


def ctl(root, *args, **env):
  e = dict(os.environ, FAKE_ROOT=str(root), **env)
  return subprocess.run([*fake(root.parent), *args], capture_output=True, text=True, env=e, timeout=60)


def starts(root):
  p = root / 'run/fake-systemd2/starts'
  return len(p.read_text().split()) if p.exists() else 0


def active(root):
  return ctl(root, 'is-active', UNIT).stdout.strip()


@pytest.fixture
def etc_root(tmp_path):
  """Legacy fixture + carrot-jetlink.service in /etc (install_server.sh, 0cb67d46c)."""
  root = fixture_root(tmp_path)
  (root / 'etc/systemd/system' / UNIT).write_text('[Service]\nExecStart=/bin/true\nRestart=always\nRestartSec=3\n')
  assert ctl(root, 'daemon-reload').returncode == 0  # boot: what systemd loaded
  return root


def cli(root, *extra, env=None):
  e = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', FAKE_ROOT=str(root), JET_SWITCH_PROC_TOKEN=str(root))
  e.update(env or {})
  args = [PY, str(HERE / 'switch_updater_linked.py'), '--root', str(root), '--manifest', str(MANIFEST),
          '--python', PY, '--boot-id', str(root.parent / 'boot_id'), '--lock-wait', '5',
          '--systemctl', ' '.join(fake(root.parent)), *extra]
  return subprocess.run(args, capture_output=True, text=True, env=e, timeout=180)


def test_model_runtime_mask_alone_does_not_hold_etc_unit(etc_root):
  """Same as measure_a.log: mask --runtime rc 0, start still runs the /etc unit."""
  assert ctl(etc_root, 'stop', UNIT).returncode == 0
  assert ctl(etc_root, 'mask', '--runtime', UNIT).returncode == 0
  assert (etc_root / 'run/systemd/system' / UNIT).is_symlink()
  assert ctl(etc_root, 'start', UNIT).returncode == 0
  assert active(etc_root) == 'active' and starts(etc_root) == 1


def test_model_dropin_needs_reload(etc_root):
  d = etc_root / 'run/systemd/system' / (UNIT + '.d')
  d.mkdir(parents=True)
  (d / sl.HOLD_DROPIN).write_text(sl.HOLD_TEXT)
  ctl(etc_root, 'stop', UNIT)
  assert ctl(etc_root, 'start', UNIT).returncode == 0 and active(etc_root) == 'active'  # not reloaded yet
  ctl(etc_root, 'stop', UNIT)
  ctl(etc_root, 'daemon-reload')
  assert ctl(etc_root, 'start', UNIT).returncode == 0 and active(etc_root) == 'inactive'


def hold(root, **env):
  old = dict(os.environ)
  os.environ.update(FAKE_ROOT=str(root), **env)
  try:
    return sl.hold_server(fake(root.parent), '/proc', 2, token=str(root), root=root)
  finally:
    os.environ.clear()
    os.environ.update(old)


def release(root, h, **env):
  old = dict(os.environ)
  os.environ.update(FAKE_ROOT=str(root), **env)
  try:
    return sl.release_server(fake(root.parent), h)
  finally:
    os.environ.clear()
    os.environ.update(old)


def test_hold_blocks_etc_unit_and_release_restores(etc_root):
  h = hold(etc_root)
  assert h['was_active'] and h['made_dropin'] and h['made_mask']
  assert active(etc_root) == 'inactive'
  assert ctl(etc_root, 'start', UNIT).returncode == 0 and active(etc_root) == 'inactive' and starts(etc_root) == 0
  release(etc_root, h)
  assert active(etc_root) == 'active' and starts(etc_root) == 1
  assert not (etc_root / 'run/systemd/system' / UNIT).exists()
  assert not (etc_root / 'run/systemd/system' / (UNIT + '.d')).exists()


@pytest.mark.parametrize('point', POINTS)
def test_switch_paused_server_start_blocked_with_etc_unit(etc_root, point):
  flag = etc_root.parent / 'pause'
  env = {'JET_SWITCH_PAUSE_AT': point, 'JET_SWITCH_PAUSE_FILE': str(flag)}
  e = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', FAKE_ROOT=str(etc_root), JET_SWITCH_PROC_TOKEN=str(etc_root), **env)
  args = [PY, str(HERE / 'switch_updater_linked.py'), '--root', str(etc_root), '--manifest', str(MANIFEST),
          '--python', PY, '--boot-id', str(etc_root.parent / 'boot_id'), '--lock-wait', '5',
          '--systemctl', ' '.join(fake(etc_root.parent))]
  p = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=e)
  import time
  deadline = time.monotonic() + 60
  while not flag.with_suffix('.reached').exists() and p.poll() is None and time.monotonic() < deadline:
    time.sleep(0.02)
  reached = flag.with_suffix('.reached').exists()
  r = ctl(etc_root, 'start', UNIT)
  during = active(etc_root)
  n_during = starts(etc_root)
  flag.with_suffix('.go').write_text('go')
  out, err = p.communicate(timeout=120)
  assert reached, err
  if point == 'after_lock':  # before the hold: the server is still the old one, nothing written yet
    assert during == 'active'
  else:
    assert r.returncode == 0 and during == 'inactive' and n_during == 0, (point, during)
  assert p.returncode == 0, err
  assert json.loads(out)['server_restore'] == 'ok' and active(etc_root) == 'active'


def test_preexisting_runtime_mask_kept(etc_root):
  m = etc_root / 'run/systemd/system' / UNIT
  m.parent.mkdir(parents=True)
  m.symlink_to('/dev/null')
  ctl(etc_root, 'stop', UNIT)
  h = hold(etc_root)
  assert h['before']['runtime_masked'] and not h['made_mask'] and not h['was_active']
  release(etc_root, h)
  log = (etc_root / 'run/fake-systemd2/log').read_text().split('\n')
  assert not any(c.startswith('unmask') for c in log) and not any(c.startswith('start') for c in log)
  assert m.is_symlink() and active(etc_root) == 'inactive'
  assert not (etc_root / 'run/systemd/system' / (UNIT + '.d')).exists()


def test_preexisting_etc_mask_recorded_and_kept(etc_root):
  f = etc_root / 'etc/systemd/system' / UNIT
  f.unlink()
  f.symlink_to('/dev/null')
  ctl(etc_root, 'stop', UNIT)
  with pytest.raises(sl.HoldAborted) as e:  # 08:50: LoadState masked -> stop, nothing done
    hold(etc_root)
  assert e.value.hold['restore'] == 'nothing done' and 'masked' in str(e.value)
  assert f.is_symlink() and os.readlink(f) == '/dev/null' and active(etc_root) == 'inactive'


def test_preexisting_dropin_and_other_dropins_kept(etc_root):
  d = etc_root / 'run/systemd/system' / (UNIT + '.d')
  d.mkdir(parents=True)
  (d / sl.HOLD_DROPIN).write_text(sl.HOLD_TEXT)  # left by an earlier killed switch
  (d / '10-other.conf').write_text('[Service]\nNice=5\n')
  ctl(etc_root, 'daemon-reload')
  ctl(etc_root, 'stop', UNIT)
  h = hold(etc_root)
  assert h['before']['dropin_present'] and not h['made_dropin']
  release(etc_root, h)
  assert (d / sl.HOLD_DROPIN).exists() and (d / '10-other.conf').exists()


def test_other_dropin_kept_own_removed(etc_root):
  d = etc_root / 'run/systemd/system' / (UNIT + '.d')
  d.mkdir(parents=True)
  (d / '10-other.conf').write_text('[Service]\nNice=5\n')
  h = hold(etc_root)
  release(etc_root, h)
  assert sorted(p.name for p in d.iterdir()) == ['10-other.conf'] and active(etc_root) == 'active'


def test_inactive_server_not_started(etc_root):
  ctl(etc_root, 'stop', UNIT)
  h = hold(etc_root)
  release(etc_root, h)
  assert active(etc_root) == 'inactive' and starts(etc_root) == 0
  assert not (etc_root / 'run/systemd/system' / UNIT).exists()


@pytest.mark.parametrize('env,why', [({'FAKE_FAIL': 'unmask'}, 'unmask'),
                                     ({'FAKE_FAIL': 'start'}, 'start'),
                                     ({'FAKE_START_DIES': '1'}, 'is failed after start')])
def test_release_failure_raises(etc_root, env, why):
  h = hold(etc_root)
  with pytest.raises(sl.ServerRestoreError) as e:
    release(etc_root, h, **env)
  assert why in str(e.value) and h['restore_failed']


@pytest.mark.parametrize('env', [{'FAKE_FAIL': 'unmask'}, {'FAKE_FAIL': 'start'}, {'FAKE_START_DIES': '1'}])
def test_cli_switched_but_restore_failed_exit_3(etc_root, env):
  # FAKE_* hit only unmask/start/is-active after start: the hold (mask, stop) is not affected
  e = env
  r = cli(etc_root, env=e)
  assert r.returncode == 3, r.stderr
  assert 'SWITCHED BUT SERVER RESTORE FAILED (not success)' in r.stderr
  assert json.loads(r.stdout)['server_restore'] == 'FAILED'
  assert json.loads(r.stderr.strip().splitlines()[-1])['server_hold']['restore_failed']


def test_cli_ok_restores(etc_root):
  r = cli(etc_root)
  assert r.returncode == 0, r.stderr
  assert json.loads(r.stdout)['server_restore'] == 'ok' and active(etc_root) == 'active'


def test_cli_switch_failed_and_restore_failed_exit_4(etc_root):
  r = cli(etc_root, env={'FAKE_FAIL': 'unmask', 'JET_SWITCH_FAIL_AT': 'verify'})
  assert r.returncode == 4, r.stderr
  assert 'SERVER RESTORE FAILED TOO' in r.stderr


def test_hold_mask_refused_undoes_dropin(etc_root):
  with pytest.raises(sl.SwitchError):
    hold(etc_root, FAKE_MASK_REFUSE='1')
  assert not (etc_root / 'run/systemd/system' / (UNIT + '.d')).exists()
  assert active(etc_root) == 'active'
