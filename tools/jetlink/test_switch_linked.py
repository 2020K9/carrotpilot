"""switch_updater_linked.py: no-window marker + update lock (fake root only).

Measured on fake roots with the real 0cb67d46c updater/runtime files:
* every kill point x legacy boot order (apply service, server
  migrate_running_release, stage timer 'automatic' 2 min later) x two reboots
* the stage timer before / during / after the switch, including a timer run
  that starts while the switch holds the lock and a switch started while an
  update action holds it
* injected I/O errors in the switch and a second one inside the recovery,
  kill points inside the recovery

crash_point() is os._exit: a process kill, NOT a power cut.
"""
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

import pytest

from test_switch_updater import MANIFEST, NEW_COMMIT, PY, assert_whole, old_root, runtime  # noqa: F401
from test_switch_review import legacy_boot, model_runs
import switch_updater as sw
import switch_updater_linked as sl

HERE = Path(__file__).resolve().parent


def run_linked(root, env=None, stamp='S1', lock_wait=5, background=False):
  e = dict(os.environ)
  e.update(env or {})
  e['PYTHONDONTWRITEBYTECODE'] = '1'
  code = ('import sys,json;sys.path.insert(0,sys.argv[1]);import switch_updater_linked as s;'
          'print(json.dumps(s.switch(sys.argv[2],sys.argv[1],sys.argv[3],sys.executable,sys.argv[4],'
          'stamp=sys.argv[5],lock_wait=float(sys.argv[6]))))')
  args = [PY, '-c', code, str(HERE), str(root), str(MANIFEST), str(root.parent / 'boot_id'), stamp, str(lock_wait)]
  if background:
    return subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=e)
  return subprocess.run(args, capture_output=True, text=True, env=e, timeout=120)


TIMER = r'''
import sys, json, os, time
from pathlib import Path as P
updater, rt, root, bid, manifest = sys.argv[1:6]
sys.path.insert(0, updater)
maps = {'/run/carrot-jetlink-update.lock': P(root) / 'run/carrot-jetlink-update.lock',
        '/dev/shm/carrot-jetlink-bootstrap.json': P(root) / 'dev/shm/carrot-jetlink-bootstrap.json',
        '/etc/carrot-jetlink-protected.json': P(root) / 'etc/carrot-jetlink-protected.json',
        '/run/carrot-storage.json': P(root) / 'run/carrot-storage.json'}
def path(x, *a):
  return maps.get(str(x), None) or P(x, *a)
(P(root) / 'dev/shm').mkdir(parents=True, exist_ok=True)
(P(root) / 'run').mkdir(parents=True, exist_ok=True)
# comma control message: parked, GitHub pin (what the comma sends today)
maps['/dev/shm/carrot-jetlink-bootstrap.json'].write_text(json.dumps(
    {'received': time.monotonic(), 'onroad': False, 'jetson_release': json.loads(P(manifest).read_text())}))
os.geteuid = lambda: 0
calls = []
import subprocess
def fake_run(c, *a, **k):
  calls.append(' '.join(map(str, c))[:80])
  raise OSError('no system commands in fake root')
subprocess.run = fake_run
subprocess.check_output = fake_run
import urllib.request
def no_net(*a, **k):
  calls.append('urlopen')
  raise OSError('no network in fake root')
urllib.request.urlopen = no_net
import update_host as u, boot_update as b
u.ROOT = P(rt); u.PROTECTED_MARKER = maps['/etc/carrot-jetlink-protected.json']; u.Path = path
b.BOOT_ID = P(bid)
sys.argv = ['update_host.py', 'automatic']
key = 'new' if 'fGh+' in (P(updater) / 'release-signing-public.pem').read_text() else 'old'
gate = b.enabled()
try:
  u.main()
  out = {'result': 'ok'}
except BaseException as e:
  out = {'result': type(e).__name__, 'error': str(e)[:160]}
out.update({'key': key, 'gate': gate, 'calls': calls})
print(json.dumps(out))
'''


def snapshot(root):
  """Every path under the runtime with content hash / link target."""
  rt = runtime(root)
  out = {}
  for dirpath, dirs, files in os.walk(rt):
    for name in dirs + files:
      p = Path(dirpath) / name
      rel = str(p.relative_to(rt))
      if p.is_symlink():
        out[rel] = 'link:' + os.readlink(p)
      elif p.is_file():
        out[rel] = hashlib.sha256(p.read_bytes()).hexdigest()[:12]
  return out


def legacy_timer(root, boot_id):
  """carrot-jetlink-update-stage.service: <installed updater>/update_host.py automatic."""
  (root.parent / 'boot_id').write_text(boot_id + '\n')
  rt = runtime(root)
  before = snapshot(root)
  r = subprocess.run([PY, '-c', TIMER, str(rt / 'updater'), str(rt), str(root), str(root.parent / 'boot_id'),
                      str(MANIFEST)], capture_output=True, text=True, timeout=60,
                     env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
  assert r.returncode == 0, r.stderr
  out = json.loads(r.stdout)
  after = snapshot(root)
  out['changed'] = sorted(k for k in set(before) | set(after) if before.get(k) != after.get(k))
  return out


def touches_switch(t):
  return [c for c in t['changed'] if c.startswith('updater') or c.startswith('boot-update-required')]


def classify(root, marker_before=None):
  """'new' = new updater + marker present; 'before' = old updater + marker exactly as before."""
  st = sw.boot_state(root)
  marker = runtime(root) / 'boot-update-required'
  if st['key'] == 'new' and st['complete'] and st['marker']:
    return 'new'
  if st['key'] == 'old' and st['complete']:
    now = marker.read_bytes() if marker.is_file() else None
    if now == marker_before:
      return 'before'
  return 'BAD ' + json.dumps(st)


def boots_with_timer(root, ids):
  res = []
  for bid in ids:
    b = legacy_boot(root, bid)
    b['timer'] = legacy_timer(root, bid)  # OnBootSec=2min, after the server started
    res.append(b)
  return res


KILL = ['after_lock', 'after_backup', 'mid_stage', 'after_stage', 'after_link', 'after_exchange',
        'after_exchange_fsync', 'after_marker_tmp', 'after_marker_rename', 'after_marker',
        'after_rename_old', 'after_verify']
BEFORE_EXCHANGE = KILL[:KILL.index('after_exchange')]


def log(*a):
  print('MEASURE', *a, flush=True)


@pytest.mark.parametrize('timer_first', [False, True], ids=['reboot', 'timer-then-reboot'])
@pytest.mark.parametrize('point', KILL)
def test_kill_point_two_legacy_boots(old_root, point, timer_first):
  r = run_linked(old_root, env={'JET_SWITCH_CRASH_AT': point})
  assert r.returncode == 91, r.stderr
  cls = classify(old_root)
  expect = 'before' if point in BEFORE_EXCHANGE else 'new'
  assert cls == expect, (point, cls)
  marker = runtime(old_root) / 'boot-update-required'
  if point == 'after_link':
    assert marker.is_symlink() and not marker.is_file()  # dangling = absent for old code
  pre = legacy_timer(old_root, 'boot-A') if timer_first else None
  if pre is not None:
    assert touches_switch(pre) == [], pre
  b1, b2 = boots_with_timer(old_root, ['boot-B', 'boot-C'])
  log(point, timer_first, cls, json.dumps({'timer_A': pre, 'B': b1, 'C': b2}))
  for t in (b1['timer'], b2['timer']):
    assert touches_switch(t) == [], t
  if expect == 'new':
    assert '--bootstrap-only' not in b1['server']['calls'] + b2['server']['calls']
    assert b1['after']['key'] == b2['after']['key'] == 'new'
    assert b1['apply'] == {'gate': True, 'selected': NEW_COMMIT} == b2['apply']
    assert model_runs(b1) and model_runs(b2)
    assert b1['timer']['result'] == 'ok' and b1['timer']['calls'] == []  # gate on: automatic returns at once
    assert_whole(old_root)
  else:
    # Same as never having run the tool (legacy behaviour, measured by test_switch_review).
    assert b1['after']['key'] == 'old'
  again = run_linked(old_root, stamp='S2')
  assert again.returncode == 0, again.stderr
  assert json.loads(again.stdout)['result'] in ('switched', 'already-new')
  assert classify(old_root, None) == 'new' and not (runtime(old_root) / 'boot-update-required').is_symlink()
  b3 = legacy_boot(old_root, 'boot-D')
  assert b3['apply'] == {'gate': True, 'selected': NEW_COMMIT}


@pytest.mark.parametrize('point', ['after_lock', 'after_stage', 'after_link', 'after_exchange', 'after_marker',
                                   'after_rename_old'])
def test_timer_during_switch_is_locked_out(old_root, point, tmp_path):
  flag = tmp_path / 'pause'
  p = run_linked(old_root, env={'JET_SWITCH_PAUSE_AT': point, 'JET_SWITCH_PAUSE_FILE': str(flag)}, background=True)
  deadline = time.monotonic() + 60
  while not flag.with_suffix('.reached').exists():
    assert time.monotonic() < deadline and p.poll() is None, p.communicate()
    time.sleep(0.05)
  t = legacy_timer(old_root, 'boot-A')
  log('during', point, json.dumps(t))
  flag.with_suffix('.go').write_text('go')
  out, err = p.communicate(timeout=120)
  assert p.returncode == 0, err
  assert t['result'] == 'BlockingIOError' and t['changed'] == [] and t['calls'] == []
  assert classify(old_root) == 'new'


def test_switch_waits_for_running_update_action_then_stops_unchanged(old_root):
  lock = old_root / sl.LOCK
  lock.parent.mkdir(parents=True, exist_ok=True)
  before = snapshot(old_root)
  with lock.open('a') as held:
    fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)  # e.g. timer 'automatic' mid-download
    t0 = time.monotonic()
    r = run_linked(old_root, lock_wait=1.5)
    waited = time.monotonic() - t0
  log('lock held by update action', r.returncode, round(waited, 2), r.stderr.strip()[-120:])
  assert r.returncode != 0 and 'nothing changed' in r.stderr and waited >= 1.5
  assert snapshot(old_root) == before
  assert run_linked(old_root).returncode == 0  # lock released -> switch completes
  assert classify(old_root) == 'new'


def test_switch_proceeds_when_lock_released_within_wait(old_root):
  lock = old_root / sl.LOCK
  lock.parent.mkdir(parents=True, exist_ok=True)
  held = lock.open('a')
  fcntl.flock(held, fcntl.LOCK_EX | fcntl.LOCK_NB)
  p = run_linked(old_root, lock_wait=30, background=True)
  time.sleep(1.0)
  assert p.poll() is None  # waiting
  held.close()
  out, err = p.communicate(timeout=120)
  assert p.returncode == 0, err
  assert json.loads(out)['result'] == 'switched'


def test_timer_before_and_after_switch(old_root):
  """Stage timer in each state with the comma's GitHub pin (no network)."""
  before = legacy_timer(old_root, 'boot-A')
  log('timer before switch', json.dumps(before))
  assert before['key'] == 'old' and before['gate'] is False
  assert touches_switch(before) == []
  assert run_linked(old_root).returncode == 0
  same = legacy_timer(old_root, 'boot-A')  # same boot: marker == this boot id -> gate off
  log('timer after switch same boot', json.dumps(same))
  assert same['key'] == 'new' and same['gate'] is False
  assert touches_switch(same) == []
  b = legacy_boot(old_root, 'boot-B')
  nxt = legacy_timer(old_root, 'boot-B')
  log('timer next boot', json.dumps(nxt))
  assert b['apply'] == {'gate': True, 'selected': NEW_COMMIT}
  assert nxt == dict(nxt, key='new', gate=True, result='ok', calls=[], changed=[])


@pytest.mark.parametrize('point', ['fsync_after_exchange', 'marker_rename', 'marker_fsync', 'rename_old', 'verify'])
@pytest.mark.parametrize('marker', [None, 'next-boot\n'])
def test_injected_error_restores_state_before(old_root, point, marker):
  rt = runtime(old_root)
  if marker is not None:
    (rt / 'boot-update-required').write_text(marker)
  mb = None if marker is None else marker.encode()
  r = run_linked(old_root, env={'JET_SWITCH_FAIL_AT': point})
  if marker is not None and point in ('marker_rename', 'marker_fsync'):
    assert r.returncode == 0, r.stderr  # existing marker kept, no marker write
    assert classify(old_root) == 'new'
    return
  assert r.returncode != 0 and 'old updater restored' in r.stderr, r.stderr
  assert classify(old_root, mb) == 'before'
  if marker is None:
    assert not (rt / 'boot-update-required').is_symlink()
    assert (rt / 'boot-update-required.rolledback-S1').is_symlink()  # moved aside, not deleted
  b1, b2 = boots_with_timer(old_root, ['boot-B', 'boot-C'])
  assert b1['after']['key'] == 'old'
  again = run_linked(old_root, stamp='S2')
  assert again.returncode == 0, again.stderr
  assert classify(old_root) == 'new'


@pytest.mark.parametrize('second', ['relink', 'exchange_back'])
@pytest.mark.parametrize('first', ['fsync_after_exchange', 'rename_old', 'verify'])
def test_second_io_error_inside_recovery_stays_safe(old_root, first, second):
  r = run_linked(old_root, env={'JET_SWITCH_FAIL_AT': first, 'JET_SWITCH_RECOVERY_FAIL_AT': second})
  assert r.returncode != 0
  cls = classify(old_root)
  log('recovery error', first, second, cls, sw.boot_state(old_root))
  assert cls in ('new', 'before'), cls
  b1, b2 = boots_with_timer(old_root, ['boot-B', 'boot-C'])
  if cls == 'new':
    assert model_runs(b1) and model_runs(b2) and b2['after']['key'] == 'new'
  again = run_linked(old_root, stamp='S2')
  assert again.returncode == 0, again.stderr
  assert classify(old_root) == 'new'


@pytest.mark.parametrize('kill', ['rec_after_relink', 'rec_after_exchange_back'])
@pytest.mark.parametrize('first', ['fsync_after_exchange', 'marker_fsync', 'verify'])
def test_kill_inside_recovery_two_legacy_boots(old_root, first, kill):
  r = run_linked(old_root, env={'JET_SWITCH_FAIL_AT': first, 'JET_SWITCH_CRASH_AT': kill})
  assert r.returncode == 91, r.stderr
  cls = classify(old_root)
  assert cls == ('new' if kill == 'rec_after_relink' else 'before'), cls
  b1, b2 = boots_with_timer(old_root, ['boot-B', 'boot-C'])
  log('recovery kill', first, kill, cls, json.dumps([b1, b2]))
  if cls == 'new':
    assert model_runs(b1) and model_runs(b2) and b2['after']['key'] == 'new'
  again = run_linked(old_root, stamp='S2')
  assert again.returncode == 0, again.stderr
  assert classify(old_root) == 'new'


def test_normal_switch_regular_marker_and_inner_file(old_root):
  r = run_linked(old_root)
  assert r.returncode == 0, r.stderr
  rt = runtime(old_root)
  m = rt / 'boot-update-required'
  assert not m.is_symlink() and m.read_text() == 'boot-A\n'
  assert (rt / 'updater' / sl.INNER).read_text() == 'boot-A\n'
  assert_whole(old_root)
  b1, b2 = boots_with_timer(old_root, ['boot-B', 'boot-C'])
  assert b1['apply'] == b2['apply'] == {'gate': True, 'selected': NEW_COMMIT}


def test_existing_marker_no_link_used(old_root):
  rt = runtime(old_root)
  (rt / 'boot-update-required').write_text('next-boot\n')
  assert run_linked(old_root).returncode == 0
  assert (rt / 'boot-update-required').read_text() == 'next-boot\n'
  assert not (rt / 'updater' / sl.INNER).exists()


def test_foreign_symlink_refused(old_root):
  rt = runtime(old_root)
  os.symlink('/elsewhere', rt / 'boot-update-required')
  r = run_linked(old_root)
  assert r.returncode != 0 and 'foreign symlink' in r.stderr
  assert sw.boot_state(old_root)['key'] == 'old'


def test_legacy_bootstrap_over_dangling_link_is_old_behaviour(old_root):
  """Kill after the link: legacy server sees no marker and reinstalls (same as before the tool)."""
  assert run_linked(old_root, env={'JET_SWITCH_CRASH_AT': 'after_link'}).returncode == 91
  b = legacy_boot(old_root, 'boot-B')
  assert b['server']['calls'][-1] == '--bootstrap-only'
  m = runtime(old_root) / 'boot-update-required'
  assert not m.is_symlink() and m.read_text() == 'boot-B\n'
