"""Legacy server reinstall racing the switch (fake root only, never /opt).

The legacy server (carrot-jetlink.service, Restart=always) runs
migrate_running_release() -> bootstrap() at every start when
boot-update-required is not a regular file (0cb67d46c server.py:10-13,
install_updates.py:26-29, 90-114).  bootstrap() takes no lock.  Here the REAL
0cb67d46c install_updates.py runs against the fake root while the switch runs:

* deterministic: the switch paused at every crash point (and the two recovery
  points), a full legacy server start in between
* deterministic: a legacy bootstrap paused after k of its file writes, resumed
  while the switch is paused at every point
* real concurrency: processes (server restarted back to back while the switch
  runs) and threads (bootstrap loop in a thread of the switch process)
* each end state followed by two reboots (legacy apply service + server + timer)

guard='off' is the switch as in 31372d20c (no server hold), guard='on' holds the
server off through a fake systemctl that models mask --runtime / stop / start on
the fake root.  The fake systemctl is a model: real systemd is NOT tested here.
crash_point() is os._exit: a process kill, NOT a power cut.
"""
import hashlib
import json
import os
from pathlib import Path
import random
import subprocess
import time

import pytest

from test_switch_updater import MANIFEST, NEW_COMMIT, OLD_COMMIT, OLD_NAMES, PY, git_show, runtime
from test_switch_review import legacy_boot, model_runs
from test_switch_linked import legacy_timer
import switch_updater as sw

HERE = Path(__file__).resolve().parent
UNIT = 'carrot-jetlink.service'
POINTS = ['after_lock', 'after_hold', 'after_backup', 'mid_stage', 'after_stage', 'after_link', 'after_exchange',
          'after_exchange_fsync', 'after_marker_tmp', 'after_marker_rename', 'after_marker', 'after_rename_old',
          'after_verify']
REC_POINTS = ['rec_after_relink', 'rec_after_exchange_back']

SYSTEMCTL = r'''
import os, sys, json, signal, time, subprocess
from pathlib import Path as P
root = P(os.environ['FAKE_ROOT'])
state = root / 'run/fake-systemd'
state.mkdir(parents=True, exist_ok=True)
mask = root / 'run/systemd/system' / sys.argv[-1]
def log(*a):
  with (state / 'log').open('a') as f:
    f.write(' '.join(a) + '\n')
cmd = sys.argv[1]
log(*sys.argv[1:])
if cmd == 'show':  # proposal 08:50: tool reads LoadState and DropInPaths (systemd order)
  prop = sys.argv[sys.argv.index('-p') + 1]
  u = sys.argv[-1]
  if prop == 'LoadState':
    f = [root / d / u for d in ('etc/systemd/system', 'run/systemd/system', 'lib/systemd/system') if os.path.lexists(root / d / u)]
    print('LoadState=' + (('masked' if f[0].is_symlink() and os.readlink(f[0]) == '/dev/null' else 'loaded') if f else 'loaded'))
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
  s = (state / 'server').read_text() if (state / 'server').exists() else 'active'
  print(s)
  sys.exit(0 if s == 'active' else 3)
if cmd == 'mask':
  mask.parent.mkdir(parents=True, exist_ok=True)
  if not os.path.lexists(mask):
    os.symlink('/dev/null', mask)
elif cmd == 'unmask':
  if mask.is_symlink():
    mask.unlink()
elif cmd == 'stop':
  # systemd stop: every process of the unit's cgroup is killed, then waited for.
  pids = (state / 'cgroup.pids').read_text().split() if (state / 'cgroup.pids').exists() else []
  for pid in map(int, pids):
    try:
      os.kill(pid, signal.SIGKILL)
    except ProcessLookupError:
      pass
  for pid in map(int, pids):
    for _ in range(400):
      try:
        st = P(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[0]
      except OSError:
        break
      if st in ('Z', 'X'):
        break
      time.sleep(0.01)
  (state / 'server').write_text('inactive')
elif cmd == 'start':
  if mask.is_symlink():
    print('Unit is masked', file=sys.stderr)
    sys.exit(1)
  (state / 'server').write_text('active')
  r = subprocess.run([sys.executable, '-c', os.environ['RACE_SERVER_CODE'], *json.loads(os.environ['RACE_SERVER_ARGS'])],
                     capture_output=True, text=True, timeout=120)
  (state / 'start_out').write_text(r.stdout + r.stderr)
'''

# One start of the legacy server's first lines with the REAL 0cb67d46c
# install_updates.py (sudo replaced by an in-process call, boot id mapped).
SERVER = r'''
import sys, json, os, time
from pathlib import Path as P
src, rt, root, bid, token = sys.argv[1:6]
state = P(root) / 'run/fake-systemd'
state.mkdir(parents=True, exist_ok=True)
if not os.environ.get('RACE_ESCAPE'):  # in the unit's cgroup (systemctl stop kills it)
  with (state / 'cgroup.pids').open('a') as f:
    f.write(f'{os.getpid()}\n')
if (P(root) / 'run/systemd/system/carrot-jetlink.service').is_symlink():
  print(json.dumps({'started': False}))
  sys.exit(0)
sys.path.insert(0, src)
import install_updates as i
pause_after = int(os.environ.get('RACE_BS_PAUSE_AFTER', '-1'))
flag = P(os.environ.get('RACE_BS_PAUSE_FILE', '/nonexistent'))
count = [0]
Base = type(P('.'))
class RP(Base):
  def replace(self, target):
    r = Base.replace(self, target)
    count[0] += 1
    if count[0] == pause_after:
      flag.with_suffix('.reached').write_text(str(count[0]))
      deadline = time.monotonic() + 120
      while not flag.with_suffix('.go').exists() and time.monotonic() < deadline:
        time.sleep(0.02)
    return r
def path(x, *a):
  return RP(bid) if str(x) == '/proc/sys/kernel/random/boot_id' else RP(x, *a)
calls = []
def run(c, **k):
  calls.append(c[-1])
  if c[-1] == '--bootstrap-only':
    i.Path = path
    try:
      i.bootstrap(P(root), src, defer_this_boot=True)
    finally:
      i.Path = P
i.subprocess.run = run
i.migrate_running_release(P(rt))
print(json.dumps({'started': True, 'calls': calls, 'writes': count[0]}))
'''


def server_args(root):
  rt = runtime(root)
  return [str(rt / 'current/tools/jetlink'), str(rt), str(root), str(root.parent / 'boot_id'), str(root),
          'install_updates.py', '--bootstrap-only']


def server_env(root, **extra):
  e = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', FAKE_ROOT=str(root))
  e.update({k: str(v) for k, v in extra.items()})
  return e


def start_server(root, background=False, **extra):
  args = [PY, '-c', SERVER, *server_args(root)]
  if background:
    return subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                            env=server_env(root, **extra))
  r = subprocess.run(args, capture_output=True, text=True, timeout=120, env=server_env(root, **extra))
  assert r.returncode == 0, r.stderr
  return json.loads(r.stdout)


def systemctl_cmd(tmp):
  shim = Path(tmp) / 'fake_systemctl.py'
  if not shim.exists():
    shim.write_text(SYSTEMCTL)
  return [PY, str(shim)]


def run_tool(root, guard, env=None, stamp='S1', background=False, lock_wait=20):
  e = server_env(root, RACE_SERVER_CODE=SERVER, RACE_SERVER_ARGS=json.dumps(server_args(root)),
                 JET_SWITCH_PROC_TOKEN=str(root))
  e.update(env or {})
  sysc = json.dumps(systemctl_cmd(root.parent)) if guard == 'on' else 'null'
  code = ''.join(['import sys,json;sys.path.insert(0,sys.argv[1]);import switch_updater_linked as s;',
                  'kw={} if sys.argv[6]=="null" else {"systemctl":json.loads(sys.argv[6])};',
                  'print(json.dumps(s.switch(sys.argv[2],sys.argv[1],sys.argv[3],sys.executable,sys.argv[4],',
                  'stamp=sys.argv[5],lock_wait=float(sys.argv[7]),**kw)))'])
  args = [PY, '-c', code, str(HERE), str(root), str(MANIFEST), str(root.parent / 'boot_id'), stamp, sysc, str(lock_wait)]
  if background:
    return subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=e)
  return subprocess.run(args, capture_output=True, text=True, env=e, timeout=180)


def wait_flag(flag, proc, timeout=60):
  deadline = time.monotonic() + timeout
  while not flag.with_suffix('.reached').exists():
    if proc.poll() is not None or time.monotonic() > deadline:
      return False
    time.sleep(0.02)
  return True


OLD = {n: hashlib.sha256(git_show(OLD_COMMIT, 'tools/jetlink/' + n)).hexdigest() for n in sw.NAMES}
NEW = {n: sw.sha(HERE / n) for n in sw.NAMES}


def state(root):
  """'new' new updater (every file = payload) + marker; 'before' old updater
  untouched + no marker (state before); 'old+legacy-marker' old updater + the
  marker the legacy bootstrap writes (what the legacy server does on its own);
  'MIXED ...' anything else."""
  rt = runtime(root)
  up = rt / 'updater'
  have = {n: sw.sha(up / n) for n in sw.NAMES if (up / n).is_file()}
  marker = (rt / 'boot-update-required').is_file()
  stray = sorted(p.name for p in up.iterdir() if p.name.endswith('.new'))
  if have == NEW and marker and not stray:
    return 'new'
  if have == OLD and not stray:
    return 'old+legacy-marker' if marker else 'before'
  bad = sorted(n for n in sw.NAMES if have.get(n) not in (NEW.get(n),))
  return 'MIXED old-in-new=' + ','.join(n for n in bad if have.get(n) == OLD.get(n)) + \
         (' stray=' + ','.join(stray) if stray else '') + (' marker' if marker else ' no-marker')


def reboot(root, n):
  """Reboot: /run is a tmpfs (moved aside here, never deleted), then the legacy
  boot order (apply service, server, stage timer)."""
  run_dir = root / 'run'
  if run_dir.exists():
    run_dir.rename(root.parent / f'run-before-reboot-{n}-{time.time_ns()}')
  b = legacy_boot(root, f'boot-R{n}')
  b['timer'] = legacy_timer(root, f'boot-R{n}')
  return {'state': state(root), 'model': 'runs' if model_runs(b) else 'STUCK',
          'gate': b['apply'].get('selected') or b['apply'].get('error', '')[:40] if b['apply']['gate'] else 'off',
          'reinstall': b['server']['calls'][-1:] == ['--bootstrap-only']}


def outcome(root, tool):
  out, err = tool if isinstance(tool, tuple) else (tool.stdout, tool.stderr)
  try:
    result = json.loads(out)['result'] if out.strip() else 'killed/failed'
  except ValueError:
    result = 'killed/failed'
  return {'tool': result, 'err': err.strip().splitlines()[-1][:120] if err.strip() else '',
          'after': state(root), 'r1': reboot(root, 1), 'r2': reboot(root, 2)}


def log(*a):
  print('MEASURE', *a, flush=True)


def good(o):
  """Accepted end states: 'new' kept over both reboots, or the switch did not
  happen and both reboots look exactly like an untouched legacy Jetson."""
  if o['after'] == 'new':
    return all(o[r]['state'] == 'new' and o[r]['model'] == 'runs' and o[r]['gate'] == NEW_COMMIT
               and not o[r]['reinstall'] for r in ('r1', 'r2'))
  if o['after'] in ('before', 'old+legacy-marker'):
    return o['r1']['state'] == 'old+legacy-marker' and o['r2']['state'] == 'old+legacy-marker'
  return False


# ---- deterministic: server start while the switch is paused ----------------

def server_during_pause(root, guard, point, extra_env=None):
  flag = root.parent / 'pause'
  env = {'JET_SWITCH_PAUSE_AT': point, 'JET_SWITCH_PAUSE_FILE': str(flag)}
  env.update(extra_env or {})
  p = run_tool(root, guard, env=env, background=True)
  reached = wait_flag(flag, p)
  s = start_server(root) if reached else None
  flag.with_suffix('.go').write_text('go')
  out, err = p.communicate(timeout=180)
  o = outcome(root, (out, err))
  o.update(reached=reached, server=s)
  return o


def all_points(guard):
  return [p for p in POINTS if guard == 'on' or p != 'after_hold']


@pytest.mark.parametrize('point', all_points('off'))
def test_unguarded_server_start_during_switch_measured(old_root, point):
  o = server_during_pause(old_root, 'off', point)
  log('off pause', point, json.dumps(o))
  assert o['reached'] and o['server']['started']
  # Recorded, not required: the unguarded switch (31372d20c) depends on timing.
  assert o['after'] in ('new', 'before', 'old+legacy-marker') or o['after'].startswith('MIXED')


@pytest.mark.parametrize('point', all_points('on'))
def test_guarded_server_cannot_start_during_switch(old_root, point):
  o = server_during_pause(old_root, 'on', point)
  log('on pause', point, json.dumps(o))
  if point == 'after_lock':  # before the hold: the start runs to the end, then the hold stops the unit
    assert o['reached'] and o['server']['started']
  else:
    assert o['reached'] and o['server'] == {'started': False}
  assert o['tool'] == 'switched' and good(o), o


def test_guarded_switch_systemctl_order_and_restart(old_root):
  r = run_tool(old_root, 'on')
  assert r.returncode == 0, r.stderr
  calls = (old_root / 'run/fake-systemd/log').read_text().split('\n')
  log('systemctl calls', json.dumps(calls))
  assert calls[:6] == ['is-active ' + UNIT, 'show -p LoadState ' + UNIT, 'mask --runtime ' + UNIT,
                       'show -p DropInPaths ' + UNIT, 'stop ' + UNIT, 'is-active ' + UNIT]
  assert calls[6:8] == ['unmask --runtime ' + UNIT, 'start --no-block ' + UNIT]
  started = (old_root / 'run/fake-systemd/start_out').read_text()
  assert json.loads(started) == {'started': True, 'calls': ['--performance-runtime'], 'writes': 0}
  assert state(old_root) == 'new'


@pytest.mark.parametrize('point', REC_POINTS)
@pytest.mark.parametrize('guard', ['off', 'on'])
def test_server_start_during_recovery(old_root, guard, point):
  o = server_during_pause(old_root, guard, point, {'JET_SWITCH_FAIL_AT': 'verify'})
  log(guard, 'recovery pause', point, json.dumps(o))
  assert o['reached']
  if guard == 'on':
    assert o['server'] == {'started': False} and o['tool'] == 'killed/failed' and good(o), o
    # 'old+legacy-marker' = the restarted legacy server reinstalled, as without the tool
    assert o['after'] in ('before', 'old+legacy-marker')


@pytest.mark.parametrize('point', ['after_hold', 'after_stage', 'after_link', 'after_exchange', 'after_marker'])
def test_guarded_kill_leaves_server_masked_until_reboot(old_root, point):
  r = run_tool(old_root, 'on', env={'JET_SWITCH_CRASH_AT': point})
  assert r.returncode == 91
  assert (old_root / 'run/systemd/system' / UNIT).is_symlink()  # runtime mask stays: server down
  assert start_server(old_root) == {'started': False}
  o = outcome(old_root, ('', 'killed'))
  log('on kill', point, json.dumps(o))
  assert good(o), o


# ---- deterministic: bootstrap already writing when the switch runs ----------

K = [1, 4, 8]  # replaces done before the pause (8 = all updater files, marker not yet)


@pytest.mark.parametrize('k', K)
@pytest.mark.parametrize('point', all_points('off'))
def test_unguarded_bootstrap_in_flight_measured(old_root, k, point):
  bflag = old_root.parent / 'bs'
  s = start_server(old_root, background=True, RACE_BS_PAUSE_AFTER=k, RACE_BS_PAUSE_FILE=bflag)
  assert wait_flag(bflag, s)
  flag = old_root.parent / 'pause'
  p = run_tool(old_root, 'off', env={'JET_SWITCH_PAUSE_AT': point, 'JET_SWITCH_PAUSE_FILE': str(flag)},
               background=True)
  reached = wait_flag(flag, p)
  bflag.with_suffix('.go').write_text('go')
  s.communicate(timeout=60)
  flag.with_suffix('.go').write_text('go')
  out, err = p.communicate(timeout=180)
  o = outcome(old_root, (out, err))
  o['reached'] = reached
  log('off inflight', k, point, json.dumps(o))
  assert o['after'] in ('new', 'before', 'old+legacy-marker') or o['after'].startswith('MIXED')


@pytest.mark.parametrize('k', K)
@pytest.mark.parametrize('escape', [False, True], ids=['in-cgroup', 'escaped-sudo'])
def test_guarded_bootstrap_in_flight(old_root, k, escape):
  """In the unit's cgroup: stop kills it mid-write.  Escaped (e.g. sudo moved it
  to a session): the switch waits until it ends, then switches."""
  bflag = old_root.parent / 'bs'
  extra = {'RACE_BS_PAUSE_AFTER': k, 'RACE_BS_PAUSE_FILE': bflag}
  if escape:
    extra['RACE_ESCAPE'] = '1'
  s = start_server(old_root, background=True, **extra)
  assert wait_flag(bflag, s)
  p = run_tool(old_root, 'on', background=True)
  if escape:
    time.sleep(1.0)
    assert p.poll() is None  # waiting for the bootstrap
    bflag.with_suffix('.go').write_text('go')
  out, err = p.communicate(timeout=180)
  s.communicate(timeout=60)
  o = outcome(old_root, (out, err))
  o['server_rc'] = s.returncode
  log('on inflight', k, 'escaped' if escape else 'cgroup', json.dumps(o))
  assert o['tool'] == 'switched' and good(o), o
  if not escape:
    assert s.returncode == -9


def test_guarded_escaped_bootstrap_never_ending_stops_unchanged(old_root):
  bflag = old_root.parent / 'bs'
  s = start_server(old_root, background=True, RACE_BS_PAUSE_AFTER=3, RACE_BS_PAUSE_FILE=bflag, RACE_ESCAPE=1)
  assert wait_flag(bflag, s)
  r = run_tool(old_root, 'on', lock_wait=1.5)
  bflag.with_suffix('.go').write_text('go')
  s.communicate(timeout=60)
  log('on escaped never ends', r.returncode, r.stderr.strip()[-160:])
  assert r.returncode != 0 and 'still running; nothing changed' in r.stderr
  assert state(old_root) in ('before', 'old+legacy-marker')
  assert not (old_root / 'run/systemd/system' / UNIT).is_symlink()  # unmasked again
  assert not list(runtime(old_root).glob('updater.switch-*')) and not (runtime(old_root) / 'updater-switch-backup').exists()


# ---- real concurrency --------------------------------------------------------

N_PROC = 30


@pytest.mark.parametrize('guard', ['off', 'on'])
def test_processes_server_restarting_during_switch(tmp_path, guard):
  counts, bad, starts = {}, [], 0
  rnd = random.Random(7)
  for i in range(N_PROC):
    base = tmp_path / f'r{i}'
    base.mkdir()
    root = fixture_root(base)
    p = run_tool(root, guard, background=True)
    servers = []
    time.sleep(rnd.uniform(0, 0.3))
    while p.poll() is None:  # Restart=always: a new start as soon as one ends
      s = start_server(root, background=True)
      servers.append(s)
      s.communicate(timeout=120)
      starts += 1
    out, err = p.communicate()
    o = outcome(root, (out, err))
    key = o['after'] + ' / ' + o['tool']
    counts[key] = counts.get(key, 0) + 1
    if not good(o):
      bad.append(o)
  log(guard, 'processes', N_PROC, 'server starts', starts, json.dumps(counts), 'not good', len(bad),
      json.dumps(bad[:3]))
  if guard == 'on':
    assert bad == []


def fixture_root(base):
  root = base / 'root'
  rt = root / 'opt/carrot-jetlink'
  up = rt / 'updater'
  up.mkdir(parents=True)
  for name in OLD_NAMES[:-1]:
    (up / name).write_bytes(git_show(OLD_COMMIT, 'tools/jetlink/' + name))
  (up / 'extra_old_only.txt').write_text('kept\n')
  src = rt / 'releases/old/tools/jetlink'
  src.mkdir(parents=True)
  for name in OLD_NAMES:
    (src / name).write_bytes(git_show(OLD_COMMIT, 'tools/jetlink/' + name))
  (rt / 'releases/old/SOURCE_COMMIT').write_text('0' * 40 + '\n')
  (rt / 'current').symlink_to(rt / 'releases/old')
  units = root / 'etc/systemd/system'
  units.mkdir(parents=True)
  cmd = '/opt/carrot-jetlink/venv/bin/python /opt/carrot-jetlink/updater/update_host.py'
  (units / 'carrot-jetlink-update-apply.service').write_text(f'[Service]\nExecStart={cmd} activate\n')
  (units / 'carrot-jetlink-update-stage.service').write_text(f'[Service]\nExecStart={cmd} automatic\n')
  (units / 'carrot-jetlink-update-stage.timer').write_text('[Timer]\nOnBootSec=2min\n')
  (base / 'boot_id').write_text('boot-A\n')
  return root


@pytest.fixture
def old_root(tmp_path):
  """Same legacy fixture as test_switch_updater.old_root."""
  return fixture_root(tmp_path)


THREAD = r'''
import sys, json, threading, time
from pathlib import Path as P
here, src, root, manifest, bid = sys.argv[1:6]
sys.path.insert(0, here)
import switch_updater_linked as s
sys.path.insert(0, src)
import install_updates as i
Base = type(P('.'))
def path(x, *a):
  return P(bid) if str(x) == '/proc/sys/kernel/random/boot_id' else P(x, *a)
stop = threading.Event()
runs = [0]
def legacy():
  # migrate_running_release() condition + bootstrap() in a loop, same process
  while not stop.is_set():
    if not (P(root) / 'opt/carrot-jetlink/boot-update-required').is_file():
      i.Path = path
      try:
        i.bootstrap(P(root), src, defer_this_boot=True)
        runs[0] += 1
      except OSError:
        pass
      finally:
        i.Path = P
    time.sleep(0.0005)
t = threading.Thread(target=legacy)
t.start()
try:
  r = s.switch(root, here, manifest, sys.executable, bid, stamp='T1', lock_wait=5)
  res = r['result']
except Exception as e:
  res = 'error: ' + str(e)[:100]
time.sleep(0.05)
stop.set()
t.join()
print(json.dumps({'result': res, 'bootstraps': runs[0]}))
'''

N_THREAD = 30


def test_threads_bootstrap_loop_inside_switch_process_unguarded(tmp_path):
  """Threads cannot be stopped by systemd: this measures the unguarded race only."""
  counts = {}
  for i in range(N_THREAD):
    base = tmp_path / f't{i}'
    base.mkdir()
    root = fixture_root(base)
    rt = runtime(root)
    r = subprocess.run([PY, '-c', THREAD, str(HERE), str(rt / 'current/tools/jetlink'), str(root), str(MANIFEST),
                        str(base / 'boot_id')], capture_output=True, text=True, timeout=180,
                       env=dict(os.environ, PYTHONDONTWRITEBYTECODE='1'))
    assert r.returncode == 0, r.stderr
    res = json.loads(r.stdout)
    key = state(root) + ' / ' + res['result'][:40]
    counts[key] = counts.get(key, 0) + 1
  log('off threads', N_THREAD, json.dumps(counts))
  assert sum(counts.values()) == N_THREAD


# ---- staging keeps sub-folders and links ------------------------------------

def test_stage_keeps_subfolders_links_and_modes(old_root):
  up = runtime(old_root) / 'updater'
  sub = up / 'before-usb-experiments'
  sub.mkdir(mode=0o700)
  (sub / 'update_host.py').write_bytes(b'# backup copy\n')
  os.symlink('update_host.py', sub / 'link.py')
  (up / 'usb-sha256-policy.json').write_text('{"format": 1}\n')
  (up / 'usb-sha256-policy.json').chmod(0o600)
  os.symlink('extra_old_only.txt', up / 'extra_link.txt')
  (up / '__pycache__').mkdir()
  (up / '__pycache__/x.pyc').write_bytes(b'x')
  r = run_tool(old_root, 'on')
  assert r.returncode == 0, r.stderr
  assert (up / 'before-usb-experiments').is_dir() and (up / 'before-usb-experiments').stat().st_mode & 0o777 == 0o700
  assert (up / 'before-usb-experiments/update_host.py').read_bytes() == b'# backup copy\n'
  assert os.readlink(up / 'before-usb-experiments/link.py') == 'update_host.py'
  assert os.readlink(up / 'extra_link.txt') == 'extra_old_only.txt'
  assert (up / 'usb-sha256-policy.json').stat().st_mode & 0o777 == 0o600
  assert not (up / '__pycache__/x.pyc').exists()  # cache not copied (imports may rebuild the folder)
  assert state(old_root) == 'new'


# Copy order of the legacy bootstrap (install_updates.py at OLD_COMMIT); pinned
# against that source in test_legacy_bootstrap_order_pinned.
LEGACY_ORDER = ['finalize_sd_image.py', 'hud_protocol.py', 'release-signing-public.pem',
                'wifi_protocol.py', 'local_release.py', 'boot_update.py', 'update_host.py']


def test_legacy_bootstrap_order_pinned():
  src = git_show(OLD_COMMIT, 'tools/jetlink/install_updates.py').decode()
  line = src.split("  names = [", 1)[1].split("]", 1)[0]
  assert [x.strip().strip("'") for x in line.split(',')] == LEGACY_ORDER


def race_expect(k):
  """Exact state after the unguarded race: the legacy bootstrap paused after
  writing k files, the switch then installed the new payload, the bootstrap
  resumed and wrote LEGACY_ORDER[k:] from the old source over it. Files whose
  old and new copies differ end up old inside the new updater."""
  late = [n for n in LEGACY_ORDER[k:] if OLD[n] != NEW[n]]
  if not late:
    return 'new'
  if all(OLD[n] == NEW[n] for n in sw.NAMES if n not in LEGACY_ORDER[k:]):
    return 'old+legacy-marker'
  return 'MIXED old-in-new=' + ','.join(sorted(late)) + ' marker'


def test_race_expect_matches_payload():
  # 2026-11 carrot-wip changed finalize_sd_image.py, hud_protocol.py and the
  # signing key, so with k=1 three files land old in the new updater.
  assert race_expect(1) == 'MIXED old-in-new=hud_protocol.py,release-signing-public.pem,update_host.py marker'
  assert race_expect(4) == 'MIXED old-in-new=update_host.py marker'


@pytest.mark.parametrize('k', [1, 4])
def test_race_reproduced_unguarded_switch_reports_switched_but_stuck(old_root, k):
  expect = race_expect(k)
  assert expect != 'new'  # the race must leave old files behind
  """31372d20c behaviour: a legacy bootstrap that started before the switch and
  finishes after its last check writes old files into the new updater/."""
  bflag = old_root.parent / 'bs'
  s = start_server(old_root, background=True, RACE_BS_PAUSE_AFTER=k, RACE_BS_PAUSE_FILE=bflag)
  assert wait_flag(bflag, s)
  flag = old_root.parent / 'pause'
  p = run_tool(old_root, 'off', env={'JET_SWITCH_PAUSE_AT': 'after_verify', 'JET_SWITCH_PAUSE_FILE': str(flag)},
               background=True)
  assert wait_flag(flag, p)
  bflag.with_suffix('.go').write_text('go')
  s.communicate(timeout=60)
  flag.with_suffix('.go').write_text('go')
  o = outcome(old_root, p.communicate(timeout=180))
  log('race reproduced', k, json.dumps(o))
  assert o['tool'] == 'switched' and o['after'] == expect
  assert o['r1']['model'] == 'STUCK' and o['r2']['model'] == 'STUCK'


def test_stage_drops_killed_bootstrap_temp_files_keeps_backup(old_root):
  up = runtime(old_root) / 'updater'
  (up / 'update_host.py.new').write_bytes(b'# half written\n')
  (up / 'other.new').write_bytes(b'not a bootstrap temp\n')
  r = run_tool(old_root, 'on')
  assert r.returncode == 0, r.stderr
  assert not (up / 'update_host.py.new').exists() and (up / 'other.new').read_bytes() == b'not a bootstrap temp\n'
  backup = next((runtime(old_root) / 'updater-switch-backup').iterdir())
  assert (backup / 'updater/update_host.py.new').read_bytes() == b'# half written\n'
