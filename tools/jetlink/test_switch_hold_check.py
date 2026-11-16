"""Hold check without start + is-active exit code (review 11/16 08:17/08:18), fake root.

The fake systemctl here also answers "show -p DropInPaths" in systemd's order
(same name: /etc > /run > /lib wins, then sorted by name) and applies the
[Unit] ConditionPathExists= list with an empty value resetting it, as
measured with real systemd 252 (jet_switch_review_out/hold2_0820/measure.log).
It is still a model: real systemd on the Jetson is NOT tested here.
"""
import json
import os
from pathlib import Path
import subprocess

import pytest

from test_switch_updater import MANIFEST, PY
from test_switch_race import fixture_root
import switch_updater_linked as sl

HERE = Path(__file__).resolve().parent
UNIT = 'carrot-jetlink.service'

SYSTEMCTL3 = r'''
import os, sys, json
from pathlib import Path as P
root = P(os.environ['FAKE_ROOT'])
st = root / 'run/fake-systemd3'
st.mkdir(parents=True, exist_ok=True)
args = sys.argv[1:]
with (st / 'log').open('a') as f:
  f.write(' '.join(args) + '\n')
cmd = args[0]
unit = 'carrot-jetlink.service'
DIRS = ['etc/systemd/system', 'run/systemd/system', 'lib/systemd/system']
if cmd in os.environ.get('FAKE_FAIL', '').split(','):
  print('Failed (fake)', file=sys.stderr)
  sys.exit(1)
def dropins():
  seen = {}
  for d in DIRS:
    dd = root / d / (unit + '.d')
    if dd.is_dir():
      for c in dd.iterdir():
        if c.name.endswith('.conf') and c.name not in seen:
          seen[c.name] = str(c)
  return [seen[n] for n in sorted(seen)]
def scan():
  frag = None
  for d in DIRS:
    f = root / d / unit
    if os.path.lexists(f):
      frag = 'masked' if f.is_symlink() and os.readlink(f) == '/dev/null' else d
      break
  conds, sec = [], None
  for c in dropins():
    for line in P(c).read_text().splitlines():
      line = line.strip()
      if line.startswith('['):
        sec = line
      elif sec == '[Unit]' and line.startswith('ConditionPathExists='):
        v = line.split('=', 1)[1]
        conds = [] if v == '' else conds + [v]
  return {'fragment': frag, 'condition_false': '!/dev/null' in conds, 'dropins': dropins()}
def loaded():
  p = st / 'loaded.json'
  return json.loads(p.read_text()) if p.exists() else scan()
def reload():
  (st / 'loaded.json').write_text(json.dumps(scan()))
def server(v=None):
  p = st / 'server'
  if v is not None:
    p.write_text(v)
  return p.read_text() if p.exists() else 'active'
def nth(name):
  p = st / ('n_' + name)
  n = int(p.read_text()) + 1 if p.exists() else 1
  p.write_text(str(n))
  return n
if cmd == 'is-active':
  n = nth('is-active')
  bad = os.environ.get('FAKE_ISACTIVE_BAD', '')  # mode@call numbers, e.g. empty@1 or rc4@2,3
  if bad:
    mode, calls = bad.split('@')
    if calls == '*' or str(n) in calls.split(','):
      if mode == 'empty':
        sys.exit(3)  # rc 3 but no text
      if mode == 'rc1':
        print('Failed to connect to bus: No such file or directory', file=sys.stderr)
        sys.exit(1)
      if mode == 'rc4':
        print('unknown', file=sys.stdout)
        sys.exit(4)
      if mode == 'word':
        print('active')
        sys.exit(3)  # text and code disagree
  s = server()
  print(s)
  sys.exit(0 if s == 'active' else 3)
if cmd == 'show':
  if '-p' in args and 'DropInPaths' in args[args.index('-p') + 1]:
    if os.environ.get('FAKE_SHOW') == 'fail':
      print('Failed to get properties (fake)', file=sys.stderr)
      sys.exit(1)
    shown = [x for x in loaded()['dropins'] if not (os.environ.get('FAKE_SHOW') == 'hide' and 'k9-switch-hold' in x)]
    mode = os.environ.get('FAKE_SHOW', '')
    if mode == 'noline':
      print('Id=' + unit)
    elif mode == 'empty':
      print('DropInPaths=')
    elif mode == 'broken':
      print('DropInPaths ' + ' '.join(shown))
    elif mode == 'relative':
      print('DropInPaths=' + ' '.join(P(x).name for x in shown))
    else:
      print('DropInPaths=' + ' '.join(shown))
  elif '-p' in args and args[args.index('-p') + 1] == 'LoadState':
    frag = loaded()['fragment']
    st_ = os.environ.get('FAKE_LOAD') or ('not-found' if frag is None else 'masked' if frag == 'masked' else 'loaded')
    if st_ == 'showfail':
      print('Failed to get properties (fake)', file=sys.stderr)
      sys.exit(1)
    print('' if st_ == 'blank' else 'LoadState=' + st_)
  else:
    print('ActiveState=' + server() + '\nSubState=x\nMainPID=0')
  sys.exit(0)
if cmd == 'daemon-reload':
  reload()
elif cmd == 'mask':
  m = root / 'run/systemd/system' / unit
  m.parent.mkdir(parents=True, exist_ok=True)
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
  with (st / 'start_calls').open('a') as f:
    f.write(' '.join(args) + '\n')
  cfg = loaded()
  if cfg['fragment'] in (None, 'masked'):
    sys.exit(1)
  if cfg['condition_false']:
    sys.exit(0)
  with (st / 'starts').open('a') as f:
    f.write('1\n')
  server('active')
'''


def fake(tmp):
  shim = Path(tmp) / 'fake_systemctl3.py'
  if not shim.exists():
    shim.write_text(SYSTEMCTL3)
  return [PY, str(shim)]


def ctl(root, *args, **env):
  e = dict(os.environ, FAKE_ROOT=str(root), **env)
  return subprocess.run([*fake(root.parent), *args], capture_output=True, text=True, env=e, timeout=60)


def log(root):
  p = root / 'run/fake-systemd3/log'
  return p.read_text().split('\n')[:-1] if p.exists() else []


def starts(root):
  p = root / 'run/fake-systemd3/starts'
  return len(p.read_text().split()) if p.exists() else 0


def active(root):
  return (root / 'run/fake-systemd3/server').read_text() if (root / 'run/fake-systemd3/server').exists() else 'active'


def dropdir(root):
  return root / 'run/systemd/system' / (UNIT + '.d')


@pytest.fixture
def etc_root(tmp_path):
  root = fixture_root(tmp_path)
  (root / 'etc/systemd/system' / UNIT).write_text('[Service]\nExecStart=/bin/true\nRestart=always\n')
  assert ctl(root, 'daemon-reload').returncode == 0
  (root / 'run/fake-systemd3/log').unlink()
  return root


def run_env(root, env, fn, *a):
  old = dict(os.environ)
  os.environ.update(FAKE_ROOT=str(root), **env)
  try:
    return fn(*a)
  finally:
    os.environ.clear()
    os.environ.update(old)


def hold(root, **env):
  return run_env(root, env, sl.hold_server, fake(root.parent), '/proc', 2, str(root), root)


def release(root, h, **env):
  return run_env(root, env, sl.release_server, fake(root.parent), h)


def cli(root, env=None):
  e = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', FAKE_ROOT=str(root), JET_SWITCH_PROC_TOKEN=str(root))
  e.update(env or {})
  args = [PY, str(HERE / 'switch_updater_linked.py'), '--root', str(root), '--manifest', str(MANIFEST),
          '--python', PY, '--boot-id', str(root.parent / 'boot_id'), '--lock-wait', '5',
          '--systemctl', ' '.join(fake(root.parent))]
  return subprocess.run(args, capture_output=True, text=True, env=e, timeout=180)


def updater_tree(root):
  out = {}
  for p in sorted((root / 'opt/carrot-jetlink').rglob('*')):
    if p.is_symlink():
      out[str(p.relative_to(root))] = 'link:' + os.readlink(p)
    elif p.is_file():
      out[str(p.relative_to(root))] = p.read_bytes()
  return out


def blocked_by_real_start(root):
  """What a start would really do with the drop-ins now on disk (model of systemd)."""
  ctl(root, 'stop', UNIT)
  ctl(root, 'daemon-reload')
  n = starts(root)
  ctl(root, 'start', UNIT)
  return starts(root) == n


# ---- 1) drop-in cases: tool verdict vs what start would do ----------------------

CASES = {
  'normal': {},
  'same_name_empty': {sl.HOLD_DROPIN: ''},
  'same_name_no_condition': {sl.HOLD_DROPIN: '[Unit]\nDescription=someone else\n'},
  'later_reset': {'60-reset.conf': '[Unit]\nConditionPathExists=\n'},
  'earlier_reset': {'10-reset.conf': '[Unit]\nConditionPathExists=\n'},
  'same_name_ours': {sl.HOLD_DROPIN: sl.HOLD_TEXT},
}
EXPECT = {'normal': 'go', 'same_name_empty': 'go', 'same_name_no_condition': 'go', 'later_reset': 'stop',
          'earlier_reset': 'go', 'same_name_ours': 'go'}


def put(root, files):
  d = dropdir(root)
  for name, text in files.items():
    d.mkdir(parents=True, exist_ok=True)
    (d / name).write_text(text)
  ctl(root, 'daemon-reload')


@pytest.mark.parametrize('case', CASES)
def test_hold_verdict_matches_real_start(etc_root, case):
  files = CASES[case]
  put(etc_root, files)
  if case == 'same_name_ours':  # left by a killed switch: the server is down (measure.log)
    ctl(etc_root, 'stop', UNIT)
  tree = updater_tree(etc_root)
  try:
    h = hold(etc_root)
    verdict = 'go'
  except sl.HoldAborted as e:
    h, verdict = e.hold, 'stop'
  assert verdict == EXPECT[case], h
  # never a start to test the hold
  assert not (etc_root / 'run/fake-systemd3/start_calls').exists()
  # what start would do with the hold drop-in in place
  d = dropdir(etc_root)
  (d / h['dropin_name']).parent.mkdir(parents=True, exist_ok=True)
  if verdict == 'stop':  # tool removed its file; put it back only to measure
    (d / h['dropin_name']).write_text(sl.HOLD_TEXT)
  really_blocked = blocked_by_real_start(etc_root)
  assert really_blocked == (verdict == 'go'), (case, h)
  if verdict == 'stop':
    (d / h['dropin_name']).unlink()
    ctl(etc_root, 'daemon-reload')
    assert h['restore'] == 'ok' and updater_tree(etc_root) == tree
  else:
    ctl(etc_root, 'start', UNIT)  # undo the measuring stop, as before hold
    release(etc_root, h)
  # files that were there before are kept byte for byte
  for name, text in files.items():
    assert (d / name).read_text() == text
  left = sorted(p.name for p in d.iterdir()) if d.exists() else []
  assert left == sorted(files)


@pytest.mark.parametrize('case', ['same_name_empty', 'same_name_no_condition'])
def test_untrusted_same_name_file_kept_own_unique_file_used(etc_root, case):
  put(etc_root, CASES[case])
  h = hold(etc_root)
  assert h['made_dropin'] and h['dropin_name'] != sl.HOLD_DROPIN and h['dropin_name'].startswith('50-k9-switch-hold-')
  assert h['before']['dropin_present'] and not h['before']['dropin_ours']
  assert (dropdir(etc_root) / h['dropin_name']).read_text() == sl.HOLD_TEXT
  assert h['check']['source'] == 'show' and '!/dev/null' in h['check']['ConditionPathExists']
  release(etc_root, h)
  assert sorted(p.name for p in dropdir(etc_root).iterdir()) == [sl.HOLD_DROPIN]
  assert active(etc_root) == 'active'


def test_old_trust_of_empty_same_name_file_would_not_hold(etc_root):
  """Why: with only the empty 50-k9-switch-hold.conf (what c48203ecf trusted) start runs."""
  put(etc_root, CASES['same_name_empty'])
  assert not blocked_by_real_start(etc_root)


def test_later_reset_aborts_before_stop_and_before_updater(etc_root):
  put(etc_root, CASES['later_reset'])
  with pytest.raises(sl.HoldAborted) as e:
    hold(etc_root)
  calls = log(etc_root)
  assert not any(c.startswith('stop') for c in calls) and not any(c.startswith('start') for c in calls)
  assert 'not in effect' in str(e.value) and e.value.hold['restore'] == 'ok'
  assert active(etc_root) == 'active' and not (etc_root / 'run/systemd/system' / UNIT).exists()


@pytest.mark.parametrize('mode', ['fail', 'hide'])
def test_show_failed_or_dropin_not_listed_aborts(etc_root, mode):
  with pytest.raises(sl.HoldAborted) as e:
    hold(etc_root, FAKE_SHOW=mode)
  assert e.value.hold['restore'] == 'ok' and active(etc_root) == 'active'
  assert not dropdir(etc_root).exists() and not (etc_root / 'run/systemd/system' / UNIT).exists()


def test_tool_never_starts_to_check_hold_cli(etc_root):
  r = cli(etc_root)
  assert r.returncode == 0, r.stderr
  calls = log(etc_root)
  starts_at = [i for i, c in enumerate(calls) if c.startswith('start')]
  assert [calls[i] for i in starts_at] == ['start --no-block ' + UNIT]  # only the restore start
  assert starts_at[0] > calls.index('unmask --runtime ' + UNIT)
  assert calls.index('show -p DropInPaths ' + UNIT) < calls.index('stop ' + UNIT)


def test_cli_hold_not_confirmed_exit_5_updater_untouched(etc_root):
  put(etc_root, CASES['later_reset'])
  tree = updater_tree(etc_root)
  r = cli(etc_root)
  assert r.returncode == 5, r.stderr
  assert 'hold not confirmed' in r.stderr
  assert json.loads(r.stderr.strip().splitlines()[-1])['server_hold']['restore'] == 'ok'
  assert updater_tree(etc_root) == tree and active(etc_root) == 'active'


def test_cli_hold_not_confirmed_and_restore_failed_exit_6(etc_root):
  put(etc_root, CASES['later_reset'])
  tree = updater_tree(etc_root)
  r = cli(etc_root, env={'FAKE_FAIL': 'unmask'})
  assert r.returncode == 6, r.stderr
  assert 'SERVER RESTORE FAILED (not success)' in r.stderr and 'switched' not in r.stdout.lower()
  rec = json.loads(r.stderr.strip().splitlines()[-1])['server_hold']
  assert rec['restore'] == 'FAILED' and rec['restore_failed']
  assert updater_tree(etc_root) == tree


# ---- 2) is-active exit code + text ----------------------------------------------

@pytest.mark.parametrize('mode', ['empty', 'rc1', 'rc4', 'word'])
def test_is_active_query_error_before_hold_nothing_done(etc_root, mode):
  with pytest.raises(sl.HoldAborted) as e:
    hold(etc_root, FAKE_ISACTIVE_BAD=mode + '@1')
  h = e.value.hold
  assert h['is_active_before']['state'] is None and 'show' in h['is_active_before']
  assert h['restore'] == 'nothing done' and not dropdir(etc_root).exists()
  assert [c.split()[0] for c in log(etc_root)] == ['is-active', 'show']


@pytest.mark.parametrize('mode', ['empty', 'rc1', 'rc4', 'word'])
def test_is_active_query_error_after_stop_aborts_and_restores(etc_root, mode):
  with pytest.raises(sl.HoldAborted) as e:
    hold(etc_root, FAKE_ISACTIVE_BAD=mode + '@2')
  h = e.value.hold
  assert h['is_active_after_stop']['state'] is None and 'unknown' in str(e.value)
  assert h['restore'] == 'ok' and active(etc_root) == 'active'
  assert not dropdir(etc_root).exists() and not (etc_root / 'run/systemd/system' / UNIT).exists()


def test_is_active_error_after_stop_restore_also_failed_exit_6(etc_root):
  tree = updater_tree(etc_root)
  r = cli(etc_root, env={'FAKE_ISACTIVE_BAD': 'rc1@2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,'
                                              + ','.join(str(i) for i in range(21, 200))})
  assert r.returncode == 6, r.stderr
  rec = json.loads(r.stderr.strip().splitlines()[-1])['server_hold']
  assert rec['restore'] == 'FAILED' and 'state unknown after start' in ' '.join(rec['restore_failed'])
  assert updater_tree(etc_root) == tree


def test_restart_check_query_error_is_not_success(etc_root, monkeypatch):
  monkeypatch.setattr(sl, 'RESTORE_WAIT', 1)
  h = hold(etc_root)
  with pytest.raises(sl.ServerRestoreError) as e:
    release(etc_root, h, FAKE_ISACTIVE_BAD='empty@*')
  assert 'state unknown after start' in str(e.value)
  assert h['restored_query']['state'] is None and h['restored_query']['rc'] == 3 and h['restored_query']['out'] == ''


@pytest.mark.parametrize('rc,out,state', [(0, 'active', 'active'), (3, 'inactive', 'inactive'),
                                          (3, 'failed', 'failed'), (3, 'activating', 'activating'),
                                          (3, '', None), (1, '', None), (4, 'unknown', None),
                                          (3, 'active', None), (0, 'inactive', None)])
def test_query_active_table(monkeypatch, rc, out, state):
  class R:
    def __init__(self, rc, out):
      self.returncode, self.stdout, self.stderr = rc, out, ''
  calls = []

  def run(systemctl, *args):
    calls.append(args)
    return R(rc, out + '\n') if args[0] == 'is-active' else R(0, 'ActiveState=x\n')
  monkeypatch.setattr(sl, 'systemctl_run', run)
  q = sl.query_active(['x'], UNIT)
  assert q['state'] == state and q['rc'] == rc and q['out'] == out
  assert ('show' in q) == (state is None)


# ---- review 11/16 08:50: no scan fallback, LoadState must be loaded ----------

@pytest.mark.parametrize('mode', ['noline', 'empty', 'broken', 'relative'])
def test_dropinpaths_missing_or_unreadable_aborts_restored(etc_root, mode):
  with pytest.raises(sl.HoldAborted) as e:
    hold(etc_root, FAKE_SHOW=mode)
  h = e.value.hold
  assert h['restore'] == 'ok' and active(etc_root) == 'active'
  # disk scan never judged: empty line = systemd lists nothing (not ours), others = no view at all
  assert h['check'].get('dropins', []) == [] and h['check'].get('source') in (None, 'show')
  assert ('disk_scan' in h['check']) == (mode != 'empty')
  calls = log(etc_root)
  assert not any(c.startswith(('stop', 'start')) for c in calls)
  assert not dropdir(etc_root).exists() and not (etc_root / 'run/systemd/system' / UNIT).exists()


@pytest.mark.parametrize('mode', ['noline', 'empty', 'broken'])
def test_cli_dropinpaths_missing_exit_5_updater_untouched(etc_root, mode):
  before = updater_tree(etc_root)
  r = cli(etc_root, {'FAKE_SHOW': mode})
  assert r.returncode == 5, r.stderr
  assert updater_tree(etc_root) == before and active(etc_root) == 'active' and starts(etc_root) == 0


def test_cli_dropinpaths_missing_and_restore_failed_exit_6(etc_root):
  before = updater_tree(etc_root)
  r = cli(etc_root, {'FAKE_SHOW': 'noline', 'FAKE_FAIL': 'unmask'})
  assert r.returncode == 6, r.stderr
  assert 'RESTORE FAILED' in r.stderr and updater_tree(etc_root) == before


@pytest.mark.parametrize('load', ['not-found', 'masked', 'error', 'bad-setting', 'blank', 'showfail'])
def test_loadstate_not_loaded_aborts_nothing_done(etc_root, load):
  with pytest.raises(sl.HoldAborted) as e:
    hold(etc_root, FAKE_LOAD=load)
  h = e.value.hold
  assert h['restore'] == 'nothing done' and 'LoadState' in str(e.value)
  assert [c.split()[0] for c in log(etc_root)] == ['is-active', 'show']
  assert not dropdir(etc_root).exists() and not (etc_root / 'run/systemd/system' / UNIT).exists()


def test_missing_unit_is_not_found_not_stopped(etc_root):
  (etc_root / 'etc/systemd/system' / UNIT).unlink()
  assert ctl(etc_root, 'daemon-reload').returncode == 0
  ctl(etc_root, 'stop', UNIT)  # is-active: inactive rc 3, same as a stopped unit
  q = ctl(etc_root, 'is-active', UNIT)
  assert q.returncode == 3 and q.stdout.strip() == 'inactive'
  with pytest.raises(sl.HoldAborted) as e:
    hold(etc_root)
  assert 'not-found' in str(e.value)


@pytest.mark.parametrize('load', ['not-found', 'masked'])
def test_cli_loadstate_exit_5_updater_untouched(etc_root, load):
  before = updater_tree(etc_root)
  r = cli(etc_root, {'FAKE_LOAD': load})
  assert r.returncode == 5, r.stderr
  assert updater_tree(etc_root) == before and starts(etc_root) == 0
