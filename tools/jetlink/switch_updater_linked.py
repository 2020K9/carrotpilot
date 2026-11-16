"""Switch with no window: the marker changes in the same rename as the updater.

Same steps and checks as switch_updater.py (backup, stage, pre-check, exchange,
verify, recovery), with two additions (review 11/16 06:28):

1. Update lock.  The whole switch holds /run/carrot-jetlink-update.lock, the
   flock every legacy update_host.py action except the early boot-gate returns
   takes (0cb67d46c update_host.py:362-363, LOCK_NB).  A timer run
   (carrot-jetlink-update-stage.timer -> update_host.py automatic) that starts
   during the switch fails on the lock and writes nothing; if an update action
   already holds the lock the switch waits, then stops with nothing changed.

2. Linked marker (only when boot-update-required is absent before the switch).
   The boot id is written inside the staged new updater as
   updater/switch-boot-marker, and boot-update-required is made a relative
   symlink to updater/switch-boot-marker BEFORE the exchange.  With the old
   updater installed the link dangles: Path.is_file() is False, which is
   exactly the state before the switch (legacy enabled() boot_update.py:26,
   migrate_running_release() install_updates.py:26 read only is_file()).
   The exchange (one renameat2) makes the link resolve, so "new updater" and
   "marker present" become visible in the same rename.  Afterwards the link is
   replaced by a regular file with the same bytes (marker stays present).
   Recovery reverses it: regular file -> link (still present, new updater),
   exchange back (link dangles = state before), link moved aside (never
   deleted).

3. Legacy server held off (review 11/16 07:22).  The legacy inference server
   (carrot-jetlink.service, Restart=always, RestartSec=3) runs
   current/tools/jetlink/server.py, whose first lines (0cb67d46c server.py:10-13)
   call install_updates.migrate_running_release() -> sudo install_updates.py
   --bootstrap-only -> bootstrap() whenever boot-update-required is not a
   regular file (install_updates.py:26-29, 182-186).  bootstrap() takes no lock
   and writes updater/<name> by path (install_updates.py:90-114), so a server
   (re)start during the switch writes old files into whatever folder is named
   updater/ at that moment.  The switch therefore, after taking the lock and
   before any write: holds carrot-jetlink.service off for this boot only, stops
   it, checks it is not active and waits until no install_updates.py
   --bootstrap-only process is left; otherwise it stops with nothing changed.

   How it is held (review 11/16 07:49, measured with real systemd 252 user
   manager, hold_0750/measure_a.log, measure_b.log): the legacy
   install_server.sh puts the unit in /etc/systemd/system, and /etc wins over
   /run, so "systemctl mask --runtime" alone returns 0, creates the /run link,
   but the unit stays loaded (UnitFileState=enabled-runtime) and Restart=always
   and "start" still run it.  The switch therefore writes the runtime drop-in
   /run/systemd/system/carrot-jetlink.service.d/50-k9-switch-hold.conf
   ([Unit] ConditionPathExists=!/dev/null, never true).  Drop-ins from /run
   apply to a unit in /etc; with it "start" returns 0 but the unit stays
   inactive (ConditionResult=no) and Restart=always does not restart it
   (NRestarts=0).  mask --runtime is still done (it reloads the daemon so the
   drop-in is loaded, and it does block a unit installed only in /lib).

   Only what this switch created is undone (state recorded before): the
   drop-in and the /run mask are removed only if they were not there before;
   a mask in /etc or /run, an existing drop-in or a stopped server are left as
   found; the server is started again only if it was active.  Every command's
   exit code is checked and the server must be active again: otherwise the
   result is NOT success (exit 3 "switched, server restore failed", exit 4
   "switch failed, server restore failed") with the state printed.
   A killed switch leaves the drop-in and the runtime mask until the next
   reboot (server stays down; reboot clears /run, the enabled unit starts).

   Hold check (review 11/16 08:17/08:18).  The switch never runs "start" to
   test the hold (if the hold did not work that start would run the legacy
   server = the reinstall it must prevent).  After the reload it reads what
   systemd applies: "systemctl show -p DropInPaths" (order as systemd uses
   them; measured on systemd 252: a later drop-in with an empty
   "ConditionPathExists=" is listed after ours and resets it, start then runs
   the unit; "Conditions" is [unprintable] and ConditionResult means nothing
   before a start), then reads each listed file in that order: the
   [Unit] ConditionPathExists list (empty value = reset) must end up holding
   "!/dev/null".  If show prints no DropInPaths line, an unreadable line or
   a non-absolute path, the switch stops (files on disk are no proof of what
   systemd applies; they are only recorded as 'disk_scan', never judged).
   Before anything "systemctl show -p LoadState" must say "loaded"
   (is-active says "inactive"/rc 3 for a missing unit too): not-found,
   masked, error, empty or a failed show stop the switch, nothing done.  An
   existing 50-k9-switch-hold.conf is only trusted if its text is exactly
   ours; otherwise it is kept untouched and an own uniquely named drop-in is
   written (only that one is removed later).  Anything unclear (show failed,
   our file not listed, unreadable file, condition reset, is-active not
   answering) stops the switch BEFORE any updater file is touched; what the
   hold already did is undone and judged: exit 5 = held check failed, server
   restored; exit 6 = held check failed AND restore failed (not success).

   is-active is judged by exit code and text together: rc 0 + active/reloading
   = running, rc 3 + inactive/failed = stopped, rc 3 + activating/
   deactivating = in between; anything else (empty, rc 1/4, other text) is a
   query error (show -p ActiveState,SubState,MainPID is recorded as well) and
   counts as unknown, never as stopped.  The restart check after the switch
   uses the same rule.

4. Staging keeps the old folder's other entries exactly: sub-folders (e.g.
   before-usb-experiments/ from install_usb_receiver.py:42-49) are copied with
   symlinks kept as symlinks, a symlinked file stays a symlink, modes are kept.
   Not copied: __pycache__/ (byte-code cache, rebuilt on import) and <name>.new
   temp files of a legacy bootstrap killed mid-write (they stay in the backup).

crash_point()/fail_point() are test hooks (os._exit is a process kill, NOT a
power cut; it proves nothing about what a real power loss keeps on disk).
"""
import argparse
import errno as errno_mod
import fcntl
import json
import os
from pathlib import Path
import shutil
import sys
import time

import switch_updater as base
from switch_updater import (NAMES, NEW_KEY, OLD_KEY, UNITS, UPDATER_COMMAND, SwitchError, check_old_updater,
                            exchange, fsync_dir, key_of, sha, tree_hashes, verify_updater, write_durable)

INNER = 'switch-boot-marker'
LINK_TARGET = 'updater/' + INNER
LOCK = 'run/carrot-jetlink-update.lock'
SERVER_UNIT = 'carrot-jetlink.service'
BOOTSTRAP_ARGS = (b'install_updates.py', b'--bootstrap-only')
SKIP_STAGE = ('__pycache__',)


def crash_point(name):
  base.crash_point(name)
  pause = os.environ.get('JET_SWITCH_PAUSE_AT')
  if pause == name:  # test only: stop here until the test says go
    flag = Path(os.environ['JET_SWITCH_PAUSE_FILE'])
    flag.with_suffix('.reached').write_text(name)
    deadline = time.monotonic() + 120
    while not flag.with_suffix('.go').exists() and time.monotonic() < deadline:
      time.sleep(0.05)


def fail_point(name):
  base.fail_point(name)


def recovery_fail_point(name):
  # Test only: second I/O error injected inside the recovery itself.
  if os.environ.get('JET_SWITCH_RECOVERY_FAIL_AT') == name:
    raise OSError(5, 'injected I/O error in recovery at ' + name)


def is_our_link(marker):
  return marker.is_symlink() and os.readlink(marker) == LINK_TARGET


def take_lock(root, wait):
  path = Path(root) / LOCK
  path.parent.mkdir(parents=True, exist_ok=True)
  stream = path.open('a')
  deadline = time.monotonic() + wait
  while True:
    try:
      fcntl.flock(stream, fcntl.LOCK_EX | fcntl.LOCK_NB)
      return stream
    except OSError as error:
      if error.errno not in (errno_mod.EAGAIN, errno_mod.EACCES) or time.monotonic() >= deadline:
        stream.close()
        raise SwitchError('update service holds ' + LOCK + '; nothing changed') from error
      time.sleep(0.2)


def make_link(marker, stamp):
  link = marker.with_name(marker.name + '.switch-link-' + stamp)
  if os.path.lexists(link):
    os.rename(link, link.with_name(link.name + '.stale-' + str(time.time_ns())))
  os.symlink(LINK_TARGET, link)
  os.replace(link, marker)


def regular_from_link(marker, stamp):
  """Replace our link by a regular file with the same bytes (stays present)."""
  data = marker.read_bytes()
  temporary = marker.with_name(marker.name + '.switch-file-' + stamp)
  write_durable(temporary, data)
  crash_point('after_marker_tmp')
  fail_point('marker_rename')
  os.replace(temporary, marker)
  crash_point('after_marker_rename')
  fail_point('marker_fsync')
  fsync_dir(marker.parent)


def systemctl_run(systemctl, *args):
  import subprocess
  return subprocess.run([*systemctl, *args], capture_output=True, text=True, timeout=60)


def bootstrap_processes(proc, token=None):
  """pids whose command line runs install_updates.py --bootstrap-only."""
  found = []
  for entry in Path(proc).iterdir():
    if not entry.name.isdigit() or int(entry.name) == os.getpid():
      continue
    try:
      words = (entry / 'cmdline').read_bytes().split(b'\0')
    except OSError:
      continue
    if any(w.endswith(BOOTSTRAP_ARGS[0]) for w in words) and BOOTSTRAP_ARGS[1] in words:
      if token is None or token.encode() in words:
        found.append(int(entry.name))
  return found


HOLD_DROPIN = '50-k9-switch-hold.conf'
HOLD_TEXT = '[Unit]\n# carrot-jetlink switch_updater_linked.py: server held off during the switch\nConditionPathExists=!/dev/null\n'
PERSIST_DIRS = ('etc/systemd/system',)
RUNTIME_DIR = 'run/systemd/system'
RESTORE_WAIT = 20


class ServerRestoreError(Exception):
  """The server could not be brought back to its state before the switch.
  report: the switch report (switch done) or None (switch failed: switch_error)."""

  def __init__(self, message, hold, report=None, switch_error=None, phase=None):
    super().__init__(message)
    self.hold = hold
    self.report = report
    self.switch_error = switch_error
    self.phase = phase  # 'hold': the hold check failed and its undo failed (exit 6)


def unit_state(root, unit, runtime_dir, persist_dirs):
  """What is there before the switch: masks (unit file -> /dev/null) and our drop-in."""
  def masked(d):
    f = Path(d) / unit
    return f.is_symlink() and os.readlink(f) == '/dev/null'
  root = Path(root)
  dropin = root / runtime_dir / (unit + '.d') / HOLD_DROPIN
  state = {'runtime_masked': masked(root / runtime_dir),
           'persist_masked': [str(d) for d in persist_dirs if masked(root / d)],
           'dropin_present': os.path.lexists(dropin)}
  if state['dropin_present']:
    try:
      state['dropin_ours'] = dropin.is_file() and not dropin.is_symlink() and dropin.read_text() == HOLD_TEXT
    except (OSError, UnicodeDecodeError):
      state['dropin_ours'] = False
  return state


class HoldAborted(SwitchError):
  """The hold could not be confirmed; nothing of the updater was touched and
  what the hold did was undone (server restored).  hold: the record."""

  def __init__(self, message, hold):
    super().__init__(message)
    self.hold = hold


ACTIVE_OUT = ('active', 'reloading', 'refreshing')
STOPPED_OUT = ('inactive', 'failed')
BETWEEN_OUT = ('activating', 'deactivating')


def query_active(systemctl, unit):
  """is-active judged by exit code AND text.  state: 'active', 'inactive',
  'failed', 'activating', 'deactivating' or None (query error)."""
  r = systemctl_run(systemctl, 'is-active', unit)
  out = r.stdout.strip()
  q = {'rc': r.returncode, 'out': out, 'err': r.stderr.strip()[:200], 'state': None}
  if r.returncode == 0 and out in ACTIVE_OUT:
    q['state'] = 'active'
  elif r.returncode == 3 and out in STOPPED_OUT + BETWEEN_OUT:
    q['state'] = out
  else:
    s = systemctl_run(systemctl, 'show', '-p', 'ActiveState,SubState,MainPID', unit)
    q['show'] = {'rc': s.returncode, 'out': s.stdout.strip()[:200], 'err': s.stderr.strip()[:200]}
  return q


def query_text(q):
  return f"is-active rc={q['rc']} out={q['out']!r} err={q['err']!r}" + (f" show={q['show']}" if 'show' in q else '')


def parse_show(text, key):
  for line in text.splitlines():
    if line.startswith(key + '='):
      return line[len(key) + 1:].split()
  return None


def scan_dropins(root, unit, dirs):
  """Drop-ins as systemd orders them: same name -> first dir wins, then by name."""
  seen = {}
  for d in dirs:
    dd = Path(root) / d / (unit + '.d')
    if dd.is_dir():
      for c in dd.iterdir():
        if c.name.endswith('.conf') and c.name not in seen:
          seen[c.name] = str(c)
  return [seen[n] for n in sorted(seen)]


def effective_path_conditions(paths):
  """[Unit] ConditionPathExists= list after reading the drop-ins in order
  (empty value resets it).  Raises OSError/UnicodeDecodeError when unreadable."""
  conds = []
  for path in paths:
    f = Path(path)
    section = None
    for line in f.read_text().splitlines():
      line = line.strip()
      if not line or line[0] in '#;':
        continue
      if line.startswith('[') and line.endswith(']'):
        section = line[1:-1]
        continue
      if section == 'Unit' and '=' in line:
        key, value = (x.strip() for x in line.split('=', 1))
        if key == 'ConditionPathExists':
          conds = [] if value == '' else conds + [value]
  return conds


def check_hold(systemctl, hold):
  """Read (never start) what systemd applies.  Returns None if held, else why not."""
  unit = hold['unit']
  ours = str(Path(hold['root']) / hold['runtime_dir'] / (unit + '.d') / hold['dropin_name'])
  check = hold['check'] = {}
  r = systemctl_run(systemctl, 'show', '-p', 'DropInPaths', unit)
  check['show'] = {'rc': r.returncode, 'out': r.stdout.strip()[:600], 'err': r.stderr.strip()[:200]}
  if r.returncode != 0:
    return f'systemctl show -p DropInPaths rc={r.returncode} {r.stderr.strip()[:120]}'
  paths = parse_show(r.stdout, 'DropInPaths')
  if paths is None or any(not os.path.isabs(x) for x in paths):
    try:  # diagnosis only: disk files are not what systemd applies
      check['disk_scan'] = scan_dropins(hold['root'], unit, ('etc/systemd/system', hold['runtime_dir'], 'lib/systemd/system'))
    except OSError as error:
      check['disk_scan'] = f'error {error}'
    what = 'no DropInPaths line' if paths is None else f'unreadable DropInPaths {paths}'
    return f'systemctl show: {what} (out={r.stdout.strip()[:200]!r}); systemd view unknown'
  check['source'] = 'show'
  check['dropins'] = paths
  norm = [os.path.normpath(x) for x in paths]
  if os.path.normpath(ours) not in norm:
    return f'hold drop-in {ours} not applied by systemd (DropInPaths={paths})'
  try:
    if Path(ours).read_text() != HOLD_TEXT:
      return f'hold drop-in {ours} text changed'
    conds = effective_path_conditions(paths)
  except (OSError, UnicodeDecodeError) as error:
    return f'drop-in unreadable: {error}'
  check['ConditionPathExists'] = conds
  if '!/dev/null' not in conds:
    later = norm[norm.index(os.path.normpath(ours)) + 1:]
    return f'ConditionPathExists=!/dev/null not in effect (reset by a later drop-in? {later})'
  return None


def hold_server(systemctl, proc, wait, token=None, root='/', unit=SERVER_UNIT,
                runtime_dir=RUNTIME_DIR, persist_dirs=PERSIST_DIRS):
  """Hold the legacy server off (this boot only) and stop it; no bootstrap left running.
  Never runs start to test the hold: the applied drop-ins are read instead.
  Returns the hold record (state before + what was created).  Raises HoldAborted
  (what it did undone, server restored) or ServerRestoreError (undo failed too)."""
  before = unit_state(root, unit, runtime_dir, persist_dirs)
  q0 = query_active(systemctl, unit)
  hold = {'unit': unit, 'root': str(root), 'runtime_dir': runtime_dir, 'before': before,
          'is_active_before': q0, 'was_active': q0['state'] in ('active', 'activating'),
          'made_dropin': False, 'made_mask': False, 'dropin_name': HOLD_DROPIN, 'stopped': False}
  if q0['state'] is None:
    hold['restore'] = 'nothing done'
    raise HoldAborted(f'{unit} state unknown ({query_text(q0)}); nothing changed', hold)
  ls = systemctl_run(systemctl, 'show', '-p', 'LoadState', unit)
  load = parse_show(ls.stdout, 'LoadState') if ls.returncode == 0 else None
  hold['load_state'] = {'rc': ls.returncode, 'out': ls.stdout.strip()[:200], 'err': ls.stderr.strip()[:200]}
  if load != ['loaded']:
    hold['restore'] = 'nothing done'
    raise HoldAborted(f'{unit} LoadState not loaded ({hold["load_state"]}); nothing changed', hold)
  try:
    if not before['dropin_present'] or not before.get('dropin_ours'):
      if before['dropin_present']:  # not ours: kept untouched, own file next to it
        hold['dropin_name'] = HOLD_DROPIN[:-len('.conf')] + f'-{os.getpid()}-{time.time_ns():x}.conf'
      dropin = Path(root) / runtime_dir / (unit + '.d') / hold['dropin_name']
      dropin.parent.mkdir(parents=True, exist_ok=True)
      tmp = dropin.with_name(dropin.name + '.tmp')
      tmp.write_text(HOLD_TEXT)
      os.replace(tmp, dropin)
      hold['made_dropin'] = True
    first = ('mask', '--runtime', unit) if not before['runtime_masked'] else ('daemon-reload',)
    r = systemctl_run(systemctl, *first)  # mask also reloads: the drop-in is loaded
    if r.returncode != 0:
      raise HoldAborted(f'systemctl {" ".join(first)} failed: {r.stderr.strip()[:120]}; nothing changed', hold)
    if first[0] == 'mask':
      hold['made_mask'] = True
    why = check_hold(systemctl, hold)
    if why:
      raise HoldAborted(f'hold not confirmed: {why}; nothing changed', hold)
    hold['stopped'] = True  # from here a restore needs a start (if it was active)
    r = systemctl_run(systemctl, 'stop', unit)
    if r.returncode != 0:
      raise HoldAborted(f'systemctl stop {unit} failed: {r.stderr.strip()[:120]}; nothing changed', hold)
    q = hold['is_active_after_stop'] = query_active(systemctl, unit)
    if q['state'] is None:
      raise HoldAborted(f'{unit} state unknown after stop ({query_text(q)}); nothing changed', hold)
    if q['state'] not in STOPPED_OUT:
      raise HoldAborted(unit + ' still active; nothing changed', hold)
    deadline = time.monotonic() + wait
    while bootstrap_processes(proc, token):
      if time.monotonic() >= deadline:
        raise HoldAborted('install_updates.py --bootstrap-only still running; nothing changed', hold)
      time.sleep(0.1)
  except BaseException as error:
    try:
      release_server(systemctl, hold)
    except ServerRestoreError as restore:
      hold['restore'] = 'FAILED'
      raise ServerRestoreError(str(restore), hold, switch_error=str(error), phase='hold') from error
    hold['restore'] = 'ok'
    if isinstance(error, HoldAborted) or not isinstance(error, Exception):
      raise
    raise HoldAborted(f'{error}; nothing changed', hold) from error
  return hold


def release_server(systemctl, hold):
  """Undo only what hold_server created; start the server only if it was active.
  Raises ServerRestoreError when a command fails or the server is not active again."""
  if hold is None:
    return
  unit = hold['unit']
  failed = []
  hold['release'] = steps = []

  def call(*args):
    r = systemctl_run(systemctl, *args)
    steps.append({'cmd': ' '.join(args), 'rc': r.returncode, 'err': r.stderr.strip()[:200]})
    if r.returncode != 0:
      failed.append(f'systemctl {" ".join(args)} rc={r.returncode} {r.stderr.strip()[:120]}')
    return r

  if hold.get('made_dropin'):
    dropin = Path(hold['root']) / hold['runtime_dir'] / (unit + '.d') / hold.get('dropin_name', HOLD_DROPIN)
    try:
      dropin.unlink()
    except FileNotFoundError:
      pass
    except OSError as error:
      failed.append(f'remove {dropin}: {error}')
    try:
      dropin.parent.rmdir()
    except OSError:
      pass  # not empty (other drop-ins) or gone: left as is
  if hold.get('made_mask'):
    call('unmask', '--runtime', unit)  # also reloads (drop-in gone)
  elif hold.get('made_dropin'):
    call('daemon-reload')
  if hold.get('was_active') and not failed:
    if hold.get('stopped', True):  # not stopped (check failed before stop): no start, only confirm
      call('start', '--no-block', unit)
    q = None
    deadline = time.monotonic() + RESTORE_WAIT
    while not failed:
      q = query_active(systemctl, unit)
      if q['state'] == 'active' or time.monotonic() >= deadline:
        break
      time.sleep(0.2)
    hold['restored_state'] = q['state'] if q else ''
    hold['restored_query'] = q
    if not failed and q is not None and q['state'] is None:
      failed.append(f'{unit} state unknown after start ({query_text(q)}) (was active before the switch)')
    elif not failed and q is not None and q['state'] != 'active':
      failed.append(f'{unit} is {q["state"]} after start (was active before the switch)')
  elif hold.get('was_active'):
    failed.append(f'{unit} not started (was active before the switch)')
  if failed:
    hold['restore_failed'] = failed
    raise ServerRestoreError('; '.join(failed), hold)


def switch(root, source, manifest_path, python, boot_id_path, stamp=None, lock_wait=120,
           systemctl=None, proc='/proc', unit=SERVER_UNIT, runtime_dir=RUNTIME_DIR, persist_dirs=PERSIST_DIRS):
  """systemctl: command list (e.g. ['systemctl']) to hold the legacy server off;
  None skips that step (fake-root tests of the earlier version only).
  unit/runtime_dir/persist_dirs: test only (real user-manager systemd tests)."""
  root = Path(root)
  lock = take_lock(root, lock_wait)
  try:
    crash_point('after_lock')
    held = None
    if systemctl:
      held = hold_server(systemctl, proc, lock_wait, os.environ.get('JET_SWITCH_PROC_TOKEN'),
                         root=root, unit=unit, runtime_dir=runtime_dir, persist_dirs=persist_dirs)
      crash_point('after_hold')
    try:
      report = _switch(root, Path(source), Path(manifest_path), python, boot_id_path,
                       stamp or time.strftime('%Y%m%d-%H%M%S'))
      report['server_held'] = systemctl is not None
    except BaseException as error:
      if systemctl:
        try:
          release_server(systemctl, held)
        except ServerRestoreError as restore:
          raise ServerRestoreError(str(restore), held, switch_error=str(error)) from error
      raise
    if systemctl:
      try:
        release_server(systemctl, held)
      except ServerRestoreError as restore:
        report['server_restore'] = 'FAILED'
        raise ServerRestoreError(str(restore), held, report=report) from None
      report['server_restore'] = 'ok'
    return report
  finally:
    lock.close()


def is_bootstrap_temp(name):
  """<name>.new left by a legacy bootstrap() killed mid-write (install_updates.py:102-108).
  Kept in the backup, not carried into the new updater."""
  return name.endswith('.new') and name[:-4] in NAMES


def copy_entry(item, target):
  """Copy one entry of the old updater as it is (link stays link, folder whole)."""
  if item.is_symlink():
    os.symlink(os.readlink(item), target)
  elif item.is_dir():
    shutil.copytree(item, target, symlinks=True)
    for folder, _, files in os.walk(target):
      for f in files:
        p = os.path.join(folder, f)
        if not os.path.islink(p):
          with open(p, 'rb') as s:
            os.fsync(s.fileno())
      fsync_dir(folder)
  elif item.is_file():
    write_durable(target, item.read_bytes())
    os.chmod(target, item.stat().st_mode & 0o7777)


def _switch(root, source, manifest_path, python, boot_id_path, stamp):
  runtime = root / 'opt/carrot-jetlink'
  updater = runtime / 'updater'
  marker = runtime / 'boot-update-required'
  units = root / 'etc/systemd/system'
  report = {'stamp': stamp, 'locked': True}

  if not updater.is_dir() or updater.is_symlink():
    raise SwitchError('no installed updater folder')
  current_key = key_of(updater / 'release-signing-public.pem')
  if current_key == NEW_KEY:
    report['selected_commit'] = verify_updater(python, updater, root, manifest_path, NEW_KEY)
    if is_our_link(marker) and marker.is_file():
      regular_from_link(marker, stamp)  # finish an interrupted switch, same bytes
      report['marker_written'] = 'link made regular (completed interrupted switch)'
    elif not marker.is_file():
      base.write_marker(marker, boot_id_path)  # only reachable for switch_updater.py leftovers
      report['marker_written'] = 'this-boot-id (completed interrupted switch)'
    report['result'] = 'already-new'
    return report
  if current_key != OLD_KEY:
    raise SwitchError('installed updater key is neither the old nor the new key; stop')
  if key_of(source / 'release-signing-public.pem') != NEW_KEY:
    raise SwitchError('payload is not the new GitHub updater')
  for name in UNITS[:2]:
    unit = units / name
    if unit.is_file() and UPDATER_COMMAND not in unit.read_text():
      raise SwitchError(f'{name} does not run {UPDATER_COMMAND}; stop')
  if marker.is_symlink() and not is_our_link(marker):
    raise SwitchError('boot-update-required is a foreign symlink; stop')
  verify_updater(python, source, root, manifest_path, NEW_KEY)
  # A dangling link of ours (earlier interrupted run) is the "absent" state.
  marker_before = marker.read_bytes() if marker.is_file() else None
  linked = marker_before is None
  report['marker_before'] = None if marker_before is None else marker_before.decode(errors='replace').strip()

  # 1. backup (same as switch_updater.py)
  backup = runtime / 'updater-switch-backup' / stamp
  if backup.exists():
    raise SwitchError('backup folder already exists; refusing to overwrite')
  backup.parent.mkdir(parents=True, exist_ok=True)
  partial = backup.with_name(stamp + '.partial')
  if partial.exists():
    raise SwitchError('partial backup folder already exists')
  shutil.copytree(updater, partial / 'updater', symlinks=True)
  (partial / 'units').mkdir()
  for name in UNITS:
    if (units / name).is_file():
      shutil.copy2(units / name, partial / 'units' / name)
  if marker_before is not None:
    write_durable(partial / 'boot-update-required', marker_before)
  info = {'marker_present': marker_before is not None, 'old_key': current_key,
          'updater_sha256': tree_hashes(updater), 'linked_marker': linked}
  write_durable(partial / 'BACKUP.json', json.dumps(info, indent=1).encode())
  for folder, _, files in os.walk(partial):
    for f in files:
      with open(os.path.join(folder, f), 'rb') as s:
        os.fsync(s.fileno())
    fsync_dir(folder)
  os.rename(partial, backup)
  fsync_dir(backup.parent)
  report['backup'] = str(backup)
  crash_point('after_backup')

  # 2. stage, plus the inner marker when linking
  staged = runtime / ('updater.switch-' + stamp)
  if staged.exists():
    raise SwitchError('staging folder already exists')
  staged.mkdir()
  for item in updater.iterdir():
    if item.name not in NAMES and item.name != INNER and item.name not in SKIP_STAGE and not is_bootstrap_temp(item.name):
      copy_entry(item, staged / item.name)
  payload = [n for n in NAMES if (source / n).is_file()]
  for i, name in enumerate(payload):
    write_durable(staged / name, (source / name).read_bytes())
    (staged / name).chmod(0o644)
    if i == len(payload) // 2:
      crash_point('mid_stage')
  if linked:
    write_durable(staged / INNER, Path(boot_id_path).read_bytes())
  staged.chmod(updater.stat().st_mode & 0o7777)
  fsync_dir(staged)
  fsync_dir(runtime)
  crash_point('after_stage')

  # 3. pre-check
  verify_updater(python, staged, root, manifest_path, NEW_KEY)

  # 3b. dangling link (old updater: is_file() False = state before)
  if linked:
    make_link(marker, stamp)
    fsync_dir(runtime)
    crash_point('after_link')

  # 4. exchange: updater and (resolving) marker together
  exchange(updater, staged)
  crash_point('after_exchange')
  replaced = runtime / ('updater.replaced-' + stamp)
  try:
    fail_point('fsync_after_exchange')
    fsync_dir(runtime)
    crash_point('after_exchange_fsync')
    if linked:
      regular_from_link(marker, stamp)
      report['marker_written'] = 'this-boot-id (gate from next boot), visible with the exchange'
    crash_point('after_marker')
    fail_point('rename_old')
    os.rename(staged, replaced)
    fsync_dir(runtime)
    crash_point('after_rename_old')
    if os.environ.get('JET_SWITCH_FORCE_VERIFY_FAIL') == '1':
      raise SwitchError('forced verification failure (test)')
    fail_point('verify')
    commit = verify_updater(python, updater, root, manifest_path, NEW_KEY)
    for name in payload:
      if sha(updater / name) != sha(source / name):
        raise SwitchError(f'{name} differs from payload after install')
    crash_point('after_verify')
  except BaseException as error:
    recover(runtime, updater, marker, marker_before, stamp, staged, replaced)
    check_old_updater(python, updater, backup)
    report['result'] = 'restored-old'
    report['error'] = str(error)[:240]
    raise SwitchError('switch failed and old updater restored: ' + str(error)[:200]) from error
  report['result'] = 'switched'
  report['selected_commit'] = commit
  report['old_folder'] = str(replaced)
  os.sync()
  return report


def recover(runtime, updater, marker, marker_before, stamp, staged, replaced):
  """Every intermediate state is "new + marker present" or "old + state before"."""
  old = replaced if replaced.exists() else staged
  if marker_before is None:
    # a) marker back to a link (new updater installed: link resolves, present)
    temporary = marker.with_name(marker.name + '.switch-file-' + stamp)
    if os.path.lexists(temporary):
      os.rename(temporary, runtime / f'{temporary.name}.rolledback-{stamp}')
    if not is_our_link(marker):
      recovery_fail_point('relink')
      # A regular file written by this tool (same bytes as updater/switch-boot-marker)
      # is replaced by the link in one rename; the marker never disappears here.
      make_link(marker, stamp + '-rec')
    fsync_dir(runtime)
    crash_point('rec_after_relink')
    # b) exchange back: link dangles = state before the switch
    recovery_fail_point('exchange_back')
    exchange(updater, old)
    fsync_dir(runtime)
    crash_point('rec_after_exchange_back')
    # c) move the dangling link aside (never deleted)
    os.rename(marker, runtime / f'{marker.name}.rolledback-{stamp}')
    fsync_dir(runtime)
  else:
    base.restore_marker(marker, marker_before, stamp)
    exchange(updater, old)
    fsync_dir(runtime)


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('--root', default='/')
  parser.add_argument('--source', default=str(Path(__file__).resolve().parent))
  parser.add_argument('--manifest', required=False, help="comma's host_release.json (GitHub)")
  parser.add_argument('--python', default='/opt/carrot-jetlink/venv/bin/python')
  parser.add_argument('--boot-id', default='/proc/sys/kernel/random/boot_id')
  parser.add_argument('--lock-wait', type=float, default=120)
  parser.add_argument('--systemctl', default='systemctl', help='systemctl command used to hold the legacy server')
  parser.add_argument('--state', action='store_true', help='only print the current state')
  args = parser.parse_args()
  if args.state:
    state = base.boot_state(args.root)
    state['marker_link'] = (Path(args.root) / 'opt/carrot-jetlink/boot-update-required').is_symlink()
    print(json.dumps(state))
    return
  if not args.manifest:
    parser.error('--manifest required')
  if args.root == '/' and os.geteuid() != 0:
    raise PermissionError('root required')
  try:
    print(json.dumps(switch(args.root, args.source, args.manifest, args.python, args.boot_id,
                            lock_wait=args.lock_wait, systemctl=args.systemctl.split(),
                            proc='/proc'), indent=1))
  except ServerRestoreError as error:
    if error.phase == 'hold':
      print('HOLD NOT CONFIRMED, nothing of the updater changed:', error.switch_error, file=sys.stderr)
      print('SERVER RESTORE FAILED (not success):', error, file=sys.stderr)
      print(json.dumps({'server_hold': error.hold, 'boot_state': base.boot_state(args.root)}), file=sys.stderr)
      sys.exit(6)
    if error.report is not None:
      print(json.dumps(error.report, indent=1))
      print('SWITCHED BUT SERVER RESTORE FAILED (not success):', error, file=sys.stderr)
    else:
      print('FAILED:', error.switch_error, file=sys.stderr)
      print('SERVER RESTORE FAILED TOO:', error, file=sys.stderr)
    print(json.dumps({'server_hold': error.hold, 'boot_state': base.boot_state(args.root)}), file=sys.stderr)
    sys.exit(3 if error.report is not None else 4)
  except HoldAborted as error:
    print('FAILED (hold not confirmed, nothing of the updater changed, server restored):', error, file=sys.stderr)
    print(json.dumps({'server_hold': error.hold, 'boot_state': base.boot_state(args.root)}), file=sys.stderr)
    sys.exit(5)
  except SwitchError as error:
    print('FAILED:', error, file=sys.stderr)
    print(json.dumps(base.boot_state(args.root)), file=sys.stderr)
    sys.exit(2)


if __name__ == '__main__':
  main()
