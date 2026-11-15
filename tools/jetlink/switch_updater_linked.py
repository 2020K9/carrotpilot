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


def switch(root, source, manifest_path, python, boot_id_path, stamp=None, lock_wait=120):
  root = Path(root)
  lock = take_lock(root, lock_wait)
  try:
    crash_point('after_lock')
    return _switch(root, Path(source), Path(manifest_path), python, boot_id_path,
                   stamp or time.strftime('%Y%m%d-%H%M%S'))
  finally:
    lock.close()


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
    if item.is_file() and item.name not in NAMES and item.name != INNER:
      write_durable(staged / item.name, item.read_bytes())
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
                            lock_wait=args.lock_wait), indent=1))
  except SwitchError as error:
    print('FAILED:', error, file=sys.stderr)
    print(json.dumps(base.boot_state(args.root)), file=sys.stderr)
    sys.exit(2)


if __name__ == '__main__':
  main()
