"""One-time switch of an installed legacy (NAS) Jetson updater to the GitHub updater.

Run once as root on a parked Jetson, from an unpacked copy of tools/jetlink of the
new release (063613a1 line).  Steps, each durable before the next:

  1. backup   copy updater/, boot-update-required and update unit files to
              <runtime>/updater-switch-backup/<stamp>/ (never overwritten)
  2. stage    write the complete new updater to <runtime>/updater.switch-<stamp>/
              (old extra files kept, new files on top, every file fsync'd)
  3. pre-check  the staged updater must accept the comma's GitHub manifest
  4. exchange renameat2(RENAME_EXCHANGE) swaps updater/ and the staged folder in
              one atomic step
  5. marker   immediately after the exchange (same directory, one fsync for
              both) boot-update-required is created if missing, with this
              boot's id.  Without it the legacy runtime's
              migrate_running_release() reinstalls the old updater at the next
              server start and arms the gate (old updater + gate = stuck).
  6. verify   the old folder becomes updater.replaced-<stamp>; the installed
              updater must accept the manifest again (new key, GitHub hosts)
              and every file must match the payload hash

Failure after the exchange first puts the marker back to its original state
(original content, or a new marker moved aside as
boot-update-required.rolledback-<stamp>), then swaps the old folder back,
re-checks it against the backup and imports it.  The combination "old updater +
marker written by this tool" is never produced.  Residual window: between the
exchange and the marker rename a killed process leaves new updater + no marker;
see docs/jetson_updater_switch_20261116.md.
"""
import argparse
import ctypes
import ctypes.util
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

NEW_KEY = 'MCowBQYDK2VwAyEAfGh+KGAlwr+DkGLiy1xu4GFNfU/sZW3YGsy3dIVmzv0='
OLD_KEY = 'MCowBQYDK2VwAyEAEhNI8j6VWOnj0/JAD+CbNLSnJVZUzCgqWmE57QO8YOw='
NAMES = ['finalize_sd_image.py', 'hud_protocol.py', 'release-signing-public.pem', 'wifi_protocol.py',
         'local_release.py', 'usb_release.py', 'boot_update.py', 'update_host.py']
REQUIRED = [n for n in NAMES if n != 'usb_release.py']
UNITS = ['carrot-jetlink-update-apply.service', 'carrot-jetlink-update-stage.service',
         'carrot-jetlink-update-stage.timer']
UPDATER_COMMAND = '/opt/carrot-jetlink/updater/update_host.py'
RENAME_EXCHANGE = 2
AT_FDCWD = -100


class SwitchError(RuntimeError):
  pass


def crash_point(name):
  # Test only: simulates a forced process kill (os._exit, no cleanup).  This is
  # NOT a power-cut test: data already written but not yet on disk survives a
  # process kill, so it proves nothing about what a real power loss keeps.
  if os.environ.get('JET_SWITCH_CRASH_AT') == name:
    os._exit(91)


def fail_point(name):
  # Test only: injected I/O error at a named point (exercises the recovery path).
  if os.environ.get('JET_SWITCH_FAIL_AT') == name:
    raise OSError(5, 'injected I/O error at ' + name)


def sha(path):
  return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def key_of(pem_path):
  lines = [line.strip() for line in Path(pem_path).read_text().splitlines()]
  return ''.join(line for line in lines if line and not line.startswith('-----'))


def fsync_dir(path):
  fd = os.open(str(path), os.O_RDONLY)
  try:
    os.fsync(fd)
  finally:
    os.close(fd)


def write_durable(path, data):
  with open(path, 'wb') as stream:
    stream.write(data)
    stream.flush()
    os.fsync(stream.fileno())


def exchange(a, b):
  libc = ctypes.CDLL(ctypes.util.find_library('c') or None, use_errno=True)
  renameat2 = getattr(libc, 'renameat2', None)
  if renameat2 is None:
    raise SwitchError('renameat2 unavailable; nothing changed')
  if renameat2(AT_FDCWD, os.fsencode(str(a)), AT_FDCWD, os.fsencode(str(b)), RENAME_EXCHANGE) != 0:
    errno = ctypes.get_errno()
    raise SwitchError(f'RENAME_EXCHANGE failed: {os.strerror(errno)}')
  # No fsync here: the caller syncs inside its recovery scope.


def write_marker(marker, boot_id_path):
  temporary = marker.with_suffix('.switch-new')
  write_durable(temporary, Path(boot_id_path).read_bytes())
  crash_point('after_marker_tmp')
  fail_point('marker_rename')
  os.replace(temporary, marker)
  crash_point('after_marker_rename')
  fail_point('marker_fsync')
  fsync_dir(marker.parent)


def restore_marker(marker, marker_before, stamp):
  """Put boot-update-required back to its state before the switch.
  Never deletes: a marker this tool created is moved aside."""
  runtime = marker.parent
  temporary = marker.with_suffix('.switch-new')
  if temporary.exists():
    os.rename(temporary, runtime / f'{temporary.name}.rolledback-{stamp}')
  if marker_before is None:
    if marker.exists():
      os.rename(marker, runtime / f'{marker.name}.rolledback-{stamp}')
  elif not marker.is_file() or marker.read_bytes() != marker_before:
    restore = marker.with_suffix('.switch-restore')
    write_durable(restore, marker_before)
    os.replace(restore, marker)
  fsync_dir(runtime)


def tree_hashes(folder):
  return {p.name: sha(p) for p in sorted(Path(folder).iterdir()) if p.is_file()}


def verify_updater(python, updater, root, manifest_path, expect_key):
  """Import the updater in a fresh process exactly as the service would and
  let it validate the manifest (format, hosts, Ed25519 signature, storage)."""
  missing = [n for n in REQUIRED if not (Path(updater) / n).is_file()]
  if missing:
    raise SwitchError(f'updater incomplete: {missing}')
  if key_of(Path(updater) / 'release-signing-public.pem') != expect_key:
    raise SwitchError('updater public key is not the expected key')
  code = ('import json,sys;from pathlib import Path;sys.path.insert(0,sys.argv[1]);'
          'import update_host as u, boot_update;'
          'u.PROTECTED_MARKER=Path(sys.argv[2])/"etc/carrot-jetlink-protected.json";'
          'm=json.loads(Path(sys.argv[3]).read_text());print(u.selected_commit(m))')
  result = subprocess.run([python, '-I', '-c', code, str(updater), str(root), str(manifest_path)],
                          capture_output=True, text=True, timeout=60, cwd=str(updater))
  if result.returncode != 0:
    raise SwitchError('manifest rejected: ' + (result.stderr.strip().splitlines() or ['?'])[-1][:200])
  return result.stdout.strip()


def check_old_updater(python, updater, backup):
  """After recovery: identical to the backup and importable with its NAS rule."""
  if tree_hashes(updater) != tree_hashes(backup / 'updater'):
    raise SwitchError('restored updater differs from backup')
  code = ('import sys;sys.path.insert(0,sys.argv[1]);import update_host as u;'
          'u.checked_url("https://upload.shind0.synology.me/models/x");print("ok")')
  result = subprocess.run([python, '-I', '-c', code, str(updater)], capture_output=True, text=True,
                          timeout=60, cwd=str(updater))
  if result.returncode != 0:
    raise SwitchError('restored updater does not import/accept its NAS host')


def switch(root, source, manifest_path, python, boot_id_path, stamp=None):
  root, source, manifest_path = Path(root), Path(source), Path(manifest_path)
  runtime = root / 'opt/carrot-jetlink'
  updater = runtime / 'updater'
  marker = runtime / 'boot-update-required'
  units = root / 'etc/systemd/system'
  stamp = stamp or time.strftime('%Y%m%d-%H%M%S')
  report = {'stamp': stamp}

  # Preflight: read-only.
  if not updater.is_dir() or updater.is_symlink():
    raise SwitchError('no installed updater folder')
  current_key = key_of(updater / 'release-signing-public.pem')
  if current_key == NEW_KEY:
    report['selected_commit'] = verify_updater(python, updater, root, manifest_path, NEW_KEY)
    if not marker.is_file():
      # Power was cut after the exchange but before step 6: finish it, or the
      # legacy runtime would reinstall the old updater at its next start.
      write_marker(marker, boot_id_path)
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
  verify_updater(python, source, root, manifest_path, NEW_KEY)  # payload itself
  marker_before = marker.read_bytes() if marker.is_file() else None
  report['marker_before'] = None if marker_before is None else marker_before.decode(errors='replace').strip()

  # 1. backup
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
          'updater_sha256': tree_hashes(updater)}
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

  # 2. stage
  staged = runtime / ('updater.switch-' + stamp)
  if staged.exists():
    raise SwitchError('staging folder already exists')
  staged.mkdir()
  for item in updater.iterdir():
    if item.is_file() and item.name not in NAMES:
      write_durable(staged / item.name, item.read_bytes())
  payload = [n for n in NAMES if (source / n).is_file()]
  for i, name in enumerate(payload):
    write_durable(staged / name, (source / name).read_bytes())
    (staged / name).chmod(0o644)
    if i == len(payload) // 2:
      crash_point('mid_stage')
  staged.chmod(updater.stat().st_mode & 0o7777)
  fsync_dir(staged)
  fsync_dir(runtime)
  crash_point('after_stage')

  # 3. pre-check before anything visible changes
  verify_updater(python, staged, root, manifest_path, NEW_KEY)

  # 4. atomic exchange (raises before any change if renameat2 is unavailable)
  exchange(updater, staged)  # staged path now holds the old updater
  crash_point('after_exchange')
  replaced = runtime / ('updater.replaced-' + stamp)
  try:
    fail_point('fsync_after_exchange')
    # 5. marker right away: shortest window in which the legacy runtime's
    # migrate_running_release() would reinstall the old updater.
    if marker_before is None:
      write_marker(marker, boot_id_path)
      report['marker_written'] = 'this-boot-id (gate from next boot)'
    fsync_dir(runtime)  # exchange + marker durable together (same directory)
    crash_point('after_marker')
    # 6. keep old folder under its final name, verify installed updater
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
    # Recovery: marker first (so "old updater + new marker" never exists, even
    # if this recovery is itself killed), then the same atomic exchange back.
    restore_marker(marker, marker_before, stamp)
    old = replaced if replaced.exists() else staged
    exchange(updater, old)
    fsync_dir(runtime)
    check_old_updater(python, updater, backup)
    report['result'] = 'restored-old'
    report['error'] = str(error)[:240]
    raise SwitchError('switch failed and old updater restored: ' + str(error)[:200]) from error
  report['result'] = 'switched'
  report['selected_commit'] = commit
  report['old_folder'] = str(replaced)
  os.sync()
  return report


def boot_state(root):
  """Read-only check usable after a power cut: what will the next boot run?"""
  runtime = Path(root) / 'opt/carrot-jetlink'
  updater = runtime / 'updater'
  key = key_of(updater / 'release-signing-public.pem')
  complete = all((updater / n).is_file() for n in REQUIRED)
  return {'key': 'new' if key == NEW_KEY else 'old' if key == OLD_KEY else 'unknown',
          'complete': complete, 'marker': (runtime / 'boot-update-required').is_file()}


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('--root', default='/')
  parser.add_argument('--source', default=str(Path(__file__).resolve().parent))
  parser.add_argument('--manifest', required=False, help="comma's host_release.json (GitHub)")
  parser.add_argument('--python', default='/opt/carrot-jetlink/venv/bin/python')
  parser.add_argument('--boot-id', default='/proc/sys/kernel/random/boot_id')
  parser.add_argument('--state', action='store_true', help='only print the current state')
  args = parser.parse_args()
  if args.state:
    print(json.dumps(boot_state(args.root)))
    return
  if not args.manifest:
    parser.error('--manifest required')
  if args.root == '/' and os.geteuid() != 0:
    raise PermissionError('root required')
  try:
    print(json.dumps(switch(args.root, args.source, args.manifest, args.python, args.boot_id), indent=1))
  except SwitchError as error:
    print('FAILED:', error, file=sys.stderr)
    print(json.dumps(boot_state(args.root)), file=sys.stderr)
    sys.exit(2)


if __name__ == '__main__':
  main()
