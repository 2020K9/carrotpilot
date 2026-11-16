"""Start program for the Jetson updater: new updater first, legacy updater only as a fallback.

systemd (carrot-jetlink-update-apply.service / -stage.service, through the
drop-in written by install_launcher.py) runs this file instead of
/opt/carrot-jetlink/updater/update_host.py.  Folders:

  /opt/carrot-jetlink/updater/         new updater slot (switch_updater_linked.py
                                       owns it; legacy bootstrap() may rewrite it)
  /opt/carrot-jetlink/updater-legacy/  0cb67d46c updater, bytes unchanged,
                                       SHA256SUMS next to it (install_launcher.py)
  /opt/carrot-jetlink/launcher/        this file (nothing in 0cb67d46c writes here)

Order of one run:
  1. take /run/carrot-jetlink-update.lock (the flock every update_host.py action
     takes) and keep it until the end; children get the same open file, so no
     other updater can start between "new" and "legacy".
  2. run the new updater in its own process group with a time limit.
  3. classify (child result file, see CHILD_KINDS) and decide:
       ok / applied           -> done (exit 0)
       verify / rejected      -> update blocked, legacy NOT run (exit 20)
       busy                   -> another updater holds the lock, nothing run (exit 30)
       connect / unrunnable / error / crash / timeout
                              -> undo a half-done activation (current link from
                                 updates/transaction.json, same steps as
                                 update_host.activate recovery), confirm it,
                                 confirm the whole process group is gone, then
                                 run the legacy updater (exit 10 if it exited 0,
                                 40 if it failed).  10 means "legacy ran", NOT
                                 "updated": legacy only knows the NAS /models/
                                 host and the old key.
  Exit 41: activation state could not be restored -> legacy not run.
  Exit 42: legacy folder missing or SHA256SUMS mismatch -> legacy not run.
  Exit 43: process group of a timed-out child did not go away -> legacy not run.
  If the new slot holds exactly the legacy bytes (legacy bootstrap() rewrote it),
  it is run once as the legacy updater (exit 10/40), never twice.

State contract: <ROOT>/launcher/state.json (durable, phases start, new_running,
new_result, recovering, legacy_running, done) and /run/carrot-jetlink-launcher.json
(last result of this boot).  A run that finds a state.json not at 'done' records
it as 'previous_interrupted' and continues normally (update_host.activate already
recovers its own transaction.json at the next activate).

crash_point() is a test hook (os._exit = process kill, NOT a power cut).
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

EXIT_OK, EXIT_LEGACY_OK, EXIT_BLOCKED, EXIT_BUSY, EXIT_LEGACY_FAILED = 0, 10, 20, 30, 40
EXIT_RESTORE_FAILED, EXIT_LEGACY_MISSING, EXIT_KILL_UNCONFIRMED = 41, 42, 43
FALLBACK_KINDS = {'connect', 'unrunnable', 'error', 'crash', 'timeout'}
BLOCK_KINDS = {'verify', 'rejected'}
LOCK = 'run/carrot-jetlink-update.lock'
RUN_RESULT = 'run/carrot-jetlink-launcher.json'
DEFAULT_TIMEOUT = {'activate': 600, 'automatic': 900, 'stage': 900, 'boot': 900}


def crash_point(name):
  if os.environ.get('JET_LAUNCHER_CRASH_AT') == name:
    os._exit(137)


def sha(path):
  return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_durable(path, text):
  path = Path(path)
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_name(path.name + '.tmp')
  with temporary.open('w') as stream:
    stream.write(text)
    stream.flush()
    os.fsync(stream.fileno())
  os.replace(temporary, path)
  fd = os.open(str(path.parent), os.O_RDONLY)
  try:
    os.fsync(fd)
  finally:
    os.close(fd)


def read_sums(folder):
  sums = {}
  for line in (Path(folder) / 'SHA256SUMS').read_text().splitlines():
    if line.strip():
      digest, name = line.split(None, 1)
      sums[name.strip()] = digest
  return sums


def legacy_intact(folder):
  try:
    sums = read_sums(folder)
    return bool(sums) and 'update_host.py' in sums and all(sha(Path(folder) / n) == d for n, d in sums.items())
  except (OSError, ValueError):
    return False


def slot_is_legacy(slot, legacy):
  try:
    sums = read_sums(legacy)
    return all((Path(slot) / n).is_file() and sha(Path(slot) / n) == d for n, d in sums.items())
  except (OSError, ValueError):
    return False


# ---------------------------------------------------------------- child side
def classify(error):
  import http.client
  import socket
  import ssl
  import urllib.error
  if isinstance(error, BlockingIOError):
    return 'busy'
  if isinstance(error, (urllib.error.URLError, http.client.HTTPException, socket.timeout, TimeoutError,
                        ConnectionError, ssl.SSLError)):
    return 'connect'
  if isinstance(error, (ImportError, SyntaxError)):
    return 'unrunnable'
  if isinstance(error, (ValueError, subprocess.CalledProcessError)):
    return 'verify'  # signature, sha/length, manifest, identity, probe of the candidate
  return 'error'


def child_main(script, action, result_path, lock_fd, update_root):
  """Runs inside the child: update_host.py <action> with the inherited lock."""
  import runpy
  held = os.fstat(lock_fd)
  real_flock = fcntl.flock

  def flock(fd, operation):
    number = fd if isinstance(fd, int) else fd.fileno()
    st = os.fstat(number)
    if (st.st_dev, st.st_ino) == (held.st_dev, held.st_ino):
      return None  # the start program already holds this lock for us
    return real_flock(fd, operation)

  fcntl.flock = flock
  status = Path(update_root) / 'updates/status.json'
  started = time.time()  # noqa: TID251 -- compared with the wall-clock 'updated' stamp the updater writes
  result = {'kind': 'ok'}
  code = 0
  try:
    if not Path(script).is_file():
      raise ImportError('updater missing: ' + str(script))
    sys.path.insert(0, str(Path(script).parent))
    sys.argv = [str(script), action]
    runpy.run_path(str(script), run_name='__main__')
  except SystemExit as stop:
    if stop.code not in (None, 0):
      result, code = {'kind': 'unrunnable', 'error': f'SystemExit {stop.code!r}'}, 2
  except BaseException as error:
    result, code = {'kind': classify(error), 'error': f'{type(error).__name__}: {str(error)[:200]}'}, 1
  if result['kind'] == 'ok':
    try:
      value = json.loads(status.read_text())
      if value.get('updated', 0) >= started - 1:
        if value.get('state') == 'rejected':
          result = {'kind': 'rejected', 'error': str(value.get('error', ''))[:200]}
        elif value.get('state') == 'applied':
          result = {'kind': 'applied'}
    except (OSError, ValueError):
      pass
  write_durable(result_path, json.dumps(result))
  return code


# --------------------------------------------------------------- parent side
def boot_id(root):
  path = Path(root) / 'proc/sys/kernel/random/boot_id'
  if not path.is_file():
    path = Path('/proc/sys/kernel/random/boot_id')
  return path.read_text().strip()


def group_alive(pgid):
  for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():
      continue
    try:
      fields = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
    except OSError:
      continue
    if fields[0] != 'Z' and (int(fields[2]) == pgid or int(fields[3]) == pgid):
      return True
  return False


def stop_group(process, pgid, grace=5.0, confirm=10.0):
  for sig, wait in ((signal.SIGTERM, grace), (signal.SIGKILL, confirm)):
    try:
      os.killpg(pgid, sig)
    except ProcessLookupError:
      pass
    deadline = time.monotonic() + wait
    while time.monotonic() < deadline:
      process.poll()
      if process.returncode is not None and not group_alive(pgid):
        return True
      time.sleep(0.05)
  process.poll()
  return process.returncode is not None and not group_alive(pgid)


def run_child(root, script, action, lock_fd, timeout, tag, python):
  result_path = Path(root) / 'run' / (f'carrot-jetlink-launcher-{tag}.json')
  try:
    result_path.unlink()
  except FileNotFoundError:
    pass
  env = dict(os.environ)
  command = [python, str(Path(__file__).resolve()), '--child', str(script), action, str(result_path),
             str(lock_fd), str(Path(root) / 'opt/carrot-jetlink')]
  process = subprocess.Popen(command, pass_fds=(lock_fd,), start_new_session=True, env=env)
  pgid = process.pid
  try:
    process.wait(timeout=timeout)
  except subprocess.TimeoutExpired:
    if not stop_group(process, pgid):
      return {'kind': 'timeout', 'group_stopped': False}
    return {'kind': 'timeout', 'group_stopped': True}
  # Normal exit: anything it left in its group must also be gone.
  if group_alive(pgid) and not stop_group(process, pgid):
    return {'kind': 'crash', 'group_stopped': False, 'returncode': process.returncode}
  try:
    value = json.loads(result_path.read_text())
  except (OSError, ValueError):
    value = {'kind': 'crash'}
  value['returncode'] = process.returncode
  value['group_stopped'] = True
  return value


def restore_activation(update_root):
  """Same steps as update_host.activate() transaction recovery.  Returns (ok, note)."""
  update_root = Path(update_root)
  transaction = update_root / 'updates/transaction.json'
  pending = update_root / 'updates/pending.json'
  if not transaction.exists():
    return True, 'no transaction'
  try:
    saved = json.loads(transaction.read_text())
    old = Path(saved['release'])
    releases = (update_root / 'releases').resolve()
    if old.is_symlink() or old.resolve().parent != releases or not old.is_dir():
      return False, 'invalid recovery release'
    link = update_root / 'current.recovery'
    if link.is_symlink() or link.exists():
      link.unlink()
    link.symlink_to(old)
    crash_point('restore_before_replace')
    os.replace(link, update_root / 'current')
    if saved.get('last_loaded') is not None:
      write_durable(update_root / 'cache/last-loaded.json', saved['last_loaded'])
    if pending.exists():
      os.replace(pending, pending.with_name('interrupted.json'))
    os.sync()
    if (update_root / 'current').resolve() != old.resolve():
      return False, 'current did not move back'
    crash_point('restore_before_unlink')
    transaction.unlink()
    write_durable(update_root / 'updates/status.json', json.dumps({'state': 'recovered', 'updated': time.time()}))  # noqa: TID251 -- persisted wall-clock stamp, same field as the updater
    os.sync()
    return True, 'restored ' + old.name
  except (OSError, ValueError, KeyError, TypeError) as error:
    return False, f'{type(error).__name__}: {error}'


def launch(root, action, python=sys.executable, new_timeout=None, legacy_timeout=None):
  root = Path(root)
  update_root = root / 'opt/carrot-jetlink'
  slot, legacy = update_root / 'updater', update_root / 'updater-legacy'
  state_path = update_root / 'launcher/state.json'
  new_timeout = new_timeout or DEFAULT_TIMEOUT.get(action, 900)
  legacy_timeout = legacy_timeout or DEFAULT_TIMEOUT.get(action, 900)
  report = {'action': action, 'boot_id': boot_id(root), 'started': time.time()}  # noqa: TID251 -- persisted wall-clock start time

  def state(phase, **extra):
    report.update(extra)
    report['phase'] = phase
    write_durable(state_path, json.dumps(report))

  def finish(code, **extra):
    report.update(extra)
    report['exit'] = code
    state('done')
    write_durable(root / RUN_RESULT, json.dumps(report))
    return code

  lock_path = root / LOCK
  lock_path.parent.mkdir(parents=True, exist_ok=True)
  lock = lock_path.open('a')
  try:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
  except BlockingIOError:
    # Do not touch state.json: the holder may be another launcher writing it.
    report.update(exit=EXIT_BUSY, result='busy: lock held')
    write_durable(root / RUN_RESULT, json.dumps(report))
    return EXIT_BUSY
  try:
    try:
      previous = json.loads(state_path.read_text())
      if previous.get('phase') != 'done':
        report['previous_interrupted'] = {'phase': previous.get('phase'), 'boot_id': previous.get('boot_id')}
    except (OSError, ValueError):
      pass
    current = update_root / 'current'
    before = str(current.resolve()) if current.exists() else None
    state('start', current_before=before)
    crash_point('after_start')

    if slot_is_legacy(slot, legacy):
      # bootstrap() of a legacy runtime rewrote the new slot: run it once, as legacy.
      return run_legacy(root, slot, action, lock, legacy_timeout, python, state, finish, legacy, 'new slot holds legacy bytes')

    state('new_running')
    crash_point('new_running')
    new = run_child(root, slot / 'update_host.py', action, lock.fileno(), new_timeout, 'new', python)
    state('new_result', new=new)
    crash_point('new_result')
    kind = new.get('kind')
    if kind in ('ok', 'applied'):
      return finish(EXIT_OK, result='new updater ' + kind)
    if kind in BLOCK_KINDS:
      return finish(EXIT_BLOCKED, result='blocked: ' + kind + ' (legacy not run)')
    if kind == 'busy':
      return finish(EXIT_BUSY, result='busy inside new updater (legacy not run)')
    if not new.get('group_stopped', True):
      return finish(EXIT_KILL_UNCONFIRMED, result='new updater group still alive (legacy not run)')
    state('recovering')
    ok, note = restore_activation(update_root)
    report['restore'] = note
    after = str(current.resolve()) if current.exists() else None
    if not ok:
      return finish(EXIT_RESTORE_FAILED, result='restore failed (legacy not run)', current_after=after)
    if note == 'no transaction' and after != before:
      status = {}
      try:
        status = json.loads((update_root / 'updates/status.json').read_text())
      except (OSError, ValueError):
        pass
      if status.get('state') == 'applied':
        return finish(EXIT_OK, result='new updater applied, then failed after the switch', current_after=after)
      return finish(EXIT_RESTORE_FAILED, result='current changed without transaction (legacy not run)', current_after=after)
    crash_point('before_legacy')
    return run_legacy(root, legacy, action, lock, legacy_timeout, python, state, finish, legacy, 'new failed: ' + kind)
  finally:
    lock.close()


def run_legacy(root, folder, action, lock, timeout, python, state, finish, legacy, why):
  if not legacy_intact(legacy):
    return finish(EXIT_LEGACY_MISSING, result=why + '; legacy folder missing or changed (not run)')
  state('legacy_running', legacy_reason=why)
  crash_point('legacy_running')
  old = run_child(root, Path(folder) / 'update_host.py', action, lock.fileno(), timeout, 'legacy', python)
  if old.get('kind') in ('ok', 'applied') and old.get('returncode') == 0:
    return finish(EXIT_LEGACY_OK, legacy=old,
                  result=why + '; legacy updater ran (NAS /models/ only; ran != updated)')
  return finish(EXIT_LEGACY_FAILED, legacy=old, result=why + '; legacy updater failed: ' + str(old.get('kind')))


def main(argv=None):
  argv = sys.argv[1:] if argv is None else argv
  if argv[:1] == ['--child']:
    script, action, result_path, lock_fd, update_root = argv[1:6]
    return child_main(script, action, result_path, int(lock_fd), update_root)
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument('action', choices=['stage', 'activate', 'automatic', 'boot'])
  parser.add_argument('--root', default='/')
  parser.add_argument('--new-timeout', type=float)
  parser.add_argument('--legacy-timeout', type=float)
  args = parser.parse_args(argv)
  code = launch(args.root, args.action, new_timeout=args.new_timeout, legacy_timeout=args.legacy_timeout)
  print(Path(args.root, RUN_RESULT).read_text(), flush=True)
  return code


if __name__ == '__main__':
  sys.exit(main())
