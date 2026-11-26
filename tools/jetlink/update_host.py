"""Verified 2020K9 GitHub Releases runtime/model updates; activation only with both services stopped.

Migrated hosts check the attached comma's pin before first inference, including
onroad startup; a separate USB-only gate waits for Internet and applies updates.
While running (also onroad) the timer pre-downloads a newer pin; only the next
boot's gate activates it. Legacy hosts retain offroad staging until migration.
Failed probes preserve the old release; the new gate still withholds execution.
No OS, user identity, network credentials or vehicle settings are updated.
"""
import argparse
import base64
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time
import urllib.parse
import urllib.request

ROOT = Path('/opt/carrot-jetlink')
PROTECTED_MARKER = Path('/etc/carrot-jetlink-protected.json')
DEFAULT_MANIFEST = 'https://github.com/2020K9/carrotpilot/releases/download/k9-jetlink-host-stable/manifest.json'
# Only 2020K9/carrotpilot release assets. GitHub answers with one redirect to
# its asset store; accept that store only for this repository id (1400258213).
RELEASE_HOST, RELEASE_PREFIX = 'github.com', '/2020K9/carrotpilot/releases/download/'
ASSET_HOST, ASSET_PREFIX = 'release-assets.githubusercontent.com', '/github-production-release-asset/1400258213/'
# One shared runtime bundle serves every catalog model: the manifest selects the
# model, the bundle carries all committed specs. install_updates.py refreshes an
# installed updater that predates this (it would tie a bundle to its pin).
MODEL_SELECTION = 1
PIN = 'openpilot/selfdrive/modeld/jetlink/jetson_model.json'
CATALOG = 'openpilot/selfdrive/modeld/model_catalog'


def verify_signature(manifest):
  from Crypto.PublicKey import ECC
  from Crypto.Signature import eddsa
  value = {k: v for k, v in manifest.items() if k != 'signature'}
  payload = json.dumps(value, sort_keys=True, separators=(',', ':')).encode()
  key = ECC.import_key(Path(__file__).with_name('release-signing-public.pem').read_text())
  eddsa.new(key, 'rfc8032').verify(payload, base64.b64decode(manifest.get('signature', ''), validate=True))


def digest(path):
  value = hashlib.sha256()
  with path.open('rb') as source:
    for block in iter(lambda: source.read(4 << 20), b''):
      value.update(block)
  return value.hexdigest()


def atomic_json(path, value):
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_suffix('.tmp')
  with temporary.open('w') as target:
    json.dump(value, target)
    target.flush()
    os.fsync(target.fileno())
  os.replace(temporary, path)


def checked_url(url, redirected=False):
  parsed = urllib.parse.urlsplit(url)
  allowed = [(RELEASE_HOST, RELEASE_PREFIX)] + ([(ASSET_HOST, ASSET_PREFIX)] if redirected else [])
  if (parsed.scheme != 'https' or not any(parsed.netloc == h and parsed.path.startswith(p) for h, p in allowed) or
      '..' in parsed.path.split('/') or '%' in parsed.path):
    raise ValueError('Updates require the configured HTTPS 2020K9 GitHub release host')
  return url


def fetch(url, path, sha256, size, progress=None):
  checked_url(url)
  if not re.fullmatch('[0-9a-f]{64}', sha256) or not isinstance(size, int) or not 0 < size <= 4 << 30:
    raise ValueError('Invalid download identity')
  if path.exists() and path.stat().st_size == size and digest(path) == sha256:
    return
  temporary = path.with_suffix('.part')
  try:
    with urllib.request.urlopen(url, timeout=45) as response, temporary.open('wb') as output:
      checked_url(response.url, redirected=True)
      count = 0
      for block in iter(lambda: response.read(1 << 20), b''):
        count += len(block)
        if count > size:
          raise ValueError('Oversized download')
        output.write(block)
        if progress is not None:
          progress(count / size)
      output.flush()
      os.fsync(output.fileno())
    if count != size or digest(temporary) != sha256:
      raise ValueError('Update checksum/length mismatch')
    os.replace(temporary, path)
  finally:
    temporary.unlink(missing_ok=True)


def validate_manifest(manifest):
  if manifest.get('format') != 1 or not re.fullmatch('[0-9a-f]{40}', str(manifest.get('source_commit', ''))):
    raise ValueError('Unsupported release manifest')
  if manifest.get('runtime') != {'arch': 'aarch64', 'l4t': '36.4.7', 'tensorrt': '10.3.0'}:
    raise ValueError('This updater cannot change the OS/runtime ABI')
  if not re.fullmatch('[0-9a-f]{64}', str(manifest.get('model', {}).get('sha256', ''))):
    raise ValueError('Model identity required')
  checked_url(manifest['bundle']['url'])
  checked_url(manifest['model']['url'])


def validate_storage_compatibility(manifest):
  # An older comma may still select the last writable-image release. Never
  # silently replace a protected runtime with code predating its storage contract.
  if PROTECTED_MARKER.exists() and manifest.get('storage_format') != 1:
    raise ValueError('Selected release does not support protected storage; keeping current runtime')


def probe_release(release, model=None):
  command = ['runuser', '-u', 'jetlink', '--', str(ROOT / 'venv/bin/python'),
             str(release / 'tools/jetlink/probe_release.py'), str(ROOT / 'cache')]
  if model is not None:
    command += [model['sha256'], str(model['size'])]
  process = subprocess.Popen(command, start_new_session=True)
  try:
    code = process.wait(timeout=960)
  except BaseException:
    # Killing only runuser can leave its GPU-building Python child alive.
    # Reap the entire isolated probe group before the old server may start.
    try:
      os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
      pass
    process.wait()
    raise
  if code:
    raise subprocess.CalledProcessError(code, command)


def stage(manifest_url=DEFAULT_MANIFEST):
  checked_url(manifest_url)
  with urllib.request.urlopen(manifest_url, timeout=30) as response:
    checked_url(response.url, redirected=True)
    manifest = json.loads(response.read(65537))
  stage_manifest(manifest)


def check_runtime(manifest):
  l4t = subprocess.check_output(['dpkg-query', '-W', '-f=${Version}', 'nvidia-l4t-core'], text=True)
  if not l4t.startswith('36.4.7-'):
    raise ValueError('Installed L4T does not match the release')
  runtime = subprocess.check_output([str(ROOT / 'venv/bin/python'), '-c', 'import tensorrt; print(tensorrt.__version__)'], text=True).strip()
  if runtime != manifest['runtime']['tensorrt'] or os.uname().machine != 'aarch64':
    raise ValueError('Installed TensorRT/architecture does not match')


def release_spec(release, model):
  """The committed spec in this release for the manifest's model, or None.

  Older bundles carry only their pin; shared bundles also carry every
  model_catalog spec. Only exact sha256 + size identity selects a spec.
  """
  release = Path(release)
  identity = (model.get('sha256'), model.get('size'))
  candidates = [release / PIN, release / PIN.replace('jetson_model.json', 'cinque_v2.json')]
  catalog = release / CATALOG
  if catalog.is_dir():
    candidates += sorted(p for p in catalog.glob('*.json')
                         if not p.name.endswith('-release.json') and p.name != 'catalog.json')
  for path in candidates:
    try:
      if path.is_symlink() or not path.is_file():
        continue
      spec = json.loads(path.read_text())
      if isinstance(spec, dict) and (spec.get('sha256'), spec.get('nbytes')) == identity:
        return spec
    except (OSError, ValueError):
      continue
  return None


def check_bundle(candidate, manifest):
  if (candidate / 'SOURCE_COMMIT').read_text().strip() != manifest['source_commit']:
    raise ValueError('Bundle source identity mismatch')
  if release_spec(candidate, manifest['model']) is None:
    raise ValueError('Bundle model contract differs from manifest')


def pin_matches(release, model):
  for name in ('jetson_model.json', 'cinque_v2.json'):
    pin = Path(release) / PIN.replace('jetson_model.json', name)
    try:
      if pin.is_file():
        spec = json.loads(pin.read_text())
        return (spec['sha256'], spec['nbytes']) == (model['sha256'], model['size'])
    except (OSError, ValueError, KeyError, TypeError):
      return False
  return True  # No pin at all: nothing to select; keep the original probe call.


def model_identity(manifest):
  model = manifest['model']
  return {'sha256': model['sha256'], 'size': model['size']}


def active_model():
  """Model the current release was last activated/probed with (None if unknown)."""
  try:
    value = json.loads((ROOT / 'updates/active.json').read_text())
    if value['source_commit'] == (ROOT / 'current/SOURCE_COMMIT').read_text().strip():
      return value['model']
  except (OSError, ValueError, KeyError, TypeError):
    pass
  # Hosts activated by an updater predating MODEL_SELECTION: the per-model
  # bundle's own pin is the model it was activated with.
  try:
    spec = json.loads((ROOT / 'current' / PIN).read_text())
    return {'sha256': spec['sha256'], 'size': spec['nbytes']}
  except (OSError, ValueError, KeyError, TypeError):
    return None


def is_active(manifest):
  """Same runtime commit and same model; a shared bundle alone is not enough."""
  try:
    current = (ROOT / 'current/SOURCE_COMMIT').read_text().strip()
  except OSError:
    return False
  if current != manifest['source_commit']:
    return False
  if manifest.get('delivery') in ('usb', 'local') or 'model' not in manifest:
    return True
  active = active_model()
  # Unknown (no record and no pin): the commit identity alone, as before.
  return active is None or active == model_identity(manifest)


def model_path(model):
  return ROOT / 'cache/models' / (model['sha256'][:16] + '.onnx')


def prune():
  """Keep only the active and the previous runtime/model; never the OS or the base image.

  Called after a successful activation (and once by a refreshed updater).
  Anything an interrupted transaction or a staged update still names is kept.
  """
  releases = (ROOT / 'releases').resolve()
  keep_releases, keep_models = set(), set()

  def remember(record):
    if not isinstance(record, dict):
      return
    if isinstance(record.get('release'), str):
      keep_releases.add(Path(record['release']).name)
    for key in ('source_commit', 'previous_release'):
      if isinstance(record.get(key), str):
        keep_releases.add(record[key])
    if isinstance(record.get('last_loaded'), str):  # previous.json / transaction.json
      try:
        remember(json.loads(record['last_loaded']))
      except ValueError:
        pass
    if isinstance(record.get('sha256'), str):  # cache/last-loaded.json shape
      keep_models.add(record['sha256'][:16])
    model = record.get('model')
    if isinstance(model, dict) and isinstance(model.get('sha256'), str):
      keep_models.add(model['sha256'][:16])

  try:
    keep_releases.add((ROOT / 'current').resolve().name)
  except OSError:
    return
  for path in [ROOT / 'updates' / name for name in
               ('active.json', 'previous-active.json', 'previous.json', 'transaction.json', 'pending.json')] + [
               ROOT / 'cache/last-loaded.json']:
    try:
      remember(json.loads(path.read_text()))
    except (OSError, ValueError):
      pass
  current = active_model()
  if current is not None:
    keep_models.add(current['sha256'][:16])
  try:
    preloaded = json.loads(PROTECTED_MARKER.read_text()).get('preloaded_model')
    if isinstance(preloaded, dict) and isinstance(preloaded.get('sha256'), str):
      keep_models.add(preloaded['sha256'][:16])
  except (OSError, ValueError, AttributeError):
    pass
  import shutil
  if releases.is_dir():
    for path in releases.iterdir():
      if (re.fullmatch('[0-9a-f]{40}', path.name) and not path.is_symlink() and path.is_dir()
          and path.name not in keep_releases and path.resolve().parent == releases):
        shutil.rmtree(path)
  updates = ROOT / 'updates'
  if updates.is_dir():
    for path in updates.glob('*.tar.gz'):
      name = path.name[:-len('.tar.gz')]
      if re.fullmatch('[0-9a-f]{40}', name) and not path.is_symlink() and name not in keep_releases:
        path.unlink()
  models = ROOT / 'cache/models'
  if models.is_dir() and keep_models:
    for path in models.glob('*.onnx'):
      if (re.fullmatch('[0-9a-f]{16}', path.stem) and not path.is_symlink()
          and path.stem not in keep_models):
        path.unlink()
  os.sync()


def record_active(manifest, previous_model, previous_release=None):
  """Remember the activated runtime+model and the ones before it (for prune)."""
  try:
    before = json.loads((ROOT / 'updates/active.json').read_text())
  except (OSError, ValueError):
    before = {}
  if previous_model is not None and previous_model != model_identity(manifest):
    atomic_json(ROOT / 'updates/previous-active.json',
                {'source_commit': before.get('source_commit'), 'model': previous_model})
  # A model-only switch keeps the shared folder; the previous runtime folder
  # stays the last different commit.
  if previous_release is None or previous_release == manifest['source_commit']:
    previous_release = before.get('previous_release')
  atomic_json(ROOT / 'updates/active.json',
              {'source_commit': manifest['source_commit'], 'model': model_identity(manifest),
               'previous_release': previous_release})


def keeps_current(manifest):
  """Same model already received: keep the running release when only the commit differs (대감님 11/27 05:43).

  A comma git pull changes release.json source_commit/bundle, but the Jetson model file is stored by its
  hash (cache/models/<sha256[:16]>.onnx). When the current release was activated with this model
  (sha256 + size) and carries its spec, nothing is downloaded and the boot gate does not block.
  Only a different model hash takes the install path.
  """
  if not isinstance(manifest, dict) or manifest.get('delivery') in ('usb', 'local') or 'model' not in manifest:
    return False
  try:
    if active_model() != model_identity(manifest):
      return False
    path = model_path(manifest['model'])
    if path.is_symlink() or not path.is_file() or path.stat().st_size != manifest['model']['size']:
      return False
    return release_spec(ROOT / 'current', manifest['model']) is not None
  except (OSError, ValueError, KeyError, TypeError):
    return False


def selected_commit(selection):
  if selection.get('delivery') == 'usb':
    from usb_release import trusted, registered
    trusted(ROOT, selection)
    validate_storage_compatibility(selection)
    # The boot client repeats its pin every 500 ms. Hash large files during
    # registration/staging/activation, not on this identity-only heartbeat.
    if not registered(ROOT, selection, verify_files=False):
      raise ValueError('Transfer the verified USB package before selecting it')
    return selection['source_commit']
  if selection.get('delivery') == 'local':
    from local_release import resolve
    record = resolve(ROOT, selection)
    validate_storage_compatibility(record)
    return record['source_commit']
  validate_manifest(selection)
  verify_signature(selection)
  validate_storage_compatibility(selection)
  commit = selection['source_commit']
  try:
    current = (ROOT / 'current/SOURCE_COMMIT').read_text().strip()
  except OSError:
    current = None
  if current == commit and not is_active(selection):
    # Shared bundle: same runtime commit, other model. The boot gate compares
    # this value with current/SOURCE_COMMIT, so it must not look active until
    # the selected model has been staged, probed and recorded.
    return commit + ':' + selection['model']['sha256'][:16]
  if current is not None and current != commit and keeps_current(selection):
    # Commit-only change: the boot gate sees the running release as selected and needs no download.
    return current
  return commit


def stage_manifest(manifest, progress=None):
  if manifest.get('delivery') == 'usb':
    from usb_release import selection
    selected_commit(manifest)
    from local_release import stage as stage_local
    stage_local(ROOT, selection(manifest), progress)
    return
  if manifest.get('delivery') == 'local':
    from local_release import stage as stage_local
    stage_local(ROOT, manifest, progress)
    return
  from finalize_sd_image import extract_bundle
  validate_manifest(manifest)
  verify_signature(manifest)
  validate_storage_compatibility(manifest)
  commit = manifest['source_commit']
  if is_active(manifest) and model_path(manifest['model']).is_file():
    return
  # Installation dependencies and kernel changes need a separately tested image.
  check_runtime(manifest)
  updates = ROOT / 'updates'
  updates.mkdir(exist_ok=True)
  bundle = updates / (commit + '.tar.gz')
  item = manifest['bundle']
  fetch(item['url'], bundle, item['sha256'], item['size'], progress=progress)
  release = ROOT / 'releases' / commit
  if not release.exists():
    with tempfile.TemporaryDirectory(dir=ROOT / 'releases', prefix='update-') as temporary:
      candidate = Path(temporary)
      extract_bundle(bundle, candidate)
      check_bundle(candidate, manifest)
      (candidate / '.bundle-sha256').write_text(item['sha256'])
      candidate.chmod(0o755)
      os.rename(candidate, release)
  elif not (release / '.bundle-sha256').exists() or (release / '.bundle-sha256').read_text() != item['sha256']:
    raise ValueError('Existing release has a different identity')
  else:
    check_bundle(release, manifest)  # A shared release must also carry this model's spec.
  model = manifest['model']
  destination = ROOT / 'cache/models' / (model['sha256'][:16] + '.onnx')
  fetch(model['url'], destination, model['sha256'], model['size'], progress=progress)
  # Cache files are consumed and managed by the unprivileged inference service.
  import pwd
  owner = pwd.getpwnam('jetlink')
  os.chown(destination, owner.pw_uid, owner.pw_gid)
  atomic_json(updates / 'pending.json', manifest)
  atomic_json(updates / 'status.json', {'state': 'staged', 'source_commit': commit, 'updated': time.time()})
  os.sync()


def activate():
  pending = ROOT / 'updates/pending.json'
  transaction = ROOT / 'updates/transaction.json'
  if not pending.exists() and not transaction.exists():
    return
  for name in ('carrot-jetlink', 'carrot-jetlink-hud'):
    if subprocess.run(['systemctl', 'is-active', '--quiet', name]).returncode == 0:
      raise RuntimeError('Activation requires stopped inference and HUD services; use the next boot')
  if transaction.exists():
    saved = json.loads(transaction.read_text())
    old = Path(saved['release'])
    if old.is_symlink() or old.resolve().parent != (ROOT / 'releases').resolve() or not old.is_dir():
      raise ValueError('Invalid recovery release')
    link = ROOT / 'current.recovery'
    link.unlink(missing_ok=True)
    link.symlink_to(old)
    os.replace(link, ROOT / 'current')
    if saved['last_loaded'] is not None:
      (ROOT / 'cache/last-loaded.json').write_text(saved['last_loaded'])
    if pending.exists():
      os.replace(pending, pending.with_name('interrupted.json'))
    os.sync()  # Recovery must be durable before its retry record disappears.
    transaction.unlink()
    atomic_json(ROOT / 'updates/status.json', {'state': 'recovered', 'updated': time.time()})
    os.sync()
    return
  manifest = json.loads(pending.read_text())
  if manifest.get('delivery') == 'local':
    from local_release import verify_staged
    manifest = verify_staged(ROOT, manifest)
  else:
    validate_manifest(manifest)
    verify_signature(manifest)
    validate_storage_compatibility(manifest)
  release = ROOT / 'releases' / manifest['source_commit']
  if release.is_symlink() or release.resolve().parent != (ROOT / 'releases').resolve():
    raise ValueError('Invalid release directory')
  previous = (ROOT / 'current').resolve()
  previous_active = active_model()
  remembered = ROOT / 'cache/last-loaded.json'
  previous_model = remembered.read_bytes() if remembered.exists() else None
  saved = {'release': str(previous), 'last_loaded': previous_model.decode() if previous_model else None}
  atomic_json(transaction, saved)
  os.sync()
  try:
    # No USB/control connection: load/build and execute synthetic tensors in a
    # separate process before the new sources are reachable by the vehicle.
    if 'model' in manifest and not pin_matches(release, manifest['model']):
      probe_release(release, model_identity(manifest))  # shared bundle, non-pin model
    else:
      probe_release(release)
    link = ROOT / 'current.next'
    link.unlink(missing_ok=True)
    link.symlink_to(release)
    atomic_json(ROOT / 'updates/previous.json', saved)
    os.replace(link, ROOT / 'current')
    if 'model' in manifest:
      record_active(manifest, previous_active, previous.name)
    atomic_json(ROOT / 'updates/status.json', {'state': 'applied', 'source_commit': manifest['source_commit'], 'updated': time.time()})
    pending.unlink()
    os.sync()
    transaction.unlink()
    os.sync()
    try:
      prune()
    except OSError as error:
      print('Old release cleanup deferred:', error, flush=True)
    # Legacy images migrate networking independently of inference. Protected
    # images keep their immutable recovery worker and never write the base OS.
    installer = release / 'tools/jetlink/install_wifi.py'
    if not Path('/etc/carrot-jetlink-protected.json').exists() and installer.is_file():
      try:
        subprocess.run(['/usr/bin/python3', str(installer)], check=True, timeout=30,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(['systemctl', 'daemon-reload'], check=True, timeout=10)
      except (OSError, subprocess.SubprocessError):
        print('Network helper refresh deferred; existing worker retained', flush=True)
  except Exception as error:
    if (ROOT / 'current').resolve() != previous:
      link = ROOT / 'current.rollback'
      link.unlink(missing_ok=True)
      link.symlink_to(previous)
      os.replace(link, ROOT / 'current')
    if previous_model is not None:
      remembered.write_bytes(previous_model)
    atomic_json(ROOT / 'updates/status.json', {'state': 'rejected', 'error': str(error)[:240], 'updated': time.time()})
    if pending.exists():
      os.replace(pending, pending.with_name('rejected.json'))
    os.sync()
    transaction.unlink(missing_ok=True)
    os.sync()
    print('Candidate rejected; retaining previous release:', error, flush=True)


def automatic_stage():
  # The private bootstrap channel arrives before model readiness. Its public
  # subset allows a newly connected host to stage the comma's pinned release.
  control_file = Path('/dev/shm/carrot-jetlink-bootstrap.json')
  try:
    control = json.loads(control_file.read_text())
    if 0 <= time.monotonic() - control['received'] < 15:
      if control.get('onroad') is False and control.get('jetson_release') is not None:
        stage_manifest(control['jetson_release'])
      return
  except (OSError, ValueError, KeyError, TypeError):
    pass
  from hud_protocol import read_snapshot
  snapshot = read_snapshot()
  if snapshot is None:
    return  # Missing telemetry never means parked/offroad.
  import base64
  if base64.b64decode(snapshot[1].get('params', {}).get('IsOnroad', '')) != b'0':
    return
  manifest = snapshot[1].get('jetson_release')
  if manifest is not None:
    # Carrot selects the compatible signed host/model release. Never silently
    # fall back to a different channel if that selection fails verification.
    stage_manifest(manifest)
  else:
    stage()  # Compatibility for C4 releases predating version pinning.


def prestage():
  # Boot-gate hosts, after this boot was approved: download/verify the comma's
  # signed pin also while onroad (Nice 19, idle I/O). Writes only updates/,
  # releases/<commit> and the model cache; never current/ or running services.
  # The next boot's gate activates it exactly as before. No unpinned channel.
  control_file = Path('/dev/shm/carrot-jetlink-bootstrap.json')
  try:
    control = json.loads(control_file.read_text())
    if 0 <= time.monotonic() - control['received'] < 15 and control.get('jetson_release') is not None:
      stage_manifest(control['jetson_release'])
      return
  except (OSError, ValueError, KeyError, TypeError):
    pass
  from hud_protocol import read_snapshot
  snapshot = read_snapshot()
  if snapshot is not None and snapshot[1].get('jetson_release') is not None:
    stage_manifest(snapshot[1]['jetson_release'])


def main():
  import fcntl
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument('action', choices=['stage', 'activate', 'automatic', 'boot'])
  parser.add_argument('--manifest', default=DEFAULT_MANIFEST)
  args = parser.parse_args()
  if os.geteuid() != 0:
    raise PermissionError('Run through the installed update service (root)')
  from boot_update import enabled, boot
  boot_gate = enabled()
  if args.action == 'automatic' and boot_gate:
    # The boot gate applies the comma's pin before this boot's first model.
    # After this boot was approved, pre-download a newer pin (also onroad) so
    # the next boot's gate only verifies and activates it.
    from boot_update import runtime_ready
    if not runtime_ready():
      return
    try:
      with Path('/run/carrot-jetlink-update.lock').open('w') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        prestage()
    except BlockingIOError:
      pass  # Another update action owns the files; retry at the next timer.
    return
  if args.action == 'activate' and boot_gate:
    # A separate transient service has no startup timeout and survives the old
    # image's oneshot apply service finishing. No writes to a protected OS.
    subprocess.run(['systemd-run', '--unit=carrot-jetlink-boot-gate',
                    '--property=Restart=on-failure', '--property=RestartSec=3',
                    '--property=Nice=19', '--property=CPUAffinity=0 1',
                    str(ROOT / 'venv/bin/python'), str(Path(__file__).resolve()), 'boot'], check=True, timeout=15)
    return
  if Path('/etc/carrot-jetlink-protected.json').exists():
    status = json.loads(Path('/run/carrot-storage.json').read_text())
    if status.get('state') != 'protected':
      print('DATA unavailable; keeping immutable recovery runtime', flush=True)
      if not boot_gate:
        return
      # Leave the services blocked and expose the storage failure over USB.
  with Path('/run/carrot-jetlink-update.lock').open('w') as lock:
    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    if args.action == 'boot':
      if not boot_gate:
        raise RuntimeError('Boot gate policy is not enabled for this boot')
      boot()
    elif args.action == 'activate':
      activate()
    elif args.action == 'automatic':
      automatic_stage()
    else:
      stage(args.manifest)


if __name__ == '__main__':
  main()
