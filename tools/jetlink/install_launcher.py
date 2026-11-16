"""Install the updater start program (updater_launcher.py) and a byte-exact legacy updater copy.

Writes only:
  <root>/opt/carrot-jetlink/updater-legacy/   legacy updater bytes + SHA256SUMS
  <root>/opt/carrot-jetlink/launcher/updater_launcher.py
  <root>/etc/systemd/system/carrot-jetlink-update-{apply,stage}.service.d/90-updater-launcher.conf
Never touches /opt/carrot-jetlink/updater (switch_updater_linked.py owns it),
boot-update-required, the unit files themselves or the timer.  The drop-ins are
written last: until they exist systemd still runs updater/update_host.py directly.

Why drop-ins: 0cb67d46c install_updates.py bootstrap() rewrites only
/opt/carrot-jetlink/updater/* and boot-update-required; configure() rewrites the
three unit files but no *.service.d/ file (power.conf / storage.conf there set no
ExecStart), so the drop-in's ExecStart survives both.

--legacy-source must be a folder with the legacy (old key, NAS) updater, for
example the current /opt/carrot-jetlink/updater BEFORE switch_updater_linked.py
runs, or its backup.  The copy is refused unless its public key is the old key.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
from switch_updater import NAMES, OLD_KEY, REQUIRED, key_of

PYTHON = '/opt/carrot-jetlink/venv/bin/python'
LAUNCHER = '/opt/carrot-jetlink/launcher/updater_launcher.py'
DROPIN = '90-updater-launcher.conf'
SERVICES = {'carrot-jetlink-update-apply.service': 'activate', 'carrot-jetlink-update-stage.service': 'automatic'}


def sha_bytes(data):
  return hashlib.sha256(data).hexdigest()


def durable(path, data, mode=0o644):
  path = Path(path)
  path.parent.mkdir(parents=True, exist_ok=True)
  temporary = path.with_name(path.name + '.new')
  with temporary.open('wb') as stream:
    stream.write(data)
    stream.flush()
    os.fsync(stream.fileno())
  temporary.chmod(mode)
  os.replace(temporary, path)
  fd = os.open(str(path.parent), os.O_RDONLY)
  try:
    os.fsync(fd)
  finally:
    os.close(fd)


def dropin_text(action):
  # 10 = legacy ran after a new-updater failure, 30 = another updater holds the lock.
  return (f'[Service]\nExecStart=\nExecStart={PYTHON} {LAUNCHER} {action}\nSuccessExitStatus=10 30\n')


def install(root, legacy_source, launcher_source=None):
  root, legacy_source = Path(root), Path(legacy_source)
  launcher_source = Path(launcher_source or Path(__file__).resolve().with_name('updater_launcher.py'))
  missing = [n for n in REQUIRED if not (legacy_source / n).is_file()]
  if missing:
    raise SystemExit(f'legacy source incomplete: {missing}')
  if key_of(legacy_source / 'release-signing-public.pem') != OLD_KEY:
    raise SystemExit('legacy source does not carry the old public key; refusing')
  files = {n: (legacy_source / n).read_bytes() for n in NAMES if (legacy_source / n).is_file()}
  sums = ''.join(f'{sha_bytes(data)}  {name}\n' for name, data in sorted(files.items()))
  legacy = root / 'opt/carrot-jetlink/updater-legacy'
  if (legacy / 'SHA256SUMS').is_file():
    if (legacy / 'SHA256SUMS').read_text() != sums:
      raise SystemExit('updater-legacy already holds different bytes; refusing to overwrite')
  for name, data in files.items():  # files first, list of sums last
    durable(legacy / name, data)
  durable(legacy / 'SHA256SUMS', sums.encode())
  durable(root / LAUNCHER.lstrip('/'), launcher_source.read_bytes())
  units = root / 'etc/systemd/system'
  for service, action in SERVICES.items():
    durable(units / (service + '.d') / DROPIN, dropin_text(action).encode())
  if hasattr(os, 'sync'):
    os.sync()
  return {'legacy_sha256': {n: sha_bytes(d) for n, d in files.items()},
          'launcher_sha256': sha_bytes(launcher_source.read_bytes())}


if __name__ == '__main__':
  parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
  parser.add_argument('--root', default='/')
  parser.add_argument('--legacy-source', required=True)
  args = parser.parse_args()
  print(json.dumps(install(args.root, args.legacy_source), indent=1))
  if args.root == '/':
    print('run: systemctl daemon-reload  (takes effect for the next timer/boot run)')
