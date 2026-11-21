"""Small, allowlisted boot diagnostics. Never include profiles, keys or journals."""
import ipaddress
import json
import math
from pathlib import Path
import subprocess
import time

STAGE = Path('/run/carrot-boot-stage.json')
UNITS = ('carrot-protected-storage', 'carrot-image-setup', 'carrot-jetlink-usbc-host',
         'nv', 'nvpmodel', 'carrot-super-firmware', 'carrot-jetlink-performance', 'NetworkManager', 'ssh',
         'carrot-jetlink-wifi', 'carrot-jetlink', 'carrot-jetlink-xorg', 'carrot-jetlink-hud')
# Jetson server health file (= host_health.STATUS, rewritten every second while the server runs). Only its
# first-time startup marks are read here, for the boot screen's loading %; nothing else is taken from it.
HOST_HEALTH = Path('/dev/shm/carrot-jetlink-health.json')
STARTUP_MARKS = ('server_start', 'hello', 'engine_req', 'engine_ready', 'infer')


def read_json(path, limit=32769):
  try:
    with Path(path).open() as stream:
      value = json.loads(stream.read(limit))
    return value if isinstance(value, dict) else {}
  except (OSError, ValueError):
    return {}


def _mark(value):
  return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value >= 0


def startup_marks(path=HOST_HEALTH, now=None):
  """{mark: done} from a fresh server health file; None when it is missing, broken or stale (same 3 s as the HUD)."""
  try:
    value = read_json(path, 1 << 18)
  except RecursionError:   # absurdly nested = broken; the screen itself must never fail on this file
    return None
  updated = value.get('updated')
  diag = value.get('hud_diag')
  startup = diag.get('startup') if isinstance(diag, dict) else None
  now = time.monotonic() if now is None else now
  if not _mark(updated) or not 0 <= now - updated < 3 or not isinstance(startup, dict):
    return None
  return {name: _mark(startup.get(name)) for name in STARTUP_MARKS}


def startup_times(path=HOST_HEALTH, now=None):
  """{"now": this clock, mark: Jetson monotonic time it happened} from the same fresh health file as startup_marks
  (same clock: both are time.monotonic on this Jetson); None when the file is missing, broken or stale. Used only for
  the remaining-time estimate next to the loading % of the boot screen."""
  now = time.monotonic() if now is None else now
  if startup_marks(path, now) is None:
    return None
  try:
    startup = read_json(path, 262144)["hud_diag"]["startup"]
  except (KeyError, TypeError, RecursionError):
    return None
  return {"now": now, **{name: float(startup[name]) for name in STARTUP_MARKS if _mark(startup.get(name))}}


def stage(name, error=None):
  value = {'stage': name, 'updated': time.monotonic()}
  if error:
    value['error'] = type(error).__name__
  temporary = STAGE.with_suffix('.tmp')
  temporary.write_text(json.dumps(value))
  temporary.replace(STAGE)


def collect():
  units = {}
  try:
    result = subprocess.run(['systemctl', 'show', '--property=Id,ActiveState,SubState,Result',
                             *[name + '.service' for name in UNITS]],
                            capture_output=True, text=True, timeout=3)
    for section in result.stdout.strip().split('\n\n'):
      fields = dict(line.split('=', 1) for line in section.splitlines() if '=' in line)
      name = fields.get('Id', '').removesuffix('.service')
      if name in UNITS:
        units[name] = {key: fields.get(key, 'unknown') for key in ('ActiveState', 'SubState', 'Result')}
  except (OSError, subprocess.SubprocessError):
    pass
  addresses = []
  try:
    result = subprocess.run(['ip', '-j', '-4', 'addr', 'show', 'scope', 'global'],
                            capture_output=True, text=True, timeout=2, check=True)
    for interface in json.loads(result.stdout):
      for entry in interface.get('addr_info', []):
        address = ipaddress.ip_address(entry.get('local', ''))
        if not address.is_link_local and not address.is_loopback:
          addresses.append(str(address))
  except (OSError, ValueError, subprocess.SubprocessError):
    pass
  network = read_json('/dev/shm/carrot-jetlink-network-status.json')
  if not 0 <= time.monotonic() - network.get('updated', -100) < 6:
    network = {}
  temperatures = []
  for path in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
    try:
      temperature = int(path.read_text()) / 1000
      if -20 <= temperature <= 150:
        temperatures.append(temperature)
    except (OSError, ValueError, TypeError):
      pass
  return {'stage': read_json(STAGE), 'storage': read_json('/run/carrot-storage.json'),
          'firmware': read_json('/run/carrot-super-firmware.json'),
          'units': units, 'addresses': addresses[:4], 'wifi': network.get('state', 'waiting'),
          'uptime': int(time.monotonic()), 'temp_c': max(temperatures, default=None),
          'jetson': startup_marks(), 'jetson_at': startup_times()}


def persist_result(directory, value):
  """Called once at the end of storage boot, on the already guarded setup mount."""
  from persistent_state import durable_write
  # Restrict persisted content even if a caller accidentally supplies raw logs.
  record = {'format': 1, 'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            'stage': str(value.get('stage', 'unknown'))[:48],
            'error': str(value.get('error', ''))[:80]}
  durable_write(Path(directory) / 'BOOT-STATUS.json', json.dumps(record).encode(), 0o600)
