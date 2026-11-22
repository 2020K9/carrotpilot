"""Jetson: keep system / kernel / carrot service logs across reboots on the DATA partition (K9 logkeep).

The protected image keeps /var/log, /etc and the journal in RAM (tmpfs / overlay), so every reboot lost
them. This keeper runs as root (started by server.py through `sudo -n systemd-run`, a transient unit:
nothing is written to the read-only OS or the /etc overlay) and writes, per boot,
/run/carrot-data/logs/<time>_<boot id>/{journal,kern,carrot}.log + dmesg_start.txt; 3 days / 1 GB.
See tools/jetlink/logkeep.py.
"""
import grp
import os
from pathlib import Path
import subprocess
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import logkeep

DATA = Path('/run/carrot-data')
ROOT = DATA / 'logs'
UNIT = 'carrot-jetlink-logkeep'
UNITS = ['carrot-jetlink.service', 'carrot-jetlink-hud.service', 'carrot-jetlink-boot-display.service',
         'carrot-jetlink-update-apply.service', 'carrot-jetlink-update-stage.service', 'carrot-jetlink-boot-gate.service',
         'carrot-jetlink-performance-observer.service', 'carrot-jetlink-wifi.service', 'carrot-jetlink-xorg.service',
         'carrot-protected-storage.service', 'carrot-image-setup.service', UNIT + '.service']


def persistent_root(data=DATA):
  """Only the mounted DATA partition: never fill the RAM-backed /run when DATA is missing."""
  return data / 'logs' if os.path.ismount(data) else None


def keeper(root):
  try:
    group = grp.getgrnam('jetlink').gr_gid
  except KeyError:
    group = None
  return logkeep.Keeper(root, logkeep.journal_streams({'carrot': UNITS}),
                        snapshots={'dmesg_start': ['dmesg', '--time-format', 'iso']}, group=group)


def start(script=None, run=subprocess.run):
  """server.py (user jetlink): start the root keeper once per boot; an already running unit is left alone."""
  if not Path('/etc/nv_tegra_release').is_file():
    return False
  script = str(script or Path(__file__).resolve())
  try:
    if run(['systemctl', 'is-active', '--quiet', UNIT], timeout=10).returncode == 0:
      return True
    r = run(['sudo', '-n', 'systemd-run', f'--unit={UNIT}', '--collect', '--property=Nice=19',
             '--property=IOSchedulingClass=idle', '--property=Restart=on-failure', '--property=RestartSec=10',
             '--property=CPUAffinity=0 1', '/usr/bin/python3', script],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=15)
    return r.returncode == 0
  except (OSError, subprocess.SubprocessError):
    return False


def start_async():
  import threading
  threading.Thread(target=start, daemon=True, name='logkeep-start').start()


def main():
  if os.geteuid() != 0:
    raise SystemExit('host_logkeep runs as root (started by server.py via sudo -n systemd-run)')
  root = persistent_root()
  if root is None:
    raise SystemExit(f'{DATA} is not mounted; not keeping logs in RAM')
  keeper(root).run()


if __name__ == '__main__':
  main()
