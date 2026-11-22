import gzip
import os
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent))
import logkeep
import host_logkeep

from openpilot.system import k9_logkeep


def _age(p, seconds):
  t = time.time() - seconds  # noqa: TID251 -- file mtimes are wall clock
  os.utime(p, (t, t))


def test_sink_parts_and_gzip(tmp_path):
  s = logkeep.Sink(tmp_path, 'journal', part_bytes=100)
  for i in range(30):
    s.write(f'line {i:03d} ................\n'.encode())
  s.close()
  for _ in range(50):
    if not list(tmp_path.glob('journal*.log.tmp')) and not (tmp_path / 'journal.log').exists():
      break
    time.sleep(0.05)
  gz = sorted(tmp_path.glob('journal*.log.gz'))
  assert gz, list(tmp_path.iterdir())
  text = b''.join(gzip.open(p).read() for p in gz) + b''.join(p.read_bytes() for p in tmp_path.glob('journal*.log'))
  assert text.count(b'\n') == 30
  # a restarted keeper continues after the last part instead of overwriting it
  s2 = logkeep.Sink(tmp_path, 'journal', part_bytes=100)
  assert s2.index == s.index
  s2.close()


def test_prune_age_cap_protect_and_scope(tmp_path):
  root = tmp_path / 'logs'
  old = root / 'a_1' / 'journal.log'
  old.parent.mkdir(parents=True)
  old.write_bytes(b'x' * 10)
  _age(old, 3 * 86400 + 60)
  young = root / 'b_2' / 'journal.log'
  young.parent.mkdir()
  young.write_bytes(b'y' * 10)
  _age(young, 86400)
  live = root / 'c_3' / 'journal.log'
  live.parent.mkdir()
  live.write_bytes(b'z' * 10)
  _age(live, 5 * 86400)          # current boot with a wrong clock: still protected
  outside = tmp_path / 'other.log'
  outside.write_bytes(b'o')
  _age(outside, 10 * 86400)
  deleted = logkeep.prune(root, min_free=0, protect={live})
  assert deleted == [old] and not old.parent.exists()
  assert young.exists() and live.exists() and outside.exists()
  # size cap: oldest first, protected kept
  for i in range(5):
    p = root / 'b_2' / f'kern.{i}.log.gz'
    p.write_bytes(b'k' * 100)
    _age(p, 3600 * (10 - i))
  logkeep.prune(root, max_bytes=250, min_free=0, protect={live})
  left = sorted(p.name for p in (root / 'b_2').iterdir())
  assert left == ['kern.3.log.gz', 'kern.4.log.gz'] and live.exists()


def test_keeper_follows_streams_and_snapshots(tmp_path):
  k = logkeep.Keeper(tmp_path, {'journal': ['sh', '-c', 'echo one; echo two; sleep 30']},
                     snapshots={'dmesg_start': ['echo', 'ring']}, fsync_s=0.05, prune_s=0.05)
  import threading
  t = threading.Thread(target=k.run, daemon=True)
  t.start()
  for _ in range(100):
    if k.dir is not None and (k.dir / 'journal.log').exists() and (k.dir / 'journal.log').read_bytes().count(b'\n') >= 2:
      break
    time.sleep(0.05)
  k.close()
  t.join(10)
  assert (k.dir / 'journal.log').read_bytes().startswith(b'one\ntwo\n')
  assert (k.dir / 'dmesg_start.txt').read_bytes() == b'ring\n'
  assert k.dir.name.endswith('_' + logkeep.boot_id()[:8])
  assert logkeep.boot_dir(tmp_path) == k.dir   # same boot -> same folder


def test_journal_streams_follow_whole_boot_without_duplicates():
  s = logkeep.journal_streams({'carrot': ['carrot-jetlink.service']})
  assert set(s) == {'journal', 'kern', 'carrot'}
  for argv in s.values():
    assert argv[0] == 'journalctl' and '-b' in argv and '-f' in argv and ['-n', 'all'] == argv[argv.index('-n'):argv.index('-n') + 2]
    assert '--cursor-file={cursor}' not in argv
  assert s['kern'][-1] == '-k' and s['carrot'][-2:] == ['-u', 'carrot-jetlink.service']


def test_restart_continues_from_last_kept_second(tmp_path):
  p = tmp_path / 'journal.log'
  p.write_bytes(b'2026-11-22T06:49:12.633705+00:00 host a: x\n# keeper: note\n')
  import datetime
  want = datetime.datetime(2026, 11, 22, 6, 49, 12, tzinfo=datetime.UTC).astimezone().strftime('%Y-%m-%d %H:%M:%S')
  assert logkeep._last_stamp(p) == want
  p2 = tmp_path / 'old.log'
  p2.write_bytes(b'2026-11-22T15:49:12.633705+0900 host a: x\n')   # systemd 249 (Jetson) offset form
  assert logkeep._last_stamp(p2) == want
  assert logkeep._last_stamp(tmp_path / 'none') is None
  seen = []
  k = logkeep.Keeper(tmp_path / 'k', {'journal': ['journalctl', '-b', '-f', '-n', 'all']})
  k.open()
  k.sinks['journal'].write(b'2026-11-22T06:49:12.633705+00:00 host a: x\n')

  class P:
    returncode = 0
    stdout = iter(())

    def terminate(self):
      k.stop.set()

    def wait(self, t=None):
      return 0

  class Out(list):
    def close(self):
      pass
  P.stdout = Out()
  real = logkeep.subprocess.Popen
  logkeep.subprocess.Popen = lambda cmd, **kw: (seen.append(cmd), P())[1]
  try:
    k._follow('journal', k.streams['journal'])
  finally:
    logkeep.subprocess.Popen = real
    k.close()
  assert seen[0] == ['journalctl', '-b', '-f', '--since', want]


def test_previous_boot_kernel(tmp_path):
  a = tmp_path / '20261122-061000_aaaaaaaa'
  a.mkdir()
  (a / 'kern.log').write_text('nvgpu: oops\n')
  cur = tmp_path / '20261122-070000_bbbbbbbb'
  cur.mkdir()
  (cur / 'kern.log').write_text('now\n')
  assert 'nvgpu: oops' in logkeep.previous_boot_kernel(tmp_path, cur)
  assert logkeep.previous_boot_kernel(tmp_path / 'none') == ''


def test_jetson_start_uses_transient_root_unit(monkeypatch, tmp_path):
  calls = []

  class R:
    def __init__(self, rc):
      self.returncode = rc

  def run(argv, **kw):
    calls.append(argv)
    return R(3 if argv[:2] == ['systemctl', 'is-active'] else 0)

  real = Path.is_file
  monkeypatch.setattr(Path, 'is_file', lambda self: True if str(self) == '/etc/nv_tegra_release' else real(self))
  assert host_logkeep.start('/x/host_logkeep.py', run=run)
  assert calls[1][:4] == ['sudo', '-n', 'systemd-run', '--unit=carrot-jetlink-logkeep']
  assert calls[1][-2:] == ['/usr/bin/python3', '/x/host_logkeep.py']
  calls.clear()
  assert host_logkeep.start('/x', run=lambda a, **k: R(0)) and calls == []   # already running: left alone
  # not mounted DATA -> no keeping in RAM
  assert host_logkeep.persistent_root(tmp_path) is None


def test_comma_swaglog_archive_and_crash_copy(tmp_path):
  src = tmp_path / 'log'
  src.mkdir()
  for n in range(4):
    p = src / f'swaglog.{n:010d}'
    p.write_text(f'msg {n}\n')
    _age(p, 600 - n)
  _age(src / 'swaglog.0000000003', 10)        # still being written
  old = src / 'swaglog.0000000009'
  old.write_text('ancient\n')
  _age(old, 4 * 86400)                       # older than 3 days: not archived
  crashes = tmp_path / 'process_crashes'
  crashes.mkdir()
  (crashes / 'a.txt').write_text('Traceback')
  k = logkeep.Keeper(tmp_path / 'keep', {})
  k.root.mkdir()
  assert k9_logkeep.archive_swaglog(k, src) == 3
  assert k9_logkeep.archive_swaglog(k, src) == 0       # once only
  data = b''.join(gzip.open(p).read() for p in (k.root / 'swaglog').glob('*.gz'))
  assert data == b'msg 0\nmsg 1\nmsg 2\n'
  _age(src / 'swaglog.0000000003', 600)
  assert k9_logkeep.archive_swaglog(k, src) == 1
  assert k9_logkeep.copy_crashes(k, [crashes]) == 1
  assert k9_logkeep.copy_crashes(k, [crashes]) == 0
  assert (k.root / 'crash' / 'process_crashes__a.txt').read_text() == 'Traceback'
  assert src.joinpath('swaglog.0000000000').exists()   # openpilot's own files are never touched


def test_comma_process_registered():
  from openpilot.system.manager.process_config import managed_processes
  p = managed_processes['k9_logkeep']
  assert p.module == 'openpilot.system.k9_logkeep'
