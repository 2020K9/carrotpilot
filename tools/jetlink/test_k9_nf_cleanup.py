"""Fake-filesystem tests: the NaN-tracer / logkeep cleanup removes only K9 paths (2026-11-25)."""
import os
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent))
import k9_nf_cleanup as c


def _touch(p, data=b'x'):
  p.parent.mkdir(parents=True, exist_ok=True)
  p.write_bytes(data)
  return p


def test_removes_only_k9_paths(tmp_path):
  cache = tmp_path / 'opt/carrot-jetlink/cache'
  keep = [_touch(cache / 'models/abcd.onnx'), _touch(cache / 'engines/abcd.engine'), _touch(cache / 'nf_trace_notes.txt'),
          _touch(tmp_path / 'keys/release.pub'), _touch(tmp_path / 'data/user/settings.json')]
  _touch(cache / 'nf_trace/20261121-1_ab/inputs.npz')
  logs = tmp_path / 'run/carrot-data/logs'
  _touch(logs / '20261122-153000_0123abcd/journal.log')
  _touch(logs / '20261122-153000_0123abcd/kern.1.log.gz')
  _touch(logs / '20261122-153000_0123abcd/carrot.log')
  _touch(logs / '20261122-153000_0123abcd/dmesg_start.txt')
  _touch(logs / '20261123-080000_89abcdef/journal.log.gz.tmp')
  keep += [_touch(logs / 'other/journal.log'), _touch(logs / '20261122-153000_0123abcd0/journal.log'),
           _touch(logs / '20261124-090000_00112233/user.txt'), _touch(logs / 'README')]
  calls = []
  def run(argv, timeout=None):
    calls.append(argv)
    R = type("R", (), {"returncode": 1})
    return R()
  removed = c.cleanup(cache=[cache], logs=logs, run=run)
  assert not (cache / 'nf_trace').exists()
  assert not (logs / '20261122-153000_0123abcd').exists() and not (logs / '20261123-080000_89abcdef').exists()
  for p in keep:
    assert p.is_file(), p
  assert logs.is_dir()
  assert calls == [['systemctl', 'is-active', '--quiet', c.UNIT]]
  assert len(removed) == 3


def test_empty_logs_dir_removed_and_unit_stopped(tmp_path):
  logs = tmp_path / 'logs'
  _touch(logs / '20261122-153000_0123abcd/kern.log')
  calls = []
  def run(argv, timeout=None):
    calls.append(argv)
    R = type("R", (), {"returncode": 0})
    return R()
  c.cleanup(cache=[], logs=logs, run=run)
  assert not logs.exists()
  assert calls[-1] == ['systemctl', 'stop', c.UNIT]


def test_symlinks_never_followed(tmp_path):
  outside = tmp_path / 'outside'
  victim = _touch(outside / 'precious.bin')
  cache = tmp_path / 'cache'
  cache.mkdir()
  os.symlink(outside, cache / 'nf_trace')
  logs = tmp_path / 'logs'
  logs.mkdir()
  os.symlink(outside, logs / '20261122-153000_0123abcd')
  c.cleanup(cache=[cache], logs=logs, run=None)
  assert victim.is_file()
  assert not (cache / 'nf_trace').is_symlink()
  assert (logs / '20261122-153000_0123abcd').is_symlink()


def test_missing_paths_are_fine(tmp_path):
  assert c.cleanup(cache=[tmp_path / 'none'], logs=tmp_path / 'nologs', run=None) == []


def test_logs_symlink_root_untouched(tmp_path):
  outside = tmp_path / 'outside'
  victim = _touch(outside / '20261122-153000_0123abcd/journal.log')
  os.symlink(outside, tmp_path / 'logs')
  c.cleanup(cache=[], logs=tmp_path / 'logs', run=None)
  assert victim.is_file()


def test_server_start_runs_cleanup_as_root():
  src = (Path(__file__).resolve().parent / 'server.py').read_text()
  assert 'k9_nf_cleanup()' in src
  calls = []
  assert c.start_as_root(lambda argv, **kw: calls.append(argv))
  assert calls[0][:2] == ['sudo', '-n'] and calls[0][-1].endswith('tools/jetlink/k9_nf_cleanup.py')
  def fail(*a, **k):
    raise OSError('no sudo')
  assert c.start_as_root(fail) is False
