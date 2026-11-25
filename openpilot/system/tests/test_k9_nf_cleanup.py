"""Fake-filesystem tests: comma cleanup removes only /data/media/0/k9_logs and k9_nf_trace (2026-11-25)."""
import os
from pathlib import Path

from openpilot.system import k9_nf_cleanup as c


def _touch(p):
  p.parent.mkdir(parents=True, exist_ok=True)
  p.write_bytes(b'x')
  return p


def test_removes_only_k9_folders(tmp_path):
  media = tmp_path / 'media/0'
  _touch(media / 'k9_logs/20261122-153000_0123abcd/journal.log')
  _touch(media / 'k9_logs/swaglog/swaglog_20261122.gz')
  _touch(media / 'k9_logs/crash/x.txt')
  _touch(media / 'k9_nf_trace/ev1/inputs.npz')
  keep = [_touch(media / 'realdata/2026-11-22--10-00-00--0/rlog.zst'), _touch(media / 'k9_logs_note.txt'),
          _touch(media / 'dashcam/a.mp4'), _touch(media / 'k9_other/x')]
  removed = c.cleanup(media)
  assert sorted(Path(p).name for p in removed) == ['k9_logs', 'k9_nf_trace']
  assert not (media / 'k9_logs').exists() and not (media / 'k9_nf_trace').exists()
  for p in keep:
    assert p.is_file(), p


def test_symlink_only_unlinked(tmp_path):
  media = tmp_path / 'm'
  media.mkdir()
  victim = _touch(tmp_path / 'params/d/CarParams')
  os.symlink(tmp_path / 'params', media / 'k9_logs')
  c.cleanup(media)
  assert victim.is_file() and not (media / 'k9_logs').exists()


def test_nothing_there(tmp_path):
  assert c.cleanup(tmp_path) == []


def test_manager_init_calls_cleanup():
  src = (Path(c.__file__).resolve().parent / 'manager/manager.py').read_text()
  assert 'k9_nf_cleanup' in src
