"""11/24 18:29 (대감님 "HUD 왼쪽 맨 아래 하단에 깃 저장소 띄워"): bottom-left repo/branch/commit text."""
import base64
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'openpilot/selfdrive/carrot/cluster'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import hud_protocol as P  # noqa: E402
import hud_health as H  # noqa: E402

REMOTE = 'https://github.com/2020K9/carrotpilot.git'


def test_same_commit_is_one_name():
  assert H.repo_label(REMOTE, 'carrot-wip', 'af8a7e0c1234', 'af8a7e0cffff') == '2020K9/carrot-wip · af8a7e0c'


def test_different_commits_show_both():
  assert H.repo_label(REMOTE, 'carrot-wip', 'af8a7e0c1234', 'b14fafe4aaaa') == 'C 2020K9/carrot-wip af8a7e0c / J b14fafe4'


def test_missing_sides():
  assert H.repo_label(REMOTE, 'carrot-wip', 'af8a7e0c', '') == 'C 2020K9/carrot-wip af8a7e0c / J -'
  assert H.repo_label(None, None, None, 'b14fafe4') == 'J b14fafe4'
  assert H.repo_label(None, None, None, '') == ''


def test_owner_forms():
  assert H.repo_owner('git@github.com:2020K9/carrotpilot.git') == '2020K9'
  assert H.repo_owner(b'https://github.com/ajouatom/openpilot') == 'ajouatom'


def test_comma_sends_git_params_within_limit():
  for key in ('GitRemote', 'GitBranch', 'GitCommit'):
    assert key in P.PARAMS
  assert len(P.display_param('GitRemote', REMOTE.encode())) <= P.PARAM_LIMIT
  assert len(P.display_param('GitCommit', b'0' * 40)) <= P.PARAM_LIMIT


def test_snapshot_keeps_last_value():
  b = lambda v: base64.b64encode(v.encode()).decode()
  got = H.snapshot_git((0., {'params': {'GitBranch': b('carrot-wip'), 'GitCommit': b('af8a7e0c')}}), {})
  assert got == {'GitBranch': 'carrot-wip', 'GitCommit': 'af8a7e0c'}
  assert H.snapshot_git(None, got) == got
  assert H.snapshot_git((0., {'params': {'GitCommit': 5}}), got) == got


def test_jetson_commit_file(tmp_path):
  f = tmp_path / 'SOURCE_COMMIT'
  f.write_text('9d444afa0a729935\n')
  assert H.read_jetson_commit(f) == '9d444afa0a729935'
  assert H.read_jetson_commit(tmp_path / 'none') == ''
