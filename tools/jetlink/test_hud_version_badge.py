"""11/26 06:18 (대감님): bottom-left C4 / J version lines - owner 2020K9 일치/불일치, commit, J 같음/다름 against the
comma's own HEAD / release.json, remote only when it could be checked ('확인 불가' otherwise)."""
import json
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'openpilot/selfdrive/carrot/cluster'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cluster_version_badge as VB  # noqa: E402
import hud_health as H  # noqa: E402

K9 = 'https://github.com/2020K9/carrotpilot'
OTHER = 'https://github.com/ajouatom/openpilot.git'
A = '2da31047e0382ab2eefb6aa6c54d7ede4effd073'
B = '9b82fa227cb86ea40e8067978b19ca5e5b502052'
MODEL = '01effc679eb46d6036ce0302673a87fe2dcfcc95a679daeeb929679e53c3647a'
REL = {'source_commit': A, 'model': {'sha256': MODEL}}


def kinds(line):
  return [k for _, k in line if k != 'text']


def test_owner_match_and_mismatch():
  assert VB.line_text(VB.c4_line('carrot-wip', K9, A, {'state': 'ok', 'sha': A})) == \
    'C4 carrot-wip 2020K9 일치 2da31047 원격 최신'
  bad = VB.c4_line('carrot-wip', OTHER, A)
  assert VB.line_text(bad) == 'C4 carrot-wip ajouatom 불일치 2da31047 원격 확인 불가'
  assert kinds(bad) == ['bad', 'unknown']
  assert VB.owner('git@github.com:2020K9/carrotpilot.git') == '2020K9'
  assert VB.owner('https://u:***@github.com/2020K9/carrotpilot') == '2020K9'
  assert kinds(VB.c4_line('', '', ''))[0] == 'unknown'


def test_jetson_same_and_different():
  same = VB.j_line('N16 Nav', K9, A, MODEL, '0' * 40, REL)
  assert VB.line_text(same) == 'J N16 2020K9 일치 2da31047 같음' and kinds(same) == ['ok', 'ok']
  assert kinds(VB.j_line('N16 Nav', K9, A, MODEL, A, None)) == ['ok', 'ok']   # = C4 HEAD
  diff = VB.j_line('N16 Nav', K9, B, MODEL, A, REL)
  assert VB.line_text(diff).endswith('9b82fa22 다름') and kinds(diff) == ['ok', 'bad']
  # same commit but another model loaded than the release.json names
  assert kinds(VB.j_line('LS2', K9, A, 'f' * 64, '', REL)) == ['ok', 'bad']
  # nothing on the device to compare with
  assert VB.line_text(VB.j_line('', '', A, '', '', None)) == 'J ? ? 확인 불가 2da31047 확인 불가'
  assert VB.short_model('MD v3/Flip Flop') == 'MD3/FF'


def test_remote_newer_or_unreachable():
  assert VB.remote_segment({'state': 'ok', 'sha': B}, A) == ('원격 다름', 'bad')
  assert VB.remote_segment({'state': 'behind', 'count': 3}, A) == ('원격 새판 +3', 'bad')
  assert VB.remote_segment({'state': 'unknown'}, A) == ('원격 확인 불가', 'unknown')
  assert VB.remote_segment(None, A) == ('원격 확인 불가', 'unknown')


def test_remote_head_never_blocks_without_network(tmp_path):
  repo = tmp_path / 'r'
  subprocess.run(['git', 'init', '-q', '-b', 'carrot-wip', str(repo)], check=True)
  subprocess.run(['git', '-C', str(repo), 'remote', 'add', 'origin', 'http://127.0.0.1:9/none.git'], check=True)
  head = VB.RemoteHead(repo, 'carrot-wip', timeout_s=2.)
  t = time.monotonic()
  assert head.status() == {'state': 'unknown'}
  assert time.monotonic() - t < .1
  head._worker.join(5)
  assert head.status() == {'state': 'unknown'}   # unreachable -> 확인 불가, never a guess


def test_remote_head_reads_local_origin(tmp_path):
  origin, repo = tmp_path / 'o', tmp_path / 'r'
  subprocess.run(['git', 'init', '-q', '-b', 'carrot-wip', str(origin)], check=True)
  subprocess.run(['git', '-C', str(origin), '-c', 'user.email=a@b', '-c', 'user.name=a', 'commit', '-q',
                  '--allow-empty', '-m', 'x'], check=True)
  subprocess.run(['git', 'clone', '-q', str(origin), str(repo)], check=True)
  sha = subprocess.check_output(['git', '-C', str(origin), 'rev-parse', 'HEAD'], text=True).strip()
  head = VB.RemoteHead(repo, 'carrot-wip')
  head.status()
  head._worker.join(5)
  assert head.status() == {'state': 'ok', 'sha': sha}


def test_comma_badge_local_reads(tmp_path):
  repo = tmp_path / 'r'
  subprocess.run(['git', 'init', '-q', '-b', 'carrot-wip', str(repo)], check=True)
  subprocess.run(['git', '-C', str(repo), 'remote', 'add', 'origin', 'https://github.com/2020K9/carrotpilot.git'],
                 check=True)
  subprocess.run(['git', '-C', str(repo), '-c', 'user.email=a@b', '-c', 'user.name=a', 'commit', '-q',
                  '--allow-empty', '-m', 'x'], check=True)
  head = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
  link = tmp_path / 'link.json'
  badge = VB.CommaVersionBadge(repo, link_status=link)
  lines = badge.lines(None, now=0.)
  assert [VB.line_text(x) for x in lines] == [f'C4 carrot-wip 2020K9 일치 {head[:8]} 원격 확인 불가']
  link.write_text(json.dumps({'peer': {'carrot_source_commit': head, 'carrot_source_origin': OTHER}}))
  lines = badge.lines(None, now=10.)
  assert VB.line_text(lines[1]) == f'J ? ajouatom 불일치 {head[:8]} 같음'


def test_jetson_hud_lines_use_snapshot():
  rec = (0., {'jetson_release': {'source_commit': A, 'model': {'sha256': MODEL}, 'bundle': {}},
              'comma_remote': {'state': 'ok', 'sha': B}})
  v = H.snapshot_versions(rec, {})
  assert v == {'release': REL, 'remote': {'state': 'ok', 'sha': B}}
  assert H.snapshot_versions(None, v) == v
  c4, j = H.version_lines({'remote': K9, 'branch': 'carrot-wip', 'commit': B},
                          {'remote': K9, 'commit': A, 'model': 'N16 Nav', 'model_sha': MODEL}, v)
  assert VB.line_text(c4) == 'C4 carrot-wip 2020K9 일치 9b82fa22 원격 최신'
  assert VB.line_text(j) == 'J N16 2020K9 일치 2da31047 같음'
  assert H.bundle_commit('', Path('/nonexistent/' + A)) == A
  assert H.bundle_commit('', Path('/nonexistent/x')) == ''
