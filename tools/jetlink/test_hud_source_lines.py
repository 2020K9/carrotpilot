"""11/25 20:57 (대감님 "콤마·젯슨 사용경로 같이 띄우게해 브랜치 뭐쓰는지 모델 뭐쓰는지"): the HUD bottom-left lines
show, for the comma (C) and the Jetson (J), repository, branch, commit, model and install path; '?' when unknown."""
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'openpilot/selfdrive/carrot/cluster'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import hud_health as H  # noqa: E402

CATALOG = ROOT / 'openpilot/selfdrive/modeld/model_catalog'


def test_repo_name_reads_the_real_remote():
  assert H.repo_name('https://github.com/ajouatom/openpilot.git') == 'ajouatom/openpilot'
  assert H.repo_name('git@github.com:2020K9/carrotpilot.git') == '2020K9/carrotpilot'
  assert H.repo_name('https://user:tok@github.com/2020K9/carrotpilot') == '2020K9/carrotpilot'
  assert H.repo_name('') == ''


def test_lines_full():
  comma = {'remote': 'https://github.com/ajouatom/openpilot.git', 'branch': 'carrot-wip', 'commit': '1cb2ef7e99',
           'model': 'Light speed 2', 'model_sha': '909a1400aa', 'path': '/data/openpilot'}
  jetson = {'remote': 'https://github.com/2020K9/carrotpilot.git', 'branch': 'carrot-wip', 'commit': '2f41618bff',
            'model': 'N16 Nav', 'model_sha': '01effc67bb', 'path': '/opt/carrot-jetlink/current'}
  assert H.repo_lines(comma, jetson) == [
    'C ajouatom/openpilot carrot-wip 1cb2ef7e · Light speed 2 909a1400',
    'J 2020K9/carrotpilot carrot-wip 2f41618b · N16 Nav 01effc67',
    'C /data/openpilot · J current']


def test_unknown_is_question_mark_never_guessed():
  lines = H.repo_lines({}, {'commit': '2f41618bff'})
  assert lines == ['C ? ? ? · ?', 'J ? ? 2f41618b · ?', 'C ? · J ?']
  assert H.repo_lines({}, {}) == []
  # an old bundle has no SOURCE_ORIGIN: remote/branch unknown, not borrowed from the comma
  assert H.repo_lines({'remote': 'https://github.com/a/b', 'branch': 'x', 'commit': '12345678'},
                      {'commit': '12345678'})[1] == 'J ? ? 12345678 · ?'


def test_catalog_names_from_loaded_sha():
  specs = {k: json.loads((CATALOG / v['jetson_spec']).read_text())['sha256']
           for k, v in json.loads((CATALOG / 'catalog.json').read_text()).items()}
  assert H.catalog_names(specs['n16'], CATALOG) == 'N16 Nav'
  assert H.catalog_names(specs['light-speed-2'], CATALOG) == 'Light speed 2'
  assert H.catalog_names(specs['md-v3'], CATALOG) == 'MD v3/Flip Flop'   # one Jetson ONNX, two names
  assert H.catalog_names('0' * 64, CATALOG) == ''
  assert H.catalog_names(None, CATALOG) == ''


def test_origin_file(tmp_path):
  f = tmp_path / 'SOURCE_ORIGIN'
  f.write_text('https://github.com/2020K9/carrotpilot.git\ncarrot-wip\n')
  assert H.read_jetson_origin(f) == ('https://github.com/2020K9/carrotpilot.git', 'carrot-wip')
  assert H.read_jetson_origin(tmp_path / 'none') == ('', '')


def test_snapshot_comma_source_keeps_last():
  rec = (0., {'comma_source': {'path': '/data/openpilot', 'model': 'Light speed 2'},
              'jetson_release': {'model': {'sha256': '909a1400' + '0' * 56}}})
  got = H.snapshot_comma_source(rec, {})
  assert got == {'path': '/data/openpilot', 'model': 'Light speed 2', 'model_sha': '909a1400' + '0' * 56}
  assert H.snapshot_comma_source(None, got) == got
  assert H.snapshot_comma_source((0., {'comma_source': {'path': 5, 'model': None}}), got) == got


def test_comma_sends_source():
  import hud_protocol as P
  src = P.comma_source()
  assert set(src) == {'path', 'model'} and src['path']
  assert len(json.dumps(src)) < 300


def test_health_reports_loaded_model():
  import host_health
  h = host_health.HostHealth.__new__(host_health.HostHealth)
  h.marks, h.inference = {}, None
  assert 'model_sha256' not in h.diagnostics(0.)
  h.note_model('01effc67' + '0' * 56)
  h.note_model(None)
  assert h.diagnostics(0.)['model_sha256'] == '01effc67' + '0' * 56


def test_bundle_origin_has_no_credentials(tmp_path):
  import build_host_bundle as B
  repo = tmp_path / 'r'
  subprocess.run(['git', 'init', '-q', '-b', 'carrot-wip', str(repo)], check=True)
  subprocess.run(['git', '-C', str(repo), 'remote', 'add', 'origin', 'https://u:secret@github.com/2020K9/carrotpilot.git'],
                 check=True)
  assert B.source_origin(['git', '-C', str(repo)]) == b'https://github.com/2020K9/carrotpilot.git\ncarrot-wip\n'


def test_release_folder_commit_is_shortened():
  lines = H.repo_lines({'path': '/data/openpilot'},
                       {'commit': '2f41618b', 'path': '/opt/carrot-jetlink/releases/2f41618bfb627806ac4831f070c59a950900f2a3'})
  assert lines[2] == 'C /data/openpilot · J 2f41618b'
