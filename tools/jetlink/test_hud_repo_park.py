"""11/26 18:44/18:56 (대감님 "3.4번째줄 요약좀해서 넣어라 너무길자넠?", "p단화면에만 2줄로 표기해라"): the HUD
bottom-left install-path and connection lines are short, fit one line each, and are shown only on the P screen;
the R screen, the driving screen and the K9 boards show only the two C4/J lines. Real raylib render (xvfb-run)."""
import json
import sys
import time
from dataclasses import replace
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(Path(__file__).parent), str(ROOT / 'openpilot/selfdrive/carrot/cluster')]
import hud_health as H  # noqa: E402

CATALOG = ROOT / 'openpilot/selfdrive/modeld/model_catalog'
SHA = json.loads((CATALOG / 'light-speed-2.json').read_text())['sha256']
FULL = '2f41618bfb627806ac4831f070c59a950900f2a3'


def test_short_lines_keep_every_part():
  lines = H.repo_lines({'remote': 'https://github.com/2020K9/carrotpilot', 'branch': 'carrot-wip', 'commit': FULL,
                        'path': '/data/openpilot'},
                       {'commit': FULL, 'path': f'/opt/carrot-jetlink/releases/{FULL}'})
  assert lines[2] == 'C /data/openpilot · J 2f41618b'
  assert H.repo_lines({'commit': '1'}, {'commit': FULL, 'path': '/opt/carrot-jetlink/current'})[2].endswith('J current')
  assert H.repo_lines({'commit': '1', 'path': '/x/y'}, {'commit': FULL, 'path': '/srv/z'})[2] == 'C /x/y · J /srv/z'
  link = {'release': '2020K9/carrotpilot', 'key': H.K9_RELEASE_KEY}
  assert H.link_line(link, '2020K9/carrotpilot', 'k9-comma4-big-light-speed-2-b4dead3e') == \
    '받기 2020K9 · 키 2020K9 · 모델 ls2-b4dead3e'
  assert H.link_line(link, '2020K9/carrotpilot', 'k9-comma4-big-light-speed-1a2b3c4d').endswith('모델 ls-1a2b3c4d')
  assert H.link_line(link, '2020K9/carrotpilot', 'k9-comma4-big-nav-n16-96244b1f').endswith('모델 n16-96244b1f')
  assert H.link_line(link, '2020K9/carrotpilot', 'k9-comma4-big-md-v3-1365126a').endswith('모델 md3-1365126a')
  # another repository is never shortened to an owner
  assert H.link_line({'release': 'someone/fork', 'key': H.K9_RELEASE_KEY}, '2020K9/carrotpilot', 't') == \
    '받기 someone/fork · 키 2020K9 · 모델 2020K9 t'


@pytest.fixture(autouse=True)
def display_params(monkeypatch):
  from hud import DisplayParams
  from openpilot.selfdrive.carrot import signal_mode
  monkeypatch.setattr(signal_mode, "_params", DisplayParams())
  monkeypatch.setattr(signal_mode, "_mode", 0)
  monkeypatch.setattr(signal_mode, "_next_read", 0.)


@pytest.fixture(scope='module')
def renderer():
  from cluster_models import SimulatorInput
  from cluster_simulator import ClusterSimulator
  state = ClusterSimulator().update(SimulatorInput(), 0.05)
  r = H.HealthRenderer(language='ko')
  r.open(hidden=True)
  r._repo_next = time.monotonic() + 1e9
  r._repo_jetson = {'remote': 'https://github.com/2020K9/carrotpilot.git', 'branch': 'carrot-wip', 'commit': FULL,
                    'path': f'/opt/carrot-jetlink/releases/{FULL}'}
  r._repo_link = {'release': '2020K9/carrotpilot', 'key': H.K9_RELEASE_KEY}
  r._repo_git = {'GitRemote': 'https://github.com/2020K9/carrotpilot.git', 'GitBranch': 'carrot-wip', 'GitCommit': FULL}
  r._repo_comma = {'path': '/data/openpilot', 'model': 'Light speed 2', 'model_sha': SHA}
  yield r, state
  r.close()


def _drawn(renderer, monkeypatch, gear, k9_board=False):
  r, st = renderer
  health = {'severity': 'ok', 'updated': 1., 'hud_diag': {'model_sha256': SHA, 'startup': {'server_start': 1.}}}
  monkeypatch.setattr(H, '_fresh', lambda *a: health)
  monkeypatch.setattr(H.HealthRenderer, 'startup_marks', {'car': 3.})
  if k9_board:
    monkeypatch.setattr(H.HealthRenderer, '_k9_turzx_active', lambda *a: True)
  drawn, draw = [], H.HealthRenderer._draw_repo_lines
  monkeypatch.setattr(H.HealthRenderer, '_draw_repo_lines', lambda self, lines: drawn.append(list(lines)) or draw(self, lines))
  r.render_to_png_bytes(replace(st, onroad=True, gear_text=gear))
  assert drawn
  return drawn[-1]


def test_only_p_screen_shows_lines_3_and_4(renderer, monkeypatch):
  r, _ = renderer
  p = _drawn(renderer, monkeypatch, 'P')
  assert len(p) == 4
  assert p[2] == 'C /data/openpilot · J 2f41618b'
  assert p[3] == '받기 2020K9 · 키 2020K9 · 모델 ls2-b4dead3e'
  # each fits one line: drawn as is, never cut with an ellipsis
  spacing = max(1., H.REPO_LABEL_SIZE * .02)
  for text in p[2:]:
    assert r._measure_text(text, H.REPO_LABEL_SIZE, spacing)[0] <= H.REPO_LABEL_MAX_W
    assert r._ellipsize_text(text, H.REPO_LABEL_SIZE, H.REPO_LABEL_MAX_W) == text
  for gear, board in (('R', False), ('D', False), ('D', True)):
    lines = _drawn(renderer, monkeypatch, gear, board)
    assert lines == p[:2], (gear, board)
    assert not any(isinstance(x, str) and ('받기' in x or '/data/openpilot' in x) for x in lines)
