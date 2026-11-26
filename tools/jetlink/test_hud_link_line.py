"""11/26 00:26 (대감님 "hud에 콤마·젯슨 어디하고 연결돼 있는지"): the fourth HUD bottom-left line shows where this
Jetson takes releases from, which signing key it trusts and which release its model came from, all read from files."""
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'openpilot/selfdrive/carrot/cluster'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import hud_health as H  # noqa: E402
import release_parts  # noqa: E402

HERE = Path(__file__).resolve().parent
CATALOG = ROOT / 'openpilot/selfdrive/modeld/model_catalog'


def test_known_key_matches_release_parts():
  assert H.K9_RELEASE_KEY == release_parts.PUBLIC_KEY.hex()


def test_updater_link_reads_installed_files():
  # this checkout's updater files are what gets installed as /opt/carrot-jetlink/updater
  link = H.updater_link(HERE)
  assert link == {'release': '2020K9/carrotpilot', 'key': H.K9_RELEASE_KEY}


def test_updater_link_other_key_and_missing(tmp_path):
  (tmp_path / 'update_host.py').write_text(
    "RELEASE_HOST, RELEASE_PREFIX = 'github.com', '/someone/fork/releases/download/'\n")
  (tmp_path / 'release-signing-public.pem').write_text(
    '-----BEGIN PUBLIC KEY-----\nMCowBQYDK2VwAyEAEhNI8j6VWOnj0/JAD+CbNLSnJVZUzCgqWmE57QO8YOw=\n-----END PUBLIC KEY-----\n')
  link = H.updater_link(tmp_path)
  assert link['release'] == 'someone/fork' and link['key'].startswith('121348f2')
  assert H.link_line(link) == '받기 someone/fork · 키 121348f2 · 모델 ?'
  assert H.updater_link(tmp_path / 'none') == {'release': '', 'key': ''}
  assert H.link_line({}) == '받기 ? · 키 ? · 모델 ?'


def test_model_release_from_signed_manifests():
  for name in ('light-speed-2-release.json', 'md-v3-release.json'):
    m = json.loads((CATALOG / name).read_text())['model']
    repo, tag = H.model_release(m['sha256'], (CATALOG,))
    assert repo == '2020K9/carrotpilot' and tag and tag in m['url']
  assert H.model_release('0' * 64, (CATALOG,)) == ('', '')
  assert H.model_release(None, (CATALOG,)) == ('', '')


def test_link_line_full():
  link = {'release': '2020K9/carrotpilot', 'key': H.K9_RELEASE_KEY}
  assert H.link_line(link, '2020K9/carrotpilot', 'k9-comma4-big-light-speed-2-b4dead3e') == \
    '받기 2020K9 · 키 2020K9 · 모델 ls2-b4dead3e'
  assert H.link_line(link, 'other/repo', 't1') == '받기 2020K9 · 키 2020K9 · 모델 other/repo t1'
