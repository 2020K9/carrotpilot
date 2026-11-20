import sys
from pathlib import Path

sys.path[:0] = [str(Path(__file__).parent), str(Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/carrot/cluster')]
import hud_health as hud
from host_health import HostHealth


def health(**startup):
  return {'severity': 'ok', 'updated': 1., 'hud_diag': {'startup': {'server_start': 1., **startup}}}


def test_no_status_is_zero_and_waits_for_server():
  assert hud.connect_progress({}, {}, 'ko') == (0, '젯슨 연결 중 0% · 젯슨 서버 시작 대기')
  assert hud.connect_progress({}, {}, 'en')[0] == 0


def test_each_real_stage_adds_one_sixth_in_order():
  marks = {}
  steps = [('hello', 33, '모델 요청'), ('engine_req', 50, '모델 준비'), ('engine_ready', 66, '첫 계산'),
           ('infer', 83, '차량 정보')]
  assert hud.connect_progress(health(), marks, 'ko') == (16, '젯슨 연결 중 16% · 콤마 연결 대기')
  startup = {}
  for key, pct, nxt in steps:
    startup[key] = 2.
    assert hud.connect_progress(health(**startup), marks, 'ko') == (pct, f'젯슨 연결 중 {pct}% · {nxt} 대기')
  assert hud.connect_progress(health(**startup), {'car': 3.}, 'ko') is None


def test_stale_server_marks_do_not_count_without_fresh_status():
  # the HUD's own car mark alone is one stage; server stages need a fresh daemon snapshot
  assert hud.connect_progress({}, {'car': 3.}, 'ko')[0] == 16


def test_bad_values_are_not_done():
  assert hud.connect_progress(health(hello=float('nan'), infer=True), {'car': None}, 'ko')[0] == 16


def test_host_marks_infer_once_on_first_inference():
  h = HostHealth.__new__(HostHealth)
  h.marks, h.inference = {'server_start': 0.}, None
  h.note_inference(1000, 2000)
  first = h.marks['infer']
  h.note_inference(1000, 2000)
  assert h.marks['infer'] == first
  assert h.diagnostics(first)['startup']['infer'] == first


def test_server_marks_hello_and_engine_request():
  src = (Path(__file__).parent / 'server.py').read_text()
  assert "health.mark('hello')" in src and "health.mark('engine_req')" in src
  assert "self.telemetry.health.mark('engine_ready')" in src
