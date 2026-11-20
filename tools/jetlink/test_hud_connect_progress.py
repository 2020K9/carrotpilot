import sys

import pytest
from pathlib import Path

sys.path[:0] = [str(Path(__file__).parent), str(Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/carrot/cluster')]
import hud_health as hud
from host_health import HostHealth


def health(**startup):
  return {'severity': 'ok', 'updated': 1., 'hud_diag': {'startup': {'server_start': 1., **startup}}}


def test_no_status_is_zero_and_waits_for_server():
  assert hud.connect_progress({}, {}, 'ko') == (0, '젯슨 연결 중 0%')
  assert hud.connect_progress({}, {}, 'en')[0] == 0


def test_each_real_stage_adds_one_sixth_in_order():
  marks = {}
  steps = [('hello', 33, '모델 요청'), ('engine_req', 50, '모델 준비'), ('engine_ready', 66, '첫 계산'),
           ('infer', 83, '차량 정보')]
  assert hud.connect_progress(health(), marks, 'ko') == (16, '젯슨 연결 중 16%')
  startup = {}
  for key, pct, _nxt in steps:
    startup[key] = 2.
    assert hud.connect_progress(health(**startup), marks, 'ko') == (pct, f'젯슨 연결 중 {pct}%')
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


def test_percent_line_has_no_stage_words():
  # the stage/status strip stays as before; the % line only carries the percent
  assert hud.connect_progress({}, {}, 'en') == (0, 'Jetson connecting 0%')
  assert '대기' not in hud.connect_progress(health(hello=2.), {}, 'ko')[1]


def test_status_strip_is_unchanged_and_percent_is_separate_line():
  src = (Path(__file__).parent / 'hud_health.py').read_text()
  body = src[src.index('def _draw_alert_overlay'):]
  assert 'message, color = progress' not in body
  # 12:30: the % now lives inside the unchanged strip (no second band under it); drawn after the strip
  assert body.index('rl.draw_rectangle(0, 0, int(DESIGN_WIDTH), 32') < body.index('self._draw_text(progress[1]')


def test_percent_line_hidden_behind_overheat_and_real_errors():
  src = (Path(__file__).parent / 'hud_health.py').read_text()
  body = src[src.index('def _draw_alert_overlay'):]
  assert "self.host_health.get('thermal_severity') not in ('warning', 'error')" in body
  assert 'progress is not None and plain and not vehicle_alert' in body


def _overlay_calls(monkeypatch, health, marks, alert=None, width=11.):
  from types import SimpleNamespace as NS
  r = object.__new__(hud.HealthRenderer)
  r.width, r.height, r.language = hud.DESIGN_WIDTH, hud.DESIGN_HEIGHT, 'ko'
  r.host_health, r.host_badge_drawn, r.startup_marks = health, True, marks
  rects, texts = [], []
  for name in ('rl_push_matrix', 'rl_pop_matrix', 'rl_scalef'):
    monkeypatch.setattr(hud.rl, name, lambda *a: None)
  monkeypatch.setattr(hud.rl, 'draw_rectangle', lambda x, y, w, h, c: rects.append((x, y, x + w, y + h)))
  monkeypatch.setattr(hud.ClusterUiRenderer, '_draw_alert_overlay', lambda *a: None)
  r._measure_text = lambda text, size, *a: (width * size / 20 * len(text), size)
  r._ellipsize_text = hud.ClusterUiRenderer._ellipsize_text.__get__(r)

  def draw(text, x, y, size, color, anchor='left'):
    w, h = r._measure_text(text, size)
    x0 = x - w / 2 if anchor == 'center' else x - w if anchor == 'right' else x
    texts.append((text, (x0, y - h / 2, x0 + w, y + h / 2), size, color))
  r._draw_text = draw
  r._draw_alert_overlay(alert or NS(size=0, text1='', text2=''))
  return rects, texts


def _hit(a, b):
  return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


@pytest.mark.parametrize('health_in,marks', [
  ({}, {}), ({'severity': 'unknown', 'reason': '모델 준비 중', 'updated': 1., 'hud_diag': {'startup': {'server_start': 1., 'hello': 2.}}}, {}),
  (health(hello=2., engine_req=2., engine_ready=2., infer=2.), {}),
  ({'severity': 'unknown', 'reason': '아주 긴 상태 문구 ' * 30, 'updated': 1., 'hud_diag': {'startup': {'server_start': 1.}}}, {})])
def test_percent_stays_inside_top_strip_and_never_overlaps(monkeypatch, health_in, marks):
  rects, texts = _overlay_calls(monkeypatch, health_in, marks)
  pct = [t for t in texts if t[0].startswith('젯슨 연결 중')]
  assert len(pct) == 1
  box = pct[0][1]
  # nothing below the 32 px top strip is covered: the + 0.00 / gauge bar / '주행 요약'·'시스템' titles stay visible
  assert all(r[3] <= 32 for r in rects) and box[1] >= 0 and box[3] <= 32
  for other in texts:
    if other is not pct[0]:
      assert not _hit(box, other[1]), other[0]
      assert other[1][1] >= 0 and other[1][3] <= 32


def test_stage_text_same_as_before_when_short(monkeypatch):
  h = {'severity': 'unknown', 'reason': '모델 준비 중', 'updated': 1., 'hud_diag': {'startup': {'server_start': 1., 'hello': 2.}}}
  rects, texts = _overlay_calls(monkeypatch, h, {})
  stage = [t for t in texts if t[0].startswith('Jetson')][0]
  assert stage[0] == 'Jetson: 모델 준비 중' and stage[2] == 22 and stage[1][1] == 16 - 11
  assert (stage[1][0] + stage[1][2]) / 2 == hud.DESIGN_WIDTH / 2
  assert rects[0] == (0, 0, int(hud.DESIGN_WIDTH), 32)


def test_100_percent_and_overheat_draw_no_percent(monkeypatch):
  done = health(hello=2., engine_req=2., engine_ready=2., infer=2.)
  assert _overlay_calls(monkeypatch, done, {'car': 3.}) == ([], [])
  hot = {**health(hello=2.), 'thermal_severity': 'error'}
  _rects, texts = _overlay_calls(monkeypatch, hot, {})
  assert [t[0] for t in texts] and not [t for t in texts if t[0].startswith('젯슨 연결 중')]
  warn = {**health(hello=2.), 'severity': 'warning', 'reason': '느림'}
  _rects, texts = _overlay_calls(monkeypatch, warn, {})
  assert not [t for t in texts if t[0].startswith('젯슨 연결 중')]
  from types import SimpleNamespace as NS
  assert _overlay_calls(monkeypatch, {}, {}, alert=NS(size=3, text1='TAKE OVER', text2='')) == ([], [])
