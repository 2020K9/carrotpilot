"""The Jetson driving screen (HUD) shows no loading % (대감님 11/20 23:12: it belongs on the waiting screen,
boot_display.py, see test_boot_loading.py). The server/host marks that feed that % stay; the HUD top strip is
the stage/overheat/warning strip exactly as before the % was added."""
import sys

import pytest
from pathlib import Path

sys.path[:0] = [str(Path(__file__).parent), str(Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/carrot/cluster')]
import hud_health as hud
from host_health import HostHealth


def health(**startup):
  return {'severity': 'ok', 'updated': 1., 'hud_diag': {'startup': {'server_start': 1., **startup}}}


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


def test_driving_screen_has_no_percent_code():
  # 23:12: no connect % on the driving screen at all (no function, no text, no drawing of it)
  src = (Path(__file__).parent / 'hud_health.py').read_text()
  assert not hasattr(hud, 'connect_progress') and not hasattr(hud, 'progress_box')
  for word in ('connect_progress', 'progress[1]', '연결 중', 'connecting', 'PROGRESS_'):
    assert word not in src, word
  body = src[src.index('def _draw_health_strip'):]
  assert body.count('self._draw_text(') == 1   # the stage/status text only


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


@pytest.mark.parametrize('health_in,marks', [
  ({}, {}), ({'severity': 'unknown', 'reason': '모델 준비 중', 'updated': 1., 'hud_diag': {'startup': {'server_start': 1., 'hello': 2.}}}, {}),
  (health(hello=2., engine_req=2., engine_ready=2., infer=2.), {}),
  ({'severity': 'unknown', 'reason': '아주 긴 상태 문구 ' * 30, 'updated': 1., 'hud_diag': {'startup': {'server_start': 1.}}}, {})])
def test_driving_screen_draws_no_percent(monkeypatch, health_in, marks):
  rects, texts = _overlay_calls(monkeypatch, health_in, marks)
  assert not [t for t in texts if '%' in t[0] or t[0].startswith('젯슨 연결 중')]
  message, color = hud.health_banner(health_in, 'ko')
  # only the stage/status strip as before the %: its text at the old full width (DESIGN_WIDTH - 40), or nothing
  fake = type('Measure', (), {'_measure_text': lambda self, text, size, *a: (11. * size / 20 * len(text), size)})()
  expected = [hud.ClusterUiRenderer._ellipsize_text(fake, message, 22, hud.DESIGN_WIDTH - 40)] if message else []
  assert [t[0] for t in texts] == expected
  assert len(rects) == len(expected)
  for _text, box, size, col in texts:
    assert col == color and size == 22 and box[1] >= 0 and box[3] <= 32
  assert all(r == (0, 0, int(hud.DESIGN_WIDTH), 32) for r in rects)


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
