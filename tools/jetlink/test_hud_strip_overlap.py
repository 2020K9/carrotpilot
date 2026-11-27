"""11/20 12:48: the Jetson top strip (stage text / overheating / warnings) never covers or touches
any other drawing (text, icon, card border). Real raylib render (needs a display, e.g. xvfb-run like the HUD tests).
11/20 23:12: the driving screen has no loading % any more (it is on the waiting screen, boot_display.py)."""
import io
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path[:0] = [str(Path(__file__).parent), str(Path(__file__).resolve().parents[2] / 'openpilot/selfdrive/carrot/cluster')]
import hud_health as hud

LONG = '아주 긴 상태 문구 ' * 30


def _h(sev, reason=None, **marks):
  d = {'severity': sev, 'updated': 1., 'hud_diag': {'startup': {'server_start': 1., **marks}}}
  if reason:
    d['reason'] = reason
  return d


ALL4 = dict(hello=2., engine_req=2., engine_ready=2., infer=2.)
STRIP_CASES = {
  'nostatus_0': ({}, {}),
  'stage_50': (_h('unknown', '모델 준비 중', hello=2., engine_req=2.), {}),
  'hot_50': ({**_h('ok', hello=2., engine_req=2.), 'thermal_severity': 'error',
              'thermal_trip': {'temp_c': 99., 'limit_c': 97.}}, {}),
  'warm_50': ({**_h('ok', hello=2., engine_req=2.), 'thermal_severity': 'warning'}, {}),
  'warn_50': (_h('warning', '모델 느림', hello=2., engine_req=2.), {}),
  'error_done': (_h('error', '모델 오류', **ALL4), {'car': 3.}),
  'long_16': (_h('unknown', LONG), {}),
  'long_en_done': (_h('unknown', LONG, **ALL4), {'car': 3.}),
}
TEXT_COLORS = (hud.WARNING, hud.ERROR, hud.NORMAL, hud.UNKNOWN)


@pytest.fixture(autouse=True)
def display_params(monkeypatch):
  # Match hud.main: use the read-only Jetson adapter, not native comma Params.
  from hud import DisplayParams
  from openpilot.selfdrive.carrot import signal_mode
  monkeypatch.setattr(signal_mode, "_params", DisplayParams())
  monkeypatch.setattr(signal_mode, "_mode", 0)
  monkeypatch.setattr(signal_mode, "_next_read", 0.)


@pytest.fixture(scope='module')
def renderer():
  from cluster_models import SimulatorInput
  from cluster_simulator import ClusterSimulator
  state = replace(ClusterSimulator().update(SimulatorInput(), 0.05), onroad=True, gear_text='D')
  r = hud.HealthRenderer(language='ko')
  r.open(hidden=True)
  yield r, state
  r.close()


def _frame(renderer, health, marks, monkeypatch, state=None):
  from PIL import Image
  r, st = renderer
  monkeypatch.setattr(hud, '_fresh', lambda *a: health)
  monkeypatch.setattr(hud.HealthRenderer, 'startup_marks', marks)
  for _ in range(2):
    png = r.render_to_png_bytes(state or st)
  return np.asarray(Image.open(io.BytesIO(png)).convert('RGB')).astype(int), r


def _on_segment(px, a, b, tol=10.):
  """distance of each pixel to the colour segment a->b (anti-aliased text over a flat background)."""
  a, b = np.asarray(a, float), np.asarray(b, float)
  ab = b - a
  t = np.clip(((px - a) @ ab) / max(ab @ ab, 1e-9), 0., 1.)
  return np.linalg.norm(px - (a + t[..., None] * ab), axis=-1) <= tol


def _freeze_system_stats(r, monkeypatch):
  # this worker's live CPU/MEM gauges (right 'system' card) are not drawn per frame: they sit in the trip-report
  # cache texture, redrawn from _system_stats once a second (TRIP_REPORT_CACHE_REFRESH_SECONDS). Freeze the
  # sample and redraw the cache from it on the first frame, so a 1 s redraw between the two screens draws the
  # same gauges (11/21: freezing the sample alone still failed 2 of 11 runs). 11/27: every pixel-identical
  # comparison of two screens needs this (test_no_strip_frame_is_unchanged[done_100] failed 3 of 20 runs without it)
  fixed = r._system_stats.sample()
  monkeypatch.setattr(r._system_stats, 'sample', lambda *a, **k: fixed)
  monkeypatch.setattr(r, '_trip_report_cache_valid', False)


@pytest.mark.parametrize('name', list(STRIP_CASES))
def test_strip_and_percent_cover_and_touch_nothing(renderer, monkeypatch, name):
  health, marks = STRIP_CASES[name]
  img, r = _frame(renderer, health, marks, monkeypatch)
  plan = r._health_strip_plan(None) if hasattr(r, '_health_strip_plan') else True
  assert plan is not None
  sy = r.height / hud.DESIGN_HEIGHT
  strip_px = int(round(32 * sy))
  bg = np.asarray(r._current_theme().bg[:3], float)
  dark = bg * (1 - 220 / 255)            # black 220 over the empty background
  band = img[:strip_px + 2].astype(float)   # the strip plus a 2 px gap: nothing else may be there
  ok = np.linalg.norm(band - bg, axis=-1) <= 6
  ok |= np.linalg.norm(band - dark, axis=-1) <= 6
  for color in TEXT_COLORS:
    ok |= _on_segment(band, dark, color)
  bad = np.argwhere(~ok)
  assert bad.size == 0, f'{name}: {len(bad)} pixels of other drawings in/next to the strip, first (y, x) {bad[:5].tolist()}'
  # the strip / % really is there (dark box) and the rest of the top row is empty background
  assert (np.linalg.norm(band[: strip_px - 2] - dark, axis=-1) <= 6).sum() > 100


@pytest.mark.parametrize('name,health,marks,alert', [
  ('done_100', _h('ok', **ALL4), {'car': 3.}, None),
  ('vehicle_alert', {}, {}, 'TAKE'),
])
def test_no_strip_frame_is_unchanged(renderer, monkeypatch, name, health, marks, alert):
  # no strip -> the normal full-size screen, pixel-identical to drawing with the strip code switched off
  r, st = renderer
  from cluster_models import ClusterAlert
  st2 = st if alert is None else replace(st, alert=ClusterAlert('TAKE OVER', size=3))
  used = []
  outer = hud.HealthRenderer._strip_outer
  monkeypatch.setattr(hud.HealthRenderer, '_strip_outer', lambda self: used.append(1) or outer(self))
  if alert is None:
    _freeze_system_stats(r, monkeypatch)
  img, _ = _frame(renderer, health, marks, monkeypatch, st2)
  assert r._health_strip_plan(getattr(st2, 'alert', None)) is None
  assert not used   # the shrink path is never entered
  if alert is not None:
    return   # the vehicle alert animates between frames; the normal path above is the old one
  monkeypatch.setattr(hud.HealthRenderer, '_draw_health_strip', lambda *a: None)
  monkeypatch.setattr(hud.HealthRenderer, '_strip_outer', lambda *a: None)   # = old full-size path
  ref, _ = _frame(renderer, health, marks, monkeypatch, st2)
  assert np.array_equal(img, ref)


def test_k9_road_boards_keep_their_own_screen(renderer, monkeypatch):
  # the K9 boards (판 0~3) never drew the health strip; still not drawn and the screen is not shrunk
  r, st = renderer
  monkeypatch.setattr(hud.HealthRenderer, '_k9_turzx_active', lambda *a: True)
  assert not r._strip_reached(st)


@pytest.mark.parametrize('name', list(STRIP_CASES))
def test_strip_draws_only_the_stage_text_no_percent(renderer, monkeypatch, name):
  # 23:12: real render; the strip draws its stage/overheat/warning text exactly as before the %, and no % text
  health, marks = STRIP_CASES[name]
  inside, texts = [False], []
  strip, draw = hud.HealthRenderer._draw_health_strip, hud.HealthRenderer._draw_text

  def strip_spy(self, plan):
    inside[0] = True
    try:
      return strip(self, plan)
    finally:
      inside[0] = False

  def draw_spy(self, text, *a, **k):
    if inside[0]:
      texts.append(text)
    return draw(self, text, *a, **k)
  monkeypatch.setattr(hud.HealthRenderer, '_draw_health_strip', strip_spy)
  monkeypatch.setattr(hud.HealthRenderer, '_draw_text', draw_spy)
  _img, r = _frame(renderer, health, marks, monkeypatch)
  message, _color = hud.health_banner(health, 'ko')
  assert message and texts
  assert set(texts) == {r._ellipsize_text(message, 22, hud.DESIGN_WIDTH - 40)}
  assert not [t for t in texts if '%' in t or '연결 중' in t or 'connecting' in t]


def test_ok_jetson_without_car_data_has_no_strip_any_more(renderer, monkeypatch):
  # 23:12: this state (all server stages done, no car data yet) used to get a %-only strip ('ok_83');
  # without the % there is no strip: the normal full-size screen, pixel-identical to the old path
  r, _st = renderer
  _freeze_system_stats(r, monkeypatch)
  health, marks = _h('ok', **ALL4), {}
  used = []
  outer = hud.HealthRenderer._strip_outer
  monkeypatch.setattr(hud.HealthRenderer, '_strip_outer', lambda self: used.append(1) or outer(self))
  img, _ = _frame(renderer, health, marks, monkeypatch)
  assert r._health_strip_plan(None) is None and not used
  monkeypatch.setattr(hud.HealthRenderer, '_draw_health_strip', lambda *a: None)
  monkeypatch.setattr(hud.HealthRenderer, '_strip_outer', lambda *a: None)   # = old full-size path
  ref, _ = _frame(renderer, health, marks, monkeypatch)
  assert np.array_equal(img, ref)
