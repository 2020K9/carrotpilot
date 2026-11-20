"""No two HUD elements overlap (대감님 2026-11-20 12:28: in the classic HUD the battery gauge ran into the steering
lines that 88540720 made 24 px). Every text draw is recorded as its glyph ink box (glyph metrics, after the current
matrix) plus the side gauge boxes; strokes / shadows of one text are merged into that text's box. Any two boxes that
intersect = fail, on the classic screen (EV6, both panel layouts, all four side gauges) and K9HudNavdy 0 / 1 / 2 / 3.
GL checks need xvfb-run."""
import inspect
import os
import sys
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import cluster_renderer as cr

pytestmark = pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")

LAT = {"applied_steer_ratio": 12.9, "applied_steer_delay_s": 0.2, "delay_learn_perc": 63, "delay_learn_s": 0.24,
       "torque_learn_perc": 61, "torque_learn_lat_accel_factor": 2.5, "torque_learn_friction": 0.13,
       "torque_filtered_lat_accel_factor": 2.745, "torque_filtered_friction": 0.1046, "learned_steer_ratio": 13.4167}
HELPERS = {"_draw_text", "_draw_text_with_stroke", "_draw_stroked_text_raw", "draw_at", "_k9l_text", "hook"}
GAUGES = {"_draw_level_gauge", "_draw_accel_block", "_draw_bipolar_gauge", "_draw_steering_output_block"}


class Recorder:
  def __init__(self):
    self.items = None
    self.depth = 0

  def point(self, x, y):
    rl = cr.rl
    if self.depth > 0:
      t = rl.rl_get_matrix_transform()
      x, y = t.m0 * x + t.m4 * y + t.m12, t.m1 * x + t.m5 * y + t.m13
    m = rl.rl_get_matrix_modelview()
    return m.m0 * x + m.m4 * y + m.m12, m.m1 * x + m.m5 * y + m.m13

  def add(self, name, kind, x0, y0, x1, y1):
    a, b = self.point(x0, y0), self.point(x1, y1)
    self.items.append((name, kind, min(a[0], b[0]), min(a[1], b[1]), max(a[0], b[0]), max(a[1], b[1])))


def _caller():
  return next((f.function for f in inspect.stack()[2:9] if f.function not in HELPERS), "?")


@pytest.fixture(scope="module")
def hud():
  import render_gear_screens as rg
  from openpilot.common import indicator_display
  from openpilot.selfdrive.carrot import signal_mode
  rl = cr.rl
  rec = Recorder()
  saved = (indicator_display.current, signal_mode.signal_mode, cr.EV6_WEATHER.snapshot, cr.time, rl.draw_text_ex,
           rl.rl_push_matrix, rl.rl_pop_matrix, cr.ClusterUiRenderer._rounded_rect)
  draw_text_ex, push, pop, rounded = saved[4:]
  indicator_display.current, signal_mode.signal_mode, cr.EV6_WEATHER.snapshot = (lambda: None), (lambda: 0), (lambda: None)
  fixed = time.struct_time((2026, 11, 18, 12, 0, 0, 2, 322, -1))
  cr.time = SimpleNamespace(monotonic=lambda: 10000.0, perf_counter=time.perf_counter, localtime=lambda *a: fixed,
                            strftime=time.strftime, time=lambda: 1794970800.0, sleep=time.sleep, thread_time=time.thread_time)

  def hook(font, text, pos, size, spacing, color):
    t = text if isinstance(text, str) else bytes(text).decode()
    # fill only (black / dark strokes and shadows are the same text a few px off, merged below)
    if rec.items is not None and t.strip() and getattr(color, "a", 255) > 40 and color.r + color.g + color.b > 60:
      sc = size / max(1, font.baseSize)
      top, bottom = 1e9, -1e9
      for ch in t:
        if ch.isspace():
          continue
        gi = rl.get_glyph_index(font, ord(ch))
        top = min(top, font.glyphs[gi].offsetY * sc)
        bottom = max(bottom, (font.glyphs[gi].offsetY + font.recs[gi].height) * sc)
      if top < bottom:
        w = rl.measure_text_ex(font, t, size, spacing).x
        rec.add(t, _caller(), pos.x, pos.y + top, pos.x + w, pos.y + bottom)
    return draw_text_ex(font, text, pos, size, spacing, color)

  def rounded_hook(self, x, y, w, h, *a, **k):
    if rec.items is not None and _caller() in GAUGES:
      rec.add("gauge", _caller(), x, y, x + w, y + h)
    return rounded(self, x, y, w, h, *a, **k)

  def push_hook():
    rec.depth += 1
    push()

  def pop_hook():
    rec.depth -= 1
    pop()

  rl.draw_text_ex, rl.rl_push_matrix, rl.rl_pop_matrix = hook, push_hook, pop_hook
  cr.ClusterUiRenderer._rounded_rect = rounded_hook
  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  r._stroked_text_texture_cache_enabled = False   # every text through draw_text_ex
  r._raw_stroked_text_enabled = False
  try:
    yield r, rec, rg
  finally:
    r.close()
    (indicator_display.current, signal_mode.signal_mode, cr.EV6_WEATHER.snapshot, cr.time, rl.draw_text_ex,
     rl.rl_push_matrix, rl.rl_pop_matrix, cr.ClusterUiRenderer._rounded_rect) = saved


def boxes(hud, state):
  r, rec, _ = hud
  r.render_to_png_bytes(state)
  rec.items = []
  try:
    r.render_to_png_bytes(state)
    items = rec.items
  finally:
    rec.items = None
  out = []
  for it in items:  # one box per text: its stroke / glow / shadow copies sit within a few px
    for i, b in enumerate(out):
      if b[:2] == it[:2] and abs(b[2] - it[2]) < 4 and abs(b[3] - it[3]) < 4:
        out[i] = (*b[:2], min(b[2], it[2]), min(b[3], it[3]), max(b[4], it[4]), max(b[5], it[5]))
        break
    else:
      out.append(it)
  return out


def overlaps(bs):
  bad = []
  for i, p in enumerate(bs):
    for q in bs[i + 1:]:
      if "gauge" in (p[0], q[0]) and p[1] == q[1]:
        continue  # one gauge's own outline / fill / value / label belong together
      if min(p[4], q[4]) > max(p[2], q[2]) and min(p[5], q[5]) > max(p[3], q[3]):
        bad.append((p, q))
  return bad


def _set_menu(r, menu):
  r.set_k9_hud_style(min(menu, 1))   # the same calls main.py makes for K9HudNavdy
  r.set_k9_hud_navdy(1)
  r.set_k9_hud_res(menu in (1, 2))
  r.set_k9_hud_3d(menu == 3)
  r.set_k9_hud_menu(menu)


def test_recorder_sees_the_old_overlap(hud):
  """the check itself: the 11/20 classic layout (battery 248..346) is caught"""
  r, _, rg = hud
  _set_menu(r, 0)
  old = (cr.SIDE_GAUGE_TOP, cr.SIDE_GAUGE_BOTTOM, cr.SIDE_GAUGE_LOWER_TOP, cr.SIDE_GAUGE_LOWER_BOTTOM, cr.SIDE_GAUGE_VALUE_Y)
  cr.SIDE_GAUGE_TOP, cr.SIDE_GAUGE_BOTTOM, cr.SIDE_GAUGE_LOWER_TOP, cr.SIDE_GAUGE_LOWER_BOTTOM, cr.SIDE_GAUGE_VALUE_Y = \
    88, 186, 248, 346, 64
  try:
    bad = overlaps(boxes(hud, replace(rg.synthetic_state("D"), **LAT)))
  finally:
    (cr.SIDE_GAUGE_TOP, cr.SIDE_GAUGE_BOTTOM, cr.SIDE_GAUGE_LOWER_TOP, cr.SIDE_GAUGE_LOWER_BOTTOM,
     cr.SIDE_GAUGE_VALUE_Y) = old
  assert any({p[1], q[1]} == {"_draw_lat_values", "_draw_level_gauge"} for p, q in bad), bad


@pytest.mark.parametrize("extra", ("plain", "all_gauges"))
@pytest.mark.parametrize("layout", ("left", "right"))
def test_classic_no_overlap(hud, layout, extra):
  from cluster_config import CLUSTER_PANEL_LAYOUT_DRIVING_LEFT, CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT
  r, _, rg = hud
  _set_menu(r, 0)
  r.set_panel_layout(CLUSTER_PANEL_LAYOUT_DRIVING_LEFT if layout == "left" else CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT)
  st = replace(rg.synthetic_state("D"), **LAT)
  if extra == "all_gauges":
    st = replace(st, steering_output=-120.0, steering_output_normalized=-0.4, steering_output_kind="torque",
                 urea_gauge=0.72, accel_mps2=-1.2)
  try:
    bs = boxes(hud, st)
    assert len(bs) > 20
    assert any(b[1] == "_draw_lat_values" for b in bs) and any(b[0] == "gauge" for b in bs)
    assert overlaps(bs) == []
  finally:
    r.set_panel_layout(CLUSTER_PANEL_LAYOUT_DRIVING_LEFT)


@pytest.mark.parametrize("menu", (0, 1, 2, 3))
def test_k9_hud_navdy_no_overlap(hud, menu):
  from test_k9_hud_3d import road_state
  r, _, rg = hud
  _set_menu(r, menu)
  try:
    for lead_d in (22.0, 45.0):
      bs = boxes(hud, replace(road_state(rg.synthetic_state("D"), lead_d=lead_d), gear_text="D", **LAT))
      assert len(bs) > 20
      assert overlaps(bs) == [], (menu, lead_d)
  finally:
    _set_menu(r, 0)
