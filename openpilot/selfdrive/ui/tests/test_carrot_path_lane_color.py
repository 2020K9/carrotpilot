"""Comma onroad path fill: blue (palette 4) / navy (5) path fill is light lime #9be15d while lane mode is active
(대감님 11/23 16:06 "예상경로추종 파란색 장판 깔리는 거 레인모드 활성화 시 연두색으로"); everything else unchanged."""
import numpy as np

from openpilot.selfdrive.ui.onroad import model_renderer as mr


def _renderer(lane):
  r = object.__new__(mr.ModelRenderer)
  r._carrot_colors = [mr.rl.Color(i, i + 1, i + 2, 120) for i in range(10)]
  r._carrot_active_lane_line = lane
  return r


def test_lane_mode_turns_only_blue_fill_lime():
  off, on = _renderer(False), _renderer(True)
  lime = mr.CARROT_LANE_MODE_PATH_FILL
  assert (lime.r, lime.g, lime.b, lime.a) == (0x9b, 0xe1, 0x5d, 120)
  for idx in range(10):
    assert off._carrot_path_color(idx) is off._carrot_colors[idx]
    if idx in (4, 5):
      assert on._carrot_path_color(idx) is lime
    else:
      assert on._carrot_path_color(idx) is on._carrot_colors[idx]


def test_lime_has_the_blue_palette_alpha():
  r = object.__new__(mr.ModelRenderer)
  mr.ModelRenderer._init_carrot(r)
  assert r._carrot_colors[4].a == mr.CARROT_LANE_MODE_PATH_FILL.a
  assert r._carrot_colors[5].a == mr.CARROT_LANE_MODE_PATH_FILL.a


def test_unset_lane_state_keeps_blue():
  r = object.__new__(mr.ModelRenderer)
  r._carrot_colors = [mr.rl.Color(i, i, i, 120) for i in range(10)]
  assert r._carrot_path_color(4) is r._carrot_colors[4]


def _strip(n=12):
  return mr.ModelPoints(projected_points=np.array([[float(i), float(i) + 0.25] for i in range(n)], dtype=np.float32))


def test_special_and_complex_modes_use_lane_fill(monkeypatch):
  fills = []
  monkeypatch.setattr(mr, "draw_polygon_solid", lambda points, color: fills.append(color))
  for lane in (False, True):
    r = _renderer(lane)
    r._path = _strip(24)
    r._draw_polygon_outline_carrot = lambda *a: None
    fills.clear()
    r._draw_special_modes_carrot(14, 14, False)
    r._draw_complex_path_carrot(14, False)
    want = mr.CARROT_LANE_MODE_PATH_FILL if lane else r._carrot_colors[4]
    assert fills and all(c is want for c in fills)
    fills.clear()
    r._draw_complex_path_carrot(13, False)  # green stays green in lane mode
    assert fills and all(c is r._carrot_colors[3] for c in fills)


def test_rainbow_stripes_keep_their_palette(monkeypatch):
  fills = []
  monkeypatch.setattr(mr, "draw_polygon_solid", lambda points, color: fills.append(color))
  r = _renderer(True)
  r._path = _strip(40)
  r._draw_polygon_outline_carrot = lambda *a: None
  r._draw_mode1_to_6_carrot(0.0, -5, 6, 14, False)
  assert mr.CARROT_LANE_MODE_PATH_FILL not in fills
  assert r._carrot_colors[4] in fills
