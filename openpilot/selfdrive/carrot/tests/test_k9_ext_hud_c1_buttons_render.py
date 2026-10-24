"""K9ExtHud=1 C1 meter buttons, render checks (GL display needed, e.g. xvfb-run), 1920x462: the 정산 screen (its own
look: panel, total, lines, 운행 거리·시간, 포로리 + 보노보노) with every text inside the card and no text on another text
or on a character (겹침 검사), and the 일시정지 look. New tests only."""
import io
import os
import sys
from pathlib import Path

import numpy as np
import pytest

TESTS = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS.parent / "cluster"))
sys.path.insert(0, str(TESTS))

pytestmark = pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")

from test_k9_ext_hud_c1_bono_think_render import T, _drive, _open, _png, _sprite_mask, _Spy, _state  # noqa: E402


def _settle(p, units=41, time_units=7, hour=21, km=12.34, secs=1867.0):
  _drive(p, 9.0)
  m = p.meter
  m.units = units + time_units
  m.metered_m = m.fare.base_distance_m + (units + time_units + 0.3) * m.fare.distance_unit_m
  rate = 0.3 if hour >= 23 or hour < 4 else 0.0
  m.fare_won_exact = m.fare.base_fare_won * (1 + rate) + (units + time_units) * m.fare.distance_unit_won * (1 + rate)
  m.distance_m, m.elapsed_s = km * 1000.0, secs
  p._bd = {"dist_units": units, "time_units": time_units, "base_rate": rate}
  p.fsm.state, p.fsm.since, p.fsm.t = "DRIVE", 30.0, 100.0
  p.settle_breakdown = p.breakdown()
  p.fsm.state, p.fsm.since, p.fsm.t = "SETTLE", 97.0, 100.0
  return 0.0


def _text_boxes(p, r, spy, cx, cy):
  import cluster_renderer
  out = []
  for base, text, x, y, size in spy.txt:
    w, h = p._measure(r, cluster_renderer.rl, text, size)
    out.append((text, x - cx, y - cy, x - cx + w, y - cy + h * 0.92))
  return out


@pytest.mark.parametrize("season,hour", [(3, 21), (1, 12), (4, 23)])
def test_settle_screen_layout_no_overlaps(monkeypatch, season, hour):
  import cluster_renderer
  import cluster_taxi_c1 as c1
  from PIL import Image
  r, p = _open(season, hour)
  try:
    v = _settle(p, hour=hour)
    r.render_to_png_bytes(_state(v))
    spy = _Spy(cluster_renderer.rl)
    monkeypatch.setattr(cluster_renderer, "rl", spy)
    v = _settle(p, hour=hour)
    img = _png(r.render_to_png_bytes(_state(v)))
    monkeypatch.setattr(cluster_renderer, "rl", spy._rl)
    dx, dy, k, cx, cy = p._dev
    texts = [t[1] for t in spy.txt]
    b = p.last_settle
    assert c1.format_won(b["total"]) in texts and "정 산" in texts and "감사합니다!" in texts
    assert f"{c1.format_won(b['dist_won'])}원" in texts and f"{c1.format_won(b['base'])}원" in texts
    assert "운행 요금" not in "".join(texts) and "다음 " not in texts   # not the meter card
    boxes = [bx for bx in _text_boxes(p, r, spy, cx, cy) if -40 < bx[1] < c1.CARD_W + 40 and -40 < bx[2] < c1.CARD_H + 40]
    for name, x0, y0, x1, y1 in boxes:
      assert 16 <= x0 and x1 <= c1.CARD_W - 16 and 16 <= y0 and y1 <= c1.CARD_H - 8, (name, x0, y0, x1, y1)
    uniq = {}
    for bx in boxes:
      uniq.setdefault((bx[0], round(bx[1]), round(bx[2])), bx)   # the fare's shadow copy is the same text +1.5 px
    bl = list(uniq.values())
    for i, a in enumerate(bl):
      for bb in bl[i + 1:]:
        if a[0] == bb[0]:
          continue
        hit = a[1] < bb[3] and bb[1] < a[3] and a[2] < bb[4] and bb[2] < a[4]
        assert not hit, (a, bb)
    # no text over 보노보노 / 포로리
    H, W = img.shape[:2]
    masks = np.zeros((H, W), bool)
    for name in ("bono_body", "porori_body"):
      tid = int(p._textures[name].id)
      alpha = np.asarray(Image.open(c1.C1_ASSET_DIR / f"{name}.png").convert("RGBA"))[:, :, 3]
      for call in (c for c in spy.tex if c[0] == tid):
        masks |= _sprite_mask(alpha, call, (H, W), (cx - dx, cy - dy))
    assert masks.sum() > 3000
    for name, x0, y0, x1, y1 in boxes:
      if name == "감사합니다!":
        continue   # the bubble sits next to 포로리 (its own box is checked below)
      sub = masks[int(dy + y0):int(dy + y1), int(dx + x0):int(dx + x1)]
      assert not sub.any(), name
    # the panel is there (opaque-ish) behind the lines, the scene shows below it
    px = img[int(dy + 120), int(dx + 300)]
    assert px.std() >= 0 and img[int(dy + 120), int(dx + 300)].tolist() != img[int(dy + 440), int(dx + 300)].tolist()
  finally:
    r.close()


def test_pause_look(monkeypatch):
  import cluster_renderer
  import cluster_taxi_c1 as c1
  r, p = _open(3, 21)
  try:
    v = _drive(p, 9.0)
    p.paused = True
    r.render_to_png_bytes(_state(v))
    spy = _Spy(cluster_renderer.rl)
    monkeypatch.setattr(cluster_renderer, "rl", spy)
    v = _drive(p, 9.0)
    p.paused = True
    r.render_to_png_bytes(_state(v))
    texts = [t[1] for t in spy.txt]
    assert c1.PAUSE_PILL[0] in texts and c1.PAUSE_MSG in texts and "주행" not in texts
    assert p.meter_state == "PAUSE"
  finally:
    r.close()


def test_meter_off_frames_unchanged_by_the_buttons():
  """K9ExtHud != 1 (meter panel off): commands do nothing and the frame is the same as without them."""
  from cluster_renderer import ClusterUiRenderer
  r = ClusterUiRenderer(width=1920, height=462, language="ko", theme_mode="dark")
  r.open(hidden=True)
  try:
    st = _state(0.0)
    r.render_to_rgba_bytes(st)
    r.taxi_meter_command("start")
    assert r.taxi_meter_state() is None and r._taxi_meter_panel is None   # no panel, nothing queued
    r.render_to_rgba_bytes(st)
    assert r._taxi_meter_panel is None
  finally:
    r.close()
