"""K9ExtHud=1 C1 meter, meter_empty2 render checks (GL display needed, e.g. xvfb-run), 1920x462: the 빈차 screen (its
own look, like the 정산 screen: panel, red '빈 차' sign, fare guide, 포로리 + 보노보노) with every text inside the card,
no text on another text, on the sign's edge or on a character (겹침 검사), the carrot wheel still not drawn, the clam
still the steering display, the LFA badge there, C1 atlases on whole pixels. New tests only."""
import os
import sys
from pathlib import Path

import numpy as np
import pytest

TESTS = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS.parent / "cluster"))
sys.path.insert(0, str(TESTS))

pytestmark = pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")

from test_k9_ext_hud_c1_bono_think_render import T, _open, _png, _sprite_mask, _Spy, _state  # noqa: E402


def _empty(p, since=10.0, ev="empty", steer=None):
  f = p.fsm
  p.fsm_mode = True
  p._clock = lambda: T
  p.freeze_animation_t = None
  f.state, f.ev, f.t, f.since = "EMPTY", ev, 100.0, 100.0 - since
  return 0.0


def _render(p, r, monkeypatch, since=10.0, ev="empty", state=None):
  import cluster_renderer
  _empty(p, since, ev)
  r.render_to_png_bytes(state or _state(0.0))
  spy = _Spy(cluster_renderer.rl)
  monkeypatch.setattr(cluster_renderer, "rl", spy)
  _empty(p, since, ev)
  img = _png(r.render_to_png_bytes(state or _state(0.0)))
  monkeypatch.setattr(cluster_renderer, "rl", spy._rl)
  return spy, img


def _text_boxes(p, r, spy, cx, cy):
  import cluster_renderer
  out = []
  for _base, text, x, y, size in spy.txt:
    w, h = p._measure(r, cluster_renderer.rl, text, size)
    out.append((text, x - cx, y - cy, x - cx + w, y - cy + h * 0.92, size))
  return out


def _hit(a, b):
  return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


@pytest.mark.parametrize("season,hour,since", [(3, 21, 7.0), (1, 12, 1.0), (4, 23, 7.0), (2, 17, 4.0), (2, 12, 7.0)])
def test_empty_screen_layout_no_overlaps(monkeypatch, season, hour, since):
  import cluster_taxi_c1 as c1
  from PIL import Image
  r, p = _open(season, hour)
  try:
    spy, img = _render(p, r, monkeypatch, since)
    dx, dy, k, cx, cy = p._dev
    texts = [t[1] for t in spy.txt]
    assert "빈 차" in texts and c1.EMPTY_WAIT in texts and c1.EMPTY_HINT in texts and c1.EMPTY_BRAND in texts
    assert f"{c1.format_won(p.base_fare_won)}원" in texts and "기본요금" in texts and "심야 할증" in texts
    assert "운 행  요 금" not in texts and "다음 " not in texts and "총 요 금" not in texts   # not the card / 정산
    bubble = {1.0: "또 만나요!", 7.0: c1.EMPTY_CALL[0]}.get(since)
    assert p.last_bubble == (bubble or "")
    if hour == 23:
      assert p.night_rate == pytest.approx(0.3) and "6,240원" in texts and any("지금 적용" in t for t in texts)
    else:
      assert "4,800원" in texts and not any("지금 적용" in t for t in texts)
    boxes = [bx for bx in _text_boxes(p, r, spy, cx, cy) if -40 < bx[1] < c1.CARD_W + 40 and -40 < bx[2] < c1.CARD_H + 40]
    on_sign = {("빈 차", 98.0), (c1.EMPTY_WAIT, 17.0)}
    assert on_sign <= {(b[0], b[5]) for b in boxes}
    for name, x0, y0, x1, y1, _ in boxes:
      assert 16 <= x0 and x1 <= c1.CARD_W - 16 and 16 <= y0 and y1 <= c1.CARD_H - 8, (name, x0, y0, x1, y1)
    uniq = {}
    for bx in boxes:
      uniq.setdefault((bx[0], round(bx[1]), round(bx[2])), bx)
    bl = list(uniq.values())
    for i, a in enumerate(bl):
      for bb in bl[i + 1:]:
        if a[0] == bb[0] and a[5] == bb[5]:
          continue
        assert not _hit(a[1:5], bb[1:5]), (a, bb)
    # the sign: its two lines inside it; nothing else on it
    sx, sy, sw, sh = c1.EMPTY_SIGN
    sign = (sx, sy, sx + sw, sy + sh)
    for name, x0, y0, x1, y1, size in bl:
      inside = sign[0] + 4 <= x0 and x1 <= sign[2] - 4 and sign[1] + 4 <= y0 and y1 <= sign[3] - 4
      if (name, size) in on_sign:
        assert inside, (name, x0, y0, x1, y1)
      else:
        g = c1.EMPTY_GLOW
        assert not _hit((x0, y0, x1, y1), (sign[0] - g, sign[1] - g, sign[2] + g, sign[3] + g)), name
    # the bubble (incl. its tail) is clear of every other text and of 보노보노
    if bubble:
      tw = p._measure(r, __import__("cluster_renderer").rl, bubble, 17)[0]
      bx, by, w, h, right = c1.bubble_rect(tw, c1.SETTLE_PORO[0])
      bub = (bx - 11, by + c1.SETTLE_BUBBLE_DY, bx + w + 11, by + c1.SETTLE_BUBBLE_DY + h)
      for name, x0, y0, x1, y1, _ in bl:
        if name != bubble:
          assert not _hit(bub, (x0, y0, x1, y1)), (name, bub)
      assert not _hit(bub, (612.0, 242.0, 726.0, 404.0)) and not _hit(bub, sign)
    # no text over 보노보노 / 포로리 (sprite alpha masks)
    H, W = img.shape[:2]
    masks = np.zeros((H, W), bool)
    for name in ("bono_body", "porori_body"):
      tid = int(p._textures[name].id)
      alpha = np.asarray(Image.open(c1.C1_ASSET_DIR / f"{name}.png").convert("RGBA"))[:, :, 3]
      for call in (c for c in spy.tex if c[0] == tid):
        masks |= _sprite_mask(alpha, call, (H, W), (cx - dx, cy - dy))
    assert masks.sum() > 3000
    for name, x0, y0, x1, y1, _ in boxes:
      if name == bubble:
        continue   # the bubble sits next to 포로리 (its own box is checked above)
      sub = masks[int(dy + y0):int(dy + y1), int(dx + x0):int(dx + x1)]
      assert not sub.any(), name
    # the LFA badge is clear of every text
    lb = c1.LFA_BADGE
    for name, x0, y0, x1, y1, _ in bl:
      assert not _hit((lb[0], lb[1], lb[0] + lb[2], lb[1] + lb[3]), (x0, y0, x1, y1)), name
    # the sign is red and lit, the panel there
    sp = img[int(dy + sy + 8):int(dy + sy + sh - 8), int(dx + sx + 8):int(dx + sx + sw - 8)]
    red = (sp[:, :, 0] > 180) & (sp[:, :, 1] < 140) & (sp[:, :, 2] < 130)
    assert red.sum() > 1500
  finally:
    r.close()


def test_empty_screen_wheel_hidden_clam_steers_lfa_badge(monkeypatch):
  r, p = _open(3, 21)
  try:
    wheel = {int(t.id) for t in (r._lfa_texture, getattr(r, "_lfa_active_texture", None)) if t is not None}
    for deg in (0.0, 33.0, -120.0):
      st = _state(0.0)
      from dataclasses import replace
      st = replace(st, steering_angle_deg=deg)
      spy, _ = _render(p, r, monkeypatch, state=st)
      assert p.display_state == "EMPTY"
      assert not [1 for t in spy.tex if t[0] in wheel]                    # no carrot wheel (K9ExtHud=1)
      shell = int(p._textures["shell"].id)
      clam = [t for t in spy.tex if t[0] == shell]
      assert len(clam) == 1 and clam[0][4] == pytest.approx(-deg)         # the clam = -steeringAngleDeg, 1:1
  finally:
    r.close()


def test_empty_screen_text_on_whole_pixels_and_atlases(monkeypatch):
  r, p = _open(3, 21)
  try:
    spy, _ = _render(p, r, monkeypatch, since=7.0)
    cx, cy = p._dev[3], p._dev[4]
    mine = [t for t in spy.txt if t[0] in p.C1_FONT_SIZES and t[0] == round(t[4])]
    assert len(mine) >= 14
    for base, text, x, y, size in mine:
      assert abs(size - base) < 1e-6, text
      assert abs((x - cx) - round(x - cx)) < 1e-6 and abs((y - cy) - round(y - cy)) < 1e-6, (text, x, y)
  finally:
    r.close()


def test_empty_is_the_start_screen_and_the_buttons_leave_it():
  import cluster_taxi_c1 as c1
  r, p = _open(3, 21)
  try:
    r.render_to_png_bytes(_state(30.0))
    assert p.display_state == "EMPTY" and p.last_bubble == ""            # HUD start: 빈차 screen, no '또 만나요!'
    r.taxi_meter_command("start")
    r.render_to_png_bytes(_state(30.0))
    assert p.display_state == "DRIVE"
    r.taxi_meter_command("empty")
    r.render_to_png_bytes(_state(30.0))
    assert p.display_state == "EMPTY" and p.fsm.ev == "empty" and r.taxi_meter_state()["state"] == "EMPTY"
    assert r.taxi_meter_state()["fare"] == 4800 and r.taxi_meter_state()["km"] == 0.0
    assert c1.EMPTY_CALL[0] != c1.SAY["EMPTY"][0]
  finally:
    r.close()
