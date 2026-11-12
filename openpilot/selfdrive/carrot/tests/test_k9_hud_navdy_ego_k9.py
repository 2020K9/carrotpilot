"""K9HudNavdy 0 and 1: the D road ego car is the K9 (대감님 11/13 05:46 "기본.고급판 둘다 k9으로넣어야지").

0 = assets/images/k9_ego_car.png (owner K9 rear render), 1 = assets/images/k9_navdy_ego.png (same K9 in Navdy grey,
make_k9_navdy_ego.py), never the generic Navdy car_12. Render checks need a GL display (xvfb-run).
"""
import os
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

import cluster_renderer as cr


def _alpha_box(im):
  return im.getchannel("A").point(lambda v: 255 if v > 20 else 0).getbbox()


def test_navdy_ego_asset_is_the_k9_rear_in_the_car_12_frame():
  ego = Image.open(cr.K9_NAVDY_EGO_PATH).convert("RGBA")
  ref = Image.open(cr.K9_NAVDY_DIR / "car_12.png").convert("RGBA")
  assert ego.size == (ref.width * 2, ref.height * 2)
  eb, rb = _alpha_box(ego), _alpha_box(ref)
  assert all(abs(e - 2 * r) <= 2 for e, r in zip(eb, rb, strict=True))  # same visible box as car_12 on screen
  # the K9 render itself: luminance pattern of k9_ego_car.png (bottom part, same aspect) matches
  src = Image.open(cr.EGO_CAR_ICON_PATH).convert("RGBA")
  sb = _alpha_box(src)
  sh = round((sb[2] - sb[0]) * (rb[3] - rb[1]) / (rb[2] - rb[0]))
  k9 = np.asarray(src.crop((sb[0], sb[3] - sh, sb[2], sb[3])).convert("L").resize((64, 52)), dtype=float)
  me = np.asarray(ego.crop(eb).convert("L").resize((64, 52)), dtype=float)
  nv = np.asarray(ref.crop(rb).convert("L").resize((64, 52)), dtype=float)

  def corr(a, b):
    return float(np.corrcoef(a.ravel(), b.ravel())[0, 1])
  assert corr(me, k9) > 0.9
  assert corr(me, nv) < corr(me, k9) - 0.3


def test_navdy_ego_asset_has_navdy_grey_tone_and_k9_lamp_band():
  ego = np.asarray(Image.open(cr.K9_NAVDY_EGO_PATH).convert("RGBA"), dtype=float)
  ref = np.asarray(Image.open(cr.K9_NAVDY_DIR / "car_12.png").convert("RGBA"), dtype=float)
  m, mr = ego[..., 3] > 128, ref[..., 3] > 128
  lum, lr = ego[..., :3][m].mean(1), ref[..., :3][mr].mean(1)
  assert np.median(lr) - 8 <= np.median(lum) <= np.median(lr) + 25  # not the black photo: the grey of the cars ahead
  assert lum.max() <= np.percentile(lr, 95) * 1.25 + 1
  red = m & (ego[..., 0] > ego[..., 1] + 20) & (ego[..., 0] > ego[..., 2] + 20)
  assert red.sum() > 500  # K9 tail lamps
  ys = np.nonzero(red)[0]
  assert ys.min() > ego.shape[0] * 0.35 and ys.max() < ego.shape[0] * 0.95  # lamps in the trunk band, not the roof
  grey = m & ~red
  assert np.abs(ego[..., 0][grey] - ego[..., 2][grey]).max() <= 1  # everything else neutral grey


@pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")
def test_both_navdy_menus_draw_the_k9_as_ego(monkeypatch):
  import render_gear_screens as rg
  paths = {}  # texture id -> file it was loaded from
  orig_load = cr.rl.load_texture

  def spy_load(path, *a, **k):
    tex = orig_load(path, *a, **k)
    paths[int(tex.id)] = Path(str(path)).resolve()
    return tex

  monkeypatch.setattr(cr.rl, "load_texture", spy_load)
  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  seen = {}
  orig_draw = cr.rl.draw_texture_pro
  cur = {"key": None}

  def spy_draw(tex, *a, **k):
    if cur["key"] is not None:
      seen.setdefault(cur["key"], set()).add(int(tex.id))
    return orig_draw(tex, *a, **k)

  monkeypatch.setattr(cr.rl, "draw_texture_pro", spy_draw)
  real_ego = cr.ClusterUiRenderer._k9_turzx_draw_ego
  try:
    st = replace(rg.synthetic_state("D"), car_fingerprint="KIA_K9", onroad=True)
    def ego_only():
      cur["key"] = (r.k9_hud_navdy, r.k9_hud_style)  # the menu / style this frame is drawn with
      seen.setdefault(cur["key"], set())
      try:
        return real_ego(r)
      finally:
        cur["key"] = None

    monkeypatch.setattr(r, "_k9_turzx_draw_ego", ego_only)
    for nav in (0, 1):
      r.set_k9_hud_navdy(nav)
      for style in (0, 1):
        r.k9_hud_style = style
        r.render_to_png_bytes(st)
        r.render_to_png_bytes(st)
    assert seen, "K9 D ego never drawn"
    want = {0: cr.EGO_CAR_ICON_PATH.resolve(), 1: cr.K9_NAVDY_EGO_PATH.resolve()}
    for (nav, style), ids in seen.items():
      assert ids, (nav, style)
      assert {paths.get(i) for i in ids} == {want[nav]}, (nav, style, {paths.get(i) for i in ids})  # K9 only
    assert {n for n, _ in seen} == {0, 1}
  finally:
    r.close()
