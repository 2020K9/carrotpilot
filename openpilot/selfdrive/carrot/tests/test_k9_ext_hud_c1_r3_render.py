"""K9ExtHud=1 C1 meter r3 render checks (GL display needed, e.g. xvfb-run).
Particles and the animated scene sprites are drawn inside a scissor at the card; nothing lands outside the card;
the bg2 HTML twinkles / drifts are drawn from the anim atlas and change over time; the carrot wheel stays hidden.
"""
import os
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

pytestmark = pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")


class _Spy:
  def __init__(self, rl):
    self._rl = rl
    self.ev = []

  def __getattr__(self, name):
    return getattr(self._rl, name)

  def begin_scissor_mode(self, x, y, w, h):
    self.ev.append(("sc+", (x, y, w, h)))
    return self._rl.begin_scissor_mode(x, y, w, h)

  def end_scissor_mode(self):
    self.ev.append(("sc-", None))
    return self._rl.end_scissor_mode()

  def draw_texture_pro(self, tex, src, dst, origin, rot, tint):
    self.ev.append(("tex", (int(tex.id), float(dst.x), float(dst.y), int(tint.a))))
    return self._rl.draw_texture_pro(tex, src, dst, origin, rot, tint)


def _state():
  from cluster_models import SimulatorInput
  from cluster_simulator import ClusterSimulator
  base = ClusterSimulator().update(SimulatorInput(), 0.05)
  return replace(base, speed_kph=40.0, onroad=True, gear_text="D", steering_angle_deg=20.0, lfa_active=True,
                 active_lane_line=True)


def _open(season, hour):
  from cluster_renderer import ClusterUiRenderer
  r = ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  r.set_taxi_meter_enabled(True)
  r.set_taxi_meter_season(season)
  p = r._taxi_meter_panel
  p._local_hour = lambda: hour
  return r, p


def _inside(ev, ids):
  """draws of the given textures, with whether they happened inside a scissor block"""
  on, out = False, []
  for k, v in ev:
    if k == "sc+":
      on = True
    elif k == "sc-":
      on = False
    elif k == "tex" and v[0] in ids:
      out.append((on, v))
  return out


def test_particles_and_anim_sprites_drawn_inside_the_card_scissor(monkeypatch):
  import cluster_renderer
  import cluster_taxi_c1 as c1
  r, p = _open(1, 21)   # spring night: 14 twinkles + petals
  try:
    r.render_to_png_bytes(_state())
    spy = _Spy(cluster_renderer.rl)
    monkeypatch.setattr(cluster_renderer, "rl", spy)
    p.freeze_animation_t = 3.0
    r.render_to_png_bytes(_state())
    ids = {int(p._textures["ptcl_white"].id), int(p._textures["anim_spring"].id)}
    d = _inside(spy.ev, ids)
    assert d and all(on for on, _ in d)
    rects = {v for k, v in spy.ev if k == "sc+"}
    cx, cy = p._last_origin if hasattr(p, "_last_origin") else (None, None)
    for x, y, w, h in rects:
      assert (w, h) == (int(c1.CARD_W), int(c1.CARD_H))
    assert sum(1 for k, _ in spy.ev if k == "sc+") == sum(1 for k, _ in spy.ev if k == "sc-")
    anim = [v for on, v in d if v[0] == int(p._textures["anim_spring"].id)]
    assert len(anim) >= 14
  finally:
    r.close()


def test_nothing_drawn_outside_the_card(monkeypatch):
  """a particle forced half outside the card at full opacity: the pixels outside the card do not change"""
  import cluster_taxi_c1 as c1
  r, p = _open(4, 12)
  try:
    p.freeze_animation_t = 2.0
    monkeypatch.setattr(c1, "particle_positions", lambda *a: [])
    a = np.frombuffer(r.render_to_rgba_bytes(_state())[0], np.uint8).copy()
    monkeypatch.setattr(c1, "particle_positions", lambda *a: [("snow", -2.0, 300.0, 0.0, (255, 0, 0), 1.0),
                                                              ("snow", c1.CARD_W + 2.0, 300.0, 0.0, (255, 0, 0), 1.0),
                                                              ("snow", 300.0, c1.CARD_H + 2.0, 0.0, (255, 0, 0), 1.0),
                                                              ("snow", 300.0, 280.0, 0.0, (255, 0, 0), 1.0)])
    monkeypatch.setattr(c1, "particle_alpha", lambda x, y: 1.0)
    buf, w, h = r.render_to_rgba_bytes(_state())
    b = np.frombuffer(buf, np.uint8).copy()
    diff = np.abs(a.astype(int) - b.astype(int)).reshape(h, w, 4).max(2)
    import cluster_renderer as cr
    ox = int(round(r._information_panel_x(cr.NAVI_LIVE_PANEL_X) + c1.CARD_OFFSET_X))
    oy = int(round(cr.NAVI_LIVE_PANEL_Y + c1.CARD_OFFSET_Y))
    cw, ch, m = int(c1.CARD_W), int(c1.CARD_H), 16
    ring = np.zeros(diff.shape, bool)                   # 16 px band around the card (the HUD elsewhere may tick)
    ring[max(0, oy - m):oy + ch + m, max(0, ox - m):ox + cw + m] = True
    ring[oy:oy + ch, ox:ox + cw] = False
    assert int((diff[ring] > 0).sum()) == 0
    inside = diff[oy:oy + ch, ox:ox + cw]
    assert (inside[270:290, 290:310] > 0).any()           # particles are drawn inside the card
  finally:
    r.close()


def _card(r, a):
  import cluster_renderer as cr
  import cluster_taxi_c1 as c1
  buf, w, h = a
  ox = int(round(r._information_panel_x(cr.NAVI_LIVE_PANEL_X) + c1.CARD_OFFSET_X))
  oy = int(round(cr.NAVI_LIVE_PANEL_Y + c1.CARD_OFFSET_Y))
  return np.frombuffer(buf, np.uint8).reshape(h, w, 4)[oy:oy + int(c1.CARD_H), ox:ox + int(c1.CARD_W)].astype(int)


def test_html_twinkles_and_drifts_move_on_screen(monkeypatch):
  import cluster_taxi_c1 as c1
  monkeypatch.setattr(c1, "particle_positions", lambda *a: [])   # only the scene animation moves
  r, p = _open(1, 21)
  try:
    for season, hour, name in ((1, 21, "spring night"), (2, 12, "summer day"), (4, 21, "winter night")):
      r.set_taxi_meter_season(season)
      p = r._taxi_meter_panel
      p._local_hour = lambda h=hour: h
      p._clock = lambda: 5000.0                       # state / bubble timers frozen too
      st = replace(_state(), speed_kph=0.0)            # stopped: fare / distance / 포로리 do not move
      p.freeze_animation_t = 0.0
      for _ in range(3):
        r.render_to_rgba_bytes(st)
      frames = []
      for t in (0.0, 0.0, 0.5, 2.0):
        p.freeze_animation_t = t
        frames.append(_card(r, r.render_to_rgba_bytes(st)))
      dd = np.abs(frames[0] - frames[1]).max(2)
      ys, xs = np.nonzero(dd)
      assert dd.max() == 0, (name, xs.min(), xs.max(), ys.min(), ys.max())     # same t -> same picture
      assert (np.abs(frames[0] - frames[3]).max(2) > 6).sum() > 30, name
      assert p.season == name.split()[0]
  finally:
    r.close()


def test_carrot_wheel_still_hidden_with_anim():
  import cluster_renderer
  r, p = _open(2, 17)
  try:
    r.render_to_png_bytes(_state())
    spy = _Spy(cluster_renderer.rl)
    cluster_renderer.rl, old = spy, cluster_renderer.rl
    try:
      r.render_to_png_bytes(_state())
    finally:
      cluster_renderer.rl = old
    wheel = {int(t.id) for t in (r._lfa_texture, getattr(r, "_lfa_active_texture", None)) if t is not None}
    assert not [1 for k, v in spy.ev if k == "tex" and v[0] in wheel]
  finally:
    r.close()
