"""K9ExtHud=1 C1 meter, meter_improve render checks (GL display needed, e.g. xvfb-run), 1920x462 like the car:
  * render_to_rgba_frame (the JPEG USB path) gives the same pixels as render_to_rgba_buffer, flags an unchanged frame,
    and the JPEG bytes through the new path equal the old path's frame by frame;
  * the TICI direct readback (glReadPixels into two reused buffers) gives the raylib readback's bytes (emulated here
    with the desktop libGL, which has the same entry points);
  * the thought in its lower place: pixels only inside its parts, never on 보노보노 or 포로리;
  * '+100원' stays left of the stats glass for a 6-digit fare. New tests only."""
import ctypes
import ctypes.util
import io
import os
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

TESTS = Path(__file__).resolve().parent
sys.path.insert(0, str(TESTS.parent / "cluster"))
sys.path.insert(0, str(TESTS))

pytestmark = pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")

from test_k9_ext_hud_c1_bono_think_render import T, _drive, _open, _png, _sprite_mask, _Spy, _state  # noqa: E402


def _frames(r, p, n, path):
  """n frames of a drive (clock + 0.05 s each), RGBA bytes through 'buffer' (old JPEG path) or 'frame' (new)."""
  out = []
  clock = [T]
  p._clock = lambda: clock[0]
  for i in range(n):
    clock[0] += 0.05
    st = replace(_state(55.0), steering_angle_deg=40.0 * np.sin(i * 0.3))
    if path == "frame":
      with r.render_to_rgba_frame(st, portrait_upload=True) as (buf, w, h, unchanged):
        out.append((bytes(buf), w, h, unchanged))
    else:
      with r.render_to_rgba_buffer(st, portrait_upload=True) as (buf, w, h):
        out.append((bytes(buf), w, h, None))
  return out


def _setup(r, p):
  _drive(p, 9.0)
  p.fsm.t, p.fsm.since = 100.0, 30.0


def _run(n, path, season=3, hour=21):
  """one renderer at a time (raylib has one window / GL context per process)"""
  r, p = _open(season, hour)
  try:
    _setup(r, p)
    return _frames(r, p, n, path), r
  finally:
    r.close()


def test_frame_path_same_pixels_and_jpeg_as_the_buffer_path():
  from cluster_usb_display import TuringUsbDisplay
  a, _ = _run(12, "buffer")
  b, _ = _run(12, "frame")
  if True:
    ua, ub = TuringUsbDisplay(jpeg_quality=68, jpeg_encoder="pillow"), TuringUsbDisplay(jpeg_quality=68, jpeg_encoder="pillow")
    for (ba, wa, ha, _), (bb, wb, hb, unchanged) in zip(a, b):
      assert (wa, ha) == (wb, hb) == (462, 1920)
      assert ba == bb
      assert ua.encode_jpeg(ba, wa, ha) == ub.encode_jpeg(bb, wb, hb, unchanged=unchanged)
    assert not any(f[3] for f in b)                    # a moving drive: every frame differs


def test_unchanged_frame_flagged_and_jpeg_reused():
  from cluster_usb_display import TuringUsbDisplay
  r, p = _open(3, 21)
  try:
    _setup(r, p)
    p.freeze_animation_t = T
    p._clock = lambda: T
    st = _state(0.0)
    u = TuringUsbDisplay(jpeg_quality=68, jpeg_encoder="pillow")
    flags, jpegs, raw = [], [], []
    for _ in range(4):
      with r.render_to_rgba_frame(st, portrait_upload=True) as (buf, w, h, unchanged):
        flags.append(unchanged)
        raw.append(bytes(buf))
        jpegs.append(u.encode_jpeg(buf, w, h, unchanged=unchanged))
    same = [raw[i] == raw[i - 1] for i in range(1, 4)]
    assert flags[0] is False and flags[1:] == same     # the flag is exactly 'same bytes as the frame before'
    fresh = TuringUsbDisplay(jpeg_quality=68, jpeg_encoder="pillow")
    for i in range(4):
      assert jpegs[i] == fresh.encode_jpeg(raw[i], 462, 1920)
    if any(same):
      assert u.jpeg_reused_frames == sum(same)
  finally:
    r.close()


def _desktop_gl():
  name = ctypes.util.find_library("GL")
  if not name:
    return None
  try:
    lib = ctypes.CDLL(name)
    lib.glReadPixels, lib.glBindFramebuffer   # noqa: B018
  except (OSError, AttributeError):
    return None
  return name


def test_direct_readback_ring_matches_raylib_readback(monkeypatch):
  gl = _desktop_gl()
  if gl is None:
    pytest.skip("no desktop libGL with glReadPixels / glBindFramebuffer")
  import cluster_gles_readback
  import cluster_renderer
  real_cdll = ctypes.CDLL
  monkeypatch.setattr(cluster_gles_readback.ctypes, "CDLL",
                      lambda n, *a, **k: real_cdll(gl if n == "libGLESv2.so" else n, *a, **k))
  a, _ = _run(6, "buffer")
  monkeypatch.setattr(cluster_renderer, "create_tici_direct_readback", cluster_gles_readback.GlesDirectReadback)
  rb, pb = _open(3, 21)
  try:
    _setup(rb, pb)
    b = _frames(rb, pb, 6, "frame")
    assert rb._direct_nv12_readback is not None and rb._rgba_frame_ring is not None   # the direct path ran
    for (ba, *_), (bb, w, h, unchanged) in zip(a, b):
      assert ba == bb and (w, h) == (462, 1920) and unchanged is False
    # same picture twice -> flagged unchanged, from the other ring buffer
    pb.freeze_animation_t = T
    pb._clock = lambda: T
    got = []
    for _ in range(3):
      with rb.render_to_rgba_frame(_state(0.0), portrait_upload=True) as (buf, w, h, unchanged):
        got.append((bytes(buf), unchanged, id(buf)))
    assert got[1][2] != got[2][2]
    assert got[2][1] == (got[2][0] == got[1][0])
  finally:
    rb.close()


def test_lower_thought_pixels_only_in_its_parts_and_off_bono_and_poro(monkeypatch):
  import cluster_renderer
  import cluster_taxi_c1 as c1
  from PIL import Image
  for season, hour in ((3, 21), (4, 12)):
    r, p = _open(season, hour)
    try:
      v = _drive(p, 3.0, progress=0.85)
      r.render_to_png_bytes(_state(v))
      spy = _Spy(cluster_renderer.rl)
      monkeypatch.setattr(cluster_renderer, "rl", spy)
      v = _drive(p, 3.0, progress=0.85)
      on = _png(r.render_to_png_bytes(_state(v)))
      calls = list(spy.tex)
      assert p.last_think == c1.TICK_THINK[0] and p.last_think_slot == "low"
      with monkeypatch.context() as m:
        m.setattr(p, "think_now", lambda *a, **k: ("", 0.0))
        v = _drive(p, 3.0, progress=0.85)
        off = _png(r.render_to_png_bytes(_state(v)))
      monkeypatch.setattr(cluster_renderer, "rl", spy._rl)
      dx, dy, k, cx, cy = p._dev
      diff = np.abs(on - off).max(axis=2) > 2
      assert diff.sum() > 2000
      tw, _ = p._measure(r, cluster_renderer.rl, c1.TICK_THINK[0], 17)
      inside = np.zeros_like(diff)
      for bob in (-4.5, 0.0, 4.5):
        for x0, y0, x1, y1 in c1.think_parts_low(tw, bob):
          inside[int(dy + y0 - 1.5):int(dy + y1 + 2.5), int(dx + x0 - 1.5):int(dx + x1 + 2.5)] = True
      assert not (diff & ~inside).any()
      for name in ("bono_body", "porori_body", "porori_tail"):
        tid = int(p._textures[name].id)
        alpha = np.asarray(Image.open(c1.C1_ASSET_DIR / f"{name}.png").convert("RGBA"))[:, :, 3]
        for call in (c for c in calls if c[0] == tid):
          mask = _sprite_mask(alpha, call, diff.shape, (cx - dx, cy - dy))
          assert mask.sum() > 300 and not (mask & diff).any(), name
    finally:
      r.close()


def test_plus_step_stays_left_of_the_stats_glass_for_a_six_digit_fare(monkeypatch):
  import cluster_renderer
  import cluster_taxi_c1 as c1
  r, p = _open(3, 23)
  try:
    for fare_units in (5, 1150):
      v = _drive(p, 0.3)
      m = p.meter
      m.units = fare_units
      m.metered_m = m.fare.base_distance_m + (fare_units + 0.4) * m.fare.distance_unit_m
      m.fare_won_exact = m.fare.base_fare_won + fare_units * m.fare.distance_unit_won
      r.render_to_png_bytes(_state(v))
      spy = _Spy(cluster_renderer.rl)
      monkeypatch.setattr(cluster_renderer, "rl", spy)
      r.render_to_png_bytes(_state(v))
      monkeypatch.setattr(cluster_renderer, "rl", spy._rl)
      cx = p._dev[3]
      plus = [t for t in spy.txt if t[1].startswith("+") and t[1].endswith("원") and t[4] == 30.0]
      assert len(plus) == 1
      w, _ = p._measure(r, cluster_renderer.rl, plus[0][1], 30)
      right = plus[0][2] - cx + w
      assert right <= c1.NO_PARTICLE_BOXES[2][0] - 8.0 + 0.5, (fare_units, right)
      if fare_units == 5:   # a 4-5 digit fare keeps the old place (x 372 / right of '원')
        import math
        fw, _ = p._measure(r, cluster_renderer.rl, c1.format_won(p.shown_fare()), 98.0 * (1.0 + 0.09 * math.sin(0.3 / 0.5 * math.pi)))
        assert abs((plus[0][2] - cx) - max(372.0, 24 + fw + 60)) < 1.0
  finally:
    r.close()
