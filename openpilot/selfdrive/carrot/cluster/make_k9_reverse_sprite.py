#!/usr/bin/env python3
"""Top view of the 2020 K9 (RJ) for the K9 reverse (R) screen, drawn in code (no photo, no 3D model).

Vector-style illustration like a factory surround-view car: smooth tapered nose and tail, a slight waist,
long hood with a centre crease, windshield / panoramic roof / rear glass (black glass with reflections),
chrome window frame, A/B/C pillars, two mirrors, trunk lines, a full-width tail lamp with red glow,
white headlamp ends, dark metal paint with top-lit specular highlights, a soft shadow and a faint light rim.
Plan size 5120 x 1915 mm, wheelbase 3105 mm. Drawn at SS x the output resolution, then reduced (smooth edges).
Car front to the left, car right side up (same as the R screen camera). Output: assets/images/k9_reverse_car.png
Frame: PAD metres of transparent margin around the 5120 x 1915 mm body at PPM pixels per metre.
"""
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter

L, W = 5.12, 1.915
WHEELBASE = 3.105
REAR_OVERHANG = 1.06  # body rear to rear axle (front overhang = rest)
FRONT_AXLE_U = L - REAR_OVERHANG - WHEELBASE
REAR_AXLE_U = L - REAR_OVERHANG
PPM = 200
PAD = 0.30
SS = 4
S = PPM * SS
HERE = Path(__file__).resolve().parent
OUT = HERE / "assets" / "images" / "k9_reverse_car.png"
WH = W / 2
SIZE = (int(round((L + 2 * PAD) * S)), int(round((W + 2 * PAD) * S)))


def px(u, v):
  """car (u from front bumper, v to the car right) -> supersampled pixel (x, y)."""
  return (np.asarray(u) + PAD) * S, (WH + PAD - np.asarray(v)) * S


def smooth(e0, e1, x):
  t = np.clip((np.asarray(x, float) - e0) / (e1 - e0), 0.0, 1.0)
  return t * t * (3 - 2 * t)


BOW_F, BOW_R = 0.15, 0.09


def body_half(u):
  """half width (m) of the body at u (0..L)."""
  u = np.asarray(u, float)
  nose = np.clip((u - BOW_F) / 1.00, 0, 1)
  tail = np.clip((L - BOW_R - u) / 0.62, 0, 1)
  g_f = 0.63 + 0.37 * (1 - (1 - nose) ** 2.0) ** 0.5
  g_r = 0.76 + 0.24 * (1 - (1 - tail) ** 2.0) ** 0.5
  hw = WH * g_f * g_r
  hw *= 0.986 + 0.014 * smooth(0.3, 1.4, u)  # nose slightly narrower than the tail
  hw *= 1 - 0.016 * np.exp(-((u - 2.55) / 0.75) ** 2)  # waist at the doors
  for axle in (FRONT_AXLE_U, REAR_AXLE_U):  # fender shoulders over the wheels
    hw *= 1 + 0.006 * np.exp(-((u - axle) / 0.45) ** 2)
  return np.minimum(hw, WH)


def body_outline(n=500):
  us = np.linspace(BOW_F, L - BOW_R, n)
  hw = body_half(us)
  vf, vr = hw[0], hw[-1]
  vv = np.linspace(-vf, vf, 120)
  front = [(BOW_F * (vv_i / vf) ** 2, vv_i) for vv_i in vv]  # nose bows forward to u=0 at the centre
  right = list(zip(us, hw, strict=True))
  vv = np.linspace(vr, -vr, 120)
  rear = [(L - BOW_R * (vv_i / vr) ** 2, vv_i) for vv_i in vv]
  left = list(zip(us[::-1], -hw[::-1], strict=True))
  return front + right + rear + left


def poly(points):
  xs, ys = px([p[0] for p in points], [p[1] for p in points])
  return list(zip(xs.tolist(), ys.tolist(), strict=True))


def mask_of(points):
  img = Image.new("L", SIZE, 0)
  ImageDraw.Draw(img).polygon(poly(points), fill=255)
  return np.asarray(img, np.float32) / 255.0


def line_mask(points, width_m):
  img = Image.new("L", SIZE, 0)
  ImageDraw.Draw(img).line(poly(points), fill=255, width=max(1, int(round(width_m * S))), joint="curve")
  return np.asarray(img, np.float32) / 255.0


def blur(a, radius_m):
  img = Image.fromarray(np.clip(a * 255, 0, 255).astype(np.uint8), "L")
  return np.asarray(img.filter(ImageFilter.GaussianBlur(radius_m * S)), np.float32) / 255.0


def over(dst_rgb, dst_a, rgb, a):
  """'over' composite of straight-alpha colour rgb with coverage a onto (dst_rgb, dst_a)."""
  a = np.clip(a, 0, 1)[..., None]
  rgb = np.broadcast_to(np.asarray(rgb, np.float32), dst_rgb.shape) if np.ndim(rgb) == 1 else rgb
  out_a = a + dst_a[..., None] * (1 - a)
  out_rgb = (rgb * a + dst_rgb * dst_a[..., None] * (1 - a)) / np.maximum(out_a, 1e-6)
  return out_rgb, out_a[..., 0]


# ---- greenhouse ---------------------------------------------------------------------------------------------
G_U = np.array([1.72, 1.95, 2.40, 3.00, 3.45, 3.80, 4.13])
G_V = np.array([0.735, 0.785, 0.805, 0.805, 0.780, 0.730, 0.650])
G_FRONT_BOW, G_REAR_BOW = 0.13, 0.09
ROOF_F, ROOF_R, ROOF_INSET = 2.40, 3.50, 0.105


def g_half(u):
  return np.interp(u, G_U, G_V)


def band(u0, u1, half, bow0, bow1, n=200):
  """closed shape between u0 and u1 with half width half(u); ends bow outward by bow0 / bow1 at the centre."""
  us = np.linspace(u0, u1, n)
  hw = half(us)
  pts = []
  vv = np.linspace(-hw[0], hw[0], 80)
  pts += [(u0 - bow0 * (1 - (x / hw[0]) ** 2), x) for x in vv]
  pts += list(zip(us, hw, strict=True))
  vv = np.linspace(hw[-1], -hw[-1], 80)
  pts += [(u1 + bow1 * (1 - (x / hw[-1]) ** 2), x) for x in vv]
  pts += list(zip(us[::-1], -hw[::-1], strict=True))
  return pts


def rounded_band(u0, u1, half, r, bow0=0.0, bow1=0.0, n=220):
  """band with rounded corners (radius r) - roof panel and glass insets."""
  us = np.linspace(u0, u1, n)
  hw = half(us)
  corner = np.sqrt(np.clip(1 - (1 - np.clip(np.minimum(us - u0, u1 - us) / r, 0, 1)) ** 2, 0, 1))
  hw = hw - r * (1 - corner)
  pts = list(zip(us, hw, strict=True)) + list(zip(us[::-1], -hw[::-1], strict=True))
  out = pts
  if bow0 or bow1:  # push the ends out at the centre line
    fixed = []
    for u, v in out:
      w = 1 - min(1.0, (v / max(half(u), 1e-3)) ** 2)
      if u - u0 < r:
        u -= bow0 * w
      elif u1 - u < r:
        u += bow1 * w
      fixed.append((u, v))
    out = fixed
  return out


def render():
  H_, W_ = SIZE[1], SIZE[0]
  yy, xx = np.mgrid[0:H_, 0:W_].astype(np.float32)
  U = xx / S - PAD
  V = WH + PAD - yy / S
  rgb = np.zeros((H_, W_, 3), np.float32)
  alpha = np.zeros((H_, W_), np.float32)

  outline = body_outline()
  body = mask_of(outline)

  # mirrors: teardrop on each side just behind the A-pillar base
  mirrors = []
  for s in (1, -1):
    base = float(body_half(2.02))
    pts = [(1.86, s * (base - 0.03)), (1.93, s * (base + 0.06)), (2.02, s * (base + 0.115)), (2.10, s * (base + 0.135)),
           (2.15, s * (base + 0.12)), (2.16, s * (base + 0.07)), (2.12, s * (base + 0.0)), (2.10, s * (base - 0.03))]
    xs = np.linspace(0, 1, 60)
    # smooth with a Catmull-Rom like resample
    arr = np.array(pts)
    t = np.linspace(0, 1, len(arr))
    sm = np.stack([np.interp(xs, t, arr[:, 0]), np.interp(xs, t, arr[:, 1])], 1)
    mirrors.append(mask_of([tuple(p) for p in sm]))
  mirror = blur(np.clip(mirrors[0] + mirrors[1], 0, 1), 0.006)
  car = np.clip(body + mirror, 0, 1)

  # tyres: only a thin lip shows at the wheel arches (wheel positions from the 3105 mm wheelbase)
  tyre = np.zeros_like(body)
  for axle in (FRONT_AXLE_U, REAR_AXLE_U):
    hw = float(body_half(axle))
    for s in (1, -1):
      e = np.exp(-((U - axle) / 0.27) ** 6) * np.clip(1 - np.abs(np.abs(V) - (hw + 0.004)) / 0.016, 0, 1)
      tyre = np.maximum(tyre, e * (np.sign(V) == s))

  # ---- shadow + light rim (outside the car) ----
  shadow = blur(blur(car, 0.03) > 0.02, 0.07) * 0.78
  rgb, alpha = over(rgb, alpha, (4, 6, 12), shadow)
  rim_out = np.clip(blur(car, 0.05) * 1.6 - car * 1.6, 0, 1)
  rgb, alpha = over(rgb, alpha, (150, 185, 235), rim_out * 0.30)
  rgb, alpha = over(rgb, alpha, (12, 12, 14), tyre * (1 - car))

  # ---- paint ----
  edge = blur(car, 0.16)
  shade = 0.40 + 0.60 * np.clip((edge - 0.5) * 2.0, 0, 1) ** 0.65
  base = np.array([24, 26, 31], np.float32)
  spec_col = np.array([150, 160, 178], np.float32)
  bw_u = body_half(np.clip(U, 0, L))
  vn = V / np.maximum(bw_u, 0.3)
  hood = smooth(0.05, 0.35, U) * (1 - smooth(1.45, 1.70, U))
  trunk = smooth(4.10, 4.35, U) * (1 - smooth(4.85, 5.05, U))
  spine = np.exp(-(vn / 0.45) ** 2) * (0.34 * hood + 0.30 * trunk) + 0.22 * np.exp(-((U - 0.75) / 0.35) ** 2 - ((V - 0.25) / 0.30) ** 2) * hood
  axles = np.exp(-((U - FRONT_AXLE_U) / 0.55) ** 2) + np.exp(-((U - REAR_AXLE_U) / 0.55) ** 2)
  fender = np.exp(-((np.abs(vn) - 0.82) / 0.07) ** 2) * axles * 0.55
  sill = np.exp(-((np.abs(vn) - 0.90) / 0.05) ** 2) * smooth(0.6, 1.2, U) * (1 - smooth(4.2, 4.8, U)) * 0.16
  spec = spine + fender + sill
  paint = base * shade[..., None] + spec_col * spec[..., None] * shade[..., None]
  rgb, alpha = over(rgb, alpha, paint, car)

  # thin inner rim light on the edge (top light catching the shoulder)
  inner = np.clip((1 - blur(car, 0.012) * 1.0) * car * 2.2, 0, 1)
  rgb, alpha = over(rgb, alpha, (175, 195, 225), inner * 0.55)

  def paint_lines(points, width_m, color, a, soft=0.0):
    nonlocal rgb, alpha
    m = line_mask(points, width_m)
    if soft:
      m = blur(m, soft)
    rgb, alpha = over(rgb, alpha, color, m * a * car)

  # hood: centre crease (light + dark side), two character lines, front shut line, cowl
  us = np.linspace(0.30, 1.62, 60)
  paint_lines(list(zip(us, np.full_like(us, 0.010), strict=True)), 0.010, (180, 190, 210), 0.55, 0.004)
  paint_lines(list(zip(us, np.full_like(us, -0.012), strict=True)), 0.012, (6, 7, 9), 0.55, 0.004)
  for s in (1, -1):
    us = np.linspace(0.22, 1.66, 60)
    vs = s * (0.50 + 0.17 * smooth(0.2, 1.7, us))
    paint_lines(list(zip(us, vs, strict=True)), 0.010, (150, 162, 185), 0.45, 0.006)
    paint_lines(list(zip(us, vs - s * 0.02, strict=True)), 0.012, (5, 6, 8), 0.35, 0.006)
    us = np.linspace(0.30, 1.66, 40)
    paint_lines(list(zip(us, s * (body_half(us) - 0.075), strict=True)), 0.006, (4, 5, 7), 0.6)  # hood side gap
  vv = np.linspace(-0.82, 0.82, 80)
  paint_lines([(0.24 + 0.06 * (1 - (x / 0.82) ** 2) * -1 + 0.06, x) for x in vv], 0.006, (4, 5, 7), 0.6)  # hood front gap

  # ---- greenhouse ----
  g_mask = mask_of(band(1.72, 4.13, g_half, G_FRONT_BOW, G_REAR_BOW))
  glass_dark = np.array([5, 7, 11], np.float32)
  sky = np.array([70, 84, 104], np.float32)
  rgb, alpha = over(rgb, alpha, glass_dark, g_mask)
  # cowl shadow line in front of the windshield
  vv = np.linspace(-0.74, 0.74, 80)
  paint_lines([(1.72 - G_FRONT_BOW * (1 - (x / 0.74) ** 2) - 0.03, x) for x in vv], 0.03, (3, 4, 6), 0.8, 0.012)

  # windshield reflection: brighter toward the top, one soft diagonal streak
  ws = g_mask * (1 - smooth(ROOF_F - 0.06, ROOF_F, U))
  t = smooth(1.55, ROOF_F, U)
  ws_col = glass_dark + sky * (0.18 + 0.45 * t ** 1.6)[..., None] + np.array([90, 105, 125]) * (
    np.exp(-(((U - 1.95) - V * 0.35) / 0.07) ** 2) * 0.35)[..., None]
  rgb, alpha = over(rgb, alpha, ws_col, ws * 0.95)

  # side glass: dark with a faint sky reflection along its length
  side_glass = g_mask * smooth(ROOF_F - 0.02, ROOF_F + 0.02, U)
  sg_col = glass_dark + sky * (0.18 + 0.10 * np.exp(-((U - 2.8) / 0.6) ** 2))[..., None]
  rgb, alpha = over(rgb, alpha, sg_col, side_glass)

  # roof panel (painted) with panoramic glass inset
  roof_half = lambda u: g_half(u) - ROOF_INSET  # noqa: E731
  roof = mask_of(rounded_band(ROOF_F, ROOF_R, roof_half, 0.12, 0.05, 0.04))
  roof_paint = base * 1.15 + spec_col * (0.16 + 0.30 * np.exp(-(vn / 0.5) ** 2))[..., None]
  rgb, alpha = over(rgb, alpha, roof_paint, roof)
  rgb, alpha = over(rgb, alpha, (175, 188, 210), np.clip(roof - blur(roof, 0.006), 0, 1) * 1.2)
  pano_half = lambda u: g_half(u) - ROOF_INSET - 0.035  # noqa: E731
  pano = mask_of(rounded_band(ROOF_F + 0.035, ROOF_R - 0.045, pano_half, 0.09, 0.04, 0.03))
  pano_col = glass_dark + sky * (0.10 + 0.55 * np.exp(-(((U - 2.55) * 0.8 - V * 0.6) / 0.22) ** 2))[..., None]
  pano_col = pano_col + sky * (0.20 * smooth(3.1, 3.4, U))[..., None]
  rgb, alpha = over(rgb, alpha, pano_col, pano)

  # pillars: A along the windshield sides, B crossbar, C wedge
  ws_half = g_half(np.clip(U, 1.6, 4.2)) - 0.012 - (ROOF_INSET - 0.005) * smooth(1.70, ROOF_F, U) ** 1.3
  a_pillar = g_mask * (1 - smooth(ROOF_F - 0.01, ROOF_F + 0.01, U)) * np.clip((np.abs(V) - ws_half) * S / 2.0, 0, 1)
  rgb, alpha = over(rgb, alpha, base * 1.3 + spec_col * 0.30, a_pillar)
  ws_edge = g_mask * (1 - smooth(ROOF_F - 0.01, ROOF_F + 0.01, U)) * np.clip(1 - np.abs(np.abs(V) - ws_half) * S / 6.0, 0, 1)
  rgb, alpha = over(rgb, alpha, (170, 180, 198), ws_edge * smooth(1.70, 1.80, U) * 0.7)
  for s in (1, -1):
    paint_lines([(2.98, s * (roof_half(2.98) - 0.01)), (2.98, s * (g_half(2.98) - 0.005))], 0.075, (12, 13, 16), 1.0)
  c_zone = g_mask * smooth(3.43, 3.47, U)
  rgb, alpha = over(rgb, alpha, base * 1.15 + spec_col * 0.10, c_zone)
  rear_half = lambda u: np.maximum(g_half(u) - ROOF_INSET - 0.01 * (u - ROOF_R), 0.05)  # noqa: E731
  rg = mask_of(rounded_band(ROOF_R + 0.035, 4.10, rear_half, 0.10, 0.03, 0.08))
  rg_col = glass_dark + sky * (0.12 + 0.40 * (1 - smooth(ROOF_R, 4.1, U)) ** 2)[..., None] + np.array([90, 105, 125]) * (
    np.exp(-(((U - 3.85) + V * 0.30) / 0.06) ** 2) * 0.22)[..., None]
  rgb, alpha = over(rgb, alpha, rg_col, rg)
  # chrome window frame (DLO) along both sides
  for s in (1, -1):
    us = np.linspace(1.86, 4.02, 120)
    paint_lines(list(zip(us, s * (g_half(us) + 0.004), strict=True)), 0.020, (205, 212, 224), 0.95)
    paint_lines(list(zip(us, s * (g_half(us) + 0.004), strict=True)), 0.050, (190, 205, 230), 0.18, 0.012)

  # ---- trunk ----
  vv = np.linspace(-0.66, 0.66, 80)
  paint_lines([(4.13 + G_REAR_BOW * (1 - (x / 0.65) ** 2) + 0.03, x) for x in vv], 0.02, (3, 4, 6), 0.7, 0.008)
  for s in (1, -1):
    us = np.linspace(4.20, 4.96, 30)
    paint_lines(list(zip(us, s * (body_half(us) - 0.12 - 0.04 * smooth(4.6, 4.96, us)), strict=True)), 0.006, (4, 5, 7), 0.65)
  vv = np.linspace(-0.80, 0.80, 80)
  paint_lines([(4.96 + 0.05 * (1 - (x / 0.8) ** 2), x) for x in vv], 0.006, (4, 5, 7), 0.65)  # trunk rear shut line
  paint_lines([(4.93 + 0.05 * (1 - (x / 0.8) ** 2), x) for x in vv], 0.010, (160, 172, 195), 0.35, 0.006)  # lip highlight

  # ---- lamps ----
  # tail lamp: one continuous strip along the rear edge, wrapping the corners
  tl_pts = []
  vr = float(body_half(L - BOW_R))
  for x in np.linspace(-vr + 0.02, vr - 0.02, 160):
    tl_pts.append((L - BOW_R * (x / vr) ** 2 - 0.035, x))
  tl = line_mask(tl_pts, 0.030)
  for s in (1, -1):  # corner wraps
    us = np.linspace(4.86, L - BOW_R - 0.02, 20)
    tl = np.maximum(tl, line_mask(list(zip(us, s * (body_half(us) - 0.022), strict=True)), 0.028))
  tl = blur(tl, 0.004)
  glow = blur(tl, 0.06)
  rgb, alpha = over(rgb, alpha, (255, 30, 40), np.clip(glow * 1.6, 0, 1) * 0.55)
  rgb, alpha = over(rgb, alpha, (255, 45, 55), tl)
  rgb, alpha = over(rgb, alpha, (255, 170, 170), blur(line_mask(tl_pts, 0.008), 0.003) * 0.7)
  # headlamps: slim white units at the front corners + glow
  hl = np.zeros_like(body)
  vf = float(body_half(BOW_F))
  for s in (1, -1):
    vv = np.linspace(0.46, vf - 0.03, 40)
    pts = [(BOW_F * (x / vf) ** 2 + 0.035, s * x) for x in vv]
    us = np.linspace(BOW_F + 0.02, 0.24, 12)
    pts += [(u, s * (body_half(u) - 0.035)) for u in us]
    hl = np.maximum(hl, line_mask(pts, 0.026))
  hl = blur(hl, 0.004)
  rgb, alpha = over(rgb, alpha, (210, 230, 255), np.clip(blur(hl, 0.05) * 1.5, 0, 1) * 0.45)
  rgb, alpha = over(rgb, alpha, (240, 248, 255), hl)
  # grille: dark centre band in the nose with a thin chrome edge (no logo)
  vv = np.linspace(-0.36, 0.36, 40)
  gr = line_mask([(BOW_F * (x / vf) ** 2 + 0.03, x) for x in vv], 0.05)
  rgb, alpha = over(rgb, alpha, (8, 9, 11), gr)
  rgb, alpha = over(rgb, alpha, (190, 198, 212), line_mask([(BOW_F * (x / vf) ** 2 + 0.058, x) for x in vv], 0.008) * 0.8)

  # mirrors: gloss black cap + highlight
  for m in mirrors:
    rgb, alpha = over(rgb, alpha, (14, 15, 18), m)
    rgb, alpha = over(rgb, alpha, (170, 185, 210), np.clip(m - blur(m, 0.008), 0, 1) * 1.4)

  out = np.dstack([np.clip(rgb, 0, 255), np.clip(alpha * 255, 0, 255)]).astype(np.uint8)
  img = Image.fromarray(out, "RGBA")
  return img.resize((SIZE[0] // SS, SIZE[1] // SS), Image.LANCZOS)


if __name__ == "__main__":
  out = Path(sys.argv[1]) if len(sys.argv) > 1 else OUT
  render().save(out, optimize=True)
  print(out)
