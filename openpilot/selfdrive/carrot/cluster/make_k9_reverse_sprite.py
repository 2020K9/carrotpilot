#!/usr/bin/env python3
"""Top view of the 2020 K9 (RJ) for the K9 reverse (R) screen, drawn in code (no photo, no 3D model, no plate, no logo).

Surround-view style illustration: smooth tapered nose and tail with a slight waist, a long hood with soft creases,
one smooth painted roof (black metal gradient) between the dark windshield and rear glass, side glass only as thin
slivers under a fine chrome window line, mirrors, four tyres at the 3105 mm wheelbase / ~1630 mm track that peek a
few px past the body, a thin full-width tail lamp line wrapping round both corners, thin white headlamps at the
front corners and a small chrome grille line at the nose. Soft shadow, no hard light rim.
Plan size 5120 x 1915 mm. Drawn at SS x the output resolution, then reduced (smooth edges).
Car front to the left, car right side up (same as the R screen camera).
Outputs (same frame: PAD metres of margin around the body at PPM px/m):
  assets/images/k9_reverse_car.png         body + rear tyres (front tyres are drawn by the renderer, steered)
  assets/images/k9_reverse_car_shadow.png  soft floor shadow, drawn under the front tyres and the body
  assets/images/k9_reverse_wheel.png       one front tyre seen from above (WHEEL_D x FRONT_TYRE_W plus WHEEL_PAD)
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
TRACK_F, TRACK_R = 1.630, 1.645
WHEEL_D = 0.70  # 245/45 R19 ~ 0.70 m
FRONT_TYRE_W, REAR_TYRE_W = 0.245, 0.275
TYRE_PEEK = 0.026  # outer sidewall shows this far past the body edge (~3 px on the R screen)
WHEEL_PAD = 0.03
PPM = 200
PAD = 0.30
SS = 4
S = PPM * SS
HERE = Path(__file__).resolve().parent
IMAGES = HERE / "assets" / "images"
OUT = IMAGES / "k9_reverse_car.png"
OUT_SHADOW = IMAGES / "k9_reverse_car_shadow.png"
OUT_WHEEL = IMAGES / "k9_reverse_wheel.png"
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


def front_edge(v):
  vf = float(body_half(BOW_F))
  return BOW_F * (np.asarray(v) / vf) ** 2


def rear_edge(v):
  vr = float(body_half(L - BOW_R))
  return L - BOW_R * (np.asarray(v) / vr) ** 2


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


def to_image(rgb, alpha, size):
  out = np.dstack([np.clip(rgb, 0, 255), np.clip(alpha * 255, 0, 255)]).astype(np.uint8)
  return Image.fromarray(out, "RGBA").resize(size, Image.LANCZOS)


# ---- greenhouse ---------------------------------------------------------------------------------------------
G_U = np.array([1.72, 1.95, 2.40, 3.00, 3.45, 3.80, 4.13])
G_V = np.array([0.735, 0.785, 0.805, 0.805, 0.780, 0.730, 0.650])
G_FRONT_BOW, G_REAR_BOW = 0.13, 0.09
ROOF_F, ROOF_R = 2.36, 3.52  # windshield top / rear glass top
SIDE_GLASS = 0.055  # side glass seen from above: only a thin sliver outside the roof


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


def tyre_mask(u_c, v_c, d, w, r=0.05):
  """rounded tyre footprint (top view) centred at (u_c, v_c)."""
  img = Image.new("L", SIZE, 0)
  x0, y0 = px(u_c - d / 2, v_c + w / 2)
  x1, y1 = px(u_c + d / 2, v_c - w / 2)
  ImageDraw.Draw(img).rounded_rectangle((float(x0), float(y0), float(x1), float(y1)), radius=r * S, fill=255)
  return np.asarray(img, np.float32) / 255.0


def tyre_paint(U, V, u_c, v_c, d, w):
  """rubber seen from above: grey tread crown with fine cross grooves, darker shoulders (reads as a tyre on the dark floor)."""
  lu = (U - u_c) / (d / 2)
  lv = (np.abs(V - v_c)) / (w / 2)
  crown = np.clip(1 - lu ** 2, 0, 1) ** 0.5
  grooves = 0.85 + 0.15 * np.cos((U - u_c) * 2 * np.pi / 0.045)
  lit = crown * (1 - 0.55 * lv ** 3) * grooves
  return np.array([20, 21, 25], np.float32) + np.array([46, 48, 54], np.float32) * lit[..., None]


def tyre_geometry(axle_u, track, tyre_w, s):
  """tyre centre v and drawn width so its outer sidewall peeks TYRE_PEEK past the body (centre on the track)."""
  v_c = s * track / 2
  outer = float(body_half(axle_u)) + TYRE_PEEK
  w = max(tyre_w, 2 * (outer - track / 2))
  return v_c, w


def render():
  H_, W_ = SIZE[1], SIZE[0]
  yy, xx = np.mgrid[0:H_, 0:W_].astype(np.float32)
  U = xx / S - PAD
  V = WH + PAD - yy / S
  outsize = (SIZE[0] // SS, SIZE[1] // SS)

  body = mask_of(body_outline())

  # mirrors: teardrop on each side just behind the A-pillar base (body coloured caps)
  mirrors = []
  for s in (1, -1):
    base = float(body_half(2.02))
    pts = [(1.86, s * (base - 0.03)), (1.93, s * (base + 0.06)), (2.02, s * (base + 0.115)), (2.10, s * (base + 0.135)),
           (2.15, s * (base + 0.12)), (2.16, s * (base + 0.07)), (2.12, s * (base + 0.0)), (2.10, s * (base - 0.03))]
    arr = np.array(pts)
    t = np.linspace(0, 1, len(arr))
    xs = np.linspace(0, 1, 60)
    sm = np.stack([np.interp(xs, t, arr[:, 0]), np.interp(xs, t, arr[:, 1])], 1)
    mirrors.append(blur(mask_of([tuple(p) for p in sm]), 0.006))
  mirror = np.clip(mirrors[0] + mirrors[1], 0, 1)
  car = np.clip(body + mirror, 0, 1)

  # ---- shadow layer (separate file: drawn under the steered front tyres) ----
  tyres_all = np.zeros_like(body)
  for axle, track, tw in ((FRONT_AXLE_U, TRACK_F, FRONT_TYRE_W), (REAR_AXLE_U, TRACK_R, REAR_TYRE_W)):
    for s in (1, -1):
      v_c, w = tyre_geometry(axle, track, tw, s)
      tyres_all = np.maximum(tyres_all, tyre_mask(axle, v_c, WHEEL_D, w))
  footprint = np.clip(car + tyres_all, 0, 1)
  sh_rgb = np.zeros((H_, W_, 3), np.float32)
  sh_a = np.zeros((H_, W_), np.float32)
  contact = blur(footprint, 0.035) * 0.85  # tight contact shadow
  sh_rgb, sh_a = over(sh_rgb, sh_a, (2, 4, 9), contact)
  ambient = blur(footprint, 0.13) * 0.55  # wide soft ambient occlusion
  sh_rgb, sh_a = over(sh_rgb, sh_a, (2, 4, 9), ambient * (1 - contact))
  shadow_img = to_image(sh_rgb, sh_a, outsize)

  # ---- body layer ----
  rgb = np.zeros((H_, W_, 3), np.float32)
  alpha = np.zeros((H_, W_), np.float32)
  for s in (1, -1):  # rear tyres under the body (front tyres: renderer, steered)
    v_c, w = tyre_geometry(REAR_AXLE_U, TRACK_R, REAR_TYRE_W, s)
    m = tyre_mask(REAR_AXLE_U, v_c, WHEEL_D, w)
    rgb, alpha = over(rgb, alpha, tyre_paint(U, V, REAR_AXLE_U, v_c, WHEEL_D, w), m)

  # paint: dark metal, lit from above - a broad soft sheen on the centre, darker towards the curved edges
  edge = blur(car, 0.20)
  shade = 0.42 + 0.58 * np.clip((edge - 0.5) * 2.0, 0, 1) ** 0.8
  base = np.array([34, 37, 45], np.float32)
  spec_col = np.array([120, 130, 150], np.float32)
  bw_u = body_half(np.clip(U, 0, L))
  vn = V / np.maximum(bw_u, 0.3)
  hood = smooth(0.10, 0.45, U) * (1 - smooth(1.45, 1.72, U))
  trunk = smooth(4.18, 4.40, U) * (1 - smooth(4.85, 5.05, U))
  sheen = np.exp(-(vn / 0.55) ** 2) * (0.30 * hood + 0.24 * trunk)
  sheen += 0.12 * np.exp(-((U - 0.80) / 0.40) ** 2 - ((V - 0.30) / 0.35) ** 2) * hood  # soft key light patch
  axles = np.exp(-((U - FRONT_AXLE_U) / 0.50) ** 2) + np.exp(-((U - REAR_AXLE_U) / 0.50) ** 2)
  fender = np.exp(-((np.abs(vn) - 0.80) / 0.08) ** 2) * axles * 0.22  # fender shoulders catch a little light
  spec = sheen + fender
  paint = base * shade[..., None] + spec_col * spec[..., None] * shade[..., None]
  rgb, alpha = over(rgb, alpha, paint, car)

  # wheel arches: darker tyre outline under the fender lip at all four wheels
  for axle, track, tw in ((FRONT_AXLE_U, TRACK_F, FRONT_TYRE_W), (REAR_AXLE_U, TRACK_R, REAR_TYRE_W)):
    for s in (1, -1):
      v_c, w = tyre_geometry(axle, track, tw, s)
      t = tyre_mask(axle, v_c, WHEEL_D * 0.98, w)
      lip = np.clip((np.abs(V) - (float(body_half(axle)) - 0.10)) / 0.10, 0, 1)  # only near the body edge
      rgb, alpha = over(rgb, alpha, (6, 7, 9), blur(t, 0.012) * lip * car * 0.55)

  def paint_lines(points, width_m, color, a, soft=0.0):
    nonlocal rgb, alpha
    m = line_mask(points, width_m)
    if soft:
      m = blur(m, soft)
    rgb, alpha = over(rgb, alpha, color, m * a * car)

  # waist / shoulder line along both sides (subtle highlight over a soft dark line)
  for s in (1, -1):
    us = np.linspace(0.55, 4.75, 120)
    vs = s * (body_half(us) * 0.885)
    paint_lines(list(zip(us, vs, strict=True)), 0.010, (150, 160, 180), 0.22, 0.008)
    paint_lines(list(zip(us, vs - s * 0.025, strict=True)), 0.016, (8, 9, 12), 0.22, 0.010)
  # hood: soft centre crease and two gentle character lines
  us = np.linspace(0.38, 1.60, 60)
  paint_lines(list(zip(us, np.full_like(us, 0.012), strict=True)), 0.010, (150, 160, 182), 0.20, 0.008)
  paint_lines(list(zip(us, np.full_like(us, -0.014), strict=True)), 0.012, (6, 7, 9), 0.20, 0.008)
  for s in (1, -1):
    us = np.linspace(0.30, 1.64, 60)
    vs = s * (0.48 + 0.17 * smooth(0.25, 1.7, us))
    paint_lines(list(zip(us, vs, strict=True)), 0.010, (140, 150, 172), 0.18, 0.010)
    paint_lines(list(zip(us, vs - s * 0.02, strict=True)), 0.012, (6, 7, 9), 0.16, 0.010)
  # panel gaps: hood front, trunk lid (fine, dark, low contrast)
  vv = np.linspace(-0.80, 0.80, 80)
  paint_lines([(front_edge(x) + 0.20, x) for x in vv], 0.005, (6, 7, 9), 0.45, 0.002)
  vv = np.linspace(-0.66, 0.66, 80)
  paint_lines([(4.13 + G_REAR_BOW * (1 - (x / 0.65) ** 2) + 0.03, x) for x in vv], 0.016, (4, 5, 7), 0.45, 0.010)
  vv = np.linspace(-0.80, 0.80, 80)
  paint_lines([(rear_edge(x) - 0.13, x) for x in vv], 0.005, (6, 7, 9), 0.40, 0.002)

  # ---- greenhouse: dark windshield and rear glass, one smooth painted roof, thin side glass slivers ----
  g_mask = mask_of(band(1.72, 4.13, g_half, G_FRONT_BOW, G_REAR_BOW))
  glass_dark = np.array([6, 8, 12], np.float32)
  sky = np.array([70, 84, 104], np.float32)
  vv = np.linspace(-0.74, 0.74, 80)  # cowl shadow in front of the windshield
  paint_lines([(1.72 - G_FRONT_BOW * (1 - (x / 0.74) ** 2) - 0.025, x) for x in vv], 0.03, (3, 4, 6), 0.55, 0.014)
  # whole cabin top in paint first (roof + pillars), then the glass areas cut into it
  rv = V / np.maximum(g_half(np.clip(U, 1.72, 4.13)), 0.3)
  ru = (U - ROOF_F) / (ROOF_R - ROOF_F)
  roof_shade = 0.58 + 0.42 * np.clip(1 - rv ** 2, 0, 1) ** 0.7
  roof_spec = 0.10 + 0.20 * np.exp(-(rv / 0.55) ** 2) * (0.6 + 0.4 * np.exp(-((ru - 0.40) / 0.45) ** 2))
  roof_col = base * 1.05 * roof_shade[..., None] + spec_col * roof_spec[..., None]
  rgb, alpha = over(rgb, alpha, roof_col, blur(g_mask, 0.004))

  def inset_half(d):
    return lambda u: np.maximum(g_half(u) - d, 0.05)

  # side glass: thin dark sliver along each side of the cabin (tumblehome seen from above)
  side = mask_of(band(1.80, 4.05, g_half, 0.0, 0.0)) * (1 - mask_of(band(1.70, 4.15, inset_half(SIDE_GLASS), 0.0, 0.0)))
  side = blur(side, 0.003)
  rgb, alpha = over(rgb, alpha, glass_dark + sky * 0.16, side * smooth(1.80, 1.95, U) * (1 - smooth(3.92, 4.05, U)))
  # windshield: from the cowl to the roof front, between thin A pillars
  ws = mask_of(band(1.72, ROOF_F, inset_half(0.040), G_FRONT_BOW, 0.05))
  t = smooth(1.60, ROOF_F, U)
  ws_col = glass_dark + sky * (0.10 + 0.30 * t ** 1.6)[..., None]
  ws_col = ws_col + np.array([80, 95, 115]) * (np.exp(-(((U - 1.98) - V * 0.32) / 0.07) ** 2) * 0.20)[..., None]
  rgb, alpha = over(rgb, alpha, ws_col, blur(ws, 0.004))
  # rear glass: from the roof back edge to the trunk, between C pillars
  rgm = mask_of(band(ROOF_R, 4.13, inset_half(0.060), 0.04, G_REAR_BOW))
  t2 = 1 - smooth(ROOF_R, 4.12, U)
  rg_col = glass_dark + sky * (0.08 + 0.24 * t2 ** 2)[..., None]
  rg_col = rg_col + np.array([80, 95, 115]) * (np.exp(-(((U - 3.86) + V * 0.28) / 0.06) ** 2) * 0.12)[..., None]
  rgb, alpha = over(rgb, alpha, rg_col, blur(rgm, 0.004))
  # chrome window line (DLO): thin, along the outer edge of the side glass
  for s in (1, -1):
    us = np.linspace(1.92, 4.00, 120)
    paint_lines(list(zip(us, s * (g_half(us) + 0.002), strict=True)), 0.007, (200, 208, 222), 0.80, 0.002)

  # ---- lamps ----
  # tail lamp: one thin light line across the rear, running smoothly round both corners and along the sides
  vr = float(body_half(L - BOW_R))
  # one continuous chain: left wrap -> rear -> right wrap
  left = [(u, -(body_half(u) - 0.020)) for u in np.linspace(L - 0.62, L - BOW_R - 0.02, 40)]
  rear = [(rear_edge(x) - 0.030, x) for x in np.linspace(-vr + 0.06, vr - 0.06, 160)]
  right = [(u, body_half(u) - 0.020) for u in np.linspace(L - BOW_R - 0.02, L - 0.62, 40)]
  tl_chain = left + rear + right
  tl = blur(line_mask(tl_chain, 0.014), 0.003)
  taper = smooth(L - 0.62, L - 0.40, U)  # wraps fade out along the sides
  tl = tl * taper
  rgb, alpha = over(rgb, alpha, (255, 30, 40), np.clip(blur(tl, 0.035) * 1.5, 0, 1) * 0.45 * car)
  rgb, alpha = over(rgb, alpha, (255, 55, 62), tl * car)
  rgb, alpha = over(rgb, alpha, (255, 175, 175), blur(line_mask(tl_chain, 0.004), 0.002) * taper * 0.55 * car)
  # headlamps: thin white light at both front corners, wrapping a little along the sides
  vf = float(body_half(BOW_F))
  hl = np.zeros_like(body)
  for s in (1, -1):
    pts = [(front_edge(x) + 0.028, s * x) for x in np.linspace(0.50, vf - 0.03, 40)]
    pts += [(u, s * (body_half(u) - 0.024)) for u in np.linspace(BOW_F + 0.02, 0.42, 20)]
    hl = np.maximum(hl, line_mask(pts, 0.013))
  hl = blur(hl, 0.003) * (1 - smooth(0.30, 0.44, U))
  rgb, alpha = over(rgb, alpha, (200, 225, 255), np.clip(blur(hl, 0.03) * 1.4, 0, 1) * 0.40 * car)
  rgb, alpha = over(rgb, alpha, (245, 250, 255), hl * car)
  # grille: small chrome line at the centre of the nose (dark sliver behind it), no logo
  vv = np.linspace(-0.24, 0.24, 40)
  rgb, alpha = over(rgb, alpha, (8, 9, 12), blur(line_mask([(front_edge(x) + 0.035, x) for x in vv], 0.030), 0.004) * car * 0.9)
  rgb, alpha = over(rgb, alpha, (205, 212, 226), line_mask([(front_edge(x) + 0.052, x) for x in vv], 0.005) * car * 0.65)
  rgb, alpha = over(rgb, alpha, (170, 178, 196), line_mask([(front_edge(x) + 0.022, x) for x in np.linspace(-0.26, 0.26, 40)], 0.004) * car * 0.55)

  # mirrors: body coloured caps with a soft top highlight
  for m in mirrors:
    rgb, alpha = over(rgb, alpha, base * 0.75, m)
    rgb, alpha = over(rgb, alpha, (90, 98, 116), np.clip(blur(m, 0.02) - 0.55, 0, 1) * 1.2 * m * 0.35)

  # very soft edge darkening so the outline melts into the shadow (no bright rim)
  rim = np.clip(car - blur(car, 0.015), 0, 1)
  rgb, alpha = over(rgb, alpha, (10, 11, 14), rim * 0.35)

  body_img = to_image(rgb, alpha, outsize)
  return body_img, shadow_img


def render_wheel():
  """one front tyre from above, rolling direction along x, WHEEL_PAD margin."""
  wpx = int(round((WHEEL_D + 2 * WHEEL_PAD) * S))
  v_c, w = tyre_geometry(FRONT_AXLE_U, TRACK_F, FRONT_TYRE_W, 1)
  hpx = int(round((w + 2 * WHEEL_PAD) * S))
  yy, xx = np.mgrid[0:hpx, 0:wpx].astype(np.float32)
  U = xx / S - WHEEL_PAD - WHEEL_D / 2
  V = yy / S - WHEEL_PAD - w / 2
  img = Image.new("L", (wpx, hpx), 0)
  ImageDraw.Draw(img).rounded_rectangle((WHEEL_PAD * S, WHEEL_PAD * S, wpx - WHEEL_PAD * S, hpx - WHEEL_PAD * S),
                                        radius=0.05 * S, fill=255)
  m = np.asarray(img, np.float32) / 255.0
  col = tyre_paint(U, V, 0.0, 0.0, WHEEL_D, w)
  rgb = np.zeros((hpx, wpx, 3), np.float32)
  a = np.zeros((hpx, wpx), np.float32)
  rgb, a = over(rgb, a, col, m)
  return to_image(rgb, a, (wpx // SS, hpx // SS)), w


if __name__ == "__main__":
  folder = Path(sys.argv[1]) if len(sys.argv) > 1 else IMAGES
  body_img, shadow_img = render()
  body_img.save(folder / OUT.name, optimize=True)
  shadow_img.save(folder / OUT_SHADOW.name, optimize=True)
  wheel_img, _ = render_wheel()
  wheel_img.save(folder / OUT_WHEEL.name, optimize=True)
  print(folder)
