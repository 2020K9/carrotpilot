#!/usr/bin/env python3
"""Generate the 2020 Kia K9 (RJ facelift, Platinum, black) parked-screen model for the cluster HUD.

Original low-poly design. Side / top / front / rear outlines were traced by hand from the owner's own photos of
the car (shape reference only) and the published dimensions: length 5120 mm, width 1915 mm, height 1490 mm,
wheelbase 3105 mm, 19-inch wheels. No photo pixels, texture or third-party mesh are part of the output.
The body is lofted from smooth cross-sections; grille, lamps, chrome, glass and wheels are separate colours.
Axes match the other cluster models: x = right, y = forward, z = up; normalised to x +-0.5, y +-0.5, z 0..1.
"""
import math
import sys

import numpy as np

L, W, H = 5.12, 1.915, 1.49
WB, R_AXLE = 3.105, 1.03          # rear axle 1.03 m from the rear bumper, front axle at 4.135 m
TIRE_R, TIRE_W = 0.355, 0.245
ARCH_R = 0.405
HW = W / 2

MAT = {
  "paint": "rgb_25282d",   # black gloss (lifted a little so the shading reads on the dark theme)
  "glass": "rgb_0b0d11",
  "trim": "rgb_101113",
  "well": "rgb_060607",
  "chrome": "rgb_dfe2e6",
  "chrome2": "rgb_9ea3a9",
  "grille": "rgb_15171a",
  "lamp": "rgb_e9eef6",
  "lamp_in": "rgb_5a6068",
  "tail": "rgb_b0141e",
  "tail2": "rgb_5e0a10",
  "plate": "rgb_c9ccd0",   # blank plate (no characters)
  "tire": "rgb_141414",
  "rim": "rgb_c2c6cb",
  "rim_dark": "rgb_2b2e33",
}


def _smooth_profile(points, n=1200, passes=6, k=15):
  u = np.linspace(0.0, L, n)
  y = np.interp(u, [p[0] for p in points], [p[1] for p in points])
  ker = np.ones(k) / k
  for _ in range(passes):
    y = np.convolve(np.pad(y, k // 2, mode="edge"), ker, mode="valid")
  return u, y


# side view (u = metres from the rear bumper): long hood, long low rear window, short trunk
TOP = [(0.0, 0.93), (0.04, 1.00), (0.12, 1.045), (0.45, 1.06), (0.88, 1.075), (1.02, 1.10), (1.30, 1.21),
       (1.60, 1.345), (1.90, 1.435), (2.30, 1.482), (2.70, 1.49), (3.00, 1.465), (3.20, 1.405), (3.50, 1.265),
       (3.80, 1.115), (3.95, 1.07), (4.40, 1.03), (4.80, 0.975), (5.00, 0.92), (5.08, 0.86), (5.12, 0.80)]
BELT = [(0.0, 0.91), (0.06, 0.985), (0.40, 1.0), (1.30, 1.005), (3.60, 0.995), (3.95, 0.995), (4.40, 0.985),
        (4.80, 0.935), (5.00, 0.88), (5.08, 0.83), (5.12, 0.775)]
BOTTOM = [(0.0, 0.36), (0.10, 0.28), (0.45, 0.20), (4.65, 0.20), (5.00, 0.24), (5.12, 0.29)]
_U, _TOP = _smooth_profile(TOP)
_, _BELT = _smooth_profile(BELT)
_, _BOT = _smooth_profile(BOTTOM)

# side window opening (u, z), chrome surround outside it, B pillar inside
DLO = [(1.30, 1.035), (1.52, 1.26), (1.85, 1.385), (2.30, 1.425), (2.95, 1.415), (3.22, 1.335), (3.64, 1.035)]


def prof(arr, u):
  return float(np.interp(u, _U, arr))


def sstep(a, b, x):
  t = min(1.0, max(0.0, (x - a) / (b - a)))
  return t * t * (3 - 2 * t)


def half_width(u):
  # plan view: elliptic corners, the front a bit squarer than the rear
  w = HW
  df, dr = L - u, u
  if df < 0.42:
    t = 1.0 - df / 0.42
    w -= 0.20 * (1.0 - math.sqrt(max(0.0, 1.0 - t * t)))
  if dr < 0.32:
    t = 1.0 - dr / 0.32
    w -= 0.17 * (1.0 - math.sqrt(max(0.0, 1.0 - t * t)))
  return w


def chaikin(pts, it=2):
  p = np.asarray(pts, float)
  for _ in range(it):
    q = [p[0]]
    for a, b in zip(p[:-1], p[1:], strict=True):
      q.append(0.75 * a + 0.25 * b)
      q.append(0.25 * a + 0.75 * b)
    q.append(p[-1])
    p = np.array(q)
  return p


def lerp(a, b, t):
  return a + (b - a) * t


def station(u):
  top, belt, bot = prof(_TOP, u), prof(_BELT, u), prof(_BOT, u)
  belt = min(belt, top - 0.02)
  w = half_width(u)
  c = sstep(0.06, 0.28, top - belt)
  wr = min(0.60, w - 0.12)
  g = top - belt
  zc = bot + 0.62 * (belt - bot)
  ctrl = [
    (0.0, bot), (0.68 * w, bot), (0.91 * w, bot + 0.035), (0.975 * w, bot + 0.15), (w, zc),
    (0.985 * w, belt - 0.10), (w - 0.045, belt - 0.02), (w - 0.085, belt + 0.01),
    (lerp(w - 0.13, w - 0.10, c), belt + lerp(0.30, 0.10, c) * g),
    (lerp(0.70 * w, wr + 0.045, c), belt + lerp(0.62, 0.62, c) * g),
    (lerp(0.45 * w, wr, c), top - lerp(0.12, 0.05, c) * g),
    (lerp(0.22 * w, 0.62 * wr, c), top - 0.008),
    (0.0, top),
  ]
  return chaikin(ctrl, 2), top, belt


def _point_in_poly(u, z, poly):
  inside = False
  for (u0, z0), (u1, z1) in zip(poly, poly[1:] + poly[:1], strict=True):
    if (z0 > z) != (z1 > z) and u < u0 + (z - z0) * (u1 - u0) / (z1 - z0):
      inside = not inside
  return inside


def _poly_dist(u, z, poly):
  best = 1e9
  for (u0, z0), (u1, z1) in zip(poly, poly[1:] + poly[:1], strict=True):
    du, dz = u1 - u0, z1 - z0
    t = max(0.0, min(1.0, ((u - u0) * du + (z - z0) * dz) / (du * du + dz * dz)))
    best = min(best, math.hypot(u - u0 - t * du, z - z0 - t * dz))
  return best


def body_material(u, x, z, nx, nz, top, belt):
  ax = abs(x)
  # wheel wells
  for ay in (R_AXLE, R_AXLE + WB):
    if math.hypot(u - ay, z - TIRE_R) < ARCH_R - 0.01 and nx > 0.2:
      return "well"
  if nz < -0.55:
    return "trim"
  side = nx > 0.35
  if side and z > belt - 0.02:
    inside = _point_in_poly(u, z, DLO)
    if inside:
      if abs(u - 2.47) < 0.035:
        return "trim"            # B pillar (gloss black)
      return "glass"
  if nz > 0.25 and z > belt + 0.05:
    if 3.10 < u < 3.90 and z > 1.075 and ax < 0.72:
      return "glass"             # windscreen
    if 1.02 < u < 1.88 and z > 1.10 and ax < 0.64:
      return "glass"             # long low rear window
  # headlamp wrap and tail lamp wrap on the corners
  if u > L - 0.17 and nx > 0.25 and 0.68 < z < 0.77 - 0.15 * (L - u):
    return "lamp"
  if u < 0.24 - 0.5 * max(0.0, z - 0.98) and nx > 0.25 and 0.787 < z < 1.025:
    return "tail2" if (z - 0.787) % 0.03 < 0.006 else "tail"
  # chrome lines: lower sill, rear bumper wrap, door handles
  if side and 0.335 < z < 0.36 and 1.15 < u < 4.05:
    return "chrome"
  if u < 0.30 and side and 0.40 < z < 0.418:
    return "chrome"
  if side and 0.89 < z < 0.915 and (abs(u - 2.05) < 0.08 or abs(u - 3.02) < 0.08):
    return "chrome"
  return "paint"


class Obj:
  def __init__(self):
    self.v, self.vn, self.faces = [], [], {}

  def add_grid(self, grid, normals, mats):
    base = len(self.v)
    n_i, n_j = grid.shape[:2]
    self.v.extend(map(tuple, grid.reshape(-1, 3)))
    self.vn.extend(map(tuple, normals.reshape(-1, 3)))
    for i in range(n_i - 1):
      for j in range(n_j - 1):
        m = mats[i][j]
        a, b, c, d = (base + i * n_j + j, base + (i + 1) * n_j + j, base + (i + 1) * n_j + j + 1, base + i * n_j + j + 1)
        self.faces.setdefault(m, []).append((a, b, c, d))

  def add_poly(self, pts, normal, mat):
    base = len(self.v)
    for p in pts:
      self.v.append(tuple(float(c) for c in p))
      self.vn.append(tuple(normal))
    self.faces.setdefault(mat, []).append(tuple(range(base, base + len(pts))))

  def front_poly(self, pts_xz, y, mat, ny=1):
    pts = [(x, y, z) for x, z in pts_xz]
    if ny < 0:
      pts = pts[::-1]
    self.add_poly(pts, (0, ny, 0), mat)

  def box(self, c, s, mat):
    cx, cy, cz = c
    half = tuple(x / 2 for x in s)
    for axis in range(3):
      for sign in (-1, 1):
        n = [0, 0, 0]
        n[axis] = sign
        u_ax, v_ax = [a for a in range(3) if a != axis]
        pts = []
        for du, dv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
          p = [cx, cy, cz]
          p[axis] += sign * half[axis]
          p[u_ax] += du * half[u_ax]
          p[v_ax] += dv * half[v_ax]
          pts.append(p)
        self.add_poly(pts, n, mat)

  def wheel(self, y, side):
    seg = 36
    x_out = side * (HW - 0.045)
    x_in = x_out - side * TIRE_W
    cz = TIRE_R

    def p(x, a, r):
      return (x, y + r * math.cos(a), cz + r * math.sin(a))
    for k in range(seg):
      a0, a1 = 2 * math.pi * k / seg, 2 * math.pi * (k + 1) / seg
      am = (a0 + a1) / 2
      self.add_poly([p(x_in, a0, TIRE_R), p(x_in, a1, TIRE_R), p(x_out, a1, TIRE_R), p(x_out, a0, TIRE_R)],
                    (0, math.cos(am), math.sin(am)), "tire")
      self.add_poly([p(x_out, a0, TIRE_R), p(x_out, a1, TIRE_R), p(x_out, a1, 0.245), p(x_out, a0, 0.245)],
                    (side, 0, 0), "tire")
      self.add_poly([p(x_out + side * 0.003, a0, 0.245), p(x_out + side * 0.003, a1, 0.245),
                     p(x_out + side * 0.003, a1, 0.222), p(x_out + side * 0.003, a0, 0.222)], (side, 0, 0), "rim")
      self.add_poly([p(x_out - side * 0.03, a0, 0.222), p(x_out - side * 0.03, a1, 0.222),
                     p(x_out - side * 0.03, a1, 0.0), p(x_out - side * 0.03, a0, 0.0)], (side, 0, 0), "rim_dark")
    # ten twin spokes, trapezoids widening towards the rim
    for k in range(10):
      for off in (-0.075, 0.075):
        a = 2 * math.pi * k / 10 + off
        w_in, w_out = 0.035, 0.07
        xs = x_out + side * 0.002
        pts = [p(xs - side * 0.012, a - w_in, 0.06), p(xs, a - w_out / 2, 0.224), p(xs, a + w_out / 2, 0.224),
               p(xs - side * 0.012, a + w_in, 0.06)]
        self.add_poly(pts, (side, 0, 0), "rim")
    hub = [p(x_out + side * 0.001, 2 * math.pi * k / 12, 0.065) for k in range(12)]
    self.add_poly(hub, (side, 0, 0), "rim")
    logo = [p(x_out + side * 0.003, 2 * math.pi * k / 12, 0.03) for k in range(12)]
    self.add_poly(logo, (side, 0, 0), "rim_dark")

  def write(self, fh, header):
    fh.write(header)
    for v in self.v:
      fh.write(f"v {v[0]:.5f} {v[1]:.5f} {v[2]:.5f}\n")
    for n in self.vn:
      fh.write(f"vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n")
    for m, faces in self.faces.items():
      fh.write(f"usemtl {MAT[m]}\n")
      for f in faces:
        fh.write("f " + " ".join(f"{i + 1}//{i + 1}" for i in f) + "\n")


def grid_normals(g):
  du = np.gradient(g, axis=0)
  dv = np.gradient(g, axis=1)
  n = np.cross(du, dv)
  n /= np.maximum(np.linalg.norm(n, axis=2, keepdims=True), 1e-9)
  return n


def recess_wells(g, us):
  # push the shell in around each wheel so the wheels sit in arches (right side, x >= 0)
  for i, u in enumerate(us):
    w_in = half_width(u) - 0.20
    for j in range(g.shape[1]):
      x, z = g[i, j, 0], g[i, j, 2]
      for ay in (R_AXLE, R_AXLE + WB):
        if math.hypot(u - ay, z - TIRE_R) < ARCH_R and x > w_in:
          g[i, j, 0] = w_in


def side_x(u, z):
  """x of the (unrecessed) right side shell at height z, greenhouse part of the station."""
  ring, top, belt = station(u)
  j0 = int(np.argmax(ring[:, 0]))
  part = ring[j0:]
  zs, xs = part[:, 1], part[:, 0]
  k = int(np.searchsorted(zs, z))
  k = min(max(k, 1), len(zs) - 1)
  t = (z - zs[k - 1]) / max(zs[k] - zs[k - 1], 1e-9)
  x = xs[k - 1] + (xs[k] - xs[k - 1]) * min(max(t, 0.0), 1.0)
  dx, dz = xs[k] - xs[k - 1], zs[k] - zs[k - 1]
  ln = math.hypot(dx, dz) or 1.0
  return x, (dz / ln, 0.0, -dx / ln)


def window_chrome(obj):
  # smooth chrome surround of the side window (sampled along the outline, 3 mm proud of the shell)
  poly = DLO + DLO[:1]
  pts = []
  for (u0, z0), (u1, z1) in zip(poly[:-1], poly[1:], strict=True):
    n = max(2, int(math.hypot(u1 - u0, z1 - z0) / 0.02))
    for k in range(n):
      t = k / n
      pts.append((u0 + (u1 - u0) * t, z0 + (z1 - z0) * t))
  pts.append(pts[0])
  cu = sum(p[0] for p in DLO) / len(DLO)
  cz = sum(p[1] for p in DLO) / len(DLO)
  half = 0.016
  for side in (1, -1):
    for (ua, za), (ub, zb) in zip(pts[:-1], pts[1:], strict=True):
      quad = []
      for (u, z), sgn in (((ua, za), -1), ((ub, zb), -1), ((ub, zb), 1), ((ua, za), 1)):
        du, dz = u - cu, z - cz
        ln = math.hypot(du, dz * 3.0) or 1.0
        uu, zz = u + sgn * half * du / ln, z + sgn * half * dz * 3.0 / ln
        zz = max(zz, prof(_BELT, uu) - 0.01)
        x, nrm = side_x(uu, zz)
        quad.append((side * (x + 0.004), uu, zz))
      if side < 0:
        quad = quad[::-1]
      obj.add_poly(quad, (side * nrm[0], 0.0, nrm[2]), "chrome")


def front_details(obj, ring_end):
  yf = L + 0.004
  # wide trapezoid grille (wider at the bottom), chrome frame, chrome diamond pattern
  gt, gb, z0, z1 = 0.45, 0.50, 0.42, 0.745

  def chamfer(t, b, za, zb, c):
    return [(-b + c, za), (b - c, za), (b, za + c), (t, zb - c), (t - c, zb), (-t + c, zb), (-t, zb - c), (-b, za + c)]
  obj.front_poly(chamfer(gt + 0.028, gb + 0.028, z0 - 0.025, z1 + 0.022, 0.06), yf, "chrome")
  obj.front_poly(chamfer(gt, gb, z0, z1, 0.045), yf + 0.003, "grille")
  rows, dz = 7, (z1 - z0) / 7
  for r in range(rows):
    zc = z0 + (r + 0.5) * dz
    half = gb + (gt - gb) * (zc - z0) / (z1 - z0)
    step = 0.068
    offs = 0.0 if r % 2 == 0 else step / 2
    n = int(half / step) + 1
    for k in range(-n, n + 1):
      xc = k * step + offs
      if abs(xc) + 0.026 > half - (0.03 if r in (0, rows - 1) else 0.0):
        continue
      obj.front_poly([(xc, zc - dz * 0.42), (xc + 0.026, zc), (xc, zc + dz * 0.42), (xc - 0.026, zc)], yf + 0.006,
                     "chrome2" if r % 2 else "chrome")
  # slim headlamps either side of the grille, rising slightly outwards, with an inner dark band
  for s in (1, -1):
    pts = [(0.52, 0.675), (0.80, 0.705), (0.80, 0.77), (0.51, 0.75)]
    obj.front_poly([(s * x, z) for x, z in (pts if s > 0 else pts[::-1])], yf, "lamp")
    band = [(0.56, 0.69), (0.79, 0.715), (0.79, 0.735), (0.56, 0.715)]
    obj.front_poly([(s * x, z) for x, z in (band if s > 0 else band[::-1])], yf + 0.002, "lamp_in")
  # lower bumper: blank plate, dark intake, chrome lip across the bottom
  obj.front_poly([(-0.26, 0.30), (0.26, 0.30), (0.26, 0.405), (-0.26, 0.405)], yf, "plate")
  obj.front_poly([(-0.72, 0.29), (-0.30, 0.29), (-0.30, 0.37), (-0.68, 0.37)], yf, "grille")
  obj.front_poly([(0.30, 0.29), (0.72, 0.29), (0.68, 0.37), (0.30, 0.37)], yf, "grille")
  obj.front_poly([(-0.74, 0.262), (0.74, 0.262), (0.74, 0.282), (-0.74, 0.282)], yf + 0.002, "chrome")
  obj.front_poly([(-0.06, z1 + 0.035), (0.06, z1 + 0.035), (0.06, z1 + 0.06), (-0.06, z1 + 0.06)], yf, "chrome")


def rear_details(obj):
  yr = -0.004
  # tall tail lamps at both ends with horizontal inner lines, chrome bar across the trunk between them
  for s in (1, -1):
    x0, x1 = 0.54, 0.79
    rim = [(x0 - 0.012, 0.80), (x0 + 0.02, 0.775), (x1, 0.775), (x1, 1.035), (x0 + 0.05, 1.035), (x0 - 0.012, 0.985)]
    obj.front_poly([(s * x, z) for x, z in (rim if s > 0 else rim[::-1])], yr + 0.001, "chrome2", ny=-1)
    red = [(x0, 0.805), (x0 + 0.025, 0.787), (x1, 0.787), (x1, 1.025), (x0 + 0.05, 1.025), (x0, 0.98)]
    obj.front_poly([(s * x, z) for x, z in (red if s > 0 else red[::-1])], yr - 0.001, "tail", ny=-1)
    for k in range(1, 8):
      za = 0.787 + k * 0.03
      pts = [(x0 + 0.03, za), (x1, za), (x1, za + 0.006), (x0 + 0.03, za + 0.006)]
      if s < 0:
        pts = [(-x, z) for x, z in pts[::-1]]
      obj.front_poly(pts, yr - 0.003, "tail2", ny=-1)
  obj.front_poly([(-0.50, 0.905), (0.50, 0.905), (0.50, 0.925), (-0.50, 0.925)], yr - 0.002, "chrome", ny=-1)
  obj.front_poly([(-0.26, 0.69), (0.26, 0.69), (0.26, 0.80), (-0.26, 0.80)], yr, "plate", ny=-1)
  obj.front_poly([(-0.06, 0.95), (0.06, 0.95), (0.06, 0.975), (-0.06, 0.975)], yr, "chrome", ny=-1)
  # lower chrome line, dark diffuser, square exhaust tips on both sides
  obj.front_poly([(-0.80, 0.40), (0.80, 0.40), (0.80, 0.418), (-0.80, 0.418)], yr, "chrome", ny=-1)
  obj.front_poly([(-0.80, 0.30), (0.80, 0.30), (0.80, 0.39), (-0.80, 0.39)], yr, "trim", ny=-1)
  for s in (1, -1):
    obj.front_poly([(s * 0.47, 0.31), (s * 0.75, 0.31), (s * 0.73, 0.375), (s * 0.49, 0.375)][::s], yr - 0.003,
                   "chrome", ny=-1)
    obj.front_poly([(s * 0.49, 0.322), (s * 0.73, 0.322), (s * 0.715, 0.362), (s * 0.505, 0.362)][::s], yr - 0.005,
                   "trim", ny=-1)


def build():
  obj = Obj()
  us = np.concatenate([np.linspace(0, 0.30, 10)[:-1], np.linspace(0.30, 4.80, 200)[:-1], np.linspace(4.80, L, 11)])
  rings, tops, belts = [], [], []
  for u in us:
    ring, top, belt = station(u)
    rings.append(ring)
    tops.append(top)
    belts.append(belt)
  g = np.array([[(x, u, z) for x, z in ring] for u, ring in zip(us, rings, strict=True)])
  orig = g.copy()
  recess_wells(g, us)
  mats_r = []
  for i, u in enumerate(us):
    row = []
    for j in range(g.shape[1] - 1):
      a, b = orig[i, j], orig[i, j + 1]
      dx, dz = b[0] - a[0], b[2] - a[2]
      ln = math.hypot(dx, dz) or 1.0
      nx, nz = dz / ln, -dx / ln
      row.append(body_material(u, (a[0] + b[0]) / 2, (a[2] + b[2]) / 2, nx, nz, tops[i], belts[i]))
    mats_r.append(row)
  for side in (1, -1):
    gs = g.copy()
    gs[..., 0] *= side
    n = grid_normals(gs)
    out = gs.copy()
    out[..., 1] = 0.0
    out[..., 2] -= 0.6
    n = np.where(np.sum(n * out, axis=2, keepdims=True) < 0, -n, n)
    mats = mats_r
    if side < 0:
      gs, n = gs[:, ::-1], n[:, ::-1]
      mats = [list(reversed(m)) for m in mats_r]
    obj.add_grid(gs, n, mats)

  # front and rear faces
  for i, ny in ((-1, 1), (0, -1)):
    ring = rings[i]
    pts = [(x, us[i], z) for x, z in ring] + [(-x, us[i], z) for x, z in reversed(ring[:-1])]
    if ny < 0:
      pts = pts[::-1]
    obj.add_poly(pts, (0, ny, 0), "paint")
  window_chrome(obj)
  front_details(obj, rings[-1])
  rear_details(obj)

  for side in (1, -1):
    # mirrors on the doors, chrome-free body colour caps
    y = 3.55
    x0 = side * (half_width(y) - 0.10)
    obj.box((x0 + side * 0.07, y, 1.03), (0.12, 0.05, 0.04), "paint")
    obj.box((x0 + side * 0.18, y - 0.02, 1.07), (0.16, 0.10, 0.12), "paint")
    obj.box((x0 + side * 0.18, y - 0.072, 1.07), (0.14, 0.006, 0.10), "glass")
    for y in (R_AXLE, R_AXLE + WB):
      obj.wheel(y, side)

  v = np.array(obj.v)
  v[:, 0] /= W
  v[:, 1] = v[:, 1] / L - 0.5
  v[:, 2] /= H
  obj.v = [tuple(p) for p in v]
  return obj


if __name__ == "__main__":
  out = sys.argv[1]
  obj = build()
  header = ("# Kia K9 (2020 RJ facelift) parked-screen model for the CarrotPilot cluster HUD.\n"
            + "# Original low-poly design generated by make_k9_park_model.py: outlines traced by hand from the owner's\n"
            + "# photos (shape reference only) and published dimensions (5120 x 1915 x 1490 mm, wheelbase 3105 mm);\n"
            + "# no third-party mesh or photo pixels are included, number plates are blank.\n"
            + "# Axes: x=right, y=forward, z=up; normalised to x +-0.5, y +-0.5, z 0..1 (renderer scales to real size).\n")
  with open(out, "w") as fh:
    obj.write(fh, header)
  print(out, len(obj.v), sum(len(f) for f in obj.faces.values()))
