#!/usr/bin/env python3
"""Generate the 2020 Kia K9 (RJ facelift, Platinum, black) parked-screen model for the cluster HUD (v4).

Original design built only from code. Side / top / front / rear outlines were traced by hand from the owner's own
photos of the car (shape reference only) and the published dimensions: length 5120 mm, width 1915 mm,
height 1490 mm, wheelbase 3105 mm, 19-inch wheels. No photo pixels, texture or third-party mesh are part of the output.
The body is lofted from creased cross-sections (rocker tuck-in, lower door crease, slightly concave door, sharp
shoulder line, tumblehome); glass, lamps, grille, chrome and trim are separate surface-following panels.
Axes match the other cluster models: x = right, y = forward, z = up; normalised to x +-0.5, y +-0.5, z 0..1.
"""
import functools
import math
import sys

import numpy as np

L, W, H = 5.12, 1.915, 1.49
WB, R_AXLE = 3.105, 1.03          # rear axle 1.03 m from the rear bumper, front axle at 4.135 m
TIRE_R, TIRE_W = 0.355, 0.245
RIM_R = 0.2415                    # 19 inch
ARCH_R = 0.405
HW = W / 2
AXLES = (R_AXLE, R_AXLE + WB)

MAT = {
  "paint": "rgb_1c1f24",   # black gloss (the renderer adds the clear-coat reflection for the K9 only)
  "glass": "rgb_0a0c10",
  "trim": "rgb_0e0f11",
  "gap": "rgb_050506",
  "well": "rgb_060607",
  "chrome": "rgb_dfe2e6",
  "chrome2": "rgb_9ea3a9",
  "grille": "rgb_111215",
  "lamp": "rgb_e9eef6",
  "lamp_in": "rgb_363c45",
  "amber": "rgb_e89a2c",
  "tail": "rgb_ff3a40",
  "tail2": "rgb_6e0c16",
  "plate": "rgb_c9ccd0",   # blank plate (no characters)
  "tire": "rgb_161616",
  "tire2": "rgb_202020",
  "rim": "rgb_c2c6cb",
  "rim_dark": "rgb_2b2e33",
  "disc": "rgb_45484d",
}


def _smooth_profile(points, n=1200, passes=6, k=15):
  u = np.linspace(0.0, L, n)
  y = np.interp(u, [p[0] for p in points], [p[1] for p in points])
  ker = np.ones(k) / k
  for _ in range(passes):
    y = np.convolve(np.pad(y, k // 2, mode="edge"), ker, mode="valid")
  return u, y


# side view (u = metres from the rear bumper): long hood, long low rear window, short high trunk
TOP = [(0.0, 1.00), (0.03, 1.035), (0.10, 1.058), (0.45, 1.07), (0.88, 1.082), (1.02, 1.10), (1.30, 1.21),
       (1.60, 1.345), (1.90, 1.435), (2.30, 1.482), (2.70, 1.49), (3.00, 1.465), (3.20, 1.405), (3.50, 1.265),
       (3.80, 1.115), (3.95, 1.07), (4.40, 1.035), (4.80, 0.985), (5.00, 0.945), (5.08, 0.905), (5.12, 0.86)]
BELT = [(0.0, 0.975), (0.06, 1.0), (0.40, 1.012), (1.30, 1.012), (3.60, 0.998), (3.95, 0.996), (4.40, 0.985),
        (4.80, 0.935), (5.00, 0.895), (5.08, 0.865), (5.12, 0.835)]
BOTTOM = [(0.0, 0.27), (0.10, 0.25), (0.45, 0.20), (4.65, 0.20), (5.00, 0.24), (5.12, 0.27)]
_U, _TOP = _smooth_profile(TOP)
_, _BELT = _smooth_profile(BELT)
_, _BOT = _smooth_profile(BOTTOM)

# side window opening (u, z): bottom edge = belt line, top edge = smooth arc
DLO_U0, DLO_U1 = 1.30, 3.64
DLO_TOP = [(1.30, 1.04), (1.52, 1.26), (1.85, 1.385), (2.30, 1.425), (2.95, 1.415), (3.22, 1.335), (3.64, 1.04)]
B_PILLAR = (2.43, 2.51)


def prof(arr, u):
  return float(np.interp(u, _U, arr))


def sstep(a, b, x):
  t = min(1.0, max(0.0, (x - a) / (b - a)))
  return t * t * (3 - 2 * t)


def lerp(a, b, t):
  return a + (b - a) * t


def half_width(u):
  # plan view: rounded corners front and rear (front squarer, rear fuller)
  w = HW
  df, dr = L - u, u
  if df < 0.50:
    t = 1.0 - df / 0.50
    w -= 0.24 * (1.0 - math.sqrt(max(0.0, 1.0 - t * t)))
  if dr < 0.40:
    t = 1.0 - dr / 0.40
    w -= 0.19 * (1.0 - math.sqrt(max(0.0, 1.0 - t * t)))
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


@functools.lru_cache(maxsize=8192)
def _station(uk):
  u = uk / 10000.0
  top, belt, bot = prof(_TOP, u), prof(_BELT, u), prof(_BOT, u)
  belt = min(belt, top - 0.02)
  w = half_width(u)
  g = top - belt
  c = sstep(0.06, 0.28, g)
  wr = min(0.60, w - 0.12)
  sh = belt - 0.075                 # sharp shoulder line
  cr = bot + 0.17                   # lower door crease
  cr = min(cr, sh - 0.12)
  mid = 0.5 * (cr + sh)
  seg_a = [(0.0, bot), (0.45 * w, bot), (0.80 * w, bot), (0.905 * w, bot + 0.012), (0.948 * w, bot + 0.05),
           (0.968 * w, bot + 0.11), (0.982 * w, cr)]
  seg_b = [(0.982 * w, cr), (0.972 * w, cr + 0.05), (0.971 * w, mid), (0.984 * w, sh - 0.07), (w, sh)]
  seg_c = [(w, sh), (w - 0.012, sh + 0.03), (w - 0.035, belt - 0.008), (w - 0.058, belt + 0.002),
           (lerp(w - 0.10, w - 0.078, c), belt + lerp(0.25, 0.08, c) * g),
           (lerp(0.70 * w, wr + 0.05, c), belt + 0.62 * g),
           (lerp(0.45 * w, wr, c), top - lerp(0.12, 0.06, c) * g),
           (lerp(0.22 * w, 0.62 * wr, c), top - 0.008), (0.0, top)]
  ring = np.concatenate([chaikin(seg_a, 2), chaikin(seg_b, 1)[1:], chaikin(seg_c, 2)[1:]])
  # raised centre section of the hood (power dome between two soft creases)
  hb = sstep(3.92, 4.15, u) * (1.0 - sstep(4.85, 5.08, u)) * (1.0 - c)
  if hb > 0:
    n0 = len(ring) - 12
    x = ring[n0:, 0]
    ring[n0:, 1] += 0.022 * hb * np.where(x < 0.42, (0.5 + 0.5 * np.cos(np.pi * np.clip(x / 0.42, 0, 1))) ** 0.6, 0.0)
  return ring, top, belt, bot


def station(u):
  return _station(int(round(u * 10000)))


def side_x(u, z):
  """x of the right body side at height z (monotonic part of the ring, rocker up to the roof)."""
  ring = station(u)[0]
  part = ring[3:]
  zs, xs = part[:, 1], part[:, 0]
  zs = np.maximum.accumulate(zs + np.arange(len(zs)) * 1e-7)
  return float(np.interp(z, zs, xs))


def top_z(u, x):
  ring = station(u)[0]
  j0 = int(np.argmax(ring[:, 0]))
  part = ring[j0:][::-1]   # x increasing from the roof centre
  xs = np.maximum.accumulate(part[:, 0] + np.arange(len(part)) * 1e-7)
  return float(np.interp(abs(x), xs, part[:, 1]))


@functools.lru_cache(maxsize=4096)
def _contour(end, zk):
  """Plan outline at height z around one end: [(x, y)], arclength from the centre line."""
  z = zk / 1000.0
  if end > 0:
    us = np.linspace(L, L - 1.2, 80)
  else:
    us = np.linspace(0.0, 1.2, 80)
  xs = np.array([side_x(u, z) for u in us])
  pts = [(0.0, us[0])] + [(x, u) for x, u in zip(xs, us, strict=True)]
  p = np.array(pts)
  s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(p, axis=0).T))])
  return p, s


def end_point(end, s, z, off=0.0):
  p, sv = _contour(end, int(round(z * 1000)))
  x = float(np.interp(abs(s), sv, p[:, 0]))
  y = float(np.interp(abs(s), sv, p[:, 1]))
  k = min(max(int(np.searchsorted(sv, abs(s))), 1), len(sv) - 1)
  dx, dy = p[k] - p[k - 1]
  ln = math.hypot(dx, dy) or 1.0
  nx, ny = dy / ln, -dx / ln
  if end > 0:
    nx, ny = -nx, -ny
  if nx < 0:
    nx, ny = -nx, -ny
  sg = 1.0 if s >= 0 else -1.0
  return (sg * (x + nx * off), y + ny * off, z)


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
        m = mats if isinstance(mats, str) else mats[i][j]
        if m is None:
          continue
        a, b, c, d = (base + i * n_j + j, base + (i + 1) * n_j + j, base + (i + 1) * n_j + j + 1, base + i * n_j + j + 1)
        self.faces.setdefault(m, []).append((a, b, c, d))

  def add_poly(self, pts, normal, mat):
    base = len(self.v)
    for p in pts:
      self.v.append(tuple(float(c) for c in p))
      self.vn.append(tuple(normal))
    self.faces.setdefault(mat, []).append(tuple(range(base, base + len(pts))))

  def surface(self, grid, mat, centre=None):
    grid = np.asarray(grid, float)
    n = grid_normals(grid)
    if centre is None:
      out = grid.copy()
      out[..., 1] = grid[..., 1] - np.clip(grid[..., 1], 1.1, L - 1.1)
      out[..., 0] = grid[..., 0]
      out[..., 2] = grid[..., 2] - 0.62
    else:
      out = grid - np.asarray(centre, float)
    n = np.where(np.sum(n * out, axis=2, keepdims=True) < 0, -n, n)
    self.add_grid(grid, n, mat)

  def write(self, fh, header):
    fh.write(header)
    sc = np.array([W, L, H])
    for v in self.v:
      fh.write(f"v {v[0]:.5f} {v[1]:.5f} {v[2]:.5f}\n")
    for n in self.vn:
      m = np.asarray(n, float) * sc            # normals in the normalised (file) space
      m /= max(float(np.linalg.norm(m)), 1e-9)
      fh.write(f"vn {m[0]:.4f} {m[1]:.4f} {m[2]:.4f}\n")
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


# ---------- surface-following panels ----------

def side_panel(obj, side, u0, u1, lo, hi, mat, off=0.003, nu=None, nz=3, inset=0.0):
  """Panel on the body side between z = lo(u) and hi(u); off = metres proud of the paint."""
  nu = nu or max(2, int(abs(u1 - u0) / 0.02) + 1)
  rows = []
  for u in np.linspace(u0, u1, nu):
    a, b = lo(u), hi(u)
    rows.append([(side * (side_x(u, z) + off - inset), u, z) for z in np.linspace(a, b, nz)])
  obj.surface(rows, mat)


def end_panel(obj, end, s0, s1, lo, hi, mat, off=0.003, ns=None, nz=3):
  ns = ns or max(2, int(abs(s1 - s0) / 0.02) + 1)
  rows = []
  for s in np.linspace(s0, s1, ns):
    a, b = lo(s), hi(s)
    rows.append([end_point(end, s, z, off) for z in np.linspace(a, b, nz)])
  obj.surface(rows, mat)


def top_panel(obj, u0, u1, half, mat, off=0.003, nu=None, nx=24):
  nu = nu or max(2, int(abs(u1 - u0) / 0.03) + 1)
  rows = []
  for u in np.linspace(u0, u1, nu):
    hw = half(u)
    rows.append([(x, u, top_z(u, x) + off) for x in np.linspace(-hw, hw, nx)])
  obj.surface(rows, mat)


def rrect(x0, x1, z0, z1, r):
  """lo(x), hi(x) of a rounded rectangle."""
  def lo(x):
    d = max(0.0, max(x0 + r - x, x - (x1 - r)))
    return z0 + r - math.sqrt(max(0.0, r * r - d * d)) if d > 0 else z0

  def hi(x):
    d = max(0.0, max(x0 + r - x, x - (x1 - r)))
    return z1 - r + math.sqrt(max(0.0, r * r - d * d)) if d > 0 else z1
  return lo, hi


# ---------- body ----------

def build_body(obj):
  us = np.unique(np.concatenate([
    np.linspace(0.0, 0.40, 14), np.linspace(0.40, 0.55, 4), np.linspace(0.55, 1.55, 51), np.linspace(1.55, 3.65, 36),
    np.linspace(3.65, 4.65, 51), np.linspace(4.65, 4.80, 4), np.linspace(4.80, L, 14)]).round(5))
  rings = [station(u)[0] for u in us]
  g = np.array([[(x, u, z) for x, z in ring] for u, ring in zip(us, rings, strict=True)])
  orig = g.copy()
  # wheel wells: push the shell in around each wheel
  for i, u in enumerate(us):
    w_in = half_width(u) - 0.22
    for j in range(g.shape[1]):
      x, z = g[i, j, 0], g[i, j, 2]
      for ay in AXLES:
        if math.hypot(u - ay, z - TIRE_R) < ARCH_R + 0.012 and x > w_in:
          g[i, j, 0] = w_in
  mats = []
  for i, u in enumerate(us):
    row = []
    for j in range(g.shape[1] - 1):
      a, b = orig[i, j], orig[i, j + 1]
      zc = 0.5 * (a[2] + b[2])
      dx, dz = b[0] - a[0], b[2] - a[2]
      ln = math.hypot(dx, dz) or 1.0
      nz = -dx / ln
      m = "paint"
      if nz < -0.55:
        m = "trim"
      for ay in AXLES:
        if math.hypot(u - ay, zc - TIRE_R) < ARCH_R + 0.012 and a[0] > half_width(u) - 0.30:
          m = "well"
      row.append(m)
    mats.append(row)
  for side in (1, -1):
    gs = g.copy()
    gs[..., 0] *= side
    m = mats if side > 0 else [list(reversed(r)) for r in mats]
    gs2 = gs if side > 0 else gs[:, ::-1]
    obj.surface(gs2, m)
  # end caps
  for u, ny in ((L, 1), (0.0, -1)):
    ring = station(u)[0]
    pts = [(x, u, z) for x, z in ring] + [(-x, u, z) for x, z in reversed(ring[:-1])]
    if ny < 0:
      pts = pts[::-1]
    obj.add_poly(pts, (0, ny, 0), "paint")


def wheel_arches(obj):
  for side in (1, -1):
    for ay in AXLES:
      angs = np.linspace(math.radians(-24), math.radians(204), 56)
      lip, liner = [], []
      for a in angs:
        u = ay + math.cos(a) * 1.0
        row_l = []
        for r, dx in ((ARCH_R - 0.004, -0.045), (ARCH_R + 0.002, -0.008), (ARCH_R + 0.016, 0.006),
                      (ARCH_R + 0.034, 0.002)):
          uu, zz = ay + r * math.cos(a), TIRE_R + r * math.sin(a)
          row_l.append((side * (side_x(uu, zz) + dx), uu, zz))
        lip.append(row_l)
        uu, zz = ay + (ARCH_R - 0.006) * math.cos(a), TIRE_R + (ARCH_R - 0.006) * math.sin(a)
        xo = side_x(uu, zz) - 0.04
        liner.append([(side * x, uu, zz) for x in np.linspace(xo, half_width(u) - 0.22, 3)])
      obj.surface(lip, "paint", centre=(0, ay, TIRE_R - 0.25))
      obj.surface(liner, "well", centre=(side * 5.0, ay, TIRE_R))


def glass_and_pillars(obj):
  def dlo_top(u):
    return float(np.interp(u, [p[0] for p in DLO_TOP], [p[1] for p in DLO_TOP]))

  def belt(u):
    return prof(_BELT, u) + 0.012
  for side in (1, -1):
    side_panel(obj, side, DLO_U0 + 0.02, DLO_U1 - 0.02, belt, lambda u: max(belt(u) + 0.002, dlo_top(u) - 0.012),
               "glass", off=0.002, nz=6)
    # chrome surround: top arc + belt strip
    side_panel(obj, side, DLO_U0, DLO_U1, lambda u: max(belt(u), dlo_top(u) - 0.014), lambda u: dlo_top(u) + 0.006,
               "chrome", off=0.005, nz=2)
    side_panel(obj, side, DLO_U0, DLO_U1, lambda u: belt(u) - 0.016, belt, "chrome", off=0.006, nz=2)
    # gloss-black B pillar and the rear quarter divider
    side_panel(obj, side, B_PILLAR[0], B_PILLAR[1], belt, lambda u: dlo_top(u) - 0.012, "trim", off=0.006, nz=4)
    side_panel(obj, side, 1.555, 1.575, belt, lambda u: dlo_top(u) - 0.01, "chrome2", off=0.006, nz=3)
  # windscreen and rear window (follow the roof surface)

  def ws_half(u):
    return max(0.05, side_x(u, prof(_BELT, u) + 0.045) - 0.05)
  top_panel(obj, 3.14, 3.90, ws_half, "glass", nu=26)
  top_panel(obj, 1.04, 1.86, lambda u: max(0.05, side_x(u, prof(_BELT, u) + 0.05) - 0.06), "glass", nu=28)
  # rear-glass third brake light strip and the rear deck emblem gap
  top_panel(obj, 1.80, 1.83, lambda u: 0.18, "tail2", off=0.005, nu=2, nx=4)


def door_details(obj):
  def belt(u):
    return prof(_BELT, u) - 0.03

  def bot(u):
    return prof(_BOT, u) + 0.10
  for side in (1, -1):
    for uc in (3.64, B_PILLAR[0] + 0.03, 1.50):
      side_panel(obj, side, uc - 0.004, uc + 0.004, bot, belt, "gap", off=0.0015, nz=12, nu=2)
    side_panel(obj, side, 1.50, 3.64, lambda u: bot(u) - 0.004, bot, "gap", off=0.0015, nz=2)
    # lower door chrome strip and the chrome sill below the crease
    side_panel(obj, side, 1.52, 3.62, lambda u: prof(_BOT, u) + 0.125, lambda u: prof(_BOT, u) + 0.145, "chrome",
               off=0.006, nz=2)
    # chrome door handles (a little proud, with a dark recess behind)
    for uc in (2.98, 1.92):
      side_panel(obj, side, uc - 0.095, uc + 0.095, lambda u: 0.905, lambda u: 0.945, "gap", off=0.002, nz=2)
      side_panel(obj, side, uc - 0.085, uc + 0.085, lambda u: 0.912, lambda u: 0.936, "chrome", off=0.016, nz=3)
    # front fender garnish (chrome strip behind the front arch)
    side_panel(obj, side, 3.70, 3.86, lambda u: 0.70, lambda u: 0.712, "chrome", off=0.004, nz=2)


def mirrors(obj):
  for side in (1, -1):
    u0, z0 = 3.50, 1.045
    xb = side_x(u0, z0)
    # stalk: from inside the door skin out to the housing
    rows = []
    for t in np.linspace(0.0, 1.0, 5):
      x = xb - 0.03 + t * 0.12
      rows.append([(side * x, u0 + 0.03 * math.cos(a) - 0.01, z0 + 0.018 * math.sin(a)) for a in np.linspace(0, 2 * math.pi, 13)])
    obj.surface(rows, "paint", centre=(side * (xb + 0.05), u0, z0))
    # housing: lofted rounded shell, glass on the back face
    cx, cy, cz = xb + 0.16, u0 - 0.03, z0 + 0.06
    rows, mrows = [], []
    for t in np.linspace(-1.0, 1.0, 12):
      sx = 0.11 * t
      k = math.sqrt(max(0.0, 1 - t ** 4)) if abs(t) < 1 else 0.0
      ring = []
      for a in np.linspace(0, 2 * math.pi, 25):
        ry = (0.075 if math.cos(a) > 0 else 0.03) * k
        ring.append((side * (cx + sx), cy + ry * math.cos(a), cz + 0.065 * k * math.sin(a) - 0.008 * t))
      rows.append(ring)
      mrows.append([(side * (cx + sx), cy - 0.031 * k, cz + 0.058 * k * math.sin(a) - 0.008 * t)
                    for a in np.linspace(-math.pi / 2, math.pi / 2, 6)])
    obj.surface(rows, "paint", centre=(side * cx, cy, cz))
    obj.surface(mrows[1:-1], "glass", centre=(side * cx, cy + 0.5, cz))
    # side repeater on the lower front of the housing
    rep = []
    for t in np.linspace(-0.2, 0.95, 8):
      k = math.sqrt(max(0.0, 1 - t ** 4))
      rep.append([(side * (cx + 0.11 * t), cy + 0.077 * k * math.cos(a), cz + 0.067 * k * math.sin(a) - 0.008 * t)
                  for a in np.linspace(-0.75, -0.35, 3)])
    obj.surface(rep, "amber", centre=(side * cx, cy, cz))


# ---------- front ----------

def front_details(obj):
  e = 1
  # large rounded grille (slightly narrower at the bottom), recessed dark mesh, chrome frame
  gx_t, gx_b, z0, z1 = 0.60, 0.565, 0.48, 0.78

  def ghalf(z):
    return gx_b + (gx_t - gx_b) * (z - z0) / (z1 - z0)

  def frame_panel(grow, mat, off):
    rows = []
    for t in np.linspace(0.0, 1.0, 120):
      # walk around the rounded trapezoid, rows = outward offsets
      ang = 2 * math.pi * t
      pts = []
      for d in grow:
        cx_ = math.cos(ang)
        cz_ = math.sin(ang)
        ex = abs(cx_) ** 0.18 * (1 if cx_ >= 0 else -1)
        ez = abs(cz_) ** 0.18 * (1 if cz_ >= 0 else -1)
        zz = 0.5 * (z0 + z1) + ez * (0.5 * (z1 - z0) + d)
        xx = ex * (ghalf(min(max(zz, z0), z1)) + d)
        pts.append(end_point(e, xx, zz, off))
      rows.append(pts)
    obj.surface(rows, mat, centre=(0, L - 3, 0.6))
  frame_panel([0.0, 0.01, 0.024], "chrome", 0.014)
  end_panel(obj, e, -gx_b + 0.01, gx_b - 0.01, lambda s: z0 + 0.012, lambda s: z1 - 0.012, "grille", off=0.002, nz=4)
  # raised chrome diamonds (four faces each so the light catches them)
  rows_n, dz = 6, (z1 - z0 - 0.03) / 6
  for rr in range(rows_n):
    zc = z0 + 0.015 + (rr + 0.5) * dz
    half = ghalf(zc) - 0.04
    step = 0.068
    offs = 0.0 if rr % 2 == 0 else step / 2
    n = int(half / step) + 1
    for k in range(-n, n + 1):
      xc = k * step + offs
      if abs(xc) + 0.03 > half:
        continue
      pc = end_point(e, xc, zc, 0.016)
      pts = [end_point(e, xc, zc - dz * 0.48, 0.004), end_point(e, xc + 0.03, zc, 0.004),
             end_point(e, xc, zc + dz * 0.48, 0.004), end_point(e, xc - 0.03, zc, 0.004)]
      for a, b in zip(pts, pts[1:] + pts[:1], strict=True):
        nrm = np.cross(np.subtract(b, a), np.subtract(pc, a))
        nrm = nrm / (np.linalg.norm(nrm) or 1.0)
        if nrm[1] < 0:
          nrm = -nrm
        obj.add_poly([a, b, pc], tuple(nrm), "chrome" if (a[2] + b[2]) > 2 * zc - 0.001 else "chrome2")
  # emblem on the hood nose
  em = []
  for t in np.linspace(-1, 1, 9):
    em.append([end_point(e, 0.075 * t, 0.805 + 0.024 * math.sqrt(max(0.0, 1 - t * t)) * v, 0.006) for v in (-1, 0, 1)])
  obj.surface(em, "chrome", centre=(0, L - 3, 0.6))
  em2 = []
  for t in np.linspace(-0.85, 0.85, 7):
    em2.append([end_point(e, 0.064 * t, 0.805 + 0.016 * math.sqrt(max(0.0, 1 - t * t)) * v, 0.008) for v in (-1, 1)])
  obj.surface(em2, "trim", centre=(0, L - 3, 0.6))
  # slim headlamps, rising outward and wrapping round the corners
  for sg in (1, -1):
    def hl_lo(s):
      t = (abs(s) - 0.63) / 0.42
      return 0.695 + 0.085 * t * t
    def hl_hi(s):
      t = (abs(s) - 0.63) / 0.42
      return 0.785 + 0.045 * t + 0.02 * t * t
    s0, s1 = 0.63, 1.05
    end_panel(obj, e, sg * s0, sg * s1, hl_lo, hl_hi, "lamp_in", off=0.004, nz=4)
    end_panel(obj, e, sg * s0, sg * s1, lambda s: hl_hi(s) - 0.016, hl_hi, "lamp", off=0.006, nz=2)   # LED line
    end_panel(obj, e, sg * (s0 + 0.03), sg * (s0 + 0.16), lambda s: hl_lo(s) + 0.01, lambda s: hl_lo(s) + 0.028,
              "lamp", off=0.006, nz=2)
    end_panel(obj, e, sg * s0, sg * s1, lambda s: hl_lo(s) - 0.008, hl_lo, "chrome2", off=0.005, nz=2)
  # lower bumper: blank plate, dark side intakes, chrome lip rising at the corners
  end_panel(obj, e, -0.275, 0.275, lambda s: 0.385, lambda s: 0.505, "trim", off=0.02, nz=2)
  end_panel(obj, e, -0.26, 0.26, lambda s: 0.392, lambda s: 0.498, "plate", off=0.022, nz=2)
  for sg in (1, -1):
    end_panel(obj, e, sg * 0.36, sg * 0.80, lambda s: 0.33, lambda s: 0.43 - 0.06 * max(0.0, abs(s) - 0.6), "grille",
              off=0.003, nz=3)

  def lip(s):
    t = max(0.0, abs(s) - 0.62) / 0.40
    return 0.30 + 0.17 * t * t
  for sg in (1, -1):
    end_panel(obj, e, 0.0, sg * 1.02, lip, lambda s: lip(s) + 0.022, "chrome", off=0.008, nz=2)


# ---------- rear ----------

def rear_details(obj):
  e = -1
  for sg in (1, -1):
    # wrap-round tail lamps: on the rear face from |s|=0.55 round the corner onto the side
    def lo(s):
      t = max(0.0, abs(s) - 0.80) / 0.35
      return 0.76 + 0.11 * t
    def hi(s):
      t = max(0.0, abs(s) - 0.80) / 0.35
      return 0.99 - 0.02 * t
    s0, s1 = 0.55, 1.15
    end_panel(obj, e, sg * (s0 - 0.012), sg * (s1 + 0.012), lambda s: lo(s) - 0.012, lambda s: hi(s) + 0.012,
              "chrome2", off=0.003, nz=3)
    end_panel(obj, e, sg * s0, sg * s1, lo, hi, "tail2", off=0.006, nz=4)
    # horizontal C-shaped light bars (open towards the centre)
    for zb in (0.815, 0.875, 0.935):
      end_panel(obj, e, sg * (s0 + 0.06), sg * (s1 - 0.05), lambda s, zb=zb: zb, lambda s, zb=zb: zb + 0.012,
                "tail", off=0.009, nz=2)
    end_panel(obj, e, sg * (s1 - 0.08), sg * (s1 - 0.05), lambda s: 0.815, lambda s: 0.947, "tail", off=0.009, nz=2)
  # trunk: emblem, recessed blank plate, chrome strip on the bumper, black diffuser and trapezoid tips
  end_panel(obj, e, -0.30, 0.30, lambda s: 0.785, lambda s: 0.965, "trim", off=0.001, nz=2)
  end_panel(obj, e, -0.56, 0.56, lambda s: 0.758, lambda s: 0.765, "gap", off=0.002, nz=2)   # trunk shut line
  end_panel(obj, e, -0.27, 0.27, lambda s: 0.80, lambda s: 0.95, "plate", off=0.002, nz=2)
  em = []
  for t in np.linspace(-1, 1, 9):
    em.append([end_point(e, 0.07 * t, 0.98 + 0.012 * math.sqrt(max(0.0, 1 - t * t)) * v, 0.005) for v in (-1, 0, 1)])
  obj.surface(em, "chrome", centre=(0, 3, 0.6))
  for sg in (1, -1):
    end_panel(obj, e, 0.0, sg * 1.05, lambda s: 0.375, lambda s: 0.40, "chrome", off=0.012, nz=2)
  end_panel(obj, e, -0.92, 0.92, lambda s: 0.272, lambda s: 0.36, "trim", off=0.004, nz=2)
  for sg in (1, -1):
    end_panel(obj, e, sg * 0.42, sg * 0.73, lambda s: 0.285, lambda s: 0.362, "chrome", off=0.03, nz=2)
    end_panel(obj, e, sg * 0.44, sg * 0.71, lambda s: 0.298, lambda s: 0.348, "gap", off=0.034, nz=2)


# ---------- wheels ----------

def wheel(obj, y, side):
  seg = 48
  x_out = side * (HW - 0.035)
  x_in = x_out - side * TIRE_W
  cz = TIRE_R
  angs = np.linspace(0, 2 * math.pi, seg + 1)

  def ring(x, r):
    return [(x, y + r * math.cos(a), cz + r * math.sin(a)) for a in angs]
  # tyre: rounded profile (tread -> shoulder -> bulged sidewall -> rim)
  prof_t = [(x_in + side * 0.02, TIRE_R), (x_out - side * 0.03, TIRE_R), (x_out - side * 0.008, TIRE_R - 0.015),
            (x_out, TIRE_R - 0.045), (x_out + side * 0.004, 0.30), (x_out, RIM_R + 0.012),
            (x_out - side * 0.006, RIM_R + 0.002)]
  obj.surface([ring(x, r) for x, r in prof_t], "tire", centre=(x_out - side * 0.12, y, cz))
  # rim lip and barrel
  obj.surface([ring(x_out - side * 0.004, RIM_R + 0.002), ring(x_out + side * 0.002, RIM_R - 0.006),
               ring(x_out - side * 0.004, RIM_R - 0.018)], "rim", centre=(x_out - side * 0.3, y, cz))
  obj.surface([ring(x_out - side * 0.004, RIM_R - 0.018), ring(x_out - side * 0.10, RIM_R - 0.03)], "rim_dark",
              centre=(x_out - side * 0.3, y, cz))
  # brake disc and dark hub face deep inside
  disc = [ring(x_out - side * 0.10, r) for r in (0.17, 0.10)]
  obj.surface(disc, "disc", centre=(x_out - side * 0.5, y, cz))
  obj.surface([ring(x_out - side * 0.10, r) for r in (RIM_R - 0.03, 0.17)], "rim_dark", centre=(x_out - side * 0.5, y, cz))
  # ten slim silver spokes with depth: face dishes in towards the hub, flanks darker
  for k in range(10):
    a = 2 * math.pi * k / 10 + 0.1
    pts_face, pts_l, pts_r = [], [], []
    for r in np.linspace(0.07, RIM_R - 0.012, 6):
      t = (r - 0.07) / (RIM_R - 0.082)
      hw = (0.016 + 0.012 * t) / r          # angular half width
      depth = -0.032 * (1 - t) - 0.004
      xf = x_out + side * depth
      xb = xf - side * 0.035
      la, ra = a - hw, a + hw
      fl = (xf, y + r * math.cos(la), cz + r * math.sin(la))
      fr = (xf, y + r * math.cos(ra), cz + r * math.sin(ra))
      pts_face.append([fl, fr])
      pts_l.append([(xb, fl[1], fl[2]), fl])
      pts_r.append([fr, (xb, fr[1], fr[2])])
    obj.surface(pts_face, "rim", centre=(x_out - side * 0.3, y, cz))
    obj.surface(pts_l, "chrome2", centre=(x_out, y + 0.3 * math.cos(a + 1.57), cz + 0.3 * math.sin(a + 1.57)))
    obj.surface(pts_r, "chrome2", centre=(x_out, y + 0.3 * math.cos(a - 1.57), cz + 0.3 * math.sin(a - 1.57)))
  # hub dish and centre cap
  obj.surface([ring(x_out - side * 0.034, 0.075), ring(x_out - side * 0.030, 0.055)], "rim", centre=(x_out - side * 0.3, y, cz))
  obj.surface([ring(x_out - side * 0.028, 0.055), ring(x_out - side * 0.022, 0.045), ring(x_out - side * 0.021, 0.0)],
              "chrome", centre=(x_out - side * 0.3, y, cz))
  obj.surface([ring(x_out - side * 0.019, 0.032), ring(x_out - side * 0.019, 0.022)], "rim_dark",
              centre=(x_out - side * 0.3, y, cz))


def build():
  obj = Obj()
  build_body(obj)
  wheel_arches(obj)
  glass_and_pillars(obj)
  door_details(obj)
  mirrors(obj)
  front_details(obj)
  rear_details(obj)
  for side in (1, -1):
    for y in AXLES:
      wheel(obj, y, side)
  v = np.array(obj.v)
  v[:, 0] /= W
  v[:, 1] = v[:, 1] / L - 0.5
  v[:, 2] /= H
  v[:, 1] = np.clip(v[:, 1], -0.505, 0.505)
  v[:, 2] = np.clip(v[:, 2], 0.0, 1.0)
  obj.v = [tuple(p) for p in v]
  return obj


if __name__ == "__main__":
  out = sys.argv[1]
  obj = build()
  header = ("# Kia K9 (2020 RJ facelift) parked-screen model for the CarrotPilot cluster HUD (v4).\n"
            + "# Original design generated by make_k9_park_model.py: outlines traced by hand from the owner's\n"
            + "# photos (shape reference only) and published dimensions (5120 x 1915 x 1490 mm, wheelbase 3105 mm);\n"
            + "# no third-party mesh or photo pixels are included, number plates are blank.\n"
            + "# Axes: x=right, y=forward, z=up; normalised to x +-0.5, y +-0.5, z 0..1 (renderer scales to real size).\n")
  with open(out, "w") as fh:
    obj.write(fh, header)
  print(out, len(obj.v), sum(len(f) for f in obj.faces.values()))
