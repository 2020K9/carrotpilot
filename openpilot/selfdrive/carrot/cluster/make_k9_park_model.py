#!/usr/bin/env python3
"""Generate the 2020 Kia K9 (RJ facelift) parked-screen model for the cluster HUD.

Original low-poly design drawn from published exterior dimensions only (no third-party mesh or image):
length 5120 mm, width 1915 mm, height 1490 mm, wheelbase 3105 mm, 19-inch wheels.
Axes match the other cluster models: x = right, y = forward, z = up; normalised to x +-0.5, y +-0.5, z 0..1.
"""
import math
import sys

import numpy as np

L, W, H = 5.12, 1.915, 1.49
WB, R_AXLE = 3.105, 1.02          # rear axle 1.02 m from the rear bumper, front axle at 4.125 m
TIRE_R, TIRE_W = 0.355, 0.245

MAT = {
  "paint": "rgb_c4c8cc",   # silver metallic (reads on both the light and the dark theme)
  "glass": "rgb_1a1e24",
  "trim": "rgb_0e0f10",
  "chrome": "rgb_d8dbde",
  "grille": "rgb_24272b",
  "lamp": "rgb_eef3ff",
  "drl": "rgb_ffffff",
  "tail": "rgb_a8121c",
  "tire": "rgb_141414",
  "rim": "rgb_b4b8bc",
}


def smooth(points, u):
  xs, ys = [q[0] for q in points], [q[1] for q in points]
  out = np.interp(u, xs, ys)
  return out


def sstep(a, b, x):
  t = min(1.0, max(0.0, (x - a) / (b - a)))
  return t * t * (3 - 2 * t)


# side profile (u = metres from the rear bumper)
TOP = [(0.0, 0.80), (0.06, 0.95), (0.18, 1.025), (0.55, 1.05), (0.98, 1.065), (1.30, 1.25), (1.62, 1.42),
       (1.95, 1.475), (2.45, 1.49), (2.85, 1.47), (3.15, 1.36), (3.55, 1.13), (3.72, 1.04), (4.20, 1.005),
       (4.70, 0.965), (4.95, 0.90), (5.07, 0.80), (5.12, 0.62)]
BELT = [(0.0, 0.80), (0.18, 1.0), (1.0, 1.0), (3.7, 0.985), (4.7, 0.95), (5.12, 0.62)]
BOTTOM = [(0.0, 0.42), (0.10, 0.27), (0.35, 0.20), (4.75, 0.20), (5.0, 0.24), (5.12, 0.36)]


def half_width(u):
  end = min(u, L - u)
  return 0.5 * W * (0.86 + 0.14 * math.sqrt(sstep(0.0, 0.75, end)))


def station(u):
  top, belt, bot = smooth(TOP, u), smooth(BELT, u), smooth(BOTTOM, u)
  belt = min(belt, top)
  wb = half_width(u)
  cab = sstep(0.0, 0.35, top - belt)
  wc_belt = wb - 0.07
  wc_top = wc_belt + (0.60 - wc_belt) * cab
  crown = 0.03 + 0.02 * cab
  ring = [
    (0.0, bot), (wb * 0.80, bot), (wb * 0.97, bot + 0.07), (wb, bot + 0.22), (wb, 0.72 * belt + 0.28 * bot + 0.12),
    (wb * 0.985, belt - 0.05), (wc_belt, belt), (wc_belt - 0.02 * cab, belt + 0.03 * cab),
    (wc_top, top - crown * 0.6), (wc_top * 0.55, top - crown * 0.1), (0.0, top),
  ]
  return ring, cab, top, belt


def body_material(u, j, cab, top):
  # ring segment j joins ring point j and j+1 (right side)
  if j == 7 and cab > 0.25 and 0.95 < u < 3.62:
    return "glass"
  if j in (8, 9) and cab > 0.25 and 0.95 < u < 3.62:
    if u < 1.66 or u > 2.98:
      return "glass"              # rear window / windscreen
    return "paint"
  if j == 6 and cab > 0.25 and 0.95 < u < 3.62:
    return "chrome"               # window line
  if j in (0, 1):
    return "trim"
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
        if m is None:
          continue
        a, b, c, d = (base + i * n_j + j, base + (i + 1) * n_j + j, base + (i + 1) * n_j + j + 1, base + i * n_j + j + 1)
        self.faces.setdefault(m, []).append((a, b, c, d))

  def add_poly(self, pts, normal, mat):
    base = len(self.v)
    for p in pts:
      self.v.append(tuple(p))
      self.vn.append(tuple(normal))
    self.faces.setdefault(mat, []).append(tuple(range(base, base + len(pts))))

  def box(self, c, s, mat):
    cx, cy, cz = c
    sx, sy, sz = (x / 2 for x in s)
    for axis in range(3):
      for sign in (-1, 1):
        n = [0, 0, 0]
        n[axis] = sign
        u_ax, v_ax = [a for a in range(3) if a != axis]
        pts = []
        for du, dv in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
          p = [cx, cy, cz]
          half = (sx, sy, sz)
          p[axis] += sign * half[axis]
          p[u_ax] += du * half[u_ax]
          p[v_ax] += dv * half[v_ax]
          pts.append(p)
        self.add_poly(pts, n, mat)

  def wheel(self, y, side):
    seg = 28
    x_out = side * (half_width(y) + 0.005)
    x_in = x_out - side * TIRE_W
    cz = TIRE_R
    for k in range(seg):
      a0, a1 = 2 * math.pi * k / seg, 2 * math.pi * (k + 1) / seg
      def p(x, a, r=TIRE_R, y=y, cz=cz):
        return (x, y + r * math.cos(a), cz + r * math.sin(a))
      n = ((0, math.cos((a0 + a1) / 2), math.sin((a0 + a1) / 2)))
      self.add_poly([p(x_in, a0), p(x_in, a1), p(x_out, a1), p(x_out, a0)], n, "tire")
      # sidewall ring and rim disc (multi-spoke look: alternating rim / dark)
      r_rim = 0.245
      self.add_poly([p(x_out, a0), p(x_out, a1), p(x_out + side * 0.004, a1, r_rim), p(x_out + side * 0.004, a0, r_rim)],
                    (side, 0, 0), "tire")
      mat = "rim" if k % 2 == 0 else "grille"
      self.add_poly([(x_out + side * 0.006, y, cz), p(x_out + side * 0.004, a0, r_rim), p(x_out + side * 0.004, a1, r_rim)],
                    (side, 0, 0), mat)

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


def build():
  obj = Obj()
  us = np.concatenate([np.linspace(0, 0.3, 7), np.linspace(0.34, 4.78, 75), np.linspace(4.82, L, 7)])
  rings, mats_r = [], []
  for u in us:
    ring, cab, top, belt = station(u)
    rings.append(ring)
    mats_r.append([body_material(u, j, cab, top) for j in range(len(ring) - 1)])
  for side in (1, -1):
    g = np.array([[(side * x, u, z) for x, z in ring] for u, ring in zip(us, rings, strict=True)])
    n = grid_normals(g)
    # point every normal away from the car centre line (z = 0.55 m) at its own station
    out = g.copy()
    out[..., 1] = 0.0
    out[..., 2] -= 0.55
    n = np.where(np.sum(n * out, axis=2, keepdims=True) < 0, -n, n)
    if side < 0:
      g = g[:, ::-1]
      n = n[:, ::-1]
      mats = [list(reversed(m)) for m in mats_r]
    else:
      mats = mats_r
    obj.add_grid(g, n, mats)

  # front and rear faces
  for u, ny in ((us[-1], 1), (us[0], -1)):
    ring = rings[-1] if ny > 0 else rings[0]
    pts = [(x, u, z) for x, z in ring] + [(-x, u, z) for x, z in reversed(ring[:-1])]
    if ny < 0:
      pts = pts[::-1]
    obj.add_poly(pts, (0, ny, 0), "paint")

  yf = L + 0.006
  # K9 RJ facelift front: tall chrome-framed grille with vertical bars, slim lamps with four-dot DRL, wide lower intake
  obj.box((0, yf, 0.555), (0.70, 0.02, 0.36), "chrome")
  obj.box((0, yf + 0.006, 0.555), (0.64, 0.02, 0.31), "grille")
  for k in range(13):
    x = -0.30 + k * 0.05
    obj.box((x, yf + 0.013, 0.555), (0.012, 0.012, 0.30), "chrome")
  for side in (1, -1):
    obj.box((side * 0.66, L - 0.03, 0.66), (0.42, 0.06, 0.075), "lamp")
    obj.box((side * 0.66, L - 0.005, 0.615), (0.40, 0.02, 0.012), "drl")
    for k in range(4):
      obj.box((side * (0.50 + k * 0.075), L + 0.002, 0.66), (0.035, 0.02, 0.035), "drl")
  obj.box((0, yf, 0.30), (1.25, 0.02, 0.10), "trim")
  obj.box((0, yf + 0.005, 0.25), (0.52, 0.02, 0.11), "trim")  # plate area

  # rear: full-width light bar, chrome strip and plate
  yr = -0.006
  obj.box((0, yr, 0.93), (1.68, 0.02, 0.065), "tail")
  for side in (1, -1):
    obj.box((side * 0.70, yr - 0.002, 0.90), (0.30, 0.02, 0.11), "tail")
  obj.box((0, yr, 0.80), (1.10, 0.02, 0.018), "chrome")
  obj.box((0, yr, 0.62), (0.52, 0.02, 0.12), "trim")
  obj.box((0, yr, 0.33), (1.50, 0.02, 0.07), "trim")
  for side in (1, -1):
    obj.box((side * 0.45, yr, 0.30), (0.18, 0.03, 0.06), "chrome")  # exhaust tips

  for side in (1, -1):
    # mirrors
    obj.box((side * (half_width(3.5) - 0.02), 3.5, 1.0), (0.12, 0.06, 0.04), "trim")
    obj.box((side * (half_width(3.5) + 0.06), 3.5, 1.03), (0.16, 0.10, 0.09), "paint")
    # lower chrome side sill trim
    obj.box((side * (half_width(2.5) + 0.004), 2.55, 0.34), (0.01, 2.1, 0.03), "chrome")
    # wheel arches (dark gaps), then wheels
    for y in (R_AXLE, R_AXLE + WB):
      seg = 20
      for k in range(seg):
        a0, a1 = math.pi * k / seg, math.pi * (k + 1) / seg
        r0, r1 = TIRE_R + 0.02, TIRE_R + 0.06
        x = side * (half_width(y) + 0.003)
        pts = [(x, y + r0 * math.cos(a0), TIRE_R + r0 * math.sin(a0)), (x, y + r0 * math.cos(a1), TIRE_R + r0 * math.sin(a1)),
               (x, y + r1 * math.cos(a1), TIRE_R + r1 * math.sin(a1)), (x, y + r1 * math.cos(a0), TIRE_R + r1 * math.sin(a0))]
        obj.add_poly(pts, (side, 0, 0), "trim")
      obj.wheel(y, side)

  # normalise: x +-0.5, y +-0.5, z 0..1
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
            + "# Original low-poly design generated by make_k9_park_model.py from published dimensions only\n"
            + "# (5120 x 1915 x 1490 mm, wheelbase 3105 mm); no third-party mesh or photo was used.\n"
            + "# Axes: x=right, y=forward, z=up; normalised to x +-0.5, y +-0.5, z 0..1 (renderer scales to real size).\n")
  with open(out, "w") as fh:
    obj.write(fh, header)
  print(out, len(obj.v), sum(len(f) for f in obj.faces.values()))
