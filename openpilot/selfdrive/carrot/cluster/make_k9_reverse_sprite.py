#!/usr/bin/env python3
"""Top view of the repository K9 model (assets/models/k9/k9_cluster.obj) for the K9 reverse (R) screen.

Orthographic look straight down, car front to the left, car right side up (same as the R screen camera).
Software z-buffer of the model faces with the model materials, black clear-coat studio reflection,
lit lamps and a soft contact shadow. No photo pixels, no third-party mesh. Output: assets/images/k9_reverse_car.png
Frame: PAD metres of transparent margin around the 5120 x 1915 mm body at PPM pixels per metre.
"""
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter

L, W, H = 5.12, 1.915, 1.49
PPM = 200
PAD = 0.20
SS = 2  # raster supersample, box-filtered down
HERE = Path(__file__).resolve().parent
OBJ = HERE / "assets" / "models" / "k9" / "k9_cluster.obj"
OUT = HERE / "assets" / "images" / "k9_reverse_car.png"
TAIL = ("rgb_ff3a40", "rgb_6e0c16")


def load(path):
  verts, faces, mats, cur = [], [], [], ""
  with open(path) as fh:
    for line in fh:
      if line.startswith("v "):
        verts.append([float(t) for t in line.split()[1:4]])
      elif line.startswith("usemtl"):
        cur = line.split()[1]
      elif line.startswith("f "):
        idx = [int(t.split("/")[0]) - 1 for t in line.split()[1:]]
        for k in range(1, len(idx) - 1):
          faces.append((idx[0], idx[k], idx[k + 1]))
          mats.append(cur)
  return np.array(verts) * (W, L, H), np.array(faces), mats


def shade(mat, n):
  """Colour of one face seen from straight above; n = unit normal with z up."""
  base = np.array([int(mat[4 + i * 2:6 + i * 2], 16) for i in range(3)], float)
  nz = abs(n[2])
  diffuse = 0.45 + 0.55 * max(0.0, float(np.dot(n, (-0.25, 0.35, 0.90))))
  rx, rz = 2 * nz * n[0], 2 * nz * nz - 1          # reflected view ray x / z (view straight down)
  sky = 0.5 + 0.5 * rz
  strip = float(np.exp(-((abs(rx) - 0.55) / 0.22) ** 2))  # two long soft boxes along the car sides
  if mat == TAIL[0]:
    return np.array((255.0, 44.0, 56.0))
  if mat == TAIL[1]:
    return np.array((200.0, 22.0, 34.0))
  if mat == "rgb_e89a2c":
    return np.array((255.0, 170.0, 60.0))
  if mat == "rgb_e9eef6":
    return np.array((236.0, 244.0, 255.0))
  if base.mean() > 120:                              # chrome, rims
    return np.clip(base * (0.55 + 0.45 * diffuse) + 40 * sky, 0, 255)
  if mat == "rgb_0a0c10":                            # glass
    return np.clip(np.array((14.0, 20.0, 32.0)) + np.array((70.0, 88.0, 118.0)) * (0.25 * sky + 0.55 * strip), 0, 255)
  if mat == "rgb_1c1f24":                            # black clear coat
    env = np.array((22.0, 26.0, 36.0)) * sky + np.array((190.0, 200.0, 220.0)) * strip * sky
    return np.clip(base * 0.75 * diffuse + 0.34 * env, 0, 255)
  return np.clip(base * diffuse + 10 * sky, 0, 255)


def _band(rgb, body, ppm, rear, half_from, half_to, depth, colour, core=None):
  """Paint a lamp strip that follows the front / rear outline, depth metres inward from the body edge."""
  h = body.shape[0]
  centre = h / 2
  for r in range(h):
    x_m = abs((r + 0.5 - centre) / ppm)
    if not (half_from <= x_m <= half_to):
      continue
    cols = np.nonzero(body[r])[0]
    if cols.size == 0:
      continue
    d0, d1 = int(round(depth[0] * ppm)), int(round(depth[1] * ppm))
    if rear:
      c0, c1 = cols[-1] - d1, cols[-1] - d0
    else:
      c0, c1 = cols[0] + d0, cols[0] + d1
    rgb[r, max(c0, 0):c1 + 1] = colour
    if core is not None:
      mid = (c0 + c1) // 2
      rgb[r, mid - max(1, (c1 - c0) // 5):mid + max(1, (c1 - c0) // 5) + 1] = core


def edge_lights(rgb, body, ppm):
  """K9 RJ cues seen from above: full-width connected tail light bar, slim LED headlights, chrome grille top."""
  _band(rgb, body, ppm, True, 0.0, 0.90, (0.008, 0.085), (215.0, 22.0, 36.0), (255.0, 96.0, 104.0))
  _band(rgb, body, ppm, False, 0.50, 0.88, (0.015, 0.075), (210.0, 226.0, 248.0), (255.0, 255.0, 255.0))
  _band(rgb, body, ppm, False, 0.0, 0.36, (0.012, 0.050), (178.0, 184.0, 192.0))


def render():
  v, f, mats = load(OBJ)
  ppm = PPM * SS
  w = int(round((L + 2 * PAD) * PPM)) * SS
  h = int(round((W + 2 * PAD) * PPM)) * SS
  col = (L / 2 + PAD - v[:, 1]) * ppm               # front (+y) on the left
  row = (W / 2 + PAD - v[:, 0]) * ppm               # car right (+x) up
  z = v[:, 2]
  zbuf = np.full((h, w), -1.0)
  rgb = np.zeros((h, w, 3))
  p0, p1, p2 = v[f[:, 0]], v[f[:, 1]], v[f[:, 2]]
  nrm = np.cross(p1 - p0, p2 - p0)
  nlen = np.linalg.norm(nrm, axis=1)
  nrm = nrm / np.maximum(nlen, 1e-12)[:, None]
  nrm[nrm[:, 2] < 0] *= -1
  cache = {}
  for t in range(len(f)):
    if nlen[t] < 1e-12:
      continue
    ia, ib, ic = f[t]
    xs = np.array((col[ia], col[ib], col[ic]))
    ys = np.array((row[ia], row[ib], row[ic]))
    zs = np.array((z[ia], z[ib], z[ic]))
    x0, x1 = max(int(np.floor(xs.min())), 0), min(int(np.ceil(xs.max())), w - 1)
    y0, y1 = max(int(np.floor(ys.min())), 0), min(int(np.ceil(ys.max())), h - 1)
    if x1 < x0 or y1 < y0:
      continue
    den = (ys[1] - ys[2]) * (xs[0] - xs[2]) + (xs[2] - xs[1]) * (ys[0] - ys[2])
    if abs(den) < 1e-9:
      continue
    gx, gy = np.meshgrid(np.arange(x0, x1 + 1) + 0.5, np.arange(y0, y1 + 1) + 0.5)
    l0 = ((ys[1] - ys[2]) * (gx - xs[2]) + (xs[2] - xs[1]) * (gy - ys[2])) / den
    l1 = ((ys[2] - ys[0]) * (gx - xs[2]) + (xs[0] - xs[2]) * (gy - ys[2])) / den
    l2 = 1 - l0 - l1
    inside = (l0 >= -1e-6) & (l1 >= -1e-6) & (l2 >= -1e-6)
    if not inside.any():
      continue
    zz = l0 * zs[0] + l1 * zs[1] + l2 * zs[2]
    sub = zbuf[y0:y1 + 1, x0:x1 + 1]
    m = inside & (zz > sub)
    if not m.any():
      continue
    key = (mats[t], tuple(np.round(nrm[t], 3)))
    if key not in cache:
      cache[key] = shade(mats[t], nrm[t])
    sub[m] = zz[m]
    rgb[y0:y1 + 1, x0:x1 + 1][m] = cache[key]
  body = zbuf >= 0
  edge_lights(rgb, body, ppm)
  body = body.astype(float)
  rgba = np.dstack((rgb * body[..., None], body))
  rgba = rgba.reshape(h // SS, SS, w // SS, SS, 4).mean(axis=(1, 3))
  a = rgba[..., 3:4]
  rgba[..., :3] = np.where(a > 0, rgba[..., :3] / np.maximum(a, 1e-6), 0)
  img = Image.fromarray(np.dstack((np.clip(rgba[..., :3], 0, 255), a[..., 0] * 255)).astype(np.uint8), "RGBA")
  mask = Image.fromarray((a[..., 0] * 255).astype(np.uint8)).filter(ImageFilter.MaxFilter(9))
  mask = mask.filter(ImageFilter.GaussianBlur(PPM * 0.06))
  shadow = Image.new("RGBA", img.size, (0, 0, 0, 0))
  shadow.putalpha(mask.point(lambda q: int(q * 0.80)))
  return Image.alpha_composite(shadow, img)


if __name__ == "__main__":
  out = Path(sys.argv[1]) if len(sys.argv) > 1 else OUT
  render().save(out, optimize=True)
  print(out)
