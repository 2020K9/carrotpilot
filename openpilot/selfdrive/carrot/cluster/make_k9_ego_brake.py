#!/usr/bin/env python3
"""Brake-light glow overlays for the HUD ego car (대감님 11/23 19:47 "내 차 브레이크등 쪽 이미지를 빨갛게 들어오게 해봐").

Run once offline:  python make_k9_ego_brake.py
For each ego sprite (k9_ego_car.png = K9HudNavdy 0 / 3D scene ego, k9_navdy_ego.png = K9HudNavdy 1 ego) the red
tail-lamp pixels of the photo itself are found (no other image), the biggest red patch on each side is kept
(the lamp clusters, not the small bumper reflectors) plus the high-mount stop lamp line in the rear window, and a same-size RGBA overlay is written:
- lamp core: bright red where the photo's lamp is
- soft glow: the core blurred a little around it
The renderer draws it with additive blending on the same rectangle only while carState.brakeLights is on;
with the brake off nothing extra is drawn (the sprite pixels stay exactly as they were).
Output: assets/images/k9_ego_car_brake.png, assets/images/k9_navdy_ego_brake.png
"""
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter

HERE = Path(__file__).resolve().parent
IMG = HERE / "assets" / "images"
PAIRS = (("k9_ego_car.png", "k9_ego_car_brake.png"), ("k9_navdy_ego.png", "k9_navdy_ego_brake.png"))
CORE = np.array((255.0, 46.0, 30.0))
GLOW = np.array((255.0, 30.0, 18.0))


def dilate(m: np.ndarray, n: int = 1) -> np.ndarray:
  for _ in range(n):
    p = np.pad(m, 1)
    m = p[1:-1, 1:-1] | p[:-2, 1:-1] | p[2:, 1:-1] | p[1:-1, :-2] | p[1:-1, 2:]
  return m


def erode(m: np.ndarray, n: int = 1) -> np.ndarray:
  return ~dilate(~m, n)


def biggest_patch(m: np.ndarray) -> np.ndarray:
  """4-connected biggest component (plain flood fill)"""
  seen = np.zeros_like(m)
  best: list[tuple[int, int]] = []
  h, w = m.shape
  for y0, x0 in zip(*np.nonzero(m), strict=True):
    if seen[y0, x0]:
      continue
    stack, comp = [(y0, x0)], []
    seen[y0, x0] = True
    while stack:
      y, x = stack.pop()
      comp.append((y, x))
      for yy, xx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
        if 0 <= yy < h and 0 <= xx < w and m[yy, xx] and not seen[yy, xx]:
          seen[yy, xx] = True
          stack.append((yy, xx))
    if len(comp) > len(best):
      best = comp
  out = np.zeros_like(m)
  for y, x in best:
    out[y, x] = True
  return out


def lamp_mask(a: np.ndarray) -> np.ndarray:
  r, g, b, al = (a[..., i].astype(np.float64) for i in range(4))
  red = (al > 128) & (r >= 60) & (r > 1.6 * g) & (r > 1.5 * b)
  w = a.shape[1]
  keep = np.zeros_like(red)
  for side in (slice(0, w // 2), slice(w // 2, w)):
    part = np.zeros_like(red)
    part[:, side] = red[:, side]
    keep |= biggest_patch(erode(dilate(part, 2), 2)) & (a[..., 3] > 128)
  # high-mount stop lamp: the thin red line in the rear window middle (dimmer in the photo)
  r, g, b, al = (a[..., i].astype(np.float64) for i in range(4))
  mid = (al > 128) & (r >= 60) & (r > g + 10) & (r > 1.18 * g)  # seen through the blue-tinted glass
  mid[:, : int(w * 0.35)] = False
  mid[:, int(w * 0.65):] = False
  mid[int(a.shape[0] * 0.6):, :] = False
  if mid.any():
    core = biggest_patch(erode(dilate(mid, 2), 2))
    ys = np.nonzero(core.any(axis=1))[0]
    band = np.zeros_like(mid)
    band[ys.min():ys.max() + 1, :] = True
    xs = np.nonzero((mid & band).any(axis=0))[0]  # the whole lamp line: all its red bits on those rows, end to end
    line = band.copy()
    line[:, : xs.min()] = False
    line[:, xs.max() + 1:] = False
    keep |= line & (al > 128)
  return keep


def make(src_name: str, out_name: str) -> None:
  a = np.asarray(Image.open(IMG / src_name).convert("RGBA"))
  core = dilate(lamp_mask(a), 1) & (a[..., 3] > 64)
  w = a.shape[1]
  core_a = Image.fromarray((core * 255).astype(np.uint8))
  glow_a = np.asarray(core_a.filter(ImageFilter.GaussianBlur(max(2.0, w * 0.022)))).astype(np.float64) / 255.0
  glow_a = np.clip(glow_a * 1.6, 0.0, 1.0) * 0.55
  ca = core.astype(np.float64) * 0.9
  alpha = np.maximum(ca, glow_a)
  t = np.where(alpha > 0, ca / np.maximum(alpha, 1e-6), 0.0)[..., None]
  rgb = CORE * t + GLOW * (1.0 - t)
  out = np.zeros(a.shape, dtype=np.float64)
  out[..., :3] = rgb
  out[..., 3] = alpha * 255.0
  Image.fromarray(np.clip(np.round(out), 0, 255).astype(np.uint8), "RGBA").save(IMG / out_name, optimize=True)
  print(IMG / out_name, a.shape[1], a.shape[0], "lamp px", int(core.sum()))


def main() -> None:
  for s, o in PAIRS:
    make(s, o)


if __name__ == "__main__":
  main()
