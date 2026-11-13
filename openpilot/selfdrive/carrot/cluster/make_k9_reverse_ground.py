#!/usr/bin/env python3
"""K9 R screen ground layer (대감님 11/13 15:46 "r단 개선 안하냐"): makes the photo car sit on the vector floor.

Reads only the existing k9_reverse_photo.png (the photo car, unchanged) and writes k9_reverse_photo_ground.png in the
same frame, drawn over the floor disc and under the path / car:
- floor reflection: the photo's own pixels mirrored per column about the bottom of the silhouette, faded and tinted
  to the navy floor (a glossy floor look; nothing new is drawn on the car)
- key light: the photo is lit from the car's left / front-left, so the floor on that side gets a faint cool wash
  and the far right side a slight darkening (both only on the floor disc), so floor and car agree on the light direction
Run once offline:  python make_k9_reverse_ground.py
"""
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
FLOOR = HERE / "assets" / "images" / "k9_reverse_photo_floor.png"
SRC = HERE / "assets" / "images" / "k9_reverse_photo.png"
OUT = HERE / "assets" / "images" / "k9_reverse_photo_ground.png"
NAVY = np.array([0.09, 0.16, 0.34], np.float32)  # RGB, the floor pool colour
REFLECT_A = 0.0      # 대감님 11/13 16:08 "반사광좀죽이라고": floor reflection off (light direction kept)
REFLECT_FADE = 50.0  # px (this file) for the reflection to fall to 1/e
BODY_A = 0.95        # alpha above this = car body (the baked shadow tops out at 0.84)


def main() -> None:
    bgra = cv2.imread(str(SRC), cv2.IMREAD_UNCHANGED).astype(np.float32) / 255.0
    rgb, a = bgra[..., 2::-1], bgra[..., 3]
    h, w = a.shape
    body = a > BODY_A
    out_rgb = np.zeros((h, w, 3), np.float32)
    out_a = np.zeros((h, w), np.float32)
    for x in range(w if REFLECT_A > 0 else 0):
        ys = np.nonzero(body[:, x])[0]
        if ys.size == 0:
            continue
        yb = ys.max()
        d = np.arange(1, h - yb)
        src = yb - d
        ok = (src >= 0) & body[np.clip(src, 0, h - 1), x]
        d, src = d[ok], src[ok]
        out_rgb[yb + d, x] = rgb[src, x]
        out_a[yb + d, x] = REFLECT_A * np.exp(-d / REFLECT_FADE)
    out_a = cv2.GaussianBlur(out_a, (0, 0), 1.6)
    out_rgb = cv2.GaussianBlur(out_rgb, (0, 0), 1.6)
    out_rgb = out_rgb * 0.55 + NAVY * 0.45  # pulled to the floor colour

    # key light from the car's left (screen left) and a little dark on the far right
    pool = cv2.imread(str(FLOOR), cv2.IMREAD_UNCHANGED)[..., 3].astype(np.float32) / 255.0  # light stays on the disc
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    lit = np.exp(-(((xx - 0.22 * w) / (0.30 * w)) ** 2 + ((yy - 0.80 * h) / (0.14 * h)) ** 2)) * 0.16 * pool
    dim = np.exp(-(((xx - 0.86 * w) / (0.20 * w)) ** 2 + ((yy - 0.78 * h) / (0.16 * h)) ** 2)) * 0.22 * pool
    light_rgb = np.array([0.55, 0.68, 0.95], np.float32)
    pm = out_rgb * out_a[..., None]  # premultiplied over: reflection, then light, then dark
    pa = out_a.copy()
    for col, al in ((light_rgb, lit), (np.zeros(3, np.float32), dim)):
        pm = col * al[..., None] + pm * (1 - al[..., None])
        pa = al + pa * (1 - al)
    rgb_o = np.clip(pm / np.maximum(pa, 1e-4)[..., None], 0, 1)
    img = np.dstack([rgb_o[..., ::-1], pa])
    cv2.imwrite(str(OUT), np.clip(img * 255 + 0.5, 0, 255).astype(np.uint8))


if __name__ == "__main__":
    main()
