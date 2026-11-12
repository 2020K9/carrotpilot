#!/usr/bin/env python3
"""K9 R screen car from 대감님's own K9 photo (11/13 02:35, k9_rear3.jpg: left side, rear 3/4, 1600 x 901).

Run once offline:  python make_k9_reverse_photo.py <k9_rear3.jpg>
- car cut out with OpenCV GrabCut from a hand-drawn outline (no learned model); every other car, truck, person,
  building and the floor are removed; largest piece kept, holes filled, 1 px soft edge
- the number plate is painted over with a plain black plate (car paint colour, faint rim): no plate text is left
- bright reflections on the paint are toned down (value above KNEE compressed), colours kept; red lamps untouched
- a new soft floor shadow: the car's 1.92 x 5.12 m footprint projected with the photo camera, blurred, under the car
Output: assets/images/k9_reverse_photo.png (RGBA, photo px x OUT_SCALE), frame = photo px box CROP (printed).
"""
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / "assets" / "images" / "k9_reverse_photo.png"
OUT_SCALE = 0.5
# photo camera, reverse-solved from the photo (see cluster_renderer.K9_REVERSE_PHOTO_CAMERA)
F_PX, RVEC, TVEC = 1484.13, (1.68427, -0.74313, 0.56378), (1.08057, 0.72563, 3.23045)
# outline of the car in the 1200 px wide preview (x 4/3 = photo px)
OUTLINE = [(28, 250), (40, 226), (98, 212), (100, 195), (122, 188), (150, 190), (172, 150), (250, 112), (330, 96),
           (450, 88), (540, 93), (575, 96), (603, 93), (608, 80), (612, 74), (640, 73), (645, 88), (700, 100), (800, 140), (880, 160),
           (985, 180), (1003, 195), (1015, 222), (1030, 295), (1030, 345), (1012, 400), (985, 445), (945, 520),
           (915, 555), (870, 566), (820, 560), (640, 545), (612, 548), (596, 563), (578, 578), (530, 586),
           (470, 573), (440, 535), (402, 440), (300, 410), (200, 386), (130, 370), (115, 383), (80, 390),
           (55, 378), (33, 330)]
PLATE = [(1259, 369), (1334, 330), (1338, 385), (1268, 431)]  # plate + holder, photo px (padded)
KNEE, KEEP = 120.0, 0.45


def project(points):
    k = np.array([[F_PX, 0, 800.0], [0, F_PX, 450.5], [0, 0, 1]])
    out, _ = cv2.projectPoints(np.array(points, float), np.array(RVEC), np.array(TVEC), k, None)
    return out.reshape(-1, 2)


def main(src: str) -> None:
    im = cv2.imread(src)
    h, w = im.shape[:2]
    outline = (np.array(OUTLINE) * w / 1200.0).astype(np.int32)
    poly = np.zeros((h, w), np.uint8)
    cv2.fillPoly(poly, [outline], 1)
    mask = np.full((h, w), cv2.GC_BGD, np.uint8)
    mask[cv2.dilate(poly, np.ones((25, 25), np.uint8)) > 0] = cv2.GC_PR_BGD
    mask[poly > 0] = cv2.GC_PR_FGD
    mask[cv2.erode(poly, np.ones((41, 41), np.uint8)) > 0] = cv2.GC_FGD
    cv2.grabCut(im, mask, None, np.zeros((1, 65)), np.zeros((1, 65)), 6, cv2.GC_INIT_WITH_MASK)
    alpha = np.where((mask == cv2.GC_FGD) | (mask == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(alpha)
    alpha = np.where(labels == 1 + np.argmax(stats[1:, 4]), 255, 0).astype(np.uint8)
    flood = alpha.copy()
    cv2.floodFill(flood, np.zeros((h + 2, w + 2), np.uint8), (0, 0), 255)
    alpha |= ~flood

    rgb = im.astype(np.float32)
    v = rgb.max(axis=2)
    red_lamp = (rgb[..., 2] > rgb[..., 1] + 40) & (rgb[..., 2] > rgb[..., 0] + 40)
    scale = np.where((v > KNEE) & ~red_lamp, (KNEE + (v - KNEE) * KEEP) / np.maximum(v, 1), 1.0)
    rgb *= scale[..., None]
    plate = np.array(PLATE, np.int32)
    cv2.fillPoly(rgb, [plate], (22, 20, 19))
    cv2.polylines(rgb, [plate], True, (52, 50, 48), 2, cv2.LINE_AA)
    inner = np.round(plate.mean(axis=0) + (plate - plate.mean(axis=0)) * 0.82).astype(np.int32)
    cv2.polylines(rgb, [inner], True, (34, 32, 31), 1, cv2.LINE_AA)

    # soft floor shadow, in a canvas larger than the photo (the footprint reaches below the photo edge)
    pad = 120
    cw, ch = w + 2 * pad, h + 2 * pad
    shadow = np.zeros((ch, cw), np.float32)
    foot = project([(-0.93, 0.05, 0), (0.93, 0.05, 0), (0.93, 5.05, 0), (-0.93, 5.05, 0)]) + pad
    cv2.fillPoly(shadow, [foot.astype(np.int32)], 1.0)
    shadow = cv2.GaussianBlur(shadow, (0, 0), 22) * 0.5
    for x, y in ((-0.815, 1.06), (-0.815, 4.165)):  # darker contact under the visible tyres
        c = project([(x, y, 0)])[0] + pad
        spot = np.zeros_like(shadow)
        cv2.ellipse(spot, (int(c[0]), int(c[1])), (70, 16), 15, 0, 360, 1.0, -1)
        shadow = np.maximum(shadow, cv2.GaussianBlur(spot, (0, 0), 9) * 0.8)

    a = cv2.GaussianBlur(cv2.erode(alpha, np.ones((3, 3), np.uint8)), (0, 0), 1.0).astype(np.float32) / 255
    canvas = np.zeros((ch, cw, 4), np.float32)
    canvas[pad:pad + h, pad:pad + w, :3] = rgb[..., ::-1]
    car_a = np.zeros((ch, cw), np.float32)
    car_a[pad:pad + h, pad:pad + w] = a
    out_a = car_a + shadow * (1 - car_a)
    canvas[..., :3] = canvas[..., :3] * (car_a / np.maximum(out_a, 1e-4))[..., None]  # shadow is black
    canvas[..., 3] = out_a * 255
    ys, xs = np.nonzero(out_a > 2 / 255)
    x0, y0, x1, y1 = xs.min() - 4, ys.min() - 4, xs.max() + 5, ys.max() + 5
    crop = canvas[y0:y1, x0:x1]
    small = cv2.resize(crop, (round((x1 - x0) * OUT_SCALE), round((y1 - y0) * OUT_SCALE)), interpolation=cv2.INTER_AREA)
    cv2.imwrite(str(OUT), cv2.cvtColor(np.clip(small + 0.5, 0, 255).astype(np.uint8), cv2.COLOR_RGBA2BGRA))
    print("CROP photo px", (x0 - pad, y0 - pad, x1 - x0, y1 - y0), "image", small.shape[1], small.shape[0])


if __name__ == "__main__":
    main(sys.argv[1])
