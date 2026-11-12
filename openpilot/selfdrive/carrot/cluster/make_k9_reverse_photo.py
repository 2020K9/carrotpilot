#!/usr/bin/env python3
"""K9 R screen car from 대감님's own K9 photo (11/13 02:35, k9_rear3.jpg: left side, rear 3/4, 1600 x 901).

Run once offline:  python make_k9_reverse_photo.py <k9_rear3.jpg>
Everything below works on the photo's own pixels (OpenCV only, no learned model, no other image):
- cut-out: GrabCut from a hand-drawn outline; largest piece kept, holes filled; the edge is pulled 2 px inside the
  car (no ground / sky fringe), smoothed (no stair steps) and feathered ~1.5 px; edge colours replaced by the colour
  just inside the car (despill: no grey / beige rim)
- studio grade like the P photo: paint deepened (gamma, contrast), outdoor colour cast and sky / building reflections
  pulled down and desaturated, navy tint; red lamps keep their colour; soft top light on the roof line and a thin navy
  rim light along the outline
- glass (side windows, rear window): dark tinted glass keeping only a faint, compressed copy of its own reflections
- number plate: plate and holder filled from the surrounding trunk paint (inpaint), then a shallow recess with a thin
  chrome edge in the photo's plate-holder shape (no plate text)
- floor (separate file, drawn under the path): the P stage look - dark navy pool and thin blue rim with glow - as an
  ellipse on the photo's ground plane (photo camera); car file: tyre contact shadows + a wide soft footprint shadow
Outputs: assets/images/k9_reverse_photo.png (car + shadow) and k9_reverse_photo_floor.png (floor), both RGBA, photo
px x OUT_SCALE, same frame = photo px box CROP (printed).
"""
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / "assets" / "images" / "k9_reverse_photo.png"
OUT_FLOOR = HERE / "assets" / "images" / "k9_reverse_photo_floor.png"
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
# glass, 1200 px preview: side windows inside the chrome frame, rear window
GLASS = [
    [(163, 216), (205, 152), (250, 128), (298, 118), (295, 140), (282, 222)],
    [(292, 222), (302, 120), (340, 116), (462, 118), (478, 140), (485, 228)],
    [(470, 146), (478, 136), (505, 152), (545, 200), (536, 218), (492, 226)],
    [(558, 108), (610, 98), (690, 100), (760, 118), (820, 140), (872, 162), (790, 186), (722, 210), (660, 160)],
]
PLATE = [(1259, 369), (1334, 330), (1338, 385), (1268, 431)]  # plate + holder, photo px (padded)
FLOOR_ELLIPSE = (1.40, 3.00)  # floor disc half sizes on the ground (m), centred under the car (y 2.56 m)


def project(points):
    k = np.array([[F_PX, 0, 800.0], [0, F_PX, 450.5], [0, 0, 1]])
    out, _ = cv2.projectPoints(np.array(points, float), np.array(RVEC), np.array(TVEC), k, None)
    return out.reshape(-1, 2)


def cut_out(im):
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
    # open: thin background slivers and loose bits off; smooth: blur + re-threshold (no stair steps); 2 px inside
    alpha = cv2.morphologyEx(alpha, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))
    alpha = np.where(cv2.GaussianBlur(alpha.astype(np.float32), (0, 0), 3.5) > 127, 255, 0).astype(np.uint8)
    alpha = cv2.erode(alpha, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(alpha)
    alpha = np.where(labels == 1 + np.argmax(stats[1:, 4]), 255, 0).astype(np.uint8)
    soft = cv2.GaussianBlur(alpha.astype(np.float32) / 255, (0, 0), 1.3)
    return alpha, np.clip((soft - 0.5) * 1.15 + 0.5, 0, 1) * (cv2.dilate(alpha, np.ones((7, 7), np.uint8)) > 0)


def grade(rgb, alpha):
    """rgb float 0..1 (R, G, B order)."""
    h, w = alpha.shape
    inside = alpha > 0
    # despill: the outer 6 px take the colour found just inside (no ground / sky colour on the rim)
    core = cv2.erode(alpha, np.ones((13, 13), np.uint8)).astype(np.float32) / 255
    near = np.dstack([cv2.GaussianBlur(rgb[..., c] * core, (0, 0), 5) for c in range(3)])
    near /= np.maximum(cv2.GaussianBlur(core, (0, 0), 5), 1e-4)[..., None]
    dist = cv2.distanceTransform(alpha, cv2.DIST_L2, 5)
    band = np.clip(1.0 - (dist - 1.0) / 5.0, 0, 1)[..., None] * (core.sum() > 0)
    rgb = rgb * (1 - band) + np.minimum(rgb, near) * band

    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    red_lamp = ((r > g + 0.16) & (r > b + 0.16)).astype(np.float32)
    red_lamp = cv2.GaussianBlur(cv2.dilate(red_lamp, np.ones((5, 5), np.uint8)), (0, 0), 2)[..., None]
    lum = (rgb * np.array([0.299, 0.587, 0.114], np.float32)).sum(axis=2, keepdims=True)
    # reflections: values above the knee squeezed, outdoor colour mostly out
    knee = 0.30
    lum2 = np.where(lum > knee, knee + (lum - knee) * 0.42, lum)
    chroma = (rgb - lum) * 0.35
    # deeper paint, more contrast: gamma on the squeezed luminance
    lum3 = np.power(np.clip(lum2, 0, 1), 1.45) * 1.22
    paint = np.clip(lum3 + chroma, 0, 1) * np.array([0.80, 0.90, 1.16], np.float32)  # navy studio tint
    lamp = np.clip(rgb * 0.92, 0, 1)
    out = paint * (1 - red_lamp) + lamp * red_lamp

    # glass: dark tint + a faint copy of its own reflections
    glass = np.zeros((h, w), np.float32)
    for poly in GLASS:
        cv2.fillPoly(glass, [(np.array(poly) * w / 1200.0).astype(np.int32)], 1.0)
    glass = cv2.GaussianBlur(glass, (0, 0), 1.5)[..., None] * inside[..., None]
    gl = np.power(np.clip(lum, 0, 1), 1.6) * 0.38
    tinted = np.clip(np.array([0.018, 0.026, 0.050], np.float32) + gl * np.array([0.75, 0.88, 1.20], np.float32), 0, 1)
    out = out * (1 - glass) + tinted * glass

    # top light: soft highlight a little below the top outline of each column
    top = np.full(w, h, np.int32)
    ys, xs = np.nonzero(alpha)
    np.minimum.at(top, xs, ys)
    yy = np.arange(h)[:, None] - top[None, :]
    light = np.where(yy >= 0, np.exp(-np.square((yy - 34.0) / 22.0)), 0.0).astype(np.float32)
    light = cv2.GaussianBlur(light, (0, 0), 6)[..., None] * (1 - glass) * (1 - red_lamp)
    out = out + light * 0.06 * np.array([0.80, 0.90, 1.0], np.float32)
    # thin navy rim light along the outline
    rim = np.exp(-np.maximum(dist - 1.5, 0) / 2.5)[..., None] * (1 - red_lamp)
    out = out + rim * 0.09 * np.array([0.28, 0.50, 1.0], np.float32)
    return np.clip(out, 0, 1)


def plate(out):
    """Plate + holder filled from the trunk paint around it, then a shallow recess with a thin chrome edge."""
    h, w = out.shape[:2]
    pts = np.array(PLATE, np.int32)
    m = np.zeros((h, w), np.uint8)
    cv2.fillPoly(m, [pts], 255)
    m = cv2.dilate(m, np.ones((5, 5), np.uint8))
    u8 = np.clip(out * 255 + 0.5, 0, 255).astype(np.uint8)
    filled = cv2.inpaint(u8, m, 9, cv2.INPAINT_TELEA).astype(np.float32) / 255
    filled = cv2.GaussianBlur(filled, (0, 0), 3) * (m[..., None] > 0) + out * (m[..., None] == 0)
    c = pts.mean(axis=0)
    inner = c + (pts - c) * 0.90
    rec = np.zeros((h, w), np.float32)
    cv2.fillPoly(rec, [np.round(inner * 4).astype(np.int32)], 1.0, cv2.LINE_AA, 2)
    rec = cv2.GaussianBlur(rec, (0, 0), 1.2)[..., None]
    filled = filled * (1 - 0.35 * rec)
    edge = np.zeros((h, w), np.float32)
    cv2.polylines(edge, [np.round(inner * 4).astype(np.int32)], True, 1.0, 2, cv2.LINE_AA, 2)
    edge = cv2.GaussianBlur(edge, (0, 0), 0.6)[..., None]
    return filled * (1 - 0.55 * edge) + np.array([0.42, 0.45, 0.52], np.float32) * 0.55 * edge


def main(src: str) -> None:
    im = cv2.imread(src)
    h, w = im.shape[:2]
    alpha, a = cut_out(im)
    out = plate(grade(im[..., ::-1].astype(np.float32) / 255, alpha))

    pad = 220
    cw, ch = w + 2 * pad, h + 2 * pad
    # shadows: wide soft footprint + dark tyre contacts
    shadow = np.zeros((ch, cw), np.float32)
    foot = project([(-0.96, 0.0, 0), (0.96, 0.0, 0), (0.96, 5.1, 0), (-0.96, 5.1, 0)]) + pad
    cv2.fillPoly(shadow, [foot.astype(np.int32)], 1.0)
    shadow = cv2.GaussianBlur(shadow, (0, 0), 26) * 0.62
    body = np.zeros_like(shadow)
    foot2 = project([(-0.88, 0.25, 0), (0.88, 0.25, 0), (0.88, 4.9, 0), (-0.88, 4.9, 0)]) + pad
    cv2.fillPoly(body, [foot2.astype(np.int32)], 1.0)
    shadow = np.maximum(shadow, cv2.GaussianBlur(body, (0, 0), 9) * 0.80)
    for x, y in ((-0.815, 1.06), (-0.815, 4.165), (0.815, 1.06)):
        cc = project([(x - 0.11, y, 0), (x + 0.11, y, 0), (x, y - 0.33, 0), (x, y + 0.33, 0)]) + pad
        spot = np.zeros_like(shadow)
        cv2.fillConvexPoly(spot, cc.astype(np.int32), 1.0)
        shadow = np.maximum(shadow, cv2.GaussianBlur(spot, (0, 0), 5) * 0.84)

    # floor: P stage look on the photo's ground plane
    t = np.linspace(0, 2 * np.pi, 361)
    ax, ay = FLOOR_ELLIPSE
    ring = (project([(ax * np.cos(v), 2.56 + ay * np.sin(v), 0) for v in t]) + pad) * 4
    ring = np.round(ring).astype(np.int32)
    disc = np.zeros((ch, cw), np.float32)
    cv2.fillPoly(disc, [ring], 1.0, cv2.LINE_AA, 2)
    pool = cv2.GaussianBlur(disc, (0, 0), 40)
    line = np.zeros((ch, cw), np.float32)
    cv2.polylines(line, [ring], True, 1.0, 2, cv2.LINE_AA, 2)
    glow = cv2.GaussianBlur(line, (0, 0), 9)
    glow = glow / max(glow.max(), 1e-4)
    line = cv2.GaussianBlur(line, (0, 0), 0.8)
    f_rgb = (pool[..., None] * np.array([0.10, 0.20, 0.48]) * 0.55 + glow[..., None] * np.array([0.20, 0.42, 1.0]) * 0.30
             + line[..., None] * np.array([0.47, 0.72, 1.0]) * 0.55)
    f_a = np.clip(pool * 0.55 + glow * 0.28 + line * 0.55, 0, 1)
    floor = np.dstack([np.clip(f_rgb / np.maximum(f_a, 1e-4)[..., None], 0, 1), f_a]).astype(np.float32)

    canvas = np.zeros((ch, cw, 4), np.float32)
    car_a = np.zeros((ch, cw), np.float32)
    car_a[pad:pad + h, pad:pad + w] = a
    canvas[pad:pad + h, pad:pad + w, :3] = out
    out_a = car_a + shadow * (1 - car_a)
    canvas[..., :3] *= (car_a / np.maximum(out_a, 1e-4))[..., None]  # shadow is black
    canvas[..., 3] = out_a
    ys, xs = np.nonzero((out_a > 2 / 255) | (f_a > 2 / 255))
    x0, y0, x1, y1 = xs.min() - 4, ys.min() - 4, xs.max() + 5, ys.max() + 5
    size = (round((x1 - x0) * OUT_SCALE), round((y1 - y0) * OUT_SCALE))
    for img, path in ((canvas, OUT), (floor, OUT_FLOOR)):
        crop = img[y0:y1, x0:x1].copy()
        crop[..., :3] *= crop[..., 3:]  # resize premultiplied (no dark / bright halo)
        small = cv2.resize(crop, size, interpolation=cv2.INTER_AREA)
        small[..., :3] /= np.maximum(small[..., 3:], 1e-4)
        rgba = np.clip(small * 255 + 0.5, 0, 255).astype(np.uint8)
        cv2.imwrite(str(path), cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGRA))
    print("CROP photo px", (x0 - pad, y0 - pad, x1 - x0, y1 - y0), "image", size[0], size[1])


if __name__ == "__main__":
    main(sys.argv[1])
