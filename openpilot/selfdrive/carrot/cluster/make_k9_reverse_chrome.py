#!/usr/bin/env python3
"""K9 R screen photo car: roof back + window chrome from the photo (대감님 11/13 16:38 "루프는 어디갔냐", 16:43 "원본 가져와서 다시 봐봐").

Photo pixels only (k9_rear3.jpg, same frame as k9_reverse_photo.png: asset px = photo px / 2 + SHIFT):
- roof: the hand outline of make_k9_reverse_photo cut the roof above the rear window (photo x 500..775, up to ~40 photo
  px = ~20 asset px of roof missing); that strip is cut again from the photo (graded like the rest) and added to the car
- then the same glare removal as make_k9_reverse_deglare (wide sun / sky sheen on trunk, roof, glass, bumper stays dead)
- window chrome (frame round the side windows / C pillar triangle, belt strip) and a thin roof outline take the photo
  pixels back (own brightness and colour); tail lamps, lower chrome, wheels, empty plate untouched (no plate text)
Since 11/13 17:03 ("루프만") only the roof is moved onto the deglared asset; window chrome stays deglared.
Run:  python make_k9_reverse_chrome.py <k9_rear3.jpg>
"""
import sys

import cv2
import numpy as np

import make_k9_reverse_deglare as dg
import make_k9_reverse_photo as mk

OUT = dg.OUT
SHIFT = (39, -45)  # asset px = photo px * 0.5 + SHIFT (fitted on the cut-out alpha, 0.16 % mismatch)
ROOF = [(440, 131), (480, 126), (520, 117), (560, 110), (600, 106),
        (650, 102), (700, 101), (740, 102), (780, 104), (820, 106), (850, 108), (850, 210), (440, 210)]
CHROME_LUM = 0.50
CHROME_SAT = 0.30


def to_asset(img: np.ndarray, size) -> np.ndarray:
    half = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA)
    m = np.float32([[1, 0, SHIFT[0]], [0, 1, SHIFT[1]]])
    return cv2.warpAffine(half, m, size, flags=cv2.INTER_LINEAR, borderValue=0)


def main(src: str) -> None:
    im = cv2.imread(src)
    h, w = im.shape[:2]
    alpha, _ = mk.cut_out(im)
    roof = np.zeros((h, w), np.uint8)
    cv2.fillPoly(roof, [np.array(ROOF, np.int32)], 255)
    alpha2 = alpha | roof
    alpha2 = np.where(cv2.GaussianBlur(alpha2.astype(np.float32), (0, 0), 2.0) > 127, 255, 0).astype(np.uint8)
    soft = cv2.GaussianBlur(alpha2.astype(np.float32) / 255, (0, 0), 1.3)
    a2 = (np.clip((soft - 0.5) * 1.15 + 0.5, 0, 1) * (cv2.dilate(alpha2, np.ones((7, 7), np.uint8)) > 0)).astype(np.float32)
    graded = mk.plate(mk.grade(im[..., ::-1].astype(np.float32) / 255, alpha2))[..., ::-1].astype(np.float32)  # BGR

    ref = dg.load_ref().astype(np.float32) / 255.0
    H, W = ref.shape[:2]
    roof_zone = cv2.dilate(roof, np.ones((9, 9), np.uint8)) > 0
    new_a = cv2.GaussianBlur(to_asset(a2 * roof_zone, (W, H)), (0, 0), 1.0)
    new_a = np.clip((new_a - 0.5) * 1.6 + 0.5, 0, 1) * (new_a > 0.02)
    new_rgb = to_asset(graded * a2[..., None], (W, H)) / np.maximum(to_asset(a2, (W, H)), 1e-4)[..., None]
    add = np.clip(new_a - ref[..., 3], 0, 1)
    out_a = np.maximum(ref[..., 3], new_a)
    rgb = (ref[..., :3] * ref[..., 3:] + np.clip(new_rgb, 0, 1) * add[..., None]) / np.maximum(out_a, 1e-4)[..., None]
    base = dg.deglare(np.dstack([np.clip(rgb, 0, 1), out_a]).astype(np.float32))
    # the added roof strip is all sky sheen in the photo: same black paint level as the deglared roof next to it
    blum = base[..., :3] @ np.array([0.114, 0.587, 0.299], np.float32)
    dark = np.clip(dg.BLACK * 1.2 / np.maximum(blum, 1e-4), 0, 1)
    wa = cv2.GaussianBlur((add > 0.02).astype(np.float32), (0, 0), 1.5)[..., None]
    base[..., :3] *= 1 - wa + wa * dark[..., None]

    # window chrome + roof outline from the photo itself
    orig = to_asset(im.astype(np.float32) / 255, (W, H))
    lum = orig @ np.array([0.114, 0.587, 0.299], np.float32)
    sat = (orig.max(2) - orig.min(2)) / np.maximum(orig.max(2), 1e-4)
    glass = np.zeros((h, w), np.float32)
    for poly in mk.GLASS[:3]:  # side windows (the chrome frame runs round them), not the rear window
        cv2.fillPoly(glass, [(np.array(poly) * w / 1200.0).astype(np.int32)], 1.0)
    glass = (to_asset(glass, (W, H)) > 0.5).astype(np.uint8)
    band = cv2.dilate(glass, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21))) > 0
    band &= ~(cv2.erode(glass, np.ones((5, 5), np.uint8)) > 0)
    solid = base[..., 3] > 0.95
    chrome = band & (lum > CHROME_LUM) & (sat < CHROME_SAT) & solid
    chrome = (cv2.dilate(chrome.astype(np.uint8), np.ones((2, 2), np.uint8)) > 0) & solid
    # roof outline: top 2..3 px of the body along the roof (asset x 115..470)
    body = (base[..., 3] > 0.5).astype(np.uint8)
    rim = (body - cv2.erode(body, np.ones((5, 5), np.uint8))) > 0
    ys, xs = np.nonzero(body)
    first = np.full(W, H)
    np.minimum.at(first, xs, ys)
    xx = np.arange(W)[None, :]
    top = rim & (np.arange(H)[:, None] <= first[None, :] + 3) & (xx >= 115) & (xx <= 470)
    keep = np.maximum(chrome.astype(np.float32), top.astype(np.float32) * 0.85)
    keep = cv2.GaussianBlur(keep, (0, 0), 0.6)[..., None]
    res = base[..., :3] * (1 - keep) + orig * keep
    img = np.dstack([res, base[..., 3]])
    # 11/13 17:03 "아래꺼에서 루프만 따서 위로옮겨": only the roof (added strip + roof outline) goes onto the
    # deglared asset (fc95ab367 = dg.deglare(load_ref())); window chrome and everything else stay as before
    old = dg.deglare(ref.copy())
    chrome_zone = cv2.GaussianBlur(chrome.astype(np.float32), (0, 0), 0.6) > 1e-3
    roof_zone = cv2.dilate(((add > 0) | top).astype(np.uint8), np.ones((9, 9), np.uint8)) > 0
    roof_zone &= ~chrome_zone
    img = np.where(roof_zone[..., None], img, old)
    cv2.imwrite(str(OUT), np.clip(img * 255 + 0.5, 0, 255).astype(np.uint8))
    print("roof zone px", int(roof_zone.sum()), "roof px added", int((add > 0.5).sum()), "chrome px", int(chrome.sum()), "outline px", int(top.sum()))


if __name__ == "__main__":
    main(sys.argv[1])
