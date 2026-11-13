#!/usr/bin/env python3
"""K9 R screen photo car: kill the sun / sky glare on the body (대감님 11/13 16:08 "반사광좀죽이라고").

Pixel brightness / saturation correction only on k9_reverse_photo.png (shape, size, frame untouched):
- glare = the smooth (low-frequency) brightness of the body above a black-paint level; it is subtracted, the fine
  detail (panel lines, chrome strips) is kept, and the result is pulled toward neutral so grey/white sheen turns black
- kept as is: tail lamps (red hue), wheels (two fixed discs), thin chrome strips (high-frequency bright lines)
- the baked ground shadow (alpha <= BODY_A) is not touched
Reads the git original (REF) so re-running gives the same file:  python make_k9_reverse_deglare.py
"""
import subprocess
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / "assets" / "images" / "k9_reverse_photo.png"
REF = "83aa2d307"  # last commit with the un-corrected photo
BODY_A = 0.95
BLACK = 0.07        # target low-frequency level of black paint (0..1)
SIGMA = 9.0         # px, glare scale
KEEP_DETAIL = 0.55  # share of fine detail kept on glare areas
WHEELS = ((380, 272, 66), (90, 178, 36))  # (cx, cy, r) in this file px


def load_ref() -> np.ndarray:
    rel = OUT.relative_to(Path(subprocess.check_output(["git", "rev-parse", "--show-toplevel"], cwd=HERE, text=True).strip()))
    raw = subprocess.check_output(["git", "show", f"{REF}:{rel.as_posix()}"], cwd=HERE)
    return cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_UNCHANGED)


def keep_mask(bgr: np.ndarray, a: np.ndarray) -> np.ndarray:
    hsv = cv2.cvtColor((bgr * 255).astype(np.uint8), cv2.COLOR_BGR2HSV).astype(np.float32)
    hue, sat = hsv[..., 0], hsv[..., 1] / 255.0
    red = ((hue < 12) | (hue > 165)) & (sat > 0.35)
    red = cv2.dilate(red.astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
    h, w = a.shape
    yy, xx = np.mgrid[0:h, 0:w]
    wheel = np.zeros((h, w), bool)
    for cx, cy, r in WHEELS:
        wheel |= (xx - cx) ** 2 + (yy - cy) ** 2 < r * r
    return red | wheel


def deglare(bgra: np.ndarray) -> np.ndarray:
    """bgra float 0..1 -> deglared bgra float 0..1."""
    bgr, a = bgra[..., :3], bgra[..., 3]
    body = (a > BODY_A).astype(np.float32)
    lum = bgr @ np.array([0.114, 0.587, 0.299], np.float32)
    k = (0, 0)
    low = cv2.GaussianBlur(lum * body, k, SIGMA) / np.maximum(cv2.GaussianBlur(body, k, SIGMA), 1e-4)
    detail = lum - low
    glare = np.clip(low - BLACK, 0, None)
    new_lum = np.clip(np.minimum(low, BLACK) + KEEP_DETAIL * detail + (1 - KEEP_DETAIL) * np.minimum(detail, 0), 0, 1)
    new_lum = np.where(detail > 0.12, lum - glare * 0.5, new_lum)  # thin chrome strips stay bright
    gain = np.clip(new_lum / np.maximum(lum, 1e-4), 0, 1)
    grey = lum[..., None]
    desat = np.clip(glare / 0.25, 0, 1)[..., None] * 0.6
    out = (bgr * (1 - desat) + grey * desat) * gain[..., None]
    w = body * (~keep_mask(bgr, a))
    w = cv2.GaussianBlur(w, (0, 0), 1.5) * body
    res = bgr * (1 - w[..., None]) + out * w[..., None]
    return np.dstack([res, a])


def main() -> None:
    img = deglare(load_ref().astype(np.float32) / 255.0)
    cv2.imwrite(str(OUT), np.clip(img * 255 + 0.5, 0, 255).astype(np.uint8))


if __name__ == "__main__":
    main()
