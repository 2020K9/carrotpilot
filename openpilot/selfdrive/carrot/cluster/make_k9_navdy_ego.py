#!/usr/bin/env python3
"""K9HudNavdy 1 ego car = the owner K9 rear render (assets/images/k9_ego_car.png, the K9HudNavdy 0 ego car) in the
Navdy grey look (대감님 11/13 05:46 "기본.고급판 둘다 k9으로넣어야지").

Run once offline:  python make_k9_navdy_ego.py
Only k9_ego_car.png and navdy/car_12.png (straight rear) pixels are used, no other image or model:
- top (front roof) rows cut so the visible box has the Navdy straight-rear aspect; scaled so the visible box sits
  where the car_12 visible box sits in a 2x car_12 frame -> drop-in for car_12 (same size / anchor on screen)
- grey: luminance only, car_12 median + TONE_LIFT (the grey cars ahead) and SPREAD x its spread, capped at its 95th percentile, so it is the same tone
- K9 tail lamp band keeps a muted red (K9 full-width lamp), everything else neutral grey
Output: assets/images/k9_navdy_ego.png (RGBA, 2 x car_12 size)
"""
from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
IMG = HERE / "assets" / "images"
SRC, REF, OUT = IMG / "k9_ego_car.png", IMG / "navdy" / "car_12.png", IMG / "k9_navdy_ego.png"
LAMP_SAT = 0.55
SPREAD = 0.6   # x car_12 brightness spread: softer than the photo, K9 lines (lamp band, chrome, glass) still read
TONE_LIFT = 15.0  # median this much above car_12: the big dark K9 glass / trunk otherwise reads darker than the Navdy car


def main() -> None:
  src = Image.open(SRC).convert("RGBA")
  ref = Image.open(REF).convert("RGBA")
  rb = ref.getchannel("A").point(lambda v: 255 if v > 20 else 0).getbbox()
  sb = src.getchannel("A").point(lambda v: 255 if v > 20 else 0).getbbox()
  rw, rh = rb[2] - rb[0], rb[3] - rb[1]
  sw = sb[2] - sb[0]
  sh = round(sw * rh / rw)
  crop = src.crop((sb[0], sb[3] - sh, sb[2], sb[3]))
  out_w, out_h = ref.width * 2, ref.height * 2
  crop = crop.resize((rw * 2, rh * 2), Image.LANCZOS)
  a = np.asarray(crop).astype(np.float64)
  r = np.asarray(ref).astype(np.float64)
  lum = a[..., :3] @ np.array([0.299, 0.587, 0.114])
  m, mr = a[..., 3] > 128, r[..., 3] > 128
  # black paint -> Navdy grey: same mean and spread as car_12 (a percentile match keeps the black trunk dark)
  rv = r[..., :3][mr].mean(1)
  grey = np.median(rv) + TONE_LIFT + (lum - np.median(lum[m])) * (rv.std() * SPREAD / lum[m].std())
  grey = np.minimum(grey, np.percentile(rv, 95))  # white plate / glints no brighter than the brightest car_12 grey
  rgb = np.repeat(grey[..., None], 3, axis=2)
  red = (a[..., 0] > 1.35 * a[..., 1]) & (a[..., 0] > 1.35 * a[..., 2]) & (a[..., 0] > 50)
  lamp = np.stack([grey * (1 + LAMP_SAT * 0.45), grey * (1 - LAMP_SAT * 0.5), grey * (1 - LAMP_SAT * 0.5)], axis=2)
  rgb[red] = lamp[red]
  out = np.zeros((out_h, out_w, 4))
  out[rb[1] * 2:rb[1] * 2 + rh * 2, rb[0] * 2:rb[0] * 2 + rw * 2, :3] = rgb
  out[rb[1] * 2:rb[1] * 2 + rh * 2, rb[0] * 2:rb[0] * 2 + rw * 2, 3] = a[..., 3]
  Image.fromarray(np.clip(out, 0, 255).astype(np.uint8)).save(OUT, optimize=True)
  print(OUT, out_w, out_h, "src crop", sb[0], sb[3] - sh, sb[2], sb[3], "lamp px", int(red.sum()))


if __name__ == "__main__":
  main()
