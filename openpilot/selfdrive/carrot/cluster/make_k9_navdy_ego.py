#!/usr/bin/env python3
"""K9HudNavdy 1 ego car = the owner K9 rear render (assets/images/k9_ego_car.png, the K9HudNavdy 0 ego car) in its
own colours, same photo as menu 0 (대감님 11/13 05:46 "기본.고급판 둘다 k9으로넣어야지", 06:17 "왜 내차를 나브디로만듬?").

Run once offline:  python make_k9_navdy_ego.py
Only k9_ego_car.png and navdy/car_12.png (straight rear) pixels are used, no other image or model:
- top (front roof) rows cut so the visible box has the Navdy straight-rear aspect; scaled so the visible box sits
  where the car_12 visible box sits in a 2x car_12 frame -> drop-in for car_12 (same size / anchor on screen)
- colours: k9_ego_car.png pixels as they are (no grey / Navdy tone; 06:17), only resampled
Output: assets/images/k9_navdy_ego.png (RGBA, 2 x car_12 size)
"""
from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
IMG = HERE / "assets" / "images"
SRC, REF, OUT = IMG / "k9_ego_car.png", IMG / "navdy" / "car_12.png", IMG / "k9_navdy_ego.png"


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
  rgb = a[..., :3]  # 06:17: the menu-0 K9 photo colours, unchanged
  out = np.zeros((out_h, out_w, 4))
  out[rb[1] * 2:rb[1] * 2 + rh * 2, rb[0] * 2:rb[0] * 2 + rw * 2, :3] = rgb
  out[rb[1] * 2:rb[1] * 2 + rh * 2, rb[0] * 2:rb[0] * 2 + rw * 2, 3] = a[..., 3]
  Image.fromarray(np.clip(out, 0, 255).astype(np.uint8)).save(OUT, optimize=True)
  print(OUT, out_w, out_h, "src crop", sb[0], sb[3] - sh, sb[2], sb[3])


if __name__ == "__main__":
  main()
