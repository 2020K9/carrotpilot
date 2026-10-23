"""Offline (WORKER): over_{season}_{tod}.png = front_{season}_{tod}.png (sea in front of the characters) with
ui_{tod}.png (frame, brand, stats glass, fare footer) on top -> one layer drawn after the characters (2 full-card
layers per frame instead of 3). usage: make_c1_over.py <c1 asset dir>"""
import sys
from pathlib import Path

from PIL import Image

d = Path(sys.argv[1])
for f in sorted(d.glob("front_*_*.png")):
  _, season, tod = f.stem.split("_")
  over = Image.open(f).convert("RGBA")
  over.alpha_composite(Image.open(d / f"ui_{tod}.png").convert("RGBA"))
  over.save(d / f"over_{season}_{tod}.png", optimize=True)
  print("over", season, tod)
