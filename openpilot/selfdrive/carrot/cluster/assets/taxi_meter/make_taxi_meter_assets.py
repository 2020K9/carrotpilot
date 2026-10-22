#!/usr/bin/env python3
"""Bake the K9ExtHud=1 taxi meter design into PNG layers (offline tool).

Input: the approved HTML/SVG design (taxi_meter_design.html, 보노보노 모범택시
bono_anim 시안). Output, next to this file:
  taxi_meter_bg.png        776x456 card: gradient, gold frame, moon, track base, goal shell
  taxi_meter_sea.png       776x76  sea waves (drawn over the characters, like the SVG)
  taxi_meter_porori.png    포로리 sprite (scale 0.86 of the SVG)
  taxi_meter_bonobono.png  보노보노 sprite (scale 0.82, clipped at the sea line)
Text, numbers, stars, progress fill and character motion are drawn live by
cluster_taxi_meter.py. To change the design, edit the HTML (or replace the PNGs)
and re-run:  python make_taxi_meter_assets.py   (needs `pip install cairosvg`).
"""
from __future__ import annotations

import math
import re
import sys
from pathlib import Path

import cairosvg

HERE = Path(__file__).resolve().parent
CARD_W, CARD_H = 776, 456


def _extract(html: str, start: str, end: str) -> str:
  i = html.index(start)
  j = html.index(end, i)
  return html[i:j]


def _wave(base: float, amp: float, length: float, ph: float) -> str:
  d = f"M0 456 L0 {base}"
  for x in range(0, 777, 8):
    y = base + amp * math.sin(x / length * 2 * math.pi + ph) + amp * 0.35 * math.sin(x / (length * 0.47) * 2 * math.pi - ph * 1.7)
    d += f" L{x} {y:.2f}"
  return d + " L776 456 Z"


def _svg(view_box: str, width: int, height: int, body: str, defs: str = "") -> bytes:
  return (f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" '
          f'width="{width}" height="{height}" viewBox="{view_box}"><defs>{defs}</defs>{body}</svg>').encode()


def main() -> int:
  html = (HERE / "taxi_meter_design.html").read_text(encoding="utf-8")
  defs = _extract(html, "<defs>", "</defs>")[len("<defs>"):]
  defs = defs.replace('href="#', 'xlink:href="#')
  shell = '<use xlink:href="#shellG"/>'
  poro = _extract(html, '<g id="poro">', "<!-- Bonobono -->").replace('href="#', 'xlink:href="#')
  bono = _extract(html, '<g id="bono">', "</g></g>\n<!-- sea -->").replace('href="#', 'xlink:href="#') + "</g>"

  card_defs = defs + (
    '<linearGradient id="cardg" x1="0" y1="0" x2="0" y2="1">'
    '<stop offset="0" stop-color="#07080d"/><stop offset=".55" stop-color="#0a0d18"/><stop offset="1" stop-color="#0b1426"/>'
    '</linearGradient><clipPath id="card"><rect x="0" y="0" width="776" height="456" rx="18"/></clipPath>'
  )
  bg = (
    '<g clip-path="url(#card)">'
    '<rect x="0" y="0" width="776" height="456" fill="url(#cardg)"/>'
    '<circle cx="505" cy="62" r="34" fill="url(#moon)" opacity=".9"/>'
    '<line x1="28" y1="326" x2="500" y2="326" stroke="#2b2618" stroke-width="3"/>'
    f'<g transform="translate(516 318)">{shell}</g>'
    '<rect x="4.5" y="4.5" width="767" height="447" rx="14" fill="none" stroke="#14110b" stroke-width="3"/>'
    '<rect x="5" y="5" width="766" height="446" rx="13.5" fill="none" stroke="#3a311c" stroke-width="1"/>'
    '</g>'
    '<rect x=".5" y=".5" width="775" height="455" rx="18" fill="none" stroke="#6d5a2e" stroke-width="1"/>'
  )
  cairosvg.svg2png(bytestring=_svg(f"0 0 {CARD_W} {CARD_H}", CARD_W, CARD_H, bg, card_defs), write_to=str(HERE / "taxi_meter_bg.png"))

  sea_top = 380
  sea = (
    f'<clipPath id="sc"><rect x="0" y="0" width="776" height="456" rx="18"/></clipPath><g clip-path="url(#sc)">'
    f'<path d="{_wave(398, 4, 260, 0.0)}" fill="#13305a" opacity=".95"/>'
    f'<path d="{_wave(406, 5, 190, 1.0)}" fill="url(#sea)"/>'
    f'<path d="{_wave(428, 4, 150, 2.0)}" fill="#0d2246" opacity=".9"/></g>'
  )
  cairosvg.svg2png(bytestring=_svg(f"0 {sea_top} {CARD_W} {CARD_H - sea_top}", CARD_W, CARD_H - sea_top, sea, defs),
                   write_to=str(HERE / "taxi_meter_sea.png"))

  # 포로리: local box x -86..38, y -80..56 (4 px padding), drawn at scale 0.86 (anchor = local 0,0)
  ps = 0.86
  cairosvg.svg2png(bytestring=_svg("-86 -80 124 136", round(124 * ps), round(136 * ps), poro, defs),
                   write_to=str(HERE / "taxi_meter_porori.png"))
  # 보노보노: local box x -66..66, y -80..120 (padding; the SVG clips it at the sea line), scale 0.82
  bs = 0.82
  cairosvg.svg2png(bytestring=_svg("-66 -80 132 200", round(132 * bs), round(200 * bs), bono, defs),
                   write_to=str(HERE / "taxi_meter_bonobono.png"))
  for name in ("bg", "sea", "porori", "bonobono"):
    print("wrote", HERE / f"taxi_meter_{name}.png")
  return 0


if __name__ == "__main__":
  sys.exit(main())
