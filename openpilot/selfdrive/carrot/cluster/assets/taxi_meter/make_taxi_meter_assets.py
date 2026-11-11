#!/usr/bin/env python3
"""Bake the K9ExtHud=1 taxi meter design into PNG layers (offline tool).

Input: the approved HTML/SVG design (taxi_meter_design.html, 보노보노 모범택시
bono_anim 시안). Output, next to this file:
  taxi_meter_bg.png            776x456 card: gradient, gold frame, moon, track base, goal shell
  taxi_meter_porori.png        포로리 body + head (scale 0.86 of the SVG; no tail, feet, arms)
  taxi_meter_porori_tail.png   포로리 tail (rotates around local -10,14)
  taxi_meter_bonobono.png      보노보노 without the 조개 arm (scale 0.82; the sea covers the bottom)
  taxi_meter_bonobono_arm.png  보노보노 arm + 조개 (rotates around local 40,80 when tapping)
Text, numbers, stars, progress fill, 포로리 feet/arms, closed eyes, the sea waves and all
motion are drawn live by cluster_taxi_meter.py (speed-linked phase). To change the design, edit the HTML (or replace the PNGs)
and re-run:  python make_taxi_meter_assets.py   (needs `pip install cairosvg`).
"""
from __future__ import annotations

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
  # split the characters into the parts cluster_taxi_meter.py moves
  tail = _extract(poro, '<g id="ptail">', "\n <ellipse")
  poro_body = poro.replace(tail, "")
  poro_body = re.sub(r'<ellipse [^>]*id="pf[LR]"/>', "", poro_body)
  poro_body = re.sub(r'<path [^>]*id="pa[LR]"/>', "", poro_body)
  arm = _extract(bono, '<g id="barm">', "\n <path")
  bono_body = bono.replace(arm, "")
  for part in ('id="pfL"', 'id="pfR"', 'id="paL"', 'id="paR"', 'id="ptail"'):
    assert part not in poro_body, part
  assert 'id="barm"' not in bono_body and 'id="barm"' in arm

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

  # 포로리 (scale 0.86, anchor = local 0,0): body box x -86..38, y -80..56; tail box x -82..-4, y -66..34
  ps = 0.86
  cairosvg.svg2png(bytestring=_svg("-86 -80 124 136", round(124 * ps), round(136 * ps), poro_body, defs),
                   write_to=str(HERE / "taxi_meter_porori.png"))
  cairosvg.svg2png(bytestring=_svg("-82 -66 78 100", round(78 * ps), round(100 * ps), tail, defs),
                   write_to=str(HERE / "taxi_meter_porori_tail.png"))
  # 보노보노 (scale 0.82): body box x -66..66, y -80..140 (the live sea covers it below ~y 113);
  # arm + 조개 box x 4..70, y 60..124
  bs = 0.82
  cairosvg.svg2png(bytestring=_svg("-66 -80 132 220", round(132 * bs), round(220 * bs), bono_body, defs),
                   write_to=str(HERE / "taxi_meter_bonobono.png"))
  cairosvg.svg2png(bytestring=_svg("4 60 66 64", round(66 * bs), round(64 * bs), arm, defs),
                   write_to=str(HERE / "taxi_meter_bonobono_arm.png"))
  for name in ("bg", "porori", "porori_tail", "bonobono", "bonobono_arm"):
    print("wrote", HERE / f"taxi_meter_{name}.png")
  return 0


if __name__ == "__main__":
  sys.exit(main())
