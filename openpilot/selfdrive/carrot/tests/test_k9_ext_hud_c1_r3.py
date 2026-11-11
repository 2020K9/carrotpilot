"""K9ExtHud=1 C1 meter, r3 (14:32 지시 1·2), no GL.

1. Particles fade to 0 near the card edge (PTCL_EDGE_FADE = 24 px) and near the no-particle boxes; no pop-in.
2. The bg2 HTML animations (c1bg.js draw(): first 14 data-tw = twinkle, every data-dr = drift) are drawn live from
   anim_{season}.png + anim.json with the c1bg.js formulas.
"""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

import cluster_taxi_c1 as c1  # noqa: E402
from cluster_taxi_c1 import CARD_H, CARD_W, NO_PARTICLE_BOXES, PTCL_EDGE_FADE, particle_alpha  # noqa: E402

ASSETS = c1.C1_ASSET_DIR

# what c1bg.js draw() animates in each bg2 scene (bg2/src/bg_*.js, 14:35 scan): (twinkles, drifts)
#  spring: clouds drift (day, dusk band); night = 9 stars + 5 of the 6 lanterns twinkle (c1bg.js caps at 14)
#  summer: cumulus drift + 2 front waves (all times); dusk/night = 3 light-path bars on the sea; night + 6 stars
#  autumn: clouds drift (day), cloud band (dusk); night 5 stars.  winter: clouds (day), band (dusk);
#  night = 6 stars + 2 cabin window glows.
HTML_ANIM = {
  ("spring", "day"): (0, 1), ("spring", "sunset"): (0, 1), ("spring", "night"): (14, 0),
  ("summer", "day"): (0, 3), ("summer", "sunset"): (3, 3), ("summer", "night"): (9, 3),
  ("autumn", "day"): (0, 1), ("autumn", "sunset"): (0, 1), ("autumn", "night"): (5, 0),
  ("winter", "day"): (0, 1), ("winter", "sunset"): (0, 1), ("winter", "night"): (8, 0),
}


def _anim():
  return json.loads((ASSETS / "anim.json").read_text())["seasons"]


# ---- 2. HTML animations ------------------------------------------------------------------------------------
def test_every_html_animation_has_a_sprite():
  d = _anim()
  for (s, tod), (ntw, ndr) in HTML_ANIM.items():
    lst = d.get(s, {}).get(tod, [])
    assert sorted(e["i"] for e in lst if e["kind"] == "tw") == list(range(ntw)), (s, tod)
    assert sorted(e["i"] for e in lst if e["kind"] == "dr") == list(range(ndr)), (s, tod)
    assert ntw <= c1.ANIM_TW_MAX
  assert {(s, t) for s, v in d.items() for t, lst in v.items() if lst} == {k for k, v in HTML_ANIM.items() if sum(v)}
  fr = [(s, t, e["i"]) for s, v in d.items() for t, lst in v.items() for e in lst if e["part"] == "front"]
  assert sorted(fr) == sorted((("summer", t, i) for t in ("day", "sunset", "night") for i in (1, 2)))


def test_sprites_in_the_atlas_and_inside_the_card():
  from PIL import Image
  d = _anim()
  for s, v in d.items():
    if not any(v.values()):
      continue
    alpha = np.asarray(Image.open(ASSETS / f"anim_{s}.png").convert("RGBA"))[:, :, 3]
    for tod, lst in v.items():
      for e in lst:
        sub = alpha[e["sy"]:e["sy"] + e["h"], e["sx"]:e["sx"] + e["w"]]
        assert sub.shape == (e["h"], e["w"]) and (sub > 0).any(), (s, tod, e["kind"], e["i"])
        assert 0 <= e["x"] and e["x"] + e["w"] <= CARD_W and 0 <= e["y"] and e["y"] + e["h"] <= CARD_H
        runs = c1.alpha_runs(sub)
        cover = np.zeros(sub.shape, bool)
        for rx, ry, rw, rh in runs:
          cover[ry:ry + rh, rx:rx + rw] = True
        assert cover[sub > 0].all()


def test_twinkle_and_drift_are_the_c1bg_formulas():
  e_tw = [x for x in c1.anim_elements("winter", "night", 3.0) if x[0]["kind"] == "tw"]
  for e, dx, a in e_tw:
    i = e["i"]
    assert dx == 0.0
    assert a == pytest.approx(e["v"] * (.6 + .4 * math.sin(3.0 * (1.1 + i * .37) + i * 2.1)))
  e_dr = [x for x in c1.anim_elements("summer", "day", 7.0) if x[0]["kind"] == "dr"]
  assert len(e_dr) == 3
  for e, dx, a in e_dr:
    assert a == 1.0 and dx == pytest.approx(math.sin(7.0 * .12 + e["i"] * 1.7) * e["v"])
  assert c1.anim_elements("spring", "day", 1.0, part="front") == []
  assert len(c1.anim_elements("summer", "dusk", 1.0, part="front")) == 2


def test_twinkles_actually_change_within_a_second():
  for s in ("spring", "summer", "autumn", "winter"):
    a0 = [a for _, _, a in c1.anim_elements(s, "night", 0.0) if True]
    a1 = [a for _, _, a in c1.anim_elements(s, "night", 0.5)]
    assert a0 and max(abs(p - q) for p, q in zip(a0, a1)) > 0.05, s


# ---- 1. particles fade near the card edge --------------------------------------------------------------------
def test_particle_alpha_ramps_from_the_card_edge():
  y = 300.0   # below every no-particle box
  assert particle_alpha(0.0, y) == 0.0 and particle_alpha(-5.0, y) == 0.0 and particle_alpha(CARD_W + 3, y) == 0.0
  assert particle_alpha(PTCL_EDGE_FADE, y) == 1.0 and particle_alpha(CARD_W - PTCL_EDGE_FADE, y) == 1.0
  xs = np.linspace(0, PTCL_EDGE_FADE, 49)
  a = [particle_alpha(float(x), y) for x in xs]
  assert all(q >= p for p, q in zip(a, a[1:]))
  assert max(q - p for p, q in zip(a, a[1:])) < 0.06          # smooth: no step bigger than 6 % per 0.5 px
  assert particle_alpha(400.0, CARD_H - 1.0) < 0.02 and particle_alpha(400.0, 1.0) < 0.02


def test_particle_alpha_zero_inside_no_particle_boxes():
  for b in NO_PARTICLE_BOXES:
    cx, cy = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
    assert particle_alpha(cx, cy) == 0.0
    assert particle_alpha(b[0] + 0.5, cy) == 0.0


def test_no_particle_pops_in_near_the_edge():
  """over 60 s at 20 fps, a particle's opacity never jumps by more than 0.3 between frames (r2: 0 -> 0.9 at 8 px)."""
  for s in ("spring", "autumn", "winter"):
    for tod in ("day", "dusk", "night"):
      prev = None
      for k in range(1200):
        t = k * 0.05
        ops = [op * particle_alpha(x, y) for _, x, y, _, _, op in c1.particle_positions(s, tod, t)]
        if prev is not None:
          for i, (p, q) in enumerate(zip(prev, ops)):
            q0 = c1.particle_positions(s, tod, t)[i]
            p0 = c1.particle_positions(s, tod, t - 0.05)[i]
            if abs(q0[2] - p0[2]) > 100 or abs(q0[1] - p0[1]) > 100:   # wrap-around: both ends are at alpha 0
              assert p < 0.02 and q < 0.02, (s, tod, i, t, p, q)
              continue
            assert abs(q - p) < 0.3, (s, tod, i, t, p, q)
        prev = ops
