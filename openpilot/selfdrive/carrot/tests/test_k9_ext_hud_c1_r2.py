"""K9ExtHud=1 C1 meter, r2 (13:20 지시 2·3·4), no GL.

2. LFA state on the C1 screen (carrot wheel stays hidden): LFA off = clam dimmed; small badge under the clam
   (lane lines solid = lane lines in use, faint = laneless; "LFA" green/grey). Badge overlaps nothing.
3. bg2 backgrounds: particles follow bg2/ptcl_params.json (copied to the assets), 12 back/over layers present.
4. CPU cache: sprites drawn from their alpha bbox, 'over' drawn as runs of non-empty tiles; every pixel with
   alpha > 0 is still covered (the picture is unchanged; the GL pixel compare is in the r2 report).
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
from cluster_taxi_c1 import (  # noqa: E402
  BONO_X, BONO_Y, BUBBLE_MAX_RIGHT, CARD_H, CARD_W, FARE_BOX, LFA_BADGE, NO_PARTICLE_BOXES, C1MeterPanel, alpha_bbox,
  alpha_runs, clam_geometry,
)

ASSETS = c1.C1_ASSET_DIR
FOOTER_BOX = (21.0, 398.0, 645.0, 446.0)   # fare footer pill of the over layers (measured: x 21..645, y 398..446)
STATS_BOX = (546.0, 72.0, 758.0, 236.0)
SHELL_REACH = 32.0                          # clam sprite circumradius (bbox 46x38 px about its centre) + hands


def _hit(a, b):
  return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def _badge_box():
  x, y, w, h = LFA_BADGE
  return (x, y, x + w, y + h)


class _State:
  def __init__(self, lfa, lane, v=0.0):
    self.speed_kph, self.steering_angle_deg, self.onroad = v, 0.0, True
    self.lfa_active, self.active_lane_line = lfa, lane


# ---- 2. LFA ---------------------------------------------------------------------------------------------
def test_lfa_state_follows_the_carrot_wheel_rule():
  p = C1MeterPanel(clock=lambda: 100.0, local_hour=lambda: 12)
  for lfa, lane, want in ((True, True, (True, True)), (False, True, (False, True)), (True, False, (True, False)),
                          (None, None, (False, False))):
    p.update(_State(lfa, lane))
    assert (p.lfa_active, p.lane_line) == want


def test_lfa_badge_inside_card_and_clear_of_everything():
  b = _badge_box()
  assert 0 < b[0] and b[2] < CARD_W - 10 and 0 < b[1] and b[3] < CARD_H - 10
  for other in (FOOTER_BOX, FARE_BOX, STATS_BOX, *NO_PARTICLE_BOXES):
    assert not _hit(b, other), other
  assert BUBBLE_MAX_RIGHT < b[0]                       # 포로리's bubble never reaches it


def test_lfa_badge_clear_of_the_clam_and_hands_at_any_angle():
  top = _badge_box()[1]
  low = 0.0
  for steer in range(-720, 721, 15):
    for bob in (-4.5, 0.0, 4.5):
      for brot in (-3.2, 0.0, 3.2):
        for tap in (0.0, 1.0):
          g = clam_geometry(float(steer), bob, brot, tap, (BONO_X, BONO_Y))
          low = max(low, g["center"][1] + SHELL_REACH, *(hy + 6.0 for _, hy in g["hands"]))
  assert low < top, (low, top)


# ---- 4. cache: crop / runs keep every visible pixel -----------------------------------------------------
def test_alpha_bbox_and_runs_cover_every_pixel():
  rng = np.random.default_rng(3)
  for _ in range(20):
    a = np.zeros((100, 150), np.uint8)
    for _ in range(rng.integers(1, 6)):
      x, y = rng.integers(0, 140), rng.integers(0, 90)
      a[y:y + rng.integers(1, 10), x:x + rng.integers(1, 10)] = rng.integers(1, 255)
    x, y, w, h = alpha_bbox(a)
    ys, xs = np.nonzero(a)
    assert xs.min() >= x and xs.max() < x + w and ys.min() >= y and ys.max() < y + h
    cov = np.zeros(a.shape, bool)
    for rx, ry, rw, rh in alpha_runs(a):
      assert not cov[ry:ry + rh, rx:rx + rw].any()      # no pixel drawn twice
      cov[ry:ry + rh, rx:rx + rw] = True
    assert cov[a > 0].all()


def _alpha(path):
  from PIL import Image
  return np.asarray(Image.open(path).convert("RGBA"))[:, :, 3]


@pytest.mark.parametrize("f", sorted(p.name for p in ASSETS.glob("over_*.png")))
def test_over_runs_cover_the_real_layers_and_skip_empty(f):
  a = _alpha(ASSETS / f)
  runs = alpha_runs(a)
  cov = np.zeros(a.shape, bool)
  for rx, ry, rw, rh in runs:
    cov[ry:ry + rh, rx:rx + rw] = True
  assert cov[a > 0].all()
  assert cov.mean() < 0.9                                # something is actually skipped
  assert len(runs) < 400                                 # few draw calls


def test_sprite_crops_cover_the_sprites():
  for name in ("porori_body", "porori_tail", "porori_fL", "porori_fR", "porori_aL", "porori_aR", "bono_body", "shell"):
    a = _alpha(ASSETS / f"{name}.png")
    x, y, w, h = alpha_bbox(a)
    assert a.sum() == a[y:y + h, x:x + w].sum()
    assert w * h < a.size


# ---- 3. bg2 backgrounds / particles ----------------------------------------------------------------------
def test_twelve_background_layers_present():
  for s in ("spring", "summer", "autumn", "winter"):
    for tod in ("day", "dusk", "night"):
      for kind in ("back", "over"):
        assert (ASSETS / f"{kind}_{s}_{tod}.png").exists(), (kind, s, tod)


def test_particles_follow_bg2_params():
  d = json.loads((ASSETS / "ptcl_params.json").read_text())["seasons"]
  for s, tods in d.items():
    for tod_json, q in tods.items():
      tod = {"sunset": "dusk"}.get(tod_json, tod_json)
      ps = c1.particle_positions(s, tod, 7.3)
      assert len(ps) == int(q.get("n") or 0), (s, tod)
      kinds = q.get("k") if isinstance(q.get("k"), list) else [q.get("k")]
      for i, (k, x, y, rot, col, op) in enumerate(ps):
        assert k == kinds[i % len(kinds)] and k in c1.PTCL_KINDS
        assert 0.0 <= op <= 1.0 and math.isfinite(x) and math.isfinite(y)


def test_particle_motion_is_the_c1bg_formula():
  ps = c1.particle_positions("winter", "night", 10.0)
  k, x, y, rot, col, op = ps[1]
  assert k == "snow" and rot == 0.0
  assert y == pytest.approx(((53 + 10.0 * 27) % 500) - 22)
  assert x == pytest.approx((83.7 + math.sin(9.0 + 1.7) * 10 + 30.0) % 800 - 12)
  fl = c1.particle_positions("summer", "night", 2.0)
  assert len(fl) == 9 and all(p[0] == "fly" for p in fl)
  assert fl[0][5] == pytest.approx(.25 + .75 * max(0.0, math.sin(3.2)))
  assert c1.particle_positions("summer", "day", 2.0) == []
