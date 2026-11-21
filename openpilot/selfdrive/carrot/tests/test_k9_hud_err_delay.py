"""K9JetErrDelay (실험중 '젯슨 에러지연', 대감님 2026-11-21 11:09): the '에러지연 작동' badge on every external HUD drive
board. Drawn only while state.jet_err_delay is True (cluster_live latch, >= 1 s from a held Jetson frame), once per
board in a fixed slot, off every other text / gauge / road car (the no-overlap recorder of test_k9_hud_no_overlap).
GL checks need xvfb-run."""
import os
import sys
from dataclasses import replace
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "cluster"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from test_k9_hud_no_overlap import GAP, LAT, TOP_ROW, _k9_screen, _set_menu, boxes, hud, k9_road, overlaps  # noqa: F401

pytestmark = pytest.mark.skipif(not os.environ.get("DISPLAY"), reason="needs a GL display (xvfb-run)")

BADGE = "에러지연 작동"
SCREENS = ("classic", 1)   # 11/21 15:47: K9HudNavdy 0 / 2 / 3 removed, board 1 only


def _badges(bs):
  return [b for b in bs if b[0] == BADGE]


@pytest.mark.parametrize("screen", SCREENS)
def test_badge_only_while_on_and_once(hud, screen):  # noqa: F811
  r, _, _ = hud
  _k9_screen(r, screen)
  try:
    st = k9_road(22.0, True)
    off = boxes(hud, st)
    on = boxes(hud, replace(st, jet_err_delay=True))
  finally:
    _k9_screen(r, 1)
    r.set_screen_mode(0)
    _set_menu(r, 0)
  assert _badges(off) == []
  found = _badges(on)
  assert len(found) == 1 and found[0][1] == "_draw_jet_err_delay", found
  # every other text box is the same with the badge on: nothing moved for it
  assert sorted(b for b in on if b[0] != BADGE) == sorted(off)


@pytest.mark.parametrize("lane", (True, False), ids=("lane", "nolane"))
@pytest.mark.parametrize("screen", SCREENS)
def test_badge_off_every_text_gauge_and_car(hud, screen, lane):  # noqa: F811
  r, _, _ = hud
  _k9_screen(r, screen)
  try:
    for lead_d in (None, 8.0, 22.0, 45.0, 80.0):
      bs = boxes(hud, replace(k9_road(lead_d, lane), jet_err_delay=True))
      assert len(_badges(bs)) == 1
      bad = overlaps(bs, GAP, TOP_ROW if screen in (1, 2) else (), cars=True)
      assert [(p, q) for p, q in bad if BADGE in (p[0], q[0])] == [], (screen, lead_d, lane)
  finally:
    _k9_screen(r, 1)
    r.set_screen_mode(0)
    _set_menu(r, 0)


@pytest.mark.parametrize("layout", ("left", "right"))
def test_classic_ev_badge_follows_the_driving_panel(hud, layout):  # noqa: F811
  from cluster_config import CLUSTER_PANEL_LAYOUT_DRIVING_LEFT, CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT
  r, _, rg = hud
  _set_menu(r, 0)
  r.set_panel_layout(CLUSTER_PANEL_LAYOUT_DRIVING_LEFT if layout == "left" else CLUSTER_PANEL_LAYOUT_DRIVING_RIGHT)
  try:
    bs = boxes(hud, replace(rg.synthetic_state("D"), jet_err_delay=True, **LAT))
  finally:
    r.set_panel_layout(CLUSTER_PANEL_LAYOUT_DRIVING_LEFT)
  found = _badges(bs)
  assert len(found) == 1
  x0 = found[0][2]
  assert (x0 < 960) == (layout == "left")   # the driving column moves with the panel swap, the badge with it
  assert [(p, q) for p, q in overlaps(bs) if BADGE in (p[0], q[0])] == []
