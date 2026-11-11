"""K9LaneAssist conflict-audit fixes (2026-10-26, 보노보노 23:43 "차선 관련 실험중 메뉴들 동시 사용 시 충돌"). Synthetic frames only,
NOT vehicle-validated.
A1 lane mode + F1 + PathOffset: the F1 merge starts at the car and ends on lane path + PathOffset (PathOffset is not added on
   top of the whole merged path any more); K9LaneAssist 0 and PathOffset 0 unchanged.
C1 laneless -> lane mode: the hold push ramps out (controlsd adds the ramp-out to the lane-mode command).
C4 a model desire (ATC turn without blinker) turns both holds off like a lane change.
"""
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.controls.tests.test_k9_lane_fix import (LP_NEW, MemParams, PATH_T, N, make_lp, lp_frames, lp_step)
from openpilot.selfdrive.controls.lib import k9_lane_center as H
from openpilot.selfdrive.controls.lib import k9_one_lane_hold as OLH

ROOT = Path(__file__).resolve().parents[3]
SEEDS = [0, 1, 2, 3, 4]


def _steady_frame(v, car_from_lane):
  # straight lane, the car (model path start, y 0) sits car_from_lane right of the lane centre, model path straight ahead
  x = v * PATH_T
  lane_c = -car_from_lane * np.ones(N)
  path = np.stack([x, np.zeros(N), np.zeros(N)], axis=1)
  return dict(v=v, ll_t=PATH_T.copy(), ll_x=x.copy(), lll_y=lane_c - 1.65, rll_y=lane_c + 1.65, lll_prob=0.95, rll_prob=0.95,
              lll_std=0.05, rll_std=0.05, lanefull_mode=True, lane_change_multiplier=1.0, lane_width_left=3.3,
              lane_width_right=3.3, path=path, curve_speed=0.0)


@pytest.mark.parametrize("po", [0.3, -0.3, 0.15])
def test_a1_path_offset_target_and_start_at_the_car(po):
  lp = make_lp(LP_NEW, MemParams(AdjustLaneOffset=0, LatMpcInputOffset=0, K9LaneAssist=1))
  lp.k9_path_offset = po
  # car already where it should be (po right of the centre): the reference must stay there, not creep further out
  out = [lp_step(lp, _steady_frame(20.0, po)) for _ in range(60)]
  path, active = out[-1]
  assert active and lp.k9_path_offset_applied
  assert abs(path[0, 1]) < 0.02                         # starts at the car
  assert abs(path[-1, 1]) < 0.02                        # ends at lane centre + po = the car's place (no extra po)
  # car at the lane centre: the reference ends po beside it (on lane path + PathOffset) and starts at the car
  # (pin rewritten 2026-10-26 recheck_0208 merge latch: "starts at the car" holds on the lane-mode entry frame, where the merge
  # is latched; this frame sequence never moves the car, so after the merge time the reference is lane path + po itself)
  lp2 = make_lp(LP_NEW, MemParams(AdjustLaneOffset=0, LatMpcInputOffset=0, K9LaneAssist=1))
  lp2.k9_path_offset = po
  # On the entry frame the 0.5 s ramp-in (unchanged) blends from the laneless reference (model path + po) with weight g, so
  # the merge being ramped in is recovered as po + (path - po) / g.
  out2 = []
  for _ in range(80):   # lane mode from frame ~21, merge up to t_max 2 s
    p_, a_ = lp_step(lp2, _steady_frame(20.0, 0.0)); out2.append((p_, a_, lp2.lane_merge_k, lp2.lane_ramp_g))
  entry = [(p, g) for p, a, mk, g in out2 if mk == 0]
  assert len(entry) == 1 and entry[0][1] > 0.0
  merged = po + (entry[0][0][:, 1] - po) / entry[0][1]
  assert abs(merged[0]) < 0.02 and merged[-1] == pytest.approx(po, abs=0.02)
  path2 = out2[-1][0]
  assert out2[-1][2] * 0.05 > lp2.lane_merge_t                    # merge over
  assert path2[-1, 1] == pytest.approx(po, abs=0.02) and path2[0, 1] == pytest.approx(po, abs=0.02)   # po once, not twice


@pytest.mark.parametrize("seed", SEEDS[:3])
def test_a1_switch_off_or_no_offset_is_unchanged(seed):
  frames = lp_frames(seed)
  for k9 in (0, 1):
    a = make_lp(LP_NEW, MemParams(AdjustLaneOffset=0, LatMpcInputOffset=6, K9LaneAssist=k9))
    b = make_lp(LP_NEW, MemParams(AdjustLaneOffset=0, LatMpcInputOffset=6, K9LaneAssist=k9))
    b.k9_path_offset = 0.0
    for f in frames:
      pa, aa = lp_step(a, f); pb, ab = lp_step(b, f)
      assert np.array_equal(pa, pb) and aa == ab
  lp = make_lp(LP_NEW, MemParams(AdjustLaneOffset=0, LatMpcInputOffset=6, K9LaneAssist=0))
  lp.k9_path_offset = 0.3
  lp_step(lp, frames[0])
  assert lp.k9_path_offset_applied is False            # switch 0: lateral_planner adds PathOffset as before


def test_a1_lateral_planner_adds_offset_once():
  src = (ROOT / "selfdrive/controls/lib/lateral_planner.py").read_text()
  assert "self.LP.k9_path_offset = self.pathOffset" in src
  assert "if not getattr(self.LP, 'k9_path_offset_applied', False):\n      self.path_xyz[:, 1] += self.pathOffset" in src


def test_c1_lane_mode_command_gets_the_ramp_out():
  src = (ROOT / "selfdrive/controls/controlsd.py").read_text()
  i = src.index("elif self.lanefull_mode_enabled:")
  block = src[i:src.index("else:", src.index("new_desired_curvature = smooth_value(curvature", i))]
  assert "if k9_lane_add != 0.0:" in block and "curvature = curvature + k9_lane_add" in block


class _CS(NS):
  pass


def _md(desire):
  return NS(meta=NS(laneChangeState=0, desire=desire), laneLines=[], laneLineProbs=[], laneLineStds=[],
            action=NS(desiredCurvature=0.0), frameId=1)


@pytest.mark.parametrize("desire,off", [("none", False), ("turnLeft", True), ("turnRight", True), ("keepLeft", True)])
def test_c4_desire_turns_the_holds_off(desire, off):
  seen = {}
  class Spy:
    active = False
    gain = 1.0; off = 0.0
    def update(self, md, v, laneless, lane_change, pressed, curv):
      seen["lc"] = lane_change; return 0.0
  for mod, step, attr in ((H, H.k9_lane_center_step, "k9_lc"), (OLH, OLH.k9_one_lane_hold_step, "k9_olh")):
    c = NS(params=NS(get_int=lambda k: 1, get=lambda k: 1), is_vw_meb=False, lanefull_mode_enabled=False)
    setattr(c, attr, Spy())
    step(c, NS(vEgo=20.0, leftBlinker=False, rightBlinker=False, steeringPressed=False), NS(latActive=True), _md(desire), 0)
    assert seen["lc"] is off, (attr, desire)
