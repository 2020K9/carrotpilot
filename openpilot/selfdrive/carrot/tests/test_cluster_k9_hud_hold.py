from dataclasses import replace
from pathlib import Path
import sys

CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

import cluster_k9_hold as hold_mod
from cluster_k9_hold import K9RoadHold
from cluster_models import DetectedVehicle, LaneMarking, ModelPathPoint
from cluster_renderer import ClusterUiRenderer as R
import render_gear_screens as rg

DT = 0.05
GONE = hold_mod.HOLD_S + hold_mod.FADE_OUT_S


def _lane(lat, visible=True):
  return LaneMarking(offset=lat, visible=visible, model_points=tuple(ModelPathPoint(float(f), lat) for f in range(0, 80, 10)))


def _state(lanes=(), cars=()):
  return replace(rg.synthetic_state("D"), car_fingerprint="KIA_K9", lanes=tuple(lanes), detected_vehicles=tuple(cars),
                 left_road_edge_points=(), right_road_edge_points=())


def _l1(d, lat=0.0, v=0.0):
  return DetectedVehicle("L1", d, lat, "radarState", primary=True, relative_speed_mps=v)


def _side(d, lat, v=0.0):
  return DetectedVehicle("A", d, lat, "modelV2.leadsV3", relative_speed_mps=v)


def _run(h, st, t):
  return h.update(st, t, R.k9_road_others, R._k9_is_lead)


def _drawn_others(st):
  return R.k9_road_others(st)


def test_still_or_first_clock_draws_the_state_as_given():
  h = K9RoadHold()
  st = _state([_lane(-1.8), _lane(1.8), _lane(5.3)], [_side(20.0, 3.5)])
  out, alpha = _run(h, st, 10.0)
  assert out is st and alpha == {}
  out, alpha = _run(h, _state(), 10.0)  # same time again (single frames, number board): no hold
  assert _drawn_others(out) == [] and alpha == {}


def test_short_lane_dropout_is_held_then_fades_and_is_gone():
  h = K9RoadHold()
  full = _state([_lane(-1.8), _lane(1.8), _lane(5.3)])
  gap = _state([_lane(-1.8), _lane(1.8), _lane(5.3, visible=False)])
  t = 0.0
  for _ in range(10):
    _run(h, full, t)
    t += DT
  out, alpha = _run(h, gap, t)
  outer = [ln for ln in out.lanes if ln.visible and ln.model_points[0].lateral_m > 4.0]
  assert len(outer) == 1 and alpha.get(id(outer[0]), 1.0) == 1.0  # held at full strength first
  t0 = t
  while t - t0 < GONE + 0.2:
    t += DT
    out, alpha = _run(h, gap, t)
    outer = [ln for ln in out.lanes if ln.visible and ln.model_points[0].lateral_m > 4.0]
    if t - t0 > hold_mod.HOLD_S + 0.01 and outer:
      assert alpha[id(outer[0])] < 1.0
  assert outer == []


def test_car_that_never_comes_back_is_gone_after_the_hold():
  h = K9RoadHold()
  t = 0.0
  for _ in range(10):
    _run(h, _state(cars=[_side(30.0, 3.5, 2.0)]), t)
    t += DT
  last_seen = t - DT
  seen_after = []
  while t - last_seen < 3.0:
    out, alpha = _run(h, _state(), t)
    if _drawn_others(out):
      seen_after.append(t - last_seen)
      v = _drawn_others(out)[0]
      assert v.longitudinal_m > 30.0  # carried on with its last relative speed
    t += DT
  assert seen_after and max(seen_after) < GONE + 1e-6


def test_new_close_lead_is_shown_at_once_at_full_strength():
  h = K9RoadHold()
  t = 0.0
  for _ in range(10):
    _run(h, _state(), t)
    t += DT
  out, alpha = _run(h, _state(cars=[_l1(8.0)]), t)
  lead = R.k9_turzx_lead(out)
  assert lead is not None and abs(lead[0] - 8.0) < 1e-6
  assert all(a == 1.0 for a in alpha.values())


def test_one_frame_car_dropout_is_bridged_and_new_side_car_fades_in():
  h = K9RoadHold()
  t = 0.0
  _run(h, _state(), t)
  t += DT
  out, alpha = _run(h, _state(cars=[_side(25.0, -3.5)]), t)
  first = _drawn_others(out)[0]
  assert alpha[id(first)] < 1.0  # fade in
  for _ in range(10):
    t += DT
    _run(h, _state(cars=[_side(25.0, -3.5)]), t)
  t += DT
  out, _ = _run(h, _state(), t)
  assert len(_drawn_others(out)) == 1
  t += DT
  out, alpha = _run(h, _state(cars=[_side(25.0, -3.5)]), t)
  assert len(_drawn_others(out)) == 1 and alpha.get(id(_drawn_others(out)[0]), 1.0) == 1.0


def test_lane_change_position_lag_is_limited():
  h = K9RoadHold()
  t, lat = 0.0, -1.8
  worst = 0.0
  for _ in range(60):  # 1.5 m/s sideways: the line keeps within LANE_MAX_LAG_M of the model
    out, _ = _run(h, _state([_lane(lat), _lane(lat + 3.6)]), t)
    near = sorted(ln.model_points[0].lateral_m for ln in out.lanes)
    worst = max(worst, abs(near[0] - lat))
    t += DT
    lat += 1.5 * DT
  assert worst <= hold_mod.LANE_MAX_LAG_M + 1e-6
