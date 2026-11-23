"""BSD (carState leftBlindspot/rightBlindspot) cars on the HUD road, 대감님 11/23 20:43 "BSD까지 넣자"."""
from pathlib import Path
from types import SimpleNamespace
import sys

CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
JETLINK_DIR = Path(__file__).resolve().parents[4] / "tools" / "jetlink"
sys.path.insert(0, str(CLUSTER_DIR))
sys.path.insert(0, str(JETLINK_DIR))

import cluster_renderer as cr  # noqa: E402
from cluster_models import DetectedVehicle  # noqa: E402
from cluster_renderer import ClusterUiRenderer  # noqa: E402
import hud_protocol  # noqa: E402


def st(left=False, right=False, vs=(), lane_w=3.6):
  return SimpleNamespace(detected_vehicles=tuple(vs), left_blindspot=left, right_blindspot=right, lane_width_m=lane_w)


def bsd(others):
  return {v.label: v for v in others if v.source == cr.K9_BSD_SOURCE}


def test_bsd_values_reach_the_jetson():
  assert {"leftBlindspot", "rightBlindspot"} <= set(hud_protocol.SERVICE_FIELDS["carState"])


def test_no_bsd_no_extra_car():
  assert bsd(ClusterUiRenderer.k9_road_others(st())) == {}


def test_left_and_right_bsd_cars_in_the_next_lanes():
  by = bsd(ClusterUiRenderer.k9_road_others(st(left=True, right=True)))
  assert set(by) == {"BL", "BR"}
  assert by["BL"].lateral_m == -3.6 and by["BR"].lateral_m == 3.6
  assert all(0.0 < v.longitudinal_m <= cr.K9_BSD_NEAR_M for v in by.values())


def test_radar_side_car_near_on_that_side_is_not_drawn_twice():
  near = DetectedVehicle(label="SL", longitudinal_m=6.0, lateral_m=-3.4, source="radarState")
  far = DetectedVehicle(label="SR", longitudinal_m=30.0, lateral_m=3.4, source="radarState")
  by = bsd(ClusterUiRenderer.k9_road_others(st(left=True, right=True, vs=(near, far))))
  assert set(by) == {"BR"}   # left: the radar car at 6 m is that car; right: 30 m is another car


def test_bsd_car_after_hold_is_not_doubled():
  first = ClusterUiRenderer.k9_road_others(st(left=True))
  again = ClusterUiRenderer.k9_road_others(st(left=True, vs=first))   # the hold feeds the drawn cars back
  assert len(bsd(again)) == 1


def _renderer():
  r = ClusterUiRenderer.__new__(ClusterUiRenderer)
  r._k9_navdy_cache = {}           # no sprites: the box fallback
  r._k9_navdy_ink = {}
  r._k9_exp1_cam = cr.k9_exp1_camera()
  r._k9_cam_proj = None
  r._k9_cx = cr.K9L_CX
  r.k9_hud_res = False
  r._k9l_perc_rects = lambda s: []
  r._k9l_navi_rects = lambda s: []
  r._k9l_top_rects = lambda s: []
  return r


def _hit(a, b):
  return not (a[2] < b[0] or a[0] > b[2] or a[3] < b[1] or a[1] > b[3])


def test_bsd_spot_on_screen_clear_of_my_car_and_text():
  r = _renderer()
  ego = r._k9_vehicle_box(cr.K9_EXP1_EGO_REAR_M, 0.0)
  rect = r.k9_cam_scene_rect()
  for side in (-1.0, 1.0):
    v = DetectedVehicle(label="BL" if side < 0 else "BR", longitudinal_m=0.5, lateral_m=side * 3.6,
                        source=cr.K9_BSD_SOURCE)
    f, lat = r.k9_bsd_spot(st(), v, [], [])
    box = r._k9_vehicle_box(f, lat)
    assert not _hit(box, ego)
    assert rect.x <= box[0] and box[2] <= rect.x + rect.width and box[3] <= rect.y + rect.height
    assert f < cr.K9_BSD_SCAN_MAX_M and side * lat > 2.5
  # a text box over the first spot pushes the car on to the next free one
  v = DetectedVehicle(label="BL", longitudinal_m=0.5, lateral_m=-3.6, source=cr.K9_BSD_SOURCE)
  f0, lat0 = r.k9_bsd_spot(st(), v, [], [])
  b0 = r._k9_vehicle_box(f0, lat0)
  r._k9l_perc_rects = lambda s: [b0]
  f1, lat1 = r.k9_bsd_spot(st(), v, [], [])
  assert f1 > f0 and not _hit(r._k9_vehicle_box(f1, lat1), b0)
