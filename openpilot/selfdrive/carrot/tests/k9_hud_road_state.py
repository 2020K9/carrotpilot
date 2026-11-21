"""road sample for the K9 D HUD tests (moved from test_k9_hud_3d.py when K9HudNavdy 3 was removed 11/21)"""
from dataclasses import replace

from cluster_models import DetectedVehicle, LaneMarking, ModelPathPoint


def line(off, curv=0.0015):
  return tuple(ModelPathPoint(float(f), off + curv * f * f) for f in range(0, 120, 3))


def road_state(base, lead_accel=-0.2, lead_d=22.0, left=False):
  lanes = tuple(LaneMarking(o, model_points=line(o)) for o in (-5.4, -1.8, 1.8, 5.4))
  vs = (DetectedVehicle("L1", lead_d, 0.2, "radar", relative_speed_mps=-0.6, acceleration_mps2=lead_accel, primary=True),
        DetectedVehicle("R2", 14.0, 3.9, "radar", radar_track_id=7))
  return replace(base, car_fingerprint="KIA_K9", lanes=lanes, left_road_edge_points=line(-6.6),
                 right_road_edge_points=line(6.6), detected_vehicles=vs, left_signal=left, speed_kph=56.0,
                 display_speed_kph=56.0)
