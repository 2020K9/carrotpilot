"""Side cars (radarState leadLeft/leadRight) on every HUD road, 11/23 "어떤 모델이든 옆차도 당연히 확인을 해서 보이게"."""
import base64
from pathlib import Path
from types import SimpleNamespace
import sys

CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
JETLINK_DIR = Path(__file__).resolve().parents[4] / "tools" / "jetlink"
sys.path.insert(0, str(CLUSTER_DIR))
sys.path.insert(0, str(JETLINK_DIR))

from cluster_renderer import ClusterUiRenderer  # noqa: E402
from cluster_route_replay import RouteLogParser, radar_state_has_side_leads  # noqa: E402
import hud_protocol  # noqa: E402

EMPTY = SimpleNamespace(status=False)


def lead(track_id, d_rel, y_rel, v_lead=15.0, v_rel=-1.0):
  return SimpleNamespace(status=True, radar=True, radarTrackId=track_id, dRel=d_rel, yRel=y_rel, vRel=v_rel,
                         vLead=v_lead, vLat=0.0, aLeadK=0.0)


def frame_vehicles(parser, radar_state, t=1.0, tracks=None, v_ego=None):
  if tracks is not None:
    parser._update_live_tracks(SimpleNamespace(points=tracks), t)
  parser._update_radar_state(radar_state, t)
  car_state = SimpleNamespace() if v_ego is None else SimpleNamespace(vEgo=v_ego, vEgoCluster=v_ego)
  return parser._frame_from_car_state(car_state, t + 0.02).detected_vehicles


def test_left_and_right_side_leads_are_drawn_cars():
  parser = RouteLogParser(recompute_cutins=False)
  vs = frame_vehicles(parser, SimpleNamespace(leadOne=lead(1, 30.0, 0.0), leadTwo=EMPTY, leadsCutIn=[],
                                              leadLeft=lead(7, 12.0, 3.4), leadRight=lead(9, 18.0, -3.5)))
  by = {v.label: v for v in vs}
  assert {"L1", "SL", "SR"} <= set(by)
  assert by["SL"].lateral_m < -2.5 and by["SR"].lateral_m > 2.5   # renderer x is right-positive
  assert not by["SL"].primary and not by["SR"].primary
  others = ClusterUiRenderer.k9_road_others(SimpleNamespace(detected_vehicles=vs))
  assert {v.label for v in others} == {"SL", "SR"}


def test_standing_side_object_is_not_a_car():
  parser = RouteLogParser(recompute_cutins=False)
  parser.current_speed_kph = 80.0
  vs = frame_vehicles(parser, SimpleNamespace(leadOne=EMPTY, leadTwo=EMPTY, leadsCutIn=[],
                                              leadLeft=lead(7, 12.0, 3.4, v_lead=0.3, v_rel=-22.0),
                                              leadRight=EMPTY))
  assert not any(v.label == "SL" for v in vs)


def test_standing_side_car_is_drawn_while_ego_stands_too():
  parser = RouteLogParser(recompute_cutins=False)
  parser.current_speed_kph = 0.0
  vs = frame_vehicles(parser, SimpleNamespace(leadOne=EMPTY, leadTwo=EMPTY, leadsCutIn=[],
                                              leadLeft=lead(7, 6.0, 3.3, v_lead=0.0, v_rel=0.0),
                                              leadRight=lead(8, 40.0, -3.3, v_lead=0.0, v_rel=0.0)))
  assert [v.label for v in vs] == ["SL"]   # the far standing one stays hidden


def test_side_lead_that_is_the_lead_car_is_not_drawn_twice():
  parser = RouteLogParser(recompute_cutins=False)
  vs = frame_vehicles(parser, SimpleNamespace(leadOne=lead(4, 20.0, 1.6), leadTwo=EMPTY, leadsCutIn=[],
                                              leadLeft=lead(4, 20.0, 1.6), leadRight=EMPTY))
  assert [v.label for v in vs] == ["L1"]


def test_older_comma_without_side_leads_uses_moving_side_radar_tracks():
  parser = RouteLogParser(recompute_cutins=False)
  parser.current_speed_kph = 72.0
  tracks = (SimpleNamespace(trackId=11, dRel=15.0, yRel=3.6, vRel=-2.0, measured=True),     # left lane car
            SimpleNamespace(trackId=12, dRel=9.0, yRel=-3.4, vRel=-20.0, measured=True),    # guardrail post
            SimpleNamespace(trackId=13, dRel=25.0, yRel=0.1, vRel=-1.0, measured=True))     # own lane
  old_state = SimpleNamespace(leadOne=lead(13, 25.0, 0.1), leadTwo=EMPTY, leadsCutIn=[])
  assert not radar_state_has_side_leads(old_state)
  vs = frame_vehicles(parser, old_state, tracks=tracks, v_ego=20.0)
  sides = [v for v in vs if v.label in ("SL", "SR")]
  assert [(v.label, v.longitudinal_m) for v in sides] == [("SL", 15.0)]


def test_snapshot_carries_side_leads_within_event_limit():
  from openpilot.cereal import log
  ev = log.Event.new_message()
  rs = ev.init("radarState")
  for name in ("leadOne", "leadTwo", "leadLeft", "leadRight"):
    getattr(rs, name).status = True
    getattr(rs, name).dRel = 20.0
  rs.leadLeft.yRel = 3.5
  rs.init("leadsCutIn", 2)
  data = hud_protocol.display_event("radarState", ev.as_reader().radarState)
  assert len(base64.b64encode(data)) <= hud_protocol.EVENT_LIMITS.get("radarState", hud_protocol.EVENT_LIMIT)
  with log.Event.from_bytes(data) as back:
    assert back.radarState.leadLeft.status and back.radarState.leadLeft.yRel == 3.5
    assert radar_state_has_side_leads(back.radarState)
  # an older comma's snapshot: no leadLeft/leadRight -> the HUD knows and falls back
  old = log.Event.new_message()
  old.init("radarState").leadOne.status = True
  with log.Event.from_bytes(old.to_bytes()) as back:
    assert not radar_state_has_side_leads(back.radarState)
