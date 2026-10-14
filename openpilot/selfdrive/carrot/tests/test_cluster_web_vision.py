from pathlib import Path
import math
import struct
import sys

import pytest

CLUSTER_DIR = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER_DIR))

from cluster_config import (
  CLUSTER_CAMERA_VIEW_MODE_AUTO_CAMERA,
  CLUSTER_CAMERA_VIEW_MODE_DEFAULT,
  CLUSTER_CAMERA_VIEW_MODE_EGO_BOTTOM,
  CLUSTER_CAMERA_VIEW_MODE_PARAM,
  CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
  CLUSTER_CAMERA_VIEW_MODE_WEB_VISION,
  CLUSTER_CAMERA_VIEW_MODE_WIDE_CAMERA,
  cluster_camera_view_is_road_camera,
  cluster_camera_view_is_web_vision,
  cluster_camera_view_prefers_wide,
  cluster_web_vision_unsupported_notice,
  normalize_cluster_camera_view_mode,
)
from cluster_web_vision import (
  CruiseOverrideHold,
  MODEL_FRAME_HISTORY_LIMIT,
  MODEL_STALE_SECONDS,
  WEB_VISION_SERVICES,
  EraseRoundRect,
  Label,
  RoundRect,
  WebVisionInputs,
  WebVisionOverlay,
  canvas_erase_hides_items,
  derive_web_hud_data,
  normalize_web_vision_params,
  read_web_vision_raw_params,
  web_alert_layout,
  web_hud_layout,
  web_state_from_message,
  select_synchronized_model,
)
from openpilot.cereal import log
from openpilot.selfdrive.carrot.realtime.compact_state import SERVICE_SCHEMAS, encode_carrot_state_frame

VIDEO_W, VIDEO_H = 526.0, 330.0
STAGE_W, STAGE_H = 1124.0, 480.0


def test_camera_view_values_zero_to_four_keep_their_meaning():
  assert [normalize_cluster_camera_view_mode(v) for v in range(5)] == [
    CLUSTER_CAMERA_VIEW_MODE_DEFAULT,
    CLUSTER_CAMERA_VIEW_MODE_EGO_BOTTOM,
    CLUSTER_CAMERA_VIEW_MODE_ROAD_CAMERA,
    CLUSTER_CAMERA_VIEW_MODE_WIDE_CAMERA,
    CLUSTER_CAMERA_VIEW_MODE_AUTO_CAMERA,
  ] == [0, 1, 2, 3, 4]
  assert [cluster_camera_view_is_road_camera(v) for v in range(5)] == [False, False, True, True, True]
  assert cluster_camera_view_prefers_wide(3, 100.0, False) is True
  assert cluster_camera_view_prefers_wide(4, 10.0, False) is True
  assert cluster_camera_view_prefers_wide(2, 10.0, False) is False
  assert not any(cluster_camera_view_is_web_vision(v) for v in range(5))


def test_camera_view_five_is_web_vision_on_the_narrow_road_camera():
  assert CLUSTER_CAMERA_VIEW_MODE_WEB_VISION == 5
  assert normalize_cluster_camera_view_mode(5) == 5
  assert normalize_cluster_camera_view_mode("5") == 5
  assert normalize_cluster_camera_view_mode("web-vision") == 5
  assert normalize_cluster_camera_view_mode(6) == CLUSTER_CAMERA_VIEW_MODE_DEFAULT
  assert cluster_camera_view_is_road_camera(5)
  assert cluster_camera_view_is_web_vision(5)
  # The web vision streams the normal road camera, never the wide one.
  assert cluster_camera_view_prefers_wide(5, 10.0, False) is False
  assert cluster_camera_view_prefers_wide(5, 100.0, True) is False


def test_camera_view_five_survives_a_params_round_trip(tmp_path, monkeypatch):
  from openpilot.common.params import Params

  params = Params()
  previous = params.get(CLUSTER_CAMERA_VIEW_MODE_PARAM)
  try:
    params.put(CLUSTER_CAMERA_VIEW_MODE_PARAM, 5)
    assert normalize_cluster_camera_view_mode(params.get_int(CLUSTER_CAMERA_VIEW_MODE_PARAM)) == 5
    for value in range(5):
      params.put(CLUSTER_CAMERA_VIEW_MODE_PARAM, value)
      assert normalize_cluster_camera_view_mode(params.get_int(CLUSTER_CAMERA_VIEW_MODE_PARAM)) == value
  finally:
    if previous is None:
      params.remove(CLUSTER_CAMERA_VIEW_MODE_PARAM)
    else:
      params.put(CLUSTER_CAMERA_VIEW_MODE_PARAM, previous)


# ---------------------------------------------------------------------------
# Inputs have the browser's decoded shape
# ---------------------------------------------------------------------------

def _decode_compact(frame: bytes, schema):
  """Python mirror of web/js/realtime/vision_compact.js for the field types used here."""
  offset = 8

  def read(fmt):
    nonlocal offset
    value = struct.unpack_from("<" + fmt, frame, offset)
    offset += struct.calcsize("<" + fmt)
    return value[0] if len(value) == 1 else value

  def field(spec, service, name):
    if spec == "bool":
      return read("B") != 0
    if spec in ("i8", "u8", "i16", "u16", "i32", "u32", "u64"):
      return read({"i8": "b", "u8": "B", "i16": "h", "u16": "H", "i32": "i", "u32": "I", "u64": "Q"}[spec])
    if spec == "f32":
      return read("f")
    if spec == "text":
      length = read("H")
      text = frame[offset:offset + length].decode()
      advance(length)
      return text
    if spec in ("f32_list", "f32_first_list"):
      return [read("f") for _ in range(read("H"))]
    if spec in ("u16_cm_list", "i16_cm_list", "i16_mm_list"):
      count = read("H")
      fmt, scale = {"u16_cm_list": ("H", 100), "i16_cm_list": ("h", 100), "i16_mm_list": ("h", 1000)}[spec]
      return [read(fmt) / scale for _ in range(count)]
    if spec[0] == "enum":
      index = read("B")
      named = (service, name) in {("carState", "gearShifter"), ("deviceState", "deviceType"),
                                  ("liveCalibration", "calStatus"), ("roadCameraState", "sensor")}
      return spec[1][index] if named else index
    if spec[0] == "struct":
      return struct_value(spec[1], service)
    if spec[0] == "struct_list":
      return [struct_value(spec[1], service) for _ in range(read("B"))]
    raise AssertionError(spec)

  def advance(count):
    nonlocal offset
    offset += count

  def struct_value(entries, service):
    return {entry[0]: field(entry[1], service, entry[0]) for entry in entries}

  service_id = frame[4]
  service = next(name for name, (sid, _) in SERVICE_SCHEMAS.items() if sid == service_id)
  return struct_value(schema, service)


def _approx(a, b):
  if isinstance(a, dict):
    return a.keys() == b.keys() and all(_approx(a[k], b[k]) for k in a)
  if isinstance(a, list):
    return len(a) == len(b) and all(_approx(x, y) for x, y in zip(a, b, strict=True))
  if isinstance(a, float) or isinstance(b, float):
    return math.isclose(float(a), float(b), rel_tol=1e-6, abs_tol=1e-6)
  return a == b


def _line(builder, xs, ys, zs=None):
  builder.x = xs
  builder.y = ys
  builder.z = zs or [0.0] * len(xs)


def _messages(lead_d=30.0, right_blindspot=False, alert_text="", v_cluster=20.0):
  xs = [float(i) * 3.0 for i in range(33)]
  model = log.ModelDataV2.new_message()
  model.frameId = 100
  _line(model.position, xs, [0.0] * 33, [0.0] * 33)
  lanes = model.init("laneLines", 4)
  for i, offset in enumerate((-5.4, -1.8, 1.8, 5.4)):
    _line(lanes[i], xs, [offset] * 33, [0.0] * 33)
  model.laneLineProbs = [0.2, 0.9, 0.9, 0.2]
  edges = model.init("roadEdges", 2)
  _line(edges[0], xs, [-7.0] * 33)
  _line(edges[1], xs, [7.0] * 33)
  model.roadEdgeStds = [0.3, 0.3]
  leads = model.init("leadsV3", 1)
  leads[0].prob = 0.9
  leads[0].x = [lead_d + 1.0]
  from openpilot.cereal import car as car_capnp
  car = car_capnp.CarState.new_message()
  car.vEgo = v_cluster
  car.vEgoCluster = v_cluster
  car.vCruiseCluster = 80.0
  car.rightBlindspot = right_blindspot
  car.gearShifter = "drive"
  radar = log.RadarState.new_message()
  radar.leadOne.status = True
  radar.leadOne.dRel = lead_d
  radar.leadOne.radar = True
  radar.leadOne.radarTrackId = 7
  radar.leadOne.vLeadK = 10.0
  center = radar.init("leadsCenter", 1)
  center[0].status = True
  center[0].dRel = lead_d
  center[0].radar = True
  center[0].vLeadK = 10.0
  calib = log.LiveCalibrationData.new_message()
  calib.calStatus = "calibrated"
  calib.rpyCalib = [0.0, 0.02, 0.0]
  camera = log.FrameData.new_message()
  camera.frameId = 100
  camera.sensor = "os04c10"
  device = log.DeviceState.new_message()
  device.deviceType = "tizi"
  device.cpuTempC = [60.0, 61.0]
  selfdrive = log.SelfdriveState.new_message()
  selfdrive.enabled = True
  selfdrive.personality = "standard"
  selfdrive.alertText1 = alert_text
  selfdrive.alertSize = "full" if alert_text else "none"
  lateral = log.LateralPlan.new_message()
  lateral.laneChangeState = "preLaneChange"
  lateral.laneChangeDirection = "right"
  controls = log.ControlsState.new_message()
  car_control = car_capnp.CarControl.new_message()
  car_control.longActive = True
  long_plan = log.LongitudinalPlan.new_message()
  long_plan.xState = 0
  from openpilot.cereal import custom
  carrot = custom.CarrotMan.new_message()
  live_params = log.LiveParametersData.new_message()
  return {
    "modelV2": model, "carState": car, "radarState": radar, "liveCalibration": calib, "roadCameraState": camera,
    "deviceState": device, "selfdriveState": selfdrive, "lateralPlan": lateral, "controlsState": controls,
    "carControl": car_control, "longitudinalPlan": long_plan, "carrotMan": carrot, "liveParameters": live_params,
  }


def test_inputs_match_the_compact_state_the_browser_decodes():
  for service, message in _messages(alert_text="TAKE CONTROL").items():
    reader = message.as_reader()
    _, schema = SERVICE_SCHEMAS[service]
    decoded = _decode_compact(encode_carrot_state_frame(service, reader, 1), schema)
    assert _approx(decoded, web_state_from_message(service, reader)), service
  assert set(WEB_VISION_SERVICES) == set(_messages())


def _inputs(now=10.0, **kwargs):
  inputs = WebVisionInputs()
  for service, message in _messages(**kwargs).items():
    inputs.update(service, message.as_reader(), now, True)
  return inputs


def _params(**values):
  raw = {"ShowLaneInfo": "2", "ShowPathMode": "9", "ShowRadarInfo": "1", "IsMetric": "1"}
  raw.update({k: str(v) for k, v in values.items()})
  return normalize_web_vision_params(raw)


def _build(overlay, inputs, now, params=None):
  snapshot = inputs.snapshot(now, params or _params())
  return overlay.build(snapshot, snapshot.params, VIDEO_W, VIDEO_H, STAGE_W, STAGE_H, now * 1000.0), snapshot


def test_settings_follow_the_web_server_snapshot():
  class FakeParams:
    def get_bool(self, name):
      assert name == "IsMetric"
      return False

    def get(self, name):
      if name in ("ShowPathEnd", "ShowPathWidth"):
        raise KeyError(name)  # not Params keys: the server sends None
      return {"ShowLaneInfo": 2, "ShowRadarInfo": 3, "RadarLatFactor": 40}.get(name)

  raw = read_web_vision_raw_params(FakeParams())
  assert raw["IsMetric"] == "0" and raw["ShowPathEnd"] is None and raw["ShowLaneInfo"] == "2"
  params = normalize_web_vision_params(raw)
  assert params["IsMetric"] == 0 and params["ShowRadarInfo"] == 3 and params["RadarLatFactor"] == 40
  assert params["ShowPathWidth"] == 100 and params["ShowPathEnd"] == 0  # web defaultParams
  assert normalize_web_vision_params({"ShowLaneInfo": None}, params)["ShowLaneInfo"] == 2


def test_non_model_inputs_update_while_model_input_is_stopped():
  overlay = WebVisionOverlay("ko")
  inputs = _inputs()
  frame, _ = _build(overlay, inputs, 10.05)
  assert frame.model_drawn and overlay.geometry_builds == 1
  base_live = len(frame.triangles)
  labels = [i.text for i in frame.canvas if isinstance(i, Label)]
  assert "30" in labels  # radar distance badge

  # Blind spot: barrier appears with no new modelV2 and no model geometry rebuild.
  inputs.update("carState", _messages(right_blindspot=True)["carState"].as_reader(), 10.1)
  frame, _ = _build(overlay, inputs, 10.1)
  assert overlay.geometry_builds == 1 and frame.model_drawn
  assert len(frame.triangles) > base_live

  # Radar: the lead badge follows radarState immediately. The lead distance
  # also bounds the drawn lane length, exactly like the web.
  inputs.update("radarState", _messages(lead_d=45.0)["radarState"].as_reader(), 10.15)
  frame, _ = _build(overlay, inputs, 10.15)
  assert "45" in [i.text for i in frame.canvas if isinstance(i, Label)]
  assert overlay.geometry_builds == 2

  # Display settings: the radar speed badge follows ShowRadarInfo.
  assert "36" in [i.text for i in frame.canvas if isinstance(i, Label)]
  hidden, _ = _build(overlay, inputs, 10.16, _params(ShowRadarInfo=0))
  assert "36" not in [i.text for i in hidden.canvas if isinstance(i, Label)]

  # Speed and alert text come from carState/selfdriveState only.
  inputs.update("carState", _messages(v_cluster=25.0, right_blindspot=True)["carState"].as_reader(), 10.2)
  inputs.update("selfdriveState", _messages(alert_text="TAKE CONTROL")["selfdriveState"].as_reader(), 10.2)
  snapshot = inputs.snapshot(10.2, _params())
  assert derive_web_hud_data(snapshot, True, CruiseOverrideHold(), 10200.0).speed == 90
  alert = web_alert_layout(snapshot.services["selfdriveState"], STAGE_W, STAGE_H, overlay.base_camera)
  assert alert is not None and alert.text1 == "TAKE CONTROL"

  # Calibration: projection changes at once and rebuilds the model shapes.
  before = frame.transform.calib
  builds = overlay.geometry_builds
  calib = _messages()["liveCalibration"]
  calib.rpyCalib = [0.0, 0.03, 0.01]
  inputs.update("liveCalibration", calib.as_reader(), 10.25)
  frame, _ = _build(overlay, inputs, 10.25)
  assert frame.transform.calib != before and overlay.geometry_builds == builds + 1


def test_stale_or_invalid_model_is_not_drawn_as_current():
  overlay = WebVisionOverlay("ko")
  inputs = _inputs(now=10.0)
  frame, _ = _build(overlay, inputs, 10.0 + MODEL_STALE_SECONDS + 0.1)
  assert not frame.model_drawn and frame.model_hidden_reason == "stale-model"
  assert frame.model_triangles == []
  # Radar-driven lead boxes keep updating from radarState.
  assert any(isinstance(i, Label) and i.text == "30" for i in frame.canvas)

  inputs.update("modelV2", _messages()["modelV2"].as_reader(), 11.0, valid=False)
  frame, _ = _build(overlay, inputs, 11.0)
  assert not frame.model_drawn and frame.model_hidden_reason == "invalid-model"

  inputs.update("modelV2", _messages()["modelV2"].as_reader(), 11.1, valid=True)
  frame, _ = _build(overlay, inputs, 11.1)
  assert frame.model_drawn and frame.model_triangles


def test_hud_data_follows_the_web_payload_rules():
  snapshot = _inputs().snapshot(10.0, _params())
  data = derive_web_hud_data(snapshot, True, CruiseOverrideHold(), 10000.0)
  assert (data.speed, data.set_speed, data.gap, data.gear) == (72, 80, 2, "D")
  assert data.cpu_temp == pytest.approx(60.5)  # web forwards the averaged temperature
  imperial = derive_web_hud_data(snapshot, False, CruiseOverrideHold(), 10000.0)
  assert (imperial.speed, imperial.set_speed) == (45, 50)


def test_hud_layout_matches_the_css_for_the_hud_viewport():
  # Boxes measured with the real CSS in Chromium at 1124x480 (testwork comparison).
  layout = web_hud_layout(STAGE_W, STAGE_H, 200.0, False, False)
  assert layout.speed == pytest.approx((30.0, 297.08, 337.2, 122.94), abs=0.05)
  assert layout.limit == pytest.approx((38.0, 106.0, 84.0, 84.0))
  assert layout.lfa == pytest.approx((28.0, 28.0, 56.0, 56.0))
  assert layout.accel == pytest.approx((947.12, 28.0, 67.44, 183.83), abs=0.05)
  assert layout.steer == pytest.approx((1028.56, 28.0, 67.44, 183.83), abs=0.05)
  assert layout.tpms == pytest.approx((852.0, 348.0, 120.0, 102.0))


def test_live_source_collects_web_vision_inputs_only_for_view_five():
  cluster_live = pytest.importorskip("cluster_live")
  messages = {name: message.as_reader() for name, message in _messages().items()}

  class FakeSubMaster:
    def __init__(self):
      self.data = dict(messages)
      self.recv_time = {name: 50.0 for name in messages}
      self.recv_frame = {name: 1 for name in messages}
      self.alive = {name: True for name in messages}
      self.valid = {name: True for name in messages}
      self.updated = {name: False for name in messages}

    def __getitem__(self, name):
      return self.data[name]

  class FakeParams:
    def get_bool(self, name):
      return True

    def get(self, name):
      return {"ShowRadarInfo": 2}.get(name)

  source = object.__new__(cluster_live.OpenpilotLiveSource)
  source.sm = FakeSubMaster()
  source.params = FakeParams()
  source._web_vision_inputs = None
  source._web_vision_params = None
  source._next_web_vision_param_read_t = 0.0
  state = cluster_live.standby_state()
  assert source._with_web_vision(state) is state

  source.set_web_vision_enabled(True)
  snapshot = source._with_web_vision(state).web_vision
  assert set(snapshot.services) == set(WEB_VISION_SERVICES)
  assert snapshot.params["ShowRadarInfo"] == 2 and snapshot.params["IsMetric"] == 1

  source.sm.data["carState"] = _messages(v_cluster=30.0)["carState"].as_reader()
  source.sm.recv_time["carState"] = 51.0
  source._update_web_vision_service("carState")
  assert source._with_web_vision(state).web_vision.services["carState"]["vEgoCluster"] == 30.0

  source.set_web_vision_enabled(False)
  assert source._with_web_vision(state).web_vision is None


# ---------------------------------------------------------------------------
# Model geometry follows the shown camera frame (vision_raw.js frame sync)
# ---------------------------------------------------------------------------

def _model_for_frame(frame_id, lane_offset):
  model = _messages()["modelV2"]
  model.frameId = frame_id
  xs = [float(i) * 3.0 for i in range(33)]
  _line(model.position, xs, [lane_offset] * 33, [0.0] * 33)
  return model.as_reader()


def _inputs_with_models(frame_ids, now=10.0):
  inputs = _inputs(now=now)
  for i, frame_id in enumerate(frame_ids):
    inputs.update("modelV2", _model_for_frame(frame_id, 0.01 * frame_id), now + 0.05 * (i + 1), True)
  return inputs


def test_model_is_the_newest_at_or_before_the_shown_frame():
  inputs = _inputs_with_models(range(101, 111))  # the newest model is frame 110
  snapshot = inputs.snapshot(10.6, _params())

  def selected(frame_id):
    return snapshot.for_presented_frame(frame_id).services["modelV2"]["frameId"]

  # Video behind the model (the model is ahead): use the model of the shown frame.
  assert selected(105) == 105
  assert selected(101) == 101
  # Video ahead of the model: the newest model is the latest at or before it.
  assert selected(115) == 110
  # A dropped model frame uses the previous one, never a later one.
  dropped = _inputs_with_models([101, 102, 103, 105]).snapshot(10.3, _params())
  assert dropped.for_presented_frame(104).services["modelV2"]["frameId"] == 103
  # Older than every remembered model (frame 100 from _inputs is the oldest): the oldest one, like the web.
  assert selected(90) == 100
  # Unknown shown frame: the newest model.
  assert selected(None) == 110
  assert select_synchronized_model((), 105) is None


def test_frame_sync_replaces_only_the_model_input():
  inputs = _inputs_with_models(range(101, 111))
  # Other inputs keep arriving after the newest model.
  inputs.update("radarState", _messages(lead_d=45.0)["radarState"].as_reader(), 10.7)
  inputs.update("carState", _messages(v_cluster=25.0, right_blindspot=True)["carState"].as_reader(), 10.7)
  inputs.update("selfdriveState", _messages(alert_text="TAKE CONTROL")["selfdriveState"].as_reader(), 10.7)
  latest = inputs.snapshot(10.7, _params())
  synced = latest.for_presented_frame(103)
  assert synced.services["modelV2"]["frameId"] == 103
  for service in WEB_VISION_SERVICES:
    if service != "modelV2":
      assert synced.services[service] is latest.services[service], service
  assert synced.services["radarState"]["leadOne"]["dRel"] == 45.0
  assert synced.services["selfdriveState"]["alertText1"] == "TAKE CONTROL"
  # Model stream liveness still follows the newest model.
  assert synced.model_age_s == latest.model_age_s and synced.model_usable()


def test_overlay_draws_the_selected_model_geometry():
  inputs = _inputs_with_models(range(101, 111))
  snapshot = inputs.snapshot(10.6, _params())
  synced = snapshot.for_presented_frame(104)
  frame = WebVisionOverlay("ko").build(synced, synced.params, VIDEO_W, VIDEO_H, STAGE_W, STAGE_H, 10600.0)

  only = _inputs(now=10.0)
  only.update("modelV2", _model_for_frame(104, 1.04), 10.2, True)
  only_snapshot = only.snapshot(10.6, _params())
  expected = WebVisionOverlay("ko").build(only_snapshot, only_snapshot.params, VIDEO_W, VIDEO_H, STAGE_W, STAGE_H,
                                          10600.0)
  latest = WebVisionOverlay("ko").build(snapshot, snapshot.params, VIDEO_W, VIDEO_H, STAGE_W, STAGE_H, 10600.0)
  assert frame.model_drawn and frame.model_triangles == expected.model_triangles
  assert frame.model_triangles != latest.model_triangles

  # A new shown frame rebuilds the geometry without a new model message.
  overlay = WebVisionOverlay("ko")
  overlay.build(snapshot.for_presented_frame(104), snapshot.params, VIDEO_W, VIDEO_H, STAGE_W, STAGE_H, 10600.0)
  overlay.build(snapshot.for_presented_frame(104), snapshot.params, VIDEO_W, VIDEO_H, STAGE_W, STAGE_H, 10650.0)
  assert overlay.geometry_builds == 1
  overlay.build(snapshot.for_presented_frame(105), snapshot.params, VIDEO_W, VIDEO_H, STAGE_W, STAGE_H, 10700.0)
  assert overlay.geometry_builds == 2


def test_model_history_matches_the_web_limit_and_same_frame_rule():
  inputs = _inputs_with_models(range(1, MODEL_FRAME_HISTORY_LIMIT + 11))
  history = inputs.snapshot(20.0).model_history
  assert len(history) == MODEL_FRAME_HISTORY_LIMIT
  assert history[0].frame_id == 11 and history[-1].frame_id == MODEL_FRAME_HISTORY_LIMIT + 10
  inputs.update("modelV2", _model_for_frame(MODEL_FRAME_HISTORY_LIMIT + 10, 9.0), 21.0, False)
  history = inputs.snapshot(21.0).model_history
  assert len(history) == MODEL_FRAME_HISTORY_LIMIT and history[-1].valid is False
  inputs.reset()
  assert inputs.snapshot(22.0).model_history == ()


# ---------------------------------------------------------------------------
# The second lead box erases what it covers (eraseLeadBoxOcclusion)
# ---------------------------------------------------------------------------

def _lead_two_frame(lead_two_y):
  messages = _messages(lead_d=18.0)
  radar = messages["radarState"]
  radar.leadTwo.status = True
  radar.leadTwo.radar = True
  radar.leadTwo.dRel = 28.0
  radar.leadTwo.yRel = lead_two_y
  radar.leadTwo.radarTrackId = 9
  inputs = WebVisionInputs()
  for service, message in messages.items():
    inputs.update(service, message.as_reader(), 10.0, True)
  overlay = WebVisionOverlay("ko")
  frame, _ = _build(overlay, inputs, 10.0)
  return overlay, frame


def test_second_lead_box_erases_the_earlier_canvas_it_covers():
  overlay, frame = _lead_two_frame(0.5)
  kinds = [type(item) for item in frame.canvas]
  erases = [i for i, item in enumerate(frame.canvas) if isinstance(item, EraseRoundRect)]
  # Web order: erase+card for lead one, its badges, then erase+card for lead two.
  assert len(erases) == 2 and erases[0] == 0
  second = frame.canvas[erases[1]]
  card = frame.canvas[erases[1] + 1]
  assert isinstance(card, RoundRect) and (card.x, card.y, card.width, card.height) == (
    second.x, second.y, second.width, second.height)
  assert second.radius == max(12 * overlay.ui_scale(VIDEO_W, VIDEO_H), 6)
  assert Label in kinds[:erases[1]]  # lead one distance badge drawn before the erase
  assert canvas_erase_hides_items(frame.canvas, overlay.measure_text)

  # Far apart, the erase does not touch anything and changes nothing.
  overlay, apart = _lead_two_frame(12.0)
  assert sum(isinstance(item, EraseRoundRect) for item in apart.canvas) == 2
  assert not canvas_erase_hides_items(apart.canvas, overlay.measure_text)
  # The first lead's own erase comes before any canvas item.
  overlay = WebVisionOverlay("ko")
  single, _ = _build(overlay, _inputs(), 10.0)
  assert isinstance(single.canvas[0], EraseRoundRect)
  assert not canvas_erase_hides_items(single.canvas, overlay.measure_text)


# ---------------------------------------------------------------------------
# Safety alerts and the unsupported route replay
# ---------------------------------------------------------------------------

def _render_probe(cluster_renderer):
  renderer = object.__new__(cluster_renderer.ClusterUiRenderer)
  renderer.screen_mode = cluster_renderer.CLUSTER_SCREEN_MODE_DEFAULT
  renderer.profile_enabled = False
  calls = []
  renderer._render_world = lambda state, lights: calls.append(("world", renderer._web_vision_view_active(state)))
  renderer._draw_hud = lambda state, lights: None
  renderer._draw_web_vision_hud = lambda state, draw_alert=True: calls.append(("web_hud", draw_alert))
  renderer._draw_alert_overlay = lambda alert: calls.append(("alert", alert.alert_type if alert else None))
  return renderer, calls


def test_hud_safety_alerts_take_priority_in_view_five(monkeypatch):
  cluster_live = pytest.importorskip("cluster_live")
  cluster_renderer = pytest.importorskip("cluster_renderer")
  from dataclasses import replace

  source = object.__new__(cluster_live.OpenpilotLiveSource)
  source._alert_onroad = False
  source.sm = type("SM", (), {"recv_time": {}})()
  updated = {"value": False}
  source._service_updated = lambda _service: updated["value"]
  source._service_data = lambda _service: type("S", (), {"enabled": True})()
  clock = {"t": 90.0}
  monkeypatch.setattr(cluster_live.time, "monotonic", lambda: clock["t"])
  web_snapshot = _inputs(alert_text="TAKE CONTROL").snapshot(10.0, _params())
  renderer, calls = _render_probe(cluster_renderer)

  def step(t, mode=5):
    clock["t"] = t
    alert = source._live_cluster_alert(None, True)
    state = replace(cluster_live.standby_state(), onroad=True, camera_view_mode=mode, alert=alert,
                    web_vision=web_snapshot)
    calls.clear()
    renderer.render(state, (False, False))
    return alert.alert_type if alert else None, list(calls)

  assert step(90.0) == (None, [("world", True), ("web_hud", True), ("alert", None)])
  # No selfdriveState since onroad started: "Waiting to start" in view 5 too.
  assert step(95.5) == ("clusterSelfdriveStartup",
                        [("world", True), ("web_hud", False), ("alert", "clusterSelfdriveStartup")])
  # Input arrives: the web alert (from selfdriveState) is drawn again.
  source.sm.recv_time["selfdriveState"] = 96.0
  updated["value"] = True
  assert step(96.0) == (None, [("world", True), ("web_hud", True), ("alert", None)])
  updated["value"] = False
  # Input stops while engaged: take control immediately, then reboot.
  assert step(101.5) == ("clusterSelfdriveTimeout",
                         [("world", True), ("web_hud", False), ("alert", "clusterSelfdriveTimeout")])
  # Changing the camera view keeps the same alert on every mode.
  for mode in (2, 3, 5, 0, 5):
    alert_type, drawn = step(101.6, mode)
    assert alert_type == "clusterSelfdriveTimeout" and ("alert", "clusterSelfdriveTimeout") in drawn, mode
    assert (("web_hud", False) in drawn) == (mode == 5), mode
  assert step(111.5)[0] == "clusterSelfdriveReboot"
  assert step(111.5)[1][-1] == ("alert", "clusterSelfdriveReboot")
  # Input recovers: the HUD alert clears and the web alert path returns.
  source.sm.recv_time["selfdriveState"] = 112.0
  assert step(112.1) == (None, [("world", True), ("web_hud", True), ("alert", None)])


def test_route_replay_view_five_draws_the_existing_camera_view():
  cluster_renderer = pytest.importorskip("cluster_renderer")
  from dataclasses import replace
  from cluster_models import ClusterAlert, RouteOverlay
  from cluster_live import standby_state

  renderer, calls = _render_probe(cluster_renderer)
  overlay = RouteOverlay(video_rgba=bytes(4 * 4 * 4), video_width=4, video_height=4, video_frame_id="seg:1",
                         camera_stream="road", panel_visible=False)
  alert = ClusterAlert("Steering Unavailable", "Take Control", size=2, status=1, alert_type="steerUnavailable")
  replay = replace(standby_state(), onroad=True, camera_view_mode=5, route_overlay=overlay, alert=alert)
  assert replay.web_vision is None
  renderer.render(replay, (False, False))
  # No web vision inputs: the road camera view and its alert, not an empty stage.
  assert calls == [("world", False), ("alert", "steerUnavailable")]
  assert cluster_camera_view_is_road_camera(5) and not cluster_camera_view_prefers_wide(5, 10.0, False)
  assert "not supported for --input route" in cluster_web_vision_unsupported_notice(5, "route")
  assert cluster_web_vision_unsupported_notice(5, "live") is None
  assert cluster_web_vision_unsupported_notice(2, "route") is None
