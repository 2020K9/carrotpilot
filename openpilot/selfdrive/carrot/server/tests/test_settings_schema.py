"""Guards the fields the settings UI now reads straight from the catalogue.

Display units and control kinds used to live in hardcoded tables inside
web/js/pages/setting.js, so adding a parameter meant editing two places and
forgetting one was silent. They are declared per parameter now; these tests
keep the declarations well formed.
"""
import json
from pathlib import Path

import pytest

from openpilot.selfdrive.carrot.server.services.settings import (
  build_menu_categories,
  current_vehicle_brand,
  filter_settings_catalog_for_brand,
  group_index,
  with_vehicle_gap_limits,
)

SETTINGS_PATH = Path(__file__).resolve().parents[3] / "carrot_settings.json"
PARAMS_KEYS_PATH = Path(__file__).resolve().parents[4] / "common" / "params_keys.h"

# Mirrors SETTING_DISPLAY_UNIT_TYPES / SETTING_CONTROL_KINDS in setting.js.
KNOWN_DISPLAY_UNITS = {"raw", "speedKph", "distanceCm", "timeSec", "timeMs", "timeMin", "percent", "degree"}
KNOWN_CONTROL_KINDS = {"toggle", "segmented", "select", "slider"}
KNOWN_RISK_LEVELS = {"high", "medium"}


@pytest.fixture(scope="module")
def settings():
  with SETTINGS_PATH.open(encoding="utf-8") as f:
    return json.load(f)


@pytest.fixture(scope="module")
def params(settings):
  return settings["params"]


def test_the_catalogue_is_readable_and_populated(params):
  assert len(params) > 100
  assert all(isinstance(p.get("name"), str) and p["name"] for p in params)


@pytest.mark.parametrize("maximum", (3, 4))
def test_gap_cycle_catalog_uses_vehicle_maximum_without_mutating_cache(settings, maximum):
  groups, by_name, groups_list = group_index(settings)
  original = by_name["CruiseGapLevels"]
  data, adapted_groups, adapted_names, _ = with_vehicle_gap_limits((settings, groups, by_name, groups_list), maximum)
  selected = adapted_names["CruiseGapLevels"]
  assert (selected["min"], selected["max"], selected["default"]) == (2, maximum, maximum)
  assert all(len(options) == maximum - 1 for options in selected["options"].values())
  from openpilot.selfdrive.carrot.server.services.params import clamp_numeric
  assert clamp_numeric(4, selected) == maximum
  assert clamp_numeric(1, selected) == 2
  assert next(p for p in data["params"] if p["name"] == "CruiseGapLevels") is selected
  assert next(p for p in adapted_groups[selected["group"]] if p["name"] == "CruiseGapLevels") is selected
  assert (original["max"], original["default"]) == (4, 4)
  assert all(len(options) == 3 for options in original["options"].values())


@pytest.mark.parametrize("maximum,stored,expected", ((3, 4, 3), (4, 4, 4), (3, 2, 2), (4, 3, 3), (3, 0, 3)))
def test_gap_cycle_read_matches_vehicle_limit(monkeypatch, maximum, stored, expected):
  from openpilot.selfdrive.carrot.server.services import params as service
  class FakeParams:
    def get_int(self, key):
      return maximum
  monkeypatch.setattr(service, "HAS_PARAMS", True)
  monkeypatch.setattr(service, "Params", FakeParams)
  monkeypatch.setattr(service, "_read_param_value", lambda params, name, default: stored)
  assert service.get_param_values(["CruiseGapLevels"], {"CruiseGapLevels": maximum}) == {"CruiseGapLevels": expected}


def test_gap_cycle_setting_is_registered_and_in_gap_menu(settings):
  assert '{"CruiseGapLevels", {PERSISTENT, INT, "4"}}' in PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  groups, by_name, _ = group_index(settings)
  categories = build_menu_categories(settings, by_name)
  section = next(section for category in categories for group in category["groups"] for section in group["sections"]
                 if "TFollowGap1" in section["items"])
  assert section["items"].index("CruiseGapLevels") < section["items"].index("TFollowGap1")


def test_rear_camera_hold_uses_meter_values_and_persistent_default(settings, params):
  hold = next(p for p in params if p["name"] == "AutoNaviRearCameraHoldDistance")
  assert (hold["min"], hold["max"], hold["default"], hold["unit"]) == (0, 300, 100, 10)
  assert '"AutoNaviRearCameraHoldDistance", {PERSISTENT, INT, "100"}' in PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  driving = next(category for category in settings["menu"] if category["id"] == "DRIVING")
  speed = next(group for group in driving["groups"] if group["id"] == "SPEED")
  cameras = next(group for group in speed["groups"] if group["id"] == "SPEED_CAMERA")
  assert hold["name"] in cameras["params"]


def test_onnx_vision_toggle_uses_existing_runtime_flag_and_first_steering_section(settings, params):
  by_name = {p["name"]: p for p in params}
  vision = by_name["ShareData"]
  assert (vision["control"], vision["min"], vision["max"], vision["default"]) == ("toggle", 0, 1, 0)
  assert vision["risk"] == "high"
  assert '{"ShareData", {PERSISTENT, INT, "0"}}' in PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  driving = next(category for category in settings["menu"] if category["id"] == "DRIVING")
  steering = next(group for group in driving["groups"] if group["id"] == "STEER")
  onnx = steering["groups"][0]
  lane_change = next(group for group in steering["groups"] if group["id"] == "STEER_LANECHANGE")
  assert onnx["id"] == "STEER_ONNX"
  detail_names = [
    "OnnxBsdEnabled",
    "OnnxLaneThreshold", "OnnxLaneIntervalMs", "OnnxBsdThreshold",
    "OnnxBsdSmoothingMs", "OnnxBsdIntervalMs",
  ]
  assert onnx["params"] == ["ShareData", *detail_names]
  assert "ShareData" not in lane_change["params"]
  for name in detail_names:
    assert by_name[name]["detail_parent"] == "ShareData"
    assert by_name[name]["detail_section"] in ("lane", "bsd")
    assert f'{{"{name}", {{PERSISTENT, INT,' in PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  bsd_enabled = by_name["OnnxBsdEnabled"]
  assert (bsd_enabled["control"], bsd_enabled["min"], bsd_enabled["max"], bsd_enabled["default"]) == ("toggle", 0, 1, 0)
  for title in ("title", "etitle", "ctitle"):
    assert "ONNX" in vision[title]


def test_obsolete_lead_response_settings_are_removed(settings, params):
  removed = {"JLeadFactor3", "RadarReactionFactor"}
  by_name = {p["name"] for p in params}
  assert removed.isdisjoint(by_name)
  assert all(
    removed.isdisjoint(group.get("params", []))
    for category in settings["menu"]
    for section in category.get("groups", [])
    for group in section.get("groups", [])
  )
  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert all(name not in params_keys for name in removed)


def test_automatic_driving_mode_exposes_manual_normal_and_eco_choices(params):
  by_name = {p["name"]: p for p in params}
  automatic = by_name["MyDrivingModeAuto"]
  assert (automatic["min"], automatic["max"], automatic["default"]) == (0, 2, 0)
  assert "1:일반↔안전" in automatic["descr"]
  assert "2:에코↔안전" in automatic["descr"]


def test_longitudinal_comfort_settings_use_driver_facing_language(params):
  by_name = {p["name"]: p for p in params}

  assert "DynamicTFollow" not in by_name

  lead_accel_response = by_name["LeadAccelResponse"]
  assert (lead_accel_response["min"], lead_accel_response["max"], lead_accel_response["default"]) == (0, 5, 0)
  assert lead_accel_response["control"] == "select"
  assert all(lead_accel_response[key].strip() for key in ("descr", "edescr", "cdescr"))
  assert lead_accel_response["options"]["ko"][3] == "3 균형 추종"
  assert lead_accel_response["options"]["ko"][-1] == "5 최대 추종(시험)"

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"LeadAccelResponse", {PERSISTENT, INT, "0"}}' in params_keys
  assert '{"DynamicTFollow",' not in params_keys

  lane_change = by_name["DynamicTFollowLC"]
  assert lane_change["default"] == 100
  assert "100은 추가 완화 없음" in lane_change["descr"]

  driving_mode = by_name["MyDrivingMode"]
  assert "ComfortBrake" not in driving_mode["descr"]
  assert "멀리서부터 천천히 감속" in driving_mode["descr"]


def test_deceleration_gap_margin_defaults_match_registry(params):
  decel_margin = next(p for p in params if p["name"] == "TFollowDecelBoost")
  assert decel_margin["default"] == 0
  assert "목표 간격" in decel_margin["descr"]
  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"TFollowDecelBoost", {PERSISTENT, INT, "0"}}' in params_keys


def test_longitudinal_pid_defaults_match_registry(params):
  by_name = {p["name"]: p for p in params}
  assert tuple(by_name[name]["default"] for name in (
    "LongTuningKpV", "LongTuningKiV", "LongTuningKf",
  )) == (100, 0, 100)
  for name in ("LongTuningKpV", "LongTuningKiV", "LongTuningKf"):
    assert by_name[name]["hidden_brands"] == ["hyundai", "toyota"]
    assert "Toyota/Lexus" in by_name[name]["edescr"]
    assert "현대·기아·제네시스" in by_name[name]["descr"]
    assert "hidden and ignored" in by_name[name]["edescr"]

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  for name, default in (
    ("LongTuningKpV", 100),
    ("LongTuningKiV", 0),
    ("LongTuningKf", 100),
  ):
    assert f'{{"{name}", {{PERSISTENT, INT, "{default}"}}}}' in params_keys


@pytest.mark.parametrize("brand", ["hyundai", "toyota"])
def test_fixed_tuning_catalog_hides_longitudinal_pid_settings(settings, brand):
  groups, by_name, groups_list = group_index(settings)
  categories = build_menu_categories(settings, by_name)
  hidden = {"LongTuningKpV", "LongTuningKiV", "LongTuningKf"}

  filtered_groups, filtered_groups_list, filtered_categories, hidden_names = filter_settings_catalog_for_brand(
    groups, groups_list, categories, brand,
  )
  visible_names = {item["name"] for items in filtered_groups.values() for item in items}
  menu_names = {
    name
    for category in filtered_categories
    for group in category["groups"]
    for section in group["sections"]
    for name in section["items"]
  }

  assert hidden_names == hidden
  assert hidden.isdisjoint(visible_names)
  assert hidden.isdisjoint(menu_names)
  assert all(
    group["count"] == sum(1 for item in filtered_groups[group["group"]]
                          if not item.get("detail_parent") and not item.get("search_only"))
    for group in filtered_groups_list
  )


def test_other_brands_keep_longitudinal_pid_settings(settings):
  groups, by_name, groups_list = group_index(settings)
  categories = build_menu_categories(settings, by_name)
  filtered_groups, _filtered_groups_list, _filtered_categories, hidden_names = filter_settings_catalog_for_brand(
    groups, groups_list, categories, "gm",
  )
  visible_names = {item["name"] for items in filtered_groups.values() for item in items}

  assert hidden_names == set()
  assert {"LongTuningKpV", "LongTuningKiV", "LongTuningKf"} <= visible_names


@pytest.mark.parametrize("car_name", [
  b"HYUNDAI_IONIQ_5_PE",
  "KIA_EV6",
  "GENESIS_G80_2ND_GEN_FL",
])
def test_vehicle_brand_falls_back_to_hkg_car_name(car_name):
  class FakeParams:
    def get(self, name):
      return {"CarParamsPersistent": None, "CarName": car_name}.get(name)

  assert current_vehicle_brand(FakeParams()) == "hyundai"


def test_vehicle_brand_does_not_hide_settings_for_other_car_name():
  class FakeParams:
    def get(self, name):
      return {"CarParamsPersistent": None, "CarName": "TOYOTA_RAV4"}.get(name)

  assert current_vehicle_brand(FakeParams()) == ""


def test_c3x_lite_hardware_setting_is_exposed(settings, params):
  by_name = {p["name"]: p for p in params}
  c3x_lite = by_name["HardwareC3xLite"]
  assert (c3x_lite["min"], c3x_lite["max"], c3x_lite["default"]) == (0, 1, 0)
  assert c3x_lite["control"] == "toggle"
  assert c3x_lite["risk"] == "high"

  vehicle = next(category for category in settings["menu"] if category["id"] == "VEHICLE")
  device_hardware = next(group for group in vehicle["groups"] if group["id"] == "VEH_DEVICE")
  assert device_hardware["params"] == ["HardwareC3xLite"]


def test_wide_camera_fallback_setting_is_exposed(settings, params):
  by_name = {p["name"]: p for p in params}
  use_wide_camera = by_name["UseWideCamera"]
  assert (use_wide_camera["min"], use_wide_camera["max"], use_wide_camera["default"]) == (0, 1, 1)
  assert use_wide_camera["control"] == "toggle"
  assert use_wide_camera["risk"] == "high"
  assert "재부팅" in use_wide_camera["descr"]
  assert "Reboot" in use_wide_camera["edescr"]

  system = next(category for category in settings["menu"] if category["id"] == "SYSTEM")
  camera = next(group for group in system["groups"] if group["id"] == "SYS_CAMERA")
  assert camera["params"] == ["UseWideCamera"]

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"UseWideCamera", {PERSISTENT, BOOL, "1"}}' in params_keys


def test_driver_monitoring_setting_is_default_on_search_only_and_session_state_is_internal(settings, params):
  by_name = {p["name"]: p for p in params}
  enabled = by_name["DriverMonitoringEnabled"]
  assert (enabled["min"], enabled["max"], enabled["default"]) == (0, 1, 1)
  assert enabled["control"] == "toggle"
  assert enabled["risk"] == "high"
  assert enabled["search_only"] is True
  assert "DM 카메라" in enabled["descr"] and "법규" in enabled["descr"]
  assert "DM camera" in enabled["edescr"] and "laws" in enabled["edescr"]
  assert "DriverMonitoringSessionDisabled" not in by_name

  vehicle = next(category for category in settings["menu"] if category["id"] == "VEHICLE")
  driver_monitoring = next(group for group in vehicle["groups"] if group["id"] == "VEH_DM")
  assert driver_monitoring["params"][:2] == ["DriverMonitoringEnabled", "DriverMonitoringMode"]

  groups, indexed, _ = group_index(settings)
  categories = build_menu_categories(settings, indexed)
  driver_monitoring_view = next(
    group for category in categories for group in category["groups"] if group["id"] == "VEH_DM"
  )
  assert "DriverMonitoringEnabled" in driver_monitoring_view["sections"][0]["items"]
  assert driver_monitoring_view["count"] == sum(
    1 for name in driver_monitoring["params"]
    if not indexed[name].get("detail_parent") and not indexed[name].get("search_only")
  )
  assert enabled in groups[enabled["group"]]

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"DriverMonitoringEnabled", {PERSISTENT, BOOL, "1"}}' in params_keys
  assert '{"DriverMonitoringSessionDisabled", {CLEAR_ON_MANAGER_START | CLEAR_ON_IGNITION_ON, BOOL}}' in params_keys


def test_vehicle_navi_can_control_exposes_route_filter_modes(settings, params):
  by_name = {p["name"]: p for p in params}
  control = by_name["VehicleNaviCanControl"]
  assert (control["min"], control["max"], control["default"]) == (0, 3, 0)
  assert control["control"] == "select"
  assert all(len(control["options"][locale]) == 4 for locale in ("ko", "en", "zh"))
  assert control["risk"] == "high"
  assert "calculated route" in control["edescr"]
  assert "VehicleSpeedCameraControlMode" in control["edescr"]

  driving = next(category for category in settings["menu"] if category["id"] == "DRIVING")
  speed = next(group for group in driving["groups"] if group["id"] == "SPEED")
  camera = next(group for group in speed["groups"] if group["id"] == "SPEED_CAMERA")
  assert "VehicleNaviCanControl" in camera["params"]

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"VehicleNaviCanControl", {PERSISTENT, INT, "0"}}' in params_keys


def test_speed_bump_early_release_distance_is_exposed_in_centimeters(settings, params):
  by_name = {p["name"]: p for p in params}
  distance = by_name["AutoNaviSpeedBumpEndDistance"]
  assert (distance["min"], distance["max"], distance["default"], distance["unit"]) == (0, 5000, 200, 10)
  assert distance["display_unit"] == "distanceCm"
  assert distance["risk"] == "high"

  driving = next(category for category in settings["menu"] if category["id"] == "DRIVING")
  speed = next(group for group in driving["groups"] if group["id"] == "SPEED")
  bump = next(group for group in speed["groups"] if group["id"] == "SPEED_BUMP")
  assert "AutoNaviSpeedBumpEndDistance" in bump["params"]

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"AutoNaviSpeedBumpEndDistance", {PERSISTENT, INT, "200"}}' in params_keys


def test_vehicle_navi_school_zone_control_is_opt_in(settings, params):
  by_name = {p["name"]: p for p in params}
  control = by_name["VehicleNaviSchoolZoneControl"]
  assert (control["min"], control["max"], control["default"]) == (0, 1, 0)
  assert control["control"] == "toggle"
  assert control["risk"] == "high"
  assert "PV5에서는 아직 동작하지 않습니다" in control["descr"]
  assert "not yet supported on the PV5" in control["edescr"]

  driving = next(category for category in settings["menu"] if category["id"] == "DRIVING")
  speed = next(group for group in driving["groups"] if group["id"] == "SPEED")
  camera = next(group for group in speed["groups"] if group["id"] == "SPEED_CAMERA")
  assert "VehicleNaviSchoolZoneControl" in camera["params"]

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"VehicleNaviSchoolZoneControl", {PERSISTENT, BOOL, "0"}}' in params_keys


def test_speedcam_decel_cancel_and_section_average_settings_are_opt_in(settings, params):
  by_name = {p["name"]: p for p in params}
  master, children = "VehicleNaviDecelCancel", ("VehicleNaviDecelCancelBox", "VehicleNaviDecelCancelMobileZone",
                                                 "VehicleNaviDecelCancelBump")
  defaults = {master: 0, "VehicleNaviSectionAvgControl": 0, **{name: 1 for name in children},
              "VehicleNaviDecelCancelUnknown": 0,  # experimental: kind unknown, opt-in
              "VehicleNaviDecelCancelEarlyWarning": 0}  # experimental: wider lead window, opt-in
  for name, default in defaults.items():
    control = by_name[name]
    assert (control["min"], control["max"], control["default"]) == (0, 1, default)
    assert control["control"] == "toggle"
    assert control["risk"] == "high"
  # Per-kind toggles live in the master's detail screen; only the master is opt-in.
  assert not by_name[master].get("detail_parent")
  assert all(by_name[name]["detail_parent"] == master and by_name[name]["group"] == by_name[master]["group"]
             for name in (*children, "VehicleNaviDecelCancelUnknown",
                          "VehicleNaviDecelCancelEarlyWarning"))

  driving = next(category for category in settings["menu"] if category["id"] == "DRIVING")
  speed = next(group for group in driving["groups"] if group["id"] == "SPEED")
  camera = next(group for group in speed["groups"] if group["id"] == "SPEED_CAMERA")
  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  for name, default in defaults.items():
    assert name in camera["params"]
    assert f'{{"{name}", {{PERSISTENT, BOOL, "{default}"}}}}' in params_keys
  for removed in ("VehicleNaviSkipBoxCamera", "VehicleNaviSkipMobileZone", "VehicleNaviSectionAvgMargin"):
    assert removed not in by_name
    assert removed not in params_keys


def test_vehicle_navi_curve_control_settings_are_removed(settings):
  driving = next(category for category in settings["menu"] if category["id"] == "DRIVING")
  speed = next(group for group in driving["groups"] if group["id"] == "SPEED")
  curve = next(group for group in speed["groups"] if group["id"] == "SPEED_CURVE")
  assert not any(name.startswith("VehicleNaviCurve") for name in curve["params"])
  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert "VehicleNaviCurve" not in params_keys


def test_tpms_position_setting_matches_device_support(params):
  by_name = {p["name"]: p for p in params}
  show_tpms = by_name["ShowTpms"]
  assert (show_tpms["min"], show_tpms["max"], show_tpms["default"]) == (0, 3, 1)
  assert "C4(MICI)는 해당 없음" in show_tpms["descr"]
  assert "Not applicable to C4 (MICI)" in show_tpms["edescr"]

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"ShowTpms", {PERSISTENT, INT, "1"}}' in params_keys


def test_external_hud_brightness_and_orientation_use_catalog_controls(settings, params):
  by_name = {p["name"]: p for p in params}
  brightness = by_name["ClusterHudBrightness"]
  assert (brightness["min"], brightness["max"], brightness["default"]) == (0, 100, 0)
  assert "live_update" not in brightness

  orientation = by_name["ClusterHudOrientation"]
  assert (orientation["min"], orientation["max"], orientation["default"]) == (0, 3, 0)
  assert orientation["control"] == "select"
  assert orientation["descr"].endswith("0: 0도\n1: X\n2: 180도\n3: X")
  assert orientation["options"]["en"] == [
    "0 degrees",
    "Unsupported",
    "180 degrees",
    "Unsupported",
  ]
  panel_layout = by_name["ClusterHudPanelLayout"]
  assert (panel_layout["min"], panel_layout["max"], panel_layout["default"]) == (0, 1, 0)
  assert panel_layout["control"] == "select"
  assert panel_layout["options"]["en"] == [
    "Driving left / info right",
    "Info left / driving right",
  ]
  screen_mode = by_name["ClusterHudScreenMode"]
  assert (screen_mode["min"], screen_mode["max"], screen_mode["default"]) == (-1, 5, 0)
  assert screen_mode["descr"].startswith("-1: 3D 전체화면\n0: 기본(내비/주행 리포트)")
  assert screen_mode["edescr"].startswith("-1: 3D fullscreen\n0: Default (navigation/driving report)")
  assert screen_mode["cdescr"].startswith("-1: 3D全屏\n0: 默认(导航/驾驶报告)")

  display = next(category for category in settings["menu"] if category["id"] == "DISPLAY")
  hud = next(group for group in display["groups"] if group["id"] == "DISP_HUD")
  basic = next(group for group in hud["groups"] if group["id"] == "HUD_BASIC")
  screen = next(group for group in hud["groups"] if group["id"] == "HUD_SCREEN")
  assert basic["params"][:3] == [
    "ClusterHud",
    "ClusterHudBrightness",
    "ClusterHudOrientation",
  ]
  assert screen["params"] == [
    "ClusterHudEncoder",
    "ClusterHudScreenMode",
    "ClusterHudPanelLayout",
    "ClusterHudCameraViewMode",
  ]
  camera_view = by_name["ClusterHudCameraViewMode"]
  assert (camera_view["min"], camera_view["max"], camera_view["default"]) == (0, 4, 0)
  assert camera_view["control"] == "select"
  assert camera_view["options"]["ko"][2:] == ["일반 카메라", "광각 카메라", "속도 자동"]
  assert camera_view["options"]["en"][2:] == ["Narrow camera", "Wide camera", "Speed automatic"]

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"ClusterHudBrightness", {PERSISTENT, INT, "0"}}' in params_keys
  assert '{"ClusterHudOrientation", {PERSISTENT, INT, "0"}}' in params_keys
  assert '{"ClusterHudPanelLayout", {PERSISTENT, INT, "0"}}' in params_keys


def test_cluster_camera_preference_is_in_brightness_and_view(settings, params):
  by_name = {p["name"]: p for p in params}
  camera = by_name["ShowCameraWithCluster"]
  assert (camera["min"], camera["max"], camera["default"], camera["unit"]) == (0, 1, 0, 1)
  assert camera["descr"] == "0: 카메라 미표시(기본)\n1: 카메라 영상 표시"
  assert camera["edescr"] == "0: Hide camera (default)\n1: Show camera video"
  assert camera["cdescr"] == "0: 不显示摄像头（默认）\n1: 显示摄像头画面"

  display = next(category for category in settings["menu"] if category["id"] == "DISPLAY")
  brightness = next(group for group in display["groups"] if group["id"] == "DISP_BRIGHT")
  assert brightness["params"] == [
    "ShowCustomBrightness",
    "ShowModelView",
    "ShowCameraWithCluster",
  ]

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"ShowCameraWithCluster", {PERSISTENT, INT, "0"}}' in params_keys


def test_carrot_radar_is_fixed_without_mode_or_sensitivity(settings, params):
  by_name = {p["name"]: p for p in params}
  for removed_name in (
    "RadarLeadModelMode",
    "RadarDPathMode",
    "RadarMotionMode",
    "CarrotRadarMode",
    "CarrotRadarCutInSensitivity",
  ):
    assert removed_name not in by_name
  vehicle = next(category for category in settings["menu"] if category["id"] == "VEHICLE")
  radar = next(group for group in vehicle["groups"] if group["id"] == "VEH_RADAR")
  assert radar["params"] == [
    "EnableRadarTracks",
    "RadarTrackFlip",
    "EnableCornerRadar",
  ]


def test_parameter_names_are_unique(params):
  names = [p["name"] for p in params]
  assert len(names) == len(set(names))


def test_every_declared_display_unit_is_one_the_ui_knows(params):
  declared = {p["name"]: p["display_unit"] for p in params if "display_unit" in p}
  assert declared, "the migration should have left declarations behind"
  unknown = {name: unit for name, unit in declared.items() if unit not in KNOWN_DISPLAY_UNITS}
  assert unknown == {}


def test_every_declared_control_is_one_the_ui_knows(params):
  declared = {p["name"]: p["control"] for p in params if "control" in p}
  assert declared
  unknown = {name: kind for name, kind in declared.items() if kind not in KNOWN_CONTROL_KINDS}
  assert unknown == {}


# "raw" is the absence of a unit, so declaring it only adds noise.
def test_no_parameter_declares_the_empty_unit(params):
  assert [p["name"] for p in params if p.get("display_unit") == "raw"] == []


def test_a_percent_parameter_still_carries_its_unit(params):
  by_name = {p["name"]: p for p in params}
  # The parameter behind the cruise-speed incident; its unit must survive.
  assert by_name["ApplyModelSpeed"]["display_unit"] == "percent"
  assert by_name["TFollowDecelBoost"]["display_unit"] == "percent"


def test_steer_ratio_rate_has_safe_catalog_and_registry_defaults(params):
  by_name = {p["name"]: p for p in params}
  steer_ratio_rate = by_name["SteerRatioRate"]
  assert (steer_ratio_rate["min"], steer_ratio_rate["max"], steer_ratio_rate["default"]) == (30, 200, 100)
  assert steer_ratio_rate["display_unit"] == "percent"

  params_keys = PARAMS_KEYS_PATH.read_text(encoding="utf-8")
  assert '{"SteerRatioRate", {PERSISTENT, INT, "100"}}' in params_keys


def test_control_overrides_land_on_parameters_that_need_them(params):
  by_name = {p["name"]: p for p in params}
  # Range inference would make these sliders, which is why they are pinned.
  assert by_name["ShowPathMode"]["control"] == "select"
  assert by_name["SoundLanguageSetting"]["control"] == "select"


def test_declared_options_cover_every_value_in_range(params):
  """Labels are indexed from `min`, so a short list silently blanks a choice."""
  declared = {p["name"]: p for p in params if "options" in p}
  assert declared

  for name, p in declared.items():
    span = p["max"] - p["min"] + 1
    for language, labels in p["options"].items():
      assert language in {"ko", "en", "zh"}, f"{name}: unexpected language {language}"
      assert isinstance(labels, list), f"{name}.{language} must be a list"
      assert len(labels) == span, f"{name}.{language} has {len(labels)} labels for {span} values"
      assert all(isinstance(label, str) and label for label in labels), f"{name}.{language} has an empty label"


def test_option_labels_exist_in_every_supported_language(params):
  for p in (p for p in params if "options" in p):
    assert set(p["options"]) == {"ko", "en", "zh"}, p["name"]


def test_every_declared_risk_is_one_the_ui_knows(params):
  declared = {p["name"]: p["risk"] for p in params if "risk" in p}
  unknown = {name: level for name, level in declared.items() if level not in KNOWN_RISK_LEVELS}
  assert unknown == {}


def test_the_incident_parameter_is_flagged_risky(params):
  # ApplyModelSpeed at -1 pins cruise to 5 km/h — the parameter this whole
  # effort started from. It stays marked so its warning badge shows.
  by_name = {p["name"]: p for p in params}
  assert by_name["ApplyModelSpeed"].get("risk") == "high"


def test_declared_numeric_bounds_stay_intact(params):
  """The migration rewrote the whole file, so the values it did not touch
  have to be exactly what they were."""
  by_name = {p["name"]: p for p in params}
  assert (by_name["ApplyModelSpeed"]["min"], by_name["ApplyModelSpeed"]["max"]) == (-120, 120)
  assert by_name["ApplyModelSpeed"]["default"] == 0
  assert by_name["TFollowDecelBoost"]["unit"] == 10


def test_k9_stop_mode_is_one_click_select_with_three_choices(settings, params):
  stop = next(p for p in params if p["name"] == "K9StopMode")
  assert (stop["min"], stop["max"], stop["default"], stop["unit"]) == (0, 3, 0, 1)
  assert stop["control"] == "select" and stop["title"] == "K9 정차 방식 선택 (실험)"
  # history of this pin: 2026-10-23..24 the 0-6 menu and the 0-3 menu (v10 / v10 + light / v11), web text short since
  # 2026-10-24 (보노보노 "웹설명 너무길게말고 짧고 간단하게"; details only in docs/user/{ko,en}/settings.md, docs/k9_menus_zh.md).
  # 2026-10-25 (보노보노 "브레이크 앞전꺼는 다 별로니 지워" / "v12 원본 그대로 ... v12k9"): 0 base, 1 v12 original, 2 v12k9
  # (menu-set pins rewritten: max 2, 3 labels, '3' banned instead of '4'; the deleted v10 / v11 labels banned)
  # 2026-10-26 (보노보노 "v12그냥놓아두고 v13추가하는걸로"): 3 v13 original added (max 3, 4 labels, '3' allowed only as the v13 label,
  # '4 or more' acts as 0 in the manual)
  # 2026-10-28 (보노보노 01:06 "원본 그대로 푸쉬해라 v14"; 4 C1 / 5 C1+ of 2026-10-27 removed): 3 = v14 original instead of v13
  # (labels only; max 3, 4 labels, '3' allowed only as the v14 label)
  # 2026-10-29 (보노보노 00:50 "이름에2붙여서 올려봐", 01:39 "이름 안햇깔리게 똑바로 써놔라"): label 2 = bono1739 원본2
  # (d5b7b048b behaviour; labels / web text only, max 3, 4 labels as before)
  # 2026-10-29 04:48 (보노보노 "메뉴1번에다 내가 밟은거 그대로를 넣으라햇잔니"): his own stop table is label 1
  # 2026-10-29 05:40 (보노보노 "지금 2번인 v12 원본은 메뉴에서 빼라" / "2번이 내가직접밟은로그 너가개선한거" / "4번에다가 ...
  # 완전정차후1초뒤 브레이크 좀더 세게"): the v12 original left the menu with its code, 2 = his table improved and
  # 4 = his table + harder braking 1 s after the stop (max 4, 5 labels, the v12 original label banned below)
  # 2026-10-29 07:10 (보노보노 "3.v14제거후 4번을 3번으로"): the v14 original left the menu with its code and his table +
  # the harder braking after 1 s moved from 4 to 3 (max 3, 4 labels, the v14 label banned below with the v12 one)
  assert stop["options"] == {"ko": ["0 기본 제동", "1 bono1739 원본2", "2 bono1739 끝 구간 개선", "3 bono1739 원본2 + 선 뒤 1초 뒤 더 세게"],
                             "en": ["0 Base braking", "1 bono1739 original2", "2 bono1739 end window improved", "3 bono1739 original2 + harder 1 s after the stop"],
                             "zh": ["0 基本制动", "1 bono1739原版2", "2 bono1739末段改良", "3 bono1739原版2＋停后1秒起加压"]}
  only_3 = {"descr": "3 원본2", "edescr": "3 original2", "cdescr": "3 原版2"}  # the one label a "3" may appear in
  for key in ("descr", "edescr", "cdescr"):
    assert "K9StopTaper" not in stop[key] and "K9CreepMode" not in stop[key]
    rest = stop[key].replace(only_3[key], "")
    assert "v8" not in stop[key] and "3:" not in rest and "3：" not in rest and "3 " not in rest
    assert "v9" not in stop[key] and "v10" not in stop[key] and "0.2" not in stop[key]
    for jargon in ("ComfortBand", "StopReq", "aReq", "m/s", "jerk", "Kalman", "래치", "latch"):
      assert jargon not in stop[key], (key, jargon)
  assert len(stop["descr"]) <= 300 and len(stop["cdescr"]) <= 300 and len(stop["edescr"]) <= 600
  for label in ("0 기본", "1 bono1739 원본2", "2 bono1739 끝 구간 개선", "3 원본2"):
    assert label in stop["descr"], label
  for label in ("0 Base", "1 bono1739 original2", "2 bono1739 end window improved", "3 original2"):
    assert label in stop["edescr"], label
  for label in ("0 基本", "1 bono1739原版2", "2 bono1739末段改良", "3 原版2"):
    assert label in stop["cdescr"], label
  for key, gone in (("descr", "v12 원본"), ("edescr", "v12 original"), ("cdescr", "v12原版")):
    assert gone not in stop[key], (key, gone)  # the v12 original left the menu 2026-10-29 05:40
  for key in ("descr", "edescr", "cdescr"):
    assert "v14" not in stop[key], key  # the v14 original left the menu 2026-10-29 07:10
  assert "K9 롱컨 전용" in stop["descr"] and "K9 openpilot long only" in stop["edescr"]
  assert "페달을 밟으면 바로 원래대로" in stop["descr"] and "Pressing a pedal returns to normal at once" in stop["edescr"]
  assert stop["descr"].endswith("실차 미확인.") and stop["edescr"].endswith("Not tested in the car.") and stop["cdescr"].endswith("未经实车确认。")
  # the facts that left the web text are still in the manual
  doc_ko = (SETTINGS_PATH.parents[2] / "docs" / "user" / "ko" / "settings.md").read_text(encoding="utf-8")
  doc_en = (SETTINGS_PATH.parents[2] / "docs" / "user" / "en" / "settings.md").read_text(encoding="utf-8")
  row_ko = next(line for line in doc_ko.splitlines() if "| `K9StopMode` |" in line)
  row_en = next(line for line in doc_en.splitlines() if "| `K9StopMode` |" in line)
  assert "0: 기본 제동" in row_ko and "1: bono1739 원본2" in row_ko and "2: bono1739 끝 구간 개선" in row_ko
  assert "바탕 v12k9" in row_ko and "3: bono1739 원본2 + 선 뒤 1초 뒤 더 세게" in row_ko
  assert "0: base braking" in row_en and "1: bono1739 original2" in row_en and "2: bono1739 end window improved" in row_en
  assert "Base v12k9" in row_en and "3: bono1739 original2 + harder 1 s after the stop" in row_en
  # 2026-10-27 evening / 2026-10-28 / 05:40 / 07:10 (the v14 original left the menu and 4 moved into its slot)
  for gone in ("3: v13 원본", "4: C1", "5: C1+", "1: v12 원본", "2: v12 원본", "3: v14 원본", "4: bono1739"):
    assert gone not in row_ko, gone
  for gone in ("3: v13 original", "4: C1", "5: C1+", "1: v12 original", "2: v12 original", "3: v14 original", "4: bono1739"):
    assert gone not in row_en, gone
  for gone in ("1: v10", "2: v10 + 약한 떨림 방지", "3: v11", "4: v10", "5: v10", "6: v11"):
    assert gone not in row_ko, gone
  for gone in ("1: v10", "2: v10 + light shudder fix", "3: v11", "4: v10", "5: v10", "6: v11"):
    assert gone not in row_en, gone
  assert "예전 1·2·3 → 2(v12k9)" in row_ko and "old 1 / 2 / 3 -> 2 (v12k9)" in row_en
  assert "K9StopModeVer 3" in row_ko and "K9StopModeVer 3" in row_en
  assert "일반 CAN(CAN-FD 아님) 카메라 SCC" in row_ko and "non-CAN-FD camera-SCC" in row_en
  assert "즉시 원래 명령" in row_ko and "returns the original command at once" in row_en
  assert "4 이상" in row_ko and "(4 or more, negative)" in row_en  # 5 or more between 2026-10-29 05:40 and 07:10
  assert "0에서 시작" in row_ko and "starts at 0" in row_en
  assert "changelog.html" in row_ko and "changelog.html" in row_en
  zh = (SETTINGS_PATH.parents[2] / "docs" / "k9_menus_zh.md").read_text(encoding="utf-8")
  assert "1：bono1739原版2" in zh and "2：bono1739末段改良" in zh and "3：bono1739原版2＋停后1秒起加压" in zh
  assert "基础v12k9" in zh and "尚未经实车确认" in zh and "原1·2·3→2（v12k9）" in zh
  assert "3：v13原版" not in zh and "4：C1" not in zh and "5：C1+" not in zh  # removed 2026-10-28
  assert "1：v12原版" not in zh and "2：v12原版" not in zh  # the v12 original left the menu 2026-10-29 05:40
  assert "3：v14原版" not in zh and "4：bono1739" not in zh  # the v14 original left the menu 2026-10-29 07:10
  assert "2：v10+轻度抖动修正" not in zh and "3：v11" not in zh and "4：v10+抖动修正" not in zh and "6：v11" not in zh
  menu = json.dumps(settings["menu"])
  assert "K9StopMode" in menu and "K9CreepMode" not in menu
  assert not any(p["name"] == "K9CreepMode" for p in params)
  # the old taper switch is deleted: not in the menu, not in the catalog (search) and not a params key
  assert "K9StopTaper" not in json.dumps(settings["menu"])
  assert not any(p["name"] == "K9StopTaper" for p in params)
