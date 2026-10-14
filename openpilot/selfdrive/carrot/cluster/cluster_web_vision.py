"""Carrot web vision (당근웹비전) drawing rules for the external HUD camera view 5.

The web renderer is the rule source; nothing here imports or changes it:
  web/js/realtime/home_drive.js                   projection, lead boxes, alert, plot
  web/js/realtime/vision_webgl2.js                triangulation, strokes, gradient
  web/js/realtime/vision_raw.js                   HUD payload
  web/src/features/drive/contents/vision/*.js     overlay policy/geometry/aux/lead
  web/src/features/drive/contents/vision/hud/*    HUD widgets and their CSS layout

Inputs are converted with the server's own compact display schema
(realtime/compact_state.py SERVICE_SCHEMAS) so every field has the shape and
resolution the browser decodes. All geometry is produced in road-camera source
pixels, exactly like the web canvas; the renderer maps it to the HUD viewport
with the same stage transform the web applies to its video element. This module
has no raylib dependency so the rules can be tested headlessly.
"""
from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field
import math
import sys
from pathlib import Path
from typing import Any


def _find_openpilot_root(start: Path) -> Path | None:
    for path in (start, *start.parents):
        if (path / "cereal").exists() and (path / "selfdrive").exists():
            return path
    return None


try:
    from openpilot.selfdrive.carrot.realtime.compact_state import SERVICE_SCHEMAS
except ImportError:  # standalone cluster checkout started from its own folder
    _root = _find_openpilot_root(Path(__file__).resolve().parent)
    if _root is not None and str(_root.parent) not in sys.path:
        sys.path.insert(0, str(_root.parent))
    from openpilot.selfdrive.carrot.realtime.compact_state import SERVICE_SCHEMAS

Point = tuple[float, float]
RGBA = tuple[float, float, float, float]  # r, g, b in 0..255, a in 0..1

# --------------------------------------------------------------------------
# Inputs
# --------------------------------------------------------------------------

# vision_compact.js HUD_SERVICES / OVERLAY_SERVICES that the drawing uses.
WEB_VISION_HUD_SERVICES = (
    "carState", "controlsState", "deviceState", "carrotMan", "selfdriveState", "longitudinalPlan",
)
WEB_VISION_OVERLAY_SERVICES = (
    "modelV2", "liveCalibration", "roadCameraState", "lateralPlan", "radarState", "carControl", "liveParameters",
)
WEB_VISION_SERVICES = WEB_VISION_HUD_SERVICES + WEB_VISION_OVERLAY_SERVICES

# The browser decodes these enums as names; every other enum arrives as its u8 index.
_ENUM_NAME_FIELDS = {
    ("carState", "gearShifter"),
    ("deviceState", "deviceType"),
    ("liveCalibration", "calStatus"),
    ("roadCameraState", "sensor"),
}

# A model older than this is not drawn as if it were current.
MODEL_STALE_SECONDS = 0.5


def _attr(value: Any, name: str) -> Any:
    try:
        return getattr(value, name)
    except Exception:
        return None


def _attr_path(value: Any, path: Sequence[str]) -> Any:
    for name in path:
        value = _attr(value, name)
        if value is None:
            break
    return value


def _py_number(value: Any) -> float:
    try:
        number = float(value)
    except Exception:
        return 0.0
    return number if math.isfinite(number) else 0.0


def _py_int(value: Any) -> int:
    try:
        return int(value)
    except Exception:
        return 0


def _py_float_list(value: Any) -> list[float]:
    if value is None or isinstance(value, (str, bytes, bytearray)):
        return []
    try:
        return [_py_number(item) for item in value][:0xffff]
    except Exception:
        return []


def _enum_index(value: Any, names: Sequence[str]) -> int:
    raw = getattr(value, "raw", None)
    numeric = _py_int(raw if raw is not None else value) if not isinstance(value, str) else -1
    if isinstance(value, str):
        try:
            numeric = int(value)
        except ValueError:
            numeric = -1
    if 0 <= numeric < len(names):
        return numeric
    normalized = str(value or "").split(".")[-1].replace("_", "").lower()
    for index, name in enumerate(names):
        if normalized == name.replace("_", "").lower():
            return index
    return 0


def _quantized(values: list[float], scale: float, minimum: int, maximum: int) -> list[float]:
    return [max(minimum, min(maximum, round(item * scale))) / scale for item in values]


def _convert_field(value: Any, spec: Any, service: str, name: str) -> Any:
    # Mirrors compact_state._pack_field followed by vision_compact.js readField.
    if spec == "bool":
        return bool(value)
    if spec in ("i8", "u8", "i16", "u16", "i32", "u32", "u64"):
        return _py_int(value)
    if spec in ("f32", "f64"):
        return _py_number(value)
    if spec == "text":
        return str(value or "")
    if spec == "f32_list":
        return _py_float_list(value)
    if spec == "f32_first_list":
        try:
            return [_py_number(value[0])] if value is not None and len(value) > 0 else []
        except Exception:
            return []
    if spec == "u16_cm_list":
        return _quantized(_py_float_list(value), 100.0, 0, 0xffff)
    if spec == "i16_cm_list":
        return _quantized(_py_float_list(value), 100.0, -32768, 32767)
    if spec == "i16_mm_list":
        return _quantized(_py_float_list(value), 1000.0, -32768, 32767)
    if spec in ("i16_list",):
        try:
            return [_py_int(item) for item in list(value or [])]
        except Exception:
            return []
    if isinstance(spec, tuple) and spec[0] == "enum":
        index = _enum_index(value, spec[1])
        return spec[1][index] if (service, name) in _ENUM_NAME_FIELDS else index
    if isinstance(spec, tuple) and spec[0] == "struct":
        return _convert_schema(value, spec[1], service)
    if isinstance(spec, tuple) and spec[0] == "struct_list":
        try:
            items = list(value or [])[:255]
        except Exception:
            items = []
        return [_convert_schema(item, spec[1], service) for item in items]
    return None


def _convert_schema(value: Any, schema: Sequence[tuple[Any, ...]], service: str) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for entry in schema:
        name, spec = entry[:2]
        source_path = entry[2] if len(entry) > 2 else (name,)
        out[name] = _convert_field(_attr_path(value, source_path), spec, service, name)
    return out


def web_state_from_message(service: str, message: Any) -> dict[str, Any]:
    """The decoded object the browser holds for one cereal message."""
    _, schema = SERVICE_SCHEMAS[service]
    return _convert_schema(message, schema, service)


@dataclass(slots=True)
class WebVisionInputs:
    """Latest decoded web-vision services plus their arrival time and validity."""

    services: dict[str, dict[str, Any]] = field(default_factory=dict)
    received_s: dict[str, float] = field(default_factory=dict)
    valid: dict[str, bool] = field(default_factory=dict)
    sequence: dict[str, int] = field(default_factory=dict)
    generation: int = 0

    def update(self, service: str, message: Any, received_s: float, valid: bool = True) -> None:
        if service not in WEB_VISION_SERVICES:
            return
        self.services[service] = web_state_from_message(service, message)
        self.received_s[service] = float(received_s)
        self.valid[service] = bool(valid)
        self.generation += 1
        self.sequence[service] = self.generation

    def reset(self) -> None:
        self.services.clear()
        self.received_s.clear()
        self.valid.clear()
        self.sequence.clear()
        self.generation += 1

    def snapshot(self, now_s: float, params: Mapping[str, float] | None = None,
                 clock_text: str | None = None) -> WebVisionSnapshot:
        model_t = self.received_s.get("modelV2")
        model_age = None if model_t is None else max(0.0, now_s - model_t)
        return WebVisionSnapshot(
            services=dict(self.services),
            model_age_s=model_age,
            model_valid=self.valid.get("modelV2", False),
            generation=self.generation,
            sequence=dict(self.sequence),
            params=dict(params or WEB_VISION_DEFAULT_PARAMS),
            now_s=now_s,
            clock_text=clock_text,
        )


@dataclass(frozen=True, slots=True)
class WebVisionSnapshot:
    services: Mapping[str, Mapping[str, Any]] = field(default_factory=dict)
    model_age_s: float | None = None
    model_valid: bool = False
    generation: int = 0
    sequence: Mapping[str, int] = field(default_factory=dict)
    params: Mapping[str, float] = field(default_factory=dict)
    now_s: float | None = None
    clock_text: str | None = None

    def model_usable(self) -> bool:
        return (
            "modelV2" in self.services
            and self.model_valid
            and self.model_age_s is not None
            and self.model_age_s <= MODEL_STALE_SECONDS
        )

    def hud_state(self) -> dict[str, Any]:
        # live_state_provider.js mergeRuntimeState(): missing services become {}.
        state = {name: self.services[name] for name in WEB_VISION_HUD_SERVICES if name in self.services}
        for name in ("carState", "controlsState", "deviceState", "selfdriveState", "longitudinalPlan", "carrotMan"):
            state.setdefault(name, {})
        state["lateralPlan"] = self.services.get("lateralPlan", {})
        state["radarState"] = self.services.get("radarState", {})
        return state

    def raw_hud_state(self) -> dict[str, Any]:
        # vision_raw.js RAW_HUD_STATE holds HUD services only.
        return {name: self.services[name] for name in WEB_VISION_HUD_SERVICES if name in self.services}

    def overlay_state(self) -> dict[str, Any]:
        state = {name: self.services[name] for name in WEB_VISION_OVERLAY_SERVICES if name in self.services}
        state["liveCalibration"] = self.services.get("liveCalibration", {})
        state["roadCameraState"] = self.services.get("roadCameraState", {})
        state["radarState"] = self.services.get("radarState", {})
        state["lateralPlan"] = self.services.get("lateralPlan", {})
        state["carrotMan"] = self.services.get("carrotMan", {})
        return state


# --------------------------------------------------------------------------
# JavaScript number helpers
# --------------------------------------------------------------------------

def js_number(value: Any) -> float:
    """Number(value) for the decoded value types (None is `undefined`)."""
    if value is None:
        return math.nan
    if isinstance(value, bool):
        return 1.0 if value else 0.0
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return 0.0
        try:
            return float(text)
        except ValueError:
            return math.nan
    return math.nan


def finite(value: Any, fallback: float = 0.0) -> float:
    number = js_number(value)
    return number if math.isfinite(number) else fallback


def clamp(value: float, minimum: float, maximum: float) -> float:
    return min(maximum, max(minimum, value))


def js_to_fixed(value: float, digits: int) -> str:
    # Number.prototype.toFixed rounds half away from zero on the decimal value.
    if not math.isfinite(value):
        return "NaN"
    scale = 10 ** digits
    rounded = math.floor(abs(value) * scale + 0.5)
    text = f"{rounded / scale:.{digits}f}"
    return f"-{text}" if value < 0 and rounded != 0 else text


def js_mod(value: float, divisor: float) -> float:
    return math.fmod(value, divisor)


def _get(obj: Any, name: str) -> Any:
    return obj.get(name) if isinstance(obj, Mapping) else None


def first_finite(values: Any, fallback: float = 0.0) -> float:
    if not isinstance(values, list):
        return fallback
    for value in values:
        number = js_number(value)
        if math.isfinite(number):
            return number
    return fallback


# --------------------------------------------------------------------------
# Display settings (server/live_runtime/broker.py _read_params + normalizeVisualParams)
# --------------------------------------------------------------------------

WEB_VISION_PARAM_KEYS = (
    "IsMetric", "ShowPathEnd", "ShowLaneInfo", "ShowPathMode", "ShowPathColor", "ShowPathModeLane",
    "ShowPathColorLane", "ShowPathColorCruiseOff", "ShowPathWidth", "ShowPlotMode", "ShowRadarInfo",
    "RadarLatFactor",
)
WEB_VISION_DEFAULT_PARAMS: dict[str, float] = {
    "IsMetric": 1.0,
    "ShowPathEnd": 0.0,
    "ShowLaneInfo": 2.0,
    "ShowPathMode": 0.0,
    "ShowPathColor": 13.0,
    "ShowPathModeLane": 0.0,
    "ShowPathColorLane": 13.0,
    "ShowPathColorCruiseOff": 19.0,
    "ShowPathWidth": 100.0,
    "ShowPlotMode": 0.0,
    "ShowRadarInfo": 0.0,
    "RadarLatFactor": 0.0,
}


def read_web_vision_raw_params(params: Any) -> dict[str, str | None]:
    """The exact snapshot the web server sends to the browser."""
    snapshot: dict[str, str | None] = {}
    for name in WEB_VISION_PARAM_KEYS:
        try:
            if name == "IsMetric":
                snapshot[name] = "1" if params.get_bool(name) else "0"
                continue
            value = params.get(name)
        except Exception:
            value = None
        if value is None:
            snapshot[name] = None
        elif isinstance(value, bytes):
            snapshot[name] = value.decode("utf-8", errors="replace")
        else:
            snapshot[name] = str(value)
    return snapshot


def _finite_param_number(value: Any, fallback: float) -> float:
    if value is None:
        return fallback
    if isinstance(value, str) and not value.strip():
        return fallback
    number = js_number(value)
    return number if math.isfinite(number) else fallback


def normalize_web_vision_params(values: Mapping[str, Any] | None, previous: Mapping[str, float] | None = None) -> dict[str, float]:
    fallback = dict(previous or WEB_VISION_DEFAULT_PARAMS)
    source = values or {}
    return {name: _finite_param_number(source.get(name), fallback[name]) for name in WEB_VISION_PARAM_KEYS}


# --------------------------------------------------------------------------
# Lane mode / policy / lead model (lane_mode.js, road_overlay_policy.js, road_overlay_lead_model.js)
# --------------------------------------------------------------------------

def _optional_bool(value: Any) -> bool | None:
    return None if value is None else bool(value)


def resolve_lane_mode_fields(requested: Any, planned: Any, controlled: Any) -> dict[str, Any]:
    requested = _optional_bool(requested)
    planned = _optional_bool(planned)
    controlled = _optional_bool(controlled)
    path_active = planned if planned is not None else (controlled if controlled is not None else False)
    presentation = "unknown"
    if requested is False:
        presentation = "laneless"
    elif controlled is True:
        presentation = "active"
    elif controlled is False:
        presentation = "armed" if requested is True or planned is True else "laneless"
    elif planned is True:
        presentation = "active"
    elif requested is True:
        presentation = "armed"
    elif planned is False:
        presentation = "laneless"
    return {
        "requested": requested,
        "planned": planned,
        "controlled": controlled,
        "pathActive": path_active is True,
        "presentation": presentation,
    }


def resolve_lane_mode_state(state: Mapping[str, Any]) -> dict[str, Any]:
    speed = _get(state.get("carState"), "useLaneLineSpeed")
    speed_number = js_number(speed) if speed is not None and speed != "" else math.nan
    requested = None if not math.isfinite(speed_number) else speed_number > 0
    return resolve_lane_mode_fields(
        requested,
        _get(state.get("lateralPlan"), "useLaneLines"),
        _get(state.get("controlsState"), "activeLaneLine"),
    )


PATH_PALETTE = (
    (255, 82, 82), (255, 153, 0), (255, 214, 74), (83, 220, 118), (78, 144, 255),
    (0, 0, 128), (139, 0, 255), (191, 150, 87), (255, 255, 255), (28, 28, 28),
)


def palette_color(index: float) -> tuple[int, int, int]:
    count = len(PATH_PALETTE)
    try:
        normalized = int((js_mod(float(index), count) + count) % count)
    except (TypeError, ValueError):
        return PATH_PALETTE[3]
    return PATH_PALETTE[normalized] if 0 <= normalized < count else PATH_PALETTE[3]


WHITE_92: RGBA = (255, 255, 255, 0.92)
BRAKE_STROKE: RGBA = (255, 76, 76, 0.96)


def resolve_road_overlay_policy(overlay: Mapping[str, Any], hud: Mapping[str, Any], params: Mapping[str, float]):
    lane_mode = resolve_lane_mode_state(hud)["pathActive"]
    model = overlay.get("modelV2")
    lane_path = _get(overlay.get("lateralPlan"), "position")
    has_lane_path = isinstance(_get(lane_path, "x"), list) and len(lane_path["x"]) > 2
    use_lane = lane_mode and has_lane_path
    selected = {
        "pathData": lane_path if use_lane else _get(model, "position"),
        "pathSource": "lateralPlan" if use_lane else "modelV2",
        "laneMode": lane_mode,
    }
    mode = finite(params["ShowPathModeLane"] if lane_mode else params["ShowPathMode"], 0.0)
    color_index = finite(params["ShowPathColorLane"] if lane_mode else params["ShowPathColor"], 13.0)
    is_cruise_off = not bool(_get(overlay.get("carControl"), "longActive"))
    if is_cruise_off:
        color_index = finite(params["ShowPathColorCruiseOff"], 19.0)
    elif color_index >= 20:
        lead_one = _get(overlay.get("radarState"), "leadOne") or {}
        accel = first_finite(_get(hud.get("longitudinalPlan"), "accels"), 0.0)
        color_index = 13
        if lead_one.get("status"):
            if abs(accel) < 0.5:
                color_index = 12
            elif accel >= 0.5:
                color_index = 11
            else:
                color_index = 10
    brake_lights = bool(_get(hud.get("carState"), "brakeLights"))
    style = {
        "mode": mode,
        "colorIndex": color_index,
        "paletteIndex": js_mod(color_index, 10),
        "emphasisStroke": color_index >= 10 or brake_lights,
        "strokeColor": BRAKE_STROKE if brake_lights else WHITE_92,
        "isCruiseOff": is_cruise_off,
        "laneMode": lane_mode,
    }
    return selected, style


E2E_DRIVING_TEXT = {"ko": "E2E 주행중", "en": "E2E driving", "zh": "E2E驾驶中"}


def resolve_lead_state(overlay: Mapping[str, Any], hud: Mapping[str, Any], language: str = "ko") -> dict[str, Any]:
    long_plan = hud.get("longitudinalPlan") or {}
    carrot_man = overlay.get("carrotMan") or hud.get("carrotMan") or {}
    car_state = hud.get("carState") or {}
    x_state = finite(long_plan.get("xState"), finite(carrot_man.get("xState"), -1.0))
    traffic_state = finite(long_plan.get("trafficState"), finite(carrot_man.get("trafficState"), -1.0))
    long_active = bool(_get(hud.get("controlsState"), "enabled"))
    v_ego = finite(car_state.get("vEgo"), finite(car_state.get("vEgoCluster"), 0.0))
    brake_hold = bool(car_state.get("brakeHoldActive"))
    soft_hold = finite(car_state.get("softHoldActive"), 0.0) > 0
    carrot_cruise = finite(car_state.get("carrotCruise"), 0.0) > 0
    if brake_hold or soft_hold or carrot_cruise:
        return {"text": "AUTOHOLD" if brake_hold else ("SOFTHOLD" if soft_hold else "CARROT"),
                "xState": x_state, "showDistanceBadge": False}
    if not long_active:
        return {"text": "", "xState": x_state, "showDistanceBadge": True}
    if x_state in (3, 5):
        text = ("Signal Error" if traffic_state >= 1000 else "Signal Ready") if v_ego < 1.0 else "Signal slowing"
        return {"text": text, "xState": x_state, "showDistanceBadge": False}
    if x_state == 4:
        return {"text": E2E_DRIVING_TEXT.get(language, E2E_DRIVING_TEXT["en"]), "xState": x_state,
                "showDistanceBadge": False}
    if x_state in (0, 1, 2):
        return {"text": "", "xState": x_state, "showDistanceBadge": True}
    return {"text": "", "xState": x_state, "showDistanceBadge": False}


def distance_badge_text_color(x_state: float) -> RGBA:
    if x_state == 0:
        return (255, 255, 255, 1.0)
    if x_state == 1:
        return (176, 176, 176, 1.0)
    return (35, 213, 93, 1.0)


def path_status_text(hud: Mapping[str, Any]) -> str:
    car_state = hud.get("carState") or {}
    if bool(car_state.get("brakeHoldActive")):
        return "AUTOHOLD"
    if finite(car_state.get("softHoldActive"), 0.0) > 0:
        return "SOFTHOLD"
    if finite(car_state.get("carrotCruise"), 0.0) > 0:
        return "CARROT"
    return ""


def radar_tracks(radar_state: Mapping[str, Any]) -> list[Mapping[str, Any]]:
    tracks = []
    for name in ("leadsLeft", "leadsRight", "leadsCenter"):
        value = radar_state.get(name)
        if isinstance(value, list):
            tracks.extend(value)
    if tracks:
        return tracks
    fallback = []
    for name in ("leadOne", "leadTwo"):
        lead = radar_state.get(name)
        if isinstance(lead, Mapping) and lead.get("status"):
            fallback.append(lead)
    return fallback


def radar_projection_line(model: Mapping[str, Any] | None) -> Mapping[str, Any] | None:
    lane_lines = _get(model, "laneLines")
    lane_lines = lane_lines if isinstance(lane_lines, list) else []
    center = lane_lines[2] if len(lane_lines) > 2 else None
    if isinstance(_get(center, "x"), list) and isinstance(_get(center, "z"), list) and center["x"] and center["z"]:
        return center
    return _get(model, "position")


def primary_vision_distance(model: Mapping[str, Any] | None) -> float:
    leads = _get(model, "leadsV3")
    lead = leads[0] if isinstance(leads, list) and leads else None
    prob = finite(_get(lead, "prob"), 0.0)
    xs = _get(lead, "x")
    distance = finite(xs[0] if isinstance(xs, list) and xs else None, 0.0)
    return max(0.0, distance - 1.52) if prob > 0.5 and distance > 0 else 0.0


# --------------------------------------------------------------------------
# Projection (home_drive.js)
# --------------------------------------------------------------------------

VIEW_FROM_DEVICE = ((0.0, 1.0, 0.0), (0.0, 0.0, 1.0), (1.0, 0.0, 0.0))
ROAD_CAMERA_PROFILES = {
    "ar_ox": (1928.0, 1208.0, 2648.0),
    "os04c10": (1344.0, 760.0, 1141.5),
}
PATH_Z_OFFSET = 1.22
MIN_DRAW_DISTANCE = 10.0
MAX_DRAW_DISTANCE = 100.0

DISPLAY_MODE_FIT = 0
DISPLAY_MODE_NORMAL = 1
DISPLAY_MODE_CROP = 2
# A wide HUD viewport uses the web's crop display mode (cover, no letterbox).
WEB_VISION_DISPLAY_MODE = DISPLAY_MODE_CROP


def _normalize_enum_name(value: Any, numeric_names: Mapping[int, str] | None = None) -> str:
    if value is None:
        return ""
    text = str(value).strip()
    if not text:
        return ""
    if numeric_names is not None:
        number = js_number(value)
        if math.isfinite(number) and float(number).is_integer() and int(number) in numeric_names:
            return numeric_names[int(number)]
    normalized = text.lower().split(".")[-1]
    return "".join(ch for ch in normalized if ch.isalnum() or ch == "_")


def select_road_camera_profile(device_type: Any, sensor: Any) -> tuple[str, str, str] | None:
    device_key = _normalize_enum_name(device_type, {
        0: "unknown", 1: "neo", 2: "chffrandroid", 3: "chffrios", 4: "tici", 5: "pc", 6: "tizi", 7: "mici",
    })
    sensor_key = _normalize_enum_name(sensor, {0: "unknown", 1: "ar0231", 2: "ox03c10", 3: "os04c10"})
    if sensor_key == "os04c10":
        return "os04c10", device_key, sensor_key
    if sensor_key in ("ar0231", "ox03c10"):
        return "ar_ox", device_key, sensor_key
    if device_key == "tici":
        return "ar_ox", device_key, sensor_key or "unknown"
    return None


def mat3_multiply(a, b):
    return tuple(tuple(sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3)) for r in range(3))


def rot_from_euler(roll: float, pitch: float, yaw: float):
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def project_point_precise(m, x: float, y: float, z: float) -> Point | None:
    w = m[2][0] * x + m[2][1] * y + m[2][2] * z
    if not math.isfinite(w) or w <= 1e-3:
        return None
    return (m[0][0] * x + m[0][1] * y + m[0][2] * z) / w, (m[1][0] * x + m[1][1] * y + m[1][2] * z) / w


def project_point(m, x: float, y: float, z: float) -> Point | None:
    point = project_point_precise(m, x, y, z)
    if point is None:
        return None
    return float(int(point[0])), float(int(point[1]))  # JavaScript `| 0`


@dataclass(frozen=True, slots=True)
class StageTransform:
    calib: tuple
    scale: float
    tx: float
    ty: float
    video_width: float
    video_height: float
    stage_width: float
    stage_height: float
    display_mode: int


def stage_transform(
    video_width: float,
    video_height: float,
    stage_width: float,
    stage_height: float,
    calibration,
    base_camera: tuple[float, float, float],
    display_mode: int = WEB_VISION_DISPLAY_MODE,
) -> StageTransform:
    base_w, base_h, focal = base_camera
    intrinsics = (
        (focal * video_width / base_w, 0.0, video_width / 2.0),
        (0.0, focal * video_height / base_h, video_height / 2.0),
        (0.0, 0.0, 1.0),
    )
    calib = mat3_multiply(intrinsics, calibration)
    contain = min(stage_width / video_width, stage_height / video_height)
    cover = max(stage_width / video_width, stage_height / video_height)
    portrait = stage_height > stage_width
    scale = contain
    if display_mode == DISPLAY_MODE_FIT:
        scale = contain * 0.94
    elif display_mode == DISPLAY_MODE_CROP:
        scale = cover
    elif portrait:
        scale = contain * 0.985
    scale = max(scale, 0.01)
    cx, cy = intrinsics[0][2], intrinsics[1][2]
    w = calib[2][0] * 1000.0
    px = calib[0][0] * 1000.0 / w if w > 1e-3 else cx
    py = calib[1][0] * 1000.0 / w if w > 1e-3 else cy
    max_x = max(0.0, cx * scale - stage_width / 2 - 5)
    max_y = max(0.0, cy * scale - stage_height / 2 - 5)
    x_offset = clamp((px - cx) * scale, -max_x, max_x)
    y_offset = clamp((py - cy) * scale, -max_y, max_y)
    return StageTransform(
        calib, scale,
        (stage_width / 2 - x_offset) - cx * scale,
        (stage_height / 2 - y_offset) - cy * scale,
        video_width, video_height, stage_width, stage_height, display_mode,
    )


def visible_source_rect(t: StageTransform | None, video_width: float, video_height: float) -> tuple[float, float, float, float]:
    if t is None:
        return 0.0, 0.0, video_width, video_height
    scale = max(t.scale, 0.01)
    raw_left = -t.tx / scale
    raw_top = -t.ty / scale
    raw_right = (t.stage_width - t.tx) / scale
    raw_bottom = (t.stage_height - t.ty) / scale
    left = clamp(min(raw_left, raw_right), 0.0, video_width)
    right = clamp(max(raw_left, raw_right), 0.0, video_width)
    top = clamp(min(raw_top, raw_bottom), 0.0, video_height)
    bottom = clamp(max(raw_top, raw_bottom), 0.0, video_height)
    if right - left < 2 or bottom - top < 2:
        return 0.0, 0.0, video_width, video_height
    return left, top, right, bottom


# --------------------------------------------------------------------------
# Projected geometry (road_overlay_projection.js, full-detail WebGL2 path)
# --------------------------------------------------------------------------

POLYLINE_SMOOTH_NEAR_DISTANCE = 16.0
POLYLINE_SMOOTH_FAR_DISTANCE = 52.0
POLYLINE_SMOOTH_MAX_STRENGTH = 0.34
POLYLINE_CENTER_SMOOTH_MAX_STRENGTH = 0.24
QUALITY_DEFAULT = "default"
QUALITY_LANE = "lane"
QUALITY_ROAD_EDGE = "road-edge"
DEFAULT_FRAME_DT_MS = 33.0
MAX_SMOOTH_DT_MS = 250.0
CURVE_SUBDIVISIONS = 4


def _xs(line: Any) -> list:
    value = _get(line, "x")
    return value if isinstance(value, list) else []


def path_length_index(line: Any, maximum_distance: float) -> int:
    xs = _xs(line)
    max_idx = 0
    for i in range(1, len(xs)):
        if js_number(xs[i]) > maximum_distance:
            break
        max_idx = i
    return max_idx


def interpolate(x: float, xs: Sequence[Any], ys: Sequence[Any]) -> float:
    if len(xs) < 2 or len(ys) < 2:
        return math.nan
    target = finite(x, math.nan)
    if not math.isfinite(target):
        return math.nan
    last_idx = min(len(xs), len(ys)) - 1
    if target <= finite(xs[0], 0.0):
        return finite(ys[0], math.nan)
    for i in range(1, last_idx + 1):
        x0 = finite(xs[i - 1], math.nan)
        x1 = finite(xs[i], math.nan)
        if not math.isfinite(x0) or not math.isfinite(x1):
            continue
        if target > x1 and i < last_idx:
            continue
        y0 = finite(ys[i - 1], math.nan)
        y1 = finite(ys[i], math.nan)
        if not math.isfinite(y0) or not math.isfinite(y1):
            return math.nan
        if abs(x1 - x0) < 1e-5:
            return y1
        return y0 + (y1 - y0) * clamp((target - x0) / (x1 - x0), 0.0, 1.0)
    return finite(ys[last_idx], math.nan)


def sample_path_z(line: Any, distance: float) -> float:
    zs = _get(line, "z")
    zs = zs if isinstance(zs, list) else []
    idx = path_length_index(line, distance)
    return finite(zs[idx] if idx < len(zs) else None, 0.0)


def sample_path_y(line: Any, distance: float) -> float:
    ys = _get(line, "y")
    return interpolate(distance, _xs(line), ys if isinstance(ys, list) else [])


def _smoothing_gain(quality: str, axis: str) -> float:
    if quality == QUALITY_ROAD_EDGE:
        return 1.12 if axis == "center" else 1.22
    if quality == QUALITY_LANE:
        return 1.06 if axis == "center" else 1.12
    return 1.0


def smooth_projected_polyline(points: list[Point], distances: list[float], maximum_distance: float, max_strength: float) -> list[Point]:
    if len(points) < 3:
        return points
    far = max(POLYLINE_SMOOTH_FAR_DISTANCE, min(finite(maximum_distance, MAX_DRAW_DISTANCE), MAX_DRAW_DISTANCE))
    span = max(1.0, far - POLYLINE_SMOOTH_NEAR_DISTANCE)
    smoothed: list[Point] | None = None
    for i in range(1, len(points) - 1):
        dist = distances[i]
        if dist <= POLYLINE_SMOOTH_NEAR_DISTANCE:
            continue
        strength = max_strength * clamp((dist - POLYLINE_SMOOTH_NEAR_DISTANCE) / span, 0.0, 1.0)
        if strength <= 0.001:
            continue
        (px, py), (cx, cy), (nx, ny) = points[i - 1], points[i], points[i + 1]
        if smoothed is None:
            smoothed = list(points)
        smoothed[i] = (
            cx + (px * 0.25 + cx * 0.5 + nx * 0.25 - cx) * strength,
            cy + (py * 0.25 + cy * 0.5 + ny * 0.25 - cy) * strength,
        )
    return smoothed if smoothed is not None else points


def _catmull_rom_at(p0: Point, p1: Point, p2: Point, p3: Point, t: float) -> Point:
    t1 = max(math.sqrt(math.hypot(p1[0] - p0[0], p1[1] - p0[1])), 1e-4)
    t2 = t1 + max(math.sqrt(math.hypot(p2[0] - p1[0], p2[1] - p1[1])), 1e-4)
    t3 = t2 + max(math.sqrt(math.hypot(p3[0] - p2[0], p3[1] - p2[1])), 1e-4)
    time_value = t1 + (t2 - t1) * t

    def lerp(a: Point, b: Point, ta: float, tb: float) -> Point:
        span = tb - ta
        if not abs(span) > 1e-9:
            return a
        w = (time_value - ta) / span
        return a[0] + (b[0] - a[0]) * w, a[1] + (b[1] - a[1]) * w

    a1 = lerp(p0, p1, 0.0, t1)
    a2 = lerp(p1, p2, t1, t2)
    a3 = lerp(p2, p3, t2, t3)
    b1 = lerp(a1, a2, 0.0, t2)
    b2 = lerp(a2, a3, t1, t3)
    x, y = lerp(b1, b2, t1, t2)
    return clamp(x, min(p1[0], p2[0]), max(p1[0], p2[0])), clamp(y, min(p1[1], p2[1]), max(p1[1], p2[1]))


def subdivide_projected_polyline(points: list[Point], divisions: int = CURVE_SUBDIVISIONS) -> list[Point]:
    steps = max(1, round(divisions))
    if len(points) < 2 or steps == 1:
        return points
    last = len(points) - 1
    dense = [points[0]]
    for i in range(last):
        p0 = points[max(0, i - 1)]
        p1 = points[i]
        p2 = points[i + 1]
        p3 = points[min(last, i + 2)]
        for step in range(1, steps):
            dense.append(_catmull_rom_at(p0, p1, p2, p3, step / steps))
        dense.append(p2)
    return dense


@dataclass(slots=True)
class Ribbon:
    left: list[Point] = field(default_factory=list)
    right: list[Point] = field(default_factory=list)
    center: list[Point] = field(default_factory=list)

    @property
    def polygon(self) -> list[Point]:
        if len(self.left) >= 2 and len(self.right) >= 2:
            return self.left + self.right[::-1]
        return []


def build_ribbon(m, line: Any, half_width: float, z_offset: float, maximum_distance: float,
                 allow_invert: bool = False, center_shift: float = 0.0, quality: str = QUALITY_DEFAULT) -> Ribbon:
    xs = _xs(line)
    ys = _get(line, "y") if isinstance(_get(line, "y"), list) else []
    zs = _get(line, "z") if isinstance(_get(line, "z"), list) else []
    left: list[Point] = []
    right: list[Point] = []
    center: list[Point] = []
    distances: list[float] = []
    max_idx = path_length_index(line, maximum_distance)
    if xs:
        # Full-detail mode visits every model sample (stride 1).
        for i in range(0, max_idx + 1):
            x = finite(xs[i], math.nan)
            if not math.isfinite(x) or x < 0:
                continue
            y = finite(ys[i] if i < len(ys) else None, 0.0) + center_shift
            z = finite(zs[i] if i < len(zs) else None, 0.0) + z_offset
            lp = project_point_precise(m, x, y - half_width, z)
            rp = project_point_precise(m, x, y + half_width, z)
            cp = project_point_precise(m, x, y, z)
            if lp is None or rp is None or cp is None:
                continue
            if not allow_invert and center and cp[1] > center[-1][1]:
                continue
            left.append(lp)
            right.append(rp)
            center.append(cp)
            distances.append(x)
    smooth_max = finite(maximum_distance, MAX_DRAW_DISTANCE)
    side = POLYLINE_SMOOTH_MAX_STRENGTH * _smoothing_gain(quality, "side")
    mid = POLYLINE_CENTER_SMOOTH_MAX_STRENGTH * _smoothing_gain(quality, "center")
    return Ribbon(
        subdivide_projected_polyline(smooth_projected_polyline(left, distances, smooth_max, side)),
        subdivide_projected_polyline(smooth_projected_polyline(right, distances, smooth_max, side)),
        subdivide_projected_polyline(smooth_projected_polyline(center, distances, smooth_max, mid)),
    )


def build_vertical_ribbon(m, line: Any, center_shift: float, top_z: float, bottom_z: float, maximum_distance: float) -> list[Point]:
    xs = _xs(line)
    ys = _get(line, "y") if isinstance(_get(line, "y"), list) else []
    zs = _get(line, "z") if isinstance(_get(line, "z"), list) else []
    top: list[Point] = []
    bottom: list[Point] = []
    distances: list[float] = []
    max_idx = path_length_index(line, maximum_distance)
    if xs:
        for i in range(0, max_idx + 1):
            x = finite(xs[i], math.nan)
            if not math.isfinite(x) or x < 0:
                continue
            y = finite(ys[i] if i < len(ys) else None, 0.0) + center_shift
            z = finite(zs[i] if i < len(zs) else None, 0.0)
            tp = project_point(m, x, y, z + top_z)
            bp = project_point(m, x, y, z + bottom_z)
            if tp is None or bp is None:
                continue
            if top and tp[1] > top[-1][1]:
                continue
            top.append(tp)
            bottom.append(bp)
            distances.append(x)
    smooth_max = finite(maximum_distance, MAX_DRAW_DISTANCE)
    top = smooth_projected_polyline(top, distances, smooth_max, POLYLINE_SMOOTH_MAX_STRENGTH)
    bottom = smooth_projected_polyline(bottom, distances, smooth_max, POLYLINE_SMOOTH_MAX_STRENGTH)
    if len(top) >= 2 and len(bottom) >= 2:
        return top + bottom[::-1]
    return []


def build_band_polygon(left: list[Point], right: list[Point], start: float, end: float) -> list[Point]:
    if len(left) < 2 or len(right) < 2 or len(left) != len(right):
        return []
    first = [(lx + (rx - lx) * start, ly + (ry - ly) * start) for (lx, ly), (rx, ry) in zip(left, right, strict=True)]
    second = [(lx + (rx - lx) * end, ly + (ry - ly) * end) for (lx, ly), (rx, ry) in zip(left, right, strict=True)]
    return first + second[::-1]


def temporal_alpha(dt_ms: float, tau_ms: float) -> float:
    if not math.isfinite(tau_ms) or tau_ms <= 0:
        return 1.0
    if not math.isfinite(dt_ms) or dt_ms <= 0:
        return 0.0
    if dt_ms >= MAX_SMOOTH_DT_MS:
        return 1.0
    return 1.0 - math.exp(-dt_ms / tau_ms)


def _smooth_points(previous: list[Point] | None, nxt: list[Point], alpha: float) -> list[Point]:
    if previous is None or len(previous) != len(nxt):
        return list(nxt)
    a = clamp(alpha, 0.0, 1.0)
    return [(p[0] + (n[0] - p[0]) * a, p[1] + (n[1] - p[1]) * a) for p, n in zip(previous, nxt, strict=True)]


def circle_polygon(cx: float, cy: float, radius: float) -> list[Point]:
    r = max(1.0, finite(radius, 1.0))
    count = round(clamp(math.pi * 2 / math.acos(1 - 0.35 / r), 12, 64))
    return [(cx + math.cos(math.pi * 2 * i / count) * radius, cy + math.sin(math.pi * 2 * i / count) * radius)
            for i in range(count)]


def dash_spans(points: Sequence[Point], pattern: Sequence[float], dash_offset: float) -> list[list[Point]]:
    dashes = [float(value) for value in pattern if float(value) > 0]
    if not dashes:
        return [list(points)]
    cycle = sum(dashes)
    spans: list[list[Point]] = []
    current: list[Point] = []
    index = 0
    remaining = dashes[0]
    drawing = True
    offset = (js_mod(finite(dash_offset, 0.0), cycle) + cycle) % cycle
    while offset > 0:
        step = min(offset, remaining)
        remaining -= step
        offset -= step
        if remaining <= 1e-6:
            index = (index + 1) % len(dashes)
            remaining = dashes[index]
            drawing = not drawing
    for i in range(len(points) - 1):
        a, b = points[i], points[i + 1]
        segment = math.hypot(b[0] - a[0], b[1] - a[1])
        if not segment > 0:
            continue
        t = 0.0
        if drawing and not current:
            current.append(a)
        while segment - t > 1e-6:
            step = min(remaining, segment - t)
            start = t / segment
            t += step
            remaining -= step
            end = t / segment
            point = (a[0] + (b[0] - a[0]) * end, a[1] + (b[1] - a[1]) * end)
            if drawing:
                if not current:
                    current.append((a[0] + (b[0] - a[0]) * start, a[1] + (b[1] - a[1]) * start))
                current.append(point)
            if remaining <= 1e-6:
                if drawing and len(current) > 1:
                    spans.append(current)
                current = []
                index = (index + 1) % len(dashes)
                remaining = dashes[index]
                drawing = not drawing
    if len(current) > 1:
        spans.append(current)
    return spans


# --------------------------------------------------------------------------
# WebGL2 batch (vision_webgl2.js): triangles with per-vertex colors
# --------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class VerticalGradient:
    rgb: tuple[int, int, int]
    bottom_alpha: float
    mid_alpha: float


Fill = RGBA | VerticalGradient


def _cross(a: Point, b: Point, c: Point) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _point_in_triangle(p: Point, a: Point, b: Point, c: Point) -> bool:
    ab, bc, ca = _cross(a, b, p), _cross(b, c, p), _cross(c, a, p)
    negative = ab < -1e-5 or bc < -1e-5 or ca < -1e-5
    positive = ab > 1e-5 or bc > 1e-5 or ca > 1e-5
    return not (negative and positive)


def triangulate(points_in: Sequence[Point]) -> tuple[list[Point], list[int]]:
    points = [p for p in points_in if math.isfinite(p[0]) and math.isfinite(p[1])]
    if len(points) < 3:
        return points, []
    if len(points) >= 4 and len(points) % 2 == 0:
        half = len(points) // 2
        indices: list[int] = []
        for index in range(half - 1):
            r = len(points) - 1 - index
            indices.extend((index, index + 1, r - 1, index, r - 1, r))
        return points, indices
    order = list(range(len(points)))
    area = sum(points[i][0] * points[(i + 1) % len(points)][1] - points[(i + 1) % len(points)][0] * points[i][1]
               for i in range(len(points))) * 0.5
    if area < 0:
        order.reverse()
    triangles: list[int] = []
    guard = len(points) * len(points)
    while len(order) > 3 and guard > 0:
        guard -= 1
        clipped = False
        for i in range(len(order)):
            prev_i = order[(i + len(order) - 1) % len(order)]
            cur_i = order[i]
            next_i = order[(i + 1) % len(order)]
            a, b, c = points[prev_i], points[cur_i], points[next_i]
            if _cross(a, b, c) <= 1e-5:
                continue
            if any(_point_in_triangle(points[k], a, b, c) for k in order if k not in (prev_i, cur_i, next_i)):
                continue
            triangles.extend((prev_i, cur_i, next_i))
            order.pop(i)
            clipped = True
            break
        if not clipped:
            break
    if len(order) == 3:
        triangles.extend(order)
    if not triangles:
        for i in range(1, len(points) - 1):
            triangles.extend((0, i, i + 1))
    return points, triangles


class TriangleBatch:
    """Vertices (x, y, r, g, b, a) in source pixels, drawn as one triangle list."""

    def __init__(self, logical_height: float) -> None:
        self.logical_height = float(logical_height)
        self.vertices: list[tuple[float, float, float, float, float, float]] = []

    def color_at(self, style: Fill, y: float) -> RGBA:
        if not isinstance(style, VerticalGradient):
            return style
        h = self.logical_height
        stop = max(0.0, min(1.0, (h - y) / (h * 0.68)))
        if stop <= 0.55:
            alpha = style.bottom_alpha + (style.mid_alpha - style.bottom_alpha) * (stop / 0.55)
        else:
            alpha = style.mid_alpha * (1 - ((stop - 0.55) / 0.45))
        return (style.rgb[0], style.rgb[1], style.rgb[2], max(0.0, min(1.0, alpha)))

    def fill(self, polygon: Sequence[Point], style: Fill | None) -> None:
        if not style:
            return
        points, indices = triangulate(polygon)
        for index in indices:
            x, y = points[index]
            self.vertices.append((x, y, *self.color_at(style, y)))

    def stroke(self, source: Sequence[Point], style: Fill | None, line_width: float, closed: bool = True) -> None:
        if not style or len(source) < 2:
            return
        half = max(0.25, finite(line_width, 1.0) or 1.0) / 2
        path = [p for p in source if math.isfinite(p[0]) and math.isfinite(p[1])]
        if len(path) < 2:
            return
        if closed and math.hypot(path[-1][0] - path[0][0], path[-1][1] - path[0][1]) > 1e-6:
            path.append(path[0])
        last = len(path) - 1

        def normal(a: Point, b: Point) -> Point | None:
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            if not length > 1e-9:
                return None
            return -dy / length, dx / length

        offsets: list[Point] = []
        for i in range(len(path)):
            before = normal(path[max(0, i - 1)], path[i])
            after = normal(path[i], path[min(last, i + 1)])
            a = before or after
            b = after or before
            if a is None or b is None:
                offsets.append((0.0, 0.0))
                continue
            mx, my = a[0] + b[0], a[1] + b[1]
            length = math.hypot(mx, my)
            if not length > 1e-9:
                offsets.append((a[0] * half, a[1] * half))
                continue
            mx, my = mx / length, my / length
            scale = min(1 / max(mx * a[0] + my * a[1], 0.25), 4)
            offsets.append((mx * half * scale, my * half * scale))
        for i in range(last):
            (x0, y0), (x1, y1) = path[i], path[i + 1]
            (ox0, oy0), (ox1, oy1) = offsets[i], offsets[i + 1]
            for x, y in (
                (x0 + ox0, y0 + oy0), (x1 + ox1, y1 + oy1), (x1 - ox1, y1 - oy1),
                (x0 + ox0, y0 + oy0), (x1 - ox1, y1 - oy1), (x0 - ox0, y0 - oy0),
            ):
                self.vertices.append((x, y, *self.color_at(style, y)))

    def polygon(self, points: Sequence[Point], fill: Fill | None, stroke: Fill | None = None, line_width: float = 1.0) -> None:
        # home_drive.js drawGeometryPolygon on the performance renderer.
        if fill:
            self.fill(points, fill)
        if stroke:
            self.stroke(points, stroke, line_width)

    def polyline(self, points: Sequence[Point], stroke: Fill, line_width: float,
                 dash: Sequence[float] = (), dash_offset: float = 0.0) -> None:
        # home_drive.js drawGeometryPolyline: dashed spans, open strokes.
        if len(points) < 2:
            return
        for span in dash_spans(points, dash, dash_offset):
            self.stroke(span, stroke, line_width, closed=False)


# --------------------------------------------------------------------------
# 2D canvas layer items (lead cards, badges and labels)
# --------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class RoundRect:
    x: float
    y: float
    width: float
    height: float
    radius: float
    fill: RGBA | None = None
    stroke: RGBA | None = None
    stroke_width: float = 1.0


@dataclass(frozen=True, slots=True)
class Label:
    text: str
    x: float
    y: float
    size: float
    weight: int = 800
    fill: RGBA = (255, 255, 255, 1.0)
    stroke: RGBA = (0, 0, 0, 0.86)
    stroke_width: float = 3.4
    align: str = "center"
    baseline: str = "middle"


CanvasItem = RoundRect | Label


@dataclass(slots=True)
class WebVisionFrame:
    transform: StageTransform | None
    video_width: float
    video_height: float
    model_triangles: list = field(default_factory=list)
    triangles: list = field(default_factory=list)
    canvas: list[CanvasItem] = field(default_factory=list)
    model_drawn: bool = False
    model_hidden_reason: str = ""


# --------------------------------------------------------------------------
# Overlay builder (one HomeDrive.renderActiveFrame overlay pass)
# --------------------------------------------------------------------------

PATH_HALF_WIDTH = 0.9
TEST_PATH_VISIBILITY_SOLID_ALPHA = 0.50
TEST_PATH_VISIBILITY_MID_ALPHA = 0.24
TEST_LANE_PROB_MIN = 0.003
TEST_LANE_PROB_BOOST = 6.0
PATH_SMOOTH_TAU_MS = 110.0
LANE_SMOOTH_TAU_MS = 140.0
LANE_VISUAL_WIDTH_GAIN = 1.4
LEAD_EMA_ALPHA = 0.85
C3_FRAME_MS = 50.0
LEAD_HOLD_MS = 1500.0
LONG_PLAN_SOURCE_LEAD1 = 2
DASH_PRESETS = {
    1: (14, 11), 2: (10, 8), 3: (26, 12), 4: (18, 10, 5, 10),
    5: (30, 12), 6: (12, 7), 7: (20, 8, 4, 8), 8: (8, 6),
}


def _rgba(rgb: Sequence[int], alpha: float) -> RGBA:
    return (rgb[0], rgb[1], rgb[2], round(clamp(alpha, 0.0, 1.0), 3))


def _has_nearby_assist_lead(lead: Any, speed_mps: float) -> bool:
    if speed_mps <= 0:
        return False
    d_rel = finite(_get(lead, "dRel"), math.inf)
    return bool(_get(lead, "status")) and 0 < d_rel < speed_mps * 3.0


class WebVisionOverlay:
    """Stateful port of the web overlay; one instance per HUD renderer."""

    def __init__(self, language: str = "ko", measure_text: Callable[[str, float, int], float] | None = None) -> None:
        self.language = language
        self.measure_text = measure_text or (lambda text, size, weight: len(text) * size * 0.6)
        self.reset()

    def reset(self) -> None:
        self._ribbons: dict[str, Ribbon] = {}
        self._geometry_key: tuple | None = None
        self._geometry_triangles: list = []
        self._geometry_dashes: list = []
        self._geometry_at_ms: float | None = None
        self._lead_ema = [(math.nan, math.nan, math.nan, -1.0, 0.0), (math.nan, math.nan, math.nan, -1.0, 0.0)]
        self._lead_two_ema = (math.nan, math.nan, math.nan, -1.0, 0.0)
        self._hold: dict[str, Any] = {"lastValidMs": 0.0, "box": None, "strokeColor": None, "isLeadScc": False,
                                      "radarDist": 0.0, "visionDist": 0.0, "badgeTextColor": (255, 255, 255, 1.0)}
        self._render_state = (math.nan, math.nan, math.nan, math.nan)
        self._calibration = None
        self._calibration_signature: str | None = None
        self._profile_key = "ar_ox"
        self._profile_resolved = False
        self.geometry_builds = 0

    # -- projection state ---------------------------------------------------

    def _calibration_matrix(self, live_calibration: Mapping[str, Any]):
        status = _normalize_enum_name(_get(live_calibration, "calStatus"),
                                      {0: "uncalibrated", 1: "calibrated", 2: "invalid", 3: "recalibrating"})
        if status != "calibrated":
            return self._calibration
        rpy = _get(live_calibration, "rpyCalib")
        if not isinstance(rpy, list) or len(rpy) < 3:
            return self._calibration
        values = [js_number(v) for v in rpy[:3]]
        if not all(math.isfinite(v) for v in values):
            return self._calibration
        signature = "|".join(js_to_fixed(v, 8) for v in values)
        if self._calibration is not None and signature == self._calibration_signature:
            return self._calibration
        self._calibration_signature = signature
        self._calibration = mat3_multiply(VIEW_FROM_DEVICE, rot_from_euler(*values))
        return self._calibration

    def _apply_camera_profile(self, device_type: Any, sensor: Any) -> bool:
        selected = select_road_camera_profile(device_type, sensor)
        if selected is None:
            return self._profile_resolved
        self._profile_resolved = True
        if selected[0] != self._profile_key:
            self._profile_key = selected[0]
            self._ribbons.clear()
            self._geometry_key = None
        return True

    @property
    def base_camera(self) -> tuple[float, float, float]:
        return ROAD_CAMERA_PROFILES[self._profile_key]

    def ui_scale(self, video_width: float, video_height: float) -> float:
        base_w, base_h, _ = self.base_camera
        return clamp(min(video_width / base_w, video_height / base_h), 0.45, 1.0)

    def _projection_scale(self, video_width: float, video_height: float) -> float:
        base_w, base_h, _ = self.base_camera
        return clamp(min(video_width / base_w, video_height / base_h), 0.05, 1.0)

    # -- frame ----------------------------------------------------------------

    def build(self, snapshot: WebVisionSnapshot, params: Mapping[str, float], video_width: float,
              video_height: float, stage_width: float, stage_height: float, now_ms: float) -> WebVisionFrame:
        hud = snapshot.hud_state()
        overlay = snapshot.overlay_state()
        model = overlay.get("modelV2")
        profile_ready = self._apply_camera_profile(
            _get(hud.get("deviceState"), "deviceType"), _get(overlay.get("roadCameraState"), "sensor"))
        calibration = self._calibration_matrix(overlay.get("liveCalibration") or {})
        projection_ready = bool(profile_ready and calibration is not None)
        transform = stage_transform(video_width, video_height, stage_width, stage_height,
                                    calibration or VIEW_FROM_DEVICE, self.base_camera)
        frame = WebVisionFrame(transform, video_width, video_height)
        if not (model and projection_ready):
            frame.model_hidden_reason = "no-model" if not model else "projection"
            return frame
        m = transform.calib
        selected, style = resolve_road_overlay_policy(overlay, hud, params)
        show_lane_info = finite(params["ShowLaneInfo"], 2.0)
        model_usable = snapshot.model_usable()
        geometry = TriangleBatch(video_height)
        if model_usable:
            self._model_geometry(geometry, m, model, overlay, hud, selected, style, params, show_lane_info,
                                 video_height, now_ms, snapshot.sequence)
            frame.model_drawn = True
        else:
            frame.model_hidden_reason = "stale-model" if snapshot.model_valid else "invalid-model"
        frame.model_triangles = geometry.vertices
        live = TriangleBatch(video_height)
        position = _get(model, "position") if model_usable else None
        self._draw_blindspot_barriers(live, m, position, overlay, hud)
        self._draw_leads(live, frame.canvas, m, model, overlay, hud, params, transform, now_ms,
                         model_usable)
        if model_usable:
            self._draw_tf_marker(live, frame.canvas, m, position, hud.get("longitudinalPlan"), params,
                                 video_width, video_height)
        frame.triangles = live.vertices
        return frame

    def _scene_max_distance(self, model: Any, overlay: Mapping[str, Any], now_ms: float) -> float:
        xs = _xs(_get(model, "position"))
        tail = finite(xs[-1] if xs else None, MIN_DRAW_DISTANCE)
        max_distance = clamp(tail, MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
        lead = _get(overlay.get("radarState"), "leadOne") or {}
        live_distance = finite(lead.get("dRel"), math.nan)
        lead_distance = math.nan
        if bool(lead.get("status")) and math.isfinite(live_distance) and live_distance > 0:
            lead_distance = live_distance
        else:
            held = self._held_box(now_ms)
            held_distance = finite(held["dRel"] if held else None, math.nan)
            if math.isfinite(held_distance) and held_distance > 0:
                lead_distance = held_distance
        if math.isfinite(lead_distance) and lead_distance > 0:
            max_distance = min(max_distance, lead_distance)
        return max_distance

    def _held_box(self, now_ms: float):
        box = self._hold["box"]
        if box is None or now_ms - self._hold["lastValidMs"] >= LEAD_HOLD_MS:
            return None
        return box

    # -- model geometry (cached until a new model or a changed input) --------

    def _model_geometry(self, out: TriangleBatch, m, model, overlay, hud, selected, style, params,
                        show_lane_info: float, video_height: float, now_ms: float,
                        sequence: Mapping[str, int]) -> None:
        car_state = hud.get("carState") or {}
        scene_max = self._scene_max_distance(model, overlay, now_ms)
        key = (
            sequence.get("modelV2"), selected["pathSource"],
            sequence.get("lateralPlan") if selected["pathSource"] == "lateralPlan" else None,
            m, scene_max, show_lane_info,
            tuple(sorted((k, v) for k, v in style.items())),
            finite(params["ShowPathWidth"], 100.0),
            finite(car_state.get("leftLaneLine"), 0.0), finite(car_state.get("rightLaneLine"), 0.0),
            video_height,
        )
        animated = 1 <= style["mode"] <= 8
        if key != self._geometry_key:
            dt = DEFAULT_FRAME_DT_MS if self._geometry_at_ms is None else max(0.0, now_ms - self._geometry_at_ms)
            self._geometry_at_ms = now_ms
            batch = TriangleBatch(video_height)
            dashes: list = []
            if show_lane_info >= 1:
                self._draw_lane_lines(batch, m, model, overlay, hud, scene_max, dt)
            if show_lane_info > 1:
                self._draw_road_edges(batch, m, model, scene_max)
            if show_lane_info >= 0:
                dashes = self._draw_path(batch, m, selected["pathData"], style, params, scene_max, dt)
            self._geometry_key = key
            self._geometry_triangles = batch.vertices
            self._geometry_dashes = dashes
            self.geometry_builds += 1
        out.vertices.extend(self._geometry_triangles)
        if animated and self._geometry_dashes:
            center, color, stroke = self._geometry_dashes
            dash = DASH_PRESETS.get(int(style["mode"]), DASH_PRESETS[1])
            dash_length = sum(dash)
            offset = -js_mod(now_ms / 120.0, dash_length)
            out.polyline(center, color, 5.5, dash, offset)
            if stroke is not None:
                out.polyline(center, stroke, 1.6)

    def _smooth(self, key: str, ribbon: Ribbon, tau_ms: float, dt_ms: float) -> Ribbon:
        if not ribbon.polygon:
            return ribbon
        alpha = temporal_alpha(dt_ms, tau_ms)
        previous = self._ribbons.get(key)
        smoothed = Ribbon(
            _smooth_points(previous.left if previous else None, ribbon.left, alpha),
            _smooth_points(previous.right if previous else None, ribbon.right, alpha),
            _smooth_points(previous.center if previous else None, ribbon.center, alpha),
        )
        self._ribbons[key] = smoothed
        return smoothed

    def _draw_lane_lines(self, out: TriangleBatch, m, model, overlay, hud, scene_max: float, dt: float) -> None:
        lane_lines = _get(model, "laneLines")
        lane_lines = lane_lines if isinstance(lane_lines, list) else []
        probs = _get(model, "laneLineProbs")
        probs = probs if isinstance(probs, list) else []
        if not lane_lines:
            return
        car_state = hud.get("carState") or {}
        left_lane_line = finite(car_state.get("leftLaneLine"), 0.0)
        right_lane_line = finite(car_state.get("rightLaneLine"), 0.0)
        max_idx = path_length_index(lane_lines[0], scene_max)
        for i, line in enumerate(lane_lines):
            prob = clamp(finite(probs[i] if i < len(probs) else None, 0.0), 0.0, 0.9)
            if prob >= 0.02:
                render_prob = prob
            elif prob >= TEST_LANE_PROB_MIN:
                render_prob = clamp(prob * TEST_LANE_PROB_BOOST, 0.02, 0.12)
            else:
                render_prob = 0.0
            if render_prob <= 0:
                continue
            highlighted = (i == 1 and math.floor(left_lane_line / 10) == 2) or (
                i == 2 and math.floor(right_lane_line / 10) == 2)
            color = (255, 217, 94) if highlighted else (255, 255, 255)
            half_width = max(0.025 if highlighted else 0.010, 0.025 * render_prob) * LANE_VISUAL_WIDTH_GAIN
            fill_alpha = clamp(render_prob, 0.12, 0.7) if prob >= 0.02 else clamp(render_prob * 3.0, 0.16, 0.26)
            stroke_alpha = 0.20 if prob >= 0.02 else 0.24
            xs = _xs(line)
            max_distance = finite(xs[max_idx] if max_idx < len(xs) else None, MAX_DRAW_DISTANCE)
            fill = _rgba(color, fill_alpha)
            stroke = _rgba(color, stroke_alpha)
            ribbon = self._smooth(f"lane:{i}", build_ribbon(m, line, half_width, 0.0, max_distance, False, 0.0,
                                                            QUALITY_LANE), LANE_SMOOTH_TAU_MS, dt)
            out.polygon(ribbon.polygon, fill, stroke, 1.0)
            if (i == 1 and js_mod(left_lane_line, 10) == 4) or (i == 2 and js_mod(right_lane_line, 10) == 4):
                shift = -0.3 if i == 1 else 0.3
                double = self._smooth(f"lane:{i}:double:{shift}",
                                      build_ribbon(m, line, half_width, 0.0, max_distance, False, shift, QUALITY_LANE),
                                      LANE_SMOOTH_TAU_MS, dt)
                out.polygon(double.polygon, fill, stroke, 1.0)

    def _draw_road_edges(self, out: TriangleBatch, m, model, scene_max: float) -> None:
        edges = _get(model, "roadEdges")
        edges = edges if isinstance(edges, list) else []
        stds = _get(model, "roadEdgeStds")
        stds = stds if isinstance(stds, list) else []
        if not edges:
            return
        max_idx = path_length_index(edges[0], scene_max)
        for i, edge in enumerate(edges):
            edge_std = clamp(finite(stds[i] if i < len(stds) else None, 0.4), 0.0, 1.0)
            alpha = clamp(1 - edge_std, 0.12, 0.66)
            xs = _xs(edge)
            ribbon = build_ribbon(m, edge, 0.025, 0.0, finite(xs[max_idx] if max_idx < len(xs) else None,
                                                              MAX_DRAW_DISTANCE), False, 0.0, QUALITY_ROAD_EDGE)
            out.polygon(ribbon.polygon, _rgba((255, 78, 59), alpha), (255, 124, 104, 0.28), 1.0)

    def _draw_path(self, out: TriangleBatch, m, path_data, style, params, scene_max: float, dt: float):
        if not _xs(path_data):
            return []
        width_ratio = clamp(finite(params["ShowPathWidth"], 100.0) / 100.0, 0.1, 2.0)
        mode = style["mode"]
        mode_key = int(mode) if float(mode).is_integer() else mode
        raw = build_ribbon(m, path_data, PATH_HALF_WIDTH * width_ratio, PATH_Z_OFFSET, max(0.0, scene_max - 2.0), False)
        ribbon = self._smooth(f"path:{'lane' if style['laneMode'] else 'model'}:{mode_key}", raw, PATH_SMOOTH_TAU_MS, dt)
        polygon = ribbon.polygon
        if len(polygon) < 3:
            return []
        base = palette_color(style["paletteIndex"])
        emphasis = style["emphasisStroke"]
        stroke = style["strokeColor"] if emphasis else None
        if mode == 0:
            out.polygon(polygon, _rgba(base, 0.42), stroke, 2.0 if emphasis else 0.0)
            return []
        solid = 0.40 if mode == 0 else 0.24
        mid = 0.20 if mode == 0 else 0.12
        if mode == 0 or style["isCruiseOff"]:
            solid = max(solid, TEST_PATH_VISIBILITY_SOLID_ALPHA)
            mid = max(mid, TEST_PATH_VISIBILITY_MID_ALPHA)
        out.polygon(polygon, VerticalGradient(base, solid, mid), stroke, 1.7 if emphasis else 0.0)
        if 13 <= mode <= 15:
            bands = []
            if mode in (13, 14):
                bands.append(build_band_polygon(ribbon.left, ribbon.right, 0.0, 0.18))
                bands.append(build_band_polygon(ribbon.left, ribbon.right, 0.82, 1.0))
            if mode in (13, 15):
                bands.append(build_band_polygon(ribbon.left, ribbon.right, 0.38, 0.62))
            for band in bands:
                out.polygon(band, _rgba(base, 0.34), stroke, 1.4 if emphasis else 0.0)
            return []
        if mode >= 9:
            step = 3 if mode == 9 else 4 if mode == 10 else 5 if mode == 11 else 6
            left, right = ribbon.left, ribbon.right
            if len(left) >= 3 and len(left) == len(right):
                for i in range(0, len(left) - 2, step):
                    nxt = min(i + 2, len(left) - 1)
                    out.polygon([left[i], left[nxt], right[nxt], right[i]], _rgba(base, 0.28), stroke, 1.1)
            return []
        if not ribbon.center:
            return []
        return [ribbon.center, _rgba(base, 0.78), stroke]

    # -- live layers ----------------------------------------------------------

    def _draw_blindspot_barriers(self, out: TriangleBatch, m, position, overlay, hud) -> None:
        if len(_xs(position)) < 2:
            return
        radar_state = overlay.get("radarState") or {}
        car_state = hud.get("carState") or {}
        lateral_plan = overlay.get("lateralPlan") or {}
        speed = finite(car_state.get("vEgo"), finite(car_state.get("vEgoCluster"), 0.0))
        lane_change_state = finite(lateral_plan.get("laneChangeState"), 0.0)
        lane_change_direction = finite(lateral_plan.get("laneChangeDirection"), 0.0)
        left_bsd = bool(car_state.get("leftBlindspot"))
        right_bsd = bool(car_state.get("rightBlindspot"))
        left_assist = (not left_bsd and lane_change_state == 1 and lane_change_direction == 1
                       and _has_nearby_assist_lead(radar_state.get("leadLeft"), speed))
        right_assist = (not right_bsd and lane_change_state == 1 and lane_change_direction == 2
                        and _has_nearby_assist_lead(radar_state.get("leadRight"), speed))
        if not (left_bsd or right_bsd or left_assist or right_assist):
            return
        gold = ((255, 215, 0, 0.48), (255, 215, 0, 0.84))
        green = ((0, 204, 0, 0.44), (0, 204, 0, 0.80))

        def ribbon(shift: float, colors) -> None:
            polygon = build_vertical_ribbon(m, position, shift, 1.15, 0.60, 40.0)
            if len(polygon) < 8:
                return
            out.polygon(polygon, colors[0], colors[1], 1.2)

        if left_bsd:
            ribbon(-1.7, gold)
        elif left_assist:
            ribbon(-1.7, green)
        if right_bsd:
            ribbon(1.7, gold)
        elif right_assist:
            ribbon(1.7, green)

    def _badge_offsets(self, vw: float, vh: float) -> dict[str, float]:
        s = self.ui_scale(vw, vh)
        return {"dx": 80 * s, "rectTopOffset": 25 * s, "textBaselineOffset": 60 * s,
                "badgeHeight": max(42 * s, 20), "fontSize": max(40 * s, 18), "radius": max(12 * s, 7),
                "strokeWidth": max(4 * s, 2.2)}

    def _clamp_margins(self, vw: float, vh: float, transform: StageTransform | None,
                       include_distance_badge: bool = True, include_state_text: bool = False) -> dict[str, Any]:
        left, top, right, bottom = visible_source_rect(transform, vw, vh)
        s = self.ui_scale(vw, vh)
        top_reserve = max(200.0 * s, 96.0)
        bottom_reserve = max(80.0 * s, 42.0)
        side_reserve = max(350.0 * s, 120.0)
        top_margin = max(top_reserve, top + 6 * s)
        max_center_y = vh - bottom_reserve
        offsets = self._badge_offsets(vw, vh)
        badge_reserve = 0.0
        if include_distance_badge:
            badge_reserve = max(badge_reserve, offsets["rectTopOffset"] + offsets["badgeHeight"] + 8 * s)
        if include_state_text:
            badge_reserve = max(badge_reserve, offsets["textBaselineOffset"] + max(offsets["fontSize"] * 0.28, 8 * s))
        max_center_y = min(max_center_y, bottom - max(badge_reserve, bottom_reserve))
        max_center_y = max(top_margin, max_center_y)
        side_inset = min(side_reserve, max(0.0, (right - left) * 0.28))
        min_cx = left + side_inset
        return {"marginX": side_reserve, "minCenterX": min_cx, "maxCenterX": max(min_cx, right - side_inset),
                "topMargin": top_margin, "bottomMargin": max(bottom_reserve, vh - max_center_y),
                "maxCenterY": max_center_y, "visible": (left, top, right, bottom)}

    def _sync_render_state(self, vw: float, vh: float, model_frame_id: Any, camera_frame_id: Any) -> None:
        width, height, last_model, last_camera = self._render_state
        next_model = finite(model_frame_id, math.nan)
        next_camera = finite(camera_frame_id, math.nan)
        source_changed = math.isfinite(width) and math.isfinite(height) and (
            abs(width - vw) > 0.5 or abs(height - vh) > 0.5)
        camera_rewind = math.isfinite(last_camera) and math.isfinite(next_camera) and next_camera < last_camera
        model_rewind = math.isfinite(last_model) and math.isfinite(next_model) and next_model < last_model
        if source_changed or camera_rewind or model_rewind:
            self._lead_ema = [(math.nan, math.nan, math.nan, -1.0, 0.0), (math.nan, math.nan, math.nan, -1.0, 0.0)]
            self._lead_two_ema = (math.nan, math.nan, math.nan, -1.0, 0.0)
            self._hold["box"] = None
        self._render_state = (vw, vh, next_model, next_camera)

    def _project_lead_box(self, lead, model_path, m, vw, vh, transform, now_ms, include_badge, include_text):
        if not (isinstance(lead, Mapping) and lead.get("status")):
            return None
        distance = finite(lead.get("dRel"), math.nan)
        if not math.isfinite(distance) or distance <= 0:
            return None
        y_center = -finite(lead.get("yRel"), 0.0)
        z = sample_path_z(model_path, distance) + PATH_Z_OFFSET
        left = project_point_precise(m, distance, y_center - 1.2, z)
        right = project_point_precise(m, distance, y_center + 1.2, z)
        if left is None or right is None:
            return None
        raw_width = abs(right[0] - left[0])
        if not math.isfinite(raw_width) or raw_width <= 1:
            return None
        c = self._clamp_margins(vw, vh, transform, include_badge, include_text)
        v_left, v_top, v_right, _ = c["visible"]
        px = clamp((left[0] + right[0]) * 0.5, c["minCenterX"], c["maxCenterX"])
        py = clamp((left[1] + right[1]) * 0.5, c["topMargin"], c["maxCenterY"])
        s = self.ui_scale(vw, vh)
        ps = self._projection_scale(vw, vh)
        side_pad = max(10 * s, 5)
        horizontal = max(1.0, 2 * max(0.0, min(px - v_left, v_right - px) - side_pad))
        vertical = max(1.0, (c["maxCenterY"] - v_top) / 0.8)
        max_width = max(1.0, min(800 * ps, horizontal, vertical))
        min_width = min(120 * ps, max_width)
        pw = clamp(raw_width, min_width, max_width)
        fx, fy, fw, ema_track, last_ms = self._lead_ema[0]
        track = finite(lead.get("radarTrackId"), -1.0) if lead.get("radarTrackId") is not None else -1.0
        dt = clamp(now_ms - last_ms, 0.0, 500.0) if last_ms > 0 else C3_FRAME_MS
        alpha = LEAD_EMA_ALPHA ** (dt / C3_FRAME_MS) if dt > 0 else 1.0
        has_prev = ema_track == track and math.isfinite(fx) and math.isfinite(fy) and math.isfinite(fw)
        sfx = fx * alpha + px * (1 - alpha) if has_prev else px
        sfy = fy * alpha + py * (1 - alpha) if has_prev else py
        sfw = fw * alpha + pw * (1 - alpha) if has_prev else pw
        width = max(math.trunc(clamp(sfw, min_width, max_width)), 1)
        height = max(math.trunc(width * 0.8), _js_round(12 * s))
        safe_min_x = v_left + side_pad + width * 0.5
        safe_max_x = v_right - side_pad - width * 0.5
        path_x = math.trunc(clamp(sfx, safe_min_x, safe_max_x) if safe_max_x >= safe_min_x
                            else v_left + (v_right - v_left) * 0.5)
        safe_min_y = min(c["maxCenterY"], max(c["topMargin"], v_top + height))
        path_y = math.trunc(clamp(sfy, safe_min_y, c["maxCenterY"]))
        self._lead_ema[0] = (float(path_x), float(path_y), float(width), track, now_ms)
        return {"rect": (path_x - width * 0.5 - side_pad, path_y - height, width + side_pad * 2, height),
                "centerX": path_x, "centerY": path_y, "radar": bool(lead.get("radar")),
                "radarDetected": track >= 0, "radarTrackId": track, "dRel": distance}

    def _project_lead_two_box(self, lead, model_path, m, vw, vh, transform, now_ms):
        if not (isinstance(lead, Mapping) and lead.get("status") and lead.get("radar")):
            return None
        distance = finite(lead.get("dRel"), math.nan)
        if not math.isfinite(distance) or distance <= 0:
            return None
        y_center = -finite(lead.get("yRel"), 0.0)
        z = sample_path_z(model_path, distance) + PATH_Z_OFFSET
        left = project_point_precise(m, distance, y_center - 1.2, z)
        right = project_point_precise(m, distance, y_center + 1.2, z)
        if left is None or right is None:
            return None
        pxl, pxr, py, ema_track, last_ms = self._lead_two_ema
        track = finite(lead.get("radarTrackId"), -1.0) if lead.get("radarTrackId") is not None else -1.0
        has_prev = ema_track == track and math.isfinite(pxl) and math.isfinite(pxr) and math.isfinite(py)
        dt = clamp(now_ms - last_ms, 0.0, 500.0) if last_ms > 0 else C3_FRAME_MS
        a2 = 0.8 ** (dt / C3_FRAME_MS) if dt > 0 else 1.0
        xl = pxl * a2 + left[0] * (1 - a2) if has_prev else left[0]
        xr = pxr * a2 + right[0] * (1 - a2) if has_prev else right[0]
        y = py * a2 + left[1] * (1 - a2) if has_prev else left[1]
        self._lead_two_ema = (xl, xr, y, track, now_ms)
        c = self._clamp_margins(vw, vh, transform, False, False)
        v_left, v_top, v_right, _ = c["visible"]
        cx = clamp((xl + xr) * 0.5, c["minCenterX"], c["maxCenterX"])
        s = self.ui_scale(vw, vh)
        ps = self._projection_scale(vw, vh)
        side_pad = max(10 * s, 5)
        horizontal = max(1.0, 2 * max(0.0, min(cx - v_left, v_right - cx) - side_pad))
        vertical = max(1.0, (c["maxCenterY"] - v_top) / 0.8)
        max_width = max(1.0, min(800 * ps, horizontal, vertical))
        min_width = min(120 * ps, max_width)
        width = max(math.trunc(clamp(max(xr - xl, 1.0), min_width, max_width)), 1)
        height = max(math.trunc(width * 0.8), _js_round(12 * s))
        safe_min_x = v_left + side_pad + width * 0.5
        safe_max_x = v_right - side_pad - width * 0.5
        center_x = clamp(cx, safe_min_x, safe_max_x) if safe_max_x >= safe_min_x else v_left + (v_right - v_left) * 0.5
        y_int = math.trunc(clamp(y, min(c["maxCenterY"], max(c["topMargin"], v_top + height)), c["maxCenterY"]))
        xl_int = math.trunc(center_x - width * 0.5)
        return {"rect": (xl_int - side_pad, y_int - height, width + side_pad * 2, height), "dRel": distance}

    def _path_end_anchor(self, model_path, m, vw, vh):
        xs = _xs(model_path)
        ys = _get(model_path, "y") if isinstance(_get(model_path, "y"), list) else []
        zs = _get(model_path, "z") if isinstance(_get(model_path, "z"), list) else []
        if not xs or not ys or not zs:
            return None
        tail = clamp(finite(xs[-1], 0.0), MIN_DRAW_DISTANCE, MAX_DRAW_DISTANCE)
        idx = path_length_index(model_path, tail)
        distance = finite(xs[idx], math.nan)
        line_y = finite(ys[idx] if idx < len(ys) else None, math.nan)
        line_z = finite(zs[idx] if idx < len(zs) else None, math.nan)
        if not (math.isfinite(distance) and math.isfinite(line_y) and math.isfinite(line_z)):
            return None
        left = project_point_precise(m, distance, line_y - 1.2, line_z + PATH_Z_OFFSET)
        right = project_point_precise(m, distance, line_y + 1.2, line_z + PATH_Z_OFFSET)
        if left is None or right is None:
            return None
        if not abs(right[0] - left[0]) > 1:
            return None
        c = self._clamp_margins(vw, vh, None)
        cx = clamp((left[0] + right[0]) * 0.5, c["marginX"], max(c["marginX"], vw - c["marginX"]))
        cy = clamp((left[1] + right[1]) * 0.5, c["topMargin"], max(c["topMargin"], vh - c["bottomMargin"]))
        return {"centerX": math.trunc(cx), "centerY": math.trunc(cy)}

    def _card(self, canvas: list, box, stroke: RGBA, fill: RGBA, primary: bool, vw: float, vh: float) -> None:
        x, y, w, h = box["rect"]
        s = self.ui_scale(vw, vh)
        radius = max((15 if primary else 12) * s, 7 if primary else 6)
        stroke_width = max((3.0 if primary else 2.2) * s, 1.7 if primary else 1.3)
        canvas.append(RoundRect(x, y, w, h, radius, fill))
        canvas.append(RoundRect(x, y, w, h, radius, None, stroke, stroke_width))

    def _distance_badges(self, canvas: list, box, radar_dist, vision_dist, is_scc, text_color, vw, vh, transform):
        o = self._badge_offsets(vw, vh)
        _, v_top, _, v_bottom = visible_source_rect(transform, vw, vh)
        base_y = min(box["centerY"] + o["rectTopOffset"], max(v_top + 4, v_bottom - o["badgeHeight"] - 4))
        s = self.ui_scale(vw, vh)

        def badge(text: str, center_x: float, background: RGBA) -> None:
            width = max(52 * s, 16 * s + len(text) * o["fontSize"] * 0.62)
            canvas.append(RoundRect(center_x - width * 0.5, base_y, width, o["badgeHeight"], o["radius"], background))
            canvas.append(Label(text, center_x, base_y + o["badgeHeight"] * 0.54, o["fontSize"], 800, text_color,
                                (0, 0, 0, 0.82), o["strokeWidth"]))

        radar_color = (255, 0, 0, 0.92) if is_scc else (255, 175, 3, 0.92)
        vision_color = (0, 0, 255, 0.92)
        radar_text = js_to_fixed(radar_dist, 1) if radar_dist < 10 else js_to_fixed(radar_dist, 0)
        vision_text = js_to_fixed(vision_dist, 1) if vision_dist < 10 else js_to_fixed(vision_dist, 0)
        if radar_dist > 0 and vision_dist > 0:
            badge(radar_text, box["centerX"] - o["dx"], radar_color)
            badge(vision_text, box["centerX"] + o["dx"], vision_color)
        elif radar_dist > 0:
            badge(radar_text, box["centerX"] - o["dx"], radar_color)
        elif vision_dist > 0:
            badge(vision_text, box["centerX"] + o["dx"], vision_color)

    def _state_badge(self, canvas: list, box, text: str, vw, vh, transform) -> None:
        if not text:
            return
        s = self.ui_scale(vw, vh)
        o = self._badge_offsets(vw, vh)
        _, v_top, _, v_bottom = visible_source_rect(transform, vw, vh)
        text_y = min(box["centerY"] + o["textBaselineOffset"], max(v_top + 6 * s, v_bottom - 6 * s))
        canvas.append(Label(text, box["centerX"], text_y, max(50 * s, 22), 900, (255, 255, 255, 1.0),
                            (0, 0, 0, 0.88), max(4.0 * s, 2.2), "center", "bottom"))

    def _draw_leads(self, gpu: TriangleBatch, canvas: list, m, model, overlay, hud, params, transform,
                    now_ms: float, model_usable: bool) -> None:
        vw, vh = transform.video_width, transform.video_height
        radar_state = overlay.get("radarState") or {}
        model_path = _get(model, "position")
        lead_state = resolve_lead_state(overlay, hud, self.language)
        self._sync_render_state(vw, vh, _get(model, "frameId"), _get(overlay.get("roadCameraState"), "frameId"))
        lead_one = radar_state.get("leadOne")
        box = self._project_lead_box(lead_one, model_path, m, vw, vh, transform, now_ms,
                                     lead_state["showDistanceBadge"] is not False, bool(lead_state["text"]))
        anchor = None
        hold = self._hold
        if box is not None:
            is_scc = box["radarTrackId"] < 1
            stroke = (0, 0, 255, 0.96) if not box["radarDetected"] else (
                (255, 0, 0, 0.96) if is_scc else (255, 175, 3, 0.96))
            self._card(canvas, box, stroke, (0, 0, 0, 0.20), True, vw, vh)
            hold.update(lastValidMs=now_ms, box=box, strokeColor=stroke, isLeadScc=is_scc)
            if lead_state["showDistanceBadge"] is not False:
                radar_dist = max(0.0, finite(lead_one.get("dRel"), 0.0)) if bool(lead_one.get("radar")) else 0.0
                vision_dist = primary_vision_distance(model)
                hold.update(radarDist=radar_dist, visionDist=vision_dist)
                if radar_dist > 0 or vision_dist > 0:
                    color = distance_badge_text_color(lead_state["xState"])
                    hold["badgeTextColor"] = color
                    self._distance_badges(canvas, box, radar_dist, vision_dist, is_scc, color, vw, vh, transform)
            self._state_badge(canvas, box, lead_state["text"], vw, vh, transform)
            anchor = box
        elif hold["box"] is not None and now_ms - hold["lastValidMs"] < LEAD_HOLD_MS:
            held = hold["box"]
            self._card(canvas, held, hold["strokeColor"], (0, 0, 0, 0.20), True, vw, vh)
            if lead_state["showDistanceBadge"] is not False and (hold["radarDist"] > 0 or hold["visionDist"] > 0):
                self._distance_badges(canvas, held, hold["radarDist"], hold["visionDist"], hold["isLeadScc"],
                                      hold["badgeTextColor"], vw, vh, transform)
            self._state_badge(canvas, held, lead_state["text"], vw, vh, transform)
            anchor = held
        else:
            hold["box"] = None

        lead_two = radar_state.get("leadTwo") or {}
        valid_two = (bool(lead_two.get("status")) and bool(lead_two.get("radar"))
                     and finite(lead_two.get("dRel"), 0.0) > finite(_get(lead_one, "dRel"), 0.0) + 3)
        status_two = 2 if finite(_get(hud.get("longitudinalPlan"), "longitudinalPlanSource"), 0.0) == LONG_PLAN_SOURCE_LEAD1 else 1
        if valid_two:
            box_two = self._project_lead_two_box(lead_two, model_path, m, vw, vh, transform, now_ms)
            if box_two is not None:
                self._card(canvas, box_two, (218, 111, 37, 0.96),
                           (255, 0, 0, 0.20) if status_two == 2 else (0, 0, 0, 0.20), False, vw, vh)
        else:
            self._lead_ema[1] = (math.nan, math.nan, math.nan, -1.0, 0.0)
            self._lead_two_ema = (math.nan, math.nan, math.nan, -1.0, 0.0)

        if anchor is not None or model_usable:
            self._path_status(canvas, model_path, hud, m, vw, vh, anchor)
        self._radar_targets(gpu, canvas, radar_state, model, m, params, vw, vh)

    def _path_status(self, canvas: list, model_path, hud, m, vw, vh, anchor) -> None:
        text = path_status_text(hud)
        if not text:
            return
        anchor = anchor or self._path_end_anchor(model_path, m, vw, vh)
        if not anchor:
            return
        s = self.ui_scale(vw, vh)
        font_size = max(50 * s, 22)
        baseline_y = clamp(anchor["centerY"] + 60 * s, font_size + 8 * s, vh - 10 * s)
        canvas.append(Label(text, anchor["centerX"], baseline_y, font_size, 900, (255, 255, 255, 1.0),
                            (0, 0, 0, 0.88), max(4.0 * s, 2.2), "center", "bottom"))

    def _radar_targets(self, gpu: TriangleBatch, canvas: list, radar_state, model, m, params, vw, vh) -> None:
        show = finite(params["ShowRadarInfo"], 0.0)
        if show <= 0:
            return
        line = radar_projection_line(model)
        if not line:
            return
        factor = finite(params["RadarLatFactor"], 0.0) / 100.0
        is_metric = finite(params["IsMetric"], 1.0) != 0
        s = self.ui_scale(vw, vh)
        for radar in radar_tracks(radar_state):
            d_rel = finite(radar.get("dRel"), 0.0)
            if d_rel <= 2.5:
                continue
            y_rel = finite(radar.get("yRel"), 0.0)
            z = sample_path_z(line, d_rel) - 0.61
            center = project_point_precise(m, d_rel, -y_rel, z)
            if center is None:
                continue
            v_lead = finite(radar.get("vLeadK"), finite(radar.get("vRel"), 0.0))
            v_lat = finite(radar.get("vLat"), 0.0)
            v_abs = math.sqrt(v_lead * v_lead + v_lat * v_lat)
            v_signed = v_abs if v_lead >= 0 else -v_abs
            model_prob = finite(radar.get("modelProb"), 0.0)
            if v_abs > 3.0:
                future = None
                if abs(v_lead) > 3.0:
                    future = project_point_precise(m, max(2.0, d_rel + v_lead * factor), -(y_rel + v_lat * factor), z)
                if future is not None:
                    color = (35, 213, 93, 0.94) if v_signed >= 0 else (255, 59, 48, 0.94)
                    gpu.polyline([center, future], color, max(3.0 * s, 1.6))
                    gpu.polygon(circle_polygon(future[0], future[1], max(10 * s, 5)), color)
                badge_color = (255, 59, 48, 0.96)
                if not bool(radar.get("radar")):
                    badge_color = (61, 123, 255, 0.96)
                elif abs(model_prob - 0.01) < 1e-3:
                    badge_color = (35, 213, 93, 0.96)
                elif v_signed > 0:
                    badge_color = (255, 167, 38, 0.96)
                text = js_to_fixed(v_signed * (3.6 if is_metric else 2.2369363), 0)
                badge_width = max(40 * s, 35 * s * len(text))
                canvas.append(RoundRect(center[0] - badge_width * 0.5, center[1] - 35 * s, badge_width,
                                        max(42 * s, 20), max(15 * s, 7), badge_color))
                canvas.append(Label(text, center[0], center[1], max(40 * s, 18), 900, stroke_width=max(4.2 * s, 2.2)))
                if show >= 2:
                    unit = 1.0 if is_metric else 3.28084
                    canvas.append(Label(js_to_fixed(y_rel * unit, 1), center[0], center[1] - 40 * s, max(30 * s, 15),
                                        900, stroke_width=max(3.8 * s, 2.0)))
                    canvas.append(Label(js_to_fixed(d_rel * unit, 1), center[0], center[1] + 30 * s, max(30 * s, 15),
                                        900, stroke_width=max(3.8 * s, 2.0)))
            elif show >= 3:
                canvas.append(Label("*", center[0], center[1], max(40 * s, 18), 900, stroke_width=max(4.2 * s, 2.2)))

    def _draw_tf_marker(self, gpu: TriangleBatch, canvas: list, m, position, long_plan, params, vw, vh) -> None:
        if finite(params["ShowPathEnd"], 0.0) <= 0:
            return
        tf_distance = finite(_get(long_plan, "desiredDistance"), 0.0)
        if tf_distance <= 0:
            return
        xs = _xs(position)
        if len(xs) < 2:
            return
        last_x = finite(xs[-1], 0.0)
        if not last_x or tf_distance > last_x:
            return
        zs = _get(position, "z") if isinstance(_get(position, "z"), list) else []
        line_y = sample_path_y(position, tf_distance)
        line_z = interpolate(tf_distance, xs, zs)
        if not math.isfinite(line_y) or not math.isfinite(line_z):
            return
        left = project_point(m, tf_distance, line_y - 1.0, line_z + PATH_Z_OFFSET)
        right = project_point(m, tf_distance, line_y + 1.0, line_z + PATH_Z_OFFSET)
        if left is None or right is None:
            return
        s = self.ui_scale(vw, vh)
        gpu.polyline([left, right], WHITE_92, max(3.0 * s, 1.6))
        label = f"{js_to_fixed(tf_distance, 0)} m"
        font_size = max(20 * s, 12)
        width = max(self.measure_text(label, font_size, 800), 1.0)
        x = clamp(right[0] + 10, 8, max(8, vw - width - 8))
        y = clamp(right[1] - 4, font_size + 8, max(font_size + 8, vh - 8))
        canvas.append(Label(label, x, y, font_size, 800, align="left", stroke_width=max(3.4 * s, 1.8)))


def _js_round(value: float) -> int:
    return math.floor(value + 0.5)


# --------------------------------------------------------------------------
# Plot (home_drive.js buildPlotData / drawPlot)
# --------------------------------------------------------------------------

PLOT_MAX_POINTS = 600
PLOT_COLORS = ((0xf6, 0xd9, 0x4f), (0x58, 0xd9, 0x7d), (0xff, 0x9f, 0x43))


def build_plot_data(snapshot: WebVisionSnapshot, params: Mapping[str, float]) -> dict[str, Any] | None:
    mode = finite(params["ShowPlotMode"], 0.0)
    if not mode:
        return None
    hud = snapshot.hud_state()
    overlay = snapshot.overlay_state()
    car_state = hud.get("carState") or {}
    controls = hud.get("controlsState") or {}
    long_plan = hud.get("longitudinalPlan") or {}
    actuators = _get(overlay.get("carControl"), "actuators") or {}
    model = overlay.get("modelV2") or {}
    lead = _get(overlay.get("radarState"), "leadOne") or {}
    live_parameters = overlay.get("liveParameters") or {}
    a_ego = finite(car_state.get("aEgo"), 0.0)
    v_ego = finite(car_state.get("vEgo"), finite(car_state.get("vEgoCluster"), 0.0))
    accel_target = first_finite(long_plan.get("accels"), 0.0)
    speed_target = first_finite(long_plan.get("speeds"), 0.0)
    pos_x = _xs(_get(model, "position"))
    vel_x = _xs(_get(model, "velocity"))
    pos32 = finite(pos_x[32] if len(pos_x) > 32 else None, first_finite(pos_x, 0.0))
    vel32 = finite(vel_x[32] if len(vel_x) > 32 else None, first_finite(vel_x, 0.0))
    vel0 = finite(vel_x[0] if vel_x else None, 0.0)
    curvature = finite(actuators.get("curvature"), 0.0) * 10000
    table = {
        1: ("1.Accel (Y:a_ego, G:a_target, O:a_out)", (a_ego, accel_target, finite(actuators.get("accel"), 0.0))),
        2: ("2.Speed/Accel(Y:speed_0, G:v_ego, O:a_ego)", (speed_target, v_ego, a_ego)),
        3: ("3.Model(Y:pos_32, G:vel_32, O:vel_0)", (pos32, vel32, vel0)),
        4: ("4.Lead(Y:accel, G:a_lead, O:v_rel)", (accel_target, finite(lead.get("aLeadK"), 0.0), finite(lead.get("vRel"), 0.0))),
        5: ("5.Lead(Y:a_ego, G:a_lead, O:j_lead)", (a_ego, finite(lead.get("aLead"), 0.0), finite(lead.get("jLead"), 0.0))),
        6: ("6.Steer(Y:actual, G:desire, O:output)", (finite(controls.get("actualLateralAccel"), 0.0) * 10.0,
                                                     finite(controls.get("desiredLateralAccel"), 0.0) * 10.0,
                                                     finite(controls.get("lateralOutput"), 0.0) * 10.0)),
        7: ("7.SteerA(Y:Actual, G:Target, O:Offset*10)", (finite(car_state.get("steeringAngleDeg"), 0.0),
                                                         finite(actuators.get("steeringAngleDeg"), 0.0),
                                                         finite(live_parameters.get("angleOffsetDeg"), 0.0) * 10.0)),
        8: ("8.SteerA(Y:Actual, G:Target, O:Offset*10)", (curvature, curvature, curvature)),
    }
    title, values = table.get(int(mode) if float(mode).is_integer() else -1, ("no data", (0.0, 0.0, 0.0)))
    return {"mode": mode, "title": title, "values": values}


class WebVisionPlotHistory:
    def __init__(self) -> None:
        self.mode: float | None = None
        self.values: list[tuple[float, float, float]] = []

    def push(self, plot: dict[str, Any] | None) -> None:
        if plot is None:
            self.mode = None
            self.values.clear()
            return
        if plot["mode"] != self.mode:
            self.mode = plot["mode"]
            self.values.clear()
        self.values.append(tuple(finite(v, 0.0) for v in plot["values"]))
        if len(self.values) > PLOT_MAX_POINTS:
            del self.values[0]

    def bounds(self) -> tuple[float, float]:
        flat = [v for sample in self.values for v in sample]
        low = min(flat) if flat else -2.0
        high = max(flat) if flat else 2.0
        low = min(low, -2.0)
        high = max(high, 2.0)
        if abs(high - low) < 1e-3:
            high += 1
            low -= 1
        return low, high


# --------------------------------------------------------------------------
# HUD payload (vision_raw.js deriveCompactHudPayload + data_bridge.js + hud/index.js mapPayload)
# --------------------------------------------------------------------------

def _finite_or_none(value: Any) -> float | None:
    if value is None or value == "":
        return None
    number = js_number(value)
    return number if math.isfinite(number) else None


def _valid_cruise_kph(value: Any) -> float | None:
    number = _finite_or_none(value)
    return number if number is not None and 0 < number < 250 else None


def resolve_cruise_kph(state: Mapping[str, Any]) -> float | None:
    car_state = state.get("carState") or {}
    controls = state.get("controlsState") or {}
    cruise = car_state.get("cruiseState") or {}
    if cruise.get("available") is False:
        return None
    for value in (car_state.get("vCruiseCluster"), car_state.get("vCruise"),
                  controls.get("vCruiseCluster"), controls.get("vCruise")):
        kph = _valid_cruise_kph(value)
        if kph is not None:
            return kph
    for value in (cruise.get("speedCluster"), cruise.get("speed")):
        mps = _finite_or_none(value)
        if mps is not None and 0.1 < mps < 70:
            return mps * 3.6
    return None


def is_cruise_display_visible(state: Mapping[str, Any], cruise_kph: float | None) -> bool:
    if _valid_cruise_kph(cruise_kph) is None:
        return False
    enabled = _optional_bool(_get(state.get("selfdriveState"), "enabled"))
    if enabled is not None:
        return enabled
    enabled = _optional_bool(_get(state.get("controlsState"), "enabled"))
    if enabled is not None:
        return enabled
    return _get(_get(state.get("carState"), "cruiseState"), "available") is not False


_EXTERNAL_NAVI_SOURCES = {"cam", "section", "bump", "police", "waze", "road", "atc", "atc2", "route"}
_VEHICLE_NAVI_SOURCES = {"hda", "hda_section", "hda_bump", "school"}
_DECEL_SOURCE_LABELS = {
    "cam": "cam", "section": "section", "bump": "bump", "police": "police", "waze": "waze", "road": "road",
    "atc": "turn", "atc2": "turn", "route": "route", "hda": "cam", "hda_section": "section", "hda_bump": "bump",
    "school": "school", "gas": "gas", "vturn": "turn", "model": "turn", "turn": "turn",
}


def deceleration_source_presentation(source: Any) -> tuple[str, int]:
    normalized = str(source or "").strip().lower()
    if not normalized:
        return "apply", 2
    if normalized in _VEHICLE_NAVI_SOURCES:
        return _DECEL_SOURCE_LABELS.get(normalized, normalized[:8]), 3
    if normalized in _EXTERNAL_NAVI_SOURCES or normalized.endswith(":n"):
        base = normalized[:-2] if normalized.endswith(":n") else normalized
        return _DECEL_SOURCE_LABELS.get(base, base[:8]), 4
    if normalized.endswith(":v"):
        base = normalized[:-2]
        return _DECEL_SOURCE_LABELS.get(base, base[:8]), 3 if base in ("cam", "section", "bump", "school") else 2
    if normalized.endswith(":c"):
        base = normalized[:-2]
        return _DECEL_SOURCE_LABELS.get(base, base[:8]), 2
    return _DECEL_SOURCE_LABELS.get(normalized, normalized[:8]), 2


def derive_cruise_override(state: Mapping[str, Any]) -> dict[str, Any] | None:
    cruise_kph = resolve_cruise_kph(state)
    visible = is_cruise_display_visible(state, cruise_kph)
    target = _finite_or_none(_get(state.get("longitudinalPlan"), "cruiseTarget"))
    if visible and target is not None and cruise_kph is not None and target > cruise_kph + 0.5:
        return {"kph": target, "label": "eco", "mode": 1}
    if not visible:
        return None
    desired = _finite_or_none(_get(state.get("carrotMan"), "desiredSpeed"))
    if desired is not None and 0 < desired < 200 and cruise_kph is not None and desired < cruise_kph:
        label, mode = deceleration_source_presentation(_get(state.get("carrotMan"), "desiredSource"))
        return {"kph": desired, "label": label, "mode": mode}
    return None


CRUISE_OVERRIDE_HOLD_MS = 2500.0


class CruiseOverrideHold:
    """data_bridge.js createCruiseOverrideHold with the live clock."""

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.held: dict[str, Any] | None = None
        self.last_seen_ms: float | None = None
        self.last_clock_ms: float | None = None

    def update(self, value: dict[str, Any] | None, clock_ms: float, active: bool) -> dict[str, Any] | None:
        if not active:
            self.reset()
            return None
        if self.last_clock_ms is not None and clock_ms < self.last_clock_ms:
            self.reset()
        self.last_clock_ms = clock_ms
        if value is not None:
            if value["mode"] in (2, 3, 4):
                self.held = value
                self.last_seen_ms = clock_ms
            else:
                self.held = None
                self.last_seen_ms = None
            return value
        age = None if self.last_seen_ms is None else clock_ms - self.last_seen_ms
        if self.held is not None and age is not None and 0 <= age <= CRUISE_OVERRIDE_HOLD_MS:
            return self.held
        self.held = None
        self.last_seen_ms = None
        return None


_GEAR_LABELS = {"park": "P", "reverse": "R", "neutral": "N", "drive": "D", "sport": "S", "low": "L"}


def _gear_label(value: Any) -> str | None:
    raw = str(value or "").strip().lower()
    if not raw:
        return None
    return _GEAR_LABELS.get(raw, raw[:2].upper())


def _level(value: Any) -> float | None:
    number = _finite_or_none(value)
    return number if number is not None and 0 <= number <= 1 else None


def _tpms(value: Any) -> dict[str, float | None] | None:
    if not isinstance(value, Mapping):
        return None

    def pressure(v: Any) -> float | None:
        number = _finite_or_none(v)
        return number if number is not None and 5 <= number <= 100 else None

    payload = {key: pressure(value.get(key)) for key in ("fl", "fr", "rl", "rr")}
    return payload if any(v is not None for v in payload.values()) else None


@dataclass(frozen=True, slots=True)
class WebHudData:
    speed: int | None
    set_speed: int | None
    gap: int | None
    gear: str | None
    speed_limit: int | None
    ev_active: bool
    lane_presentation: str
    active_lane_line: bool | None
    cruise_override: dict[str, Any] | None
    driving_mode: int | None
    lfa_active: Any
    steer_angle: float | None
    accel: float | None
    steer_output: float | None
    left_blinker: bool
    right_blinker: bool
    fuel_gauge: float | None
    urea_gauge: float | None
    tpms: dict[str, float | None] | None
    traffic_state: int
    cpu_temp: float | None


def derive_web_hud_data(snapshot: WebVisionSnapshot, is_metric: bool, override_hold: CruiseOverrideHold,
                        clock_ms: float) -> WebHudData:
    state = snapshot.raw_hud_state()
    device = state.get("deviceState") or {}
    car_state = state.get("carState") or {}
    carrot_man = state.get("carrotMan") or {}
    controls = state.get("controlsState") or {}
    selfdrive = state.get("selfdriveState") or {}
    long_plan = state.get("longitudinalPlan") or {}
    temps = [js_number(v) for v in device.get("cpuTempC") or [] if math.isfinite(js_number(v))]
    v_ego = car_state.get("vEgoCluster") if car_state.get("vEgoCluster") is not None else car_state.get("vEgo")
    v_ego_number = js_number(v_ego)
    v_ego_kph = v_ego_number * 3.6 if isinstance(v_ego, (int, float)) and math.isfinite(v_ego_number) else None
    x_limit = js_number(carrot_man.get("xSpdLimit"))
    x_type = js_number(carrot_man.get("xSpdType"))
    if math.isfinite(x_limit) and x_limit > 0 and x_type != 22:
        limit = x_limit
    else:
        road = js_number(carrot_man.get("nRoadLimitSpeed"))
        limit = road if math.isfinite(road) and road > 0 else None
    personality = js_number(selfdrive.get("personality"))
    gap = personality + 1 if math.isfinite(personality) else None
    gear = _gear_label(car_state.get("gearShifter"))
    # deriveNormalizedHudVehicleMetrics() replaces the drive-only step with the
    # raw carState.gearStep whenever it is positive.
    raw_step = _finite_or_none(car_state.get("gearStep"))
    gear_step = _js_round(raw_step) if raw_step is not None and raw_step > 0 else None
    lane_mode = resolve_lane_mode_state(state)
    lat_active = None  # RAW_HUD_STATE carries no carControl
    lfa_active = lat_active if lat_active is not None else bool(
        selfdrive.get("enabled") if selfdrive.get("enabled") is not None else controls.get("enabled"))
    planner_traffic = _finite_or_none(long_plan.get("trafficState"))
    traffic = planner_traffic if planner_traffic in (1, 2) else None
    if traffic is None:
        carrot_traffic = _finite_or_none(carrot_man.get("trafficState"))
        traffic = carrot_traffic if carrot_traffic in (1, 2) else 0
    driving_mode = _finite_or_none(long_plan.get("myDrivingMode"))
    driving_mode = int(driving_mode) if driving_mode in (1, 2, 3, 4) else None
    cruise_kph = resolve_cruise_kph(state)
    override = derive_cruise_override(state)
    if override is not None and not (override["kph"] > 0):
        override = None
    override = override_hold.update(override, clock_ms, is_cruise_display_visible(state, cruise_kph))

    def to_unit(kph: float | None) -> int | None:
        if kph is None:
            return None
        return _js_round(kph if is_metric else kph * 0.621371)

    steering_angle = _finite_or_none(car_state.get("steeringAngleDeg"))
    a_ego = _finite_or_none(car_state.get("aEgo"))
    steer_output = _finite_or_none(controls.get("lateralOutput"))
    mapped_override = None
    if override is not None:
        mapped_override = {"kph": to_unit(override["kph"]), "label": str(override.get("label") or ""),
                           "mode": override.get("mode", 0)}
    gap_value = None if gap is None or not gap > 0 else _js_round(gap)
    gear_text = str(_js_round(gear_step)) if gear_step is not None and gear_step > 0 else (
        None if gear is None else gear.strip().upper()[:2])
    return WebHudData(
        speed=to_unit(v_ego_kph),
        set_speed=to_unit(cruise_kph) if cruise_kph is not None and cruise_kph > 0 else None,
        gap=gap_value,
        gear=gear_text,
        speed_limit=to_unit(limit) if limit is not None and limit > 0 else None,
        ev_active=bool(car_state.get("evModeValid")) and bool(car_state.get("evModeActive")),
        lane_presentation=lane_mode["presentation"],
        active_lane_line=lane_mode["controlled"],
        cruise_override=mapped_override,
        driving_mode=driving_mode,
        lfa_active=lfa_active,
        steer_angle=steering_angle,
        accel=a_ego,
        steer_output=steer_output,
        left_blinker=bool(car_state.get("leftBlinker")),
        right_blinker=bool(car_state.get("rightBlinker")),
        fuel_gauge=_level(car_state.get("fuelGauge")),
        urea_gauge=_level(car_state.get("ureaGauge")),
        tpms=_tpms(car_state.get("tpms")),
        traffic_state=int(traffic),
        # drivingHudUpdateFromCarPayload() forwards only the averaged cpuTempC.
        cpu_temp=sum(temps) / len(temps) if temps else None,
    )


# --------------------------------------------------------------------------
# HUD layout (hud/style.js for a viewport larger than the 520px container query)
# --------------------------------------------------------------------------

def _css_clamp(minimum: float, value: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))


@dataclass(frozen=True, slots=True)
class WebHudLayout:
    width: float
    height: float
    lfa: tuple[float, float, float, float]
    wifi: tuple[float, float, float, float]
    clock: tuple[float, float, float, float]
    clock_font: float
    limit: tuple[float, float, float, float]
    turn: tuple[float, float, float, float]
    accel: tuple[float, float, float, float]
    steer: tuple[float, float, float, float]
    fuel: tuple[float, float, float, float]
    urea: tuple[float, float, float, float]
    speed: tuple[float, float, float, float]
    tpms: tuple[float, float, float, float]
    devtemp_font: float
    devtemp_pad_block: float
    devtemp_pad_inline: float
    top_gradient_h: float
    bottom_gradient_h: float


def web_hud_layout(width: float, height: float, clock_text_width: float, fuel_visible: bool,
                   urea_visible: bool) -> WebHudLayout:
    cqw = width / 100.0
    # top-left: [lfa, wifi, clock] row then the speed limit sign.
    tl_pad = _css_clamp(10, 3 * cqw, 28)
    tl_gap = _css_clamp(10, 2.4 * cqw, 22)
    row_gap = _css_clamp(8, 1.6 * cqw, 18)
    lfa_size = _css_clamp(28, 5.6 * cqw, 56)
    wifi_size = _css_clamp(24, 4.8 * cqw, 48)
    clock_font = _css_clamp(24, 4.8 * cqw, 48)
    clock_h = clock_font
    row_h = max(lfa_size, wifi_size, clock_h)
    x = tl_pad
    lfa = (x, tl_pad + (row_h - lfa_size) / 2, lfa_size, lfa_size)
    x += lfa_size + row_gap
    wifi = (x, tl_pad + (row_h - wifi_size) / 2, wifi_size, wifi_size)
    x += wifi_size + row_gap
    clock = (x, tl_pad + (row_h - clock_h) / 2, clock_text_width, clock_h)
    limit_w = _css_clamp(46, 8 * cqw, 84)
    limit = (tl_pad + _css_clamp(2, 1.2 * cqw, 10), tl_pad + row_h + tl_gap, limit_w, limit_w)
    # top-center turn signal.
    turn_w = _css_clamp(120, 28 * cqw, 280)
    turn_h = turn_w * 80 / 280
    tc_pad_top = _css_clamp(54, 11 * cqw, 108)
    turn = ((width - turn_w) / 2, tc_pad_top, turn_w, turn_h)
    # top-right: two gauge columns.
    tr_pad = _css_clamp(10, 3 * cqw, 28)
    tr_gap = _css_clamp(6, 1.4 * cqw, 14)
    gauge_w = _css_clamp(40, 6 * cqw, 74)
    gauge_h = gauge_w * 169 / 62
    level_h = gauge_w * 156 / 62
    right_col_x = width - tr_pad - gauge_w
    left_col_x = right_col_x - tr_gap - gauge_w
    accel = (left_col_x, tr_pad, gauge_w, gauge_h)
    steer = (right_col_x, tr_pad, gauge_w, gauge_h)
    fuel = (left_col_x, tr_pad + gauge_h, gauge_w, level_h) if fuel_visible else (0.0, 0.0, 0.0, 0.0)
    urea = (right_col_x, tr_pad + gauge_h, gauge_w, level_h) if urea_visible else (0.0, 0.0, 0.0, 0.0)
    # bottom-left speed panel above the device temperature strip.
    devtemp_font = _css_clamp(12, 2 * cqw, 20)
    devtemp_pad_block = _css_clamp(2, 0.5 * cqw, 5)
    devtemp_pad_inline = _css_clamp(5, 1.2 * cqw, 11)
    devtemp_h = devtemp_font + devtemp_pad_block * 2
    bl_pad = _css_clamp(10, 3 * cqw, 30)
    speed_w = _css_clamp(180, 30 * cqw, 384)
    speed_h = speed_w * 140 / 384
    speed = (bl_pad, height - devtemp_h - bl_pad - speed_h, speed_w, speed_h)
    # bottom-right tpms.
    br_right = _css_clamp(98, 11 * cqw, 122)
    br_pad = _css_clamp(10, 3 * cqw, 30)
    tpms_w = _css_clamp(84, 12 * cqw, 120)
    tpms_h = tpms_w * 68 / 80
    tpms = (width - br_right - br_pad - tpms_w, height - br_pad - tpms_h, tpms_w, tpms_h)
    return WebHudLayout(
        width, height, lfa, wifi, clock, clock_font, limit, turn, accel, steer, fuel, urea, speed, tpms,
        devtemp_font, devtemp_pad_block, devtemp_pad_inline,
        _css_clamp(72, height * 0.16, 148), _css_clamp(104, height * 0.24, 212),
    )


# --------------------------------------------------------------------------
# Onroad alert (home_drive.js renderOnroadAlert + drive.css)
# --------------------------------------------------------------------------

ALERT_SIZE_NONE = 0
ALERT_SIZE_SMALL = 1
ALERT_SIZE_MID = 2
ALERT_SIZE_FULL = 3
ALERT_STATUS_CRITICAL = 2
ONROAD_ALERT_SCALE = 1.5


@dataclass(frozen=True, slots=True)
class WebAlertLayout:
    text1: str
    text2: str
    font1: float
    font2: float
    gap: float
    offset_y: float
    max_width: float
    primary: tuple[int, int, int]
    secondary: tuple[int, int, int]
    show_text2: bool


def _hex(value: str) -> tuple[int, int, int]:
    return int(value[1:3], 16), int(value[3:5], 16), int(value[5:7], 16)


def web_alert_layout(selfdrive_state: Mapping[str, Any] | None, stage_width: float, stage_height: float,
                     base_camera: tuple[float, float, float],
                     display_mode: int = WEB_VISION_DISPLAY_MODE) -> WebAlertLayout | None:
    state = selfdrive_state or {}
    text1 = str(state.get("alertText1") or "").strip()
    text2 = str(state.get("alertText2") or "").strip()
    alert_type = str(state.get("alertType") or "").strip()
    size = finite(state.get("alertSize"), ALERT_SIZE_NONE)
    status = finite(state.get("alertStatus"), 0)
    if size == ALERT_SIZE_NONE or (not text1 and not text2 and not alert_type):
        return None
    base_w, base_h, _ = base_camera
    portrait = stage_height > stage_width
    stage_scale = clamp(min(stage_width / base_w, stage_height / base_h), 0.52, 0.90)
    mode_scale = 0.88 if display_mode == 0 else 0.96 if display_mode == 2 else 0.92
    orientation_scale = 0.80 if portrait else 0.90
    width_scale = clamp(stage_width / 420, 0.72, 0.94) if portrait else 1.0
    resolution_scale = (clamp(stage_height / base_h, 0.98, 1.06) if portrait
                        else clamp(stage_width / base_w, 1.00, 1.08))
    text_scale = stage_scale * mode_scale * orientation_scale * width_scale * resolution_scale * 1.24
    long_primary = len(text1) > 15
    primary_base = 32 if size == ALERT_SIZE_SMALL else 40 if size == ALERT_SIZE_MID else (44 if long_primary else 48)
    secondary_base = 0 if size == ALERT_SIZE_SMALL else (20 if size == ALERT_SIZE_MID else 24)
    s = ONROAD_ALERT_SCALE
    font1 = clamp(_js_round(primary_base * text_scale * s),
                  _js_round((16 if size == ALERT_SIZE_SMALL else 20 if size == ALERT_SIZE_MID else 22) * s),
                  _js_round((32 if size == ALERT_SIZE_SMALL else 40 if size == ALERT_SIZE_MID else 44) * s))
    font2 = 0 if size == ALERT_SIZE_SMALL else clamp(_js_round(secondary_base * text_scale * s), _js_round(14 * s),
                                                     _js_round(29 * s))
    offset_ratio = 0.06 if portrait else 0.11
    mode_offset = -6 if display_mode == 0 else 6 if display_mode == 2 else 0
    offset_y = _js_round(stage_height * offset_ratio) + mode_offset
    gap = clamp(_js_round(font1 * 0.16), _js_round(6 * s), _js_round(14 * s)) if text2 and size != ALERT_SIZE_SMALL else 0
    max_ratio = (0.94 if portrait else 0.84) if size == ALERT_SIZE_FULL else (0.90 if portrait else 0.74)
    critical = status == ALERT_STATUS_CRITICAL
    return WebAlertLayout(
        text1, text2, font1, font2, gap, offset_y, _js_round(stage_width * max_ratio),
        _hex("#ff5a63") if critical else _hex("#ffb12a"),
        _hex("#ffe3e5") if critical else _hex("#ffffff"),
        bool(text2) and size != ALERT_SIZE_SMALL,
    )
