"""10 Hz longitudinal summary of one rlog for the Carrot Routes compare page.

Usage: python drive_analysis.py LOG OUT.json
Exit code 0 writes OUT.json; any other code leaves no output file. Values that
are missing, stale or recorded with Event.valid=false are null, never filled.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import statistics
import sys
from typing import Any


HERE = Path(__file__).resolve()


def _repo_root() -> Path:
  # Repository layout keeps this file in tools/carrot_route_vault; the image is flat in /app.
  for root in (HERE.parent, *HERE.parents):
    if (root / "openpilot/cereal/log.capnp").is_file():
      return root
  return HERE.parent


REPO_ROOT = _repo_root()
if str(REPO_ROOT) not in sys.path:
  sys.path.insert(0, str(REPO_ROOT))

SCHEMA_VERSION = 1
SAMPLE_HZ = 10
STEP_NS = 100_000_000
FAST_SERVICES = ("carState", "carControl", "controlsState", "selfdriveState")
SLOW_SERVICES = ("longitudinalPlan", "radarState", "modelV2")
# Unapproved defaults for synthetic development: 100 Hz services 0.25 s, 20 Hz services 0.5 s.
STALE_NS = {**{name: 250_000_000 for name in FAST_SERVICES}, **{name: 500_000_000 for name in SLOW_SERVICES}}
LOG_NAMES = ("rlog.zst", "rlog.bz2", "rlog")
CORRELATION_MIN_CELLS = 10

# Unapproved defaults; m/s and m/s^2 float64 constants compared with float64(recorded Float32).
EPISODE_RULES = {
  "comparison": "float64(recorded) vs float64(constant)",
  "cellsPerSecond": SAMPLE_HZ,
  "stopSpeed": 0.30,
  "stopDecelMinStartSpeed": 5.50,
  "flatAccel": -0.15,
  "sustainCells": 10,
  "lookbackCells": 300,
  "startAccelEndSpeed": 5.50,
  "startAccelEndAccel": 0.10,
  "startAccelMaxCells": 150,
  "cruiseDecelAccel": -0.50,
  "cruiseDecelCells": 15,
  "cruiseDecelMinStartSpeed": 8.00,
  "cruiseDecelExpandCells": 300,
  "priority": ["stop_decel", "creep_stop", "start_accel", "cruise_decel"],
}

# Primitive capnp fields read as 0/False both when recorded so and when absent from an older writer.
AMBIGUOUS_DEFAULT_FIELDS = (
  "longitudinalPlan.vTargetNow",
  "controlsState.upAccelCmd",
  "controlsState.uiAccelCmd",
  "controlsState.ufAccelCmd",
  "longitudinalPlan.aTarget",
  "longitudinalPlan.shouldStop",
  "carState.vCruise",
  "carControl.longActive",
  "radarState.leadOne.aLeadK",
  "radarState.leadOne.modelProb",
)

UNITS = {
  "t": "s", "video_time_s": "s", "vEgo": "m/s", "aEgo": "m/s^2", "vCruiseKph": "km/h",
  "gas": "ratio", "brake": "ratio", "actAccel": "m/s^2", "upAccelCmd": "m/s^2", "uiAccelCmd": "m/s^2",
  "ufAccelCmd": "m/s^2", "aTarget": "m/s^2", "vTargetNow": "m/s", "leadDist": "m", "leadRelSpeed": "m/s",
  "leadAccel": "m/s^2", "leadProb": "ratio", "jerk": "m/s^3",
}

COLUMNS = (
  "t", "video_time_s", "vEgo", "aEgo", "vCruiseKph", "standstill", "gas", "gasPressed", "brake", "brakePressed",
  "ccEnabled", "longActive", "actAccel", "actLongControlState", "longControlState", "upAccelCmd", "uiAccelCmd",
  "ufAccelCmd", "aTarget", "vTargetNow", "shouldStop", "sdState", "sdEnabled", "sdActive", "hasLead", "leadDist",
  "leadRelSpeed", "leadAccel", "leadRadar", "leadTrackId", "leadProb", "jerk",
)


def _num(value: Any) -> float | None:
  value = float(value)
  return value if math.isfinite(value) else None


def _extract(service: str, msg: Any) -> dict:
  if service == "carState":
    return {"vEgo": _num(msg.vEgo), "aEgo": _num(msg.aEgo), "vCruiseKph": _num(msg.vCruise),
            "standstill": bool(msg.standstill), "gas": _num(msg.gas), "gasPressed": bool(msg.gasPressed),
            "brake": _num(msg.brake), "brakePressed": bool(msg.brakePressed)}
  if service == "carControl":
    return {"ccEnabled": bool(msg.enabled), "longActive": bool(msg.longActive),
            "actAccel": _num(msg.actuators.accel), "actLongControlState": str(msg.actuators.longControlState)}
  if service == "controlsState":
    return {"longControlState": str(msg.longControlState), "upAccelCmd": _num(msg.upAccelCmd),
            "uiAccelCmd": _num(msg.uiAccelCmd), "ufAccelCmd": _num(msg.ufAccelCmd)}
  if service == "longitudinalPlan":
    return {"aTarget": _num(msg.aTarget), "vTargetNow": _num(msg.vTargetNow), "shouldStop": bool(msg.shouldStop)}
  if service == "selfdriveState":
    return {"sdState": str(msg.state), "sdEnabled": bool(msg.enabled), "sdActive": bool(msg.active)}
  if service == "radarState":
    lead = msg.leadOne
    if not lead.status:
      return {"hasLead": False, "leadDist": None, "leadRelSpeed": None, "leadAccel": None,
              "leadRadar": None, "leadTrackId": None, "leadProb": None}
    return {"hasLead": True, "leadDist": _num(lead.dRel), "leadRelSpeed": _num(lead.vRel),
            "leadAccel": _num(lead.aLeadK), "leadRadar": bool(lead.radar), "leadTrackId": int(lead.radarTrackId),
            "leadProb": _num(lead.modelProb)}
  if service == "modelV2":
    return {"timestampEof": int(msg.timestampEof)}
  raise KeyError(service)


SERVICE_COLUMNS = {
  "carState": ("vEgo", "aEgo", "vCruiseKph", "standstill", "gas", "gasPressed", "brake", "brakePressed"),
  "carControl": ("ccEnabled", "longActive", "actAccel", "actLongControlState"),
  "controlsState": ("longControlState", "upAccelCmd", "uiAccelCmd", "ufAccelCmd"),
  "longitudinalPlan": ("aTarget", "vTargetNow", "shouldStop"),
  "selfdriveState": ("sdState", "sdEnabled", "sdActive"),
  "radarState": ("hasLead", "leadDist", "leadRelSpeed", "leadAccel", "leadRadar", "leadTrackId", "leadProb"),
}


def _replay():
  from openpilot.selfdrive.carrot.radar.tools import radar_validation_replay as replay
  return replay


def _sample(messages: list[tuple[int, bool, dict]], times: list[int], limit_ns: int):
  """Latest message at or before each cell, then judged alone; never fall back to an older one."""
  chosen, invalid, position = [], 0, -1
  for t_ns in times:
    while position + 1 < len(messages) and messages[position + 1][0] <= t_ns:
      position += 1
    if position < 0:
      chosen.append(None)
      continue
    ns, valid, _values = messages[position]
    if not valid:
      invalid += 1
      chosen.append(None)
    elif t_ns - ns > limit_ns:
      chosen.append(None)
    else:
      chosen.append(position)
  return chosen, invalid


def _runs(flags: list[bool], lo: int, hi: int) -> list[tuple[int, int]]:
  """Inclusive (start, end) runs of True within [lo, hi)."""
  runs, start = [], None
  for k in range(lo, hi):
    if flags[k] and start is None:
      start = k
    elif not flags[k] and start is not None:
      runs.append((start, k - 1))
      start = None
  if start is not None:
    runs.append((start, hi - 1))
  return runs


def _ge(value, constant):
  return value is not None and value >= constant


def _lt(value, constant):
  return value is not None and value < constant


def find_episodes(series: dict, rules: dict = EPISODE_RULES) -> tuple[list[dict], list[dict]]:
  v, a, standstill = series["vEgo"], series["aEgo"], series["standstill"]
  n = len(v)
  present = [standstill[k] is not None and v[k] is not None for k in range(n)]
  stop = [present[k] and (standstill[k] is True or v[k] <= rules["stopSpeed"]) for k in range(n)]
  move = [present[k] and v[k] > rules["stopSpeed"] and standstill[k] is not True for k in range(n)]
  sustain, lookback = rules["sustainCells"], rules["lookbackCells"]
  candidates = []

  def run_before(flags, k):
    return k >= sustain and all(flags[k - sustain:k])

  for e in range(n):
    if stop[e] and run_before(move, e):
      lo = max(0, e - lookback)
      flat = [r for r in _runs([_ge(x, rules["flatAccel"]) for x in a], lo, e) if r[1] - r[0] + 1 >= sustain]
      if flat:
        start, clipped = flat[-1][1], False
      else:
        start, clipped = lo, e - lookback < 0
      kind = "stop_decel" if _ge(v[start], rules["stopDecelMinStartSpeed"]) else "creep_stop"
      candidates.append((kind, start, e, clipped, False))
    if move[e] and run_before(stop, e):
      start = e - 1
      limit = start + rules["startAccelMaxCells"]
      end, clipped = None, False
      for j in range(start + 1, min(limit, n - 1) + 1):
        if _ge(v[j], rules["startAccelEndSpeed"]):
          end = j
          break
        if j + sustain <= n and all(_lt(x, rules["startAccelEndAccel"]) for x in a[j:j + sustain]):
          end = j
          break
      if end is None:
        end, clipped = (limit, False) if limit <= n - 1 else (n - 1, True)
      candidates.append(("start_accel", start, end, False, clipped))

  decel = [x is not None and x <= rules["cruiseDecelAccel"] for x in a]
  expand = rules["cruiseDecelExpandCells"]
  for run_start, run_end in _runs(decel, 0, n):
    if run_end - run_start + 1 < rules["cruiseDecelCells"]:
      continue
    start = next((i for i in range(run_start - 1, max(0, run_start - expand) - 1, -1) if _ge(a[i], rules["flatAccel"])), None)
    start_clipped = False
    if start is None:
      start, start_clipped = max(0, run_start - expand), run_start - expand < 0
    end = next((i for i in range(run_end + 1, min(n - 1, run_end + expand) + 1) if _ge(a[i], rules["flatAccel"])), None)
    end_clipped = False
    if end is None:
      end, end_clipped = min(n - 1, run_end + expand), run_end + expand > n - 1
    if _ge(v[start], rules["cruiseDecelMinStartSpeed"]) and not any(stop[start:end + 1]):
      candidates.append(("cruise_decel", start, end, start_clipped, end_clipped))

  t = series["t"]
  discarded, complete = [], []
  for kind, start, end, sc, ec in candidates:
    if not all(present[start:end + 1]):
      discarded.append({"type": kind, "start_t": t[start], "end_t": t[end], "reason": "carState-null"})
    else:
      complete.append((kind, start, end, sc, ec))
  priority = {name: i for i, name in enumerate(rules["priority"])}
  complete.sort(key=lambda item: (item[1], priority[item[0]]))
  kept = []
  for item in complete:
    if any(item[1] <= other[2] and other[1] <= item[2] for other in kept):
      discarded.append({"type": item[0], "start_t": t[item[1]], "end_t": t[item[2]], "reason": "overlap"})
    else:
      kept.append(item)
  return [episode_stats(series, *item) for item in kept], discarded


def _pearson(xs: list[float], ys: list[float]) -> float | None:
  if len(xs) < CORRELATION_MIN_CELLS:
    return None
  mx, my = math.fsum(xs) / len(xs), math.fsum(ys) / len(ys)
  sxx = math.fsum((x - mx) ** 2 for x in xs)
  syy = math.fsum((y - my) ** 2 for y in ys)
  if sxx == 0 or syy == 0:
    return None
  sxy = math.fsum((x - mx) * (y - my) for x, y in zip(xs, ys, strict=True))
  return max(-1.0, min(1.0, sxy / math.sqrt(sxx * syy)))


def episode_stats(series: dict, kind: str, s: int, e: int, start_clipped: bool, end_clipped: bool) -> dict:
  def values(name, lo=s):
    return [x for x in series[name][lo:e + 1] if x is not None]

  def agg(fn, name):
    items = values(name)
    return fn(items) if items else None

  def mean(items):
    return math.fsum(items) / len(items)

  t, v = series["t"], series["vEgo"]
  jerks = values("jerk", s + 1)
  pairs = [(c, x) for c, x in zip(series["actAccel"][s:e + 1], series["aEgo"][s:e + 1], strict=True)
           if c is not None and x is not None]
  states: dict[str, int] = {}
  for state in values("actLongControlState"):
    states[state] = states.get(state, 0) + 1
  long_active = values("longActive")
  start_v, end_v = v[s], v[e]
  return {
    "type": kind,
    "start_t": t[s], "end_t": t[e],
    "start_video_t": series["video_time_s"][s], "end_video_t": series["video_time_s"][e],
    "duration": t[e] - t[s],
    "start_v": start_v, "end_v": end_v,
    "min_v": agg(min, "vEgo"), "max_v": agg(max, "vEgo"),
    "delta_v": end_v - start_v if start_v is not None and end_v is not None else None,
    "min_a": agg(min, "aEgo"), "max_a": agg(max, "aEgo"), "avg_a": agg(mean, "aEgo"),
    "min_cmd_a": agg(min, "actAccel"), "max_cmd_a": agg(max, "actAccel"), "avg_cmd_a": agg(mean, "actAccel"),
    "max_jerk": max(abs(x) for x in jerks) if jerks else None,
    "std_jerk": statistics.pstdev(jerks) if jerks else None,
    "cmd_actual_corr": _pearson([p[0] for p in pairs], [p[1] for p in pairs]),
    "start_lead_d": series["leadDist"][s], "min_lead_d": agg(min, "leadDist"), "end_lead_d": series["leadDist"][e],
    "lead_track_ids": sorted(set(values("leadTrackId"))) or None,
    "states": states or None,
    "long_active_ratio": sum(1 for x in long_active if x) / len(long_active) if long_active else None,
    "pedal_input": any(series["gasPressed"][k] is True or series["brakePressed"][k] is True for k in range(s, e + 1)),
    "start_clipped": start_clipped, "end_clipped": end_clipped,
  }


def analyze_events(events) -> dict:
  """events: time-ordered objects with logMonoTime, valid, which() and service attributes."""
  replay = _replay()
  messages: dict[str, list[tuple[int, bool, dict]]] = {name: [] for name in (*FAST_SERVICES, *SLOW_SERVICES)}
  first_q_eof = 0
  for event in events:
    which = event.which()
    if which == "qRoadEncodeIdx" and first_q_eof == 0:
      first_q_eof = int(event.qRoadEncodeIdx.timestampEof)
    elif which in messages:
      values = _extract(which, getattr(event, which))
      messages[which].append((int(event.logMonoTime), bool(event.valid), values))
  car = messages["carState"]
  if not car:
    raise ValueError("no carState in log")
  t0 = car[0][0]
  count = (car[-1][0] - t0) // STEP_NS + 1
  times = [t0 + k * STEP_NS for k in range(count)]
  if not times:
    raise ValueError("no samples")

  series: dict[str, list] = {"t": [(t_ns - t0) / 1e9 for t_ns in times]}
  invalid_counts, chosen = {}, {}
  for service, columns in SERVICE_COLUMNS.items():
    chosen[service], invalid_counts[service] = _sample(messages[service], times, STALE_NS[service])
    for column in columns:
      series[column] = [None if p is None else messages[service][p][2][column] for p in chosen[service]]

  jerk = [None] * count
  car_pick = chosen["carState"]
  for k in range(1, count):
    p, q = car_pick[k - 1], car_pick[k]
    if p is None or q is None or p == q:
      continue
    a0, a1 = car[p][2]["aEgo"], car[q][2]["aEgo"]
    if a0 is not None and a1 is not None:
      jerk[k] = _num((a1 - a0) / ((car[q][0] - car[p][0]) / 1e9))
  series["jerk"] = jerk

  models = messages["modelV2"]
  model_pick, invalid_counts["modelV2"] = _sample(models, times, STALE_NS["modelV2"])
  video: list[float | None] = [None] * count
  anchors = 0
  if first_q_eof > 0:
    anchors = sum(1 for _ns, valid, values in models
                  if valid and replay.aligned_video_time_s(first_q_eof, values["timestampEof"]) is not None)
    for k, p in enumerate(model_pick):
      if p is None:
        continue
      ns, _valid, values = models[p]
      base = replay.aligned_video_time_s(first_q_eof, values["timestampEof"])
      if base is not None:
        video[k] = base + (times[k] - ns) / 1e9
  series["video_time_s"] = video
  first_video = next((x for x in video if x is not None), None)
  offsets = [(ns - values["timestampEof"]) / 1e9 for ns, valid, values in models if valid]

  episodes, discarded = find_episodes(series)
  return {
    "schemaVersion": SCHEMA_VERSION,
    "kind": "carrotDrive",
    "sampleHz": SAMPLE_HZ,
    "t0LogMonoNs": t0,
    "units": UNITS,
    "missingServices": [name for name in (*FAST_SERVICES, *SLOW_SERVICES) if not messages[name]],
    "invalidCounts": invalid_counts,
    "ambiguousDefaultFields": list(AMBIGUOUS_DEFAULT_FIELDS),
    "videoAligned": first_video is not None,
    "videoClock": {
      "method": "modelV2-anchor", "anchors": anchors, "nullCells": sum(1 for x in video if x is None),
      "firstVideoTimeS": first_video, "medianLogMinusEofS": statistics.median(offsets) if offsets else None,
    },
    "episodeRules": EPISODE_RULES,
    "episodes": episodes,
    "discardedEpisodes": discarded,
    "series": {name: series[name] for name in COLUMNS},
  }


def read_complete_log_bytes(path: Path) -> bytes:
  """Decompress a whole log, rejecting truncated zstd frames that read_log_bytes accepts silently."""
  route_replay = _replay()._route_replay_module()
  data = Path(path).read_bytes()
  if Path(path).suffix == ".bz2" or data.startswith(b"BZh"):
    return route_replay.read_log_bytes(Path(path))  # bz2.decompress already raises on a truncated stream
  if Path(path).suffix == ".zst" or data.startswith(b"\x28\xb5\x2f\xfd"):
    import zstandard
    output, rest = [], data
    while rest:
      reader = zstandard.ZstdDecompressor().decompressobj()
      output.append(reader.decompress(rest))
      if not reader.eof:
        raise ValueError("truncated zstd frame")
      rest = reader.unused_data
    return b"".join(output)
  return data


def analyze_log(path: Path) -> dict:
  replay = _replay()
  schema = replay._route_replay_module().load_openpilot_log_schema()
  data = read_complete_log_bytes(path)
  if not data:
    raise ValueError("empty log")
  # list() forces the whole stream so a partial final message raises here.
  events = replay.monotonic_log_events(list(schema.Event.read_multiple_bytes(data)))
  result = analyze_events(events)
  result["sourceVersion"] = source_version()
  result["sourceLog"] = Path(path).name
  return result


def source_files() -> list[Path]:
  files = [HERE,
           REPO_ROOT / "openpilot/selfdrive/carrot/radar/tools/radar_validation_replay.py",
           REPO_ROOT / "openpilot/selfdrive/carrot/cluster/cluster_route_replay.py",
           REPO_ROOT / "opendbc_repo/opendbc/car/car.capnp"]
  files.extend(sorted((REPO_ROOT / "openpilot/cereal").glob("*.capnp")))
  return files


def source_version() -> str:
  digest = hashlib.sha256()
  for path in source_files():
    digest.update(path.name.encode())
    digest.update(path.read_bytes())
  return digest.hexdigest()[:12]


def main(argv: list[str] | None = None) -> int:
  args = sys.argv[1:] if argv is None else argv
  if len(args) != 2:
    print("usage: drive_analysis.py LOG OUT.json", file=sys.stderr)
    return 2
  source, output = Path(args[0]), Path(args[1])
  temporary = output.with_name(f".{output.name}.{secrets.token_hex(6)}.tmp")
  try:
    result = analyze_log(source)
    with open(temporary, "w", encoding="utf-8") as stream:
      json.dump(result, stream, allow_nan=False, ensure_ascii=False, separators=(",", ":"))
    os.replace(temporary, output)
  except Exception as exc:  # noqa: BLE001 - one-line reason, no partial output
    print(f"drive analysis failed: {type(exc).__name__}: {str(exc).splitlines()[0] if str(exc) else ''}", file=sys.stderr)
    return 2
  finally:
    temporary.unlink(missing_ok=True)
  return 0


if __name__ == "__main__":
  sys.exit(main())
