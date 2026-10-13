"""drive_analysis: test_log_* write real capnp bytes; test_pure_* feed float64 fakes to analyze_events."""
import bz2
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest
import zstandard

from .. import drive_analysis as da

T0 = 1_000_000_000_000
STEP = 100_000_000


def schema():
  return da._replay()._route_replay_module().load_openpilot_log_schema()


# ---- capnp log builders -------------------------------------------------------------------------

def capnp_event(ns, service, fill=None, valid=True):
  event = schema().Event.new_message()
  event.logMonoTime = ns
  event.valid = valid
  message = event.init(service)
  if fill:
    fill(message)
  return event.to_bytes()


def car_fill(v, a, standstill=False, v_cruise=50.0, gas_pressed=False):
  def fill(m):
    m.vEgo = v
    m.aEgo = a
    m.standstill = standstill
    m.vCruise = v_cruise
    m.gasPressed = gas_pressed
  return fill


def control_fill(accel, state="pid"):
  def fill(m):
    m.enabled = True
    m.longActive = True
    m.actuators.accel = accel
    m.actuators.longControlState = state
  return fill


def radar_fill(d, status=True, track=7):
  def fill(m):
    m.leadOne.status = status
    m.leadOne.dRel = d
    m.leadOne.vRel = -1.0
    m.leadOne.radar = True
    m.leadOne.radarTrackId = track
  return fill


def model_fill(eof):
  def fill(m):
    m.timestampEof = eof
  return fill


def q_fill(eof):
  def fill(m):
    m.timestampEof = eof
  return fill


def write_zst(path, chunks):
  path.write_bytes(zstandard.ZstdCompressor().compress(b"".join(chunks)))
  return path


def profile(i):
  """20 s at 100 Hz: 3 s cruise, decel -1.25 to stop at 11 s, stopped until 15 s, then +1.5."""
  t = i / 100
  if t < 3:
    return 10.0, 0.0
  if t < 11:
    return 10.0 - 1.25 * (t - 3), -1.25
  if t < 15:
    return 0.0, 0.0
  return 1.5 * (t - 15), 1.5


def cc_accel(i):
  _v, a = profile(i)
  return a * 0.8 + 0.05 * math.sin(i * 0.37)


def build_profile_log(path, *, car=True, control=True, radar=True, model=True, q=True,
                      car_valid=lambda i: True, car_skip=lambda i: False, radar_status=lambda i: True,
                      radar_valid=lambda i: True, model_valid=lambda i: True, model_skip=lambda i: False,
                      a_override=None, d_override=None, q_offset_ns=0):
  chunks = []
  if q:
    chunks.append(capnp_event(T0 - q_offset_ns, "qRoadEncodeIdx", q_fill(T0 - q_offset_ns)))
  for i in range(2000):
    ns = T0 + i * 10_000_000
    v, a = profile(i)
    if a_override and i in a_override:
      a = a_override[i]
    if car and not car_skip(i):
      chunks.append(capnp_event(ns, "carState", car_fill(v, a, standstill=v == 0.0), valid=car_valid(i)))
    if control:
      chunks.append(capnp_event(ns, "carControl", control_fill(cc_accel(i))))
    if i % 5 == 0:
      if radar:
        d = 40.0 - i / 100 if not (d_override and i in d_override) else d_override[i]
        chunks.append(capnp_event(ns, "radarState", radar_fill(d, status=radar_status(i)), valid=radar_valid(i)))
      if model and not model_skip(i):
        chunks.append(capnp_event(ns, "modelV2", model_fill(ns), valid=model_valid(i)))
  return write_zst(path, chunks)


def f32(x):
  return float(np.float32(x))


# ---- 1. normal --------------------------------------------------------------------------------

def test_log_normal_profile(tmp_path):
  result = da.analyze_log(build_profile_log(tmp_path / "rlog.zst"))
  s = result["series"]
  assert len(s["t"]) == (1999 * 10_000_000) // STEP + 1 == 200
  assert result["t0LogMonoNs"] == T0
  assert s["t"][37] == 3.7
  for k in (0, 37, 108, 152, 199):
    v, a = profile(10 * k)
    assert s["vEgo"][k] == f32(v)
    assert s["aEgo"][k] == f32(a)
    assert s["actAccel"][k] == f32(cc_accel(10 * k))
  assert set(s["vCruiseKph"]) == {50.0}
  for k in (31, 150, 160):
    expected = (f32(profile(10 * k)[1]) - f32(profile(10 * k - 10)[1])) / 0.1
    assert s["jerk"][k] == expected
  assert s["jerk"][0] is None
  kinds = [(e["type"], round(e["start_t"] * 10), round(e["end_t"] * 10)) for e in result["episodes"]]
  # stop: flat run ends at cell 29, first stop cell 108 (v=0.25); start: cell 151 v=0.15 is still stop,
  # cell 152 has float32(0.30) > 0.30 and is the first moving cell; end at first v >= 5.50 (cell 187).
  assert kinds == [("stop_decel", 29, 108), ("start_accel", 151, 187)]
  stop = result["episodes"][0]
  pairs = [(c, x) for c, x in zip(s["actAccel"][29:109], s["aEgo"][29:109], strict=True)]
  expected_corr = np.corrcoef([p[0] for p in pairs], [p[1] for p in pairs])[0, 1]
  assert abs(stop["cmd_actual_corr"] - expected_corr) < 1e-12
  assert stop["start_clipped"] is False and stop["end_clipped"] is False
  assert stop["states"] == {"pid": 80}
  assert stop["lead_track_ids"] == [7]
  assert result["missingServices"] == ["controlsState", "selfdriveState", "longitudinalPlan"]
  assert result["episodeRules"]["comparison"] == "float64(recorded) vs float64(constant)"
  assert "longitudinalPlan.vTargetNow" in result["ambiguousDefaultFields"]


# ---- 2. missing -------------------------------------------------------------------------------

def test_log_missing_services_and_gap(tmp_path):
  result = da.analyze_log(build_profile_log(tmp_path / "rlog.zst", radar=False, control=False,
                                            car_skip=lambda i: 501 <= i <= 539))
  s = result["series"]
  assert "radarState" in result["missingServices"] and "carControl" in result["missingServices"]
  assert set(s["leadDist"]) == {None} and set(s["hasLead"]) == {None}
  assert set(s["actAccel"]) == {None}
  assert all(e["cmd_actual_corr"] is None for e in result["episodes"])
  # 0.2 s after the last carState is still sampled; 0.3 s is stale.
  assert s["vEgo"][52] == f32(profile(500)[0])
  assert s["vEgo"][53] is None and s["aEgo"][53] is None
  assert s["vEgo"][54] == f32(profile(540)[0])
  # recorded 0.0 stays 0.0
  assert s["aEgo"][0] == 0.0 and s["vEgo"][130] == 0.0


def test_log_lead_status_false_is_null(tmp_path):
  result = da.analyze_log(build_profile_log(tmp_path / "rlog.zst", radar_status=lambda i: i >= 1000))
  s = result["series"]
  assert s["hasLead"][50] is False and s["leadDist"][50] is None and s["leadTrackId"][50] is None
  assert s["hasLead"][120] is True and s["leadDist"][120] == f32(40.0 - 12.0)


# ---- 3. invalid ------------------------------------------------------------------------------

def test_log_invalid_messages_are_null_not_filled(tmp_path):
  result = da.analyze_log(build_profile_log(
    tmp_path / "rlog.zst", car_valid=lambda i: not 600 <= i <= 649,
    radar_valid=lambda i: not 1200 <= i <= 1220, model_valid=lambda i: not 1600 <= i <= 1620))
  s = result["series"]
  for k in range(60, 65):
    for column in da.SERVICE_COLUMNS["carState"]:
      assert s[column][k] is None
  assert result["invalidCounts"]["carState"] == 5
  assert s["jerk"][60] is None and s["jerk"][65] is None and s["jerk"][66] is not None
  assert s["vEgo"][65] == f32(profile(650)[0])
  assert "stop_decel" not in [e["type"] for e in result["episodes"]]
  assert {"type": "stop_decel", "start_t": 2.9, "end_t": 10.8, "reason": "carState-null"} in result["discardedEpisodes"]
  assert s["leadDist"][120] is None and s["leadDist"][122] is None and s["leadDist"][119] is not None
  assert s["video_time_s"][160] is None and s["video_time_s"][162] is None and s["video_time_s"][163] is not None


def test_log_latest_model_invalid_previous_valid_is_null(tmp_path):
  # modelV2 at 5.05 s is valid; the latest one at 5.10 s is invalid -> cell 51 null, never 5.05-based.
  result = da.analyze_log(build_profile_log(tmp_path / "rlog.zst", model_valid=lambda i: i != 510))
  video = result["series"]["video_time_s"]
  assert video[51] is None
  assert video[50] == pytest.approx(5.0, abs=1e-9) and video[52] == pytest.approx(5.2, abs=1e-9)
  assert result["invalidCounts"]["modelV2"] == 1


def test_log_latest_message_selected_before_checks_for_every_service(tmp_path):
  # Same rule for carState/radarState: the latest message alone is judged.
  result = da.analyze_log(build_profile_log(tmp_path / "rlog.zst", car_valid=lambda i: i != 700,
                                            radar_valid=lambda i: i != 900))
  s = result["series"]
  assert s["vEgo"][70] is None and s["vEgo"][69] is not None
  assert s["leadDist"][90] is None and s["leadDist"][89] is not None


# ---- 4. NaN / inf ----------------------------------------------------------------------------

def test_log_nan_inf_become_null(tmp_path):
  log = build_profile_log(tmp_path / "rlog.zst", a_override={700: math.nan, 800: math.inf}, d_override={700: math.inf})
  out = tmp_path / "out.json"
  assert da.main([str(log), str(out)]) == 0

  def reject(name):
    raise AssertionError(name)
  data = json.loads(out.read_text(), parse_constant=reject)
  s = data["series"]
  assert s["aEgo"][70] is None and s["aEgo"][80] is None
  assert s["jerk"][70] is None and s["jerk"][71] is None and s["jerk"][81] is None
  assert s["leadDist"][70] is None and s["hasLead"][70] is True
  assert "NaN" not in out.read_text() and "Infinity" not in out.read_text()


# ---- pure fakes ------------------------------------------------------------------------------

class FakeEvent:
  def __init__(self, ns, service, message, valid=True):
    self.logMonoTime, self.valid, self._which = ns, valid, service
    setattr(self, service, message)

  def which(self):
    return self._which


def fake_car(v, a, standstill=False):
  return SimpleNamespace(vEgo=v, aEgo=a, vCruise=50.0, standstill=standstill, gas=0.0, gasPressed=False,
                         brake=0.0, brakePressed=False)


def fake_radar(d, status=True):
  return SimpleNamespace(leadOne=SimpleNamespace(status=status, dRel=d, vRel=0.0, aLeadK=0.0, radar=True,
                                                 radarTrackId=3, modelProb=0.0))


def cells_events(cells, *, valid=None, skip=(), radar=None):
  """cells: (v, a, standstill) per 100 ms cell, one carState exactly at each cell time."""
  events = []
  for k, (v, a, st) in enumerate(cells):
    ns = T0 + k * STEP
    if k not in skip:
      events.append(FakeEvent(ns, "carState", fake_car(v, a, st), valid=valid(k) if valid else True))
    if radar is not None and radar[k] is not None:
      events.append(FakeEvent(ns, "radarState", fake_radar(*radar[k])))
  return events


def run_cells(cells, **kwargs):
  return da.analyze_events(cells_events(cells, **kwargs))


def kinds(result):
  return [(e["type"], round(e["start_t"] * 10), round(e["end_t"] * 10)) for e in result["episodes"]]


def stop_cells(final_v, moving=10):
  return [(1.0, -1.0, False)] * moving + [(final_v, -1.0, False)] + [(0.0, 0.0, True)] * 3


def up(c):
  return math.nextafter(c, math.inf)


def down(c):
  return math.nextafter(c, -math.inf)


# ---- 5. boundaries (float64) ----------------------------------------------------------------

def test_pure_stop_speed_boundary():
  assert kinds(run_cells(stop_cells(0.30)))[0][2] == 10
  assert kinds(run_cells(stop_cells(down(0.30))))[0][2] == 10
  assert kinds(run_cells(stop_cells(up(0.30))))[0][2] == 11  # 0.30+ulp is moving; first stop is the next cell


def flat_then_stop(start_v, flat_a=0.0):
  return [(start_v, flat_a, False)] * 10 + [(1.0, -1.0, False)] * 5 + [(0.0, 0.0, True)] * 3


def test_pure_stop_decel_speed_boundary():
  assert kinds(run_cells(flat_then_stop(5.50)))[0][0] == "stop_decel"
  assert kinds(run_cells(flat_then_stop(up(5.50))))[0][0] == "stop_decel"
  assert kinds(run_cells(flat_then_stop(down(5.50))))[0][0] == "creep_stop"


def test_pure_flat_accel_boundary():
  for value, start, clipped in ((-0.15, 9, False), (up(-0.15), 9, False), (down(-0.15), 0, True)):
    episode = run_cells(flat_then_stop(6.0, value))["episodes"][0]
    assert round(episode["start_t"] * 10) == start and episode["start_clipped"] is clipped


def cruise_cells(decel, count=15):
  return [(10.0, 0.0, False)] * 5 + [(10.0, decel, False)] * count + [(9.0, 0.0, False)] * 5


def test_pure_cruise_decel_boundary_and_run_length():
  assert kinds(run_cells(cruise_cells(-0.50))) == [("cruise_decel", 4, 20)]
  assert kinds(run_cells(cruise_cells(down(-0.50)))) == [("cruise_decel", 4, 20)]
  assert kinds(run_cells(cruise_cells(up(-0.50)))) == []
  assert kinds(run_cells(cruise_cells(-1.0, 14))) == []
  assert kinds(run_cells(cruise_cells(-1.0, 15))) == [("cruise_decel", 4, 20)]


def launch_cells(accel, moving=20):
  return [(0.0, 0.0, True)] * 10 + [(1.0, accel, False)] * moving


def test_pure_start_accel_end_boundary():
  assert kinds(run_cells(launch_cells(down(0.10)))) == [("start_accel", 9, 10)]
  ended = run_cells(launch_cells(0.10))["episodes"][0]
  assert (round(ended["end_t"] * 10), ended["end_clipped"]) == (29, True)
  ended = run_cells(launch_cells(up(0.10)))["episodes"][0]
  assert (round(ended["end_t"] * 10), ended["end_clipped"]) == (29, True)
  speed = [(0.0, 0.0, True)] * 10 + [(1.0, 1.0, False)] * 3 + [(5.50, 1.0, False)] + [(6.0, 1.0, False)] * 3
  assert kinds(run_cells(speed)) == [("start_accel", 9, 13)]
  speed[13] = (down(5.50), 1.0, False)
  assert kinds(run_cells(speed)) == [("start_accel", 9, 14)]


def test_pure_sustain_counts():
  assert kinds(run_cells(stop_cells(0.0, moving=9))) == []
  assert kinds(run_cells(stop_cells(0.0, moving=10))) != []
  nine = [(0.0, 0.0, True)] * 9 + [(1.0, 1.0, False)] * 3
  ten = [(0.0, 0.0, True)] * 10 + [(1.0, 1.0, False)] * 3
  assert kinds(run_cells(nine)) == [] and kinds(run_cells(ten))[0][0] == "start_accel"


def test_pure_stale_limit_inclusive():
  for offset, expected in ((50_000_000, 0.25), (50_000_000 - 1, None)):
    events = [FakeEvent(T0, "carState", fake_car(1.0, 0.0)),
              FakeEvent(T0 + offset, "carState", fake_car(0.25, 0.0)),
              FakeEvent(T0 + 10 * STEP, "carState", fake_car(1.0, 0.0))]
    assert da.analyze_events(events)["series"]["vEgo"][3] == expected


def test_pure_correlation_cell_count():
  xs = [float(i) for i in range(10)]
  ys = [x * 2 + (1 if i % 3 else 0) for i, x in enumerate(xs)]
  assert da._pearson(xs[:9], ys[:9]) is None
  assert abs(da._pearson(xs, ys) - np.corrcoef(xs, ys)[0, 1]) < 1e-12
  assert da._pearson(xs, [1.0] * 10) is None


# ---- 6. boundaries through Float32 capnp ----------------------------------------------------

def float32_variants(c):
  base = np.float32(c)
  return [base, np.nextafter(base, np.float32(-np.inf), dtype=np.float32), np.nextafter(base, np.float32(np.inf), dtype=np.float32)]


def log_cells(path, cells):
  chunks = [capnp_event(T0 + k * STEP, "carState", car_fill(float(v), float(a), standstill=st)) for k, (v, a, st) in enumerate(cells)]
  return da.analyze_log(write_zst(path, chunks))


@pytest.mark.parametrize("constant", (0.30, 5.50, -0.15, -0.50, 0.10))
def test_log_float32_boundaries(tmp_path, constant):
  exact = float(np.float32(constant)) == constant
  assert exact == (constant in (5.50, -0.50))
  read_values = []
  for n, value in enumerate(float32_variants(constant)):
    path = tmp_path / f"{n}.zst"
    if constant == 0.30:
      result = log_cells(path, stop_cells(value))
      read = result["series"]["vEgo"][10]
      assert kinds(result)[0][2] == (10 if read <= 0.30 else 11)
    elif constant == 5.50:
      result = log_cells(path, flat_then_stop(value))
      read = result["series"]["vEgo"][9]
      assert result["episodes"][0]["type"] == ("stop_decel" if read >= 5.50 else "creep_stop")
    elif constant == -0.15:
      result = log_cells(path, flat_then_stop(6.0, value))
      read = result["series"]["aEgo"][0]
      assert round(result["episodes"][0]["start_t"] * 10) == (9 if read >= -0.15 else 0)
    elif constant == -0.50:
      result = log_cells(path, cruise_cells(value))
      read = result["series"]["aEgo"][5]
      assert kinds(result) == ([("cruise_decel", 4, 20)] if read <= -0.50 else [])
    else:
      result = log_cells(path, launch_cells(value))
      read = result["series"]["aEgo"][10]
      assert round(result["episodes"][0]["end_t"] * 10) == (10 if read < 0.10 else 29)
    read_values.append(read)
  assert len(set(read_values)) == 3
  if constant == 0.30:
    assert read_values[0] > 0.30  # recorded float32(0.30) is a moving cell


# ---- 7. video axis --------------------------------------------------------------------------

def test_log_video_start_offset_gap_and_missing(tmp_path):
  result = da.analyze_log(build_profile_log(tmp_path / "a.zst", q_offset_ns=400_000_000,
                                            model_skip=lambda i: 500 < i < 700))
  video = result["series"]["video_time_s"]
  assert result["videoAligned"] is True
  assert abs(video[0] - 0.4) < 1e-9
  assert abs(video[55] - (5.4 + 0.5)) < 1e-9  # 0.5 s after the last anchor is still inside the limit
  assert all(video[k] is None for k in range(56, 70))
  assert abs(video[70] - 7.4) < 1e-9
  clock = result["videoClock"]
  assert clock["nullCells"] == 14 and clock["firstVideoTimeS"] == video[0]
  assert clock["anchors"] == 400 - 39 and clock["medianLogMinusEofS"] == 0.0
  missing = da.analyze_log(build_profile_log(tmp_path / "b.zst", q=False))
  assert missing["videoAligned"] is False and set(missing["series"]["video_time_s"]) == {None}
  assert missing["videoClock"]["firstVideoTimeS"] is None


# ---- 8. corrupt / truncated -----------------------------------------------------------------

def many_events():
  return [capnp_event(T0 + i * 10_000_000, "carState", car_fill(1.0 + i * 0.123, (i * 37 % 101) / 7.0)) for i in range(3000)]


def recoverable_events(data):
  count = 0
  try:
    for _event in schema().Event.read_multiple_bytes(data):
      count += 1
  except Exception:
    pass
  return count


def test_log_corrupt_and_truncated_logs_fail_without_output(tmp_path):
  raw = b"".join(many_events())
  zst = zstandard.ZstdCompressor().compress(raw)[:-7]
  partial = zstandard.ZstdDecompressor().decompressobj().decompress(zst)
  assert recoverable_events(partial) >= 1
  bz = bz2.compress(raw, 1)[:-7]
  assert recoverable_events(bz2.BZ2Decompressor().decompress(bz)) >= 1
  last = capnp_event(T0, "carState", car_fill(1.0, 0.0))
  cases = {
    "empty.zst": b"",
    "random": bytes(range(256)) * 5,
    "nocar.zst": zstandard.ZstdCompressor().compress(capnp_event(T0, "radarState", radar_fill(10.0))),
    "trunc.zst": zst,
    "trunc.bz2": bz,
    "trunc_rlog": raw + last[:len(last) // 2],
  }
  for name, data in cases.items():
    source, out = tmp_path / name, tmp_path / f"{name}.json"
    source.write_bytes(data)
    assert da.main([str(source), str(out)]) == 2, name
    assert not out.exists() and not list(tmp_path.glob(f".{name}.json.*")), name
  two = tmp_path / "two.zst"
  chunks = many_events()
  two.write_bytes(zstandard.ZstdCompressor().compress(b"".join(chunks[:1500])) + zstandard.ZstdCompressor().compress(b"".join(chunks[1500:])))
  out = tmp_path / "two.json"
  assert da.main([str(two), str(out)]) == 0
  assert len(json.loads(out.read_text())["series"]["t"]) == 300


def test_log_record_baseline_read_log_bytes_truncation(tmp_path, capsys):
  route_replay = da._replay()._route_replay_module()
  raw = b"".join(many_events())
  for name, data in (("trunc.zst", zstandard.ZstdCompressor().compress(raw)[:-7]), ("trunc.bz2", bz2.compress(raw, 1)[:-7])):
    path = tmp_path / name
    path.write_bytes(data)
    try:
      out = route_replay.read_log_bytes(path)
      try:
        parsed = f"read_multiple_bytes ok {sum(1 for _ in schema().Event.read_multiple_bytes(out))} events"
      except Exception as exc:  # noqa: BLE001
        parsed = f"read_multiple_bytes {type(exc).__name__}"
      print(f"BASELINE {name}: read_log_bytes no exception, {len(out)}/{len(raw)} bytes; {parsed}")
    except Exception as exc:  # noqa: BLE001
      print(f"BASELINE {name}: read_log_bytes {type(exc).__name__}: {str(exc)[:80]}")


# ---- 9. episode rules -----------------------------------------------------------------------

def test_pure_episode_clipping_null_and_overlap():
  clipped = run_cells([(8.0, -1.0, False)] * 20 + [(0.0, 0.0, True)] * 3)
  assert kinds(clipped) == [("stop_decel", 0, 20)] and clipped["episodes"][0]["start_clipped"] is True

  gap = [(6.0, 0.0, False)] * 10 + [(1.0, -1.0, False)] * 15 + [(0.0, 0.0, True)] * 3
  # cells 10/11 repeat cell 9 (flat, so the start becomes cell 11); cell 12 is 0.3 s old: one null cell
  result = run_cells(gap, skip={10, 11, 12})
  assert result["series"]["vEgo"][11] is not None and result["series"]["vEgo"][12] is None
  assert result["episodes"] == []
  assert result["discardedEpisodes"] == [{"type": "stop_decel", "start_t": 1.1, "end_t": 2.5, "reason": "carState-null"}]

  same = ([(10.0, 0.0, False)] * 20 + [(10.0 - 0.4 * i, -1.0, False) for i in range(20)]
          + [(0.5, 0.0, False)] + [(0.45, -1.0, False)] * 8 + [(0.0, 0.0, True)] * 3)
  result = run_cells(same)
  assert kinds(result) == [("stop_decel", 19, 49)]
  assert {"type": "cruise_decel", "start_t": 1.9, "end_t": 4.0, "reason": "overlap"} in result["discardedEpisodes"]

  different = ([(0.0, 0.0, True)] * 10 + [(1.0, 1.0, False)] * 4 + [(8.5, 1.0, False)]
               + [(9.0, -0.6, False)] * 15 + [(8.0, 0.0, False)] * 3)
  result = run_cells(different)
  assert kinds(result) == [("start_accel", 9, 14)]
  assert [d["type"] for d in result["discardedEpisodes"]] == ["cruise_decel"]


def test_pure_start_accel_cap_and_log_end():
  long_launch = [(0.0, 0.0, True)] * 10 + [(1.0, 0.5, False)] * 200
  episode = run_cells(long_launch)["episodes"][0]
  assert (round(episode["start_t"] * 10), round(episode["end_t"] * 10), episode["end_clipped"]) == (9, 159, False)
  short = [(0.0, 0.0, True)] * 10 + [(1.0, 0.5, False)] * 50
  episode = run_cells(short)["episodes"][0]
  assert (round(episode["end_t"] * 10), episode["end_clipped"]) == (59, True)


# ---- 10. boundary statistics ----------------------------------------------------------------

def test_pure_boundary_statistics_are_cell_values():
  cells = flat_then_stop(6.0)
  radar = [(30.0 - k, True) for k in range(len(cells))]
  radar[9] = (0.0, False)    # start cell: no lead
  radar[15] = (0.0, False)   # end cell (first stop): no lead
  episode = run_cells(cells, radar=radar)["episodes"][0]
  assert round(episode["start_t"] * 10) == 9 and round(episode["end_t"] * 10) == 15
  assert episode["start_lead_d"] is None and episode["end_lead_d"] is None
  assert episode["min_lead_d"] == 30.0 - 14
  assert episode["start_v"] == 6.0 and episode["end_v"] == 0.0 and episode["delta_v"] == -6.0
