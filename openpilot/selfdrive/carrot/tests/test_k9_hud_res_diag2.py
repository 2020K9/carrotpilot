"""K9HudNavdy 2 진단 창 보완 (노비 11/16 05:39 검토): 콤마 모델 실행 / 젯슨 왕복 (콤마 시계) / 젯슨 내부 계산 (젯슨 시계)은
따로 재고 서로 빼지 않는다. 화면 생성 FPS와 전송 FPS(USB 전송 함수가 성공을 돌려준 프레임만)는 따로 센다.
차량 정보 수신 시각은 새 유효 carState가 왔을 때만 바뀐다(묶음 패킷만 와서는 안 바뀜)."""
import base64
import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[4]
CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
JETLINK = ROOT / "tools" / "jetlink"
sys.path.insert(0, str(CLUSTER))

import cluster_renderer as cr
import cluster_tx_stats
from cluster_live import OpenpilotLiveSource, jetlink_roundtrip_ms, live_comma_diag
from cluster_tx_stats import TxStats, count_h264_pictures

COMMA = {"cores": [23.0, 41.0], "cpu": 32.0, "gpu": 41.0, "mem_pct": 57.0, "temps": [], "power_w": 6.2,
         "free_pct": 41.0, "thermal": "green", "fan_pct": 30.0, "model_ms": 38.2, "roundtrip_ms": 21.7,
         "car_rx_age_s": 0.05}
JET = {"cores": [(45.0, 1510.0, True)], "cpu": 45.0, "gpu": 63.0, "gpu_mhz": 1020.0, "mem_used_gb": 3.1,
       "mem_total_gb": 7.4, "mem_pct": 41.9, "temps": [], "infer_gpu_ms": 11.0, "infer_total_ms": 12.4, "startup": {}}


def fake_width(text, size):
  return sum(size * (0.56 if ord(ch) < 128 else 1.0) for ch in text)


def renderer(jet=JET, startup=None, tx=(None, None)):
  r = cr.ClusterUiRenderer.__new__(cr.ClusterUiRenderer)
  r.k9_jetson_diag = lambda: jet
  if startup is not None:
    r.k9_startup = lambda: startup
  r.k9_tx_stats = lambda: tx
  r.set_k9_res_page(0)
  return r


def rows(r, st):
  """(left label, right value) per drawn row of page 0"""
  texts = r.k9_res_layout(st, fake_width)[0]
  by_base = {}
  for text, x, base, _s, _c in texts:
    by_base.setdefault(base, []).append((x, text))
  return {items[0][1]: (items[-1][1] if len(items) > 1 else None)
          for items in (sorted(v) for _, v in sorted(by_base.items()))}


def st(**kw):
  return SimpleNamespace(**{"comma_diag": COMMA, "actual_fps": 9.8, **kw})


# ---- (d) names ----------------------------------------------------------------------------------------------------

def test_renamed_rows_and_no_old_names():
  got = rows(renderer(startup={"connect": (21.4, "젯슨"), "model": (48.9, "젯슨"), "car": (50.0, "젯슨"),
                               "frame": (63.0, "젯슨")}, tx=(9.6, 0)), st())
  assert got["콤마 모델 실행 · 콤마"] == "38.2 ms"
  assert got["젯슨 왕복 · 콤마"] == "21.7 ms"
  assert got["내부 계산 GPU/전체 · 젯슨"] == "11.0/12.4 ms"
  assert got["화면 생성 FPS"] == "9.8" and got["전송 FPS"] == "9.6"
  assert got["연결 요청 수신"] == "21.4 · 젯슨" and got["첫 USB 전송 반환"] == "63.0 · 젯슨"
  words = " ".join(f"{k} {v}" for k, v in got.items())
  for old in ("요청→결과", "연결 대기", "화면 출력", "실측"):
    assert old not in words, old
  for path in (CLUSTER / "cluster_renderer.py", ROOT / "openpilot/selfdrive/carrot_settings.json"):
    text = path.read_text()
    assert "연결 대기" not in text and "화면 출력" not in text and "요청→결과" not in text, path
  item = next(p for p in json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text())["params"]
              if p["name"] == "K9HudNavdy")
  for needle in ("콤마 모델 실행", "젯슨 왕복", "화면 생성 FPS", "전송 FPS", "연결 요청 수신", "첫 USB 전송 반환",
                 "실제로 뜬 시각은 아님", "빼지 않음"):
    assert needle in item["descr"], needle
  assert "round trip" in item["edescr"] and "send FPS" in item["edescr"] and "往返" in item["cdescr"]


def test_every_new_glyph_is_in_the_diag_font():
  glyphs = set(map(chr, range(32, 127))) | set(cr.K9R_GLYPHS)
  for text in rows(renderer(startup={"frame": (1.0, "젯슨")}, tx=(0.0, 3)), st()):
    assert set(text) <= glyphs, text


def test_new_rows_fit_inside_the_block():
  startup = dict.fromkeys(("connect", "model", "car", "frame"), (12.3, "젯슨"))
  texts, _ = renderer(startup=startup, tx=(10.0, 12345)).k9_res_layout(st(), fake_width)
  assert texts[-2][0] == "첫 USB 전송 반환" or texts[-1][0] == "첫 USB 전송 반환"
  for _t, _x, base, size, _c in texts:
    assert cr.K9R_TOP <= base - size and base + 3 <= cr.K9R_BOTTOM


# ---- (c) round trip ------------------------------------------------------------------------------------------------

def test_roundtrip_dashes_on_comma_model(monkeypatch):
  import openpilot.common.jetlink_status as js
  status = {"active": True, "roundtrip_ms": 21.7}
  monkeypatch.setattr(js, "_fresh", lambda path, *a, **k: dict(status) if path == js.MODEL_STATUS else {})
  assert jetlink_roundtrip_ms() == 21.7
  status["active"] = False          # comma's own model replaced the Jetson
  assert jetlink_roundtrip_ms() is None
  status.clear()                    # stale / missing status
  assert jetlink_roundtrip_ms() is None
  assert jetlink_roundtrip_ms(SimpleNamespace(jetlink_roundtrip_ms=lambda: None)) is None   # Jetson panel, comma model
  assert jetlink_roundtrip_ms(SimpleNamespace(jetlink_roundtrip_ms=lambda: 18.0)) == 18.0
  device = SimpleNamespace(cpuUsagePercent=[10.0], gpuUsagePercent=5.0, memoryUsagePercent=30.0)
  diag = live_comma_diag(device, SimpleNamespace(modelExecutionTime=0.040), 0.1, None)
  assert diag["roundtrip_ms"] is None and diag["model_ms"] == pytest.approx(40.0)
  got = rows(renderer(), st(comma_diag=diag))
  assert got["젯슨 왕복 · 콤마"] == "--" and got["콤마 모델 실행 · 콤마"] == "40.0 ms"


def test_modeld_roundtrip_measured_around_infer_and_cleared_on_local_model():
  src = (ROOT / "openpilot/selfdrive/modeld/jetlink/model.py").read_text()
  infer = src.index("result = self.client.infer(")
  assert src.rindex("warped = time.monotonic()", 0, infer) < infer < src.index("replied = time.monotonic()", infer)
  assert "self.roundtrip_ms = (replied - warped) * 1000." in src
  local = src.index("result = self.small.run(bufs, transforms, inputs, prepare_only)")
  assert src.rindex("self.roundtrip_ms = None", 0, local) > src.index("restoring local model")
  assert "'roundtrip_ms': getattr(self, 'roundtrip_ms', None) if self.active else None" in src


def test_jetson_panel_gets_comma_roundtrip_from_snapshot(monkeypatch):
  monkeypatch.syspath_prepend(str(JETLINK))
  import hud
  monkeypatch.setattr(hud, "read_snapshot", lambda: (1.0, {"jetlink_roundtrip_ms": 19.5}))
  assert hud.DisplayParams.jetlink_roundtrip_ms(None) == 19.5
  monkeypatch.setattr(hud, "read_snapshot", lambda: (1.0, {"jetlink_roundtrip_ms": None}))
  assert hud.DisplayParams.jetlink_roundtrip_ms(None) is None
  monkeypatch.setattr(hud, "read_snapshot", lambda: None)
  assert hud.DisplayParams.jetlink_roundtrip_ms(None) is None
  import hud_protocol
  import openpilot.common.jetlink_status as js
  monkeypatch.setattr(js, "_fresh", lambda path, *a, **k: {"active": False, "roundtrip_ms": 30.0})
  assert hud_protocol.jetlink_roundtrip_ms() is None
  monkeypatch.setattr(js, "_fresh", lambda path, *a, **k: {"active": True, "roundtrip_ms": 30.0})
  assert hud_protocol.jetlink_roundtrip_ms() == 30.0


# ---- (b) 전송 FPS ---------------------------------------------------------------------------------------------------

def test_send_fps_below_render_fps_when_sends_fail_or_are_dropped_and_dash_or_zero_when_stopped():
  tx = TxStats()
  tx.enabled = True
  assert tx.snapshot(0.0) == (None, None)          # nothing tried yet: --
  t = 100.0
  for i in range(40):                              # render loop 10 fps for 4 s
    t += 0.1
    if i % 4 == 0:
      tx.fail(now=t)                               # send call raised / soft-ack failure
    elif i % 4 == 1:
      tx.fail(now=t)                               # H264 queue dropped the picture
    else:
      tx.sent(now=t)
  fps, failed = tx.snapshot(t)
  assert fps == pytest.approx(5.0, abs=0.6) and fps < 10.0 and failed == 20
  delayed = TxStats()
  delayed.enabled = True
  t = 0.0
  for _ in range(40):                              # sending takes 0.3 s while rendering stays 10 fps
    t += 0.3
    delayed.sent(now=t)
  assert delayed.snapshot(t)[0] == pytest.approx(3.3, abs=0.6)
  fps, _ = tx.snapshot(104.0 + 2.5)               # transfer completely stopped
  assert fps == 0.0
  got = rows(renderer(tx=(0.0, 20)), st())
  assert got["화면 생성 FPS"] == "9.8" and got["전송 FPS"] == "0.0 · 실패 20"
  got = rows(renderer(tx=(None, None)), st())
  assert got["전송 FPS"] == "--"


def test_disabled_counter_never_reports():
  tx = TxStats()
  tx.sent(now=1.0)
  assert tx.snapshot(1.0) == (None, None)


def test_usb_send_counts_only_returned_success(monkeypatch):
  import cluster_usb_display as ud
  tx = TxStats()
  tx.enabled = True
  monkeypatch.setattr(ud, "TX", tx)
  d = ud.TuringUsbDisplay.__new__(ud.TuringUsbDisplay)
  d.dev, d._send_jpeg, d._cmd_upload_jpeg = object(), object(), 1
  outcome = []
  def send_frame(cmd, frame):
    if outcome.pop(0):
      return
    raise OSError("usb timeout")
  d._send_frame = send_frame
  d._handle_frame_error = lambda exc: None
  outcome[:] = [True, False, True, False, False]
  for _ in range(5):
    d.send_jpeg(b"x")
  assert len(tx._sent) == 2 and tx.failed == 3


def test_h264_pipeline_counts_pictures_after_every_chunk_returned(monkeypatch):
  import cluster_h264_pipeline as hp
  tx = TxStats()
  tx.enabled = True
  monkeypatch.setattr(hp, "TX", tx)
  p = hp.H264UsbPipeline.__new__(hp.H264UsbPipeline)
  p._closing = False
  results = []
  p._send_h264_chunk = lambda chunk, size, source: results.pop(0)
  results[:] = [True, True]
  p._send_h264_frame([b"a", b"b"], 64, "ffmpeg", 1)
  results[:] = [True, False]                       # one chunk without confirmation: picture not sent
  p._send_h264_frame([b"a", b"b"], 64, "ffmpeg", 1)
  def boom(chunk, size, source):
    raise RuntimeError("TURZX USB H264 chunk upload failed")
  p._send_h264_chunk = boom
  with pytest.raises(RuntimeError):
    p._send_h264_frame([b"a"], 64, "ffmpeg", 2)
  assert len(tx._sent) == 1 and tx.failed == 3
  unit = hp._NativeAccessUnit((b"\x00\x00\x01\x67\x42",), True, False)   # SPS only: not a picture
  assert hp._unit_pictures(unit) == 0 and hp._unit_pictures(hp._NativeAccessUnit((b"x",), False, False)) == 1


def test_h264_picture_count_across_pipe_reads():
  idr, p_slice, sps = b"\x00\x00\x01\x65\x88", b"\x00\x00\x00\x01\x41\x9a", b"\x00\x00\x01\x67\x42\x00"
  second_slice = b"\x00\x00\x01\x41\x04"           # first_mb_in_slice != 0: same picture
  stream = sps + idr + b"\x11" * 50 + second_slice + b"\x22" * 9 + p_slice + b"\x33" * 7 + p_slice
  for cut in range(1, len(stream)):
    a, tail = count_h264_pictures(stream[:cut])
    b, _ = count_h264_pictures(stream[cut:], tail)
    assert a + b == 3, cut
  assert count_h264_pictures(stream)[0] == 3


def test_main_enables_counting_only_for_a_real_usb_output():
  src = (CLUSTER / "main.py").read_text()
  i = src.index("usb_display = TuringUsbDisplay(")
  assert src.index("USB_TX_STATS.enabled = True") > i and not cluster_tx_stats.TX.enabled


# ---- (a) carState stops while bundles keep coming -----------------------------------------------------------------

def _car_event(mono):
  from openpilot.cereal import log
  return base64.b64encode(log.Event.new_message(logMonoTime=mono, valid=True, carState={}).to_bytes()).decode()


def _bundle(sent, car_mono=None, valid=True):
  record = {"sent": sent, "events": {}, "valid": {}, "alive": {}, "received": {}, "mono": {}}
  if car_mono is not None:
    record["events"]["carState"] = _car_event(car_mono)
    record.update(valid={"carState": valid}, alive={"carState": True}, received={"carState": sent},
                  mono={"carState": car_mono})
  return record


def test_jetson_bundles_without_new_carstate_show_lost_after_2s(monkeypatch):
  monkeypatch.syspath_prepend(str(JETLINK))
  import hud
  import hud_health
  marks = {}
  monkeypatch.setattr(hud, "MARKS", marks)
  sm = hud.RemoteSubMaster(["carState", "deviceState"])
  feed = []
  monkeypatch.setattr(hud, "read_snapshot", lambda: feed[-1])
  for i in range(20):                              # bundles arrive, no carState in them at all
    feed.append((10.0 + 0.1 * i, _bundle(5.0 + 0.1 * i)))
    sm.update(0)
  assert "car" not in marks and "car_last" not in marks
  feed.append((12.0, _bundle(7.0, car_mono=1000, valid=False)))   # new but invalid carState
  sm.update(0)
  assert "car" not in marks
  feed.append((12.1, _bundle(7.1, car_mono=2000)))
  sm.update(0)
  assert marks == {"car": 12.1, "car_last": 12.1} and sm.updated["carState"]
  for i in range(1, 30):                           # bundles keep coming, carState frozen (same logMonoTime)
    feed.append((12.1 + 0.1 * i, _bundle(7.1 + 0.1 * i, car_mono=2000)))
    sm.update(0)
    assert not sm.updated["carState"]
  assert marks == {"car": 12.1, "car_last": 12.1}
  r = hud_health.HealthRenderer.__new__(hud_health.HealthRenderer)
  r.startup_marks = marks
  r.host_health = {}
  r.k9_tx_stats = lambda: (None, None)
  r.set_k9_res_page(0)
  monkeypatch.setattr(hud_health.time, "monotonic", lambda: 15.0)   # 2.9 s after the last new carState
  assert r.k9_car_rx_age(None) == pytest.approx(2.9)
  got = rows(r, st())
  assert got["차량 정보 받은 뒤"] == "수신 끊김 3 s"
  monkeypatch.setattr(hud_health.time, "monotonic", lambda: 13.0)
  assert rows(r, st())["차량 정보 받은 뒤"] == "0.90 s"


def test_comma_last_rx_only_moves_on_new_valid_carstate():
  src = OpenpilotLiveSource.__new__(OpenpilotLiveSource)
  sm = SimpleNamespace(updated={"carState": False, "deviceState": True}, valid={"carState": True},
                       recv_time={"carState": 0.0}, alive={"carState": True, "deviceState": True})
  src.sm = sm
  assert src._k9_note_car_state(50.0) is None          # only deviceState etc. updated: no first receive
  sm.updated["carState"], sm.valid["carState"], sm.recv_time["carState"] = True, False, 50.5
  assert src._k9_note_car_state(50.5) is None          # invalid carState is not a receive
  sm.valid["carState"], sm.recv_time["carState"] = True, 51.0
  assert src._k9_note_car_state(51.0) == 51.0
  sm.updated["carState"] = False
  for now in (51.5, 52.0, 53.5):                       # other services keep updating, carState stopped
    sm.recv_time["carState"] = 51.0
    assert src._k9_note_car_state(now) == 51.0
  device = SimpleNamespace(cpuUsagePercent=[10.0], gpuUsagePercent=5.0, memoryUsagePercent=30.0)
  diag = live_comma_diag(device, None, 53.5 - 51.0, None)
  assert rows(renderer(), st(comma_diag=diag))["차량 정보 받은 뒤"] == "수신 끊김 2 s"
  diag = live_comma_diag(device, None, 1.9, None)
  assert rows(renderer(), st(comma_diag=diag))["차량 정보 받은 뒤"] == "1.90 s"

