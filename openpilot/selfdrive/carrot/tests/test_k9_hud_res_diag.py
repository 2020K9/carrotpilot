"""K9HudNavdy 2 (대감님 11/16 01:33 "2번 tpms말고도 전체삭제하고 ... 코어별항목도추가"): the screen-right block is
diagnostics only. Values come from what is already collected; missing / stale = '--' or 수신 끊김, never guessed.
Render checks need a GL display (run under xvfb-run like the other cluster render tests)."""
import io
import json
import sys
import time
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[4]
CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
JETLINK = ROOT / "tools" / "jetlink"
sys.path.insert(0, str(CLUSTER))

import cluster_renderer as cr
from cluster_live import live_comma_diag

COMMA = {"cores": [23.0, 41.0, 18.0, 9.0, 64.0, 37.0, 12.0, 30.0], "cpu": 29.25, "gpu": 41.0, "mem_pct": 57.0,
         "temps": [("CPU", 62.4, [58.1, 60.2, 62.4, 59.0]), ("GPU", 55.3, [55.3]), ("MEM", 50.1, [50.1])],
         "power_w": 6.2, "free_pct": 41.0, "thermal": "green", "fan_pct": 30.0, "model_ms": 38.2, "car_rx_age_s": 0.05}
JET = {"cores": [(45.0, 1510.0, True), (38.0, 1510.0, True), (None, None, False), (20.0, 1510.0, True)],
       "cpu": 34.3, "gpu": 63.0, "gpu_mhz": 1020.0, "mem_used_gb": 3.1, "mem_total_gb": 7.4, "mem_pct": 41.9,
       "temps": [("cpu", 52.1), ("gpu", 50.3), ("soc0", 49.0)], "infer_gpu_ms": 11.0, "infer_total_ms": 12.4, "startup": {}}
STARTUP = {"connect": (21.4, "젯슨"), "model": (48.9, "젯슨"), "car": (None, "젯슨"), "frame": (63.0, "젯슨")}


def fake_width(text, size):
  return sum(size * (0.56 if ord(ch) < 128 else 1.0) for ch in text)


def renderer(jet=None, startup=None, page=0):
  r = cr.ClusterUiRenderer.__new__(cr.ClusterUiRenderer)
  if jet is not None:
    r.k9_jetson_diag = lambda: jet
  if startup is not None:
    r.k9_startup = lambda: startup
  r.set_k9_res_page(page)
  return r


def state(**kw):
  return SimpleNamespace(**{"comma_diag": COMMA, "actual_fps": 9.8, **kw})


def all_text(texts):
  return [t[0] for t in texts]


def assert_clean(texts, bars, width=fake_width):
  x0, x1 = cr.K9L_EDGE, cr.K9L_EDGE + cr.K9L_CAR_W
  boxes = []
  for text, x, base, size, _c in texts:
    w = width(text, size)
    assert x0 - 0.5 <= x and x + w <= x1 + 0.5, (text, x, w)
    assert cr.K9R_TOP <= base - size and base + 3 <= cr.K9R_BOTTOM, (text, base)
    boxes.append((x, x + w, base - size, base + 0.25 * size, text))
  for x, y, w, _f in bars:
    assert x0 - 0.5 <= x and x + w <= x1 + 0.5 and cr.K9R_TOP <= y and y + 3 <= cr.K9R_BOTTOM
  for i, a in enumerate(boxes):
    for b in boxes[i + 1:]:
      assert not (a[0] < b[1] and b[0] < a[1] and a[2] < b[3] and b[2] < a[3]), (a[4], b[4])


# ---- data -------------------------------------------------------------------------------------------------------

def test_comma_diag_keeps_real_core_count_and_none_for_missing():
  ds = SimpleNamespace(cpuUsagePercent=[10] * 12, gpuUsagePercent=41, memoryUsagePercent=57, cpuTempC=[50.0, 61.5],
                       gpuTempC=[55.0], memoryTempC=0.0, powerDrawW=6.2, freeSpacePercent=41.0, thermalStatus=0,
                       fanSpeedPercentDesired=30)
  d = live_comma_diag(ds, SimpleNamespace(modelExecutionTime=0.0382), 0.05)
  assert len(d["cores"]) == 12 and d["cpu"] == 10.0
  assert d["temps"][0] == ("CPU", 61.5, [50.0, 61.5]) and [t[0] for t in d["temps"]] == ["CPU", "GPU"]  # 0.0 = no sensor
  assert abs(d["model_ms"] - 38.2) < 1e-9 and d["car_rx_age_s"] == 0.05 and d["thermal"] == "green"
  empty = live_comma_diag(SimpleNamespace())
  assert empty["cores"] == [] and all(empty[k] is None for k in ("cpu", "gpu", "mem_pct", "power_w", "free_pct",
                                                                   "fan_pct", "model_ms", "car_rx_age_s", "thermal"))
  assert live_comma_diag(None) is None


def test_jetson_diag_from_fresh_health_only(monkeypatch):
  monkeypatch.syspath_prepend(str(JETLINK))
  import hud_health
  health = {"performance": {"version": 1, "cpu": [{"online": True, "busy_pct": 10.0, "clock_mhz": 1510.0},
                                                  {"online": False, "busy_pct": None}],
                            "gpu": {"17000000.gpu": {"load_pct": 35.0, "clock_mhz": 612.0}},
                            "memory": {"MemTotal_kb": 8 * 1048576, "MemAvailable_kb": 6 * 1048576}},
            "sensors": [{"name": "cpu-thermal", "temp_c": 52.1}, {"name": "gpu-thermal", "temp_c": 900}],
            "hud_diag": {"inference": {"gpu_ms": 11.0, "total_ms": 12.4}, "startup": {"hello": 21.4}}}
  d = hud_health.jetson_diag(health)
  assert d["cores"] == [(10.0, 1510.0, True), (None, None, False)] and d["cpu"] == 10.0
  assert (d["gpu"], d["gpu_mhz"], d["mem_used_gb"], d["mem_total_gb"], d["mem_pct"]) == (35.0, 612.0, 2.0, 8.0, 25.0)
  assert d["temps"] == [("cpu", 52.1)] and (d["infer_gpu_ms"], d["infer_total_ms"]) == (11.0, 12.4)
  assert hud_health.jetson_diag({}) is None   # stale STATUS (_fresh -> {}) = 수신 끊김
  stale = hud_health.jetson_diag({"performance": {"status": "stale"}, "sensors": []})
  assert stale["cores"] == [] and stale["cpu"] is None and stale["infer_total_ms"] is None


def test_host_health_diag_is_memory_only_and_expires(monkeypatch):
  monkeypatch.syspath_prepend(str(JETLINK))
  import host_health
  h = host_health.HostHealth.__new__(host_health.HostHealth)
  h.inference, h.marks = None, {}
  h.note_inference(11000, 12400)
  h.mark("hello")
  first = h.marks["hello"]
  h.mark("hello")
  assert h.marks["hello"] == first   # first time only
  now = time.monotonic()
  d = h.diagnostics(now)
  assert d["inference"]["gpu_ms"] == 11.0 and d["inference"]["total_ms"] == 12.4 and d["startup"]["hello"] == first
  assert "inference" not in h.diagnostics(now + 3.5)   # old inference time is dropped, not kept
  src = (JETLINK / "host_health.py").read_text()
  assert "json.dumps({**value, 'hud_diag': self.diagnostics(started)})" in src and "self.snapshot = value" in src


def test_server_and_hud_hooks_are_record_only():
  server = (JETLINK / "server.py").read_text()
  assert "self.telemetry.health.note_inference(gpu_us, total_us)" in server
  assert "health.mark('hello')" in server and "self.telemetry.health.mark('engine_ready')" in server
  hud = (JETLINK / "hud.py").read_text()
  for needle in ("MARKS.setdefault('car', received)", "MARKS['car_last'] = received", "MARKS.setdefault('frame', now)",
                 "HealthRenderer.startup_marks = MARKS"):
    assert needle in hud


def test_settings_describe_diagnostics():
  data = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text())
  item = next(p for p in data["params"] if p["name"] == "K9HudNavdy")
  assert "진단" in item["descr"] and "diagnostic" in item["edescr"].lower() and "诊断" in item["cdescr"]


# ---- layout -----------------------------------------------------------------------------------------------------

def test_missing_values_are_dashes_and_lost():
  texts, bars = renderer().k9_res_layout(state(comma_diag=None, actual_fps=None), fake_width)
  words = all_text(texts)
  assert words.count("수신 끊김") == 2   # 콤마 + 젯슨 headers
  assert "--" in words and "CPU --" in words and "MEM --" in words and "화면 생성 FPS" in words
  digits = [w for w in words if any(ch.isdigit() for ch in w)]
  assert digits == [], digits   # nothing numeric survives without data
  assert_clean(texts, bars)


def test_stale_car_packet_shows_lost():
  r = renderer(JET, STARTUP)
  words = all_text(r.k9_res_layout(state(comma_diag={**COMMA, "car_rx_age_s": 7.2}), fake_width)[0])
  assert "수신 끊김 7 s" in words
  words = all_text(r.k9_res_layout(state(), fake_width)[0])
  assert "0.05 s" in words and "38.2 ms" in words and "11.0/12.4 ms" in words and "9.8" in words
  assert "21.4 · 젯슨" in words and "63.0 · 젯슨" in words and words.count("--") >= 1   # car stage not measured


def test_comma_renderer_never_invents_jetson_or_startup():
  words = all_text(renderer().k9_res_layout(state(), fake_width)[0])
  assert "수신 끊김" in words and "11.0/12.4 ms" not in words
  assert sum(1 for w in words if w == "--") >= 5   # 내부 계산 + 4 startup stages


def test_layout_inside_block_no_overlap_many_cores():
  for n_comma, n_jet in ((8, 6), (16, 12), (32, 32), (64, 64), (1, 1), (0, 0)):
    comma = {**COMMA, "cores": [float(i % 101) for i in range(n_comma)]}
    jet = {**JET, "cores": [(float(i % 101), 1510.0 + i, i % 5 != 3) for i in range(n_jet)],
           "temps": [(f"zone{i}", 40.0 + i) for i in range(20)]}
    for page in (0, 1):
      texts, bars = renderer(jet, STARTUP, page).k9_res_layout(state(comma_diag=comma), fake_width)
      assert_clean(texts, bars)
      if page == 0:
        assert len([b for b in bars if b[3] is not None or b[2] <= 60]) == n_comma + len([c for c in jet["cores"] if c[2]])


def test_page_switch_is_manual_only(monkeypatch):
  r = renderer(JET, STARTUP)
  assert r.k9_res_page == 0
  a = r.k9_res_layout(state(), fake_width)
  monkeypatch.setattr(time, "monotonic", lambda: 1e9)   # time passing never flips the page
  monkeypatch.setattr(time, "time", lambda: 2e9)
  assert r.k9_res_layout(state(), fake_width) == a and r.k9_res_page == 0
  r.set_k9_res_page(1)
  b = r.k9_res_layout(state(), fake_width)
  assert all_text(b[0])[0] == "콤마 코어" and "C7 30%" in all_text(b[0]) and "C2 꺼짐" in all_text(b[0])
  r.set_k9_res_page(5)
  assert r.k9_res_page == 0 and r.k9_res_layout(state(), fake_width) == a
  assert cr.ClusterUiRenderer.k9_res_page == 0


def test_every_drawn_glyph_is_in_the_diag_font():
  glyphs = set(map(chr, range(32, 127))) | set(cr.K9R_GLYPHS)
  for page in (0, 1):
    for st in (state(), state(comma_diag=None)):
      for text in all_text(renderer(JET, STARTUP, page).k9_res_layout(st, fake_width)[0]):
        assert set(text) <= glyphs, text


# ---- render (GL) ------------------------------------------------------------------------------------------------

def _render_frames(fn):
  import render_gear_screens as rg
  from openpilot.common import indicator_display
  from openpilot.selfdrive.carrot import signal_mode
  old = (indicator_display.current, signal_mode.signal_mode, cr.EV6_WEATHER.snapshot, cr.time)
  indicator_display.current, signal_mode.signal_mode, cr.EV6_WEATHER.snapshot = (lambda: None), (lambda: 0), (lambda: None)
  fixed = time.struct_time((2026, 11, 16, 1, 50, 0, 0, 320, -1))   # P clock / animations: one fixed time for every frame
  cr.time = SimpleNamespace(monotonic=lambda: 10000.0, perf_counter=time.perf_counter, localtime=lambda *a: fixed,
                            strftime=time.strftime, time=lambda: 1794761400.0, sleep=time.sleep, thread_time=time.thread_time)
  r = cr.ClusterUiRenderer(language="ko", theme_mode="dark")
  r.open(hidden=True)
  try:
    def png(st):
      r.render_to_png_bytes(st)
      return np.array(Image.open(io.BytesIO(r.render_to_png_bytes(st))).convert("RGB"))
    return fn(r, rg, png)
  finally:
    r.close()
    indicator_display.current, signal_mode.signal_mode, cr.EV6_WEATHER.snapshot, cr.time = old


def test_render_0_1_ignore_diag_and_2_only_changes_right_block():
  def body(r, rg, png):
    out = {}
    for car in ("KIA_K9", "KIA_EV6"):
      for gear in ("D", "P", "R"):
        base = replace(rg.synthetic_state(gear), car_fingerprint=car, actual_fps=9.8)
        diag = replace(base, comma_diag=COMMA)
        for menu in (0, 1, 2):
          r.set_k9_hud_style(min(menu, 1))
          r.set_k9_hud_navdy(1)
          r.set_k9_hud_res(menu == 2)
          r.set_k9_res_page(0)
          out[(car, gear, menu, "plain")] = png(base)
          out[(car, gear, menu, "diag")] = png(diag)
    r.set_k9_res_page(1)   # page 1 in its own pass: extra frames would move the EV6 animation state between menus
    for car in ("KIA_K9", "KIA_EV6"):
      for gear in ("D", "P", "R"):
        r.set_k9_hud_style(1)
        r.set_k9_hud_res(True)
        out[(car, gear, 2, "p1")] = png(replace(rg.synthetic_state(gear), car_fingerprint=car, actual_fps=9.8, comma_diag=COMMA))
    r.set_k9_res_page(0)
    return out
  f = _render_frames(body)
  for car in ("KIA_K9", "KIA_EV6"):
    for gear in ("D", "P", "R"):
      for menu in (0, 1):
        assert np.array_equal(f[(car, gear, menu, "plain")], f[(car, gear, menu, "diag")]), (car, gear, menu)
      if car == "KIA_EV6" or gear != "D":   # EV6 and K9 P / R have no K9 D scene: 2 == 1
        for kind in ("plain", "diag"):
          assert np.array_equal(f[(car, gear, 1, "plain")], f[(car, gear, 2, kind)]), (car, gear, kind)
  one, two, p1 = f[("KIA_K9", "D", 1, "diag")], f[("KIA_K9", "D", 2, "diag")], f[("KIA_K9", "D", 2, "p1")]
  scale = one.shape[1] / cr.DESIGN_WIDTH
  x_block = (cr.DESIGN_WIDTH - cr.K9L_EDGE - cr.K9L_CAR_W - 2) * scale
  for img in (two, p1):
    ys, xs = np.nonzero(np.any(one != img, axis=2))
    assert len(xs) > 1000 and xs.min() >= x_block
  assert np.any(two != p1)


def test_render_real_font_no_overlap_many_cores():
  def body(r, rg, png):
    jet = {**JET, "cores": [(float(i), 1510.0, True) for i in range(32)], "temps": [(f"zone{i}", 40.0 + i) for i in range(20)]}
    comma = {**COMMA, "cores": [float(i) for i in range(32)]}
    r.k9_jetson_diag = lambda: jet
    r.k9_startup = lambda: STARTUP
    st = replace(rg.synthetic_state("D"), car_fingerprint="KIA_K9", comma_diag=comma, actual_fps=9.8)
    r.set_k9_hud_style(1)
    r.set_k9_hud_navdy(1)
    r.set_k9_hud_res(True)
    png(st)   # fonts loaded inside the frame
    for page in (0, 1):
      r.set_k9_res_page(page)
      texts, bars = r.k9_res_layout(st)
      assert_clean(texts, bars, r._k9r_width)
    return True
  assert _render_frames(body)
