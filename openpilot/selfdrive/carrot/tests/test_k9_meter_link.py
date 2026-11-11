"""K9ExtHud=1 taxi meter remote over UDP (selfdrive/carrot/k9_meter_link.py, 보노보노 2026-10-24 21:20 "택시미터기에 연동되는
시작.일시정지.정산 누르면되는 어플하나만들어"): request handling, the K9ExtHud gate, and a full UDP round trip with a fake app
client (python) against the receiver + a simulated HUD (the real C1 meter panel + MeterButtonLink). New tests only."""
import json
import socket
import sys
import threading
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "cluster"))

from openpilot.selfdrive.carrot import k9_meter_link as kml  # noqa: E402


class _P:
  def __init__(self, ext=1):
    self.ext = ext

  def get_int(self, k):
    assert k == "K9ExtHud"
    return self.ext


class _Mem:
  def __init__(self):
    self.kv = {}
    self.lock = threading.Lock()

  def get(self, k):
    with self.lock:
      v = self.kv.get(k)
    return v.encode() if isinstance(v, str) else v

  def put(self, k, v):
    with self.lock:
      self.kv[k] = v


def _req(cmd, rid=1, cid="t"):
  return json.dumps({"k9meter": 1, "cmd": cmd, "id": rid, "cid": cid}).encode()


def test_status_and_commands_queue_with_unique_tokens():
  mem = _Mem()
  mem.put("K9MeterState", json.dumps({"state": "DRIVE", "fare": 5300, "km": 2.31, "sec": 412, "night": 0.0, "ts": 100.0}))
  s = kml.K9MeterServer(_P(), mem, clock=lambda: 101.5)
  r = json.loads(s.handle(_req("status", 7)))
  assert r["ok"] and r["id"] == 7 and r["state"] == "DRIVE" and r["fare"] == 5300 and r["age"] == 1.5 and r["ext_hud"] == 1
  r = json.loads(s.handle(_req("pause", 8)))
  assert r["ok"] and r["applied"] is True
  r = json.loads(s.handle(_req("pause", 8)))          # the app resent the same press (no ack seen): not twice
  assert r["ok"] and r["applied"] is False
  s.handle(_req("pause", 9))
  s.handle(_req("settle", 10))
  q = mem.kv["K9MeterCmd"].split(";")
  assert [e.rsplit(":", 1)[1] for e in q] == ["pause", "pause", "settle"]
  assert len({e.rsplit(":", 1)[0] for e in q}) == 3
  for i in range(20):
    s.handle(_req("start", 100 + i))
  assert len(mem.kv["K9MeterCmd"].split(";")) == kml.QUEUE_MAX


def test_not_ours_or_garbled_gets_no_answer():
  s = kml.K9MeterServer(_P(), _Mem())
  for data in (b"", b"\xff\xfe", b"not json", b"[1,2]", json.dumps({"Carrot2": "x"}).encode(),
               json.dumps({"k9meter": 2, "cmd": "start"}).encode()):
    assert s.handle(data) is None
  r = json.loads(s.handle(_req("explode")))
  assert r["ok"] is False and "K9MeterCmd" not in s._mem.kv


def test_state_missing_or_old_format():
  mem = _Mem()
  assert kml.read_state(mem) == {"age": None}
  mem.put("K9MeterState", "PAUSE")
  assert kml.read_state(mem)["state"] == "PAUSE"
  mem.put("K9MeterState", "{broken")
  assert kml.read_state(mem) == {"age": None}


def _free_port():
  with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    s.bind(("127.0.0.1", 0))
    return s.getsockname()[1]


def test_gate_k9exthud_not_1_never_opens_the_port(monkeypatch):
  monkeypatch.setattr(kml, "GATE_POLL_S", 0.01)
  for ext in (-1, 0):
    u = kml.K9MeterUdp(_P(ext), _Mem(), port=_free_port(), bind_ip="127.0.0.1")
    for _ in range(3):
      u.step(time.monotonic())
    assert u.sock is None
  p = _P(1)
  u = kml.K9MeterUdp(p, _Mem(), port=_free_port(), bind_ip="127.0.0.1")
  u.step(time.monotonic())
  assert u.sock is not None
  p.ext = -1                                          # switched off: closed again
  u.step(time.monotonic())
  assert u.sock is None


def test_carrot_man_starts_the_receiver_thread():
  src = (ROOT / "carrot_man.py").read_text()
  assert "from openpilot.selfdrive.carrot.k9_meter_link import K9MeterUdp" in src
  assert "K9MeterUdp(carrot_man.params, carrot_man.params_memory).run" in src
  assert kml.K9_METER_PORT not in (7705, 7706, 7709, 7710, 7711, 7712, 7713, 7714)


class _Hud(threading.Thread):
  """The HUD side in a thread: the real C1 panel + MeterButtonLink on the same memory params (20 fps)."""

  def __init__(self, mem, speed=60.0):
    super().__init__(daemon=True)
    import cluster_taxi_c1 as c1
    import main as cluster_main
    from types import SimpleNamespace
    self.c1, self.main = c1, cluster_main
    self.clock = [1000.0]
    self.p = c1.C1MeterPanel(clock=lambda: self.clock[0], local_hour=lambda: 12)
    self.r = SimpleNamespace(taxi_meter_command=self.p.command, taxi_meter_state=self.p.meter_status)
    self.src = SimpleNamespace(params_memory=mem)
    self.speed = speed
    self.stop = False

  def run(self):
    st = type("S", (), {})()
    st.onroad, st.steering_angle_deg, st.lfa_active, st.active_lane_line = True, 0.0, True, True
    while not self.stop:
      self.main.feed_taxi_meter_buttons(self.r, self.src)
      st.speed_kph = self.speed
      self.clock[0] += 0.5                           # 10x faster than real time: 0.05 s wall = 0.5 s of driving
      self.p.feed_occupant(False, self.clock[0])
      for _ in range(10):
        self.p.update(st)
      time.sleep(0.05)


class _App:
  """Fake k9 meter app: what the Android app does (status poll, press with retry until ack)."""

  def __init__(self, target):
    self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.s.settimeout(0.3)
    self.target = target
    self.n = 0

  def send(self, cmd, retries=3):
    self.n += 1
    for _ in range(retries):
      self.s.sendto(json.dumps({"k9meter": 1, "cmd": cmd, "id": self.n, "cid": "fake"}).encode(), self.target)
      try:
        data, addr = self.s.recvfrom(2048)
      except socket.timeout:
        continue
      r = json.loads(data)
      if r.get("id") == self.n:
        return r, addr
    raise AssertionError(f"no reply to {cmd}")

  def status_until(self, pred, timeout=4.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
      r, _ = self.send("status")
      if pred(r):
        return r
      time.sleep(0.1)
    raise AssertionError(f"state never matched, last {r}")


def test_udp_round_trip_start_pause_resume_settle_start():
  mem = _Mem()
  port = _free_port()
  u = kml.K9MeterUdp(_P(1), mem, port=port, bind_ip="127.0.0.1")
  t = threading.Thread(target=u.run, daemon=True)
  t.start()
  hud = _Hud(mem)
  hud.start()
  app = None
  try:
    time.sleep(0.6)
    app = _App(("127.0.0.1", port))
    r = app.status_until(lambda r: r.get("state") == "EMPTY")
    assert r["fare"] == 4800 and r["age"] is not None and r["age"] < 3
    r, addr = app.send("start")
    assert r["applied"] and addr == ("127.0.0.1", port)
    r = app.status_until(lambda r: r.get("state") == "DRIVE" and r.get("km", 0) > 0.1)
    app.send("pause")
    paused = app.status_until(lambda r: r.get("state") == "PAUSE")
    time.sleep(1.0)                                   # 10 s of driving while paused
    r = app.status_until(lambda r: r.get("state") == "PAUSE")
    assert (r["fare"], r["km"], r["sec"]) == (paused["fare"], paused["km"], paused["sec"])
    app.send("pause")                                 # resume
    r = app.status_until(lambda r: r.get("state") == "DRIVE" and r["km"] > paused["km"])
    app.send("settle")
    s1 = app.status_until(lambda r: r.get("state") == "SETTLE")
    time.sleep(0.8)
    s2 = app.status_until(lambda r: r.get("state") == "SETTLE")
    assert (s1["fare"], s1["km"]) == (s2["fare"], s2["km"])
    app.send("start")
    r = app.status_until(lambda r: r.get("state") == "DRIVE" and r["km"] < 0.2)
  finally:
    if app is not None:
      app.s.close()
    hud.stop = True
    u.running = False
    t.join(3)
