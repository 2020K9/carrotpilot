"""K9ExtHud=1 taxi meter remote over UDP 7720, meter_empty2 (보노보노 2026-10-24 22:01 "어플에 빈차도 넣어바"): the full
app sequence 시작 -> 일시정지 -> 재개 -> 정산 -> 빈차 -> 시작 with a fake app client against the receiver + the simulated
HUD (the real C1 panel + MeterButtonLink), and the K9ExtHud gate for the new command. New tests only."""
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "cluster"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from openpilot.selfdrive.carrot import k9_meter_link as kml  # noqa: E402
from test_k9_meter_link import _App, _free_port, _Hud, _Mem, _P  # noqa: E402


def test_udp_round_trip_start_pause_resume_settle_empty_start():
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
    assert r["fare"] == 4800 and r["km"] == 0.0 and r["sec"] == 0
    time.sleep(0.6)                                   # the HUD 'drives' at 60 km/h: 빈차 counts nothing
    r = app.status_until(lambda r: r.get("state") == "EMPTY")
    assert (r["fare"], r["km"], r["sec"]) == (4800, 0.0, 0)
    app.send("start")
    app.status_until(lambda r: r.get("state") == "DRIVE" and r.get("km", 0) > 0.1)
    app.send("pause")
    paused = app.status_until(lambda r: r.get("state") == "PAUSE")
    time.sleep(0.6)
    r = app.status_until(lambda r: r.get("state") == "PAUSE")
    assert (r["fare"], r["km"], r["sec"]) == (paused["fare"], paused["km"], paused["sec"])
    app.send("pause")                                 # 재개
    app.status_until(lambda r: r.get("state") == "DRIVE" and r["km"] > paused["km"])
    app.send("settle")
    s1 = app.status_until(lambda r: r.get("state") == "SETTLE")
    assert s1["km"] > 0.1 and s1["sec"] > 0
    r, _ = app.send("empty")                          # 빈차
    assert r["ok"] and r["applied"]
    e1 = app.status_until(lambda r: r.get("state") == "EMPTY")
    assert (e1["fare"], e1["km"], e1["sec"]) == (4800, 0.0, 0)
    time.sleep(0.6)
    e2 = app.status_until(lambda r: r.get("state") == "EMPTY")
    assert (e2["fare"], e2["km"], e2["sec"]) == (4800, 0.0, 0)
    for cmd in ("pause", "settle"):                   # nothing to pause / settle in 빈차
      app.send(cmd)
    time.sleep(0.4)
    assert app.status_until(lambda r: r.get("state") == "EMPTY")["km"] == 0.0
    app.send("start")
    r = app.status_until(lambda r: r.get("state") == "DRIVE" and 0.0 < r["km"] < 0.3)
    assert hud.p.fsm.state == "DRIVE"
  finally:
    if app is not None:
      app.s.close()
    hud.stop = True
    u.running = False
    t.join(3)


def test_empty_command_needs_k9exthud_1(monkeypatch):
  monkeypatch.setattr(kml, "GATE_POLL_S", 0.01)
  for ext in (-1, 0):
    mem = _Mem()
    u = kml.K9MeterUdp(_P(ext), mem, port=_free_port(), bind_ip="127.0.0.1")
    for _ in range(3):
      u.step(time.monotonic())
    assert u.sock is None and "K9MeterCmd" not in mem.kv
