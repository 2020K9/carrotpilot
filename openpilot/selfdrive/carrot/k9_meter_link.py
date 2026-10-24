"""K9ExtHud=1 taxi meter remote (보노보노 2026-10-24 21:20 "택시미터기에 연동되는 시작.일시정지.정산 누르면되는 어플하나만들어").

A small Android app (k9 meter app, built outside the repo) has three big buttons: 시작 (reset and start), 일시정지/재개,
정산. It talks to the comma over UDP on K9_METER_PORT (7720), next to the carrot_man ports (7705 broadcast, 7706
CarrotNavi, 7709-7714) but separate from them, so CarrotNavi's remote address (7706) is never taken over.

Discovery: the app sends {"k9meter": 1, "cmd": "status"} to the LAN broadcast address (and/or a typed address, e.g. a
Tailscale IP); the comma answers by unicast, so the app learns its address from the reply.
Request:  {"k9meter": 1, "cmd": "start" | "pause" | "settle" | "status", "id": <int>, "cid": "<client token>"}
Reply:    {"k9meter": 1, "ok": true, "id": <same>, "cmd": ..., "applied": true|false (false = a repeat of the same
           (cid, id), not applied twice), "state": "EMPTY|BOARD|DRIVE|PAUSE|PAY|SETTLE", "fare": won, "km": .., "sec": ..,
           "night": rate, "age": s since the HUD last wrote its state (large = HUD not running), "ext_hud": 1}
A command goes to the HUD through the memory param K9MeterCmd (a queue of the last few "token:cmd", tokens unique per
press); the HUD (cluster_taxi_c1.MeterButtonLink) applies new tokens in order and writes K9MeterState (JSON) back,
which every reply carries. The app polls "status" about once a second, so a press shows up within ~1 s.

K9ExtHud != 1: the socket is not even opened (checked every 2 s), nothing is answered - everything as before.
"""
from __future__ import annotations

import json
import os
import socket
import time

K9_METER_PORT = 7720
EXT_HUD_PARAM = "K9ExtHud"
METER_CMD_PARAM = "K9MeterCmd"
METER_STATE_PARAM = "K9MeterState"
COMMANDS = ("start", "pause", "settle")
QUEUE_MAX = 8
SEEN_MAX = 64
GATE_POLL_S = 2.0


def _get_str(params, key: str) -> str:
  try:
    v = params.get(key)
  except Exception:
    return ""
  if isinstance(v, bytes):
    v = v.decode("utf-8", "replace")
  return "" if v is None else str(v)


def read_state(params_memory, now_wall: float | None = None) -> dict:
  """K9MeterState (JSON written by the HUD) -> dict with 'age' (s since written); {} state when missing/garbled."""
  raw = _get_str(params_memory, METER_STATE_PARAM)
  try:
    st = json.loads(raw) if raw.startswith("{") else ({"state": raw} if raw else {})
  except ValueError:
    st = {}
  if not isinstance(st, dict):
    st = {}
  ts = st.pop("ts", None)
  now_wall = time.time() if now_wall is None else now_wall
  st["age"] = round(now_wall - float(ts), 1) if isinstance(ts, (int, float)) else None
  return st


class K9MeterServer:
  """Request handling (no socket here): handle(data, addr) -> reply bytes or None."""

  def __init__(self, params, params_memory, clock=time.time) -> None:
    self._params = params
    self._mem = params_memory
    self._clock = clock
    self._seen: dict[tuple[str, int], bool] = {}
    self._prefix = os.urandom(3).hex()
    self._n = 0
    self._queue: list[str] = []

  def ext_hud(self) -> int:
    try:
      return int(self._params.get_int(EXT_HUD_PARAM))
    except Exception:
      return -1

  def _push(self, cmd: str) -> None:
    self._n += 1
    self._queue = (self._queue + [f"{self._prefix}{self._n:06d}:{cmd}"])[-QUEUE_MAX:]
    put = getattr(self._mem, "put_nonblocking", None) or self._mem.put
    put(METER_CMD_PARAM, ";".join(self._queue))

  def handle(self, data: bytes, addr=None) -> bytes | None:
    try:
      req = json.loads(data.decode("utf-8"))
    except (ValueError, UnicodeDecodeError):
      return None
    if not isinstance(req, dict) or req.get("k9meter") != 1:
      return None                                   # not ours (stray packets): no answer
    cmd = str(req.get("cmd") or "status")
    rid = req.get("id")
    cid = str(req.get("cid") or (addr[0] if addr else ""))
    reply = {"k9meter": 1, "id": rid, "cmd": cmd, "ext_hud": self.ext_hud()}
    if cmd in COMMANDS:
      key = (cid, int(rid) if isinstance(rid, int) else -1)
      applied = key not in self._seen or rid is None
      if applied:
        self._push(cmd)
        if rid is not None:
          self._seen[key] = True
          if len(self._seen) > SEEN_MAX:
            self._seen.pop(next(iter(self._seen)))
      reply.update(ok=True, applied=applied)
    elif cmd == "status":
      reply.update(ok=True)
    else:
      reply.update(ok=False, err="unknown cmd")
    reply.update(read_state(self._mem, self._clock()))
    return json.dumps(reply, ensure_ascii=False).encode("utf-8")


class K9MeterUdp:
  """carrot_man thread: listens on K9_METER_PORT only while K9ExtHud = 1."""

  def __init__(self, params, params_memory, port: int = K9_METER_PORT, bind_ip: str = "0.0.0.0") -> None:
    self.server = K9MeterServer(params, params_memory)
    self.port = port
    self.bind_ip = bind_ip
    self.sock: socket.socket | None = None
    self.running = True

  def _open(self) -> None:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((self.bind_ip, self.port))
    s.settimeout(0.5)
    self.sock = s
    print(f"[k9_meter] UDP {self.port} open (K9ExtHud=1)", flush=True)

  def _close(self) -> None:
    if self.sock is not None:
      try:
        self.sock.close()
      except OSError:
        pass
      self.sock = None
      print("[k9_meter] UDP closed", flush=True)

  def step(self, now: float) -> None:
    """One gate check + up to one request (0.5 s timeout)."""
    want = self.server.ext_hud() == 1
    if want and self.sock is None:
      try:
        self._open()
      except OSError as exc:
        print(f"[k9_meter] bind failed: {exc}", flush=True)
        time.sleep(GATE_POLL_S)
        return
    elif not want and self.sock is not None:
      self._close()
    if self.sock is None:
      time.sleep(GATE_POLL_S)
      return
    deadline = now + GATE_POLL_S
    while time.monotonic() < deadline and self.running:
      try:
        data, addr = self.sock.recvfrom(2048)
      except socket.timeout:
        continue
      except OSError:
        self._close()
        return
      out = self.server.handle(data, addr)
      if out is not None:
        try:
          self.sock.sendto(out, addr)
        except OSError:
          pass

  def run(self) -> None:
    while self.running:
      try:
        self.step(time.monotonic())
      except Exception as exc:   # never take carrot_man down
        print(f"[k9_meter] error: {exc}", flush=True)
        self._close()
        time.sleep(GATE_POLL_S)
    self._close()
