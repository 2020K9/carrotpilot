"""K9ActiveSound: engine RPM / gear / load from the K9 CAN to the active-sound app on the Carlinkit box (UDP 7730).

보노보노 2026-10-25 00:26: "안드로이드폰에서 엑티브사운드 ... 콤마쪽에서 rpm·기어단수를 카링킷으로 = 선택2 ... 지연값 제일 적게".
The app (com.k9.activesound, built outside the repo) synthesises an engine sound on the box; the box sends its audio to
the car speakers (Android Auto media channel). This process only READS the CAN (cereal 'can' + 'carState') and sends
small UDP packets. It never sends anything to the car.

Param K9ActiveSound (int, default 0, SYSTEM > 사운드 menu):
  0 (or any other value / unreadable): the manager never starts this process -> no socket, no port, nothing changes.
  1: runs while onroad (ignition on).

K9 signals (bus 0, 100 Hz, checked on K9 rlogs 2026-10-22):
  EMS_366 0x366  N      bits 8..23  x0.25 rpm   (EMS11 0x316 does not exist on the K9; carState.engineRpm stays 0)
                 TQI_1  bits 0..7   x0.390625 % (engine torque, used as load)
  EMS12   0x329  PV_AV_CAN bits 48..55 x0.3906 % (accelerator pedal)
  LVR11   0x368  CF_Lvr_GearInf bits 0..3 (gear step 1..8, = carState.gearStep)
  carState: vEgo, gearShifter, gasPressed, brakePressed.

Protocol (UDP, little endian; port K9_ACTIVE_SOUND_PORT 7730, not CarrotNavi 7706 nor the meter 7720):
  app -> comma  HELLO  "<4sBBI"  b"K9AH", ver 1, flags 0, token (app ms clock, echoed back for the RTT)
                The app sends it about every 0.5 s to the LAN broadcast address and to the comma's Tailscale IP; the
                comma sends data to every source address heard within SUB_TTL_S (max MAX_SUBS), so a LAN path and a
                Tailscale path can both be live; the app uses whichever packet of a seq arrives first.
                Only private (10/8, 172.16/12, 192.168/16), CGNAT/Tailscale (100.64/10) and loopback sources.
  comma -> app  DATA   PKT_FMT (30 bytes), one per new EMS_366 frame batch (100 Hz):
                magic b"K9AS", ver, flags, seq, t_ms (comma monotonic ms mod 2^32), rpm_q (rpm*4), tqi raw, pedal raw,
                gear step (-1 unknown), lever (GearShifter enum), v (km/h*100), echo token, echo hold ms,
                can age (0.1 ms: CAN receive -> UDP send)
"""
from __future__ import annotations

import ipaddress
import socket
import struct
import time

K9_ACTIVE_SOUND_PARAM = "K9ActiveSound"
K9_ACTIVE_SOUND_PORT = 7730
HELLO_MAGIC = b"K9AH"
DATA_MAGIC = b"K9AS"
PROTO_VER = 1
HELLO_FMT = "<4sBBI"
PKT_FMT = "<4sBBIIHBBbBHIHH"
PKT_SIZE = struct.calcsize(PKT_FMT)
SUB_TTL_S = 3.0
MAX_SUBS = 4
GATE_POLL_S = 2.0
CS_STALE_S = 0.5

ADDR_EMS_366 = 0x366
ADDR_EMS12 = 0x329
ADDR_LVR11 = 0x368

FLAG_RPM_VALID = 1
FLAG_CS_FRESH = 2
FLAG_GAS = 4
FLAG_BRAKE = 8
FLAG_ENGINE_ON = 16

_ALLOWED_NETS = [ipaddress.ip_network(n) for n in ("10.0.0.0/8", "172.16.0.0/12", "192.168.0.0/16",
                                                   "100.64.0.0/10", "127.0.0.0/8")]


def active_sound_mode(params) -> int:
  """K9ActiveSound as the manager reads it: 1 = on, anything else / unreadable = 0 (off)."""
  try:
    return 1 if int(params.get_int(K9_ACTIVE_SOUND_PARAM)) == 1 else 0
  except Exception:
    return 0


def source_allowed(ip: str) -> bool:
  try:
    a = ipaddress.ip_address(ip)
  except ValueError:
    return False
  return a.version == 4 and any(a in n for n in _ALLOWED_NETS)


def decode_ems366(dat: bytes) -> tuple[float, int] | None:
  """EMS_366 -> (rpm, TQI_1 raw)."""
  if len(dat) < 3:
    return None
  return (dat[1] | (dat[2] << 8)) * 0.25, dat[0]


def decode_pedal(dat: bytes) -> int | None:
  return dat[6] if len(dat) >= 7 else None


def decode_gear_step(dat: bytes) -> int | None:
  return (dat[0] & 0x0F) if len(dat) >= 1 else None


class ActiveSoundCore:
  """Everything except the sockets / cereal (unit-testable). Times are monotonic seconds."""

  def __init__(self) -> None:
    self.subs: dict[tuple[str, int], list] = {}   # addr -> [last_hello_t, token, token_rx_t]
    self.seq = 0
    self.rpm = 0.0
    self.tqi = 0
    self.pedal = 0
    self.gear_step = -1
    self.rpm_t = -1.0
    self.can_t = 0.0          # logMonoTime (s) of the CAN message that carried the newest EMS_366
    self.v_kph = 0.0
    self.lever = 0
    self.gas = False
    self.brake = False
    self.cs_t = -1.0
    self.dirty = False

  # ---- inputs
  def on_hello(self, data: bytes, addr: tuple[str, int], now: float) -> bool:
    if len(data) < struct.calcsize(HELLO_FMT) or not source_allowed(addr[0]):
      return False
    magic, ver, _flags, token = struct.unpack_from(HELLO_FMT, data)
    if magic != HELLO_MAGIC or ver != PROTO_VER:
      return False
    if addr not in self.subs:
      self.expire(now)
      if len(self.subs) >= MAX_SUBS:
        oldest = min(self.subs, key=lambda k: self.subs[k][0])
        del self.subs[oldest]
    self.subs[addr] = [now, token, now]
    return True

  def expire(self, now: float) -> None:
    for k in [k for k, v in self.subs.items() if now - v[0] > SUB_TTL_S]:
      del self.subs[k]

  def on_can(self, frames, can_age_s: float, now: float) -> None:
    """frames: iterable of (src, address, dat) from one 'can' message; can_age_s = now - its logMonoTime."""
    for src, address, dat in frames:
      if src != 0:
        continue
      if address == ADDR_EMS_366:
        d = decode_ems366(dat)
        if d is not None:
          self.rpm, self.tqi = d
          self.rpm_t = now
          self.can_t = now - max(0.0, can_age_s)
          self.dirty = True
      elif address == ADDR_EMS12:
        p = decode_pedal(dat)
        if p is not None:
          self.pedal = p
      elif address == ADDR_LVR11:
        g = decode_gear_step(dat)
        if g is not None:
          self.gear_step = g

  def on_car_state(self, v_ego: float, gear_shifter: int, gas: bool, brake: bool, now: float) -> None:
    self.v_kph = max(0.0, float(v_ego) * 3.6)
    self.lever = int(gear_shifter) & 0xFF
    self.gas = bool(gas)
    self.brake = bool(brake)
    self.cs_t = now

  # ---- output
  def packets(self, now: float) -> list[tuple[tuple[str, int], bytes]]:
    """One packet per subscriber when a new EMS_366 arrived since the last call."""
    if not self.dirty:
      return []
    self.dirty = False
    self.expire(now)
    if not self.subs:
      return []
    self.seq = (self.seq + 1) & 0xFFFFFFFF
    flags = FLAG_RPM_VALID
    if self.cs_t >= 0 and now - self.cs_t < CS_STALE_S:
      flags |= FLAG_CS_FRESH
    if self.gas:
      flags |= FLAG_GAS
    if self.brake:
      flags |= FLAG_BRAKE
    if self.rpm > 300.0:
      flags |= FLAG_ENGINE_ON
    t_ms = int(now * 1000.0) & 0xFFFFFFFF
    rpm_q = max(0, min(65535, int(round(self.rpm * 4.0))))
    gear = self.gear_step if 0 <= self.gear_step <= 15 else -1
    v = max(0, min(65535, int(round(self.v_kph * 100.0))))
    can_age = max(0, min(65535, int(round((now - self.can_t) * 10000.0)))) if self.can_t > 0 else 65535
    out = []
    for addr, (_t, token, rx_t) in self.subs.items():
      hold = max(0, min(65535, int(round((now - rx_t) * 1000.0))))
      out.append((addr, struct.pack(PKT_FMT, DATA_MAGIC, PROTO_VER, flags, self.seq, t_ms, rpm_q, self.tqi & 0xFF,
                                    self.pedal & 0xFF, gear, self.lever, v, token & 0xFFFFFFFF, hold, can_age)))
    return out


def open_socket(port: int = K9_ACTIVE_SOUND_PORT, bind_ip: str = "0.0.0.0") -> socket.socket:
  s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
  try:
    s.setsockopt(socket.IPPROTO_IP, socket.IP_TOS, 0xB8)   # DSCP EF (low-latency queue on Wi-Fi WMM: voice)
  except OSError:
    pass
  s.bind((bind_ip, port))
  s.setblocking(False)
  return s


def poll_hellos(sock: socket.socket, core: ActiveSoundCore, now: float) -> None:
  for _ in range(16):
    try:
      data, addr = sock.recvfrom(64)
    except (BlockingIOError, InterruptedError):
      return
    except OSError:
      return
    core.on_hello(data, addr, now)


class HelloThread:
  """Blocking hello reader, so a hello is time-stamped when it arrives (the echo hold time - and so the app's RTT -
  does not include the up-to-10 ms wait for the next CAN batch of the main loop)."""

  def __init__(self, sock: socket.socket, core: ActiveSoundCore, lock) -> None:
    import threading
    self.sock, self.core, self.lock = sock, core, lock
    self.stop = threading.Event()
    self.thread = threading.Thread(target=self.run, name="k9as_hello", daemon=True)

  def run(self) -> None:
    while not self.stop.is_set():
      try:
        data, addr = self.sock.recvfrom(64)
      except (TimeoutError, socket.timeout, BlockingIOError, InterruptedError):
        continue
      except OSError:
        if self.stop.is_set():
          return
        time.sleep(0.1)
        continue
      now = time.monotonic()
      with self.lock:
        self.core.on_hello(data, addr, now)


def send_all(sock: socket.socket, pkts) -> None:
  for addr, b in pkts:
    try:
      sock.sendto(b, addr)
    except OSError:
      pass


def _boot_s() -> float:
  try:
    return time.clock_gettime(time.CLOCK_BOOTTIME)
  except (AttributeError, OSError):
    return time.monotonic()


def main() -> None:
  from openpilot.common.params import Params
  import openpilot.cereal.messaging as messaging

  params = Params()
  if active_sound_mode(params) != 1:
    return                      # off: no socket at all (the manager does not start us then anyway)
  can_sock = messaging.sub_sock("can", timeout=100)
  cs_sock = messaging.sub_sock("carState", conflate=True)
  import threading
  sock = open_socket()
  sock.settimeout(0.5)            # hello thread blocks; sendto on UDP does not wait in practice
  core = ActiveSoundCore()
  lock = threading.Lock()
  hello = HelloThread(sock, core, lock)
  hello.thread.start()
  next_gate = time.monotonic() + GATE_POLL_S
  print(f"[k9_active_sound] UDP {K9_ACTIVE_SOUND_PORT} open (K9ActiveSound=1)", flush=True)
  try:
    while True:
      msgs = messaging.drain_sock_raw(can_sock, wait_for_one=True)
      now = time.monotonic()
      cs = messaging.recv_sock(cs_sock)
      if cs is not None:
        c = cs.carState
        core.on_car_state(c.vEgo, c.gearShifter.raw, c.gasPressed, c.brakePressed, now)
      for raw in msgs:
        evt = messaging.log_from_bytes(raw)
        age = _boot_s() - evt.logMonoTime * 1e-9          # logMonoTime = nanos_since_boot (CLOCK_BOOTTIME)
        core.on_can(((f.src, f.address, f.dat) for f in evt.can), age, time.monotonic())
      with lock:
        pkts = core.packets(time.monotonic())
      send_all(sock, pkts)
      if now >= next_gate:
        next_gate = now + GATE_POLL_S
        if active_sound_mode(params) != 1:
          break
  finally:
    hello.stop.set()
    sock.close()
    print("[k9_active_sound] UDP closed", flush=True)


if __name__ == "__main__":
  main()
