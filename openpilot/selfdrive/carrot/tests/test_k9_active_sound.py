"""K9ActiveSound sender (selfdrive/carrot/k9_active_sound.py): decoding, gate, protocol, sockets."""
import socket
import struct
import time

import pytest

from openpilot.selfdrive.carrot import k9_active_sound as kas


class FakeParams:
  def __init__(self, v):
    self.v = v

  def get_int(self, key):
    assert key == "K9ActiveSound"
    if isinstance(self.v, Exception):
      raise self.v
    return self.v


def _hello(token=1234, magic=b"K9AH", ver=1):
  return struct.pack("<4sBBI", magic, ver, 0, token)


# ---- DBC decoding (bus 0 K9 frames)
def _pack(msg, values):
  from opendbc.can.packer import CANPacker
  packer = CANPacker("hyundai_kia_generic")
  addr, dat, bus = packer.make_can_msg(msg, 0, values)
  return addr, bytes(dat)


def test_ems366_decode_matches_dbc():
  for rpm, tq in ((680.0, 30), (1234.5, 102), (6500.25, 255), (0.0, 0)):
    addr, dat = _pack("EMS_366", {"N": rpm, "TQI_1": tq * 0.390625})
    assert addr == kas.ADDR_EMS_366
    r, t = kas.decode_ems366(dat)
    assert r == pytest.approx(rpm, abs=0.25) and t == tq


def test_pedal_and_gear_decode_match_dbc():
  addr, dat = _pack("EMS12", {"PV_AV_CAN": 50.0})
  assert addr == kas.ADDR_EMS12 and kas.decode_pedal(dat) * 0.3906 == pytest.approx(50.0, abs=0.4)
  for g in (1, 5, 8):
    addr, dat = _pack("LVR11", {"CF_Lvr_GearInf": g})
    assert addr == kas.ADDR_LVR11 and kas.decode_gear_step(dat) == g


def test_short_frames_ignored():
  assert kas.decode_ems366(b"\x01\x02") is None and kas.decode_pedal(b"\x00" * 6) is None and kas.decode_gear_step(b"") is None
  c = kas.ActiveSoundCore()
  c.on_can([(0, kas.ADDR_EMS_366, b"\x01")], 0.0, 1.0)
  assert not c.dirty


# ---- gate (0 = nothing at all)
@pytest.mark.parametrize("v,mode", [(0, 0), (1, 1), (2, 0), (-1, 0), (None, 0), (ValueError("x"), 0)])
def test_mode(v, mode):
  assert kas.active_sound_mode(FakeParams(v)) == mode


def test_manager_gate_onroad_and_param():
  from openpilot.system.manager import process_config as pc
  assert pc.enable_k9_active_sound(True, FakeParams(1), None) is True
  assert pc.enable_k9_active_sound(False, FakeParams(1), None) is False
  for v in (0, 2, -1, ValueError("x")):
    assert pc.enable_k9_active_sound(True, FakeParams(v), None) is False
  p = pc.managed_processes["k9_active_sound"]
  assert p.module == "openpilot.selfdrive.carrot.k9_active_sound"
  assert p.should_run(True, FakeParams(0), None) is False
  assert p.should_run(True, FakeParams(1), None) is True


def test_main_param0_opens_no_socket(monkeypatch):
  import openpilot.common.params as params_mod

  class P0(FakeParams):
    def __init__(self):
      super().__init__(0)

  monkeypatch.setattr(params_mod, "Params", P0)

  def boom(*a, **k):
    raise AssertionError("socket opened with K9ActiveSound 0")

  monkeypatch.setattr(kas.socket, "socket", boom)
  kas.main()   # returns at once


def test_port_is_new():
  assert kas.K9_ACTIVE_SOUND_PORT == 7730
  assert kas.K9_ACTIVE_SOUND_PORT not in (7705, 7706, 7709, 7710, 7711, 7712, 7713, 7714, 7720)
  assert kas.PKT_SIZE == 30


# ---- hellos / subscribers
@pytest.mark.parametrize("ip,ok", [("10.189.154.20", True), ("192.168.43.5", True), ("172.20.1.1", True), ("100.83.185.59", True),
                                   ("127.0.0.1", True), ("8.8.8.8", False), ("172.32.0.1", False), ("100.128.0.1", False), ("x", False)])
def test_hello_source_filter(ip, ok):
  c = kas.ActiveSoundCore()
  assert c.on_hello(_hello(), (ip, 5000), 1.0) is ok
  assert (len(c.subs) == 1) is ok


def test_hello_bad_magic_version_length():
  c = kas.ActiveSoundCore()
  assert not c.on_hello(_hello(magic=b"K9AX"), ("10.0.0.2", 1), 0.0)
  assert not c.on_hello(_hello(ver=2), ("10.0.0.2", 1), 0.0)
  assert not c.on_hello(b"K9AH", ("10.0.0.2", 1), 0.0)
  assert not c.subs


def test_subscriber_ttl_and_cap():
  c = kas.ActiveSoundCore()
  for i in range(6):
    assert c.on_hello(_hello(), (f"10.0.0.{i}", 1000), float(i) * 0.1)
  assert len(c.subs) == kas.MAX_SUBS and ("10.0.0.0", 1000) not in c.subs and ("10.0.0.5", 1000) in c.subs
  c.expire(0.5 + kas.SUB_TTL_S + 0.01)
  assert not c.subs


# ---- data packets
def _frames(rpm=1500.0, tq=80, ped=64, gear=3):
  e = bytearray(8)
  q = int(round(rpm * 4))
  e[0] = tq; e[1] = q & 0xFF; e[2] = q >> 8
  p = bytearray(8); p[6] = ped
  g = bytearray(8); g[0] = gear
  return [(0, kas.ADDR_EMS_366, bytes(e)), (0, kas.ADDR_EMS12, bytes(p)), (0, kas.ADDR_LVR11, bytes(g))]


def test_no_packets_without_subscriber_or_new_rpm():
  c = kas.ActiveSoundCore()
  c.on_can(_frames(), 0.004, 10.0)
  assert c.packets(10.0) == []           # nobody listening
  c.on_hello(_hello(), ("10.0.0.2", 7000), 10.0)
  assert c.packets(10.0) == []           # the frame above was consumed; nothing new
  c.on_can([(0, kas.ADDR_EMS12, bytes(8))], 0.0, 10.01)
  assert c.packets(10.01) == []          # pedal only: no new rpm -> no packet


def test_packet_fields():
  c = kas.ActiveSoundCore()
  c.on_hello(_hello(token=777), ("10.0.0.2", 7000), 10.000)
  c.on_hello(_hello(token=888), ("100.83.185.59", 7000), 10.001)
  c.on_car_state(20.0, 2, True, False, 10.002)
  c.on_can(_frames(1500.25, 80, 64, 3), 0.0042, 10.010)
  pk = c.packets(10.010)
  assert len(pk) == 2
  got = {a[0]: struct.unpack(kas.PKT_FMT, b) for a, b in pk}
  m = got["10.0.0.2"]
  assert m[0] == b"K9AS" and m[1] == 1 and m[3] == 1
  assert m[2] == kas.FLAG_RPM_VALID | kas.FLAG_CS_FRESH | kas.FLAG_GAS | kas.FLAG_ENGINE_ON
  assert m[5] == 6001 and m[6] == 80 and m[7] == 64 and m[8] == 3 and m[9] == 2
  assert m[10] == 7200                                   # 20 m/s = 72.00 km/h
  assert m[11] == 777 and m[12] == 10                    # echo token, held 10 ms
  assert m[13] == 42                                     # CAN -> send 4.2 ms
  assert got["100.83.185.59"][11] == 888
  c.on_can(_frames(800.0), 0.0, 10.020)
  m2 = struct.unpack(kas.PKT_FMT, c.packets(10.020)[0][1])
  assert m2[3] == 2 and m2[5] == 3200


def test_bus_filter_and_engine_off_flag():
  c = kas.ActiveSoundCore()
  c.on_hello(_hello(), ("10.0.0.2", 7000), 0.0)
  c.on_can([(128, kas.ADDR_EMS_366, b"\x00\xff\xff" + bytes(5)), (2, kas.ADDR_EMS_366, b"\x00\xff\xff" + bytes(5))], 0.0, 0.01)
  assert c.packets(0.01) == []
  c.on_can(_frames(0.0), 0.0, 0.02)
  m = struct.unpack(kas.PKT_FMT, c.packets(0.02)[0][1])
  assert m[2] & kas.FLAG_ENGINE_ON == 0 and m[2] & kas.FLAG_CS_FRESH == 0 and m[8] == 3


def test_udp_roundtrip_loopback():
  srv = kas.open_socket(port=0, bind_ip="127.0.0.1")
  port = srv.getsockname()[1]
  cli = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  cli.bind(("127.0.0.1", 0)); cli.settimeout(1.0)
  try:
    core = kas.ActiveSoundCore()
    cli.sendto(_hello(token=42), ("127.0.0.1", port))
    t_end = time.monotonic() + 1.0
    while not core.subs and time.monotonic() < t_end:
      kas.poll_hellos(srv, core, time.monotonic()); time.sleep(0.005)
    assert core.subs
    core.on_can(_frames(2345.0), 0.001, time.monotonic())
    kas.send_all(srv, core.packets(time.monotonic()))
    data, _ = cli.recvfrom(64)
    m = struct.unpack(kas.PKT_FMT, data)
    assert len(data) == 30 and m[5] == 2345 * 4 and m[11] == 42
  finally:
    srv.close(); cli.close()


def test_not_wired_into_carrot_man():
  import inspect
  from openpilot.selfdrive.carrot import carrot_man
  assert "k9_active_sound" not in inspect.getsource(carrot_man)


def test_hello_thread_stamps_arrival():
  import threading
  srv = kas.open_socket(port=0, bind_ip="127.0.0.1"); srv.settimeout(0.2)
  port = srv.getsockname()[1]
  core = kas.ActiveSoundCore(); lock = threading.Lock()
  ht = kas.HelloThread(srv, core, lock); ht.thread.start()
  cli = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); cli.bind(("127.0.0.1", 0)); cli.settimeout(1.0)
  try:
    t_send = time.monotonic()
    cli.sendto(_hello(token=9), ("127.0.0.1", port))
    t_end = time.monotonic() + 1.0
    while time.monotonic() < t_end:
      with lock:
        if core.subs:
          break
      time.sleep(0.001)
    with lock:
      (_, (t_hello, token, rx_t)), = core.subs.items()
    assert token == 9 and 0.0 <= rx_t - t_send < 0.05
  finally:
    ht.stop.set(); ht.thread.join(1.0); srv.close(); cli.close()
  assert not ht.thread.is_alive()
