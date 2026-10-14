#!/usr/bin/env python3
"""Offline K9 stop JerkUpperLimit replay (K9StopJerkEase). Analysis only, no vehicle access.

Replays recorded CAN through the real CarInterface.update() and the logged carControl through
CarInterface.apply() in the code tree on PYTHONPATH, then decodes every generated SCC11/12/14 frame.
Run once with the base tree (--modes base) and once with the modified tree (--modes 0,1,2).
This reproduces transmitted values only; it is not a vehicle-response or comfort result.

Params are written to a fresh directory per mode, created with tempfile.mkdtemp inside PARAMS_ROOT.
PARAMS_ROOT, if given, must not exist yet (an existing directory, file or symlink is refused and left
untouched); the tool creates it. Without PARAMS_ROOT a new directory under TMPDIR is used.

usage: PYTHONPATH=<tree> PARAMS_ROOT=<new path> python k9_stop_jerk_ease_replay.py \
         --schema <cereal dir of the recorded commit> --out out.pkl --modes 0,1,2 \
         --start <logMonoTime ns> --end <logMonoTime ns> <segment dir> [...]
"""
import argparse
import os
import pickle
import sys
import tempfile
from collections import deque

import capnp
import zstandard

SCC_ADDR = {1056: "SCC11", 1057: "SCC12", 905: "SCC14"}
SCC_SIGNALS = {
  "SCC12": ("ACCMode", "StopReq", "aReqRaw", "aReqValue"),
  "SCC14": ("ComfortBandUpper", "ComfortBandLower", "JerkUpperLimit", "JerkLowerLimit", "ACCMode"),
}


def read_events(schema, segs):
  capnp.remove_import_hook()
  log = capnp.load(os.path.join(schema, "log.capnp"), imports=[schema, os.path.join(schema, "include")])
  events = []
  for seg in segs:
    raw = open(os.path.join(seg, "rlog.zst"), "rb").read()
    data = zstandard.ZstdDecompressor().decompressobj().decompress(raw)
    try:
      for ev in log.Event.read_multiple_bytes(data):
        events.append(ev)
    except capnp.KjException:
      pass  # truncated tail of an interrupted segment
  return events


def to_current(msg, cls):
  """Copy a recorded struct into the current schema, dropping fields that no longer exist."""
  def filt(d, schema):
    out = {}
    fields = schema.fields
    for k, v in d.items():
      if k not in fields:
        continue
      f = fields[k]
      if f.proto.which() == "slot" and f.proto.slot.type.which() == "struct" and isinstance(v, dict):
        v = filt(v, f.schema)
      elif f.proto.which() == "group" and isinstance(v, dict):
        v = filt(v, f.schema)
      out[k] = v
    return out
  return cls.new_message(**filt(msg.to_dict(), cls.schema))


def make_params_root(path):
  """Create the run's params root. An existing path (directory, file or symlink, even dangling) is refused."""
  if path is None:
    return tempfile.mkdtemp(prefix="k9_stop_jerk_ease_params_")
  try:
    os.mkdir(path)
  except FileExistsError:
    sys.exit(f"refusing PARAMS_ROOT={path!r}: it already exists (or is a symlink); give a new path")
  return path


def write_params(root, mode, entries, overrides):
  """Write params into a new directory for this mode and return it (used as PARAMS_ROOT)."""
  mode_root = tempfile.mkdtemp(prefix=f"mode_{mode}_", dir=root)
  d = os.path.join(mode_root, "d")
  os.mkdir(d)
  for k, v in {**entries, **overrides}.items():
    with open(os.path.join(d, k), "wb") as fh:
      fh.write(v)
  return mode_root


def replay(events, mode, start, end):
  from opendbc.car import structs
  from opendbc.car.car_helpers import interfaces
  from opendbc.can import CANParser

  cp_ev = next(ev for ev in events if ev.which() == "carParams")
  CP = to_current(cp_ev.carParams, structs.CarParams).as_reader()
  CI = interfaces[CP.carFingerprint](CP)
  parsers = {name: CANParser("hyundai_kia_generic", [(name, 0)], 0) for name in SCC_SIGNALS}

  # carOutput after each sendcan holds the accel actually applied in that step; it identifies which
  # recent carControl card used (the latest or the one before), independent of the replayed code.
  applied_accel = {}
  pending_send = []
  for ev in events:
    if ev.which() == "sendcan":
      pending_send.append(ev.logMonoTime)
    elif ev.which() == "carOutput":
      for ts in pending_send:
        applied_accel[ts] = ev.carOutput.actuatorsOutput.accel
      pending_send = []

  rows = []
  recent_cc = deque(maxlen=3)
  pending_can = []
  last_cc = None
  last_cs_log = None
  for ev in events:
    w = ev.which()
    t = ev.logMonoTime
    if w == "can":
      pending_can.append((t, [(m.address, bytes(m.dat), m.src) for m in ev.can]))
    elif w == "carControl":
      last_cc = ev.carControl
      recent_cc.append(ev.carControl)
    elif w == "carState":
      last_cs_log = ev.carState
    elif w == "sendcan" and last_cc is not None:
      if not pending_can:
        continue
      cc = last_cc
      if t in applied_accel:
        cc = next((c for c in reversed(recent_cc) if c.actuators.accel == applied_accel[t]), last_cc)
      CI.CS.speedcam_engaged = cc.enabled
      if last_cs_log is not None:
        CI.CS.speedcam_v_cruise = last_cs_log.cruiseState.speed * 3.6
      CS = CI.update(pending_can)
      pending_can = []
      recorded = {m.address: bytes(m.dat) for m in ev.sendcan if m.address in SCC_ADDR}
      # Follow the recorded 50 Hz SCC phase (frame % 2 == 0 sends SCC); depends only on the log.
      if (CI.CC.frame % 2 == 0) != (1057 in recorded):
        CI.CC.frame += 1
      if last_cs_log is not None:
        # Use the published carState of this step as controller input; CAN copies (SCC11-14,
        # LKAS11, CLU11, MDPS12, soft hold) still come from the replayed CarState.
        CI.CS.out = to_current(last_cs_log, structs.CarState)
        CS = CI.CS.out
      CC = to_current(cc, structs.CarControl).as_reader()
      _, sends = CI.apply(CC, t)
      if t < start or t > end:
        continue
      gen = {}
      for addr, dat, bus in sends:
        if addr in SCC_ADDR:
          gen[SCC_ADDR[addr]] = (dat, bus)
      dec = {}
      for name, (dat, _) in gen.items():
        if name in parsers:
          addr = next(a for a, n in SCC_ADDR.items() if n == name)
          parsers[name].update([t, [(addr, dat, 0)]])
          dec[name] = {s: parsers[name].vl[name][s] for s in SCC_SIGNALS[name]}
      rec = {}
      for addr, dat in recorded.items():
        name = SCC_ADDR[addr]
        if name in parsers:
          parsers[name].update([t, [(addr, dat, 0)]])
          rec[name] = {s: parsers[name].vl[name][s] for s in SCC_SIGNALS[name]}
      a = cc.actuators
      rows.append(dict(
        t=t, mode=mode, sends=[(addr, bytes(dat), bus) for addr, dat, bus in sends], dec=dec, rec=rec,
        vEgo=CS.vEgo, aEgo=CS.aEgo, gas=CS.gasPressed, brake=CS.brakePressed, softHold=CI.CS.softHoldActive,
        enabled=cc.enabled, longActive=cc.longActive, lcs=str(a.longControlState), accel=a.accel,
        override=cc.cruiseControl.override, soft_hold_mode=CI.CC.soft_hold_mode,
      ))
  return dict(cp=dict(fp=CP.carFingerprint, flags=CP.flags, opLong=CP.openpilotLongitudinalControl), rows=rows)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--schema", required=True)
  ap.add_argument("--out", required=True)
  ap.add_argument("--modes", default="0,1,2")
  ap.add_argument("--start", type=int, required=True)
  ap.add_argument("--end", type=int, required=True)
  ap.add_argument("segs", nargs="+")
  args = ap.parse_args()
  root = make_params_root(os.environ.get("PARAMS_ROOT"))

  events = read_events(args.schema, args.segs)
  init = next(ev.initData for ev in events if ev.which() == "initData")
  entries = {}
  for e in init.params.entries:
    if "/" not in e.key:
      entries[e.key] = bytes(e.value)
  if "FingerPrints" not in entries:
    # Not stored in initData: rebuild from the first 2 s of received CAN (address/length per bus).
    fp, t0 = {}, None
    for ev in events:
      if ev.which() != "can":
        continue
      t0 = t0 or ev.logMonoTime
      if ev.logMonoTime - t0 > 2e9:
        break
      for m in ev.can:
        if m.src < 128:
          fp.setdefault(m.src, {})[m.address] = len(m.dat)
    for bus in range(8):
      fp.setdefault(bus, {})
    entries["FingerPrints"] = repr(fp).encode()
  out = {}
  for mode in args.modes.split(","):
    overrides = {"ControlsReady": b"1"}  # card sets this before the first apply()
    if mode != "base":
      overrides["K9StopJerkEase"] = mode.encode()
    os.environ["PARAMS_ROOT"] = write_params(root, mode, entries, overrides)
    out[mode] = replay(events, mode, args.start, args.end)
  out["git"] = init.gitCommit
  with open(args.out, "wb") as fh:
    pickle.dump(out, fh)


if __name__ == "__main__":
  main()
