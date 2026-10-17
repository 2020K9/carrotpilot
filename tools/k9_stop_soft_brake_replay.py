#!/usr/bin/env python3
"""Offline K9 stop soft brake replay (K9StopSoftBrakeTime/Accel). Analysis only, no vehicle access.

Replays one recorded stop through the code tree on PYTHONPATH:
- card step at every recorded sendcan: CarInterface.update() on the recorded CAN, the cruise helper
  (VCruiseCarrot.update_v_cruise, which owns softHoldActive and this trial's state) on the recorded carState
  and latest services, then CarInterface.apply() with the recorded carControl.
- controlsd step at every recorded carControl: LongControl.update() on the recorded carState (softHoldActive
  replaced by the replayed helper value), longitudinalPlan and radarState.
Every generated SCC frame, StopReq, softHoldActive and LongControl output inside the window is stored.
Run the base tree with --time base (which always writes StopEaseEnable=0, the removed stop easing off) and the
modified tree with --time 0 (off) --stop-ease-leftover (stored StopEase values that must now be ignored,
StopEaseAccel=-20 included), then compare with k9_stop_soft_brake_compare.py. The replayed outputs are not fed back into the recorded inputs, so a non-zero
time is not a closed-loop or vehicle-response result.

Params come from initData, written to a fresh directory created inside PARAMS_ROOT (which must not exist yet;
an existing directory, file or symlink is refused). Without PARAMS_ROOT a new directory under TMPDIR is used.

usage: PYTHONPATH=<tree> PARAMS_ROOT=<new path> python k9_stop_soft_brake_replay.py --schema <cereal dir> \
         --out out.pkl --time base|0..150 --start <logMonoTime ns> --end <logMonoTime ns> <segment dir> [...]
"""
import argparse
import os
import pickle
import sys
import tempfile
import time
from collections import deque

import capnp
import zstandard

SCC_ADDR = {1056: "SCC11", 1057: "SCC12", 905: "SCC14"}
SCC_SIGNALS = {
  "SCC12": ("ACCMode", "StopReq", "aReqRaw", "aReqValue"),
  "SCC14": ("ComfortBandUpper", "ComfortBandLower", "JerkUpperLimit", "JerkLowerLimit", "ACCMode"),
}
SERVICES = ("radarState", "longitudinalPlan", "carrotMan", "drivingModelData")


# 부모판 43d443e4 의 tools/k9_stop_jerk_ease_replay.py 34~86행에서 그대로 옮긴 함수 넷(그 도구는 지워졌다).
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


class NullReader:
  """No bluetooth remote input in replay."""
  def __init__(self, *args, **kwargs):
    self.is_repeat = False

  def read(self, **kwargs):
    return None


class ReplaySubMaster(dict):
  def __init__(self, latest, t, cc):
    super().__init__(carControl=cc)
    self.alive, self.valid, self.seen, self.recv_time = {}, {}, {}, {}
    for name in SERVICES:
      ev = latest.get(name)
      self[name] = getattr(ev[1], name) if ev is not None else None
      age = (t - ev[0]) / 1e9 if ev is not None else None
      self.seen[name] = ev is not None
      self.alive[name] = ev is not None and age <= 1.0  # bounded age instead of the live frequency check
      self.valid[name] = ev is not None and ev[1].valid
      # carrotMan freshness uses time.monotonic(); keep the recorded message age.
      self.recv_time[name] = time.monotonic() - (age if age is not None else 1e9)
    if self["radarState"] is None:
      from openpilot.cereal import log
      self["radarState"] = log.RadarState.new_message().as_reader()


def replay(events, start, end, is_metric):
  from openpilot.cereal import car
  from openpilot.common.constants import CV
  from openpilot.common.params import Params
  import openpilot.selfdrive.car.cruise as cruise
  from openpilot.selfdrive.controls.lib.longcontrol import LongControl
  from opendbc.car import structs
  from opendbc.car.car_helpers import interfaces
  from opendbc.can import CANParser

  cruise.CommandReader = NullReader
  cruise.Params = lambda *args: Params()  # no /dev/shm params; the memory store is unused by the helper

  cp_ev = next(ev for ev in events if ev.which() == "carParams")
  CP = to_current(cp_ev.carParams, structs.CarParams).as_reader()
  CI = interfaces[CP.carFingerprint](CP)
  helper = cruise.VCruiseCarrot(CP)
  LoC = LongControl(CP)
  parsers = {name: CANParser("hyundai_kia_generic", [(name, 0)], 0) for name in SCC_SIGNALS}
  has_trial = getattr(helper, "k9_stop_soft_brake", None) is not None

  # carOutput 으로 그 걸음에서 실제 적용한 carControl 을 고른다.
  applied_accel = {}
  pending_send = []
  for ev in events:
    if ev.which() == "sendcan":
      pending_send.append(ev.logMonoTime)
    elif ev.which() == "carOutput":
      for ts in pending_send:
        applied_accel[ts] = ev.carOutput.actuatorsOutput.accel
      pending_send = []

  card_rows, lc_rows = [], []
  recent_cc = deque(maxlen=3)
  latest = {}
  pending_can = []
  last_cc = last_cs_log = None
  last_plan_t = None
  soft_hold = 0
  for ev in events:
    w = ev.which()
    t = ev.logMonoTime
    if w == "can":
      pending_can.append((t, [(m.address, bytes(m.dat), m.src) for m in ev.can]))
    elif w in SERVICES:
      latest[w] = (t, ev)
      if w == "longitudinalPlan":
        last_plan_t = t
    elif w == "carState":
      last_cs_log = ev.carState
    elif w == "carControl":
      last_cc = ev.carControl
      recent_cc.append(ev.carControl)
      if last_cs_log is None or "longitudinalPlan" not in latest or "radarState" not in latest:
        continue
      # controlsd step
      cs = to_current(last_cs_log, car.CarState)
      cs.softHoldActive = soft_hold
      cs = cs.as_reader()
      plan = latest["longitudinalPlan"][1].longitudinalPlan
      radar = latest["radarState"][1].radarState
      limits = CI.get_pid_accel_limits(CP, cs.vEgo, cs.vCruise * CV.KPH_TO_MS)
      t_since_plan = round((t - last_plan_t) / 1e7) * 0.01
      accel, a_target, jerk = LoC.update(ev.carControl.longActive, cs, plan, limits, t_since_plan, radar)
      if start <= t <= end:
        lc_rows.append(dict(t=t, accel=float(accel), aTarget=float(a_target), jerk=float(jerk),
                            state=str(LoC.long_control_state), rec_accel=ev.carControl.actuators.accel))
    elif w == "sendcan" and last_cc is not None:
      if not pending_can or last_cs_log is None:
        continue
      cc = last_cc
      if t in applied_accel:
        cc = next((c for c in reversed(recent_cc) if c.actuators.accel == applied_accel[t]), last_cc)
      CI.CS.speedcam_engaged = cc.enabled
      CI.CS.speedcam_v_cruise = helper.v_cruise_kph
      CI.update(pending_can)
      pending_can = []
      # card: cruise helper on this step's carState, then the controller
      cs_in = to_current(last_cs_log, car.CarState)
      helper.update_v_cruise(cs_in, ReplaySubMaster(latest, t, cc), is_metric)
      soft_hold = helper._soft_hold_active
      CI.CS.softHoldActive = soft_hold
      soft_brake = getattr(helper, "k9_stop_soft_brake_accel", None)
      if has_trial:
        CI.CS.k9_stop_soft_brake_accel = soft_brake
      recorded = {m.address: bytes(m.dat) for m in ev.sendcan if m.address in SCC_ADDR}
      # Follow the recorded 50 Hz SCC phase (frame % 2 == 0 sends SCC); depends only on the log.
      if (CI.CC.frame % 2 == 0) != (1057 in recorded):
        CI.CC.frame += 1
      CI.CS.out = to_current(last_cs_log, structs.CarState)
      CS = CI.CS.out
      _, sends = CI.apply(to_current(cc, structs.CarControl).as_reader(), t)
      if t < start or t > end:
        continue
      dec = {}
      for addr, dat, _ in sends:
        name = SCC_ADDR.get(addr)
        if name in parsers:
          parsers[name].update([t, [(addr, dat, 0)]])
          dec[name] = {s: parsers[name].vl[name][s] for s in SCC_SIGNALS[name]}
      card_rows.append(dict(
        t=t, sends=[(addr, bytes(dat), bus) for addr, dat, bus in sends], dec=dec, soft_hold=soft_hold,
        soft_brake=soft_brake, v_cruise=float(helper.v_cruise_kph), activate=int(helper._activate_cruise),
        vEgo=CS.vEgo, standstill=CS.standstill, aTarget=cc.actuators.aTarget, lcs=str(cc.actuators.longControlState),
        enabled=cc.enabled, gas=CS.gasPressed, brake=CS.brakePressed,
      ))
  return dict(cp=dict(fp=CP.carFingerprint, flags=CP.flags, opLong=CP.openpilotLongitudinalControl),
              has_trial=has_trial, card=card_rows, longcontrol=lc_rows)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--schema", required=True)
  ap.add_argument("--out", required=True)
  ap.add_argument("--time", default="0", help="K9StopSoftBrakeTime to write, or 'base' to leave params as recorded")
  ap.add_argument("--stop-ease-leftover", action="store_true",
                  help="write StopEaseEnable=1, Dist=80, Speed=30, Accel=-20 (removed settings left on a device)")
  ap.add_argument("--start", type=int, required=True)
  ap.add_argument("--end", type=int, required=True)
  ap.add_argument("segs", nargs="+")
  args = ap.parse_args()
  root = make_params_root(os.environ.get("PARAMS_ROOT"))

  events = read_events(args.schema, args.segs)
  init = next(ev.initData for ev in events if ev.which() == "initData")
  entries = {e.key: bytes(e.value) for e in init.params.entries if "/" not in e.key}
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
  overrides = {"ControlsReady": b"1"}  # card sets this before the first apply()
  if args.time == "base":
    overrides["StopEaseEnable"] = b"0"  # comparison baseline: 131f8b1c with stop easing off
  else:
    overrides["K9StopSoftBrakeTime"] = args.time.encode()
  if args.stop_ease_leftover:
    overrides.update({"StopEaseEnable": b"1", "StopEaseDist": b"80", "StopEaseSpeed": b"30", "StopEaseAccel": b"-20"})
  os.environ["PARAMS_ROOT"] = write_params(root, args.time, entries, overrides)
  is_metric = entries.get("IsMetric", b"0") == b"1"
  out = replay(events, args.start, args.end, is_metric)
  out["git"] = init.gitCommit
  out["time"] = args.time
  out["overrides"] = {k: v.decode() for k, v in overrides.items()}
  out["recorded_stop_ease"] = {k: entries[k].decode() for k in ("StopEaseEnable", "StopEaseDist", "StopEaseSpeed",
                                                                  "StopEaseAccel") if k in entries}
  with open(args.out, "wb") as fh:
    pickle.dump(out, fh)


if __name__ == "__main__":
  main()
