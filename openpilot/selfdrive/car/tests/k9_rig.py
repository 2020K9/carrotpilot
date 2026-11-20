"""Shared CarController rig for the K9 stop tests (moved from the deleted test_k9_stop_taper.py, taper parts removed).

Model-only harness: checks source logic and SCC12/SCC14 encoding through the real CarController, not K9 vehicle
behaviour.
"""
import importlib.util
import random
import subprocess
import sys
from pathlib import Path

from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.hyundai import carcontroller
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import BASE_PARAMS, MemoryParams, make_cp
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import make_ci_state as _base_ci_state

__all__ = ["BASE_PARAMS", "MemoryParams", "make_cp", "make_ci_state"]

LCS = structs.CarControl.Actuators.LongControlState


def make_ci_state():
  """2026-10-25: with .cp = None like the real hyundai CarState before its first update (K9StopMode 1, the v12 page code,
  reads CS.cp directly); everything else as test_k9_stop_soft_brake_removed.make_ci_state."""
  cs = _base_ci_state()
  cs.cp = None
  return cs

ROOT = Path(__file__).resolve().parents[4]
CC_REL = "opendbc_repo/opendbc/car/hyundai/carcontroller.py"
HY = "opendbc_repo/opendbc/car/hyundai"


class Rig:
  """stale_taper=True leaves the deleted K9StopTaper=True in Params (an old device); it must never be read."""
  def __init__(self, monkeypatch, cp=None, stale_taper=False, module=carcontroller, **values):
    self.params = MemoryParams({**BASE_PARAMS, **({"K9StopTaper": True} if stale_taper else {}), **values})
    monkeypatch.setattr(module, "Params", lambda *a, **k: self.params)
    self.cc = module.CarController({Bus.pt: "hyundai_kia_generic"}, make_cp() if cp is None else cp)
    self.parser = CANParser("hyundai_kia_generic", [("SCC12", 0), ("SCC14", 0)], 0)
    self.frame = 0
    self.can = []
    self.scc14 = []
    self.scc12 = []  # (frame, v, decoded SCC12) of frames that sent SCC12
    self.inputs = []  # (longControlState, v, lead or None, lead_visible) of every step, index = frame

  def step(self, accel, state, v, standstill=False, a_target=None, gas=False, brake=False, a_ego=None, lead=None,
           lead_visible=True):
    self.inputs.append((state, v, lead, lead_visible))
    cc = structs.CarControl(enabled=True, longActive=True)
    cc.actuators.accel, cc.actuators.aTarget = accel, accel if a_target is None else a_target
    cc.actuators.longControlState = state
    if lead is not None:
      cc.hudControl.leadVisible, cc.hudControl.leadDistance, cc.hudControl.leadRelSpeed = lead_visible, lead[0], lead[1]
    cs = sys.modules[__name__].make_ci_state()
    cs.out = structs.CarState(vEgo=v, vEgoRaw=v, standstill=standstill, aEgo=accel if a_ego is None else a_ego,
                              canValid=True, gasPressed=gas, brakePressed=brake)
    out, sends = self.cc.update(cc.as_reader(), cs, self.frame * 10_000_000)
    self.can.append(sorted((addr, bytes(dat), bus) for addr, dat, bus in sends))
    scc = [m for m in sends if m[0] in (1057, 905)]
    sent = None
    if scc:
      self.parser.update([self.frame * 10_000_000, scc])
      if any(m[0] == 1057 for m in scc):
        sent = dict(self.parser.vl["SCC12"])
        self.scc12.append((self.frame, v, sent))
      if any(m[0] == 905 for m in scc):
        self.scc14.append((self.frame, v, dict(self.parser.vl["SCC14"])))
    self.frame += 1
    return out.accel, sent


def mixed_sequence(rig, seed="k9-stop-taper-off"):
  """Repeated stops (no lead/closing lead), pedals, standstill hold, hard braking and restarts with noisy aEgo."""
  rng = random.Random(seed)
  for cycle in range(4):
    lead = (6.0, -0.5) if cycle % 2 else None
    for _ in range(80):
      rig.step(rng.uniform(-1.0, 0.5), LCS.pid, 5.0 + rng.uniform(-0.5, 0.5))
    for i in range(160):
      a = -2.0 if cycle == 3 and 60 <= i < 80 else rng.uniform(-1.4, -0.4)
      rig.step(a, LCS.stopping if i > 40 else LCS.pid, max(1.6 - 0.01 * i, 0.02), a_ego=rng.uniform(-1.2, -0.2),
               brake=cycle == 2 and 100 <= i < 110, lead=lead)
    for i in range(150):
      rig.step(-1.2, LCS.stopping, 0.0, standstill=True, gas=cycle == 1 and 50 <= i < 55)
    for i in range(60):
      rig.step(0.8, LCS.pid, 0.05 * i, standstill=i == 0)
  return rig


def edge_sequence(rig, seed="k9-stop-taper-edges"):
  """Plan/radar lead disagreement, standstill flicker and lagging vEgo, accel sign oscillation, sub-1 km/h creep."""
  rng = random.Random(seed)
  for cycle in range(4):
    for _ in range(60):
      rig.step(-0.8, LCS.pid, 4.0)
    for i in range(140):
      v = max(1.6 - 0.011 * i, 0.03)
      a = 0.05 if cycle == 1 and i % 30 == 29 else rng.uniform(-1.3, -0.6)
      lead = (rng.uniform(1.2, 3.0), -v) if cycle == 2 and i % 7 < 3 else None
      rig.step(a, LCS.stopping if i > 30 else LCS.pid, 0.25 if cycle == 3 and i > 100 else v,
               a_ego=rng.uniform(-1.0, 0.1), lead=lead, lead_visible=i % 2 == 0)
    for i in range(80):
      rig.step(-1.1, LCS.stopping, 0.105 if i < 3 else 0.0, standstill=i not in (5, 6))
    for i in range(40):
      rig.step(0.8, LCS.pid, 0.05 * i, standstill=i == 0)
  return rig


def gentle_sequence(rig, seed="k9-gentle"):
  """Gentle stops above the -1.0 urgent latch (feathering exercised), from 6 m/s, with mild aEgo noise."""
  rng = random.Random(seed)
  for _cycle in range(3):
    for i in range(200):
      rig.step(-0.5, LCS.pid, max(6.0 - 0.02 * i, 1.0), a_target=-0.5, a_ego=-0.5 + rng.uniform(-0.05, 0.05))
    for i in range(200):
      v = max(1.0 - 0.005 * i, 0.06)
      rig.step(-0.6, LCS.pid if v > 0.5 else LCS.stopping, v, a_target=-0.4, a_ego=-0.3 + rng.uniform(-0.05, 0.05))
    for _ in range(100):
      rig.step(-0.6, LCS.stopping, 0.0, standstill=True, a_target=-0.4, a_ego=0.0)
    for i in range(60):
      rig.step(0.8, LCS.pid, 0.05 * i, standstill=i == 0)
  return rig


def _git_show(commit, rel):
  return subprocess.run(["git", "show", f"{commit}:{rel}"], cwd=ROOT, check=True, capture_output=True, text=True,
                        encoding="utf-8").stdout


# 2026-11-11 (k9_230_fix, 대감님 19:27): rebased baselines. Each frozen commit below is replaced by a snapshot commit whose
# tree is carrot-wip bdc77e07 with that commit's K9 brake code put back (carcontroller.py and k9_creep.py / k9_hard_brake.py /
# k9_stop_taper.py byte-for-byte from the commit, plus only the upstream 7d337b05 line `new_actuators.stopObjectActive = ...`
# after `new_actuators.accel = accel`; c62a4716 also takes the 0bc09563 pedal-interceptor removal, pedal hunks only, as its
# CC imports get_pedal_interceptor_params that this tree no longer has). Each snapshot's parent is its original commit; carrot-wip reaches the snapshots
# through an `-s ours` merge (no tree change). The guard below is unchanged and now runs against the snapshot. Proven before
# use: the old CC on its own opendbc + car.capnp and the snapshot CC on this tree send identical CAN bytes and accel
# (k9_230fix_out/proof: K9 and K7, menus None/0-6, mixed / edge / gentle / lead_hold / override sequences).
# 2026-11-17: snapshots moved onto the upstream 2b202df1 merge (opendbc_repo = merge c18b3a04, K9 brake files of the
# old snapshot, carcontroller.py 3-way merged with the upstream CAN-FD Kick changes); parents are the 11-11 snapshots.
# 2026-11-21: snapshots moved onto the upstream 5c970aa5 merge (opendbc_repo = merge 608cfa59, K9 brake files and
# tesla/interface.py of the 11-17 snapshot, carcontroller.py 3-way merged with the upstream change = 2 comment lines);
# parents are the 11-17 snapshots.
REBASED = {
  "aeef67c68039dd29d4a5c207b76cb21c194c1e3a": "1e4604deb91b9acec93fe55b4eb7ab8d40df61a5",
  "1d0961da0a4eeccaa70a2af7ea40b7006ec72eaf": "96db69401051cc687084628534f952ce838bb113",
  "badf1ea7bf1e72ccb5d512bfc2aa7f0ae311e7ed": "f8d58827bd060fecfb610f7c623e2ad0ad72ad7b",
  "f8532bed73e7ee914e4bd6197a396f887ab69ee5": "464d617092be32d9ef2e57b1e8740bedb1b98b62",
  "c62a47160b0e9381cf05eb0c49d3b2beffed5eeb": "0ea4e93d038311a4097d7b7e6313271be5be719e",
  "9df95178c9867ff58d06a2824ecc4f2854f8ec09": "8bf127ddf5a14be015152fd2279eed7e4615ca2e",
  "88114288a04478ec5c030f3baac96a652f0156b2": "c0471216f230eb0e89ce426c7804afe4d0030d55",
  "b6696b3786254cc6f74f237751b4d28b84eb24e5": "85b867b0fba1d86c06f8a521df8f79c9cb4b239f",
  "6f518e3542141b015cc8884273e097ac50a814c3": "9b028f457b581cedd8d20ce96ae9c942f69f9841",
  "f9cd096066f4462578d233bf9936d0489a92dc2a": "def63f13981458a9027bd251cef1719a5e1604c4",
  # 2026-11-17: the 11-11 snapshots themselves (named directly by some tests, e.g. test_k9_stop_v12 PREV) and f7e77539
  "0f07296dc6c50de9d632100111d16ef6ef191fe4": "1e4604deb91b9acec93fe55b4eb7ab8d40df61a5",
  "8a1634a799e5ab9610bc4adba3f5b3bc1c849b0b": "96db69401051cc687084628534f952ce838bb113",
  "1c8b7fff66c7f59198892f60d23d4e89f6e2ca41": "f8d58827bd060fecfb610f7c623e2ad0ad72ad7b",
  "7e74cb5294462ac7c7b54765cb5d0193cbd6f2ff": "464d617092be32d9ef2e57b1e8740bedb1b98b62",
  "f7d247fd7c229b5154dc28eef20e249c9a96a3cb": "0ea4e93d038311a4097d7b7e6313271be5be719e",
  "75ef97f9960cbb5fb23c0e70f536cc0d64bc1771": "8bf127ddf5a14be015152fd2279eed7e4615ca2e",
  "2b72f73f4edf0ae0890761b1ff459efb404e1d61": "c0471216f230eb0e89ce426c7804afe4d0030d55",
  "a9982ace841046f58e7306520ff163086391b418": "85b867b0fba1d86c06f8a521df8f79c9cb4b239f",
  "f726b7cd4c70e1e5f5e1934c7128ff34bb3182cb": "9b028f457b581cedd8d20ce96ae9c942f69f9841",
  "df53bb25a9d96f34b988e5deceeab7d61275d30e": "def63f13981458a9027bd251cef1719a5e1604c4",
  "f7e77539c955ca29f0c337b2f4b6e21d852731f9": "c478dda23ed4b1cb3221515dd27f64c11b46153e",
  # 2026-11-21: the 11-17 snapshots themselves (parents of the 11-21 snapshots)
  "eed8ae2244ac58192f85302b99441e686c560cf5": "1e4604deb91b9acec93fe55b4eb7ab8d40df61a5",
  "00f4227a437114cab794042a4b1a9be27dfb1f9b": "96db69401051cc687084628534f952ce838bb113",
  "aa6f4da7620317d2e2f6e0f5236dbdadd32d04a1": "f8d58827bd060fecfb610f7c623e2ad0ad72ad7b",
  "30403b39ed8f084ebed7460167d4ed1ae8211acc": "464d617092be32d9ef2e57b1e8740bedb1b98b62",
  "788f4ea9945981fef706594462b09f917c3d6f58": "0ea4e93d038311a4097d7b7e6313271be5be719e",
  "c8e19a35322d7d27c3d636c2dbc6006fc3ec2c8c": "8bf127ddf5a14be015152fd2279eed7e4615ca2e",
  "52aacd05e31dc1859f7d3fbd94f4ecadac81f124": "c0471216f230eb0e89ce426c7804afe4d0030d55",
  "f275bce8ecb22b508c58598828c0ff7f464dabea": "85b867b0fba1d86c06f8a521df8f79c9cb4b239f",
  "3f9b3df5725e174ac7e3a1885b3a9642a7fedb39": "9b028f457b581cedd8d20ce96ae9c942f69f9841",
  "8f0f2b3da2438601a412b0216fd7bb1d299dcd29": "def63f13981458a9027bd251cef1719a5e1604c4",
  "946cf856f52f3fb4fbdbfce9cbd997b9f17c622f": "c478dda23ed4b1cb3221515dd27f64c11b46153e",
}


PEDAL_UPSTREAM = "2b202df1"  # carrot upstream carrot-wip the Comma Pedal files were restored from (2026-11-17)
PEDAL_UPSTREAM_FILES = {
  f"{HY}/interface.py", f"{HY}/carstate.py", f"{HY}/pedal_interceptor.py", "opendbc_repo/opendbc/car/pedal_interceptor.py",
  "opendbc_repo/opendbc/can/dbc.py", "opendbc_repo/opendbc/dbc/pedal_interceptor.dbc",
  "opendbc_repo/opendbc/safety/safety/safety_hyundai.h", "opendbc_repo/opendbc/safety/safety/safety_pedal_interceptor.h",
}


def _git_show_bytes(commit, rel):
  return subprocess.run(["git", "show", f"{commit}:{rel}"], cwd=ROOT, check=True, capture_output=True).stdout


def _rebased(commit):
  full = subprocess.run(["git", "rev-parse", f"{commit}^{{commit}}"], cwd=ROOT, check=True, capture_output=True,
                        text=True).stdout.strip()
  return REBASED.get(full, commit)


def load_frozen_cc(commit, tmp_dir, tag):
  """The real CarController of `commit`, with that commit's k9_stop_taper.py/k9_creep.py (if present) as its imports.
  Everything else under opendbc_repo must be byte-identical to the working tree (asserted)."""
  commit = _rebased(commit)
  changed = subprocess.run(["git", "diff", "--name-only", commit, "--", "opendbc_repo"], cwd=ROOT, check=True,
                           capture_output=True, text=True).stdout.split()
  changed = [f for f in changed if "/tests/" not in f]  # test files are never imported by the controller
  # 2026-11-17 14:01 (대감님 "빠진 부분 푸시해"): the Comma Pedal (Ray EV) files are back exactly as in the carrot upstream
  # 2b202df1. That path needs CC_ONLY_CAR + PEDAL_INTERCEPTOR flags on the Ray EV and is never built for the K9, so a file
  # of this list may differ from `commit` only while it is byte-identical to that upstream commit; anything else stops.
  for f in [f for f in changed if f in PEDAL_UPSTREAM_FILES]:
    assert (ROOT / f).read_bytes() == _git_show_bytes(PEDAL_UPSTREAM, f), f
    changed.remove(f)
  # car data shared by all brands (e.g. c6736bca removed KIA_K5_DL3_24_HEV) may differ when the K9 is not touched
  car_data = {f"{HY}/values.py", f"{HY}/fingerprints.py", "opendbc_repo/opendbc/car/fingerprints.py",
              "opendbc_repo/opendbc/car/torque_data/params.toml"}
  for f in set(changed) & car_data:
    diff = subprocess.run(["git", "diff", commit, "--", f], cwd=ROOT, check=True, capture_output=True, text=True).stdout
    # 2026-10-26 (brake_v13): fd0f3cd8 changed only the KIA_K9 CarSpecs line (wheelbase 3.105, mass 2015). Car specs reach
    # both controllers the same way (through the CP of make_cp, built from this tree's values.py), so a hunk near the K9 may
    # change CarSpecs(...) lines only; any other changed line in a hunk that mentions K9 still stops the frozen load.
    for hunk in diff.split("\n@@")[1:]:
      if "K9" in hunk:
        body = [ln[1:] for ln in hunk.splitlines()[1:] if ln[:1] in "+-"]
        assert body and all(ln.strip().startswith("CarSpecs(") for ln in body), f
  changed = [f for f in changed if f not in car_data]
  # b6696b37 committed two generated corner-radar dbc files with only the generator's header comments added (CAN-FD
  # corner radar, never loaded by the K9 controller): ignore a *_generated.dbc whose diff adds/removes comment lines only
  for f in [f for f in changed if f.startswith("opendbc_repo/opendbc/dbc/") and f.endswith("_generated.dbc")]:
    diff = subprocess.run(["git", "diff", "-U0", commit, "--", f], cwd=ROOT, check=True, capture_output=True,
                          text=True).stdout.splitlines()
    body = [ln[1:] for ln in diff if ln[:1] in "+-" and not ln.startswith(("+++", "---"))]
    assert all(not ln.strip() or ln.startswith("CM_ \"") for ln in body), f
    changed.remove(f)
  # 2026-10-26 (brake_v13): the f29102fa upstream merge (6f05191c) changed hyundaicanfd.py inside create_acc_control_scc2
  # only (CAN-FD SCC_CONTROL, never built for the non-CAN-FD K9); accept that file only while every hunk is in that function.
  canfd = f"{HY}/hyundaicanfd.py"
  if canfd in changed:
    diff = subprocess.run(["git", "diff", "-U0", commit, "--", canfd], cwd=ROOT, check=True, capture_output=True,
                          text=True).stdout.splitlines()
    heads = [ln for ln in diff if ln.startswith("@@")]
    # 2026-11-11 (ec79cae3 via upstream merge 3cc3c4b0): create_ccnc_messages (CAN-FD CCNC, built only under
    # `if self.CP.flags & HyundaiFlags.CANFD:` like scc2) may change exactly these two lines; any other line there stops.
    ccnc_old = ['        hold = _blink_hold_side(md, desire, create_ccnc_messages.__dict__.setdefault("_blink_hold", {}))']
    ccnc_new = ['        hold_desire = 0 if getattr(CS.out, "cruiseCancelState", False) else desire',
                '        hold = _blink_hold_side(md, hold_desire, create_ccnc_messages.__dict__.setdefault("_blink_hold", {}))']
    hunks = []
    for ln in diff:
      if ln.startswith("@@"):
        hunks.append((ln, [], []))
      elif hunks and ln[:1] == "-" and not ln.startswith("---"):
        hunks[-1][1].append(ln[1:])
      elif hunks and ln[:1] == "+" and not ln.startswith("+++"):
        hunks[-1][2].append(ln[1:])
    assert heads, canfd
    for head, minus, plus in hunks:
      fn = head.split("@@")[2].strip()
      assert fn.startswith("def create_acc_control_scc2(") or (fn.startswith("def create_ccnc_messages(") and
                                                               minus == ccnc_old and plus == ccnc_new), canfd
    changed.remove(canfd)
  # 2026-11-11 (ec79cae3): car.capnp may only append the CarState field cruiseCancelState @115 (a new ordinal changes no
  # existing field layout; the K9 path never reads it). Exactly this one added line and nothing removed, else stop.
  capnp = "opendbc_repo/opendbc/car/car.capnp"
  if capnp in changed:
    diff = subprocess.run(["git", "diff", "-U0", commit, "--", capnp], cwd=ROOT, check=True, capture_output=True,
                          text=True).stdout.splitlines()
    body = [ln for ln in diff if ln[:1] in "+-" and not ln.startswith(("+++", "---"))]
    assert body == ["+  cruiseCancelState @115 :Bool; # Carrot cruise CANCEL latch, cleared by SET/RES or +/-."], capnp
    changed.remove(capnp)
  # 2026-10-28 (K9CenterToFront, 실험중 메뉴): interface.py may change only the centerToFront assignment (lateral vehicle
  # model; nothing in the longitudinal path under test reads CP.centerToFront). Any other changed line stops the frozen load.
  iface = f"{HY}/interface.py"
  if iface in changed:
    diff = subprocess.run(["git", "diff", "-U0", commit, "--", iface], cwd=ROOT, check=True, capture_output=True,
                          text=True).stdout.splitlines()
    body = [ln[1:].strip() for ln in diff if ln[:1] in "+-" and not ln.startswith(("+++", "---"))]
    ok = ("ret.centerToFront = ret.wheelbase * 0.4", "ret.centerToFront = ret.wheelbase * (ctf / 100.0)",
          "if ctf != 40 and 30 <= ctf <= 80:", "else:",
          # 2026-10-28 23:45 x0.001 unit (400 = 0.4, 405 = 0.405; old x0.01 30..80 still read the same)
          "if ctf != 400 and 300 <= ctf <= 800:", "ret.centerToFront = ret.wheelbase * (ctf / 1000.0)",
          "elif ctf != 40 and 30 <= ctf <= 80:  # old x0.01 value (653441e2), manager_init moves it to x0.001")
    assert body and all(ln in ok or ln.startswith('ctf = params.get_int("K9CenterToFront")') for ln in body), iface
    changed.remove(iface)
  # 2026-11-15 23:41 (대감님 승인 "그냥때려넣어", 노비 23:42 기준판 10개 독립 확인): upstream 189c2faf3 changes exactly one
  # Tesla-only line inside the Tesla CarInterface alpha-long branch (no Hyundai file references tesla, and this rig loads only
  # the Hyundai K9 files). Exactly this one removed + one added line and nothing else in the file, else stop.
  tesla = "opendbc_repo/opendbc/car/tesla/interface.py"
  if tesla in changed:
    diff = subprocess.run(["git", "diff", "-U0", commit, "--", tesla], cwd=ROOT, check=True, capture_output=True,
                          text=True).stdout.splitlines()
    body = [ln for ln in diff if ln[:1] in "+-" and not ln.startswith(("+++", "---"))]
    assert body == ["-      if has_vehicle_bus:",
                    '+      if has_vehicle_bus and Params().get_bool("TeslaAutoSpeedLimit"):'], tesla
    changed.remove(tesla)
  assert set(changed) <= {CC_REL, f"{HY}/k9_creep.py", f"{HY}/k9_stop_taper.py", f"{HY}/k9_hard_brake.py"}, changed
  tmp_dir = Path(tmp_dir)
  saved, injected = {}, []

  def load(name, source):
    path = tmp_dir / f"{tag}_{name.rsplit('.', 1)[-1]}.py"
    path.write_text(source, encoding="utf-8")
    spec = importlib.util.spec_from_file_location(f"{tag}_{name.rsplit('.', 1)[-1]}", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

  try:
    for name, rel in (("opendbc.car.hyundai.k9_stop_taper", f"{HY}/k9_stop_taper.py"),
                      ("opendbc.car.hyundai.k9_creep", f"{HY}/k9_creep.py")):
      try:
        source = _git_show(commit, rel)
      except subprocess.CalledProcessError:
        continue
      module = load(name, source)
      saved[name] = sys.modules.get(name)
      sys.modules[name] = module
      injected.append(name)
    cc = load("carcontroller", _git_show(commit, CC_REL))
  finally:
    for name in injected:
      if saved[name] is None:
        sys.modules.pop(name, None)
      else:
        sys.modules[name] = saved[name]
  assert cc.__file__ != carcontroller.__file__
  return cc
