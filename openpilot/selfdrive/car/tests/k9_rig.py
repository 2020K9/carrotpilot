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

  def step(self, accel, state, v, standstill=False, a_target=None, gas=False, brake=False, a_ego=None, lead=None,
           lead_visible=True):
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


def load_frozen_cc(commit, tmp_dir, tag):
  """The real CarController of `commit`, with that commit's k9_stop_taper.py/k9_creep.py (if present) as its imports.
  Everything else under opendbc_repo must be byte-identical to the working tree (asserted)."""
  changed = subprocess.run(["git", "diff", "--name-only", commit, "--", "opendbc_repo"], cwd=ROOT, check=True,
                           capture_output=True, text=True).stdout.split()
  changed = [f for f in changed if "/tests/" not in f]  # test files are never imported by the controller
  # car data shared by all brands (e.g. c6736bca removed KIA_K5_DL3_24_HEV) may differ when the K9 is not touched
  car_data = {f"{HY}/values.py", f"{HY}/fingerprints.py", "opendbc_repo/opendbc/car/fingerprints.py",
              "opendbc_repo/opendbc/car/torque_data/params.toml"}
  for f in set(changed) & car_data:
    diff = subprocess.run(["git", "diff", commit, "--", f], cwd=ROOT, check=True, capture_output=True, text=True).stdout
    assert "K9" not in diff, f
  changed = [f for f in changed if f not in car_data]
  # b6696b37 committed two generated corner-radar dbc files with only the generator's header comments added (CAN-FD
  # corner radar, never loaded by the K9 controller): ignore a *_generated.dbc whose diff adds/removes comment lines only
  for f in [f for f in changed if f.startswith("opendbc_repo/opendbc/dbc/") and f.endswith("_generated.dbc")]:
    diff = subprocess.run(["git", "diff", "-U0", commit, "--", f], cwd=ROOT, check=True, capture_output=True,
                          text=True).stdout.splitlines()
    body = [ln[1:] for ln in diff if ln[:1] in "+-" and not ln.startswith(("+++", "---"))]
    assert all(not ln.strip() or ln.startswith("CM_ \"") for ln in body), f
    changed.remove(f)
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
