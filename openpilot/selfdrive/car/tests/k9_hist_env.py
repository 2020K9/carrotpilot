"""Historical-environment proof for k9_rig.load_frozen_cc (2026-11-05 k9guard_0518). Model-only, synthetic inputs (실차 미검증).

k9_rig.load_frozen_cc runs a past K9 controller (the reference commit's carcontroller.py and k9_*.py) against THIS tree's common
modules (hyundaican.py, hyundaicanfd.py, values.py, carstate.py, opendbc.can, car.capnp ...), so the reference side and the
current side of every frozen comparison share one common environment. That is only a historical comparison if the reference
controller sends the same K9 CAN in this tree's common environment as in its own commit's. The byte-equality guard in
load_frozen_cc proves that by text; once an upstream merge has touched the common files it can no longer say anything.

This module proves the same thing by behaviour instead, without any allow list: the reference controller (same files, same
loader as load_frozen_cc) is driven twice, in two fresh interpreters,

  (H) with the reference commit's own complete opendbc package (git archive of <commit>:opendbc_repo/opendbc, read-only), and
  (C) with this tree's opendbc package (the common environment the frozen tests use),

through every k9_rig sequence for every K9 menu value, and every frame's sorted CAN sends (addr, bytes, bus) plus accel out
must be identical. Any difference in what the K9 sends (a product change in a common file) stops the frozen load.

Run as a module:  python -m openpilot.selfdrive.car.tests.k9_hist_env <opendbc parent dir> <commit> <out.json>
"""
import hashlib
import json
import os
import subprocess
import sys
import tarfile
from pathlib import Path

SEQS = ("mixed_sequence", "edge_sequence", "gentle_sequence")
# every K9 menu value any frozen test drives (and the invalid ones that must act as 0), K9 only (make_cp default)
VARIANTS = ([("base", {})] + [(f"K9StopMode={m}", {"K9StopMode": m}) for m in (-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 99)] +
            [(f"K9CreepMode={m}", {"K9CreepMode": m}) for m in (-1, 0, 1, 2, 3, 4, 5, 6, 7, 8, 99)])


def drive(env_parent, commit, tmp_dir):
  sys.path.insert(0, str(env_parent))  # `opendbc` resolves to env_parent/opendbc before anything imports it
  import opendbc
  assert Path(opendbc.__file__).resolve().is_relative_to(Path(env_parent).resolve()), opendbc.__file__
  import pytest
  from openpilot.selfdrive.car.tests import k9_rig
  from openpilot.selfdrive.car.tests.k9_common_env import load_frozen_cc_common_env  # loading part of load_frozen_cc only
  cc = load_frozen_cc_common_env(commit, tmp_dir, "hist")
  out = {"opendbc": str(Path(opendbc.__file__).resolve().parent), "runs": {}}
  for name, values in VARIANTS:
    for seq in SEQS:
      mp = pytest.MonkeyPatch()
      try:
        rig = getattr(k9_rig, seq)(k9_rig.Rig(mp, module=cc, **values))
      finally:
        mp.undo()
      frames = [[[a, d.hex(), b] for a, d, b in f] for f in rig.can]
      out["runs"][f"{name}/{seq}"] = frames
  return out


def digest(frames):
  return hashlib.sha256(json.dumps(frames).encode()).hexdigest()


def prove(root, commit, tmp_dir, timeout=900):
  """Assert the reference controller of `commit` sends identical CAN in its own opendbc (H) and in this tree's (C)."""
  tmp_dir = Path(tmp_dir)
  hist = tmp_dir / "hist_opendbc"
  hist.mkdir(parents=True, exist_ok=True)
  tar = tmp_dir / "hist_opendbc.tar"
  with open(tar, "wb") as f:
    subprocess.run(["git", "archive", commit, "opendbc_repo/opendbc"], cwd=root, check=True, stdout=f)
  with tarfile.open(tar) as t:
    t.extractall(hist, filter="data")
  tar.unlink()
  res = {}
  for side, parent in (("H", hist / "opendbc_repo"), ("C", Path(root) / "opendbc_repo")):
    dump = tmp_dir / f"hist_{side}.json"
    env = {**os.environ, "PYTHONPATH": str(root)}
    p = subprocess.run([sys.executable, "-m", "openpilot.selfdrive.car.tests.k9_hist_env", str(parent), commit, str(dump),
                        str(tmp_dir / f"cc_{side}")], cwd=root, env=env, capture_output=True, text=True, timeout=timeout)
    assert p.returncode == 0, f"{side} env run failed: {p.stderr[-3000:]}"
    res[side] = json.loads(dump.read_text())
  h, c = res["H"]["runs"], res["C"]["runs"]
  assert res["H"]["opendbc"] != res["C"]["opendbc"]
  assert set(h) == set(c) and len(h) == len(VARIANTS) * len(SEQS)
  for k in sorted(h):
    if h[k] != c[k]:
      i = next((i for i, (a, b) in enumerate(zip(h[k], c[k], strict=False)) if a != b), min(len(h[k]), len(c[k])))
      raise AssertionError(f"{commit}: K9 CAN differs between its own opendbc and this tree's common environment: {k} frame {i}" +
                           f" H={h[k][i] if i < len(h[k]) else None} C={c[k][i] if i < len(c[k]) else None}")
  return {"runs": len(h), "frames": sum(len(v) for v in h.values()), "sha256": digest([h[k] for k in sorted(h)])}


if __name__ == "__main__":
  env_parent, commit, out_path, tmp = sys.argv[1:5]
  Path(tmp).mkdir(parents=True, exist_ok=True)
  Path(out_path).write_text(json.dumps(drive(env_parent, commit, tmp)))
