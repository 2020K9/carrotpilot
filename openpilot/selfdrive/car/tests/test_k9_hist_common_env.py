"""공통환경 비교 — 역사적 원검사 대체 아님 (2026-11-05 k9guard_split_0550). Model-only, synthetic inputs (실차 미검증).

This file is a separate, additional check. It does NOT replace, relax or bypass the historical byte-equality guard in
k9_rig.load_frozen_cc: the original frozen tests keep calling that guard unchanged and keep failing/erroring where it refuses.

What this file checks, per reference commit of the original frozen tests:
  1. test_hist_opendbc_vs_current_opendbc_same_k9_can[<commit>]: the reference controller (that commit's carcontroller.py and
     k9_*.py) sends identical K9 CAN frames (addr, bytes, bus, every frame, every k9_rig sequence, every K9 menu value) with
     that commit's own complete opendbc package and with this tree's opendbc package (k9_hist_env.prove).
  2. test_reference_vs_current_controller_in_common_env[<family>]: only after (1) holds for the family's reference commits,
     the family's existing 공통환경 비교 file (test_k9_common_env_<family>.py: the original test objects, same inputs, same
     expected values, same thresholds, collected again with load_frozen_cc_common_env) is run in a fresh pytest and must pass.

Scope limit (stated honestly): (1) only compares what the K9 sends. A change in a common file that the K9 does not use (e.g.
hyundaican.create_acc_commands) passes this file; only the historical byte check refuses it.
"""
import os
import subprocess
import sys
from pathlib import Path

import pytest

from openpilot.selfdrive.car.tests import k9_hist_env, k9_rig

COMMITS = ("aeef67c6", "1d0961da", "badf1ea7bf1e72ccb5d512bfc2aa7f0ae311e7ed", "9df95178c", "b6696b37", "6f518e35", "f9cd0960")
FAMILIES = {
  "creep_mode": ("aeef67c6", "1d0961da"),
  "hard_brake": ("badf1ea7bf1e72ccb5d512bfc2aa7f0ae311e7ed",),
  "stop_v12": ("b6696b37",),
  "stop_v12k9_band": ("6f518e35",),
  "stop_v12k9_fix": ("b6696b37", "f9cd0960"),
}
_PROOF = {}


def _prove(commit, tmp_root):
  if commit not in _PROOF:
    try:
      _PROOF[commit] = k9_hist_env.prove(k9_rig.ROOT, commit, Path(tmp_root) / f"hist_{commit[:9]}")
    except AssertionError as e:
      _PROOF[commit] = e
  if isinstance(_PROOF[commit], AssertionError):
    raise AssertionError(f"공통환경 비교: {_PROOF[commit]}")
  return _PROOF[commit]


@pytest.mark.parametrize("commit", COMMITS)
def test_hist_opendbc_vs_current_opendbc_same_k9_can(commit, tmp_path_factory):
  proof = _prove(commit, tmp_path_factory.mktemp("hist"))
  assert proof["runs"] == len(k9_hist_env.VARIANTS) * len(k9_hist_env.SEQS) and proof["frames"] > 0


@pytest.mark.parametrize("family", sorted(FAMILIES))
def test_reference_vs_current_controller_in_common_env(family, tmp_path_factory):
  tmp = tmp_path_factory.mktemp(f"ce_{family}")
  for commit in FAMILIES[family]:
    _prove(commit, tmp)
  rel = f"openpilot/selfdrive/car/tests/test_k9_common_env_{family}.py"
  env = {k: v for k, v in os.environ.items() if not k.startswith("PYTEST_XDIST") and k != "PYTEST_CURRENT_TEST"}
  p = subprocess.run([sys.executable, "-m", "pytest", "-p", "no:cacheprovider", "-n", "0", "-q", "--hypothesis-seed=0",
                      f"--basetemp={tmp / 'bt'}", rel], cwd=k9_rig.ROOT, env=env, capture_output=True, text=True, timeout=1800)
  assert p.returncode == 0, (p.stdout + p.stderr)[-3000:]
