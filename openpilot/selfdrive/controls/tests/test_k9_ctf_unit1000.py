"""K9CenterToFront x0.001 unit (2026-10-28 23:45 대감님 "기본값이 40이면 내가 셋팅할때 400쓰게 ... 405라고하면 40.5되게").

400 = original 0.4, 405 = 0.405, 300..800; old x0.01 values (30..80) read the same and manager_init moves them once
(40 -> 400, 45 -> 450). Same ratio before and after the move.
"""
import json
import os

import pytest

from openpilot.selfdrive.controls.lib.k9_ctf import (LiveCenterToFront, ctf_ratio, migrate_k9_ctf, K9_CTF_DEFAULT,
                                                     K9_CTF_MIN, K9_CTF_MAX)

WB = 3.105


@pytest.mark.parametrize("raw,ratio", [(400, 0.4), (405, 0.405), (450, 0.45), (455, 0.455), (300, 0.3), (800, 0.8),
                                       (299, 0.4), (801, 0.4), (0, 0.4), (-400, 0.4), (None, 0.4), (4000, 0.4)])
def test_new_unit(raw, ratio):
  assert ctf_ratio(raw) == ratio


@pytest.mark.parametrize("old", range(30, 81))
def test_old_value_same_ratio_after_move(old):
  assert ctf_ratio(old) == ctf_ratio(old * 10)  # exact, same double


class _P:
  def __init__(self, v):
    self.v = v
    self.writes = []

  def get_int(self, k):
    assert k == "K9CenterToFront"
    if isinstance(self.v, Exception):
      raise self.v
    return self.v

  def put_int(self, k, v):
    assert k == "K9CenterToFront"
    self.writes.append(v)
    self.v = v


@pytest.mark.parametrize("old,new", [(40, 400), (45, 450), (30, 300), (80, 800), (41, 410)])
def test_migrate_moves_old_value_once(old, new):
  p = _P(old)
  assert migrate_k9_ctf(p) is True
  assert p.v == new and p.writes == [new]
  assert migrate_k9_ctf(p) is False  # second start: no x10 again
  assert p.writes == [new]
  assert ctf_ratio(new) == ctf_ratio(old)


@pytest.mark.parametrize("v", [400, 405, 300, 800, 0, 29, 81, 299, 801, -40, None])
def test_migrate_leaves_others(v):
  p = _P(v)
  assert migrate_k9_ctf(p) is False and p.writes == []


def test_migrate_never_raises():
  assert migrate_k9_ctf(_P(RuntimeError("no param"))) is False

  class _Bad(_P):
    def put_int(self, k, v):
      raise OSError("ro")
  assert migrate_k9_ctf(_Bad(40)) is False


def test_live_405():
  from types import SimpleNamespace
  CP = SimpleNamespace(brand="hyundai", wheelbase=WB, mass=2010.0, tireStiffnessFactor=0.65)
  live = LiveCenterToFront(CP, 400)
  assert live.poll(400) is None
  assert live.poll(405) == WB * 0.405
  assert live.poll(400) == WB * 0.4


def test_manager_calls_migration_before_defaults():
  import inspect
  from openpilot.system.manager import manager
  src = inspect.getsource(manager.manager_init)
  assert "migrate_k9_ctf(params)" in src
  assert src.index("migrate_k9_ctf(params)") < src.index("set unset params to their default value")


def test_settings_and_params_default():
  root = os.path.join(os.path.dirname(__file__), "..", "..")
  with open(os.path.join(root, "carrot_settings.json")) as f:
    d = json.load(f)
  p = [x for x in d["params"] if x["name"] == "K9CenterToFront"][0]
  assert (p["default"], p["min"], p["max"]) == (K9_CTF_DEFAULT, K9_CTF_MIN, K9_CTF_MAX) == (400, 300, 800)
  assert "x0.001" in p["title"] and "x0.001" in p["etitle"] and "x0.001" in p["ctitle"]
  for k in ("descr", "edescr", "cdescr"):
    assert "405 = 40.5%" in p[k] and "/1000" in p[k], k
  with open(os.path.join(root, "..", "common", "params_keys.h")) as f:
    keys = f.read()
  assert '{"K9CenterToFront", {PERSISTENT, INT, "400"}}' in keys
