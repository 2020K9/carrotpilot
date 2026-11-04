"""K9StopMode 0 / 3 / 5 menu (2026-11-03 17:10, the driver "0.3번5번만 남겨두고 삭제", 17:14 정정): deleted stored values and
the kept menu, checked on the current tree only (no frozen controller from an old commit, so it does not depend on the
k9_rig environment-identity checks). Added 2026-11-04 (fix_1925); no existing test, threshold or input was changed.

Expected values are written out from the rule text (k9_stop_mode_migration.py docstring / carrot_settings.json), not
computed by the code under test:
- marker 9 (06:08 numbering 0-3: 1 kind 13, 2 kind 15, 3 kind 17): 1 -> 0, 2 -> 3, 3 -> 5, 4 / 5 -> 0
- marker 8 (2026-10-30 numbering 0-5: 1 kind 13, 2 kind 16, 3 kind 15, 4 kind 14, 5 kind 17): kind 15 -> 3, kind 17 -> 5,
  every other kind (13 / 16 / 14, off the menu) -> 0
- marker 7 (07:10 numbering 0-3: 1 kind 13, 2 kind 14, 3 kind 15): 3 -> 3, 1 / 2 -> 0
- marker 10 (current): never rewritten; a hand-written 1 / 2 / 4 acts as 0 in the controller
Checks source logic and SCC12/SCC14 encoding, not K9 vehicle behaviour.
"""
import json
from pathlib import Path

import pytest

from opendbc.car.hyundai import carcontroller
from opendbc.car.hyundai import k9_creep as kc
from openpilot.selfdrive.car import k9_stop_mode_migration as mig
from openpilot.selfdrive.car.tests.k9_rig import LCS, Rig, edge_sequence, gentle_sequence, mixed_sequence

SEQS = (mixed_sequence, edge_sequence, gentle_sequence)
SETTINGS = Path(__file__).resolve().parents[2] / "carrot_settings.json"


class FakeParams:
  def __init__(self, **v):
    self.v = dict(v)

  def get(self, k, *a, **kw):
    return self.v.get(k)

  def put_int(self, k, val):
    self.v[k] = int(val)


def _setting():
  data = json.loads(SETTINGS.read_text(encoding="utf-8"))
  found = []

  def walk(o):
    if isinstance(o, dict):
      if o.get("name") == "K9StopMode" and "choices" in o:
        found.append(o)
      for x in o.values():
        walk(x)
    elif isinstance(o, list):
      for x in o:
        walk(x)
  walk(data)
  assert len(found) == 1
  return found[0]


def test_kept_menu_is_0_3_5_with_kinds_15_17():
  assert kc.MENU == {3: kc.V12K9OH, 5: kc.V12K9OGR} and (kc.V12K9OH, kc.V12K9OGR) == (15, 17)
  assert carcontroller.K9_CREEP_MENU == {3: 15, 5: 17}


def test_settings_offer_only_0_3_5_and_store_nothing_else():
  from openpilot.selfdrive.carrot.server.services.params import clamp_numeric
  p = _setting()
  assert p["choices"] == [0, 3, 5] and p["default"] == 0
  for v in (0, 3, 5):
    assert clamp_numeric(float(v), p) == v
  for v in (1, 2, 4):  # deleted numbers can never be stored from the settings page: they become 0
    assert clamp_numeric(float(v), p) == 0


@pytest.mark.parametrize("old,want", [(0, 0), (1, 0), (2, 3), (3, 5), (4, 0), (5, 0)])
@pytest.mark.parametrize("pending", [False, True])
def test_marker9_deleted_and_moved_values(old, want, pending):
  p = FakeParams(K9StopMode=old, K9StopModeVer=90 + old if pending else 9)
  mig.migrate_k9_stop_mode(p)
  assert p.v == {"K9StopMode": want, "K9StopModeVer": 10}
  again = dict(p.v)
  assert mig.migrate_k9_stop_mode(p) is False and p.v == again


@pytest.mark.parametrize("old,want", [(0, 0), (1, 0), (2, 0), (3, 3), (4, 0), (5, 5)])
def test_marker8_kinds_15_17_keep_3_5_others_0(old, want):
  p = FakeParams(K9StopMode=old, K9StopModeVer=8)
  mig.migrate_k9_stop_mode(p)
  assert p.v == {"K9StopMode": want, "K9StopModeVer": 10}


@pytest.mark.parametrize("old,want", [(0, 0), (1, 0), (2, 0), (3, 3)])
def test_marker7_kind15_keeps_3_others_0(old, want):
  p = FakeParams(K9StopMode=old, K9StopModeVer=7)
  mig.migrate_k9_stop_mode(p)
  assert p.v == {"K9StopMode": want, "K9StopModeVer": 10}


@pytest.mark.parametrize("stored", [1, 2, 4])
def test_marker10_deleted_value_is_left_but_acts_as_0(monkeypatch, stored):
  p = FakeParams(K9StopMode=stored, K9StopModeVer=10)
  assert mig.migrate_k9_stop_mode(p) is False and p.v == {"K9StopMode": stored, "K9StopModeVer": 10}
  for seq in SEQS:
    ref = seq(Rig(monkeypatch, K9StopMode=0))
    monkeypatch.undo()
    r = seq(Rig(monkeypatch, K9StopMode=stored))
    monkeypatch.undo()
    assert len(r.can) > 100 and r.can == ref.can and r.scc14 == ref.scc14
    assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0


@pytest.mark.parametrize("mode,kind", [(3, 15), (5, 17)])
def test_kept_menus_select_their_kind_and_change_braking(monkeypatch, mode, kind):
  r = Rig(monkeypatch, K9StopMode=mode)
  r.step(-0.5, LCS.pid, 3.0)
  assert r.cc.k9_creep_mode == mode and r.cc.k9_creep is not None and r.cc.k9_creep.mode == kind
  assert r.params.values["K9StopMode"] == mode  # the controller never writes the menu value
  monkeypatch.undo()
  differs = []
  for seq in SEQS:
    ref = seq(Rig(monkeypatch, K9StopMode=0))
    monkeypatch.undo()
    new = seq(Rig(monkeypatch, K9StopMode=mode))
    monkeypatch.undo()
    differs.append(new.can != ref.can)
  assert any(differs)  # the kept menu really changes the stop braking against 0


def test_menus_3_and_5_are_different_methods(monkeypatch):
  out = {}
  for mode in (3, 5):
    out[mode] = [seq(Rig(monkeypatch, K9StopMode=mode)).can for seq in SEQS]
    monkeypatch.undo()
  assert out[3] != out[5]
