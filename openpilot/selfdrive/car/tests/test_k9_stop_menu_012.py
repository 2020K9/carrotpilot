"""K9StopMode 0 / 1 / 2 (2026-11-05 07:16, 대감님 "메뉴 브레이크 0.3.5 말하는 거면 0.1.2로 바꿔"): the 0 / 3 / 5 menu of
c62a4716 (3 = kind 15, 5 = kind 17, K9StopModeVer 10) gets the numbers 0 / 1 / 2 with the same behaviour. Stored values
move once to K9StopModeVer 11: 3 -> 1, 5 -> 2, 1 / 2 / 4 (acted as 0) -> 0, 0 unchanged, other values left as stored.
(a) the real CarController of this tree against the frozen c62a4716 one: new 0 / 1 / 2 send the old 0 / 3 / 5 bytes;
(b) migration: every (marker, stored) pair ends on the kind it ran on c62a4716; once only; pending 100 + v crash safe.
Model-only harness with synthetic inputs (실차 미검증).
"""
import importlib.util
import json

import pytest

from opendbc.car.hyundai import k9_creep as kc
from openpilot.selfdrive.car import k9_stop_mode_migration as mig
from openpilot.selfdrive.car.tests.k9_rig import ROOT, Rig, _git_show, load_frozen_cc
from openpilot.selfdrive.car.tests.test_k9_stop_gap import RUNS
from openpilot.selfdrive.car.tests.test_k9_stop_v12 import FakeParams

REF = "c62a47160b0e9381cf05eb0c49d3b2beffed5eeb"  # menu 0 / 3 / 5, K9StopModeVer 10
NEW_OF_OLD = {0: 0, 3: 1, 5: 2}


@pytest.fixture(scope="module")
def ref_cc(tmp_path_factory):
  return load_frozen_cc(REF, tmp_path_factory.mktemp("ref_c62"), "cc_c62a4716")


@pytest.fixture(scope="module")
def ref_mig(tmp_path_factory):
  path = tmp_path_factory.mktemp("k9_012_old") / "ref_mig.py"
  path.write_text(_git_show(REF, "openpilot/selfdrive/car/k9_stop_mode_migration.py"), encoding="utf-8")
  spec = importlib.util.spec_from_file_location("k9_012_ref_mig", path)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


def test_menu_is_0_1_2():
  assert kc.MENU == {1: 15, 2: 17} and kc.MODES == (1, 2)
  assert mig.STOP_MODE_VER == 11 and mig.MOVE11_PENDING_BASE == 100
  assert mig.MOVE_V11 == {0: 0, 1: 0, 2: 0, 3: 1, 4: 0, 5: 2}


def test_settings_offer_0_1_2_only():
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  e = next(p for p in catalog["params"] if p["name"] == "K9StopMode")
  assert (e["min"], e["max"], e["choices"]) == (0, 2, [0, 1, 2])
  assert e["options"]["ko"] == ["0 기본 제동", "1 bono1739 원본2 + 선 뒤 1초 뒤 더 세게", "2 격차수정 + 1 km/h 아래 서서히 더 잡기"]
  assert all(len(e["options"][loc]) == 3 for loc in ("ko", "en", "zh"))


@pytest.mark.parametrize("seq", list(RUNS))
@pytest.mark.parametrize("new_mode,old_mode", [(0, 0), (1, 3), (2, 5), (3, 0), (5, 0)])
def test_new_number_sends_the_old_number_bytes(monkeypatch, ref_cc, new_mode, old_mode, seq):
  ref = RUNS[seq](Rig(monkeypatch, module=ref_cc, K9StopMode=old_mode))
  monkeypatch.undo()
  new = RUNS[seq](Rig(monkeypatch, K9StopMode=new_mode))
  monkeypatch.undo()
  assert len(new.can) > 400 and new.can == ref.can and new.scc14 == ref.scc14


MARKERS = [None, 2, 3, 4, 5, 6, 7, 8, 9, 10, 50, 53, 60, 64, 70, 75, 80, 85, 90, 91, 92, 93, 94, 95, -1, "9"]
STORED = [None, 0, 1, 2, 3, 4, 5, 6, 9, 99, -1, "x"]


def _start(marker, stored):
  return {**({} if stored is None else {"K9StopMode": stored}), **({} if marker is None else {"K9StopModeVer": marker})}


def _kind(stored, menu):
  return menu.get(stored, 0) if isinstance(stored, int) and not isinstance(stored, bool) else 0


@pytest.mark.parametrize("marker", MARKERS)
def test_every_device_keeps_its_kind_once(ref_mig, marker):
  for stored in STORED:
    old = FakeParams(**_start(marker, stored))
    ref_mig.migrate_k9_stop_mode(old)
    new = FakeParams(**_start(marker, stored))
    mig.migrate_k9_stop_mode(new)
    assert new.v["K9StopModeVer"] == 11, (marker, stored, new.v)
    o = old.v.get("K9StopMode")
    want = NEW_OF_OLD.get(o, 0 if o in (1, 2, 4) else o)
    assert new.v.get("K9StopMode") == want, (marker, stored, old.v, new.v)
    assert _kind(new.v.get("K9StopMode"), kc.MENU) == _kind(o, {3: 15, 5: 17})
    again = FakeParams(**new.v)
    assert mig.migrate_k9_stop_mode(again) is False and again.v == new.v  # once only


def test_marker10_table():
  for old, want in ((0, 0), (1, 0), (2, 0), (3, 1), (4, 0), (5, 2), (6, 6), (9, 9), (-1, -1)):
    p = FakeParams(K9StopMode=old, K9StopModeVer=10)
    assert mig.migrate_k9_stop_mode(p) is (want != old) and p.v == {"K9StopMode": want, "K9StopModeVer": 11}
    assert mig.migrate_k9_stop_mode(p) is False and p.v == {"K9StopMode": want, "K9StopModeVer": 11}
  for v in (0, 1, 2):  # a marker-11 device (new numbering) is never moved again
    p = FakeParams(K9StopMode=v, K9StopModeVer=11)
    assert mig.migrate_k9_stop_mode(p) is False and p.v == {"K9StopMode": v, "K9StopModeVer": 11}


def test_old_markers_land_on_new_numbers():
  for marker, old, want in ((9, 2, 1), (9, 3, 2), (9, 1, 0), (8, 3, 1), (8, 5, 2), (8, 1, 0), (8, 2, 0), (7, 3, 1), (6, 4, 1)):
    p = FakeParams(K9StopMode=old, K9StopModeVer=marker)
    mig.migrate_k9_stop_mode(p)
    assert p.v == {"K9StopMode": want, "K9StopModeVer": 11}, (marker, old, p.v)


@pytest.mark.parametrize("old", range(6))
def test_pending_marker_100_is_crash_safe(old):
  want = mig.MOVE_V11[old]
  for stored in (old, want):
    p = FakeParams(K9StopMode=stored, K9StopModeVer=100 + old)
    for _ in range(4):
      mig.migrate_k9_stop_mode(p)
    assert p.v == {"K9StopMode": want, "K9StopModeVer": 11}, (old, stored, p.v)


class _Cut(FakeParams):
  def __init__(self, n, **v):
    super().__init__(**v)
    self.n = n

  def put_int(self, k, x):
    if self.n <= 0:
      raise OSError("cut")
    self.n -= 1
    super().put_int(k, x)


@pytest.mark.parametrize("marker", [None, 8, 9, 10, 90, 93, 95])
def test_power_loss_at_any_write(marker):
  for stored in STORED:
    ref = FakeParams(**_start(marker, stored))
    mig.migrate_k9_stop_mode(ref)
    for n in range(14):
      p = _Cut(n, **_start(marker, stored))
      mig.migrate_k9_stop_mode(p)
      q = FakeParams(**p.v)
      mig.migrate_k9_stop_mode(q)
      assert q.v == ref.v, (marker, stored, n, p.v, q.v)
