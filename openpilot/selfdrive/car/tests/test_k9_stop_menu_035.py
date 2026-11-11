# 2026-11-08: menu 3 is half-gap (kind 18), current marker 12; historical inputs stay unchanged.
"""(Number pins updated 2026-11-05 07:16, 대감님 "0.3.5 ... 0.1.2로 바꿔": this tree runs the old 0 / 3 / 5 as 0 / 1 / 2 and the
final marker is 11 (MOVE_V11 after MOVE_V10); inputs and references unchanged.) K9StopMode 0 / 3 / 5 (2026-11-03 17:10, 대감님 "0.3번5번만 남겨두고 삭제", 17:14 정정): the 06:08 renumbering (0-3, 1 / 2 / 3 =
kinds 13 / 15 / 17, K9StopModeVer 9) was wrong. Only 0 / 3 / 5 stay with their 9df95178c numbers and behaviour: 0 base, 3 = kind
15 (원본2 + the 1 s hold, ramp from 1.5 km/h), 5 = kind 17 (격차수정 + the ramp from 1 km/h); kind 13 left the menu (its code is
the base of 15 / 16 / 17). Stored values move once to K9StopModeVer 10.
(a) the real CarController of this tree against the frozen 9df95178c one: 0 / 3 / 5 (and unset) send the same CAN bytes on
    every frame; 1 / 2 / 4 send what 9df95178c sent for 0;
(b) the stored-value migration per marker: every device ends on the kind it ran on 9df95178c (marker 8 or older / pending
    50..75) or on 88114288a (marker 9 / pending 80..85), except that kinds 13 / 14 / 16 (off the menu) end on 0; the
    marker-9 table itself, idempotence, and a power loss at any write.
Model-only harness with synthetic inputs: source logic and SCC12/SCC14 encoding, not K9 vehicle behaviour (실차 미검증).
"""
import importlib.util
import json
from types import SimpleNamespace

import pytest

from opendbc.car.hyundai import k9_creep as kc
from openpilot.selfdrive.car import k9_stop_mode_migration as mig
from openpilot.selfdrive.car.tests.k9_rig import ROOT, Rig, _git_show, load_frozen_cc
from openpilot.selfdrive.car.tests.test_k9_creep_mode import SEQS
from openpilot.selfdrive.car.tests.test_k9_stop_gap import RUNS
from openpilot.selfdrive.car.tests.test_k9_stop_v12 import FakeParams

REF = "9df95178c"  # carrot-wip before the 06:08 renumbering: menu 0-5, 3 = kind 15, 5 = kind 17, K9StopModeVer 8
PREV = "88114288a"  # the 06:08 renumbering: menu 0-3, 1 / 2 / 3 = kinds 13 / 15 / 17, K9StopModeVer 9
OFF_MENU = (13, 14, 16)  # kinds that have no menu value now


@pytest.fixture(scope="module")
def ref_cc(tmp_path_factory):
  return load_frozen_cc(REF, tmp_path_factory.mktemp("ref_9df"), "cc_9df95178c")


def _load(commit, rel, tag, d):
  path = d / f"{tag}.py"
  path.write_text(_git_show(commit, rel), encoding="utf-8")
  spec = importlib.util.spec_from_file_location(f"k9_035_{tag}", path)
  m = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(m)
  return m


@pytest.fixture(scope="module")
def olds(tmp_path_factory):
  d = tmp_path_factory.mktemp("k9_035_old")
  M, K = "openpilot/selfdrive/car/k9_stop_mode_migration.py", "opendbc_repo/opendbc/car/hyundai/k9_creep.py"
  return SimpleNamespace(ref_mig=_load(REF, M, "ref_mig", d), ref_kc=_load(REF, K, "ref_kc", d),
                         prev_mig=_load(PREV, M, "prev_mig", d), prev_kc=_load(PREV, K, "prev_kc", d))


def _kw(mode):
  return {} if mode is None else {"K9StopMode": mode}


# 2026-11-05 07:16: this tree's number of the 9df95178c menu value (number pin only)
NEW_NUM = {None: None, 0: 0, 1: 6, 2: 4, 3: 1, 4: 5, 5: 2}


def M11(v):
  return mig.MOVE_V11.get(v, v) if isinstance(v, int) and not isinstance(v, bool) else v


# --- (a) behaviour: frame for frame against 9df95178c ----------------------------------------------------------------
def test_menu_is_0_3_5():
  assert kc.MENU == {1: kc.V12K9OH, 2: kc.V12K9OGR, 3: kc.V12K9OGR_HALF} and kc.MODES == (1, 2, 3) and (kc.V12K9OH, kc.V12K9OGR) == (15, 17)
  assert mig.STOP_MODE_VER == 12


@pytest.mark.parametrize("seq", list(RUNS))
@pytest.mark.parametrize("mode", [None, 0, 3, 5])
def test_0_3_5_send_the_9df95178c_bytes(monkeypatch, ref_cc, mode, seq):
  ref = RUNS[seq](Rig(monkeypatch, module=ref_cc, **_kw(mode)))
  monkeypatch.undo()
  new = RUNS[seq](Rig(monkeypatch, **_kw(NEW_NUM[mode])))
  monkeypatch.undo()
  assert len(new.can) > 400 and new.can == ref.can and new.scc14 == ref.scc14
  if mode in (3, 5):
    assert new.cc.k9_creep.mode == ref_cc.K9_CREEP_MENU[mode] == kc.MENU[NEW_NUM[mode]]


@pytest.mark.parametrize("seq", list(SEQS))
@pytest.mark.parametrize("mode", [1, 2, 4])
def test_removed_numbers_send_the_9df95178c_0_bytes(monkeypatch, ref_cc, mode, seq):
  ref = SEQS[seq](Rig(monkeypatch, module=ref_cc, K9StopMode=0))
  monkeypatch.undo()
  new = SEQS[seq](Rig(monkeypatch, K9StopMode=NEW_NUM[mode]))
  monkeypatch.undo()
  assert new.can == ref.can and new.scc14 == ref.scc14 and new.cc.k9_creep is None and new.cc.k9_creep_mode == 0


def test_settings_offer_0_3_5_only():
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  e = next(p for p in catalog["params"] if p["name"] == "K9StopMode")
  assert (e["min"], e["max"], e["choices"]) == (0, 3, [0, 1, 2, 3])
  for loc, three, five in (("ko", "1 bono1739 원본2 + 선 뒤 1초 뒤 더 세게", "2 격차수정 + 1 km/h 아래 서서히 더 잡기"),
                           ("en", "1 bono1739 original2 + harder 1 s after the stop", "2 Gap fix + gradually harder below 1 km/h"),
                           ("zh", "1 bono1739原版2＋停后1秒起加压", "2 差距修正＋1 km/h以下逐渐加大制动")):
    assert e["options"][loc][1] == three and e["options"][loc][2] == five  # the 9df95178c labels of 3 / 5
  js = (ROOT / "openpilot/selfdrive/carrot/web/js/pages/setting.js").read_text(encoding="utf-8")
  assert "?.item?.choices" in js and "out.filter((value) => choices.includes(value))" in js


# --- (b) stored-value migration ---------------------------------------------------------------------------------------
MARKERS = [None, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 50, 51, 52, 53, 60, 61, 62, 63, 64, 70, 71, 72, 73, 74, 75,
           80, 81, 82, 83, 84, 85, 90, 91, 92, 93, 94, 95, 10, 11, -1, "9"]
STORED = [None, 0, 1, 2, 3, 4, 5, 6, 7, 9, 99, -1, "x"]


def _start(marker, stored):
  return {**({} if stored is None else {"K9StopMode": stored}), **({} if marker is None else {"K9StopModeVer": marker})}


def _kind(stored, menu):
  try:
    m = int(0 if stored is None else stored)
  except (TypeError, ValueError, OverflowError):
    m = 0
  return menu.get(m, 0)


def test_marker9_table():
  """The 06:08 numbering (1 / 2 / 3 = kinds 13 / 15 / 17): 1 -> 0, 2 -> 3, 3 -> 5, a 4 / 5 that acted as 0 -> 0."""
  for old, want in ((0, 0), (1, 0), (2, 3), (3, 5), (4, 0), (5, 0), (6, 6), (9, 9), (-1, -1), (99, 99)):
    p = FakeParams(K9StopMode=old, K9StopModeVer=9)
    want = M11(want)  # 2026-11-05 07:16: then 3 -> 1, 5 -> 2
    assert mig.migrate_k9_stop_mode(p) is (want != old) and p.v == {"K9StopMode": want, "K9StopModeVer": 12}
    assert mig.migrate_k9_stop_mode(p) is False and p.v == {"K9StopMode": want, "K9StopModeVer": 12}
  p = FakeParams(K9StopModeVer=9)
  assert mig.migrate_k9_stop_mode(p) is False and p.v == {"K9StopModeVer": 12}


def test_marker8_3_and_5_keep_their_number():
  """The 9df95178c numbering: 3 (kind 15) and 5 (kind 17) end on 3 / 5 (3 -> 2 -> 3, 5 -> 3 -> 5), never on 5 / 0."""
  for old, want in ((0, 0), (1, 0), (2, 0), (3, 3), (4, 0), (5, 5), (6, 6)):
    p = FakeParams(K9StopMode=old, K9StopModeVer=8)
    mig.migrate_k9_stop_mode(p)
    assert p.v == {"K9StopMode": M11(want), "K9StopModeVer": 12}, (old, p.v)  # 2026-11-05 07:16 numbers


@pytest.mark.parametrize("marker", MARKERS)
def test_every_device_runs_what_it_ran_before(olds, marker):
  """Each device ends on the kind it ran before the update: on 9df95178c for its numbering (marker 8 or older, pending
  50..75), on 88114288a for the 06:08 numbering (marker 9, pending 80..85); a kind that left the menu ends on 0."""
  assert olds.ref_mig.STOP_MODE_VER == 8 and olds.prev_mig.STOP_MODE_VER == 9
  assert olds.ref_kc.MENU == {1: 13, 2: 16, 3: 15, 4: 14, 5: 17} and olds.prev_kc.MENU == {1: 13, 2: 15, 3: 17}
  is_int = isinstance(marker, int)
  for stored in STORED:
    new = FakeParams(**_start(marker, stored))
    mig.migrate_k9_stop_mode(new)
    assert new.v["K9StopModeVer"] == mig.STOP_MODE_VER, (marker, stored, new.v)
    if is_int and 90 <= marker <= 95:
      continue  # pending markers of this rule: their own test below
    on9 = is_int and (marker == 9 or 80 <= marker <= 85)
    old_mig, old_kc = (olds.prev_mig, olds.prev_kc) if on9 else (olds.ref_mig, olds.ref_kc)
    if is_int and marker in (10, 11):  # 10: already done; 11: an unknown marker, treated as no marker (the 0-6 menu) by both
      old_mig, old_kc = olds.ref_mig, olds.ref_kc
    old = FakeParams(**_start(marker, stored))
    old_mig.migrate_k9_stop_mode(old)
    if is_int and marker == 10:  # 2026-11-05 07:16: marker 10 now takes MOVE_V11 once
      assert new.v == _start(12, M11(stored))
      continue
    if is_int and marker == 11:  # 2026-11-05 07:16: 11 is the current marker; 2026-11-06 06:21: only 3 / 4 -> 1 / 2 (MERGE_V12)
      assert new.v == _start(12, mig.MERGE_V12[stored] if stored in (3, 4) and not isinstance(stored, bool) else stored)
      continue
    was = _kind(old.v.get("K9StopMode"), old_kc.MENU)
    assert _kind(new.v.get("K9StopMode"), kc.MENU) == (0 if was in OFF_MENU else was), (marker, stored, old.v, new.v)
    again = FakeParams(**new.v)
    assert mig.migrate_k9_stop_mode(again) is False and again.v == new.v  # once only


@pytest.mark.parametrize("old", range(6))
def test_pending_marker_90_is_crash_safe(old):
  """The pre-move value goes into the marker (90 + value) before the value is rewritten: a power loss before or after the
  value write ends on MOVE_V10[old]; repeating the pass any number of times changes nothing."""
  want = mig.MOVE_V10[old]
  for stored in (old, want):  # before / after the value write
    p = FakeParams(K9StopMode=stored, K9StopModeVer=mig.MOVE10_PENDING_BASE + old)
    for _ in range(4):
      mig.migrate_k9_stop_mode(p)
    assert p.v == {"K9StopMode": M11(want), "K9StopModeVer": 12}, (old, stored, p.v)  # 2026-11-05 07:16: then MOVE_V11


class _Cut(FakeParams):
  """put_int works `n` times, then raises: a failed write, or a power loss before write n + 1."""
  def __init__(self, n, **v):
    super().__init__(**v)
    self.n = n

  def put_int(self, k, x):
    if self.n <= 0:
      raise OSError("cut")
    self.n -= 1
    super().put_int(k, x)


@pytest.mark.parametrize("marker", MARKERS)
def test_power_loss_at_any_write_lands_on_the_same_value(marker):
  for stored in STORED:
    ref = FakeParams(**_start(marker, stored))
    mig.migrate_k9_stop_mode(ref)
    for n in range(12):
      p = _Cut(n, **_start(marker, stored))
      mig.migrate_k9_stop_mode(p)  # never raises
      if p.v.get("K9StopModeVer") == mig.STOP_MODE_VER:  # 2026-11-06: MERGE_V12 has no marker
        assert p.v == ref.v, (marker, stored, n)  # the final marker is written last
      q = FakeParams(**p.v)  # the next start
      mig.migrate_k9_stop_mode(q)
      assert q.v == ref.v, (marker, stored, n, p.v, q.v)


def test_pending_ranges_name_one_rule_each():
  ranges = [set(range(b, b + w)) for b, w in ((mig.SWAP_PENDING_BASE, 4), (mig.MOVE_PENDING_BASE, 5), (mig.MOVE8_PENDING_BASE, 6),
                                             (mig.MOVE9_PENDING_BASE, 6), (mig.MOVE10_PENDING_BASE, 6))]
  assert all(not (a & b) for i, a in enumerate(ranges) for b in ranges[i + 1:])
  assert mig.STOP_MODE_VER not in set().union(*ranges)
  assert set(mig.MOVE_V10.values()) <= {0, 3, 5} and not all(mig.MOVE_V10.get(v, v) == v for v in mig.MOVE_V10.values())
