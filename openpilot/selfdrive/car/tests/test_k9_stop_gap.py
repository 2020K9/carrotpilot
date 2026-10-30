"""K9StopMode 2 = 격차수정 (kind 16 V12K9OG; 2026-10-30, 보노보노 15:33 "1번메뉴만들때 내차에서 따온 거하고 1번메뉴와 비교해서
차이가 얼마나 나는지 보고 격차난만큼 조절해" / 17:14 "오차난만큼 실차로그와맞게 보정한거 2번에넣어봐 당장" / 17:29 "이전에하던개선은
4번 / 2번은 격차만큼조정한거"): menu 1 (kind 13) with the measured gap added to the table request before the stop (B17O_GAP_*),
and never a weaker request than kind 13 on the same inputs (its kind 13 shadow).
Pins: never weaker than menu 1 (K9Creep objects with the make_jerk band feedback on random inputs and on a lead hand-over
at 0.0 m after table frames, and the real CarController on the stop sequences), the request at the table speeds, the gap
share cleared outside the stopping state, the menu wiring (2 -> kind 16, 5 -> kind 17, other values 0, other cars and the K9
without camera SCC untouched), the settings / manual names, and the stored-value migration against e0d14a1a4 (the push
before the 0-5 menu: every device runs the method it ran there; a marker-7 2 moves to 4 with kind 14).
Model-only harness with synthetic inputs: source logic and SCC12/SCC14 encoding, not K9 vehicle behaviour (실차 미검증).
"""
import importlib.util
import json
import math
import random
from types import SimpleNamespace

import numpy as np
import pytest

from opendbc.car.hyundai import k9_creep as kc
from opendbc.car.hyundai.k9_creep import K9Creep, V12K9O, V12K9OG
from opendbc.car.hyundai.values import CAR, HyundaiFlags
from openpilot.selfdrive.car import k9_stop_mode_migration as mig
from openpilot.selfdrive.car.tests.k9_rig import LCS, ROOT, _git_show, edge_sequence, gentle_sequence, mixed_sequence
from openpilot.selfdrive.car.tests.test_k9_creep_mode import SEQS, rig, upd
from openpilot.selfdrive.car.tests.test_k9_stop_soft_brake_removed import K9_FLAGS, make_cp
from openpilot.selfdrive.car.tests.test_k9_stop_v12 import FakeParams, _random_inputs

BASE = "e0d14a1a4"  # carrot-wip before the 0-5 menu: K9StopMode 0-3 (1 / 2 / 3 = kinds 13 / 14 / 15), K9StopModeVer 7


def _pair(frames):
  """Kind 13 and kind 16 on the same K9Creep inputs, bands() on every 2nd frame as make_jerk calls it (non-CAN-FD, applied
  frames only, the band from each one's own request, previous band its own); yields (frame, kw, kind 13 out, kind 16 out)."""
  one, two = K9Creep(V12K9O), K9Creep(V12K9OG)
  prev = {one: (0.0, 0.0), two: (0.0, 0.0)}
  for f, kw in enumerate(frames):
    out = {one: one.update(**kw), two: two.update(**kw)}
    if f % 2 == 0:
      for c, (a, _) in out.items():
        cu, cl = float(np.clip(0.9 + a * 0.2, 0, 1.2)), float(np.clip(0.8 + a * 0.2, 0, 1.2))
        prev[c] = c.bands(1.0, kw["v_ego"], cu, cl, *prev[c])[1:] if c.active else (cu, cl)
    yield f, kw, out[one], out[two]


def _handover(lead_v, a_target):
  """Rolling down from 2.0 m/s in the stopping state (table frames), a lead at 0.0 m from lead_v on (kind 8 takes the
  stop and latches StopReq), standstill and a hold, noisy aEgo."""
  for i in range(500):
    v = max(2.0 - 0.006 * i, 0.0)
    yield {"accel": a_target, "a_target": a_target, "stopping": True, "pid": False, "v_ego": v, "v_ego_raw": v,
           "a_ego": -0.35 + 0.1 * math.sin(i / 5), "long_accel": None, "esp_standstill": v == 0.0, "long_active": True,
           "override": False, "gas": False, "brake": False, "can_valid": True, "lead_visible": v <= lead_v,
           "lead_distance": 0.0}


def gap_sequence(rig, seed="k9-gap"):
  """CarController stops in the stopping state from below B17O_SPEED (the table runs) with a weak, a table-like, a strong
  and a briefly urgent plan, and one with a lead at 0.0 m from ~0.53 m/s (kind 8 takes the stop), standstill, restart."""
  rng = random.Random(seed)
  for cycle in range(5):
    for i in range(100):
      rig.step(-0.4, LCS.pid, max(4.0 - 0.01 * i, 3.1), a_target=-0.4, a_ego=-0.4)
    for i in range(250):
      v = max(2.9 - 0.012 * i, 0.0)
      a = -1.2 if cycle == 3 and 150 <= i < 160 else (-0.05, -0.3, -0.6, -0.3, -0.3)[cycle]
      rig.step(a, LCS.stopping, v, standstill=v == 0.0, a_target=a, a_ego=-0.4 + rng.uniform(-0.1, 0.1),
               lead=(0.0, 0.0) if cycle == 4 and v <= 0.53 else None)
    for _ in range(150):
      rig.step(-0.5, LCS.stopping, 0.0, standstill=True, a_target=-0.5, a_ego=0.0)
    for i in range(60):
      rig.step(0.8, LCS.pid, 0.05 * i, standstill=i == 0)
  return rig


RUNS = {**SEQS, "gap": gap_sequence}


def _run(monkeypatch, mode, seq):
  """The CarController of menu `mode` on RUNS[seq]; returns the rig and the sent accel of every frame."""
  r = rig(monkeypatch, mode)
  acc, step = [], r.step

  def rec(*a, **k):
    out = step(*a, **k)
    acc.append(out[0])
    return out

  r.step = rec
  RUNS[seq](r)
  monkeypatch.undo()
  return r, acc


# --- (a) never a weaker request than menu 1 --------------------------------------------------------------------------
@pytest.mark.parametrize("seed", range(6))
def test_never_weaker_than_kind13_on_random_inputs(seed):
  for f, kw, r1, r2 in _pair(kw for kw, _ in _random_inputs(random.Random(f"k9-gap-{seed}"), 20000)):
    assert r2[0] <= r1[0] + 1e-9 and r2[1] == r1[1], (f, kw, r1, r2)


@pytest.mark.parametrize("lead_v", [0.53, 0.8, 0.3])
@pytest.mark.parametrize("a_target", [-0.05, -0.3, -0.6])
def test_never_weaker_than_kind13_on_a_lead_handover(lead_v, a_target):
  """Weaker before the kind 13 floor (check 1, 2026-10-30: 17 frames up to +0.027 m/s^2): a lead at 0.0 m hands the stop
  to kind 8 after table frames while kind 16 runs the narrower band of its stronger request."""
  stronger = 0
  for f, kw, r1, r2 in _pair(_handover(lead_v, a_target)):
    assert r2[0] <= r1[0] + 1e-9 and r2[1] == r1[1], (f, kw, r1, r2)
    stronger += r2[0] < r1[0] - 1e-6
  assert stronger > 0  # the gap was sent before the hand-over


@pytest.mark.parametrize("seq", list(RUNS))
def test_never_weaker_than_menu1_through_the_carcontroller(monkeypatch, seq):
  """The real CarController (ComfortBand and band compensation included): menu 2's sent accel and SCC12 aReq at or below
  menu 1's on every frame, StopReq on the same frames."""
  one, a1 = _run(monkeypatch, 1, seq)
  two, a2 = _run(monkeypatch, 2, seq)
  assert one.cc.k9_creep.mode == V12K9O and two.cc.k9_creep.mode == V12K9OG and len(a1) == len(a2) > 400
  weaker = [(f, x, y) for f, (x, y) in enumerate(zip(a1, a2, strict=True)) if y > x + 1e-9]
  assert not weaker, (len(weaker), weaker[:5])
  assert [f for f, _, _ in one.scc12] == [f for f, _, _ in two.scc12]
  for (f, _, s1), (_, _, s2) in zip(one.scc12, two.scc12, strict=True):
    assert s2["StopReq"] == s1["StopReq"], f
    assert s2["aReqValue"] <= s1["aReqValue"] + 1e-9 and s2["aReqRaw"] <= s1["aReqRaw"] + 1e-9, (f, s1, s2)
  if seq == "gap":
    assert any(y < x - 0.01 for x, y in zip(a1, a2, strict=False))  # the gap was sent


def test_gap_zero_is_menu1_bit_for_bit(monkeypatch):
  """B17O_GAP_A is the gap: all zeros and menu 2 is menu 1 again, CAN bit for bit."""
  for seq in RUNS:
    one, _ = _run(monkeypatch, 1, seq)
    monkeypatch.setattr(kc, "B17O_GAP_A", [0.0] * len(kc.B17O_GAP_BP))
    two, _ = _run(monkeypatch, 2, seq)
    assert one.can == two.can and one.scc14 == two.scc14, seq


# --- (b) the request at the table speeds -----------------------------------------------------------------------------
# -(B17O_TAB_V + B17O_GAP_A) at these speeds, gap share 1.0 (B17O_GAP_STEP 1.0): update if B17O_TAB_V / B17O_GAP_A / STEP change
PINNED = {0.1: -0.63, 0.2: -0.613, 0.35: -0.698, 0.4: -0.6995, 0.45: -0.701, 0.6: -0.9863333, 1.0: -0.9924286, 2.0: -1.333}


@pytest.mark.parametrize("v", sorted(PINNED))
def test_request_is_table_plus_gap(v):
  tab = float(np.interp(v, kc.B17O_TAB_BP, kc.B17O_TAB_V))
  gap = max(float(np.interp(v, kc.B17O_GAP_BP, kc.B17O_GAP_A)), 0.0)
  c, one = K9Creep(V12K9OG), K9Creep(V12K9O)
  for n in range(1, 6):  # a weaker plan than the table: the table (+ gap) is sent; the share comes in at B17O_GAP_STEP
    kw = {"accel": -0.05, "a_target": -0.05, "v_ego": v, "v_ego_raw": v}
    acc, stop = upd(c, **kw)
    assert stop is False and upd(one, **kw)[0] == pytest.approx(-tab, abs=1e-9)
    assert acc == pytest.approx(-(tab + min(1.0, n * kc.B17O_GAP_STEP) * gap), abs=1e-9), n
    if n == 1:
      assert acc == pytest.approx(PINNED[v], abs=1e-6)  # B17O_GAP_STEP 1.0: the whole gap on the first table frame
  acc, _ = upd(c, accel=-0.99, a_target=-0.99, v_ego=v, v_ego_raw=v)  # a plan above the urgent -1.0: the stronger one
  assert acc == pytest.approx(min(-0.99, -(tab + gap)), abs=1e-9)


# --- (c) the gap share -----------------------------------------------------------------------------------------------
def test_gap_share_only_in_the_stopping_state():
  c, one = K9Creep(V12K9OG), K9Creep(V12K9O)
  seen = []
  for i in range(150):  # the table window, rolling down to 0.3 m/s
    v = max(2.5 - 0.015 * i, 0.3)
    for x in (c, one):
      upd(x, accel=-0.05, a_target=-0.05, v_ego=v, v_ego_raw=v)
    seen.append(c.b17g_f)
  assert seen[0] == pytest.approx(min(1.0, kc.B17O_GAP_STEP)) and seen[-1] == 1.0
  assert all(b >= a for a, b in zip(seen, seen[1:], strict=False))  # only up within a stop
  assert one.b17g_f == 0.0  # kind 13 never sets it
  upd(c, stopping=False, pid=True, accel=-0.3, a_target=-0.3)  # out of the stopping state: cleared
  assert c.b17g_f == 0.0
  upd(c, accel=-0.05, a_target=-0.05, v_ego=0.4, v_ego_raw=0.4)  # the next stop starts from 0 again
  assert c.b17g_f == pytest.approx(min(1.0, kc.B17O_GAP_STEP))
  upd(c, accel=-0.05, a_target=-0.05, v_ego=0.4, v_ego_raw=0.4, gas=True)  # a pedal: reset()
  assert c.b17g_f == 0.0
  d = K9Creep(V12K9OG)
  for v in (3.2, 3.0):  # at or above B17O_SPEED the table does not start: no share
    upd(d, accel=-0.05, a_target=-0.05, v_ego=v, v_ego_raw=v)
    assert d.b17g_f == 0.0


# --- (d) menu wiring -------------------------------------------------------------------------------------------------
def test_menu2_is_kind16(monkeypatch):
  assert kc.MENU[2] == kc.V12K9OG == V12K9OG == 16 and kc.V12K9OG in kc.KINDS and kc.MODES == (1, 2, 3, 4, 5)
  c = K9Creep(V12K9OG)
  assert c.v12 and c.b17o and c.b17g and not c.b17s and not c.b17h and not c.b17r
  c5 = K9Creep(kc.V12K9OGR)  # menu 5 (17:31 "...그걸 5번"): the same gap plus the ramp below 1 km/h
  assert kc.MENU[5] == kc.V12K9OGR == 17 and c5.v12 and c5.b17o and c5.b17g and c5.b17r and not c5.b17s and not c5.b17h
  assert not any(K9Creep(k).b17g for k in kc.KINDS if k not in (V12K9OG, kc.V12K9OGR))
  r = rig(monkeypatch, 2)
  r.step(-0.5, LCS.pid, 3.0)
  assert r.cc.k9_creep_mode == 2 and r.cc.k9_creep.mode == V12K9OG
  assert r.params.values["K9StopMode"] == 2  # the controller never writes the menu value


@pytest.mark.parametrize("value", [6, -1, float("nan")])
def test_other_values_act_as_0(monkeypatch, value):
  for seq in (mixed_sequence, edge_sequence, gentle_sequence):
    ref = seq(rig(monkeypatch, 0))
    monkeypatch.undo()
    r = seq(rig(monkeypatch, value))
    monkeypatch.undo()
    assert len(r.can) > 400 and r.can == ref.can and r.scc14 == ref.scc14
    assert r.cc.k9_creep is None and r.cc.k9_creep_mode == 0


@pytest.mark.parametrize("menu", [2, 5])
@pytest.mark.parametrize("cp", ["k7", "no_camera_scc"])
def test_gap_menus_never_run_outside_the_k9_camera_scc(monkeypatch, cp, menu):
  kw = {"fingerprint": CAR.KIA_K7} if cp == "k7" else {"flags": int(K9_FLAGS & ~HyundaiFlags.CAMERA_SCC)}
  for seq in (mixed_sequence, edge_sequence, gentle_sequence):
    ref = seq(rig(monkeypatch, 0, cp=make_cp(**kw)))
    monkeypatch.undo()
    new = seq(rig(monkeypatch, menu, cp=make_cp(**kw)))
    monkeypatch.undo()
    assert len(new.can) > 400 and ref.can == new.can and ref.scc14 == new.scc14 and new.cc.k9_creep is None


# --- (e) settings / manual names -------------------------------------------------------------------------------------
def test_settings_and_manual_names():
  catalog = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text(encoding="utf-8"))
  e = next(p for p in catalog["params"] if p["name"] == "K9StopMode")
  assert (e["min"], e["max"], e["default"]) == (0, 5, 0)
  assert (e["options"]["ko"][2], e["options"]["en"][2], e["options"]["zh"][2]) == ("2 격차수정", "2 Gap fix", "2 差距修正")
  assert "2 격차수정" in e["descr"] and "2 Gap fix" in e["edescr"] and "2 差距修正" in e["cdescr"]
  for rel, item in (("docs/user/ko/settings.md", "2: 격차수정"), ("docs/user/en/settings.md", "2: gap fix"),
                    ("docs/k9_menus_zh.md", "2：差距修正")):
    assert item in (ROOT / rel).read_text(encoding="utf-8"), rel


# --- (f) stored-value migration against e0d14a1a4 --------------------------------------------------------------------
MARKERS = [None, 0, 1, 2, 3, 4, 5, 6, 7, 8, 50, 51, 52, 53, 54, 60, 61, 62, 63, 64, 65, -1, "7"]
STORED = [None, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, -1, 99, "x"]


@pytest.fixture(scope="module")
def base(tmp_path_factory):
  """The migration and the menu of e0d14a1a4 (from git, not from this tree)."""
  d, mods = tmp_path_factory.mktemp("gap_base"), {}
  for name, rel in (("mig", "openpilot/selfdrive/car/k9_stop_mode_migration.py"),
                    ("kc", "opendbc_repo/opendbc/car/hyundai/k9_creep.py")):
    path = d / f"base_{name}.py"
    path.write_text(_git_show(BASE, rel), encoding="utf-8")
    spec = importlib.util.spec_from_file_location(f"k9_gap_base_{name}", path)
    mods[name] = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mods[name])
  return SimpleNamespace(**mods)


def _kind(stored, menu):
  """The K9Creep kind the CarController runs for a stored value (get_int, anything outside the menu 0); 0 = base."""
  try:
    m = int(0 if stored is None else stored)
  except (TypeError, ValueError, OverflowError):
    m = 0
  return menu.get(m, 0)


def _start(marker, stored):
  return {**({} if stored is None else {"K9StopMode": stored}), **({} if marker is None else {"K9StopModeVer": marker})}


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


def test_migration_every_older_device_runs_what_it_ran_on_e0d14a1a4(base):
  assert base.kc.MENU == {1: kc.V12K9O, 2: kc.V12K9OS, 3: kc.V12K9OH} and base.mig.STOP_MODE_VER == 7 and mig.STOP_MODE_VER == 8
  for marker in MARKERS:
    for stored in STORED:
      old, new = FakeParams(**_start(marker, stored)), FakeParams(**_start(marker, stored))
      base.mig.migrate_k9_stop_mode(old)
      mig.migrate_k9_stop_mode(new)
      if marker == mig.STOP_MODE_VER:  # 8: already on the 0-5 menu, never touched (no device had it before this push)
        assert new.v == _start(marker, stored)
        continue
      assert new.v["K9StopModeVer"] == mig.STOP_MODE_VER and old.v["K9StopModeVer"] == base.mig.STOP_MODE_VER, (marker, stored)
      assert _kind(new.v.get("K9StopMode"), kc.MENU) == _kind(old.v.get("K9StopMode"), base.kc.MENU), (marker, stored, old.v, new.v)
      want = mig.MOVE_V8.get(old.v.get("K9StopMode"), old.v.get("K9StopMode"))  # 2 -> 4 (kind 14), a 4 / 5 that acted as 0 -> 0
      assert new.v.get("K9StopMode") == want, (marker, stored, old.v, new.v)


def test_migration_power_loss_at_any_write_lands_on_the_same_value():
  for marker in MARKERS:
    for stored in STORED:
      ref = FakeParams(**_start(marker, stored))
      mig.migrate_k9_stop_mode(ref)
      for n in range(10):
        p = _Cut(n, **_start(marker, stored))
        mig.migrate_k9_stop_mode(p)  # never raises
        if p.v.get("K9StopModeVer") == mig.STOP_MODE_VER:
          assert p.v == ref.v, (marker, stored, n)  # the marker is written last
        q = FakeParams(**p.v)  # the next start
        mig.migrate_k9_stop_mode(q)
        assert q.v == ref.v, (marker, stored, n, p.v, q.v)


@pytest.mark.parametrize("marker", [2, 4, 5, 7, 50, 53, 60, 61, 62])
def test_migration_old_4_failed_write_leaves_the_pending_marker(marker):
  """The paths that end on a stored 4 that acted as 0: if writing the 0 fails, the final marker is not written (the pending 74 is)."""
  class NoZero(FakeParams):
    def put_int(self, k, x):
      if k == "K9StopMode" and x == 0:
        raise OSError("write failed")
      super().put_int(k, x)
  p = NoZero(K9StopMode=4, K9StopModeVer=marker)
  mig.migrate_k9_stop_mode(p)
  assert p.v == {"K9StopMode": 4, "K9StopModeVer": mig.MOVE8_PENDING_BASE + 4}
  q = FakeParams(**p.v)
  assert mig.migrate_k9_stop_mode(q) is True and q.v == {"K9StopMode": 0, "K9StopModeVer": mig.STOP_MODE_VER}
