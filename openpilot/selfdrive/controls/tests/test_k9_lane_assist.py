"""K9LaneAssist (controls/lib/k9_lane.py): the 2026-10-25 lane menu merge, K9LaneCenter from 1 km/h, gain / offset values and
the HUD line. Synthetic frames only, no vehicle data (NOT vehicle-validated)."""
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.controls.lib import k9_lane as KL
from openpilot.selfdrive.controls.lib import k9_lane_center as H
from openpilot.selfdrive.controls.lib import k9_one_lane_hold as OLH
from openpilot.selfdrive.controls.lib import k9_lane_fix as LF
from openpilot.selfdrive.modeld import k9_lr_view as LRV

ROOT = Path(__file__).resolve().parents[3]
X = 192.0 * (np.arange(33) / 32.0) ** 2
DT = H.DT
OLD = ("K9LaneFix", "K9LRView", "K9OneLaneHold", "K9LaneCenter")


class MemParams:
  def __init__(self, **values):
    self.values = dict(values)
    self.puts = []

  def get(self, k, block=False, return_default=False):
    return self.values.get(k)

  def get_int(self, k, *a, **kw):
    v = self.values.get(k, 0)
    if isinstance(v, Exception):
      raise v
    return int(v)

  def put_int(self, k, v):
    self.puts.append((k, v))
    self.values[k] = int(v)


# ---------------------------------------------------------------- menu merge / migration
@pytest.mark.parametrize("old, expect", [
  ({}, None), ({"K9LaneFix": 0, "K9LRView": 0, "K9OneLaneHold": 0, "K9LaneCenter": 0}, None),
  ({"K9LaneFix": 1}, 1), ({"K9LRView": 1}, 1), ({"K9LRView": 2}, 1), ({"K9LRView": 3}, 1), ({"K9LRView": 4}, None),
  ({"K9OneLaneHold": 1}, 1), ({"K9LaneCenter": 1}, 1), ({"K9OneLaneHold": 1, "K9LaneFix": 1}, 1), ({"K9LaneFix": -1}, None),
])
def test_migration(old, expect):
  p = MemParams(**old)
  changed = KL.migrate_k9_lane(p)
  assert p.values.get("K9LaneAssist") == expect
  assert changed == (expect == 1)
  assert p.values["K9LaneVer"] == KL.K9_LANE_VER
  # once only: a later 0 (driver turned it off) is never overwritten again
  p.values["K9LaneAssist"] = 0
  assert not KL.migrate_k9_lane(p) and p.values["K9LaneAssist"] == 0


def test_migration_keeps_an_already_set_switch_and_never_raises():
  p = MemParams(K9LaneAssist=1, K9LaneFix=1)
  assert not KL.migrate_k9_lane(p) and p.values["K9LaneAssist"] == 1 and ("K9LaneAssist", 1) not in p.puts

  class Bad(MemParams):
    def get(self, k, block=False, return_default=False):
      raise KeyError(k)
  assert KL.migrate_k9_lane(Bad()) is False


def test_manager_runs_the_migration_before_processes():
  src = (ROOT / "system/manager/manager.py").read_text()
  init = src[src.index("def manager_init"):src.index("# set unset params to their default value")]
  assert "migrate_k9_lane(params)" in init


def test_every_module_reads_the_one_switch():
  for v, on in ((0, False), (1, True), (2, False), (-1, False), (KeyError(), False)):
    p = MemParams(K9LaneAssist=v, K9LaneFix=1, K9LRView=1, K9OneLaneHold=1, K9LaneCenter=1)   # old keys: no effect
    assert KL.k9_lane_on(p) == on
    assert H.k9_lane_center_mode(p) == (1 if on else 0)
    assert OLH.k9_one_lane_hold_mode(p) == (1 if on else 0)
    assert LF.k9_lane_fix_mode(p) == (1 if on else 0)
    assert LRV.read_mode(p) == (1 if on else 0)
    assert LRV.read_one_lane_hold(p) == on


@pytest.mark.parametrize("raw, gain", [(100, 1.0), (0, 0.0), (200, 2.0), (250, 2.0), (-5, 0.0), (55, 0.55), (KeyError(), 1.0)])
def test_gain_read(raw, gain):
  assert KL.k9_lane_gain(MemParams(K9LaneCenterGain=raw)) == gain


@pytest.mark.parametrize("raw, off", [(0, 0.0), (10, 0.10), (-30, -0.30), (31, 0.30), (-150, -0.30), (KeyError(), 0.0)])
def test_offset_read(raw, off):
  assert KL.k9_lane_offset_m(MemParams(PathOffset=raw)) == pytest.approx(off)


def test_settings_menu_and_keys():
  cat = json.loads((ROOT / "selfdrive/carrot_settings.json").read_text())

  def leaves(nodes):
    for n in nodes:
      if "params" in n:
        yield n
      yield from leaves(n.get("groups", []))
  exp = [g for g in leaves(cat["menu"]) if g["id"] == "EXPERIMENTAL"]
  assert len(exp) == 1 and exp[0]["params"] == ["K9StopMode", "K9LaneAssist", "K9LaneCenterGain", "K9ExtHud", "K9MeterSeason"]
  names = [p["name"] for p in cat["params"]]
  assert not set(OLD) & set(names)
  sw = next(p for p in cat["params"] if p["name"] == "K9LaneAssist")
  assert (sw["min"], sw["max"], sw["default"], sw["control"]) == (0, 1, 0, "select") and len(sw["options"]["ko"]) == 2
  g = next(p for p in cat["params"] if p["name"] == "K9LaneCenterGain")
  assert (g["min"], g["max"], g["default"]) == (0, 200, 100) and "control" not in g
  po = next(p for p in cat["params"] if p["name"] == "PathOffset")
  assert (po["min"], po["max"], po["default"]) == (-150, 150, 0)   # existing carrot param reused, range untouched
  keys = (ROOT / "common/params_keys.h").read_text()
  assert '{"K9LaneAssist", {PERSISTENT, INT, "0"}},' in keys and '{"K9LaneCenterGain", {PERSISTENT, INT, "100"}},' in keys
  assert '{"K9LaneVer", {PERSISTENT, INT}},' in keys
  for k in OLD:   # kept (no effect) for the migration / a downgrade
    assert '{"%s", {PERSISTENT, INT, "0"}},' % k in keys


# ---------------------------------------------------------------- K9LaneCenter from 1 km/h
def frame(y_left, y_right, p=0.9, std=0.05, slope=0.0):
  yl = y_left + slope * X
  yr = y_right + slope * X
  lines = [NS(x=X.tolist(), y=(yl - 3.6).tolist()), NS(x=X.tolist(), y=yl.tolist()),
           NS(x=X.tolist(), y=yr.tolist()), NS(x=X.tolist(), y=(yr + 3.6).tolist())]
  frame.n += 1
  return NS(frameId=frame.n, laneLines=lines, laneLineProbs=[0.5, p, p, 0.5], laneLineStds=[0.1, std, std, 0.1],
            position=NS(x=X.tolist(), y=np.zeros(33).tolist()), meta=NS(laneChangeState=0), action=NS(desiredCurvature=0.0))


frame.n = 0


def run(h, v_fn, seconds, y0=0.0, bias=0.0, model_curv_fn=None, lines_fn=None, plant=True, pressed_fn=None):
  """100 Hz; a model frame every 5 ticks. The car follows the hold plus the weak laneless model pull (c0 -1.34e-3, h -0.055)."""
  y, psi, out, ys, vs, md = y0, 0.0, [], [], [], None
  for k in range(int(round(seconds / DT))):
    t = k * DT
    v = float(v_fn(t))
    if k % 5 == 0:
      md = lines_fn(t, y, psi) if lines_fn is not None else frame(-1.75 - y, 1.75 - y, slope=-psi)
    mc = float(model_curv_fn(t)) if model_curv_fn is not None else 0.0
    kk = h.update(md, v, True, False, bool(pressed_fn(t)) if pressed_fn else False, mc)
    out.append(kk); ys.append(y); vs.append(v)
    if plant:
      psi += v * (kk + bias - 1.34e-3 * y - 0.055 * psi) * DT
      y += v * psi * DT
  return np.array(out), np.array(ys), np.array(vs)


def test_low_speed_hold_from_1kmh():
  for v in (0.30, 0.6, 1.4, 2.0, 2.9):
    out, _, _ = run(H.LaneCenter(), lambda t, v=v: v, 6.0, y0=0.25, plant=False)
    assert out[-1] < 0.0, v                       # car right of centre -> pulled left
  out, _, _ = run(H.LaneCenter(), lambda t: 0.27, 6.0, y0=0.25, plant=False)
  assert np.max(np.abs(out)) == 0.0                # below 1 km/h nothing
  out, _, _ = run(H.LaneCenter(), lambda t: 0.0, 6.0, y0=0.25, plant=False)
  assert np.max(np.abs(out)) == 0.0


@pytest.mark.parametrize("seed", range(5))
def test_low_speed_caps_and_rate_random_stop_and_go(seed):
  """random stop-and-go 0..15 km/h with offsets, bias, noisy lines: never more than K_MAX, never faster than KR_MAX / jerk."""
  rng = np.random.default_rng(seed)
  knots = np.cumsum(rng.uniform(1.0, 4.0, 40)); vk = rng.choice([0.0, 0.0, 0.3, 0.8, 1.5, 2.5, 4.0], 40)
  v_fn = lambda t: float(np.interp(t, knots, vk))
  noise = rng.normal(0.0, 0.02, 100000)
  lines = lambda t, y, p: frame(-1.75 - y + noise[int(t * 20) % 100000], 1.75 - y + noise[int(t * 20 + 7) % 100000], slope=-p)
  out, ys, vs = run(H.LaneCenter(), v_fn, 120.0, y0=float(rng.uniform(-0.3, 0.3)), bias=float(rng.uniform(-2e-4, 2e-4)), lines_fn=lines)
  assert np.max(np.abs(out)) <= H.K_MAX + 1e-12
  dk = np.abs(np.diff(out)) / DT
  lim = np.minimum(H.J_MAX / np.maximum(vs[1:], H.V_MIN) ** 2, H.KR_MAX)
  assert np.all(dk <= lim + 1e-9)
  assert np.max(np.abs(out[vs < H.V_MIN])) <= H.KR_MAX * 1.0 + 1e-12   # at a stop only the last ramp-out is left
  # steering-wheel equivalent (K9 L 3.105 m, SR 12.9): <= 4.6 deg, <= 9.2 deg/s
  assert np.degrees(np.max(np.abs(out)) * 3.105 * 12.9) < 4.7
  assert np.degrees(np.max(dk) * 3.105 * 12.9) < 9.3
  # no flutter: what is left after a 1 s moving average stays below 0.3 deg steering wheel
  hp = out - np.convolve(out, np.ones(100) / 100.0, mode="same")
  assert np.degrees(np.max(np.abs(hp[100:-100])) * 3.105 * 12.9) < 0.4


def test_stop_ramps_out_and_start_ramps_in():
  v_fn = lambda t: 4.0 if t < 10.0 else (max(4.0 - (t - 10.0) * 2.0, 0.0) if t < 20.0 else min((t - 20.0) * 1.0, 4.0))
  out, ys, vs = run(H.LaneCenter(), v_fn, 30.0, y0=0.3, plant=False)
  stop = (np.arange(len(out)) * DT > 13.0) & (np.arange(len(out)) * DT < 20.0)
  assert np.max(np.abs(out[stop])) == 0.0          # fully released while stopped
  assert out[int(9.9 / DT)] < 0.0 and out[-1] < 0.0


def test_turn_at_low_speed_and_no_lines_do_nothing():
  # low-speed intersection turn (curvature 0.1 1/m) and no lane lines: nothing added
  out, _, _ = run(H.LaneCenter(), lambda t: 2.0, 6.0, y0=0.3, plant=False, model_curv_fn=lambda t: 0.1)
  assert np.max(np.abs(out)) == 0.0
  out, _, _ = run(H.LaneCenter(), lambda t: 2.0, 6.0, y0=0.3, plant=False, lines_fn=lambda t, y, p: frame(-1.75 - y, 1.75 - y, p=0.3))
  assert np.max(np.abs(out)) == 0.0
  out, _, _ = run(H.LaneCenter(), lambda t: 2.0, 6.0, y0=0.3, plant=False, lines_fn=lambda t, y, p: frame(-2.5 - y, 2.5 - y))
  assert np.max(np.abs(out)) == 0.0                # width 5 m: implausible -> off (unchanged rule)


def test_integral_does_not_learn_below_11kmh():
  h = H.LaneCenter()
  run(h, lambda t: 2.5, 30.0, y0=0.3, plant=False)
  assert h.integ == 0.0
  run(h, lambda t: 6.0, 5.0, y0=0.3, plant=False)
  assert h.integ > 0.0


def test_above_11kmh_unchanged_at_highway():
  """>= 36 km/h the new caps never bind: identical to the fe527be5 rules (A_MAX / v^2 and J_MAX / v^2)."""
  for v in (10.0, 20.0, 30.0):
    assert H.A_MAX / v ** 2 <= H.K_MAX + 1e-12 and H.J_MAX / v ** 2 <= H.KR_MAX


# ---------------------------------------------------------------- gain / offset
def test_gain_scales_the_push_and_one_is_bit_identical():
  a, _, _ = run(H.LaneCenter(), lambda t: 15.0, 4.0, y0=0.25, plant=False)
  h1 = H.LaneCenter(); h1.gain = 1.0
  b, _, _ = run(h1, lambda t: 15.0, 4.0, y0=0.25, plant=False)
  assert np.array_equal(a, b)
  h0 = H.LaneCenter(); h0.gain = 0.0
  z, _, _ = run(h0, lambda t: 15.0, 4.0, y0=0.25, plant=False)
  assert np.max(np.abs(z)) == 0.0
  h2 = H.LaneCenter(); h2.gain = 2.0
  d, _, _ = run(h2, lambda t: 15.0, 4.0, y0=0.25, plant=False)
  assert abs(d[-1]) > 1.5 * abs(a[-1])
  assert np.max(np.abs(d)) <= min(H.A_MAX / 15.0 ** 2, H.K_MAX) + 1e-12   # caps unchanged


def test_offset_moves_the_target():
  h = H.LaneCenter(); h.off = 0.20                   # keep the car 20 cm right of the centre
  out, ys, _ = run(h, lambda t: 15.0, 60.0, y0=0.0)
  assert out[100:].max() > 0.0 and 0.10 < ys[-1] < 0.25
  h = H.LaneCenter(); h.off = 0.20
  out, _, _ = run(h, lambda t: 15.0, 5.0, y0=0.20, plant=False)
  assert np.max(np.abs(out)) == 0.0                  # already at the target


def test_one_lane_hold_gain_one_offset_zero_is_bit_identical():
  from openpilot.selfdrive.controls.tests.test_k9_one_lane_hold import frame as of, run as orun   # noqa: F401
  a = OLH.OneLaneHold(); b = OLH.OneLaneHold(); b.gain = 1.0; b.off = 0.0
  assert a.gain == 1.0 and a.off == 0.0
  frames = [of(-1.7, 1.9) for _ in range(40)] + [of(-1.7, 1.9, p_right=0.1) for _ in range(200)]
  outs = []
  for h in (a, b):
    o = []
    for k in range(len(frames) * 5):
      o.append(h.update(frames[k // 5], 15.0, True, False, False, 0.0))
    outs.append(np.array(o))
  assert np.array_equal(outs[0], outs[1])


class _Ctl:
  def __init__(self, params):
    self.params = params; self.is_vw_meb = False; self.lanefull_mode_enabled = False


def _cs(v=15.0):
  return NS(vEgo=v, leftBlinker=False, rightBlinker=False, steeringPressed=False)


def test_values_read_live_within_0_1s():
  p = MemParams(K9LaneAssist=1, K9LaneCenterGain=100, PathOffset=0)
  c = _Ctl(p); md = frame(-1.5, 2.0)
  for _ in range(5):
    H.k9_lane_center_step(c, _cs(), NS(latActive=True), md, 0)
  assert c.k9_lc.gain == 1.0 and c.k9_lc.off == 0.0
  p.values.update(K9LaneCenterGain=150, PathOffset=-40)
  for _ in range(10):
    H.k9_lane_center_step(c, _cs(), NS(latActive=True), md, 0)
  assert c.k9_lc.gain == 1.5 and c.k9_lc.off == pytest.approx(-0.30)
  p.values["K9LaneAssist"] = 0
  for _ in range(10):
    assert H.k9_lane_center_step(c, _cs(), NS(latActive=True), md, 0) == 0.0
  assert c.k9_lc is None


# ---------------------------------------------------------------- HUD line
def test_controls_state_fields():
  from cereal import log
  m = log.ControlsState.new_message(k9LaneState=2, k9LaneAccel=-0.05)
  assert m.k9LaneState == 2 and m.k9LaneAccel == pytest.approx(-0.05)
  src = (ROOT / "selfdrive/controls/controlsd.py").read_text()
  assert "cs.k9LaneState = " in src and "cs.k9LaneAccel = " in src


def _md(y1, y2, p1=0.9, p2=0.9, s=0.05):
  return NS(laneLines=[NS(y=[y1 - 3.6]), NS(y=[y1]), NS(y=[y2]), NS(y=[y2 + 3.6])], laneLineProbs=[0.5, p1, p2, 0.5],
            laneLineStds=[0.1, s, s, 0.1])


def test_hud_offset_and_text():
  import sys
  sys.path.insert(0, str(ROOT / "selfdrive/carrot/cluster"))
  from cluster_lat_values import LaneOffsetFilter, lane_offset_m, lane_values_text
  assert lane_offset_m(_md(-1.9, 1.7)) == pytest.approx(0.10)       # 10 cm right of the centre
  assert lane_offset_m(_md(-1.6, 2.0)) == pytest.approx(-0.20)
  assert lane_offset_m(_md(-1.9, 1.7, p2=0.3)) is None
  assert lane_offset_m(_md(-1.9, 1.7, s=0.5)) is None
  assert lane_offset_m(_md(-3.0, 3.0)) is None and lane_offset_m(None) is None
  assert lane_offset_m(_md(float("nan"), 1.7)) is None
  f = LaneOffsetFilter()
  assert f.update(0.1, 0.0) == pytest.approx(0.1)
  assert 0.1 < f.update(0.3, 0.5) < 0.3
  assert f.update(None, 0.55) is None and f.update(0.2, 0.6) == pytest.approx(0.2)
  assert lane_values_text(0.123, 2, -0.0412) == "OFS +12cm  LC -0.04"
  assert lane_values_text(-0.08, 3, 0.051) == "OFS -8cm  LC +0.05"
  assert lane_values_text(None, 1, 0.0) == "OFS --  LC --"
  assert lane_values_text(0.0, 0, 0.0) == "OFS +0cm  LC off"
  assert lane_values_text(0.05, None, None) == "OFS +5cm  LC --"
  assert lane_values_text(float("nan"), 2, float("nan")) == "OFS --  LC --"


def test_hud_live_source():
  import sys
  sys.path.insert(0, str(ROOT / "selfdrive/carrot/cluster"))
  from cluster_live import OpenpilotLiveSource
  src = object.__new__(OpenpilotLiveSource)
  sm = {"modelV2": _md(-1.9, 1.7), "controlsState": NS(k9LaneState=2, k9LaneAccel=-0.03)}
  src._service_data = lambda n: sm.get(n)
  src._service_alive = lambda n: sm.get(n) is not None
  src._service_receive_time = lambda n: 10.0
  off, st, acc = src._live_lane_values(10.1)
  assert off == pytest.approx(0.10) and st == 2 and acc == pytest.approx(-0.03)
  off, _, _ = src._live_lane_values(11.0)          # stale model frame (> 0.5 s) -> '--'
  assert off is None
  sm.pop("controlsState")
  assert src._live_lane_values(10.2)[1:] == (None, None)
  assert math.isfinite(src._live_lane_values(10.2)[0])
