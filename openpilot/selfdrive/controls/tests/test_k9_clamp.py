"""K9Clamp (k9_clamp.py) pins: the three fixes against aabc88 ej 399efba7 (노비 11/18 13:53) and the menu / alert wiring."""
import json
import os

import numpy as np
import pytest

from openpilot.cereal import log, car
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.selfdrive.controls.lib import k9_clamp as kc

BASEDIR = os.path.join(os.path.dirname(__file__), "..", "..", "..")
N = 33
FIVE = ["K9Clamp", "K9ClampLaneCm", "K9ClampEdgeCm", "K9ClampGain", "K9ClampAlert"]
X = np.linspace(0.0, 100.0, N)


def make_md(path_y=0.0, left=-1.75, right=1.75, lprob=0.9, rprob=0.9, edges=None, edge_std=0.2,
            desire=None, lcs=None, lines=True, position=True):
  md = log.ModelDataV2.new_message()
  if position:
    md.position.x = X.tolist()
    md.position.y = [float(path_y * min(x / 15.0, 1.0)) for x in X]
    md.position.z = [0.0] * N
    md.position.t = np.linspace(0, 10, N).tolist()
  if lines:
    ll = md.init('laneLines', 4)
    for i, y in enumerate((left - 3.5, left, right, right + 3.5)):
      ll[i].x = X.tolist()
      ll[i].y = [float(y)] * N
      ll[i].t = np.linspace(0, 10, N).tolist()
    md.laneLineProbs = [0.5, lprob, rprob, 0.5]
    md.laneLineStds = [0.1] * 4
  if edges is not None:
    re = md.init('roadEdges', 2)
    for i, y in enumerate(edges):
      re[i].x = X.tolist()
      re[i].y = [float(y)] * N
    md.roadEdgeStds = [edge_std, edge_std]
  else:
    md.init('roadEdges', 2)
  if desire is not None:
    md.meta.desire = desire
  if lcs is not None:
    md.meta.laneChangeState = lcs
  return md


def path(md):
  return np.column_stack([np.array(md.position.x), np.array(md.position.y), np.zeros(N)])


def run_corr(c, md, n, v=20.0, enabled=True, blocked=False):
  k = 0.0
  for _ in range(n):
    k = c.update(md, v, enabled, blocked)
  return k


VIOL = make_md(path_y=-0.9)   # 1 s ahead (20 m) the path is 0.85 m from the left line -> 0.5 m violation


# ---------- fix 1: the correction is 0 in the same frame ----------
@pytest.mark.parametrize("cause", ["off", "blocked", "blinker_block", "no_lines", "no_position", "nan_path", "gain0"])
def test_fix1_zero_same_frame(cause, monkeypatch):
  c = kc.SteerCorrector(DT_CTRL)
  k = run_corr(c, VIOL, 200)
  assert k > 1e-4
  md, enabled, blocked = VIOL, True, False
  if cause == "off":
    enabled = False
  elif cause == "blocked":
    md = make_md(path_y=-0.9, lcs=log.LaneChangeState.preLaneChange)
    blocked = kc.lane_change_block(md, False, False, False)
  elif cause == "blinker_block":
    blocked = kc.lane_change_block(VIOL, True, False, False)
  elif cause == "no_lines":
    md = make_md(path_y=-0.9, lines=False)
  elif cause == "no_position":
    md = make_md(path_y=-0.9, position=False)
  elif cause == "nan_path":
    md = make_md(path_y=-0.9)
    md.position.y = [float('nan')] * N
  elif cause == "gain0":
    monkeypatch.setattr(kc, "GAIN", 0.0)
  assert c.update(md, 20.0, enabled, blocked) == 0.0


def test_fix1_atc_blocks():
  cm = log.Event.new_message().init('carrotMan')
  cm.activeCarrot = 2
  cm.xDistToTurn = 50
  assert kc.atc_active_from_carrot_man(cm)
  assert kc.lane_change_block(VIOL, False, False, kc.atc_active_from_carrot_man(cm))
  cm.xDistToTurn = 150
  assert not kc.atc_active_from_carrot_man(cm)


# ---------- fix 2: unreadable settings -> off ----------
class _P:
  def __init__(self, v=None, exc=False):
    self.v, self.exc = v, exc

  def get_int(self, k):
    assert k == "K9Clamp"
    if self.exc:
      raise RuntimeError("unknown key")
    return self.v


@pytest.mark.parametrize("p,on", [(_P(exc=True), False), (_P(None), False), (_P(0), False), (_P(2), False),
                                  (_P(-1), False), (_P(1), True)])
def test_fix2_unreadable_is_off(p, on):
  assert kc.k9_clamp_on(p) is on


# ---------- fix 3: both modes share the dead band, holdoff and block ----------
def _path_step(pc, md, enabled=True, blocked=False, v=20.0):
  p, changed = pc.apply(path(md), md, v, enabled, blocked)
  return changed


def _corr_step(c, md, enabled=True, blocked=False, v=20.0):
  c.update(md, v, enabled, blocked)
  return c.gate.db_active


@pytest.mark.parametrize("mode", ["path", "steer"])
def test_fix3_deadband(mode):
  if mode == "path":
    obj, step = kc.PathClamp(DT_MDL), _path_step
  else:
    obj, step = kc.SteerCorrector(DT_CTRL), _corr_step
  # left line -1.75, lo = -0.40; violation = -0.40 - y
  assert not step(obj, make_md(path_y=-0.55))   # 0.15 m < 0.20 on
  assert step(obj, make_md(path_y=-0.65))       # 0.25 m > 0.20 -> on
  assert step(obj, make_md(path_y=-0.55))       # 0.15 m, still on (off below 0.10)
  assert not step(obj, make_md(path_y=-0.45))   # 0.05 m -> off


@pytest.mark.parametrize("mode", ["path", "steer"])
def test_fix3_holdoff_2s(mode):
  if mode == "path":
    obj, step, dt = kc.PathClamp(DT_MDL), _path_step, DT_MDL
  else:
    obj, step, dt = kc.SteerCorrector(DT_CTRL), _corr_step, DT_CTRL
  md = make_md(path_y=-0.9)
  assert step(obj, md)
  assert not step(obj, md, blocked=True)
  n = int(round(kc.HOLDOFF_SEC / dt))
  for _ in range(n - 1):
    assert not step(obj, md)
  for _ in range(3):
    step(obj, md)
  assert step(obj, md)


@pytest.mark.parametrize("mode", ["path", "steer"])
@pytest.mark.parametrize("what", ["desire", "lcs", "left", "right", "atc"])
def test_fix3_same_block(mode, what):
  md = make_md(path_y=-0.9, desire=log.Desire.laneChangeLeft if what == "desire" else None,
               lcs=log.LaneChangeState.laneChangeStarting if what == "lcs" else None)
  blocked = kc.lane_change_block(md, what == "left", what == "right", what == "atc")
  assert blocked
  if mode == "path":
    pc = kc.PathClamp(DT_MDL)
    p0 = path(md)
    p, changed = pc.apply(p0.copy(), md, 20.0, True, blocked)
    assert not changed and np.array_equal(p, p0)
  else:
    c = kc.SteerCorrector(DT_CTRL)
    assert run_corr(c, md, 50, blocked=blocked) == 0.0


def test_no_block_normal():
  assert not kc.lane_change_block(make_md(), False, False, False)


# ---------- behaviour ----------
def test_path_clamp_moves_inside_only_violation():
  pc = kc.PathClamp(DT_MDL)
  md = make_md(path_y=-0.9)
  p, changed = pc.apply(path(md), md, 20.0, True, False)
  assert changed
  assert np.all(p[:kc.PATH_N, 1] >= -1.75 + kc.MIN_LANE - kc.DEADBAND_OFF - 1e-9)
  assert np.array_equal(p[kc.PATH_N:, 1], path(md)[kc.PATH_N:, 1])   # far plan (t > 2.5 s) untouched
  pc2 = kc.PathClamp(DT_MDL)
  md2 = make_md(path_y=0.0)
  p0 = path(md2)
  p2, changed2 = pc2.apply(p0.copy(), md2, 20.0, True, False)
  assert not changed2 and np.array_equal(p2, p0)


def test_no_flat_extrapolation():
  # plan longer than the lines: no bound past the last line point
  md = make_md(path_y=0.0)
  for i in (1, 2):
    md.laneLines[i].x = (X * 0.3).tolist()
  lo, hi = kc._bounds(kc.ClampGate(DT_MDL), np.array([10.0, 50.0]), (np.array(md.laneLines[1].x), np.array(md.laneLines[1].y)),
                      None, None, None)
  g = kc.ClampGate(DT_MDL)
  g.left_active = True
  lo, hi = kc._bounds(g, np.array([10.0, 50.0]), (np.array(md.laneLines[1].x), np.array(md.laneLines[1].y)), None, None, None)
  assert np.isfinite(lo[0]) and lo[1] == -np.inf


def test_path_clamp_off_untouched():
  pc = kc.PathClamp(DT_MDL)
  md = make_md(path_y=-0.9)
  p0 = path(md)
  p, changed = pc.apply(p0.copy(), md, 20.0, False, False)
  assert not changed and np.array_equal(p, p0)


def test_edge_fallback_and_narrow_lane():
  c = kc.SteerCorrector(DT_CTRL)
  md = make_md(path_y=1.2, lprob=0.1, rprob=0.1, edges=(-3.0, 1.9))
  k = run_corr(c, md, 300)
  assert k < 0.0   # pushed left, away from the right edge
  c2 = kc.SteerCorrector(DT_CTRL)
  md2 = make_md(path_y=0.6, left=-1.2, right=1.2)   # 2.4 m lane: bounds conflict -> centre
  assert run_corr(c2, md2, 300) < 0.0


def test_caps_and_slew():
  c = kc.SteerCorrector(DT_CTRL)
  md = make_md(path_y=-3.0)
  ks = [c.update(md, 3.0, True, False) for _ in range(400)]
  assert max(abs(k) for k in ks) <= kc.MAX_CURVATURE + 1e-12
  assert max(abs(b - a) for a, b in zip([0.0] + ks, ks, strict=False)) <= kc.SLEW_RATE * DT_CTRL + 1e-12
  c2 = kc.SteerCorrector(DT_CTRL)
  k = run_corr(c2, md, 1000, v=30.0)
  assert abs(k) * 30.0 ** 2 <= kc.MAX_LAT_ACCEL + 1e-9


def test_speed_fade():
  assert kc.speed_strength(4.9 / 3.6) == 0.0
  assert kc.speed_strength(7.5 / 3.6) == pytest.approx(0.5)
  assert kc.speed_strength(10.0 / 3.6) == 1.0
  c = kc.SteerCorrector(DT_CTRL)
  assert run_corr(c, VIOL, 200, v=1.0) == 0.0


def test_alert_hysteresis_cooldown():
  c = kc.SteerCorrector(DT_CTRL)
  run_corr(c, VIOL, 300)
  assert c.alert_active
  run_corr(c, make_md(path_y=0.0), 300)
  assert not c.alert_active
  run_corr(c, VIOL, 100)
  assert not c.alert_active   # 5 s cooldown
  run_corr(c, VIOL, 500)
  assert c.alert_active
  c.update(VIOL, 20.0, False, False)
  assert not c.alert_active   # off -> no alert


# ---------- wiring ----------
def test_menu_entry():
  with open(os.path.join(BASEDIR, "selfdrive", "carrot_settings.json"), encoding="utf-8") as f:
    d = json.load(f)
  p = next(x for x in d["params"] if x["name"] == "K9Clamp")
  assert (p["min"], p["max"], p["default"]) == (0, 1, 0)
  for k in ("descr", "edescr", "cdescr", "title", "etitle", "ctitle"):
    assert p[k]
  exp = [g for t in d["menu"] for g in t.get("groups", []) if g.get("id") == "EXPERIMENTAL"]
  assert exp and exp[0]["params"][-6:-1] == FIVE   # 2026-11-21 +K9JetErrDelay last (대감님 16:07)
  by = {x["name"]: x for x in d["params"]}
  assert [(by[n]["min"], by[n]["max"], by[n]["default"], by[n]["unit"]) for n in FIVE[1:]] == \
    [(0, 300, 135, 1), (0, 300, 115, 1), (0, 20, 10, 1), (0, 1, 1, 1)]   # 1 cm steps, ej defaults
  for n in FIVE:
    for k in ("descr", "edescr", "cdescr", "title", "etitle", "ctitle"):
      assert by[n][k]


def test_param_default_off():
  with open(os.path.join(BASEDIR, "common", "params_keys.h"), encoding="utf-8") as f:
    s = f.read()
  for k, v in (("K9Clamp", "0"), ("K9ClampLaneCm", "135"), ("K9ClampEdgeCm", "115"), ("K9ClampGain", "10"), ("K9ClampAlert", "1")):
    assert f'{{"{k}", {{PERSISTENT, INT, "{v}"}}}}' in s


def test_event_alert():
  from openpilot.selfdrive.selfdrived.events import EVENTS, ET
  a = EVENTS[log.OnroadEvent.EventName.k9ClampActive][ET.PERMANENT]
  assert a.audible_alert == car.CarControl.HUDControl.AudibleAlert.prompt
  assert a.alert_text_1 == "Clamp steering correction active"
  cs = log.ControlsState.new_message()
  assert cs.k9ClampActive is False


# ---------- 5 settings (대감님 15:16 "왜 다안넣음?") ----------
class _MP:
  def __init__(self, **kv):
    self.kv = kv

  def get_int(self, k):
    v = self.kv[k]
    if isinstance(v, Exception):
      raise v
    return v

  def get(self, k, return_default=False):
    if k not in self.kv:
      return None
    return self.get_int(k)


def test_cfg_defaults_and_values():
  c = kc.read_cfg(_MP(K9Clamp=1))
  assert (c.enabled, c.min_lane, c.min_edge, c.gain, c.alert) == (True, 1.35, 1.15, 1.0, True)
  c = kc.read_cfg(_MP(K9Clamp=1, K9ClampLaneCm=151, K9ClampEdgeCm=99, K9ClampGain=15, K9ClampAlert=0))
  assert (c.min_lane, c.min_edge, c.gain, c.alert) == (1.51, 0.99, 1.5, False)
  c = kc.read_cfg(_MP(K9Clamp=1, K9ClampLaneCm=999, K9ClampEdgeCm=-5, K9ClampGain=99, K9ClampAlert=RuntimeError()))
  assert (c.min_lane, c.min_edge, c.gain, c.alert) == (3.0, 0.0, 2.0, True)
  assert not kc.read_cfg(_MP(K9Clamp=0, K9ClampLaneCm=151)).enabled
  assert not kc.read_cfg(_MP(K9Clamp=RuntimeError())).enabled


def test_cfg_distance_moves_the_bound():
  md = make_md(path_y=-0.30)            # 1.45 m from the left line
  c135 = kc.SteerCorrector(DT_CTRL)
  assert run_corr(c135, md, 300, enabled=kc.ClampCfg(True)) == 0.0
  c180 = kc.SteerCorrector(DT_CTRL)
  assert run_corr(c180, md, 300, enabled=kc.ClampCfg(True, min_lane=1.80)) > 0.0
  pc = kc.PathClamp(DT_MDL)
  md = make_md(path_y=-0.30, right=2.5)  # wide enough that 1.80 m from both lines does not conflict
  p, ch = pc.apply(path(md), md, 20.0, kc.ClampCfg(True, min_lane=1.80), False)
  assert ch and np.all(p[:kc.PATH_N, 1] >= -1.75 + 1.80 - kc.DEADBAND_OFF - 1e-9)


def test_cfg_gain_scales_and_zero_is_same_frame():
  a = kc.SteerCorrector(DT_CTRL)
  b = kc.SteerCorrector(DT_CTRL)
  ka = run_corr(a, VIOL, 2000, enabled=kc.ClampCfg(True, gain=1.0))
  kb = run_corr(b, VIOL, 2000, enabled=kc.ClampCfg(True, gain=2.0))
  assert kb > ka > 0.0
  assert b.update(VIOL, 20.0, kc.ClampCfg(True, gain=0.0), False) == 0.0


def test_cfg_alert_off_silences_only():
  c = kc.SteerCorrector(DT_CTRL)
  k = run_corr(c, VIOL, 300, enabled=kc.ClampCfg(True, alert=False))
  assert k > 0.0 and not c.alert_active


def test_carrot_web_shows_saves_and_applies_all_five(tmp_path, monkeypatch):
  """Carrot Web: the catalog lists the 5 items, /api/param_set saves them (clamped, 1 cm steps), /api/params_bulk reads them
  back, and the driving code reads the saved values (read_cfg on the same Params)."""
  import asyncio
  from aiohttp import web
  from aiohttp.test_utils import TestClient, TestServer
  from openpilot.common.params import Params
  from openpilot.selfdrive.carrot.server.features import params as fp
  from openpilot.selfdrive.carrot.server.services import param_changes
  monkeypatch.setattr(param_changes, "CARROT_PARAM_CHANGES_PATH", str(tmp_path / "chg.jsonl"), raising=False)
  monkeypatch.setattr(fp, "is_drive_engaged", lambda request: False)
  from openpilot.selfdrive.carrot.server.services.settings import get_settings_cached
  _, _, by_name, _ = get_settings_cached()
  for n in FIVE:
    assert n in by_name
  P = Params()
  for n in FIVE:   # as manager_init does on a fresh install: unset keys get the params_keys.h default
    P.remove(n)
    P.put(n, P.get_default_value(n))

  async def go():
    app = web.Application()
    fp.register(app)
    async with TestClient(TestServer(app)) as cl:
      r = await cl.get("/api/params_bulk?names=" + ",".join(FIVE))
      v = (await r.json())["values"]
      assert [int(v[n]) for n in FIVE] == [0, 135, 115, 10, 1]
      for n, val in (("K9Clamp", 1), ("K9ClampLaneCm", 142), ("K9ClampEdgeCm", 121), ("K9ClampGain", 13), ("K9ClampAlert", 0)):
        r = await cl.post("/api/param_set", json={"name": n, "value": val})
        assert (await r.json())["ok"]
      r = await cl.post("/api/param_set", json={"name": "K9ClampLaneCm", "value": 999})
      assert (await r.json())["value"] == 300
      r = await cl.post("/api/param_set", json={"name": "K9ClampLaneCm", "value": 142})
      r = await cl.get("/api/params_bulk?names=" + ",".join(FIVE))
      v = (await r.json())["values"]
      assert [int(v[n]) for n in FIVE] == [1, 142, 121, 13, 0]
  asyncio.run(go())
  c = kc.read_cfg(Params())
  assert (c.enabled, c.min_lane, c.min_edge, c.gain, c.alert) == (True, 1.42, 1.21, 1.3, False)
  for n in FIVE:
    P.remove(n)
