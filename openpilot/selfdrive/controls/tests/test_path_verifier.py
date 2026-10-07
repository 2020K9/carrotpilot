"""Tests for selfdrive/controls/lib/path_verifier.py.

The module under test is deliberately free of openpilot imports, so it is loaded
by file path: that lets this file run on the python3.9 available offline as well
as on the 3.12 the repo targets.
"""
from __future__ import annotations

import importlib.util
import math
import os

import numpy as np
import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PV = os.path.normpath(os.path.join(_HERE, "..", "lib", "path_verifier.py"))
_spec = importlib.util.spec_from_file_location("path_verifier_under_test", _PV)
pv = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(pv)

DT = 0.01
SUB = 5                      # control cycles per 20 Hz model frame
DT_MDL = DT * SUB
V = 20.0                     # m/s, well above MIN_ACTIVE_SPEED
LANE_HALF = 1.8              # m
EDGE_HALF = 5.0              # m
X = np.linspace(0.0, 80.0, 33)


def road_y(curv: float, x: np.ndarray = X) -> np.ndarray:
  """Lateral offset of a constant-curvature road. y is right positive, so a
  positive curvature bends right -- the desiredCurvature convention."""
  return 0.5 * curv * x * x


def frame(road_curv: float, model_curv: float, *, probs: float = 0.0,
          stds: float = 0.1, edge_std: float = 0.2, v: float = V,
          ego_curv: float = None, model_plan_curv: float = None, **kw):
  """Inputs for one model frame of a constant-curvature road.

  ``road_curv``   what the lane lines / road edges show
  ``model_curv``  modelV2.action.desiredCurvature
  ``ego_curv``    measured curvature (defaults to the road, i.e. we are on it)
  """
  mp = model_curv if model_plan_curv is None else model_plan_curv
  ec = road_curv if ego_curv is None else ego_curv
  out = dict(
    v_ego=v, model_curvature=model_curv,
    plan_x=X, plan_y=road_y(mp),
    lll_prob=probs, rll_prob=probs, lll_std=stds, rll_std=stds,
    lane_x=X, lll_y=road_y(road_curv) - LANE_HALF, rll_y=road_y(road_curv) + LANE_HALF,
    edge_x=X, le_y=road_y(road_curv) - EDGE_HALF, re_y=road_y(road_curv) + EDGE_HALF,
    le_std=edge_std, re_std=edge_std,
    yaw_rate=ec * v, lat_active=True, dt=DT,
  )
  out.update(kw)
  return out


class Sim:
  """Drives PathVerifier at DT with inputs refreshed at the model rate."""

  def __init__(self, **cfg):
    self.v = pv.PathVerifier(**cfg)
    self.out = []
    self.model = []
    self.dbg = []

  def step(self, n_frames: int = 1, **kw):
    for _ in range(n_frames):
      for _ in range(SUB):
        o, d = self.v.update(**kw)
        self.out.append(o)
        self.model.append(float(kw["model_curvature"]))
        self.dbg.append(d)
    return self.out[-1], self.dbg[-1]

  def warm(self, road_curv: float, n_frames: int = 30, **kw):
    """Engaged, lanes clearly visible: arms the verifier and fills lane memory."""
    self.step(n_frames, **frame(road_curv, road_curv, probs=0.9, **kw))

  def lat_accel_dev(self, v: float = V):
    return [abs(o - m) * v * v for o, m in zip(self.out, self.model)]


# ---------------------------------------------------------------------------
# 1. exact passthrough whenever the verifier has no authority
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("name,over", [
  ("lanes_visible", dict(probs=0.9)),
  ("disengaged", dict(lat_active=False)),
  ("steering_pressed", dict(steering_pressed=True)),
  ("slow", dict(v=3.0)),
])
def test_exact_passthrough(name, over):
  s = Sim()
  # a left curve with the model flipping hard right: the worst case for a hold
  for i in range(60):
    mc = -0.004 if i < 30 else +0.004
    o, d = s.step(**frame(-0.004, mc, **over))
  assert all(o == m for o, m in zip(s.out, s.model)), name
  assert max(s.lat_accel_dev()) == 0.0
  assert d["authority"] == 0.0


def test_lane_visible_passthrough_is_bit_exact_after_laneless():
  """Lanes coming back must return the output to the model exactly (steady state)."""
  s = Sim()
  s.warm(-0.004)
  s.step(20, **frame(-0.004, +0.004))          # laneless flip -> hold
  assert max(s.lat_accel_dev()) > 0.0
  n = len(s.out)
  s.step(80, **frame(-0.004, +0.004, probs=0.9))
  # transient decay is allowed (requirement 7c) but the steady state must be exact
  assert s.out[-1] == s.model[-1]
  assert max(s.lat_accel_dev()[n + 200:]) == 0.0


# ---------------------------------------------------------------------------
# 2. a correct change is never delayed
# ---------------------------------------------------------------------------
def test_supported_curve_onset_has_no_delay():
  """A real curve onset: the lanes drop out, then the road and the model bend left
  together.  The verifier must not touch a single sample."""
  s = Sim()
  s.warm(0.0)                                   # straight road, lanes confident
  n0 = len(s.out)
  for i in range(20):                           # laneless, road and model sweep together
    c = -0.0002 * i
    s.step(**frame(c, c, ego_curv=c))
  dev = s.lat_accel_dev()[n0:]
  assert max(dev) == 0.0


def test_static_wrong_value_is_not_corrected():
  """By construction the verifier only damps *changes*: it never injects steering
  the model did not ask for.  A value that was already wrong when the verifier
  gained authority therefore passes through unchanged."""
  s = Sim()
  s.step(60, **frame(-0.004, +0.004, probs=0.0, stds=1.0, ego_curv=0.0))
  d = s.dbg[-1]
  assert d["contradict_conf"] > 0.0             # it does see the contradiction
  assert d["hold_weight"] > 0.0
  assert max(s.lat_accel_dev()) == 0.0          # but there is no change to absorb


def test_unsupported_curve_onset_passes_through():
  """No usable evidence at all -> passthrough, whatever the model does."""
  s = Sim()
  s.step(60, **frame(0.0, 0.0, probs=0.0, stds=1.0, edge_std=1.0))
  n0 = len(s.out)
  for i in range(30):
    s.step(**frame(0.0, -0.0003 * i, probs=0.0, stds=1.0, edge_std=1.0,
                   ego_curv=0.0))
  assert max(s.lat_accel_dev()[n0:]) == 0.0


def test_supported_s_curve_passes():
  """Left-right-left with the evidence following the model: no deviation."""
  s = Sim()
  s.warm(0.0)
  n0 = len(s.out)
  for i in range(80):
    c = 0.004 * math.sin(2.0 * math.pi * i / 40.0)
    s.step(**frame(c, c, ego_curv=c))
  assert max(s.lat_accel_dev()[n0:]) == 0.0


# ---------------------------------------------------------------------------
# 3. the seg5 failure: a short opposite flip is held
# ---------------------------------------------------------------------------
def test_short_opposite_flip_is_held():
  s = Sim()
  s.warm(-0.004)                                # left curve, lanes confident
  o, d = s.step(20, **frame(-0.004, +0.004))    # 1 s of flipped model, laneless
  assert d["decision"] in (pv.DECISION_HOLD, pv.DECISION_CONVERGE)
  assert d["contradict_conf"] > 0.0
  assert d["hold_weight"] > 0.0
  dev = s.lat_accel_dev()
  assert max(dev) > 0.05                        # it really did hold something back
  assert max(dev) <= pv.MAX_LAT_ACCEL_DEVIATION + 1e-9   # requirement 4


def test_lanes_vanish_and_flip_in_the_same_frame(requirement="6a"):
  """The seg5 timing exactly: the lane probs collapse in the same model frame as
  the bad flip.  An arming delay keyed on lane loss would sit in passthrough
  right through the failure."""
  s = Sim()
  s.warm(-0.004)                                # 1.5 s engaged with lanes
  n0 = len(s.out)
  o, d = s.step(16, **frame(-0.004, +0.004))    # probs -> 0 and flip together
  assert d["armed"] is True
  assert d["authority"] > 0.0
  assert max(s.lat_accel_dev()[n0:]) > 0.05


def test_persistent_contradicted_value_converges():
  """The model is never locked out: holding a contradicted value converges."""
  s = Sim()
  s.warm(-0.004)
  s.step(10, **frame(-0.004, +0.004))
  peak = max(s.lat_accel_dev())
  assert peak > 0.05
  s.step(30, **frame(-0.004, +0.004))           # 1.5 s total of the flipped value
  tail = s.lat_accel_dev()[-SUB:]
  assert max(tail) < 0.1 * peak


# ---------------------------------------------------------------------------
# 4. evidence quality gates
# ---------------------------------------------------------------------------
def test_edges_with_large_std_are_ignored():
  """Road edges above EDGE_STD_MAX must contribute nothing, even when they would
  have produced a (correct) contradiction."""
  # no lane memory (the probs are never high enough), no radar, straight ego: the
  # road edges are the only possible source of evidence
  kw = dict(probs=0.0, stds=1.0, ego_curv=0.0)

  def run(edge_std):
    s = Sim()
    s.step(20, **frame(-0.004, -0.004, edge_std=edge_std, **kw))   # model on the road
    n0 = len(s.out)
    s.step(20, **frame(-0.004, +0.004, edge_std=edge_std, **kw))   # then it flips
    return s, n0

  blind, n0 = run(0.9)
  assert blind.dbg[-1]["evidence"]["road_edges"]["confidence"] == 0.0
  assert max(blind.lat_accel_dev()[n0:]) == 0.0

  seeing, n0 = run(0.2)
  d2 = seeing.dbg[-1]
  assert d2["evidence"]["road_edges"]["confidence"] > 0.0
  assert d2["evidence"]["road_edges"]["vote"] == pv.VOTE_CONTRADICT
  assert max(seeing.lat_accel_dev()[n0:]) > 0.0


def test_lead_lane_change_is_not_evidence():
  """A lead sliding sideways while we drive straight is changing lanes; its trail
  says nothing about road curvature."""
  s = Sim()
  f = frame(0.0, 0.0, probs=0.0, stds=1.0, edge_std=1.0, ego_curv=0.0)
  for i in range(60):                           # 3 s: lead drifts 3 m right, we go straight
    s.step(**dict(f, lead_present=True, lead_d_rel=40.0,
                  lead_y_rel=-1.5 + 3.0 * i / 59.0, lead_v_lead=V))
  assert s.dbg[-1]["evidence"]["lead_trail"]["confidence"] == 0.0

  s2 = Sim()
  c = -0.004
  for i in range(60):                           # same geometry, but it is a real curve
    d_rel = 40.0 + 2.0 * math.sin(i / 7.0)      # radar range always breathes a little
    s2.step(**dict(frame(c, c, probs=0.0, stds=1.0, edge_std=1.0, ego_curv=c),
                   lead_present=True, lead_d_rel=d_rel,
                   lead_y_rel=float(road_y(c, np.array([d_rel]))[0]), lead_v_lead=V))
  ev = s2.dbg[-1]["evidence"]["lead_trail"]
  assert ev["confidence"] > 0.0
  assert abs(ev["curvature"] - c) < 0.002


def _roadside_tracks(curv: float, travelled: float, side_off: float = 6.0, n: int = 16):
  """Stationary roadside points as the radar would report them: ground-fixed, so
  they march toward us as we drive."""
  d_rel = 10.0 + (np.linspace(0.0, 70.0, n) - travelled) % 70.0
  y = road_y(curv, d_rel)
  return dict(track_d_rel=np.concatenate([d_rel, d_rel]),
              track_y_rel=np.concatenate([y - side_off, y + side_off]),
              track_v_lead=np.zeros(2 * n),
              track_id=np.arange(2 * n))


def test_stationary_radar_points_give_roadside_evidence():
  """Requirement 6c: a cloud of stationary liveTracks points fits the roadside."""
  s = Sim()
  c = -0.004
  f = frame(c, c, probs=0.0, stds=1.0, edge_std=1.0, ego_curv=c)
  for i in range(60):
    s.step(**dict(f, **_roadside_tracks(c, V * DT_MDL * i)))
  ev = s.dbg[-1]["evidence"]["road_tracks"]
  assert ev["confidence"] > 0.0
  assert ev["curvature"] is not None
  assert abs(ev["curvature"] - c) < 0.002


def test_roadside_cloud_is_dropped_after_a_big_heading_change():
  """At an intersection the roadside behind belongs to the road we are leaving,
  so it must not be allowed to contradict a correct turn."""
  s = Sim()
  f = frame(0.0, 0.0, probs=0.0, stds=1.0, edge_std=1.0, ego_curv=0.0)
  for i in range(40):                           # 2 s straight: fill the cloud
    s.step(**dict(f, **_roadside_tracks(0.0, V * DT_MDL * i)))
  assert s.dbg[-1]["evidence"]["road_tracks"]["confidence"] > 0.0
  # now turn hard left; stop feeding new points so only the old cloud remains
  turn = frame(0.0, -0.02, probs=0.0, stds=1.0, edge_std=1.0, ego_curv=-0.02)
  s.step(10, **turn)                            # 0.5 s at 0.4 rad/s -> 0.2 rad
  assert s.dbg[-1]["evidence"]["road_tracks"]["confidence"] == 0.0


# ---------------------------------------------------------------------------
# 5. requirement 6b: intentional path changes always pass through
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("over", [
  dict(lane_change_active=True),
  dict(desire_turn=True),
  dict(nav_turn=True),
])
def test_intentional_change_passes_through(over):
  s = Sim()
  s.warm(-0.004)
  n0 = len(s.out)
  s.step(20, **frame(-0.004, +0.004, **over))
  assert max(s.lat_accel_dev()[n0:]) == 0.0
  assert s.dbg[-1]["intent"] is True


def test_intent_release_is_delayed():
  """Passthrough persists INTENT_RELEASE_SECONDS after the manoeuvre ends."""
  s = Sim()
  s.warm(-0.004)
  s.step(4, **frame(-0.004, +0.004, lane_change_active=True))
  n0 = len(s.out)
  s.step(20, **frame(-0.004, +0.004))           # 1 s after the lane change ended
  assert max(s.lat_accel_dev()[n0:]) == 0.0
  assert s.dbg[-1]["intent"] is True
  s.step(40, **frame(-0.004, +0.004))           # now past 2 s
  assert s.dbg[-1]["intent"] is False
  assert max(s.lat_accel_dev()[-SUB:]) > 0.0


def test_blinker_is_directional():
  """A blinker only exempts a change toward the side it signals.  In the seg5
  scene the driver had the left blinker on while the model flipped RIGHT."""
  s = Sim()
  s.warm(-0.004)
  n0 = len(s.out)
  s.step(16, **frame(-0.004, +0.004, blinker_left=True))   # signalled left, flipped right
  assert max(s.lat_accel_dev()[n0:]) > 0.0

  s2 = Sim()
  s2.warm(-0.004)
  n0 = len(s2.out)
  s2.step(16, **frame(-0.004, +0.004, blinker_right=True))  # signalled right, moved right
  assert max(s2.lat_accel_dev()[n0:]) == 0.0


def test_blinker_literal_variant():
  """blinker_directional=False is the conservative variant: any blinker exempts."""
  s = Sim(blinker_directional=False)
  s.warm(-0.004)
  n0 = len(s.out)
  s.step(16, **frame(-0.004, +0.004, blinker_left=True))
  assert max(s.lat_accel_dev()[n0:]) == 0.0


# ---------------------------------------------------------------------------
# 6. requirement 6e: fail-safe inputs
# ---------------------------------------------------------------------------
@pytest.mark.parametrize("over", [
  dict(pose_valid=False),
  dict(calibrated=False),
])
def test_invalid_ego_odometry_is_full_passthrough(over):
  s = Sim()
  s.warm(-0.004)
  n0 = len(s.out)
  s.step(20, **frame(-0.004, +0.004, **over))
  assert max(s.lat_accel_dev()[n0:]) == 0.0
  assert s.dbg[-1]["authority"] == 0.0
  assert all(e["confidence"] == 0.0 for e in s.dbg[-1]["evidence"].values())


def test_radar_fault_zeroes_radar_evidence_only():
  s = Sim()
  c = -0.004
  d_rel = np.linspace(10.0, 80.0, 16)
  y_off = road_y(c, d_rel)
  tracks = dict(track_d_rel=np.concatenate([d_rel, d_rel]),
                track_y_rel=np.concatenate([y_off - 6.0, y_off + 6.0]),
                track_v_lead=np.zeros(2 * d_rel.size),
                track_id=np.arange(2 * d_rel.size))
  s.step(60, **dict(frame(c, c, probs=0.9, ego_curv=c), **tracks))
  s.step(10, **dict(frame(c, +0.004, radar_valid=False), **tracks))
  ev = s.dbg[-1]["evidence"]
  for k in pv.RADAR_EVIDENCE:
    assert ev[k]["confidence"] == 0.0, k
  assert ev["lane_memory"]["confidence"] > 0.0   # vision evidence survives


def test_model_frame_drop_zeroes_model_evidence():
  s = Sim()
  s.warm(-0.004)
  s.step(10, **frame(-0.004, +0.004, model_ok=False))
  ev = s.dbg[-1]["evidence"]
  for k in pv.MODEL_EVIDENCE:
    assert ev[k]["confidence"] == 0.0, k


# ---------------------------------------------------------------------------
# 7. requirement 4 / robustness
# ---------------------------------------------------------------------------
def test_nan_and_empty_inputs_are_safe():
  s = Sim()
  s.warm(-0.004)
  o, d = s.step(**frame(-0.004, float("nan")))
  assert o == 0.0 or math.isfinite(o)
  assert d["reason"] == "nonfinite_input"

  o, d = s.step(**dict(frame(-0.004, +0.004), plan_x=None, plan_y=None,
                       lane_x=None, lll_y=None, rll_y=None,
                       edge_x=None, le_y=None, re_y=None))
  assert math.isfinite(o)

  o, d = s.step(**dict(frame(-0.004, +0.004), plan_x=np.array([]), plan_y=np.array([]),
                       yaw_rate=float("nan")))
  assert math.isfinite(o)

  o, d = s.step(**dict(frame(-0.004, +0.004), v_ego=float("nan")))
  assert math.isfinite(o)

  o, d = s.step(**dict(frame(-0.004, +0.004), dt=0.0))
  assert math.isfinite(o)


def test_output_is_always_finite_on_random_input():
  rng = np.random.default_rng(7)
  s = Sim()
  for _ in range(400):
    bad = rng.random() < 0.1
    o, _d = s.step(**dict(
      frame(float(rng.normal(0, 0.01)), float(rng.normal(0, 0.01)),
            probs=float(rng.random()), stds=float(rng.random()),
            edge_std=float(rng.random()), v=float(rng.uniform(0.0, 40.0)),
            steering_pressed=bool(rng.random() < 0.2),
            lat_active=bool(rng.random() < 0.8)),
      **({"yaw_rate": float("nan")} if bad else {})))
    assert math.isfinite(o)


def test_reset_clears_state():
  s = Sim()
  s.warm(-0.004)
  s.step(10, **frame(-0.004, +0.004))
  assert abs(s.v.effect) > 0.0
  s.v.reset()
  assert s.v.effect == 0.0
  assert s.v.deviation == 0.0
  assert s.v.hold_weight == 0.0
  assert s.v.lane_mem is None
  assert s.v.cloud_n == 0
  assert s.v.trails == {}
  assert s.v.t == 0.0
  o, d = s.step(**frame(-0.004, +0.004))
  assert o == d["model"]                        # nothing left to act on


def test_unknown_config_is_rejected():
  with pytest.raises(TypeError):
    pv.PathVerifier(not_a_setting=1.0)


# ---------------------------------------------------------------------------
# 8. requirement 7f: no steps, no chatter
# ---------------------------------------------------------------------------
def _assert_no_added_step(s: Sim, v: float = V, slack: float = 1e-12):
  """Per-cycle |d out| never exceeds |d model| plus the jerk-limit allowance."""
  worst = 0.0
  for i in range(1, len(s.out)):
    d_out = abs(s.out[i] - s.out[i - 1])
    d_mdl = abs(s.model[i] - s.model[i - 1])
    allow = pv.DEV_JERK_LIMIT / max(v * v, 1.0) * DT + slack
    worst = max(worst, d_out - d_mdl - allow)
  assert worst <= 0.0, "added step of %.3e 1/m beyond the jerk allowance" % worst


def test_no_step_at_any_gate_edge():
  """Lanes appear/disappear, steeringPressed on/off, the speed gate, lat on/off
  and a lane change start/end -- every edge must ramp, not step."""
  s = Sim()
  s.warm(-0.004)
  base = dict(road_curv=-0.004, model_curv=+0.004)
  edges = [
    {},                                        # laneless: build a deviation
    dict(probs=0.9),                           # lanes appear
    {},                                        # lanes disappear
    dict(steering_pressed=True),               # driver grabs the wheel
    {},                                        # lets go
    dict(v=4.0),                               # drops under the speed gate
    dict(v=20.0),                              # back over it
    dict(lat_active=False),                    # disengage
    {},                                        # re-engage
    dict(lane_change_active=True),             # lane change starts
    {},                                        # and ends
  ]
  for over in edges:
    v = over.get("v", V)
    n0 = len(s.out)
    s.step(10, **frame(base["road_curv"], base["model_curv"], **over))
    for i in range(max(n0, 1), len(s.out)):
      d_out = abs(s.out[i] - s.out[i - 1])
      d_mdl = abs(s.model[i] - s.model[i - 1])
      assert d_out <= d_mdl + pv.DEV_JERK_LIMIT / max(v * v, 1.0) * DT + 1e-12, over


def test_no_chatter_when_prob_rattles_around_the_gate():
  """The lane probs in the real scene rattle across 0.3 frame by frame.  The
  authority must not switch, and the output must stay smooth."""
  rng = np.random.default_rng(3)
  s = Sim()
  s.warm(-0.004)
  n0 = len(s.out)
  for _ in range(60):
    p = float(np.clip(0.30 + rng.normal(0.0, 0.04), 0.0, 1.0))
    s.step(**frame(-0.004, +0.004, probs=p))
  _assert_no_added_step(s)
  # the deviation may vary with authority but it must not jump cycle to cycle
  dev = [d["deviation"] for d in s.dbg[n0:]]
  jumps = [abs(dev[i] - dev[i - 1]) for i in range(1, len(dev))]
  assert max(jumps) <= pv.DEV_JERK_LIMIT / (V * V) * DT + 1e-12


def test_deviation_never_exceeds_the_lateral_accel_cap():
  rng = np.random.default_rng(11)
  s = Sim()
  s.warm(-0.006)
  for _ in range(200):
    v = float(rng.uniform(6.0, 35.0))
    o, d = s.step(**frame(-0.006, +0.006, v=v))
    assert abs(d["deviation_lat_accel"]) <= pv.MAX_LAT_ACCEL_DEVIATION + 1e-9


def test_config_is_honoured():
  """The sweep knobs actually change behaviour (and nothing else is accepted)."""
  loose = Sim(max_deviation=0.5)
  tight = Sim(max_deviation=0.1)
  for s in (loose, tight):
    s.warm(-0.006)
    s.step(12, **frame(-0.006, +0.006))
  assert max(tight.lat_accel_dev()) <= 0.1 + 1e-9
  assert max(loose.lat_accel_dev()) > max(tight.lat_accel_dev())
