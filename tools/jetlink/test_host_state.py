"""K9JetHostState (실험중 젯슨 상태값 CPU, 11/21): toggle reaches the Jetson load, the host path is the pre-10/21
path, the comma commits only offroad / stopped + assistance off, and a reconnect while driving keeps the last
committed value. Deferred-CUDA harness = ordering/values only, not real GPU validation."""
import ast
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT), str(ROOT / "third_party/jetlink"), str(Path(__file__).resolve().parent)]
from jetlink.client import JetlinkClient
from jetlink.server import session as S
from openpilot.selfdrive.modeld.jetlink.contracts import JETSON_SPEC as SPEC
from openpilot.selfdrive.modeld.jetlink import host_state as H
from test_device_state import runtime  # noqa: F401  (pytest fixture)

PAIRS = {"state_a": "next_state_a", "state_b": "next_state_b"}


def fake_engine(calls):
  buffers = {name: np.zeros(shape, np.float32) for name, shape in SPEC.input_shapes.items()}
  def configure(pairs, **kw):
    calls.append(kw)
  return NS(inputs={n: NS(shape=s) for n, s in SPEC.input_shapes.items()},
            outputs={n: NS(shape=s) for n, s in SPEC.output_shapes.items()},
            host_input=buffers.__getitem__, warm=lambda: "warm", run=lambda: None, last_gpu_us=1,
            close=lambda: None, configure_state_feedback=configure)


def make_host():
  backend = NS(describe=lambda: {"backend": "trt", "runtime_version": "10.3.0", "device": "Orin-sm87"})
  return S.EngineHost(NS(backend=backend, inventory=list))


@pytest.mark.parametrize("host_state", [False, True])
def test_warm_passes_choice_and_default_call_is_unchanged(host_state):
  calls = []
  host = make_host()
  loaded = host._warm(fake_engine(calls), SPEC, host_state=host_state)
  assert loaded.host_state is host_state
  assert calls == ([{"device_state": False}] if host_state else [{}])


def test_resident_engine_reloads_only_for_a_different_mode_and_is_never_ready_while_replaced(monkeypatch):
  host = make_host()
  host.loaded = host._warm(fake_engine([]), SPEC)
  started = []
  def start(job, req, entry, model_path, spec):
    started.append(req)
    host.job = job
  monkeypatch.setattr(host, "_start", start)
  monkeypatch.setattr(host, "_spec_on_disk", lambda entry, model_path, req: SPEC)
  monkeypatch.setattr(host.cache, "entry", lambda sha: NS(exists=True), raising=False)
  monkeypatch.setattr(host.cache, "model_path", lambda sha: Path("/nonexistent"), raising=False)
  same = host.request(S.Request(SPEC.sha256, 0, SPEC.frame_skip), None)
  assert same["state"] == "ready" and not started
  flip = host.request(S.Request(SPEC.sha256, 0, SPEC.frame_skip, host_state=True), None)
  assert [r.host_state for r in started] == [True]
  status = host.status(SPEC.sha256, SPEC.frame_skip)
  assert status["state"] == "building" and flip["state"] != "ready"


def test_engine_req_and_hello_carry_the_choice():
  sent = []
  class Stop(Exception):
    pass
  client = JetlinkClient(NS(send_json=lambda kind, seq, d: sent.append(d)))
  client._expect = lambda *a, **k: (_ for _ in ()).throw(Stop())
  for value in (False, True):
    client.host_state = value
    with pytest.raises(Stop):
      client.ensure_engine(SPEC.sha256, 0, frame_skip=SPEC.frame_skip)
  assert sent[0] == {"sha256": SPEC.sha256, "nbytes": 0, "frame_skip": SPEC.frame_skip}   # 0 = old payload
  assert sent[1]["host_state"] is True
  session = S.Session.__new__(S.Session)
  session.request = None
  session.on_engine_req.__func__  # exists
  replies = []
  session._send_json = lambda kind, seq, d: replies.append(d)
  session.host = NS(request=lambda req, sess: {"req": req})
  for payload in sent:
    session.on_engine_req(NS(payload=memoryview(json.dumps(payload).encode()), seq=1))
  assert [r["req"].host_state for r in replies] == [False, True]
  assert S.HOST_STATE_CAPABILITY == H.CAPABILITY


@pytest.mark.parametrize("dtype", [np.float16, np.float32])
def test_explicit_host_choice_overrides_gpu_default_and_matches_gpu_values(runtime, monkeypatch, dtype):  # noqa: F811
  from test_device_state import make_engine
  results = []
  for explicit_host in (False, True):
    e, q, host = make_engine(runtime, True, monkeypatch, dtype)   # env = 1 (GPU default)
    if explicit_host:
      e.close()
      e, q, host = make_engine(runtime, False, monkeypatch, dtype)
      monkeypatch.setenv("CARROT_TRT_DEVICE_STATE", "1")
      e.device_state = None
      e.configure_state_feedback.__func__  # same method, explicit False below
      e2 = runtime.engine.TrtEngine.configure_state_feedback
      e._state_host_sources = {}
      e2(e, q.spec.state_pairs, device_state=False)
      assert e.device_state is None
    values = []
    for i in range(12):
      if i == 7:
        q.reset()
      q.step_into(np.full((2, 2), i + 1, np.uint8), np.array([i / 4, 0], np.float32), host)
      out = e.run()
      q.feedback(out)
      values.append(out["outputs"].copy())
    e.close()
    results.append(values)
  np.testing.assert_array_equal(results[0], results[1])


class Params(dict):
  def get_int(self, key):
    value = self[key]
    if isinstance(value, Exception):
      raise value
    return int(value)
  def get_bool(self, key):
    return bool(self.get(key, False))
  def put_int(self, key, value):
    self[key] = int(value)


def onroad(menu, committed):
  return Params({H.PARAM: menu, H.COMMITTED: committed, "IsOffroad": False, "IsOnroad": True})


def test_offroad_commits_at_once_and_onroad_only_when_stopped_and_inactive():
  p = Params({H.PARAM: 1, H.COMMITTED: 0, "IsOffroad": True, "IsOnroad": False})
  assert H.HostStateLatch(p, safe=lambda: False).resolve() is True and p[H.COMMITTED] == 1
  p = onroad(1, 0)
  assert H.HostStateLatch(p, safe=lambda: False).resolve() is False and p[H.COMMITTED] == 0
  assert H.HostStateLatch(p, safe=lambda: True).resolve() is True and p[H.COMMITTED] == 1
  p = onroad(RuntimeError("read"), 1)
  assert H.HostStateLatch(p, safe=lambda: True).resolve() is True   # read error keeps the committed value


def test_change_while_driving_then_link_drop_then_reconnect_keeps_committed_value():
  p = onroad(0, 0)
  moving = {"safe": False}
  peer = {H.CAPABILITY: True}
  client = NS(host_state=None)
  latch = H.HostStateLatch(p, safe=lambda: moving["safe"], clock=iter(range(0, 1000, 2)).__next__)
  assert H.apply(client, peer, latch) is False
  p[H.PARAM] = 1                       # menu changed while driving
  assert latch.poll(client.host_state) is False
  # link drops while driving, new session / new latch: still the committed 0
  client2 = NS(host_state=None)
  assert H.apply(client2, peer, H.HostStateLatch(p, safe=lambda: moving["safe"])) is False
  moving["safe"] = True                # stopped, assistance off
  assert latch.poll(False) is True and p[H.COMMITTED] == 1
  moving["safe"] = False               # driving again, reconnect
  client3 = NS(host_state=None)
  assert H.apply(client3, peer, H.HostStateLatch(p, safe=lambda: False)) is True
  p[H.PARAM] = 0                       # back to 0 while driving: no switch until stopped
  assert H.apply(NS(host_state=None), peer, H.HostStateLatch(p, safe=lambda: False)) is True


def test_old_bundle_is_never_asked():
  p = Params({H.PARAM: 1, H.COMMITTED: 1, "IsOffroad": True, "IsOnroad": False})
  client = NS(host_state=None)
  assert H.apply(client, {}, H.HostStateLatch(p)) is False and client.host_state is False


def test_stopped_inactive_check_rejects_stale_or_moving():
  def sm(valid=True, alive=True, standstill=True, v=0.0, enabled=False, active=False):
    msgs = {"carState": NS(standstill=standstill, vEgo=v), "carControl": NS(enabled=enabled),
            "selfdriveState": NS(active=active)}
    return NS(update=lambda t: None, valid={k: valid for k in msgs}, alive={k: alive for k in msgs},
              __getitem__=None, **{"get": msgs.__getitem__}), msgs
  class SM:
    def __init__(self, **kw):
      self.kw = kw
      self.msgs = {"carState": NS(standstill=kw.get("standstill", True), vEgo=kw.get("v", 0.0)),
                   "carControl": NS(enabled=kw.get("enabled", False)),
                   "selfdriveState": NS(active=kw.get("active", False))}
      self.valid = {k: kw.get("valid", True) for k in self.msgs}
      self.alive = {k: kw.get("alive", True) for k in self.msgs}
    def update(self, timeout):
      pass
    def __getitem__(self, key):
      return self.msgs[key]
  assert H.StoppedInactive(SM())() is True
  for bad in ({"valid": False}, {"alive": False}, {"standstill": False}, {"v": 0.5}, {"enabled": True},
              {"active": True}):
    assert H.StoppedInactive(SM(**bad))() is False


def test_daemon_checks_in_both_loops_and_reconnects_from_the_frame_loop():
  daemon = ROOT / "openpilot/selfdrive/modeld/jetlink/daemon.py"
  tree = ast.parse(daemon.read_text())
  serve = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "_serve_local")
  calls = [n for n in ast.walk(serve) if isinstance(n, ast.Call) and ast.unparse(n.func) == "host_state_changed"]
  assert len(calls) == 2
  inner = [n for n in ast.walk(serve) if isinstance(n, ast.While)][1]
  assert any(isinstance(n, ast.Raise) and "K9JetHostState" in ast.unparse(n) for n in ast.walk(inner))
  text = daemon.read_text()
  assert "jet_host_state.apply(client, peer" in text and text.index("jet_host_state.apply(") < text.index("prepare(client, peer")


def test_menu_is_one_experimental_0_1_default_0():
  settings = json.loads((ROOT / "openpilot/selfdrive/carrot_settings.json").read_text())
  items = [i for i in settings["params"] if i.get("name") == "K9JetHostState"] if "params" in settings else None
  text = (ROOT / "openpilot/selfdrive/carrot_settings.json").read_text()
  assert text.count("\"name\": \"K9JetHostState\"") == 1
  assert "K9JetHostStateSet" not in text
  keys = (ROOT / "openpilot/common/params_keys.h").read_text()
  assert "{\"K9JetHostState\", {PERSISTENT, INT, \"0\"}}" in keys
