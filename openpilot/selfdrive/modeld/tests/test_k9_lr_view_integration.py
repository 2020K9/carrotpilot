"""K9LRView integration checks that the closed-loop / replay / MetaDrive harnesses do not cover.

lrview_r14: r9..r13 shipped `sm[carControl]` (undefined name) in modeld main(); every harness called
k9_lr_view.py directly, so nothing ran the modeld loop line and modeld would have died with NameError
on its first model output (K9LRView 0 included). These tests close that gap:
  (a) static check: no undefined names in modeld.py, k9_lr_view.py and the K9LRView tests (symtable, no tools needed);
  (b) smoke: the real modeld.main() loop runs with fake camera / model / SubMaster / Params and drives the K9LRView
      part (K9OneLaneHold read every 2nd frame, K9LRView re-read every 100th, apply(engaged=carControl.latActive)).
"""
import ast
import builtins
import symtable
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[4]
MODELD = ROOT / "openpilot/selfdrive/modeld/modeld.py"
STATIC_FILES = [
  "openpilot/selfdrive/modeld/modeld.py",
  "openpilot/selfdrive/modeld/k9_lr_view.py",
  "openpilot/selfdrive/modeld/tests/test_k9_lr_view.py",
  "openpilot/selfdrive/modeld/tests/test_k9_lr_view_integration.py",
  "openpilot/selfdrive/controls/tests/test_k9_lane_fix_lr_view.py",
]
MODULE_NAMES = {"__file__", "__name__", "__doc__", "__spec__", "__loader__", "__package__", "__builtins__",
                "__path__", "__annotations__", "__dict__", "__cached__"}
BAD_LINE = "engaged=bool(sm[carControl].latActive)"
GOOD_LINE = "engaged=bool(sm['carControl'].latActive)"


# ---------------------------------------------------------------- (a) static: undefined names (like ruff/pyflakes F821)
def undefined_names(source, filename="<src>"):
  """[(scope, name)] for every name that is read somewhere but bound nowhere it could resolve to
  (module scope, a `global` assignment in a function, or builtins)."""
  if any(isinstance(n, ast.ImportFrom) and any(a.name == "*" for a in n.names) for n in ast.walk(ast.parse(source))):
    raise AssertionError(f"{filename}: star import, names cannot be checked")
  top = symtable.symtable(source, filename, "exec")
  bound = set(MODULE_NAMES) | set(dir(builtins))

  def collect(t):
    for s in t.get_symbols():
      if t is top and (s.is_assigned() or s.is_imported() or s.is_namespace()):
        bound.add(s.get_name())
      elif s.is_declared_global() and s.is_assigned():
        bound.add(s.get_name())
    for c in t.get_children():
      collect(c)
  collect(top)

  bad = []
  def walk(t):
    for s in t.get_symbols():
      if s.is_referenced() and s.is_global() and s.get_name() not in bound:
        bad.append((t.get_name(), s.get_name()))
    for c in t.get_children():
      walk(c)
  walk(top)
  return sorted(set(bad))


def test_static_check_finds_the_r13_modeld_name_error():
  # the checker itself: the r9..r13 line must be caught, quoted keys and ordinary code must not
  src = MODELD.read_text(encoding="utf-8")
  assert GOOD_LINE in src and BAD_LINE not in src
  assert ("main", "carControl") in undefined_names(src.replace(GOOD_LINE, BAD_LINE), "modeld_r13.py")
  assert undefined_names("import os\nX = 1\ndef f(a):\n  global Y\n  Y = a\n  return [os.sep, X, len(a), __file__, b]\n") == [("f", "b")]
  with pytest.raises(AssertionError):
    undefined_names("from os import *\nx = sep\n")
  ok = ("def f():\n  return Y\ndef g():\n  global Y\n  Y = 2\nclass C:\n  z = 1\n  def m(self):\n" +
        "    return super().m(), [i for i in range(3)], (lambda q: q)(1)\n")
  assert undefined_names(ok) == []


@pytest.mark.parametrize("rel", STATIC_FILES)
def test_no_undefined_names(rel):
  path = ROOT / rel
  assert undefined_names(path.read_text(encoding="utf-8"), str(path)) == []


def test_modeld_apply_line_uses_the_subscribed_carcontrol():
  src = MODELD.read_text(encoding="utf-8")
  loop = src[src.index("def main("):]
  assert '"carControl"' in loop[:loop.index("k9_lr_view = K9LRView()")]   # subscribed by the SubMaster in main()
  assert loop.count("k9_lr_view.apply(") == 1
  assert "model_output = k9_lr_view.apply(model_output, " + GOOD_LINE + ")" in loop


# ---------------------------------------------------------------- (b) smoke: the real modeld main() loop
class _Stop(Exception):
  pass


class _FakeParams:
  def __init__(self, store):
    self.store = store

  def _v(self, key, default):
    v = self.store.get(key, default)
    if isinstance(v, BaseException):
      raise v
    return v

  def get_int(self, key, block=False):
    return int(self._v(key, 0))

  def get_float(self, key, block=False):
    return float(self._v(key, 0.0))

  def get_bool(self, key, block=False):
    return bool(self._v(key, False))

  def get(self, key, block=False, return_default=False):
    return self._v(key, None)

  def __getattr__(self, name):              # put / put_bool / put_int(_nonblocking) ...
    if name.startswith("put"):
      return lambda key, value: self.store.__setitem__(key, value)
    raise AttributeError(name)


def _run_modeld(monkeypatch, n_frames, params_at, lat_active, real_model_buf=None):
  """Run modeld.main(demo=True) for n_frames camera frames. params_at(frame) -> dict of param changes applied
  right after the loop's own reads of that frame; lat_active(frame) -> carControl.latActive for that frame.
  real_model_buf: None = fake model (random parsed outputs); else the real ModelState runs on this camera buffer.
  Returns [(frame, engaged, one_lane_hold, mode, same_object, lane_lines_y)] per apply() call and the modelV2 sent."""
  from openpilot.selfdrive.modeld import modeld as MD
  from openpilot.selfdrive.modeld import k9_lr_view as LV
  from openpilot.selfdrive.controls.lib import desire_helper
  from openpilot.selfdrive.modeld.parse_model_outputs import Parser
  from openpilot.selfdrive.modeld.tests.test_k9_lr_view import OUTPUT_SLICES

  store = {"K9LRView": 1, "K9OneLaneHold": 0, "VEgoStopping": 50}
  store.update(params_at(0))
  fake_params = _FakeParams(store)
  monkeypatch.setattr(MD, "Params", lambda *a, **k: fake_params)
  monkeypatch.setattr(desire_helper, "Params", lambda *a, **k: fake_params)
  for name in ("usbgpu_present", "wait_for_usbgpu_present"):
    monkeypatch.setattr(MD, name, lambda *a, **k: False)
  monkeypatch.setattr(MD, "usbgpu_compiled_path", lambda *a, **k: None)
  monkeypatch.setattr(MD, "config_realtime_process", lambda *a, **k: None)

  class FakeVipc:                            # the real receive_camera_pair() reads recv() / frame_id / timestamps
    width, height, buffer_len = 1928, 1208, 4
    frame_id = timestamp_sof = timestamp_eof = 0
    def __init__(self, *a, **k):
      pass
    @staticmethod
    def available_streams(name, block=False):
      return {MD.VisionStreamType.VISION_STREAM_ROAD}
    def connect(self, block):
      return True
    def recv(self):
      f = state["frame"] = state["frame"] + 1
      if f > n_frames:
        raise _Stop
      store.update(params_at(f))       # after this frame's param reads: takes effect at the next read
      sm_box["sm"].set_lat_active(lat_active(f))
      self.frame_id, self.timestamp_sof, self.timestamp_eof = f, f * 50_000_000, f * 50_000_000 + 10_000_000
      return object() if real_model_buf is None else real_model_buf
  monkeypatch.setattr(MD, "VisionIpcClient", FakeVipc)

  state = {"frame": 0}
  rng = np.random.default_rng(14)
  runs = []

  class FakeModel:
    vision_input_names = ["img", "big_img"]
    usbgpu = False
    def __init__(self, *a, **k):
      pass
    def run(self, bufs, transforms, inputs, prepare_only):
      raw = rng.normal(0, 1, 2576).astype(np.float32)
      out = Parser().parse_outputs({k: raw[np.newaxis, v] for k, v in OUTPUT_SLICES.items()})
      runs.append(out)
      return out
  if real_model_buf is None:
    monkeypatch.setattr(MD, "ModelState", FakeModel)


  sm_box = {}
  class FakeSM:
    def __init__(self, services, *a, **k):
      self.data = {s: getattr(MD.messaging.new_message(s), s) for s in services}
      self.updated = dict.fromkeys(services, False)
      self.seen = dict.fromkeys(services, False)
      sm_box["sm"] = self
    def set_lat_active(self, on):
      self.data["carControl"].latActive = bool(on)
    def update(self, timeout=0):
      pass
    def __getitem__(self, s):
      return self.data[s]
  monkeypatch.setattr(MD, "SubMaster", FakeSM)

  sent = []
  class FakePM:
    def __init__(self, *a, **k):
      pass
    def send(self, name, msg):
      if name == "modelV2":
        sent.append((state["frame"], [list(l.y) for l in msg.modelV2.laneLines]))
  monkeypatch.setattr(MD, "PubMaster", FakePM)

  calls = []
  class SpyLRView(LV.K9LRView):
    def apply(self, outputs, engaged=True):
      new = super().apply(outputs, engaged=engaged)
      calls.append((state["frame"], engaged, self.one_lane_hold, self.mode, new is outputs,
                    np.array(new["lane_lines"][0, :, :, 0], copy=True)))
      return new
  monkeypatch.setattr(MD, "K9LRView", SpyLRView)

  with pytest.raises(_Stop):
    MD.main(demo=True)
  return calls, sent


def test_modeld_main_loop_runs_k9_lr_view(monkeypatch):
  # K9LRView 1; K9OneLaneHold 0 -> 1 at frame 31, read error at 61, 1 at 71, 2 (not 1 = off) at 85;
  # K9LRView 1 -> 0 at frame 50 (main() re-reads it at frame 100); latActive on for frames 11..60 and 81..
  def params_at(f):
    return {31: {"K9OneLaneHold": 1}, 61: {"K9OneLaneHold": ValueError("bad")}, 71: {"K9OneLaneHold": 1},
            85: {"K9OneLaneHold": 2}, 50: {"K9LRView": 0}}.get(f, {})
  def lat(f):
    return 11 <= f <= 60 or f >= 81
  calls, sent = _run_modeld(monkeypatch, 104, params_at, lat)

  assert [c[0] for c in calls] == list(range(1, 105))          # apply() ran once per model output, no crash
  assert [f for f, _ in sent] == list(range(1, 105))           # and every modelV2 was published
  for f, engaged, olh, mode, same, _ in calls:
    assert engaged is lat(f)                                    # engaged = this frame's carControl.latActive
    # K9OneLaneHold read at start and on even frames (0.1 s), a read error = off
    assert olh is (32 <= f <= 61 or 72 <= f <= 85), f
    assert mode == (1 if f < 100 else 0), f                     # K9LRView re-read every 100 frames
    assert same is (mode == 0)                                  # 0: the model output object passes through untouched
  # with K9LRView 1 the published near lines are the tracker's, with 0 the raw model's
  for (_, _, _, _, _, y), (_, sent_y) in zip(calls, sent, strict=True):
    np.testing.assert_allclose(np.asarray(sent_y, dtype=np.float32), y.astype(np.float32), rtol=0, atol=0)


def test_modeld_main_loop_off_publishes_the_raw_model(monkeypatch):
  calls, sent = _run_modeld(monkeypatch, 6, lambda f: {"K9LRView": 0} if f == 0 else {}, lambda f: f % 2 == 0)
  assert len(calls) == 6 and all(c[3] == 0 and c[4] for c in calls)
  assert [c[1] for c in calls] == [False, True, False, True, False, True]


# ---------------------------------------------------------------- (c) paths the harnesses never take
def test_apply_exception_path_returns_raw_and_recovers():
  # a malformed output dict inside apply(): the raw object comes back, the tracker restarts, the next frame works
  from openpilot.selfdrive.modeld.k9_lr_view import K9LRView
  from openpilot.selfdrive.modeld.tests.test_k9_lr_view import parsed_outputs
  view = K9LRView()
  view.set_mode(1)
  for k in range(3):
    view.apply(parsed_outputs(k), engaged=True)
  assert view.tracker.prob is not None
  for broken in ({k: v for k, v in parsed_outputs(7).items() if k != "plan"},
                 dict(parsed_outputs(8), lane_lines=np.zeros((1, 3, 33, 2), dtype=np.float32))):
    assert view.apply(broken, engaged=True) is broken
    assert view.tracker.prob is None                                    # tracker restarted
    view.apply(parsed_outputs(10), engaged=True)
  out = view.apply(parsed_outputs(9), engaged=False)
  assert np.all(np.isfinite(out["lane_lines"])) and out is not None


def test_reads_with_the_real_params(tmp_path):
  # modeld reads K9OneLaneHold every 2nd frame and K9LRView every 100th through the C++ Params
  from openpilot.common.params import Params
  from openpilot.selfdrive.modeld.k9_lr_view import read_mode, read_one_lane_hold
  from openpilot.selfdrive.controls.lib.k9_one_lane_hold import k9_one_lane_hold_mode
  p = Params(str(tmp_path))
  assert read_one_lane_hold(p) is False and read_mode(p) == 0           # nothing stored = off
  for v in (0, 1, 2, -1):
    p.put_int("K9OneLaneHold", v)
    assert read_one_lane_hold(p) is (v == 1)
    assert read_one_lane_hold(p) is (k9_one_lane_hold_mode(p) == 1)      # same reading as controlsd
  for v, expect in ((0, 0), (1, 1)):
    p.put_int("K9LRView", v)
    assert read_mode(p) == expect
