"""Remaining time next to the connect % (대감님 2026-11-21 16:07): Jetson waiting screen (boot_display.remaining,
drawn after the 로딩 N% text on its line) and comma badge (jetlink_status.progress / progress_text). Both are
ESTIMATES from stage times measured in the comma logs of the car (see the constants); never shown without fresh
status, gone once the HUD / Jetson model has taken over."""
import json
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import boot_display
import boot_status
from boot_display import STAGE_SECONDS, remaining, render
from test_boot_loading import SCREENS, drawn, marks

from openpilot.common import jetlink_status as js

NAMES = [name for name, _ in boot_display.LOADING_STAGES]


def timed(done, spent=0.0):
  """first `done` stages finished at t=100 + 1 s each; now = last mark + spent"""
  status = marks(done)
  at = {name: 100.0 + i for i, name in enumerate(NAMES[:max(done, 1)]) if name != "hud"}
  at["now"] = max(at.values()) + spent
  return {**status, "jetson_at": at}


def test_remaining_is_the_measured_stages_ahead():
  assert remaining(timed(1)) == round(sum(STAGE_SECONDS.values()))          # server up: all of hello .. hud ahead
  assert remaining(timed(1, 4.0)) == round(sum(STAGE_SECONDS.values()) - 4.0)   # 4 s already spent in hello
  assert remaining(timed(4)) == round(STAGE_SECONDS["infer"] + STAGE_SECONDS["hud"])
  assert remaining(timed(5)) == round(STAGE_SECONDS["hud"])
  assert remaining(timed(5, 60.0)) == 1                                        # late: never 0 / negative


def test_no_time_without_fresh_marks_or_after_hand_over():
  assert remaining(marks(3)) is None                     # old status without times
  assert remaining({**timed(3), "jetson": None}) is None
  assert remaining({**timed(5), "hud": True}) is None    # HUD has the panel: 100 %, no time
  assert remaining({"jetson_at": {"now": 1.0}, "jetson": {}}) is None


def test_startup_times_same_file_and_rules_as_the_marks(tmp_path):
  path = tmp_path / "h.json"
  path.write_text(json.dumps({"updated": 50.0, "hud_diag": {"startup": {"server_start": 10.0, "hello": 17.0,
                                                                      "engine_req": "x"}}}))
  assert boot_status.startup_times(path, now=51.0) == {"now": 51.0, "server_start": 10.0, "hello": 17.0}
  assert boot_status.startup_times(path, now=60.0) is None   # stale like startup_marks (3 s)
  path.write_text("{")
  assert boot_status.startup_times(path, now=51.0) is None


@pytest.mark.parametrize("done", range(1, 6))
@pytest.mark.parametrize("name", list(SCREENS))
def test_time_drawn_after_the_percent_on_its_line_touching_nothing(monkeypatch, name, done):
  status, accent = SCREENS[name]
  st = {**status, **timed(done, 0.5)}
  image, calls = drawn(st, monkeypatch)
  pct = [c for c in calls if c[0].startswith("로딩 ")]
  eta = [c for c in calls if "남은 시간" in c[0]]
  assert len(pct) == 1 and [c[0] for c in eta] == [f" · 남은 시간 약 {remaining(st)}초 (추정)"]
  p, e = pct[0], eta[0]
  assert e[2] == accent and e[3] == p[3] and e[4][1] == p[4][1]     # same colour, size and line
  assert e[1][0] >= p[1][2] - 1                                       # right after the % text
  assert e[1][2] <= 665 + 600                                         # inside the stage column (next column 1290)
  for other in calls:
    if other is not e and other is not p:
      o, b = other[1], e[1]
      assert not (b[0] - 2 < o[2] and o[0] < b[2] + 2 and b[1] - 2 < o[3] and o[1] < b[3] + 2), other[0]


def test_hand_over_frame_has_no_time(monkeypatch):
  status, _ = SCREENS["waiting_normal"]
  _image, calls = drawn({**status, **timed(5), "hud": True}, monkeypatch)
  assert [c[0] for c in calls if "%" in c[0]] == ["로딩 100%"] and not [c for c in calls if "남은" in c[0]]


def test_collect_carries_the_times_only_from_a_fresh_file(tmp_path, monkeypatch):
  monkeypatch.setattr(boot_status, "HOST_HEALTH", tmp_path / "missing.json")
  monkeypatch.setattr(boot_status.subprocess, "run", lambda *a, **k: (_ for _ in ()).throw(OSError()))
  assert boot_status.collect()["jetson_at"] is None
  assert render(boot_status.collect()).size == (1920, 462)


# ---- comma badge ---------------------------------------------------------------------------------------------

def link(state, stage=None):
  return {"state": state, "preparation": {"stage": stage} if stage else {}}


@pytest.fixture(autouse=True)
def fresh_tracker():
  js._TRACK.clear()
  js._PROGRESS[:] = [None, -1.0]
  yield
  js._TRACK.clear()
  js._PROGRESS[:] = [None, -1.0]


def walk(steps):
  """[(t, link, model)] -> [progress] in order"""
  return [js.progress(t, l, m) for t, l, m in steps]


def test_cold_start_like_the_log_counts_down_and_rises():
  # 11/18 15:12 cold start, times from the log: connecting 0, waiting-host 9.5, handoff 22.4, loading 31.3, ready 31.7
  steps = [(0.0, link("connecting", "usb-setup"), {}), (9.5, link("connecting", "waiting-host"), {}),
           (22.4, link("connecting", "runtime-handoff"), {}), (25.0, link("connecting", "network-setup"), {}),
           (31.3, link("loading", "prepare"), {}), (31.7, link("ready"), {})]
  got = walk(steps)
  assert all(g is not None for g in got)
  pcts = [g[0] for g in got]
  assert pcts == sorted(pcts) and pcts[-1] <= 99
  assert got[1][1] == round(js.BOOT_WAIT_S - 9.5 + js.HANDOFF_TO_LOAD_S + js.LOAD_TO_READY_S + js.READY_TO_FIRST_S + .5)
  assert got[-1][1] == 2                  # ready: ~1.05 s to the first Jetson result, shown rounded up
  assert js.progress(33.0, link("ready"), {"active": True}) is None and js._TRACK == {}


def test_reconnect_with_the_jetson_up_is_short():
  got = js.progress(0.0, link("connecting", "network-setup"), {})
  assert got == (0, int(-(-(js.RECONNECT_TO_LOAD_S + js.LOAD_TO_READY_S + js.READY_TO_FIRST_S) // 1)))
  late = js.progress(30.0, link("connecting", "network-setup"), {})
  assert late[1] == 2 and late[0] < 100   # over the measured time: small, never 0, never 100 % before the model runs


def test_retry_after_the_model_step_puts_it_ahead_again():
  js.progress(0.0, link("loading", "prepare"), {})
  js.progress(0.3, link("ready"), {})
  again = js.progress(3.0, link("connecting", "usb-setup"), {})
  assert "ready" not in js._TRACK and again[1] >= 5


@pytest.mark.parametrize("state", ["waiting", "updating", "stopped", "blocked"])
def test_nothing_while_not_connecting_and_the_wait_restarts(state):
  js.progress(0.0, link("connecting", "usb-setup"), {})
  assert js.progress(5.0, link(state), {}) is None and js._TRACK == {}


def test_held_preparation_and_unknown_states_show_nothing():
  assert js.progress(0.0, link("loading", "waiting"), {}) is None
  assert js.progress(0.0, link("retrying"), {}) is None
  assert js.progress(0.0, {}, {}) is None


def test_badge_text_suffix_and_no_files_means_no_suffix(monkeypatch, tmp_path):
  monkeypatch.setattr(js, "LINK_STATUS", tmp_path / "none.json")
  monkeypatch.setattr(js, "MODEL_STATUS", tmp_path / "none2.json")
  assert js.progress_text(10.0) == ""
  monkeypatch.setattr(js, "progress", lambda now: (45, 8))
  assert js.progress_text(10.2) == ""                       # cached for half a second
  assert js.progress_text(10.6) == " 45% · 약 8초"


def test_both_comma_renderers_add_it_to_the_jetson_loading_badge_only():
  root = Path(__file__).resolve().parents[2]
  for rel in ("openpilot/selfdrive/ui/onroad/hud_renderer.py", "openpilot/selfdrive/ui/mici/onroad/hud_renderer.py"):
    src = (root / rel).read_text()
    i = src.index("text, state = ui_state.jetlink_badge")
    assert "text += progress_text()" in src[i:i + 400] and chr(34) + "loading" + chr(34) in src[i:i + 400]
