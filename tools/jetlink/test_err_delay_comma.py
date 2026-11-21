"""K9JetErrDelay comma side (대감님 2026-11-21 11:09, 노비 11:13): the hold rules (output_recovery), jetlinkd's decision
and header-only reply, modeld's handling (nothing published for the held frame, the next valid frame at once, the
setting switched on / off within 0.5 s without a restart, its own bound), and the HUD data path. No vehicle, no
Jetson: fakes and the real classes."""
import json
import sys
from types import SimpleNamespace as NS

import numpy as np
import pytest

from openpilot.selfdrive.modeld.jetlink import link, model, output_recovery as R
from openpilot.selfdrive.modeld.jetlink.output_recovery import JetlinkOutputRecovery
from jetlink.client import HeldOutput, NonFiniteOutput


class Drive:
  """20 Hz frames on a fake clock"""
  def __init__(self):
    self.t, self.frame = 100.0, 0
    self.r = JetlinkOutputRecovery(clock=lambda: self.t)
    self.r.connected()

  def step(self, dt=.05):
    self.frame = (self.frame + 1) & 0xFFFFFFFF
    self.t += dt

  def good(self, n=1):
    for _ in range(n):
      self.step()
      self.r.ok(self.frame)

  def bad(self, enabled=True, restored=True, dt=.05):
    self.step(dt)
    return self.r.fault(self.frame, enabled, restored)


def test_one_status_4_is_held():
  d = Drive()
  d.good(5)
  assert d.bad() and d.r.held == 1 and d.r.last_reason == 'held'


def test_two_in_a_row_and_alternating_fall_back():
  d = Drive()
  d.good(5)
  assert d.bad() and not d.bad()
  assert d.r.last_reason == '2 in a row'
  d = Drive()
  d.good(5)
  assert d.bad()
  d.good()
  assert not d.bad()                          # broken, valid, broken
  assert d.r.last_reason.startswith('second status 4 2 frames')


@pytest.mark.parametrize('valid, held', [(19, False), (20, True), (21, True), (40, True)])
def test_window_is_20_frames(valid, held):
  d = Drive()
  d.good(5)
  assert d.bad()
  d.good(valid)
  assert d.bad() is held, d.r.last_reason     # 19 valid then broken = frame 20 after it -> caught; 21 -> held


@pytest.mark.parametrize('case', ['stale', 'first', 'off', 'not_restored'])
def test_the_existing_path_otherwise(case):
  d = Drive()
  d.good(5)
  if case == 'stale':
    assert not d.bad(dt=.15) and 'ms old' in d.r.last_reason          # previous result > 100 ms old
  elif case == 'first':
    d.r.connected()
    assert not d.bad() and d.r.last_reason == 'no valid result yet on this connection'
  elif case == 'off':
    assert not d.bad(enabled=False) and d.r.last_reason == 'setting off'
  else:
    assert not d.bad(restored=False) and 'did not restore' in d.r.last_reason
  assert len(d.r.faults) == 1                                          # every status 4 is recorded


@pytest.mark.parametrize('switch', ['before', 'after', 'off_fault'])
def test_history_survives_setting_changes(switch):
  """노비 11:13: held once, setting 1 -> 0 -> 1 right before / after the held frame, a second status 4 within 1 s
  still falls back; a status 4 while the setting was 0 counts too."""
  d = Drive()
  d.good(5)
  if switch == 'off_fault':
    assert not d.bad(enabled=False)        # setting 0: falls back now, and is recorded
    d.r.connected()                        # comma rejoins 5 s later on a new connection ...
    d.t += 0.3                             # (made short here so only the history decides)
    d.good(3)
    assert not d.bad() and 'second status 4' in d.r.last_reason
    return
  if switch == 'before':
    d.good(1)                              # the setting went 1 -> 0 -> 1 on these frames: jetlinkd keeps one object
  assert d.bad()
  if switch == 'after':
    d.good(1)
  d.good(4)
  assert not d.bad() and 'second status 4' in d.r.last_reason


def test_reconnect_keeps_the_history_one_second_rule_crosses_connections():
  d = Drive()
  d.good(5)
  assert d.bad()
  d.r.connected()
  d.good(3)
  assert not d.bad()                       # 0.2 s later on a new connection: still the 1 s rule
  d = Drive()
  d.good(5)
  assert not d.bad(enabled=False)          # fell back (setting off) ...
  d.r.connected()
  d.t += 9.0                               # ... rejoined 9 s later; modeld's frame ids did not advance meanwhile
  d.good(3)
  assert d.bad(), d.r.last_reason          # 4 frame ids later but another connection, 9 s later: held


def test_frame_id_wraps():
  d = Drive()
  d.frame = 0xFFFFFFF0
  d.good(5)
  assert d.bad()
  d.good(5)
  assert not d.bad()                       # 6 frames across the wrap


# ---- jetlinkd: the decision and the reply ----------------------------------------------------------------------------

def serve(monkeypatch, script, offered=True, recovery=None):
  """run daemon._serve_local over scripted (flags, result) requests; result = 'ok' | 'held' | 'nan'."""
  if sys.platform == 'win32':
    monkeypatch.setitem(sys.modules, 'fcntl', NS())
  from openpilot.selfdrive.modeld.jetlink import daemon
  from openpilot.common import runtime_diagnostics
  monkeypatch.setattr(runtime_diagnostics.RuntimeDiagnostics, '_schedstat', lambda _: ())
  monkeypatch.setitem(sys.modules, 'openpilot.common.params', NS(Params=lambda: NS(put_bool_nonblocking=lambda *a: None)))
  monkeypatch.setitem(sys.modules, 'openpilot.common.swaglog', NS(cloudlog=NS(event=lambda *a, **kw: None)))
  attached = iter([True] * (len(script) + 1) + [False, False])
  monkeypatch.setattr(daemon, 'host_attached', lambda: next(attached))
  monkeypatch.setattr(daemon, 'update_affinity', lambda: None)
  monkeypatch.setattr(daemon, 'publish', lambda *a, **kw: None)
  recovery = recovery if recovery is not None else JetlinkOutputRecovery()
  monkeypatch.setattr(R, '_PROCESS', recovery)
  hellos, replies, begun = [], [], []
  monkeypatch.setattr(daemon, 'send', lambda conn, data: hellos.append(json.loads(data)))
  requests = iter([daemon.REQUEST.pack(i + 1, f, 0) + bytes(link.SPEC.warped_nbytes) + bytes(link.SPEC.packed_nbytes)
                   for i, (f, _) in enumerate(script)])
  monkeypatch.setattr(daemon, 'PacketReader', lambda _: NS(receive=lambda c: next(requests)))
  monkeypatch.setattr(daemon, 'send_parts', lambda conn, header, *rest: replies.append((daemon.REPLY.unpack(header), len(rest))))
  results = iter([r for _, r in script])
  output = np.zeros(link.SPEC.output_nelem, np.float32)

  def begin(images, packed, frame, reset, want_state, **options):
    begun.append((frame, reset, options))
    return frame

  def end(seq):
    kind = next(results)
    if kind == 'held':
      raise HeldOutput(f'inference failed: NOT_FINITE (frame {seq}); recurrent state held')
    if kind == 'nan':
      client.dead = True
      raise NonFiniteOutput(f'inference failed: NOT_FINITE (frame {seq})')
    return output

  class Connection:
    def __enter__(self): return self
    def __exit__(self, *a): pass
    def settimeout(self, t): pass

  client = NS(spec=link.SPEC, output_projection_available=False, deferred_map_available=False, last_state=None,
              last_timings=(20000, 1000, 22000), infer_begin=begin, infer_end=end, dead=False,
              t=NS(last_receive={}), hold_state_available=offered)
  connection = Connection()
  error = None
  try:
    daemon._serve_local(NS(accept=lambda: (connection, None)), client, {'carrot_host': 'jetson'}, None,
                        NS(sent=lambda sof: None), None)
  except Exception as exc:  # what main() sees
    error = exc
  return NS(hellos=hellos, replies=replies, begun=begun, error=error, recovery=recovery, client=client)


H = R.IPC_FLAG


def test_jetlinkd_holds_one_and_replies_header_only(monkeypatch):
  s = serve(monkeypatch, [(1 | H, 'ok'), (H, 'ok'), (H, 'held'), (H, 'ok')])
  assert s.error is None and s.hellos[0][R.IPC_CAPABILITY] is True
  assert [b[2] for b in s.begun] == [{'hold': True}] * 4 and [b[1] for b in s.begun] == [True, False, False, False]
  assert s.replies == [((1, 20000, 1000, 22000), 1), ((2, 20000, 1000, 22000), 1), ((3, 20000, 1000, 22000), 0),
                       ((4, 20000, 1000, 22000), 1)]
  assert s.recovery.held == 1 and len(s.recovery.faults) == 1 and not s.client.dead


@pytest.mark.parametrize('script, why', [
  ([(1 | H, 'ok'), (H, 'held'), (H, 'held')], 'in a row'),
  ([(1 | H, 'ok'), (H, 'held'), (H, 'ok'), (H, 'held')], 'second status 4'),
  ([(1 | H, 'held')], 'no valid result'),
  ([(1, 'ok'), (0, 'nan')], 'setting off'),
])
def test_jetlinkd_otherwise_raises_the_existing_error(monkeypatch, script, why):
  s = serve(monkeypatch, script)
  assert isinstance(s.error, NonFiniteOutput) and not isinstance(s.error, HeldOutput)
  assert str(s.error) == f'inference failed: NOT_FINITE (frame {len(script)})'   # the same text as before
  assert s.client.dead and why in s.recovery.last_reason
  assert [r[0][0] for r in s.replies] == list(range(1, len(script)))             # no reply for the failed frame


@pytest.mark.parametrize('script', [
  # setting 1 -> 0 -> 1 on the two frames right before the held one, then a second status 4 within 20 frames
  [(1 | H, 'ok'), (0, 'ok'), (H, 'ok'), (H, 'held'), (H, 'ok'), (H, 'held')],
  # 1 -> 0 -> 1 on the two frames right after the held one
  [(1 | H, 'ok'), (H, 'ok'), (H, 'held'), (0, 'ok'), (H, 'ok'), (H, 'held')],
  # the first status 4 came while the setting was 0 (existing failure) and still counts for the next 1 s
  [(1, 'ok'), (0, 'nan')],
], ids=['toggle_before', 'toggle_after', 'off_first'])
def test_jetlinkd_history_survives_setting_toggles(monkeypatch, script):
  recovery = JetlinkOutputRecovery()
  s = serve(monkeypatch, script, recovery=recovery)
  assert isinstance(s.error, NonFiniteOutput) and not isinstance(s.error, HeldOutput)
  assert str(s.error) == f'inference failed: NOT_FINITE (frame {len(script)})'
  if len(script) > 2:
    assert recovery.held == 1 and 'second status 4' in recovery.last_reason   # the toggles cleared nothing
    assert [b[2] for b in s.begun] == [{'hold': True} if f & H else {} for f, _ in script]
  else:
    assert recovery.last_reason == 'setting off' and len(recovery.faults) == 1
    s2 = serve(monkeypatch, [(1 | H, 'ok'), (H, 'held')], recovery=recovery)   # rejoin on a new connection, same jetlinkd
    assert isinstance(s2.error, NonFiniteOutput) and 'second status 4' in recovery.last_reason


def test_old_jetson_never_offered_and_flag_rejected(monkeypatch):
  s = serve(monkeypatch, [(1, 'ok'), (0, 'ok')], offered=False)
  assert R.IPC_CAPABILITY not in s.hellos[0] and [b[2] for b in s.begun] == [{}, {}]
  s = serve(monkeypatch, [(1 | H, 'ok')], offered=False)
  assert s.begun == [] and s.error is None        # 'invalid reset flag': local client ended, nothing sent to the Jetson


# ---- modeld: JoiningModel ----------------------------------------------------------------------------------------------

def joining(monkeypatch, tmp_path, script, setting):
  from test_hotplug_resources import joining_model
  m = joining_model(monkeypatch)
  clock = [500.0]
  monkeypatch.setattr(model.time, 'monotonic', lambda: clock[0])
  status = tmp_path / 'status.json'
  monkeypatch.setattr(model, 'MODEL_STATUS', status)
  monkeypatch.setattr(model, 'FAULT', tmp_path / 'fault')
  m.ready = m.join_allowed = True
  m.preparation = None
  calls = []
  outputs = iter(script)

  def infer(images, packed, frame, reset, source_sof=0, **kw):
    calls.append((frame, kw.get('hold', False), m.views['desire'].copy()))
    clock[0] += .04
    kind = next(outputs)
    return None if kind == 'held' else np.full(link.SPEC.output_nelem, float(frame), np.float32)

  m.client = NS(spec=link.SPEC, image_buffer=lambda: None, infer=infer, close=lambda: calls.append('closed'),
                err_delay=True, shared=None, project_output=False)
  m.active, m.reset = True, False
  m._err_params = NS(get_int=lambda key: setting[0])
  m.parser = NS(parse_outputs=lambda result: float(result[0]))
  return m, calls, clock, status


def run_frames(m, clock, n, desire=None):
  out = []
  for i in range(n):
    clock[0] += .01
    inputs = {'desire_pulse': np.eye(8, dtype=np.float32)[(desire or [0] * n)[i]],
              'traffic_convention': np.array([1, 0], np.float32), 'action_t': np.array([.2, .3], np.float32)}
    out.append(m.run({}, {}, inputs, False))
  return out


def test_modeld_publishes_nothing_for_the_held_frame_and_the_next_at_once(monkeypatch, tmp_path):
  m, calls, clock, status = joining(monkeypatch, tmp_path, ['ok', 'ok', 'held', 'ok', 'ok'], [1])
  out = run_frames(m, clock, 5, desire=[0, 0, 3, 3, 3])
  assert out[:2] == [1.0, 2.0] and out[2] is None and out[3:] == [4.0, 5.0]   # the frame after: new value, no hold
  assert all(c[1] for c in calls if c != 'closed') and 'closed' not in calls
  assert m.client is not None and m.active and not (tmp_path / 'fault').exists()
  # the held frame's desire edge (frame 3 starts desire 3) is not used up: frame 4 sends the pulse again
  assert calls[2][2][3] == 1.0 and calls[3][2][3] == 1.0 and calls[4][2][3] == 0.0
  record = json.loads(status.read_text())
  assert record['err_delay_count'] == 1 and record['err_delay_at'] == pytest.approx(500.0 + 3 * .05 - .0, abs=.02)
  assert m.err_delay_count == 1


def test_modeld_own_bound_falls_back_like_before(monkeypatch, tmp_path):
  m, calls, clock, status = joining(monkeypatch, tmp_path, ['ok', 'held', 'ok', 'held'], [1])
  m.small.reset = lambda: calls.append('small reset')
  out = run_frames(m, clock, 4)
  assert out[:3] == [1.0, None, 3.0] and out[3] == 'local'                  # second held within 20 frames: fallback
  assert calls[-2:] == ['closed', 'small reset'] and (tmp_path / 'fault').exists()   # FAULT -> commIssue as before
  assert m.client is None and not m.active and m.next_join > clock[0]


def test_setting_switches_within_half_a_second_without_restart(monkeypatch, tmp_path):
  setting = [0]
  m, calls, clock, _ = joining(monkeypatch, tmp_path, ['ok'] * 40, setting)
  run_frames(m, clock, 10)
  assert not any(c[1] for c in calls)
  setting[0] = 1
  t_on = clock[0]
  run_frames(m, clock, 15)
  first = next(i for i, c in enumerate(calls) if c[1])
  assert first >= 10 and (first - 10) * .05 <= R.PARAM_READ_SECONDS + .05     # on: at the next 0.5 s read
  setting[0] = 0
  n = len(calls)
  run_frames(m, clock, 15)
  last = max(i for i, c in enumerate(calls) if c[1])
  assert (last - n + 1) * .05 <= R.PARAM_READ_SECONDS + .05 and not calls[-1][1]
  assert t_on > 0 and m.client is not None


def test_no_hold_without_a_capable_jetson_and_no_param_read(monkeypatch, tmp_path):
  m, calls, clock, _ = joining(monkeypatch, tmp_path, ['ok'] * 3, [1])
  m.client.err_delay = False
  m._err_params = NS(get_int=lambda key: pytest.fail('read without a capable Jetson'))
  run_frames(m, clock, 3)
  assert not any(c[1] for c in calls)


def test_unreadable_setting_is_off(monkeypatch, tmp_path):
  m, calls, clock, _ = joining(monkeypatch, tmp_path, ['ok'] * 3, [1])
  def broken(key):
    raise KeyError(key)
  m._err_params = NS(get_int=broken)
  run_frames(m, clock, 3)
  assert not any(c[1] for c in calls)


# ---- IPC client ------------------------------------------------------------------------------------------------------

def test_ipc_client_header_only_reply_is_a_held_frame():
  c = object.__new__(link.Client)
  c.timeout, c.shared, c.project_output, c.deferred_nav, c.err_delay = .15, None, False, False, True
  c.spec = link.SPEC
  sent = []
  replies = [link.REPLY.pack(7, 1, 2, 3)]
  c.sock = NS(settimeout=lambda t: None)
  c.reader = NS(receive=lambda sock, deadline: memoryview(replies.pop(0)))
  link_send = link.send_parts
  try:
    link.send_parts = lambda sock, *parts, deadline=None: sent.append(parts[0][:link.REQUEST.size])
    image = np.zeros(link.SPEC.warped_shape, np.uint8)
    packed = np.zeros(link.SPEC.packed_nelem, np.float32)
    assert c._infer(image, packed, 7, hold=True) is None
    assert link.REQUEST.unpack(bytes(sent[0]))[1] & R.IPC_FLAG
    replies.append(link.REPLY.pack(8, 1, 2, 3))
    with pytest.raises(ValueError, match='invalid inference reply size'):
      c._infer(image, packed, 8)                       # no hold asked: a header alone is still an error
  finally:
    link.send_parts = link_send


# ---- HUD data path -----------------------------------------------------------------------------------------------------

def test_status_file_to_comma_panel_and_snapshot_to_jetson_panel(monkeypatch, tmp_path):
  from openpilot.common import jetlink_status
  path = tmp_path / 'model.json'
  monkeypatch.setattr(jetlink_status, 'MODEL_STATUS', path)
  path.write_text(json.dumps({'updated': 10.0, 'err_delay_count': 2, 'err_delay_at': 9.7}))
  assert jetlink_status.err_delay(10.2) == (2, pytest.approx(.5))
  path.write_text(json.dumps({'updated': 10.0, 'err_delay_count': 0, 'err_delay_at': None}))
  assert jetlink_status.err_delay(10.2) is None
  path.write_text(json.dumps({'updated': 1.0, 'err_delay_count': 2, 'err_delay_at': 0.9}))
  assert jetlink_status.err_delay(10.2) is None          # stale status: nothing
  sys.path.insert(0, str(model.Path(model.__file__).resolve().parents[4] / 'tools/jetlink'))
  import hud
  import hud_protocol
  path.write_text(json.dumps({'updated': 10.0, 'err_delay_count': 3, 'err_delay_at': 9.9}))
  monkeypatch.setattr(jetlink_status.time, 'monotonic', lambda: 10.1)
  assert hud_protocol.jetlink_err_delay() == [3, pytest.approx(.2)]
  monkeypatch.setattr(hud, 'read_snapshot', lambda: (50.0, {'jetlink_err_delay': [3, .2]}))
  monkeypatch.setattr(hud.time, 'monotonic', lambda: 50.3)
  assert hud.DisplayParams.jetlink_err_delay(None) == (3, pytest.approx(.5))   # + the snapshot's age on the Jetson
  monkeypatch.setattr(hud, 'read_snapshot', lambda: (50.0, {'jetlink_err_delay': None}))
  assert hud.DisplayParams.jetlink_err_delay(None) is None


def test_badge_latch_at_least_one_second_and_while_holds_repeat():
  sys.path.insert(0, str(model.Path(model.__file__).resolve().parents[2] / 'carrot/cluster'))
  from cluster_live import JetErrDelayLatch, JET_ERR_DELAY_SHOW_S, jetlink_err_delay
  latch = JetErrDelayLatch()
  assert not latch.update(0.0, (5, 300.0))               # panel start: an old hold is not shown
  assert latch.update(1.0, (6, 0.08))                    # a new one: from now ...
  assert latch.update(1.0 + JET_ERR_DELAY_SHOW_S - .01, (6, 1.07))
  assert not latch.update(1.0 + JET_ERR_DELAY_SHOW_S + .01, (6, 1.09))   # ... for 1 s
  assert latch.update(3.0, (7, .05)) and latch.update(3.8, (8, .03))   # repeats extend it
  assert latch.update(4.7, (8, .93)) and not latch.update(4.81, (8, 1.04))
  assert not latch.update(9.0, (9, 2.5))                 # a missed one (seen 2.5 s late) is not shown
  assert jetlink_err_delay(NS(jetlink_err_delay=lambda: (2, .1))) == (2, .1)
  assert jetlink_err_delay(NS(jetlink_err_delay=lambda: None)) is None
  assert jetlink_err_delay(NS(jetlink_err_delay=lambda: (0, .1))) is None


def test_setting_registered_last_in_experimental_with_texts():
  from pathlib import Path
  root = Path(model.__file__).resolve().parents[4]
  data = json.loads((root / 'openpilot/selfdrive/carrot_settings.json').read_text())
  exp = []
  def walk(node):
    if isinstance(node, dict):
      if node.get('id') == 'EXPERIMENTAL':
        exp.append(node)
      for g in node.get('groups', []) or []:
        walk(g)
  for m in data['menu']:
    walk(m)
  assert len(exp) == 1 and exp[0]['params'][-1] == 'K9JetErrDelay'
  p = next(p for p in data['params'] if p['name'] == 'K9JetErrDelay')
  assert (p['min'], p['max'], p['default'], p['control']) == (0, 1, 0, 'select')
  assert '젯슨 에러지연' in p['title'] and p['etitle'] and p['ctitle']
  for phrase in ('젯슨 4번(NaN)이 한 장 나면 직전 결과로 버팀', '오류를 막는 기능이 아니라', '2장 연속이거나 1초 안에 두 번 나면 지금처럼 끊김',
                 '켜고 끄면 바로 적용'):
    assert phrase in p['descr'], phrase
  assert 'Not a fix' in p['edescr'] and 'applies at once' in p['edescr']
  keys = (root / 'openpilot/common/params_keys.h').read_text()
  assert '{"K9JetErrDelay", {PERSISTENT, INT, "0"}}' in keys
