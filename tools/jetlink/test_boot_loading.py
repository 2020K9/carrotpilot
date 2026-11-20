"""Loading % on the Jetson waiting screen (boot_display.py, 대감님 11/20 10:38·11:21·23:12).

The stage cell stays as it is; one % line right below it, same column, the screen's own accent colour, touching
nothing. % = real stages done / 6 (server marks from the server health file + the hand-over to the HUD), no timer.
No / broken / stale health file: the screen is drawn exactly as before and there is no % line."""
import json
from pathlib import Path
import subprocess
import sys

import pytest

sys.path.insert(0, str(Path(__file__).parent))
import boot_display
import boot_status
from boot_display import LOADING_STAGES, loading, render

HERE = Path(__file__).parent
ALL5 = dict.fromkeys(boot_status.STARTUP_MARKS, True)
NORMAL, ERROR = '#65d7bb', '#ff8876'
UNITS_OK = {'ssh': {'ActiveState': 'active'}, 'carrot-jetlink': {'ActiveState': 'active'}}
SCREENS = {   # every board of the waiting screen, as lines() makes them
  'waiting_normal': ({'stage': {'stage': 'protected'}, 'storage': {'state': 'protected'}, 'units': UNITS_OK,
                      'addresses': ['192.168.43.20'], 'wifi': 'connected', 'uptime': 38, 'temp_c': 46.5}, NORMAL),
  'booting_early': ({'uptime': 9}, NORMAL),
  'boot_error': ({'stage': {'stage': 'protected'}, 'storage': {'state': 'protected'},
                  'units': {**UNITS_OK, 'carrot-jetlink-wifi': {'ActiveState': 'failed'}},
                  'addresses': ['192.168.43.20'], 'wifi': 'waiting', 'uptime': 38, 'temp_c': 46.5}, ERROR),
  'boot_error_storage': ({'stage': {'stage': 'data-check', 'error': 'RuntimeError'}, 'storage': {'state': 'base-recovery'},
                          'units': {'carrot-protected-storage': {'ActiveState': 'failed'}}, 'uptime': 25}, ERROR),
  'super_fw_install': ({'stage': {'stage': 'protected'}, 'storage': {'state': 'protected'}, 'units': UNITS_OK,
                        'firmware': {'state': 'staging'}, 'addresses': ['192.168.43.20'], 'wifi': 'connected',
                        'uptime': 38, 'temp_c': 46.5}, NORMAL),
  'super_setup_error': ({'stage': {'stage': 'protected'}, 'storage': {'state': 'protected'}, 'units': UNITS_OK,
                         'firmware': {'state': 'power_check', 'power': {'state': 'Super power mode not set'}},
                         'addresses': ['192.168.43.20'], 'wifi': 'connected', 'uptime': 38, 'temp_c': 46.5}, ERROR),
}


def marks(done):
  """the first `done` loading stages finished, in code order (the 6th is the hand-over to the HUD)"""
  names = [name for name, _ in LOADING_STAGES]
  return {'jetson': {name: name in names[:done] for name in boot_status.STARTUP_MARKS}, 'hud': done >= 6}


def drawn(status, monkeypatch):
  """(image, [(text, bbox, fill, size, xy)]) of one real render"""
  from PIL import ImageDraw
  calls = []
  original = ImageDraw.ImageDraw.text

  def text(self, xy, txt, fill=None, font=None, *args, **kwargs):
    calls.append((txt, self.textbbox(xy, txt, font=font), fill, font.size, tuple(xy)))
    return original(self, xy, txt, fill, font, *args, **kwargs)
  with monkeypatch.context() as m:
    m.setattr(ImageDraw.ImageDraw, 'text', text)
    image = render(status)
  return image, calls


def test_each_real_stage_adds_one_sixth_in_order():
  assert [name for name, _ in LOADING_STAGES] == [*boot_status.STARTUP_MARKS, 'hud']
  assert [loading(marks(done)) for done in range(7)] == [0, 16, 33, 50, 66, 83, 100]
  # counted by stage, never by time: the same stages give the same % whenever they happened
  assert loading({'jetson': {**dict.fromkeys(boot_status.STARTUP_MARKS, False), 'hello': True}}) == 16


def test_hand_over_to_the_hud_is_the_last_stage_and_100_percent():
  assert loading({'jetson': ALL5}) == 83   # every server stage done, the HUD has not asked yet
  assert loading({'jetson': ALL5, 'hud': True}) == 100
  assert loading({'jetson': ALL5, 'hud': 1}) == 83   # only a real hand-over flag counts


def test_no_or_unreadable_health_means_no_percent():
  for status in ({}, {'jetson': None}, {'jetson': []}, {'jetson': 'x'}, {'jetson': None, 'hud': True}):
    assert loading(status) is None


def test_bad_values_are_not_done(tmp_path):
  path = tmp_path / 'health.json'
  path.write_text(json.dumps({'updated': 100., 'hud_diag': {'startup': {
    'server_start': 1., 'hello': float('nan'), 'engine_req': True, 'engine_ready': '2', 'infer': -1.}}}))
  value = boot_status.startup_marks(path, now=101.)
  assert value == {'server_start': True, 'hello': False, 'engine_req': False, 'engine_ready': False, 'infer': False}
  assert loading({'jetson': value}) == 16


@pytest.mark.parametrize('content', [None, '', '{', 'not json', '[]', '"x"', '{"updated": 1}',
                                     '{"updated": "1", "hud_diag": {"startup": {}}}',
                                     '{"updated": 100, "hud_diag": []}', '{"updated": 100, "hud_diag": {"startup": 3}}',
                                     '{"updated": true, "hud_diag": {"startup": {}}}', '[' * 100000 + ']' * 100000])
def test_missing_or_broken_health_file_gives_no_marks(tmp_path, content):
  path = tmp_path / 'health.json'
  if content is not None:
    path.write_text(content)
  assert boot_status.startup_marks(path, now=101.) is None


def test_stale_health_file_gives_no_marks(tmp_path):
  # a dead server's old file must not keep its stages on the screen (same 3 s freshness as the HUD)
  path = tmp_path / 'health.json'
  path.write_text(json.dumps({'updated': 100., 'hud_diag': {'startup': {'server_start': 1., 'hello': 2.}}}))
  assert boot_status.startup_marks(path, now=102.9) == {**dict.fromkeys(boot_status.STARTUP_MARKS, False),
                                                         'server_start': True, 'hello': True}
  assert boot_status.startup_marks(path, now=103.) is None
  assert boot_status.startup_marks(path, now=99.) is None   # from the future (another boot) = not current


def test_health_file_is_the_servers(monkeypatch):
  import host_health
  assert boot_status.HOST_HEALTH == host_health.STATUS
  h = host_health.HostHealth.__new__(host_health.HostHealth)
  h.marks, h.inference = {'server_start': 0.}, None
  for name in ('hello', 'engine_req', 'engine_ready'):
    h.mark(name)
  h.note_inference(1000, 2000)
  startup = h.diagnostics(1.)['startup']
  assert set(startup) == set(boot_status.STARTUP_MARKS)


def test_loading_stage_order_follows_the_code():
  # Jetson server: start (health file) -> HELLO_REQ -> ENGINE_REQ -> ENGINE_RESP ready -> first OK INFER_RESP
  server = (HERE / 'server.py').read_text()
  host = (HERE / 'host_health.py').read_text()
  assert "self.marks = {'server_start': time.monotonic()}" in host and "self.mark('infer')" in host
  hello, req = server.index("health.mark('hello')"), server.index("health.mark('engine_req')")
  assert server.rindex('P.Msg.HELLO_REQ', 0, hello) > server.rindex('def handle', 0, hello)
  assert server.rindex('P.Msg.ENGINE_REQ', 0, req) > hello
  assert "obj.get('state') == 'ready'" in server[server.index("mark('engine_ready')") - 200:server.index("mark('engine_ready')")]
  # comma daemon: greet -> prepare (ENGINE_REQ/ready) -> 10 warm-up inferences -> serve_local (HUD data from here)
  daemon = (HERE.parents[1] / 'openpilot/selfdrive/modeld/jetlink/daemon.py').read_text()
  main = daemon[daemon.index('def main'):]
  order = [main.index(s) for s in ('peer = greet(client', 'prepare(client, peer', 'client.infer(np.zeros', 'serve_local(listener')]
  assert order == sorted(order)
  serve = daemon[daemon.index('def serve_local'):daemon.index('def _serve_local')]
  assert 'display_publisher(client, peer, Publisher)' in serve and 'publish_hud(client, publisher)' in daemon
  # Jetson HUD: waits for comma HUD data with IsOnroad, then asks for the panel = the hand-over
  hud = (HERE / 'hud.py').read_text()
  body = hud[hud.index('def main'):]
  assert body.index("settings.get_bool('IsOnroad')") < body.index('with panel_owner(hud=True)')
  display = (HERE / 'boot_display.py').read_text()
  assert 'while not requested():' in display and "'hud': True" in display


@pytest.mark.parametrize('done', range(7))
@pytest.mark.parametrize('name', list(SCREENS))
def test_percent_line_right_below_stage_cell_and_touches_nothing(monkeypatch, name, done):
  status, accent = SCREENS[name]
  image, calls = drawn({**status, **marks(done)}, monkeypatch)
  pct = [c for c in calls if c[0].startswith('로딩 ')]
  assert [c[0] for c in pct] == [f'로딩 {loading(marks(done))}%']
  _text, box, fill, _size, xy = pct[0]
  _title, rows, _detail, error = boot_display.lines(status)
  assert fill == accent == (ERROR if error else NORMAL)   # the board's own accent colour (red boards: red)
  cell_value = [c for c in calls if c[4] == (665, 177)]   # stage cell (Super 펌웨어 on the firmware boards) value
  below = [c for c in calls if c[4] == (665, 238)]        # the next row's label in the same column
  assert [c[0] for c in cell_value] == [rows[1][2]] and [c[0] for c in below] == [rows[4][0]]
  assert xy[0] == cell_value[0][4][0] == 665     # same column as the stage cell
  assert box[1] - cell_value[0][1][3] >= 4      # right below its value, not touching it
  assert below[0][1][1] - box[3] >= 4           # and clear of the next row's label
  for other in calls:
    if other is not pct[0]:
      o = other[1]
      assert not (box[0] - 2 < o[2] and o[0] < box[2] + 2 and box[1] - 2 < o[3] and o[1] < box[3] + 2), other[0]
  # pixels: only the % line changed; nothing else on the screen moved, and its surroundings were empty
  import numpy as np
  plain = np.asarray(render(status)).astype(int)
  now = np.asarray(image).astype(int)
  ys, xs = np.nonzero(np.abs(now - plain).sum(axis=-1))
  assert ys.size and box[0] - 1 <= xs.min() and xs.max() <= box[2] + 1 and box[1] - 1 <= ys.min() and ys.max() <= box[3] + 1
  around = plain[box[1] - 3:box[3] + 4, box[0] - 3:box[2] + 4]
  assert (around == np.array([0x10, 0x18, 0x21])).all()


@pytest.mark.parametrize('name', list(SCREENS))
def test_without_health_file_the_screen_is_exactly_as_before(monkeypatch, name):
  import numpy as np
  status, _accent = SCREENS[name]
  for value in (None, [], 'broken'):
    image, calls = drawn({**status, 'jetson': value, 'hud': True}, monkeypatch)
    assert not [c for c in calls if '%' in c[0] or c[0].startswith('로딩')]
    assert np.array_equal(np.asarray(image), np.asarray(render(status)))


def test_collect_survives_missing_and_broken_health_file(tmp_path, monkeypatch):
  monkeypatch.setattr(boot_status.subprocess, 'run', lambda *a, **k: subprocess.CompletedProcess(a, 0, stdout='[]', stderr=''))
  path = tmp_path / 'health.json'
  monkeypatch.setattr(boot_status, 'HOST_HEALTH', path)
  monkeypatch.setattr(boot_status.startup_marks, '__defaults__', (path, None))
  status = boot_status.collect()
  assert status['jetson'] is None and render(status).size == (1920, 462)
  path.write_text('{"updated": ')
  assert boot_status.collect()['jetson'] is None
  path.write_text(json.dumps({'updated': boot_status.time.monotonic(), 'hud_diag': {'startup': {'server_start': 1., 'hello': 2.}}}))
  status = boot_status.collect()
  assert loading(status) == 33 and render(status).size == (1920, 462)


class FakePanel:
  landscape_width, landscape_height = 1920, 462

  def __init__(self):
    self.frames = []

  def send_jpeg(self, data):
    assert data[:2] == b'\xff\xd8'
    self.frames.append(data)


def test_hand_over_frame_is_100_percent_and_stays(monkeypatch):
  seen = []
  monkeypatch.setattr(boot_display, 'render', lambda status, **k: seen.append(status) or render(status, **k))
  answers = iter([False, False, True])
  reads = []

  def read():
    reads.append(1)
    return {**SCREENS['waiting_normal'][0], 'jetson': {**ALL5, 'infer': len(reads) > 1}}
  panel = FakePanel()
  boot_display.show_until_hud(panel, read=read, requested=lambda: next(answers), marks=lambda: dict(ALL5),
                              sleep=lambda s: None)
  assert [loading(s) for s in seen] == [66, 83, 100]   # last = the HUD asked for the panel
  assert len(reads) == 2 and len(panel.frames) == 3    # the hand-over frame waits for no new systemctl read
  assert seen[-1]['hud'] is True and 'hud' not in seen[0]


def test_hand_over_without_a_shown_frame_sends_nothing(monkeypatch):
  panel = FakePanel()
  boot_display.show_until_hud(panel, read=lambda: pytest.fail('no read'), requested=lambda: True,
                              marks=lambda: pytest.fail('no marks'), sleep=lambda s: None)
  assert panel.frames == []


def test_hand_over_without_health_file_shows_no_percent(monkeypatch):
  seen = []
  monkeypatch.setattr(boot_display, 'render', lambda status, **k: seen.append(status) or render(status, **k))
  answers = iter([False, True])
  boot_display.show_until_hud(FakePanel(), read=lambda: dict(SCREENS['boot_error'][0], jetson=None),
                              requested=lambda: next(answers), marks=lambda: None, sleep=lambda s: None)
  assert [loading(s) for s in seen] == [None, None]


def test_percent_line_has_no_stage_words(monkeypatch):
  _image, calls = drawn({**SCREENS['waiting_normal'][0], **marks(2)}, monkeypatch)
  assert [c[0] for c in calls if '%' in c[0]] == ['로딩 33%']
