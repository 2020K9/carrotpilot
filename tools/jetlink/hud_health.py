"""Jetson-local temperature display; never substitute it for vehicle statistics."""
import base64
import json
import math
from pathlib import Path
import re
import time

from host_health import STATUS
from hud_protocol import SNAPSHOT_STATE
from cluster_config import CLUSTER_SCREEN_MODE_NAVI
from openpilot.common.jetlink_status import _fresh
from cluster_models import GitBranchStatus
import cluster_version_badge as VB
from cluster_renderer import (ClusterUiRenderer, DESIGN_WIDTH, DESIGN_HEIGHT,
                              EGPU_STATUS_CENTER_X, TOP_STATUS_CENTER_Y, rl, rl_color)

NORMAL = (100, 225, 170)
WARNING = (255, 205, 70)
ERROR = (255, 90, 90)
UNKNOWN = (190, 190, 190)


def temperature_label(health):
  value = health.get('temp_c')
  if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or not -20 <= value <= 150:
    return '--°C', UNKNOWN
  severity = health.get('thermal_severity', 'unknown')
  color = {'ok': NORMAL, 'warning': WARNING, 'error': ERROR}.get(severity, UNKNOWN)
  return f'{value:.1f}°C', color


def health_banner(health, language):
  thermal = health.get('thermal_severity')
  if thermal in ('warning', 'error'):
    trip = health.get('thermal_trip') or {}
    temp, limit = trip.get('temp_c'), trip.get('limit_c')
    detail = ''
    if all(isinstance(v, (int, float)) and math.isfinite(v) for v in (temp, limit)):
      detail = f" · {temp:.1f}°C / {'기준' if language == 'ko' else 'limit'} {limit:.1f}°C"
    if language == 'ko':
      title = 'Jetson 과열' if thermal == 'error' else 'Jetson 온도 높음'
      message = f'{title}{detail} · 냉각 상태 확인'
    else:
      title = 'Jetson overheating' if thermal == 'error' else 'Jetson temperature high'
      message = f'{title}{detail} · Check cooling'
    return message, ERROR if thermal == 'error' else WARNING
  if health.get('severity', 'unknown') != 'ok':
    reason = health.get('reason') or ('온도·상태 정보 없음' if language == 'ko' else 'Temperature/health unavailable')
    return f'Jetson: {reason}', ERROR if health.get('severity') == 'error' else WARNING
  return '', NORMAL


# The loading % is on the Jetson waiting screen (boot_display.py), not on this driving screen (대감님 11/20 23:12).


def _pct(value):
  return float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and 0 <= value <= 100 else None


def jetson_resources(health):
  """K9HudNavdy 2: Jetson CPU (online core average) / GPU load / memory % from host_performance via host_health;
  None for anything missing or stale, never guessed."""
  perf = health.get('performance') if isinstance(health, dict) else None
  if not isinstance(perf, dict) or perf.get('version') != 1:
    return (None, None, None)
  cores = [_pct(c.get('busy_pct')) for c in perf.get('cpu') or () if isinstance(c, dict) and c.get('online')]
  cores = [c for c in cores if c is not None]
  gpu = None
  for item in (perf.get('gpu') or {}).values():
    if isinstance(item, dict) and _pct(item.get('load_pct')) is not None:
      gpu = _pct(item.get('load_pct'))
      break
  mem = None
  memory = perf.get('memory') or {}
  total, available = memory.get('MemTotal_kb'), memory.get('MemAvailable_kb')
  if isinstance(total, int) and isinstance(available, int) and total > 0 and 0 <= available <= total:
    mem = 100. * (total - available) / total
  return (sum(cores) / len(cores) if cores else None, gpu, mem)


def _num(value, low=None, high=None):
  if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
    return None
  if (low is not None and value < low) or (high is not None and value > high):
    return None
  return float(value)


def jetson_diag(health):
  """K9HudNavdy 2: everything for the Jetson block from the fresh host_health STATUS (Jetson clock only);
  None when the snapshot is missing or stale, None per value when that value is missing."""
  if not isinstance(health, dict) or not health:
    return None
  perf = health.get('performance')
  perf = perf if isinstance(perf, dict) and perf.get('version') == 1 else {}
  cores = []
  for item in perf.get('cpu') or ():
    if isinstance(item, dict):
      online = bool(item.get('online'))
      cores.append((_pct(item.get('busy_pct')) if online else None, _num(item.get('clock_mhz'), 0, 10000) if online else None, online))
  busy = [c[0] for c in cores if c[0] is not None]
  gpu = gpu_mhz = None
  for item in (perf.get('gpu') or {}).values():
    if isinstance(item, dict) and _pct(item.get('load_pct')) is not None:
      gpu, gpu_mhz = _pct(item.get('load_pct')), _num(item.get('clock_mhz'), 0, 10000)
      break
  memory = perf.get('memory') or {}
  total, available = memory.get('MemTotal_kb'), memory.get('MemAvailable_kb')
  mem_used = mem_total = mem_pct = None
  if isinstance(total, int) and isinstance(available, int) and total > 0 and 0 <= available <= total:
    mem_used, mem_total, mem_pct = (total - available) / 1048576, total / 1048576, 100. * (total - available) / total
  temps = []
  for sensor in health.get('sensors') or ():
    if isinstance(sensor, dict) and isinstance(sensor.get('name'), str):
      t = _num(sensor.get('temp_c'), -20, 150)
      if t is not None:
        temps.append((sensor['name'].removesuffix('-thermal').removesuffix('_thermal')[:10], t))
  diag = health.get('hud_diag') if isinstance(health.get('hud_diag'), dict) else {}
  inference = diag.get('inference') if isinstance(diag.get('inference'), dict) else {}
  startup = diag.get('startup') if isinstance(diag.get('startup'), dict) else {}
  return {'cores': cores, 'cpu': sum(busy) / len(busy) if busy else None, 'gpu': gpu, 'gpu_mhz': gpu_mhz,
          'mem_used_gb': mem_used, 'mem_total_gb': mem_total, 'mem_pct': mem_pct, 'temps': temps,
          'infer_gpu_ms': _num(inference.get('gpu_ms'), 0, 10000), 'infer_total_ms': _num(inference.get('total_ms'), 0, 10000),
          'startup': {k: _num(v, 0) for k, v in startup.items() if isinstance(k, str)}}


STRIP_H = 32.   # top strip (stage text / overheating / warnings), design px
STRIP_GAP = 4.  # empty gap between the strip and the smaller screen below, design px


# 11/24 18:29 (대감님 "HUD 왼쪽 맨 아래 하단에 깃 저장소 띄워"): which code is running, always shown small at the
# bottom-left. 11/25 20:57 (대감님 "콤마·젯슨 사용경로 같이 띄우게해 브랜치 뭐쓰는지 모델 뭐쓰는지"): three lines,
#   C <owner/repo> <branch> <commit> · <model> <model sha>   (comma Git* Params + snapshot comma_source/jetson_release)
#   J <owner/repo> <branch> <commit> · <model> <model sha>   (this bundle's SOURCE_ORIGIN/SOURCE_COMMIT + loaded engine)
#   C <install path> · J <install path>
# Anything not known is '?', never guessed.
BUNDLE_ROOT = Path(__file__).resolve().parents[2]
SOURCE_COMMIT_FILE = BUNDLE_ROOT / 'SOURCE_COMMIT'
SOURCE_ORIGIN_FILE = BUNDLE_ROOT / 'SOURCE_ORIGIN'
CATALOG_DIR = BUNDLE_ROOT / 'openpilot/selfdrive/modeld/model_catalog'
REPO_LABEL_SIZE = 15
REPO_LABEL_MARGIN = 4.
REPO_LABEL_LINE_GAP = 2.
REPO_LABEL_COLOR = (170, 178, 186)
REPO_LABEL_MAX_W = 610.
UNKNOWN_TEXT = '?'


def _text(value):
  if isinstance(value, bytes):
    value = value.decode('utf-8', 'replace')
  return value.strip() if isinstance(value, str) else ''


def repo_owner(remote):
  remote = _text(remote)
  m = re.search(r'github\.com[:/]+([^/\s]+)/', remote)
  if m:
    return m.group(1)
  m = re.search(r'([^/:\s]+)/[^/\s]+?(?:\.git)?/?$', remote)
  return m.group(1) if m else ''


def repo_name(remote):
  """'owner/name' of a git remote URL (never any credentials), '' when it is not one."""
  remote = _text(remote)
  m = re.search(r'([^/:@\s]+)/([^/\s]+?)(?:\.git)?/?$', remote)
  return f'{m.group(1)}/{m.group(2)}' if m else ''


def _sha8(value):
  value = _text(value)
  return value[:8] if re.fullmatch(r'[0-9a-fA-F]{7,64}', value) else ''


def side_line(tag, remote, branch, commit, model, model_sha=''):
  parts = [tag, repo_name(remote) or UNKNOWN_TEXT, _text(branch) or UNKNOWN_TEXT, _sha8(commit) or UNKNOWN_TEXT]
  model = ' '.join(x for x in (_text(model), _sha8(model_sha)) if x)
  return ' '.join(parts) + ' · ' + (model or UNKNOWN_TEXT)


def repo_label(remote, branch, commit, jetson_commit):
  """One-line form (11/24): '2020K9/carrot-wip · af8a7e0c', or 'C 2020K9/carrot-wip af8a7e0c / J b14fafe4'."""
  branch, commit, jetson = _text(branch), _text(commit)[:8], _text(jetson_commit)[:8]
  owner = repo_owner(remote)
  name = f'{owner}/{branch}' if owner and branch else (branch or owner)
  if commit and jetson and commit == jetson:
    return f'{name} · {commit}' if name else commit
  if commit:
    comma = ' '.join(x for x in ('C', name, commit) if x)
    return f'{comma} / J {jetson or "-"}'
  if jetson:
    return f'C {name or "-"} / J {jetson}' if name else f'J {jetson}'
  return ''


REPO_KEYS = ('remote', 'branch', 'commit', 'model', 'model_sha')


def repo_lines(comma, jetson):
  """comma/jetson: dicts of remote, branch, commit, model, model_sha, path. [] when no repository, branch or commit is known."""
  if not any(_text(d.get(k)) for d in (comma, jetson) for k in ('remote', 'branch', 'commit')):
    return []
  return [side_line('C', *(comma.get(k) for k in REPO_KEYS)),
          side_line('J', *(jetson.get(k) for k in REPO_KEYS)),
          f"C {_short_path(comma.get('path'))} · J {_short_jetson_path(jetson.get('path'))}"]


def _short_path(value):
  # releases/<40-hex commit> -> releases/<8>, so the line fits; '?' when unknown
  return re.sub(r'\b([0-9a-f]{8})[0-9a-f]{32}\b', r'\1', _text(value)) or UNKNOWN_TEXT


# 11/26 18:44 (대감님 "3.4번째줄 요약좀해서 넣어라 너무길자넠?"): the Jetson always installs under
# /opt/carrot-jetlink, so that part is left out: releases/<commit> -> <commit 8>, other folders by their name
JETSON_ROOT = '/opt/carrot-jetlink/'


def _short_jetson_path(value):
  path = _short_path(value)
  for prefix in (JETSON_ROOT + 'releases/', JETSON_ROOT):
    if path.startswith(prefix) and len(path) > len(prefix):
      return path[len(prefix):]
  return path


# 11/26 00:26 (대감님 "hud에 콤마·젯슨 어디하고 연결돼 있는지"): one more line under the three, read from what is
# installed on this Jetson (never a fixed string):
#   J 받기 <owner/repo of the installed updater's allowed releases> · 키 <signing key> · 모델 <release the model came from>
UPDATER_DIR = Path('/opt/carrot-jetlink/updater')
K9_RELEASE_KEY = '7c687e286025c2bf839062e2cb5c6ee0614d7d4fec656dd81accb7748566cefd'   # = release_parts.PUBLIC_KEY
RELEASE_URL = re.compile(r'github\.com/([^/\s]+/[^/\s]+)/releases/download/([^/\s]+)/')


def updater_link(directory=UPDATER_DIR):
  """{'release': owner/repo the installed updater accepts, 'key': raw Ed25519 public key hex}; '' when unknown."""
  link = {'release': '', 'key': ''}
  try:
    m = re.search(r"RELEASE_HOST, RELEASE_PREFIX = 'github\.com', '/([^/']+/[^/']+)/releases/download/'",
                  (directory / 'update_host.py').read_text())
    link['release'] = m.group(1) if m else ''
  except OSError:
    pass
  try:
    lines = (directory / 'release-signing-public.pem').read_text().splitlines()
    der = base64.b64decode(''.join(x.strip() for x in lines if x.strip() and not x.startswith('-----')), validate=True)
    link['key'] = der[-32:].hex() if len(der) == 44 else ''
  except (OSError, ValueError):
    pass
  return link


def model_release(sha256, roots=(CATALOG_DIR, BUNDLE_ROOT / 'openpilot/selfdrive/modeld/jetlink')):
  """(owner/repo, release tag) of the signed release manifest in this bundle whose model is this ONNX; ('', '')."""
  sha = _text(sha256).lower()
  if not sha:
    return '', ''
  for root in roots:
    try:
      paths = sorted(Path(root).glob('*.json'))
    except OSError:
      continue
    for path in paths:
      try:
        model = json.loads(path.read_text(encoding='utf-8')).get('model')
        if isinstance(model, dict) and str(model.get('sha256', '')).lower() == sha:
          m = RELEASE_URL.search(str(model.get('url', '')))
          if m:
            return m.group(1), m.group(2)
      except (OSError, ValueError, AttributeError):
        continue
  return '', ''


# 11/26 18:44: shorter fourth line. The usual repository name is left out (owner only), and the model release
# names are shortened; any other repository or release is shown in full.
RELEASE_REPO_NAME = 'carrotpilot'
MODEL_TAG_SHORT = (('light-speed-2-', 'ls2-'), ('light-speed-', 'ls-'), ('fairy-tale-', 'ft-'), ('flip-flop-', 'ff-'),
                   ('nav-n16-', 'n16-'), ('md-v3-', 'md3-'))


def _short_repo(repo):
  owner, _, name = repo.partition('/')
  return owner if name == RELEASE_REPO_NAME else repo


def _short_model_tag(tag):
  tag = re.sub(r'^k9-comma4-big-', '', _text(tag))   # shared prefix of every model release, to fit
  for long, short in MODEL_TAG_SHORT:
    if tag.startswith(long):
      return short + tag[len(long):]
  return tag


def link_line(link, model_repo='', model_tag=''):
  release, key = _text(link.get('release')), _text(link.get('key'))
  key_text = ('2020K9' if key == K9_RELEASE_KEY else key[:8]) if key else UNKNOWN_TEXT
  model_repo = _text(model_repo)
  model = ' '.join(x for x in ((_short_repo(model_repo) if model_repo != release else ''), _short_model_tag(model_tag))
                   if x)
  return f'받기 {_short_repo(release) or UNKNOWN_TEXT} · 키 {key_text} · 모델 {model or UNKNOWN_TEXT}'


def read_jetson_commit(path=SOURCE_COMMIT_FILE):
  try:
    return path.read_text().strip()
  except OSError:
    return ''


def read_jetson_origin(path=SOURCE_ORIGIN_FILE):
  """(remote, branch) the bundle was built from; ('', '') for bundles older than this file."""
  try:
    lines = path.read_text().splitlines()
  except OSError:
    return '', ''
  return (lines[0].strip() if lines else ''), (lines[1].strip() if len(lines) > 1 else '')


def catalog_names(sha256, directory=CATALOG_DIR):
  """Catalog names whose Jetson spec is this ONNX ('/'-joined when several share it), '' when unknown."""
  sha = _text(sha256).lower()
  if not sha:
    return ''
  try:
    names = []
    for entry in json.loads((directory / 'catalog.json').read_text(encoding='utf-8')).values():
      spec = json.loads((directory / entry['jetson_spec']).read_text(encoding='utf-8'))
      if str(spec.get('sha256', '')).lower() == sha and entry.get('name') and entry['name'] not in names:
        names.append(entry['name'])
    return '/'.join(names)
  except (OSError, ValueError, KeyError, TypeError, AttributeError):
    return ''


def snapshot_git(record, old):
  """GitRemote/GitBranch/GitCommit from a HUD snapshot; a missing or bad one keeps its last value."""
  values = dict(old)
  params = record[1].get('params') if record and isinstance(record[1], dict) else None
  for key in ('GitRemote', 'GitBranch', 'GitCommit'):
    item = params.get(key) if isinstance(params, dict) else None
    if isinstance(item, str) and len(item) <= 128:
      try:
        values[key] = base64.b64decode(item).decode('utf-8', 'replace')
      except (ValueError, TypeError):
        pass
  return values


def snapshot_versions(record, old):
  """11/26 06:18: the release.json the comma's checkout selects (source_commit + model sha only) and its
  background remote-head check; a missing or bad value keeps its last value."""
  values = dict(old)
  body = record[1] if record and isinstance(record[1], dict) else {}
  release = body.get('jetson_release')
  if isinstance(release, dict) and isinstance(release.get('source_commit'), str):
    model = release.get('model') if isinstance(release.get('model'), dict) else {}
    values['release'] = {'source_commit': release['source_commit'][:64],
                         'model': {'sha256': str(model.get('sha256', ''))[:64]}}
  remote = body.get('comma_remote')
  if isinstance(remote, dict) and remote.get('state') in ('ok', 'unknown', 'behind'):
    values['remote'] = {k: remote[k] for k in ('state', 'sha', 'count') if k in remote and len(str(remote[k])) <= 64}
  return values


def bundle_commit(commit, path=BUNDLE_ROOT):
  """SOURCE_COMMIT, or the 40-hex release folder name when the bundle has none; '' when neither."""
  if _text(commit):
    return _text(commit)
  try:
    name = Path(path).resolve().name
  except OSError:
    return ''
  return name if re.fullmatch(r'[0-9a-f]{40}', name) else ''


def version_lines(comma, jetson, versions):
  """(C4 line, J line) segment tuples for the bottom-left block."""
  c4 = VB.c4_line(comma.get('branch'), comma.get('remote'), comma.get('commit'), versions.get('remote'))
  j = VB.j_line(jetson.get('model'), jetson.get('remote'), jetson.get('commit'), jetson.get('model_sha'),
                comma.get('commit'), versions.get('release'))
  return tuple(c4), tuple(j)


def snapshot_comma_source(record, old):
  """The comma's install path / selected model name (comma_source) and the model of the Jetson release it
  asks for (jetson_release); a missing or bad value keeps its last value."""
  values = dict(old)
  body = record[1] if record and isinstance(record[1], dict) else {}
  source = body.get('comma_source')
  if isinstance(source, dict):
    for key in ('path', 'model'):
      if isinstance(source.get(key), str) and len(source[key]) <= 128:
        values[key] = source[key]
  release = body.get('jetson_release')
  model = release.get('model') if isinstance(release, dict) else None
  if isinstance(model, dict) and isinstance(model.get('sha256'), str):
    values['model_sha'] = model['sha256']
  return values


# 대감님 11/26 22:22: the Jetson map-lost box is a black field with white text naming only what is really lost.
# 05:14 the comma rebooted: snapshots stopped and the map video stopped, but the last navigation state stayed, so
# the screen said "연결됨". Each link is now judged from its own live state, never from the others.
COMMA_SNAPSHOT_TIMEOUT_S = 1.   # same limit as hud.snapshot_status 'ok'
LINK_TEXT = {'ko': {'comma': '콤마 연결 끊김', 'map': '지도 영상 없음', 'navi': '내비 정보 없음'},
             'en': {'comma': 'Comma link lost', 'map': 'No map video', 'navi': 'No navigation data'}}


def link_losses(snapshot_received, navi_ok, dashboard, now):
  """Lost links in fixed order: comma snapshot age, navigation (carrotNavi alive·valid·connected on a fresh
  snapshot), map video (stalled or no current map frame)."""
  lost = []
  comma_ok = snapshot_received is not None and 0 <= now - snapshot_received < COMMA_SNAPSHOT_TIMEOUT_S
  if not comma_ok:
    lost.append('comma')
  map_ok = bool(dashboard is not None and not dashboard.map_stream_stalled
                and any(f.key == 'render:map_main' and f.present for f in dashboard.media))
  if not map_ok:
    lost.append('map')
  if not (comma_ok and navi_ok is True):
    lost.append('navi')
  return lost


class HealthRenderer(ClusterUiRenderer):
  startup_marks = {}   # hud.py MARKS (Jetson monotonic)
  link_state = {}      # hud.py LINK: {'navi': carrotNavi alive·valid·connected in the last snapshot}
  _link_dashboard = None
  _strip_deferred = False
  _strip_inner = None

  def k9_jetson_resources(self):
    return jetson_resources(getattr(self, 'host_health', None) or {})

  def k9_jetson_diag(self):
    return jetson_diag(getattr(self, 'host_health', None) or {})

  def k9_car_rx_age(self, state):
    received = _num(self.startup_marks.get('car_last'), 0)
    return None if received is None else max(0., time.monotonic() - received)

  def k9_startup(self):
    """(seconds after Jetson boot, measured by) per stage; None = not measured. All Jetson CLOCK_MONOTONIC."""
    server = (self.k9_jetson_diag() or {}).get('startup') or {}
    marks = {k: _num(v, 0) for k, v in self.startup_marks.items()}
    return {'connect': (server.get('hello'), '젯슨'), 'model': (server.get('engine_ready'), '젯슨'),
            'car': (marks.get('car'), '젯슨'), 'frame': (marks.get('frame'), '젯슨')}

  def _strip_reached(self, state):
    # Same branches as ClusterUiRenderer.render: the K9 road boards return before _draw_alert_overlay.
    return (self._ev6_park_screen_active(state) or self._ev6_reverse_screen_active(state)
            or not self._k9_turzx_active(state))

  def _strip_outer(self):
    """0 = screen, the capture target, or None when the active target is unknown (then draw as before)."""
    get = getattr(rl, 'rl_get_active_framebuffer', None)
    if get is None:
      return None
    fbo = get()
    if fbo == 0:
      return 0
    cap = getattr(self, '_capture_target', None)
    return cap if cap is not None and cap.id == fbo else None

  _repo_git = {}
  _repo_comma = {}
  _repo_next = 0.
  _repo_jetson = None
  _repo_in_footer = False
  _repo_versions = {}
  _repo_badge_key = None
  _repo_badge = ()
  _repo_park = False

  def repo_lines(self):
    now = time.monotonic()
    if now >= self._repo_next:
      self._repo_next = now + 1.
      if self._repo_jetson is None:
        remote, branch = read_jetson_origin()
        self._repo_jetson = {'remote': remote, 'branch': branch, 'commit': read_jetson_commit(),
                             'path': str(BUNDLE_ROOT)}
        self._repo_link = updater_link()
      try:
        from hud_protocol import read_snapshot
        record = read_snapshot()
        self._repo_git = snapshot_git(record, self._repo_git)
        self._repo_comma = snapshot_comma_source(record, self._repo_comma)
        self._repo_versions = snapshot_versions(record, self._repo_versions)
      except Exception:
        pass
    git = self._repo_git
    health = getattr(self, 'host_health', None)
    diag = health.get('hud_diag') if isinstance(health, dict) else None
    sha = diag.get('model_sha256') if isinstance(diag, dict) else None
    jetson = {**self._repo_jetson, 'model': catalog_names(sha), 'model_sha': _text(sha)}
    comma = {'remote': git.get('GitRemote'), 'branch': git.get('GitBranch'), 'commit': git.get('GitCommit'),
             **self._repo_comma}
    lines = repo_lines(comma, jetson)
    if lines:
      if getattr(self, '_repo_model_sha', None) != jetson['model_sha']:
        self._repo_model_sha = jetson['model_sha']
        self._repo_model_release = model_release(jetson['model_sha'])
      lines.append(link_line(getattr(self, '_repo_link', {}), *self._repo_model_release))
      # 11/26 06:18: the C/J lines become C4/J version lines (owner 일치/불일치, commit, 같음/다름); rebuilt
      # only when an input changes
      key = (tuple(sorted((k, str(v)) for k, v in comma.items())), tuple(sorted((k, str(v)) for k, v in jetson.items())),
             str(sorted(self._repo_versions.items())))
      if key != self._repo_badge_key:
        self._repo_badge_key = key
        self._repo_badge = version_lines(comma, {**jetson, 'commit': bundle_commit(jetson.get('commit'))},
                                         self._repo_versions)
      lines[:2] = self._repo_badge
    # 11/26 18:56 (대감님 "p단화면에만 2줄로 표기해라"): the install-path and connection lines only on the P screen
    if not self._repo_park:
      lines = lines[:2]
    return lines

  def _repo_line_height(self):
    _, h = self._measure_text('0', REPO_LABEL_SIZE, max(1., REPO_LABEL_SIZE * .02))
    return h

  def _draw_repo_lines(self, lines):
    """Bottom line at the very bottom-left, the others stacked above it (design coordinates)."""
    h = self._repo_line_height()
    y = DESIGN_HEIGHT - REPO_LABEL_MARGIN - h * .5
    spacing = max(1., REPO_LABEL_SIZE * .02)
    for text in reversed(lines):
      if isinstance(text, str):
        text = self._ellipsize_text(text, REPO_LABEL_SIZE, REPO_LABEL_MAX_W)
        self._draw_text_with_stroke(text, REPO_LABEL_MARGIN, y, REPO_LABEL_SIZE, REPO_LABEL_COLOR, (5, 9, 12), 2,
                                    cache=True)
      else:   # (text, kind) segments: 불일치/다름 red, 확인 불가 amber, 일치/같음 green
        x = REPO_LABEL_MARGIN
        for part, kind in text:
          room = REPO_LABEL_MARGIN + REPO_LABEL_MAX_W - x
          if room <= 8. or not part:
            break
          # spaces around a segment as a fixed gap (raylib measures them unevenly)
          lead, trail = len(part) - len(part.lstrip()), len(part) - len(part.rstrip())
          x += lead * REPO_LABEL_SIZE * .3
          part = self._ellipsize_text(part.strip(), REPO_LABEL_SIZE, room)
          color = {'ok': NORMAL, 'bad': ERROR, 'unknown': WARNING}.get(kind, REPO_LABEL_COLOR)
          if part:
            self._draw_text_with_stroke(part, x, y, REPO_LABEL_SIZE, color, (5, 9, 12), 2, cache=True)
            x += self._measure_text(part, REPO_LABEL_SIZE, spacing)[0]
          x += trail * REPO_LABEL_SIZE * .3
      y -= h + REPO_LABEL_LINE_GAP
    return len(lines) * h + (len(lines) - 1) * REPO_LABEL_LINE_GAP

  def _draw_git_status(self, status, network_address=None, actual_fps=None):
    # The bottom-left status row is where the running code goes (the Jetson bundle has no .git of its own).
    # The network/FPS row that sits above that row moves up so the lines never cover it.
    lines = self.repo_lines()
    if not lines:
      super()._draw_git_status(status, network_address, actual_fps)
      return
    if len(lines) == 4:
      # driving screen: four lines would reach the speed arc, so the connection line takes the install-path row
      # here (the path is still shown on P/R and K9 board screens)
      lines = lines[:2] + lines[3:]
    self._repo_in_footer = True
    block = self._draw_repo_lines(lines)
    if network_address:
      row_h = max(self._measure_text('0', 20, .4)[1], 14.)   # the base status row (text 20, dot 7)
      # base network row bottom = 12 + row_h + 4 above the bottom edge; keep it 2 px above the top line
      shift = max(0., REPO_LABEL_MARGIN + block + 2. - (12. + row_h + 4.))
      rl.rl_push_matrix()
      rl.rl_translatef(0., -shift, 0.)
      try:
        super()._draw_git_status(None, network_address, actual_fps)
      finally:
        rl.rl_pop_matrix()

  def _draw_repo_label(self):
    """Screens without that status row (K9 boards, P/R screens): the same lines at the very bottom-left."""
    lines = self.repo_lines()
    if not lines:
      return
    rl.rl_push_matrix()
    rl.rl_scalef(self.width / DESIGN_WIDTH, self.height / DESIGN_HEIGHT, 1.)
    try:
      self._draw_repo_lines(lines)
    finally:
      rl.rl_pop_matrix()

  def render(self, state, signal_lights=None):
    self._link_dashboard = getattr(state, 'navi_dashboard', None)
    if self.k9_link_down_screen(state):
      self._draw_link_down_screen()
      return
    self._repo_in_footer = False
    try:
      self._repo_park = bool(self._ev6_park_screen_active(state))
    except Exception:
      self._repo_park = False
    self._render_health(state, signal_lights)
    if not self._repo_in_footer:
      self._draw_repo_label()

  def _render_health(self, state, signal_lights=None):
    self.host_health = _fresh(STATUS, time.monotonic())
    self.host_badge_drawn = False
    # 11/20 12:48: while the top strip is shown, the whole screen is drawn a little smaller
    # below it (y 32~), so the strip covers no text, icon or card. No strip = exactly as before.
    shown = (getattr(self, '_window_open', False) and self._strip_reached(state)
             and self._health_strip_plan(getattr(state, 'alert', None)) is not None)
    outer = self._strip_outer() if shown else None
    if outer is None:
      super().render(state, signal_lights)
      return
    inner = self._strip_inner
    if inner is None or inner.texture.width != self.width or inner.texture.height != self.height:
      if inner is not None:
        rl.unload_render_texture(inner)
      inner = self._strip_inner = rl.load_render_texture(self.width, self.height)
      rl.set_texture_filter(inner.texture, rl.TextureFilter.TEXTURE_FILTER_BILINEAR)
    if outer != 0:
      rl.end_texture_mode()
    rl.begin_texture_mode(inner)
    self._strip_deferred = True
    try:
      rl.clear_background(rl_color(self._current_theme().bg))
      super().render(state, signal_lights)
    finally:
      self._strip_deferred = False
      rl.end_texture_mode()
      if outer != 0:
        rl.begin_texture_mode(outer)
    strip_px = (STRIP_H + STRIP_GAP) * self.height / DESIGN_HEIGHT
    scale = (self.height - strip_px) / self.height
    w, h = self.width * scale, self.height * scale
    rl.clear_background(rl_color(self._current_theme().bg))
    rl.rl_set_blend_factors(rl.RL_ONE, rl.RL_ZERO, rl.RL_FUNC_ADD)
    rl.begin_blend_mode(rl.BlendMode.BLEND_CUSTOM)
    try:
      rl.draw_texture_pro(inner.texture, rl.Rectangle(0, 0, float(self.width), -float(self.height)),
                          rl.Rectangle((self.width - w) / 2, strip_px, w, h), rl.Vector2(0, 0), 0., rl.WHITE)
    finally:
      rl.end_blend_mode()
    rl.rl_push_matrix()
    rl.rl_scalef(self.width / DESIGN_WIDTH, self.height / DESIGN_HEIGHT, 1.)
    try:
      self._draw_health_strip(self._health_strip_plan(None))
    finally:
      rl.rl_pop_matrix()

  def k9_link_losses(self, now=None):
    now = time.monotonic() if now is None else now
    return link_losses(SNAPSHOT_STATE.get('received'), self.link_state.get('navi'), self._link_dashboard, now)

  def k9_link_down_screen(self, state):
    """대감님 11/26 23:12: on the navigation screen a lost map (the old MAP STREAM STALLED case) blanks the whole
    Jetson screen; the left band (speed·limit) and the right band (navigation·연결됨·STREAMS) are hidden too."""
    dashboard = getattr(state, 'navi_dashboard', None)
    if dashboard is None or not dashboard.map_stream_stalled:
      return False
    try:
      if self._ev6_park_screen_active(state) or self._ev6_reverse_screen_active(state):
        return False
      return self._effective_screen_mode(state) == CLUSTER_SCREEN_MODE_NAVI
    except Exception:
      return False

  def _draw_link_down_screen(self):
    rl.draw_rectangle(0, 0, int(self.width), int(self.height), rl_color((0, 0, 0)))
    rl.rl_push_matrix()
    rl.rl_scalef(self.width / DESIGN_WIDTH, self.height / DESIGN_HEIGHT, 1.)
    try:
      self._draw_navi_map_stalled(rl.Rectangle(0., 0., float(DESIGN_WIDTH), float(DESIGN_HEIGHT)), None)
    finally:
      rl.rl_pop_matrix()

  def _draw_navi_map_stalled(self, rect, age_ms):
    rl.draw_rectangle_rec(rect, rl_color((0, 0, 0)))
    texts = LINK_TEXT['ko' if self.language == 'ko' else 'en']
    lines = [texts[k] for k in self.k9_link_losses()] or [texts['map']]
    size, gap = 26., 40.
    top = rect.y + rect.height * .5 - gap * (len(lines) - 1) * .5
    for i, line in enumerate(lines):
      self._draw_text(self._ellipsize_text(line, size, rect.width - 24), rect.x + rect.width * .5, top + gap * i,
                      size, (255, 255, 255), anchor='center')

  def close(self):
    if self._strip_inner is not None:
      rl.unload_render_texture(self._strip_inner)
      self._strip_inner = None
    super().close()

  def _temperature_badge(self, x, y):
    label, color = temperature_label(self.host_health)
    rect = rl.Rectangle(x - 52, y - 17, 104, 60)
    rl.draw_rectangle_rounded(rect, .25, 8, rl_color((0, 0, 0), 190))
    rl.draw_rectangle_rounded_lines_ex(rect, .25, 8, 2., rl_color(color))
    self._draw_text('jetSON', x, y, 19, color, anchor='center')
    self._draw_text(label, x, y + 26, 20, color, anchor='center')
    self.host_badge_drawn = True

  def _draw_egpu_status(self, state):
    # The normal status row's transform also follows left/right panel swaps.
    self._temperature_badge(EGPU_STATUS_CENTER_X + 20, TOP_STATUS_CENTER_Y)

  def _draw_alert_overlay(self, alert, **slot):
    rl.rl_push_matrix()
    rl.rl_scalef(self.width / DESIGN_WIDTH, self.height / DESIGN_HEIGHT, 1.)
    try:
      if not self.host_badge_drawn:
        # Full navigation/graph modes omit the driving status row.
        self._temperature_badge(DESIGN_WIDTH - 66, TOP_STATUS_CENTER_Y)
      if not self._strip_deferred:
        # render() draws it above the shrunken screen; a direct call still draws it in place
        self._draw_health_strip(self._health_strip_plan(alert))
    finally:
      rl.rl_pop_matrix()
    # Vehicle alerts remain last and take precedence over the health strip.
    # K9 R passes its alert slot (center, max_w; 8e25c4c5); hand it on unchanged.
    super()._draw_alert_overlay(alert, **slot)

  def _health_strip_plan(self, alert):
    """(message, color) for the top strip; None when nothing is shown there. Vehicle alerts win."""
    message, color = health_banner(self.host_health, self.language)
    vehicle_alert = alert is not None and alert.size > 0 and (alert.text1 or alert.text2)
    if vehicle_alert or not message:
      return None
    return message, color

  def _draw_health_strip(self, plan):
    if plan is None:
      return
    message, color = plan
    rl.draw_rectangle(0, 0, int(DESIGN_WIDTH), 32, rl_color((0, 0, 0), 220))
    self._draw_text(self._ellipsize_text(message, 22, DESIGN_WIDTH - 40),
                    DESIGN_WIDTH / 2, 16, 22, color, anchor='center')
