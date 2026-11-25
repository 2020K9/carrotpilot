"""11/26 06:18 (대감님): the bottom-left C4 (comma) / J (Jetson) lines say whether each runs 2020K9 code and
whether both are the same build, only from what is already on the device; a remote check is shown only when it
could actually be made, otherwise '확인 불가'. Never guessed, never a blocking network call in the render loop.

  C4 <branch> <owner> 일치|불일치 <commit8> 원격 최신|원격 새판 <8>|원격 확인 불가
  J <model> <owner> 일치|불일치 <commit8> 같음|다름|확인 불가

Each line is a list of (text, kind) segments; kind is 'text', 'ok', 'bad' or 'unknown' (the renderer picks the
colours, 'bad' = 불일치/다름 stands out)."""
from __future__ import annotations

import os
import re
import subprocess
import threading
import time

K9_OWNER = '2020K9'
UNKNOWN = '?'
REMOTE_REFRESH_S = 300.   # background, occasional (never per frame)
REMOTE_TIMEOUT_S = 4.
SHORT_MODEL = {'N16 Nav': 'N16', 'Light speed 2': 'LS2', 'Light speed': 'LS', 'MD v3': 'MD3', 'md-v3': 'MD3',
               'Flip Flop': 'FF', 'Fairy Tale': 'FT'}


def text(value):
  if isinstance(value, bytes):
    value = value.decode('utf-8', 'replace')
  return value.strip() if isinstance(value, str) else ''


def owner(remote):
  """Owner of a git remote / release URL ('2020K9' for https://github.com/2020K9/carrotpilot), '' if not one."""
  remote = text(remote)
  m = re.search(r'github\.com[:/]+([^/\s:@]+)/', remote)
  if m:
    return m.group(1)
  m = re.search(r'([^/:@\s]+)/[^/\s]+?(?:\.git)?/?$', remote)
  return m.group(1) if m else ''


def sha8(value):
  value = text(value)
  return value[:8].lower() if re.fullmatch(r'[0-9a-fA-F]{7,64}', value) else ''


def same_commit(a, b):
  a, b = text(a).lower(), text(b).lower()
  if not (re.fullmatch(r'[0-9a-f]{7,64}', a) and re.fullmatch(r'[0-9a-f]{7,64}', b)):
    return None
  n = min(len(a), len(b))
  return a[:n] == b[:n]


def short_model(name):
  name = text(name)
  return '/'.join(SHORT_MODEL.get(x, x) for x in name.split('/')) if name else ''


def owner_segment(remote):
  who = owner(remote)
  if not who:
    return [(f'{UNKNOWN} ', 'text'), ('확인 불가', 'unknown')]
  return [(f'{who} ', 'text'), ('일치', 'ok') if who == K9_OWNER else ('불일치', 'bad')]


def remote_segment(remote_head, commit):
  """remote_head: {'state': 'ok'|'unknown', 'sha': <remote branch head>} from a background check, or None."""
  head = remote_head if isinstance(remote_head, dict) else {}
  if head.get('state') == 'behind':
    return (f"원격 새판 +{head.get('count') or '?'}", 'bad')
  if head.get('state') == 'ok':
    same = same_commit(head.get('sha'), commit)
    if same:
      return ('원격 최신', 'ok')
    if same is False:
      return ('원격 다름', 'bad')   # short: the row must not reach the road drawing
  return ('원격 확인 불가', 'unknown')


def c4_line(branch, remote, commit, remote_head=None):
  return ([(f'C4 {text(branch) or UNKNOWN} ', 'text'), *owner_segment(remote),
           (f' {sha8(commit) or UNKNOWN} ', 'text'), remote_segment(remote_head, commit)])


def jetson_match(j_commit, j_model_sha, c4_commit, release):
  """같음 when the Jetson bundle commit is the C4 HEAD or the source_commit of the release.json in the comma's
  checkout (and the loaded model is that release's model when both are known); 다름 when known and not;
  확인 불가 when the device does not have both sides."""
  release = release if isinstance(release, dict) else {}
  expected = text(release.get('source_commit'))
  model = release.get('model') if isinstance(release.get('model'), dict) else {}
  results = [x for x in (same_commit(j_commit, c4_commit), same_commit(j_commit, expected)) if x is not None]
  if not results:
    return ('확인 불가', 'unknown')
  if not any(results):
    return ('다름', 'bad')
  want, have = text(model.get('sha256')).lower(), text(j_model_sha).lower()
  if want and have and want != have:
    return ('다름', 'bad')
  return ('같음', 'ok')


def j_line(model_name, remote, commit, model_sha, c4_commit, release):
  return ([(f'J {short_model(model_name) or UNKNOWN} ', 'text'), *owner_segment(remote),
           (f' {sha8(commit) or UNKNOWN} ', 'text'), jetson_match(commit, model_sha, c4_commit, release)])


def line_text(segments):
  return ''.join(s for s, _ in segments)


class RemoteHead:
  """Remote branch head via read-only `git ls-remote` in a low-priority background thread every
  REMOTE_REFRESH_S (writes nothing to the repository). status() never blocks; until a check succeeds, or when
  there is no network, it is {'state': 'unknown'}."""

  def __init__(self, repo, branch=None, refresh_s=REMOTE_REFRESH_S, timeout_s=REMOTE_TIMEOUT_S, enabled=True):
    self.repo, self.branch = repo, branch
    self.refresh_s, self.timeout_s, self.enabled = refresh_s, timeout_s, enabled
    self._status = {'state': 'unknown'}
    self._next = 0.
    self._lock = threading.Lock()
    self._worker = None

  def status(self, branch=None):
    if branch:
      self.branch = text(branch)
    now = time.monotonic()
    with self._lock:
      if (self.enabled and self.repo and self.branch and now >= self._next
          and not (self._worker is not None and self._worker.is_alive())):
        self._next = now + self.refresh_s
        self._worker = threading.Thread(target=self._refresh, name='version-badge-remote', daemon=True)
        self._worker.start()
      return dict(self._status)

  def _refresh(self):
    try:
      os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    except (AttributeError, OSError, ValueError):
      pass
    status = {'state': 'unknown'}
    try:
      env = {**os.environ, 'GIT_TERMINAL_PROMPT': '0'}
      out = subprocess.run(['git', '-C', str(self.repo), 'ls-remote', '--heads', 'origin', f'refs/heads/{self.branch}'],
                           stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True, env=env,
                           timeout=self.timeout_s, check=False)
      first = out.stdout.split() if out.returncode == 0 else []
      if first and re.fullmatch(r'[0-9a-f]{40}', first[0]):
        status = {'state': 'ok', 'sha': first[0]}
    except (OSError, subprocess.SubprocessError):
      pass
    with self._lock:
      self._status = status


def catalog_name(sha256, directory):
  """Model catalog name(s) whose Jetson spec is this ONNX ('/'-joined), '' when unknown."""
  import json
  from pathlib import Path
  sha = text(sha256).lower()
  if not sha:
    return ''
  try:
    directory = Path(directory)
    names = []
    for entry in json.loads((directory / 'catalog.json').read_text(encoding='utf-8')).values():
      spec = json.loads((directory / entry['jetson_spec']).read_text(encoding='utf-8'))
      if str(spec.get('sha256', '')).lower() == sha and entry.get('name') and entry['name'] not in names:
        names.append(entry['name'])
    return '/'.join(names)
  except (OSError, ValueError, KeyError, TypeError, AttributeError):
    return ''


def git_status_head(status, commit):
  """The comma cluster's own GitBranchStatus (its background fetch) as a remote_segment input."""
  state = getattr(status, 'state', None)
  if state == 'ok':
    return {'state': 'ok', 'sha': text(commit)}
  if state == 'pull':
    m = re.search(r'\+(\d+)', text(getattr(status, 'detail', '')))
    return {'state': 'behind', 'count': int(m.group(1)) if m else 0}
  return None


class CommaVersionBadge:
  """Comma-side lines for the comma's own external HUD (cluster main.py). Local reads only: the checkout's
  HEAD/origin are re-read when .git/HEAD or the branch ref changes (checked every 2 s), the Jetson side from
  the shared-memory link status file (peer = what the connected Jetson reported in its hello)."""

  def __init__(self, repo, link_status='/dev/shm/carrot-jetlink.json', release_path=None, catalog_dir=None):
    from pathlib import Path
    self.repo = Path(repo) if repo else None
    base = self.repo or Path('.')
    self.link_status = Path(link_status)
    self.release_path = Path(release_path) if release_path else base / 'openpilot/selfdrive/modeld/jetlink/host_release.json'
    self.catalog_dir = Path(catalog_dir) if catalog_dir else base / 'openpilot/selfdrive/modeld/model_catalog'
    self._git_key = self._link_key = None
    self._git = {}
    self._release = None
    self._peer = {}
    self._next = 0.
    self._lines = ()
    self._lines_key = None

  def _git_files_key(self):
    try:
      head = (self.repo / '.git/HEAD')
      ref = head.read_text().strip()
      paths = [head] + ([self.repo / '.git' / ref[5:]] if ref.startswith('ref: ') else [])
      return tuple((str(p), p.stat().st_mtime_ns) if p.exists() else (str(p), 0) for p in paths)
    except OSError:
      return None

  def _read_git(self):
    def run(*args):
      try:
        out = subprocess.run(['git', '-C', str(self.repo), *args], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                             text=True, timeout=1., check=False)
        return out.stdout.strip() if out.returncode == 0 else ''
      except (OSError, subprocess.SubprocessError):
        return ''
    self._git = {'commit': run('rev-parse', 'HEAD'), 'branch': run('symbolic-ref', '--short', '-q', 'HEAD'),
                 'remote': re.sub(r'^([a-z+]+://)[^/@]*@', r'\1', run('remote', 'get-url', 'origin'))}
    try:
      from openpilot.common.jetson_model import selected_release
      self._release = selected_release(self.release_path)
    except Exception:
      self._release = None

  def _read_peer(self):
    import json
    try:
      key = self.link_status.stat().st_mtime_ns
    except OSError:
      self._peer, self._link_key = {}, None
      return
    if key == self._link_key:
      return
    self._link_key = key
    try:
      peer = json.loads(self.link_status.read_text()).get('peer')
      self._peer = peer if isinstance(peer, dict) else {}
    except (OSError, ValueError, AttributeError):
      self._peer = {}

  def lines(self, git_status=None, now=None):
    if self.repo is None:
      return ()
    now = time.monotonic() if now is None else now
    if now >= self._next:
      self._next = now + 2.
      key = self._git_files_key()
      if key != self._git_key:
        self._git_key = key
        self._read_git()
      self._read_peer()
    git, peer = self._git, self._peer
    head = git_status_head(git_status, git.get('commit'))
    j_commit = text(peer.get('carrot_source_commit'))
    lines_key = (tuple(sorted(git.items())), head and tuple(sorted(head.items())), j_commit,
                 text(peer.get('carrot_source_origin')), text(peer.get('loaded')), id(self._release))
    if lines_key != self._lines_key:
      self._lines_key = lines_key
      lines = [tuple(c4_line(git.get('branch'), git.get('remote'), git.get('commit'), head))]
      if j_commit:
        lines.append(tuple(j_line(catalog_name(peer.get('loaded'), self.catalog_dir), peer.get('carrot_source_origin'),
                                  j_commit, peer.get('loaded'), git.get('commit'), self._release)))
      self._lines = tuple(lines)
    return self._lines
