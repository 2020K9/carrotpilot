"""Read-only display identity; independent of model/runtime imports."""
import json
import ipaddress
import math
from pathlib import Path
import time

from openpilot.common.jetlink_peer import classify_auto_peer, is_mac_peer, may_provision

LINK_STATUS = Path('/dev/shm/carrot-jetlink.json')
MODEL_STATUS = Path('/dev/shm/carrot-jetlink-model.json')
HOST_LABELS = {'jetson': 'jetSON', 'mac': 'MAC'}


def update_badge(value):
  if not isinstance(value, dict):
    value = {}
  state = value.get('state')
  if state == 'waiting_internet':
    return 'Jetson 업데이트 필요 · 인터넷 연결 대기', 'loading'
  if state == 'downloading':
    fraction = value.get('fraction')
    percent = f' {int(fraction * 100)}%' if isinstance(fraction, (int, float)) and 0 <= fraction <= 1 else ''
    return f'Jetson 업데이트 다운로드 중{percent}', 'loading'
  if state == 'verifying':
    return 'Jetson 업데이트 검증·적용 중', 'loading'
  if state == 'failed':
    return 'Jetson 업데이트 실패 · 재시도 대기', 'error'
  return 'Jetson 업데이트 확인 중', 'loading'


def _read(path):
  try:
    value = json.loads(path.read_text())
    return value if isinstance(value, dict) else {}
  except (OSError, ValueError):
    return {}


def host_label(peer, mode='usb'):
  if not isinstance(peer, dict):
    return 'Jetlink'
  if mode == 'auto':
    mode = classify_auto_peer(peer, 'usb') or 'auto'
  if mode in ('ios', 'android') and may_provision(peer, mode):
    return 'iOS' if mode == 'ios' else 'Android'
  host = peer.get('carrot_host')
  if isinstance(host, str) and host in HOST_LABELS:
    return HOST_LABELS[host]
  # Compatibility with the already-installed server, before host metadata.
  device = str(peer.get('device', '')).lower()
  if peer.get('backend') == 'trt' and 'orin' in device:
    return 'jetSON'
  if mode == 'usb' and is_mac_peer(peer):
    return 'MAC'
  return 'Jetlink'


def _fresh(path, now=None, *, handshake=False):
  try:
    value = _read(path)
    # Sample after the read: a concurrent atomic replacement can carry a
    # timestamp newer than the time at which this reader started.
    if now is None:
      now = time.monotonic()
    # HELLO blocks for 3s in auto probing, or 5s in the normal path. A 3s
    # display lease falsely declares a dead daemon during that bounded wait.
    # Cover HELLO + the 2s reconnect pause; all other reports still expire at 3s.
    limit = 8 if handshake and value.get('state') == 'connecting' else 3
    if handshake and value.get('state') == 'connecting':
      # Root setup has a 15s subprocess bound; NCM creation adds up to 5s.
      # These explicit phases must outlive those operations, plus reconnect.
      phase = (value.get('preparation') or {}).get('stage')
      if phase in ('usb-setup', 'network-setup'):
        limit = 22
    if isinstance(value, dict) and 0 <= now - value['updated'] < limit:
      return value
  except (OSError, ValueError, KeyError, TypeError):
    pass
  return {}


def err_delay(now=None):
  """K9JetErrDelay on the comma: (frames modeld held so far, seconds since the last one) from modeld's fresh status,
  None without a held frame or a fresh status (HUD '에러지연 작동')."""
  if now is None:
    now = time.monotonic()
  status = _fresh(MODEL_STATUS, now)
  count, at = status.get('err_delay_count'), status.get('err_delay_at')
  if (type(count) is not int or count <= 0 or type(at) not in (int, float) or not math.isfinite(at)):
    return None
  return count, max(0., now - at)


# Jetson connect progress on the comma screen (대감님 11/21 16:07 "접속 로드율(%)과 남은 예상 시간(초)"). An ESTIMATE:
# seconds each step took in the comma logs of the car (swaglog jetlinkConnectionState + jetsonPerformance):
BOOT_WAIT_S = 22.4          # first connecting to the Jetson answering (runtime-handoff), 11/18 15:12 cold start (1 log)
HANDOFF_TO_LOAD_S = 9.1     # runtime-handoff to loading: 8.9 s (11/18 15:12), 9.3 s (11/21 15:07)
RECONNECT_TO_LOAD_S = 5.3   # connecting to loading when the Jetson is already up: median of 7 (11/21 14:52~14:59)
LOAD_TO_READY_S = 0.4       # loading to ready: 0.36~0.45 s, 9 logs
READY_TO_FIRST_S = 1.05     # ready to the first Jetson result (1 Hz jetsonPerformance, upper bound): median of 6
_TRACK = {}                 # phase: first time seen in this wait (UI process clock), reset once the Jetson model runs
_PROGRESS = [None, -1.0]    # (result, time) cache: the badge is drawn every frame, the files are read twice a second


def _phase(link, model):
  if model.get("active"):
    return "active"
  state, stage = link.get("state"), (link.get("preparation") or {}).get("stage")
  if state == "ready":
    return "ready"
  if state == "loading":
    return None if stage in ("waiting", "resident-wait") else "loading"
  if state == "connecting":
    if stage == "runtime-handoff":
      return "handoff"
    return "down" if stage in ("transport-selection", "usb-probe", "waiting-host") else "connecting"
  return "reset" if state in ("waiting", "updating", "stopped", "blocked") else None


def progress(now=None, link=None, model=None):
  """(percent 0..99, whole seconds left >= 1) while the comma connects to the Jetson, else None. ESTIMATE: the
  measured time of every step still ahead minus the time already spent in the current one (constants above)."""
  now = time.monotonic() if now is None else now
  link = _fresh(LINK_STATUS, now, handshake=True) if link is None else link
  model = _fresh(MODEL_STATUS, now) if model is None else model
  phase, t = _phase(link, model), _TRACK
  if phase in ("active", "reset"):
    t.clear()
    return None
  if phase is None:
    return None
  t.setdefault("start", now)
  if phase in ("connecting", "down", "handoff") and ("loading" in t or "ready" in t):
    t.pop("loading", None)      # a new attempt after the model step: those steps are ahead again
    t.pop("ready", None)
    t.pop("connecting", None)   # and this attempt is timed from now
  if phase == "connecting" and "handoff" in t:
    phase = "handoff"           # the USB setup right after the hand-off belongs to that step
  t.setdefault(phase, now)

  def left(key, seconds):
    return max(0., seconds - (now - t[key]))
  tail = LOAD_TO_READY_S + READY_TO_FIRST_S
  if "ready" in t:
    rest = left("ready", READY_TO_FIRST_S)
  elif "loading" in t:
    rest = left("loading", LOAD_TO_READY_S) + READY_TO_FIRST_S
  elif "handoff" in t:
    rest = left("handoff", HANDOFF_TO_LOAD_S) + tail
  elif "down" in t:
    rest = left("start", BOOT_WAIT_S) + HANDOFF_TO_LOAD_S + tail
  else:
    rest = left("connecting", RECONNECT_TO_LOAD_S) + tail
  rest = max(1., rest)
  spent = now - t["start"]
  return min(99, int(100 * spent / (spent + rest))), int(math.ceil(rest))


def progress_text(now=None):
  """short suffix for the comma eGPU/Jetson badge, e.g. 45% · 약 8초 (estimate); empty when there is none."""
  now = time.monotonic() if now is None else now
  if _PROGRESS[0] is None or not 0 <= now - _PROGRESS[1] < .5:
    try:
      _PROGRESS[:] = [progress(now) or (), now]
    except Exception:
      _PROGRESS[:] = [(), now]
  value = _PROGRESS[0]
  return f" {value[0]}% · 약 {value[1]}초" if value else ""


def preparation_text(link, model):
  if model.get('active'):
    return ''
  phase = (link.get('preparation') or {}).get('stage')
  if link.get('state') == 'connecting':
    return {'usb-setup': 'USB 연결 준비 중', 'network-setup': 'USB 통신 준비 중',
            'runtime-handoff': '실행 프로그램 시작 중',
            'waiting-host': '응답 대기 중', 'usb-probe': '응답 확인 중',
            'transport-selection': '연결 장치 확인 중'}.get(phase, '연결 대기')
  if link.get('state') == 'loading':
    if phase == 'resident-wait':
      return '모델 준비 대기 · USB 유지'
    if phase == 'waiting':
      message = str((link.get('preparation') or {}).get('message') or '')
      if 'ignition off' in message.lower():
        return '시동 끄고 모델 준비 필요'
      return '모델 준비 보류'
    return '모델 준비 중'
  if link.get('state') == 'ready' and model:
    return '콤마 영상 준비 중' if model.get('warp_preparing') else '모델 전환 대기'
  return ''


def diagnostics():
  """Only current telemetry can report health or an address; HELLO is identity."""
  saved = _read(LINK_STATUS)
  link = _fresh(LINK_STATUS, handshake=True)
  model = _fresh(MODEL_STATUS)
  now = time.monotonic()
  if not saved.get('peer') and saved.get('state') not in ('connecting', 'loading', 'retrying', 'blocked') and not model.get('active'):
    return None
  result = {'label': host_label(saved.get('peer'), saved.get('transport', 'usb')), 'state': link.get('state', 'disconnected'),
            'severity': 'unknown', 'reason': preparation_text(link, model), 'addresses': [], 'temp_c': None,
            'active': bool(model.get('active')), 'fresh': False}
  preparation = link.get('preparation') or {}
  if link.get('state') == 'loading' and preparation.get('stage') in ('waiting', 'resident-wait'):
    result['reason'] = (result['reason'] + ' · ' + str(preparation.get('message') or '')[:160]).rstrip(' ·')
  if link.get('state') == 'updating' and result['label'] == 'jetSON':
    text, style = update_badge(link.get('host_update') or {})
    result.update(severity='error' if style == 'error' else 'warning', reason=text, fresh=True, active=False)
    return result
  model_error = str(model.get('error') or '')[:240] if not model.get('active') else ''
  if model_error:
    result.update(severity='error', reason=model_error)
  # A current waiting report confirms the optional host is absent, including
  # startup before modeld's first report. A remembered HELLO is not a fault.
  # Keep current model failures/active-session conflicts and stale link state
  # visible; only explicit, fresh absence is quiet.
  if link.get('state') == 'waiting' and not model.get('active') and not model_error:
    result.update(reason='Host not connected')
    return result
  if result['state'] in ('retrying', 'stopped', 'disconnected', 'waiting', 'blocked'):
    result.update(severity='error', reason=str(link.get('error') or 'Host connection unavailable')[:240])
    return result
  telemetry = link.get('telemetry') or {}
  health = telemetry.get('carrot_health') if isinstance(telemetry, dict) else None
  try:
    age = now - float(link['telemetry_updated']) + float(health['age_s'])
    if not 0 <= age < 3:
      return result
  except (KeyError, TypeError, ValueError):
    return result
  result.update(fresh=True, severity=health.get('severity', 'unknown'), reason=str(health.get('reason', ''))[:240])
  if result['severity'] not in ('ok', 'warning', 'error', 'unknown'):
    result['severity'] = 'unknown'
  temperature = health.get('temp_c')
  if isinstance(temperature, (int, float)) and math.isfinite(temperature) and -20 <= temperature <= 150:
    result['temp_c'] = temperature
  entries = health.get('addresses')
  for entry in (entries[:8] if isinstance(entries, list) else []):
    try:
      address = ipaddress.ip_address(entry['address'])
      if not address.is_loopback and not address.is_unspecified and not address.is_link_local:
        result['addresses'].append(str(address))
    except (KeyError, TypeError, ValueError):
      continue
  if model_error:
    result.update(severity='error', reason=model_error)
  elif result['severity'] == 'ok' and not result['active']:
    result['reason'] = preparation_text(link, model) or result['reason']
  return result


def badge():
  link = _fresh(LINK_STATUS, handshake=True)
  model = _fresh(MODEL_STATUS)
  label = host_label(link.get('peer'), link.get('transport', 'usb'))
  if link.get('state') == 'updating' and label == 'jetSON':
    return update_badge(link.get('host_update') or {})
  health = diagnostics()
  if health:
    label = health['label']
    if health['state'] == 'waiting' and health['severity'] == 'unknown':
      return None
    if health['severity'] == 'error':
      return f'{label} ERROR', 'error'
    if health['severity'] == 'warning':
      return f'{label} HOT', 'loading'
    if label == 'jetSON' and link.get('state') == 'ready' and (not health['fresh'] or health['severity'] == 'unknown'):
      return f'{label} DATA?', 'loading'
  if model.get('active'):
    return (f'{label} 사용 중' if label == 'jetSON' else label), 'active'
  progress = preparation_text(link, model)
  if progress:
    return f'{label} {progress}', 'loading'
  state = link.get('state')
  if state == 'ready':
    return f'{label} READY', 'ready'
  if state == 'connecting':
    return f'{label} 연결 대기', 'loading'
  if state == 'loading':
    return f'{label} 모델 준비 중', 'loading'
  if state == 'retrying':
    return f'{label} RETRY', 'error'
  return None
