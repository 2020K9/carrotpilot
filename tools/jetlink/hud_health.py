"""Jetson-local temperature display; never substitute it for vehicle statistics."""
import math
import time

from host_health import STATUS
from openpilot.common.jetlink_status import _fresh
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


# Jetson HUD only (대감님 11/20 10:38/10:39): one connect % from real first-time stage marks, no timer.
# Each stage counts once it has happened (Jetson CLOCK_MONOTONIC marks); % = done / all; hidden at 100.
CONNECT_STAGES = (
  ('server', '젯슨 서버 시작', 'Jetson server'),     # host_health STATUS fresh (daemon running)
  ('hello', '콤마 연결', 'comma link'),              # HELLO_REQ from comma
  ('engine_req', '모델 요청', 'model request'),      # ENGINE_REQ received
  ('engine_ready', '모델 준비', 'model prepare'),    # ENGINE_RESP ready
  ('infer', '첫 계산', 'first inference'),          # first successful INFER_RESP
  ('car', '차량 정보', 'vehicle data'),              # first valid carState in the HUD
)


def connect_progress(health, hud_marks, language='ko'):
  """(percent int, text) while connecting, None once every stage is done."""
  diag = health.get('hud_diag') if isinstance(health, dict) and isinstance(health.get('hud_diag'), dict) else {}
  server = diag.get('startup') if isinstance(diag.get('startup'), dict) else {}
  done = {'server': bool(health) and _num(server.get('server_start'), 0) is not None,
          'car': _num((hud_marks or {}).get('car'), 0) is not None}
  for key in ('hello', 'engine_req', 'engine_ready', 'infer'):
    done[key] = done['server'] and _num(server.get(key), 0) is not None
  count = sum(1 for key, _ko, _en in CONNECT_STAGES if done[key])
  if count == len(CONNECT_STAGES):
    return None
  pct = int(100 * count / len(CONNECT_STAGES))
  # Percent only: the existing stage/status strip above stays exactly as before (대감님 11/20 11:21).
  if language == 'ko':
    return pct, f'젯슨 연결 중 {pct}%'
  return pct, f'Jetson connecting {pct}%'


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


class HealthRenderer(ClusterUiRenderer):
  startup_marks = {}   # hud.py MARKS (Jetson monotonic)

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

  def render(self, state, signal_lights=None):
    self.host_health = _fresh(STATUS, time.monotonic())
    self.host_badge_drawn = False
    super().render(state, signal_lights)

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

  def _draw_alert_overlay(self, alert):
    rl.rl_push_matrix()
    rl.rl_scalef(self.width / DESIGN_WIDTH, self.height / DESIGN_HEIGHT, 1.)
    try:
      if not self.host_badge_drawn:
        # Full navigation/graph modes omit the driving status row.
        self._temperature_badge(DESIGN_WIDTH - 66, TOP_STATUS_CENTER_Y)
      message, color = health_banner(self.host_health, self.language)
      vehicle_alert = alert is not None and alert.size > 0 and (alert.text1 or alert.text2)
      if message and not vehicle_alert:
        rl.draw_rectangle(0, 0, int(DESIGN_WIDTH), 32, rl_color((0, 0, 0), 220))
        self._draw_text(self._ellipsize_text(message, 22, DESIGN_WIDTH - 40),
                        DESIGN_WIDTH / 2, 16, 22, color, anchor='center')
      # Connect % on its own line right under the unchanged status strip (대감님 11/20 11:21).
      # Same priority as before: overheating / real Jetson warnings and errors stay alone.
      progress = connect_progress(self.host_health, self.startup_marks, self.language)
      plain = not message or (self.host_health.get('severity', 'unknown') == 'unknown'
                              and self.host_health.get('thermal_severity') not in ('warning', 'error'))
      if progress is not None and plain and not vehicle_alert:
        rl.draw_rectangle(0, 32, int(DESIGN_WIDTH), 28, rl_color((0, 0, 0), 200))
        self._draw_text(progress[1], DESIGN_WIDTH / 2, 46, 20, WARNING, anchor='center')
    finally:
      rl.rl_pop_matrix()
    # Vehicle alerts remain last and take precedence over the health strip.
    super()._draw_alert_overlay(alert)
