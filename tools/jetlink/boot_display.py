"""CPU-rendered USB diagnosis, independent of comma packets, model and Xorg."""
from io import BytesIO
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / 'openpilot/selfdrive/carrot/cluster'))
from boot_status import collect, startup_marks
from display_owner import hud_requested, panel_owner

# Loading % on this waiting screen (대감님 11/20 10:38·11:21·23:12). Only stages that really pass while this
# screen is up, in the order the code makes them happen; % = stages done / 6, never a timer or a guess.
# The first five are the server's first-time marks in its health file (boot_status.startup_marks); without a
# fresh, readable file there is no % line at all and the screen is drawn exactly as before. The comma sends
# HUD data only after its warm-up inferences, and hud.py asks for the panel only after that data says
# IsOnroad, so the last stage (the hand-over to the HUD) is 100%.
LOADING_STAGES = (
  ('server_start', '젯슨 서버 시작'),    # server.py start: host_health.HostHealth() writes the health file
  ('hello', '콤마 연결'),               # server.py HELLO_REQ (comma jetlink daemon greet)
  ('engine_req', '모델 요청'),           # server.py ENGINE_REQ (comma prepare -> ensure_engine)
  ('engine_ready', '모델 준비'),         # server.py ENGINE_RESP ready
  ('infer', '첫 계산'),                 # host_health.note_inference: first OK INFER_RESP (comma warm-up)
  ('hud', 'HUD 화면으로 넘김'),          # display_owner.hud_requested(): hud.py takes the panel
)
LOADING_DY, LOADING_SIZE = 94, 21      # one line right below the stage cell's value (y+61, 23 px), above the next row
# Remaining time next to the %, an ESTIMATE from measured stage times (대감님 11/21 16:07). Seconds each stage took
# after the previous one, from the comma logs of the car (swaglog jetlinkConnectionState + jetsonPerformance; the
# comma sees these Jetson steps): 11/18 15:12 cold start (comma_logs_1118 c) and 11/21 14:52~15:07 (7 reconnects,
# 1 Jetson reboot). hello: Jetson ready to comma HELLO+ENGINE_REQ, 6.3 / 6.7 s after a Jetson (re)start;
# engine_req: sent in the same step as HELLO (not separable, 0); engine_ready: comma loading to ready, 0.36~0.45 s
# (median of 9: 0.37); infer: ready to first Jetson result, median of 6 = 1.05 s (1 Hz log, upper bound); hud: comma
# warm-up (10 frames at 20 Hz) + HUD data + IsOnroad: NOT in any log, 1.5 s is a guess from those code constants.
# Server start itself has no % line (no health file yet), so no time either.
STAGE_SECONDS = {"hello": 6.5, "engine_req": 0.0, "engine_ready": 0.37, "infer": 1.05, "hud": 1.5}


def remaining(status):
  """estimated whole seconds until the HUD takes the panel, or None (no fresh timed marks / already handed over):
  the measured time of every stage not done yet, minus the time already spent in the current one (never below 1)."""
  at, marks = status.get("jetson_at"), status.get("jetson")
  if not isinstance(at, dict) or not isinstance(marks, dict) or status.get("hud") is True or "server_start" not in at:
    return None
  todo = [name for name, _ in LOADING_STAGES[1:] if name == "hud" or marks.get(name) is not True]
  done = [t for name, t in at.items() if name != "now" and (name == "server_start" or marks.get(name) is True)]
  spent = max(0.0, at["now"] - max(done))
  left = max(0.0, STAGE_SECONDS[todo[0]] - spent) + sum(STAGE_SECONDS[name] for name in todo[1:])
  return max(1, round(left))


def loading(status):
  """% of the loading stages done, or None (no fresh server health file: then no % line)."""
  marks = status.get('jetson')
  if not isinstance(marks, dict):
    return None
  done = sum(1 for name, _ in LOADING_STAGES[:-1] if marks.get(name) is True) + (status.get('hud') is True)
  return 100 * done // len(LOADING_STAGES)


def lines(status):
  units = status.get('units', {})
  failed = [name for name, value in units.items() if value.get('ActiveState') == 'failed']
  stage = status.get('stage', {})
  storage = status.get('storage', {}).get('state', 'waiting')
  error = bool(failed or stage.get('error') or storage == 'base-recovery')
  title = ('부팅 상태를 확인해 주세요', 'BOOT NEEDS ATTENTION') if error else ('젯슨 연결 준비 중', 'JETSON STARTING / WAITING')
  rows = [
    ('시스템 보호', 'SYSTEM STORAGE', storage),
    ('부팅 단계', 'BOOT STAGE', stage.get('stage', 'starting')),
    ('무선 네트워크', 'WI-FI', str(status.get('wifi', 'waiting'))),
    ('접속 주소', 'IP ADDRESS', ' / '.join(status.get('addresses', [])) or 'Waiting for network'),
    ('원격 접속', 'SSH', units.get('ssh', {}).get('ActiveState', 'waiting')),
    ('콤마 USB 서비스', 'COMMA USB SERVICE', units.get('carrot-jetlink', {}).get('ActiveState', 'waiting')),
  ]
  detail = ', '.join(failed[:3]) or stage.get('error', '')
  firmware = status.get('firmware', {})
  if firmware:
    state = firmware.get('state', 'checking')
    rows[1] = ('Super 펌웨어', 'SUPER FIRMWARE', state)
    if state in ('stage_capsule', 'staging', 'reboot_pending'):
      title = ('Super 펌웨어 자동 설치 중', 'SUPER FIRMWARE UPDATE')
      detail = '전원을 유지하세요. 펌웨어 설치 후 자동으로 재부팅합니다.'
      error = False
    elif state not in ('ready', 'firmware_ready'):
      title = ('Super 설정 확인 필요', 'SUPER SETUP NEEDS ATTENTION')
      detail = firmware.get('error') or firmware.get('power', {}).get('state') or state
      error = True
  return title, rows, detail, error


def render(status, root=ROOT, size=(1920, 462)):
  from PIL import Image, ImageDraw, ImageFont
  image = Image.new('RGB', size, '#101821')
  draw = ImageDraw.Draw(image)
  font_path = root / 'openpilot/selfdrive/assets/fonts/KaiGenGothicKR-Bold.ttf'
  font = lambda n: ImageFont.truetype(str(font_path), n)
  title, rows, detail, error = lines(status)
  accent = '#ff8876' if error else '#65d7bb'
  pct = loading(status)
  draw.rectangle((0, 0, 12, size[1]), fill=accent)
  draw.text((38, 20), title[0], font=font(36), fill='white')
  draw.text((40, 65), title[1], font=font(19), fill=accent)
  draw.text((1550, 34), f"BOOT +{status.get('uptime', 0)}s", font=font(22), fill='#97a9bc')
  if status.get('temp_c') is not None:
    draw.text((1750, 34), f"{status['temp_c']:.1f} C", font=font(22), fill=accent)
  for index, (ko, en, value) in enumerate(rows):
    x, y = 40 + (index % 3) * 625, 116 + (index // 3) * 122
    draw.text((x, y), ko, font=font(26), fill='#dce7f1')
    draw.text((x, y+34), en, font=font(15), fill='#8198ae')
    value_font = font(23)
    value = str(value)
    while value and draw.textlength(value, font=value_font) > 585:
      value = value[:-4] + '...' if len(value) > 4 else ''
    draw.text((x, y+61), value, font=value_font, fill=accent)
    if index == 1 and pct is not None:
      # The stage cell stays as it is; its loading % is one more line right below it (same column and colour).
      draw.text((x, y + LOADING_DY), f'로딩 {pct}%', font=font(LOADING_SIZE), fill=accent)
      left = remaining(status)
      if left is not None:   # its own draw call after the % (same line, same font): the % text itself is unchanged
        dx = draw.textlength(f"로딩 {pct}%", font=font(LOADING_SIZE))
        draw.text((x + dx, y + LOADING_DY), f" · 남은 시간 약 {left}초 (추정)", font=font(LOADING_SIZE), fill=accent)
  footer = detail or '콤마 연결과 계기판 화면 준비를 기다립니다'
  draw.text((40, 373), footer[:105], font=font(22), fill=accent)
  draw.text((40, 410), 'Waiting for comma / dashboard. This is a status screen, not driving guidance.',
            font=font(17), fill='#97a9bc')
  return image


def panel_frame(frame):
  from PIL import Image
  # Match cluster_renderer portrait_upload's render/readback transform.
  return frame.transpose(Image.Transpose.ROTATE_270)


def show(panel, status):
  frame = render(status, size=(panel.landscape_width, panel.landscape_height))
  stream = BytesIO()
  panel_frame(frame).save(stream, format='JPEG', quality=82)
  panel.send_jpeg(stream.getvalue())


def show_until_hud(panel, read=collect, requested=hud_requested, marks=startup_marks, sleep=time.sleep):
  """A status frame every second until the HUD asks for the panel; then one hand-over frame (100%) that stays
  on the panel until the HUD's first frame. Its other fields are the last ones read (no new systemctl wait)."""
  status = None
  while not requested():
    status = read()
    show(panel, status)
    sleep(1)
  if status is not None:
    show(panel, {**status, 'jetson': marks(), 'hud': True})


def main():
  import cluster_usb_display as usb
  import openpilot.common.usbgpu_bus_lock as bus
  # RAM path exists even if the storage setup failed before mounting /tmp.
  bus.USBGPU_BUS_LOCK_PATH = '/run/carrot-jetlink-display/usb.lock'
  usb._set_cluster_hud_connected = lambda value: None
  usb._usbgpu_reset_protected = lambda: True  # Diagnostics never reset the shared USB path.
  class Panel(usb.TuringUsbDisplay):
    def close(self):
      # Preserve the last diagnostic image until the HUD sends its first frame.
      self._disconnected = True
      super().close()
  while True:
    with panel_owner() as acquired:
      if acquired and usb.find_supported_usb_product(0x0092) is not None:
        panel = Panel(brightness=65, display_fps=1, expected_product_id=0x0092)
        try:
          panel.open()
          show_until_hud(panel)
        except Exception as error:
          print('Boot display retry:', type(error).__name__, flush=True)
        finally:
          panel.close()
    time.sleep(1)


if __name__ == '__main__':
  main()
