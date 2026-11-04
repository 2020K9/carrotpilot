import io
import json
import os
import pickle
import shutil
import struct
import tempfile
from collections.abc import Collection
from pathlib import Path
from typing import TypeVar


MODELS_DIR = Path(__file__).resolve().parent / 'models'
TG_INPUT_DEVICES_PATH = MODELS_DIR / 'tg_input_devices.json'
VisionStreamT = TypeVar("VisionStreamT")


def select_vision_streams(available_streams: Collection[VisionStreamT], road_stream: VisionStreamT,
                          wide_stream: VisionStreamT, use_wide_camera: bool) -> tuple[VisionStreamT | None, bool]:
  """Select modeld camera inputs without waiting on a disabled wide camera."""
  if road_stream in available_streams:
    return road_stream, use_wide_camera and wide_stream in available_streams
  if use_wide_camera and wide_stream in available_streams:
    return wide_stream, False
  return None, False


def _default_tg_input_devices(process_name: str):
  backend = os.getenv("DEV", "QCOM")
  defaults = {
    'openpilot.selfdrive.modeld.modeld': {
      'default': {'WARP_DEV': backend, 'QUEUE_DEV': backend},
    },
    'openpilot.selfdrive.modeld.dmonitoringmodeld': {
      'default': {'DEV': backend},
    },
  }
  return defaults[process_name]['default']


def get_tg_input_devices(process_name: str):
  try:
    with open(TG_INPUT_DEVICES_PATH) as f:
      return json.load(f)[process_name]['default']
  except FileNotFoundError:
    return _default_tg_input_devices(process_name)

LIGHTSPEED_C4_ONNX = MODELS_DIR / 'lightspeed_c4_direct.onnx'
LIGHTSPEED_C4_PKL = MODELS_DIR / 'lightspeed_c4_direct_driving_tinygrad.pkl'
LIGHTSPEED_STUDENT_ONNX = MODELS_DIR / 'lightspeed_student_b.onnx'
LIGHTSPEED_STUDENT_PKL = MODELS_DIR / 'lightspeed_student_driving_tinygrad.pkl'
# internal (dev) selection of the directly reduced light speed; off unless one of these is set.
# Not a user param on purpose: the user menu (LightspeedStudentModel) is unchanged.
LIGHTSPEED_C4_SELECT_ENV = 'LIGHTSPEED_C4_DIRECT'
LIGHTSPEED_C4_SELECT_FILE = MODELS_DIR / 'lightspeed_c4_direct.select'


def lightspeed_c4_direct_selected(environ=None, select_file=None) -> bool:
  environ = os.environ if environ is None else environ
  select_file = LIGHTSPEED_C4_SELECT_FILE if select_file is None else select_file
  v = environ.get(LIGHTSPEED_C4_SELECT_ENV)
  if v is not None:
    return v.strip() == '1'
  try:
    return Path(select_file).read_text().strip() == '1'
  except OSError:
    return False


def optional_compile_cmd(cmd: str, pkl_path, tag: str) -> str:
  # an optional model's compile never fails the build: drop any stale/partial pkl first and after a failure,
  # so the chunk step then writes an empty manifest and modeld falls back to the stock model at start
  fail = f'echo "  [{tag}] compile failed; stock model unaffected"'
  return f'rm -f {pkl_path}; {cmd} || (rm -f {pkl_path}; {fail})'


def make_internal_model(model_cls, cam_w: int, cam_h: int, use_student: bool, params=None, use_direct: bool | None = None, **kwargs):
  # read once at modeld start; a student / direct load failure rebuilds the stock model with a fresh state.
  # The model that actually loaded (and why it fell back) goes to cloudlog and LightspeedModelStatus.
  from openpilot.common.swaglog import cloudlog
  if use_direct is None:
    use_direct = lightspeed_c4_direct_selected()
  model = fallback_reason = None
  if use_direct:
    try:
      model = model_cls(cam_w, cam_h, LIGHTSPEED_C4_PKL, **kwargs)
    except Exception as e:
      fallback_reason = f"{type(e).__name__}: {e}"[:200]
      cloudlog.exception("lightspeed c4 direct model failed to load; using stock model")
  elif use_student:
    try:
      model = model_cls(cam_w, cam_h, LIGHTSPEED_STUDENT_PKL, **kwargs)
    except Exception as e:
      fallback_reason = f"{type(e).__name__}: {e}"[:200]
      cloudlog.exception("lightspeed student model failed to load; using stock model")
  if model is None:
    model = model_cls(cam_w, cam_h, **kwargs)
  if use_direct:
    status = {"model": "lightspeed_c4_direct" if fallback_reason is None else "stock",
              "requested_student": use_student, "requested_direct": True, "fallback_reason": fallback_reason}
  else:
    status = {"model": "lightspeed_student_b" if use_student and fallback_reason is None else "stock",
              "requested_student": use_student, "fallback_reason": fallback_reason}
  cloudlog.warning(f"modeld internal model: {status}")
  if params is not None:
    params.put("LightspeedModelStatus", status)
  return model


def modeld_pkl_path():
  return MODELS_DIR / 'driving_tinygrad.pkl'

def dump_oob(obj, f):
  with tempfile.TemporaryFile(dir=".") as tmp:
    def buffer_callback(pb: pickle.PickleBuffer):
      m = pb.raw()
      tmp.write(struct.pack('<q', m.nbytes))
      tmp.write(m)
      pb.release() # keep peak ram at ~1 buffer
    stream = io.BytesIO()
    pickle.Pickler(stream, protocol=5, buffer_callback=buffer_callback).dump(obj)
    opcodes = stream.getvalue()
    f.write(struct.pack('<q', len(opcodes)))
    f.write(opcodes)
    tmp.seek(0)
    shutil.copyfileobj(tmp, f)

def load_oob(f):
  opcodes = f.read(struct.unpack('<q', f.read(8))[0])
  def buffers():
    prev = None
    while (h := f.read(8)):
      if prev is not None:
        prev.release()
      buf = bytearray(struct.unpack('<q', h)[0])
      if f.readinto(buf) != len(buf):
        raise EOFError("incomplete model buffer")
      prev = pickle.PickleBuffer(buf)
      yield prev
  return pickle.load(io.BytesIO(opcodes), buffers=buffers())


def validate_model_file(path: str | Path) -> None:
  with open(path, "rb") as f:
    load_oob(f)
    if f.read(1):
      raise ValueError("unexpected model buffer data")
