import hashlib
import json
import subprocess
from pathlib import Path

import pytest

import openpilot.selfdrive.modeld.lightspeed_c4_fetch as fetch
from openpilot.selfdrive.modeld.helpers import LIGHTSPEED_C4_ONNX, LIGHTSPEED_C4_PKL, LIGHTSPEED_C4_SELECT_ENV, \
  LIGHTSPEED_STUDENT_PKL, MODELS_DIR, lightspeed_c4_direct_selected, make_internal_model, optional_compile_cmd
from openpilot.selfdrive.ui.ui_state import lightspeed_badge

MODELD = Path(__file__).resolve().parents[1]
STOCK_PKL = MODELS_DIR / 'driving_tinygrad.pkl'


class FakeParams:
  def __init__(self):
    self.store = {}

  def put(self, k, v):
    self.store[k] = v


class FakeModel:
  fail = ()
  created: list = []

  def __init__(self, cam_w, cam_h, pkl_path=None, *, enable_shadow=True):
    if pkl_path in FakeModel.fail:
      raise FileNotFoundError("no pkl")
    self.pkl_path = pkl_path or STOCK_PKL
    self.state = []
    FakeModel.created.append(self)


def make(use_student=False, use_direct=None, fail=()):
  FakeModel.fail, FakeModel.created = fail, []
  params = FakeParams()
  model = make_internal_model(FakeModel, 1928, 1208, use_student, params, use_direct=use_direct, enable_shadow=False)
  return model, params.store["LightspeedModelStatus"]


# selection: internal only, default off
def test_selection_default_off(tmp_path):
  assert not lightspeed_c4_direct_selected({}, tmp_path / 'missing')


@pytest.mark.parametrize("env,file,want", [
  ({LIGHTSPEED_C4_SELECT_ENV: '1'}, None, True),
  ({LIGHTSPEED_C4_SELECT_ENV: '0'}, '1', False),
  ({LIGHTSPEED_C4_SELECT_ENV: 'yes'}, None, False),
  ({}, '1\n', True),
  ({}, '0', False),
  ({}, '', False),
])
def test_selection_env_and_file(tmp_path, env, file, want):
  f = tmp_path / 'sel'
  if file is not None:
    f.write_text(file)
  assert lightspeed_c4_direct_selected(env, f) is want


def test_selection_is_not_a_user_param():
  keys = (MODELD.parents[1] / 'common/params_keys.h').read_text()
  assert 'LightspeedC4' not in keys and 'lightspeed_c4' not in keys
  assert '{"LightspeedStudentModel", {PERSISTENT, BOOL, "0"}}' in keys


def test_make_internal_model_reads_selection(monkeypatch):
  monkeypatch.setenv(LIGHTSPEED_C4_SELECT_ENV, '1')
  model, _ = make()
  assert model.pkl_path == LIGHTSPEED_C4_PKL
  monkeypatch.setenv(LIGHTSPEED_C4_SELECT_ENV, '0')
  model, _ = make()
  assert model.pkl_path == STOCK_PKL


# loading
@pytest.mark.parametrize("use_student", [False, True])
def test_direct_loads(use_student):
  model, s = make(use_student, use_direct=True)
  assert model.pkl_path == LIGHTSPEED_C4_PKL and len(FakeModel.created) == 1
  assert s == {"model": "lightspeed_c4_direct", "requested_student": use_student, "requested_direct": True, "fallback_reason": None}
  assert lightspeed_badge(s) == ("LS-C4", "active")


@pytest.mark.parametrize("use_student", [False, True])
def test_direct_failure_falls_back_to_stock_with_new_state(use_student):
  model, s = make(use_student, use_direct=True, fail=(LIGHTSPEED_C4_PKL,))
  assert model.pkl_path == STOCK_PKL and len(FakeModel.created) == 1 and model.state == []
  assert s["model"] == "stock" and s["requested_direct"] and s["fallback_reason"] == "FileNotFoundError: no pkl"
  assert lightspeed_badge(s) == ("LS>STOCK", "error")


def test_direct_off_keeps_student_meaning():
  model, s = make(True, use_direct=False)
  assert model.pkl_path == LIGHTSPEED_STUDENT_PKL and "requested_direct" not in s
  model, s = make(False, use_direct=False)
  assert model.pkl_path == STOCK_PKL and lightspeed_badge(s) is None


# build isolation
def run_sh(cmd):
  return subprocess.run(['bash', '-c', cmd], capture_output=True, text=True)


def test_optional_compile_failure_never_fails_and_leaves_no_partial(tmp_path):
  pkl = tmp_path / 'm.pkl'
  pkl.write_text('stale')
  r = run_sh(optional_compile_cmd(f"sh -c 'echo partial > {pkl}; exit 3'", pkl, 'LIGHTSPEED C4 DIRECT'))
  assert r.returncode == 0 and not pkl.exists()
  assert '[LIGHTSPEED C4 DIRECT] compile failed; stock model unaffected' in r.stdout


def test_optional_compile_success_keeps_pkl(tmp_path):
  pkl = tmp_path / 'm.pkl'
  pkl.write_text('stale')
  r = run_sh(optional_compile_cmd(f'echo new > {pkl}', pkl, 'X'))
  assert r.returncode == 0 and pkl.read_text() == 'new\n'


def test_sconscript_builds_direct_as_optional():
  s = (MODELD / 'SConscript').read_text()
  assert "optional = pkl_path in (LIGHTSPEED_STUDENT_PKL, LIGHTSPEED_C4_PKL)" in s
  assert "model_builds.append((LIGHTSPEED_C4_PKL, [str(LIGHTSPEED_C4_ONNX)], str(LIGHTSPEED_C4_ONNX), tg_flags))" in s
  assert "os.getenv('BUILD_LIGHTSPEED_C4') != '0'" in s


# committed artifact
def test_committed_onnx_matches_catalog_and_has_metadata():
  entry = json.loads(fetch.CATALOG.read_text())['light-speed']['c4_builtin']
  assert entry['filename'] == LIGHTSPEED_C4_ONNX.name
  data = LIGHTSPEED_C4_ONNX.read_bytes()
  assert hashlib.sha256(data).hexdigest() == entry['reduced_sha256']
  # compile_modeld needs output_slices from the model metadata (onnx.utils.extract_model drops it)
  assert b'output_slices' in data


# fetch script cleanup
def test_fetch_download_failure_leaves_nothing(tmp_path, monkeypatch):
  out = tmp_path / 'lightspeed_c4_direct.onnx'

  def half_download(url, dst):
    Path(dst).write_text('half')
    raise OSError('network')
  monkeypatch.setattr(fetch.urllib.request, 'urlretrieve', half_download)
  monkeypatch.setattr('sys.argv', ['fetch', '--out', str(out)])
  with pytest.raises(OSError):
    fetch.main()
  assert list(tmp_path.iterdir()) == []


def test_fetch_reduced_sha_mismatch_removes_output(tmp_path, monkeypatch):
  src, out = tmp_path / 'src.onnx', tmp_path / 'lightspeed_c4_direct.onnx'
  src.write_text('src')
  entry = json.loads(fetch.CATALOG.read_text())['light-speed']['c4_builtin']
  monkeypatch.setattr(fetch, 'sha256', lambda p: entry['sha256'] if Path(p) == src else 'other')
  monkeypatch.setattr(fetch, 'shrink', lambda s, d: Path(d).write_text('cut'))
  monkeypatch.setattr('sys.argv', ['fetch', '--src', str(src), '--out', str(out)])
  with pytest.raises(AssertionError, match='reduced sha256'):
    fetch.main()
  assert not out.exists() and src.exists()
