"""Cheddar from Carrot Web's model list: compiled by its pinned runtime, run by its worker."""
from __future__ import annotations

import ast
import hashlib
from pathlib import Path

import pytest

from carrot.model_selector.tests.test_recompile import installer  # same lightweight stubs
from openpilot.selfdrive.modeld.cheddar_runtime import MODEL_SHA256
from openpilot.selfdrive.modeld.helpers import CHEDDAR_RUNTIME_DIR

MODELD = Path(__file__).parents[3] / 'openpilot/selfdrive/modeld/modeld.py'


def _factory():
  # Exercise the production ModelState.__new__ without loading vehicle-only native modules.
  cls = next(node for node in ast.parse(MODELD.read_text(encoding='utf8')).body
             if isinstance(node, ast.ClassDef) and node.name == 'ModelState')
  method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == '__new__')
  minimal = ast.ClassDef(name='ModelState', bases=[], keywords=[], body=[method], decorator_list=[])
  return minimal


def _model_state(monkeypatch, default_pkl):
  from openpilot.selfdrive.modeld import cheddar_model
  calls = []
  monkeypatch.setattr(cheddar_model, 'CheddarModelState', lambda *args: calls.append(args) or 'cheddar')
  scope = {'Path': Path, 'CHEDDAR_RUNTIME_DIR': CHEDDAR_RUNTIME_DIR, 'modeld_pkl_path': lambda external: default_pkl}
  exec(compile(ast.fix_missing_locations(ast.Module(body=[_factory()], type_ignores=[])), str(MODELD), 'exec'), scope)
  return scope['ModelState'], calls


def test_builtin_internal_model_stays_on_the_regular_state(monkeypatch, tmp_path):
  pkl = tmp_path / 'models/driving_tinygrad.pkl'
  pkl.parent.mkdir()
  state, calls = _model_state(monkeypatch, pkl)
  assert isinstance(state(1928, 1208, False), state)
  assert calls == []


def test_selector_cheddar_directory_runs_in_the_cheddar_worker(monkeypatch, tmp_path):
  pkl = tmp_path / 'data_models/driving_tinygrad.pkl'
  (pkl.parent / CHEDDAR_RUNTIME_DIR).mkdir(parents=True)
  state, calls = _model_state(monkeypatch, pkl)
  assert state(1928, 1208, False) == 'cheddar'
  assert state(1344, 760, False, pkl) == 'cheddar'
  assert calls == [(1928, 1208, pkl), (1344, 760, pkl)]
  # eGPU models never take this path.
  assert isinstance(state(1928, 1208, True, pkl), state)


def _fake_run(record, make_runtime=True):
  def run(cmd, env=None):
    record.append((cmd, env))
    out = Path(cmd[cmd.index('--output') + 1])
    out.write_bytes(b'pkl')
    if make_runtime:
      (out.parent / CHEDDAR_RUNTIME_DIR).mkdir(exist_ok=True)
  return run


def test_cheddar_download_uses_the_pinned_runtime_compiler(monkeypatch, tmp_path):
  onnx = tmp_path / 'driving_supercombo.onnx'
  onnx.write_bytes(b'cheddar')
  monkeypatch.setattr(installer, '_is_cheddar', lambda path: path == onnx)
  record = []
  monkeypatch.setattr(installer, '_run', _fake_run(record))
  installer._compile_supercombo(tmp_path, {'DEV': 'QCOM', 'FLOAT16': '1'})
  (cmd, env), = record
  assert Path(cmd[1]).name == 'cheddar_runtime.py'
  assert cmd[cmd.index('--onnx') + 1] == str(onnx)
  assert cmd[cmd.index('--camera-resolutions') + 1:] == ['1928x1208', '1344x760']
  assert env['DEV'] == 'QCOM' and env['FLOAT16'] == '1'
  assert (tmp_path / 'driving_tinygrad.pkl').read_bytes() == b'pkl'


def test_cheddar_compile_without_runtime_fails_the_install(monkeypatch, tmp_path):
  (tmp_path / 'driving_supercombo.onnx').write_bytes(b'cheddar')
  monkeypatch.setattr(installer, '_is_cheddar', lambda path: True)
  monkeypatch.setattr(installer, '_run', _fake_run([], make_runtime=False))
  with pytest.raises(installer.InstallError):
    installer._compile_supercombo(tmp_path, {})


def test_cheddar_is_recognized_only_by_its_exact_onnx_hash(tmp_path):
  onnx = tmp_path / 'driving_supercombo.onnx'
  onnx.write_bytes(b'not cheddar')
  assert hashlib.sha256(b'not cheddar').hexdigest() != MODEL_SHA256
  assert installer._is_cheddar(onnx) is False


def test_only_the_exact_cheddar_entry_passes_the_newer_selector_gate():
  from carrot.model_selector import manifest
  cheddar = {'id': 'Cheddar', 'minimum_selector_version': 5,
             'files': {'driving_supercombo.onnx': {'size': 69671018, 'sha256': MODEL_SHA256}}}
  assert manifest._pinned_runtime_model(cheddar)
  other = {'id': 'Smoked Cheddar', 'minimum_selector_version': 5,
           'files': {'driving_supercombo.onnx': {'size': 69700577, 'sha256': '1fde7dcb' + '0' * 56}}}
  assert not manifest._pinned_runtime_model(other)
  extra = {'files': {'driving_supercombo.onnx': {'sha256': MODEL_SHA256}, 'driving_vision.onnx': {'sha256': MODEL_SHA256}}}
  assert not manifest._pinned_runtime_model(extra)
  assert not manifest._pinned_runtime_model({'files': None})
