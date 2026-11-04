"""Restored from deleted test_egpu_model.py (4562d6a3): shared (Jetson) model library API tests.

The eGPU-only fixture patches (model_cache_dir, active_manifest, active_model_compiled) are dropped
because those functions were removed; test bodies, inputs and expectations are verbatim.
"""
import asyncio
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace

import pytest


@pytest.fixture
def feature(monkeypatch, tmp_path):
  spec = importlib.util.spec_from_file_location(
    "openpilot.selfdrive.carrot.server.features.egpu_model_test",
    Path(__file__).parents[1] / "features/egpu_model.py",
  )
  module = importlib.util.module_from_spec(spec)
  spec.loader.exec_module(module)
  return module


def test_library_only_allows_curated_selection_even_offroad(feature, monkeypatch, tmp_path):
  from openpilot.common import big_model_catalog as C
  monkeypatch.setenv('CARROT_BIG_MODEL_DIR', str(tmp_path))
  monkeypatch.setattr(C, 'boot_id', lambda: 'same-boot')
  C.freeze_boot()
  monkeypatch.setattr(feature, 'HAS_PARAMS', True)
  monkeypatch.setattr(feature, 'Params', lambda: SimpleNamespace(get_bool=lambda key: key == 'IsOnroad'))
  class Request:
    def __init__(self, action, model): self.body = {'action': action, 'model': model}
    async def json(self): return self.body
  response = asyncio.run(feature.api_library(Request('select', 'flip-flop')))
  assert response.status == 200
  value = json.loads(response.body)['library']
  assert value['selected'] == 'flip-flop' and value['effective'] == 'n16' and value['reboot_required']
  assert asyncio.run(feature.api_library(Request('delete', 'md-v3'))).status == 409
  monkeypatch.setattr(feature, 'Params', lambda: SimpleNamespace(get_bool=lambda key: key == 'IsOffroad'))
  assert asyncio.run(feature.api_library(Request('delete', 'n16'))).status == 409
  assert asyncio.run(feature.api_library(Request('delete', 'md-v3'))).status == 409
  assert asyncio.run(feature.api_library(Request('restore', 'md-v3'))).status == 409
  assert asyncio.run(feature.api_library(Request('select', '../../anything'))).status == 409


def test_library_maintenance_runs_off_request_thread_and_stops(feature, monkeypatch):
  from aiohttp import web
  calls = []
  monkeypatch.setattr(feature, 'maintain_library_once', lambda: calls.append('run'))
  app = web.Application()
  feature.register(app)
  assert feature.library_maintenance in app.cleanup_ctx
  async def run():
    context = feature.library_maintenance(app)
    await anext(context)
    for _ in range(100):
      if calls:
        break
      await asyncio.sleep(.01)
    assert calls == ['run']
    await context.aclose()
    await asyncio.sleep(.01)
    assert calls == ['run']
  asyncio.run(run())


def test_library_retired_entry_cannot_be_selected(feature, monkeypatch, tmp_path):
  from openpilot.common import big_model_catalog as C
  monkeypatch.setenv('CARROT_BIG_MODEL_DIR', str(tmp_path))
  models = C.catalog()
  models['md-v3']['retired'] = True
  monkeypatch.setattr(C, 'catalog', lambda: models)
  class Request:
    async def json(self): return {'action': 'select', 'model': 'md-v3'}
  assert asyncio.run(feature.api_library(Request())).status == 409
  assert not (tmp_path / 'selection.json').exists()
