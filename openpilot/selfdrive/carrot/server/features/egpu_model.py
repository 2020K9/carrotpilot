"""Jetlink (Jetson) status and the shared external model library.

comma 4 has no eGPU path; the route names keep their old /api/egpu prefix so
the web card and the Jetson model library keep working unchanged.
"""
from __future__ import annotations

import asyncio
from contextlib import suppress

from aiohttp import web
from openpilot.common.jetlink_status import diagnostics as jetlink_diagnostics

from ..services.params import HAS_PARAMS, Params


async def api_status(_request: web.Request) -> web.Response:
  from openpilot.selfdrive.modeld.model_library import status
  return web.json_response({'ok': True, 'available': False, 'jetlink': jetlink_diagnostics(),
                            'library': await asyncio.to_thread(status)})


async def api_library(request: web.Request) -> web.Response:
  from openpilot.common import big_model_catalog as catalog
  from openpilot.selfdrive.modeld import model_library as library
  try:
    body = await request.json()
    if not isinstance(body, dict) or set(body) != {'action', 'model'}:
      raise ValueError('invalid model action')
    action, key = body['action'], body['model']
    if action != 'select' or not isinstance(key, str) or key not in catalog.available():
      raise ValueError('unknown model action')
    await asyncio.to_thread(catalog.select, key)
    return web.json_response({'ok': True, 'library': await asyncio.to_thread(library.status)})
  except (ValueError, OSError) as exc:
    return web.json_response({'ok': False, 'error': str(exc)}, status=409)


def maintain_library_once():
  from openpilot.selfdrive.modeld.model_library import reconcile_retired
  if HAS_PARAMS and Params is not None:
    return reconcile_retired(Params())
  return []


async def library_maintenance(app):
  async def run():
    while True:
      try:
        await asyncio.to_thread(maintain_library_once)
      except (OSError, ValueError):
        pass  # retry on the next offroad pass
      await asyncio.sleep(30)
  task = asyncio.create_task(run())
  try:
    yield
  finally:
    task.cancel()
    with suppress(asyncio.CancelledError):
      await task


def register(app: web.Application) -> None:
  app.cleanup_ctx.append(library_maintenance)
  app.router.add_get("/api/egpu/model", api_status)
  app.router.add_post("/api/egpu/model/library", api_library)
