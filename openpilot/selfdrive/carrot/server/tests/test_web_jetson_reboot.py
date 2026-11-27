"""Web Jetson controls removed by owner request, 2026-11-27."""
import asyncio
from pathlib import Path
import pytest
from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer
import importlib.util
import sys
import types

# Load the real system feature without initializing unrelated native msgq routes.
def load_system():
  package = "openpilot.selfdrive.carrot.server.features"
  directory = Path(__file__).resolve().parents[1] / "features"
  namespace = types.ModuleType(package)
  namespace.__path__ = [str(directory)]
  spec = importlib.util.spec_from_file_location(package + ".system", directory / "system.py")
  module = importlib.util.module_from_spec(spec)
  with pytest.MonkeyPatch.context() as patch:
    patch.setitem(sys.modules, package, namespace)
    spec.loader.exec_module(module)
  return module

system = load_system()
from tools.jetlink import host_reboot, host_soft_reset
WEB = Path(__file__).parents[2] / "web"

@pytest.mark.parametrize("engaged", [False, True])
def test_removed_endpoints_cannot_submit_requests(monkeypatch, tmp_path, engaged):
  calls = []
  monkeypatch.setattr(system, "is_drive_engaged", lambda request: engaged)
  monkeypatch.setattr(host_reboot, "REQUEST_FILE", tmp_path / "reboot.json")
  monkeypatch.setattr(host_soft_reset, "SOFT_REQUEST_FILE", tmp_path / "reset.json")
  def forbidden(*args, **kwargs):
    calls.append(True)
    raise AssertionError("removed web feature must not submit")
  monkeypatch.setattr(host_reboot, "submit", forbidden)
  monkeypatch.setattr(host_soft_reset, "submit", forbidden)
  async def run():
    app = web.Application()
    system.register(app)
    app.cleanup_ctx.clear()
    async with TestClient(TestServer(app)) as client:
      for endpoint in ("jetson_reboot", "jetson_soft_reset"):
        for method in ("GET", "POST"):
          for suffix in ("", "_status"):
            response = await client.request(method, "/api/" + endpoint + suffix)
            assert response.status == 404
      if engaged:
        assert (await client.post("/api/reboot", json={})).status == 409
        assert (await client.post("/api/poweroff", json={})).status == 409
  asyncio.run(run())
  assert calls == []
  assert not host_reboot.REQUEST_FILE.exists()
  assert not host_soft_reset.SOFT_REQUEST_FILE.exists()


def test_web_buttons_and_handlers_removed_other_controls_remain():
  device = (WEB / "js/pages/setting_device.js").read_text()
  actions = (WEB / "js/pages/setting_device_actions.js").read_text()
  for removed in ("btnDeviceJetsonReboot", "btnDeviceJetsonSoftReset",
                  "bindJetsonReboot", "bindJetsonSoftReset",
                  "/api/jetson_reboot", "/api/jetson_soft_reset"):
    assert removed not in device + actions
  for retained in ("btnDeviceReboot", "btnDevicePoweroff", "btnDeviceRecalib"):
    assert retained in device and retained in actions


def test_other_system_routes_retained():
  app = web.Application()
  system.register(app)
  paths = {route.resource.canonical for route in app.router.routes()}
  assert {"/api/reboot", "/api/poweroff", "/api/recalibrate", "/api/device_network"} <= paths
  assert not any("jetson_reboot" in path or "jetson_soft_reset" in path for path in paths)
