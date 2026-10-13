import asyncio
import gzip
import json
import os
import threading
from pathlib import Path
from urllib.parse import urlsplit

from aiohttp import web
from aiohttp.test_utils import TestClient, TestServer

from .. import drive_jobs as drive_jobs_module
from ..drive_jobs import DriveJobs
from ..radar import RadarJobs
from ..server import ROUTE_VIEWER_KEY, create_app
from .test_viewer import DIRECTORY, OTHER_ROUTE, ROUTE, admin_headers, create_share, seed_route, viewer_config


def record_drive_calls(monkeypatch, status=202):
  calls = []

  async def response(_self, source):
    calls.append(source)
    return web.json_response({"status": "preparing"}, status=status)

  monkeypatch.setattr(DriveJobs, "response", response)
  return calls


def test_public_drive_scope_and_log_selection(tmp_path, monkeypatch):
  calls = record_drive_calls(monkeypatch)

  async def run():
    seed_route(tmp_path, ROUTE, (0, 1, 2))
    segments = tmp_path / "uploads/routes" / DIRECTORY
    (segments / f"{ROUTE}--1/rlog.zst").unlink()
    (segments / f"{ROUTE}--2/rlog.zst").unlink()
    (segments / f"{ROUTE}--2/qlog.zst").unlink()
    async with TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False))) as client:
      base = f"/routes/{DIRECTORY}/{ROUTE}--0"
      assert (await client.get(base + "/drive/1")).status == 404
      assert not calls
      assert (await client.get(base + "/drive/x")).status == 400
      assert (await client.get(base + "/drive/0")).status == 202
      assert calls[-1] == segments / f"{ROUTE}--0/rlog.zst"
      wide = f"/routes/{DIRECTORY}/{ROUTE}--0:3"
      only_qlog = await client.get(wide + "/drive/1")
      assert only_qlog.status == 404 and "rlog" in await only_qlog.text()
      assert (await client.get(wide + "/drive/2")).status == 404
      assert len(calls) == 1

  asyncio.run(run())


def test_share_drive_checks_token_every_request(tmp_path, monkeypatch):
  calls = record_drive_calls(monkeypatch, status=200)  # behaves like a cache hit

  async def run():
    seed_route(tmp_path, ROUTE, (0, 1))
    seed_route(tmp_path, OTHER_ROUTE, (0,))
    async with TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False))) as client:
      share = await create_share(client)
      other = await create_share(client, OTHER_ROUTE)
      shared = urlsplit(share["shareUrl"]).path
      assert (await client.get(shared + "/drive/1")).status == 200
      assert (await client.get(urlsplit(other["shareUrl"]).path + "/drive/1")).status == 404
      assert (await client.get("/s/notatoken/drive/0")).status == 404
      count = len(calls)
      assert (await client.post(f"/api/admin/shares/{share['id']}/revoke", headers=admin_headers(), json={})).status == 200
      assert (await client.get(shared + "/drive/1")).status == 404
      assert len(calls) == count

  asyncio.run(run())


def test_compare_page_and_asset_are_static(tmp_path):
  async def run():
    seed_route(tmp_path, ROUTE, (0,))
    async with TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False))) as client:
      response = await client.get("/compare", params={"a": "/routes/x/<script>MARK_A</script>", "ai": "0", "b": "MARK_B"})
      body = await response.text()
      assert response.status == 200
      assert "script-src 'self' 'unsafe-inline'" in response.headers["Content-Security-Policy"]
      assert response.headers["X-Robots-Tag"] == "noindex, nofollow, noarchive"
      assert "MARK_A" not in body and "MARK_B" not in body
      assert DIRECTORY not in body and ROUTE not in body
      assert '<script type="module" src="/assets/compare_view.js"></script>' in body
      asset = await client.get("/assets/compare_view.js")
      assert asset.status == 200 and asset.headers["Content-Type"].startswith("text/javascript")
      source = await asset.text()
      assert "http://" not in source and "https://" not in source and "import(" not in source
      for page in (f"/routes/{DIRECTORY}/{ROUTE}--0", urlsplit((await create_share(client))["shareUrl"]).path):
        text = await (await client.get(page)).text()
        assert "attachRadarReview(video)" in text
        for label in ("주행 분석", "비교 A로", "비교 B로", "A·B 비교 열기", "carrotCompare"):
          assert label in text

  asyncio.run(run())


class FakeProcess:
  """Stands in for asyncio subprocesses; records ordering and concurrency."""
  running = 0
  peak = 0

  def __init__(self, log, args, result=0, hang=False, release=None):
    self.log, self.args, self.result, self.hang = log, args, result, hang
    self.release = release or asyncio.Event()
    self.returncode = None
    self.killed = asyncio.Event()
    self.output = Path(args[-1] if "drive_analysis.py" in args[1] else args[4])
    self.output.write_text("partial")
    FakeProcess.running += 1
    FakeProcess.peak = max(FakeProcess.peak, FakeProcess.running)

  def kill(self):
    self.log.append(("kill", self.output.name))
    self.killed.set()

  async def wait(self):
    if self.returncode is not None:
      return self.returncode
    waiters = [asyncio.ensure_future(self.killed.wait())]
    if not self.hang:
      waiters.append(asyncio.ensure_future(self.release.wait()))
    try:
      await asyncio.wait(waiters, return_when=asyncio.FIRST_COMPLETED)
    finally:
      for waiter in waiters:
        waiter.cancel()
    FakeProcess.running -= 1
    if self.killed.is_set():
      self.returncode = -9
      self.log.append(("waited", self.output.name))
      return self.returncode
    if self.result == 0:
      if "drive_analysis.py" in self.args[1]:
        payload = {"schemaVersion": 1, "kind": "carrotDrive", "series": {"t": [0.0]}}
      else:
        payload = {"schemaVersion": 1, "frames": [{}]}
      self.output.write_text(json.dumps(payload))
    self.returncode = self.result
    return self.returncode


def install_fake_processes(monkeypatch, plan=None):
  FakeProcess.running = FakeProcess.peak = 0
  log, processes = [], []
  plan = plan or (lambda args: {})

  async def create(*args, **_kwargs):
    log.append(("spawn", Path(args[-1] if "drive_analysis.py" in args[1] else args[4]).name,
                sorted(p.name for p in Path(args[-1] if "drive_analysis.py" in args[1] else args[4]).parent.glob(".*"))))
    process = FakeProcess(log, args, **plan(args))
    processes.append(process)
    return process

  monkeypatch.setattr(asyncio, "create_subprocess_exec", create)
  return log, processes


def test_radar_and_drive_share_one_running_slot(tmp_path, monkeypatch):
  from openpilot.selfdrive.carrot.radar.tools import radar_web_export
  monkeypatch.setattr(radar_web_export, "source_version", lambda: "radar-test")
  log, processes = install_fake_processes(monkeypatch)

  async def run():
    seed_route(tmp_path, ROUTE, (0,))
    async with TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False))) as client:
      viewer = client.server.app[ROUTE_VIEWER_KEY]
      assert viewer.radar_jobs.gate is viewer.drive_jobs.gate
      base = f"/routes/{DIRECTORY}/{ROUTE}--0"
      assert (await client.get(base + "/radar/0")).status == 202
      assert (await client.get(base + "/drive/0")).status == 202
      for _ in range(20):
        await asyncio.sleep(0)
      assert len(processes) == 1 and FakeProcess.peak == 1
      processes[0].release.set()
      await until(lambda: len(processes) == 2)
      processes[1].release.set()
      await asyncio.gather(*viewer.radar_jobs.jobs.values(), *viewer.drive_jobs.jobs.values())
      assert FakeProcess.peak == 1
      cached = await client.get(base + "/drive/0")
      # server.py middleware rewrites every response to "no-store" (DriveJobs itself sets "private, no-store")
      assert cached.status == 200 and "no-store" in cached.headers["Cache-Control"]
      assert json.loads(await cached.read())["kind"] == "carrotDrive"

  asyncio.run(run())


async def until(condition, attempts=200):
  for _ in range(attempts):
    if condition():
      return
    await asyncio.sleep(0.01)
  raise AssertionError("condition not reached")


async def finish(jobs, processes, count):
  """Release processes one by one until `count` have been spawned and every job is done."""
  for n in range(count):
    await until(lambda: len(processes) > n)
    processes[n].release.set()
  await asyncio.gather(*jobs.jobs.values(), return_exceptions=True)


def test_drive_jobs_limits_dedup_failure_and_cache(tmp_path, monkeypatch):
  results = {}
  log, processes = install_fake_processes(monkeypatch, lambda args: {"result": results.get(Path(args[2]).name, 0)})
  monkeypatch.setattr(drive_jobs_module.drive_analysis, "source_version", lambda: "drive-test")

  async def run():
    jobs = DriveJobs(tmp_path / "cache")
    sources = []
    for i in range(6):
      source = tmp_path / f"rlog{i}.zst"
      source.write_bytes(b"x" * (i + 1))
      sources.append(source)
    first = await jobs.response(sources[0])
    assert first.status == 202 and first.headers["Retry-After"] == "2"
    assert (await jobs.response(sources[0])).status == 202  # duplicate request joins the same job
    for source in sources[1:4]:
      assert (await jobs.response(source)).status == 202
    await asyncio.sleep(0.05)
    assert len(processes) == 1 and len(jobs.jobs) == 4  # one running + three waiting
    busy = await jobs.response(sources[4])
    assert busy.status == 503 and busy.headers["Retry-After"] == "5"
    await finish(jobs, processes, 4)
    assert len(processes) == 4  # the duplicate did not spawn a second process
    assert (await jobs.response(sources[4])).status == 202  # finished jobs no longer count
    await finish(jobs, processes, 5)
    hit = await jobs.response(sources[0])
    assert hit.status == 200 and hit.headers["Content-Encoding"] == "gzip"
    assert hit.headers["Cache-Control"] == "private, no-store"

    results[sources[5].name] = 2
    assert (await jobs.response(sources[5])).status == 202
    await finish(jobs, processes, 6)
    assert (await jobs.response(sources[5])).status == 422
    results[sources[5].name] = 0
    assert (await jobs.response(sources[5])).status == 202  # retried after the failure was reported
    await finish(jobs, processes, 7)
    assert (await jobs.response(sources[5])).status == 200

    os.utime(sources[0], ns=(1, 1))
    assert (await jobs.response(sources[0])).status == 202  # changed mtime -> new key
    await finish(jobs, processes, 8)
    assert len(processes) == 8

  asyncio.run(run())


def test_drive_cache_limit_keeps_newest(tmp_path, monkeypatch):
  monkeypatch.setattr(drive_jobs_module, "CACHE_MAX_BYTES", 1)
  jobs = DriveJobs(tmp_path / "cache")
  jobs.cache.mkdir()
  old = jobs.cache / "old.json.gz"
  old.write_bytes(gzip.compress(b"{}"))
  os.utime(old, ns=(1, 1))
  temporary = jobs.cache / ".t.json"
  temporary.write_text(json.dumps({"schemaVersion": 1, "kind": "carrotDrive", "series": {"t": [0]}}))
  jobs.publish(temporary, temporary.with_suffix(".gz"), jobs.cache / "new.json.gz")
  assert not old.exists() and (jobs.cache / "new.json.gz").exists()
  for bad in ({"schemaVersion": 2, "kind": "carrotDrive", "series": {"t": [0]}},
              {"schemaVersion": 1, "kind": "other", "series": {"t": [0]}},
              {"schemaVersion": 1, "kind": "carrotDrive", "series": {"t": []}}):
    temporary.write_text(json.dumps(bad))
    try:
      jobs.publish(temporary, temporary.with_suffix(".gz"), jobs.cache / "bad.json.gz")
    except ValueError:
      pass
    else:
      raise AssertionError(bad)
  assert not (jobs.cache / "bad.json.gz").exists()


def test_drive_timeout_kills_waits_and_cleans_before_releasing_slot(tmp_path, monkeypatch):
  log, processes = install_fake_processes(monkeypatch, lambda args: {"hang": Path(args[2]).name == "hang.zst"})
  monkeypatch.setattr(drive_jobs_module.drive_analysis, "source_version", lambda: "drive-test")

  async def run():
    gate = asyncio.Semaphore(1)
    jobs = DriveJobs(tmp_path / "cache", gate)
    jobs.timeout = 0.05
    hang, other = tmp_path / "hang.zst", tmp_path / "other.zst"
    hang.write_bytes(b"h")
    other.write_bytes(b"o")
    assert (await jobs.response(hang)).status == 202
    assert (await jobs.response(other)).status == 202
    await until(lambda: len(processes) == 2)
    first = processes[0]
    events = [entry[0] for entry in log]
    assert events == ["spawn", "kill", "waited", "spawn"]
    assert first.returncode == -9
    assert log[3][2] == []  # no temporary file left in the cache when the next job got the slot
    assert not first.output.exists() and not first.output.with_suffix(".gz").exists()
    assert (await jobs.response(hang)).status == 422
    processes[1].release.set()
    await asyncio.gather(*jobs.jobs.values())

  asyncio.run(run())


def test_cleanup_cancels_waiting_and_kills_running(tmp_path, monkeypatch):
  log, processes = install_fake_processes(monkeypatch)
  monkeypatch.setattr(drive_jobs_module.drive_analysis, "source_version", lambda: "drive-test")

  async def run():
    seed_route(tmp_path, ROUTE, (0, 1))
    client = TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False)))
    await client.start_server()
    app = client.server.app
    viewer = app[ROUTE_VIEWER_KEY]
    assert viewer.drive_jobs.close in app.on_cleanup
    base = f"/routes/{DIRECTORY}/{ROUTE}--0:2"
    assert (await client.get(base + "/drive/0")).status == 202
    assert (await client.get(base + "/drive/1")).status == 202
    for _ in range(20):
      await asyncio.sleep(0)
    assert len(processes) == 1
    tasks = list(viewer.drive_jobs.jobs.values())
    await client.close()
    assert len(processes) == 1  # the waiting job never spawned
    assert [entry[0] for entry in log] == ["spawn", "kill", "waited"]
    assert processes[0].returncode == -9
    assert all(task.done() for task in tasks)
    cache = viewer.drive_jobs.cache
    assert not [p for p in cache.glob(".*")]
    assert not [t for t in asyncio.all_tasks() if t is not asyncio.current_task() and not t.done()]

  asyncio.run(run())


class StalledPublish:
  """Replaces publish: writes the compressed file, then blocks its worker thread until released."""

  def __init__(self, monkeypatch, jobs_class, fail=False):
    self.started, self.release = threading.Event(), threading.Event()
    self.calls, self.events, self.fail = 0, [], fail
    original = jobs_class.publish

    def publish(jobs, temporary, compressed, output):
      self.calls += 1
      compressed.write_bytes(b"partial")
      self.started.set()
      self.release.wait(10)
      if self.fail:
        self.events.append("publish-failed")
        raise ValueError("invalid analysis")
      original(jobs, temporary, compressed, output)
      self.events.append("publish-done")

    monkeypatch.setattr(jobs_class, "publish", publish)


def run_cancel_during_publish(tmp_path, monkeypatch, jobs_class, extra_cancels=0, fail=False):
  _log, processes = install_fake_processes(monkeypatch)
  stalled = StalledPublish(monkeypatch, jobs_class, fail)

  async def run():
    gate = asyncio.Semaphore(1)
    jobs = jobs_class(tmp_path / "cache", gate)
    jobs.version = "test"
    source = tmp_path / "rlog.zst"
    source.write_bytes(b"x")
    response = await (jobs.response(source) if jobs_class is DriveJobs else jobs.response(source, "auto", 0))
    assert response.status == 202
    task = next(iter(jobs.jobs.values()))
    await until(lambda: len(processes) == 1)
    processes[0].release.set()
    await until(stalled.started.is_set)
    acquired = []

    async def next_job():
      async with gate:
        acquired.append(sorted(p.name for p in jobs.cache.glob(".*")))
        stalled.events.append("next-acquired")

    waiter = asyncio.create_task(next_job())
    closer = asyncio.create_task(jobs.close(None))
    for _ in range(extra_cancels):
      await asyncio.sleep(0.02)
      task.cancel()
    await asyncio.sleep(0.2)
    assert not closer.done() and not task.done()
    assert gate.locked() and not acquired
    assert sorted(p.suffix for p in jobs.cache.glob(".*")) == [".gz", ".json"]  # not cleaned while publishing
    stalled.release.set()
    await asyncio.wait_for(closer, 5)
    await asyncio.wait_for(waiter, 5)
    assert task.cancelled()
    assert stalled.calls == 1
    assert stalled.events == ["publish-failed" if fail else "publish-done", "next-acquired"]
    assert acquired == [[]]  # the next job got the slot only after the temporary files were removed
    assert not gate.locked()
    assert not list(jobs.cache.glob(".*"))
    assert bool(list(jobs.cache.glob("*.json.gz"))) is not fail
    assert len(processes) == 1

  asyncio.run(run())


def test_drive_cancel_during_publish_waits_before_cleanup_and_release(tmp_path, monkeypatch):
  run_cancel_during_publish(tmp_path, monkeypatch, DriveJobs)


def test_radar_cancel_during_publish_waits_before_cleanup_and_release(tmp_path, monkeypatch):
  run_cancel_during_publish(tmp_path, monkeypatch, RadarJobs)


def test_drive_double_cancel_during_publish_keeps_cleanup_order(tmp_path, monkeypatch):
  run_cancel_during_publish(tmp_path, monkeypatch, DriveJobs, extra_cancels=2)


def test_radar_double_cancel_during_publish_keeps_cleanup_order(tmp_path, monkeypatch):
  run_cancel_during_publish(tmp_path, monkeypatch, RadarJobs, extra_cancels=2)


def test_drive_publish_exception_during_cancel_keeps_cleanup_order(tmp_path, monkeypatch):
  run_cancel_during_publish(tmp_path, monkeypatch, DriveJobs, fail=True)


def test_radar_publish_exception_during_cancel_keeps_cleanup_order(tmp_path, monkeypatch):
  run_cancel_during_publish(tmp_path, monkeypatch, RadarJobs, fail=True)


def test_drive_double_cancel_during_kill_still_waits_process(tmp_path, monkeypatch):
  log, processes = [], []

  class SlowExitProcess(FakeProcess):
    """Exits only after `exit` is set, even once killed."""

    def __init__(self, *args, **kwargs):
      super().__init__(*args, **kwargs)
      self.exit = asyncio.Event()

    async def wait(self):
      if self.returncode is not None:
        return self.returncode
      await self.killed.wait()
      await self.exit.wait()
      FakeProcess.running -= 1
      self.returncode = -9
      self.log.append(("waited", self.output.name))
      return self.returncode

  async def create(*args, **_kwargs):
    log.append(("spawn", Path(args[-1]).name))
    process = SlowExitProcess(log, args, hang=True)
    processes.append(process)
    return process

  monkeypatch.setattr(asyncio, "create_subprocess_exec", create)

  async def run():
    gate = asyncio.Semaphore(1)
    jobs = DriveJobs(tmp_path / "cache", gate)
    jobs.version = "test"
    source = tmp_path / "rlog.zst"
    source.write_bytes(b"x")
    assert (await jobs.response(source)).status == 202
    task = next(iter(jobs.jobs.values()))
    await until(lambda: len(processes) == 1)

    async def next_job():
      async with gate:
        log.append(("acquired", sorted(p.name for p in jobs.cache.glob(".*"))))

    waiter = asyncio.create_task(next_job())
    task.cancel()
    await until(processes[0].killed.is_set)
    task.cancel()
    await asyncio.sleep(0.1)
    assert not task.done() and gate.locked()
    assert [entry[0] for entry in log] == ["spawn", "kill"]
    assert list(jobs.cache.glob(".*"))  # the temporary file stays until the process has exited
    processes[0].exit.set()
    await asyncio.wait_for(waiter, 5)
    assert task.cancelled()
    assert [entry[0] for entry in log] == ["spawn", "kill", "waited", "acquired"]
    assert log[-1][1] == []
    assert not gate.locked()

  asyncio.run(run())


def test_cleanup_radar_running_and_drive_waiting_never_spawns_drive(tmp_path, monkeypatch):
  from openpilot.selfdrive.carrot.radar.tools import radar_web_export
  monkeypatch.setattr(radar_web_export, "source_version", lambda: "radar-test")
  monkeypatch.setattr(drive_jobs_module.drive_analysis, "source_version", lambda: "drive-test")
  log, processes = install_fake_processes(monkeypatch)

  async def run():
    seed_route(tmp_path, ROUTE, (0,))
    client = TestClient(TestServer(create_app(viewer_config(tmp_path), start_cleanup=False)))
    await client.start_server()
    viewer = client.server.app[ROUTE_VIEWER_KEY]
    base = f"/routes/{DIRECTORY}/{ROUTE}--0"
    assert (await client.get(base + "/radar/0")).status == 202
    await until(lambda: len(processes) == 1)
    assert (await client.get(base + "/drive/0")).status == 202
    for _ in range(20):
      await asyncio.sleep(0)
    tasks = [*viewer.radar_jobs.jobs.values(), *viewer.drive_jobs.jobs.values()]
    assert len(tasks) == 2 and len(processes) == 1
    await client.close()
    assert len(processes) == 1
    assert not any("drive_analysis.py" in p.args[1] for p in processes)  # the waiting drive job never spawned
    assert [entry[0] for entry in log] == ["spawn", "kill", "waited"]
    assert processes[0].returncode == -9
    assert all(task.done() for task in tasks)
    assert not viewer.radar_jobs.gate.locked()
    assert not list(viewer.radar_jobs.cache.glob(".*"))
    assert not [t for t in asyncio.all_tasks() if t is not asyncio.current_task() and not t.done()]

  asyncio.run(run())
