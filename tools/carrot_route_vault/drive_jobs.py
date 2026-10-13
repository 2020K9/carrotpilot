"""Bounded background drive-analysis jobs; callers enforce the route capability first."""
from __future__ import annotations

import asyncio
import gzip
import hashlib
import json
from pathlib import Path
import secrets
import sys

from aiohttp import web

try:
  from . import drive_analysis
except ImportError:
  import drive_analysis


# Unapproved defaults: pending jobs, subprocess limit and cache size.
MAX_PENDING = 4
TIMEOUT_S = 120
CACHE_MAX_BYTES = 256 * 1024**2


class DriveJobs:
  def __init__(self, cache: Path, gate: asyncio.Semaphore | None = None):
    self.cache = cache
    self.jobs: dict[str, asyncio.Task] = {}
    self.gate = gate if gate is not None else asyncio.Semaphore(1)
    self.version = None
    self.timeout = TIMEOUT_S

  async def close(self, _app):
    for job in self.jobs.values():
      job.cancel()
    await asyncio.gather(*self.jobs.values(), return_exceptions=True)

  async def response(self, source: Path):
    if self.version is None:
      self.version = await asyncio.to_thread(drive_analysis.source_version)
    stat = await asyncio.to_thread(source.stat)
    key = hashlib.sha256(f"{source}\0{stat.st_size}\0{stat.st_mtime_ns}\0{self.version}".encode()).hexdigest()
    output = self.cache / f"{key}.json.gz"
    if output.is_file():
      self.jobs.pop(key, None)
      return web.FileResponse(output, headers={"Content-Type": "application/json", "Content-Encoding": "gzip", "Cache-Control": "private, no-store"})
    job = self.jobs.get(key)
    if job is not None and job.done():
      self.jobs.pop(key)
      if job.cancelled() or job.exception():
        return web.json_response({"status": "error", "message": "주행 분석에 실패했습니다. 로그 형식과 서버 분석 환경을 확인해 주세요."}, status=422)
    if job is None:
      for other_key, other_job in list(self.jobs.items()):
        if other_job.done():
          if not other_job.cancelled():
            other_job.exception()
          self.jobs.pop(other_key)
      if len(self.jobs) >= MAX_PENDING:
        return web.json_response({"status": "busy"}, status=503, headers={"Retry-After": "5"})
      self.jobs[key] = asyncio.create_task(self.prepare(source, output))
    return web.json_response({"status": "preparing"}, status=202, headers={"Retry-After": "2", "Cache-Control": "no-store"})

  async def prepare(self, source, output):
    async with self.gate:
      self.cache.mkdir(parents=True, exist_ok=True)
      temporary = self.cache / f".{secrets.token_hex(12)}.json"
      compressed = temporary.with_suffix(".gz")
      process = None
      try:
        process = await asyncio.create_subprocess_exec(
          sys.executable, str(Path(__file__).with_name("drive_analysis.py")), str(source), str(temporary),
          stdout=asyncio.subprocess.DEVNULL, stderr=asyncio.subprocess.DEVNULL,
        )
        await asyncio.wait_for(process.wait(), timeout=self.timeout)
        if process.returncode:
          raise RuntimeError("drive analysis failed")
        await asyncio.to_thread(self.publish, temporary, compressed, output)
      finally:
        if process is not None and process.returncode is None:
          process.kill()
          await process.wait()
        temporary.unlink(missing_ok=True)
        compressed.unlink(missing_ok=True)

  def publish(self, temporary, compressed, output):
    payload = json.loads(temporary.read_text(encoding="utf-8"))
    if payload.get("schemaVersion") != 1 or payload.get("kind") != "carrotDrive" or not payload.get("series", {}).get("t"):
      raise ValueError("invalid drive analysis")
    with gzip.open(compressed, "wb") as stream:
      stream.write(temporary.read_bytes())
    compressed.replace(output)
    entries = sorted(self.cache.glob("*.json.gz"), key=lambda item: item.stat().st_mtime, reverse=True)
    total = 0
    for entry in entries:
      total += entry.stat().st_size
      if total > CACHE_MAX_BYTES and entry != output:
        entry.unlink(missing_ok=True)
