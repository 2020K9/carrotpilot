#!/usr/bin/env python3
import os
import subprocess
from pathlib import Path

# NOTE: Do NOT import anything here that needs be built (e.g. params)
from openpilot.common.basedir import BASEDIR
from openpilot.common.scons_cache_lock import acquire_cache_lock
from openpilot.common.spinner import Spinner
from openpilot.common.text_window import TextWindow
from openpilot.common.swaglog import cloudlog, add_file_handler
from openpilot.system.hardware import HARDWARE, AGNOS
from openpilot.system.version import get_build_metadata

MAX_CACHE_SIZE = 4e9 if "CI" in os.environ else 2e9
CACHE_DIR = Path(os.environ.get("SCONS_CACHE_DIR") or ("/data/scons_cache" if AGNOS else "/tmp/scons_cache"))

TOTAL_SCONS_NODES = 2705
MAX_BUILD_PROGRESS = 100


def build(spinner: Spinner, dirty: bool = False, minimal: bool = False) -> None:
  env = os.environ.copy()
  env['SCONS_PROGRESS'] = "1"
  env['PYTHONUNBUFFERED'] = "1"
  nproc = os.cpu_count()
  if nproc is None:
    nproc = 2

  extra_args = ["--minimal"] if minimal else []

  if AGNOS:
    HARDWARE.set_power_save(False)
    os.sched_setaffinity(0, range(8))  # ensure we can use the isolcpus cores

  # building with all cores can result in using too
  # much memory, so retry with less parallelism
  compile_output: list[bytes] = []
  for n in (nproc, nproc/2, 1):
    compile_output.clear()
    scons: subprocess.Popen = subprocess.Popen(["scons", f"-j{int(n)}", "--cache-populate", *extra_args], cwd=BASEDIR, env=env,
                                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    assert scons.stdout is not None

    # Read progress and build output from scons and update spinner.
    while scons.poll() is None:
      try:
        line = scons.stdout.readline()
        if line is None:
          continue
        line = line.rstrip()

        prefix = b'progress: '
        if line.startswith(prefix):
          i = int(line[len(prefix):])
          spinner.update_progress(MAX_BUILD_PROGRESS * min(1., i / TOTAL_SCONS_NODES), 100.)
        elif len(line):
          compile_output.append(line)
          line_text = line.decode('utf8', 'replace')
          print(line_text)
          spinner.update(line_text)
      except Exception:
        pass

    if scons.returncode == 0:
      break
    compile_output += scons.stdout.read().split(b'\n')
    if is_cache_lock_failure(compile_output) or any(b'SCons cache is in use:' in line for line in compile_output):
      message = ("SCons cache lock failed; cache preserved. Reboot to let the launcher repair "
                 "/data/scons_cache, or check for another running build.")
      print(message)
      spinner.update(message)
      compile_output.append(message.encode('utf8'))
      break

  if scons.returncode != 0:
    # Read remaining output
    if scons.stdout is not None:
      compile_output += scons.stdout.read().split(b'\n')

    # Build failed log errors
    error_s = b"\n".join(compile_output).decode('utf8', 'replace')
    add_file_handler(cloudlog)
    cloudlog.error("scons build failed\n" + error_s)

    # Show TextWindow
    spinner.close()
    if not os.getenv("CI") and os.getenv("CARROT_STARTUP_RECOVERY") != "1":
      with TextWindow("openpilot failed to build\n \n" + error_s) as t:
        t.wait_for_exit()
    exit(1)

  # enforce max cache size only while no other patched SCons process uses it
  prune_cache(CACHE_DIR)


def prunable_cache_files(cache_dir: Path) -> list[Path]:
  # Only cached build results, which SCons keeps in two-character bucket
  # directories. The top level holds the SCons config (and its lock) and
  # carrot's Params stamp; pruning those by age breaks or slows the next boot.
  if not cache_dir.is_dir():
    return []
  return [f for d in cache_dir.iterdir() if d.is_dir() and len(d.name) == 2 for f in d.iterdir() if f.is_file()]


def is_cache_lock_failure(output: list[bytes]) -> bool:
  return any(b'SConsLockFailure' in line for line in output)


def prune_cache(cache_dir: Path) -> None:
  # The build just finished, but another process may already have started.
  try:
    guard = acquire_cache_lock(cache_dir)
  except (OSError, RuntimeError) as exc:
    print(f"Skipping cache pruning: {exc}")
    return
  with guard:
    cache_files = prunable_cache_files(cache_dir)
    cache_files.sort(key=lambda f: f.stat().st_mtime)
    cache_size = sum(f.stat().st_size for f in cache_files)
    for f in cache_files:
      if cache_size < MAX_CACHE_SIZE:
        break
      cache_size -= f.stat().st_size
      f.unlink()


if __name__ == "__main__":
  with Spinner() as spinner:
    spinner.update_progress(0, 100)
    build_metadata = get_build_metadata()
    build(spinner, build_metadata.openpilot.is_dirty, minimal = AGNOS)
