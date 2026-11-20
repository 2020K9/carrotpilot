"""Private X display for the HUD/cluster render tests.

`xvfb-run -a` picks a display number by looking for free /tmp/.X<n>-lock files
and only then starts Xvfb. Two runs started at the same moment (several test
jobs on one machine) can pick the same number: the second Xvfb exits, and the
second run silently draws on the first run's server. When the first run
ends it kills that server, and every GL window the second run opens afterwards
fails with "X11: Failed to open display :N" -> "HUD graphics window
initialization failed". Which test hits that gap changes from run to run.

Second cause, also seen with a single run: an X server resets itself when its
last client disconnects. The render tests open and close a window one after
another, so the next test's XOpenDisplay can land inside that reset and fail
the same way (measured on the worker: 22 of 3000 open/close cycles failed
without -noreset, 0 of 3000 with it; worse under CPU load).

So this session starts its own Xvfb with -noreset and `-displayfd` (the server itself
picks and holds a free number, no race) and points DISPLAY at it. The server
dies with the pytest process (PR_SET_PDEATHSIG), so a timeout-killed run does
not leave a stray Xvfb holding a display number either.

Set K9_TEST_KEEP_DISPLAY=1 to draw on the caller's DISPLAY instead.
"""
import ctypes
import os
import select
import shutil
import signal
import subprocess

_XVFB = None


def _die_with_parent():
  try:
    ctypes.CDLL("libc.so.6", use_errno=True).prctl(1, signal.SIGKILL)  # PR_SET_PDEATHSIG
  except OSError:
    pass


def _start_private_xvfb():
  if os.environ.get("K9_TEST_KEEP_DISPLAY") == "1" or shutil.which("Xvfb") is None:
    return None
  read_fd, write_fd = os.pipe()
  proc = subprocess.Popen(
    ["Xvfb", "-displayfd", str(write_fd), "-screen", "0", "2400x1200x24", "-nolisten", "tcp", "-noreset"],
    pass_fds=(write_fd,), stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    preexec_fn=_die_with_parent,
  )
  os.close(write_fd)
  number = b""
  try:
    while not number.endswith(b"\n"):
      ready, _, _ = select.select([read_fd], [], [], 30.0)
      if not ready:
        break
      chunk = os.read(read_fd, 64)
      if not chunk:
        break
      number += chunk
  finally:
    os.close(read_fd)
  if not number.strip().isdigit():
    proc.kill()
    proc.wait()
    raise RuntimeError("private Xvfb did not report a display number")
  os.environ["DISPLAY"] = ":" + number.strip().decode()
  os.environ.pop("XAUTHORITY", None)
  return proc


def pytest_configure(config):
  global _XVFB
  if _XVFB is None:
    _XVFB = _start_private_xvfb()


def pytest_unconfigure(config):
  global _XVFB
  if _XVFB is not None:
    _XVFB.terminate()
    try:
      _XVFB.wait(timeout=10)
    except subprocess.TimeoutExpired:
      _XVFB.kill()
      _XVFB.wait()
    _XVFB = None
