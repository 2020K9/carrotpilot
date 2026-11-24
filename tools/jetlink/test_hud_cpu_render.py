"""The Jetson HUD draws with the CPU (Mesa llvmpipe), never the NVIDIA GPU (대감님 11/24 19:30 "hud cpu에서 쓰게끔해")."""
import os
from pathlib import Path
import subprocess
import sys
import textwrap

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import hud


def test_cpu_renderer_selects_mesa_software_gl():
  env = {}
  present = {hud.MESA_GLX[0], hud.MESA_SWRAST[0]}
  assert hud.use_cpu_renderer(env, present.__contains__) == 'cpu'
  assert env == {'__GLX_VENDOR_LIBRARY_NAME': 'mesa', 'LIBGL_ALWAYS_SOFTWARE': '1', 'GALLIUM_DRIVER': 'llvmpipe',
                 'LP_NUM_THREADS': '2'}
  assert hud.RENDERER['mode'] == 'cpu'


def test_missing_mesa_keeps_the_hud_running_and_says_gpu(capsys):
  env = {}
  assert hud.use_cpu_renderer(env, lambda path: False) == 'gpu'
  assert env == {} and 'CPU renderer unavailable' in capsys.readouterr().err


def test_cpu_renderer_is_chosen_before_any_graphics_import():
  src = (Path(hud.__file__)).read_text()
  main = src[src.index('def main():'):]
  assert main.index('use_cpu_renderer()') < main.index('import main as cluster')
  assert "'renderer': RENDERER.get('mode')" in main


@pytest.mark.skipif(not os.environ.get('DISPLAY'), reason='needs an X display (xvfb-run)')
def test_window_really_opens_on_llvmpipe():
  code = textwrap.dedent('''
    import ctypes, sys
    sys.path.insert(0, %r)
    import hud
    assert hud.use_cpu_renderer() == 'cpu'
    import pyray as rl
    rl.set_trace_log_level(rl.LOG_NONE)
    rl.set_config_flags(rl.FLAG_WINDOW_HIDDEN)
    rl.init_window(64, 64, 'cpu')
    gl = ctypes.CDLL('libGL.so.1')
    gl.glGetString.restype = ctypes.c_char_p
    print(gl.glGetString(0x1F01).decode())
    rl.close_window()
  ''' % str(Path(hud.__file__).resolve().parent))
  out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=120)
  assert out.returncode == 0, out.stderr[-2000:]
  assert 'llvmpipe' in out.stdout
