import ast
import os
from pathlib import Path
import subprocess
import sys

from openpilot.common.basedir import BASEDIR


def test_scons_rebuilds_model_when_serialization_helper_changes(tmp_path: Path):
  # Use the actual compiler dependency list with a tiny SCons build, avoiding
  # an ONNX/GPU compile while exercising SCons' dependency signatures.
  source = (Path(BASEDIR) / 'openpilot/selfdrive/modeld/SConscript').read_text(encoding='utf8')
  tree = ast.parse(source)
  deps = next(n for n in tree.body if isinstance(n, ast.Assign) and any(
    isinstance(t, ast.Name) and t.id == 'compile_modeld_script' for t in n.targets))
  namespace = {'File': lambda name: name.lstrip('#'), 'modeld_dir': 'openpilot/selfdrive/modeld'}
  exec(compile(ast.Module(body=[deps], type_ignores=[]), '<model dependencies>', 'exec'), namespace)
  for name in namespace['compile_modeld_script']:
    path = tmp_path / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text('initial')
  helpers_path = tmp_path / 'openpilot/selfdrive/modeld/helpers.py'
  helpers_path.write_text('initial')
  (tmp_path / 'SConstruct').write_text(
    'env = Environment(tools=[])\n' +
    'def compile_model(target, source, env):\n' +
    '  open(str(target[0]), "w").write("compiled")\n' +
    f'env.Command("model.chunkmanifest", {namespace["compile_modeld_script"]!r}, compile_model)\n')
  command = [sys.executable, '-m', 'SCons', '--max-drift=-1', 'model.chunkmanifest']
  def run(*args):
    result = subprocess.run(command + list(args), cwd=tmp_path, capture_output=True, text=True, check=False)
    assert result.returncode in (0, 1), result.stdout + result.stderr
    return result.returncode
  assert run() == 0
  assert run('-q') == 0
  helpers_path.write_text('changed serialization implementation')
  modified_time = helpers_path.stat().st_mtime + 2
  os.utime(helpers_path, (modified_time, modified_time))
  assert run('-q') == 1
