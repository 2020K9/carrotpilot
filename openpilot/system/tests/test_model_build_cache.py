import shlex
import shutil
import pytest
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


# [False-1] checked the USB eGPU big-model rebuild branch, which integ3 removed from launch_chffrplus.sh
# (c4-only eGPU removal). Kept visible as a skip instead of deleted; [True-0] keeps the original check.
@pytest.mark.parametrize('artifact_ready, expected_rebuild', [(True, '0'), pytest.param(False, '1', marks=pytest.mark.skip(reason='eGPU big model removed in integ3'))])
def test_launcher_accepts_precompiled_model_without_legacy_chunks(tmp_path, artifact_ready, expected_rebuild):
  bash = 'C:/Program Files/Git/bin/bash.exe' if os.name == 'nt' else shutil.which('bash')
  if not bash or not Path(bash).is_file():
    pytest.skip('bash unavailable')
  models = tmp_path / 'openpilot/selfdrive/modeld/models'
  models.mkdir(parents=True)
  (models / '.build_stamp').write_text('stamp:')
  (models / '.big_model_build_stamp').write_text('model-sha')
  (models / 'tg_input_devices.json').write_text('{}')
  (models / 'driving_tinygrad.pkl.chunkmanifest').write_text('1')
  source = (Path(BASEDIR) / 'launch_chffrplus.sh').read_text(encoding='utf8')
  function = source[source.index('function invalidate_modeld_build_if_needed {'):]
  function = function[:function.index('\n}')+2]
  script = (f'DIR={shlex.quote(tmp_path.as_posix())}\nBIG_MODEL_SHA=model-sha\nFORCE_REBUILD=0\n'
            + 'git() { echo stamp; }\n'
            + f'big_model_artifact_ready() {{ return {0 if artifact_ready else 1}; }}\n'
            + function + '\ninvalidate_modeld_build_if_needed\necho "$FORCE_REBUILD"\n')
  result = subprocess.run([bash, '-c', script], capture_output=True, text=True, check=True)
  assert result.stdout.strip().splitlines()[-1] == expected_rebuild
