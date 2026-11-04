import os
import shlex
import shutil
import subprocess
from pathlib import Path

import pytest

from openpilot.common.basedir import BASEDIR


# Current (eGPU-removed) launcher behaviour. The original
# test_launcher_accepts_precompiled_model_without_legacy_chunks[False-1] in
# test_model_build_cache.py checks the removed USB eGPU big-model branch and is
# expected to fail on this branch; this file checks what the launcher does instead.
@pytest.mark.parametrize('artifact_ready', [True, False])
@pytest.mark.parametrize('big_stamp', ['model-sha', 'stale'])
def test_launcher_ignores_removed_egpu_big_model_state(tmp_path, artifact_ready, big_stamp):
  bash = 'C:/Program Files/Git/bin/bash.exe' if os.name == 'nt' else shutil.which('bash')
  if not bash or not Path(bash).is_file():
    pytest.skip('bash unavailable')
  models = tmp_path / 'openpilot/selfdrive/modeld/models'
  models.mkdir(parents=True)
  (models / '.build_stamp').write_text('stamp:')
  (models / '.big_model_build_stamp').write_text(big_stamp)
  (models / 'tg_input_devices.json').write_text('{}')
  (models / 'driving_tinygrad.pkl.chunkmanifest').write_text('1')
  marker = tmp_path / 'big_model_artifact_ready_called'
  source = (Path(BASEDIR) / 'launch_chffrplus.sh').read_text(encoding='utf8')
  assert 'BIG_MODEL_SHA' not in source and 'big_model_artifact_ready' not in source
  function = source[source.index('function invalidate_modeld_build_if_needed {'):]
  function = function[:function.index('\n}')+2]
  script = (f'DIR={shlex.quote(tmp_path.as_posix())}\nBIG_MODEL_SHA=model-sha\nFORCE_REBUILD=0\n'
            + 'git() { echo stamp; }\n'
            + f'big_model_artifact_ready() {{ touch {shlex.quote(marker.as_posix())}; return {0 if artifact_ready else 1}; }}\n'
            + function + '\ninvalidate_modeld_build_if_needed\necho "$FORCE_REBUILD"\n')
  result = subprocess.run([bash, '-c', script], capture_output=True, text=True, check=True)
  assert result.stdout.strip().splitlines()[-1] == '0'
  assert 'eGPU' not in result.stdout
  assert not marker.exists()


def test_launcher_still_rebuilds_when_regular_model_stamp_changes(tmp_path):
  bash = 'C:/Program Files/Git/bin/bash.exe' if os.name == 'nt' else shutil.which('bash')
  if not bash or not Path(bash).is_file():
    pytest.skip('bash unavailable')
  models = tmp_path / 'openpilot/selfdrive/modeld/models'
  models.mkdir(parents=True)
  (models / '.build_stamp').write_text('old:')
  (models / 'tg_input_devices.json').write_text('{}')
  (models / 'driving_tinygrad.pkl.chunkmanifest').write_text('1')
  source = (Path(BASEDIR) / 'launch_chffrplus.sh').read_text(encoding='utf8')
  function = source[source.index('function invalidate_modeld_build_if_needed {'):]
  function = function[:function.index('\n}')+2]
  script = (f'DIR={shlex.quote(tmp_path.as_posix())}\nFORCE_REBUILD=0\n'
            + 'git() { echo stamp; }\n' + function + '\ninvalidate_modeld_build_if_needed\necho "$FORCE_REBUILD"\n')
  result = subprocess.run([bash, '-c', script], capture_output=True, text=True, check=True)
  assert result.stdout.strip().splitlines()[-1] == '1'
