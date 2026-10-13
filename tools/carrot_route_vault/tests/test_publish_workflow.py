import re
import shutil
import subprocess
from pathlib import Path

WORKFLOW = Path(__file__).resolve().parents[3] / ".github" / "workflows" / "carrot-route-vault-publish.yaml"
TEXT = WORKFLOW.read_text(encoding="utf-8")


def steps():
  body = TEXT.split("\n    steps:\n", 1)[1]
  return re.split(r"\n(?=      - )", body)


def step_index(predicate):
  return next(i for i, step in enumerate(steps()) if predicate(step))


def resolve_step():
  return steps()[step_index(lambda s: "name: Resolve image name" in s)]


def test_no_raw_owner_in_image_name():
  assert "ghcr.io/${{ github.repository_owner }}" not in TEXT


def test_resolved_image_name_is_lowercase(tmp_path):
  bash = shutil.which("bash")
  assert bash, "bash is required to execute the workflow's image-name step"
  run = re.search(r"^\s+run: (.+)$", resolve_step(), re.M).group(1)
  for owner, expected in (("2020K9", "2020k9"), ("ajouatom", "ajouatom")):
    env_file = tmp_path / f"env-{owner}"
    env_file.write_text("")
    subprocess.run([bash, "-c", run], check=True, env={"GITHUB_REPOSITORY_OWNER": owner, "GITHUB_ENV": str(env_file)})
    assert env_file.read_text() == f"IMAGE=ghcr.io/{expected}/carrot-route-vault\n"


def test_resolve_step_uses_bash():
  assert re.search(r"^\s+shell: bash$", resolve_step(), re.M)


def test_image_resolved_once_before_first_use():
  assert not re.search(r"^\s+IMAGE:", TEXT, re.M)
  resolve = step_index(lambda s: "name: Resolve image name" in s)
  first_use = step_index(lambda s: '"$IMAGE' in s)
  assert resolve < first_use


def test_js_tests_run_before_image_build():
  node = step_index(lambda s: "node --test 'tools/carrot_route_vault/tests/js/*.test.mjs'" in s)
  build = step_index(lambda s: "docker build" in s)
  assert node < build


def test_js_steps_do_not_ignore_failure():
  assert "continue-on-error" not in TEXT
  for step in steps():
    if "actions/setup-node" in step or "node --test" in step:
      assert "continue-on-error" not in step
