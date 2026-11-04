"""공통환경 비교 경로 (2026-11-04 k9cmp2235) for the five K9 families whose frozen-controller tests stop at k9_rig's guard.

The original tests stay exactly as they are and keep failing/erroring at k9_rig.load_frozen_cc's guard (k9_rig.py:166,
hyundaicanfd.py changed outside create_acc_control_scc2). This module adds a separately named path next to them:

  * the very same test function objects (same code, same asserts, same inputs, same thresholds, same parametrization) and the
    same fixtures of the original module are collected again from a test_k9_common_env_<family>.py file;
  * while that file runs, the original module's global ``load_frozen_cc`` is replaced by ``load_frozen_cc_common_env``: the
    body of k9_rig.load_frozen_cc from ``tmp_dir = Path(tmp_dir)`` on (the loading part, unchanged AST), without the statements
    before it (the byte-equality guard). So the past controller of each test's own reference commit (its carcontroller.py and
    the k9_stop_taper.py / k9_creep.py it had, exactly what the original loads) runs against this tree's common modules.

This is NOT the historical byte-equality check (the common files are this tree's, not the reference commit's), and it is a
model-only harness with synthetic inputs (실차 미검증). Results go to $K9CMP_OUT/results.jsonl when that is set.
"""
import ast
import copy
import hashlib
import inspect
import json
import os
import subprocess
import textwrap

import pytest

from openpilot.selfdrive.car.tests import k9_rig

GUARD_END = "tmp_dir = Path(tmp_dir)"


def record(kind, **data):
  out = os.environ.get("K9CMP_OUT")
  if out:
    with open(os.path.join(out, "results.jsonl"), "a", encoding="utf-8") as f:
      f.write(json.dumps({"kind": kind, **data}, ensure_ascii=False, default=str) + "\n")


def _orig_def():
  return ast.parse(textwrap.dedent(inspect.getsource(k9_rig.load_frozen_cc))).body[0]


def split_loader():
  """(docstring, dropped guard statements, kept loading statements) of k9_rig.load_frozen_cc."""
  fn = _orig_def()
  idx = next(i for i, s in enumerate(fn.body) if ast.unparse(s) == GUARD_END)
  return fn.body[0], fn.body[1:idx], fn.body[idx:]


def _build_loader():
  fn = _orig_def()
  doc, _dropped, kept = split_loader()
  names = {n.id for s in kept for n in ast.walk(s) if isinstance(n, ast.Name)}
  assert not names & {"changed", "car_data", "diff", "canfd", "iface"}, names  # loading part never uses the guard's names
  new = copy.deepcopy(fn)
  new.name = "load_frozen_cc_common_env"
  new.body = [copy.deepcopy(doc)] + copy.deepcopy(kept)
  mod = ast.fix_missing_locations(ast.Module(body=[new], type_ignores=[]))
  ns = dict(vars(k9_rig))
  exec(compile(mod, k9_rig.__file__ + ":load_frozen_cc_common_env", "exec"), ns)
  return ns["load_frozen_cc_common_env"]


load_frozen_cc_common_env = _build_loader()


def common_env_files(commit):
  changed = subprocess.run(["git", "diff", "--name-only", commit, "--", "opendbc_repo"], cwd=k9_rig.ROOT, check=True,
                           capture_output=True, text=True).stdout.split()
  own = {k9_rig.CC_REL, f"{k9_rig.HY}/k9_stop_taper.py", f"{k9_rig.HY}/k9_creep.py"}
  return sorted(f for f in changed if "/tests/" not in f and f not in own)


def _guard_result(commit, tmp_dir, tag):
  try:
    k9_rig.load_frozen_cc(commit, tmp_dir, tag + "_guard")
    return "loads"
  except AssertionError as e:
    return f"AssertionError: {e}"


def _is_fixture(v):
  return hasattr(v, "_pytestfixturefunction") or type(v).__name__ == "FixtureFunctionDefinition"


def _fn(v):
  return getattr(v, "_get_wrapped_function", lambda: getattr(v, "__wrapped__", v))()


def bind(orig, test_names, ns):
  """Collect `test_names` of module `orig` (same objects) and all its fixtures into namespace `ns`, with orig.load_frozen_cc
  replaced by load_frozen_cc_common_env while this module runs."""
  assert orig.load_frozen_cc is k9_rig.load_frozen_cc
  fixtures = sorted(k for k, v in vars(orig).items() if _is_fixture(v))
  for k in fixtures:
    ns[k] = getattr(orig, k)
  for n in test_names:
    ns[n] = getattr(orig, n)
  seen = set()

  def loader(commit, tmp_dir, tag):
    cc = load_frozen_cc_common_env(commit, tmp_dir, tag)
    if commit not in seen:
      seen.add(commit)
      record("env", module=orig.__name__, ref=commit, guard=_guard_result(commit, tmp_dir, tag),
             common_env_files=common_env_files(commit), cc_file=cc.__file__)
    record("load", module=orig.__name__, ref=commit, tag=tag, during=os.environ.get("PYTEST_CURRENT_TEST"))
    return cc

  @pytest.fixture(scope="module", autouse=True)
  def _k9_common_env():
    mp = pytest.MonkeyPatch()
    mp.setattr(orig, "load_frozen_cc", loader)
    try:
      yield
    finally:
      mp.undo()
    assert orig.load_frozen_cc is k9_rig.load_frozen_cc

  ns["_k9_common_env"] = _k9_common_env

  def test_common_env_same_code_as_original():
    """AST proof: every collected test (and fixture) is the original object and its AST equals the committed original's."""
    rel = os.path.relpath(orig.__file__, k9_rig.ROOT)
    head = subprocess.run(["git", "show", f"HEAD:{rel}"], cwd=k9_rig.ROOT, check=True, capture_output=True,
                          text=True).stdout
    with open(orig.__file__, encoding="utf-8") as f:
      work = f.read()
    defs_head = {d.name: d for d in ast.parse(head).body if isinstance(d, ast.FunctionDef)}
    defs_work = {d.name: d for d in ast.parse(work).body if isinstance(d, ast.FunctionDef)}
    proof = {}
    for n in list(test_names) + fixtures:
      assert ns[n] is getattr(orig, n)
      code = _fn(ns[n]).__code__
      if code.co_filename != orig.__file__:
        proof[n] = {"defined_in": code.co_filename}  # an imported fixture: same object, checked in its own module
        continue
      a, b = ast.dump(defs_head[n]), ast.dump(defs_work[n])
      assert a == b, n
      assert code.co_firstlineno in [defs_work[n].lineno] + [d.lineno for d in defs_work[n].decorator_list], n
      n_assert = sum(isinstance(x, ast.Assert) for x in ast.walk(defs_work[n]))
      proof[n] = {"ast_sha256": hashlib.sha256(a.encode()).hexdigest()[:16], "asserts": n_assert}
    doc, dropped, kept = split_loader()
    built = ast.parse(textwrap.dedent(inspect.getsource(k9_rig))).body
    orig_fn = next(d for d in built if isinstance(d, ast.FunctionDef) and d.name == "load_frozen_cc")
    assert [ast.dump(s) for s in orig_fn.body[len(orig_fn.body) - len(kept):]] == [ast.dump(s) for s in kept]
    record("ast_proof", module=orig.__name__, file=rel, same_as_HEAD=head == work, tests=proof,
           loader_dropped_statements=len(dropped), loader_kept_statements=len(kept),
           loader_kept_sha256=hashlib.sha256("".join(ast.dump(s) for s in kept).encode()).hexdigest()[:16])
    assert head == work

  ns["test_common_env_same_code_as_original"] = test_common_env_same_code_as_original
  return fixtures
