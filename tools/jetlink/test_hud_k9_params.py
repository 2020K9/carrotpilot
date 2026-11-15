"""Every vehicle Param the cluster renderer reads must reach a Jetson-attached panel (same HUD as the C4 panel)."""
import ast
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(Path(__file__).resolve().parent))
from hud_protocol import PARAMS  # noqa: E402

CLUSTER = ROOT / 'openpilot/selfdrive/carrot/cluster'
# Not forwarded on purpose: write-back/remote meter (read-only adapter), the Jetson fixes its own encoder in argv,
# large navi image blobs travel through hud_navi, memory-only GPS/address params, adapter-local switches.
NOT_FORWARDED = {'CarrotNaviDebug', 'CarrotNaviImage', 'ClusterHudConnected', 'ClusterHudEncoder', 'K9MeterCmd',
                 'K9MeterState', 'LastGPSPosition', 'NetworkAddress'}
# dict.get() on /proc meminfo/status text, not Params
PROC_FIELDS = {'Buffers', 'Cached', 'MemAvailable', 'MemFree', 'MemTotal', 'Threads', 'VmRSS'}
GETTERS = {'get', 'get_bool', 'get_int', 'get_float'}


def read_keys():
  keys = set()
  for path in CLUSTER.glob('*.py'):
    if path.name.startswith('test_'):
      continue
    tree = ast.parse(path.read_text())
    consts = {n.targets[0].id: n.value.value for n in ast.walk(tree) if isinstance(n, ast.Assign)
              and len(n.targets) == 1 and isinstance(n.targets[0], ast.Name) and isinstance(n.value, ast.Constant)
              and isinstance(n.value.value, str) and n.targets[0].id.endswith('_PARAM')}
    for node in ast.walk(tree):
      if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr in GETTERS:
        if not node.args:
          continue
        arg = node.args[0]
        if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
          keys.add(arg.value)
        elif isinstance(arg, ast.Name) and arg.id in consts:
          keys.add(consts[arg.id])
      # for key in ("CustomSR", ...): params.get_float(key)
      if isinstance(node, ast.For) and isinstance(node.iter, ast.Tuple):
        values = [e.value for e in node.iter.elts if isinstance(e, ast.Constant) and isinstance(e.value, str)]
        if values and any(isinstance(c, ast.Attribute) and c.attr in GETTERS for c in ast.walk(node)):
          keys.update(values)
      # self._finite_param_float("CustomSR", ...) / self._event_service_from_param("LiveDelay", ...)
      if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and
          node.func.attr in ('_finite_param_float', '_param_int', '_event_service_from_param') and node.args and
          isinstance(node.args[0], ast.Constant) and isinstance(node.args[0].value, str)):
        keys.add(node.args[0].value)
  return {k for k in keys if re.fullmatch('[A-Z][A-Za-z0-9]+', k) and not k.isupper()}


def test_cluster_params_forwarded_to_jetson_panel():
  missing = read_keys() - set(PARAMS) - NOT_FORWARDED - PROC_FIELDS
  assert not missing, f'cluster reads Params the Jetson panel never receives: {sorted(missing)}'


def test_k9_hud_switches_forwarded():
  for key in ('ClusterHudK9Style', 'K9HudNavdy', 'K9MeterSeason', 'CustomSR', 'LiveDelay', 'LiveTorqueParameters',
              'LiveParametersV2', 'LateralTorqueCustom'):
    assert key in PARAMS
