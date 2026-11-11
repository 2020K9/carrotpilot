import ast
from pathlib import Path

SRC = Path(__file__).resolve().parents[1] / "controlsd.py"


def test_lanechangestate_defined_at_module_level():
  # 5db92699 병합에서 정의가 빠져 controlsd가 NameError로 죽었다 (11/10 차량 로그)
  tree = ast.parse(SRC.read_text())
  names = {t.id for n in tree.body if isinstance(n, ast.Assign) for t in n.targets if isinstance(t, ast.Name)}
  used = any(isinstance(n, ast.Name) and n.id == "LaneChangeState" for n in ast.walk(tree))
  assert not used or "LaneChangeState" in names
