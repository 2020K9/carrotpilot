"""Mode 2 wiring, preserving old defaults and P/R dispatch ordering."""
import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
CLUSTER = ROOT / 'openpilot/selfdrive/carrot/cluster'


def test_reader_no_write_and_exact_range():
  module = ast.parse((CLUSTER / 'main.py').read_text())
  cls = next(n for n in module.body if isinstance(n, ast.ClassDef) and n.name == 'K9HudNavdyParamReader')
  namespace = {}
  exec(compile(ast.Module(body=[cls], type_ignores=[]), '<reader>', 'exec'), namespace)
  reader = namespace['K9HudNavdyParamReader'].__new__(namespace['K9HudNavdyParamReader'])
  class Params:
    def __init__(self, value): self.value = value
    def get_int(self, key):
      assert key == 'K9HudNavdy'
      return self.value
  for value, expected in [(0, 0), (1, 1), (2, 2), (3, 1), (-1, 1), ('bad', 1)]:
    reader._params = Params(value)
    assert reader.read() == expected


def test_mode_two_is_only_in_k9_road_branch():
  text = (CLUSTER / 'cluster_renderer.py').read_text()
  tree = ast.parse(text)
  cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'ClusterUiRenderer')
  render = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'render')
  source = ast.get_source_segment(text, render)
  assert source.index('_ev6_park_screen_active') < source.index('elif k9_road:')
  assert source.index('_ev6_reverse_screen_active') < source.index('elif k9_road:')
  assert source.index('_k9_turzx_draw_hud') < source.index('_k9_turzx_draw_alert')
  main = (CLUSTER / 'main.py').read_text()
  assert main.count('renderer.set_k9_hud_menu(2 if k9_hud_menu == 2 else 1)') == 2
