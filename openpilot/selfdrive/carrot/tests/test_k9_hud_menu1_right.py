"""K9HudNavdy 1 (대감님 11/18 17:17 "1번 오른쪽칸 내용을 3번 오른쪽칸처럼"): the screen-right block of 1 is the
diagnostics block of 3 (and 2); 0 and 3 keep their mapping (17:14 "3번 걍 놔둬")."""
from pathlib import Path

MAIN = Path(__file__).resolve().parents[1] / "cluster" / "main.py"


def test_main_maps_menu_1_and_2_to_the_right_diagnostics_block():
  src = MAIN.read_text()
  assert src.count("renderer.set_k9_hud_res(True)") == 1   # 11/21: only 1 is left, its right block = diagnostics
  assert "set_k9_hud_3d" not in src and "K9_MENU_FPS_POLICY" not in src   # 3 (3D board) removed
  assert src.count("renderer.set_k9_hud_style(1)") == 1
