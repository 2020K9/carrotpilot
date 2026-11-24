"""8e25c4c5 gave ClusterUiRenderer._draw_alert_overlay an alert slot (center, max_w) for the K9 R screen;
the Jetson HealthRenderer override must pass it on (car log 11/22 16:32: TypeError ... 'center', HUD restarting)."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'openpilot/selfdrive/carrot/cluster'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

import hud_health as hud  # noqa: E402


def test_reverse_alert_slot_reaches_base(monkeypatch):
  calls = []
  monkeypatch.setattr(hud.ClusterUiRenderer, '_draw_alert_overlay', lambda self, alert, **slot: calls.append(slot))
  monkeypatch.setattr(hud.rl, 'rl_push_matrix', lambda: None)
  monkeypatch.setattr(hud.rl, 'rl_pop_matrix', lambda: None)
  monkeypatch.setattr(hud.rl, 'rl_scalef', lambda *a: None)
  r = hud.HealthRenderer.__new__(hud.HealthRenderer)
  r.width, r.height = hud.DESIGN_WIDTH, hud.DESIGN_HEIGHT
  r.host_badge_drawn = True
  r._strip_deferred = True
  r._draw_alert_overlay(NS(size=1, text1='A', text2=''), center=(100., 200.), max_w=300.)
  r._draw_alert_overlay(NS(size=1, text1='A', text2=''))
  assert calls == [{'center': (100., 200.), 'max_w': 300.}, {}]
