"""대감님 2026-11-18 18:16 "지연학습줄지워": the 지연학습 line is gone from the external HUD (classic lines above the TPMS car)
and from every K9HudNavdy panel (조향 학습 list, panels 0..3). The other lines keep their text; above the TPMS car they close up
towards the car (slots 2..5) so 학습 / 적용 / 학습 조향비 keep their old slots and 조향비 moves down one slot.
GL check needs xvfb-run."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

OTHERS = ("조향비 12.90  지연설정 0.20초", "학습 횡가속 2745  마찰 105", "적용 횡가속 2500  마찰 130",
          "학습 조향비 13.42  학습지연 0.24초")


def _st(unknown=False):
  if unknown:
    return NS(applied_steer_ratio=None, applied_steer_delay_s=None, delay_learn_perc=None, delay_learn_s=None,
              torque_learn_perc=None, torque_learn_lat_accel_factor=None, torque_learn_friction=None,
              torque_filtered_lat_accel_factor=None, torque_filtered_friction=None, learned_steer_ratio=None)
  return NS(applied_steer_ratio=12.9, applied_steer_delay_s=0.2, delay_learn_perc=63, delay_learn_s=0.24,
            torque_learn_perc=61, torque_learn_lat_accel_factor=2.5, torque_learn_friction=0.13,
            torque_filtered_lat_accel_factor=2.745, torque_filtered_friction=0.1046, learned_steer_ratio=13.4167)


def _classic(st):
  import cluster_renderer as cr
  calls = []
  r = object.__new__(cr.ClusterUiRenderer)
  r._draw_text_with_stroke = lambda text, x, y, *a, **k: calls.append((text, y))
  cr.ClusterUiRenderer._draw_lat_values(r, st)
  return cr, calls


def test_classic_hud_has_no_delay_learn_line():
  for st in (_st(), _st(unknown=True), NS(**{**vars(_st()), "delay_learn_perc": 100})):
    _, calls = _classic(st)
    assert len(calls) == 4 and not any("지연학습" in t for t, _ in calls)


def test_classic_hud_other_lines_same_text_closed_up():
  cr, calls = _classic(_st())
  assert calls == list(zip(OTHERS, cr.LAT_VALUES_SLOT_Y[1:], strict=True))
  ys = [y for _, y in calls]
  assert all(abs((b - a) - cr.HUD_LINE_STEP) < 1e-6 for a, b in zip(ys, ys[1:]))     # no empty slot between lines
  assert ys[-1] == cr.LAT_VALUES_SLOT_Y[-1]                                            # bottom line still right above the car


def test_k9_panels_lat_lines_have_no_delay_learn():
  import cluster_renderer as cr
  r = object.__new__(cr.ClusterUiRenderer)
  for st in (_st(), _st(unknown=True)):
    lines = cr.ClusterUiRenderer.k9_lat_lines(r, st)
    assert lines and not any("지연학습" in n or any("지연학습" in v for v in vals) for n, vals, _ in lines)
    assert not any(n == "" for n, _, _ in lines)
  names = [n for n, _, _ in cr.ClusterUiRenderer.k9_lat_lines(r, _st())]
  assert "조향비" in names[0] and "학습지연" in " ".join(names)                      # 조향비 first, line 5 kept


def test_renderer_source_draws_no_delay_learn_text():
  src = (CLUSTER / "cluster_renderer.py").read_text()
  assert "delay_learn_text(" not in src
