"""K9 external HUD lines 2 / 3 (지연학습 / 토크학습) and the five-line Korean layout (k9-hud-ld-lt, 보노보노 2026-10-25 20:28 / 20:38
/ 20:43 / 20:53). Display only. Unit checks here; GL placement checks are in test_k9_ext_hud_sr_sad.py / test_k9_ext_hud_lane.py."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

from cluster_lat_values import (  # noqa: E402
  delay_learn_text, lane_gap_text, lane_offset_text, lat_values_text, learned_delay, learned_torque, torque_learn_text,
)


def _ld(perc, lag=0.1778, est=0.1842):
  return NS(calPerc=perc, lateralDelay=lag, lateralDelayEstimate=est)


def _lt(perc, af_raw=3.163, fr_raw=0.087, af_f=2.745, fr_f=0.1046):
  return NS(calPerc=perc, latAccelFactorRaw=af_raw, frictionCoefficientRaw=fr_raw,
            latAccelFactorFiltered=af_f, frictionCoefficientFiltered=fr_f)


def test_delay_learn_estimate_while_learning_lag_when_done():
  assert learned_delay(_ld(63)) == (63, pytest.approx(0.1842))      # < 100 % -> current estimate
  assert learned_delay(_ld(100)) == (100, pytest.approx(0.1778))    # 100 % -> lateralDelay
  assert learned_delay(None) == (None, None)
  assert learned_delay(_ld(0, 0.3, 0.3)) == (0, pytest.approx(0.3))  # 1ca start: unestimated, default 0.3 s
  assert learned_delay(_ld(float("nan"), float("nan"), float("nan"))) == (None, None)
  assert learned_delay(_ld(120)) == (100, pytest.approx(0.1778))    # clamped


def test_torque_learn_raw_while_learning_filtered_when_done():
  assert learned_torque(_lt(61)) == (61, pytest.approx(3.163), pytest.approx(0.087))
  assert learned_torque(_lt(100)) == (100, pytest.approx(2.745), pytest.approx(0.1046))
  assert learned_torque(_lt(0, 0.0, 0.0, 2.3, 0.1)) == (0, None, None)   # 1ca start: nothing learned yet -> '-'
  assert learned_torque(None) == (None, None, None)
  assert learned_torque(NS(calPerc=50)) == (None, None, None)            # broken message


def test_texts():
  assert lat_values_text(12.9, 0.20) == "조향비 12.90  지연설정 0.20초"
  assert delay_learn_text(63, 0.2418) == "지연학습 63%  0.24초"
  assert delay_learn_text(None, None) == "지연학습 -  -"
  assert torque_learn_text(61, 3.163, 0.087) == "토크학습 61%  횡가속 3.16  마찰 0.09"
  assert torque_learn_text(0, None, None) == "토크학습 0%  횡가속 -  마찰 -"
  assert lane_offset_text(0.123) == "치우침 오른쪽 12cm" and lane_offset_text(-0.08) == "치우침 왼쪽 8cm"
  assert lane_offset_text(0.01) == "치우침 가운데" and lane_offset_text(None) == "치우침 --"
  assert lane_gap_text(1.87, 1.63) == "좌 차선 1.87m   우 차선 1.63m"
  for t in (lat_values_text(12.9, 0.2), delay_learn_text(63, 0.24), torque_learn_text(61, 3.16, 0.09)):
    assert "적용" not in t and not any(w in t for w in ("SR", "SAD", "LD", "LT", "AF"))


def test_state_fields_default_unknown():
  from cluster_models import ClusterUiState
  f = ClusterUiState.__dataclass_fields__
  for k in ("delay_learn_perc", "delay_learn_s", "torque_learn_perc", "torque_learn_lat_accel_factor", "torque_learn_friction"):
    assert f[k].default is None, k


def test_live_source_reads_learners():
  from cluster_live import OpenpilotLiveSource
  src = object.__new__(OpenpilotLiveSource)
  sm = {"liveDelay": _ld(63), "liveTorqueParameters": _lt(61)}
  src._service_data = lambda n: sm.get(n)
  src._service_alive = lambda n: sm.get(n) is not None
  (dp, ds), (tp, af, fr) = src._live_learn_values()
  assert (dp, tp) == (63, 61) and ds == pytest.approx(0.1842) and af == pytest.approx(3.163) and fr == pytest.approx(0.087)
  sm.clear()
  assert src._live_learn_values() == ((None, None), (None, None, None))


def test_renderer_five_lines_order():
  import cluster_renderer as cr
  ys = list(cr.LAT_VALUES_LINE_Y) + list(cr.LANE_VALUES_LINE_Y)
  assert len(ys) == 5 and ys == sorted(ys) and len(set(ys)) == 5          # 1 (top) .. 5 (bottom)
  assert ys[-1] == cr.LAT_VALUES_CENTER_Y                                 # bottom line where SR / SAD was (4a44e564)
