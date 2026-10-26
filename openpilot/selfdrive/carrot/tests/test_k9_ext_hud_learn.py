"""K9 external HUD lines 2 / 3 (지연학습 / 토크학습) and the five-line Korean layout (k9-hud-ld-lt, 보노보노 2026-10-25 20:28 / 20:38
/ 20:43 / 20:53). Display only. Unit checks here; GL placement checks are in test_k9_ext_hud_sr_sad.py / test_k9_ext_hud_lane.py."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

CLUSTER = Path(__file__).resolve().parents[1] / "cluster"
sys.path.insert(0, str(CLUSTER))

from cluster_lat_values import (  # noqa: E402
  applied_torque, delay_learn_text, lane_gap_text, lane_offset_text, lat_values_text, learned_delay, torque_learn_text,
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


def test_torque_line_shows_applied_values_never_raw():
  # 보노보노 2026-10-26 17:23: LateralTorqueCustom 1 -> menu values x 0.001 (what latcontrol_torque uses), whatever calPerc
  assert applied_torque(_lt(61), 1, 2500, 131) == (61, pytest.approx(2.5), pytest.approx(0.131))
  assert applied_torque(_lt(100), 1, 2500, 131) == (100, pytest.approx(2.5), pytest.approx(0.131))
  # LateralTorqueCustom 0 -> Filtered (controlsd update_live_torque_params), also while learning (= CarParams seed)
  assert applied_torque(_lt(61), 0, 2500, 131) == (61, pytest.approx(2.745), pytest.approx(0.1046))
  assert applied_torque(_lt(0, 0.0, 0.0, 2.3, 0.1), 0) == (0, pytest.approx(2.3), pytest.approx(0.1))
  for perc in (0, 14, 61, 99, 100):                                      # Raw (3.163 / 0.087) never shown
    _, af, fr = applied_torque(_lt(perc), 0)
    assert af != pytest.approx(3.163) and fr != pytest.approx(0.087)
  assert applied_torque(None, 1, 2500, 131) == (None, pytest.approx(2.5), pytest.approx(0.131))
  assert applied_torque(None, 0) == (None, None, None)
  assert applied_torque(_lt(61), None, 2500, 131) == (61, None, None)    # menu not read yet -> '-'
  assert applied_torque(NS(calPerc=50), 0) == (50, None, None)            # broken message


def test_texts():
  assert lat_values_text(12.9, 0.20) == "조향비 12.90  지연설정 0.20초"
  assert delay_learn_text(63, 0.2418) == "지연학습 63%  0.24초"
  assert delay_learn_text(None, None) == "지연학습 -  -"
  assert torque_learn_text(61, 2.5, 0.131) == "토크학습 61%  횡가속 2.50  마찰 0.13"
  assert torque_learn_text(0, None, None) == "토크학습 0%  횡가속 -  마찰 -"
  assert lane_offset_text(0.123) == "치우침 오른쪽 12cm" and lane_offset_text(-0.08) == "치우침 왼쪽 8cm"
  assert lane_offset_text(0.01) == "치우침 가운데" and lane_offset_text(None) == "치우침 --"
  assert lane_gap_text(1.87, 1.63) == "좌 1.87m   우 1.63m"
  for t in (lat_values_text(12.9, 0.2), delay_learn_text(63, 0.24), torque_learn_text(61, 2.5, 0.131)):
    assert "적용" not in t and not any(w in t for w in ("SR", "SAD", "LD", "LT", "AF"))


def test_state_fields_default_unknown():
  from cluster_models import ClusterUiState
  f = ClusterUiState.__dataclass_fields__
  for k in ("delay_learn_perc", "delay_learn_s", "torque_learn_perc", "torque_learn_lat_accel_factor", "torque_learn_friction"):
    assert f[k].default is None, k


class _Params:
  def __init__(self, values):
    self.values = values
    self.reads = 0

  def get_float(self, key):
    self.reads += 1
    return float(self.values.get(key, 0.0))


def test_live_source_reads_learners():
  from cluster_live import OpenpilotLiveSource
  src = object.__new__(OpenpilotLiveSource)
  src.params = _Params({"LateralTorqueCustom": 1, "LateralTorqueAccelFactor": 2500, "LateralTorqueFriction": 131})
  sm = {"liveDelay": _ld(63), "liveTorqueParameters": _lt(61)}
  src._service_data = lambda n: sm.get(n)
  src._service_alive = lambda n: sm.get(n) is not None
  (dp, ds), (tp, af, fr) = src._live_learn_values(100.0)
  assert (dp, tp) == (63, 61) and ds == pytest.approx(0.1842) and af == pytest.approx(2.5) and fr == pytest.approx(0.131)
  reads = src.params.reads
  src.params.values.update(LateralTorqueCustom=0)
  src._live_learn_values(100.5)                                          # menu re-read at most once per second
  assert src.params.reads == reads
  (_, _), (tp, af, fr) = src._live_learn_values(101.1)
  assert tp == 61 and af == pytest.approx(2.745) and fr == pytest.approx(0.1046)
  sm.clear()
  assert src._live_learn_values(102.2) == ((None, None), (None, None, None))


def test_renderer_five_lines_order():
  import cluster_renderer as cr
  ys = list(cr.LAT_VALUES_LINE_Y) + list(cr.LANE_VALUES_LINE_Y)
  assert len(ys) == 5 and ys == sorted(ys) and len(set(ys)) == 5          # 1 (top) .. 5 (bottom)
  # 2026-10-26 18:04 (보노보노 "차선치우침.현재차선좌우간격 아래 tpms보여주는 차모양 왼쪽으로 옮겨놔"): line 3 where SR / SAD
  # was (4a44e564), right above the TPMS car; lines 4 / 5 left of the car, centred on its height
  assert ys[2] == cr.LAT_VALUES_CENTER_Y
  assert ys[2] < cr.TPMS_STATUS_CAR_CENTER_Y - cr.TPMS_STATUS_ICON_H * 0.5
  assert (ys[3] + ys[4]) * 0.5 == cr.TPMS_STATUS_CAR_CENTER_Y
  assert cr.LANE_VALUES_RIGHT_X < cr.TPMS_STATUS_CENTER_X - cr.TPMS_STATUS_ICON_W * 0.5


def test_comma_screen_has_no_k9_hud_lines():
  # 보노보노 2026-10-26 17:24 "외부uhd에 넣은거 콤마화면에서는 빼라": the comma (mici) onroad HUD draws none of the K9 HUD lines
  src = (Path(__file__).resolve().parents[2] / "ui" / "mici" / "onroad" / "hud_renderer.py").read_text(encoding="utf-8")
  for word in ("cluster_lat_values", "_k9_lane", "치우침", "토크학습", "지연학습", "조향비", "lane_gap_text", "lane_offset_text", "k9LaneState"):
    assert word not in src, word
