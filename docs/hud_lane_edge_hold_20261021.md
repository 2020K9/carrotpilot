# External HUD lane/road-edge presentation hold, 2026-10-21

Presentation-only change to the default external cluster HUD. No vehicle, CAN,
setting, safety, model, steering or planned-path change. Edge colours and alpha
are unchanged (held edges reuse the existing `road_edge_color`).

## What changed

- Thin lane lines: a short hold when a detected line is hidden only by a
  confidence dip while current geometry is still valid.
- Wide lane highlight: kept steady across the same short dips.
- Road edges, by selected line style:
  - T and 4: existing hold behaviour kept as before.
  - 2 and 3: new bounded hold, only for edges hidden by the near-lane gate whose
    current geometry is valid. `road_edge_outside_lane_line` rejects edges with
    fewer than two points, forward points beyond the lane line range, or any
    point touching or crossing the adjacent inner lane line.
  - 1: unchanged.
- Bounds, offsets, visibility flags used by radar/object filtering are not
  modified; tests assert all other parser keys are identical.

## Evidence and limits

- Offline helper replay (drawing stub, synthetic 10 Hz ticks) over five KIA K9
  logs: right edge show/hide toggles 14 -> 4 on one log, left 2 -> 2; the other
  four logs were 0 -> 0. Only one of five logs is affected.
- Colour/brightness flicker is not fixed; measured colour-class and alpha
  changes remain.
- Offline Xvfb renders: one straight frame is pixel-identical before/after.
  Pixel benefit of an accepted hold event is not demonstrated. No in-car test.
- Tests: full `test_cluster_*.py` suite run tree-first (`PYTHONPATH=<tree>`,
  locally built `params_pyx.so`, no project modules from other checkouts):
  535 passed on this change, 511 passed on the pre-change snapshot.

## 한국어 요약

외부 HUD 표시 전용 변경입니다. 차량, CAN, 설정, 안전, 모델, 조향, 주행 경로는 바꾸지 않았고 색과 alpha도 그대로입니다.
- 얇은 차선은 신뢰도가 잠깐 떨어질 때 짧게 유지합니다. 넓은 강조도 같은 구간에서 그대로 유지합니다.
- 도로 경계는 선 스타일별로 다릅니다. T와 ④는 기존 hold를 그대로 둡니다. ②와 ③은 현재 기하가 유효할 때만 짧게 유지합니다(점이 2개 미만이거나, 차선 범위를 벗어나거나, 차선에 접하거나 가로지르면 유지하지 않음). ①은 바뀌지 않았습니다.
- 오프라인 helper 측정에서 5개 로그 중 1개만 우측 경계 전환이 14회에서 4회로 줄었습니다. 색과 밝기 깜빡임은 해결하지 못했고, 실차 검증은 하지 않았습니다.
- 테스트: 현재 트리만 import하도록 실행해 535개 통과했고, 변경 전 스냅샷은 511개 통과했습니다.
