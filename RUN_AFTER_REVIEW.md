# 회피 3.2 — 향후 검사 절차 (문서 전용, 이 작성 단계에서 실행하지 않음)

이 문서는 실행 권한이 아니다. 시험 실행은 부모 세션이 별도 기록으로 시행한다. 통과해도 안전 확보·효과 검증이 아니며 운영 기본값은 비활성 그대로다.

## 명령 (저장소 루트 `/home/ssm-user/work/lanemode_avoid_v3_2/repo`에서)

```
PYTHONPATH=/home/ssm-user/work/lanemode_avoid_v3_2/repo python3 -m pytest -q \
  openpilot/selfdrive/controls/tests/test_lane_avoid.py \
  openpilot/selfdrive/controls/tests/test_lane_avoid_v3_2.py \
  openpilot/selfdrive/controls/tests/test_lane_model_speed_planner.py
```

## 의존성 (확인 안 됨 — 설치·빌드는 별도 승인)
- python3, pytest, numpy
- `openpilot` 패키지 불러오기 경로(위 PYTHONPATH). lane_avoid.py/lane_avoid_audit.py는 numpy만 쓴다.
- 계획기 시험(`make_planner`, test_lane_model_speed_planner.py)은 lateral_planner.py의 의존(cereal/pycapnp 메시지, openpilot.common, 횡 MPC 모듈 등)이 필요하다. 해당 시험 파일이 쓰는 대체·준비 방식을 따른다(정적 확인 안 함).

## 기록 방법
1. 기존 v3/v3.1 시험과 v3.2 신규(test_lane_avoid_v3_2.py)를 나눠 기록한다. 실패는 assert를 고쳐 맞추지 않고 원문·실패 내용을 보고한다.
2. 예상 실패(REVIEW.md §4): `test_new_avoid_side_detection_revokes_and_never_grows`, `test_required_span_covers_body_and_whole_path`(bend 단언). 사람 결정 전에는 그대로 보고한다.
3. 재생 평가·실차 검증은 승인 수치·정책과 별도 평가 절차가 정해진 뒤 따로 승인받는다.
