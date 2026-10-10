# 회피 3.1 — 향후 검사 절차 (문서 전용, 실행하지 않음)

이 문서는 실행 권한이 아니다. 단위 테스트 실행에도 별도 사람 승인이 필요하다. 그 승인이 나더라도 시험·스윕·시뮬·차량·배포 권한으로 넓히지 않는다.

1. 승인 후 지정 격리 사본에서만 실행한다. 의존성 설치나 빌드가 필요하면 그것도 따로 승인받는다.
   - 대상: `openpilot/selfdrive/controls/tests/test_lane_avoid.py`, `openpilot/selfdrive/controls/tests/test_lane_model_speed_planner.py`
2. 결과는 기존 v3 테스트와 v3.1 신규 테스트로 나눠 기록한다. 실패한 테스트는 assert를 고쳐 맞추지 않고 원문과 실패 내용을 보고한다.
   - REVIEW.md §3의 예상 충돌(`test_new_avoid_side_detection_revokes_and_never_grows`)은 실패가 예상된다. 정책 승인 전에는 그대로 보고한다.
3. 통과해도 안전 확보·효과 검증이 아니다. 운영 기본값은 비활성 그대로다.
4. 재생 평가·실차 검증은 승인 수치·정책과 별도 평가 절차가 정해진 뒤 따로 승인받는다.
