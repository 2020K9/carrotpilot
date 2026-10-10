# 쏠림·코너 v3 — 보류 3건 정적 검토 보고서

- 승인 지시문: lanebias_corner_prompt_v3.md, SHA256 b1b250a64c69c6a3db1c6fcaf5472ca6f73f19f691295b2fd16adf88d2a2ee0b (감사원 2026-10-10 17:20 승인 기록 기준)
- 기준 커밋 0e117698806939a80ab25d5611838ab2368e732e / 트리 2791c2cb77f336a6d52bfd4245cbd37552a58daf
- 격리 경로: /home/ssm-user/work/lanebias_corner_v3/repo (워커 i-043a7af79082c7f70). 최종 로컬 커밋 식별자는 git log로 확인한다(이 파일이 그 커밋에 포함되므로 자기 식별자를 적을 수 없다).
- **테스트·구문 검사·빌드·시험·원로그·차량·push는 모두 미실행이다.** 아래 대응은 소스 정적 열람과 작성 결과일 뿐이며 통과·개선·실차 해결을 뜻하지 않는다.
- 신규 기능은 기본 비활성이다. `LanelessCenterConfig` 필드는 모두 `None`이고 `APPROVED_CONFIG = None`이다. 그래서 `LanelessCenterCorrection().enabled == False`이고 controlsd는 기존 줄을 그대로 실행한다. Params 키·설정 화면·사용자 문서는 추가하지 않았다. 테스트의 `TEST_CFG` 수치는 합성 입력이며 승인값이 아니다.

## 1. 변경 파일

| 파일 | 내용 |
|---|---|
| openpilot/selfdrive/controls/lib/laneless_center.py (신규) | 양쪽 차선 중앙 정의, 지점별 중앙 제한, 곡률 증분 변환, 소비 체인(혼합→평활→clip) 순수 함수, 상태·차단 사유·기록 |
| openpilot/selfdrive/controls/lib/lat_error_decomposition.py (신규) | 소비 곡률 이력 재구성 P, 같은 진행 위치 s 기준 오차 분해, 0 부근 비율 처리 |
| openpilot/selfdrive/controls/controlsd.py | import 29~30, 생성 97, `laneless_center_delta` 172~191, `consume_laneless_center` 193~208, 일반 분기 328~344, LaC 출력 기록 385~387 |
| openpilot/selfdrive/controls/tests/test_laneless_center.py (신규) | 보류 ①② 작성 테스트(미실행) |
| openpilot/selfdrive/controls/tests/test_lat_error_decomposition.py (신규) | 보류 ③ 작성 테스트(미실행) |

기존 테스트 파일과 assert는 수정·삭제하지 않았다. lane_planner_2.py, lateral_planner.py, lat_mode_blend.py, path_verifier.py, latcontrol_torque.py도 변경하지 않았다. AdjustLaneOffset·offset_curve·offset_lane(:161~176), `use_laneless_center_adjust=False`(:198), ±0.4(:210~213), LatMpcInputOffset(:233~246), UseLaneLineSpeed와 각 기본값은 그대로다.

## 2. 보류 ①~③ 대응표

| 보류 | 원본 위치(0e117698) | 수정 위치 | 작성 테스트 | 남은 blocker |
|---|---|---|---|---|
| ① 레인리스 보정 → 실제 소비 곡률 | controlsd.py:265,282~293,326~331; lane_planner_2.py:198~207,249 | controlsd.py:328~344가 PathVerifier(:326~327) 뒤 레인리스 후보에 delta를 더하고 `consume_curvature`로 혼합·평활·clip을 거쳐 `self.desired_curvature`를 만든다. 이 값이 LaC.update(:379~)에 들어간다. laneless_center.py `consume_curvature`, `LanelessCenterCorrection.update` | test_pure_laneless_correction_reaches_consumed_curvature, test_plan_only_edit_does_not_reach_consumption, test_zero_delta_chain_is_bit_identical_to_original, test_controlsd_default_branch_kept_and_new_path_gated, test_lane_mode_weight_one_has_no_laneless_addition, test_transition_adds_correction_once_scaled_by_laneless_weight, test_clip_saturation_is_distinguished_from_missing_connection, test_blocked_inputs_never_request(불일치·누락·오래됨·검증기 개입·차선변경·운전자 조향), test_inactive_resets_immediately_and_block_releases_at_rate | 모든 설정값 미승인(비활성). 기록(trace)은 메모리에만 있고 cereal 필드가 없어 로그에 남지 않는다(스키마 변경 미승인). controlsd 전체를 불러오는 연결 테스트는 무거운 의존성 때문에 작성하지 않았고, 소스 문자열 정적 검사와 순수 함수 검사로 대신했다. 차단 시 해제 속도는 미승인 `max_delta_rate`를 쓴다(복귀 정책 미승인). 평활 상태에 남은 보정 잔량은 기존 필터를 덮어쓰지 않으면 지울 수 없다. 이 경우 위반으로 기록하고 그 주기만 보정 없이 다시 계산한다. 이 잔류 처리는 미해결이다. |
| ② 지점마다 중앙 초과 금지 | lane_planner_2.py:201~205(한 시점), :210~213(±0.4), :221(필터), :249(전체 가산), :178~196(lane_path_y는 양쪽 중앙이 아닐 수 있음) | laneless_center.py `lane_center`(양쪽 차선, 같은 x 표본, 좌<우, 신뢰도), `curvature_delta_interval`(모든 점+조밀화 점의 교집합), `center_bounded_target`, `uniform_shift_interval`, `consumed_within_bounds`; controlsd `consume_laneless_center`가 최종 clip 값과 보정 없는 그림자 체인의 차이로 재검사 | test_far_point_already_on_center_allows_nothing, test_far_point_center_in_opposite_direction_allows_nothing, test_path_crossing_center_allows_nothing, test_every_point_and_interpolated_point_stays_between_path_and_center(직진·좌우 완만·급코너), test_filter_residual_beyond_new_interval_is_detected, test_blocked_cycle_reports_any_nonzero_consumed_delta, test_left_right_symmetry, test_lane_center_requires_both_lines_same_positions_and_order, test_center_bounded_target_and_uniform_shift_interval | P 재구성은 소각도·구간 내 일정 곡률·같은 초기 상태를 가정한다. 이것은 실차 경로 보장이 아니다. 모델 위치 배열과 차선선의 시간 대응은 같은 modelV2 메시지·같은 x로만 맞췄고, 차선선 t 축은 쓰지 않았다. CAMERA_OFFSET과 후단 PathOffset은 레인모드 계획 경로에만 적용된다. 레인리스 직접 곡률에는 들어가지 않아 이번 구조에서는 검사 대상이 아니다. 레인모드(B1) 지점별 제한은 미구현이다. |
| ③ 실제 소비 곡률·명령 기준 오차 분해 | 2판 §2 C(계획 위치 기반); controlsd.py:265,282~293,326~331 | lat_error_decomposition.py `reconstruct_from_curvature`, `center_frame_at`, `decompose`, `safe_ratio`. 소비 단계 기록은 laneless_center trace의 model, corrected, blended, smoothed, clipped(=LaC 입력), curvature_limited, torque, steering_angle_deg | test_sum_is_preserved_at_same_progress, test_straight_and_left_right_curves, test_frame_change_is_consistent, test_sign_flip, test_longitudinal_mismatch_is_unevaluable_not_rematched, test_known_time_delay_shows_up_as_tracking_not_hidden, test_duplicate_reversed_nonfinite_time_rejected, test_same_plan_different_consumed_curvature_changes_path_share, test_different_plan_same_consumed_curvature_same_path_share, test_command_limit_seen_as_consumed_difference, test_vehicle_model_error_lands_in_tracking_residual, test_ratio_near_zero_target_is_not_filled | 생성·소비 시각과 메시지 식별은 기록 구조에 아직 연결하지 않았다(trace에 시각 없음). 차량 전송 직전 명령(carcontroller)과 차량 응답은 기록하지 않는다. 실제 로그 정렬·재현·분해 검증은 미실행이다. 종방향 허용치는 호출자가 주는 미승인 값이고, 없으면 평가불가다. |

## 3. 실제 소비 경로 연결표 (K9 일반 분기, 기능 켜짐을 가정)

| 단계 | 위치 | 변수 | 비고 |
|---|---|---|---|
| 1 원 모델 직접 곡률 | controlsd.py:307 | `laneless_curvature = model_v2.action.desiredCurvature` | 기록 `trace["model"]`(검증기 후 값) |
| 2 안전 보정(PathVerifier) | :326~327 | `laneless_curvature` | 검증기 effect≠0이면 보정을 차단한다(`path_verifier_active`). 보정은 검증기 출력 뒤에 더하며 검증기를 우회하지 않는다. |
| 3 보정 후보 | :328~330 → laneless_center.update | delta, `trace["request"/"interval"/"delta"/"reason"]` | 허가·차단 사유 기록 |
| 4 혼합 | consume_curvature | `blend_lat_mode(w, lane_target, laneless+delta)` | lane_target에는 delta가 없다. 기여분은 (1−w)·delta이며, w>max_lane_mode_weight이면 차단한다. |
| 5 평활 | consume_curvature | tau=blend(w, LatSmooth, 0.1) | 기존 식과 동일하다 |
| 6 곡률 제한 | consume_curvature → clip_curvature | `self.desired_curvature`, `curvature_limited` | 그림자 체인과 비교해 지점별로 재검사한다. 위반하면 그 주기를 delta=0으로 다시 계산한다. |
| 7 횡제어기 입력/출력 | :379~384 LaC.update | torque, steeringAngleDeg | trace에 기록한다(:385~387) |
| 꺼짐/비활성/VW MEB | :309~322, :331~335, :341~344 | 기존 줄 그대로 | VW MEB 분기와 latActive 거짓 분기는 손대지 않았다. 켜짐 상태에서는 reset만 호출한다. |

시작 준비 보호(:212~222, :255, :271~273)와 운전자 우선(steeringPressed 차단, LaC.reset)은 보존된다. 시작 준비 미완료는 CC.latActive=False이므로 보정이 inactive로 0이 된다.

## 4. 중앙 제한 수식·적용 구간

- 모델 좌표계(x 전방, y 우측 양수, 곡률 양수=우회전)에서 b_i=position.y(x_i), c_i=(laneLines[1].y_i+laneLines[2].y_i)/2를 쓴다. x_i는 같은 메시지의 차선 x 표본이며 [check_x_min, check_x_max] 범위와 계획 x 범위 안의 점만 쓴다.
- 지점별 조건은 min(b_i,c_i) ≤ y_i ≤ max(b_i,c_i)이다. b_i=c_i이면 추가 보정은 0이다.
- 곡률 증분 dk는 y_i=b_i+dk·x_i²/2로 재구성한다. 허용 dk는 ∩_i [min(d_i,0), max(d_i,0)]·2/x_i²(d_i=c_i−b_i)이고, 점 사이는 DENSIFY=4로 조밀화한 선형보간 점에도 같은 조건을 적용한다. DENSIFY는 판정 수치가 아니라 검사 밀도다.
- 요청값은 eval_x에서 b+g(c−b)를 겨냥한 2(target−b)/eval_x²이고, 이를 위 구간으로 자른다. 교집합이 {0}인데 요청이 0이 아니면 `no_common_interval`로 차단한다.
- 최종 소비 검사는 dk_consumed = clip 후 desired − 그림자(보정 없음, 자체 상태) desired로 계산해 같은 점 집합에서 재검사한다.
- g, 범위, 신뢰도, 불일치 허용치, 나이, 변화율은 모두 미승인(None)이다.

## 5. 오차 분해 계약

- 고정 평면 좌표계 하나를 쓴다. 원점과 축은 호출자가 명시한다. C(s)는 호(弧) 길이로 매개화한 중앙선이며 범위 밖으로 외삽하지 않는다.
- 시각 t에서 E(t)는 자차(측정 또는 시뮬, 출처는 호출자가 표기), P(t)는 소비 곡률 이력을 ψ'=vκ, x'=v cosψ, y'=v sinψ로 사다리꼴 적분한 값이다. 초기 (x0, y0, ψ0)는 명시하고, 시각은 엄격 증가·유한해야 한다.
- 하나의 s에서 C(s)의 법선 방향으로 E와 P를 함께 측정한다. 서로 다른 최근접점은 고르지 않는다. 접선 방향 차이가 허용치(미승인, 없으면 평가불가)를 넘으면 평가불가다.
- 전체=e_lat, 경로 몫=p_lat, 추종 몫=e_lat−p_lat이다. 합 보존은 장부상 항등식일 뿐 인과 분해의 증명이 아니다. 추종 몫에는 제어기·명령 제한·차량 지연·추정 오차·시뮬 모델 오차가 섞인다(test_vehicle_model_error_lands_in_tracking_residual).
- 비율은 분모 절댓값이 하한(미승인) 미만이거나 하한이 미설정이면 None이다. 과거 "경로 1/3·추종 2/3" 수치는 기준으로 쓰지 않는다.

## 6. 후보별 상태

- A1: 철회 유지. 코드 변경 없음.
- A2: 소비식은 latcontrol_torque.py:145~162 그대로 기록만 하고 수치·방향은 정하지 않았다. 미착수(보류 ③ 기록 구조 우선).
- A3: LatMpcInputOffset 2/0 비교 명세·테스트는 **미작성(시간 내 미착수)**.
- B1(레인모드 중앙 제한): **미구현**. 레인모드 경로(lane_planner_2→lateral_planner→lat_plan.curvatures→get_lag_adjusted_curvature)는 변경하지 않았다.
- B2: 이번 laneless_center로 재설계했다(비활성). 급코너 감쇠 하위 옵션은 기준 신호·승인값이 없어 미구현·막힘이다.
- C: 배타성은 `max_lane_mode_weight`로 레인모드 가중치가 있을 때 B2를 차단하는 방식으로 작성했다. B1이 없어 조합 충돌 테스트는 미작성이다.

## 7. 회피 3.1과의 충돌표(정적)

회피 코드(lane_avoid.py, 81fae5e0 계열)는 이 기준 사본에 없다. 이식·자동 병합 검사는 하지 않았다.

| 항목 | 회피 3.1 | 이번 변경 | 충돌 |
|---|---|---|---|
| 파일 | lateral_planner.py:167~173(경로 전체에 applied 가산), lane_avoid.py | controlsd.py 레인리스 분기, 신규 lib 2개 | 직접 같은 줄은 없다. 둘 다 최종적으로 `self.desired_curvature`에 반영된다. |
| 모드 | 레인모드 계획 경로 | 레인리스 직접 곡률(w ≤ 승인 한도) | 전환 구간(0<w<1)에서 두 기여가 혼합으로 함께 들어갈 수 있다. 결합 규칙이 미승인이므로 결합은 비활성·막힘이다. |
| 중앙 제한 | 회피는 중앙을 벗어날 수 있다 | 이 모듈은 중앙 너머를 금지한다 | 회피 중에 레인리스 중앙 보정을 하면 회피를 되돌릴 수 있다. 회피 활성 신호를 이 모듈 차단 사유로 넣는 연결은 미구현이다. |
| 테스트 객체 | 회피 테스트 픽스처 | 순수 함수 입력 | 공유 픽스처 없음 |

높은 차선 확률·빈 객체 목록은 이 모듈에서 빈 공간 증거로 쓰지 않는다(이동 허가 판단 없음). 보정량은 차선 중앙까지로만 제한된다.

## 8. 향후 평가 명세(전부 미실행)

1. 도구·기능 코드리뷰와 별도 실행 승인을 받는다 → 2. 입력·시계·좌표·명령 출처를 확인한다(trace에 시각·메시지 식별을 추가해 cereal로 기록할지 승인) → 3. 이 두 테스트 파일의 단위 검사를 실행한다(승인 후) → 4. 기준선(비활성) 재현과 비트 동일성을 확인한다 → 5. 실제 모드 가중치·소비값을 확인한다 → 6. 승인된 단독 후보만 비교한다 → 7. 코너별·직진·정상 장면·복귀 안전을 평가한다.
- 지표: 코너 안쪽 물기·직진 쏠림의 평균/최대, 중앙 반대편 초과 횟수, 다른 장면 악화, 곡률·조향 명령 변화율, 복귀 지연을 같은 유효 표본으로 잰다. 최소 개선폭·허용치는 미승인이다. 미활성 0건을 개선으로 세지 않는다.
- 판정은 개선/실패/평가불가로 나누며, 이번 결과 표는 모두 **미실행**이다. 녹화 기반 시뮬은 모델이 새 영상에 반응하는 검증이 아니다.

## 9. 남은 미확인

새 곡률 연결의 실차 거동과 최종 명령 자료, 실제 모드와 시간 정렬, 평활 잔류 처리, P 재구성 가정의 타당성, 급코너 기준 신호, 모든 신규 수치·공간 관측·복귀 정책, 회피 이식·결합 충돌, 테스트 결과(미실행이므로 오타·임포트 오류 가능성 포함), 차량 상태·실차 효과.
