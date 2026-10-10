# 레인모드 회피 3.1 — 보류 5건 정적 검토 보고서

상태: **코드리뷰 대기 (STATUS=AWAITING_CODE_REVIEW)**. 테스트 **미실행**(실행 미허용). 안전 확보·효과 검증·차량 해결 보고가 아니다.

## 1. 승인·기준·커밋

| 항목 | 값 |
|---|---|
| 원 지시문 | lanemode_avoid_prompt_v3_1.md, SHA256 `54af503a8c81f771597f796dd79d13da0eab3a707770a3107cf654921b342c41` (감사원 2026-10-10 17:16 승인, 요청자 전달 기록 기준) |
| 이어하기 지시문 | lanemode_avoid_v3_1_continue_v2.md (같은 범위, ../cont/) |
| 격리 사본 | 워커 i-043a7af79082c7f70 `/home/ssm-user/work/lanemode_avoid_v3_1/repo`, 브랜치 `lanemode-avoid-v3-1` |
| 기준 커밋 / 트리 | `0e117698806939a80ab25d5611838ab2368e732e` / `2791c2cb77f336a6d52bfd4245cbd37552a58daf` |
| 이식 커밋 | `5e4574ce` — v3 코드 커밋 7361cb5의 기능·테스트 변경만 이식 (리뷰 커밋 a605a227 미이식) |
| WIP 보존 커밋 | `6b57d243` — 1차 실행 시간초과(17:59:26) 시점의 미커밋 수정분을 런처가 남김. 고쳐 쓰거나 되돌리지 않음 |
| 코드·테스트 최종 커밋 | `137afb0d` — 보류 1~5 테스트, 재진입 대기 보존 수정 |
| 리뷰 자료 커밋 | 이 파일을 담은 커밋 (`git log -1 -- REVIEW.md`) |

push 안 함. 원본 저장소·이전 사본 `/home/ssm-user/work/lanemode_avoid_v3`(미추적 자료 포함)은 읽기만 했다.

### 이식 충돌 기록
- 5e4574ce 커밋 메시지 기준: 6개 blob이 7361cb5와 동일하다. 기존 파일 3개(lane_planner_2.py, lateral_planner.py, test_lane_model_speed_planner.py)는 81fae5e와 0e117698의 기준 blob이 같아서 충돌 없이 적용됐다(1차 실행 기록, 이번에 다시 검증하지 않음).
- `git diff 0e117698 HEAD --stat -- openpilot/selfdrive/controls/controlsd.py` 결과는 비어 있다. controlsd.py는 바뀌지 않았으므로 병합본의 시작 준비·차량모델 준비(168~180행), 횡제어 활성 제한(213행), 최초 목표곡률 초기화(229~231행)가 그대로 보존된다.
- 기준 대비 변경 파일은 6개뿐이다(lane_avoid.py, lane_avoid_audit.py, lane_planner_2.py, lateral_planner.py, test_lane_avoid.py, test_lane_model_speed_planner.py). 이 밖의 변경은 없다.
- 호출 순서(lateral_planner.update): `get_d_path`(157) → `pathOffset`(170) → `lane_avoid.refresh()` 후 lane mode면 `update()` + `path_xyz[:,1] += applied`(173~177), lane mode가 아니면 `mode_inactive()`(180). 횡이동량은 기존 카메라·차선·공통 오프셋 뒤에 한 번만 더하며, 비활성이면 경로 수치가 바뀌지 않는다.

## 2. 보류 5건 대응표

경로 접두사는 `openpilot/selfdrive/controls/`. 원본 행은 v3 회수 소스(refs/reviews/lanemode_avoid_cc/src_changed) 기준, 수정 후 행은 137afb0d 기준이다. 테스트 행은 tests/test_lane_avoid.py.

| 보류 | 원본 위치 | 수정 후 위치 | 작성 테스트 | 남은 blocker |
|---|---|---|---|---|
| 1 경로 전체 점유·관측·경계 | lib/lane_avoid.py:171~183 evaluate_side(고정 영역만), :236~269 road_edge_allowance(candidate_x_m만), :186~201·:226~235 레이더 고정 x; lib/lateral_planner.py:167~173 경로 전체에 applied 가산 | lib/lane_avoid.py:233 required_x_range(차체 후단~마지막 점+차체 전단), :248 required_space(0·현재·목표 오프셋 전부, 지점·구간·반폭·여유), :265 evaluate_side(spaces 추가, None=UNKNOWN), :335 road_edge_allowance(전 지점·구간 knot, 외삽 금지), :465 radar_region_x(고정∪동적), :533~555 update에서 매 주기 산출; lib/lateral_planner.py:244 레이더 동적 x | :737 span, :751 먼 지점만 경계 침범, :764 구간 중간·차체 전단 좁아짐, :777 부분 관측·외삽, :787 차체 폭·여유만으로 부족, :799 고정 영역만 덮는 관측, :809 고정 영역 밖 경로 위 레이더 객체, :821 복귀 공간, :835 최종 경로 소비 연결; 기존 :152 정상 사례(좌우 대칭) | 점유 예측(이동 객체) 정책, 차체 기하 모델, 차체 길이·폭·여유 수치, 관측원별 실제 공간 범위, 독립 모델 측면 감지 부재(SRC_MODEL 항상 unsupported → 실제 입력으로는 CLEAR 불가) |
| 2 실제 메시지 시각 신선도 | lib/lateral_planner.py:221~222, :242 경계 age 0.0, :253 model_age 0.0; lib/lane_avoid.py:245, :370~373 | lib/lateral_planner.py:84 평가 시계 `time.monotonic`, :226~282 now−logMonoTime(미수신=None, 경계는 모델 age 상속, model_active 아니면 None); lib/lane_avoid.py:495~506 평가 시각 역행·중복·공백, :508~518 model_t 누락·미래·역행·중복·clock_unverified, :557~561 builder age와 t−model_t 동시 검사 | :860 메시지 고정·시각만 진행, :869 builder age 0 거짓, :889/:899 누락·NaN·미래·역행·반복·시계 미검증(시작/진행 중), :914 한계 양쪽 값, :924 builder 실제 경과·레이더만 오래됨·경계 상속·미래·미수신 | 신선도 수치 전부 미승인. logMonoTime은 발행 시각이고 센서 촬영 시각은 아니다(그 지연은 측정하지 않음). 근거는 cereal/messaging/__init__.py:43(new_message logMonoTime=time.monotonic)·:252(SubMaster update_msgs(time.monotonic()))의 정적 열람뿐이다. carState BSD는 독립 시각·유효 필드가 없어 False는 항상 UNKNOWN이다. 경계 표본의 독립 시각 필드는 없고 꾸미지 않았다 |
| 3 여유 축소 시 적용량 제한 | lib/lane_avoid.py:409~410 목표만 자름, :426~448 제한기 후 적용량, :433~444 RETURN_RISK 유지, :465~485 _step | lib/lane_avoid.py:543 cap(관측된 여유와 max_offset, 미관측이면 0), :665 _step 교집합 선택(room 상한 포함), :628~638 교집합이 없으면 CONFLICT + `constraint_conflict_policy_unapproved`(유지 승인 아님); lane_avoid_audit.py:88~100 모든 프레임에서 applied ≤ edge_room, consumed=applied | :959 연속성 안에서 즉시 축소 가능, :982 여유 축소·0·미관측·방향 반전 × 복귀 안전 여부의 교집합 부재 → CONFLICT·감사 fail, :996 운영 승인 목록 비어 있음·blocker | 교집합 부재 시 정책(constraint_conflict_policy) 미승인. 현재 자리표시는 이동 거부(값 유지)이며 경계 침범 상태로 남는다. 운영 활성화는 blockers()로 차단. 기존 안전 처리(차선변경 보호·운전자 우선)와의 연결은 미확인 |
| 4 강제 영점 예외 제거·연속성 | lib/lane_avoid.py:429~431, :442~443, :476~484 rate 강제 0; tests:518~529(특히 :525~528 목표 도달 표본 제외); lane_avoid_audit.py:56~85 | lib/lane_avoid.py:665~712 _step: 강제 0 없음, 이산 제동 프로파일, 모든 제약을 clip; lane_avoid_audit.py:102~130 실제 차분 변화율과 그 변화를 실제 dt로 매 프레임 검사, 미분 불가 구간은 non_differentiable로 나열(삭제 안 함), :28 최종 명령 신호 미검증 목록 | :1014 목표 도달 포함 전 표본, :1027 불균일 dt·감지 취소·복귀·부호 전환, :1051 진행 중 복귀 위험 전환 → CONFLICT 보고, :1070 도달 프레임 변화율 위반 검출, :1079 첫 표본·공백·불일치·소비값, :1092 최종 신호 미검증 표시. 기존 :518~529는 보존 | 변화율·변화 한도 미승인. 최종 조향·곡률·횡가속 신호의 변화율 검사는 없다(정의·단위·소비 지점만 lane_avoid_audit.py:28에 나열). CONFLICT 프레임에서는 연속성이 깨진다(정책 미승인 → 운영 차단). 불균일 dt에서는 제동 프로파일이 근사다 |
| 5 승인 목록 비활성·재진입 초기화 | lib/lane_avoid.py:53~54 빈 튜플, :127~130 검사, :314~321 초기화; lib/lateral_planner.py:169~173; lib/lane_avoid.py:358~361, :433~444 잔류 | lib/lane_avoid.py:69~78 승인 튜플 5개(모두 빈 값), :133~169 blockers, :444~463 refresh(매 프레임 재검증, 변경 시 초기화, 재진입 대기 보존, note 누적), :473~485 mode_inactive(재진입 대기 보존), :417~423 상태 소유권; lib/lateral_planner.py:173~182 | :1100 모드 끔→여러 주기→켬 첫 적용량 0·상태 초기화·대기, :1124 반복 전환, :1138 planner 게시 경로에 잔류 없음, :1168 설정 끔·다시 켬, :1186 승인 철회, :1198 시간 공백 후 옛 허가 재사용 없음, :1209 수치만 채운 경우 비활성 | 비영 잔류에서 모드 종료나 승인 변경 시 내부 상태를 0으로 초기화하는 전환은 안전 처리로 승인되지 않았다(사유 `mode_exit_with_residual_unverified`·`approval_change_with_residual_unverified`). 시작 준비 미완료·모델 무효 복구는 controlsd 무변경으로 보존되며, 이 기능 쪽 테스트는 모델 무효·시간 공백까지만 다룬다 |

### 이번 이어하기에서 고친 코드 (137afb0d)
- `mode_inactive()`: `reset()`이 `revoked_t`를 지우므로 레인모드 비활성이 두 번째 주기부터 이어지면 재진입 대기가 사라졌다. 이전 값을 보존하도록 고쳤다(lane_avoid.py:480~483).
- `refresh()`: 승인이 바뀌어 초기화될 때도 재진입 대기를 보존한다. 끔→켬 연속 변경에서 잔류 폐기 사유가 덮어써지지 않도록 note를 누적한다(lane_avoid.py:449~462).

## 3. 테스트 준비 객체 충돌 (기존 assert 원문 보존)

기존 확인 문은 삭제·약화하지 않았다. `git diff 5e4574ce HEAD`로 삭제된 테스트 행은 준비 객체 7행뿐이며 모두 대체됐다.
- `FULL_COV (-15,40)` → `(-15,115)`, 경계 x를 `X(0..100)` → `EDGE_X(0..110)`로 바꿨다. v3.1의 필요 공간이 차체 후단~마지막 점+차체 전단(−1~104 m)이라 기존 값으로는 모든 사례가 UNKNOWN이 된다.
- TEST_CFG에 v3.1 필드(vehicle_front_m=4.0, vehicle_rear_m=1.0, side_clearance_m=0.3, 정책 3종 TEST_POLICY)를 넣었다. `approved_policies`는 튜플 5개를 테스트 안에서만 패치한다. **합성 입력이며 운영 승인값이 아니다.**
- `inp()`에 `model_t=t, clock_verified=True`를 추가하고, `frame()`에 `rate`, `edge_room`을 추가했다(감사 검사 강화).
- **예상 충돌(미실행, 정적 추론):** `test_new_avoid_side_detection_revokes_and_never_grows`의 `all(o.state in (RETURN, RETURN_RISK))`와 `audit(...)['violations'] == []`. 진행 중(변화율 0.3 m/s)에 회피 쪽이 감지되면 증가도 복귀도 금지되고, 변화율을 한 프레임 안에 0으로 만들 수 없어 v3.1에서는 CONFLICT가 되고 변화율 연속성 위반이 기록된다. 원문을 보존했고, 새 테스트(:1051)가 이 동작을 명시한다. 해결하려면 constraint_conflict 정책 승인이 필요하다.
- `test_road_edge_not_extrapolated_and_shape_checked:333`(X 경계 + NaN)은 이제 x 범위 부족만으로도 UNKNOWN이 되어 NaN 판별력이 약해졌다. assert 자체는 바꾸지 않았다.

## 4. 승인 설정 목록 (전부 미승인, 빈칸을 임의 수치로 채우지 않음)

미설정 동작: 아래 항목 중 하나라도 비거나 무효면 `blockers()`가 비지 않는다. 그러면 `active=False`, 상태는 DISABLED, `lane_avoid_out=None`이 되고 경로는 바뀌지 않는다(추가 적용량 0).

| 분류 | 항목 | 승인 |
|---|---|---|
| 기능 | enabled | 미승인(False) |
| 수치 | max_offset_m, entry_rate_mps, return_rate_mps, max_abs_curvature, candidate_x_m, candidate_deadband_m | 미승인 |
| 감지 영역 | required_region_x_m, required_region_lat_m, coverage{(bsd/radar/model, left/right)} | 미승인 |
| 차체·여유 | vehicle_half_width_m, vehicle_front_m, vehicle_rear_m, side_clearance_m, road_edge_margin_m, road_edge_std_max_m, body_geometry_model | 미승인 |
| 시각·신선도 | max_age_s{bsd, radar, model, road_edge, model_path}, max_input_gap_s, 시계 대응 근거 | 미승인 |
| 변화율·연속성 | max_rate_change_mps2(적용량의 2차 미분), 최종 조향·곡률·횡가속 변화율 한도 | 미승인 / 정의 없음 |
| 복귀·유지 | return_risk_policy, center_invalid_policy, constraint_conflict_policy, occupancy_prediction_policy | 미승인(튜플 비어 있음) |
| 재진입 | entry_confirm_frames, reentry_wait_s, 잔류 중 모드 종료·승인 변경 전환 정책 | 미승인 |
| 판정 기준 | 감사 `no_violation_found`는 합격이 아니다. 합격·개선 판정 기준은 따로 승인받은 재생 평가 절차가 정한다 | 미승인 |

## 5. 쏠림·코너 3판과의 충돌표 (자동 병합·조합 활성화 금지)

쏠림·코너 작업의 소스는 이번에 열람하지 않았다. 아래는 이 작업이 건드리는 지점만 적은 것이다.

| 파일·지점 | 이 작업 | 충돌 가능성 |
|---|---|---|
| lateral_planner.py:170~182 경로 오프셋 가산 순서 | pathOffset 뒤에 applied를 한 번 가산 | 쏠림·코너가 같은 경로에 오프셋을 더하면 중복 가산이나 제한 우회가 생길 수 있다. 중앙 제한과 복귀 처리 소유권을 정해야 한다(미확인) |
| lateral_planner.py:226~282 입력 조립 | base_y = 현재 path_xyz y | 다른 기능의 오프셋이 먼저 더해지면 base가 바뀌고 경계 여유 계산도 달라진다(미확인) |
| lane_planner_2.py:234~237, :252 복사본 | model/lane y와 d_prob 복사(경로 무변경) | 같은 혼합부를 수정하면 문맥 충돌이 난다(미확인) |
| controlsd.py | 무변경 | 해당 없음 |

## 6. 미해결·미확인
- 테스트 실행 결과 없음(실행 미허용). 구문·불러오기 검사도 하지 않았다. 위 기대값은 모두 정적 추론이다.
- 관측원의 실제 공간 범위, 독립 모델 측면 감지(없음), 도로 경계 품질(가드레일 검출이 아님), 시계 근거(정적 열람만), 승인 수치·정책 전부.
- 안전 제약 교집합이 없을 때의 연결 정책, 잔류 중 모드 종료·승인 변경 시 전환 정책, 최종 조향·곡률 명령의 연속성.
- 쏠림·코너 작업과의 기능 간 충돌, 현재 차량 코드·설정, 실차 효과.
- 비활성이라는 사실은 결함 수정 완료나 안전 검증을 대신하지 않는다.

향후 검사 절차는 RUN_AFTER_REVIEW.md에 문서로만 적었다(실행하지 않음).
