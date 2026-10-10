# 쏠림·코너 v4 — 이전 보류 3건과 18:58 보류 3건 정적 검토 보고서

- 지시문: refs/lanebias_corner_prompt_v4.md, SHA256 74e6271dec7c8b1456e603e15eafbb0a5ffec493d3b39c4a0931f28dbf74477f. 실행 근거는 회장 2026-10-10 19:43 사람 승인이다(refs/APPROVAL.md, SHA256 6e77278922acc91d39fb64d76c2458c237f71db4ee67a257bf628413adf38bf5). 이 승인은 지시문 머리의 "감사원 승인 대기"를 대체한다. 감사원에는 제출하지 않았다.
- 기준 커밋: 931f447e2b45218f9cc55ffae8d225632408656f, 트리 151ad180ad2710897327cb730a0d12d3d52e7685. 병합 기준 0e117698 계보는 그대로 두었다(되돌림·재이식 없음).
- 격리 경로: /home/ssm-user/work/lanebias_corner_v4/repo. 최종 로컬 커밋 식별자는 `git log`로 확인한다. 이 파일이 그 커밋 안에 들어가므로 자기 식별자를 적을 수 없다.
- 원본 저장소(lanebias_corner_v3/repo)는 수정하지 않았다. 지시문이 말한 `/home/ssm-user/work/reviews/...` 제출 꾸러미 사본은 이 작업 환경에 없다. 사실 확인은 기준 커밋 소스를 직접 열어 했다. 회피 쪽은 lanemode_avoid_v3_1/repo 안의 커밋 9bd8d65b를 `git grep`으로 읽기 전용 열람했다. 회피 3.2 작업 사본은 보유하지 않았다.
- **시험·구문 검사·불러오기·빌드·원로그·차량·push는 모두 미실행이다.** 이 작성 단계에서는 아무것도 실행하지 않았다(부모 지시). 아래 내용은 작성 결과일 뿐이다. 통과·개선·안전 확보를 뜻하지 않는다. 작성한 시험에는 오타나 임포트 오류가 있을 수 있다.

## 0. 시험 명령과 의존성(부모가 별도 기록으로 실행)

```
cd /home/ssm-user/work/lanebias_corner_v4/repo
python -m pytest -q \
  openpilot/selfdrive/controls/tests/test_laneless_center.py \
  openpilot/selfdrive/controls/tests/test_laneless_center_v4.py \
  openpilot/selfdrive/controls/tests/test_lat_error_decomposition.py
```

- 필요한 것: Python 3, numpy, pytest. 저장소 루트를 import 경로에 두어야 `openpilot.*`와 `opendbc.*`를 찾는다. drive_helpers.py:1~6이 불러오는 `openpilot.cereal`(log, capnp 생성물), `opendbc.car.vehicle_model`, `opendbc.car.volkswagen.values`, `openpilot.common.realtime`, `openpilot.selfdrive.modeld.constants`도 불러올 수 있어야 한다. v3 시험과 같은 의존성이며, 새 외부 패키지는 없다.
- controlsd.py 전체를 불러오는 시험은 없다. controlsd 연결은 소스 문자열·순서 검사로 확인한다. 이 검사는 정적 확인이며 실행 연결 증명이 아니다.

## 1. 변경 파일

| 파일 | 내용 |
|---|---|
| openpilot/selfdrive/controls/lib/laneless_center.py | 설정 수치 검증(`_real`, `_CONFIG_RULES`, `complete/errors/approved`); 관측·안전 계약(`SideClearance`, `ClearanceObservation`, `CandidateSafety`, `APPROVED_SAFETY=None`, `side_clearance_reason`); `RESIDUAL_POLICY_APPROVED=False`; `operational` 게이트; 기준 체인 소유와 시드 독립성(`reset`, `baseline_desired`); 후보 안전 검사(`safety_reason`); 매 주기 설정 재검증; 해제·잔류 미해결 래치; `check_consumed` 유한성·안전 재검사; `consume`(후보 검사 → 재계산 → 재검사 → 확정) |
| openpilot/selfdrive/controls/lib/lat_error_decomposition.py | `_real`; `decompose`는 허용치 유한·≥0 검증, 사유(`reason`) 기록, 결과 유한성 검사; `safe_ratio`는 하한 유한·>0, ±0 분모 선거부, 몫 유한성 검사 |
| openpilot/selfdrive/controls/controlsd.py | import 29; `laneless_center_delta`가 원 모델 직접 곡률을 넘기고(:183) `clearance=None`을 넘긴다(:191); `consume_laneless_center`(:194~202)는 라이브러리 `consume`에 위임하고, 원 모델 곡률·검증기 후 곡률·modelV2 frameId·수신 프레임·소비 프레임을 기록한다; 분기 게이트는 `operational`이다(:322) |
| openpilot/selfdrive/controls/tests/test_laneless_center_v4.py (신규) | 이번 3건과 연결 시험(미실행) |
| openpilot/selfdrive/controls/tests/test_lat_error_decomposition.py | 끝에 v4 허용치·비율 시험을 추가(미실행). 기존 시험은 그대로 두었다 |
| REVIEW.md | 이 보고서. v3 판은 931f447e:REVIEW.md에 남아 있다 |

기존 test_laneless_center.py는 수정하지 않았다. 기존 assert는 삭제·약화하지 않았다. 기존 시험이 쓰는 `enabled`, `shadow_desired`, `check_consumed`, `update` 인터페이스는 유지했다. path_verifier.py, lane_planner_2.py, lateral_planner.py, lat_mode_blend.py, latcontrol_*는 변경하지 않았다. 시작 준비·횡제어 활성 제한·최초 목표곡률 초기화(:206, :249, :265~267), VW MEB 분기, 기본 꺼짐 분기의 기존 줄(:326~329, :338)도 그대로다.

## 2. 대응표(원본 행 = 931f447e, 수정 행 = 이번 작업 트리)

| 보류 | 원본 | 수정 | 작성 시험 | 미해결 |
|---|---|---|---|---|
| 이번1 소비 실패 후 오염 상태 재사용 | controlsd.py:193~208(재계산값 미검사, 그림자 시드=prev); laneless_center.py:35~38, 173~176 | laneless_center.py `consume`:540~(첫 검사 → delta 0 재계산 → **재검사** → 실패하면 `unresolved=recompute_failed` 래치, `final_status` 기록); `reset`:348~(시드 독립성 `baseline_independent`); `check_consumed`:519~(비유한 거부); update 설정 무효 분기(진행 중 종료 + 잔류 → `handover_with_residual` 래치) | test_normal_intersection_passes_first_check_and_stays_in_bounds, test_residual_after_interval_change_is_rechecked_and_not_reported_valid(중앙 0/반전), test_block_with_residual_in_state_is_flagged_not_silently_emitted(운전자·차선변경·모델무효·관측무효·모드·나이 NaN), test_setting_cancel_mid_run_is_flagged, test_curvature_clip_saturation_is_limited_not_violation, test_reentry_without_baseline_marks_seed_not_independent, test_never_requesting_chain_is_bit_identical_to_original, test_off_from_start_and_stop_mid_run_are_different_cases, test_nonfinite_consumed_value_fails | **재검사도 실패할 때 무엇을 내보낼지(유지/순간 0/기준 복귀/경사)는 사람 결정 필요다.** 지금은 v3의 delta 0 재계산값을 그대로 내보낸다. 이 값은 미검증으로 표시하고 `operational`을 영구 False로 래치한다. 평활 잔류를 연속성을 지키며 지우는 처리는 미구현이다. 꺼짐 상태로 시작했을 때의 동일성과 동작 중 종료 시 상태 인계는 별개다. 동작 중 종료 뒤 controlsd 기본 분기는 잔류가 든 `desired_curvature`에서 이어간다. 이 인계는 미해결이며 래치로 표시만 한다. latActive 거짓으로 인한 reset은 래치하지 않는다(기존 해제: `desired_curvature=self.curvature`, clip 경사 포함). |
| 이번2 후보 안전 검사·이동 차단 미연결 | controlsd.py:326~330(검증기 뒤에 보정); laneless_center.py:170, 195~201, 229~232, 262~274 | `CandidateSafety.candidate_check(x, y)`: 보정 **후보 경로**(모델 위치 + 재구성 보정, 조밀화 격자)를 받는 무상태 계약. `safety_reason`:370~: 후보 검사 → 점유 띠 쪽과 이동 쪽(구축·해제 모두) 공간 검사 → `side_clearance_reason`(미지원·무효·오래됨·범위 부족·감지·접근·빈 공간 근거 없음·공간 부족은 모두 차단). update:491~ 후보 실패 시 확정하지 않고 해제 경사로 바꾼 뒤 해제도 검사한다. 해제도 실패하면 `release_unverified` 래치. `check_consumed`는 최종 소비 dk를 직전 소비 dk 대비 다시 검사한다 | test_candidate_path_not_raw_model_path_is_checked, test_candidate_only_invades_barrier_never_committed, test_move_side_not_proven_free_blocks_even_with_good_lanes_and_zero_verifier(12사례), test_unobserved_clearance_blocks, test_both_sides_occupied_blocks_either_direction, test_left_right_symmetry_of_side_selection, test_verifier_authority_zero_is_not_a_pass(False/1/np.True_/예외), test_release_without_geometry_is_unresolved, test_return_only_dangerous_is_unresolved, test_consumed_value_rechecked_against_clearance, test_library_never_advances_path_verifier_state, test_controlsd_gates_on_operational_and_delegates_consumption | **안전 계약 구현이 없다.** 기존 PathVerifier.update(path_verifier.py:813~)는 주행 거리계·궤적 상태를 진행시키므로 한 주기에 두 번 부를 수 없다. 무상태 후보 검사기는 미구현이다. controlsd에는 차량·장애물·BSD·도로 경계를 공간·범위·시각과 함께 주는 입력이 없다. CS.leftBlindspot 같은 참/거짓 신호를 빈 공간 근거로 지어내지 않았으므로 `clearance=None`, 즉 미지원 차단이다. 차체 폭·모서리·여유(`lateral_margin`)와 관측 범위·나이는 사람 결정 필요다. 이동 중 객체 접근은 관측 제공자의 `approaching` 신호에 맡긴다. 그 신호 생성은 미구현이다. 재구성 dk·x²/2(소각도, 일정 곡률)는 `required_range_x`까지 외삽해 필요 공간을 잡는다. 이것은 가정이다. 회피 활성·잔류 상태 연결은 미구현이다(§6). |
| 이번3 수치 유효성 | laneless_center.py:81~82; lat_error_decomposition.py:89, 94, 101~107 | `LanelessCenterConfig.errors()`:141~, 필드 규칙 `_CONFIG_RULES`; `approved()`=완전+수치 유효(사람 승인은 `APPROVED_CONFIG`로 별개); update에서 매 주기 재검증(`config_invalid`). decompose:88~ / safe_ratio:122~ | test_every_field_rejects_unset_nonnumeric_bool_nonfinite_negative(10필드×9값), test_field_specific_ranges_rejected, test_mathematical_boundaries_are_numerically_valid, test_valid_numbers_are_not_human_approval, test_config_mutated_at_runtime_is_rejected_next_cycle, test_invalid_safety_contract_is_not_connected; 분해·비율: test_invalid_tangential_tolerance_is_unevaluable_with_reason, test_unset_tolerance_reason, test_zero_tolerance_accepts_only_exact_progress, test_tolerance_boundary_is_inclusive, test_large_finite_inputs_with_large_mismatch_not_ok, test_ratio_invalid_lower_bound, test_ratio_signed_zero_denominator_rejected_before_division, test_ratio_bound_inclusive_and_negative_denominator, test_ratio_nonfinite_result_is_not_a_number, test_ratio_missing_or_nonnumeric_numerator_no_exception | 운영 상·하한은 전부 사람 결정 필요다(아래 표). `safe_ratio`에는 저장소 안 소비자가 없다(시험만 호출). 그래서 "소비 측까지" 전달 검사는 반환값 None 검사로 대신했다. 범위 밖 값은 잘라 쓰지 않고 거부한다. |
| 이전① 소비 곡률 연결 | v3 대응 유지 | 기록: `model_raw`(원 모델)와 `verified`(검증기 후) 분리, `model_frame_id`, `model_recv_frame`, `consume_frame` 추가; 불일치 검사는 원 모델 곡률로 한다 | 위 controlsd 순서 시험 + v3 시험 | trace는 메모리에만 있고 cereal 필드가 없다(스키마 변경 미승인). carcontroller 전송 직전 명령과 차량 응답은 기록하지 않는다(미관측). |
| 이전② 지점별 중앙 초과 금지 | v3 대응 유지 | 최종 소비 dk도 같은 점 집합에서 재검사한다(재계산값 포함) | v3 시험 + 이번1 시험 | B1(레인모드) 미구현. 재구성 가정은 v3와 같다. |
| 이전③ 오차 분해 | v3 대응 유지 | 허용치·비율 검증 강화 | 위 | 실제 로그 정렬은 미실행이다. 합 보존은 장부 항등식이다. |

## 3. 입력 → 후보 → 안전 검사 → 최종 소비 연결표(operational=True를 가정; 현재는 불가)

| 단계 | 위치 | 비고 |
|---|---|---|
| 1 원 모델 직접 곡률 | controlsd.py:301 `laneless_curvature`; 기록 `model_raw` | 불일치 검사 입력(:183) |
| 2 PathVerifier | :320~321 | effect≠0이면 차단한다. effect=0은 통과 근거가 아니다 |
| 3 보정 요청·후보 | :324 → update:406~ | 차선 중앙 구간, 요청, 경사 |
| 4 후보 안전 검사 | update → `safety_reason` | 후보 경로 검사기, 점유·이동 공간 검사. 실패하면 확정하지 않고 해제한다. 해제도 실패하면 래치 |
| 5 혼합·평활·clip | `consume` → `consume_curvature`(후보) + 기준 체인 | 기준 체인 상태는 라이브러리가 소유한다 |
| 6 최종 소비 검사 | `check_consumed` | 유한성, 지점별 중앙, 안전 계약(직전 소비 dk → 이번 dk) |
| 7 재계산·재검사 | `consume` | 실패하면 `final_status=unresolved`, 래치 |
| 8 확정 | `self.desired_curvature`(:333) → actuators.curvature(:340) → LaC.update(:373~) | trace에 torque·steering_angle 기록 |

현재 `operational`은 다음 이유로 항상 False이다. `APPROVED_CONFIG=None`, `APPROVED_SAFETY=None`, `RESIDUAL_POLICY_APPROVED=False`. 따라서 controlsd는 :326~329 기존 줄을 실행한다. 이 차단은 운영 활성화 차단이며 결함 해결이 아니다.

## 4. 설정 수치 표(모두 미승인 기본값 None)

| 필드 | 단위 | 수학적 유효범위(구현) | 운영 승인범위 | 미설정/무효 동작 |
|---|---|---|---|---|
| min_lane_prob | 확률 | [0,1] | 사람 결정 필요 | 비활성/`config_invalid` |
| max_lane_std | m | >0 | 사람 결정 필요 | 〃 |
| check_x_min | m | >0(dk=2dy/x² 정의) | 〃 | 〃 |
| check_x_max | m | >check_x_min | 〃 | 〃 |
| eval_x | m | [check_x_min, check_x_max] | 〃 | 〃 |
| gain | 비율 | [0,1](0=무보정) | 〃 | 〃 |
| max_path_direct_mismatch | 1/m | ≥0(0=완전 일치만) | 〃 | 〃 |
| max_model_age | s | >0 | 〃 | 〃 |
| max_delta_rate | 1/m/s | >0 | 〃 | 〃 |
| max_lane_mode_weight | 가중치 | [0,1] | 〃 | 〃 |
| CandidateSafety.max_obs_age | s | >0 | 〃 | 계약 미연결(None) |
| CandidateSafety.required_range_x | m | >0 | 〃 | 〃 |
| CandidateSafety.lateral_margin | m | ≥0 | 차체 수치 포함, 사람 결정 필요 | 〃 |
| decompose max_tangential | m | 유한 ≥0(0=정확 일치), 경계 포함 | 〃 | 평가불가 + reason |
| safe_ratio min_abs_denominator | 분모 단위 | 유한 >0, \|분모\|=하한은 허용 | 〃 | None |

모든 수치에서 bool·문자열·None·NaN·Inf는 거부한다. 시험의 TEST_CFG·safety() 수치는 **미승인 기본값 — 합성 시험 전용**이다. DENSIFY=4는 검사 밀도이며, 지점 사이 전체 안전을 증명하지 않는다.

## 5. 중앙 제한·오차 분해 계약

v3 §4·§5를 이어받는다. 추가 사항은 다음과 같다. (1) 최종 소비 dk는 첫 계산값과 재계산값 모두 같은 점 집합에서 검사한다. (2) 기준 체인 시드가 잔류를 품었을 수 있으면 `baseline_independent=False`로 기록한다. 이 경우 그 기준 대비 dk는 독립 무보정 기준 대비 값이 아니다. (3) 분해 결과에 평가불가 사유를 남긴다. 유한 입력이라도 결과가 비유한이면 평가불가다.

## 6. 회피(9bd8d65b)와의 충돌표(정적)

| 항목 | 회피(9bd8d65b) | 이번 변경 | 판정 |
|---|---|---|---|
| 파일·위치 | lateral_planner.py:173~182(`path_xyz[:,1] += lane_avoid_out.applied`, 레인모드 계획) | controlsd.py 레인리스 분기와 lib | 같은 줄은 없다. 둘 다 lat_mode 혼합을 거쳐 `desired_curvature`로 간다 |
| 모드 | 레인모드 계획 → lat_plan.curvatures → lane_curvature | 레인리스 후보(w ≤ max_lane_mode_weight) | 0<w<1 혼합 구간에서 두 기여가 함께 들어갈 수 있다. 결합 규칙은 미승인이므로 결합 비활성·사람 결정 필요 |
| 상태 소유 | lane_avoid(활성/잔류/복귀) | laneless_center(delta, 기준 체인) | 회피 상태를 이 모듈 차단 입력으로 넘기는 연결은 **미구현**이다. 자동 결합하지 않았다 |
| 중앙 제한 | 회피는 중앙을 벗어날 수 있다 | 중앙 너머를 금지한다 | 회피 중 중앙 보정이 회피를 되돌릴 위험이 있다. 교집합 판정은 미구현 |

## 7. 후보별 상태

- A1: 철회 유지. 변경 없음.
- A2: 미착수.
- A3: LatMpcInputOffset 2/0 비교 명세·시험 미작성(미착수).
- B1: 미구현.
- B2: v4 보강(비활성). 급코너 감쇠는 미구현·미해결.
- C: 조합 충돌 시험 미작성(B1 부재).

## 8. 향후 평가 명세(전부 미실행)

v3 §8 절차를 따른다. 코드리뷰·별도 실행 승인 → 입력·시각·좌표·명령 출처 확인 → §0 단위 검사 → 꺼짐 기준선 재현 → 실제 모드·소비 확인 → 승인된 단독 후보 비교 → 코너·직진·정상 장면·복귀 평가. 지표는 쏠림과 코너 안쪽 물기의 평균/최대, 중앙 반대편 초과, 정상 장면 악화, 곡률·조향 명령 변화율, 복귀 지연이다. 개선/실패/평가불가로 구분한다. 미활성 0건은 개선으로 세지 않는다. 승인 한도가 없으면 평가불가다.

## 9. 남은 결함·미확인

1. 재검사 실패 시 내보낼 값과 잔류 제거·연속성 정책(사람 결정 필요). 현재는 미검증 값을 내보내고 래치한다.
2. 무상태 후보 경로 검사기와 측방·경계 공간 관측 제공자가 없다. 차체 수치와 여유도 미정이다.
3. 회피 상태 연결과 결합 교집합 판정.
4. cereal 기록, 전송 직전 명령, 차량 응답.
5. latActive 해제 시 잔류는 기존 clip 경사로 처리한다(래치 대상 아님).
6. 지시문 §8.2의 전체 소스 사본·지문 목록(SHA256SUMS)은 이 사본에서 만들지 않았다(부모가 회수 단계에서 준비).
7. 작성 시험의 실행 결과는 없다. → §10 격리 실행 결과로 대체(통합시험 아님).

## 10. 시험 실패 수정 1회(회장님 승인 범위, 2026-10-10)

- 부모 초회 격리 실행(`--noconftest`): 251통과 1실패. 실패는 `test_laneless_center.py::test_every_point_and_interpolated_point_stays_between_path_and_center[straight]`이며, 마지막 점(x=40)의 재구성 오프셋이 0.4를 1ulp가량 넘었다.
- 원인: 모듈은 차선선 중점 `0.5·(ly+ry)`로 중앙을 구한다. 시험 입력(0.4∓1.8)의 중점은 0.40000000000000013이라 실제 중앙보다 ulp만큼 바깥이고, 그 위에서 계산한 상한 `d·2/x²`를 `dk·x²/2`로 재구성할 때도 반올림 초과가 생길 수 있다.
- 수정(보수적으로만 좁힘, 정책·허용오차·활성화 변경 없음):
  - `update`: 검사 구간 중앙을 경로 쪽으로 `4·eps·(|ly|+|ry|)`만큼 당기며 경로를 넘지 않는다(약 1e-15 m).
  - `curvature_delta_interval`: 양 끝값을 정확한 재구성이 경계 안에 들어올 때까지 `nextafter`로 0 쪽으로 옮기며, 최대 64회 뒤에는 0으로 둔다.
  - 새 재현 시험 `test_interval_endpoints_reconstruct_inside_bounds_despite_rounding`. 기존 단언은 바꾸지 않았다.
- 재실행: `PYTHONDONTWRITEBYTECODE=1 /home/ssm-user/work/py312_ctl/bin/python -m pytest --noconftest -o addopts= -p no:cacheprovider -q openpilot/selfdrive/controls/tests/test_laneless_center.py openpilot/selfdrive/controls/tests/test_laneless_center_v4.py openpilot/selfdrive/controls/tests/test_lat_error_decomposition.py`. 종료코드 0, 253통과 0실패.
- 이 결과는 conftest 없는 격리 실행이며 전체 통합시험 통과가 아니다. 일반 pytest는 `msgq.ipc_pyx` 부재로 준비 단계에서 실패했다(미해결).
