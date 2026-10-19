# K9StopTaper 야간 시뮬레이션 결과 (ALT11, 2026-10-20 00:15–05:00 KST)

English follows the Korean section.

## 요약
- 기준 커밋 632267bb, `k9_stop_taper.py` sha256 9c43708b…(캠페인 내내 작업 트리와 동일). **제품 코드와 임계값은 바꾸지 않았다.** 기본 OFF, StopReq·정차 유지/소프트홀드 시점은 그대로다.
- 모든 캠페인은 05:00:00에 스스로 끝났다(완료 표시: v1 worker 5/5, v2 worker 2/2, v3 민감도 Phase A 5472/5472).
- **확정된 모듈 계약 위반은 0건**이다(프레임 불변식 위반 0, fuzz/replay 오류 0, 음성 대조 불일치 0).
- 그러나 **단순 모델(K9 미보정) 안에서 원래 명령보다 나빠지는 경로가 재현되었다.** 원래 명령은 섰지만 ON은 서지 못하거나 0.5m 넘게 더 가는 경우, 새 앞차 접촉, 출발 지연이다. 따라서 "원래보다 나빠지지 않는다", 쾌적성 개선, 안전성, 실차 효과 중 어느 것도 주장할 수 없다.
- 이전 adverse 증거는 그대로 유지한다: 12% 내리막 +0.453m, 피치 흔들림 1.772배, 1,776건 중 204건 양쪽 미정지.
- 소유자 결정이 필요한 질문 Q1–Q3(아래)는 구현하지 않았다.

## 실행량과 고유 커버리지 (반복과 구분)
| 캠페인 | 실행 수 | 고유 커버리지 | 반복·한계 |
|---|---|---|---|
| v1 closed (실제 LongControl + 모듈, 무작위 plant) | 697,112 | seed마다 다른 무작위 케이스 | OFF 자체 실패(info) 191,658건(27.5%)은 비교 불가 |
| v1 fuzz (모듈 직접, 무작위 입력열) | 349,827 | seed마다 다른 입력열 | 632267bb와의 차분은 cur=base라 공허함 |
| v1 replay (기록 K9 정차 73개에 섭동) | 116,667 | **기본 정차 73개뿐** | 정차당 약 1,600회 섭동 반복, 개루프 |
| v2 rand2 / grid / focus | 15,317 / 15,316 / 15,316 | grid는 **4200칸 중 2800칸만**, 전부 jitter(정확 칸 0) | focus는 v1 플래그 seed 주변을 일부러 과표집 → 비율은 대표성 없음 |
| 정확 grid 감사 | 4,200 | 4200/4200칸 1회씩, jitter 없음 | 원래 planner, dead 0.1, a_app 1.0 고정 |
| v3 Phase A (정확 factorial) | 5,472 | 항목 152 × 축 36, 모두 고유 | 항목은 기존 REG/warn 칸·대표 seed에서 선택 |
| v3 Phase B (무작위 확장) | 43,860 | 같은 152 항목 주변의 무작위 섭동 | 새 영역 탐색이 아니라 이웃 반복, 전수 아님 |

음성 대조: v2 OFF/OFF·ON/ON 비트 동일 4,596/4,596, 정확 grid 210/210, v3 OFF/OFF·ON/ON·null-taper==OFF 1,907/1,907.

## v1/v2 지표 결함과 v3에서 바로잡은 구분
- v1(하네스 결함, 결과는 보존): 첫 접촉을 정지로 본 t_stop, OFF/ON 비대칭 관측창과 30s 상한, 정지점을 지나도 재계획하지 않는 planner 대역, 흔들림 비율의 작은 분모, 미정지를 출발 지연으로 셈. 그래서 v1 최대 dx +7.394m와 흔들림 38.784배는 하네스에 민감한 값이며 실차 크기가 아니다.
- v2: 동일 관측창, 지속 정지 D(0.3/1/2/3s), censoring, OFF 실패 분리(off_infeasible), replan ablation, 음성 대조를 추가했다. 남은 결함 두 가지가 확인되었다.
  1. grid seed 식 때문에 한 worker가 mod-3 잔여류만 방문하여 **2800/4200칸만 도달**하고 모두 jitter되었다(문서의 "첫 바퀴 정확 칸"은 틀림).
  2. 출발 명령이 ON의 정지를 **선점**하면 `REG_on_not_stopped`로 잘못 분류하거나(dwell 중이면) 거리 검사를 생략했다.
- v3: 선점(`PREEMPT_on_before_zero/in_dwell`)을 따로 분류하되 **거리·접촉은 선점과 별개로 평가**한다. 표본 재분류에서 v2 미정지 96건 중 93건이 선점이었지만 그중 51건은 여전히 dx>0.5m, 41건은 dx>0.25m였다. 즉 선점은 이름만 바꿀 뿐 악화 증거를 지우지 않는다.

## 최종 수치 (회귀 플래그는 시험 정의이며 차량 임계값 아님)
- v1 closed: REGRESSION 1,105 = 추가거리>0.5m 545, ON 미정지 296, 출발 지연 184, 새 앞차 접촉 80. warn 4,187. fuzz·replay 플래그 0.
- v2(원래 planner, D=1s): rand2 REG_dx 4 + 미정지 2(15,317건 중), grid(jitter) REG_dx 118 + 미정지 28, focus REG_dx 638 + 미정지 1,136 + 새 앞차 접촉 277 + 출발 지연 401. v2 미정지에는 위 선점 오분류가 섞여 있다.
- 정확 grid: ok 2,925, off_infeasible 1,125, warn_dx 87, **REG_dx>0.5 57**(최대 2.07m), 미정지 0. REG는 OFF가 겨우 설 수 있는 경계 칸(급내리막, 큰 비선형 제동 이득, 큰 지연)에 몰린다.
- v3 Phase A(D=1s 또는 접촉 REG): planner orig 399/2,736, replan 135/2,736. ESC hold 가정별 v0_only 443/2,736, `ASSUMED_stopreq_moving` 91/2,736. 같은 config의 hold 쌍에서 v0_only REG 443건 중 353건이 가정 변경으로 사라지고 90건은 남는다. **가정 쪽에서만 새로 생긴 REG가 1건 있다**(v0_only에서는 OFF가 서지 못해 비교 불가였으나, 가정에서는 OFF는 서고 ON은 서지 못함).
- v3 Phase B: REG 2,651/43,860(이웃 반복). hold별 v0_only 2,234/21,915, 가정 417/21,945.
- 해석: 남은 악화 경로 대부분은 "ESC는 v=0에서만 정차 유지"라는 plant 가정, 지연, 비선형 이득에 민감하다. 실제 K9 ESC 동작이 미보정이므로 어느 가정도 사실로 보지 않는다. 가정을 바꿔도 악화가 모두 사라지지는 않는다.

## 새 회귀 시험 (`openpilot/selfdrive/car/tests/test_k9_stop_taper_properties.py`)
시뮬레이터 없이 현재 계약만 검사한다. 정차 거리, 끄덕임, 쾌적성, 안전성은 검사하지 않는다.
- seed 32개 × 6,000프레임 입력열(정차·standstill 깜빡임·인터록·비유한/거대 유한값·급제동·앞차 불일치·레이더 전용 근접 앞차·약한 감속)에서 매 프레임 계약을 검사한다: 원래보다 세지 않고 0 이상으로 풀지 않음, 인터록·비유한이면 원래 값과 disarm, 급제동·근접 앞차면 원래 값, 해제율·복원율 한계, 독립적으로 다시 쓴 floor 식, 에피소드마다 REARM_SPEED 재무장 필요.
- 모든 종료 사유(11개)가 도달되는지와 실제 taper가 충분히 실행되는지 확인한다. reset()이 새 객체와 같고 결정적임을 확인한다.
- Q1 특성 시험: 유한 1e9 v_ego 한 프레임이 abort 뒤 재무장시킨다(standstill 플래그가 있으면 재무장 안 함). 소유자가 규칙을 바꾸면 이 시험도 바뀌어야 한다.
- CarController: OFF(누락/false)에서 K9·K7 모두 전환 전 컨트롤러(38df0d7c)와 CAN 비트 동일(seed 3개 × 1,200프레임), ON에서 K7 동일, K9은 StopReq/ACCMode 유지·aReq가 세지지 않음.
- 오탐·공허함 감사(저장소 밖 변이 시험): 모듈 사본 변이 12개 중 11개가 잡혔다. 남은 1개(`left_window` 뒤 armed 유지)는 phase가 done이라 재무장 전에는 진입할 수 없어 관측 불가한 등가 변이다. 처음 floor 검사는 모듈 함수를 그대로 써서 변이를 놓쳤기 때문에 독립 식으로 바꿨다.

## 소유자 결정 대기 (구현하지 않음)
- Q1: 단발 유한 이상값(v_ego 1e9)이 REARM_SPEED 재무장을 허용한다. 연속 프레임 조건이나 타당 범위 검사를 넣을지. 기록 K9 정차 창 75,565 carState 프레임에서는 제어 중 이런 스파이크가 관측되지 않았다.
- Q2: 정지 여유가 거의 없는 급내리막·약한 제동에서 taper가 남은 여유를 소비한다. 경사 추정으로 진입을 막을지.
- Q3: 감속하는 앞차에서 늦어진 정지가 더 센 최종 제동과 출발 지연으로 이어지는 모델 경로를 허용할지.

## 한계와 음성 증거
- 모든 plant 요소(지연, 비선형 이득, ESC hold 두 가정, 피치, 마찰)와 planner 대역은 K9 보정이 아니다. 기록 정차 replay는 개루프다. D, 0.25/0.5m, 0.1s는 시험 정의다.
- 기록 로그·경로·케이스 원자료는 비공개로 남기고 이 문서에는 집계만 싣는다.
- 효과 증거 없음: 피치·승차감 측정이 없으며, 이전 기록 K9 정차 4건 관찰은 aReq 영향이 작을 수 있음을 시사할 뿐 짝지은 시험이 아니다.

---

# K9StopTaper overnight simulation outcome (ALT11, 2026-10-20 00:15–05:00 KST)

## Summary
- Baseline 632267bb; `k9_stop_taper.py` sha256 9c43708b… (identical to the working tree for the whole run). **No production code or threshold was changed.** Default OFF and StopReq / hold / soft-hold timing are unchanged.
- All campaigns ended on their own at 05:00:00 (done markers: v1 workers 5/5, v2 workers 2/2, v3 sensitivity Phase A 5472/5472).
- **Zero confirmed module-contract violations** (frame invariants 0, fuzz/replay errors 0, negative-control mismatches 0).
- However, **within simple, non-K9-calibrated models, paths worse than the original command were reproduced**: the original stops but ON does not, or travels >0.5 m further; new lead contact; delayed departure. No claim of "never worse than the original", comfort, safety or real-vehicle effect can be made.
- Prior adverse evidence is retained: +0.453 m on a 12 % downhill, 1.772x pitch swing, 204 of 1,776 cases where neither arm stopped.
- Owner questions Q1–Q3 (below) were not implemented.

## Volume and unique coverage (vs repetition)
| Campaign | Runs | Unique coverage | Repetition / limits |
|---|---|---|---|
| v1 closed (real LongControl + module, random plant) | 697,112 | distinct random case per seed | 191,658 (27.5 %) where OFF itself failed are not comparable (info) |
| v1 fuzz (module only, random input sequences) | 349,827 | distinct sequence per seed | differential vs 632267bb vacuous (cur = base) |
| v1 replay (perturbed recorded K9 stops) | 116,667 | **only 73 base stops** | ~1,600 perturbations per stop, open loop |
| v2 rand2 / grid / focus | 15,317 / 15,316 / 15,316 | grid reached **only 2,800 of 4,200 cells**, all jittered (0 exact) | focus deliberately oversamples v1-flagged neighbourhoods; its rates are not representative |
| exact grid audit | 4,200 | every cell once, no jitter | original planner, dead 0.1 s, a_app 1.0 fixed |
| v3 Phase A (exact factorial) | 5,472 | 152 items x 36 axes, all unique | items selected from earlier REG/warn cells and representative seeds |
| v3 Phase B (random extension) | 43,860 | random perturbations around the same 152 items | neighbourhood repetition, not new territory; not exhaustive |

Negative controls: v2 OFF/OFF and ON/ON bit-identical 4,596/4,596; exact grid 210/210; v3 OFF/OFF, ON/ON and null-taper==OFF 1,907/1,907.

## v1/v2 metric flaws and the corrected v3 distinctions
- v1 (harness flaws, results preserved): first-touch t_stop, asymmetric OFF/ON horizons with a 30 s cap, a planner surrogate that never replans after the stop point, small ratio denominators for swing, non-stops counted as departure delays. The v1 maxima (+7.394 m dx, 38.784x swing) are therefore harness-sensitive, not vehicle magnitudes.
- v2 added equal observation windows, sustained stop D (0.3/1/2/3 s), censoring, separation of OFF failures (off_infeasible), a replan ablation and negative controls. Two remaining flaws were found:
  1. its grid seed formula made each worker visit a single mod-3 residue class, so **only 2,800/4,200 cells were reachable**, all jittered (the setup note's "exact first pass" was wrong);
  2. a departure command that **preempted** ON's stop was misclassified as `REG_on_not_stopped` or, during dwell, skipped the distance check.
- v3 classifies preemption separately (`PREEMPT_on_before_zero/in_dwell`) but **evaluates distance and contact independently of preemption**. In the sampled reclassification 93 of 96 v2 non-stops were preemptions, yet 51 still had dx > 0.5 m and 41 dx > 0.25 m: preemption renames, it does not remove adverse evidence.

## Final counts (regression flags are test definitions, not vehicle thresholds)
- v1 closed: REGRESSION 1,105 = extra distance > 0.5 m 545, ON did not stop 296, departure delayed 184, new lead contact 80; warn 4,187. fuzz and replay: 0 flags.
- v2 (original planner, D = 1 s): rand2 REG_dx 4 + non-stop 2 (of 15,317); grid (jittered) REG_dx 118 + non-stop 28; focus REG_dx 638 + non-stop 1,136 + new lead contact 277 + departure delay 401. v2 non-stops include the preemption misclassification above.
- Exact grid: ok 2,925, off_infeasible 1,125, warn_dx 87, **REG_dx > 0.5 m 57** (max 2.07 m), non-stops 0. REG cells cluster where OFF can only just stop (steep downhill, large nonlinear brake gain, large lag).
- v3 Phase A (REG at D = 1 s or contact): planner orig 399/2,736, replan 135/2,736; by ESC hold assumption v0_only 443/2,736, `ASSUMED_stopreq_moving` 91/2,736. In same-config hold pairs, 353 of the 443 v0_only REGs disappear under the assumption and 90 remain; **one REG appears only under the assumption** (with v0_only OFF could not stop, so it was not comparable; with the assumption OFF stops and ON does not).
- v3 Phase B: REG 2,651/43,860 (neighbourhood repetition); by hold v0_only 2,234/21,915, assumption 417/21,945.
- Interpretation: most remaining adverse paths are sensitive to the plant assumption that the ESC holds only at v = 0, to lag and to nonlinear gain. Real K9 ESC behaviour is uncalibrated, so neither assumption is taken as fact, and changing it does not remove all adverse paths.

## New regression tests (`openpilot/selfdrive/car/tests/test_k9_stop_taper_properties.py`)
They check the current contract without a simulator; they do not check stopping distance, nod, comfort or safety.
- 32 seeds x 6,000-frame streams (stops, standstill flicker, interlocks, non-finite and huge finite values, strong braking, plan/radar lead disagreement, radar-only close leads, weak deceleration) with a per-frame contract: never stronger and never released to zero or positive; interlock/non-finite gives the original and disarms; strong braking / close lead gives the original; release and reapply rate limits; an independently restated floor; every episode needs a REARM_SPEED run.
- Every exit reason (11) is reached and real tapers run; reset() equals a fresh object and streams are deterministic.
- Q1 characterization: one finite 1e9 vEgo frame re-arms after an abort (not with the standstill flag). If the owner changes this rule the test must change with it.
- CarController: OFF (missing/false) is CAN bit-identical to the pre-candidate controller 38df0d7c on K9 and K7 (3 seeds x 1,200 frames); ON leaves K7 identical and on K9 keeps StopReq/ACCMode with aReq never stronger.
- False-positive / vacuity audit (mutation test outside the repo): 11 of 12 mutations of a module copy are caught. The survivor (staying armed after `left_window`) is an unobservable equivalent mutant because phase done blocks entry until a re-arm. The first floor check reused the module's own function and missed a mutation; it now uses an independent formula.

## Awaiting owner decisions (not implemented)
- Q1: a single finite outlier (vEgo 1e9) permits the REARM_SPEED re-arm. Add a consecutive-frame or plausibility rule? No such spike was observed during control in 75,565 carState frames of recorded K9 stop windows.
- Q2: on steep downhills / weak brakes with almost no stopping margin the taper consumes the remaining margin. Block entry with a grade estimate?
- Q3: accept the model path where a later stop behind a decelerating lead leads to stronger final braking and a delayed departure?

## Limits and negative evidence
- Every plant element (lag, nonlinear gain, both ESC hold assumptions, pitch, friction) and the planner surrogate are not K9-calibrated. Recorded-stop replay is open loop. D, 0.25/0.5 m and 0.1 s are test definitions.
- Recorded logs, routes and per-case raw data remain private; this document contains aggregates only.
- No evidence of benefit: there is no pitch or ride measurement, and the earlier observation of four recorded K9 stops only suggests aReq may have little influence; it is not a paired test.
