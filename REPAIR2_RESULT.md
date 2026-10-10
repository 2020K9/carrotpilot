# 회피 3.2 시험 실패 4개 수정 (수정 2) 결과

기준 HEAD `5c09378279be16a846aaed5526e3d863784bfbdd`. 승인: 회장님 2026-10-10 19:43 "감사원 바이패스하고 모두 풀승인", 20:27 "테스트 통과하게 못고침?". 정확한 커밋 번호·변경 파일 수는 `../repair2_run/result.txt`에 따로 적었다(이 문서는 그 커밋에 포함되므로 자기 번호를 적을 수 없다).

## 1. 결론

- 지정 격리 명령(3파일, `--noconftest`) 최종 결과: **236 통과, 0 실패, 종료코드 0**(시도 2).
- 기준 실패 4건(`final_pytest.txt`: 223 통과, 4 실패)이 모두 통과한다. 새 회귀시험 10건(매개변수 포함)도 추가했다.
- 이것은 `--noconftest` 격리 실행이다. **전체 통합시험 통과가 아니다.** 일반 pytest는 이전 기록대로 msgq.ipc_pyx가 없어 준비 단계에서 실패한다. 이번에는 다시 확인하지 않았다.
- 기능은 계속 비활성이다. 기본값, 승인 튜플, 운영 구현 등록부는 바꾸지 않았다(§7).

## 2. 원인과 변경

### 2.1 실패 1·2 — `test_new_avoid_side_detection_revokes_and_never_grows[left/right]`
- 원인: 회피량이 증가하는 중(크기축 변화율 u_prev=0.3 > d=max_rate_change·dt=0.1)에 회피 쪽이 점유되면 증가 금지(u≤0)가 걸린다. 원 장애물이 남아 있으면 복귀 금지(u≥0)도 걸린다. 여기에 양방향 연속성 |u−u_prev|≤d까지 더하면 교집합이 비어 CONFLICT(무효 출력)가 되었다.
- 결정: 이동 차단 우선. 감지 취소 뒤에는 |applied|가 한 주기도 늘지 않는다. 연속성은 줄어드는 쪽으로만 완화한다.
- 변경(`lane_avoid.py` `_step`, `_violations`): 허가가 없을 때 연속성 하한만 `min(u_prev − d, 0)`으로 바꾼다. 상한(u_prev+d)과 다른 제약은 그대로다.
  - u_prev > d이면 하한이 0이 되어 증가가 그 주기에 즉시 멈춘다(u=0). 완화되는 것은 이 정지뿐이다.
  - 0보다 아래(실제 감소)는 기존 연속성 폭 d를 그대로 따른다. 다음 주기부터는 u_prev=0 기준으로 한 주기에 d씩만, 복귀율 상한 이내에서 0을 향해 줄어든다.
  - 복귀 쪽이 막혀 있으면 기존 복귀 금지(u≥0)가 그대로라서 회피량을 유지한다. 장애물 쪽으로 강제 복귀시키지 않는다.
  - 허가가 있는 주기와 감소 중(u_prev≤0)인 경우는 기존과 같다. 다른 충돌(가장자리 여유 축소, 시간 결함 중 이동, 위험 처리기 제안 거부 등)도 그대로다.
- 공용 점검기(`lane_avoid_audit.py` `audit_frames`): 같은 한쪽 규칙을 반영했다. 허가 없는 프레임에서 크기축 하한을 `min(u_prev − d, 0)`으로 둔다. 허가가 있는 프레임의 정지, 0 아래로 넘는 급감소는 계속 `rate_change_above_limit`로 판정한다(새 시험으로 확인).
- 시험 1·2 자체는 **수정하지 않았다**(원문 그대로 통과, 부록 참조).

### 2.2 실패 3 — `test_required_span_covers_body_and_whole_path`
- 원인: 계단형 경로(x>70에서 −0.8 m)에서 회전한 차체 모서리가 띠 모형(0.8+0.95+0.3=2.05)보다 바깥으로 나간다. 그래서 2.550168484768921이 된다.
- 결정에 따라 기대값만 회전 모형 값으로 바꿨다. 구현 함수를 호출하지 않고 다음과 같이 독립 계산했다. 허용오차는 기존 `pytest.approx` 기본값 그대로다.
  - 격자 간격 h=100/32=3.125. 계단은 X[22]=68.75와 X[23]=71.875 사이에 있다.
  - X[23]의 중앙차분 기울기 k=−0.8/(2h)=−0.128.
  - 왼쪽 최외곽 모서리는 앞끝 4.0 m, 왼쪽 반폭 0.95 m에 있다.
  - lat_max = 0.8 + (4.0·0.128 + 0.95)/√(1+0.128²) + 0.3 = 2.550168484768921

### 2.3 실패 4 — `test_planner_consumes_exactly_the_controller_offset`
- 원인: LaneModelSpeedGuard가 약 1초 동안 준비한 뒤에야 레인모드를 켠다(의도된 새 동작). 그래서 첫 플래너 프레임의 `lanelines_active` 단언이 거짓이었다.
- 변경: 준비 단계에서 `lanelines_active`가 될 때까지(최대 200 프레임) 플래너를 진행하고, 도달했는지 단언한다. 그 뒤 기존 40 프레임 반복과 모든 단언은 원문 그대로다.

## 3. 바꾼 기존 시험과 이유

| 시험 | 바꾼 것 | 이유 |
|---|---|---|
| test_lane_avoid.py::test_required_span_covers_body_and_whole_path | `bend` 기대값 1개 | 결정 3: 회전 모서리 모형. 독립 계산식을 주석으로 남김 |
| test_lane_avoid.py::test_planner_consumes_exactly_the_controller_offset | 준비 단계 추가(단언 무변경) | 결정 4: 준비시간 진행 |
| test_lane_avoid.py::test_return_risk_while_moving_is_reported_conflict_not_hidden[left/right] | CONFLICT 기대 3개 → 새 규칙 기대 | 결정 1·2와 같은 시나리오(증가 중 취소, 원 장애물 잔존). 기존 기대는 양방향 연속성 때문에 생긴 충돌을 전제했고, 원문 주석도 시험 1·2와 충돌한다고 적고 있었다. 시도 1에서 실패하여 확인했다 |
| test_lane_avoid_v3_2.py::test_in_progress_cancel_conflict_is_detected_and_invalid[left/right] | CONFLICT·무효 기대 2개 → 새 규칙 기대 | 같은 시나리오의 결함 검출 시험. 결정 1·2가 이 "사람 결정 필요" 항목을 해소했다 |
| test_lane_avoid_v3_2.py::test_cancel_after_hold_with_return_side_clear_is_valid_on_every_frame | 주석 2줄만 | "GROWING 불가능, 사람 결정" 서술이 사실이 아니게 되어 고침. 코드·단언 무변경 |

**주의(검토 요청):** 아래 두 시험은 결정문에 이름이 없다.
- `test_return_risk_while_moving_is_reported_conflict_not_hidden`
- `test_in_progress_cancel_conflict_is_detected_and_invalid`

둘 다 결정 1·2가 정한 바로 그 시나리오에서 옛 결과(CONFLICT)를 단언하므로, 새 규칙에 맞추려면 기대값을 바꿀 수밖에 없다. 기대 위험 표시(`RETURN_RISK`, `return_risk_policy_unapproved`)는 그대로 단언해 위험을 숨기지 않는다. 옛 단언은 각 시험에 주석으로 남겼다.

## 4. 변경한 단언 대응표

| 위치 | 변경 전 | 변경 후 |
|---|---|---|
| test_required_span… | `lat_max == pytest.approx(0.8 + 1.25)` | `lat_max == pytest.approx(0.8 + (4.0*k + 0.95)/math.sqrt(1 + k*k) + 0.3)`, k=0.8/(2·100/32) |
| test_planner_consumes… | (없음) | 준비 반복 뒤 `assert planner.lanelines_active`(추가) |
| test_return_risk_while_moving… | `state == CONFLICT and 'constraint_conflict_policy_unapproved' in reasons` | `state == RETURN_RISK and 'return_risk_policy_unapproved' in reasons` |
| 〃 | `any(c.startswith('conflict_') for c in conflict)` | `conflict == () and all(o.valid for o in outs2)` |
| 〃 | `res['violations'] and all(... state == CONFLICT ...)` | `res['violations'] == []` |
| test_in_progress_cancel… | `state == CONFLICT and not valid` | `state == RETURN_RISK and valid and conflict == ()` 및 applied 유지 단언 추가 |
| 〃 | `'output_invalid' in rules(audit)` | `audit violations == []` |

이 밖의 기존 단언은 삭제·완화하지 않았다. 허용오차 확대, 표본 제외, xfail도 없다.

## 5. 추가 회귀시험 (test_lane_avoid_v3_2.py)

- `test_revocation_while_growing_never_grows_and_safe_return_obeys_rate_limits`: 방향(좌/우) × 복귀 쪽(막힘/안전) × 주기(균일 0.05, 불균일 0.04/0.06/0.05), 8건. 감지 취소 직후 120 주기 동안 매 주기 다음을 단언한다.
  - |applied| 비증가, 유효 출력, 비허가, 측 교차 없음.
  - 첫 취소 주기는 rate=0(즉시 정지).
  - 이후 매 주기 |Δrate| ≤ max_rate_change·dt, |rate| ≤ return_rate.
  - 안전 복귀면 0·STANDBY 도달, 막혔으면 회피량 유지.
  - 점검기 위반 없음.
- `test_audit_exempts_only_the_unpermitted_stop_of_growth`: 점검기가 허가 없는 정지만 면제하는지 확인한다. 허가 있는 정지와 0 아래로의 급감소는 위반으로 남는다.

## 6. 시험 실행 기록 (../repair2_run/)

명령(작업 디렉터리 repo, 출력은 파일로 재지정, 파이프 없음):
```
PYTHONDONTWRITEBYTECODE=1 /home/ssm-user/work/py312_ctl/bin/python -m pytest --noconftest -o addopts= -p no:cacheprovider -q openpilot/selfdrive/controls/tests/test_lane_avoid.py openpilot/selfdrive/controls/tests/test_lane_avoid_v3_2.py openpilot/selfdrive/controls/tests/test_lane_model_speed_planner.py
```

| 시도 | 상태 | 종료코드 | 결과 | 기록 |
|---|---|---|---|---|
| 1 | 코드·시험 1~4 수정 후 | 1 | 234 통과, 2 실패(test_return_risk_while_moving…[left/right]) | attempt1_command.txt, attempt1_pytest.txt, attempt1_rc.txt |
| 2 | 위 시험 기대값 변경 후(최종) | 0 | 236 통과, 0 실패 | attempt2_command.txt, attempt2_pytest.txt, attempt2_rc.txt |

- 종료코드: 셸 안 `$?` 기록이 샌드박스에서 거부되었다. 그래서 Claude Code Bash 도구가 직접 보고한 종료코드(비0이면 "Exit code N", 0이면 표시 없음)를 기록했다.
- 같은 실패가 3번 반복된 경우는 없다.
- 수정 전 반복 시험은 하지 않았다(기준 기록 `tests_run/final_pytest.txt` 사용).
- 기준 원문은 `git show 5c09378…`로 `../repair2_run/orig_*.py`에 보존했다.

## 7. 기능 비활성 및 기본값 무변경 근거

- `LaneAvoidConfig` 필드와 기본값, `APPROVED_*` 튜플(모두 `()`), `POLICY_IMPLEMENTATIONS` 운영 등록부를 바꾸지 않았다. `git diff`의 `lane_avoid.py` 변경은 `_step`, `_violations`의 연속성 하한과 문서 문자열뿐이다.
- 기본 설정은 여전히 `feature_disabled`를 포함한 blocker를 가지고 DISABLED이다. 출력은 정확히 0이다. 이를 확인하는 기존 시험들은 시도 2에서 통과했다.
  - `test_default_config_is_disabled_with_every_blocker`
  - `test_numeric_values_without_approved_return_policy_stay_disabled`
  - `test_production_conflict_policy_cannot_activate`
  - `test_production_registry_has_no_operational_policy`
- TEST_CFG와 시험 구현은 합성 준비 자료이며 승인값이 아니다.

## 8. 남은 제한

- 격리 실행일 뿐이다. 전체 통합시험, 실차, 재생 검증은 아니다.
- 한쪽 연속성 완화로 정지 주기의 회피량 변화율이 u_prev에서 0으로 한 번에 바뀐다. 즉 그 주기의 2차 미분은 상한을 넘을 수 있다. 결정(이동 차단 우선)에 따른 것이다. 최종 곡률·조향 명령에 미치는 영향은 검증하지 않았다(`FINAL_COMMAND_SIGNALS_UNVERIFIED`).
- 감소 중 복귀 쪽이 새로 막히는 대칭 경우(u_prev<0에서 u≥0 요구)는 결정 범위 밖이다. 기존대로 CONFLICT(무효)로 남는다.
- 결정 범위 밖의 다른 CONFLICT 경로와 미승인 정책(위험 처리, 충돌 처리, 점유 예측, 차체 모형 승인)도 그대로 미해결이다.
- 두 시험(§3 주의)은 결정문에 이름 없이 기대값을 바꿨다. 사람 확인이 필요하다.

## 부록: 변경 전후 함수 원문 (함수 전체)

### `openpilot/selfdrive/controls/lib/lane_avoid.py` :: `LaneAvoidController._step`

변경 전 (기준 HEAD 5c09378, git show):

```python
  def _step(self, target, dt, permitted, return_ok, cap):
    """Choose this frame's applied offset inside the intersection of every constraint,
    on the magnitude m = s * applied along the motion side s (u = actual dm/dt):
      continuity   |u - u_prev| <= max_rate_change_mps2 * dt
      rates        -return_rate_mps <= u <= entry_rate_mps
      room         m + u*dt <= cap (min(max_offset, observed edge room); 0 if not observed)
      no crossing  m + u*dt >= 0
      permission   u <= 0 unless permitted; u >= 0 unless return_ok
    The desired rate follows a discrete braking profile, so at uniform dt a target is
    reached with a rate the next frame can bring back to 0. No constraint is skipped and
    no rate is forced to 0. Returns (applied, conflict); conflict names the binding
    lower/upper constraints when the intersection is empty."""
    cfg = self.cfg
    s = self.motion_sign
    if s == 0.0:
      if target == 0.0:
        return 0.0, ()
      s = math.copysign(1.0, target)
    m, u_prev = s * self.applied, s * self.rate
    m_t = max(s * target, 0.0)
    m_cap = cap[RIGHT if s > 0.0 else LEFT]
    d = cfg.max_rate_change_mps2 * dt
    lower = {'continuity': u_prev - d, 'return_rate': -cfg.return_rate_mps, 'no_side_crossing': -m / dt}
    upper = {'continuity': u_prev + d, 'entry_rate': cfg.entry_rate_mps, 'edge_room_or_max_offset': (m_cap - m) / dt}
    if not permitted:
      upper['no_growth_without_permission'] = 0.0
    if not return_ok:
      lower['no_return_without_clear_return_side'] = 0.0
    lo_name = max(lower, key=lower.get)
    hi_name = min(upper, key=upper.get)
    lo, hi = lower[lo_name], upper[hi_name]
    if lo > hi:
      return self.applied, (f'conflict_{lo_name}_vs_{hi_name}',)
    e = m_t - m
    land = abs(e) <= d * dt
    if land:
      u_des = e / dt  # final step: |u_des| <= d, so the next frame can return the rate to 0
    else:
      lim = cfg.entry_rate_mps if e > 0.0 else cfg.return_rate_mps
      # largest rate whose discrete stopping distance (steps of d) still fits in |e|;
      # d*dt/8 covers the gap between the discrete sum and its continuous bound
      brake = d * (math.sqrt(0.25 + 2.0 * (abs(e) - d * dt / 8.0) / (d * dt)) - 0.5)
      u_des = math.copysign(min(lim, brake), e)
    u = min(max(u_des, lo), hi)
    nxt = m_t if (land and u == u_des) else m + u * dt
    # float rounding only: a step that reaches the target exactly in real arithmetic
    # (e.g. forced by the continuity bound) must not land past it by one ulp, which the
    # next frame would read as target < applied (return risk) while the rate is non-zero
    if abs(nxt - m_t) <= ROUNDING_EPS:
      nxt = m_t
    # float rounding only: the bounds above already keep 0 <= nxt <= m_cap
    nxt = min(max(nxt, 0.0), m_cap)
    return s * nxt, ()
```

변경 후:

```python
  def _step(self, target, dt, permitted, return_ok, cap):
    """Choose this frame's applied offset inside the intersection of every constraint,
    on the magnitude m = s * applied along the motion side s (u = actual dm/dt):
      continuity   |u - u_prev| <= max_rate_change_mps2 * dt; without permission the
                   lower bound is min(u_prev - d, 0) (growth stops at once, repair 2)
      rates        -return_rate_mps <= u <= entry_rate_mps
      room         m + u*dt <= cap (min(max_offset, observed edge room); 0 if not observed)
      no crossing  m + u*dt >= 0
      permission   u <= 0 unless permitted; u >= 0 unless return_ok
    The desired rate follows a discrete braking profile, so at uniform dt a target is
    reached with a rate the next frame can bring back to 0. No constraint is skipped and
    no rate is forced to 0. Returns (applied, conflict); conflict names the binding
    lower/upper constraints when the intersection is empty."""
    cfg = self.cfg
    s = self.motion_sign
    if s == 0.0:
      if target == 0.0:
        return 0.0, ()
      s = math.copysign(1.0, target)
    m, u_prev = s * self.applied, s * self.rate
    m_t = max(s * target, 0.0)
    m_cap = cap[RIGHT if s > 0.0 else LEFT]
    d = cfg.max_rate_change_mps2 * dt
    # repair 2 (human decision 2026-10-10): blocking movement has priority. Without
    # permission a growing |applied| stops in this frame (u = 0 allowed whatever u_prev);
    # continuity is relaxed only toward that stop, never below 0, so any decrease toward
    # 0 still changes the rate by at most d per frame.
    lo_cont = min(u_prev - d, 0.0) if not permitted else u_prev - d
    lower = {'continuity': lo_cont, 'return_rate': -cfg.return_rate_mps, 'no_side_crossing': -m / dt}
    upper = {'continuity': u_prev + d, 'entry_rate': cfg.entry_rate_mps, 'edge_room_or_max_offset': (m_cap - m) / dt}
    if not permitted:
      upper['no_growth_without_permission'] = 0.0
    if not return_ok:
      lower['no_return_without_clear_return_side'] = 0.0
    lo_name = max(lower, key=lower.get)
    hi_name = min(upper, key=upper.get)
    lo, hi = lower[lo_name], upper[hi_name]
    if lo > hi:
      return self.applied, (f'conflict_{lo_name}_vs_{hi_name}',)
    e = m_t - m
    land = abs(e) <= d * dt
    if land:
      u_des = e / dt  # final step: |u_des| <= d, so the next frame can return the rate to 0
    else:
      lim = cfg.entry_rate_mps if e > 0.0 else cfg.return_rate_mps
      # largest rate whose discrete stopping distance (steps of d) still fits in |e|;
      # d*dt/8 covers the gap between the discrete sum and its continuous bound
      brake = d * (math.sqrt(0.25 + 2.0 * (abs(e) - d * dt / 8.0) / (d * dt)) - 0.5)
      u_des = math.copysign(min(lim, brake), e)
    u = min(max(u_des, lo), hi)
    nxt = m_t if (land and u == u_des) else m + u * dt
    # float rounding only: a step that reaches the target exactly in real arithmetic
    # (e.g. forced by the continuity bound) must not land past it by one ulp, which the
    # next frame would read as target < applied (return risk) while the rate is non-zero
    if abs(nxt - m_t) <= ROUNDING_EPS:
      nxt = m_t
    # float rounding only: the bounds above already keep 0 <= nxt <= m_cap
    nxt = min(max(nxt, 0.0), m_cap)
    return s * nxt, ()
```

### `openpilot/selfdrive/controls/lib/lane_avoid.py` :: `LaneAvoidController._violations`

변경 전 (기준 HEAD 5c09378, git show):

```python
  def _violations(self, new_applied, dt, permitted, return_ok, cap, tol=1e-9):
    """Constraints of _step violated by publishing `new_applied` this frame (v3.2 final
    re-check; also validates risk-handler proposals). Empty list = consumable."""
    cfg = self.cfg
    s = self.motion_sign
    if s == 0.0:
      if new_applied == 0.0:
        return [] if self.rate == 0.0 or abs(self.rate) <= cfg.max_rate_change_mps2 * dt + tol else ['continuity']
      s = math.copysign(1.0, new_applied)
    m, u_prev, m_new = s * self.applied, s * self.rate, s * new_applied
    u = (m_new - m) / dt
    m_cap = cap[RIGHT if s > 0.0 else LEFT]
    bad = []
    if abs(u - u_prev) > cfg.max_rate_change_mps2 * dt + tol:
      bad.append('continuity')
    if u > cfg.entry_rate_mps + tol:
      bad.append('entry_rate')
    if u < -cfg.return_rate_mps - tol:
      bad.append('return_rate')
    if m_new > m_cap + tol * dt:
      bad.append('edge_room_or_max_offset')
    if m_new < -tol * dt:
      bad.append('no_side_crossing')
    if not permitted and u > tol:
      bad.append('no_growth_without_permission')
    if not return_ok and u < -tol:
      bad.append('no_return_without_clear_return_side')
    return bad
```

변경 후:

```python
  def _violations(self, new_applied, dt, permitted, return_ok, cap, tol=1e-9):
    """Constraints of _step violated by publishing `new_applied` this frame (v3.2 final
    re-check; also validates risk-handler proposals). Empty list = consumable."""
    cfg = self.cfg
    s = self.motion_sign
    if s == 0.0:
      if new_applied == 0.0:
        return [] if self.rate == 0.0 or abs(self.rate) <= cfg.max_rate_change_mps2 * dt + tol else ['continuity']
      s = math.copysign(1.0, new_applied)
    m, u_prev, m_new = s * self.applied, s * self.rate, s * new_applied
    u = (m_new - m) / dt
    m_cap = cap[RIGHT if s > 0.0 else LEFT]
    d = cfg.max_rate_change_mps2 * dt
    # repair 2: without permission, growth may stop at once (lower bound min(u_prev - d, 0))
    lo_cont = min(u_prev - d, 0.0) if not permitted else u_prev - d
    bad = []
    if u < lo_cont - tol or u > u_prev + d + tol:
      bad.append('continuity')
    if u > cfg.entry_rate_mps + tol:
      bad.append('entry_rate')
    if u < -cfg.return_rate_mps - tol:
      bad.append('return_rate')
    if m_new > m_cap + tol * dt:
      bad.append('edge_room_or_max_offset')
    if m_new < -tol * dt:
      bad.append('no_side_crossing')
    if not permitted and u > tol:
      bad.append('no_growth_without_permission')
    if not return_ok and u < -tol:
      bad.append('no_return_without_clear_return_side')
    return bad
```

### `openpilot/selfdrive/controls/lib/lane_avoid_audit.py` :: `audit_frames`

변경 전 (기준 HEAD 5c09378, git show):

```python
def audit_frames(frames, cfg):
  violations = []
  missing = []
  non_diff = []
  counts = {}
  prev = None
  prev_rate = None  # actual rate of the previous interval (or the first frame's 'rate')
  gap = cfg.max_input_gap_s if _num(cfg.max_input_gap_s) else None
  jmax = cfg.max_rate_change_mps2 if _num(cfg.max_rate_change_mps2) else None
  for i, f in enumerate(frames):
    if not all(k in f for k in REQUIRED_FIELDS) or not all(
        isinstance(v, (int, float)) and math.isfinite(v) for v in (f.get('t'), f.get('applied'), f.get('target'))):
      missing.append(i)
      if i > 0:
        non_diff.append({'frame': i, 'why': 'invalid_frame'})
      prev = None
      prev_rate = None
      continue
    key = (f['state'], f['avoid_side'])
    counts[key] = counts.get(key, 0) + 1

    def fail(rule, **raw):
      violations.append({'frame': i, 'rule': rule, 'state': f['state'], 'avoid_side': f['avoid_side'], **raw})

    applied, target = f['applied'], f['target']
    move_side = _side_of(target) or _side_of(applied) or f['avoid_side']
    if f['permitted']:
      if move_side is None:
        fail('permitted_without_direction')
      else:
        if f['side_state'].get(move_side) != CLEAR:
          fail('permitted_avoid_side_not_clear', side=move_side, side_state=f['side_state'].get(move_side))
        if f['edge_state'].get(move_side) != CLEAR:
          fail('permitted_avoid_edge_not_clear', side=move_side, edge_state=f['edge_state'].get(move_side))
    if target != 0.0 and not f['permitted']:
      fail('target_without_permission', target=target)
    if isinstance(cfg.max_offset_m, (int, float)) and abs(applied) > cfg.max_offset_m:
      fail('offset_above_limit', applied=applied, limit=cfg.max_offset_m)
    # v3.1: every frame's applied offset (residual included) must fit the observed edge room
    side_now = _side_of(applied)
    if side_now is not None:
      room = f.get('edge_room')
      if isinstance(room, dict):
        r = room.get(side_now)
        if not _num(r) or abs(applied) > r + 1e-12:
          fail('applied_exceeds_edge_room', side=side_now, applied=applied, room=r)
      elif f['edge_state'].get(side_now) != CLEAR:
        fail('applied_toward_unobserved_edge', side=side_now, applied=applied,
             edge_state=f['edge_state'].get(side_now))
    # v3.2: an invalid output is a failure of the trace, never a normal frame
    if f.get('valid') is False:
      fail('output_invalid', invalid_reasons=f.get('invalid_reasons'))
    if 'consumed' in f and not (_num(f['consumed']) and abs(f['consumed'] - applied) <= RATE_TOL):
      fail('consumed_differs_from_applied', applied=applied, consumed=f['consumed'])

    if prev is None:
      # first sample (or after an invalid frame): only the frame's own rate can seed continuity
      prev_rate = f['rate'] if _num(f.get('rate')) else None
    else:
      dt = f['t'] - prev['t']
      a0 = prev['applied']
      if dt <= 0.0:
        if applied != a0:
          fail('offset_changed_without_time', dt=dt, before=a0, after=applied)
        non_diff.append({'frame': i, 'why': 'time_not_increasing', 'dt': dt})
        prev_rate = None
      else:
        r = (applied - a0) / dt
        if _num(f.get('rate')) and abs(f['rate'] - r) > RATE_TOL:
          fail('reported_rate_mismatch', reported=f['rate'], actual=r)
        if gap is not None and dt > gap:
          non_diff.append({'frame': i, 'why': 'time_gap', 'dt': dt})
          r_next = None
        else:
          r_next = r
          if prev_rate is None:
            non_diff.append({'frame': i, 'why': 'no_previous_rate'})
          elif jmax is None:
            if r != prev_rate:
              fail('rate_change_limit_unapproved', rate_before=prev_rate, rate_after=r)
          elif abs(r - prev_rate) > jmax * dt + RATE_TOL:
            fail('rate_change_above_limit', rate_before=prev_rate, rate_after=r, change=abs(r - prev_rate),
                 limit=jmax * dt, dt=dt)
        prev_rate = r_next
        if a0 * applied < 0.0:
          fail('offset_crossed_sides', before=a0, after=applied)
        grew = abs(applied) > abs(a0)
        shrank = abs(applied) < abs(a0)
        side = _side_of(applied) or _side_of(a0)
        if grew:
          if not f['permitted']:
            fail('growth_without_permission', before=a0, after=applied)
          if f['side_state'].get(side) != CLEAR or f['edge_state'].get(side) != CLEAR:
            fail('growth_toward_unclear_side', side=side, side_state=f['side_state'].get(side),
                 edge_state=f['edge_state'].get(side))
          rate = cfg.entry_rate_mps
        else:
          rate = cfg.return_rate_mps
        if shrank:
          ret = other_side(side)
          if f['side_state'].get(ret) != CLEAR or f['edge_state'].get(ret) != CLEAR:
            fail('return_move_without_clear_return_side', side=ret, side_state=f['side_state'].get(ret),
                 edge_state=f['edge_state'].get(ret), before=a0, after=applied)
        if isinstance(rate, (int, float)) and abs(applied - a0) > rate * dt + 1e-12:
          fail('rate_above_limit', rate=abs(applied - a0) / dt, limit=rate)
        elif not isinstance(rate, (int, float)) and (grew or shrank):
          fail('rate_limit_unapproved', before=a0, after=applied)
    prev = f

  blockers = list(cfg.blockers())
  if violations:
    result = 'fail'
  elif blockers or missing or not frames or non_diff:
    result = 'not_evaluable'
  else:
    # still not a pass: return-side proximity, risk-policy and coverage evidence are
    # evaluated by the approved replay tool, not by this trace check
    result = 'no_violation_found'
  return {
    'result': result,
    'violations': violations,
    'frames': len(frames),
    'invalid_frames': missing,
    'non_differentiable': non_diff,
    'counts_by_state_side': counts,
    'blockers': blockers,
    'final_command_signals_unverified': FINAL_COMMAND_SIGNALS_UNVERIFIED,
  }
```

변경 후:

```python
def audit_frames(frames, cfg):
  violations = []
  missing = []
  non_diff = []
  counts = {}
  prev = None
  prev_rate = None  # actual rate of the previous interval (or the first frame's 'rate')
  gap = cfg.max_input_gap_s if _num(cfg.max_input_gap_s) else None
  jmax = cfg.max_rate_change_mps2 if _num(cfg.max_rate_change_mps2) else None
  for i, f in enumerate(frames):
    if not all(k in f for k in REQUIRED_FIELDS) or not all(
        isinstance(v, (int, float)) and math.isfinite(v) for v in (f.get('t'), f.get('applied'), f.get('target'))):
      missing.append(i)
      if i > 0:
        non_diff.append({'frame': i, 'why': 'invalid_frame'})
      prev = None
      prev_rate = None
      continue
    key = (f['state'], f['avoid_side'])
    counts[key] = counts.get(key, 0) + 1

    def fail(rule, **raw):
      violations.append({'frame': i, 'rule': rule, 'state': f['state'], 'avoid_side': f['avoid_side'], **raw})

    applied, target = f['applied'], f['target']
    move_side = _side_of(target) or _side_of(applied) or f['avoid_side']
    if f['permitted']:
      if move_side is None:
        fail('permitted_without_direction')
      else:
        if f['side_state'].get(move_side) != CLEAR:
          fail('permitted_avoid_side_not_clear', side=move_side, side_state=f['side_state'].get(move_side))
        if f['edge_state'].get(move_side) != CLEAR:
          fail('permitted_avoid_edge_not_clear', side=move_side, edge_state=f['edge_state'].get(move_side))
    if target != 0.0 and not f['permitted']:
      fail('target_without_permission', target=target)
    if isinstance(cfg.max_offset_m, (int, float)) and abs(applied) > cfg.max_offset_m:
      fail('offset_above_limit', applied=applied, limit=cfg.max_offset_m)
    # v3.1: every frame's applied offset (residual included) must fit the observed edge room
    side_now = _side_of(applied)
    if side_now is not None:
      room = f.get('edge_room')
      if isinstance(room, dict):
        r = room.get(side_now)
        if not _num(r) or abs(applied) > r + 1e-12:
          fail('applied_exceeds_edge_room', side=side_now, applied=applied, room=r)
      elif f['edge_state'].get(side_now) != CLEAR:
        fail('applied_toward_unobserved_edge', side=side_now, applied=applied,
             edge_state=f['edge_state'].get(side_now))
    # v3.2: an invalid output is a failure of the trace, never a normal frame
    if f.get('valid') is False:
      fail('output_invalid', invalid_reasons=f.get('invalid_reasons'))
    if 'consumed' in f and not (_num(f['consumed']) and abs(f['consumed'] - applied) <= RATE_TOL):
      fail('consumed_differs_from_applied', applied=applied, consumed=f['consumed'])

    if prev is None:
      # first sample (or after an invalid frame): only the frame's own rate can seed continuity
      prev_rate = f['rate'] if _num(f.get('rate')) else None
    else:
      dt = f['t'] - prev['t']
      a0 = prev['applied']
      if dt <= 0.0:
        if applied != a0:
          fail('offset_changed_without_time', dt=dt, before=a0, after=applied)
        non_diff.append({'frame': i, 'why': 'time_not_increasing', 'dt': dt})
        prev_rate = None
      else:
        r = (applied - a0) / dt
        if _num(f.get('rate')) and abs(f['rate'] - r) > RATE_TOL:
          fail('reported_rate_mismatch', reported=f['rate'], actual=r)
        if gap is not None and dt > gap:
          non_diff.append({'frame': i, 'why': 'time_gap', 'dt': dt})
          r_next = None
        else:
          r_next = r
          if prev_rate is None:
            non_diff.append({'frame': i, 'why': 'no_previous_rate'})
          elif jmax is None:
            if r != prev_rate:
              fail('rate_change_limit_unapproved', rate_before=prev_rate, rate_after=r)
          else:
            # repair 2 (human decision 2026-10-10): without permission a growing |offset| may
            # stop at once; on the magnitude axis the lower bound is min(u_prev - d, 0), so
            # only the drop to 0 is exempt and any decrease still obeys the limit
            s = -1.0 if (a0 < 0.0 or (a0 == 0.0 and applied < 0.0)) else 1.0
            lo = s * prev_rate - jmax * dt
            if not f['permitted']:
              lo = min(lo, 0.0)
            if s * r < lo - RATE_TOL or s * r > s * prev_rate + jmax * dt + RATE_TOL:
              fail('rate_change_above_limit', rate_before=prev_rate, rate_after=r, change=abs(r - prev_rate),
                   limit=jmax * dt, dt=dt)
        prev_rate = r_next
        if a0 * applied < 0.0:
          fail('offset_crossed_sides', before=a0, after=applied)
        grew = abs(applied) > abs(a0)
        shrank = abs(applied) < abs(a0)
        side = _side_of(applied) or _side_of(a0)
        if grew:
          if not f['permitted']:
            fail('growth_without_permission', before=a0, after=applied)
          if f['side_state'].get(side) != CLEAR or f['edge_state'].get(side) != CLEAR:
            fail('growth_toward_unclear_side', side=side, side_state=f['side_state'].get(side),
                 edge_state=f['edge_state'].get(side))
          rate = cfg.entry_rate_mps
        else:
          rate = cfg.return_rate_mps
        if shrank:
          ret = other_side(side)
          if f['side_state'].get(ret) != CLEAR or f['edge_state'].get(ret) != CLEAR:
            fail('return_move_without_clear_return_side', side=ret, side_state=f['side_state'].get(ret),
                 edge_state=f['edge_state'].get(ret), before=a0, after=applied)
        if isinstance(rate, (int, float)) and abs(applied - a0) > rate * dt + 1e-12:
          fail('rate_above_limit', rate=abs(applied - a0) / dt, limit=rate)
        elif not isinstance(rate, (int, float)) and (grew or shrank):
          fail('rate_limit_unapproved', before=a0, after=applied)
    prev = f

  blockers = list(cfg.blockers())
  if violations:
    result = 'fail'
  elif blockers or missing or not frames or non_diff:
    result = 'not_evaluable'
  else:
    # still not a pass: return-side proximity, risk-policy and coverage evidence are
    # evaluated by the approved replay tool, not by this trace check
    result = 'no_violation_found'
  return {
    'result': result,
    'violations': violations,
    'frames': len(frames),
    'invalid_frames': missing,
    'non_differentiable': non_diff,
    'counts_by_state_side': counts,
    'blockers': blockers,
    'final_command_signals_unverified': FINAL_COMMAND_SIGNALS_UNVERIFIED,
  }
```

### `openpilot/selfdrive/controls/tests/test_lane_avoid.py` :: `test_new_avoid_side_detection_revokes_and_never_grows`

변경 없음 (기준 HEAD와 동일). 원문:

```python
@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_new_avoid_side_detection_revokes_and_never_grows(approved_policies, direction):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, frames = avoid_then(ctrl, direction, n=12)  # still growing, rate > 0
  assert ctrl.rate != 0.0
  blocked = side(bsd=occ())
  seq_l, seq_r = (blocked, right) if direction == LEFT else (left, blocked)
  before = outs[-1].applied
  outs2, frames2 = run(ctrl, [dict(shift=shift, left=seq_l, right=seq_r)] * 20, t0=12 * DT)
  assert not outs2[0].permitted and outs2[0].target == 0.0
  # the original obstacle is still there -> no return movement, never further out
  assert all(abs(o.applied) <= abs(before) for o in outs2)
  assert all(o.applied == before for o in outs2)  # no movement toward the remaining obstacle either
  assert all(o.state in (RETURN, RETURN_RISK) and not o.permitted for o in outs2)
  assert outs2[-1].state == RETURN_RISK
  assert audit_frames(frames + frames2, TEST_CFG)['violations'] == []
```

### `openpilot/selfdrive/controls/tests/test_lane_avoid.py` :: `test_required_span_covers_body_and_whole_path`

변경 전 (기준 HEAD 5c09378, git show):

```python
def test_required_span_covers_body_and_whole_path():
  assert la.required_x_range(X, TEST_CFG) == (-1.0, 104.0)  # rear at ego .. front beyond the last point
  assert la.required_x_range(X, LaneAvoidConfig()) is None
  assert la.required_x_range(X[::-1], TEST_CFG) is None
  sp = la.required_space(X, np.zeros(N), [0.0, -0.3], LEFT, TEST_CFG)
  assert (sp.x_min, sp.x_max, sp.lat_min) == (-1.0, 104.0, 0.0)
  assert sp.lat_max == pytest.approx(0.3 + 0.95 + 0.3)  # offset + half width + clearance
  # the base path's own lateral excursion counts too (not only the offset)
  bend = np.where(X > 70.0, -0.8, 0.0)
  assert la.required_space(X, bend, [0.0], LEFT, TEST_CFG).lat_max == pytest.approx(0.8 + 1.25)
  assert la.required_space(X, np.zeros(N), [float('nan')], LEFT, TEST_CFG) is None
```

변경 후:

```python
def test_required_span_covers_body_and_whole_path():
  assert la.required_x_range(X, TEST_CFG) == (-1.0, 104.0)  # rear at ego .. front beyond the last point
  assert la.required_x_range(X, LaneAvoidConfig()) is None
  assert la.required_x_range(X[::-1], TEST_CFG) is None
  sp = la.required_space(X, np.zeros(N), [0.0, -0.3], LEFT, TEST_CFG)
  assert (sp.x_min, sp.x_max, sp.lat_min) == (-1.0, 104.0, 0.0)
  assert sp.lat_max == pytest.approx(0.3 + 0.95 + 0.3)  # offset + half width + clearance
  # the base path's own lateral excursion counts too (not only the offset)
  bend = np.where(X > 70.0, -0.8, 0.0)
  # repair 2 (human decision 2026-10-10): the wider, more conservative rotated-corner model
  # replaces the band expectation 0.8 + 1.25 = 2.05. Independent derivation: X spacing
  # h = 100/32 = 3.125; the step lies between X[22] = 68.75 and X[23] = 71.875, so the
  # central-difference slope at X[23] is k = -0.8 / (2h) = -0.128 and its heading has
  # sin = -0.128/sqrt(1+k^2), cos = 1/sqrt(1+k^2). The outermost LEFT (-y) corner of that
  # pose is the body front (4.0 m) at the left half width (0.95 m):
  #   lat_max = 0.8 + (4.0 * 0.128 + 0.95) / sqrt(1 + 0.128^2) + 0.3 (side clearance)
  #           = 2.550168484768921 (band alone: 0.8 + 0.95 + 0.3 = 2.05 is smaller)
  k = 0.8 / (2.0 * (100.0 / 32.0))
  assert la.required_space(X, bend, [0.0], LEFT, TEST_CFG).lat_max == pytest.approx(
    0.8 + (4.0 * k + 0.95) / math.sqrt(1.0 + k * k) + 0.3)
  assert la.required_space(X, np.zeros(N), [float('nan')], LEFT, TEST_CFG) is None
```

### `openpilot/selfdrive/controls/tests/test_lane_avoid.py` :: `test_planner_consumes_exactly_the_controller_offset`

변경 전 (기준 HEAD 5c09378, git show):

```python
def test_planner_consumes_exactly_the_controller_offset(approved_policies):
  # final consumption: the published path is the base path plus out.applied, nothing else
  planner = make_planner()
  planner.lane_avoid = LaneAvoidController(TEST_CFG)
  shift, left, right = mirror(LEFT)
  n = [0]

  def fake_inputs(sm, carrot, md, model_active):
    n[0] += 1
    return inp(1.0 + n[0] * DT, shift, left, right)
  planner.lane_avoid_inputs = fake_inputs
  seen = []
  for _ in range(40):
    sm = planner_inputs()
    planner.update(sm, SimpleNamespace(atc_active=False))
    o = planner.lane_avoid_out
    assert planner.lanelines_active and o is not None
    assert np.array_equal(planner.path_xyz[:, 1], np.asarray(sm['modelV2'].position.y) + 0.01 + o.applied)
    seen.append(o.applied)
  assert min(seen) < 0.0
```

변경 후:

```python
def test_planner_consumes_exactly_the_controller_offset(approved_policies):
  # final consumption: the published path is the base path plus out.applied, nothing else
  planner = make_planner()
  planner.lane_avoid = LaneAvoidController(TEST_CFG)
  shift, left, right = mirror(LEFT)
  n = [0]

  def fake_inputs(sm, carrot, md, model_active):
    n[0] += 1
    return inp(1.0 + n[0] * DT, shift, left, right)
  planner.lane_avoid_inputs = fake_inputs
  # repair 2 (human decision 2026-10-10): LaneModelSpeedGuard's readiness time is intended;
  # advance planner frames until lane mode is active, then check the unchanged asserts
  for _ in range(200):
    planner.update(planner_inputs(), SimpleNamespace(atc_active=False))
    if planner.lanelines_active:
      break
  assert planner.lanelines_active
  seen = []
  for _ in range(40):
    sm = planner_inputs()
    planner.update(sm, SimpleNamespace(atc_active=False))
    o = planner.lane_avoid_out
    assert planner.lanelines_active and o is not None
    assert np.array_equal(planner.path_xyz[:, 1], np.asarray(sm['modelV2'].position.y) + 0.01 + o.applied)
    seen.append(o.applied)
  assert min(seen) < 0.0
```

### `openpilot/selfdrive/controls/tests/test_lane_avoid.py` :: `test_return_risk_while_moving_is_reported_conflict_not_hidden`

변경 전 (기준 HEAD 5c09378, git show):

```python
@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_return_risk_while_moving_is_reported_conflict_not_hidden(approved_policies, direction):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, frames = avoid_then(ctrl, direction, n=12)  # still growing
  rate0 = ctrl.rate
  assert abs(rate0) > TEST_CFG.max_rate_change_mps2 * DT
  blocked = side(bsd=occ())
  l2, r2 = (blocked, right) if direction == LEFT else (left, blocked)
  outs2, frames2 = run(ctrl, [dict(shift=shift, left=l2, right=r2)] * 10, t0=12 * DT)
  # neither growth (avoid side occupied) nor return (obstacle remains) is allowed, and the
  # rate cannot reach 0 within the continuity limit in one frame -> empty intersection.
  # NOTE: conflicts with the kept v3 assert `state in (RETURN, RETURN_RISK)` in
  # test_new_avoid_side_detection_revokes_and_never_grows (REVIEW.md, fixture conflicts).
  assert outs2[0].state == la.CONFLICT and 'constraint_conflict_policy_unapproved' in outs2[0].reasons
  assert any(c.startswith('conflict_') for c in outs2[0].conflict)
  res = audit_frames(frames + frames2, TEST_CFG)
  all_outs = outs + outs2
  assert res['violations'] and all(all_outs[v['frame']].state == la.CONFLICT for v in res['violations'])
```

변경 후:

```python
@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_return_risk_while_moving_is_reported_conflict_not_hidden(approved_policies, direction):
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, frames = avoid_then(ctrl, direction, n=12)  # still growing
  rate0 = ctrl.rate
  assert abs(rate0) > TEST_CFG.max_rate_change_mps2 * DT
  blocked = side(bsd=occ())
  l2, r2 = (blocked, right) if direction == LEFT else (left, blocked)
  outs2, frames2 = run(ctrl, [dict(shift=shift, left=l2, right=r2)] * 10, t0=12 * DT)
  # neither growth (avoid side occupied) nor return (obstacle remains) is allowed.
  # repair 2 (human decision 2026-10-10): blocking movement has priority and continuity is
  # relaxed only toward stopping the growth, so the rate drops to 0 at once and this is no
  # longer an empty intersection; the risk is still reported (not hidden). Previously:
  #   assert outs2[0].state == la.CONFLICT and 'constraint_conflict_policy_unapproved' in outs2[0].reasons
  #   assert any(c.startswith('conflict_') for c in outs2[0].conflict)
  #   assert res['violations'] and all(all_outs[v['frame']].state == la.CONFLICT for v in res['violations'])
  assert outs2[0].state == la.RETURN_RISK and 'return_risk_policy_unapproved' in outs2[0].reasons
  assert outs2[0].conflict == () and all(o.valid for o in outs2)
  res = audit_frames(frames + frames2, TEST_CFG)
  assert res['violations'] == []
```

### `openpilot/selfdrive/controls/tests/test_lane_avoid_v3_2.py` :: `test_in_progress_cancel_conflict_is_detected_and_invalid`

변경 전 (기준 HEAD 5c09378, git show):

```python
@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_in_progress_cancel_conflict_is_detected_and_invalid(approved_policies, direction):
  # defect-detection test: growing (rate 0.3) when the avoid side becomes occupied and the
  # obstacle remains -> no offset satisfies no-growth, no-return and continuity. The
  # output is invalid; this does NOT resolve the kept v3 assert in
  # test_new_avoid_side_detection_revokes_and_never_grows (human decision, REVIEW.md).
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, _, frames = avoid_then(ctrl, direction, n=12)
  blocked = side(bsd=occ())
  l2, r2 = (blocked, right) if direction == LEFT else (left, blocked)
  outs2, frames2 = run(ctrl, [dict(shift=shift, left=l2, right=r2)] * 3, t0=12 * DT)
  assert outs2[0].state == la.CONFLICT and not outs2[0].valid
  assert 'output_invalid' in rules(audit_frames(frames + frames2, TEST_CFG))
```

변경 후:

```python
@pytest.mark.parametrize('direction', [LEFT, RIGHT])
def test_in_progress_cancel_conflict_is_detected_and_invalid(approved_policies, direction):
  # defect-detection test: growing (rate 0.3) when the avoid side becomes occupied and the
  # obstacle remains -> no offset satisfies no-growth, no-return and continuity. The
  # output is invalid; this does NOT resolve the kept v3 assert in
  # test_new_avoid_side_detection_revokes_and_never_grows (human decision, REVIEW.md).
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, _, frames = avoid_then(ctrl, direction, n=12)
  blocked = side(bsd=occ())
  l2, r2 = (blocked, right) if direction == LEFT else (left, blocked)
  outs2, frames2 = run(ctrl, [dict(shift=shift, left=l2, right=r2)] * 3, t0=12 * DT)
  # repair 2 (human decision 2026-10-10): blocking movement has priority; continuity is
  # relaxed only toward stopping the growth, so this is no longer a conflict. The offset
  # is held (both sides blocked) and every frame is valid. Previously:
  #   assert outs2[0].state == la.CONFLICT and not outs2[0].valid
  #   assert 'output_invalid' in rules(audit_frames(frames + frames2, TEST_CFG))
  assert outs2[0].state == la.RETURN_RISK and outs2[0].valid and outs2[0].conflict == ()
  assert all(o.applied == frames[-1]['applied'] for o in outs2)
  assert audit_frames(frames + frames2, TEST_CFG)['violations'] == []
```

### `openpilot/selfdrive/controls/tests/test_lane_avoid_v3_2.py` :: `test_cancel_after_hold_with_return_side_clear_is_valid_on_every_frame`

변경 전 (기준 HEAD 5c09378, git show):

```python
def test_cancel_after_hold_with_return_side_clear_is_valid_on_every_frame(approved_policies):
  # acceptance (held, rate 0): avoid side becomes occupied, return side clear -> a feasible
  # return exists on every frame, uniform and non-uniform intervals. The GROWING case is
  # infeasible by construction (no growth + continuity, see the test above): human decision.
  for steps in ([DT], [0.04, 0.06, 0.05]):
    ctrl = LaneAvoidController(TEST_CFG)
    shift, left, right = mirror(LEFT)
    t, outs, frames = 0.0, [], []
    for i in range(100):
      if i < 40:
        o = ctrl.update(inp(t, shift, left, right))
      else:
        o = ctrl.update(inp(t, 0.0, left=side(bsd=occ()), right=side()))
      outs.append(o)
      frames.append({'t': t, 'state': o.state, 'permitted': o.permitted, 'target': o.target, 'applied': o.applied,
                     'avoid_side': o.avoid_side, 'side_state': dict(o.side_state), 'edge_state': dict(o.edge_state),
                     'rate': o.rate, 'edge_room': dict(o.edge_room), 'valid': o.valid})
      t += steps[i % len(steps)]
    assert outs[39].rate == 0.0 and outs[39].applied != 0.0
    assert all(o.valid for o in outs)
    assert outs[-1].applied == 0.0
    assert audit_frames(frames, TEST_CFG)['violations'] == []
```

변경 후:

```python
def test_cancel_after_hold_with_return_side_clear_is_valid_on_every_frame(approved_policies):
  # acceptance (held, rate 0): avoid side becomes occupied, return side clear -> a feasible
  # return exists on every frame, uniform and non-uniform intervals. The GROWING case is
  # covered by the repair-2 one-sided continuity rule (see the tests above/below).
  for steps in ([DT], [0.04, 0.06, 0.05]):
    ctrl = LaneAvoidController(TEST_CFG)
    shift, left, right = mirror(LEFT)
    t, outs, frames = 0.0, [], []
    for i in range(100):
      if i < 40:
        o = ctrl.update(inp(t, shift, left, right))
      else:
        o = ctrl.update(inp(t, 0.0, left=side(bsd=occ()), right=side()))
      outs.append(o)
      frames.append({'t': t, 'state': o.state, 'permitted': o.permitted, 'target': o.target, 'applied': o.applied,
                     'avoid_side': o.avoid_side, 'side_state': dict(o.side_state), 'edge_state': dict(o.edge_state),
                     'rate': o.rate, 'edge_room': dict(o.edge_room), 'valid': o.valid})
      t += steps[i % len(steps)]
    assert outs[39].rate == 0.0 and outs[39].applied != 0.0
    assert all(o.valid for o in outs)
    assert outs[-1].applied == 0.0
    assert audit_frames(frames, TEST_CFG)['violations'] == []
```

### `openpilot/selfdrive/controls/tests/test_lane_avoid_v3_2.py` :: `test_revocation_while_growing_never_grows_and_safe_return_obeys_rate_limits`

변경 전 (기준 HEAD 5c09378, git show):

```python
# (없음: 이번에 새로 추가)
```

변경 후:

```python
@pytest.mark.parametrize('direction', [LEFT, RIGHT])
@pytest.mark.parametrize('return_clear', [False, True])
@pytest.mark.parametrize('steps', [[DT], [0.04, 0.06, 0.05]])
def test_revocation_while_growing_never_grows_and_safe_return_obeys_rate_limits(approved_policies, direction,
                                                                                return_clear, steps):
  # repair 2 regression: right after the avoid side becomes occupied while |applied| grows,
  # |applied| is non-increasing on every frame. Return side blocked -> held (never forced
  # toward the obstacle). Return side clear -> the growth stops at once, then the decrease
  # obeys the rate-change limit from rate 0 and the return rate on every later frame.
  ctrl = LaneAvoidController(TEST_CFG)
  shift, left, right, outs, frames = avoid_then(ctrl, direction, n=12)
  assert ctrl.rate * la.SIDE_SIGN[direction] > TEST_CFG.max_rate_change_mps2 * DT  # growing faster than d
  blocked = side(bsd=occ())
  ret_side = right if direction == LEFT else left
  if return_clear:
    ret_side = side()
  l2, r2 = (blocked, ret_side) if direction == LEFT else (ret_side, blocked)
  t, prev = 12 * DT, outs[-1]
  outs2 = []
  for i in range(120):
    o = ctrl.update(inp(t, 0.0 if return_clear else shift, l2, r2))
    dt = t - frames[-1]['t']
    frames.append({'t': t, 'state': o.state, 'permitted': o.permitted, 'target': o.target, 'applied': o.applied,
                   'avoid_side': o.avoid_side, 'side_state': dict(o.side_state), 'edge_state': dict(o.edge_state),
                   'rate': o.rate, 'edge_room': dict(o.edge_room), 'valid': o.valid})
    assert o.valid and not o.permitted
    assert abs(o.applied) <= abs(prev.applied)  # never grows, not even for one frame
    assert o.applied * prev.applied >= 0.0
    if i == 0:
      assert o.rate == 0.0  # the growth stops in the first revoked frame
    else:
      assert abs(o.rate - prev.rate) <= TEST_CFG.max_rate_change_mps2 * dt + 1e-9
      assert abs(o.rate) <= TEST_CFG.return_rate_mps + 1e-9
    outs2.append(o)
    prev = o
    t += steps[i % len(steps)]
  if return_clear:
    assert outs2[-1].applied == 0.0 and outs2[-1].state == la.STANDBY
  else:
    assert all(o.applied == outs[-1].applied for o in outs2)
  assert audit_frames(frames, TEST_CFG)['violations'] == []
```

### `openpilot/selfdrive/controls/tests/test_lane_avoid_v3_2.py` :: `test_audit_exempts_only_the_unpermitted_stop_of_growth`

변경 전 (기준 HEAD 5c09378, git show):

```python
# (없음: 이번에 새로 추가)
```

변경 후:

```python
def test_audit_exempts_only_the_unpermitted_stop_of_growth():
  # repair 2: the shared checker accepts a growth stop (rate -> 0) on an unpermitted frame,
  # but not on a permitted frame and not a jump past 0 into a return
  def f(t, applied, a0, permitted):
    return {'t': t, 'state': 'x', 'permitted': permitted, 'target': -0.3 if permitted else 0.0, 'applied': applied,
            'avoid_side': LEFT, 'side_state': {LEFT: CLEAR, RIGHT: CLEAR}, 'edge_state': {LEFT: CLEAR, RIGHT: CLEAR},
            'rate': (applied - a0) / DT, 'edge_room': {LEFT: 1.0, RIGHT: 1.0}}
  head = [f(0.0, -0.10, -0.085, True), f(DT, -0.115, -0.10, True)]
  stop = audit_frames(head + [f(2 * DT, -0.115, -0.115, False)], TEST_CFG)
  assert 'rate_change_above_limit' not in rules(stop)
  stop_permitted = audit_frames(head + [f(2 * DT, -0.115, -0.115, True)], TEST_CFG)
  assert 'rate_change_above_limit' in rules(stop_permitted)
  past_zero = audit_frames(head + [f(2 * DT, -0.11, -0.115, False)], TEST_CFG)
  assert 'rate_change_above_limit' in rules(past_zero)
```
