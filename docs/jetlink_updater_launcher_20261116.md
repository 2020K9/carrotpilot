# 젯슨 갱신기 시작 프로그램 (새 판 먼저, 실패 시 구판) — 2026-11-16

대감님 06:44 "새 갱신기 먼저 실행하게 만들고 새 갱신기 실패 시 구갱신기가 작동하게", 노비 06:45 검토 요구.
기준 커밋 31372d20c (전환 도구 v3). 새 파일만 추가: `tools/jetlink/updater_launcher.py`,
`tools/jetlink/install_launcher.py`, `tools/jetlink/test_updater_launcher.py`, 이 문서.
update_host.py · switch_updater*.py · 기존 시험은 바꾸지 않았다.

## 폴더

- `/opt/carrot-jetlink/updater/` 새 갱신기 칸. switch_updater_linked.py(31372d20c)가 관리한다. 구판 bootstrap()도 이 칸만 덮어쓴다.
- `/opt/carrot-jetlink/updater-legacy/` 구판(0cb67d46c) 갱신기 바이트 그대로 + `SHA256SUMS`. install_launcher.py가 만들고, 이후 아무도 쓰지 않는다.
- `/opt/carrot-jetlink/launcher/updater_launcher.py` 시작 프로그램.
- `/etc/systemd/system/carrot-jetlink-update-{apply,stage}.service.d/90-updater-launcher.conf` 진입점을 시작 프로그램으로 바꾸는 drop-in (`ExecStart=` 비우고 다시 지정, `SuccessExitStatus=10 30`).

## 순서

1. `/run/carrot-jetlink-update.lock`(구판·신판 update_host.py가 잡는 같은 파일)을 LOCK_NB로 잡는다. 못 잡으면 아무것도 안 돌리고 30.
2. 잠금을 쥔 채 새 갱신기를 따로 프로세스 그룹으로 돌린다(시간 제한: activate 600초, automatic 900초; 유닛 한도 1000/1800초 안). 자식은 같은 열린 잠금 파일을 물려받고, 자식 안에서 그 파일에 대한 flock만 "이미 쥠"으로 넘긴다. 그래서 새 판과 구판 사이에 다른 갱신기가 끼어들 틈이 없다.
3. 결과 분류 후 결정 (아래 표).

## 종료 코드 / 상태 파일 계약

자식 결과: `/run/carrot-jetlink-launcher-{new,legacy}.json` `{"kind": ...}`

- `ok` / `applied` → 끝. 종료 0.
- `verify`(ValueError: 서명·sha·길이·manifest·식별, CalledProcessError) / `rejected`(이번 실행에서 status.json이 rejected) → 업데이트 차단, 구판 안 부름. 종료 20.
- `busy`(BlockingIOError) → 구판 안 부름. 종료 30.
- `connect`(URLError·HTTPException·socket.timeout·ConnectionError·SSLError) / `unrunnable`(파일 없음·ImportError·SyntaxError·SystemExit≠0) / `error`(그 밖의 예외) / `crash`(결과 파일 없음) / `timeout` → 아래 원복 확인 후 구판 실행. 구판 종료 0이면 10, 실패면 40.
- 구판 실행 전 확인: `updates/transaction.json`이 있으면 update_host.activate()의 복구와 같은 순서로 current를 되돌리고(pending → interrupted.json, status recovered) current가 원래 판을 가리키는지 확인. 실패하면 41(구판 안 부름). 거래 기록 없이 current만 바뀌었으면 status가 applied일 때만 0, 아니면 41.
- 시간 초과: 그룹에 SIGTERM 5초 → SIGKILL, 그룹·세션에 산 프로세스가 0이 될 때까지 확인. 확인 못 하면 43(구판 안 부름).
- updater-legacy가 없거나 SHA256SUMS와 다르면 42(구판 안 부름).
- 새 칸이 구판 바이트와 똑같으면(구판 bootstrap이 덮어쓴 경우) 구판으로 한 번만 돌림(10/40).

지속 상태: `/opt/carrot-jetlink/launcher/state.json` (phase: start, new_running, new_result, recovering, legacy_running, done).
다음 실행이 done이 아닌 기록을 보면 `previous_interrupted`로 남기고 평소대로 돈다. 이 부팅 마지막 결과: `/run/carrot-jetlink-launcher.json`.

**10은 "구판이 돌았다"이지 "업데이트됐다"가 아니다.** 구판은 NAS `/models/` 주소와 옛 공개키만 안다. 새 키로 서명된 GitHub 배포판은 구판이 받지도 적용하지도 못한다(구판 activate가 새 판 pending을 만나면 서명 실패로 rejected 처리).

## 구판 시작 코드와 함께 쓰기 (0cb67d46c 소스 확인)

- `migrate_running_release()` → `boot-update-required`가 파일이면 바로 return, 아니면 `install_updates.py --bootstrap-only` → `bootstrap()`.
- `bootstrap()`은 `/opt/carrot-jetlink/updater/` 안 8개 파일과 `boot-update-required`만 쓴다. launcher/·updater-legacy/·drop-in은 안 건드린다.
- `configure()`(전체 설치)는 유닛 파일 3개를 다시 쓰지만 `*.service.d/`는 안 건드린다. 기존 drop-in(power.conf, 보호 이미지 storage.conf)은 ExecStart를 안 정하므로 충돌 없음.
- 그래서 31372d20c 링크 방식과 겹치는 수정은 없다: 전환 도구가 새 칸 + 표시 링크를 맡고, 시작 프로그램은 표시를 건드리지 않는다. 표시가 살아 있으면 구판 시작 코드가 새 칸을 안 덮고, 표시가 사라져 구판이 새 칸을 덮어도 시작 프로그램은 구판으로 한 번 돌 뿐이다(새 칸 재설치는 전환 도구 다시 실행 몫).

## 설치 순서 (실기기 미실행)

1. 전환 전, 현재 `/opt/carrot-jetlink/updater`(구판)로 `install_launcher.py --legacy-source /opt/carrot-jetlink/updater` (옛 키 아니면 거부, 이미 다른 바이트가 있으면 거부).
2. switch_updater_linked.py로 새 칸 전환.
3. `systemctl daemon-reload`.

## 남은 위험 / 미확인

- 새 판 activate가 boot gate 상태면 `systemd-run carrot-jetlink-boot-gate ... boot`로 따로 떠나므로 그 'boot' 실패는 시작 프로그램이 못 본다(구판으로 안 넘어감).
- 디스크 문제 같은 일반 OSError는 `error`로 보고 구판으로 넘긴다(구판도 같은 디스크라 대개 같이 실패).
- 새 판이 반쯤 받은 `updates/` 파일을 구판이 만나는 경우: 구판이 sha로 거르는 것까지만 소스로 확인, 시험 안 함.
- 보호 이미지(/etc 불변)에서 drop-in을 쓸 수 있는지 미확인.
- 실제 전원 차단, 젯슨 실기기 동작, 실제 systemd 동작은 미확인. 시험의 중단은 os._exit(프로세스 강제 종료)다.
