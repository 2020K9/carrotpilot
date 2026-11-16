# 구판 젯슨 갱신기 일회성 전환 (NAS → GitHub), 2026-11-16

대상: `/opt/carrot-jetlink/updater/`가 0cb67d46c 계열(NAS `/models/` 주소만, 옛 공개키 `…EhNI8j6…`)인 젯슨.
결과: 같은 폴더가 063613a16 계열(GitHub 주소, 새 공개키 `MCowBQYDK2VwAyEAfGh+…`)로 바뀜.

## 출처(두 판을 따로 씀)
- 갱신기 묶음 판 = **063613a16** (063613a1650b525564f852cecf14c2c8104e74d7). 젯슨 `updater/`에 들어가는 8개 파일은 이 커밋 그대로(현재 브랜치 파일과 바이트 동일 확인).
- 전환 도구 판 = `tools/jetlink/switch_updater.py`는 063613a16에 **없는 신규 파일**. 처음 39c7e6985에서 추가, 이번 보완판은 아래 해시의 커밋(보고서·`git log -- tools/jetlink/switch_updater.py`로 확인). 파일 sha256은 아래 표.
- 시험: `tools/jetlink/test_switch_updater.py`(39c7e6985), `tools/jetlink/test_switch_review.py`(보완판, 신규).

갱신기 묶음 파일 sha256 (063613a16 `tools/jetlink/`):
- `finalize_sd_image.py` 8ae49a9772d80e68cb0f1c151567c40b582faa617e691669d1485956fc961813
- `hud_protocol.py` d32f96d921381028f0800df2d97779751cb1caae2a7459a46c7c1ccf34cb094f
- `release-signing-public.pem` 8e18f68f63a904c8fefdb91622faeeec8b6d7d1e2de591532585bc9131b0713d
- `wifi_protocol.py` 6ffa68fafe9e83599487b35ff3b17ef53e1acbd938ea9ff746737c91668a77c3
- `local_release.py` fc92e7491d0c5c7f306f97b7a1e8f11ddd5d86c865a82227da6ee0b57a6fff1b
- `usb_release.py` 8bd1d17261c29bcfe2f7ea8390a34152486294606affb5b6658c048e55447b34
- `boot_update.py` ff3662d5640a8ba6c14b7622506a0a55afaae2aa1cc81c6b87948eb7c6bb3e0e
- `update_host.py` 2c8b2ea0a772bf5f5a2edce489237b698ecd98f458d46372fb7dc73430963ab3

전환 도구 sha256 (보완판):
- `switch_updater.py` fc7be77512b11fec1b636a8209e10e19f44e9888809c7082a928984fcc10c878

## 왜 따로 필요한가
- `update_host.activate()`는 `current` 링크만 바꾸고 `updater/`는 안 바꿈.
- 갱신기 설치는 `install_updates.bootstrap()`뿐인데, 옛 실행본의 `migrate_running_release()`와
  오프라인 설치는 `boot-update-required`가 있으면 건너뜀.
- 반대로 `boot-update-required`가 **없으면** 옛 실행본(0cb67d46c) 서버가 켜질 때
  `--bootstrap-only`로 **옛 갱신기를 다시 깔고 표시 파일을 만듦**(가짜 루트 실측, 아래).
- 옛 갱신기 + 표시 파일(다른 부팅 id) = 다음 부팅 관문이 켜지고, 옛 갱신기는 GitHub 명세를 NAS 주소 아님으로 거부
  → 서버가 `wait_for_runtime()`에서 계속 기다림 = **모델 안 뜸**(가짜 루트 실측). 스크립트는 이 조합을 만들지 않는다.

## 스크립트 단계 (보완판 순서)
1. 백업 `<runtime>/updater-switch-backup/<시각>/` (updater 전체·서비스 3개·boot-update-required·해시 목록). 이미 있으면 중단.
2. `updater.switch-<시각>/`에 새 파일 작성, 파일마다 fsync. 옛 폴더의 다른 항목: `switch_updater.py`와 31372d20c `switch_updater_linked.py`는 **최상위 보통 파일만** 옮겼다(하위 폴더는 빠지고, 파일을 가리키는 심볼릭 링크는 보통 파일로 바뀜). 0cb67d46c 기준 실제 updater에 생길 수 있는 하위 폴더: `before-usb-experiments/`(`install_usb_receiver.py:42-49`, 0700), 보호 이미지가 아니면 `__pycache__/`(보호 이미지만 `PYTHONDONTWRITEBYTECODE=1`, `install_protected.py:84-86`). 0cb67d46c 코드가 updater 안에 심볼릭 링크를 만드는 곳은 없음(소스 검색). 07:23 판(아래)은 하위 폴더·링크·권한을 그대로 옮기고 `__pycache__/`만 뺀다.
3. 사전 확인: 그 폴더의 update_host로 콤마 명세(host_release.json) 서명·주소 검사.
4. `renameat2(RENAME_EXCHANGE)`로 폴더 맞바꿈(한 번에). renameat2가 없으면 아무것도 안 바꾸고 중단.
5. **맞바꿈 바로 뒤** boot-update-required가 없을 때만 생성(내용 = 이번 부팅 id). 같은 폴더라 fsync 한 번으로 맞바꿈과 함께 기록.
6. 옛 폴더를 `updater.replaced-<시각>`로 이름 바꿈 → 설치된 폴더로 다시 서명·주소 검사 + 파일 해시 비교.
4 뒤 어떤 단계든 실패(맞바꿈 뒤 첫 fsync 포함) → 복구:
- 먼저 표시 파일을 원래대로: 원래 없었으면 새 표시를 지우지 않고 `boot-update-required.rolledback-<시각>`으로 치움, 원래 있었으면 원래 내용.
- 그다음 맞바꿈으로 옛 폴더 복귀 → 백업과 해시 비교 + 옛 갱신기 import·NAS 주소 허용 확인.
- 이 순서라 복구 도중 프로세스가 죽어도 "옛 갱신기 + 이 도구가 쓴 표시"는 생기지 않음(남는 것은 "새 갱신기 + 표시 없음" → 현재 도구에 마련된 대응은 수동 재실행).
지우는 것은 없음(백업·옛 폴더·치운 표시 모두 남김).

## 중단 시험의 뜻 (중요)
- 시험의 중단(`JET_SWITCH_CRASH_AT`)은 `os._exit`로 **프로세스 강제 종료를 흉내낸 것**뿐이다.
  실제 전원 차단 때 디스크에 무엇이 남는지(쓰기 캐시·파일시스템 저널 순서)는 **입증하지 않는다**(미확인).
- 오류 주입(`JET_SWITCH_FAIL_AT`)은 정해진 지점에서 I/O 오류를 던져 복구 경로만 시험한다.

## 맞바꿈 뒤·표시 기록 전 중단 → 구판 부팅이 먼저 도는 경우 (가짜 루트 실측)
구판 부팅 순서 모의: ① `carrot-jetlink-update-apply.service`(Before=carrot-jetlink) = 설치된 updater의 `update_host.py activate`, 관문 켜지면 Gate.select(콤마 명세) ② `carrot-jetlink.service` = current(0cb67d46c)의 server.py → `migrate_running_release()`(`--bootstrap-only`는 진짜 옛 `bootstrap()`을 가짜 루트에 실행) → `wait_for_runtime()` 판단. 재부팅 2번(boot-B, boot-C).
- 중단 지점 `after_exchange`, `after_marker_tmp`(표시 이름 바꾸기 전): boot-B는 정상 시작하지만 서버가 옛 갱신기를 다시 깔고 표시(boot-B)를 씀 → **전환 되돌려짐**. boot-C는 관문 켜짐 + 옛 갱신기가 GitHub 명세 거부 → 모델 막힘.
  - 복구 실측: 그 상태에서 전환 도구를 다시 돌리면 `switched`, 그다음 부팅에서 관문이 063613a1 선택.
- 중단 지점 `after_marker_rename`, `after_marker`, `after_rename_old`, `after_verify`: 두 번 재부팅 모두 새 갱신기 유지, 재설치 호출 없음, 관문이 063613a1 선택.
- 정상 전환 뒤 같은 부팅(boot-A)에서 서버 재시작: 관문 꺼짐, 재설치 없음.
- 위험 조합 평가:
  - 표시를 맞바꿈 **앞**에 쓰는 순서: 그 사이 중단 = 옛 갱신기 + 표시 → **바로 다음 부팅**에 모델 막힘. 채택 안 함.
  - 표시를 맞바꿈 **뒤**(39c7e6985는 검사 뒤 = 하위 프로세스 검사 시간만큼 창이 김): 이번 판은 맞바꿈 직후로 당겨 창을 표시 임시 파일 기록+이름 바꾸기 사이로 줄임. 현재 두 항목(updater 폴더, 표시 파일)을 따로 바꾸는 구현에는 창이 남음. 다른 전환·복구 설계까지 불가능하다고 입증한 것은 아님.
  - 남은 창에 걸리면: 다음 부팅은 정상, 그다음 부팅부터 모델 막힘 → 전환 직후 `--state`가 `{"key":"new","complete":true,"marker":true}`인지 반드시 확인, 아니면 재부팅 전에 다시 실행.
  - **남은 위험(`switch_updater.py` 기준)**: 이 도구에 마련된 대응은 재부팅 전 사람이 확인하고 수동으로 다시 실행하는 것뿐이며, 자동 복구가 아니다. 갑작스러운 전원 차단이면 재부팅 전에 이 확인을 할 수 있다고 보장할 수 없고, 그 경우 위 "그다음 부팅부터 모델 막힘"이 그대로 일어날 수 있다.
  - 다른 복구 설계는 이 판 작성 때 아직 검토하지 않았다(검토 중이었음). 그 뒤 검토·시제작 결과는 아래 "v3 검토" 절(`switch_updater_linked.py`, 대감님 승인 전 시제작).
- 39c7e6985 시험 `test_power_cut_new_without_marker_then_legacy_start[after_rename_old]`은 옛 순서(검사 뒤 표시)를 전제로 "after_rename_old에서 표시 없음"을 단언하므로 이번 순서 수정 뒤 실패한다. 기존 시험 단언을 바꾸지 않는 규칙에 따라 그대로 두었고, 새 순서의 기대값은 `test_switch_review.py`가 단언한다.

## 다음 부팅 모의 범위 (`test_switch_review.py::test_next_boot_downloads_verifies_unpacks_activates`)
전환된 가짜 루트에서 설치된 updater의 `boot_update.Gate`를 그대로 돌림:
- 확인: 관문 켜짐 → 명세 서명 검사(**시험용 키**, 같은 코드 경로) → GitHub 주소 검사 → 묶음 다운로드(로컬 가짜 응답, 주소는 GitHub/asset 규칙 통과) → 길이·sha256 검사 → 풀기(`extract_bundle`) → `check_bundle`(SOURCE_COMMIT·모델 계약) → 모델 다운로드·sha → `systemctl stop`(기록만) → `activate()` → `current` 링크가 새 릴리스 → status applied, transaction·pending 없음 → READY(부팅 id·커밋).
- 깨진 묶음: sha 불일치로 실패, `current` 그대로, releases에 새 폴더 없음.
- 진짜 키 서명 검사는 별도 시험(`test_real_github_manifest_signature_verifies_with_new_key_only`)에서 실제 콤마 명세로 확인.
- 미확인: 실제 GitHub 내려받기·리다이렉트, `check_runtime`(dpkg L4T·TensorRT, 건너뜀), `probe_release`(GPU 시험 실행, 건너뜀), jetlink 사용자 chown, 실제 systemctl 동작, 새 릴리스의 install_wifi, 그 뒤 서버 시작.

## 장치 적용 순서 (사람이 직접, 이 문서는 확정 절차 아님)
사전 확인
- 차 주차, 젯슨 전원 계속 유지(콤마·젯슨 켜진 상태), 진행 중 전원 끊지 않기.
- 콤마가 cdc5ef408 이상이고 `host_release.json`이 GitHub 주소·063613a1인지.
- 젯슨 현재 상태 읽기(쓰기 없음):
  - `ls -la /opt/carrot-jetlink/updater/` 와 `sha256sum /opt/carrot-jetlink/updater/*`
  - `cat /opt/carrot-jetlink/updater/release-signing-public.pem` → `…EhNI8j6…`(옛 키)인지
  - `grep -n synology /opt/carrot-jetlink/updater/update_host.py` → NAS 주소인지
  - `ls -la /opt/carrot-jetlink/boot-update-required; cat` 그 파일(있는지/내용)
  - `cat /etc/systemd/system/carrot-jetlink-update-*.service` → ExecStart가 `/opt/carrot-jetlink/updater/update_host.py`인지
  - `readlink /opt/carrot-jetlink/current; cat /opt/carrot-jetlink/current/SOURCE_COMMIT`
  - `/opt/carrot-jetlink/venv/bin/python -c "import Crypto"` 성공하는지
  - `df -h /opt/carrot-jetlink` 여유 공간(수백 KB 필요)
  - 보호 이미지면 `cat /run/carrot-storage.json` 상태가 protected인지
명령
1. 063613a16의 `tools/jetlink/` 8개 파일(위 sha256)과 보완판 `switch_updater.py`(위 sha256), 콤마의 `host_release.json`을 젯슨 한 폴더(예 `/home/jetlink/switch0607/`)에 둔다. `sha256sum`으로 위 값과 같은지 확인.
2. 미리보기(쓰기 없음): `sudo /opt/carrot-jetlink/venv/bin/python switch_updater.py --state`
3. 전환: `sudo /opt/carrot-jetlink/venv/bin/python switch_updater.py --manifest host_release.json`
   - 성공 출력: `"result": "switched"`, `"selected_commit": "063613a1…"`.
   - 실패 출력: `FAILED: … old updater restored` + 상태 줄(key old, marker는 원래대로).
4. 확인: `--state`가 `{"key":"new","complete":true,"marker":true}`; 아니면 재부팅 전에 3을 다시 실행. `ls /opt/carrot-jetlink/updater-switch-backup/`.
5. 재부팅은 사람이 판단. 다음 부팅 때 관문이 켜지고 콤마 명세로 063613a1을 받으려 함(인터넷 필요, 모델 시작 늦어질 수 있음).
되돌리기(필요할 때만): 백업 폴더의 `updater/`를 같은 방식(새 폴더 → 맞바꿈)으로 되돌리고, 백업에 boot-update-required가 없었다면 그 파일을 치움. 스크립트 기능 아님, 사람 판단.

## 미확인
- 실제 전원 차단 때 기록 보존(시험은 프로세스 강제 종료 모의뿐).
- 실제 젯슨의 updater 파일 내용·키·boot-update-required 유무(읽기 전 미확인).
- 젯슨 커널/파일시스템에서 renameat2 RENAME_EXCHANGE 동작(워커에서만 시험).
- 다음 부팅의 실제 다운로드·L4T/TensorRT 검사·GPU probe·서버 시작(위 범위 밖).
- 구판 부팅의 stage 타이머(OnBootSec=2min, `automatic`): `switch_updater.py` 시험에는 없음. v3 검토에서 가짜 루트로 측정(아래).
- 보호 저장 이미지의 `/etc/carrot-jetlink-protected.json`·`/run/carrot-storage.json` 실제 값.
- 옛 개인키 존재 여부(못 찾음, 없다고 단정 안 함).

## v3 검토 (노비 11/16 06:28): 타이머·남은 창 자동 복구 — `switch_updater_linked.py` 시제작, 대감님 승인 전
기존 `switch_updater.py`와 기존 시험 2개는 바꾸지 않았다. 새 파일 `tools/jetlink/switch_updater_linked.py`, 새 시험 `tools/jetlink/test_switch_linked.py`.
모든 측정은 워커 가짜 루트(구판 0cb67d46c 실제 파일). `os._exit` 중단은 프로세스 강제 종료 모의이며 실제 전원 차단 입증이 아니다.

### 구판 자동 갱신 타이머 경로 (0cb67d46c 소스)
- 타이머 `carrot-jetlink-update-stage.timer`: OnBootSec=2min, OnUnitActiveSec=15min, RandomizedDelaySec=30 (`install_updates.py:150-157`).
- 서비스 `carrot-jetlink-update-stage.service`: `ExecStart=/opt/carrot-jetlink/venv/bin/python /opt/carrot-jetlink/updater/update_host.py automatic` (`install_updates.py:125,139-148`). 즉 **그 순간 설치된 updater 폴더**의 코드가 돈다.
- `update_host.main()` (`update_host.py:335-371`): `boot_update.enabled()`가 참(표시 있음 + 부팅 id 다름)이면 `automatic`은 잠금 없이 바로 끝(`:345-346`). 아니면 `/run/carrot-jetlink-update.lock`을 `LOCK_EX|LOCK_NB`로 잡고(`:362-363`, 못 잡으면 BlockingIOError로 실패) `automatic_stage()`.
- `automatic_stage()` (`:307-332`): 콤마 제어 파일(`/dev/shm/carrot-jetlink-bootstrap.json`, 15초 안, onroad False)의 `jetson_release`로 `stage_manifest()`. 구판은 GitHub 주소를 `checked_url`에서 ValueError로 거부, 이 ValueError는 `:317`의 except가 삼키고 HUD 스냅샷 경로(`read_snapshot`, `:320`)로 넘어감.
- 새 판(063613a16) update_host도 같은 잠금 경로(`update_host.py:368-369`)를 쓴다.

### 타이머 측정 (가짜 루트, 네트워크·시스템 명령 막음)
- 전환 전(옛 updater, 표시 없음): 결과 정상 종료, 시스템 명령·네트워크 호출 0, runtime 변경 0.
- 전환 중(새 도구가 잠금 보유, 멈춤 지점 after_lock·after_stage·after_link·after_exchange·after_marker·after_rename_old 6곳): 타이머 BlockingIOError, 변경 0, 호출 0. 전환은 끝까지 성공.
- 업데이트 동작이 먼저 잠금 보유: 새 도구가 기다린 뒤(시험 1.5초) "nothing changed"로 중단, runtime 스냅샷 변화 0. 잠금이 풀리면 기다리던 전환이 이어서 성공.
- 전환 직후 같은 부팅: 새 updater, 관문 꺼짐(표시 = 이번 부팅 id) → 새 코드가 GitHub 명세를 받아 `check_runtime`(dpkg-query) 호출까지 감(가짜 루트라 실패, 변경 0). 실제 젯슨에서는 다음 부팅용 내려받기를 시도할 수 있음(미확인).
- 다음 부팅: 관문 켜짐 → `automatic` 즉시 종료(호출·변경 0).
- 중단 뒤 각 상태에서 부팅마다 apply → 서버 → 타이머 순서, 타이머가 먼저 도는 경우까지: 타이머가 updater·표시를 바꾼 경우 0.
- 잠금이 없는 `switch_updater.py`에서 타이머가 전환과 겹칠 때 옛 프로세스가 새 폴더 모듈을 늦게 import하는 섞임은 측정하지 않음(미확인). 새 도구는 잠금으로 이 겹침 자체를 막는다.

### 남은 창 자동 복구 방안 판정
- (가) 표식을 새 updater 폴더 **안에만** 넣기: **불가**. 구판이 보는 곳은 폴더 밖 `/opt/carrot-jetlink/boot-update-required` 하나(`boot_update.py:25-26`, `install_updates.py:26`).
- (나) 폴더 밖 표시를 폴더 안 파일로 가는 **상대 심볼릭 링크**로 미리 만들기: **가능, 시제작함**. 구판 두 곳 모두 `Path.is_file()`만 봄 → 옛 폴더일 때 링크가 비어 있어 False = 전환 전과 같은 상태, 맞바꿈(renameat2 한 번) 순간 링크가 풀려 True. updater와 표시가 같은 rename으로 함께 바뀐다. 그 뒤 링크를 같은 내용의 보통 파일로 바꿈(표시는 계속 있음). 구판 `bootstrap()`은 표시를 `temporary.replace(marker)`로 쓰므로(`install_updates.py:109-112`) 빈 링크 위에도 정상 동작(시험 확인).
  - 새 도구 위치: 잠금 `switch_updater_linked.py:74`, 안쪽 표식 `:201`, 링크 `:212`, 맞바꿈 `:217`, 보통 파일로 `:98`, 남의 링크면 중단 `:151`, 재실행 마무리 `:134`, 복구 `:253`(보통 파일 → 링크(표시 유지) → 맞바꿈 되돌림(링크 비어 전환 전 상태) → 링크 치움, 지우지 않음).
- (다) 부팅 전 서비스(`carrot-jetlink.service`에 ExecStartPre drop-in)로 새 updater면 표시 보충: **조건부 가능, 시제작 안 함**. 맞바꿈 전에 /etc에 새 파일을 써야 하고, 보호 이미지가 /etc 쓰기를 허용하는지 미확인. (나)가 추가 서비스 없이 같은 효과를 내서 보류.
- (라) `migrate_running_release`가 읽는 조건 활용: (나)가 바로 이것(`install_updates.py:26`의 `is_file()`).
- (마) 표시를 먼저 쓰되 옛 갱신기+표식이 안전하게: **불가**. 표시가 있으면 다음 부팅 관문이 옛 updater의 `Gate`로 돌고 옛 코드는 GitHub 명세를 거부(기존 실측 `test_before_switch_old_updater_rejects_github`). 옛 코드를 바꾸려면 결국 폴더 교체가 필요.

### (나) 측정: 중단 지점 × 구판 부팅·타이머 × 재부팅 2회
- 중단 12곳(after_lock, after_backup, mid_stage, after_stage, after_link, after_exchange, after_exchange_fsync, after_marker_tmp, after_marker_rename, after_marker, after_rename_old, after_verify) × (바로 재부팅 / 같은 부팅 타이머 뒤 재부팅) = 24경우.
  - 맞바꿈 전 5곳: 옛 updater + 표시 없음 = 도구를 안 돌린 것과 같은 상태(그 뒤는 구판 원래 동작).
  - 맞바꿈부터 7곳: 새 updater + 표시 있음, 두 번 재부팅 모두 재설치 호출 없음, 관문이 063613a1을 고름(아래 '모델 판정' 참고). **`switch_updater.py`에 있던 after_exchange·after_marker_tmp 창이 사라짐.**
  - 모든 경우 그 뒤 재실행으로 완료(링크 남지 않고 보통 파일).
- 오류 주입 5곳 × 표시 없음/있음: 전환 전 상태로 정확히 복구(표시 원래대로, 링크는 rolledback 이름으로 치움), 두 번 부팅 뒤 재실행 성공.
- 복구 중 두 번째 입출력 오류(relink, exchange_back) × 첫 오류 3곳: 남는 상태는 모두 "새 + 표시" 또는 "전환 전"; "새"였던 경우 두 번 부팅 모두 관문이 063613a1을 고름; 재실행 성공.
- 복구 중 강제 종료(rec_after_relink, rec_after_exchange_back) × 첫 오류 3곳: 각각 "새 + 표시", "전환 전"; 재실행 성공.
- 남은 위험: 맞바꿈 전 5곳에서 끊기면 전환 전 상태(표시 없는 구판이면 구판 서버가 원래대로 옛 갱신기를 다시 깜). 이는 도구가 만든 상태가 아니라 원래 상태.

### v3 미확인
- 실제 전원 차단 때 링크 생성·맞바꿈의 디스크 기록 순서(ext4 저널 동작, 젯슨 커널) — 프로세스 종료 모의뿐.
- 실제 젯슨에서 renameat2 동작, 실제 systemd 타이머 시각·동시 실행, 실제 HUD 스냅샷 경로(`read_snapshot`)를 탄 경우.
- 전환 직후 같은 부팅에서 새 updater 타이머가 실제로 내려받기를 하는지(가짜 루트에선 dpkg 단계에서 멈춤).
- 장치의 boot-update-required 실제 유무(있으면 링크 안은 쓰이지 않고 창 자체가 없음).

### 모델 판정 표현 정정 (노비 11/16 07:22)
`test_switch_review.py:78-83`의 `model_runs()`는 관문이 켜졌으면 **관문이 고르는 판이 콤마 명세 커밋(063613a1)인지**, 꺼졌으면 **`wait_for_runtime()`이 기다리는지**만 본다. 실제 내려받기·TensorRT·모델 시작은 보지 않는다. 이 문서의 "모델 뜸"은 "관문이 063613a1을 고름(또는 기다리지 않음)", "모델 막힘/안 뜸"은 "관문이 판을 못 고름(또는 기다림)"으로 읽는다. 시험의 `'runs'`/`'STUCK'` 이름도 같은 뜻이다.

## 07:23 검토 (노비 11/16 07:22): 전환 **중** 구판 서버 재설치 경합
### 누가 언제 부르나 (0cb67d46c 소스)
- `carrot-jetlink.service` (`install_server.sh:15-39`): `ExecStart=… current/tools/jetlink/server.py …`, `Restart=always`, `RestartSec=3`, `StartLimitIntervalSec=0`. 부팅 때뿐 아니라 **서버가 끝날 때마다 3초 뒤 다시 시작**.
- `server.py:10-13`: 젯슨(`/etc/nv_tegra_release`)이면 import 전에 `migrate_running_release()`.
- `install_updates.py:24-29`: `current/tools/jetlink/boot_update.py`가 있고 `boot-update-required`가 **보통 파일이 아니면** `sudo -n … install_updates.py --bootstrap-only` (`:182-186`) → `bootstrap(/, current/tools/jetlink, defer_this_boot=True)`.
- `bootstrap()` `:90-114`: 잠금 없음. `updater/<이름>.new`를 쓰고 `replace(updater/<이름>)` — **그 순간 `updater/`라는 이름이 가리키는 폴더**에 쓴다. 마지막에 표시(이번 부팅 id).
- 그 밖: `offline_bootstrap.py:17-27`(이미지 설정 서비스, 서비스들보다 먼저, 부팅 때 한 번), `install_updates.configure()`(이미지 만들 때, `refresh_sd_image.py:77`). 사람이 부팅 뒤 돌리는 전환과 겹칠 수 있는 것은 **서버 시작(재시작) 경로뿐**.
- 판정: **실제로 전환 도중 실행될 수 있다.** 표시가 없는 젯슨에서 전환 중 서버가 죽거나(USB 끊김 등) 누가 재시작하면 3초 뒤 bootstrap이 돈다. 31372d20c 잠금(`/run/carrot-jetlink-update.lock`)은 bootstrap이 안 잡으므로 막지 못한다.

### 측정 (가짜 루트, 실제 0cb67d46c `install_updates.py`, `test_switch_race.py`)
분류: new = 새 갱신기(파일 전부 새 판)+표시 / before = 전환 전 그대로 / old+legacy-marker = 옛 갱신기 + 구판 bootstrap이 쓴 표시(도구 없이 구판 서버가 원래 하는 일) / MIXED = 섞임. 각 끝 상태마다 재부팅 2회(apply → 서버 → 타이머).
- 31372d20c(서버 막기 없음):
  - 멈춤 12곳마다 서버 한 번 통째로 시작: 12/12 new. (맞바꿈 전 bootstrap은 옛 폴더에 같은 옛 파일을 쓰고 끝, 맞바꿈 뒤에는 표시가 보통 파일이라 건너뜀.)
  - bootstrap이 전환 **전에** 시작해 k개(1·4·8) 쓴 뒤 멈춤, 전환 멈춤 12곳에서 재개 = 36경우: new 22, before 12(맞바꿈~검사 사이에 재개 → 도구 검사가 잡아 복구, 도구는 실패), **after_verify 뒤 재개 2경우: 도구는 "switched"라고 끝났는데 k=1 → 옛 갱신기+표시, k=4 → 새 폴더에 옛 `update_host.py` 섞임. 두 경우 모두 재부팅 2회 모두 관문이 판을 못 고름.** (`test_race_reproduced_unguarded_switch_reports_switched_but_stuck`)
  - 진짜 동시 실행, 31372d20c 코드 그대로(워커 `race_on_31372d20c.log`): 프로세스 30회(서버를 끝나자마자 다시 시작, 서버 시작 287번) → new 28, **old+legacy-marker 1(도구 실패·복구 뒤 서버가 재설치)**, **new 폴더에 죽은 bootstrap의 `wifi_protocol.py.new` 찌꺼기 1**. 같은 조건 두 번째 30회(`race_fixed.log`의 `off processes`)는 30/30 new — 무작위로는 드물게 잡힘. 스레드 30회(같은 프로세스에서 bootstrap 반복): 30/30 new.
- 07:23 판(서버 막기, 아래): 멈춤 13곳 서버 시작 → after_lock(막기 전)만 시작되고 나머지 12곳 시작 안 됨, 13/13 new·재부팅 2회 모두 관문이 063613a1 고름. bootstrap 진행 중(k=1·4·8) × 서비스 묶음 안(정지가 죽임)/밖(sudo로 빠져나간 경우 기다림) 6경우 모두 new. 진짜 동시 실행 프로세스 30회(서버 시작 1453번): 30/30 new(처음 측정에서 정지로 죽은 bootstrap의 `.new` 찌꺼기가 새 폴더로 옮겨진 2회를 찾아, 그 찌꺼기는 백업에만 남기고 새 폴더로는 안 옮기게 고침). 밖에서 끝나지 않으면 "nothing changed"로 중단, 변경 0, 마스크 풀림. 복구 지점 2곳: 마스크 뒤 다시 시작된 서버가 구판대로 재설치 → old+legacy-marker(도구 없이 서버가 재시작한 것과 같음). 막은 뒤 강제 종료 5곳: 마스크가 /run에 남아 서버 안 뜸 → 재부팅 때 풀림, 끝 상태 before 또는 new.

### 고친 것 (`switch_updater_linked.py`, 31372d20c 위 새 커밋)
- 잠금을 잡은 뒤, 어떤 쓰기보다 먼저: `systemctl is-active` → `systemctl mask --runtime carrot-jetlink.service`(이번 부팅만, /run) → `systemctl stop` → 다시 is-active 확인 → `install_updates.py --bootstrap-only` 프로세스가 없을 때까지 기다림(잠금 대기 시간 안, 넘으면 변경 없이 중단). 끝나면(성공·실패 모두) `unmask --runtime`, 원래 켜져 있었으면 `start --no-block`. 다시 뜬 서버는 표시가 보통 파일이라 bootstrap을 건너뜀(시험 확인).
- 하위 폴더·링크·권한 보존(`copy_entry`), `__pycache__/`만 제외.
- 구판 파일 자체를 장치에서 고치는 방안은 쓰지 않았다: 서명된 `current` 릴리스를 바꿔야 하고(보호 이미지면 읽기 전용·서명 검사 깨짐), 고친 파일이 다음 갱신에 덮이며, 차마다 수동 작업이 된다.
- 이 방안의 위험: 전환 동안 추론 서버가 멈춤(주차 상태에서만). 도구가 강제 종료되면 재부팅 전까지 서버가 안 뜸(마스크가 /run에 남음 — 재부팅하면 풀림, 수동이면 `systemctl unmask --runtime carrot-jetlink.service && systemctl start carrot-jetlink`). sudo가 bootstrap을 다른 cgroup으로 옮기는지는 미확인 → 그래서 프로세스 목록으로 한 번 더 확인.

### 07:23 미확인
- 실제 systemd(mask --runtime·stop·cgroup 정리) — 시험은 가짜 systemctl 모형. 실제 젯슨·실제 전원 차단·실제 sudo/PAM 세션 동작 미확인.
- HUD 등 다른 서비스가 `carrot-jetlink.service`를 끌어올리는지(`Requires=`/`Wants=`) — 마스크면 시작은 거부되지만 그 서비스 쪽 실패 동작은 미확인.
