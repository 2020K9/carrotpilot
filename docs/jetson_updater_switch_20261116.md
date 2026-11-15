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
2. `updater.switch-<시각>/`에 새 파일 작성(옛 폴더의 다른 파일은 유지), 파일마다 fsync.
3. 사전 확인: 그 폴더의 update_host로 콤마 명세(host_release.json) 서명·주소 검사.
4. `renameat2(RENAME_EXCHANGE)`로 폴더 맞바꿈(한 번에). renameat2가 없으면 아무것도 안 바꾸고 중단.
5. **맞바꿈 바로 뒤** boot-update-required가 없을 때만 생성(내용 = 이번 부팅 id). 같은 폴더라 fsync 한 번으로 맞바꿈과 함께 기록.
6. 옛 폴더를 `updater.replaced-<시각>`로 이름 바꿈 → 설치된 폴더로 다시 서명·주소 검사 + 파일 해시 비교.
4 뒤 어떤 단계든 실패(맞바꿈 뒤 첫 fsync 포함) → 복구:
- 먼저 표시 파일을 원래대로: 원래 없었으면 새 표시를 지우지 않고 `boot-update-required.rolledback-<시각>`으로 치움, 원래 있었으면 원래 내용.
- 그다음 맞바꿈으로 옛 폴더 복귀 → 백업과 해시 비교 + 옛 갱신기 import·NAS 주소 허용 확인.
- 이 순서라 복구 도중 프로세스가 죽어도 "옛 갱신기 + 이 도구가 쓴 표시"는 생기지 않음(남는 것은 "새 갱신기 + 표시 없음" → 다시 실행하면 마무리).
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
  - 표시를 맞바꿈 **뒤**(39c7e6985는 검사 뒤 = 하위 프로세스 검사 시간만큼 창이 김): 이번 판은 맞바꿈 직후로 당겨 창을 표시 임시 파일 기록+이름 바꾸기 사이로 줄임. 창이 0은 아님(두 파일이 따로라 원자적으로 함께 못 바꿈).
  - 남은 창에 걸리면: 다음 부팅은 정상, 그다음 부팅부터 모델 막힘 → 전환 직후 `--state`가 `{"key":"new","complete":true,"marker":true}`인지 반드시 확인, 아니면 재부팅 전에 다시 실행.
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
- 구판 부팅의 stage 타이머(OnBootSec=2min, `automatic`)는 모의에 넣지 않음.
- 보호 저장 이미지의 `/etc/carrot-jetlink-protected.json`·`/run/carrot-storage.json` 실제 값.
- 옛 개인키 존재 여부(못 찾음, 없다고 단정 안 함).
