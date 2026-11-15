# 구판 젯슨 갱신기 일회성 전환 (NAS → GitHub), 2026-11-16

대상: `/opt/carrot-jetlink/updater/`가 0cb67d46c 계열(NAS `/models/` 주소만, 옛 공개키 `…EhNI8j6…`)인 젯슨.
결과: 같은 폴더가 063613a1 계열(GitHub 주소, 새 공개키 `MCowBQYDK2VwAyEAfGh+…`)로 바뀜.
도구: `tools/jetlink/switch_updater.py`, 시험 `tools/jetlink/test_switch_updater.py`(가짜 루트 폴더).

## 왜 따로 필요한가
- `update_host.activate()`는 `current` 링크만 바꾸고 `updater/`는 안 바꿈.
- 갱신기 설치는 `install_updates.bootstrap()`뿐인데, 옛 실행본의 `migrate_running_release()`와
  오프라인 설치는 `boot-update-required`가 있으면 건너뜀.
- 반대로 `boot-update-required`가 **없으면** 옛 실행본(0cb67d46c) 서버가 켜질 때
  `--bootstrap-only`로 **옛 갱신기를 다시 깔아** 전환을 되돌림(시험 `test_marker_absent_legacy_runtime_would_reinstall_old_updater`).
  그래서 스크립트는 새 갱신기 확인이 끝난 **뒤에만** 이 파일을 만든다(내용 = 이번 부팅 id → 이번 부팅은 그대로, 다음 부팅부터 관문 켜짐).
- 옛 갱신기 + 이 파일 조합은 GitHub 명세를 못 받아 모델이 막힐 수 있으므로, 스크립트는 그 조합을 만들지 않는다.

## 스크립트 단계
1. 백업 `<runtime>/updater-switch-backup/<시각>/` (updater 전체·서비스 3개·boot-update-required·해시 목록). 이미 있으면 중단.
2. `updater.switch-<시각>/`에 새 파일 작성(옛 폴더의 다른 파일은 유지), 파일마다 fsync.
3. 사전 확인: 그 폴더의 update_host로 콤마 명세(host_release.json) 서명·주소 검사.
4. `renameat2(RENAME_EXCHANGE)`로 폴더 맞바꿈(한 번에). 옛 폴더는 `updater.replaced-<시각>`.
5. 설치된 폴더로 다시 서명·주소 검사 + 파일 해시 비교.
6. boot-update-required가 없을 때만 생성.
3~6 실패 → 맞바꿈으로 옛 폴더 복귀 → 백업과 해시 비교 + 옛 갱신기 import·NAS 주소 허용 확인.
지우는 것은 없음(백업·옛 폴더 모두 남김).

## 장치 적용 순서 (사람이 직접, 이 문서는 확정 절차 아님)
사전 확인
- 차 주차, 시동 끔이 아니라 젯슨 전원 계속 유지(콤마·젯슨 켜진 상태), 진행 중 전원 끊지 않기.
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
1. 063613a1의 `tools/jetlink/` 중 `switch_updater.py`, `finalize_sd_image.py hud_protocol.py release-signing-public.pem wifi_protocol.py local_release.py usb_release.py boot_update.py update_host.py`
   와 콤마의 `host_release.json`을 젯슨 한 폴더(예 `/home/jetlink/switch0607/`)에 둔다. `sha256sum`으로 원본과 같은지 확인.
2. 미리보기(쓰기 없음): `sudo /opt/carrot-jetlink/venv/bin/python switch_updater.py --state`
3. 전환: `sudo /opt/carrot-jetlink/venv/bin/python switch_updater.py --manifest host_release.json`
   - 성공 출력: `"result": "switched"`, `"selected_commit": "063613a1…"`.
   - 실패 출력: `FAILED: … old updater restored` + 상태 줄(key old).
4. 확인: `--state`가 `{"key":"new","complete":true,"marker":true}`; `ls /opt/carrot-jetlink/updater-switch-backup/`.
5. 재부팅은 사람이 판단. 다음 부팅 때 관문(boot gate)이 켜지고 콤마 명세로 063613a1을 받으려 함(인터넷 필요, 모델 시작 늦어질 수 있음).
되돌리기(필요할 때만): 백업 폴더의 `updater/`를 같은 방식(새 폴더 → 맞바꿈)으로 되돌리고, 백업에 boot-update-required가 없었다면 그 파일을 치움. 스크립트 기능 아님, 사람 판단.

## 전원 끊김
- 2단계 전까지 끊김: 옛 갱신기 그대로(남은 백업/작업 폴더는 무해).
- 4단계 맞바꿈은 한 번에 일어나므로 updater는 언제나 옛것 전체 또는 새것 전체.
- 4~6 사이 끊김(새 갱신기, 파일 없음): 다시 실행하면 `already-new`로 6단계를 마무리.
  다시 실행하기 전에 옛 실행본 서버가 켜지면 옛 갱신기를 전체로 다시 깔고 파일을 만든다
  → 옛 갱신기 + 관문 켜짐 = GitHub 명세 거부 상태가 될 수 있음(시험에서 호출만 확인, 실제 결과 미확인).
  그래서 전환 직후 바로 `--state` 확인 필요.

## 미확인
- 실제 젯슨의 updater 파일 내용·키·boot-update-required 유무(읽기 전 미확인).
- 젯슨 커널/파일시스템에서 renameat2 RENAME_EXCHANGE 동작(워커 ext4/xfs에서만 시험, 없으면 아무것도 안 바꾸고 중단).
- 실제 부팅 뒤 관문이 063613a1을 내려받아 적용하는지(이 시험은 명세 선택까지만).
- 보호 저장 이미지의 `/etc/carrot-jetlink-protected.json`·`/run/carrot-storage.json` 실제 값.
- 옛 개인키 존재 여부(못 찾음, 없다고 단정 안 함).
