# 2020K9 GitHub 배포판 + 대감님 서명키 (2026-11-15)

- 받는 곳: `https://github.com/2020K9/carrotpilot/releases/download/k9-<옛 NAS 폴더 이름>/<파일 이름>`
  (옛 `https://upload.shind0.synology.me/models/<폴더>/<파일>` 과 1:1 대응).
- `update_host.checked_url`: 위 github.com 경로만 허용. 깃허브가 넘겨주는 리다이렉트 응답은
  `release-assets.githubusercontent.com/github-production-release-asset/1400258213/`(이 저장소 id)만 허용.
- 서명: `tools/jetlink/release-signing-public.pem` 교체(ed25519). 비밀키는 저장소에 없다.
- 실행 묶음 4개(n16 / md-v3 / light-speed / light-speed-2)는 이 브랜치의 커밋에서
  `build_host_bundle.py` 로 새로 만들었다(묶음마다 고정 모델 커밋이 다르다).

## 이미 설치된 젯슨에 처음 한 번 넣기

이미 설치된 젯슨의 `/opt/carrot-jetlink/updater/` 에는 옛 공개키와 NAS 전용 주소 검사가 있어서
새 서명 매니페스트를 받지 않는다. 새 N16 실행 묶음에서 업데이터만 한 번 직접 바꾼다.

```bash
curl -LO https://github.com/2020K9/carrotpilot/releases/download/k9-jetlink-host-063613a1650b525564f852cecf14c2c8104e74d7/precompiled-runtime.tar.gz
echo "d11da8e0d1f464ad329e0edc6cf477201a043d9ca92440c242468c58586d5424  precompiled-runtime.tar.gz" | sha256sum -c -
mkdir -p /tmp/k9host && tar xzf precompiled-runtime.tar.gz -C /tmp/k9host
sudo systemctl stop carrot-jetlink carrot-jetlink-hud carrot-jetlink-update-stage
sudo /opt/carrot-jetlink/venv/bin/python -c "import sys; sys.path.insert(0, /tmp/k9host/tools/jetlink); from install_updates import bootstrap; bootstrap(/, /tmp/k9host/tools/jetlink)"
sudo reboot
```

그 뒤에는 콤마가 이 브랜치의 `host_release.json`(또는 모델별 `*-release.json`)을 넘기면
젯슨 업데이터가 새 키로 서명을 확인하고 깃허브에서 묶음·모델을 받아 기존 부팅 검사를 거쳐 바꾼다.
