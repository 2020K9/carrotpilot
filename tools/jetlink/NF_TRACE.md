# K9 NOT_FINITE 추적 (nf_trace)

대감님 2026-11-21 20:57 "추적프로그램 만들면 되지 않니?" — 4번 에러(NOT_FINITE, Light speed 2에서만, 간격이 짧아지다 젯슨 무응답·재부팅)의 원인을 다음 주행에서 자동으로 잡는다. 기존 JL_NF 줄(6e691a2ff·48f01799f)을 그대로 두고 그 뒤에 붙였다.

- 항상 켜져 있다. 메뉴·설정이 없다. 에러가 난 때만 파일을 쓴다.
- 제어 값, 출력, 상태 이어받기, 에러 처리는 바꾸지 않는다. 복사만 한다. 추적기에서 오류가 나도 삼키고 넘어간다.

## 젯슨 (`third_party/jetlink/jetlink/server/nf_trace.py`)
- 매 프레임: 상태를 뺀 입력(new_img·desire·traffic_convention·action_t)을 5칸 고리에 복사하고, 답을 보낸 뒤 그 프레임의 `outputs`도 복사한다. 그 프레임에 들어간 순환 상태는 GPU 상태 경로에서 그래프 직전에 GPU 안에서 비동기로 한 번 복사하고(`DeviceState.trace_snapshot`), 호스트 상태 경로에서는 호스트에서 복사한다.
- NOT_FINITE(답을 보낸 뒤): 그 프레임의 입력·입력 상태·출력 전부(GPU 다음 상태 포함)와 직전 정상 프레임 4개의 입력·출력을 쓰기 스레드로 넘긴다. 텐서는 15초에 한 번까지만 저장하고, 그보다 잦은 에러는 meta와 건강 기록만 남긴다.
- 재실행: 링크가 2초 동안 비면 같은 입력과 상태를 같은 엔진에 3번 다시 넣는다. 콤마는 NOT_FINITE 뒤 매번 다시 접속하므로 이 빈 틈이 생긴다. 재실행은 `host.lock` 아래에서 돌고, 살아 있는 상태·입력 버퍼·플래그를 따로 두었다가 그대로 되돌린다.
  - `nan_again`: 같은 입력에서 또 NaN이 난다. 입력이나 모델 수치 문제다(fp16 넘침 등).
  - `clean`: 다시 넣으면 정상이다. GPU 클럭·전원·커널·메모리 쪽 순간 문제다.
  - `mixed`: 결과가 들쭉날쭉하다. GPU나 전원이 불안정하다고 본다.
- 건강 기록: host_performance가 1초마다 쓰는 상태 파일(GPU 클럭·부하, 전원 레일 mV/mA, nvpmodel, cooling/throttle, EMC, 메모리)과 thermal zone 온도를 1초 고리 버퍼에 모은다. 에러 때 앞 60초를 바로 쓰고, 마지막 에러 뒤 60초까지는 1초마다 이어 쓰고 fsync한다. 그래서 바로 뒤에 멈추거나 재부팅해도 그 직전까지 남는다. dmesg에서 nvgpu·mmu·xid·throttle·전압·usb 같은 줄을 골라 에러 때와 60초 뒤에 저장한다. 재부팅으로 기록이 끊긴 사건은 다음 부팅 때 `interrupted`로 표시하고, 직전 부팅 커널 로그(`journalctl -k -b -1`)를 붙인다.
- 저장 위치: 젯슨 서버 `--cache` 폴더 아래 `nf_trace/<UTC시각>-b<boot>-f<frame>-<n>/`. 설치본에서는 `<설치 ROOT>/cache/nf_trace`이고, 보통 `/opt/carrot-jetlink/cache/nf_trace`다(install_server.sh). 엔진 캐시와 같은 디스크라 재부팅해도 남는다. 최대 8건, 합계 400MB를 넘으면 오래된 것부터 지운다.
  - `meta.json`: JL_NF 줄, 재실행 결과, 텐서 통계(비유한 개수, max|x|)
  - `tensors.npz`: `inputs__*`, `state__*`, `outputs__*`, `prev<i>__*`
  - `health_pre.jsonl`, `health_post.jsonl`
  - `dmesg_at_error.txt`, `dmesg_after.txt`, `prev_boot_kernel.txt`

## 콤마 (`tools/jetlink/nf_trace_link.py`, jetlinkd)
- rlog: 사건이 생기거나 재실행 결과가 나올 때마다 `jetsonNfTrace` 이벤트를 한 줄 남긴다(`line="JL_NF_TRACE id=… frame=… replay=nan_again … JL_NF …"`). 주행 중에도 남는다. 젯슨이 이미 보내는 상태 JSON의 `carrot_nf_trace`만 읽으므로 USB 통신은 늘지 않는다.
- 파일 끌어오기: 오프로드(IsOffroad)일 때만, 5초마다 최대 8MB씩 jetlink 메시지 0x4060/0x4061로 가져와 `/data/media/0/k9_nf_trace/<id>/`에 둔다. 다 받으면 `.complete`를 만들고, 합계 1.5GB를 넘으면 오래된 것부터 지운다.

## 가져와서 보기 (게이트웨이 → 콤마 → 워커)
```
comma-ssh 'tar -C /data/media/0 -czf /data/media/0/k9_nf_trace.tgz k9_nf_trace'
comma-scp-secretary2 get /data/media/0/k9_nf_trace.tgz .openclaw/tmp/k9_nf_trace.tgz
worker-scp-secretary2 put .openclaw/tmp/k9_nf_trace.tgz /home/ssm-user/k9_research/nf_trace/k9_nf_trace.tgz
worker-ssh 'cd ~/k9_research/nf_trace && tar xzf k9_nf_trace.tgz && python3 <carrot>/tools/jetlink/nf_trace_analyze.py k9_nf_trace [--onnx light-speed-2.onnx]'
```
- rlog만 볼 때는 logMessage에서 `jetsonNfTrace`를 찾는다.
- `--onnx`를 주면 받은 입력과 상태를 워커의 onnxruntime fp32로 다시 돌린다. 젯슨에서는 NaN인데 fp32에서 유한하면 TensorRT fp16 정밀도 문제다.
