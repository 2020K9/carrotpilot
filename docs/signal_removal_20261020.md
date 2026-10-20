# 신호등 기능 제거 / Traffic-light feature removal (2026-10-20)

- 카메라 기반 보조 신호등 인식·표시·제어(신호보조 모드, 앞차 브레이크등 관찰, 신호 관찰기/추적기,
  modeld 신호 shadow, 오프라인 분석·학습 도구)를 제거했습니다. 내비·폰의 신호 정보 입력, 녹색 자동 출발,
  신호 알림·소리·아이콘도 제거했습니다. 설정 `SignalAssistMode`, `LeadBrakeLights`와 메모리 키
  `TrafficLight`는 삭제되었습니다.
  Removed camera-based auxiliary traffic-light recognition, display and control (Signal Assistance Mode,
  lead brake-lamp observation, signal observer/tracker processes, modeld signal shadow, offline analysis
  and training tools), navigation/phone traffic-light inputs, automatic green departure, and signal
  alerts, sounds and icons. Params `SignalAssistMode`, `LeadBrakeLights` and memory key `TrafficLight`
  are deleted.
- 주행 모델 자체의 일반 정지 예측(앞차 없음)은 안전을 위해 유지하며, 설정 이름을
  `ModelStopPrediction`(`0` 끔, `1` 켬·기본)으로 바꿨습니다. 폐지된 `TrafficLightDetectMode`는
  시작 시 새 키가 없을 때만 한 번 옮겨집니다(`0`→`0`, `1`/`2`→`1`). 옛 키는 이 마이그레이션에서만
  읽으며 플래너는 읽지 않습니다. 이 정지 후에는 자동 출발하지 않으며 가속 페달 또는 일반 앞차 추종으로
  출발합니다. `TrafficStopDistanceAdjust`는 일반 모델 정지 위치 보정으로 남습니다.
  The driving model's generic no-lead stop prediction is kept for safety and renamed
  `ModelStopPrediction` (`0` off, `1` on, default). The retired `TrafficLightDetectMode` is copied once at
  startup only when the new key is unset (`0`->`0`, `1`/`2`->`1`); the old key is read only by that
  migration, never by the planner. No automatic departure follows such a stop: use the accelerator or
  ordinary lead following. `TrafficStopDistanceAdjust` remains the generic model stop-position offset.
- 사전학습 모델 가중치에 담긴 신호 관련 학습은 코드 삭제로 지울 수 없습니다. 현재 모델은 신호 앞에서 스스로
  정지를 예측할 수 있습니다. `ModelStopPrediction=0`은 그 예측을 감속에 쓰지 않을 뿐 모델을 바꾸지 않습니다.
  Signal semantics learned by the pretrained model cannot be removed by deleting code; the current model
  may still predict stops at signals. `ModelStopPrediction=0` only stops using that prediction.
- 로그 호환을 위해 다음 Cap'n Proto 필드/열거값은 삭제하지 않고 값을 채우지 않는 deprecated 항목으로 남깁니다.
  Kept as deprecated, never-populated schema items for log compatibility:
  `carrotMan.trafficState`, `longitudinalPlan.signalDetectionSuspended` / `signalObservationColor` /
  `signalAssistAction`, `EventName.trafficSignGreen` / `trafficSignChanged`,
  `AudibleAlert.trafficSignGreen` / `trafficSignChanged`.
  `longitudinalPlan.trafficState`는 모델 정지 예측 상태(1 정지, 2 출발)로 계속 발행되며 신호등 상태가 아니고
  표시하지 않습니다. `longitudinalPlan.trafficState` is still published as the model-stop prediction state
  only (1 stop, 2 go); it is not a traffic-light state and is not displayed.
- 자원: 과거 K9 로그에서 제거된 신호 작업자 프로세스는 한 코어의 44.1–46.3%, RSS 75–77 MiB를 썼습니다.
  이는 제거된 프로세스의 과거 비용이며, 같은 조건 전후 비교로 측정한 이득이 아닙니다. 서로 다른 경로의
  로그로는 카메라 지연 변화나 이득 부재를 증명할 수 없습니다. GPU 회수량은 증명되지 않았습니다(0으로
  확인된 것도 아님). 카메라 개선은 아직 없습니다.
  Resources: in historical K9 logs the removed signal worker used 44.1–46.3% of one core and 75–77 MiB
  RSS. This is the removed process's historical cost, not a measured paired gain; logs from different
  routes cannot prove that camera latency is unchanged or that there is no benefit. Reclaimed GPU is
  unproven (not confirmed zero). No camera improvement has been made yet.
- 기존 `docs/signal_*.md` 조사 문서는 이력으로만 남기며 현재 동작 설명이 아닙니다.
  Earlier `docs/signal_*.md` investigation documents are historical records only.
