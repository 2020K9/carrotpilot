# 신호등 기능 제거 / Traffic-light feature removal (2026-10-20)

- 카메라 기반 보조 신호등 인식·표시·제어(신호보조 모드, 앞차 브레이크등 관찰, 신호 관찰기/추적기,
  오프라인 분석 도구 `tools/signal_analysis/`)를 제거했습니다. 설정 `SignalAssistMode`,
  `LeadBrakeLights`와 메모리 키 `TrafficLight`는 삭제되었습니다.
  Removed camera-based auxiliary traffic-light recognition, display and control (Signal
  Assistance Mode, lead brake-lamp observation, signal observer/tracker processes and the
  offline `tools/signal_analysis/` tools). Params `SignalAssistMode`, `LeadBrakeLights` and the
  memory key `TrafficLight` are deleted.
- `TrafficLightDetectMode` 키는 저장 설정 호환을 위해 유지하되 의미가 바뀌었습니다: 앞차가 없을 때
  주행 모델 자체의 정지 예측으로 감속·정지하는 기능(“모델 정지 예측 감속”)이며 `0` 끔, `1` 켬(기본).
  신호등 감지가 아니며(정지선·표지·장애물 등으로도 정지 예측 가능), 자동 녹색 출발(예전 `2`)은
  제거되어 저장된 `2`는 `1`로 처리합니다. 정지 후 출발은 가속 페달(또는 일반 앞차 추종)로 합니다.
  `TrafficLightDetectMode` is retained for saved-settings compatibility with a new meaning
  ("Model Stop Prediction"): `0` off, `1` on (default) slows/stops on the driving model's own
  stop prediction with no lead. It is not a traffic-light detector; automatic green departure
  (old `2`) is removed and a saved `2` is treated as `1`. Departure needs the accelerator (or a
  lead moving away under normal lead following). `TrafficStopDistanceAdjust` remains as the
  generic model-stop position offset.
- 로그 호환을 위해 다음 Cap'n Proto 필드/열거값은 삭제하지 않고 비활성(deprecated, inert)으로 남깁니다.
  Kept as deprecated, inert schema items for log compatibility:
  `carrotMan.trafficState`, `longitudinalPlan.signalDetectionSuspended` /
  `signalObservationColor` / `signalAssistAction`, `EventName.trafficSignGreen` /
  `trafficSignChanged`, `AudibleAlert.trafficSignGreen` / `trafficSignChanged`.
  `longitudinalPlan.trafficState`는 계속 발행되지만 모델 정지 예측 상태(1 정지 예측, 2 출발 예측)일 뿐
  신호등 상태가 아니며 표시하지 않습니다. `longitudinalPlan.trafficState` is still published as the
  model-stop prediction state only (1 stop, 2 go), not a traffic-light state, and is not displayed.
- 기존 `docs/signal_*.md` 조사 문서는 이력으로만 남기며 현재 동작 설명이 아닙니다.
  Earlier `docs/signal_*.md` investigation documents are historical records only.
