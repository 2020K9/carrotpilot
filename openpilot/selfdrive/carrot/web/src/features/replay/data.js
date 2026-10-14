"use strict";

window.CarrotReplayInsightsData = window.CarrotReplayInsightsData || (() => {
  const ADVANCED_WORKER_URL = "/js/realtime/replay_advanced_worker.js?v=2607-02";
  const POLICY = Object.freeze({
    chunkSize: 240,
    numericSampleIntervalMs: 75,
    maxStateSamples: 6000,
    rawGraphPointLimit: 1200,
    // 그래프 점 사이가 이보다 벌어지면 선을 끊는다. 근거 없는 임시값이다.
    graphGapMs: 300,
  });

  const GROUPS = Object.freeze([
    { key: "control", labelKey: "replay_group_control" },
    { key: "vehicle", labelKey: "replay_group_vehicle" },
    { key: "navigation", labelKey: "replay_group_navigation" },
    { key: "planning", labelKey: "replay_group_planning" },
    { key: "perception", labelKey: "replay_group_perception" },
    { key: "location", labelKey: "replay_group_location" },
    { key: "sensors", labelKey: "replay_group_sensors" },
    { key: "camera", labelKey: "replay_group_camera" },
    { key: "device", labelKey: "replay_group_device" },
    { key: "connectivity", labelKey: "replay_group_connectivity" },
    { key: "diagnostics", labelKey: "replay_group_diagnostics" },
    { key: "raw", labelKey: "replay_group_raw" },
  ]);
  const SERVICE_GROUPS = Object.freeze({
    carState: "vehicle",
    controlsState: "control",
    deviceState: "device",
    peripheralState: "device",
    carrotMan: "navigation",
    selfdriveState: "control",
    gpsLocationExternal: "location",
    longitudinalPlan: "planning",
    modelV2: "perception",
    liveCalibration: "sensors",
    roadCameraState: "camera",
    lateralPlan: "planning",
    radarState: "perception",
    carControl: "control",
    liveDelay: "diagnostics",
    liveTorqueParameters: "sensors",
    liveParameters: "vehicle",
  });

  function scheduleWork(callback) {
    if (typeof window.requestIdleCallback === "function") {
      window.requestIdleCallback(callback, { timeout: 80 });
    } else {
      window.setTimeout(() => callback({ timeRemaining: () => 8 }), 0);
    }
  }

  function normalizedGroup(group) {
    const key = String(group || "");
    return GROUPS.some((entry) => entry.key === key) ? key : "raw";
  }

  function clientRawSource(manifest) {
    const rlog = manifest?.rlog;
    if (!rlog || typeof rlog !== "object") return null;
    const url = String(rlog.url || "");
    const compression = String(rlog.compression || "");
    if (!url || !["zstd", "bzip2", "none"].includes(compression)) return null;
    return {
      url,
      compression,
      expectedSegment: Number(manifest?.segmentIndex) || 0,
      baseMonoTime: String(manifest?.rawFirstMonoTimeNanos || ""),
    };
  }

  function canQueryClientRaw(manifest) {
    return typeof Worker === "function" && Boolean(clientRawSource(manifest));
  }

  function runClientWorker(type, options = {}) {
    const source = clientRawSource(options.manifest);
    if (!source || typeof Worker !== "function") return Promise.resolve(null);
    return new Promise((resolve, reject) => {
      const worker = new Worker(ADVANCED_WORKER_URL);
      const signal = options.signal;
      let settled = false;
      const finish = (callback, value) => {
        if (settled) return;
        settled = true;
        signal?.removeEventListener?.("abort", onAbort);
        worker.terminate();
        callback(value);
      };
      const onAbort = () => {
        try { worker.postMessage({ type: "abort" }); } catch {}
        finish(reject, new DOMException("Aborted", "AbortError"));
      };
      worker.onmessage = (event) => {
        const payload = event?.data || {};
        if (payload.type === "progress") {
          const loaded = Math.max(0, Number(payload.loaded) || 0);
          const total = Math.max(0, Number(payload.total) || 0);
          options.onProgress?.(total > 0 ? Math.min(1, loaded / total) : 0);
        } else if (payload.type === "complete") {
          finish(resolve, payload.result || null);
        } else if (payload.type === "error") {
          finish(reject, new Error(String(payload.error || "Client field processing failed")));
        }
      };
      worker.onerror = (event) => finish(reject, new Error(event?.message || "Client field worker failed"));
      if (signal?.aborted) return onAbort();
      signal?.addEventListener?.("abort", onAbort, { once: true });
      worker.postMessage({
        type,
        ...source,
        service: String(options.service || ""),
        field: options.field || null,
        timeMs: Math.max(0, Number(options.timeMs) || 0),
        maxPoints: Math.max(50, Math.min(3000, Number(options.maxPoints) || POLICY.rawGraphPointLimit)),
      });
    });
  }

  function queryClientTrack(options = {}) {
    if (!options.service || !options.field?.path) return Promise.resolve(null);
    return runClientWorker("query", options);
  }

  function queryClientSnapshot(options = {}) {
    if (!options.service) return Promise.resolve(null);
    return runClientWorker("snapshot", options);
  }

  function catalog(availableServices) {
    const filtered = availableServices instanceof Set || Array.isArray(availableServices);
    const available = availableServices instanceof Set ? availableServices : new Set(availableServices || []);
    const source = window.CarrotVisionCompact?.catalog?.() || [];
    return source
      .filter((entry) => !filtered || available.has(entry.service))
      .map((entry) => ({ ...entry, group: normalizedGroup(entry.group || SERVICE_GROUPS[entry.service]) }));
  }

  function valueAtPath(object, path) {
    let value = object;
    for (const key of String(path || "").split(".")) {
      if (!key || value == null || typeof value !== "object") return undefined;
      value = value[key];
    }
    return value;
  }

  function normalizedValue(value, type) {
    if (typeof value === "number" && Number.isFinite(value)) return { kind: "number", value };
    if (typeof value === "boolean") return { kind: "state", value };
    if (typeof value === "string") return { kind: "state", value };
    if (Array.isArray(value)) {
      const numeric = value.filter((item) => typeof item === "number" && Number.isFinite(item));
      const average = numeric.length ? numeric.reduce((sum, item) => sum + item, 0) / numeric.length : null;
      return {
        kind: "summary",
        value: { type, length: value.length, average },
      };
    }
    if (value && typeof value === "object") {
      return {
        kind: "summary",
        value: { type, keys: Object.keys(value).length },
      };
    }
    return null;
  }

  function valueSignature(value) {
    if (value == null) return "null";
    if (typeof value === "object") return JSON.stringify(value);
    return `${typeof value}:${String(value)}`;
  }

  function buildSummarySeries(options = {}) {
    const records = Array.isArray(options.records) ? options.records : [];
    const decodeRecord = typeof options.decodeRecord === "function" ? options.decodeRecord : () => [];
    const shouldCancel = typeof options.shouldCancel === "function" ? options.shouldCancel : () => false;
    const onProgress = typeof options.onProgress === "function" ? options.onProgress : null;
    const series = { speed: [], steer: [], accel: [], command: [], lead: [] };
    const lastSampleAt = { speed: -Infinity, steer: -Infinity, accel: -Infinity, command: -Infinity, lead: -Infinity };
    let index = 0;

    function push(name, timeMs, value) {
      const numeric = Number(value);
      if (!Number.isFinite(numeric) || timeMs - lastSampleAt[name] < POLICY.numericSampleIntervalMs) return;
      series[name].push({ timeMs, value: numeric });
      lastSampleAt[name] = timeMs;
    }

    return new Promise((resolve) => {
      const step = (deadline) => {
        if (shouldCancel()) return resolve(null);
        const end = Math.min(records.length, index + POLICY.chunkSize);
        const start = index;
        while (index < end && (index - start < 80 || deadline.timeRemaining() > 1)) {
          const record = records[index];
          const timeMs = Number(record?.timeMs || 0);
          try {
            for (const frame of decodeRecord(record)) {
              if (frame?.service === "carState") {
                push("speed", timeMs, Number(frame.decoded?.vEgo) * 3.6);
                push("steer", timeMs, frame.decoded?.steeringAngleDeg);
                push("accel", timeMs, frame.decoded?.aEgo);
              } else if (frame?.service === "carControl") {
                // 빠진 명령 가속은 0으로 채우지 않고 점을 만들지 않는다.
                const commandAccel = frame.decoded?.actuators?.accel;
                if (typeof commandAccel === "number") push("command", timeMs, commandAccel);
              } else if (frame?.service === "radarState" && frame.decoded?.leadOne?.status) {
                push("lead", timeMs, frame.decoded.leadOne.dRel);
              }
            }
          } catch {}
          index += 1;
        }
        onProgress?.(records.length ? index / records.length : 1);
        if (index < records.length) return scheduleWork(step);
        resolve(series);
      };
      scheduleWork(step);
    });
  }

  function buildTrack(options = {}) {
    const records = Array.isArray(options.records) ? options.records : [];
    const decodeRecord = typeof options.decodeRecord === "function" ? options.decodeRecord : () => [];
    const service = String(options.service || "");
    const field = options.field || {};
    const path = String(field.path || "");
    const shouldCancel = typeof options.shouldCancel === "function" ? options.shouldCancel : () => false;
    const onProgress = typeof options.onProgress === "function" ? options.onProgress : null;
    const samples = [];
    let kind = "state";
    let index = 0;
    let lastSampleTimeMs = -Infinity;
    let lastSignature = "";
    let count = 0;
    let sum = 0;
    let min = Infinity;
    let max = -Infinity;

    return new Promise((resolve) => {
      const step = (deadline) => {
        if (shouldCancel()) return resolve(null);
        const end = Math.min(records.length, index + POLICY.chunkSize);
        const start = index;
        while (index < end && (index - start < 80 || deadline.timeRemaining() > 1)) {
          const record = records[index];
          const timeMs = Number(record?.timeMs || 0);
          try {
            for (const frame of decodeRecord(record)) {
              if (frame?.service !== service) continue;
              const normalized = normalizedValue(valueAtPath(frame.decoded, path), field.type);
              if (!normalized) continue;
              kind = normalized.kind;
              const signature = valueSignature(normalized.value);
              const numericReady = kind === "number" && timeMs - lastSampleTimeMs >= POLICY.numericSampleIntervalMs;
              const stateReady = kind !== "number" && signature !== lastSignature;
              if ((numericReady || stateReady) && samples.length < POLICY.maxStateSamples) {
                samples.push({ timeMs, value: normalized.value });
                lastSampleTimeMs = timeMs;
                lastSignature = signature;
              }
              if (kind === "number") {
                count += 1;
                sum += normalized.value;
                min = Math.min(min, normalized.value);
                max = Math.max(max, normalized.value);
              }
            }
          } catch {}
          index += 1;
        }
        onProgress?.(records.length ? index / records.length : 1);
        if (index < records.length) return scheduleWork(step);
        resolve({
          service,
          field,
          kind,
          renderer: String(field.recommendedView || (kind === "number" ? "graph" : "state-band")),
          statisticsPolicy: String(field.statisticsPolicy || ""),
          samples,
          stats: kind === "number" && count
            ? { count, min, max, average: sum / count }
            : { count: samples.length },
        });
      };
      scheduleWork(step);
    });
  }

  function finiteNumber(value) {
    return typeof value === "number" && Number.isFinite(value);
  }

  // 저크는 걸러내기 없이 그래프에 찍는 실제 가속 점끼리의 차이를 시간으로 나눈 값이며,
  // 뒤쪽 점 시각에 둔다. startMs 는 계산에 쓴 앞쪽 점 시각이다.
  function jerkSeries(points, maxGapMs = POLICY.graphGapMs) {
    const list = Array.isArray(points) ? points : [];
    const result = [];
    for (let index = 1; index < list.length; index += 1) {
      const previous = list[index - 1];
      const current = list[index];
      if (!finiteNumber(previous?.timeMs) || !finiteNumber(current?.timeMs)) continue;
      if (!finiteNumber(previous.value) || !finiteNumber(current.value)) continue;
      const dtMs = current.timeMs - previous.timeMs;
      if (dtMs <= 0 || dtMs > maxGapMs) continue;
      result.push({
        timeMs: current.timeMs,
        startMs: previous.timeMs,
        value: (current.value - previous.value) / (dtMs / 1000),
      });
    }
    return result;
  }

  // 시각이 이웃한 두 점 사이에 있고 그 간격이 maxGapMs 이하일 때만 가까운 점을 돌려준다.
  function gapAwareSample(samples, timeMs, maxGapMs = POLICY.graphGapMs) {
    const list = Array.isArray(samples) ? samples : [];
    if (!list.length || !finiteNumber(timeMs)) return null;
    let low = 0;
    let high = list.length;
    while (low < high) {
      const middle = Math.floor((low + high) / 2);
      if (list[middle].timeMs < timeMs) low = middle + 1;
      else high = middle;
    }
    const next = list[low];
    if (next?.timeMs === timeMs) return next;
    const previous = list[low - 1];
    if (!previous || !next || next.timeMs - previous.timeMs > maxGapMs) return null;
    return timeMs - previous.timeMs <= next.timeMs - timeMs ? previous : next;
  }

  // 저크 점은 [startMs, timeMs] 구간의 값이다. 그 구간 밖(빈 구간)이면 null 이다.
  function spanningSample(samples, timeMs) {
    const list = Array.isArray(samples) ? samples : [];
    if (!list.length || !finiteNumber(timeMs)) return null;
    let low = 0;
    let high = list.length;
    while (low < high) {
      const middle = Math.floor((low + high) / 2);
      if (list[middle].timeMs < timeMs) low = middle + 1;
      else high = middle;
    }
    const sample = list[low];
    return sample && sample.startMs <= timeMs ? sample : null;
  }

  function symmetricRange(samples) {
    let span = 1;
    for (const sample of samples) if (finiteNumber(sample?.value)) span = Math.max(span, Math.abs(sample.value));
    return { min: -span, max: span };
  }

  function graphCoordinate(sample, durationMs, range, height) {
    const x = Math.max(0, Math.min(600, sample.timeMs / durationMs * 600));
    const y = height - 5 - ((sample.value - range.min) / (range.max - range.min)) * (height - 10);
    return `${x.toFixed(2)},${y.toFixed(2)}`;
  }

  // isContinuous(앞 점, 현재 점)이 거짓이면 새 조각을 시작해 빈 구간을 가로질러 잇지 않는다.
  function brokenPath(samples, durationMs, range, height, isContinuous) {
    let path = "";
    samples.forEach((sample, index) => {
      const command = index > 0 && isContinuous(samples[index - 1], sample) ? "L" : "M";
      path += `${path ? " " : ""}${command}${graphCoordinate(sample, durationMs, range, height)}`;
    });
    return path;
  }

  // 가속도 그래프 묶음: 실제·명령 가속은 두 값 범위를 합친 같은 세로축을 쓰고,
  // 저크는 최종 실제 가속 그래프 점 배열에서 계산한다.
  function accelerationGraph(options = {}) {
    const actual = Array.isArray(options.actual) ? options.actual : [];
    const command = Array.isArray(options.command) ? options.command : [];
    const durationMs = Number(options.durationSeconds) * 1000;
    const height = Number(options.height) || 90;
    const maxGapMs = POLICY.graphGapMs;
    const range = symmetricRange([...actual, ...command]);
    const jerk = jerkSeries(actual, maxGapMs);
    const jerkRange = symmetricRange(jerk);
    const drawable = durationMs > 0;
    return {
      range,
      jerk,
      jerkRange,
      actualPoints: drawable
        ? actual.map((sample) => graphCoordinate(sample, durationMs, range, height)).join(" ")
        : "",
      commandPath: drawable
        ? brokenPath(command, durationMs, range, height, (previous, current) => (
          current.timeMs - previous.timeMs <= maxGapMs
        ))
        : "",
      jerkPath: drawable
        ? brokenPath(jerk, durationMs, jerkRange, height, (previous, current) => current.startMs === previous.timeMs)
        : "",
    };
  }

  function sampleAt(samples, timeMs) {
    if (!samples.length) return null;
    let low = 0;
    let high = samples.length - 1;
    while (low < high) {
      const middle = Math.ceil((low + high) / 2);
      if (samples[middle].timeMs <= timeMs) low = middle;
      else high = middle - 1;
    }
    return samples[low];
  }

  return {
    POLICY,
    GROUPS,
    catalog,
    canQueryClientRaw,
    queryClientTrack,
    queryClientSnapshot,
    buildSummarySeries,
    buildTrack,
    sampleAt,
    jerkSeries,
    gapAwareSample,
    spanningSample,
    accelerationGraph,
  };
})();
