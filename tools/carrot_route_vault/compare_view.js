// Drive comparison page: recorded values and definitions only; no judgement of driving quality.
// Pure helpers are exported for `node --test`; the DOM is wired only when `document` exists.

// Unapproved defaults.
export const SYNC_TOLERANCE_S = 0.15;
export const BUFFER_RELEASE_MS = 5000;
export const POLL_DEADLINE_MS = 360000;
export const RATES = [0.25, 0.5, 1, 2];
export const READY_ENOUGH = 3;  // HTMLMediaElement.HAVE_FUTURE_DATA

export function parseSafePath(value, origin) {
  if (typeof value !== 'string' || !value) return null;
  if (/[\u0000-\u001f\u007f]/.test(value)) return null;
  if (!value.startsWith('/routes/') && !value.startsWith('/s/')) return null;
  if (value.includes('//') || value.includes('\\') || /(\.|%2e){2}/i.test(value) || /%2f|%5c|[?#]/i.test(value)) return null;
  let url, base;
  try { base = new URL(origin); url = new URL(value, base.origin); } catch { return null; }
  if (url.origin !== base.origin) return null;
  return url.pathname.replace(/\/$/, '');
}

export function parseIndex(value) {
  if (typeof value !== 'string' || !/^\d{1,7}$/.test(value)) return null;
  const index = Number(value);
  return index <= 1000000 ? index : null;
}

export function retryAfterMs(header) {
  const seconds = typeof header === 'string' && /^\d+$/.test(header.trim()) ? Number(header.trim()) : 2;
  return Math.max(1, Math.min(30, seconds)) * 1000;
}

const finite = v => typeof v === 'number' && Number.isFinite(v);

// Log time L (s) -> video time (s) of one side, or null where the cell has no video anchor.
export function videoAt(t, v, L) {
  if (!Array.isArray(t) || !t.length || !finite(L) || L > t[t.length - 1] + 0.1) return null;
  let lo = 0, hi = t.length;
  while (lo < hi) { const mid = (lo + hi) >> 1; if (t[mid] <= L) lo = mid + 1; else hi = mid; }
  const k = lo - 1;
  if (k < 0 || !finite(v[k])) return null;
  return v[k] + (L - t[k]);
}

export function timelineRange(sides) {
  const usable = sides.filter(s => s && Array.isArray(s.t) && s.t.length);
  if (!usable.length) return [0, 0];
  return [Math.min(...usable.map(s => -s.base)), Math.max(...usable.map(s => s.t[s.t.length - 1] - s.base))];
}

// Stateless judgement for one side at one moment.
export function syncDecision({target, currentTime, duration, hasVideo, error}) {
  if (!hasVideo || error) return {action: 'pause', reason: 'no-video'};
  if (target === null || target === undefined || !finite(target)) return {action: 'pause', reason: 'no-target'};
  if (target < 0 || (finite(duration) && target > duration)) return {action: 'pause', reason: 'out-of-range'};
  return Math.abs(currentTime - target) > SYNC_TOLERANCE_S ? {action: 'seek', reason: 'drift'} : {action: 'keep', reason: 'in-sync'};
}

export const INITIAL_SYNC = Object.freeze({mode: 'released', reason: 'initial', bufferingSince: null, bufferReleased: false});

// Stateful step: remembers released/buffering so a returning target resumes playback and a
// buffering side that exceeded BUFFER_RELEASE_MS stays released instead of holding the clock.
export function syncStep(prev, input, nowMs) {
  const state = prev || INITIAL_SYNC;
  const decision = syncDecision(input);
  if (decision.action === 'pause') {
    return {state: {mode: 'released', reason: decision.reason, bufferingSince: null, bufferReleased: false}, action: 'pause', reason: decision.reason};
  }
  const synced = {mode: 'synced', reason: null, bufferingSince: null, bufferReleased: false};
  const drift = decision.action === 'seek';
  const buffering = !(input.readyState >= READY_ENOUGH);
  if (state.bufferReleased) {
    if (!buffering) return {state: synced, action: 'seek-play', reason: 'buffer-recovered'};
    return {state, action: 'none', reason: 'buffer-timeout'};
  }
  if (buffering) {
    const since = state.bufferingSince ?? nowMs;
    if (nowMs - since > BUFFER_RELEASE_MS) {
      return {state: {mode: 'released', reason: 'buffer-timeout', bufferingSince: null, bufferReleased: true}, action: 'none', reason: 'buffer-timeout'};
    }
    return {state: {...synced, reason: 'buffering', bufferingSince: since}, action: drift ? 'seek' : 'keep', reason: 'buffering'};
  }
  if (state.mode === 'released') return {state: synced, action: drift ? 'seek-play' : 'play', reason: 'resume'};
  return {state: synced, action: decision.action, reason: decision.reason};
}

export function clockShouldHold(states) {
  return states.some(s => s && s.mode === 'synced' && s.bufferingSince !== null && !s.bufferReleased);
}

export function defaultEpisodes(a, b) {
  const listA = a?.episodes || [], listB = b?.episodes || [];
  if (!b) return listA.length ? [0, null] : null;
  for (let i = 0; i < listA.length; i++) {
    const j = listB.findIndex(e => e.type === listA[i].type);
    if (j >= 0) return [i, j];
  }
  return null;
}

export function deltaValue(a, b) {
  return finite(a) && finite(b) ? b - a : null;
}

export function endLeadLabel(type) {
  return type === 'stop_decel' || type === 'creep_stop' ? '정차 시점 선행차 거리' : '장면 끝 선행차 거리';
}

export const REASON_TEXT = {
  'no-video': '영상 없음/재생 실패: 그래프만 동기',
  'no-target': '영상 시각 없음 구간',
  'out-of-range': '영상 범위 밖',
  'buffer-timeout': '영상 지연: 동기 해제',
  'buffering': '영상 버퍼링',
  'initial': '',
};

const TYPE_TEXT = {stop_decel: '감속 정지', creep_stop: '저속 정지', start_accel: '발진', cruise_decel: '주행 중 감속'};
const STAT_KEYS = ['type', 'start_t', 'end_t', 'start_video_t', 'end_video_t', 'duration', 'start_v', 'end_v', 'min_v', 'max_v',
  'delta_v', 'min_a', 'max_a', 'avg_a', 'min_cmd_a', 'max_cmd_a', 'avg_cmd_a', 'max_jerk', 'std_jerk', 'cmd_actual_corr',
  'start_lead_d', 'min_lead_d', 'end_lead_d', 'lead_track_ids', 'states', 'long_active_ratio', 'pedal_input',
  'start_clipped', 'end_clipped'];

export function formatStat(value) {
  if (value === null || value === undefined) return '—';
  if (typeof value === 'number') return finite(value) ? value.toFixed(3) : '—';
  if (typeof value === 'boolean') return value ? '예' : '아니오';
  if (Array.isArray(value)) return value.join(', ');
  if (typeof value === 'object') return Object.entries(value).map(([k, n]) => `${k}:${n}`).join(', ');
  return String(value);
}

// Arrays the page reads; each must match series.t in length.
const DRIVE_SERIES = ['t', 'video_time_s', 'vEgo', 'aEgo', 'actAccel', 'leadDist', 'jerk'];
export const GRAPH_SERIES = ['vEgo', 'aEgo', 'actAccel', 'leadDist', 'jerk'];
const EPISODE_NUMBERS = ['start_t', 'end_t', 'start_video_t', 'end_video_t', 'duration', 'start_v', 'end_v', 'min_v', 'max_v',
  'delta_v', 'min_a', 'max_a', 'avg_a', 'min_cmd_a', 'max_cmd_a', 'avg_cmd_a', 'max_jerk', 'std_jerk', 'cmd_actual_corr',
  'start_lead_d', 'min_lead_d', 'end_lead_d', 'long_active_ratio'];
const EPISODE_FLAGS = ['pedal_input', 'start_clipped', 'end_clipped'];
const isObject = v => v !== null && typeof v === 'object' && !Array.isArray(v);
const isCount = v => Number.isInteger(v) && v >= 0;
const isStrings = v => Array.isArray(v) && v.every(x => typeof x === 'string');

// Checks every field the page displays before anything is published.
export function validateDrive(data) {
  const fail = reason => ({ok: false, reason});
  if (!isObject(data)) return fail('data');
  if (data.schemaVersion !== 1 || data.kind !== 'carrotDrive') return fail('schema');
  const series = data.series;
  if (!isObject(series)) return fail('series');
  const t = series.t;
  if (!Array.isArray(t) || !t.length) return fail('series.t');
  for (let k = 0; k < t.length; k++) if (!finite(t[k]) || (k && t[k] < t[k - 1])) return fail('series.t');
  for (const name of DRIVE_SERIES) {
    const values = series[name];
    if (!Array.isArray(values) || values.length !== t.length || !values.every(v => v === null || finite(v))) return fail(`series.${name}`);
  }
  if (!isStrings(data.missingServices)) return fail('missingServices');
  if (!isStrings(data.ambiguousDefaultFields)) return fail('ambiguousDefaultFields');
  if (!isObject(data.invalidCounts) || !Object.values(data.invalidCounts).every(isCount)) return fail('invalidCounts');
  if (typeof data.videoAligned !== 'boolean') return fail('videoAligned');
  if (typeof data.sourceVersion !== 'string' || typeof data.sourceLog !== 'string') return fail('source');
  if (!Array.isArray(data.episodes)) return fail('episodes');
  for (const [i, e] of data.episodes.entries()) {
    const at = `episodes[${i}]`;
    if (!isObject(e) || typeof e.type !== 'string') return fail(at);
    for (const key of EPISODE_NUMBERS) if (e[key] !== null && !finite(e[key])) return fail(`${at}.${key}`);
    if (!finite(e.start_t) || !finite(e.end_t) || e.start_t < t[0] || e.end_t > t[t.length - 1] || e.start_t > e.end_t) return fail(`${at}.range`);
    for (const key of EPISODE_FLAGS) if (e[key] !== undefined && typeof e[key] !== 'boolean') return fail(`${at}.${key}`);
    if (e.lead_track_ids != null && !(Array.isArray(e.lead_track_ids) && e.lead_track_ids.every(finite))) return fail(`${at}.lead_track_ids`);
    if (e.states != null && !(isObject(e.states) && Object.values(e.states).every(isCount))) return fail(`${at}.states`);
  }
  return {ok: true, data};
}

// Non-zero invalidCounts per service (server definition) and null cells per graph series.
export function missingSummary(data) {
  return {
    services: Object.entries(data.invalidCounts).filter(([, n]) => n > 0),
    series: GRAPH_SERIES.map(name => [name, data.series[name].filter(v => v === null).length]).filter(([, n]) => n > 0),
  };
}

export function noteLines(data) {
  const nullCells = data.series.vEgo.filter(x => x === null).length;
  const {services, series} = missingSummary(data);
  return [
    `코드 ${data.sourceVersion} · ${data.sourceLog} · ${data.series.t.length}칸(10Hz, 로그 시각)${data.videoAligned ? '' : ' · 영상 시각 없음: 추정 동기 안 함'}`,
    ...(nullCells ? [`carState 무효/결측 칸 ${nullCells}개(값 없음 처리)`] : []),
    ...(services.length ? [`서비스별 무효/결측 칸(invalidCounts: valid=false 칸): ${services.map(([name, n]) => `${name} ${n}`).join(', ')}`] : []),
    ...(series.length ? [`그래프 값 없음 칸: ${series.map(([name, n]) => `${name} ${n}${name === 'leadDist' ? '(선행차 없음 포함)' : ''}`).join(', ')}`] : []),
    ...(data.missingServices.length ? [`기록 없는 서비스: ${data.missingServices.join(', ')}`] : []),
    `0은 기록값 또는 기본값일 수 있음: ${data.ambiguousDefaultFields.join(', ')}`,
  ];
}

export function episodeOptions(data) {
  const v = x => finite(x) ? (x * 3.6).toFixed(1) : '—';
  return [{value: '', text: '장면 선택 안 함'}, ...data.episodes.map((e, i) => ({value: String(i),
    text: `${TYPE_TEXT[e.type] || e.type} · t ${e.start_t.toFixed(1)}s · ${v(e.start_v)}→${v(e.end_v)} km/h${e.start_clipped || e.end_clipped ? ' · 잘림' : ''}`}))];
}

export function cardItems(e) {
  if (!e) return [];
  const card = (label, value, unit = '') => ({label, text: finite(value) ? `${value.toFixed(3)}${unit}` : '—'});
  return [
    card('상관도 cmd_actual_corr (시간 지연 미보정)', e.cmd_actual_corr),
    card(endLeadLabel(e.type), e.end_lead_d, ' m'),
    card('저크 표준편차: aEgo 차분(10Hz)', e.std_jerk, ' m/s³'),
    card('최대 저크: aEgo 차분(10Hz)', e.max_jerk, ' m/s³'),
    card('최저 가속도', e.min_a, ' m/s²'),
    card('길이', e.duration, ' s'),
    ...(e.pedal_input ? [{label: 'gasPressed 또는 brakePressed', text: '운전자 페달 입력 있음'}] : []),
  ];
}

export function statTable(keys, episodes) {
  if (!episodes.some(Boolean)) return null;
  const pair = keys.length > 1;
  return {
    head: ['항목', ...keys.map(k => k.toUpperCase()), ...(pair ? ['Δ(B−A)'] : [])],
    rows: STAT_KEYS.map(key => {
      const values = episodes.map(e => e ? e[key] : null);
      return [key, ...values.map(formatStat), ...(pair ? [formatStat(deltaValue(values[0], values[1]))] : [])];
    }),
  };
}

// While the common clock holds for a buffering video, every video waits so they resume together.
export function playIntent({running, hold, state}) {
  return running && !hold && state?.mode === 'synced' ? 'play' : 'pause';
}

class LoadError extends Error {}

function abortError() {
  const error = new Error('aborted');
  error.name = 'AbortError';
  return error;
}

// Loading, clock and video sync without the DOM. deps: fetch, now, setTimeout, clearTimeout,
// AbortController, requestAnimationFrame. The controller alone schedules frames: one pending frame at most.
// view callbacks (all optional): title, status, retry, notes, episodes, selected, align, layout, frame.
export function createCompareController({sides: specs, deps, view = {}, align: initialAlign = 'episode'}) {
  const sides = specs.map(spec => ({
    key: spec.key, path: spec.path, index: spec.index, invalid: Boolean(spec.invalid), video: spec.video,
    data: null, episode: null, userPicked: false, base: 0, sync: INITIAL_SYNC, videoUrl: '', mediaError: false,
    generation: 0, abort: null, timer: null, finished: false, failed: false, expired: false,
    status: '', statusError: false, retryVisible: false,
  }));
  const clock = {tau: 0, running: false, last: null, rate: 1, range: [0, 0]};
  let align = initialAlign === 'start' ? 'start' : 'episode';
  let alignByUser = false, defaultsChosen = false, frame = null, started = false;
  const show = (name, ...args) => { if (typeof view[name] === 'function') view[name](...args); };
  const sideOf = ref => typeof ref === 'number' ? sides[ref] : sides.find(s => s.key === ref);
  const selected = side => side.data && side.episode !== null ? side.data.episodes[side.episode] ?? null : null;

  function setStatus(side, text, error = false) { side.status = text; side.statusError = error; show('status', side.key, text, error); }
  function setRetry(side, visible) { side.retryVisible = visible; show('retry', side.key, visible); }
  function pickEpisode(side, value) { side.episode = value; show('selected', side.key, value === null ? '' : String(value)); }

  // reset: a person changed alignment or episode, so restart at the beginning; otherwise only clamp τ.
  function relayout(reset) {
    for (const side of sides) {
      const episode = selected(side);
      side.base = align === 'episode' && episode ? episode.start_t : 0;
    }
    clock.range = timelineRange(sides.filter(s => s.data).map(s => ({t: s.data.series.t, base: s.base})));
    const [lo, hi] = clock.range;
    if (reset) clock.tau = align === 'episode' ? 0 : lo;
    clock.tau = Math.min(Math.max(clock.tau, lo), hi);
    if (hi <= lo) clock.running = false;
    show('layout', {range: [lo, hi], cards: sides.map(s => ({key: s.key, items: cardItems(selected(s))})),
                    table: statTable(sides.map(s => s.key), sides.map(selected))});
  }

  function sameType(side, episode) {
    const i = episode ? side.data.episodes.findIndex(e => e.type === episode.type) : -1;
    return i >= 0 ? i : null;
  }

  // Once every side has finished; never overrides a person's episode or alignment choice.
  function chooseDefaults() {
    defaultsChosen = true;
    const picked = sides.find(s => s.userPicked);
    if (picked) {
      for (const side of sides) if (side.data && !side.userPicked) pickEpisode(side, sameType(side, selected(picked)));
    } else {
      const [a, b] = sides;
      const pick = defaultEpisodes(a.data, b ? b.data : undefined);
      if (pick) { pickEpisode(a, pick[0]); if (b) pickEpisode(b, pick[1]); }
      else if (!alignByUser) { align = 'start'; show('align', align); }
    }
  }

  function settle(side) {
    side.finished = true;
    if (!defaultsChosen) {
      if (sides.every(s => s.finished)) chooseDefaults();
    } else if (side.data) {
      const other = sides.find(s => s !== side && selected(s));
      if (other) pickEpisode(side, sameType(side, selected(other)));
      else pickEpisode(side, sides.length === 1 && side.data.episodes.length ? 0 : null);
    }
    relayout(false);
  }

  function clearSide(side) {
    side.data = null; side.episode = null; side.userPicked = false; side.sync = INITIAL_SYNC; side.mediaError = false;
    side.video.pause(); side.video.removeAttribute('src');
    if (typeof side.video.load === 'function') side.video.load();
    show('episodes', side.key, []); show('notes', side.key, []);
    relayout(false);
  }

  function sleep(ms, signal) {
    return new Promise((resolve, reject) => {
      if (signal.aborted) { reject(abortError()); return; }
      const onAbort = () => { deps.clearTimeout(id); reject(abortError()); };
      const id = deps.setTimeout(() => { signal.removeEventListener('abort', onAbort); resolve(); }, Math.max(0, ms));
      signal.addEventListener('abort', onAbort, {once: true});
    });
  }

  // One absolute deadline per load covers the manifest, every poll, every body read and every wait.
  async function load(ref) {
    const side = sideOf(ref);
    const generation = ++side.generation;
    const current = () => generation === side.generation;
    if (side.abort) side.abort.abort();
    if (side.timer !== null) deps.clearTimeout(side.timer);
    side.abort = side.timer = null;
    side.finished = side.failed = side.expired = false;
    setRetry(side, false);
    if (side.invalid) {
      clearSide(side);
      side.failed = true;
      setStatus(side, '잘못된 비교 주소입니다(경로 또는 index).', true);
      settle(side);
      return;
    }
    const controller = new deps.AbortController(), signal = controller.signal;
    const deadline = deps.now() + POLL_DEADLINE_MS;
    let timedOut = false;
    side.abort = controller;
    side.timer = deps.setTimeout(() => { timedOut = true; controller.abort(); }, POLL_DEADLINE_MS);
    try {
      setStatus(side, '세그먼트 정보를 불러오는 중…');
      const manifest = await deps.fetch(`${side.path}/manifest`, {cache: 'no-store', signal});
      if (!current()) return;
      if (!manifest.ok) throw new LoadError(manifest.status === 404 ? '로그를 찾을 수 없습니다.' : `세그먼트 정보를 불러오지 못했습니다 (${manifest.status}).`);
      const info = await manifest.json();
      if (!current()) return;
      const segment = (Array.isArray(info?.segments) ? info.segments : []).find(s => Number(s?.index) === side.index);
      if (!segment) throw new LoadError('선택한 세그먼트가 이 링크 범위에 없습니다.');
      show('title', side.key, `${side.key.toUpperCase()} · ${typeof segment.name === 'string' ? segment.name : ''}`);
      side.videoUrl = typeof segment.videoUrl === 'string' ? segment.videoUrl : '';
      for (;;) {
        if (deps.now() >= deadline) { timedOut = true; throw abortError(); }
        setStatus(side, '주행 분석을 준비하는 중…');
        const response = await deps.fetch(`${side.path}/drive/${side.index}`, {cache: 'no-store', signal});
        if (!current()) return;
        if (response.status === 202 || response.status === 503) {
          await sleep(Math.min(retryAfterMs(response.headers.get('Retry-After')), deadline - deps.now()), signal);
          if (!current()) return;
          continue;
        }
        if (!response.ok) throw new LoadError(response.status === 404 ? '주행 분석용 rlog가 없습니다.' : response.status === 422 ? '주행 분석에 실패했습니다.' : `주행 분석을 불러오지 못했습니다 (${response.status}).`);
        const data = await response.json();  // aborted through the request's signal
        if (!current()) return;
        const checked = validateDrive(data);
        if (!checked.ok) throw new LoadError(`주행 분석 형식이 올바르지 않습니다 (${checked.reason}).`);
        const notes = noteLines(data), options = episodeOptions(data);
        side.data = data; side.episode = null; side.userPicked = false; side.sync = INITIAL_SYNC;
        if (side.videoUrl && data.videoAligned) { side.mediaError = false; side.video.src = side.videoUrl; }
        else { side.video.pause(); side.video.removeAttribute('src'); }
        show('notes', side.key, notes); show('episodes', side.key, options);
        setStatus(side, '');
        settle(side);
        return;
      }
    } catch (error) {
      if (!current()) return;
      clearSide(side);
      if (timedOut) {
        side.expired = true;
        setStatus(side, '분석이 아직 끝나지 않았습니다.');
        setRetry(side, true);
      } else {
        side.failed = true;
        setStatus(side, error instanceof LoadError ? error.message : `네트워크 오류로 불러오지 못했습니다: ${error?.message || error}`, true);
      }
      settle(side);
    } finally {
      if (current()) { deps.clearTimeout(side.timer); side.timer = side.abort = null; }
    }
  }

  function failFrame(side, error) {
    try {
      clearSide(side);
    } finally {
      side.failed = true;
      setStatus(side, `화면 처리 중 오류로 이 쪽 표시를 멈췄습니다: ${error?.message || error}`, true);
    }
  }

  function stepSide(side, now) {
    const video = side.video, hasVideo = Boolean(side.data?.videoAligned && side.videoUrl && video.getAttribute('src'));
    const target = side.data ? videoAt(side.data.series.t, side.data.series.video_time_s, side.base + clock.tau) : null;
    const step = syncStep(side.sync, {target, currentTime: video.currentTime, duration: video.duration, hasVideo,
                                      error: side.mediaError || Boolean(video.error), readyState: video.readyState}, now);
    side.sync = step.state;
    return {action: step.action, target};
  }

  function applySide(side, step, hold) {
    const video = side.video;
    if (step.action === 'seek' || step.action === 'seek-play') video.currentTime = step.target;
    if (playIntent({running: clock.running, hold, state: side.sync}) === 'pause') {
      if (!video.paused) video.pause();
    } else if (video.paused) {
      const playing = video.play();
      if (playing && typeof playing.catch === 'function') playing.catch(() => { side.mediaError = true; });
    }
  }

  function schedule() {
    if (frame === null) frame = deps.requestAnimationFrame(now => { frame = null; tick(now); });
  }

  // A failing side is cleared and reported; the other side, the clock and the next frame continue.
  function tick(now) {
    try {
      const elapsed = clock.last === null ? 0 : now - clock.last;
      clock.last = now;
      if (clock.running && !clockShouldHold(sides.map(s => s.sync))) {
        clock.tau += elapsed / 1000 * clock.rate;
        if (clock.tau >= clock.range[1]) { clock.tau = clock.range[1]; clock.running = false; }
      }
      const steps = sides.map(side => {
        try { return stepSide(side, now); } catch (error) { failFrame(side, error); return null; }
      });
      const hold = clock.running && clockShouldHold(sides.map(s => s.sync));
      sides.forEach((side, i) => {
        if (!steps[i]) return;
        try { applySide(side, steps[i], hold); } catch (error) { failFrame(side, error); }
      });
      show('frame', snapshot());
    } finally {
      schedule();
    }
  }

  function snapshot() {
    return {
      clock: {...clock, range: [...clock.range]}, align, defaultsChosen,
      sides: sides.map(s => ({key: s.key, data: s.data, episode: s.episode, userPicked: s.userPicked, base: s.base, sync: s.sync,
                              status: s.status, statusError: s.statusError, retryVisible: s.retryVisible,
                              finished: s.finished, failed: s.failed, expired: s.expired})),
    };
  }

  return {
    start() {
      if (started) return;
      started = true;
      for (const side of sides) load(side.key);
      schedule();
    },
    load, tick,
    retry: ref => load(ref),
    play() {
      if (clock.tau >= clock.range[1]) clock.tau = clock.range[0];
      clock.running = clock.range[1] > clock.range[0];
    },
    pause() { clock.running = false; },
    seek(tau) { if (finite(tau)) clock.tau = Math.min(Math.max(tau, clock.range[0]), clock.range[1]); },
    setRate(rate) { clock.rate = rate; for (const side of sides) side.video.playbackRate = rate; },
    setAlign(value) { align = value === 'start' ? 'start' : 'episode'; alignByUser = true; relayout(true); },
    setEpisode(ref, value) {
      const side = sideOf(ref);
      if (!side.data) return;
      const i = value === null || value === '' ? null : Number(value);
      side.episode = Number.isInteger(i) && i >= 0 && i < side.data.episodes.length ? i : null;
      side.userPicked = true;
      relayout(true);
    },
    markMediaError(ref) { sideOf(ref).mediaError = true; },
    state: snapshot,
  };
}

function el(tag, attrs = {}, ...children) {
  const node = document.createElement(tag);
  for (const [key, value] of Object.entries(attrs)) {
    if (key === 'class') node.className = value; else if (key === 'text') node.textContent = value; else node.setAttribute(key, value);
  }
  for (const child of children) node.append(child);
  return node;
}

const STYLE = `
.cmp-grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:16px;margin:16px 0}
.cmp-side h2{margin:0 0 6px}.cmp-side video{width:100%;background:#090e14;border-radius:12px;aspect-ratio:16/9}
.cmp-status{min-height:1.5em;font-size:13px;color:var(--muted);overflow-wrap:anywhere}.cmp-error{color:var(--red)}
.cmp-transport{display:flex;align-items:center;gap:12px;padding:12px 16px;margin:16px 0;flex-wrap:wrap}
.cmp-transport input[type=range]{flex:1;min-width:160px;padding:0;border:0;box-shadow:none;accent-color:var(--orange)}
.cmp-transport select,.cmp-side select{width:auto;padding:6px}.cmp-time{font-variant-numeric:tabular-nums;font-size:12px}
.cmp-graph{display:block;width:100%;height:190px;background:#090e14;border-radius:8px;margin-top:8px}
.cmp-cards{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px;margin-top:10px}
.cmp-card{padding:12px}.cmp-card strong{display:block;font-size:18px;font-variant-numeric:tabular-nums}.cmp-card span{font-size:11px;color:var(--muted)}
.cmp-table{width:100%;border-collapse:collapse;font-size:13px;font-variant-numeric:tabular-nums}
.cmp-table th,.cmp-table td{border-bottom:1px solid var(--line);padding:6px 8px;text-align:right}.cmp-table th:first-child,.cmp-table td:first-child{text-align:left}
.cmp-note{font-size:12px;color:var(--muted);margin:4px 0}.cmp-sync{font-size:12px;color:var(--orange2);min-height:1.4em}
`;

const COLORS = {a: '#ff8a36', b: '#56baff'};


// Thin DOM layer: builds elements and hands browser services to the controller.
function init() {
  document.head.append(el('style', {text: STYLE}));
  const root = document.querySelector('#compare');
  const params = new URLSearchParams(location.search);
  const specs = [];
  for (const key of ['a', 'b']) {
    const raw = params.get(key), rawIndex = params.get(key + 'i');
    if (key === 'b' && raw === null) continue;
    const path = parseSafePath(raw, location.origin), index = parseIndex(rawIndex ?? '');
    specs.push({key, path, index, invalid: path === null || index === null});
  }

  root.append(
    el('p', {class: 'cmp-note', text: '이 주소는 공유 링크와 같은 접근 권한을 가집니다. 전달할 때 주의하세요.'}),
    el('p', {class: 'cmp-note', text: '영상 시각: modelV2 기준·동기 오차 미검증. 장면 기준: 자동 기준(미승인 기본값). 숫자와 정의만 표시하며 주행 품질을 판정하지 않습니다.'}),
  );
  const alignSelect = el('select', {'aria-label': '정렬 기준'}, el('option', {value: 'episode', text: '장면 시작 정렬'}), el('option', {value: 'start', text: '로그 시작 정렬'}));
  alignSelect.value = params.get('align') === 'start' ? 'start' : 'episode';
  const playButton = el('button', {type: 'button', text: '재생'});
  const scrub = el('input', {type: 'range', min: '0', max: '0', step: '0.01', value: '0', 'aria-label': '공통 재생 위치'});
  const rateSelect = el('select', {'aria-label': '배속'}, ...RATES.map(r => el('option', {value: String(r), text: `${r}×`})));
  rateSelect.value = '1';
  const timeLabel = el('span', {class: 'cmp-time'});
  const grid = el('div', {class: 'cmp-grid'});
  root.append(grid, el('div', {class: 'card cmp-transport'}, playButton, scrub, timeLabel, rateSelect, alignSelect));

  const dom = {};
  for (const spec of specs) {
    const side = dom[spec.key] = {
      title: el('h2', {text: spec.key.toUpperCase()}),
      status: el('p', {class: 'cmp-status', role: 'status'}),
      retry: el('button', {type: 'button', class: 'secondary hidden', text: '분석 대기 중(공유 대기열) — 다시 불러오기'}),
      select: el('select', {'aria-label': `${spec.key.toUpperCase()} 장면`}),
      video: el('video', {muted: '', playsinline: '', preload: 'auto'}),
      syncLabel: el('div', {class: 'cmp-sync'}),
      notes: el('div'),
      cards: el('div', {class: 'cmp-cards'}),
    };
    side.video.muted = true;
    spec.video = side.video;
    grid.append(el('section', {class: 'card card-pad cmp-side'}, side.title, side.status, side.retry, side.select, side.syncLabel, side.video, side.notes, side.cards));
  }

  const graphs = [
    {title: '속도 vEgo (km/h)', series: [['vEgo', 3.6, false]]},
    {title: '가속도 aEgo(실선)·actAccel(점선) (m/s²)', series: [['aEgo', 1, false], ['actAccel', 1, true]]},
    {title: '선행차 거리 leadDist (m)', series: [['leadDist', 1, false]]},
    {title: '저크: aEgo 차분(10Hz) (m/s³)', series: [['jerk', 1, false]]},
  ];
  const graphSection = el('section', {class: 'card card-pad'});
  for (const graph of graphs) {
    graph.canvas = el('canvas', {class: 'cmp-graph', 'aria-label': graph.title});
    graphSection.append(el('div', {class: 'cmp-note', text: graph.title}), graph.canvas);
  }
  const legend = el('p', {class: 'cmp-note'});
  for (const spec of specs) legend.append(el('span', {style: `color:${COLORS[spec.key]};margin-right:14px`, text: `■ ${spec.key.toUpperCase()}`}));
  graphSection.prepend(legend);
  const tableSection = el('section', {class: 'card card-pad'});
  root.append(graphSection, tableSection);

  function renderLayout({range, cards, table}) {
    scrub.min = String(range[0]); scrub.max = String(range[1]);
    for (const {key, items} of cards) {
      dom[key].cards.replaceChildren(...items.map(c => el('div', {class: 'soft-card cmp-card'}, el('strong', {text: c.text}), el('span', {text: c.label}))));
    }
    tableSection.replaceChildren(el('div', {class: 'eyebrow', text: '장면 통계 대조'}),
      el('p', {class: 'cmp-note', text: 'Δ는 A·B 모두 유한한 수치일 때만 B−A로 계산합니다. 단위 SI(m/s, m/s², m, m/s³).'}));
    if (!table) { tableSection.append(el('p', {class: 'cmp-note', text: '선택한 장면이 없습니다.'})); return; }
    const head = el('tr', {}, ...table.head.map(text => el('th', {text})));
    const body = table.rows.map(row => el('tr', {}, ...row.map(text => el('td', {text}))));
    tableSection.append(el('table', {class: 'cmp-table'}, el('thead', {}, head), el('tbody', {}, ...body)));
  }

  function context(canvas) {
    const w = canvas.clientWidth, h = canvas.clientHeight, ratio = Math.min(devicePixelRatio || 1, 2);
    if (canvas.width !== Math.round(w * ratio) || canvas.height !== Math.round(h * ratio)) { canvas.width = Math.round(w * ratio); canvas.height = Math.round(h * ratio); }
    const ctx = canvas.getContext('2d'); ctx.setTransform(ratio, 0, 0, ratio, 0, 0); ctx.clearRect(0, 0, w, h); ctx.font = '11px system-ui';
    return {ctx, w, h};
  }

  function draw({clock, sides}) {
    const [lo, hi] = clock.range, span = hi - lo || 1;
    for (const graph of graphs) {
      const {ctx, w, h} = context(graph.canvas);
      const left = 44, right = w - 10, top = 10, bottom = h - 20;
      const x = tau => left + (tau - lo) / span * (right - left);
      let min = Infinity, max = -Infinity;
      for (const side of sides) for (const [name, scale] of graph.series) for (const value of side.data?.series[name] || []) if (finite(value)) { min = Math.min(min, value * scale); max = Math.max(max, value * scale); }
      if (!finite(min)) { min = 0; max = 1; }
      if (min === max) { min -= 1; max += 1; }
      const y = value => bottom - (value - min) / (max - min) * (bottom - top);
      ctx.strokeStyle = '#283440'; ctx.fillStyle = '#8592a3';
      for (const value of [min, (min + max) / 2, max]) { ctx.beginPath(); ctx.moveTo(left, y(value)); ctx.lineTo(right, y(value)); ctx.stroke(); ctx.textAlign = 'right'; ctx.fillText(value.toFixed(1), left - 4, y(value) + 4); }
      ctx.textAlign = 'center';
      for (let i = 0; i <= 4; i++) { const tau = lo + span * i / 4; ctx.fillText(`${tau.toFixed(1)} s`, x(tau), h - 5); }
      ctx.textAlign = 'left';
      for (const side of sides) {
        if (!side.data) continue;
        const t = side.data.series.t;
        for (const [name, scale, dashed] of graph.series) {
          const values = side.data.series[name];
          ctx.strokeStyle = COLORS[side.key]; ctx.lineWidth = 1.5; ctx.setLineDash(dashed ? [5, 4] : []); ctx.beginPath();
          let started = false;
          for (let k = 0; k < t.length; k++) {
            if (!finite(values[k])) { started = false; continue; }
            const px = x(t[k] - side.base), py = y(values[k] * scale);
            if (started) ctx.lineTo(px, py); else { ctx.moveTo(px, py); started = true; }
          }
          ctx.stroke(); ctx.setLineDash([]);
        }
      }
      const cx = x(clock.tau);
      ctx.strokeStyle = '#e1e7ed'; ctx.beginPath(); ctx.moveTo(cx, top); ctx.lineTo(cx, bottom); ctx.stroke();
      let line = 0;
      for (const side of sides) {
        const reason = side.sync.mode === 'released' ? REASON_TEXT[side.sync.reason] : '';
        if (reason) { ctx.fillStyle = COLORS[side.key]; ctx.fillText(`${side.key.toUpperCase()}: ${reason}`, Math.min(cx + 6, right - 170), top + 12 + 13 * line++); }
      }
    }
    scrub.value = String(clock.tau);
    timeLabel.textContent = `τ ${clock.tau.toFixed(2)} / ${clock.range[0].toFixed(1)}…${clock.range[1].toFixed(1)} s`;
    playButton.textContent = clock.running ? '일시정지' : '재생';
    for (const side of sides) dom[side.key].syncLabel.textContent = side.sync.mode === 'released' ? (REASON_TEXT[side.sync.reason] || '') : (side.sync.reason === 'buffering' ? REASON_TEXT.buffering : '');
  }

  const controller = createCompareController({
    sides: specs,
    align: alignSelect.value,
    deps: {
      fetch: (url, options) => fetch(url, options), now: () => performance.now(),
      setTimeout: (fn, ms) => setTimeout(fn, ms), clearTimeout: id => clearTimeout(id), AbortController,
      requestAnimationFrame: fn => requestAnimationFrame(fn),
    },
    view: {
      title: (key, text) => { dom[key].title.textContent = text; },
      status: (key, text, error) => { dom[key].status.textContent = text; dom[key].status.classList.toggle('cmp-error', error); },
      retry: (key, visible) => { dom[key].retry.classList.toggle('hidden', !visible); },
      notes: (key, lines) => { dom[key].notes.replaceChildren(...lines.map(text => el('p', {class: 'cmp-note', text}))); },
      episodes: (key, options) => { dom[key].select.replaceChildren(...options.map(o => el('option', {value: o.value, text: o.text}))); },
      selected: (key, value) => { dom[key].select.value = value; },
      align: value => { alignSelect.value = value; },
      layout: renderLayout,
      frame: draw,
    },
  });
  for (const spec of specs) {
    const side = dom[spec.key];
    side.video.addEventListener('error', () => controller.markMediaError(spec.key));
    side.retry.onclick = () => controller.retry(spec.key);
    side.select.onchange = () => controller.setEpisode(spec.key, side.select.value);
  }
  playButton.onclick = () => { if (controller.state().clock.running) controller.pause(); else controller.play(); };
  scrub.oninput = () => controller.seek(Number(scrub.value));
  rateSelect.onchange = () => controller.setRate(Number(rateSelect.value));
  alignSelect.onchange = () => controller.setAlign(alignSelect.value);
  controller.start();
}

if (typeof document !== 'undefined') init();
