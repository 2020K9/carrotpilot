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

function init() {
  document.head.append(el('style', {text: STYLE}));
  const root = document.querySelector('#compare');
  const params = new URLSearchParams(location.search);
  let align = params.get('align') === 'start' ? 'start' : 'episode';
  const sides = [];
  for (const key of ['a', 'b']) {
    const raw = params.get(key), rawIndex = params.get(key + 'i');
    if (key === 'b' && raw === null) continue;
    const path = parseSafePath(raw, location.origin), index = parseIndex(rawIndex ?? '');
    sides.push({key, path, index, invalid: path === null || index === null, data: null, sync: INITIAL_SYNC,
                videoUrl: '', mediaError: false, episode: null, base: 0, generation: 0});
  }

  root.append(
    el('p', {class: 'cmp-note', text: '이 주소는 공유 링크와 같은 접근 권한을 가집니다. 전달할 때 주의하세요.'}),
    el('p', {class: 'cmp-note', text: '영상 시각: modelV2 기준·동기 오차 미검증. 장면 기준: 자동 기준(미승인 기본값). 숫자와 정의만 표시하며 주행 품질을 판정하지 않습니다.'}),
  );
  const alignSelect = el('select', {'aria-label': '정렬 기준'}, el('option', {value: 'episode', text: '장면 시작 정렬'}), el('option', {value: 'start', text: '로그 시작 정렬'}));
  alignSelect.value = align;
  const playButton = el('button', {type: 'button', text: '재생'});
  const scrub = el('input', {type: 'range', min: '0', max: '0', step: '0.01', value: '0', 'aria-label': '공통 재생 위치'});
  const rateSelect = el('select', {'aria-label': '배속'}, ...RATES.map(r => el('option', {value: String(r), text: `${r}×`})));
  rateSelect.value = '1';
  const timeLabel = el('span', {class: 'cmp-time'});
  const grid = el('div', {class: 'cmp-grid'});
  root.append(grid, el('div', {class: 'card cmp-transport'}, playButton, scrub, timeLabel, rateSelect, alignSelect));

  for (const side of sides) {
    side.title = el('h2', {text: side.key.toUpperCase()});
    side.status = el('p', {class: 'cmp-status', role: 'status'});
    side.retry = el('button', {type: 'button', class: 'secondary hidden', text: '분석 대기 중(공유 대기열) — 다시 불러오기'});
    side.select = el('select', {'aria-label': `${side.key.toUpperCase()} 장면`});
    side.video = el('video', {muted: '', playsinline: '', preload: 'auto'});
    side.video.muted = true;
    side.syncLabel = el('div', {class: 'cmp-sync'});
    side.notes = el('div');
    side.cards = el('div', {class: 'cmp-cards'});
    grid.append(el('section', {class: 'card card-pad cmp-side'}, side.title, side.status, side.retry, side.select, side.syncLabel, side.video, side.notes, side.cards));
    side.video.addEventListener('error', () => { side.mediaError = true; });
    side.retry.onclick = () => load(side);
    side.select.onchange = () => { side.episode = side.select.value === '' ? null : Number(side.select.value); relayout(); };
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
  for (const side of sides) legend.append(el('span', {style: `color:${COLORS[side.key]};margin-right:14px`, text: `■ ${side.key.toUpperCase()}`}));
  graphSection.prepend(legend);
  const tableSection = el('section', {class: 'card card-pad'});
  root.append(graphSection, tableSection);

  const clock = {tau: 0, running: false, last: 0, rate: 1, range: [0, 0]};

  function setStatus(side, text, error = false) { side.status.textContent = text; side.status.classList.toggle('cmp-error', error); }

  function relayout() {
    align = alignSelect.value;
    for (const side of sides) {
      const episode = side.data && side.episode !== null ? side.data.episodes[side.episode] : null;
      side.base = align === 'episode' && episode ? episode.start_t : 0;
    }
    clock.range = timelineRange(sides.filter(s => s.data).map(s => ({t: s.data.series.t, base: s.base})));
    clock.tau = align === 'episode' ? 0 : clock.range[0];
    scrub.min = String(clock.range[0]); scrub.max = String(clock.range[1]);
    renderCards(); renderTable(); draw();
  }

  function fillEpisodes(side) {
    side.select.replaceChildren(el('option', {value: '', text: '장면 선택 안 함'}));
    side.data.episodes.forEach((e, i) => {
      const clipped = e.start_clipped || e.end_clipped ? ' · 잘림' : '';
      const v = x => finite(x) ? (x * 3.6).toFixed(1) : '—';
      side.select.append(el('option', {value: String(i), text: `${TYPE_TEXT[e.type] || e.type} · t ${e.start_t.toFixed(1)}s · ${v(e.start_v)}→${v(e.end_v)} km/h${clipped}`}));
    });
  }

  function chooseDefaults() {
    const loaded = sides.every(s => s.data || s.invalid || s.failed);
    if (!loaded) return;
    const [a, b] = sides;
    const pick = defaultEpisodes(a.data, b ? b.data : undefined);
    if (pick) { a.episode = pick[0]; if (b) b.episode = pick[1]; } else alignSelect.value = 'start';
    for (const side of sides) if (side.data) side.select.value = side.episode === null ? '' : String(side.episode);
    relayout();
  }

  async function load(side) {
    const token = ++side.generation;
    side.retry.classList.add('hidden'); side.failed = false;
    if (side.invalid) { setStatus(side, '잘못된 비교 주소입니다(경로 또는 index).', true); side.failed = true; chooseDefaults(); return; }
    try {
      setStatus(side, '세그먼트 정보를 불러오는 중…');
      const manifest = await fetch(`${side.path}/manifest`, {cache: 'no-store'});
      if (!manifest.ok) throw new Error(manifest.status === 404 ? '로그를 찾을 수 없습니다.' : `세그먼트 정보를 불러오지 못했습니다 (${manifest.status}).`);
      const info = await manifest.json();
      const segment = (info.segments || []).find(s => Number(s.index) === side.index);
      if (!segment) throw new Error('선택한 세그먼트가 이 링크 범위에 없습니다.');
      side.title.textContent = `${side.key.toUpperCase()} · ${segment.name || ''}`;
      side.videoUrl = segment.videoUrl || '';
      const deadline = Date.now() + POLL_DEADLINE_MS;
      while (token === side.generation) {
        setStatus(side, '주행 분석을 준비하는 중…');
        const response = await fetch(`${side.path}/drive/${side.index}`, {cache: 'no-store'});
        if (response.status === 202 || response.status === 503) {
          if (Date.now() > deadline) { setStatus(side, '분석이 아직 끝나지 않았습니다.'); side.retry.classList.remove('hidden'); return; }
          await new Promise(resolve => setTimeout(resolve, retryAfterMs(response.headers.get('Retry-After'))));
          continue;
        }
        if (!response.ok) throw new Error(response.status === 404 ? '주행 분석용 rlog가 없습니다.' : response.status === 422 ? '주행 분석에 실패했습니다.' : `주행 분석을 불러오지 못했습니다 (${response.status}).`);
        const data = await response.json();
        if (data.schemaVersion !== 1 || data.kind !== 'carrotDrive' || !Array.isArray(data.series?.t) || !data.series.t.length) throw new Error('주행 분석 형식이 올바르지 않습니다.');
        if (token !== side.generation) return;
        side.data = data;
        if (side.videoUrl && data.videoAligned) { side.mediaError = false; side.video.src = side.videoUrl; } else side.video.removeAttribute('src');
        const nullCells = data.series.vEgo.filter(x => x === null).length;
        side.notes.replaceChildren(
          el('p', {class: 'cmp-note', text: `코드 ${data.sourceVersion} · ${data.sourceLog} · ${data.series.t.length}칸(10Hz, 로그 시각)${data.videoAligned ? '' : ' · 영상 시각 없음: 추정 동기 안 함'}`}),
          ...(nullCells ? [el('p', {class: 'cmp-note', text: `carState 무효/결측 칸 ${nullCells}개(값 없음 처리)`})] : []),
          ...(data.missingServices.length ? [el('p', {class: 'cmp-note', text: `기록 없는 서비스: ${data.missingServices.join(', ')}`})] : []),
          el('p', {class: 'cmp-note', text: `0은 기록값 또는 기본값일 수 있음: ${data.ambiguousDefaultFields.join(', ')}`}),
        );
        fillEpisodes(side);
        setStatus(side, '');
        chooseDefaults();
        return;
      }
    } catch (error) {
      if (token === side.generation) { side.failed = true; setStatus(side, error.message, true); chooseDefaults(); }
    }
  }

  function selected(side) { return side.data && side.episode !== null ? side.data.episodes[side.episode] : null; }

  function renderCards() {
    for (const side of sides) {
      const e = selected(side);
      side.cards.replaceChildren();
      if (!e) continue;
      const card = (label, value, unit = '') => el('div', {class: 'soft-card cmp-card'}, el('strong', {text: finite(value) ? `${value.toFixed(3)}${unit}` : '—'}), el('span', {text: label}));
      side.cards.append(
        card('상관도 cmd_actual_corr (시간 지연 미보정)', e.cmd_actual_corr),
        card(endLeadLabel(e.type), e.end_lead_d, ' m'),
        card('저크 표준편차: aEgo 차분(10Hz)', e.std_jerk, ' m/s³'),
        card('최대 저크: aEgo 차분(10Hz)', e.max_jerk, ' m/s³'),
        card('최저 가속도', e.min_a, ' m/s²'),
        card('길이', e.duration, ' s'),
      );
      if (e.pedal_input) side.cards.append(el('div', {class: 'soft-card cmp-card'}, el('strong', {text: '운전자 페달 입력 있음'}), el('span', {text: 'gasPressed 또는 brakePressed'})));
    }
  }

  function renderTable() {
    const episodes = sides.map(selected);
    tableSection.replaceChildren(el('div', {class: 'eyebrow', text: '장면 통계 대조'}),
      el('p', {class: 'cmp-note', text: 'Δ는 A·B 모두 유한한 수치일 때만 B−A로 계산합니다. 단위 SI(m/s, m/s², m, m/s³).'}));
    if (!episodes.some(Boolean)) { tableSection.append(el('p', {class: 'cmp-note', text: '선택한 장면이 없습니다.'})); return; }
    const head = el('tr', {}, el('th', {text: '항목'}), ...sides.map(s => el('th', {text: s.key.toUpperCase()})), ...(sides.length > 1 ? [el('th', {text: 'Δ(B−A)'})] : []));
    const body = STAT_KEYS.map(key => {
      const values = episodes.map(e => e ? e[key] : null);
      const cells = values.map(v => el('td', {text: formatStat(v)}));
      if (sides.length > 1) cells.push(el('td', {text: formatStat(deltaValue(values[0], values[1]))}));
      return el('tr', {}, el('td', {text: key}), ...cells);
    });
    tableSection.append(el('table', {class: 'cmp-table'}, el('thead', {}, head), el('tbody', {}, ...body)));
  }

  function context(canvas) {
    const w = canvas.clientWidth, h = canvas.clientHeight, ratio = Math.min(devicePixelRatio || 1, 2);
    if (canvas.width !== Math.round(w * ratio) || canvas.height !== Math.round(h * ratio)) { canvas.width = Math.round(w * ratio); canvas.height = Math.round(h * ratio); }
    const ctx = canvas.getContext('2d'); ctx.setTransform(ratio, 0, 0, ratio, 0, 0); ctx.clearRect(0, 0, w, h); ctx.font = '11px system-ui';
    return {ctx, w, h};
  }

  function draw() {
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
    for (const side of sides) side.syncLabel.textContent = side.sync.mode === 'released' ? (REASON_TEXT[side.sync.reason] || '') : (side.sync.reason === 'buffering' ? REASON_TEXT.buffering : '');
  }

  function syncSides(now) {
    for (const side of sides) {
      const video = side.video, hasVideo = Boolean(side.data?.videoAligned && side.videoUrl && video.getAttribute('src'));
      const target = side.data ? videoAt(side.data.series.t, side.data.series.video_time_s, side.base + clock.tau) : null;
      const step = syncStep(side.sync, {target, currentTime: video.currentTime, duration: video.duration, hasVideo,
                                        error: side.mediaError || Boolean(video.error), readyState: video.readyState}, now);
      side.sync = step.state;
      if (step.action === 'seek' || step.action === 'seek-play') video.currentTime = target;
      if (step.action === 'pause') video.pause();
      if (!clock.running) video.pause();
      else if (side.sync.mode === 'synced' && video.paused) video.play().catch(() => { side.mediaError = true; });
    }
  }

  function tick(now) {
    if (clock.running && !clockShouldHold(sides.map(s => s.sync))) {
      clock.tau += (now - clock.last) / 1000 * clock.rate;
      if (clock.tau >= clock.range[1]) { clock.tau = clock.range[1]; clock.running = false; }
    }
    clock.last = now;
    syncSides(now);
    draw();
    requestAnimationFrame(tick);
  }

  playButton.onclick = () => {
    if (!clock.running && clock.tau >= clock.range[1]) clock.tau = clock.range[0];
    clock.running = !clock.running && clock.range[1] > clock.range[0];
  };
  scrub.oninput = () => { clock.tau = Number(scrub.value); };
  rateSelect.onchange = () => { clock.rate = Number(rateSelect.value); for (const side of sides) side.video.playbackRate = clock.rate; };
  alignSelect.onchange = relayout;
  for (const side of sides) load(side);
  requestAnimationFrame(now => { clock.last = now; tick(now); });
}

if (typeof document !== 'undefined') init();
