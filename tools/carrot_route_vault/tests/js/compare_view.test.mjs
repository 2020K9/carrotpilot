// Pure helper and state-machine tests for compare_view.js (no DOM, no browser playback).
import test from 'node:test';
import assert from 'node:assert/strict';
import {
  BUFFER_RELEASE_MS, INITIAL_SYNC, clockShouldHold, deltaValue, defaultEpisodes, endLeadLabel, parseIndex,
  parseSafePath, retryAfterMs, syncDecision, syncStep, timelineRange, videoAt,
} from '../../compare_view.js';

const ORIGIN = 'https://routes.example';
const series = (n, first) => ({t: Array.from({length: n}, (_, k) => k / 10), v: Array.from({length: n}, (_, k) => first + k / 10)});
const near = (actual, expected) => assert.ok(Math.abs(actual - expected) < 1e-9, `${actual} != ${expected}`);
const ready = {currentTime: 0, duration: 60, hasVideo: true, error: false, readyState: 4};

test('path validation', () => {
  assert.equal(parseSafePath('/routes/d/s', ORIGIN), '/routes/d/s');
  assert.equal(parseSafePath('/s/tok', ORIGIN), '/s/tok');
  assert.equal(parseSafePath('/routes/TEST%20CAR%20x/2026-07-20--00-00-00--0:2', ORIGIN), '/routes/TEST%20CAR%20x/2026-07-20--00-00-00--0:2');
  for (const bad of ['https://x', '//x', '/routes/../x', '/s/a%2E%2E/', '/s/a.%2e/', '/admin', '/routes/a\u0000b', '/routes/a\\b',
                     '/routes/a\nb', '/routes/a%2Fb', '/routes/a?x=1', '', null, undefined]) {
    assert.equal(parseSafePath(bad, ORIGIN), null, String(bad));
  }
  assert.equal(parseIndex('0'), 0);
  assert.equal(parseIndex('1000000'), 1000000);
  for (const bad of ['-1', '1000001', '1.5', 'x', '', '12345678']) assert.equal(parseIndex(bad), null, bad);
});

test('Retry-After is honoured, defaulted and clamped', () => {
  assert.equal(retryAfterMs('5'), 5000);
  assert.equal(retryAfterMs(null), 2000);
  assert.equal(retryAfterMs('soon'), 2000);
  assert.equal(retryAfterMs('0'), 1000);
  assert.equal(retryAfterMs('99'), 30000);
});

test('start offset: video target follows the recorded video time, not base + tau', () => {
  const {t, v} = series(100, 0.4);
  near(videoAt(t, v, 0 + 0), 0.4);
  near(videoAt(t, v, 0 + 0.05), 0.45);
  near(videoAt(t, v, 3.0 + 0), v[30]);
  assert.equal(videoAt(t, v, -0.01), null);
  near(videoAt(t, v, t[99] + 0.1), v[99] + 0.1);
  assert.equal(videoAt(t, v, t[99] + 0.1001), null);
  assert.deepEqual(timelineRange([{t, base: 3.0}, {t: t.slice(0, 50), base: 0}]), [-3.0, t[99] - 3.0]);
});

test('missing video cells pause and resume without extrapolating across the gap', () => {
  const {t, v} = series(100, 0.4);
  for (let k = 40; k < 60; k++) v[k] = null;
  assert.equal(videoAt(t, v, 4.05), null);
  assert.equal(videoAt(t, v, 5.95), null);
  assert.equal(syncDecision({...ready, target: videoAt(t, v, 4.5)}).action, 'pause');
  const target = videoAt(t, v, 6.0);
  near(target, v[60]);
  assert.equal(syncDecision({...ready, currentTime: 0, target}).action, 'seek');
});

test('missing or failed A video pauses only A', () => {
  assert.deepEqual(syncDecision({...ready, hasVideo: false, target: 1}), {action: 'pause', reason: 'no-video'});
  assert.deepEqual(syncDecision({...ready, error: true, target: 1}), {action: 'pause', reason: 'no-video'});
  assert.equal(syncDecision({...ready, currentTime: 0, target: 1}).action, 'seek');
  assert.equal(syncDecision({...ready, currentTime: 1, target: 1.1}).action, 'keep');
});

test('a shorter video pauses past its end instead of holding the last frame', () => {
  assert.deepEqual(syncDecision({...ready, duration: 30, currentTime: 29.9, target: 30.5}), {action: 'pause', reason: 'out-of-range'});
  assert.deepEqual(syncDecision({...ready, target: -0.1}), {action: 'pause', reason: 'out-of-range'});
  assert.equal(syncDecision({...ready, duration: 60, currentTime: 30.4, target: 30.5}).action, 'keep');
});

test('sync tolerance is 0.15 s', () => {
  assert.equal(syncDecision({...ready, currentTime: 0, target: 0.15}).action, 'keep');  // |diff| exactly 0.15
  assert.equal(syncDecision({...ready, currentTime: 0.15, target: 0}).action, 'keep');
  assert.equal(syncDecision({...ready, currentTime: 0, target: 0.1501}).action, 'seek');
});

test('state: target null releases, returning target inside tolerance still plays', () => {
  let step = syncStep(INITIAL_SYNC, {...ready, currentTime: 5, target: 5}, 0);
  assert.equal(step.action, 'play');
  step = syncStep(step.state, {...ready, currentTime: 5, target: 5.05}, 16);
  assert.equal(step.action, 'keep');
  step = syncStep(step.state, {...ready, currentTime: 5.05, target: null}, 32);
  assert.deepEqual([step.action, step.state.mode, step.state.reason], ['pause', 'released', 'no-target']);
  step = syncStep(step.state, {...ready, currentTime: 5.05, target: 5.1}, 48);
  assert.deepEqual([step.action, step.state.mode], ['play', 'synced']);
  step = syncStep(step.state, {...ready, currentTime: 5.1, target: 5.12}, 64);
  assert.equal(step.action, 'keep');
});

test('state: buffering holds the clock for 5 s, then releases without re-holding', () => {
  let step = syncStep(INITIAL_SYNC, {...ready, currentTime: 1, target: 1}, 0);
  step = syncStep(step.state, {...ready, currentTime: 1, target: 1}, 10);
  assert.equal(step.action, 'keep');
  const buffering = {...ready, currentTime: 1, target: 1, readyState: 2};
  step = syncStep(step.state, buffering, 1000);
  assert.equal(clockShouldHold([step.state]), true);
  step = syncStep(step.state, buffering, 1000 + 4900);
  assert.equal(clockShouldHold([step.state]), true);
  step = syncStep(step.state, buffering, 1000 + BUFFER_RELEASE_MS);
  assert.equal(clockShouldHold([step.state]), true);  // exactly 5 s is not yet "over 5 s"
  step = syncStep(step.state, {...buffering, target: 1.01}, 1000 + BUFFER_RELEASE_MS + 1);
  assert.deepEqual([step.state.mode, step.state.reason, step.state.bufferReleased], ['released', 'buffer-timeout', true]);
  assert.equal(clockShouldHold([step.state]), false);
  for (let frame = 1; frame <= 5; frame++) {
    step = syncStep(step.state, {...buffering, target: 1 + frame}, 7000 + frame * 16);
    assert.equal(step.action, 'none');
    assert.equal(clockShouldHold([step.state]), false);
  }
  step = syncStep(step.state, {...ready, currentTime: 1, target: 6}, 8000);
  assert.deepEqual([step.action, step.state.mode, step.state.bufferReleased], ['seek-play', 'synced', false]);
  const other = syncStep(INITIAL_SYNC, {...ready, target: 0}, 0).state;
  assert.equal(clockShouldHold([other, step.state]), false);
});

test('state: out of range, then back in range seeks and plays', () => {
  let step = syncStep(INITIAL_SYNC, {...ready, duration: 30, currentTime: 29, target: 29}, 0);
  step = syncStep(step.state, {...ready, duration: 30, currentTime: 29, target: 29.05}, 16);
  assert.equal(step.action, 'keep');
  step = syncStep(step.state, {...ready, duration: 30, currentTime: 30, target: 31}, 32);
  assert.deepEqual([step.action, step.state.reason], ['pause', 'out-of-range']);
  step = syncStep(step.state, {...ready, duration: 30, currentTime: 30, target: 10}, 48);
  assert.equal(step.action, 'seek-play');
});

test('episode defaults, delta and labels', () => {
  const a = {episodes: [{type: 'cruise_decel'}, {type: 'stop_decel'}]};
  const b = {episodes: [{type: 'stop_decel'}, {type: 'cruise_decel'}]};
  assert.deepEqual(defaultEpisodes(a, b), [0, 1]);
  assert.equal(defaultEpisodes(a, {episodes: [{type: 'start_accel'}]}), null);
  assert.deepEqual(defaultEpisodes(a, undefined), [0, null]);
  assert.equal(deltaValue(1, 3), 2);
  for (const [x, y] of [[null, 1], [1, null], ['a', 'b'], [true, false], [[1], [2]], [NaN, 1], [1, Infinity]]) assert.equal(deltaValue(x, y), null);
  assert.equal(endLeadLabel('stop_decel'), '정차 시점 선행차 거리');
  assert.equal(endLeadLabel('creep_stop'), '정차 시점 선행차 거리');
  assert.equal(endLeadLabel('start_accel'), '장면 끝 선행차 거리');
  assert.equal(endLeadLabel('cruise_decel'), '장면 끝 선행차 거리');
});

// Controller tests: fake clock, fake fetch (request signal also aborts body reads), fake videos and frames.
import {
  POLL_DEADLINE_MS, createCompareController, missingSummary, noteLines, playIntent, validateDrive,
} from '../../compare_view.js';

const flush = () => new Promise(resolve => setImmediate(resolve));

function fakeClock() {
  let now = 0, seq = 0;
  const timers = [], scheduled = [], cleared = [];
  return {
    now: () => now, scheduled, cleared,
    setTimeout: (fn, ms) => { const id = ++seq; timers.push({id, at: now + Math.max(0, ms || 0), fn}); scheduled.push({id, at: now + Math.max(0, ms || 0)}); return id; },
    clearTimeout: id => { cleared.push(id); const i = timers.findIndex(t => t.id === id); if (i >= 0) timers.splice(i, 1); },
    async advance(ms) {
      const end = now + ms;
      await flush();
      for (;;) {
        timers.sort((x, y) => x.at - y.at || x.id - y.id);
        if (!timers.length || timers[0].at > end) break;
        const next = timers.shift();
        now = next.at;
        next.fn();
        await flush();
      }
      now = end;
      await flush();
    },
  };
}

const aborted = [];  // every fake request or body read rejected by its request signal
function hang(signal) {
  return new Promise((_, reject) => {
    const fail = () => { const error = new Error('The operation was aborted.'); error.name = 'AbortError'; aborted.push(signal); reject(error); };
    if (signal?.aborted) fail(); else signal?.addEventListener('abort', fail, {once: true});
  });
}

function respond(spec, signal) {
  if (spec === 'hang' || signal?.aborted) return hang(signal);
  if (typeof spec?.then === 'function') return spec.then(value => respond(value, null));
  const {status = 200, headers = {}, body, stall = false, bodyLater} = spec;
  return Promise.resolve({
    status, ok: status >= 200 && status < 300, headers: {get: name => headers[name] ?? null},
    json: () => bodyLater ?? (stall || signal?.aborted ? hang(signal) : Promise.resolve(structuredClone(body))),
  });
}

function fakeVideo() {
  return {
    t: 0, seeks: 0, playCalls: 0, pauseCalls: 0, paused: true, readyState: 4, duration: 3600, error: null,
    src: '', playbackRate: 1, throwOnPlay: false,
    get currentTime() { return this.t; },
    set currentTime(value) { this.t = value; this.seeks++; },
    play() { this.playCalls++; if (this.throwOnPlay) throw new Error('play failed'); this.paused = false; return Promise.resolve(); },
    pause() { this.pauseCalls++; this.paused = true; },
    getAttribute(name) { return name === 'src' && this.src ? this.src : null; },
    removeAttribute(name) { if (name === 'src') this.src = ''; },
  };
}

function episode(type, start, end, extra = {}) {
  return {type, start_t: start, end_t: end, start_video_t: start + 0.4, end_video_t: end + 0.4, duration: end - start,
          start_v: 10, end_v: 0, min_v: 0, max_v: 10, delta_v: -10, min_a: -1, max_a: 0, avg_a: -0.5, min_cmd_a: -1,
          max_cmd_a: 0, avg_cmd_a: -0.5, max_jerk: 1, std_jerk: 0.5, cmd_actual_corr: 0.9, start_lead_d: 20, min_lead_d: 5,
          end_lead_d: 5, lead_track_ids: [3], states: {pid: 10}, long_active_ratio: 1, pedal_input: false,
          start_clipped: false, end_clipped: false, ...extra};
}

function drive({n = 200, log = 'rlog.zst', episodes} = {}) {
  const t = Array.from({length: n}, (_, k) => k / 10);
  return {
    schemaVersion: 1, kind: 'carrotDrive', sourceVersion: 'abc123', sourceLog: log, videoAligned: true,
    missingServices: [], ambiguousDefaultFields: ['longitudinalPlan.vTargetNow'], invalidCounts: {carState: 0, carControl: 0, radarState: 0},
    series: {t, video_time_s: t.map(x => x + 0.4), vEgo: t.map(() => 10), aEgo: t.map(() => -0.5), actAccel: t.map(() => -0.4),
             leadDist: t.map(() => 30), jerk: t.map(() => 0)},
    episodes: episodes ?? [episode('stop_decel', 2, 8), episode('start_accel', 10, 14)],
  };
}

const ok = body => ({status: 200, body});
const manifestOf = key => ok({segments: [{index: 0, name: `${key}-seg`, videoUrl: `/video/${key}.mp4`}]});

function setup(handler, {keys = ['a', 'b'], manifest = manifestOf} = {}) {
  const clock = fakeClock(), frames = [], calls = [], views = {layout: null, frames: 0};
  let rafCalls = 0;
  const fetch = (url, options = {}) => {
    const key = url.split('/')[2];
    const isManifest = url.endsWith('/manifest');
    calls.push({url, key, manifest: isManifest, at: clock.now(), signal: options.signal});
    const count = calls.filter(c => c.url === url).length;
    return respond(isManifest ? manifest(key, count) : handler(key, count, clock.now()), options.signal);
  };
  const videos = Object.fromEntries(keys.map(k => [k, fakeVideo()]));
  const controller = createCompareController({
    sides: keys.map(key => ({key, path: `/routes/${key}`, index: 0, invalid: false, video: videos[key]})),
    deps: {fetch, now: clock.now, setTimeout: clock.setTimeout, clearTimeout: clock.clearTimeout, AbortController,
           requestAnimationFrame: cb => { rafCalls++; frames.push(cb); return rafCalls; }},
    view: {layout: layout => { views.layout = layout; }, frame: () => { views.frames++; if (views.throwFrame) throw new Error('draw failed'); }},
  });
  // Advance the fake clock in frame steps: playing, ready videos advance; then the pending frame runs.
  async function run(ms, step = 50) {
    for (let elapsed = 0; elapsed < ms; elapsed += step) {
      await clock.advance(step);
      for (const video of Object.values(videos)) if (!video.paused && video.readyState >= 3) video.t += step / 1000 * controller.state().clock.rate;
      const callback = frames.shift();
      if (callback) callback(clock.now());
    }
  }
  const side = key => controller.state().sides[keys.indexOf(key)];
  return {clock, frames, calls, views, videos, controller, run, side, rafCalls: () => rafCalls};
}

const always202 = (retry = '2') => ({status: 202, headers: {'Retry-After': retry}});

test('compare: A ready while B keeps 202 — A plays and seeks before B expires', async () => {
  const h = setup(key => key === 'a' ? ok(drive()) : always202());
  h.controller.start();
  await h.run(1000);
  assert.ok(h.side('a').data);
  assert.equal(h.side('b').data, null);
  assert.deepEqual(h.controller.state().clock.range.map(x => x + 0), [0, 19.9]);  // no episode yet: base 0
  h.controller.play();
  await h.run(1000);
  const {tau, running} = h.controller.state().clock;
  assert.ok(running && tau > 0.8, `tau ${tau}`);
  assert.ok(h.videos.a.playCalls > 0 && !h.videos.a.paused);
  h.controller.seek(5);
  assert.equal(h.controller.state().clock.tau, 5);
  await h.run(100);
  assert.ok(Math.abs(h.videos.a.currentTime - (h.controller.state().clock.tau + 0.4)) <= 0.15);
  assert.equal(h.side('b').finished, false);
  assert.equal(h.side('b').retryVisible, false);
  assert.ok(h.clock.now() < POLL_DEADLINE_MS);
});

test('compare: B expires — A keeps playing, defaults chosen, retry visible', async () => {
  const h = setup(key => key === 'a' ? ok(drive({n: 4000})) : always202());
  h.controller.start();
  await h.run(500);
  h.controller.play();
  await h.run(1000);
  assert.equal(h.controller.state().defaultsChosen, false);
  await h.clock.advance(POLL_DEADLINE_MS - 2000);
  await h.run(1000);
  const tauAtExpiry = h.controller.state().clock.tau;
  assert.equal(h.side('b').expired, true);
  assert.equal(h.side('b').retryVisible, true);
  assert.equal(h.side('b').status, '분석이 아직 끝나지 않았습니다.');
  assert.equal(h.controller.state().defaultsChosen, true);
  await h.run(1000);
  assert.equal(h.controller.state().clock.running, true);
  assert.ok(h.controller.state().clock.tau > tauAtExpiry);
  assert.equal(h.videos.a.paused, false);
  assert.ok(h.side('a').data);
});

test('compare: B retry succeeds after expiry — range grows, A selection and tau kept', async () => {
  let ready = false;
  const h = setup(key => key === 'a' ? ok(drive({n: 4000})) : ready ? ok(drive({n: 5000, log: 'b.zst'})) : always202());
  h.controller.start();
  await h.run(500);
  h.controller.play();
  await h.run(500);
  await h.clock.advance(POLL_DEADLINE_MS);
  await h.run(200);
  assert.equal(h.side('b').retryVisible, true);
  h.controller.pause();
  const before = h.controller.state();
  const [tau, episodeA] = [before.clock.tau, before.sides[0].episode];
  // B failing first gives A alone its first episode (defaultEpisodes with b=null), base 2.
  assert.deepEqual(before.clock.range, [-2, 397.9]);
  ready = true;
  h.controller.retry('b');
  await h.run(500);
  const after = h.controller.state();
  assert.equal(after.sides[1].data.sourceLog, 'b.zst');
  assert.equal(after.sides[1].retryVisible, false);
  assert.deepEqual(after.clock.range, [-2, 497.9]);
  assert.equal(after.sides[1].episode, 0);  // same type as A's current episode
  assert.equal(after.sides[0].episode, episodeA);
  assert.equal(after.clock.tau, tau);
});

test('validateDrive rejects missing arrays, length mismatch, bad episodes', () => {
  assert.equal(validateDrive(drive()).ok, true);
  const broken = [
    d => { delete d.series.actAccel; },
    d => { d.series.jerk.pop(); },
    d => { d.series.t.reverse(); },
    d => { d.episodes[0].start_t = '2'; },
    d => { d.episodes[0].start_t = 999; d.episodes[0].end_t = 1000; },
    d => { d.episodes[0].start_t = 9; },  // start after end
    d => { d.missingServices = 'carState'; },
    d => { d.series.vEgo[3] = 'x'; },
    d => { d.series.t = []; },
  ];
  for (const change of broken) {
    const data = drive();
    change(data);
    const result = validateDrive(data);
    assert.equal(result.ok, false, change.toString());
    assert.equal(typeof result.reason, 'string');
  }
});

async function brokenB(change) {
  const bad = drive();
  change(bad);
  const h = setup(key => key === 'a' ? ok(drive()) : ok(bad));
  h.controller.start();
  await h.run(200);
  assert.equal(h.side('b').data, null);
  assert.equal(h.videos.b.getAttribute('src'), null);
  assert.equal(h.side('b').statusError, true);
  assert.match(h.side('b').status, /형식이 올바르지 않습니다/);
  h.controller.play();
  const start = h.controller.state().clock.tau;
  for (let i = 0; i < 50; i++) await h.run(50);
  const state = h.controller.state();
  assert.ok(state.clock.tau > start + 2, `tau ${state.clock.tau}`);
  assert.equal(state.sides[0].sync.mode, 'synced');
  assert.equal(h.videos.a.paused, false);
  assert.ok(Math.abs(h.videos.a.currentTime - (state.sides[0].base + state.clock.tau + 0.4)) <= 0.15);
  assert.equal(h.frames.length, 1);
}

test('compare: B missing array — B cleared, A and clock keep running', async () => {
  await brokenB(d => { delete d.series.jerk; });
});

test('compare: B bad episode — same as above', async () => {
  await brokenB(d => { d.episodes[0].start_t = 'x'; });
});

test("compare: exception inside one side's sync keeps the frame loop", async () => {
  const h = setup(() => ok(drive()));
  h.videos.a.throwOnPlay = true;
  h.controller.start();
  await h.run(200);
  h.controller.play();
  const calls = h.rafCalls();
  await h.run(50);
  assert.equal(h.rafCalls(), calls + 1);  // the throwing frame still scheduled its successor
  assert.equal(h.side('a').data, null);
  assert.match(h.side('a').status, /play failed/);
  const tau = h.controller.state().clock.tau;
  await h.run(500);
  assert.equal(h.rafCalls(), calls + 11);
  assert.ok(h.controller.state().clock.tau > tau);
  assert.equal(h.videos.b.paused, false);
});

test('deadline: hanging drive request aborts at 360 s and shows retry', async () => {
  const h = setup(() => 'hang', {keys: ['a']});
  h.controller.start();
  await h.clock.advance(POLL_DEADLINE_MS - 1);
  const request = h.calls.find(c => !c.manifest);
  assert.ok(request);
  assert.equal(request.signal.aborted, false);
  assert.equal(h.side('a').retryVisible, false);
  await h.clock.advance(1);
  assert.equal(request.signal.aborted, true);
  assert.ok(aborted.includes(request.signal));
  assert.equal(h.side('a').retryVisible, true);
  assert.equal(h.side('a').status, '분석이 아직 끝나지 않았습니다.');
});

test('deadline: hanging manifest request is covered by the same deadline', async () => {
  const h = setup(() => ok(drive()), {keys: ['a'], manifest: () => 'hang'});
  h.controller.start();
  await h.clock.advance(POLL_DEADLINE_MS - 1);
  assert.equal(h.calls.length, 1);
  assert.equal(h.side('a').retryVisible, false);
  await h.clock.advance(1);
  assert.equal(h.calls[0].signal.aborted, true);
  assert.equal(h.side('a').retryVisible, true);
  assert.equal(h.side('a').expired, true);
});

test('deadline: Retry-After 30 at 350 s waits only 10 s', async () => {
  const h = setup((_key, _count, now) => always202(now >= 350000 ? '30' : '25'), {keys: ['a']});
  h.controller.start();
  await h.clock.advance(POLL_DEADLINE_MS - 1);
  const polls = h.calls.filter(c => !c.manifest);
  assert.equal(polls.length, 15);
  assert.equal(polls.at(-1).at, 350000);
  assert.equal(h.side('a').retryVisible, false);
  await h.clock.advance(1);
  assert.equal(h.side('a').retryVisible, true);
  assert.equal(h.clock.now(), POLL_DEADLINE_MS);
  await h.clock.advance(20000);
  assert.equal(h.calls.filter(c => !c.manifest).length, 15);
});

test('deadline: retry after expiry starts a new deadline and succeeds', async () => {
  let ready = false;
  const h = setup(() => ready ? ok(drive()) : 'hang', {keys: ['a']});
  h.controller.start();
  const firstTimer = h.clock.scheduled.find(s => s.at === POLL_DEADLINE_MS).id;
  await h.clock.advance(POLL_DEADLINE_MS);
  assert.equal(h.side('a').retryVisible, true);
  ready = true;
  const retryAt = h.clock.now();
  h.controller.retry('a');
  assert.ok(h.clock.cleared.includes(firstTimer));
  const secondTimer = h.clock.scheduled.find(s => s.at === retryAt + POLL_DEADLINE_MS);
  assert.ok(secondTimer && secondTimer.id !== firstTimer);
  await h.clock.advance(100);
  assert.ok(h.side('a').data);
  assert.equal(h.side('a').retryVisible, false);
  assert.ok(h.clock.cleared.includes(secondTimer.id));
});

test('missingSummary counts per service and per graph series', () => {
  const data = drive();
  data.invalidCounts = {carState: 0, carControl: 3, radarState: 2, modelV2: 0};
  for (const k of [1, 2, 3, 4]) data.series.actAccel[k] = null;
  for (const k of [5, 6, 7, 8, 9, 10]) data.series.leadDist[k] = null;
  assert.deepEqual(missingSummary(data), {services: [['carControl', 3], ['radarState', 2]], series: [['actAccel', 4], ['leadDist', 6]]});
  const lines = noteLines(data);
  assert.ok(lines.some(l => l.includes('carControl 3, radarState 2')));
  assert.ok(lines.some(l => l.includes('그래프 값 없음 칸: actAccel 4, leadDist 6(선행차 없음 포함)')));
  assert.deepEqual(missingSummary(drive()), {services: [], series: []});
});

test('hold: A buffering pauses B and both resume together without repeated seeks', async () => {
  assert.equal(playIntent({running: true, hold: true, state: {mode: 'synced'}}), 'pause');
  assert.equal(playIntent({running: true, hold: false, state: {mode: 'synced'}}), 'play');
  assert.equal(playIntent({running: false, hold: false, state: {mode: 'synced'}}), 'pause');
  assert.equal(playIntent({running: true, hold: false, state: {mode: 'released'}}), 'pause');
  const h = setup(() => ok(drive({n: 1000})));
  h.controller.start();
  await h.run(200);
  h.controller.play();
  await h.run(1000);
  assert.ok(!h.videos.a.paused && !h.videos.b.paused);
  h.videos.a.readyState = 2;
  await h.run(100);
  const tau = h.controller.state().clock.tau, bTime = h.videos.b.currentTime, bSeeks = h.videos.b.seeks;
  await h.run(2000);
  assert.equal(h.videos.b.paused, true);
  assert.equal(h.videos.b.currentTime, bTime);
  assert.equal(h.videos.b.seeks, bSeeks);
  assert.equal(h.controller.state().clock.tau, tau);
  const plays = [h.videos.a.playCalls, h.videos.b.playCalls];
  h.videos.a.readyState = 4;
  await h.run(50);
  assert.deepEqual([h.videos.a.playCalls, h.videos.b.playCalls], [plays[0] + 1, plays[1] + 1]);
  const seeks = [h.videos.a.seeks, h.videos.b.seeks];
  await h.run(1000);
  assert.ok(h.videos.a.seeks - seeks[0] <= 1 && h.videos.b.seeks - seeks[1] <= 1);
  // Buffering beyond 5 s releases A only; B and the clock continue.
  h.videos.a.readyState = 2;
  await h.run(6000);
  const state = h.controller.state();
  assert.deepEqual([state.sides[0].sync.mode, state.sides[0].sync.reason], ['released', 'buffer-timeout']);
  assert.equal(h.videos.b.paused, false);
  const later = state.clock.tau;
  await h.run(500);
  assert.ok(h.controller.state().clock.tau > later);
});

test('frame: one tick schedules exactly one next frame (no double scheduling)', async () => {
  const h = setup(() => ok(drive()));
  h.controller.start();
  h.controller.start();
  assert.equal(h.rafCalls(), 1);
  assert.equal(h.frames.length, 1);
  for (let i = 1; i <= 10; i++) {
    await h.run(50);
    assert.equal(h.rafCalls(), 1 + i);
    assert.equal(h.frames.length, 1);
  }
  h.controller.tick(h.clock.now());  // a direct tick while a frame is pending adds no second loop
  assert.equal(h.frames.length, 1);
  assert.equal(h.rafCalls(), 11);
  h.controller.play();
  h.videos.a.throwOnPlay = true;
  await h.run(50);
  assert.equal(h.rafCalls(), 12);
  assert.equal(h.frames.length, 1);
  assert.equal(h.side('a').statusError, true);
  // An exception escaping the frame (drawing) still schedules exactly one successor.
  h.views.throwFrame = true;
  await h.clock.advance(50);
  assert.throws(() => h.frames.shift()(h.clock.now()), /draw failed/);
  assert.equal(h.rafCalls(), 13);
  assert.equal(h.frames.length, 1);
  h.views.throwFrame = false;
  await h.run(50);
  assert.equal(h.rafCalls(), 14);
  assert.equal(h.frames.length, 1);
});

test('compare: user picks A episode and plays before B expires — initial defaults keep A selection and tau', async () => {
  let ready = false;
  const h = setup(key => key === 'a' ? ok(drive({n: 4000, episodes: [episode('stop_decel', 2, 8), episode('start_accel', 10, 14)]}))
    : ready ? ok(drive({n: 4000, episodes: [episode('stop_decel', 1, 5), episode('start_accel', 20, 30)]})) : always202());
  h.controller.start();
  await h.run(200);
  h.controller.setEpisode('a', '1');
  h.controller.play();
  await h.run(2000);
  assert.ok(h.videos.a.playCalls > 0 && !h.videos.a.paused);
  await h.clock.advance(POLL_DEADLINE_MS - 2500);
  await h.run(200);
  const before = h.controller.state().clock.tau;
  await h.run(400);  // B expires inside this window
  const state = h.controller.state();
  assert.equal(state.sides[1].expired, true);
  assert.equal(state.defaultsChosen, true);
  assert.equal(state.sides[0].episode, 1);
  assert.equal(state.align, 'episode');
  assert.equal(state.clock.running, true);
  assert.ok(state.clock.tau >= before);
  assert.equal(h.videos.a.paused, false);
  // A late B success takes the same episode type as A's current choice, still without resetting τ.
  h.controller.pause();
  const tau = h.controller.state().clock.tau;
  ready = true;
  h.controller.retry('b');
  await h.run(200);
  assert.equal(h.side('b').episode, 1);
  assert.equal(h.side('a').episode, 1);
  assert.equal(h.controller.state().clock.tau, tau);
});

test('deadline: body stalls after headers — request signal aborts json() at 360 s', async () => {
  const h = setup(() => ({status: 200, stall: true}), {keys: ['a']});
  h.controller.start();
  await h.clock.advance(POLL_DEADLINE_MS - 1);
  const request = h.calls.find(c => !c.manifest);
  assert.equal(request.signal.aborted, false);
  assert.equal(h.side('a').retryVisible, false);
  assert.ok(!aborted.includes(request.signal));
  await h.clock.advance(1);
  assert.equal(request.signal.aborted, true);
  assert.ok(aborted.includes(request.signal));  // the pending json() rejected through the request signal
  assert.equal(h.side('a').data, null);
  assert.equal(h.videos.a.getAttribute('src'), null);
  assert.equal(h.side('a').retryVisible, true);
});

test('generation: late result of previous load does not clear new success', async () => {
  const deferred = [];
  const later = () => { let resolve, reject; const promise = new Promise((y, n) => { resolve = y; reject = n; }); deferred.push({resolve, reject}); return promise; };
  // 1: response arrives late; 2: headers now, body late; 3: response fails late; 4: succeeds.
  const h = setup((_key, count) => count === 2 ? {status: 200, bodyLater: later()} : count <= 3 ? later() : ok(drive({log: 'third.zst'})), {keys: ['a']});
  h.controller.start();
  for (let generation = 2; generation <= 4; generation++) {
    await h.clock.advance(100);
    h.controller.retry('a');
  }
  await h.clock.advance(100);
  const check = () => {
    const side = h.side('a');
    assert.equal(side.data.sourceLog, 'third.zst');
    assert.equal(h.videos.a.getAttribute('src'), '/video/a.mp4');
    assert.equal(side.status, '');
    assert.equal(side.statusError, false);
    assert.equal(side.retryVisible, false);
    assert.deepEqual(h.controller.state().clock.range, [-2, 17.9]);  // single side: first episode by default
  };
  check();
  assert.equal(deferred.length, 3);
  deferred[0].resolve(ok(drive({log: 'first.zst'})));
  deferred[1].resolve(drive({log: 'second.zst'}));
  deferred[2].reject(new TypeError('network down'));
  await h.clock.advance(POLL_DEADLINE_MS * 2);
  check();
});

test('validateDrive rejects bad top-level values and bad episode entries, keeps display stats', () => {
  const broken = [
    d => { d.series = []; },
    d => { d.series = null; },
    d => { d.episodes = {}; },
    d => { d.invalidCounts.carState = -1; },
    d => { d.invalidCounts.carState = 1.5; },
    d => { d.invalidCounts.carState = '3'; },
    d => { d.invalidCounts = []; },
    d => { d.videoAligned = 'yes'; },
    d => { d.sourceLog = 5; },
    d => { d.ambiguousDefaultFields = [1]; },
    d => { d.episodes[0] = 5; },
    d => { d.episodes[0] = null; },
    d => { d.episodes[0].type = 3; },
    d => { d.episodes[0].cmd_actual_corr = '0.9'; },
    d => { delete d.episodes[0].end_lead_d; },
    d => { d.episodes[0].std_jerk = NaN; },
    d => { d.episodes[0].pedal_input = 'yes'; },
    d => { d.episodes[0].states = {pid: 'x'}; },
    d => { d.episodes[0].lead_track_ids = ['3']; },
  ];
  for (const change of broken) {
    const data = drive();
    change(data);
    assert.equal(validateDrive(data).ok, false, change.toString());
  }
  const nulls = drive({episodes: [episode('stop_decel', 2, 8, {cmd_actual_corr: null, end_lead_d: null, std_jerk: null, max_jerk: null,
                                                               min_a: null, lead_track_ids: null, states: null})]});
  delete nulls.episodes[0].pedal_input;
  assert.equal(validateDrive(nulls).ok, true);
});

test('compare: published side renders existing stat cards and table', async () => {
  const h = setup(key => ok(drive({episodes: [episode('stop_decel', 2, 8, {pedal_input: key === 'a', end_lead_d: null})]})));
  h.controller.start();
  await h.run(200);
  h.controller.setEpisode('a', '0');
  const {cards, table} = h.views.layout;
  const labels = cards[0].items.map(c => c.label);
  assert.deepEqual(labels, ['상관도 cmd_actual_corr (시간 지연 미보정)', '정차 시점 선행차 거리', '저크 표준편차: aEgo 차분(10Hz)',
                            '최대 저크: aEgo 차분(10Hz)', '최저 가속도', '길이', 'gasPressed 또는 brakePressed']);
  assert.deepEqual(cards[0].items.map(c => c.text), ['0.900', '—', '0.500 m/s³', '1.000 m/s³', '-1.000 m/s²', '6.000 s', '운전자 페달 입력 있음']);
  assert.equal(cards[1].items.length, 6);
  assert.deepEqual(table.head, ['항목', 'A', 'B', 'Δ(B−A)']);
  assert.equal(table.rows.length, 29);
  assert.deepEqual(table.rows.find(r => r[0] === 'min_lead_d'), ['min_lead_d', '5.000', '5.000', '0.000']);
  assert.deepEqual(table.rows.find(r => r[0] === 'type'), ['type', 'stop_decel', 'stop_decel', '—']);
});
