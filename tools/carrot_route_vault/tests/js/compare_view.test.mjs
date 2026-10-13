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
