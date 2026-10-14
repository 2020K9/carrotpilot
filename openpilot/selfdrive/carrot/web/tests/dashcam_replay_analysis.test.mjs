import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import test from "node:test";
import vm from "node:vm";

// 대시캠 영상 칸 → 리플레이, 실패 시 일반 재생, 일반 재생기의 분석 단추,
// 리플레이 가속도 그래프의 명령 가속 줄과 저크 줄을 실제 코드로 확인한다.

const root = path.resolve(import.meta.dirname, "..");
const read = (relativePath) => fs.readFileSync(path.join(root, relativePath), "utf8");

// ---------- 시험용 가짜 DOM ----------

class FakeClassList {
  constructor(owner) { this.owner = owner; }
  list() { return String(this.owner.className || "").split(/\s+/).filter(Boolean); }
  add(...names) { this.owner.className = [...new Set([...this.list(), ...names])].join(" "); }
  remove(...names) { this.owner.className = this.list().filter((name) => !names.includes(name)).join(" "); }
  contains(name) { return this.list().includes(name); }
  toggle(name, force) {
    const on = force === undefined ? !this.contains(name) : Boolean(force);
    if (on) this.add(name);
    else this.remove(name);
    return on;
  }
}

class FakeElement {
  constructor(doc, tag = "div") {
    this.ownerDocument = doc;
    this.tagName = String(tag).toUpperCase();
    this.className = "";
    this.id = "";
    this.children = [];
    this.parentElement = null;
    this.dataset = {};
    this.style = {};
    this.hidden = false;
    this.disabled = false;
    this.textContent = "";
    this.innerHTML = "";
    this.attributes = new Map();
    this.listeners = [];
    this.options = [];
    this.placeholders = new Map();
    this.classList = new FakeClassList(this);
  }

  get isConnected() {
    let node = this;
    while (node.parentElement) node = node.parentElement;
    return node === this.ownerDocument.body;
  }

  append(...nodes) {
    for (const node of nodes) {
      if (!(node instanceof FakeElement)) continue;
      if (node.isFragment) {
        const moved = [...node.children];
        node.children = [];
        this.append(...moved);
        continue;
      }
      node.remove();
      node.parentElement = this;
      this.children.push(node);
    }
  }

  appendChild(node) {
    this.append(node);
    return node;
  }

  replaceChildren(...nodes) {
    for (const child of this.children) child.parentElement = null;
    this.children = [];
    this.append(...nodes);
  }

  remove() {
    const parent = this.parentElement;
    if (!parent) return;
    parent.children = parent.children.filter((child) => child !== this);
    parent.placeholders.forEach((value, key) => { if (value === this) parent.placeholders.delete(key); });
    this.parentElement = null;
  }

  setAttribute(name, value) { this.attributes.set(name, String(value)); }
  getAttribute(name) { return this.attributes.has(name) ? this.attributes.get(name) : null; }
  removeAttribute(name) { this.attributes.delete(name); }
  hasAttribute(name) { return this.attributes.has(name); }
  toggleAttribute(name, force) {
    const on = force === undefined ? !this.hasAttribute(name) : Boolean(force);
    if (on) this.setAttribute(name, "");
    else this.removeAttribute(name);
    return on;
  }

  addEventListener(type, listener, options = {}) {
    this.listeners.push({ type, listener, once: options?.once === true });
  }

  removeEventListener(type, listener) {
    this.listeners = this.listeners.filter((entry) => !(entry.type === type && entry.listener === listener));
  }

  dispatch(type, init = {}) {
    const event = {
      type,
      target: this,
      currentTarget: this,
      stopPropagation() {},
      stopImmediatePropagation() {},
      preventDefault() {},
      ...init,
    };
    for (const entry of [...this.listeners]) {
      if (entry.type !== type) continue;
      if (entry.once) this.removeEventListener(type, entry.listener);
      entry.listener.call(this, event);
    }
  }

  click() { this.dispatch("click"); }
  focus() {}
  blur() {}
  scrollIntoView() {}
  getBoundingClientRect() { return { left: 0, top: 0, width: 0, height: 0, right: 0, bottom: 0 }; }

  matches(selector) {
    if (selector.startsWith(".")) return this.classList.contains(selector.slice(1));
    return this.tagName === selector.toUpperCase();
  }

  descendants() {
    const out = [];
    const walk = (node) => node.children.forEach((child) => { out.push(child); walk(child); });
    walk(this);
    return out;
  }

  querySelectorAll(selector) {
    return this.descendants().filter((node) => node.matches(selector));
  }

  // innerHTML 로 만든 자식은 표식이 들어 있을 때만 그 innerHTML 주인 아래 하나짜리 가짜 요소로 돌려준다.
  querySelector(selector) {
    const real = this.querySelectorAll(selector)[0];
    if (real) return real;
    const marker = selector.startsWith(".") ? selector.slice(1) : `<${selector}`;
    const owner = [this, ...this.descendants()].find((node) => String(node.innerHTML).includes(marker));
    if (!owner) return null;
    if (!owner.placeholders.has(selector)) {
      const element = selector === "video" ? new FakeVideo(this.ownerDocument) : new FakeElement(this.ownerDocument);
      if (selector.startsWith(".")) element.className = selector.slice(1);
      element.parentElement = owner;
      owner.placeholders.set(selector, element);
    }
    return owner.placeholders.get(selector);
  }

  closest(selector) {
    if (selector !== "[data-action]") return null;
    let node = this;
    while (node) {
      if (node.dataset?.action) return node;
      node = node.parentElement;
    }
    return null;
  }

  contains(target) {
    return target === this || this.descendants().includes(target);
  }
}

class FakeVideo extends FakeElement {
  constructor(doc) {
    super(doc, "video");
    this.currentTime = 0;
    this.duration = Number.NaN;
    this.readyState = 0;
    this.paused = true;
    this.ended = false;
    this.playbackRate = 1;
    this.src = "";
    this.srcObject = null;
    this.playCalls = [];
  }

  play() {
    this.playCalls.push(this.currentTime);
    this.paused = false;
    return Promise.resolve();
  }

  pause() { this.paused = true; }
  load() { this.readyState = 0; }
}

function createDocument(options = {}) {
  const doc = {
    ids: new Map(),
    addEventListener() {},
    removeEventListener() {},
    querySelector: () => null,
    querySelectorAll: () => [],
    createDocumentFragment() {
      const fragment = new FakeElement(doc, "fragment");
      fragment.isFragment = true;
      return fragment;
    },
    createElement(tag) {
      return String(tag).toLowerCase() === "video" ? new FakeVideo(doc) : new FakeElement(doc, tag);
    },
    getElementById(id) {
      if (!doc.ids.has(id)) {
        if (!options.autoIds) return null;
        doc.ids.set(id, id === "carrotRoadVideo" ? new FakeVideo(doc) : new FakeElement(doc));
      }
      return doc.ids.get(id);
    },
  };
  doc.body = new FakeElement(doc, "body");
  doc.documentElement = new FakeElement(doc, "html");
  return doc;
}

function loadKoreanStrings() {
  let strings = null;
  vm.runInNewContext(read("js/translations/ko.js"), {
    window: { CarrotTranslations: { register: (_lang, pack) => { strings = pack.strings; } } },
  });
  return strings;
}

const ko = loadKoreanStrings();

function uiText(key, fallback, vars = null) {
  let value = String(ko?.[key] ?? fallback ?? key);
  for (const [name, replacement] of Object.entries(vars || {})) value = value.replaceAll(`{${name}}`, String(replacement));
  return value;
}

const flush = async (rounds = 6) => {
  for (let index = 0; index < rounds; index += 1) await new Promise((resolve) => setImmediate(resolve));
};

function deferred() {
  let resolve;
  let reject;
  const promise = new Promise((res, rej) => { resolve = res; reject = rej; });
  return { promise, resolve, reject };
}

// ---------- 로그 화면(대시캠) 환경 ----------

const ROUTE = "000001aa--b2c3d4e5f6";
const SEGMENTS = [`${ROUTE}--0`, `${ROUTE}--1`, `${ROUTE}--2`];
const logsDocument = createDocument();
const routesHost = new FakeElement(logsDocument);
logsDocument.ids.set("dashcamRoutes", routesHost);
const toasts = [];
const logsWindow = {
  setTimeout: (...args) => setTimeout(...args),
  clearTimeout: (...args) => clearTimeout(...args),
  setInterval: () => 0,
  clearInterval() {},
  requestAnimationFrame: () => 0,
  cancelAnimationFrame() {},
  addEventListener() {},
  removeEventListener() {},
  matchMedia: () => ({ matches: false, addEventListener() {}, removeEventListener() {} }),
  innerWidth: 1280,
  innerHeight: 720,
  open() {},
  scrollTo() {},
  CarrotMediaTransport: {
    createActionButton(kind, options = {}) {
      const button = new FakeElement(logsDocument, "button");
      button.dataset.kind = kind;
      button.disabled = Boolean(options.disabled);
      button.addEventListener("click", () => options.onActivate?.());
      return button;
    },
    setActionState() {},
    createInputController: () => ({ dispose() {} }),
  },
};

Object.assign(globalThis, {
  window: logsWindow,
  document: logsDocument,
  localStorage: { getItem: () => null, setItem() {}, removeItem() {} },
  sessionStorage: { getItem: () => null, setItem() {}, removeItem() {} },
  requestAnimationFrame: () => 0,
  cancelAnimationFrame() {},
  getUIText: uiText,
  escapeHtml: (value) => String(value ?? ""),
  showAppToast: (message, options = {}) => { toasts.push({ message: String(message), tone: options.tone || "" }); },
  openAppDialog: () => Promise.resolve(null),
  getJson: () => Promise.reject(new Error("offline")),
  postJson: () => Promise.reject(new Error("offline")),
  fetch: () => Promise.reject(new Error("offline")),
  isCompactLandscapeMode: () => false,
  showPage() {},
  CURRENT_PAGE: "logs",
  LANG: "ko",
});

const dashcam = await import("../src/features/logs/dashcam.js");
const logsRuntime = await import("../src/features/logs/runtime.js");
logsRuntime.initLogsPage();
await flush();

function resetLogs() {
  logsDocument.body.replaceChildren();
  toasts.length = 0;
  dashcam.dashcamState.routes = [{
    route: ROUTE,
    segmentFolders: [...SEGMENTS],
    segmentTimes: {},
    segmentCount: SEGMENTS.length,
  }];
}

function installReplay(startImpl) {
  const calls = [];
  logsWindow.CarrotVisionReplay = {
    start: (options) => {
      calls.push(options);
      return startImpl(options, calls.length);
    },
    isActive: () => false,
    updateSegments() {},
  };
  return calls;
}

function openPlayers() {
  return logsDocument.body.children.filter((node) => node.classList.contains("dashcam-player-overlay"));
}

function playerTitle(overlay) {
  return overlay.descendants().map((node) => node.innerHTML).join("\n");
}

function clickTile(segment) {
  const tile = new FakeElement(logsDocument);
  tile.dataset = { action: "play", route: ROUTE, segment };
  routesHost.dispatch("click", { target: tile });
}

const segmentTitle = (segment) => `${ROUTE.replace(/^0+(?=\d{3})/, "")} · ${uiText("segment_label", "Segment")} ${segment.split("--").pop()}`;

test("영상 칸을 누르면 일반 재생기 대신 리플레이가 그 구간으로 열린다", async () => {
  resetLogs();
  const calls = installReplay(() => Promise.resolve(true));
  clickTile(SEGMENTS[1]);
  await flush();

  assert.equal(calls.length, 1);
  assert.equal(calls[0].route, ROUTE);
  assert.equal(calls[0].segment, SEGMENTS[1]);
  assert.deepEqual(calls[0].segments, SEGMENTS);
  assert.equal(calls[0].startTime, undefined);
  assert.equal(openPlayers().length, 0, "리플레이가 시작되면 일반 재생기를 열지 않는다");
  assert.equal(toasts.length, 0);
});

for (const [name, startImpl, expectsToast] of [
  ["start 가 false 를 돌려줌", () => Promise.resolve(false), false],
  ["start 가 비동기로 오류를 던짐", () => Promise.reject(new Error("rlog 없음")), true],
  ["start 가 곧바로 오류를 던짐", () => { throw new Error("화면 요소 없음"); }, true],
]) {
  test(`리플레이 시작 실패(${name})는 같은 구간을 일반 재생기로 연다`, async () => {
    resetLogs();
    installReplay(startImpl);
    clickTile(SEGMENTS[2]);
    await flush();

    const players = openPlayers();
    assert.equal(players.length, 1);
    assert.match(playerTitle(players[0]), new RegExp(segmentTitle(SEGMENTS[2])));
    assert.equal(toasts.some((toast) => toast.tone === "error"), expectsToast);
  });
}

test("리플레이를 못 쓰는 브라우저(start 없음)는 일반 재생기로 연다", async () => {
  resetLogs();
  logsWindow.CarrotVisionReplay = undefined;
  clickTile(SEGMENTS[0]);
  await flush();

  const players = openPlayers();
  assert.equal(players.length, 1);
  assert.match(playerTitle(players[0]), new RegExp(segmentTitle(SEGMENTS[0])));
  assert.ok(toasts.some((toast) => toast.message === uiText("replay_unavailable", "")));
});

test("사용자가 닫았거나 교체된 리플레이(start 가 undefined)는 일반 재생기를 열지 않는다", async () => {
  resetLogs();
  installReplay(() => Promise.resolve(undefined));
  clickTile(SEGMENTS[1]);
  await flush();
  assert.equal(openPlayers().length, 0);
});

test("연속 선택에서는 이전 요청의 늦은 실패를 무시하고 최신 요청의 실패만 일반 재생으로 연다", async () => {
  resetLogs();
  const pending = [deferred(), deferred(), deferred()];
  const calls = installReplay((_options, order) => pending[order - 1].promise);
  clickTile(SEGMENTS[0]);
  clickTile(SEGMENTS[1]);
  clickTile(SEGMENTS[2]);
  await flush();
  assert.deepEqual(calls.map((call) => call.segment), SEGMENTS);

  pending[0].resolve(false);
  pending[1].reject(new Error("늦은 오류"));
  await flush();
  assert.equal(openPlayers().length, 0, "밀려난 요청의 늦은 실패는 화면을 열지 않는다");
  assert.equal(toasts.length, 0);

  pending[2].resolve(false);
  await flush();
  const players = openPlayers();
  assert.equal(players.length, 1);
  assert.match(playerTitle(players[0]), new RegExp(segmentTitle(SEGMENTS[2])));
});

test("구간 메뉴의 재생은 리플레이를 거치지 않고 일반 재생기를 연다", () => {
  const source = read("src/features/logs/dashcam.js");
  assert.match(source, /selected === "play" && typeof options\.activePlayerClose !== "function"\) openDashcamPlayer\(route, segment\)/);
});

async function openAnalyzablePlayer() {
  dashcam.openDashcamPlayer(ROUTE, SEGMENTS[0]);
  const [overlay] = openPlayers();
  assert.ok(overlay, "일반 재생기가 열려야 한다");
  const analyze = overlay.querySelector(".dashcam-player-analyze");
  assert.ok(analyze, "위쪽 단추 줄에 분석 단추가 있어야 한다");
  assert.match(playerTitle(overlay), /<div class="dashcam-player-top">[\s\S]*class="[^"]*dashcam-player-analyze" type="button" aria-label="분석" title="분석"/);
  const video = overlay.querySelector("video");
  const selectSegment = async (segment) => {
    const button = overlay.descendants().find((node) => node.dataset?.segmentId === segment);
    assert.ok(button, `구간 목록에 ${segment} 이 있어야 한다`);
    button.click();
    await flush();
  };
  return { overlay, analyze, video, selectSegment };
}

test("분석 단추는 누른 순간의 구간과 구간 안 초로 리플레이를 열고 일반 재생기를 닫는다", async () => {
  resetLogs();
  const pending = deferred();
  const calls = installReplay(() => pending.promise);
  const { overlay, analyze, video, selectSegment } = await openAnalyzablePlayer();

  await selectSegment(SEGMENTS[2]);
  video.currentTime = 7.25;
  video.paused = false;
  analyze.click();
  await flush();

  assert.equal(calls.length, 1);
  assert.equal(calls[0].segment, SEGMENTS[2], "처음 연 구간이 아니라 지금 보던 구간이어야 한다");
  assert.equal(calls[0].startTime, 7.25);
  assert.equal(video.paused, true, "리플레이와 이중 재생하지 않게 일반 재생을 멈춘다");
  assert.equal(overlay.style.display, "none", "리플레이가 화면을 넘겨받는 동안 일반 재생기는 가려진다");

  pending.resolve(true);
  await flush();
  assert.equal(overlay.isConnected, false, "리플레이가 시작되면 일반 재생기를 닫는다");
});

test("분석 진입 실패(false·오류)는 같은 구간·위치에 멈춘 일반 재생기로 되돌리고 다시 재생하지 않는다", async () => {
  for (const failure of [() => Promise.resolve(false), () => Promise.reject(new Error("rlog 없음"))]) {
    resetLogs();
    const calls = installReplay(failure);
    const { overlay, analyze, video, selectSegment } = await openAnalyzablePlayer();
    await selectSegment(SEGMENTS[1]);
    video.currentTime = 31.5;
    video.paused = false;
    const playsBefore = video.playCalls.length;

    analyze.click();
    await flush();
    assert.equal(overlay.isConnected, true);
    assert.equal(overlay.style.display, "");
    assert.equal(video.currentTime, 31.5);
    assert.equal(video.paused, true);
    assert.equal(video.playCalls.length, playsBefore, "실패 뒤 일반 재생을 다시 시작하지 않는다");
    assert.equal(openPlayers().length, 1, "일반 재생기를 하나 더 열지 않는다");

    analyze.click();
    await flush();
    assert.deepEqual(calls.map((call) => [call.segment, call.startTime]), [
      [SEGMENTS[1], 31.5],
      [SEGMENTS[1], 31.5],
    ], "되돌린 뒤에도 같은 구간·위치를 쓴다");
  }
});

test("분석 진입 중 사용자가 리플레이를 닫으면(취소) 일반 재생기도 다시 나타나지 않는다", async () => {
  resetLogs();
  installReplay(() => Promise.resolve(undefined));
  const { overlay, analyze } = await openAnalyzablePlayer();
  analyze.click();
  await flush();
  assert.equal(overlay.isConnected, false);
  assert.equal(openPlayers().length, 0);
});

// ---------- 리플레이 start 의 시작 시각과 취소 ----------

function loadVisionReplay(options = {}) {
  const doc = createDocument({ autoIds: true });
  const applied = [];
  const replayToasts = [];
  let transportOptions = null;
  const records = [0, 10_000, 20_000, 25_000].map((timeMs) => ({
    timeMs,
    frames: [{ service: "carState", decoded: { vEgo: timeMs / 1000 } }],
  }));
  const context = {
    document: doc,
    AbortController,
    DOMException,
    performance,
    Element: FakeElement,
    setTimeout,
    clearTimeout,
    requestAnimationFrame: () => 0,
    cancelAnimationFrame() {},
    getUIText: uiText,
    showPage() {},
    showAppToast: (message) => { replayToasts.push(String(message)); },
    addEventListener() {},
    CarrotVisionSyncAvailability: options.syncAvailability,
    CarrotMediaTransport: {
      create: (transportOpts) => {
        transportOptions = transportOpts;
        return {
          sync() {}, reset() {}, syncLabels() {}, setFallbackThumbnail() {}, setRate() {},
          captureFallbackFrame() {}, generateFilmstrip() {},
        };
      },
    },
    CarrotVisionRaw: {
      applyCompactFrames: (frames) => { applied.push(frames.map((frame) => frame.decoded.vEgo)); return frames.length; },
      clearState() {},
    },
    CarrotReplayClient: {
      capabilities: () => ({
        client: typeof options.client === "function" ? options.client() : options.client !== false,
        mode: "test",
      }),
      fetchSource: () => options.fetchSource?.() || Promise.resolve({}),
      start: () => ({
        video: { complete: options.complete || new Promise(() => {}), urlPromise: Promise.resolve("blob:replay") },
        timelinePromise: Promise.resolve({ records, durationMs: 30_000 }),
        session: { dispose() {} },
        profile: { mode: "test" },
      }),
    },
  };
  context.window = context;
  vm.runInNewContext(read("js/realtime/vision_replay.js"), context);
  return {
    replay: context.CarrotVisionReplay,
    video: doc.getElementById("carrotRoadVideo"),
    applied,
    toasts: replayToasts,
    transport: transportOptions,
  };
}

// start 가 끝났는지 표시해 두고 결과를 볼 수 있게 한다.
function track(promise) {
  const tracked = { settled: false, value: undefined };
  tracked.promise = Promise.resolve(promise).then((value) => {
    tracked.settled = true;
    tracked.value = value;
    return value;
  });
  return tracked;
}

const waitListenerTypes = ["loadedmetadata", "seeked", "loadeddata", "canplay", "error"];
const waitListenerCount = (video) => video.listeners.filter((entry) => waitListenerTypes.includes(entry.type)).length;

// 영상 길이를 알린 뒤, 요청 위치에서 화면을 그릴 수 있게 됐음을 알린다.
async function makeVideoReady(video, duration = 30) {
  video.duration = duration;
  video.readyState = 1;
  video.dispatch("loadedmetadata");
  await flush();
  video.readyState = 2;
  video.dispatch("seeked");
  await flush();
}

test("리플레이 start 는 요청 위치(유효 범위로 제한)에서 영상이 준비되고 기록도 적용된 뒤에만 성공하고 한 번만 재생한다", async () => {
  for (const [startTime, expected, lastSpeed] of [[12.5, 12.5, 10], [99, 30, 25]]) {
    const { replay, video, applied } = loadVisionReplay();
    const baseline = waitListenerCount(video);
    const started = track(replay.start({ route: ROUTE, segment: SEGMENTS[1], startTime }));
    await flush();
    assert.equal(started.settled, false, "영상 길이를 알기 전에는 성공으로 끝내지 않는다");

    video.duration = 30;
    video.readyState = 1;
    video.dispatch("loadedmetadata");
    await flush();
    assert.equal(video.currentTime, expected, "영상 길이를 안 뒤 유효 범위로 제한한 위치로 옮긴다");
    assert.equal(started.settled, false, "영상 길이 정보만으로는 재생 준비 완료가 아니다");
    assert.deepEqual(video.playCalls, [], "준비 전에는 재생하지 않는다");

    video.readyState = 2;
    video.dispatch("seeked");
    await flush();
    assert.equal(started.settled, true);
    assert.equal(started.value, true);
    assert.deepEqual(video.playCalls, [expected], "옮긴 위치에서 한 번만 재생한다");
    assert.equal(applied.at(-1).at(-1), lastSpeed, "기록도 같은 위치까지 적용한다");
    assert.equal(waitListenerCount(video), baseline, "준비 대기가 남지 않는다");
  }
});

test("처음 위치 준비 중 탐색(12.5초 요청 → 준비 중 20초)은 막혀 영상과 기록이 같은 위치에서 성공하고, 닫기는 그대로 된다", async () => {
  const { replay, video, applied, transport } = loadVisionReplay();
  const started = track(replay.start({ route: ROUTE, segment: SEGMENTS[1], startTime: 12.5 }));
  await flush();
  video.duration = 30;
  video.readyState = 1;
  video.dispatch("loadedmetadata");
  await flush();
  assert.equal(video.currentTime, 12.5);
  assert.equal(started.settled, false);

  // 준비 중 사용자가 20초로 옮긴다.
  const interactiveWhilePreparing = transport.canInteract();
  const seekWhilePreparing = replay.diagnosticPlayback.seek(20);

  video.readyState = 2;
  video.dispatch("seeked");
  await flush();
  assert.equal(started.value, true);
  assert.equal(video.currentTime, 12.5, "성공했을 때 영상은 요청한 12.5초에 있다(20초로 옮겨져 있지 않다)");
  assert.equal(applied.at(-1).at(-1), 10, "기록도 영상과 같은 12.5초 위치다");
  assert.deepEqual(video.playCalls, [12.5]);
  assert.equal(seekWhilePreparing, false, "준비 중 20초 탐색은 받지 않는다");
  assert.equal(interactiveWhilePreparing, false, "준비 중에는 탐색 막대·재생 단추가 동작하지 않는다");

  assert.equal(transport.canInteract(), true, "준비가 끝나면 다시 탐색할 수 있다");
  assert.equal(replay.diagnosticPlayback.seek(20), true);
  assert.equal(video.currentTime, 20);
  assert.equal(applied.at(-1).at(-1), 20);

  const closed = loadVisionReplay();
  const closing = track(closed.replay.start({ route: ROUTE, segment: SEGMENTS[1], startTime: 12.5 }));
  await flush();
  closed.video.duration = 30;
  closed.video.readyState = 1;
  closed.video.dispatch("loadedmetadata");
  await flush();
  await closed.replay.stop({ returnToLogs: true });
  await flush();
  assert.equal(closing.settled, true);
  assert.equal(closing.value, undefined, "준비 중 닫기는 그대로 취소로 끝난다");
  assert.equal(closed.replay.isActive(), false);
});

test("시작 시각이 없거나 0이어도 영상이 준비된 뒤에 처음부터 성공하고, 준비 전에 닫히면 재생하지 않는다", async () => {
  for (const startTime of [undefined, 0]) {
    const plain = loadVisionReplay();
    const started = track(plain.replay.start({ route: ROUTE, segment: SEGMENTS[0], startTime }));
    await flush();
    plain.video.duration = 30;
    plain.video.readyState = 1;
    plain.video.dispatch("loadedmetadata");
    await flush();
    assert.equal(started.settled, false, "시작 시각 0도 영상 길이 정보만으로 성공하지 않는다");
    assert.deepEqual(plain.video.playCalls, []);
    plain.video.readyState = 2;
    plain.video.dispatch("seeked");
    await flush();
    assert.equal(started.value, true);
    assert.deepEqual(plain.video.playCalls, [0]);
    assert.equal(plain.applied.at(-1).at(-1), 0);
  }

  const closed = loadVisionReplay();
  const baseline = waitListenerCount(closed.video);
  const closing = closed.replay.start({ route: ROUTE, segment: SEGMENTS[0], startTime: 5 });
  await flush();
  await closed.replay.stop({ returnToLogs: true });
  assert.equal(await closing, undefined, "준비 전 닫기는 취소다");
  assert.equal(waitListenerCount(closed.video), baseline, "닫으면 준비 대기가 남지 않는다");
  await makeVideoReady(closed.video);
  assert.deepEqual(closed.video.playCalls, []);
});

test("리플레이 start 는 실제 실패에만 false 이고, 닫힘·교체는 false 가 아니다", async () => {
  const unsupported = loadVisionReplay({ client: false });
  assert.equal(await unsupported.replay.start({ route: ROUTE, segment: SEGMENTS[0] }), false);

  const source = deferred();
  const closedWhileLoading = loadVisionReplay({ fetchSource: () => source.promise });
  const loading = closedWhileLoading.replay.start({ route: ROUTE, segment: SEGMENTS[0] });
  await closedWhileLoading.replay.stop({ returnToLogs: true });
  source.resolve({});
  assert.equal(await loading, undefined);

  const replaced = loadVisionReplay();
  const first = replaced.replay.start({ route: ROUTE, segment: SEGMENTS[0] });
  await flush();
  await makeVideoReady(replaced.video);
  assert.equal(await first, true);
  const replacing = replaced.replay.start({ route: ROUTE, segment: SEGMENTS[1] });
  replaced.replay.stop({ returnToLogs: true });
  assert.equal(await replacing, undefined);
});

test("준비 전 영상 오류·영상 처리 실패는 false 이고, 준비 중 닫기·교체는 취소(undefined)이며 대기가 남지 않는다", async () => {
  for (const stage of ["before-metadata", "while-seeking"]) {
    const { replay, video, toasts: replayToasts } = loadVisionReplay();
    const baseline = waitListenerCount(video);
    const started = track(replay.start({ route: ROUTE, segment: SEGMENTS[0], startTime: 4 }));
    await flush();
    if (stage === "while-seeking") {
      video.duration = 30;
      video.readyState = 1;
      video.dispatch("loadedmetadata");
      await flush();
    }
    video.dispatch("error");
    await flush();
    assert.equal(started.value, false, `${stage} 영상 오류는 실패다`);
    assert.equal(replay.isActive(), false);
    assert.equal(replayToasts.length, 1);
    assert.equal(waitListenerCount(video), baseline);
    assert.deepEqual(video.playCalls, []);
  }

  const completion = deferred();
  const failedPipeline = loadVisionReplay({ complete: completion.promise });
  const pipelineStart = track(failedPipeline.replay.start({ route: ROUTE, segment: SEGMENTS[0] }));
  await flush();
  completion.reject(new Error("영상 변환 실패"));
  await flush();
  assert.equal(pipelineStart.value, false, "준비 전 영상 처리 실패도 실패다");

  const replacedWhileWaiting = loadVisionReplay();
  const baseline = waitListenerCount(replacedWhileWaiting.video);
  const waiting = track(replacedWhileWaiting.replay.start({ route: ROUTE, segment: SEGMENTS[0], startTime: 4 }));
  await flush();
  const next = track(replacedWhileWaiting.replay.start({ route: ROUTE, segment: SEGMENTS[1] }));
  await flush();
  assert.equal(waiting.settled, true);
  assert.equal(waiting.value, undefined, "준비 중 다른 영상으로 바꾸면 취소다");
  await makeVideoReady(replacedWhileWaiting.video);
  assert.equal(next.value, true);
  assert.deepEqual(replacedWhileWaiting.video.playCalls, [0], "바뀐 영상만 한 번 재생한다");
  assert.equal(waitListenerCount(replacedWhileWaiting.video), baseline);
});

test("실패 정리(stop)를 기다리는 동안 닫거나 다른 영상으로 바꾸면 실패가 아닌 취소로 끝난다", async () => {
  for (const action of ["close", "replace"]) {
    const availability = deferred();
    let clientReady = false;
    const loaded = loadVisionReplay({ client: () => clientReady, syncAvailability: () => availability.promise });
    const failing = track(loaded.replay.start({ route: ROUTE, segment: SEGMENTS[0] }));
    await flush();
    assert.equal(failing.settled, false, "실패 정리가 아직 끝나지 않았다");

    let next = null;
    if (action === "close") {
      await loaded.replay.stop({ returnToLogs: true });
    } else {
      clientReady = true;
      next = track(loaded.replay.start({ route: ROUTE, segment: SEGMENTS[1] }));
    }
    availability.resolve(true);
    await flush();
    assert.equal(failing.settled, true);
    assert.equal(failing.value, undefined, `${action} 은 취소다`);
    assert.deepEqual(loaded.toasts, [], "취소된 요청은 실패 알림을 띄우지 않는다");
    if (next) {
      await makeVideoReady(loaded.video);
      assert.equal(next.value, true);
      assert.equal(loaded.replay.isActive(), true);
    }
  }
});

// ---------- 로그 화면과 실제 리플레이를 이어서 ----------

async function waitTimers() {
  await new Promise((resolve) => setTimeout(resolve, 5));
  await flush();
}

test("분석 전환 중이나 실패 복구 뒤에 늦게 온 canplay 는 숨긴/되돌린 일반 영상을 재생하지 않는다", async () => {
  for (const timing of ["예약 뒤", "예약 전"]) {
    resetLogs();
    const pending = deferred();
    installReplay(() => pending.promise);
    const { overlay, analyze, video, selectSegment } = await openAnalyzablePlayer();
    await selectSegment(SEGMENTS[1]);
    if (timing === "예약 뒤") {
      await waitTimers();
      assert.ok(video.listeners.some((entry) => entry.type === "canplay"), "구간 변경이 canplay 재생을 예약했다");
    }
    const playsBefore = video.playCalls.length;
    analyze.click();
    await flush();
    await waitTimers();
    assert.equal(overlay.style.display, "none");
    assert.equal(video.listeners.some((entry) => entry.type === "canplay"), false, "전환할 때 예약을 지운다");
    video.readyState = 2;
    video.dispatch("canplay");
    await flush();
    assert.equal(video.playCalls.length, playsBefore, "전환 중 늦은 canplay 는 재생하지 않는다");

    pending.resolve(false);
    await flush();
    assert.equal(overlay.style.display, "", "실패하면 일반 재생기를 되돌린다");
    video.dispatch("canplay");
    await waitTimers();
    assert.equal(video.playCalls.length, playsBefore, "실패 복구 뒤 늦은 canplay 도 재생하지 않는다");
    assert.equal(video.paused, true);
  }
});

function useRealReplay(options = {}) {
  const loaded = loadVisionReplay(options);
  logsWindow.CarrotVisionReplay = loaded.replay;
  return loaded;
}

test("분석 진입: 준비 전 영상 오류는 실패로 일반 재생기를 되돌리고, 준비 중 닫기·바꾸기는 취소로 닫는다", async () => {
  resetLogs();
  let loaded = useRealReplay();
  let player = await openAnalyzablePlayer();
  player.analyze.click();
  await flush();
  assert.equal(player.overlay.style.display, "none");
  loaded.video.dispatch("error");
  await flush();
  assert.equal(player.overlay.isConnected, true, "준비 전 영상 오류는 실패라 일반 재생기가 돌아온다");
  assert.equal(player.overlay.style.display, "");
  assert.equal(openPlayers().length, 1);

  resetLogs();
  loaded = useRealReplay();
  player = await openAnalyzablePlayer();
  player.analyze.click();
  await flush();
  await loaded.replay.stop({ returnToLogs: true });
  await flush();
  assert.equal(player.overlay.isConnected, false, "준비 중 닫기는 취소라 일반 재생기를 닫는다");
  assert.equal(openPlayers().length, 0);

  resetLogs();
  loaded = useRealReplay();
  player = await openAnalyzablePlayer();
  player.analyze.click();
  await flush();
  clickTile(SEGMENTS[2]);
  await flush();
  assert.equal(player.overlay.isConnected, false, "준비 중 다른 영상으로 바꾸면 취소라 일반 재생기를 닫는다");
  await makeVideoReady(loaded.video);
  assert.equal(loaded.replay.isActive(), true);
  assert.equal(openPlayers().length, 0, "바뀐 리플레이가 시작돼도 일반 재생기는 다시 안 열린다");
});

test("실패 정리 중 닫기·바꾸기면 영상 칸과 분석 단추 어느 쪽도 일반 재생기를 다시 열지 않는다", async () => {
  for (const entry of ["tile", "analyze"]) {
    for (const action of ["close", "replace"]) {
      resetLogs();
      const availability = deferred();
      let clientReady = false;
      const loaded = useRealReplay({ client: () => clientReady, syncAvailability: () => availability.promise });
      let player = null;
      if (entry === "tile") {
        clickTile(SEGMENTS[0]);
      } else {
        player = await openAnalyzablePlayer();
        player.analyze.click();
      }
      await flush();
      if (action === "close") {
        await loaded.replay.stop({ returnToLogs: true });
      } else {
        clientReady = true;
        clickTile(SEGMENTS[1]);
        await flush();
      }
      availability.resolve(true);
      await flush();
      if (action === "replace") await makeVideoReady(loaded.video);
      assert.equal(openPlayers().length, 0, `${entry}·${action}: 일반 재생기를 다시 열지 않는다`);
      if (player) assert.equal(player.overlay.isConnected, false);
      assert.deepEqual(loaded.toasts, [], `${entry}·${action}: 실패 알림이 없다`);
    }
  }
});

// ---------- 그래프 묶음: 명령 가속 줄과 저크 줄 ----------

function loadInsightsData() {
  const context = { setTimeout };
  context.window = context;
  vm.runInNewContext(read("src/features/replay/data.js"), context);
  return context.CarrotReplayInsightsData;
}

const data = loadInsightsData();
// vm 안에서 만든 배열·객체는 프로토타입이 달라 평범한 값으로 바꿔 비교한다.
const plain = (value) => JSON.parse(JSON.stringify(value));
const approx = (actual, expected) => assert.ok(Math.abs(actual - expected) < 1e-9, `${actual} ≈ ${expected}`);
const coordinates = (path) => [...String(path).matchAll(/([ML])(-?[\d.]+),(-?[\d.]+)/g)]
  .map((match) => ({ op: match[1], x: Number(match[2]), y: Number(match[3]) }));

test("그래프 묶음에 carControl.actuators.accel 명령 가속 줄이 생기고 빠진 값은 0으로 채우지 않는다", async () => {
  const frames = {
    0: [{ service: "carState", decoded: { aEgo: 0 } }, { service: "carControl", decoded: { actuators: { accel: -0.5 } } }],
    100: [{ service: "carState", decoded: { aEgo: 0.5 } }, { service: "carControl", decoded: { actuators: {} } }],
    200: [{ service: "carState", decoded: { aEgo: 0.2 } }, { service: "carControl", decoded: { latActive: true } }],
    300: [{ service: "carState", decoded: { aEgo: 0.8 } }, { service: "carControl", decoded: { actuators: { accel: 0.25 } } }],
  };
  const series = await data.buildSummarySeries({
    records: Object.keys(frames).map((timeMs) => ({ timeMs: Number(timeMs) })),
    decodeRecord: (record) => frames[record.timeMs],
  });

  assert.deepEqual(plain(series.command), [
    { timeMs: 0, value: -0.5 },
    { timeMs: 300, value: 0.25 },
  ]);
  assert.deepEqual(plain(series.accel.map((sample) => sample.value)), [0, 0.5, 0.2, 0.8]);

  const graph = data.accelerationGraph({ actual: series.accel, command: series.command, durationSeconds: 1 });
  assert.ok(graph.commandPath.startsWith("M"), "명령 가속 줄이 그려진다");
  assert.ok(graph.jerkPath.startsWith("M"), "저크 줄이 그려진다");
  assert.deepEqual(plain(graph.jerk.map((sample) => sample.timeMs)), [100, 200, 300]);
  [5, -3, 6].forEach((expected, index) => approx(graph.jerk[index].value, expected));
});

test("저크는 최종 실제 가속 점끼리 (현재−이전)/(밀리초 차÷1000)이고 300ms 는 계산, 300ms 초과·0 이하·무효는 빈칸이다", () => {
  const points = [
    { timeMs: 0, value: 0 },
    { timeMs: 100, value: 0.5 },
    { timeMs: 400, value: 0.2 },
    { timeMs: 701, value: 1.0 },
    { timeMs: 701, value: 1.4 },
    { timeMs: 800, value: 1.5 },
    { timeMs: 900, value: Number.NaN },
    { timeMs: 1000, value: 2.0 },
    { timeMs: 1100, value: 1.0 },
  ];
  const jerk = data.jerkSeries(points);

  assert.deepEqual(plain(jerk.map((sample) => [sample.startMs, sample.timeMs])), [
    [0, 100],
    [100, 400],
    [701, 800],
    [1000, 1100],
  ], "첫 점·301ms·시간 차 0·무효 값 구간은 계산하지 않고, 값은 뒤쪽 점 시각에 둔다");
  approx(jerk[0].value, 5);
  approx(jerk[1].value, (0.2 - 0.5) / 0.3);
  approx(jerk[2].value, (1.5 - 1.4) / 0.099);
  approx(jerk[3].value, -10);
  assert.equal(data.POLICY.graphGapMs, 300);

  const graph = data.accelerationGraph({ actual: points, command: [], durationSeconds: 2 });
  assert.deepEqual(coordinates(graph.jerkPath).map((point) => point.op), ["M", "L", "M", "M"],
    "빈 구간을 가로질러 잇지 않고 새 조각으로 끊는다");

  assert.equal(data.spanningSample(jerk, 250).value, jerk[1].value);
  assert.equal(data.spanningSample(jerk, 550), null, "빈 구간의 현재값은 없음(--)이다");
  assert.equal(data.spanningSample(jerk, 950), null);
  assert.equal(data.spanningSample(jerk, 1200), null);
});

test("실제·명령 가속은 두 값 범위를 합친 같은 세로축을 쓰고 같은 값은 같은 높이에 그린다", () => {
  const actual = [{ timeMs: 0, value: 0.5 }, { timeMs: 100, value: -0.25 }];
  const command = [
    { timeMs: 0, value: 0.5 },
    { timeMs: 100, value: 2.0 },
    { timeMs: 400, value: -0.25 },
    { timeMs: 701, value: 0.5 },
  ];
  const graph = data.accelerationGraph({ actual, command, durationSeconds: 1 });
  assert.deepEqual(plain(graph.range), { min: -2, max: 2 }, "명령 가속 범위까지 합친 축이다");

  const actualPoints = graph.actualPoints.split(" ").map((pair) => pair.split(",").map(Number));
  const commandPoints = coordinates(graph.commandPath);
  assert.equal(actualPoints[0][1], commandPoints[0].y, "같은 0.5 는 같은 높이다");
  assert.equal(actualPoints[1][1], commandPoints[2].y, "같은 -0.25 는 같은 높이다");
  assert.deepEqual(commandPoints.map((point) => point.op), ["M", "L", "L", "M"],
    "명령 가속 줄도 정확히 300ms 는 잇고 300ms 초과는 끊는다");

  assert.equal(data.gapAwareSample(command, 250).value, 2.0);
  assert.equal(data.gapAwareSample(command, 550), null, "빈 구간의 명령 가속 현재값은 없음(--)이다");
});

test("리플레이 그래프 칸에 점선 명령 가속 줄과 가속도 아래 저크 그래프가 있고 글자는 ko.js 에 있다", () => {
  const html = read("index.html");
  const insights = read("src/features/replay/insights.js");
  const css = read("src/features/replay/insights.css");
  const accelIndex = html.indexOf('data-replay-graph="accel"');
  const jerkIndex = html.indexOf('data-replay-graph="jerk"');
  const leadIndex = html.indexOf('data-replay-graph="lead"');
  assert.ok(accelIndex > 0 && accelIndex < jerkIndex && jerkIndex < leadIndex, "저크 그래프는 가속도 바로 아래다");
  assert.match(html.slice(accelIndex, jerkIndex), /<path class="carrot-replay-insights__commandLine"/);
  assert.match(html.slice(jerkIndex, leadIndex), /m\/s³/);
  assert.match(css, /\.carrot-replay-insights__commandLine \{ stroke-dasharray:/);
  assert.match(insights, /dataTracks\?\.accelerationGraph\?\.\(/);
  assert.match(insights, /name === "jerk"\s*\?\s*dataTracks\?\.spanningSample/);
  assert.equal(ko.replay_graph_jerk, "저크");
  assert.equal(ko.replay_graph_command_accel, "명령");
  assert.equal(ko.dashcam_analyze, "분석");
});
