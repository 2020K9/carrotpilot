import assert from "node:assert/strict";
import test from "node:test";

import { createRoadOverlayPolicy, LANE_MODE_PATH_FILL } from "../src/features/drive/contents/vision/road_overlay_policy.js";

// 대감님 11/23 16:06 "예상경로추종 파란색 장판 깔리는 거 레인모드 활성화 시 연두색으로"
const overlayState = {
  modelV2: { position: { x: [0, 10, 20], y: [0, 0, 0] } },
  lateralPlan: { position: { x: [0, 10, 20], y: [0, 0.1, 0.2] }, useLaneLines: true },
  carControl: { longActive: true },
};
const lane = { lateralPlan: { useLaneLines: true }, controlsState: { activeLaneLine: true } };
const laneless = { lateralPlan: { useLaneLines: false }, controlsState: { activeLaneLine: false } };

test("blue path fill is light lime only while lane mode is active", () => {
  const policy = createRoadOverlayPolicy();
  assert.deepEqual({ ...LANE_MODE_PATH_FILL }, { r: 0x9b, g: 0xe1, b: 0x5d });
  for (const color of [4, 14, 5, 15]) {
    const params = { ShowPathMode: 13, ShowPathColor: color, ShowPathModeLane: 14, ShowPathColorLane: color };
    assert.equal(policy.resolve(overlayState, lane, params).pathStyle.fillColor, LANE_MODE_PATH_FILL);
    assert.equal(policy.resolve(overlayState, laneless, params).pathStyle.fillColor, null);
  }
});

test("other colours and cruise-off colour are unchanged in lane mode", () => {
  const policy = createRoadOverlayPolicy();
  for (const color of [0, 3, 13, 19]) {
    const params = { ShowPathColor: color, ShowPathColorLane: color };
    assert.equal(policy.resolve(overlayState, lane, params).pathStyle.fillColor, null);
  }
  const off = { ...overlayState, carControl: { longActive: false } };
  const style = policy.resolve(off, lane, { ShowPathColorLane: 14, ShowPathColorCruiseOff: 19 }).pathStyle;
  assert.equal(style.colorIndex, 19);
  assert.equal(style.fillColor, null);
});

test("sharp curve: useLaneLines=1 but activeLaneLine=0 (laneless steering) stays blue", () => {
  const policy = createRoadOverlayPolicy();
  const curve = { lateralPlan: { useLaneLines: true }, controlsState: { activeLaneLine: false } };
  for (const color of [4, 14, 5, 15]) {
    const params = { ShowPathMode: 13, ShowPathColor: color, ShowPathModeLane: 14, ShowPathColorLane: color };
    const { selectedPath, pathStyle } = policy.resolve(overlayState, curve, params);
    assert.equal(pathStyle.fillColor, null);
    // path shape/mode selection still follows lateralPlan.useLaneLines (unchanged)
    assert.equal(selectedPath.pathSource, "lateralPlan");
    assert.equal(pathStyle.laneMode, true);
  }
  // activeLaneLine missing -> previous fallback (useLaneLines) decides
  const noActive = { lateralPlan: { useLaneLines: true }, controlsState: {} };
  const params = { ShowPathColor: 4, ShowPathColorLane: 4 };
  assert.equal(policy.resolve(overlayState, noActive, params).pathStyle.fillColor, LANE_MODE_PATH_FILL);
});
