import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import test from "node:test";
import vm from "node:vm";

const root = new URL("../", import.meta.url);
const settings = readFileSync(new URL("js/pages/setting.js", root), "utf8");
const catalog = JSON.parse(readFileSync(new URL("../../carrot_settings.json", root), "utf8"));

function functionSource(text, name) {
  const start = text.search(new RegExp(`^(?:async )?function ${name}\\(`, "m"));
  assert.notEqual(start, -1);
  const end = text.indexOf("\n}", start);
  return text.slice(start, end + 2);
}

function createSettings(language = "ko") {
  const byName = new Map(catalog.params.map((item) => [item.name, item]));
  const context = vm.createContext({
    LANG: language,
    findSettingItemByName: (name) => (byName.has(name) ? { item: byName.get(name) } : null),
    getSoundLanguageSettingOptions: () => [],
    getSoundLanguageSettingOptionLabel: (value) => String(value),
  });
  vm.runInContext([
    functionSource(settings, "getSettingOptionValues"),
    functionSource(settings, "getDeclaredSettingOptionLabel"),
  ].join("\n"), context);
  return context;
}

test("camera view list shows web vision right below the wide camera", () => {
  const context = createSettings("ko");
  const values = context.getSettingOptionValues("ClusterHudCameraViewMode", { min: 0, max: 5 });
  assert.deepEqual([...values], [0, 1, 2, 3, 5, 4]);
  assert.deepEqual([...values].map((value) => context.getDeclaredSettingOptionLabel("ClusterHudCameraViewMode", value)),
    ["기본", "하단 뷰", "일반 카메라", "광각 카메라", "당근웹비전", "속도 자동"]);
});

test("stored camera view values keep their labels in every language", () => {
  for (const [language, labels] of Object.entries({
    ko: ["기본", "하단 뷰", "일반 카메라", "광각 카메라", "속도 자동", "당근웹비전"],
    en: ["Default", "Bottom view", "Narrow camera", "Wide camera", "Speed automatic", "당근웹비전"],
    zh: ["Default", "Bottom view", "Narrow camera", "Wide camera", "Speed automatic", "당근웹비전"],
  })) {
    const context = createSettings(language);
    assert.deepEqual([0, 1, 2, 3, 4, 5].map((value) => context.getDeclaredSettingOptionLabel("ClusterHudCameraViewMode", value)), labels);
  }
});

test("other select settings keep their ascending order", () => {
  const context = createSettings("ko");
  for (const item of catalog.params) {
    if (item.name === "ClusterHudCameraViewMode" || !Number.isInteger(item.min) || !Number.isInteger(item.max)) continue;
    const expected = [];
    for (let value = item.min; value <= item.max; value += 1) expected.push(value);
    assert.deepEqual([...context.getSettingOptionValues(item.name, { min: item.min, max: item.max })], expected, item.name);
  }
});

test("an option order that is not a permutation of the range is ignored", () => {
  const byName = new Map([["X", { item: { name: "X", option_order: [0, 2, 9] } }]]);
  const context = vm.createContext({ findSettingItemByName: (name) => byName.get(name) || null });
  vm.runInContext(functionSource(settings, "getSettingOptionValues"), context);
  assert.deepEqual([...context.getSettingOptionValues("X", { min: 0, max: 2 })], [0, 1, 2]);
});
