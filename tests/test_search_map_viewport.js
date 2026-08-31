#!/usr/bin/env node
"use strict";
/*
 * Headless unit test for the WebUI map viewport math (wheel zoom / drag pan).
 * Loads search_map.js in a fake browser scope and exercises the pure helpers
 * exposed on window.SvgViewport - no DOM required.
 */
const fs = require("fs");
const path = require("path");
const vm = require("vm");

const srcPath = path.join(__dirname, "..", "app", "manual_web_demo", "static", "search_map.js");
const source = fs.readFileSync(srcPath, "utf8");

let failures = 0;
function ok(cond, msg) {
  if (!cond) {
    console.error("  FAIL: " + msg);
    failures += 1;
  } else {
    console.log("  ok: " + msg);
  }
}

const window = {};
const sandbox = { window, console };
vm.createContext(sandbox);
vm.runInContext(source, sandbox, { filename: srcPath });

const V = window.SvgViewport;
if (!V) { console.error("window.SvgViewport not exposed"); process.exit(1); }

const view = { minX: 0, minY: 0, width: 600, height: 400 };

console.log("wheel zoom:");
const zi = V.zoom(view, 0.5, 300, 200, 600, 400); // 放大 2x, 锚点中心
ok(Math.abs(zi.width - 300) < 1e-9 && Math.abs(zi.height - 200) < 1e-9, "zoom-in halves the span");
ok(Math.abs(zi.minX - 150) < 1e-9 && Math.abs(zi.minY - 100) < 1e-9, "center stays centered after zoom-in");
// 锚点世界坐标不变性
const wx = view.minX + (300 / 600) * view.width;
const wzx = zi.minX + (300 / 600) * zi.width;
ok(Math.abs(wx - wzx) < 1e-6, "cursor world X fixed");
const zo = V.zoom(view, 2, 0, 0, 600, 400); // 缩小，锚点左上角
ok(zo.width > 600 && zo.height > 400, "zoom-out enlarges span");
ok(Math.abs(zo.minX - 0) < 1e-9 && Math.abs(zo.minY - 0) < 1e-9, "corner anchor stays fixed");

console.log("clamp:");
const tiny = V.zoom({ minX: 0, minY: 0, width: 10, height: 10 }, 0.01, 5, 5, 100, 100);
ok(tiny.width >= V.clampSpan(0), "extremely small span clamped");
const huge = V.zoom({ minX: 0, minY: 0, width: 1e9, height: 1e9 }, 100, 5, 5, 100, 100);
ok(huge.width <= 2e6, "extremely large span clamped");
ok(V.clampSpan(100) === 100 && V.clampSpan(0) === 8 && V.clampSpan(-5) === 8, "clampSpan bounds");

console.log("drag pan:");
// 向右拖 100px => 内容右移 => minX 减小
const panned = V.pan(view, 100, 50, 600, 400);
ok(Math.abs(panned.minX - (-100)) < 1e-9, "right drag decreases minX");
ok(Math.abs(panned.minY - (-50)) < 1e-9, "down drag decreases minY");
ok(Math.abs(panned.width - 600) < 1e-9 && Math.abs(panned.height - 400) < 1e-9, "pan preserves span");

console.log("toBoxString:");
ok(V.toBoxString({ minX: 1.2345, minY: 2.3456, width: 600.01, height: 400 }) === "1.23 2.35 600.01 400.00", "box string format");
ok(V.toBoxString(null) === null, "null view -> null");

console.log(failures === 0 ? "ALL VIEWPORT TESTS PASSED" : failures + " FAILURES");
process.exit(failures === 0 ? 0 : 1);
