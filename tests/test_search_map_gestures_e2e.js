#!/usr/bin/env node
"use strict";
/*
 * Integration test for the WebUI map gestures: instantiates the real
 * SearchMapRenderer with a minimal fake DOM (no browser needed), then fires
 * wheel / drag / double-click events exactly like a user would and checks the
 * SVG viewBox reacts (zoom-in, pan, reset-to-fit).
 */
const fs = require("fs");
const path = require("path");
const vm = require("vm");

const srcPath = path.join(__dirname, "..", "app", "manual_web_demo", "static", "search_map.js");
const source = fs.readFileSync(srcPath, "utf8");

let failures = 0;
function ok(cond, msg) {
  if (!cond) { console.error("  FAIL: " + msg); failures += 1; }
  else { console.log("  ok: " + msg); }
}
function parseVB(str) {
  const p = String(str || "").trim().split(/\s+/).map(Number);
  return { minX: p[0], minY: p[1], w: p[2], h: p[3] };
}

function makeEl() {
  return {
    attrs: {}, listeners: {}, style: { cursor: "" }, textContent: "", childNodes: [],
    setAttribute(k, v) { this.attrs[k] = String(v); },
    getAttribute(k) { return this.attrs[k]; },
    addEventListener(t, fn) { (this.listeners[t] = this.listeners[t] || []).push(fn); },
    appendChild(c) { this.childNodes.push(c); return c; },
    clientWidth: 600, clientHeight: 400,
    getBoundingClientRect() { return { left: 0, top: 0, width: 600, height: 400 }; },
  };
}
const svg = makeEl();
const documentStub = { createElementNS: () => makeEl() };
const windowObj = {};
const sandbox = { window: windowObj, document: documentStub, console };
vm.createContext(sandbox);
vm.runInContext(source, sandbox, { filename: srcPath });

const R = windowObj.SearchMapRenderer;
if (!R) { console.error("SearchMapRenderer not exposed"); process.exit(1); }
const renderer = new R(svg, null);
renderer.render(null, null); // 空状态渲染 -> 得到初始 fit viewBox

let lastPrevented = false;
function makeEv(extra) { return Object.assign({ preventDefault() { lastPrevented = true; } }, extra || {}); }
function fire(type, ev) {
  lastPrevented = false;
  (svg.listeners[type] || []).forEach(function (fn) { fn(ev); });
}

const fitBox = svg.getAttribute("viewBox");
ok(!!fitBox, "initial viewBox set: " + fitBox);
const fitSpan = parseVB(fitBox);

// ---- wheel zoom in ----
fire("wheel", makeEv({ deltaY: -120, clientX: 300, clientY: 200 }));
let zoomed = parseVB(svg.getAttribute("viewBox"));
ok(lastPrevented === true, "wheel default prevented (page doesn't scroll)");
ok(zoomed.w < fitSpan.w && zoomed.h < fitSpan.h, "wheel-up zoomed in (span smaller)");

// ---- drag pan ----
fire("mousedown", makeEv({ button: 0, clientX: 10, clientY: 10 }));
fire("mousemove", makeEv({ clientX: 40, clientY: 30 })); // 向右下拖
fire("mouseup", makeEv({}));
const afterPan = parseVB(svg.getAttribute("viewBox"));
ok(afterPan.minX < zoomed.minX, "drag right pans content (minX decreased)");
ok(afterPan.minY < zoomed.minY, "drag down pans content (minY decreased)");
ok(svg.style.cursor === "grab", "cursor restored to grab after drag");

// ---- dblclick resets to auto-fit ----
fire("dblclick", makeEv({}));
const resetBox = svg.getAttribute("viewBox");
ok(resetBox === fitBox, "double-click resets to auto-fit viewBox");

console.log(failures === 0 ? "ALL GESTURE E2E TESTS PASSED" : failures + " FAILURES");
process.exit(failures === 0 ? 0 : 1);
