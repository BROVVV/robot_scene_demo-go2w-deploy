#!/usr/bin/env node
"use strict";
/*
 * Headless unit test for the WebUI 3D map framing math (slam_map_3d.js).
 *
 * 回归背景（2026-09-11）：odom 发散把持久化轨迹污染成 ±1e20，写入器据此算出的
 * focus_bounds 撑到 1e19，前端 span≈1.7e20 → scale≈0（地图缩成一个像素 = 看不见）
 * 且 drawGrid 的网格循环要从 -8.7e19 跑到 +8.7e19、步长 5，且浏览器主线程永久卡死。
 * saneFrame 必须挡住这种画幅并退回点云 bounds。
 *
 * 没有 DOM：document.getElementById 返回 null，模块在拿到画幅函数之后立刻 return。
 */
const fs = require("fs");
const path = require("path");
const vm = require("vm");

const srcPath = path.join(__dirname, "..", "app", "manual_web_demo", "static", "slam_map_3d.js");
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
const sandbox = { window, console, document: { getElementById: function () { return null; } } };
vm.createContext(sandbox);
vm.runInContext(source, sandbox, { filename: srcPath });

const F = window.Slam3dFrame;
if (!F) { console.error("window.Slam3dFrame not exposed"); process.exit(1); }

// 真实值：POISONED 是 2026-09-11 快照文件里被污染过的 focus_bounds 原文。
const POISONED = {
  min: [-7.391, -1.4165444806754435e+19, 0.15],
  max: [1.7381901366971282e+20, 7.369829140240715, 2.8]
};
const BOUNDS = {
  min: [-15.264734268188477, -3.4751272201538086, 0.15006108582019806],
  max: [10.138838768005371, 8.800317764282227, 2.521256446838379]
};
const NORMAL_FOCUS = { min: [-7.391, -9.225, 0.15], max: [3.168, 7.37, 2.521] };

console.log("saneFrame:");
ok(F.saneFrame(POISONED) === null, "发散画幅被拒");
ok(F.saneFrame(NORMAL_FOCUS) === NORMAL_FOCUS, "正常画幅通过");
ok(F.saneFrame(null) === null, "null 被拒");
ok(F.saneFrame({ min: [0, 0, 0], max: [1, 1, 1] }) !== null, "完全缺失才拒，零尺寸仍合法");
ok(F.saneFrame({ min: [NaN, 0, 0], max: [1, 1, 1] }) === null, "NaN 被拒");
ok(F.saneFrame({ min: [-Infinity, 0, 0], max: [1, 1, 1] }) === null, "Inf 被拒");

console.log("centerAndSpan —— 污染画幅必须退回点云 bounds:");
const poisonedView = F.centerAndSpan(POISONED, BOUNDS);
ok(poisonedView.span < 100, "span 不再爆炸（实测 " + poisonedView.span.toFixed(2) + "）");
ok(Math.abs(poisonedView.center[0] - (-2.562947750091553)) < 1e-6, "center 回到点云中心");
// drawGrid 的实际迭代次数：这就是当初把浏览器卡死的那个循环
const half = Math.max(2, Math.ceil(poisonedView.span / 2));
const step = poisonedView.span > 25 ? 5 : (poisonedView.span > 10 ? 2 : 1);
const iterations = (2 * Math.ceil(half / step) + 1) * 2;
ok(iterations < 1000, "drawGrid 迭代有界（" + iterations + " 次）");

console.log("centerAndSpan —— 正常画幅仍按 focus 取景（别把功能修没了）:");
const normalView = F.centerAndSpan(NORMAL_FOCUS, BOUNDS);
ok(Math.abs(normalView.span - 16.595) < 1e-6, "span 用 focus_bounds（" + normalView.span.toFixed(3) + "）");
ok(Math.abs(normalView.center[0] - (-2.1115)) < 1e-6, "center 用 focus_bounds");

console.log("centerAndSpan —— 没有 focus 时退回点云 bounds:");
ok(Math.abs(F.centerAndSpan(null, BOUNDS).span - 25.403573036193848) < 1e-6, "span 用 bounds");

console.log("默认视角 pitch —— 重置视角必须回到同一个角度:");
// 2026-09-14：c776eae 把默认 pitch 0.63 改成 1.35（近俯视）时漏改了双击重置，
// 双击会把视角退回"发糊"那个斜角。两处必须一致。
const declared = /var pitch = ([\d.]+);/.exec(source);
const reset = /dblclick[\s\S]{0,200}?pitch = ([\d.]+);/.exec(source);
ok(declared !== null, "能找到 pitch 默认值声明");
ok(reset !== null, "能找到双击重置");
ok(declared && reset && declared[1] === reset[1],
   "双击重置与默认值一致（声明 " + (declared && declared[1]) +
   " / 重置 " + (reset && reset[1]) + "）");

console.log("objectLabel —— canvas 只许读 label_zh（计划书 §7.4）:");
ok(F.objectLabel({ label_zh: "音箱", raw_label: "speaker" }) === "音箱",
   "读的是 label_zh");
// 本文件存在的第二个理由：整改前画的就是 item.label，于是这三个词原样上屏。
ok(F.objectLabel({ raw_label: "barrel" }) === "未分类物体",
   "raw_label=barrel 不会作为可见文字画出");
ok(F.objectLabel({ label: "bin" }) === "未分类物体",
   "老字段 label 也不作为兜底");
ok(F.objectLabel({ label_zh: "桶", raw_label: "barrel" }) === "桶",
   "有中文时原样用");
ok(F.objectLabel({}) === "未分类物体", "缺少中文名时显示未分类物体");
ok(F.objectLabel({ label_zh: "   " }) === "未分类物体", "空白中文名同样兜底");
ok(F.objectLabel(null) === "未分类物体", "null 不炸");
// 源码级断言：文本绘制必须走 objectLabel，不许再出现裸的 item.label / raw_label。
ok(!/fillText\(item\.label/.test(source), "canvas 上没有裸 item.label 的 fillText");
ok(!/fillText\([^)]*raw_label/.test(source), "fillText 里不许出现 raw_label");
// 3 处 = 函数声明本身 + 点/框各一次调用。
ok((source.match(/objectLabel\(item\)/g) || []).length === 3,
   "两个绘制函数（点/框）都走 objectLabel");

console.log("objectVerified —— 目标例外换颜色:");
ok(F.objectVerified({ decision: "TARGET_VERIFIED" }) === true, "复核目标认出来");
ok(F.objectVerified({ decision: "DISPLAY" }) === false, "普通显示不是例外");
ok(F.objectVerified({}) === false, "缺字段按普通处理");

console.log("objectFilterSummary —— 状态栏的过滤统计:");
ok(F.objectFilterSummary({ input: 20, displayed: 4, filtered: 16 })
   === "物体框 4 个 / 已过滤 16 个", "正常统计");
ok(F.objectFilterSummary({ input: 3, displayed: 3 })
   === "物体框 3 个 / 已过滤 0 个", "缺 filtered 时自己算差值");
ok(F.objectFilterSummary({ input: 0, displayed: 0, filtered: 0 })
   === "物体框 0 个 / 已过滤 0 个", "零也照报");
ok(F.objectFilterSummary(null) === "", "老快照没有这个字段时不显示");
ok(F.objectFilterSummary({}) === "", "字段不全时宁可不显示，也不报 0/0");

console.log(failures === 0 ? "ALL SLAM3D FRAME TESTS PASSED" : failures + " FAILURES");
process.exit(failures === 0 ? 0 : 1);
