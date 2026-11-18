#!/usr/bin/env node
"use strict";
/*
 * Headless unit test for the pure semantic-topology display layout
 * (plan §19 / §36).  It loads search_map.js in a fake browser scope and only
 * exercises the DOM-independent helpers exposed on window.TopologyLayout.
 *
 * Run with:  node tests/test_search_topology_layout.js
 */
const fs = require("fs");
const path = require("path");
const vm = require("vm");

const searchMapPath = path.join(__dirname, "..", "app", "manual_web_demo", "static", "search_map.js");
const source = fs.readFileSync(searchMapPath, "utf8");

let failures = 0;
function assert(cond, msg) {
  if (!cond) {
    console.error("  FAIL: " + msg);
    failures += 1;
    return;
  }
  console.log("  ok: " + msg);
}
function assertEqual(a, b, msg) {
  if (a !== b) {
    console.error("  FAIL: " + msg + " (expected " + JSON.stringify(b) + ", got " + JSON.stringify(a) + ")");
    failures += 1;
    return;
  }
  console.log("  ok: " + msg);
}

const window = {};
const sandbox = {
  window: window,
  document: undefined,
  console: console,
};
vm.createContext(sandbox);
vm.runInContext(source, sandbox, { filename: searchMapPath });

const TL = window.TopologyLayout;
if (!TL) {
  console.error("window.TopologyLayout not exposed - cannot test");
  process.exit(1);
}

// ---- fixture: 3 objects, 2 relations ----
const nodes = [
  { node_id: "obj_003", label: "办公椅", status: "TENTATIVE", observation_count: 1 },
  { node_id: "obj_001", label: "办公桌", status: "CONFIRMED", observation_count: 4 },
  { node_id: "obj_002", label: "绿色垃圾桶", status: "CONFIRMED", observation_count: 2 },
];
const edges = [
  { edge_id: "obj_001__near__obj_002", from: "obj_001", to: "obj_002", relation: "near" },
  { edge_id: "obj_003__left_of__obj_001", from: "obj_003", to: "obj_001", relation: "left_of" },
];

console.log("computeTopologyLayout determinism / separation:");
const pos1 = TL.computeLayout(nodes, edges, {});
const pos2 = TL.computeLayout(nodes, edges, {});
assertEqual(JSON.stringify(pos1), JSON.stringify(pos2), "same graph -> same positions");

const ids = Object.keys(pos1);
assertEqual(ids.length, 3, "all three nodes positioned");

// no two nodes coincide
let collide = false;
for (let i = 0; i < ids.length; i++) {
  for (let j = i + 1; j < ids.length; j++) {
    if (pos1[ids[i]].x === pos1[ids[j]].x && pos1[ids[i]].y === pos1[ids[j]].y) collide = true;
  }
}
assert(!collide, "no two nodes overlap exactly");
assert(pos1["obj_003"].x < pos1["obj_001"].x,
       "left_of relation places the subject to the left");

function cardsOverlap(a, b) {
  return Math.abs(a.x - b.x) < 152 && Math.abs(a.y - b.y) < 76;
}
let cardOverlap = false;
for (let i = 0; i < ids.length; i++) {
  for (let j = i + 1; j < ids.length; j++) {
    if (cardsOverlap(pos1[ids[i]], pos1[ids[j]])) cardOverlap = true;
  }
}
assert(!cardOverlap, "node cards have a readable collision gap");

console.log("directional constraints:");
const directionNodes = [
  {node_id: "a"}, {node_id: "b"}, {node_id: "c"}, {node_id: "d"},
];
const directionEdges = [
  {from: "a", to: "b", relation: "above", relation_scope: "VIEW_RELATIVE", status: "CONFIRMED"},
  {from: "c", to: "b", relation: "left_of", relation_scope: "VIEW_RELATIVE", status: "CONFIRMED"},
  {from: "d", to: "b", relation: "right_of", relation_scope: "VIEW_RELATIVE", status: "CONFIRMED"},
];
const reversedPrevious = {
  a: {x: 200, y: 220}, b: {x: 200, y: 0},
  c: {x: 400, y: 120}, d: {x: 0, y: 120},
};
const directionPos = TL.computeLayout(directionNodes, directionEdges, reversedPrevious);
assert(directionPos.a.y < directionPos.b.y, "above overrides a stale reversed layout");
assert(directionPos.c.x < directionPos.b.x, "left_of overrides a stale reversed layout");
assert(directionPos.d.x > directionPos.b.x, "right_of overrides a stale reversed layout");
const directionIds = Object.keys(directionPos);
let directionOverlap = false;
for (let i = 0; i < directionIds.length; i++) {
  for (let j = i + 1; j < directionIds.length; j++) {
    if (cardsOverlap(directionPos[directionIds[i]], directionPos[directionIds[j]])) directionOverlap = true;
  }
}
assert(!directionOverlap, "direction-aware layout still keeps all cards apart");

const pinnedPrevious = {a: {x: 100, y: 250}, b: {x: 100, y: 100}};
const pinnedPos = TL.computeLayout(
  [{node_id: "a"}, {node_id: "b"}],
  [{from: "a", to: "b", relation: "above", status: "CONFIRMED"}],
  pinnedPrevious,
  {b: true}
);
assertEqual(pinnedPos.b.y, 100, "a pinned anchor remains fixed while its neighbor moves");
assert(pinnedPos.a.y < pinnedPos.b.y, "direction is satisfied around a pinned anchor");

// Incremental layout keeps the existing screen positions while adding a node.
const nextNodes = nodes.concat([{ node_id: "obj_004", label: "纸箱", status: "CONFIRMED" }]);
const nextEdges = edges.concat([{ from: "obj_001", to: "obj_004", relation: "near" }]);
const nextPos = TL.computeLayout(nextNodes, nextEdges, pos1);
assert(Object.keys(nextPos).length === 4, "new connected node receives a position");
assert(Math.abs(nextPos["obj_001"].x - pos1["obj_001"].x) < 30, "existing node remains stable on incremental update");

// fingerprint stable regardless of node order
const fpA = TL.fingerprint(
  nodes,
  edges
);
const fpB = TL.fingerprint(nodes.slice().reverse(), edges);
assertEqual(fpA, fpB, "fingerprint is order-independent");

console.log("components:");
const comps = TL.components(nodes, { obj_001: ["obj_002", "obj_003"], obj_002: ["obj_001"], obj_003: ["obj_001"] });
assertEqual(comps.length, 1, "single connected component");

console.log("relation labels:");
const e = { relation: "near", observation_count: 3, relation_scope: "STRUCTURAL" };
assert(TCheckLabel(TL, e), "edge label contains 邻近 and ×3");

function TCheckLabel(tl, edgeObj) {
  const label = tl.relationLabel(edgeObj, []);
  return label.indexOf("邻近") >= 0 && label.indexOf("×3") >= 0;
}

console.log("visible relative-position labels:");
const decorated = TL.decorateEdges(
  [
    { edge_id: "near", from: "obj_001", to: "obj_002", relation: "near" },
    { edge_id: "seen", from: "obj_001", to: "obj_002", relation: "co_visible" },
  ],
  [
    { object_id: "obj_001", map_xyz: [2.0, 1.0, 0.0] },
    { object_id: "obj_002", map_xyz: [1.0, 0.0, 0.0] },
  ]
);
const shown = decorated.filter(function (edge) { return edge._show_relation_label; });
assertEqual(shown.length, 1, "one persistent label is shown per object pair");
assert(shown[0]._display_relation_label.indexOf("左侧") >= 0, "legacy map gets a left/right fallback");
assert(shown[0]._display_relation_label.indexOf("前方") >= 0, "legacy map gets a front/back fallback");

const measured = TL.decorateEdges(
  [
    { edge_id: "a_left", from: "obj_001", to: "obj_002", relation: "left_of" },
    { edge_id: "b_front", from: "obj_002", to: "obj_001", relation: "in_front_of" },
    { edge_id: "near", from: "obj_001", to: "obj_002", relation: "near" },
  ],
  []
);
const measuredLabel = measured.filter(function (edge) { return edge._show_relation_label; })[0];
assert(measuredLabel._display_relation_label.indexOf("左侧") >= 0, "measured direction is visible");
assert(measuredLabel._display_relation_label.indexOf("后方") >= 0, "reverse edge is normalized to the label anchor");

const coVisibleOnly = TL.decorateEdges(
  [{ edge_id: "seen", from: "obj_010", to: "obj_011", relation: "co_visible" }],
  []
);
assert(coVisibleOnly[0]._show_relation_label, "co-visible-only line is not left unlabeled");
assert(coVisibleOnly[0]._display_relation_label.indexOf("共同观测") >= 0, "co-visible fallback is explicit");

console.log("isolated nodes are laid out too:");
const isoNodes = [
  { node_id: "obj_007", label: "杂物", status: "TENTATIVE" },
  { node_id: "obj_009", label: "纸箱", status: "TENTATIVE" },
];
const isoPos = TL.computeLayout(isoNodes, [], {});
assertEqual(Object.keys(isoPos).length, 2, "isolated objects get positions");

console.log(failures === 0 ? "ALL JS LAYOUT TESTS PASSED" : failures + " FAILURES");
process.exit(failures === 0 ? 0 : 1);
