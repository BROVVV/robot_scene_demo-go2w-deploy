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

// ---- fixture: 3 objects, 2 relations (b uses x4 observer count to be root) ----
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

// root (max observation_count) at depth 0 => x = TOPO_PAD (26)
assertEqual(pos1["obj_001"].x, 26, "root obj_001 (max obs) sits at first layer x");

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

console.log("isolated nodes are laid out too:");
const isoNodes = [
  { node_id: "obj_007", label: "杂物", status: "TENTATIVE" },
  { node_id: "obj_009", label: "纸箱", status: "TENTATIVE" },
];
const isoPos = TL.computeLayout(isoNodes, [], {});
assertEqual(Object.keys(isoPos).length, 2, "isolated objects get positions");

console.log(failures === 0 ? "ALL JS LAYOUT TESTS PASSED" : failures + " FAILURES");
process.exit(failures === 0 ? 0 : 1);
