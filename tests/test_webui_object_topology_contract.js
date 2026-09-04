#!/usr/bin/env node
"use strict";
/*
 * 拓扑契约测试。
 *
 * 构造同时含 P1、F01、obj_001、obj_002 的完整导航图（semantic_navigation_graph_v1），
 * 断言 WebUI 的语义拓扑投影保留对象节点 + PLACE 中转站及其挂接边（OBSERVED_FROM /
 * CONNECTED_TO），并剔除 FRONTIER 节点与 FRONTIER_TO / MOVED_TO 纯导航关系。
 * PLACE 必须留下：去掉它，不同视角看到的物体之间没有任何边，拓扑必然碎片化。
 *
 * Run with:  node tests/test_webui_object_topology_contract.js
 */
const fs = require("fs");
const path = require("path");
const vm = require("vm");

const searchMapPath = path.join(
  __dirname, "..", "app", "manual_web_demo", "static", "search_map.js");
const source = fs.readFileSync(searchMapPath, "utf8");

let failures = 0;
function assert(cond, msg) {
  if (!cond) { console.error("  FAIL: " + msg); failures += 1; return; }
  console.log("  ok: " + msg);
}
function assertEqual(a, b, msg) {
  if (a !== b) {
    console.error("  FAIL: " + msg + " (expected " + JSON.stringify(b) +
                  ", got " + JSON.stringify(a) + ")");
    failures += 1;
    return;
  }
  console.log("  ok: " + msg);
}

const window = {};
const sandbox = { window: window, document: undefined, console: console };
vm.createContext(sandbox);
vm.runInContext(source, sandbox, { filename: searchMapPath });

const objectOnly = window.TopologyLayout && window.TopologyLayout.objectOnly;
if (!objectOnly) {
  console.error("window.TopologyLayout.objectOnly not exposed - cannot test");
  process.exit(1);
}

// ---- 完整导航图：Place + Frontier + Object，含导航关系边 -------------------
const navigationGraph = {
  schema_version: "semantic_navigation_graph_v1",
  revision: 7,
  current_place_id: "P1",
  nodes: [
    { node_id: "P1", node_type: "PLACE", label: "P1", current: true },
    { node_id: "F01", node_type: "FRONTIER", label: "F01", status: "OPEN" },
    { node_id: "F02", node_type: "FRONTIER", label: "F02", status: "OPEN" },
    { node_id: "obj_001", node_type: "OBJECT", label: "办公桌",
      status: "CONFIRMED", observation_count: 4 },
    { node_id: "obj_002", node_type: "OBJECT", label: "白色垃圾桶",
      status: "CONFIRMED", observation_count: 2 },
  ],
  edges: [
    { from: "obj_001", to: "obj_002", relation: "near", status: "CONFIRMED" },
    { from: "P1", to: "obj_001", relation: "OBSERVED_FROM" },
    { from: "P1", to: "F01", relation: "FRONTIER_TO" },
    { from: "P1", to: "F02", relation: "MOVED_TO" },
  ],
};

// 后端已经过滤好的对象投影（正常路径）。
const objectTopology = {
  schema_version: "semantic_object_topology_v1",
  revision: 7,
  nodes: [
    { node_id: "obj_001", node_type: "OBJECT", label: "办公桌",
      status: "CONFIRMED", observation_count: 4 },
    { node_id: "obj_002", node_type: "OBJECT", label: "白色垃圾桶",
      status: "CONFIRMED", observation_count: 2 },
  ],
  edges: [
    { from: "obj_001", to: "obj_002", relation: "near", status: "CONFIRMED" },
  ],
  stats: { node_count: 2, edge_count: 1 },
};

function ids(topology) {
  return (topology.nodes || []).map(function (n) { return n.node_id; }).sort();
}

console.log("case 1: backend object_topology passes through unchanged");
const projected = objectOnly(objectTopology);
assertEqual(JSON.stringify(ids(projected)),
            JSON.stringify(["obj_001", "obj_002"]), "nodes = obj_001, obj_002");
assertEqual(projected.edges.length, 1, "one object relation edge survives");
assertEqual(projected.edges[0].relation, "near", "relation is near");
assertEqual(projected.schema_version, "semantic_object_topology_v1",
            "schema_version preserved");

console.log("case 2: a leaked full navigation graph keeps objects + place, drops frontiers");
const guarded = objectOnly(navigationGraph);
const guardedIds = ids(guarded);
// 排序后大写 P 的 ASCII 小于小写 o，所以 P1 在最前面。
assertEqual(JSON.stringify(guardedIds),
            JSON.stringify(["P1", "obj_001", "obj_002"]),
            "nodes = P1, obj_001, obj_002");
assert(guardedIds.indexOf("P1") >= 0, "P1 kept as the cross-view hub");
assert(guardedIds.indexOf("F01") < 0, "F01 absent");
assert(guardedIds.indexOf("F02") < 0, "F02 absent");
const relations = guarded.edges.map(function (e) { return e.relation; });
assert(relations.indexOf("OBSERVED_FROM") >= 0, "OBSERVED_FROM kept");
assert(relations.indexOf("FRONTIER_TO") < 0, "FRONTIER_TO absent");
assert(relations.indexOf("MOVED_TO") < 0, "MOVED_TO absent");
assertEqual(guarded.edges.length, 2, "near + OBSERVED_FROM remain");
guarded.edges.forEach(function (edge) {
  assert(guardedIds.indexOf(edge.from) >= 0 && guardedIds.indexOf(edge.to) >= 0,
         "edge endpoints subset of kept nodes: " + edge.from + "->" + edge.to);
});

console.log("case 3: node type comes from node_type, never from the id prefix");
const spoofed = objectOnly({
  nodes: [
    { node_id: "obj_bad", node_type: "FRONTIER", label: "伪装成物体的前沿" },
    { node_id: "P9", node_type: "OBJECT", label: "ID 不带前缀的真物体" },
  ],
  edges: [{ from: "obj_bad", to: "P9", relation: "near" }],
});
assertEqual(JSON.stringify(ids(spoofed)), JSON.stringify(["P9"]),
            "obj_ prefixed FRONTIER dropped, OBJECT kept regardless of id");
assertEqual(spoofed.edges.length, 0, "edge with a dropped endpoint is removed");

console.log("case 4: isolated objects are shown, no fallback to the nav graph");
const isolated = objectOnly({
  nodes: [{ node_id: "obj_001", node_type: "OBJECT", label: "办公桌" }],
  edges: [],
});
assertEqual(isolated.nodes.length, 1, "isolated object node kept");
assertEqual(isolated.edges.length, 0, "no relations invented");
assertEqual(objectOnly(null), null, "missing topology stays null (empty state)");
assertEqual(objectOnly({}), null, "topology without nodes stays null");

if (failures) {
  console.error("\n" + failures + " JS TOPOLOGY CONTRACT TEST(S) FAILED");
  process.exit(1);
}
console.log("\nALL JS TOPOLOGY CONTRACT TESTS PASSED");
