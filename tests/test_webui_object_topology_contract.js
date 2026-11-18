#!/usr/bin/env node
"use strict";
/*
 * 拓扑契约测试。
 *
 * 构造同时含 P1、F01、obj_001、obj_002 的完整导航图（semantic_navigation_graph_v1），
 * 断言 WebUI 两个投影各自的契约：
 *
 *   objectOnly   —— 导航拓扑视图（计划3 步骤 2.1/2.2）：保留对象节点 + PLACE 中转站
 *                   及其挂接边（OBSERVED_FROM / CONNECTED_TO），剔除 FRONTIER 节点与
 *                   FRONTIER_TO / MOVED_TO 纯导航关系。PLACE 必须留下：去掉它，不同
 *                   视角看到的物体之间没有任何边，拓扑必然碎片化。
 *   objectStrict —— 物体拓扑视图（默认；计划1 §7.4）：只留 OBJECT 与物体间关系，
 *                   「不出现 P1/P2/F01/F02」「不出现 OBSERVED_FROM/FRONTIER_TO/MOVED_TO」。
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
const objectStrict = window.TopologyLayout && window.TopologyLayout.objectStrict;
if (!objectStrict) {
  console.error("window.TopologyLayout.objectStrict not exposed - cannot test");
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

// ===========================================================================
// objectStrict —— 计划1 §7.4 的验收契约（语义拓扑页的默认视图）
// ===========================================================================
console.log("case 5: strict view passes an already-object-only projection through");
const strictPlain = objectStrict(objectTopology);
assertEqual(JSON.stringify(ids(strictPlain)),
            JSON.stringify(["obj_001", "obj_002"]), "nodes = obj_001, obj_002");
assertEqual(strictPlain.edges.length, 1, "object-to-object edge survives");
assertEqual(strictPlain.edges[0].relation, "near", "relation is near");

console.log("case 6: strict view drops P1 and every edge that touched it");
const strictGuarded = objectStrict(navigationGraph);
assertEqual(JSON.stringify(ids(strictGuarded)),
            JSON.stringify(["obj_001", "obj_002"]),
            "only objects remain, P1 / F01 / F02 gone");
const strictRelations = strictGuarded.edges.map(function (e) { return e.relation; });
// 计划1 §7.4：「不出现 OBSERVED_FROM/FRONTIER_TO/MOVED_TO」。
// 这三类边都有一端是 PLACE 或 FRONTIER，节点过滤这一步就把它们剔掉了。
assert(strictRelations.indexOf("OBSERVED_FROM") < 0, "OBSERVED_FROM absent");
assert(strictRelations.indexOf("FRONTIER_TO") < 0, "FRONTIER_TO absent");
assert(strictRelations.indexOf("MOVED_TO") < 0, "MOVED_TO absent");
assertEqual(strictGuarded.edges.length, 1, "only the object-object edge remains");
assertEqual(strictGuarded.edges[0].relation, "near", "kept edge is the object relation");

console.log("case 7: strict view trusts node_type, not the id prefix");
const strictSpoofed = objectStrict({
  nodes: [
    { node_id: "obj_bad", node_type: "PLACE", label: "伪装成物体的地点" },
    { node_id: "P9", node_type: "OBJECT", label: "ID 不带前缀的真物体" },
  ],
  edges: [{ from: "obj_bad", to: "P9", relation: "near" }],
});
assertEqual(JSON.stringify(ids(strictSpoofed)), JSON.stringify(["P9"]),
            "obj_ prefixed PLACE dropped, OBJECT kept regardless of id");
assertEqual(strictSpoofed.edges.length, 0, "edge with a dropped endpoint is removed");

console.log("case 8: strict view keeps isolated objects and reports honest stats");
const strictIsolated = objectStrict({
  nodes: [
    { node_id: "obj_001", node_type: "OBJECT", label: "办公桌" },
    { node_id: "P1", node_type: "PLACE", label: "P1" },
  ],
  edges: [{ from: "P1", to: "obj_001", relation: "OBSERVED_FROM" }],
  stats: { node_count: 2, edge_count: 1 },
});
assertEqual(strictIsolated.nodes.length, 1, "isolated object node kept");
assertEqual(strictIsolated.edges.length, 0, "no relations invented");
assertEqual(strictIsolated.stats.node_count, 1,
            "stats recomputed for the filtered set (PLACE is not an object)");
assertEqual(strictIsolated.stats.edge_count, 0, "filtered edge count");
assertEqual(objectStrict(null), null, "missing topology stays null (empty state)");
assertEqual(objectStrict({}), null, "topology without nodes stays null");

console.log("case 9: render selects projections only from state.graph");
const integrated = JSON.parse(JSON.stringify(navigationGraph));
integrated.nodes.push({ node_id: "nav_only", node_type: "OBJECT", label: "nav" });
const rootOnly = { node_id: "root_only", node_type: "OBJECT", label: "root" };
const spatialOnly = { nodes: [{ node_id: "spatial_only", node_type: "OBJECT" }], edges: [] };
const stateGraph = {
  nodes: [rootOnly], edges: [], object_topology: objectTopology,
  integrated_topology: integrated, objects: [{ object_id: "evidence" }],
};
function rendered(view, graph) {
  let captured;
  const renderer = { topologyView: view, renderObjectTopology: function (topology, objects) {
    captured = { topology, objects };
  } };
  window.SearchMapRenderer.prototype.render.call(renderer, graph,
    { semantic_graph: spatialOnly });
  return captured;
}
const objectsRender = rendered("objects", stateGraph);
assertEqual(objectsRender.topology.nodes.map(n => n.node_id).sort().join(","),
  "obj_001,obj_002", "object view excludes integrated, raw and spatial-only nodes");
assertEqual(objectsRender.objects, stateGraph.objects, "same state.graph supplies semantic objects");
const navRender = rendered("navigation", stateGraph);
assertEqual(navRender.topology.nodes.map(n => n.node_id).sort().join(","),
  "P1,nav_only,obj_001,obj_002", "navigation view selects integrated projection and drops frontier");
assertEqual(rendered("objects", { nodes: [rootOnly], edges: [] }).topology,
  null, "no fallback to the raw navigation graph or spatial graph");
assertEqual(rendered("navigation", { object_topology: objectTopology }).topology.nodes.length,
  2, "navigation view can use the same state's object projection");

if (failures) {
  console.error("\n" + failures + " JS TOPOLOGY CONTRACT TEST(S) FAILED");
  process.exit(1);
}
console.log("\nALL JS TOPOLOGY CONTRACT TESTS PASSED");
console.log("RENDER SOURCE SELECTION TESTS PASSED");
