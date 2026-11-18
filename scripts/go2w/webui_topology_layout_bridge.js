#!/usr/bin/env node
"use strict";

// Reuse the exact deterministic relation-constrained layout exported by the
// live WebUI.  The Python rosbag renderer sends incremental graph snapshots on
// stdin and receives stable positions plus WebUI-decorated relation labels.
const fs = require("fs");
const path = require("path");
const vm = require("vm");

const projectRoot = path.resolve(__dirname, "..", "..");
const searchMapPath = path.join(
  projectRoot, "app", "manual_web_demo", "static", "search_map.js");
const source = fs.readFileSync(searchMapPath, "utf8");
const window = {};
const sandbox = {window: window, document: undefined, console: console};
vm.createContext(sandbox);
vm.runInContext(source, sandbox, {filename: searchMapPath});

const payload = JSON.parse(fs.readFileSync(0, "utf8"));
let previous = {};
const results = (payload.snapshots || []).map(function (snapshot) {
  const objects = (snapshot.nodes || []).map(function (node) {
    return {object_id: node.node_id, map_xyz: node.map_xyz};
  });
  const decorated = window.TopologyLayout.decorateEdges(
    snapshot.edges || [], objects);
  const positions = window.TopologyLayout.computeLayout(
    snapshot.nodes || [], decorated, previous, {});
  previous = positions;
  return {key: snapshot.key, positions: positions, edges: decorated};
});
process.stdout.write(JSON.stringify({snapshots: results}));
