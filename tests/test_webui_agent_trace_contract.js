#!/usr/bin/env node
"use strict";
/*
 * 任务 Agent 解释层的展示契约测试（计划书 §5.2）。
 *
 * 构造一份真实的 task_agent_trace_v1（外加 legacy / 空 trace 两种边界），断言
 * agent_trace.js 的纯渲染层满足展示规则：
 *
 *   §5.2 选择方向 = 文字 + 角度，正负约定由页面写明（+ 左转 / − 右转）。
 *   §5.2 缺失数据写"未提供"，绝不显示成绿色 0。
 *   §5.2 知识条目显示来源、置信度和 used / context_only。
 *   §5.2 候选行只显示代码真正用过的排序分量，没有虚构的综合成功率。
 *   §5.2 offline replay 用固定醒目标签；计划状态图标 + 文字。
 *   §5.2 任务 / 标签 / 知识库文本一律转义。
 *   §5.3 下游没给原因就写"下游未提供拒绝原因"，不猜。
 *
 * Run with:  node tests/test_webui_agent_trace_contract.js
 */
const fs = require("fs");
const path = require("path");
const vm = require("vm");

const modulePath = path.join(
  __dirname, "..", "app", "manual_web_demo", "static", "agent_trace.js");
const source = fs.readFileSync(modulePath, "utf8");

let failures = 0;
function assert(cond, msg) {
  if (!cond) { console.error("  FAIL: " + msg); failures += 1; return; }
  console.log("  ok: " + msg);
}
function assertEqual(actual, expected, msg) {
  if (actual !== expected) {
    console.error("  FAIL: " + msg + "\n        expected " +
                  JSON.stringify(expected) + "\n        got      " +
                  JSON.stringify(actual));
    failures += 1;
    return;
  }
  console.log("  ok: " + msg);
}

const sandbox = { window: {}, console: console };
vm.createContext(sandbox);
vm.runInContext(source, sandbox, { filename: modulePath });
const T = sandbox.window.AgentTrace;
if (!T) {
  console.error("window.AgentTrace not exposed - cannot test");
  process.exit(1);
}

// ---- 一份真实的 trace（字段取自 reasoner 的实际输出形状） ------------------
const trace = {
  schema_version: "task_agent_trace_v1",
  trace_id: "directive_abc",
  mode: "offline_replay",
  task: { raw_text: "找到白色垃圾桶", canonical_target: "白色垃圾桶",
          attributes: [], relations: [], constraints: [] },
  plan: [
    { id: "understand_task", title: "理解任务", status: "completed",
      detail: "解析目标、属性、关系与限制。", capability: "supported" },
    { id: "search_or_explore", title: "选择搜索方向", status: "active",
      detail: "按证据感知的候选排序给出本轮搜索偏好。", capability: "supported" },
    { id: "report_result", title: "汇报结果", status: "pending",
      detail: "仅在收到任务结束事件后才算完成。", capability: "supported" },
  ],
  active_subtask: { id: "search_or_explore", title: "搜索目标",
                    reason: "目标尚未通过视觉判据确认，本轮只产生搜索偏好。" },
  inputs: {
    graph_match: { state: "partial_match", score: 0.425,
                   matched_scene_node_ids: ["obj_bin_001"],
                   supporting_anchor_scene_node_ids: [] },
    scene_graph: { node_count: 2, labels: ["垃圾桶", "办公桌"] },
    observation: { scan_index: 3, robot_yaw_deg: 90.0,
                   observed_heading_sectors: [4],
                   distance_from_origin_m: 0.0, has_semantic_observation: true },
    memory: { observation_memory_count: 4, negative_memory_present: false,
              negative_target_key: "白色垃圾桶" },
    topology: null,
    missing_sources: ["topology_summary", "task_context"],
  },
  knowledge: [
    { id: "object_prior:phone", content: "手机常在桌上",
      source: "data/scene_kb/object_location_priors.json", confidence: 0.86,
      status: "context_only", supports: [],
      reason: "该条知识的相关对象未出现在本轮场景图中，仅作背景，未参与方向排序。" },
  ],
  candidates: [
    // rank_key 的分量顺序就是 _rank_key 元组的顺序，位置固定：
    // 覆盖度/负记忆、语义距离、知识距离、−辅助分、|转角|、候选下标。
    // 选中项必须是**逐项比较下来最小的那一行**（`_rank_candidate_headings` 取 min）。
    // 这里刻意让两行在语义距离上反向：赢家离语义证据 26.42°（更远），输家只有 5.0°，
    // 但赢家的 rank_key[0] = 0.00 而输家是 0.45（它所在的扇区本轮已经看过了），
    // 覆盖度排在语义距离前面 —— 所以赢的是赢家。这正是方案 C 的排序语义，
    // 也是真机 explore_go2w_20260916_200905 的形状：输在覆盖度，不是输在"离证据更远"。
    { id: "heading:-30", relative_heading_deg: -30, absolute_sector: 2,
      observed: false, negative_memory_penalty: 0.0,
      semantic_support_refs: ["obj_bin_001"], knowledge_support_refs: [],
      auxiliary_support_refs: [], rank_key: [0.0, 26.420000000000016, null, -0.0, 30, 0],
      selected: true, rejection_reasons: [] },
    { id: "heading:+30", relative_heading_deg: 30, absolute_sector: 4,
      observed: true, negative_memory_penalty: 0.0,
      semantic_support_refs: ["obj_bin_001"], knowledge_support_refs: [],
      auxiliary_support_refs: [], rank_key: [0.45, 5.0, null, -0.0, 30, 1],
      selected: false,
      rejection_reasons: ["sector_already_observed", "lower_rank"] },
  ],
  selected_preference: {
    directive_kind: "explore_unseen", relative_heading_deg: -30,
    reason_code: "partial_without_anchor",
    reason_zh: "任务目标「白色垃圾桶」：仅目标节点形成 PARTIAL，缺少可检查锚点，因此转入未观测方向探索，本轮建议相对转向 -30°。",
    evidence_refs: [],
  },
  uncertainties: [
    { code: "missing_topology_summary", detail: "未拿到 topology_summary，相关判断只能建立在其余来源上。" },
  ],
  safety: { can_confirm_target: false, can_authorize_motion: false,
            allow_forward: false },
  downstream: { status: "pending", decision_id: null, reason: null },
};

const html = T.render(trace, { trace: trace, updated_at: 1758000000.0,
                               decision_id: null, downstream_status: "pending",
                               downstream_reason: null });
const all = Object.keys(html).map(function (k) { return html[k]; }).join("\n");

console.log("case 1: heading text carries the sign convention in words");
assertEqual(T.headingText(30), "左前方（+30.0°，按当前坐标约定）", "+30 is left-front");
assertEqual(T.headingText(-30), "右前方（−30.0°，按当前坐标约定）", "-30 is right-front");
assertEqual(T.headingText(60), "左侧方（+60.0°，按当前坐标约定）", "+60 is left-side");
assertEqual(T.headingText(90), "正左方（+90.0°，按当前坐标约定）", "+90 is due left");
assertEqual(T.headingText(0), "正前方（0°）", "0 is dead ahead");
assertEqual(T.headingText(null), "未提供", "null heading says 未提供, never 0");
assertEqual(T.headingText(undefined), "未提供", "undefined heading says 未提供");
assert(T.ANGLE_CONVENTION.indexOf("+ 为左转") >= 0, "convention states + = left");
assert(all.indexOf(T.ANGLE_CONVENTION) >= 0, "convention is printed on the panel");

console.log("case 2: numbers are fixed-precision and missing is never 0");
assertEqual(T.num(26.420000000000016, 2), "26.42", "raw float is rounded for display");
assertEqual(T.num(0.0, 2), "0.00", "a real zero still renders as a zero");
assertEqual(T.num(null, 2), null, "null stays null");
assertEqual(T.num("", 2), null, "empty string stays null");
assertEqual(T.num("abc", 2), null, "garbage stays null");
assert(html.inputs.indexOf("拓扑摘要</span><span class=\"v\">未提供") >= 0,
       "missing topology renders 未提供, not 0");
assert(html.candidates.indexOf("26.42") >= 0, "rank_key shows the real component");
assert(html.candidates.indexOf("26.420000000000016") < 0, "raw float is not dumped");
assert(html.candidates.indexOf("—") >= 0, "absent evidence component renders as —");

console.log("case 3: offline replay is a fixed prominent label");
assert(html.mode.indexOf("离线回放 · 非实机") >= 0, "offline badge present");
assert(html.mode.indexOf("agent-badge") >= 0, "offline badge is styled");
const liveHtml = T.render(Object.assign({}, trace, { mode: "live" }), null);
assert(liveHtml.mode.indexOf("离线回放") < 0, "live mode has no offline badge");

console.log("case 4: plan status is icon plus text, never colour alone");
assert(html.plan.indexOf("✔ 已完成") >= 0, "completed = icon + text");
assert(html.plan.indexOf("▶ 进行中") >= 0, "active = icon + text");
assert(html.plan.indexOf("○ 待执行") >= 0, "pending = icon + text");
const blocked = T.render(Object.assign({}, trace, {
  plan: [{ id: "s2", title: "把它拿起来",
           status: "unsupported_by_current_executor", detail: "" }],
}), null);
assert(blocked.plan.indexOf("⊘ 当前执行器不支持") >= 0,
       "unsupported stage states the reason in words");

console.log("case 5: knowledge shows source, confidence and used/context_only");
assert(html.knowledge.indexOf("context_only") >= 0, "context_only is labelled");
assert(html.knowledge.indexOf("0.86") >= 0, "confidence is shown");
assert(html.knowledge.indexOf("object_location_priors.json") >= 0, "source is shown");
const used = T.render(Object.assign({}, trace, {
  knowledge: [{ id: "k1", content: "垃圾桶常在桌边", source: "kb.json",
                confidence: 0.5, status: "used", supports: ["heading:+30"],
                reason: "参与了方向排序。" }],
}), null);
assert(used.knowledge.indexOf("used") >= 0, "used is labelled");
assert(used.knowledge.indexOf("heading:+30") >= 0, "supported candidate is listed");

console.log("case 6: candidate rows expose only the real sort components");
assert(html.candidates.indexOf("代码实际比较顺序") >= 0, "rank_key is labelled as the real key");
// 表头必须正好是那 8 个真实分量列，不许混进"综合成功率 / 总分"这类编出来的列。
const headers = html.candidates.match(/<th>(.*?)<\/th>/g).map(function (cell) {
  return cell.replace(/<\/?th>/g, "");
});
assertEqual(JSON.stringify(headers),
            JSON.stringify(["方向", "语义", "知识", "辅助", "已观测", "负记忆",
                            "排序键（代码实际比较顺序）", "结果"]),
            "table header is exactly the real components");
assert(html.candidates.indexOf("没有虚构的综合成功率") >= 0,
       "the panel states it shows no invented composite score");
assert(html.candidates.indexOf("✔ 选中") >= 0, "the selected row is marked");
assert(html.candidates.indexOf("sector_already_observed") >= 0,
       "the loser states which key component it lost on");
assert(html.candidates.indexOf("扇区 2") >= 0, "absolute sector is shown");

console.log("case 7: the downstream verdict never invents a rejection reason");
const rejected = T.render(trace, { trace: trace, updated_at: 1758000000.0,
                                   decision_id: "decision_1",
                                   downstream_status: "rejected",
                                   downstream_reason: null });
assert(rejected.downstream.indexOf("Agent 建议未成为最终决定") >= 0,
       "rejection is stated");
assert(rejected.downstream.indexOf("下游未提供拒绝原因") >= 0,
       "absent reason says 未提供, no guessing");
const withReason = T.render(trace, { trace: trace, downstream_status: "rejected",
                                     downstream_reason: "冻结候选生成器未产生可用语义目标" });
assert(withReason.downstream.indexOf("冻结候选生成器未产生可用语义目标") >= 0,
       "a real reason is shown when the downstream provides one");
const accepted = T.render(trace, { trace: trace, decision_id: "d9",
                                   downstream_status: "accepted" });
assert(accepted.downstream.indexOf("✔ 已采纳") >= 0, "accepted verdict");
assert(accepted.downstream.indexOf("未成为最终决定") < 0, "no rejection text when accepted");
assert(html.downstream.indexOf("待确认") >= 0, "pending verdict says 待确认");

console.log("case 8: text from task / labels / knowledge is escaped");
const hostile = T.render(Object.assign({}, trace, {
  task: { raw_text: '<img src=x onerror=alert(1)>', canonical_target: "</script>",
          attributes: [], relations: [], constraints: [] },
  knowledge: [{ id: "k'\"1", content: '<script>alert(2)</script>',
                source: '<b>kb</b>', confidence: 0.5, status: "used",
                supports: ["<i>x</i>"], reason: "<u>r</u>" }],
  selected_preference: Object.assign({}, trace.selected_preference, {
    reason_zh: '<svg onload=alert(3)>' }),
  uncertainties: [{ code: "<x>", detail: "<y>" }],
  plan: [{ id: "p", title: "<script>t</script>", status: "pending", detail: "<d>" }],
  candidates: [Object.assign({}, trace.candidates[0], {
    auxiliary_support_refs: ["<a>"], rejection_reasons: ["<r>"],
    rank_key: [0, "<k>", null, 0, 30, 0] })],
}), null);
const hostileAll = Object.keys(hostile).map(function (k) { return hostile[k]; }).join("\n");
assert(hostileAll.indexOf("<script>") < 0, "no raw <script> survives");
assert(hostileAll.indexOf("<img") < 0, "no raw <img> survives");
// `esc()` 转义的是 < > & "，所以 `onerror=` 这类字样会**作为文本**留在输出里 ——
// 那是被中和的文本，不是活的属性。判据必须落在"有没有未被转义的 <"（即能否
// 组成标签）上，而不是落在这个子串在不在。原先那条 indexOf("onerror=") < 0
// 只是代理指标：它仅在 trace.task 不被渲染时才成立。下面三条是直接判据，
// 并且多一条正向证据 —— 载荷要"被转义后仍在"，而不是被悄悄丢掉。
assert(hostileAll.indexOf("<img") < 0 && hostileAll.indexOf("<script") < 0,
       "no unescaped tag boundary survives (no live handler is possible)");
assert(hostileAll.indexOf("&lt;img src=x onerror=alert(1)&gt;") >= 0,
       "the hostile task text is rendered escaped, not silently dropped");
assert(hostileAll.replace(/&lt;img src=x onerror=alert\(1\)&gt;/g, "")
        .indexOf("onerror=") < 0,
       "every surviving onerror= comes from a neutralised literal");
assert(hostileAll.indexOf("&lt;script&gt;") >= 0, "escaped text is present");

console.log("case 9: legacy backend and no-trace both get explicit empty states");
const legacy = T.render(Object.assign({}, trace, {
  candidates: [],
  selected_preference: { directive_kind: "legacy_scan", relative_heading_deg: 30,
                         reason_code: "legacy_scan_sequence", reason_zh: "按序扫描。",
                         evidence_refs: [] },
}), null);
assert(legacy.candidates.indexOf("不产生候选方向排名") >= 0,
       "legacy backend states there is no candidate audit");
assert(legacy.candidates.indexOf("legacy_scan_sequence") >= 0,
       "legacy reason code is shown");
assert(legacy.candidates.indexOf("<table") < 0, "no empty table is rendered");
[null, undefined, {}].forEach(function (missing) {
  const empty = T.render(missing, null);
  assertEqual(empty.mode, "暂无 Agent 决策记录", "no-trace mode is stated");
  ["safety", "plan", "choice", "downstream", "candidates", "knowledge",
   "inputs", "uncertainties"].forEach(function (section) {
    assert(empty[section].length > 0 && empty[section].indexOf("class=\"muted\"") >= 0,
           "no-trace " + section + " has an explicit empty state");
  });
});

console.log("case 10: every trace section is always present (UI never crashes)");
["mode", "safety", "plan", "choice", "downstream", "candidates", "knowledge",
 "inputs", "uncertainties"].forEach(function (section) {
  assert(typeof html[section] === "string" && html[section].length > 0,
         "section " + section + " is a non-empty string");
});
// 极端残缺的 trace：字段全缺，仍不许抛异常
const sparse = T.render({ schema_version: "task_agent_trace_v1" }, null);
assert(sparse.choice.indexOf("未提供") >= 0, "sparse trace says 未提供 everywhere");
assert(sparse.uncertainties.indexOf("没有记录到不确定项") >= 0,
       "sparse trace reports no uncertainties honestly");

console.log("case 11: history line carries the per-round agent evidence (§5.4)");
assertEqual(T.renderHistoryLine(null), "", "no agent block -> no extra line");
assertEqual(T.renderHistoryLine(undefined), "", "undefined agent -> no extra line");
const line = T.renderHistoryLine({
  trace_id: "directive_abc", cycle: 3, active_subtask: "搜索目标",
  directive_kind: "explore_unseen", relative_heading_deg: -30,
  reason_code: "partial_without_anchor", evidence_refs: ["obj_bin_001"],
  downstream_status: "accepted",
});
assert(line.indexOf("directive_abc") >= 0, "trace id is in the history row");
assert(line.indexOf("扫描序号 3") >= 0, "scan cycle is in the history row");
assert(line.indexOf("搜索目标") >= 0, "active subtask is in the history row");
assert(line.indexOf("partial_without_anchor") >= 0, "reason code is in the history row");
assert(line.indexOf("obj_bin_001") >= 0, "evidence refs are in the history row");
assert(line.indexOf("✔ 已采纳") >= 0, "downstream result is in the history row");
assert(line.indexOf("右前方") >= 0, "the chosen heading is spelled out");
const nullHistory = T.renderHistoryLine({ downstream_status: "pending" });
assert(nullHistory.indexOf("未提供") >= 0, "missing history fields say 未提供");
assert(new RegExp(T.esc('<x onerror="y">')).test(nullHistory) === false,
       "history line escapes hostile text");

console.log("case 12: raw task and goal graph reach the panel (§1.1 第 1、2 块)");
// task 块一直在 trace 里，但渲染层从没读过 —— 页面上看不到用户当初下的原话。
assert(html.plan.indexOf("原始任务") >= 0, "raw task text has its own row");
assert(html.plan.indexOf("找到白色垃圾桶") >= 0, "the raw task text is actually shown");
assert(html.plan.indexOf("标准化目标") >= 0, "canonical target has its own row");
// 目标图摘要必须让"裸目标"可见：1 个目标节点、0 条关系。这正是无锚点缺陷的形状，
// 面板要把它摆出来，而不是留给读者一句笼统的"已匹配到任务锚点"。
const bare = T.render(Object.assign({}, trace, {
  inputs: Object.assign({}, trace.inputs, {
    goal_graph: { task_id: "t", raw_query: "白色垃圾桶", target_node_id: "goal_target",
                  nodes: [{ node_id: "goal_target", label: "白色垃圾桶",
                            role: "goal_target" }],
                  edges: [], build_source: "target_profile", warnings: [] },
  }),
}), null);
assert(bare.inputs.indexOf("目标图") >= 0, "goal graph has a summary row");
// 光有标签不算数：这一行必须真的读到了数据，不能停在"未提供"。
assert(bare.inputs.indexOf('目标图</span><span class="v">未提供') < 0,
       "the goal row carries real data, not a bare 未提供");
assert(bare.inputs.indexOf("1 个目标节点") >= 0, "a bare target shows exactly one node");
assert(bare.inputs.indexOf("关系 0") >= 0,
       "a bare target shows zero relations instead of a vague 已匹配");
const rich = T.render(Object.assign({}, trace, {
  inputs: Object.assign({}, trace.inputs, {
    goal_graph: { task_id: "t", raw_query: "垃圾桶旁边的桌子", target_node_id: "goal_target",
                  nodes: [{ node_id: "goal_target", label: "桌子", role: "goal_target" },
                          { node_id: "anchor_bin", label: "唯一锚点甲", role: "anchor" }],
                  edges: [{ edge_id: "e1", source_node_id: "anchor_bin",
                            target_node_id: "goal_target", relation: "next_to" }],
                  build_source: "target_profile", warnings: [] },
  }),
}), null);
assert(rich.inputs.indexOf("2 个目标节点") >= 0, "both goal nodes are counted");
assert(rich.inputs.indexOf("关系 1") >= 0, "the relation count is shown");
// 锚点标签要选一个在别处不可能出现的字符串：用"垃圾桶"会被场景标签那一行
// 满足，断言就废了（第一版就是这么写的，注入 B 下它照样绿）。
assert(rich.inputs.indexOf("唯一锚点甲") >= 0, "the anchor label is listed");
// 新读的字段绝不能在残缺 trace 上渲染出 undefined
assert(sparse.inputs.indexOf("undefined") < 0, "sparse inputs never print undefined");
assert(sparse.plan.indexOf("undefined") < 0, "sparse plan never print undefined");
assert(html.inputs.indexOf("undefined") < 0, "full trace never prints undefined either");

console.log("case 13: every evidence bearing says how it was obtained");
// ±25 的粗推角与量出来的方位角在 trace 里长得一模一样（都是 heading_delta_deg），
// 只有 heading_source 能把它们分开。真机上目前 100% 是前者，所以这一行不是装饰：
// 没有它，面板上的每一个"证据方位"都会被读成测量值。
const withEvidence = T.render(Object.assign({}, trace, {
  selected_preference: Object.assign({}, trace.selected_preference, {
    evidence: [
      { source: "goal_graph", ref: "obj_bin_001", label: "垃圾桶",
        heading_delta_deg: -30.0, heading_source: "measured" },
      { source: "goal_graph", ref: "obj_chair_002", label: "办公椅",
        heading_delta_deg: 25.0, heading_source: "coarse_lr" },
      { source: "knowledge", ref: "kb:bin", label: "办公椅",
        heading_delta_deg: 25.0, heading_source: "coarse_lr" },
    ],
  }),
}), null);
assert(withEvidence.choice.indexOf("量出来的") >= 0,
       "a measured bearing is labelled as measured");
assert(withEvidence.choice.indexOf("按左右粗推") >= 0,
       "a left/right-derived bearing is labelled as such");
assert(withEvidence.choice.indexOf("知识库") >= 0,
       "the bearing's origin (knowledge) is named");
assert(withEvidence.choice.indexOf("垃圾桶 −30.0°") >= 0,
       "the bearing value itself is shown");
// 缺 heading_source 时必须如实写"来源未标注"，绝不默认成 measured ——
// 那会把一个写死的常数说成测量值，正是这个字段要防的事。
const noProvenance = T.render(Object.assign({}, trace, {
  selected_preference: Object.assign({}, trace.selected_preference, {
    evidence: [{ source: "goal_graph", ref: "r", label: "椅子",
                 heading_delta_deg: 25.0 }],
  }),
}), null);
assert(noProvenance.choice.indexOf("来源未标注") >= 0,
       "a missing provenance is labelled as unlabelled");
assert(noProvenance.choice.indexOf("量出来的") < 0,
       "a missing provenance never claims to be measured");
assert(noProvenance.choice.indexOf("undefined") < 0,
       "a missing provenance never prints undefined");
// 没有 evidence 键（旧 trace / legacy 后端）时给明确空态，不炸也不猜。
assert(html.choice.indexOf("无（本轮没有登记方位）") >= 0,
       "a trace without evidence gets an explicit empty state");

if (failures) {
  console.error("\n" + failures + " JS AGENT TRACE CONTRACT TEST(S) FAILED");
  process.exit(1);
}
console.log("\nALL JS AGENT TRACE CONTRACT TESTS PASSED");
