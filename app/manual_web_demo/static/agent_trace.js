/*
 * 任务 Agent 解释层的纯渲染层（计划书 §5）。
 *
 * 这里只做"trace -> HTML 字符串"的确定性变换，不碰 DOM、不发请求、不读全局。
 * 拆出来是为了能像 search_map.js 那样用 headless node 钉住展示规则：
 *
 *   §5.2 选择方向 = 文字 + 角度，且正负约定由页面写明，前端不自己猜左右。
 *   §5.2 缺失数据写"未提供"，绝不显示成绿色 0。
 *   §5.2 知识条目显示来源、置信度和 used / context_only。
 *   §5.2 候选行只显示代码真正用过的排序分量，没有虚构的综合成功率。
 *   §5.2 offline replay 用固定醒目标签。
 *   §5.2 计划状态用图标 + 文字共同表达，不能只靠颜色。
 *   §5.2 所有来自任务 / 标签 / 知识库的文本走 esc()。
 *
 * 角度约定与语义层一致：`_entity_heading_delta` 把 right/右 映射成负角，
 * 所以 + 是左转（逆时针），− 是右转（顺时针）。
 */
(function (global) {
  "use strict";

  var ANGLE_CONVENTION = "角度约定：+ 为左转（逆时针），− 为右转（顺时针）";

  // 证据从哪来（与方位怎么来是两件事，见下面的 headingSourceText）。
  var EVIDENCE_ORIGIN = {
    goal_graph: "目标图",
    knowledge: "知识库"
  };
  // 方位怎么来。两者含义差着量级：measured 是节点自己带的方位角，coarse_lr
  // 只是按左右推出来的一个写死常数（±25°，仓库里没有测量支持这个数）。
  // 真机上真正命中过的只有 coarse_lr —— 1036 个场景节点里 heading_delta_deg
  // 一个都没有。既然只有一个值会实际出现，标不标平时看不出差别；标出来是为了
  // 它一旦命中时不至于被当成测量值：两种来源产出的角度长得一模一样。
  var HEADING_SOURCE = {
    measured: "量出来的",
    coarse_lr: "按左右粗推"
  };
  var HEADING_SOURCE_UNKNOWN = "来源未标注";

  // 状态一律图标 + 文字，颜色只是附加信息。
  var PLAN_STATUS = {
    completed: { icon: "✔", text: "已完成" },
    active: { icon: "▶", text: "进行中" },
    pending: { icon: "○", text: "待执行" },
    blocked: { icon: "⛔", text: "已阻塞" },
    unsupported_by_current_executor: { icon: "⊘", text: "当前执行器不支持" }
  };
  var DOWNSTREAM_STATUS = {
    pending: { icon: "…", text: "待确认" },
    accepted: { icon: "✔", text: "已采纳" },
    rejected: { icon: "✖", text: "未采纳" },
    superseded: { icon: "↻", text: "已被替代" }
  };

  var EMPTY_SECTION =
    '<div class="muted">尚未收到带 decision_trace 的决策。' +
    "旧快照与旧事件没有该字段，属正常情况。</div>";

  function esc(value) {
    return String(value)
      .replace(/&/g, "&amp;")
      .replace(/</g, "&lt;")
      .replace(/>/g, "&gt;")
      .replace(/"/g, "&quot;");
  }

  // 数字一律 toFixed：rank_key 里是原始 float（如 26.420000000000016）。
  // 取不到就是 null，交给调用方写"未提供"，绝不用 0 冒充测量值。
  function num(value, digits) {
    if (value === null || value === undefined || value === "") return null;
    var parsed = Number(value);
    return isFinite(parsed) ? parsed.toFixed(digits) : null;
  }

  function headingText(deg) {
    if (deg === null || deg === undefined || deg === "") return "未提供";
    var value = Number(deg);
    if (!isFinite(value)) return "未提供";
    if (Math.abs(value) < 0.5) return "正前方（0°）";
    var magnitude = Math.abs(value);
    var side = value > 0 ? "左" : "右";
    var word = magnitude > 75
      ? "正" + side + "方"
      : side + (magnitude <= 45 ? "前方" : "侧方");
    return word + "（" + (value > 0 ? "+" : "−") + magnitude.toFixed(1) +
      "°，按当前坐标约定）";
  }

  // 证据那一行的紧凑角度：`headingText` 那句话在小表格里太长，这里只要度数。
  function bearingShort(deg) {
    var text = num(deg, 1);
    if (text === null) return "方位未提供";
    var value = Number(deg);
    return (value >= 0 ? "+" : "−") + Math.abs(value).toFixed(1) + "°";
  }

  // 认不出来源就如实写"来源未标注"，绝不默认成 measured —— 那会把一个写死的
  // 常数说成测量值，正是这个字段要防的事。
  function headingSourceText(source) {
    return HEADING_SOURCE[source] || HEADING_SOURCE_UNKNOWN;
  }

  function evidenceBearingText(items) {
    if (!items.length) return "无（本轮没有登记方位）";
    return items.map(function (item) {
      var label = item.label ? esc(item.label) + " " : "";
      return label + esc(bearingShort(item.heading_delta_deg)) + "（" +
        esc(headingSourceText(item.heading_source)) + " · " +
        esc(EVIDENCE_ORIGIN[item.source] || item.source || "来源未标注") + "）";
    }).join("、");
  }

  function clockText(ts) {
    if (ts === null || ts === undefined) return "未提供";
    var value = Number(ts);
    if (!isFinite(value) || value <= 0) return "未提供";
    return new Date(value * 1000).toLocaleTimeString();
  }

  function row(label, value) {
    return '<div class="agent-kv"><span class="k">' + esc(label) +
      '</span><span class="v">' + value + "</span></div>";
  }

  function yesno(value) {
    return value ? "是" : "否";
  }

  function statusText(table, status) {
    var meta = table[status] || { icon: "?", text: String(status || "未提供") };
    return meta.icon + " " + esc(meta.text);
  }

  function renderMode(trace, agent) {
    var offline = trace.mode === "offline_replay";
    return '<span class="agent-badge' + (offline ? "" : " live") + '">' +
      (offline ? "离线回放 · 非实机" : "实机 " + esc(trace.mode || "live")) +
      "</span>" +
      '<span class="agent-schema">' + esc(trace.schema_version || "未提供") +
      "</span>";
  }

  function renderSafety(trace, agent) {
    var safety = trace.safety || {};
    return row("安全边界", "可确认目标 " + yesno(safety.can_confirm_target) +
        " · 可授权运动 " + yesno(safety.can_authorize_motion) +
        " · 允许前进 " + yesno(safety.allow_forward)) +
      row("本轮 trace", esc(trace.trace_id || "未提供") +
        " · 更新于 " + esc(clockText(agent && agent.updated_at)));
  }

  function renderPlan(trace) {
    var stages = trace.plan || [];
    var active = trace.active_subtask || {};
    var task = trace.task || {};
    // §1.1 第 1 块要"原始任务与标准化目标"。task 块一直在 trace 里，
    // 只是从没被读过 —— 页面因此看不到用户当初下的那句原话。
    var html = (task.raw_text || task.canonical_target
      ? row("原始任务", task.raw_text ? esc(task.raw_text) : "未提供") +
        row("标准化目标", task.canonical_target
          ? esc(task.canonical_target) : "未提供")
      : "") + stages.map(function (stage) {
      return '<div class="agent-plan-row' +
        (stage.status === "active" ? " active" : "") + '">' +
        '<span class="agent-status">' + statusText(PLAN_STATUS, stage.status) +
        "</span><span>" + esc(stage.title || stage.id || "") + "</span>" +
        (stage.detail
          ? ' <span class="agent-detail">' + esc(stage.detail) + "</span>" : "") +
        "</div>";
    }).join("");
    if (active.title) {
      html += '<div class="agent-active-subtask">当前子任务：' + esc(active.title) +
        (active.reason ? " —— " + esc(active.reason) : "") + "</div>";
    }
    return html || '<div class="muted">未提供</div>';
  }

  function renderChoice(trace) {
    var preference = trace.selected_preference || {};
    var refs = preference.evidence_refs || [];
    return row("本轮选择", esc(preference.directive_kind || "未提供") + " · " +
        esc(headingText(preference.relative_heading_deg))) +
      row("依据", esc(preference.reason_zh || "未提供")) +
      row("reason code", esc(preference.reason_code || "未提供")) +
      row("证据引用", refs.length ? esc(refs.join("、")) : "无（本轮没有可引用的证据）") +
      row("证据方位", evidenceBearingText(preference.evidence || [])) +
      '<div class="agent-note">' + esc(ANGLE_CONVENTION) + "</div>";
  }

  function renderDownstream(trace, agent) {
    var status = (agent && agent.downstream_status) || "pending";
    var html = row("下游决定", statusText(DOWNSTREAM_STATUS, status) +
      (agent && agent.decision_id ? " · " + esc(agent.decision_id) : ""));
    if (status === "rejected") {
      // 只有下游真的给了原因才显示原因，否则如实写"未提供"，不猜（§5.3）。
      html += '<div class="agent-note warn">Agent 建议未成为最终决定：' +
        esc((agent && agent.downstream_reason) || "下游未提供拒绝原因") + "</div>";
    } else if (status === "pending") {
      html += '<div class="agent-note">下游尚未给出可核对的关联：本轮 trace ' +
        esc(trace.trace_id || "未提供") + " 还没有出现在最终候选的来源里。</div>";
    }
    return html;
  }

  function supportRefs(list) {
    return (list && list.length) ? esc(list.join("、")) : "—";
  }

  function rankKey(parts) {
    return (parts || []).map(function (part) {
      if (part === null || part === undefined) return "—";
      if (typeof part !== "number") return esc(part);
      return part % 1 === 0 ? String(part) : part.toFixed(2);
    }).join(" , ");
  }

  function renderCandidates(trace) {
    var rows = trace.candidates || [];
    if (!rows.length) {
      // legacy 后端只重放既有扫描序列，本来就没有候选排名 —— 给明确空态，
      // 而不是一张空表。
      var reasonCode = (trace.selected_preference || {}).reason_code;
      return '<div class="muted">该后端不产生候选方向排名' +
        (reasonCode ? "（reason code: " + esc(reasonCode) + "）" : "") +
        "，本轮没有可审计的候选分量。</div>";
    }
    var body = rows.map(function (candidate) {
      var penalty = num(candidate.negative_memory_penalty, 2);
      return '<tr class="' + (candidate.selected ? "selected" : "") + '">' +
        "<td>" + esc(headingText(candidate.relative_heading_deg)) +
          '<div class="muted">扇区 ' + esc(candidate.absolute_sector) + "</div></td>" +
        "<td>" + supportRefs(candidate.semantic_support_refs) + "</td>" +
        "<td>" + supportRefs(candidate.knowledge_support_refs) + "</td>" +
        "<td>" + supportRefs(candidate.auxiliary_support_refs) + "</td>" +
        "<td>" + (candidate.observed ? "是" : "否") + "</td>" +
        "<td>" + (penalty === null ? "未提供" : esc(penalty)) + "</td>" +
        '<td class="agent-rank">' + rankKey(candidate.rank_key) + "</td>" +
        "<td>" + (candidate.selected
          ? "✔ 选中"
          : "✖ " + esc((candidate.rejection_reasons || []).join("、") ||
                        "lower_rank")) + "</td></tr>";
    }).join("");
    return '<table class="agent-table"><tr><th>方向</th><th>语义</th><th>知识</th>' +
      "<th>辅助</th><th>已观测</th><th>负记忆</th>" +
      "<th>排序键（代码实际比较顺序）</th><th>结果</th></tr>" + body + "</table>" +
      '<div class="agent-note">' + esc(ANGLE_CONVENTION) +
      "。排序键就是代码逐项比较的分量与顺序（— 表示本轮没有该类证据），" +
      "没有虚构的综合成功率。</div>";
  }

  function renderKnowledge(trace) {
    var items = trace.knowledge || [];
    if (!items.length) {
      return '<div class="muted">本轮未检索到本地知识。' +
        '这只是"本轮没有"，不是负证据。</div>';
    }
    return items.map(function (item) {
      var used = item.status === "used";
      var confidence = num(item.confidence, 2);
      return '<div class="agent-know"><div><b>' +
        esc(item.content || item.id || "未提供") + "</b> " +
        '<span class="agent-status">' +
        (used ? "✔ 参与定向 (used)" : "○ 仅背景 (context_only)") + "</span></div>" +
        '<div class="obj-meta">来源 ' + esc(item.source || "未提供") +
        " · 置信度 " + (confidence === null ? "未提供" : esc(confidence)) +
        " · id " + esc(item.id || "未提供") + "</div>" +
        '<div class="obj-meta">' + esc(item.reason || "") + "</div>" +
        (used ? '<div class="obj-meta">支持候选：' +
          esc((item.supports || []).join("、")) + "</div>" : "") +
        "</div>";
    }).join("");
  }

  function renderInputs(trace) {
    var inputs = trace.inputs || {};
    var observation = inputs.observation || {};
    var memory = inputs.memory || {};
    var scene = inputs.scene_graph || {};
    var match = inputs.graph_match || {};
    var goal = inputs.goal_graph;
    var topology = inputs.topology;
    var sectors = observation.observed_heading_sectors || [];
    var labels = scene.labels || [];
    var yaw = num(observation.robot_yaw_deg, 1);
    var score = num(match.score, 3);
    var scanIndex = observation.scan_index;
    var html = row("图匹配", esc(match.state || "未提供") +
        (score === null ? "" : " · score " + esc(score)) +
        " · 匹配场景节点 " + (match.matched_scene_node_ids || []).length +
        " · 支撑锚点 " + (match.supporting_anchor_scene_node_ids || []).length) +
      row("观测", "scan_index " +
        esc(scanIndex === null || scanIndex === undefined ? "未提供" : scanIndex) +
        " · yaw " + (yaw === null ? "未提供" : esc(yaw) + "°") +
        " · 已观测扇区 " + (sectors.length ? esc(sectors.join("、")) : "无")) +
      row("场景", scene.node_count === undefined
        ? "未提供"
        : esc(scene.node_count) + " 个节点" +
          (labels.length
            ? "：" + esc(labels.slice(0, 12).join("、")) +
              (labels.length > 12 ? "…" : "")
            : "")) +
      row("记忆", "观察记忆 " +
        esc(memory.observation_memory_count === undefined
          ? "未提供" : memory.observation_memory_count) +
        " 条 · 负记忆 " + (memory.negative_memory_present ? "有" : "无")) +
      row("目标画像", memory.negative_target_key
        ? "规范名 " + esc(memory.negative_target_key)
        : "未提供") +
      // 目标图摘要：裸目标（无关系）在这里会显示成"1 个目标节点 · 关系 0"，
      // 正好把"没有锚点可查"这件事摆到台面上，而不是留一句笼统的"已匹配"。
      row("目标图", goal
        ? esc((goal.nodes || []).length) + " 个目标节点 · 关系 " +
          esc((goal.edges || []).length) + " · 构建来源 " +
          esc(goal.build_source || "未提供") +
          ((goal.nodes || []).length
            ? "：" + esc((goal.nodes || []).map(function (node) {
                return node.label || node.node_id;
              }).slice(0, 8).join("、"))
            : "") +
          ((goal.warnings || []).length
            ? " · 警告 " + esc((goal.warnings || []).join("、")) : "")
        : "未提供") +
      row("拓扑摘要", topology
        ? "地点 " + esc(topology.place_count) + " · 关系 " +
          esc(topology.edge_count) + " · 当前 " +
          esc(topology.current_place_id || "未提供")
        : "未提供（第一阶段不参与方向排序）");
    var missing = inputs.missing_sources || [];
    return html + row("缺失来源", missing.length ? esc(missing.join("、")) : "无");
  }

  function renderUncertainties(trace) {
    var items = trace.uncertainties || [];
    if (!items.length) {
      return '<div class="muted">本轮没有记录到不确定项。</div>';
    }
    return items.map(function (item) {
      return row(item.code || "未提供", esc(item.detail || ""));
    }).join("");
  }

  /** 每轮历史里的 Agent 依据行（§5.4）。无 agent 字段时返回空串。 */
  function renderHistoryLine(agent) {
    if (!agent) return "";
    var refs = (agent.evidence_refs || []).length
      ? " · 证据 " + esc((agent.evidence_refs || []).join("、")) : "";
    return '<div class="decision-history-agent">' +
      '<span class="agent-schema">' + esc(agent.trace_id || "未提供") + "</span>" +
      " · 扫描序号 " +
      esc(agent.cycle === null || agent.cycle === undefined ? "未提供" : agent.cycle) +
      " · " + esc(agent.active_subtask || "未提供") +
      " · " + esc(agent.directive_kind || "未提供") + " " +
      esc(headingText(agent.relative_heading_deg)) +
      " · " + esc(agent.reason_code || "未提供") +
      " · 下游 " + statusText(DOWNSTREAM_STATUS, agent.downstream_status) + refs +
      "</div>";
  }

  /** 把一份 trace 变成各分区的 HTML。没有 trace 时给统一明确空态。 */
  function render(trace, agent) {
    // `schema_version` 是这个信封的身份字段。没有它就不是一份 trace：
    // 一个 {} 若走正常分支，会因为 `trace.mode || "live"` 反过来宣称"实机"，
    // 那是拿未知冒充已知。宁可显示明确空态。
    if (!trace || !trace.schema_version) {
      return {
        mode: "暂无 Agent 决策记录",
        safety: EMPTY_SECTION, plan: EMPTY_SECTION, choice: EMPTY_SECTION,
        downstream: EMPTY_SECTION, candidates: EMPTY_SECTION,
        knowledge: EMPTY_SECTION, inputs: EMPTY_SECTION,
        uncertainties: EMPTY_SECTION,
      };
    }
    return {
      mode: renderMode(trace, agent),
      safety: renderSafety(trace, agent),
      plan: renderPlan(trace),
      choice: renderChoice(trace),
      downstream: renderDownstream(trace, agent),
      candidates: renderCandidates(trace),
      knowledge: renderKnowledge(trace),
      inputs: renderInputs(trace),
      uncertainties: renderUncertainties(trace),
    };
  }

  global.AgentTrace = {
    ANGLE_CONVENTION: ANGLE_CONVENTION,
    PLAN_STATUS: PLAN_STATUS,
    DOWNSTREAM_STATUS: DOWNSTREAM_STATUS,
    esc: esc,
    num: num,
    headingText: headingText,
    clockText: clockText,
    render: render,
    renderHistoryLine: renderHistoryLine,
  };
})(typeof window !== "undefined" ? window : this);
