# GO2W 任务 Agent 与自主搜索决策链 WebUI 离线改造实施计划

> 用途：把本文原样交给接手本仓库的 AI。该 AI 应按本文从基线取证、实现、测试、注错校准到离线 A/B 验收一次性完成，不需要重新发明方案。
>
> 工作目录：`/home/mxt/robotscene`
>
> 指定分支：`fix/go2w-autonomous-search-lio-topology-target`
>
> 核心目标：在完全离线、不连接机器狗、不产生运动指令、不调用付费 API 的条件下，让自主搜索在决定“下一步往哪走”前汇总任务、知识库、目标图、场景图、观测覆盖、负记忆和可获得的拓扑信息；用这些证据选择下一步搜索偏好；并在 WebUI 中展示任务总体计划、当前步骤、候选方向、选择依据、知识来源、不确定性以及最终下游导航决定。

---

## 0. 给执行 AI 的开工指令

接手后必须严格按以下顺序开始，不能直接改代码：

1. 进入 `/home/mxt/robotscene`，确认当前分支正好是 `fix/go2w-autonomous-search-lio-topology-target`。
2. 完整阅读 `docs/GO2W_给外部AI的离线改造任务书.md`，不能只读摘要或用搜索命中代替全文阅读。
3. 继续阅读该任务书引用的以下验收章节：
   - `docs/GO2W_断电中断与验收A完成交接_20260916.md` 的 §19。
   - `docs/GO2W_真机验收_交接书.md` 的 §6.5。
4. 执行 `git status --short`，把开工时已有的修改和未跟踪文件记入交付报告。这些内容均视为用户资产，不得删除、覆盖、回退、加入本任务提交或借机整理。
5. 给 §2.1 的冻结目录和文件生成开工 SHA-256 清单，清单写入 `/tmp`，不要写入业务目录。
6. 使用指定 Python 环境跑基线测试与真实录制 A/B，确认测试基础设施和录制语料与交接文档一致。
7. 只有基线成立后，才按本文阶段实施。

指定 Python：

```bash
PY=/home/mxt/anaconda3/envs/go2_robot_scene_demo/bin/python
```

每一条 Python 测试、评估、注错以及恢复后的复测命令都必须使用新的字节码缓存目录：

```bash
PYTHONPYCACHEPREFIX="$(mktemp -d)" "$PY" ...
```

不能用 `python -B` 代替。`-B` 不能阻止解释器读取已经存在的陈旧 `.pyc`。

---

## 1. 本次要交付的产品结果

### 1.1 用户在 WebUI 中应当看到什么

搜索页面新增一个“任务 Agent / 决策依据”区域，至少包含以下五块：

1. **总体任务计划**
   - 原始任务与标准化目标。
   - 任务分解步骤。
   - 每个步骤的状态：`已完成 / 进行中 / 等待 / 受阻 / 当前系统不支持执行`。
   - 当前激活的子任务及进入该子任务的原因。

2. **本轮可用上下文**
   - 目标画像和目标图摘要。
   - 当前场景图中的实体、方位及置信度摘要。
   - 已观测方向、当前航向、扫描序号。
   - 正/负观测记忆及其新鲜度。
   - 当前可获得的拓扑摘要，例如当前地点、相邻地点或前沿数量；如果此层没有数据，必须明确显示“本轮未提供”，不得伪造。

3. **知识库证据**
   - 实际检索到的知识项。
   - 来源文件或来源类型。
   - 原始置信度。
   - 本轮是否被采用。
   - 对哪个候选方向产生了什么影响。
   - 如果检索结果为零，显示“未检索到相关本地知识”，不能把缺失数据写成负证据。

4. **候选方向与排序依据**
   - 每个候选相对航向。
   - 对应绝对方向扇区。
   - 是否已观测。
   - 负记忆惩罚。
   - 场景图、目标图、知识库和辅助提示分别提供了哪些支持。
   - 排序键和未入选原因。
   - 被选中的“搜索偏好”，以及它是否已经被下游导航接受。

5. **实际下游结果**
   - 现有导航候选和最终 `DECISION_RECORDED` 继续照常显示。
   - 明确区分“推理层建议方向”“导航层候选”“真正发给执行器的决定”。
   - 离线运行必须醒目标注 `OFFLINE / REPLAY / NO MOTION`。
   - 如果安全层或候选生成器拒绝了推理层建议，必须显示拒绝原因，不能只让候选静默消失。

### 1.2 系统行为应发生什么变化

当前已知失败链条是：仅匹配到目标本身时，图匹配状态可能为 `PARTIAL`，但没有可用锚点；推理器仍生成 `inspect_anchor` 和固定的 `+30°`；冻结的候选生成器正确拒绝无锚点指令；语义候选因此消失，最终常由 `local_scan` 获胜。

本次把行为锁定为：

- `PARTIAL` 但没有受证据支持的锚点时，绝不生成 `inspect_anchor(anchor=None)`。
- 不再用无依据的固定 `+30°` 作为该分支的默认答案。
- 该情况统一回落到证据感知的 `EXPLORE_UNSEEN` 排序，并在决策依据中写明：`仅目标节点形成 PARTIAL，缺少可检查锚点，因此转入未观测方向探索`。
- `ZERO`、`PARTIAL 无锚点` 和需要重新观测的分支共享同一个候选方向评估器，避免三套互相矛盾的方向规则。
- 当目标图上下文、用户显式锚点或本地知识能够与场景中的可见实体建立可追溯关联时，允许它们参与方向排序。
- 当没有这种关联时，知识只能展示为背景信息，不能凭常识虚构方向。
- 推理层只给出搜索偏好，不确认目标、不绕过安全层、不直接授权前进。
- 离线评估期间所有输出必须保持 `allow_forward=false`，且不能进入真实运动发送路径。

### 1.3 “思维链”的产品定义

这里实现的是**结构化、可审计的决策依据**，不是输出模型的私有隐藏思维链，也不是一段不可验证的长篇自然语言自言自语。

每一步依据都必须能还原到以下一种可检查来源：

- 用户任务或解析后的目标/约束。
- 目标图节点或边。
- 场景图实体、位置或置信度。
- 本地知识库条目。
- 观测覆盖。
- 正/负记忆。
- 录制数据中的姿态、航向和扇区。
- 现有安全规则与下游候选拒绝原因。

自然语言说明只负责把结构化字段翻译成人能读懂的话，不得成为唯一证据。

---

## 2. 不能突破的边界

### 2.1 冻结清单

必须以 `docs/GO2W_给外部AI的离线改造任务书.md` §1.1 的原文为最终准绳。至少包括以下内容：

- 3D 建图相关实现。
- `min_local_map_matching_rate`。
- LIO 滤波链。
- `app/perception/depth_object_localizer.py`。
- `app/spatial/semantic_object_map.py`。
- `app/slam/slam_bridge.py`。
- `app/spatial/slam_web_map_state.py`。
- `ros2_ws/src/plain_slam_ros2`。
- `ros2_ws/src/go2w_plain_slam_bridge`。
- **整个 `app/navigation/` 目录。**

特别容易误改、但绝对禁止修改的文件：

- `app/navigation/candidate_goal_generator.py`
- `app/navigation/local_scan.py`
- `app/navigation/decision_models.py`
- `app/navigation/decision_record.py`

可以给冻结代码补或运行测试，也可以只读调用其公开接口，但不能修改它。最终交付前必须用开工 SHA-256 清单复核。

### 2.2 绝对禁止的外部动作

- 不连接机器狗。
- 不对 `192.168.123.18` 做 `ping`、`ssh`、`curl` 或任何探测。
- 不启动真实运动控制节点。
- 不调用运动发送接口。
- 不调用 OpenAI、Claude 或其他付费/联网模型 API。
- 不下载新数据集，不用联网信息替代本地录制。
- 不输出 `.env`、令牌或其他秘密。
- 不运行会触发网络视频等待的完整 `pytest tests/`。

### 2.3 Git 与已有工作区纪律

- 禁止 `git add .`。
- 禁止 `git reset`。
- 禁止 `git clean`。
- 禁止 `git checkout -- ...`。
- 不得修改、删除或提交开工前已有的脏文件和未跟踪文件。
- 未经用户明确要求，不创建提交。
- 如用户要求提交，只能显式列出本任务文件；提交说明按原任务书要求附加：

```text
Co-Authored-By: Claude Code <noreply@anthropic.com>
```

### 2.4 判据纪律

以下内容不能算作成功证据：

- 只看到“没有检出问题”。
- 只看到一次测试为绿。
- 测试因为节点名拼错、没有收集到用例、导入失败或前置条件未触发而退出。
- 只证明新旧结果不同，却没有证明新结果与可见证据更一致。
- 为了让数字好看临时添加阈值、权重或放宽断言。

每一项新验收判据都要遵守：

1. 原实现或正确实现下，测试为绿。
2. 对被保护的行为做最小定向注错。
3. 测试必须因为目标断言失败而变红。
4. 验证不是 `not found`、`no tests ran`、收集错误或其他假红。
5. 恢复文件，核对 SHA-256。
6. 使用新的 `PYTHONPYCACHEPREFIX` 再跑一次，必须重新变绿。

凡是加入新的数值阈值或排序权重，都必须先用真实录制做经验校准；未完成校准就不能合入。为了避免引入无法证明的数值，本计划优先使用可解释的字典序排序和现有字段。

### 2.5 本次明确不声称证明的事项

离线验收只能证明：

- 推理输入更完整。
- 方向选择会对证据变化做出可复现响应。
- 无锚点指令不会再被静默生成后丢弃。
- WebUI 能完整展示结构化依据。
- 冻结导航消费者能够接受合法指令。
- 不会在离线评估中产生运动授权。

离线验收不能证明：

- 真机一定走得更快或更安全。
- 新方向在所有现场布局中都是最优路径。
- 实际电机、底盘、LIO 和网络时延均正常。
- 语义检测器在实时运行中的延迟和召回率已经提高。

交付报告必须保留这些限制，不能把离线改善写成真机成功。

---

## 3. 当前链路与改造落点

### 3.1 当前决策链

```text
任务文本
  -> SearchTaskContext / TargetProfile
  -> GoalGraphBuilder
  -> SceneGraph + GraphMatcher
  -> SearchReasoner
  -> SearchDirective
  -> app/live_robot/search_directive_adapter.py
  -> 冻结的 app/navigation/candidate_goal_generator.py
  -> 导航候选与安全选择
  -> DECISION_RECORDED
  -> SearchStateStore
  -> WebUI
```

真实入口主要在 `scripts/go2w/run_semantic_exploration.py`。真实录制离线评估入口为 `scripts/evaluate_live_search_reasoners.py`，决策对比入口为 `scripts/compare_reasoner_decisions.py`。

### 3.2 两类候选必须分开

系统中存在两类含义不同的候选，UI 和数据模型不能混为一谈：

| 层级 | 含义 | 本次是否可改 |
|---|---|---|
| 推理候选 | `SearchReasoner` 比较的相对方向或语义指令 | 可以，位于 `app/reasoning/` |
| 导航候选 | 候选目标生成器输出的可走目标、局部扫描或前沿 | 只读，`app/navigation/` 冻结 |
| 最终决定 | 安全和选择逻辑记录的实际决定 | 不改变模型，只在 UI 关联展示 |

推理 Agent 可以建议“优先查看左前方的桌子附近”，但不能把它包装成“机器人已经选择了左前方路径”。只有后续 `DECISION_RECORDED` 才代表下游实际决定。

### 3.3 最小改造原则

不建立新的大型规划框架，不重写导航栈，不复制一套场景图，不为测试创建生产配置开关。最小目标是：

1. 在允许修改的推理层增加一个确定性的任务 Agent/决策审计器。
2. 在 `SearchDirective` 中携带一个可选、JSON 安全的 `decision_trace`。
3. 让运行脚本和离线评估器向推理上下文传入已经存在的数据。
4. 让状态存储和 WebUI 展示该 trace。
5. 保持冻结的导航接口不变，用测试证明新指令可被现有消费者接受。

---

## 4. 已锁定的产品与技术决定

执行 AI 不需要再向用户询问以下选择。

### 4.1 图匹配语义不在第一轮修改

暂不改变 `GraphMatcher` 对 `ZERO / PARTIAL / MATCHED` 的定义。原因是 `PARTIAL` 可能仍对其他任务有价值，直接重定义会扩大影响面。

修复点放在 `SearchReasoner`：只有存在可用 `anchor_node_id`、方位或位置证据时，`PARTIAL` 才能产生 `inspect_anchor`；否则回落到证据感知探索。

### 4.2 不使用固定 `+30°` 作为无锚点默认方向

所有需要探索的分支调用同一个候选方向排序器。候选集合沿用现有离散方向，除非代码当前集合已有变化：

```text
-30, +30, -60, +60, -90, +90
```

不要为了本任务新添角度、角度阈值或转向权重。

### 4.3 排序采用可解释字典序，不引入未经校准的新加权总分

每个候选方向生成结构化组件，再按以下优先级排序：

1. 现有安全/有效性约束。
2. 显式任务锚点或目标图上下文与可见实体之间的方向支持。
3. 可追溯的本地知识关联与可见实体之间的方向支持。
4. 是否未观测。
5. 现有负记忆惩罚。
6. 现有辅助提示分值。
7. 较小的转向幅度作为最后的确定性 tie-breaker。

实现时必须先核对当前 `_best_unseen_heading` 的实际排序键，尽量在现有键上增加字段，而不是重写全部逻辑。任何改变上述顺序的决定都要有真实录制的对照证据。

UI 可以展示每项数值，但不能把字典序组件相加后伪装成“成功概率”。

### 4.4 知识库只有在可落到当前视觉证据时才影响方向

知识分三种状态：

- `used`：知识中的相关对象与本轮场景实体匹配，并且该实体具有可用方位，因此影响了某个方向。
- `context_only`：知识与目标相关，但本轮没有可定位实体，只展示，不参与方向选择。
- `not_retrieved`：没有相关知识；这是“缺失”，不是“该方向不可能”。

不得凭空加入“垃圾桶通常在门后面”等仓库里不存在的常识。

### 4.5 拓扑信息第一阶段只作为上下文和下游关联证据

如果当前推理调用点可以只读获得地点、相邻节点、前沿或图版本，则写入 trace 并在 UI 展示。如果没有，就明确记录缺失。

除非真实录制校准能证明某个拓扑量具有分辨力，否则本次不新增“前沿距离小于 X”“节点分数大于 Y”一类规则。实际路径可达性仍由冻结导航层负责。

### 4.6 任务 Agent 是确定性的本地状态机，不是联网 LLM

任务 Agent 不调用任何模型 API。它根据现有 `SearchTaskContext` 和搜索状态生成计划快照。

默认计划阶段：

1. `UNDERSTAND_TASK`：解析目标、属性、关系和限制。
2. `RETRIEVE_CONTEXT`：加载本地知识、目标图、场景图、记忆和拓扑摘要。
3. `SEARCH_OR_EXPLORE`：选择本轮语义搜索偏好。
4. `VERIFY_TARGET`：等待视觉判据确认，知识不能代替检测。
5. `REPORT_RESULT`：仅在已有任务结束事件后完成。

如果 `SearchTaskContext.subtasks` 含多个子任务，全部列入总体计划。当前执行链不能安全执行的子任务必须标记为 `unsupported_by_current_executor` 或 `blocked`，不得假装已经调度或完成。此次不扩建多机器人或通用工作流引擎。

---

## 5. 数据契约

### 5.1 `SearchReasoningContext` 的最小扩展

在 `app/reasoning/semantic_navigation/models.py` 中，仅按需要新增以下可选字段；默认值必须保证现有调用者兼容：

```python
task_context: dict[str, Any] | None
knowledge_items: list[dict[str, Any]]
topology_summary: dict[str, Any] | None
run_mode: str  # live / offline_replay / test
```

要求：

- 全部可 JSON 序列化。
- 缺失和空值语义明确。
- 修复 `SearchReasoningContext.to_dict()` 中对未定义 `observation_memory_ids` 的引用，并用测试校准。
- 不在上下文中复制整帧图像、深度图或无法控制体积的原始日志。

### 5.2 `SearchDirective` 的最小扩展

新增一个可选字段：

```python
decision_trace: dict[str, Any] | None = None
```

不能改动冻结导航消费者依赖的原有字段语义。额外字段必须可被旧消费者忽略。

### 5.3 `decision_trace` 推荐结构

字段名可以在实现时按仓库风格微调，但语义不能丢失：

```json
{
  "schema_version": "task_agent_trace_v1",
  "trace_id": "与本轮 directive 可关联的 ID",
  "mode": "offline_replay",
  "task": {
    "raw_text": "...",
    "canonical_target": "...",
    "attributes": [],
    "relations": [],
    "constraints": []
  },
  "plan": [
    {
      "id": "understand_task",
      "title": "理解任务",
      "status": "completed",
      "detail": "...",
      "capability": "supported"
    }
  ],
  "active_subtask": {
    "id": "search_or_explore",
    "title": "搜索目标",
    "reason": "目标尚未通过视觉判据确认"
  },
  "inputs": {
    "graph_match": {},
    "goal_graph": {},
    "scene_graph": {},
    "observation": {},
    "memory": {},
    "topology": {},
    "missing_sources": []
  },
  "knowledge": [
    {
      "id": "...",
      "content": "...",
      "source": "data/scene_kb/...",
      "confidence": 0.0,
      "status": "used | context_only",
      "supports": ["candidate-id"],
      "reason": "..."
    }
  ],
  "candidates": [
    {
      "id": "heading:+30",
      "relative_heading_deg": 30.0,
      "absolute_sector": 0,
      "observed": false,
      "negative_memory_penalty": 0.0,
      "semantic_support_refs": [],
      "knowledge_support_refs": [],
      "auxiliary_support_refs": [],
      "rank_key": [],
      "selected": true,
      "rejection_reasons": []
    }
  ],
  "selected_preference": {
    "directive_kind": "explore_unseen",
    "relative_heading_deg": 30.0,
    "reason_code": "partial_without_anchor",
    "reason_zh": "...",
    "evidence_refs": []
  },
  "uncertainties": [],
  "safety": {
    "can_confirm_target": false,
    "can_authorize_motion": false,
    "allow_forward": false
  },
  "downstream": {
    "status": "pending | accepted | rejected | superseded",
    "decision_id": null,
    "reason": null
  }
}
```

规则：

- `rank_key` 展示实际代码使用的分量和顺序，不能重新算一套只供 UI 好看的解释。
- `evidence_refs` 必须能指回 trace 中的知识、场景节点、目标节点或记忆记录。
- 未知值用 `null` 或 `missing_sources`，不能填 `0` 冒充测量结果。
- 文本字段进入 UI 时必须按文本转义，不能使用不受控的 `innerHTML` 拼接。

### 5.4 状态存储契约

不新增一套平行的 WebSocket 协议。优先复用当前 `TARGET_MATCH_UPDATED` 中已经携带的 `directive`：

1. `explorer_search_adapter.py` 保证 `directive.decision_trace` 不被丢弃。
2. `SearchStateStore._empty_snapshot()` 增加 `agent` 默认结构。
3. `_on_target_match()` 从 directive 提取 trace，更新 `snapshot.agent`。
4. `_on_decision_recorded()` 把最近的真实 `decision_id` 与最新 trace 关联，并更新 `downstream.status`；不修改冻结的 `DecisionRecord` 模型。
5. 页面重连后从 snapshot 完整恢复 Agent 面板，而不是只依赖瞬时事件。

只有在现有事件无法无损传递时才考虑新增事件类型；如果复用现有事件可行，就不要增加 `AGENT_UPDATED`。

---

## 6. 文件级改造清单

下表是允许范围，不代表每个文件必须修改。能少改就少改。

| 文件 | 预期改动 | 验收点 |
|---|---|---|
| `app/reasoning/semantic_navigation/models.py` | 上下文可选字段、`decision_trace`、JSON 序列化修复 | 旧调用兼容，`to_dict()` 可运行 |
| `app/reasoning/semantic_navigation/search_reasoner.py` | 无锚点回落、统一方向候选、证据感知字典序、trace 原始分量 | 不再产生空锚点 inspect，离线不授权运动 |
| `app/reasoning/semantic_navigation/task_agent.py`（建议新增，最多一个新核心模块） | 生成总体计划、上下文摘要、候选审计和自然语言说明 | 纯函数/确定性/无 API |
| `app/knowledge/retrieval.py` 或其现有数据装配位置 | 仅在现有 `KnowledgeItem.metadata` 不足时补充来源和关系元数据 | 不改原始置信度，不创造知识 |
| `scripts/go2w/run_semantic_exploration.py` | 把已有 task、知识和只读拓扑摘要送入 reasoning context | 不碰运动控制逻辑，不改变冻结消费者 |
| `scripts/evaluate_live_search_reasoners.py` | 用与生产一致的上下文构造方式，输出 trace 统计和证据对齐结果 | 仍只回放录制，无 API/运动 |
| `scripts/compare_reasoner_decisions.py` | 仅在需要时增加忽略随机 ID、比较 trace 关键字段 | baseline/baseline 必须仍为 0 |
| `app/live_robot/explorer_search_adapter.py` | 仅确保 trace 透传；若已自动透传则不改 | 增量事件含 trace |
| `app/live_robot/search_state_store.py` | 持久化 agent 快照，关联后续决定 | 重连可恢复 |
| `app/manual_web_demo/templates/index.html` | Agent 面板结构 | 无数据时有明确空态 |
| `app/manual_web_demo/static/search_ui.js` | 渲染计划、知识、候选、依据、下游状态 | 转义、防空值、兼容旧快照 |
| `app/manual_web_demo/static/style.css` | 最少必要样式 | 小屏可读，不遮挡现有地图/控制区 |
| 定向测试文件 | 新行为、状态、UI、冻结消费者契约 | 每条新判据做注错校准 |

### 6.1 不得借机进行的改动

- 不把 `DecisionRecord` 升级为新版本。
- 不把 Agent trace 塞进 `app/navigation/decision_models.py`。
- 不重构 `local_scan`。
- 不调整 `candidate_goal_generator` 的门槛或评分。
- 不修改 LIO、SLAM、深度定位或语义地图。
- 不建立数据库或新消息总线。
- 不引入前端框架。
- 不加入新的联网依赖。
- 不通过改真实录制数据来迎合测试。

---

## 7. 分阶段实施步骤

## 阶段 0：基线、录制语料和冻结清单

### 0.1 记录工作区

交付报告记录：

- 分支名和 `HEAD`。
- `git status --short`。
- Python 解释器路径。
- 本地时间。
- 已有用户修改清单。

不得为获得“干净工作区”而清理任何东西。

### 0.2 冻结文件哈希

对 §2.1 全部冻结目标生成 SHA-256 清单，存入类似：

```text
/tmp/go2w_frozen_before.sha256
```

结束时在同样文件集合上重新计算并逐项比较。若任何哈希变化，立即停止交付并恢复本任务造成的改动；不能带着变化继续验收。

### 0.3 基线测试

先跑与本次路径直接相关的定向测试，再跑交接书指定筛选：

```bash
PYTHONPYCACHEPREFIX="$(mktemp -d)" "$PY" -m pytest \
  tests/test_semantic_navigation_auxiliary_hints.py \
  tests/test_semantic_navigation_graph_matcher.py \
  tests/test_evaluate_live_search_reasoners.py \
  tests/test_search_state_store_spatial.py \
  tests/test_autonomous_search_websocket.py -q

PYTHONPYCACHEPREFIX="$(mktemp -d)" "$PY" -m pytest tests/ -q \
  -k "reason or semantic or navigation or goal_graph or graph_match or exploration or shadow or replay or local_scan"
```

如果实际文件名不同，先用 `rg --files tests` 查明现有文件，不能用“文件没找到”当作通过。

### 0.4 真实录制 A/B 基线

按 `docs/GO2W_给外部AI的离线改造任务书.md` 中现有的 57 条 full SceneGraph 会话做完整回放。预期当前语料为：

- 可用：57。
- 按规则拒绝：45。
- 现有指令分布约为：`explore_unseen=49`、`inspect_anchor=4`、`reobserve=4`。

必须检查每条回放都真实注入了：

- `replay_context`。
- 当前 yaw。
- observed sectors。
- 对应 SceneGraph。

如果数量或输入分布与交接书不一致，先解释语料漂移；不能默默换成较小子集，也不能直接沿用旧数字。

用 `scripts/compare_reasoner_decisions.py` 先做 baseline 对 baseline，对比结果必须为 0。不得使用 `diff -r`，因为随机 UUID 会产生假差异。

基线结果全部写入 `/tmp/go2w_task_agent_baseline/`，避免污染仓库已有输出。

## 阶段 1：先建立有分辨力的测试

### 1.1 无锚点 `PARTIAL` 测试

构造最小上下文：

- 目标图只有目标，或匹配结果为 `PARTIAL`。
- `anchor_node_id=None`。
- 提供 yaw、observed sectors 和负记忆。

断言：

- directive 不是 `inspect_anchor`。
- directive 为合法的 `explore_unseen` 或当前仓库等价枚举。
- `anchor_node_id` 不被伪造。
- `allow_forward=false`。
- `reason_code=partial_without_anchor`。
- 候选排序使用统一 ranker，而不是硬编码 `+30`。

注错校准：临时恢复“PARTIAL 无条件 inspect_anchor”的错误逻辑，该测试必须因 directive kind/anchor 断言变红。

### 1.2 证据敏感性成对测试

构造两个除证据方向外完全相同的上下文：

- A：相关可见实体在左侧。
- B：同一实体在右侧。
- yaw、observed sectors、负记忆和任务均相同。

断言选择方向随证据镜像变化，且 `evidence_refs` 指向对应实体/知识。

再构造一个删除关联知识或上下文边的控制组，断言该知识不再被标为 `used`。不能只断言“结果非空”。

注错校准：临时忽略语义/知识支持字段，成对测试必须因为左右选择不再变化而变红。

### 1.3 缺失信息语义测试

断言：

- 没有知识项时，trace 记录 `knowledge=[]` 和明确缺失/零检索状态。
- 没有拓扑摘要时记录 `topology=null` 或 `missing_sources`。
- 不把未知位置写成 `(0, 0)`。
- 不把缺失置信度写成 `0.0`。

注错校准：临时把缺失拓扑伪装成空的有效测量，测试必须变红。

### 1.4 序列化和兼容测试

断言：

- `SearchReasoningContext.to_dict()` 在观测记忆为空和非空时都可运行。
- `SearchDirective.to_dict()` 包含可选 trace。
- 老调用者未传新字段时仍可创建对象。
- trace 可由标准 `json.dumps()` 序列化。

注错校准：临时移除 trace 或恢复未定义变量，目标测试必须变红。

### 1.5 冻结消费者契约测试

只调用、不修改 `app/navigation/candidate_goal_generator.py`：

- 用新 reasoner 生成一条合法 directive。
- 把它送入当前冻结消费者。
- 断言所需字段完整、不会因为 `anchor=None` 的非法 inspect 而静默消失。
- 如果消费者因为地图或安全条件正常拒绝，应断言获得可解释的拒绝结果，而不是强行要求生成路径。

注错校准：将 fixture 中 directive 改为 `inspect_anchor(anchor=None)`，测试必须识别为无效或被拒绝。

### 1.6 状态存储与 WebUI 契约测试

断言：

- `TARGET_MATCH_UPDATED` 后 `snapshot.agent` 保存 trace。
- 页面重连取得的完整 snapshot 仍包含计划、知识和候选。
- `DECISION_RECORDED` 后，最新 trace 能关联到 downstream decision。
- 旧事件不带 trace 时页面保持可用并显示空态。
- 前端把恶意标签按文本展示，不执行 HTML。

注错校准：分别临时去掉 store 更新和前端 render 调用，对应测试必须变红。

## 阶段 2：实现推理层修复

### 2.1 统一探索候选生成

在 `search_reasoner.py` 中抽取或整理一个内部函数，返回：

- 选中 heading。
- 每个 heading 的实际排序分量。
- 证据引用。
- 未入选原因。

`ZERO`、`PARTIAL 无锚点` 和其他需要探索的路径复用它。不要在多个分支复制同一排序逻辑。

### 2.2 有效锚点判定

`inspect_anchor` 至少要求：

- 非空锚点 ID。
- 锚点能追溯到目标图/场景图匹配。
- 存在能转换成相对方向的空间信息，或现有合法 heading。

如果任何必要字段缺失，回落到探索，并把缺失项写入 uncertainties。

不要新加一个“置信度大于 X”的门槛，除非先完成真实录制校准。

### 2.3 场景与知识方向支持

方向支持只来自可追溯关系：

1. 用户显式关系，例如“桌子旁的白色垃圾桶”。
2. 目标图中的上下文节点。
3. 本地知识条目中的相关对象，且该对象实际出现在本轮场景图中。
4. 已存在的 auxiliary hints。

方位转换沿用项目当前角度约定，先用现有 `_anchor_heading` 或等价逻辑确认正负方向，不要凭直觉反转。

中心、未知或无法离散到现有候选的实体只进入 trace，不影响选择。

### 2.4 保持安全不变量

无论候选如何排序：

- 推理器不设置目标已确认。
- 推理器不把知识置信度当成检测置信度。
- 离线 trace 的 `can_authorize_motion=false`。
- 离线 directive 的 `allow_forward=false`。
- 运动是否执行仍完全由现有下游安全链决定。

## 阶段 3：实现确定性任务 Agent 和决策审计

### 3.1 `task_agent.py`

建议只新增这一个小模块，职责限定为：

- 根据 `SearchTaskContext` 生成总体计划。
- 标记当前步骤和系统能力边界。
- 汇总已经传入的上下文，不自己读取全局状态。
- 根据 reasoner 的真实候选分量生成 `decision_trace`。
- 生成简短中文说明和不确定性列表。

它不能：

- 决定底盘速度。
- 调用网络或 LLM。
- 修改地图。
- 直接确认目标。
- 创建导航候选。
- 在 trace 中重新计算一套与 reasoner 不同的排名。

### 3.2 复杂任务计划

如果已有 `SearchTaskContext.subtasks`：

- 保留原顺序和原始文本。
- 展示每个子任务依赖。
- 当前搜索目标对应的子任务标为 active。
- 已有成功/失败事件才能改变完成状态。
- 当前执行器不支持的动作明确标成 unsupported。

如果没有 subtasks，则使用 §4.6 的五阶段默认计划。不能根据最终结果倒填一份看似聪明的计划。

### 3.3 知识检索

复用 `app/knowledge/retrieval.py` 和 `data/scene_kb/` 中的本地内容：

- 目标直接先验。
- 在任务明确给出房间类型时使用房间先验。
- 在任务明确给出楼层/范围时使用布局知识。

不能仅凭场景里出现“桌子”就断言当前房间是办公室；如果要做房间分类，属于另一个需要校准的任务，本次不实现。

如果现有 `KnowledgeItem.metadata` 没有把相关对象、来源文件等必要内容传出，可做最小向后兼容扩展并补测试。

## 阶段 4：生产路径与真实录制评估路径统一

### 4.1 生产路径装配

在 `scripts/go2w/run_semantic_exploration.py` 创建 `SearchReasoningContext` 的位置传入：

- 当前任务上下文。
- 本地知识检索结果。
- 当前只读拓扑摘要。
- yaw 和 observed sectors。
- 明确的 `run_mode`。

不要改该脚本中的运动执行逻辑。离线测试只能调用其纯构建函数或 mock 路径，不能启动真机入口。

### 4.2 评估器装配

`scripts/evaluate_live_search_reasoners.py` 必须走同一个上下文构造函数或同一组纯辅助函数，避免评估器比生产路径掌握更多“作弊信息”。

评估器输出每个会话的：

- 目标和 session ID。
- yaw 与 observed sectors 是否存在。
- graph match 状态。
- 知识检索数量及被采用数量。
- 候选方向排序摘要。
- 选中 directive。
- reason code。
- 是否被冻结消费者接受。
- 安全不变量。

不能把录制结束以后才出现的信息提供给早先决策，避免未来信息泄漏。

### 4.3 生产消费者和评估适配器差异

交接文档指出，离线评估器可能通过 `search_directive_adapter`，而实际生产还经过冻结的 `candidate_goal_generator`。所以必须同时验证：

1. reasoner A/B 输出改变且可解释。
2. 新 directive 通过冻结消费者契约测试。

只完成第一项不能算交付。

## 阶段 5：WebUI 任务 Agent 面板

### 5.1 页面布局

在现有“下一步决策”区域附近增加面板，建议顺序：

```text
[运行模式与安全状态]
[总体计划 / 当前子任务]
[本轮选择和一句话依据]
[候选方向比较]
[知识库与场景证据]
[记忆、覆盖与拓扑摘要]
[不确定性 / 缺失输入]
[下游导航候选和实际决定（现有区域）]
[历史记录]
```

不要让 Agent 面板替换现有候选、地图或安全状态；它是解释层。

### 5.2 展示规则

- 选择方向使用文字加角度，例如“左前方（+30°，按当前坐标约定）”。
- 明确显示角度正负约定，避免前端自行猜左右。
- 知识条目显示来源、置信度和 `used/context_only`。
- 候选行显示真实排序组件，不显示虚构的综合成功率。
- 缺失数据使用“未提供/无法判断”，不能显示成绿色 0。
- offline replay 使用固定醒目标签。
- 计划状态使用文本和图标共同表达，不能只靠颜色。
- 所有来自任务、检测标签和知识库的文本用安全文本节点渲染。

### 5.3 下游关联

WebUI 同时展示：

- `Agent 建议：探索 +30°`。
- `导航层：生成了哪些候选`。
- `最终决定：local_scan / frontier / semantic`。

若二者不同，显示类似：

```text
Agent 建议未成为最终决定：冻结候选生成器未产生可用语义目标；最终由 local_scan 获胜。
```

只有真实下游事件提供原因时才展示具体原因；没有原因就显示“下游未提供拒绝原因”，不得猜测。

### 5.4 历史

每轮历史至少保留：

- trace ID。
- 时间/扫描序号。
- 当前子任务。
- 选中偏好。
- reason code。
- 关键 evidence refs。
- 后续 decision ID 和结果。

限制历史大小时沿用现有 store 的策略，不新增未经验证的大型内存缓冲。

---

## 8. 离线 A/B 与验收方法

## 8.1 三套运行

至少运行：

1. `legacy`。
2. 修改前 `semantic_navigation` 基线。
3. 修改后 `semantic_navigation`，以及仓库已有的 `hybrid` 对照。

所有运行使用同一录制集合、同一过滤规则、同一 yaw 和 observed sectors 输入。结果写到不同 `/tmp` 子目录。

## 8.2 必须报告的计数

- 总 session 数、可用数、拒绝数和拒绝原因。
- 每种 directive kind 数量。
- `PARTIAL` 总数。
- `PARTIAL + anchor=None` 数量。
- `inspect_anchor + anchor=None` 数量，修改后必须为 0。
- 各方向扇区的选择数量。
- 使用场景/知识证据的决定数。
- 仅展示、未用于选择的知识条目数。
- 冻结消费者接受/拒绝的 directive 数量。
- `allow_forward=true` 和危险前进数量，离线必须为 0。
- 缺 yaw、缺 observed sectors、缺 SceneGraph 的会话数量。

## 8.3 不能只看“方向变多”

“出现了三个方向”只能证明分布变化，不能证明变聪明。需要做成对证据检查：

- 对有明确左/右实体方位的录制，报告选择是否与该证据一致。
- 对改变唯一证据的合成/回放 fixture，选择必须随证据改变。
- 对证据缺失的录制，系统必须诚实回到覆盖/负记忆排序，而不是编造语义理由。
- 对同一输入重复两次，除允许忽略的随机 ID 外，候选排序和 reason code 必须一致。

报告分子、分母和会话 ID，不在没有校准前添加“必须高于 80%”一类门槛。

## 8.4 建议的通过条件

以下条件不引入新经验阈值，可以作为硬性 DoD：

- 全部相关基线和新增测试通过。
- 每条新增判据完成注错校准。
- baseline 对 baseline 的语义差异为 0。
- 真实录制评估未丢失 yaw/observed sectors 注入。
- `inspect_anchor(anchor=None)=0`。
- 新产生的语义 directive 通过冻结消费者契约。
- 受控左右镜像证据导致相应方向镜像变化。
- 移除证据后，该证据不再出现在 used 列表。
- 缺失知识/拓扑不被伪装成负证据。
- 离线 `allow_forward=true=0`。
- WebUI 快照和增量事件都能显示 Agent trace。
- 旧事件/旧快照无 trace 时 UI 不崩溃。
- 冻结文件 SHA-256 与开工清单完全一致。
- 没有意外把 `.pyc`、测试输出或已有运行时文件加入改动。

## 8.5 与既有 §22 指标的关系

交接材料中提到至少三个方向扇区和 local scan 交替/耗尽等现场指标。本次可以在离线报告中提供对应分布，但不能宣称已完成真机 §22 验收，也不能为了满足扇区数而硬编码轮转。

---

## 9. 注错校准执行模板

每个新测试或新判据在交付报告中填一行：

| 判据 | 绿色命令 | 定向注错 | 红色原因 | 恢复哈希 | 恢复后结果 |
|---|---|---|---|---|---|
| PARTIAL 无锚点回落 | `pytest ...::test_...` | 恢复无条件 inspect | directive kind 断言失败 | SHA 一致 | PASS |
| 证据改变方向 | `pytest ...::test_...` | 忽略 evidence rank | 镜像方向断言失败 | SHA 一致 | PASS |
| trace 进入 store | `pytest ...::test_...` | 跳过 agent 更新 | snapshot 缺字段 | SHA 一致 | PASS |
| WebUI 渲染 | JS/pytest 指定测试 | 去掉 render 调用 | DOM 内容缺失 | SHA 一致 | PASS |

安全流程：

1. 注错前确认目标文件是否已有用户修改。
2. 复制到唯一的 `/tmp` 备份并记录 SHA-256。
3. 用最小补丁注入错误。
4. 用新的 `PYTHONPYCACHEPREFIX` 运行指定测试。
5. 确认失败来自预期断言，且确实收集到目标测试。
6. 从 `/tmp` 备份恢复；只有明确确认文件开工时干净，才可以按交接书使用 `git show HEAD:<path>` 恢复。
7. 验证恢复后 SHA-256 与注错前一致。
8. 再使用新的 `PYTHONPYCACHEPREFIX` 运行绿测。

不能遗留注错代码，不能用 Git 破坏性命令恢复。

---

## 10. 最终测试矩阵

执行 AI 应根据实际测试文件名填充精确 node ID，并在报告中保留命令和退出码。

### 10.1 推理单元测试

```bash
PYTHONPYCACHEPREFIX="$(mktemp -d)" "$PY" -m pytest \
  tests/test_semantic_navigation_auxiliary_hints.py \
  tests/test_semantic_navigation_graph_matcher.py \
  <新增或扩展的 task-agent/reasoner 测试> -q
```

### 10.2 状态与 WebSocket

```bash
PYTHONPYCACHEPREFIX="$(mktemp -d)" "$PY" -m pytest \
  tests/test_search_state_store_spatial.py \
  tests/test_autonomous_search_websocket.py \
  <新增 WebUI 合同测试> -q
```

### 10.3 离线评估器

```bash
PYTHONPYCACHEPREFIX="$(mktemp -d)" "$PY" -m pytest \
  tests/test_evaluate_live_search_reasoners.py -q
```

随后按原任务书提供的完整 57 会话执行方式运行 `scripts/evaluate_live_search_reasoners.py`。不要擅自发明另一套录制发现逻辑。

### 10.4 交接筛选回归

```bash
PYTHONPYCACHEPREFIX="$(mktemp -d)" "$PY" -m pytest tests/ -q \
  -k "reason or semantic or navigation or goal_graph or graph_match or exploration or shadow or replay or local_scan"
```

不要运行完整未筛选 `pytest tests/`，避免命中已知网络等待测试。

### 10.5 静态核查

至少核查：

- 改动文件全部属于计划范围。
- `app/navigation/` 无 diff。
- 其他冻结目标哈希不变。
- 没有写入真实机器人地址调用。
- 没有新增付费 API 客户端或网络请求。
- 没有意外加入 `.pyc`、`__pycache__`、A/B 大输出或秘密。

---

## 11. 停止条件

出现下列任一情况，执行 AI 必须暂停实现、保留证据并报告，不得用猜测绕过：

- 当前分支不符且无法无损切换。
- 基线用例失败，且失败与本任务无关。
- 57/45 录制分布发生无法解释的变化。
- 录制没有 yaw 或 observed sectors，导致方向比较失真。
- 需要修改 `app/navigation/` 才能实现核心结果。
- 需要修改任一冻结映射、定位或 ROS2 包。
- 新判据无法通过定向注错证明有分辨力。
- 新策略必须依赖未经真实数据校准的阈值。
- 发现需要连接机器狗或调用外部 API 才能继续。
- 冻结文件哈希发生变化。
- 工作区用户改动与目标文件发生无法安全合并的冲突。

如果只是个别录制缺少可选知识或拓扑摘要，不属于阻塞；应标记缺失并继续用其余已有证据，不能补造数据。

---

## 12. 最终交付物

执行完成后应交付：

1. 允许范围内的最小代码改动。
2. 全部新增或扩展测试。
3. WebUI 任务 Agent 面板。
4. 真实录制离线 A/B 结果，原始大文件放 `/tmp`，仓库内只保留精炼报告。
5. 一份建议命名为 `docs/GO2W_任务Agent与寻路决策链_离线改造交付报告.md` 的交付报告，包含：
   - 改动文件清单。
   - 未改动冻结项声明和 SHA-256 对比。
   - 行为变化前后对比。
   - 57 会话统计或语料漂移解释。
   - 每条新判据的注错校准记录。
   - 所有测试命令和结果。
   - WebUI 字段和截图或离线 DOM 证据。
   - 已知限制和真机待验项。
6. 最终 `git status --short`，清楚区分本任务文件与开工前用户文件。

默认不提交 Git。只有用户明确要求时，才显式添加本任务文件并提交。

---

## 13. Definition of Done

以下项目全部勾选，任务才算完成：

- [ ] 已全文阅读指定离线任务书和两个引用验收章节。
- [ ] 分支、HEAD、工作区和冻结哈希已记录。
- [ ] 未连接或探测机器狗。
- [ ] 未调用付费或联网 API。
- [ ] 未发送任何运动指令。
- [ ] 未修改 `app/navigation/` 或其他冻结内容。
- [ ] `PARTIAL` 且无锚点时不再生成 `inspect_anchor`。
- [ ] 无锚点分支不再硬编码 `+30°`。
- [ ] 探索方向由统一、可解释的候选排序器产生。
- [ ] 知识只有落到本轮可见实体时才参与方向排序。
- [ ] 缺失知识和拓扑明确标记为缺失，不当作负证据。
- [ ] 任务 Agent 能展示总体计划、当前步骤和能力边界。
- [ ] trace 包含输入摘要、候选、选择依据、证据引用和不确定性。
- [ ] WebUI 能在增量事件和重连 snapshot 两种情况下恢复 Agent 面板。
- [ ] WebUI 明确区分推理偏好、导航候选和实际决定。
- [ ] 冻结消费者合同测试通过。
- [ ] 真实录制 A/B 使用完整 yaw、observed sectors 和 SceneGraph。
- [ ] baseline/baseline 规范化对比为 0。
- [ ] 离线危险前进和 `allow_forward=true` 为 0。
- [ ] 每条新增验收判据经过“绿—注错红—恢复绿”校准。
- [ ] 每次 Python 运行使用新的 `PYTHONPYCACHEPREFIX`。
- [ ] 最终冻结哈希与开工一致。
- [ ] 没有覆盖或提交用户已有工作区内容。
- [ ] 交付报告明确写出离线无法证明的真机事项。

---

## 14. 给执行 AI 的最终原则

本任务不是让系统“说出更长的理由”，而是让系统在决定前真正读取更多现有信息，并把实际用过与没有用过的信息如实展示出来。

判断实现是否合格时，始终追问四个问题：

1. 这个方向为什么优于另一个方向，能否指向具体记录？
2. 删除或反转该证据后，决定是否按预期改变？
3. 这个建议是否被冻结导航层真正接受，还是只停留在推理日志里？
4. WebUI 展示的是系统实际使用的数据，还是事后生成的好听解释？

只有四个问题都有可复核答案，才可以宣布离线改造完成。
