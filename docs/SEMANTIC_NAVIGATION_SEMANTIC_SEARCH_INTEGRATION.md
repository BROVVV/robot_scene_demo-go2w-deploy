# SemanticNavigation-style 语义搜索融合

本项目没有把 SemanticNavigation 的 Habitat、RGB-D mapper、FMM、Grounded-SAM 或
`real_world_env` 整仓接入。当前实现只按其公开的 Goal Graph、在线 Scene Graph
matching、分阶段 search policy 和 negative memory 思想，基于本项目现有接口重新
实现了一个可回滚的“下一视角”推理层。

## 边界

- 输入复用 `TargetProfile`、现有完整场景分析和 `ObservedSceneGraphBuilder`。
- 输出只有 `SearchDirective`：选哪个方向看、是否保持并重观测，以及解释和证据。
- V1 scene node 只记录 observation pose、heading sector、bbox 和证据引用；不会输出
  target 3D position、map goal 或任何伪造的 metric pose。
- `strong_match` 仍不是目标确认。目标候选继续走 quick detect → approach → verify →
  evidence gate → `target_reached`。
- Reasoner 不发布 `/cmd_vel`，不调用 Action；所有实际短步仍经过现有状态机、
  `step_planner`、clearance、odom/radius、mode/error、motion Action、STOP/disarm。
- `configs/go2w/navigation_gate.yaml` 没有因本功能解锁。

## 数据流

```text
TargetProfile → GoalGraph
stable REOBSERVE event → existing full scene analysis
                     → ObservedSceneGraphBuilder
GoalGraph + observed SceneGraph + session negative memory
                     → explainable graph match
                     → legacy / semantic_navigation / hybrid reasoner
                     → SearchDirective
                     → directive_to_step_plan
                     → existing safety and short-step chain
```

Semantic observer 是事件驱动和带 TTL 缓存的；不会按 15–28 Hz 相机帧调用 LLM。
它只在初始稳定观察或运动后的 REOBSERVE 分支被 Runner 请求。

## 后端与运行模式

- `legacy`：严格保留原 `plan_scan_step()`。
- `semantic_navigation`：按 zero / partial / strong graph match 选择未观察方向或 anchor。
- `hybrid`：优先真实观察图和负证据，不能形成可靠 directive 时回退 legacy。
- `shadow`：记录 semantic directive，但实际执行一定使用 legacy step。
- `active`：只有 directive 置信度达标且能安全转换时才使用 semantic step；否则自动
  回退 legacy。

仓库默认：

```text
LIVE_SEARCH_SEMANTIC_REASONING_ENABLED=false
LIVE_SEARCH_REASONER_BACKEND=legacy
LIVE_SEARCH_REASONER_MODE=shadow
LIVE_SEARCH_REASONER_ALLOW_FORWARD=false
```

## 命令

真机前先做 observe-only：

```bash
python run_live_robot_demo.py \
  --target "饮水机旁边的蓝色垃圾桶" \
  --detector llm \
  --semantic-reasoning \
  --search-reasoner hybrid \
  --search-reasoner-mode shadow
```

正式 state-machine shadow（实际动作仍为 legacy）：

```bash
/usr/bin/python3 scripts/go2w/run_autonomous_loop.py \
  --mode state_machine_search \
  --target "饮水机旁边的蓝色垃圾桶" \
  --detector llm \
  --semantic-reasoning \
  --search-reasoner hybrid \
  --search-reasoner-mode shadow \
  --semantic-no-forward \
  --max-radius 1.5 \
  --front-half-plane-only \
  --turn-only \
  --max-motion-steps 1 \
  --min-rotation-clearance 0.511 \
  --odom-topic /go2w/odom/fused \
  --output outputs/live_sessions/semantic_navigation_shadow.jsonl
```

狭窄现场首轮必须同时使用上述四个附加门：初始前向半圆、最终执行器禁止所有 forward、
一次成功动作后停止、旋转 clearance 有效且左右两侧均满足 0.511 m 全机身转向包络。
当前 L2 近场与该包络重叠，预处理器发布
`/go2w/safety/rotation_clearance_valid=false`，左右数值为 `NaN`（unknown）；有效性缺失、
数值缺失或低于 0.511 m 都会在申请动作前 fail-closed。`turn-only` 不依赖 reasoner，
因此 legacy scan、目标 approach 和 semantic directive 都不能绕过它。

只有完成 shadow 复核后，才把 mode 改为 `active`。V1 active 默认仍需
`--semantic-no-forward`，只让 reasoner 选择单步转向和重观测。即使显式传
`--semantic-allow-forward`，forward 也仍受固定初始前向半平面、radius、clearance、
odom 和 Action 门禁约束。

回滚无需改代码：去掉 `--semantic-reasoning`，或设
`--search-reasoner legacy`。默认配置本身就是 legacy。

## 可审计输出

Runner 主 JSONL 会写：

- `reasoner_shadow_compare` / `reasoner_active_decision`
- semantic directive、legacy step、graph match 与证据引用
- 长期 observation memory 的读取数量/`memory_id` 与明确的只读状态
- PSG / precomputed situated-prior 的 enabled/available/used refs，以及 replan cache 状态
- `semantic_reasoner_error` 和 legacy fallback

observe-only pipeline 还会写 `goal_graph.json`、`semantic_navigation_match.json`、
`search_directive.json`。离线比较：

```bash
python scripts/evaluate_live_search_reasoners.py \
  --session outputs/live_sessions/<session_id> \
  --target "饮水机旁边的蓝色垃圾桶"
```

评测器完全离线，不调用 API、不执行运动，并生成：

- `reasoner_comparison.json/.md`：三个 backend 的 directive、适配后计划和差异；
- `shadow_comparison.json`：明确记录 semantic proposed step 与 shadow 实际 legacy step；
- `reasoner_metrics.json`：从 session artifact/Event JSONL 推导 detector、crop verify、
  candidate/target 时间、执行步数、转角和距离。无法由证据确定的 revisit 指标保持
  `null`，不会臆造；`success_proxy` 仅代表存在候选视觉证据，不等于 target confirmed。

现场保存 session 的离线 A/B 证据位于
`outputs/reasoner_evaluation/{phone_live_20260806,semantic_navigation_live_20260813_01,semantic_navigation_live_20260813_02,semantic_navigation_live_20260813_03}`。

带显式关系约束的目标确认还要求
`TARGET_CONFIRMATION_REQUIRE_RELATION_EVIDENCE`：crop/track 负责确认主体视觉身份，
GoalGraph 与 Observed SceneGraph 必须同时给出 strong relation match。Reasoner 的建议
本身不能充当该关系证据。

## 仍未验证

- 真机 shadow 的成功运动 A/B（已通过真实传感器的决策等价与 fail-closed 验收：
  semantic/legacy 均为 `r30`，旋转门在 arm 前拒绝，0 步、odom 零变化）；
- active turn-only；
- active short-forward；
- RGB–LiDAR metric graph（等待外参/camera TF gate）；
- map goal 与 Nav2（等待 Level D/Nav2 gate）。

上述真机 shadow 安全验收证据位于
`outputs/go2w_acceptance/semantic_navigation_shadow_fail_closed_20260813/result.json`。状态机模式现在
延迟到某一步通过边界、旋转有效性与 LiDAR freshness 后才 arm；被门禁拒绝的 shadow
不会访问 arm/急停服务。quick 检测若明确返回目标不存在，可复用其场景摘要形成不含
伪造物体/关系的 zero-match 观察，避免对同一稳定帧重复调用 full-scene API。

本实现没有复制 SemanticNavigation 源文件，因此不新增 vendored 第三方代码；算法来源说明保留
在本文件和改动计划中。未来若复制上游 MIT 源码，必须补充第三方 notice。

## 长期与短期记忆边界

`LIVE_SEARCH_REASONER_USE_OBSERVATION_MEMORY=true` 时，observe-only 和真机状态机都会
通过现有 `ObservationMemoryStore` 读取与目标相关的长期 observation，并传入
`SearchReasoningContext.observation_memory`。已有记录若没有可验证的观测 yaw/heading，
不能据其 bbox 猜测真机转向；V1 只把它作为带 provenance 的上下文与审计引用。

本次 session 的失败方向由 `SemanticSearchMemory` 管理并按 TTL 过期。它不调用
`ObservationMemoryStore.append()`；只有其它既有流程中满足视觉 provenance 规则的正向
observation 才允许写长期库。ROS runner 的事件与 reasoner artifact 均显式记录
`persistent_write_attempted=false`。

## PSG、situated prior 与 replan

Hybrid 的严格优先级为：Observed GoalGraph match → visited/negative memory → PSG 或
precomputed situated prior → legacy 未观察方向。`VideoPSGPredictor` 只能从当前
Observed Scene Graph 的 node 生成提示；未被真实 node 支撑的预测会被丢弃。Situated
prior 不在真机 step loop 中另发网络请求，只消费上游已经提供且
`can_confirm_target=false` 的安全 payload。两种辅助提示均只能表达单步转向，不能提出
forward、不能覆盖负证据、更不能作为 target confirmation。

`LIVE_SEARCH_REASONER_MIN_REPLAN_SECONDS` 只节流完全相同的稳定观察。缓存 key 包含
frame、heading sector、机器人 x/y/yaw 与 recoverable-error；observer 还会在平移超过
0.05 m 时强制刷新。因此该配置不会在运动后复用旧场景。每次使用都会在 artifact 中
记录 `replan.throttled` 和复用的 directive id。
