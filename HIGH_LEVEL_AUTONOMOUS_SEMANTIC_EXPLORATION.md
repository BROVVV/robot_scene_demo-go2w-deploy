# 高层自主语义探索系统（High-Level Autonomous Semantic Exploration）

> 版本：2026-08-17 · 对应实施计划书 `robot_scene_demo_高层自主语义探索_一次性实施计划书_20260817.md`
> 平台：Unitree Go2-W（operator-supervised experimental backend），未来可迁移到成熟机器狗。

本文档描述系统架构、RobotBackend 平台抽象、Explorer 状态机、空间语义记忆、
Goal 评分、实验模式与未来机器人迁移方式。

---

## 1. 系统架构

```
自然语言目标
  → TargetProfile / GoalGraph（SemanticNavigation）
  → LiveObservation（LiveSemanticObserver + 快速检测）
  → GraphMatcher / SemanticMatch
  → 空间语义记忆（ExplorationGraph + SemanticSearchMemory）
  → CandidateGoalGenerator（SemanticNavigation directive / 未访问扇区 / 图节点 / anchor / last-known / fallback）
  → ExplorationPlanner（可配置权重评分）
  → AutonomousExplorer（最高层 orchestrator）
  → RobotBackend（平台抽象）
      ├── Go2WExperimentalBackend（当前：relative + topological）
      └── MockBackend / MockMetricBackend（离线测试 / 未来 metric 验证）
```

核心原则：

1. 高层（SemanticNavigation / SceneGraph / Memory / Planner / Explorer）**不允许直接依赖**
   Unitree、`/go2w/motion`、SportModeState、Pandar、旋转 lease、Go2-W footprint。
2. 所有硬件访问通过 `RobotBackend` 接口；当前 Go2-W 是其中一个实现。
3. strong graph match 不是最终确认；最终确认由视觉/关系 evidence + verify 完成。

## 2. RobotBackend 平台抽象

文件：`app/navigation/robot_backend.py`

| 类型 | 说明 |
| --- | --- |
| `RobotCapabilities` | 平台能力位（global pose / metric nav / relative translation / rotation / heading / cancel / feedback / obstacle avoidance） |
| `PoseQuality` | `UNAVAILABLE` / `RELATIVE` / `METRIC` —— 高层禁止伪造 metric pose |
| `NavigationStatus` | `ACCEPTED/RUNNING/SUCCEEDED/FAILED/CANCELLED/TIMEOUT/OPERATOR_STOP/BACKEND_UNAVAILABLE/REJECTED` |
| `NavigationHandle` / `NavigationResult` | 一次 goal 执行的结果（requested/observed motion、耗时、attempt、provenance） |
| `BackendHealth` | ready / degraded / capabilities / pose_quality，机器可读 |
| `RobotBackend(Protocol)` | `capabilities() / get_pose() / execute_goal() / get_navigation_status() / cancel() / stop() / health()` |

`ExplorationGoal`（`app/navigation/models.py`）是平台无关的高层动作语言：

```text
REOBSERVE | ROTATE_VIEW | RELATIVE_MOVE | NAVIGATE_POSE | INSPECT_ANCHOR | REVISIT_NODE | STOP
```

- relative 字段（`relative_dx/dy/dyaw`）只被 relative backend 消费；
- `position/yaw/frame` 只被 metric backend 消费；
- Planner 只会生成 backend 能力允许的 goal 类型。

### 2.1 Go2WExperimentalBackend

文件：`app/navigation/go2w_experimental_backend.py`

- 通过注入的 callable 执行运动（`execute_step`、`odometry`、`stop`、`cancel`、
  `health_probe`），保持 ROS 无关、可单测；真机接线在
  `scripts/go2w/run_semantic_exploration.py` 中复用 `run_autonomous_loop.py`
  的审计过的运动执行器（`/go2w/motion` Action + `/go2w/arm` + `/go2w/emergency_stop`）。
- 动作映射：`ROTATE_VIEW/INSPECT_ANCHOR/REVISIT_NODE → lN/rN（|N|≤30°）`；
  `RELATIVE_MOVE → f（≤0.30 m）`；`NAVIGATE_POSE → REJECTED`（能力不允许）。
- 机会式 request-vs-observed 学习（rotation_scale / forward_scale）：
  低样本或低置信度时不应用（`confidence=low`）。

### 2.2 Mock 与未来后端

- `app/navigation/backend_factory.py`：`create_backend("go2w_experimental" | "mock" | "mock_metric")`。
- `MockBackend`：relative 能力 + 可脚本化 outcome（成功/失败/超时/operator stop），
  模拟位姿更新，CI 不需要机器狗即可跑通 Explorer。
- `MockMetricBackend`：模拟成熟机器人（map pose + `NAVIGATE_POSE` 执行），
  证明同一 Explorer 不修改 GoalGraph/GraphMatcher/Memory/Planner 即可迁移。

## 3. AutonomousExplorer 状态机

文件：`app/live_robot/autonomous_explorer.py`

```text
BOOTSTRAP → OBSERVE → MATCH → VERIFY → UPDATE_MEMORY → PLAN → EXECUTE
  → WAIT_RESULT → RECOVER/REPLAN → OBSERVE → … → TARGET_FOUND
  | SEARCH_EXHAUSTED | OPERATOR_STOP | FAILED | FINISHED
```

主循环要点：

- `observer()` 注入：返回 `LiveObservation`（bundle_id、scene_objects/relations、
  scene_graph、target_match、pose、sensor_health）；异常按
  `PerceptionFailure` 恢复，首周期失败 → `PERCEPTION_FAILURE`。
- `matcher()` 注入：返回 `SemanticMatch`（has_candidate、graph_match、
  directive、anchor_labels、target_score…）；异常 → 空 match（fallback 继续）。
- `verifier()` 注入：返回 `VerificationOutcome`；`confirmed` →
  `graph.mark_target_confirmed` + `backend.stop()` + `TARGET_FOUND`。
- 记忆更新：`ExplorationGraph.add_observation`（objects/relations/scene_graph/
  heading_sector/信息增益），目标未见 → `mark_negative` + 负记忆扇区惩罚。
- 规划：`generate_live_exploration_candidates` → `select_exploration_goal`。
- 执行：`backend.execute_goal(goal)` → `_wait_result`（轮询 terminal status，
  支持操作者中断）；结果记录进图（`record_navigation`，节点/扇区失败计数）。
- Recovery：FAILED → replan（失败扇区 tabu）；TIMEOUT → cancel + 有限重试；
  OPERATOR_STOP → 立即 `backend.stop()`，不自动继续。

## 4. 空间语义记忆（ExplorationGraph）

文件：`app/navigation/exploration_graph.py`

- `ObservationNode`：node_id、pose（PoseQuality）、heading_sector、objects、
  relations、scene_graph、target_match_level、semantic_relevance、
  information_gain、visited_count、negative_evidence_count、
  navigation_fail_count、reachable_state、source_bundle_id、provenance。
- `ExplorationEdge`：source/target node、action_type、requested/observed motion、
  navigation_result、cost。
- 节点状态：`UNSEEN/OBSERVED/VISITED/SEMANTIC_INTEREST/NEGATIVE/UNREACHABLE/
  TARGET_CANDIDATE/TARGET_CONFIRMED`。
- 扇区覆盖集合独立于节点合并维护（重复观察同一 view 不会“抹掉”已覆盖扇区）。
- 失败计数：`sector_failure_count(goal_type, sector)`，供 Planner 惩罚与
  UNREACHABLE 判定。
- 序列化：`outputs/live_runs/<session_id>/exploration_graph.json`
  （含 observed_sectors、recent_goals，可加载回放）。

## 5. 候选目标生成（Candidate Goal Generator）

文件：`app/navigation/candidate_goal_generator.py`
（`generate_live_exploration_candidates`，原视频候选函数保留）

| 来源 | 输出 |
| --- | --- |
| A. SemanticNavigation semantic directive | `INSPECT_ANCHOR`（anchor 方向）/ `ROTATE_VIEW`（heading）/ `REOBSERVE` |
| B. 未访问 heading sector（12×30°） | `ROTATE_VIEW` |
| C. 图中 UNSEEN / SEMANTIC_INTEREST 节点 | `REVISIT_NODE` |
| D. 当前视图中的强语义 anchor | `INSPECT_ANCHOR`（semantic_relevance 高） |
| E. last-known / target-lost | `REVISIT_NODE`（target_candidate 节点） |
| F. fallback 探索 | `ROTATE_VIEW` + （能力允许时）`RELATIVE_MOVE` |
| G. metric backend | `NAVIGATE_POSE`（图节点位姿/引导位姿） |

## 6. 探索目标评分（Exploration Planner）

文件：`app/navigation/exploration_planner.py`（`score_exploration_goal` /
`select_exploration_goal`，原 frontier 函数保留）

```text
score =
    0.35*semantic_relevance + 0.25*novelty + 0.20*information_gain
  + 0.10*frontier_bonus + 0.10*continuity_bonus
  - 0.30*visited_penalty - 0.25*negative_evidence_penalty
  - 0.35*navigation_failure_penalty - 0.15*estimated_motion_cost
  - 0.20*oscillation_penalty
```

- 所有权重在 `configs/exploration/default.yaml` 的 `exploration.scoring` 中，
  是默认策略而非结论，可调。
- `select_exploration_goal` 支持 `exclude_node_ids/exclude_sectors`
  （recent-goal tabu + 失败扇区排除）。
- 防振荡：`ExplorationGraph.oscillation_penalty` 检测 A-B-A-B 两周期模式；
  Explorer 对最近 2 个已执行 goal 的扇区/节点做 tabu。

## 7. 实验模式（Operator-Supervised Experiment）

- Profile：`configs/go2w/high_level_experiment.yaml`
  （`operator_supervised_experiment`：`production_safe=false`、
  `operator_present=true`、无人工标定要求、Stage2/Pandar/Nav2 均不要求）。
- Readiness：`app/live_robot/experiment_readiness.py` —— 纯自动检查
  （camera/bundle/LLM/motion action/robot mode/emergency stop/pose freshness），
  输出机器可读 JSON；`metric_pose_unavailable` 只是 degraded，不阻塞。
- 既有安全系统（stage2_readiness、rotation lease、dual lidar safety、
  navigation gate、fail-closed policy）**全部保留**，实验 profile 不修改它们。

## 8. 一键运行

```bash
bash scripts/go2w/start_semantic_exploration.sh --target "饮水机旁边的蓝色垃圾桶"
# 或
/usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
  --target "..." --backend go2w_experimental --reasoner semantic_navigation \
  --operator-supervised-experiment --finish-on-visual-confirmation \
  --max-seconds 600 --max-motion-steps 50 \
  --output outputs/live_sessions/high_level_semantic_exploration.jsonl
```

离线：`--backend mock|mock_metric`（可加 `--mock-scenario`）。
回放：`--replay <session.jsonl>`（确定性重放 observation 事件）。

## 9. 未来机器人迁移

1. 实现 `ProductionRobotBackend`（map pose + metric navigation + 避障）。
2. 在 `backend_factory.create_backend` 注册。
3. 高层（SemanticNavigation / GraphMatcher / SemanticMemory / Planner / Explorer）零改动；
   Planner 自动生成 `NAVIGATE_POSE`，`PoseQuality.METRIC` 进入 ObservationNode。

验证方式：`--backend mock_metric` 离线 E2E + 新增单测
`tests/test_autonomous_explorer.py::test_scenario_i_metric_backend_navigate_pose`。

## 10. 测试

- 新增：`tests/test_robot_backend.py`、`tests/test_go2w_experimental_backend.py`、
  `tests/test_exploration_graph.py`、`tests/test_live_candidate_goal_generator.py`、
  `tests/test_live_exploration_planner.py`、`tests/test_autonomous_explorer.py`
  （Scenario A–I）、`tests/test_experiment_readiness.py`、
  `tests/test_exploration_recovery.py`、`tests/test_exploration_budget.py`。
- 运行：`/home/brov/miniconda3/envs/go2_robot_scene_demo/bin/python -m pytest tests/ -q`。
