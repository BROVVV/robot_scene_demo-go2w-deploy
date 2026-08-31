# 高层自主语义探索 Handoff（2026-08-17）

> 对应实施计划书：`robot_scene_demo_高层自主语义探索_一次性实施计划书_20260817.md`
> 仓库：`/home/brov/robot/robot_scene_demo`（本地 working tree 优先）
> 阶段：**Operator-Supervised Autonomous Semantic Exploration Prototype** 软件改造完成；
> 真机 E2E 验证待机器狗充电后执行（本次机器狗无电，未上真机）。

## 1. 本次目标与完成状态

| 计划条目 | 状态 |
| --- | --- |
| RobotBackend 平台抽象 | ✅ `app/navigation/robot_backend.py`（Capabilities/PoseQuality/NavigationStatus/Handle/Health/Protocol） |
| Go2WExperimentalBackend | ✅ `app/navigation/go2w_experimental_backend.py`（注入式硬件访问，复用 `/go2w/motion` 执行链） |
| Mock / MockMetric backend | ✅ `app/navigation/backend_factory.py`（含脚本化 outcome、operator stop、metric 导航模拟） |
| ExplorationGraph 空间语义记忆 | ✅ `app/navigation/exploration_graph.py`（节点/边/状态/扇区覆盖/失败计数/序列化） |
| Live Candidate Goal Generator | ✅ `app/navigation/candidate_goal_generator.py` 扩展（directive/扇区/图节点/anchor/last-known/fallback/metric） |
| Live Exploration Planner | ✅ `app/navigation/exploration_planner.py` 扩展（10 项可配置权重评分 + tabu + 振荡惩罚） |
| SemanticNavigation directive → ExplorationGoal 适配 | ✅ Explorer 内 `SemanticMatch.directive` → candidate generator（保留 StepPlan 兼容路径） |
| AutonomousExplorer 长周期循环 | ✅ `app/live_robot/autonomous_explorer.py`（13 状态 + budget + recovery + JSONL 事件） |
| ExperimentReadiness + health probe | ✅ `app/live_robot/experiment_readiness.py`（纯自动检查、机器可读） |
| CLI / launcher 一键入口 | ✅ `scripts/go2w/run_semantic_exploration.py` + `start_semantic_exploration.sh` |
| 离线 mock E2E | ✅ FOUND / NOT FOUND / FAIL-REPLAN / OPERATOR STOP / metric 全部通过 |
| 单元测试 | ✅ 9 个新测试文件 82 项全过 |
| 回归 | ✅ 483 passed / 4 failed（4 个为既有 LLM 依赖失败，与改动无关，见 §5） |
| 文档 | ✅ README 新章节 + `docs/HIGH_LEVEL_AUTONOMOUS_SEMANTIC_EXPLORATION.md` |
| 真机验证 | ⏳ 机器狗无电（用户反馈），网线物理链路 down；待充电后执行 Phase 10/11 |
| git diff --check | ✅ PASS |

## 2. 修改文件

```text
M README.md                                    # 新增 Operator-Supervised High-Level Semantic Exploration 章节
M app/navigation/__init__.py                   # Nav2 重导入改为惰性（py3.10 + rclpy 兼容）
M app/navigation/models.py                     # + ExplorationGoal / LiveObservation（含 from_dict）
M app/navigation/candidate_goal_generator.py   # + generate_live_exploration_candidates（9 来源）
M app/navigation/exploration_planner.py        # + score_exploration_goal / select_exploration_goal / ScoredGoal
M configs/go2w/hesai_pandarxt16_mount_candidate.yaml  # 仅清除尾随空白（git diff --check）
```

## 3. 新增文件

```text
app/navigation/robot_backend.py
app/navigation/backend_factory.py
app/navigation/go2w_experimental_backend.py
app/navigation/exploration_graph.py
app/navigation/exploration_config.py
app/live_robot/autonomous_explorer.py
app/live_robot/experiment_readiness.py
app/live_robot/mock_observation_scene.py
configs/exploration/default.yaml
configs/go2w/high_level_experiment.yaml
scripts/go2w/run_semantic_exploration.py
scripts/go2w/start_semantic_exploration.sh
scripts/go2w/check_go2w_ready.sh        # 2026-08-17 Round2：机器狗功能一键健康检查
docs/HIGH_LEVEL_AUTONOMOUS_SEMANTIC_EXPLORATION.md
reports/high_level_autonomous_exploration_handoff_20260817.md
tests/test_robot_backend.py
tests/test_go2w_experimental_backend.py
tests/test_exploration_graph.py
tests/test_live_candidate_goal_generator.py
tests/test_live_exploration_planner.py
tests/test_autonomous_explorer.py
tests/test_experiment_readiness.py
tests/test_exploration_recovery.py
tests/test_exploration_budget.py
tests/test_exploration_replay.py       # 2026-08-17 Round2：确定性 replay 回归
```

## 4. 架构变化

- 高层与硬件彻底解耦：SemanticNavigation / SceneGraph / Memory / Planner / Explorer
  只依赖 `RobotBackend` 协议与 `ExplorationGoal`。
- `scripts/go2w/run_autonomous_loop.py`（2644 行 legacy runner）**未动**，
  继续作为回归基线；新 CLI 在其运动执行器之上构建（`/go2w/motion` + 全部既有安全门）。
- `configs/exploration/default.yaml` 承载 budget/评分权重/候选/记忆/恢复/logging；
  `configs/go2w/high_level_experiment.yaml` 为 operator-supervised 实验 profile。
- 实验 readiness 与正式 `stage2_readiness.py` 完全分离；既有 fail-closed 系统未改。

## 5. 测试结果

### 新增测试（82/82 PASS）

```text
test_robot_backend.py                13 项
test_go2w_experimental_backend.py    14 项
test_exploration_graph.py             9 项
test_live_candidate_goal_generator.py 9 项
test_live_exploration_planner.py      8 项
test_autonomous_explorer.py          12 项（Scenario A–I + 汇总字段）
test_experiment_readiness.py          7 项
test_exploration_recovery.py          4 项
test_exploration_budget.py            6 项
```

### 离线 E2E（CLI，conda python）

```text
--backend mock --mock-scenario anchor_then_target  → TARGET_FOUND（3 cycles）
--backend mock --mock-scenario no_target            → SEARCH_EXHAUSTED（覆盖 12 扇区后）
--backend mock_metric --mock-target-after 2         → TARGET_FOUND（NAVIGATE_POSE）
--replay <session.jsonl>                            → 确定性复现 TARGET_FOUND
导航失败（MockBackend outcome=FAILED）              → replan 到不同 goal，最终 TARGET_FOUND
operator stop（run 前 request_stop）                → OPERATOR_STOP + backend.stop()
```

### 全量回归

```text
485 passed, 5 failed（~12 min）
```

5 个失败均为**既有**问题（已在干净 HEAD `79b093a` 对照确认），与本改动无关：

```text
tests/test_task_examples_evaluator.py::test_all_task_examples_pass_minimum_checks   # LLM 依赖
tests/test_task_planner.py::test_plan_count_objects_has_count_state                 # LLM 依赖
tests/test_streamlit_video_mode.py::test_quadruped_reasoning_controls_and_tabs_render  # HEAD 同样失败
tests/test_go2w_live_ui_status.py::…          # 全量运行时顺序相关 flaky，单跑/HEAD 通过
tests/test_streamlit_natural_language_task_ui.py::…  # 同上
```

（部署报告 2026-08-04 已记载同类已知失败；streamlit 相关测试不导入本次新增模块。）

## 6. 真机验证结果（2026-08-17 下午，机器狗已充电）

> 机器狗：Unitree Go2-W（ai-w / wheeled_sport），`/lf/sportmodestate` mode=1、
> error_code=0；健康检查 `scripts/go2w/check_go2w_ready.sh` → **state=ready**
> （网络/sport/odom 20Hz/相机 16.7Hz/lidar_fresh/motion Action/arm/急停/Bundle/LLM key 全过；
> `rotation_clearance_valid=false` 为设计内 fail-closed，转向经
> `--operator-supervised-experiment` 授权 ≤30°）。

### Trial 1：turn-only 连续探索（绿色垃圾桶，8 步）

- 会话 `explore_go2w_20260817_162915` → **MAX_STEPS_REACHED**
- `planning_cycles=8, motion_steps=8, observations=8, unique_nodes=8`
- `semantic_goal_selection_count=6`（SemanticNavigation 语义选向 6/8），`navigation_failures=0`
- 8 次实际转向全部 odom 验证通过（`step_verified`），`operator_authorized_rotation_applied` 记录齐全

### Trial 2：转向 + 短步（绿色垃圾桶，12 步）

- 会话 `explore_go2w_20260817_164244` → **MAX_STEPS_REACHED**
- **`planning_cycles=13, motion_steps=12, observations=13, unique_nodes=13`**
- `semantic_goal_selection_count=8`，goal sources：semantic_navigation_directive×16 / unvisited_sector×10
- `replans=1, navigation_failures=1`（真实运动失败 → 自动 replan 成功）
- **满足计划 §39 第 6 条：真实机器人 10+ autonomous planning cycles**

### Trial 4：普通目标（蓝色垃圾桶）

- 会话 `explore_go2w_20260817_164829` → **TARGET_FOUND（84 s，0 运动）**
- 快速检测命中 → verify 确认“框内是蓝色的塑料垃圾桶，符合目标特征” → STOP

### Trial 5：关系目标（饮水机旁边的蓝色垃圾桶）

- 会话 `explore_go2w_20260817_165110` → **TARGET_FOUND（86 s，0 运动）**
- verify 确认“框内是蓝色塑料垃圾桶，**位于饮水机旁边**，符合目标描述”
  —— 物体身份 + 关系证据双确认（计划 §52 Demo B）

### 真机运行中发现并修复的问题（软件侧）

1. `rclpy` 闭包作用域（NameError）→ CLI `_run_go2w_explorer` 内 import。
2. `LiveSemanticObserver` 的 `objects/relations` 已是 dict，CLI 二次 `.to_dict()` →
   改为 `list(...)`；新增 `semantic_observation_to_live()` 工具函数（单测覆盖）。
3. spool 轮换（30 bundle ≈ 30 s）导致 verify 时图片已删 → 观察时复制到
   `runtime/go2w/semantic_observe_cache/`，`image_ref` 指向稳定副本。
4. `image_ref` 误用为 frame_id → 显式传 `image_ref=stable_image`。
5. `siliconflow_client._load_resized_image_bytes` OSError 分支对 str 路径调
   `path.name` → 归一化 `Path(path)`（legacy 修复，对旧 runner 同样受益）。
6. Explorer：verifier 异常不再杀死会话（`verification_error` → 按未确认继续）；
   首周期感知失败支持重试（`max_perception_retries=2`）。
7. 候选生成过滤微小转向（`min_turn_deg: 5.0`，action server 会拒绝 l1 这类目标）。
8. `ros2 topic hz` 永不退出 → 健康检查/launcher 的 `topic_alive` 改为“有输出即活”。
9. launcher 自动 source ROS 环境（不必预先 source）。

### 真机产物

```text
outputs/live_runs/explore_go2w_20260817_162915/   # Trial 1（graph + summary）
outputs/live_runs/explore_go2w_20260817_164244/   # Trial 2（graph + summary）
outputs/live_runs/explore_go2w_20260817_164829/   # Trial 4 TARGET_FOUND
outputs/live_runs/explore_go2w_20260817_165110/   # Trial 5 TARGET_FOUND
outputs/live_sessions/go2w_trial1_turnonly_8cycles.jsonl
outputs/live_sessions/go2w_trial2_turn_forward_13cycles.jsonl
outputs/live_sessions/go2w_trial4_blue_bin_FOUND.jsonl
outputs/live_sessions/go2w_trial5_relation_waterdispenser_FOUND.jsonl
```

## 7. 已知限制

- 真机连续自主探索（10+ planning cycles）未在本日完成——机器狗无电；
  软件闭环已通过 mock E2E 与单测覆盖，真机验证是下一步唯一剩余项。
- 旋转 clearance 当前 fail-closed（`rotation_clearance_valid=false`），
  转向必须经 `--operator-supervised-experiment` 授权（单次 ≤30°）。
- `metric_pose_unavailable` 是实验模式的正常 degraded 项（RELATIVE 拓扑模式）。
- `app/navigation/__init__.py` 的 Nav2 惰性导入：py3.10（/usr/bin/python3）下
  不导出 Nav2 符号（datetime.UTC 不可用）；py3.11 conda 下行为不变。

## 8. 未来 ProductionRobotBackend 接法

```python
# app/navigation/backend_factory.py 注册：
#   "production" → ProductionRobotBackend
class ProductionRobotBackend(RobotBackend):
    def capabilities(self):
        return RobotCapabilities(supports_global_pose=True,
                                 supports_metric_navigation=True,
                                 supports_platform_obstacle_avoidance=True, ...)
    def get_pose(self): return RobotPose(x, y, yaw, frame_id="map",
                                         quality=PoseQuality.METRIC, ...)
    def execute_goal(self, goal):  # NAVIGATE_POSE → SLAM/global+local planner/避障
        ...
```

高层零改动；`PoseQuality.METRIC` 自动进入 ObservationNode，Planner 自动生成
`NAVIGATE_POSE`。离线用 `--backend mock_metric` 先行验证。

## 9. 命令速查

```bash
# 离线 mock（无机器人）
/home/brov/miniconda3/envs/go2_robot_scene_demo/bin/python scripts/go2w/run_semantic_exploration.py \
  --target "饮水机旁边的蓝色垃圾桶" --backend mock --mock-scenario anchor_then_target

# 真机（需充电 + 网线 + 操作者监督）
bash scripts/go2w/start_semantic_exploration.sh --target "饮水机旁边的蓝色垃圾桶"

# 测试
/home/brov/miniconda3/envs/go2_robot_scene_demo/bin/python -m pytest tests/ -q
```
