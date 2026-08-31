# SemanticNavigation V1 现有实现审计报告（2026-08-13）

> 项目：`/home/brov/robot/robot_scene_demo`
> 基线 HEAD：`e861b953ba195e1a47bcdb9491f955e69c5ae451`
> 审计性质：KEEP-only。只确认现有 SemanticNavigation V1 可直接接续，不重写、不"统一风格"大重构。
> 审计范围：`app/reasoning/semantic_navigation/*`、`app/live_robot/*` 与安全链、ROS `go2w_lidar_preprocessor` 包、脚本与配置。

---

## 1. 结论

SemanticNavigation V1 软件核心**完整且可原地接续**。Observe-only 与 Shadow 已通过；
Active turn-only / short-forward 被安全证据缺口阻塞，而不是被算法缺口阻塞。

| 模块 | 结论 |
|---|---|
| GoalGraphBuilder | **KEEP** |
| GraphMatcher | **KEEP** |
| SemanticMemory | **KEEP** |
| SearchReasoner | **KEEP** |
| Router | **KEEP** |
| LiveSemanticObserver | **KEEP** |
| SearchDirectiveAdapter | **KEEP** |
| Shadow/Active 模式 | **KEEP** |

真实阻塞项全部在安全证据侧：

- 内置 L2 对整机旋转扫掠带 720/720 方位不可观测；
- PandarXT-16 尚无正式外参、自遮挡、时钟与四向证据；
- 旧硬件（安装前）四向物理证据不可用于当前硬件运动许可。

## 2. SemanticNavigation 数据流（审计确认）

```
TargetProfile
  -> GoalGraphBuilder.build(profile)            # router.py:42
  -> SemanticNavigationGraphMatcher.match(...)             # router.py:26-32, graph_matcher.py:19
       ZERO / PARTIAL / STRONG（无 confirmed；strong != confirmed）
  -> SearchReasoner.propose(context)            # search_reasoner.py:35-92
  -> SearchDirective（models.py:97-117）
  -> directive_to_step_plan(...)                # search_directive_adapter.py:9
  -> StepPlan（step_planner.py:23-28，yaw clamp <= 30°）
  -> StepSearchRunner（shadow=legacy / active=全部安全门后执行）
```

`SearchReasoningContext.safety_context`（models.py:134）已存在但只作 artifact；
`SearchDirective` 本轮新增可选 `safety_intent` / `requires_rotation_clearance` /
`requires_front_clearance`（默认 observe / False / False），不破坏既有构造点。

## 3. 安全链现状（审计确认）

```
step_planner 产出候选 step
  -> rotation_lease.evaluate_rotation_lease_step（turn-only <= 30°）
  -> motion_bounds.evaluate_step_boundary（radius / fixed half-plane）
  -> resolve_rotation_clearance_source -> evaluate_rotation_clearance
  -> _safety_ok（sport mode + lidar fresh + front clearance）
  -> arm -> /go2w/motion Action -> triple STOP + disarm
```

- `motion_bounds.py` 纯 2D，无整机几何字段；`rotation_lease.py` 绑定
  `origin_pose(odom_wheel)`、时间窗、envelope 半径和 5 份 capture SHA-256，
  但**不绑定当前硬件状态/几何/外参/时钟**（本轮已补）。
- `rotation_crosscheck.py` 严格单传感器（baseline + 四向），本轮扩展
  `build_rotation_evidence(..., hardware_binding=...)` 可选绑定。
- `rotation_observability_report` 给出 720/720 方位不可观测的几何审计，
  本轮新增 `dual_lidar_observability` 把 Pandar 作为互补传感器纳入。

## 4. Pandar / 双雷达缺口（本轮代码已补齐）

- **zero-return**：`lidar_evidence.classify_zero_returns` / `state_for_sensor`，
  `range <= 0.05 m -> INVALID_RETURN`，不得进入 obstacle/free/clear/occupancy。
- **时钟分级**：`app/live_robot/pandar_clock.py`，默认
  `HOST_RECEIVE_TIME_ONLY`；metric 融合前必须显式检查 tier。
- **双雷达证据融合**：`ros2_ws/.../lidar_evidence.py`，
  `fuse_dual_lidar_evidence` 保持 provenance；`UNKNOWN != CLEAR`；
  任何有效 OCCUPIED 优先。
- **旋转可观测性**：`ros2_ws/.../dual_lidar_observability.py`，Pandar 仅在
  外参 validated 时贡献正式 CLEAR。
- **当前硬件几何**：`app/live_robot/current_hardware.py` + 
  `configs/go2w/current_hardware_{geometry,state}.yaml`，
  `0.70 x 0.43 x 0.70 m`，最高点为固定 Pandar 保护框架。
- **Stage 2 readiness**：`app/live_robot/stage2_readiness.py`，
  12 项 machine-readable 全真才 `stage2_ready`。

## 5. 禁止触碰项（保持）

- `configs/go2w/official_reference.yaml`（厂商原始 0.50 m 高度不改）
- `configs/go2w/navigation_gate.yaml`（fail-closed 未解锁）
- `unitree_go2w_control/*`、`ros2_ws/src/hesai_ros_driver/*`
- `app/reasoning/semantic_navigation/*` 既有逻辑（只允许修 bug / 补接口）
