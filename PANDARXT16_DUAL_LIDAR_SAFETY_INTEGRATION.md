# PandarXT-16 双雷达安全集成说明

> 2026-08-13 · 项目：`/home/brov/robot/robot_scene_demo`
> 原则：**SemanticNavigation 负责"去哪看"，双雷达安全层负责"能不能动"**。Pandar 永远不能
> 成为绕过安全链的第二个控制器；SemanticNavigation 也不能因 strong match 直接运动。

---

## 1. 职责分工

| 层 | 负责 | 禁止 |
|---|---|---|
| SemanticNavigation / Hybrid Reasoner | GoalGraph、SceneGraph 匹配、SearchDirective | 不确认目标；不带 `authorizes_motion` |
| search_directive_adapter | SearchDirective -> StepPlan（yaw clamp <= 30°） | 不 bypass 安全门 |
| Existing Physical Safety Gate | motion bounds / odom radius / mode / error | 不直接发布运动 |
| 双雷达安全证据层 | builtin L2 + Pandar 的 provenance 证据、旋转可观测性 | 不简单 concat 后宣称 clear |
| rotation lease / arm | pose-bound 短时许可 | 不允许失效 lease 通过 |
| go2w_motion_control | 执行 / STOP | 不被 LLM 直接触发 |

## 2. 证据模型（`ros2_ws/src/go2w_lidar_preprocessor/go2w_lidar_preprocessor/lidar_evidence.py`）

```text
EvidenceState: CLEAR / OCCUPIED / UNKNOWN / SELF_OCCLUDED / SENSOR_BLIND /
               STALE / UNVALIDATED_GEOMETRY

硬规则：
  UNKNOWN != CLEAR
  NO_RETURN != CLEAR
  UNVALIDATED_GEOMETRY != CLEAR

融合（fuse_dual_lidar_evidence）：
  有效传感器任一 OCCUPIED                       -> final=OCCUPIED
  至少一个 validated+fresh 传感器完整覆盖目标扫掠区且 CLEAR，且无有效 occupied
                                                -> final=CLEAR
  其它                                        -> final=UNKNOWN
```

`state_for_sensor` 保证 `range <= 0.05 m` 的零/近零回波被标记为 INVALID，不进入
obstacle / free space / clearance / occupancy / rotation evidence / metric
SceneGraph。

## 3. 时钟分级（`app/live_robot/pandar_clock.py`）

```text
UNVALIDATED
HOST_RECEIVE_TIME_ONLY            # 当前默认（PTP Free Run，use_timestamp_type=1）
HOST_CLOCK_MODEL_VALIDATED        # 可作 metric 融合
PTP_VALIDATED                     # 可作 metric 融合
```

正式 metric fusion 前必须显式检查 tier（`require_tier_for_metric`）。

## 4. 旋转可观测性（`dual_lidar_observability.py`）

```text
每个方位：扫掠带 [footprint_radius, envelope_radius]
  builtin_observable  = L2 无盲区覆盖整条带
  pandar_observable   = Pandar 无盲区覆盖整条带
  sector_observable   = builtin_observable OR (pandar_validated AND pandar_observable)

full_rotation_observability_valid       # 全部方位可观测
requested_turn_observability_valid      # 请求转向范围可观测
```

Pandar 仅在 `extrinsics_validated=true` 时贡献正式 CLEAR；否则只作诊断。

## 5. 当前硬件几何（`configs/go2w/current_hardware_geometry.yaml`）

```text
length=0.70 m, width=0.43 m, height=0.70 m
最高点 = PandarXT-16 固定保护框架（不是松弛电缆）
无需重新测长宽高
authorizes_motion=false
```

`current_hardware_state.yaml` 把几何/外参/内置雷达配置绑定在一起；任一
mount/geometry/extrinsic/clock 变化都会使旧 pose-bound lease 因 hash 失效。

## 6. 安全门顺序（run_autonomous_loop 执行前）

```text
1. rotation_lease_step（turn <= 30°）
2. evaluate_step_boundary（radius / fixed half-plane）
3. resolve_rotation_clearance_source -> evaluate_rotation_clearance
4. evaluate_dual_lidar_rotation_gate（dual-lidar enabled 时）
5. _safety_ok（mode/error + lidar fresh + front clearance）
6. arm -> /go2w/motion -> triple STOP + disarm
```

## 7. 只读验收顺序

```bash
bash scripts/go2w/start_hesai_pandarxt16.sh --with-preprocessor   # Pandar driver + 诊断预处理
/usr/bin/python3 scripts/go2w/validate_pandarxt16_clock.py --samples 30 --output <path>
/usr/bin/python3 scripts/go2w/validate_dual_lidar_observability_ros.py --output <path>
```

多场景外参：

```bash
/usr/bin/python3 scripts/go2w/capture_dual_lidar_calibration_ros.py --scene corner_01 --frames 8
/usr/bin/python3 scripts/go2w/capture_dual_lidar_calibration_ros.py --scene doorframe_02 --frames 8
/usr/bin/python3 scripts/go2w/capture_dual_lidar_calibration_ros.py --scene pillar_03 --frames 8
/usr/bin/python3 scripts/go2w/calibrate_pandarxt16_extrinsics.py --capture-dir outputs/go2w_acceptance/dual_lidar_calibration
/usr/bin/python3 scripts/go2w/validate_pandarxt16_extrinsics.py --output <path>
```

## 8. Stage 2 / 3

`app/live_robot/stage2_readiness.py`：

```text
stage2_ready = semantic_v1_ready AND pandar_raw_ready AND
               pandar_preprocess_ready AND pandar_extrinsics_validated AND
               current_hardware_geometry_loaded AND
               dual_lidar_rotation_observability_valid AND
               current_hardware_four_direction_evidence_valid AND
               pose_bound_rotation_lease_valid AND odom_fresh AND mode_ok AND
               motion_action_available AND no_stage2_error

stage3_ready = stage2_ready AND stage2_active_turn_only_pass AND
               semantic_forward_enabled AND front_clearance_valid AND
               short_forward_scope_valid
```

Stage 3 严格依赖 Stage 2 真实 PASS；缺任一 check 即 fail-closed。
