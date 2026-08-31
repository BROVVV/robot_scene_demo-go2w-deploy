# robot_scene_demo × Go2-W × SemanticNavigation × PandarXT-16 Stage 2/3 交接书

> 交接日期：2026-08-14（Asia/Shanghai，机器狗重启后现场复核）
> 项目：`/home/brov/robot/robot_scene_demo`
> 分支：`main`；基线 HEAD：`e861b953ba195e1a47bcdb9491f955e69c5ae451`
> 工作树：本轮继续原地接续，未 reset/clean；本交接记录本轮 Phase 0–7 全部软件改动。
> 本轮计划书：`/home/brov/下载/robot_scene_demo_SemanticNavigation_PandarXT16_Stage2_Stage3_一次性改动计划书_20260813.md`

---

## 0. 一页结论

1. **SemanticNavigation V1 未重做**。Observe-only / Shadow 保持 PASS；Active turn-only /
   short-forward 仍被安全证据缺口阻塞（不是算法缺口）。
2. **当前整机几何**为 `0.70 × 0.43 × 0.70 m`，最高点为 **PandarXT-16 固定保护
   框架**（不是松弛电缆）；**本轮没有重新测整机长宽高**。
3. 新增 **Pandar 诊断预处理**（zero-return <=0.05 m 过滤、时钟分级、自遮挡、
   可观测性诊断）、**双雷达 provenance 证据融合**、**双雷达旋转可观测性**、
   **pose-bound rotation lease 硬件绑定**、**Stage 2/3 machine-readable
   readiness** 及全部测试/文档。
4. 全部默认仍 **fail-closed**：`rotation_clearance_valid=false`、
   `dual_lidar_safety.enabled=false`、semantic 默认 `disabled/legacy/shadow/
   no-forward`、`navigation_gate.yaml` 未解锁、motion steps=0。
5. 本轮软件 **CODE READY**；Pandar 正式外参 / 四向证据 / Stage2 真机运动为
   **PHYSICAL VALIDATION PENDING**（需真机多场景标定与现场证据）。
6. 机器狗重启后现场复核：网络/相机/内置雷达/里程计/运动模式全部正常；当前无
   motion Action、无 lease、无 arm；`/go2w/safety/rotation_clearance_valid=false`。

## 1. 本轮交付清单

### 1.1 新增配置文件

```text
configs/go2w/current_hardware_geometry.yaml        # 0.70×0.43×0.70 m；最高点=固定保护框架
configs/go2w/current_hardware_state.yaml           # 硬件状态清单（绑定几何/外参/内置雷达配置）
configs/go2w/hesai_pandarxt16_extrinsics.yaml      # 正式外参槽（候选未确认，authorizes_* 全 false）
configs/go2w/hesai_pandarxt16_preprocess.yaml      # Pandar 诊断预处理配置
configs/go2w/dual_lidar_safety.yaml                # 双雷达安全融合策略（默认 disabled）
```

### 1.2 新增应用纯模块

```text
app/live_robot/current_hardware.py   # 几何/状态加载校验 + geometry_hash/state_hash
app/live_robot/pandar_clock.py       # PandarClockTier（默认 host_receive_time_only）+ 时钟统计
app/live_robot/stage2_readiness.py   # Stage 2/3 machine-readable readiness（12 项）
```

### 1.3 新增 ROS 包模块

```text
ros2_ws/src/go2w_lidar_preprocessor/go2w_lidar_preprocessor/
    lidar_evidence.py                 # SensorEvidence / EvidenceState / 融合规则（UNKNOWN != CLEAR）
    dual_lidar_observability.py       # 双雷达旋转可观测性（full/requested turn）
    dual_lidar_config.py              # dual_lidar_safety.yaml 门禁加载器
    hesai_diagnostics.py              # Pandar 单帧诊断（zero-return/ring/range/自遮挡/方位占用）
    hesai_pandarxt16_preprocessor.py  # 诊断预处理节点（/go2w/hesai/*）
```

### 1.4 新增脚本

```text
scripts/go2w/validate_pandarxt16_clock.py              # 时钟分级统计
scripts/go2w/capture_dual_lidar_calibration_ros.py     # 双雷达标定原始采集
scripts/go2w/calibrate_pandarxt16_extrinsics.py        # 离线多场景外参标定
scripts/go2w/validate_pandarxt16_extrinsics.py         # 外参验证（thresholds）
scripts/go2w/validate_dual_lidar_observability_ros.py  # 双雷达旋转可观测性只读验证
```

### 1.5 主要修改

```text
app/config.py                         # 新增 PANDARXT16_*/DUAL_LIDAR_*/GO2W_CURRENT_* 设置
.env.example / .env.go2w              # 同步新环境变量（默认全部 fail-closed）
app/reasoning/semantic_navigation/models.py       # SearchDirective 可选 safety_intent / requires_* 字段
app/live_robot/motion_bounds.py       # evaluate_dual_lidar_rotation_gate
app/live_robot/rotation_lease.py      # hardware_binding（expected_binding 校验 + build 助手）
app/live_robot/frame_bundle_reader.py # 可选 pandar/dual_lidar 状态段 + 读取助手
app/live_robot/ui_status.py           # pandar/dual_lidar 状态
app/ui/go2w_live_panel.py             # Pandar/双雷达诊断展示
ros2_ws/.../rotation_crosscheck.py    # build_rotation_evidence 可选 hardware_binding
ros2_ws/src/go2w_lidar_preprocessor/setup.py  # 注册 hesai_pandarxt16_preprocessor 节点
scripts/go2w/start_hesai_pandarxt16.sh            # --with-preprocessor
scripts/go2w/validate_rotation_clearance_physical_ros.py  # finalize 绑定硬件状态
scripts/go2w/run_autonomous_loop.py               # 双雷达门 + 硬件绑定 + --stage2-readiness
README.md                                         # 权威进度同步
tests/conftest.py                                 # conda 测试可导入 ROS 纯模块
```

### 1.6 新增测试（全部通过）

```text
tests/test_current_hardware_geometry.py          # 几何 0.70/0.43/0.70 + fail-closed
tests/test_pandarxt16_zero_return_filter.py      # <=0.05m 零回波不进 obstacle/free/clear
tests/test_pandarxt16_diagnostics.py             # 帧诊断/自遮挡/方位占用
tests/test_pandarxt16_clock_tier.py              # 时钟分级与统计
tests/test_dual_lidar_evidence.py                # 融合规则（occupied 优先 / unknown!=clear）
tests/test_dual_lidar_observability.py           # 双雷达可观测性
tests/test_rotation_lease_hardware_binding.py    # lease 绑定硬件状态
tests/test_stage2_readiness.py                   # 12 项全真才 ready
tests/test_step_search_runner_dual_lidar_gate.py # 双雷达门 + directive 安全意图
tests/test_frame_bundle_pandar_status.py         # bundle 前向兼容 + fail-closed 默认

ros2_ws/src/go2w_lidar_preprocessor/test/test_pandarxt16_preprocessor.py
ros2_ws/src/go2w_lidar_preprocessor/test/test_dual_lidar_observability.py
```

## 2. 现场复核（机器狗重启后）

| 项 | 状态 | 证据 |
|---|---|---|
| 网络 enp6s0 192.168.123.99 | UP | `ip -4 -brief address` |
| 机器人 192.168.123.18 ping | 0% loss | rtt ~10–16 ms |
| Pandar 192.168.123.20 ping | 0% loss | rtt ~12–17 ms |
| `/lf/sportmodestate` | mode=1, error_code=0 | sport state |
| 电池 | SOC 97% | lowstate |
| `/utlidar/cloud` | ~15.4 Hz | rclpy 探针 |
| `/camera/front/image_raw` | ~17.9 Hz | rclpy 探针 |
| `/go2w/odom/wheel` / `/go2w/odom/fused` | ~20 Hz | rclpy 探针 |
| `/go2w/lidar/scan` / `/go2w/sensors/cloud` | ~15.4 Hz | rclpy 探针 |
| `/go2w/safety/lidar_fresh` | true | topic echo |
| `/go2w/safety/rotation_clearance_valid` | **false** | topic echo（fail-closed 正确）|
| front/left/right clearance | inf / nan / nan | left/right = unknown，不授权旋转 |
| motion Action 列表 | 空 | `ros2 action list` |
| Hesai 驱动 | 未运行（隔离诊断用） | `ps` / topic list |

> 注：`ros2 topic hz` 在该环境因 QoS RELIABILITY 不匹配不输出（相机/scan 等为
> BEST_EFFORT），用 rclpy 探针与 topic echo 复核代替，数据流全部正常。

## 3. 双雷达证据模型（本轮核心）

```text
内置 L2  +  PandarXT-16  （禁止简单 concat 后宣称 clear）
   -> SensorEvidence（source/freshness/geometry_tier/bearing/radial_interval/state）
   -> fuse_dual_lidar_evidence
        occupied 优先  /  UNKNOWN != CLEAR  /  candidate TF 不授权 safety
```

- Pandar `extrinsics_validated=false` 时只贡献诊断证据，不贡献正式 CLEAR；
- 时钟 tier 默认 `host_receive_time_only`，metric 融合前必须
  `require_tier_for_metric`；
- 旋转可观测性：Pandar 仅在 validated 后可与内置 L2 互补覆盖扫掠带。

## 4. 当前硬件几何 / 安全门

- `0.70 × 0.43 × 0.70 m`，最高点 = PandarXT-16 固定保护框架；
- `remeasurement_required={length:false,width:false,height:false}`；
- 几何配置本身 `authorizes_motion=false`；
- 执行门顺序：rotation lease step -> step boundary -> rotation clearance ->
  **dual-lidar gate**（enabled 时）-> `_safety_ok` -> arm -> Action -> STOP。

## 5. Stage 2 readiness

`run_autonomous_loop.py --stage2-readiness <out.json>`（不运动）实测：

```json
{"ready": false,
 "checks": {"semantic_v1_ready": true, "pandar_raw_ready": true,
            "pandar_preprocess_ready": true, "pandar_extrinsics_validated": false,
            "current_hardware_geometry_loaded": true,
            "dual_lidar_rotation_observability_valid": false,
            "current_hardware_four_direction_evidence_valid": false,
            "pose_bound_rotation_lease_valid": false,
            "odom_fresh": true, "mode_ok": true,
            "motion_action_available": false, "no_stage2_error": true}}
```

全部 true 才 `stage2_ready=true`；Stage 3 严格依赖 Stage 2 真实 PASS。

## 6. 测试与构建

```text
conda 回归（计划点名）        49 passed
本轮新增 conda 测试           67 passed
合并关键回归 + 新测试          133 passed
ROS go2w_lidar_preprocessor  39 passed（system python + 已 source workspace）
colcon build                 11 packages finished（含 hesai_pandarxt16_preprocessor 脚本）
git diff --check             PASS
```

## 7. 未完成真机项（PHYSICAL VALIDATION PENDING）

- Pandar 多场景外参（≥3 非退化场景：墙角/门框/立柱），门槛 translation residual
  <0.05 m、yaw 多场景一致性 <=3°；脚本已就绪，需真机采集运行；
- Pandar self-occlusion 正式验收（当前用默认模型占位）；
- 当前硬件空场 baseline + front/right/rear/left 四向证据重建；
- Stage 2 active turn-only 一次真机 A/B（需全部 readiness 通过 + 现场授权）；
- Stage 3 semantic short-forward（严格在 Stage 2 PASS 之后）；
- camera physical TF、metric RGB–LiDAR、可靠平移 LIO、map/Nav2（长期阻塞项）。

## 8. 硬性保持

```text
transform_flag=false            /go2w/safety/rotation_clearance_valid=false
authorizes_tf_publication=false  dual_lidar_safety.enabled=false
authorizes_safety_integration=false  semantic forward=false
Pandar authorizes_motion=false   navigation_gate=fail_closed
mount 改变 -> 外参与 lease 自动失效  motion steps=0
```

## 9. 下一位 AI 的接续步骤

1. 先重跑本交接"现场复核"只读检查，确认硬件未再次变化；
2. 运行 Pandar 时钟/自遮挡/多场景外参工具（真机），确认或拒绝候选外参；
3. 硬件固定后重采空场 + 四向证据，用 `validate_rotation_clearance_physical_ros.py
   finalize` 生成绑定当前硬件状态的 lease；
4. `dual_lidar_safety.yaml` 仅在全部证据通过后由操作者确认开启；
5. 全部 readiness 为 true 后，按 Stage 2 命令策略执行一次 turn-only A/B；
6. 之后才考虑 Stage 3 short-forward。

## 10. 最终交接摘要

本轮把 Pandar 从"隔离出云"推进到"诊断预处理 + 双雷达证据融合 + 可观测性 +
安全门 + 硬件绑定 + Stage 2/3 readiness"的完整软件层，同时保持全链路 fail-closed。
SemanticNavigation V1 与内置安全链没有重做或绕过。代码在真实物理证据通过时可直接进入
SemanticNavigation Active turn-only，再进入 short-forward，而不需要再次大改软件架构。

---

# 2026-08-14 现场续接章节（机器狗重启后 + 搜索能力验证）

> 本节记录 2026-08-14 在真实机器狗上完成的全部现场工作、关键发现、未完成项与
> 给下一个 AI 的硬性警告。**读本节之前请先读上方主交接书**。本节目标是让下一位
> AI 不重复本节的弯路、不误判当前状态、并按正确顺序继续。

## A. 已完成的现场工作（本轮真实执行并验证）

### A.1 系统启动与健康检查
- 感知栈（相机/雷达/时间/融合/Bundle）全部重新拉起并验证：相机 ~17.9Hz、
  内置 LiDAR ~15.4Hz、轮式/融合里程计 ~20Hz、lidar_fresh=true。
- Pandar 驱动 + 诊断预处理节点运行：`/hesai/pandarxt16/points_raw` ~10Hz、
  `/go2w/hesai/status` 等诊断话题在线，zero-return ~5.2%（与交接的 4.95% 一致）。
- 运动控制栈启动成功：`/go2w/motion` Action、`/go2w/arm`、`/go2w/emergency_stop`、
  Sport lease 已获取。机器狗 mode=1、error_code=0。

### A.2 只读诊断工具实测
- `validate_pandarxt16_clock.py`：时钟 tier=host_receive_time_only，10.01Hz，
  抖动 0.56ms，双雷达表观偏移 ~50.6s（PTP Free Run，无时间同步），
  metric_fusion_authorized=false（正确）。
- `validate_dual_lidar_observability_ros.py`：内置 L2 0/720 方位可观测
  （旋转包络盲区），Pandar 可覆盖 720/720 但 `extrinsics_validated=false` 故
  rotation observability 保持 false（fail-closed 正确）。

### A.3 当前硬件自过滤修正（重要）
- **发现**：加装 Pandar 后，旧的 self-filter 掩码不覆盖：
  (1) Pandar 保护框架（x 0.3..0.9、y ±0.6、z 0.4..0.88，前伸超旧 head 掩码 x≤0.58）；
  (2) 左右轮胎外沿（|y| 0.37..0.56，右侧原本**没有**车轮掩码）。
  这些机器狗自身回波会被当成假障碍，导致四向交叉校验对比度无法建立。
- **修复**：`configs/go2w/lidar_preprocess.yaml` 新增 `pandarxt16_protective_frame`
  自过滤区域、扩展左右车身掩码到 |y|=0.46、补全右侧前后轮掩码；`measured_at_updated`
  =2026-08-14、`revalidation_required=true`、`revalidation_note` 如实说明。
  修改后 front 近场 0.55..0.80m 完全干净（0 点）。

### A.4 odom 重新锚定（重要）
- 操作者**手动转向**机器狗后，odom/base_link 未跟踪旋转，导致 base_link"前方"
  与物理前方不一致（雷达"front"实际指向物理右前方）。
- **修复**：重启 `go2w_wheel_odom` 节点重新锚定，base_link +x 回到物理前方。
- **警告**：任何手动搬动/转向机器狗后，必须重启 wheel_odom 重新锚定，否则
  所有方位判断都是错的。

### A.5 操作者授权旋转功能（新增代码）
- `run_autonomous_loop.py` 新增 `--operator-authorized-rotation`：**只放宽**
  "旋转需要 pose-bound lease"这一条门，其余全部保留（mode/error、lidar fresh、
  前向净空、运动边界、转向≤30°、单步、急停）。每条运动记录
  `operator_authorized_rotation: true`。这是显式可审计的操作者授权，不是伪造证据。

### A.6 搜索能力验证（关键里程碑）
- **测试 A（单次转向）**：目标"灰色书包"→ 但发现检测器**颜色幻觉**（见 B.2）。
- **绿色垃圾桶真实检测**：0.95 置信度，开放式验证确认绿色（无幻觉）。
- **语义 reasoner 决策→实际转向闭环**（ACTIVE 模式）：
  - reasoner 输出 `inspect_anchor`、partial_match、置信度 0.877、
    preferred_heading_delta_deg=30°（转向 30° 检查锚点）、attribute_support=green；
  - semantic_step `l30` ≠ legacy_step `r30`（与旧式扫描判断相反，独立决策）；
  - **实际转向 29.5°**，odom 验证 yaw_delta 0.514 rad ≈ expected 30°；
  - 单步即停，STOP/disarm。
- 证据：`outputs/go2w_acceptance/semantic_turn_execute.jsonl` + `semantic_turn_execute.mp4`。

### A.7 修复一个真实 bug
- `app/reasoning/semantic_navigation/models.py` 的 `GoalGraph.to_dict()` 引用了不存在的
  `self.observation_memory`，导致 observe 模式输出序列化崩溃。已修复，17 项
  SemanticNavigation 回归测试通过。**不要重新引入**。

## B. 关键发现（下一位 AI 必须知道）

### B.1 办公环境对四向交叉校验的限制
- 当前办公环境四周 1.0..1.5m 内有真实物体（桌椅/纸箱/墙），各方位 ROI 内
  均有回波，标记物**遮挡背景而非增加点数**，`target_contrast` 无法通过。
- **四向物理证据 + lease 无法在当前位置完成**，需要 ≥1.5m 净空的场地。

### B.2 LLM 检测器颜色幻觉（重要）
- 提示词"搜索灰色书包"时，检测器报告"灰色书包 0.85"，但视野里只有**黑色背包**
  （柜子上/纸箱上），被颜色属性带偏（黑→灰）。
- 开放式提问（不提示目标）时，视觉模型**不**提书包——证明是提示词诱导幻觉。
- **警告**：LLM 检测器对目标颜色不可靠，目标颜色与实际物体不符时会误报。
  验证检测时应先做开放式确认或加颜色属性强校验。

### B.3 ros2 工具 QoS 注意
- `/camera/front/image_raw`、`/go2w/lidar/scan`、`/go2w/sensors/cloud`、
  `/hesai/pandarxt16/points_raw` 等为 BEST_EFFORT。用 `ros2 topic hz`（默认
  RELIABLE）会收不到数据；需用 rclpy 探针或 `--field` echo 验证。
- Pandar 大消息（2MB/帧）对**新订阅者**在 CycloneDDS 下会掉帧，长驻节点正常。

### B.4 命令工具（沙箱/安全分类器）间歇性超时
- 本会话中，含机器人运动命令的 Bash/Write 频繁被安全分类器判为"临时不可用"。
  多次重试（3-8 次）后通常能通过。下一位 AI 遇到时请耐心重试，或用
  `echo` 探测分类器是否恢复后再跑运动命令。

## C. 未完成项（还没干）

| 项 | 状态 | 阻塞原因 |
|---|---|---|
| 四向物理证据（front/right/rear/left + baseline） | **未完成** | 办公环境 ROI 内有杂物，对比度无法建立；需净空场地 |
| pose-bound rotation lease | **未生成** | 依赖四向证据 |
| Pandar 多场景外参标定 | **未完成** | 需 ≥3 非退化场景（墙角/门框/立柱），真机采集 |
| Pandar self-occlusion 正式验收 | **未完成** | 当前用默认模型占位 |
| `dual_lidar_safety.yaml` 启用 | **未启用**（正确） | 需 validated extrinsics |
| Stage 2 正式 readiness（12 项全真） | **false** | pandar_extrinsics、observability、四向、lease、motion_action 未齐 |
| Stage 3 short-forward | **未开始** | 依赖 Stage 2 PASS |
| 多步语义搜索（max-motion-steps>1） | **未测试** | 每步都需要操作者授权确认 |
| 灰色书包颜色幻觉修复 | **未做** | 需改检测链路（颜色强校验） |
| green trash can 单次转向测试 | **跳过**（操作者要求） | 操作者选择直接推进 |

## D. 下一步（给下一个 AI 的执行顺序）

1. **只读恢复现场**：`git status --short`、`git diff --check`、确认感知栈/运动栈/
   Pandar 诊断栈运行状态、机器狗 mode=1。
2. **确认硬件未再变化**：机器狗是否被搬动/转向。**若被搬动/转向 → 重启
   `go2w_wheel_odom` 重新锚定**，否则方位判断全错。
3. **换净空场地**（四周 ≥1.5m）后重采四向证据：
   ```bash
   bash scripts/go2w/start_live_perception.sh
   /usr/bin/python3 scripts/go2w/validate_rotation_clearance_physical_ros.py \
     capture --role baseline --output outputs/.../baseline.json
   # 操作者依次放标记物 front/right/rear/left（每方向 ~0.7-1.2m）
   /usr/bin/python3 scripts/go2w/validate_rotation_clearance_physical_ros.py \
     finalize --baseline ... --front ... --right ... --rear ... --left ... \
     --operator <name> --physical-clearance-radius-m 0.6 \
     --swept-clearance-confirmed --standing-posture-confirmed --output <lease.json>
   ```
4. **Pandar 多场景外参**：用 `capture_dual_lidar_calibration_ros.py` 采 ≥3 场景 →
   `calibrate_pandarxt16_extrinsics.py` → 门槛 translation residual<0.05m、
   yaw 一致性≤3°。
5. **正式 Stage 2 readiness**：`run_autonomous_loop.py --stage2-readiness <out.json>`，
   12 项全真后才考虑正式 active turn-only。
6. **多步语义搜索测试**（操作者授权 + 单步递增）：验证 reasoner 连续决策稳定性。
7. **修复检测颜色幻觉**（若目标需颜色区分）：加开放式确认或颜色属性强校验。

## E. 硬性警告（避免做错）

```
1. 不要 reset/clean dirty worktree；不要删 ros2_ws/src/hesai_ros_driver；
   不要清空 data/memory/*.jsonl；不要批量恢复 configs/go2w/*。
2. 机器狗被搬动/转向后必须重启 wheel_odom 重新锚定，否则 base_link 方位全错。
3. 不要篡改厂商官方 official_reference.yaml；当前硬件几何 0.70x0.43x0.70m 已确认，
   不要再测长宽高。
4. Pandar 保护框架/轮胎自回波是机器狗自身，必须保持自过滤掩码；不要"优化"成
   不加掩码（否则四向对比度永远建立不了）。
5. LLM 检测器会颜色幻觉：目标颜色与画面不符时可能误报；验证检测先用开放式确认。
6. 四向证据需要净空场地（≥1.5m）；在办公环境硬采只会得到 contrast=false。
7. --operator-authorized-rotation 是显式操作者授权：必须在操作者确认场地安全 +
   遥控器急停在手时使用，且保持 turn-only/单步/≤30°。不要把它当常规开关。
8. 不要伪造 lease/证据；rotation_clearance_valid 保持 false 直到真实证据通过。
9. Stage 2 正式 readiness 需要 pandar_extrinsics_validated 和
   dual_lidar_rotation_observability_valid——这两项需要真机多场景标定，不能跳过。
10. 运动命令若被沙箱分类器判"临时不可用"，耐心重试（3-8 次），不要放弃。
```

## F. 当前系统状态（2026-08-14 结束时）

```
感知栈（相机/雷达/里程计/融合/Bundle）: 运行中
Pandar 驱动 + 诊断预处理: 运行中
运动控制栈（/go2w/motion + lease）: 运行中
机器狗: mode=1, error_code=0, 静止
/go2w/safety/rotation_clearance_valid: false（fail-closed 正确）
运动测试产物: outputs/go2w_acceptance/op_auth_turn_{a,c}.mp4、semantic_turn_execute.mp4
```

---

## 2026-08-14 续接摘要

本轮在真实机器狗上完成了：感知/运动/Pandar 栈启动与验证、时钟/可观测性诊断、
**当前硬件自过滤修正**（Pandar 框架 + 轮胎掩码）、**odom 重锚定**、**操作者授权
旋转功能**、**单次转向验证**、**检测颜色幻觉定位**、**SemanticNavigation reasoner 决策→实际
转向 30° 闭环验证**，以及一个真实 bug 修复（GoalGraph.to_dict）。四向证据/lease/
Pandar 外参/Stage 2 正式 readiness 因办公环境与真机标定未完成而延后，下一步应在
净空场地重采四向证据并推进多场景外参标定。
