# robot_scene_demo × Go2-W × SemanticNavigation 阶段交接书（重点：外接 PandarXT-16）

> 交接日期：2026-08-13（Asia/Shanghai）  
> 项目：`/home/brov/robot/robot_scene_demo`  
> 分支：`main`  
> 基线 HEAD：`e861b953ba195e1a47bcdb9491f955e69c5ae451`  
> 工作树：大量未提交修改和新增文件，必须原地接续，禁止 reset/checkout/clean  
> 用途：把本文、README 和两份计划书交给下一位 AI 后，可无缝继续代码与真机工作。

---

## 0. 一页结论

1. 两份用户计划书是：
   - `/home/brov/下载/robot_scene_demo_go2w_builtin_rgb_lidar_codex_implementation_plan.md`
   - `/home/brov/下载/robot_scene_demo_SemanticNavigation_语义搜索融合详细改动计划书_20260813.md`
2. 内置 RGB、内置 LiDAR/IMU 时间桥、相机内参、静态 LiDAR 预处理、Frame Bundle、
   状态机、LLM quick/verify 和 SemanticNavigation 风格语义搜索主体已实现。
3. SemanticNavigation V1 代码层基本完成；真实 Stage 1 observe-only/shadow 已验证，但没有运动。
   Stage 2 active turn-only 和 Stage 3 short-forward 尚未通过。
4. 内置 LiDAR 的正前碰撞高度误报和左轮自回波已修正，但它对整机 `0.511 m` 原地旋转
   扫掠环带在 720/720 个方向存在不可观测空间。因此：
   - `/go2w/safety/rotation_clearance_valid=false`；
   - 左右正式 clearance 是 `unknown/NaN`，不是“检测到障碍”；
   - 不得凭空场点云开放转向。
5. 用户后来在顶部加装了 **Hesai PandarXT-16**。它属于计划外扩展，目前已经：
   - 正确识别并重定向到工作站；
   - 使用官方 ROS 2 驱动稳定出云；
   - 通过隔离的 20 帧只读验收；
   - 生成照片+地面拟合的候选安装外参。
6. 新雷达尚未发布正式 TF，尚未接入 `/go2w/lidar/*`、self filter、安全或运动链，
   尚未完成精确外参、时间同步、整机外廓和遮挡验收。
7. 加装雷达改变了质量、最高点、电缆外廓、自遮挡和潜在扫掠包络。安装前的
   `baseline/front` 四向物理证据全部不得再用于运动授权，必须重做。
8. 正式能力不能写成完整 Level A–F：Level A 仍缺相机 TF；历史部分 Level B 小范围
   运动不代表新硬件安全；Level C–F 继续被正式外参、平移 LIO、地图和 Nav2 阻断。
9. `configs/go2w/navigation_gate.yaml` 相对 Git 基线没有 diff，继续 fail-closed。
10. 本交接时 Hesai 驱动已停止；内置只读感知节点仍运行；ROS Action 为空；机器狗
    `mode=1`、`error_code=0`、速度与 `yaw_speed` 均为 0。

---

## 1. 权威资料和阅读顺序

下一位 AI 修改任何代码前应依次完整阅读：

1. 本交接书；
2. `README.md` 顶部“Go2-W 真机项目当前进度还原指南（2026-08-13）”；
3. `reports/go2w_semantic_navigation_semantic_search_handoff_20260813.md`；
4. `reports/go2w_pandarxt16_mount_candidate_20260813.md`；
5. 两份用户计划书；
6. `reports/go2w_codex_handoff_20260813.md`；
7. `reports/go2w_codex_handoff_20260807.md`；
8. `reports/go2w_codex_continuation_status_20260806.md`；
9. 以下安全配置：

```text
configs/go2w/navigation_gate.yaml
configs/go2w/search_policy.yaml
configs/go2w/lidar_preprocess.yaml
configs/go2w/official_reference.yaml
configs/go2w/sensor_extrinsics.yaml
configs/go2w/rgb_lidar_fusion.yaml
configs/go2w/hesai_pandarxt16.yaml
configs/go2w/hesai_pandarxt16_mount_candidate.yaml
```

资料冲突时按以下优先级：

```text
当前实时只读事实 / 最新证据 JSON
> 本交接书
> 最新 handoff 的后置补充章节
> README 顶部权威进度
> 较早交接书和计划假设
```

旧报告中“左右近障碍”和 RGB–LiDAR `fusion_ready=true` 等结论已被后续实测修正，
不得复活旧结论。

---

## 2. 仓库、环境和工作树

### 2.1 Git 快照

```text
repo:   /home/brov/robot/robot_scene_demo
remote: https://github.com/BROVVV/robot_scene_demo.git
branch: main
HEAD:   e861b953ba195e1a47bcdb9491f955e69c5ae451
HEAD message: docs: add 2026-08-13 handoff for next AI session
tracked diff: 52 files changed, 3177 insertions(+), 191 deletions(-)
```

统计不含大量 untracked 模块和嵌套 Hesai Git 仓库。当前 dirty worktree 就是阶段成果。

严禁：

```text
git reset --hard
git checkout -- .
git clean -fd / -fdx
删除 ros2_ws/src/hesai_ros_driver
清空 data/memory/*.jsonl
批量恢复 configs/go2w/*
```

开始时先保存：

```bash
git status --short
git diff --check
git diff --stat
```

提交边界由用户决定；不要擅自提交私人输出、模型数据或第三方嵌套 Git 历史。

### 2.2 双 Python 环境

必须保持：

```text
应用 / Streamlit / 模型 / SciPy：Conda go2_robot_scene_demo
ROS 2 Humble 节点和消息：      /usr/bin/python3 (Python 3.10)
```

Hesai 构建若继承 Conda，曾因 `em` 模块冲突失败。统一构建脚本已固定系统 Python，并
设置 `WITH_PTCS_USE=OFF`。不要让 Conda Python 直接加载 Humble `rclpy`。

---

## 3. 用户授权与当前安全边界

用户曾给出：

```text
GO2W_MOTION_READY
```

授权仅覆盖：以授权时位姿和朝向为固定原点，原始正前方 180° 半平面、半径 <=1.5 m。
代码已有固定前半平面、最大半径、turn-only 和单步门；转向后不能重定义“当前前方”。

但当前仍不得运动，因为：

- 新装 Pandar 后旧硬件几何和旧四向证据失效；
- 内置 L2 对旋转包络存在全方向不可观测空间；
- Pandar 未完成正式外参、自遮挡、近场和整机包络验收；
- `rotation_clearance_valid=false`；
- 没有当前可用的 pose-bound rotation lease；
- 当前 ROS 图中没有 motion Action、lease holder 或 arm 服务。

如果机器狗被搬动、重启后位姿不确定，或无法证明仍在授权原点，必须重新向用户确认并
记录 origin，不能沿用旧坐标。

永久禁止：

```text
/lowcmd / LowCmd
关节位置、速度、力矩控制
ReleaseMode() / Damp()
修改固件或关闭厂商安全保护
由 LLM/SemanticNavigation 直接发布运动
把 RGB 像素或候选图节点伪装成米制 map goal
```

---

## 4. 原 Go2-W 完整计划：逐阶段状态

### 4.1 分级能力总表

| 级别 | 当前状态 | 已有成果 | 当前缺口 |
|---|---|---|---|
| Level A 实时感知 | PARTIAL | 标准 RGB、CameraInfo、内参、实时 Bundle、检测/证据门/UI | 相机物理 TF 未测，按计划定义不能称完整 Level A |
| Level B 短步搜索 | HISTORICAL PARTIAL / 当前硬件 BLOCKED | 状态机、历史小范围搜索、STOP/verify、运动边界软件门 | 新雷达使旧旋转证据失效；当前不能复用历史运动 PASS |
| Level C 三维定位 | SOFTWARE CORE / PHYSICAL BLOCKED | 融合、投影、聚类和诊断 overlay 软件 | camera TF 与导航级 RGB–LiDAR 外参未通过，禁止 metric 3D |
| Level D LIO/空间记忆 | PARTIAL / BLOCKED | Point-LIO 静止 PASS、yaw 可用、wheel/fused 实验接线 | Point-LIO 平移失败，无可靠运动尺度和完整 metric memory |
| Level E Nav2 plan_only | BLOCKED | `/scan`、Nav2 软件包和配置存在 | Level D、map、完整 TF、现场 planner health 未通过 |
| Level F Nav2 execute | BLOCKED | arbiter/watchdog/bridge 核心和保守配置存在 | 全套实时门、接管、地图、定位及实机验收未完成 |

### 4.2 阶段 0–1：审计、基线、环境和回归

状态：**核心 PASS；完整外部服务全量回归不能宣称全 PASS**。

- 正确项目路径、remote、branch、HEAD 和旧 Go2-W 证据已审计；
- 既有修改和 memory 文件得到保护；
- Conda/ROS Python 隔离保持；
- 原 mock、视频/search、live_robot 核心回归多轮通过；
- 广泛回归最新记录：`244 passed, 1 deselected in 373.74s`；
- 计划点名的离线 unittest 后续 63 项通过；
- 全目录曾在旧 `tests/test_task_planner.py` 的真实 TLS/API 调用等待 13 分钟后人工停止。
  这不是本轮断言失败，但也不能写成最新完整全量 PASS。

### 4.3 阶段 2：内置 RGB ROS 2 桥

状态：**桥和内参 PASS，相机 TF BLOCKED**。

- `go2w_camera_bridge` 发布 Image、CompressedImage、CameraInfo 和诊断；
- 生产输入固定为只读 VideoHub RPC，损坏帧跳过并自动重连；
- 1920×1080 输出；
- 9×6 内角点、15 mm 棋盘、105 视角物理内参；
- PnP sanity 均值约 0.859 px、RMS 约 1.024 px；
- LLM 前原子快照修复了 spool 清理竞态；
- 官方 URDF 无相机 link，真实 `base_link -> front_camera_*` 未测。

### 4.4 阶段 3：时间域诊断

状态：**内置 LiDAR/IMU PASS**。

- 120 秒拟合并写入 `time_sync.yaml`，云 RMSE <1 ms；
- 保留 LIO 原始相对时间，另发 ROS 对齐话题；
- stale 有明确门禁。

Pandar 不属于该 PASS：其 PTP 为 Free Run，目前用 host receive time，仍需独立评估。

### 4.5 阶段 4：URDF、TF 与物理几何

状态：**内置 LiDAR 静态几何 PASS；完整 TF 和 Pandar TF BLOCKED**。

- 固定 Unitree 官方 Go2-W URDF 和 L2 资料；
- 厂家外廓 `0.70 × 0.43 × 0.50 m`，轮胎 7 英寸；
- `base_link -> utlidar_lidar` 已现场复核，z-up pitch `-15.09°`；
- `utlidar_lidar -> utlidar_imu` 已固定；
- `physical_measurements.yaml` 未测字段未用猜值伪装 confirmed；
- Pandar 只有 candidate，正式驱动 `transform_flag=false`。

### 4.6 阶段 5：LiDAR 预处理与近场保护

状态：**直行碰撞高度链 PASS；旋转 BLOCKED**。

已完成：NaN/Inf/距离/高度/体素/自车/地面处理、全高度语义点、碰撞高度点、720-bin
scan、front clearance、freshness/stale、wheel self-return A/B、Bundle 状态，以及
rotation cross-check/短时 lease 软件。

关键修正：旧约 0.58 m 正前“障碍”位于站立碰撞高度以上；左右部分近点是轮组/近场。
当前正式语义：

```text
front: no_return
left: unknown
right: unknown
rotation_clearance_valid: false
```

旋转可观测性：

```text
unobservable bearings: 720 / 720
front/rear blind radial length: 0.040 m
left/right blind radial length: 0.155 m
worst sampled gap: 0.290780 m
minimum-range affected: 584 / 720
named self-region affected: 482 / 720
```

仅改 YAML 的 `valid` 会被加载器拒绝；必须提供完整物理证据合同。

### 4.7 阶段 6：LIO 与里程计

状态：**静止和 yaw 部分 PASS；平移 BLOCKED**。

- 官方 Point-LIO 隔离 Noetic 路径可运行，5 分钟静止通过；
- ±10° yaw 符号正确、幅值约 89%，项目仅用其 yaw；
- 平移在 0.2/0.4 滤波下出现偏 69°、塌缩或发散；
- wheel odom 为 dq×0.089 m 实验估算，转向跳过平移；
- fused odom 仍 EXPERIMENTAL；USLAM 当前固件未启用。

### 4.8 阶段 7：RGB–LiDAR 三维融合

状态：**软件/诊断 overlay PASS；导航级 3D BLOCKED**。

- 录包、提取、PnP、投影、聚类、overlay 和门禁已实现；
- 现有候选 7 场景、33.6 px，只允许诊断；
- 已修复 experimental 被误写 confirmed/validated 的风险；

```text
rgb_lidar_overlay_ready=true
rgb_lidar_extrinsics_validated=false
fusion_ready=false
authorizes_3d_output=false
metric 3D topics=0 messages
```

仍需相机 TF、非退化多场景外参和移动位置复核。

### 4.9 阶段 8–9：Frame Bundle 与实时搜索

状态：**Frame Bundle PASS；2D 实时观察/证据 PASS；metric target BLOCKED**。

- 原子 `image.jpg + frame_bundle.json + READY`；有界 spool 和 session 隔离；
- 603.24 秒 soak：489 Bundles、0.816 Hz、末帧年龄 0.354 s、最多 30 Bundle；
- Bundle 区分相机、内参、LiDAR、LIO、TF、RGB–LiDAR 两级和 clearance 状态；
- LLM quick/crop verify、tracking、scene graph、PSG、evidence gate、memory 可用；
- 显式关系任务要求关系 SceneGraph evidence；
- 真实“蓝色垃圾桶 near 饮水机”三帧复放得到 visual_confirmed；
- 空间状态仍为 `target_2d_only`，模型 route plan 未执行。

### 4.10 阶段 10–11：状态机与安全配置

状态：**软件 PASS；当前新硬件运动 BLOCKED**。

- 状态机和 candidate approach->verify->target_reached 主链保留；
- reasoner 只插入 target-not-found / SELECT_NEXT_VIEW；
- 修复 arm-before-gate，仅单步全门通过后即时 arm；
- 历史 scan360/level_a/state_machine 小范围搜索通过，但不能证明新硬件包络安全；
- 默认仍为 `motion_enabled=false`、semantic disabled/legacy/shadow/no-forward；
- max turn 30°、forward 0.3 m、front clearance 0.6 m、timeout 0.3 s；
- Nav2 disabled/execute false。

### 4.11 阶段 12–14：控制、Nav2、UI

状态：**core/显示部分完成，执行能力 BLOCKED**。

- arbiter、disabled bridge、限幅、300 ms watchdog 和 rotation gate 有测试；
- 当前不创建 Action client、不启动 lease；
- 遥控接管、lease 丢失、Collision Monitor/Velocity Smoother 端到端未完成；
- 无可靠平移 LIO、map、`map->odom`、完整 TF，Nav2 plan_only/execute 均不能称 PASS；
- UI 已扩展 live target/semantic/传感器状态，但完整 Level F 控制 UI 未验收；
- 旧 Streamlit AppTest 有长超时基线问题。

---

## 5. SemanticNavigation 语义搜索计划完成情况

### 5.1 已实现

`app/reasoning/semantic_navigation/` 已有 models、GoalGraphBuilder、GraphMatcher、SemanticMemory、
reasoner/router 和 auxiliary hints：

- 复用 `TargetProfile`；target/explicit anchor/inferred context 分级；
- 不写死 room/object prior；
- 复用 `ObservedSceneGraphBuilder`；
- exact/alias/lexical/attribute/relation/context 可解释匹配；
- zero/partial/strong，无 confirmed；
- event-driven `LiveSemanticObserver`、TTL、heading/profile/pose cache；
- session negative memory、TTL、sector penalty、解封；
- 长期 ObservationMemoryStore 只读检索，不永久写每次失败 scan；
- legacy/semantic_navigation/hybrid；
- directive 转 StepPlan，转向硬 clamp <=30°；
- forward 默认禁用；异常、低置信度自动 fallback legacy；
- PSG/situated prior 只作弱辅助，不能覆盖视觉事实/负证据；
- replan throttling 已实际接线。

Runner/视觉链还完成：

- 仅 detection recoverable error 或 `best is None` 调语义层；
- candidate 仍优先 approach/verify；
- shadow 实际执行 legacy；active 仍需全部安全门；
- 修复 LLM 0-based index、关系端点映射和颜色属性；
- 不同标签相似度低于 0.55 禁止错误串轨；
- 关系目标 crop 包含最近视觉锚点；
- `TARGET_CONFIRMATION_REQUIRE_RELATION_EVIDENCE`；
- strong graph match 不直接确认目标。

配置/CLI 已支持 semantic、legacy|semantic_navigation|hybrid、shadow|active、no-forward。仓库默认仍是：

```text
disabled + legacy + shadow + no-forward
```

回滚只需去掉 `--semantic-reasoning` 或使用 `--search-reasoner legacy`。

### 5.2 输出和 A/B

observe-only 可输出 GoalGraph、SceneGraph、match、directive 和 provenance。离线 evaluator 已
在三份真实 session replay，均满足：

```text
actual_shadow_behavior_matches_legacy=true
dangerous_forward_request_count=0
```

### 5.3 真机分级

| 阶段 | 状态 | 结论 |
|---|---|---|
| Stage 1 Observe-only | PASS | 真实 Bundle/LLM/Graph/关系证据和 2D visual confirmation 通过 |
| Stage 1 Shadow | PASS（无运动） | zero-match 语义与 legacy 等价，rotation gate 在 arm 前拒绝，odom delta=0 |
| Stage 2 Active turn-only | BLOCKED | 旋转包络不可观测；新雷达使旧 baseline 作废 |
| Stage 3 Hybrid short-forward | NOT STARTED/BLOCKED | 必须先有 Stage 2 证据，且 forward 继续过原安全链 |
| V2 metric graph | BLOCKED | camera TF、正式 RGB–LiDAR/Pandar 外参未通过 |
| V3 map+Nav2 | BLOCKED | Level D/map/Nav2 gate 未通过 |

---

## 6. 计划外的重要修复

1. 固定初始朝向的 180° 前半平面和 1.5 m 半径门；
2. `turn-only`、`max-motion-steps=1`；
3. LiDAR 全高度语义点与碰撞高度点分离；
4. 修正正前高处假障碍和左轮自回波；
5. 720 方位旋转可观测性硬审计；
6. rotation valid 物理证据合同和 pose-bound 短时许可；
7. 修复 state machine 提前 arm；
8. Bundle 的 `inf/NaN` 状态语义；
9. 实验 RGB–LiDAR overlay 与 metric geometry 分级；
10. 真机长期 Observation Memory 只读接线；
11. 输入帧原子快照，消除远程推理竞态；
12. 实测厂商 `SwitchAvoidMode()` 返回 3203（API 未实现），未误报避障开启；
13. 识别、重定向、构建并隔离验收用户后加的 PandarXT-16。

---

## 7. 重点：新加装 Hesai PandarXT-16

### 7.1 新硬件为什么使旧证据失效

原计划明确以内置 RGB、LiDAR、IMU 为主。用户在四向物理标定期间把 PandarXT-16 安装
到顶部，因此这是后加硬件。它改变质量/重心、整机最高点、松弛电缆动态外廓、自遮挡、
互相遮挡和潜在旋转扫掠半径。

安装发生在旧 `front_v2.json` 等候选采集过程之后。无论旧文件局部检查是否通过，均不得
用于当前硬件运动许可。

### 7.2 设备识别

已通过设备 Web 状态和 UDP/驱动解析确认：

```text
model:       Hesai PandarXT-16
device IP:   192.168.123.20
laser count: 16
firmware:    1.2.8
spin:        600 rpm
FOV:         360 deg
standby:     false
PTP:         Free Run
```

设备序列号曾读取，但本交接书不重复传播硬件标识。

### 7.3 网络重定向

初始点云目的端是 `192.168.123.18:2368`；`.18` 是 Go2-W 的 NVIDIA Orin NX，不是雷达。
用户明确给出 `PANDAR_REDIRECT_READY` 后，按“写前读取 -> 只改一个字段 -> 写后读取”仅把
destination IP 改成 `192.168.123.99`。

保留 UDP 2368、GPS 10110、600 rpm 和其他配置。实际流量：

```text
192.168.123.20:10000 -> 192.168.123.99:2368
```

`tcpdump` 已确认持续数据。需要时可通过同一设备配置接口把 destination 回滚到 `.18`；
不要同时改端口、转速或固件。

交接时网络：

```text
enp6s0 = 192.168.123.99/24, UP
192.168.123.18 ping: 0% loss
192.168.123.20 ping: 0% loss
```

### 7.4 官方驱动与构建

路径：`ros2_ws/src/hesai_ros_driver`，是主仓库中的 untracked 嵌套 Git：

```text
HesaiLidar_ROS_2.0 HEAD:
e7e112f0809f0eed5e3c81c55a1a0376474db234  (driver 2.0.12)

SDK submodule:
9d5dc4fc4ade5be5f6a6ca00e71dd4050b054168
```

官方版本支持 PandarXT-16、Ubuntu 22.04 和 ROS 2 Humble。

构建注意：

- 无免密 sudo，未安装 `libpcap-dev`；实时 UDP 路径不需要系统 libpcap；
- 继承 Conda 会因 `em` 冲突失败；
- 应运行 `bash scripts/go2w/build_ros2.sh`；
- 最新统一构建 11 个 ROS 包全部完成，只有 CMake 信息/未使用变量提示。

### 7.5 隔离配置、话题与启动

配置：`configs/go2w/hesai_pandarxt16.yaml`。

```text
source_type=1
device=192.168.123.20
udp_port=2368
PTC=9347 plain TCP
host=192.168.123.99
multicast_ip_address=""     # 单播必须为空
device_udp_src_port=10000
use_timestamp_type=1        # host receive time
transform_flag=false
frame=pandarxt16_link_unvalidated
```

ROS 输出：

```text
/hesai/pandarxt16/points_raw   sensor_msgs/msg/PointCloud2
/hesai/pandarxt16/packet_loss  hesai_ros_driver/msg/LossPacket
```

它没有接到 `/utlidar/cloud`、`/go2w/sensors/cloud`、`/go2w/lidar/*`、TF、安全、Action
或 Nav2。绝不能把约 15.4 Hz 的 `/utlidar/cloud` 误认成 Pandar。

只读启动和验收：

```bash
cd /home/brov/robot/robot_scene_demo
bash scripts/go2w/start_hesai_pandarxt16.sh

# 第二终端
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
/usr/bin/python3 scripts/go2w/validate_hesai_pandarxt16_ros.py \
  --samples 20 \
  --output outputs/go2w_acceptance/hesai_pandarxt16_20260813/result.json
```

启动器只做网络/路由/驱动检查和诊断出云，不启动 Sport、lease、Action、Nav2 或安全发布。

### 7.6 20 帧真机验收

证据：`outputs/go2w_acceptance/hesai_pandarxt16_20260813/result.json`。

```text
checks: 12/12 passed
frames: 20
header rate: 10.0032 Hz
arrival rate: 9.9958 Hz
points/frame: 64000
fields: x, y, z, intensity, ring, timestamp
rings: 0..15 全部有有效回波
header stamps: 严格递增
point timestamp span/revolution: 约 0.10 s
minimum valid return fraction (>5 cm): 0.949875
median valid return fraction: 0.950219
packet counter: 853181
reported packet loss: 0
diagnostic_only: true
authorizes_motion: false
authorizes_safety_integration: false
```

约 4.95% 是零/近零返回。未来预处理必须显式过滤；不能因值 finite 就把 `(0,0,0)` 当
真实障碍或自由空间。

### 7.7 角度修正、firetime 与时钟

- PTC `192.168.123.20:9347` 成功；
- 驱动从雷达读取 angle correction 成功；
- 日志会显示 `FATAL load firetime error`，但官方 SDK 明确 firetime 可选且缺失不阻止
  运行；不要伪造文件压日志；
- 缺 firetime 仍是高精度几何限制；
- PTP Free Run，目前使用 host receive timestamp；
- 正式融合前需评估 PTP/GPS 或建立独立时钟映射。

### 7.8 照片和点云推断的安装候选

详见：

```text
configs/go2w/hesai_pandarxt16_mount_candidate.yaml
reports/go2w_pandarxt16_mount_candidate_20260813.md
```

照片中机器狗正前方朝右；安装脚接近纵向中线、位于前肩轴后方。候选：

```text
base_link -> pandarxt16_link_unvalidated
translation: x=+0.130 m, y=+0.015 m, z=+0.014 m
rotation: roll=+0.385 deg, pitch=+0.905 deg, yaw=+11.357 deg
```

不确定度：

```text
x +/-0.060 m
y +/-0.035 m
z +/-0.040 m
roll/pitch +/-1 deg
yaw +/-15 deg
```

Pandar 地面拟合：

```text
z = 0.01579893*x - 0.00672136*y - 0.54194777
sensor height above floor = 0.541868 m
inliers = 3433
RMSE = 0.005402 m
level tilt = 0.984 deg
```

它对离地高度和 roll/pitch 较强，不能决定 x/y/yaw。8+8 帧双雷达静态配准给出 yaw
约 `+11.357°`、y 约 `+0.014 m`，但：

```text
median nearest-neighbor error = 0.3516 m
fraction within 0.10 m = 13.54%
```

所以只作 yaw 初值/y 符号提示，拒绝作为 metric calibration。

候选配置的 `confirmed`、`authorizes_tf_publication`、`authorizes_preprocessor_integration`、
`authorizes_safety_integration`、`authorizes_motion` 全部为 false。正式驱动仍
`transform_flag=false`。不得把候选直接发布为正式 TF。

### 7.9 整机包络候选

照片中最高点是松弛电缆环，明显超过原 0.50 m 厂家高度：

```text
candidate full height = 0.65 +/-0.05 m
```

水平投影看起来仍在原 `0.70 × 0.43 m` 内，因此旧 `0.511 m` 扫掠半径只作为候选保留。
照片不能证明电缆运动时不甩出、支架所有边缘都在 envelope 内或姿态变化不改变外廓，故
`0.511 m` 不是新硬件 validated envelope。

### 7.10 当前运行状态

```text
Pandar device: online, pingable
Pandar destination: 192.168.123.99:2368
hesai_ros_driver_node: stopped
/hesai/pandarxt16/points_raw: no publisher / unknown topic
project read-only nodes: camera/time/lidar/fusion/live_bridge/wheel_odom running
ROS Action list: empty
/go2w/safety/lidar_fresh: true
/go2w/safety/rotation_clearance_valid: false
robot mode: 1
robot error_code: 0
robot velocity: [0,0,0]
robot yaw_speed: 0
```

驱动停止不改变设备 destination；雷达若上电仍向 `.99:2368` 发 UDP，只是无人消费。不要
擅自做 systemd 常驻或接入主感知栈。

### 7.11 Pandar 最短正确接续路径

1. 固定支架和顶部电缆；任何重新安装都使候选失效。
2. 实测安装脚中心相对 body/base 的 x/y/z、前向标志、最高点和最外点，记录方法、操作者、
   时间和不确定度。
3. 至少采三个有非平行墙面/门框/立柱的双 LiDAR 静态场景；单地面不能约束平面内平移/yaw。
4. 建议验收：translation residual <0.05 m，yaw 多场景一致性 <=3°。
5. 评估 Free Run 漂移，决定 PTP/GPS 或 host clock map。
6. 先做新 namespace 的 diagnostic preprocessor，明确过滤 <=5 cm 零返回；不得替换
   `/utlidar/cloud`。
7. 验证机身、支架、电缆、轮腿和两雷达互遮挡，先 read-only A/B。
8. 实测新整机 envelope，重新计算 footprint/swept radius。
9. 硬件固定后重新采空场 baseline 和 front/right/rear/left；旧文件只保留历史。
10. 全部通过后再决定 Pandar 是互补诊断还是进入有 provenance 的融合安全层；禁止简单
    拼云后开放转向。

---

## 8. 关键证据索引

### 8.1 内置感知与安全

```text
outputs/go2w_acceptance/camera_bridge_live/result.json
outputs/go2w_acceptance/time_bridge_live/result.json
outputs/go2w_acceptance/level_a_stationary_soak_fixed/result.json
outputs/go2w_acceptance/lidar_collision_height_live_20260813/result.json
outputs/go2w_acceptance/lidar_wheel_self_filter_live_20260813/result.json
outputs/go2w_acceptance/lidar_current_scene_recheck_20260813/result.json
outputs/go2w_acceptance/lidar_rotation_observability_20260813/result.json
outputs/go2w_acceptance/live_bundle_current_scene_20260813/result.json
```

### 8.2 RGB–LiDAR 与 SemanticNavigation

```text
outputs/go2w_acceptance/rgb_lidar_geometry_tier_20260813/result.json
outputs/go2w_acceptance/frame_bundle_geometry_tier_20260813/result.json
outputs/live_runs/semantic_navigation_observe_20260813_01/
outputs/live_runs/semantic_navigation_observe_20260813_03/
outputs/live_runs/semantic_navigation_context_verify_replay_20260813/
outputs/live_sessions/semantic_navigation_shadow_fail_closed_20260813_03.jsonl
outputs/go2w_acceptance/semantic_navigation_shadow_fail_closed_20260813/result.json
```

### 8.3 旧旋转物理工具证据（不得授权当前硬件）

```text
outputs/go2w_acceptance/rotation_physical_crosscheck_20260813/baseline.json
outputs/go2w_acceptance/rotation_physical_crosscheck_20260813/front_no_target_negative.json
outputs/go2w_acceptance/rotation_physical_crosscheck_20260813/front_probe.json
outputs/go2w_acceptance/rotation_physical_crosscheck_20260813/front.json
outputs/go2w_acceptance/rotation_physical_crosscheck_20260813/front_v2.json
outputs/live_sessions/rotation_lease_invalid_fail_closed_20260813.jsonl
```

这些只验证工具行为和历史诊断，不授权新硬件运动。

### 8.4 Pandar

```text
outputs/go2w_acceptance/hesai_pandarxt16_20260813/result.json
configs/go2w/hesai_pandarxt16.yaml
configs/go2w/hesai_pandarxt16_mount_candidate.yaml
reports/go2w_pandarxt16_mount_candidate_20260813.md
```

---

## 9. 测试与构建基线

```text
SemanticNavigation 初期新增/受影响 unittest: 53 tests OK
SemanticNavigation pytest focus:             25 passed
现场链修复集中回归:              51 passed
关系 crop/tracker 回归:           23 passed
广泛回归（已知基线排除）:        244 passed, 1 deselected
Replan/PSG/situated prior:        28 focused tests passed
计划点名 offline unittest:       63 passed
rotation crosscheck/lease:        21 passed
LiDAR package latest:             26 tests passed
ROS workspace historical latest: 86 tests, 0 failure/error/skip
Hesai 加入后统一 colcon build:    11 packages finished
Pandar live acceptance:          12/12 checks passed
git diff --check:                PASS
```

不能写“最新全目录完全通过”：完整 suite 曾被真实外部 API TLS 握手阻塞，较早也有 README
已知 task examples/count_objects/Streamlit 基线问题。修改核心代码后至少重跑：

```bash
conda run -n go2_robot_scene_demo python -m pytest -q \
  tests/test_semantic_navigation_goal_graph_builder.py \
  tests/test_semantic_navigation_graph_matcher.py \
  tests/test_semantic_navigation_semantic_memory.py \
  tests/test_semantic_navigation_search_reasoner.py \
  tests/test_semantic_navigation_auxiliary_hints.py \
  tests/test_search_directive_adapter.py \
  tests/test_live_semantic_observer.py \
  tests/test_step_search_runner_semantic_reasoning.py \
  tests/test_motion_bounds.py \
  tests/test_rotation_lease.py

bash scripts/go2w/build_ros2.sh
git diff --check
```

Hesai 的 `LossPacket.msg` 源注释可能使某些 `ros2 interface show` 版本报解析错误；运行期
Python message import、订阅和字段读取已通过，不要把 CLI 显示问题误判为无丢包 topic。

---

## 10. 当前改动文件地图

### 10.1 SemanticNavigation / live search 新文件

```text
app/reasoning/semantic_navigation/__init__.py
app/reasoning/semantic_navigation/models.py
app/reasoning/semantic_navigation/goal_graph_builder.py
app/reasoning/semantic_navigation/graph_matcher.py
app/reasoning/semantic_navigation/semantic_memory.py
app/reasoning/semantic_navigation/search_reasoner.py
app/reasoning/semantic_navigation/router.py
app/reasoning/semantic_navigation/auxiliary_hints.py
app/live_robot/semantic_observer.py
app/live_robot/search_directive_adapter.py
scripts/evaluate_live_search_reasoners.py
docs/SEMANTIC_NAVIGATION_SEMANTIC_SEARCH_INTEGRATION.md
```

### 10.2 运动边界和旋转许可新文件

```text
app/live_robot/motion_bounds.py
app/live_robot/rotation_lease.py
ros2_ws/src/go2w_lidar_preprocessor/go2w_lidar_preprocessor/rotation_crosscheck.py
scripts/go2w/validate_rotation_clearance_physical_ros.py
```

### 10.3 Pandar 新文件

```text
configs/go2w/hesai_pandarxt16.yaml
configs/go2w/hesai_pandarxt16_mount_candidate.yaml
scripts/go2w/start_hesai_pandarxt16.sh
scripts/go2w/validate_hesai_pandarxt16_ros.py
reports/go2w_pandarxt16_mount_candidate_20260813.md
ros2_ws/src/hesai_ros_driver/  # nested official Git repo
```

### 10.4 主要 tracked 修改

```text
.env.example
.env.go2w
README.md
app/config.py
app/detectors/siliconflow_vision_worker.py
app/live_robot/frame_bundle_reader.py
app/live_robot/live_search_pipeline.py
app/live_robot/search_state_machine.py
app/live_robot/step_search_runner.py
app/live_robot/ui_status.py
app/llm_clients/siliconflow_client.py
app/memory/observation_memory_store.py
app/ui/go2w_live_panel.py
app/video/frame_analyzer.py
app/video/frame_scene_parser.py
app/video/object_tracker.py
app/video/observed_scene_graph_builder.py
app/video/semantic_verifier.py
app/video/target_profile.py
configs/go2w/lidar_preprocess.yaml
configs/go2w/rgb_lidar_fusion.yaml
configs/go2w/sensor_extrinsics.yaml
docs/GO2W_REAL_ROBOT_DEPLOYMENT.md
reports/go2w_plan_completion_audit.md
run_live_robot_demo.py
scripts/go2w/build_ros2.sh
scripts/go2w/run_autonomous_loop.py
scripts/go2w/start_search_session.sh
scripts/go2w/validate_lidar_preprocessor_ros.py
scripts/go2w/validate_live_frame_bundle.py
scripts/go2w/validate_rgb_lidar_fusion_gate_ros.py
```

此外有多个 ROS package 源码/测试和应用测试被修改，必须以实时 `git status --short` 为准，
不要只按本列表选择提交。`log/` 是 colcon/运行日志，不要因为 untracked 就自动提交或删除。

### 10.5 主要新增测试

```text
tests/test_semantic_navigation_goal_graph_builder.py
tests/test_semantic_navigation_graph_matcher.py
tests/test_semantic_navigation_semantic_memory.py
tests/test_semantic_navigation_search_reasoner.py
tests/test_semantic_navigation_auxiliary_hints.py
tests/test_search_directive_adapter.py
tests/test_live_semantic_observer.py
tests/test_step_search_runner_semantic_reasoning.py
tests/test_evaluate_live_search_reasoners.py
tests/test_motion_bounds.py
tests/test_rotation_lease.py
tests/test_video_semantic_verifier.py
ros2_ws/src/go2w_lidar_preprocessor/test/test_rotation_crosscheck.py
```

---

## 11. 明确未完成项和阻塞关系

### P0：直接阻塞当前运动

- 新雷达支架/电缆没有物理固定和测量证据；
- 新整机扫掠包络未验收；
- 内置 LiDAR sensor-only 旋转可观测性不完整；
- Pandar 尚不是标定后的互补安全传感器；
- 新硬件空场 baseline + 四向实体标定未完成；
- 没有当前 pose-bound rotation lease。

### P1：传感器几何

- camera TF 未测；
- 内置 RGB–LiDAR 外参未通过 moved-position recheck；
- Pandar x/y/yaw 只有候选；
- Pandar PTP/firetime/zero-return/self-occlusion 未正式验收；
- `physical_measurements.yaml` 仍有未测项。

### P2：定位与地图

- wheel radius/track/4WS kinematics 未正式标定；
- Point-LIO translation BLOCKED；USLAM BLOCKED；
- 无 map 和 `map->odom`；
- Nav2 plan_only/execute BLOCKED。

### P3：系统级控制

- 遥控器接管未物理验收；
- `/cmd_vel` leased bridge 未在当前硬件执行；
- Collision Monitor/Velocity Smoother 端到端未验收；
- LiDAR/LIO/lease 丢失后的组合 STOP 场景未全跑。

### P4：SemanticNavigation 真机能力

- Stage 2 active turn-only 未运行成功；
- Stage 3 semantic short-forward 未开始；
- 没有证明 semantic 优于 legacy 的带运动 A/B；
- 当前只有 observe/shadow 安全证据，不是完整自主语义导航。

---

## 12. 下一位 AI 的推荐执行顺序

### Step 1：只读恢复现场

```bash
cd /home/brov/robot/robot_scene_demo
git status --short
git diff --check
ip -4 -brief address show enp6s0
ping -c 2 192.168.123.18
ping -c 2 192.168.123.20
```

读取 ROS 图，确认 Action 为空、rotation gate=false。不要先启动 motion control。

### Step 2：确认硬件没有再次变化

让操作者确认：Pandar 支架位置、电缆是否固定、机器狗是否搬动、站姿是否相同、标定区域
是否改变。任一变化都使旧候选只剩初值意义。

### Step 3：正式化双 LiDAR 多场景标定工具

新增可复现的“只读双云 capture + offline calibration + multi-scene report”，不要继续依赖
一次性内联 Python。输入保留两套原始 frame/stamp；输出应包含：

```text
candidate transform
scene-wise residual
plane-normal diversity / degeneracy
timestamp statistics
inlier ratio
uncertainty
confirmed=false until thresholds pass
```

优先墙角、门框、立柱等非平行结构。

### Step 4：Pandar 独立 diagnostic preprocessor

先发布新命名空间，例如：

```text
/go2w/hesai/points_filtered_diagnostic
/go2w/hesai/status
/go2w/hesai/self_occlusion_debug
```

验证 zero return、range、ring、timestamp、候选 TF、多场景 overlay。仍不得发布
`rotation_clearance_valid=true`。

### Step 5：新外廓和四向物理验收

硬件固定、外参通过后：

1. 实测水平最外点和最高点；
2. 更新候选 footprint/swept radius，但先不授权；
3. 重采空场 baseline；
4. 同一低矮宽目标依次 front/right/rear/left；
5. 检查 pose drift、hash chain、方向、距离、点数增量或中位距离替换；
6. 生成当前硬件唯一的短时 pose-bound lease。

### Step 6：SemanticNavigation Stage 2 active turn-only

仅在 Step 5 全通过后保持：

```text
state_machine_search
active + turn-only
semantic forward disabled
max one successful step
turn <=30 deg
fixed initial half-plane
radius <=1.5 m
fresh LiDAR
valid pose-bound lease
STOP/disarm after step
```

先用目标不存在任务，记录 legacy/semantic directive、arm 时机、Action、STOP、odom 和 lease
过期行为；不得直接跑多步搜索。

### Step 7：Stage 3 short-forward

只有 Stage 2 有明确运动证据后才考虑。semantic forward 即使开启也必须经过：

```text
step_planner -> front clearance -> fixed motion bounds -> odom -> Action -> STOP
```

### Step 8：回到原计划长期缺口

```text
camera physical TF
-> 内置 RGB–LiDAR 多场景正式外参
-> wheel/4WS/LIO translation
-> Level D metric memory
-> map + Nav2 plan_only
-> Collision Monitor/Velocity Smoother/remote takeover
-> 最后才是 Nav2 execute
```

---

## 13. 常用安全命令

只读内置感知：

```bash
cd /home/brov/robot/robot_scene_demo
bash scripts/go2w/start_live_perception.sh
```

Pandar 隔离诊断：

```bash
bash scripts/go2w/start_hesai_pandarxt16.sh
```

Ctrl-C 会正常关闭 socket/parser，不要常规使用 kill -9。

构建：

```bash
bash scripts/go2w/build_ros2.sh
```

停止：

```bash
bash scripts/go2w/stop_all.sh
```

停止脚本只应处理项目拥有的进程，不要批量杀未知 ROS/用户进程。SemanticNavigation observe/shadow
命令以 README 和 `docs/SEMANTIC_NAVIGATION_SEMANTIC_SEARCH_INTEGRATION.md` 为准；任何 active 命令都
必须先确认当前物理门，不能因代码支持就执行。

---

## 14. 下一位 AI 的首轮检查清单

```text
[ ] 已读本交接、README 顶部和两份计划书。
[ ] 确认 HEAD=e861b95 且没有 reset dirty worktree。
[ ] 确认 navigation_gate 未解锁。
[ ] 确认 semantic 默认 disabled/legacy/shadow/no-forward。
[ ] 确认外接雷达 destination 仍为 .99:2368。
[ ] 确认是否运行 hesai driver，且没把 /utlidar/cloud 当 Pandar。
[ ] 确认 mount candidate 的全部 authorizes_* 为 false。
[ ] 确认 rotation_clearance_valid=false。
[ ] 确认无 motion Action/lease/arm 后才做只读工作。
[ ] 没有复用安装前 baseline/front 作为当前运动证据。
[ ] 没有把照片候选发布为正式 TF。
[ ] 没把 Point-LIO yaw PASS 写成 translation PASS。
[ ] 没把 diagnostic overlay 写成 metric 3D PASS。
[ ] 没让 SemanticNavigation strong_match 确认目标。
[ ] 没启用 Nav2 execute。
```

---

## 15. 下一阶段完成定义

最近的正确阶段目标不是“马上让机器狗转动”，而是：

> 在最终固定的 PandarXT-16 硬件状态下，得到可重复的多场景外参、明确的时钟/零回波/
> 自遮挡报告、实测整机 envelope，并重做空场+四向实体检测，使证据能生成短时、位置绑定、
> 只覆盖一次原位转向的许可；随后才执行 SemanticNavigation Stage 2 的一次 turn-only 真机 A/B。

完成前必须保持：

```text
transform_flag=false
authorizes_tf_publication=false
authorizes_safety_integration=false
rotation_clearance_valid=false
semantic forward=false
navigation_gate=fail_closed
motion steps=0
```

---

## 16. 最终交接摘要

本阶段没有把 SemanticNavigation 或新雷达变成绕过安全链的控制器。已完成：

- 可回滚、可 shadow、可审计的 SemanticNavigation 风格 semantic next-view；
- 真实 2D 视觉关系和 evidence gate；
- 内置 LiDAR 误障碍修正与旋转盲区硬门；
- 固定原始半平面/半径和短时物理许可软件；
- PandarXT-16 识别、受控重定向、官方驱动构建、隔离出云、质量验收和候选几何。

仍未完成：

- 新雷达正式外参、时钟、self-filter 与安全融合；
- 新整机物理包络和四向证据；
- 当前硬件下 active turn-only/short-forward；
- camera TF、metric RGB–LiDAR、可靠平移 LIO、地图和 Nav2。

下一位 AI 不能把“代码存在”“照片估计”“诊断点云无回波”或“历史小范围运动通过”升级
成当前硬件的运动许可。
