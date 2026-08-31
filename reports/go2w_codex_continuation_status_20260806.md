# Go2-W robot_scene_demo 计划执行状态交接（2026-08-06）

> 用途：新窗口/新 AI 继续工作时以此文档为唯一进度快照，避免漏做、重复做。
> 原始计划书：`/home/brov/下载/robot_scene_demo_go2w_builtin_rgb_lidar_codex_implementation_plan.md`
> 上一份交接：`reports/go2w_ai_continuation_handoff_20260806.md`
> 项目根目录：`/home/brov/robot/robot_scene_demo`

---

## 0. 最高约束（必须始终遵守）

1. **机器人不得移动**：禁止 Sport/lease/Move/StopMove/MotionCommand、`/cmd_vel`、Nav2 execute、姿态/关节/低层控制。也不得为了“安全”擅自发 StopMove。
2. 保留所有未提交修改和 dirty 工作树；不得 `git reset --hard`、批量清理。
3. 不得覆盖 `data/memory/observational_memory.jsonl` 和 `video_spatial_memory.jsonl`（各有用户预存 10 行）。
4. 不得把 API Key、密码、凭据写入代码/日志/报告。
5. 所有“已确认”状态必须有真实证据；当前外参/融合是**用户授权的实验性覆盖**，不是正式标定。

---

## 1. 当前配置真值（2026-08-06 晚）

| 配置 | 状态 | 说明 |
|---|---|---|
| `configs/go2w/camera_intrinsics.yaml` | `calibrated` | 9×6 内角点、15 mm 方格、105 视角，不要重做 |
| `configs/go2w/time_sync.yaml` | `stable: true` | 120 s 拟合，云 RMSE 0.593 ms |
| `configs/go2w/lidar_preprocess.yaml` | `validated/confirmed` | 静止实测通过；但**依赖的官方 base→LiDAR 翻转有疑点，需复核** |
| `configs/go2w/point_lio.yaml` | `enabled: true`, `stationary_read_only_validated` | 仅静止五分钟通过；运动未验证 |
| `configs/go2w/sensor_extrinsics.yaml` | `calibrated/confirmed: true`（实验性） | 用户授权降低门槛 5→40 px；代理误差 33.6 px；**不是几何可信标定**；`acceptance_override` 有记录 |
| `configs/go2w/rgb_lidar_fusion.yaml` | `enabled: true`, `validated`（实验性） | 同步窗 50→3000 ms（相机比云新约 2.33 s）；消费 `/go2w/sensors/cloud` 原始 z-up 帧 |
| `configs/go2w/navigation_gate.yaml` | `level_d_passed: false`, `map_valid: false`, `tf_valid: false`, `nav2_allow_execute: false` | 导航仍 fail-closed |

---

## 2. 计划 14 个阶段逐项状态

| 阶段 | 状态 | 已完成 | 未做/阻塞 |
|---|---|---|---|
| 0 审计与基线 | PASS | Git 基线、补丁、用户预存文件保护 | 无 |
| 1 环境与回归 | PASS/PARTIAL | Conda 3.11 与 ROS 3.10 隔离、smoke/mock/构建通过 | CUDA 不可用；8 个依赖外部 LLM/Key 的测试空 Key 未过 |
| 2 内置 RGB 桥 | 桥/内参 PASS，TF BLOCKED | 只读 RPC 图像桥、Image/Compressed/CameraInfo、真实内参 | 相机相对 base_link 的可信 TF |
| 3 时间诊断/桥 | PASS | 云/IMU 对时稳定 | 相机↔云实时同步存在约 2.33 s 偏差（融合已用 3 s 窗凑合） |
| 4 URDF/TF/几何 | PARTIAL | 官方尺寸、LiDAR/LiDAR-IMU TF 已固定 | 相机 TF 未验收；**官方 base→LiDAR pitch 165° 与本机发布帧疑似不匹配，需复核**；base_footprint 高度未验收 |
| 5 LiDAR 预处理 | 静止 PASS | 自身/地面过滤、720-bin scan、clearance、超时保护 | 运动状态未验证；若官方翻转被修正，过滤参数需重验 |
| 6 LIO | 静止 PASS / 运动 BLOCKED | 官方 point_lio 五分钟静止通过；rko_lio 被拒 | 直线/矩形/360°/回环/重启试验全部禁止 |
| 7 RGB–LiDAR 融合 | 软件 PASS / 实机实验性打开 | 融合核心、同步、PnP、overlay 门控测试通过；端到端 3D 定位已跑通 | 外参未几何确认；3D 定位有 ~0.3 m 距离偏差，**不可用于导航** |
| 8 实时数据桥 | PASS | 原子 Bundle、READY-last、1 Hz、30 Bundle 上限 | 无 |
| 9 实时搜索 | PARTIAL | 真机 Bundle 入口、证据门控、基线场景 | 完整 2D 检测链（GroundingDINO/SAM2）实机未验证；相机 TF 门控未开 |
| 10 搜索状态机 | 软件 PASS / 运动 DISABLED | observe-only 安全路径 | PLAN_STEP/MOVE/STOP 实机循环禁止 |
| 11 安全配置 | 软件 PASS | 严格默认值、传感器超时、执行默认禁用 | 运动安全参数无实机授权 |
| 12 仲裁与 bridge | 核心 PASS / 执行 BLOCKED | 仲裁优先级、零速 fail-safe、300 ms watchdog | 不创建 Action client；接管/lease/STOP 实机链未验收 |
| 13 地图与 Nav2 | 软件配置 / 实机 BLOCKED | planner-only/execute 门控、保守参数 | 无运动 LIO、无地图、TF 不完整；execute 禁止 |
| 14 Streamlit | 软件 PASS / 端到端 PARTIAL | 状态面板、blocker 显示、控制禁用 | 无完整 Level A/3D/导航实机演示 |

---

## 3. 本轮（2026-08-06）新增完成的工作

### 3.1 新箱体验证与目标准备（多轮）

- 用户先后尝试：纸贴箱侧（低）、纸贴箱顶、纸贴箱侧（垫高）、纸贴箱侧（更高）→ 相机/LiDAR 视场始终对不上；最终通过“箱子垫高 + 纸贴侧立 + 灯光调整”解决。
- 期间给用户提供了浏览器实时相机预览 + 棋盘叠加（检测角点/中心/法向，辅助摆位）。
- 关键结论：**本机 L2 发布点云仰角只有约 -1.15°~+89.6°（上视锥）**，官方 L2 规格为 360°×90°（水平面以上半球）+ 负角模式 96°；L2 装在头部下巴（低于相机），避障靠多传感器配合。

### 3.2 7 个静止姿态采集

| 姿态 | bag | 相机 PnP | LiDAR 箱侧平面 |
|---|---|---|---|
| 正面中距 | `extrinsic_repos_box_front_01` | 0.27 px | 0.74×0.53 m，RMS 1.3 cm |
| 正面近距 | `extrinsic_repos_box_near_01` | 0.39 px | RMS 1.4 cm |
| 正面远距 | `extrinsic_repos_box_far_01` | 0.18 px | 较稀疏，RMS 1.3 cm |
| yaw 左 30° | `extrinsic_repos_box_yaw_left_01` | 0.34 px | RMS 1.3 cm |
| yaw 右 30° | `extrinsic_repos_box_yaw_right_01` | 0.45 px | RMS 1.3 cm（该姿态平面对应性最差） |
| 后仰 38° | `extrinsic_repos_box_tilt_back_01` | bag 图像光照失败 | 平面可提取 |
| 前倾 22° | `extrinsic_repos_box_tilt_forward_01` | 0.80 px | ROI 多面混淆，选面不稳 |

### 3.3 平面法外参候选与实验性接入

- 5 姿态（正/近/远/左右 yaw）RANSAC 箱侧平面 + 相机棋盘平面联合求解，得到候选 R/t：
  - 平面距离残差 0.1–2.5 cm；棋盘角点到平面 0.5–5.5 cm；
  - 投影到纸面角点最近点 25–40 px（部分受 L2 点间距影响）。
- **退化原因**：所有箱侧平面法向几乎平行（都朝相机），绕观察轴旋转和平行于平面的平移约束不足；加入倾斜姿态因数据质量差反而更糟。
- 用户授权：门槛 5→40 px，标记 `acceptance_override`，`moved_position_recheck` 为操作者覆盖（物理复检仍禁止）。
- 代码/配置改动：
  - `fusion_node.py` 新增 `cloud_topic` 参数，实机改用 `/go2w/sensors/cloud`（原始 z-up）；
  - `rgb_lidar_fusion.yaml` 同步窗 50→3000 ms；
  - `start_live_perception.sh` 传入 `cloud_topic:=/go2w/sensors/cloud`；
  - 融合测试更新为“带 override 开门 / 去掉 override 关门”，13 个测试通过；`validate_rgb_lidar_overlay.sh` PASS。
- 端到端实机验证：临时棋盘检测器喂 2D 检测 → 融合输出 3D 定位（相机坐标约 0.16, -0.12, 1.35 m；与 PnP 距离偏差约 0.3 m）。

### 3.4 重大发现（必须让下一个 AI 知道）

1. **L2 发布帧是 z-up（已水平化）**：`/utlidar/imu` 静止加速度约 (2.36, 0.04, 9.82)，点云 z 为正；官方 `base_link -> utlidar_lidar` 的 pitch 2.8782 rad（约 165°）若直接叠加会把点云翻到世界朝下。此前“地面平面审计”很可能是翻转后对上表面的误拟合。
2. **官方 TF 的争议未解决**：可能正确的是“发布帧已 z-up，官方翻转只适用于原始 SDK 帧”。需要 1–2 分钟只读复核（相机画面 + IMU + 已知地面/箱子）后再决定是否修正 `official_reference.yaml`。当前**未修改**该文件。
3. **融合时间不同步**：相机图像比 `/go2w/sensors/cloud` 新约 2.33 s（云话题缓冲），静止场景用 3 s 窗可工作；运动场景必须重新解决。
4. **旧六姿态外参失败分析需要重看**：当时在官方翻转帧上做，若翻转错误则结论无效。

---

## 4. 未做 / 阻塞清单

| 项 | 状态 | 阻塞原因 |
|---|---|---|
| 相机 TF（camera→base_link） | BLOCKED | 外参未几何确认 + 官方 base→LiDAR 翻转疑点 |
| base→LiDAR 官方 TF 复核/修正 | PENDING | 需要只读物理验证；修正会影响预处理/过滤参数 |
| 外参正式几何确认 | PENDING | 需要大型平板或更可靠的倾斜姿态 |
| 完整 2D 检测链（GroundingDINO/SAM2）实机 | PENDING | 未接入/未验证 |
| 运动 LIO（直线/矩形/360°/回环） | BLOCKED | 用户禁止移动 |
| 小地图与重定位 | BLOCKED | 无运动 LIO |
| Nav2 plan_only/execute 实机 | BLOCKED | 无地图/TF/运动授权 |
| 搜索状态机 PLAN_STEP/MOVE/STOP | BLOCKED | 运动禁止 |
| 移动机器人复检外参 | BLOCKED | 运动禁止 |
| CUDA 加速与外部 LLM 8 项测试 | PARTIAL | 无 GPU / 空 Key |

---

## 5. 下一步建议顺序（保持机器人静止）

1. **复核并修正 base→LiDAR 官方 TF**（只读）：确认发布帧是否已 z-up，若是则把 `official_reference.yaml` 的方向改成与发布帧一致，并重验 `lidar_preprocess.yaml` 过滤参数。这一步不做，后面所有 base_link 相关结论都不可信。
2. **决定外参路线**：
   - A（推荐）：准备 ≥0.5–0.6 m 大型平板，棋盘贴中心，重新 5–7 姿态采集 → 正式平面法求解 → 恢复到 5 px 门禁验收。
   - B（凑和）：保留当前实验性候选，仅用于静止观察，绝不用于导航。
3. 外参可信后：推导/静态验证相机 TF。
4. 用真实 2D 检测链（GroundingDINO/SAM2）替换临时棋盘检测器，复跑融合端到端。
5. 静止 observe-only 搜索 + Streamlit 3D 展示。
6. 之后必须停下等运动授权：Level D 运动 LIO → 小地图 → Nav2 plan_only → 安全链/接管/急停 → 极短距离分级测试。

---

## 6. 关键证据索引

- 交接报告：`reports/go2w_ai_continuation_handoff_20260806.md`
- 部署报告：`reports/go2w_robot_scene_demo_deployment_report.md`
- 外参采集报告：`reports/go2w_rgb_lidar_extrinsic_capture_report.md`
- 外参候选：`outputs/calibration/plane_extrinsic_candidate/candidate.yaml`
- 实验验收报告：`outputs/calibration/plane_extrinsic_candidate/overlay_validation_report.json`
- 融合端到端结果：`outputs/calibration/plane_extrinsic_candidate/fusion_e2e_live_result.json`
- 新箱体观测检查：`outputs/calibration/extrinsic_*_front_01/observability_check.json`
- 7 个姿态 bag 与 v2 数据集：`outputs/calibration/extrinsic_repos_box_*`
- 相机内参：`outputs/go2w_acceptance/camera_calibration_20260806/`
- 时间桥：`outputs/go2w_acceptance/time_bridge_live/result.json`
- LIO 静止五分钟：`outputs/go2w_acceptance/point_lio_stationary/result_5min.json`
- 十分钟 soak：`outputs/go2w_acceptance/level_a_stationary_soak_fixed/result.json`

---

## 7. 交接完成判据

- 以第 2 节阶段表定位未完成项；
- 不要重做：相机内参、时间桥、静止 LIO、十分钟 soak、已通过的软件测试；
- 不越过运动边界；
- 任何 capability 提升必须有真实证据；实验性 override 必须保留 `acceptance_override` 标记；
- 新窗口继续时先读本文档 + 上一份交接报告 + 计划书，再读部署/外参报告。

---

## 8. 2026-08-06 续作更新（本会话完成，TF 修正 + 检测链 + 运动准备）

### 8.1 base→LiDAR 官方 TF 已修正（重大，影响后续所有几何）

- 本机 `/utlidar/cloud` 与 `/utlidar/imu` 发布帧已经是 **z-up**：
  - 实时 IMU 静止比力约 `(3.08, 0.005, 9.81) m/s²`（+z 朝天、+x 上仰约 17°）；
  - 点云仰角范围 `-1.14..+89.6°`，z 范围 `-0.122..1.126 m`。
- 官方 URDF `radar_joint` 的 pitch `2.8782 rad (165°)` 是针对传感器 z-down 原生安装帧的转换；直接叠加在已 z-up 的发布帧上会把点云翻到世界朝下。
- 修正后 ROS TF：`base_link -> utlidar_lidar` 平移不变 `(0.28945, 0, -0.046825)`，rpy 改为 `(0, -0.263392653559, 0)`（= 2.8782 - π），即 -15.09°。
- 已修改：`configs/go2w/official_reference.yaml`（含 `accepted_basis` 与实机证据字段）；三个包测试同步更新（description/lidar_preprocessor/lio 全部 PASS）。
- 旧 `lidar_stationary_geometry`（“地面” z=-0.42）已标记 superseded：那是翻转后对上表面的误拟合，不是地板。

### 8.2 LiDAR 预处理修正后实机复验 PASS

- `configs/go2w/lidar_preprocess.yaml`：地面阈值改为 URDF 腿链推导值（base 地面约 -0.528 m，`ground_separation_height_m=-0.448`，高度窗 `-0.588..0.972`）。
- 新增自过滤区域：头部/相机（x 0.28..0.58, |y|<=0.31, z 0.30..0.82）与机身两侧（|y| 0.255..0.37）。
- 实机复验（20 帧）：z_min=-0.448、z_max=0.972、scan 720-bin、frame=base_link、自过滤 0 点、front clearance 0.583 m / left 0.370 / right 0.404。
- 证据：`outputs/go2w_acceptance/lidar_preprocessor_live_corrected_tf/result.json`。
- `validate_lidar_preprocessor_ros.py` 的地面阈值已改为读取配置，不再硬编码旧值。

### 8.3 Point-LIO 修正 TF 后静止回归 PASS

- 60 s 静止：924 odom、15.38 Hz、最大间隔 67 ms、末位移 0.059 m、最大位移 0.098 m、yaw span 2.38°、stale 0.22 s。
- 证据：`outputs/go2w_acceptance/point_lio_stationary_corrected_tf/result.json`、`stale_timeout.json`。
- `point_lio.yaml` 已记录新旧证据；运动 LIO 仍待首次小范围运动试验。

### 8.4 实时 2D 检测链真机跑通（Level A 观察搜索可运行）

- GroundingDINO（SwinT OGC）+ SAM2 tiny 在 CPU 上对实时 Bundle 图像推理成功（约 28 s/帧，含模型加载）。
- `run_live_robot_demo.py --target 手机 --detector grounded_sam --search-mode observe_only` 端到端通过：目标画像 fallback、track 连续（2 帧同一 bbox）、evidence gate 正确拒绝低分候选（`target_not_seen`）。
- 新增 `--disable-llm-profile` / `--disable-crop-verify`，Streamlit 面板默认关闭外部 LLM 调用；避免未授权触发付费 API。
- 当前 UI 显示：camera/lidar/extrinsics/fusion PASS，lio/tf BLOCKED（真实状态）。

### 8.5 运动准备（用户已授权小范围活动与转向）

- 控制项目 dry-run 12/12 PASS（timed goal、cancel、lease loss、state stale、emergency stop、relative yaw）。
- 首次实际运动仍需操作者确认：场地清空、遥控器在手；脚本要求 `I_HAVE_CLEARED_THE_AREA`。
- 建议首次试验：`vx=0.05 m/s × 2 s`（约 10 cm）+ `±10°` 低速转向，同时记录 LIO odom；之后再做矩形/回环。

### 8.6 2026-08-07 实际运动试验结果（机器人重启后）

- 机器人重启后：网络/感知栈恢复（新会话 `live_20260807_094848`），运动模式 `ai-w`、`mode=1`、`error=0`。
- 已执行（操作者授权 `GO2W_MOTION_READY`）：
  1. 前进 `vx=0.05 m/s × 2 s`：Action success、error=0、轮编码器确认运动（峰值 dq 0.64 rad/s、约 3.15 s），但 **Point-LIO（纯 LiDAR 模式）odom 没有跟踪**（位姿仍在静止漂移量级）；
  2. 左转 +10°（实测 10.83°）：IMU 输入模式开始跟踪（yaw 变化约 -8.2°，**符号与 Action 相反**），随后位姿开始发散；
  3. 右转 -10°（实测 -9.87°）：**Point-LIO 位姿爆炸**（记录结束时 x≈16368 m、z≈-4271 m，机器人实际静止）。
- 结论：**Point-LIO 运动里程计两种模式均失败**（纯 LiDAR 冻结、IMU 模式发散），Level D 继续 BLOCKED，Nav2/step-search 保持 fail-closed。
- 每次动作后机器人都回到静止：`mode=1`、`error=0`、速度为零；最后执行 `emergency_stop` 三次 STOP 成功，运动栈干净退出。
- 证据：`outputs/go2w_acceptance/point_lio_motion_small/motion_trial_summary.json` 与两个 `odom_log.jsonl`。
- 下一步建议：先修 Point-LIO 的 IMU 坐标系/重力对齐与输出 yaw 符号，或改用机器人自带 USLAM/轮式编码器里程计（`/utlidar/robot_odom`、`/uslam/*` 当前未发布，需要进一步排查）；在此之前不得开启任何依赖 odom 的模式。

### 8.7 当前状态速查

| 项 | 状态 |
|---|---|
| base→LiDAR TF | 已修正并实机验证（-15.09°，z-up） |
| LiDAR 预处理 | 修正后实机复验 PASS |
| Point-LIO 静止 | 修正后回归 PASS |
| Point-LIO 运动 | **FAIL**（纯 LiDAR 冻结；IMU 模式发散），Level D 仍 BLOCKED |
| 实时 2D 检测链 | 真机 Bundle PASS（CPU） |
| 观察搜索 | observe-only 端到端 PASS（fail-closed 正确） |
| 3D 融合 | 仍为实验性 override，不可用于导航 |
| 运动执行 | 小范围前进/±10° 转向已实机执行并安全停止；运动栈已退出 |

### 8.8 2026-08-07 Point-LIO IMU 对齐排查与 USLAM 未开启结论

#### 8.8.1 Point-LIO IMU 对齐排查（只读，未运动）

**1. 初始化（重力对齐）经离线复算和日志反推是自洽的，不是 yaw 反号的原因。**

- 实机静止 IMU 均值：`linear_acceleration ≈ (2.84, 0.00, 9.86) m/s²`，
  `angular_velocity ≈ 0`（250 Hz 正常，重启后话题健康）。
- `Set_init()`（`point_lio_ws/src/point_lio_unilidar/src/IMU_Processing.hpp:157`）
  中 `gravity_=[0,0,-9.81]` 与 `tmp_gravity=-mean_acc≈(-2.84,0,-9.86)` 夹角约 16°，
  **不进入共线退化分支**，rot_init 为绕 y 轴 −16.05° 的旋转：
  `rot_init euler ≈ (0.015°, -16.05°, -0.002°)`。
- 用运动日志第一条 odom 反推（先撤掉 bridge 的 base→imu 外参）得到的原始
  Point-LIO 初始姿态为 `(0.57°, -15.75°, -1.15°)`，与 Set_init 预测一致。
  因此**之前“gravity 与 body z 共线导致 rot=-I（180°翻转）”的假设被否证**；
  把 `gravity` 改成 `[0,0,+9.81]` 反而会让 align_cos≈-0.96、初值接近 180° 翻转并
  把世界重力方向搞反，**不要采用该修改**。
- 附注：静止比力模长约 10.26 m/s²（比 9.81 高约 4.6%），`acc_norm: 9.81` 未做归一，
  L2 加速度计存在可观的尺度偏差，但这只影响尺度/漂移，不解释 yaw 反号。

**2. yaw 反号发生在 Point-LIO 内部，bridge 转换不是原因。**

- 左转 +10.83°（Action 反馈）：日志 odom 的 base 帧 yaw 变化约 −8.9°；
  撤掉外参后的 IMU 帧 yaw 变化约 −8.2°，符号相同。
- 右转 −9.87°：yaw 变化先 +4° 左右后位姿爆炸到公里级（机器人实际静止）。
- 即左右转的 yaw 响应都反向，且与机器人自身 Sport yaw 反馈（+10.83°）符号相反。

**3. 最可能的根因：Go2-W 固件发布的 `/utlidar/imu` 陀螺仪 z 轴符号与
  加速度计/点云 z-up 帧不一致（或 LIO 得到的陀螺仪约定相反）。**

- L2 官方 SDK 仅说明“点云坐标系 L 与 IMU 坐标系 I 轴向平行、只差平移”，
  但 Go2-W 的 `/utlidar/*` 是机器人裸 DDS 固件直接发布，不是本机 SDK 包；
  固件版本 `lidar_state.software_version=1.0.0.38`。
- `lidar_state.imu_rpy` 在两次会话间从 roll +43° 跳到 −76°，内部姿态估计
  明显不可信，不能用来验证，反而支持“固件 IMU 数据链存在约定问题”的怀疑。
- 唯一能一锤定音的办法：再做一次 ±10° 低速转向，同时录制 `/utlidar/imu`
  原始角速度，并由操作者目视确认物理转向方向：
  - 若物理左转时 `ωz<0` → 固件陀螺仪 z 反号，应在 LIO 桥接输入处对 `ωz`
    取负（这是坐标约定修正，不是掩盖）；
  - 若物理左转时 `ωz>0` → IMU 符号正常，则问题在 Point-LIO/外参/场景，需要
    继续查（例如先解决纯 LiDAR 模式“前进 10 cm 完全不跟踪”的注册强度问题）。

**4. 纯 LiDAR 模式冻结是独立问题：**

- 前进 10 cm 时轮编码器确认运动，但 odom 停留在静止漂移量级。说明该场景下
  LiDAR 约束对平移太弱（L2 只有上视锥、特征少/对称），IMU 又是“输入”侧，
  一旦 yaw 约定出错就无法靠 LiDAR 拉回，随后发散。

#### 8.8.2 USLAM 为什么没开（结论：固件侧功能，当前固件未启用/未开放）

**现场证据（2026-08-07 重启后，只读）：**

- `/utlidar/robot_odom`、`/utlidar/robot_pose`：话题存在（机器人固件发布者，
  GID `f6.a4…`，与 cloud/imu 同一参与者），但监听 12–72 s **零消息**。
- `/utlidar/cloud_deskewed`、`/utlidar/voxel_map*`、`/utlidar/grid_map`、
  `/uslam/localization/odom`、`/uslam/frontend/odom`、`/uslam/navigation/global_path`：
  全部零消息；`/uslam/server_log` 只有订阅者没有发布者。
- `/uslam/client_command`：有发布者（GID `8b.01…`，机器人另一个管理侧参与者，
  它同时发布 `/utlidar/switch`）但**没有订阅者**——说明请求 SLAM 的“客户端”活着，
  而真正的 USLAM 服务端没有运行。
- `unitree_sdk2_python` 里**没有任何 uslam/robot_odom/mapping 客户端代码**，
  只有 `go2_utlidar_switch.py`（`rt/utlidar/switch` 电源开关）。
- 健康基线正常：`/utlidar/cloud` 15.5 Hz、`/utlidar/imu` 250 Hz、
  `/utlidar/lidar_state` 5 Hz（`software_version=1.0.0.38`）。

**社区/官方线索：**

- MYBOTSHOP（代理 Unitree 支持）2026-02 明确答复：**Unitree 仍在更新 Go2-W
  固件，当前版本禁用了下巴 LiDAR 的 odom/SLAM 输出**，只能用 VSLAM 或外接
  雷达，等后续固件恢复
  （forum.mybotshop.de/t/unitree-go2w-no-functions-in-app-problems-with-ros-launch-files/1604）。
- 2026-05/06 两位 Go2-W 用户复现相同现象：只有 `/utlidar/cloud` 与
  `/utlidar/imu` 有数据，其余 LiDAR 话题为空
  （forum.mybotshop.de/t/unitree-go2-w-lidar-in-chin-orientation/1815；
  forum.mybotshop.de/t/unitree-go2-w-ros2-odometry-and-topic-availability-issues/1831）。

**结论：USLAM 是机器人固件/机内服务，当前固件未启用；主机侧没有任何 SDK 或
命令能把它“打开”。** 因此 `/uslam/*`、`/utlidar/robot_odom` 维持 BLOCKED，
不参与任何导航/执行门控。可行路线按优先级：
1. 向 Unitree 确认当前固件版本是否已恢复或何时恢复 USLAM（或更新固件后复检）；
2. 继续修 Point-LIO（先做一次带原始 IMU 录制的 ±10° 验证转向，确定 yaw 符号）；
3. 备选：外接雷达/VSLAM 作为独立里程计。

### 8.9 2026-08-07 转向验证矩阵（已执行 5 轮 ±10°，全部安全静止）

本轮按授权做了 5 轮 ±10° 转向，每次都录制 `/utlidar/imu`、`/lf/sportmodestate`、
`/lf/lowstate` 和 `/lio/odom`，且每次动作后回到 `mode=1/error=0`、三次 STOP 成功。

#### 原始 IMU 关键测量（决定性证据）

同一物理旋转，两路 IMU 的 z 轴角速度符号完全相反：

| 指令（Sport 反馈） | L2 `/utlidar/imu` ωz | 机载 `/lf/lowstate` gyro z |
|---|---:|---:|
| +10°（Sport +10.53°） | **−0.093 rad/s** | **+0.095 rad/s** |
| −10°（Sport −9.65°） | **+0.052 rad/s** | **−0.060 rad/s** |

#### LIO 各配置转向结果（均 0.15 rad/s）

| 配置 | +10° Sport 指令 LIO yaw | −10° 指令 LIO yaw | 静止后稳定性 |
|---|---:|---:|---|
| imu-input、0.4 m、无修正（原方案） | −8.2°（幅 76%） | +4.1° 后爆炸 | 发散 |
| imu-input、0.4 m、gyro (1,1,−1) | +2.15°（幅 21%） | −5.67° | 发散 |
| imu-input、0.4 m、gyro (1,−1,−1) | +1.58° | −8.93° | 发散 |
| lidar-only、0.2 m、gyro (1,−1,−1) | −8.98°（幅 88%） | +8.14° | **稳定（cm 级）** |
| lidar-only、0.2 m、gyro (1,−1,−1)、外参 Rz(π) | −9.87°（幅 94%） | +9.21° | **稳定** |
| lidar-only、0.2 m、**官方默认无修正** | **−8.14°（幅 87%）** | **+8.16°** | **稳定（10 s 漂移 2 cm/yaw 0.3°）** |

结论：

- **当前最佳可用配置 = 官方默认（外参单位阵、陀螺仪不修正）+ `use_imu_as_input=false`
  + `filter_size_surf/map=0.2`**：LIO 转向幅度约 87–94%，静止稳定。
- LIO 的 yaw 与 Sport/Action 反馈**恒为反号**，且与 L2 原始 ωz 同号（稳定模式下
  LiDAR 约束占主导，陀螺仪修正与否都不改变 yaw 符号）。这强烈说明：
  **L2 IMU/点云是标准 z-up 右手系且彼此一致；反号的是 Go2-W 机载 IMU/Sport 的
  yaw 约定**（项目里 `yaw_command_sign` 的 ±1 肉眼标定从未完成，08-05 审计也标注
  “操作者肉眼方向确认待补录”）。
- 尚缺唯一地面真值：**+10° 指令时机器人物理上向左还是向右转**。请操作者目视确认：
  - 若 +10° = 物理右转（俯视顺时针）→ LIO 是物理正确的，只需把运动侧
    `yaw_command_sign` 改为 −1 并复测，Point-LIO 无需任何修正；
  - 若 +10° = 物理左转（俯视逆时针）→ LIO 世界系是 yaw 镜像，需要在输出侧做
    y 反射约定（odom + 地图一起反射），再做复测。

证据目录：`outputs/go2w_acceptance/imu_turn_verify_20260807/`
（`imu_turn_log.jsonl`、`imu_turn_log_corrected.jsonl`、`imu_turn_log_rx180.jsonl`、
`imu_turn_log_lidaronly_dense.jsonl`、`imu_turn_log_extrinsic_yawflip.jsonl`、
`imu_turn_log_clean_official.jsonl`）。

当前现场：机器人静止（mode=1/error=0/已 disarm），感知栈与 Point-LIO 仍在运行
（`/lio/odom` 15.4 Hz），等待目视确认后再定 yaw 约定。

### 8.10 2026-08-07 最终修复：LIO yaw 约定 + 转向后溜车（已实机验证）

#### 目视确认结果

操作者确认：**+10° 指令 = 物理左转（俯视逆时针）**。因此 LIO 稳定解报告的
−8°~−9° 是 yaw 镜像（LIO 世界系相对 base 是镜像的），需要输出侧约定修正；
机载 IMU/Sport 的 yaw 约定本身是对的。

#### 修复 1：Point-LIO 桥接输出 y 反射（`yaw_reflect=true`）

- `point_lio_bridge.py` 新增 `yaw_reflect` 参数：在 `imu_pose_to_base_pose()`
  输出处把世界系做 X-Z 平面反射（位置 y→−y、姿态 yaw→−yaw），并把反射后的
  旋转矩阵转回四元数；配置 `point_lio.yaml` 的 `imu_frame.yaw_reflect=true`，
  `config_gate` 校验布尔值，测试新增 1 个（共 10 个测试通过）。
- 实机验证（2026-08-07 11:07，纯 LiDAR + 0.2 m 滤波 + 官方外参 + 不修正陀螺仪）：

| 指令（物理） | Sport 反馈 | 修正后 LIO yaw | 位置漂移 |
|---|---:|---:|---:|
| +10°（物理左转） | +11.01° | **+9.80°（89%）** | cm 级 |
| −10°（物理右转） | −9.92° | **−8.82°（89%）** | cm 级 |

- yaw 符号正确、幅度约 89%，静止稳定。

#### 修复 2：转向后溜车（动作服务器轮速主动刹车）

根因：转向到位后机器人回到 mode=1，轮子失去高控制锁定，StopMove 释放后自由
溜动约 2–4 cm（实测轮 q 持续减小 0.27 rad、dq −0.6~−0.9 rad/s 约 3–4 s）。

修复：`go2w_motion_action_server` 在最终 STOP 前新增
`post_turn_rollback_control` 阶段（最多 3 s，20 Hz）：

- 读 `lowstate` 四轮 dq 均值；
- 命令 `vx = −gain × mean_dq × wheel_radius`（轮子向后溜就向前刹），
  限幅 0.10 m/s；
- 轮速稳定低于 0.03 rad/s 连续 8 次后退出，再做原有零速保持 1 s，最后 STOP。

新参数（`motion_control.yaml`）：`post_turn_rollback_control_sec=3.0`、
`post_turn_rollback_deadband_radps=0.03`、`post_turn_rollback_gain=0.8`、
`post_turn_rollback_max_vx=0.10`、`post_turn_rollback_stable_samples=8`、
`wheel_radius_m=0.089`；结果日志记录 peak dq/vx 与完成标志。

实机验证（同一轮 ±10° 试验）：

- 两个 goal 的 `post_turn_rollback_completed=true`，peak dq 2.11/1.03 rad/s，
  峰值刹车指令 0.10/0.073 m/s；
- 转向动作结束后轮 q 变化从原来的 −0.27 rad（约 2.4 cm，持续 3–4 s）降到
  **约 −0.05 rad（约 0.5 cm，8 s 缓慢蠕行，无突然后溜）**；转向后约 1.5 s
  内轮速归零并保持。

#### 未完成项

- 直线/矩形运动尚未用新配置验证（原先纯 LiDAR 前进 10 cm 不跟踪的问题待复测）；
- base→LiDAR TF 的 180° yaw 问题仍未在 TF 层处理（本次用输出反射解决 odom，
  不影响 `/scan`/clearance 的既有验收，但后续建图/导航前应复核传感器 TF）；
- Level D/Nav2 保持 BLOCKED（`navigation_gate.yaml` 未改动）。

### 8.11 2026-08-07 直线复测结论 + 轮式里程计兜底（已实机验证）

#### 直线复测（新配置：纯 LiDAR/IMU 输入 + 0.2 m 滤波 + yaw_reflect）

- `vx=0.05 m/s × 2 s`：机器人本身就没按指令走——轮编码器只转了约 0.06 rad
  （≈0.5 cm），说明 ai-w 低速存在死区（对比 08-07 早前同指令实际也只走了约
  2.5 cm）。
- `vx=0.12 m/s × 2 s`：轮子实际走了约 14–18 cm（前进 1.55–1.97 rad），
  **Point-LIO 平移幅度能对上（约 13–14 cm），但方向相对自身航向系统性偏
  ~55–60°，且 z 上漂 5 cm**。纯 LiDAR 与 IMU 输入两种模式一致。
- 结论：LIO 的旋转（yaw）可用（±10° 转向 89%），**直线平移在此场景不可信**
  （疑似 LiDAR↔IMU 外参旋转或上视锥 ICP 系统性偏差，需要专门标定/深挖）。
  按“多次尝试不达标先降指标”原则，直线 LIO 暂定 BLOCKED，启用轮式兜底。

#### 轮式里程计兜底（新增，已实机验证）

新增 `go2w_wheel_odom` 节点（`ros2_ws/src/go2w_lio_bringup`）：

- 输入：`/lf/lowstate` 四轮编码器（indices 12–15，20 Hz）+ `/lf/sportmodestate`
  yaw（与转向控制器同源，已目视验证左转约定）；
- 输出：`/go2w/odom/wheel`（`odom_wheel -> base_link`），20 Hz；
- 模型：每帧轮 q 增量均值 × 轮半径（0.089 m 标称）沿当前 unwrapped yaw 积分；
  原地转向时四轮 dq 均值≈0，位置基本不动，yaw 跟随 Sport；
- 参数/配置：`configs/go2w/wheel_odom.yaml`、`launch/wheel_odom.launch.py`；
- 单测：`test_wheel_odom.py`（unwrap、直线、原地转向），共 15 个测试通过。

实机往返验证（0.12 m/s × 2 s 前进 + 后退 + 10° 左转）：

| 阶段 | 轮式里程计 | 编码器推算 |
|---|---:|---:|
| 前进 0.12×2 s | +17.7 cm | +17.5 cm |
| 后退 0.12×2 s | −18.7 cm | −18.6 cm |
| +10° 左转 | yaw +11.07°（Sport +11.05°） | 转向带 12.5 cm 前移补偿 |

- 末端状态 (0.112, 0.005) m、yaw +11.05°，与“转向补偿导致净前移约 11 cm”
  的物理预期一致。
- 限制（文档已注明）：轮半径/运动学未标定、转弯平移按“沿当前航向前进”
  近似（无侧滑模型）、Sport yaw 静止漂移约 −0.019°/min。仅用于小范围演示，
  长距离导航前必须标定。

#### 当前可运行能力

- 小范围运动链可用：Action（timed/yaw）+ 溜车刹车 + 轮式里程计 `/go2w/odom/wheel`
  + LIO yaw/转向（`/lio/odom` 修正后）+ 感知/融合栈。
- Level D/Nav2 仍 BLOCKED（无可信直线 odom 与地图）；轮式里程计为实验性兜底，
  `navigation_gate.yaml` 未改。

### 8.12 2026-08-07 自主运行里程碑：自动规划循环 + 反应式漫游（已实机）

新增 `scripts/go2w/run_autonomous_loop.py`：机器人自己 arm → 执行步骤序列 →
每步用 `/go2w/odom/wheel` 和净空校验 → 自动急停 + disarm，全程无人干预。

**1. 固定规划循环（pattern 模式）实机通过（9 步）**

`f, l20, f, r20, f, l20, f, r20, f`：5 次前进（10–19 cm）+ 4 次 ±20° 转向
（实际 19.4–20.3°），全部动作成功、步进校验通过、前向净空全程 ~0.58 m，
结束后自动 STOP/disarm。证据：`autonomous_loop_01.jsonl`。

**2. 反应式自主漫游（wander 模式）实机通过（90 秒 × 2 轮）**

- 决策：前向净空 > 0.45 m → 前进 2 s；否则朝左/右净空大的一侧转 30°。
- 遇到**低处障碍/墙**（L2 只看得见上方，前向净空可能显示 0.5 m+ 但轮子被挡）：
  前进步骤轮式里程计净位移 < 3 cm → 自动重试 1 次 → 仍无位移 → 自动转 90°
  绕开，不中止漫游。
- 两轮漫游分别跑满 90 秒（一次 57 秒因早期版本无绕障逻辑而中止；加入绕障后
  跑满），共处理 4 次“前进被挡”，均自动转向后继续；结束时自动急停+解除臂，
  `mode=1/error=0`。
- 证据：`autonomous_wander_01/02/03/04.jsonl`。

**3. 轮式里程计转向修正（实机验证）**

- 发现 4WS 转向时四轮 dq 均值≠0，90° 转向被误算为 1.87 m 前进；新增
  `turn_yaw_rate_threshold_radps=0.10` + `skip_translation_during_turns`：
  转向期间（|Sport yaw rate|>0.10）不积分位移，只更新航向。
- 单次 90° 转向复测：位移从 1.87 m 降到 **0.23 m**，yaw +90.6°（Action 反馈
  +90.59°）。

**4. 安全结论**

- “无位移即中止/绕障”机制有效，弥补了 L2 上视锥看不见低处障碍的盲区；
- 自主运行仍只限小范围、操作者在场（`GO2W_MOTION_READY` 授权）；Level D 地图
  与 Nav2 仍 BLOCKED，轮式里程计仍是实验性兜底（轮半径/运动学未标定）。

### 8.13 2026-08-07 相机引导自主接近（Level A 核心闭环，已实机）

把相机 2D 检测接进自主漫游，新增 `run_autonomous_loop.py --mode camera_guided`：

- 每轮：读最新 frame bundle 图像 → GroundingDINO（禁用 SAM2，CPU 约 10 s）
  检测目标 → 取最高分 bbox：
  - 目标在画面中心 ±0.08 内 → 前进 2 s 靠近；
  - 否则按中心偏移比例转 ±25° 以内（`l/rN`）；
  - bbox 面积占比 ≥ 0.15 → 判定“已到达”停止；
  - 未检测到 → 左右交替转 30° 继续搜索。
- 全部步骤继续复用轮式里程计校验、无位移重试、净空与 mode/error 安全检查，
  结束自动急停+解除臂。

实机验证（目标“手机”，180 秒，11 次检测全部命中）：

| 检测 | cx | 动作 |
|---|---:|---|
| 1 | 0.433 | 前进 22 cm |
| 2 | 0.913（偏右） | 右转 19.4° |
| 3 | 0.726 | 右转 10.8° |
| 4 | 0.585 | 右转 3.4°（居中） |
| 5–9 | 0.537–0.564 | 连续前进 13–16 cm/步 |
| 10 | 0.584 | 右转 3.4° 微调 |

- 手机从 cx 0.43 → 稳定居中 0.54–0.58；bbox 面积占比 0.0042 → 0.011（持续靠近）；
  置信度 0.20 → 0.62。180 秒到时自动停止，未到 0.15 的“到达”阈值（手机仍较远，
  每步前进受 ai-w 低速死区限制只有 13–16 cm）。
- 证据：`outputs/go2w_acceptance/imu_turn_verify_20260807/camera_guided_01.jsonl`
  与动作服务器 `logs/20260807_121418/`。
- 提速参数（下次可调）：`--forward-vx 0.18 --forward-seconds 3`、
  `--reach-area-ratio 0.08`、`--max-seconds 300`。

### 8.14 2026-08-07 Level A 搜索状态机闭环（搜索→发现→靠近，已实机）

`run_autonomous_loop.py --mode level_a_search` 实现完整 Level A 行为：

- **SEARCH**：目标不在画面时按“摆动扫描”走：右 30°×3 → 左 30°×3 → 小前进 →
  左 30°×3 → 右 30°×3 → 小前进（净航向归零、线缆不持续扭转），每步之间检测；
- **DISCOVER**：GroundingDINO 命中目标（score ≥ 0.15）即切换；
- **APPROACH**：目标偏左/右按偏移比例转 ±25°，居中后前进 2 s 靠近；bbox 面积
  占比 ≥ 0.15 判定“已到达”；
- **RANGE_LIMIT**：以本轮起点为圆心，轮式里程计半径超过 `--max-radius`（本次
  实验 1.0 m）立即停止——**这是本次实验的小范围约束，后面自由探索时把
  `--max-radius 0` 即可去掉限制**；
- 全部步骤复用轮式里程计校验、无位移重试、净空/mode/error 检查，结束自动
  急停+解除臂。

实机验证（目标“手机”，半径 1.0 m，240 秒上限）：

- 9 次 DISCOVER 全部命中手机：自动左转 5° 对齐 → 前进 → 反复检测/微调
  （r24/r10/r4/l20/r14），轮式里程计半径从 0 推进到 0.84 m；
- 到达 1.01 m 时触发 **RANGE_LIMIT**，机器人自动急停并解除臂（mode=1/error=0），
  全程未超出实验半径。
- 证据：`outputs/go2w_acceptance/imu_turn_verify_20260807/level_a_search_01.jsonl`
  与动作服务器 `logs/20260807_122450/`。
- 说明：本次手机一开始就在画面内，SEARCH 扫描段没有实际触发。要演示“搜索看
  不到的目标”，把手机放到相机视野外（或先让机器人转 90°）再跑即可看到
  r30×3/l30×3 扫描过程。

### 8.15 2026-08-07 灰色书包搜索演示（已实机，接近成功）

- 新增目标词条：`书包/灰色书包 → backpack. schoolbag. gray backpack. rucksack`。
- 实机 300 秒上限、半径 1.0 m（`--target 灰色书包`）：

| 检测 | 书包 cx | 面积 | 半径 |
|---|---:|---:|---:|
| 1 | 0.10（画面最左） | 0.042 | 0.00 |
| 2 | 0.33 | 0.057 | 0.06 |
| 3–5 | 0.48–0.50（居中） | 0.057 | 0.03–0.34 |
| 6–9 | 0.57–0.83 | 0.065–0.075 | 0.51–1.00 |

- 机器人自动把书包从画面左侧对齐到中心（score 0.41–0.55），持续靠近到
  1.00 m，随后触发实验半径限制自动急停（mode=1/error=0）。
- 说明：书包放置位置仍在相机边缘视野内（cx=0.10），因此一开始就被发现，
  SEARCH 扫描段仍未触发。要完整演示“转头找到看不见的目标”，需把书包放到
  **相机视野完全之外**（约正前方向左/右 70° 以上，FOV 105°），可用
  `--scan-span 4` 让扫描覆盖 ±120°。证据：
  `outputs/go2w_acceptance/imu_turn_verify_20260807/level_a_search_backpack_01.jsonl`。

### 8.16 2026-08-07 全程录像 + 目标锁定框演示（灰色书包，已实机）

- `run_autonomous_loop.py` 新增视频录制：独立 ROS 节点 + 独立 executor 线程订阅
  `/camera/front/image_raw`（bgr8），实时写 MP4（默认 768×432 @15 fps）；
  检测命中后初始化 **CSRT 追踪器**，在两次检测之间持续更新绿框，并叠加
  `label score LOCK`；未命中时画面显示 “searching...”。
  参数：`--record-video <path>`、`--video-fps`、`--video-scale`。
- 演示流程：书包放正前方 → `--pre-scan-turns 3` 让机器人先盲转右 90°
  （书包被甩出视野）→ 主循环扫描找回 → 锁定靠近 → 半径 1.0 m 自动停止。
- 实机结果（`--target 灰色书包 --scan-span 4 --max-radius 1.0 --max-seconds 300`）：
  - 盲转后第一次发现书包 cx=0.13（画面左缘），随后对齐到 cx≈0.46，score
    0.50–0.62，面积 0.026→0.051，半径推进到 0.96 m 触发范围限制；
  - 视频 968 帧：前 410 帧 “searching”，发现后 **558 帧绿框+置信度持续锁定**
    （schoolbag 0.58→0.62，`backpack_search_locked.jsonl` 佐证）；
  - 录像约 65 s 回放（实际过程约 2 分钟，检测子进程占 CPU 时录制帧率降低，
    帧本身覆盖全程）。
- 证据：
  `outputs/go2w_acceptance/imu_turn_verify_20260807/backpack_search_locked.mp4`、
  `backpack_search_locked.jsonl`、`level_a_search_backpack_video_01.jsonl`。

### 8.17 2026-08-07 重测：无盲转直接搜索（灰色书包 v2）

- 用户指出 8.16 锁定的是裤子（误检）；本轮收紧词条
  （去掉 schoolbag/backpack，只留 `gray backpack. grey backpack. rucksack`）、
  置信度门槛 0.15→0.35、**不加盲转**（用户已预先转动机器人使书包不在视野）。
- 实机 300 秒、半径 1.0 m、录像 1179 帧：
  - 第 1–2 次检测即命中 “gray backpack”（cx≈0.48 居中、面积 0.075→0.094、
    score 0.40–0.41），机器人直接靠近；
  - 第 3 次起检测漂移（cx 0.17 面积 0.009、cx 0.92 面积 0.029），score
    0.37–0.49，机器人反复转向纠正，半径到 1.07 m 触发范围限制停止；
  - 录像锁定 1110/1179 帧（94%），标签 “gray backpack grey backpack”，
    score 0.367–0.412。
- **待用户目视确认**：锁定的到底是真书包还是再次误检（裤子/其他灰色物体）。
  若仍误检，下一步加判别手段（SAM2 掩码面积/颜色直方图校验、或目标放明显
  标记物）。
- 证据：`backpack_search_v2.mp4`、`backpack_search_v2.jsonl`、
  `level_a_search_backpack_video_02.jsonl`。

### 8.18 2026-08-07 360° 扫描选优 + 前往目标（灰色书包，已实机，成功到达）

- 录像编码修复：原 mp4v（MPEG-4 Part 2）部分播放器打不开，改为 H.264
  （avc1，回退 mp4v），已用 cv2 验证可正常打开。
- 新增 `--mode scan360_approach`：先原地右转 360°（默认 8×45°），每个朝向
  检测一次并记录候选（label/score/bbox/heading）；扫描完成后选**全部图像中
  置信度最高**的候选，转回该方向，再进入“对齐→靠近→到达/半径限制”闭环。
- 实机（`--target 灰色书包 --target-score-min 0.35 --max-radius 1.0
  --max-seconds 420`，录像 `backpack_scan360.mp4`）：
  - 360° 扫描 8 个朝向共收集 12 个候选（score 0.36–0.51），最佳
    **rucksack 0.508 @ −89°（右侧）**；
  - 机器人右转 138° 对准该方向，靠近时再次检测 “gray backpack” score 0.425、
    cx=0.486（居中）、**面积 0.222 ≥ 0.15 → TARGET_REACHED，自动停止**；
  - 全程录像 2104 帧 @15fps（H.264，可正常打开），锁定 1898/2104 帧（90%），
    标签/置信度持续叠加（sidecar `backpack_scan360.jsonl`）。
- 证据：`backpack_scan360.mp4`、`backpack_scan360.jsonl`、
  `scan360_backpack_01.jsonl`。
