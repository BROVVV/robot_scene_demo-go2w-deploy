# Go2-W `robot_scene_demo` 交接书（2026-08-13）

> 用途：新聊天窗口接手时，把**原始计划书**
> （`/home/brov/下载/robot_scene_demo_go2w_builtin_rgb_lidar_codex_implementation_plan.md`）
> 与本交接书一起交给新 AI，即可继续工作，避免重复做/漏做。

## 0. 新 AI 接手前必读/必做

1. 读本交接书 → 原始计划书 → `reports/go2w_codex_handoff_20260807.md`
   （8.x 节）→ `reports/go2w_codex_continuation_status_20260806.md`。
2. **不要重复**已完成并留有证据的工作（见第 5 节“不要重复做的事”）。
3. 所有运动行为必须先得到操作者明确授权（当前授权口径：
   `GO2W_MOTION_READY`，小范围半径 ≤1.0 m 活动/转向；场地清空、遥控器可急停）。
4. 仓库已推送到 GitHub，工作树干净；**不要** `git reset --hard`、
   `git checkout -- .`、批量清理。

## 1. 项目与仓库状态

| 项 | 值 |
|---|---|
| 项目根目录 | `/home/brov/robot/robot_scene_demo` |
| 分支 / HEAD | `main` / `dd17e34`（2026-08-13 已推送） |
| Remote | `https://github.com/BROVVV/robot_scene_demo.git` |
| 仓库内容 | 代码/配置/脚本/测试/文档/ROS2 包（214 个文件） |
| 未入库 | `.env`（密钥）、`outputs/` 证据、`point_lio_ws`、`ros2_ws/build|install|log`、模型权重 |

## 2. 现场状态（2026-08-13 检查）

- 机器人 `192.168.123.18` **当前 ping 不通**（可能关机/断网）；各 ROS 栈均未运行。
- 接手第一步：让操作者开机/连网，然后按 README「0.5 启动顺序」恢复：
  感知栈 → 轮式/融合里程计 → 运动控制栈 →（可选）Point-LIO。
- 恢复后必须确认：`/lf/sportmodestate` 的 `mode=1, error_code=0`；
  `/camera/front/image_raw` 约 15–28 Hz；`/go2w/odom/fused` 20 Hz。

## 3. 硬约束（必须始终遵守）

1. 禁止 `/lowcmd`、`LowCmd`、`ReleaseMode()`、`Damp()`、姿态/关节/低层控制；
   禁止改固件或关闭避障、安全保护、急停。
2. 保留用户文件：`data/memory/observational_memory.jsonl`、
   `data/memory/video_spatial_memory.jsonl`（各 10 行预存数据）。
3. 禁止把 API Key/密码/Token 写入代码、日志、报告；`.env` 不入库。
4. Level D/Nav2 保持 fail-closed：`configs/go2w/navigation_gate.yaml` 未改动。
5. 结论必须有真实证据；实验性覆盖需保留 `acceptance_override` 等标记。

## 4. 已完成工作（带证据）

### 4.1 工程与仓库

- Conda `go2_robot_scene_demo`（Py3.11）与 ROS2 Humble 系统 Python 隔离；
- `ros2_ws` 10 个 Go2-W 包构建通过；ROS 包 pytest 17 个通过；
- 主项目 live_robot 相关测试（状态机/规划器/编排器/LLM worker）21+6 个通过；
- README 已重写（含“Go2-W 真机项目当前进度还原指南”），全仓库已推送
  `main@dd17e34`。

### 4.2 传感器链

- 内置 RGB 只读 RPC 桥：`/camera/front/image_raw(+compressed)`、CameraInfo；
  **相机桥已根治**：损坏帧跳过、3 s 读超时、自动重连（退避 1→10 s）；
- 相机内参已标定（9×6、15 mm、105 视角）：
  `configs/go2w/camera_intrinsics.yaml`；
- LiDAR/IMU 时间桥稳定：`configs/go2w/time_sync.yaml`；
- base→LiDAR TF 已修正（z-up、pitch −15.09°）：`official_reference.yaml`；
- LiDAR 预处理/`/scan`/clearance/自过滤：`lidar_preprocess.yaml`；
- 证据：`outputs/go2w_acceptance/camera_calibration_20260806/`、
  `time_bridge_live/`、`lidar_preprocessor_live_corrected_tf/`、
  `lidar_preprocessor_live/`。

### 4.3 运动控制（unitree_go2w_control，独立项目）

- Action `/go2w/motion`（timed velocity / relative yaw）、服务
  `/go2w/arm`、`/go2w/emergency_stop`；三次 STOP + disarm；
- 转向后溜车刹车（`post_turn_rollback_control`）；
- 每次动作前自动重新 arm（解决 LLM 检测超时导致 arm 过期）；
- 自主脚本异常兜底：任何异常先急停再 disarm。

### 4.4 里程计（本仓库）

- 轮式里程计 `/go2w/odom/wheel`（20 Hz）：四轮 dq×0.089 m + Sport yaw；
  转弯跳过平移；前进/后退/90° 转向实测通过；
- **融合里程计 `/go2w/odom/fused`**：轮式平移沿 Sport+LIO 融合航向积分；
  LIO 门禁（新鲜度按主机接收时刻 ≤0.5 s、位置范数 ≤5 m、逐 tick 与 Sport
  一致）；发散自动回退 Sport；诊断 `/go2w/odom/fused/status`；
- `run_autonomous_loop.py` 支持 `--odom-topic /go2w/odom/fused`；
- 证据：`outputs/go2w_acceptance/fusion_validation_20260807/`；
  配置：`configs/go2w/wheel_odom.yaml`。

### 4.5 Point-LIO

- 转向（yaw）：±10° 实测幅度 89%、符号正确（桥接 `yaw_reflect=true`）；
- **平移 BLOCKED**（详见 5.2）；
- 配置：`configs/go2w/point_lio.yaml`、`point_lio_unilidar_l2.yaml`；
  启动：`scripts/go2w/run_point_lio_ros1.sh`（支持 `POINT_LIO_CONFIG` 覆盖）。

### 4.6 LLM 检测线路（用户指定替代 GroundingDINO+SAM2）

- `app/detectors/siliconflow_vision_worker.py`：quick（目标在不在+bbox）与
  verify（框内物体复核）两种模式；像素坐标归一化已修；
- 默认模型 `Qwen/Qwen3-VL-30B-A3B-Instruct`（5–15 s/次），可
  `--llm-model` 切回 8B；
- `run_live_robot_demo.py` 默认 `llm`；
- 录像：中文叠加（Noto CJK）、左下角实时指令角标、复核结论；
- 证据：`outputs/go2w_acceptance/restart_verify_20260807/llm_scan360_graybackpack_07.*`
  （找到→靠近→复核“挂在办公椅上的灰色书包”→`target_reached`）。

### 4.7 自主搜索与状态机

- `run_autonomous_loop.py` 模式：
  `pattern / wander / camera_guided / level_a_search / scan360_approach /
  state_machine_search`；
- scan360 高分命中（≥0.80）提前停止扫描；半径限制 `--max-radius`；
- `app/live_robot`：`search_state_machine.py`（PLAN→MOVE→WAIT→REOBSERVE）、
  `step_planner.py`、`step_search_runner.py`；`--mode state_machine_search`
  真机通过（`llm_sm_graybackpack_01.*`）。

### 4.8 其它

- Bundle 新鲜度门禁：>5 s 且重试 6 次才中止；
- `patches/go2w/point_lio_noetic_pcl115.patch`（PCL 1.15 构建）；
- 交接/报告：`reports/go2w_codex_handoff_20260807.md`、
  `reports/go2w_robot_scene_demo_deployment_report.md` 等。

## 5. 未完成 / 阻塞项

| 项 | 状态 | 原因 / 备注 |
|---|---|---|
| Point-LIO 直线平移 | **BLOCKED** | 0.2 滤波：幅度 94% 但方向偏 +69°、z 上漂；0.4 滤波：幅度只剩 19%；±69° 外参候选发散；干净重启后 20 cm 前进仍发散到 km 级。只可信 yaw |
| USLAM `/uslam/*` | **BLOCKED** | 当前固件未启用（2026-02/06 社区同现） |
| Level D / 地图 / Nav2 | **BLOCKED** | 无可信直线里程计、无地图、TF 不完整；`navigation_gate.yaml` 未改 |
| RGB–LiDAR 外参 | EXPERIMENTAL | 代理误差 33.6 px、门槛放宽到 40 px；**非几何可信**，3D 定位不可用于导航 |
| 相机 TF | BLOCKED | 依赖正式外参 |
| base→LiDAR TF 的 yaw | 未复核 | 当前用桥接 `yaw_reflect` 解决 odom 镜像；TF 层与 `/scan`/clearance 未重新审计 |
| 轮式里程计标定 | 未做 | 轮半径 0.089 m 为标称；轮距/4WS 运动学未标定 |
| 自由探索（`--max-radius 0`） | 已实现未实机 | 受线缆长度限制 |
| 录像时间精确性 | PARTIAL | 检测期间 CPU 争用，回放快于真实时间；未用硬件编码 |
| CUDA / 外部 LLM 8 项测试 | PARTIAL | 无 GPU / 空 Key 时跳过或失败（属既有情况） |

## 6. 下一步（按优先级）

1. **恢复现场**：开机/连网 → 按 README 0.5 启动 → 确认
   `mode=1/error=0` 与各话题频率。
2. **轮式里程计标定**（需要操作者卷尺实测轮径/轮距）：改
   `wheel_odom.yaml` 的 `wheel_radius_m`，补 4WS 转弯运动学；标定后
   `/go2w/odom/fused` 平移更可信，这是 Level D 前置。
3. **相机外参正式标定 + 相机 TF + base→LiDAR TF yaw 复核**：需要
   ≥0.6 m 大平板、5–7 姿态；通过后恢复 5 px 门禁、重验 `/scan`/clearance。
4. **LIO 平移深挖（可选/低收益）**：`filter_size 0.4` 已试（失败）；
   可再试 IMU-input 调参或外部里程计融合；不阻塞 2/3。
5. **建图 + Nav2**：依赖 2/3；先 `plan_only`，再逐级
   （0.3 m → 1.0 m → 绕障 → 目标观察点），所有安全门禁 fail-closed。
6. **收尾改进**：Streamlit 真机控制、录像硬件编码/时间戳、自由探索
   （线缆解决后）。

## 7. 不要重复做的事

- 相机内参标定（9×6/15 mm，已完成）；
- 时间桥 120 s 拟合（`time_bridge_live`）；
- base→LiDAR 方向修正（pitch −15.09°，z-up）；
- LiDAR 预处理/scan/clearance 静止验收；
- ±10° 转向矩阵与 yaw_reflect 结论（5 轮证据）；
- 溜车刹车与 wheel odom 前进/后退/90° 验证；
- 融合里程计实现与 `fusion_trial_01` 验证；
- LLM quick/verify worker、录像中文叠加、指令角标；
- scan360/level_a/state_machine_search 真机闭环演示；
- 相机桥逐帧容错+自动重连修复；
- README 重写与全仓库推送（main@dd17e34）。

## 8. 常用命令速查

```bash
# 感知栈
cd /home/brov/robot/robot_scene_demo
bash scripts/go2w/start_live_perception.sh

# 轮式 + 融合里程计
source /opt/ros/humble/setup.bash
source /home/brov/robot/unitree_ros2/cyclonedds_ws/install/setup.bash
source ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/configs/go2w/cyclonedds_go2w.xml"
ros2 launch go2w_lio_bringup wheel_odom.launch.py

# 运动控制（unitree_go2w_control）
cd /home/brov/robot/unitree_go2w_control
source scripts/setup_go2w_ros2.sh
ros2 launch go2w_motion_control go2w_motion_control.launch.py

# 自主搜索（LLM 默认）
/usr/bin/python3 scripts/go2w/run_autonomous_loop.py \
  --mode scan360_approach --target "灰色书包" --detector llm \
  --llm-model Qwen/Qwen3-VL-30B-A3B-Instruct \
  --target-score-min 0.45 --max-radius 1.0 --max-seconds 420 \
  --reach-area-ratio 0.08 --odom-topic /go2w/odom/fused \
  --record-video outputs/go2w_acceptance/demo.mp4 \
  --output outputs/go2w_acceptance/demo.jsonl
```

详细步骤与故障排查见 README「Go2-W 真机项目当前进度还原指南」。

## 9. 关键文件索引

```text
configs/go2w/camera_intrinsics.yaml
configs/go2w/official_reference.yaml
configs/go2w/time_sync.yaml
configs/go2w/lidar_preprocess.yaml
configs/go2w/wheel_odom.yaml
configs/go2w/point_lio.yaml
configs/go2w/point_lio_unilidar_l2.yaml
configs/go2w/navigation_gate.yaml        # fail-closed，未改
scripts/go2w/run_autonomous_loop.py
scripts/go2w/start_live_perception.sh
scripts/go2w/run_point_lio_ros1.sh
app/detectors/siliconflow_vision_worker.py
app/live_robot/search_state_machine.py
app/live_robot/step_planner.py
app/live_robot/step_search_runner.py
ros2_ws/src/go2w_lio_bringup/go2w_lio_bringup/wheel_odom.py
ros2_ws/src/go2w_camera_bridge/go2w_camera_bridge/camera_bridge_node.py
reports/go2w_codex_handoff_20260807.md
reports/go2w_codex_continuation_status_20260806.md
```

## 10. 交接完成判据

- 已按第 0 节读完交接书/计划书/历史报告；
- 已恢复现场并确认 `mode=1/error=0`、话题健康；
- 未重做第 7 节任何已通过项；
- 从第 6 节优先级开始推进，每项结论都有新证据；
- 未越过安全边界；Level D/Nav2 保持 fail-closed。
