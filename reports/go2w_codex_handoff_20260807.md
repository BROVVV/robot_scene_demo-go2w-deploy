# Go2-W `robot_scene_demo` 交接书（2026-08-07 晚）

> 用途：新聊天窗口接手时，把**原始计划书**（`/home/brov/下载/robot_scene_demo_go2w_builtin_rgb_lidar_codex_implementation_plan.md`）与本交接书一起提供给新 AI，即可继续工作，避免重复做/漏做。
> 项目根目录：`/home/brov/robot/robot_scene_demo`（计划书里的 `/root/...` 是旧候选路径，不要使用）。
> 更细的历史记录见 `reports/go2w_codex_continuation_status_20260806.md`（8.x 节）和 `reports/go2w_robot_scene_demo_deployment_report.md`。

## 0. 新 AI 接手前必读/必做

1. 读本交接书 → 原始计划书 → `reports/go2w_codex_continuation_status_20260806.md`（重点 8.6–8.18 节）→ `reports/go2w_robot_scene_demo_deployment_report.md`。
2. 确认现场：`/lf/sportmodestate` 应为 `mode=1 / error_code=0`；`ps aux` 检查感知栈、LIO、轮式里程计、控制栈进程（现状见第 1 节）。
3. **不要重复**已完成并留有证据的验证（相机内参标定、时间同步、静止 LIO、±10° 转向矩阵、360° 扫描选优、溜车修复等；证据清单见第 5 节）。
4. 所有运动行为必须先在对话里明确授权范围（当前授权：`GO2W_MOTION_READY` 小范围；本次实验半径限制 1.0 m，以后自由探索用 `--max-radius 0`）。

## 1. 现场状态快照（2026-08-07 ~13:10）

| 项 | 状态 |
|---|---|
| 机器人 | 静止，`mode=1`、`error_code=0`，已 disarm，控制栈已退出（lease 已释放） |
| 感知栈 | 运行中（`start_live_perception.sh`，会话 `live_20260807_094848`；camera/lidar/融合/Bundle 正常） |
| Point-LIO ROS1 侧 | **未运行**（roscore/pointlio_mapping/endpoint 已停） |
| Point-LIO ROS2 桥 | 运行中（`point_lio_bridge`，会自动重连 ROS1 侧） |
| 轮式里程计 | 运行中（`go2w_wheel_odom`，`/go2w/odom/wheel` 20 Hz） |
| 网络 | 主机 `192.168.123.99` ↔ 机器人 `192.168.123.18`（enp6s0） |
| 机器人当前坐标 | 轮式里程计系约 `(-6.2, 1.3)`（相对 11:33 原点）；上次 360° 演示停在目标前约 0.33 m |

## 2. 硬约束（必须始终遵守）

1. 机器人运动仅限操作者授权的小范围活动/转向（`GO2W_MOTION_READY`）；本次实验建议半径 ≤1.0 m；运动前确认场地清空、遥控器可急停。
2. 禁止 `/lowcmd`、`LowCmd`、`ReleaseMode()`、`Damp()`、姿态/关节/低层控制；禁止改固件或关闭安全保护。
3. 保留 dirty 工作树；禁止 `git reset --hard`、`git checkout -- .`、批量清理。
4. 禁止覆盖 `data/memory/observational_memory.jsonl`、`data/memory/video_spatial_memory.jsonl`（各有用户预存 10 行）。
5. 禁止把 API Key/密码写入代码、日志、报告；`.env` 已有 Grounded-SAM-2 相关配置（无 API Key 也可跑演示）。
6. 所有能力结论 fail-closed：Level D/Nav2 仍 BLOCKED；`navigation_gate.yaml` 未改动。

## 3. 已完成工作

### 3.1 基础工程（2026-08-06 及之前）

- 相机只读 RPC 桥 + 真机内参标定（9×6、15 mm，已完成勿重做）。
- LiDAR/IMU 时间同步（120 s 拟合，云 RMSE 0.59 ms）。
- 官方 URDF/尺寸导入、`base→LiDAR` z-up 修正（pitch −15.09°）、LiDAR→IMU 外参。
- LiDAR 预处理、`/scan`、clearance、超时保护（静止实测通过）。
- 官方 Point-LIO 静止五分钟通过；RGB–LiDAR 融合（实验性 override，代理误差 33.6 px、门槛 40 px）；原子 Bundle 传输；软件级搜索状态机/仲裁/Nav2 门控/Streamlit。
- 详情见 `reports/go2w_codex_continuation_status_20260806.md` 第 1–7 节（接手者应读）。

### 3.2 Point-LIO IMU 对齐排查与 yaw 修复（2026-08-07）

- 重力初始化离线复算：`Set_init` 非退化，`rot_init ≈ (0.02°, -16.05°, 0°)`；**`gravity: [0,0,9.81]` 的修改方向被否证**，不要改。
- 5 轮 ±10° 转向矩阵（全部带原始 IMU 录制，证据 `imu_turn_log*.jsonl`）；操作者目视确认 **+10° 指令 = 物理左转**。
- 结论：稳定 LIO 世界系相对 base 是 **yaw 镜像** → 在桥接输出做 `yaw_reflect=true`（世界系 X-Z 反射：`y→-y`、`yaw→-yaw`）。
- **当前最佳转向配置**：`use_imu_as_input=false`（lidar-only）+ `POINT_LIO_FILTER_SIZE_SURF/MAP=0.2` + 官方单位阵外参 + **不修正陀螺仪**（`gyro_sign=[1,1,1]`）+ 桥接 `yaw_reflect=true`；转向幅度约 89–94%，静止稳定。
- **直线 LIO 仍 BLOCKED**：0.12 m/s 前进 14–18 cm 时，LIO 位移幅度能对上但方向系统性偏 55–60°、z 上漂约 5 cm（纯 LiDAR 与 IMU 输入模式一致；疑似 LiDAR↔IMU 外参旋转或上视锥 ICP 系统性偏差，需专门标定）。

### 3.3 运动控制增强（unitree_go2w_control）

- **溜车修复**：动作服务器新增 `post_turn_rollback_control`（按四轮 dq 反向前进刹停，限幅 0.10 m/s）+ `post_turn_zero_velocity_hold`；参数在 `ros2_ws/src/go2w_motion_control/config/motion_control.yaml`。实测转向结束后轮 q 变化从 −0.27 rad（约 2.4 cm/3–4 s）降到约 −0.05 rad（0.5 cm 级慢蠕行）。
- 发现 **ai-w 低速死区**：`vx=0.05` 基本不动，`vx=0.12` 实际约 14–18 cm/2 s；后续演示统一用 0.12。

### 3.4 轮式里程计兜底（新增）

- `go2w_wheel_odom` 节点：四轮编码器增量均值 × 0.089 m，沿 Sport yaw（unwrapped）积分；**转向期间（|yaw rate|>0.10 rad/s）不积分位移**（4WS 转向均值模型失真，90° 误算 1.87 m → 修正后 0.23 m）。
- 输出 `/go2w/odom/wheel`（20 Hz，`odom_wheel → base_link`）；配置 `configs/go2w/wheel_odom.yaml`。
- 验证：前进/后退与编码器一致（17.7/18.7 cm）；90° 转向 yaw +90.6°（Sport +90.59°）。
- 限制：轮半径 0.089 m 为标称、4WS 运动学未标定，仅小范围可信。

### 3.5 自主运行（`scripts/go2w/run_autonomous_loop.py`）

全部模式：自动 arm → 每步用 `/go2w/odom/wheel` + 净空 + `mode/error` 校验（无净位移自动重试/绕障）→ 结束自动急停 + disarm。

| 模式 | 行为 | 实机证据 |
|---|---|---|
| `pattern` | 固定步骤序列（前进/±20°） | `autonomous_loop_01.jsonl` |
| `wander` | 净空反应式漫游（前向>0.45 前进，否则朝净空大侧转 30°；被挡转 90° 绕行） | `autonomous_wander_01..04.jsonl` |
| `camera_guided` | GroundingDINO 检测目标→居中→靠近 | `camera_guided_01.jsonl` |
| `level_a_search` | SEARCH（±90°/±120° 摆动扫描，线缆不持续扭转）→DISCOVER→APPROACH→RANGE_LIMIT（`--max-radius`，0=无限） | `level_a_search_*.jsonl` |
| `scan360_approach` | 原地右转 360°（默认 8×45°）每个朝向检测，**选全部图像中置信度最高者**，转回该方向靠近 | `scan360_backpack_01.jsonl` |

- 目标词条：`PROMPT_MAP`（手机/箱子/瓶子/杯子/书/人/书包/灰色书包）；灰色书包词条已收紧为 `gray backpack. grey backpack. rucksack`（去掉了易误中裤子的 schoolbag/backpack）。
- 检测：GroundingDINO（禁 SAM2，CPU 约 10 s/帧），经 conda env `go2_robot_scene_demo` 子进程调用，配置读取 `.env`（模型在 `/home/brov/robot/Grounded-SAM-2`）。

### 3.6 视频录制与目标锁定（新增）

- `BundleVideoRecorder`（脚本内类）：独立 ROS 节点 + 独立 executor 线程订阅 `/camera/front/image_raw`（bgr8）；**H.264（avc1）MP4**（旧 mp4v 文件部分播放器打不开，已弃用）；检测命中后用 CSRT 追踪器持续锁定绿框 + `label score LOCK`。
- 参数：`--record-video <path>`、`--video-fps`（默认 15）、`--video-scale`（默认 0.4 → 768×432）。
- 实机验证：`backpack_scan360.mp4`（2104 帧 @15fps，可正常打开；锁定 1898/2104 帧）。

### 3.7 USLAM 结论

- `/utlidar/robot_odom`、`/utlidar/robot_pose`、`/uslam/*` 重启后仍全部零消息；`unitree_sdk2_python` 无 USLAM 客户端；社区/Unitree 支持确认 **当前 Go2-W 固件禁用了下巴 LiDAR 的 odom/SLAM 输出**（2026-02，2026-06 其他用户同现）。
- 主机侧无法开启，维持 BLOCKED。备选：等固件更新 / 外接雷达或 VSLAM / 继续用本项目轮式+LIO 方案。

## 4. 未完成 / 阻塞项

| 项 | 状态 | 原因 / 备注 |
|---|---|---|
| 直线 LIO | **BLOCKED** | 方向偏 55–60°、z 上漂；疑似外参旋转或 ICP 系统性偏差，需专门标定/深挖 |
| USLAM | **BLOCKED** | 固件未启用 |
| Level D / Nav2（地图、定位、execute） | **BLOCKED** | 无直线 odom、无地图、TF 不完整 |
| 相机外参 | 实验性 override | 代理误差 33.6 px、门槛 5→40 px 用户授权；需大型平板正式标定 |
| base→LiDAR TF 的 yaw | 未复核 | 当前用桥接输出 `yaw_reflect` 解决 odom 镜像；TF 层与 `/scan`/clearance 未重新审计 |
| 轮式里程计标定 | 未做 | 轮半径/轮距/4WS 运动学未标定 |
| 目标误检判别 | 弱 | 灰色书包曾误检成裤子；已用词条+阈值缓解，未做 SAM2 掩码/颜色校验 |
| 实时 2D 检测链 | PARTIAL | GroundingDINO 约 10 s/帧（CPU）；SAM2 掩码、crop verify 未接入运动循环 |
| 录像时间精确性 | PARTIAL | 检测期间 CPU 争用导致回放快于真实时间；未用硬件编码 |
| 自由探索（无限半径） | 已实现未实机 | `--max-radius 0`；受线缆长度限制未跑 |
| app 层搜索状态机整合 | PARTIAL | 行为在脚本中实现；`app/live_robot/search_state_machine.py` 仍 observe-only |
| 矩形/回环/连续运动验收 | PARTIAL | 单步前进/转向已验证；完整矩形/回环未做 |

## 5. 下一步建议（按优先级）

1. **用户确认** `backpack_scan360.mp4` 锁定的是真书包（若仍误检 → 加判别层：SAM2 掩码面积 / 灰色颜色直方图 / 目标贴标记物）。
2. **轮式里程计标定**：实测轮半径（当前 0.089 标称）、轮距/4WS 转向模型，让 `/go2w/odom/wheel` 在转向时更准（当前转向位移近似）。
3. **修 LIO 直线方向偏差**：验证 lidar↔IMU 外参旋转假设（可做一次已知直线运动的离线 ICP/外参估计），或考虑 wheel odom 与 LIO 融合。
4. **自由探索验证**：线缆允许后跑 `--mode level_a_search --max-radius 0`（或 wander）。
5. **相机外参正式标定 + base→LiDAR TF 复核**：大型平板 5–7 姿态；复核后重验 `/scan`/clearance。
6. **完整 Level A**：把 `run_autonomous_loop.py` 的行为接入 `app/live_robot` 搜索状态机、SAM2 校验、安全门禁，并固化为 ROS launch。
7. **建图 + Nav2**：依赖 2–6（线性 odom、地图、TF）。
8. **录像改进**：硬件编码/时间戳精确回放/自动上传。

## 6. 关键文件索引

### 配置

- `configs/go2w/point_lio.yaml`：桥接门禁；`imu_frame.gyro_sign=[1,1,1]`、`yaw_reflect=true`；`motion_validation.status=turns_validated_linear_blocked_20260807`
- `configs/go2w/point_lio_unilidar_l2.yaml`：官方单位阵外参（勿改回实验值）
- `configs/go2w/wheel_odom.yaml`：轮式里程计配置
- `configs/go2w/navigation_gate.yaml`：未改动，保持 fail-closed
- `configs/go2w/official_reference.yaml`：base→LiDAR z-up 修正（TF 层 yaw 未复核）
- `/home/brov/robot/unitree_go2w_control/ros2_ws/src/go2w_motion_control/config/motion_control.yaml`：溜车刹车参数、`yaw_command_sign=1`

### 代码

- `scripts/go2w/run_autonomous_loop.py`：自主运行器（pattern/wander/camera_guided/level_a_search/scan360_approach + 视频录制）
- `scripts/go2w/record_imu_turn.py`：原始 IMU/轮/odom 录制器
- `scripts/go2w/run_point_lio_ros1.sh`：支持 `POINT_LIO_USE_IMU_AS_INPUT`、`POINT_LIO_FILTER_SIZE_SURF/MAP` 环境变量
- `ros2_ws/src/go2w_lio_bringup/`：`point_lio_bridge.py`（gyro_sign/yaw_reflect）、`wheel_odom.py`、`config_gate.py`、`launch/`（point_lio、wheel_odom）
- `/home/brov/robot/unitree_go2w_control/ros2_ws/src/go2w_motion_control/src/motion_action_server.cpp`：溜车刹车、转向/直线 Action
- `app/detectors/grounded_sam_worker.py` / `grounded_sam_subprocess.py`：GroundingDINO 检测（conda env `go2_robot_scene_demo`）

### 报告与证据

- `reports/go2w_codex_continuation_status_20260806.md`（8.6–8.18 为本轮）
- `reports/go2w_robot_scene_demo_deployment_report.md`
- `reports/go2w_ai_continuation_handoff_20260806.md`（上一份交接）
- 证据目录：`outputs/go2w_acceptance/imu_turn_verify_20260807/`
  - `imu_turn_log*.jsonl`（5 轮转向矩阵）、`autonomous_*.jsonl`、`camera_guided_01.jsonl`、`level_a_search_*.jsonl`、`scan360_backpack_01.jsonl`
  - `backpack_scan360.mp4/.jsonl`（H.264，可播放；锁定 1898/2104 帧）
  - `backpack_search_locked.mp4`、`backpack_search_v2.mp4`（旧 mp4v，兼容性差）

## 7. 常用启动与演示命令

```bash
# 0) 统一注意：ROS 脚本用 /usr/bin/python3（系统默认 python3 是 conda 3.14，会报 rclpy 错）

# 1) 感知栈（若未运行）
cd /home/brov/robot/robot_scene_demo
scripts/go2w/start_live_perception.sh

# 2) Point-LIO（最佳转向配置：lidar-only + 0.2 滤波）
POINT_LIO_OUTPUT_DIR=outputs/go2w_acceptance/xxx \
POINT_LIO_USE_IMU_AS_INPUT=false \
POINT_LIO_FILTER_SIZE_SURF=0.2 POINT_LIO_FILTER_SIZE_MAP=0.2 \
scripts/go2w/run_point_lio_ros1.sh
# 另开终端启动桥：
source /opt/ros/humble/setup.bash
source /home/brov/robot/unitree_ros2/cyclonedds_ws/install/setup.bash
source ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$PWD/configs/go2w/cyclonedds_go2w.xml"
ros2 launch go2w_lio_bringup point_lio.launch.py \
  lio_config:=configs/go2w/point_lio.yaml \
  reference_config:=configs/go2w/official_reference.yaml \
  time_config:=configs/go2w/time_sync.yaml

# 3) 轮式里程计
ros2 launch go2w_lio_bringup wheel_odom.launch.py

# 4) 运动控制栈（需申请 lease）
cd /home/brov/robot/unitree_go2w_control
source scripts/setup_go2w_ros2.sh
ros2 launch go2w_motion_control go2w_motion_control.launch.py

# 5) 自主演示示例（360° 扫描选优 → 前往目标，带 H.264 录像）
cd /home/brov/robot/robot_scene_demo
source /opt/ros/humble/setup.bash
source /home/brov/robot/unitree_ros2/cyclonedds_ws/install/setup.bash
source /home/brov/robot/unitree_go2w_control/ros2_ws/install/setup.bash
source ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
/usr/bin/python3 scripts/go2w/run_autonomous_loop.py \
  --mode scan360_approach --target 灰色书包 \
  --target-score-min 0.35 --max-radius 1.0 --max-seconds 420 \
  --record-video outputs/.../demo.mp4 --output outputs/.../demo.jsonl
```

## 8. 已知坑与注意事项

- **lease 3207**：若有旧 `go2w_motion_action_server`/`hold_sport_lease` 残留，新控制栈申请 lease 会报 3207；先 `ps aux | grep -E "go2w_motion_action|hold_sport_lease"` 并 kill 后重启。
- **L2 低处盲区**：LiDAR 上视锥看不到轮子高度障碍（前向净空可能 0.5 m+ 但轮子被挡）；靠“命令成功但净位移 <3 cm → 重试 → 90° 绕障”兜底，不要只信 clearance。
- **ai-w 低速死区**：`vx=0.05` 基本不动，演示用 ≥0.12；实际位移约命令值的 60–75%。
- **线缆**：原地 360°（scan360 模式）会扭转线缆；本次实验半径限制 1.0 m；自由探索前先解决线缆。
- **视频编码**：新录像为 H.264（avc1）；`backpack_search_locked.mp4`/`backpack_search_v2.mp4` 为旧 mp4v，部分播放器打不开。
- **检测 CPU 争用**：GroundingDINO 子进程（~10 s）期间录像帧率下降，回放比真实时间快。
- **机器人重启后**：程序可能识别不到（用户提示），先确认 `mode=1/error=0`、lease 正常再运动。
- **不要动** `.env` 之外的用户文件；不要写凭据到代码/日志/报告。

## 9. 2026-08-07 下午续作（重启后验证 + LLM 检测线路 + 相机冻结修复）

> 本节覆盖 15:20–16:10 的新一轮工作。第 1 节“现场状态快照”已被本节取代。

### 9.1 机器人重启后链路验证

- 网络/感知栈/轮式里程计/Point-LIO 桥在重启后仍在运行，但**相机 Bundle 实际已冻结**：
  `frame_id` 停在 10623、`image.jpg` mtime 停在 13:14:27（机器人重启时刻）。
- 9 步 pattern（5 前进 + 4×±20°）实机 PASS：每步轮式里程计校验通过、每步后静止、
  结束自动三次 STOP + disarm（证据 `outputs/go2w_acceptance/restart_verify_20260807/pattern_after_restart_01.jsonl`）。
- 之后两次“目标搜索”全部无效：因为一直在读 13:14 的旧图（GroundingDINO 和 LLM
  都返回完全相同的位置）。**结论：之前怀疑的“灰色书包误检”很大程度是相机断流后
  拿旧图反复检测造成的假象，不是 GroundingDINO 算法本身的问题**；但用户仍决定
  改用 LLM 线路，因为其语义区分（灰色 vs 黑色书包）明显更强。

### 9.2 相机冻结根因与修复

- 根因：`go2w_camera_bridge` 在机器人重启瞬间退出；`live_bridge` 只在收到新相机帧时
  写 Bundle，相机断流后**不再更新** `latest/`，旧 Bundle 的 `sensor_health.camera`
  仍为 true。
- 修复：`kill -INT` 旧感知栈进程组 → 重新 `scripts/go2w/start_live_perception.sh`
  （新会话 `live_20260807_154342`）→ 相机恢复约 15–18 Hz，Bundle 持续更新。
- 防护：`run_autonomous_loop.py` 新增 **3 秒 Bundle 新鲜度门禁**
  （`image_receive_time_ns` 或文件 mtime），超过 3 秒直接中止而不是盲转；
  camera_guided/level_a_search/scan360_approach 四处检测错误处理均已接入。

### 9.3 LLM 检测线路（用户指定，替代 GroundingDINO+SAM2）

- 新增 `app/detectors/siliconflow_vision_worker.py`（conda 子进程，与 ROS 隔离）：
  - `--quick`：紧凑 prompt，只返回“目标在不在 + 一个紧贴 bbox”，实测 5–15 s/次；
  - 非 quick：复用 `SiliconFlowVisionClient.analyze_scene()` 完整场景理解；
  - 修复模型返回**像素坐标**而非 0–1 坐标时 bbox 被丢弃的问题。
- `run_autonomous_loop.py` 新增 `--detector {llm,grounded_sam}`，**默认 llm**；
  `--llm-model` 默认 `Qwen/Qwen3-VL-30B-A3B-Instruct`（比 8B 快约一半，语义结论一致），
  可切回 `Qwen/Qwen3-VL-8B-Instruct`。
- `run_live_robot_demo.py` 默认检测后端从 `grounded_sam` 改为 `llm`。
- README 的 Go2-W 真机部署节已补充自主运行与 LLM 检测用法。

### 9.4 LLM 360° 真机搜索成功（证据）

- 命令：`--mode scan360_approach --target 黑色书包 --detector llm
  --max-radius 1.0 --max-seconds 420`。
- 8 个朝向 ×45°：6 个朝向明确“无黑色书包”，2 个朝向命中：
  `-44.6° score=0.90`、`-177.5° score=0.85`；其中一个朝向 LLM 明确说
  “有灰色书包但目标是黑色书包，颜色不符”（这正是 GroundingDINO 无法做到的判别）。
- 转回最佳朝向（r93）后再次命中，bbox 面积占比 0.266 ≥ 0.15 → `TARGET_REACHED`，
  自动三次 STOP + disarm，最终 `mode=1 / error=0`、静止。
- 证据：
  `outputs/go2w_acceptance/restart_verify_20260807/llm_scan360_backpack_01.jsonl`
  （含 8 个 heading、2 个 candidate、target_reached、finish）
  `outputs/go2w_acceptance/restart_verify_20260807/llm_scan360_backpack_01.mp4`
  （H.264，2272 帧，锁定标签“黑色书包 score 0.9”）。
- **待操作者目视确认**：视频最后锁定的“黑色书包”是否真是黑书包；LLM 还报告
  画面某朝向有灰色书包（原目标），下一轮可改搜“灰色书包”并对照。

### 9.5 当前现场状态

- 感知栈：新会话 `live_20260807_154342` 运行中（camera ~15–18 Hz、Bundle 更新）。
- 运动控制栈：`go2w_motion_control`（lease holder + action server）运行中。
- 机器人：静止，`mode=1 / error=0`，已 disarm。
- 待办（按优先级）：目视确认 LLM 锁定目标 → 把 LLM quick 接入
  `app/live_robot` 状态机正式链路 → 轮式里程计标定 → LIO 直线修复/融合 →
  地图/Nav2。

### 9.6 视频叠加与目标复核（用户反馈后新增）

用户指出：9.4 演示最后锁定的其实是一把**黑色椅子**，不是书包；且候选框中文名
在视频里显示为“????”。

已修复：

1. **中文叠加**：`BundleVideoRecorder` 新增 PIL + Noto Sans CJK 渲染
   （`_draw_cjk_text`），锁定标签“黑色书包 0.90 LOCK”、搜索状态和指令全部可用
   中文显示；无 CJK 字体时自动降级为 ASCII，不再出现问号。
2. **实时指令角标**：每次 `_execute_step` 前把实际运动指令写入视频左下角
   （例如“指令: 左转 30°”“指令: 前进 0.12 m/s × 2s”）；LLM 检测结果和复核
   结论也会显示（“LLM: 未发现目标 @ -89°”“复核: 黑色椅子 拒绝”）。
   录像 sidecar JSONL 每帧新增 `command` 字段。
3. **到达前 LLM 复核**：`siliconflow_vision_worker.py` 新增 `--verify` 模式，
   用 bbox 归一化坐标让模型回答“框内物体是什么 + 是否属于目标”；自主循环在
   `area_ratio >= 到达阈值` 时先复核，`is_target=true` 才写 `target_reached`，
   否则记录 `target_verification` 并右转 15° 继续观察（事件里带
   `object_name_zh/is_target/reason_zh`）。
4. 复核链路已用 9.4 录像末帧端到端自测：quick 仍判“黑色书包”bbox，verify 返回
   “黑色背包/是目标/置信 0.9”——**说明 LLM 对同一画面仍可能持续误判，真机
   复核结论仍需操作者目视确认**；若仍误判，下一步可加“目标颜色+结构属性”约束
   （书包=软包/背带，椅子=靠背/坐面）或让复核模型同时输出属性再比对。

改动文件：
- `scripts/go2w/run_autonomous_loop.py`（叠加、指令角标、复核、3s Bundle 新鲜度）
- `app/detectors/siliconflow_vision_worker.py`（quick/verify 模式、像素坐标归一化）
- `run_live_robot_demo.py`（默认 `llm`）
- `tests/test_siliconflow_vision_worker.py`（新增 6 个单元测试）

### 9.7 2026-08-07 晚：灰色书包真机搜索成功（含过程修复）

用户要求“跑一次新的”，先后跑了 4 次 `scan360_approach --target 灰色书包
--detector llm`，最终成功（`llm_scan360_graybackpack_04`）：

- **成功运行**（04）：LLM 在第 2 个朝向 **-44° 命中“灰色书包”score=0.85**，
  触发新的 `scan360_early_hit` 提前结束扫描（不再转走）；随后对齐靠近：
  cx 0.32→0.46、面积 0.079→0.094、距离推进到 0.98 m；**实验半径限制触发
  RANGE_LIMIT，自动急停 + disarm，`mode=1/error=0`**。
- 录像 `llm_scan360_graybackpack_04.mp4`（3582 帧）sidecar 确认叠加生效：
  “LLM 命中: 灰色书包 0.85 @ -44°”“右转 45°”“左转 20°/16°/11°/8°/5°/4°”
  “前进 0.12 m/s × 2s”等全部实时显示。
- 过程修复（都是本轮暴露的问题）：
  1. **arm 过期**（03 失败）：首次 LLM 检测 120 s 超时后，动作服务器
     `arm_timeout_sec=60` 已过期 → goal 被拒“motion is not armed”。
     修复：`_execute_step` 每次发 goal 前重新 `_arm(True)`（幂等）。
  2. **异常安全**：`_send_goal` 抛异常会导致脚本直接崩溃、跳过急停。
     修复：`_execute_step` 捕获异常并重试；`main()` 加 try/except 兜底，
     任何未处理异常也先 `emergency_stop` + `disarm`。
  3. **相机抖动**：相机流偶发卡 3–4 秒，3 s 门禁直接中止整次搜索。
     修复：门禁放宽到 5 s，并最多重试 3 次（每次 3 s，机器人静止等待安全）。
  4. **转回目标失败**（02 失败）：高分命中后已转到下一朝向，`l44` 回转到目标
     时被“turn crossed target outside tolerance”拒绝。修复：高分（≥0.80）
     命中立即停止扫描不再转走；需回转时先等 1 s；失败 goal 自动重试一次。
- 证据：`outputs/go2w_acceptance/restart_verify_20260807/llm_scan360_graybackpack_02/03/04.*`
  （02 为转回失败、03 为 arm 过期、04 为成功）。
- 结论：LLM 线路能稳定区分“灰色书包”与黑色椅子/黑色背包（本轮没有把椅子当
  目标），距离限制前未触发复核（面积 0.094 < 0.15）；下一轮可调大
  `--reach-area-ratio` 或放开半径继续靠近，以触发到达复核。

### 9.8 2026-08-07 深夜：完整闭环“搜索→到达→LLM 复核”成功 + 相机桥根治

用户确认 9.7 视频里的灰色书包就是目标，要求继续。

#### 完整闭环（`llm_scan360_graybackpack_07`，成功）

- 360° 扫描在第 1 个朝向（0°）即命中“灰色书包”score=0.85 → `scan360_early_hit`
  提前停止扫描；
- 靠近阶段首次检测面积 0.0906 ≥ `--reach-area-ratio 0.08` → 触发 LLM 复核；
- 复核结果：`object_name_zh=灰色书包, is_target=true, confidence=0.92,
  reason_zh=框内是挂在办公椅上的灰色书包，符合目标特征` → `target_reached`；
- 自动急停 + disarm，`mode=1/error=0`，静止。
- 这同时解释了之前的“黑色椅子误判”：**灰色书包挂在黑色办公椅靠背上**，
  bbox 把椅子和书包一起框住，模型早期只报“黑色书包/椅子”。
- 证据：`outputs/go2w_acceptance/restart_verify_20260807/llm_scan360_graybackpack_07.jsonl/.mp4`。

#### 相机桥根治（之前反复卡死/抖动）

根因：`camera_bridge_node.py` 的 RPC 子进程流中只要出现一帧损坏 JPEG
（`cv2.imdecode` 抛异常），整个 `_rpc_loop` 就退出且**不会重启**，相机从此
停更；旧 Bundle 还被 live_bridge 当健康发布。

修复（`ros2_ws/src/go2w_camera_bridge/.../camera_bridge_node.py`，已重新构建并
重启感知栈 `live_20260807_163307`）：

1. 逐帧容错：单帧解码失败只记录诊断并跳过，不再杀死整个 RPC 循环；
2. 读流加 3 秒 `select` 超时，worker 卡死不再无限阻塞；
3. worker 异常/超时后自动重启，退避 1→10 秒；成功发布一帧后复位退避；
4. 实机观察：重启初期 3 次自动重连后相机连续稳定（15–17 Hz，Bundle 更新），
   没有再出现永久冻结。

#### 其它过程加固

- Bundle 新鲜度重试窗口 3 次 → 6 次（每次 3 秒，共约 18 秒），覆盖相机偶发
  长卡；确认断流才中止。
- 每次发动作前重新 arm（避免 LLM 检测超时导致 arm 过期、goal 被拒）；
- `_execute_step` 对 goal 发送异常自动重试；`main()` 兜底保证任何异常先急停。

#### 下一步（按优先级）

1. 轮式里程计标定（轮半径/4WS 运动学，需操作者卷尺实测）；
2. LIO 直线方向偏差修复或 wheel+LIO 融合；
3. 相机外参正式标定 + base→LiDAR TF 复核（大型平板）；
4. 把 LLM quick/复核接入 `app/live_robot` 正式状态机与 launch；
5. 建图 + Nav2（依赖 1–3）。

### 9.9 2026-08-07 深夜：LLM 快速检测/复核接入 app/live_robot 正式状态机

完成第 1 优先级“接入 app/live_robot 状态机”（纯代码 + 真机验证）：

#### 新增/修改（全部通过测试）

- `app/live_robot/search_state_machine.py`：
  - 补齐 `PLAN_STEP → MOVE → WAIT_FOR_STOP → REOBSERVE` 迁移
    （`plan_step/motion_started/motion_completed`）；
  - `VisualEvidence` 新增 `require_mask/require_track_vote`（默认 true，
    原有 fail-closed 语义不变）；LLM 线路可声明“无需 mask/track，用复核代替
    crop verify”完成确认，同时 `source=llm_quick` 仍禁止 LLM 上下文直接确认。
- `app/live_robot/step_planner.py`（新）：纯函数步进规划——扫描序列
  （右扫→回→前进→左扫→回→前进，净航向归零）、对齐转向、前进/半径判定、
  到达复核请求、复核拒绝右转 15°。
- `app/live_robot/step_search_runner.py`（新）：框架级 step-search 编排器，
  检测/复核/动作/传感器全部注入式（ROS 无关、可单测）；相机陈旧直接中止，
  其它检测异常转扫描步。
- `scripts/go2w/run_autonomous_loop.py`：新增 `--mode state_machine_search`，
  用上述正式状态机驱动真机（内部复用 LLM quick 检测、LLM verify 复核、
  `/go2w/motion` 动作、轮式里程计校验）。

#### 测试

- `tests/test_live_search_state_machine.py` 新增 3 个（计划/移动/停止循环、
  未验证动作、LLM 证据门控）；
- `tests/test_step_planner.py` 新增 9 个；
- `tests/test_step_search_runner.py` 新增 4 个（到达复核、偏置对齐、复核拒绝
  重试、无目标超时）；
- 共 21 个相关测试 PASS；live pipeline/navigation gate/frame bundle 在 conda
  Python 下 9 个 PASS。

#### 真机验证（`llm_sm_graybackpack_01`）

`--mode state_machine_search --target 灰色书包 --reach-area-ratio 0.08`：
目标在视野内直接 `target_found(0.85, area 0.0901)` → LLM 复核通过
（“放置在办公椅上的灰色书包”，置信 0.92）→ `target_reached`，steps=0，
自动急停 + disarm，`mode=1/error=0`。

证据：`outputs/go2w_acceptance/restart_verify_20260807/llm_sm_graybackpack_01.jsonl/.mp4`。

#### 剩余优先级

1. 轮式里程计标定（需要操作者卷尺实测轮半径/轮距）；
2. LIO 直线方向偏差修复或 wheel+LIO 融合；
3. 相机外参正式标定 + base→LiDAR TF 复核（大型平板）；
4. 建图 + Nav2（依赖 1–3）。

### 9.10 2026-08-07 深夜：LIO 直线方向偏差深挖（结论：平移 BLOCKED，证据充分）

按用户优先级先做 LIO 直线方向偏差，做了受控标定试验与离线分析。**结论：
Point-LIO（lidar-only、0.2 m 滤波、官方 identity 外参）对平移在数值上就不
稳定，不是简单 yaw 外参能修的；维持“轮式里程计兜底 + LIO 只用于航向”。**

#### 受控试验（`outputs/go2w_acceptance/lio_calibration_20260807/`）

1. **identity 基线前进 36 cm**（`forward_trial_02`，0.12 m/s×2s×2 步）：
   - 轮式里程计 36.6 cm 直线前进、yaw 0.8°；Sport yaw ~0°；
   - LIO（桥接 + yaw_reflect）位移 34.4 cm（幅度 94%），但方向相对自身航向
     偏 **+69°**，z 上漂 **+10.4 cm**；航向本身几乎不变（dyaw 0.6°）。
   - 轨迹显示 LIO 认为自己在“横着走”（航向 ~9°，位移沿 +y）。
2. **重复试验**（旧日志 `imu_turn_log_forward_fast.jsonl`）：相对航向偏
   **+62°/67°**，z 上漂约 5-6 cm——偏差可复现、系统性。
3. **原始点云帧检查**：静止抓取 `/go2w/sensors/cloud`，近场方位直方图显示
   前方场景集中在 ±0°、机器人自身机身集中在 ±180° → **云帧 +x 就是正前方，
   不存在 69° 云帧 yaw 偏置**。
4. **外参候选试验**：`configs/go2w/point_lio_unilidar_l2_calibration.yaml`
   分别设 `extrinsic_R=Rz(+69°)` 和 `Rz(-69°)`（脚本新增
   `POINT_LIO_CONFIG` 覆盖），两种都在 20–35 cm 前进中**发散到公里级**
   （trial 03/04），即纯 yaw 外参方向假设被否证。
5. **干净重启后再测**（`forward_trial_06`，identity，20 cm 前进）：原始
   `/pointlio/odom` 和桥接 `/lio/odom` **同时发散到 13–17 km**——证明问题在
   Point-LIO 内部，不是桥接/反射引入。

#### 结论与处置

- Point-LIO 平移在此办公室场景（L2 上视锥、无足够平移约束）数值不稳定；
  转向（yaw）仍可用（±10° 89%，此前已验收）。
- **直线 LIO 维持 BLOCKED**；继续用 `/go2w/odom/wheel`（轮式里程计）作为
  小范围平移兜底；`navigation_gate.yaml` 未改动，Level D/Nav2 保持 fail-closed。
- 候选后续方向：
  1. wheel+LIO 融合（轮式平移 + LIO/Sport 航向），可在桥接层或 `go2w_wheel_odom`
     内做，先把平移可信；
  2. 再试 `filter_size_surf/map=0.4`（官方默认）与更保守协方差，看平移是否
     不再发散（未在本轮测试，可作下一次离线/短测）；
  3. IMU-input 模式重新调参（此前两种模式均发散/冻结）；
  4. 备选：外接雷达/VSLAM，或等 Unitree 固件恢复 USLAM。

#### 本轮新增/修改文件

- `scripts/go2w/run_point_lio_ros1.sh`：支持 `POINT_LIO_CONFIG` 环境变量；
- `configs/go2w/point_lio_unilidar_l2_calibration.yaml`：实验性 Rz(±69°)
  候选（未通过，保留作证据，勿用于生产）；
- 证据目录：`outputs/go2w_acceptance/lio_calibration_20260807/`
  （forward_trial_02/03/04/05/06、retreat_01、motion jsonl）；
- 临时诊断脚本 `/tmp/lio_raw_recorder.py`（ROS1 原始 odom 记录器）。

### 9.11 2026-08-07 深夜：wheel+LIO 航向融合（已实现 + 真机验证）

按用户要求实现 wheel+LIO 融合。结论：**轮式平移 + Sport/LIO 融合航向**，
新增 `/go2w/odom/fused`；LIO 平移继续 BLOCKED，不参与。

#### 实现（`go2w_wheel_odom` 节点扩展）

- `/go2w/odom/wheel` 保持原语义（纯轮式 + Sport yaw）；
- 新增 `/go2w/odom/fused`（20 Hz）：轮式编码器平移沿**融合航向**积分；
  `fused_yaw_delta = sport_delta + lio_yaw_weight * (lio_delta - sport_delta)`，
  默认 `lio_yaw_weight=0.35`；
- LIO 航向使用门禁（全部满足才融合，否则该 tick 回退 Sport）：
  - 新鲜度：主机收到时刻 < `lio_max_age_sec`（0.5 s）——**不能用消息 stamp**，
    因为 Point-LIO stamp 沿用点云时间、与主机差约 534 s；
  - 位置范数 ≤ `lio_max_position_m`（5 m，拦截 km 级发散）；
  - 数值有限；每 tick 与 Sport 的 yaw 差 ≤ `lio_max_yaw_step_rad`（0.05 rad）；
  - 连续 3 次不一致只告警一次，之后持续回退 Sport；
- 诊断话题 `/go2w/odom/fused/status`：`lio_valid/lio_used/lio_position_norm/
  sport_yaw/fused_yaw/violations`；
- 配置：`configs/go2w/wheel_odom.yaml` 新增 `heading.fusion` 段；
  launch 新增 `lio_enabled/lio_yaw_weight/lio_max_position_m` 参数；
- 单元测试新增 `fuse_yaw_delta`、`heading_delta_sane`（17 个测试全过）。

#### 真机验证（`outputs/go2w_acceptance/fusion_validation_20260807/`）

- 无 LIO：状态 `Sport heading only`，fused 与 wheel 一致（20 Hz）；
- 有 LIO（identity + 0.2 m 滤波）：状态 `LIO heading fused`，
  `lio_used=true` 全程，位置范数最大 0.60 m（门限 5 m），无违规；
- 动作 `l20 → 前进 → r20 → 前进`：
  - wheel：平移 0.633 m、yaw 0.06°→1.25°；
  - fused：平移 0.633 m、yaw 0.03°→1.08°（LIO 权重 0.35 + LIO 89% 幅度的
    预期微差）；
  - 每步后静止，结束自动 STOP/disarm，`mode=1/error=0`。
- 若未来 LIO 平移修复，可把 `lio_yaw_weight` 调高或改用 LIO 位置；当前
  `/go2w/odom/fused` 即为小范围推荐里程计，`navigation_gate.yaml` 仍未改动。

#### 文件

- `ros2_ws/src/go2w_lio_bringup/go2w_lio_bringup/wheel_odom.py`
- `ros2_ws/src/go2w_lio_bringup/launch/wheel_odom.launch.py`
- `ros2_ws/src/go2w_lio_bringup/test/test_wheel_odom.py`
- `configs/go2w/wheel_odom.yaml`
- 证据：`outputs/go2w_acceptance/fusion_validation_20260807/fusion_trial_01*.jsonl`

### 9.12 2026-08-07 深夜：自主循环接入融合里程计 + LIO filter 0.4 平移试验

#### `run_autonomous_loop.py` 新增 `--odom-topic`

- 默认 `/go2w/odom/wheel`（行为不变）；可切 `/go2w/odom/fused` 让搜索/位移
  校验直接使用融合里程计。
- 真机验证：`--odom-topic /go2w/odom/fused --mode pattern --pattern f,f`
  两步前进均通过校验（25.2 cm / 22.7 cm），结束静止；
  证据 `outputs/go2w_acceptance/fusion_validation_20260807/odom_topic_fused_01.jsonl`。

#### LIO `filter_size=0.4`（官方默认）平移试验

- 配置：lidar-only、filter 0.4/0.4、identity 外参、yaw_reflect；
- 前进 0.32 m（轮式）结果：LIO 位移只有 **0.061 m（19%）**、方向
  **-127°**、z −0.03 m、dyaw 1.92°；位置范数最大 0.72 m（未发散但基本不跟踪
  平移）。
- 与 0.2 滤波对比：0.2 能跟上幅度（94%）但方向偏 +69°、z 上漂；0.4 幅度
  塌缩。**结论强化：Point-LIO 平移在此场景不可用（两个滤波档位都不行），
  维持轮式/融合里程计兜底。**
- 证据：`outputs/go2w_acceptance/lio_filter04_20260807/filter04_forward_01.jsonl`

#### 下一步

1. **轮式里程计标定**（需要操作者卷尺实测轮径/轮距，标定后 `--odom-topic
   /go2w/odom/fused` 平移会更准）；
2. 或继续调 Point-LIO（IMU-input 重新调参、场景约束分析）——收益预期较低；
3. 相机外参正式标定 + base→LiDAR TF 复核（大平板）；
4. 建图 + Nav2（依赖 1/3）。

### 9.13 2026-08-07 晚：灰色书包复测成功（验证层拦截误检后真正确认）

用户要求再测一次“寻找灰色书包”并导出视频。使用
`scan360_approach --target 灰色书包 --detector llm --odom-topic
/go2w/odom/fused --reach-area-ratio 0.08`：

- 第 2 个朝向（-44.6°）quick 检测“灰色书包”score 0.90 → 提前停止扫描并靠近；
- **到达复核连续两次拒绝**：`黑色背包，is_target=false，置信 0.95`（框内是黑色
  背包，不是灰色书包）→ 机器人未停在误检目标，右转继续搜索；
- 继续搜索后最终命中并复核通过：`灰色书包，is_target=true，置信 0.92，
  “放置在办公椅上的灰色书包”` → `target_reached`；
- 结束自动急停 + disarm，`mode=1/error=0`；融合里程计位移校验全程正常。
- 证据：`outputs/go2w_acceptance/restart_verify_20260807/
  llm_scan360_graybackpack_08.jsonl/.mp4`（123 s、H.264、768×432）。
- 导出视频：`/home/brov/下载/go2w_灰色书包搜索_20260807.mp4`
  （已用 ffprobe 验证可播放，h264/mp4）。
