# Go2-W + PandarXT-16 + plain_slam_ros2 集成 — 交接书（HANDOVER）

> 交接日期：2026-08-31
> 配套文档：`GO2W_PANDAR_PLAIN_SLAM_IMPLEMENTATION_PLAN.md`（计划书，2700 行，**新 AI 必须先通读**）
> 本文档目的是：让一个没有本会话上下文的新 AI，仅凭「计划书 + 本文档 + 仓库代码」即可无缝继续。

---

## 0. 一句话现状

**plain_slam_ros2 + Hesai PandarXT-16 的 mapping-assist 链路已全部实现并在真机运行通过**：真实 Pandar 点云 → `/go2w/slam/pandar_points` → LIO/SLAM（`/go2w/slam/aligned_scan`、`map_3d`、`odom_base`）→ ray-traced `/go2w/slam/map_2d`（free/occupied/unknown）→ `PlainSlamSpatialProvider`（METRIC_LIDAR，真机提取 8 个 frontier）→ 供 `run_semantic_exploration.py --spatial-provider plain_slam` 使用。`/go2w/slam/ready=true`（READY_MAPPING_ASSIST），**全程未改任何安全授权字段、未抢占 `/go2w/odom/fused`、未发任何运动指令**。

---

## 1. 环境拓扑（必须先搞清楚的事实）

| 项 | 宿主机（工作站） | 机器狗（192.168.123.18） |
|---|---|---|
| 用户名/密码 | mxt / 123（sudo 同为 123） | unitree / 123（sudo 同为 123） |
| 架构/系统 | x86_64，Ubuntu（python3.10.12），**ROS 2 Humble 完整可用** | Jetson aarch64（nvidia l4t 35.3.1，Ubuntu 20.04 Focal） |
| ROS 真相 | `/opt/ros/humble` = 真 Humble，正常 | **`/opt/ros/humble` 是 Foxy 的副本**（ros2cli 0.9.13，`/opt/ros/humble/bin/ros2` 符号链接到 `/opt/ros/foxy/bin/ros2`）；真实可用 ROS = **Foxy**（apt 官方 focal 源） |
| 项目位置 | `/home/mxt/robotscene`（**git 仓库**，分支 `feature/semantic-object-topology`） | `/home/unitree/robotscene`（非 git，文件部署副本） |
| 网络 | `enp3s0`: 192.168.1.10 + **192.168.123.99/24**（Pandar 192.168.123.20 直连本机网口）；wlo1: 192.168.3.129 | eth0: 192.168.123.18；机器狗 DCU（运动控制器）经内网发布传感器数据 |
| 传感器数据源 | Hesai PandarXT-16（10Hz，64000 点/帧，host 192.168.123.99 收 UDP:2368） | DCU 裸 DDS 应用发布 `/utlidar/imu`（约 250Hz）、`/lf/sportmodestate`、`/lf/lowstate`（域 0，跨机可见） |
| SSH | - | `sshpass -p 123 ssh unitree@192.168.123.18` |

**DDS 关键事实**：宿主机与机器狗跨机通过 CycloneDDS 域 0 互通。宿主机网卡多地址时 CycloneDDS 可能错选 192.168.1.10 → 必须固定到 192.168.123.99（见 §4 修复）。

---

## 2. 已完成的全部工作（按计划书 Step）

### Step 1：vendor plain_slam_ros2 + 依赖锁 ✅
- `ros2_ws/src/plain_slam_ros2/` @ commit `0c4b610f6a486a906d51dd32e897bc787f67b208`（`configs/go2w/plain_slam_lock.yaml` 记录）。
- Sophus **v0.9.5**（tag 不是 v0.9.0！）+ nanoflann **v1.6.3**（apt 的 1.4.2 太旧，需装到 /usr/local）。
- **4 个上游最小补丁**（必须保留，否则宿主机 GCC12/Eigen3.4 或机器狗 GCC9/Eigen3.3 编译失败）：
  - `patches/plain_slam_ros2/0001-eigen-cholesky-includes.patch`
  - `patches/plain_slam_ros2/0002-eigen-dense-includes.patch`
  - `patches/plain_slam_ros2/0003-eigen-geometry-quaternion-io.patch`
  - `patches/plain_slam_ros2/0004-eigen33-quaternion-coeffs.patch`
- `scripts/go2w/vendor_plain_slam_ros2.sh`：clone → pin → **自动幂等应用补丁** → 装 Sophus/nanoflann（支持 `SUDO_PASSWORD` 环境变量）。
- `scripts/go2w/install_dependencies.sh`、`build_ros2.sh` 已加相应依赖/检查（plain_slam 缺失时 build 显式报错）。

### Step 2：配置生成器 ✅
- `scripts/go2w/generate_plain_slam_pandar_config.py`：读 `official_reference.yaml` + `hesai_pandarxt16_extrinsics.yaml`，自动推导 `T_imu_pandar`（**验证值 [-0.13042, 0.02966, 0.09357]，与计划书 §2.2 参考完全一致**）。
- 生成 `runtime/go2w/plain_slam/` 下：
  - `generated_lio_3d_config.yaml` / `generated_slam_3d_config.yaml`（**带节点名包装**，launch parameters 用）
  - `lio_3d_params.yaml` / `slam_3d_params.yaml`（**无包装顶层格式！上游固定文件名**：`lio_3d_interface.cpp` 硬编码读 `<param_files_dir>/lio_3d_params.yaml`）
  - `generated_lio_3d_params.yaml` / `generated_slam_3d_params.yaml`（计划书命名的同内容别名）
  - `config_provenance.json`（mapping_assist_only、candidate_unconfirmed）
- `--check` 模式输出计划书要求 4 行结论；**不修改任何源 YAML、不提升安全字段**。
- 配置文件：`configs/go2w/plain_slam.yaml`（主）、`plain_slam_lio_params.yaml`、`plain_slam_slam_params.yaml`、`plain_slam_bridge.yaml`、`plain_slam_lock.yaml`、`plain_slam_mapping.rviz`。

### Step 3–7：go2w_plain_slam_bridge C++ 包 ✅（`ros2_ws/src/go2w_plain_slam_bridge/`）
- `include/go2w_plain_slam_bridge/`：`timestamp_policy.hpp`（7 种时间戳自动策略，绝不静默猜）、`pointcloud_utils.hpp`、`transform_utils.hpp`（**注意：矩阵平移在第 4 列 m[3]/m[7]/m[11]，不是 m[12..14]**）、`occupancy_grid.hpp`（DDA ray-trace + GroundEstimator + polar_downsample；**C++14 兼容，勿用 std::clamp**——机器狗 GCC9/Foxy 默认 C++14）。
- `src/`：`pandar_slam_adapter.cpp`（schema 校验/过滤/时间戳策略，输出 24 字节 schema x,y,z,intensity F32 + timestamp F64）、`plain_slam_odom_adapter.cpp`（IMU pose→base pose，publish_tf 默认 false）、`pointcloud_to_occupancy.cpp`（DDA 射线 + 自动地面 + log-odds，80×80m@0.1m，2Hz）、`plain_slam_health_monitor.cpp`（五类 topic 新鲜度状态机 → READY_MAPPING_ASSIST；**时间用 steady_clock，勿混 rclcpp 时钟**）。
- `launch/plain_slam_go2w.launch.py`：**参数必须以扁平 dict 传**（节点名 `go2w_plain_slam_lio` ≠ YAML 段名 `lio_3d_node`，传文件会被静默忽略导致 lidar_type=livox！）；launch 同步重生成 runtime 配置；`start_upstream:=false` 支持 bridge-only 离线测试。
- `scripts/imu_fallback_adapter.py`：见 §4 关键修复。
- 包内 gtest：`timestamp_policy_test`（10）、`occupancy_projection_test`（3）、`transform_utils_test`（2），宿主机与机器狗均全绿。

### Step 8：PlainSlamSpatialProvider ✅（`app/spatial/plain_slam_spatial_provider.py`）
- `app/spatial/models.py` 新增 `SPATIAL_QUALITY_METRIC_LIDAR`；`spatial_transform.py::quality_weight` 已纳入（权重 1.0）。
- **QoS 修复（真 bug）**：`/go2w/slam/odom_base` 由 adapter 以 SensorDataQoS（BEST_EFFORT）发布，provider 默认 RELIABLE 订阅收不到 → 必须 BEST_EFFORT 订阅（`QoSProfile(reliability=BEST_EFFORT, depth=10)`）。
- 实机验收：`get_map=true`、`get_pose=true`、`quality=METRIC_LIDAR`、`frontier_count=8`。

### Step 9：探索入口改造 ✅
- `run_semantic_exploration.py`：新增 `--spatial-provider {camera,rtabmap,plain_slam}`（默认 camera，`--rtabmap`/`--plain-slam` 兼容别名）；frontier 逻辑改为 provider-generic（metric map 优先于 BEV）；验收日志行（`spatial provider = ...`、`map source = ...`、`route planner consumes ...`）。
- `start_semantic_exploration.sh`：支持 `GO2W_SPATIAL_PROVIDER` 与 `GO2W_START_PLAIN_SLAM=1`（自动拉起 mapping，60s 等 ready，未就绪不阻塞）。

### Step 10：一键脚本 ✅
- `start_plain_slam_mapping.sh`（source `setup_environment.sh` → 生成配置 → Hesai 探测（不双开）→ 等点云 → launch → 等 ready → 摘要；`--rviz/--no-start-hesai/--record/--debug`）、`stop_plain_slam_mapping.sh`（只杀本项目 ros2_ws 路径进程，TERM+KILL 兜底；不动运动栈/既有 Hesai）、`check_plain_slam_mapping.sh`、`record_plain_slam_debug_bag.sh`、`save_plain_slam_map.sh`（PCD/PGM/YAML/provenance；大 grid 用 Python 订阅探针，勿用 `ros2 topic echo` 全量）。
- `test_plain_slam_bridge_offline.sh`：fake fixture 全链路离线验收（map 三角、ready、TF 隔离、fused 无 bridge publisher）。
- Nav2 Level B：`robot_scene_nav_bringup` 增加 `use_plain_slam_map:=false` 参数 + `scripts/plain_slam_map_relay.py`（默认关闭）。

### Step 11：测试与真机验证 ✅（详见 §5 数据）
- 宿主机：13 包 colcon build 0 error；新增 30 个 Python 单测全绿；离线 bridge smoke PASS。
- 机器狗（Foxy）：`plain_slam_ros2` + `go2w_plain_slam_bridge` + `robot_scene_nav_bringup` 构建成功；gtest 15/15 通过。
- 真机只读 smoke：**全部 PASS**（见 §5）。

### Step 12：文档 ✅
- `docs/go2w/plain_slam_pandarxt16.md`（架构/topic/启动/health/降级）、`docs/go2w/plain_slam_license_note.md`（上游学术/个人免费，商用需授权）、`docs/GO2W_REAL_ROBOT_DEPLOYMENT.md` 顶部提醒。

---

## 3. 新增/修改文件清单（交付物）

### 新增（未跟踪）
```
configs/go2w/plain_slam.yaml、plain_slam_lio_params.yaml、plain_slam_slam_params.yaml、
  plain_slam_bridge.yaml、plain_slam_lock.yaml、plain_slam_mapping.rviz
scripts/go2w/generate_plain_slam_pandar_config.py、publish_plain_slam_fake_fixture.py、
  acceptance_plain_slam_provider.py、start_plain_slam_mapping.sh、stop_plain_slam_mapping.sh、
  check_plain_slam_mapping.sh、record_plain_slam_debug_bag.sh、save_plain_slam_map.sh、
  test_plain_slam_bridge_offline.sh、vendor_plain_slam_ros2.sh
ros2_ws/src/plain_slam_ros2/           （vendor + 本地 patch commits）
ros2_ws/src/go2w_plain_slam_bridge/    （完整 C++ 包 + launch + gtest + scripts/imu_fallback_adapter.py）
patches/plain_slam_ros2/*.patch        （4 个）
app/spatial/plain_slam_spatial_provider.py
tests/test_plain_slam_*.py             （5 个：extrinsic/config_generation/timestamp_policy/
                                        occupancy_projection/spatial_provider）
docs/go2w/plain_slam_pandarxt16.md、plain_slam_license_note.md
runtime/go2w/plain_slam/*              （生成物，勿手改）
```

### 修改（tracked）
```
app/spatial/models.py（METRIC_LIDAR）、app/spatial/spatial_transform.py（quality_weight）
scripts/go2w/run_semantic_exploration.py、start_semantic_exploration.sh
scripts/go2w/build_ros2.sh、install_dependencies.sh
ros2_ws/src/robot_scene_nav_bringup/（CMakeLists、launch、scripts/plain_slam_map_relay.py）
docs/GO2W_REAL_ROBOT_DEPLOYMENT.md
```

### 外部会话（非本会话）已并入的关键修改——已核实，**不要回退**
```
scripts/go2w/setup_environment.sh（新：DDS 固定/环境清理）
scripts/go2w/start_plain_slam_mapping.sh（source setup_environment.sh）
ros2_ws/src/go2w_plain_slam_bridge/scripts/imu_fallback_adapter.py（重写为 host-stamp 对齐版）
configs/go2w/plain_slam.yaml（topics.imu: /go2w/slam/imu）
configs/go2w/cyclonedds_go2w.xml、scripts/go2w/start_hesai_pandarxt16.sh、unitree_go2w_control/* 等
```

---

## 4. 关键根因与修复（新 AI 必须理解，避免重新踩坑）

### 4.1 IMU 时间戳落后 Pandar ~718 秒（LIO 空扫描的根因）
- 现象：LIO 一直输出**空** aligned_scan（`w=0`），日志刷 `No relevant IMU measurements were found`；SLAM 一直 `Skipping SLAM process because the scan cloud is empty`。
- 根因：`/utlidar/imu` 数据其实一直在（约 250Hz），但 Unitree 传感器时钟保留 DCU 开机 epoch，**比 Pandar（主机时钟）落后约 717–719 秒** → LIO 找不到与扫描时间匹配的 IMU → gravity 估计永不完成 → 跳过扫描。
- 修复：`imu_fallback_adapter.py` 订阅 `/utlidar/imu`，`copy.deepcopy` 保留真实测量值，**仅把 `header.stamp` 改写为接收时刻**（`node.get_clock().now()`），发布 `/go2w/slam/imu`；状态经 `/go2w/slam/imu_status` 标记 `IMU_SOURCE_OK_HOST_STAMP`（真流）或 `SYNTHETIC_STATIC`（无真流时的静态合成兜底，仅 mapping-assist）。LIO（`generated_lio_3d_config.yaml` 的 `imu_topic`）与 health monitor 均改订阅 `/go2w/slam/imu`。
- 验证：对齐后 IMU 250.1Hz，`imu_status=IMU_SOURCE_OK_HOST_STAMP`，aligned_scan 9.99Hz/60775 点，SLAM_READY=true。

### 4.2 DDS 选中错误网卡
- 根因：宿主机 enp3s0 同时有 192.168.1.10 与 192.168.123.99，CycloneDDS 名称选择可能挑错地址。
- 修复：`setup_environment.sh` 自动探测 `192.168.123.x` 网卡/地址（GO2W_INTERFACE/GO2W_HOST_IP），按模板生成 `runtime/go2w/cyclonedds_<iface>.xml` 并 export `CYCLONEDDS_URI`。启动/检查脚本统一 source 它。

### 4.3 ROS 环境污染
- 机器狗 `/opt/ros/humble` 是 Foxy 副本；宿主机可能残留 Noetic/Foxy overlay。`setup_environment.sh` 统一清理并显式导出 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`、`ROS_DOMAIN_ID`。

### 4.4 其它已修的真 bug（有测试保护）
- transform 矩阵平移列位置（m[3]/m[7]/m[11]）→ `transform_utils_test` 锁定。
- launch 参数节点名不匹配 → 扁平 dict 传参。
- 上游 params 文件固定名/无包装 → 生成器已对齐。
- provider 订阅 QoS（BEST_EFFORT）→ 实机 pose 可收。
- occupancy 的 steady_clock 混用崩溃、`std::clamp`（C++17）机器狗 GCC9 不兼容。
- fake fixture 用 `struct.pack_into`（机器狗 numpy 1.17 + rclpy 组合段错误）；IMU 节奏 100Hz 稳定。

---

## 5. 真机验证结果（2026-08-31，mapping-only，未发运动指令）

```
/go2w/slam/ready:              true  (READY_MAPPING_ASSIST)
aligned_scan:                  9.99 Hz，单帧 60775 点（真实，非 0）
/go2w/slam/imu（对齐后）:      250.1 Hz（IMU_SOURCE_OK_HOST_STAMP）
/utlidar/imu（原始）:          250.4 Hz
/go2w/slam/pandar_points:      ~10 Hz（64000 点/帧，ABSOLUTE_SECONDS，过滤少量零回波）
/go2w/slam/odom_base:          有数据（pslam_odom / base_link_mapping_assist）
/go2w/slam/map_2d:             2 Hz；free 38597 / occupied 39 / unknown 601371（80×80m@0.1m）
health:                        POINTCLOUD_OK / IMU_OK freq=230Hz / LIO_OK /
                               ALIGNED_SCAN_OK / GROUND_ESTIMATE_OK /
                               EXTRINSIC_CANDIDATE / mode=MAPPING_ASSIST
                               motion_authorized=false safety_authorized=false
PlainSlamSpatialProvider:      get_map=true get_pose=true quality=METRIC_LIDAR
                               frontier_count=8（含 distance_m 5.07/19.05/19.58m）
/hesai/pandarxt16/points_raw:  10.003 Hz（真实 Pandar，64000 点/帧）
/go2w/odom/fused:              未被任何 bridge 节点发布（授权链未动）
```

地图存档：`outputs/maps/20260831_173149/`（map_2d.pgm + map_2d.yaml + provenance.json）。

---

## 6. 未完成事项（新 AI 的 TODO，按优先级）

1. **全量 pytest 收尾**（唯一未跑完的自动化项）：
   - 分片结果：slice0=220 passed / slice1=193 passed / slice2=197 passed+1 failed / slice3 的 A 组=76 passed+4 failed；**slice3 的 B 组存在挂起测试（整片 130 中断），未定位**。
   - 已知失败均与 plain_slam 无关（环境/外部差异）：
     - `test_ros2_command_exporter.py::...test_builds_cmd_vel_commands_from_route_steps`：期望 distance_m=0.8，实际 0.3（外部对运动步长的安全限制改动，测试未同步）。
     - `test_streamlit_video_mode.py`、`test_go2w_live_ui_status.py` 等：streamlit_app.py 在项目根、测试相对路径解析失败（环境性）。
     - `test_task_examples_evaluator.py` 等示例类。
   - 建议：定位 slice3 B 组挂起者（分组二分），确认后视情况跳过并记录。
2. **map_3d 证据保存**：静止时无新位姿关键帧 → `/go2w/slam/map_3d` 数据稀疏，`save_plain_slam_map.sh` 会跳过 PCD。机器狗移动/有新关键帧后再保存一次即可。
3. **机器狗本地运行时 smoke**：机器狗 Foxy 本地 rclpy 曾有 CycloneDDS 段错误历史（`/opt/ros/humble`=Foxy 副本混乱所致，外部已修环境 overlay）；代码已同步+构建通过。若需在机器狗本地跑节点，先验证 `python3 -c "import rclpy; rclpy.init()"` 不再崩溃。
4. **探索干跑**（完整版）：`run_semantic_exploration.py --spatial-v2 --spatial-provider plain_slam --dry-run-motion --allow-degraded` 依赖 LLM/相机/运动后端；核心 spatial 数据链已用 `acceptance_plain_slam_provider.py` 验证（get_map/get_pose/frontiers 全通）。
5. **小范围运动验证**（用户已授权，但**当前 mapping-only 保持运行，未发任何运动指令**）：
   - 前置：运动栈运行（`unitree_go2w_control` 的 go2w_motion_control + wheel odom + lease holder，`/go2w/motion` action server 在线，`/go2w/odom/fused` 正常发布）。
   - 流程建议：静止建图基线 → 原地转向 ≤30° → 低速直行 ≤1.5m → 观察 map_2d 增长、`/go2w/odom/fused` 运动执行正常、plain_slam 影子里程计不介入。
   - 严格遵守：不提高授权、随时可急停、操作者在场。
6. **最终交付报告 + delivery_check**（本会话的交付 gate）：需整理「修改文件清单 + 构建结果 + 测试结果 + 一键启动命令 + 已知降级项」并提交 evidence（file/text/run 类均可，页面类不适用）。

---

## 7. 一键命令速查

```bash
# —— 真机 mapping（宿主机 Humble）——
bash scripts/go2w/start_plain_slam_mapping.sh            # 或加 --rviz / --record
bash scripts/go2w/stop_plain_slam_mapping.sh
bash scripts/go2w/check_plain_slam_mapping.sh
bash scripts/go2w/save_plain_slam_map.sh
bash scripts/go2w/record_plain_slam_debug_bag.sh

# —— 离线 bridge-only 自检（无需真机）——
PLAIN_SLAM_TEST_SECONDS=50 bash scripts/go2w/test_plain_slam_bridge_offline.sh

# —— 语义探索一键（plain_slam 作空间来源）——
GO2W_START_PLAIN_SLAM=1 GO2W_SPATIAL_PROVIDER=plain_slam \
  bash scripts/go2w/start_semantic_exploration.sh --target "你的目标"

# —— 干跑（不发运动）——
python3 scripts/go2w/run_semantic_exploration.py --target "测试" \
  --spatial-v2 --spatial-provider plain_slam --dry-run-motion --allow-degraded

# —— provider 实机验收 ——
/usr/bin/python3 scripts/go2w/acceptance_plain_slam_provider.py

# —— 构建（宿主机）——
bash scripts/go2w/install_dependencies.sh
bash scripts/go2w/vendor_plain_slam_ros2.sh --commit 0c4b610f6a486a906d51dd32e897bc787f67b208
bash scripts/go2w/build_ros2.sh
```

---

## 8. 给新 AI 的操作注意事项（血泪坑）

1. **执行命令**：本环境没有直接 bash 工具时，可用宿主 `dev_stage_add` 挂一个 exec 工具（Node v22 无 `require`，用 `process.getBuiltinModule('child_process').execSync(cmd, {shell:'/bin/bash'})`；返回值必须 `JSON.stringify`；`timeout` 别超 600s）。
2. **pkill 自杀坑**：命令字符串里 `pkill -f '<关键词>'` 会匹配并杀掉执行命令自身（code=null 无输出）。清理进程请写成脚本文件再执行。
3. **长任务**：SSH 内 `nohup ... &` + 轮询日志；单次 exec 控制在 300s 内。
4. **测试环境**：pytest 用 `.venv/bin/python -m pytest`（3.13）；ROS 侧 Python 用 `/usr/bin/python3`（3.10）。
5. **机器狗构建**：`source /opt/ros/foxy/setup.bash`（不要用 humble 副本）；机器狗构建很慢（plain_slam 约 9.5 分钟）。
6. **launch 参数**：必须扁平 dict；YAML 段名与节点名不匹配会静默丢参数（表现为 lidar_type=Livox）。
7. **上游 params 文件**：`lio_3d_params.yaml`/`slam_3d_params.yaml` 无包装、固定文件名；生成器每次 launch 自动重生成 runtime。
8. **Eigen 补丁**：任何重新 vendor 都必须应用 `patches/plain_slam_ros2/*.patch`（vendor 脚本已自动做）。
9. **IMU 时钟**：若再次出现 LIO 空扫描，先查 `ros2 topic hz /go2w/slam/imu` 与 `/go2w/slam/imu_status`；DCU 重启后 IMU 可能从 1Hz 渐进恢复到 250Hz，需等待。
10. **机器狗重启后**：DCU 完全启动比 Ubuntu 慢；`/utlidar/imu` 与 `/lf/sportmodestate` 恢复后才有数据流（topic 占位≠有数据）。
11. **安全红线（不可违反）**：`confirmed/authorizes_*` 保持 false；`/go2w/odom/fused` 与运动链不被 mapping 触碰；`/go2w/slam/ready=true` 仅表示 mapping 可用，绝不等于运动授权。
12. **静止时的正常现象**：`MAP3D_STALE`（无新位姿关键帧）、map_3d 数据稀——不是故障；IMU 用 host-stamp 对齐后不需要重启机器狗。

---

## 9. 关键文件索引（新 AI 第一站）

| 文件 | 作用 |
|---|---|
| `GO2W_PANDAR_PLAIN_SLAM_IMPLEMENTATION_PLAN.md` | 计划书（权威需求） |
| `configs/go2w/plain_slam.yaml` | 主配置（topic/frame/健康阈值/语义） |
| `scripts/go2w/setup_environment.sh` | DDS 固定 + 环境清理（一切 ROS 命令前 source） |
| `scripts/go2w/generate_plain_slam_pandar_config.py` | 配置生成器（`--check` 自检） |
| `ros2_ws/src/go2w_plain_slam_bridge/` | 全部 bridge 代码（4 C++ 节点 + launch + gtest + IMU 适配器） |
| `app/spatial/plain_slam_spatial_provider.py` | 探索侧 provider |
| `scripts/go2w/start_plain_slam_mapping.sh` | 一键启动（真机） |
| `scripts/go2w/test_plain_slam_bridge_offline.sh` | 离线全链路自检 |
| `scripts/go2w/acceptance_plain_slam_provider.py` | provider 实机验收 |
| `docs/go2w/plain_slam_pandarxt16.md` | 使用文档（架构/启动/health/降级） |
| `outputs/maps/20260831_173149/` | 真机地图存档（证据） |
