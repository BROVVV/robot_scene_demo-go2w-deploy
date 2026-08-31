# Go2-W + Hesai PandarXT-16 + plain_slam_ros2 实时 3D 建图融合实施计划

> 目标仓库：`BROVVV/robot_scene_demo`
>
> 目标分支：`robot-go2w-deployment-20260828`
>
> 目标平台：Unitree Go2-W + 外置 Hesai PandarXT-16 + ROS 2 Humble / Ubuntu 22.04
>
> 本文用途：**直接交给代码 AI/编程 Agent 执行。AI 应按本文一次性完成代码修改、配置、构建脚本、启动脚本、自动检查和文档，不再向操作者要求做手工外参标定、PTP/GPS 标定或专门推/转机器人采集标定数据。**

---

## 0. 一句话任务定义

在现有 `robot_scene_demo` 中集成 `plain_slam_ros2`，以 **Hesai PandarXT-16 为主要 3D LiDAR**、以项目现有 IMU 为惯性输入，实现：

1. 机器狗正常执行现有探索/导航动作时，后台持续实时 LIO；
2. 持续生成和更新 3D 点云地图；
3. 从 LIO 对齐后的实时扫描生成可用于探索的 2D OccupancyGrid；
4. 让现有 `FrontierExtractor`、`SemanticRoutePlanner`、SemanticNavigation V2 能读取该地图；
5. **第一版不让 plain_slam 直接接管机器人运动里程计，也不替换 `/go2w/odom/fused`；**
6. 当前未确认的 Pandar 外参只作为 `mapping_assist` 的 best-effort 参数使用，**不得修改现有安全授权状态，不得把它伪装成已经标定完成；**
7. 所有能自动判断、自动派生、自动降级的事情都自动完成，不要求操作者做额外标定动作。

最终目标数据流：

```text
Hesai PandarXT-16
        │
        ▼
Hesai ROS2 Driver
        │
        ▼
/hesai/pandarxt16/points_raw
        │
        ▼
go2w_plain_slam_bridge/pandar_slam_adapter
        │
        ▼
/go2w/slam/pandar_points
        │
        ├──────────────────────────────┐
        │                              │
        ▼                              ▼
 plain_slam_ros2                 /utlidar/imu
   lio_3d_node                         │
        │                              │
        └──────────────┬───────────────┘
                       ▼
             LIO + deskew + local map
                       │
         ┌─────────────┼─────────────┐
         ▼             ▼             ▼
   aligned scan      odometry     3D local map
         │             │             │
         │             │             └──► /go2w/slam/map_3d
         │             ▼
         │      plain_slam_odom_adapter
         │             │
         │             ▼
         │     /go2w/slam/odom_base
         │
         ▼
 pointcloud_to_occupancy
         │
         ▼
/go2w/slam/map_2d  (free / occupied / unknown)
         │
         ▼
PlainSlamSpatialProvider
         │
         ├──► FrontierExtractor
         ├──► SemanticRoutePlanner
         └──► SemanticNavigation V2

现有运动定位链：
/go2w/odom/fused ─────────► 现有运动执行 / motion gate

注意：第一版 plain_slam 是“地图和空间理解来源”，不是安全运动权威。
```

---

# 1. 强制约束：AI 不得改变的原则

## 1.1 不要求人工重新标定

本次实现**禁止把以下事情作为完成前置条件**：

- 不要求操作者手动测量 Pandar 安装位置；
- 不要求操作者让机器狗“直走 1 米 / 原地转几圈 / 走 8 字”来专门标定；
- 不要求采集多场景外参数据；
- 不要求配置 PTP/GPS 才能运行；
- 不要求手工测 IMU bias；
- 不要求手工改 TF 数字后反复试；
- 不要求先修复旧 Point-LIO 才能使用新链路。

### 实施策略

优先复用：

- `configs/go2w/hesai_pandarxt16_extrinsics.yaml` 中已有 Pandar 候选外参；
- `configs/go2w/official_reference.yaml` 中已经接受的 `base_link -> utlidar_lidar -> utlidar_imu` 厂商几何；
- `/utlidar/imu` 作为默认 IMU；
- Hesai ROS2 驱动已经提供的每点 `timestamp`；
- 现有 `/go2w/odom/fused` 作为运动执行的可靠兜底。

如果某项数据质量不足，程序应：

1. 自动检测；
2. 输出明确 warning/health 状态；
3. 尽可能降级继续建图；
4. **不得自动提高任何 safety authorization；**
5. 不因为 mapping 模块异常而破坏已有运动链。

---

## 1.2 不修改 Pandar 的安全授权事实

当前：

`configs/go2w/hesai_pandarxt16_extrinsics.yaml`

明确是：

```yaml
calibration_status: candidate_unconfirmed
confirmed: false
authorizes_tf_publication: false
authorizes_safety_integration: false
authorizes_motion: false
```

**这些字段保持原样。**

本计划允许将候选外参数字用于：

- plain_slam 内部 LIO 数学变换；
- mapping-only 地图；
- exploration/frontier 的 best-effort 空间辅助。

但不允许：

- 修改 `confirmed: true`；
- 修改任何 `authorizes_*: true`；
- 把 Pandar 点云直接接入已有 collision/safety chain；
- 用这个候选值宣称完成正式标定。

如果需要 frame 名，使用隔离的 mapping frame，例如：

```text
pslam_odom
pslam_imu
pandarxt16_link_unvalidated
```

不要发布一个看起来像正式授权的 `base_link -> pandarxt16_link` 静态 TF。

---

## 1.3 第一版绝对不要抢占 `/go2w/odom/fused`

不要：

```text
plain_slam -> /go2w/odom/fused
```

也不要修改现有 wheel odom / fused odom owner。

新增：

```text
/go2w/slam/odom_imu
/go2w/slam/odom_base
```

作为独立 shadow odometry。

现有运动执行仍使用：

```text
/go2w/odom/fused
```

这样即使 Pandar 外参不完美、LIO 漂移或初始化失败，机器狗现有控制链也不会被拖垮。

---

# 2. 当前仓库事实与本次利用方式

## 2.1 Pandar 驱动现状

文件：

```text
configs/go2w/hesai_pandarxt16.yaml
```

当前关键值：

```yaml
use_timestamp_type: 1
default_frame_frequency: 10.0
transform_flag: false
ros_frame_id: pandarxt16_link_unvalidated
ros_send_point_cloud_topic: /hesai/pandarxt16/points_raw
send_point_cloud_ros: true
send_imu_ros: false
```

保持上述原则：

- 不强制改 PTP；
- 不打开 Hesai 自身 transform；
- 不强制开启 Hesai IMU；
- 继续使用 `/hesai/pandarxt16/points_raw`。

Hesai 官方 ROS2 driver 的 PointCloud2 已经输出：

```text
x          FLOAT32
y          FLOAT32
z          FLOAT32
intensity  FLOAT32
ring       UINT16
timestamp  FLOAT64
```

而 plain_slam 对未知 LiDAR 的通用格式要求是：

```text
x          FLOAT32
y          FLOAT32
z          FLOAT32
intensity  FLOAT32
timestamp  FLOAT64
```

因此 Pandar 原始 schema 本身已经非常接近直接兼容。

---

## 2.2 Pandar 候选外参

来源：

```text
configs/go2w/hesai_pandarxt16_extrinsics.yaml
```

候选：

```yaml
base_link -> pandarxt16_link
translation_m:
  x: 0.130
  y: 0.015
  z: 0.014
rotation_rpy_deg:
  roll: 0.385
  pitch: 0.905
  yaw: 11.357
```

本次**直接使用，不要求重新标定。**

但是 plain_slam 要的是：

```text
IMU -> LiDAR
```

所以不能直接把 `base -> Pandar` 的 6DoF 原样填进去。

程序必须自动根据：

```text
base -> utlidar_lidar
utlidar_lidar -> utlidar_imu
base -> Pandar candidate
```

计算：

```text
utlidar_imu -> Pandar
```

公式：

```text
T_base_imu = T_base_utlidar * T_utlidar_imu
T_imu_pandar = inverse(T_base_imu) * T_base_pandar
```

按当前仓库数字，计算结果应大致接近：

```text
translation ≈ [-0.13042, 0.02966, 0.09357] m

rotation matrix ≈
[ 0.942376, -0.188276,  0.276548,
  0.196897,  0.980418, -0.003478,
 -0.270478,  0.057729,  0.960994 ]
```

这是**单元测试 sanity reference**，不是让人手动填参数。

配置生成器必须每次从 YAML 源文件自动算，避免未来候选值更新后产生两套真相。

---

## 2.3 IMU 默认来源

第一选择：

```text
/utlidar/imu
```

原因：

- 已是 `sensor_msgs/msg/Imu`；
- 已在当前项目和 Point-LIO 链里使用；
- 不需要新增 Unitree 自定义消息解析作为主路径；
- 与项目现有 L2 sensor stack 共存。

不要依赖 Pandar IMU，因为当前：

```yaml
send_imu_ros: false
```

本次不要为了集成强行开启它。

### IMU 自动健康检查

新增启动前/运行时检查：

- acceleration / gyro 全部 finite；
- `|a|` 不得小于 `0.5 m/s²` 持续多帧；
- `|a|` 不得大于 `50 m/s²` 持续多帧；
- 任意轴绝对值不得出现极端数如 `1e6`；
- timestamp 单调递增；
- 频率建议 >= 50 Hz，目标约 200 Hz；
- 连续坏帧达到阈值时进入 DEGRADED。

**注意：plain_slam 的 `acc_scale` 对标准 ROS `m/s²` IMU 要设为 `1.0`，不要沿用它 Livox 示例里的 `9.79`。**

如果 `/utlidar/imu` 不健康：

- 不要让 AI 停下来要求用户做标定；
- health 标记 `IMU_DEGRADED`；
- 如果仓库/当前 Unitree 消息中能直接读取 body IMU，则启用自动 fallback adapter；
- 若 fallback 不可实现，plain_slam 不得接管运动，现有 `/go2w/odom/fused` 仍正常；
- mapping launcher 应明确退出 LIO 子进程或标记不可用，而不是影响整机控制。

---

# 3. plain_slam_ros2 集成原则

上游：

```text
https://github.com/NaokiAkai/plain_slam_ros2
```

能力：

- ROS2 LIO；
- loosely/tightly coupled；
- GICP loop detection；
- Pose Graph Optimization；
- 3D point cloud map；
- Ubuntu 22.04 / ROS2 Humble。

## 3.1 不修改上游核心算法源码作为第一选择

优先：

```text
ros2_ws/src/plain_slam_ros2/    # 原版/固定版本
ros2_ws/src/go2w_plain_slam_bridge/  # 所有 Go2-W 适配
```

不要把 Pandar 特判大量写进上游 `plain_slam_ros2`。

如果必须修上游 bug：

- patch 必须极小；
- 新增 `patches/plain_slam_ros2/*.patch` 或在文档中记录；
- 不要散落无说明修改。

---

## 3.2 固定依赖版本

AI 实施时：

1. 获取 `plain_slam_ros2` 当前检出的 commit；
2. 把 commit 写入：

```text
configs/go2w/plain_slam_lock.yaml
```

例如：

```yaml
schema_version: 1
plain_slam_ros2:
  repository: https://github.com/NaokiAkai/plain_slam_ros2.git
  commit: <实际检出的完整SHA>
nanoflann:
  minimum_version: "1.6.0"
  preferred_tag: "v1.6.3"
sophus:
  repository: https://github.com/strasdat/Sophus.git
  tag_or_commit: <实际安装时固定值>
```

不要在文档里写一个猜测 SHA。

### License

plain_slam_ros2 当前许可证是**学术/个人免费，商业使用需要作者书面授权**。

AI 必须新增：

```text
docs/go2w/plain_slam_license_note.md
```

并在主实施 README 中提醒。

---

# 4. 最终新增/修改文件清单

下面是 AI 的施工清单。

## 4.1 新增 ROS2 适配包

新增：

```text
ros2_ws/src/go2w_plain_slam_bridge/
├── CMakeLists.txt
├── package.xml
├── include/go2w_plain_slam_bridge/
│   ├── pointcloud_utils.hpp
│   ├── transform_utils.hpp
│   └── occupancy_grid.hpp
├── src/
│   ├── pandar_slam_adapter.cpp
│   ├── plain_slam_odom_adapter.cpp
│   ├── pointcloud_to_occupancy.cpp
│   └── plain_slam_health_monitor.cpp
└── launch/
    └── plain_slam_go2w.launch.py
```

职责分别为：

### `pandar_slam_adapter.cpp`

输入：

```text
/hesai/pandarxt16/points_raw
```

输出：

```text
/go2w/slam/pandar_points
```

职责：

- 验证 PointCloud2 schema；
- 过滤 NaN/Inf；
- 过滤过近/过远点；
- 自动检查每点时间戳；
- 必要时自动归一化时间；
- 输出 plain_slam 需要的精简字段：
  `x,y,z,intensity,timestamp`；
- 保持 SensorDataQoS + keep_last(1)；
- 不做坐标旋转；外参交给 plain_slam 参数；
- 不把候选 Pandar TF 发布成正式 TF。

### `plain_slam_odom_adapter.cpp`

输入：

```text
/go2w/slam/imu_odom_raw
```

或者 plain_slam 的 raw odom topic。

输出：

```text
/go2w/slam/odom_base
/go2w/slam/base_pose
```

职责：

- plain_slam 内部姿态是 IMU frame；
- 根据已知 `base -> utlidar_imu` 自动变换成 base pose；
- 只发布 topic；默认 `publish_tf=false`，避免和现有 TF owner 冲突；
- frame 保持独立 `pslam_odom`；
- 不发布 `/go2w/odom/fused`。

### `pointcloud_to_occupancy.cpp`

输入：

```text
/go2w/slam/aligned_scan
/go2w/slam/odom_base
```

输出：

```text
/go2w/slam/map_2d
/go2w/slam/map_debug_cloud   # 可选
```

职责：

- 不是简单把 3D 点云压扁；
- 必须通过 scan + robot pose 做 2D ray tracing；
- 生成真正包含 `free / occupied / unknown` 的 OccupancyGrid；
- 供 FrontierExtractor 使用。

### `plain_slam_health_monitor.cpp`

订阅：

```text
Pandar raw / adapted
IMU
plain_slam odom
aligned cloud
map_2d
```

输出：

```text
/go2w/slam/health          diagnostic_msgs/DiagnosticArray
/go2w/slam/ready           std_msgs/Bool
```

`ready=true` 只代表 mapping pipeline 可用，**不代表安全授权/运动授权。**

---

## 4.2 新增配置

新增：

```text
configs/go2w/plain_slam.yaml
configs/go2w/plain_slam_lio_params.yaml
configs/go2w/plain_slam_slam_params.yaml
configs/go2w/plain_slam_bridge.yaml
configs/go2w/plain_slam_lock.yaml
```

同时生成运行时配置：

```text
runtime/go2w/plain_slam/generated_lio_3d_config.yaml
runtime/go2w/plain_slam/generated_lio_3d_params.yaml
runtime/go2w/plain_slam/generated_slam_3d_config.yaml
runtime/go2w/plain_slam/generated_slam_3d_params.yaml
```

运行时生成文件不要手工维护。

---

## 4.3 新增配置生成器

新增：

```text
scripts/go2w/generate_plain_slam_pandar_config.py
```

输入：

```text
configs/go2w/official_reference.yaml
configs/go2w/hesai_pandarxt16_extrinsics.yaml
configs/go2w/plain_slam.yaml
configs/go2w/plain_slam_lio_params.yaml
configs/go2w/plain_slam_slam_params.yaml
```

自动派生：

- `T_base_imu`；
- `T_imu_pandar`；
- plain_slam rotation matrix；
- runtime YAML；
- provenance JSON/YAML。

输出：

```text
runtime/go2w/plain_slam/config_provenance.json
```

内容必须包含：

```json
{
  "pandar_extrinsic_source": "configs/go2w/hesai_pandarxt16_extrinsics.yaml",
  "pandar_extrinsic_status": "candidate_unconfirmed",
  "used_for": "mapping_assist_only",
  "authorizes_motion": false,
  "authorizes_safety": false,
  "derived_imu_to_pandar": {...}
}
```

---

## 4.4 新增启动脚本

新增：

```text
scripts/go2w/start_plain_slam_mapping.sh
scripts/go2w/stop_plain_slam_mapping.sh
scripts/go2w/check_plain_slam_mapping.sh
```

以及可选：

```text
scripts/go2w/record_plain_slam_debug_bag.sh
```

---

## 4.5 新增 SpatialProvider

新增：

```text
app/spatial/plain_slam_spatial_provider.py
```

参考：

```text
app/spatial/rtabmap_spatial_provider.py
```

输入：

```text
/go2w/slam/map_2d       nav_msgs/OccupancyGrid
/go2w/slam/odom_base    nav_msgs/Odometry
```

输出项目内部：

```text
SpatialMapSnapshot
SpatialPose
FrontierCandidate
```

它必须实现 `SpatialProvider` protocol：

```python
quality()
get_pose()
get_map()
get_frontiers()
camera_point_to_spatial()
```

并实现：

```python
spin_once()
close()
transform_provenance()
health()
```

---

## 4.6 修改语义探索入口

修改：

```text
scripts/go2w/run_semantic_exploration.py
```

### 新 CLI

新增：

```text
--spatial-provider {camera,rtabmap,plain_slam}
```

默认：

```text
camera
```

保留旧：

```text
--rtabmap
```

作为兼容 alias，不破坏已有脚本。

可以额外支持：

```text
--plain-slam
```

作为：

```text
--spatial-provider plain_slam
```

的 alias。

### provider 初始化

逻辑改为：

```python
if args.spatial_provider == "plain_slam":
    camera_provider = PlainSlamSpatialProvider(
        enable_ros=True,
        map_topic="/go2w/slam/map_2d",
        odom_topic="/go2w/slam/odom_base",
        fallback=CameraLocalSpatialProvider(...),
    )
elif args.spatial_provider == "rtabmap":
    ...
else:
    ...
```

### frontier 逻辑去 RTAB-Map 特判

当前代码有：

```python
if args.rtabmap and camera_provider is not None:
    ...
```

改为 provider-generic：

```python
if camera_provider is not None and hasattr(camera_provider, "get_map"):
    spin = getattr(camera_provider, "spin_once", None)
    if spin:
        spin()
    map_snap = camera_provider.get_map()
    if map_snap is not None:
        pose = camera_provider.get_pose()
        frontiers = frontier_extractor.extract(map_snap, pose)
```

之后才 fallback 到 BEV。

也就是说优先级：

```text
plain_slam / RTABMap metric map
        ↓ unavailable
D435 LightweightDepthBEV
        ↓ unavailable
relative camera frontier
```

不要让 plain_slam 模式仍然优先使用 D435 BEV。

---

# 5. Pandar PointCloud Adapter 详细设计

## 5.1 输入/输出参数

`configs/go2w/plain_slam_bridge.yaml`：

```yaml
pandar_adapter:
  ros__parameters:
    input_topic: /hesai/pandarxt16/points_raw
    output_topic: /go2w/slam/pandar_points
    expected_input_frame: pandarxt16_link_unvalidated

    range_min_m: 0.30
    range_max_m: 60.0

    drop_nan: true
    drop_inf: true

    timestamp_mode: auto
    scan_period_s: 0.10
    absolute_stamp_tolerance_s: 5.0
    timestamp_fallback: linear_scan

    self_filter_enabled: false

    output_frame: pandarxt16_link_unvalidated
```

### 为什么 `self_filter_enabled: false`

仓库已有 Pandar 自遮挡候选 AABB，但它同样是未正式验证的。

第一版为了不误删大量近距离环境点，默认关闭大体积 self filter，只做：

- zero point；
- NaN/Inf；
- min/max range。

后续如果项目现有 self filter 明显可靠，可以配置开启，但不是本次完成条件。

---

## 5.2 timestamp 自动策略

Hesai 官方 driver 本身提供 `timestamp: Float64`，所以正常情况**原样保留**。

Adapter 必须对每帧做轻量检查：

### 情况 A：绝对时间

如果：

```text
median(point.timestamp)
```

和：

```text
msg.header.stamp
```

在允许范围内（例如 5 秒），判定为：

```text
ABSOLUTE_SECONDS
```

原样输出。

### 情况 B：相对 scan 时间

如果：

```text
abs(timestamp) < 10
max(timestamp)-min(timestamp) < 0.2s
```

判定为相对时间：

```text
output_timestamp = header_start + point_relative_timestamp
```

### 情况 C：timestamp 全 0 / 非法

自动 fallback：

```text
t_i = header_stamp + scan_period * i / (N-1)
```

并 health 标记：

```text
TIMESTAMP_SYNTHETIC
```

这不是高精度，但符合本项目“不要求人工 PTP/标定”的目标，并且比完全没有 per-point time 更好。

### 情况 D：奇怪的大值但单调

不要猜单位后静默使用。

- 如果可自动识别 ns/us/ms，转换成秒；
- 否则 health 标记 error；
- 仍然保留现有运动链；
- mapping LIO 可停止，但不能影响 robot control。

---

## 5.3 Adapter 输出字段必须精确

输出 PointCloud2 只建立：

```text
x          FLOAT32
y          FLOAT32
z          FLOAT32
intensity  FLOAT32
timestamp  FLOAT64
```

不需要 `ring`。

`point_step`、offset、endianness 要正确。

每帧保留原始 header stamp 和 frame_id。

---

# 6. 自动生成 plain_slam 配置

## 6.1 LIO config

运行时生成类似：

```yaml
lio_3d_node:
  ros__parameters:
    lidar_type: "hesai_pandarxt16"
    use_as_localizer: false
    map_cloud_dir: "/tmp/go2w_plain_slam/"
    param_files_dir: "/ABS/PATH/runtime/go2w/plain_slam"

    pointcloud_topic: "/go2w/slam/pandar_points"
    imu_topic: "/utlidar/imu"

    imu_pose_topic: "/go2w/slam/imu_pose_raw"
    imu_odom_topic: "/go2w/slam/imu_odom_raw"
    lio_map_cloud_topic: "/go2w/slam/lio_map_cloud"
    aligned_scan_cloud_topic: "/go2w/slam/aligned_scan"
    deskewed_scan_cloud_topic: "/go2w/slam/deskewed_scan"

    odom_frame: "pslam_odom"
    imu_frame: "pslam_imu"
```

**必须使用隔离 frame**：

```text
pslam_odom -> pslam_imu
```

因为上游 `lio_3d_node` 会自己广播 `odom_frame -> imu_frame`。

不要设成现有 `odom -> base_link`，否则可能和已有 TF owner 冲突。

---

## 6.2 LIO 参数推荐初值

`configs/go2w/plain_slam_lio_params.yaml` 作为模板：

```yaml
initial_pose:
  translation: [0.0, 0.0, 0.0]
  rotation_matrix:
    [1.0, 0.0, 0.0,
     0.0, 1.0, 0.0,
     0.0, 0.0, 1.0]

# 实际内容由 generate_plain_slam_pandar_config.py 覆盖生成
extrinsics:
  imu_to_lidar:
    translation: [AUTO, AUTO, AUTO]
    rotation_matrix: [AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO, AUTO]

imu_params:
  acc_scale: 1.0

keyframe:
  distance_th: 1.0
  angle_th: 20.0
  min_active_points_rate: 0.80

scan_cloud_preprocess:
  clip_range: 0.35
  filter_size: 0.15

normal_map:
  num_keyframes: 15
  filter_size: 0.15
  num_normal_points: 10
  normal_eigen_val_thresh: 0.2

estimator:
  use_loose_coupling: true
  huber_delta: 0.1
  num_max_iteration: 5
  num_max_matching_points: 8000
  max_correspondence_dist: 1.5
  convergence_th: 0.1
```

### 设计理由

#### `acc_scale: 1.0`

ROS Imu 已经是标准 `m/s²`。

#### `use_loose_coupling: true`

本项目明确不做新一轮精细 LiDAR-IMU 标定，因此第一版用较宽容的 loose coupling 做 best-effort。

后续如果自动健康指标证明 tight coupling 更好，再做配置开关；不要把 tight coupling 作为本次前置要求。

#### `clip_range: 0.35`

Pandar 用于室内/近距离导航，plain_slam 默认 1 m 太激进。

#### voxel/filter `0.15`

Pandar 约 10 Hz、点数较多，0.15 m 是计算量和室内几何之间的折中起点。

### 所有阈值都必须配置化

不要硬编码到 C++。

---

# 7. SLAM 后端配置

上游 SLAM 默认关键帧距离 10 m 对室内机器狗过稀。

建议：

```yaml
preprocess:
  accumulation_cycle: 2
  scan_cloud_filter_size: 0.15

keyframe:
  distance_th: 1.5
  angle_th: 25.0
  min_local_map_matching_rate: 0.70

loop_closure:
  num_loop_candidates: 10
  loop_detection_distance_thresh: 12.0
  loop_detection_height_thresh: 3.0
  error_average_th: 0.8
  active_points_rate_th: 0.45
  gicp_max_iteration: 8
  gicp_epsilon: 0.3
  gicp_max_correspondence_dist: 3.0
  gicp_huber_delta: 0.5

pose_graph:
  huber_delta: 0.1
```

第一版目标：

- 小到中型室内环境；
- 稳定实时优先；
- 不是追求论文 benchmark 极限精度。

这些参数全部放配置，不要写死。

---

## 7.1 SLAM topic 重映射

生成：

```yaml
slam_3d_node:
  ros__parameters:
    map_cloud_dir: "/tmp/go2w_plain_slam/"
    param_files_dir: "/ABS/PATH/runtime/go2w/plain_slam"

    imu_pose_topic: "/go2w/slam/imu_pose_raw"
    deskewed_scan_cloud_topic: "/go2w/slam/deskewed_scan"

    filtered_map_cloud_topic: "/go2w/slam/map_3d"
    graph_nodes_topic: "/go2w/slam/pose_graph_nodes"
    odom_edges_topic: "/go2w/slam/pose_graph_odom_edges"
    loop_edges_topic: "/go2w/slam/pose_graph_loop_edges"
    graph_poses_topic: "/go2w/slam/pose_graph_poses"

    map_frame: "pslam_map"
```

注意：

- `pslam_map`、`pslam_odom` 与现有 nav TF 隔离；
- 第一期不要尝试让 plain_slam 发布正式 `map -> odom -> base_link`；
- semantic exploration 使用 provider 自己的 pslam map/pose 坐标即可。

---

# 8. plain_slam 启动方式

不要修改上游 `slam_3d.launch.py` 来硬编码 Go2-W 路径。

在：

```text
ros2_ws/src/go2w_plain_slam_bridge/launch/plain_slam_go2w.launch.py
```

直接启动：

```python
Node(
    package="plain_slam_ros2",
    executable="lio_3d_node",
    name="go2w_plain_slam_lio",
    parameters=[generated_lio_config, {"param_files_dir": generated_param_dir}],
)

Node(
    package="plain_slam_ros2",
    executable="slam_3d_node",
    name="go2w_plain_slam_slam",
    parameters=[generated_slam_config, {"param_files_dir": generated_param_dir}],
)
```

同时启动：

```text
pandar_slam_adapter
plain_slam_odom_adapter
pointcloud_to_occupancy
plain_slam_health_monitor
```

---

# 9. 3D → 2D OccupancyGrid：必须按下面方式实现

这是“地图真正辅助导航”的关键。

## 9.1 禁止的错误实现

不要只做：

```text
3D global cloud
   ↓ 删除 z
2D occupied cells
```

因为那样只能知道“哪里有障碍”，无法知道：

```text
free
unknown
```

没有 unknown/free，就无法做真正 frontier extraction。

---

## 9.2 正确做法：实时 scan ray tracing

输入：

```text
/go2w/slam/aligned_scan
/go2w/slam/odom_base
```

`aligned_scan` 已经在 LIO 世界/里程计 frame 中。

每个有效激光点：

```text
sensor origin ---------> endpoint
       free cells          occupied
```

使用 Bresenham / DDA 2D ray casting：

- ray 穿过的 cell：更新 free log-odds；
- 障碍 endpoint：更新 occupied log-odds；
- 从未被 ray 穿过的 cell：unknown。

---

## 9.3 地图默认参数

```yaml
occupancy:
  ros__parameters:
    map_frame: pslam_odom
    resolution_m: 0.10
    width_m: 80.0
    height_m: 80.0
    origin_x_m: -40.0
    origin_y_m: -40.0

    publish_rate_hz: 2.0
    max_scan_update_hz: 10.0

    min_range_m: 0.35
    max_range_m: 25.0

    ground_mode: auto
    ground_search_radius_m: 3.0
    ground_histogram_bin_m: 0.05

    floor_tolerance_m: 0.08
    obstacle_min_height_m: 0.10
    obstacle_max_height_m: 1.60

    log_odds_hit: 0.85
    log_odds_miss: -0.40
    log_odds_min: -2.0
    log_odds_max: 3.5

    free_probability_threshold: 0.35
    occupied_probability_threshold: 0.65

    angular_ray_bin_deg: 0.5
```

80 m × 80 m @ 0.1 m = 800 × 800 = 640k cell，可接受。

---

## 9.4 自动地面高度估计

因为用户不想手工测 base-ground 高度，本次必须自动估计。

每隔约 1 秒：

1. 取机器人当前 3 m 半径内的 aligned scan 点；
2. 对 z 做 5 cm histogram；
3. 在低位候选中选最密集水平层作为 ground candidate；
4. 对 ground_z 做 EMA；
5. 如果检测失败，沿用上一帧；
6. 尚无上一帧时，使用点云 z 的低百分位数作为临时值。

**不要因为 ground 估计失败就停止现有机器人运动链。**

health 中标记：

```text
GROUND_ESTIMATE_OK
GROUND_ESTIMATE_FALLBACK
```

---

## 9.5 斜坡/桌面等

第一版不做完整 3D traversability。

只需要：

- 地板不标障碍；
- 墙、椅子、桌腿、柜体等在 `ground+0.10 ~ ground+1.60m` 内产生障碍；
- 天花板不投影成障碍。

这已经足够让 3D LiDAR 地图显著改善 frontier 与路径成本。

---

# 10. Odom Adapter 设计

## 10.1 为什么要转换 IMU pose → base pose

plain_slam 输出核心姿态对应 IMU frame。

项目的导航/空间逻辑更希望看到机器狗 base pose。

已知：

```text
base_link -> utlidar_lidar -> utlidar_imu
```

所以：

```text
T_world_base = T_world_imu * inverse(T_base_imu)
```

Adapter 自动从 `official_reference.yaml` 读取，不手填。

输出：

```text
/go2w/slam/odom_base
```

消息：

```text
nav_msgs/msg/Odometry
header.frame_id = pslam_odom
child_frame_id = base_link_mapping_assist
```

推荐 child frame 用：

```text
base_link_mapping_assist
```

避免暗示它是正式 `base_link` TF owner。

不广播 TF，默认：

```yaml
publish_tf: false
```

---

# 11. PlainSlamSpatialProvider 详细要求

文件：

```text
app/spatial/plain_slam_spatial_provider.py
```

可以复用 `RtabmapSpatialProvider` 的绝大部分 OccupancyGrid / Odometry 解析方式，但 source/quality/provenance 要正确。

## 11.1 map callback

把 OccupancyGrid 转为：

```python
SpatialMapSnapshot(
    revision=...,
    resolution_m=...,
    origin=(...),
    width=...,
    height=...,
    free=[...],
    occupied=[...],
    unknown=[...],
    source="plain_slam_pandarxt16",
    provenance={
        "frame_id": msg.header.frame_id,
        "mapping_mode": "mapping_assist",
        "pandar_extrinsic_status": "candidate_unconfirmed",
    },
)
```

## 11.2 odom callback

生成：

```python
SpatialPose(
    x=...,
    y=...,
    yaw=...,
    frame_id="pslam_odom",
    source="plain_slam_pandarxt16_odom",
    ...
)
```

## 11.3 quality

项目目前主要质量常量是 camera/RGBD 语义。

不要滥用 `METRIC_RGBD` 表示 LiDAR。

建议在：

```text
app/spatial/models.py
```

新增：

```python
SPATIAL_QUALITY_METRIC_LIDAR = "METRIC_LIDAR"
```

并确保已有逻辑不会因为未知 quality 崩溃。

如果不想扩大模型变更，可以先返回项目现有通用 metric quality，但 provenance 必须明确 `source=plain_slam_pandarxt16`。

优先方案：新增 `METRIC_LIDAR`。

---

## 11.4 camera_point_to_spatial

Semantic object 仍来自 RGB/D435，因此 provider 仍需要把 camera point 变到 pslam spatial frame。

本次不要要求新相机标定。

优先级：

1. 如果 TF2 能得到 `pslam_odom <- d435_color_optical_frame`，使用 TF2；
2. 否则沿用项目现有 `camera_point_to_map()` nominal fallback；
3. 用当前 `SpatialPose` 做平面变换；
4. provenance 标记：

```text
transform_source: nominal_extrinsic_fallback
```

不要因为这一步不是精确 TF 就阻塞 LiDAR frontier 建图。

---

# 12. 与现有 SemanticNavigation V2 的融合方式

现有结构已经有：

```text
PlaceGraph
SemanticObjectMap
SemanticEntityGraph
SpatialMemory
FrontierExtractor
SpatialSearchReasoner
LongTermGoalSelector
SemanticRoutePlanner
LocalGoalExecutor
```

不要重写这些。

只替换/新增空间数据来源：

```text
旧：RTABMap / D435 BEV
新：plain_slam Pandar metric map
```

### 最终 priority

当：

```bash
--spatial-v2 --spatial-provider plain_slam
```

时：

```text
PlainSlam map+pose
   ↓
FrontierExtractor
   ↓
SemanticRoutePlanner
   ↓
LongTermGoalSelector
   ↓
LocalGoalExecutor
```

如果 plain_slam map 暂时断流：

```text
D435 BEV fallback
```

如果 D435 也不可用：

```text
CameraLocalSpatialProvider fallback
```

不得因为 SLAM 掉线直接让整个 high-level semantic loop crash。

---

# 13. Nav2 融合边界

本次实现两个层级。

## Level A：必须完成

`/go2w/slam/map_2d` 被：

- FrontierExtractor 使用；
- SemanticRoutePlanner 使用；
- Web/diagnostic 可视化使用。

这已经实现“3D 建图辅助导航/探索”。

## Level B：实现接口，但默认不开启 map authority

为 Nav2 增加可选参数：

```text
use_plain_slam_map:=false
```

如果未来开启，可让 Nav2 使用：

```text
/go2w/slam/map_2d
```

作为规划地图/障碍输入。

但本次默认：

```text
false
```

原因：

- Pandar 外参仍是 candidate；
- plain_slam odom 未接管 motion localization；
- 不能把 mapping_assist 偷偷升级成 safety authority。

**AI 不得为了“看起来集成完整”而默认让它接管 Nav2 collision safety。**

---

# 14. 启动脚本行为

新增：

```text
scripts/go2w/start_plain_slam_mapping.sh
```

## 14.1 默认启动顺序

```text
1. source ROS2 Humble
2. source ros2_ws/install/setup.bash
3. generate_plain_slam_pandar_config.py
4. 检查 /hesai/pandarxt16/points_raw 是否已有 publisher
5. 若没有，则启动现有 start_hesai_pandarxt16.sh
6. 等待 Pandar topic，超时明确退出 mapping launcher
7. 检查 /utlidar/imu
8. 启动 go2w_plain_slam_bridge + plain_slam nodes
9. 等待 /go2w/slam/ready
10. 打印 topic/health 摘要
```

## 14.2 不重复启动 Hesai driver

如果：

```bash
ros2 topic info /hesai/pandarxt16/points_raw
```

已有 publisher，直接复用。

不要创建第二个 Hesai driver。

---

## 14.3 CLI

建议：

```bash
bash scripts/go2w/start_plain_slam_mapping.sh
```

可选：

```bash
bash scripts/go2w/start_plain_slam_mapping.sh --rviz
bash scripts/go2w/start_plain_slam_mapping.sh --no-start-hesai
bash scripts/go2w/start_plain_slam_mapping.sh --record
bash scripts/go2w/start_plain_slam_mapping.sh --debug
```

不要要求用户传外参数字。

---

# 15. 一键探索启动方式

修改：

```text
scripts/go2w/start_semantic_exploration.sh
```

增加可选环境变量/参数：

```text
GO2W_SPATIAL_PROVIDER=plain_slam
GO2W_START_PLAIN_SLAM=1
```

逻辑：

```text
如果 GO2W_START_PLAIN_SLAM=1：
    若 /go2w/slam/ready 不存在：
        启动 start_plain_slam_mapping.sh

然后 run_semantic_exploration.py：
    --spatial-v2
    --spatial-provider plain_slam
```

最终用户可以：

```bash
GO2W_START_PLAIN_SLAM=1 \
GO2W_SPATIAL_PROVIDER=plain_slam \
bash scripts/go2w/start_semantic_exploration.sh
```

如果项目原脚本本身需要 target 参数，保持原参数传递方式，不破坏现有 CLI。

---

# 16. Health / 自动降级

新增 health 状态机：

```text
STARTING
  ↓
POINTCLOUD_OK
  ↓
IMU_OK
  ↓
LIO_OK
  ↓
MAP3D_OK
  ↓
MAP2D_OK
  ↓
READY_MAPPING_ASSIST
```

可能降级：

```text
TIMESTAMP_SYNTHETIC
EXTRINSIC_CANDIDATE
IMU_DEGRADED
LIO_STALE
MAP2D_STALE
```

`/go2w/slam/ready=true` 条件建议：

- Pandar adapted cloud 最近 1 秒内有数据；
- IMU 最近 0.2 秒内有数据；
- plain_slam odom 最近 1 秒内有数据；
- aligned scan 最近 1 秒内有数据；
- OccupancyGrid 最近 2 秒内有数据。

但永远附带状态：

```text
mode = MAPPING_ASSIST
motion_authorized = false
safety_authorized = false
```

---

# 17. 自动质量判断，不要求人工专门操控

用户不希望为了验收做专门标定动作。

因此测试应依赖：

- 机器人静止时数据；
- 用户正常探索时自然产生的运动；
- rosbag / topic 自动统计。

## 17.1 静止自动检查

机器狗不动时即可做：

### PointCloud

- 8~12 Hz 左右；
- 点数非 0；
- x/y/z finite；
- timestamp span 合理；
- timestamp 单调/基本单调。

### IMU

- 频率足够；
- finite；
- acceleration magnitude 合理。

### LIO

静止 30 秒内：

- pose 不得出现 NaN；
- translation 不能几秒内爆到几十米；
- z drift 不能持续无限发散；
- quaternion normalized。

这些均由脚本自动计算。

---

## 17.2 正常探索过程自动观察

不要求用户执行特殊动作。

在普通导航/探索运行期间后台计算：

```text
plain_slam displacement
vs
/go2w/odom/fused displacement
```

仅做诊断，不融合。

窗口例如 10 秒：

- 位移比例；
- yaw 增量差；
- 突跳；
- z 漂移；
- odom stale。

输出：

```text
/go2w/slam/health
runtime/go2w/plain_slam/quality.json
```

### 自动评级

例如：

```text
GOOD
USABLE_MAPPING_ONLY
DEGRADED
FAILED
```

第一版无论 `GOOD` 还是 `USABLE_MAPPING_ONLY`，都不自动替换 `/go2w/odom/fused`。

---

# 18. 自动测试文件

新增：

```text
tests/test_plain_slam_config_generation.py
tests/test_plain_slam_extrinsic_derivation.py
tests/test_plain_slam_spatial_provider.py
tests/test_plain_slam_occupancy_projection.py
tests/test_plain_slam_timestamp_policy.py
```

如项目 ROS package tests 独立，则放对应 package `test/`。

---

## 18.1 外参派生单测

使用当前仓库 fixture，断言：

```text
T_imu_pandar.translation
≈ [-0.13042, 0.02966, 0.09357]
```

rotation matrix 大致接近本计划前述值。

容差：

```text
1e-4 ~ 1e-3
```

同时断言：

```text
confirmed == false
authorizes_motion == false
```

生成器不得改源 YAML。

---

## 18.2 Timestamp 单测

覆盖：

1. absolute seconds；
2. relative 0~0.1s；
3. zero timestamps；
4. NaN；
5. microseconds；
6. nanoseconds；
7. non-monotonic outlier。

输出必须明确 mode，不允许 silent guessing。

---

## 18.3 OccupancyGrid 单测

构造模拟走廊：

```text
robot at (0,0)
wall endpoints x=3m
```

检查：

- robot→wall ray cells = free；
- wall endpoint = occupied；
- 未扫描区域 = unknown；
- FrontierExtractor 能从 free/unknown 边界得到 frontier。

这是核心验收。

---

## 18.4 SpatialProvider 单测

给 fake OccupancyGrid / Odometry，断言：

```text
get_map() != None
get_pose() != None
get_frontiers() returns candidates
quality == METRIC_LIDAR (如果新增)
provenance source == plain_slam_pandarxt16
```

---

# 19. 构建脚本修改

修改：

```text
scripts/go2w/install_dependencies.sh
scripts/go2w/build_ros2.sh
```

## 19.1 dependencies

确保：

```text
libyaml-cpp-dev
libeigen3-dev
Sophus
nanoflann >= 1.6.0
```

### nanoflann

不要盲信 Ubuntu 22.04 apt 版本满足上游要求。

脚本应：

1. 先检测系统 header/version；
2. 如果 >=1.6 则复用；
3. 否则安装固定兼容版本，例如 `v1.6.3`；
4. 必须幂等。

### Sophus

脚本安装必须幂等：

- 已安装并能 `find_package(Sophus)` 就跳过；
- 否则下载固定 tag/commit、build、install；
- 不要每次运行都重编。

---

## 19.2 build_ros2.sh

保证：

```text
plain_slam_ros2
go2w_plain_slam_bridge
```

被 colcon 编译。

如果 plain_slam source 缺失：

- 给出明确提示；
- 如果本次 AI 采用 git submodule，应执行 submodule init；
- 不允许静默跳过然后启动时才失败。

---

# 20. RViz 配置

新增：

```text
configs/go2w/plain_slam_mapping.rviz
```

Fixed Frame：

```text
pslam_odom
```

显示：

```text
/go2w/slam/map_3d
/go2w/slam/aligned_scan
/go2w/slam/map_2d
/go2w/slam/odom_base
```

颜色建议：

- 3D global map：intensity/height；
- aligned scan：单独颜色；
- OccupancyGrid：默认；
- robot pose/path：单独显示。

不依赖正式 base_link TF 才能打开 RViz。

---

# 21. 地图保存

第一版增加一个薄 wrapper：

```text
scripts/go2w/save_plain_slam_map.sh
```

保存到：

```text
outputs/maps/<timestamp>/
├── map_3d.pcd
├── map_2d.pgm
├── map_2d.yaml
├── pose_graph.json      # 若上游容易导出
└── provenance.json
```

`provenance.json` 必须记录：

```text
Pandar model
plain_slam commit
candidate extrinsic source
candidate_unconfirmed
IMU source
point timestamp mode
mapping_assist_only
```

如果上游 PCD save API 不方便，至少支持通过订阅当前 `/go2w/slam/map_3d` 写 PCD；不要因为 pose graph export 不方便阻塞整个任务。

---

# 22. 日志和资源限制

Pandar 约 10Hz，CPU 端不能无限复制完整全局地图。

## 22.1 高频 topic

使用：

```text
SensorDataQoS
keep_last(1)
```

用于：

- raw/adapted PointCloud；
- aligned scan。

## 22.2 global map

map3d 发布频率低于 scan；不要每帧重复深拷贝超大地图给多个 Python 节点。

SemanticNavigation 只订阅 2D OccupancyGrid，不订阅完整 3D cloud。

## 22.3 地图尺寸

OccupancyGrid 默认 80m 方形固定图，避免动态 resize 在每帧发生。

后续需要更大环境再改配置。

---

# 23. 失败隔离

任何 plain_slam 组件 crash：

```text
不得 kill：
/go2w/odom/fused
motion action server
control arbiter
camera stack
existing safety chain
```

启动脚本不要用一个错误的 `trap` 把整机 ROS graph 全部杀掉。

stop script 只关闭：

```text
go2w_plain_slam_bridge nodes
plain_slam nodes
由本脚本自己启动的 Hesai driver（可追踪 PID 时）
```

如果 Hesai driver 原本已经存在，不得 stop 它。

---

# 24. 旧 Point-LIO 的处理

不要删除：

```text
point_lio_ws/
configs/go2w/point_lio.yaml
现有 Point-LIO scripts
```

但默认不要同时启动两个 LIO backend 去发布同名 topic。

所有 plain_slam topic 使用：

```text
/go2w/slam/*
```

和旧：

```text
/lio/*
```

隔离。

后续可以再做统一 `lio_backend` selector，但不是这次阻塞项。

---

# 25. 推荐配置总表

| 功能 | Topic / Frame | 说明 |
|---|---|---|
| Pandar raw | `/hesai/pandarxt16/points_raw` | 现有 Hesai driver |
| SLAM point input | `/go2w/slam/pandar_points` | 适配后 PointCloud2 |
| IMU input | `/utlidar/imu` | 默认复用现有 IMU |
| plain_slam odom frame | `pslam_odom` | 隔离现有 odom |
| plain_slam imu frame | `pslam_imu` | 隔离 TF |
| raw IMU pose | `/go2w/slam/imu_pose_raw` | plain_slam |
| raw IMU odom | `/go2w/slam/imu_odom_raw` | plain_slam |
| base odom | `/go2w/slam/odom_base` | adapter 输出 |
| aligned scan | `/go2w/slam/aligned_scan` | 世界坐标扫描 |
| deskewed scan | `/go2w/slam/deskewed_scan` | IMU frame 去畸变 scan |
| local LIO cloud | `/go2w/slam/lio_map_cloud` | LIO local map |
| global 3D map | `/go2w/slam/map_3d` | SLAM 输出 |
| 2D nav/explore map | `/go2w/slam/map_2d` | free/occupied/unknown |
| health | `/go2w/slam/health` | DiagnosticArray |
| ready | `/go2w/slam/ready` | mapping readiness |
| existing nav odom | `/go2w/odom/fused` | 保持现状，运动兜底 |

---

# 26. 实施顺序：AI 必须按此顺序施工

## Step 1：创建依赖锁和 vendor plain_slam

- 加入 `plain_slam_ros2`；
- 固定 commit；
- 配好 Sophus/nanoflann；
- 先确保纯 `colcon build` 通过。

## Step 2：实现配置生成器

- 读取官方几何；
- 读取 Pandar candidate；
- 自动算 `imu_to_lidar`；
- 生成 runtime plain_slam YAML；
- 写 provenance；
- 添加 unit tests。

## Step 3：实现 Pandar adapter

- schema 验证；
- timestamp auto；
- filtering；
- 输出 `/go2w/slam/pandar_points`。

## Step 4：实现 plain_slam bringup

- 隔离 frame；
- Go2-W config；
- 启动 LIO + SLAM。

## Step 5：实现 odom adapter

- IMU pose → base pose；
- 不广播冲突 TF；
- 输出 shadow odom。

## Step 6：实现 OccupancyGrid bridge

- aligned scan ray tracing；
- auto ground；
- free/occupied/unknown；
- 输出 `/go2w/slam/map_2d`。

## Step 7：实现 health monitor

- 监控五类 topic；
- 不影响已有 motion stack。

## Step 8：实现 `PlainSlamSpatialProvider`

- map/pose；
- frontiers；
- provenance；
- fallback。

## Step 9：修改 semantic exploration

- provider generic；
- 加 plain_slam CLI；
- 让 metric map 优先于 BEV。

## Step 10：一键脚本

- start/stop/check；
- 可选 RViz；
- 不重复拉起 Hesai。

## Step 11：自动测试

- Python tests；
- C++ build/tests；
- ROS graph smoke test。

## Step 12：文档

新增：

```text
docs/go2w/plain_slam_pandarxt16.md
```

内容至少包括：

- 架构；
- 启动命令；
- topic；
- health；
- limitation；
- license；
- mapping_assist 安全边界。

---

# 27. 自动 smoke test 命令

AI 修改完代码后必须至少执行/准备以下检查。

## 27.1 构建

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

要求：0 build error。

---

## 27.2 配置生成测试

```bash
python3 scripts/go2w/generate_plain_slam_pandar_config.py --check
```

预期输出包含：

```text
Pandar extrinsic: candidate_unconfirmed
Mode: mapping_assist_only
Derived imu_to_pandar: OK
Motion authorization changed: NO
Safety authorization changed: NO
```

---

## 27.3 只检查传感器，不发运动

```bash
bash scripts/go2w/start_plain_slam_mapping.sh
```

然后：

```bash
ros2 topic hz /hesai/pandarxt16/points_raw
ros2 topic hz /go2w/slam/pandar_points
ros2 topic hz /utlidar/imu
ros2 topic hz /go2w/slam/odom_base
ros2 topic hz /go2w/slam/aligned_scan
ros2 topic hz /go2w/slam/map_2d
```

不需要让机器狗执行标定动作。

---

## 27.4 Topic schema

```bash
ros2 topic echo /go2w/slam/ready --once
ros2 topic echo /go2w/slam/health --once
ros2 topic info /go2w/slam/map_2d -v
ros2 topic info /go2w/slam/map_3d -v
```

---

## 27.5 TF 冲突检查

必须确保新增系统没有第二个 owner 广播正式：

```text
odom -> base_link
```

plain_slam 应只拥有：

```text
pslam_odom -> pslam_imu
```

如果发现 plain_slam 输出和现有 owner frame 重名，视为失败，修改配置隔离。

---

# 28. 语义探索验收

启动 mapping 后：

```bash
python3 scripts/go2w/run_semantic_exploration.py \
  --target "测试目标" \
  --spatial-v2 \
  --spatial-provider plain_slam \
  --dry-run-motion \
  --allow-degraded
```

`--dry-run-motion` 用来先验证不发运动命令的完整数据链。

要求日志能看到：

```text
spatial provider = plain_slam
map source = plain_slam_pandarxt16
map revision > 0
pose source = plain_slam_pandarxt16_odom
frontier count >= 0
route planner consumes /go2w/slam/map_2d
```

当环境里已有 free/unknown 边界时，应产生 frontier。

---

# 29. 真实正常探索时的目标行为

用户正常启动探索后，不需要额外做任何“标定动作”。

系统应自然工作：

```text
机器狗开始探索
      ↓
Pandar 每 100ms 扫一帧
      ↓
plain_slam 利用 IMU 去畸变 + 匹配
      ↓
3D map 持续扩展
      ↓
aligned scan 对 2D grid 做 ray tracing
      ↓
已扫描区域成为 free
墙/障碍成为 occupied
未扫描区域仍为 unknown
      ↓
FrontierExtractor 发现 free/unknown 边界
      ↓
SemanticNavigation 给 frontier 加语义优先级
      ↓
SemanticRoutePlanner 计算地图路径成本
      ↓
现有 LocalGoalExecutor / motion stack 执行动作
      ↓
新的区域被 Pandar 扫到
      ↓
地图继续扩展
```

这就是本次最终要实现的闭环。

---

# 30. 完成定义（Definition of Done）

只有同时满足以下条件，AI 才可以声明任务完成。

## 代码层

- [ ] `plain_slam_ros2` 已加入并能编译；
- [ ] `go2w_plain_slam_bridge` 能编译；
- [ ] Pandar adapter 已实现；
- [ ] 自动外参生成已实现；
- [ ] odom adapter 已实现；
- [ ] 3D scan → OccupancyGrid ray tracing 已实现；
- [ ] health monitor 已实现；
- [ ] `PlainSlamSpatialProvider` 已实现；
- [ ] `run_semantic_exploration.py` 支持 plain_slam provider；
- [ ] start/stop/check 脚本齐全。

## 数据层

- [ ] `/go2w/slam/pandar_points` 有数据；
- [ ] `/go2w/slam/odom_base` 有数据；
- [ ] `/go2w/slam/aligned_scan` 有数据；
- [ ] `/go2w/slam/map_3d` 能持续更新；
- [ ] `/go2w/slam/map_2d` 有 free/occupied/unknown；
- [ ] `/go2w/slam/health` 有明确状态。

## 集成层

- [ ] FrontierExtractor 能读取 plain_slam map；
- [ ] SemanticRoutePlanner 能读取 plain_slam map；
- [ ] provider 断流后能 fallback；
- [ ] `/go2w/odom/fused` 未被替换；
- [ ] motion safety authorization 未被篡改；
- [ ] Pandar candidate extrinsic 仍标记 `candidate_unconfirmed`。

## 人工要求

- [ ] 没有要求用户重新标定 Pandar；
- [ ] 没有要求用户配置 PTP/GPS 才能运行；
- [ ] 没有要求用户执行专门校准运动；
- [ ] 用户只需正常启动传感器/探索即可让地图自然生长。

---

# 31. AI 实施时的禁止事项

以下行为直接视为不合格实现：

1. **不要**为了省事把 Pandar candidate 改成 `confirmed=true`；
2. **不要**把 `authorizes_motion` 改成 true；
3. **不要**默认让 plain_slam 发布现有 `odom -> base_link`；
4. **不要**覆盖 `/go2w/odom/fused`；
5. **不要**删除旧 Point-LIO；
6. **不要**只把 3D cloud 压成 occupied bitmap，却没有 free/unknown；
7. **不要**要求用户手动输入 `imu_to_lidar` 数字；
8. **不要**要求用户重新量安装位置；
9. **不要**把没有 PTP 当成完全不能运行；
10. **不要**因为 SLAM crash 就停止整机已有 motion/safety stack；
11. **不要**在 Python 中每帧复制巨大 global 3D map 给 SemanticNavigation；
12. **不要**让多个 Hesai driver 同时监听同一设备；
13. **不要**硬编码绝对项目路径；
14. **不要**静默吞 timestamp/IMU 错误；
15. **不要**把 mapping readiness 与 safety authorization 混为一谈。

---

# 32. 推荐实现细节：自动外参计算伪代码

配置生成器只需要 Python `math` + `yaml`，无需 SciPy。

```python
# 伪代码
T_base_l2 = pose_to_matrix(
    official["frames"]["base_to_lidar"]["translation_m"],
    official["frames"]["base_to_lidar"]["rotation_rpy_rad"],
)

T_l2_imu = pose_to_matrix(
    official["frames"]["lidar_to_lidar_imu"]["translation_m"],
    official["frames"]["lidar_to_lidar_imu"]["rotation_rpy_rad"],
)

T_base_imu = T_base_l2 @ T_l2_imu

T_base_pandar = pose_to_matrix(
    [cand_x, cand_y, cand_z],
    deg2rad([cand_roll, cand_pitch, cand_yaw]),
)

T_imu_pandar = inverse_rigid(T_base_imu) @ T_base_pandar
```

`inverse_rigid`：

```python
R_inv = R.T
t_inv = -R.T @ t
```

写入 plain_slam：

```yaml
extrinsics:
  imu_to_lidar:
    translation: [tx, ty, tz]
    rotation_matrix: [r00,r01,r02,r10,r11,r12,r20,r21,r22]
```

同时生成单元测试，防止矩阵方向写反。

---

# 33. 推荐实现细节：Occupancy ray tracing 伪代码

```cpp
for each aligned_scan:
    pose = latest_base_pose;
    ground_z = ground_estimator.update(scan, pose);

    rays = polar_downsample(scan, angular_bin_deg=0.5);

    for each point p in rays:
        if !finite(p): continue;

        range = hypot(p.x - sensor_x, p.y - sensor_y);
        if range < min_range || range > max_range: continue;

        relative_h = p.z - ground_z;

        // Traversed space is observed free.
        raytrace_free(sensor_xy, p.xy);

        // Only robot-height geometry becomes occupied endpoint.
        if obstacle_min_height <= relative_h <= obstacle_max_height:
            update_hit(p.xy);

    publish OccupancyGrid at configured rate;
```

对地板点：

- 可以用于 ray clearing；
- 不作为 occupied endpoint。

对天花板点：

- 不作为 occupied endpoint。

---

# 34. 推荐实现细节：Provider fallback

```python
class PlainSlamSpatialProvider:
    def get_map(self):
        if self._plain_slam_map_is_fresh():
            return self._map
        return None

    def get_frontiers(self):
        if self._map is not None and self._pose is not None:
            result = self.frontier_extractor.extract(self._map, self._pose)
            if result:
                return result
        return self.fallback.get_frontiers()
```

注意：

`get_pose()` 在 map 存在时优先用 plain_slam pose，使 frontier 坐标和 map 坐标一致；不要把 pslam map 和 `/go2w/odom/fused` pose 直接混在同一个 frame 中。

现有 `/go2w/odom/fused` 仍用于底层运动执行，不等于 SpatialProvider 必须用它做地图 pose。

---

# 35. 后续可选升级（本次不要阻塞在这里）

以下可以预留接口，但**不作为本次完成条件**：

## 35.1 plain_slam odom 自动择优融合

未来若后台质量持续 GOOD，可做：

```text
wheel odom + pslam yaw/translation
        ↓
EKF / selector
        ↓
/go2w/odom/fused_v2
```

但必须单独验证后才能切换。

## 35.2 Nav2 正式使用 2D map

未来：

```text
pslam map -> map server / costmap
```

再配标准：

```text
map -> odom -> base_link
```

这需要更严格的 frame authority 策略，本次不要强行做。

## 35.3 OctoMap / voxel 3D traversability

未来可以把 `/go2w/slam/map_3d` 输入：

- OctoMap；
- Voxblox；
- elevation map；
- 2.5D traversability。

当前先完成 2D ray-traced map。

## 35.4 自动在线外参微调

未来可以做在线 LiDAR-IMU extrinsic optimization，但用户明确不想进行人工标定，所以不是本次目标。

---

# 36. 最终用户体验

最终用户不应该需要理解 plain_slam 内部配置。

预期操作：

## 只开 3D 建图

```bash
bash scripts/go2w/start_plain_slam_mapping.sh --rviz
```

看到：

```text
[OK] Hesai PandarXT-16: receiving
[OK] Point schema: compatible
[OK] Timestamp: absolute/auto
[OK] IMU: /utlidar/imu
[WARN] Pandar extrinsic: candidate_unconfirmed, mapping-assist only
[OK] plain_slam LIO: running
[OK] 3D map: updating
[OK] 2D occupancy: updating
[OK] Spatial mapping ready
[INFO] /go2w/odom/fused remains motion authority
```

## 开语义探索 + 实时 3D 建图

```bash
GO2W_START_PLAIN_SLAM=1 \
GO2W_SPATIAL_PROVIDER=plain_slam \
bash scripts/go2w/start_semantic_exploration.sh <保留项目原有参数>
```

机器狗正常探索即可，**不需要额外“为了 SLAM”做一套人工标定动作。**

---

# 37. 参考资料

## 当前项目

- Repository / branch：
  `https://github.com/BROVVV/robot_scene_demo/tree/robot-go2w-deployment-20260828`
- Pandar driver config：
  `configs/go2w/hesai_pandarxt16.yaml`
- Pandar candidate extrinsics：
  `configs/go2w/hesai_pandarxt16_extrinsics.yaml`
- Unitree official geometry：
  `configs/go2w/official_reference.yaml`
- Semantic exploration：
  `scripts/go2w/run_semantic_exploration.py`
- Existing RTABMap provider：
  `app/spatial/rtabmap_spatial_provider.py`

## plain_slam_ros2

- `https://github.com/NaokiAkai/plain_slam_ros2`
- Required generic PointCloud2 fields：
  `x/y/z/intensity Float32 + timestamp Float64`
- LIO config：
  `config/lio_3d_config.yaml`
- LIO params：
  `config/lio_3d_params.yaml`
- SLAM config：
  `config/slam_3d_config.yaml`
- SLAM params：
  `config/slam_3d_params.yaml`

## Hesai ROS2 driver

- `https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0`
- `source_driver_ros2.hpp` 中官方 PointCloud2 schema 包含：
  `x/y/z/intensity/ring/timestamp`。

---

# 38. 给执行 AI 的最后指令

**请不要只给建议或伪代码。按此文档实际修改仓库。**

执行时：

1. 先检查当前 branch 文件是否与本文描述一致；
2. 如果文件小范围变动，按当前代码结构适配，不要停下来向用户确认；
3. 保留现有安全边界和旧功能；
4. 完成所有新增代码、配置、脚本和测试；
5. 实际运行静态测试、构建测试；
6. 能在没有真机的步骤全部在本机完成；
7. 真机数据不可用时，提供 fake ROS messages / unit test 验证逻辑；
8. 真机可用时，自动 smoke test topic，但**不要发任何用于标定的机器人运动命令**；
9. 不要求用户重新标定；
10. 最终输出“修改文件清单 + 构建结果 + 测试结果 + 一键启动命令 + 已知降级项”。

实现结果的核心判断不是“plain_slam 启动了”，而是必须满足：

```text
PandarXT-16
   ↓
实时 LIO
   ↓
实时 3D map
   ↓
ray-traced 2D free/occupied/unknown map
   ↓
FrontierExtractor + SemanticRoutePlanner
   ↓
机器狗正常探索时地图持续增长
```

同时：

```text
现有 /go2w/odom/fused 与安全运动链保持不受影响。
```

---

**文档结束。**
