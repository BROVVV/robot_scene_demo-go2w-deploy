# Go2-W + Hesai PandarXT-16 + plain_slam_ros2 实时 3D 建图融合

> Status: implemented per
> `GO2W_PANDAR_PLAIN_SLAM_IMPLEMENTATION_PLAN.md` (mapping-assist first version).
>
> **安全边界（第一版不变）**：本管线是 `MAPPING_ASSIST`——它产生 3D 地图、
> shadow odometry 与 2D OccupancyGrid 供探索/空间理解使用，但**不是运动权威**。
> `/go2w/odom/fused` 与现有运动/安全链保持完全不受影响；Pandar 外参保持
> `candidate_unconfirmed`；`/go2w/slam/ready=true` 只代表 mapping 可用。

## 1. 架构

```text
Hesai PandarXT-16
        │  (现有 hesai_ros_driver)
        ▼
/hesai/pandarxt16/points_raw
        │  pandar_slam_adapter（schema 校验/过滤/timestamp 自动策略）
        ▼
/go2w/slam/pandar_points
        │                              ┌──────────────────────┐
        ├──────────────────────────────► plain_slam_ros2 LIO │◄── /utlidar/imu
        │                              │ (lio_3d_node)        │
        │                              └──────────┬───────────┘
        │                                 aligned_scan / odom
        ▼                                         ▼
  slam_3d_node ──► /go2w/slam/map_3d      plain_slam_odom_adapter
        │                                  (IMU pose → base pose)
        │                                        │
        │                                        ▼
        │                               /go2w/slam/odom_base
        │                                        │
        ▼                                        ▼
   pointcloud_to_occupancy（ray tracing → free/occupied/unknown）
        │
        ▼
/go2w/slam/map_2d ──► PlainSlamSpatialProvider ──► FrontierExtractor
                                                  └► SemanticRoutePlanner
                                                  └► SemanticNavigation V2
```

Frame 隔离：`pslam_odom` / `pslam_imu` / `pslam_map`；base 影子 frame
`base_link_mapping_assist`。新增节点**不发布** `odom -> base_link`。

## 2. 依赖与构建

```bash
# 1) 依赖（ROS 包、yaml-cpp、eigen3、gtest）
bash scripts/go2w/install_dependencies.sh
# 2) vendor plain_slam_ros2（固定 commit 写入 plain_slam_lock.yaml）+ Sophus/nanoflann
bash scripts/go2w/vendor_plain_slam_ros2.sh          # 或 --commit <SHA>
# 3) 构建
bash scripts/go2w/build_ros2.sh
```

本机（开发工作站）与机器狗（192.168.123.18, Ubuntu 22.04 / ROS2 Humble）都
需要执行以上步骤；部署时把 `ros2_ws/src` 与 `scripts/configs/tests` 同步到
机器狗后重新构建。

## 3. 启动

```bash
# 只开 3D/2D 建图（可选 --rviz / --record / --debug / --no-start-hesai）
bash scripts/go2w/start_plain_slam_mapping.sh
# 停止（只关闭 mapping 相关进程，Hesai 若原本在跑则保留）
bash scripts/go2w/stop_plain_slam_mapping.sh
# 健康检查
bash scripts/go2w/check_plain_slam_mapping.sh
# 保存地图
bash scripts/go2w/save_plain_slam_map.sh
# 录调试 bag
bash scripts/go2w/record_plain_slam_debug_bag.sh
```

一键启动语义探索 + 实时建图（plain_slam 作为空间来源）：

```bash
GO2W_START_PLAIN_SLAM=1 GO2W_SPATIAL_PROVIDER=plain_slam \
  bash scripts/go2w/start_semantic_exploration.sh --target "你的目标"
```

探索干跑（不发运动命令）：

```bash
python3 scripts/go2w/run_semantic_exploration.py \
  --target "测试目标" --spatial-v2 --spatial-provider plain_slam \
  --dry-run-motion --allow-degraded
```

## 4. Topic 一览

| Topic | 类型 | 说明 |
|---|---|---|
| `/hesai/pandarxt16/points_raw` | PointCloud2 | 现有 driver，未授权 frame |
| `/go2w/slam/pandar_points` | PointCloud2 | 精简 schema x/y/z/intensity(F32)+timestamp(F64) |
| `/go2w/slam/point_status` | String | timestamp mode / schema / 丢点统计 |
| `/go2w/slam/imu_pose_raw` | PoseStamped | plain_slam IMU 姿态（上游） |
| `/go2w/slam/imu_odom_raw` | Odometry | plain_slam IMU 里程计（上游） |
| `/go2w/slam/odom_base` | Odometry | base 影子里程计（frame=pslam_odom） |
| `/go2w/slam/base_pose` | PoseStamped | base 影子位姿 |
| `/go2w/slam/aligned_scan` | PointCloud2 | LIO 对齐扫描（世界系） |
| `/go2w/slam/deskewed_scan` | PointCloud2 | 去畸变扫描 |
| `/go2w/slam/map_3d` | PointCloud2 | SLAM 全局 3D 地图 |
| `/go2w/slam/map_2d` | OccupancyGrid | ray-traced free/occupied/unknown |
| `/go2w/slam/occupancy_status` | String | 地面估计状态 |
| `/go2w/slam/health` | DiagnosticArray | 管线健康状态 |
| `/go2w/slam/ready` | Bool | mapping readiness（≠运动授权） |

## 5. Health / ready 语义

状态机：`STARTING → POINTCLOUD_OK → IMU_OK → LIO_OK → MAP3D_OK →
MAP2D_OK → READY_MAPPING_ASSIST`。`ready=true` 需要五个数据源最近
（cloud 1s / IMU 0.2s / odom 1s / aligned 1s / map2d 2s）内均有数据。

降级标记（health 消息 message 中携带）：

- `TIMESTAMP_SYNTHETIC` / `TIMESTAMP_CONVERTED`：Pandar 时间戳合成/换算；
- `EXTRINSIC_CANDIDATE`：Pandar 外参仍未标定，仅供 mapping assist；
- `IMU_DEGRADED`：IMU 频率不足或数值异常；
- `LIO_STALE` / `MAP2D_STALE`：增量数据断流；
- `GROUND_ESTIMATE_FALLBACK`：自动地面估计降级到低百分位。

`/go2w/slam/health` 中 `authorization` 条目恒为：
`mode=MAPPING_ASSIST motion_authorized=false safety_authorized=false`。

## 6. 配置

| 文件 | 用途 |
|---|---|
| `configs/go2w/plain_slam.yaml` | 主配置：topic/frame/健康阈值/语义 |
| `configs/go2w/plain_slam_lio_params.yaml` | LIO 调参与 extrinsics 模板 |
| `configs/go2w/plain_slam_slam_params.yaml` | SLAM 后端调参 |
| `configs/go2w/plain_slam_bridge.yaml` | bridge 节点全部参数 |
| `configs/go2w/plain_slam_lock.yaml` | 上游 commit 锁 |
| `runtime/go2w/plain_slam/config_provenance.json` | 生成溯源（自动） |

外参**自动推导**：`scripts/go2w/generate_plain_slam_pandar_config.py` 读取
`official_reference.yaml` + `hesai_pandarxt16_extrinsics.yaml`，计算
`T_imu_pandar = inverse(T_base_imu) * T_base_pandar` 并生成 runtime YAML，
无需人工测量/标定。`--check` 模式验证与计划书 sanity reference 一致。

## 7. 限制与降级项（第一版）

1. Pandar 外参 `candidate_unconfirmed`：外参误差会被 LIO 吸收一部分，
   但地图可能存在系统偏移；**不得用本管线地图做碰撞/安全决策**；
2. plain_slam odom 未接管运动：探索期间姿态可能和 `/go2w/odom/fused`
   有毫米~厘米级差异（坐标系不同），运动执行仍用 fused odom；
3. 无 PTP：per-point timestamp 依赖 driver 绝对/相对时间自动识别；
   若识别失败会显式标记并降级（不静默）；
4. 自动地面估计为启发式（3m 半径 histogram + EMA）；地板高差/多楼层
   场景可能退化；
5. 80×80 m @ 0.1 m 固定地图；更大环境需改配置；
6. 第一版不发布正式 `map -> odom -> base_link` TF，不与 Nav2 地图服务
   对接（Level B 接口保留，默认 `use_plain_slam_map:=false`）。

## 8. License

plain_slam_ros2 学术/个人免费，商业使用需作者书面授权；详见
`docs/go2w/plain_slam_license_note.md`。