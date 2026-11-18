# SLAM 迁移审计报告（20260908 计划书 §3）

> 本文件是《GO2W 稳定三维 SLAM + 语义物体标注 MVP 重构实施计划_20260908》第 3 章的交付物。
> 结论全部来自当前本地工作树的源码核对，**未使用文件名猜测**；凡需真机取证才能定论的项，
> 明确标 `PENDING_LIVE`，不得当作已确认。
>
> 审计时间：2026-09-08（本地静态审计）；真机补证见文末「真机补证记录」。

---

## 1. 现有三维地图的真实数据链（§3.1）

```text
输入
  /hesai/pandarxt16/points_raw            (PandarXT-16, 10 Hz)
    -> pandar_slam_adapter                (schema 校验/过滤/时间戳策略)
  /go2w/slam/pandar_points
  /utlidar/imu  -> imu_fallback_adapter.py -> /go2w/slam/imu
        (ImuTimestampAligner: 启动窗口 500 样本中位数估 offset 后锁定；
         陀螺 yaw 轴符号翻转；回退/重复/跳变要求重启 session)
        │
        ▼
  lio_3d_node  (plain_slam_ros2 LIO 前端: normal_map + GICP, use_loose_coupling=true)
       发布 /go2w/slam/imu_pose_raw、/go2w/slam/aligned_scan、
            /go2w/slam/deskewed_scan、/go2w/slam/lio_map_cloud
        │
        ▼
  slam_3d_node (plain_slam_ros2 pose-graph 后端)      <-- 唯一权威世界坐标
       SetData() 每帧: slam_pose_ = slam_pose_ * (prev_odom^-1 * odom)
       每 accumulation_cycle=2 帧: 用 odom_pose 把扫描拼进一个 keyframe,
          再转回当前 imu frame, voxel 0.15 过滤, 写盘
          /tmp/pslam_data/{raw,filtered}_cloud<idx>.bin      <-- 写盘即固化
       keyframe 判据: distance_th=1.5 m / angle_th=25° 或 local_map_matching_rate<0.70
       回环: GetTargetCandidateIndices(KD-tree 最近 30 个 pose)
             + GICP(error_average_th=0.8, active_points_rate_th=0.45,
                    距离门限 12 m / 高度 3 m)   —— 无描述子
       位姿图: 检测到新 loop edge 才 g_optimizer_.Optimize(odom_edges_, loop_edges_, pose_graph_)
       地图:   BuildFilteredMapCloud() 遍历【全部 keyframe】, 用【当前优化后 pose】
               T * p 重新变换并拼整图 (上限 2e7 点, 超出从最旧截断)
       发布:   /go2w/slam/map_3d  (frame_id = pslam_map, QoS(1).transient_local.reliable)
       TF:     map -> odom, 用 IMU pose 的 header.stamp (不是 now())
        │
        ▼
 消费方
   scripts/go2w/plain_slam_web_bridge.py / plain_slam_web_cloud_relay.py
     -> app/spatial/slam_web_map_state.py::SlamWebMapState
        accept_map() 整张 replace(); MotionEpisodeGate 判 LIO 假平移并冻结地图
   go2w_plain_slam_bridge/plain_slam_health_monitor.cpp  (只订阅 map_3d 新鲜度)
   app/spatial/plain_slam_spatial_provider.py            (pose/map -> 自主搜索)
```

**发布者唯一性**：`/go2w/slam/map_3d` 的发布者只有 `slam_3d_node`（topic 名由
`scripts/go2w/generate_plain_slam_pandar_config.py` 从 `configs/go2w/plain_slam.yaml::topics.map_3d`
写入 `filtered_map_cloud_topic`）。全仓 `map_3d` 检索到的其余位置都是**订阅者/配置/WebUI 状态**，
不存在第二套发布同名地图的节点。→ 计划书 §2.2「与新 SLAM 冲突的第二套地图发布器」**不存在**。

---

## 2. 反模式检查（§3.2）

全仓检索 `append / concatenate / vstack / global_cloud / world_cloud / accumulate /
map_points / point_history / transform_points / /map_3d / PointCloud2` 后的逐项结论：

| 计划书怀疑的反模式 | 本项目实际情况 | 结论 |
|---|---|---|
| `global_points.extend(T_world_sensor @ current_scan)` 且历史点永不随回环更新 | `BuildFilteredMapCloud()` 每次位姿图更新后用**优化后的 pose** 重建全图 | **不成立**（`NOT_ROOT_CAUSE`） |
| 第二套 `/map_3d` 发布器 | 只有 `slam_3d_node` | **不成立** |
| WebUI 侧独立累积 | `GlobalVoxelCloud` 只被 `SlamWebMapState.accept_map()` 以 `replace()` 整张替换 | **不成立** |
| 手工 world-map fusion | 未发现 | **不成立** |

**但是，审计发现两个计划书没有点名、却真实存在的“半永久”结构：**

1. **`ROOT_CAUSE_CANDIDATE: intra-keyframe accumulation`**
   `slam_3d_interface.cpp::SetData()` 用 **LIO odom 位姿**把 `accumulation_cycle=2` 帧扫描
   拼成一个 keyframe，然后写盘（`filtered_cloud<idx>.bin`）。写盘之后这个 keyframe 的
   **内部点云形状就固定了**。回环优化只能修正 keyframe 之间的位姿，**无法修正 keyframe
   内部的形变**：0.2 s 窗口内的前端漂移会永久变成“一面墙两层”。
   这是“每帧 transform 后 append”在**单 keyframe 尺度**上的变体，必须计入根因候选。

2. **`filtered_map_cloud_` 的 2e7 点截断**
   超过上限后从**最旧的 keyframe** 开始丢弃，表现为“地图后半段突然变稀”，
   不会造成重影，但会污染 overlap 类指标，评估器必须记录 `map_point_count` 与截断标志。

---

## 3. TF 审计（§3.3）

已确认的 TF 发布者（源码级）：

| 节点 | 发布的变换 | 说明 |
|---|---|---|
| `slam_3d_node` | `map -> odom` | `T_map_odom = slam_pose * odom_pose^-1`，stamp 用 IMU pose 时间戳 |
| `lio_3d_node` | 有 `tf_broadcaster_` | 具体 frame/频率 `PENDING_LIVE` |
| `plain_slam_odom_adapter` | 可选 | `publish_tf` 默认 **false** |
| `go2w_lio_bringup/wheel_odom.py`、`point_lio_bridge.py` | 有 | 与 plain_slam 是否重叠 `PENDING_LIVE` |

计划要求的 frame tree：

```text
pslam_map
└── pslam_odom
    └── base_link_mapping_assist   (frames.base_child, 见 configs/go2w/plain_slam.yaml)
        ├── pandarxt16_link_unvalidated
        └── <camera optical frame?>          <-- PENDING_LIVE, 关键缺口
```

**关键缺口**：新计划书 §10 要求 `lookup_transform("map", camera_frame, time=image_timestamp)`。
当前没有证据表明相机 optical frame 挂在 `pslam_map` 这棵树上；若不可达，语义物体就无法投影到
地图坐标系。这一项必须在真机上用 `ros2 run tf2_tools view_frames` + `tf2_echo` 取证，
**不得假设**。

其余待真机确认项（`PENDING_LIVE`）：frame id 是否重复、static 与 dynamic 是否冲突、
sensor extrinsic 是否被重复应用、map/odom 是否接反、点云是否已在 base frame 又被二次变换。

---

## 4. 时间戳审计（§3.4）

| 流 | 时基 | 现状 |
|---|---|---|
| Pandar | 主机时钟 | `timestamp_policy.hpp` 7 种策略，绝不静默猜 |
| IMU | Unitree 传感器 epoch（开机后可能落后 host ~717–719 s） | `imu_timestamp_aligner.py` 启动窗口估 offset→锁定→冻结；锁定后 offset 跳变>5 s 要求重启 session；数据空洞只丢一帧 |
| LiDAR–IMU 对齐 | — | 由 LIO 内部按 header.stamp 匹配，具体 skew `PENDING_LIVE` |
| RGB / Depth | D435 | `max_rgb_depth_skew_ms=40`（计划书 §32）尚未实现，`PENDING_LIVE` |
| 语义 TF 查询 | — | **当前未实现**：没有 `T_map_camera(t_image)` 查询路径 |
| TF 是否用 `now()` | — | `slam_3d_node` 用消息时间戳，符合要求；语义链待建 |

---

## 5. 根因候选与排除结论

| 候选 | 状态 |
|---|---|
| 不可逆全图 append | **排除**（§2） |
| 第二套地图发布器 / 双地图 | **排除** |
| TF 用 `now()` | **排除**（SLAM 侧） |
| intra-keyframe accumulation（0.2 s 内用 odom 拼帧并写盘固化） | **保留，高优先级** |
| 回环召回弱（无描述子，仅距离门限 + GICP） | **保留** |
| 前端位姿质量（松耦合、acc_scale=0.954、convergence_th=0.01 已修） | **保留但已多轮整改**；外参仍 `candidate_unconfirmed` |
| 语义投影用错时间戳 / 相机 frame 不可达 | **保留，尚未验证** |

**结论**：重影不可能靠“加大 voxel、放大 merge radius、改去重”解决，也**不是“plain_slam 选错项目”**。
必须先用 Dataset A（原地旋转假平移/墙面厚度）与 Dataset B（原点往返重叠 RMSE/回环次数）
拿到量化数据，再决定改哪一条。换 LIO-SAM 只在“回环召回”这一环有实质收益，且同样依赖
IMU 时基与外参正确；在数据出来之前换后端属于赌博。

---

## 6. 与计划书 §2.2「删除或停用」的对应关系

| 计划书要求删除 | 审计结论 |
|---|---|
| 旧全局 3D 点云累积器 | **不存在**，无需删除 |
| 旧手工 world-map fusion | **不存在** |
| “收到新 scan 就 transform + append”的永久累计 | 全图级**不存在**；但存在 keyframe 级固化（§2.1），改为修复而非删除 |
| 与新 SLAM 冲突的第二套地图发布器 | **不存在** |
| 旧的伪全局 object pose 累积逻辑 | 语义层尚未接地图坐标系，`PENDING_LIVE` |

---

## 7. 真机补证记录（验证阶段填写）

> 以下每项必须附原始命令输出，不得只写结论。

- [x] **相机 frame 名 = `d435_color_optical_frame`**（`/go2w/d435/color/camera_info`，2026-09-08 18:45 实测）
- [x] `/tf_static` 实测只含 `base_link → utlidar_lidar → utlidar_imu`
- [ ] **关键缺口（下一步第一优先）**：`d435_color_optical_frame` **没有任何父 TF**；
      且 `pslam_map → base_link` 链未接通（`plain_slam_odom_adapter.publish_tf` 默认 false）
      → 计划书 §10 的 `T_map_camera(t_image)` **当前不可实现**
- [ ] `PENDING_LIVE` 完整 TF 树（`view_frames` + `tf2_echo` 关键边）
- [ ] `PENDING_LIVE` `/go2w/slam/map_3d` 发布者计数（`ros2 topic info -v`）
- [ ] `PENDING_LIVE` LiDAR/IMU/RGB/Depth 时间戳年龄与 skew 实测
- [ ] `PENDING_LIVE` IMU 对齐状态（`/go2w/slam/imu_status` 的 mode/offset/locked/drift）
- [ ] `PENDING_LIVE` 相机 optical frame 在 `pslam_map` 树下是否可达
- [ ] `PENDING_LIVE` `/tmp/pslam_data` keyframe 文件数与 keyframe 内部是否可见重影
