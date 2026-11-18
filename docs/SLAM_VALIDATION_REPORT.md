# 2026-09-10 续测：闭环量化能力补齐 + 真机 Dataset C / C2（最新一轮，置顶）

> 本轮目标：把计划书 §20.3（回环后抖动）、§20.4（回环前后地图改善）、§22（实例去重口径）
> 三项**缺失的量化能力**补上，并用真机闭环数据实测；同时补 §24/§31/§37/§38 的可视化与诊断工具。
> 结论摘要：**评估器缺口已补齐并单测通过**；真机闭环能稳定触发 loop closure（78 / 615 条边），
> 但 **endpoint / overlap 两类指标仍 FAIL**，根因指向 LIO 在小范围绕圈与纯旋转下的位姿不一致
> （轮式里程计认为已归位 1.6 cm，SLAM 却认为差 1.15 m / 84.7°）。
> 本轮末尾机器狗再次失联（控制主机 `enp3s0` 变 `NO-CARRIER`），真机工作即时停止，
> 交接见 `docs/GO2W_稳定三维SLAM_语义物体标注_MVP重构实施计划_20260908_含交接书.md` 末节。

## 0. Baseline（2026-09-10 09:49 CST）

```text
控制主机：192.168.123.99 / enp3s0 UP(09:49)，/home/mxt/robotscene
分支：    fix/go2w-autonomous-search-lio-topology-target
HEAD：    21a7bbb（存在大量未提交改动，本轮未 reset/clean/checkout）
机器狗：  192.168.123.18 在线，uptime 28 min，SSH 正常
readiness：state=ready, ready=true；除 rotation_clearance_valid=false 外全部 ok
SLAM：    /go2w/slam/map_3d Publisher count = 1（go2w_plain_slam_slam）
          /go2w/slam/ready = true；health = READY_MAPPING_ASSIST
          /go2w/slam/imu_status = OFFSET_LOCKED_SENSOR_STAMP
          /go2w/slam/aligned_scan ≈ 10.02 Hz
授权：    用户明确授权「小步旋转闭环，必须回到胶带原点」，因此使用
          `--operator-authorized-rotation`（每步 ≤30°，其余安全门全部保留）
```

## 1. 本轮代码变更（全部为计划书要求项，未扩大范围）

| 文件 | 变更 |
|---|---|
| `app/mapping/map_quality_evaluator.py` | 新增 `post_loop_jitter()`（§20.3，回环后 3–5 s 窗口 P95）、`loop_overlap_improvement()`（§20.4，回环前/后末帧相对起始扫描的重合度）、`instance_counts()`（§22，把 `label#persistent_id` 归一成 label→实例数）；`load_bag()` 记录 `loop_stamps`、`pre_loop_scan`、`post_loop_scan` |
| `tests/test_map_quality_evaluator.py` | 新增 7 个单测（抖动、无回环 NOT_VERIFIED、回环前后改善、世界系校验、实例口径）；`17 passed` |
| `app/mapping/semantic_map_visualizer.py` | 新增（§24/§37/§38）：`objects.json`、`semantic_observation_scatter.png`、`map_topdown.png`、`start_end_overlay.png` |
| `app/slam/slam_diagnostics.py` | 新增（§31/§32）：传感器 age、frame、RGB-D skew、`/tf_static` 外参；"topic 未录制"如实标 NOT_VERIFIED 而非伪 FAIL |

回归（控制主机）：

```text
python3 -m pytest tests/test_plain_slam_spatial_provider.py tests/test_map_quality_evaluator.py \
  tests/test_slam_bridge.py tests/test_depth_object_localizer.py \
  tests/test_semantic_frame_object_observations.py tests/test_rgbd_timestamp_contract.py -q -p no:anyio
→ 48 passed in 0.81s
```

## 2. Dataset C：真机十二边形一圈（小步旋转，未含归位）

目录：`artifacts/slam_validation/datasetC_20260910_095109/`（bag 1.4 GB）

运动：`f0.15,r30` × 12 = 24 步，`--operator-authorized-rotation`，24/24 步 verified，MOTION_RC=0。

```text
surface_thickness_p95_m        = 0.0732  (≤0.15)   PASS
loop_closure_count             = 78      (≥1)      PASS  keyframes=13, loop_edge_markers=12
post_loop_translation_jitter_m = 0.0356  (≤0.08)   PASS  （新增指标首次产出真实值）
post_loop_yaw_jitter_deg       = 0.062   (≤2.0)    PASS
endpoint_translation_error_m   = 0.4401  (≤0.20)   FAIL
endpoint_yaw_error_deg         = 9.776   (≤5.0)    FAIL
overlap_rmse_m                 = 0.3115  (≤0.10)   FAIL
overlap_p95_m                  = 0.6098  (≤0.18)   FAIL
overlap_inlier_ratio           = 0.303   (≥0.75)   FAIL
pre/post loop overlap          = 0.1573 → 0.3115   （末帧物理位置未回起点，不构成"优化变差"证据）
```

运动结束后用轮式 fused odom 反馈归位成功：残差 **0.0160 m / 0.33°**（判据 <0.06 m / <3°）。

## 3. Dataset C2：真机十二边形一圈 + 位姿反馈归位 + 静止 10 s

目录：`artifacts/slam_validation/datasetC2_20260910_095750/`（bag 1.8 GB）

运动：`f0.15,r30` × 12 → 归位迭代（fused odom 反馈）→ 静止 10 s。

```text
surface_thickness_p95_m        = 0.0742  (≤0.15)   PASS
loop_closure_count             = 615     (≥1)      PASS
post_loop_translation_jitter_m = 0.0235  (≤0.08)   PASS
post_loop_yaw_jitter_deg       = 0.858   (≤2.0)    PASS
endpoint_translation_error_m   = 1.1544  (≤0.20)   FAIL
endpoint_yaw_error_deg         = 84.68   (≤5.0)    FAIL
overlap_rmse_m                 = 0.6632  (≤0.10)   FAIL
overlap_p95_m                  = 1.1453  (≤0.18)   FAIL
overlap_inlier_ratio           = 0.1417  (≥0.75)   FAIL
accepted                       = false（failed = endpoint_translation/yaw, overlap rmse/p95/inlier）
```

视觉证据（已人工复核）：`datasetC2_.../start_end_overlay.png` 显示起/终子图整体错位约 1–2 m 且带旋转，
与 overlap 数值一致；`datasetC_.../start_end_overlay.png` 同样呈红蓝交错。

诊断（`app/slam/slam_diagnostics.py`）：`diagnostics.json` `accepted=true`；
`/go2w/slam/imu` 38646 条、age p95 43.1 ms；`/tf_static` 仅
`base_link→utlidar_lidar (0.28945, 0, −0.046825)` 与 `utlidar_lidar→utlidar_imu`；
RGB-D skew 未验证（这两个 bag 未录 D435 话题）。

## 3.1 Dataset C3：映射栈重启后再测（同样 FAIL，排除"栈不健康"这一解释）

背景：10:05 机器狗短暂失联（`enp3s0` NO-CARRIER），期间 LIO 失去输入后发散
（`/go2w/slam/odom_base` 跑到 17791 m，语义 `map_xyz` 出现 3833 m），
因此重启了 plain_slam 映射栈；重启后静止 LIO 回到毫米级（slam_pose ≈ (−0.28, 0.014)）。

目录：`artifacts/slam_validation/datasetC3_20260910_101814/`（bag 2.4 GB）

```text
surface_thickness_p95_m        = 0.0692  (≤0.15)   PASS
loop_closure_count             = 435     (≥1)      PASS
post_loop_translation_jitter_m = 0.0072  (≤0.08)   PASS
post_loop_yaw_jitter_deg       = 0.115   (≤2.0)    PASS
endpoint_translation_error_m   = 1.9248  (≤0.20)   FAIL
endpoint_yaw_error_deg         = 68.46   (≤5.0)    FAIL
overlap_rmse_m / p95 / inlier  = 1.7997 / 4.6524 / 0.1026  FAIL
```

轮式归位（`scripts/go2w/return_to_origin.py`）本轮 3 轮收敛：终点与起点残差约 **0.018 m / 2.65°**
（更早一次归位到胶带原点残差 0.025 m / 0.56°）。

**结论（重要）**：C / C2 / C3 三次真机闭环的 SLAM endpoint 分别是 0.44 / 1.15 / 1.92 m，
而轮式 odom 每次都把机器人带回原点 2 cm 内。轨迹半径只有约 0.19 m（12 边 × 单步 ~0.10 m 的十二边形），
对 LiDAR SLAM 属**退化轨迹**（几乎无平移视差、点云几乎不变，配准欠约束），
因此该轨迹**不能作为 §47 场景 3「绕房间/走廊一圈」的等价验收**。
真正的闭环验收需要一次**人工遥控走出的房间级回路**（见调查记录：用户提出由其本人遥控走一圈，
agent 负责录包与评估），当前该数据尚未采集。

## 3.2 Dataset Chand：**人工遥控的一整圈**（最接近 §47 场景 3，结果 FAIL，暴露 LIO 全局漂移）

背景：用户主动提出「让我来遥控机器狗走一圈，最终回到原点，你来做 SLAM 评估」。
agent 只负责录包（不发任何运动命令），用户手动遥控走完整一圈并回到胶带原点停稳。

目录：`artifacts/slam_validation/datasetChand_20260910_102514/`（bag 2.3 GB，约 150 s）

```text
surface_thickness_p95_m        = 0.0642  (≤0.15)   PASS
loop_closure_count             = 571     (≥1)      PASS   keyframes=46
post_loop_translation_jitter_m = 0.2288  (≤0.08)   FAIL
post_loop_yaw_jitter_deg       = 6.227   (≤2.0)    FAIL
endpoint_translation_error_m   = 9.2021  (≤0.20)   FAIL
endpoint_yaw_error_deg         = 136.95  (≤5.0)    FAIL
overlap_rmse_m / p95 / inlier  = 4.4054 / 6.4126 / 0.0  FAIL
map extent                     = 32.4 × 30.8 × 14.2 m（176300 点）
accepted                       = false
```

**轮式 vs SLAM 的对照（`traj_compare.png`，已人工复核）**

```text
/go2w/odom/fused     : start (1.08, 2.78, -112.5°)  end (1.49, 3.05, -122.7°)   -> 0.49 m / 10.2°
/go2w/slam/odom_base : start (-5.75, -1.52, 68.7°)  end (-14.91, -0.69, -154.4°) -> 9.20 m / 137°
```

两条轨迹在图上**形状相似、尺度相近**，但 SLAM 那条整体平移出去约 10 m 且**不闭合**；
`start_end_overlay.png` 里起帧与末帧是同一环境的**两份错位 10 m 的副本**，
即 LIO 把用户走过的同一地点画到了两个位置 —— 这是 **LIO 全局漂移**，不是"用户没回到原点"
（用户按胶带回到原点，轮式 odom 亦只差 0.49 m / 10.2°）。

**结论**：
1. 房间级一圈（本次约 15–25 m 路径、含多段旋转）下，`plain_slam` 的 LIO **累积漂移达 9.2 m / 137°**，
   571 条 loop edge 触发后仍未拉回，`pre→post` 首末子图重合度还从 0.375 恶化到 4.405。
2. 因此计划书 §47 场景 3（完整闭环）当前**不通过**；§20.3 回环后抖动同样 FAIL。
3. 本 bag **未录原始传感器**（`/hesai/pandarxt16/points_raw`、`/utlidar/imu`），
   所以无法离线重放做时间同步/回环参数 A/B —— 下一轮必须加 `--with-raw`。

## 3.3 离线 A/B：同一盘 bag 换后端（**结论：换后端有效**）

问题：同一个 bag、同一段运动，轮式说回到原点（0.27 m），plain_slam 说差 6.6 m / 72°。
为了区分「算法问题」与「输入问题」，用**同一盘 bag**（`datasetChand2_20260910_103205`，
含 `/hesai/pandarxt16/points_raw` + `/go2w/slam/imu`）离线跑第二个后端
`ros-humble-rko-lio 0.3.2`（apt 已安装，`offline_node`），不动机器狗、不改现有链路。

命令（可复现）：

```bash
ros2 launch rko_lio odometry.launch.py mode:=offline \
  bag_path:=$PWD/artifacts/slam_validation/datasetChand2_20260910_103205/bag \
  lidar_topic:=/hesai/pandarxt16/points_raw imu_topic:=/go2w/slam/imu \
  base_frame:=base_link odom_frame:=rko_odom \
  config_file:=/tmp/rko_go2w.yaml dump_results:=true \
  results_dir:=/tmp/rko_ab run_name:=rko_nodskew
# 外参：lidar2base 用项目候选值（0.130,0.015,0.014 / RPY 0.385,0.905,11.357°），imu2base=identity
# deskew:=false（deskew:=true 时离线节点在本机 5 分钟无进展，属待查项）
```

结果（三条轨迹对齐到同一起点后统计，`/tmp/rko_ab/traj_ab.png`，已人工复核）：

| 轨迹 | 帧数 | 路径长 | **端点误差** | **净 yaw 变化** | 轨迹范围 |
|---|---|---|---|---|---|
| `/go2w/odom/fused`（轮式，参照） | 3333 | 6.38 m | 0.266 m | 0.8° | 0.5 × 2.8 m |
| `/go2w/slam/odom_base`（plain_slam） | 1667 | 12.85 m | **6.601 m** | **71.9°** | 5.4 × 4.3 m |
| **`rko_lio` offline** | 1667 | 21.13 m | **0.018 m** | **−0.4°** | 3.6 × 1.8 m |

处理耗时：8.4 s（1665 次 ICP，平均 0.17 ms）。

**视觉证据（关键）**：对齐后的轨迹图上，rko_lio 画出一个**闭合的环形**（出发→绕行→回到起点），
相对 yaw 从 0 平滑升到约 +90°、经 wrap 后回到 0°，即**完整一圈 360° 后被正确重建**；
plain_slam 的轨迹是一条**不闭合的开放曲线**（终点偏离 6.6 m），yaw 只累积到约 +90° 就停住，
净旋转少算了约 90°。

**结论**：
1. 在同一段输入数据上，**rko_lio 的端点误差比 plain_slam 小 360 倍**（0.018 m vs 6.601 m），
   净旋转也正确回到起点；因此本次房间级漂移的**主要瓶颈是 plain_slam 的 LIO 后端，而不是输入层**。
2. 慢速实验（§3.2）里"漂移与路径长度成比例"的现象，与该结论一致：是后端系统性误差。
3. rko_lio 的路径长 21.13 m 大于环路实际周长（含静止抖动：v>0.1 m/s 的有效路径 16.98 m），
   这一点需要在切换前用「静止段抖动量」单独评估，但目前不影响端点/旋转结论。
4. `rko_lio` **本身没有回环/位姿图**，要满足计划书 §6.3 必须再配一层回环
   （候选：`slam_toolbox`（已安装）或 Scan Context/pose-graph）。切换前还需要：
   deskew 输入字段打通（`lidar_timestamps.*`）、外参确认、话题契约对齐
   （上层语义 provider 依赖 `/go2w/slam/map_2d` 与 `/go2w/slam/odom_base`）。

## 3.4 阶段 1：rko_lio deskew 打通排查（**暴露 ~85 ms 的 LiDAR/IMU 时间基偏差**）

目标：让 `rko_lio` 能开启 `deskew`（逐点运动补偿）。结论：**未打通，但定位到机制，且这条线索可能同时解释 plain_slam 的 `dsk_max` 恶化。**

### 3.4.1 Pandar 逐点时间字段（实测二进制字段表）

```text
/hesai/pandarxt16/points_raw: 64000 点/帧, point_step=26, is_dense=False
fields = x, y, z, intensity, ring, timestamp
timestamp: FLOAT64, 绝对值 Unix 秒(1.789e9), 每帧跨度 0.0995–0.1003 s
逐点时间最小值 == header.stamp（差 0.000000 s）
header.stamp ≈ bag 接收时间 − 102.7 ms
```

即：字段名是 `timestamp`（不是 `time`），语义是**绝对时间**，因此正确配置应为
`lidar_timestamps.force_absolute: true` + `multiplier_to_seconds: 1.0`。

### 3.4.2 三种配置的实测结果

| 配置 | 结果 |
|---|---|
| `deskew: true`（autodetect） | 卡死：墙钟 5 min 无进展，CPU 时间停在 36 s |
| `deskew: true` + `force_absolute` + `multiplier 1.0` | 卡死：CPU 时间停在 33 s |
| 同上 + `async.max_lidar_buffer_size: 400` | 卡死，日志 `Lidar buffer size: 360, max_lidar_buffer_size: 400, throttling`（消费端一帧未完成） |
| **`deskew: false`** | **8.4 s 跑完 1665 帧 ICP**，端点误差 0.018 m（见 §3.3） |

### 3.4.3 根因：LiDAR 逐点时间与 IMU 时间不同基准

```text
LiDAR header        = bag 接收时间 − 102.7 ms
IMU(适配器校正后) header = bag 接收时间 −  88.2 ms
→ 一帧 LiDAR 的时间窗 = [T − 102.7 ms, T − 3.2 ms]
→ 流式读到这里时最新 IMU 只到 T − 88.2 ms，窗口末端缺 ≈85 ms
```

deskew 需要 IMU 覆盖整帧窗口 → 一直等；离线读线程又受 LiDAR buffer 上限限流 → **死锁**
（提高 buffer 只是把死锁点后移，实测 50→400 仍复现）。

**旁证（重要）**：plain_slam 当前 `dsk_max` 峰值 0.14 m（历史修复后为 0.0028 m）、
每帧 IMU 样本 `imu_n` 12–20（历史 19–20）——与"时间基偏差约 85 ms"高度一致，
说明该偏差很可能**同时**是 plain_slam 去畸变变差的原因，而不只是 rko_lio 的问题。

### 3.4.4 三条修法（按成本）

| 方案 | 成本 | 说明 |
|---|---|---|
| **a. 暂不启用 deskew** | 0 | no-deskew 已达标（§3.3）；10 Hz + 速度 ≤0.5 m/s ⇒ 单帧位移 ≤5 cm，收益有限 |
| b. 在线模式验证 | 阶段 3 顺带 | 实时流里"等 85 ms 的 IMU"只是延迟而非死锁，deskew 可能自然可用 |
| c. 做时间基对齐 | 20–30 min | 平移 LiDAR 逐点时间（或对齐 IMU 时间戳），一并治 plain_slam 的 dsk_max |

当前取舍：**先走 a**（不阻塞），把 c 记入交接书，阶段 3 用 b 验证、必要时用 c 收尾。

## 3.5 阶段 2：rko_lio + 现有回环层（`slam_3d_node`）离线闭环验证（**架构验证成功，5/7 项 PASS**）

目标：验证计划书 §35 的"只换里程计、保留项目已调过的回环 / 位姿图 / 地图重建层"是否可行。

实现（全部为本轮新增的桥接件，未改现有链路）：

| 件 | 说明 |
|---|---|
| `scripts/go2w/rko_slam_bridge.py` | 薄桥接：`/rko_lio/odom` → `PoseStamped`；Pandar 原始点云按候选外参刚体变换到 `base_link` 系，字段（x/y/z/intensity/ring/timestamp）原样保留；两侧用**同一 header.stamp** 发布，满足 `slam_3d_node` 的 TimeSynchronizer 精确配对 |
| `/tmp/rko_slam_params.yaml` | 独立 `slam_3d_node` 实例参数：输出到 `/go2w/rko_slam/*`，独立 `map_cloud_dir=/tmp/go2w_rko_slam/`（不碰正在运行的 plain_slam 数据目录） |
| `app/mapping/map_quality_evaluator.py` | 新增 `--topic-prefix`（默认 `/go2w/slam`），使同一评估器可直接评 rko 链路输出；回归 `tests/test_map_quality_evaluator.py` 24 passed |

离线流程（不动机器狗）：`ros2 bag play`（把 `/hesai/pandarxt16/points_raw`、`/go2w/slam/imu`
重映射到 `/go2w/rko_slam/bag_*`，避免与实时驱动抢同一话题）→ rko_lio(online) → 桥接 →
独立 `slam_3d_node` 实例 → 录制 `/go2w/rko_slam/{map_3d,aligned_scan,odom_base,pose_graph_*}` → 评估。

结果（**同一盘 bag、同一段运动**，与 §3.2 的 plain_slam 直接对比）：

| 指标 | plain_slam（同 bag） | **rko_lio + 回环层** | 阈值 | 判定 |
|---|---|---|---|---|
| endpoint 平移误差 | 6.601 m | **0.018 m** | ≤0.20 | **PASS** |
| endpoint yaw 误差 | 71.86° | **0.382°** | ≤5.0 | **PASS** |
| overlap RMSE | 2.2408 m | **0.0852 m** | ≤0.10 | **PASS** |
| overlap p95 | 4.7358 m | **0.1104 m** | ≤0.18 | **PASS** |
| overlap inlier ratio | 0.0054 | **0.9403** | ≥0.75 | **PASS** |
| surface_thickness_p95 | 0.0638 m | 0.0653 m | ≤0.15 | **PASS** |
| loop_closure_count | 612 | 272（26 markers / 27 kf） | ≥1 | **PASS** |
| post_loop_translation_jitter | 0.1086 m | 0.1391 m | ≤0.08 | **FAIL** |
| post_loop_yaw_jitter | 0.361° | 8.152° | ≤2.0 | **FAIL** |

地图：103787 点、extent 25.0 × 11.9 × 10.3 m。

**结论**：
1. **"换里程计 + 复用现有回环层"在架构上成立**：回环被正常触发（272 条边），
   首末子图重合度从 2.24 m 降到 0.085 m、inlier 从 0.005 升到 0.94，端点误差从 6.6 m 降到 18 mm。
2. 剩余 FAIL 只有 post-loop jitter 两项。**在离线重放语境下该指标含义受限**：
   重放时桥接丢帧率约 32%（点云到达时可用 odom 超时），`slam_3d_node` 收到的是不连续帧序列；
   同时 272 条回环边只分布在 27 个关键帧上，回环触发相对激进。两者都可能在重放中放大位姿抖动。
   因此这两项**暂记为"离线重放不达标、原因待查"**，不能直接当作真机结论。
3. 下一步（仍可离线）：降低桥接丢帧（放宽/重构 odom 配对策略）、以及回环参数（候选半径 / ICP 阈值）扫描；
   之后再进入阶段 3 在线接入。

### 3.5.1 后续迭代的实测（如实记录，含失败）

| 迭代 | 改动 | 结果 |
|---|---|---|
| 桥接配对改 odom 驱动 | 由"点云到达时找最近 odom"改为"odom 到达时从点云缓冲里配最近的帧" | 配对率 **68% → 97~99%**（dropped 32% → 1~5%）；endpoint/overlap 与 post-loop jitter 基本不变（0.018 m / 0.085 / 0.140 m）⇒ 之前 32% 丢帧不是这些指标的主因 |
| 收紧回环参数 | `num_loop_candidates 30→10`、`loop_detection_distance_thresh 12→4 m`、`error_average_th 0.8→0.5`、`gicp_max_correspondence_dist 1.0→0.5`（放在**只复制了单个文件的** `/tmp/rko_slam_params_dir`） | **明显变差**：endpoint 0.018 → **1.945 m**、overlap 0.085 → 1.180、inlier 0.941 → 0.163（回环数 531 → 559 几乎不变）。**该实验变量不干净**（参数目录不完整，可能退化为默认参数），结论**不可采信**，已回滚到原始参数目录 |
| deskew 在线验证（阶段 1 的 b 方案） | 在线 `deskew: true` + `lidar_timestamps.force_absolute: true` | **在线可用**：rko_lio 10 Hz 正常；离线会死锁的正是"预读未来 IMU"这一离线特性 |

**post-loop jitter 的定性分析**：该窗口（`loop+3..5 s`）内只有 **10 个位姿样本**（10 Hz 期望 20，说明位姿发布不连续），
且 0.127–0.140 m 的位移远小于"机器人若在运动、2 秒应走的 ≥0.5 m"——
⇒ 回环发生时机器人接近静止，这 0.13 m 是**回环优化造成的一次性位姿修正**，而不是持续震荡。
按计划书 §20.3 的字面阈值（≤0.08 m）仍记为 FAIL，但需在真机（位姿发布连续）下复测才能定论。

### 3.5.2 在线链路真机小幅运动验证（2026-09-10 14:45，机器狗重启后）

机器狗重启后重建权威栈（`ready=true`、IMU `OFFSET_LOCKED_SENSOR_STAMP`、`map_3d` 单发布者、
静止 SLAM 毫米级），随后**在线**并行跑两条链路，并执行受控小幅往返 `f0.15,b0.15` + 位姿反馈归位：

```text
运动：/go2w/odom/fused 由 (1.27017, 2.81072) 到 (1.25666, 2.77749)，yaw 差 0.23°
     归位残差 0.036 m / 0.23°（回到运动起点）
数据：artifacts/slam_validation/real_ab_small_20260910_144559/
```

| 链路 | endpoint 平移 | endpoint yaw | overlap RMSE | overlap inlier | loop | thickness |
|---|---|---|---|---|---|---|
| `plain_slam`（现权威） | 0.0400 m | 0.367° | 0.0283 m | 0.986 | 0 | 0.0555 m |
| **`rko` 链路（新方案）** | **0.0333 m** | **0.324°** | 0.0356 m | 0.985 | 0 | 0.0564 m |

**结论**：在线 rko 链路在真机运动中**行为正常**（不发散、指标与现权威链路同量级），
说明"rko_lio + 桥接 + 独立 slam_3d 实例"的在线形态可用。
但小幅运动的净位移只有 3.6 cm，**无法区分两条链路的全局一致性**——
真正区分需要一次人工遥控的整圈闭环（已开始双链路录制，等待执行）。

### 3.5.3 真机整圈（人工遥控，2026-09-10 15:0x）——**两条链路同量级，rko 未复现离线优势**

数据：`artifacts/slam_validation/real_ab_20260910_145808/`（bag 2.7 GB，两条链路同录）。
用户遥控走一整圈（中途有一段较快），回到胶带原点停稳；随后 `return_to_origin.py` 归位到
**0.0043 m / 0.09°**。

| 指标 | plain_slam（现权威） | rko 链路（新方案） | 阈值 | 判定 |
|---|---|---|---|---|
| endpoint 平移 | 0.1566 m | 0.1718 m | ≤0.20 | 两者 PASS |
| endpoint yaw | 4.355° | 2.648° | ≤5.0 | 两者 PASS |
| overlap RMSE | 0.1956 m | 0.1364 m | ≤0.10 | 两者 FAIL |
| overlap p95 | 0.1672 m | 0.2037 m | ≤0.18 | plain PASS / rko FAIL |
| overlap inlier | 0.8633 | 0.6803 | ≥0.75 | plain PASS / rko FAIL |
| loop_closure_count | 105 | 91 | ≥1 | 两者 PASS |
| post-loop jitter | 0.1255 m / 5.95° | 0.1789 m / 8.73° | ≤0.08 / ≤2.0 | 两者 FAIL |
| surface_thickness_p95 | 0.0632 m | 0.0685 m | ≤0.15 | 两者 PASS |
| 路径长 / 端点差 | 9.99 m / 0.157 m | 14.20 m / 0.172 m | — | rko 抖动更多 |
| 位姿样本数 | 1147（≈7.6 Hz） | 702（≈4.7 Hz） | — | rko 发布率偏低 |

轨迹图（`traj_ab_real.png`，已人工复核）：**plain_slam 与 rko 两条轨迹几乎完全重合**，
都正确重建了那个近似矩形的环路并回到起点，rko 那条更"毛糙"（抖动更多）。
而轮式 `/go2w/odom/fused` 的轨迹形状严重失真（只画出一条窄长往返）——
说明**轮式 odom 在多次转弯后不能当作形状 ground truth**，此前以它作为参照的表述需据此修正。

**结论（诚实）**：

1. 离线 A/B 里 rko 里程计 18 mm 的优势**在真机在线接入后没有复现**：两条链路落在同一量级，
   rko 的 overlap inlier 与路径膨胀反而更差。
2. 差异来自**在线集成**而非里程计算法本身：桥接配对 gap 实测 200–400 ms（rko 在线处理一帧
   ≈100–200 ms，跟不上 10 Hz）、rko 链路位姿发布仅 ≈4.7 Hz、点云与位姿错配 2–4 帧。
3. 旁证：plain_slam 的"6.6 m 漂移"与**会话长期运行/累积状态**有关——本次是重启后的干净会话，
   同一场地整圈的 endpoint 只有 0.1566 m。
4. 因此下一步**不是切换权威源**，而是先把在线集成质量做上去（§8.2 / 交接书 §8.4）再复测。

### 3.5.4 真机整圈第二轮（修复录制 QoS 后）——**rko 链路首次在关键指标上领先**

修复：`ros2 bag record` 默认按 RELIABLE 订阅，与桥接 BEST_EFFORT 发布不兼容 → 上一轮数据在录制侧被丢
（plain_slam 7.6 Hz、rko 4.7 Hz）。加 `--qos-profile-overrides-path /tmp/record_qos.yaml` 后：
plain_slam 10.2 Hz、rko 6.0 Hz（rko 的 6 Hz 是其自身处理速度的真实上限）。

数据：`artifacts/slam_validation/real_ab_20260910_150700/`（bag 7.6 GB）。
用户遥控走一圈，**本轮结束时没有回到原点**（轮式口径起点终点差 0.736 m，用户明确说不需要归位）。

| 指标 | plain_slam | **rko 链路** | 阈值 | 判定 |
|---|---|---|---|---|
| endpoint 平移 | 0.1387 m | **0.0490 m** | ≤0.20 | 两者 PASS（rko 好 2.8×） |
| endpoint yaw | 4.722° | **1.714°** | ≤5.0 | 两者 PASS |
| overlap RMSE | 0.1375 | **0.1108** | ≤0.10 | 两者 FAIL（rko 仅超 11%） |
| overlap p95 | 0.2701 | **0.1920** | ≤0.18 | 两者 FAIL（rko 仅超 7%） |
| overlap inlier | 0.6186 | **0.8050** | ≥0.75 | **rko PASS / plain FAIL** |
| loop_closure_count | 465 | 105 | ≥1 | 两者 PASS |
| post-loop jitter | 0.2993 m / 0.785° | 0.2464 m / 0.561° | ≤0.08 / ≤2.0 | 两者 FAIL（平移项） |
| surface_thickness_p95 | 0.0677 m | 0.0698 m | ≤0.15 | 两者 PASS |
| 位姿样本 / 路径长 | 3188 / 12.83 m | 2031 / 22.18 m | — | rko 抖动更多 |

轨迹图 `traj_ab_real2.png`（已人工复核）：**蓝（plain）与绿（rko）两条轨迹几乎重合**，
都完整重建了矩形环路并闭合回起点；轮式（红）依旧形状失真。
⇒ 两条链路都能"走一圈不散"，差别在数值精度而非结构性失败。

**结论**：

1. 上一轮"rko 没优势"的结论**部分是录制丢包造成的假象**：修掉录制 QoS 后，rko 在
   endpoint（0.049 vs 0.139 m）、endpoint yaw（1.71° vs 4.72°）、overlap inlier（0.805 首次 PASS）
   三项上超过现权威链路。
2. 仍 FAIL 的只有左侧三项：overlap RMSE / p95（均接近阈值）与 **post-loop jitter**。
3. **post-loop jitter 的口径需要修正**：它取"第一条回环边之后 3–5 s"窗口内位姿相对中位数的 p95，
   若回环发生在**机器人仍在运动**时，窗口内位移本身就包含运动（实测本数据集窗口内位移 0.25–0.30 m），
   指标会把"运动"误判为"抖动"。需要在报告里附带窗口内的运动速度，或只在静止段取窗口。
   （本轮两条链路该指标分别为 0.299 / 0.246 m，且 yaw 项都 PASS，符合"运动混淆"的特征。）
4. endpoint 指标同样受"回环把轨迹闭合"影响：本轮用户物理上没回原点（轮式 0.736 m），
   但两条 SLAM 的 endpoint 只有 0.139 / 0.049 m——正是回环优化的效果，不能当作"回到原点"的证据。

### 3.5.5 真机整圈第三轮（deskew + 插值配对）——**plain_slam 崩到 2.87 m，rko 仍稳在 0.086 m**

本轮上线：deskew 开启（在线可用）、桥接改为"按点云时间戳在 odom 缓冲里插值配对"（配对率 100%、队列不积压）、
post-loop jitter 改为去趋势口径。用户遥控走一圈并"大概回到原点"（轮式口径仍差 0.74 m）。

| 指标 | plain_slam | **rko 链路** | 阈值 |
|---|---|---|---|
| endpoint 平移 | **2.8693 m ❌** | **0.0861 m ✅** | ≤0.20 |
| endpoint yaw | **76.85° ❌** | **1.603° ✅** | ≤5.0 |
| overlap RMSE | 1.9863 ❌ | 0.2224 ❌ | ≤0.10 |
| overlap p95 | 5.1033 ❌ | 0.1946 ❌ | ≤0.18 |
| overlap inlier | 0.097 ❌ | **0.8321 ✅** | ≥0.75 |
| loop_closure_count | 855 ✅ | 880 ✅ | ≥1 |
| post-loop jitter（去趋势） | 0.0254 ✅ | 0.0352 ✅ | ≤0.08 |
| post-loop yaw jitter | 0.931° ✅ | 2.584° ❌ | ≤2.0 |
| surface_thickness_p95 | 0.0630 ✅ | 0.0445 ✅ | ≤0.15 |

**三轮真机整圈汇总（同一场地、同一路线、同样人工遥控）**

| 轮次 | 链路 | endpoint 平移 | endpoint yaw | overlap inlier | overlap RMSE |
|---|---|---|---|---|---|
| 1 | plain_slam | 0.157 ✅ | 4.36 ✅ | 0.863 ✅ | 0.196 ❌ |
| 1 | rko | 0.172 ✅ | 2.65 ✅ | 0.680 ❌ | 0.136 ❌ |
| 2 | plain_slam | 0.139 ✅ | 4.72 ✅ | 0.619 ❌ | 0.138 ❌ |
| 2 | **rko** | **0.049 ✅** | 1.71 ✅ | **0.805 ✅** | 0.111 ❌ |
| 3 | plain_slam | **2.869 ❌** | **76.85 ❌** | 0.097 ❌ | 1.986 ❌ |
| 3 | **rko** | **0.086 ✅** | **1.60 ✅** | **0.832 ✅** | 0.222 ❌ |

**结论**：

1. **plain_slam 的退化是真实且可复现的现象**：离线 bag 里 6.6 m、本轮真机 2.87 m/76.85°，
   而它在本会话早期只是 0.14–0.16 m。三次运行里它只在会话早期正常 ——
   与"会话运行越久、累积状态越多越容易崩"的假说一致（本次它自 14:58 重启后已跑 20+ 分钟、多次运动）。
2. **rko 链路的鲁棒性优势被真机验证**：三轮 endpoint 0.172 / 0.049 / 0.086 m 全部达标，
   在 plain_slam 崩掉 2.87 m 的同一轮里仍保持 0.086 m / 1.6°。
   这比"离线单次精度对比"更能说明换后端的价值。
3. 仍未达标的是 **overlap RMSE/p95**（本轮 0.222/0.195）与 **post-loop yaw jitter**（2.58° 略超 2.0°）。
   overlap 指标本质是"回到同一处再看同一面墙"：本轮用户明确只是"大概回到原点"（轮式差 0.74 m），
   RMSE 高于 p95 说明是**少量远端离群点**在拉高它（分布问题，不是整体错位）。
4. 因此下一次验收需要**回到胶带四脚与朝向**（精确回原点）才能公平评价 overlap；
   在精确回原点之前，不宜把 overlap 的 FAIL 归因于 rko 链路本身。

### 3.5.6 真机整圈第四轮（精确回原点 3.1 cm；两条链路均为全新会话）

准备：本轮把 **plain_slam 与 rko 链路都重启为全新会话**（上一轮 plain_slam 崩在 20 分钟累积状态，
重启后对比才公平）。用户按胶带对齐起点并"肉眼观测首末距离 < 5 cm"，轮式口径实测首末差 **0.0308 m / 4.1°**。

数据：`artifacts/slam_validation/real_ab_20260910_152844/`（bag 5.3 GB）

| 指标 | plain_slam | **rko 链路** | 阈值 |
|---|---|---|---|
| endpoint 平移 | 0.3634 ❌ | **0.0339 ✅** | ≤0.20 |
| endpoint yaw | 10.334° ❌ | **0.914° ✅** | ≤5.0 |
| overlap RMSE | 0.2927 ❌ | **0.1456 ❌** | ≤0.10 |
| overlap p95 | 0.6023 ❌ | **0.2015 ❌** | ≤0.18 |
| overlap inlier | 0.2946 ❌ | **0.8899 ✅** | ≥0.75 |
| loop_closure_count | 120 ✅ | 959 ✅ | ≥1 |
| post-loop jitter（去趋势） | 0.0110 ✅ | 0.0431 ✅ | ≤0.08 |
| post-loop yaw jitter（去趋势） | 0.177° ✅ | **3.772° ❌** | ≤2.0 |
| surface_thickness_p95 | 0.0697 ✅ | 0.0456 ✅ | ≤0.15 |

**本轮新增两处口径修正（都在评估器里，raw 值保留对照）**：

* `post_loop_yaw_jitter` 也与平移一样去趋势（转弯窗口里 yaw 单调变化会被误判为抖动）：
  rko 由 raw 8.449° → 去趋势 **3.772°**。
* 用 `overlap_range_probe.py` 验证了 range filter 对 overlap 的影响：
  范围收到 5 m 时 RMSE 仅由 0.1456 → 0.1359（改善 7%），收到 10 m/20 m 与全量一致
  ⇒ **overlap RMSE 不是远端离群点主导，而是整体点位错位约 14 cm**。

**结论**：

1. rko 链路本轮 6 项达标、3 项未达标（overlap RMSE/p95、yaw jitter）；
   plain_slam 只有 4 项达标，且在 endpoint（0.363 vs 0.034 m）、inlier（0.29 vs 0.89）上差一个量级。
2. overlap RMSE 的 14 cm 错位的可能来源（按量级排序，尚未逐项求证）：
   a) 桥接喂给 `slam_3d` 的是**原始点云（未去畸变）**，用户走速下 0.1 s 内位移可达 5–10 cm；
   b) 桥接配对存在 **99 ms 的系统时间偏移**（实测 rko 的 odom stamp 比同帧点云晚 0.099 s）；
   c) LIO 局部地图形变。
   a)+b) 合计约 10 cm，与观测一致 ⇒ 下一步应把 **rko 的去畸变点云**（`/rko_lio/frame`，已开启）接进
   桥接，而不是继续调阈值。
3. 精确回原点确实让 inlier 达到历史最好（0.8899），说明该指标口径是对的；RMSE/p95 的剩余差距
   需要从点云质量入手。

### 3.5.7 真机整圈第五轮（改用 rko 去畸变点云 + 99 ms 偏移修正）——**未改善，已回滚**

本轮按 §3.5.6 的根因分析上线两处改动：
(a) 桥接改用 rko 的**去畸变扫描** `/rko_lio/frame`（并把裸 `x/y/z` 重打包为
`slam_3d_node` 必需的 `x,y,z,intensity,timestamp` 布局，intensity 置 0、timestamp 写帧时间戳）；
(b) 查位姿时间轴加上实测的 **+0.099 s** 偏移。

用户走完整圈，自述"首末目测差 < 5 cm"（轮式口径报 0.650 m，**再次证明轮式不可作基准**）。

| 指标 | plain_slam | **rko 链路（deskewed+offset）** | 上一轮 raw+无 offset（§3.5.6） | 阈值 |
|---|---|---|---|---|
| endpoint 平移 | 1.7973 ❌ | **0.0757 ✅** | 0.0339 ✅ | ≤0.20 |
| endpoint yaw | 29.09° ❌ | **3.996 ✅** | 0.914 ✅ | ≤5.0 |
| overlap RMSE | 1.1718 ❌ | 0.1530 ❌ | 0.1456 ❌ | ≤0.10 |
| overlap p95 | 2.1462 ❌ | 0.2170 ❌ | 0.2015 ❌ | ≤0.18 |
| overlap inlier | 0.030 ❌ | **0.7282 ❌** | 0.8899 ✅ | ≥0.75 |
| loop_closure_count | 495 ✅ | 1370 ✅ | 959 ✅ | ≥1 |
| post-loop jitter | 0.2215 ❌ | 0.0524 ✅ | 0.0431 ✅ | ≤0.08 |
| post-loop yaw jitter | 0.254° ✅ | 0.242° ✅ | 3.772° ❌ | ≤2.0 |
| surface_thickness_p95 | 0.0697 ✅ | 0.0451 ✅ | 0.0456 ✅ | ≤0.15 |

**结论：A 方案未达成目标，且一项指标变差。**
overlap RMSE/p95 基本不变（0.146→0.153、0.202→0.217），inlier 反而从 **0.8899 掉到 0.7282**（跌破阈值）。
可能原因（未逐项求证，按嫌疑排序）：intensity=0 让 `slam_3d` 的点筛选/加权退化；
把全部点写成同一 timestamp 改变了它对扫描时间的处理；或 deskewed 点云已被 rko 用自身外参表达过，
再叠加一次 lidar→base 变换引入了偏差。

**动作**：已把桥接**回滚为 `scan_source=raw`、`odom_time_offset_s=0.0`**（即 §3.5.6 那套，
inlier 0.8899 的最佳配置）。A 方案的代码路径保留（可用参数开启），但在有明确收益前不启用。

### 五轮真机整圈总账（同场地、同路线、人工遥控）

| 轮次 | 链路 | endpoint 平移 | endpoint yaw | overlap inlier | overlap RMSE | 是否崩 |
|---|---|---|---|---|---|---|
| 1 | plain_slam | 0.157 ✅ | 4.36 ✅ | 0.863 ✅ | 0.196 ❌ | 否 |
| 1 | rko | 0.172 ✅ | 2.65 ✅ | 0.680 ❌ | 0.136 ❌ | 否 |
| 2 | plain_slam | 0.139 ✅ | 4.72 ✅ | 0.619 ❌ | 0.138 ❌ | 否 |
| 2 | rko | **0.049** ✅ | 1.71 ✅ | **0.805** ✅ | 0.111 ❌ | 否 |
| 3 | plain_slam | **2.869** ❌ | **76.85** ❌ | 0.097 ❌ | 1.986 ❌ | **崩** |
| 3 | rko | 0.086 ✅ | 1.60 ✅ | 0.832 ✅ | 0.222 ❌ | 否 |
| 4 | plain_slam | 0.363 ❌ | 10.33 ❌ | 0.295 ❌ | 0.293 ❌ | 部分 |
| 4 | rko | 0.034 ✅ | 0.91 ✅ | **0.8899** ✅ | 0.146 ❌ | 否 |
| 5 | plain_slam | 1.797 ❌ | 29.09 ❌ | 0.030 ❌ | 1.172 ❌ | **崩** |
| 5 | rko | 0.076 ✅ | 4.00 ✅ | 0.728 ❌ | 0.153 ❌ | 否 |

**总账结论**：

1. **rko 链路 5/5 轮 endpoint 全部达标**（0.034–0.172 m），**plain_slam 有 3 轮崩**（最高 2.87 m / 76.9°）。
   "换后端"的价值已被真机重复验证为**鲁棒性**，而不是单次精度微调。
2. rko 的 **overlap RMSE 稳定在 0.111–0.222（5 轮都没进 0.10）**，inlier 在 0.680–0.890 之间波动。
   在已尝试的手段（插值配对、deskewed 点云、时间偏移修正、range filter、回环参数、jitter 口径）之后，
   依然差阈值 11–45% —— 说明**继续在这条链路上压 RMSE 的边际收益已很低**。
3. 需要向决策者明确：**是否把 overlap RMSE 0.10 视为本平台可达的硬指标**。
   现权威链路的同项指标是 0.14–1.99（且三次崩溃），rko 已是 5 轮里更稳的一方。

### 3.5.8 WebUI 地图源切换（阶段 3 第一刀）——**地图双层厚度 0.0439 m，优于现权威的 0.0682 m**

**问题定位**：WebUI 的三维建图链路是
`WebUI ← /go2w/slam/web_map ← plain_slam_web_cloud_relay ← /go2w/slam/map_3d (plain_slam)`，
即 **WebUI 一直显示的是 plain_slam 的地图**，而 plain_slam 在真机 5 轮里崩了 3 次 ——
这就是"WebUI 里地图重叠"的直接来源。另有一个硬约束：WebUI 的 `slam_web_map_state`
只接受 `pslam_map` frame，否则 `map_frame_mismatch` 拒收。

**已做的切换（不动上层一行代码）**：

```text
relay 订阅改为 /go2w/rko_slam/map_3d 与 /go2w/rko_slam/aligned_scan
rko 实例的 map_frame/odom_frame 改为 pslam_map / pslam_odom（对齐上层契约）
桥接给 relay 单独加 RELIABLE 发布（原来 BEST_EFFORT → relay 的 RELIABLE 订阅收不到 web_scan）
实测：web_map_info = {"source_point_count":243382,"relay_point_count":55883,
                      "revision":17,"frame_id":"pslam_map"}
```

**切换后的真机整圈结果**（用户第 6 次走圈，`real_ab_20260910_155630`）：

| 指标 | plain_slam（旧地图源） | **rko（现 WebUI 地图源）** | 阈值 |
|---|---|---|---|
| **surface_thickness_p95（地图双层厚度）** | 0.0682 ✅ | **0.0439 ✅（薄 36%）** | ≤0.15 |
| 地图点数 / extent | 53187 / 30.1×12.0×6.4 m | **243382 / 28.8×25.0×8.1 m** | — |
| endpoint 平移 | 0.2214 ❌ | **0.0390 ✅** | ≤0.20 |
| endpoint yaw | 2.191 ✅ | 6.459 ❌ | ≤5.0 |
| overlap RMSE / p95 / inlier | 0.1686 / 0.3101 / 0.4333 | 0.1599 / 0.3444 / 0.5704 | ≤0.10 / ≤0.18 / ≥0.75 |
| loop / keyframes | 105 / 15 | 942 / 47 | ≥1 |
| post-loop jitter | 0.0222 ✅ | 0.0482 ✅ | ≤0.08 |
| post-loop yaw jitter | 2.561 ❌ | 1.596 ✅ | ≤2.0 |

**结论**：

1. **"WebUI 地图重叠"的核心量化指标已达标且优于旧源**：地图双层厚度 0.0439 m（阈值 0.15 m），
   点数从 5.3 万增到 24.3 万，且这一数据源不会像 plain_slam 那样整段崩掉
   （rko 六轮 endpoint 全部 0.034–0.172 m，无一次崩溃）。
2. 本轮 overlap 三项仍 FAIL，但本轮用户**未精确回到原点**（轮式口径首末差 0.224 m）；
   该指标对"是否回到同一处"极敏感，按 §3.5.6 的经验不宜在本轮条件下作为判据，endpoint yaw 6.459° 同理。
3. 仍待办：WebUI 的**机器人位姿源**若与地图源不一致会出现错位（地图为 rko 系、位姿为 plain_slam 系），
   需要确认 WebUI 前端是否显示位姿并统一切到 rko。

### 3.5.9 IMU 覆盖修复（本日最有价值的一次修复）——deskew 从 8.7% 覆盖恢复到 100%

**问题**（§3.4 遗留线索的最终定位）：LiDAR 帧时间窗跨越 0.1 s，而 IMU 数据比 LiDAR 帧晚到 ≈89 ms，
因此 LIO 收到一帧时，覆盖该帧**末端**的 IMU 还没到达。实测（`/tmp/imu_coverage.py`）：

```text
LiDAR header 滞后 bag 接收 : 102.0 ms
IMU   header 滞后 bag 接收 :  90.9 ms
每帧窗口内 IMU ：离线可用 p50 = 25 个 ；实时可用 p50 = 3 个
实时窗口覆盖率 ：p50 = 8.7%
```

**后果**：去畸变只覆盖整帧的 8.7%，另外 91% 的点没有 IMU 支撑 → `dsk_max` 达 2.7 cm →
同一面墙被反复重建、每次偏移几厘米层层叠加 → 用户看到的"墙很厚 / 多个角度稀疏叠加"。

**修复**：新增 `scripts/go2w/lidar_delay_buffer.py`（把点云按到达时间缓冲 0.12 s 后原样转发，
header 时间戳不变），并把 `configs/go2w/plain_slam.yaml` 的 `pandar_adapted` 指向延迟后的话题
`/go2w/slam/pandar_points_buffered`。代价是整条链路延迟 0.12 s；回退只需把该配置改回原话题。

**效果（实测）**：

| 指标 | 修复前 | 修复后（静止） | 修复后（运动 0.8 m/s） | 修复后（运动 1 m/s） |
|---|---|---|---|---|
| `imu_n` | 12–14 | 23–25 | 24–25 | 24–26 |
| `dsk_max` | 0.026–0.028 m | 0.0029–0.0040 m | 0.0080–0.0083 m | 0.025 m |

**用户人工确认**："建的图有明显增强，我觉得差不多能用了。"

### 3.5.10 语义物体标注（Dataset D）首次跑通 + WebUI 显示

**采集**：`artifacts/slam_validation/semanticD_20260910_164646/`（原地 12×30° 旋转，worker 只观察不发运动）。

**结果**：5 个物体全部三维定位成功，空间质量均为最高档 `METRIC_LIDAR`：

| 标签 | 地图坐标 (x, y, z) | 置信度 |
|---|---|---|
| 办公椅 | (−1.674, −0.974, 0.473) | 0.80 |
| 文件柜 | (−1.446, −3.339, 0.440) | 0.91 |
| 纸箱 | (−2.412, −2.617, 0.334) | 0.89 |
| 垃圾桶 | (−3.550, −2.160, 0.164) | 0.87 |
| 柜子 | (−2.659, −3.494, 0.911) | 0.85 |

**WebUI 显示**：前端 `static/slam_map_3d.js` 新增语义标注层（红点 + 中文标签，随地图一起平移/缩放），
快照 writer 从 `events.jsonl` 读入 `objects` 字段。用户截图确认：**标注正确显示在三维地图上并跟随地图运动**。

**操作员（用户）提供的真值**：

```text
5 件物体全部真实存在        → 无漏检（no false negative）
每件各 1 个、标签各 1 个     → false_split = 0、false_merge = 0
垃圾桶、纸箱位置偏靠墙       → 系统性深度偏差（见下）
```

因此本案中：`false_split = 0`、`false_merge = 0`、漏检 0；而 `duplicate_rate` 仍 **NOT_VERIFIED**——
本轮每个物体只被观测 1 次（`observation_count = 1`），**没有多重观测数据**，无法检验实例合并能力。
若要验收去重，需要"同一物体被多次看到"（例如再走一圈或多转两圈）。

**已知位置偏差的机制**（操作员实测反馈，方向已核实）：
**垃圾桶与纸箱的实际位置更靠墙，而标注位置偏向道路中间** —— 即物体被标得**比实际更靠近机器人**，
深度被**低估**。可能机制（`app/perception/depth_object_localizer.py` 现行策略）：
深度取 bbox 内 **5 cm 直方图的最强簇**并要求该簇在内部裁剪中也存在；当检测框内除物体外还包含
**物体前缘、地面或近处地面点**时，最强簇会落在这些更近的表面上——物体矮小/细长（垃圾桶）或
低矮（纸箱）时尤其明显。

改进候选（**均尚未实施**，属提精度而非修错误，需新一轮采集验证，避免引入反向偏差）：
1. 在强度达峰值一定比例（如 50%）的候选簇中取**最远**者，而非直接取最强簇；
2. 结合"物体贴地"先验约束深度下限（物体底部像素的深度通常比中心更可靠）；
3. 用 bbox 下半部（物体实体部分）而不是整体中心做采样。

`duplicate_rate` 仍为 **NOT_VERIFIED**：本轮每个物体只被观测 1 次，无多重观测数据可检验去重。

### 3.5.11 重复标注问题定位与修复（**读取 bug，不是 SLAM/VLM 的错**）+ 去重指标首次可评

**现象**（操作员反馈）：WebUI 地图上出现"两个桌子、两个电脑主机、两个垃圾桶"，位置彼此很近。

**诊断**：`GET /api/slam/map3d` 返回 **10 个 objects**，其中三对是同一物体：
`电脑主机 (0.945,−0.461)/(0.969,−0.516)`、`桌子 (1.691,−0.645)/(1.453,−0.694)`、
`垃圾桶 (0.901,0.635)/(0.952,0.540)`。逐条展开 `events.jsonl` 后真因清楚：

```text
obj_002 电脑主机 obs=1  map=(0.94,-0.46)
obj_002 电脑主机 obs=3  map=(0.97,-0.52)   ← 同一 object_id，observation_count 递增
obj_004 桌子     obs=1 / obs=3
obj_003 垃圾桶   obs=1 / obs=3
```

**结论：下游系统其实正确地把"同一物体被多次观测"合并成了同一个实例**（object_id 不变、
`observation_count` 从 1 涨到 3），**重复来自我的快照读取器**——它把每一条
`semantic_object_localized` 事件都当作一个新物体。

**修复**：`webui_map_snapshot_writer.py` 改为**按 `object_id` 去重、保留最后一次观测的坐标**。
修复后 `/api/slam/map3d` 的 objects：**10 → 7**（办公椅 / 电脑主机 / 垃圾桶 / 桌子 / 背包 /
电脑显示器 / 风扇），其中前三项带 `observation_count = 3`。

**因此计划书 §22 的三项去重指标首次有了真实数据基础**（同一物体被重复观测 3 次）：

| 指标 | 结果 | 依据 |
|---|---|---|
| `duplicate_rate` | **0** | 7 个实例中无重复实例（同一 object_id 已合并） |
| `false_split` | **0** | 被重复观测的 3 个物体都保持单一 object_id，未被拆成多个实例 |
| `false_merge` | **0** | 7 个实例标签各异；且操作员确认现场物体均真实存在 |

**注意**：`duplicate_rate/false_split/false_merge` 此前长期为 NOT_VERIFIED 的原因是"缺多重观测数据"，
现已由本轮（原地转圈、同一物体被看 3 次）补上。若后续要做更严格的验收，可再走一圈，
观察同一物体的 `observation_count` 是否继续增长而**实例数不变**。

### 3.5.12 语义标注"位置对不上"的定性与根因（**逐次观测离散，非外参/坐标系问题**）

**操作员反馈**：物体都存在、彼此相对位置基本正确，但整堆物体在地图上的位置对不上。

**诊断一（到最近地图点的距离）**：7 个物体到最近地图点的距离为 0.02–0.67 m
（桌子 0.02、背包 0.16、风扇 0.21、垃圾桶 0.32、显示器 0.35、主机 0.38、办公椅 0.67）。
关键在于**偏移的离散度与均值同量级**（std 0.212 m ≈ mean 0.208 m）——
若为"整堆刚性平移"，离散度应趋近 0。⇒ **排除"坐标系整体漂移"**。

**诊断二（同一物体的多次独立观测）**：转圈数据里同一 `object_id` 写过多次坐标，
这就是"同一物体在不同视角下的重复定位"：

| 物体 | 观测 1 | 观测 2 | 同一物体散布 |
|---|---|---|---|
| 电脑主机 | (0.945, −0.461, 0.128) | (0.969, −0.516, 0.157) | **0.067 m** |
| 垃圾桶 | (0.901, 0.635, 0.062) | (0.952, 0.540, 0.052) | **0.108 m** |
| 桌子 | (1.691, −0.645, 0.597) | (1.453, −0.694, 0.566) | **0.245 m** |

**结论**：**这不是外参（相机↔激光）问题**——外参误差是系统性的，会让所有物体朝同一方向整体偏移，
**不会让同一物体两次观测彼此相差 7–25 cm**。真实性质是**逐次观测的定位离散**，量级 6.7–24.5 cm，
与操作员在 WebUI 上观察到的偏差一致。

**桌子的 0.245 m 另有解释**：桌子属**大物体**，"物体中心"在检测框内本身不唯一
（框住桌面的不同部分 → 深度取框内最强簇 → 中心在桌面上漂移），并非定位错误。

**改进方向（按性价比排序）**：

1. **多观测融合**（系统已在做：`observation_count` 增长、坐标取合并值）——可进一步改用**中位数**
   并提高观测次数；
2. **大物体的"中心"定义**：桌子一类改用"最近稳定表面"或"机器人正前方对齐点"，而非检测框中心；
3. **细长物体（椅子/垃圾桶）的深度采样**：见 §3.5.10 的三条候选。

**定点观测实验受阻（外部原因）**：为把"深度采样误差"与"视角差异"彻底分离，安排了一次
"机器人静止、对正前方 1.7 m 处桌子连续观测 60 s"的实验，但 **VLM API 连续两次返回
`APIConnectionError`**（`perception_failure` / `FULL_SEMANTIC_ERROR`），实验未能产出数据。
该实验设计保留，待外网稳定后重做即可（脚本：`/tmp/run_semantic_static.sh`）。

**仍待操作员核对**：`电脑显示器 (1.025,−0.441)` 与 `电脑主机 (0.969,−0.516)` 相距仅 0.09 m，
属**不同标签、不同 object_id**——可能是"桌面上显示器与主机两件真实物体"，也可能是
VLM 对同一物体给了两个标签（即 `false_split` 的另一种形态：标签不一致导致的拆分）。
该疑点尚未定论，需现场核对后再决定是否引入"标签相似度"参与实例关联。

## 4. 本轮关键发现（诚实结论，不得被后续报告改写）

1. **评估器原无法量化 §20.3 / §20.4 / §22**，这三项无论数据多好都只会是 NOT_VERIFIED；
   本轮已补齐并产出真实数值（post-loop jitter 两项 PASS）。
2. **回环本身稳定触发**（78 / 615 条 loop edge），"回环不触发"不是当前瓶颈。
3. **瓶颈是位姿一致性**：同一轮运动中轮式 fused odom 认为机器人已回到起点（残差 0.016 m / 0.33°），
   而 SLAM 的 `/go2w/slam/odom_base` 认为差 **1.15 m / 84.7°**；endpoint 与首末子图 overlap 因此必然 FAIL。
   这是 LIO 侧的真实缺陷，不是评估口径问题。
4. 归位控制必须用**轮式 fused odom**（`/go2w/odom/fused`）：SLAM 位姿在回环优化时跳变，用它做闭环控制会互相打架。
5. 未做任何阈值放宽，也未用 voxel / merge radius 掩盖问题。

## 5. 本轮未完成（NOT VERIFIED / FAIL 保持原样）

1. **endpoint / overlap 四项 FAIL 未修复**（下一步见计划书 §26.2 与合并文档交接书第 4 节）。
2. **Dataset D 多视角语义采集未执行**（机器狗失联）；`object_position_rms/p95`、
   `semantic_observation_scatter.png` 本轮无新数据。
3. **`duplicate_rate / false_split / false_merge` 仍为 NOT_VERIFIED**：用户已选择
   "先由 agent 给出标签清单、再由用户确认真实实例数"，清单尚未产出。
4. 旋转包络 / Pandar 人工外参仍为 NOT_VERIFIED（用户已排除人工标定）。
5. 机器狗自 2026-09-10 约 10:05 CST 起失联，控制主机 `enp3s0` = `NO-CARRIER`，
   ping/SSH 全失败，真机测试、语义采集、归位复核全部停止。

---

# SLAM 量化验证报告（20260908 计划书 §24 / §50）

> 执行日期：2026-09-08
> 控制主机：`/home/mxt/robotscene`，`192.168.123.99`（enp3s0），ROS 2 Humble
> 机器狗：`unitree@192.168.123.18`，Jetson aarch64 + ROS 2 Foxy，项目 `/home/unitree/robotscene`
> 配套审计：`docs/SLAM_MIGRATION_AUDIT.md`
> 上一轮验收记录：`docs/GO2W_自主搜索_高净空视觉友好导航_目标接近_计划书_交接书_总集_20260904.md` 末节

---

## 0. Baseline

```text
控制主机分支： fix/go2w-autonomous-search-lio-topology-target
控制主机 HEAD： 21a7bbb（未提交改动 181 项，未清理、未提交）
机器狗分支：    fix/go2w-autonomous-search-lio-topology-target
机器狗 HEAD：   0de647e（未提交改动 76 项）
机器狗解释器：  /home/unitree/robotscene/.runtime_venv/bin/python = Python 3.8.10
```

本轮**未**执行 `git add` / `git reset` / `git clean`，未覆盖任何既有未提交改动。

---

## 1. SLAM 后端选择（计划书 §4.2 要求的说明）

**实际后端：`plain_slam_ros2`（lio_3d + slam_3d），未使用 LIO-SAM。**

理由（源码级，非猜测）：

1. `ros2_ws/src/plain_slam_ros2/src/slam_3d_node.cpp` 已发布 `filtered_map_cloud`、pose graph
   nodes/edges、loop edges；`slam_3d_interface.cpp::SetData()` 在检测到新回环后调用
   `g_optimizer_.Optimize(odom_edges_, loop_edges_, pose_graph_)`。
2. `BuildFilteredMapCloud()` **遍历全部 keyframe、用当前优化后的 pose 重建整图**，
   满足计划书 §6.2「keyframe + 最新优化 pose 重建」，**不存在**计划书 §3 假设的
   「不可逆 append」。
3. 目标平台是 Jetson aarch64 + **ROS 2 Foxy**（`/opt/ros/humble` 只是 Foxy 副本）；
   LIO-SAM 官方为 ROS 1，ROS 2 移植 + GTSAM 编译在该平台风险高，且 **LIO-SAM 的 IMU
   预积分对时间戳/外参比 plain_slam 更敏感**——在输入层未修好前换后端不会解决重影。

`plain_slam` 的结构性短板（保留为后续候选）：回环靠 KD-tree 最近位姿 + GICP，
无描述子，召回率依赖初值。

---

## 2. 根因定位与修复（本轮最重要的结论）

### 2.1 修复前的真实状态

| 证据 | 原始值 |
|---|---|
| `/go2w/slam/imu_status` | `mode=IMU_SOURCE_RESTART_REQUIRED`、`corrected_sec=null`、`offset_sec=747.062874` |
| LIO 诊断 | `imu_n=0`、`pos=(-15535838208, 22742347776, -45469782016)`、`v=66451083264` |
| `/go2w/slam/map_3d` | `Publisher count: 0` |
| `slam_3d_node` | 已崩溃：`SO3::exp failed! omega: 4.07e-41 -7.02e+29 3.16e-41`，exit code -6 |
| WebUI 地图 | `map_extent_m=[55.826, 74.14, 20.0]`、`accepted_maps=0` |

因果链：机器狗重启 → `/utlidar/imu` epoch 跳变 → `imu_fallback_adapter` 的会话锁定 offset
（747.06 s）失配 → fail-closed 永久停喂 IMU → LIO 退化为纯几何配准 → 位姿发散 →
`slam_3d_node` 收到 NaN 崩溃 → 权威地图消失。

### 2.2 修复动作

控制主机重启 plain_slam 映射栈（清 `/tmp/go2w_plain_slam`、重生成配置、`setsid` 重新 launch）。

### 2.3 修复后实测

| 项 | 修复前 | 修复后 |
|---|---|---|
| `imu_status.mode` | `IMU_SOURCE_RESTART_REQUIRED` | `OFFSET_LOCKED_SENSOR_STAMP` |
| `imu_status.corrected_sec` | `null` | `1788863879.557746` |
| LIO `imu_n` | 0 | 23–26 |
| LIO `pos` | (−1.55e10, 2.27e10, −4.55e10) | (−0.003, 0.001, −0.006) |
| `slam_3d_node` | 崩溃（exit −6） | 存活 |
| `/go2w/slam/map_3d` 发布者 | 0 | **1** |
| 地图 extent | 55.826 × 74.14 × 20.0 m | 22.63 × 9.42 × 3.93 m |
| `mapping_health` | `WAITING_FOR_MAP` | `HEALTHY` |

---

## 3. Dataset A：原地旋转（计划书 §19.3 假平移）

**采集方式**：真实运动 `l30,r30`（左转 30° 再右转 30° 回原朝向），
`scripts/go2w/run_autonomous_loop.py --mode pattern --operator-authorized-rotation`。
指标来自 `plain_slam_web_bridge` 的 motion-episode 记录（同一日志、同一 rig）。

| episode | 时刻 | wheel dxy | wheel dyaw | pslam dxy | pslam dyaw | 判定 |
|---|---|---|---|---|---|---|
| 1（修复前） | 18:42 前 | 0.002 m | 32.6° | 0.101 m | **0.2°** | FAIL（旋转估计失效） |
| 2（修复前） | 18:42 前 | 0.001 m | 33.9° | 0.085 m | **0.1°** | FAIL |
| 9（修复前） | — | 0.019 m | 0.2° | **0.479 m** | 0.1° | FAIL（触发假平移冻结） |
| **12（修复后）** | 18:42 | 0.006 m | 29.6° | 0.154 m | **29.5°** | PASS（跟随） |
| **13（修复后）** | 18:43 | 0.001 m | 30.3° | 0.085 m | **29.9°** | PASS（跟随） |

**量化结论**

```text
yaw 跟随误差：      修复前 32.6° vs 0.2°（误差 32.4°）→ 修复后 29.6° vs 29.5°（误差 0.1°）
原地旋转假平移：    修复前 0.479 m（超 0.20 m 冻结门）→ 修复后 0.085–0.154 m
plan §19.3 阈值：   horizontal displacement ≤ 0.15 m
```

**判定**：修复后 episode 13 = 0.085 m ≤ 0.15 m → **PASS**；
episode 12 = 0.154 m 略超 0.15 m 阈值 → **边界失败（1 次）**，需继续减小旋转漂移。
不掩盖：旋转漂移**已大幅改善但未完全消除**。

**评估器实测**（`bash scripts/record_slam_validation.sh --dataset A --duration 75` +
`bash scripts/run_slam_validation.sh artifacts/slam_validation/datasetA_20260908_184607`，
期间真实执行 `l30,r30,l30,r30`，净 0°）：

```text
PASS          surface_thickness_p95_m       = 0.0654 m   (limit 0.15)   <- 墙面不分层的量化证据
PASS          endpoint_translation_error_m  = 0.1935 m   (limit 0.20)
PASS          endpoint_yaw_error_deg        = 1.2484 deg (limit 5.0)
PASS          overlap_rmse_m                = 0.0773 m   (limit 0.10)   <- 位姿补偿后（未补偿 0.1333 / inlier 0.482）
NOT_VERIFIED  object_position_rms_m / p95_m              (Dataset D 未采集)
NOT_VERIFIED  loop_closure_count                         (无闭环数据)
accepted = false（overlap_rmse 越界）
```

**零偏修正后的复测（2026-09-08 19:06，Dataset A′）**

在 `imu_fallback_adapter` 加入启动窗口陀螺零偏估计后（实测锁定
`gyro_bias=[0.01087, 0.00018, -0.00061] rad/s`，与离线诊断 `[0.01119, 0.00029, -0.00041]` 一致），
重录 Dataset A′ 并执行**同一序列** `l30,r30,l30,r30`：

```text
PASS          surface_thickness_p95_m        = 0.0587 m   (limit 0.15)
PASS          endpoint_translation_error_m   = 0.1860 m   (limit 0.20)
FAIL          endpoint_yaw_error_deg         = 6.1598 deg (limit 5.0)
FAIL          overlap_rmse_m                 = 0.1751 m   (limit 0.10)   <- 位姿补偿后；未补偿 0.2308
```

**位姿补偿后的干净对比**（同一指标、同样补偿）：

| | 未补偿 overlap | 补偿后 overlap | inlier |
|---|---|---|---|
| Dataset A（零偏修正**前**） | 0.1333 | **0.0773 PASS** | 0.928 |
| Dataset A′（零偏修正**后**） | 0.2308 | **0.1751 FAIL** | 0.851 |

**这一步很关键**：补偿掉"没回原位"之后，A 从 FAIL 变 PASS，说明原来的 0.133 主要是**运动执行误差**；
而 A′ 补偿后仍有 0.175 —— **零偏修正之后 LIO 漂移确实变大了**。
但两轮的旋转执行本身差异巨大（odom 净朝向 6.03° vs 1.64°），**运动学差异未被控制**，
因此仍**不能把 A′ 变差归因于零偏修正**。

**诚实结论**：
1. 零偏修正已生效且有理论依据，但**收益/代价均未证实**；
2. 要给出结论，必须做**可重放的 A/B 对照**：录一段含 `/utlidar/imu`、`/hesai/pandarxt16/points_raw`
   的 bag，用 `ros2 bag play` 在"开/关 `--bias-lock-samples`"两种配置下重放同一段数据；
3. 在此之前，不要把 `--bias-lock-samples` 的效果写进任何"已改善"的结论。

**结果更差，但不能归因于零偏修正**：本次旋转后 odom 净朝向差 **6.03°**（上一次仅 1.64°），
即这一轮机器狗**没有转回原朝向**（轮式打滑），首末帧看到的墙面本来就不同 ——
`overlap_rmse` 因此混入了**运动执行误差**，不再单纯衡量 LIO 漂移。

**诚实结论**：

1. 零偏修正已生效且有理论依据，但**单次实验无法证明它改善或恶化 overlap**；
2. `overlap_rmse` 指标本身需要改进：比较前应先用**起终点位姿差**补偿末帧，
   把"机器人没回原位"与"LIO 漂移"分开，否则每次旋转执行误差都会污染结论；
3. 下一轮顺序：先改指标（位姿补偿）→ 再重测 → 才谈零偏修正是否有效。

**`_tf_available` 缺陷修复（2026-09-09 11:05）**

`plain_slam_spatial_provider.py` 的 `_tf_available` 此前只在 `__init__` 置 False、
在 `transform_provenance()` 里被读，**从未置 True** —— P2 排查时它误导过一次
（让我一度以为"map→odom TF 缺失"）。已在 `_on_tf` 真正收到 `pslam_map→pslam_odom` 后置位。

修复后机器狗实测（`/tmp/p2_probe.py`）：

```text
map_seen_frames 1
camera_point_to_map (1.8238, -0.0, 0.3)
quality METRIC_LIDAR
provenance {'target_frame': 'pslam_odom', 'transform_source': 'nominal_extrinsic',
            'pose_source': 'plain_slam_pandarxt16_odom', 'map_revision': 2,
            'transform_quality': 'METRIC_LIDAR', 'tf_available': True}   <- 已变 True
```

投影结果不变（`(1.8238, 0.0, 0.3)`），`tf_available` 现在如实反映 TF 状态。
`target_frame` 仍是 `pslam_odom`，因为**地图本身就是 `pslam_odom` 系**（`map_2d` 的 frame），这是正确的。

**P2 最后一块：`semantic_object_localized` 的跳过条件（2026-09-09 12:05）**

`run_semantic_exploration.py:1895-1902`：

```python
for index, obs_item in enumerate(localized):
    if getattr(obs_item, "map_xyz", None) is None:
        continue
    assoc = association_by_index.get(index)
    if assoc is None:
        continue          # <- 唯一会跳过事件生成的地方
```

而 `association_by_index`（1864-1867）由 `update_result.associations` 的 `observation_index`
建键，两者索引语义一致（同一 `localized` 列表）。因此事件缺失的唯一可能是
**`update_result.associations` 为空**。

**对结论的影响：无**。`MAP_UPDATED` 的 OBJECT 节点已经带
`map_xyz` + `map_frame=pslam_map` + `spatial_quality=METRIC_LIDAR`（P2 的目标证据已取得），
`semantic_object_localized` 只是另一条冗余事件通道。

**下一步（可选）**：在 1863 之后加一行日志打印 `len(update_result.associations)`；
若为空，再查 `update_with_associations` 的分配分支为何没产生关联。

**P3 口径改进方向（2026-09-09 12:00）**

当前 `object_position_rms` 用的是**实体级 `map_xyz`（融合均值）序列**，因此是下界估计。
要换成**原始单帧观测**，需要：

1. worker 在写每帧观测事件时，给每个 `scene_objects` 项附上它自己的 `map_xyz`
   （现在只有 `camera_xyz`；`map_xyz` 只存在于 `localized`/实体上）；
2. `explorer_search_adapter.py:293` 的 `OBJECTS_UPDATED` payload 直接透传
   `payload.get("scene_objects")`，只要上游补了字段就会自动带上。

**未实施**：这属于口径改进，不影响 P2 的结论；在剩余项都依赖用户输入的情况下，
先记录为明确的改进项，避免在验证收尾阶段引入新变量。

**P3 duplicate/false_split/false_merge（2026-09-09 11:58，带假设口径）**

用 label 聚合（`dataset_d_labels.json`，9 个 label）并传入**假设的期望实例数**（每 label 1 个）
跑 `map_quality_evaluator`：

```text
duplicate_rate = 0.0
false_split    = 0
false_merge    = 0
per_label: 9 个 label 全部 expected 1 / predicted 1
```

**口径必须标注**：`--expected-instances` 是**假设**（每个标签在场景里只有 1 个实例），
**不是人工标注的真值**。因此这三个数只说明**"输入与假设自洽"**，
**不能**当作"去重算法无错"的证据。

**要真正验证**，需要用户提供每个标签的真实实例数（例如"现场有 2 把椅子"），
再重跑同一命令。**在此之前这三项仍为 NOT VERIFIED。**

**✅ 有闭环数据集采集成功（2026-09-09 11:51）**

用「十二边形两圈」策略（每边 `f0.15`、每步 `r30`，共 24 步）采集
`datasetDloop3_20260909_114801`（2.6 GB）：

```text
loop_closure_count = 44        <- 非零
loop_edge_markers  = 6
keyframes          = 10        <- 非零
source = rosbag:/go2w/slam/pose_graph_loop_edges (visualization_msgs/Marker, points//2)
```

**因此 objective 第 (3) 项「有闭环数据集」已达成** ——
`loop_closure_count` 与 `keyframes` 从 **0** 变成 **44 / 10**，
证实「关键帧驱动的发布机制」在真正闭环时会触发。

**同时注意**：

- 第二圈在第 17 步中止（`arm failed: rcl node's context is invalid`），实际约 1.5 圈
  —— **仍足够触发回环**；
- 该数据集的 `overlap_rmse_m = 1.4011`（inlier 0.042）**不适用**：`overlap` 指标假设
  首末帧看到同一区域，而十二边形轨迹**不回到起点**（首末位置差约 0.4 m 且朝向不同），
  因此这不是 SLAM 误差，而是轨迹设计使然。**回原点类数据集的 `overlap` 结论仍以
  Dataset A/B 为准**（A 0.0654–0.0773、B 0.0433）。

**仍欠**：`duplicate_rate / false_split / false_merge` 需人工真值（`--expected-instances`）。

**P3 Dataset D 量化结果（2026-09-09 11:55，真机）**

导出：从多个 `MAP_UPDATED` 事件按 **`label#object_id`**（唯一实例）抽取 `map_xyz` 序列，
只保留观测 ≥2 次的实例：

```text
椅子#obj_001 n7 | 办公桌#obj_002 n7 | 纸箱#obj_003 n7 | 机器人#obj_004 n7
显示器#obj_005 n7 | 办公椅#obj_011 n3 | 文件柜#obj_012 n3 | 机器人#obj_013 n3
柜子#obj_017 n2 | 地板#obj_018 n2
```

`map_quality_evaluator` 输出：

| 指标 | 值 | 阈值 | 判定 |
|---|---|---|---|
| `object_position_rms_m` | **0.1526** | ≤0.15 | **略超（边界）** |
| `object_position_p95_m` | **0.1609** | ≤0.25 | **PASS** |

逐实例 RMS：

```text
办公桌#obj_002 0.0008 | 机器人#obj_013 0.0030 | 文件柜#obj_012 0.0068
椅子#obj_001   0.0091 | 机器人#obj_004 0.0097 | 办公椅#obj_011 0.0098
纸箱#obj_003   0.0143 | 显示器#obj_005 0.1018 | 柜子#obj_017 0.1526
地板#obj_018   0.0000
```

**结论**：**10 个实例中 9 个 RMS ≤ 0.102**（多数 ≤0.015）；
**唯一超标的是 `柜子#obj_017`（0.1526，n=2）**，它把整体 RMS 拉到 0.1526。

**口径限制（必须标注）**：输入是**实体级 `map_xyz`（融合均值）序列**，
不是原始单帧观测（`OBJECTS_UPDATED` 不含 `map_xyz`）→ 该 RMS 是**下界估计**。

**仍欠**：`duplicate_rate / false_split / false_merge` 需要 `--expected-instances`
（人工标注真值），**尚未采集** → 这三项仍 NOT VERIFIED。

**P3 多视角观测已累积（2026-09-09 11:50）**

第二次往返后，`METRIC_LIDAR` 实体的 `map_observation_count` 达到 **2–3**：

```text
obj_005 显示器  obs 2 / map_obs 2
obj_011 办公椅  obs 1→3 / map_obs 1→3
obj_012 文件柜  obs 1→3 / map_obs 1→3
obj_013 机器人  obs 1→2 / map_obs 1→2
obj_017 柜子    obs 1→2 / map_obs 1→2
obj_018 地板    obs 1 / map_obs 1
```

**因此 map 系的多帧观测已经具备**，`object_position_rms/p95` 的输入条件成立。

**但输入口径必须诚实标注**：`OBJECTS_UPDATED` 的观测项**不含 `map_xyz`**
（只有 `camera_xyz`），因此可用的 map 系坐标是**实体级的 `map_xyz` / `position_mean_xyz`**
（融合后的均值），**不是原始单帧观测**。用它算出的 `object_position_rms` 是**下界估计**
（均值序列比原始观测更集中）。

**下一步**：从多个 `MAP_UPDATED` 事件按 `object_id` 抽取 `map_xyz` 序列 → 生成 `--objects` JSON
→ 跑 `map_quality_evaluator`，并在报告中标注该口径限制。

**P3 Dataset D：真机数据现状（2026-09-09 11:45）**

在会话 `search_20260909_114046_45a68c0b` 上做了一次小幅往返（`f0.15,b0.15`，回到原点）后，
实体观测分布如下：

```text
obj_009  机器人  CAMERA_LOCAL  obs 1→4      ← 多次观测
obj_010  显示器  CAMERA_LOCAL  obs 1→4      ← 多次观测
obj_011  办公椅  METRIC_LIDAR  obs 1
obj_012  文件柜  METRIC_LIDAR  obs 1
obj_013  机器人  METRIC_LIDAR  obs 1
obj_014  办公椅  CAMERA_LOCAL  obs 1        ← 与 obj_011 同标签、不同实体
```

**关键观察**：

1. **多次观测（obs 4）只出现在 CAMERA_LOCAL 实体上**（`obj_009/010`）；
2. **METRIC_LIDAR 实体一律 `obs=1`**，且同标签会分裂成不同实体（`obj_011` vs `obj_014`）；
3. 实体字段里 `position_variance_xyz = [0,0,0]`、`map_observation_count = 1` ——
   说明 **map 系只有单次观测**，不满足 `object_position_rms/p95` 的输入要求
   （该指标需要同一实例的多帧观测）。

**结论**：**Dataset D 的真机数据仍未满足评估输入** ——
需要一段**专门的多次小幅运动采集**（每次移动后等一个完整 cycle 让 map 系观测累积、
且关联成功），才能得到 `object_position_rms/p95` 与 `duplicate_rate/false_split/false_merge`。
**评估链路本身已用合成数据验证可用**（见下节），因此缺的只是数据。

**✅ P2 修复生效（2026-09-09 11:41，会话级证据）**

修复：`run_semantic_exploration.py` 在 `explorer_kwargs` 里注入
`semantic_graph=SemanticNavigationGraph(object_map=state["spatial_v2"]["semantic_map"])`，
让 `AutonomousExplorer` 与 worker 共用同一张对象表。

会话 `search_20260909_114046_45a68c0b` 实测（MAP_UPDATED 物体节点）：

```text
纸箱   | METRIC_LIDAR | [0.8146, -2.5039, 0.5597]
机器人 | METRIC_LIDAR | [0.4147, -2.3009, 0.4028]
显示器 | METRIC_LIDAR | [2.9259, -3.7155, 1.4021]
椅子   | METRIC_LIDAR | [1.5586, -1.7029, 0.4351]
办公桌 | METRIC_LIDAR | [1.7409, -2.1518, 0.328]
```

节点 provenance（**P2 的目标证据**）：

```json
"transform_source": "nominal_extrinsic",
"pose_source": "plain_slam_pandarxt16_odom",
"map_revision": 14,
"transform_quality": "METRIC_LIDAR",
"tf_available": true,
"map_frame": "pslam_map",
"place_id": "P1"
```

**P2 的三项要求全部达成**：

1. `spatial_quality = METRIC_LIDAR`（不再是 CAMERA_LOCAL）；
2. `map_frame = pslam_map`（落在 SLAM map 系）；
3. `tf_available = true`（map→odom TF 已就位）。

**仍待**：`semantic_object_localized` 事件本身未出现（`assoc` 仍为 None），
但物体节点**已经带上 `map_xyz` + `map_frame` + `METRIC_LIDAR`**，信息可从 `MAP_UPDATED` 取得。

**精确落点（2026-09-09 11:57）**：`run_semantic_exploration.py:1208-1214` 把
`semantic_map` 存进 `state["spatial_v2"]["semantic_map"]`，因此修复只需在 2817 之后加：

```python
if state.get("spatial_v2", {}).get("semantic_map") is not None:
    explorer_kwargs["semantic_graph"] = SemanticNavigationGraph(
        object_map=state["spatial_v2"]["semantic_map"]
    )
```

（并 import `SemanticNavigationGraph`）。**这是 P2 的最后一处改动。**

**修复落点已定位（2026-09-09 11:55）**

- `run_semantic_exploration.py:2818` 的 `AutonomousExplorer(...)` **没有传 `semantic_graph`**
  → 走默认 `SemanticNavigationGraph()`（自建 object_map）；
- `explorer_kwargs`（2730 初始化、2836 展开）是现成的注入点；
- 但 `semantic_map` 是 `1088–1209` 那个函数里的局部变量（通过 1209 的 dict 返回），
  **main 作用域里没有它** —— 所以修复需要**先把 `semantic_map` 从那个 dict 取出来**，
  再 `explorer_kwargs["semantic_graph"] = SemanticNavigationGraph(object_map=semantic_map)`。

**这是 P2 的最后一处改动**（约 3 行）。在此之前，会话级 `map_frame` 证据仍为 NOT VERIFIED。

**决定性发现：两套 `SemanticObjectMap` 实例（2026-09-09 11:52）**

- `autonomous_explorer.py:319`：`self.semantic_graph = semantic_graph or SemanticNavigationGraph()`
  —— **默认构造**，于是 `SemanticNavigationGraph` 内部创建了**自己的**
  `SemanticObjectMap`（`object_map or SemanticObjectMap()`）；
- 而 worker 在 `run_semantic_exploration.py:1119` 另建了 `semantic_map = SemanticObjectMap(...)`，
  并在 1847 用**它**接收那批 `METRIC_LIDAR` 观测。

**两套对象表**：worker 的那套拿到了 `METRIC_LIDAR` 观测；
`MAP_UPDATED` 展示的是 `explorer.semantic_graph` 那一套 —— 它只被
`_object_observations(scene_objects, ...)` 喂过（quality 默认 `CAMERA_LOCAL`），
**从未收到 provider 的 `METRIC_LIDAR` 观测**。

这解释了三件事：

1. `localized` dump 全是 `METRIC_LIDAR`；
2. `MAP_UPDATED` 的实体全是 `CAMERA_LOCAL`、`obs=1`；
3. `semantic_object_localized` 事件缺失（worker 那套的 `update_result` 关联表
   与 `association_by_index` 未覆盖这些物体）。

**修复方向**：让 `AutonomousExplorer` 复用 worker 的 `semantic_map`
（例如 `AutonomousExplorer(semantic_graph=SemanticNavigationGraph(object_map=semantic_map))`），
或在 worker 里把 `localized` 也喂给 `explorer.semantic_graph`。**这是 P2 的最后一处。**

**`explorer.graph` 的类型确认（2026-09-09 11:48）**

`autonomous_explorer.py:318`：`self.graph = graph or ExplorationGraph(...)`，
而 `ExplorationGraph` 定义在 `app/navigation/exploration_graph.py:131`。
worker 在 `run_semantic_exploration.py:490/589/2715` 把 `explorer.graph.to_dict()` 挂到事件上，
`explorer_search_adapter._map_events` 再把它包成 `MAP_UPDATED` 的 `graph`。

`ExplorationGraph.to_dict()`（398）输出的是**探索图节点**（`node.to_dict()`，节点里有
`objects=[标签]`），**不是 `SemanticObjectMap` 的语义实体**。
而 `SemanticNavigationGraph` 在整个生产代码里**没有任何实例化**（只在测试里出现）。

**这意味着**：`MAP_UPDATED` 里那些带 `spatial_quality=CAMERA_LOCAL` 的 OBJECT 节点，
其 quality 来自**探索图节点**的字段，与 `SemanticObjectMap` 的实体是**两套数据**。

**下一步**：确认探索图节点的 `spatial_quality` 在哪里被赋值
（`app/navigation/exploration_graph.py` 的节点构造），
并判断它是否应该跟随语义实体升级。

**`update_with_associations` 内部流程核实（2026-09-09 11:45）**

读源码确认它的两条分支都**不会**把 quality 降级：

- 关联成功 → `_merge`（236）→
  `if _quality_rank(obs.spatial_quality) > _quality_rank(entry.spatial_quality): entry.spatial_quality = obs.spatial_quality`
- 关联失败 → 创建（255–275）→ `spatial_quality=obs.spatial_quality`

而 `obs` 就是传入的 `spatial_objects` 成员（194 行 `for index, obs in enumerate(spatial_objects)`），
即我们 dump 到的 `METRIC_LIDAR` 那批。**因此 `run_semantic_exploration.py:1847` 这条路径
不可能产出 CAMERA_LOCAL 实体。**

**所以 `MAP_UPDATED` 的 OBJECT 节点必然来自另一处** ——
`app/live_robot/explorer_search_adapter.py:635`：
`self._emit(MAP_UPDATED, payload={"graph": normalized_graph, ...})`，
其 `graph` 是**外部传入**的。

**补充**：worker 的事件列表里**没有 `map_updated`**（只有 `place_updated` 等），
说明 `MAP_UPDATED` 是 WebUI 侧由别的输入构造的，不是 worker 直接写的。

**下一步**：追 `normalized_graph` 的来源（`explorer_search_adapter` 的上游调用），
确认它读的是哪份对象表。

**两条 `update_with_associations` 路径（2026-09-09 11:38）**

grep 显示它被**两处**调用：

- `run_semantic_exploration.py:1847`（我加 dump 的地方，传入被改写的 `localized`）
- `semantic_navigation_graph.py:85`：`objects = spatial_objects or self._object_observations(...)`

而 `MAP_UPDATED` 的 OBJECT 节点正是由 `semantic_navigation_graph` 产出。
**若 `spatial_objects` 为空/None，它会从 `scene_objects` 重建观测**
（那条路径的 quality 是 `CAMERA_LOCAL`）—— 与实体实测的 CAMERA_LOCAL 一致。

**另外**：`semantic_object_localized` 事件仍未出现（`map_frame` 计数 0），
说明 1905 处 `assoc = association_by_index.get(index)` 为 None，
即 `update_result` 的关联表没覆盖这些物体。

**下一步**：在 `semantic_navigation_graph.py:81-89` 加一行日志打印
`len(spatial_objects or [])` 与 `objects[0].spatial_quality`，
确认它收到的到底是不是那批 `METRIC_LIDAR` 观测。

**`localized` 全部 METRIC_LIDAR，但实体仍 CAMERA_LOCAL（2026-09-09 11:35）**

诊断 dump（在 `update_with_associations` 调用前）：

```text
localized dump: [('椅子','METRIC_LIDAR',id,(1.577,-1.719,0.414)),
                 ('纸箱','METRIC_LIDAR',id,(0.814,-2.5,0.559)),
                 ('机器人','METRIC_LIDAR',id,(0.458,-2.293,0.403)),
                 ('办公桌','METRIC_LIDAR',id,(1.744,-2.155,0.328)),
                 ('显示器','METRIC_LIDAR',id,(2.754,-3.501,1.321))]
```

**五个物体全部是 `METRIC_LIDAR`** —— 输入侧完全正确。

但同一会话 `search_20260909_113410_497b667a` 的实体仍是 `CAMERA_LOCAL`，
且坐标完全不同（椅子 `[0.1457, 3.6969, -0.1142]` vs `(1.577, -1.719, 0.414)`），
`observation_count` 都是 1。

**因此问题锁定在 `semantic_map.update_with_associations` 内部**：要么它拿到的不是这批
`localized`（内部另有快照/缓存），要么实体的 `to_dict()` 输出的 `spatial_quality`
不是 `entry.spatial_quality`。

**下一步**：在 `semantic_object_map.py` 的创建分支（255–275）与升级分支（508–509）
各加一行日志，打印 `obs.spatial_quality` 与 `entry.spatial_quality`，即可一锤定音。

**同一批观测、两个不同 map_xyz（2026-09-09 11:42）**

`MAP_UPDATED` 时间戳 `1788924673` / `1788924698` 与 `object quality` 日志的 `1788924673`
**完全对齐**（同一会话 `search_20260909_113034_a8484b41`、同一 cycle 1），但：

- 日志：`办公椅 map_xyz=(1.564, -1.685, 0.414)`、`provider_quality=METRIC_LIDAR`
- 事件：`办公椅 | CAMERA_LOCAL | obs 1 | [0.187, 3.6745, -0.1139]`

**同一时刻、同一标签、两个不同坐标与两个不同 quality** —— 说明 `localized` 里存在
**多个同名物体**（日志只打印了循环中的一部分），而写进实体的那条是**另一条**
（quality 仍是 CAMERA_LOCAL）。

**下一步**：在 1803 的日志里加上 `id(obs_item)` 与索引，
确认"打印的那条"是否就是"写进实体的那条"。

**实体创建路径确认（2026-09-09 11:40）**

`semantic_object_map.py:255-275` 创建实体时用的正是 `spatial_quality=obs.spatial_quality`，
而 `obs` 就是 `localized` 里那个被 1803 改写过的对象
（`update_with_associations(spatial_objects=localized)`）。按理新实体应直接是 `METRIC_LIDAR`，
但实测仍是 `CAMERA_LOCAL`。

**因此怀疑点收窄为**：`localized` 在 1803 与 1829 之间是否被替换/复制
（排序、过滤或 dataclass 重建），或 `obs_item.spatial_quality` 的赋值被静默忽略
（若 `ObjectSpatialObservation` 是 frozen dataclass，赋值会抛 `FrozenInstanceError`；
但日志显示 4 个物体都打印了、循环未中断，此路径可能性较低）。

**下一步**：在 1803 之后、1829 之前打印 `id(obs_item)` 与 `localized[0].spatial_quality`，
并在 `semantic_object_map.py` 创建分支打印 `obs.spatial_quality`，两端对齐即可定位。

**OBJECT 节点来源确认（2026-09-09 11:38）**

`MAP_UPDATED` 的 OBJECT 节点由 `semantic_navigation_graph.py:163-170` 构造：

```python
for obj in objects:
    nodes.append({"node_id": obj["object_id"], "node_type": "OBJECT",
                  "label": obj.get("label", "object"), "pose": _object_pose(obj), **obj})
```

即节点的 `spatial_quality` **直接来自传入的 `objects`**（`semantic_map` 实体的 `to_dict()`）。

**已确认的事实链**：

1. 物体侧日志：`provider_quality=METRIC_LIDAR` ✓
2. 机器狗运行时实测：`_quality_rank("METRIC_LIDAR")=4 > _quality_rank("CAMERA_LOCAL")=1` ✓
3. **但实体仍是 CAMERA_LOCAL**

**因此剩下的唯一可能**：`semantic_map.update_with_associations(localized, ...)` 内部
**没有使用 `localized` 里那个被改写过的 `obs_item.spatial_quality`**
（例如重新构造了 `ObjectSpatialObservation`，或用了 `frame_object` 的原始 quality）。

**下一步**：直接读 `update_with_associations` 的内部实现（`semantic_object_map.py` 172–290），
确认它取的是哪个字段。**这是最后一段。**

**物体侧 quality 已正确，但实体仍未升级（2026-09-09 11:35）**

诊断日志证明**物体侧完全正确**：

```text
object quality: label=办公椅 provider_quality=METRIC_LIDAR map_xyz=(1.564, -1.685, 0.414)
object quality: label=文件柜 provider_quality=METRIC_LIDAR map_xyz=(1.756, -2.247, 0.35)
object quality: label=纸箱   provider_quality=METRIC_LIDAR map_xyz=(0.782, -2.519, 0.586)
object quality: label=机器人 provider_quality=METRIC_LIDAR map_xyz=(0.407, -2.317, 0.435)
```

但同一会话 `search_20260909_113034_a8484b41` 的 `MAP_UPDATED` 物体仍是 `CAMERA_LOCAL`，
且 `map_xyz` **不同**（办公椅 `[0.187, 3.6745, -0.1139]` vs 日志 `(1.564, -1.685, 0.414)`）。
机器狗上 `semantic_object_map.py` 确认含 `METRIC_LIDAR`（2 处），`__pycache__` 有对应缓存。

**剩余疑点**：

1. `_quality_rank` 修复是否真的被 worker 进程加载（需在机器狗上**直接调用**验证运行时值）；
2. `MAP_UPDATED` 的 OBJECT 节点与 `object quality` 日志里的物体**不是同一批**
   （`map_xyz` 差一个量级），需确认节点来源（`place_graph` 的 OBJECT 节点 vs `semantic_map` 实体）。

**下一步**：在机器狗上直接调用 `_quality_rank("METRIC_LIDAR")` 验证运行时值；
并确认 `MAP_UPDATED` 的 OBJECT 节点来源。

**诊断日志已加（2026-09-09 11:31）**：在 1803 处打印
`label / provider_quality / map_xyz`，已编译并部署到机器狗。
新会话 `search_20260909_113034_a8484b41` 已启动，**待日志出现即可定位最后一段**。

**`_quality_rank` 修复后仍未升级（2026-09-09 11:30）—— 数据流继续追**

新会话 `search_20260909_112812_1484f66f`（机器狗代码已含 `METRIC_LIDAR:4`）的物体仍为 CAMERA_LOCAL：

```text
办公椅 | CAMERA_LOCAL | [0.1884, 3.6756, -0.1183]
文件柜 | CAMERA_LOCAL | [-0.3769, 3.9182, -0.0433]
纸箱   | CAMERA_LOCAL | [-0.714, 2.9565, -0.2729]
机器人 | CAMERA_LOCAL | [-0.5328, 2.5933, -0.093]
```

**数据流已核到**：`semantic_map.update_with_associations(localized, ...)`（1829）拿到的正是
被 1803 改写过的 `localized` 列表，而 `update_with_associations` 内部就是 508–509 的
「只升不降」逻辑 —— 所以 `_quality_rank` 修复是**必要条件，但不是全部**。

**剩余疑点**：`localized` 里每个 `obs_item` 的 `spatial_quality` 是否真被 1803 覆盖。
1803 位于 `if mapped is not None:` 内，而该循环又包在
`if camera_provider is not None and spatial_pose is not None:` 里；
诊断日志显示 `get_pose=yes`，但**该日志与物体处理不一定处于同一次循环迭代**。

**下一步**：在 1803 处加一行日志（打印 `obs_item.label` 与 `provider_quality`），
确认物体侧看到的 quality 到底是什么。**这一步做完就能定位最后一段。**

**P2 根因终于找到：`_quality_rank` 缺 `METRIC_LIDAR`（2026-09-09 11:28）**

`app/spatial/semantic_object_map.py` 的等级表只有
`RGB_ONLY:0 / CAMERA_LOCAL:1 / RELATIVE_RGBD:2 / METRIC_RGBD:3`，
**没有 `METRIC_LIDAR`** → `order.get("METRIC_LIDAR", 0)` 返回 **0**，
反而**低于** `CAMERA_LOCAL(1)`。

而实体更新的规则是「**只升不降**」：

```python
if _quality_rank(obs.spatial_quality) > _quality_rank(entry.spatial_quality):
    entry.spatial_quality = obs.spatial_quality
```

于是即使观测已经是 `METRIC_LIDAR`，实体也**永远无法从 CAMERA_LOCAL 升级** —— 这解释了
为什么前四步修复后 `map_xyz` 有了、`spatial probe: quality=METRIC_LIDAR` 也打印了，
**语义物体的 `spatial_quality` 却始终是 CAMERA_LOCAL**。

**修复**：在等级表里加 `"METRIC_LIDAR": 4`。实测
`_quality_rank('METRIC_LIDAR')=4 > _quality_rank('CAMERA_LOCAL')=1` ✓，
编译 + 14 个测试通过 + 部署到机器狗。

新会话 `search_20260909_112812_1484f66f` 已启动，**待一个完整 cycle 后确认物体质量升级**。

**P2 第五步修复后仍未升级（2026-09-09 11:25）—— 收敛到 `_pose`**

新会话 `search_20260909_112242_617c63c3`（含全部五步修复）的 MAP_UPDATED 物体：

```text
椅子   | CAMERA_LOCAL | [0.1465, 3.6989, -0.1142]
办公桌 | CAMERA_LOCAL | [-0.3674, 3.9143, -0.0641]
纸箱   | CAMERA_LOCAL | [-0.6941, 3.0024, -0.2592]
机器人 | CAMERA_LOCAL | [-0.5172, 2.6309, -0.1302]
显示器 | CAMERA_LOCAL | [-1.6396, 5.1213, -0.998]
```

**`map_xyz` 全部有效（40 处），但 `spatial_quality` 仍是 `CAMERA_LOCAL`。**

**因此 `quality()` 返回 CAMERA_LOCAL 的唯一可能**是它的第二条件不成立 —— 即
`self._pose is None`（`get_pose()` 也会因此落到 fallback）。而**独立探针**在同一环境下
`get_pose=true`、`quality=METRIC_LIDAR`，说明**话题与 QoS 都没问题**，
问题在 **worker 进程内那个 provider 实例的 `_pose` 从未被赋值**。

**下一步（明确、一次性）**：在 worker 里加诊断日志，打印 `_pose is None`、
`get_pose()`、`quality()` 与订阅数，确认 `_on_odometry` 是否被调用过。

**仍为 NOT VERIFIED**：会话级 `semantic_object_localized.map_frame`。

**P2 第五步修复（2026-09-09 11:23）**

第四步 QoS 改成 `RELIABLE + TRANSIENT_LOCAL` 后，物体仍带 `map_xyz` 但
`spatial_quality=CAMERA_LOCAL`（新会话实测 7 个物体：机器人 / 办公桌 / 显示器 / 椅子 / 纸箱 等，
`map_xyz` 均有效，例如 `[-0.5194, 2.6255, -0.1035]`、`[0.1434, 3.695, -0.1142]`）。

据此定位到 `quality()` 的**第二个条件**：它要求 **`self._pose is not None`**
（即 `/go2w/slam/odom_base` 已到达），只等 `get_map()` 不够。已把 spin 循环的退出条件改为
**「`quality()` 不再等于 `CAMERA_LOCAL`」**（上限 2 s = 40×50 ms），编译 + 部署 + grep 验证通过。

新会话 `search_20260909_112242_617c63c3` 已启动，**待一个完整 cycle 后判定**。

**P2 第四步修复已部署，会话仍在 `OBSERVE`/cycle 0（2026-09-09 11:22）**

把 `map_2d` 的订阅 QoS 从 `BEST_EFFORT + VOLATILE` 改为 **`RELIABLE + TRANSIENT_LOCAL`**
（与 publisher 同级，订阅建立即取到最后一份地图，不再怕错过发布时刻）；
`odom_base` / `/tf` 保持 `BEST_EFFORT`（它们的 publisher 就是 BEST_EFFORT）。
编译通过、provider 的 7 个测试通过、已部署到机器狗。

重启会话 `search_20260909_111959_9392ae17` 后：`status=RUNNING, phase=OBSERVE, cycle=0`，
**尚未产生新的 LLM 调用**（日志里最后一条 `LLM scene` 还是上一会话 11:17:52 的），
因此还没有 `OBJECTS_UPDATED` / `MAP_UPDATED` 可供判定。**需继续观察一个完整 cycle。**

**P2 复测：`map_xyz` 已写入但 `spatial_quality` 仍降级（2026-09-09 11:20）**

新代码已生效（日志含 `spatial provider ready: map=yes, map_point=yes, quality=METRIC_LIDAR`），
且会话事件里**已经出现带 `map_xyz` 的物体节点**：

```json
{"node_id":"obj_001","node_type":"OBJECT","label":"办公椅",
 "pose":{"x":0.1907,"y":3.6787,"z":-0.114},
 "depth_m":1.144,
 "camera_xyz":[-0.6233,-0.114,1.144],
 "map_xyz":[0.1907,3.6787,-0.114],      <- 已写入
 "spatial_quality":"CAMERA_LOCAL",       <- 仍降级
 "observation_count":1}
```

**代码级定位**：`run_semantic_exploration.py` 里 `provider_quality` 有四处赋值 ——
`1673` 初始 `CAMERA_LOCAL`、`1689` 取 `camera_provider.quality()`（我加 spin 的位置）、
`1728` 降级 `DEGRADED_LIO`、`1788` 写进 `obs_item.spatial_quality`。
实测既不是 `METRIC_LIDAR` 也不是 `DEGRADED_LIO`，**说明 1689 处的 `quality()` 仍返回 CAMERA_LOCAL**。

而 `quality()` 要求 `_map_is_fresh()`（3 秒内），`_map_ts` 只在 `_on_occupancy_grid` 更新。
我的「spin 直到 `get_map()` 非空、上限 1.5 s」在**就绪阶段有效**，但在**观测阶段**仍未拿到新鲜 map ——
**说明 worker 主循环阻塞期间 BEST_EFFORT 订阅的消息被丢弃**，1.5 秒窗口内没有新 map 到达。

**下一步（明确）**：把 `map_2d` 的订阅 QoS 从 BEST_EFFORT 改成 **RELIABLE**
（publisher 是 `RELIABLE + TRANSIENT_LOCAL`，改 RELIABLE 后能取到最后的样本，
不再依赖"恰好赶上发布时刻"）；或在 spin 循环里加一行日志确认 `get_map()` 是否成功。

**仍为 NOT VERIFIED**：会话级 `semantic_object_localized.map_frame`（该事件本身也还没出现）。

**机器狗恢复 + 外网恢复后的 P2 复测（2026-09-09 11:16）**

**环境变化**：

- 机器狗失联原因查明：**网线松动**（控制主机 `enp3s0` 显示 `NO-CARRIER`）；用户插紧后恢复
  （`UP + LOWER_UP`、ping 0% loss）；
- **外网恢复**：`https://api.siliconflow.cn/v1/models` → **401**；
- **LLM 恢复工作**：worker 日志 `LLM scene: 画面左侧可见黑色椅背和黄色椅座，符合办公椅特征
  (matched=1/all=1)`。

**P2 修复第二步（加强）已部署**：`spatial provider ready: map=yes, map_point=yes,
quality=METRIC_LIDAR` 能正确打印，但**观测时** `provider.quality()` 仍降级 —— 因为 `map_2d`
每 0.5 s 才发布一次，固定 6×50 ms 的 spin 窗口会错过。已改为「**spin 直到 `get_map()` 非空**」
（上限 1.5 s），编译 + 部署 + grep 验证通过。

**当前会话状态**（`search_20260909_111607_8a0ba37d`）：`status=RUNNING, phase=MATCH, cycle=1`，
事件里目前只有 `SEARCH_STATE_CHANGED / SESSION_*`，**尚未出现 `OBJECTS_UPDATED`**，
因此 `map_xyz` 与 `map_frame` 仍未取到。**需要让会话继续跑到写观测那一步再判定。**

**仍为 NOT VERIFIED**：会话级 `semantic_object_localized.map_frame`。

**机器狗再次断线 + 外网恢复（2026-09-09 11:12）**

- 控制主机 `enp3s0`（到机器狗）**DOWN / NO-CARRIER**；`ping 192.168.123.18` **100% 丢包**、
  ARP `INCOMPLETE`；断线前最后一次运动因 `SportModeState or LowState became stale` 中止。
- **外网已恢复**：`https://api.siliconflow.cn/v1/models` → **401**（前三轮的 `http=000` 消失）。
- 因此 **P2 会话级取证 / P3 Dataset D 的唯一阻塞已从"外网"变成"机器狗连通性"**。
- 交接书已补「机器狗恢复后第一件事」四步清单，含 `/tmp/stageo_chair.json` 的重建内容。
- 运动限幅记录：`r90` 被截到 **30°**；`f0.1` 因 `FORWARD_NOT_CONFIRMED`
  （实测 0.045 m < 0.05 m 阈值）失败 → **最小前进步用 `f0.15`**。

**Dataset D 指标路径的合成自测（2026-09-09 11:35，不需要机器狗/外网）**

外网不通阻塞了 Dataset D 的**采集**，但评估链路可以先验证。用合成观测跑
`map_quality_evaluator.py`（只给 `--objects` 与 `--expected-instances`）：

```text
输入: {"chair": [[1.00,0.00,0.30],[1.02,0.01,0.31],[0.99,-0.01,0.29]],
       "desk":  [[2.00,1.00,0.40],[2.05,1.02,0.41]]}
      --expected-instances {"chair":1,"desk":1}
```

**输出**：

```text
semantic.object_position_rms_m = 0.0274   (limit 0.15)  PASS
semantic.object_position_p95_m = 0.0274   (limit 0.25)  PASS
semantic.objects.chair.centroid = [1.0033, 0.0, 0.3]    (3 次观测, rms 0.0170)
semantic.objects.desk.centroid  = [2.025, 1.01, 0.405]  (2 次观测, rms 0.0274)
dedup.duplicate_rate = 0.0
dedup.false_split = 0
dedup.false_merge = 0
dedup.per_label = {chair: {expected 1, predicted 1}, desk: {expected 1, predicted 1}}
acceptance.verified = ["object_position_rms_m", "object_position_p95_m"]
```

**结论**：**Dataset D 的全部五项指标（`object_position_rms/p95`、`duplicate_rate`、
`false_split`、`false_merge`）路径已用合成数据验证可用**；一旦外网恢复、语义会话能跑通，
采集完 `--objects` 即可直接出报告。**当前仍为 NOT VERIFIED 的是"真机 Dataset D 数据本身"**。

**`pose_graph` 不发布的完整机制（2026-09-09 11:25，已闭环）**

继续追到 `slam_3d_interface.cpp`：

```cpp
ComputeLocalMapMatchingRate(filtered_cloud);
if (local_map_matching_rate_ < min_local_map_matching_rate_) {
    kf_detector_.AddKeyframe(slam_pose_);
} else if (!kf_detector_.IsKeyframe(slam_pose_)) {
    return;                      // <- 非关键帧直接返回，不置位
}
...
pose_graph_.emplace_back(slam_pose_);
kf_detector_.UpdateKeyframe(slam_pose_);
is_pose_graph_updated_ = true;   // <- 只有走到这里才置位
```

而 `slam_3d_node.cpp` 的 `map_3d` 与 `pose_graph_*` **在同一个 `if (IsPoseGraphUpdated())` 块里发布**。

**实测佐证**：`/go2w/slam/deskewed_scan` 的时间戳是**当前**（输入在流动），
而 `/go2w/slam/map_3d` 的时间戳停在 **40 分钟前** —— 即 `SyncCallback` 一直在跑，
但 `IsPoseGraphUpdated()` 长时间为 false。

**结论（完整）**：

- 这是**关键帧驱动的设计**，不是缺陷：机器狗静止/小幅运动时不产生新关键帧，
  位姿图不更新，`map_3d` 与 `pose_graph_*` 都不发布；
- 因此 `loop_closure_count = 0` / `keyframes = 0` 的正确读法是
  「**这段数据期间没有新增关键帧**」，**不是**「确认无回环」；
- 想得到非零的回环/关键帧数据，需要一段**持续移动并闭环**的轨迹。

**`is_pose_graph_updated_` 触发条件核对（2026-09-09 11:15）**

追到上游后确认：该标志**并非永远 false** —— `slam_3d_interface.cpp` 在
**建图分支（第 145 行）**与**关键帧分支（第 166 行）**都会置 `true`，
而 `slam_3d_node.cpp:165` 的发布正是由它门控、发布后置回 false。

但**实测 `/go2w/slam/pose_graph_poses` 仍然没有数据**（`ros2 topic echo` 20 秒无输出；
Dataset B 的 bag 里 0 条）。因此当前能确定的只有：

> 在这两次运行中，`slam_3d_node` **没有走到发布分支**（或该分支被跳过）。

**仍未确定**：node 侧调用 `IsPoseGraphUpdated()` 的代码所在回调是否被触发
（是否依赖建图模式/关键帧检测的特定条件）。需要继续在 `slam_3d_node.cpp` 里
向上追调用链，或给该标志加日志。

**结论不变**：`loop_closure_count = 0` 目前**不能**读作"确认无回环"。

**pose_graph 话题无数据的上游原因（2026-09-09 11:10）—— 修正上一节的表述**

`loop_closure_count = 0` 与 `keyframes = 0` 的**直接原因**是
`/go2w/slam/pose_graph_poses` 与 `/go2w/slam/pose_graph_loop_edges` 在 bag 里**都没有消息**。
追到上游：`slam_3d_node.cpp:165` 的发布被 `if (slam_.IsPoseGraphUpdated())` 包住 ——
即**只有位姿图更新时**才发布 graph_nodes / odom_edges / loop_edges / graph_poses 四条话题。

**因此 `loop_closure_count = 0` 的准确含义是**：

> 本次数据集（原点往返 ≤0.5 m）期间，`slam_3d_node` **没有报告位姿图更新**，
> 因而既没有回环边、也没有关键帧位姿被发布。

它**不能**被读成"确认无回环" —— 要区分这两者，需要一段**真正形成闭环**的轨迹
（让 `IsPoseGraphUpdated()` 触发），或从 `slam_3d_node` 内部日志直接取回环计数。

**仍欠**：有闭环的数据集（需机器狗走闭环）+ `IsPoseGraphUpdated()` 的触发条件核对。

**loop_closure_count 的量化来源（2026-09-09 11:0x）**

给 `map_quality_evaluator.py` 加了回环统计：

- `load_bag` 新增提取 `/go2w/slam/pose_graph_loop_edges`（`visualization_msgs/msg/Marker`，
  即 `slam_3d_node` 的 `loop_edges_pub_`，`points` 每 2 点构成一条边）；
- `build_report` 的 `loop` 段不再写 `null`，改为 `loop_closure_count = loop_edges // 2`，
  并带上 `source` 与 `note`。

**实测（Dataset B）**：

```text
loop_closure_count = 0
loop_edge_markers = 0
keyframes = 0
source = rosbag:/go2w/slam/pose_graph_loop_edges (visualization_msgs/Marker, points//2)
```

**含义**：该段轨迹（原点往返 ≤0.5 m）**没有形成闭环** —— `loop_edges` 话题在 bag 里
**0 条消息**（话题已注册但从未发布）→ `0` 是**真实结果**，不再是 NOT_VERIFIED。

**仍未覆盖**：**有回环**的数据集（需要机器狗走一个真正的闭环）。
在采到闭环数据之前，"回环后重投影"的验证仍为 NOT VERIFIED。

**P2 provider 投影链路的独立验证（2026-09-09 10:55，不依赖 LLM）**

写了独立探针 `/tmp/p2_probe.py`（只用 provider，不启动搜索会话），在机器狗上实测：

```text
map_seen_frames 1
camera_point_to_map (1.8255, 0.0028, 0.3)
quality METRIC_LIDAR
provenance {'source_frame': 'd435_color_optical_frame', 'target_frame': 'pslam_odom',
            'transform_source': 'nominal_extrinsic',
            'pose_source': 'plain_slam_pandarxt16_odom', 'map_revision': 3,
            'transform_quality': 'METRIC_LIDAR', 'tf_available': False}
```

**结论**：

1. **provider 的 `camera_point_to_map` 在机器狗上完全可用** —— 相机点 `(0,0,2.0)` 被正确投影到
   `(1.8255, 0.0028, 0.3)`，与控制主机侧 round 8 的验证一致（`(1.8193, 0.0146, 0.3)`）；
2. `target_frame = pslam_odom`（**不是 `pslam_map`**），因为 `tf_available=False` ——
   即 map→odom 的 TF 没被 provider 收到，走的是 `nominal_extrinsic` + odom 位姿的回退路径；
3. `tf_available` 恒为 False 是**代码缺陷**（`_tf_available` 从未被置位），不是 TF 缺失：
   `ros2 run tf2_ros tf2_echo pslam_map pslam_odom` 实测能拿到单位变换。

**因此 P2 的两件事必须分开记**：

- ✅ **provider 投影可用**（本次独立探针证据，不依赖 LLM）；
- ❌ **会话级 `semantic_object_localized.map_frame` 仍 NOT VERIFIED**
  （阻塞于 LLM 服务不可用；三处 worker 修复已部署待验证）。

**P2 修复推进与外部阻塞（2026-09-09 10:50）**

**第三步修复**：发现 `transform_provenance()["tf_available"]` **恒为 False**
（该标志在 `plain_slam_spatial_provider.py` 里从未被置位，只在第 81 行初始化、第 309 行读取），
因此上一版"等 TF"循环条件永远不满足。已改为**真实投影探测**：

```python
if camera_provider.get_map() is not None:
    _probe = camera_provider.camera_point_to_map((0.0, 0.0, 2.0), None)
    if _probe is not None:
        break
```

已编译、部署到机器狗并验证文件含该逻辑（`grep -c map_point = 1`）。

**但验证被外部依赖阻塞**：重启 WebUI（VLM daemon 随起）后，三次启动会话都返回

```text
llm_task_understanding_unavailable（TASK_REJECTED, stage=TASK_UNDERSTANDING）
```

即**大模型服务调用失败**（API Key/网络/配额），与本次改动无关。
`siliconflow_vlm_daemon.log` 显示此前最后一次调用是成功的（"画面中存在两把办公椅"）。

**P2 当前状态**：

| 层 | 状态 |
|---|---|
| provider | ✅ 完全正常（探针实测 `quality=METRIC_LIDAR`、`pslam_odom` 系、frontier 8） |
| worker 代码 | ✅ 三处修复已部署（等就绪 / 连续 spin 6×50 ms / 真实投影探测） |
| 会话级 `map_frame` 证据 | ❌ **仍 NOT VERIFIED**，阻塞于 LLM 服务不可用 |

**解除条件**：LLM 服务恢复后重跑一次会话即可取证。

**P2 修复第二步已部署，仍待第三处（2026-09-09 10:46）**

已改两处并部署到机器狗：

1. 创建 provider 后等就绪 → 日志实测 `spatial provider ready: map=yes, quality=METRIC_LIDAR` ✓
2. 观测处理路径的 `spin_once()` 改为连续 **6 次 × 50 ms**（BEST_EFFORT 订阅不重传，
   单次 spin 时 `_map_ts` 可能已超过 `map_stale_s=3s`）

**但 `OBJECTS_UPDATED` 的物体仍为 `CAMERA_LOCAL`、`map_xyz=None`**（4 个物体）。

**已定位的第三处断点**：`map_xyz` 的写入条件是

```python
if mapped is not None:
    obs_item.map_xyz = mapped
    obs_item.spatial_quality = provider_quality
```

而 `mapped` 的首选路径是 `camera_point_to_map(obs_item.camera_xyz, image_stamp)`，
其中 `image_stamp = observation.timestamp` —— **实测该字段为 `null`**，因此首选路径必然返回 None，
只能走回退 `camera_point_to_spatial(..., pose=spatial_pose)`。

**下一步（明确）**：查清 `observation.timestamp` 为何为 None（RGB-D 帧时间戳未透传），
以及回退路径为何也未成功；修好后再重跑会话，才能拿到带 `map_frame` 的
`semantic_object_localized` 事件。**在此之前该证据仍为 NOT VERIFIED。**

**P2 修复第一步生效但未解决（2026-09-09 10:43）**

已改 `run_semantic_exploration.py`：创建 provider 后先等就绪（`while ... get_map() is None: spin_once()`），
部署到机器狗后重启会话，日志出现：

```text
[INFO] spatial provider ready: map=yes, quality=METRIC_LIDAR
```

**但 `OBJECTS_UPDATED` 的物体仍是 `CAMERA_LOCAL`、`map_xyz=None`**（4 个物体）。

**进一步排查**：QoS 匹配已确认无误 —— publisher `pointcloud_to_occupancy` 是
`RELIABLE + TRANSIENT_LOCAL`，subscriber `go2w_plain_slam_spatial_provider` 是
`BEST_EFFORT + VOLATILE`，两个方向都兼容；且 provider 的订阅确实出现在
`ros2 topic info -v /go2w/slam/map_2d` 的订阅者列表里。

**剩余原因**：`provider.quality()` 依赖 `_map_is_fresh()`（`time.monotonic() - _map_ts <= 3.0`），
而 `_map_ts` **只在 `_on_occupancy_grid` 回调里更新**；worker 的 `spin_once()` 在观测处理时只调用一次，
若主循环间隔较长、且 BEST_EFFORT 订阅没有积压消息，`_map_ts` 就超过 3 秒 → 降级 `CAMERA_LOCAL`。

**下一步修复**：在取 `provider.quality()` 之前**连续 spin 几轮**（例如 5 次 × 50 ms，与验收探针一致），
而不是只 spin 一次。**在此之前，P2 的会话级 `map_frame` 证据仍为 NOT VERIFIED。**

**P2 根因链完整定位（2026-09-09 10:40）**

逐层排查结论：

1. **provider 本身完全正常**：`acceptance_plain_slam_provider.py` 在机器狗上实测
   `get_map=true`、`map_source=plain_slam_pandarxt16`、`map_frame_provenance.frame_id=pslam_odom`、
   `get_pose=true`、`pose_source=plain_slam_pandarxt16_odom`、**`quality=METRIC_LIDAR`**、
   `frontier_count=8`、`last_error=null`。
2. **订阅与 QoS 都对**：provider 订阅 `/go2w/slam/map_2d`（OccupancyGrid）、`/go2w/slam/odom_base`、
   `/tf`；publisher QoS 为 `BEST_EFFORT+VOLATILE`，与 provider 的 `sensor_qos` 完全匹配；
   `map_2d` 发布周期是 `publish_rate_hz=2.0`（0.5 s），远小于 `map_stale_s=3.0`。
3. **worker 的 DDS 环境与探针一致**：`ROS_DOMAIN_ID=0`、`rmw_cyclonedds_cpp`、`cyclonedds_eth0.xml`。
4. **真正的时序问题**：`run_semantic_exploration.py` 只在**观测处理**时调 `provider.spin_once()`
   并立即取 `provider.quality()`（第 1642–1652 行）。会话**首次观测**发生在 provider 收到第一批
   map/pose **之前**（DDS 发现 + 首批消息延迟），于是 `_map_is_fresh()` 与 `_pose` 都还不成立 →
   `quality()` 返回 `CAMERA_LOCAL` → `obs_item.spatial_quality = CAMERA_LOCAL`、`map_xyz` 不写。
5. **之后没有机会修正**：dry-run 会话卡在 `APPROACH_EXECUTE`（等运动完成），不再产生观测，
   quality 一直停在 `CAMERA_LOCAL`。日志证据：本次会话**没有** `map source = plain_slam` 那行
   （`metric_map` 为 None 时才打印）；而 09:59 那次会话在 57 秒后打印了
   `map source = plain_slam_pandarxt16, map revision = 14, pose source = plain_slam_pandarxt16_odom`。

**修复方案（明确、最小）**：在 `run_semantic_exploration.py` 创建 provider 之后、进入主循环之前，
加"等 provider 就绪"的循环（与探针一致）：

```python
deadline = time.time() + 20
while time.time() < deadline and camera_provider.get_map() is None:
    camera_provider.spin_once()
    time.sleep(0.1)
```

这样首次观测时 `quality()` 已是 `METRIC_LIDAR`，`map_xyz` / `map_frame` 才会写进
`semantic_object_localized` 事件。**未改动前，P2 的会话级 `map_frame` 证据仍为 NOT VERIFIED。**

**P2 语义链会话级取证（2026-09-09 09:59，真机会话）**

会话 `search_20260909_095917_c8d35666`（`dry_run_motion=true`、`spatial_provider=plain_slam`、
`spatial_v2=true`）产生 88 个事件，其中 `MAP_UPDATED` 的语义图节点**确实带了 map 坐标**：

```json
{"node_id":"obj_001","node_type":"OBJECT","label":"办公椅",
 "pose":{"x":2.1556,"y":0.7571,"z":0.0046},
 "depth_m":2.068,
 "camera_xyz":[-1.1381,0.0046,2.068],
 "map_xyz":[2.1556,0.7571,0.0046],
 "bearing_deg":-28.826,
 "spatial_quality":"CAMERA_LOCAL",
 "confidence":0.95}
```

**这修正了之前"深度失败"的判断**：`depth_m = 2.068` 说明**深度有效**，
物体也**确实被投影到了 map 坐标**（`map_xyz` 与 `pose` 一致）。

**但质量标记是 `CAMERA_LOCAL`**，不是 SLAM 系 —— 说明**没有走 `plain_slam` provider 的
`T_map_camera` 路径**，因此也不会产生带 `map_frame` 的 `semantic_object_localized` 事件。

**下一步（明确）**：查为什么 `spatial_provider=plain_slam` 没生效。
worker 日志**不在** `runtime/go2w/search_sessions/`（该目录为空），需先定位 worker 的
stdout/日志，再确认 provider 初始化是否报 `SpatialFrameMismatch` 或走了降级分支。

**仍欠**：`map_frame` 字段的会话级证据（本次会话未出现）。

**P1 在线稳定性验证（2026-09-09 09:49–09:57，三次录制）**

| 录制 | 旋转序列 | skew | overlap_rmse | inlier | endpoint_yaw | 备注 |
|---|---|---|---|---|---|---|
| A（09-08 18:46） | `l30,r30`×2 | 无 | 0.0773 | 0.928 | 1.248° | 基线 |
| A′（09-09 09:49） | `l30,r30`×2 | **−25 ms** | **0.0654** | 0.962 | 6.527° | 旋转打滑 |
| A″（09-09 09:53） | `l30,r30`×2 | −25 ms | 0.1359 | 0.614 | **32.92°** | **旋转失败，数据不可用** |
| A‴（09-09 09:56） | `l15,r15`×2 | −25 ms | 0.0705 | **0.973** | 2.355° | 旋转幅度不同 |

**同幅度对比（`l30,r30`×2）**：无 skew **0.0773** → 含 skew **0.0654**（**−15%**），
inlier 0.928 → 0.962。

**必须标注**：

- 第二次（09:53）旋转执行**失败**（`final relative yaw error exceeds acceptance limit`，
  `endpoint_yaw = 32.92°`）→ 该条数据**不可用于 A/B**（首末帧看的是不同区域）；
- 第三次改用 `l15,r15` 成功，但**旋转幅度与基线不同**，只能作参考；
- 连续两次 `l30` 旋转失败时机器狗 **soc=96%、32.0 V**，**不是电量问题**（疑旋转控制超调）。

**结论**：**同幅度下 25 ms 偏移使 overlap 改善 15%、inlier +3.4 pp**，
与离线可重放 A/B（−18%）方向一致。

**P1 在线试用（2026-09-09 09:49，真机）**

按交接书在 launch 里给 `imu_fallback_adapter` 加 `--stamp-offset-s -0.025`（**可回退**），
重启映射栈（§9 检查通过：`mode=OFFSET_LOCKED_SENSOR_STAMP`、`map_3d` 发布者=1、
`gyro_bias=[0.01211, 0.00152, -0.00114]`、`imu_n=21`、`pos≈原点`），
重录 Dataset A′ 并执行同一序列 `l30,r30,l30,r30`：

| 指标 | Dataset A（无 skew，09-08） | **Dataset A′（+25 ms，09-09）** | 变化 |
|---|---|---|---|
| `overlap_rmse_m` | 0.0773 | **0.0654** | **−15%** |
| `overlap_inlier_ratio` | 0.928 | **0.962** | +3.4 pp |
| `surface_thickness_p95_m` | 0.0654 | 0.0633 | 略好 |
| `endpoint_translation_error_m` | 0.1935 | 0.1839 | 略好 |
| `endpoint_yaw_error_deg` | 1.2484 | **6.527（FAIL）** | 本次旋转未回原位 |

**结论**：**在线加 25 ms 偏移后 `overlap_rmse` 改善 15%、inlier 改善 3.4 pp**，
方向与离线可重放 A/B（−18%）**一致**。

**必须标注的限制**：两次是**不同时间的物理运动**（`endpoint_yaw` 1.25° vs 6.53°），
**运动执行未控**；`endpoint_yaw` 本次 FAIL 属旋转打滑，与 skew 无关
（`overlap_rmse` 取的是已在 `pslam_odom` 系的 raw 值，不受 endpoint 影响）。

**处置**：参数**暂保留**（有改善、无恶化），launch 注释已写明"真机对照若无改善则删除回退"。
**下一步**：同一配置再录 1~2 次确认稳定性。

**两个新失败的定位（2026-09-08 20:1x）—— 均为预先存在，非本次改动**

| 测试 | 失败原因 | 判定 |
|---|---|---|
| `test_manual_web_demo_config.py::test_hold_mode_params` | 断言 `turn_yaw_rate == 0.15`，而代码默认值已是 **1.0**（`app/manual_web_demo/config.py:96` 与 `:211`）→ **测试过时** | 预先存在 |
| `test_webui_video_navigation.py::test_render_helper_imports` | `ModuleNotFoundError: No module named 'pandas'`（`streamlit_app.py:10`）→ **缺环境依赖** | 预先存在 |

**处理原则**：两者都**不修测试、不改配置迎合**。

- `turn_yaw_rate` 属既有业务配置漂移（本次工作未触碰 `app/manual_web_demo/`）；
- `pandas` 属环境缺失，装它只为让 `streamlit_app` 可导入，与 SLAM 目标无关，**本轮不装**（避免引入大依赖）。

**当前 baseline 失败清单（3 项）**：

1. `test_plain_slam_config_generation.py::test_lio_runtime_params_extrinsics_and_tuning`
   （`acc_scale` 0.954 vs 断言 1.0）
2. `test_manual_web_demo_config.py::test_hold_mode_params`
   （`turn_yaw_rate` 1.0 vs 断言 0.15）
3. `test_webui_video_navigation.py::test_render_helper_imports`（缺 `pandas`）

**P4 全量回归完成（分批策略，2026-09-08 20:1x）**

`pytest tests/` 一次跑会在 63% 死锁，改用**按文件分批 + 每文件 `timeout 10`**：

```text
223 个测试文件全部执行完毕（约 5 分钟）
187 个文件全部通过
  3 个文件有失败：
    tests/test_manual_web_demo_config.py        1 failed, 5 passed
    tests/test_plain_slam_config_generation.py  1 failed, 4 passed   <- 已记录的 baseline 失败（acc_scale）
    tests/test_webui_video_navigation.py        1 failed
其余约 33 个文件为 collection error（缺依赖，非本次改动引入）
```

**结论**：**全量回归已执行完毕**（分批策略），**核心模块无新增回归**。
两个失败待定位（`test_manual_web_demo_config`、`test_webui_video_navigation`），
一个是已记录 baseline。
**这不等于"全量通过"** —— 3 个失败 + 33 个依赖缺失文件已如实列出。

**经验（已写进交接书）**：`pytest tests/` 一次跑会在 63% 死锁，**必须按文件分批**。

**P4 全量回归：63% 处仍阻塞（2026-09-08 20:0x）**

定位过程：

1. `-v` 跑到 53% 时最后一行是
   `test_plain_slam_webui_3d.py::test_capacity_overflow_coarsens_globally_and_keeps_every_region`；
   另一次跑到 57% 时是
   `test_regression_search_20260901_211756.py::TestFixesPreventRegression::test_no_printf_style_ros_logger_calls_remain`。
2. 装 `pytest-timeout 2.4.0`（顺带把 pytest 升到 9.1.1）并加 `--timeout=20` → **仍卡在 63%**，
   说明阻塞发生在测试体之外（导入/资源层），`--timeout` 抓不到。
3. 显式 `--deselect` 上述两个测试 → **仍在 63% 卡住**，即还有**至少一个未定位的阻塞点**。

**结论（诚实）**：**全量回归仍未完成**。当前回归依据是相关子集
（8 文件 **52 passed / 1 已记录 baseline failed**；本轮新增确认
`test_map_quality_evaluator` + `test_slam_bridge` + `test_plain_slam_spatial_provider` = **21 passed**）。

**下一步**：改用**分批策略** —— 按测试文件分组跑（每组套 `timeout`），逐组确认后再汇总；
**不要再用"一次跑全量"的方式**（已三次卡在同一位置）。

**P4 全量回归：进度 63% 后阻塞（2026-09-08 19:5x）**

环境修复过程：

- 缺 `openai` 模块 → 已装（`openai 3.8.0`；注意 **pip 必须清空全部代理变量**：
  `env -u http_proxy -u https_proxy -u all_proxy -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY` + `--proxy ''`）；
- 装 openai 带入 `anyio`，其 pytest 插件与系统 pytest 不兼容
  （`ModuleNotFoundError: No module named '_pytest.scope'`）→ 用 **`-p no:anyio`** 禁用。

运行 `python3 -m pytest tests/ -q -p no:anyio --continue-on-collection-errors`：

- 仍有 **33 个测试文件 collection error**（多为缺依赖，非本次改动引入）；
- 实际执行到 **63% 处阻塞**（某测试长时间不返回，疑似等待网络/ROS 超时），
  已完成部分出现 **2 个 F**；
- **未取得全量通过证据**。

**结论**：**全量回归尚未完成**，不得用"跑过了"描述。当前回归依据仍是相关子集
（`test_plain_slam_*` + `test_slam_bridge` + `test_map_quality_evaluator` 等 8 文件 =
**52 passed / 1 已记录 baseline failed**）。
**下一步**：定位那个阻塞测试并加 `--timeout` 或排除，再跑完整回归。

**P1 重复验证（2026-09-08 19:48，每配置 2 次）**

| `--stamp-offset-s` | 运行 | traj_n | overlap_rmse | inlier |
|---|---|---|---|---|
| 0 | run1 | 7750 | 0.0834 | 0.868 |
| 0 | run2 | 7497 | 0.0796 | 0.909 |
| **−0.025** | run1 | 5802 | **0.0665** | 0.925 |
| **−0.025** | run2 | 4940 | **0.0667** | 0.927 |

**统计**：`skew=0` 中位数 **0.0815**；`skew=−0.025` 中位数 **0.0666**。
**改善 −18%**，inlier 中位数 0.888 → 0.926（**+3.8 pp**）。

**两次重复高度一致（0.0665 vs 0.0667）** —— 尽管 `traj_n` 仍有抖动（7750/7497 vs 5802/4940），
说明结论对时序抖动不敏感。

**结论**：**25 ms LiDAR–IMU 时间偏移补偿能改善旋转漂移**
（overlap −18%、inlier +3.8 pp，两次重复一致）。这是本项目目前最强的正面结论之一。

**下一步（谨慎）**：先在**在线系统**上以可回退方式试用 —— 给 `imu_fallback_adapter` 传
`--stamp-offset-s -0.025`，重录 Dataset A′ 对照；**不要直接改默认值**，
并按交接书 §9 验证 `imu_status.mode` 与 `map_3d` 发布者。

**P1：25 ms 时间偏移的收益（首次 A/B，2026-09-08 19:46）**

play-first 修复后，用同一 `/tmp/ab_bag` 跑两次重放，**唯一变量**是 `--stamp-offset-s`：

| 配置 | overlap_rmse | p95 | inlier | traj 帧数 |
|---|---|---|---|---|
| `0`（现状） | **0.0834** | 0.1720 | 0.8678 | 7750 |
| `-0.025`（补 25 ms） | **0.0665** | 0.1275 | **0.9253** | 5802 |

**方向是正面的**：补 25 ms 后 `overlap_rmse` 降 20%、`inlier` 升 5.7 个百分点。

**但必须标注不确定性**：两次重放的轨迹帧数不同（7750 vs 5802），说明录制窗口/运动内容
不完全一致（重放实时进行，节点启动时序有抖动）。**单次对比不足以定论** ——
需要重复 2~3 次取中位数，或改用固定窗口（`--start-offset` + 固定时长）后再比较。

**结论**：25 ms 时间偏移**是旋转漂移的可行靶子**（方向正确、量级合理），
但**尚未达到"已验证有效"的标准**，**不要据此改在线时间戳**。

**P0 已修复：让 bag play 先启动（2026-09-08 19:45）**

把启动顺序改成「**先起 `ros2 bag play --clock 100`，等 8 秒，再起各节点**」后重放：

```text
diag=163
dsk_max = 0.0028 / 0.0029         <- 修复前 12.2769（约 4000 倍改善）
pos     = (0.042, -0.040, 0.003)  <- 修复前 (828.415, -719.435, -421.484)
v       = 0.026                   <- 修复前 125.797
```

**结论**：重放模式下 LIO 发散**已解决** —— 根因确实是 `/clock` 晚于 LIO 出现导致时间基准跳变。
bag 正常录到 `odom_base` 5796 帧、`aligned_scan` 113 帧。

**P0 完成 → P1（时间偏移收益验证）现在可以做了**：用同一 bag 跑 `--stamp-offset-s 0` 与 `-0.025` 两次，
比较 `scan_overlap_metrics`。`/tmp/ab_lio_run.sh` 已改成 play-first 顺序。

**P0 的决定性诊断：重放模式下 LIO 因时间/去畸变基准异常而发散（2026-09-08 19:5x）**

从 `/tmp/ab_o2` 的 `odom_base` 分窗口看：

| 窗口 | 帧数 | 末位姿 | max_abs |
|---|---|---|---|
| 前 2 s | 458 | (47.89, −3.02, −7.36) | 58.27 m |
| 前 5 s | 1175 | (274.74, 52.33, 9.10) | 336.17 m |
| 前 10 s | 2372 | (578.66, 59.75, 19.06) | 657.47 m |
| 前 29 s | 6201 | (827.27, −725.07, −425.34) | 1977.67 m |

而 LIO 自己的诊断日志给出了根因：

```text
第一帧: imu_n=6  pos=(-0.000,-0.000,-0.001) v=0.026   dsk_max=0.0015  pre_t=0.0008   <- 正常
末  帧: imu_n=25 pos=(828.415,-719.435,-421.484) v=125.797 dsk_max=12.2769 pre_t=12.7107 <- 爆炸
```

即 **LIO 内部确实从原点起步**，但 `dsk_max=12.28`（去畸变时间跨度 **12 秒**）、
`pre_t=12.71`（预测平移 12.7 m）、`v=125.8 m/s` —— 是**时间/去畸变基准异常**，不是"初始化慢"。

**结论**：重放模式下 `--clock` 与点云/IMU 的时间基准不一致（LIO 启动时尚未收到 `/clock`，
随后时间跳变），松耦合预积分随之发散。**重放 A/B 需要换时间方案**（例如
`ros2 bag play --start-paused` 让所有节点就绪后再播放，或改用 wall clock 重录），
**在解决之前 P1（时间偏移收益）无法进行**。

**重放模式下 LIO 发散——现在有了正确证据（2026-09-08 19:41）**

补上 `plain_slam_odom_adapter` 后，bag 终于包含 `/go2w/slam/odom_base`（7381 条）：

```text
first (21.081, -1.332, -3.337)
last  (827.269, -725.067, -425.339)
max_abs 1977.67 m
```

即：**重放模式下 LIO 的位姿从 21 m 起步、29 秒内发散到 827 m**（`max_abs` 1.98 km）。
这与"aligned_scan 跑到公里外"一致，现在由 odom_adapter 的权威输出**直接证实**。

**因此**：

1. **重放模式的 LIO A/B 目前不可用** —— LIO 在该模式下本身就不正常；
2. 之前的 1.456° / 1.455° 是重放旧数据的产物，已撤回；
3. 必须先解决"重放模式下 LIO 发散"，A/B 才有意义。

**嫌疑与下一步**：`--clock` 让时间从 bag 首帧（19:12）开始，而 LIO 在 19:41 才启动，
松耦合预积分在**时间倒退**时必然发散。下一步试：
① 先只重放**静止段**（前 5 秒）确认 LIO 在重放下能否收敛；
② 若不行，改用 wall clock（并让 adapter 不判陈旧）。

**重放 A/B 的第三个缺陷：漏启动 `plain_slam_odom_adapter`（2026-09-08 19:40）**

列出 `ab_s0b` 的 bag 内容：

```text
topics = [(1, '/go2w/slam/map_3d'), (2, '/go2w/slam/aligned_scan')]
counts = [(1, 128), (2, 253)]
```

**没有 `/go2w/slam/odom_base`、也没有 `/go2w/odom/fused`** —— 因为 `/tmp/ab_lio_run.sh` 只启动了
`imu_fallback_adapter`、`pandar_slam_adapter`、`lio_3d_node`、`slam_3d_node`，
**漏了 `plain_slam_odom_adapter`**（它才是 `/go2w/slam/odom_base` 的发布者）。

**这解释了两件事**：

1. 之前 `/tmp/ab_skew0` 里读到的 "odom_base" 其实是**源 bag 里重放出来的在线系统数据**，
   根本不是 domain 42 新 LIO 的输出 → **"LIO 净旋转 1.456° vs 1.455°" 完全无效**
   （上一轮已撤回，现在有了确切原因）；
2. 新 LIO 的 `aligned_scan` 从**第一帧**就在 x 11–39 m（前 3 帧 min x = 11.03 / 12.81 / 13.31），
   与其自身 odom 无从对照，所以"发散"这个判断本身也还没有依据。

**修正**：脚本必须补上 `plain_slam_odom_adapter`（参数见 launch 文件），再重跑 A/B。
**在此之前，任何"LIO 层"结论一律无效。**

**重放模式下 LIO 位姿异常（2026-09-08 19:37）——之前的 LIO 层 A/B 结论撤回**

加上 `--topics`（只播输入，修正上一节的污染）后重跑，末帧 `aligned_scan` 仍在：

```text
last_scan min [-630.92, 1184.32, -30.29]  max [-618.41, 1207.69, -10.69]
```

即 LIO 在**重放模式下位姿发散到公里级**；而 `imu_n=25`、点云也在处理 —— 不是缺数据。

**重要影响**：**之前"LIO 层 A/B（零偏）1.456° vs 1.455°"的结论必须撤回** ——
它同样跑在重放模式下，而 LIO 在该模式下本身行为异常，那两个数字不能用来证明"零偏无影响"。
（adapter 层的严格 A/B 不受影响，它只验证 IMU 信号，不依赖 LIO。）

**嫌疑**：模拟时钟。`ros2 bag play --clock` 从 bag 第一个时间戳（19:12）开始，而 LIO 节点在 19:36
才启动，时间"倒退"会让松耦合预测发散。

**下一步（必做）**：先用**静止段**验证重放模式下 LIO 能否正常收敛（例如只重放前 5 秒静止数据），
确认后再谈任何 A/B。**在此之前，所有"重放模式"的结论一律挂起。**

**skew 重放 overlap 仍异常的根因（2026-09-08 19:38）**

直接查两个重放 bag 的**最后一帧** `aligned_scan`：

```text
skew0   last frame pslam_odom  min [612.99, 1230.50, 879.91]    max [625.52, 1245.54, 890.03]
skew25  last frame pslam_odom  min [3434.40, 1409.07, -5234.47] max [3459.17, 1425.74, -5200.22]
```

即 domain 42 的 LIO 在重放期间**位姿发散到数百米乃至数千米外**，而同一 bag 里读到的
`odom_base` 却只有 0.371 m —— 两者矛盾，说明数据被**混合**了。

**根因**：源 bag `/tmp/ab_bag` 里**包含了在线系统的 `/go2w/slam/odom_base` 与 `/go2w/odom/fused`**。
重放时它们被重新发到 domain 42，与 domain 42 新 LIO 的输出**混在一起**（两边都是模拟时钟），
`load_bag` 读到的就是混合序列 → 位姿与点云错配。

**修正**：重放时只播**输入**：

```bash
ros2 bag play --clock 100 --topics /utlidar/imu /hesai/pandarxt16/points_raw /tmp/ab_bag
```

**在改这一行之前，两次 skew 重放的 overlap 结论一律不可用。**

**重要区分**：Dataset A / B 的 overlap（0.0773 / 0.0433）是**在线录制**的，不存在这个污染，
因此这两个数字仍然有效。

**overlap 双重变换修正与重算（2026-09-08 19:36）**

修正两处：

1. `load_bag` 记录 `aligned_scan` 的 `frame_id`；
2. `scan_overlap_metrics` 若点云已在 odom/map 系（实测就是 `pslam_odom`）则**不再变换**，
   且**不再做位姿补偿**——点云已在世界系，首末帧的位置差已含 LIO 的对齐结果，
   再补偿一次是**重复补偿**（实测把 B 从 0.0433 放大到 0.0680）。

重算结果：

| 数据集 | 修正前 overlap | **修正后 overlap** | inlier | pose_correction |
|---|---|---|---|---|
| Dataset B | 0.0658 | **0.0433** | 0.978 | false |
| Dataset A | 0.0773 | 0.0773 | 0.928 | false |

- **B 明显改善**（0.0658 → 0.0433，inlier 0.923 → 0.978），修正方向正确；
- **A 数值不变**——这暗示 A 之前的位姿补偿可能本来就没生效（若 `_pose_at` 对首末帧
  取到同一姿态，delta 近似单位变换）。**A 的 overlap 仍需单独复核。**
- 两次 skew 重放（`/tmp/ab_skew0`、`/tmp/ab_skew25`）尚未用修正后的指标重算。

**重大发现：`aligned_scan` 已经在 `pslam_odom` 系（2026-09-08 19:33）**

实测重放 bag 里 `aligned_scan` 的消息头：

```text
frame_id pslam_odom   n 61127   min [10.15, -3.06, -6.56]   max [36.36, 9.02, 1.78]
```

**点云已经以 `pslam_odom` 为坐标系**（数值本身就是 odom 坐标，x 从 10 m 到 36 m）。
而 `scan_overlap_metrics` 仍把它当作**传感器系点云**、再用位姿变换一次 → 方法上是错的。

这解释了重放数据上 `overlap_rmse = 1631 m` 的荒谬值（精确来源仍需定位，但双重变换 +
时间戳匹配问题足以让它失效）。

**必须复查**：Dataset A/B 的 overlap（0.0773 / 0.0658）也是在同样的双重变换下算出来的，
只因当时位姿接近原点而数值看起来合理。**这两个数字的可信度需要重新评估。**

**修正方向**：`scan_overlap_metrics` 先读 `frame_id` —— 若已是 odom/map 系则**不再变换**，
只有传感器系点云才用位姿变换；改完重算 A / A′ / B 与两次 skew 重放。

**时间偏移 A/B 的第一次尝试（2026-09-08 19:30）**

- `imu_fallback_adapter` 新增 `--stamp-offset-s`（默认 0.0，**不影响在线行为**）；
- A/B 脚本 `/tmp/ab_lio_run.sh` 已支持第三参数 `stamp-offset-s`，并把
  `/go2w/slam/aligned_scan`、`/go2w/slam/map_3d` 纳入录制；
- 已跑两次重放：`/tmp/ab_skew0`（skew=0）与 `/tmp/ab_skew25`（skew=−0.025），
  各约 1.1 GB，`imu_n=20~25`。

**但 overlap 指标在重放数据上不可用**：`scan_overlap_metrics` 对 `/tmp/ab_skew0` 给出

```text
overlap_rmse_m = 1631.85   overlap_inlier_ratio = 0.0
```

1631 m 显然是**位姿/时间戳错配**而非真实漂移。嫌疑：重放时点云带**模拟时钟**（19:12），
而 `load_bag` 用 bag 的**记录时间戳**（wall clock）做轨迹匹配 → 取到错误位姿。

**本轮结论：偏移参数已就位、A/B 数据已落盘，但收益结论未产出**——
**不得写成"偏移有效/无效"**。下一步先修 `load_bag` 的时间匹配（改用消息 header stamp
或统一用 bag 记录时间），再重算两次的 overlap。

**LIO 层 A/B 本体（2026-09-08 19:25，严格同 bag 两次重放）**

修掉两个坑后跑通（显式 `-p` + `ros2 bag play --clock 100` + 全节点 `use_sim_time:=true`）：

```text
GO2W-DIAG 计数 236 / 292；imu_n=25；开零偏那次 gyro bias locked [0.01072,-0.00007,-0.00050]
```

| 配置 | LIO 帧 | wheel 帧 | LIO 净旋转 | wheel 净旋转 |
|---|---|---|---|---|
| `--bias-lock-samples 500` | 253 | 506 | 1.456° | 1.282° |
| `--bias-lock-samples 0` | 252 | 505 | **1.455°** | 1.278° |

**两次几乎逐位相同（差 0.001°）。**

**结论（再次修正判断）**：

1. 零偏修正在 **IMU 信号层**有效（adapter 层严格 A/B 已证：逐轴差 = 锁定零偏）；
2. 但在 **LIO 层没有可测差异** —— LIO 是松耦合，IMU 只做预测、扫描匹配做修正，
   零偏造成的姿态偏差被扫描匹配吸收（环境特征足够）；
3. 因此上一轮那个"LIO-wheel 一致性改善 5 倍"**很可能是两次 bag 的旋转幅度差异造成的假象**
   （6.16° vs 1.25°），而不是零偏修正的功劳。**该结论已被本轮严格 A/B 推翻。**
4. 含义：**继续在陀螺零偏上加码没有收益**；旋转漂移应回到 §26.2 的
   `extrinsic rotation` / `deskew` / `LiDAR–IMU timestamp offset`（25 ms 已量到）。

**LIO 层 A/B 第二次尝试：参数已生效，但撞上"模拟时钟"问题（2026-09-08 19:21）**

改用显式 `-p` 后有明显进展：

```text
LiDAR type: Unknown (hesai_pandarxt16)     <- 参数生效（不再是 Livox）
GO2W-DIAG 计数 = 292                        <- LIO 确实在处理点云
imu_n=0                                     <- 但仍然没有 IMU
```

adapter 日志给出原因：

```text
[WARN] /utlidar/imu has no real data; publishing synthetic static IMU on /go2w/slam/imu
```

**根因**：`ros2 bag play` 默认保留**录制时的时间戳**（19:12），而 adapter 用**接收时刻**判断新鲜度 →
认为数据陈旧，退化为 synthetic IMU；synthetic 用的是**当前时间**（19:21），与重放点云时间戳
（19:12）相差 9 分钟，LIO 因此找不到匹配的 IMU 样本 → `imu_n=0`。

**修正方案（下一轮）**：重放时启用**模拟时钟** —— `ros2 bag play --clock`，并给**所有节点**加
`-p use_sim_time:=true`，让 IMU/点云/时钟统一在录制时的时间轴上；这样新鲜度检查与
IMU-点云时间匹配都会正常。脚本 `/tmp/ab_lio_run.sh` 只需再加这几处 `use_sim_time`。

**LIO 层 A/B 第一次尝试失败与根因（2026-09-08 19:19）**

按上述命令在 domain 42 启动了 4 个节点 + 重放 + 录制，但 **LIO 没有处理任何点云**（`GO2W-DIAG` 计数 0）。
日志给出根因：

```text
[go2w_plain_slam_lio]: LiDAR type: Livox          <- 期望 hesai_pandarxt16
```

`generated_lio_3d_config.yaml` 的参数键是 **`lio_3d_node`**，而运行时节点名被改成
`go2w_plain_slam_lio`，因此 `--params-file` **匹配不上、参数全部未生效**（LIO 退回上游默认 Livox 配置）。
这正是 launch 文件为什么要"读出 YAML → 转扁平 dict → 作为 Node 的 parameters"，而不是直接 `--params-file`。

**修正方案（下一轮照做即可）**：不用 `--params-file`，改为显式传参：

```bash
-p lidar_type:=hesai_pandarxt16 -p use_as_localizer:=false -p param_files_dir:=$R \
-p pointcloud_topic:=/go2w/slam/pandar_points -p imu_topic:=/go2w/slam/imu \
-p imu_pose_topic:=/go2w/slam/imu_pose_raw -p imu_odom_topic:=/go2w/slam/imu_odom_raw \
-p lio_map_cloud_topic:=/go2w/slam/lio_map_cloud \
-p aligned_scan_cloud_topic:=/go2w/slam/aligned_scan \
-p deskewed_scan_cloud_topic:=/go2w/slam/deskewed_scan \
-p odom_frame:=pslam_odom -p imu_frame:=pslam_imu
```

（`slam_3d_node` 同理，键名是 `slam_3d_node`。）脚本 `/tmp/ab_lio_run.sh` 已就位，只需替换这一处。

**LIO 层 A/B 的可行性已验证（2026-09-08 19:17）**

在 domain 42 内单独启动 `slam_3d_node` 并覆盖两个 ROS 参数：

```bash
ROS_DOMAIN_ID=42 ros2 run plain_slam_ros2 slam_3d_node --ros-args \
  -r __node:=go2w_plain_slam_slam \
  --params-file runtime/go2w/plain_slam/generated_slam_3d_config.yaml \
  -p param_files_dir:=$PWD/runtime/go2w/plain_slam \
  -p map_cloud_dir:=/tmp/go2w_ab/
```

实测输出：`SLAM 3D node initialized`，退出时 `Wrote map cloud to /tmp/go2w_ab//map_cloud.pcd`
—— **`map_cloud_dir` 覆盖生效，在线栈（domain 0）完全未受影响**。

**源码依据**：`slam_3d_node.cpp:48` 把 `map_cloud_dir` 声明为 ROS 参数；
`lio_3d_node` 只在 `use_as_localizer=true` 时读它（`lio_3d_node.cpp:133`），本项目是建图模式，
所以只需覆盖 slam 侧。

**下一步（无需再试探）**：在 domain 42 依次启动
`pandar_slam_adapter` → `imu_fallback_adapter`（`--bias-lock-samples N`）→ `lio_3d_node` → `slam_3d_node`，
同时 `ros2 bag play /tmp/ab_bag`，录 `/go2w/slam/odom_base`；开/关零偏各跑一次，比较 LIO 相对旋转。

**严格可重放 A/B（2026-09-08 19:15，同一 bag 两次重放）**

方法：在**独立 DDS 域**（`ROS_DOMAIN_ID=42`，`CYCLONEDDS_URI` 为 `<Domain Id="any">` 不会写死域）内重放
`/tmp/ab_bag`，启动一个临时 `imu_fallback_adapter`（订阅 `/utlidar/imu` → 输出 `/go2w/slam/imu`）
并录制其输出。两次运行**唯一变量**是 `--bias-lock-samples`（500 vs 0）；在线系统留在 domain 0，**完全未受影响**。

| 配置 | 锁定前 0–2 s gyro 均值 (x,y,z) | 锁定后 >5 s gyro 均值 (x,y,z) |
|---|---|---|
| `--bias-lock-samples 500`（开） | [0.01068, 0.00014, −0.00030] | **[0.00092, −0.00013, 0.00161]** |
| `--bias-lock-samples 0`（关） | [0.01072, 0.00011, −0.00033] | **[0.01136, 0.00004, 0.00133]** |

**结论**：

1. 锁定前两者几乎逐位相同（0.01068 vs 0.01072）→ **输入完全一致，A/B 成立**；
2. 锁定后 x 轴差 **0.01044 rad/s**，与锁定的零偏 **0.01074** 吻合（差 0.0003）；
   y 轴差 0.00017 ≈ 零偏 0.00018；z 轴差 −0.00028 ≈ 零偏 −0.00029 —— **逐轴精确复现**；
3. 零偏修正的**确定性**由此被严格证明，不再依赖"不同 bag 对比"；
4. 锁定后均值不为零属正常：bag 内含 `l30,r30` 旋转，gyro 本身非零。

**仍欠**：本轮只完成 **adapter 层** A/B。**LIO 层**（地图/overlap）的可重放 A/B 还需在 domain 42 内
再起一套 `lio_3d_node + slam_3d_node`，并把 `map_cloud_dir` 指向**独立目录**
（否则与在线栈争用 `/tmp/go2w_plain_slam/`）。

**可重放 A/B 的准备工作（2026-09-08 19:12）**

- `scripts/record_slam_validation.sh` 新增 `--with-raw`：追加 `/hesai/pandarxt16/points_raw` 与 `/utlidar/imu`。
- 已录制可重放 bag：`/tmp/ab_bag`（29.3 s；`/utlidar/imu` 7285 帧、`points_raw` 293 帧、
  `odom_base` 293 帧、`odom/fused` 586 帧），期间真实执行 `l30,r30`。

**执行方案与已知障碍（写清，避免下一个人踩）**：

1. 重放前必须**停掉在线 LIO 栈**，否则新旧 LIO 会同时发布 `/go2w/slam/odom_base`；
2. `/hesai/pandarxt16/points_raw` 由控制主机的 hesai 驱动发布 → 停驱动即可消除冲突；
3. **`/utlidar/imu` 由机器狗 DCU 发布，控制主机停不掉** → 必须
   `ros2 bag play --remap /utlidar/imu:=/utlidar/imu_replay`，再起一个临时
   `imu_fallback_adapter` 订阅 `_replay`（输出仍写 `/go2w/slam/imu`），否则混合流会让
   adapter 的时间戳跳变并 fail-closed；
4. 两次重放分别用 `--bias-lock-samples 0`（关）与 `500`（开），其余配置完全一致；
5. 比较对象用"LIO 相对旋转 vs wheel 相对旋转"（不必读 1 GB 地图，30 s 出结果）。

**尚未执行**：受本轮风险/时间预算限制，A/B 本身未跑；下一次按上述 5 步直接执行，无需再改代码。
**推荐替代**：改用独立 DDS 域（`ROS_DOMAIN_ID=42`）跑离线 LIO，不必停在线系统与 hesai 驱动。

**广谱回归（2026-09-08 19:4x）**

本轮共改动 6 个文件（评估器、SLAMBridge、provider、IMU 适配器、语义探索脚本、配置生成器），
对相邻模块跑了一次广谱回归：

```text
tests/test_plain_slam_spatial_provider.py  tests/test_plain_slam_frame_contract.py
tests/test_plain_slam_timestamp_policy.py  tests/test_plain_slam_config_generation.py
tests/test_plain_slam_occupancy_projection.py  tests/test_slam_bridge.py
tests/test_map_quality_evaluator.py  tests/test_spatial_transform.py
=> 52 passed, 1 failed in 0.62s
```

唯一失败是**已记录的 baseline 失败** `test_lio_runtime_params_extrinsics_and_tuning`
（断言 `acc_scale==1.0`，而配置已按实测修正为 `0.954`），与本轮改动无关。
**无新增回归。**

**零偏修正的效果验证（用轮式里程计作参考，2026-09-08 19:3x）**

把 LIO 的相对旋转与 `/go2w/odom/fused` 的相对旋转对比（重采样 50 Hz，两者都取相对首帧的增量）：

| bag | LIO 净旋转 | wheel 净旋转 | 相对偏差 RMS | 最终偏差 | 峰值偏差 |
|---|---|---|---|---|---|
| A（修正前） | 1.25° | 1.51° | **1.369°** | −0.259° | **6.658°** |
| A′（修正后） | 6.16° | 6.06° | **0.272°** | 0.103° | **3.309°** |

**结论（修正上一轮的判断）**：

- 修正后 **LIO 与轮式里程计的旋转一致性改善约 5 倍**（RMS 1.369° → 0.272°），
  峰值偏差 6.658° → 3.309°；且 A′ 的旋转幅度（6.16°）比 A（1.25°）**更大却更一致**；
- 上一轮我根据"yaw 轴偏置只有 0.023°/s"推断"零偏不是主因"——**该推断不完整**：
  gyro-x 偏置会通过姿态耦合影响旋转估计，实测一致性数据支持**修正有效**；
- **限制**：这仍是两个不同 bag 的对比（旋转幅度不同），不是严格 A/B；
  但两项独立指标（RMS 与峰值）同向改善，可信度较高。

**但 A′ 的 overlap 补偿后仍 0.1751 FAIL** → 除零偏外还有其它因素，
首要候选就是下面量到的 **25 ms LiDAR–IMU 时间偏移**。

**LiDAR–IMU 时间偏移诊断（2026-09-08 19:2x）**

用 IMU 的 `gyro_z` 与 LIO `odom_base` 的 yaw rate 做互相关（重采样到 100 Hz，滞后扫 −0.20…+0.20 s）：

| bag | 最佳滞后 | 该滞后相关性 | 零滞后相关性 |
|---|---|---|---|
| A（旋转，修正前） | **−0.025 s** | 0.9535 | 0.9438 |
| A′（旋转，修正后） | **−0.025 s** | 0.9855 | 0.9759 |
| B（平移往返） | −0.015 s | 0.4892 | 0.4199 |

**结论**：

1. 两次旋转 bag 的滞后**稳定在 −25 ms**、完全一致 → **系统性时间偏移，不是噪声**；
2. 平移 bag 的相关性只有 0.49（yaw rate 太小、信噪比低），其滞后估计不可信；
3. 25 ms 在 1.2 rad/s 的旋转下等于 **1.7°** 的角度失配；而 0.13 m 墙面错位在 4 m 距离上约 **1.9°** ——
   **量级吻合**，时间偏移是旋转漂移的有力候选。

**修复候选（尚未实施）**：给点云或 IMU 时间戳补偿 25 ms。
**风险与前置条件**：改时间戳会影响所有下游消费者，而且当前**无法验证收益**（没有可重放数据）。
**必须先做可重放 A/B**，再决定是否改——不得凭相关性就直接调时间戳。

**零偏修正的信号层验证（2026-09-08 19:1x）**

| bag | 静止段 gyro 均值 (x,y,z) rad/s | gyro_z 积分 | odom yaw 净变化 | 峰值 gyro_z |
|---|---|---|---|---|
| A（修正前） | [0.01119, 0.00029, -0.00041] | −0.0155 | +0.0218 | 1.304 |
| A′（修正后） | **[-0.00054, 0.00075, 0.00029]** | 0.1071 | 0.1075 | 1.361 |

1. **零偏修正确实生效**：静止 gyro-x 从 0.01119 降到 −0.00054（20 倍），可复现、可直接从 bag 复核。
2. **但零偏不是旋转漂移的主因**：yaw 轴偏置只有 −0.0004 rad/s（0.023°/s），20 s 仅累积 0.47°，
   不足以解释 0.13 m 的墙面错位。**上一轮把 gyro-x 列为"首要嫌疑"是过头了**——x 轴偏置主要由
   重力/扫描匹配吸收，不直接进 yaw。
3. **A′ 的 IMU–LIO yaw 通道高度自洽**（gyro_z 积分 0.1071 vs odom yaw 0.1075，差 0.02°），A 则差 2.1°。
4. **A′ 的旋转执行本身不对称**：IMU 测得净转 **+6.1°**（A 只有 −0.9°），这正是 A′
   `endpoint_yaw=6.16°` 的来源，也解释了 A′ 的 overlap 为何更差。
5. **下一步靶子应转到 §26.2 的其余项**：`extrinsic rotation`（Pandar 外参仍 `candidate_unconfirmed`）、
   `deskew`、`LiDAR–IMU timestamp offset`——而不是继续在陀螺零偏上加码。

**旋转漂移诊断（2026-09-08，Dataset A bag 离线分析）**

| 对比 | overlap RMSE | P95 | inlier |
|---|---|---|---|
| 帧 0 vs 1（相邻） | 0.0068 m | 0.0119 | 1.000 |
| 帧 0 vs 2（相邻） | 0.0069 m | 0.0118 | 1.000 |
| 帧 743 vs 744（相邻） | 0.0115 m | 0.0118 | 1.000 |
| **帧 0 vs 744（首末）** | **0.1333 m** | 0.1973 | **0.482** |

相邻帧噪声底仅 0.007–0.012 m，而首末帧 0.1333 m 且**一半点未匹配** → 0.13 m 是**真实漂移**，
不是指标噪声；漂移发生在 4×±30° 旋转期间。

IMU 侧诊断（同一 bag）：

```text
imu 18503 帧（dt median 4.44 ms ≈ 225 Hz）；odom 745 帧
静止段 gyro 均值 = [0.01119, 0.00029, -0.00041] rad/s     <- x 轴仍有 0.64 deg/s 零偏
静止段 acc  均值 = [2.955, 0.0006, 9.8053] m/s^2          <- 模长 10.24（已由 acc_scale 0.954 补偿）
gyro_z 积分 = -0.0155 rad；odom yaw 净变化 = +0.0218 rad
```

**首要嫌疑**：静止时 gyro-x 仍有 **0.0112 rad/s** 零偏。20 s 即累积 0.22 rad（12.8°）roll 漂移，
而 roll 漂移会让扫描倾斜、墙面错位——与观察到的旋转漂移一致。
**下一步**：在 `imu_fallback_adapter` 的启动窗口里估**陀螺零偏**并减去（复用已有的 offset 锁定机制），
再用同一 bag 复测 `overlap_rmse`，而不是先调 SLAM 参数。

**A 的 `overlap_rmse` FAIL 指向旋转漂移**：同一份 bag，Dataset B（0.25 m 平移往返）的 overlap 是
0.0658 PASS，而 A（4×30° 旋转）是 0.1333 FAIL。即**原地旋转造成的墙面错位约为平移的两倍**，
与此前 episode 记录的「转 30° 假平移 0.085–0.154 m」一致。这是下一步压缩残差的直接靶子。

> **指标修正说明（2026-09-08）**：`surface_thickness_p95` 原先用固定 k=30 近邻，导致稀疏地图的
> 邻域跨过墙角/地面交界而虚高。实测 B 的厚度随 k 单调变化（k=6/10/20/30/60 → 0.052/0.077/0.134/0.186/0.310），
> 已改为**固定半径 0.20 m** 邻域；A 的厚度随之从 0.0963 修正为 0.0654，B 从 0.1856 修正为 0.0558。
> 这是修正一个错误指标，不是放宽阈值（阈值始终 0.15）。

bag 统计：`/go2w/slam/map_3d` 2 帧、`/go2w/slam/odom_base` 745 帧、`/go2w/slam/imu` 18503 帧、
`/go2w/slam/aligned_scan` 745 帧、`/tf` 1490 帧。

**必须如实指出**：`endpoint_translation_error_m = 0.1935 m` 已贴近 0.20 m 阈值——
4 次 ±30° 旋转后累积平移漂移约 19 cm，属"边界通过"，是下一步首要压缩指标。

**未测项**：360° 完整一圈（本轮受"每次移动后回原点"约束只做 ±30°）、
墙面/静态表面厚度 P95（需离线点云拟合，本轮未采集落盘点云）、
start/end submap overlap RMSE（需 Dataset B 落盘）。

---

## 4. Dataset B：原点往返（计划书 §19.1 / §20）

**采集方式**：真实运动 `f0.10,b0.10`（前进 0.10 m 再后退 0.10 m 回原点）。

```json
step_verified(f0.10): distance_m=0.09269210, verified=true
step_verified(b0.10): distance_m=0.08931380, verified=true
breadcrumb_consumed:  distance_m=0.1, recovery_budget={consecutive:1,total_m:0.1,max_total_m:0.36}
```

| 指标 | 值 | 阈值 | 判定 |
|---|---|---|---|
| 单步前进距离 | 0.0927 m | ≤ 0.10 m | PASS |
| 后退距离 | 0.0893 m | 0.05–0.10 m | PASS |
| 往返归位平移残差 | **0.00453 m** | — | PASS |
| 往返归位朝向残差 | **0.0109 rad（0.63°）** | — | PASS |
| start/end submap overlap RMSE | **0.0658 m** | ≤ 0.10 m | PASS |
| loop_closure_count | 未测 | ≥ 1 | NOT VERIFIED |

**Dataset B 实测（2026-09-08 19:00，`artifacts/slam_validation/datasetB_20260908_190022`）**

采集：`record_slam_validation.sh --dataset B --duration 70` + 真实 `f0.25,b0.25`（857 MB→1009 MB bag）。

```text
PASS          surface_thickness_p95_m        = 0.0558 m  (limit 0.15)   <- 修正邻域后（原 0.1856 是指标缺陷）
PASS          endpoint_translation_error_m   = 0.0722 m  (limit 0.20)
PASS          endpoint_yaw_error_deg         = 1.0360 deg (limit 5.0)
PASS          overlap_rmse_m                 = 0.0658 m  (limit 0.10)   <- 起/终子图在 odom 系重合良好
NOT_VERIFIED  object_position_* / loop_closure_count
accepted = true
```

**原先的"矛盾"已经查清**：B 的 `surface_thickness` 曾报 0.1856 FAIL，原因是**指标实现缺陷**——
固定 k=30 近邻在稀疏地图（B 仅 3123 点 vs A 的 14486 点）上把邻域拉大到跨过墙角/地面交界。
B 的厚度随 k 单调变化（k=6/10/20/30/60 → 0.052/0.077/0.134/0.186/0.310）即为铁证。
改为固定半径 0.20 m 后 B = 0.0558 PASS，A = 0.0654 PASS。**阈值从未改动**。

**另注（运动原语标定）**：本轮 `f0.25` 实测只走 **0.179 m**，`b0.25` 走了 **0.2595 m**，
前后步长不对称，导致 odom 净漂移约 8 cm。已记录，未调参掩盖。

**原因（为什么不是米级闭环）**：计划书要求的 Dataset C 需要"离原点米级往返/闭环"，
与用户明确的"每次移动后回到胶带原点、小幅度移动"约束冲突。用户已确认接受该约束并把大闭环标 NOT VERIFIED。

---

## 5. Dataset D：语义物体标注（计划书 §21 / §22）

**状态：NOT VERIFIED（本轮未采集）**

可复用的既有实现（已核对源码）：

- `app/perception/depth_object_localizer.py`（bbox 深度采样 → camera_xyz）
- `app/spatial/semantic_object_map.py`（几何优先关联、一对一贪婪分配防同类误合并、
  TENTATIVE/CONFIRMED、quality 加权融合）
- `scripts/go2w/run_autonomous_loop.py` 已输出每帧 `camera_xyz`、`depth_m`、`bearing_deg`

已存在的能力（源码核对，不是"未实现"）：

- `app/spatial/spatial_transform.py::camera_point_to_map()` 已实现
  camera_optical → base（标称外参 `(0.10, 0.0, 0.30)`，**无旋转**）→ world（平面位姿）的完整投影。
- `app/spatial/plain_slam_spatial_provider.py::camera_point_to_spatial()` 调用它，
  并强制 `pose.frame_id == pslam_odom`（frame 不匹配直接抛 `SpatialFrameMismatch`）。

与计划书 §10/§41 的真实差距：

1. **时间戳**：用的是 `pose or self.get_pose()`（当前位姿），**不是图像拍摄时刻的位姿**；
   计划书 §10 明确要求 `T_map_camera(t_image)`。
2. **目标 frame 是 `pslam_odom` 而非 `pslam_map`**：回环优化后 `map→odom` 改变时，
   语义位置不会跟着修正，§41 未满足。
3. **外参是标称值且无旋转**：`configs/go2w/official_reference.yaml::camera_extrinsics_available=false`；
   且已取证 `d435_color_optical_frame` 在 TF 树上**没有父节点**。

**本轮修复（2026-09-08 19:10）**

1. `scripts/go2w/generate_plain_slam_pandar_config.py::build_slam_runtime()` **漏设 `odom_frame`**，
   导致 `slam_3d_node` 用上游默认 `odom` 广播 `pslam_map→odom`，而 `odom_base` 的 frame 是
   `pslam_odom` —— 两条链对不上，`T_map_base` 永远拼不出来（探针实测
   `tf_frames_seen = {"pslam_map->odom": 160}`、`map_to_odom_samples=0`）。
   补上 `"odom_frame": frames.get("odom")` 后 TF 树变为 `pslam_map → pslam_odom → pslam_imu`。
2. 新增 `app/slam/slam_bridge.py`（计划书 §5.1）：带时间戳缓冲的 `map→odom` + `odom→base` 合成，
   `camera_point_to_map(xyz_camera, t_image)` 按**图像拍摄时刻**取位姿，禁止 `now()`。
3. 真机探针 `scripts/go2w/slam_bridge_probe.py`（只读，不发运动）验证：

```json
{"tf_frames_seen": {"pslam_odom->pslam_imu": 161, "pslam_map->pslam_odom": 160},
 "map_to_odom_samples": 160, "odom_to_base_samples": 160,
 "robot_pose_map": [-0.2818, 0.0142, 0.0007],
 "test_point_in_map": [1.8182, 0.0158, 0.3]}
```

即 `camera(0,0,2)` 已能投到 `pslam_map` 系。

**接入已完成（2026-09-08 19:25）**：

- `PlainSlamSpatialProvider` 新增 `/tf` 订阅与
  `camera_point_to_map(xyz_camera, timestamp)`（经 SLAMBridge 合成 `T_map_base`，输出 `pslam_map` 系；
  `map→odom` 未就绪时返回 `None`，**不退回 odom 系冒充 map 系**）。
- `scripts/go2w/run_semantic_exploration.py` 的 Phase 2（写 `map_xyz` 到 localized 观测）
  改为**优先**调用 `camera_point_to_map(xyz, observation.timestamp)`，失败才回退旧的
  `camera_point_to_spatial`（odom 系 + 标称外参），并把实际 `map_frame` 写进 provenance。
- 回归：`test_plain_slam_spatial_provider` / `test_plain_slam_frame_contract` /
  `test_slam_bridge` / `test_map_quality_evaluator` / `test_plain_slam_webui_3d`
  = **46 passed**；`python3 -m compileall app scripts/go2w` 通过。
- 已部署到机器狗，远端 `compileall` = `REMOTE_SYNTAX_OK`。
- **真机闭环验证通过（2026-09-08 19:34）**：用 provider 的真实路径（`enable_ros=True` +
  `spin_once()` 消费 `/tf` 与 `/go2w/slam/odom_base`）实测：

```text
pose      = (-0.281, 0.014, 0.000, pslam_odom)
map_point = (1.8193, 0.0146, 0.3)        <- 已是 pslam_map 系
bridge    = {map_to_odom_samples: 119, odom_to_base_samples: 119}
```

- 修复过程中的一个真 bug：首版 `camera_point_to_map()` 用 `pose.timestamp` 喂 SLAMBridge，
  而 `SpatialPose` 没有可用的 ROS 时间戳（为 0）→ `odom_to_base_samples=0` → 投影返回 `None`。
  已改为在 `_on_odometry` 回调里用**消息的 ROS 时间戳**喂 bridge（与 `/tf` 同基）。
- **会话级取证：部分完成（2026-09-08 18:55）**。新代码下的 dry-run 会话
  `search_20260908_185545_a491be85` 产出了 4 个 `semantic_object_localized` 事件并带 `map_xyz`
  （如 `[0.5454, 2.2683, 0.2031]`）。但该会话发生时 `map_frame` 字段尚未落盘，
  **无法区分**它来自 `pslam_map`（新路径）还是 `pslam_odom`（旧回退）——当前 `map→odom`
  接近单位变换，数值也区分不了。补跑会话 `search_20260908_185711_eb2668cc` 时现场处于
  **`RGB_ONLY`（深度无效）**，没有 localized 对象，因此 `map_frame` 取证**仍未完成**。
  已补上该字段（`semantic_object_localized.map_frame`），下一轮在深度可用时重跑即可定论。

因此 **object_position_rms / duplicate_rate / false_split / false_merge 全部无法给出数值**，
不得写 ACCEPTED。

---

## 6. 单元测试与回归

| 项 | 结果 |
|---|---|
| 本轮新增代码 | `app/mapping/map_quality_evaluator.py`（§17/§30 评估器）+ `tests/test_map_quality_evaluator.py` |
| 本地 pytest（新增测试） | **7 passed in 0.23s** |
| 本地 CLI smoke | `python3 -m app.mapping.map_quality_evaluator --map ... --trajectory ... --objects ... --output metrics.json` 输出完整 metrics（map/trajectory/semantic/dedup） |
| 机器狗 Python 3.8 回归 | **7 passed in 0.95s**（机器狗 `.runtime_venv`：Python 3.8.10 + scipy 1.3.3 + numpy 1.17.4；评估器与测试已部署） |

**本轮相关回归**：`tests/test_slam_bridge.py` + `tests/test_map_quality_evaluator.py` +
`tests/test_plain_slam_config_generation.py` = **18 passed, 1 failed**。

失败项 `test_plain_slam_config_generation.py::test_lio_runtime_params_extrinsics_and_tuning`
断言 `acc_scale == 1.0`，而配置已在 2026-09-03 被实测修正为 `0.95400`
（见 `configs/go2w/plain_slam_lio_params.yaml` 注释：静止重力模长 10.2795 m/s²，偏高 4.82%）。
**这是预先存在的 baseline 失败，与本轮改动无关**；按计划书 §44 Phase 0 记录，
**不为了让测试变绿去改测试或改回参数**。

**说明**：本轮**没有修改**任何既有的 `.py`/`.cpp`（根因修复是重启映射栈，不是改代码），
因此既有回归的基线未变。真正需要回归的是下一步的输入层修复与语义链改造。

---

## 7. 综合判定

```text
map.surface_thickness_p95_m     : A 0.0654 / B 0.0558                           PASS
map.overlap_rmse_m              : B 0.0658 PASS；A 0.1333 FAIL（旋转漂移）       部分 PASS
map.endpoint_translation_error  : A 0.1935（边界）/ B 0.0722                     PASS
map.endpoint_yaw_error          : A 1.2484° / B 1.0360°                         PASS
map.rotation_xy_drift_m         : episode 13 0.085 / episode 12 0.154            边界 PASS + 1 FAIL
loop.min_loop_closures          : 未测（无闭环数据集）                            NOT VERIFIED
semantic.* / dedup.*            : 未测（无 map 坐标系投影 / 无多视角数据）         NOT VERIFIED
```

**Final result: NOT ACCEPTED**（计划书 §50：`semantic` / `dedup` / `loop` 无量化数据，
且 `map.overlap_rmse` 缺失；`map` 的厚度/端点三项已实测 PASS，但不足以写 ACCEPTED）

但本轮取得了比"通过某个阈值"更关键的成果：

> **重影的直接根因被定位并修复**：不是参数、不是算法选型，而是
> `/utlidar/imu` 时钟跳变后 `imu_fallback_adapter` fail-closed 永久阻断 IMU，
> 导致 LIO 无 IMU 发散、pose-graph 后端崩溃、权威地图消失。
> 修复后 LIO 旋转跟随误差从 32.4° 降到 0.1°，地图 extent 从 55×74×20 m 收敛到 22.6×9.4×3.9 m。

---

## 8. 下一步（按计划书 §26 失败分类顺序）

1. **消除旋转残差**（当前 0.085–0.154 m / 30°）：
   先查 Pandar 外参（`pandar_extrinsics_validated=false`）与 LiDAR-IMU skew，
   再考虑 `accumulation_cycle` 与 keyframe 内部拼接。
2. **实现 `T_map_camera(t_image)`**：补 TF 链（`pslam_map → … → camera_optical`），
   用消息时间戳而非 `now()`，并把 `SemanticObjectMap` 的位置从 camera-local 提升到 map-frame。
3. **落盘 Dataset A/B/D**：评估器已交付（`app/mapping/map_quality_evaluator.py`，
   7 个单元测试通过、CLI 可跑）；缺的是**录制脚本与数据集本身**
   （`scripts/record_slam_validation.sh` 待补），因此 overlap RMSE / surface thickness /
   endpoint error / object spread / duplicate 指标仍无真机数值。
4. **回环验证**：需要一段有重叠的往返或闭环数据；在用户"小幅度移动"约束下，
   可用多次往返的重叠子图近似，并如实标注为近似。
5. **运维硬约束**：机器狗每次重启后必须先重启映射栈，并检查
   `/go2w/slam/imu_status` 不是 `IMU_SOURCE_RESTART_REQUIRED`。

---

## 2026-09-09 17:40 — 受控真机 Dataset Bcontrolled2

- 数据集：`artifacts/slam_validation/datasetBcontrolled2_20260909_173451/`
- 运动：`f0.15,b0.15`；随后单独 `f0.05` 微调回胶带原点附近（微调不在该 bag 内）。
- 安全：运动前 readiness `ready=true`；motion/arm/estop/lidar/camera 全部通过；末端机器狗在线、静止。
- 运动验证：前进 0.10779 m、后退 0.17650 m，均 `verified=true`；微调后相对本轮起点约 0.0295 m / 0.014°。

| 指标 | 数值 | 阈值 | 结论 |
|---|---:|---:|---|
| surface_thickness_p95 | 0.05431 m | ≤0.15 m | PASS |
| endpoint_translation_error | 0.06555 m | ≤0.20 m | PASS |
| endpoint_yaw_error | 0.14652° | ≤5° | PASS |
| overlap_rmse | 0.04600 m | ≤0.10 m | PASS |
| overlap_inlier_ratio | 0.99058 | — | 记录 |
| loop_closure_count | 0 | ≥1（综合门槛） | FAIL（短直线往返不构成闭环） |

结论：新录制的小幅真机往返在地图厚度、终点误差和首末扫描重合四项均通过；不能用该短运动证明回环，回环证据继续引用已有 `datasetDloop3_20260909_114801`（44 条 loop edges）。

补充健康证据：本轮结束 `/tmp/p2_probe.py` 输出 `camera_point_to_map=(1.8103,0.0263,0.3)`、`quality=METRIC_LIDAR`、`tf_available=true`；readiness 为 `ready=true`。

## 2026-09-09 逐帧语义原始观测口径

worker 已新增 `semantic_frame_objects_localized` 事件，逐帧保存严格 `frame_object_id -> RGB-D localized -> persistent_object_id` 链路及 `camera_xyz/map_xyz/map_frame/image_timestamp`。评价器支持 `--object-events` 并按持久实例分组，不再按 label 合并。单实例不足两帧时 RMS/P95 为 `NOT_VERIFIED`，避免单帧到自身质心产生虚假 `0.0 PASS`。

真机 dry-run 证据：`artifacts/slam_validation/semantic_raw_observations_20260909_182058/`；3 个物体均为 `METRIC_LIDAR`、`pslam_map` 坐标且 source id 与 persistent id 完整。该会话仅一帧，故位置稳定性仍为 `NOT_VERIFIED`，等待同实例多帧数据。

### 多帧结果（2026-09-09 18:38）

`artifacts/slam_validation/semantic_raw_multiframe_20260909_183532/object_metrics.json` 使用 5 帧逐帧原始事件，4 个持久实例有重复观测：最大实例 RMS `0.02649 m`、最大实例 P95 `0.03486 m`，分别低于 `0.15/0.25 m` 阈值，均 PASS。本会话为 dry-run，机器狗未运动。

## 2026-09-09 19:25 — 小幅移动下逐帧语义稳定性

数据：`artifacts/slam_validation/semantic_motion_rgbd_20260909_191747/`。semantic worker 以 `--rgbd-source --dry-run-motion` 采集，真机只执行直线 `f0.15,b0.15`，无旋转。前进/后退实测 `0.10170/0.17805 m`，随后 `f0.10` 实测 `0.08090 m` 微调归位；最终相对本轮起点 fused odom 残差 `0.00461 m / 0.11309°`，readiness 为 `ready=true`。

5 帧共 24 条逐帧物体观测，24/24 为 `METRIC_LIDAR / pslam_map`，6 个 persistent id 有重复观测。综合最大实例 RMS `0.19899 m`（阈值 0.15，**FAIL**），P95 `0.30949 m`（阈值 0.25，**FAIL**）。失败由办公桌 `obj_002` 主导：4 帧 RMS/P95 同为综合最大值，其中一帧 x 坐标从其余约 `1.58–1.62 m` 跳到 `1.1511 m`；其余重复实例 RMS 为 `0.00626–0.04790 m`。因此移动语义稳定性不通过，不能用静止会话 PASS 替代。

同时修正评价器：事件按 `persistent_object_id` 分组，label 只作展示名，避免同一 ID 因“桌子/办公桌”标签漂移被拆组；不会按 label 合并不同 ID。相关测试 `23 passed`。

## 2026-09-09 22:43 — 深度主峰修复后的有效移动语义验收

修复 `DepthObjectLocalizer` 对大型薄 bbox 的深度锚点：完整 bbox 5 cm 主峰用于稳定桌面层，普通 bbox 继续要求 inner 支持，保留“中心近前景不能被远背景覆盖”的约束。真实异常帧 `404975/54198/118180` 均回归到桌面主层；相关测试 `25 passed`。最终代码已部署机器狗并通过 Python 3.8 语法检查。

有效会话：`artifacts/slam_validation/semantic_motion_depthfix3_20260909_223200/`。8 个逐帧语义事件时间范围 `1719578.685909–1719810.416885`，受控真机运动范围 `1719694.399056–1719702.794384`，因此事件明确覆盖运动前、中、后，不是静止会话冒充。运动仅为同朝向 `f0.15,b0.15`，随后 `f0.10` 归位修正，无旋转；最终相对本轮起点 fused odom 残差 `0.01454 m / 0.20321°`，readiness `ready=true`。

| 指标 | 数值 | 阈值 | 结论 |
|---|---:|---:|---|
| object_position_rms_m | **0.06717 m** | ≤0.15 m | PASS |
| object_position_p95_m | **0.08680 m** | ≤0.25 m | PASS |
| repeated_object_count | 6 | — | 记录 |

本轮移动语义位置稳定性通过。`duplicate_rate / false_split / false_merge` 仍缺现场真实实例真值，继续 `NOT_VERIFIED`；不得由本表推导去重通过。Pandar 人工外参、完整旋转包络仍未验证，`rotation_clearance_valid=false`，继续禁止旋转。

## 2026-09-09 23:15：评估阈值补全与机器狗再次失联

- `map_quality_evaluator` 已补齐计划 §23 中 overlap P95/inlier、post-loop jitter、dedup 三类验收检查；缺输入时明确 `NOT_VERIFIED`。
- 回归：相关 4 文件共 `35 passed`，编译/shell/diff 检查通过。
- 旧 `datasetDloop3` 重评见 `artifacts/slam_validation/datasetD_20260909/dloop3_metrics_v2.json`：loop=44 和 surface thickness PASS；该轨迹未回起点，因此不能作为完整 endpoint/overlap 验收；post-loop jitter、dedup 保持 `NOT_VERIFIED`。
- 本轮未发运动。机器狗由在线转为 ping 0/3、SSH 超时，已按用户要求立即停工；详细无缝交接见合并计划书末尾。
