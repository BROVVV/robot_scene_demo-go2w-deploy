# GO2W 稳定三维 SLAM + 语义物体标注 MVP 重构实施计划

> **目的**：直接交给 Coding Agent / AI 执行。  
> **项目**：`BROVVV/robot_scene_demo-go2w-deploy`  
> **目标**：删除/停用当前失败的自研三维建图累积模块，先实现一个稳定、可验证的三维 SLAM 地图，并把 D435 视觉识别到的静态物体稳定标注到 `map` 坐标系中。  
> **本次范围严格限制为 MVP**：  
> 1. 稳定 3D SLAM；  
> 2. 稳定全局点云地图；  
> 3. RGB/RGB-D 目标识别结果投影到三维地图；  
> 4. 多帧/多视角同一物体融合与去重；  
> 5. 自动化地图质量、回环质量、语义标注质量验证。  
>
> **本次明确不做**：复杂 Scene Graph、房间理解、CLIP 地图、神经隐式地图、视觉-LiDAR 紧耦合 SLAM、LLM 地图推理、多机器人地图融合。

---

# 0. 给执行 AI 的总指令

执行本计划时必须满足以下要求。

1. **先检查当前本地工作区，而不是只看 GitHub。**
   - 用户已经对项目做过尚未上传 GitHub 的修改。
   - 必须以当前本地代码为真实基线。
   - 执行前记录：
     - 当前分支；
     - `git status`;
     - 当前 HEAD；
     - 未提交文件；
     - 与当前 3D 建图、SLAM、RGB-D 物体定位有关的文件列表。

2. **不得覆盖或回滚用户尚未提交的其它功能改动。**
   - 只修改本计划涉及的 SLAM、3D map、semantic map、配置、测试、可视化相关代码。
   - 对无关的自主搜索/目标接近改动保持原样。

3. **先找出旧三维地图的真实数据链，再删除。**
   - 不允许凭文件名猜测。
   - 必须从最终地图发布 topic / API 反向追踪：
     - 谁生成 pose；
     - 谁 transform point cloud；
     - 谁累计历史点云；
     - 谁发布 global 3D map；
     - 谁消费该地图。
   - 确认替代链路能工作后，再删除或 disable 旧实现。

4. **不允许再使用“每帧点云 transform 后永久 append 到全局数组”的地图逻辑。**
   - 这是本次整改的核心禁区。
   - 全局地图必须来自：
     - SLAM 自己维护的全局地图；
     - 或基于 keyframe + 最新优化 pose 重建。
   - 回环优化后，历史地图必须能随优化后的历史 pose 一起更新。

5. **第一版 SLAM 不做视觉紧耦合。**
   - 定位/几何建图主干采用 **LiDAR + IMU SLAM**。
   - D435 只负责：
     - RGB 目标检测；
     - Depth 目标三维定位；
     - 语义物体标注。
   - 这样最大限度降低复杂度。

6. **默认目标 SLAM 后端：优先 LIO-SAM。**
   - 原因：需要稳定回环、pose graph 和全局一致性。
   - 如果当前 ROS/驱动/点云字段/IMU 类型与 LIO-SAM 无法合理兼容，允许使用：
     - FAST-LIO2 + 已验证的 loop closure / Scan Context；
   - 但执行 AI 必须在实施报告中明确说明为什么没有使用 LIO-SAM。
   - **禁止**使用没有回环能力的纯累积 LIO 作为最终“全局地图完成方案”。

7. **不允许使用“看起来不错”作为测试结论。**
   必须实现量化评估工具，对以下四项给出数值：
   - 地图重影；
   - 回环稳定；
   - 物体标注稳定；
   - 同一物体重复实例数量。

8. **Coding Agent 不能在测试失败时直接结束。**
   必须按本文“自动修复循环”继续排查和修改，直到：
   - 所有自动测试通过；
   - 所有离线回放验收指标通过；
   - 无新的相关回归失败。
   如果因为缺少真实传感器数据/标定文件导致无法证明某项真机指标，必须明确报告“未验证”，不能谎称完成。

9. **最终完成标准**：
   - 不是“代码能启动”；
   - 而是“在定义好的测试数据集上四类指标全部通过”。

---

# 1. 最终目标效果

完成后系统数据流应尽量简单：

```text
Pandar XT16 + IMU
        │
        ▼
  LiDAR-Inertial SLAM
        │
        ├──────────────► T_map_base
        │
        ├──────────────► optimized keyframe poses
        │
        └──────────────► stable global 3D map
                              │
                              │
D435 RGB ──► Object Detector  │
                              │
D435 Depth ─► DepthObjectLocalizer
                              │
                              ▼
                       object camera_xyz
                              │
                              ▼
                      T_map_camera(t_image)
                              │
                              ▼
                       object map_xyz
                              │
                              ▼
                      SemanticMapManager
                              │
                     ┌────────┴─────────┐
                     ▼                  ▼
              semantic_objects     MarkerArray
                                        │
                                        ▼
                                  RViz / WebUI
```

最终看到：

- 一张稳定三维点云地图；
- 机器人走一圈回来后墙体不产生明显“双层”；
- 同一个固定物体从不同角度看到，地图里位置基本不漂；
- 同一个椅子/垃圾桶不会生成几十个实例；
- 回环闭合后，语义物体仍然和优化后的地图对齐。

---

# 2. 本次模块范围

## 2.1 保留

优先复用：

- Pandar 驱动；
- IMU 数据源；
- D435 RGB / Depth；
- 现有视觉 detector；
- 现有 `DepthObjectLocalizer`；
- 现有 TF / frame 配置中正确的部分；
- 现有 WebUI / event 机制；
- 现有自主搜索模块，只改变它读取 3D map / semantic object 的接口来源。

## 2.2 删除或停用

删除/disable 当前失败的：

- 旧全局 3D point cloud accumulator；
- 旧手工 world-map fusion；
- 旧“收到新 scan 就 transform + append”的永久累计；
- 与新 SLAM authoritative map 冲突的第二套地图发布器；
- 旧的伪全局 object pose 累积逻辑；
- 任何同时发布同名 `/map_3d` 且数据来源不是 authoritative SLAM 的节点。

## 2.3 新增的核心模块最多四个

建议：

```text
app/slam/slam_bridge.py
app/mapping/semantic_map.py
app/mapping/semantic_map_visualizer.py
app/mapping/map_quality_evaluator.py
```

如果现有目录结构不同，可以按仓库风格调整，但不要拆出十几个小模块。

---

# 3. 第一阶段：审计当前地图重影根因

执行 AI 必须先完成审计报告，写入：

```text
docs/SLAM_MIGRATION_AUDIT.md
```

至少回答以下问题。

## 3.1 现有地图到底是谁生成的

列出：

```text
input topic
→ processing node/module
→ pose source
→ transform
→ accumulation
→ output topic
```

必须画清楚。

## 3.2 检查是否存在下面的反模式

全仓搜索：

```text
append
concatenate
vstack
hstack
global_cloud
world_cloud
accumulate
map_points
point_history
transform_points
/map_3d
PointCloud2
```

检查是否存在：

```python
global_points.extend(T_world_sensor @ current_scan)
```

但历史点永远不随 loop closure 更新。

如果有，必须在审计里明确标记：

```text
ROOT_CAUSE_CANDIDATE: irreversible scan accumulation
```

## 3.3 TF 审计

必须输出完整 frame tree：

```text
map
└── odom
    └── base_link
        ├── lidar
        ├── imu
        └── camera
```

检查：

- frame id 是否重复；
- 是否有多个节点同时发布同一 transform；
- static 与 dynamic TF 是否冲突；
- sensor extrinsic 是否被重复应用；
- map/odom 是否反了；
- LiDAR 点云是否已经在 base frame 又二次转换。

## 3.4 时间戳审计

检查：

- LiDAR timestamp；
- IMU timestamp；
- RGB timestamp；
- Depth timestamp；
- detection timestamp；
- TF lookup 是否使用消息时间，而不是 `now()`；
- RGB 与 Depth 是否来自同一/近似同步帧。

记录最大时间差。

---

# 4. 选择并接入稳定 SLAM 后端

## 4.1 默认使用 LIO-SAM

优先适配 LIO-SAM：

输入：

```text
Pandar PointCloud2
IMU
```

输出至少提供：

```text
T_map_base / T_map_lidar
optimized trajectory
keyframe cloud / global map
loop closure status
```

如果使用 LIO-SAM，必须确认：

- Pandar 点云字段满足 deskew 所需字段；
- ring / time 字段映射正确；
- IMU angular velocity / acceleration 单位正确；
- IMU orientation 若不可用则正确配置；
- extrinsicRot / extrinsicRPY / extrinsicTrans 与真实安装一致；
- gravity 方向正确；
- point cloud timestamp 与 IMU 同一时基。

## 4.2 允许的 fallback

若 LIO-SAM 与当前环境客观不兼容：

```text
FAST-LIO2
+
Scan Context / pose graph loop closure
```

但必须满足：

- 有全局 loop closure；
- 优化后 keyframe pose 可更新；
- global map 可由优化后的 keyframe pose 重建；
- 不能只是 FAST-LIO2 的无回环地图长期累积。

---

# 5. SLAM authoritative source 统一

整改后只允许一个“权威世界坐标”。

配置：

```yaml
slam:
  map_frame: map
  odom_frame: odom
  base_frame: base_link
  lidar_frame: <真实 lidar frame>
  imu_frame: <真实 imu frame>
  camera_frame: <真实 camera optical frame>
```

## 5.1 只允许一个 authoritative pose

新增 `SLAMBridge`：

```python
class SLAMBridge:
    def get_robot_pose(timestamp=None): ...
    def get_camera_pose(timestamp): ...
    def get_global_map(): ...
    def get_slam_quality(): ...
```

所有上层模块统一通过该接口或统一 ROS topic 获取。

## 5.2 禁止上层重新积分 odom 来产生第二份 global pose

语义地图、自主搜索、WebUI 都不应自己维护新的 `map` frame。

---

# 6. Global 3D Map 的正确构建方式

## 6.1 首选

如果 SLAM 后端直接提供经过回环优化的全局点云：

```text
直接使用 SLAM global map
```

并进行：

- voxel downsample；
- range filter；
- optional outlier removal。

不要再次累计。

## 6.2 如果 SLAM 只提供 keyframe

维护：

```python
@dataclass
class SlamKeyframe:
    id: int
    timestamp: float
    cloud_sensor_frame: np.ndarray
    optimized_pose_map_sensor: np.ndarray
```

重建：

```python
def rebuild_global_map(keyframes):
    clouds = []
    for kf in keyframes:
        clouds.append(transform(kf.cloud_sensor_frame, kf.optimized_pose_map_sensor))
    return voxel_downsample(concat(clouds))
```

每次发生 loop closure / pose graph update：

```text
mark global map dirty
→ rebuild or incrementally rebuild affected submaps
```

## 6.3 禁止

```text
old global cloud
+
new optimized cloud
```

这种混合更新。

回环前错误位置上的旧点必须被替换，而不是保留。

---

# 7. 地图发布

统一输出：

```text
/go2w/slam/map_3d
```

或复用项目已有 authoritative topic，但只能一个发布者。

消息：

```text
sensor_msgs/PointCloud2
frame_id = map
```

推荐同时输出：

```text
/go2w/slam/trajectory
/go2w/slam/status
/go2w/slam/loop_closure
```

`status` 至少包含：

```json
{
  "initialized": true,
  "imu_ready": true,
  "map_points": 123456,
  "keyframes": 312,
  "loop_closure_count": 2,
  "last_loop_closure_sec": 3.2
}
```

---

# 8. 语义物体标注：只做最轻量版本

不做 scene graph。

每个物体只维护：

```python
@dataclass
class SemanticMapObject:
    object_id: str
    label: str

    position_map: tuple[float, float, float]
    confidence: float

    observation_count: int
    first_seen_timestamp: float
    last_seen_timestamp: float

    position_covariance_or_variance: tuple[float, float, float] | None

    status: str = "ACTIVE"
```

可选保存：

```python
last_bbox
last_depth_m
source_frame_id
```

第一版不用 embedding。

---

# 9. 视觉检测到 3D 物体坐标

必须复用现有 `DepthObjectLocalizer`。

数据链：

```text
detector bbox/mask
+
registered depth
+
camera intrinsics
→
camera_xyz
```

深度估计不要只取 bbox 单像素。

优先：

```text
bbox 中央有效区域
→ 剔除 0 / NaN / 超范围
→ percentile trim
→ median
```

如果已有实现更稳，直接复用。

---

# 10. 最关键：使用“图像拍摄时间”的 TF

物体 map 坐标：

```text
P_map =
T_map_camera(t_image)
*
P_camera
```

必须：

```python
lookup_transform(
    target_frame="map",
    source_frame=camera_frame,
    time=image_timestamp,
)
```

禁止：

```python
time=now()
```

也禁止在检测结果晚 0.5s 后，用当前 robot pose 投影过去。

---

# 11. 语义观测不能只保存最终 map xyz

为了让 loop closure 后物体跟地图一起调整，必须保存“原始观测锚点”。

建议：

```python
@dataclass
class SemanticObservation:
    observation_id: str
    object_label: str

    timestamp: float
    camera_xyz: tuple[float, float, float]
    confidence: float

    keyframe_id: int | None
    camera_pose_at_observation: Any

    map_xyz_at_insert: tuple[float, float, float]
```

最佳方式：

```text
每个 semantic observation 绑定最近 SLAM keyframe
```

loop closure 后：

```text
keyframe pose changed
→ recompute observation map xyz
→ refit SemanticMapObject fused position
```

如果当前 SLAM bridge 暂时拿不到 keyframe ID，第一版允许保存：

```text
timestamp
+
camera_xyz
```

在 loop closure 完成后重新查询 `T_map_camera(timestamp)` 并重算。

---

# 12. 同一物体去重：轻量版本

不使用 CLIP embedding。

采用：

```text
label + spatial distance + temporal continuity + optional geometry
```

## 12.1 新观测关联

对于新观测 `obs`：

先找：

```python
candidate_objects = [
    obj for obj in objects
    if obj.label == obs.label
]
```

计算：

```python
distance = ||obj.position_map - obs.position_map||
```

基本 gate：

```yaml
semantic_map:
  merge_radius_m: 0.45
  hard_max_merge_radius_m: 0.70
```

第一版：

```text
同 label
+
distance <= merge_radius
→ 优先关联现有实例
```

但必须防止两把并排椅子被合并。

## 12.2 防过度合并

如果同类目标密集：

使用：

- 当前观测方向；
- 上一帧关联；
- 3D 距离；
- bbox tracking continuity；
- 如果 detector 已有 track id，则优先 track id。

规则建议：

```text
连续帧：
  优先沿用 track association

跨较长时间重访：
  使用 3D spatial nearest neighbor

如果最近两个同类 object 与新观测距离都接近：
  不立即 merge
  创建 AMBIGUOUS observation
  等下一次观测再决定
```

不要为了“无重复”把真正的两把椅子错误合成一把。

---

# 13. Object position fusion

同一实例多次观测后，不用简单覆盖。

使用 confidence/depth-quality 加权 EMA：

```python
w_new = clamp(confidence * depth_quality, min_w, max_w)

p_fused = (
    w_old * p_old + w_new * p_obs
) / (w_old + w_new)
```

或维护所有有效 observation，在 loop closure 后重新 robust mean。

建议用 robust median / trimmed mean，避免一帧深度异常把实例拖走。

---

# 14. 物体确认策略

为了 MVP 简化：

- 静态环境假设；
- 不自动删除 object；
- 低 confidence 单帧 detection 不进入 persistent semantic map。

建议：

```yaml
semantic_map:
  min_detection_confidence: 0.55
  min_confirm_observations: 2
```

状态：

```text
CANDIDATE
→ 2 次以上一致观测
→ CONFIRMED
```

RViz 默认只显示 `CONFIRMED`。

这会显著减少误检和重复标签。

---

# 15. 语义地图输出

输出：

```text
/go2w/semantic_map/objects
/go2w/semantic_map/markers
```

objects 可以是项目自己的 JSON/schema。

MarkerArray：

每个 confirmed object：

1. `SPHERE` 或 `CUBE` 标位置；
2. `TEXT_VIEW_FACING` 显示：

```text
chair_003
conf=0.91
obs=12
```

---

# 16. RViz / WebUI 最低展示

至少可以同时看：

- SLAM 3D point cloud；
- robot trajectory；
- semantic object marker；
- object labels。

不要求本次做截图级 SuperMap UI。

---

# 17. 地图重影自动评估器

新增：

```text
app/mapping/map_quality_evaluator.py
```

以及 CLI：

```bash
python -m app.mapping.map_quality_evaluator   --bag <bag_path>   --output <report.json>
```

必须生成机器可读 JSON。

---

# 18. 测试数据集

执行 AI 必须建立四类 replay 数据。

优先使用现有 rosbag / recorded session。

如果仓库没有：

创建录制脚本：

```text
scripts/record_slam_validation.sh
```

并在真机一次性录制以下数据。

## Dataset A：原地 360°

```text
机器人原地缓慢转 360°
```

目的：

- 测假平移；
- 测旋转后地图墙面重影。

## Dataset B：A→B→A

```text
直线/走廊出去
→ 返回原点
```

目的：

- 短程漂移；
- 起终点重合。

## Dataset C：闭环

```text
A→B→C→D→A
```

至少有一个明显 loop closure。

目的：

- 验证 pose graph；
- 验证地图优化后是否重叠。

## Dataset D：固定物体多视角

场景中固定：

```text
chair / trash_can / monitor 等
```

同一个物体至少从：

```text
5 个不同 viewpoint
```

观察。

最好再放两个相同类别但位置不同的物体，用于防止过度合并。

---

# 19. 指标 1：地图不重影

“地图不重影”必须量化。

## 19.1 开始/结束子图重合

对于 Dataset B/C：

取：

```text
start submap
end submap
```

loop closure 后统一到 map frame。

使用 nearest-neighbor / ICP evaluation：

```text
overlap_rmse_m
overlap_p95_m
inlier_ratio
```

建议第一版验收：

```yaml
map_quality:
  overlap_rmse_m_max: 0.10
  overlap_p95_m_max: 0.18
  inlier_ratio_min: 0.75
```

## 19.2 墙面/静态表面厚度

对平面区域拟合局部 plane。

统计点到拟合平面的双侧厚度：

```text
surface_thickness_p95
```

建议：

```text
P95 <= 0.15m
```

如果出现两层墙，该指标应明显失败。

## 19.3 原地旋转假平移

Dataset A：

360° 结束后：

```text
horizontal displacement <= 0.15m
```

如果原地转圈出现数米 translation，直接失败。

---

# 20. 指标 2：回环稳定

Dataset C 必须检测真实 loop closure。

## 20.1 回环次数

```text
loop_closure_count >= 1
```

否则 Dataset C 直接不合格。

## 20.2 回到起点后的位姿误差

若有起点 ground truth：

```text
start/end translation error <= 0.20m
start/end yaw error <= 5°
```

如果无 ground truth：

使用起始局部点云和结束局部点云 ICP 估计闭环误差。

## 20.3 回环后不能持续跳变

在 loop closure 发生后 3–5 秒：

```text
post_loop_translation_jitter_p95 <= 0.08m
post_loop_yaw_jitter_p95 <= 2°
```

## 20.4 回环优化必须让地图变好

保存：

```text
pre_loop_overlap_rmse
post_loop_overlap_rmse
```

要求：

```text
post_loop_overlap_rmse < pre_loop_overlap_rmse
```

通常改善比例目标：

```text
>= 20%
```

如果优化前已经满足很严格的绝对阈值，可以豁免改善比例，但最终绝对阈值必须通过。

---

# 21. 指标 3：物体标注稳定

Dataset D：

同一真实物体至少 5 个视角。

每次有效观测保留：

```text
map_xyz
```

## 21.1 位置稳定性

统计：

```text
centroid_rms_error
centroid_p95_error
axis_stddev
```

第一版建议：

```yaml
semantic_quality:
  object_position_rms_m_max: 0.15
  object_position_p95_m_max: 0.25
```

## 21.2 回环前后标签跟图一致

保存 loop closure 前：

- object position；
- 附近 local surface position。

loop closure 后重算。

要求：

```text
relative_object_surface_shift <= 0.15m
```

否则说明地图变了但标签没跟着变。

## 21.3 多次回放稳定

同一 bag replay 3 次：

要求：

```text
object_count identical
centroid run-to-run difference <= 0.05m
```

---

# 22. 指标 4：同一物体不能重复几十次

Dataset D 给每个真实物体人工标一个 GT 或 expected count。

## 22.1 Duplicate Rate

定义：

```text
duplicate_rate =
(max(0, predicted_instances - expected_instances))
/
expected_instances
```

固定测试场景验收：

```text
duplicate_rate = 0
```

## 22.2 Observation-to-instance ratio

同一 chair 被检测 30 次：

允许：

```text
observations = 30
instances = 1
```

不允许：

```text
instances = 30
```

要求：

```text
all observations of same static object
→ exactly one CONFIRMED SemanticMapObject
```

## 22.3 防止反向错误

额外放：

```text
chair_A
chair_B
```

两者相距 > 1m。

必须输出：

```text
2 个 chair instances
```

最终：

```text
false_split = 0
false_merge = 0
```

---

# 23. 综合验收阈值

第一版默认阈值：

```yaml
acceptance:
  map:
    rotation_xy_drift_m_max: 0.15
    overlap_rmse_m_max: 0.10
    overlap_p95_m_max: 0.18
    surface_thickness_p95_m_max: 0.15
    overlap_inlier_ratio_min: 0.75

  loop:
    min_loop_closures: 1
    endpoint_translation_error_m_max: 0.20
    endpoint_yaw_error_deg_max: 5.0
    post_loop_translation_jitter_m_max: 0.08
    post_loop_yaw_jitter_deg_max: 2.0

  semantic:
    object_position_rms_m_max: 0.15
    object_position_p95_m_max: 0.25
    object_surface_relative_shift_m_max: 0.15

  dedup:
    duplicate_rate_max: 0.0
    false_split_max: 0
    false_merge_max: 0
```

这些是 MVP 推荐值。若硬件数据证明某项不现实，可以提出调整，但必须在最终报告中记录原因与实测数据，不能为过测试任意放宽。

---

# 24. 自动验证报告

每次 replay 后必须生成：

```text
artifacts/slam_validation/<run_id>/
```

至少：

```text
metrics.json
summary.md
trajectory.csv
objects.json
map_final.pcd 或 ply
map_preview.png
semantic_map_preview.png
```

`metrics.json` 示例：

```json
{
  "map": {
    "rotation_xy_drift_m": 0.06,
    "overlap_rmse_m": 0.07,
    "overlap_p95_m": 0.13,
    "surface_thickness_p95_m": 0.10
  },
  "loop": {
    "loop_closure_count": 2,
    "endpoint_translation_error_m": 0.11,
    "endpoint_yaw_error_deg": 2.4
  },
  "semantic": {
    "object_position_rms_m": 0.09,
    "object_position_p95_m": 0.16
  },
  "dedup": {
    "duplicate_rate": 0.0,
    "false_split": 0,
    "false_merge": 0
  },
  "accepted": true
}
```

---

# 25. 自动“失败就继续改”的迭代规则

Coding Agent 必须按以下逻辑工作：

```text
baseline
↓
implement
↓
run unit tests
↓
run replay Dataset A-D
↓
compute metrics
↓
all pass?
  ├─ yes -> full regression -> finish
  └─ no
       ↓
       classify failure
       ↓
       modify targeted subsystem
       ↓
       rerun failed dataset
       ↓
       rerun all A-D
       ↓
       repeat
```

不得：

```text
失败一次
→ 写“建议现场再调”
→ 结束
```

只要有足够测试数据，就必须继续迭代。

---

# 26. 失败分类与修复顺序

## 26.1 地图重影失败

优先排查：

1. LiDAR/IMU 外参；
2. LiDAR timestamp；
3. IMU timestamp；
4. ring/time 字段；
5. deskew；
6. map/odom/base TF 冲突；
7. 重复应用 extrinsic；
8. global map 是否仍在不可逆 append；
9. loop closure pose 更新后是否真正重建 map。

## 26.2 原地旋转漂移

优先：

1. IMU gravity；
2. IMU axis；
3. extrinsic rotation；
4. gyro units；
5. accelerometer units；
6. deskew；
7. LiDAR timestamp offset。

## 26.3 回环不触发

优先：

1. loop closure enabled；
2. descriptor；
3. candidate search radius；
4. loop frequency；
5. keyframe density；
6. ICP fitness threshold；
7. 环境重叠是否足够。

## 26.4 回环后地图更糟

优先：

1. false loop；
2. loop registration fitness；
3. pose graph noise；
4. wrong frame；
5. duplicate transform；
6. map rebuild bug。

## 26.5 物体位置漂

优先：

1. RGB/Depth 同步；
2. detection timestamp；
3. `T_map_camera(t)` 是否拿错时间；
4. camera extrinsic；
5. depth scale；
6. bbox depth sampling；
7. loop closure 后是否重投影 semantic observations。

## 26.6 同物体重复

优先：

1. merge radius 太小；
2. object map xyz 因 TF 错误抖动；
3. detector label 不稳定；
4. 每帧都创建 object；
5. 没有 candidate→confirmed；
6. loop closure 后旧 object 未重关联。

不要先把 merge radius 暴力调到 2m。

## 26.7 两个同类真实物体被合并

优先：

- 减小 merge radius；
- 加 temporal tracking；
- 加 3D separation；
- ambiguous association 延迟确认。

---

# 27. 单元测试清单

必须新增或扩展。

## 27.1 SLAM Bridge

```text
tests/test_slam_bridge.py
```

覆盖：

- authoritative map frame；
- timestamp pose lookup；
- map provider 不二次累计；
- loop update 后 pose 可变化。

## 27.2 Keyframe Map Rebuild

如果使用自建 keyframe map：

```text
tests/test_keyframe_map_rebuild.py
```

模拟：

```text
KF1 pose old=(0,0)
KF2 pose old=(1.2,0)
KF2 optimized=(1.0,0)
```

要求：

- final map 只有 optimized 位置；
- old 1.2m 位置旧点不残留。

## 27.3 Timestamp TF

```text
tests/test_semantic_timestamp_transform.py
```

模拟：

```text
t0 x=0
t1 x=1m
```

图片拍摄于 t0，detector 在 t1 返回。

要求：

- object 使用 t0 TF；
- 不得使用 t1 当前 pose。

## 27.4 Semantic Object Fusion

```text
tests/test_semantic_map_fusion.py
```

同一 chair 多次观测：

```text
[2.00, 1.00]
[2.05, 1.03]
[1.95, 0.98]
```

要求最终 1 instance。

## 27.5 Two Same-Class Objects

```text
tests/test_semantic_map_same_class_instances.py
```

chair_A `[2,1]`，chair_B `[4,1]`。

要求 2 instances。

## 27.6 Loop Semantic Reprojection

```text
tests/test_semantic_loop_reprojection.py
```

模拟：

```text
loop 前 keyframe pose = x=3.3
loop 后 = x=3.0
```

要求 object map pose 跟着修正。

## 27.7 Candidate / Confirmed

```text
tests/test_semantic_object_confirmation.py
```

- 单帧误检不 confirmed；
- 两/三次一致观测进入 confirmed。

---

# 28. 离线 replay 集成测试

建议：

```text
tests/integration/test_slam_replay_rotation.py
tests/integration/test_slam_replay_return.py
tests/integration/test_slam_replay_loop.py
tests/integration/test_semantic_replay_multiview.py
```

如果 CI 不适合跑大型 bag：

- 标记 `@pytest.mark.slam_replay`;
- 提供统一脚本；
- 最终交付前仍必须实际运行。

---

# 29. `run_slam_validation.sh`

新增：

```bash
#!/usr/bin/env bash
set -euo pipefail

# 1. run rotation bag
# 2. run return bag
# 3. run loop bag
# 4. run semantic bag
# 5. call evaluator
# 6. aggregate metrics
# 7. fail with non-zero exit code if any threshold fails
```

只有所有指标通过：

```text
exit 0
```

任意失败：

```text
exit 1
```

---

# 30. 评估工具实现原则

尽量复用已有依赖。

可用：

- NumPy
- SciPy KDTree
- Open3D（仅在项目已有或部署环境接受时）
- 现有 PCL/ROS 工具

核心函数：

```python
compute_nn_overlap_metrics(...)
fit_plane_thickness(...)
compute_endpoint_error(...)
compute_object_spread(...)
compute_duplicate_metrics(...)
```

---

# 31. 外参与时间同步诊断

新增：

```bash
python -m app.slam.slam_diagnostics
```

输出：

```text
LiDAR frame
IMU frame
Camera frame

T_base_lidar
T_base_imu
T_base_camera

LiDAR age
IMU age
RGB age
Depth age

RGB-depth skew
LiDAR-IMU skew statistics
```

如果：
- frame 缺失；
- TF 冲突；
- timestamp skew 过阈值；

validation 直接 fail。

---

# 32. RGB-D 同步要求

建议：

```yaml
semantic_map:
  max_rgb_depth_skew_ms: 40
  max_detection_tf_age_ms: 100
```

如果 D435 支持 registered/aligned depth，优先使用。

---

# 33. 语义去重配置

建议初始：

```yaml
semantic_map:
  enabled: true

  min_detection_confidence: 0.55
  min_confirm_observations: 2

  merge_radius_m: 0.45
  hard_max_merge_radius_m: 0.70

  observation_history_limit: 50

  robust_position_fusion: true
  outlier_distance_m: 0.50

  max_rgb_depth_skew_ms: 40
  require_metric_depth: true

  display_candidates: false
  display_confirmed: true
```

---

# 34. SLAM 配置

具体参数按最终后端和 XT16 驱动确定，但必须显式包含：

```yaml
slam:
  backend: lio_sam

  map_frame: map
  odom_frame: odom
  base_frame: base_link
  lidar_frame: pandar
  imu_frame: imu_link

  loop_closure_enabled: true

  global_map_voxel_size_m: 0.05
  visualization_voxel_size_m: 0.08

  authoritative_map_topic: /go2w/slam/map_3d
```

---

# 35. 旧模块删除策略

不要第一步直接 `rm`。

按：

```text
1. 标记 legacy
2. 新 SLAM 在新 topic 并行 observe-only
3. replay 指标通过
4. 切 authoritative source
5. 上层依赖迁移
6. 旧模块 disable
7. full regression
8. 最后删除 dead code
```

---

# 36. 上层自主搜索兼容

本次不重做自主搜索。

只做接口切换：

```text
legacy 3D map
→
SLAMBridge / authoritative map
```

语义目标读取：

```text
SemanticMapManager.objects
```

不要在本次重新设计 frontier、LLM 或 target approach。

---

# 37. 地图重影可视化

除了数值，必须保存：

- top-down projection；
- side projection；
- start/end submap overlay；
- loop 前/后 overlay。

Coding Agent 失败时先看这些图，再调参数。

---

# 38. 物体标注稳定性可视化

保存：

```text
semantic_observation_scatter.png
```

对于每个 object 显示：

- 所有观测点；
- 融合 centroid；
- 最终 marker。

如果散点明显分两团，即使最终被 merge 成一个 instance，也视为定位链有问题。

---

# 39. 不允许用去重掩盖定位错误

如果同一个 chair 的观测分布：

```text
一团 x=2m
另一团 x=4m
```

不能把：

```text
merge_radius=3m
```

来“消除重复”。

必须先修：

- SLAM；
- TF；
- timestamp；
- depth。

---

# 40. 不允许用 voxel downsample 掩盖双地图

如果两层墙相距 0.3m：

不能设置：

```text
voxel_size = 0.5m
```

把它们糊成一层。

建议评估 voxel size：

```text
<= 0.05–0.08m
```

---

# 41. 回环后的语义更新

当收到：

```text
loop_closure
pose_graph_updated
```

SemanticMapManager 必须：

```text
reproject semantic observations
→ reassociate if needed
→ recompute fused objects
→ republish markers
```

---

# 42. Object ID 稳定性

同一个实例一旦 confirmed：

```text
chair_001
```

loop closure 后仍保持：

```text
chair_001
```

不能因坐标改变重新创建新 ID。

---

# 43. 日志要求

至少记录：

```text
SLAM_LOOP_CLOSURE
SLAM_MAP_REBUILT
SLAM_MAP_QUALITY

SEMANTIC_OBSERVATION
SEMANTIC_ASSOCIATED
SEMANTIC_NEW_CANDIDATE
SEMANTIC_CONFIRMED
SEMANTIC_REPROJECTED
SEMANTIC_MERGE_REJECTED
```

---

# 44. 测试执行顺序

## Phase 0：Baseline

```bash
git status
git diff
```

记录当前已有失败。

## Phase 1：SLAM 与 TF

只让：

```text
LiDAR + IMU + SLAM
```

工作。

通过 Dataset A/B/C。

## Phase 2：Global Map

确认：

- authoritative map；
- loop 后 rebuild；
- no ghosting。

## Phase 3：Semantic Projection

接：

- D435；
- detector；
- DepthObjectLocalizer；
- timestamp TF。

## Phase 4：Object Fusion

接：

- candidate；
- confirm；
- merge；
- loop reprojection。

## Phase 5：Validation

跑 unit + replay + full regression。

---

# 45. Coding Agent 强制自修复循环

```text
WHILE any acceptance metric fails:

    identify failing dataset
    identify failing subsystem

    if map ghosting:
        do NOT tune semantic merge
        fix SLAM / TF / timestamps / map rebuild

    if loop closure failure:
        fix loop detection / registration / graph

    if object scatter:
        fix RGB-D / TF timestamp / camera extrinsic

    if duplicate objects with small scatter:
        fix association / confirmation logic

    if two real same-class objects merged:
        tighten association / tracking continuity

    rerun the failed dataset

    if it passes:
        rerun ALL datasets A-D

END

then run full regression
```

只有：

```text
ALL ACCEPTANCE METRICS PASS
```

才允许输出“完成”。

---

# 46. 真机验收前硬门槛

如果离线 Dataset A-D 没通过：

```text
禁止进入 autonomous search 真机测试
```

先修底层地图。

---

# 47. 真机最终验收

### 场景 1：原地旋转

- 360°；
- 无明显平移；
- 墙体不分叉。

### 场景 2：走廊往返

- A→B→A；
- 起终点地图重合。

### 场景 3：完整闭环

- 绕房间/走廊一圈；
- 至少 1 次 loop closure；
- 闭环后地图变得更一致。

### 场景 4：单物体多视角

- 同一 chair 5 个角度；
- 最终 1 个 chair instance；
- marker 位置稳定。

### 场景 5：两个同类物体

- 两把 chair；
- 最终正好 2 instances；
- 不 false merge。

---

# 48. 最终验收 Checklist

## SLAM

- [ ] 旧失败 3D accumulator 已 disable/delete
- [ ] 只有一个 authoritative global map
- [ ] map/odom/base/sensors TF 树明确
- [ ] 无 TF 冲突
- [ ] 点云/IMU timestamp 正确
- [ ] global map 支持 loop 后更新
- [ ] 不存在 irreversible old-point append

## 地图不重影

- [ ] 360° XY drift <= 阈值
- [ ] overlap RMSE <= 阈值
- [ ] overlap P95 <= 阈值
- [ ] surface thickness <= 阈值
- [ ] preview 无明显双墙

## 回环

- [ ] loop closure >= 1
- [ ] endpoint translation <= 阈值
- [ ] endpoint yaw <= 阈值
- [ ] loop 后地图质量变好
- [ ] loop 后不持续 oscillate

## 语义

- [ ] RGB/Depth 同步
- [ ] 使用 image timestamp TF
- [ ] 物体 map xyz 来自 DepthObjectLocalizer + T_map_camera
- [ ] loop 后 semantic observation 重投影
- [ ] object marker 仍贴合地图

## 去重

- [ ] 同一真实物体多视角 = 1 confirmed instance
- [ ] 30 次 detection 不产生 30 个 object
- [ ] 两个同类真实物体仍保持两个实例
- [ ] false_split = 0
- [ ] false_merge = 0

## 回归

- [ ] unit tests pass
- [ ] replay A pass
- [ ] replay B pass
- [ ] replay C pass
- [ ] replay D pass
- [ ] validation script exit 0
- [ ] full pytest pass 或只有已记录 baseline unrelated failures
- [ ] compile/import checks pass

---

# 49. 最终交付文件

Coding Agent 完成后至少提交：

```text
app/slam/slam_bridge.py
app/mapping/semantic_map.py
app/mapping/semantic_map_visualizer.py
app/mapping/map_quality_evaluator.py

scripts/run_slam_validation.sh
scripts/record_slam_validation.sh

configs/.../slam.yaml
configs/.../semantic_map.yaml

docs/SLAM_MIGRATION_AUDIT.md
docs/SLAM_VALIDATION_REPORT.md
```

以及相关 tests。

---

# 50. 最终验证报告格式

`docs/SLAM_VALIDATION_REPORT.md` 必须包含：

```text
Baseline HEAD:
Working tree state:

SLAM backend:
Loop closure backend:

Dataset A:
  rotation_xy_drift:
  PASS/FAIL

Dataset B:
  overlap_rmse:
  overlap_p95:
  PASS/FAIL

Dataset C:
  loop_count:
  endpoint_error:
  post_loop_rmse:
  PASS/FAIL

Dataset D:
  object_position_rms:
  object_position_p95:
  expected_instances:
  predicted_instances:
  false_split:
  false_merge:
  PASS/FAIL

Full test:
  passed:
  failed:
  skipped:

Final result:
  ACCEPTED / NOT ACCEPTED
```

没有量化数据不能写 `ACCEPTED`。

---

# 51. 给 Coding Agent 的最终提示词

```text
你正在修改用户本地的 robot_scene_demo-go2w-deploy 工作区。
注意用户已有尚未上传 GitHub 的改动，因此以本地 working tree 为准，不要覆盖无关修改。

严格按照《GO2W 稳定三维 SLAM + 语义物体标注 MVP 重构实施计划》执行。

核心目标只有四个：
1. 地图不重影；
2. 回环稳定；
3. 物体三维标注稳定；
4. 同一物体不会被重复创建几十次，同时两个真实同类物体不能被错误合并。

实现要求：
- 删除/停用现有失败的自研 3D 点云永久累积模块；
- 使用 LiDAR + IMU SLAM 作为唯一几何/位姿真值；
- 首选 LIO-SAM；若环境不兼容，使用 FAST-LIO2 + 有效 loop closure；
- 不做视觉紧耦合 SLAM；
- D435 只负责 RGB 检测和 depth 目标定位；
- 必须使用图像 timestamp 对应的 T_map_camera；
- semantic observation 必须能在 loop closure 后重投影；
- object association 使用轻量的 label + 3D spatial gating + temporal continuity；
- 不引入 CLIP/Scene Graph/复杂 world model。

必须先建立 baseline 和数据链审计，再修改。

必须实现自动 validation evaluator 和 run_slam_validation.sh。
必须使用 Dataset A-D 对地图重影、回环、物体漂移、重复实例进行量化。

只要任何 acceptance metric 失败，就继续定位原因、修改、重跑；不能因为“看起来还行”停止。
不能用增大 voxel size 掩盖双地图。
不能用巨大 merge radius 掩盖 object pose 漂移。
不能用去重逻辑掩盖 TF/timestamp/SLAM 错误。

完成前必须：
- 跑 unit tests；
- 跑 Dataset A-D replay；
- 所有 acceptance metrics PASS；
- 跑 full regression；
- 写 SLAM_VALIDATION_REPORT.md。

如果缺少真实 rosbag 导致某个真机指标无法实际验证，必须明确标记 NOT VERIFIED，并提供录制命令和验证脚本，不能虚构通过结果。
```

---

# 52. 本次重构成功的定义

本次不追求 SuperMap 完整体。

只要稳定做到：

```text
稳定 LiDAR-IMU SLAM
+
稳定 3D global map
+
RGB-D 物体 map-frame 定位
+
多视角 instance fusion
+
loop 后 semantic reprojection
+
自动质量验证
```

就算 MVP 成功。

最终系统应满足：

> 机器人绕环境一圈回来，墙还是同一面墙；  
> 相机从五个角度看到同一把椅子，地图上仍然只有一把椅子；  
> 回环闭合后，椅子标签会和地图一起被修正，而不是漂在旧位置；  
> 两把真实不同的椅子仍然能保留为两个实例。

这四条是本次实现的最高验收标准。
