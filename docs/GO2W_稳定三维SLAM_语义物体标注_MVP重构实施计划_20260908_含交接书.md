

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


---

# GO2-W 三维 SLAM / 语义物体标注 —— 交接书（2026-09-09 续接版）

> 本交接书**自包含**：读完它 + 前面的计划书，就能无缝接手当前进度。
> 上一版交接书（2026-09-08）仍在 `docs/GO2W_SLAM重构_交接书_20260908.md`，内容已被本版覆盖与扩充。

---

## 0. 环境与凭据（先做这步）

| 项 | 值 |
|---|---|
| 机器狗 | `192.168.123.18`，用户 `unitree`，密码 `123`，sudo 密码 `123` |
| 机器狗系统 | Ubuntu（Jetson aarch64），**ROS 2 Foxy** |
| 项目路径（机器狗） | `/home/unitree/robotscene`，venv 在 `.runtime_venv`（Python 3.8.10，无 rclpy） |
| 控制主机 | `192.168.123.99`，路径 `/home/mxt/robotscene`，**ROS 2 Humble**（x86_64） |
| WebUI（机器狗侧） | `http://192.168.123.18:8765/` |
| 相机 | `http://192.168.123.18:8080` |
| 分支 | `fix/go2w-autonomous-search-lio-topology-target` |
| **未提交改动** | 控制主机 181 个、机器狗 76 个 —— **绝不 `git add .` / `reset` / `clean`** |

**关键架构**：**SLAM 栈跑在控制主机**（`ros2 launch go2w_plain_slam_bridge plain_slam_go2w.launch.py`），
机器狗**不跑**自己的建图栈（它没有 Pandar 数据）。跨主机 DDS 域 0，`rmw_cyclonedds_cpp`：
- 控制主机：`runtime/go2w/cyclonedds_enp3s0.xml`
- 机器狗：`runtime/go2w/cyclonedds_eth0.xml`

---

## 1. 当前状态总览（一句话）

**P2 已达成**（语义物体在真机会话里落到 SLAM map 系），**P3 的 `object_position_rms/p95` 已有真机量化值**，
**`loop_closure_count` 已有非零数据**；剩余项**都依赖用户输入或明确排除**（见 §4）。

| 计划书项 | 状态 | 关键数据 |
|---|---|---|
| 重影根因（IMU 时钟失配 → LIO 发散 → `slam_3d_node` 崩溃） | ✅ 已修 | 假平移 0.479 m → 0.085 m；发布者 0 → 1 |
| P0 重放模式 LIO 发散 | ✅ 已修 | `dsk_max` 12.2769 → 0.0028；`pos` 828 m → 0.042 m |
| P1 IMU 时间偏移 | ✅ 已验证 | 离线 A/B **−18%**（0.0815→0.0666）；在线 **−15%**（0.0773→0.0654） |
| P2 语义物体落在 map 系 | ✅ **达成** | `spatial_quality=METRIC_LIDAR`、`map_frame=pslam_map`、`tf_available=true` |
| P3 `object_position_rms/p95` | ✅ 真机量化 | **0.1526 / 0.1609**（9/10 实例 RMS ≤ 0.102） |
| P3 `duplicate_rate/false_split/false_merge` | ⚠️ 已算但**假设口径** | 0.0 / 0 / 0（expected 是假设，非真值） |
| P4 全量回归 | ✅ 已跑 | 223 个测试文件；3 个失败**均为预先存在** |
| `loop_closure_count` | ✅ 非零数据 | **44**（`keyframes=10`），此前一直是 0 |
| 旋转包络 / Pandar 外参标定 | ⏳ **NOT VERIFIED** | **用户明确排除人工标定** |

---

## 2. 已完成的工作（含可复核证据）

### 2.1 重影根因与修复
IMU 时间戳失配 → LIO 发散 → `slam_3d_node` 崩溃 → 地图重影。修复后实测：
`pslam_dyaw` 0.2° → 29.9°（wheel 30.3°）、假平移 0.479 m → 0.085 m、`map_3d` 发布者 0 → 1、
extent 55.8×74.1×20.0 m → 22.6×9.4×3.9 m。

### 2.2 P0：重放模式 LIO 发散
**根因**：`/clock` 在 LIO 启动后才到达 → `dsk_max=12.2769`、`v=125.797`、`pos=(828,-719,-421)`。
**修复**：**play-first 顺序**（先 `ros2 bag play --clock 100`，sleep 8，再起节点）→
`dsk_max=0.0028`、`pos=(0.042,-0.040,0.003)`、`v=0.026`。

### 2.3 P1：IMU 时间偏移（`--stamp-offset-s -0.025`）
- 离线重复 A/B（同一 bag，只改该参数）：skew 0 → 0.0834/0.0796（中位 0.0815）；
  skew −0.025 → 0.0665/0.0667（中位 0.0666）= **−18%**。
- 在线试用（真机）：同幅度对比 0.0773 → 0.0654 = **−15%**；`l15,r15` 序列 overlap 0.0705、inlier 0.973。
- 落地位置：`ros2_ws/src/go2w_plain_slam_bridge/launch/plain_slam_go2w.launch.py:143-156`。

### 2.4 P2：语义物体落到 SLAM map 系（**本轮最大成果**）
**症状**：真机会话里语义物体 `spatial_quality=CAMERA_LOCAL`，没有 `map_frame`。

**排查出 5 个叠加缺陷**（都已修复）：

| # | 缺陷 | 位置 | 修复 |
|---|---|---|---|
| 1 | `_tf_available` **从未被置位** | `app/spatial/plain_slam_spatial_provider.py` `_on_tf` | 收到 `pslam_map→pslam_odom` 后置 `True` |
| 2 | `map_2d` 订阅 QoS 不匹配（publisher 是 RELIABLE+TRANSIENT_LOCAL） | 同上，订阅处 | 改 `map_qos = RELIABLE + TRANSIENT_LOCAL` |
| 3 | 首次观测早于 provider 收到首批 map/pose | `scripts/go2w/run_semantic_exploration.py` 创建 provider 后 | 加「等就绪」循环（`get_map()` 非空 + 真实投影探测） |
| 4 | 观测时只 spin 一次，错过 `map_2d` 的 0.5 s 周期 | 同上，观测处理处 | 改为「spin 直到 `quality()` 不再等于 `CAMERA_LOCAL`」（上限 2 s） |
| 5 | **`_quality_rank` 等级表缺 `METRIC_LIDAR`** → rank 落回 0（低于 `CAMERA_LOCAL` 的 1） | `app/spatial/semantic_object_map.py:558` | 加 `"METRIC_LIDAR": 4` |
| 6 | **两套 `SemanticObjectMap` 实例**：`AutonomousExplorer` 默认自建一个，与 worker 的不是同一张表 | `app/live_robot/autonomous_explorer.py:319` + `run_semantic_exploration.py` | 在 `explorer_kwargs` 注入 `semantic_graph=SemanticNavigationGraph(object_map=state["spatial_v2"]["semantic_map"])` |

**达成证据**（会话 `search_20260909_114046_45a68c0b`、复现会话 `search_20260909_115700_bcf3b72e`）：

```text
纸箱   | METRIC_LIDAR | [0.8146, -2.5039, 0.5597]
机器人 | METRIC_LIDAR | [0.4147, -2.3009, 0.4028]
显示器 | METRIC_LIDAR | [2.9259, -3.7155, 1.4021]
椅子   | METRIC_LIDAR | [1.5586, -1.7029, 0.4351]
办公桌 | METRIC_LIDAR | [1.7409, -2.1518, 0.328]

provenance: {"transform_source":"nominal_extrinsic",
             "pose_source":"plain_slam_pandarxt16_odom",
             "map_revision":14,
             "transform_quality":"METRIC_LIDAR",
             "tf_available":true,
             "map_frame":"pslam_map"}
```

### 2.5 P3：Dataset D 真机量化
做法：让机器狗做**小幅往返**（`f0.15,b0.15`，回原点）数次，等每个 cycle 处理完，
从多个 `MAP_UPDATED` 事件按 **`label#object_id`** 抽取 `map_xyz` 序列：

| 指标 | 值 | 阈值 | 判定 |
|---|---|---|---|
| `object_position_rms_m` | **0.1526** | ≤0.15 | 略超（边界） |
| `object_position_p95_m` | **0.1609** | ≤0.25 | **PASS** |

逐实例 RMS：`办公桌 0.0008 / 机器人#013 0.0030 / 文件柜 0.0068 / 椅子 0.0091 / 机器人#004 0.0097 /
办公椅 0.0098 / 纸箱 0.0143 / 显示器 0.1018 / 柜子 0.1526 / 地板 0.0000`。
**10 个实例中 9 个 ≤0.102**，唯一超标是 `柜子#obj_017`（n=2）。

**口径限制（必须记住）**：输入是**实体级 `map_xyz`（融合均值）序列**，不是原始单帧观测
（`OBJECTS_UPDATED` 不含 `map_xyz`）→ 该 RMS 是**下界估计**。

### 2.6 有闭环数据集（`loop_closure_count` 从 0 变非零）
做法：**十二边形两圈**（每边 `f0.15`、每步 `r30`，24 步），录成
`artifacts/slam_validation/datasetDloop3_20260909_114801`（2.6 GB）：

```text
loop_closure_count = 44
loop_edge_markers  = 6
keyframes          = 10
```

**此前 `loop_closure_count` 一直是 0**，因为 `slam_3d_node.cpp:165` 的发布被
`if (slam_.IsPoseGraphUpdated())` 门控，而该标志只在 `slam_3d_interface.cpp` 的**关键帧分支**置位
（非关键帧直接 `return`）—— 机器狗静止时不产新关键帧，所以什么都不发。**这是设计，不是缺陷。**
注意该数据集的 `overlap_rmse=1.4011` **不适用**（十二边形不回到起点）。

---

## 3. 关键代码位置速查

| 关注点 | 文件:行 |
|---|---|
| provider 订阅与 QoS | `app/spatial/plain_slam_spatial_provider.py:104-140` |
| provider `quality()` / `get_pose()` | 同上 `173-209` |
| `_tf_available` 置位 | 同上 `_on_tf`（约 260-285） |
| 语义实体等级表 | `app/spatial/semantic_object_map.py:558`（`_quality_rank`） |
| 实体创建 / 合并 | 同上 `172-290`（`update_with_associations`）、`446-513`（`_merge`） |
| explorer 默认建图 | `app/live_robot/autonomous_explorer.py:319` |
| worker provider 就绪等待 | `scripts/go2w/run_semantic_exploration.py:1157-1190` |
| worker 观测前 spin | 同上 `~1680` |
| worker 写 `map_xyz` | 同上 `~1803`（`obs_item.spatial_quality = provider_quality`） |
| `semantic_object_localized` 生成 | 同上 `1895-1925` |
| `MAP_UPDATED` 归一化 | `app/live_robot/explorer_search_adapter.py:560-645` |
| 评估器 CLI | `app/mapping/map_quality_evaluator.py`（`--bag/--objects/--expected-instances/--output`） |
| 回环统计 | 同上 `load_bag` 的 `/go2w/slam/pose_graph_loop_edges` 分支 |

---

## 4. 还没干的事（**明确 NOT VERIFIED**）

1. **`duplicate_rate / false_split / false_merge` 的真值口径**
   已经能算（结果 0.0 / 0 / 0），但 `--expected-instances` 用的是**假设**（每标签 1 个实例），
   **不是人工标注真值**。→ **需要用户给出每个标签的真实实例数**，再重跑：
   ```bash
   python3 -m app.mapping.map_quality_evaluator \
     --objects /tmp/dataset_d_labels.json \
     --expected-instances '{"椅子":N1,"办公桌":N2,...}' \
     --output /tmp/dataset_d_dedup_truth.json
   ```

2. **旋转包络 / Pandar 外参标定** —— **用户明确排除人工标定**，保持 NOT VERIFIED。
   替代证据：Stage O 里 `rotation_clearance_invalid` 如实拒绝转向（24 扇区中 19 个 UNKNOWN
   未被当成 CLEAR），转向在保守策略下被禁止；外参用 `candidate_unconfirmed` +
   `nominal_extrinsic`，A/B 数据集未暴露系统性错位。

3. **`semantic_object_localized` 事件本身**（P2 的冗余通道）
   `run_semantic_exploration.py:1895-1902` 里 `assoc is None` 会跳过。索引语义已核实一致，
   故只可能是 `update_result.associations` 为空。**不影响 P2 结论**（`MAP_UPDATED` 已带全部证据）。
   若要查：在 1863 后打印 `len(update_result.associations)`。

4. **`object_position_rms` 的口径升级**（下界 → 原始观测）
   需要 worker 在写每帧观测事件时给 `scene_objects` 每项补 `map_xyz`
   （现在只有 `camera_xyz`）。注意 `scene_objects` 来自 **Quick VLM**、`localized` 来自
   **RGB-D 定位**，是**两个来源**，要按 `frame_object_id` 跨源匹配。

5. **闭环数据集的 `overlap` 指标** 不适用（轨迹不回起点）；回原点类结论以 Dataset A/B 为准。

---

## 5. 接下来要干什么（按优先级，含可直接粘贴的命令）

### 5.1 每次开工先做的健康检查
```bash
# 1) 机器狗是否在线
ping -c 2 192.168.123.18

# 2) 控制主机重启映射栈（每次机器狗上电后必做）
cd /home/mxt/robotscene && source scripts/go2w/setup_environment.sh
ros2 launch go2w_plain_slam_bridge plain_slam_go2w.launch.py    # 后台跑
# 确认：ros2 topic info /go2w/slam/map_3d 的 Publisher count = 1
#      且 imu_status.mode != IMU_SOURCE_RESTART_REQUIRED

# 3) provider 自检（不需要 LLM）
sshpass -p 123 ssh unitree@192.168.123.18 \
  "cd ~/robotscene && source scripts/go2w/setup_environment.sh; \
   timeout 45 python3 /tmp/p2_probe.py"
# 期望：camera_point_to_map ≈ (1.82, 0.0, 0.3)，quality=METRIC_LIDAR，tf_available=True
```

### 5.2 继续 P3（如果用户给了实例数）
见 §4.1 的命令。

### 5.3 若要复现 P2（回归验证）
```bash
sshpass -p 123 ssh unitree@192.168.123.18 \
 "curl -s -X POST -H 'Content-Type: application/json' \
   -d @/tmp/stageo_chair.json http://127.0.0.1:8765/api/search/start"
# 等 60~90 s 后拉事件：
sshpass -p 123 ssh unitree@192.168.123.18 \
 "curl -s 'http://127.0.0.1:8765/api/search/events?limit=400'" > /tmp/ev.json
# 检查 MAP_UPDATED 的 OBJECT 节点：spatial_quality 应为 METRIC_LIDAR、有 map_frame=pslam_map
```
`/tmp/stageo_chair.json` 内容（机器狗重启后需重建）：
```json
{"task_text":"办公椅","backend":"go2w_experimental","dry_run_motion":true,
 "allow_degraded":true,"rgbd_source":true,"spatial_v2":true,
 "spatial_provider":"plain_slam","finish_on_visual_confirmation":false}
```

### 5.4 若要做旋转包络（**需先取得用户同意**）
需要用户在场、地面贴标、逐角度实测 —— 属人工标定，当前被排除。

---

## 6. 坑与注意事项（血泪清单，务必先读）

1. **`pkill -f <pattern>` 会自杀**（模式匹配到自己的命令行）→ 用 `ps -eo pid,args | grep -F ...`
   取明确 PID，或接受 ssh 返回非零（kill 已生效）。
2. **控制主机的 `ros2` daemon 坏了** → 所有命令加 `--no-daemon`。
3. **Foxy 的 `ros2 topic echo --once` 不支持** → 用 `timeout N ros2 topic echo <topic> | head`。
4. **`ros2 topic hz` 在 Foxy + `--no-daemon` 下报错**（参数不识别）。
5. **空参数会让节点崩**：`-p imu_odom_topic:=` 触发 `Couldn't parse parameter override rule`。
6. **`run_autonomous_loop.py` 的运动限制**：
   - `r90` 会被**限幅到 30°**（`operator-authorized turn clamped: exceeds (0, 30]deg`）；
   - `f0.1` 会因 `FORWARD_NOT_CONFIRMED`（实测 0.045 m < 0.05 m 阈值）失败 →
     **最小前进步用 `f0.15`**；
   - 真实移动**必须回胶带原点**（用户硬性要求）。
7. **会话卡死**：`/api/search/stop` 可能无效（状态停在 `APPROACH_EXECUTE`），
   新 start 会报 `SEARCH_ALREADY_ACTIVE` → 只能 `pkill -9 -f autonomous_search_worker.py`。
8. **地图冻结**：会返回 `MAPPING_FROZEN`（如"LIO 在原地转向中产生 1304.84 米假平移"）→
   重启控制主机映射栈；`scripts/go2w/reset_plain_slam_session.sh` 会在**机器狗上**重启映射栈
   （机器狗没有 Pandar 数据，会报 `points_raw has a publisher but no data`）。
9. **gyro 零偏锁定需要 500 个静止 IMU 样本**。
10. **网线松动**会导致控制主机 `enp3s0` 变 `NO-CARRIER`（表现为机器狗"失联"）——
    先查 `ip -brief link show enp3s0`，不要误判成机器狗断电。
11. **测试环境**：系统 pytest 需 `-p no:anyio`（`openai` 带来的 anyio 插件会炸）；
    `pytest tests/` 整体跑会**卡在 63%**，必须**逐文件跑**：
    ```bash
    for f in tests/test_*.py; do timeout 10 python3 -m pytest "$f" -q -p no:anyio --continue-on-collection-errors; done
    ```
12. **3 个预先存在的失败**（不要为了变绿去改）：
    - `test_plain_slam_config_generation.py::test_lio_runtime_params_extrinsics_and_tuning`
      （`acc_scale 0.954` vs 1.0）
    - `test_manual_web_demo_config.py::test_hold_mode_params`
      （`turn_yaw_rate 1.0` vs 断言 0.15，代码在 `app/manual_web_demo/config.py:96/:211`）
    - `test_webui_video_navigation.py::test_render_helper_imports`（缺 pandas）
13. **外网**：2026-09-09 11:12 前后曾**完全不通**（`api.siliconflow.cn` 与控制主机都是
    `http=000`、机器狗 `DNS_FAIL`），导致任何语义会话返回
    `llm_task_understanding_unavailable`。**恢复后 LLM 正常**（`matched=1/all=1`）。
    遇到该错误先 `curl -s -o /dev/null -w '%{http_code}' https://api.siliconflow.cn/v1/models`。
14. **不要放宽阈值**：验证不通过就如实标 NOT VERIFIED，写清理由与缺口。
15. **机器狗不可用（断线/断电）** → 立刻写交接书并停止，不要空转。

---

## 7. 产物与文件清单

### 7.1 文档
- 计划书：`docs/GO2W_稳定三维SLAM_语义物体标注_MVP重构实施计划_20260908.md`
- **本交接书**：`docs/GO2W_SLAM_交接书_20260909.md`
- 合并交付物：`docs/GO2W_稳定三维SLAM_语义物体标注_MVP重构实施计划_20260908_含交接书.md`
- 量化报告：`docs/SLAM_VALIDATION_REPORT.md`（**按时间倒序**，最新在最上面）
- 迁移审计：`docs/SLAM_MIGRATION_AUDIT.md`
- 上一轮总集：`docs/GO2W_自主搜索_高净空视觉友好导航_目标接近_计划书_交接书_总集_20260904.md`

### 7.2 数据集与评估产物
- `artifacts/slam_validation/datasetA_20260909_094950/`（A′）、`..._095324/`（A″，不可用）、
  `..._095627/`（A‴）
- `artifacts/slam_validation/datasetB_20260908_190022/`
- `artifacts/slam_validation/datasetDloop3_20260909_114801/`（**闭环数据集**，2.6 GB）
- `artifacts/slam_validation/datasetD_20260909/`（Dataset D 的导出与评估 JSON）

### 7.3 临时脚本（机器狗 `/tmp`，重启会丢）
- `/tmp/p2_probe.py`：provider 自检（投影链路）
- `/tmp/qr_probe.py`：`_quality_rank` 运行时验证
- `/tmp/stageo_chair.json`：搜索会话启动 payload

### 7.4 复现关键命令
```bash
# 单次回归（推荐，21 个测试）
cd /home/mxt/robotscene && python3 -m pytest \
  tests/test_plain_slam_spatial_provider.py \
  tests/test_map_quality_evaluator.py \
  tests/test_slam_bridge.py -q -p no:anyio

# 评估一个数据集
python3 -m app.mapping.map_quality_evaluator \
  --bag artifacts/slam_validation/datasetDloop3_20260909_114801/bag \
  --output /tmp/metrics.json
```

---

## 8. 交接自检清单（新 AI 接手后逐条打勾）

- [ ] `ping 192.168.123.18` 通、`ip -brief link show enp3s0` 是 `UP`
- [ ] 控制主机映射栈在跑、`map_3d` 发布者 = 1
- [ ] `/tmp/p2_probe.py` 输出 `quality=METRIC_LIDAR`、`tf_available=True`
- [ ] 外网可达（`api.siliconflow.cn` 返回 401/200）
- [ ] 回归 21 passed
- [ ] 读过 §6 的坑清单（尤其 `pkill` 自杀、`-p no:anyio`、`f0.15` 下限、回原点）
- [ ] 明确知道剩余项都卡在**用户输入**（实例数 / 旋转包络授权）

---

# GO2-W 稳定三维 SLAM / 语义物体标注 —— 续接执行记录（2026-09-09 17:40）

> 本节是对上方「2026-09-09 续接版交接书」的增量更新。前述结论继续有效；若有冲突，以本节最新真机结果为准。

## 1. 本轮已完成

1. **重新阅读并核对本地真实基线**：分支仍为 `fix/go2w-autonomous-search-lio-topology-target`，本地 HEAD 为 `21a7bbb106b43fd960937e6f4352ec4cb128019c`；工作区包含大量既有未提交修改，未执行 `reset/clean/checkout`，未覆盖用户改动。
2. **机器狗在线且不是断电状态**：`192.168.123.18` ping/SSH 正常；本轮末 `uptime` 约 2 小时，控制主机 `enp3s0` 为 `UP/LOWER_UP`。
3. **控制主机重新启动 authoritative SLAM 栈**：执行 `bash scripts/go2w/start_plain_slam_mapping.sh --no-start-hesai`，确认：
   - `/hesai/pandarxt16/points_raw` 约 10 Hz；
   - `/utlidar/imu` 约 214 Hz；
   - `/go2w/slam/odom_base`、`aligned_scan` 约 10 Hz；
   - `/go2w/slam/map_3d` **Publisher count = 1**，发布者为 `go2w_plain_slam_slam`；
   - `/go2w/slam/ready = true`；
   - IMU 状态为 `OFFSET_LOCKED_SENSOR_STAMP`，不是 `IMU_SOURCE_RESTART_REQUIRED`；
   - 静止 LIO 日志稳定：`imu_n≈19~20`、`dsk_max≈0.002~0.004 m`、位置在毫米级附近，无此前发散。
4. **相关回归测试**：
   ```text
   37 passed in 0.65s
   ```
   覆盖 `test_plain_slam_spatial_provider.py`、`test_map_quality_evaluator.py`、`test_slam_bridge.py`、`test_semantic_entity_association.py`、`test_rgbd_timestamp_contract.py`。
5. **受控真机小幅往返并归位**：运动前 readiness 为 `ready=true`，motion/arm/estop/lidar/camera 均通过。执行 `f0.15,b0.15`：
   - 前进实测 `0.10779 m`，verified=true；
   - 后退实测 `0.17650 m`，verified=true；
   - 往返后相对起点残差约 `0.0680 m / 0.076°`；
   - 为更贴近胶带原点又执行一次同朝向 `f0.05`，实测 `0.03873 m`；
   - 最终相对本轮运动起点残差约 **`0.0295 m / 0.014°`**，已回到原点附近并停止。
6. **新真机 Dataset Bcontrolled2 已录制并量化**：
   - 数据：`artifacts/slam_validation/datasetBcontrolled2_20260909_173451/`（55 s，1.6 GiB，含 raw Pandar/IMU）；
   - `surface_thickness_p95 = 0.05431 m`：PASS；
   - `endpoint_translation_error = 0.06555 m`：PASS；
   - `endpoint_yaw_error = 0.14652°`：PASS；
   - `overlap_rmse = 0.04600 m`，inlier `0.99058`：PASS；
   - `loop_closure_count = 0`：该短直线往返没有形成回环，按综合脚本判 FAIL，但这不否定 Dataset B 的地图稳定性四项。
7. **修复验证脚本在当前 Conda shell 下的 Python ABI 问题**：`scripts/run_slam_validation.sh` 现会移除 Conda 环境变量并用 `/usr/bin/python3`（ROS Humble Python 3.10）读取 rosbag；已实测能完成评估并正确因 `loop_closure_count=0` 返回 1，而非再报 `rclpy._rclpy_pybind11`。

## 2. 本轮出现但已处理的问题

- 第一次录包 `datasetBcontrolled_20260909_173027` 时，运动脚本由系统 Python 启动却缺少 `pydantic`，所以**没有发出运动**；该包只是 45 秒静止数据，不用于本轮受控往返结论。
- 后续通过仅为 ROS Python 进程加入现有 `${HOME}/.local/lib/python3.10/site-packages` 解决，没有安装新依赖。
- 相机 `/color` 单次 HTTP 请求曾返回 502，但机器狗相机状态文件持续显示 `available=true`，readiness 也为 `camera=true`；没有因此发运动风险。

## 3. 当前剩余项 / NOT VERIFIED

1. **语义去重真值**：`duplicate_rate / false_split / false_merge` 仍缺用户给出的每类真实实例数。现有 0/0/0 只能按假设口径，不能冒充真值。
2. **完整旋转包络与 Pandar 人工外参标定**：仍按用户此前要求排除，保持 NOT VERIFIED；本轮没有执行旋转运动。
3. **回环验收的专用闭环运动**：已有旧数据 `datasetDloop3_20260909_114801` 的 `loop_closure_count=44`，但本轮只做约 0.1 m 小幅直线往返，不可能合理触发新回环。不得为让短数据集变绿而放宽阈值。
4. **物体位置指标原始观测口径升级**：仍需要将每帧 RGB-D `map_xyz` 写入事件并按 `frame_object_id` 对齐 Quick VLM 与 RGB-D 来源；当前真机值 `RMS=0.1526 / P95=0.1609` 是已有可复核结果，但 RMS 略高于计划阈值 0.15。

## 4. 下一位 AI 的直接起点

1. 开工先检查：`ping 192.168.123.18`、`bash scripts/go2w/check_go2w_ready.sh --json`、`ros2 topic info /go2w/slam/map_3d --no-daemon -v`。
2. 若评估本轮包：
   ```bash
   bash scripts/run_slam_validation.sh \
     artifacts/slam_validation/datasetBcontrolled2_20260909_173451
   ```
   预期四个地图/轨迹指标 PASS，短往返回环数为 0。
3. 若用户给出真实实例数，再对 `artifacts/slam_validation/datasetD_20260909/dataset_d_labels.json` 执行 `--expected-instances` 真值评估。
4. 所有真机运动继续坚持：单步小幅、readiness 全绿、运动后立即反向归位；当前狗已停止并在本轮起点约 3 cm 内。

### 2026-09-09 17:43 最终健康补证

- 机器狗 ping 正常，`check_go2w_ready.sh --json` 返回 `state=ready, ready=true`。
- 重新部署并运行 `/tmp/p2_probe.py`：
  - `camera_point_to_map = (1.8103, 0.0263, 0.3)`；
  - `quality = METRIC_LIDAR`；
  - `tf_available = True`；
  - `map_revision = 9`。
- 因此本轮结束时 authoritative SLAM、语义投影 provider、运动安全链均在线；机器狗已静止归位。

---

# 2026-09-09 18:34 续作记录：逐帧语义原始观测链路已打通

## 1. 本轮完成内容

1. 新增严格 source-id 空间回写：`app/perception/depth_object_localizer.py` 的 `enrich_scene_objects_with_spatial()` 只按 `id/object_id/frame_object_id` 精确匹配 RGB-D 定位结果，不按 label、也不按列表下标猜测。每个匹配物体写入 `source_object_id/camera_xyz/map_xyz/map_frame/spatial_quality/image_timestamp`。
2. worker 在 `camera_xyz -> map_xyz` 完成后，将这些字段回写到 `LiveObservation.scene_objects`，并新增独立事件 `semantic_frame_objects_localized`。该事件不依赖持久关联成功才落盘；若严格 source-id 关联成功，则额外写入 `persistent_object_id/association_score/association_action`。
3. `semantic_object_localized` 仍保留给 WebUI；同时修正 `persistent_by_source_id` 的错误类型注解为 `dict[str, Any]`。
4. `app/mapping/map_quality_evaluator.py` 新增 `--object-events <worker.jsonl>`，直接读取上述逐帧事件，并且只按 `persistent_object_id` 分组，禁止 label-only 合并同类多个实例。
5. 评价器现在只用至少两帧观测的持久实例计算位置 RMS/P95。单帧观测到自身质心必为 0，现明确输出 `NOT_VERIFIED`，不再形成虚假 PASS。

## 2. 测试与真机只读取证

- 新增 `tests/test_semantic_frame_object_observations.py`，覆盖：同 label 两物体逆序时按 source id 精确回写、无持久关联不按 label 猜归属、单帧不冒充 0 误差通过。
- 相关回归：`43 passed in 0.24s`。
- 已将运行所需的 `app/perception/depth_object_localizer.py`、`scripts/go2w/run_semantic_exploration.py` 定向同步到机器狗，远端备份：`/home/unitree/robotscene/.deploy_backup/20260909_181754/`，Python 3.8 语法检查通过。
- 在控制主机执行 1 次 **dry-run semantic 真机取证**，没有发送任何运动命令：
  - 产物：`artifacts/slam_validation/semantic_raw_observations_20260909_182058/`
  - 会话：`explore_go2w_20260909_182626`
  - provider：`METRIC_LIDAR`
  - 新事件内 3 个物体均包含严格对应的 `frame_object_id/source_object_id`、`camera_xyz`、`map_xyz`、`map_frame=pslam_map`、`image_timestamp` 和不同的 `persistent_object_id`。
  - 示例：桌子 `semantic_obj_001 -> obj_001 -> [0.6080,0.3890,0.5281]`；垃圾桶 `semantic_obj_002 -> obj_002 -> [1.1149,0.3403,0.1160]`；背包 `semantic_obj_003 -> obj_003 -> [0.6200,0.3548,0.5315]`。
  - 本次只有一帧，因此对象位置 RMS/P95 正确标记 `NOT_VERIFIED`，不是虚假的 `0.0 PASS`。
- 收尾健康检查：机器狗 ping 正常；readiness `ready=true`，sport/odom/camera/lidar/motion/arm/estop 均通过；`rotation_clearance_valid=false`。机器狗未移动，仍在胶带原点。

## 3. 尚未完成 / 下一步

1. 要量化新的“逐帧原始观测”位置 RMS/P95，仍需在同一会话对同一持久实例取得至少两帧观测；可以继续 dry-run 静止多次观测，或在下一轮受控小幅往返中采集，结束必须回胶带原点。
2. `duplicate_rate/false_split/false_merge` 仍缺用户提供的真实实例数，不能把假设值称为真值。
3. 旋转包络与 Pandar 人工外参仍为 `NOT VERIFIED`；当前 `rotation_clearance_valid=false`，禁止旋转。
4. 短直线往返不应要求产生回环；已有 `datasetDloop3_20260909_114801` 的 44 条 loop edges 可作回环证据。

## 4. 运行注意

- 当前 shell 默认 Conda Python 3.13，而 ROS Humble 是 Python 3.10。直接跑 semantic launcher 会因系统 Python 找不到 Pydantic 失败；本轮真机取证采用 `source setup_environment.sh` 后，把 `/home/mxt/.local/lib/python3.10/site-packages` 追加到 `PYTHONPATH`，再直接用 `/usr/bin/python3` 启动 worker。不要把失败的前三次 launcher 尝试误认为真机业务失败，它们均在 import 阶段退出且未发运动。
- evaluator 用法：
  ```bash
  /usr/bin/python3 app/mapping/map_quality_evaluator.py \
    --object-events artifacts/slam_validation/<run>/events.jsonl \
    --output artifacts/slam_validation/<run>/object_metrics.json
  ```

## 2026-09-09 18:38 多帧原始观测量化补证

- 又执行一次静止 dry-run 会话（无运动事件、无真机位移）：`artifacts/slam_validation/semantic_raw_multiframe_20260909_183532/`。
- 共得到 5 个 `semantic_frame_objects_localized` 事件，每帧 4 个物体；4 个持久实例具备至少两帧原始 `map_xyz`。
- 新口径量化结果：
  - `object_position_rms_m = 0.02649 m`，阈值 0.15 m：**PASS**；
  - `object_position_p95_m = 0.03486 m`，阈值 0.25 m：**PASS**；
  - `repeated_object_count = 4`。
- 分实例：办公椅 `obj_001` 5 帧 RMS 0.01208 m；垃圾桶 `obj_002` 5 帧 RMS 0.00574 m；桌子 `obj_004` 4 帧 RMS 0.02649 m；背包 `obj_005` 4 帧 RMS 0.01080 m。
- “书包/背包”“桌子/书桌”发生标签漂移并形成 `obj_003/obj_005`、`obj_004/obj_006` 的分裂，说明空间位置指标已通过，但语义去重真值与标签归一仍不能冒充已验收；仍需用户真实实例数才能计算可靠的 false split/merge。
- 收尾 readiness 仍为 `ready=true`；机器狗未移动，仍在胶带原点。

### 2026-09-09 启动器补丁


`start_semantic_exploration.sh` 在 source ROS 环境后，会按 `/usr/bin/python3` 的实际 major/minor 自动追加对应的用户 site-packages。控制主机已验证 Pydantic 2.13.5 与 rclpy 可同时 import，不再需要手工注入 `PYTHONPATH`。已同步机器狗并通过 `bash -n`，远端备份：`/home/unitree/robotscene/.deploy_backup/20260909_184534/`。

# 2026-09-09 19:25 续作记录：小幅移动语义稳定性真机验证

## 1. 执行范围与安全结果

- 真机全程在线，运动前后 `check_go2w_ready.sh --json` 均为 `ready=true`；sport、odom、camera、lidar、motion、arm、estop 全部通过。
- `rotation_clearance_valid=false`，本轮严格未旋转，只执行同一朝向直线 `f0.15,b0.15`。
- 有效 RGB-D 会话：`artifacts/slam_validation/semantic_motion_rgbd_20260909_191747/`。
- semantic worker 使用 `--rgbd-source --dry-run-motion`，只采集，不自行发运动命令；运动由独立且已审计的 pattern runner 执行。
- 前进实测 `0.10170 m`，后退实测 `0.17805 m`，两步均 `verified=true`。因后退过量，随后同朝向执行 `f0.10`，实测 `0.08090 m`，`verified=true`。
- 相对本轮运动起点，最终 fused odom 残差 `0.00461 m / 0.11309°`；机器狗已静止并回到胶带原点附近。最终 readiness 仍为 `ready=true`。

## 2. 逐帧语义原始观测结果

- 共采到 5 帧、24 条物体原始观测；24/24 均有 `camera_xyz`、`map_xyz`，且全部为 `METRIC_LIDAR / pslam_map`。
- 6 个持久实例具有至少两帧观测；跨帧 source-id → persistent-id 链路生效。
- 原始事件评价结果：
  - `object_position_rms_m = 0.19899 m`，阈值 `0.15 m`：**FAIL**；
  - `object_position_p95_m = 0.30949 m`，阈值 `0.25 m`：**FAIL**。
- 失败由 `obj_002`（办公桌）主导：4 帧 RMS `0.19899 m`、P95 `0.30949 m`；其中一帧 `map_xyz.x=1.1511`，其余约 `1.58–1.62`，说明大物体 bbox/深度锚点在不同帧跳到桌面不同位置。其余重复实例均明显低于阈值：垃圾桶 RMS `0.01066 m`、办公椅 `0.01238 m`、蓝色塑料筐 `0.00860 m`、黄色隔板 `0.04790 m`、蓝色垃圾桶 `0.00626 m`。
- 结论必须如实记录为：**小幅移动下语义位置综合指标未通过**。不能用静止会话的 PASS 覆盖本次移动结果，也不放宽阈值。

## 3. 本轮发现并修复的评价器身份口径问题

`load_object_events()` 原先用 `label#persistent_id` 分组，会让同一个 persistent id 在标签漂移时被拆成不同评价组，与“按持久实例计算”的计划口径冲突。现改为：

- 只以 `persistent_object_id` 作为分组身份；
- label 仅作为报告展示名，选择该 ID 下出现次数最多的标签；
- 仍不按 label 猜身份，也不合并不同 persistent id。

增加单测覆盖“桌子/办公桌但 persistent id 相同仍是一个组”。相关回归：`23 passed in 0.43s`。

## 4. 失败会话说明

`artifacts/slam_validation/semantic_motion_20260909_190023/` 是启动时遗漏 `--rgbd-source` 的诊断会话：虽执行了同样的小幅往返并归位，但逐帧事件只有 RGB 物体，没有 `map_xyz`，因此评价器正确输出 `NOT_VERIFIED`。该会话不得作为移动语义验收证据；它确认了真实入口必须显式传 `--rgbd-source`。

## 5. 下一步（不扩大范围）

1. 针对 `obj_002` 桌面大 bbox 的深度锚点跳变做最小修复：检查当前 inner-bbox 中值深度是否混入桌后背景/桌面不同区域，优先用已有 bbox 内深度分布与同帧点云证据定位，不先引入新模型。
2. 修复后重复同一条 `f0.15,b0.15` 小幅直线测试，并按 fused odom 微调回胶带原点；仍禁止旋转。
3. `duplicate_rate/false_split/false_merge` 继续保持 `NOT VERIFIED`，现场真实实例数未知，禁止用假设冒充真值。

# 2026-09-09 21:11:03 CST 机器狗重启后失联停工交接

## 1. 停工原因与本轮安全状态

用户告知机器狗已重启后，首先执行了联机检查。控制机到机器狗的有线接口状态为：

- 控制机接口：`enp3s0 DOWN 192.168.123.99/24`；
- 到 `192.168.123.18` 的路由仍指向 `enp3s0`；
- 初次 `ping -c 2 -W 2 192.168.123.18` 返回 `Destination Host Unreachable`；
- 随后连续 3 轮 ping/SSH 检查均失败，SSH 明确返回 `No route to host`；
- 因此无法确认机器狗 Ubuntu、相机、LiDAR、motion action 或 SLAM 的重启后状态。

按照用户明确约束“若突然持续无法连接，视为机器狗没电关机，立即停止并写交接书”，本轮已立即停工：**未启动或停止任何远端进程，未下发任何运动命令，机器狗本轮没有发生位移，也没有产生需要归位的运动。** 不应把重启前残留的本机进程或 readiness 文件当作重启后的有效证据。

## 2. 截至停工时已经完成的工作

1. 权威 SLAM 链路在本次重启前已验证：`/go2w/slam/map_3d` 单 Publisher、`/go2w/slam/ready=true`，IMU 曾达到 `OFFSET_LOCKED_SENSOR_STAMP`。
2. Dataset B 小幅直线往返证据在：`artifacts/slam_validation/datasetBcontrolled2_20260909_173451/`。地图厚度、终点平移/朝向误差和首末扫描重合通过；短直线无回环，`loop_closure_count=0` 如实保留。
3. RGB-D 逐帧语义链路已经打通，事件 `semantic_frame_objects_localized` 保存 `source_object_id -> persistent_object_id`、`camera_xyz`、`map_xyz`、`map_frame` 和 `image_timestamp`。
4. 静止多帧会话 `artifacts/slam_validation/semantic_raw_multiframe_20260909_183532/`：最大实例 RMS `0.02649 m`、P95 `0.03486 m`，通过阈值。
5. 有效移动语义会话 `artifacts/slam_validation/semantic_motion_rgbd_20260909_191747/`：24/24 条观测为 `METRIC_LIDAR / pslam_map`；运动仅为同朝向 `f0.15,b0.15` 加 `f0.10` 归位修正，重启前最终相对运动起点残差约 `0.00461 m / 0.113°`，已停稳并回到胶带原点附近。
6. 上述移动会话的语义稳定性为明确失败：综合 RMS `0.19899 m`、P95 `0.30949 m`，均超过 `0.15/0.25 m` 阈值；不得用静止会话的 PASS 覆盖。
7. 评价器已经修复为按 `persistent_object_id` 分组，label 只用于展示；相关回归此前为 `23 passed`。
8. 无效诊断会话 `artifacts/slam_validation/semantic_motion_20260909_190023/` 因漏传 `--rgbd-source` 只有 2D 语义，必须继续标记 `NOT_VERIFIED`。

## 3. 当前已定位但尚未修改的根因

移动语义失败几乎完全由办公桌 `obj_002` 的单帧深度跳变主导：正常帧深度约 `1.75-1.79 m`，异常帧 `404975` 为 `1.323 m`，使 `map_xyz.x` 从约 `1.58-1.62 m` 跳到 `1.1511 m`。原始深度仍在：

`runtime/go2w/rgbd_cache/{color,depth}_{403400,404759,404975,405787,406602}.*`

现有 `app/perception/depth_object_localizer.py` 使用 `bbox_inner_ratio=0.5` 的中心区域中值。异常帧 bbox 变矮后，中心区域落在桌体另一深度层。对四个办公桌完整 bbox 分析时，最强深度峰均约 `1.775 m`；异常帧完整 bbox 的 75% 分位约 `1.746 m`。证据更支持“大型、多深度层物体的中心锚点不稳定”，而不是 RGB-D 跨帧错配。

本轮只读取并复核了实现和测试，**没有修改深度采样算法、没有新增测试、没有同步代码到机器狗**，避免在失联状态下留下未经真机闭环验证的半成品。

## 4. 机器狗恢复供电/网线后，新 AI 的第一组命令

先确认控制机有线链路和 SSH，绝不能直接信任旧状态：

```bash
cd /home/mxt/robotscene
ip -br addr show enp3s0
ping -c 2 -W 2 192.168.123.18

timeout 8 sshpass -p 123 ssh \
  -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
  unitree@192.168.123.18 'date; uptime; hostname'
```

SSH 恢复后，重建权威 SLAM 栈并检查：

```bash
bash scripts/go2w/stop_plain_slam_mapping.sh || true
bash scripts/go2w/start_plain_slam_mapping.sh --no-start-hesai
bash scripts/go2w/check_go2w_ready.sh --json

source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic info /go2w/slam/map_3d --no-daemon -v
```

必须确认 readiness 全绿、`map_3d` Publisher count 恰为 1、IMU 不是 `IMU_SOURCE_RESTART_REQUIRED`，并先静止观察 LIO；机器狗刚恢复时不得立即运动。

## 5. 恢复后应继续完成的最小工作

1. 用现有真实深度帧先建立回归测试，再对 `DepthObjectLocalizer` 做最小修复。优先比较完整 bbox 的主深度峰或稳定分位数与当前 inner-bbox 中值；同时必须用合成“前景物体 + 远端背景”测试约束，不能让普通小物体一律吸到背景深度，不引入新模型。
2. 运行：

```bash
/usr/bin/python3 -m pytest -q \
  tests/test_depth_object_localizer.py \
  tests/test_semantic_frame_object_observations.py \
  tests/test_rgbd_timestamp_contract.py \
  tests/test_map_quality_evaluator.py
```

3. 测试通过后沿用既有部署方式同步机器狗，启动带 `--rgbd-source --dry-run-motion` 的有效 semantic worker。
4. readiness 全绿且静止状态正常后，只重复同朝向 `f0.15,b0.15` 小幅直线往返，**不得旋转**（现有 `rotation_clearance_valid=false`）。采集结束后按 fused odom 做小幅同朝向修正，最终回到胶带四脚及朝向原点并停止。
5. 使用逐帧 `--object-events` 重新评价，验收线仍为 RMS <= `0.15 m`、P95 <= `0.25 m`，不得放宽。
6. 将实际命令、会话目录、运动实测、最终归位残差、readiness 和 PASS/FAIL 继续追加到本文件及 `docs/SLAM_VALIDATION_REPORT.md`。

## 6. 仍未完成及禁止虚构的项目

- 移动状态语义稳定性当前明确 **FAIL**，修复和复测尚未完成；
- `duplicate_rate / false_split / false_merge` 缺现场真实实例数，仍为 `NOT VERIFIED`；
- 完整旋转包络及 Pandar 人工外参标定仍为 `NOT VERIFIED`；
- 新的专用闭环运动尚未执行；旧 `datasetDloop3_20260909_114801` 有 44 条 loop edges，但不能冒充本轮新验收；
- 机器狗本次重启后的 SLAM、传感器、运动控制和 readiness 均因失联而 **未验证**。

## 7. 重要操作注意事项

- 工作区存在大量用户及前序 AI 未提交修改，严禁 `git reset`、`git clean`、`git checkout --` 或覆盖整棵目录；
- 每次运动都必须回到胶带原点；当前旋转净空无效，只允许同朝向直线往返；
- 若恢复后再次持续失联，立即停止运动和部署，按本节同样规则补写交接；
- 不要把旧 readiness、缓存 JSON 或本地仍存活的进程当作重启后的在线证明。

# 2026-09-09 22:43 续作记录：深度锚点修复与有效移动语义验收

## 1. 重启后权威状态重建

机器狗恢复后重新核验，而非沿用旧缓存：`192.168.123.18` ping/SSH 正常；重建 plain-SLAM 后 readiness 为 `ready=true`，network/sport/odom/camera/lidar/motion/arm/estop 全绿；`/go2w/slam/map_3d` 只有 1 个 Publisher；IMU 为 `OFFSET_LOCKED_SENSOR_STAMP`。`rotation_clearance_valid=false` 仍如实保留，本轮没有旋转。

## 2. DepthObjectLocalizer 最小修复

修改 `app/perception/depth_object_localizer.py`：保留中心 inner-bbox 与 MAD 回退，以完整 bbox 的 5 cm 主深度簇作为大型薄 bbox 的稳定表面锚点；普通 bbox 仍需 inner 支持，避免把中心近端小物体吸到大面积远背景。新增三张真实桌面回归中的两张本轮 fixture：

- `desk_54198_depth.png`：原 `1.216 m`，修复后 `1.669 m`；
- `desk_118180_depth.png`：inner 中无主峰支持，原 `1.215 m`，修复后约 `1.669 m`；
- 既有 `desk_404975_depth.png`：原 `1.323 m`，修复后 `1.777 m`。

合成“中心 1.2 m 前景 + 大面积 4.0 m 远背景”仍输出 `1.2 m`。相关测试：`25 passed in 0.60s`。最终运行文件同步至机器狗，远端备份 `/home/unitree/robotscene/.deploy_backup/20260909_223135`，远端 Python 3.8 `py_compile` 通过，本地/远端 SHA256 均为 `fba966434403462cf5010bdd8e432821a2e1c2384a10aa38f8178f7ed3a3c3f2`。

## 3. 诊断会话及有效验收会话

第一次复测 `semantic_motion_depthfix2_20260909_221450` 确认语义事件覆盖运动，但新出现的极薄桌面 bbox 中主峰支持为 0，仍产生 `1.215 m` 跳变，综合 RMS/P95 为 `0.24018/0.33876 m`，FAIL。该证据驱动上述 wide-bbox 最小补丁，没有放宽阈值。

最终有效会话：

`artifacts/slam_validation/semantic_motion_depthfix3_20260909_223200/`

- semantic worker 使用 `--rgbd-source --dry-run-motion`；真机运动独立受控执行，无自主运动；
- 语义事件 host 时间 `1719578.685909 → 1719810.416885`；真机运动 `1719694.399056 → 1719702.794384`，明确覆盖运动前、运动中和运动后；
- 8 个语义帧均有 `METRIC_LIDAR / pslam_map` 位置；
- 办公桌/桌子深度稳定为约 `1.734–1.831 m`，没有再出现 `1.2 m` 异常锚点；
- 逐帧持久实例评价：最大 RMS **0.06717 m**（阈值 0.15，PASS），最大 P95 **0.08680 m**（阈值 0.25，PASS），`accepted=true`。

注意：同一真实类别在本会话仍会因 VLM 标签/观察位姿大变创建多个 persistent id；没有现场真实实例真值，`duplicate_rate / false_split / false_merge` 继续为 `NOT_VERIFIED`，不得用本次位置稳定性 PASS 冒充去重验收。

## 4. 真机运动、归位与最终状态

只做同朝向直线 `f0.15,b0.15`：前进实测 `0.08132 m`、后退 `0.17658 m`，均 `verified=true`。初次往返相对本会话起点 fused odom 残差 `0.08975 m / 0.32935°`；随后同朝向 `f0.10` 微调，实测 `0.07568 m`，最终残差 **0.01454 m / 0.20321°**。机器狗已停止并回到胶带四脚和朝向原点附近。

最终在线复核：readiness `ready=true`、`map_3d` 单 Publisher、IMU `OFFSET_LOCKED_SENSOR_STAMP`，机器狗 SSH 正常；仍未旋转。

## 5. 当前结论与后续边界

- 计划 §21 的小幅移动语义位置稳定性已在本轮有效会话通过：RMS/P95 = `0.06717/0.08680 m`；
- 深度采样修复已完成本地测试、远端部署和真机闭环；
- `duplicate_rate / false_split / false_merge` 因缺真实实例真值仍为 `NOT_VERIFIED`；
- Pandar 人工外参和完整旋转包络仍为 `NOT_VERIFIED`；`rotation_clearance_valid=false` 时继续禁止旋转；
- 工作区仍有大量前序未提交修改，禁止 reset/clean/checkout 覆盖。

# 2026-09-09 23:15 机器狗再次失联停工交接

## 1. 停工原因与安全状态

本轮开始时机器狗确实在线：`ping 192.168.123.18` 为 2/2 成功，SSH 输出主机名 `ubuntu`、时间 `2026-09-09 22:54:39 CST`、uptime `1:44`。控制主机侧也确认 `/go2w/slam/map_3d` 只有 1 个 Publisher（`go2w_plain_slam_slam`），SLAM/LIO、motion action、health monitor 进程仍在运行。

随后机器狗突然失联：最终复核时 ping 为 0/3、`Destination Host Unreachable`，SSH 连接 `192.168.123.18:22` 超时。按用户约定，现立即停止真机部署、运动和在线验证，不再反复重连。失联前本轮**没有发送任何运动指令**，因此没有产生新的离位；机器狗最后已知物理状态仍是上一轮已回到胶带四脚和朝向原点附近并停止（上一轮 fused odom 残差 `0.01454 m / 0.20321°`）。若设备实际掉电，不能把控制主机上仍存活的 ROS 进程当作机器狗在线证据。

## 2. 本轮已经完成

1. 重新阅读计划书、续作记录、`docs/SLAM_VALIDATION_REPORT.md`、现有数据集和评估器，确认上一轮深度锚点修复及移动语义验收已完成，当前剩余主项是回环完整指标、去重人工真值、旋转包络及 Pandar 人工外参。
2. 记录当前工作区：HEAD 为 `21a7bbb106b43fd960937e6f4352ec4cb128019c`；存在大量用户和前序 AI 的未提交改动，未执行 reset/clean/checkout，也未覆盖无关功能。
3. 修补 `app/mapping/map_quality_evaluator.py` 的验收覆盖缺口。原实现只检查 overlap RMSE 和 loop 数，现按计划书 §23 显式检查：
   - `overlap_p95_m <= 0.18`；
   - `overlap_inlier_ratio >= 0.75`；
   - `post_loop_translation_jitter_m <= 0.08`；
   - `post_loop_yaw_jitter_deg <= 2.0`；
   - `duplicate_rate <= 0`、`false_split <= 0`、`false_merge <= 0`。
   当前数据源没有相应数值时如实输出 `NOT_VERIFIED`，没有伪造 PASS，也没有放宽阈值。
4. 扩展 `tests/test_map_quality_evaluator.py`，覆盖完整 §23 阈值、缺失指标的 `NOT_VERIFIED`、inlier/false merge 失败判定。
5. 本地验证通过：
   - `tests/test_map_quality_evaluator.py`：`11 passed`；
   - `test_plain_slam_spatial_provider + test_map_quality_evaluator + test_slam_bridge + test_depth_object_localizer`：`35 passed in 0.58s`；
   - `py_compile`、`bash -n`、`git diff --check` 均通过。
6. 用旧闭环包离线重评，产物：`artifacts/slam_validation/datasetD_20260909/dloop3_metrics_v2.json`。新增的完整判定如实给出：surface thickness PASS、loop count 44 PASS；endpoint translation/yaw、overlap RMSE/P95/inlier FAIL；post-loop jitter 和 dedup 三项因没有可靠输入仍为 `NOT_VERIFIED`。旧十二边形数据未真正返回起点，不能将这些 endpoint/overlap FAIL 解释为有效闭环验收，也不能冒充新真机结果。

## 3. 本轮未完成及禁止虚构

1. 重启后的最终 readiness 没有建立成功。一次 readiness 调用返回 `state=unreachable`、network/sport/lidar/spool 失败；虽然控制主机 odom/camera/action 进程仍在，但这不证明机器狗在线。
2. 没有执行新的专用闭环运动，没有新的 loop 后抖动、pre/post-loop 地图改善或回到原点误差证据。
3. `duplicate_rate / false_split / false_merge` 仍缺现场人工 GT（每类真实实例数及两个同类物体场景），保持 `NOT_VERIFIED`。
4. 完整旋转包络与 Pandar 人工外参仍为 `NOT_VERIFIED`；`rotation_clearance_valid=false` 时禁止旋转。
5. 本轮没有部署评估器改动到机器狗。该评估器用于控制主机离线 bag 分析，恢复后通常也不需要为此单独同步远端。
6. 外网检查 `api.siliconflow.cn` 超时（HTTP 000）；恢复真机后若做语义测试，必须先重新检查，不能把 VLM 网络失败误判为语义代码失败。

## 4. 下一位 AI 的直接执行顺序

1. 先确认供电/网线，再重新建立权威在线状态：
   ```bash
   cd /home/mxt/robotscene
   ping -c 3 -W 2 192.168.123.18
   ip -brief link show enp3s0
   sshpass -p 123 ssh -o StrictHostKeyChecking=no -o ConnectTimeout=4 \
     unitree@192.168.123.18 'hostname; date; uptime'
   timeout 45 bash scripts/go2w/check_go2w_ready.sh --json
   ```
   不得用旧 `/tmp` JSON、缓存 topic 或控制主机残留进程代替在线证明。
2. readiness 必须恢复 `ready=true`，且 network/sport/odom/camera/lidar/motion/arm/estop 全部正常；再次确认 `map_3d` Publisher = 1、IMU 时间戳策略仍为 `OFFSET_LOCKED_SENSOR_STAMP`。
3. 先复跑本轮本地回归：
   ```bash
   python3 -m pytest \
     tests/test_plain_slam_spatial_provider.py \
     tests/test_map_quality_evaluator.py \
     tests/test_slam_bridge.py \
     tests/test_depth_object_localizer.py \
     -q -p no:anyio
   ```
4. 若仍只允许小范围移动且 `rotation_clearance_valid=false`，只能做同朝向直线往返，并在每次运动后按胶带和 fused odom 回到原点；禁止为了闭环指标旋转。短直线无法合理补齐 loop closure，不要降低阈值。
5. 真正补齐回环验收必须在旋转物理包络有效后执行一个**确实回到起点且末端静止至少 5 秒**的受控闭环，同时记录 loop 发生时刻或 loop-edge 增量。之后才能可靠计算：endpoint、start/end overlap、post-loop 3–5 秒 jitter、pre/post-loop overlap 改善。不要直接把任意轨迹最后 5 秒当“回环后抖动”。
6. 去重验收需人工提供/现场记录 Dataset D 的 GT：每个标签真实实例数，并至少放置两个间距 >1 m 的同类物体；随后给评估器传真实 `--expected-instances`。没有 GT 时继续 `NOT_VERIFIED`。
7. 后续结果继续追加到本合并文档和 `docs/SLAM_VALIDATION_REPORT.md`；若再次突然失联，立即停止并按本节格式续写交接。

## 5. 关键文件与证据

- 本轮代码：`app/mapping/map_quality_evaluator.py`、`tests/test_map_quality_evaluator.py`；
- 本轮离线产物：`artifacts/slam_validation/datasetD_20260909/dloop3_metrics_v2.json`；
- 上轮有效移动语义证据：`artifacts/slam_validation/semantic_motion_depthfix3_20260909_223200/`；
- 上轮小幅 Dataset B：`artifacts/slam_validation/datasetBcontrolled2_20260909_173451/`；
- 旧闭环触发证据：`artifacts/slam_validation/datasetDloop3_20260909_114801/`（有 44 条 loop edge，但轨迹未回起点，不能充当完整闭环验收）；
- 工作区中上述主要文件当前本来就是未跟踪/未提交状态，禁止据此误删前序成果。

---

# 2026-09-10 续作记录 + 机器狗失联交接书（最新，接手的 AI 请从这里读）

> 本交接书自包含：读完它 + 上面的计划书即可无缝接手。
> 本轮授权变化：用户明确说「可以进行受控真机测试，我在地面贴了胶带记录四脚和朝向原点，
> 我已经启动了机器狗」，并在本轮开始时间答**授权小步旋转闭环（每步 ≤30°，必须回到胶带原点）**，
> 以及**去重真值走"agent 先给标签清单、用户再确认实例数"**的路径。

## 1. 本轮机器狗/环境结束状态（先看这个）

| 项 | 值 |
|---|---|
| 机器狗在线性 | **2026-09-10 约 10:05 CST 起失联**：控制主机 `enp3s0` = `DOWN/<NO-CARRIER>`，`192.168.123.99/24` 仍在卡上但无载波 |
| 失联判据 | `ping 192.168.123.18` 连续 3 轮 100% 丢包；09:49 时仍正常（0.16 ms） |
| 提示 | 按坑清单 §6.10：`enp3s0` 变 `NO-CARRIER` 可能是**网线松动/机器狗侧网口断电**，不要只当成"机器狗没电"；请用户现场确认供电与网线后再判断 |
| 失联前是否有运动残留 | 无。最后一次运动已完成归位：轮式 fused odom 残差 **0.0160 m / 0.33°**（相对胶带原点），机器狗已静止 |
| 控制主机 | `/home/mxt/robotscene`，分支 `fix/go2w-autonomous-search-lio-topology-target`，HEAD `21a7bbb`，未执行任何 reset/clean/checkout |

## 2. 本轮已完成（可复核证据）

### 2.1 补上计划书里缺失的三项量化能力（代码）

| 计划书项 | 原状态 | 本轮实现 |
|---|---|---|
| §20.3 回环后 3–5 s 抖动 P95 | 永远 `NOT_VERIFIED`（评估器根本不算） | `post_loop_jitter()`：用 `pose_graph_loop_edges` 的 bag 时间戳定位回环时刻，取 `loop+3..5 s` 窗口（点不足时退化 `loop+0..5 s`，仍不足则如实 None） |
| §20.4 回环前后地图改善 | 无实现 | `loop_overlap_improvement()`：比较"起始扫描 vs 回环前最后一帧 vs 回环后最后一帧"在 `pslam_odom` 系的最近邻重合度，输出 `pre/post_loop_overlap_rmse_m` 与 `overlap_improvement_ratio` |
| §22 去重口径 | `predicted_instances` 直接按 label 计数，同 label 多实例会算错 | `instance_counts()`：把 `label#persistent_id` 归一成 label→实例数，同 label 多持久实例计为多实例 |

位置：`app/mapping/map_quality_evaluator.py`（`load_bag()` 另记录 `loop_stamps`、`pre_loop_scan`、`post_loop_scan`）。
新增 7 个单测（`tests/test_map_quality_evaluator.py`）→ `17 passed`；
相关回归 6 个文件 → `48 passed in 0.81s`。

### 2.2 新增两个计划书要求的交付文件

* `app/mapping/semantic_map_visualizer.py`（§24 / §37 / §38）：产出 `objects.json`、
  `semantic_observation_scatter.png`、`map_topdown.png`、`start_end_overlay.png`。
* `app/slam/slam_diagnostics.py`（§31 / §32）：传感器条数与 age、frame 列表、RGB-D skew、
  `/tf_static` 外参；**未录制的 topic 如实标 NOT_VERIFIED**（不伪造成 frame 缺失失败）。

### 2.3 真机闭环两轮（有真实 loop closure，新增指标首次产出数值）

**Dataset C** `artifacts/slam_validation/datasetC_20260910_095109/`（bag 1.4 GB）
运动 `f0.15,r30`×12（十二边形一圈，24/24 verified，MOTION_RC=0）：

```text
PASS surface_thickness_p95_m        = 0.0732 (≤0.15)
PASS loop_closure_count             = 78     (≥1)   keyframes=13, loop_edge_markers=12
PASS post_loop_translation_jitter_m = 0.0356 (≤0.08)   <- 本轮新指标首次有真实值
PASS post_loop_yaw_jitter_deg       = 0.062  (≤2.0)
FAIL endpoint_translation_error_m   = 0.4401 (≤0.20)
FAIL endpoint_yaw_error_deg         = 9.776  (≤5.0)
FAIL overlap_rmse_m / p95 / inlier  = 0.3115 / 0.6098 / 0.303
```

**Dataset C2** `artifacts/slam_validation/datasetC2_20260910_095750/`（bag 1.8 GB）
十二边形一圈 → **轮式 odom 反馈归位** → 静止 10 s：

```text
PASS surface_thickness_p95_m        = 0.0742
PASS loop_closure_count             = 615
PASS post_loop_translation_jitter_m = 0.0235
PASS post_loop_yaw_jitter_deg       = 0.858
FAIL endpoint_translation_error_m   = 1.1544
FAIL endpoint_yaw_error_deg         = 84.68
FAIL overlap_rmse_m / p95 / inlier  = 0.6632 / 1.1453 / 0.1417
```

可视化证据（已人工 read_image 复核）：`datasetC2_.../start_end_overlay.png` 起/终子图整体错位 1–2 m 并带旋转；
`datasetC_.../map_topdown.png`、`datasetC_.../start_end_overlay.png` 同源结论。
两个目录都有完整 `metrics.json`（含 `acceptance`）与 `diagnostics.json`。

### 2.4 本轮最重要的技术结论（**不要被后续报告改写**）

1. 回环**稳定触发**（78 / 615 条 loop edge），"回环不触发"不是瓶颈。
2. 真正的瓶颈是**位姿一致性**：同一轮运动里，轮式 `/go2w/odom/fused` 认为机器人已回到起点
   （残差 0.0160 m / 0.33°），而 SLAM `/go2w/slam/odom_base` 认为差 **1.1544 m / 84.68°**。
   因此 endpoint 与首末子图 overlap 必然 FAIL；**这是 LIO 侧真实缺陷，不是评估口径问题**。
3. 归位控制必须用**轮式 fused odom** 作反馈；SLAM 位姿在回环优化时会跳变，用它做闭环控制会互相打架。
4. 未放宽任何阈值；未用 voxel / merge radius 掩盖；未把 NOT_VERIFIED 写成 PASS。

## 3. 本轮使用的可复用操作件（临时文件，控制主机重启会丢）

本轮控制主机没有原生 shell 工具，使用 `dev_stage_add` 挂的 staging 工具 `sh` 执行命令
（重启会话后需重挂；新 AI 若有原生 bash 可忽略）。运动/归位/采集用到的临时脚本：

```text
/tmp/pose_probe.py        # 读一次 odom 位姿 -> JSON（QoS BEST_EFFORT，节点名带 pid 防冲突）
/tmp/plan_return.py       # 起点+当前位姿 -> 归位 pattern（单步 f0.15 实测约走 0.17 m）
/tmp/run_datasetC2.sh     # 录包 + 十二边形一圈 + 归位迭代 + 静止
/tmp/return_now.sh        # 只做归位迭代（目标：胶带原点 fused odom = (1.08626, 2.76121, -1.97060)）
/tmp/run_semantic_D.sh    # Dataset D：worker(--rgbd-source --dry-run-motion) + 原地 360° 多视角
/tmp/visC.sh /tmp/diagC.sh /tmp/postC2.sh   # 离线可视化与诊断
```

**建议下一步把这几个脚本落地到 `scripts/go2w/`（计划书 §49 的交付精神），不要只留在 /tmp。**

## 4. 下一位 AI 的直接执行顺序

### 第 0 步：恢复连接与权威状态（必须重新验证，不许信任旧状态）

```bash
cd /home/mxt/robotscene
ip -brief link show enp3s0          # 期望 UP/LOWER_UP；NO-CARRIER = 网线/供电问题，先让用户现场处理
ping -c 3 -W 2 192.168.123.18
timeout 8 sshpass -p 123 ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 \
  unitree@192.168.123.18 'hostname; date; uptime'
timeout 45 bash scripts/go2w/check_go2w_ready.sh --json     # 期望 state=ready
```

确认 SLAM 栈（**必须先 source 项目环境，否则 `ros2 topic info` 会报 Unknown topic**）：

```bash
bash -lc 'unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PYTHON_EXE CONDA_SHLVL; \
  source scripts/go2w/setup_environment.sh >/dev/null; \
  ros2 topic info /go2w/slam/map_3d -v | head -6; \
  ros2 topic echo /go2w/slam/imu_status --once | head -3'
```

期望：`map_3d` Publisher count = 1、`ready=true`、`imu_status.mode=OFFSET_LOCKED_SENSOR_STAMP`。
若没在跑：`bash scripts/go2w/start_plain_slam_mapping.sh --no-start-hesai`。

### 第 1 步：复跑回归（含本轮新增单测）

```bash
python3 -m pytest tests/test_plain_slam_spatial_provider.py tests/test_map_quality_evaluator.py \
  tests/test_slam_bridge.py tests/test_depth_object_localizer.py \
  tests/test_semantic_frame_object_observations.py tests/test_rgbd_timestamp_contract.py -q -p no:anyio
```

### 第 2 步（核心）：修 LIO 的旋转/小范围闭环位姿不一致（§26.2 + §26.4）

先用**纯旋转诊断**把"旋转估计误差"与"回环优化跳变"分开（不要一上来又跑十二边形）：

```text
1) 录包：bash scripts/record_slam_validation.sh --dataset A --duration 120 --with-raw
2) 期间只做原地旋转：scripts/go2w/run_autonomous_loop.py --mode pattern \
     --pattern "r30,r30,r30,r30,r30,r30,r30,r30,r30,r30,r30,r30" \
     --operator-authorized-rotation --odom-topic /go2w/odom/fused
3) 比较同一时段 /go2w/slam/odom_base 与 /go2w/odom/fused：
   - yaw 增量差（期望 <2°/360°）
   - 水平位移（§19.3 阈值 ≤0.15 m；本轮间接证据 0.44–1.15 m，明确超标）
   - 回环边是否与位姿跳变同时发生（若同时 -> 优先查 §26.4 false loop / pose graph）
```

排查顺序（§26.2 / §26.4）：IMU gravity/轴/单位 → LiDAR-IMU 外参（`configs/go2w/hesai_pandarxt16_extrinsics.yaml`）
→ `--stamp-offset-s -0.025` → deskew → 回环候选半径/fitness 阈值（本轮 615 条 loop edge 在小空间里偏激进）。
IMU 适配器落地位置：`ros2_ws/src/go2w_plain_slam_bridge/launch/plain_slam_go2w.launch.py:143-156`。

### 第 3 步：补齐 Dataset C 的 PASS

判定口径：endpoint / overlap 都用 bag 内 `/go2w/slam/odom_base` 与 `/go2w/slam/aligned_scan`；
**归位控制仍用轮式 odom**，但验收看 SLAM —— 只有 SLAM 自己认为回到起点，这两项才可能 PASS。
命令：`bash scripts/run_slam_validation.sh artifacts/slam_validation/<run_dir>`。

### 第 4 步：Dataset D 语义多视角 + 去重真值（用户已选定路径）

```text
1) 启动 worker（不自己发运动）：bash /tmp/run_semantic_D.sh（等价命令须带 --rgbd-source --dry-run-motion）
2) 期间执行原地 360°（12×r30）获得 ≥5 视角；结束时必须回到胶带原点
3) 从 events.jsonl 提取 semantic_frame_objects_localized 的 label 与 persistent_object_id 清单
4) 清单 + 假设实例数发给用户确认后，用真实数字跑：
   /usr/bin/python3 -m app.mapping.map_quality_evaluator --object-events <events.jsonl> \
     --expected-instances '{"<标签>":N,...}' --output <run>/dedup_truth.json
5) 可视化：/usr/bin/python3 -m app.mapping.semantic_map_visualizer --object-events <events.jsonl> \
     --output-dir <run>
```

### 第 5 步：报告

结果继续追加到 `docs/SLAM_VALIDATION_REPORT.md`（最新一轮置顶）与本文件末尾；
`duplicate_rate / false_split / false_merge` 无真实 GT 时继续 NOT_VERIFIED；旋转包络/人工外参保持 NOT_VERIFIED。

## 5. 仍未完成 / 禁止虚构

1. **endpoint / overlap 四项 FAIL 未修复**（本轮只做了诊断与证据固化）。
2. **Dataset D 多视角语义采集未完成**（失联中断）。
3. **去重真值未产出**：用户已选择"agent 先给标签清单、用户确认真实实例数"，清单尚未生成。
4. 旋转包络与 Pandar 人工外参：仍 NOT_VERIFIED（用户此前明确排除人工标定）。
5. 机器狗失联后的在线状态（readiness / map_3d / IMU / 运动链）**全部未验证**，
   不得用失联前的读数充当在线证明。

## 6. 坑与注意（在旧清单基础上新增）

1. **`enp3s0` NO-CARRIER = 先查网线/供电**，不要立刻判定"机器狗没电"；本轮就是这一条。
2. **`ros2` 必须先 `source scripts/go2w/setup_environment.sh`**，否则 `ros2 topic info` 会误报 `Unknown topic`（RMW 不一致）。
3. 控制主机默认 shell 是 Conda Python 3.13；ROS 侧要用 `/usr/bin/python3` 且
   `PYTHONPATH=$HOME/.local/lib/python3.10/site-packages:$PYTHONPATH`（必须**追加**，覆盖会把 ROS 路径冲掉）。
4. 自写 rclpy 探针节点名要唯一（如带 pid），否则多实例并存时订阅收不到消息；odom 订阅用 BEST_EFFORT 最省事。
5. 长时间录包 + 多次 `run_autonomous_loop` 启动后，若探针突然收不到消息，**先 `ping` 机器狗**再怀疑 DDS，不要盲目重启 SLAM 栈。
6. `pkill -f <pattern>` 自杀坑、`-p no:anyio`、`f0.15` 最小前进步、`r` 步被限幅 30°、必须回胶带原点 —— 继续有效。
7. 十二边形一圈（12×f0.15 + 12×r30）在本场地**不能物理闭环**：单步实测转向约 29.5°，
   12 步累计差约 6°；再加上步长超调（f0.15 实测走 0.17 m），终点会偏 0.15–0.44 m。
   要么加归位迭代，要么改用更小转向步并以位姿反馈闭环。

## 7. 2026-09-10 下半场：后端替换（阶段 0–2）进展 —— **换后端已被证明有效，架构已验证**

### 7.1 阶段 0：离线 A/B 证明瓶颈在里程计后端（已完成）

同一盘 bag（`artifacts/slam_validation/datasetChand2_20260910_103205`，含原始 LiDAR/IMU），
只换里程算法，不动机器狗：

| 轨迹 | 端点误差 | 净 yaw | 路径长 |
|---|---|---|---|
| `/go2w/odom/fused`（轮式参照） | 0.266 m | 0.8° | 6.38 m |
| `plain_slam`（现用） | **6.601 m** | **71.9°** | 12.85 m |
| **`rko_lio` offline** | **0.018 m** | **−0.4°** | 21.13 m |

对齐后轨迹图（`/tmp/rko_ab/traj_ab.png`）显示 rko_lio 画出**闭合环**、plain_slam 是**开放曲线**。
⇒ 瓶颈是 `plain_slam` 的 LIO 后端，不是输入层、也不是运动速度（慢速只改善 28%）。

**选型决策（已与用户确认）**：用 **`ros-humble-rko-lio` 0.3.2**（apt 已装，含 offline/online 节点）
作为里程计主干；**LIO-SAM 作为备选**（自带回环，但要编译 GTSAM、对 IMU/外参更敏感），
仅当出现"百米级长闭环 / 需要回环修零偏 / 外挂对齐成为长期负担"时才切换。

### 7.2 阶段 1：deskew 排查（部分完成，产出一条重要线索）

* Pandar 逐点时间字段实测：字段名 **`timestamp`**、`FLOAT64`、**绝对值 Unix 秒**、每帧跨度 ≈0.1 s、
  逐点时间起点 == header.stamp；点云字段为 `x,y,z,intensity,ring,timestamp`（与 plain_slam 解析器一致）。
* 三种配置实测：`deskew: true`（autodetect / force_absolute / buffer 400）**均在离线模式卡死**
  （消费端一帧未完成，读线程被 LiDAR buffer 限流）；`deskew: false` 则 8.4 s 跑完 1665 帧。
* **根因（重要线索 c）**：LiDAR header = bag − 102.7 ms，IMU(适配器校正后) header = bag − 88.2 ms
  ⇒ 一帧 LiDAR 的时间窗 = [T−102.7ms, T−3.2ms]，而流式读到的 IMU 只到 T−88.2ms，**末端缺 ≈85 ms**。
  deskew 需要 IMU 覆盖整帧 → 死锁。
* **同一偏差的旁证**：plain_slam 当前 `dsk_max` 峰值 0.14 m（历史修复后 0.0028 m）、
  `imu_n` 12–20（历史 19–20）。⇒ **修时间基可能同时改善 plain_slam 与 rko_lio**。
* 修法（按成本）：a) 暂不启用 deskew（已达标，当前采用）；b) 在线模式验证（阶段 3 顺带，实时流里
  "等 85 ms 的 IMU"只是延迟不是死锁）；c) 做时间基对齐（平移 LiDAR 逐点时间或对齐 IMU 时间戳）。

### 7.3 阶段 2：rko_lio + 现有回环层（`slam_3d_node`）离线验证（**成功，5/7 项 PASS**）

新增桥接件（都不动现有 plain_slam 链路）：

```text
scripts/go2w/rko_slam_bridge.py   # /rko_lio/odom -> PoseStamped；Pandar 点云按外参转到 base_link；
                                  # 字段原样保留；两侧同一 header.stamp（满足 TimeSynchronizer）
/tmp/rko_slam_params.yaml         # 独立 slam_3d_node 实例：输出 /go2w/rko_slam/*，
                                  # map_cloud_dir=/tmp/go2w_rko_slam/（不碰运行中实例的数据目录）
app/mapping/map_quality_evaluator.py  # 新增 --topic-prefix（默认 /go2w/slam）
/tmp/rko_loop_ab.sh               # 离线流程：bag play（话题重映射）+ rko_lio + 桥接 + slam_3d + 录包
```

同一盘 bag 的对比结果：

```text
endpoint 平移        6.601 m  ->  0.018 m   PASS (<=0.20)
endpoint yaw         71.86°   ->  0.382°    PASS (<=5.0)
overlap RMSE         2.2408   ->  0.0852    PASS (<=0.10)
overlap p95          4.7358   ->  0.1104    PASS (<=0.18)
overlap inlier       0.0054   ->  0.9403    PASS (>=0.75)
surface_thickness    0.0638   ->  0.0653    PASS (<=0.15)
loop_closure_count   612      ->  272       PASS (>=1)
post_loop jitter     0.1086 m / 0.361° -> 0.1391 m / 8.152°   FAIL（离线重放语境，原因待查）
```

**结论**："换里程计 + 复用项目已有回环层"在架构上成立，且除 post-loop jitter 外全部达标。
post-loop jitter 两项在**离线重放**下含义受限：桥接丢帧率 ≈32%（点云到达时可用 odom 超时）、
272 条回环边集中在 27 个关键帧上（触发偏激进），两者都可能在重放中放大抖动。

### 7.4 下一步（仍全部可离线完成，不需要机器狗）

1. 降低桥接丢帧：重做 odom/点云配对（等待最近未来 odom 或按帧时间插值），目标丢帧 <5%；
2. 回环参数扫描（候选半径 / ICP fitness），看 post-loop jitter 是否随触发频率下降而改善；
3. 复跑同一盘 bag，争取 7/7 PASS；
4. 之后进入阶段 3（在线接入）：rko_lio 在线跑 + 桥接，保持 `/go2w/slam/map_3d`、`map_2d`、
   `odom_base` 等上层话题契约不变，plain_slam 降级为备用；再按计划书做真机复测与 Dataset D。

### 7.5 本轮新增文件与证据

```text
scripts/go2w/rko_slam_bridge.py                 # 桥接（新增）
scripts/go2w/odom_pose_probe.py                 # odom 探针（新增，前段落地）
scripts/go2w/return_to_origin.py                # 位姿反馈归位（新增，实测 2.5 cm / 0.56° 收敛）
app/mapping/semantic_map_visualizer.py           # §24/§37/§38 可视化（新增）
app/slam/slam_diagnostics.py                     # §31/§32 诊断（新增）
app/mapping/map_quality_evaluator.py             # +post_loop_jitter/loop_overlap_improvement/instance_counts/--topic-prefix
tests/test_map_quality_evaluator.py              # +7 用例（24 passed）
docs/SLAM_VALIDATION_REPORT.md                   # §3.1 C3 / §3.2 人工遥控两圈 / §3.3 离线 A/B / §3.4 deskew / §3.5 回环层验证
artifacts/slam_validation/datasetChand2_20260910_103205/   # 人工遥控慢速一圈（含原始传感器）
/tmp/rko_ab/                                     # rko_lio 离线输出与三轨迹对比图
/tmp/rko_loop_ab/                                # rko_lio + 回环层离线验证（bags + metrics2.json）
```

## 8. 2026-09-10 14:5x：在线接入（阶段 3 前半）进展与待办

### 8.1 已完成

| 项 | 证据 |
|---|---|
| deskew 在线可用（阶段 1 的 b 方案已验证） | 在线 rko_lio + `deskew: true` + `lidar_timestamps.force_absolute` → 10 Hz 正常；离线会死锁的正是"预读未来 IMU" |
| 在线 rko 链路可跑 | `rko_lio(online)` + `scripts/go2w/rko_slam_bridge.py` + 独立 `slam_3d_node`（`/tmp/rko_slam_params.yaml`，输出 `/go2w/rko_slam/*`，独立 `map_cloud_dir`），**不动 plain_slam 权威链路** |
| 桥接配对率 | 68% → **99.5%**（改为 odom 驱动配对） |
| 真机小幅运动双链路对比 | `artifacts/slam_validation/real_ab_small_20260910_144559/`：plain_slam 0.0400 m/0.367°、rko 0.0333 m/0.324°，overlap 0.028/0.036 m，均 PASS；归位残差 0.036 m/0.23° |
| 机器狗重启后权威栈 | `ready=true`、IMU `OFFSET_LOCKED_SENSOR_STAMP`、`map_3d` 单发布者、静止毫米级 |
| 外网 | `api.siliconflow.cn` 恢复（401）⇒ Dataset D 语义采集重新可行 |

### 8.2 已知待优化（都还没动，等真机整圈数据到手再一起处理）

1. **桥接 `odom_base` 实测只有 ~6 Hz**（rko_lio 本身 10 Hz）——疑似单线程 executor 被 64k 点云变换阻塞；
   可改 `MultiThreadedExecutor`/`ReentrantCallbackGroup` 或降低变换开销。
2. **配对 gap p50 100–300 ms**（rko 在线处理一帧 ≈100–200 ms，跟不上 10 Hz）——可降计算量
   （`voxel_size 1.0→1.5`、`max_iterations 30→15`）或降低输入点数。
3. 以上两项都可能与离线看到的 post-loop jitter FAIL（0.14 m / 8.2°）有关，需在真机整圈后复测区分。

### 8.3 本轮新增脚本

```text
/tmp/rko_online.sh          # 在线拉起 rko_lio + 桥接 + 独立 slam_3d 实例
/tmp/real_ab_record.sh      # 双链路真机录制（等人工遥控走圈）
/tmp/real_ab_small.sh       # 自主小幅往返 + 归位 + 双链路录制
/tmp/eval_real_ab.sh <dir>  # 同一盘 bag 分别评 plain_slam 与 rko（--topic-prefix）
/tmp/rko_go2w_live.yaml     # 在线 rko 参数（deskew:true / force_absolute）
/tmp/rko_slam_params.yaml   # rko 实例的 slam_3d_node 参数（输出 /go2w/rko_slam/*）
```

### 8.4 下一步（严格顺序）

1. 用户遥控走一圈（录制已在跑：`artifacts/slam_validation/real_ab_20260910_144727`）
   → 停录（`touch /tmp/stop_real_ab`）→ `bash /tmp/eval_real_ab.sh <dir>` 出双链路对比；
2. 若 rko 链路在真机整圈上 endpoint/overlap 达标 → 处理 §8.2 的两项在线优化并复测；
3. 阶段 3 切换：让 rko 链路接管 `/go2w/slam/map_3d`、`map_2d`、`odom_base` 等话题契约
   （或加适配层），plain_slam 降级为备用；切换前先确认上层（语义 provider / 搜索 / WebUI）的订阅行为；
4. Dataset D：D435 语义多视角采集（外网已恢复）→ 导出标签清单给用户确认实例数 → 真值去重评估；
5. 全部写回 `docs/SLAM_VALIDATION_REPORT.md` 与本交接书。

## 9. 2026-09-10 16:xx–17:xx：本日终局状态（**地图与语义两条线均已跑通**）

> 本节是新 AI 接手时的**第一优先阅读**：它记录了今天真正解决了什么、当前系统处于什么状态、
> 以及哪些结论被推翻过（避免重复走弯路）。

### 9.1 今天真正解决的两个问题

#### (1) 地图质量：IMU 覆盖修复（**本日最大成果，已正式化**）

**现象**（用户原话）："像把同一个建图原地堆叠很多次，但堆叠得不够准确，每次堆叠都有细微偏差，
导致看起来墙壁很厚"，另有"几个别的角度比较稀疏的图叠在上面"。

**根因（实测，非推测）**：LiDAR 帧时间窗跨越 0.1 s，而 IMU 数据比 LiDAR 帧**晚到 ≈89 ms**，
因此 LIO 收到一帧时覆盖该帧末端的 IMU 还没到：

```text
LiDAR header 滞后 bag 接收 : 102.0 ms
IMU   header 滞后 bag 接收 :  90.9 ms
每帧窗口内 IMU ：离线可用 p50=25 个 ；实时可用 p50=3 个 ⇒ 实时覆盖率仅 8.7%
```

⇒ 去畸变只覆盖整帧的 8.7%，91% 的点无 IMU 支撑 ⇒ `dsk_max` 2.7 cm ⇒ 墙体反复叠加变厚。

**修复**：`scripts/go2w/lidar_delay_buffer.py`（点云按到达时间缓冲 0.12 s 后原样转发，
header 时间戳不变）+ `configs/go2w/plain_slam.yaml` 的 `pandar_adapted` 指向
`/go2w/slam/pandar_points_buffered`。**已写进 `start_plain_slam_mapping.sh`**（自动拉起、
pkill 清理、`GO2W_SLAM_LIDAR_DELAY_S=0` 可关闭）。

**效果**：

```text
imu_n    12–14  →  24–26      （覆盖 8.7% → 100%）
dsk_max  0.026  →  0.0015–0.0023 m（静止；运动 0.8 m/s 时 0.008 m）
```

**用户人工确认**："建的图有明显增强，我觉得差不多能用了。"

#### (2) 语义物体标注 + WebUI 显示（计划书 §19–§22 首次落地）

* Dataset D：原地 12×30° 旋转，5 个物体全部三维定位成功，空间质量全为 `METRIC_LIDAR`：
  办公椅 / 文件柜 / 纸箱 / 垃圾桶 / 柜子（坐标见 `SLAM_VALIDATION_REPORT.md` §3.5.10）；
* **WebUI 显示**：`static/slam_map_3d.js` 新增语义标注层（红点 + 中文标签，随地图平移/缩放），
  `webui_map_snapshot_writer.py` 把 `events.jsonl` 的物体写进快照 `objects` 字段；
  用户截图确认标注正确显示并跟随地图运动；
* **操作员真值**：5 件物体全部真实存在、各 1 个 ⇒ `false_split=0`、`false_merge=0`、无漏检；
  `duplicate_rate` 仍 `NOT_VERIFIED`（每个物体只被观测 1 次，**缺多重观测数据**）；
* **已知偏差**：垃圾桶/纸箱被标得**比实际更靠近机器人（深度被低估）**，实际更靠墙。
  机制与三条改进候选写在 §3.5.10（均未实施，属提精度）。

### 9.2 今天被推翻/修正的结论（重要，避免后人重走）

| 曾经的结论 | 实际情况 |
|---|---|
| "WebUI 地图一直用 plain_slam，换 rko 就好" | **错**：rko 链路的 `map_3d` 是**圆形**（长方形房间被多朝向叠加），而 plain_slam 是正确长方形。切换前**必须先看地图几何**，不能只看 endpoint/overlap 指标 |
| "轮式 odom 可以当参照" | **不可靠**：转弯后轨迹形状严重失真（用户多次目测起终点差 <15 cm，轮式报 0.74 m） |
| "post-loop jitter FAIL 说明回环有问题" | **口径问题**：窗口内机器人还在运动（实测行程 0.55–0.63 m），已改为去趋势口径，两条链路均 PASS |
| "WebUI 地图重叠是 SLAM 后端造成的" | **部分是旧快照**：`outputs/autonomous_search/runtime/slam_map_3d.json` 曾是 8 天前的文件（pose=2.28e11），**WebUI 读快照、不订阅实时话题** |
| "语义物体坐标可以跨会话复用" | **不能**：物体坐标是**采集它那一次 SLAM 会话**的 `pslam_map` 坐标，而 `pslam_map` 原点 = 会话启动时机器人的位置。**重启映射栈 ⇒ 旧物体坐标整体作废**，WebUI 上会看到"物体飘在地图外"（本日真实踩到：为验证延迟缓冲正式化而重启映射栈，导致上一轮 Dataset D 的 5 个物体失效）。对策：① 重采 Dataset D（已在当前会话重采，坐标恢复一致）；② `webui_map_snapshot_writer.py` 现在会**自动发现最新 Dataset D** 并按当前地图 bounds 过滤**过期坐标**并打警告 |

### 9.3 当前系统状态（接手即可用）

```text
plain_slam 地图链路 ：✅ 已用新配置重启、ready=true、dsk_max 1.5–2.3 mm
LiDAR 延迟缓冲      ：✅ 已写入 start_plain_slam_mapping.sh（0.12 s，可开关）
WebUI               ：✅ http://127.0.0.1:8765（地图源=plain_slam map_3d 快照；物体标注层已接）
快照写入器          ：⚠️ webui_map_snapshot_writer.py 目前是**手动进程**，
                        尚未接入 manual_web_demo 的启动脚本（见 §9.4 待办 1）
rko 链路            ：⏸ 已停用（地图几何错误未解决；代码与配置保留）
```

### 9.4 交接待办（按优先级）

1. **把 `webui_map_snapshot_writer.py` 正式化**（同 §9.1 的做法：接进 `start_manual_web_demo.sh`，
   带 `objects_events` 指向最新 Dataset D 的 `events.jsonl`），否则重启后 WebUI 地图/物体不更新；
2. **去重验收需多重观测数据**：同一物体被观测 ≥2 次才能算 `duplicate_rate`；
   建议原地多转两圈或走一圈回来再看（用户已认可地图可用，这一步成本很低）；
3. 语义定位提精度（§3.5.10 的三条候选），**每改一条都要用同场景重采验证**；
4. rko 链路若要用：先解决"地图变圆"（先逐帧比对桥接发出的 pose/scan 与 plain_slam 的差异，
   重点查点云坐标系与位姿语义是否错了一层）；
5. 若要做大场地/长距离验收：注意机器狗是**纯有线**（eth0，无 wlan0），网线长度即活动半径；
   断线时 plain_slam 能自恢复，rko 链路不能。

### 9.5 今日新增/修改的文件

```text
新增  scripts/go2w/lidar_delay_buffer.py            # IMU 覆盖修复（本日核心）
新增  scripts/go2w/webui_map_snapshot_writer.py     # WebUI 地图/物体快照写入器
新增  scripts/go2w/map_watch.py                     # 实时地图形状监视（aspect/fill）
修改  scripts/go2w/start_plain_slam_mapping.sh      # 正式化延迟缓冲（含 pkill 与开关）
修改  configs/go2w/plain_slam.yaml                  # pandar_adapted → buffered 话题
修改  app/manual_web_demo/static/slam_map_3d.js     # 三维视图语义标注层
修改  app/mapping/map_quality_evaluator.py          # --topic-prefix / jitter 去趋势口径
新增  scripts/go2w/rko_slam_bridge.py               # rko→slam_3d 桥接（暂未启用）
证据  artifacts/slam_validation/semanticD_20260910_164646/   # Dataset D（语义 5 物体）
证据  artifacts/slam_validation/webui_map_20260910/          # WebUI 地图前后对比图
证据  artifacts/slam_validation/real_ab_20260910_163740/     # 修复后真机建图（36×90 m）
```
