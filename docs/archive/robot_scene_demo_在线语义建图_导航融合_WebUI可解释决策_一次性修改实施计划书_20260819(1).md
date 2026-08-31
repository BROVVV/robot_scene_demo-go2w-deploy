# robot_scene_demo 在线语义建图 × 导航融合 × WebUI 可解释决策 一次性修改实施计划书

> 版本：2026-08-19  
> 项目：https://github.com/BROVVV/robot_scene_demo  
> 目标平台：Unitree Go2-W + Intel RealSense D435 + ROS2 Humble + RTAB-Map  
> 执行对象：具备代码仓库读写、运行测试、修改 Python/JavaScript/ROS2 配置能力的 Coding AI  
> 本文定位：**直接用于执行修改，不是讨论稿、建议清单或研究方向说明。**

---

# 0. 给执行 AI 的最高优先级指令

你拿到本计划书以后，不要再输出一份新的“建议”“TODO”“架构分析”或“可以这样做”的说明。

你的任务是：

1. 拉取并审计 `robot_scene_demo` 当前最新 working tree / GitHub `main`。
2. 先阅读仓库根目录已经存在的三份实施计划与 README，理解现有架构，避免推倒重来：
   - `robot_scene_demo_RGBD_UniGoalV2_PSG_空间自主探索_一次性重构实施计划书_20260818.md`
   - `robot_scene_demo_真机自主语义搜索_WebUI_一次性实施计划书_20260817.md`
   - `robot_scene_demo_高层自主语义探索_一次性实施计划书_20260817.md`
3. **以当前代码为事实来源。** 计划书与代码冲突时，先确认当前代码已有实现，再做最小兼容性重构。
4. 直接实现本计划要求的全部代码、测试、前端展示、配置和文档。
5. 不得只新增空接口、占位实现、固定假数据或 `TODO` 后声称完成。
6. 不得以“缺少人工标定”为理由阻塞软件闭环。需要外参时必须提供：
   - TF 优先；
   - nominal 外参 fallback；
   - degraded mode；
   - health/quality 明示。
7. 不得删除现有安全门禁、急停、Operator Supervision、运动限幅。
8. 不得把 API Key、Token、`.env`、机器敏感配置写入仓库。
9. 所有新增逻辑必须有可离线执行的单元测试；ROS/真机部分必须有可 mock 的纯 Python 核心。
10. 修改完成以后必须实际运行：
    - 相关新增测试；
    - 现有空间导航核心测试；
    - WebUI mock E2E；
    - 能运行的全部 `pytest` 子集。
11. 最终交付不能只是“代码能 import”，必须满足本文最后的验收标准。
12. 如果当前仓库已经实现了某一部分，不要重复造一套；直接把缺失链路接通并补测试。

最终目标不是“页面上多显示几个字段”，而是把系统真正闭环成：

```text
自然语言目标
    ↓
GoalGraph / TargetProfile
    ↓
D435 同步 RGB-D
    ↓
目标检测 / VLM SceneGraph
    ↓
DepthObjectLocalizer
    ↓
camera_xyz
    ↓
camera → base → map/odom 坐标变换
    ↓
map_xyz
    ↓
Persistent Entity Association
    ↓
Semantic Entity Graph
    ↕
PlaceGraph + RTAB Occupancy Map + Frontier
    ↓
语义相关性 + PSG + 几何可达性 + 路径代价
    ↓
Long-Term Spatial Goal
    ↓
Route / Next Local Goal
    ↓
结构化 DecisionRecord
“下一步做什么 / 为什么 / 其他候选为什么没选”
    ↓
LocalGoalExecutor
    ↓
RobotBackend
    ↓
Go2-W 相对转向 / 短步移动
    ↓
新观测
    ↓
地图更新 + 实体融合 + 路径重规划
```

---

# 1. 本次必须一次解决的三个核心问题

本次修改必须同时解决以下三个问题，不能只做其中一部分。

## 1.1 问题 A：当前导航到底有没有融合图片识别路线规划

当前代码已经存在两条历史技术路线：

### 离线视频路线

`app/navigation/navigation_planning_pipeline.py`

核心流程大致是：

```text
video
→ estimate_video_trajectory
→ build_video_navigation_map
→ localize_semantic_goal
→ select_goal
→ plan_visual_path
→ generate_navigation_instructions
→ Nav2 adapter
```

它属于：

> 视觉识别/视频轨迹 → 地图 → 目标位置 → 路径规划。

### 当前真机 SemanticNavigation V2

`run_semantic_exploration.py` 当前已经初始化并使用：

- `PlaceGraph`
- `SemanticObjectMap`
- `SpatialMemory`
- `FrontierExtractor`
- `LongTermGoalSelector`
- `LocalGoalExecutor`
- PSG prior
- RGB-D depth localization

当前更接近：

```text
识别
→ 语义匹配
→ Frontier / Anchor Region
→ 长期探索目标
→ ROTATE_VIEW / RELATIVE_MOVE
→ 重新观察
→ 重规划
```

这已经让视觉识别参与了导航决策，但还没有形成完整的：

```text
持久语义地图
→ 基于地图的目标位置
→ 路径代价
→ 路线规划
→ 路线执行
```

### 本次修改后的要求

必须把两条路线统一成一个共享的“在线空间语义导航层”：

```text
视觉语义决定“去哪”
RTAB/几何地图决定“怎么到”
PlaceGraph 决定“经过哪些空间节点”
RobotBackend 决定“怎么执行”
```

**不要**把离线 `run_video_navigation_planning()` 整个塞进真机循环。

应该复用它的理念和可复用算法，把离线和在线统一到共享数据结构和 planner 接口。

---

## 1.2 问题 B：WebUI 缺少真正 SLAM-like 的物体节点图

现在已经有：

- RTAB Occupancy Map 接口；
- `PlaceGraph`；
- `SemanticObjectMap`；
- `SearchMapRenderer`；
- WebSocket spatial events。

但是关键链路没有打通：

```text
camera_xyz
→ map_xyz
```

同时：

- `PlaceGraph` 当前主要只和“当前 Place”比较，回到旧地点时可能产生重复 Place；
- `SemanticObjectMap` 只有 `map_xyz` 才能可靠跨视角融合；
- `observed_scene_graph()` 当前对象节点有了，但 `edges` 仍为空；
- Place 中当前保存的 `observed_object_ids` 实际上容易混入 label，而不是稳定 persistent object id；
- 对象位置更新是“新值覆盖旧值”，还不是可靠概率融合；
- WebUI 已能画 map_xyz 物体，但上游通常没有 map_xyz。

本次必须升级成真正的：

```text
Occupancy Map
+
Robot Trajectory
+
Place Graph
+
Persistent Object Entity Graph
+
Place ↔ Object observation edges
+
Frontier / Long-Term Goal overlay
```

并且必须解决：

> 同一个现实物体从不同位置、不同角度被多次识别时，不得轻易变成两个永久节点。

---

## 1.3 问题 C：WebUI 缺少“下一步具体做什么 + 为什么”

当前前端已经支持：

- `GOAL_SELECTED`
- `DECISION_RECORDED`
- `next_motion_command`
- `components`
- `reasons`
- candidate list

但是当前 SemanticNavigation V2 的 planner 仍有近似占位逻辑：

```python
ScoredGoal(
    goal=candidates[0],
    score=1.0,
    components={"spatial_v2": 1.0},
    reasons=["SemanticNavigation V2 spatial intent"],
)
```

这意味着前端有显示能力，但后端没有提供真实可解释信息。

本次必须让 planner 输出真实结构化解释：

```text
下一步动作
选中的长期目标
候选排名
分数拆解
目标语义证据
地图/Frontier 证据
预计信息增益
路线代价
访问惩罚
负证据
为什么没选第二名
```

注意：

**不要输出隐藏式 LLM chain-of-thought。**

这里只需要 planner 可验证、可复现的结构化决策依据。

---

# 2. 当前代码基线：执行 AI 必须先核对

截至本计划编写时，当前 `main` 已存在以下实现。执行 AI 开工时必须重新确认，但原则上不应重复造轮子。

## 2.1 WebUI 默认配置

`configs/go2w/autonomous_search_web.yaml`

当前关键配置：

```yaml
search:
  reasoner: semantic_navigation
  backend: go2w_experimental
  rgbd_source: true
  spatial_v2: true
  rtabmap: false
```

本次要把 RTAB-Map 空间能力接通，但不能粗暴地“只把 false 改 true”就算完成。

---

## 2.2 当前 RTAB Provider 缺口

`app/spatial/rtabmap_spatial_provider.py`

当前：

```python
def camera_point_to_spatial(...):
    return None
```

这是本次 P0/P1 级核心缺口。

当前 provider 能读：

- `/rtabmap/map`
- `/rtabmap/odom`

但尚未完成：

```text
D435 optical frame
→ base_link
→ map/odom
```

---

## 2.3 当前语义对象更新位置

`run_semantic_exploration.py`

当前流程里先产生 `localized`，然后：

```python
semantic_map.update(localized, ...)
```

在这之前没有可靠的：

```python
localized[i].map_xyz = ...
```

所以跨视角实体融合无法真正工作。

---

## 2.4 当前 SemanticObjectMap

`app/spatial/semantic_object_map.py`

已有：

- `object_id`
- `map_xyz`
- `camera_xyz`
- `observation_count`
- `first_seen`
- `last_seen`
- `seen_from_places`
- `merge_history`
- `association_score`

这是很好的基础，必须复用。

但是当前：

- `_find_existing()` 主要基于同 label + 空间距离；
- 跨视角只有 map_xyz 才可靠；
- merge 后位置直接被新观测覆盖；
- `observed_scene_graph()` 的 `edges=[]`；
- 缺少实体 lifecycle；
- 缺少同帧一对一 data association；
- 缺少稳定的 position uncertainty。

---

## 2.5 当前 PlaceGraph

`app/spatial/place_graph.py`

已有正确原则：

> 原地旋转不创建新 Place；只有真实位移才建立新 Place。

但是当前 `_resolve_place()` 重点只处理 current place。

本次必须升级成：

> 有可靠 pose 时，全图寻找最近既有 Place，实现 revisit / loop closure 级 Place 去重。

---

## 2.6 当前 LongTermGoalSelector

`app/navigation/long_term_goal_selector.py`

已经有：

- spatial gain
- PSG prior
- travel cost
- visited penalty
- ZERO / PARTIAL / STRONG / VERIFY
- Frontier / Anchor Region / Verify Target

这是本次决策解释与路径融合的核心，应在其上增强，不要重写一套平行 planner。

---

## 2.7 当前 LocalGoalExecutor

`app/navigation/local_goal_executor.py`

当前已经正确分离：

```text
where to explore
vs
how to execute
```

当前 Go2-W：

```text
EXPLORE_FRONTIER
→ rotate
→ short forward
```

未来 metric backend：

```text
→ NAVIGATE_POSE
```

这个分层必须保留。

---

## 2.8 当前 Go2-W backend 能力

`app/navigation/go2w_experimental_backend.py`

当前明确：

```text
supports_global_pose = false
supports_metric_navigation = false
supports_relative_translation = true
supports_relative_rotation = true
```

因此本次不能假装 Go2-W 已拥有完整 Nav2 global navigation。

应该实现：

```text
全局/拓扑高层路线
→ 当前路线第一个局部方向
→ ROTATE_VIEW
→ 短步 RELATIVE_MOVE
→ 新 pose / 新 map
→ replan
```

同时保持：

> 如果将来 backend 支持 metric navigation，直接复用同一个高层 route/goal。

---

# 3. 最终系统架构

最终统一为以下七层。

## Layer 1：Perception

输入：

```text
RGB + Depth + intrinsics
```

输出：

```text
ObjectSpatialObservation
label
bbox
confidence
depth_m
camera_xyz
bearing
```

---

## Layer 2：Spatial Transform

输入：

```text
camera_xyz
camera frame
robot/base pose
TF
```

输出：

```text
map_xyz
spatial_quality
transform provenance
```

---

## Layer 3：Persistent World Model

包含：

```text
Occupancy Map
PlaceGraph
SemanticEntityMap
SemanticEntityGraph
Frontier
Robot pose/history
```

---

## Layer 4：Semantic Goal Reasoning

```text
GoalGraph
×
Observed SceneGraph
×
Persistent SemanticEntityGraph
×
PSG
```

输出：

```text
ZERO / PARTIAL / STRONG / VERIFY
SemanticPrior
```

---

## Layer 5：Route / Long-Term Planning

输入：

```text
frontier
semantic region
known target entity
occupancy map
PlaceGraph
negative memory
```

输出：

```text
ranked spatial intents
route/path cost
selected long-term goal
```

---

## Layer 6：Local Execution

输入：

```text
long-term goal
route first segment
RobotCapabilities
```

输出：

```text
ROTATE_VIEW
RELATIVE_MOVE
NAVIGATE_POSE
```

---

## Layer 7：Explainability + WebUI

每一个 planning cycle 必须产生：

```text
DecisionRecord
MapSnapshot
SemanticGraphSnapshot
Candidates
Route
```

前端只负责展示，不自己“推理为什么”。

---

# 4. Phase 0：先修搜索启动状态机，解决 STARTING 卡死

这是第一优先级。

如果搜索永远停在：

```text
STARTING
TASK_UNDERSTANDING
cycle = 0
```

后面的地图、Decision、Frontier 都不会出现。

---

## 4.1 修改范围

重点检查：

```text
app/manual_web_demo/search_session_service.py
app/manual_web_demo/search_executor.py
scripts/go2w/autonomous_search_worker.py
app/live_robot/search_state_store.py
app/live_robot/explorer_search_adapter.py
app/manual_web_demo/static/search_ui.js
```

---

## 4.2 建立明确 startup stage

不用新增过多复杂状态枚举，但必须在现有 `STARTING` 下公开 stage。

推荐：

```text
SPAWN_WORKER
WORKER_READY
LOAD_PIPELINE
WAIT_RGBD
WAIT_SPATIAL_PROVIDER
START_EXPLORER
RUNNING
```

状态 snapshot 增加：

```json
{
  "startup": {
    "stage": "WAIT_RGBD",
    "stage_started_at": 0,
    "last_progress_at": 0,
    "worker_alive": true,
    "worker_state": "ready",
    "last_worker_message_at": 0,
    "last_error": null
  }
}
```

可以复用 `SEARCH_STATE_CHANGED`，不要求为了这一项新增全新 Event 类型。

---

## 4.3 Subprocess executor 必须正确识别 worker 生命周期

当前子进程启动以后，必须保证：

```text
process spawned
→ ready
→ start command accepted
→ worker thread running
→ explorer session_start
```

任一环节失败必须显式 `FAILED`。

增加：

```text
last_message_at
process_return_code
startup_deadline
```

要求：

- 子进程退出但没有 session_result → 立即发布 error；
- ready 超时 → FAILED；
- start 后长期没有 `worker_status/running` 或 explorer event → FAILED；
- 不允许一直留在 STARTING。

---

## 4.4 worker 必须主动报告进度

`autonomous_search_worker.py` 不要只在外部发 `status` 命令时返回状态。

启动流程中主动 emit：

```json
{"type":"worker_status","status":{"state":"starting","stage":"LOAD_PIPELINE"}}
{"type":"worker_status","status":{"state":"starting","stage":"WAIT_RGBD"}}
{"type":"worker_status","status":{"state":"running","stage":"RUNNING"}}
```

在 import、参数构建、pipeline 初始化、explorer 创建等关键边界发一次。

---

## 4.5 WebUI 行为

STARTING/RUNNING/PAUSED 时：

- “开始搜索”按钮 disabled；
- 不允许第二次请求造成红色 `search already active`；
- 显示当前 session id；
- 显示 startup stage 中文说明。

例如：

```text
正在启动搜索进程
正在等待 D435 RGB-D
正在连接 RTAB-Map
正在初始化 SemanticNavigation
```

若 15~30 秒无进展，显示：

```text
启动异常：
最后阶段 WAIT_RGBD
worker alive = true
last progress = 23.4s ago
```

---

## 4.6 Startup 验收

必须新增测试覆盖：

1. mock 启动后 `STARTING → RUNNING`；
2. worker import exception → FAILED；
3. worker 直接退出 → FAILED；
4. STARTING 时重复 start 不创建第二 session；
5. 前端按钮在 STARTING/RUNNING disabled；
6. 首个 observation 后 cycle >= 1；
7. 刷新 WebUI 不恢复一个已经死亡的 phantom STARTING session。

---

# 5. Phase 1：打通 camera_xyz → map_xyz

这是整个语义地图可靠性的根基。

---

## 5.1 坐标系必须先统一

当前 DepthObjectLocalizer 产生的 D435 optical 坐标语义是：

```text
camera x = 图像右
camera y = 图像下
camera z = 镜头前方
```

ROS `base_link` 通常：

```text
base x = 机器人前
base y = 机器人左
base z = 上
```

因此**绝对不能默认 optical frame 和 base_link 零旋转同轴。**

必须审计：

```text
scripts/go2w/start_rgbd_spatial_stack.sh
scripts/go2w/realsense_rgbd_bridge.py
camera_info.frame_id
TF tree
```

当前脚本中若仍存在：

```text
base_link → d435_color_optical_frame
translation = (0.10, 0, 0.30)
rotation = identity
```

必须修正 frame 建模。

---

## 5.2 推荐 TF 设计

使用：

```text
base_link
→ d435_link
→ d435_color_optical_frame
```

其中：

- `d435_link` 使用 ROS body frame 习惯；
- `d435_color_optical_frame` 使用标准 optical frame；
- translation 可以先复用当前 nominal 值；
- rotation 必须符合 optical convention。

如果 bridge 已经发布正确 optical TF，不重复发布。

启动前使用：

```bash
ros2 run tf2_ros tf2_echo base_link d435_color_optical_frame
```

进行自动 health 检查。

---

## 5.3 RtabmapSpatialProvider 增强

修改：

```text
app/spatial/rtabmap_spatial_provider.py
```

实现：

```python
camera_point_to_spatial(
    xyz_camera,
    pose=None,
    ...
) -> tuple[float, float, float] | None
```

优先级：

### A. TF2 真值路径

优先：

```text
map_frame / odom_frame
← tf2
d435_color_optical_frame
```

直接把点变换到 map frame。

必须处理：

- transform unavailable；
- stale transform；
- frame mismatch；
- timestamp 不可用。

### B. 2D pose + nominal camera extrinsic fallback

若 TF 不可用但存在可靠 robot pose：

```text
p_base = T_base_camera × p_camera
p_world = T_world_base × p_base
```

至少支持平面 yaw。

对于标准 aligned camera：

```text
x_base ≈ z_camera + camera_forward_offset
y_base ≈ -x_camera + camera_left_offset
z_base ≈ -y_camera + camera_height
```

然后：

```text
x_world = pose.x + cos(yaw)*x_base - sin(yaw)*y_base
y_world = pose.y + sin(yaw)*x_base + cos(yaw)*y_base
```

不要把这一 fallback 假装成精确标定。

---

## 5.4 Spatial quality 规则

定义清楚：

```text
RGB_ONLY
CAMERA_LOCAL
RELATIVE_RGBD
METRIC_RGBD
```

推荐：

### `METRIC_RGBD`

满足：

- RGB-D 有效；
- map/odom pose 有效；
- camera→base transform 已知；
- map frame 明确；
- RTAB occupancy map / pose 可用。

### `RELATIVE_RGBD`

满足：

- RGB-D 有效；
- robot relative pose 有效；
- nominal extrinsic 可用；
- 但没有稳定 SLAM map 或 TF quality 较低。

### `CAMERA_LOCAL`

只有 camera_xyz。

---

## 5.5 transform provenance

每个对象必须保存：

```json
{
  "source_frame": "d435_color_optical_frame",
  "target_frame": "map",
  "transform_source": "tf2|nominal_extrinsic",
  "pose_source": "rtabmap_odom",
  "map_revision": 12,
  "timestamp": 0,
  "transform_quality": "METRIC_RGBD"
}
```

这对调试重复节点非常重要。

---

## 5.6 run_semantic_exploration 的顺序必须改

当前不能继续：

```text
localize
→ semantic_map.update
```

必须变成：

```text
localize camera_xyz
→ 获取 spatial pose
→ camera_point_to_spatial
→ 写入 map_xyz
→ 更新 SceneGraph attributes
→ entity association
→ Place/Object graph update
```

伪代码：

```python
localized = depth_localizer.localize(...)

spatial_pose = camera_provider.get_pose()

for obs in localized:
    if obs.camera_xyz is None:
        continue

    map_xyz = camera_provider.camera_point_to_spatial(
        obs.camera_xyz,
        pose=spatial_pose,
    )

    if map_xyz is not None:
        obs.map_xyz = map_xyz
        obs.spatial_quality = camera_provider.quality()
        obs.provenance["map_frame"] = ...
```

然后才：

```python
semantic_map.update_with_associations(...)
```

---

## 5.7 SceneGraph 同步 map_xyz

当前 SceneGraph 节点已有：

```text
depth_m
bearing_deg
camera_xyz
spatial_quality
```

增加：

```text
map_xyz
persistent_object_id
entity_status
association_confidence
```

这样 PSG 和后续 graph matcher 可以引用真实实体。

---

## 5.8 transform 单元测试

新增纯 Python 测试：

### Case 1

机器人：

```text
pose=(0,0,0)
```

camera object：

```text
camera=(0,0,2)
```

期望：

```text
对象约在机器人前方 2m + camera offset
```

### Case 2

机器人 yaw=90°

相机前方对象应映射到世界坐标左/右正确方向。

### Case 3

camera x>0（图像右）

映射到 base y 应为负方向。

### Case 4

TF unavailable

不能错误生成 `METRIC_RGBD`。

---

# 6. Phase 2：升级 PlaceGraph，加入全图 Place 去重与 revisit

当前 PlaceGraph 的“原地旋转不建新节点”是正确的，保留。

但是需要增加：

> 回到旧地点时复用旧 Place。

---

## 6.1 全局 nearest-place association

有 metric/relative pose 时：

```python
nearest = min(all_places, distance(current_pose, place.pose))
```

如果：

```text
distance <= place_merge_distance
```

则：

```text
current_place = nearest existing Place
created = false
revisited = true
```

而不是创建 P17。

---

## 6.2 movement edge

如果：

```text
previous_current_place != resolved_place
```

增加 movement edge：

```text
P8 → P3
```

这就是高层拓扑上的 loop closure / revisit。

必须防止：

- 重复 self-edge；
- 同一 observation 重复插 edge；
- 每帧都重复插相同 edge。

---

## 6.3 Place pose 融合

不要永远保存第一次 pose。

增加：

```text
pose_observation_count
pose_mean
pose_quality
last_pose_update
```

至少做 confidence-weighted / running mean。

yaw 使用 circular mean，不能直接普通平均 359° 与 1°。

---

## 6.4 Place observed objects 必须存 persistent ID

当前 observation 注册时可以继续暂存 label 用于 debug。

但 Place 的核心：

```text
observed_object_ids
```

必须最终存：

```text
obj_001
obj_004
```

而不是：

```text
垃圾桶
桌子
```

增加：

```python
place_graph.attach_objects(place_id, persistent_object_ids)
```

或等效接口。

---

## 6.5 PlaceGraph tests

必须覆盖：

1. 原地旋转不会创建 Place；
2. 位移 > threshold 创建新 Place；
3. 离开 P1 后回到 P1 附近，不创建 P3；
4. revisit 增加 visit_count；
5. P2 → P1 创建回访边；
6. Place observed_object_ids 是 persistent entity id。

---

# 7. Phase 3：SemanticObjectMap → Persistent SemanticEntityMap

这是解决“同一物体两次识别变成两个节点”的核心。

---

# 7.1 保持向后兼容

当前：

```python
SemanticObjectMap.update(...) -> list[str]
```

可能已有测试或调用依赖。

不要直接破坏所有调用。

建议新增：

```python
update_with_associations(...) -> SemanticMapUpdateResult
```

然后旧 `update()` 内部调用新方法，只返回 created ids。

---

## 7.2 新结果结构

建议：

```python
@dataclass
class SemanticAssociation:
    observation_index: int
    source_object_id: str | None
    persistent_object_id: str
    action: str
    association_score: float
    distance_m: float | None
    reasons: list[str]

@dataclass
class SemanticMapUpdateResult:
    created_ids: list[str]
    updated_ids: list[str]
    associations: list[SemanticAssociation]
    rejected_pairs: list[dict]
```

---

# 7.3 实体 lifecycle

每个对象增加：

```text
TENTATIVE
CONFIRMED
STALE
REJECTED（可选）
```

推荐规则：

### TENTATIVE

第一次识别：

```text
observation_count = 1
```

### CONFIRMED

满足其一：

```text
>= 2 个不同 RGB-D frame 的高质量 map observation
```

或 degraded mode：

```text
>= 3 次空间一致 observation
```

### STALE

长时间未观测，不删除。

重新观测后可恢复 CONFIRMED。

---

# 7.4 位置估计不能直接覆盖

当前：

```python
entry.map_xyz = obs.map_xyz
```

必须替换成融合。

最低要求：

```text
running weighted mean
+
position variance / covariance estimate
```

字段建议：

```text
position_mean_xyz
position_variance_xyz
map_observation_count
```

权重可由：

```text
detection confidence
×
spatial quality weight
×
depth quality
```

得到。

推荐：

```text
METRIC_RGBD     1.0
RELATIVE_RGBD   0.6
CAMERA_LOCAL    不参与跨视角 global fusion
```

---

# 7.5 Data Association 不能只看 label

核心原则：

> 几何是主证据，语义是 gate，外观是辅助证据。

---

## 7.5.1 Hard gates

候选 pair 必须先通过：

```text
label/category compatible
map frame compatible
distance < hard max
不是同帧不合理重复 assignment
```

如果没有 map_xyz：

- 只允许同 camera frame；
- 或 same Place + bearing/depth 强一致的弱关联；
- 不允许单凭 label 跨 Place 合并。

---

## 7.5.2 推荐 score

第一版：

```text
association_score =
    0.60 * geometry_score
  + 0.15 * semantic_score
  + 0.10 * place_continuity
  + 0.10 * appearance_score
  + 0.05 * temporal_score
```

如果当前没有可靠 appearance feature：

```text
重新归一化已有项
```

不要为了 appearance 强行引入一个重量级 GPU 模型。

可以使用：

- 已有视觉 embedding；
- crop HSV histogram；
- 轻量 perceptual signature；

但必须作为辅助，不得凌驾于 geometry。

---

## 7.5.3 一帧一对一匹配

同一 frame 内：

```text
一个 persistent entity
最多匹配一个 detection
```

否则两个并排垃圾桶可能都被 merge 到一个 obj。

可以使用：

- Hungarian；
- 或按最低 cost 排序的一对一 greedy；

优先避免新增重型依赖。

---

# 7.6 动态距离阈值

不要固定所有物体都 0.4m。

建议：

```text
small object: 0.20~0.30m
medium object: 0.35~0.50m
large object: 0.50~0.80m
```

如果没有 object size，使用：

```text
base threshold = 0.35m
+
pose/depth uncertainty
```

---

# 7.7 多个同类别物体绝不能被误合并

必须专门测试：

```text
绿色垃圾桶 A @ (2.0, 1.0)
绿色垃圾桶 B @ (3.2, 1.0)
```

必须得到：

```text
obj_001
obj_002
```

而不是一个对象。

再测试：

```text
同一个 A
从 P1 观测 = (2.02, 1.01)
从 P2 观测 = (1.96, 1.04)
```

必须合并为：

```text
obj_001
observation_count = 2
```

---

# 7.8 保留 association debug

每次 merge 保存：

```json
{
  "source_observation_id": "...",
  "persistent_object_id": "obj_001",
  "association_score": 0.87,
  "distance_m": 0.14,
  "geometry_score": 0.91,
  "semantic_score": 1.0,
  "appearance_score": 0.66,
  "decision": "MERGED"
}
```

若创建新 entity：

```text
reason = no candidate passed hard gates
```

以后 WebUI 可调试“为什么这个垃圾桶变成两个节点”。

---

# 8. Phase 4：Semantic Entity Graph

当前 `observed_scene_graph()` 不能再返回：

```json
{
  "nodes": [...],
  "edges": []
}
```

---

## 8.1 Graph node 类型

至少：

```text
PLACE
OBJECT
```

可选：

```text
FRONTIER
TARGET_HYPOTHESIS
```

Frontier 也可以继续作为 overlay，不强制成为 persistent entity。

---

## 8.2 OBJECT node

示例：

```json
{
  "node_id": "obj_004",
  "node_type": "OBJECT",
  "label": "绿色垃圾桶",
  "status": "CONFIRMED",
  "map_xyz": [2.14, 1.32, 0.0],
  "confidence": 0.88,
  "observation_count": 4,
  "seen_from_places": ["P2", "P3"],
  "spatial_quality": "METRIC_RGBD",
  "position_variance": [0.03, 0.05, 0.08],
  "last_seen": 0
}
```

---

## 8.3 PLACE node

示例：

```json
{
  "node_id": "P3",
  "node_type": "PLACE",
  "pose": {...},
  "visit_count": 2,
  "heading_coverage": {...}
}
```

---

## 8.4 必须有的边

### MOVED_TO

```text
P1 → P2
```

来自 PlaceGraph movement edge。

### OBSERVED_FROM

```text
P2 → obj_004
```

属性：

```json
{
  "observation_count": 2,
  "last_seen": 0
}
```

---

## 8.5 可选可靠关系边

只有有充分证据时才建立：

```text
NEAR
LEFT_OF
RIGHT_OF
IN_FRONT_OF
```

第一版只要求：

```text
OBSERVED_FROM
MOVED_TO
```

不要为了“图看起来丰富”制造不可靠关系。

---

## 8.6 Graph schema

建议新模块：

```text
app/spatial/semantic_entity_graph.py
```

统一输出：

```json
{
  "schema_version": "semantic_entity_graph_v1",
  "revision": 18,
  "frame_id": "map",
  "nodes": [],
  "edges": [],
  "current_place_id": "P3"
}
```

---

# 9. Phase 5：把图片识别真正融合进路线规划

这一 Phase 是对问题 1 的最终技术闭环。

目标：

> 不是“识别影响转向”，而是“识别形成世界模型，世界模型改变目标与路线”。

---

# 9.1 新增共享 SemanticRoutePlanner

建议新模块：

```text
app/navigation/semantic_route_planner.py
```

职责：

```text
Spatial Map
+
PlaceGraph
+
SemanticEntityGraph
+
Frontier
+
Selected Long-Term Intent
↓
RoutePlan
```

---

## 9.2 RoutePlan 数据结构

建议：

```python
@dataclass
class RoutePlan:
    route_id: str
    frame_id: str
    target_type: str
    target_id: str | None
    target_position: tuple[float, float] | None
    waypoints: list[tuple[float, float]]
    place_sequence: list[str]
    path_length_m: float | None
    reachable: bool
    planner_source: str
    cost_components: dict[str, float]
    provenance: dict[str, Any]
```

---

# 9.3 Metric map 时使用真实路径代价

如果 RTAB Occupancy Map 可用：

- 不再只用 frontier `distance_m` 作为 travel cost；
- 在 free/occupied grid 上计算 path cost；
- 第一版可以实现轻量 8-connected A*；
- occupied 不允许穿过；
- unknown 可设置高 cost 或禁止；
- 对机器人 footprint 做简单 inflation，或复用已有地图膨胀工具。

输出：

```text
path_length
reachable
waypoints
```

---

# 9.4 Topological fallback

无 occupancy map 时：

```text
PlaceGraph shortest path
```

如果目标是 known object：

```text
robot current Place
→ shortest Place route
→ 最近 observed-from Place
→ local inspect
```

如果目标是 frontier：

```text
relative / current-place frontier
```

---

# 9.5 视觉语义如何进入路线

例：

目标：

```text
找绿色垃圾桶
```

当前地图：

```text
P1: 门
P2: 办公桌
P3: 饮水机
P4: 未知
```

PSG/GoalGraph 认为：

```text
办公区域 / 饮水区与垃圾桶更相关
```

则 Frontier candidate score 应由：

```text
semantic relevance
+
PSG prior
+
information gain
+
novelty
-
path cost
-
visited penalty
-
negative evidence
```

共同决定。

这就是本次要求的“图片识别路线规划融合”。

---

# 9.6 LongTermGoalSelector 增强

当前已有：

```text
spatial_gain
psg
travel
visited
```

增加：

```text
semantic_relevance
route_cost
route_reachability
negative_evidence
continuity_bonus
target_proximity
```

推荐但不强制固定权重：

```text
score =
    + 0.30 semantic_relevance
    + 0.25 spatial_gain
    + 0.20 psg_prior
    + 0.10 novelty
    + 0.05 continuity
    - 0.20 normalized_route_cost
    - 0.15 visited_penalty
    - 0.20 negative_evidence_penalty
```

注意：

- 权重做 config；
- 不要把 magic number 散落在代码；
- 保存 score breakdown。

---

# 9.7 Go2-W relative backend 的执行方式

即使 RoutePlan 是 metric/global，也不能因为 backend 不支持 NAVIGATE_POSE 就失败。

当前 Go2-W：

```text
RoutePlan
→ 取第一段方向
→ LocalGoalExecutor
→ rotate
→ short forward
→ stop
→ observe
→ update map
→ replan
```

这叫 receding-horizon execution。

将来：

```text
supports_metric_navigation=true
```

则：

```text
RoutePlan
→ NAVIGATE_POSE
```

---

# 9.8 离线视频 pipeline 的统一方式

不要直接调用整个：

```python
run_video_navigation_planning()
```

进入真机。

应该让离线 pipeline 的最终地图/轨迹能够适配成：

```text
SpatialMapSnapshot
PlaceGraph
SemanticEntityGraph
```

如果合理，可新增 adapter：

```text
app/navigation/video_navigation_spatial_adapter.py
```

这样：

```text
离线视频
实时 RTAB
```

最终都喂给同一个：

```text
SemanticRoutePlanner
LongTermGoalSelector
```

这样才算技术路线真正统一。

---

# 10. Phase 6：DecisionRecord 变成一级数据模型

不得继续用：

```text
components={"spatial_v2": 1.0}
```

冒充决策解释。

---

## 10.1 新建结构化模型

建议：

```text
app/navigation/decision_record.py
```

或在现有 models 中加入。

数据结构：

```json
{
  "decision_id": "D00017",
  "cycle": 5,
  "timestamp": 0,
  "map_revision": 31,

  "match_state": "PARTIAL",

  "selected_intent": {
    "intent_type": "EXPLORE_FRONTIER",
    "target_frontier_id": "frontier_03"
  },

  "selected_goal": {
    "goal_id": "local_003",
    "goal_type": "ROTATE_VIEW"
  },

  "next_motion_command": {
    "type": "ROTATE_VIEW",
    "yaw_deg": 30,
    "instruction_zh": "向右转 30°"
  },

  "reason_code": "EXPLORE_SEMANTIC_FRONTIER",

  "reason_zh": "目标尚未确认。P3 东侧前沿靠近已确认的办公桌语义锚点，预计信息增益较高，路线可达且代价低于其他候选，因此选择该方向。",

  "score": 0.73,

  "score_breakdown": {
    "semantic_relevance": 0.81,
    "spatial_gain": 0.67,
    "psg_prior": 0.52,
    "novelty": 0.74,
    "route_cost_penalty": 0.21,
    "visited_penalty": 0.00,
    "negative_evidence_penalty": 0.00
  },

  "evidence": {
    "anchor_object_ids": ["obj_003"],
    "anchor_labels": ["办公桌"],
    "current_place_id": "P3",
    "spatial_quality": "METRIC_RGBD",
    "route_id": "route_005"
  },

  "alternatives": [
    {
      "candidate_id": "frontier_01",
      "score": 0.48,
      "rejected_reason_zh": "语义相关度较低且路线更长"
    }
  ]
}
```

---

# 10.2 reason_zh 必须来自结构化规则

不要让 LLM自由生成无法复现的原因。

推荐：

```python
build_reason_zh(
    match_state,
    selected,
    score_components,
    evidence,
    alternatives,
)
```

规则模板即可。

例如：

### ZERO

```text
当前没有发现目标或可靠锚点，因此优先探索未访问且信息增益最高的前沿。
```

### PARTIAL

```text
已发现与目标相关的锚点“饮水机”，因此优先探索其附近尚未覆盖区域。
```

### STRONG

```text
当前检测到高置信目标候选，下一步转向并接近该候选进行视觉确认。
```

---

# 10.3 Alternatives 必须真实

至少保存 top 3。

不能所有候选都写固定原因。

每个 rejected candidate 至少对比：

```text
score delta
最大的负面项
```

例如：

```text
F2 比 F3 低 0.18，主要因为已访问惩罚 0.20。
```

---

# 10.4 planner 接线

修改 `run_semantic_exploration.py`。

不能再：

```python
return ScoredGoal(
    goal=candidates[0],
    score=1.0,
    components={"spatial_v2": 1.0},
    ...
)
```

必须把：

```text
LongTermGoalSelector ScoredIntent
RoutePlan
LocalGoalExecutor goal
```

关联起来。

可在 spatial candidate generator 把 planning context 放入：

```text
state["last_spatial_plan"]
```

然后 planner 返回真实：

```text
score
components
reasons
```

并建立 DecisionRecord。

---

# 11. Phase 7：实时 spatial event 必须完整

当前已有事件词汇：

```text
SPATIAL_POSE_UPDATED
SPATIAL_MAP_UPDATED
FRONTIERS_UPDATED
PLACE_CREATED
PLACE_UPDATED
SEMANTIC_OBJECT_LOCALIZED
PSG_PRIOR_UPDATED
LONG_TERM_GOAL_SELECTED
LOCAL_GOAL_PROGRESS
DECISION_RECORDED
```

本次要保证每 cycle 真正发出来。

---

## 11.1 observation 后

至少发送：

```text
RGBD_FRAME_UPDATED
SPATIAL_POSE_UPDATED
SEMANTIC_OBJECT_LOCALIZED
PLACE_UPDATED
```

如 map revision 改变：

```text
SPATIAL_MAP_UPDATED
```

如有新 frontier：

```text
FRONTIERS_UPDATED
```

---

## 11.2 entity association 后

事件 payload 至少包含：

```json
{
  "persistent_object_id": "obj_004",
  "action": "MERGED",
  "association_score": 0.88,
  "map_xyz": [2.1, 1.2, 0.0],
  "status": "CONFIRMED"
}
```

如果不想新增 event type，可继续复用 `SEMANTIC_OBJECT_LOCALIZED`，但 payload 要完整。

---

## 11.3 spatial snapshot 增加

在 `SearchStateStore` 的：

```text
spatial
```

加入：

```json
{
  "semantic_graph": null,
  "route_plan": null,
  "map_health": {},
  "association_debug": []
}
```

---

# 12. Phase 8：WebUI 地图升级

现在 `SearchMapRenderer` 已经有：

- Place；
- occupancy；
- frontier；
- semantic object；
- PSG region；
- long-term goal。

本次不要重写框架，直接修数据和尺度。

---

## 12.1 必须显示

地图区域至少显示：

```text
灰/黑：free / unknown
红：occupied
蓝箭头：robot pose
线：robot trajectory / Place movement
圆点：Place
菱形：Persistent Object
黄圈：Frontier
紫色区域：PSG region
星号：selected long-term goal
高亮路径：RoutePlan
```

---

## 12.2 Object 节点

只显示：

```text
CONFIRMED
```

为主节点。

`TENTATIVE`：

- 可以显示半透明；
- 必须视觉上和 confirmed 区分。

Object label：

```text
obj_004
绿色垃圾桶
obs=4
```

点击显示：

```text
status
map_xyz
confidence
observation_count
seen_from_places
association_score
spatial_quality
last_seen
```

---

## 12.3 Place ↔ Object 边

显示 `OBSERVED_FROM` 边。

为了避免图太乱：

- 默认细线；
- hover/select object 时高亮相关 Place；
- 可提供“隐藏语义边”开关。

---

## 12.4 Route path

新增 route overlay：

```text
robot
→ waypoint 1
→ waypoint 2
→ selected frontier
```

如果当前 backend relative-only：

UI 仍显示 global/high-level route，但标注：

```text
执行模式：短步重规划
```

---

## 12.5 地图尺度修复

当前 renderer 的 fallback layout 使用几十像素级坐标，而 metric map 使用米。

必须明确两种 mode：

### metric mode

所有：

```text
Place
Object
Robot
Frontier
Route
Occupancy
```

统一米坐标。

grid step 推荐：

```text
0.5m / 1.0m
```

不要继续固定 `50` 当作 metric grid step。

### topology mode

无 metric pose 才使用 synthetic layout。

不要把两种坐标混在一个 viewBox 里。

---

## 12.6 自动 fit bounds

fit bounds 必须考虑：

```text
places
objects
robot
frontiers
route
occupancy bounds
```

而不是只有 Place 和 robot。

---

# 13. Phase 9：WebUI 下一步决策区

当前右侧“下一步决策”必须最终显示成真正可读内容。

---

## 13.1 顶部：下一步具体动作

例：

```text
下一步：
向右转 30°
```

下一行：

```text
随后：前进 0.25m 后重新观察
```

如果 metric backend：

```text
导航到 F3
坐标 (2.42m, 1.18m)
```

---

## 13.2 为什么

必须显示：

```text
原因：
目标尚未确认；
P3 发现“办公桌”语义锚点；
F3 未访问且信息增益 0.67；
到 F3 的路径约 1.8m；
综合得分 0.73，为当前最高。
```

---

## 13.3 score breakdown

显示真实：

```text
语义相关度       +0.81
空间信息增益     +0.67
PSG 先验         +0.52
新颖度           +0.74
路径代价         -0.21
已访问惩罚       -0.00
负证据惩罚       -0.00
综合             0.73
```

---

## 13.4 alternatives

显示：

```text
候选 2：F2  0.48
未选原因：该区域已探索，且路线比 F3 长 0.9m

候选 3：P1 revisit  0.31
未选原因：已有两次负证据
```

---

## 13.5 Decision history

保留每 cycle：

```text
D1 → rotate +30
D2 → forward 0.25
D3 → F2
D4 → inspect obj_004
```

点击能看当时：

```text
map revision
evidence
score
route
```

---

# 14. Phase 10：WebUI 搜索状态不再显示“假空白”

当前在 `TASK_UNDERSTANDING / STARTING` 时右侧容易显示：

```text
等待决策...
Candidates 空
Decision history 空
```

改成阶段感知 UI。

### STARTING

```text
尚未进入规划阶段
当前：等待 RGB-D / Reasoner 初始化
```

### OBSERVE

```text
正在观察环境
```

### MATCH

```text
正在匹配目标与场景
```

### PLAN

才显示：

```text
候选与下一步决策
```

这样用户能明确分辨：

> “现在还没规划”

和：

> “planner 没数据”。

---

# 15. Phase 11：配置收敛

修改：

```text
configs/go2w/autonomous_search_web.yaml
```

增加可调空间配置。

建议：

```yaml
spatial:
  provider: auto

  rtabmap:
    enabled: true
    map_topic: /rtabmap/map
    odom_topic: /rtabmap/odom
    map_frame: map
    camera_frame: d435_color_optical_frame
    base_frame: base_link

  camera_extrinsic:
    mode: tf2_then_nominal
    nominal_translation_m: [0.10, 0.0, 0.30]

  place_graph:
    merge_distance_m: 0.35
    relocation_min_displacement_m: 0.10

  entity_map:
    base_merge_distance_m: 0.35
    confirm_min_observations: 2
    stale_after_seconds: 120
    geometry_weight: 0.60
    semantic_weight: 0.15
    place_weight: 0.10
    appearance_weight: 0.10
    temporal_weight: 0.05

  route_planner:
    mode: auto
    allow_unknown: false
    inflation_radius_m: 0.25
    max_waypoints: 32

  decision:
    keep_alternatives: 3
```

兼容旧：

```text
search.rtabmap
search.spatial_v2
```

旧字段不能立刻删除。

---

# 16. Phase 12：运行目录与可回放数据

每 session 必须落盘：

```text
outputs/live_runs/<session_id>/
```

至少：

```text
task.json
summary.json
events.jsonl
place_graph.json
semantic_entity_map.json
semantic_entity_graph.json
spatial_map_summary.json
decisions.jsonl
routes.jsonl
```

不要每帧保存巨型 occupancy JSON 导致磁盘爆炸。

可以：

- 只保存 map metadata/revision；
- 关键 revision 存压缩 snapshot；
- event 中只引用 revision。

---

# 17. Phase 13：Summary 增强

`summary.json` 增加：

```json
{
  "unique_places": 6,
  "unique_objects": 14,
  "confirmed_objects": 8,
  "tentative_objects": 6,
  "entity_merges": 19,
  "entity_creations": 14,
  "revisited_places": 2,
  "map_revisions": 31,
  "frontiers_discovered": 11,
  "route_plans": 8,
  "decision_count": 8,
  "mean_route_cost_m": 1.83
}
```

方便后续论文与实验。

---

# 18. 新增/修改文件建议

执行 AI 先审计，实际名称可按现有风格微调，但最终功能必须覆盖。

---

## 18.1 重点修改

```text
scripts/go2w/run_semantic_exploration.py

app/spatial/rtabmap_spatial_provider.py
app/spatial/place_graph.py
app/spatial/semantic_object_map.py
app/spatial/models.py

app/navigation/long_term_goal_selector.py
app/navigation/local_goal_executor.py

app/live_robot/explorer_search_adapter.py
app/live_robot/search_state_store.py

app/manual_web_demo/search_session_service.py
app/manual_web_demo/search_executor.py
app/manual_web_demo/static/search_ui.js
app/manual_web_demo/static/search_map.js

scripts/go2w/autonomous_search_worker.py
scripts/go2w/start_rgbd_spatial_stack.sh

configs/go2w/autonomous_search_web.yaml
```

---

## 18.2 推荐新增

```text
app/spatial/semantic_entity_graph.py
app/spatial/spatial_transform.py

app/navigation/semantic_route_planner.py
app/navigation/decision_record.py
```

可根据现有包结构合并，但不要把所有逻辑都塞回 `run_semantic_exploration.py`。

---

# 19. 测试计划

必须新增以下类别测试。

---

## 19.1 Spatial transform

建议：

```text
tests/test_spatial_transform.py
tests/test_rtabmap_spatial_provider_transform.py
```

覆盖：

- optical → base axis；
- yaw；
- translation；
- quality；
- no TF fallback。

---

## 19.2 Place Graph

```text
tests/test_place_graph_revisit.py
```

覆盖：

- rotate no new place；
- relocate new place；
- return to existing place；
- loop edge；
- pose fusion。

---

## 19.3 Entity Association

```text
tests/test_semantic_entity_association.py
```

至少 8 个 case：

1. 同 object 两 frame，位置近 → merge；
2. 两个同 label，位置远 → separate；
3. 两个同 label，位置比较近但同帧 → one-to-one，不 collapse；
4. 无 map_xyz 跨 place → 不盲目 merge；
5. 同 camera frame camera_xyz 近 → 可 merge；
6. position weighted fusion；
7. tentative → confirmed；
8. stale → reobserved confirmed。

---

## 19.4 Semantic Graph

```text
tests/test_semantic_entity_graph.py
```

覆盖：

```text
PLACE nodes
OBJECT nodes
MOVED_TO
OBSERVED_FROM
stable IDs
```

---

## 19.5 Route planner

```text
tests/test_semantic_route_planner.py
```

覆盖：

1. grid A* 绕 obstacle；
2. unreachable；
3. shorter geometric route；
4. semantic candidate 虽远一点但综合分更高；
5. topological fallback；
6. known object → observed Place route。

---

## 19.6 Decision Record

```text
tests/test_decision_record.py
```

覆盖：

```text
selected candidate
score breakdown
alternative reason
next motion
reason_zh
map revision
```

不得出现固定：

```text
spatial_v2 = 1.0
```

---

## 19.7 Startup lifecycle

```text
tests/test_search_startup_lifecycle.py
```

覆盖：

```text
STARTING → RUNNING
STARTING → FAILED
dead subprocess
repeat start
worker ready
```

---

## 19.8 Web contract

```text
tests/test_search_spatial_web_contract.py
```

检查 `/api/search/state` / WebSocket snapshot 至少包含：

```text
spatial.semantic_graph
spatial.route_plan
last_decision
next_motion_command
```

---

# 20. Mock E2E 必须增强

当前 `--mock` 不能只测试“找到目标”。

新增 deterministic spatial mock scene。

例如：

```text
P1
  desk @ map(1.0, 0.5)

P2
  green trash bin @ map(2.0, 0.5)

frontiers
  F1 @ (1.8,0.0)
  F2 @ (0.0,2.0)
```

目标：

```text
绿色垃圾桶
```

E2E 应观察：

```text
Session RUNNING
→ Place P1
→ object desk created
→ F1 semantic score > F2
→ DecisionRecord 选择 F1
→ local goal generated
→ next observation
→ trash bin obj created
→ target confirmed
```

WebUI mock 必须能完整显示：

```text
地图
对象
Frontier
路线
Decision
```

不依赖真机。

---

# 21. 真机运行流程

修改完成以后提供一键验证顺序。

---

## 21.1 先健康检查

```bash
bash scripts/go2w/check_go2w_ready.sh --json
```

---

## 21.2 启动 RGB-D + RTAB

```bash
bash scripts/go2w/start_rgbd_spatial_stack.sh start
```

自动检查：

```text
D435 RGB
D435 Depth
CameraInfo
TF base↔camera
RTAB map
RTAB pose
```

---

## 21.3 dry-run

```bash
/usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
  --target "绿色垃圾桶" \
  --backend go2w_experimental \
  --reasoner semantic_navigation \
  --rgbd-source \
  --spatial-v2 \
  --rtabmap \
  --dry-run-motion \
  --max-planning-cycles 5 \
  --max-motion-steps 5
```

必须看到 event：

```text
spatial_pose_updated
semantic_object_localized with map_xyz
place_created
frontiers_updated
long_term_goal_selected
decision_recorded
```

---

## 21.4 WebUI

```bash
bash scripts/go2w/start_autonomous_search_web.sh --enable-autonomous-motion
```

启动后：

```text
rtabmap enabled
```

应明确显示在系统状态。

---

# 22. 真机验收场景

至少做三类。

---

## 场景 A：同一物体重复观测

把机器人在正常搜索中从两个角度看到同一个垃圾桶。

验收：

```text
第一次：
obj_001 TENTATIVE

第二次：
仍然 obj_001
observation_count >= 2
status = CONFIRMED
```

不允许：

```text
obj_001
obj_002
```

同时存在且坐标重合。

---

## 场景 B：两个同类别物体

如果视野中真实存在两个同类垃圾桶：

必须形成两个节点。

不能为了“去重”强行合并。

---

## 场景 C：地图与决策联动

目标不可见时：

WebUI 必须出现：

```text
P1/P2...
frontiers
selected route
next action
why
```

机器人每短步后：

```text
robot pose 移动
map revision 增加
Place/Entity graph 更新
route 重算
decision id 增加
```

---

# 23. 数据可靠性原则

本项目地图必须明确区分“事实”和“预测”。

---

## Observed Fact

```text
RTAB map
robot pose
detected object
depth
map_xyz
Place
observation relation
```

---

## Inferred / Predicted

```text
PSG region
semantic prior
predicted target area
```

UI 必须用不同 visual style。

不能把 PSG 预测区域当成已经识别的物体节点。

---

# 24. 禁止的错误实现

执行 AI 不得采用以下方式“快速完成”。

---

## 禁止 1

只把：

```yaml
rtabmap: false
```

改成：

```yaml
true
```

然后声称地图完成。

---

## 禁止 2

对象只按：

```text
label 相同
```

就 merge。

---

## 禁止 3

完全不做 map_xyz，继续使用 camera_xyz 跨视角比较。

---

## 禁止 4

把每个 detection / 每帧都永久保存成 graph node。

---

## 禁止 5

只在前端伪造：

```text
下一步：向右
原因：AI认为右边更好
```

---

## 禁止 6

planner 继续返回：

```text
score=1
spatial_v2=1
```

---

## 禁止 7

为了做全局路线，直接让当前 `go2w_experimental` 假装支持：

```text
NAVIGATE_POSE
```

---

## 禁止 8

删除旧的：

```text
offline video navigation
mock
relative mode
camera-local fallback
```

---

## 禁止 9

因为 TF 不可用就整个搜索 crash。

应该 degraded。

---

# 25. 代码质量要求

1. 空间变换计算放独立函数，方便纯 Python 测试。
2. Entity association 不要写成 300 行一个函数。
3. Planner score 权重从 config 读取。
4. 所有 public snapshot JSON-safe。
5. 前后端字段统一 snake_case。
6. 所有 timestamp 明确单位。
7. map frame 明确保存。
8. 每个 persistent object 有 stable id。
9. 所有 graph revision 单调增加。
10. 不允许前端根据 label 自己“推断 persistent id”。

---

# 26. 性能要求

目标不是每帧都跑完整重规划。

建议：

```text
RGB-D 采集：持续
Semantic perception：沿用当前节流
Object map update：每次有效 perception
Occupancy update：跟 RTAB revision
Route replan：
  - 新 planning cycle
  - route invalid
  - navigation failed
  - significant map revision
  - new strong semantic evidence
```

Entity association 对象量通常较小。

第一版 O(N×M) + one-to-one greedy 足够，不要过早引入数据库。

---

# 27. 安全要求

本次所有真机运动仍必须遵守：

```text
/go2w/motion
/go2w/arm
/go2w/emergency_stop
Operator supervision
max turn
max forward
stop after motion
```

SemanticRoutePlanner 只能决定高层目标。

不能绕过底层安全。

---

# 28. Definition of Done

只有以下全部满足才能宣布完成。

---

## DoD 1：搜索状态

WebUI 点击一次开始后：

```text
STARTING
→ RUNNING
→ OBSERVE
```

不能长期停在 STARTING。

---

## DoD 2：map_xyz

至少一个 RGB-D object event 能看到：

```json
{
  "camera_xyz": [...],
  "map_xyz": [...],
  "spatial_quality": "METRIC_RGBD"
}
```

RTAB 不可用时明确降级。

---

## DoD 3：Place 去重

机器人离开后回到旧位置：

```text
复用旧 Place
```

---

## DoD 4：Object 去重

同一现实物体跨两次观测：

```text
persistent_object_id 不变
```

---

## DoD 5：不同物体不误合并

两个同类但不同位置实体：

```text
两个 persistent object
```

---

## DoD 6：Semantic Graph

Web API / snapshot 存在：

```text
PLACE
OBJECT
MOVED_TO
OBSERVED_FROM
```

---

## DoD 7：WebUI Map

用户能实时看到：

```text
Occupancy
Robot
Places
Objects
Frontiers
Route
Selected Goal
```

---

## DoD 8：视觉参与路线

至少一个 mock/E2E 测试证明：

```text
改变语义 evidence
→ selected frontier/route 改变
```

而不是只由距离决定。

---

## DoD 9：Decision

WebUI 显示：

```text
下一步动作
原因
score breakdown
alternatives
```

---

## DoD 10：Planner 非占位

代码中不再以：

```text
{"spatial_v2": 1.0}
```

作为 V2 真实评分。

---

## DoD 11：Route

Spatial snapshot 中有：

```text
route_plan
```

包含：

```text
target
waypoints/place_sequence
reachable
path cost
```

---

## DoD 12：测试

新增测试全部通过。

现有相关测试不得回归。

---

# 29. 执行顺序：不要乱序

严格按以下顺序完成。

```text
STEP 1
审计 current tree + existing tests

STEP 2
修 STARTING lifecycle

STEP 3
实现 spatial transform

STEP 4
把 map_xyz 接入 run_semantic_exploration

STEP 5
升级 PlaceGraph revisit

STEP 6
升级 SemanticObjectMap entity association

STEP 7
生成 SemanticEntityGraph

STEP 8
实现 SemanticRoutePlanner

STEP 9
LongTermGoalSelector 接入 route cost + semantic score

STEP 10
生成真实 DecisionRecord

STEP 11
Explorer/SearchEvent/StateStore 全链路透传

STEP 12
WebUI metric map / object graph / route / decision UI

STEP 13
mock E2E

STEP 14
pytest regression

STEP 15
真机 dry-run instructions + health checks

STEP 16
更新 README/docs
```

不要先大改 WebUI，然后再发现后端没有数据。

---

# 30. 执行 AI 的最终输出格式

修改完成后，不要只说“完成”。

必须输出：

## A. 修改文件清单

```text
modified:
...
new:
...
```

## B. 核心链路说明

明确回答：

```text
camera_xyz 如何变 map_xyz
object 如何去重
route 如何生成
DecisionRecord 如何产生
```

## C. 测试结果

例如：

```text
pytest ...
XX passed
```

失败的测试必须解释，不得隐藏。

## D. 真机启动命令

给出从：

```text
health
→ spatial stack
→ web
```

完整命令。

## E. 已知 degraded behavior

例如：

```text
无 RTAB → RELATIVE_RGBD
无 TF → nominal extrinsic
无 metric backend → short-step receding horizon
```

---

# 31. 最终目标状态示例

用户在 WebUI 输入：

```text
找到绿色垃圾桶
```

页面应逐步变成：

```text
搜索状态
RUNNING
Phase: PLAN
Cycle: 4

地图
Robot @ P3
P1 -- P2 -- P3
          │
          ├─ obj_002 办公桌
          └─ obj_004 绿色垃圾桶候选

obj_002:
CONFIRMED
obs=3

obj_004:
TENTATIVE
obs=1

Frontier:
F1 score 0.31
F2 score 0.74  ← selected

Route:
P3 → F2
path = 1.6m

下一步：
右转 24°

随后：
前进 0.25m，然后重新观察

为什么：
当前目标尚未确认；
P3 附近已发现办公桌语义锚点；
F2 位于该锚点相关区域且尚未探索；
F2 信息增益 0.69；
路径长度 1.6m；
综合评分 0.74，高于 F1 的 0.31。

Decision:
D00004

执行后：

robot pose 更新
↓
map revision +1
↓
Place / Entity graph 更新
↓
route replan
↓
D00005
```

最终再次看到同一个垃圾桶：

```text
obj_004
observation_count: 2
TENTATIVE → CONFIRMED
```

而不是再新增一个：

```text
obj_005
```

这才算本次修改完成。

---

# 32. 一句话总原则

这次重构的核心不是“加一个 SLAM 页面”，而是把：

```text
视觉识别结果
```

从一次性的 detection，升级成：

```text
有世界坐标
有稳定身份
可重复融合
可被路线规划使用
可被 planner 解释
可被 WebUI 实时观察
```

的 **Persistent Spatial Semantic World Model**。

最终必须做到：

> **机器人看到的物体，真正成为地图中的长期实体；地图中的长期实体，真正参与下一步路线选择；路线选择的原因，真正以结构化、可验证的形式显示在 WebUI。**
