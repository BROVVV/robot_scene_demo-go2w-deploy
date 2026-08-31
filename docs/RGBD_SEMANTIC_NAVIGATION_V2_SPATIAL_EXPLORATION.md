# RGB-D SemanticNavigation V2 Spatial Exploration

> 本文档描述 `robot_scene_demo` 从 2D next-view 搜索升级为 RGB-D 空间语义探索的
> 模块化设计与当前实现状态。计划书：
> `robot_scene_demo_RGBD_SemanticNavigationV2_PSG_空间自主探索_一次性重构实施计划书_20260818.md`

## 1. 目标

最终行为：

```text
D435 RGB-D → 3D Semantic SceneGraph → Spatial Map → Frontier → PSG Prior
→ SemanticNavigation V2 Long-Term Spatial Goal → LocalGoalExecutor → Go2-W 短步平移
→ 新 Place → Replan → TARGET_FOUND
```

核心原则：

- `heading sector` 只代表 Local View Coverage，不再冒充 Frontier。
- 原地旋转只更新当前 Place 的 `heading_coverage`，不创建新 Place。
- `RELATIVE_MOVE` / `ROTATE_VIEW` 是 LocalGoalExecutor primitive，不是一级探索候选。
- PSG 是 Semantic Prior Provider，不能直接输出动作，也不能把预测写入 Observed SceneGraph。

## 2. 模块

### 2.1 D435 原子 RGB-D 服务

机器狗侧 `scripts/go2w/realsense_stream.py` 新增：

```text
GET /rgbd/latest.json
GET /rgbd/frame/<id>/color.jpg
GET /rgbd/frame/<id>/depth.png
```

`color.jpg` 与 `depth.png` 来自同一个 RealSense frameset，缓存最近 32 帧，
`frame_id` 原子标识。深度保留 16-bit PNG，`depth_unit_m=0.001`。

### 2.2 工作站 RGB-D Source

- `app/perception/rgbd_source.py`：`RGBDFrame` / `RGBDFrameBundle` / `RGBDSource` 协议。
- `app/perception/realsense_http_rgbd_source.py`：通过 HTTP 原子端点下载 color/depth 到本地缓存。

### 2.3 Depth Object Localizer

- `app/perception/depth_object_localizer.py`：`DepthObjectLocalizer`
  - 输入 bbox/mask + aligned depth + intrinsics；
  - 输出 `depth_m` / `camera_xyz` / `bearing_deg` / `spatial_quality`；
  - 采样使用 bbox 中心区域 + 有效深度过滤 + MAD 异常剔除 + median。

### 2.4 Spatial Models / Provider / Frontier / PlaceGraph

- `app/spatial/models.py`：`SpatialPose` / `SpatialMapSnapshot` / `FrontierCandidate` /
  `PlaceNode` / `MovementEdge` / `SemanticRegion` / `ExplorationIntent`。
- `app/spatial/spatial_provider.py`：`SpatialProvider` 协议。
- `app/spatial/camera_local_spatial_provider.py`：无地图时的 `CAMERA_LOCAL` fallback，
  输出相对 Frontier（bearing + relocate_distance）。
- `app/spatial/frontier_extractor.py`：从 free/unknown 边界提取真正 Frontier。
- `app/spatial/place_graph.py`：`PlaceGraph`，平移才创建 Place，旋转只更新 heading coverage。
- `app/spatial/semantic_object_map.py`：观测到的 3D 物体事实图，不混入 PSG。
- `app/spatial/spatial_memory.py`：Frontier / Region 负证据与 blacklist。
- `app/spatial/lightweight_depth_bev.py`：RTAB-Map 不可用时的轻量 BEV fallback。

### 2.5 PSG Semantic Prior

- `app/reasoning/semantic_navigation/semantic_prior_provider.py`：`RuleSemanticPriorProvider`
  - 输入 GoalGraph + Observed SceneGraph + SpatialContext；
  - 输出 `SemanticPrior`（predicted nodes / relations / regions / frontier_scores）；
  - 预测与观测严格分离。

### 2.6 SemanticNavigation V2 Spatial Reasoner

- `app/reasoning/semantic_navigation/spatial_reasoner.py`：`SpatialSearchReasoner`
  - ZERO → `EXPLORE_FRONTIER`
  - PARTIAL → `INSPECT_ANCHOR_REGION`
  - STRONG / VERIFY → `APPROACH_TARGET` / `VERIFY_TARGET`

### 2.7 Long-Term Goal Selector

- `app/navigation/long_term_goal_selector.py`：`LongTermGoalSelector`
  - 打分：`spatial_gain + goal_graph_relevance + psg_prior - travel_cost - visited_penalty`
  - 输出可解释的 `ScoredIntent`。

### 2.8 Local Goal Executor

- `app/navigation/local_goal_executor.py`：`LocalGoalExecutor`
  - `EXPLORE_FRONTIER` → ROTATE_VIEW → RELATIVE_MOVE（或 NAVIGATE_POSE）。
  - `turn_only=true` 时只旋转不前进。

### 2.8.5 RTAB-Map / D435 ROS2 Bridge

- `scripts/go2w/realsense_rgbd_bridge.py`：把 D435 HTTP 原子帧发布为
  `/go2w/d435/color/image_raw`、`/go2w/d435/depth/image_rect_raw`、
  `/go2w/d435/color/camera_info`、`/go2w/d435/rgbd_health`。
- `ros-humble-rtabmap-ros` 已安装。
- `RtabmapSpatialProvider` 订阅 `/rtabmap/map` 与 `/rtabmap/odom`，
  RTAB-Map 不可用时自动降级到 `CameraLocalSpatialProvider` / `LightweightDepthBEVMapper`。

### 2.9 V2 真机主循环接入

`run_semantic_exploration.py --spatial-v2` 在保留旧 `AutonomousExplorer` 外壳的前提下：

- 观察阶段：D435 原子 RGB-D → DepthObjectLocalizer → 更新 `PlaceGraph` / `SemanticObjectMap`
- 规划阶段：`SpatialSearchReasoner` 选择 `ExplorationIntent`（Frontier / Anchor Region / Target）
- 执行阶段：`LocalGoalExecutor` 将 intent 分解为 `ROTATE_VIEW + RELATIVE_MOVE` primitive
- `--dry-run-motion` 可无运动验证整条链路

### 2.10 WebUI / SearchEvent

- `SearchEvent` 新增 `RGBD_FRAME_UPDATED` / `SPATIAL_POSE_UPDATED` /
  `SPATIAL_MAP_UPDATED` / `FRONTIERS_UPDATED` / `PLACE_CREATED` /
  `SEMANTIC_OBJECT_LOCALIZED` / `PSG_PRIOR_UPDATED` / `LONG_TERM_GOAL_SELECTED` 等。
- `SearchStateStore` 维护 `spatial` 快照。
- REST：
  - `GET /api/search/spatial-map`
  - `GET /api/search/place-graph`
  - `GET /api/search/frontiers`
  - `GET /api/search/semantic-map`
- 前端：
  - 自主搜索 Tab 相机已切换到 D435：`/api/d435.mjpeg`
  - SVG 地图支持 PlaceGraph / Frontiers / Semantic Objects / PSG Region / Long-Term Goal 叠加

## 3. 降级矩阵

| D435 | SpatialProvider | 行为 |
| --- | --- | --- |
| online + RTAB-Map good | METRIC/RELATIVE | 真正 Frontier + PlaceGraph |
| online + RTAB-Map lost | CAMERA_LOCAL | RelativeFrontier fallback，仍可移动 |
| depth invalid | RGB_ONLY | 旧 2D 路径 |
| PSG unavailable | geometry frontier | 无 PSG 分数，仍可探索 |

## 4. 真机验证（2026-08-18）

- `check_go2w_ready.sh` → `state=ready`
- D435 `/rgbd/latest.json` PASS，color/depth 同 `frame_id`
- `validate_rgbd_spatial_stack.py` PASS，depth_valid_fraction=0.937
- 小范围运动：`run_autonomous_loop.py --pattern r10 --turn-only --operator-authorized-rotation`
  → 10° 转向成功，odom 验证 `yaw_delta_rad≈0.1577`（≈9.02°）
- RTAB-Map + D435 Bridge：
  - `/rtabmap/map` 可接收（60×45 OccupancyGrid）
  - 当前静态场景 RTAB-Map 未产生 free 栅格，系统自动降级到 `LightweightDepthBEVMapper`
- V2 真机运动验证：
  - 目标“不存在的红色独角兽” → `ZERO`
  - V2 选择 metric frontier `frontier_03_-234_-168`
  - `LocalGoalExecutor` 执行 `ROTATE_VIEW +30°`
  - 真实转向成功，odom 验证 `yaw_delta_deg=29.309`

## 5. 当前限制

- D435→base_link 外参仍未标定，`camera_xyz` 只作为 `CAMERA_LOCAL`，不伪装 map_xyz。
- RTAB-Map ROS2 接入尚未在本机启动，当前默认 SpatialProvider 为 `camera_local` fallback。
- 新的 V2 Spatial Explorer 主循环尚未完全替代旧 `AutonomousExplorer`；当前通过
  `--rgbd-source` 把 D435 颜色/深度带入 `LiveObservation`，PlaceGraph / Frontier /
  LongTermGoalSelector 已可作为独立模块/测试使用。
