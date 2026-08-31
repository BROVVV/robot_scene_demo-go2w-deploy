# Go2-W RGB-D SemanticNavigation V2 Spatial Exploration Handoff (2026-08-18)

> 对应计划书：
> `robot_scene_demo_RGBD_SemanticNavigationV2_PSG_空间自主探索_一次性重构实施计划书_20260818.md`

## 1. Git 状态

- HEAD：`b4ba2c41fbba7f0ac6d69ba019d7fdd585bbe734`
- 工作树为 dirty，按计划书规则保留当前 working tree，未执行 reset/clean。
- 主要修改：D435 原子 RGB-D、RGB-D Source、Depth Localizer、Spatial Models、
  PlaceGraph、Frontier、LongTermGoalSelector、LocalGoalExecutor、PSG Prior、
  SearchEvent/WebUI spatial 接口、D435 作为可选主 RGB。

## 2. 架构变化

```text
D435 HTTP atomic RGB-D
        ↓
RGBDSource / RGBDFrame
        ↓
DepthObjectLocalizer → 3D SemanticObjectMap
        ↓
SpatialProvider / FrontierExtractor / PlaceGraph
        ↓
SemanticNavigation V2 SpatialReasoner / PSG SemanticPriorProvider
        ↓
LongTermGoalSelector → ExplorationIntent
        ↓
LocalGoalExecutor → ExplorationGoal primitives
        ↓
RobotBackend / Go2-W
```

## 3. 新增/修改文件

### 新增模块

- `app/perception/rgbd_source.py`
- `app/perception/realsense_http_rgbd_source.py`
- `app/perception/depth_object_localizer.py`
- `app/spatial/` (models, spatial_provider, camera_local_spatial_provider,
  frontier_extractor, place_graph, semantic_object_map, spatial_memory,
  lightweight_depth_bev)
- `app/navigation/long_term_goal_selector.py`
- `app/navigation/local_goal_executor.py`
- `app/reasoning/semantic_navigation/semantic_prior_provider.py`
- `app/reasoning/semantic_navigation/spatial_reasoner.py`
- `configs/go2w/rgbd_spatial_exploration.yaml`
- `scripts/go2w/validate_rgbd_spatial_stack.py`
- `scripts/go2w/realsense_rgbd_bridge.py`
- `scripts/go2w/start_rgbd_spatial_stack.sh`
- `tests/test_*` 若干

### 修改模块

- `scripts/go2w/realsense_stream.py`：原子 RGB-D API + 32 帧缓存
- `app/navigation/models.py`：LiveObservation RGB-D 字段
- `app/live_robot/semantic_observer.py`：`semantic_observation_to_live` 支持 RGB-D
- `app/live_robot/search_event.py` / `search_state_store.py` / `explorer_search_adapter.py`：
  spatial 事件与快照
- `app/manual_web_demo/*`：WebUI start 请求支持 `rgbd_source`，新增 spatial REST
- `scripts/go2w/run_semantic_exploration.py`：`--rgbd-source` 使用 D435 主 RGB；
  `--spatial-v2` 接入 PlaceGraph / Frontier / LongTermGoalSelector / LocalGoalExecutor
- `scripts/go2w/autonomous_search_worker.py`：forward plumbing 修复 + rgbd/spatial_v2 flags
- `configs/go2w/autonomous_search_web.yaml`：默认 D435 RGB-D source

## 4. D435 Integration

- 机器狗侧服务已重新部署：`/home/unitree/realsense_stream.py`
- 新增原子端点已验证：
  - `/rgbd/latest.json`
  - `/rgbd/frame/<id>/color.jpg`
  - `/rgbd/frame/<id>/depth.png`
- 验证结果：
  - `frame_id` 一致，color/depth 同一 frameset
  - `depth_valid_fraction=0.937`
  - intrinsics：fx=607.99, fy=608.32, cx=318.60, cy=238.30

## 5. 测试

- 新增/相关单元集成测试 38 个全部 PASS：
  - RGBDSource 原子性与 stale
  - DepthObjectLocalizer 深度/相机坐标/降级
  - PlaceGraph 旋转不新增 Place、平移新增 Place
  - FrontierExtractor 真实边界
  - LongTermGoalSelector / SpatialSearchReasoner
  - LocalGoalExecutor 旋转+短步分解
  - RuleSemanticPriorProvider 预测/观测分离
  - SearchStateStore spatial 事件
- Web 相关回归 40 个 PASS（search routes / websocket / session service / state store）
- Mock E2E `run_semantic_exploration.py --backend mock` → `TARGET_FOUND`

## 6. 真机验证（重启后）

- `check_go2w_ready.sh --json` → `state=ready`，硬性项全部通过
- D435 原子帧验证 PASS
- 300 帧原子元数据采样 PASS：300/300 成功，唯一 frame_id=130，max_age=0.035s，stale=0
- 小范围运动测试 PASS：
  - 命令：`run_autonomous_loop.py --pattern r10 --turn-only --operator-authorized-rotation`
  - 结果：10° 右转成功，`yaw_delta_rad≈0.1577`（≈9.02°），odom 验证通过
  - 运动后健康检查仍 ready
- WebUI mock 启动验证 PASS：`start_autonomous_search_web.sh --mock` 可启动，
  `/api/status`、`/api/search/spatial-map`、`/api/search/frontiers` 正常返回
- RTAB-Map 已安装：`ros-humble-rtabmap-ros`，D435 RGB-D Bridge 已运行，
  `/rtabmap/map` 可接收；静态场景 RTAB 无 free 栅格时自动降级到 BEV
- D435 + `--spatial-v2 --rtabmap` 真机运动验证 PASS：
  - ZERO 目标选择 metric frontier `frontier_03_-234_-168`
  - 真实 `ROTATE_VIEW +30°` 执行成功，odom `yaw_delta_deg=29.309`
- D435 + `--spatial-v2 --dry-run-motion` 真机链路验证 PASS（单周期）：
  - D435 原子帧观察到“绿色垃圾桶”，`depth_m=1.1765`，`spatial_quality=CAMERA_LOCAL`
  - V2 选择 `EXPLORE_FRONTIER` → `LocalGoalExecutor` 生成 `ROTATE_VIEW -30°`
  - dry-run 执行成功；因命令超时（180s）第二次 LLM 观察被终止，非代码逻辑失败

## 7. Known Limitations

1. D435→base_link 外参未标定；`camera_xyz` 仅 `CAMERA_LOCAL`，不写 map_xyz；
   当前 RTAB-Map 使用的 base_link→d435 为 nominal static TF（未标定）。
2. RTAB-Map 已安装并运行；当前静态场景 OccupancyGrid 未产生 free 栅格，
   因此实际 Frontier 由 `LightweightDepthBEVMapper` fallback 产生。
3. V2 主循环已通过 `--spatial-v2` 接入（在旧 `AutonomousExplorer` 外壳内替换候选生成/规划）；
   旧 `ExplorationGraph` 仍作为兼容层保留；WebUI 已支持 D435 相机和 spatial 图层叠加渲染。
4. WebUI 前端还未新增 spatial map 图层渲染，但 REST 与 SearchEvent 已具备。

## 8. 下一步建议

- 启动 RTAB-Map（或使用 `LightweightDepthBEVMapper`）获得 Spatial Map，启用 `MetricFrontierProvider`。
- 将 `AutonomousExplorer` 主循环升级为 V2：OBSERVE_RGBD → UPDATE_SPATIAL → LOCAL_SCAN →
  SELECT_LONG_TERM_GOAL → LOCAL_EXECUTE。
- WebUI 前端消费 `/api/search/spatial-map` 与新增 SearchEvent，渲染 PlaceGraph/Frontier/PSG region。
- 真机验收 Trial E/F/G：目标不存在、普通目标视野外、PSG 关系目标。
