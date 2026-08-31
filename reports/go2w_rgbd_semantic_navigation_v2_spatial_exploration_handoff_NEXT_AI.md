# Go2-W RGB-D SemanticNavigation V2 Spatial Exploration — 交接书（给下一个 AI）

> 生成时间：2026-08-18
> 主计划书：`robot_scene_demo_RGBD_SemanticNavigationV2_PSG_空间自主探索_一次性重构实施计划书_20260818.md`
> 当前工作树：`/home/brov/robot/robot_scene_demo`
> 当前 HEAD：`b4ba2c41fbba7f0ac6d69ba019d7fdd585bbe734`（工作树 dirty，按计划书规则保留，未 reset/clean）

---

## 0. 给下一个 AI 的总体结论

当前项目已经从“2D next-view 搜索 + 原地转圈”升级为：

```text
D435 原子 RGB-D
  → RGBDSource / DepthObjectLocalizer
  → PlaceGraph / SemanticObjectMap
  → SpatialProvider（RTAB-Map / CameraLocal / LightweightDepthBEVMapper）
  → PSG SemanticPriorProvider
  → SemanticNavigation V2 SpatialReasoner / LongTermGoalSelector
  → LocalGoalExecutor
  → Go2-W 真实运动
```

**已完成一次真实机器人按 V2 metric Frontier 转向的验证**，并且完成了多周期真实搜索（5 个 planning cycle、5 次运动、其中多次 forward relocation）。

但**尚未达到计划书全部 PASS 验收**，主要是：
1. 没有保存/输出 PlaceGraph 到 WebUI 事件流；
2. RTAB-Map 当前静态场景未产生 free 栅格，真实 Frontier 实际由 BEV fallback 产生；
3. 未完成“普通目标视野外 TARGET_FOUND”和“PSG 关系目标 TARGET_FOUND”的真机验收；
4. WebUI 前端 spatial 图层代码已就绪，但尚未在长时间真机会话中截图/验收；
5. LOCAL_SCAN 已加，但会消耗 motion budget，需要调参。

---

## 1. 已完成内容（按计划书章节）

### Phase 1：D435 原子 RGB-D API
- `scripts/go2w/realsense_stream.py` 新增：
  - `GET /rgbd/latest.json`
  - `GET /rgbd/frame/<id>/color.jpg`
  - `GET /rgbd/frame/<id>/depth.png`
- color/depth 来自同一 RealSense frameset，缓存最近 32 帧。
- 已部署到机器狗 `/home/unitree/realsense_stream.py` 并验证。

### Phase 2：RGBDSource + FrameBundle V2
- `app/perception/rgbd_source.py`：`RGBDFrame` / `RGBDFrameBundle` / `RGBDSource`
- `app/perception/realsense_http_rgbd_source.py`：HTTP 原子帧下载/缓存
- `LiveObservation` 已增加 `depth_ref / rgbd_frame_id / intrinsics / depth_scale / spatial_quality / camera_xyz / map_xyz`

### Phase 3：DepthObjectLocalizer
- `app/perception/depth_object_localizer.py`
- 输出 `depth_m / camera_xyz / bearing_deg / spatial_quality`
- 采样规则：bbox 中心区域 + 有效深度过滤 + MAD 异常剔除 + median

### Phase 4：3D Semantic SceneGraph / SemanticObjectMap
- `app/spatial/semantic_object_map.py`
- `app/spatial/models.py`：`ObjectSpatialObservation` 相关模型
- 真机已验证：绿色垃圾桶 `depth_m=1.1765`、`bearing_deg=9.102`、`CAMERA_LOCAL`

### Phase 5：RGB-D ROS2 Bridge
- `scripts/go2w/realsense_rgbd_bridge.py`
- 发布：
  - `/go2w/d435/color/image_raw`
  - `/go2w/d435/depth/image_rect_raw`
  - `/go2w/d435/color/camera_info`
  - `/go2w/d435/rgbd_health`
- 已真机跑通，DDS 订阅验证通过。

### Phase 6：RTAB-Map / SpatialProvider
- 已安装 `ros-humble-rtabmap-ros`
- `app/spatial/rtabmap_spatial_provider.py` 订阅 `/rtabmap/map`、`/rtabmap/odom`
- `/rtabmap/map` 真机可接收（60×45 OccupancyGrid）
- 当前静态场景 RTAB-Map 未产生 free 栅格 → 自动降级到 `LightweightDepthBEVMapper`
- `scripts/go2w/start_rgbd_spatial_stack.sh` 一键启停 bridge + TF + rtabmap

### Phase 7：FrontierExtractor
- `app/spatial/frontier_extractor.py`
- 真机 ZERO 搜索中已选择 metric frontier：`frontier_03_-234_-168`（来自 BEV）

### Phase 8：PlaceGraph
- `app/spatial/place_graph.py`
- 原地旋转不新增 Place；平移 >= `relocation_min_displacement_m` 新增 Place
- 单元测试覆盖：10 observations 1 Place、3 relocations 4 Places

### Phase 9：信息增益分层
- `LongTermGoalSelector` 中区分 `spatial_gain / psg_prior / travel_cost / visited_penalty`
- 计划书要求 view/semantic/spatial 三分，当前在 selector 评分中体现，未单独建模

### Phase 10：LOCAL_SCAN / Long-Term Goal / LocalExecutor
- `app/navigation/local_goal_executor.py`
- `run_semantic_exploration.py --spatial-v2` 已接入：
  - `EXPLORE_FRONTIER` → `ROTATE_VIEW + RELATIVE_MOVE`
  - `INSPECT_ANCHOR_REGION` / `APPROACH_TARGET` / `VERIFY_TARGET` 映射已实现
- 已加 `--max-local-rotations`（默认 3）实现 bounded LOCAL_SCAN

### Phase 11：SemanticNavigation V2 Spatial Reasoner
- `app/reasoning/semantic_navigation/spatial_reasoner.py`
- `ZERO → EXPLORE_FRONTIER`
- `PARTIAL → INSPECT_ANCHOR_REGION`
- `STRONG / VERIFY → APPROACH_TARGET / VERIFY_TARGET`

### Phase 12：PSG SemanticPriorProvider
- `app/reasoning/semantic_navigation/semantic_prior_provider.py`
- 规则版 PSG，预测与观测严格分离
- `SemanticRegion` / `SemanticPrior` 模型已建
- 负证据：`app/spatial/spatial_memory.py`

### Phase 13：LongTermGoalSelector
- `app/navigation/long_term_goal_selector.py`
- 可解释打分输出 `ScoredIntent`

### Phase 14：Web forward plumbing
- `SearchStartRequest` 已支持 `rgbd_source / spatial_v2 / rtabmap`
- `autonomous_search_worker.py` 已把参数透传为 CLI flags
- `enable_autonomous_motion=true + turn_only=false` 会自动加 `--semantic-allow-forward`

### Phase 15：WebUI Spatial Map / Place / Frontier / PSG
- 后端：
  - `/api/search/spatial-map`
  - `/api/search/place-graph`
  - `/api/search/frontiers`
  - `/api/search/semantic-map`
- 前端：
  - 自主搜索 Tab 相机已切到 `/api/d435.mjpeg`
  - `/api/d435.depth.mjpeg`
  - `search_map.js` 支持 PlaceGraph / Frontiers / Semantic Objects / PSG Region / Long-Term Goal / Occupancy 叠加
- `SearchEvent` 已扩展 spatial 事件

### Phase 16：测试
- 新增/相关测试 38 个全部 PASS：
  - RGBDSource / DepthLocalizer / PlaceGraph / Frontier / LongTermGoalSelector / LocalGoalExecutor / PSG / SearchStateStore / Web routes / CameraLocal / BEV

### Phase 17-19：真机
- D435 300 帧原子采样 PASS（300/300，max_age 0.035s，stale 0）
- D435 物体 depth/bearing 真机 PASS
- RTAB-Map 地图 topic 真机 PASS（但 free 栅格缺失）
- V2 真实运动验证 PASS：
  - ZERO 目标 → metric frontier → `ROTATE_VIEW +30°` → odom `29.309°`
- 多周期真实搜索 PASS：
  - 5 planning cycles / 5 motion steps / 5 observations
  - 运动序列包含 local rotation + 多次 forward relocation（每次 ~0.16m）

---

## 2. 当前代码关键入口

```bash
# 启动 RGB-D spatial stack（bridge + TF + rtabmap）
bash scripts/go2w/start_rgbd_spatial_stack.sh start

# 停止
bash scripts/go2w/start_rgbd_spatial_stack.sh stop

# 真机 V2 + RTAB-Map（转向）
/usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
  --target "不存在的红色独角兽" \
  --backend go2w_experimental --reasoner semantic_navigation \
  --rgbd-source --spatial-v2 --rtabmap \
  --operator-supervised-experiment --turn-only \
  --max-local-rotations 1 \
  --max-planning-cycles 1 --max-motion-steps 1 \
  --output outputs/live_sessions/rgbd_v2_rtabmap.jsonl

# 真机 V2 多周期 + 前进 relocation
/usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
  --target "不存在的红色独角兽" \
  --backend go2w_experimental --reasoner semantic_navigation \
  --rgbd-source --spatial-v2 --rtabmap \
  --operator-supervised-experiment \
  --max-local-rotations 1 \
  --max-seconds 600 --max-planning-cycles 5 --max-motion-steps 5 \
  --output outputs/live_sessions/rgbd_v2_multicycle.jsonl

# 纯软件验证 D435
/home/brov/miniconda3/envs/go2_robot_scene_demo/bin/python \
  scripts/go2w/validate_rgbd_spatial_stack.py
```

---

## 3. 当前未完成 / 需要下一个 AI 继续做的事

### 3.1 必须优先（影响验收）
1. **把 PlaceGraph / SemanticObjectMap / SpatialMemory 持久化到 session artifacts 并接入 WebUI 事件流**
   - 代码中 `_run_go2w_explorer` 已新增写 `place_graph.json / semantic_map.json / spatial_memory.json`（刚加入，尚未重新跑真机验证）。
   - 还需在 `AutonomousExplorer` 或 adapter 中把 `PLACE_CREATED / FRONTIERS_UPDATED / LONG_TERM_GOAL_SELECTED` 等事件发到 WebSocket，让 WebUI 实时显示。

2. **解决 RTAB-Map 当前无 free 栅格问题**
   - 当前 `/rtabmap/map` 只有 occupied 和 unknown，free=0，导致 metric Frontier 由 BEV fallback 产生。
   - 可能原因：
     - D435→base_link 外参是 nominal static TF（未标定），RTAB-Map 坐标系不对；
     - depth 编码/尺度可能不满足 RTAB-Map 期望；
     - `odom_fused→base_link` 用 static identity，机器人运动时 odom 不一致。
   - 建议：先用小范围 forward/转向让 RTAB-Map 积累更多帧，观察 free 栅格是否出现；如果仍无，检查 `rtabmap.log` 和 depth 话题。

3. **完成真机验收 Trial E（目标不存在）**
   - 需要至少 3 次 relocation、4 个 Place。
   - 当前多周期 run 已出现 4 次 forward relocation，但 PlaceGraph 未保存，无法直接证明。
   - 下一步：跑完上面多周期命令后检查 `outputs/live_runs/<session_id>/place_graph.json`，确认 Places >= 4。

4. **完成真机验收 Trial F（普通目标视野外 TARGET_FOUND）**
   - 例如“绿色垃圾桶”，但开始时让目标在视野外。
   - 需要 LOCAL_SCAN → Frontier → Relocate → New Place → Target → Verify → TARGET_FOUND。

5. **完成真机验收 Trial G（PSG 关系目标）**
   - 例如“饮水机旁边的蓝色垃圾桶”。
   - 需要 PSG 预测区域 → Anchor 3D → SemanticRegion → 搜索 → Target verify。

### 3.2 建议优化
6. **LOCAL_SCAN 不应消耗 motion budget 或应单独计数**
   - 当前 local scan 的 `ROTATE_VIEW` 会计入 `motion_steps`，导致多周期时过早 `MAX_STEPS_REACHED`。
   - 建议：在 `AutonomousExplorer` 中区分 `motion_steps` 与 `local_scan_steps`，或调大 `--max-motion-steps`。

7. **WebUI 长时间真机截图/验收**
   - 前端代码已就绪，但尚未在真实多周期搜索中验证 spatial 图层实时刷新。
   - 建议启动 WebUI 后跑一次多周期搜索，检查 PlaceGraph/Frontier/Long-Term Goal 是否实时出现。

8. **D435 外参标定（非阻塞）**
   - 计划书允许 nominal/relative/degraded 模式；当前 `base_link→d435_color_optical_frame` 是 nominal static TF。
   - 若要 RTAB-Map metric 地图更稳定，后续可做棋盘格/ground plane 外参估计。

9. **LLM 版 PSG**
   - 当前是 `RuleSemanticPriorProvider`；可替换为 LLM 生成，但接口已固定。

---

## 4. 环境与操作注意事项

- 机器狗：`192.168.123.18`，SSH `unitree@192.168.123.18`（密码见操作员，不在本文档记录）
- 本机 sudo：操作员已授权，但文档不保存密码
- 机器人健康检查：
  ```bash
  bash scripts/go2w/check_go2w_ready.sh --json
  ```
- 真机运动需要操作员持遥控器监督：
  - 转向：`--operator-supervised-experiment` 或 `--operator-authorized-rotation`
  - 前进：`--operator-supervised-experiment` + 不传 `--turn-only`
- 当前后台进程：**已停止**，需要时用 `start_rgbd_spatial_stack.sh start`

---

## 5. Definition of Done 对照

### 已满足（代码/测试/部分真机）
- [x] 已保护当前 working tree
- [x] D435 成为主 RGB（WebUI 默认 `rgbd_source=true`）
- [x] 原子 RGB-D frame（color/depth 同 frame_id）
- [x] RGBDFrame / FrameBundle V2 / 旧 RGB-only 兼容
- [x] DepthObjectLocalizer（真机物体 depth/bearing）
- [x] SpatialQuality
- [x] 3D SemanticObjectMap / PSG 分离
- [x] RGB-D bridge 或等价同步接口
- [x] SpatialProvider / True Frontier / RelativeFrontier fallback
- [x] PlaceGraph（原地旋转不增 Place、平移增 Place）
- [x] Heading coverage Place-local
- [x] view/semantic/spatial gain 分层（selector 评分）
- [x] bounded LOCAL_SCAN（`--max-local-rotations`）
- [x] ExplorationIntent / EXPLORE_FRONTIER / INSPECT_ANCHOR_REGION / APPROACH_TARGET
- [x] RELATIVE_MOVE 降为 local primitive
- [x] LongTermGoalSelector
- [x] PSG SemanticPriorProvider / negative memory
- [x] Web forward plumbing
- [x] WebUI D435 / depth / spatial REST / SearchEvent spatial
- [x] Mock/unit tests PASS
- [x] 真 D435 300+ frame PASS
- [x] 真 Object depth/bearing PASS
- [x] 真机 V2 运动 PASS（ZERO → metric frontier → 真实转向）
- [x] 多周期真机搜索 PASS（5 cycle / 5 motion / 多次 forward）

### 未完成 / 需继续
- [ ] PlaceGraph 真机 session 持久化验证（代码刚加，未重新跑）
- [ ] WebUI spatial 图层真机实时验收
- [ ] RTAB-Map 产生 free 栅格 / metric map 稳定
- [ ] Trial E：目标不存在，>=3 relocation / >=4 Place 的正式记录
- [ ] Trial F：普通目标视野外 TARGET_FOUND
- [ ] Trial G：PSG 关系目标 TARGET_FOUND
- [ ] PSG 错误预测 blacklist 真机验证
- [ ] README / 最终 handoff 再更新（完成上述后）

---

## 6. 关键文件索引

| 模块 | 路径 |
| --- | --- |
| D435 原子服务 | `scripts/go2w/realsense_stream.py` |
| RGBDSource | `app/perception/rgbd_source.py` |
| HTTP RGBDSource | `app/perception/realsense_http_rgbd_source.py` |
| DepthLocalizer | `app/perception/depth_object_localizer.py` |
| Spatial models | `app/spatial/models.py` |
| SpatialProvider | `app/spatial/spatial_provider.py` |
| CameraLocal | `app/spatial/camera_local_spatial_provider.py` |
| RTAB-Map | `app/spatial/rtabmap_spatial_provider.py` |
| Frontier | `app/spatial/frontier_extractor.py` |
| PlaceGraph | `app/spatial/place_graph.py` |
| SemanticObjectMap | `app/spatial/semantic_object_map.py` |
| SpatialMemory | `app/spatial/spatial_memory.py` |
| BEV fallback | `app/spatial/lightweight_depth_bev.py` |
| LongTermGoalSelector | `app/navigation/long_term_goal_selector.py` |
| LocalGoalExecutor | `app/navigation/local_goal_executor.py` |
| PSG Prior | `app/reasoning/semantic_navigation/semantic_prior_provider.py` |
| SemanticNavigation V2 | `app/reasoning/semantic_navigation/spatial_reasoner.py` |
| V2 CLI 接入 | `scripts/go2w/run_semantic_exploration.py` |
| D435 ROS2 Bridge | `scripts/go2w/realsense_rgbd_bridge.py` |
| RGB-D Stack 启停 | `scripts/go2w/start_rgbd_spatial_stack.sh` |
| WebUI D435 代理 | `app/manual_web_demo/web_server.py` |
| WebUI spatial 地图 | `app/manual_web_demo/static/search_map.js` |
| WebUI spatial 事件 | `app/manual_web_demo/static/search_ui.js` |

---

## 7. 建议下一个 AI 的第一步

1. 先跑 `bash scripts/go2w/check_go2w_ready.sh --json` 确认机器狗 ready。
2. 跑 `bash scripts/go2w/start_rgbd_spatial_stack.sh start` 启动 RTAB-Map 栈。
3. 跑多周期命令并检查 `outputs/live_runs/<session_id>/place_graph.json`：
   ```bash
   /usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
     --target "不存在的红色独角兽" \
     --backend go2w_experimental --reasoner semantic_navigation \
     --rgbd-source --spatial-v2 --rtabmap \
     --operator-supervised-experiment \
     --max-local-rotations 1 \
     --max-seconds 600 --max-planning-cycles 5 --max-motion-steps 5 \
     --output outputs/live_sessions/rgbd_v2_multicycle.jsonl
   ```
4. 确认 `place_graph.json` 中 Places >= 4，Edges >= 3。
5. 再跑 Trial F / Trial G 完成普通目标与关系目标 TARGET_FOUND。
