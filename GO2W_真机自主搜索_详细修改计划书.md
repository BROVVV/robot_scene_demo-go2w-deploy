# GO2W 真机自主搜索问题修复实施计划书

> **用途**：把本文件直接交给负责修改代码的 AI。  
> **目标**：依据 2026-08-31 真机排查结果，一次性修复「拓扑图无真实物体」「三维建图运动后重影」「连续左转 30°」「单次 VLM 超时直接 PERCEPTION_FAILURE」四条故障链，并补齐测试、日志与验收手段。  
> **原则**：不是“调几个参数”，而是修正语义新鲜度、导航覆盖状态、坐标系契约、异常恢复和可观测性。

---

# 0. 执行 AI 必须先读

## 0.1 本次真实运行基线

本次故障测试实际运行的是：

```text
host branch: feature/semantic-object-topology
host commit: 81cda75（排查时）
session: search_20260831_192836_aa3ca51c
target: 白色垃圾桶
spatial provider: plain_slam
RTAB-Map: 未运行
Point-LIO 独立栈: 未运行
```

注意：

- 排查表标题虽然写过 `robot-go2w-deployment-20260831`，**本次真机实际代码基于 `feature/semantic-object-topology`**。
- 机器狗端部署副本是独立仓库/工作副本，修改后必须确认宿主机源码与真机部署代码一致。
- **不要在错误的旧分支上修改后就宣布完成。**

执行前必须记录：

```bash
git branch --show-current
git rev-parse HEAD
git status --short
```

并在最终报告中写出实际修改的 branch/commit。

---

# 1. 已经确认的故障链

## 1.1 问题一：拓扑图没有真实物体

已确认的实际现象：

```text
Full Semantic 成功次数 = 0
Full Semantic 至少 2 次显式 75s timeout
6 次 observation 的 scene_objects 全部为空
6 次 observation 使用的 semantic frame 一直冻结在首帧 358695
最后 semantic age ≈ 111 秒
机器狗期间已经旋转约 151°
SemanticObjectMap 输入 object 数始终为 0
拓扑最终只有 PLACE + FRONTIER，没有 OBJECT
```

根因链：

```text
Full Semantic 长时间超时
        ↓
超时结果被降级成“空场景”
        ↓
空场景又被当成可复用的 latest semantic
        ↓
后续一直复用首帧 semantic
        ↓
objects 永远为空
        ↓
SemanticObjectMap 没有输入
        ↓
拓扑图没有 OBJECT
```

同时存在潜在跨帧 bug：

```text
old semantic bbox
       +
current RGB-D depth
       ↓
错误的 3D localization
```

本次因为 objects 为空没有真正触发，但必须一起修掉。

---

## 1.2 问题二：三维建图一运动就重影

本次三维建图实际后端是：

```text
plain_slam_ros2
WebUI 输入主要来自 /go2w/slam/aligned_scan
plain_slam map frame = pslam_odom
```

已经确认：

```text
探索/语义空间 pose 来自 /go2w/odom/fused（wheel odom）
代码却硬编码 frame_id="odom"

plain_slam 地图属于 pslam_odom

PlainSlamSpatialProvider 中存在：
wheel odom pose
    +
pslam map 数据
直接混算

没有明确坐标变换
```

另外：

```text
odom_fused -> base_link TF 不存在
base_link <-> D435 optical TF 不完整/未验证
RGB-D bridge 使用 ROS 接收时间，不是真实采集时间
```

对“WebUI 3D 扇形重影”的当前置信度：

```text
坐标系混用：高风险，必须修
plain_slam/LIO 快速旋转时自身漂移：尚未完全确认
```

因此本计划要求：

1. **先修坐标系契约和 WebUI 点云累积逻辑；**
2. **不直接修改 plain_slam_ros2 上游算法；**
3. 修完后用最小旋转实验判断是否仍有 LIO 本体漂移。

---

## 1.3 问题三：连续左转 30°

真实运行 6 次动作全部是：

```text
l30
l30
l30
l30
l30
l30
```

运动执行本身是正常的：

```text
每次 odom yaw 增量约 29.7° ~ 30.9°
累计约 151°
```

真正错误的是“记录状态”：

```text
实际到达 sector:
0 → 1 → 2 → 3 → 4 → 5 ...

heading_coverage:
始终只有 {"0": N}
```

于是：

```text
covered = len(heading_coverage) = 1
max_local_rotations = 3
covered < 3 永远成立
```

再叠加代码固定候选顺序：

```python
for delta_sector in (1, -1, 2, -2):
```

`+1` 是左转，因此永远优先左 30°。

根因不是运动控制，而是：

```text
导航 coverage 错误依赖 stale semantic heading_sector
+
local scan 固定左优先
+
local scan quota 的语义设计不正确
```

---

## 1.4 最终 PERCEPTION_FAILURE

直接触发异常：

```text
subprocess.TimeoutExpired
→ RuntimeError("SiliconFlow vision API timed out")
```

位置：

```text
scripts/go2w/run_autonomous_loop.py
Go2WAutonomousLoop._detect_llm()
```

传播到：

```text
app/live_robot/autonomous_explorer.py
OBSERVE
```

当前逻辑：

```text
如果 observations == 0：
    perception error 可以 retry

如果之前已经成功观察过：
    一次瞬态 perception error
    → 直接 FAILED
```

这导致第 6 轮一次 Quick VLM 20s 超时直接结束整个任务。

此行为必须修改。

---

# 2. 总体修改策略

必须按以下顺序实施：

```text
P0-A 语义任务状态和 fresh-frame binding
        ↓
P0-B heading coverage 与语义彻底解耦
        ↓
P0-C local scan 防死循环
        ↓
P1-A perception timeout 分类与 retry
        ↓
P1-B plain_slam 空间坐标统一
        ↓
P1-C 3D WebUI 点云 frame/timestamp/运动门控
        ↓
P2-A RGB-D timestamp/frame contract
        ↓
P2-B topology frontier ID
        ↓
P2-C 日志、健康状态、回归测试
```

**不要反过来先调 RTAB-Map、运动 PID、turn_kp 或 plain_slam 算法参数。**

---

# 3. P0-A：重构 Full Semantic 的“有效/过期/失败”状态

## 3.1 涉及文件

执行 AI 先按符号搜索，不要死信行号。

重点：

```text
app/live_robot/semantic_observer.py
app/live_robot/async_semantic_observer.py        # 如果当前工作树存在
scripts/go2w/run_semantic_exploration.py
scripts/go2w/run_autonomous_loop.py
SiliconFlow vision worker / daemon 相关文件
scripts/go2w/start_autonomous_search_web.sh
```

---

## 3.2 禁止继续使用的错误语义

当前错误模式类似：

```text
Full Semantic timeout
→ scene_objects=[]
→ source=degraded/fallback
→ 放入 _latest
→ 后续当正常语义继续用
```

必须改成：

```text
SUCCESS
PENDING
TIMEOUT
ERROR
STALE
```

这些状态互相明确区分。

### 强制不变量

**只有真正成功解析的 Full Semantic 结果，才能进入 `latest_success`.**

下面这些结果禁止覆盖 `latest_success`：

```text
timeout
process error
HTTP error
JSON parse error
degraded empty fallback
cancelled
obsolete frame result
```

---

## 3.3 SemanticObservation 增加元信息

给语义结果补齐以下字段；若已有类似字段，则复用，不要重复造概念：

```python
semantic_source_frame_id: str
semantic_capture_timestamp: float | None
semantic_completed_timestamp: float | None
semantic_age_ms: float | None
semantic_status: str
semantic_quality: str
semantic_error_code: str | None
semantic_error_detail: str | None
semantic_source_pose: dict | None
```

推荐状态：

```text
fresh_full
fresh_quick_scene
pending
stale
timeout
error
unavailable
```

要求：

```text
semantic_status != fresh_*
```

时，绝不能把该结果解释成：

```text
“本场景确认没有物体”
```

必须解释成：

```text
“本次场景语义不可用/尚未完成”
```

---

## 3.4 删除“首帧 75 秒同步阻塞”

当前真机第一轮因为 Full Semantic 等待约 75 秒才继续。

修改为：

```text
OBSERVE 当前帧
    ↓
Quick VLM
    ↓
立刻得到 target decision
    ↓
Full Semantic 提交后台 single-flight
    ↓
当前轮不等待 75 秒
```

Full Semantic 不应该卡住实时控制主循环。

但是：

**后台语义未完成时，不能用旧结果假装是当前结果。**

---

## 3.5 Full Semantic 必须采用 single-flight

每个搜索 session 最多允许：

```text
1 个 Full Semantic 请求正在执行
```

不要出现：

```text
cycle1 启动 75s 请求
cycle2 再启动一个
cycle3 再启动一个
...
```

否则 API 负载和进程数会越来越糟。

建议行为：

```python
if no_inflight:
    submit(current_frame)

elif inflight_frame is still relevant:
    do nothing

elif inflight is too old:
    mark obsolete/cancel if possible
    submit newest frame
```

必须记录：

```text
semantic_request_started
semantic_request_completed
semantic_request_timeout
semantic_request_discarded_obsolete
semantic_result_applied
```

每条日志至少带：

```text
frame_id
latency_ms
status
object_count
```

---

# 4. P0-A2：增加轻量场景语义 fallback，避免 Full Semantic 挂掉后拓扑彻底空白

只把 Full Semantic timeout 改成“不缓存”还不够，因为 API 如果长期 >75s，拓扑还是永远没有对象。

因此需要一个**快路径场景物体列表**。

## 4.1 不要破坏 Quick target detector 的职责

当前 Quick VLM 的目标判断语义应继续保持：

```text
target_decision
target_objects
```

不要把普通物体混进 `target_objects`。

建议增加独立字段：

```json
{
  "target_decision": {},
  "target_objects": [],
  "scene_summary_zh": "...",
  "scene_objects_light": [
    {
      "label": "办公椅",
      "confidence": 0.91,
      "bbox_2d": [x1, y1, x2, y2],
      "category": "furniture"
    }
  ]
}
```

限制：

```text
scene_objects_light 最多 6~10 个显著物体
关系可以为空
输出必须小
不要要求复杂长推理
```

目标是：

```text
Quick 一次调用
既完成“目标有没有”
又至少能给拓扑提供粗粒度场景物体
```

Full Semantic 成功后再补：

```text
更完整 object list
relations
scene type
attributes
```

---

## 4.2 semantic 结果优先级

使用顺序：

```text
当前 frame Full Semantic 成功
    > 当前 frame scene_objects_light
    > 当前 frame 无语义
```

**禁止：**

```text
111 秒前 Full Semantic
    > 当前 frame
```

旧语义只能作为历史 memory，不能当当前视觉事实。

---

# 5. P0-A3：严格修复 semantic frame binding

这是必须修的潜在严重 bug。

当前危险路径：

```python
semantic = old_frame_semantic
rgbd_frame = current_frame
depth_localizer.localize(semantic.objects, rgbd_frame)
```

必须彻底禁止。

---

## 5.1 建立 ObservationBundle / FrameBundle 契约

每次采集都形成不可拆分对象：

```text
frame_id
capture_timestamp
rgb_path
depth_path
intrinsics
depth_scale
depth_aligned_to_rgb
capture_pose
```

建议保留最近：

```text
30~60 个 frame
```

的 bounded cache。

---

## 5.2 depth localization 规则

只有满足：

```python
semantic.semantic_source_frame_id == rgbd_frame.frame_id
```

才允许：

```python
depth_localizer.localize(...)
```

或者：

```text
从 RGB-D cache 找到 semantic_source_frame_id 对应的原始 depth
```

再做 localization。

找不到同一 frame：

```text
不做 3D localization
不使用 current depth 顶替
```

只保留：

```text
2D semantic
spatial_quality="SEMANTIC_2D_ONLY"
```

并记录：

```text
semantic_depth_frame_mismatch
```

---

# 6. P0-B：heading coverage 与 Full Semantic 完全解耦

这是解决无限左转最关键的一项。

## 6.1 错误设计

当前 place 更新使用类似：

```python
heading_sector=semantic.heading_sector
```

这是错误的。

因为：

```text
semantic 可能是旧帧
semantic 可能 pending
semantic 可能失败
```

但机器狗“当前朝向”是运动/位姿事实，不是视觉语义事实。

---

## 6.2 新设计

区分两个概念：

```text
navigation_heading_sector
semantic_source_heading_sector
```

### navigation_heading_sector

来源：

```text
当前 observation 对应的 capture-time odom/pose
```

用于：

```text
PlaceGraph.heading_coverage
local scan
visited sector
frontier generation
oscillation detection
```

### semantic_source_heading_sector

来源：

```text
产生该 semantic 的旧/当前 frame capture pose
```

仅用于：

```text
解释历史语义物体“从哪个方向看到”
```

它**绝不能**控制当前导航 coverage。

---

## 6.3 AutonomousExplorer 修改

当前逻辑是：

```python
if observation.heading_sector is None and pose is not None:
    observation.heading_sector = ...
```

改为：

```text
只要当前 pose 有效：
始终根据当前 pose 计算 navigation heading sector
```

不要因为 observer 已经塞了一个 stale sector 就跳过计算。

更稳妥的做法是明确增加字段：

```python
observation.navigation_heading_sector
observation.semantic_heading_sector
```

如果不想改 schema 太大，则至少：

```text
LiveObservation.heading_sector = current capture pose sector
SemanticObservation.heading_sector = semantic source sector
```

不要混用。

---

## 6.4 run_semantic_exploration 修改

以下调用：

```python
place_graph.register_observation(
    heading_sector=semantic.heading_sector,
    ...
)
```

必须改成：

```python
heading_sector=observation.heading_sector
```

或者明确：

```python
heading_sector=navigation_heading_sector
```

同理检查：

```text
entity_graph
spatial memory
observed_sectors
planner state
frontier coverage
```

所有属于“机器人当前看过哪些方向”的地方，都必须使用**当前 observation/capture pose sector**。

---

# 7. P0-C：重写 local scan，彻底防止无限单向旋转

## 7.1 不再使用固定“找到第一个就 return”

当前：

```python
for delta_sector in (1, -1, 2, -2):
    ...
    return [goal]
```

必须修改。

---

## 7.2 将“覆盖”和“旋转次数”分开

当前：

```python
covered = len(current_place.heading_coverage)
if covered < max_local_rotations:
```

变量语义不一致。

`max_local_rotations` 应该表示：

```text
这个 Place 最多执行多少次 LOCAL_SCAN 旋转动作
```

因此每个 Place 增加或维护：

```text
local_scan_steps
local_scan_target_sectors
last_local_scan_direction
```

例如：

```text
初始朝向 sector 0
local_scan_steps = 0

转一次后：
local_scan_steps = 1

max_local_rotations = 3
→ 同一个 Place 最多执行 3 个 local scan 动作
```

heading coverage 仍作为信息覆盖数据，但不再充当 rotation counter。

---

## 7.3 候选必须左右对称

推荐生成：

```text
+30°
-30°
+60°
-60°
```

中所有合法候选，返回给 planner，而不是第一个直接 return。

至少需要加入重复方向惩罚：

```text
如果上一轮刚左转：
本轮同分时优先右侧
```

建议 planner score 增加：

```text
unvisited_sector_bonus
information_gain
same_direction_repeat_penalty
recent_sector_penalty
```

---

## 7.4 加死循环保护器

增加硬保护：

```text
same_place_consecutive_rotate_count
same_direction_consecutive_count
```

建议默认：

```text
same_place LOCAL_SCAN 达 max_local_rotations
→ 强制退出 LOCAL_SCAN

同方向相同 30°动作连续 >= 2 且没有新信息
→ 下一次禁止再次选择同方向
```

“新信息”可定义为至少一个：

```text
新 navigation sector
新 object
新 frontier
target score 提升
新 place
```

---

## 7.5 motion success 后用实际 odom 修正 sector

动作目标：

```text
requested +30°
```

和实际动作：

```text
observed +30.3°
```

可能有差异。

运动完成后：

```text
读取真实当前 yaw
重新计算 reached_sector
记录 reached_sector
```

日志必须包含：

```text
requested_dyaw_deg
observed_dyaw_deg
sector_before
sector_requested
sector_reached
coverage_before
coverage_after
```

**不要记录“计划想去的 sector”冒充“实际到达 sector”。**

---

# 8. P1-A：修改 perception retry，不再因为一次 Quick timeout 直接判死

## 8.1 涉及文件

```text
app/live_robot/autonomous_explorer.py
scripts/go2w/run_autonomous_loop.py
scripts/go2w/run_semantic_exploration.py
```

---

## 8.2 增加可恢复错误分类

不要只抛：

```python
RuntimeError("SiliconFlow vision API timed out")
```

建议统一成结构化错误，例如：

```python
PerceptionFailure(
    code="QUICK_VLM_TIMEOUT",
    recoverable=True,
    detail="..."
)
```

至少分类：

```text
QUICK_VLM_TIMEOUT
FULL_SEMANTIC_TIMEOUT
RGBD_TIMEOUT
RGB_IMAGE_ERROR
DEPTH_ERROR
VLM_PARSE_ERROR
TF_UNAVAILABLE
FRAME_MISMATCH
UNKNOWN_PERCEPTION_ERROR
```

---

## 8.3 retry 逻辑

删除以下错误条件：

```python
if observations == 0 and perception_retries < ...
```

改为：

```python
if error_is_recoverable and perception_retries < max_perception_retries:
    retry
```

无论之前是否已经有 observation。

默认：

```text
max_perception_retries = 2
```

含义：

```text
当前 OBSERVE 最多：
原始尝试 1 次 + retry 2 次
```

建议 backoff：

```text
retry 1: 1 秒
retry 2: 2 秒
```

机器人保持 stop 状态，不允许在感知失败时继续使用 stale frame 发新运动命令。

---

## 8.4 Full Semantic timeout 不得结束搜索

因为 Full Semantic 已经改成后台增强信息：

```text
Full Semantic timeout
→ semantic quality degraded
→ 不应该抛到 AutonomousExplorer 导致 FAILED
```

Quick target perception 才属于关键搜索感知。

---

## 8.5 最终失败信息必须精准

最终若 Quick 连续超时超过重试次数，WebUI 应显示：

```text
PERCEPTION_ERROR
cause: QUICK_VLM_TIMEOUT
attempts: 3
last_success_age_s: ...
recoverable: true/false
```

不要再只显示：

```text
搜索链路返回了未分类异常
```

---

# 9. P1-B：统一 plain_slam 空间坐标系

## 9.1 核心原则

**一个 SpatialProvider 内部只能存在一个世界坐标系。**

当前 plain_slam 世界：

```text
pslam_odom
```

因此：

```text
PlainSlamSpatialProvider
semantic object map_xyz
frontier map_xy
place spatial position
WebUI plain_slam overlay
```

都必须明确使用：

```text
pslam_odom
```

不能再把 wheel odom 的：

```text
x/y/yaw
frame="odom"
```

直接塞进去与 pslam map 混算。

---

## 9.2 修 run_semantic_exploration

当前危险逻辑类似：

```python
spatial_pose = SpatialPose(
    x=wheel_odom.x,
    y=wheel_odom.y,
    yaw=wheel_odom.yaw,
    frame_id="odom",
)
camera_provider.set_pose(spatial_pose)
```

当：

```text
camera_provider = PlainSlamSpatialProvider
```

时禁止这么做。

推荐：

```text
运动控制 pose：
    wheel odom
    只用于相对运动/当前 yaw/运动验证

空间地图 pose：
    从 PlainSlamSpatialProvider.get_pose()
    frame_id 必须是 pslam_odom
```

如果 plain_slam pose 当前不可用：

```text
不要拿 wheel pose 冒充 pslam pose
```

降级为：

```text
spatial_quality="NO_GLOBAL_SPATIAL_POSE"
只保留 camera-local / 2D topology
```

---

## 9.3 PlainSlamSpatialProvider 必须做 frame assertion

在：

```text
set_pose()
camera_point_to_spatial()
get_frontiers()
```

增加 frame contract。

例如：

```python
if pose.frame_id != self.map_frame:
    raise SpatialFrameMismatch(...)
```

或者明确通过 TF transform 后才能使用。

严禁 silently 混算。

日志：

```text
SPATIAL_FRAME_MISMATCH:
pose_frame=odom
map_frame=pslam_odom
```

---

## 9.4 不要盲目打开 wheel odom TF

排查表发现：

```text
odom_fused -> base_link TF 当前不存在
```

这确实需要把 TF 契约理顺，但**不要直接无脑 `publish_tf=true`**，否则可能以后与 pslam TF 形成 `base_link` 多父节点。

执行 AI 必须先画清实际 TF tree。

推荐架构：

```text
pslam_odom
   ↓ dynamic pose from plain_slam
base_link / pslam body frame
   ↓ static sensor extrinsics
lidar / camera frames
```

wheel odom：

```text
/go2w/odom/fused
```

可以继续作为运动控制 topic。

如果必须发布 wheel TF，请使用明确的 wheel frame 命名，并确保不会与 pslam 同时给同一 child frame 建第二个父关系。

---

# 10. P1-C：修 WebUI 三维点云累积

## 10.1 涉及模块

搜索：

```text
plain_slam_web_bridge
/go2w/slam/aligned_scan
map_3d
voxel
point cloud accumulation
```

---

## 10.2 每帧必须验证 frame_id

目标 world frame：

```text
pslam_odom
```

处理规则：

```text
incoming PointCloud2.header.frame_id == pslam_odom
    → 可以直接累积

否则：
    → 必须按该消息 timestamp 用 TF 转到 pslam_odom
    → TF 不可用就丢弃该帧
```

**绝不能把 sensor frame 的点直接当 world point 累积。**

---

## 10.3 每次新 mapping session 清空旧 voxel map

WebUI/bridge 启动新的建图 session 时：

```text
clear voxel accumulator
clear previous pose
clear stale map revision
```

避免旧 session 污染新实验。

---

## 10.4 增加运动期保护

当前表现像转动时每帧都被错误累积。

在不修改 plain_slam 上游算法的前提下，给 WebUI 累积层加入保守 gate：

```text
机器人明显运动时：
    可以继续显示 latest aligned_scan
    但先不把该帧永久写入 accumulated voxel map

运动停止并稳定后：
    等待 settle 窗口
    再恢复 permanent accumulation
```

建议条件复用现有运动稳定阈值或从 odom 获取：

```text
abs(yaw_rate) <= 0.03 rad/s
abs(vx), abs(vy) 接近 0
连续稳定 >= 300~500ms
```

不要修改 motion controller 的 PID 或转角。

这个 gate 的目的：

```text
避免 LIO 在动态过渡阶段的一两帧错误 pose 永久污染 WebUI 3D 地图
```

---

## 10.5 加入 scan diagnostics

每隔固定帧数输出：

```text
scan_frame_id
scan_stamp
target_map_frame
point_count
pose_x
pose_y
pose_yaw
delta_yaw
stationary
accumulated_voxels
dropped_reason
```

对异常 pose jump：

```text
只报警/丢弃明显坏帧
不要自动调 LIO 参数
```

---

# 11. P2-A：修 D435 RGB-D timestamp 和 frame 契约

虽然本次 plain_slam 3D 重影不是 D435 的直接主因，但这是下一步物体 3D 定位必炸的问题，必须一起清掉。

## 11.1 使用真实采集时间

当前：

```python
stamp = self.get_clock().now().to_msg()
```

是在 HTTP 下载和解码后才打时间。

HTTP 服务端已经提供：

```text
host_timestamp
device_timestamp_ms
```

修改策略：

1. 优先使用服务端 `host_timestamp`；
2. 做合法性和时钟偏差校验；
3. 如果 server clock 与 ROS host 不在同一时基，建立 offset；
4. 实在不可用才 fallback 到 receive time；
5. fallback 时明确记录：

```text
timestamp_quality="receive_time"
```

---

## 11.2 aligned depth 的 frame_id

当前服务端声明：

```text
depth_aligned_to_color=true
```

如果这个声明经过验证，则 aligned depth 实际处于 color camera geometry。

因此建议：

```text
color frame_id = d435_color_optical_frame
aligned depth frame_id = d435_color_optical_frame
CameraInfo = color camera info
```

而不是：

```text
depth frame_id=d435_depth_optical_frame
但系统没有 depth↔color TF
```

如果：

```text
depth_aligned_to_color=false
```

则必须：

```text
独立 depth CameraInfo
真实 depth optical frame
depth→color extrinsic/registration
```

否则拒绝 RGB-D 3D localization。

---

## 11.3 D435 static TF

不要编造外参数字。

读取项目已经存在的：

```text
sensor_extrinsics.yaml
camera_intrinsics.yaml
```

如果外参仍标记：

```text
candidate_unconfirmed
```

则：

```text
可以发布“候选外参”用于实验
但 readiness/health 必须显示 UNCALIBRATED
禁止把该定位标成高质量 GLOBAL
```

后续实机标定后再更新数值。

---

# 12. P2-B：Frontier ID 唯一化

当前：

```text
F01
F02
...
F12
```

只按 sector 命名。

多 Place 后会冲突。

改为内部唯一 ID：

```text
frontier:P1:01
frontier:P1:02
frontier:P2:01
```

UI 可以继续显示短标签：

```text
F01
```

但 node_id 必须全局唯一。

同步修改：

```text
edge source/target
serialization
WebUI selection
tests
```

---

# 13. 拓扑 WebUI 必须增加“语义健康状态”

当前最大的问题之一是：

```text
语义已经挂了
UI 看起来还像正常运行
```

至少显示：

```text
Quick VLM: OK / RETRY / ERROR
Full Semantic: FRESH / PENDING / TIMEOUT / STALE
Semantic frame: 363214
Current RGB frame: 363214
Semantic age: 820 ms
Objects this frame: 5
Spatial localization: RGBD / 2D_ONLY / NO_TF
```

当 Full Semantic timeout：

```text
不要显示成“场景中没有物体”
```

显示：

```text
Full Semantic unavailable
Using lightweight scene objects
```

如果连 lightweight objects 都没有：

```text
Semantic unavailable — frontier-only topology
```

这样以后看到 `P1 + Fxx` 就不会再误判。

---

# 14. “运动后新帧”门控

排查显示运动控制端已经有：

```text
稳定检测
约 250ms stable samples
1s post-turn zero velocity hold
```

所以**不要再粗暴加入 5 秒 sleep**。

但感知端需要新帧约束。

运动结束后记录：

```text
motion_end_timestamp
last_frame_id_before_motion
```

下一轮 OBSERVE 必须满足至少一项：

```text
frame_id != last_frame_id_before_motion
capture_timestamp > motion_end_timestamp
```

最好两项都满足。

如果 2~3 秒内没有新帧：

```text
FRAME_STALE
→ perception retry
```

不能继续拿运动前的帧做下一步计划。

---

# 15. 统一每轮 Observation 的数据模型

建议每轮最终写入统一结构：

```json
{
  "bundle_id": "363214",
  "capture_timestamp": 0,
  "capture_pose": {
    "x": 0,
    "y": 0,
    "yaw_rad": 0,
    "frame_id": "..."
  },

  "navigation_heading_sector": 5,

  "quick": {
    "status": "ok",
    "latency_ms": 5300,
    "target_present": false,
    "scene_object_count": 6
  },

  "semantic": {
    "status": "fresh_full",
    "source_frame_id": "363214",
    "age_ms": 300,
    "object_count": 9
  },

  "spatial": {
    "provider": "plain_slam",
    "map_frame": "pslam_odom",
    "pose_frame": "pslam_odom",
    "quality": "GLOBAL"
  }
}
```

这样以后日志不需要再靠推断。

---

# 16. 必须新增的日志事件

至少新增：

```text
camera_frame_selected
quick_vlm_started
quick_vlm_completed
quick_vlm_retry
quick_vlm_failed

semantic_submitted
semantic_completed
semantic_timeout
semantic_stale
semantic_discarded
semantic_applied

semantic_depth_frame_mismatch

navigation_sector_observed
local_scan_candidate
local_scan_selected
local_scan_quota_exhausted

motion_sector_reached

spatial_frame_mismatch
slam_scan_dropped
slam_scan_accumulated
```

关键日志必须包含 frame/sector/latency，不要只写自然语言。

---

# 17. 自动化测试要求

AI 不得只改代码不补测试。

---

## 17.1 Semantic timeout 测试

构造：

```text
Full Semantic analyze 永久 timeout
```

期望：

```text
latest_success 不被 degraded empty 覆盖
semantic_status = timeout/pending
不会把 objects=[] 当“确认空场景”
搜索主循环不会因为 Full Semantic timeout 直接 FAILED
```

---

## 17.2 Lightweight scene fallback 测试

输入：

```text
Quick:
target_present=false
scene_objects_light=[chair, box, cabinet]
Full Semantic timeout
```

期望：

```text
LiveObservation.scene_objects 有 3 个
SemanticObjectMap 收到对象
Topology 出现 OBJECT 节点
source/quality 标记 quick_scene_light
```

---

## 17.3 stale semantic frame 测试

构造：

```text
semantic frame = A
current RGB-D frame = B
A != B
```

期望：

```text
不得执行 old bbox + B depth localization
输出 SEMANTIC_2D_ONLY
产生 frame_mismatch log
```

---

## 17.4 heading coverage 回归

模拟 odom：

```text
0°
30°
60°
90°
120°
150°
```

同时让 semantic 永远返回：

```text
heading_sector=0
```

期望：

```text
navigation coverage 仍然记录：
0,1,2,3,4,5

绝不能仍为 {"0":6}
```

这是本次最重要的回归测试。

---

## 17.5 local scan quota 测试

设置：

```text
max_local_rotations=3
```

期望：

```text
同一个 Place 最多产生 3 次 local scan rotate
第 4 次必须 fall through 到其他规划分支
```

---

## 17.6 左右偏置测试

无额外信息时连续规划多轮：

期望：

```text
不能稳定输出：
l30,l30,l30,l30,l30...
```

至少应该：

```text
左右均有机会
或达到 quota 后退出 local scan
```

---

## 17.7 Quick VLM 瞬态 timeout 测试

模拟：

```text
cycle 1~5 success
cycle 6 attempt 1 timeout
cycle 6 attempt 2 success
```

期望：

```text
产生 quick/perception retry event
session 不 FAILED
继续搜索
```

---

## 17.8 Quick 连续失败测试

模拟：

```text
连续超过 max_perception_retries
```

期望最终：

```text
PERCEPTION_FAILURE
cause=QUICK_VLM_TIMEOUT
attempts 精确
不是 “unclassified exception”
```

---

## 17.9 Spatial frame mismatch 测试

输入：

```text
provider map_frame=pslam_odom
pose.frame_id=odom
```

期望：

```text
不得直接混算
必须 transform 或显式拒绝/degrade
```

---

## 17.10 Frontier ID 测试

创建：

```text
P1 sector 1
P2 sector 1
```

期望：

```text
两个 frontier node_id 不同
```

---

# 18. 真机修改后验收顺序

**不要直接再次跑完整自主搜索。**

按以下顺序。

---

## 18.1 验收 A：静止语义

机器人不动。

场景里放：

```text
办公椅
纸箱
柜子
垃圾桶
```

运行 30 秒。

通过条件：

```text
Quick 正常
Full Semantic 即使 timeout，scene_objects_light 仍能输出普通物体
SemanticObjectMap object_count > 0
Topology 至少出现 2~3 个 OBJECT 节点
WebUI 能看到 semantic status
没有 111 秒 stale semantic 被当当前结果
```

失败则不要进行运动测试。

---

## 18.2 验收 B：纯旋转 coverage

关闭自主搜索，只做：

```text
初始
左转 30°
停稳
再左转 30°
停稳
再左转 30°
```

检查：

```text
navigation sectors：
0 → 1 → 2 → 3

Place heading_coverage：
至少包含 0,1,2,3
```

绝不能：

```text
{"0":4}
```

---

## 18.3 验收 C：plain_slam 三维建图

流程：

```text
静止 5s
左转 30°
静止 5s
左转 30°
静止 5s
```

同时保存：

```text
/go2w/slam/aligned_scan
/go2w/slam/odom_base
/go2w/odom/fused
TF tree
WebUI 3D screenshot
```

通过条件：

```text
同一墙面/桌腿等稳定结构不能每转 30°多复制一层
scan frame 必须明确属于/转换到 pslam_odom
运动期被 gate 的 scan 不进入永久 voxel map
```

如果修完坐标契约以后仍出现明显扇形重影：

```text
不要继续改本仓库 planner
```

而是判定进入：

```text
plain_slam/LIO sensor fusion 专项
```

需要采集 rosbag 后再分析：

```text
Pandar raw points
IMU
pslam odom
aligned_scan
timestamps
```

再决定 IMU 外参、time sync、deskew 或 LIO 参数。

---

## 18.4 验收 D：感知 retry

通过测试注入或临时把一次 Quick 请求模拟为 timeout。

通过条件：

```text
第一次 timeout
→ robot stop
→ retry
→ 第二次成功
→ session 继续
```

而不是：

```text
一次 timeout → FAILED
```

---

## 18.5 验收 E：完整自主搜索

最后才跑：

```text
找到白色垃圾桶
```

建议第一次限制：

```text
max_motion_steps 较小
max_local_rotations=3
操作员全程持遥控急停
```

通过标准：

```text
1. 不再连续无限 l30
2. heading coverage 随实际 yaw 变化
3. 至少有 OBJECT 拓扑节点
4. semantic frame age 不持续无限增长
5. Quick 单次 timeout 可恢复
6. 3D WebUI 不再每转一次复制一层结构
7. 不出现 old semantic + current depth
```

---

# 19. 不允许修改的内容

本次没有证据支持以下改动，AI 不要顺手乱改：

```text
1. 不调 turn_kp / turn_control_hz / 30°运动参数
   原因：6 次实测角度都正确。

2. 不调 RTAB-Map 参数
   原因：本次 RTAB-Map 根本没运行。

3. 不直接修改 plain_slam_ros2 上游 LIO 算法
   原因：LIO 本体漂移尚未通过最小实验确认。

4. 不编造 D435 外参数值
   原因：当前外参未完成实机标定。

5. 不提高任何安全授权权限
   不修改 autonomous motion 的安全边界和急停逻辑。

6. 不为了“让图看起来正常”直接清空/隐藏异常点而不修 frame contract。
```

---

# 20. 推荐修改文件清单

实际文件名以当前工作树为准，AI 必须先搜索确认。

### 必改

```text
app/live_robot/autonomous_explorer.py

scripts/go2w/run_semantic_exploration.py
scripts/go2w/run_autonomous_loop.py

app/live_robot/semantic_observer.py
app/live_robot/async_semantic_observer.py   # 当前工作树若存在

app/spatial/place_graph.py
app/spatial/plain_slam_spatial_provider.py

app/spatial/semantic_navigation_graph.py
```

### 很可能需要修改

```text
SiliconFlow quick/full semantic worker
VLM daemon/client
scripts/go2w/start_autonomous_search_web.sh

scripts/go2w/realsense_rgbd_bridge.py
app/perception/realsense_http_rgbd_source.py

plain_slam WebUI bridge / aligned_scan accumulation module
WebUI semantic topology/status components
```

### 必须补测试

搜索现有 tests 目录后，在对应模块旁新增：

```text
test_semantic_timeout_not_cached_as_success
test_semantic_frame_binding
test_navigation_heading_sector_uses_current_pose
test_heading_coverage_with_stale_semantic
test_local_scan_quota
test_local_scan_no_fixed_left_loop
test_perception_retry_after_previous_success
test_quick_scene_objects_build_topology
test_plain_slam_frame_mismatch
test_frontier_ids_unique_across_places
test_rgbd_capture_timestamp
```

---

# 21. 建议的实现提交顺序

为了方便定位回归，建议至少拆成这些逻辑 commit：

```text
commit 1
fix semantic freshness + frame binding

commit 2
fix heading coverage + local scan loop

commit 3
fix perception retry + structured error codes

commit 4
fix plain_slam frame contract + 3D scan accumulation

commit 5
fix RGB-D timestamp/frame semantics

commit 6
fix frontier ids + WebUI health/status

commit 7
tests + runbook + log observability
```

如果执行 AI 最终只提交一个 commit，也必须在最终报告里按上述模块列清楚 diff。

---

# 22. 完成定义（Definition of Done）

不能以“代码能运行”作为完成。

必须全部满足：

- [ ] Full Semantic timeout 不再生成“可复用的正常空语义”
- [ ] Full Semantic 不再首轮同步卡 75 秒
- [ ] 后台 Full Semantic 最多一个 in-flight request
- [ ] 当前 semantic frame 与当前 RGB-D frame 可追踪
- [ ] old semantic bbox 不会配 current depth
- [ ] Full Semantic 不可用时仍有 lightweight scene object fallback
- [ ] 拓扑静止测试能生成 OBJECT
- [ ] navigation heading sector 来自当前 pose，不来自 stale semantic
- [ ] 实际旋转 30° 后 heading coverage 正确增加
- [ ] `max_local_rotations` 真正限制旋转次数
- [ ] local scan 不会永久固定向左
- [ ] 单次 Quick timeout 会 retry
- [ ] retry 耗尽后错误 cause 精确
- [ ] PlainSlamSpatialProvider 不再混用 odom 与 pslam_odom
- [ ] WebUI 点云只在正确 world frame 下累积
- [ ] mapping session 会清空旧 voxel accumulator
- [ ] 运动期异常 scan 不会永久污染累积图
- [ ] D435 timestamp 使用采集时间或明确标记 receive-time fallback
- [ ] aligned depth frame contract 正确
- [ ] Frontier node ID 跨 Place 唯一
- [ ] 上述自动化测试全部通过
- [ ] 完成静止语义、30° coverage、3D 旋转、timeout retry 四项真机验收

---

# 23. 执行 AI 最终必须交付的报告

修改完成后不要只回复“已修复”。

必须输出：

```text
【实际修改基线】
branch:
before commit:
after commit:

【修改文件】
1.
2.
3.

【问题1：拓扑无物体】
修改点：
为什么能解决：
测试结果：

【问题2：3D 重影】
修改点：
统一后的 map frame：
aligned_scan frame：
运动期 scan gate：
最小旋转实验结果：
如果仍存在重影，是否确认是 LIO 本体问题：

【问题3：连续左转】
heading sector 来源：
coverage 更新逻辑：
local_scan quota：
左右选择逻辑：
测试结果：

【PERCEPTION_FAILURE】
错误分类：
retry 次数：
单次 timeout 注入测试：
最终失败 cause：

【Frame Binding】
current frame:
semantic source frame:
depth frame:
mismatch 行为：

【自动化测试】
命令：
通过数：
失败数：

【真机验收】
A 静止语义：
B 30° coverage：
C 3D rotation：
D timeout retry：
E full search：

【仍未解决/需要人工标定】
例如 D435 外参、LIO 本体漂移等。
```

---

# 24. 最关键的实现思想

执行过程中如果遇到代码结构和本文不完全一致，请遵守以下四条不变量，而不是机械照抄行号。

## 不变量 1：状态不能撒谎

```text
timeout ≠ empty scene
stale ≠ current frame
pending ≠ no objects
```

---

## 不变量 2：导航朝向是运动状态，不是语义状态

```text
heading coverage
必须来自当前真实 pose/yaw
```

绝不能由一张 111 秒前的语义图片决定机器狗现在朝哪个方向。

---

## 不变量 3：坐标不能裸混

```text
wheel odom
pslam_odom
camera optical
```

三者只有经过明确 transform 才能互相使用。

任何：

```text
frame A 的数值 + frame B 的地图直接运算
```

都必须禁止。

---

## 不变量 4：瞬态错误不能等于任务死亡

```text
一次 VLM timeout
```

应先：

```text
stop → retry → fresh observation
```

只有连续失败达到阈值，才进入：

```text
PERCEPTION_FAILURE
```

---

# 25. 最终预期

修改完成后，正确行为应该变成：

```text
机器人停稳
   ↓
取得运动后的 fresh frame
   ↓
Quick VLM：
目标判断 + 少量场景物体
   ↓
后台 Full Semantic：
成功则增强
失败则只标 degraded，不污染 current semantic
   ↓
当前 odom/pose 计算 navigation sector
   ↓
拓扑：
P1 + OBJECT + 真正未覆盖 FRONTIER
   ↓
planner：
在有限 local scan 内左右合理选择
   ↓
动作完成
   ↓
根据真实 odom 更新 reached sector
   ↓
重新观察
```

三维侧：

```text
plain_slam 全部使用 pslam_odom
   ↓
aligned_scan frame 检查/transform
   ↓
运动阶段不过早永久累积坏帧
   ↓
停稳后继续融合
```

异常侧：

```text
Quick timeout
   ↓
结构化 QUICK_VLM_TIMEOUT
   ↓
robot stop
   ↓
retry
   ↓
成功则继续
   ↓
多次失败才终止
```

这才是本次四个问题应该统一收敛到的目标架构。
