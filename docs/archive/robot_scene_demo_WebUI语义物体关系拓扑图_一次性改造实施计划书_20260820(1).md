# robot_scene_demo WebUI 语义物体关系拓扑图一次性改造实施计划书

> 版本：2026-08-20
> 项目：https://github.com/BROVVV/robot_scene_demo
> 目标：把当前 WebUI 中“空间坐标 + Place + Object + Frontier + Route 混合在同一个 SVG 上”的地图展示，改造成**默认显示持久物体之间关系的语义拓扑图**，并保留原空间地图作为可切换的调试视图。
> 执行对象：具备仓库读写、Python/JavaScript 测试与前端调试能力的 Coding AI。
> 本文用途：**直接作为一次性实施指令执行，不是讨论稿，不是建议清单。**

---

# 0. 给执行 AI 的最高优先级指令

拿到本计划书后，直接修改代码，不要再输出一份“方案分析”“建议”“TODO”。

执行要求：

1. 拉取并审计当前 GitHub `main` 最新代码。
2. 以当前仓库代码为事实来源；本计划书中提到的文件和接口如果已经有更新，优先兼容当前实现。
3. 不要推翻目前已经完成的：
   - `camera_xyz -> map_xyz`
   - RTAB / spatial provider
   - `PlaceGraph`
   - `SemanticObjectMap`
   - persistent object association
   - `SemanticEntityGraph`
   - route planner
   - DecisionRecord
   - WebUI spatial state
4. 本次只重点解决：
   - **物体关系持久化**
   - **frame object id -> persistent object id 的正确映射**
   - **OBJECT -> OBJECT relation graph**
   - **WebUI 默认语义物体拓扑图**
   - **拓扑布局与 metric/map_xyz 完全解耦**
5. 原来的 metric spatial map 不删除。改成第二个可切换视图，用于：
   - RTAB occupancy
   - robot pose
   - Place
   - Frontier
   - Route
   - map_xyz 调试
6. 语义拓扑图不得使用：
   - `map_xyz`
   - `camera_xyz`
   - RTAB occupancy
   - Place pose
   来决定节点屏幕位置。
7. 语义拓扑图的节点位置只能是**display layout**，绝不能反向影响导航。
8. 所有 OBJECT node 必须使用稳定 persistent id：
   - `obj_001`
   - `obj_002`
   - ...
9. 绝对禁止使用：
   ```text
   label -> persistent id
   ```
   作为实体身份映射。
10. 必须支持同类别多个物体：
    ```text
    椅子 A -> obj_004
    椅子 B -> obj_007
    ```
11. OBJECT relation 必须持久去重：
    ```text
    obj_001 --near--> obj_002
    ```
    连续看到 10 次仍然是一条 edge，只增加 observation_count / confidence。
12. 修改完成后必须新增并运行测试。
13. 前端必须做稳定布局，实时刷新时不能所有节点不断跳动。
14. 不允许引入重量级前端图形依赖作为第一版必要条件；优先继续使用当前原生 SVG renderer。
15. 不允许破坏当前导航、运动、安全和 Decision pipeline。
16. 最终必须做到：
    > 后台继续维护真实空间世界模型；WebUI 默认展示“机器人理解的物体及物体间关系”。

---

# 1. 当前代码基线审计

执行 AI 开工前重新确认，但截至 2026-08-20 当前 `main` 的关键事实如下。

---

## 1.1 当前 `SemanticEntityGraph` 只有 Place/Object 和两种边

文件：

```text
app/spatial/semantic_entity_graph.py
```

当前 graph 的定位已经是 persistent world-model graph。

已有节点：

```text
PLACE
OBJECT
```

已有边：

```text
MOVED_TO
OBSERVED_FROM
```

当前 `snapshot()` 返回：

```json
{
  "schema_version": "semantic_entity_graph_v1",
  "revision": 0,
  "frame_id": "map",
  "nodes": [],
  "edges": [],
  "current_place_id": null,
  "route_plan": null
}
```

但是目前没有：

```text
OBJECT -> OBJECT
```

例如：

```text
obj_001 --near--> obj_002
obj_003 --left_of--> obj_001
```

所以当前 graph 不是用户想要的“物体关系拓扑图”。

---

## 1.2 当前 WebUI renderer 是混合空间图

文件：

```text
app/manual_web_demo/static/search_map.js
```

当前 `render()` 会：

1. 读取普通 exploration graph nodes；
2. 合并 PlaceGraph places；
3. `computeLayout()`；
4. `drawGrid()`；
5. 画普通 graph edge；
6. 画 robot；
7. `drawSpatialOverlay()`：
   - occupancy
   - frontiers
   - semantic objects
   - PSG region
   - long-term goal
8. `drawSemanticEdges()`；
9. `drawRoute()`。

其中 semantic object 当前直接按：

```javascript
x = obj.map_xyz[0]
y = -obj.map_xyz[1]
```

来画。

因此当前画面实际上混合了：

```text
exploration topology
+
PlaceGraph
+
metric coordinates
+
occupancy
+
frontier
+
persistent objects
+
route
+
PSG
```

这正是“节点都挤在一起 / 图看不清”的根本 UI 架构问题。

---

## 1.3 当前 realtime semantic observer 已经有关系数据

文件：

```text
app/live_robot/semantic_observer.py
```

当前 full-scene payload 已经读：

```text
scene_objects
scene_relations
```

并转成：

```text
FrameObject
FrameRelation
```

`SemanticObservation` 已经保留：

```python
objects=[...]
relations=[...]
scene_graph=...
```

`semantic_observation_to_live()` 也继续透传：

```python
scene_objects=list(semantic.objects)
scene_relations=list(semantic.relations)
```

因此：

> **无需重新调用一套关系识别模型。**

本次应该直接复用现有实时关系结果。

---

## 1.4 当前离线关系词表已经存在

文件：

```text
app/video/observed_scene_graph_builder.py
```

当前 `_add_frame_relations()` 已经支持：

```text
near
left_of
right_of
in_front_of
behind
on
under
above
below
in
inside
contains
attached_to
blocks
adjacent_to
```

这一套关系 vocabulary 直接作为在线 persistent relation 的基础。

不要重新设计一套完全不同的关系枚举。

---

## 1.5 当前 FrameObject 使用 `frame_object_id`

文件：

```text
app/video/schemas.py
```

当前：

```python
@dataclass
class FrameObject:
    frame_object_id: str
```

而：

```python
FrameObject.to_dict()
```

会直接产生：

```json
{
  "frame_object_id": "semantic_obj_001"
}
```

---

## 1.6 当前 DepthObjectLocalizer 丢失了 `frame_object_id`

文件：

```text
app/perception/depth_object_localizer.py
```

当前读取 object id 使用：

```python
item.get("id") or item.get("object_id")
```

正常 RGB-D 和 `_rgb_only()` 都没有读取：

```text
frame_object_id
```

因此实时场景中：

```text
FrameObject.frame_object_id = semantic_obj_001
```

在进入：

```text
ObjectSpatialObservation.object_id
```

时可能变成：

```text
None
```

这会破坏：

```text
frame object
→ entity association
→ persistent object
→ relation remap
```

这条关键链路。

---

## 1.7 当前 runner 仍存在 label -> persistent id 映射

文件：

```text
scripts/go2w/run_semantic_exploration.py
```

当前 spatial observation 后虽然已经：

```python
update_result = semantic_map.update_with_associations(...)
```

但是 WebUI event 中的 object id 当前仍有类似：

```python
state.get("persistent_id_for_label", {}).get(obs_item.label)
```

后面还会构造：

```python
persistent_by_label[entry.label] = entry.object_id
```

这个实现对：

```text
椅子 A
椅子 B
椅子 C
```

是不可靠的。

本次必须彻底从**拓扑 identity 路径**中删除 label-based identity。

---

# 2. 本次最终目标

完成后 WebUI 默认应该看到：

```text
                  ┌─────────────────┐
                  │ obj_001         │
                  │ 办公桌          │
                  │ CONFIRMED · ×4  │
                  └────────┬────────┘
                           │
                        邻近 ×3
                           │
                           ▼
┌─────────────────┐   左侧   ┌─────────────────┐
│ obj_003         │ ───────► │ obj_002         │
│ 办公椅          │          │ 绿色垃圾桶      │
│ CONFIRMED · ×2  │          │ CONFIRMED · ×2  │
└─────────────────┘          └────────┬────────┘
                                     │
                                     │ 相邻
                                     ▼
                            ┌─────────────────┐
                            │ obj_005         │
                            │ 文件柜          │
                            └─────────────────┘
```

默认拓扑视图中：

```text
不显示 occupancy
不显示 P1/P2/P3
不显示 robot metric pose
不显示 frontier
不显示 route
不按 map_xyz 摆放
```

这些东西保留在第二个：

```text
空间地图
```

视图。

---

# 3. 总体架构

最终必须形成两套完全分离的 display projection。

```text
                    Persistent World Model
                           │
          ┌────────────────┴────────────────┐
          │                                 │
          ▼                                 ▼
 Spatial Projection                 Semantic Topology Projection
          │                                 │
          │                                 │
 RTAB occupancy                      OBJECT nodes
 Robot pose                          OBJECT relation edges
 PlaceGraph                          stable entity ids
 Frontier                            relation confidence
 Route                               observation count
 map_xyz                             lifecycle
          │                                 │
          ▼                                 ▼
  WebUI「空间地图」                  WebUI「语义拓扑」
```

关键原则：

```text
World Model 可以共享
Display Layout 不共享
```

---

# 4. Phase 1：修复 FrameObject ID 在 RGB-D 链路中的丢失

这是 P0。

如果这里不修，后面 relation 无法可靠从 frame id 映射成 persistent id。

---

## 4.1 修改文件

```text
app/perception/depth_object_localizer.py
```

---

## 4.2 新增统一 helper

建议：

```python
def _source_object_id(item: dict[str, Any]) -> str | None:
    value = (
        item.get("id")
        or item.get("object_id")
        or item.get("frame_object_id")
    )
    if value is None:
        return None
    text = str(value).strip()
    return text or None
```

注意顺序可按兼容性决定，但必须支持：

```text
frame_object_id
```

---

## 4.3 修改正常 RGB-D 路径

原：

```python
object_id=item.get("id") or item.get("object_id")
```

改为：

```python
object_id=_source_object_id(item)
```

---

## 4.4 修改 `_rgb_only()`

同样：

```python
object_id=_source_object_id(item)
```

---

## 4.5 provenance 中保存原始 ID

建议：

```python
provenance={
    ...
    "source_object_id": _source_object_id(item),
}
```

方便调试：

```text
semantic_obj_001
→ obj_004
```

---

## 4.6 测试

新增：

```text
tests/test_depth_object_localizer_identity.py
```

必须覆盖：

### case 1

```python
{"frame_object_id": "semantic_obj_001"}
```

输出：

```text
ObjectSpatialObservation.object_id == semantic_obj_001
```

### case 2

RGB only fallback 也一样。

### case 3

同时有：

```text
id
object_id
frame_object_id
```

使用兼容优先级且稳定。

---

# 5. Phase 2：禁止使用 label 作为 identity

这是第二个 P0。

---

## 5.1 修改文件

```text
scripts/go2w/run_semantic_exploration.py
```

---

## 5.2 删除拓扑链路对以下结构的依赖

```text
persistent_id_for_label
persistent_by_label
map_xyz_by_label
```

如果其他 legacy 逻辑仍需要 label map，可以暂时保留独立兼容字段，但：

> 语义 topology / relation / semantic object WebUI event 禁止读取它。

---

## 5.3 使用 update_result 构造可靠映射

当前已经有：

```python
update_result = semantic_map.update_with_associations(...)
```

从：

```text
update_result.associations
```

构造：

```python
association_by_index = {
    assoc.observation_index: assoc
    for assoc in update_result.associations
}
```

和：

```python
persistent_by_source_id = {
    assoc.source_object_id: assoc.persistent_object_id
    for assoc in update_result.associations
    if assoc.source_object_id
}
```

必须保证：

```text
semantic_obj_001 -> obj_004
semantic_obj_002 -> obj_008
```

---

## 5.4 WebUI semantic_object_localized event 改造

当前不要再：

```python
for obs_item in localized:
    object_id = persistent_id_for_label.get(obs_item.label)
```

改为：

```python
for index, obs_item in enumerate(localized):
    assoc = association_by_index.get(index)
    if assoc is None:
        continue

    persistent_id = assoc.persistent_object_id
    entity = semantic_map.objects.get(persistent_id)

    if entity is None:
        continue

    node._write({
        "event": "semantic_object_localized",
        "object": {
            **entity.to_dict(),
            "source_object_id": assoc.source_object_id,
            "association_score": assoc.association_score,
            "association_action": assoc.action,
        },
        "host_s": node._host_s(),
    })
```

如果当前 event payload 需要保持旧字段，也保留，但 `object_id` 必须来自 association。

---

## 5.5 为什么必须这样做

错误：

```text
label "chair"
→ obj_004
```

因为存在：

```text
chair A
chair B
```

正确：

```text
frame_object_id
→ observation_index/source_object_id
→ SemanticAssociation
→ persistent_object_id
```

---

## 5.6 测试

必须新增：

```text
tests/test_live_persistent_id_mapping.py
```

场景：

```text
frame object 1 = chair
frame object 2 = chair
```

association：

```text
semantic_obj_001 -> obj_004
semantic_obj_002 -> obj_005
```

最终两个 event：

```text
obj_004
obj_005
```

不能都变成同一个。

---

# 6. Phase 3：新增 Persistent Object Relation 模型

修改：

```text
app/spatial/semantic_entity_graph.py
```

不要另外造第二个独立 world model。

关系应成为当前 `SemanticEntityGraph` 的一部分。

---

## 6.1 新增 relation dataclass

建议：

```python
from dataclasses import dataclass, field

@dataclass
class PersistentObjectRelation:
    edge_id: str

    source_object_id: str
    target_object_id: str
    relation: str

    confidence: float = 0.5
    observation_count: int = 1

    first_seen: float = 0.0
    last_seen: float = 0.0

    status: str = "TENTATIVE"

    relation_scope: str = "STRUCTURAL"

    source_observation_ids: list[str] = field(default_factory=list)
    descriptions_zh: list[str] = field(default_factory=list)

    provenance: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        ...
```

---

## 6.2 Relation status

至少：

```text
TENTATIVE
CONFIRMED
STALE
```

推荐规则：

```text
observation_count == 1
→ TENTATIVE

observation_count >= relation_confirm_min_observations
→ CONFIRMED

long time no observation
→ STALE
```

第一版默认：

```text
relation_confirm_min_observations = 2
```

---

## 6.3 stable edge id

必须 deterministic。

例如：

```python
edge_id = f"{source_id}__{relation}__{target_id}"
```

结果：

```text
obj_001__near__obj_004
```

连续 10 次看到同关系时仍是同一个 edge。

---

## 6.4 relation storage

在：

```python
SemanticEntityGraph.__init__()
```

增加：

```python
self.object_relations: dict[
    tuple[str, str, str],
    PersistentObjectRelation
] = {}
```

key：

```text
(source_object_id, relation, target_object_id)
```

---

# 7. Phase 4：统一关系 vocabulary

不要再创造第四套关系词。

---

## 7.1 在线 relation vocabulary 与离线保持一致

直接使用：

```python
OBJECT_RELATIONS = {
    "near",
    "left_of",
    "right_of",
    "in_front_of",
    "behind",
    "on",
    "under",
    "above",
    "below",
    "in",
    "inside",
    "contains",
    "attached_to",
    "blocks",
    "adjacent_to",
}
```

与：

```text
app/video/observed_scene_graph_builder.py
```

保持一致。

---

## 7.2 relation normalize

新增：

```python
RELATION_ALIASES = {
    "left": "left_of",
    "right": "right_of",
    "front": "in_front_of",
    "in_front": "in_front_of",
    "front_of": "in_front_of",
    "close": "near",
    "close_to": "near",
    "next_to": "adjacent_to",
    "beside": "adjacent_to",
    "within": "inside",
}
```

实现：

```python
def normalize_relation(value: str | None) -> str | None:
    ...
```

不认识的 relation：

```text
默认不要强制改成 near
```

在线 persistent graph 应：

```text
unknown relation
→ ignore + association_debug
```

不要制造假关系。

离线 builder 旧逻辑可保持兼容，不必本次强改。

---

# 8. Phase 5：区分稳定关系与视角关系

这一点决定图是否可靠。

---

## 8.1 STRUCTURAL 关系

更适合持久化：

```text
near
adjacent_to
inside
in
contains
on
attached_to
blocks
```

relation_scope：

```text
STRUCTURAL
```

---

## 8.2 VIEW_RELATIVE 关系

以下关系可能随机器人视角改变：

```text
left_of
right_of
in_front_of
behind
above
below
under
```

relation_scope：

```text
VIEW_RELATIVE
```

不要直接当成永恒世界事实。

---

## 8.3 WebUI 呈现

```text
STRUCTURAL + CONFIRMED
→ 实线

STRUCTURAL + TENTATIVE
→ 虚线 / 低透明

VIEW_RELATIVE
→ 虚线 + “视角”标记
```

例如：

```text
左侧 · 视角关系
```

---

## 8.4 可选：基于 map_xyz 升级为 WORLD_RELATIVE

如果两个 object 都有可靠：

```text
METRIC_RGBD
map_xyz
```

未来可以从世界坐标重新计算：

```text
near
left/right in map frame
front/back in map frame
```

但**本次不作为完成条件**。

本次主要目标是关系拓扑展示，不继续扩大 SLAM 范围。

---

# 9. Phase 6：把 realtime frame relation 映射成 persistent relation

这是整个功能的核心。

---

## 9.1 修改 `sync_from_observation()`

当前：

```python
sync_from_observation(
    observation_id,
    heading_sector,
    labels,
    spatial_objects,
    pose,
    timestamp,
    place_id,
    update_result,
)
```

增加：

```python
relations: list[dict[str, Any]] | None = None
```

建议最终：

```python
def sync_from_observation(
    self,
    *,
    observation_id: str,
    heading_sector: int | None,
    labels: list[str],
    spatial_objects: list[Any],
    pose: SpatialPose | None = None,
    timestamp: float | None = None,
    place_id: str | None = None,
    update_result: Any | None = None,
    relations: list[dict[str, Any]] | None = None,
) -> dict[str, Any]:
```

---

## 9.2 构造 frame id -> persistent id

在 graph 内部使用：

```python
source_to_persistent = {
    str(assoc.source_object_id): str(assoc.persistent_object_id)
    for assoc in update_result.associations
    if assoc.source_object_id and assoc.persistent_object_id
}
```

---

## 9.3 处理 relation

新增：

```python
def _sync_object_relations(
    self,
    *,
    relations: list[dict[str, Any]],
    update_result: Any,
    observation_id: str,
    timestamp: float,
) -> None:
```

伪代码：

```python
for raw in relations:
    source_frame_id = str(raw.get("subject_id") or "").strip()
    target_frame_id = str(raw.get("object_id") or "").strip()

    source_id = source_to_persistent.get(source_frame_id)
    target_id = source_to_persistent.get(target_frame_id)

    if not source_id or not target_id:
        debug("relation_endpoint_unresolved")
        continue

    if source_id == target_id:
        debug("self_relation_rejected")
        continue

    relation = normalize_relation(raw.get("relation"))

    if relation not in OBJECT_RELATIONS:
        debug("relation_not_allowed")
        continue

    confidence = clamp(raw.get("confidence", 0.5), 0, 1)

    self._upsert_object_relation(
        source_object_id=source_id,
        target_object_id=target_id,
        relation=relation,
        confidence=confidence,
        observation_id=observation_id,
        timestamp=timestamp,
        description_zh=raw.get("description_zh"),
    )
```

---

## 9.4 endpoint 不得用 label fallback

禁止：

```python
if not source_id:
    source_id = find_by_label(subject_label)
```

因为：

```text
两个办公椅
两个垃圾桶
```

会错误合并。

如果 relation endpoint 无法通过 frame id association：

```text
本帧这条 relation 不进入 persistent graph
```

并记录 debug 即可。

---

# 10. Phase 7：Relation evidence merge

---

## 10.1 创建关系

第一次：

```text
obj_001 near obj_004
```

创建：

```json
{
  "edge_id": "obj_001__near__obj_004",
  "source_object_id": "obj_001",
  "target_object_id": "obj_004",
  "relation": "near",
  "confidence": 0.72,
  "observation_count": 1,
  "status": "TENTATIVE"
}
```

---

## 10.2 再次看到

第二次相同：

```text
obj_001 near obj_004
```

必须：

```text
observation_count += 1
confidence 融合
last_seen 更新
source_observation_ids 增加
status -> CONFIRMED
```

不能创建新 edge。

---

## 10.3 confidence 融合

第一版用 running weighted mean：

```python
new_conf = (
    old_conf * old_count + obs_conf
) / (old_count + 1)
```

即可。

---

## 10.4 description 去重

`descriptions_zh`：

```text
最多保留最近 N 条
```

推荐：

```text
N = 5
```

重复字符串不重复添加。

---

# 11. Phase 8：处理逆关系

这部分要避免图中产生混乱。

---

## 11.1 方向关系

如：

```text
left_of
right_of
contains
inside
above
below
in_front_of
behind
```

本身是有向关系。

可只保存视觉系统原始方向，不必自动生成 inverse edge。

例如只保存：

```text
obj_1 --left_of--> obj_2
```

前端不需要同时保存：

```text
obj_2 --right_of--> obj_1
```

---

## 11.2 对称关系

```text
near
adjacent_to
attached_to
```

语义上接近无向关系。

推荐 canonicalize：

```python
if relation in SYMMETRIC_RELATIONS:
    a, b = sorted([source_id, target_id])
```

使：

```text
obj_1 near obj_2
obj_2 near obj_1
```

都合并成一条：

```text
obj_1 --near-- obj_2
```

避免重复双边。

---

## 11.3 symmetric relation edge direction

snapshot 可以：

```json
{
  "directed": false
}
```

有向关系：

```json
{
  "directed": true
}
```

---

# 12. Phase 9：冲突关系处理

实时 VLM 可能出现：

```text
frame 1:
obj_1 left_of obj_2

frame 2:
obj_1 right_of obj_2
```

不要简单同时 CONFIRMED。

---

## 12.1 VIEW_RELATIVE

对于视角关系：

```text
允许不同时间有冲突
```

因为视角变了。

建议 edge 增加：

```text
last_observation_heading_sector
last_observation_pose
```

WebUI 默认只显示：

```text
最近 / confidence 最高的 view-relative edge
```

或者显示：

```text
左侧（当前视角）
```

---

## 12.2 STRUCTURAL 冲突

例如：

```text
inside
contains
```

出现明显反向冲突时：

```text
不要自动确认
```

记录：

```text
conflict_count
status=TENTATIVE
```

第一版不需要复杂 probabilistic graph。

---

# 13. Phase 10：SemanticEntityGraph snapshot 增加 object_topology projection

不要让前端自己从完整世界图里做复杂筛选。

---

## 13.1 新增函数

```python
def object_topology_snapshot(self) -> dict[str, Any]:
    ...
```

---

## 13.2 输出 schema

推荐：

```json
{
  "schema_version": "semantic_object_topology_v1",
  "revision": 18,
  "generated_at": 0,

  "nodes": [
    {
      "node_id": "obj_001",
      "node_type": "OBJECT",
      "label": "办公桌",
      "status": "CONFIRMED",
      "confidence": 0.91,
      "observation_count": 4,
      "spatial_quality": "METRIC_RGBD",
      "is_target_candidate": false,
      "is_target_confirmed": false
    }
  ],

  "edges": [
    {
      "edge_id": "obj_001__near__obj_002",
      "from": "obj_001",
      "to": "obj_002",
      "relation": "near",
      "relation_scope": "STRUCTURAL",
      "directed": false,
      "confidence": 0.84,
      "observation_count": 3,
      "status": "CONFIRMED",
      "description_zh": "办公桌靠近绿色垃圾桶"
    }
  ],

  "stats": {
    "node_count": 5,
    "edge_count": 4,
    "confirmed_nodes": 4,
    "confirmed_edges": 2,
    "connected_components": 1
  }
}
```

---

## 13.3 加入主 graph snapshot

当前：

```python
return {
    ...
    "nodes": nodes,
    "edges": ...,
    "route_plan": ...
}
```

增加：

```python
"object_topology": self.object_topology_snapshot()
```

不要改变：

```text
schema_version = semantic_entity_graph_v1
```

以免破坏兼容。

---

## 13.4 projection 节点过滤

默认：

```text
CONFIRMED
+
TENTATIVE
```

都输出。

STALE 可输出：

```text
status=STALE
```

由前端决定是否显示。

不要只输出 confirmed，否则刚出现的新物体在 UI 中完全不可见。

---

# 14. Phase 11：runner 把 `semantic.relations` 传入 EntityGraph

修改：

```text
scripts/go2w/run_semantic_exploration.py
```

当前：

```python
entity_graph.sync_from_observation(
    ...
    update_result=update_result,
)
```

改为：

```python
entity_graph.sync_from_observation(
    observation_id=observation.bundle_id,
    heading_sector=semantic.heading_sector,
    labels=observation.object_labels,
    spatial_objects=localized,
    pose=spatial_pose,
    timestamp=observation.timestamp,
    place_id=place_id,
    update_result=update_result,
    relations=list(getattr(semantic, "relations", None) or []),
)
```

如果当前 `semantic.relations` 已经是 dict list，不二次 `.to_dict()`。

---

# 15. Phase 12：Place attach object 也不要再靠 label 猜

当前 `SemanticEntityGraph._entry_touches()` 有：

```python
if observation_id in entry.source_observation_ids:
    return True
return entry.label in labels
```

这个 label fallback 对 persistent identity 同样危险。

本次建议同步修掉。

---

## 15.1 正确 attach 方式

`update_result.associations` 已经知道本帧有哪些 persistent objects。

直接：

```python
persistent_ids = [
    assoc.persistent_object_id
    for assoc in update_result.associations
]
```

然后：

```python
place_graph.attach_objects(place.place_id, persistent_ids)
```

不要遍历所有 object_map 再按 label 判断。

---

## 15.2 `_entry_touches()`

如果其他 legacy 使用，可保留。

但是当前：

```text
Place -> object attach
```

改成严格 association-based。

这也会让：

```text
OBSERVED_FROM
```

边更可靠。

---

# 16. Phase 13：SearchStateStore 不新增第二套状态

当前：

```text
spatial.semantic_graph
```

已经存在。

当前 map update 已能保存 semantic graph。

因此原则上：

```text
不新增 object_topology 到根 snapshot
```

前端从：

```javascript
state.spatial.semantic_graph.object_topology
```

直接读取。

---

## 16.1 可选便捷 alias

只有当前前端状态绑定特别复杂时，才增加：

```text
spatial.object_topology
```

但推荐避免重复 state。

---

# 17. Phase 14：WebUI 地图拆成两个显示模式

修改：

```text
app/manual_web_demo/static/search_map.js
```

以及对应 WebUI HTML/CSS 文件。

---

## 17.1 两个模式

顶部增加：

```text
[语义拓扑] [空间地图]
```

默认：

```text
语义拓扑
```

内部状态：

```javascript
this.mode = "semantic_topology";
```

可选：

```text
semantic_topology
spatial_map
```

---

## 17.2 用户切换后记住选择

当前 session 内：

```javascript
this.mode
```

即可。

可选：

```text
localStorage
```

但不是必要。

默认必须仍为：

```text
semantic_topology
```

---

# 18. Phase 15：重构 SearchMapRenderer

不要把现有 500+ 行 `render()` 继续堆逻辑。

---

## 18.1 推荐结构

```javascript
SearchMapRenderer.prototype.render = function (mapData, spatialData) {
    this.data = mapData || this.data || {};
    this.spatial = spatialData || this.spatial || {};

    if (this.mode === "semantic_topology") {
        this.renderObjectTopology(
            this.spatial &&
            this.spatial.semantic_graph &&
            this.spatial.semantic_graph.object_topology
        );
        return;
    }

    this.renderSpatialMap(this.data, this.spatial);
};
```

---

## 18.2 把当前 render body 移到

```javascript
renderSpatialMap()
```

尽量保持当前空间地图行为不变，避免导航调试能力回归。

---

## 18.3 新增

```javascript
renderObjectTopology(topology)
```

这个函数内部绝对不能调用：

```text
drawGrid
drawSpatialOverlay
drawRoute
metric computeLayout
map_xyz
frontier
occupancy
robot metric pose
```

---

# 19. Phase 16：语义拓扑 display layout

第一版使用稳定 BFS / layered layout。

不要先上 force simulation。

---

## 19.1 为什么不用 force-directed

实时图不断更新时 force simulation 容易：

```text
新节点出现
→ 所有旧节点重新漂移
→ 用户无法形成空间记忆
```

我们需要的是：

```text
稳定
可读
不跳
```

---

## 19.2 renderer 状态

在构造器：

```javascript
this.topologyPositions = {};
this.topologyComponentAnchors = {};
```

---

## 19.3 layout 输入

只读：

```text
topology.nodes
topology.edges
```

不读取：

```text
map_xyz
```

---

## 19.4 构图

构造 adjacency：

```javascript
var adjacency = {};
```

对于有向/无向 edge，布局阶段都可以当无向连接关系处理。

方向只用于 arrow。

---

## 19.5 connected components

求 connected components。

按以下优先级排序：

1. 包含 target confirmed；
2. 包含 target candidate；
3. node 数量多；
4. 最早稳定 ID。

---

## 19.6 component root

优先：

```text
target confirmed
```

其次：

```text
target candidate
```

否则：

```text
observation_count 最大节点
```

最后：

```text
node_id lexical smallest
```

这样布局稳定。

---

## 19.7 BFS 层

例如：

```text
root depth 0
neighbors depth 1
neighbors depth 2
```

推荐：

```javascript
LAYER_GAP_X = 180
ROW_GAP_Y = 90
NODE_WIDTH = 124
NODE_HEIGHT = 54
COMPONENT_GAP_Y = 150
```

不要硬编码散落在函数中，放常量。

---

## 19.8 position persistence

如果：

```text
obj_004
```

已经有 display position，且图结构没有根本变化：

```text
尽量保留旧位置
```

新节点：

```text
根据其邻接节点插入最近空槽
```

第一版可以：

```text
每次 component BFS
但 root/order deterministic
```

已经可以大幅减少跳动。

更好版本：

```text
已有 node 优先使用 cached y-order
```

---

# 20. Phase 17：孤立节点布局

一个新 object 可能暂时没有 relation。

不能丢掉。

在图底部增加：

```text
暂未建立关系
```

孤立节点横向/网格排列：

```text
obj_007  obj_009  obj_012
```

如果后来建立 relation：

```text
自动进入对应 connected component
```

---

# 21. Phase 18：节点视觉设计

当前小圆点 / 小菱形不适合关系拓扑。

改成卡片节点。

---

## 21.1 SVG node

例如：

```text
┌────────────────────┐
│ obj_004            │
│ 绿色垃圾桶         │
│ ✓ CONFIRMED · ×3   │
└────────────────────┘
```

建议：

```text
width = 124
height = 54
rx = 8
```

---

## 21.2 状态视觉

### CONFIRMED

```text
正常亮度
实线边框
```

### TENTATIVE

```text
较低 opacity
虚线边框
```

### STALE

```text
灰色
```

---

## 21.3 target

如果该 node 与当前搜索目标相关：

```text
加 ★ / ◎
```

不要改变 persistent id。

---

## 21.4 文本

至少：

```text
obj_004
绿色垃圾桶
CONFIRMED · ×3
```

label 超长：

```text
ellipsis / 截断
```

detail panel 显示完整名称。

---

# 22. Phase 19：关系边视觉设计

---

## 22.1 中文映射

```javascript
var RELATION_ZH = {
  near: "邻近",
  adjacent_to: "相邻",
  left_of: "左侧",
  right_of: "右侧",
  in_front_of: "前方",
  behind: "后方",
  on: "位于其上",
  under: "位于其下",
  above: "上方",
  below: "下方",
  in: "内部",
  inside: "内部",
  contains: "包含",
  attached_to: "连接",
  blocks: "阻挡",
};
```

---

## 22.2 edge label

中点显示：

```text
邻近 · ×3
```

hover/detail 可显示：

```text
confidence 0.84
```

默认不要塞太多数字导致画面乱。

---

## 22.3 directed

有向关系：

```text
left_of
contains
inside
...
```

画箭头。

无向关系：

```text
near
adjacent_to
attached_to
```

不画箭头。

---

## 22.4 relation_scope

### STRUCTURAL

实线。

### VIEW_RELATIVE

虚线。

并可显示：

```text
左侧 · 视角
```

---

## 22.5 tentative edge

透明度降低。

---

# 23. Phase 20：edge label 防重叠

简单第一版：

```text
label 放 edge midpoint
背景加半透明 rect
```

即：

```text
[节点]──── 邻近 ────[节点]
```

text 后放一个背景：

```javascript
<rect rx="3">
```

避免线穿过文字。

如果多条关系连接同一对节点：

```text
near
left_of
```

推荐合并显示成：

```text
邻近 · 左侧(视角)
```

或者 edge 轻微 offset。

第一版优先：

```text
同 pair relation label 合并
```

避免平行线过多。

---

# 24. Phase 21：点击节点 detail

点击 object node，右侧 detail 至少显示：

```text
Persistent ID
Label
Status
Confidence
Observation Count
First Seen
Last Seen
Seen From Places
Spatial Quality
map_xyz（如有）
关联关系
```

例如：

```text
关系：
→ obj_002 邻近 ×3
← obj_003 左侧（视角）×1
```

注意：

```text
map_xyz 只在 detail 中显示
```

不参与 topology layout。

---

# 25. Phase 22：点击 edge detail

如果实现成本合理，加 edge click。

显示：

```text
source
target
relation
relation_scope
status
confidence
observation_count
first_seen
last_seen
descriptions
```

如果当前 detail panel 只方便 node click，本项可排在节点 detail 后，但建议完成。

---

# 26. Phase 23：Topology UI 空状态

以下情况必须有清楚提示。

---

## 26.1 没有 object

```text
尚未识别到可建立语义拓扑的物体
```

---

## 26.2 有 object，无 relation

显示孤立 object，同时顶部提示：

```text
已识别 4 个物体，暂未获得可靠物体关系
```

不能空白。

---

## 26.3 只有 tentative relation

显示：

```text
关系仍在确认中
```

---

# 27. Phase 24：Spatial Map 继续保留

切换到：

```text
空间地图
```

继续展示当前：

```text
Occupancy
PlaceGraph
Robot
Frontier
Semantic Object map_xyz
Route
PSG
```

这部分不是本次要删除的东西。

---

## 27.1 可以顺手改 tab 文案

原“地图”如果含义模糊，可显示：

```text
语义拓扑
空间地图
```

顶部状态：

```text
拓扑节点 8 · 关系 11
```

空间模式：

```text
Map revision 31 · Place 5 · Frontier 4
```

---

# 28. Phase 25：不要把 Place 混入 object topology

语义拓扑视图默认节点只包含：

```text
OBJECT
```

不包含：

```text
PLACE
FRONTIER
ROBOT
TARGET_HYPOTHESIS
```

否则又会回到“所有节点堆在一起”。

Place/Object relation：

```text
OBSERVED_FROM
```

仍保留在 backend full `SemanticEntityGraph`。

只是：

```text
object_topology projection
```

不包含 Place。

---

# 29. Phase 26：关系可靠性过滤

---

## 29.1 最低 confidence

配置建议：

```text
relation_min_confidence = 0.45
```

低于：

```text
不进入 persistent relation
```

或者 debug only。

---

## 29.2 relation confirmation

建议：

```text
STRUCTURAL:
2 次确认

VIEW_RELATIVE:
1 次可显示但始终标记 view-relative
```

---

## 29.3 Stale

例如：

```text
relation_stale_after_seconds = 180
```

不要物理删除。

---

# 30. Phase 27：配置

在现有 Go2-W search config 中增加：

```yaml
semantic_topology:
  enabled: true
  default_web_view: true

  relation_min_confidence: 0.45
  relation_confirm_min_observations: 2
  relation_stale_after_seconds: 180

  include_tentative_objects: true
  include_stale_objects: true

  include_view_relative_relations: true

  max_relation_descriptions: 5

  layout:
    mode: layered_bfs
    layer_gap_x: 180
    row_gap_y: 90
    component_gap_y: 150
```

如果当前配置 loader 不方便把 UI layout 放 YAML：

```text
backend relation 参数放 YAML
frontend layout 常量放 JS
```

也可以。

不要为了参数化而大幅重写 config 系统。

---

# 31. Phase 28：Graph revision

每次发生以下任意情况：

```text
new persistent object
object updated
new persistent relation
relation updated
relation status changed
```

graph revision 应增加。

如果当前 `sync_from_observation()` 每 observation 都 revision +1，兼容即可。

`object_topology.revision` 直接使用：

```text
SemanticEntityGraph.revision
```

不要再维护第二个容易不同步的 revision。

---

# 32. Phase 29：关系调试信息

当前 graph 已有：

```text
association_debug
```

建议 relation unresolved 也写进去：

```json
{
  "type": "relation_association",
  "observation_id": "...",
  "subject_id": "semantic_obj_001",
  "object_id": "semantic_obj_002",
  "relation": "near",
  "result": "accepted",
  "persistent_source_id": "obj_001",
  "persistent_target_id": "obj_004"
}
```

失败：

```json
{
  "result": "rejected",
  "reason": "source_endpoint_unresolved"
}
```

这样以后出现“为什么这条边没有画出来”可以查。

---

# 33. Phase 30：离线视频关系能力复用原则

不要把整个 offline video mapping pipeline 接入 realtime。

应该复用的是：

```text
FrameObject schema
FrameRelation schema
relation vocabulary
ObservedSceneGraphBuilder 关系语义
```

而实时 persistent identity 仍由：

```text
SemanticObjectMap
```

负责。

---

## 33.1 正确职责

```text
LiveSemanticObserver
→ 当前帧 object/relation

SemanticObjectMap
→ object identity persistence

SemanticEntityGraph
→ relation persistence

WebUI
→ topology visualization
```

---

# 34. Phase 31：不要创建第二套 object tracker

当前 `LiveSemanticObserver` 内部调用：

```text
VideoObjectTracker().build_tracks([frame])
```

这个单帧 tracker 主要用于构造单帧 scene graph。

不要误以为它能承担跨实时帧 persistent entity identity。

跨帧 identity 应继续：

```text
SemanticObjectMap
```

负责。

---

# 35. Phase 32：ObjectTopology schema 测试

新增：

```text
tests/test_semantic_object_topology_schema.py
```

检查：

```text
schema_version
revision
nodes
edges
stats
```

Node：

```text
node_id startswith obj_
node_type == OBJECT
```

Edge endpoint：

```text
必须都存在于 nodes
```

---

# 36. Phase 33：Persistent relation 测试

新增：

```text
tests/test_semantic_entity_graph_relations.py
```

至少覆盖以下。

---

## Case 1：一帧关系正确 remap

Frame：

```text
semantic_obj_001 = desk
semantic_obj_002 = bin

semantic_obj_001 near semantic_obj_002
```

association：

```text
semantic_obj_001 -> obj_001
semantic_obj_002 -> obj_002
```

输出：

```text
obj_001 --near--> obj_002
```

---

## Case 2：第二帧 frame id 改变

Frame 2：

```text
semantic_obj_010 = desk
semantic_obj_011 = bin
```

association：

```text
semantic_obj_010 -> obj_001
semantic_obj_011 -> obj_002
```

relation：

```text
semantic_obj_010 near semantic_obj_011
```

最终：

```text
仍然 2 个 object nodes
仍然 1 条 near edge
edge observation_count == 2
```

---

## Case 3：同类别多个实体

```text
semantic_obj_001 = chair A
semantic_obj_002 = chair B
```

association：

```text
obj_004
obj_005
```

relation：

```text
A near B
```

输出必须：

```text
obj_004 near obj_005
```

不能通过 label collapse。

---

## Case 4：unresolved endpoint

如果：

```text
semantic_obj_009
```

没有 association：

```text
relation reject
graph 不 crash
```

---

## Case 5：self relation

两个 frame ids 被 entity association 合并到同一 persistent id：

```text
source == target
```

该 edge 必须 reject。

---

## Case 6：symmetric relation

```text
obj_1 near obj_2
obj_2 near obj_1
```

最终一条 edge。

---

## Case 7：directed relation

```text
obj_1 left_of obj_2
```

保留方向。

---

## Case 8：relation status

第一次：

```text
TENTATIVE
```

第二次：

```text
CONFIRMED
```

---

# 37. Phase 34：Runner integration test

新增：

```text
tests/test_run_semantic_exploration_relation_bridge.py
```

可 mock：

```text
semantic.relations
localized
update_result
```

验证：

```text
semantic.relations
→ sync_from_observation(relations=...)
```

而不是关系数据到这里丢失。

---

# 38. Phase 35：Web contract test

新增/扩展：

```text
tests/test_search_spatial_web_contract.py
```

最终 snapshot：

```python
snapshot["spatial"]["semantic_graph"]["object_topology"]
```

必须存在。

验证：

```text
nodes
edges
revision
```

---

# 39. Phase 36：Web renderer 测试

如果项目已有 JS test infrastructure，增加 JS unit test。

如果没有，不要为了这一项引入完整 Jest 工程。

至少可：

1. 写纯函数：
   ```javascript
   computeTopologyLayout(topology, previousPositions)
   ```
2. 抽成无 DOM 依赖。
3. 用简单 Node script / browser E2E 测试。

必须验证：

```text
相同 graph 输入
→ 相同 positions
```

和：

```text
所有 node positions 不完全重合
```

---

# 40. Phase 37：Mock E2E

当前 `--mock` 场景增加 deterministic semantic topology。

目标：

```text
绿色垃圾桶
```

Frame 1：

```text
desk
chair
bin

desk near bin
chair left_of desk
```

Frame 2：

```text
相同 desk/bin
frame_object_id 变化

desk near bin
```

预期：

```text
persistent nodes = 3
persistent relation edges = 2
near observation_count >= 2
```

WebUI：

```text
默认显示 object topology
```

---

# 41. Phase 38：WebUI Definition of Done

默认打开搜索页面后，在“地图”区域：

```text
默认是语义拓扑
```

---

## 必须满足

### 1

图中节点只显示：

```text
persistent OBJECT
```

### 2

不会显示：

```text
P1
P2
Frontier
Occupancy
Robot
Route
```

### 3

每个 node 显示：

```text
obj_xxx
label
status
obs count
```

### 4

object relation 是真实 backend 数据。

### 5

关系线上有中文 relation label。

### 6

重复识别同一个物体：

```text
不新建 node
```

### 7

重复识别同一个关系：

```text
不新建 edge
```

### 8

同 label 多物体：

```text
可以同时存在
```

### 9

节点屏幕位置不使用 map_xyz。

### 10

切换到“空间地图”后，原 spatial map 仍可使用。

---

# 42. Phase 39：后端 Definition of Done

---

## DoD 1

`DepthObjectLocalizer` 保留：

```text
frame_object_id
```

---

## DoD 2

runner 不再使用 label 决定 persistent id。

---

## DoD 3

`SemanticEntityGraph` 有 persistent：

```text
OBJECT -> OBJECT
```

relation store。

---

## DoD 4

`SemanticEntityGraph.snapshot()` 含：

```text
object_topology
```

---

## DoD 5

关系 endpoint 来自：

```text
SemanticAssociation.source_object_id
→ persistent_object_id
```

---

## DoD 6

relation repeated observation 可融合。

---

## DoD 7

graph 对 unresolved relation 不 crash。

---

# 43. 重点禁止的错误实现

---

## 禁止 1：前端根据 map_xyz 拉开节点

不允许：

```javascript
displayX = object.map_xyz[0] * scale
```

语义 topology 完全不能这样。

---

## 禁止 2：label 作为 identity

不允许：

```python
persistent_by_label["chair"]
```

用于 topology。

---

## 禁止 3：每帧 relation 直接 append

不允许：

```python
edges.append(new_relation_every_frame)
```

必须 upsert。

---

## 禁止 4：把 Place 也画进默认 object topology

这会重新造成图混乱。

---

## 禁止 5：删除空间地图

空间地图仍然对导航 debug 有价值。

---

## 禁止 6：前端自己猜关系

前端只显示 backend relation。

---

## 禁止 7：重新调用 LLM 生成 topology

当前 `scene_relations` 已经存在。

---

## 禁止 8：用同 label relation endpoint fallback

宁可 relation 暂时不显示，也不要错误连边。

---

# 44. 修改文件清单

执行 AI 必须首先检查下列文件。

---

## 44.1 必改

```text
app/perception/depth_object_localizer.py

app/spatial/semantic_entity_graph.py

scripts/go2w/run_semantic_exploration.py

app/manual_web_demo/static/search_map.js
```

---

## 44.2 很可能修改

WebUI 页面 HTML：

```text
app/manual_web_demo/templates/*
```

或当前实际搜索页面文件。

WebUI CSS：

```text
app/manual_web_demo/static/*
```

---

## 44.3 测试

建议新增：

```text
tests/test_depth_object_localizer_identity.py
tests/test_semantic_entity_graph_relations.py
tests/test_semantic_object_topology_schema.py
tests/test_live_persistent_id_mapping.py
tests/test_run_semantic_exploration_relation_bridge.py
tests/test_search_spatial_web_contract.py
```

按当前 tests 命名风格调整即可。

---

# 45. 推荐代码结构

`semantic_entity_graph.py` 最终大致：

```python
GRAPH_SCHEMA_VERSION = "semantic_entity_graph_v1"
OBJECT_TOPOLOGY_SCHEMA_VERSION = "semantic_object_topology_v1"

OBJECT_RELATIONS = {...}
RELATION_ALIASES = {...}
SYMMETRIC_RELATIONS = {...}
VIEW_RELATIVE_RELATIONS = {...}


@dataclass
class PersistentObjectRelation:
    ...


class SemanticEntityGraph:

    def __init__(...):
        ...
        self.object_relations = {}

    def sync_from_observation(..., relations=None):
        ...
        self._attach_observed_objects(...)
        self._sync_object_relations(...)
        self.revision += 1
        return self.snapshot()

    def _association_mapping(...):
        ...

    def _sync_object_relations(...):
        ...

    def _upsert_object_relation(...):
        ...

    def _mark_stale_relations(...):
        ...

    def object_topology_snapshot(...):
        ...

    def snapshot(...):
        ...
```

---

# 46. 推荐前端结构

`search_map.js` 最终大致：

```javascript
function SearchMapRenderer(svg, detailEl) {
  this.svg = svg;
  this.detailEl = detailEl;
  this.data = null;
  this.spatial = null;

  this.mode = "semantic_topology";

  this.topologyPositions = {};
}

SearchMapRenderer.prototype.setMode = function (mode) {
  ...
};

SearchMapRenderer.prototype.render = function (mapData, spatialData) {
  ...
};

SearchMapRenderer.prototype.renderObjectTopology = function (topology) {
  ...
};

SearchMapRenderer.prototype.renderSpatialMap = function (mapData, spatial) {
  // 当前 render 的空间图逻辑迁移到这里
};

function computeTopologyLayout(topology, previousPositions) {
  ...
}

function connectedComponents(nodes, edges) {
  ...
}

function relationLabelZh(relation) {
  ...
}

SearchMapRenderer.prototype.drawTopologyEdge = function (...) {
  ...
};

SearchMapRenderer.prototype.drawTopologyNode = function (...) {
  ...
};
```

---

# 47. 拓扑布局算法伪代码

```javascript
function computeTopologyLayout(topology, previousPositions) {
  const nodes = topology.nodes || [];
  const edges = topology.edges || [];

  const adjacency = buildAdjacency(nodes, edges);
  const components = findConnectedComponents(nodes, adjacency);

  sortComponentsDeterministically(components);

  let yOffset = 0;
  const positions = {};

  for (const component of components) {
    const root = selectStableRoot(component);
    const layers = bfsLayers(root, component, adjacency);

    const componentHeight = measureLayerHeight(layers);

    for (let depth = 0; depth < layers.length; depth++) {
      const layer = stableSortNodes(layers[depth], previousPositions);

      for (let row = 0; row < layer.length; row++) {
        const node = layer[row];

        positions[node.node_id] = {
          x: PAD + depth * LAYER_GAP_X,
          y: yOffset + PAD + row * ROW_GAP_Y,
        };
      }
    }

    yOffset += componentHeight + COMPONENT_GAP_Y;
  }

  layoutIsolatedNodes(...);

  return positions;
}
```

---

# 48. 图刷新策略

不要每个 WebSocket message 都让图闪烁。

---

## 48.1 revision check

当前 topology：

```text
revision
```

与 renderer last revision 一样：

```text
不重算 layout
```

---

## 48.2 只变 observation_count

如果 node/edge topology 结构没变：

```text
保留 positions
只刷新 label/status
```

---

## 48.3 graph structure changed

出现：

```text
new node
new edge
```

才重新计算 layout。

---

# 49. Topology fingerprint

前端可计算：

```javascript
fingerprint = JSON.stringify({
  nodes: nodeIds.sort(),
  edges: edgeKeys.sort()
})
```

只有 fingerprint 变才 recompute layout。

---

# 50. 真实关系与预测关系分开

本次 object topology 默认只画：

```text
observed relation
```

不画：

```text
PSG predicted relation
LLM hypothetical target relation
```

未来如果要画预测关系，必须：

```text
不同颜色/虚线
```

但不在本次主范围内。

---

# 51. 目标节点高亮

如果当前 search target 是：

```text
绿色垃圾桶
```

并且：

```text
obj_004
```

是当前 strong candidate / confirmed target：

节点显示：

```text
★ 目标
```

来源必须是当前已有 target match / object evidence。

不要前端仅靠：

```text
label.includes(target)
```

粗暴判断。

如果 backend 暂时没有 object-level target id，可先不实现强高亮，不得造假。

---

# 52. Session 输出建议

如果当前 run artifact 已经保存：

```text
semantic_entity_graph.json
```

确保里面包括：

```text
object_topology
```

可选增加：

```text
semantic_object_topology.json
```

但不强制。

不必保存第二套真值，只是 projection。

---

# 53. 性能要求

关系数量通常远小于 occupancy cells。

本次 renderer 应做到：

```text
50 object nodes
100 relation edges
```

仍能顺畅。

避免每次 update：

```text
复杂 O(N^3)
```

BFS/layout：

```text
O(V + E)
```

即可。

---

# 54. 安全要求

本次只改变：

```text
world graph relation persistence
WebUI display
```

不得改变：

```text
Go2-W motion command
emergency stop
operator supervision
motion limits
route execution
```

如果 topology relation 暂时错误：

```text
最多影响展示
```

本次默认不要直接让新 OBJECT relation 改 planner 权重。

先把图做稳定。

以后再单独讨论：

```text
relation-aware navigation
```

---

# 55. 推荐实施顺序

严格按以下顺序：

```text
STEP 1
拉最新 main
运行现有相关 tests

STEP 2
修 DepthObjectLocalizer frame_object_id

STEP 3
runner 建立 association_by_index / persistent_by_source_id

STEP 4
删除 topology path 的 label identity

STEP 5
SemanticEntityGraph 新增 PersistentObjectRelation

STEP 6
relation normalize / symmetric / scope

STEP 7
sync_from_observation 接入 relations

STEP 8
relation upsert / dedup / lifecycle

STEP 9
object_topology_snapshot

STEP 10
SearchStateStore / Web contract 确认透传

STEP 11
前端拆 renderObjectTopology / renderSpatialMap

STEP 12
实现 deterministic layered layout

STEP 13
节点卡片 + edge label + detail

STEP 14
加“语义拓扑/空间地图”切换

STEP 15
unit tests

STEP 16
mock E2E

STEP 17
browser manual verify

STEP 18
更新 README / docs
```

不要先改前端造假图。

---

# 56. 单元测试命令

按项目环境实际 Python 执行。

至少：

```bash
pytest -q \
  tests/test_depth_object_localizer_identity.py \
  tests/test_semantic_entity_graph_relations.py \
  tests/test_semantic_object_topology_schema.py \
  tests/test_live_persistent_id_mapping.py \
  tests/test_run_semantic_exploration_relation_bridge.py \
  tests/test_search_spatial_web_contract.py
```

然后跑现有相关：

```bash
pytest -q tests -k "semantic or spatial or search or place or route"
```

如果项目完整 suite 可接受：

```bash
pytest -q
```

---

# 57. 人工 Web 验证场景

---

## 场景 A：3 个不同物体

当前画面：

```text
办公桌
办公椅
绿色垃圾桶
```

关系：

```text
办公桌 near 垃圾桶
办公椅 left_of 办公桌
```

WebUI 必须是：

```text
obj_001
obj_002
obj_003
```

三张卡。

---

## 场景 B：同一个物体第二次出现

下一帧 frame ids 全变：

```text
semantic_obj_021
semantic_obj_022
semantic_obj_023
```

persistent association 仍：

```text
obj_001
obj_002
obj_003
```

WebUI：

```text
节点数量不增加
```

---

## 场景 C：同一关系第二次出现

`near` 再看到一次。

WebUI：

```text
edge 仍 1 条
×2
```

---

## 场景 D：两个相同 label

真实两个椅子：

```text
obj_004 chair
obj_005 chair
```

都必须同时存在。

---

## 场景 E：切换空间地图

点击：

```text
空间地图
```

仍能看到：

```text
RTAB
Robot
Place
Object map_xyz
Frontier
Route
```

---

# 58. 最终交付报告要求

Coding AI 完成修改后必须输出：

---

## A. Modified files

```text
modified:
...

new:
...
```

---

## B. Identity bridge 说明

明确写清：

```text
frame_object_id
→ ObjectSpatialObservation.object_id
→ SemanticAssociation.source_object_id
→ persistent_object_id
```

---

## C. Relation persistence 说明

明确写清：

```text
scene_relations
→ persistent endpoint remap
→ relation upsert
→ SemanticEntityGraph
→ object_topology
```

---

## D. UI 说明

明确写清：

```text
semantic topology
不使用 map_xyz

spatial map
继续使用 metric data
```

---

## E. Tests

输出：

```text
XX passed
```

失败测试不能隐藏。

---

# 59. 最终验收数据示例

后端：

```json
{
  "schema_version": "semantic_entity_graph_v1",
  "revision": 21,

  "object_topology": {
    "schema_version": "semantic_object_topology_v1",
    "revision": 21,

    "nodes": [
      {
        "node_id": "obj_001",
        "node_type": "OBJECT",
        "label": "办公桌",
        "status": "CONFIRMED",
        "confidence": 0.92,
        "observation_count": 4
      },
      {
        "node_id": "obj_002",
        "node_type": "OBJECT",
        "label": "绿色垃圾桶",
        "status": "CONFIRMED",
        "confidence": 0.88,
        "observation_count": 3
      },
      {
        "node_id": "obj_003",
        "node_type": "OBJECT",
        "label": "办公椅",
        "status": "CONFIRMED",
        "confidence": 0.85,
        "observation_count": 2
      }
    ],

    "edges": [
      {
        "edge_id": "obj_001__near__obj_002",
        "from": "obj_001",
        "to": "obj_002",
        "relation": "near",
        "relation_scope": "STRUCTURAL",
        "directed": false,
        "status": "CONFIRMED",
        "confidence": 0.86,
        "observation_count": 3
      },
      {
        "edge_id": "obj_003__left_of__obj_001",
        "from": "obj_003",
        "to": "obj_001",
        "relation": "left_of",
        "relation_scope": "VIEW_RELATIVE",
        "directed": true,
        "status": "TENTATIVE",
        "confidence": 0.72,
        "observation_count": 1
      }
    ]
  }
}
```

WebUI：

```text
┌───────────────┐
│ obj_003       │
│ 办公椅        │
│ CONFIRMED ×2  │
└───────┬───────┘
        │ 左侧 · 视角
        ▼
┌───────────────┐     邻近 ×3     ┌───────────────┐
│ obj_001       │ ─────────────── │ obj_002       │
│ 办公桌        │                 │ 绿色垃圾桶    │
│ CONFIRMED ×4  │                 │ CONFIRMED ×3  │
└───────────────┘                 └───────────────┘
```

---

# 60. 最重要的架构结论

本次绝对不要继续试图通过：

```text
调整 map_xyz 缩放
调整 SVG viewBox
放大 metric distance
```

来解决默认 WebUI 图“节点堆在一起”。

正确架构是：

```text
真实空间世界模型
=
RTAB + map_xyz + PlaceGraph + Route

语义物体拓扑显示
=
Persistent Object + Persistent Object Relation + Display Layout
```

最终要做到：

```text
同一个物体
→ 永远尽量保持同一个 obj_xxx

同一个关系
→ 永远尽量保持同一条 persistent edge

屏幕位置
→ 只是显示布局

真实物理坐标
→ 只用于导航和空间地图
```

---

# 61. 一句话完成标准

只有当系统满足下面这句话，才能宣布本次改造完成：

> **WebUI 默认显示一张稳定、可读、实时更新的“持久物体关系拓扑图”：节点是 persistent object，边是经 frame-object 到 persistent-object 映射后的真实物体关系；重复识别不会重复建节点或边；同类别多物体不会因 label 混淆；拓扑布局完全独立于 map_xyz，同时原有空间地图仍可切换查看。**
