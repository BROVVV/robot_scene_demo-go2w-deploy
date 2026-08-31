# 预测性场景图与 PredictVLM 改进方案

## 目标

在当前 `robot_scene_demo` 中加入“预测性场景图（Predictive Scene Graph, PSG）”和 PredictVLM 的最小可运行原型。

当前 demo 已经能输出：

- 已观测物体 `objects`
- 已观测/补全关系 `relations`
- 拓扑图 `topology_graph`
- 目标判断和路线规划

下一步目标不是推倒重写，而是在现有结果之上增加一层 PSG，让图中同时包含：

- `OBSERVED`：当前图片中实际看到的节点
- `IMAGINED`：PredictVLM 或规则推测存在但尚未验证的节点
- `VERIFIED`：后续验证通过的假想节点
- `REFUTED`：后续证伪的假想节点

低延迟原则：

- 单张图片识别仍只调用一次视觉 API 或本地检测器。
- PredictVLM 默认不自动调用，由用户点击“生成想象节点”触发。
- 第一版提供“规则快速想象”模式，不调用 API。
- 拓扑连通继续由本地关系补全完成，不让 VLM 枚举所有边。

---

## 第一步：扩展数据结构

修改：

```text
app/schemas.py
```

新增枚举：

```text
NodeStatus = OBSERVED | IMAGINED | VERIFIED | REFUTED
EdgeStatus = OBSERVED | INFERRED
EdgeRelationClass = SPATIAL | SEMANTIC | TOPOLOGICAL
```

新增模型：

```text
NodeSource
PositionDistribution
PSGNode
PSGEdge
PredictiveSceneGraph
ImaginedEntity
ImaginedRelation
PredictVLMResult
```

`PSGNode` 字段：

```text
id
type
label_zh
status
position
embedding_text
embedding_visual
confidence
source
goal_relevance
bbox_2d
observed_object_id
expected_to_contain
rationale
```

`PSGEdge` 字段：

```text
source
target
relation_type
relation_class
strength
status
description_zh
```

第一版要求：

- `embedding_text` 和 `embedding_visual` 允许为 `null`。
- `position` 先支持方向和估计距离，不实现粒子集。
- 不接 CLIP，不接 BEV，避免第一版变慢。

---

## 第二步：把现有结果转换成 PSG

新增：

```text
app/services/psg_builder.py
```

实现：

```python
build_psg_from_scene(result: SceneAnalysisResult) -> PredictiveSceneGraph
```

转换规则：

- 每个 `SceneObject` 转成一个 `PSGNode`。
- 节点 `status=OBSERVED`。
- 节点 `type=obj.name`。
- 节点 `label_zh=obj.name_zh`。
- 节点 `confidence=obj.confidence`。
- 节点 `observed_object_id=obj.id`。
- 每个 `SceneRelation` 转成一个 `PSGEdge`。
- 空间关系边：
  - `relation_class=SPATIAL`
  - `status=OBSERVED`
  - `strength=relation.confidence`

这一阶段不调用任何 VLM。

---

## 第三步：定义 PredictVLM Prompt

新增：

```text
app/prompts_predictvlm.py
```

PredictVLM 输入：

- 当前 PSG 的节点列表
- 当前 PSG 的边列表
- 用户目标描述
- 当前场景摘要
- 可选：REFUTED 节点列表

PredictVLM 输出 JSON：

```json
{
  "imagined_entities": [
    {
      "type": "doorway",
      "label_zh": "门口",
      "rationale": "画面右侧有柜子和通道边缘，可能通向另一区域",
      "estimated_direction_deg": 30,
      "estimated_distance_m": 3.0,
      "confidence": 0.62,
      "expected_to_contain": ["room", "storage"]
    }
  ],
  "imagined_relations": [
    {
      "source": "obj_003",
      "target": "imagined_001",
      "relation_type": "reachable_through",
      "relation_class": "TOPOLOGICAL",
      "description_zh": "可从桌子右侧通道到达假想门口",
      "strength": 0.6
    }
  ],
  "target_likelihood_per_imagined": {
    "imagined_001": 0.7
  }
}
```

Prompt 约束：

- 不要重复生成已观测节点。
- 只生成 2 到 5 个最有价值的假想节点。
- 每个假想节点必须可验证。
- 每个假想节点必须给出 `rationale`。
- 优先低延迟，输出短 JSON。
- 只输出 JSON，不输出 Markdown。

---

## 第四步：实现 PredictVLM 客户端

新增：

```text
app/llm_clients/predict_vlm_client.py
```

接口：

```python
class PredictVLMClient:
    def predict_imagined_nodes(
        self,
        psg: PredictiveSceneGraph,
        target_text: str,
    ) -> PredictVLMResult:
        ...
```

实现要求：

- 复用现有硅基流动 OpenAI-compatible 配置。
- 第一版只传文本 PSG，不传图片。
- 使用较小 `max_tokens`。
- 设置请求超时。
- 解析失败时输出原始文本，方便调试。

这样 PredictVLM 调用不会拖慢主识别流程。

---

## 第五步：把想象结果写入 PSG

新增：

```text
app/services/psg_imagination.py
```

实现：

```python
apply_predictvlm_result(
    psg: PredictiveSceneGraph,
    predict_result: PredictVLMResult,
) -> PredictiveSceneGraph
```

规则：

- 每个 `imagined_entity` 变成 `PSGNode`。
- 节点 id 使用：

```text
imagined_001
imagined_002
...
```

- 节点 `status=IMAGINED`。
- 节点 `source.method=PredictVLM`。
- 节点 `source.rationale=rationale`。
- 节点 `position` 使用方向 + 距离表达。
- 节点 `goal_relevance` 来自 `target_likelihood_per_imagined`。
- 每个 `imagined_relation` 变成 `PSGEdge`。
- imagined relation 的 `status=INFERRED`。

---

## 第六步：让拓扑图支持 PSG

修改：

```text
app/services/topology_builder.py
```

新增：

```python
build_psg_topology_graph(psg: PredictiveSceneGraph)
export_psg_topology_graph(psg, output_dir)
```

显示规则：

- `OBSERVED` 节点：蓝色
- `IMAGINED` 节点：浅黄色
- `VERIFIED` 节点：绿色
- `REFUTED` 节点：灰色或红色
- `OBSERVED` 边：实线
- `INFERRED` 边：虚线

第一版如果 Matplotlib 难以实现虚线节点，至少用颜色区分节点状态。

---

## 第七步：导出 PSG 表格和 JSON

新增：

```text
app/services/psg_exporter.py
```

输出文件：

```text
outputs/predictive_scene_graph.json
outputs/psg_nodes.csv
outputs/psg_edges.csv
outputs/psg_graph.png
outputs/psg_graph.graphml
```

`psg_nodes.csv` 字段：

```text
id
type
中文名
status
confidence
goal_relevance
direction_deg
distance_m
source_method
rationale
expected_to_contain
observed_object_id
```

`psg_edges.csv` 字段：

```text
source
target
relation_class
relation_type
status
strength
description_zh
```

---

## 第八步：Streamlit 增加 PSG 页面

修改：

```text
streamlit_app.py
```

新增按钮：

```text
生成想象节点
```

新增 tab：

```text
预测性场景图
PSG节点表
PSG边表
```

页面流程：

1. 用户先运行“真实 API”或 “GroundingDINO+SAM2”。
2. 系统得到 observed scene result。
3. 系统构建 observed PSG。
4. 用户点击“生成想象节点”。
5. 调用规则快速想象或 PredictVLM。
6. 展示包含 `OBSERVED + IMAGINED` 的 PSG。

默认不要在“开始分析”时自动调用 PredictVLM。

---

## 第九步：增加无 API 的规则版想象

新增：

```text
app/services/rule_based_predictor.py
```

接口与 PredictVLM 保持一致：

```python
predict_imagined_nodes(psg, target_text) -> PredictVLMResult
```

规则示例：

- 看到 `door / doorway / corridor / entrance` → imagined room
- 看到 `desk + monitor + computer` → imagined office_area
- 看到 `chair + clothing` → imagined storage_area 或 nearby_personal_item_area
- 目标是 `手机`，且看到桌子 → imagined phone_on_table_area
- 目标是 `书包`，且看到椅子或门口 → imagined bag_near_chair_or_door

Streamlit 中增加选择：

```text
想象方式：规则快速 / PredictVLM
```

默认使用“规则快速”，保证最低延迟。

---

## 第十步：加入验证状态机最小版

新增：

```text
app/services/psg_verifier.py
```

接口：

```python
verify_imagined_nodes(
    old_psg: PredictiveSceneGraph,
    new_scene: SceneAnalysisResult,
) -> PredictiveSceneGraph
```

第一版规则：

- 如果新观测中出现 imagined node 的 `type` 或 `expected_to_contain`：
  - `IMAGINED -> VERIFIED`
- 如果用户手动点击“证伪”：
  - `IMAGINED -> REFUTED`

Streamlit 中给 imagined nodes 提供按钮：

```text
验证通过
证伪
```

这一版先手动模拟闭环，不接真实机器人运动。

---

## 第十一步：目标图最小版

新增：

```text
app/services/goal_graph_builder.py
```

第一版只支持文本目标。

示例：

```text
挂着黄衣服的椅子
```

解析为：

- node: chair
- node: yellow_clothing
- edge: clothing on/near/occluding chair

输出：

```text
GoalGraph
```

第一版不要接 CLIP，先用关键词匹配。

---

## 第十二步：增加决策摘要

新增：

```text
app/services/psg_decision.py
```

输入：

```text
PredictiveSceneGraph
target_text
```

输出：

```json
{
  "observed_match_score": 0.4,
  "full_match_score": 0.75,
  "prediction_gain": 0.35,
  "mode": "VERIFY_IMAGINATION",
  "recommended_node_id": "imagined_001",
  "reason_zh": "已观测节点无法直接匹配目标，但假想节点与目标相关度高"
}
```

模式：

```text
VERIFY_IMAGINATION
REFINE_OBSERVED
RESTART_IMAGINATION
EXPLORE
```

第一版只展示决策摘要，不控制真实机器人。

---

## 推荐实施顺序

1. 扩展 schema：加入 PSGNode、PSGEdge、PredictiveSceneGraph。
2. 实现 `build_psg_from_scene`。
3. 实现 PSG JSON、CSV、Graph 导出。
4. Streamlit 展示 observed PSG。
5. 实现 rule-based imagined nodes。
6. Streamlit 展示 imagined nodes。
7. 实现 PredictVLM prompt 和 client。
8. 加“规则快速 / PredictVLM”切换。
9. 加目标图最小版。
10. 加 observed/full/prediction_gain 分数。
11. 加手动 VERIFIED/REFUTED 状态转换。
12. 最后再考虑自动验证闭环和长期记忆。

---

## 第一版不要做的内容

为了保持低延迟和实现可控，第一版不要做：

- CLIP embedding
- 粒子集
- BEV 投影
- 长期记忆库
- 自动机器人闭环验证
- 多轮连续导航
- 每次分析都自动调用 PredictVLM

---

## 第一版必须做的内容

第一版必须实现：

- PSG 数据结构
- `OBSERVED + IMAGINED` 节点共存
- `INFERRED` 边
- 规则版 imagined nodes
- 可选 PredictVLM imagined nodes
- PSG 节点表
- PSG 边表
- PSG 拓扑图
- Streamlit 中能看到“已观测图”和“预测性场景图”的差异

---

## 验收标准

完成后至少要能做到：

1. 用模拟数据运行 demo。
2. 生成 observed PSG。
3. 点击“生成想象节点”后，出现至少 2 个 `IMAGINED` 节点。
4. `psg_nodes.csv` 中能看到 `OBSERVED` 和 `IMAGINED` 两类节点。
5. `psg_edges.csv` 中能看到 `OBSERVED` 和 `INFERRED` 两类边。
6. `psg_graph.png` 中能用颜色区分 observed 和 imagined。
7. 不点击“生成想象节点”时，不产生额外 PredictVLM 调用。
8. 默认“规则快速”模式不调用 API。
9. PredictVLM 模式调用失败时，页面能显示错误，不影响已有 observed result。
