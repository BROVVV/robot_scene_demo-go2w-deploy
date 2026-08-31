# robot_scene_demo 当前功能说明

本文档描述当前 `robot_scene_demo` 已实现的能力，目的是让另一个 AI 或工程师能快速理解现有 demo 的边界、数据流和可扩展点。

## 1. Demo 定位

当前项目是一个“单张图片 + 目标描述”的机器狗场景理解 Demo。它不是完整机器狗闭环系统，不接 ROS2、不控制真实机器狗、不做连续建图或 SLAM。

输入：

- 一张场景图片。
- 一个目标文本，例如“桌子上的手机”“黑色底座上的绿色物体”“挂着黄衣服的椅子”。

输出：

- 结构化场景 JSON：物体、关系、拓扑、目标判断、路线规划。
- 可选知识增强 JSON：任务解析、检索知识、预测性场景图、推理假设、任务规划、知识更新。
- 物体表 CSV。
- 关系表 CSV。
- 拓扑图 PNG 和 GraphML。
- 预测性场景图 GraphML。
- 带 bbox 的标注图 PNG。
- CLI 或 Streamlit 页面展示结果。

## 2. 入口和运行模式

### CLI

入口文件：`run_demo.py`

支持参数：

```bash
python run_demo.py --image path/to/image.jpg --target "桌子上的手机"
python run_demo.py --image path/to/image.jpg --target "桌子上的手机" --detector llm
python run_demo.py --image path/to/image.jpg --target "桌子上的手机" --detector grounded_sam
python run_demo.py --image path/to/image.jpg --target "找到手机" --enable-knowledge
python run_demo.py --mock
python run_demo.py --mock --enable-knowledge
```

模式：

- `--mock`：读取 `examples/mock_scene_result.json`，不调用 API，也不运行本地检测器。
- `--detector llm`：使用硅基流动 OpenAI-compatible 视觉大模型。
- `--detector grounded_sam`：使用 Grounding DINO + SAM2 本地检测路径。
- `--enable-knowledge`：启用知识库、预测性场景图、结构化假设、任务规划和知识更新输出。默认关闭，旧流程保持可用。
- 不传 `--detector` 时，使用 `.env` 中的 `DETECTION_BACKEND`，默认 `llm`。

### Streamlit

入口文件：`streamlit_app.py`

运行：

```bash
streamlit run streamlit_app.py
```

页面模式：

- 模拟数据。
- 真实 API。
- GroundingDINO+SAM2。
- 知识增强流程开关。
- 预测性场景图展示开关。

页面会显示上传图片、场景摘要、目标判断、路线规划、物体表、关系表、拓扑图、标注图和 JSON。
开启知识增强流程后，还会显示任务解析结果、检索知识、预测性场景图、推理假设、任务规划和知识更新。

## 3. 核心数据结构

定义文件：`app/schemas.py`

主输出模型是 `SceneAnalysisResult`：

```text
SceneAnalysisResult
├── scene_summary_zh: str
├── objects: list[SceneObject]
├── relations: list[SceneRelation]
├── topology: TopologyGraph
├── target_decision: TargetDecision
└── route_plan: RoutePlan
```

重要字段：

- `SceneObject`
  - `id`: 如 `obj_001`
  - `name`: 英文名
  - `name_zh`: 中文名
  - `category`: 类别
  - `color`: 颜色，可为 `None`
  - `attributes`: 属性和来源说明
  - `position`: 图像方位与估计距离
  - `bbox_2d`: 0 到 1 的归一化 bbox
  - `confidence`: 0 到 1
- `SceneRelation`
  - `source_id`
  - `target_id`
  - `relation_type`: 支持左右、前后、上下、包含、邻近、遮挡等
  - `description_zh`
  - `estimated_distance_m`
  - `confidence`
- `TargetDecision`
  - `target_text`
  - `is_present`
  - `matched_object_ids`
  - `match_reason_zh`
  - `confidence`
- `RoutePlan`
  - `route_type`: `approach_visible_target` 或 `explore_likely_location`
  - `summary_zh`
  - `steps`
  - `safety_notes_zh`

所有 Pydantic 模型默认 `extra="forbid"`，因此新增字段必须先更新 schema。

知识增强主输出模型是 `KnowledgeAwareSceneResult`，它不替换 `SceneAnalysisResult`，而是包在更高层：

```text
KnowledgeAwareSceneResult
├── base_scene: SceneAnalysisResult
├── parsed_task: RobotTask
├── retrieved_knowledge: list[KnowledgeItem]
├── predictive_scene_graph: PredictiveSceneGraph
├── hypotheses: list[SceneHypothesis]
├── reasoning_summary_zh: str
├── task_plan: TaskPlan
├── knowledge_updates: list[KnowledgeUpdate]
└── final_answer_zh: str
```

任务类型当前支持：

```text
find_object, count_objects, inspect_area, check_door_state,
find_room, navigate_to_location, verify_condition,
summarize_scene, compare_states
```

## 4. 两条场景理解路径

### 4.1 LLM 视觉路径

相关文件：

- `app/llm_clients/base.py`
- `app/llm_clients/siliconflow_client.py`
- `app/services/scene_analyzer.py`

流程：

1. `SiliconFlowVisionClient` 读取 `.env` 配置。
2. 将图片缩放到 `IMAGE_MAX_SIDE` 以内并转为 JPEG data URL。
3. 调用硅基流动 Chat Completions 接口。
4. 要求模型输出紧凑 JSON。
5. `extract_json_from_text()` 解析模型返回，兼容纯 JSON、Markdown 代码块和前后带文本的 JSON。
6. `_normalize_fast_result()` 将模型的简化 JSON 转成 `SceneAnalysisResult` 兼容结构。
7. `SceneAnalyzer` 用 Pydantic 校验结果并保存 `outputs/scene_result.json`。

LLM 路径特点：

- 物体、关系、目标判断、路线规划主要由视觉大模型一次性给出。
- 可开启 `ENABLE_LOW_OBJECT_RETRY`，当识别物体少于 `MIN_OBJECTS_FOR_COMPLEX_SCENE` 时追加提示重试一次。
- 推理模型来自 `SILICONFLOW_REASONING_MODEL`，视觉模型来自 `SILICONFLOW_VISION_MODEL`；当前视觉默认是 `Qwen/Qwen3-VL-8B-Instruct`。

### 4.2 Grounding DINO + SAM2 本地检测路径

相关文件：

- `app/detectors/base.py`
- `app/detectors/vocabulary.py`
- `app/detectors/grounded_sam_subprocess.py`
- `app/detectors/grounded_sam_worker.py`
- `app/services/detector_scene_builder.py`

流程：

1. `GroundedSAMSubprocessDetector` 根据目标文本构造开放词表 prompt。
2. 启动外部 Python 环境运行 `grounded_sam_worker.py`。
3. worker 调用 Grounding DINO 检测 bbox。
4. 如果 `ENABLE_SAM2=true`，再用 SAM2 预测 mask 面积比例。
5. detector 将外部 JSON 转成 `DetectedObject`。
6. `build_scene_from_detections()` 将检测结果转成 `SceneObject`。
7. 本地规则估计物体方位、距离、目标匹配和路线规划。
8. `relation_enricher` 补全空间关系和拓扑。

本地检测路径特点：

- 不调用大模型识别物体。
- 需要本机已有 Grounding DINO/SAM2 代码、权重和 Python 环境。
- 目标匹配和路线规划是规则版，只覆盖当前词表和少量目标组合。
- bbox 来自检测器，因此可以生成更有意义的标注图。

## 5. 结果后处理

### 5.1 标签标准化

文件：`app/services/scene_normalizer.py`

功能：

- 如果 `name_zh` 看起来还是英文，则用词表转中文。
- 如果类别是 `unknown`，按词表推断类别。
- 如果颜色为空，按标签关键词推断颜色。

词表文件：`app/detectors/vocabulary.py`

### 5.2 关系补全

文件：`app/services/relation_enricher.py`

功能：

- 去掉无效关系和重复关系。
- 根据 bbox 中心点或位置估计补全稀疏空间关系。
- 每个物体连接若干最近邻，默认每个源节点最多 2 个邻居。
- 若关系图存在多个连通分量，会添加最近跨分量关系，保证拓扑图尽量连通。
- 自动补全关系的中文描述会带有“根据图像位置自动补全”标记。

注意：这些关系不是模型显式识别，而是本地规则推断。

### 5.3 输出写入

文件：`app/services/output_writer.py`

统一输出流程：

1. `prepare_analysis_result()`
   - 标准化标签。
   - 补全关系和拓扑。
2. `write_analysis_outputs()`
   - 写 `scene_result.json`
   - 写 `object_table.csv`
   - 写 `relation_table.csv`
   - 写 `topology_graph.png`
   - 写 `topology_graph.graphml`
    - 如果有原图路径，写 `annotated_scene.png`

### 5.4 知识增强流程

文件：

- `app/services/knowledge_aware_analyzer.py`
- `app/services/knowledge_output_writer.py`
- `app/reasoning/task_parser.py`
- `app/reasoning/scene_reasoner.py`
- `app/planning/task_planner.py`
- `app/knowledge/kb_updater.py`

流程：

1. 调用旧 `SceneAnalyzer` 或读取 mock 得到 `SceneAnalysisResult`。
2. `parse_robot_task()` 把用户目标解析成 `RobotTask`。
3. `retrieve_relevant_knowledge()` 从本地知识库检索房间、物体位置和楼层布局知识。
4. `build_predictive_scene_graph()` 生成 PSG，把可见节点和假想节点放在同一图里。
5. `reason_about_scene()` 生成结构化假设和可解释推理摘要。
6. `plan_task()` 按任务类型生成 `TaskPlan`。
7. `update_knowledge_from_scene()` 追加观察记录，并只把稳定高置信事实合并进长期知识库。
8. `write_knowledge_aware_outputs()` 写出知识增强结果文件。

## 6. 输出文件说明

默认输出目录：`outputs/`

文件：

- `scene_result.json`
  - Pydantic 校验后的完整场景结构。
- `object_table.csv`
  - 物体表，包含 id、英文名、中文名、类别、颜色、属性、可见性、位置、bbox、置信度。
- `relation_table.csv`
  - 关系表，包含 source/target id、中文名、关系类型、中文描述、距离、置信度。
- `topology_graph.png`
  - NetworkX + Matplotlib 绘制的关系图。
- `topology_graph.graphml`
  - GraphML 格式拓扑图，便于后续图算法读取。
- `annotated_scene.png`
  - 在原图上画 bbox 和中文标签。
- `outputs/uploads/`
  - Streamlit 上传图片保存位置。
- `knowledge_aware_result.json`
  - 知识增强完整输出。
- `parsed_task.json`
  - 用户目标解析结果。
- `retrieved_knowledge.json`
  - 本次检索到的知识项。
- `predictive_scene_graph.graphml`
  - PSG GraphML，包含节点类型、可见性、置信度和推理来源。
- `hypotheses.json`
  - 结构化场景假设。
- `knowledge_updates.json`
  - 本次新增、确认、冲突或忽略的知识更新。
- `reasoning_report.md`
  - 面向阅读的推理报告。

## 7. 本地场景知识库

默认知识库目录：`data/scene_kb/`

文件：

- `floor_layout.json`
  - 楼层、走廊、门牌和房间布局。
- `room_type_priors.json`
  - 办公室、会议室等房间类型先验。
- `object_location_priors.json`
  - 手机、椅子等物体常见位置。
- `observations.jsonl`
  - 每次知识增强运行追加的观察记录。

查询脚本：

```bash
python scripts/query_scene_kb.py --target "手机" --room_type office --location floor_5
```

任务样例评估：

```bash
python scripts/evaluate_task_examples.py
```

## 8. 配置项

配置文件：`.env`，模板：`.env.example`

主要配置：

```text
SILICONFLOW_API_KEY=
SILICONFLOW_BASE_URL=https://api.siliconflow.cn/v1
SILICONFLOW_REASONING_MODEL=deepseek-ai/DeepSeek-V4-Flash
SILICONFLOW_VISION_MODEL=Qwen/Qwen3-VL-8B-Instruct
SILICONFLOW_TIMEOUT_SECONDS=25
SILICONFLOW_MAX_TOKENS=2048
IMAGE_MAX_SIDE=640
IMAGE_DETAIL=low
ENABLE_LOW_OBJECT_RETRY=false
MIN_OBJECTS_FOR_COMPLEX_SCENE=10
OUTPUT_DIR=outputs

DETECTION_BACKEND=llm
GROUNDED_SAM_ROOT=/home/user/python3.10.0/Grounded-SAM-2
GROUNDED_SAM_PYTHON=/home/user/python3.10/bin/python
GROUNDED_SAM_PYTHONPATH=/home/user/python3.10/lib/python3.10/site-packages
GROUNDING_DINO_CONFIG=grounding_dino/groundingdino/config/GroundingDINO_SwinT_OGC.py
GROUNDING_DINO_CHECKPOINT=gdino_checkpoints/groundingdino_swint_ogc.pth
GROUNDING_DINO_BOX_THRESHOLD=0.25
GROUNDING_DINO_TEXT_THRESHOLD=0.20
ENABLE_SAM2=true
SAM2_CONFIG=configs/sam2.1/sam2.1_hiera_t.yaml
SAM2_CHECKPOINT=checkpoints/sam2.1_hiera_tiny.pt
MAX_DETECTED_OBJECTS=30
DETECTION_DEVICE=auto
DETECTOR_TIMEOUT_SECONDS=60
```

## 9. 模块职责速查

```text
run_demo.py
  CLI 编排入口。

streamlit_app.py
  Web UI 编排入口。

app/config.py
  .env 配置读取和默认值。

app/schemas.py
  Pydantic 数据结构，是所有输出的结构契约。

app/llm_clients/siliconflow_client.py
  硅基流动视觉模型调用、图片压缩、返回 JSON 归一化。

app/detectors/
  本地开放词表检测器接口、词表、Grounding DINO/SAM2 子进程封装和 worker。

app/services/scene_analyzer.py
  在 LLM client 和 object detector 两种路径之间做统一编排。

app/services/knowledge_aware_analyzer.py
  串联原场景分析、任务解析、知识检索、PSG、推理、任务规划和知识更新。

app/services/psg_builder.py
  构建预测性场景图并导出 GraphML。

app/services/knowledge_output_writer.py
  写出知识增强结果 JSON、GraphML 和推理报告。

app/knowledge/
  本地场景知识库 schema、读写、检索和更新。

app/reasoning/
  任务解析、假设生成、证据评分、验证动作和推理摘要。

app/planning/
  按任务类型生成高层任务计划。

app/services/detector_scene_builder.py
  把本地检测结果转成 SceneAnalysisResult，并做规则目标匹配和路线规划。

app/services/relation_enricher.py
  本地补全空间关系和 topology。

app/services/scene_normalizer.py
  中文名、类别、颜色标准化。

app/services/output_writer.py
  统一写出 JSON、CSV、拓扑图和标注图。

app/services/table_exporter.py
  生成物体表和关系表。

app/services/topology_builder.py
  生成 NetworkX 图、PNG 和 GraphML。

app/services/image_annotator.py
  在原图上绘制 bbox 和标签。

app/services/target_matcher.py
  格式化目标判断文本。

app/services/route_planner.py
  格式化路线规划文本。

app/utils/json_utils.py
  从模型文本中提取 JSON。
```

## 10. 当前限制

- 仍是单图 Demo，不接 ROS2、不控制真实机器狗、不做连续 SLAM。
- 知识库是本地 JSON/JSONL，不是多机器人共享数据库，也不是向量库。
- PSG 和假设生成是规则版，不是端到端具身智能策略。
- 运行中知识更新能追加观察并合并部分稳定事实，但还没有多次观测一致性学习或冲突老化策略。
- LLM 路径的 bbox 是归一化占位框，通常不能用于精确定位；Grounding DINO/SAM2 路径 bbox 更可靠。
- Grounding DINO/SAM2 路径只做物体检测，关系、目标匹配和路线规划主要是规则推断。
- 距离是图像启发式估计，不是真实深度测量。
- `RoutePlan` 和 `TaskPlan` 都是文本/结构化计划，不是可执行运动控制。

## 11. 后续扩展建议

后续更适合继续做：

1. 把 `observations.jsonl` 升级为 SQLite 或图数据库。
2. 引入真实定位和多帧 episode memory。
3. 增加更细的门状态、房间类型和遮挡区域识别。
4. 将 `TaskPlan` 转成 ROS2/Nav2 可执行动作前，先接入深度和避障验证。
5. 给 LLM 任务解析和假设生成增加可选 provider，但保留规则 fallback。
