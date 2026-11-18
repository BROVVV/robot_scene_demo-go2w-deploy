# Go2-W YOLOE-Seg 双入口可选感知路线实施计划书

> 日期：2026-10-27  
> 目标：在不删除、不替换现有 GroundingDINO + RAM++ + SAM2 路线的前提下，新增 `yoloe_seg` 作为可选的本地开放词汇实例分割后端，并同时接入“自主搜索”与“实体遥控器驾驶时的被动语义建图”。  
> 本轮性质：增量接入方案，不改动运动架构、Nav2 职责、VLM 调度策略或现有感知节拍。

---

## 1. 结论与实施原则

该改造可行，且与当前工程的数据契约兼容。YOLOE-Seg 能同时给出类别、置信度、二维框和实例 mask；只要将输出转成项目现有的 `DetectedObject` 格式，后续的 RGB-D 深度定位、三维物体框、持久对象合并、语义拓扑和 WebUI 可视化都可继续复用。

实施时遵守五条原则：

1. `yoloe_seg` 是第三条可选路线，不是对 `grounded_sam` 或 `llm` 的就地替换。
2. 两个业务入口共用同一个 YOLOE-Seg 适配层和同一份输出契约，不各写一套转换逻辑。
3. 每个会话启动前选定一个检测后端；运行中不热切换模型，避免对象 ID、阈值和延迟统计失去可比性。
4. 选择 YOLOE-Seg 后若模型、CUDA 或依赖不可用，必须显式报错，不得静默退回 Grounded-SAM、VLM 或 CPU。
5. 不把检测后端选择与运动权限绑在一起。尤其是遥控建图模式，新路线仍只读传感器并写语义结果，不发送任何运动指令。

---

## 2. 本轮范围

### 2.1 本轮要实现

- 新增 `yoloe_seg` 检测后端，输出真实实例 mask，并适配现有 `DetectedObject` 契约。
- 自主搜索可在 `llm`、`grounded_sam`、`yoloe_seg` 之间显式选择。
- 实体遥控器被动建图可在 `grounded_sam`、`yoloe_seg` 之间显式选择。
- 复用现有目标词、关系锚点词、场景词调度和 `detection_slots`，保持自主搜索的目标/锚点/场景职责分离。
- 复用遥控建图的 `detection_schedule`、危险词优先级、深度定位、史莱姆颜色增强、持久对象合并和拓扑发布。
- 记录实际生效的后端、模型、阈值、设备、精度、prompt 签名、推理耗时和回退状态。
- 提供离线回放对比和分阶段真机验收，可随时通过配置切回 `grounded_sam`。

### 2.2 本轮明确不做

以下是先前已认可的下一阶段架构方向，本轮暂不改动：

> 高频安全控制由 Nav2/LiDAR 持续负责；GroundingDINO 做低成本、事件触发的目标巡检；VLM 只在“战略决策点”使用；Agent 只选地图目标，不直接发运动指令。

因此本轮不修改：

- “走一步、识别一次”的现有触发时机与检测分频策略；
- VLM 调用时机、复核流程和调用预算；
- Agent 的长程目标决策方式；
- Nav2 全局/局部路径规划、避障和控制链；
- 前沿点距离、路口/分岔点候选生成逻辑；
- RAM++ 的现有 Grounded-SAM 功能。YOLOE-Seg 线路不要求 RAM++，但也不从旧线路删除它。

---

## 3. 现状与真正的接入点

### 3.1 自主搜索

当前链路为：

```text
Web/API SearchStartRequest.detector
  -> search_session_service
  -> autonomous_search_worker.py --detector
  -> run_semantic_exploration.py
  -> run_autonomous_loop.py::_detect()
  -> GroundedSAM 或云端 VLM
```

现有 `--detector` 只允许 `llm` 和 `grounded_sam`。除了扩展参数枚举，还必须处理三类硬编码：

- `run_semantic_exploration.py` 用 `detector == "grounded_sam"` 决定是否走本地结构化物体分支；
- `run_autonomous_loop.py` 的检测器缓存、构造、关闭和诊断都使用 `_gsam_detector` / `_ensure_gsam_detector` 语义；
- 场景物体整理分支也只把 `grounded_sam` 当作“本地有框、有 mask 的检测器”。

如果只在 argparse 中增加 `yoloe_seg`，运行时会落入错误分支，看似参数被接受，实际不会走 YOLOE-Seg。

### 3.2 遥控器被动建图

当前链路为：

```text
start_remote_control_mapping.sh
  -> remote_control_semantic_mapping_worker.py
  -> GroundedSAMSubprocessDetector
  -> DepthObjectLocalizer
  -> SemanticObjectMap
  -> 语义物体/拓扑/WebUI
```

这条链路目前没有检测器参数，worker 直接构造 Grounded-SAM，启动脚本的 Python、模型路径、GPU 和 smoke test 预检也都是 Grounded-SAM 专用的。因此遥控建图不能只改 worker，必须同时改启动参数、预检、状态发布和冒烟测试。

### 3.3 可复用的下游契约

`app/detectors/base.py::DetectedObject` 已包含新路线所需的主要字段：

```text
label / label_zh / category
bbox_2d / score / text_score
mask_rle / mask_size / mask_area_ratio / mask_source
source / source_prompt_term / detection_slots
```

`DepthObjectLocalizer` 已优先使用 mask 计算三维位置和包围盒，mask 不可用时才回退到 bbox。因此新路线的关键不是重写后续流水线，而是严格完成 YOLOE-Seg 到 `DetectedObject` 的适配。

---

## 4. 目标架构

```text
                                      +--------------------+
自主搜索词槽 target/anchor/scene --->|                    |
                                      | DetectorFactory    |
遥控建图场景词/危险词 ----------->| + capability table |
                                      +---------+----------+
                                                |
                   +----------------------------+-------------------------+
                   |                            |                         |
                   v                            v                         v
             VLM detector          Grounded-SAM adapter          YOLOE-Seg adapter
                                  DINO + RAM++ + SAM2        persistent YOLOE worker
                   |                            |                         |
                   +----------------------------+-------------------------+
                                                |
                                                v
                                      list[DetectedObject]
                                                |
                      +-------------------------+---------------------+
                      |                                               |
                      v                                               v
              自主搜索目标/场景分支                         遥控语义建图
                      |                                               |
                      +------------> DepthObjectLocalizer <----------+
                                            |
                                  3D 物体 / 拓扑 / WebUI
```

检测器不只用名称分支，而是声明能力：

| 能力 | `llm` | `grounded_sam` | `yoloe_seg` |
| --- | --- | --- | --- |
| 开放词汇目标 | 是 | 是 | 是 |
| 结构化 bbox | 有，但不适合当稳定几何源 | 是 | 是 |
| 实例 mask | 否 | 是 | 是 |
| 本地推理 | 否 | 是 | 是 |
| RAM++ 场景扩词 | 不适用 | 可选 | 第一版不需要 |
| 支持 target/anchor/scene 词槽 | 通过 VLM 契约 | 是 | 是 |

建议新增一个集中的 backend 定义与 factory，避免在多个入口重复写字符串判断。最低能力接口为：

```python
detect(image_path, target_text)
detect_with_dynamic_terms(
    image_path,
    target_text,
    dynamic_terms,
    scene_terms=None,
    *,
    frame_id="",
    slots=None,
)
close()
```

对不支持的可选能力（例如 YOLOE-Seg 第一版不运行 RAM++ tagging），由 capability 显式说明，不通过 `hasattr` 或异常猜测。

---

## 5. YOLOE-Seg 适配设计

### 5.1 模型运行方式

使用独立常驻子进程加载 YOLOE-Seg，不在每帧中重新 import、加载权重或构建文本特征。主进程与 worker 使用带 `request_id` 的 JSON Lines 协议，最少支持：

- `health`：返回模型、设备、精度、库版本和显存状态；
- `detect`：输入图像路径、词表、阈值、帧号和词槽，返回框、mask 和诊断；
- `shutdown`：有序释放 CUDA 上下文和子进程。

当 prompt 签名未变时复用已编码的文本词表；目标、词槽或场景轮换词变化时，才更新 YOLOE 词表。缓存键必须包含排序去重后的词表、词槽与模型 ID。

### 5.2 输出转换契约

每个 YOLOE-Seg 实例转成 `DetectedObject` 时，按下列规则处理：

- `bbox_2d`：转为 `[0, 1]` 归一化的 `(x1, y1, x2, y2)`，严格裁剪到图像范围；
- `score`：保留 YOLOE 最终置信度；`text_score` 只在后端能严格给出独立文本相似度时填写，否则为 `None`；
- `label`：使用实际命中的 prompt term，并通过现有 vocabulary 解析 `label_zh`、`category` 和颜色；
- `source_prompt_term`：记录 YOLOE 输出类别索引对应的原始 prompt，不能简单写成泛化的 `object`；
- `detection_slots`：用与 Grounded-SAM 相同的规则，记录该 prompt 属于 `target`、`anchor`、`scene` 中的哪些槽；
- `mask_rle`：转为项目现有的行段编码 `[row, start_x, length]`，不引入第二种 RLE 格式；
- `mask_size`：严格使用 mask 原分辨率 `(height, width)`，与 RGB-D 对齐后才交给深度定位；
- `mask_area_ratio`：由二值 mask 像素数除以全图像素数；
- `mask_source="yoloe_seg"`，`source="yoloe_seg"`，attributes 中记录模型标识和分割来源。

如 mask 缺失、尺寸不合法或与图像不对齐，该实例可保留 bbox 作为降级观测，但必须在诊断中计数，不得伪造 mask。

### 5.3 词表与词槽

自主搜索继续使用 `TargetProfile` 产生的英文检测词、颜色组合词、上下文词和关系锚点词。YOLOE-Seg 不自行重新解析中文任务，以免与现有 TargetProfile 出现两套不同词义。

遥控建图没有当前搜索目标，继续使用 `semantic_mapping.yaml` 中的：

```text
priority_terms + 当前 scene_profile 轮换组 + hazard prompt_terms
```

第一版 YOLOE-Seg 不调用 RAM++。当 `use_ram_scene_terms=false` 时行为与现有工业园配置一致；未来如需要动态扩词，再独立评估 RAM++ 带来的额外延迟和误检，不纳入此次接入。

### 5.4 进程、GPU 与依赖隔离

YOLOE-Seg 的 Python/Ultralytics/Torch 版本可能与 Grounded-SAM-2 环境不同，因此使用独立解释器，不直接改写现有 Grounded-SAM 环境。自主搜索需要动态设置目标词，第一版必须选用支持文本 prompt 的 `*-seg.pt`，不能误用不接受 `set_classes()` 的 `*-seg-pf.pt`。建议配置项：

```dotenv
YOLOE_PYTHON=/absolute/path/to/yoloe-env/bin/python
YOLOE_MODEL=/absolute/path/to/yoloe-seg-checkpoint.pt
YOLOE_DEVICE=cuda:0
YOLOE_PRECISION=fp16
YOLOE_CONFIDENCE=0.25
YOLOE_IOU_THRESHOLD=0.50
YOLOE_IMAGE_SIZE=640
YOLOE_MAX_DETECTIONS=100
YOLOE_TIMEOUT_SECONDS=30
```

模型权重、文本编码器、tokenizer/CLIP 依赖在部署阶段预先下载，并对关键权重记录 SHA-256。官方文档明确说明，文本 prompt 模式首次 `set_classes()` 可能另外获取文本编码器，所以只预下载 `.pt` 文件不足以保证离线运行。真机会话中不允许临时联网下载。启动预检必须验证：解释器可执行、依赖可 import、权重和文本编码器齐备、CUDA 真正可初始化、模型能对一组自定义词完成小图 warm-up。

---

## 6. 自主搜索接入计划

### 6.1 选择入口

保留现有 `detector` 字段，合法值扩展为：

```text
llm | grounded_sam | yoloe_seg
```

选择优先级保持现有语义：

```text
本次搜索请求 > AUTONOMOUS_SEARCH_DEFAULT_DETECTOR > YAML > 内置默认
```

实施期间不更改默认值，生产配置仍保持 `grounded_sam`。WebUI 在“高级显示”内增加检测路线选择器，启动任务后锁定，会话页和历史页始终显示实际生效值。

### 6.2 代码接线

- `SearchStartRequest.validate()` 和 `run_semantic_exploration.py --detector` 接受 `yoloe_seg`。
- `autonomous_search_worker.py` 继续原样透传 `--detector`，无需创建第二组参数。
- 将 `_gsam_detector` / `_ensure_gsam_detector()` / `_close_detector()` 泛化为本地结构化检测器生命周期，由 factory 构造 Grounded-SAM 或 YOLOE-Seg。
- 将 `detector == "grounded_sam"` 这类业务判断改为“是否支持本地结构化物体/mask”能力判断，确保 YOLOE-Seg 走与 Grounded-SAM 相同的目标候选、对象地图和几何关系链。
- 保持 VLM 复核现状不变。YOLOE-Seg 只替换“产生候选框/mask”的后端，不改变复核触发条件和决策状态机。
- 缓存键继续包含图像、prompt 和 slots，另加 backend/model 标识，避免切换后误用另一模型的结果。

### 6.3 自主搜索失败策略

- 启动前 YOLOE-Seg 预检失败：拒绝启动搜索会话，不下发 Nav2 目标。
- 运行中 worker 崩溃或超时：撤销当前自主移动，会话转为可观测的 `perception_degraded/failed`，不将空检测解释为“视野中没有目标”。
- 不做运行中静默回退。若后续要增加 fallback，必须作为独立开关并记录事件。

---

## 7. 遥控器被动建图接入计划

### 7.1 选择入口

`start_remote_control_mapping.sh` 新增：

```bash
--detector grounded_sam|yoloe_seg
```

配置优先级：

```text
--detector
  > REMOTE_MAPPING_DETECTOR
  > semantic_mapping.yaml: perception.detector
  > grounded_sam
```

示例：

```bash
# 原路线，行为不变
bash scripts/go2w/start_remote_control_mapping.sh --detector grounded_sam

# 新增 YOLOE-Seg 路线
bash scripts/go2w/start_remote_control_mapping.sh --detector yoloe_seg

# 单帧无运动冒烟检查
bash scripts/go2w/start_remote_control_mapping.sh --detector yoloe_seg --smoke
```

选择在启动前完成，WebUI 只读显示“当前检测路线”，不新增可远程重启感知侧车的控制 API。这与遥控建图模式的 display-only 安全边界一致。

### 7.2 启动器改造

`start_remote_control_mapping.sh` 按 backend 分支预检：

- `grounded_sam`：保留当前 `GROUNDED_SAM_PYTHON`、Python path、Torch 和 CUDA 检查；
- `yoloe_seg`：检查 `YOLOE_PYTHON`、权重、YOLOE/Ultralytics import、CUDA、版本和 warm-up；
- 共通：检查 RGB-D、SLAM 依赖、磁盘、时钟和 GPU，然后才启动任何本模式进程。

smoke 路径和正式 worker 启动命令都必须传入同一个 `--detector`，防止“预检的是 YOLOE-Seg，正式却跑 Grounded-SAM”。启动成功后把 backend 写入会话状态和日志首部。

### 7.3 侧车 worker 改造

- `remote_control_semantic_mapping_worker.py` 新增 `--detector` 并使用共用 factory，不再直接构造 `GroundedSAMSubprocessDetector`。
- `_prepare_detector_env()` 拆分为共通环境和 backend 专用环境，错误信息必须带实际 backend。
- `_detect()` 继续使用现有的场景词轮换和 hazard priority terms，只替换检测器实现。
- `_ram_scene_terms()` 依 capability 决定是否调用。YOLOE-Seg 返回空扩展词表时不记录成 RAM++ 错误。
- 当 YOLOE-Seg 单帧失败时，当前关键帧计入 `failed_keyframes`，SLAM 和实体遥控不受影响；连续失败达现有阈值后侧车进入 `DEGRADED`。
- 关闭时通过统一 `close()` 收回 worker，不在业务层识别具体子进程类型。

### 7.4 遥控建图特有风险

遥控建图会持续遇到不同场景词组，YOLOE-Seg 的文本词表更新不能每帧重建模型。需用 prompt hash 缓存文本特征，并在回放中确认词组轮换不造成显存持续增长。

当检测速度慢于关键帧产生速度时，保留现有有界队列和丢帧策略，不让感知拖慢 SLAM。本轮不改关键帧触发参数，只比较两个检测后端在相同输入队列下的处理能力。

---

## 8. 配置、可观测性与会话取证

### 8.1 配置分层

| 范围 | 配置内容 | 位置 |
| --- | --- | --- |
| 通用部署 | Python、模型路径、device、precision、timeout | `.env` / `app.config` |
| 自主搜索 | 默认 detector | `configs/go2w/autonomous_search_web.yaml` |
| 遥控建图 | detector、confidence、IoU、max detections | `configs/go2w/semantic_mapping.yaml` |
| 单次会话 | detector 显式选择 | Web/API 或启动 CLI |

队列、词表轮换、业务置信度门和模型原始置信度要分开命名，避免用一个 `confidence` 同时控制模型 NMS 前召回和语义地图接纳。

### 8.2 必须记录的生效值

每个会话的 status/events/session archive 至少包含：

```text
detector_backend
detector_model_id
detector_model_sha256
detector_library_version
detector_device
detector_precision
detector_confidence / detector_iou / detector_image_size
prompt_hash / prompt_term_count / detection_slots
inference_latency_ms / queue_wait_ms / total_latency_ms
detection_count / masks_valid / masks_invalid
fallback_used=false
```

WebUI 显示的 backend 来自 worker 回传的“实际生效值”，不是前端自己重复请求参数。这能识别“页面选了 YOLOE-Seg，子进程却启动了其它模型”的假绿状态。

---

## 9. 文件级改造清单

### 9.1 建议新增

```text
app/detectors/backend.py
app/detectors/factory.py
app/detectors/yoloe_seg_subprocess.py
app/detectors/yoloe_seg_worker.py
tests/test_yoloe_seg_adapter.py
tests/test_detector_factory.py
tests/test_yoloe_seg_contract.py
```

如 YOLOE-Seg 依赖与主环境冲突，再增加独立的 lock/requirements 文件和环境安装说明，不把未锁定的 `ultralytics` 直接塞入现有主 `requirements.txt`。

### 9.2 建议修改

```text
app/config.py
.env.example
app/detectors/base.py                         # 仅补通用能力/诊断，不破坏现有字段
scripts/go2w/run_autonomous_loop.py
scripts/go2w/run_semantic_exploration.py
scripts/go2w/autonomous_search_worker.py      # 核对透传，通常无需改核心逻辑
app/manual_web_demo/search_models.py
app/manual_web_demo/search_session_service.py
app/manual_web_demo/templates/index.html
app/manual_web_demo/static/search_ui.js
configs/go2w/autonomous_search_web.yaml
scripts/go2w/remote_control_semantic_mapping_worker.py
scripts/go2w/start_remote_control_mapping.sh
configs/go2w/semantic_mapping.yaml
```

### 9.3 禁止顺手改动

```text
Nav2 参数、BT、controller/planner
LiDAR 预处理和 collision monitor
运动授权、仲裁、急停、cmd_vel
frontier / 路口 / 长程目标算法
VLM 战略决策和调用节拍
plain_slam 和地图坐标系
```

---

## 10. 分阶段实施顺序

### 阶段 A：离线可行性小样本

1. 锁定一个 YOLOE-Seg checkpoint、推理库版本和许可证处理方案。
2. 用项目已有 RGB 帧验证中英文目标词、场景词、相近类别和 mask 输出。
3. 记录冷启动、warm 推理、显存、输出分辨率和无目标帧行为。

退出条件：模型能在目标主机 GPU 上稳定运行，并能产生可转换的实例 mask。

### 阶段 B：适配器与契约

1. 实现常驻 worker、subprocess client、factory 和 capability 定义。
2. 实现 bbox 归一化、mask row-run RLE、label/slot/source 转换。
3. 用伪 worker 和固定 mask 完成单元测试，不依赖 GPU。

退出条件：一份 YOLOE-Seg 输出能无分支地通过现有 `DepthObjectLocalizer`。

### 阶段 C：自主搜索接入

1. 打通 Web/API/worker/CLI/backend factory 全链路选择。
2. 泛化现有 `grounded_sam` 硬编码分支，保持目标角色、锚点角色、场景角色和异步 VLM 复核契约。
3. 使用历史帧跑 dry-run，禁止运动。

退出条件：三个 detector 选项都能被正确路由，且默认 Grounded-SAM 回归通过。

### 阶段 D：遥控建图接入

1. 增加 CLI/env/YAML 选择和分后端预检。
2. 将建图 worker 改为 factory 构造，保持关键帧队列、场景词轮换和危险检测逻辑不变。
3. 运行 `--smoke --detector yoloe_seg`，检查一帧识别、深度定位和状态文件。

退出条件：无运动 smoke 成功，且不启动任何软件运动链。

### 阶段 E：回放对比与真机验收

1. 在同一批录制帧上对比 Grounded-SAM 和 YOLOE-Seg。
2. 先做静止真机、再做实体遥控慢速建图、最后做自主搜索 dry-run 与受控短程运动。
3. 任何一阶段不达标都通过单个 detector 开关切回 Grounded-SAM，不回滚其它业务改动。

---

## 11. 测试与验收

### 11.1 单元和契约测试

- backend 合法值、配置优先级和非法值显式拒绝；
- YOLOE 类别索引到 `source_prompt_term` 的一一映射；
- target/anchor/scene 词槽归属、重叠词与空词槽；
- bbox 裁剪、零面积框过滤和像素/归一化转换；
- mask 编码后再解码必须与原 mask 逐像素一致；
- mask 尺寸错误、空 mask、越界 mask 和 bbox-only 降级；
- worker 超时、崩溃、非 JSON 输出、request_id 不匹配和有序关闭；
- prompt 缓存命中、换目标后失效和显存不增长；
- `DepthObjectLocalizer` 能消费 YOLOE mask，产生合法 `camera_xyz` 和三维尺寸。

### 11.2 双入口接线测试

| 场景 | 核心断言 |
| --- | --- |
| 自主搜索请求 `yoloe_seg` | 请求、worker argv、runner 和 factory 生效值完全一致 |
| 自主搜索默认配置 | 仍启动 `grounded_sam`，旧会话行为不变 |
| YOLOE 本地检测 | 走结构化物体分支，不误落入 VLM-only 分支 |
| 遥控建图 `--detector yoloe_seg` | 预检、smoke 和正式侧车全部使用 YOLOE-Seg |
| 遥控建图 `grounded_sam` | 现有场景词轮换、危险增强和拓扑回归通过 |
| 选中 YOLOE 但权重缺失 | 启动显式失败，不静默改走 Grounded-SAM |
| 遥控 worker 运行中检测失败 | SLAM 继续，感知状态降级，不会发出运动指令 |

### 11.3 回放对比指标

用同一批帧、同一词表和各自锁定阈值比较：

- 目标级 recall/precision，尤其是小物体、遮挡、远距离和颜色限定目标；
- 关键场景类别召回率和每帧误检数；
- mask IoU 或人工分级，以及有效深度像素比例；
- 成功产生 `map_xyz` 和三维尺寸的比例；
- 多帧持久 ID 稳定性、重复建点和标签抖动；
- cold start、warm p50/p95、峰值显存、关键帧队列深度和丢帧率；
- 自主搜索的误 `TARGET_FOUND`、强制 VLM 复核次数和单轮感知耗时；
- 遥控建图的每分钟处理关键帧数、延迟分布和语义地图污染率。

初次准入门槛：

1. YOLOE-Seg warm p95 延迟不高于同机 Grounded-SAM 的 60%；
2. 核心目标 recall 相对 Grounded-SAM 基线的下降不超过 3 个百分点；
3. 三维定位成功率相对 Grounded-SAM 基线的下降不超过 3 个百分点；
4. 回放集不允许出现新增的未经复核误 `TARGET_FOUND`；
5. 连续回放无 worker 泄漏、显存持续增长或 prompt 串线。

如第 2、3 项未达标但速度优势明显，YOLOE-Seg 仍可保留为“实验性可选路线”，但不升格为默认。

---

## 12. 真机验收顺序

1. **宿主离线回放**：不连机器狗，验证契约、性能和资源。
2. **连机只读静止 smoke**：机器狗静止，只取一帧 RGB-D，验证 mask 和深度定位。
3. **遥控建图短路线**：由操作员使用实体遥控器慢速驾驶，检查队列、落图、ID 和 WebUI；项目不发运动指令。
4. **自主搜索 dry-run**：运行完整感知和决策链，不授权运动。
5. **受控自主短程**：通过现有场地清空、操作员在场和运动授权门后，再做小范围对比。

每一阶段都单独归档 detector backend、模型 hash、配置快照、events、status、原始帧和标注帧，不将两个 backend 的结果混在同一会话中。

---

## 13. 回滚方案

本改造不修改数据库和地图文件格式，回滚只需切换检测后端：

```text
自主搜索：detector=grounded_sam
遥控建图：--detector grounded_sam
```

回滚后仍保留 YOLOE-Seg 已产生的会话记录，供对比分析；不将 YOLOE 模型库从共用环境中“卸载回滚”，因为它本来就应与 Grounded-SAM 环境隔离。

---

## 14. 完成定义

只有同时满足以下条件，才算“YOLOE-Seg 已作为额外可选路线接入”：

- 自主搜索和遥控被动建图都有明确选择入口；
- 两条业务链共用一个 YOLOE-Seg 适配器，且输出完整 `DetectedObject`；
- mask 能被现有深度定位消费，并在地图上生成真实三维物体位置/尺寸；
- 自主搜索的 target/anchor/scene 角色、VLM 复核和会话状态没有因新 backend 而改变语义；
- 遥控建图的场景词轮换、危险增强、关键帧队列和只读运动边界保持不变；
- 缺少权重、CUDA 不可用、worker 崩溃等错误不会静默回退；
- Grounded-SAM 仍是默认和一键回退路线，现有回归测试通过；
- 本轮没有顺手修改 Nav2、Agent、VLM 节拍或“走一步识别一次”的当前调度架构。

---

## 15. 官方技术参考

- [Ultralytics YOLOE 开放词汇检测与实例分割](https://docs.ultralytics.com/models/yoloe)
- [THU-MIG YOLOE 官方实现](https://github.com/THU-MIG/yoloe)

