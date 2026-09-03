# 知识增强场景推理验证说明

本文件记录第十阶段的工程验证口径。当前验证目标不是论文级 benchmark，而是确认新增模块能按任务类型输出可校验的结构化结果。

## 已覆盖测试

- `tests/test_task_parser.py`：验证自然语言任务到 `RobotTask` 的规则解析。
- `tests/test_scene_kb.py`：验证本地 JSON/JSONL 知识库加载、检索、追加观察和置信度更新。
- `tests/test_psg_builder.py`：验证规则版预测性场景图、假想节点和 GraphML 输出。
- `tests/test_hypothesis_generator.py`：验证目标不可见时的候选位置假设。
- `tests/test_scene_reasoner.py`：验证可解释推理摘要和推荐验证动作。
- `tests/test_task_planner.py`：验证不同任务类型的 `TaskPlan`。
- `tests/test_kb_updater.py`：验证稳定知识和临时状态的更新边界。
- `tests/test_knowledge_aware_analyzer.py`：验证知识增强主流程和输出文件。
- `tests/test_llm_prompt_contract.py`：验证 LLM 可选辅助字段不破坏旧 `SceneAnalysisResult`。

## 场景样例

`examples/tasks/` 下保存最小场景任务样例，用于后续批量评估：

- `find_phone_not_visible.json`
- `count_chairs_room.json`
- `inspect_open_doors_floor.json`
- `find_room_by_doorplate.json`
- `navigate_corridor_end_room.json`

## 评估指标

- 任务解析是否匹配预期任务类型。
- 目标不可见时是否生成候选位置和验证动作。
- 知识检索是否返回相关先验。
- 推理摘要是否引用当前视觉证据和知识库证据。
- 任务规划是否按任务类型选择不同策略。
- 知识更新是否区分稳定知识和临时状态。
- 输出 JSON 是否能通过 Pydantic 校验。
