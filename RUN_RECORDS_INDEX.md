# 输入输出与运行记录索引

本索引对应 2026-08-19 的公开快照。记录用于复现问题、理解状态机和验证代码输出，不包含 API 密钥、私有 `.env`、模型权重、第三方仓库副本或进程 PID。

## 1. 记录规模

- `outputs/`：174 个非 PID 文件，约 9.8 MB。
- `logs/20260819_115208/`：7 组底层运动目标，共 63 个 JSON/JSONL/CSV/文本文件，约 60 KB。
- `outputs/live_runs/`：12 条自主搜索事件流和 10 组探索输出。
- `outputs/live_runs_test/`：25 组 Web/搜索状态机测试运行，其中包含一次 `TARGET_FOUND`、一次 `SEARCH_EXHAUSTED` 和多次操作员停止场景。

## 2. 输入记录

搜索任务输入可以从以下位置恢复：

- `outputs/live_runs/explore_go2w_*/summary.json` 的 `target` 字段。
- `outputs/live_runs/search_*/events.jsonl` 中的任务启动、候选、目标和状态事件。
- `outputs/live_runs_test/search_*/summary.json` 与配套 `events.jsonl`。
- `logs/20260819_115208/goal_*/goal.json` 和 `requests.jsonl`：底层运动目标、模式、速度、持续时间和相对转角。

本快照中的代表性自然语言任务包括：

- `找到厕所`
- `黑色沙发`
- `不存在的红色独角兽`
- `饮水机旁边的蓝色垃圾桶`

## 3. 输出记录

### 自主搜索事件流

`outputs/live_runs/search_YYYYMMDD_HHMMSS/events.jsonl`

每行是一个按时间追加的 JSON 事件。该文件最适合还原 WebUI 时间线、任务状态、候选生成、目标选择、动作执行、地图更新、急停和搜索结束顺序。分析时应保留原始顺序，并检查事件内的 `cycle`、`payload`、`target`、`status` 和时间字段。

### 探索结果

`outputs/live_runs/explore_go2w_YYYYMMDD_HHMMSS/`

- `summary.json`：目标、最终状态、循环数和运行模式摘要。
- `exploration_graph.json`：探索节点、边、访问/候选状态和位姿。
- `place_graph.json`：地点级节点与关系。
- `semantic_map.json`：识别对象及语义关系。文件只有空数组时，表示该次运行没有写入完整场景对象，不表示环境里没有物体。
- `spatial_memory.json`：跨循环空间记忆。

其中与现场截图问题最相关的运行：

| 运行目录 | 输入目标 | 最终状态 | 循环数 |
| --- | --- | --- | ---: |
| `explore_go2w_20260819_164632` | 黑色沙发 | `OPERATOR_STOP` | 7 |
| `explore_go2w_20260819_171252` | 找到厕所 | `SEARCH_EXHAUSTED` | 22 |
| `explore_go2w_20260819_171809` | 找到厕所 | `OPERATOR_STOP` | 10 |
| `explore_go2w_20260819_172023` | 找到厕所 | `OPERATOR_STOP` | 1 |

`找到厕所` 出现在后台摘要中，说明该文本至少在这些运行里成功传入搜索进程。若 WebUI 仍显示旧目标，应同时核对急停锁存、会话启动返回值、当前 session 指针和前端缓存。

### 搜索状态机测试记录

`outputs/live_runs_test/search_YYYYMMDD_HHMMSS/`

每组包含 `events.jsonl`、`exploration_graph.json` 和 `summary.json`。`search_20260819_133530` 的摘要为 `TARGET_FOUND`；`search_20260819_132321` 为 `SEARCH_EXHAUSTED`。其余多用于验证停止、接管或短周期状态转换。

### 底层运动请求与反馈

`logs/20260819_115208/goal_<id>/`

- `goal.json`：一次运动目标的输入参数。
- `requests.jsonl` / `responses.jsonl`：厂商接口请求与响应元数据。
- `feedback.jsonl`：执行过程反馈。
- `result.json`：最终误差、轮速证据、回滚和停止结果。
- `safety_events.jsonl`：安全事件。
- `low_state.csv` / `sport_state.csv`：采样遥测。
- `process_audit.txt`：当时相关进程审计。

### 其他输出

- `outputs/autonomous_search/`：Web 自主搜索服务状态与服务日志。
- `outputs/manual_web_demo/`：手动控制 Web 服务、ROS worker 日志和现场快照。
- `outputs/scene_result.json`、`outputs/reasoning_report.json` 等：离线或单次推理输出。
- `outputs/crops/`：目标候选裁剪图，用于理解视觉验证输入。
- `reports/`：阶段性部署、实验、验收和后续工作说明。

## 4. 分析注意事项

1. `events.jsonl` 可能很大，应流式逐行读取，不要一次拼进提示词。
2. 先读对应 `summary.json`，再按关键事件类型筛选事件流，最后查看图文件和服务日志。
3. `OPERATOR_STOP` 是运行结果，不应解释成目标搜索失败；应继续查看停止发生前的决策和动作。
4. 图节点坐标可能非常接近。应比较原始数值和边关系，不要仅依据 WebUI 的重叠显示判断数据完全相同。
5. 服务 `.log` 是诊断证据，JSON/JSONL/CSV 才是更稳定的机器可读接口。
6. 记录来自真实硬件和测试模式的混合运行，必须根据各文件的 `mode`、`dry_run`、后端类型和安全门控字段区分。
