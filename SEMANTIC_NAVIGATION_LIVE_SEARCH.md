# Live semantic search

本项目的搜索入口是自然语言任务，不再把用户输入直接当作机器人目标标签。

```text
WebUI task_text
  -> task understanding / capability gate
  -> SearchTaskContext
  -> live observer + Place/Object/Frontier graph
  -> live graph Dijkstra + bounded motion command
  -> stop and re-observe
```

## WebUI

`POST /api/search/start` 使用以下字段：

```json
{
  "task_text": "找到饮水机旁边的蓝色垃圾桶",
  "backend": "go2w_experimental",
  "reasoner": "semantic",
  "enable_autonomous_motion": false
}
```

页面刷新或 WebSocket 重连后，`GET /api/search/state` 会恢复任务理解结果、统一语义地图、当前具体运动指令和完整决策历史。决策历史也可通过 `GET /api/search/decisions` 获取。

## 离线 dry-run

不连接 ROS、不运动机器人即可验证完整观察—建图—决策—事件链：

```bash
conda activate go2_robot_scene_demo
python scripts/go2w/run_semantic_exploration.py \
  --target "饮水机旁边的蓝色垃圾桶" \
  --backend mock \
  --reasoner semantic \
  --max-planning-cycles 5
```

WebUI 的 mock 后端同样使用确定性离线任务上下文，不会为了任务解析访问真实 API。每次运行目录会写入 `task.json`、`semantic_map.json`、`decisions.jsonl`、`events.jsonl`、`summary.json` 和 `final_state.json`。

## 模型与感知配置

- `SILICONFLOW_REASONING_MODEL`：任务理解、关系推理和搜索决策，默认 `deepseek-ai/DeepSeek-V4-Flash`。
- `SILICONFLOW_VISION_MODEL`：图像理解和目标复核，保留当前视觉模型配置。
- `DETECTION_BACKEND=llm` 与 `LIVE_VISION_BACKEND=siliconflow_vlm`：真机主链路使用实时 VLM；本地分割器只保留为显式诊断/兼容路径。

`.env.example` 不包含任何 API key。真机运动仍需操作者监督配置；每个运动指令都会给出明确转向角、前进距离、停止并重新观察要求及安全原因。
