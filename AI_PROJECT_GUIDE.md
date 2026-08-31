# AI 项目阅读指南

本文档是仓库的 AI 入口。它描述 2026-08-19 工作区快照的代码边界、运行链路、记录位置和已知现象。部署细节以 [`README.md`](README.md) 为准，阶段性实验结论和历史设计以 [`reports/`](reports/) 为准。

## 1. 项目目标

本项目面向 Unitree Go2-W 真机的自然语言目标搜索：操作员输入“找到厕所”“寻找黑色沙发”等任务，系统从 RGB-D、LiDAR、IMU 和轮式里程计获取观测，理解目标，持续构建探索/地点/语义记忆，选择下一观察动作，并通过 WebUI 展示状态、证据、地图和控制历史。

仓库同时包含离线/Mock、视频、ROS2 dry-run 和真机实验路径。任何真机运动都必须保留急停、控制权租约、碰撞保护和操作员监督。

## 2. 推荐阅读顺序

1. `README.md`：从零部署、环境变量、WebUI 与 ROS2 启动方式。
2. `AI_PROJECT_GUIDE.md`：架构和当前问题总览。
3. `RUN_RECORDS_INDEX.md`：输入、输出和真机记录的索引与格式。
4. `app/manual_web_demo/`：Web 服务、搜索会话、控制权和前端界面。
5. `scripts/go2w/run_semantic_exploration.py` 与 `scripts/go2w/autonomous_search_worker.py`：自主搜索主循环和 Web 后台进程入口。
6. `app/task_understanding/`、`app/reasoning/`、`app/navigation/`、`app/perception/`：任务理解、推理、导航和感知实现。
7. `ros2_ws/src/` 与 `unitree_go2w_control/`：传感器桥接、时间同步、里程计、融合和底层运动控制。
8. `configs/go2w/`、`.env.example`、`.env.go2w`：可公开配置模板；真实密钥只允许放在被 Git 忽略的 `.env`。
9. `tests/`：行为约束和回归测试。
10. `outputs/`、`logs/`、`reports/`：运行证据、底层遥测和交接报告。

## 3. 运行链路

```text
自然语言任务
  -> Web API / 搜索会话服务
  -> 自主搜索 worker
  -> 任务目标解析与搜索画像
  -> RGB-D/视觉观测 + 空间位姿
  -> 场景推理、探索图、地点图、语义记忆
  -> 候选观察动作与安全门控
  -> Go2-W/ROS2 运动后端
  -> 新观测并循环更新

所有阶段
  -> events.jsonl / summary.json / 图数据 / WebUI 时间线
```

关键模块：

- `app/manual_web_demo/web_server.py`：Web 路由和状态接口。
- `app/manual_web_demo/search_session_service.py`：搜索任务生命周期和子进程管理。
- `scripts/go2w/run_semantic_exploration.py`：高层搜索循环、观测、决策与记录。
- `app/navigation/candidate_goal_generator.py`：候选观察目标生成。
- `app/navigation/local_goal_executor.py`：局部目标执行和结果封装。
- `app/reasoning/llm_situated_search_reasoner.py`：结合观测和记忆的搜索推理。
- `app/task_understanding/llm_task_interpreter.py`：自然语言任务解释。
- `app/perception/realsense_http_rgbd_source.py`：D435 HTTP RGB-D 输入。
- `app/mapping/`、`app/memory/`：地图、图结构和记忆数据。
- `unitree_go2w_control/`：Go2-W 控制封装、租约、监控和实验工具。

## 4. 当前快照中需要继续解决的现象

这些是 2026-08-19 实机观察和代码审阅所得，不代表已经修复：

1. Web 自主搜索链路没有完整复用自然语言任务理解流程；同时急停锁存时，新任务可能在前端看似提交但无法启动。已有记录证明“找到厕所”曾正确进入后台，因此需要分别修复任务解析接入和任务启动/急停状态反馈。
2. 目标缺失时的快速观测路径可能只记录“未找到目标”，没有把画面中的所有对象和关系写入完整语义图。
3. 当前探索图主要按观测帧/节点包组织，不是“每个识别物体一个节点”；运行中还可能产生自环边。
4. WebUI 直接使用很小的米制位移配合较大的固定节点半径和显示范围，导致节点视觉重叠；部分空间更新事件只有消费端而没有稳定生产端。
5. 下一步决策展示偏语义标签，缺少明确的距离、步数、转角、选择原因和可回放的完整历史载荷。
6. 推理模型和视觉模型的职责需要拆分。文本推理模型不能直接替代支持图像输入的视觉模型。

分析上述问题时，应优先对照 `outputs/live_runs/` 中 2026-08-19 的事件流和图数据，避免只根据前端截图推断。

## 5. 配置与依赖边界

- `.env` 含本机真实凭据，明确不进入 Git。
- `.env.example` 和 `.env.go2w` 只保存可公开模板或非密钥参数。
- `external/` 保存本机第三方仓库、模型和检查点，不作为本项目源代码提交；依赖应按 README 重新安装。
- ROS2 的 `build/`、`install/`、`log/`，Python 虚拟环境、缓存和模型权重均不提交。
- `runtime/` 中 PID、锁和现场中间态不属于可复现记录。
- 仓库内的 IP、端口和硬件标识是当前实验网络配置，不应被理解为通用默认值。

## 6. 代码快照说明

本次发布保留工作区内已有的项目改动及其输入/输出记录，不对其功能含义做额外重构。运行记录中包含成功、搜索耗尽、操作员停止和 dry-run 等不同结局，AI 在判断功能是否完成时必须读取 `status`、`mode`、`robot_action`、安全事件和证据字段，不能把文件存在等同于功能已经通过真机验收。

## VLM-only 低延迟语义导航（2026-08-28）

生产真机入口保持 `scripts/go2w/run_semantic_exploration.py`。当前默认视觉路线为 VLM-only：
- Quick VLM 只负责目标候选，`target_decision.is_present` 是目标 gate。
- Full Semantic 由后台 `AsyncSemanticObservationManager` 异步更新，不阻塞普通运动循环。
- VLM daemon（Unix socket）优先，不可用时自动回退 subprocess。
- GroundingDINO/SAM2 仅保留为 baseline，不进入生产 runtime。
