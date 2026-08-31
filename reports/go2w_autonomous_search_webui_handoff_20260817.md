# Go2-W 自主语义搜索 WebUI 实施交接（handoff）

> 日期：2026-08-17
> 计划书：`robot_scene_demo_真机自主语义搜索_WebUI_一次性实施计划书_20260817.md`
> 仓库：`https://github.com/BROVVV/robot_scene_demo`
> 平台：Unitree Go2-W（实验后端），WebUI/高层搜索平台无关

## 1. 本轮完成内容

### 1.1 核心组件（全部新增）

| 文件 | 职责 |
| --- | --- |
| `app/live_robot/search_event.py` | SearchEvent 统一事件协议（schema `search_event_v1`，26 种事件类型，event_id 单调） |
| `app/live_robot/search_event_bus.py` | 线程安全事件总线（publish/subscribe/recent/clear） |
| `app/live_robot/search_state_store.py` | 最新快照存储（锁 + 深拷贝，地图 revision 单调，timeline<=500） |
| `app/live_robot/explorer_search_adapter.py` | AutonomousExplorer 扁平事件 → SearchEvent 桥（含 MAP_UPDATED 节点展平与首帧节点合成） |
| `app/manual_web_demo/control_ownership.py` | ControlOwner（NONE/MANUAL/AUTONOMOUS/ESTOP 互斥 + estop latch） |
| `app/manual_web_demo/search_models.py` | Start 请求/会话模型 + yaml/env 默认值加载 |
| `app/manual_web_demo/search_executor.py` | SearchExecutor 协议 + SubprocessSearchExecutor（JSONL IPC）+ InProcessMockExecutor（测试/CI） |
| `app/manual_web_demo/search_session_service.py` | 会话生命周期（STARTING/RUNNING/PAUSED/…），产物落盘，history |
| `app/manual_web_demo/search_routes.py` | `/api/search/*` + `/ws/search`（snapshot→事件→心跳） |
| `scripts/go2w/autonomous_search_worker.py` | 搜索 worker 子进程（ROS2 Python，复用 run_semantic_exploration 装配，event_hook 实时转发） |

### 1.2 增强（既有文件小改）

- `app/live_robot/autonomous_explorer.py`：新增 `request_pause/request_resume/paused`、
  PAUSED 状态、`action_start` 事件、全量 candidates 评分事件（§33/§34）。
- `scripts/go2w/run_semantic_exploration.py`：`--dry-run-motion` 标志（真实感知/推理、
  零运动命令，§60）；`event_hook` 注入点（worker 实时转发 + graph 快照增强）；
  stdout 改单行 JSON（worker IPC 兼容）。
- `app/manual_web_demo/web_server.py`：挂 search router/WS、estop 联动、手动/自主
  控制权互斥、`/api/status` 增加 owner/search、`/api/search/readiness`。
- 前端：`templates/index.html` 三 Tab（自主搜索/手动控制/系统状态，保留原 DOM id
  兼容 app.js）、`static/search_ui.js`（WS 重连、状态渲染、canvas overlay、调试视图）、
  `static/search_map.js`（SVG 拓扑地图）、`static/style.css` 响应式扩展。

### 1.3 配置与脚本

- `configs/go2w/autonomous_search_web.yaml`（§117 默认值）
- `scripts/go2w/start_autonomous_search_web.sh`（`--enable-autonomous-motion` / `--mock`，
  自动停旧的项目自有 web 进程）
- `scripts/go2w/stop_autonomous_search_web.sh`（只停 web.pid）

### 1.4 测试（47 个新增，全部 PASS）

`test_search_event.py`、`test_search_state_store.py`、`test_control_ownership.py`、
`test_search_session_service.py`（6 种 mock 场景：首帧命中/锚点引导/搜索穷尽/
导航失败→REPLAN/暂停恢复/操作员停止）、`test_autonomous_search_web_routes.py`
（REST + 409 冲突 + estop 联动）、`test_autonomous_search_websocket.py`
（snapshot/事件/重连）、`test_live_exploration_graph.py`（地图 schema/revision/产物）。

## 2. 真机验收结果（2026-08-17，机器狗重启后）

重启后机器狗功能验证：相机新鲜（<0.1s）、odom 5Hz、robot_mode=1/error=0、
/usr/bin/python3 的 ROS worker ready、SiliconFlow LLM 正常（已识别 蓝色塑料篮 等）。

| 验收项 | 结果 |
| --- | --- |
| A 相机/状态 Web 启动 | PASS：`/api/status` camera fresh、readiness ready |
| B 搜索 dry-run（真实相机+LLM+SemanticNavigation+Planner，零运动） | **PASS：TARGET_FOUND（50s，1 cycle）**，识别到 blue_plastic_basket（bbox [0.24,0.61,0.33,0.75]，属性 on floor/near chair/blue） |
| C turn-only 真机搜索（授权自主运动，≤30° 转向） | **PASS：TARGET_FOUND**，1 个真实自主转向（SemanticNavigation 选向 score 0.61），转向后第 2 周期命中"绿色垃圾桶"；graph 1 节点 1 边 |
| D 连续搜索（转向）10+ 周期 | **PASS（12-cycle 会话）**：见下 |
| Pause/Resume 真机验证 | **PASS**：真实搜索中（cycle 5）暂停 → 当前转向完成后停车（PAUSED 保持 15s+ 零运动）→ 恢复继续 cycle 7 |
| E 目标不存在 | 逻辑已具备（SEARCH_EXHAUSTED / MAX_STEPS_REACHED），D 会话以 MAX_STEPS_REACHED 正常收尾 |
| F/G 普通/关系目标 | B（蓝色塑料篮）、C（绿色垃圾桶）均 TARGET_FOUND；关系目标有 CLI 历史证据（Trial 5） |

### 12-cycle 连续自主循环会话（`search_20260817_193637`）

- **12 planning cycles / 11 次真实转向 / 12 observations / 12 unique nodes**（超过计划书 D 的 10+ 周期要求）
- 1 次导航失败自动 replan 恢复；4 次 SemanticNavigation 语义选向 + 8 次 fallback 选向
- 地图实时增长：12 节点 / 12 边；`MAX_PLANNING_CYCLES_REACHED` 正常收尾（灭火器目标，房间内未见）
- 中途 pause/resume 一次（cycle 5 暂停 → 转向完成即停车 → 恢复 cycle 7 继续）

### 前端验证

- 三 Tab 结构 + 全部 73 个 JS getElementById 目标与 DOM id 一一对应（含修复 app.js
  依赖的 `light-motion` 元素）
- 服务端实测：/ws/search snapshot 实时显示运行中真机会话；409 冲突保护在真实会话
  运行期间正确拒绝第二个 start

## 3. 一键启动

```bash
cd /home/brov/robot/robot_scene_demo
bash scripts/go2w/start_autonomous_search_web.sh                    # 只读
bash scripts/go2w/start_autonomous_search_web.sh --enable-autonomous-motion
bash scripts/go2w/start_autonomous_search_web.sh --mock
bash scripts/go2w/stop_autonomous_search_web.sh
# 浏览器 http://127.0.0.1:8765
```

## 4. 已知限制 / 待办

1. 既有测试失败 4 个（与本次改动无关，改前即失败）：`test_task_planner` 的
   count 状态用例、`test_streamlit_natural_language_task_ui`、
   `test_go2w_live_ui_status`、`test_task_examples_evaluator`（前两者还会挂起）。
2. BOOTSTRAP 阶段 ~45s（rclpy 初始化 + 传感器等待 + readiness 探测），首轮 LLM
   观察 ~30s；可接受，但启动提示可优化。
3. `finish_on_visual_confirmation` 参数在 Explorer 内仍是"只存储"（与既有行为一致）。
4. `--profile-config` 仍是死参数（既有）。
5. `/ws/search` 事件重复：SEARCH_FINISHED 由 adapter 与服务各发一次（幂等，无副作用）。
6. 地图为 topological/relative；metric 模式留给未来 ProductionRobotBackend。
7. 暂停在运动执行中（同步 backend）需等当前 ≤30° 动作完成后生效（设计如此）。

## 5. 关键文档

- `docs/GO2W_AUTONOMOUS_SEARCH_WEBUI.md`（架构/API/WS/事件/地图/会话/互斥/mock/真机/排障）
- README.md 新增 "Go2-W Autonomous Semantic Search WebUI" 章节
- 计划书：`robot_scene_demo_真机自主语义搜索_WebUI_一次性实施计划书_20260817.md`
