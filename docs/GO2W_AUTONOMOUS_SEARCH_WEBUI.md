# Go2-W Autonomous Semantic Search WebUI

> 版本：2026-08-17
> 计划书：`robot_scene_demo_真机自主语义搜索_WebUI_一次性实施计划书_20260817.md`
> 平台：Unitree Go2-W（实验后端）+ 未来任意 `RobotBackend`

把既有 Manual WASD Demo 升级为 **Manual + Autonomous Search Web Console**：
浏览器输入自然语言目标 → 真实机器狗自主语义探索 → 页面实时显示相机、当前物体、
目标证据、下一步决策、候选评分、探索拓扑地图与事件时间线。

## 1. 架构

```
Browser（Target / Camera / Objects / Map / Logs；只发任务与展示）
   │  HTTP + WebSocket
   ▼
FastAPI Web Process（Conda Python）
   app/manual_web_demo/web_server.py
   ├─ SearchSessionService（会话生命周期 + ControlOwner 互斥）
   ├─ SearchEventBus / SearchStateStore / ExplorerSearchAdapter
   ├─ /api/search/* 与 /ws/search（search_routes.py）
   └─ 原 Manual WASD 链路（controller / analyzer / MJPEG / /ws/control）
   │  JSONL stdin/stdout（{"cmd":...} / {"type":...}）
   ▼
Autonomous Search Worker（子进程，ROS2 系统 Python）
   scripts/go2w/autonomous_search_worker.py
   └─ 复用 run_semantic_exploration.py 装配：
      AutonomousLoop(rclpy) → LiveSemanticObserver → SemanticSearchController(SemanticNavigation)
      → AutonomousExplorer(OBSERVE→MATCH→VERIFY→UPDATE_MEMORY→PLAN→EXECUTE→REPLAN)
      → Nav2SearchBackend（ComputePathToPose → 走廊宽度判定 → NavigateToPose）
```

真机后端只有一个：`nav2`（计划书 §6.6）。相对位移原语（≤30° 转向 /
≤0.30m 前进）与旧的 `go2w_experimental` 后端已从生产路径移除，且不静默回退。

进程隔离（计划书 §12）：FastAPI 在 Conda 环境；搜索 worker 在 `/usr/bin/python3`
+ ROS2 Humble；ROS worker 照旧。搜索决策绝不发生在浏览器（§6、§91）。

测试/CI 路径：`InProcessMockExecutor`（同进程线程 + mock scene + mock backend），
无 ROS 依赖，覆盖计划书 §102 全部场景。

## 2. 进程模型

| 进程 | 环境 | 职责 |
| --- | --- | --- |
| uvicorn `web_server:app` | Conda `go2_robot_scene_demo` | HTTP/WS、状态存储、会话管理 |
| `manual_web_demo_ros_worker.py` | /usr/bin/python3 + ROS | 相机 latest.jpg、手动运动、安全状态 |
| `autonomous_search_worker.py` | /usr/bin/python3 + ROS | 自主搜索循环（按需拉起） |

搜索 worker 由 Web 进程惰性拉起（`SubprocessSearchExecutor`），stdout 只走协议
JSONL；异常/退出由 Web 侧记录并停止会话。Web 退出时：停止搜索 → 停 worker →
flush 日志（lifespan 关闭路径）。

## 3. 快速开始

```bash
# 真机唯一入口：Nav2 单链路。默认只规划、不执行。
bash scripts/go2w/start_nav2_autonomous_search.sh

# 现场验收通过后才加这一个开关（计划书 §9：命令行上只有这一个开关）。
# 它还必须配上 §10 的文件授权：configs/go2w/navigation_gate.yaml 的
# nav2_allow_execute 得先被**用户本人**改成 true，否则脚本直接拒绝启动。
bash scripts/go2w/start_nav2_autonomous_search.sh --enable-motion

# 离线前端开发（mock，无需 ROS）
bash scripts/go2w/start_autonomous_search_web.sh --mock

bash scripts/go2w/stop_autonomous_search_web.sh   # 只停项目自有 web.pid
```

`start_autonomous_search_web.sh` 现在是**旧入口**：它不启动 Nav2 bringup、
坐标桥、速度平滑和 Collision Monitor，单独用它做 Nav2 自主运动速度链是断的。
新入口内部会调用它，WebUI 功能不会少。`--enable-motion` 只是让
`nav2_launch(execution_enabled:=true)`，速度仍要再过 `go2w_cmd_vel_bridge`
的九道闸门（运动授权 / 急停 / 租约 / 雷达 / 定位新鲜 / 控制来源 / …）。

浏览器打开 `http://127.0.0.1:8765`（端口 `AUTONOMOUS_SEARCH_PORT` 可改）。
配置默认值来自 `configs/go2w/autonomous_search_web.yaml`，环境变量
`AUTONOMOUS_SEARCH_DEFAULT_BACKEND` / `AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION`
可覆盖。

## 4. API

### POST /api/search/start

普通 WebUI 只提交自然语言，运动权限、backend、RGB-D 和预算均继承服务启动配置：

```json
{"task_text": "饮水机旁边的蓝色垃圾桶"}
```

调试客户端仍可显式覆盖高级字段：

```json
{
  "target": "饮水机旁边的蓝色垃圾桶",
  "reasoner": "semantic_navigation",
  "backend": "nav2",                   // nav2 | mock | mock_metric
  "finish_on_visual_confirmation": true,
  "turn_only": false,
  "enable_autonomous_motion": false,
  "dry_run_motion": false,
  "max_seconds": 600,
  "max_planning_cycles": 100,
  "max_motion_steps": 50
}
```

立即返回 `{"ok": true, "session_id": "search_...", "status": "STARTING"}`；
已有运行中会话返回 409；手动控制占用时返回 409（`manual_control_active`）。

### 其余

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| POST | `/api/search/pause` / `resume` / `stop` / `estop` | 会话控制（estop 与手动急停联动） |
| GET | `/api/search/state` | 最新快照（刷新/F5/重连恢复用） |
| GET | `/api/search/map` | 探索图快照（`live_exploration_graph_v1`） |
| GET | `/api/search/objects` | `{current, session_seen, target_evidence}` |
| GET | `/api/search/events?limit=N` | 最近 SearchEvent |
| GET | `/api/search/history?limit=N` | 最近 10 次 WebUI 会话列表 |
| GET | `/api/search/history/{session_id}` | 某次会话的完整状态、事件和产物索引 |
| GET | `/api/search/readiness` | 自动就绪检查（人工标定不阻塞） |
| GET | `/api/search/executor` | 搜索 worker 状态 |
| WS | `/ws/search` | 连接→snapshot→增量事件（15s 心跳） |

## 5. WebSocket 协议

```
→ {"type":"snapshot","state":{...}}           连接即发完整快照
→ {"type":"events","events":[...]}           最近事件（<=200）
→ {"type":"event","event":{SearchEvent}}     增量事件（含 event_id 单调递增）
→ {"type":"heartbeat"}                       每 15s
← 客户端无需发任何消息；断开按 1s/2s/5s 退避重连，重连后重新取 snapshot
```

前端 `appState` + `applySearchEvent(event)` 统一收口（`static/search_ui.js`），
`static/search_map.js` 渲染 SVG 拓扑地图（节点状态：当前▲/已访问●/未访问○/
语义兴趣★/负证据◌/不可达✕/候选◎/确认✓，仅展示，不参与导航决策）。

## 6. SearchEvent

`app/live_robot/search_event.py`：`event_id`（单调）/ `session_id` / `timestamp` /
`event_type` / `cycle` / `payload`（JSON-safe，图像走 MJPEG 不进事件）。

类型全集（§16）：`SESSION_CREATED / SESSION_STARTED / SEARCH_STATE_CHANGED /
OBSERVATION_UPDATED / OBJECTS_UPDATED / TARGET_MATCH_UPDATED / VERIFICATION_STARTED /
VERIFICATION_FINISHED / TARGET_CONFIRMED / MEMORY_UPDATED / MAP_UPDATED /
CANDIDATES_GENERATED / GOAL_SELECTED / ACTION_STARTED / ACTION_FINISHED / REPLAN /
PAUSED / RESUMED / OPERATOR_STOP / ERROR / SEARCH_EXHAUSTED / SEARCH_FINISHED`。

映射：`AutonomousExplorer._emit` 的扁平字典 → `ExplorerSearchAdapter`
（`app/live_robot/explorer_search_adapter.py`）→ SearchEventBus → {WebSocket 推送,
SearchStateStore, JSONL 日志}。explorer 事件中的 `graph` 字段由 worker 注入
（observation / memory_update / navigation_result 携带全图快照）。

## 7. 地图 schema（`live_exploration_graph_v1`）

```json
{
  "schema_version": "live_exploration_graph_v1",
  "revision": 37,
  "map_mode": "topological",
  "current_node_id": "node_009",
  "robot": {"x": 1.1, "y": 0.3, "yaw": 0.52, "pose_quality": "relative"},
  "nodes": [{"node_id":"node_009","x":1.1,"y":0.3,"yaw":0.52,"pose":{...},
             "pose_quality":"relative","reachable_state":"SEMANTIC_INTEREST",
             "visited_count":1,"objects":["饮水机","门"],
             "target_match_level":"partial","semantic_relevance":0.91,
             "information_gain":0.72,"timestamp":1786969265.1}],
  "edges": [{"source_node_id":"node_008","target_node_id":"node_009",
             "action_type":"ROTATE_VIEW","navigation_result":"succeeded"}]
}
```

`revision` 单调递增（SearchStateStore 维护）；当前 Go2-W 为 topological/relative，
未来 metric backend 输出 `map_mode: metric`，WebUI 不重写（§50）。

## 8. 会话生命周期

`IDLE → STARTING → RUNNING ⇄ PAUSED → (TARGET_FOUND | SEARCH_EXHAUSTED |
OPERATOR_STOP | FAILED | FINISHED)`

- Pause（§40）：停止生成新 goal、停当前运动、保留 memory/graph；Resume 重新
  OBSERVE→REPLAN，不恢复旧指令（explorer 新增 `request_pause/request_resume`）。
- Stop（§41）：取消当前 goal、backend.stop、`OPERATOR_STOP`、flush 日志。
- 急停（§42）：`/api/estop` 与 `/api/search/estop` 都会 owner→ESTOP、停手动、
  停搜索，之后不再产生新动作（latched）。
- 产物（§56）：worker 原有产物之外，WebUI 原子维护
  `webui_state.json / webui_events.jsonl / webui_session.json`。刷新、停止和失败都不清空；
  完整 WebUI 会话滚动保留最近 10 次。旧 CLI/实验目录不带 WebUI 标记，不会被滚动清理。

## 9. Manual / Autonomous 互斥（ControlOwner）

`NONE / MANUAL / AUTONOMOUS / ESTOP`（`app/manual_web_demo/control_ownership.py`）。

- 手动启用时启动搜索 → 409 `manual_control_active`；
- 搜索运行中启用手动 → 409 `autonomous_search_running`（UI 提示先停止/暂停搜索）；
- 急停覆盖一切且 latch。

## 10. Mock 与测试

`make_mock_executor_factory()`（`search_session_service.py`）提供进程内 mock：
场景 `target_appears_after_n / no_target / anchor_then_target / scene_steps`、
`outcome_sequence`（导航失败→REPLAN）、`backend_latency_sec`（慢速暂停测试）。

测试文件（`tests/`）：

```text
test_search_event.py           事件/总线
test_search_state_store.py     快照/地图 revision
test_control_ownership.py      所有权互斥
test_search_session_service.py 生命周期 + 6 种 mock 场景（首帧命中/锚点/穷尽/
                               失败重规划/暂停恢复/操作员停止）
test_autonomous_search_web_routes.py   REST（含 409 冲突、estop 联动）
test_autonomous_search_websocket.py    snapshot/事件/重连
test_live_exploration_graph.py  地图 schema/revision/产物落盘
```

运行：`python -m pytest tests/test_search_*.py tests/test_autonomous_search_*.py
tests/test_control_ownership.py tests/test_live_exploration_graph.py`

## 11. 真机使用

1. 先启动感知栈（相机/odom/LiDAR）与运动栈（`start_live_perception.sh`、
   `unitree_go2w_control` 的 motion control），确认 `check_go2w_ready.sh`。
2. `bash scripts/go2w/start_nav2_autonomous_search.sh`（Nav2 单链路唯一入口；
   默认只规划，现场验收通过后才加 `--enable-motion`。见 §11.1）。
3. 浏览器输入目标（如"饮水机旁边的蓝色垃圾桶"、"蓝色塑料篮"）。
4. 如需机器人运动，用 `--enable-autonomous-motion` 启动服务；页面提交任务时自动
   继承该权限。未授权时为 dry-run：真实感知/推理但不下发运动。
5. 全程手持遥控器监督；异常时按页面急停或遥控器急停。

搜索 worker 日志：`outputs/autonomous_search/logs/search_worker.log`；
Web 日志：`outputs/autonomous_search/logs/web_server.log`。

### 11.1 静止上电验证（只规划、不运动，计划书 §13.2）

现场人员在**不做任何运动**的前提下，按顺序核对下面 7 项。全程不传
`--enable-motion`；机器人应保持完全静止，可以随时按遥控器急停。

```bash
# 1) 只规划启动：起坐标桥、planner_server 和 bt_navigator，控制器/平滑器/碰撞
#    监视器都不会起，速度到不了机器人。bt_navigator 不带速度链路，但少了它
#    /navigate_to_pose 就不存在，搜索后端连 ComputePathToPose 都不会发。
bash scripts/go2w/start_nav2_autonomous_search.sh
ros2 node list | grep -E 'planner_server|bt_navigator|controller_server'
# 期望：前两个在，controller_server 不在。

# 2) /map 的 frame_id 必须是 map，且只有 1 个发布者。
ros2 topic echo /map --field header.frame_id --once        # 期望：map
ros2 topic info /map -v | grep 'Publisher count:'          # 期望：1

# 3) TF 主链连续、无重复发布者：map -> odom_fused -> base_link 三个 frame
#    都只由 plain_slam_nav_frame_bridge 串起来，所以桥只能有一个进程。
pgrep -af plain_slam_nav_frame_bridge | wc -l              # 期望：1
ros2 run tf2_ros tf2_echo map base_link                    # 应持续有输出，不报
                                                           # "Lookup would require
                                                           #  extrapolation into the future"
# 4) 静止 10 分钟：map -> odom_fused 不应无解释地大跳。取首尾两次读数比平移。
ros2 run tf2_ros tf2_echo map odom_fused      # 记下 translation；10 分钟后再看一次
                                              # 只允许 LIO 的正常微漂（cm 量级），
                                              # 出现米级跳变即为异常，先查日志

# 5) 请求多个 ComputePathToPose（点换几个方向/距离各发一次）。
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, \
   orientation: {w: 1.0}}}, use_start: false}"
# 期望：result 里的 path.poses 非空，且每个点在 RViz 的 global costmap 上落在
# 膨胀层（inflation_radius 1.00 m）之外、并偏向走廊中间，不贴墙。

# 6) 只规划模式下 /go2w/nav2_cmd_vel 根本不该有发布者：碰撞监视器没启动。
ros2 topic info /go2w/nav2_cmd_vel -v | grep 'Publisher count:'   # 期望：0
# 并且机器人一步都不动；网关日志里不应出现任何非零速度放行。
grep -i 'blocked\|allow\|cmd_vel' outputs/autonomous_search/logs/cmd_vel_bridge.log | tail

# 7) 走廊分析的四项（宽度、最小侧向净空、判定、速度上限）落在每个候选决策的
#    结构化记录里，随 navigation_result 事件落盘，同时也进了 WebUI 状态快照的
#    decisions 列表（网页上直接能翻，不用 jq）。
ls -t outputs/live_runs/*/events.jsonl | head -1          # 本次会话的事件流
jq -c 'select(.event=="navigation_result" and .candidate_decision != null)
       | .candidate_decision' "$(ls -t outputs/live_runs/*/events.jsonl | head -1)" | tail -5
# 期望每行含：corridor_decision、min_left_body_gap_m、min_right_body_gap_m、
# min_side_gap_m、speed_limit_mps（0.05~0.10）、accepted，被拒时含 reject_reason
# （取值只有 NO_MAP / NO_LOCALIZATION / NO_PATH / GOAL_OCCUPIED / TOO_NARROW /
# NAV2_NOT_READY 六种）。
```

第 7 项在默认（只规划）模式下的预期形态要说清楚：`accepted` 一定是 `false`，
`reject_reason` 一定是 `NAV2_NOT_READY` —— 路径本身能规划出来、走廊四项也都算得
出来，卡在最后那道「速度上限设不进去」（`controller_server` 没起，速度上限参数
服务不可用）上。这是正确的行为，不是故障：只规划模式本来就只到规划为止。若
`corridor_decision` 缺失而只有 `reject_reason`，那才是问题 —— 说明 `ComputePathToPose`
压根没发出去（先查 `bt_navigator` 与 `planner_server` 的 lifecycle 是否 active）。

以上 7 项全绿之前不要进入 §13.3 的低速现场验收。

## 12. 故障排查

| 症状 | 原因/处理 |
| --- | --- |
| Search 红灯 | worker 未运行或已退出；查看 `/api/search/executor` 与 worker 日志 |
| 相机灰/红 | ROS worker 未起或 enp6s0 断链；`start_live_perception.sh` |
| 开始搜索 409 | 已有会话运行，或手动控制已启用（先停手动） |
| 页面显示离线（OFFLINE） | `/ws/search` 断连，前端自动 1s/2s/5s 重连并重新取快照 |
| 搜索一直在 BOOTSTRAP | backend.health() 未就绪（motion action server 未起） |
| 真机搜索报 BLOCKED | 未授权运动（`--enable-autonomous-motion` / 勾选"自主运动"） |
| 事件里有 ERROR | 查看 message（PERCEPTION_ERROR/LLM_ERROR/BACKEND_ERROR…） |

WebUI 的错误卡片同时显示稳定错误代码、直接原因、原因分类、发生阶段、处理建议和日志
路径。启动失败或 worker 意外退出也会先归档最后快照，再允许开始新任务。

## 13. 未来后端（RobotBackend）

WebUI/Explorer 不写死 `/go2w/*`：搜索状态全部来自通用 `RobotBackend`
（`app/navigation/robot_backend.py`）。未来成熟机器狗实现
`ProductionRobotBackend`（metric pose / NAVIGATE_POSE / 平台避障）后：
`backend_factory.create_backend("production")` 注册 + 启动参数换 backend 即可，
Web 层无需改动（§72）。
