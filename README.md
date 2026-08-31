# robot_scene_demo 从零部署与运行手册

## 当前真机主入口（2026-08-26 统一规划后）

```text
Primary real-robot entry:
  scripts/go2w/run_semantic_exploration.py

世界模型：
  Semantic Topological World Model（PLACE / NAV_EDGE / FRONTIER / OBJECT）
  WebUI 只显示 navigation-aware semantic topology，不再显示 metric spatial map

运动安全：
  forward + rotate + breadcrumb-safe BACKWARD_RECOVERY（b0.05~b0.12m）
  没有已确认 forward breadcrumb 时，自主后退一律 REJECTED

代码状态：
  旧离线视频 demo / Nav2 legacy / spatial map 已退役或移入 docs/archive
```

> **Codex/全新电脑入口：** 先阅读 [`CODEX_DEPLOY.md`](CODEX_DEPLOY.md)，然后运行
> `bash scripts/bootstrap_fresh_machine.sh --profile=full`。该脚本安装并编译全部软件能力，
> 但不会启动或移动机器狗。

本文档目标：把本文件交给 AI 或运维人员后，可以在一台“什么都没配置”的 Ubuntu 机器上，从零部署并跑通完整流程：

- mock 场景理解
- 硅基流动视觉 API 场景理解
- GroundingDINO + SAM2 本地开放词表检测
- LLM-first GroundingDINO Prompt Expansion：把“找到卧室 / 检查打开的门”等自然语言任务转成 DINO 可检测的英文物体、结构和锚点词表
- （兼容保留）第一视角视频目标搜索与视频语义记忆；不是真机自主搜索主链
- 无人工先验的大模型自生成常识推理与观察记忆导航
- Streamlit Web UI
- ROS2 可接收的 `/cmd_vel` 兼容 dry-run 输出
- ROS2 Humble Navigation2 的真实全局规划、导航执行、反馈、取消与路径可视化
- 平台避障辅助下的自适应移动距离 Motion Horizon 输出
- Go2-W 真机：内置 RGB/LiDAR/IMU 采集、LLM 视觉目标搜索（硅基流动 API）、
  wheel+LIO 融合里程计、短步搜索状态机与录像叠加（详见下方
  “Go2-W 真机项目当前进度还原指南”）

项目默认仍是安全的离线/半离线 Demo，不会控制真实机器狗。旧
`ros2_motion_plan.json` 只保留作兼容调试；正式导航链路使用独立的 ROS2 Humble
Nav2 Worker，并且默认 `disabled`。只有环境变量允许、CLI/Web UI 再次确认、
footprint 与急停确认全部通过时，`execute` 模式才会请求 Nav2 执行。

动态运动视界只决定候选观察位姿、搜索半径或停止距离，不直接生成正式避障轨迹。
全局/局部规划交给 Nav2，最终硬件保护仍由 Collision Monitor、机器狗底层、
厂商 SDK 和操作员急停共同负责。

---

# 从零开始完整部署（Zero-to-Run Quickstart）

> 如果你拿到本仓库 GitHub 链接，想让一个 AI 从零部署并实现全部功能，按下面顺序执行。
> 详细硬件/ROS/真机说明见本文档后续章节。

## 0. 环境要求

- 工作站：Ubuntu 22.04，x86_64，16GB+ 内存
- Python：推荐 `miniconda` + `python 3.11`
- ROS：`ros-humble-desktop`（含 `ros2`、`rclpy`、`cv_bridge`）
- 机器狗：Unitree Go2-W，网线直连工作站 `192.168.123.99/24 ↔ 192.168.123.18`
- D435：Intel RealSense D435 通过 USB 接到 Go2-W Jetson，机器狗上运行
  `realsense-stream.service`（HTTP :8080）
- 操作员：真机运动时必须持遥控器监督

## 1. 克隆仓库

```bash
git clone https://github.com/BROVVV/robot_scene_demo.git
cd robot_scene_demo
```

## 2. 创建 Python 环境并安装依赖

```bash
conda create -n go2_robot_scene_demo python=3.11 -y
conda activate go2_robot_scene_demo
pip install -r requirements.txt
```

如果要用 GroundingDINO + SAM2 本地检测，还需要安装
`Grounded-SAM-2` 依赖（见 `docs/` 或仓库根目录说明）。

## 3. 安装/准备 ROS Humble

```bash
sudo apt install -y ros-humble-desktop ros-humble-cv-bridge
# 项目 ROS2 工作空间
cd ros2_ws
colcon build --symlink-install
cd ..
```

RTAB-Map（RGB-D 空间探索需要）：

```bash
sudo apt install -y ros-humble-rtabmap-ros
```

### 3.1 部署 Go2-W 高层运动控制（本仓库已内置）

截图中缺少的 `go2w_motion_interfaces`、`go2w_motion_control`、
`/go2w/motion` Action、`/go2w/arm` 和 `/go2w/emergency_stop` 已随本仓库放在
`unitree_go2w_control/`。新电脑只需执行一次：

```bash
bash scripts/go2w/install_dependencies.sh
bash scripts/go2w/setup_go2w_control.sh
```

脚本会从 Unitree 官方仓库准备 `unitree_ros2` 消息依赖并编译控制包；它只安装和
编译，不会启动节点、申请 lease 或让机器狗运动。启动控制服务使用：

```bash
bash scripts/go2w/start_motion_control.sh
```

硅基流动 API 密钥不在 GitHub 中。请复制 `.env.example` 为 `.env`，只在本机填写
`SILICONFLOW_API_KEY`，不要把 `.env` 提交到仓库。

## 4. 配置环境变量

```bash
cp .env.example .env
# 编辑 .env，至少填入：
# SILICONFLOW_API_KEY=你的硅基流动 API Key
```

真机模式可参考 `.env.go2w`。

## 5. 机器狗与 D435 部署

```bash
# 机器狗上电、网线连通后，先做健康检查
bash scripts/go2w/check_go2w_ready.sh --json

# 部署 D435 HTTP 流服务（首次/更新）
bash scripts/go2w/start_realsense_stream.sh install

# 验证 D435 原子 RGB-D
python scripts/go2w/validate_rgbd_spatial_stack.py
```

## 6. 先跑离线/纯软件功能

```bash
# Mock 场景理解 + SemanticNavigation V2 空间探索
python scripts/go2w/run_semantic_exploration.py --target "饮水机旁边的蓝色垃圾桶" \
  --backend mock --max-seconds 30 --max-planning-cycles 5

# 兼容离线视频目标搜索（不是真机自主搜索主链）
python run_video_demo.py --input 你的视频.mp4 --target "蓝色垃圾桶"
```

## 7. 启动 WebUI（离线 mock 或真机）

```bash
# Mock 前端（不需要机器狗）
bash scripts/go2w/start_autonomous_search_web.sh --mock

# 真机只读 WebUI
bash scripts/go2w/start_autonomous_search_web.sh

# 真机 + 自主运动授权（操作员持遥控器）
bash scripts/go2w/start_autonomous_search_web.sh --enable-autonomous-motion
```

浏览器打开：`http://127.0.0.1:8765`

## 8. 启动 RGB-D 空间探索栈（D435 + RTAB-Map）

```bash
# 启动 D435 ROS2 Bridge + static TF + RTAB-Map
bash scripts/go2w/start_rgbd_spatial_stack.sh start

# 真机 SemanticNavigation V2 空间搜索（dry-run，不运动）
/usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
  --target "不存在的红色独角兽" \
  --backend go2w_experimental --reasoner semantic_navigation \
  --rgbd-source --spatial-v2 --rtabmap --dry-run-motion \
  --max-planning-cycles 1 --max-motion-steps 1

# 真机 SemanticNavigation V2 空间搜索（真实运动，需操作员持遥控器）
/usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
  --target "不存在的红色独角兽" \
  --backend go2w_experimental --reasoner semantic_navigation \
  --rgbd-source --spatial-v2 --rtabmap \
  --operator-supervised-experiment \
  --max-local-rotations 1 \
  --max-seconds 600 --max-planning-cycles 5 --max-motion-steps 5
```

## 9. 运行测试

```bash
cd robot_scene_demo
python -m pytest tests/ -q
```

> 注意：`tests/` 目录下部分测试需要 ROS2/GPU 环境；纯 Python 核心测试可单独跑：
> `pytest tests/test_rgbd_source.py tests/test_depth_object_localizer.py ...`

## 10. 停止

```bash
bash scripts/go2w/stop_autonomous_search_web.sh
bash scripts/go2w/start_rgbd_spatial_stack.sh stop
bash scripts/go2w/check_go2w_ready.sh --json
```

---

## Operator-Supervised High-Level Semantic Exploration（2026-08-17 新增）

> 本节描述本仓库新增的**高层自主语义探索实验系统**：自然语言目标 → 实时观察 →
> SceneGraph/GoalGraph → SemanticNavigation 语义匹配 → 空间语义记忆 → 候选探索目标 →
> 平台无关 RobotBackend → 连续执行 → 重规划 → 视觉确认 → TARGET_FOUND。
> 详细设计见 `docs/HIGH_LEVEL_AUTONOMOUS_SEMANTIC_EXPLORATION.md`。

### 用途

- 把已有的 SemanticNavigation / SceneGraph / 搜索状态机 / 探索骨架收敛为**连续长周期闭环**：
  `observe → match → verify → update memory → plan → execute → replan → …`。
- 当前 Go2-W 以 `go2w_experimental`（Relative + Topological）backend 运行；
  未来成熟机器狗只需实现 `RobotBackend`（metric 模式），高层代码不变。

### 这不是 production safety mode

- 实验 profile 为 `operator_supervised_experiment`（`configs/go2w/high_level_experiment.yaml`）：
  `production_safe: false`、`research_only: true`、`operator_present: true`。
- **不要求** Pandar 正式外参 / Stage2 readiness / 四方向物理证据 / Nav2 gate。
- 转向仍受既有运动门禁约束：无旋转 lease 时转向需要
  `--operator-supervised-experiment`（等价操作者授权，单次 ≤30°，其余安全门全部保留）。
- 禁止 LowCmd / 关节控制 / 固件修改；所有运动走 `/go2w/motion` + `/go2w/arm` +
  `/go2w/emergency_stop`。

### 启动条件

1. 机器狗已上电，网线接通（本机 `192.168.123.99/24` ↔ 狗 `192.168.123.18`），
   遥控器在手可随时急停。
2. 操作者全程在场监督，只负责急停，不给方向建议。
3. `.env` 已配置 `SILICONFLOW_API_KEY`。

上电后先跑一键健康检查（验证机器狗各项功能：网络/sport 模式/里程计/相机/
安全话题/motion Action/急停服务/Bundle 新鲜度/LLM key，输出机器可读 JSON）：

```bash
bash scripts/go2w/check_go2w_ready.sh          # 人类可读 + JSON
bash scripts/go2w/check_go2w_ready.sh --json   # 只输出 JSON
# 退出码：0=ready 1=degraded(非阻塞项缺失) 2=unreachable(未连接/未上电)
```

### 一键命令

```bash
cd /home/brov/robot/robot_scene_demo
bash scripts/go2w/start_semantic_exploration.sh --target "饮水机旁边的蓝色垃圾桶"
```

launcher 自动：网络预检 → source ROS 环境 → 启动只读感知栈与轮式里程计（如未运行）→
校验 `/go2w/motion` Action server（不可用则输出 `MOTION_BACKEND_UNAVAILABLE` 并退出）→
运行探索 CLI。也可直接：

```bash
/usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
  --target "饮水机旁边的蓝色垃圾桶" \
  --backend go2w_experimental --reasoner semantic_navigation \
  --operator-supervised-experiment --finish-on-visual-confirmation \
  --max-seconds 600 --max-motion-steps 50 \
  --output outputs/live_sessions/high_level_semantic_exploration.jsonl
```

离线（不需要机器狗）验证同一闭环：

```bash
/home/brov/miniconda3/envs/go2_robot_scene_demo/bin/python \
  scripts/go2w/run_semantic_exploration.py --target "饮水机旁边的蓝色垃圾桶" \
  --backend mock --mock-scenario anchor_then_target
```

### 真机验证结果（2026-08-17）

| 实验 | 目标 | 结果 |
| --- | --- | --- |
| 健康检查 `check_go2w_ready.sh` | — | `state=ready`（sport mode=1/error=0、odom 20Hz、相机 16.7Hz、lidar_fresh、motion/arm/急停、Bundle、LLM key） |
| Trial 1 turn-only | 绿色垃圾桶 | 8 个自主规划周期、8 次转向全部 odom 验证（MAX_STEPS_REACHED，6/8 由 SemanticNavigation 选向） |
| Trial 2 转向+短步 | 绿色垃圾桶 | **13 个自主规划周期、12 次运动**、1 次导航失败自动 replan（MAX_STEPS_REACHED） |
| Trial 4 普通目标 | 蓝色垃圾桶 | **TARGET_FOUND（84 s，0 运动）** |
| Trial 5 关系目标 | 饮水机旁边的蓝色垃圾桶 | **TARGET_FOUND（86 s，0 运动）**，verify 确认“位于饮水机旁边” |

真机 JSONL/graph/summary：`outputs/live_sessions/go2w_trial*.jsonl`、
`outputs/live_runs/explore_go2w_*/`。

### 输出目录

- 会话事件 JSONL：`outputs/live_sessions/*.jsonl`（含每次 observation/match/verify/
  memory_update/selected_goal/navigation_result/replan，可完全回放）。
- `outputs/live_runs/<session_id>/exploration_graph.json`（节点/边/扇区覆盖/失败计数）。
- `outputs/live_runs/<session_id>/summary.json`（TARGET_FOUND/TIMEOUT/SEARCH_EXHAUSTED/
  OPERATOR_STOP 等终止原因与统计）。

### 停止方式

- 遥控器急停（物理最高优先级）。
- Ctrl-C 会走既有清理路径：紧急停止 → disarm → 关闭输出。
- 会话内 `request_stop()`（`OPERATOR_STOP`）会立即 `backend.stop()` 并结束，不自动继续。

### 未来机器狗接入（RobotBackend）

高层（SemanticNavigation/SceneGraph/Memory/Planner/Explorer）只依赖
`app/navigation/robot_backend.py` 的协议与 `app/navigation/models.py` 的
`ExplorationGoal`。未来机器人实现：

```python
class ProductionRobotBackend(RobotBackend):
    def capabilities(self): ...      # supports_global_pose=true, supports_metric_navigation=true
    def get_pose(self): ...          # PoseQuality.METRIC, frame=map
    def execute_goal(self, goal): ...  # NAVIGATE_POSE → 底层 SLAM/规划/避障/轨迹跟踪
    ...
```

`backend_factory.create_backend("...")` 注册新 backend 后，CLI/Explorer 无需改动。
离线用 `--backend mock_metric` 即可验证 metric 路径（同一 Explorer 生成
`NAVIGATE_POSE` 目标）。

## Go2-W Autonomous Semantic Search WebUI（2026-08-17 新增）

在既有 Manual WASD Demo 的同一个 FastAPI 服务上叠加"真机自主语义搜索 Web 控制台"
（计划书 `robot_scene_demo_真机自主语义搜索_WebUI_一次性实施计划书_20260817.md`）。

三个 Tab：

- **自主搜索**：输入自然语言目标 → 开始/暂停/继续/停止/急停；实时显示相机
  （含检测框 overlay）、当前物体、Session 累计物体、目标/锚点/关系证据、搜索阶段、
  下一步决策（意图 + 理由 + 评分分量）、Candidates 排名、SVG 探索拓扑地图、
  事件时间线；调试视图显示 GoalGraph / SceneGraph / Candidates / 原始状态。
- **手动控制**：保留原有 WASD+QE 手动控制与场景物体表。
- **系统状态**：Camera / ROS Worker / Motion / Control Owner / Search / LLM /
  搜索就绪检查 / 历史会话。

一键启动（与手动 Demo 同端口 8765，会先停掉项目自有的旧 Web 进程）：

```bash
cd /home/brov/robot/robot_scene_demo
bash scripts/go2w/start_autonomous_search_web.sh                     # 只读启动（相机+LLM+手动，搜索可 dry-run）
bash scripts/go2w/start_autonomous_search_web.sh --enable-autonomous-motion  # 授权自主运动（<=30°转向/<=0.30m 前进）
bash scripts/go2w/start_autonomous_search_web.sh --mock              # 离线前端开发（mock 后端，无需 ROS）
bash scripts/go2w/stop_autonomous_search_web.sh                      # 停止（只停项目自有 web.pid）
```

然后浏览器打开 `http://127.0.0.1:8765`，在"自主搜索"Tab 只需输入自然语言目标并点击
"开始搜索"。运动权限由启动命令决定：使用 `--enable-autonomous-motion` 启动即授权
本服务内的搜索任务，未授权则自动 dry-run，页面不会再用复选框覆盖服务配置。

> 如果再次点击出现 **"无法开始: search already active in state STARTING"**：这是上一次
> 会话的 worker 子进程在启动阶段退出/卡死但状态没被回收导致的。已内置自动修复——
> 服务端会在新请求到达时自动识别已经退出或超过 120s 的僵尸 worker，把退出前状态和
> 具体中断原因归档，再允许新会话；仍健康运行的旧任务不会被刷新后的浏览器误停止。

> 如果输入目标后立刻 **"搜索结束: FAILED"**：现在界面上会直接显示具体失败原因
> （例如缺依赖 / rclpy 缺失）。常见情况与解决：
> - **本机/开发环境没有 ROS**：这是正常现象（真机搜索需要 ROS 的 rclpy 驱动运动）。
>   想先看完整效果，用离线模式：
>   ```bash
>   bash scripts/go2w/start_autonomous_search_web.sh --mock   # 无需 ROS/真机
>   ```
>   然后在浏览器输入「绿色垃圾桶」即可看到：搜索→建图→语义拓扑→找到目标的全流程。
> - **真机**：用 `--enable-autonomous-motion` 启动；
>   后端会自动挑选带 rclpy 的 ROS Python 来跑搜索 worker（优先 `/usr/bin/python3`，
>   可用 `GO2W_WORKER_PYTHON` 显式覆盖）。若仍失败，`outputs/autonomous_search/logs/
>   search_worker.log` 里有完整错误。

**地图默认显示「语义拓扑」**：识别出的持久物体（`obj_xxx`）会实时聚成关系拓扑图
（重复识别不新建节点/边，同类别多物体不混淆；节点位置是显示布局、不用物理坐标）。
右上角可切换「空间地图」查看 RTAB / Place / Frontier / 轨迹等导航调试数据。
D435 RGB-D 镜头默认 **1280×720** 满宽度模式（约 69° 彩色 / 87° 深度视场）；用
`bash scripts/go2w/tune_d435_fov.sh status|wide|mode|reset` 可在真机上 SSH 查看/调整。

关键接口（详见 `docs/GO2W_AUTONOMOUS_SEARCH_WEBUI.md`）：

```text
POST /api/search/start   {task_text}（高级字段默认继承服务启动配置） → 立即返回 session_id
POST /api/search/pause | resume | stop | estop
GET  /api/search/state | map | spatial-map | place-graph | frontiers | semantic-map | objects | events | history | readiness | executor
GET  /api/search/history/{session_id}  → 完整状态、决策、拓扑、事件和产物索引
WS   /ws/search         连接即发 snapshot → 随后增量 SearchEvent（心跳 15s，自动重连）
```

架构：浏览器只负责发任务/暂停/停止/展示；决策全部来自后端
`AutonomousExplorer`（OBSERVE→MATCH→VERIFY→UPDATE_MEMORY→PLAN→EXECUTE→REPLAN），
事件经 `SearchEventBus` → WebSocket；真机搜索在独立子进程
（`scripts/go2w/autonomous_search_worker.py`，ROS2 系统 Python，JSONL stdin/stdout IPC）
中运行，FastAPI 进程保持 Conda 环境（计划书 §12/§86）。Manual / Autonomous 运动
控制权互斥由 `ControlOwner` 保证，急停同时停止搜索与会话。

输出：每次任务在 `outputs/live_runs/<session_id>/` 保存 worker 原始产物，并额外原子保存
`webui_state.json`、`webui_events.jsonl`、`webui_session.json`。页面刷新、停止任务、
搜索失败或 Web 服务重启后仍可回看；WebUI 完整记录滚动保留最近 10 次（不删除旧的
非 WebUI CLI/实验运行目录）。

## RGB-D Spatial Semantic Exploration（2026-08-18 新增）

当前项目已把 D435 作为主 RGB-D 传感器，并新增 SemanticNavigation V2 空间探索所需的核心模块：

```text
D435 /rgbd/latest.json + /rgbd/frame/<id>/{color.jpg,depth.png}
  → RGBDSource → DepthObjectLocalizer → 3D SemanticObjectMap
  → SpatialProvider / FrontierExtractor / PlaceGraph
  → PSG SemanticPriorProvider → SemanticNavigation V2 SpatialReasoner
  → LongTermGoalSelector → LocalGoalExecutor → RobotBackend
```

关键入口：

```bash
# 真机语义搜索时使用 D435 主 RGB（WebUI 默认已开启 rgbd_source）
bash scripts/go2w/start_autonomous_search_web.sh --enable-autonomous-motion

# 直接 CLI 使用 D435 RGB-D + SemanticNavigation V2 空间探索
/usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
  --target "饮水机旁边的蓝色垃圾桶" \
  --backend go2w_experimental --reasoner semantic_navigation \
  --operator-supervised-experiment --rgbd-source --spatial-v2 \
  --output outputs/live_sessions/rgbd_spatial_exploration.jsonl

# 无运动 dry-run 验证 V2 链路（D435 + PlaceGraph + Frontier + LocalExecutor）
/usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
  --target "绿色垃圾桶" --backend go2w_experimental --reasoner semantic_navigation \
  --rgbd-source --spatial-v2 --dry-run-motion \
  --max-seconds 60 --max-planning-cycles 2 \
  --output outputs/live_sessions/rgbd_v2_dryrun.jsonl

# 使用 RTAB-Map（需先 start_rgbd_spatial_stack.sh start）
/usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
  --target "不存在的红色独角兽" --backend go2w_experimental --reasoner semantic_navigation \
  --rgbd-source --spatial-v2 --rtabmap --operator-supervised-experiment \
  --turn-only --max-planning-cycles 1 \
  --output outputs/live_sessions/rgbd_v2_rtabmap.jsonl

# 纯软件验证 D435 原子帧 + 深度有效比例
/home/brov/miniconda3/envs/go2_robot_scene_demo/bin/python \
  scripts/go2w/validate_rgbd_spatial_stack.py

# 启动 RTAB-Map + D435 ROS2 Bridge（可选，供 --rtabmap 使用）
bash scripts/go2w/start_rgbd_spatial_stack.sh start
```

详细文档：`docs/RGBD_SEMANTIC_NAVIGATION_V2_SPATIAL_EXPLORATION.md`。

### 在线语义建图 × 导航融合 × 可解释决策（2026-08-19 一次性修改）

本次修改把视觉识别结果从“一次性 detection”升级为**持久空间语义世界模型**
（Persistent Spatial Semantic World Model），并打通了
`camera_xyz → map_xyz → Place/Object 实体图 → 路线 → 结构化决策 → WebUI`。

#### 新模块

```text
app/spatial/spatial_transform.py         # 纯 Python camera→base→world 变换（可离线测试）
app/spatial/semantic_entity_graph.py     # PLACE/OBJECT + MOVED_TO/OBSERVED_FROM 实体图
app/navigation/semantic_route_planner.py # metric grid A* + 拓扑 PlaceGraph 路线回退
app/navigation/decision_record.py        # 结构化、可复现、可解释的决策记录
```

#### 升级的既有模块

```text
app/spatial/rtabmap_spatial_provider.py  # 实现 camera_point_to_spatial（TF2 优先 + nominal 回退）
app/spatial/place_graph.py               # 全局最近 Place 关联 / revisit / movement edge / pose 融合
app/spatial/semantic_object_map.py       # 一对一 data association + 动态阈值 + 概率位置融合 + lifecycle
app/navigation/long_term_goal_selector.py# 真实 semantic_relevance / route_cost / negative_evidence 评分
app/live_robot/search_state_store.py     # spatial.semantic_graph / route_plan / startup stage
```

#### 关键链路

1. **camera_xyz → map_xyz**：D435 optical 帧 → base_link（轴约定修正）→ map/odom。
   TF2 可用时走真值；否则用 nominal 外参 + 平面 yaw 回退（声明为 `RELATIVE_RGBD`，
   绝不假装 `METRIC_RGBD`）。
2. **实体去重**：几何为主证据、语义为 gate、一对一 greedy 分配；`obj_id` 稳定持久，
   跨视角回到同一物体不复建节点，两个并排同类物体不会被误合并。
3. **路线**：`SemanticRoutePlanner` 在 occupancy grid 上做膨胀 A*，无地图时用
   PlaceGraph 拓扑最短路径。
4. **决策**：`DecisionRecord` 携带真实 `score_breakdown`、证据、未选候选原因、
   `reason_zh`（规则模板，无 LLM chain-of-thought），不再是 `spatial_v2=1.0` 占位。

#### 真机启动顺序

```bash
# 1) 健康检查
bash scripts/go2w/check_go2w_ready.sh --json

# 2) D435 + RTAB 空间栈（含 TF 健康检查）
bash scripts/go2w/start_rgbd_spatial_stack.sh start
bash scripts/go2w/start_rgbd_spatial_stack.sh check

# 3) dry-run（零运动验证空间链）
/usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
  --target "绿色垃圾桶" --backend go2w_experimental --reasoner semantic_navigation \
  --rgbd-source --spatial-v2 --rtabmap --dry-run-motion \
  --max-planning-cycles 5 --max-motion-steps 5

# 4) WebUI（rtabmap=auto，无 RTAB 时自动降级）
bash scripts/go2w/start_autonomous_search_web.sh --enable-autonomous-motion
```

#### 已知 degraded 行为

- 无 RTAB → `RELATIVE_RGBD`（BEV / nominal 外参）
- TF 不可用 → nominal optical 外参
- 无 metric backend → 短步 receding-horizon 执行

### WebUI 真机验证结果（2026-08-17，机器狗重启后）

| 验收项 | 结果 |
| --- | --- |
| A 相机/状态 Web 启动 | PASS（camera fresh、readiness ready） |
| B 搜索 dry-run（真实相机+LLM+SemanticNavigation+Planner，零运动） | **TARGET_FOUND**（50 s，1 cycle，识别 blue_plastic_basket） |
| C turn-only 真机搜索（≤30° 转向） | **TARGET_FOUND**：1 次真实自主转向（SemanticNavigation 选向）后第 2 周期命中"绿色垃圾桶" |
| D 连续自主循环（12 周期） | **8 次真实转向全部成功**、8/8 由 SemanticNavigation 选向、覆盖 8 个朝向扇区、MAX_STEPS_REACHED 正常收尾 |
| Pause/Resume 真机验证 | 真实搜索中暂停 → 当前转向完成后停车（零运动保持）→ 恢复继续 |

## Go2-W 真机项目当前进度还原指南（2026-08-15）

> 本节是“照着做就能还原当前真机进度”的权威步骤。项目其余章节保留离线/视频/
> Nav2 软件流程。硬件证据（bag、录像、JSONL）体积大且含私人画面，**不进入
> 本仓库**；本节给出证据所在的本机路径。

### 0.1 当前能力状态（真机验收结论）

| 能力 | 状态 | 说明 / 证据 |
|---|---|---|
| 内置 RGB ROS2 桥（RPC） | PASS | `/camera/front/image_raw(+compressed)`、CameraInfo；损坏帧跳过 + 自动重连 |
| 相机内参（9×6，15 mm） | PASS | `configs/go2w/camera_intrinsics.yaml`；105 视角 |
| LiDAR/IMU 时间桥 | PASS | `configs/go2w/time_sync.yaml`；云 RMSE <1 ms |
| base→LiDAR TF | PASS（实机复核） | `official_reference.yaml`，pitch −15.09°（z-up） |
| LiDAR 预处理 /scan / clearance | PARTIAL（静止实测） | 碰撞高度/轮组自滤/720-bin PASS；旋转包络 **BLOCKED** |
| 外接 Hesai PandarXT-16 raw | PASS（隔离只读出云） | 10 Hz、64,000 点/帧、16 ring、累计丢包 0；正式外参 **BLOCKED** |
| Pandar 诊断预处理 | CODE READY / PHYSICAL PENDING | `hesai_pandarxt16_preprocessor` 节点；zero-return（<=0.05 m）过滤；时钟 tier 默认 `host_receive_time_only` |
| Pandar 正式外参 | **BLOCKED / CANDIDATE** | 照片+静态配准候选不发布 TF；需多场景配准 + self-occlusion 验收 |
| Pandar 安全融合 | **BLOCKED** | `dual_lidar_safety.yaml` 默认 disabled；Pandar 只在 validated 后贡献正式 CLEAR |
| 双雷达旋转可观测性 | **BLOCKED**（工具就绪） | 内置 L2 720/720 不可观测；Pandar validated 后可互补，仍按实时证据 |
| 轮式里程计 `/go2w/odom/wheel` | EXPERIMENTAL | 四轮 dq×0.089 m + Sport yaw；转弯跳过平移 |
| 融合里程计 `/go2w/odom/fused` | EXPERIMENTAL | 轮式平移 + Sport/LIO 融合航向；LIO 门禁自动回退 |
| Point-LIO 转向（yaw） | PASS | ±10° 实测 89% 幅度、符号正确（yaw_reflect） |
| Point-LIO 平移 | **BLOCKED** | 0.2/0.4 滤波均失败（偏 69°/塌缩/发散），只用其 yaw |
| USLAM `/uslam/*` | **BLOCKED** | 当前固件未启用 |
| LLM 快速检测（硅基流动） | PASS | `--detector llm`，默认 30B-A3B，单次 5–15 s |
| LLM 目标复核（`--verify`） | PASS | 到达前确认物体身份，防椅子/书包误判 |
| SemanticNavigation V1 software core | PASS | GoalGraph/GraphMatcher/Memory/Reasoner/Router 完整，KEEP 审计见 `reports/semantic_navigation_v1_existing_implementation_audit_20260813.md` |
| SemanticNavigation Observe-only | PASS | 真实 Bundle/LLM/Graph/关系证据与 2D visual confirmation 通过 |
| SemanticNavigation Shadow | PASS（无运动） | `actual_shadow_behavior_matches_legacy=true`，`dangerous_forward_request_count=0` |
| SemanticNavigation Active turn-only | **BLOCKED（正式）** / 操作者授权单次转向已验证 | 2026-08-14：`--operator-authorized-rotation` 单次转向实测通过（reasoner 决策→实际转向 30°，odom 验证）；正式 Stage 2 仍需 readiness 12 项全真 |
| SemanticNavigation 语义决策→运动闭环 | **PASS（操作者授权单次）** | reasoner `inspect_anchor` 决策 30° 转向并实际执行 29.5°；证据 `outputs/go2w_acceptance/semantic_turn_execute.*` |
| SemanticNavigation Short-forward | **BLOCKED** | 严格依赖 Stage 2 PASS；semantic forward 默认关闭 |
| 当前整机外廓 | `0.70 × 0.43 × 0.70 m` | `configs/go2w/current_hardware_geometry.yaml`；最高点 = PandarXT-16 固定保护框架 |
| RGB–LiDAR 外参/3D 定位 | EXPERIMENTAL（未几何确认） | 诊断叠加可用；导航外参与 3D 输出门禁为 false |
| Camera TF | **BLOCKED** | 官方 URDF 无相机 link，物理 TF 未测 |
| Level D / 地图 / Nav2 | **BLOCKED** | `navigation_gate.yaml` fail-closed 未改动 |

### 0.1.1 Go2-W Manual WASD Web Demo（独立手动 Demo）

独立的浏览器手动控制 Demo：实时相机 + WASD+QE 小范围运动 + SiliconFlow 场景物体表。
不使用 SemanticNavigation/Nav2/3D；相机来自正式 camera bridge；W/S/A/D/Q/E 通过现有高层运动
控制链（`/go2w/motion`）执行——W/S 前后、A/D 横移、Q/E 转向；直接复用项目现有
SiliconFlow 视觉配置，后台异步列出主要物体。详见
[`docs/GO2W_MANUAL_WASD_WEB_DEMO.md`](docs/GO2W_MANUAL_WASD_WEB_DEMO.md)。

```bash
# 相机 + LLM（键盘控制默认禁用）
bash scripts/go2w/start_manual_web_demo.sh

# 允许运动（需清场确认）
bash scripts/go2w/start_manual_web_demo.sh --enable-motion

# 停止
bash scripts/go2w/stop_manual_web_demo.sh
```

### 0.2 硬件与网络前置

- Ubuntu 22.04 x86_64 + ROS2 Humble；至少 16 GB RAM；
- 主机直连网口 `enp6s0`：`192.168.123.99/24`，机器人 `192.168.123.18`；
  新增 PandarXT-16 为 `192.168.123.20`，点云只单播到工作站
  `192.168.123.99:2368`；
- 机器狗处于 `ai-w` 运动模式，`/lf/sportmodestate` 的 `mode=1, error_code=0`；
- 运动授权：操作者明确授权 `GO2W_MOTION_READY`；任何运动必须位于授权时初始位姿
  的固定正前方 180° 半平面内，且距该初始位置半径 ≤1.5 m；场地与门禁不满足时
  fail-closed，遥控器可急停。

### 0.3 依赖与当前仓库边界

1. **unitree_go2w_control 已内置**：源码、ROS 2 Action/服务包、启动脚本和
   BSD 许可的 Unitree SDK Python 源码位于仓库 `unitree_go2w_control/`；
2. **unitree_ros2 / cyclonedds_ws**：由 `scripts/go2w/setup_go2w_control.sh` 从
   Unitree 官方仓库自动克隆并编译，不把官方构建产物复制进本项目；
3. **Grounded-SAM-2**（可选，`--detector grounded_sam` 才需要）；
4. **Point-LIO 隔离环境**：`conda env go2w_point_lio_noetic` +
   `point_lio_ws`（构建时应用 `patches/go2w/point_lio_noetic_pcl115.patch`）；
5. **Hesai ROS 2.0 官方驱动**：位于 `ros2_ws/src/hesai_ros_driver`，固定提交
   `e7e112f0809f0eed5e3c81c55a1a0376474db234`，SDK 子模块固定
   `9d5dc4fc4ade5be5f6a6ca00e71dd4050b054168`。仅用于外接雷达隔离诊断。

外接雷达照片与静态地面拟合已生成不发布 TF 的安装候选：`xyz=(+0.130,
+0.015,+0.014) m`，`rpy=(+0.385,+0.905,+11.357)°`。候选文件
`configs/go2w/hesai_pandarxt16_mount_candidate.yaml` 明确设置全部授权为 false。

**当前整机几何（操作者确认，2026-08-13）：`0.70 × 0.43 × 0.70 m`，最高点为
PandarXT-16 固定保护框架（不是松弛电缆）。不再安排整机长宽高复测。** 该几何
记录于 `configs/go2w/current_hardware_geometry.yaml`，仅描述当前加装后的整机
外廓，`authorizes_motion=false`。正式外参仍需多场景配准和实测；诊断/时钟/自遮挡/
双雷达可观测性工具已就绪，见 `docs/PANDARXT16_DUAL_LIDAR_SAFETY_INTEGRATION.md`。

### 0.4 环境与构建（一次性）

```bash
# 主项目：Conda 环境 go2_robot_scene_demo + 依赖
cd /home/brov/robot/robot_scene_demo
bash scripts/go2w/install_dependencies.sh

# ROS2 工作区（系统 Python /usr/bin/python3）
bash scripts/go2w/build_ros2.sh

# Point-LIO（Noetic 隔离；首次需要 clone 上游 + 打补丁）
bash scripts/go2w/setup_point_lio_noetic.sh

# 运动控制工作区（源码已在本仓库内）
bash scripts/go2w/setup_go2w_control.sh
```

配置模板（**不含任何密钥**）：

```bash
cp .env.example .env          # 填入 SILICONFLOW_API_KEY（不要提交）
# .env.go2w 已在本仓库，按需使用
```

如果当前副本缺少 Hesai 驱动，先固定安装官方版本再构建：

```bash
git clone --recurse-submodules https://github.com/HesaiTechnology/HesaiLidar_ROS_2.0.git \
  ros2_ws/src/hesai_ros_driver
git -C ros2_ws/src/hesai_ros_driver checkout \
  e7e112f0809f0eed5e3c81c55a1a0376474db234
git -C ros2_ws/src/hesai_ros_driver submodule update --init --recursive
bash scripts/go2w/build_ros2.sh
```

### 0.5 启动顺序（每次真机运行）

```bash
# 终端 1：只读感知栈（相机/LiDAR/时间/融合/Bundle）
cd /home/brov/robot/robot_scene_demo
bash scripts/go2w/start_live_perception.sh

# 可选终端：新增 PandarXT-16 隔离诊断；不接入 /go2w/lidar/* 或运动安全链
# --with-preprocessor 额外启动 zero-return 过滤 + 诊断状态节点（/go2w/hesai/*）
bash scripts/go2w/start_hesai_pandarxt16.sh --with-preprocessor

# 另一个已 source ROS 工作区的终端：20 帧只读验收
/usr/bin/python3 scripts/go2w/validate_hesai_pandarxt16_ros.py \
  --output outputs/go2w_acceptance/hesai_pandarxt16_20260813/result.json

# Pandar 时钟分级与双雷达旋转可观测性（只读，不授权运动）
/usr/bin/python3 scripts/go2w/validate_pandarxt16_clock.py --samples 30 \
  --output outputs/go2w_acceptance/pandarxt16_clock/result.json
/usr/bin/python3 scripts/go2w/validate_dual_lidar_observability_ros.py \
  --output outputs/go2w_acceptance/dual_lidar_observability/result.json

# 多场景外参采集 -> 离线标定 -> 外参验证（全部 fail-closed，不发布 TF）
/usr/bin/python3 scripts/go2w/capture_dual_lidar_calibration_ros.py --scene corner_01 --frames 8
/usr/bin/python3 scripts/go2w/calibrate_pandarxt16_extrinsics.py \
  --capture-dir outputs/go2w_acceptance/dual_lidar_calibration
/usr/bin/python3 scripts/go2w/validate_pandarxt16_extrinsics.py --output <path>

# Stage 2 machine-readable readiness（不运动；输出 JSON 后退出）
/usr/bin/python3 scripts/go2w/run_autonomous_loop.py --stage2-readiness \
  outputs/go2w_acceptance/stage2_readiness.json

# 终端 2：轮式 + 融合里程计（/go2w/odom/wheel 与 /go2w/odom/fused）
source /opt/ros/humble/setup.bash
source "${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}/cyclonedds_ws/install/setup.bash"
source /home/brov/robot/robot_scene_demo/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file:///home/brov/robot/robot_scene_demo/configs/go2w/cyclonedds_go2w.xml"
ros2 launch go2w_lio_bringup wheel_odom.launch.py

# 终端 3：运动控制（lease holder + Action server）
bash scripts/go2w/start_motion_control.sh

# 终端 4（可选，供融合航向）：Point-LIO + 桥
cd /home/brov/robot/robot_scene_demo
POINT_LIO_OUTPUT_DIR=outputs/go2w_acceptance/lio_xxx \
POINT_LIO_USE_IMU_AS_INPUT=false \
POINT_LIO_FILTER_SIZE_SURF=0.2 POINT_LIO_FILTER_SIZE_MAP=0.2 \
scripts/go2w/run_point_lio_ros1.sh
# 另开终端：
ros2 launch go2w_lio_bringup point_lio.launch.py \
  lio_config:=configs/go2w/point_lio.yaml \
  reference_config:=configs/go2w/official_reference.yaml \
  time_config:=configs/go2w/time_sync.yaml
```

启动后自检：

```bash
ros2 topic hz /camera/front/image_raw      # ~15–28 Hz
ros2 topic hz /go2w/odom/wheel             # 20 Hz
ros2 topic hz /go2w/odom/fused             # 20 Hz
ros2 topic echo /go2w/odom/fused/status --once
ros2 topic echo /lf/sportmodestate --once  # mode=1 error_code=0
```

### 0.6 自主搜索运行命令

```bash
cd /home/brov/robot/robot_scene_demo
source /opt/ros/humble/setup.bash
source "${GO2W_UNITREE_ROOT:-$HOME/unitree_ros2}/cyclonedds_ws/install/setup.bash"
source unitree_go2w_control/ros2_ws/install/setup.bash
source /home/brov/robot/robot_scene_demo/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file:///home/brov/robot/robot_scene_demo/configs/go2w/cyclonedds_go2w.xml"

# 360° 扫描 → 高分提前停止 → 靠近 → LLM 复核 → 到达（推荐演示）
/usr/bin/python3 scripts/go2w/run_autonomous_loop.py \
  --mode scan360_approach --target "灰色书包" --detector llm \
  --llm-model Qwen/Qwen3-VL-30B-A3B-Instruct \
  --target-score-min 0.45 --max-radius 1.0 --max-seconds 420 \
  --reach-area-ratio 0.08 --odom-topic /go2w/odom/fused \
  --record-video outputs/go2w_acceptance/scan360_demo.mp4 \
  --output outputs/go2w_acceptance/scan360_demo.jsonl

# Level A 摆动搜索（发现→对齐→靠近）
/usr/bin/python3 scripts/go2w/run_autonomous_loop.py \
  --mode level_a_search --target "手机" --detector llm \
  --llm-model Qwen/Qwen3-VL-30B-A3B-Instruct \
  --max-radius 1.0 --max-seconds 300 --odom-topic /go2w/odom/fused \
  --output outputs/go2w_acceptance/level_a_demo.jsonl

# 正式 app/live_robot 状态机驱动
/usr/bin/python3 scripts/go2w/run_autonomous_loop.py \
  --mode state_machine_search --target "灰色书包" --detector llm \
  --llm-model Qwen/Qwen3-VL-30B-A3B-Instruct \
  --reach-area-ratio 0.08 --max-radius 1.0 --max-seconds 300 \
  --odom-topic /go2w/odom/fused \
  --record-video outputs/go2w_acceptance/sm_demo.mp4 \
  --output outputs/go2w_acceptance/sm_demo.jsonl

# 语义 reasoner 决策 → 实际转向闭环（2026-08-14 验证，操作者授权）
# 说明：目标未强确认时触发 SemanticNavigation reasoner 自主决策转向方向；
# --operator-authorized-rotation 是显式操作者授权，仅放宽"旋转需 lease"这一条门，
# 其余安全门（mode/error、lidar fresh、前向净空、运动边界、≤30°、单步、急停）全部保留。
# 必须在操作者确认场地安全 + 遥控器急停在手时使用。
/usr/bin/python3 scripts/go2w/run_autonomous_loop.py \
  --mode state_machine_search --target "绿色垃圾桶" --detector llm \
  --llm-model Qwen/Qwen3-VL-30B-A3B-Instruct \
  --semantic-reasoning --search-reasoner semantic_navigation --search-reasoner-mode active \
  --operator-authorized-rotation --turn-only --max-motion-steps 1 \
  --max-radius 1.5 --front-half-plane-only --min-clearance 0.6 \
  --target-score-min 0.99 --odom-topic /go2w/odom/wheel \
  --record-video outputs/go2w_acceptance/semantic_turn_execute.mp4 \
  --output outputs/go2w_acceptance/semantic_turn_execute.jsonl
```

每次运动都满足：短步（前进 2 s×0.12 m/s、转向 ≤30°）、轮式/融合里程计校验、
前向净空门禁、`mode/error` 检查、无位移自动重试/绕障、结束三次 STOP + disarm。
`--operator-authorized-rotation` 的详细说明见
`reports/go2w_semantic_navigation_pandarxt16_stage2_stage3_handoff_20260813.md` 2026-08-14 续接章节。

### 0.7 关键配置与代码索引

```text
configs/go2w/camera_intrinsics.yaml      # 相机内参（已标定）
configs/go2w/official_reference.yaml     # 官方几何 / base→LiDAR（厂商原始，不改）
configs/go2w/current_hardware_geometry.yaml  # 当前整机 0.70×0.43×0.70 m（操作者确认）
configs/go2w/current_hardware_state.yaml     # 硬件状态清单（绑定几何/外参/内置雷达配置）
configs/go2w/time_sync.yaml              # LiDAR/IMU 时间桥
configs/go2w/lidar_preprocess.yaml       # /scan、clearance、自过滤
configs/go2w/wheel_odom.yaml             # 轮式 + 融合里程计参数
configs/go2w/point_lio.yaml              # Point-LIO 桥接/门禁
configs/go2w/point_lio_unilidar_l2.yaml  # 官方 L2 基线（identity 外参）
configs/go2w/hesai_pandarxt16.yaml           # Pandar 驱动（diagnostic-only）
configs/go2w/hesai_pandarxt16_preprocess.yaml # Pandar 诊断预处理配置
configs/go2w/hesai_pandarxt16_extrinsics.yaml # Pandar 正式外参槽（未确认）
configs/go2w/dual_lidar_safety.yaml      # 双雷达安全融合策略（默认 disabled）
configs/go2w/navigation_gate.yaml        # fail-closed，未改动
scripts/go2w/run_autonomous_loop.py      # pattern/wander/camera_guided/
                                         # level_a_search/scan360_approach/
                                         # state_machine_search；--stage2-readiness
app/detectors/siliconflow_vision_worker.py   # LLM quick/verify worker
app/live_robot/step_planner.py               # 纯函数步进规划
app/live_robot/step_search_runner.py         # 状态机编排器
app/live_robot/current_hardware.py           # 当前硬件几何/状态加载与 hash
app/live_robot/pandar_clock.py               # Pandar 时钟分级（默认 host_receive_time_only）
app/live_robot/stage2_readiness.py           # Stage 2/3 machine-readable readiness
ros2_ws/src/go2w_lio_bringup/.../wheel_odom.py  # 融合里程计
ros2_ws/src/go2w_lidar_preprocessor/.../lidar_evidence.py          # 双雷达证据融合
ros2_ws/src/go2w_lidar_preprocessor/.../dual_lidar_observability.py # 旋转可观测性
ros2_ws/src/go2w_lidar_preprocessor/.../hesai_pandarxt16_preprocessor.py # Pandar 诊断节点
reports/go2w_codex_handoff_20260813.md      # Go2-W 基础链最新交接
reports/go2w_semantic_navigation_semantic_search_handoff_20260813.md  # 语义搜索融合交接
reports/semantic_navigation_v1_existing_implementation_audit_20260813.md  # SemanticNavigation V1 KEEP 审计
docs/PANDARXT16_DUAL_LIDAR_SAFETY_INTEGRATION.md  # 双雷达安全集成说明
```

### 0.8 证据位置（本机，未入库）

```text
outputs/go2w_acceptance/camera_calibration_20260806/
outputs/go2w_acceptance/time_bridge_live/
outputs/go2w_acceptance/lidar_preprocessor_live_corrected_tf/
outputs/go2w_acceptance/imu_turn_verify_20260807/          # 转向矩阵/搜索演示
outputs/go2w_acceptance/restart_verify_20260807/           # 相机修复后 LLM 搜索
outputs/go2w_acceptance/lio_calibration_20260807/          # LIO 直线试验
outputs/go2w_acceptance/fusion_validation_20260807/        # 融合里程计试验
```

### 0.9 常见故障与解决

- **lease 3207**：旧 `go2w_motion_action_server`/`hold_sport_lease` 残留 →
  kill 后重启运动栈；
- **相机断流**：新版相机桥损坏帧跳过 + 3 s 读超时 + 自动重连（退避 1→10 s）；
- **Bundle 陈旧**：>5 s 且重试 6 次仍不更新 → 自主循环拒绝动作并安全中止；
- **goal 被拒 “motion is not armed”**：LLM 检测可能超过 60 s arm 时限，脚本
  每次发动作前自动重新 arm；
- **LLM 慢/超时**：默认 `--llm-model Qwen/Qwen3-VL-30B-A3B-Instruct`（约 5–15 s）；
  8B 更慢；`SILICONFLOW_TIMEOUT_SECONDS` 在 `.env` 中调大；
- **椅子被当书包**：到达前会先 `--verify` 复核，`is_target=false` 则右转 15°
  继续观察；仍误判时建议加颜色/结构属性约束；
- **LIO 平移不可用**：属已知 BLOCKED，用 `/go2w/odom/fused`（轮式平移 +
  融合航向）；轮半径 0.089 m 为标称值，尚未标定。

### 0.10 安全硬约束

禁止 `/lowcmd`、`LowCmd`、`ReleaseMode()`、`Damp()`、固件修改、关闭安全保护；
Level D–F / Nav2 保持 fail-closed（`navigation_gate.yaml` 未改动）；任何运行
前确认场地与遥控器。

## 0. 推荐硬件与系统前提

推荐系统：

- Ubuntu 22.04 或 24.04 x86_64
- 至少 16 GB RAM
- 至少 20 GB 可用磁盘
- NVIDIA GPU 用于 GroundingDINO + SAM2，本项目已验证 RTX 4090 + CUDA PyTorch 可运行

Nav2 正式规划/执行固定支持 **Ubuntu 22.04 + ROS2 Humble**。Ubuntu 24.04
仍可运行感知、推理、Web UI 和 `offline_preview`，但不属于本项目的 Humble
Worker 验收平台。

如果没有 NVIDIA GPU：

- `mock`、`真实 API`、LLM runtime prior、观察记忆、Streamlit UI 可以跑。
- `GroundingDINO+SAM2` 可能无法跑通或速度极慢，不建议作为验收标准。

检查 GPU：

```bash
nvidia-smi
```

如果 `nvidia-smi` 不存在或报错，先安装 NVIDIA 驱动并重启。Ubuntu 常用方式：

```bash
sudo ubuntu-drivers devices
sudo ubuntu-drivers autoinstall
sudo reboot
```

重启后再次确认：

```bash
nvidia-smi
```

## 1. 安装系统基础依赖

```bash
sudo apt update
sudo apt install -y \
  git curl wget ca-certificates build-essential pkg-config \
  libgl1 libglib2.0-0 libsm6 libxext6 libxrender1 \
  tmux unzip aria2
```

确认：

```bash
git --version
curl --version
tmux -V
```

## 2. 安装 Miniconda

如果系统已经有 conda，可以跳过本节。

```bash
cd /tmp
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh
bash Miniconda3-latest-Linux-x86_64.sh
```

安装时建议允许初始化 shell。安装完成后重新打开终端，或执行：

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda --version
```

如果 conda 安装在 `/opt/conda`，则执行：

```bash
source /opt/conda/etc/profile.d/conda.sh
conda --version
```

## 3. 获取项目代码

选择一个工作目录，例如 `/root/gpufree-data` 或 `/home/$USER/workspace`。

```bash
mkdir -p /root/gpufree-data
cd /root/gpufree-data
git clone https://github.com/BROVVV/robot_scene_demo.git
cd robot_scene_demo
```

如果你已经有项目目录：

```bash
cd /root/gpufree-data/robot_scene_demo
```

确认结构：

```bash
ls
```

应看到：

```text
app  data  docs  examples  scripts  tests  run_demo.py  streamlit_app.py
```

## 4. 创建 Python 环境

```bash
conda create -n go2_robot_scene_demo python=3.11 -y
conda activate go2_robot_scene_demo
```

确认：

```bash
which python
python --version
```

应显示 Python 3.11，且路径在 `go2_robot_scene_demo` 环境内。

升级 pip 并安装项目基础依赖：

```bash
python -m pip install --upgrade pip
pip install -r requirements.txt
```

如果网络慢，可以换源：

```bash
pip install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple
```

## 5. 配置 `.env`

```bash
cp .env.example .env
chmod 600 .env
nano .env
```

基础配置建议：

```text
SILICONFLOW_API_KEY=
SILICONFLOW_BASE_URL=https://api.siliconflow.cn/v1
SILICONFLOW_REASONING_MODEL=deepseek-ai/DeepSeek-V4-Flash
SILICONFLOW_VISION_MODEL=Qwen/Qwen3-VL-8B-Instruct
SILICONFLOW_TIMEOUT_SECONDS=25
SILICONFLOW_MAX_TOKENS=2048
IMAGE_MAX_SIDE=1280
IMAGE_DETAIL=high
ENABLE_LOW_OBJECT_RETRY=true
MIN_OBJECTS_FOR_COMPLEX_SCENE=6
OUTPUT_DIR=outputs

DETECTION_BACKEND=llm
```

重要说明：

- `DETECTION_BACKEND=llm` 只需要硅基流动视觉 API，不需要本地 GPU 检测模型。
- `DETECTION_BACKEND=grounded_sam` 会调用本地 GroundingDINO + SAM2，并且默认先调用 LLM 生成 GroundingDINO 英文开放词表 prompt。
- 如果要跑 `grounded_sam` 主流程，建议同时配置 `SILICONFLOW_API_KEY`；否则 prompt expansion 会明确失败，避免 GroundingDINO 静默收到空 prompt。
- 如果只是调试本地 worker，可先不配置 API Key，直接使用第 8.6 节手写 `--text-prompt` 验证 GroundingDINO/SAM2 环境。

平台避障辅助动态运动视界建议配置：

```text
PLATFORM_OBSTACLE_AVOIDANCE_ASSUMED=true
ENABLE_DYNAMIC_MOTION_HORIZON=true
MOTION_HORIZON_PROFILE=platform_assisted_auto
MOTION_STRICT_SAFE_MAX_STEP_M=0.5
MOTION_PLATFORM_INDOOR_DEFAULT_STEP_M=1.2
MOTION_PLATFORM_INDOOR_MAX_STEP_M=2.0
MOTION_PLATFORM_OPEN_DEFAULT_STEP_M=3.0
MOTION_PLATFORM_OPEN_MAX_STEP_M=5.0
MOTION_ABSOLUTE_MAX_STEP_M=6.0
MOTION_TARGET_CONFIRM_MAX_STEP_M=0.8
MOTION_PLATFORM_FALLBACK_STEP_M=1.5
MOTION_DEFAULT_STOP_AND_REOBSERVE=true
MOTION_ENABLE_OBSERVE_WHILE_MOVING=false
MOTION_SHORTEN_ON_TARGET_CANDIDATE=true
MOTION_ALLOW_LLM_RECOMMENDED_HORIZON=true
MOTION_LLM_HORIZON_WEIGHT=0.6
```

无人工先验导航建议配置：

```text
STATIC_KNOWLEDGE_BASE_ENABLED=false
HANDWRITTEN_OBJECT_PRIORS_ENABLED=false
HANDWRITTEN_LOCATION_PRIORS_ENABLED=false
HANDWRITTEN_ROOM_PRIORS_ENABLED=false
STATIC_OBJECT_PROMPTS_ENABLED=false
ALLOW_HANDCRAFTED_SEARCH_RULES=false

LLM_COMMONSENSE_PRIOR_ENABLED=true
LLM_PRIOR_GENERATION_MODE=runtime
LLM_PRIOR_CAN_CONFIRM_TARGET=false
LLM_PRIOR_MAX_HYPOTHESES=8
LLM_PRIOR_MAX_DETECTOR_PROMPTS=12

EVIDENCE_GATING_ENABLED=true
TARGET_CONFIRMATION_REQUIRE_VISUAL_EVIDENCE=true
TARGET_CONFIRMATION_REQUIRE_BBOX=true
TARGET_CONFIRMATION_REQUIRE_CROP_VERIFY=true
TARGET_CONFIRMATION_MIN_SCORE=0.72

OBSERVATION_MEMORY_ENABLED=true
OBSERVATION_MEMORY_STORE_PATH=data/memory/observational_memory.jsonl
OBSERVATION_MEMORY_WRITE_VISUAL_ONLY=true
OBSERVATION_MEMORY_REQUIRE_PROVENANCE=true

PRIOR_USAGE_AUDIT_ENABLED=true
PRIOR_USAGE_REPORT_PATH=outputs/prior_usage_report.json
```

视频目标搜索与建图辅助建议配置：

```text
VIDEO_MODE_DEFAULT=target_search
VIDEO_ENABLE_SCENE_MAPPING_DEFAULT=false
VIDEO_ENABLE_NAVIGATION_TOPOLOGY_DEFAULT=false
VIDEO_USE_SCENE_MAP_FOR_SEARCH_DEFAULT=true
VIDEO_ALLOW_SCENE_MAP_ONLY_DEBUG=false
VIDEO_TARGET_SEARCH_REQUIRED_WHEN_TARGET_PRESENT=true
VIDEO_SCENE_MAPPING_REUSE_TARGET_FRAMES=true
VIDEO_SCENE_MAPPING_REUSE_OBJECT_TRACKS=true
VIDEO_ENABLE_SCENE_MEMORY=true
VIDEO_FULL_SCENE_MAP_ENABLED=false
VIDEO_ALWAYS_WRITE_MEMORY=true
VIDEO_ENABLE_VIDEO_PSG=true
VIDEO_TOPOLOGY_ANNOTATE_TARGET_SEARCH=true
VIDEO_TOPOLOGY_ADD_TARGET_CANDIDATE_NODES=true
VIDEO_TOPOLOGY_ADD_TARGET_SEARCH_SCORES=true
```

视频运行模式的主任务永远是 `target_search`。Web UI 不再提供“视频全场景建图”作为顶层运行模式；全场景建图、导航拓扑图和拓扑辅助排序都只是“视频目标搜索”内部的辅助功能。`scene_map_only` 只保留给 CLI 高级调试，不适合真实导航目标搜索。

Video-to-Navigation Planning 默认启用，但只生成安全的视觉预览规划，不会控制机器人：

```text
VIDEO_NAVIGATION_ENABLED=true
VIDEO_NAVIGATION_MODE=visual_preview
VIDEO_POSE_BACKEND=auto
VIDEO_POSE_ALLOW_RELATIVE=true
VIDEO_POSE_REQUIRE_METRIC_FOR_NAV2=true
VIDEO_NAVIGATION_AUTO_PLAN=true
VIDEO_NAVIGATION_AUTO_EXPLORATION=true
VIDEO_NAVIGATION_MAX_FRAMES=300
VIDEO_NAVIGATION_FRAME_SAMPLE_INTERVAL=5
VIDEO_NAVIGATION_TARGET_OBSERVATION_DISTANCE=1.5
VIDEO_NAVIGATION_ENABLE_FRONTIER_EXPLORATION=true
VIDEO_NAVIGATION_EXPLORATION_MAX_CANDIDATES=8
VIDEO_NAVIGATION_ALLOW_NAV2_FROM_METRIC_VIDEO=false
VISUAL_NAV_EXECUTION_ENABLED=false
```

普通 RGB MP4 会标记为 `scale_status=relative`，Web UI 显示 `Visual Preview / Relative`，路径长度使用相对单位，不会伪装成米制 Nav2 路径。只有 RGB-D、双目、视觉惯性或外部标定提供可靠尺度与 `map` 坐标变换后，才允许进入真实 Nav2 handoff。CLI 可通过 `--video-map-transform-json` 提供 `T_map_video_map`：

```json
{
  "T_map_video_map": {
    "x": 1.2,
    "y": -0.4,
    "yaw": 0.0,
    "source": "external_calibration"
  }
}
```

GroundingDINO Prompt Expansion 建议配置：

```text
GROUNDING_PROMPT_LLM_EXPANSION_ENABLED=true
GROUNDING_PROMPT_REQUIRE_NON_EMPTY=true
GROUNDING_PROMPT_FAIL_FAST_ON_EMPTY=true
GROUNDING_PROMPT_RETRY_ON_EMPTY=true
GROUNDING_PROMPT_MAX_RETRIES=1
GROUNDING_PROMPT_MAX_TERMS=24
GROUNDING_PROMPT_MIN_TERMS=3
GROUNDING_PROMPT_DEBUG_OUTPUT=outputs/grounding_prompt_plan.json
GROUNDING_PROMPT_RETRY_DEBUG_OUTPUT=outputs/grounding_prompt_retry_plan.json
```

这组配置是 GroundingDINO+SAM2 主流程的关键桥接层。系统不会把“卧室 / 房间 / 区域”这种抽象场景词直接作为唯一 DINO prompt，而是让 LLM 根据完整任务语义生成英文可见代理物体、结构锚点、门牌/入口/标识等开放词表，例如：

```text
bed . wardrobe . nightstand . curtain . window . door . doorway . room entrance .
```

门状态任务也不会只靠 DINO 判断“打开/关闭”，而是先检测 `door / doorway / door frame / door handle` 等可见对象，再交给后续 crop verify、视觉模型或状态判断模块确认。

如果你没有 API Key，又必须临时跑 `grounded_sam` 主流程，可以在 `.env` 中关闭 LLM prompt expansion：

```text
GROUNDING_PROMPT_LLM_EXPANSION_ENABLED=false
```

关闭后系统会回到 TargetProfile / dynamic terms 兼容路径；这只适合作为离线调试或兼容旧流程，不是推荐实验配置。

本项目现在的“常识”来自运行时 LLM 假设和机器人观察记忆，不来自开发者写死的物体-位置先验。LLM 假设只能用于排序搜索区域、生成动态检测词、建议下一视角；目标是否找到必须通过 bbox/crop/mask/frame 等视觉证据门控。`TARGET_CONFIRMATION_REQUIRE_CROP_VERIFY=true` 表示有视觉 API Key 时启用 crop 复核硬门控；如果当前环境没有可用 API Key，系统会自动降级为 bbox/frame/mask/分数门控，避免本地 GroundingDINO+SAM2 检测结果因无法调用 crop verifier 而全部被拒绝。

策略档位说明：

- `strict_safe`：严格安全模式，恢复最大 0.5m 单段距离。
- `platform_assisted_indoor`：室内平台避障辅助，通常 0.8m 到 2.0m。
- `platform_assisted_open_area`：开放区域平台避障辅助，通常 2.0m 到 5.0m。
- `platform_assisted_auto`：根据场景类型、任务阶段、目标候选状态自动选择。

安全要求：

- 不要把真实 API Key 写入 README。
- 不要提交 `.env`。
- 不要把 `.env` 发给别人。
- 如果 API Key 曾经暴露在聊天记录或日志里，建议到硅基流动后台轮换一次。

确认 `.env` 没被 Git 跟踪：

```bash
git status --short .env
```

正常应显示：

```text
?? .env
```

或无输出；只要不是准备提交的 tracked 文件即可。

## 6. 先跑基础验收

### 6.1 核心 smoke test

```bash
python -m py_compile \
  app/config.py \
  app/perception/grounding_prompt_planner.py \
  app/detectors/grounded_sam_subprocess.py \
  app/detectors/grounded_sam_worker.py \
  run_demo.py \
  streamlit_app.py

python -m unittest \
  tests.test_grounding_prompt_planner \
  tests.test_grounded_sam_prompt_integration \
  tests.test_grounded_sam_runtime
```

期望：

```text
OK
```

可选全量回归：

```bash
python -m unittest discover -s tests
```

当前开发分支上，全量回归可能出现两类非部署阻塞问题：

- Streamlit AppTest 在无浏览器/低资源 bare mode 下超时。
- `scripts/evaluate_task_examples.py` 中部分 legacy task type 期望仍按旧任务模板断言，而当前系统已切到 LLM-first 自然语言任务理解。
- 部分新测试使用 pytest 风格，例如视频参数归一化和建图辅助链路；基础 `requirements.txt` 不包含 pytest。如需运行这些测试，先执行 `pip install pytest`，再运行 `python -m pytest tests/test_run_video_demo_args.py tests/test_scene_map_as_auxiliary.py`。

如果目标是验证“空机器能部署并跑通主流程”，优先以本节 smoke test、mock 流程、Web UI 健康检查和实际 `run_demo.py` 输出为准。

### 6.2 mock 流程

mock 不需要图片、不需要 API Key、不需要 GPU。

```bash
python run_demo.py --mock \
  --enable-llm-prior \
  --enable-observation-memory \
  --enable-evidence-gating \
  --disable-handwritten-priors
```

成功后应生成：

```text
outputs/scene_result.json
outputs/object_table.csv
outputs/relation_table.csv
outputs/topology_graph.png
outputs/topology_graph.graphml
outputs/ros2_motion_plan.json
outputs/motion_horizon_decision.json
outputs/llm_generated_priors.json
outputs/dynamic_detector_prompts.json
outputs/grounding_prompt_plan.json
outputs/grounding_prompt_retry_plan.json
outputs/evidence_gating_report.json
outputs/observation_memory_updates.json
outputs/prior_usage_report.json
outputs/knowledge_aware_result.json
outputs/parsed_task.json
outputs/capability_gate_result.json
outputs/navigation_task.json
outputs/actionability_report.md
outputs/retrieved_knowledge.json
outputs/predictive_scene_graph.graphml
outputs/hypotheses.json
outputs/knowledge_updates.json
outputs/reasoning_report.md
```

兼容说明：旧参数 `--enable-knowledge` 仍可使用，但会提示 deprecated，并映射为 LLM runtime prior + observation memory + evidence gating；默认不再启用静态 KB 或手写位置先验。

### 6.3 任务样例回归

```bash
python scripts/evaluate_task_examples.py
```

理想输出 JSON 中包含：

```json
"passed": true
```

当前 LLM-first 任务理解改造后，部分样例仍使用旧版 `count_objects` / `navigate_to_location` 模板期望，可能输出 `passed=false`。只要失败项是 `legacy_task_type`，通常表示样例断言尚未同步新任务 schema，不代表部署失败。真正需要优先处理的是导入错误、配置错误、模型调用错误、输出文件缺失或主流程异常退出。

## 7. 跑硅基流动真实 API

准备一张图片，例如：

```bash
ls /root/gpufree-data/微信图片_20260617144106.jpg
```

运行：

```bash
python run_demo.py \
  --image "/root/gpufree-data/微信图片_20260617144106.jpg" \
  --target "巡查玄关区域，识别地面可通行区域和主要障碍物" \
  --detector llm \
  --enable-llm-prior \
  --enable-observation-memory \
  --enable-evidence-gating \
  --disable-handwritten-priors
```

成功后会生成基础输出、LLM runtime prior、证据门控、观察记忆、审计报告和 ROS2 dry-run 指令文件。

如果报 `API 请求失败`：

1. 检查 `.env` 里的 `SILICONFLOW_API_KEY`。
2. 检查网络是否能访问 `https://api.siliconflow.cn/v1`。
3. 检查模型名 `Qwen/Qwen3-VL-8B-Instruct` 是否仍可用。
4. 临时调大超时：

```text
SILICONFLOW_TIMEOUT_SECONDS=60
```

## 8. 安装 GroundingDINO + SAM2

本节用于跑本地开放词表检测器。推荐有 NVIDIA GPU。

### 8.0 检查 CUDA Toolkit

GroundingDINO 本地扩展通常需要 `nvcc` 编译器。先检查：

```bash
nvcc --version
```

如果没有 `nvcc`，但 `nvidia-smi` 正常，可以安装 CUDA Toolkit。Ubuntu 22.04 + CUDA 12.8 示例：

```bash
cd /tmp
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt update
sudo apt install -y cuda-toolkit-12-8
```

加入环境变量：

```bash
echo 'export CUDA_HOME=/usr/local/cuda-12.8' >> ~/.bashrc
echo 'export PATH=$CUDA_HOME/bin:$PATH' >> ~/.bashrc
source ~/.bashrc
nvcc --version
```

Ubuntu 24.04 时，把上面的 `ubuntu2204` 换成 `ubuntu2404`。如果你安装的是其他 CUDA 版本，把 `CUDA_HOME` 改成真实路径，例如 `/usr/local/cuda-12.1`。

### 8.1 安装 PyTorch GPU 版

进入项目环境：

```bash
conda activate go2_robot_scene_demo
cd /root/gpufree-data/robot_scene_demo
```

安装 CUDA 版 PyTorch。已验证 `cu128` 可用：

```bash
python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
```

如果你的服务器驱动较旧，不支持 CUDA 12.8，可改用 PyTorch 官方给出的其他 CUDA wheel，例如 `cu121`：

```bash
python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
```

验证：

```bash
python - <<'PY'
import torch
print("torch", torch.__version__)
print("cuda_available", torch.cuda.is_available())
print("cuda", torch.version.cuda)
print("gpu", torch.cuda.get_device_name(0) if torch.cuda.is_available() else None)
PY
```

`cuda_available` 应为 `True`。

### 8.2 下载 Grounded-SAM-2 源码

推荐放在项目同级目录：

```bash
cd /root/gpufree-data
git clone https://github.com/IDEA-Research/Grounded-SAM-2.git
cd Grounded-SAM-2
```

如果 `git clone` 很慢，可以先下载 zip 再解压：

```bash
cd /root/gpufree-data
wget -O Grounded-SAM-2.zip https://github.com/IDEA-Research/Grounded-SAM-2/archive/refs/heads/main.zip
unzip Grounded-SAM-2.zip
mv Grounded-SAM-2-main Grounded-SAM-2
cd Grounded-SAM-2
```

### 8.3 安装 Grounded-SAM-2 和 GroundingDINO 依赖

```bash
conda activate go2_robot_scene_demo
cd /root/gpufree-data/Grounded-SAM-2
```

安装 SAM2：

```bash
SAM2_BUILD_CUDA=1 SAM2_BUILD_ALLOW_ERRORS=1 \
python -m pip install --no-build-isolation -e .
```

安装 GroundingDINO 依赖：

```bash
python -m pip install \
  transformers==4.40.2 "tokenizers<0.20,>=0.19" \
  addict yapf timm opencv-python pycocotools "supervision>=0.22.0"
```

安装 GroundingDINO 本地包：

```bash
CUDA_HOME=/usr/local/cuda TORCH_CUDA_ARCH_LIST=8.9 \
python -m pip install --no-build-isolation -e /root/gpufree-data/Grounded-SAM-2/grounding_dino
```

说明：

- `TORCH_CUDA_ARCH_LIST=8.9` 适合 RTX 4090。
- 其他 GPU 可先不设置该变量，或按 GPU 架构调整。
- 如果没有 `/usr/local/cuda`，但 PyTorch CUDA 可用，可以先去掉 `CUDA_HOME=/usr/local/cuda` 重试。

验证导入：

```bash
PYTHONPATH=/root/gpufree-data/Grounded-SAM-2:/root/gpufree-data/Grounded-SAM-2/grounding_dino \
python - <<'PY'
import torch
import groundingdino
import groundingdino._C
from grounding_dino.groundingdino.util.inference import load_model, load_image, predict
from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor
print("cuda_available", torch.cuda.is_available())
print("groundingdino ok")
print("sam2 ok")
PY
```

### 8.4 下载模型权重

进入 Grounded-SAM-2 目录：

```bash
cd /root/gpufree-data/Grounded-SAM-2
mkdir -p checkpoints gdino_checkpoints
```

SAM2 tiny 权重：

```bash
wget -O checkpoints/sam2.1_hiera_tiny.pt \
  https://dl.fbaipublicfiles.com/segment_anything_2/092824/sam2.1_hiera_tiny.pt
```

GroundingDINO SwinT 权重：

```bash
wget -O gdino_checkpoints/groundingdino_swint_ogc.pth \
  https://huggingface.co/ShilongLiu/GroundingDINO/resolve/main/groundingdino_swint_ogc.pth
```

如果 `wget` 很慢，可使用 `aria2c`：

```bash
aria2c -x 16 -s 16 -o groundingdino_swint_ogc.pth \
  -d gdino_checkpoints \
  https://huggingface.co/ShilongLiu/GroundingDINO/resolve/main/groundingdino_swint_ogc.pth
```

确认文件存在且大小合理：

```bash
ls -lh checkpoints/sam2.1_hiera_tiny.pt
ls -lh gdino_checkpoints/groundingdino_swint_ogc.pth
```

参考大小：

```text
sam2.1_hiera_tiny.pt            149M 左右
groundingdino_swint_ogc.pth     662M 左右
```

### 8.5 配置项目使用 GroundingDINO+SAM2

回到项目目录：

```bash
cd /root/gpufree-data/robot_scene_demo
nano .env
```

设置或确认：

```text
DETECTION_BACKEND=grounded_sam
GROUNDED_SAM_ROOT=/root/gpufree-data/Grounded-SAM-2
GROUNDED_SAM_PYTHON=python
GROUNDED_SAM_PYTHONPATH=/root/gpufree-data/Grounded-SAM-2:/root/gpufree-data/Grounded-SAM-2/grounding_dino
GROUNDING_DINO_CONFIG=grounding_dino/groundingdino/config/GroundingDINO_SwinT_OGC.py
GROUNDING_DINO_CHECKPOINT=gdino_checkpoints/groundingdino_swint_ogc.pth
GROUNDING_DINO_BOX_THRESHOLD=0.25
GROUNDING_DINO_TEXT_THRESHOLD=0.20
GROUNDING_DINO_HIGH_RECALL_BOX_THRESHOLD=0.10
GROUNDING_DINO_HIGH_RECALL_TEXT_THRESHOLD=0.08
ENABLE_GDINO_HIGH_RECALL=true
ENABLE_SAM2=true
SAM2_CONFIG=configs/sam2.1/sam2.1_hiera_t.yaml
SAM2_CHECKPOINT=checkpoints/sam2.1_hiera_tiny.pt
MAX_DETECTED_OBJECTS=30
DETECTION_DEVICE=auto
DETECTOR_TIMEOUT_SECONDS=180

GROUNDING_PROMPT_LLM_EXPANSION_ENABLED=true
GROUNDING_PROMPT_REQUIRE_NON_EMPTY=true
GROUNDING_PROMPT_FAIL_FAST_ON_EMPTY=true
GROUNDING_PROMPT_RETRY_ON_EMPTY=true
GROUNDING_PROMPT_MAX_RETRIES=1
GROUNDING_PROMPT_MAX_TERMS=24
GROUNDING_PROMPT_MIN_TERMS=3
GROUNDING_PROMPT_DEBUG_OUTPUT=outputs/grounding_prompt_plan.json
GROUNDING_PROMPT_RETRY_DEBUG_OUTPUT=outputs/grounding_prompt_retry_plan.json
```

如果运行前已经 `conda activate go2_robot_scene_demo`，`GROUNDED_SAM_PYTHON=python` 会使用当前环境。也可以写成你机器上的真实 Python 路径；查询方式：

```bash
conda activate go2_robot_scene_demo
which python
```

如果输出是 `/opt/conda/envs/go2_robot_scene_demo/bin/python`，则 `.env` 里应写：

```text
GROUNDED_SAM_PYTHON=/opt/conda/envs/go2_robot_scene_demo/bin/python
```

### 8.6 直接验证 worker

```bash
cd /root/gpufree-data/robot_scene_demo
PYTHONPATH=/root/gpufree-data/Grounded-SAM-2:/root/gpufree-data/Grounded-SAM-2/grounding_dino \
python app/detectors/grounded_sam_worker.py \
  --image "/root/gpufree-data/微信图片_20260617144106.jpg" \
  --output /tmp/grounded_sam_worker_test.json \
  --root /root/gpufree-data/Grounded-SAM-2 \
  --text-prompt "phone. smartphone. screen-like object." \
  --grounding-config grounding_dino/groundingdino/config/GroundingDINO_SwinT_OGC.py \
  --grounding-checkpoint gdino_checkpoints/groundingdino_swint_ogc.pth \
  --box-threshold 0.25 \
  --text-threshold 0.20 \
  --sam2-config configs/sam2.1/sam2.1_hiera_t.yaml \
  --sam2-checkpoint checkpoints/sam2.1_hiera_tiny.pt \
  --max-objects 20 \
  --device auto
```

检查结果：

```bash
python - <<'PY'
import json
p="/tmp/grounded_sam_worker_test.json"
data=json.load(open(p, encoding="utf-8"))
objs=data.get("objects", [])
print("objects", len(objs))
print("with_sam2_mask", sum(o.get("mask_area_ratio") is not None for o in objs))
print(objs[:2])
PY
```

期望：

- `objects` 大于 0。
- `with_sam2_mask` 大于 0。如果等于 0，通常是 SAM2 config 或 checkpoint 路径错误。

上面的 `--text-prompt` 只是 worker 手动 debug 示例。项目主流程默认不会依赖固定室内物体 prompt 表；GroundingDINO 检测词优先由 `GroundingPromptPlanner` 根据自然语言任务解析结果、导航任务和 TargetProfile 动态生成，LLM runtime prior 只作为后续搜索假设和动态复核线索，不能代替视觉证据确认目标。
worker 现在会拒绝空 `--text-prompt`。如果看到：

```text
text_prompt is empty. GroundingDINO requires a non-empty open-vocabulary prompt.
```

说明应该先检查 prompt 生成链路，而不是继续调低检测阈值。

### 8.6.1 验证 LLM-first GroundingDINO prompt expansion

GroundingDINO 不是整图场景理解模型，不能指望它直接检测“卧室”“房间”“区域”这类抽象目标。项目主流程会先解析自然语言任务，再调用 `GroundingPromptPlanner` 生成英文开放词表。

推荐先用房间类任务验证 prompt plan 是否生成：

```bash
python run_demo.py \
  --image "/root/gpufree-data/微信图片_20260617144106.jpg" \
  --target "找到卧室" \
  --detector grounded_sam \
  --disable-crop-verify \
  --disable-handwritten-priors
```

即使当前图片里没有卧室，也应该生成：

```text
outputs/grounding_prompt_plan.json
outputs/detection_debug_report.md
```

检查 prompt：

```bash
python - <<'PY'
import json
p="outputs/grounding_prompt_plan.json"
data=json.load(open(p, encoding="utf-8"))
print("target_category:", data.get("target_category"))
print("strategy:", data.get("grounding_strategy"))
print("prompt_valid:", data.get("is_valid_for_grounding_dino"))
print("prompt:", data.get("grounding_prompt"))
print("warnings:", data.get("warnings"))
PY
```

期望：

- `prompt_valid` 为 `True`。
- `grounding_prompt` 非空。
- room / area / scene / floor / corridor 任务中，prompt 不应只有 `bedroom .`、`room .` 这类抽象词。
- 如果第一次 DINO 返回 0 candidate 且开启 retry，会额外生成 `outputs/grounding_prompt_retry_plan.json`。

### 8.7 跑项目 GroundingDINO+SAM2 主流程

```bash
cd /root/gpufree-data/robot_scene_demo
python run_demo.py \
  --image "/root/gpufree-data/微信图片_20260617144106.jpg" \
  --target "找到卧室" \
  --detector grounded_sam \
  --enable-llm-prior \
  --enable-observation-memory \
  --enable-evidence-gating \
  --disable-handwritten-priors
```

成功后应看到类似：

```text
场景摘要：本地 Grounding DINO/SAM2 检测到 ... 个物体，补全 ... 条空间关系。
已生成：
outputs/scene_result.json
outputs/object_table.csv
outputs/relation_table.csv
outputs/topology_graph.png
outputs/topology_graph.graphml
outputs/ros2_motion_plan.json
outputs/annotated_scene.png
outputs/grounding_prompt_plan.json
outputs/detection_debug_report.md
...
```

`detection_debug_report.md` 会记录：

- 原始任务、intent、目标类别
- prompt 来源和生成策略
- direct terms / proxy object terms / context anchor terms
- 最终 GroundingDINO prompt
- 0 candidate retry prompt
- 原始候选数、过滤候选数、候选融合结果

## 9. 启动 Streamlit Web UI

前台启动：

```bash
cd /root/gpufree-data/robot_scene_demo
conda activate go2_robot_scene_demo
bash scripts/start_web_ui.sh
```

默认地址：

```text
http://localhost:8501
```

如果端口被占用：

```bash
bash scripts/start_web_ui.sh 8502
```

后台启动：

```bash
tmux new-session -d -s robot_scene_demo_ui \
  'bash -lc "cd /root/gpufree-data/robot_scene_demo && conda run -n go2_robot_scene_demo streamlit run streamlit_app.py --server.address 0.0.0.0 --server.port 8501 --server.headless true"'
```

检查健康状态：

```bash
curl --noproxy '*' -fsS http://127.0.0.1:8501/_stcore/health
```

期望：

```text
ok
```

只验证 Nav2 面板和离线路径、不连接 ROS 时：

```bash
NAV2_ENABLED=true NAV2_MODE=offline_preview \
  bash scripts/start_web_ui.sh
```

页面中的 `offline_preview` 会显式标记为“非 Nav2 真实路径 / 不可执行”，不会在
ROS 不可用时把 `plan_only` 或 `execute` 静默降级为模拟路径。

查看 UI 日志：

```bash
tmux attach -t robot_scene_demo_ui
```

退出 tmux 查看但不停止服务：按 `Ctrl+b`，再按 `d`。

停止 UI：

```bash
tmux kill-session -t robot_scene_demo_ui
```

## 10. Web UI 使用说明

左侧配置：

- `运行模式`
  - `模拟数据`：不需要图片、不需要 API Key。
  - `真实 API`：上传图片，调用硅基流动视觉模型。
  - `GroundingDINO+SAM2`：上传图片，调用本地检测器。
  - `视频目标搜索（兼容）`：历史离线分析入口；不控制真机，也不是真机自主搜索主链。
- `自然语言任务`：直接输入开放式任务，例如 `帮我找到手机`、`找到张三，然后在安全距离处报告位置`。
- `场景图片`：真实 API 和 GroundingDINO+SAM2 模式需要上传。
- `视频目标搜索` 模式下的辅助功能：
  - `启用视频记忆`：记录稳定场景、负目标证据和长期视频空间记忆。
  - `启用视频 PSG`：根据真实观察生成可探索候选区域，不能单独确认目标。
  - `启用全场景建图辅助`：在目标搜索过程中额外构建场景图，不替代目标搜索。
  - `生成导航拓扑图`：只有启用建图辅助后可选，输出 place/passage/free_space/obstacle/PSG 拓扑。
  - `使用拓扑图辅助目标搜索排序`：只给候选区域和 next-best-view 排序，目标确认仍必须依赖视觉证据。
- `知识增强流程`：建议打开。
- `预测性场景图`：显示 PSG。
- `高精度复查`：只对真实 API 模式有意义。
- `运动视界设置`
  - `运动策略档位`：严格安全、平台避障室内、平台避障开放区域、平台避障自动。
  - `假设机械狗已有基础避障`：开启后允许高层规划输出更长移动段。
  - `启用自适应移动视界`：关闭后恢复严格安全单步裁剪。
  - `开放区域最大移动距离` / `室内最大移动距离`：运行时覆盖 `.env` 中的最大距离。

结果区：

- 场景摘要
- 目标判断
- 路线规划
- 物体表
- 关系表
- 拓扑图
- 标注图
- 任务解析
- ROS2 指令 JSON
- 原始 JSON
- 知识增强结果
- 知识增强页签中的 `运动视界决策`：显示策略档位、场景类型、任务阶段、LLM 推荐距离、规则裁剪后距离、最终导出距离和原因。

### 10.1 自然语言任务理解与安全门控

Web UI 不再要求用户选择“找可见目标 / 找不可见目标”等任务模板。系统会先把自然语言任务解析为 `ParsedTask`，再经过能力与安全门控生成 `NavigationTask`。

可执行范围只包括观察、搜索、巡查、导航到更好视角、接近目标附近、安全距离停止和反馈。拿取、开柜、翻找、破坏、推撞、攻击、殴打、伤害、强行接触人员等子任务会被拦截。

如果任务同时包含可执行和不可执行部分，例如：

```text
找到张三，然后把他打一顿
```

系统只保留定位/搜索/安全距离观察/停止反馈部分，并在 `outputs/actionability_report.md` 中说明被拦截的伤害行为。目标是否可见不会由用户输入决定，解析阶段固定为 `initial_visibility_state="unknown"`，后续只能由视觉检测和 evidence gating 判断为 `visual_candidate`、`visual_confirmed` 或当前视角未确认。

### 10.2 视频目标搜索（兼容入口）

Web UI 的“运行模式”中选择“视频目标搜索（兼容）”后，可以上传
`mp4/avi/mov/mkv` 视频，设置目标、检测器、关键帧采样 FPS 和最大分析帧数。

当前产品语义：

- 该入口仅用于历史离线分析；真机主入口是 `Go2-W 实时目标搜索`，没有独立视频导航主链。
- 只要提供目标，后端必须先执行目标搜索。
- `启用全场景建图辅助`、`生成导航拓扑图`、`使用拓扑图辅助目标搜索排序` 都是目标搜索内部的辅助开关。
- 看到客厅、沙发、电视柜等上下文，只能生成“可能搜索区域”和下一步观察建议，不能把目标升级为 `visual_confirmed`。
- `scene_map_only` 只用于 CLI 高级调试；带 `--target` 时会被防呆拒绝。
- 视频分析成功后会自动生成 Video-to-Navigation 规划；即使没有 ROS2、没有 Nav2 goal 或没有目标线索，也会降级显示 `Visual Preview` 或 `Exploration`，不会让导航区域空白。

也可以直接使用命令行：

```bash
python run_video_demo.py \
  --video "/path/to/robot_walk.mp4" \
  --target "手机" \
  --mode target_search \
  --detector mock \
  --sample-fps 1.0 \
  --max-frames 120 \
  --enable-video-memory \
  --enable-video-navigation \
  --video-navigation-mode visual_preview
```

RGB-D、双目或外部尺度已验证输入可以使用 metric preview；只有同时提供 `T_map_video_map` 时才会准备 Nav2 map-frame goal：

```bash
python run_video_demo.py \
  --video "/path/to/rgbd_walk.mp4" \
  --target "红色背包" \
  --detector llm \
  --enable-video-navigation \
  --video-navigation-mode metric_preview \
  --video-pose-backend metric \
  --depth-dir "/path/to/depth" \
  --video-map-transform-json "/path/to/T_map_video_map.json"
```

真实检测时把 `mock` 替换为 `llm` 或 `grounded_sam`。如需在目标搜索过程中启用场景建图和导航拓扑辅助：

```bash
python run_video_demo.py \
  --video "/path/to/robot_walk.mp4" \
  --target "电视" \
  --mode target_search \
  --detector llm \
  --sample-fps 2.0 \
  --max-frames 300 \
  --enable-video-memory \
  --enable-video-psg \
  --enable-scene-mapping \
  --enable-navigation-topology \
  --use-scene-map-for-search
```

兼容旧命令时，`--mode full_scene_map` 或 `--enable-full-scene-map` 如果同时带 `--target`，会自动归一化为 `target_search + --enable-scene-mapping + --enable-navigation-topology`。只有显式 `--scene-map-only` 且不提供 `--target` 时，才会只建图不搜索目标。

不启用建图辅助时，视频目标搜索主输出包括：

```text
outputs/video_target_profile.json
outputs/video_target_search.json
outputs/video_target_timeline.json
outputs/video_target_candidates.json
outputs/video_object_tracks.json
outputs/video_track_summary.json
outputs/video_crop_verify_results.json
outputs/video_tracking_debug_report.md
outputs/video_candidate_regions.json
outputs/video_navigation_trace.json
outputs/video_reasoning_report.md
outputs/video_llm_generated_priors.json
outputs/video_dynamic_detector_prompts.json
outputs/video_evidence_gating_report.json
outputs/video_observation_memory_updates.json
outputs/video_prior_usage_report.json
outputs/video_frames/
outputs/video_frames_annotated/
outputs/video_scene_results/
```

启用建图辅助后，会在上述目标搜索主输出之外额外生成：

```text
outputs/video_frame_observations.json
outputs/video_place_segments.json
outputs/video_all_objects.json
outputs/video_observed_scene_graph.json
outputs/video_observed_scene_graph.graphml
outputs/video_psg_layer.json
outputs/video_hybrid_scene_graph.json
outputs/video_hybrid_scene_graph.graphml
outputs/video_navigation_map.json
outputs/video_navigation_topology.json
outputs/video_navigation_topology.graphml
outputs/video_navigation_topology.png
outputs/video_navigation_topology_debug.md
outputs/video_topology_search_ranking.json
```

视频记忆和 PSG 相关输出包括：

```text
outputs/video_memory_graph.json
outputs/video_memory_graph.graphml
outputs/video_memory_updates.json
outputs/video_spatial_memory_snapshot.json
outputs/video_predictive_scene_graph.graphml
outputs/video_predictive_scene_graph.json
outputs/video_hypotheses.json
data/memory/video_spatial_memory.jsonl
```

视频模式采用“场景中心记忆 + 目标条件推理”。即使采样帧里没有目标或候选物，
系统仍会记录环境类型、稳定参照物、可通行区域和负目标证据，生成 PSG 搜索假设，
并把去重后的观察写入长期 JSONL 记忆库。重复运行同一视频时，长期记忆库会跳过
高度相似的条目，但本次运行的记忆更新和推理报告仍会生成。

视频模式默认处理过去录制的第一视角视频。没有 odom、SLAM 位姿、深度或地图时，
系统输出目标出现时间、画面位置、参照物、候选区域、环境记忆和回访建议，
但不生成真实可执行导航路线。

相关环境变量位于 `.env.example`，常用项包括：

```text
VIDEO_ENABLE_SCENE_MAPPING_DEFAULT=false
VIDEO_ENABLE_NAVIGATION_TOPOLOGY_DEFAULT=false
VIDEO_USE_SCENE_MAP_FOR_SEARCH_DEFAULT=true
VIDEO_ALLOW_SCENE_MAP_ONLY_DEBUG=false
VIDEO_ENABLE_SCENE_MEMORY=true
VIDEO_ALWAYS_WRITE_MEMORY=true
VIDEO_ENABLE_VIDEO_PSG=true
VIDEO_ENABLE_NEGATIVE_EVIDENCE=true
VIDEO_MEMORY_STORE_PATH=data/memory/video_spatial_memory.jsonl
VIDEO_ENABLE_MEMORY_RETRIEVAL=true
VIDEO_MEMORY_RETRIEVAL_TOP_K=10
```

视频目标支持自然语言开放词表描述，例如：

```text
请帮我找一台能打印 A3 纸的设备
找到红色把手的白色柜门
寻找靠近饮水机的蓝色垃圾桶
寻找挂在墙上的红色消防器材
```

每次视频运行会先生成 `outputs/video_target_profile.json`，其中包含核心实体、
中英文开放词表、属性、关系约束和上下文线索。LLM 模式直接按目标画像逐帧判断；
GroundingDINO 模式先动态生成检测提示词，对于带颜色、用途或关系约束的复杂目标，
再使用视觉 LLM 对候选帧进行语义复核和 bbox 对齐。目标画像解析失败时会降级为
原始目标文本，不会让整段视频直接失败。

## 11. ROS2 dry-run 指令数据

每次运行基础分析都会生成：

```text
outputs/ros2_motion_plan.json
outputs/motion_horizon_decision.json
```

`ros2_motion_plan.json` 是 ROS2 `/cmd_vel` 兼容数据。移动距离不再无条件固定为 0.5m，而是由 `Motion Horizon Planner` 根据场景、任务阶段、目标候选状态、LLM 建议和 `.env` 硬上限裁剪。核心字段：

```json
{
  "dry_run": true,
  "topic": "/cmd_vel",
  "message_type": "geometry_msgs/msg/Twist",
  "command_rate_hz": 10.0,
  "platform_obstacle_avoidance_assumed": true,
  "dynamic_motion_horizon_enabled": true,
  "motion_horizon_profile": "platform_assisted_auto",
  "motion_horizon_decision": {
    "motion_policy": "platform_assisted_open_area",
    "recommended_distance_m": 3.0,
    "max_allowed_distance_m": 5.0,
    "decision_reason_zh": "当前为开放区域搜索阶段，平台具备基础避障能力，允许较长移动段以提高搜索效率。"
  },
  "commands": [
    {
      "source_action": "move_forward",
      "distance_m": 3.0,
      "twist": {
        "linear": {"x": 0.25, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
      },
      "duration_sec": 12.0,
      "interruptible_by_platform": true,
      "platform_obstacle_avoidance_assumed": true,
      "requires_stop_after_motion": true,
      "observe_while_moving": false
    }
  ]
}
```

`motion_horizon_decision.json` 是同一份动态距离决策的独立调试输出，便于查看最终距离为什么被放宽或缩短。

预览指令，不发布 ROS2：

```bash
python scripts/publish_ros2_motion_plan.py outputs/ros2_motion_plan.json
```

示例输出：

```text
dry_run=True topic=/cmd_vel rate=10Hz
commands=2
cmd_001 step=1 action=move_forward duration=2s linear.x=0.25 angular.z=0
cmd_002 step=2 action=stop duration=1s linear.x=0 angular.z=0
```

后续在机器狗或 ROS2 主机上接收数据时，有两种方式。

### 11.1 方式 A：只把 JSON 交给 ROS2 节点

推荐实际工程中采用这种方式。流程：

1. `robot_scene_demo` 生成 `outputs/ros2_motion_plan.json`。
2. 你自己的 ROS2 节点读取这个 JSON。
3. 按 `commands` 顺序向 `/cmd_vel` 发布 `geometry_msgs/msg/Twist`。
4. 每条命令持续发布 `duration_sec` 秒。
5. 发布频率使用 `command_rate_hz`。
6. 结束后发布零速度 Twist。

### 11.2 方式 B：使用项目内置 publisher 脚本

在 Ubuntu 22.04 上安装 ROS2 Humble/Nav2。脚本会初始化 ROS2 APT 软件源并安装
本项目所需的 Nav2、Simple Commander、Collision Monitor、Velocity Smoother
和 colcon：

```bash
bash scripts/install_nav2_humble.sh
```

source ROS2 环境：

```bash
source /opt/ros/humble/setup.bash
```

正确文件名必须是 `setup.bash`，不能写成 `setup.bas`，也不能拼成
`/opt/ros/humble/setup.bashe/setup.bas`。

确认 Python 能导入 ROS2：

```bash
/usr/bin/python3 - <<'PY'
import rclpy
from geometry_msgs.msg import Twist
print("ros2 python ok")
PY
```

这只验证 ROS2 系统 Python。默认双 Python 部署中，Conda Python 不直接加载
Humble 的 `rclpy`，系统 Python 也不自动包含主项目的 Pydantic 依赖。因此旧
publisher 的真实发布只作为兼容接口；推荐的正式执行方式是后文的隔离式 Nav2
Worker。

先 dry-run 预览：

```bash
python scripts/publish_ros2_motion_plan.py outputs/ros2_motion_plan.json
```

只有在你已经准备了同时包含项目依赖与 Humble `rclpy` 的兼容 Python 环境时，
才能使用旧链路真实发布：

```bash
python scripts/publish_ros2_motion_plan.py \
  outputs/ros2_motion_plan.json \
  --execute \
  --allow-dry-run-plan \
  --force-legacy-while-nav2-inactive
```

如果你的机器狗不是监听 `/cmd_vel`，可以改 topic：

```bash
python scripts/publish_ros2_motion_plan.py \
  outputs/ros2_motion_plan.json \
  --execute \
  --allow-dry-run-plan \
  --force-legacy-while-nav2-inactive \
  --topic /your_robot/cmd_vel
```

安全要求：

- 活动的 Nav2 `execute` 任务存在时，旧 publisher 会拒绝发布，避免双重
  `/cmd_vel` 来源竞争；`--force-legacy-while-nav2-inactive` 不能绕过此互斥。
- 第一次必须架空机器狗或断开电机执行。
- 必须有急停。
- 必须确认机器狗底盘坐标系中 `linear.x > 0` 是前进。
- 必须确认 `angular.z > 0` 的旋转方向。
- 本项目估计距离来自单张图和规则，不等价于真实导航。
- 真机执行前应接入深度、避障、SLAM 或机器狗厂商 SDK 的安全策略。
- `platform_obstacle_avoidance_assumed=true` 只表示高层允许更长移动段，不表示本项目已经实现避障。
- `strict_safe` 模式仍可把单段移动恢复为 0.5m 上限。

## 12. 常用命令汇总

进入项目：

```bash
cd /root/gpufree-data/robot_scene_demo
conda activate go2_robot_scene_demo
```

测试：

```bash
python -m unittest discover -s tests
```

mock：

```bash
python run_demo.py --mock
python run_demo.py --mock \
  --enable-llm-prior \
  --enable-observation-memory \
  --enable-evidence-gating \
  --disable-handwritten-priors
python run_demo.py --mock \
  --enable-llm-prior \
  --enable-observation-memory \
  --enable-evidence-gating \
  --disable-handwritten-priors \
  --motion-profile platform_assisted_auto \
  --platform-obstacle-avoidance
```

真实 API：

```bash
python run_demo.py \
  --image "/path/to/image.jpg" \
  --target "找到手机" \
  --detector llm \
  --enable-llm-prior \
  --enable-observation-memory \
  --enable-evidence-gating \
  --disable-handwritten-priors \
  --motion-profile platform_assisted_auto \
  --platform-obstacle-avoidance \
  --max-open-step 5.0
```

GroundingDINO+SAM2：

```bash
python run_demo.py \
  --image "/path/to/image.jpg" \
  --target "找到卧室" \
  --detector grounded_sam \
  --enable-llm-prior \
  --enable-observation-memory \
  --enable-evidence-gating \
  --disable-handwritten-priors
```

Web UI：

```bash
bash scripts/start_web_ui.sh
```

ROS2 指令预览：

```bash
python scripts/publish_ros2_motion_plan.py outputs/ros2_motion_plan.json
```

旧静态知识库查询（仅调试兼容，不参与默认 target found 判断）：

```bash
python scripts/query_scene_kb.py --target "手机" --room_type office --location floor_5
```

任务样例：

```bash
python scripts/evaluate_task_examples.py
```

## 13. 输出文件说明

基础输出：

```text
outputs/parsed_task.json              自然语言任务结构化解析，initial_visibility_state 固定为 unknown
outputs/capability_gate_result.json   机器狗能力边界与安全门控结果
outputs/navigation_task.json          可进入感知/导航管线的导航任务
outputs/actionability_report.md       已执行/已拦截子任务的人类可读报告
outputs/scene_result.json              场景结构化结果
outputs/object_table.csv               物体表
outputs/relation_table.csv             关系表
outputs/topology_graph.png             拓扑图图片
outputs/topology_graph.graphml         拓扑图 GraphML
outputs/annotated_scene.png            标注图，有原图时生成
outputs/ros2_motion_plan.json          ROS2 /cmd_vel dry-run 指令数据
outputs/motion_horizon_decision.json   自适应移动视界决策
```

LLM runtime prior / evidence gate 输出：

```text
outputs/llm_generated_priors.json        LLM 运行时自生成常识搜索假设，不能确认目标
outputs/dynamic_detector_prompts.json    用户目标 + LLM prior + 视觉摘要生成的动态检测词
outputs/grounding_prompt_plan.json       GroundingDINO prompt expansion 计划，记录策略、词表和最终 prompt
outputs/grounding_prompt_retry_plan.json 0 candidate 时的高召回 retry prompt，只有触发 retry 时生成
outputs/detection_debug_report.md        Grounded-SAM/crop/fusion 调试报告，含最终 GroundingDINO prompt
outputs/evidence_gating_report.json      目标状态与视觉证据门控结果
outputs/observation_memory_updates.json  本次观察记忆写入记录
outputs/prior_usage_report.json          本次是否使用静态/手写先验的审计报告
```

目标状态说明：

```text
llm_hypothesis_only：只有 LLM 常识假设，目标未确认。
visual_candidate：有视觉候选，但还未通过门控。
visual_confirmed：视觉证据通过门控，目标确认。
user_confirmed：用户确认。
```

视频目标搜索主输出：

```text
outputs/video_target_profile.json        视频目标画像与开放词表
outputs/video_target_search.json         视频目标搜索主结果，目标状态以视觉证据为准
outputs/video_target_timeline.json       目标候选/状态时间线
outputs/video_target_candidates.json     目标候选摘要
outputs/video_object_tracks.json         视频目标/物体 track 结果
outputs/video_track_summary.json         track-level 投票摘要
outputs/video_crop_verify_results.json   候选 crop 复核结果
outputs/video_tracking_debug_report.md   视频 tracking 调试报告
outputs/video_candidate_regions.json     未确认时的候选搜索区域
outputs/video_navigation_trace.json      下一步观察/导航建议轨迹
outputs/video_reasoning_report.md        视频目标搜索推理报告
```

视频记忆、运行时 prior 与 evidence gate 输出：

```text
outputs/video_memory_graph.json
outputs/video_memory_graph.graphml
outputs/video_memory_updates.json
outputs/video_spatial_memory_snapshot.json
outputs/video_predictive_scene_graph.json
outputs/video_predictive_scene_graph.graphml
outputs/video_hypotheses.json
outputs/video_llm_generated_priors.json
outputs/video_dynamic_detector_prompts.json
outputs/video_evidence_gating_report.json
outputs/video_observation_memory_updates.json
outputs/video_prior_usage_report.json
data/memory/video_spatial_memory.jsonl
```

视频目标搜索内的建图辅助输出。只有启用 `--enable-scene-mapping` 或 Web UI 的“启用全场景建图辅助”后才要求生成：

```text
outputs/video_frame_observations.json
outputs/video_place_segments.json
outputs/video_all_objects.json
outputs/video_observed_scene_graph.json
outputs/video_observed_scene_graph.graphml
outputs/video_psg_layer.json
outputs/video_hybrid_scene_graph.json
outputs/video_hybrid_scene_graph.graphml
outputs/video_navigation_map.json
outputs/video_navigation_topology.json
outputs/video_navigation_topology.graphml
outputs/video_navigation_topology.png
outputs/video_navigation_topology_debug.md
outputs/video_topology_search_ranking.json
```

视频目标状态补充：

```text
target_not_seen：没有任何目标候选。
target_candidate：检测到疑似目标，但证据不足。
target_visual_confirmed：目标被 bbox / crop verify / track voting / evidence gating 视觉确认。
target_lost_after_seen：之前看到过目标，后续帧丢失。
target_unconfirmed_but_likely_area_found：未看到目标，但拓扑或上下文发现可能搜索区域。
```

兼容输出：

```text
outputs/knowledge_aware_result.json
outputs/parsed_task.json
outputs/retrieved_knowledge.json
outputs/predictive_scene_graph.graphml
outputs/hypotheses.json
outputs/knowledge_updates.json
outputs/reasoning_report.md
outputs/quadruped_search_plan.json
outputs/quadruped_ros2_motion_plan.json
outputs/llm_search_hypotheses.json
outputs/actionability_report.md
outputs/visual_grounding_report.json
```

## 14. Git 与隐私注意事项

不要提交：

- `.env`
- API Key
- `outputs/`
- `__pycache__/`
- conda 环境目录
- 大模型权重
- 私人图片
- ROS2 机器狗真实地址、token、证书

检查：

```bash
git status --short
git diff -- . ':!outputs'
```

如果 remote URL 里含 token，立刻改掉：

```bash
git remote set-url origin https://github.com/<用户名>/<仓库名>.git
```

## 15. 故障排查

### 15.1 `ModuleNotFoundError: No module named app`

确认在项目根目录执行：

```bash
cd /root/gpufree-data/robot_scene_demo
python run_demo.py --mock
```

### 15.2 `cuda_available False`

检查：

```bash
nvidia-smi
python - <<'PY'
import torch
print(torch.__version__)
print(torch.cuda.is_available())
print(torch.version.cuda)
PY
```

处理：

- 安装或修复 NVIDIA 驱动。
- 安装匹配的 PyTorch CUDA wheel。
- 确认没有装成 CPU-only PyTorch。

### 15.3 GroundingDINO 报 `BertModel.get_head_mask`

固定 transformers 版本：

```bash
pip install "transformers==4.40.2" "tokenizers<0.20,>=0.19"
```

### 15.4 SAM2 没有 mask

重点检查 `.env`：

```text
SAM2_CONFIG=configs/sam2.1/sam2.1_hiera_t.yaml
SAM2_CHECKPOINT=checkpoints/sam2.1_hiera_tiny.pt
```

不要写成：

```text
SAM2_CONFIG=sam2/configs/sam2.1/sam2.1_hiera_t.yaml
```

### 15.5 GroundingDINO prompt 为空

如果报：

```text
GroundingDINO prompt is empty
```

或 worker 报：

```text
text_prompt is empty. GroundingDINO requires a non-empty open-vocabulary prompt.
```

处理顺序：

1. 检查 `.env` 是否启用：

```text
GROUNDING_PROMPT_LLM_EXPANSION_ENABLED=true
GROUNDING_PROMPT_REQUIRE_NON_EMPTY=true
```

2. 检查是否配置了 `SILICONFLOW_API_KEY`。Grounded-SAM 主流程默认需要 LLM 生成开放词表。

3. 查看：

```bash
ls -lh outputs/grounding_prompt_plan.json outputs/detection_debug_report.md
cat outputs/detection_debug_report.md
```

4. 如果 `outputs/grounding_prompt_plan.json` 不存在，说明 prompt expansion 调用没有成功，优先检查 API Key、网络、模型名和超时。

5. 如果只是离线调试本地检测器，可以临时关闭：

```text
GROUNDING_PROMPT_LLM_EXPANSION_ENABLED=false
```

然后用第 8.6 节 worker 命令手动传入非空 `--text-prompt`。

### 15.6 GroundingDINO 0 candidate

0 candidate 不一定是 SAM2 问题。排查顺序：

1. 打开 `outputs/detection_debug_report.md`，看 `Final GroundingDINO Prompt` 是否是具体英文可见词。
2. 如果目标是房间/区域/场景，prompt 不应只有 `bedroom .`、`room .`、`area .`。
3. 查看是否触发 retry：

```bash
cat outputs/grounding_prompt_retry_plan.json
```

4. 临时降低阈值：

```text
GROUNDING_DINO_BOX_THRESHOLD=0.10
GROUNDING_DINO_TEXT_THRESHOLD=0.08
ENABLE_GDINO_HIGH_RECALL=true
```

5. 用第 8.6 节 worker 手动传一个确定能检测的词，例如 `person . chair . table . door .`，验证本地 DINO/SAM2 环境本身是否正常。

### 15.7 Streamlit 端口占用

换端口：

```bash
bash scripts/start_web_ui.sh 8502
```

或查占用：

```bash
ss -ltnp | grep 8501
```

### 15.8 ROS2/Nav2 设置脚本或 `rclpy` 不存在

先检查正确文件：

```bash
test -f /opt/ros/humble/setup.bash && echo "ROS2 setup ok"
```

若不存在，在 Ubuntu 22.04 上运行：

```bash
bash scripts/install_nav2_humble.sh
```

然后使用系统 Python 验证。不要用 Conda Python 直接导入 Humble 的 `rclpy`：

```bash
source /opt/ros/humble/setup.bash
/usr/bin/python3 - <<'PY'
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
print("ok")
PY
```

如果曾把 `NAV2_SETUP_BASH` 错写成
`/opt/ros/humble/setup.bashe/setup.bas`，当前网关会在标准安装存在时修复到
`/opt/ros/humble/setup.bash`；仍建议同时修正 shell 或 `.env` 中的原始值。

Conda 环境中构建 ROS 工作区必须固定系统 Python：

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3
```

### 15.9 API 超时

调大：

```text
SILICONFLOW_TIMEOUT_SECONDS=60
SILICONFLOW_MAX_TOKENS=2048
```

或先用：

```bash
python run_demo.py --mock --enable-knowledge
```

确认本地流程没有问题。

## 16. 最小验收清单

在全新 Ubuntu 上，至少完成以下命令才算部署成功：

```bash
python -m py_compile app/config.py app/perception/grounding_prompt_planner.py app/detectors/grounded_sam_subprocess.py run_demo.py streamlit_app.py
python -m unittest tests.test_grounding_prompt_planner tests.test_grounded_sam_prompt_integration tests.test_grounded_sam_runtime
python -m unittest discover -s tests -p 'test_nav2_*.py'
python run_demo.py --mock --enable-llm-prior --enable-observation-memory --enable-evidence-gating --disable-handwritten-priors
python run_demo.py --mock --enable-nav2 --nav2-mode offline_preview --nav2-goal-x 2 --nav2-goal-y 1 --nav2-wait
python scripts/publish_ros2_motion_plan.py outputs/ros2_motion_plan.json
bash scripts/start_web_ui.sh
```

可选回归检查：

```bash
python -m unittest discover -s tests
python scripts/evaluate_task_examples.py
pip install pytest
python -m pytest tests/test_run_video_demo_args.py tests/test_scene_map_as_auxiliary.py
```

如果可选回归只失败在 Streamlit AppTest 超时或 legacy task type 断言，可先按第 6.1 和第 6.3 节说明判断，不把它作为部署失败。

如果配置了真实 API：

```bash
python run_demo.py --image "/path/to/image.jpg" --target "找到手机" --detector llm --enable-knowledge
```

如果配置了 GroundingDINO+SAM2：

```bash
python run_demo.py --image "/path/to/image.jpg" --target "找到卧室" --detector grounded_sam --enable-llm-prior --enable-observation-memory --enable-evidence-gating --disable-handwritten-priors
```

最终应能访问：

```text
http://localhost:8501
```

并能看到或下载：

```text
outputs/scene_result.json
outputs/knowledge_aware_result.json
outputs/ros2_motion_plan.json
outputs/motion_horizon_decision.json
outputs/grounding_prompt_plan.json
```
## 高精度目标识别模式

项目现在支持“目标画像与动态开放词表 → 高召回候选检测 → 候选 crop
视觉复核 → 多源分数融合 → 视频 track-level 投票”。没有 API Key 时会跳过
crop 复核；mock 仍可独立运行。ROS2 输出仍默认 `dry_run=true`。

推荐图片命令：

```bash
python run_demo.py \
  --image "/path/to/image.jpg" \
  --target "找到手机" \
  --detector grounded_sam \
  --high-recall \
  --enable-crop-verify \
  --enable-knowledge
```

推荐视频命令：

```bash
python run_video_demo.py \
  --video "/path/to/robot_walk.mp4" \
  --target "手机" \
  --mode target_search \
  --detector grounded_sam \
  --sample-fps 3.0 \
  --max-frames 300 \
  --enable-tracking \
  --enable-crop-verify \
  --enable-knowledge \
  --enable-video-memory \
  --enable-video-navigation \
  --video-navigation-mode visual_preview
```

如果需要在高精度视频搜索中同时生成场景拓扑辅助结果，在上述命令后追加：

```bash
  --enable-video-psg \
  --enable-scene-mapping \
  --enable-navigation-topology \
  --use-scene-map-for-search
```

新增的主要调试输出包括：

```text
outputs/grounding_prompt_plan.json
outputs/grounding_prompt_retry_plan.json
outputs/target_profile.json
outputs/candidate_objects.json
outputs/crop_verify_results.json
outputs/fused_objects.json
outputs/detection_debug_report.md
outputs/video_track_summary.json
outputs/video_crop_verify_results.json
outputs/video_tracking_debug_report.md
outputs/video_navigation/<request_id>/visual_navigation_plan.json
outputs/video_navigation/<request_id>/navigation_instructions.json
outputs/video_navigation/<request_id>/webui_manifest.json
```

如果普通 RGB 视频中没有找到目标，系统会自动生成探索式导航规划，并写入 frontier 候选点；如果找到目标或疑似目标，则生成目标/候选观察位姿，而不是把 bbox 中心直接当作机器人 goal。

内置示例标注可用于检查评估链路：

```bash
python scripts/evaluate_detection_accuracy.py
python scripts/evaluate_video_target_search.py
```
# 无人工先验的大模型自生成常识推理

本项目不是“完全无常识”，而是“无开发者预置常识库”：系统不默认读取 object-location、room-object、目标搜索规则等手写先验；LLM 可以在运行时根据目标、当前画面、可见物体、空间关系和观察记忆生成搜索假设。

这些假设的 `prior_source` 是 `llm_runtime_commonsense`，`can_confirm_target=false`。它们只能用于候选区域排序、动态检测词生成、下一视角建议和解释搜索策略。目标确认必须通过 `evidence_gating_report.json` 中的视觉门控。

推荐单图命令：

```bash
python run_demo.py \
  --image "/path/to/image.jpg" \
  --target "找到手机" \
  --detector grounded_sam \
  --enable-llm-prior \
  --enable-observation-memory \
  --enable-evidence-gating \
  --disable-handwritten-priors
```

推荐视频命令：

```bash
python run_video_demo.py \
  --video "/path/to/robot_walk.mp4" \
  --target "手机" \
  --mode target_search \
  --detector grounded_sam \
  --sample-fps 3.0 \
  --max-frames 300 \
  --enable-llm-prior \
  --enable-tracking \
  --enable-crop-verify \
  --enable-observation-memory \
  --enable-evidence-gating \
  --disable-handwritten-priors \
  --enable-video-navigation \
  --video-navigation-mode visual_preview
```

# LLM-first 机械狗情境搜索

单图知识增强模式现在支持“视觉事实 → LLM 情境推理 → 视觉证据门控 →
机械狗动作门控 → PSG v2 → 下一视角计划”。推断节点不会把目标误标记为已找到，
超出机械狗能力的开柜、翻找、拿取和低头精查会被改写或标记为需要人工。

快速验收：

```bash
python run_demo.py --mock --enable-knowledge --enable-llm-reasoning --quadruped-mode
```

新增输出包括：

- `outputs/llm_search_hypotheses.json`
- `outputs/quadruped_search_plan.json`
- `outputs/reasoned_predictive_scene_graph.json`
- `outputs/reasoned_predictive_scene_graph.graphml`
- `outputs/actionability_report.md`
- `outputs/quadruped_ros2_motion_plan.json`
- `outputs/reasoned_annotated_scene.png`（提供原图时）
- `outputs/visual_grounding_report.json`

LLM API 不可用时流程不会中断，会明确标记推理不可用，并降级为基于当前视觉锚点
的保守转向/重观测方案。

当目标尚未视觉确认时，LLM 生成的 `suggested_detector_prompts_en` 会触发一次
可选二次视觉复核。只有复核结果具有 bbox/mask/crop 等视觉证据时，目标才会从
`inferred` 升级为 `observed`。GroundingDINO+SAM2 在无 CUDA 环境下可能超时，
系统会返回可读的 `DetectorRuntimeError`，不会输出伪造检测结果。

# 平台避障辅助动态运动视界

单图与知识增强模式现在支持“平台避障辅助 + 高层语义自适应移动距离”。旧逻辑中 ROS2 单次前进/后退会被固定裁剪到 0.5m；现在系统会根据场景类型、任务阶段、目标候选状态、LLM 推荐距离和配置硬上限动态计算高层移动段长度。

核心原则：

- 本项目不实现避障、局部路径规划、代价地图、深度图避障或急停。
- `platform_obstacle_avoidance_assumed=true` 表示真实安全由机械狗底层平台/SDK/ROS 安全层负责。
- 开放区域搜索可以生成 2m 以上移动段。
- 普通室内搜索可以生成 0.8m 到 2m 左右移动段。
- 目标候选出现、目标确认阶段或信息不足时会自动缩短到 0.3m 到 0.8m。
- LLM 不可用时，如果平台避障假设开启，会降级到保守的 1.0m 到 1.5m；如果平台避障假设关闭，则回到严格 0.5m。
- 运动后仍保留 stop 命令，不取消段后停稳和重观测。

快速验收：

```bash
python run_demo.py --mock --enable-knowledge \
  --motion-profile platform_assisted_open_area \
  --platform-obstacle-avoidance \
  --max-open-step 5.0
```

注意：mock 样例中的目标已经可见，因此会进入目标确认阶段并主动缩短距离。这是预期行为。若要验证开放区域长距离，可使用目标未出现、场景开阔的图片或测试：

```bash
python -m unittest tests.test_motion_horizon tests.test_ros2_motion_dynamic_horizon
```

典型 `outputs/motion_horizon_decision.json`：

```json
{
  "enabled": true,
  "profile": "platform_assisted_auto",
  "platform_obstacle_avoidance_assumed": true,
  "scene_type": "open_area",
  "task_phase": "search",
  "motion_policy": "platform_assisted_open_area",
  "recommended_distance_m": 3.0,
  "max_allowed_distance_m": 5.0,
  "requires_stop_after_motion": true,
  "observe_while_moving": false,
  "source": "mixed",
  "decision_reason_zh": "当前为开放区域搜索阶段，平台具备基础避障能力，允许较长移动段以提高搜索效率。"
}
```

如果需要恢复旧版保守行为：

```bash
python run_demo.py --mock --enable-knowledge \
  --motion-profile strict_safe \
  --disable-dynamic-motion-horizon
```

# Navigation2（ROS2 Humble）

### 架构与模式

项目采用主程序与 ROS Worker 隔离的架构：

```text
Streamlit / run_demo.py（Conda Python）
        ↓ 原子 JSON/JSONL + subprocess
nav2_bridge_worker.py（/usr/bin/python3 + ROS2 Humble）
        ↓ ComputePathToPose / NavigateToPose
Navigation2
```

现有感知、LLM-first 推理、证据门控、视频记忆、PSG、拓扑候选和 Motion Horizon
继续负责“去哪里观察”；Nav2 负责 map 坐标下的真实规划与执行。单图像素和视频帧
坐标不会被伪造成 map pose，自动目标必须包含可验证的坐标来源 `provenance`。

| 模式 | ROS/Nav2 | 是否执行 | 说明 |
|---|---:|---:|---|
| `disabled` | 不需要 | 否 | 默认值，保持旧流程 |
| `offline_preview` | 不需要 | 否 | 固定 fixture，只验证接口和 UI |
| `plan_only` | 需要 | 否 | 调用真实 `ComputePathToPose` |
| `execute` | 需要 | 是 | 先规划，再调用 `NavigateToPose` |

`plan_only` 和 `execute` 失败时不会自动降级为离线路径。

### 安装与构建

```bash
cd /root/gpufree-data/robot_scene_demo
bash scripts/install_nav2_humble.sh

source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
cd ..
```

Humble 的环境脚本固定是：

```text
/opt/ros/humble/setup.bash
```

不要写成 `setup.bas` 或 `/opt/ros/humble/setup.bashe/setup.bas`。项目会在标准安装
确实存在时自动纠正这类可识别拼写错误；自定义 ROS 安装则必须提供一个完整文件：

```text
NAV2_SETUP_BASH=/custom/ros/humble/setup.bash
NAV2_SYSTEM_PYTHON=/usr/bin/python3
NAV2_WORKSPACE_SETUP=/absolute/path/to/ros2_ws/install/setup.bash
```

### 启动 Nav2 和健康检查

先启动机器人底层、里程计、LaserScan、定位与 `map → base_link` TF，再加载地图。
纯规划使用不含 controller 或 `cmd_vel` 节点的 launch：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 launch robot_scene_nav_bringup robot_scene_nav2_plan_only.launch.py \
  map:=/absolute/path/to/map.yaml
```

完整 launch 默认 `execution_enabled:=false`，此时不会启动执行栈。只有 21 项门禁
通过后，外层受控启动器才可显式设置 true；速度链固定为
`controller_server → velocity_smoother → collision_monitor →
/go2w/nav2_cmd_vel → arbiter → cmd_vel_bridge`。footprint 采用固定来源的 Unitree
厂家站立外廓，但现场间隙和动态姿态尚未验证时，执行门禁仍不会开放。

另一个终端执行：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
/usr/bin/python3 scripts/check_nav2_runtime.py \
  --json outputs/nav2_health.json
```

健康检查会验证 ROS distro、系统 Python 导入、两个 Action Server、地图、TF、
里程计、`/cmd_vel` 与 Collision Monitor。外部 Nav2 图未启动时会在有限时间内
返回阻塞项，不会无限等待。Worker 本身也会按照
`NAV2_PLANNING_TIMEOUT_SECONDS` 返回 `NAV2_ACTION_SERVER_UNAVAILABLE` 或
`NAV2_PLANNING_TIMEOUT`。

### 离线预览与真实规划

无 ROS 的离线接口/UI 验收：

```bash
python run_demo.py --mock --enable-nav2 --nav2-mode offline_preview \
  --nav2-goal-x 2 --nav2-goal-y 1 --nav2-wait

NAV2_ENABLED=true NAV2_MODE=offline_preview \
  bash scripts/start_web_ui.sh
```

真实 Nav2 只规划：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python run_demo.py --mock --enable-nav2 --nav2-mode plan_only \
  --nav2-goal-x 1 --nav2-goal-y 0 --nav2-goal-yaw 0 \
  --nav2-use-current-start --nav2-wait
```

也可以使用快捷脚本：

```bash
bash scripts/run_nav2_plan_only.sh 1.0 0.0 0.0
```

### 执行模式与安全门控

`execute` 必须通过 21 项实时能力门控和操作员二次确认。推荐只使用统一入口：

```bash
bash scripts/go2w/start_search_session.sh \
  --target "红色背包" --mode nav2_execute
```

仓库中的 footprint 使用 Unitree 厂家站立外廓，Collision Monitor 区域保守包络
该外廓；两者仍不代表现场动态间隙已经验收。直接构造旧式四复选框请求会因缺少
`capability_gate_result.json` 被请求模型和 Worker 双重拒绝。

### 输出与验收

每个任务保存在：

```text
outputs/nav2/jobs/<request_id>/
```

其中包括请求、状态、全局路径 JSON/CSV、路径图、指令预览、执行反馈、
`cmd_vel` 轨迹、Worker 日志和导航报告；`outputs/nav2_*` 是最近任务快捷副本。
所有 JSON 使用 UTF-8 原子写入，Web UI 不会读到半写文件。

Nav2 单元测试：

```bash
python -m unittest discover -s tests -p 'test_nav2_*.py'
```

更完整的部署、安全说明和 Web UI 验收步骤见：

- [`docs/NAV2_INTEGRATION.md`](docs/NAV2_INTEGRATION.md)
- [`docs/NAV2_WEBUI_TESTING.md`](docs/NAV2_WEBUI_TESTING.md)

## Go2-W SemanticNavigation-style 语义搜索推理

`state_machine_search` 的“目标未出现 / SELECT_NEXT_VIEW”分支现已支持一个可回滚、
可审计的 SemanticNavigation 风格语义搜索层。它只融合 Goal Graph、现有 Observed Scene Graph
匹配、zero/partial/strong search policy 和 session negative memory；不是完整
SemanticNavigation runtime，也不替代检测器、运动控制或 Nav2。

安全语义保持不变：graph strong match 不能确认目标；目标仍必须经过现有视觉证据
和 LLM verify。V1 只记录 observation pose、heading、bbox 和 provenance，不依赖
尚未验收的 RGB–LiDAR 外参，不生成 3D target/map goal。Reasoner 只能提供高层
`SearchDirective`，实际动作仍经过状态机、`step_planner`、clearance、odom/radius、
mode/error、motion Action、STOP/disarm；Nav2 gate 继续 fail-closed。

真机状态机与 observe-only 现在使用同一套配置化 graph-match 阈值，并可只读检索现有
`ObservationMemoryStore`。检索到的长期 observation 作为每次
`SearchReasoningContext` 的输入，runner 事件只记录数量与 `memory_id`；本 session 的
negative scan memory 仍只存在内存中，`persistent_write_attempted=false`，不会把失败
扫描写回长期 JSONL。该路径已兼容 ROS 2 Humble 的系统 Python 3.10。

`LIVE_SEARCH_REASONER_MIN_REPLAN_SECONDS` 也已进入真实 runner：只有 semantic observer
明确复用同一稳定帧、机器人位姿未变且仍在最小间隔内时，才复用上次 directive；平移
超过 0.05 m、heading sector/帧/错误状态变化都会重新推理。`hybrid` 还可以复用现有
Video PSG 与已经由上游产生的 situated prior，但它们只在 `zero_match` 且候选方向在
真实观察、visited/negative-memory 成本上同分时做末级 tie-break。PSG 必须由 observed
node 支撑；LLM prior 必须声明 `can_confirm_target=false`。两者均被改写为 turn-only、
`allow_forward=false` 的提示，不能确认目标，也不会在真机循环中额外触发网络调用。

默认完全保持旧行为：

```text
LIVE_SEARCH_SEMANTIC_REASONING_ENABLED=false
LIVE_SEARCH_REASONER_BACKEND=legacy
LIVE_SEARCH_REASONER_MODE=shadow
LIVE_SEARCH_REASONER_ALLOW_FORWARD=false
```

推荐先运行 shadow；此时会记录 hybrid/SemanticNavigation 建议，但实际执行仍严格使用 legacy
scan step：

```bash
/usr/bin/python3 scripts/go2w/run_autonomous_loop.py \
  --mode state_machine_search \
  --target "饮水机旁边的蓝色垃圾桶" \
  --detector llm \
  --semantic-reasoning \
  --search-reasoner hybrid \
  --search-reasoner-mode shadow \
  --semantic-no-forward \
  --max-radius 1.5 \
  --front-half-plane-only --turn-only \
  --max-motion-steps 1 --min-rotation-clearance 0.511 \
  --odom-topic /go2w/odom/fused \
  --output outputs/live_sessions/semantic_navigation_shadow.jsonl
```

shadow 回放验证后才可将模式改为 `active`；V1 active 默认仅选择单步转向和重观测。
狭窄现场的 `--turn-only` 是最终执行器硬门，legacy/active 均不能绕过。当前 L2
近场与 0.511 m 全机身转向包络重叠，`rotation_clearance_valid=false`，左右 clearance
发布为 `NaN`（unknown）；即使数值无回波或未设置最小距离，转向也会 fail-closed。
Runner 还显式订阅 `/go2w/safety/lidar_fresh`：启动前和每一步都要求 freshness=true，
旧 clearance、缺失值或 NaN 均不能通过；前向 `inf` 只在同一时刻 freshness=true 时
表示当前没有碰撞高度内回波。
Frame Bundle 用 `front/left/right_status` 区分 `measured`、`no_return` 和 `unknown`，
同时携带 `rotation_clearance_valid`，不会再把前方无回波与左右近场未知都压成同一个
JSON `null`。即使未来启用 leased `/cmd_vel` bridge，非零 yaw 也必须先通过旋转有效性门。

2026-08-13 现场复核还识别出旧自滤遗漏的左前/左后轮低位回波：80 帧 A/B 中，持续
0.39–0.51 m 的轮簇被两个局部实测掩码消除，修正后 80 帧只有 1 个单帧孤点落入
0.511 m 转向包络。正前碰撞走廊 80/80 无回波；左前约 0.61–0.62 m 的剩余低位簇与
相机画面左缘白色办公椅轮/支腿方向一致，因此保留为环境回波；右前最近约 0.96 m。
这些 raw sector 数值只用于诊断，左右正式状态仍是 `unknown`，没有据此开放转向。
证据为 `outputs/go2w_acceptance/lidar_wheel_self_filter_live_20260813/result.json`。

旋转门现还有配置级防误解锁：永久的
`rotation_clearance_validation.valid=true` 只有在同时记录
360° 物理+LiDAR 交叉验证方法、操作者/时间/站姿、至少 0.511 m 的验收包络、证据路径，
并明确通过水平 yaw、近场盲区缓解、自滤物理复核、全扇区检测和站姿五项检查时才可
加载。每个证据路径还必须指向可读 JSON，且其机器人型号、操作者、站姿、包络、五项
检查和 `robot_motion_commanded=false` 必须与配置一致。只改一个布尔值或填写不存在的
路径都会使预处理器关闭安全门；人工检查生成的短时、位置绑定证据不能写入这个永久门，
永久门还要求新增覆盖传感器或重新安装并物理复核后的完整 sensor-only 近场可观测性。
最新几何审计显示，当前 720/720 个
方向均含不可观测自由空间：左右轴向盲段 0.155 m，前后 0.04 m，局部最坏约
0.291 m。因此 `rotation_clearance_valid=false` 有直接几何依据，而非把“无回波”误当
障碍。证据为
`outputs/go2w_acceptance/lidar_rotation_observability_20260813/result.json`。

当前现场可使用只读四向工具生成一次性 Stage 2 证据。先在没有标定物时采集 baseline，
再由人把同一个低矮、LiDAR 可见的宽目标依次放到前/右/后/左；机器狗全程不动：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
PY=/usr/bin/python3
OUT=outputs/go2w_acceptance/rotation_physical_crosscheck_20260813
$PY scripts/go2w/validate_rotation_clearance_physical_ros.py capture \
  --role baseline --output $OUT/baseline.json
# 根据 baseline 的 recommended_low_occupancy_distances_m 选择每个方向距离：
$PY scripts/go2w/validate_rotation_clearance_physical_ros.py capture \
  --role front --baseline $OUT/baseline.json \
  --expected-distance-m 0.60 --output $OUT/front.json
# right/rear/left 同理，分别写入 right.json、rear.json、left.json。
$PY scripts/go2w/validate_rotation_clearance_physical_ros.py finalize \
  --baseline $OUT/baseline.json --front $OUT/front.json \
  --right $OUT/right.json --rear $OUT/rear.json --left $OUT/left.json \
  --operator "现场操作者姓名" --physical-clearance-radius-m 0.60 \
  --swept-clearance-confirmed --standing-posture-confirmed \
  --output $OUT/rotation_lease.json
# 仅在上述命令 PASS 且许可尚未过期时执行一次 Stage 2：
$PY scripts/go2w/run_autonomous_loop.py \
  --mode state_machine_search --target "手机" \
  --semantic-reasoning --search-reasoner hybrid \
  --search-reasoner-mode active --semantic-no-forward \
  --turn-only --front-half-plane-only --max-radius 1.5 \
  --max-motion-steps 1 --min-rotation-clearance 0.511 \
  --odom-topic /go2w/odom/wheel \
  --rotation-clearance-evidence $OUT/rotation_lease.json \
  --output outputs/live_sessions/semantic_navigation_stage2_active_turn.jsonl
```

`finalize` 还要求操作者实测完整 0.511 m 圆形扫掠范围（包括后方和四轮外侧）为空。
每个非 baseline capture 会立即做 before/after 对照；目标没有稳定出现在指定方向/距离，
或点数没有相对现场背景明显增加时，该次命令直接失败，不必等到 finalize 才发现。
产物最长有效 15 分钟、绑定 `odom_wheel` 初始位置且允许平移不超过 3 cm；它只能通过
Runner 的 `--rotation-clearance-evidence` 用于原地转向。实时原始侧向话题
`/go2w/diagnostics/lidar_clearance_raw` 本身没有安全权威性，只有该许可有效且数据年龄
不超过 0.3 s 时才被 Runner 采用。任何前进都会使这份原位许可失效。
传入短时许可时，Runner 还会强制上述 Stage 2 参数组合，并在执行端拒绝非转向、超过
30° 或多步命令；不能拿该 JSON 运行 pattern、wander 或 short-forward。

真实传感器 shadow 的 fail-closed 验收也已完成：semantic 与 legacy 均提出 `r30`，但
`rotation_clearance_valid=false` 在 arm 前拒绝该步，成功运动 0 步、起止 odom 完全
一致，且没有启动运动控制图。状态机现在只在某一步通过全部运动门后才即时 arm；明确
的 quick target-absent 结果可复用为不含伪造物体/关系的 zero-match 观察，避免重复
full-scene API。证据为
`outputs/go2w_acceptance/semantic_navigation_shadow_fail_closed_20260813/result.json`。这不是成功转向
PASS；active turn-only 与 short-forward 仍保持关闭。

回滚只需去掉 `--semantic-reasoning` 或使用 `--search-reasoner legacy`。完整架构、
输出、离线 A/B 和分级真机验收见
[`docs/SEMANTIC_NAVIGATION_SEMANTIC_SEARCH_INTEGRATION.md`](docs/SEMANTIC_NAVIGATION_SEMANTIC_SEARCH_INTEGRATION.md)。

## Go2-W 内置 RGB + LiDAR 真机部署

当前部署默认 fail-closed；小范围运动只在操作者明确授权
（`GO2W_MOTION_READY`）并通过 `scripts/go2w/run_autonomous_loop.py` 时执行。
已经实机通过的是内置 RGB 的只读采集、LiDAR/IMU 时间对齐、原子帧 Bundle，
以及带轮式里程计校验的小范围前进/转向/自主搜索。Go2-W 尺寸与内置 LiDAR/IMU 静态 TF 已采用
Unitree 官方产品页、固定提交的官方 URDF 和官方 LiDAR SDK，并通过隔离 ROS 域
`/tf_static` 验证。LiDAR 现场方向、地面/自身过滤、碰撞高度带 `/scan` 和 300 ms
stale 门禁已完成静止只读实机验收；全高度 `/go2w/lidar/obstacles` 保留给语义，安全
扫描使用 `/go2w/lidar/collision_obstacles`。旋转包络近场仍未验收，因此不授权转向。
官方 Unitree Point-LIO 固定版本已在隔离 Noetic
环境中通过 5 分钟静止只读验收；CameraInfo 已用实测 9×6/15 mm 棋盘完成标定，
相机外参、RGB-LiDAR 外参、移动里程计试验、地图和 Nav2 尚未完成物理验收。因此当前能力是
Level A/B 部分能力（观察、扫描、搜索、靠近、每步 STOP），Level C--F 仍阻断，不能把软件模块存在解释为
实机能力通过。

固定厂家参考位于 `configs/go2w/official_reference.yaml`：站立外廓为
`0.70 × 0.43 × 0.50 m`，`base_link -> utlidar_lidar` 为
`xyz=(0.28945, 0, -0.046825) m, rpy=(0, 2.8782, 0) rad`，
`utlidar_lidar -> utlidar_imu` 为纯平移
`(-0.007698, -0.014655, 0.00667) m`。该文件明确不授权运动；相机位姿和
`base_link` 离地高度没有在官方 Go2-W URDF 中公开，仍保持未知。

传感器与派生话题：

```text
/camera/front/image_raw
/camera/front/camera_info
/utlidar/cloud
/utlidar/imu
/go2w/lio_input/cloud_raw
/go2w/lio_input/imu_raw
/go2w/lidar/scan                 # 静止只读实机验收已通过
/lio/odom                        # Point-LIO 静止只读验收通过；默认不常驻启动
```

安装、构建和只读启动：

```bash
bash scripts/go2w/install_dependencies.sh
bash scripts/go2w/build_ros2.sh
bash scripts/go2w/start_live_perception.sh
```

启动器会先检查 `enp6s0` 的物理 carrier 和 `192.168.123.0/24` 主机地址；缺失时
直接 fail closed。Bundle 默认按 1 Hz 输出并只保留当前会话最近 30 个，避免长时间
运行无界写盘。计划要求的 10 分钟静止传输验收可用：

```bash
bash scripts/go2w/run_level_a_acceptance.sh
```

首轮 600 秒试验在约第 423 个 Bundle 后丢失有线 carrier，因此没有伪报 PASS；该
历史失败已被下述修复后通过结果取代。完整相机内参和 RGB-LiDAR 外参流程见
`docs/GO2W_REAL_ROBOT_DEPLOYMENT.md`。

恢复载波并修复长测器的跨会话统计与限频漂移后，最终 603.24 秒复验通过全部传输
门禁：489 帧、0.816 Hz、末帧 0.354 秒、30 帧/8.49 MiB、载波连续、内存稳定且
清理无残留。CameraInfo 已为 true；总 Level A 仍只因
`camera_tf_not_validated` 保持 false。证据位于
`outputs/go2w_acceptance/level_a_stationary_soak_fixed/result.json`。

当前内参来自 105 组 1920×1080 实拍，实时复验通过 10 组非零 K 的同步
Image/CompressedImage/CameraInfo；独立棋盘帧的平均/RMS/最大重投影误差为
0.859/1.024/3.375 px。证据位于
`outputs/go2w_acceptance/camera_calibration_20260806/`。相机 TF 和相机—雷达外参
仍保持未知，不能因 CameraInfo 通过而开启三维融合或导航。

RGB-LiDAR ROS 节点已接入只读启动器。当前平面靶候选只通过静止诊断叠加门，明确没有
通过导航级几何门：`/perception/rgb_lidar_overlay_ready=true`，同时
`/perception/fusion_ready=false`、
`/perception/rgb_lidar_extrinsics_validated=false`。连续 5 组真机状态与 3 个 metric 3D
输出话题零消息的验收通过，诊断同时给出 `authorizes_3d_output=false` 和
`authorizes_motion=false`。证据位于
`outputs/go2w_acceptance/rgb_lidar_geometry_tier_20260813/result.json`。完成真实标定、
移动位置复核并显式晋级配置后，节点才会生成相机相对三维位置；它不会用实验候选或
网络图片猜测导航几何。

复现官方 TF/静止地面方向与 LiDAR 预处理验收：

```bash
/usr/bin/python3 scripts/go2w/audit_stationary_lidar_geometry.py \
  --reference-file configs/go2w/official_reference.yaml \
  --output outputs/go2w_acceptance/lidar_stationary_geometry/result.json
bash scripts/go2w/test_lidar_preprocessor_live.sh
bash scripts/go2w/setup_point_lio_noetic.sh
bash scripts/go2w/test_point_lio_stationary_live.sh
```

Point-LIO 使用官方 `point_lio_unilidar` 提交
`18ed5976d8fab2bd8a5148c26a40692bd3c0dc91`。最终 300 秒静止证据包含 4,615
帧里程计，频率 15.385 Hz，最大消息间隔 68.6 ms，最终/最大漂移
0.0785/0.0934 m，航向跨度 1.624°，桥丢包为 0；输入停止后 0.164 s 进入
stale 且不重发旧位姿。证据位于
`outputs/go2w_acceptance/point_lio_stationary/result_5min.json`。RKO-LIO 两组静止
A/B 试验均产生明显假运动，已在 `configs/go2w/lio.yaml` 中禁用。

该通过仅覆盖静止处理，不授权移动。直线、矩形、原地旋转、建图与 Nav2 验收均
未执行；在当前“狗不能移动”的约束下继续保持阻断。

生产入口固定使用只读 VideoHub RPC，最近联合验收得到 166 个 1920×1080 Bundle；
内容门禁还会拒绝已观察到的 H.264/DDS 纯绿色损坏帧。`/frontvideostream` 只保留为
显式诊断入口，不再由生产启动器自动选择。最新 Bundle 健康状态为
`camera=true, lidar=true, camera_info_calibrated=false, lio=false, tf=false`。

另一个终端可请求静止观察；当前传感器门禁关闭时会明确返回阻断项：

```bash
bash scripts/go2w/start_search_session.sh --target "手机" --mode observe_only
```

### 自主小范围运行与检测后端

真机自主运行使用 `scripts/go2w/run_autonomous_loop.py`，默认调用**硅基流动视觉
大模型 API**（`--detector llm`）做目标识别与场景理解，不再默认依赖
GroundingDINO+SAM2（可通过 `--detector grounded_sam` 显式回退）。LLM 快速检测
只请求“目标在不在 + 一个紧贴 bbox”，实测单次约 5–15 秒；完整场景理解（物体、
关系、路线建议）由同一 API 的非 quick 模式提供。默认视觉模型为
`Qwen/Qwen3-VL-30B-A3B-Instruct`，可用 `--llm-model
Qwen/Qwen3-VL-8B-Instruct` 切回更高细节模型。

```bash
# 360° 扫描 → 选最佳命中方向 → 靠近（半径限制 1.0 m，录像）
/usr/bin/python3 scripts/go2w/run_autonomous_loop.py \
  --mode scan360_approach --target "黑色书包" --detector llm \
  --max-radius 1.0 --max-seconds 420 \
  --record-video outputs/live_sessions/scan360_llm.mp4 \
  --output outputs/live_sessions/scan360_llm.jsonl

# 摆动扫描 → 发现 → 对齐 → 靠近
/usr/bin/python3 scripts/go2w/run_autonomous_loop.py \
  --mode level_a_search --target "手机" --detector llm \
  --max-radius 1.0 --max-seconds 300 \
  --output outputs/live_sessions/level_a_llm.jsonl
```

自主循环每步都校验轮式里程计、前向净空与 `mode/error`，无位移自动重试/绕障，
结束自动三次 STOP 并解除 arm。Bundle 超过 3 秒未更新时会拒绝继续动作并安全
中止（相机偶发抖动时最多等待重试 6 次，确认断流才中止，防止拿旧图盲动）。
每次发动作前会重新 arm，避免 LLM 检测耗时超过动作服务器 arm 时限后 goal 被拒；
任何未处理异常也会先急停再 disarm。`run_live_robot_demo.py` 的默认检测后端也
已改为 `llm`。

相机桥（`go2w_camera_bridge`）内置 RPC 逐帧容错与自动重连：损坏帧只跳过不
退出，子进程卡死超过 3 秒自动重启（退避 1→10 秒），避免相机流永久冻结。

LLM 快速检测/复核已接入 `app/live_robot` 正式搜索状态机
（`search_state_machine.py` + `step_planner.py` + `step_search_runner.py`），
自主脚本可用 `--mode state_machine_search` 直接驱动该链路：

```bash
/usr/bin/python3 scripts/go2w/run_autonomous_loop.py \
  --mode state_machine_search --target "灰色书包" --detector llm \
  --reach-area-ratio 0.08 --max-radius 1.0 --max-seconds 300 \
  --record-video outputs/live_sessions/sm_search.mp4 \
  --output outputs/live_sessions/sm_search.jsonl
```

### wheel+LIO 融合里程计

`go2w_wheel_odom` 同时发布：

- `/go2w/odom/wheel`：纯轮式编码器 + Sport yaw（原语义）；
- `/go2w/odom/fused`：轮式平移沿 Sport+LIO 融合航向积分（推荐小范围里程计）。

融合航向 = Sport yaw delta + `lio_yaw_weight` ×（LIO yaw delta − Sport yaw
delta），默认权重 0.35。LIO 航向只有在新鲜（按主机接收时刻，≤0.5 s）、位置
范数 ≤5 m、数值有限且与 Sport 逐 tick 一致时才参与；任何违规自动回退 Sport，
连续 3 次不一致只告警一次。诊断见 `/go2w/odom/fused/status`。Point-LIO 平移
仍 BLOCKED，不参与融合；`navigation_gate.yaml` 保持 fail-closed。

自主运行脚本支持 `--odom-topic` 选择位移校验来源：

```bash
/usr/bin/python3 scripts/go2w/run_autonomous_loop.py \
  --mode scan360_approach --target "灰色书包" --detector llm \
  --odom-topic /go2w/odom/fused \
  --max-radius 1.0 --max-seconds 420 \
  --output outputs/live_sessions/search_fused.jsonl
```

`--record-video` 输出使用 Noto Sans CJK 渲染中文标签（不再出现 `????`），并在
视频左下角实时叠加当前运动指令（如“左转 30°”“前进 0.12 m/s × 2s”）、LLM
检测状态和复核结论。到达判定（bbox 面积占比 ≥ `--reach-area-ratio`）前会再调用
一次 `--verify` 复核，模型确认框内物体属于目标才写 `target_reached`；复核拒绝
时记录 `target_verification` 事件并右转 15° 继续观察，避免把椅子等相似物体
当成目标。360° 扫描遇到高分命中（score ≥ 0.80）会提前停止扫描、直接靠近，
不再多转一圈后绕回来。

短步搜索、Nav2 只规划和 Nav2 执行使用同一入口，但不会静默降级：

```bash
bash scripts/go2w/start_search_session.sh --target "手机" --mode step_search
bash scripts/go2w/start_search_session.sh --target "红色背包" --mode nav2_plan_only
bash scripts/go2w/start_search_session.sh --target "红色背包" --mode nav2_execute
```

`nav2_plan_only` 需要 Level D、厂家 footprint 的现场间隙复核、有效 `/scan`、
LIO、地图、TF 和 ComputePathToPose 全部通过。`nav2_execute` 还需要 Collision Monitor、Velocity
Smoother、lease、仲裁器、300 ms watchdog、急停、遥控器接管检测、零错误状态、
操作员 arm 和二次确认。当前结果保存在：

```text
outputs/go2w_acceptance/navigation_gate/plan_only.json
outputs/go2w_acceptance/navigation_gate/execute.json
```

执行速度链固定为：

```text
controller_server
→ velocity_smoother
→ collision_monitor
→ /go2w/nav2_cmd_vel
→ go2w_control_arbiter
→ go2w_cmd_vel_bridge
→ /go2w/motion（唯一 lease holder）
```

控制器上限为 `0.15 m/s` 和 `0.20 rad/s`，桥接器会再次限速/限加速度。完整
执行 launch 默认 `execution_enabled:=false`，此时不启动 Nav2 执行节点；纯规划
另有 `robot_scene_nav2_plan_only.launch.py`，其中不包含 controller、smoother、
collision monitor 或任何 `cmd_vel` 发布节点。

Streamlit 选择“Go2-W 实时目标搜索”可查看相机、LiDAR、LIO、TF、lease、控制源、
搜索状态、证据和全部导航门禁。门禁失败时按钮禁用并显示阻断项。

停止主机侧 Worker 并保留日志：

```bash
bash scripts/go2w/stop_all.sh
```

本次部署从未取得运动 lease，所以该脚本只停止项目自己的主机进程并写入取消/
执行禁用标记，不会向机器人发送 `StopMove`，也不声称能停止外部进程管理的
Sport lease。标定文件位于 `configs/go2w/`，会话
输出位于 `outputs/live_sessions/`，ROS 日志位于 `runtime/go2w/sessions/`。详细分级
状态和物理待办见 `reports/go2w_robot_scene_demo_deployment_report.md`。

## VLM-only 低延迟具身语义导航（2026-08-28 改造）

本仓库已按 `robot_scene_demo_VLM_only低延迟语义导航_一次性AI改造计划书_20260828.md` 完成 VLM-only 低延迟改造。

### 核心变化

- Quick VLM 只负责目标候选，`objects` 兼容字段只包含 `target_objects`；普通场景物体由后台 Full Semantic 负责。
- 主循环使用 `target_decision.is_present + target_objects + score` 作为目标 gate，不再把普通物体误判为目标候选。
- 同一 frame 的 Verify VLM 最多调用一次；跨 fresh frame 可再次验证。
- Full Semantic VLM 移出常规运动 critical path：首次 warm-up 可同步，之后由 `AsyncSemanticObservationManager` 后台异步更新世界模型。
- 新增 `latency_profile` JSONL 事件，记录 quick/semantic/verify/blocking/cycle 等延迟。
- 新增长驻 VLM daemon（Unix socket + JSON line），realtime lane 与 background lane 分离；daemon 不可用时自动回退 subprocess。
- 保留 GroundingDINO/SAM2 代码仅作 baseline，不进入生产 VLM-only runtime。

### 关键新增/修改文件

- `app/detectors/siliconflow_vision_worker.py`：Quick 数据契约、Verify context+crop、token 配置化。
- `app/llm_clients/siliconflow_client.py`：Full Semantic 只做 scene graph，不再要求 route_plan/task_understanding。
- `app/live_robot/async_semantic_observer.py`：异步语义管理器（coalescing、stale 保护、capture pose）。
- `app/live_robot/latency_profiler.py`：延迟采集。
- `app/live_robot/verify_cache.py`：同帧 Verify 去重。
- `app/detectors/siliconflow_vision_daemon.py` / `siliconflow_vision_protocol.py`：VLM 长驻 daemon。
- `scripts/go2w/run_semantic_exploration.py`：目标 gate、force 修正、异步语义接入、latency 事件、异步 common-sense prior。
- `scripts/go2w/run_autonomous_loop.py`：daemon 优先、subprocess fallback。
- `configs/exploration/default.yaml`：`verify_attempts: 1`。

### 真机启动

```bash
bash scripts/go2w/start_semantic_exploration.sh \
  --target "饮水机旁边的蓝色垃圾桶" \
  --max-planning-cycles 5 \
  --max-motion-steps 5
```

如需禁用 VLM daemon：

```bash
NO_VLM_DAEMON=1 bash scripts/go2w/start_semantic_exploration.sh --target "..."
```

### 相关测试

```bash
.venv/bin/python -m pytest tests/test_vlm_quick_contract.py \
  tests/test_verify_fresh_frame_policy.py \
  tests/test_async_semantic_observer.py \
  tests/test_vlm_daemon_protocol.py \
  tests/test_vlm_latency_events.py \
  tests/test_vlm_only_runtime.py -q
```
