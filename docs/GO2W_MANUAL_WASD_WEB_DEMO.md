# Go2-W Manual WASD+QE Web Demo

独立、简洁、可单独启动的小型 Web Demo：浏览器实时显示 Go2-W 内置相机，用户通过
**WASD+QE** 控制机器狗小范围活动，页面右侧每隔几秒异步调用项目已经配置好的
SiliconFlow 视觉模型列出当前画面主要物体并自动刷新。

> 目标（计划书 2026-08-14）：不接 SemanticNavigation / Nav2 / 3D / 目标搜索状态机，
> 不重复已有相机与运动底层，不允许 LLM 阻塞控制，不允许浏览器断连后继续移动。

---

## 1. 功能

| 能力 | 说明 |
|---|---|
| 实时相机 | 左侧 MJPEG 画面（约 8~12 FPS），左下角叠 FPS / 按键 / 当前命令 |
| WASD+QE 控制 | W 前进、S 后退、A 左移、D 右移、Q 左转、E 右转；Space 停止、Esc 急停 |
| 连续控制 | **长按 = 连续运动**：按住期间保持一个长 timed-velocity goal（到 `hold_duration_sec` 自动续约），中间不停顿；转向用连续 `yaw_rate`，无单步角度限制 |
| 双层 Deadman | 浏览器 → Web 300ms；Web → ROS worker 500ms |
| 场景物体表 | 右侧表格（名称/数量/位置/置信），每约 5 秒后台识别一次，失败保留上次结果 |
| **LLM 开关** | 页面右上「大模型分析: 开/关」按钮，**默认关闭**（省 token），可随时切换 |
| 状态灯 | Camera / Motion / LLM 三盏灯：绿=ready、黄=受限、红=stale/error、灰=offline |

## 2. 架构

```
┌─────────────────────────────────────────┐
│ Conda: Manual Demo Web Process          │
│  FastAPI                                │
│   ├── index.html + app.js + style.css   │
│   ├── /api/camera.mjpeg  (latest.jpg)   │
│   ├── /ws/control        (键盘)         │
│   ├── /api/objects       (物体表)       │
│   └── SceneObjectAnalyzer (后台线程)    │
│         └── 复用现有 SiliconFlow client │
└──────────────────┬──────────────────────┘
                   │ local JSONL IPC (stdin/stdout)
┌──────────────────▼──────────────────────┐
│ /usr/bin/python3: ROS Worker            │
│  订阅 /camera/front/image_raw/compressed│
│       /lf/sportmodestate, /go2w/safety/*│
│  原子写 outputs/manual_web_demo/runtime │
│  ActionClient /go2w/motion              │
│  Client     /go2w/arm /emergency_stop   │
└─────────────────────────────────────────┘
```

相机、键盘控制、LLM 识别三条独立链路，互相异步（计划书 §26）。

## 3. 安装

```bash
# 主项目依赖（含 fastapi/uvicorn，start 脚本会自动补齐）
cd /home/brov/robot/robot_scene_demo
bash scripts/go2w/install_dependencies.sh

# ROS2 工作区（/usr/bin/python3 侧，含相机桥 / 运动 Action server）
bash scripts/go2w/build_ros2.sh

# 运动控制工作区（unitree_go2w_control 内，提供 /go2w/motion）
cd /home/brov/robot/unitree_go2w_control
source /opt/ros/humble/setup.bash
colcon build --packages-select go2w_motion_interfaces go2w_motion_control
```

SiliconFlow 配置复用主项目 `.env` 中已有的 `SILICONFLOW_*`，本 Demo **不新建第二套
API Key / endpoint / model 配置**（计划书 §23）。

## 4. 启动

```bash
# 只读感知栈（相机/LiDAR/时间/融合/Bundle）
bash scripts/go2w/start_live_perception.sh

# 运动控制（lease holder + Action server，unitree_go2w_control 内）
cd /home/brov/robot/unitree_go2w_control
source scripts/setup_go2w_ros2.sh
ros2 launch go2w_motion_control go2w_motion_control.launch.py

# 本 Demo：相机 + LLM，键盘控制默认禁用
bash scripts/go2w/start_manual_web_demo.sh

# 显式允许运动（会要求清场确认）
bash scripts/go2w/start_manual_web_demo.sh --enable-motion
```

启动后自动打开 `http://127.0.0.1:8765`。控制默认禁用，需点击 **「启用键盘控制」**
才生效；页面刷新/重连后重新禁用。

## 5. 安全

- 运动只走现有 `/go2w/motion` Action 链；**禁止 LowCmd / 关节控制 / 裸 SDK 速度循环**。
- **Gate 先于 arm**：每个方向 pulse 前检查相机新鲜度与对应安全门，全部 PASS 才 arm → Action。
- **W 前进**：要求 `/go2w/safety/lidar_fresh` 且 `front_clearance_m ≥ 0.30 m`（可配置）；
  连续前进期间 worker 每 0.1s 复查一次清障，若前方 <0.30 m 连续两次立即取消运动（不靠单次启动检查）。
- **S 后退**：默认 `MANUAL_DEMO_ALLOW_BACKWARD=false`，无已验证 rear safety 时不可用。
- **A/D 横移**：默认 `MANUAL_DEMO_ALLOW_STRAFE=false`，lateral safety 未验证时不可用；
  即使打开，还需在 `motion_control.yaml` 调高 `max_abs_vy`（当前为 0）才会被 Action server 接受。
- **Q/E 转向**：走现有 rotation safety chain；`rotation_clearance_valid != true` 时 BLOCKED，
  不绕过。
- **相机 stale（>1s）**：画面标红，控制自动禁用并 STOP。
- **Deadman**：浏览器心跳 >300ms 未到 → STOP + 禁用；Web 进程崩溃后 ROS worker
  在 >500ms 无 keepalive 且运动在飞时自行 STOP。
- **Emergency Stop**：Esc 或红色按钮 → `/go2w/emergency_stop`，随后控制保持禁用直到重新启用。

## 6. WASD+QE 映射

| 键 | 语义 | 说明 |
|---|---|---|
| W | 前进 | 长按连续前进（vx=0.12，按住不停） |
| S | 后退 | 长按连续后退（vx=−0.10） |
| A | 左移（横移） | 长按连续左移（vy=+0.06） |
| D | 右移（横移） | 长按连续右移（vy=−0.06） |
| Q | 左转 | 长按连续左转（yaw_rate=+0.15，无角度上限） |
| E | 右转 | 长按连续右转（yaw_rate=−0.15，无角度上限） |
| Space | 停止 | 取消当前运动 |
| Esc | 急停 | `/go2w/emergency_stop` |

**长按 = 连续运动**：按住期间保持一个长 timed-velocity goal，到达 `hold_duration_sec`
（默认 30s）后自动续约，中间不停顿；松手立即 STOP。同一时刻只执行一种运动；
W+S / A+D / Q+E 冲突组合一律 STOP；其它组合**后按的键优先**，切换前先 STOP 当前运动。
转向不再受单步角度限制（用户 2026-08-14 要求）。

## 7. LLM 间隔

`MANUAL_DEMO_LLM_INTERVAL_SECONDS=5`（约每 5 秒尝试一次）。单 worker，最大并发=1；
上一轮未完成时跳过本轮；相机 stale 时暂停识别。识别失败保留上次成功表格并显示
「SiliconFlow 识别暂时失败」。

**LLM 开关**：`MANUAL_DEMO_LLM_ENABLED` 默认 `false`（省 token）。页面右上
「大模型分析: 开/关」按钮可随时切换；也支持 `POST /api/llm/enable` / `POST /api/llm/disable`。

## 8. 故障排查

| 现象 | 处理 |
|---|---|
| 画面黑屏/等待相机 | 先确认 `start_live_perception.sh` 已运行：`ros2 topic hz /camera/front/image_raw/compressed` |
| Motion=OFFLINE | 确认 `go2w_motion_control.launch.py` 已启动：`ros2 service list \| grep /go2w/motion` |
| 前进 BLOCKED：front clearance | 确认 LiDAR 预处理在跑、前方无障碍 ≥0.30 m |
| 转向 BLOCKED：rotation_clearance_invalid | 旋转包络当前未验证，属预期 fail-closed 行为 |
| LLM 识别失败 | 确认 `.env` 中 `SILICONFLOW_API_KEY` 有效；失败会保留上次结果 |
| 键盘无反应 | 点击「启用键盘控制」；页面刷新后会重新禁用 |
| 日志 | `outputs/manual_web_demo/logs/web_server.log` 与 `ros_worker.log` |

## 9. 测试

```bash
cd /home/brov/robot/robot_scene_demo
conda run -n go2_robot_scene_demo python -m pytest \
  tests/test_manual_web_demo_config.py \
  tests/test_manual_drive_controller.py \
  tests/test_manual_drive_deadman.py \
  tests/test_manual_scene_object_parser.py \
  tests/test_manual_scene_object_scheduler.py \
  tests/test_manual_ros_worker_protocol.py \
  tests/test_manual_web_demo_api.py
```

覆盖：键盘状态机（tap/hold/切换/冲突）、deadman（心跳超时/断连/release_all）、
一个 goal 在飞限制、LLM 调度与解析、API 端点、ROS worker JSONL 协议。

## 10. 停止

```bash
bash scripts/go2w/stop_manual_web_demo.sh
```

先优雅关闭 Web（lifespan 内 disable 控制 → 按状态 cancel/estop → worker shutdown），
再兜底清理 worker 进程；**不 kill 相机桥 / 用户其它 ROS 节点**。
