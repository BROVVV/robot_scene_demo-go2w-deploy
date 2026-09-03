# Navigation2 集成

本项目保留开放词表感知、LLM-first 推理、证据门控、视频记忆、PSG、拓扑候选和
Motion Horizon；Nav2 只负责 map 坐标下的真实规划与执行。主项目 Python 与 ROS2
Humble Worker 通过 `outputs/nav2/jobs/<request_id>/` 原子 JSON/JSONL 隔离。

## 模式

- `disabled`：默认值，不要求 ROS。
- `offline_preview`：固定 fixture，仅验证接口和 UI，明确标为非真实路径。
- `plan_only`：调用 Humble `BasicNavigator.getPath()`；失败不会降级。
- `execute`：先规划后 `goToPose()`，记录反馈与最终 `/cmd_vel`。

单图像素和视频帧坐标绝不转换成伪 map pose。自动候选必须有 frame、地图坐标及
provenance；否则请在 CLI/Web UI 手工输入地图目标。

## 安装和构建

```bash
bash scripts/install_nav2_humble.sh
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --symlink-install --cmake-args \
  -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
python3 ../scripts/check_nav2_runtime.py --json ../outputs/nav2_health.json
```

如果主项目运行在 Conda 中，上面的 CMake 参数不可省略；它防止 ROS 构建
误用 Conda Python。Worker 本身始终使用 `/usr/bin/python3`。

安装脚本会在全新 Ubuntu 22.04 上初始化 ROS 2 官方 APT 软件源。正确的系统环境
脚本是 `/opt/ros/humble/setup.bash`。不要写成 `setup.bas`，也不要把两个路径
拼接成 `/setup.bashe/setup.bas`。自定义安装位置时只设置一个完整文件路径：

```bash
NAV2_SETUP_BASH=/custom/ros/humble/setup.bash
```

启动外部定位、地图和传感器后：

```bash
ros2 launch robot_scene_nav_bringup robot_scene_nav2.launch.py map:=/path/to/map.yaml
```

示例 footprint、速度和 collision monitor 区域不能用于真机。必须实测 footprint，
验证 map→base_link TF、定位、LaserScan、急停、速度链和底层适配。

## 验证

```bash
python -m pytest -q tests/test_nav2_*.py
python run_demo.py --mock --enable-nav2 --nav2-mode offline_preview \
  --nav2-goal-x 2 --nav2-goal-y 1 --nav2-wait
```

真实只规划：

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
python run_demo.py --mock --enable-nav2 --nav2-mode plan_only \
  --nav2-goal-x 1 --nav2-goal-y 0 --nav2-goal-yaw 0 \
  --nav2-use-current-start --nav2-wait
```

仿真/安全测试区执行：

```bash
NAV2_ALLOW_EXECUTE=true NAV2_FOOTPRINT_CONFIRMED=true \
NAV2_EMERGENCY_STOP_CONFIRMED=true python run_demo.py --mock \
  --enable-nav2 --nav2-mode execute --nav2-goal-x 1 --nav2-goal-y 0 \
  --nav2-use-current-start --nav2-allow-execute --nav2-safety-confirmed \
  --nav2-footprint-confirmed --nav2-estop-confirmed --nav2-wait
```

## 输出与错误

每个 job 保存请求、状态、路径 JSON/CSV/PNG、指令解释、反馈、速度轨迹、日志和
报告；`outputs/nav2_*` 是最近任务快捷副本。ROS setup、action、TF、路径或安全
门控失败均返回明确 `NAV2_*` 错误，绝不自动使用 offline fixture。
