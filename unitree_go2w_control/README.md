# Unitree Go2-W 高层运动恢复

本项目使用 Unitree 官方 SDK2 Python 与 ROS 2 高层 Sport API 恢复 Go2-W 运动控制。禁止 `/lowcmd`、关节位置/速度/力矩控制、`ReleaseMode()`、关闭避障或修改机器人固件。

旧流程的错误是把 `StandDown` 成功作为 `Move` 的前置条件。Go2-W 当前已确认处于 `ai-w`（`wheeled_sport(go2W)`），新流程会独立测试 `Move 1008`，并在所有退出路径发送至少三次 `StopMove 1003`。

当前验收结果：不带 lease 的 `Move` 虽返回成功，但现场确认机身不动；带 Sport lease 的 `vyaw=0.08 rad/s`、`0.4 s` 受限转向已由现场操作者确认机身真实转动。SDK 脚本现默认申请 lease，ROS 2 包装脚本也会保持 lease 并把 lease ID 写入每个 Move/STOP 请求。带 lease 的姿态实机复验中，`StandDown 1005` 仍被当前 `ai-w` 服务以 `-1` 拒绝；机器人没有趴下，测试已安全停止并确认保持 `mode=1` 静止。

## 安全运动测试

测试前必须将机器人放在平整地面、清空周围至少 2 米，并让遥控器保持可立即急停。脚本只接受精确确认字符串：

```text
I_HAVE_CLEARED_THE_AREA
```

SDK2 Python 路径：

```bash
IFACE="$(scripts/detect_unitree_interface.sh)"
.venv/bin/python scripts/inspect_motion_mode.py --interface "$IFACE"
.venv/bin/python scripts/safe_sdk_move_test.py --interface "$IFACE" \
  --vx 0.05 --vy 0 --vyaw 0 --duration 0.5
```

SDK 脚本默认启用 Sport lease；`--disable-lease` 仅用于诊断对照，不应作为正常运行方式。

ROS 2 路径（仅在 SDK2 路径通过后执行）：

```bash
scripts/safe_ros2_move_test.sh
```

姿态能力是独立测试，不再阻塞移动测试：

```bash
.venv/bin/python scripts/safe_sdk_posture_test.py --interface "$IFACE"
```

姿态脚本也默认申请 Sport lease，并在任何失败或退出路径执行三次 STOP；如果已经接受趴下动作但未重新站起，还会先执行安全 `StandUp`。当前机器的带 lease 结果为 `StandDown=-1`，不可改用 `Damp` 或低层控制模拟趴下；需由 Unitree 确认该 `ai-w` 固件对姿态 API 的能力和前置条件。

诊断采集：

```bash
scripts/collect_go2w_diagnostics.sh
```

所有结果保存在 `logs/<时间戳>/`，恢复结论见 `reports/go2w_motion_recovery_report.md`。

## 遥控器姿态被动抓取

该流程只监听官方遥控器、Sport 请求/响应和机器人状态，不申请电脑端 Sport lease，也不发布运动或关节控制消息。禁止对捕获的控制话题执行 `ros2 bag play`。

```bash
# 只读预检；创建新的 captures/<时间戳> 目录
bash scripts/preflight_remote_capture.sh

# 完整现场流程：QoS dry run、空闲基线、3 次趴下和 3 次起立
bash scripts/run_remote_posture_capture.sh

# 只读离线分析最近一次完整抓取
bash scripts/analyze_remote_capture.sh "$(<.latest_remote_capture)"
```

完整流程要求操作者精确输入 `I_CONFIRM_PASSIVE_CAPTURE`，每轮姿态动作只能通过官方遥控器触发。若预检发现网络、ROS、PCAP 权限、电脑端控制进程或只读审计不合格，流程会在启动现场抓取前退出。

关键输出：

- `captures/<时间戳>/analysis/action_round_summaries.json`
- `captures/<时间戳>/analysis/path_classification.json`
- `reports/go2w_remote_posture_capture_report.md`
- 路径 B/C 或证据不足时的 `reports/unitree_go2w_remote_posture_api_question.md`

只有至少两轮一致、可匹配响应且与姿态变化相关的非基线公开 Sport 请求才能进入复刻审计；只有遥控器 keys 而没有公开请求时，不会生成伪造遥控器发布程序。

2026-08-05 实机抓取结果：三轮趴下和三轮起立均重复观察到遥控器 `keys=0x0120`（bits 5+8）及腿关节/模式变化；动作窗口内没有出现新的公开 Sport 请求，所有 272 条请求均为基线周期 `API 1034 economicGait`。结论为路径 B——当前 `wheeled_sport` 很可能在机器人内部消费遥控器状态。证据不足以安全复刻，因此不生成控制程序；详见 `reports/go2w_remote_posture_capture_report.md`。

## ROS 2 定时速度与定角转向 Action

加载环境并启动一体化 lease holder 与 Action Server：

```bash
source scripts/setup_go2w_ros2.sh
ros2 launch go2w_motion_control go2w_motion_control.launch.py
```

另开终端加载同一环境，授权后执行动作：

```bash
source scripts/setup_go2w_ros2.sh
scripts/go2w_arm.sh on
scripts/go2w_move_time.sh --vx 0.10 --seconds 3
scripts/go2w_move_time.sh --vx -0.08 --seconds 2
scripts/go2w_turn_angle.sh --degrees -90 --max-yaw-rate 0.20
scripts/go2w_turn_angle.sh --degrees 45 --max-yaw-rate 0.15
```

正角度左转，负角度右转。速度乘时间只是理论路程，实际距离由状态积分估算；转角依赖 IMU yaw 闭环。首次转角前必须运行 `scripts/calibrate_yaw_direction.sh`，并在标定后重启 launch。所有运动都必须保持有效 Sport lease。

Go2-W 固件在 `vx=0` 的纯 yaw 命令下可能产生纵向后滑。首次 `+0.04 m/s` 实机标定仍有少量后滑，因此 Action Server 当前默认在转向期间加入 `+0.05 m/s` 的前向补偿，接近目标角度时线性衰减并在停止窗内归零，硬上限为 `0.06 m/s`。可在启动时调整或关闭：

```bash
ros2 launch go2w_motion_control go2w_motion_control.launch.py \
  turn_longitudinal_compensation_vx:=0.05

# 关闭补偿用于诊断对照
ros2 launch go2w_motion_control go2w_motion_control.launch.py \
  turn_longitudinal_compensation_vx:=0.0
```

该补偿只作用于相对转向，不改变定时直线运动。改变数值后必须重新从左右 10° 低速测试开始，禁止直接测试 90°。

现场复测确认 `0.05 m/s` 已消除转向过程中的后滑，但旧停止时序在进入目标窗后立即 `StopMove`，仍可能在控制器释放时回滚约 5 cm。当前实现会先用高层 `Move(0,0,0)` 主动保持轮速 1 秒，再执行三次 `StopMove` 和静止验证；这不是低层电机制动，Cancel、故障和急停仍会跳过保持阶段并立即 STOP。保持时间可在 `0–3` 秒内配置：

```bash
ros2 launch go2w_motion_control go2w_motion_control.launch.py \
  post_turn_zero_velocity_hold_sec:=1.0
```

每次运动脚本都会要求精确输入 `I_HAVE_CLEARED_THE_AREA`。非交互自动化只有同时设置 `GO2W_AREA_CLEARED=I_HAVE_CLEARED_THE_AREA` 并传入 `--yes` 才会放行。默认限制为 `|vx|<=0.20 m/s`、`vy=0`、定时 `|yaw_rate|<=0.20 rad/s`、相对角度不超过 180°，且只有已验证的初始 `mode=1` 可授权。

取消、急停与解除授权：

```bash
# 正在运行 go2w_action_client.py 时按 Ctrl-C 会请求 Action Cancel
scripts/go2w_stop.sh
scripts/go2w_arm.sh off
```

`go2w_stop.sh` 调用 `/go2w/emergency_stop`，会撤销授权、终止活动 Goal、重复 StopMove 并验证静止。Goal 正常结束、Cancel、lease 丢失、状态陈旧、机器人 error、RPC 失败和节点退出也都会进入 STOP 路径。

日志位于 `logs/<时间戳>/goal_<UUID>/`，包含 Goal、每个带 lease 请求及匹配响应、Sport/LowState、反馈、安全事件和最终结果。汇总见 `reports/go2w_action_motion_control_report.md` 与 `reports/go2w_action_dry_run_report.md`。

常见拒绝原因：未授权、lease/状态过期、MotionSwitcher 不是 `ai-w`、机器人不静止、初始模式不在白名单、转向方向尚未标定、参数越界或已有活动 Goal。出现 `ERROR_STATE_STALE` 时检查网卡与 DDS；出现 `ERROR_STOP_FAILED` 时立即使用遥控器急停。

安全限制：机器人必须位于平整防滑地面，周围至少 2 米无障碍，遥控器在手且没有其他控制节点。不得发布 `/lowcmd`，不得调用 `ReleaseMode()`、Damp 或关节/电机控制，也不得关闭保护、避障或修改固件。完整实机顺序由 `scripts/run_motion_acceptance.sh` 执行，但只有 yaw 方向已标定为 ±1 时才会开始。
