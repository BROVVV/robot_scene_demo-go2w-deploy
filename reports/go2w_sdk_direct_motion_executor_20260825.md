# Go2-W Python SDK 直驱运动改造与实机验收报告

日期：2026-08-25  
设备：Unitree Go2-W，`192.168.123.18`，Ubuntu 20.04 / ROS 2 Foxy  
部署目录：`/home/unitree/robotscene`

## 结论

`LEASE_DENIED` 的根因已经从架构上消除：ROS 2 Action Server 不再复制 lease ID
并向 `/api/sport/request` 转发运动命令。持有 Sport lease 的 Python SDK 客户端
同时运行本地 Unix socket 运动执行器，所有 Move/Stop 都由这个同一客户端直接发送。

最终全负载实机验收结果：

- `Move(0.05, 0, 0)` 持续 0.5 秒，Action 返回 `success=true`。
- `last_move_status_code=0`，`last_stop_status_code=0`。
- 四轮编码器给出强运动证据，`wheel_evidence_strong=true`，28 个采样。
- 最终 `robot_mode=1`、`robot_error_code=0`、`yaw_rate=0`。
- 实机动作前后 lease ID 均为 `1787646658937065`。
- 60 秒全负载压力测试中 lease 变化 0 次、失活采样 0 次；12 次 SDK Stop
  全部返回 0。
- 最终日志中 `LEASE_DENIED/3205=0`、lease 不存在 `3206=0`、Action 进程
  崩溃 0 次。
- 急停和上锁服务均返回成功，交付状态为停止、未解锁。

实机结果文件：
`/home/unitree/robotscene/logs/20260825_164210/goal_9e4c253586884d23b4a0cf16b60d4f25/result.json`

## 架构改造

1. `hold_sport_lease.py` 成为唯一的 lease 所有者和 SDK 运动执行器。
2. C++ `LeasedSportClient` 通过 `/tmp/go2w_sdk_motion.sock` 请求 Move/Stop；不再
   发布 Unitree Sport ROS 请求。
3. `lease_status_bridge.py` 作为独立 ROS-only 进程发布 lease/mode 状态，避免
   rclpy 与 Unitree Python SDK CycloneDDS participant 在同一进程冲突。
4. 续租使用 10 Hz reliable DDS no-reply 心跳，并进行低频同步健康探测；运动
   RPC 若明确返回 lease 拒绝，会使本地 lease 立即失效并触发重新申请。
5. Foxy Action/Service 使用单线程 executor；高频 SportModeState/LowState 由独立
   状态节点执行，既规避 `Executing action server but nothing is ready` 竞态，又能
   在急停服务等待期间持续完成停稳采样。
6. Action launch 配置了自动重启作为额外保护。
7. Web 搜索 worker 的 ESTOP 增加有界终止：第三方视觉/LLM 调用阻塞 IPC 时，
   宽限期后精确 TERM/KILL worker，不阻塞 Web API。

## 安全边界

- Unix socket 权限为 `0600`。
- Python 执行器再次校验有限数值和硬上限：`|vx|<=0.20`、`|vy|<=0.10`、
  `|yaw_rate|<=0.25`。
- C++ Action Server 原有 arm、状态新鲜度、机器人错误码、并发目标、限速、
  pre-stop、final-stop、编码器证据和停稳验证均保留。
- holder 退出时发送三次 Stop。

## 服务状态与入口

- WebUI：`http://192.168.123.18:8765`
- SDK socket：`/tmp/go2w_sdk_motion.sock`
- lease 状态：`/tmp/go2w_lease_status/`
- motion 日志：`/tmp/go2w_motion_sdk_direct.log`
- motion launch PID：`/tmp/go2w_motion_control.pid`
- 远端修改前备份：
  `/home/unitree/robotscene/backups/sdk_direct_20260825_155132`

交付时运行的关键进程包括 lease holder、lease bridge、Motion Action Server、
独立状态节点、实时感知、wheel odom 和 WebUI。WebUI 状态为默认禁用，控制所有者
为 `NONE`。

## 验证摘要

- Python SDK 协议、Web/Search 控制与 ESTOP 回归：109 项通过。
- C++ 构建通过；`go2w_motion_control` 18 项测试通过。
- Python 文件 `py_compile` 通过。
- WebSocket `/ws/control` 与 `/ws/search` 握手成功。
- WebUI readiness：`ready=true`；相机 1280×720、新鲜，运动服务可用，
  mode=1/error=0。
- 实际搜索 dry-run 完成任务理解、worker 启动、RGB-D/readiness 检查并进入观察
  阶段；测试后已急停并清理会话。第三方视觉模型调用阻塞时的有界 ESTOP 已补充
  自动化回归测试。
