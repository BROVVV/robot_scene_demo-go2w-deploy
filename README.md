# Go2-W 自主语义搜索、对象拓扑与固定世界三维建图

这是 Go2-W 机器狗的具身视觉与自主搜索部署仓库。项目把自然语言任务、D435 RGB-D 相机、Hesai PandarXT-16、plain_slam 映射辅助、对象语义拓扑、运动安全门和 WebUI 串成一条可验收链路。

仓库地址：<https://github.com/BROVVV/robot_scene_demo-go2w-deploy>

> 重要：README 里的“部署完成”只表示软件已安装、代码可启动、接口可检查；不等于动态 SLAM、遥控建图或自主运动已经通过现场验收。机器狗运动前必须有人在现场持遥控急停，且先通过本文的安全门。

## 1. 能力边界

WebUI（默认端口 8765）提供：

- 输入自然语言搜索任务，例如“寻找办公椅”或“寻找饮水机旁边的蓝色垃圾桶”；
- 实时相机画面、VLM 目标理解、物体列表和搜索状态；
- 语义拓扑视图：只显示物体节点及物体之间的语义关系；
- 空间三维地图视图：显示固定世界坐标系中的 plain_slam 全局点云、机器人位姿和局部扫描；
- 搜索暂停、恢复、停止、急停以及历史任务回放；
- 手动 WebSocket 控制和搜索决策、frontier、readiness 诊断接口。

当前设计中，导航器内部仍然保留 Place 和 Frontier，但 WebUI 的“语义拓扑”只投影对象节点，不把 P1、F01、OBSERVED_FROM、FRONTIER_TO 混进对象图。三维地图只接受 pslam_map 固定世界点云；pslam_odom 局部扫描不能冒充全局地图。

## 2. 运行架构

推荐的两机部署如下：

| 组件 | Ubuntu x86 控制主机 | Go2-W 机器狗 |
| --- | --- | --- |
| 典型地址 | 192.168.123.99（实际以现场网卡为准） | 192.168.123.18 |
| ROS | ROS 2 Humble，Python 3.10 | 设备自带 ROS 2 Foxy，Python 3.8 |
| 运行目录 | /home/mxt/robotscene | /home/unitree/robotscene |
| 主要进程 | Hesai PandarXT-16、plain_slam、IMU/odom 适配器、Web 点云 relay | 运动控制、相机/状态链、Web 点云 bridge、FastAPI WebUI、VLM daemon |
| 最终浏览器地址 | 可通过网络访问 | http://192.168.123.18:8765/ |

两机必须在同一个 192.168.123.0/24 网段，ROS_DOMAIN_ID 默认都是 0，并使用仓库的 scripts/go2w/setup_environment.sh。不要在一个 shell 中混用 ROS 1、ROS 2 Foxy 和 ROS 2 Humble 的环境变量。

如果现场把所有 ROS 进程放在同一台机器上，也必须遵守“/go2w/odom/fused 只有一个 publisher、/go2w/motion 只有一个 action server、plain_slam 只有一个实例”的原则。

## 3. 硬件和现场前提

新电脑只负责恢复软件，以下条件不能由 GitHub 或安装脚本替代：

1. 一台 Ubuntu 22.04 x86_64 控制主机。建议至少 16 GB 内存；要运行本地 Grounded-SAM-2，建议使用 NVIDIA GPU、匹配的驱动和更多显存。
2. 一台可正常开机的 Go2-W，机器狗地址为 192.168.123.18。
3. 机器狗上的 D435/相机 HTTP 服务可访问，默认检查地址为 http://192.168.123.18:8080/health。
4. Hesai PandarXT-16 已接入控制主机或现场网络，并能发布 /hesai/pandarxt16/points_raw。
5. 控制主机有一张连接机器狗的有线网卡，carrier 为 1，并配置 192.168.123.x/24 地址。
6. 现场有操作员、遥控器和实体急停；自主运动实验区域应平整、干燥、清空至少约 2 m。
7. VLM 在线搜索需要可用的 SiliconFlow API Key 和网络代理/出口。没有 Key 时仍可跑 mock WebUI 和离线回归测试。

SSH 密码、GitHub Token、SiliconFlow Key 都不要写进仓库、脚本、日志或命令历史。下面用环境变量名表示机器狗 SSH 密码。

## 4. 新电脑从零安装

### 4.1 安装基础工具并克隆

在 Ubuntu 22.04 x86 控制主机上执行：

~~~bash
sudo apt update
sudo apt install -y git curl ca-certificates

git clone https://github.com/BROVVV/robot_scene_demo-go2w-deploy.git
cd robot_scene_demo-go2w-deploy
~~~

如果目录已经存在，先确认没有未保存的本地修改，再执行：

~~~bash
git pull --ff-only origin main
~~~

### 4.2 选择安装档位

推荐首次部署使用 go2w：

~~~bash
bash scripts/bootstrap_fresh_machine.sh --profile=go2w
~~~

安装脚本是幂等的，不会启动 ROS 节点，不会给机器狗发运动指令。它会创建仓库内 .venv、安装 Python 依赖、安装/检查 ROS 2 Humble、固定获取 Unitree ROS2 和 Hesai 驱动、构建项目 ROS2 工作区，并执行部署自检。

三个档位的区别：

~~~bash
# 仅 WebUI、Mock、LLM 客户端和 Python 测试；适合没有 ROS 的开发电脑
bash scripts/bootstrap_fresh_machine.sh --profile=core

# 真机 ROS/Unitree/Hesai 能力，不下载本地 Grounded-SAM-2 权重
bash scripts/bootstrap_fresh_machine.sh --profile=go2w

# 额外安装 Grounded-SAM-2、GroundingDINO/SAM2 和公开模型权重
bash scripts/bootstrap_fresh_machine.sh --profile=full
~~~

full 会下载较大的第三方仓库和公开权重，耗时、磁盘空间和 GPU 依赖都更高。D435 HTTP 服务和 VLM 在线服务不属于这个下载步骤。

### 4.3 创建私有运行配置

bootstrap 通常会从 .env.example 创建被 Git 忽略的 .env。若没有，则执行：

~~~bash
cp .env.example .env
chmod 600 .env
~~~

编辑 .env，至少确认：

~~~dotenv
SILICONFLOW_API_KEY=填写现场的SiliconFlow密钥
SILICONFLOW_VISION_MODEL=使用项目默认模型或现场批准的模型
D435_BASE_URL=http://192.168.123.18:8080
~~~

不要把真实 Key 写入 .env.example、.env.go2w、README 或 Git。GO2W_AREA_CLEARED 只在现场确认清场后临时导出，不要长期写进配置。

### 4.4 配置控制主机网卡

先找出与机器狗相连的网卡：

~~~bash
ip -br link
ip -4 -br addr
ip route get 192.168.123.18
ping -c 3 192.168.123.18
~~~

如果控制主机没有 192.168.123.x/24 地址，使用 Ubuntu 的 NetworkManager/网络设置为有线网卡配置静态地址，例如 192.168.123.99/24，网关可留空。然后显式指定网卡：

~~~bash
export GO2W_INTERFACE=<连接机器狗的网卡名>
export GO2W_HOST_IP=192.168.123.99
ping -c 3 192.168.123.18
~~~

setup_environment.sh 会重新生成本项目的 CycloneDDS 配置，并清理继承的 ROS 1 环境。每个新终端都要重新执行：

~~~bash
source scripts/go2w/setup_environment.sh
~~~

### 4.5 安装验证

~~~bash
bash scripts/verify_fresh_deployment.sh --profile=go2w
# 如果安装了 Grounded-SAM-2：
bash scripts/verify_fresh_deployment.sh --profile=full
~~~

验证脚本会检查 Python/FastAPI、核心回归测试、启动脚本语法、ROS workspace、运动 action server 和对应 profile 的构建产物。它不检查现场传感器数据，也不代表允许运动。

## 5. 把项目部署到机器狗

机器狗目标目录固定为 /home/unitree/robotscene。部署脚本只同步整改计划涉及的源代码、配置、脚本和测试，不同步 .env、模型权重、build/、install/、日志、图像和历史运行产物；被覆盖的旧文件会备份到机器狗的 .deploy_backup/<时间戳>/。

先用安全方式准备 SSH 密码环境变量，再执行 dry-run：

~~~bash
read -rsp 'Go2-W SSH password: ' GO2W_ROBOT_PASSWORD
printf '\n'
export GO2W_ROBOT_PASSWORD
export GO2W_ROBOT_HOST=192.168.123.18
export GO2W_ROBOT_USER=unitree

bash scripts/go2w/deploy_plan_to_robot.sh --dry-run
~~~

确认差异后再真正同步：

~~~bash
bash scripts/go2w/deploy_plan_to_robot.sh
~~~

最后应看到 REMOTE_SYNTAX_OK。任务完成后清除密码变量：

~~~bash
unset GO2W_ROBOT_PASSWORD
~~~

首次配置或更新机器狗私有 .env 时，使用现场批准的安全传输方式单独复制，并在机器狗上执行：

~~~bash
chmod 600 /home/unitree/robotscene/.env
~~~

不要把 .env 放进 deploy_plan_to_robot.sh 的同步列表，也不要把密码写成 sshpass -p ...。

## 6. 首次启动顺序

启动顺序不能颠倒。所有以下命令都应在独立终端、tmux 或现场允许的持久 SSH 会话中运行。

### 6.1 控制主机：plain_slam 和 Pandar

控制主机终端执行：

~~~bash
cd /home/mxt/robotscene
export GO2W_INTERFACE=<连接机器狗的网卡名>
source scripts/go2w/setup_environment.sh

# 自动复用已有 Hesai publisher；没有时才启动项目配置的 Hesai 诊断驱动
bash scripts/go2w/start_plain_slam_mapping.sh
~~~

该脚本会生成运行时 plain_slam 参数，检查 Pandar 和 IMU，启动 plain_slam，并启动只用于 WebUI 显示的 compact cloud relay。它不会启动运动，不会发布 /go2w/odom/fused，不会改变运动授权。

若现场已有 Hesai 驱动而脚本误判不到，可使用：

~~~bash
bash scripts/go2w/start_plain_slam_mapping.sh --no-start-hesai
~~~

如果要录制诊断 bag：

~~~bash
bash scripts/go2w/start_plain_slam_mapping.sh --record
~~~

不要在控制主机和机器狗同时启动两套 Pandar/plain_slam。启动后检查：

~~~bash
source scripts/go2w/setup_environment.sh
ros2 topic info /hesai/pandarxt16/points_raw -v
ros2 topic info /go2w/slam/aligned_scan -v
ros2 topic info /go2w/slam/web_map -v
ros2 topic echo /go2w/slam/health --once
~~~

### 6.2 机器狗：运动控制和相机

机器狗终端执行：

~~~bash
cd /home/unitree/robotscene
source scripts/go2w/setup_environment.sh
bash scripts/go2w/start_motion_control.sh
~~~

start_motion_control.sh 会启动唯一的 /go2w/motion action server。它本身不会因为启动就让机器狗运动，但后续任何真实运动都必须由现场操作员授权。

确认机器狗侧相机 ROS topic 不存在时，另开机器狗终端启动只读感知链：

~~~bash
cd /home/unitree/robotscene
source scripts/go2w/setup_environment.sh
bash scripts/go2w/start_live_perception.sh
~~~

相机 HTTP 服务本身先检查：

~~~bash
curl -fsS http://192.168.123.18:8080/health
curl -I http://192.168.123.18:8080/color
~~~

### 6.3 机器狗：VLM 和 WebUI

在机器狗上执行：

~~~bash
cd /home/unitree/robotscene
bash scripts/go2w/start_autonomous_search_web.sh
~~~

启动器会自动尝试启动 VLM daemon、机器人相机 HTTP capture、FastAPI WebUI 和 ROS 侧三维地图 bridge。默认绑定 0.0.0.0:8765，只读模式下不授权自主运动。

如果控制主机的 plain_slam topic 已经能通过 DDS 在机器狗发现，可以使用：

~~~bash
bash scripts/go2w/start_autonomous_search_web.sh --with-plain-slam
~~~

--with-plain-slam 会在看不到现有 mapping topic 时尝试启动本机 mapping。split 部署下，如果日志出现“本机启动第二套 mapping”，应立即停止该实例并先修复 DDS/网卡发现，不能保留两套 SLAM。

打开：

~~~text
http://192.168.123.18:8765/
~~~

如果 WebUI 实际跑在控制主机，则打开 http://<控制主机IP>:8765/；浏览器必须能访问运行 Uvicorn 的那台机器。

## 7. WebUI 使用方法

1. 先只读打开 WebUI，确认相机、WebSocket、VLM 状态和三维地图都在更新。
2. 在任务输入框写完整自然语言目标，例如“寻找办公椅”，不要只依赖内部 object id。
3. 先查看 Search Readiness。地图不健康、运动 action 缺失、odom 重复或传感器不新鲜时，不要点击自主搜索。
4. “语义拓扑”查看物体之间关系；“空间地图”查看固定世界三维点云。两者数据来源不同，不能用对象拓扑代替三维地图。
5. 搜索过程中始终让遥控急停在操作员手中。搜索步长和转向由现有 motion gate 限制，逻辑 60° 转向会拆成 30° + 剩余角度，不是把单次动作上限改成 60°。

关键接口：

~~~bash
BASE=http://192.168.123.18:8765
curl -fsS "$BASE/api/status"
curl -fsS "$BASE/api/search/readiness"
curl -fsS "$BASE/api/slam/map3d"
curl -fsS "$BASE/api/search/semantic-map"
~~~

搜索 API 示例：

~~~bash
curl -fsS -X POST "$BASE/api/search/start" \
  -H 'Content-Type: application/json' \
  -d '{"task_text":"寻找办公椅"}'

curl -fsS -X POST "$BASE/api/search/pause"
curl -fsS -X POST "$BASE/api/search/resume"
curl -fsS -X POST "$BASE/api/search/stop"
curl -fsS -X POST "$BASE/api/search/estop"
~~~

search/start 返回 409 且错误为 MAPPING_FROZEN 时，说明 LIO 漂移健康门已锁存；这是保护动作，不是 WebUI 故障。先按第 10 节新建 mapping session。

## 8. 三维地图正确性的验收标准

三维地图接口 /api/slam/map3d 应满足：

- schema_version=go2w_slam_web_cloud_v2；
- canonical frame 为 frame_id=pslam_map；
- mapping_health=HEALTHY 且 fresh=true；
- map_revision 在有新地图时递增；
- source_map_points > global_cached_voxels >= web_display_points，三层容量不能被同一个显示点数冒充；
- 地图范围持续覆盖已走过的区域，刷新页面后旧区域不能突然消失；
- /api/search/readiness 中 slam_map_not_drifting=true。

/go2w/slam/web_scan 是局部扫描预览，/go2w/slam/web_map 是全局地图 relay，/go2w/odom/fused 是运动/搜索的权威里程计。不能把局部扫描改名成全局地图，也不能在前端旋转点云来掩盖 SLAM 位姿错误。

建议每次现场验收都保存原始 JSON：

~~~bash
OUT=outputs/acceptance/$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUT"
curl -s "$BASE/api/status" > "$OUT/status.json"
curl -s "$BASE/api/search/readiness" > "$OUT/readiness.json"
curl -s "$BASE/api/slam/map3d" > "$OUT/map3d.json"
curl -s "$BASE/api/search/semantic-map" > "$OUT/semantic-map.json"
~~~

## 9. 分级真机验收（必须按顺序）

在 A–D 阶段由操作员用遥控器移动，软件只采集数据和显示状态；E/F 才进入自主动作实验。每一项都要保存日志、原始 JSON 和现场视频。

| 阶段 | 操作 | 通过判据 |
| --- | --- | --- |
| A 静止 60 秒 | 机器狗不动 | LIO 位移接近 0，地图健康持续为 HEALTHY，边界不漂 |
| B 左右分步约 30° | 操作员遥控 | wheel/odom 与 pslam 的每步 yaw 一致，原地转向不产生明显 XY 假平移 |
| C 360° 闭环 | 操作员原地转一圈 | 不出现旋转副本、走廊重影或地图突然冻结后自行恢复 |
| D 遥控行走建图 | 操作员走过多个区域 | 三维地图持续扩展，刷新后早先区域仍在 |
| E 逻辑 60° | 急停在手，自主转向 | 日志可见 r30 + r(remaining) 两段，累计里程计闭合，WebUI 显示总角度正确 |
| F 自主搜索 | 急停在手，自主搜索 | 决策超过 10 步或确实走到 frontier/route 分支；不能把规划异常伪装为 SEARCH_EXHAUSTED；对象拓扑只含对象节点 |

在 A–D 全部通过、且当前 /api/slam/map3d 健康前，不得执行自主平移搜索。当前传感器外参仍属于 candidate_unconfirmed 的环境，不能仅凭离线测试宣称动态 SLAM 已验收。

自主运动授权命令：

~~~bash
cd /home/unitree/robotscene
export GO2W_AREA_CLEARED=I_HAVE_CLEARED_THE_AREA
bash scripts/go2w/start_autonomous_search_web.sh --enable-autonomous-motion --with-plain-slam
~~~

如果不想在 shell 中设置环境变量，启动器会交互要求输入 I_CONFIRM。没有清场或没有急停时不要输入确认。

## 10. 停止、复位和恢复

### 10.1 正常停止

先按实体急停/停止运动，再停止软件：

~~~bash
# 机器狗：停止 WebUI
bash scripts/go2w/stop_autonomous_search_web.sh

# 控制主机：停止本项目的 mapping/relay，不影响外部运动控制
bash scripts/go2w/stop_plain_slam_mapping.sh

# 控制主机：停止项目自有 worker 和搜索取消标志；不接管外部 Sport lease
bash scripts/go2w/stop_all.sh
~~~

如果 start_motion_control.sh 是在前台终端运行，回到该终端按 Ctrl-C。不要用宽泛的 killall、pkill ros2 或删除整个运行目录的方式停机。

### 10.2 地图冻结或 LIO 发散

看到 DEGRADED_LIO_DRIFT、MAPPING_FROZEN、地图边界跳变或原地转向产生米级假平移时：

~~~bash
bash scripts/go2w/reset_plain_slam_session.sh
~~~

该操作会写 reset marker、有序停止旧 mapping、重启 mapping、等待 IMU/LiDAR/odom/map，再等待新 session 的健康快照。它不会发布运动命令。若 WebUI bridge 在机器狗上运行，需要在对应机器上处理相同的 GO2W_SLAM_RESET_MARKER，并确认新 mapping_session_id 已生成。

不要只清网页 voxel、只刷新浏览器或只重新启动 WebUI 来掩盖坏 session；坏位姿必须锁存到新 session。

### 10.3 日志位置

~~~text
outputs/autonomous_search/runtime/slam_map_3d.json
outputs/autonomous_search/runtime/slam_reset.marker
outputs/autonomous_search/logs/web_server.log
outputs/autonomous_search/logs/slam_web_bridge.log
outputs/autonomous_search/logs/plain_slam_web_cloud_relay.log
runtime/go2w/sessions/
runtime/go2w/pids/
~~~

机器狗和控制主机各自保存本机日志。问题报告请附：启动命令、两机 IP/网卡、/api/status、/api/search/readiness、/api/slam/map3d 原始输出，以及相关日志尾部。

## 11. 离线开发和回归测试

先确认使用项目 .venv，不要用没有 Streamlit/项目依赖的系统 Python：

~~~bash
source .venv/bin/activate
python --version
~~~

完整的计划回归集：

~~~bash
.venv/bin/python3 -m pytest -q \
  tests/test_go2w_experimental_backend.py \
  tests/test_local_scan.py \
  tests/test_autonomous_explorer.py \
  tests/test_live_candidate_goal_generator.py \
  tests/test_exploration_budget.py \
  tests/test_search_state_store_spatial.py \
  tests/test_go2w_live_ui_status.py \
  tests/test_plain_slam_webui_3d.py \
  tests/test_spatial_pose_validator.py \
  tests/test_regression_search_20260901_211756.py \
  tests/test_webui_object_topology_contract.py \
  tests/test_plain_slam_session_reset_path.py
~~~

前端拓扑契约：

~~~bash
node tests/test_webui_object_topology_contract.js
~~~

仅检查新电脑 WebUI 是否能启动：

~~~bash
bash scripts/go2w/start_autonomous_search_web.sh --mock
~~~

浏览器打开 http://127.0.0.1:8765/。mock 模式不需要 ROS、机器狗、相机或 API Key，也不会运动。退出时执行：

~~~bash
bash scripts/go2w/stop_autonomous_search_web.sh
~~~

plain_slam_web_bridge.py、plain_slam_web_cloud_relay.py 等 ROS 侧车必须使用 /usr/bin/python3，因为它们需要系统 ROS Python 的 rclpy；纯 Python 测试使用 .venv/bin/python3。机器狗上的新 Python 代码必须兼容 Python 3.8。

## 12. 故障排查速查表

| 现象 | 检查 | 处理 |
| --- | --- | --- |
| ping 不通 192.168.123.18 | ip -4 -br addr、网卡 carrier、网线 | 先修复物理链路和 192.168.123.0/24 地址 |
| ROS topic 数量为 0 | 两机分别 source setup_environment.sh，检查 ROS_DOMAIN_ID、CycloneDDS、网卡 | 清理旧 ROS 环境，确认只有一个 DDS 配置和正确网卡 |
| /go2w/odom/fused publisher 多于 1 | ros2 topic info /go2w/odom/fused -v、ps -ef | 停止重复 wheel odom/sport odom，不得继续自主运动 |
| /go2w/motion action server 多于 1 | ros2 action info /go2w/motion | 只保留一个 motion action server |
| 相机空白 | curl 相机 health、ros2 topic list、camera capture 日志 | 先恢复 D435 HTTP 服务，再检查 ROS camera bridge |
| WebUI 起不来 | web_server.log、curl 127.0.0.1:8765/api/status | 检查端口占用、Python 解释器、.env 和 FastAPI 依赖 |
| VLM 不可用 | siliconflow_vlm_daemon.log、SILICONFLOW_API_KEY | 重新配置 Key、网络和代理；先用 mock 验证 UI |
| 三维地图空白 | /api/slam/map3d 的 reason、mapping_health、bridge 日志 | 确认 /go2w/slam/web_map 有 publisher，必要时新建 session |
| MAPPING_FROZEN | /api/search/readiness 的 blocking 项 | 停止自主搜索，按第 10.2 节复位并重新验收 |
| 搜索三四步就结束 | 搜索 worker 日志、/api/search/events | 查看真实 PLANNING_ERROR、候选异常或预算耗尽，不能把错误改成成功 |
| 地图刷新后旧房间消失 | map_revision、source_map_points、global_cached_voxels、web_display_points | 检查是否启动了旧 bridge 或把局部 scan 当全局 map |
| SSH 后台进程断开 | 是否在前台 SSH 会话直接启动 | 使用持久终端/tmux，或使用脚本自身的 setsid 启动器；不要混用旧进程 |

## 13. 代码和配置维护规则

- 每次更新先在控制主机运行 git diff --check、回归测试和 verify_fresh_deployment.sh。
- 运行时目录、相机帧、模型权重、.env、PID 和日志不应提交。
- 修改控制主机上的 ROS 源码后，要重新 colcon build --symlink-install 或按对应包构建，并重新 source workspace。
- 修改机器狗侧计划文件后使用 deploy_plan_to_robot.sh --dry-run，确认后再同步；不要直接覆盖机器狗上的标定、固件和历史产物。
- 不要通过放宽安全阈值、增大单次转角、删除 Place/Frontier、改写坐标系或吞掉异常来“修复”验收失败。
- 任何未做的现场测试都必须写成“未执行”，不能写成 HEALTHY、已完成或已验收。

## 14. 相关文档

- [AI_PROJECT_GUIDE.md](AI_PROJECT_GUIDE.md)：给新 AI/开发者的工程入口。
- [CODEX_DEPLOY.md](CODEX_DEPLOY.md)：全新电脑的简版部署入口。
- [GO2W_自主搜索_拓扑_SLAM_目标感知_一键整改与验收计划.md](GO2W_自主搜索_拓扑_SLAM_目标感知_一键整改与验收计划.md)：整改目标、实现范围和验收条目。
- [docs/GO2W_整改计划_与_交接书_20260903.md](docs/GO2W_整改计划_与_交接书_20260903.md)：当前整改、已完成代码工作、未完成真机验收和现场交接记录。
- [GO2W_真机自主搜索_详细修改计划书.md](GO2W_真机自主搜索_详细修改计划书.md)：真机搜索链路的详细计划。

## 15. 安全声明

本项目会在用户明确授权并通过启动器安全检查后调用真实机器狗运动链。任何现场运行都必须由具备机器狗操作能力的人员负责，保持实体急停可用，并根据实际环境重新判断风险。仓库中的脚本、测试和 WebUI 不能替代机器狗厂商安全规范、现场风险评估或人工急停。
