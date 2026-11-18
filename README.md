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

### 2.1 视觉职责分工

| 组件 | 职责 |
| --- | --- |
| RAM++ | 自动场景 tags（这一帧大致有哪些类别） |
| GroundingDINO | 开放词汇定位 |
| SAM2 | 实例 mask |
| D435 | 3D localization |
| VLM | 复杂属性/关系与漏检兜底 |

**RAM++ 不决定任务目标是否存在。** 用户目标词由 TargetProfile（任务解析 + 颜色 +
上下文 + 关系）独立产生，无论 RAM++ 这一帧给出什么 tag 都会进
GroundingDINO 的 prompt；RAM++ 只负责扩大普通场景物体的覆盖面。
RAM++ 漏词时，由低频的整帧 VLM 目标扫描兜底，而不是直接宣告目标不存在。
普通场景物体不再走 VLM 全场景枚举 —— Grounded-SAM 模式的检测结果本身就是场景物体来源。

另有可选的本地 `yoloe_seg` 开放词汇实例分割路线，可同时用于自主搜索和实体
遥控器被动语义建图；Grounded-SAM 仍是默认后端。安装、无运动 smoke、切换和
回滚方法见 [YOLOE-Seg 部署与切换](docs/YOLOE_SEG_部署与切换.md)。

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

建图期间每个关键帧会在 `/tmp/go2w_plain_slam` 留下两个 `.bin`，且**运行中不能删**（回环检测和出图都要读回它们）。这个目录只增不减，2026-09-11 曾把根文件系统写到 100%（约 47G）。所以启动时有两道保护：磁盘余量不足直接拒绝启动；起来后由 `scripts/go2w/guard_plain_slam_disk.sh` 盯着，缓存超过 `GO2W_SLAM_DISK_MAX_CACHE_GB`（默认 20）或磁盘剩余低于 `GO2W_SLAM_DISK_MIN_FREE_GB`（默认 20）时有序停掉建图栈保住磁盘——运动链不受影响。日志在 `runtime/go2w/sessions/plain_slam_disk_guard.log`。

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

在控制主机上执行：

~~~bash
cd /home/mxt/robotscene
bash scripts/go2w/start_nav2_autonomous_search.sh
~~~

这是 Nav2 单链路的唯一入口。它按顺序检查感知/激光雷达/plain_slam 是否已在跑
（不在跑就报出该启动哪个脚本，不代管），拉起 Nav2 bringup、控制仲裁与
cmd_vel bridge，等齐 `/map`、`map -> odom_fused -> base_link` TF、
`/go2w/nav/localization_valid`、`planner_server` active 之后，才启动 WebUI 与
搜索 worker（后端固定 `nav2`）。默认绑定 0.0.0.0:8765。

**默认只规划、不执行**：不带开关时 Nav2 以 `execution_enabled:=false` 启动，
控制器、速度平滑和 Collision Monitor 一个都不起，任何速度都不可能进入机器人。
现场验收和设备所有者授权后，执行模式必须同时显式传执行与受监督转向参数：

~~~bash
bash scripts/go2w/start_nav2_autonomous_search.sh \
  --enable-motion --operator-supervised-rotation
~~~

即便加了它，速度仍要再过 `go2w_cmd_vel_bridge` 的九道闸门（运动授权 / 急停 /
租约 / 雷达 / 定位新鲜 / 控制来源 / 机器人错误 / 遥控 / 命令看门狗）。
`configs/go2w/navigation_gate.yaml` 的 `nav2_allow_execute` 记录设备所有者的执行授权。
它只开放启动资格，不能替代启动时及每条命令上的实时安全检查。正式 `field_large`
模式不限制会话目标数，每个 Nav2 目标仍同时受规划路径 3 m 和独立轮式里程 3 m 上限约束。
自主前沿搜索通常只取当前已观测路径的 0.50 m 前缀，走完就停下更新地图并重规划；
这让地图随移动逐步扩展，同时保留 3 m 作为任何单个 Nav2 目标都不能越过的硬上限。

旧的 `start_autonomous_search_web.sh` 仍可用，但它是**旧入口**：只起 WebUI 与
ROS 侧车，不启动 Nav2 bringup / 坐标桥 / 速度平滑 / Collision Monitor，单独用
它做 Nav2 自主运动会得到一条断掉的速度链。新入口内部会调用它，所以 WebUI 功能
不会少（`--mock` 等离线开发用法照旧）。

打开：

~~~text
http://192.168.123.99:8765/
~~~

WebUI 运行在控制主机，`192.168.123.18` 是机器狗地址；如果控制主机地址变化，
打开 `http://<控制主机IP>:8765/`。浏览器必须能访问运行 Uvicorn 的那台机器。

### 6.4 静止上电验证：只规划、不运动（计划书 §13.2）

**前置：轮式里程计必须已经在跑。** `map -> odom_fused -> base_link` 这条 TF
主链的后半段由 `wheel_odom` 发布。上游没有任何一个脚本会替你发布它：§6.1 的
`start_plain_slam_mapping.sh` 明确「不会发布 `/go2w/odom/fused`」，
`start_nav2_autonomous_search.sh` 按「本脚本不代管感知/SLAM」的约定**既不启动
它也不等待它**。漏了它，就绪闸门会停在一句没有归因的
`[FAIL] map -> odom_fused -> base_link TF 连通`：

~~~bash
ros2 launch go2w_lio_bringup wheel_odom.launch.py
ros2 topic info /go2w/odom/fused -v | sed -n '/Publisher count/p'   # 必须是 1
~~~

机器狗摆好、人退到安全距离，**不带开关**启动（第 1 项本身也是这条命令）：

~~~bash
cd /home/unitree/robotscene
source scripts/go2w/setup_environment.sh     # 七项复核都在这个终端里做
bash scripts/go2w/start_nav2_autonomous_search.sh
~~~

启动脚本**只在自己进程里**装环境，不回传给父 shell，所以这一行必须自己敲。
**别图省事只 source `/opt/ros/foxy/setup.bash`**：那样 `ros2` 命令都能跑，但
`/usr/bin/python3` 没有 `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，第 4 项那段内联
Python 里每个 rclpy 节点建图都会 `std::bad_alloc` —— 看着像内存不够，真因是 DDS
实现不对。

启动脚本自己已经等齐了 `/map` 唯一发布者、`map -> base_link` TF 和
`planner_server` active。下面七项是给现场人员的复核，任何一项不对就先停下来，
不要进入 §9 的自主运动阶段。

**1. `/map` 的 `frame_id` 必须是 `map`**

~~~bash
ros2 topic echo /map --no-arr \
  --qos-durability transient_local --qos-reliability reliable 2>/dev/null \
  | grep -m1 'frame_id'
~~~

**Foxy 上没有 `--once`，也没有 `--field`**（两者都是后续版本才加的），照抄
`ros2 topic echo --once --field header.frame_id /map` 会以参数错误告终。
`/map` 是 latched 的，必须显式给 `transient_local` 才收得到；`--no-arr` 只是
免得把 64 万个栅格值打到终端上。**别用 `ros2 topic echo` 去查非 latched 的
话题**（如 `/go2w/nav/localization_valid`）：这台 Foxy 上它对 VOLATILE 话题
收不到任何消息，空输出长得像「话题没在发」。查这类话题用 §6.4 第 4 项那种
内联 rclpy 写法。

**2. `/map` 只能有一个发布者**

~~~bash
ros2 topic info /map -v | sed -n '/Publisher count/p'
~~~

**3. TF 主链连续、没有重复发布者**

~~~bash
ros2 run tf2_tools view_frames.py   # 生成 frames.pdf / frames.gv，在 /tmp 下跑，别把产物落在仓库里
ros2 run tf2_ros tf2_echo map base_link
ros2 topic info /tf -v | sed -n '/Publisher count/p'    # 重复发布者先在这里现形
~~~

机器狗是 Foxy，可执行名带 `.py` 后缀；`ros2 run tf2_tools view_frames`（Humble 起
才叫这个名字）在狗上会回 `No executable found`，别把它当成"TF 不在"。`view_frames.py`
最后调 `dot` 画图，狗上 `/usr/bin/dot` 是有的。

注意 `view_frames` 只能显示**每个子坐标系最终生效的那一个** broadcaster（它的 dot
数据按子坐标系做字典键，后来的覆盖先前的）。所以"同一个子坐标系有两个 broadcaster"
这件事它**看不出来**——那种情况要靠 `/tf` 的 Publisher count，或者看那条边上报的
平均频率是不是翻倍。

`map -> odom_fused` 由 plain_slam 坐标桥发布，`odom_fused -> base_link` 由运动栈发布。
同一个子坐标系挂到了两个不同的父坐标系上，就是第二个 `/map` 或第二套定位混进来了，
先解决它。

**4. 静止 10 分钟，`map -> odom_fused` 不应有无解释的大跳变**

`tf2_echo` 只能靠肉眼，下面这段给出可复核的最大跳变（采样 2 Hz，跑 600 秒）：

~~~bash
/usr/bin/python3 - <<'PY'
import rclpy, math
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

rclpy.init()
node = Node("plan_only_drift_probe")
buffer = Buffer()
TransformListener(buffer, node)
last = None
max_xy = max_yaw = 0.0
samples = 0
for _ in range(1200):                      # 1200 * 0.5s = 600s
    rclpy.spin_once(node, timeout_sec=0.5)
    try:
        t = buffer.lookup_transform("map", "odom_fused", rclpy.time.Time(), Duration(seconds=0.2))
    except Exception:
        continue
    tr = t.transform.translation
    q = t.transform.rotation
    yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
    if last is not None:
        max_xy = max(max_xy, math.hypot(tr.x - last[0], tr.y - last[1]))
        max_yaw = max(max_yaw, abs((yaw - last[2] + 180.0) % 360.0 - 180.0))
    last = (tr.x, tr.y, yaw)
    samples += 1
node.destroy_node()
rclpy.shutdown()
print(f"samples={samples} max_xy_jump={max_xy:.4f} m max_yaw_jump={max_yaw:.3f} deg")
PY
~~~

机器狗静止时两个数都应接近 0。出现米级跳变说明 SLAM 在漂或有人在推 map，先查
地图健康，不要继续。

**5. 请求多个 `ComputePathToPose`，确认路径在膨胀区外、偏向走廊中间**

**先确认障碍层真的在吃扫描**，否则这一项是空过的：`/go2w/lidar/scan` 照常 14 Hz 发帧，
但帧时间戳一旦偏出 TF 缓冲就会被障碍层整条丢掉，而雷达链上看不出任何异常。判据是
Nav2 日志里**没有**持续的丢弃：

~~~bash
grep -c "Message Filter dropping message" outputs/autonomous_search/logs/nav2_bringup.log
# 0，或只有启动瞬间那一两条（那时 map 帧还没出现）。持续增长就是时间轴偏了。
~~~

但**这个 `0` 单独不能当结论**：障碍层根本没 activate 时它同样一帧都不丢，两种情况的
grep 输出一模一样。所以必须再要一个正面证据 —— 代价地图里得真有障碍格：

~~~bash
/usr/bin/python3 - <<'PY'
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import GetCostmap
rclpy.init(); n = Node("costmap_hist")
c = n.create_client(GetCostmap, "/global_costmap/get_costmap")
assert c.wait_for_service(timeout_sec=10), "拿不到 get_costmap，代价地图没在跑"
f = c.call_async(GetCostmap.Request())
rclpy.spin_until_future_complete(n, f, timeout_sec=20)
m = f.result().map                      # Foxy 上字段是 .metadata，不是 .info
h = {}
for v in m.data:
    h[v] = h.get(v, 0) + 1
print("%dx%d res=%.3f" % (m.metadata.size_x, m.metadata.size_y, m.metadata.resolution))
print("254(致命)=%d 253(内切)=%d 其它非零=%d 空闲(0)=%d" % (
    h.get(254, 0), h.get(253, 0),
    sum(v for k, v in h.items() if k not in (0, 253, 254)), h.get(0, 0)))
assert h.get(254, 0) + h.get(253, 0) > 0, "整张地图没有一个障碍格：障碍层没在打标，第 5 项是空过"
n.destroy_node(); rclpy.shutdown()
PY
~~~

实测（2026-09-21，时间对齐修好之后）：`254=196 253=838 其它非零=3068 空闲=636932`。
**一个 254/253 都没有就说明这一项空过了**，此时上面那条 grep 很可能也是 0 —— 千万别
把它读成"正常"。全 `0`（整张 `FREE`）正是障碍层没有输入时代价地图退化的样子。

**机器狗的钟一跳变，`configs/go2w/time_sync.yaml` 就作废**（RTC 没电，每次重启都要
从宿主推钟，见 `scripts/go2w/sync_robot_clock.sh`）。失效后障碍层丢 100% 的帧，代价
地图退化成整张 `FREE`。重启过机器狗或跑过 `sync_robot_clock.sh apply` 之后，**必须
在机器狗上重测**这份拟合（`measure_time_sync --duration 120`，重测时让机器保持在
稳态负载，别在窗口开始前"腾机器"），再重启 `start_live_perception.sh`。

只规划模式下没有控制器，但规划器是活的，可以直接点它：

~~~bash
ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}, planner_id: GridBased}" \
  --feedback
~~~

字段名按狗的 Foxy 写：goal 里是 `pose` 加 `planner_id` 两项，**没有** `goal` 也没有
`use_start`（那两个是 Humble 起的写法，照抄会静默解析失败或直接报错）。狗上可以用
`cat /opt/ros/foxy/share/nav2_msgs/action/ComputePathToPose.action` 复核这张表。

换几个目标点各点一次，看返回的 `path.poses`：路径不能穿过障碍物膨胀区，宽走廊里
应当贴着中线走。规划器用的是 `/map` 的静态层加 `/go2w/lidar/scan` 障碍层。

注意 **UNKNOWN 在当前配置下是「可通行」的**：`nav2_params_humble.yaml` 的
`global_costmap` 写着 `track_unknown_space: false`，静态层会把地图里的未知格按
`FREE_SPACE` 填，所以规划器**会**把路径规划进尚未探索的区域。这对"自主搜索"是
有意的（探索未知本来就是任务目标），但验收时别把它当成"UNKNOWN 被挡住了"的
证据 —— 那一条没有生效。要改成不可通行是 `track_unknown_space: true`，代价是
未知区域一大片就规划不出路径，**不要在验收现场临时改**。

**6. `/go2w/nav2_cmd_vel` 即使有规划也不会进入机器人**

~~~bash
ros2 topic info /go2w/nav2_cmd_vel -v | sed -n '/Publisher count/p'   # 只规划模式应为 0
ros2 topic info /go2w/cmd_vel_selected -v | sed -n '/Publisher count/p'  # 仲裁器在跑，应为 1
timeout 60 ros2 topic echo /go2w/cmd_vel_bridge/status                  # 应持续打印拒绝原因
~~~

只规划模式下 Collision Monitor、控制器和速度平滑都没起，`/go2w/nav2_cmd_vel`
根本没有发布者。即便有速度到达，bridge 的闸门也会因为 `execution_disabled`
（没传 `execution_enabled:=true`）拒绝——`/go2w/cmd_vel_bridge/status` 会把这条
原因打出来。速度最终是通过带租约的 `MotionCommand` action 下发的，不是
`/cmd_vel` 话题。

**7. 走廊分析日志：每个候选一行**

计划书 §14 要求的结构化记录（方位、路径长度、左右最小侧向净空、走廊判定、速度
上限、拒因）随 `navigation_result` 事件落到该次 WebUI 会话的 `events.jsonl`，
字段在事件的**顶层**（`AutonomousExplorer._emit` 把关键字参数平铺进事件，
不套 `payload`）：

~~~bash
python3 - <<'PY'
import glob, json, os
paths = sorted(glob.glob("outputs/live_runs/*/events.jsonl"), key=os.path.getmtime)
if not paths:
    raise SystemExit("还没有 WebUI 会话的 events.jsonl")
for line in open(paths[-1], encoding="utf-8"):
    row = json.loads(line)
    if row.get("event") != "navigation_result":
        continue
    record = row.get("candidate_decision")
    if record:
        print(json.dumps(record, ensure_ascii=False))
PY
~~~

同样的记录在网页上也能直接翻：`explorer_search_adapter` 把 `navigation_result`
转写成 `ACTION_FINISHED` 交给 WebUI 状态存储，落在快照的
`robot.last_motion_result.candidate_decision`。注意这两处**不在同一个文件里** ——
`ACTION_FINISHED` 不进 `outputs/live_runs/*/events.jsonl`，按它去 grep 那个目录
一行都搜不到。

`candidate_decision.reject_reason` 只会是 `NO_MAP / NO_LOCALIZATION / NO_PATH /
GOAL_OCCUPIED / TOO_NARROW / NAV2_NOT_READY` 里的一个。看到别的值说明有代码路径
绕过了这套判定（`nav2_search_backend._candidate_reject_reason`）。

一个**会让现场读错**的细节：走廊分析器内部的原因串比这个枚举细，收口时枚举外的
一律记成 `TOO_NARROW` —— 所以 `NO_PATH_SAMPLES`（路径一个采样点都取不到）在候选
记录里长的就是 `TOO_NARROW`，而它和「真的窄」处置不同。**原始串不丢，它在同一事件
的 `message` 里**（`corridor rejected the path: <原始串>`）。看到 `TOO_NARROW`
先去 `message` 里核对，别直接按「走廊太窄」处置。

另有一类**在规划之前就被挡下**的目标（顶层 `reject_reason` 为 `NOT_MAP_FRAME`
或 `INVALID_GOAL`），它们根本不生成 `candidate_decision`：`NOT_MAP_FRAME` 意味着
有调用方把非地图系目标送进了 Nav2——§6.7 做完之后生产路径上不该再出现，出现了
就是回归。

七项都过了，才可以进入 §9 的 E/F 阶段和 §13.3 的低速现场验收。**`configs/go2w/navigation_gate.yaml`
的 `nav2_allow_execute` 在设备所有者明确授权前必须保持 `false`；授权后仍须保留全部实时闸门。**

## 7. WebUI 使用方法

2026-10-14 恢复了 9 月使用过的网页相对运动搜索，作为**单独的受监督短距离验收档位**：
`bash scripts/go2w/start_autonomous_search_web.sh --supervised-legacy-search`
只读启动；与 `--enable-autonomous-motion` 同用才会进入现场预检并允许运动。
Nav2 仍是普通网页启动的默认后端。短程验收档位限制 3 个高层导航目标尝试、
3 轮规划，并在分段/修正指令入口限制总共 3 次物理指令尝试；失败不返还次数。
每次前进请求不超过 0.20 m，实测超限则停止后续动作；前进包线仍待真机复验。
2026-10-15 仅完成三条转向原语，净位移均小于 0.02 m；停止计时缺陷已离线修复，
完整搜索、无新信息脱困和 Nav2 到位仍未验收，见
[接手审查报告](reports/go2w_handoff_review_20261015.md)。

以下是 2026-10-15 的旧受监督相对运动档记录；正式 Nav2 大场地入口与当前验收状态
见本节开头及 [2026-10-23 接手审查](reports/go2w_nav2_unlimited_takeover_review_20261023.md)。
该旧档当时改为：**单个前进目标最多 3 m，总时长、规划轮数和运动次数不限**。
3 m 是上限；已知较近前沿时按其距离请求，缺少距离信息时先走不超过 0.30 m 的观察短步。
每个目标仍拆为不超过 0.30 m 的执行段，逐段停下并检查传感器和运动闸门。
单条动作/感知超时、故障停车、无进展处理、找到目标、手动停止和急停仍生效。
这次改的是现有受监督相对运动档，**未完成 Nav2 真机验收**；当前网页保持只读，地图仍待恢复。

选择该档的只读启动命令：

```bash
bash scripts/go2w/start_autonomous_search_web.sh --supervised-legacy-search --legacy-profile field_large
```

现场前置条件通过后，运动启动在同一命令末尾加 `--enable-autonomous-motion`，并完成既有现场确认。
`supervised_short` 仍保留“三条物理指令、每步前进请求 ≤0.20 m、总时限 240 s”的短测包线。
详细配置和本轮离线验证见 [持续搜索参数说明](reports/go2w_continuous_search_20261015.md)。
本次证据与未通过项见 [网页旧路径验收报告](reports/go2w_legacy_web_search_restore_20261014.md)。

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
curl -fsS "$BASE/api/search/state"
~~~

搜索的图只有一个来源：`/api/search/state` 的 `graph` 字段（含 Place、物体、
Frontier、关系边与 `graph_revision`）。`/api/search/map`、`/place-graph`、
`/semantic-map`、`/objects`、`/frontiers`、`/decisions`、`/events`、
`/executor` 这些从同一份 state 重复投影的 GET 路由已经删除，不要再照着旧文档
调用；历史会话在 `/api/search/history/{session_id}` 里同样是这一份图。

每个 Web 搜索会话在 `outputs/live_runs/<session_id>/` 下归档四份：`webui_session.json`
（索引）、`webui_state.json`（**不含图**的小状态）、`webui_graph.json`（唯一图快照）、
`webui_events.jsonl`（只追加小事件）；`events.jsonl` 是 worker 的原始运行证据。

### 搜索回放（二维地图 + 实测轨迹视频）

2026-10-21 接管整改状态见 [Nav2 与回放接管报告](reports/go2w_takeover_nav2_20261021.md)：
已修复第二轮残留问题及跨重启地图静默复用，同位置新图规划恢复通过；本轮 0.5m
运动尝试被保护阻断，未通过大地图自主搜索真机验收。正常入口保持只读。

真实搜索默认自动录制：`ReplayManager` 在搜索启动时拉起独立的**只读** ROS 采集器
（`/usr/bin/python3`，只订阅 `/map`、`/go2w/odom/fused`、定位状态、TF，并通过
HTTP 取 D435 帧，不发布任何消息），会话结束后在后台生成 1920×1080 / 30 fps 的
H.264 MP4。历史详情页的「搜索回放」面板可播放、拖动、切换 1×/5×/10×/20×、
下载 MP4、查看最终地图和封面。

- 配置：`configs/go2w/search_replay.yaml`；mock 模式默认关闭，避免合成数据入档。
- 离线重建：
  `.venv/bin/python scripts/go2w/render_search_replay.py --session-dir outputs/live_runs/<session_id> --speed 10`
- 录制包与状态机见 `app/search_replay/`；真实/合成来源、验收限制与证据见
  [搜索回放实施与验收报告](reports/go2w_search_replay_20261021.md)。
- 尚未有一次真实运动的自主搜索被录制；真实只读验收素材是静止采集（0.00 m），
  不能把合成轨迹当作真机轨迹。
旧归档把图嵌在 `webui_state.json` 里，运行时**不做**静默兼容，需要跑一次迁移：

~~~bash
# 先看报告（不写任何文件），确认后加 --apply；原文件先整目录备份
.venv/bin/python scripts/go2w/migrate_webui_archive_v1.py
.venv/bin/python scripts/go2w/migrate_webui_archive_v1.py --apply
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

### 8.1 三维物体与地图怎么配对（计划书 §2.3，2026-10-08）

三维视图上并排显示的两样东西来自**两个进程、两个文件**，靠一对标识配对：

| 文件 | 谁写 | 内容 |
| --- | --- | --- |
| `<runtime>/slam_map_3d.json` | 当前模式唯一的地图桥 | 点云、轨迹、地图健康 |
| `<runtime>/slam_map_meta.json` | 同一个地图桥，**在地图之后**原子写 | 只含 `map_epoch`/`frame_id`/`map_revision`/`generated_at` |
| `<runtime>/semantic_objects.json` | 语义进程（自主搜索 / 遥控侧车） | 待显示物体 + 过滤统计 |

`map_epoch` 是地图桥在**进程启动时**随机生成的一段前缀 + 本进程的地图会话计数，
地图 reset 时再换一次 —— 桥重启就是新 epoch，所以上一轮会话的物体不会画到新地图
上。`GET /api/slam/map3d` 只在 `map_epoch` 与 `frame_id` **同时相等**时合成
`objects`，否则返回空 `objects` 并在 `object_overlay_status` 里说明原因：

| object_overlay_status | 含义 |
| --- | --- |
| `OK` | 配对成功，`objects` / `object_filter_stats` 可用 |
| `NO_OBJECT_SNAPSHOT` | 本模式没有对象源（手动演示就是这种），或文件读不出来 |
| `FRAME_MISMATCH` | 物体与地图不在同一个坐标系 |
| `MAP_EPOCH_MISMATCH` | 物体属于**另一张**地图（桥重启过 / SLAM reset 过） |

判据是数据里的标识，不是文件修改时间、也不是坐标包围盒 —— 后者永远猜不准
（2026-09-17 实测：页面画的是 09-10 归档里的 7 个物体，当天搜索刚定位的 20 个
一个都没上屏）。两个地图桥都**不再**从 `events.jsonl` 里挑物体；原始逐帧检测仍
写在 `events.jsonl` 里，供取证与离线回放，不参与在线选源。

`/api/slam/map3d` 仍是前端唯一的三维接口（没有第二个 `/api/objects3d`），点云
按文件 `mtime_ns` 缓存解析结果。三个路径由启动脚本显式给出：
`GO2W_SLAM_MAP_SNAPSHOT` / `GO2W_SLAM_MAP_META` / `GO2W_SEMANTIC_OBJECTS_SNAPSHOT`
（手动演示用 `outputs/manual_web_demo/runtime/`，遥控建图用自己的默认目录，
见 §11.4；自主搜索用 `outputs/autonomous_search/runtime/`）。**"物体没显示"先看
`object_overlay_status`，不要去看文件时间。**

建议每次现场验收都保存原始 JSON：

~~~bash
OUT=outputs/acceptance/$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUT"
curl -s "$BASE/api/status" > "$OUT/status.json"
curl -s "$BASE/api/search/readiness" > "$OUT/readiness.json"
curl -s "$BASE/api/slam/map3d" > "$OUT/map3d.json"
curl -s "$BASE/api/search/state" > "$OUT/search_state.json"
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

### 9.1 E/F 与 §13.3 之前必读：正前方软件不设防

**E/F 阶段和计划书 §13.3 走的是同一件事，而这件事有一个已知的、不会自己报错的缺口：
机器人正前方的碰撞保护在软件上是敞开的。持急停的人是正前方唯一的保护。**
这不是"雷达脏了先擦擦"的现场问题，是链路本身的结论，现场改配置改不掉：

- Nav2 的全局和局部 costmap **只有** `/go2w/lidar/scan` 一个观测源。
- 那条扫描线在**正前方一段连续扇区里一个回波都没有**。2026-09-28 当前栈实测：
  一次都没返回的束 **295/720（41%）**，最长连续盲区 **−68.0°..+69.0° = 137.0°**，
  有限回波只到 0.41~2.60 m（配置量程 6.20 m）。
  **这个跨度会跳**（同一天两次重测为 133.0° 与 137.0°，更早一次 113.5°），
  按 **±70°** 记，别按某一次的窄值记 —— 按窄值选站位会站进盲区。
- 原因不是方位而是高度：原始云正前方**有**点（3 m 内 2215 个），但约 91% 在碰撞带上沿
  `+0.052` 之上，带内 **0 点**。**雷达看得见前上方，看不见正前方的地面到膝盖这一段。**
- 因此 `/go2w/safety/front_clearance` 恒为 `inf`。而
  `app/live_robot/motion_bounds.py` 的 `evaluate_lidar_motion_readiness()`
  只对 `None`/`NaN` 拒绝——`inf` 既不是 `None` 也不是 `NaN`，`inf < minimum` 为假，
  **判为通过**。函数注释写着 fail closed，但它没覆盖"前方没有任何观测"这个真实故障态。
- `configs/go2w/lidar_preprocess.yaml` 里的 `front_stop_clearance_m: 0.6`
  **全仓库没有任何代码读取**，是个死值，别指望它。

反面证据（说明坏的只是前向那一段，不是整条带）：机器人**后方**碰撞带内实测有真实回波
（0.35~2.06 m、轮子高度），现场已确认那是一个真实物体。所以碰撞带本身是工作的，
前向的空是这一段特有的。

**因此 E/F 与 §13.3 现场必须做到：**

- 一人持急停，且**站位能直接看到机器人正前方**，不要站在它侧后。
- 只走直走廊短目标（§13.3 原文要求），**不要把"它会自己停"当成前提**去试贴墙。
- §13.3 那条"遇临时障碍时 Collision Monitor 能先减速、再停止"**不能用正前方的障碍去测**——
  正前方它测不出来，会得到一个假的通过。要测就用侧前方的障碍。
- 机身到墙面的 0.40 m 判据要**用尺量**，不要读 `front_clearance`：那里永远是 `inf`。

这条缺口的处置决定是"照常验收、人工兜底"，由用户 2026-09-28 现场拍板。要真正补上前向观测
（例如改接那颗独立的 Hesai），属于另一条线，**不要混进本次验收**——那会让"验收通过"变成
"验收一个刚改过、还没人验过的东西"。

E/F 阶段的 Nav2 自主运动授权命令（唯一入口，§6.3）：

~~~bash
cd /home/unitree/robotscene
export GO2W_AREA_CLEARED=I_HAVE_CLEARED_THE_AREA
bash scripts/go2w/start_nav2_autonomous_search.sh --enable-motion
~~~

如果不想在 shell 中设置环境变量，启动器会交互要求输入 I_CONFIRM。没有清场或没有急停时不要输入确认。

旧的 `start_autonomous_search_web.sh --enable-autonomous-motion` 是旧入口：它不启动
Nav2 bringup、坐标桥、速度平滑和 Collision Monitor，单独用它做 Nav2 自主运动只会
得到一条断掉的速度链，现场不要再用它。

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

## 11. 实体遥控器主动建图

这条链路和 §7 的 WASD 网页遥控**是两种不同的"遥控"，不要混用**：这里全程由
Unitree **实体遥控器**控制机器狗，软件一个运动指令都不发。operate 时人拿着遥控
器走，软件在后台周期采 D435 关键帧、识别物体，并把物体落到统一地图坐标。默认
使用二维 `map` 占据栅格；仍可显式选择三维 `pslam_map`。WebUI 只当地图查看器。

### 11.1 启动与停止

~~~bash
# 默认二维：需要 /go2w/lidar/mapping_scan；入口启动或复用二维 mapping
bash scripts/go2w/start_remote_control_mapping.sh --map-mode 2d

# 保留的三维模式
bash scripts/go2w/start_remote_control_mapping.sh --map-mode 3d

# 再拿起实体遥控器开始走
# ...

# 结束：先用实体遥控器把机器狗停稳，再停软件
bash scripts/go2w/stop_remote_control_mapping.sh
~~~

启动器会按 A–F 顺序做完：传感器预检 → 复用或启动所选地图栈 → 建本次语义会话
→ 起二维栅格桥或三维快照写入器 → 起语义侧车 → 以只读模式起 WebUI（默认
`http://127.0.0.1:8766`）。任一步失败都会明确报出来并指向对应日志。

**绝不传 `--enable-motion`。** 本项目里没有任何一条通往这条链路的命令接受它；
这条链路根本没有运动客户端。

### 11.2 安全边界

- 不发送 `/go2w/motion`、`cmd_vel`、`/wirelesscontroller`，不调用 Sport API、
  arm、stop 或 estop；不启动自主搜索、Nav2 或 frontier。语义侧车只在内存中维护
  一个只读的稀疏 PlaceGraph，用于“园区综合拓扑”显示真实经过的观察点，不参与运动。
- 不修改 `plain_slam` 算法；侧车只订阅位姿、只读 D435。
- **识别失败不影响实体遥控器和 SLAM。** 侧车崩了，遥感照旧、建图照旧，只是地图上
  的物体不再增加；`status.json` 会显示 `degraded` 或进程退出。
- 停止脚本只结束它自己拥有的进程：WebUI → 快照写入器 → 语义侧车 →（**只有**存在
  ownership marker 时才）`plain_slam`。SLAM 若是外部已有进程，会打印
  `reused, left running` 并保留。停止脚本不使用 `pkill ros2`、`killall python`
  或宽泛的 `pkill -f python`，也不下发任何运动停止 API。
- **停止软件不会停止机器狗运动**，操作员必须先用实体遥控器停稳。
- 只读硬门做在 API/WS 入口：`/api/control/*`、`/api/estop*`、`/api/llm/enable`、
  `/api/search/{start,resume,estop}` 一律 403，控制 WebSocket 输入一律拒收，
  退出时也不替操作员发停止命令。前端同时显示"实体遥控器模式，网页只读"并把控制区
  置灰。`/api/llm/disable` 是唯一放行的写入口 —— 它只会让软件更安静。

### 11.3 关键帧与阈值

按「位移 / 转角 / 最长等待」三选一留关键帧，先到先算；队列里的待推理帧上限 20，
满了先丢与相邻帧最像的冗余帧，再丢最旧的。检测器始终单实例连续消费，上一帧跑完
立刻取下一帧，没有固定 `sleep(5)`。

| 环境变量 | 默认 | 含义 |
| --- | --- | --- |
| `REMOTE_MAPPING_KEYFRAME_DISTANCE_M` | `0.75` | 行进多少米留一帧 |
| `REMOTE_MAPPING_KEYFRAME_YAW_DEG` | `15` | 累计转多少度留一帧 |
| `REMOTE_MAPPING_KEYFRAME_MAX_SECONDS` | `1.0` | 慢速或静止时的最长间隔 |
| `REMOTE_MAPPING_CAPTURE_POLL_SECONDS` | `0.2` | 多久判断一次要不要留帧 |
| `REMOTE_MAPPING_QUEUE_SIZE` | `20` | 待推理关键帧上限 |
| `REMOTE_MAPPING_RGBD_BASE_URL` | `http://192.168.123.18:8080` | D435 HTTP RGB-D 服务 |
| `REMOTE_MAPPING_WEB_PORT` | `8766` | 本模式 WebUI 端口（避开 §7 的 8765） |
| `REMOTE_MAPPING_MAP_MODE` | `2d` | 默认地图模式；也可用命令行 `--map-mode 3d` |
| `REMOTE_MAPPING_SNAPSHOT_PATH` | `outputs/remote_control_mapping/runtime/slam_map_3d.json` | 地图快照 |
| `REMOTE_MAPPING_META_PATH` | 同目录 `slam_map_meta.json` | 地图元数据（`map_epoch` 等，见 §8.1） |
| `REMOTE_MAPPING_OBJECTS_PATH` | 同目录 `semantic_objects.json` | 待显示物体（侧车按 epoch 写） |
| `REMOTE_MAPPING_SNAPSHOT_PERIOD_S` | `1.0` | 快照重写周期；三维页面按它轮询 |

园区感知与模拟泄漏参数集中在
`configs/go2w/semantic_mapping.yaml`：`scene_profile` 切换 `industrial_park` / `indoor`；
`detection_schedule` 配置危险/管道常驻词和普通地标轮换；
`hazard.visual_enhancement.hsv_ranges` 调整指定演示材料颜色；
`hazard.evidence_window_*` 调整近期多帧确认。`status.json` 的
`last_detection_terms` 会记录本轮实际询问的类别，便于核查轮换是否执行。

WebUI 的“物体拓扑”只显示真实物体；“园区综合拓扑”另外显示沿实际轨迹按距离或
转向采样的“观察点”，不会伪造物体填空。模拟信号的位置会明确区分“精确定位”、
“管道表面近似位置”、“仅关联宿主管道”和“位置待确认”。荧光绿色规则只用于指定
材料的受控演示，不代表通用工业泄漏或材质识别能力。

检测模型路径、检测阈值、D435 外参和语义合并半径继续用项目现有配置，没有第二套。

> **本模式的地图快照为什么不在 `outputs/autonomous_search/runtime/`：** 那个文件
> 同时被 WASD 演示台和自主搜索台各自的快照写入器写，两个写手会互相覆盖。本模式
> 换到自己的目录，前端协议和 `slam_map_3d.js` 完全不动，只是不再和演示台抢同一个
> 文件。WebUI 通过 `GO2W_SLAM_MAP_SNAPSHOT` 指向自己的快照。

### 11.4 产物与排障

~~~text
outputs/remote_control_mapping/runtime/events.jsonl          # 语义事件（每行一个 JSON）
outputs/remote_control_mapping/runtime/status.json           # 侧车状态（原子写）
outputs/remote_control_mapping/runtime/slam_map_3d.json      # 地图+物体快照
outputs/remote_control_mapping/runtime/events_<时间戳>.jsonl  # 上一轮的归档
outputs/remote_control_mapping/logs/semantic_mapper.log
outputs/remote_control_mapping/logs/snapshot_writer.log
outputs/remote_control_mapping/logs/web_server.log
~~~

每次启动都会把上一份 `events.jsonl` / `status.json` 改名归档再新建空的：`pslam_map`
在新 SLAM session 里可能重新定义，旧坐标系的物体绝不能混进新地图。

常见现象：

| 现象 | 先看哪里 |
| --- | --- |
| `state` 停在 `waiting`，没有物体 | `status.json` 的 `last_error`；多半是 D435 RGB-D 服务没起 |
| `state=degraded` | `semantic_mapper.log`；连续三轮失败才会进 degraded |
| 地图有物体但位置整体偏移 | 两机时钟不一致 —— 侧车检测到偏差超 5 s 会退回接收时刻，日志里会写；见 §11.6 |
| 地图上有旧会话的物体 | 不该发生；检查是否手工指定了别的 `events.jsonl` |
| 地图上冒出一整块"地面/地板"物体框 | `non_object_label` 没命中；RAM++ 会返回**复合写法**（`地板/地面`），整串比不中就见 `app/detectors/vocabulary.py` 的 `_COMPOUND_LABEL_SEPARATORS` |
| 侧车进程不在但 SLAM 正常 | 符合预期（§9.2 隔离），重启启动脚本即可 |
| 跑久了网页变卡、地图刷新变慢 | 先看 `snapshot_writer.log` 里 `snapshot written` 的**实际间隔**是不是比配置的 `period_s` 大；见 §11.7 |

真机验收按计划书 §10.3：起软件 → 确认网页显示只读 → 点控制确认被拒且机器狗不动
→ 静止 20 秒确认地图不自行漂移 → 用实体遥控器缓慢前进约 2 米并左右转向 → 确认
地图持续扩展 → 在椅子/桌子/门附近经过 → 等两三轮识别后确认 events 出现
`semantic_object_localized` 且地图上出现物体标记 → 先停稳机器狗再运行停止脚本。

### 11.5 单轮无运动冒烟检查（计划书 §10.2）

机器狗**静止**时先跑这个，确认"RGB-D → 检测 → 深度的 map_xyz → events"整条链是通的：

~~~bash
bash scripts/go2w/start_remote_control_mapping.sh --smoke
~~~

只跑一轮识别就退出，不启 SLAM、快照写入器和 WebUI，也不发送任何运动命令。
结果会打印状态、事件行数和一行 `VERDICT=`：

| VERDICT | 含义 |
| --- | --- |
| `MAPXYZ_OK` | 有物体拿到了 `map_xyz` —— §10.2 第三项判据已被实际检验 |
| `MAPXYZ_UNVERIFIED` | 检出了物体但一个都没落图 —— 判据**没被检验**，换个朝向重跑 |
| `EMPTY_SCENE` | 本轮没检出物体（可能是合法空结果）—— 判据同样没被检验 |

为什么值得单独跑：侧车本身需要 `rclpy`（只有 `/usr/bin/python3` 有），而检测器
子进程需要 `torch`（只有 conda 环境有），两者不是同一个解释器。手工照抄命令会漏掉
`.env` 注入，报"`GROUNDED_SAM_PYTHON` 无效且没有可用的回退解释器"，看着像检测器坏了。
`--smoke` 复用启动脚本 A 段那套环境注入，保证和正式启动走的是同一条路。

### 11.6 机器狗时钟：为什么必须对齐、怎么对齐

D435 的 HTTP 服务跑在机器狗上，**帧时间戳用的是机器狗的钟**；而落图要按时间戳去找
拍摄时刻的 SLAM 位姿。侧车因此有一致性门（`--max-clock-skew-seconds`，默认 5 s）：
偏差超了就**放弃拍摄时间戳**、改用接收时刻，状态里 `timestamp_source` 变成
`receive_time_clock_skew`、每个物体带一个 `clock_skew_seconds`。

功能不会报错，所以这是个安静的降级：2026-09-18 实测机器狗快 13.6 s，**每一帧**都在
降级，时间戳系统性偏移一个网络往返，而且这条 WARNING 会把别的问题盖住。
启动脚本 A 段（`A6`）每次都会打印时钟检查结果，超容差时给修法。

~~~bash
bash scripts/go2w/sync_robot_clock.sh          # 只读测量并报告（= status）
bash scripts/go2w/sync_robot_clock.sh apply    # 读取 -> 下推 -> 复测
~~~

`status` 的退出码可以直接当闸门用：容差内 `0`、超容差 `3`、测不到 `4`。
容差**不写死**，读的是 worker 里那个常量，避免两处各写一个 5.0 后走岔。

**改钟前必须先停 SLAM**（`apply` 默认会拒绝）：钟一跳变，任何跨机对齐时间轴的环节
都会看到一个大台阶，宿主侧 IMU 兜底适配器的步长限制会被卡住并留下时间轴脱钩，
已知后果是**地图重影**。确实要带着 SLAM 改就加 `--force`，并自己承担后果。

~~~bash
bash scripts/go2w/stop_plain_slam_mapping.sh
bash scripts/go2w/sync_robot_clock.sh apply
bash scripts/go2w/start_remote_control_mapping.sh
~~~

为什么不能让机器狗自己同步：它**没有外网**（NTP 和 HTTPS 都是 no route to host），
宿主也没有在 123/udp 上对外服务（只装了客户端 `timesyncd`），所以唯一可用的方向是
从宿主把时间推过去——这就是 `apply` 做的事。

**RTC 是坏的，软件修不了**，这一点很容易误判：备份电池没电，上电清零后从 0 开始数
（实测 RTC 计数 10189s ≈ uptime 10180s）。而且 `hwclock --systohc` 在这种情况下
**照样返回 0**、`hwclock --show` 还会静默回落到系统时钟把正确时间打印出来——两个
都是假绿。唯一可信的读法是 `timedatectl` 的 `RTC time` 行（`apply` 就是这么验的）。

所以重启后的恢复靠的是 systemd 自己的机制：timesyncd 启动时会用
`/var/lib/systemd/timesync/clock` 里存的时间戳把钟**往前拨**（只前拨、不回拨），
`apply` 会更新这个文件。注意落点是**上一次 apply 的时刻**、不是当前时刻——关机期间
它不会自己走，所以断电越久越慢，重启后（或长时间断电后）重新跑一次 `apply`。

### 11.7 长跑变卡：写入器每轮整读 events.jsonl

> **2026-10-08 之后这一条已经作废一半**：计划书 §2.3 起，快照写入器**不再读任何
> `events.jsonl`** —— 在线物体只由语义进程按 `map_epoch` 写 `semantic_objects.json`
> （见 §8.1），`IncrementalObjectRecordReader` 因为失去调用点已经删除
> （`load_object_records` 还在，只给离线回放与归档迁移用）。下面留的是
> 2026-09-18 当时的排查记录，仍然值得一读的是这个量级：**任何整读事件日志的代码
> 都会随时长线性变慢**（843 ms 那一项占 flush 的 84%），重写这类读口之前先量一遍。

遥控建图跑久了页面会变卡。2026-09-18 实测的账（`period_s` 被启动脚本覆盖成 **1.0**，
节点默认的 5.0 不是生效值）：

| 快照写入器每轮 flush 的开销 | 修前 | 修后 |
| --- | --- | --- |
| 读语义物体（整读 events.jsonl） | **843 ms** | **0.8 ms** |
| 轨迹过滤（当时 53 个位姿） | 47 ms | 47 ms |
| points 逐点 `round()` | 64 ms | 64 ms |
| `json.dumps` + 落盘 705 KB | 32 ms | 32 ms |

**843 ms 那一项随运行时长线性增长**（events.jsonl 只追加）：3.8 MB→80 ms、
9.6→220、19→391、38 MB→843 ms。它一项就占了 flush 的 84%，而 flush 的活压在
1.0 s 的周期上，rclpy 于是跳拍 —— 日志里 `snapshot written` 的实际间隔是
**1.8~2.0 s，比配置慢一倍**（判据的 `starve_timeout_s = 2 × period_s` 也建立在
这个周期上）。

根因是每轮都用 `load_object_records()` 把整份文件重新 `json.loads` 一遍 ——
拆开量过：**99% 时间在解析，去重+归一化只要 8 ms**。所以写端改用
`IncrementalObjectRecordReader`（`app/spatial/semantic_object_gate.py`）：只解析
新增的字节，判定语义与整读逐字一致。稳态每轮 0.8 ms，约 **900×**。

换源、截断、**同路径原地重写**（新会话截断重写、归档覆盖同名文件：inode 不变、
长度还可能变长）都要能认出来，最后一条靠比对已消费段末尾 256 字节，只看
inode/长度是认不出的。等价性校准覆盖这四种情况 + 边写边读含半行，全部逐字一致。
改这个文件前先跑一遍那个校准（见提交记录里的 `check_incremental.py` 思路），
"读出来的物体和整读一样"是唯一的判据。

仍未处理的第二项：**轨迹过滤是 O(点数 × 位姿数)**，位姿按 0.25 m 累积、没有上限
（实测 53 位姿 47 ms、300 位姿 287 ms、1200 位姿 1088 ms）。本次那种十几分钟的
走动离它变主因还很远，但走得很远的会话要留意；真要动它得先决定是否接受近似
（把位姿按距离抽稀会改变边界点的取舍）。

## 12. 离线开发和回归测试

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

## 13. 故障排查速查表

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
| 搜索三四步就结束 | 搜索 worker 日志、`outputs/live_runs/<session_id>/events.jsonl` 与 `webui_events.jsonl` | 查看真实 PLANNING_ERROR、候选异常或预算耗尽，不能把错误改成成功 |
| 地图刷新后旧房间消失 | map_revision、source_map_points、global_cached_voxels、web_display_points | 检查是否启动了旧 bridge 或把局部 scan 当全局 map |
| SSH 后台进程断开 | 是否在前台 SSH 会话直接启动 | 使用持久终端/tmux，或使用脚本自身的 setsid 启动器；不要混用旧进程 |

## 14. 代码和配置维护规则

- 每次更新先在控制主机运行 git diff --check、回归测试和 verify_fresh_deployment.sh。
- 运行时目录、相机帧、模型权重、.env、PID 和日志不应提交。
- 修改控制主机上的 ROS 源码后，要重新 colcon build --symlink-install 或按对应包构建，并重新 source workspace。
- 修改机器狗侧计划文件后使用 deploy_plan_to_robot.sh --dry-run，确认后再同步；不要直接覆盖机器狗上的标定、固件和历史产物。
- 不要通过放宽安全阈值、增大单次转角、删除 Place/Frontier、改写坐标系或吞掉异常来“修复”验收失败。
- 任何未做的现场测试都必须写成“未执行”，不能写成 HEALTHY、已完成或已验收。

## 15. 相关文档

- [AI_PROJECT_GUIDE.md](AI_PROJECT_GUIDE.md)：给新 AI/开发者的工程入口。
- [CODEX_DEPLOY.md](CODEX_DEPLOY.md)：全新电脑的简版部署入口。
- [GO2W_自主搜索_对象拓扑_固定世界三维建图_整改计划_20260901.md](GO2W_自主搜索_对象拓扑_固定世界三维建图_整改计划_20260901.md)：整改目标、实现范围和验收条目。
- [docs/GO2W_整改计划_与_交接书_20260903.md](docs/GO2W_整改计划_与_交接书_20260903.md)：当前整改、已完成代码工作、未完成真机验收和现场交接记录。
- [GO2W_真机自主搜索_详细修改计划书.md](GO2W_真机自主搜索_详细修改计划书.md)：真机搜索链路的详细计划。

## 16. 安全声明

本项目会在用户明确授权并通过启动器安全检查后调用真实机器狗运动链。任何现场运行都必须由具备机器狗操作能力的人员负责，保持实体急停可用，并根据实际环境重新判断风险。仓库中的脚本、测试和 WebUI 不能替代机器狗厂商安全规范、现场风险评估或人工急停。

## 17. 2026-09-04 三维建图修复状态

机器狗端本轮已经落地并准备上传的修复包括：

- LIO 参数落值：`acc_scale=0.95400`、`max_correspondence_dist=0.5`、`num_keyframes=30`、`convergence_th=0.01`、`num_max_iteration=10`；
- 回环参数调整：`num_loop_candidates=30`、`gicp_max_correspondence_dist=1.0`；
- `start_plain_slam_mapping.sh` 启动新会话时清理旧的 SLAM 进程树和 `/tmp/go2w_plain_slam`，避免历史位姿图和关键帧继续叠加；
- WebUI 对象拓扑保留 PLACE 中转节点及 `OBSERVED_FROM`/`CONNECTED_TO` 跨视角关系，地点合并距离调整为 0.6 m；
- 地图健康状态只在完整运动 episode 结束时判定，前端恢复后可继续工作；点云诊断工具兼容包含 FLOAT64 timestamp 的 PointCloud2 消息。

这些改动已经在机器狗端进行过实机观察，但截至 2026-09-04 仍不能宣称现场验收全部通过：

- 新栈单段约 30° 转向的 LIO 假平移曾降到约 0.042–0.154 m，360° 回转闭环的 M4 为 0.0369 m，回环边 M5 为 14；
- 连续动作合并为较大 episode 时仍出现过 `pslam_dxy=0.3684 m`，触发 `DEGRADED_LIO_DRIFT` 并冻结后续地图接收；
- M1/M2/M3/M6 按现行评估窗口仍未通过，不能自行修改评估窗口来制造通过结果；
- 本轮搜索因机器狗断电没有取得对象拓扑 T1–T5 终值；诊断探针撤除、重新编译和 60 秒收尾冒烟也必须单独完成。

因此，重新上电后必须先做网络和 `lidar_fresh` 检查，再读取每个 episode 的 `dyaw` 验证几何位移假说；涉及漂移门阈值、地面候选窗口或转速的修改必须由用户明确决定。现场工作记录应以当前交接文档和原始 JSON/日志为准，不能只看 WebUI 显示的 `HEALTHY` 字样。
