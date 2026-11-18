# Go2-W 外接 Ubuntu 笔记本：CfC 只读 ROS 2 接入指南

> 检查日期：2026-10-14  
> 使用场景：通过 USB-C 转网口适配器和网线，将个人 Ubuntu 笔记本接入 Go2-W 现有有线局域网，在笔记本上运行 CfC 神经网络并被动读取雷达、IMU、里程计等 ROS 2 数据。  
> 安全边界：禁止发送运动指令、调用运动服务、播放 rosbag 或修改机器狗、雷达、交换机及现有 x86 控制主机配置。

## 1. 检查结论

个人笔记本可以接入现有 `192.168.123.0/24` 网络并订阅 ROS 2 数据。推荐配置如下：

| 项目 | 推荐值 |
| --- | --- |
| 笔记本地址 | `192.168.123.50`（使用前必须检查冲突） |
| 子网掩码 | `255.255.255.0`，即 `/24` |
| 网关 | 留空 |
| DNS | 留空 |
| ROS 2 | Humble（推荐 Ubuntu 22.04） |
| RMW | `rmw_cyclonedds_cpp` |
| ROS Domain | `0` |
| DDS 网卡绑定 | 笔记本 USB 网卡的 `192.168.123.50` |

正确配置不会改变现有系统状态，但 DDS 订阅并不是物理意义上的完全静默：笔记本会发送发现报文和可靠传输确认报文，发布端也会向笔记本额外发送订阅的数据。第一阶段应优先读取 LaserScan、IMU 和 Odometry，避免同时持续订阅多个完整三维点云。

## 2. 当前网络拓扑

已确认的设备如下：

| 设备 | IP 地址 | 作用 |
| --- | --- | --- |
| 网络网关 | `192.168.123.1` | 现场局域网网关 |
| Go2-W Jetson | `192.168.123.18` | 相机服务及部分机载服务，接口为 `eth0` |
| Hesai PandarXT-16 | `192.168.123.20` | 外接 16 线三维激光雷达 |
| 现有 x86 控制主机 | `192.168.123.99` | ROS 2 Humble、Hesai 驱动、预处理、Nav2，接口为 `enp3s0` |
| Go2-W 运动主控 | `192.168.123.161` | Unitree DDS/RPC 状态与运动接口 |
| 个人 Ubuntu 笔记本 | 建议 `192.168.123.50` | CfC 推理及只读 ROS 2 订阅 |

现有 x86 控制主机链路实测为 `1000 Mb/s`、全双工。它的 `enp3s0` 同时配置了 `192.168.1.10/24` 和 `192.168.123.99/24`，项目因此显式将 CycloneDDS 绑定到 `192.168.123.99`，避免选错地址。

## 3. 网线应该插在哪里

仓库和当前网络中没有保存交换机型号、端口编号、VLAN 表或 LLDP 邻居，因此无法通过软件判断应插“3 号口”还是“5 号口”。

现场应按以下规则连接：

1. 找到连接现有 x86 控制主机 `enp3s0` 网线的交换机；或者找到同时连接 `192.168.123.18`、`.20`、`.99`、`.161` 的交换机。
2. 将笔记本接入该交换机的任意一个未使用的普通 LAN/access 端口。
3. 不要拔掉或替换任何现有网线。
4. 不要插入标为 `WAN`、`Internet` 或专用管理口的端口。
5. 如果是管理型交换机，新端口必须已经属于现有 `192.168.123.0/24` 所在的同一 VLAN。当前只读范围不允许修改 VLAN。
6. 如果没有空闲端口，应停止操作并由现场网络管理员扩展端口，不能擅自串接或改线。

如需判断具体物理端口，必须查看交换机、端口标签和现有布线照片；仅凭 IP 地址无法反推出物理端口号。

## 4. 雷达与可读取数据

### 4.1 内置雷达

机器狗内置雷达为 **Unitree 4D LiDAR L2**。

| 话题 | 类型 | 当前实测状态 | 来源 |
| --- | --- | --- | --- |
| `/utlidar/cloud` | `sensor_msgs/msg/PointCloud2` | 约 15.4 Hz | Go2-W 直接发布 |
| `/utlidar/imu` | `sensor_msgs/msg/Imu` | 约 250 Hz | Go2-W 直接发布 |
| `/utlidar/robot_odom` | `nav_msgs/msg/Odometry` | 有发布端声明，但当前未测到数据 | Go2-W 直接发布 |
| `/utlidar/robot_pose` | `geometry_msgs/msg/PoseStamped` | 有发布端声明，但当前未测到数据 | Go2-W 直接发布 |

### 4.2 外接雷达

外接雷达为 **Hesai PandarXT-16**：

- 雷达地址：`192.168.123.20`
- 点云 UDP：`2368`
- PTC：`9347`
- 数据以单播方式发送至现有 x86 控制主机 `192.168.123.99`
- ROS 2 点云：`/hesai/pandarxt16/points_raw`
- 默认约 10 Hz

Pandar 原始 UDP 数据只发给 `.99`，所以笔记本仅接入交换机后不能直接截获该单播 UDP。笔记本可以通过 ROS 2 订阅现有 x86 驱动发布的 `/hesai/pandarxt16/points_raw`，但这会依赖现有 x86 主机继续运行，并增加一份点云传输。

Pandar 当前外参和时间同步仍未完成正式验证，只允许用于诊断、数据采集和建图研究，不能用于运动安全放行。

### 4.3 现有 x86 主机加工后的话题

以下话题可被笔记本订阅，不需要修改或登录现有 x86 主机，但要求该主机上的对应进程保持运行：

| 话题 | 类型 | 当前实测状态 |
| --- | --- | --- |
| `/go2w/lidar/scan` | `sensor_msgs/msg/LaserScan` | 约 15.4 Hz |
| `/go2w/odom/fused` | `nav_msgs/msg/Odometry` | 约 20 Hz |
| `/go2w/lidar/cloud_filtered` | `sensor_msgs/msg/PointCloud2` | 过滤后点云 |
| `/tf` | `tf2_msgs/msg/TFMessage` | 动态坐标变换 |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | 静态坐标变换 |

CfC 第一阶段建议使用：

```text
/go2w/lidar/scan
/utlidar/imu
/go2w/odom/fused
/tf
/tf_static
```

如果要求现有 x86 控制主机关机后笔记本仍能独立工作，则不能依赖 `/go2w/lidar/scan` 和 `/go2w/odom/fused`。届时需要：

1. 从 `/utlidar/cloud` 在笔记本生成 LaserScan；
2. 安装 Unitree 自定义消息；
3. 读取约 10 Hz 的 `/lf/sportmodestate`；
4. 在笔记本上自行转换并发布本地 Odometry；
5. 保证该转换节点仍然不创建任何运动控制发布者或客户端。

## 5. Ubuntu 笔记本网络配置

以下操作仅修改个人笔记本。

### 5.1 找到 USB 网卡名称

插入 USB-C 网卡和网线后执行：

```bash
ip -br link
```

USB 网卡通常类似：

```text
enx001122334455
```

也可能是 `enp...`。可以在插拔 USB 网卡前后分别运行一次命令，通过新增的接口确定名称。

以下示例假定网卡名为：

```bash
GO2W_IF=enx001122334455
```

确认物理链路：

```bash
ip link show "$GO2W_IF"
```

正确连接后应看到 `LOWER_UP`。

### 5.2 检查 IP 地址冲突

安装 `arping`：

```bash
sudo apt update
sudo apt install iputils-arping
```

检查候选地址：

```bash
sudo arping -D -I "$GO2W_IF" -c 5 192.168.123.50
```

- 如果没有收到响应，`.50` 大概率未被占用，可以继续；
- 如果收到响应，不要使用 `.50`，依次检查 `.51`、`.52` 等；
- 不要使用 `.1`、`.18`、`.20`、`.99` 或 `.161`。

### 5.3 使用 Ubuntu 图形界面配置

进入：

```text
设置 → 网络 → USB 有线连接旁边的齿轮 → IPv4
```

配置：

```text
IPv4 方法：手动
地址：192.168.123.50
子网掩码：255.255.255.0
网关：留空
DNS：留空
IPv6：禁用
```

如果界面提供“仅将此连接用于其网络中的资源”，应勾选。不要给该有线连接设置默认网关，这样笔记本访问互联网仍然走 Wi-Fi。

保存后断开并重新连接 USB 有线网络。

### 5.4 验证网络

```bash
ip -4 addr show dev "$GO2W_IF"
ip route get 192.168.123.18
ping -c 3 192.168.123.18
ping -c 3 192.168.123.20
ping -c 3 192.168.123.99
```

路由结果应包含：

```text
dev enx... src 192.168.123.50
```

如果 `.18`、`.20`、`.99` 都不通，依次检查：

1. USB 网卡是否为 `LOWER_UP`；
2. 是否使用了 `/24`；
3. 网线是否插入正确交换机；
4. 交换机端口是否属于相同 VLAN；
5. 是否存在 IP 冲突。

## 6. 安装 ROS 2 和 CycloneDDS

本指南推荐 Ubuntu 22.04 + ROS 2 Humble，以匹配现有 x86 控制主机。

检查 Humble：

```bash
test -f /opt/ros/humble/setup.bash && echo "ROS 2 Humble 已安装"
```

如果已经安装：

```bash
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp ros-humble-ros2bag
```

如果 `/opt/ros/humble/setup.bash` 不存在，先按照 ROS 2 官方文档安装 `ros-humble-ros-base`：

<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

如果笔记本是 Ubuntu 24.04，不建议直接把当前步骤替换成 Jazzy 后假定一定兼容。优先使用 Ubuntu 22.04 环境、虚拟机或容器运行 Humble，保持与现有系统相同的 ROS 版本和 RMW。

## 7. CycloneDDS 配置

### 7.1 创建配置文件

```bash
nano ~/go2w_cyclonedds.xml
```

粘贴：

```xml
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface
          address="192.168.123.50"
          priority="default"
          multicast="default" />
      </Interfaces>
      <AllowMulticast>spdp</AllowMulticast>
      <MaxMessageSize>14720B</MaxMessageSize>
      <FragmentSize>1344B</FragmentSize>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>
    </Discovery>
    <Internal>
      <SocketReceiveBufferSize min="16MB" max="16MB" />
      <SocketSendBufferSize min="16MB" max="16MB" />
      <Watermarks>
        <WhcHigh>16MB</WhcHigh>
      </Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
```

如果最终使用的不是 `.50`，必须把 XML 中的地址改成笔记本实际地址。

在 `nano` 中按 `Ctrl+O`、回车保存，再按 `Ctrl+X` 退出。

### 7.2 设置当前终端环境

每次打开新的 ROS 终端执行：

```bash
source /opt/ros/humble/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/go2w_cyclonedds.xml"

ros2 daemon stop || true
```

不要在笔记本上直接使用本项目的 `scripts/go2w/setup_environment.sh`。该脚本面向现有控制主机，会自动寻找 `.99` 所在网卡并生成控制主机配置，不适合个人笔记本。

### 7.3 防火墙

检查 Ubuntu 防火墙：

```bash
sudo ufw status
```

如果显示 `active`，只允许 Go2-W 网段从 USB 网卡进入：

```bash
sudo ufw allow in on "$GO2W_IF" from 192.168.123.0/24 comment 'Go2W ROS2 DDS'
```

该操作只修改个人笔记本。

## 8. 只读验证步骤

### 8.1 查看话题

```bash
ros2 topic list | sort
```

检查端点：

```bash
ros2 topic info /utlidar/cloud -v
ros2 topic info /utlidar/imu -v
ros2 topic info /go2w/lidar/scan -v
ros2 topic info /go2w/odom/fused -v
```

### 8.2 测量频率

```bash
ros2 topic hz /utlidar/imu
ros2 topic hz /go2w/lidar/scan
ros2 topic hz /go2w/odom/fused
```

当前现场参考值：

```text
/utlidar/cloud       约 15.4 Hz
/utlidar/imu         约 250 Hz
/go2w/lidar/scan     约 15.4 Hz
/go2w/odom/fused     约 20 Hz
```

### 8.3 查看单条数据

```bash
ros2 topic echo --once --qos-profile sensor_data /utlidar/imu
ros2 topic echo --once --qos-profile sensor_data /go2w/odom/fused
```

不要直接在终端持续打印 `/utlidar/cloud` 或 Pandar 点云；点云数组很大，会大量占用终端和 CPU。对于点云只使用 `topic info`、`topic hz` 或自己的限频订阅节点。

## 9. CfC 只读接入建议

第一阶段 CfC 节点建议输入：

```text
/go2w/lidar/scan       局部障碍距离
/utlidar/imu           角速度、线加速度
/go2w/odom/fused       当前位姿和速度
/tf、/tf_static        坐标关系
```

第一阶段应满足：

1. CfC 节点只创建 subscription；
2. 不创建 Twist、Sport Request、Motion Action publisher/client；
3. 输出速度只记录到本地文件或仅在进程内计算；
4. 不接入 `/go2w/*cmd_vel`；
5. 不调用 `/go2w/motion`；
6. 先比较 CfC 推理结果与现有系统状态，不执行；
7. 对输入时间戳、丢帧、NaN、推理超时和隐藏状态重置进行记录。

读取 `/go2w/lidar/scan` 和 `/go2w/odom/fused` 不会修改现有 x86 主机，但会依赖该主机继续发布这些话题。

## 10. 对现有系统的影响

### 10.1 正常情况下

- 新增一个 DDS participant；
- 增加少量 SPDP 发现流量；
- 每个被订阅话题增加一个数据接收端；
- 不改变机器狗模式、租约、急停或运动状态；
- 不修改现有主机、雷达和机器人配置。

### 10.2 主要风险

| 风险 | 后果 | 避免方法 |
| --- | --- | --- |
| IP 地址冲突 | 现有设备掉线或通信异常 | 配置前使用 `arping -D` |
| 有线连接设置了默认网关 | 笔记本互联网路由异常 | 网关和 DNS 留空，设置 `never-default` |
| 同时订阅多个原始点云 | 增加网络和发布端 CPU 负载 | 第一阶段仅用 LaserScan、IMU、Odom |
| 可靠订阅端处理过慢 | 发布端维护更多队列和重传状态 | CfC 使用小队列、及时丢弃旧帧 |
| 运行 Unitree 示例程序 | 可能创建控制 publisher/client | 不安装或不运行运动示例 |
| `ros2 bag play` | 可能重放控制话题 | 严禁在机器人网络中执行 bag play |

当前链路为千兆全双工，增加 LaserScan、IMU 和 Odometry 订阅通常负担很小，但完整 PointCloud2 并非零成本。

## 11. 只读红线

禁止执行：

```text
ros2 topic pub ...
ros2 service call ...
ros2 action send_goal ...
ros2 bag play ...
```

禁止接触或发布：

```text
/api/sport/request
/go2w/motion
/go2w/arm
/go2w/manual_cmd_vel
/go2w/nav2_cmd_vel
/go2w/search_cmd_vel
/go2w/cmd_vel_selected
/utlidar/client_cmd
/utlidar/mapping_cmd
```

禁止：

- 修改雷达 IP、转速、PTP、GPS 或 PTC 参数；
- 修改交换机 VLAN；
- 修改现有 x86 控制主机；
- 重启机器狗、雷达、相机或现有 ROS 2 节点；
- 获取 Sport lease；
- 复位急停；
- 通过自动运动采集动态数据。

## 12. 故障排查

### 能 ping，但看不到 ROS 2 话题

检查：

```bash
echo "$ROS_DOMAIN_ID"
echo "$RMW_IMPLEMENTATION"
echo "$CYCLONEDDS_URI"
cat ~/go2w_cyclonedds.xml
sudo ufw status
```

应分别看到：

```text
0
rmw_cyclonedds_cpp
file:///home/<用户名>/go2w_cyclonedds.xml
```

然后重新启动个人笔记本上的 ROS CLI daemon：

```bash
ros2 daemon stop || true
ros2 topic list
```

### 只能看到部分话题

- `/utlidar/*` 主要来自机器狗，可直接读取；
- `/go2w/lidar/scan` 和 `/go2w/odom/fused` 由现有 x86 主机发布，该主机对应进程不运行时不会出现；
- `/hesai/pandarxt16/points_raw` 由 `.99` 上的 Hesai 驱动发布；
- `/lf/sportmodestate` 使用 Unitree 自定义消息，没有安装消息包时可能只能看到话题名而不能解析内容。

### 点云丢包或频率明显偏低

1. 确认 USB-C 网卡协商到千兆：

   ```bash
   sudo ethtool "$GO2W_IF" | grep -E 'Speed|Duplex|Link detected'
   ```

2. 避免通过百兆 USB 网卡、扩展坞或劣质网线；
3. 暂停点云可视化，只测 `ros2 topic hz`；
4. 先停止 Pandar ROS 订阅，只保留 L2；
5. CfC 使用最新帧覆盖策略，不堆积历史点云。

## 13. 验收清单

- [ ] 网线接入现有机器人网络交换机的空闲 LAN/access 端口；
- [ ] 未拔除或改接现有网线；
- [ ] USB 网卡显示 `LOWER_UP`；
- [ ] `192.168.123.50` 已通过 `arping -D` 冲突检查；
- [ ] 笔记本配置为 `192.168.123.50/24`；
- [ ] 有线网关和 DNS 留空；
- [ ] 能 ping 通 `.18`、`.20`、`.99`；
- [ ] ROS 2 Humble 和 `rmw_cyclonedds_cpp` 已安装；
- [ ] `ROS_DOMAIN_ID=0`；
- [ ] CycloneDDS 绑定笔记本自己的 `.50`；
- [ ] 能看到 `/utlidar/cloud` 和 `/utlidar/imu`；
- [ ] 能看到 `/go2w/lidar/scan` 和 `/go2w/odom/fused`，或已明确接受对现有 x86 主机的运行依赖；
- [ ] CfC 节点不创建任何运动 publisher、service client 或 action client；
- [ ] 未运行 `ros2 bag play` 或 Unitree 运动示例。

完成以上项目后，笔记本即可作为 CfC 的只读观测计算节点接入，不改变现有 Go2-W 控制链。
