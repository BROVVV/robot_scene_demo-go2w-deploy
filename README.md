# RobotScene — GO2-W 机器狗部署版

> **仓库身份：Unitree GO2-W Jetson 生产部署快照。**
> 这是机器狗版本，与 Ubuntu x86 电脑开发仓库明确分离。

- 机器狗部署仓库：<https://github.com/BROVVV/robot_scene_demo-go2w-deploy>
- 电脑开发仓库：<https://github.com/BROVVV/robot_scene_demo>
- 当前机器狗发布：`20261117_24`
- 回退发布：`20261117_23`
- 发布 manifest SHA256：`08dd9c81ca286431fdf29b464bbff95cedb06ddfc3e307eb65d17614304cba6e`
- 状态日期：2026-11-18

## 这是机器狗版本

本仓库是经过验收的机器狗不可变发布的净化源码快照，不是电脑端 Humble 开发环境。机器狗私有配置、运行缓存、Python 缓存和嵌套仓库元数据不会上传 GitHub。

| 项目 | 本仓库：机器狗部署版 | 电脑开发版 |
| --- | --- | --- |
| 平台 | Jetson aarch64 / JetPack 5.1.1 | Ubuntu x86_64 |
| ROS / Python | ROS 2 Foxy / Python 3.8 | ROS 2 Humble / Python 3.10 |
| 主要用途 | GO2-W 本地运行 | 开发、测试、构建发布 |
| 地图 | 二维占据栅格 | 可包含实验配置 |
| WebUI | 二维相机、地图、拓扑和状态 | 开发与调试 |
| Web 3D / 点云 | 关闭 | 实验代码可能保留 |
| 运动 | 默认关闭 | 默认不发送真机动作 |

不要把电脑端的 `.venv`、`.runtime_venv`、`build/`、`install/`、日志和运行输出复制到本仓库或 Jetson 生产目录。

## 机器狗上的路径

```text
Git 工作区（维护用）：
/home/unitree/robotscene

当前不可变发布：
/home/unitree/robotscene_releases/20261117_24

上一回退发布：
/home/unitree/robotscene_releases/20261117_23

运行管理器：
/home/unitree/robotscene_runtime/bin/robot_project.py

运行状态与会话：
/home/unitree/robotscene_runtime/service/project2d

验收交接：
/home/unitree/robotscene_runtime/logs/acceptance_20261113_delivery/handoff_current
```

运行中的 Release24 保持不可变。本 README 的 GitHub 展示更新不会修改其 manifest 或运行文件。仓库根目录的 `GITHUB_SNAPSHOT_PROVENANCE.json` 记录来源、原始 manifest SHA 和 GitHub 净化边界。

## 当前生产档

- `GO2W_PERCEPTION_MODE=2d`
- Web 二维地图：开启
- Web 三维地图：关闭
- Web 点云桥：关闭
- D435 深度：保留，用于目标空间定位与局部几何
- 日常模式：`teleoperated_2d_mapping`
- 日常运动：`disabled`
- 导航执行授权：默认 `false`
- systemd：`robotscene-project-2d.service`

日常二维页面：

```text
无线：http://192.168.3.62:8766/
原生有线：http://192.168.123.18:8766/
```

网络地址以现场实际分配为准。

## 管理命令

```bash
PROJECT=/home/unitree/robotscene_runtime/bin/robot_project.py

# 查看当前 release、模式、电量和进程
python3 "$PROJECT" status

# 启动、停止
python3 "$PROJECT" start
python3 "$PROJECT" stop --wait

# 日常二维遥控建图模式
python3 "$PROJECT" mode-mapping

# 自主搜索只规划模式
python3 "$PROJECT" mode-search

# 回退上一不可变发布
python3 "$PROJECT" rollback
```

systemd 状态：

```bash
systemctl status robotscene-project-2d.service
systemctl is-enabled robotscene-project-2d.service
journalctl -u robotscene-project-2d.service -n 200 --no-pager
```

不要绕过管理器直接在不可变 release 中改代码，也不要同时启动两套相机桥、LiDAR 预处理、Nav2 或 Web 服务。

## 已验收能力

截至2026-11-18：

- D435 USB 3、1280×720 新鲜画面；
- Hesai LiDAR 二维扫描与二维地图；
- YOLOE-Seg 开放词汇目标识别；
- 可选 VLM 目标理解与复核；
- 目标深度、相机坐标和地图坐标链路；
- 二维语义对象、对象拓扑和搜索历史回放；
- 自主搜索开始、暂停、恢复、停止和急停；
- Release23 约0.117 m 受监督 Nav2 实体短程；
- Release24 LiDAR、定位与 Wi-Fi 故障闭锁及恢复；
- 办公椅与垃圾桶两个不同的非运动搜索会话；
- systemd 开机托管、状态检查和上一版本回退；
- 电脑开发项目保持独立可运行。

## 尚未授权的能力

**不能因为项目已经迁移完成，就直接进行无人监管的大范围自主搜索。**

当前实体验收只覆盖小范围、低速、有人监督的短程动作。跨房间、跨楼层、楼梯/玻璃/斜坡环境和长时间无人搜索需要另外完成：

- 地图导航授权和禁行区检查；
- 地理围栏、最大速度、单目标距离和总里程限制；
- 2 m → 5 m → 单房间 → 跨房间分级验收；
- 低电量停止或返航策略；
- 无线中断、传感器异常和人员进入时的持续安全行为；
- 更长时间的资源、温度和模型延迟检查。

## 项目组成

```text
app/                         感知、推理、语义地图、搜索和 WebUI
configs/go2w/                机器狗生产配置
ros2_ws/src/                 Foxy ROS 节点、launch 与 Nav2
unitree_go2w_control/         Unitree 运动控制与安全接口
compat/foxy_nav2/            Foxy 兼容源码
requirements/                Jetson Python/ML 锁定依赖
deploy/                      systemd、运行模板与管理资源
scripts/go2w/                构建、启动、停止、诊断和验收
tests/                       Foxy 合同与安全门测试
release_manifest.json        机器狗原始发布清单（来源证明）
GITHUB_SNAPSHOT_PROVENANCE.json  GitHub 净化快照说明
```

## 更新流程

1. 在电脑开发仓库完成修改和测试。
2. 生成新的不可变 release 和完整 manifest。
3. 在 Jetson 上使用 Foxy/Python 3.8 原生构建。
4. 先运行只读检查，再停止旧服务并原子切换。
5. 验证 D435、LiDAR、二维页面、停止和回滚。
6. 保留上一发布，禁止原地覆盖生产 release。

GitHub 仓库中的 README 是项目页说明；机器狗运行时以 release manifest 和运行管理器选择结果为准。

## 安全规则

- 日常模式运动必须保持关闭。
- 只规划模式不能启动速度控制链。
- 实体动作必须由设备所有者明确授权，现场人员手持遥控器。
- LiDAR、定位、急停、遥控接管、命令看门狗、速度和里程门不得删除。
- 不允许自动重试失败的实体动作。
- 地图文件存在不代表获得大范围导航授权。
- 不要在站立、充电或线缆未整理时启动运动。

## 凭据与运行数据

不得提交：

- `.env`、API Key、GitHub Token、SSH 密码；
- SSH 私钥、无线密码或 NetworkManager 连接；
- 相机录像、真实地图、运行日志和现场数据；
- `build/`、`install/`、虚拟环境和模型权重。

发布凭据只应通过临时凭据助手或安全的 CI Secret 提供，不能写入文件、提交、日志或命令历史。

