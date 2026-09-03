# Codex 全新电脑部署入口

本文件是把 GitHub 链接交给 Codex 后的首要入口。目标是从一个干净的 Ubuntu 22.04
x86_64 工作站恢复项目的软件能力，同时保持真机运动默认关闭。

## 给 Codex 的直接指令

```text
请克隆这个仓库，完整阅读 CODEX_DEPLOY.md、AI_PROJECT_GUIDE.md 和 README.md，
运行 bash scripts/bootstrap_fresh_machine.sh --profile=full，修复任何环境差异，
再运行 bash scripts/verify_fresh_deployment.sh --profile=full。
不要把任何 API Key 提交到 Git；没有操作员、遥控急停和清空场地时不得启动真机运动。
```

## 一键安装

```bash
git clone https://github.com/BROVVV/robot_scene_demo-go2w-deploy.git
cd robot_scene_demo-go2w-deploy
bash scripts/bootstrap_fresh_machine.sh --profile=full
```

安装脚本是幂等、非运动的。它会：

- 创建仓库内 `.venv` 并安装 WebUI、LLM、Mock、测试依赖；
- 创建被 Git 忽略的 `.env`，但不会写入或索取密钥；
- 安装 ROS 2 Humble/Go2-W 主机依赖；
- 按固定提交获取 Unitree ROS2 和 Hesai ROS2 驱动；
- 编译 Unitree 消息、Go2-W Action/急停控制和项目 ROS2 工作区；
- 按固定提交获取 Grounded-SAM-2，安装 GroundingDINO/SAM2 并下载公开权重；
- 运行部署自检。安装过程不会启动 ROS 节点或给机器人发送指令。

可选档位：

```bash
# 仅 WebUI、Mock、SiliconFlow API 路径；不安装 ROS/GPU 本地检测
bash scripts/bootstrap_fresh_machine.sh --profile=core

# 增加 Go2-W、ROS2、Unitree、Hesai；不安装 Grounded-SAM-2
bash scripts/bootstrap_fresh_machine.sh --profile=go2w

# 软件能力最完整，默认档位
bash scripts/bootstrap_fresh_machine.sh --profile=full
```

## 本机私密配置

编辑 `.env`，至少按实际需要填写：

```dotenv
SILICONFLOW_API_KEY=
```

`.env`、模型权重、第三方仓库、ROS 构建目录、相机图像、搜索记录、日志、PID 和锁均被
Git 忽略。它们由 bootstrap 或运行时生成。不要把 GitHub Token 或模型 API Key 写入
`.env.go2w`、文档、脚本、测试或提交历史。

## 验证与启动

```bash
bash scripts/verify_fresh_deployment.sh --profile=full

# 完全离线 Mock WebUI，不会运动
bash scripts/go2w/start_autonomous_search_web.sh --mock
# 浏览器：http://127.0.0.1:8765
```

连接真机前：

```bash
bash scripts/go2w/check_go2w_ready.sh --json
bash scripts/go2w/start_motion_control.sh

# 只读真机 WebUI，不授权自主运动
bash scripts/go2w/start_autonomous_search_web.sh

# 仅在操作员持遥控器、场地清空后运行
bash scripts/go2w/start_autonomous_search_web.sh --enable-autonomous-motion
```

## 无法由 GitHub 自动提供的外部条件

完整真机功能仍需要实体设备和现场配置：Go2-W、D435/Jetson HTTP RGB-D 服务、对应
以太网地址、可用 GPU/驱动（本地 Grounded-SAM）、SiliconFlow 账号密钥，以及操作员
遥控急停。仓库提供这些组件的主机代码、安装入口、安全门和诊断，但不会绕过硬件、
账号或安全确认。

详细架构见 [AI_PROJECT_GUIDE.md](AI_PROJECT_GUIDE.md)，所有运行方式和故障排查见
[README.md](README.md)。
