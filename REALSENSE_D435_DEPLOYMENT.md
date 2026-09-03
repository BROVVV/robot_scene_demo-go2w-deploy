# Go2-W 机载 Intel RealSense D435 RGB-D 部署文档

日期：2026-08-18（Asia/Shanghai）
状态：已部署并验收（机器狗 USB 接入 D435，本机实时查看彩色/深度/设备信息）

## 1. 背景与目标

- 机器狗：Unitree Go2-W（内部 Ubuntu 20.04.5 LTS，aarch64/Jetson，SSH: `unitree@192.168.123.18`）。
- 新硬件：Intel RealSense D435（序列号 `044322072021`，固件 `5.17.3.10`）经 USB 3.0 接入机器狗。
- 目标：在本机（192.168.123.99，enp6s0 直连机器狗）实时查看彩色画面、深度、
  设备内参/外参/传感器状态等全部信息。

## 2. 机器狗功能检查（重启后）

`bash scripts/go2w/check_go2w_ready.sh` → **state: ready**：

| 检查项 | 结果 |
|---|---|
| network / sport_mode / odom / camera / lidar_fresh | ✅ |
| motion_action / arm_service / emergency_stop | ✅ |
| spool_bundle / llm_api_key | ✅ |
| rotation_clearance_valid | ⚠️ false（项目已知常闭安全门：需人工物理 360° 验证，非故障） |

## 3. 架构

```
┌─ 机器狗 (192.168.123.18, Ubuntu aarch64) ─────────────┐
│  D435 ─USB3.0─> pyrealsense2 2.55.1 (aarch64 wheel)   │
│       │  采集线程: 640x480@30 color(bgr8)+depth(z16)  │
│       │  深度对齐彩色 + jet 伪彩 + 中心距离叠加          │
│       ▼                                              │
│  realsense_stream.py :8080 (systemd 服务, 开机自启)    │
└──────────────┬───────────────────────────────────────┘
               │ 千兆网线 enp6s0 (192.168.123.0/24)
┌──────────────▼───────────────────────────────────────┐
│  本机 (192.168.123.99)                                 │
│   • 浏览器: http://192.168.123.18:8080/ （推荐）        │
│   • OpenCV 工具: view_realsense.py                     │
└──────────────────────────────────────────────────────┘
```

## 4. 部署步骤（可复现）

1. **机器狗侧安装 pyrealsense2**（Python 3.8 / aarch64，需先在本机下载 wheel 再 scp，
   机器狗无外网）：
   ```bash
   # 本机：从 PyPI 获取 cp38 manylinux2014_aarch64 wheel（版本 2.55.1）
   # 机器狗：
   pip3 install --user /tmp/pyrealsense2-2.55.1.6486-cp38-cp38-manylinux2014_aarch64.whl
   echo 123 | sudo -S pip3 install /tmp/pyrealsense2-2.55.1.6486-cp38-cp38-manylinux2014_aarch64.whl
   ```
   wheel 自带头 librealsense 动态库，无需系统编译。

2. **udev 规则**（Intel 官方 `config/99-realsense-libusb.rules`，`MODE:=0666`）：
   ```bash
   echo 123 | sudo -S cp 99-realsense-libusb.rules /etc/udev/rules.d/
   echo 123 | sudo -S udevadm control --reload-rules && echo 123 | sudo -S udevadm trigger
   ```

3. **部署流服务（一键）**：
   ```bash
   bash scripts/go2w/start_realsense_stream.sh install   # 上传脚本 + 安装 systemd 服务
   bash scripts/go2w/start_realsense_stream.sh status    # 检查
   ```
   服务：`realsense-stream.service`（`User=unitree`，`Restart=always`，开机自启）。

4. **本机查看**：
   - 浏览器打开 http://192.168.123.18:8080/ （推荐，彩色+深度+全部信息同页）
   - 或 `bash scripts/go2w/start_realsense_stream.sh view` 后按提示运行 OpenCV 工具

## 5. HTTP 端点（机器狗 :8080）

| 端点 | 说明 |
|---|---|
| `/` | HTML 查看页：彩色流 + 深度伪彩流 + 设备/内参/外参/传感器信息 + 一键快照 |
| `/color` | MJPEG 彩色流（640x480@30，JPEG q82） |
| `/depth` | MJPEG 深度流（深度对齐彩色、jet 伪彩、中心十字+距离标注） |
| `/depth_raw` | 原始 16-bit 深度 PNG（z16，单位毫米） |
| `/info.json` | 设备信息、彩色+深度内参、depth↔color 外参、传感器选项、帧统计 |
| `/health` | 服务健康：`ok/streaming/age_s/fps` |
| `/rgbd/latest.json` | 原子 RGB-D 帧元数据（frame_id/color_url/depth_url/intrinsics） |
| `/rgbd/frame/<id>/color.jpg` | 与 depth.png 同一 frameset 的彩色 JPEG |
| `/rgbd/frame/<id>/depth.png` | 与 color.jpg 同一 frameset 的 16-bit 深度 PNG |
| `/snapshot` | 保存一帧快照到机器狗 `/home/unitree/realsense_snapshots/<时间戳>/` |
| `/snap/` | 快照目录浏览/下载 |

## 6. 验证结果（2026-08-18）

- 设备枚举：D435 / 044322072021 / fw 5.17.3.10 / USB 3.2 ✅
- 彩色流 640x480@30fps、深度流 640x480@30fps，93.4% 有效深度像素（p50=851mm）✅
- 内参：color fx=607.99 fy=608.32 ppx=318.60 ppy=238.30；depth fx=383.56 ppx=323.57 ✅
- depth→color 外参平移 [0.0148, 0.0, 0.00008] m（相机出厂标定）✅
- 快照 GET/POST、文件下载、systemd 开机自启 ✅

## 7. 已知限制与后续

1. **相机相对机器狗 base_link 的外参未标定**：当前 depth↔color 为相机内部标定；如需深度点云
   投影到 robot frame 或与 LiDAR 融合，需棋盘格/多场景外参标定（参考本目录
   `GO2W_REAL_ROBOT_DEPLOYMENT.md` 的 RGB-LiDAR 外参工作流）。
2. **机器狗无 RTC/NTP**：重启后时钟回退，开机后可同步：
   ```bash
   ssh unitree@192.168.123.18 "echo 123 | sudo -S date -s @$(date +%s)"
   ```
3. **ROS2 接入（可选）**：机器狗有 ROS Foxy、本机有 Humble；可扩展节点把 color/depth 发布为
   `/camera/color/image_raw`、`/camera/depth/image_raw` + CameraInfo，经 CycloneDDS 与本机直连。
4. 分辨率可调：`systemctl edit realsense-stream` 修改 ExecStart 追加
   `--width 1280 --height 720`；`/health` 的 `age_s` 可用于监控流中断。
5. 相机 USB 供电对线材敏感，出现 `age_s` 增大时先检查 USB 连接。

## 7.5 取景范围（FOV）说明 —— 为什么“只能看到正前方一小块”

**核心物理事实：D435 的最大硬件视场是固定的，但低分辨率模式会“更窄”。**
- 满宽度视场：彩色（RGB）约 **69.4°(H) × 42.5°(V)**、深度（Depth）约 **87°(H) × 58°(V)**。
- **重要：640×480 等低分辨率是中心裁剪**，实测彩色 HFOV 只有约 **55°**、深度约
  **70°**——这就是“只能看到正前方一小块”的常见且可修复的原因。
- 本项目默认已切到 **848×480**（约 69° 彩色 / 87° 深度），或可用 `--wide` 上
  **1280×720**（同为约 69°，但更清晰），即可把取景范围真正拉宽。
- 深度对齐（depth-aligned）到彩色后，感知 FOV 受彩色约 69° 限制；想用满深度 87°
  需另做深度为主投影（超本节范围）。

**SSH 能做/应做的（工具已写好）：**

```bash
bash scripts/go2w/tune_d435_fov.sh status   # SSH 查看当前分辨率 + 实测 HFOV/VFOV
bash scripts/go2w/tune_d435_fov.sh wide     # SSH 切到 1280x720 最宽最清晰
bash scripts/go2w/tune_d435_fov.sh mode 1280 720 30
bash scripts/go2w/tune_d435_fov.sh reset    # 恢复默认 1280x720
```
- 本机免 SSH 校验当前模式是否用满硬件视场：
  ```bash
  python3 scripts/go2w/validate_camera_fov.py --base http://192.168.123.18:8080
  ```
- 流服务本身现在会实时上报 **FOV**：`.html` 页面状态栏、`/fov`、`/info.json` 的
  `fov_deg` 字段（彩色 + 深度），用于核对当前模式没有裁剪。

**想真正“看得更大范围”的可行方向：**
1. 调高相机安装/压低倾角 → 扩大近处地面视场；
2. 让机器狗转头/多方向扫视（语义搜索逻辑本身已在多 heading_sector 旋转扫视）；
3. 硬件换广角/鱼眼镜头或改用 FOV 更宽的相机。

## 8. 本仓库文件

| 文件 | 说明 |
|---|---|
| `scripts/go2w/realsense_stream.py` | 机器狗上运行的 RGB-D 流服务源码 |
| `scripts/go2w/view_realsense.py` | 本机 OpenCV 查看/录制工具（`s` 快照 / `r` 录制 / `d` 原始深度 / `q` 退出） |
| `scripts/go2w/start_realsense_stream.sh` | 部署/启停/状态一键脚本 |
| `scripts/go2w/tune_d435_fov.sh` | SSH 进入机器狗查看/切换 D435 分辨率模式与实时 FOV（status/wide/mode/reset；默认 1280x720） |
| `scripts/go2w/validate_camera_fov.py` | 本机免 SSH 校验 D435 用满硬件视场（/fov、/info.json） |
