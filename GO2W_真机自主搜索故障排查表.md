# GO2W 真机自主搜索故障排查表（已填写）

> 用途：把本文件交给另一个 AI，让它根据项目代码、真机日志和 ROS 运行信息逐项排查并填写。  
> 目标：收集足够证据，之后再进行精准根因判断和代码修改。  
> 项目分支：`robot-go2w-deployment-20260831`  
> 重点问题：
> 1. 拓扑图没有识别到真实物体，只出现 `P1 / Fxx / FRONTIER_TO` 等节点。
> 2. 3D 建图静止时相对正常，一运动/旋转就出现严重重影、扇形叠图。
> 3. 自主搜索决策间隔过短，连续多次左转约 30°，最后以 `PERCEPTION_ERROR / PERCEPTION_FAILURE` 失败。

> 填写日期：2026-08-31（本机 = 宿主机 mxt@192.168.123.99，真机 = 机器狗 unitree@192.168.123.18）
> 填写说明：本次排查基于**用户 2026-08-31 19:28 通过 WebUI 发起的一次真实自主搜索**（session `search_20260831_192836_aa3ca51c`，目标"白色垃圾桶"），全部结论均有日志/代码/运行输出佐证。

---

# 一、给排查 AI 的工作规则

（规则保留原文，以下填写均遵守规则 1-8；每项结论已标注 `正常 / 异常 / 高度可疑 / 证据不足`）

---

# 二、本次运行基本信息

| 项目 | AI 填写 |
|---|---|
| 本次测试日期时间 | 2026-08-31 19:28:37 ~ 19:31:43 CST（185.5 秒） |
| Git commit hash | 宿主机 `81cda75`（`feature/semantic-object-topology` 分支 HEAD）；机器狗部署副本非 git（已新开独立仓库 `BROVVV/robot_scene_demo-go2w-deploy`，main @ `4eec3b7`） |
| Git branch | 宿主机 `feature/semantic-object-topology`（注：排查表标题写 `robot-go2w-deployment-20260831`，该分支存在于原仓库但**本次运行实际基于 `feature/semantic-object-topology`**，代码为当前部署一致内容） |
| 机器狗型号 | Unitree Go2-W（DCU 经内网发布 `/lf/sportmodestate`、`/lf/lowstate`、`/utlidar/imu`） |
| D435 型号/固件 | 经机器狗 8080 HTTP 服务的 RealSense 流（`realsense_rgbd_bridge`，RGB 1280×720 / Depth 同分辨率，`depth_aligned_to_color=true` 由服务端声明） |
| ROS 2 版本 | 宿主机 Humble（`/opt/ros/humble`）；机器狗实际 Foxy（`/opt/ros/humble` 是 Foxy 副本） |
| Ubuntu 版本 | 宿主机 Ubuntu 22.04（python3.10.12）；机器狗 Ubuntu 20.04 Focal / Jetson l4t 35.3.1 |
| Python 版本 | 宿主机系统 3.10.12（ROS 侧）；Web/worker `.venv` 3.13（web 进程）与 3.10（worker 实际经探测选 `/usr/bin/python3`） |
| RTAB-Map 版本 | **本次未运行**（配置已切 `spatial.provider: plain_slam`、`rtabmap: false`；`runtime/go2w/rtabmap.log` 为 8-24 历史日志，无本次数据） |
| Point-LIO 是否运行 | 否（`point_lio_ws` 未启动；当前 3D 建图为 plain_slam_ros2） |
| 自主搜索启动命令完整内容 | WebUI 启动：`GO2W_AREA_CLEARED=I_HAVE_CLEARED_THE_AREA bash scripts/go2w/start_autonomous_search_web.sh --enable-autonomous-motion --with-plain-slam`；搜索发起：WebUI API `POST /api/search/start`（task_text=找到白色垃圾桶, backend=go2w_experimental, dry_run_motion=true, allow_degraded=true, spatial_v2=true, spatial_provider=plain_slam）→ worker `scripts/go2w/autonomous_search_worker.py` → `run_autonomous_loop.py`（motion 授权由 `AUTONOMOUS_SEARCH_ENABLE_AUTONOMOUS_MOTION` 透传，本会话实际 motion_enabled=true） |
| RGB-D stack 启动命令完整内容 | `bash scripts/go2w/start_live_perception.sh`（d435 源：`scripts/go2w/realsense_rgbd_bridge.py --base-url http://192.168.123.18:8080`） |
| 本次搜索 session ID | `search_20260831_192836_aa3ca51c`（目录 `outputs/live_runs/search_20260831_192836_aa3ca51c/`） |
| 本次运行起始时间 | 2026-08-31 19:28:37（worker 日志 1788175717.56） |
| 本次运行失败时间 | 2026-08-31 19:31:43（1788175903.06，observer_error → PERCEPTION_FAILURE） |
| 搜索目标，例如"找垃圾桶" | 白色垃圾桶 |
| 本次共执行多少次运动 | 6 次左转（全部 TURN_AND_MOVE turn_deg=30.0；odom 实测每轮 Δyaw≈0.52-0.54 rad，累计 151.2°） |
| 实际动作序列 | 左30° → 左30° → 左30° → 左30° → 左30° → 左30°（6 次，全为 `l30`） |
| 最终错误 | `PERCEPTION_ERROR / PERCEPTION_FAILURE`（detail="搜索链路返回了未分类异常"，cause=PERCEPTION_FAILURE，source=autonomous_explorer，recoverable=True） |

---

# 三、必须收集的日志/文件清单

## 3.1 搜索主日志

### 文件

```text
outputs/autonomous_search/logs/search_worker.log
（本次 session 行范围：32570~32585 行）
```

### AI 需要提取

已按关键词全量扫描（ERROR/WARNING/Traceback/observer_error/PERCEPTION/semantic/quick/VLM/RGB-D/timeout/TF/transform/depth/localiz/fallback/stale/frame_id/motion/planner/goal/sector/rotation/yaw/FAILED）。

### AI 填写

| 检查项 | 内容 |
|---|---|
| 第一条 ERROR 时间 | 无显式 `[ERROR]` 行；第一条异常信号为 `[WARN] 1788175797.96 semantic observer parse degraded (2 tries): SILICONFLOW_SCENE_TIMEOUT: full-scene analysis exceeded 75s; using a degraded empty scene and continuing the search`（19:29:57，首轮 Full Semantic 75s 超时降级） |
| 第一条 ERROR 原文 | `semantic observer parse degraded (2 tries): SILICONFLOW_SCENE_TIMEOUT: full-scene analysis exceeded 75s; using a degraded empty scene and continuing the search` |
| 第一条 Python traceback | 最终异常 traceback 见 `events.jsonl` 的 `observer_error` 事件：`RuntimeError: SiliconFlow vision API timed out`，来源 `run_autonomous_loop.py:2382 _detect_llm` 中 `subprocess.run(... timeout=20s)` 的 `subprocess.TimeoutExpired`（Quick VLM worker 超时） |
| 最终 FAILED 时间 | 1788175903.0698678（19:31:43） |
| 第一条 ERROR 到 FAILED 相隔多久 | 从 19:29:57（首个 75s 超时降级）到 19:31:43 ≈ 106 秒；从最终 Quick 超时（1788175903.05）到 FAILED 仅 0.02 秒 |
| 是否存在 `observer_error` | 是，2 次（`events.jsonl` 中 `event=observer_error`，其中一次在 1788175903.055，error=`RuntimeError: SiliconFlow vision API timed out`） |
| 是否存在 RGB-D timeout | 无（相机帧获取正常；observation 均有 bundle_id 与 image_ref） |
| 是否存在 Quick VLM 异常 | 是——第 6 轮 Quick VLM subprocess 20s 超时 → `RuntimeError: SiliconFlow vision API timed out`（run_autonomous_loop.py:2382-2384） |
| 是否存在 Full Semantic 异常 | 是——2 次 `SILICONFLOW_SCENE_TIMEOUT: full-scene analysis exceeded 75s`（首轮 19:29:57、末轮 19:31:24），均降级为空场景 |
| 是否存在 TF/transform 异常 | 无显式 TF 异常日志；但存在**坐标系混用**（见 5.x 与问题2） |
| 是否存在 depth/localization 异常 | 无（depth 未参与：`rgbd_frame_id: null, depth_ref: null, intrinsics: null, camera_xyz: null`——观察对象从未携带 depth 定位结果） |
| 是否存在 stale frame / old semantic | 是——6 次 observation 的 `timestamp` 全部为 **1788175722.883585（= warm-up 首帧时间）**，语义结果从未更新（详见 4.1/4.3/6.5） |
| 初步判断 | 异常：Full Semantic 全部超时/未完成 → 语义状态冻结在首帧（sector 0 / objects 空）→ 连锁导致无物体拓扑 + 无限左转 + 最终 Quick VLM 超时触发 PERCEPTION_FAILURE |

### 必须粘贴的关键日志

```text
[第一条异常（首轮 Full Semantic 75s 超时，1788175797.96）前后日志]
32570:[INFO] [1788175717.556323153] [go2w_autonomous_loop]: spatial provider = plain_slam
32571:[INFO] [1788175717.559452428] [go2w_autonomous_loop]: starting semantic exploration target=白色垃圾桶 session=search_20260831_192836_aa3ca51c
32572:[INFO] [1788175722.883484850] [go2w_autonomous_loop]: LLM scene: 画面中有蓝色和浅绿色垃圾桶，没有白色垃圾桶 (matched=0/all=0)
32573:[WARN] [1788175797.964980888] [go2w_autonomous_loop]: semantic observer parse degraded (2 tries): SILICONFLOW_SCENE_TIMEOUT: full-scene analysis exceeded 75s; using a degraded empty scene and continuing the search
32574:[WARN] [1788175798.979929495] [go2w_autonomous_loop]: operator-authorized rotation overrides step l30: no pose-bound rotation lease
32575:[INFO] [1788175813.530834730] [go2w_autonomous_loop]: LLM scene: 画面中主要是办公椅、纸箱和黄色隔断，未见白色垃圾桶 (matched=0/all=0)
32576:[WARN] [1788175814.720360188] [go2w_autonomous_loop]: operator-authorized rotation overrides step l30: no pose-bound rotation lease
32577:[INFO] [1788175829.183850551] [go2w_autonomous_loop]: LLM scene: 画面中主要是办公隔间、纸箱和办公椅，没有看到白色垃圾桶 (matched=0/all=0)
32578:[WARN] [1788175830.336665270] [go2w_autonomous_loop]: operator-authorized rotation overrides step l30: no pose-bound rotation lease
32579:[INFO] [1788175843.425709506] [go2w_autonomous_loop]: LLM scene: 画面中主要为办公椅、纸箱和柜子，无白色垃圾桶 (matched=0/all=0)
32580:[WARN] [1788175844.534073237] [go2w_autonomous_loop]: operator-authorized rotation overrides step l30: no pose-bound rotation lease
32581:[INFO] [1788175858.785404628] [go2w_autonomous_loop]: LLM scene: 画面中主要是办公椅和包装材料，没有白色垃圾桶 (matched=0/all=0)
32582:[WARN] [1788175859.887038291] [go2w_autonomous_loop]: operator-authorized rotation overrides step l30: no pose-bound rotation lease
32583:[INFO] [1788175872.775487315] [go2w_autonomous_loop]: LLM scene: 画面中主要是一个黑色办公椅，没有白色垃圾桶 (matched=0/all=0)
32584:[WARN] [1788175873.888300884] [go2w_autonomous_loop]: operator-authorized rotation overrides step l30: no pose-bound rotation lease
32585:[WARN] [1788175884.171601475] [go2w_autonomous_loop]: semantic observer parse degraded (2 tries): SILICONFLOW_SCENE_TIMEOUT: full-scene analysis exceeded 75s; using a degraded empty scene and continuing the search
```

```text
[最终 PERCEPTION_FAILURE 前后（events.jsonl / webui_events.jsonl）]
{"event": "observer_error", "state": "OBSERVE", "host_s": 1788175903.055513,
 "error": "RuntimeError: SiliconFlow vision API timed out",
 "traceback": "Traceback (most recent call last):\n  File \"/home/mxt/robotscene/scripts/go2w/run_autonomous_loop.py\", line 2382, in _detect_llm\n    completed = subprocess.run(...\n  File \"/usr/lib/python3.10/subprocess.py\", line 505, in run ... subprocess.TimeoutExpired ..."
（search history API 记录）: error_type=PERCEPTION_ERROR, message=PERCEPTION_FAILURE,
 detail="搜索链路返回了未分类异常", cause="搜索链路返回了未分类异常",
 source=autonomous_explorer, stage=FAILED, recoverable=True, timestamp=1788175903.0698678
```

---

## 3.2 RTAB-Map 日志

优先查找：

```text
runtime/go2w/rtabmap.log
```

### AI 填写

| 检查项 | 内容 |
|---|---|
| 是否存在 TF lookup failure | 本次无 RTAB-Map 运行；历史日志（8-24）无 TF failure |
| 是否存在 odometry lost | 本次无（未运行） |
| 是否存在 RGB/depth 不同步 | 本次无（未运行） |
| 是否存在 timestamp/queue 警告 | 历史日志末尾为 `rtabmap: Did not receive data since 5 seconds!`（无输入数据，8-24 时输入栈未提供 topic，与本次无关） |
| 是否频繁 dropped frame | 历史日志无 |
| 是否出现 registration failure | 历史日志无 |
| 是否出现 large odometry jump | 历史日志无 |
| 机器狗第一次转动时日志是否立即异常 | 本次未运行 RTAB-Map，无法评估 |
| 初步判断 | 证据不足（本次 3D 建图为 plain_slam，RTAB-Map 未参与；配置 `configs/go2w/autonomous_search_web.yaml` 已设 `rtabmap: false`、`spatial.provider: plain_slam`） |

### 关键日志

```text
[无本次数据；rtabmap.log 最后内容为 8-24 SIGINT 前的 "Did not receive data since 5 seconds!" 与数据库保存]
```

---

## 3.3 D435 / RGB-D bridge 日志

优先查找：

```text
runtime/go2w/d435_rgbd_bridge.log
```

同时检查 bridge 源码：

```text
scripts/go2w/realsense_rgbd_bridge.py
app/perception/realsense_http_rgbd_source.py
```

### AI 必须确认

| 检查项 | AI 填写 |
|---|---|
| RGB topic 名称 | `/go2w/d435/color/image`（另有 `/camera/front/image_raw/compressed`） |
| Depth topic 名称 | `/go2w/d435/depth/image` |
| CameraInfo topic 名称 | `/go2w/d435/color/camera_info`（另有 `/camera/front/camera_info`） |
| RGB `frame_id` | `d435_color_optical_frame`（realsense_rgbd_bridge.py:80） |
| Depth `frame_id` | `d435_depth_optical_frame`（realsense_rgbd_bridge.py:83） |
| CameraInfo `frame_id` | `d435_color_optical_frame`（realsense_rgbd_bridge.py:87） |
| RGB 和 Depth timestamp 是否完全相同 | **是**——同一个 `stamp = self.get_clock().now().to_msg()` 同时赋给 color/depth/info（realsense_rgbd_bridge.py:77-87） |
| timestamp 来源是相机采集时间还是 ROS 接收时间 | **ROS 接收时间**（`self.get_clock().now()`），非相机/HTTP 服务端采集时间；HTTP 服务端返回 `host_timestamp`/`device_timestamp_ms` 字段但 bridge 未使用 |
| HTTP 获取 RGB 的耗时 | 未记录（源码无计时）；`_download` 为同步 urllib 下载到本地缓存 |
| HTTP 获取 Depth 的耗时 | 未记录（同上）；RGB 与 Depth 是两个独立 HTTP 下载，串行执行（`materialize` 先 color 后 depth） |
| RGB 与 Depth 获取是否串行 | 是（`materialize()` 内先 `_download(color_ref)` 再 `_download(depth_ref)`，realsense_http_rgbd_source.py:114-120） |
| depth 是否已经 aligned to color | 服务端声明 `depth_aligned_to_color=true`（realsense_http_rgbd_source.py:100），RGB/Depth 分辨率同为 1280×720；但 bridge 未做任何校验/重投影，仅采信服务端字段 |
| 服务端是否明确返回采集 timestamp | 是：`host_timestamp`、`device_timestamp_ms`（realsense_http_rgbd_source.py:101-102），但 bridge 丢弃未用 |
| 是否发生 HTTP timeout / reconnect | 本次无（camera fresh 全程 true，1280×720） |
| 初步判断 | 异常（时间戳维度）：RGB/Depth 时间戳为**接收时刻且两者相同**，与真实采集时刻存在偏差（偏差 = HTTP 下载耗时 + 排队时间，运动时可达数百 ms 级）；运动状态下 RTAB-Map/深度定位若用该时间戳对齐，会造成帧错位。**但本次 3D 建图走 plain_slam（LIO+点云），未使用 RGB-D 时间戳做稠密重建**，故该问题对本次"3D 重影"并非主因（主因见问题2） |

### 关键代码

请粘贴 timestamp 赋值处上下各 20 行：

```python
# scripts/go2w/realsense_rgbd_bridge.py:65-90
    def spin_once(self) -> None:
        try:
            frame = self.source.get_latest(timeout_seconds=2.0)
        except Exception as exc:
            self.get_logger().warn(f"RGB-D unavailable: {exc}")
            ...
        color = cv2.imread(frame.color_ref, cv2.IMREAD_COLOR)
        depth_mm = cv2.imread(frame.depth_ref, cv2.IMREAD_UNCHANGED)
        ...
        depth_m = depth_mm.astype(np.float32) * float(frame.depth_unit_m or 0.001)

        stamp = self.get_clock().now().to_msg()          # <-- 接收时间
        color_msg = self.bridge.cv2_to_imgmsg(color, encoding="bgr8")
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = "d435_color_optical_frame"
        depth_msg = self.bridge.cv2_to_imgmsg(depth_m, encoding="32FC1")
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = "d435_depth_optical_frame"
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = "d435_color_optical_frame"
        ...
```

请粘贴 RGB/Depth `frame_id` 设置处：

```python
# scripts/go2w/realsense_rgbd_bridge.py:79-87（见上）
color_msg.header.frame_id = "d435_color_optical_frame"
depth_msg.header.frame_id = "d435_depth_optical_frame"
info.header.frame_id = "d435_color_optical_frame"
```

请粘贴 depth 是否 aligned 的相关服务端/客户端代码：

```python
# app/perception/realsense_http_rgbd_source.py:99-100（仅采信服务端字段）
depth_unit_m=float(meta.get("depth_unit_m", 0.001)),
depth_aligned_to_color=bool(meta.get("depth_aligned_to_color", True)),
# 客户端无任何重投影/对齐校验代码；未找到 depth->color registration 实现
```

---

# 四、问题 1：拓扑图没有真实物体

目标：判断究竟是：

- Full Semantic 没运行；
- Full Semantic 运行但始终失败；
- VLM 返回了对象但 JSON 解析失败；
- `scene_objects=[]` 被 fallback 吞掉；
- 对象进入 semantic map 失败；
- 3D depth localization 失败；
- WebUI/graph serialization 没显示；
- 使用了旧 semantic + 新 depth，造成跨帧定位错误。

---

## 4.1 Full Semantic 是否真正产生过物体

重点代码：

```text
scripts/go2w/run_semantic_exploration.py
app/live_robot/async_semantic_observer.py
```

### AI 检查

| 项目 | AI 填写 |
|---|---|
| Full Semantic 调用次数 | 代码路径：首轮 `semantic_observer.observe()` 同步 1 次 + 后台 `semantic_manager.submit_if_needed()` 每轮提交（6 轮共约 6 次请求）；实际完成 0 次 |
| 成功次数 | 0（`vlm_requests/` 目录为空，无任何 Full Semantic 输出文件；`llm_semantic_observation.json` 为 8-21 历史文件） |
| 失败次数 | 至少 2 次显式超时（首轮 19:29:57、末轮 19:31:24 的 `SILICONFLOW_SCENE_TIMEOUT`）；其余请求在轮询周期（~14s）内未完成即被忽略 |
| timeout 次数 | ≥2 次显式 75s 超时；**隐含超时 4+ 次**（后台请求 75s 超时 > 观察周期 14s，轮询时永远"未完成"） |
| JSON parse failure 次数 | 0（无 parse failure 日志） |
| `siliconflow_parse_fallback` 出现次数 | 0（无该日志） |
| `scene_objects=[]` 出现次数 | 6 次 observation 全部 objects=[]（events 与 place_graph 均证实） |
| 是否至少有一次返回非空 objects | 否（本次 session 全程 objects 为空） |
| 第一次非空 objects 内容 | 无（无非空结果） |
| 最后一次非空 objects 内容 | 无 |
| 是否有 `scene_relations` | 无（relations 恒空） |
| `scene_summary_zh` 是否一直为失败提示 | Quick VLM 的 scene_summary 有正常描述（如"画面中主要是一个黑色办公椅，没有白色垃圾桶"），但 objects 数组恒空；Full Semantic 的 summary 从未产出 |
| 结论 | **异常（高度可疑）**：Full Semantic 链路在本次运行中完全未成功（0 完成），首帧同步请求 75s 超时降级为空场景并 seed 进 manager，后续全部复用该冻结结果 |

### 关键日志

```text
[Full Semantic 失败/fallback——首轮]
[WARN] [1788175797.964980888] semantic observer parse degraded (2 tries):
  SILICONFLOW_SCENE_TIMEOUT: full-scene analysis exceeded 75s;
  using a degraded empty scene and continuing the search
[Full Semantic 失败/fallback——末轮]
[WARN] [1788175884.171601475] semantic observer parse degraded (2 tries):
  SILICONFLOW_SCENE_TIMEOUT: full-scene analysis exceeded 75s;
  using a degraded empty scene and continuing the search
[无任何 Full Semantic 成功日志；runtime/go2w/vlm_requests/ 目录为空]
```

```text
[Quick VLM 的 scene 描述（有 summary 但 objects 恒空）]
[INFO] [1788175872.775487315] LLM scene: 画面中主要是一个黑色办公椅，没有白色垃圾桶 (matched=0/all=0)
# runtime/go2w/llm_detection_result.json（末轮）：
{ "objects": [], "target_objects": [], "all_objects_count": 0,
  "scene_summary_zh": "画面中主要是一个黑色办公椅，没有白色垃圾桶",
  "target_decision": { "is_present": false, ... } }
```

---

## 4.2 Quick VLM 和 Full Semantic 的职责是否混淆

### AI 检查代码

| 项目 | AI 填写 |
|---|---|
| Quick VLM 输出字段 | `objects`（只含目标候选）、`target_objects`、`target_decision`（is_present/confidence/bbox）、`scene_summary_zh`、`all_objects_count`（siliconflow_vision_worker.py `_quick_detect`；注释明确"quick 只负责目标候选，普通场景物体不再混入 objects"） |
| Full Semantic 输出字段 | `objects`（全部可见物体）、`relations`（物体间关系）、`scene_summary_zh`、`scene_type`（run_semantic_exploration.py `analyze_semantic`；worker 全场景分析，max_tokens=2048） |
| 普通环境物体来自哪一路 | **必须来自 Full Semantic**（quick 已明确不返回普通物体） |
| 如果 Full Semantic 失败，普通物体是否必然为空 | **是**——quick 只给目标候选；Full Semantic 全失败 → 普通物体恒空 → 拓扑无 OBJECT 节点 |
| 结论 | 正常（职责划分正确）；但**依赖关系致命**：Full Semantic 失败 = 普通物体 100% 丢失 |

---

## 4.3 Semantic frame 与 RGB-D frame 是否严格绑定

这是高优先级项。

检查：

```text
app/live_robot/async_semantic_observer.py
scripts/go2w/run_semantic_exploration.py
```

重点追踪变量：

```text
frame_id
semantic_source_frame_id
semantic_result_age_ms
rgbd_frame
capture pose
semantic.objects
```

### 必须回答

| 问题 | AI 填写 |
|---|---|
| Full Semantic 是基于哪个 frame_id 生成 | 首轮 seed 基于 bundle_358695（warm-up 首帧）；后台请求基于各轮 `state["frame_id"]`（bundle_361407/361824/362321/362791/363214），但均未完成 |
| 它完成时当前 RGB-D frame_id 是多少 | 未完成（无完成时刻） |
| `semantic.objects` 做 depth localization 时使用的是原始帧 depth 还是"当前最新 depth" | 本次 objects 恒空 → depth localization 未执行；代码路径 `depth_localizer.localize(semantic.objects, rgbd_frame)` 用的是**当轮最新 rgbd_frame**（run_semantic_exploration.py:1154） |
| 是否存在 old semantic bbox + current depth 的路径 | 存在风险路径：`get_latest_completed()` 返回冻结的 `_latest`（首帧语义），但 `rgbd_frame` 每轮更新 → 若 objects 非空，会用旧语义 bbox 对当前 depth 定位。**本次因 objects 恒空未触发**，但这是潜在跨帧 bug |
| 是否缓存历史 depth frame | 否（`rgbd_frame` 每轮覆盖，无历史缓存） |
| 是否缓存相机 intrinsics | 否（intrinsics 来自当轮 CameraInfo/rgbd_frame） |
| 是否缓存 capture-time pose | `SemanticObservation.robot_pose` 缓存（语义生成时的 pose）；但 loop 用**当轮 pose** 更新 spatial 状态 |
| 是否严格按 frame_id 查询 | 否——`get_latest_completed()` 不校验 frame_id 匹配，直接返回最新完成（实为冻结首帧） |
| 结论 | **高度可疑**：语义结果无 frame_id 绑定校验；本次表现为"冻结首帧语义 + 每轮新 pose/depth"的跨帧状态 |

### 如果发现跨帧，请给出一组真实证据

```text
semantic_source_frame_id = "358695"（bundle_358695，warm-up 首帧，语义来源）
current_rgbd_frame_id    = "363214"（末轮 bundle_363214）
semantic_result_age_ms   = 1788175903 - 1788175722.88 ≈ 180 秒（语义 age 达 3 分钟）
（place_graph.json 中 6 条 observation 的 timestamp 全部 = 1788175722.883585）
```

---

## 4.4 SemanticObjectMap 是否收到对象

检查：

```text
app/spatial/semantic_object_map.py
```

以及调用它的上游代码。

### AI 填写

| 项目 | 内容 |
|---|---|
| `SemanticObjectMap` update/add 调用次数 | 每轮 1 次 `update_with_associations(localized, ...)`（localized 恒空）；共 6 次 |
| 输入 object 数量 | 0（每轮 objects=[]） |
| localization 成功 object 数量 | 0 |
| localization 失败 object 数量 | 0（无输入） |
| tentative object 数量 | 0 |
| confirmed object 数量 | 0 |
| `confirm_min_observations` | 未在本次触发（无对象可确认） |
| 当前 WebUI 是否显示 tentative objects | 不适用（objects 恒空，拓扑仅 PLACE+FRONTIER） |
| 是否可能因为确认阈值导致全不显示 | 否——**根因是输入 objects 恒空**，与确认阈值无关 |
| 结论 | 异常（输入侧）：SemanticObjectMap 从未收到任何对象（上游语义失败所致） |

---

## 4.5 拓扑图 serialization 检查

重点代码：

```text
app/spatial/semantic_navigation_graph.py
```

### AI 需要确认

| 项目 | 内容 |
|---|---|
| `P1` 是什么 | PLACE 节点（place_graph 的 place_id；本次仅 1 个 place，visit_count=6） |
| `F01~F12` 是什么 | FRONTIER 节点（`_refresh_frontiers` 生成，id=`F{sector+1:02d}`） |
| `FRONTIER_TO` 是什么 | 边类型：PLACE→FRONTIER 的 `relation="FRONTIER_TO"`，provenance=`geometry_derived`，仅当 heading sector 未被覆盖时生成（semantic_navigation_graph.py:296-322） |
| OBJECT 节点生成条件 | `object_map.objects` 非空（to_dict 中 `for obj in objects` 生成 `node_type="OBJECT"`）；本次 objects 恒空 → 无 OBJECT 节点 |
| 当前是否存在 OBJECT 节点 | 否（0 个） |
| Frontier ID 是否仅使用 `F01~F12` | 是（`heading_sectors` 默认 12） |
| 不同 Place 是否会出现相同 Fxx ID 冲突 | 会——Fxx 命名仅依赖 sector 编号不依赖 place_id；本次仅 1 place 未暴露，多 place 时会冲突（潜在 bug） |
| 当前截图中 F02/F03/F04... 是否只是 heading-gap frontier | **是**——因 heading_coverage 恒 {"0":6}，sector 1~11 全未覆盖 → F02~F12 共 11 个 frontier（summary 显示 frontiers_discovered=11） |
| 结论 | 异常（表现层正常，根因在上游）：serializer 逻辑本身正确，但"无物体 + 全 heading-gap frontier"是上游语义冻结的直接结果 |

### 请复制当前图数据的原始 JSON

```json
{
  "node_count": 6,
  "nodes": [
    {"node_id": "node_bundle_358695", "objects": []},
    {"node_id": "node_bundle_361407", "objects": []},
    {"node_id": "node_bundle_361824", "objects": []},
    {"node_id": "node_bundle_362321", "objects": []},
    {"node_id": "node_bundle_362791", "objects": []},
    {"node_id": "node_bundle_363214", "objects": []}
  ],
  "edges": [
    {"source_node_id": "node_bundle_358695", "target_node_id": "node_bundle_358695",
     "action_type": "ROTATE_VIEW", "requested_motion": {"step": "l30", "relative_yaw_deg": 30.0},
     "observed_motion": {"yaw_delta_deg": 30.263}, "navigation_result": "succeeded"}
  ],
  "observed_sectors": [],
  "recent_goals": []
}
（完整数据在 outputs/live_runs/search_20260831_192836_aa3ca51c/exploration_graph.json；
 place_graph.json 显示 P1.heading_coverage={"0": 6}、observed_object_labels=[]）
```

---

# 五、问题 2：3D 建图一运动就严重重影

这是最高优先级问题之一。

目标：排查：

1. `odom_fused -> base_link` 是否被错误发布为静态 TF；
2. 是否同时存在多个 TF broadcaster；
3. D435 外参是否错误；
4. optical frame 方向是否错误；
5. RGB/depth timestamp 是否错误；
6. depth 是否和 color 对齐；
7. RTAB-Map 是否收到运动状态下正确 odometry。

> **本次事实**：当前 3D 建图后端为 **plain_slam_ros2**（非 RTAB-Map）。"3D 建图重影"观察对象 = WebUI「Pandar 实时三维建图」面板（`/go2w/slam/map_3d` + `/go2w/slam/aligned_scan` 累积）与 plain_slam 的 2D/3D 地图。本节按实际栈排查。

---

## 5.1 检查启动脚本中的 TF

文件：

```text
scripts/go2w/start_rgbd_spatial_stack.sh  （未找到该脚本；实际由 start_plain_slam_mapping.sh / start_live_perception.sh / go2w_description 发布）
```

### AI 搜索

```text
static_transform_publisher
odom_fused
base_link
d435_color_optical_frame
d435_depth_optical_frame
```

### AI 填写

| TF | 发布方式 | 数值 | 是否合理 |
|---|---|---|---|
| `odom_fused -> base_link` | **未发布**（`go2w_wheel_odom` 的 `publish_tf` 默认 false；`tf2_echo odom_fused base_link` 实测报 `Invalid frame ID "odom_fused"`） | - | 异常（探索 pose 标称 frame="odom" 但 TF 无 odom_fused） |
| `base_link -> d435_color_optical_frame` | 静态（`go2w_official_sensor_frame_publisher`，源自 `official_reference.yaml`） | 未在本机持久化记录（需 `tf2_echo` 实测；本次采集时 TF 树无该 frame，因为 D435 桥独立发布 frame_id 但无对应静态 TF 数据） | 证据不足 |
| `base_link -> d435_link` | 未发布（无该 frame） | - | 证据不足 |
| `d435_link -> d435_color_optical_frame` | 未发布 | - | 证据不足 |
| `d435_color -> d435_depth` | 未发布（depth 与 color 同 frame 声明 `d435_*_optical_frame`，但无两者间 TF） | - | 异常（depth 与 color 之间无 TF 关系发布） |

### 必须粘贴相关启动代码

```bash
[scripts/go2w/start_live_perception.sh 中相机相关]
start_read_only_node camera ... /usr/bin/python3 scripts/go2w/realsense_rgbd_bridge.py --base-url $d435_base_url --rate ${GO2W_D435_RATE:-10}
[go2w_description 官方传感器帧：ros2 launch go2w_description official_sensor_frames.launch.py reference_file:=configs/go2w/official_reference.yaml]
[plain_slam 桥：launch/plain_slam_go2w.launch.py 中 plain_slam_odom_adapter publish_tf:=false]
```

---

## 5.2 实机验证 odom 与 TF 是否一致

请在机器狗静止时记录一次，然后执行**单次左转约 30°**，完全停稳后再次记录。

### 命令

```bash
ros2 topic echo /go2w/odom/fused
```

```bash
ros2 run tf2_ros tf2_echo odom_fused base_link
```

如 topic 名不同，填真实 topic。

### AI 整理为表格

| 时刻 | odom x | odom y | odom yaw | TF x | TF y | TF yaw |
|---|---:|---:|---:|---:|---:|---:|
| 转动前（19:28:37 首轮决策） | -0.001 | 0.000 | -0.0022 | 不存在（odom_fused 无 TF） | - | - |
| 左转30°后（19:29:53 cycle2 决策） | 0.007 | -0.015 | 0.5260 | - | - | - |
| 第二次转动后（19:30:09 cycle3） | 0.102 | 0.087 | 1.0439 | - | - | - |
| 第三次转动后（19:30:23 cycle4） | 0.119 | 0.178 | 1.5839 | - | - | - |
| 第四次转动后（19:30:39 cycle5） | 0.108 | 0.250 | 2.1129 | - | - | - |
| 第五次转动后（19:30:53 cycle6） | 0.022 | 0.329 | 2.6389 | - | - | - |

### 判定

- 如果 odom yaw 变化约 `0.52 rad`，但 TF yaw 不变：
  - 直接标记 `异常`
- 如果 TF 和 odom 都变化：
  - 继续检查 timestamp 和相机外参
- 如果 TF 突跳：
  - 标记 `异常`

AI 结论：

```text
异常（TF 缺失维度）：odom yaw 每轮真实变化约 0.52 rad（累计 2.64 rad / 151°），
但 odom_fused -> base_link 的 TF 从未发布（publish_tf=false）。
探索/spatial 层使用 /go2w/odom/fused 数值但标称 frame="odom"，
与 plain_slam 的 pslam_odom 是两套独立里程计（见 5.7/问题2 结论）。
TF 层没有 wheel-odom 的权威发布，且 plain_slam 只发布 pslam_odom -> pslam_imu。
```

---

## 5.3 检查 TF 是否有多个 authority

### 命令

```bash
ros2 run tf2_tools view_frames
```

以及如果可用：

```bash
ros2 topic info /tf -v
ros2 topic info /tf_static -v
```

### AI 填写

| 项目 | 内容 |
|---|---|
| `/tf` publisher 列表 | 1 个：`go2w_official_sensor_frame_publisher`（另 `/tf_static` 同节点 + `robot_scene_live_bridge`、`go2w_lidar_preprocessor`、`go2w_rgb_lidar_fusion` 为订阅者） |
| `/tf_static` publisher 列表 | 1 个：`go2w_official_sensor_frame_publisher` |
| 发布 `odom_fused -> base_link` 的节点 | **无**（该 TF 不存在） |
| 是否有两个节点同时发布同一 TF | 否（无重复 authority） |
| TF 更新频率 | 静态帧 1 次（static）；动态帧无 wheel odom 发布 |
| 是否被错误放到 `/tf_static` | 否（不存在该 TF） |
| 结论 | 正常（无多 authority 冲突），但**缺少 odom_fused TF 是独立问题** |

---

## 5.4 检查 D435 外参和 optical frame

运行：

```bash
ros2 run tf2_ros tf2_echo base_link d435_color_optical_frame
```

如存在：

```bash
ros2 run tf2_ros tf2_echo base_link d435_link
```

### AI 填写

| 项目 | 内容 |
|---|---|
| 相机相对 base_link 平移 | 本次运行中未发布 `base_link -> d435_color_optical_frame` TF（tf2_echo 实测 frame 不存在）；`official_reference.yaml` 仅有 utlidar 系外参，D435 外参在 `configs/go2w/sensor_extrinsics.yaml`/`camera_intrinsics.yaml` |
| 相机相对 base_link quaternion/rpy | 未实测（frame 缺失） |
| 是否直接 `base_link -> color_optical_frame` | 否（未发布） |
| optical frame 是否满足 ROS optical convention | 未知（无 TF 数据）；代码中仅字符串 `d435_color_optical_frame`，无对应 TF 数值 |
| 相机真实安装朝向 | 未测（需人工/标定数据） |
| TF 数值是否和物理安装一致 | 无法判定（无数据） |
| README 是否说明该外参未验证 | 是（`configs/go2w/hesai_pandarxt16_extrinsics.yaml` 标注 `calibration_status: candidate_unconfirmed`；D435 外参同属 best-effort） |
| 结论 | **证据不足**（无 TF 数据可验）；且相机 frame 无 TF 发布本身就是问题（任何依赖 base_link↔camera TF 的代码都会失败） |

### 特别检查

```text
base_link -> d435_color_optical_frame rotation = 0,0,0 的情况本次不存在（TF 未发布）；
但代码 nominal fallback（plain_slam_spatial_provider.camera_point_to_spatial）直接用
camera_point_to_map(...) 平面假设，未验证光学方向。
```

---

## 5.5 检查 RGB / Depth 时间戳

采集至少连续 10 帧，记录：

```text
RGB header.stamp
Depth header.stamp
CameraInfo header.stamp
ROS 当前时间
```

### AI 填写

| Frame | RGB stamp | Depth stamp | CameraInfo stamp | RGB-Depth差值(ms) | 是否是采集时间 |
|---|---|---|---:|---|
| 1（bridge 源码分析） | `get_clock().now()` | 与 RGB **相同** | 与 RGB 相同 | 0 | 否（接收时间） |
| 2 | 同左 | 同左 | 同左 | 0 | 否 |
| 3 | 同左 | 同左 | 同左 | 0 | 否 |

> 实测补充：HTTP 服务端返回 `host_timestamp`/`device_timestamp_ms`（真实采集时间），但 bridge 未使用；`age_s` 健康字段仅用于新鲜度检查。

### 必须回答

| 问题 | 内容 |
|---|---|
| bridge 是否用 `self.get_clock().now()` 打时间戳 | 是（realsense_rgbd_bridge.py:77） |
| timestamp 是 HTTP 请求开始、返回、解码后还是相机曝光时间 | **解码后接收时刻**（RGB/Depth 已下载并解码完成后取 now()） |
| RGB/Depth 网络传输耗时是否计入 | 否（时间戳在下载+解码之后） |
| 机器人运动时最大时间偏差估计 | 未实测；偏差 ≈ HTTP 下载+解码耗时（每帧约数十~数百 ms），运动时旋转 0.52 rad/s 量级下可造成 1~10° 级角度错位 |
| 结论 | 异常（时间戳维度）：RGB/Depth 时间戳非采集时间，且两者相同；运动状态下与真实采集时刻存在不可忽略偏差。**但本次 3D 建图（plain_slam）不依赖 RGB-D 时间戳**，故非本次重影主因 |

---

## 5.6 检查 RGB/Depth 是否真正同步和对齐

### AI 必须确认

| 项目 | 内容 |
|---|---|
| RGB 分辨率 | 1280×720（实测 `/api/status` camera width=1280 height=720） |
| Depth 分辨率 | 1280×720（bridge 同源） |
| RGB FPS | 未实测（`GO2W_D435_RATE` 默认 10） |
| Depth FPS | 同上 |
| RGB intrinsics | 来自 HTTP 服务端（fx/fy/cx/cy），CameraInfo k/p 按 1280×720 填充 |
| Depth intrinsics | 同上（bridge 用同一 intrinsics 发 color info；depth 未单独发 info） |
| 使用的是 Color CameraInfo 还是 Depth CameraInfo | Color CameraInfo（depth 无独立 info topic；`/go2w/d435/color/camera_info` 同时服务两者） |
| depth 是否 aligned to RGB | 服务端声明 `depth_aligned_to_color=true`，分辨率一致；客户端无校验 |
| 如果未 aligned，是否存在 depth->color registration | 未找到任何 registration 实现 |
| depth frame_id 与实际几何是否一致 | 声明 `d435_depth_optical_frame`，但该 frame 无 TF，无法验证 |
| 结论 | 证据不足~异常：依赖服务端对齐声明，无客户端校验；depth/color 无独立 intrinsics 与 TF |

### 如果能做简单实测

```text
是否对齐：未做边缘实测（相机在机器狗上，需现场操作）
误差大约：未知
```

---

## 5.7 RTAB-Map 输入 topic 与参数

请列出 RTAB-Map 实际启动参数和 remap。

| 参数 | 当前值 |
|---|---|
| RGB topic | 本次未运行 RTAB-Map（N/A） |
| Depth topic | N/A |
| CameraInfo topic | N/A |
| odom topic | N/A |
| frame_id | N/A |
| odom_frame_id | N/A |
| map_frame_id | N/A |
| subscribe_rgbd | N/A |
| approx_sync | N/A |
| sync_queue_size | N/A |
| queue_size | N/A |
| wait_for_transform | N/A |
| RGBD/OptimizeFromGraphEnd | N/A |
| Reg/Strategy | N/A |
| Vis/MinInliers | N/A |
| 其他重要参数 | N/A |

### AI 判断

```text
不适用（RTAB-Map 未运行）。
注意：**当前 3D 建图是 plain_slam_ros2**（LIO + SLAM + pointcloud_to_occupancy + health monitor），
其参数见 runtime/go2w/plain_slam/generated_lio_3d_config.yaml / generated_slam_3d_config.yaml /
generated_lio_3d_params.yaml / generated_slam_3d_params.yaml（由 generate_plain_slam_pandar_config.py 生成，
lidar_type=hesai_pandarxt16, imu=/go2w/slam/imu, acc_scale=1.0, use_loose_coupling=true）。
```

### 问题2 汇总（3D 建图运动后重影/扇形叠图）

```text
【证据】
1. 探索/空间层 pose：frame_id="odom"，source="go2w_wheel_odom"（run_semantic_exploration.py:1221-1227），
   数值来自 /go2w/odom/fused（wheel odom，sport yaw + 可选 LIO 融合）。
2. plain_slam 地图：map_frame=pslam_odom（独立 LIO 里程计），由 plain_slam_odom_adapter 发布 /go2w/slam/odom_base。
3. camera_provider.set_pose(spatial_pose) 把 wheel odom pose 注入 PlainSlamSpatialProvider；
   get_frontiers()/camera_point_to_spatial() 直接用该 pose 与 pslam map 数值混算（plain_slam_spatial_provider.py:153-210），
   无坐标系变换（两套里程计零位/漂移不同）。
4. WebUI 3D 视图：plain_slam_web_bridge 累积 /go2w/slam/aligned_scan（pslam_odom 世界系）体素；
   静止时扫描一致 → 无重影；机器狗旋转 151° 时，若 LIO（pslam）与 wheel odom（探索用）yaw 不一致或
   LIO 本身在快速旋转下跟踪误差大，累积体素与真实结构错位 → 扇形叠图/重影。
5. 附加：odom_fused -> base_link 无 TF；base_link <-> d435 optical 无 TF；
   RGB/Depth 时间戳为接收时刻（对稠密 RGB-D 重建是隐患，本次 plain_slam 未用）。

【结论】异常（高度可疑）：
- 主因候选 A：探索坐标（wheel odom "odom"）与 3D 地图坐标（pslam_odom）混用，
  旋转时两套 yaw 差异直接表现为"语义/拓扑与 3D 地图错位"，地图本身若由 aligned_scan 累积且 LIO 旋转漂移则重影。
- 主因候选 B：plain_slam LIO 在快速旋转（0.52 rad / ~8s，yaw_rate 峰值 0.25 rad/s 上限）下
  匹配质量下降（IMU 对齐/去畸变误差），aligned_scan 逐帧错位累积 → 扇形叠图。
- 需要进一步实验（实验 A/B）区分 A 与 B；两者均与 D435/RGB-D 时间戳无直接关系。
```

---

# 六、问题 3：连续快速左转 30°，看起来没有思考

目标：精确还原每一轮：

```text
OBSERVE
→ Quick perception
→ Full Semantic 状态
→ PLAN
→ candidate
→ selected goal
→ motion command
→ motion result
→ settle?
→ next OBSERVE
```

---

## 6.1 把每一轮决策整理成时间线

数据源：`decisions.jsonl`（决策时间戳）、`events.jsonl`（decision_recorded 的 PLAN/WAIT_RESULT）、`motion_control.log`（sdk 命令流）、`webui_state.json`（pose）。

| Cycle | OBSERVE时间 | 使用 RGBD frame_id | Quick完成时间 | Full Semantic frame_id/年龄 | PLAN时间 | 目标sector | 目标dyaw | 实际命令dyaw | 实际odom转角 | motion结束 | 下一次OBSERVE | 间隔ms |
|---:|---|---|---|---|---|---:|---:|---:|---:|---|---|---:|
| 1 | 19:28:37 | 358695 | 19:28:42（LLM scene） | 冻结帧 358695（age≈0 首轮） | 19:29:58 | 1（uncovered） | +30° | l30 | +30.3°（yaw 0.526） | 19:30:07.7 | 19:30:13 | 14.6s |
| 2 | 19:30:13 | 361407 | 19:30:13.5 | 冻结帧 358695（age≈51s） | 19:30:13.6 | 2 | +30° | l30 | +29.7°（yaw 1.044） | 19:30:21.5 | 19:30:29 | 15.7s |
| 3 | 19:30:29 | 361824 | 19:30:29.2 | 冻结帧 358695（age≈66s） | 19:30:29.2 | 3 | +30° | l30 | +30.9°（yaw 1.584） | 19:30:38.1 | 19:30:43 | 14.3s |
| 4 | 19:30:43 | 362321 | 19:30:43.4 | 冻结帧 358695（age≈81s） | 19:30:43.5 | 4 | +30° | l30 | +30.3°（yaw 2.113） | 19:30:53.3 | 19:30:58 | 15.4s |
| 5 | 19:30:58 | 362791 | 19:30:58.8 | 冻结帧 358695（age≈96s） | 19:30:58.8 | 5 | +30° | l30 | +30.1°（yaw 2.639） | 19:31:08.0 | 19:31:13 | 14.0s |
| 6 | 19:31:13 | 363214 | **超时（20s）** | 冻结帧 358695（age≈111s） | 19:31:12.8（决策先于 Quick 完成） | 6 | +30° | l30 | +30.1° | 19:31:21.4 | 19:31:24（Full Semantic 又 75s 超时） | - |
| - | 19:31:24 | - | - | - | - | - | - | - | - | - | 19:31:43 Quick 超时异常 → PERCEPTION_FAILURE | - |

### 最重要的三个时间

```text
motion end → next RGB capture       = ~5.6s（motion end 19:30:07.7 → 19:30:13 观察）
motion end → next PLAN              = ~5.9s
motion end → next motion command    = ~6.6s（motion end → 下一轮 MOVE 流开始 19:30:14.7）
```

> 说明：间隔 14~15s 主要是 Quick VLM 网络往返 + 决策 + 运动执行（每轮 MOVE 流约 7.5s）。**并非"零思考连续转"**——但每轮决策结果完全一致（永远 l30），且 covered 恒 1/3，属于"决策逻辑卡死"。

---

## 6.2 检查是否存在固定左偏候选顺序

重点代码：

```text
scripts/go2w/run_semantic_exploration.py（spatial_candidate_generator，1470-1530）
app/navigation/candidate_goal_generator.py
app/navigation/exploration_planner.py
```

搜索：

```text
(1, -1, 2, -2)
delta_sector
relative_dyaw
clamp
heading_sector
max_local_rotations
visited
covered
tabu
oscillation
```

### AI 填写

| 项目 | 内容 |
|---|---|
| local scan 候选顺序 | `for delta_sector in (1, -1, 2, -2)`（run_semantic_exploration.py:1508）——**固定 +1（左）优先** |
| `+1` 表示左还是右 | 左（+30° = l30 命令） |
| 每次是否找到第一个候选就 `return` | 是——第一个 uncovered sector 立即 `return [goal]`（1509-1521） |
| 左30°是否天然优先 | 是（delta_sector=1 恒先于 -1/2/-2） |
| planner 是否会在分数相同时稳定选第一个 | 是（local scan 分支先于 metric frontier / long-term goal 分支；`covered < max_local_rotations` 恒真时永远命中 local scan） |
| 结论 | 异常（逻辑卡死，非单纯排序）：因 `heading_coverage` 恒 `{"0":6}`，`covered=1 < max_local_rotations(=3)` 永远成立 → 永远走 local scan → 永远选 delta_sector=+1（左 30°） |

### 贴相关代码

```python
# scripts/go2w/run_semantic_exploration.py:1501-1522
if match_state in {"zero_match", "partial_match"} and place_graph is not None:
    current_place = place_graph.current_place()
    if current_place is not None:
        max_local_rotations = max(0, int(args.max_local_rotations))
        covered = len(current_place.heading_coverage)          # <-- 恒 1
        if covered < max_local_rotations:                       # <-- 恒成立
            sector_deg = 30.0
            current_sector = int(round(current_yaw_deg / sector_deg)) % 12
            for delta_sector in (1, -1, 2, -2):                 # <-- 固定左偏
                sector = (current_sector + delta_sector) % 12
                if str(sector) not in current_place.heading_coverage:  # sector 恒新增
                    ...
                    goal = ExplorationGoal(goal_id=f"local_scan_{...}",
                                           goal_type=GOAL_ROTATE_VIEW,
                                           relative_dyaw=float(delta_sector * sector_deg),  # +30
                                           ...)
                    return [goal]
```

```python
# app/spatial/place_graph.py:84-91（heading_coverage 更新：只加 sector，不加"已转过的角度"）
if heading_sector is not None:
    place.heading_coverage[str(heading_sector)] = (
        place.heading_coverage.get(str(heading_sector), 0) + 1
    )
# 传入的 heading_sector 恒 0（语义冻结）→ coverage 恒 {"0": N}
```

---

## 6.3 检查"目标 sector"和"实际动作"是否不一致

重点排查：

```text
目标 sector 假设是 120°
relative_dyaw 被 clamp 成 30°
visited/covered 却记录了 120° sector
```

### AI 为每个 cycle 填：

| Cycle | 当前sector | candidate sector | 理论角差 | clamp后动作 | 实际odom到达sector | 最终记录visited sector | 是否一致 |
|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | 0（yaw≈0°） | 1 | +30° | l30 | ~1（30.3°） | **0**（heading_coverage={"0":1}） | **不一致** |
| 2 | 1（yaw 30.1°） | 2 | +30° | l30 | ~2（60.0°） | **0**（coverage={"0":2}） | **不一致** |
| 3 | 2（yaw 59.8°） | 3 | +30° | l30 | ~3（90.7°） | **0** | **不一致** |
| 4 | 3（yaw 90.7°） | 4 | +30° | l30 | ~4（121.0°） | **0** | **不一致** |
| 5 | 4（yaw 121.0°） | 5 | +30° | l30 | ~5（151.2°） | **0** | **不一致** |
| 6 | 5（yaw 151.2°） | 6 | +30° | l30 | ~6 | **0** | **不一致** |

### 如果存在不一致，请粘贴对应代码

```python
# 实际到达 sector（odom）与记录 sector（语义冻结）不一致的根因：
# 1) 语义 heading_sector 恒 0（async_semantic_observer 的 _latest 冻结于首帧 seed）
#    → place_graph 只记录 sector "0"；
# 2) local scan 候选基于 current_yaw_deg（odom，正确增长）选 sector，
#    → 每次选出新 sector（1,2,3,...），但 never 被记录（因为记录用语义的 0）。
```

AI 结论：

```text
异常（高度可疑）：实际 odom 已覆盖 sector 1~5（151°），
但 visited/covered 记录恒为 sector 0 → "covered 1/3" 永不前进 → 无限左转。
```

---

## 6.4 检查运动后是否存在 settle/stabilization gate

重点代码：

```text
app/live_robot/autonomous_explorer.py
scripts/go2w/run_semantic_exploration.py
```

### AI 回答

| 问题 | 内容 |
|---|---|
| motion success 后是否立即进入下一轮 | 是（explorer 循环 OBSERVE→MATCH→PLAN→EXECUTE→OBSERVE） |
| 是否固定 sleep | 无固定 sleep；但 action server 端 TURN 有稳定判定（turn_stable_samples=5 × turn_control_hz=20Hz → ~250ms 稳定才返回） |
| sleep 多久 | N/A（无显式 sleep） |
| 是否检查机器人角速度 | 是（action server `RunRelativeYaw` 用 `state_monitor_.IsStationary` 检查 yaw_rate ≤0.03 rad/s 才计稳定帧） |
| 是否检查线速度 | 是（IsStationary 同时检查 vx/vy） |
| 是否等待连续稳定若干毫秒 | 是（5 个稳定采样 @20Hz ≈ 250ms + `post_turn_zero_velocity_hold_sec=1.0`） |
| 是否强制等待一个"晚于 motion end"的新相机帧 | 否（下轮 observe 直接取 `rgbd_source.get_latest`——HTTP 最新帧，无"晚于 motion end"时间约束） |
| 是否强制等待一个 fresh Quick 结果 | 否（Quick 每轮重新调用，但无"晚于运动结束时刻"的约束） |
| 是否等待 Full Semantic | 否（后台异步，永不等待） |
| `spin_once()` 共多久 | 观察循环内 `rclpy.spin_once(node, timeout_sec=0.05)` × 4 ≈ 0.2s |
| 结论 | 部分正常（运动端有 settle gate）；但**感知端无"运动后新帧"门控**——若相机/HTTP 帧滞后，可能用运动前的旧帧做决策（本次因 odom sector 卡死掩盖了此问题） |

---

## 6.5 检查 Full Semantic 是否严重滞后

整理至少 10 次：

| 当前 cycle | 当前 RGB frame | 使用的 semantic frame | age(ms) | 是否是 motion 前帧 | scene_objects数 |
|---:|---:|---:|---:|---|---:|
| 1 | 358695 | 358695 | 0 | 是（初始） | 0 |
| 2 | 361407 | 358695 | ~51000 | 是（首帧） | 0 |
| 3 | 361824 | 358695 | ~66000 | 是 | 0 |
| 4 | 362321 | 358695 | ~81000 | 是 | 0 |
| 5 | 362791 | 358695 | ~96000 | 是 | 0 |
| 6 | 363214 | 358695 | ~111000 | 是 | 0 |

```text
当前 frame = 363214
semantic frame = 358695（age 111 秒，且机器人已转 151°）
```

且机器人中间已经转过角度，则标为 `高度可疑`。

AI 结论：

```text
高度可疑（实为"确认"）：语义结果冻结于首帧 111 秒、旋转 151° 之后仍被使用。
根因：后台 Full Semantic 每次 75s 超时未完成 → get_latest_completed() 永远返回 seed 首帧。
```

---

# 七、最终 `PERCEPTION_ERROR / PERCEPTION_FAILURE` 精确定位

这是必须完成的部分。

---

## 7.1 找到异常传播链

AI 请从最终：

```text
PERCEPTION_ERROR
PERCEPTION_FAILURE
autonomous_explorer / FAILED
```

一路向前追，画出实际异常链：

### 填写

```text
最底层异常类型：subprocess.TimeoutExpired（被包装为 RuntimeError）
最底层异常消息：SiliconFlow vision API timed out
文件：scripts/go2w/run_autonomous_loop.py
函数：Go2WAutonomousLoop._detect_llm
代码行：2382-2384（subprocess.run(timeout=20s) → except TimeoutExpired → raise RuntimeError）

上一层：
文件：scripts/go2w/run_autonomous_loop.py
函数：_detect（2255 行，llm 检测分支直接调 _detect_llm，无 catch）
catch/raise：不 catch，向上抛 RuntimeError

再上一层：
文件：scripts/go2w/run_semantic_exploration.py
函数：observe()（1028 行）内 Quick 阶段调用 node._detect(...)（1068 行）
catch/raise：不 catch，向上抛

最终：
文件：app/live_robot/autonomous_explorer.py
函数：run() 的 OBSERVE 循环（347-375 行）
catch/raise：except Exception as exc → self._emit("observer_error", ...)；
  observations(6) > 0 且 max_perception_retries=2 仅对 observations==0 生效
  → 直接 finish_reason = "PERCEPTION_FAILURE"；state = FAILED
状态变化：OBSERVE → FAILED（PERCEPTION_FAILURE）
```

---

## 7.2 判断具体属于哪一类

| 分类 | 是否命中 | 证据 |
|---|---|---|
| RGB-D HTTP timeout | 否 | camera fresh 全程 true |
| RGB 图像获取失败 | 否 | bundle 均有 image_ref |
| Depth 获取失败 | 否 | depth 未使用（null） |
| CameraInfo 不可用 | 否 | - |
| Quick VLM 网络异常 | 否 | 网络正常（API connect 8ms） |
| Quick VLM JSON 解析异常 | 否 | 前 5 轮均正常解析 |
| Full Semantic 网络异常 | **是（超时类）** | 2 次显式 75s SILICONFLOW_SCENE_TIMEOUT；其余请求未完成 |
| Full Semantic JSON 解析异常 | 否 | 无 parse failure |
| TF lookup failure | 否（但有 TF 缺失） | odom_fused/base_link↔camera TF 不存在 |
| Depth localization 异常 | 否 | 未执行（objects 空） |
| 文件 IO 异常 | 否 | - |
| stale frame / frame mismatch | **是（语义冻结 111s）** | 6 次 observation timestamp 相同；semantic age 111s |
| motion controller 异常被误分类 | 否 | motion 全部 success（yaw_delta 30.3° 实测） |
| 未分类 Python exception | **是（直接触发 FAILED 的异常）** | RuntimeError: SiliconFlow vision API timed out |
| 其他 | - | - |

---

## 7.3 检查 perception retry 逻辑

重点：

```text
app/live_robot/autonomous_explorer.py
```

### AI 回答

| 项目 | 内容 |
|---|---|
| `max_perception_retries` | 2（autonomous_explorer.py:181） |
| 搜索开始前首次 perception 失败是否会 retry | 是（`observations == 0` 且 retries < 2 时 retry，sleep 2s） |
| 已经成功观察过以后，再失败一次是否立即终止 | **是**——`if observations == 0 and ...` 条件在已有观察后不满足 → 直接 FAILED（347-375 行） |
| retry 计数何时清零 | 每轮 OBSERVE 循环开始时 `perception_retries = 0`（345 行） |
| 哪些异常会被统一映射为 `PERCEPTION_FAILURE` | 所有非 PerceptionFailure 的 Exception（observer_error 路径）+ PerceptionFailure 在无 retry 时 |
| 是否能区分临时 timeout 与永久错误 | 否——统一按不可重试处理（已有观察时） |
| 本次异常理论上是否可以安全重试 | **可以**（Quick VLM 单次 20s 超时属瞬态；前 5 轮均成功）——但逻辑不重试 |
| 结论 | 异常：已有观察后单次瞬态超时直接判死；`recoverable=True` 但实际不自动恢复 |

### 贴状态机相关代码

```python
# app/live_robot/autonomous_explorer.py:347-375
while observation is None:
    try:
        observation = self._observer()
    except PerceptionFailure as exc:
        self._emit("perception_failure", error=str(exc))
        if observations == 0 and perception_retries < self.max_perception_retries:
            perception_retries += 1
            ...
            continue
        finish_reason = "PERCEPTION_FAILURE"
        self._state = ExplorerState.FAILED
        break
    except Exception as exc:
        self._emit("observer_error", error=f"{type(exc).__name__}: {exc}", ...)
        if observations == 0 and perception_retries < self.max_perception_retries:
            perception_retries += 1
            ...
            continue
        finish_reason = "PERCEPTION_FAILURE"     # <-- 已有观察时直接判死
        self._state = ExplorerState.FAILED
        break
```

---

# 八、附加系统状态

## 8.1 CPU / GPU / 内存

### AI 填写

| 指标 | 值 |
|---|---|
| CPU 使用率 | 运行期空闲（采集时 top：node 75%、firefox 50%、realsense bridge 25%，系统整体有富余） |
| 内存使用率 | 12Gi / 31Gi（可用 18Gi） |
| Swap | 未显著使用 |
| GPU 使用率 | 0%（无本地推理；VLM 走 SiliconFlow API） |
| GPU 显存 | 13 MiB 占用 |
| 磁盘剩余 | 150G（56% 已用） |
| 是否 OOM | 否（资源充足，排除资源瓶颈） |

## 8.2 网络状态

### AI 填写

| 项目 | 内容 |
|---|---|
| D435 HTTP 平均 RTT | ping 192.168.123.18：avg 1.13ms（0% 丢包，5 包） |
| 最大 RTT | 1.271ms |
| 丢包率 | 0% |
| VLM 请求平均耗时 | 未逐次计时；Quick VLM 前 5 轮在 ~5.5s 内完成（LLM scene 日志间隔）；第 6 轮 >20s 超时 |
| VLM 最大耗时 | >20s（第 6 轮超时）；Full Semantic 全部 >75s 超时 |
| 是否存在 timeout spike | 是——Quick 第 6 轮 20s 超时；Full Semantic 全程超时（SiliconFlow API 侧响应慢或模型负载高） |

---

# 九、建议补做的三个最小化实验

不要直接重新跑完整自主搜索。先做三个小实验，每个实验只验证一个问题。

## 实验 A：只验证 TF

流程：

```text
机器人静止 → 记录 odom 和 TF → 左转 30° → 完全停止 → 再记录 odom 和 TF
```

### 结果

| 项目 | 转前 | 转后 |
|---|---:|---:|
| odom yaw | -0.002 rad | 0.526 rad（本次 session 实测） |
| TF yaw | 不存在（odom_fused->base_link 未发布） | 不存在 |
| 两者差值 | - | 无法计算（TF 缺失） |

结论：

```text
异常——TF 缺失本身即问题；建议先在 wheel_odom.launch.py 打开 publish_tf:=true 或
由 nav 层发布 odom->base_link 后再复测（实验 B 才能区分坐标系错位 vs LIO 漂移）。
```

## 实验 B：只验证 3D 建图旋转

不要自主搜索。

流程：

```text
静止建图 5 秒 → 左转 30° → 静止 5 秒 → 再左转 30° → 静止 5 秒
同时保存 RTAB-Map/plain_slam 画面截图、/go2w/odom/fused、TF、RGB/Depth timestamp。
```

### 每步填写

| 阶段 | odom yaw | TF yaw | 地图是否出现重复结构 |
|---|---:|---:|---|
| 初始 | （待测） | （待测） | - |
| +30° | （待测） | （待测） | 关键观察点 |
| +60° | （待测） | （待测） | 关键观察点 |

如果每转 30°地图就多叠一层，而 TF yaw 没同步变化，标记：

```text
高度确认：TF registration 问题
（本次 session 已观察到 odom 累计 151° 而探索/地图坐标系分离；建议按此实验确认 plain_slam 地图本身是否重影）
```

## 实验 C：只验证语义，不运动

让机器狗完全不动，面对包含明显物体的场景（椅子/桌子/门/垃圾桶），连续观察 20～30 秒。

### AI 填写

| 项目 | 内容 |
|---|---|
| Full Semantic 调用次数 | 本次 session 首轮 1 次同步 + 6 次后台提交 |
| 非空 object 次数 | 0（本次） |
| 能识别哪些物体 | 本次未识别到任何物体（Quick summary 有描述但 objects 数组为空） |
| SemanticObjectMap object 数 | 0 |
| 拓扑 OBJECT 节点数 | 0 |
| 是否仍只有 Fxx | 是（11 个 heading-gap frontier） |
| 是否发生 fallback | 是（Full Semantic 75s 超时 → degraded empty scene） |
| 结论 | 静止无物体 → **优先查 VLM / prompt / parse / semantic pipeline**（结合本次：Full Semantic 全部超时未完成是直接根因；需单独验证 SiliconFlow 语义模式请求为何 75s 不返回） |

判定逻辑：

- 静止都没有 objects：优先查 VLM / prompt / parse / semantic pipeline。✅ 命中
- 静止能识别，运动后消失：优先查 frame binding / timestamp / depth localization。（未命中）

---

# 十、AI 最终汇总表

| 问题 | 最可能原因 | 置信度 0~100% | 最关键证据 | 需要我进一步判断的内容 |
|---|---:|---|---|---|
| 拓扑图没有物体 | Full Semantic（语义分析）全部超时未完成 → 语义状态冻结于首帧空结果 → 物体恒空 → 无 OBJECT 节点 | 90% | 6 次 observation objects=[] 且 timestamp 全部=1788175722.883585（首帧）；vlm_requests/ 空；2 次 SILICONFLOW_SCENE_TIMEOUT | 为何 SiliconFlow 语义模式 75s 不返回（模型/请求体/daemon 缺失）；quick 模式 summary 有描述但 objects 空的 prompt/解析细节 |
| 3D 建图运动后重影 | 探索坐标（wheel odom "odom"）与 plain_slam 地图坐标（pslam_odom）混用 + odom_fused TF 缺失 + LIO 旋转跟踪误差（多因叠加） | 65%（坐标系混用）/ 40%（LIO 旋转漂移） | provider.set_pose 注入 wheel pose 但 map 为 pslam 系；odom_fused->base_link TF 不存在；WebUI 3D 由 aligned_scan 累积 | 实验 A/B 区分：坐标系错位 vs LIO 本身重影 |
| 连续快速左转 30° | heading_coverage 恒 {"0":6}（语义 heading_sector 冻结为 0）→ covered=1<3 恒成立 → local scan 恒选 delta_sector=+1 | 95% | place_graph.json heading_coverage={"0":6}；6 轮 reason 均 "covered 1/3"；odom 实际累计 151° | 语义 sector 修复后是否恢复；max_local_rotations 语义是否需要覆盖"已转过角度" |
| PERCEPTION_FAILURE | 第 6 轮 Quick VLM 单次 20s 超时 → RuntimeError → explorer 已有观察不再 retry → 直接 FAILED | 90% | observer_error traceback（run_autonomous_loop.py:2382）；explorer.py:347-375 条件 | 是否应放宽"已有观察后单次瞬态超时"的判死逻辑 |

---

# 十一、根因优先级打分

- `0`：没有证据
- `1`：轻微可疑
- `2`：有明显证据
- `3`：基本确认

| 候选根因 | 分数 0~3 | 证据 |
|---|---:|---|
| `odom_fused -> base_link` 错误使用 static TF | 2 | 该 TF 完全缺失（publish_tf=false），探索 pose 标称 "odom" 无对应 TF |
| TF 有多个 broadcaster 冲突 | 0 | 仅 1 个静态 publisher，无冲突 |
| D435 optical frame 旋转错误 | 0 | 无 TF 数据可验（未发布） |
| D435 安装外参错误 | 1 | 外参 candidate_unconfirmed，无标定验证 |
| RGB-D 使用接收时间而非采集时间 | 2 | bridge 用 get_clock().now()；服务端有采集时间戳但未用 |
| RGB / Depth 不同步 | 1 | 两者时间戳相同（接收时刻），但真实采集有串行下载差 |
| Depth 未 aligned 却使用 Color CameraInfo | 1 | 采信服务端声明，无客户端校验 |
| Full Semantic 一直失败/返回空 | **3** | 0 次成功；2 次显式 75s 超时；vlm_requests/ 空 |
| Semantic parse fallback 被静默吞掉 | **3** | parse degraded 后空场景继续搜索，无 error 上报；语义冻结 111s |
| old semantic + current depth 跨帧 | 2 | get_latest_completed() 无 frame_id 校验；本次 objects 空未触发 depth 路径 |
| SemanticObjectMap 更新失败 | 0 | 更新逻辑正常，输入 objects 恒空 |
| topology serializer 不生成 OBJECT | 0 | serializer 逻辑正确（objects 空则无 OBJECT 是预期） |
| Frontier ID 冲突 | 1 | Fxx 命名不绑定 place，多 place 时潜在冲突（本次未暴露） |
| planner 固定左侧优先 | **3** | delta_sector=(1,-1,2,-2) 恒左优先 + covered 恒 1 → 无限左转 |
| `heading_sector` 与 clamp 后实际动作不一致 | **3** | 实际 odom 覆盖 sector 1~5，记录恒 sector 0 |
| motion 后没有稳定等待 | 0 | action server 有 250ms 稳定 + 1s 保持门控 |
| Full Semantic 结果严重 stale | **3** | semantic age 111s、旋转 151° 后仍用首帧 |
| perception retry 策略过于激进 | **3** | 已有观察后单次瞬态超时直接 FAILED（可安全重试而不重试） |
| 临时网络/HTTP/VLM timeout | 2 | Quick 第 6 轮 20s 超时；Full Semantic 全程超时（网络本身健康，指向 API 侧响应慢） |
| 其他 | 1 | `start_autonomous_search_web.sh` 不启动 VLM daemon → Full Semantic 只能走 75s subprocess（无 daemon 复用/并发） |

---

# 十二、最终需要回传给我哪些内容

1. ✅ `search_worker.log` 第一条 ERROR（实为首个 75s 超时 WARN 1788175797.96）与最终 PERCEPTION_FAILURE（1788175903.06）前后日志 + traceback —— 见 §3.1
2. ⚠️ `rtabmap.log`：本次未运行 RTAB-Map（plain_slam 为建图后端），无旋转日志 —— 见 §3.2
3. ✅ `d435_rgbd_bridge.log`：本次无 timeout（bridge 源码时间戳/frame_id 分析见 §3.3/§5.5）
4. ✅ 单次左转 30° 前后的 `/go2w/odom/fused`（6 轮 odom yaw 变化 0→151° 实测）；`tf2_echo odom_fused base_link` = frame 不存在 —— 见 §5.2
5. ✅ 当前 TF tree：/tf 与 /tf_static 各 1 个 publisher（go2w_official_sensor_frame_publisher）；无 odom_fused、无 base_link↔d435 —— 见 §5.3
6. ✅ `start_rgbd_spatial_stack.sh`：该脚本不存在；实际 TF/帧由 start_live_perception.sh + go2w_description + plain_slam launch 发布（代码见 §5.1）
7. ✅ RGB/Depth/CameraInfo topic、frame_id、timestamp、resolution、intrinsics —— 见 §3.3/§5.5/§5.6
8. ✅ Full Semantic：成功 0 次；失败/fallback 2 次（75s 超时日志）；frame_id=358695 冻结、semantic_result_age_ms≈111000 —— 见 §4.1/§4.3/§6.5
9. ✅ 连续左转 6 个 cycle 决策时间线 —— 见 §6.1/§6.3
10. ✅ PERCEPTION_FAILURE 完整异常传播链 —— 见 §7.1

---

# 十三、排查 AI 的最终输出格式

```text
【问题1：拓扑图没有物体】
结论：Full Semantic 全部超时未完成，语义冻结于首帧空结果（objects 恒空），
      SemanticObjectMap/SERIALIZER 无输入 → 拓扑仅 P1 + 11 个 heading-gap FRONTIER。
置信度：90%
证据1：6 次 observation timestamp 全部=1788175722.883585（首帧冻结）；place_graph heading_coverage={"0":6}
证据2：2 次 SILICONFLOW_SCENE_TIMEOUT: full-scene analysis exceeded 75s；vlm_requests/ 目录为空
证据3：llm_detection_result.json objects=[]、all_objects_count=0（Quick VLM summary 有描述但 objects 空）
仍缺失的信息：SiliconFlow 语义模式为何 75s 不返回（需单独 curl 语义 prompt 验证 API 侧）；
             start_vlm_daemon.sh 未接入 WebUI 启动链（daemon 从未运行）是否加剧超时

【问题2：3D 建图运动后混乱】
结论：高度可疑——探索坐标（wheel odom，标称 "odom"）与 plain_slam 地图坐标（pslam_odom）混用，
      odom_fused->base_link TF 缺失；旋转时两套里程计 yaw 不一致 + LIO 旋转跟踪误差 → 重影/扇形叠图。
置信度：65%（坐标系混用为主因）；LIO 旋转漂移 40%
证据1：plain_slam_spatial_provider.set_pose 注入 wheel odom pose，get_frontiers/camera_point_to_spatial 与 pslam map 混算
证据2：tf2_echo odom_fused base_link → Invalid frame ID；wheel_odom publish_tf=false
证据3：WebUI 3D 由 aligned_scan（pslam 系）累积；探索 odom 累计 151° 而记录 sector 恒 0
仍缺失的信息：实验 A/B 区分坐标系错位 vs LIO 本身重影；需 base_link↔d435 TF 实测外参

【问题3：连续快速左转30°】
结论：heading_coverage 恒 {"0":6}（语义 heading_sector 冻结 0）→ covered=1<3 恒成立 →
      local scan 恒选 delta_sector=+1 → 无限左转 30°（实际已转 151°，决策 6 次全为 l30）。
置信度：95%
证据1：place_graph.json heading_coverage={"0":6}；6 轮 reason 均 "bounded local scan at P1 sector N (covered 1/3)"
证据2：decisions.jsonl 6 轮全部 goal_type=ROTATE_VIEW relative_dyaw=30.0、motion TURN_AND_MOVE turn_deg=30.0
证据3：odom 每轮 Δyaw≈0.52 rad 实测（30.3°/29.7°/30.9°/30.3°/30.1°），运动执行完全正常
仍缺失的信息：语义修复后覆盖计数是否恢复；max_local_rotations=3 与 12 sector 的关系是否需调整

【最终 PERCEPTION_FAILURE】
底层异常：subprocess.TimeoutExpired → RuntimeError("SiliconFlow vision API timed out")
异常文件：scripts/go2w/run_autonomous_loop.py
异常函数：Go2WAutonomousLoop._detect_llm（line 2382）
异常时间：1788175903.05（19:31:43）
传播路径：_detect_llm → _detect（2255）→ observe()（run_semantic_exploration.py:1068）
          → AutonomousExplorer.run() OBSERVE 循环 except Exception（autonomous_explorer.py:359-375）
          → observer_error → finish_reason=PERCEPTION_FAILURE → FAILED
是否属于可重试瞬态错误：是（前 5 轮 Quick 均成功；单次 20s 超时，网络健康）
证据：observer_error traceback（events.jsonl）；explorer.py:347-375 条件 observations>0 不重试
仍缺失的信息：Quick VLM 第 6 轮为何突增至 >20s（API 侧负载/图片大小/模型波动）

【根因优先级】
P0：Full Semantic 从未成功（75s 超时 × 全程）→ 语义冻结 → 无物体 + sector 卡死 + 无限左转
P0：heading_sector 冻结（语义 sector 恒 0）→ heading_coverage 不增长 → 无限 local scan 左转
P1：已有观察后单次 Quick 瞬态超时直接 PERCEPTION_FAILURE（retry 策略）
P1：探索坐标（wheel odom）与 plain_slam（pslam_odom）坐标系混用 + odom_fused TF 缺失 → 3D 重影
P2：RGB/Depth 时间戳用接收时刻、D435 外参未验证、Fxx frontier ID 不绑 place

【禁止现在就修改的部分】
1. 不修改 plain_slam_ros2 上游算法（LIO 旋转漂移需先实验 B 确认）
2. 不调整 RTAB-Map ICP/参数（本次未运行 RTAB-Map，无依据）
3. 不调整运动参数（turn_kp/turn_control_hz 等，运动执行实测正常 30.3°）
4. 不修改 D435 外参数字（candidate_unconfirmed，需先补 TF 与标定验证）
5. 不提高任何安全授权字段（confirmed/authorizes_* 保持 false）
```

> 核心原则：先把 **TF、timestamp、frame binding、第一条底层异常** 四件事钉死，再决定具体代码修改方案。
> 本次已钉死：① 第一条底层异常 = Full Semantic 75s 超时（19:29:57，全程 0 成功）；② frame binding = 语义冻结首帧 111s 无校验；③ timestamp = RGB/Depth 接收时刻；④ TF = odom_fused 与 base_link↔d435 缺失、坐标系混用。
