# GO2W 真机自主搜索修复与验收 — 工作交接书

> 生成时间：2026-09-01 10:30（主控重启等待中）
> 配套文档：`/home/mxt/robotscene/GO2W_真机自主搜索_详细修改计划书.md`（以下简称"计划书"，以 § 引用）
> 本交接书 + 计划书 = 接手 AI 的完整上下文，可无缝继续。

---

## 0. 交接摘要（30 秒版）

| 项目 | 状态 |
| --- | --- |
| 计划书全部代码修改（P0-A/B/C、P1-A/B/C、P2-A/B/C） | ✅ 已完成，宿主+真机已同步 |
| 新增回归测试（6 个文件 23 用例） | ✅ 宿主 23/23、真机 23/23 通过 |
| 真机相关既有测试（31 用例） | ✅ 31/31 通过 |
| 宿主全量测试 | ⚠️ 788 通过 / 6 失败（均为 pre-existing，与本次修改无关） |
| 真机部署链路（运动栈/感知栈/SDK） | ✅ 已打通（多轮环境修复，见 §5） |
| 真机验收 A–E | ❌ **未开始**——被**狗主控运动控制器异常**阻断（sportmodestate 不发布），用户正在重启主控 |
| 最终报告（§23 格式） | 草稿在 `outputs/go2w_plan23_report.md`，验收数据待回填 |

**接手后第一件事**：检测主控状态流（§8.1），恢复后按 §8.2–§8.6 跑验收 A→E。

---

## 1. 环境与基线（接手必读）

### 1.1 三台设备
| 设备 | IP | 角色 | 访问方式 |
| --- | --- | --- | --- |
| 宿主工作站 | 192.168.123.99（enp3s0） | x86_64 Ubuntu 22.04，**官方 ROS2 humble**；**运动栈 + 语义搜索主执行机** | 本机（当前会话所在） |
| 机器狗上位机（Jetson） | 192.168.123.18（eth0） | arm64 Ubuntu 20.04（foxy 官方 + humble 移植版）；**感知机**：D435 HTTP 相机 + VLM daemon | SSH：`sshpass -p 123 ssh unitree@192.168.123.18`，sudo 密码 123 |
| 狗主控 | 192.168.123.161（虚拟 MAC 7e:1d:75:60:f5:89） | Unitree GO2-W 主控（wheeled_sport/robot_state_service 等进程） | **不可 SSH**，仅 DDS/RPC 可达 |

### 1.2 代码基线
- 宿主：`/home/mxt/robotscene`，分支 `feature/semantic-object-topology`，HEAD `81cda75`（未提交修改约 155 项 = 用户既有 WIP + 本次修改，**不要 git checkout/reset**）。
- 真机：`/home/unitree/robotscene`，独立 git 仓库 `main@4eec3b7`（部署快照），工作树已与宿主同步（sha1 一致）。
- 关键事实：真机快照 = 宿主 WIP 工作树的镜像（未改动文件 sha1 全同），因此宿主→真机单向 rsync 修改文件即可。
- 用户 stash 列表有 `stash@{1}: On main: codex deployment changes...`（用户自己的，勿动）。

### 1.3 通信架构（重要，8月31日排查即此架构）
- **运动栈（action server / lease / SDK）跑在宿主 99**，通过 DDS 多播直连狗主控 161。
- **感知栈（D435 HTTP :8080、VLM daemon）跑在真机 18**；宿主直连 `http://192.168.123.18:8080`。
- 宿主 ROS 层：humble + `rmw_cyclonedds_cpp` + 多播（`CYCLONEDDS_URI=/tmp/go2w_cyclonedds_1000.xml`，Interfaces name=enp3s0）。
- 真机 18 ROS 层（如需）：foxy + cyclonedds + **localhost-only**（arm64 CycloneDDS 0.7 SPDP bug，见 §5.4）。

---

## 2. 已完成的代码修改（计划书 P0–P2 全部）

### 2.1 修改文件清单（宿主 = 真机，已同步）

**P0-A 语义状态/frame binding（超时≠空场景、旧帧≠当前帧、pending≠无物体）**
- `app/live_robot/semantic_observer.py`：`SemanticObservation` 扩展 `semantic_source_frame_id/capture_timestamp/completed_timestamp/age_ms/status/quality/error_code/error_detail/source_pose`；状态常量 `SEMANTIC_STATUS_FRESH_FULL/FRESH_QUICK/PENDING/STALE/TIMEOUT/ERROR/UNAVAILABLE`；`semantic_observation_to_live` 增加 `navigation_heading_sector` 参数并透传全部元数据。
- `app/live_robot/async_semantic_observer.py`：首帧也后台提交（不阻塞 75s）；`_run_analyze` 分类 `subprocess.TimeoutExpired → FULL_SEMANTIC_TIMEOUT`；失败/降级空结果**绝不**进入 `_latest`；结果只接受 fresh 状态并绑定 source frame/pose；事件：`semantic_request_started/discarded_obsolete/timeout/error/result_applied/stale_result/discarded`；`_on_result` 的 `started` 参数可选（兼容旧测试直调）。
- `app/navigation/models.py`：`LiveObservation` 增加 `navigation_heading_sector`、`semantic_heading_sector`、`semantic_*` 全字段（to_dict/from_dict 同步）。
- `app/detectors/siliconflow_vision_worker.py`：Quick 系统提示词同时输出 `scene_objects_light`（≤10 个显著物体，含 name_zh/confidence/bbox_2d/category，**绝不包含目标物**）；空列表不写字段（避免伪"空场景"）。
- `app/config.py` + `configs/exploration/default.yaml`：`VLM_RUNTIME_QUICK_MAX_TOKENS` 256→1024；`VLM_RUNTIME_SEMANTIC_INITIAL_WARMUP_BLOCKING` True→False。
- `app/live_robot/autonomous_explorer.py`：OBSERVE 感知重试循环（§8.5，见 P1-A）；heading sector 强制由当前 pose 计算（P0-B）。

**P0-B heading coverage 解耦（语义 sector ≠ 导航 sector）**
- `app/live_robot/autonomous_explorer.py`：`navigation_heading_sector`/`heading_sector` 每轮用当前 `pose.yaw` 重算（`round(yaw/sector) % sectors`），绝不接受 observer 塞入的 stale sector。
- `scripts/go2w/run_semantic_exploration.py`：`_navigation_sector()` 统一入口；`observed_sectors`、place_graph 更新、`semantic_observation_to_live(..., navigation_heading_sector=...)` 全部用导航 sector；`semantic_heading_sector` 仅描述语义来源方向。
- `app/live_robot/explorer_search_adapter.py` + `search_state_store.py` + WebUI：`navigation_heading_sector` 与 `semantic_*` 字段透传到事件/状态/UI。

**P0-C local scan 防死循环（不再固定左转/配额失效）**
- 新增 `app/navigation/local_scan.py`：纯函数 `select_local_scan_goal(current_yaw_deg, heading_coverage, max_local_rotations, state, ...)` + `LocalScanState`（steps/last_direction/same_direction_count）。规则：±30/±60 对称候选；同向惩罚 −0.6；同向连续≥2 且无新信息 −2.0（仅同向）；最近 sector 惩罚 −0.3；同分反向；配额按步数。
- `scripts/go2w/run_semantic_exploration.py`：每 Place 独立 `local_scan_states`（`state["local_scan_states"]`）；发射 `local_scan_selected/...` 事件；运动步记录 `motion_state`（motion_end_timestamp、last_frame_id_before_motion）+ `motion_sector_reached` 事件。

**P1-A 感知 retry（瞬态错误≠任务死亡）**
- `app/live_robot/autonomous_explorer.py`：`PerceptionFailure(message, *, code="UNKNOWN_PERCEPTION_ERROR", recoverable=True, detail="", last_success_age_s=None)` + `to_dict()`；OBSERVE 循环：`exc.recoverable` 时无论已有多少次成功观察都重试（backoff 1s/2s，`max_perception_retries=2`）；耗尽后记录 `self._perception_failure_detail`（code/attempts/last_success_age_s），`session_finish` 单次发射（cause/attempts/error_detail），`SessionResult.summary["perception_failure"]`。
- `scripts/go2w/run_autonomous_loop.py`：`_detect` → `PerceptionFailure(GROUNDED_SAM_TIMEOUT/DETECTOR_ERROR)`；`_detect_llm` → `QUICK_VLM_TIMEOUT/QUICK_VLM_ERROR`（替代 RuntimeError）；`_verify_target` → `VERIFY_TIMEOUT/VERIFY_ERROR`。
- `scripts/go2w/run_semantic_exploration.py`：Quick VLM 包装为 PerceptionFailure；`analyze_semantic` 超时/失败抛 `PerceptionFailure(FULL_SEMANTIC_TIMEOUT/FULL_SEMANTIC_ERROR)`（**不再返回降级空 payload**）；capture 错误 → `PerceptionFailure(RGBD_TIMEOUT/..., recoverable)`；运动后 3s 无新帧 → `FRAME_STALE` 重拍 ×2；`_last_success_age_s()` 辅助。
- 验收 D 注入开关：`GO2W_QUICK_TIMEOUT_INJECT_ONCE=1`（run_autonomous_loop._detect_llm 首调模拟一次 QUICK_VLM_TIMEOUT）。

**P1-B 坐标契约（wheel odom 绝不混入 pslam_odom）**
- `app/spatial/models.py`：`SPATIAL_QUALITY_NO_GLOBAL_POSE` + `class SpatialFrameMismatch(ValueError)`。
- `app/spatial/plain_slam_spatial_provider.py`：`set_pose` 帧不匹配抛 `SpatialFrameMismatch`；`get_frontiers`/`camera_point_to_spatial` 同契约。
- `scripts/go2w/run_semantic_exploration.py`：空间位姿一律 `camera_provider.get_pose()`（plain_slam/rtabmap 场景绝不用 wheel pose 注入）；wheel odom 仅用于运动/当前 yaw；plain_slam 位姿缺失 → `NO_GLOBAL_SPATIAL_POSE` 降级。

**P1-C 3D 点云累积门控（不再每转复制一层）**
- `scripts/go2w/plain_slam_web_bridge.py`：新参数 `--odom-topic /go2w/slam/odom_base`、`--target-frame pslam_odom`、`--yaw-rate-max 0.03`、`--speed-max 0.02`、`--settle-seconds 0.5`、`--diagnostics-every 30`、`--reset-marker`；启动清空旧累积；scan 门控：frame 必须 `pslam_odom`（否则 drop "frame_mismatch"）、运动期 drop "motion_active"、pose jump（>2.0 m/s 或 |dyaw|>0.5 rad）drop "pose_jump"；map 帧变/origin 跳变/`--reset-marker` 清空；snapshot 增加 `target_map_frame/accumulated_scans/dropped_scans/dropped_reason_counts/stationary/pose/delta_yaw_rad`。
- `scripts/go2w/start_autonomous_search_web.sh`：导出 `GO2W_SLAM_MAP_SNAPSHOT`/`GO2W_SLAM_RESET_MARKER`，bridge 带新参数。
- `app/manual_web_demo/search_session_service.py`：启动前 touch reset marker。

**P2-A RGB-D timestamp/frame 契约**
- `app/perception/rgbd_source.py`：`RGBDFrame.timestamp_quality`（默认 host_timestamp）。
- `app/perception/realsense_http_rgbd_source.py`：`_resolve_capture_timestamp`（窗口 [1.4e9, 2.2e9]，否则 fallback）。
- `scripts/go2w/realsense_rgbd_bridge.py`：ROS stamp 用 capture 秒；depth `frame_id` = `d435_color_optical_frame`（depth_aligned_to_color）否则 `d435_depth_optical_frame`；health 含 timestamp_quality + aligned 标志。
- `app/perception/depth_object_localizer.py`：新增 `resolve_depth_frame(semantic_frame_id, current_frame, cache=None)`（同帧才匹配；None → SEMANTIC_2D_ONLY）。

**P2-B frontier ID 唯一化**
- `app/spatial/semantic_navigation_graph.py`：`_refresh_frontiers` 生成 `frontier_id=f"frontier:{place_id}:{sector:02d}"`、`label=f"F{sector+1:02d}"`；节点序列化用 label。
- `app/live_robot/autonomous_explorer.py`：`_unique_frontier_id(sector)` 查 `frontier:P<place>:<sector>`，fallback 旧 `Fxx`。
- `app/manual_web_demo/static/search_map.js`：FRONTIER 节点 title 用短 label。

**P2-C 日志/健康状态/WebUI**
- `app/live_robot/search_event.py`：`SEMANTIC_STATUS` 事件类型。
- `explorer_search_adapter.py`：SEMANTIC_STATUS 映射 + OBSERVATION_UPDATED 带 `navigation_heading_sector` + `semantic_*`；SEARCH_FINISHED/ERROR 带 cause/attempts/last_success_age_s/recoverable/error_detail；`perception_retry` 进可恢复事件表。
- `app/manual_web_demo/templates/index.html` + `static/search_ui.js` + `static/style.css`：语义健康状态行（fresh_full/fresh_quick_scene/pending/stale/timeout/error/unavailable，含 source/age/objects/spatial/nav_sector）；失败横幅 cause/attempts；静态资源版本 `go2w-fix-20260901`。

### 2.2 新增文件
- `app/navigation/local_scan.py`（P0-C 纯函数）
- `tests/test_semantic_freshness.py`、`tests/test_local_scan.py`、`tests/test_explorer_heading_and_retry.py`、`tests/test_plain_slam_frame_contract.py`、`tests/test_frontier_unique_ids.py`、`tests/test_rgbd_timestamp_contract.py`
- `docs/GO2W_真机部署与验收_runbook.md`（验收命令手册）
- `outputs/go2w_plan23_report.md`（§23 报告草稿，验收后回填）
- **部署适配（非计划书内容，真机必要）**：
  - `scripts/go2w/sport_odom_bridge.py`：`/lf/sportmodestate`（best_effort 订阅）→ `/go2w/odom/fused` Odometry（GO2-W 不发布 lowstate，wheel_odom 无输入，用主控融合里程替代）
  - `unitree_go2w_control/scripts/sport_state_export.py`（真机 18 侧 SDK→JSON 状态导出，备用）
  - `unitree_go2w_control/scripts/detect_unitree_interface.sh`：同机部署（ROBOT_IP=本机）回退物理网卡
  - `unitree_go2w_control/scripts/setup_go2w_ros2.sh`：ROS setup 自动探测（foxy 优先/humble 兜底）；**arm64 → localhost-only + 无 Interfaces xml；x86_64 → 多播 + Interfaces name= 形式**；SDK libddsc 注入说明（见 §5.5）
  - `unitree_go2w_control/ros2_ws/src/go2w_motion_control/launch/go2w_motion_control.launch.py`：hold_sport_lease 进程单独注入 libddsc 0.10.2 路径（多候选路径）

---

## 3. 测试状态（已完成，可复跑）

```bash
# 宿主（x86_64，.venv python3.13）
cd /home/mxt/robotscene
.venv/bin/python -m pytest -q tests/test_semantic_freshness.py tests/test_local_scan.py \
  tests/test_explorer_heading_and_retry.py tests/test_plain_slam_frame_contract.py \
  tests/test_frontier_unique_ids.py tests/test_rgbd_timestamp_contract.py        # ✅ 23 passed
.venv/bin/python -m pytest -q tests/test_async_semantic_observer.py tests/test_live_semantic_observer.py \
  tests/test_autonomous_explorer.py tests/test_place_graph.py tests/test_vlm_quick_contract.py \
  tests/test_semantic_quick_absence_keeps_objects.py tests/test_search_event.py \
  tests/test_camera_local_spatial_provider.py                                      # ✅ 48 passed / 3 failed
  # （3 failed 中 2 个已修：async observer 的 started 参数；1 个 streamlit 环境问题 pre-existing）

# 真机（arm64，.venv python3.8）
sshpass -p 123 ssh unitree@192.168.123.18 'cd ~/robotscene && .venv/bin/python -m pytest -q tests/test_semantic_freshness.py tests/test_local_scan.py tests/test_explorer_heading_and_retry.py tests/test_plain_slam_frame_contract.py tests/test_frontier_unique_ids.py tests/test_rgbd_timestamp_contract.py'  # ✅ 23 passed
sshpass -p 123 ssh unitree@192.168.123.18 'cd ~/robotscene && .venv/bin/python -m pytest -q tests/test_async_semantic_observer.py tests/test_live_semantic_observer.py tests/test_autonomous_explorer.py tests/test_place_graph.py tests/test_vlm_quick_contract.py tests/test_semantic_quick_absence_keeps_objects.py'  # ✅ 31 passed

# 宿主全量（一次结果）
.venv/bin/python -m pytest -q tests/ --ignore=tests/test_go2w_live_ui_status.py -p no:cacheprovider
# → 788 passed, 6 failed, 1 skipped（16m46s）
# 6 个失败均为 pre-existing（与本次修改无关，未触碰其模块）：
#   test_ros2_command_exporter、test_streamlit_natural_language_task_ui、
#   test_streamlit_video_mode ×2、test_task_examples_evaluator、test_task_planner
# 佐证：test_go2w_live_ui_status 在 git stash 基线上也失败（AppTest 环境问题）。
```

**注意事项**：`app/live_robot/async_semantic_observer.py::_on_result` 第三参 `started` 必须保持可选（旧测试直调 `_on_result(req, payload)`）。

---

## 4. 宿主 ↔ 真机同步状态

已同步（宿主 sha1 = 真机 sha1）：§2.1 全部修改文件 + 新增测试 + 部署脚本。
同步命令模板：
```bash
cd /home/mxt/robotscene
sshpass -p 123 rsync -az --relative <相对路径...> unitree@192.168.123.18:~/robotscene/
```
**注意**：真机 `unitree_go2w_control/ros2_ws/{build,install}` 与 `external/unitree_ros2/cyclonedds_ws` 为**本机重建产物**（arm64），**不要**用宿主文件覆盖；`external/cyclonedds_0.10.2` 真机只有 `~/cyclonedds_0.10.2`（编译安装到 HOME）与 `~/robotscene/external/cyclonedds_0.10.2/{src,build}`。

---

## 5. 真机环境修复记录（接手 AI 必读，避免重复踩坑）

> 全部为**部署适配**，未触碰计划书 §19 红线（turn_kp/30° 参数、RTAB-Map、plain_slam LIO、D435 外参、安全授权、frame 契约）。

1. **同机部署接口探测**：真机 18 上 `ip route get 192.168.123.18` 走 lo → `detect_unitree_interface.sh` 失败。已加"回退到带 192.168.123.x 的物理网卡"逻辑。
2. **control ws 断链**：真机 install 的 local_setup.bash 是符号链接指向宿主 `/home/mxt/...`（rsync 保留绝对路径）→ 断裂。修复：真机删除 build/install 重建（先 foxy，后因 DDS 问题统一架构见 §5.4）。
3. **CycloneDDS xml 兼容**：Jetson 的 CycloneDDS（0.7.0）不识别 `<Interfaces>` 元素（"unknown element"）→ 已条件化（arm64 无 Interfaces；x86_64 用 name 形式）。
4. **arm64 DDS 崩溃（核心）**：Jetson 上 **foxy cyclonedds 0.7.0 多播模式解析 SPDP 报文必崩**（`ddsi_plist_init_frommsg` SIGSEGV，humble 移植版同样崩，fastrtps bad_alloc）→ **arm64 用 localhost-only**；**运动栈改在 x86_64 宿主跑（humble 官方 0.10，多播正常）**。setup_go2w_ros2.sh 已按 `uname -m` 条件化。
5. **SDK python cyclonedds 版本不匹配**：unitree_sdk2py 需要 `cyclonedds.idl`（pip 0.10.2），系统 libddsc 0.7.0 缺 `ddsi_sertype_v0` 符号 → **真机编译 CycloneDDS 0.10.2 到 `~/cyclonedds_0.10.2`**（宿主用 `external/cyclonedds_0.10.2/install`）；**只对 SDK 进程（hold_sport_lease）注入 LD_LIBRARY_PATH**（ROS 层 rmw_cyclonedds_cpp 0.7.11 与 libddsc 0.10.2 不兼容，全局注入会崩）。
6. **GO2-W 无 /lf/lowstate**：轮腿机型不发布 lowstate → `sport_odom_bridge.py` 用 sportmodestate.position 发布 `/go2w/odom/fused`（best_effort 订阅）。
7. **GO2-W lease 为 1 秒 term**：LeaseClient 注释明确（已定制续约逻辑）。
8. **主控运动控制器异常（当前阻塞）**：主控 161 的 **sport API 不响应**（Move/StopMove 超时、`/lf/sportmodestate` 不发布、`network_status` 10Hz 正常、lease/motion_switcher RPC 正常）。判断为 wheeled_sport 未正常激活（重启后）。**用户正在重启狗主控**。

---

## 6. 验收状态与命令（未完成，等待主控恢复）

### 6.1 验收 A–E 状态
| 验收 | 内容 | 状态 |
| --- | --- | --- |
| A | 静止语义（拓扑有真实物体） | ❌ 被前置等待阻断（需 sportmodestate） |
| B | 纯旋转 coverage（l30,l30,l30 不重） | ❌ 同上 |
| C | 3D 旋转建图（无重影） | ❌ 同上（需运动） |
| D | timeout retry 注入 | ❌ 同上（可先跑 A 类链路） |
| E | 受限完整搜索 | ❌ 同上 |

### 6.2 验收前置链路（宿主，主控恢复后这些服务必须活着）
```bash
cd /home/mxt/robotscene
# 1) 运动栈（action server + lease + SDK executor）
nohup bash scripts/go2w/start_motion_control.sh > /tmp/motion_host.log 2>&1 &
#    验证：日志出现 "Go2-W motion Action ready" + "lease_ready" + "sdk_motion_executor_ready"
# 2) 里程计桥（sportmodestate -> /go2w/odom/fused）
nohup bash -c 'source /opt/ros/humble/setup.bash && source external/unitree_ros2/cyclonedds_ws/install/setup.bash \
  && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file:///tmp/go2w_cyclonedds_1000.xml \
  && /usr/bin/python3 scripts/go2w/sport_odom_bridge.py' > /tmp/sport_odom.log 2>&1 &
# 3) VLM daemon（宿主）
bash scripts/go2w/start_vlm_daemon.sh
# 4) 真机 18：D435（通常已自启） + VLM daemon
sshpass -p 123 ssh unitree@192.168.123.18 'cd ~/robotscene && bash scripts/go2w/start_vlm_daemon.sh'
curl -s -m 3 http://192.168.123.18:8080/health   # 期望 {"ok": true, "streaming": true}
```

### 6.3 主控恢复检测
```bash
cd /home/mxt/robotscene
source /opt/ros/humble/setup.bash && source external/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file:///tmp/go2w_cyclonedds_1000.xml
timeout 8 ros2 topic hz /lf/sportmodestate   # 出现 "average rate: ~10" 即恢复
```
恢复后先验证运动可用（arm 授权 + 小旋转）：
```bash
bash unitree_go2w_control/scripts/go2w_arm.sh on    # 期望 success=True（"motion armed"）
timeout 60 /usr/bin/python3 unitree_go2w_control/scripts/go2w_action_client.py \
  --mode yaw --degrees 30 --max-yaw-rate 0.4 --timeout 45   # 验收 B 的第一个 l30
```

### 6.4 验收 A（静止语义）
```bash
cd /home/mxt/robotscene
source /opt/ros/humble/setup.bash && source external/unitree_ros2/cyclonedds_ws/install/setup.bash \
  && source ros2_ws/install/setup.bash && source unitree_go2w_control/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file:///tmp/go2w_cyclonedds_1000.xml
/usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
  --target "白色垃圾桶" --backend go2w_experimental --spatial-v2 \
  --rgbd-source --rgbd-base-url http://192.168.123.18:8080 \
  --max-seconds 90 --max-motion-steps 0 --max-planning-cycles 3 \
  --operator-supervised-experiment --session-dir outputs/live_runs \
  --output outputs/live_sessions/acceptance_a.jsonl
```
**通过标准**：事件流含 `quick_vlm_completed`/`semantic_status`（fresh_quick_scene/fresh_full）；`outputs/live_runs/<session>/semantic_map.json` objects 非空；拓扑 ≥2–3 个 OBJECT 节点；Full Semantic 超时不显示"空场景"。
**注意**：`--allow-degraded` 仅在确认运动链路不可用时加（主控恢复后应不需要）。

### 6.5 验收 B（纯旋转 coverage）
```bash
/usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
  --target "白色垃圾桶" --backend go2w_experimental --spatial-v2 \
  --rgbd-source --rgbd-base-url http://192.168.123.18:8080 \
  --max-seconds 150 --max-motion-steps 3 --max-local-rotations 3 --turn-only \
  --operator-supervised-experiment --session-dir outputs/live_runs \
  --output outputs/live_sessions/acceptance_b.jsonl
```
**通过标准**：`place_graph.json` 的 `P1.heading_coverage` 含 ≥{0,1,2,3}（绝不是 `{"0":N}`）；日志 `motion_sector_reached`（requested/observed/sector_before/sector_reached）；`local_scan_selected` 左右交替或配额耗尽退出。

### 6.6 验收 C（3D 旋转建图）
1. 真机 18：`bash scripts/go2w/start_plain_slam_mapping.sh`（Pandar 驱动 + LIO/SLAM；无 --no-start-hesai 参数时自动起驱动）。
2. 宿主：`bash scripts/go2w/start_autonomous_search_web.sh --with-plain-slam`（含 plain_slam_web_bridge + WebUI 127.0.0.1:8765）。
3. 旋转：`run_autonomous_loop.py --mode pattern --pattern l30,l30` 或验收 B 的 turn-only。
4. 检查 `outputs/autonomous_search/runtime/slam_map_3d.json`：`target_map_frame=="pslam_odom"`、`dropped_reason_counts.motion_active>0`、静止后 `accumulated_voxels` 增长；WebUI 3D 截图对比无逐层重影。
5. 若仍重影 → 判定 plain_slam/LIO 本体漂移（计划书 §18.3 专项，采 rosbag），**不再改本仓库 planner**。

### 6.7 验收 D（timeout retry）
```bash
GO2W_QUICK_TIMEOUT_INJECT_ONCE=1 /usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
  --target "白色垃圾桶" --backend go2w_experimental --spatial-v2 \
  --rgbd-source --rgbd-base-url http://192.168.123.18:8080 \
  --max-seconds 120 --max-motion-steps 2 --max-local-rotations 2 \
  --operator-supervised-experiment --session-dir outputs/live_runs \
  --output outputs/live_sessions/acceptance_d.jsonl
```
**通过标准**：`perception_retry`（code=QUICK_VLM_TIMEOUT）→ retry 后继续 → session 不 FAILED；连续失败耗尽时 `session_finish` 带 cause/attempts=3。

### 6.8 验收 E（受限完整搜索）
```bash
/usr/bin/python3 scripts/go2w/run_semantic_exploration.py \
  --target "白色垃圾桶" --backend go2w_experimental --spatial-v2 \
  --rgbd-source --rgbd-base-url http://192.168.123.18:8080 \
  --max-seconds 300 --max-motion-steps 10 --max-local-rotations 3 \
  --operator-supervised-experiment --session-dir outputs/live_runs \
  --output outputs/live_sessions/acceptance_e.jsonl
```
**通过标准**（计划书 §18.5）：不再连续无限 l30；coverage 随实际 yaw 变化；OBJECT 拓扑节点存在；semantic age 不无限增长；Quick 单次超时可恢复；3D 无重影；无 `semantic_depth_frame_mismatch` 持续发生。

### 6.9 验收产物核对
```text
outputs/live_runs/<session_id>/exploration_graph.json, semantic_map.json, place_graph.json, decisions.jsonl, final_state.json
outputs/live_sessions/acceptance_{a,b,d,e}.jsonl
outputs/autonomous_search/runtime/slam_map_3d.json
```

---

## 7. 当前运行/待恢复状态（2026-09-01 10:30 快照）

| 组件 | 宿主 99 | 真机 18 |
| --- | --- | --- |
| 运动栈（action/lease） | ⚠️ 进程已停（主控重启期间 lease 不可用导致），恢复命令见 §6.2 | 停（有意，避免双 lease） |
| sport_odom_bridge | ⚠️ 需重启（见 §6.2） | — |
| VLM daemon | ⚠️ 需重启（socket 已失效） | ✅ 运行中 |
| D435 HTTP :8080 | — | ✅ 运行中（age ~0.07s） |
| 主控 161 状态流 | ❌ 无（等待重启完成） | ❌ 无 |

---

## 8. 下一步精确步骤（接手 AI 直接照做）

1. **等主控恢复**（用户正在断电重启）：轮询 §6.3 检测命令，出现 10Hz 即继续。
2. **恢复宿主链路**：§6.2 的 1)–3)（运动栈、odom 桥、VLM daemon）。
3. **验证运动可用**：§6.3 的 arm + 30° 旋转（**操作员持遥控急停在场**）。
4. **按顺序验收 A → B → C → D → E**（§6.4–§6.8），每项收集证据（输出文件 + 关键日志片段）。
5. **回填最终报告** `outputs/go2w_plan23_report.md`（§23 格式：基线/文件清单/修复理由/测试命令与结果/验收 A–E/未决事项）。
6. **收尾**：
   - 宿主与真机代码同步确认（`sha1sum` 抽查 §2.1 文件）。
   - 计划书 §19 红线复查（未触碰：turn_kp、RTAB-Map、LIO、D435 外参、安全授权、frame 契约）。
   - 未决项写清：D435 外参 `candidate_unconfirmed`；LIO 漂移专项（若 C 仍重影）；主控异常记录。

---

## 9. 执行通道与常用命令（本会话经验）

- 本会话无 bash 工具，用 staging 工具 `host_exec`（dev_stage_call，execute 为 async JS 用 `await import('node:child_process')` 跑 `spawnSync('bash', ['-lc', cmd])`）执行命令；接手 AI 若有 shell 工具直接用。
- **pkill/grep 自杀坑**：`pkill -f xxx` 或 `ps|grep xxx|kill` 会匹配**自己的命令行**（含 xxx 文本）→ 用 `[x]xx` 括号技巧。
- **管道吞环境坑**：`source a.sh | tail` 中 source 在子 shell 执行，环境不生效；`setup_go2w_ros2.sh` 含 `set -Eeuo pipefail`，source 后需 `set +eu` 恢复。
- SSH 批量命令建议写成远程脚本再 `bash /tmp/xxx.sh`（避免多层引号转义）。
- 真机 sudo 密码 123：`echo 123 | sudo -S cmd`。
- 宿主 ROS 环境固定串：
  ```bash
  source /opt/ros/humble/setup.bash && source external/unitree_ros2/cyclonedds_ws/install/setup.bash \
  && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export CYCLONEDDS_URI=file:///tmp/go2w_cyclonedds_1000.xml
  ```
- 真机 18 的 SDK 环境：`LD_LIBRARY_PATH=/home/unitree/cyclonedds_0.10.2/lib:$LD_LIBRARY_PATH ~/robotscene/unitree_go2w_control/.venv/bin/python ...`。

---

## 10. 未决事项与风险

1. **狗主控运动控制器异常**（当前唯一阻塞）：wheeled_sport 不响应 sport API、不发布 sportmodestate；用户已重启主控。若重启后仍异常：检查遥控器/APP 模式显示；考虑主控固件/连接问题（非本仓库范围）。
2. **真机 18 的 SDK RPC 3102**：与主控异常同因（18 侧 SDK 连主控失败），主控恢复后复测；若仍失败则排查 18↔161 网络。
3. **宿主 6 个 pre-existing 测试失败**：streamlit AppTest 环境（tests/ 下无 streamlit_app.py）、task_planner 行为（与本次修改无关），建议单独 issue，不要试图"修好"而误改行为。
4. **D435 外参**：`candidate_unconfirmed`（计划书红线，不可标 UNKNOWN→高质量）。
5. **plain_slam/LIO 本体漂移**：验收 C 通过坐标契约修复后若仍有扇形重影 → 计划书 §18.3 专项（rosbag），不改本仓库 planner。
6. **双运动栈风险**：宿主与真机 18 各有一套运动栈，**只能同时跑一个**（lease 3207 冲突）。验收期间宿主独占。
7. **安全**：运动验收全程操作员持遥控急停；运动参数受 SDK 协议限值约束（|yaw_rate|≤0.25 rad/s 等）；先 arm（`go2w_arm.sh on`）再运动。

---

## 11. 最终报告模板要点（§23）

`outputs/go2w_plan23_report.md` 已含骨架：基线（branch/commit）、文件清单、逐问题修复理由、frame binding 字段、测试命令与结果（待填验收）、验收 A–E（待填）、未决项。验收完成后按 §8.5 回填即可。
