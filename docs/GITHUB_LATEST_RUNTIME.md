# Go2-W 项目 GitHub 运行上下文

更新时间：**2026-09-16 18:10**（Asia/Shanghai）
§22 补记：**2026-09-16 20:21** —— 只补了 §22 验收 B 的结果与判据 1 的修订，
其余各节仍是 18:10 的实测值，未重新采集。

> **阅读顺序**：先读下面「2026-09-16 现状」，再读交接书
> `docs/GO2W_断电中断与验收A完成交接_20260916.md`。
> `## 2026-09-01 20:51 接管复核` 及其之后的全部内容**保留为 historical regression evidence，一字未删**
> （§34 原话：「不要把旧异常记录删掉；标记为 historical regression evidence」）。
> 若与最新一节冲突，以最新一节为准。

---

## 2026-09-16 现状（最新）

### 新 commit / 部署版本

| 项 | 值 |
|---|---|
| 主机 HEAD | `c376fac0926611294d11fe3c61030b3dfbbb7bce`（2026-09-16） |
| 分支 | `fix/go2w-autonomous-search-lio-topology-target` |
| 主机工作树 | **大量未提交**（含本批新增的 17 个 test 文件与 6 个新模块） |
| 机器狗 `/home/unitree/robotscene` | HEAD `0de647e`（**2026-09-04**），工作树 94 项修改 |

⚠ **两侧历史已分叉，这是 2026-09-16 新查出来的事实。** 狗上 HEAD `0de647e`
在本机仓库里**不存在**（`git cat-file -t` → `fatal: Not a valid object name`）。
主机 09-14/15/16 的六笔提交（Hesai 外参 90.0、关系边帧竞争、拓扑两视图、LIO nofix 闸门、
Place 身份、4 个事件）**狗上都没有**。**上真机运动之前必须先解决这一条。**

### 最新 session id

| 用途 | session |
|---|---|
| 探索（15:55，含真实原地旋转） | `explore_go2w_20260916_155531` → `TARGET_FOUND`（白色垃圾桶），2 步，Place=1 |
| 搜索（11:08） | `search_20260916_110826_3f7ac850` → `MAX_PLANNING_CYCLES_REACHED`，12 步，57 objects，Place=1，`navigation_failures=0` |
| §21 验收 A 静止取证 | `artifacts/slam_validation/level_a_static_20260916/` |

### LIO timestamp mode

`configs/go2w/plain_slam_bridge.yaml:17` → **`timestamp_mode: auto`**（未改）。
§21 判的 "IMU corrected stamp" 只对 `/go2w/slam/imu` 成立（戳超前 `/utlidar/imu` **830.8 s**）。

### extrinsic status

| 外参 | 状态 |
|---|---|
| **Hesai PandarXT-16（LIO 输入）** | **已修并验收**：`init_yaw_rad` **11.357° → 90.0°**，实测 pslam 朝向偏角 **−77.4° → +0.38°**（`7976a1d`）。附带拆除了 `calibrate_pandarxt16_extrinsics.py` 里从 11.357 出发的 ±15° 搜索窗（那颗定时炸弹永远搜不到 90°） |
| **D435（RGB-D）** | **仍 `candidate_unconfirmed`**（计划书红线，不许标 UNKNOWN→高质量）。**未动。** |

### rotation drift 指标

| 项 | 值 |
|---|---|
| §21 静止 60 s pslam XY drift | **0.0135 m / 0.0035 m**（两轮独立取样，目标 <0.10 m） |
| 一次 ~33° 原地旋转（非 §22 协议） | wheel Δxy **0.0035 m**，Place 仍 **1**；pslam Δxy **本批未采** |
| §22（原地分扇区旋转） | **已跑（2026-09-16 20:09），未全通过** —— 8 条判据 7 ✓ / 1 ✗，唯一缺的是判据 5 的 pslam Δxy（**无数据**）。逐条判定见 `GO2W_断电中断与验收A完成交接_20260916.md` §17 |
| §23（360°） | **NOT RUN** —— §22 未全通过，按 §22「失败立即停，不进入 §23」的规定不启动 |

### Place count / 3D map result

```
Place count       = 1（current_place = P1）
heading_coverage  = {"0":1,"1":2,"2":1}   ← 覆盖 {0,1,2}，3 个扇区（18:08 快照，来自 15:55 那次运行）
```

> **§22 判据 1 已于 2026-09-16 修订**：原文写 `heading_coverage ⊇ {0,1,2,3}`，
> 其隐含的「3 次**同向** 30°」与 `app/navigation/local_scan.py:77-83` 故意惩罚同向旋转的
> 设计**互斥**（`:90` 注释「同向时优先选择与上次相反的方向（避免无限单向旋转）」），
> **重跑只会得到同样结果**，故改判据文字而非改代码。修订后 = 「覆盖 **≥3 个不同扇区**」，
> 权威文本见 `docs/GO2W_真机验收_交接书.md` §6.5，代码证据与实测见上文交接文档 §17.4。
>
> 按**修订后**判据，2026-09-16 20:09 那次运行的 `heading_coverage = {"0":1,"1":1,"11":1}`
> （扇区轨迹 0→1→11→10）是**达标**的；**但 §22 整体仍判未通过**，
> 卡在判据 5 —— 两次运行都没能在段间独立采到 pslam Δxy。别把这一条读成「§22 已过」。

`GET :8765/api/slam/map3d`（2026-09-16 18:08 实测）：

```
available=true          point_count=7184        source_map_points=17626
mapping_health=HEALTHY  lio_pose_valid=true     map_status=LIVE
fresh=true              mapping_session_id=1    canonical_frame=pslam_map
ros_stamp=1789545394    generated_at=1789553278  ← ros_stamp 比 generated_at 冻 7884 s
```

`GET :8765/api/search/readiness`：`ready=true`、`blocking=[]`、
`degraded=["search_worker_available"]`、`owner=NONE`。

⚠ **上面 `ros_stamp` 冻 2.2 小时而 `mapping_health` 仍报 `HEALTHY` —— 这是交接书 §6 那个
闸门盲区的现场复现，不是新 bug。** 同一时刻 `[GO2W-DIAG]` 的实测是
`imu_n=26 pt=0.1030 iter=1 fix_t=0.0039 pos=(0.616,0.224,-0.012)`（**LIO 完全健康且在流**），
所以这 2.2 小时冻的是「机器人静止 ⇒ 没有新关键帧 ⇒ 地图不重发」，
**`ros_stamp` 冻住与 LIO 死亡同形**。盲区本身仍是**未修**状态（等你定 `N`）。

### target preemption result

**真机未触发。** 代码侧 `target_sweep`（§44.7/§44.8）的单测全覆盖
（`tests/test_target_sweep_trigger.py`，6 tests），但"连续空 candidate 达阈值 ⇒
VLM full-frame sweep 触发 / 有 candidate 时不 sweep"这条链**在真机上还没有一次被观察到**。
本批未做（需要真机连续空帧场景）。

### 仍未解决风险

| # | 风险 | 状态 |
|---|---|---|
| 1 | **§13.5 闸门对「LIO 输入消失」不表态** | **未修，且在等你拍板**（`N` 取多少、要不要"现场手动清"出口）。见上文实测复现 |
| 2 | **机器狗/主机历史分叉** | **未处理**。运动验收前必须解决，否则验收对象不是同一份代码 |
| 3 | **§22 / §23 / §24–§28 全部未执行** | 需要运动授权。§23 计划书明写「只有 §22 通过才允许执行」 |
| 4 | D435 外参 `candidate_unconfirmed` | 计划书红线，未动 |
| 5 | 仓库根 3 份未跟踪计划书 + `reports/cc_switch_codex_takeover_report_3.20.1.md` | 等表态 |
| 6 | 两条计划-代码不一致：§44.1 的 `living room`/`wood floor` 删不掉、§44.6 的 `_verification_region` 无距离门 | **已如实记录，未擅自"修好"**。见 `outputs/go2w_autonomous_search_fix_report.md` §33.4 |
| 7 | `outputs/autonomous_search/logs/*.log` 被 git 跟踪且体积巨大（单个 91 万行） | 未处理。会让每次 `git status` 都很难看 |

---

## 2026-09-01 20:51 接管复核（historical regression evidence）

- 清理主机遗留的第二个 `plain_slam_web_cloud_relay`；`/go2w/slam/web_scan` 当前仅 1 个
  publisher，远端 3D bridge 仍正常接收。
- 最新真实 WebUI 会话 `search_20260901_205129_fdb39af8` 回放为 `TARGET_FOUND`，目标状态
  `PRESENT`，Full Semantic 错误 0，Place 仅 `P1`，未发送物理运动 goal。
- 机器狗端地图连续复核为 `HEALTHY`，`pslam_odom`、`lio_pose_valid=true`、
  `motion_odom_seen=true`、`dropped_scans=0`；当前点数约 5209–5212。
- 启动自检返回码 0；`lidar_fresh`、`spool_bundle` 仍是明确降级项。`rotation_clearance_valid`
  为 false，故 `motion_authorized/safety_authorized` 继续为 false。

## 2026-09-01 整改后现场状态（historical regression evidence —— 标题里的"最新"已失效，保留原文）

- 整改代码已部署到机器狗 `/home/unitree/robotscene`；覆盖前的 20 个远端源码文件已保存到
  `/home/unitree/robotscene/.codex_backups/20260901/`。
- 机器狗端 `go2w_plain_slam_bridge` 已在 ROS 2 Foxy 环境中构建成功；新增 IMU 时间对齐器、
  三态目标门控、空间位姿校验、几何关系提取和独立运动里程计门控均已随正式启动链路部署。
- 当前现场服务：WebUI 已在机器狗 `0.0.0.0:8765` 监听；D435 HTTP capture、VLM daemon、
  plain_slam display relay 和 3D Web bridge 均在线。浏览器地址为
  `http://192.168.123.18:8765/`，三维地图接口为 `/api/slam/map3d`。
- 当前只读验收实测：相机 `1280x720` 且 fresh；VLM `Qwen/Qwen3-VL-8B-Instruct`
  返回 `status=ok`；地图 `available=true`、`mapping_health=HEALTHY`、
  `frame_id=pslam_odom`、`point_count≈5820`、`motion_odom_seen=true`、
  `lio_pose_valid=true`、`stationary=true`。跨主机显示使用紧凑 relay topic
  `/go2w/slam/web_scan`，不改变运动控制坐标链。
- 真实 WebUI 后端只读干跑 `search_20260901_173622_fa9e0a38` 经真实相机/VLM
  返回 `TARGET_FOUND`（办公椅），`planning_cycles=0`、`motion_steps=0`，没有发送运动 goal；
  worker 正常退出并完成归档。此前 `search_20260901_165344_61d612fa` 也曾通过同一链路。
- 当前安全状态：WebUI owner 为 `NONE`、命令为 `stop`、`motion_in_flight=false`，正式自主
  运动保持关闭。机器狗本机已有且仅有 1 个 `/go2w/motion` action server，
  `/go2w/arm`、`/go2w/emergency_stop` 在线，readiness 为 `ready=true`。
- 真机运动验收仍需在清空场地、确认 `/go2w/odom/fused` 唯一发布者、确认机器人周围安全且
  遥控器可立即急停后，由现场操作员显式授权；在此之前不发送真实运动 goal。

- 本仓库对应机器狗项目目录：`/home/unitree/robotscene`。
- 主机工作区：`/home/mxt/robotscene`。
- 机器狗地址：`192.168.123.18`。
- WebUI：`http://192.168.123.18:8765`（运行在机器狗系统；mapping-assist 的传感器链仍在
  主机 `192.168.123.99`，通过 DDS + display-only relay 提供三维显示）。
- 机器狗端 D435 HTTP `/health`：最近检查 HTTP 200。
- 机器狗端 SiliconFlow VLM daemon：在线。
- 为避免主机工作树与机器狗现场工作树混淆，机器狗源码另存于
  `github_artifacts/robot_project_snapshot_20260901/`：来源是机器狗的
  `/home/unitree/robotscene`，同步时现场 HEAD 为 `4eec3b7`，当时工作树有 459 个修改项。
  该快照包含 1,870 个源码/配置/文档文件，未包含机器狗 `.git` 元数据、`.env`、虚拟环境、
  第三方依赖、ROS 编译产物和运行日志。

## 整改前历史运行状态（归档）

- plain_slam mapping-assist：运行中，独立 session 启动。
- 3D 显示数据源：`/go2w/slam/map_3d` + `/go2w/slam/aligned_scan`。
- 显示/累积坐标系：`pslam_odom`；display-only bridge 不发布运动控制数据。
- 最新快照：`available=true`、`fresh=true`、`stationary=true`，`frame_id=pslam_odom`，
  `target_map_frame=pslam_odom`。
- 动作服务可用；最近任务结束后为 `DISABLED/stop`，无动作在途。
- 手动 turn 按钮因正式四向 rotation-clearance 尚未标定而保持安全禁用；操作员监督实验
  通道只允许已确认的 ≤30° 原地转向。

## 最近验收证据

- A：静止语义通过；真实 D435/VLM 产生物体和 frontier 拓扑。
- B：`outputs/live_sessions/acceptance_b_20260901_114550_turns.jsonl`，3 个 30°
  原地转向成功，导航失败为 0。
- C：`outputs/live_sessions/acceptance_c_20260901_120316_turns.jsonl`，两步成功；
  运动期 `motion_active` 丢帧，停稳后 voxel 继续累积，错误帧被拒绝。
- D：一次 Quick VLM timeout 注入后 retry 恢复，无运动伪成功。
- E/WebUI：
  - `search_20260901_120501_96392a00`：自主搜索权限开启、turn-only、1 个受限原地转向，
    结果 `MAX_STEPS_REACHED`（预算结束，非错误）。
  - `search_20260901_121219_0a3965d1`：通过 WebUI 首帧确认“垃圾桶”，结果
    `TARGET_FOUND`，`motion_steps=0`。

## 最新追加运行（需后续 AI 优先关注）

会话 `search_20260901_122525_25291a89`（目标“白色垃圾桶”）是在上述验收之后产生的
最新运行记录，不应与已完成的受限验收混为一谈：

- 前 9 个原地扫描动作均成功，观测 yaw 约 29.64°–31.53°；第 5 周期虽然观测到
  30.79°，但动作结果被记录为失败，随后按恢复策略继续。
- 第 10 周期出现 `FULL_SEMANTIC_ERROR`，最终会话以 `FAILED/WORKER_INTERRUPTED`
  结束；这是需要后续排查的最新异常，不代表机器狗未停机。
- 最终安全状态：`motion_in_flight=false`、命令为 `stop`、动作状态 `DISABLED`，
  相机和 mapping 仍在线。

## 给后续 AI 的读取顺序

1. 先读 `GO2W_真机自主搜索_详细修改计划书.md` 和 `docs/GO2W_真机验收_交接书.md`。
2. 再读 `outputs/go2w_plan23_report.md` 获取验收结论和未决风险。
3. 搜索事件读取 `outputs/live_runs/<session_id>/events.jsonl`、`webui_state.json`、
   `summary.json`。
4. mapping 运行日志读取 `github_artifacts/robot_runtime_logs/` 及
   `outputs/autonomous_search/logs/`。
5. 如果需要判断机器狗现场代码，以
   `github_artifacts/robot_project_snapshot_20260901/` 为准；仓库根目录代码是主机项目工作树
   的提交快照，两者不要按同一个 Git HEAD 推断。

## 上传边界

- 未上传 `.env`、API token、密码、虚拟环境、模型权重、第三方仓库、ROS build/install
  产物、相机 JPEG/PNG 缓存和运行 PID 文件。
- 机器狗运行日志已脱离第三方依赖目录集中归档；代码和配置保留为可阅读源码。
- 仍需人工标定：D435 外参 `candidate_unconfirmed`；原地旋转后的 LIO 平面漂移需按
  IMU 外参、时间同步和 deskew 专项继续分析。
