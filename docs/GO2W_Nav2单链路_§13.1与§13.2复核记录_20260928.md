# §13.1 与 §13.2 复核记录（2026-09-28）

对应计划书：`docs/GO2W_Nav2自主搜索单链路改造计划.md`
分支：`fix/go2w-autonomous-search-lio-topology-target`（宿主 `/home/mxt/robotscene` ↔ 机器狗 `/home/unitree/robotscene`）

本记录只覆盖 **AI 可完成的部分**：§13.1 全部、§13.2 中不涉及现场判断的部分。
§13.3 / §13.4 按计划书第 13 节只能由现场人员完成，不在本记录内。
`configs/go2w/navigation_gate.yaml` 的 `nav2_allow_execute` 全程保持 `false`，未改。

---

## 1. §13.1 离线验证（六项，全部完成）

| # | 项目 | 结果 | 证据 |
| --- | --- | --- | --- |
| 1 | 修改的 Python 文件语法检查 | ✅ | 37 个改动 `.py` 全部 `py_compile` 通过；`ros2_ws` 子树另查一遍通过 |
| 2 | 本文涉及的 pytest | ✅ | 计划书 §12 对应六个测试文件 **118 passed**（3.19s） |
| 3 | 受影响 ROS 2 package 的 `colcon build` | ✅ | **13 packages finished**（41.7s），0 失败 |
| 4 | package tests | ✅（功能性全绿） | 见下 §1.1 |
| 5 | `rg` 验证生产路径无 `RELATIVE_MOVE` / `go2w_experimental` 回退 | ✅ | 见下 §1.2 |
| 6 | `git diff` 只含本文范围、不覆盖用户改动 | ✅ | 本次三个提交只动 4 个文件；工作区里 74 项用户在制品未被触碰 |

第 2 项的六个文件：`test_plain_slam_nav_frame_bridge.py`、`test_nav2_search_backend.py`、
`test_corridor_analyzer.py`、`test_nav2_autonomous_search_contract.py`、
`test_experiment_readiness.py`、`test_search_worker_python.py`。

### 1.1 第 4 项的完整口径

`colcon test` 剩余 60 条失败，**没有一条是功能性失败**，逐类已定性：

| 类别 | 条数 | 定性 |
| --- | --- | --- |
| `uncrustify` | 33 | 全在 `plain_slam_ros2/include/plain_slam/*.hpp`。该包是 SLAM 上游，本计划 §4.2 明说不替换 SLAM 算法，与本次改动无关 |
| `flake8` / `pep257` | 4 | 全在 `scripts/imu_timestamp_aligner.py` 一个文件上（1×E501 + 3×D 类） |
| `xmllint` | 2 | **不是代码问题**：它要去 `download.ros.org` 拉 `package_format3.xsd`，本机无外网 ⇒ 恒失败 |
| `go2w_lio_bringup` pytest | 1 (error) | **宿主环境限制**：`ModuleNotFoundError: No module named 'unitree_go'`，那是机器狗的厂商消息包，宿主本就没有 |

`go2w_lidar_preprocessor` 原本 1 failed，已修，现 **39 passed**（见 §3.3）。
按计划书 §3.1「不做无关重构」，上表几项均未顺手改。

### 1.2 第 5 项的判据与结论

计划书 §13.1-5 要求验的是**生产搜索路径没有 `go2w_experimental` 回退**。结论：满足，
且有三道独立守卫，全部由通过的合同测试钉住（`test_nav2_autonomous_search_contract.py`）：

1. **请求边界**：`SearchStartRequest().backend == "nav2"`；`load_search_defaults()["backend"] == "nav2"`；
   传 `go2w_experimental` 或 `mock`/`mock_metric` 时 `validate()` 返回 `unsupported backend`
   （断言到原因串本身，不是只断言"非 None"）。
2. **worker 边界**：`autonomous_search_worker.build_argv` 默认 `nav2`，遇非 nav2/mock 直接抛错。
3. **启动器源码**：`start_semantic_exploration.sh` 内不含 `go2w_experimental`，且真实路径转发调用方的 backend、不写死。

**残留（判断，非缺陷）**：`scripts/go2w/run_semantic_exploration.py:400` 的 `--backend`
choices 里仍有 `go2w_experimental`。按 §4.3「go2w_experimental 文件可以暂时留在仓库，
避免破坏独立诊断脚本」保留——摘掉它会让两个**活的**诊断脚本失效：
`validation/run_semantic_D.sh`、`validation/run_semantic_static.sh`（语义观测采集，
自身写明"worker 不自己发运动"，被多份交接文档引用）。该选项在生产路径上不可达（三道守卫），
只能由人工显式敲。若评审认为应按字面完全移除，需同时为这两个脚本另找出路。

---

## 2. §13.2 静止上电验证（七项）

计划书原文：「现场人员运行，但必须先在文档给出命令」。命令见 README §6.4。
下表区分「AI 已测（只读、无需现场判断）」与「须现场实跑」。

| # | 项目 | 状态 | 结果 |
| --- | --- | --- | --- |
| 1 | 不使用 `--enable-motion` 启动 | 现场 | 命令见 README §6.4；启动器默认 `execution_enabled=false` |
| 2 | `/map` 的 `frame_id=map` | 现场 | 命令见 README §6.4 第 1 项（Foxy 无 `--once`/`--field`，已按实测改写） |
| 3 | TF 主链连续且无重复发布者 | 现场 | 命令见 README §6.4 第 3 项。**前置：轮式里程计必须先起**，见 §3.1 |
| 4 | 静止 10 min，`map -> odom_fused` 无无解释大跳变 | ✅ **已在当前栈重测** | `samples=1190  max_xy_jump=0.0042 m  max_yaw_jump=0.096°`（1200×0.5 s）；栈修复前的旧值 `0.0016 m / 0.019°` 一并留档，见 §5 |
| 5 | 多个 `ComputePathToPose`：路径在膨胀区外、偏向走廊中间 | ✅ **硬判据已在当前栈重测** | 见下 §2.1 |
| 6 | `/go2w/nav2_cmd_vel` 即使有规划也不进机器人 | ✅ **已在当前栈重测** | `Publisher count=0`；`/go2w/cmd_vel_selected` 发布者 1；`/cmd_vel` 不存在 |
| 7 | 走廊分析日志含宽度、最小侧向净空、判定、速度上限 | ⏳ **现可查，须跑一次新会话** | 见下 §2.2 |

### 2.1 第 5 项

硬判据全部成立：

- planner 对**自身格子即为 254** 的目标和**出图**目标一律 ABORT（`planning_time=0`）。
- 全部返回路径里**没有任何一点落在代价 ≥ 253**（最差 238）。
- 阳性对照：`254=242  253=568  其它非零=1334  空闲=637856`，与 README 记录的基线
  `254=196  253=838` 同量级 ⇒ 判定脚本本身有分辨力，不是恒空。

**2026-09-28 当前栈重测 + 补一条更硬的正面对照。** README §6.4 第 5 项那个直方图断言
（"254/253 一个都没有就是空过"）**只是个下界**：静态层也会往代价地图里打 254，所以
"有 254"并不能证明障碍层真的在吃扫描。补做了一次分解——

```text
/map            800x800 res=0.100 origin=(-40,-40)  占据格=61
/global_costmap 800x800 res=0.100 origin=(-40,-40)  254=249  253=664
窗口内静态占据格上限 = 61      → 障碍层至少贡献 188 个致命格
```

`/map` 全图只有 61 个占据格，而代价地图里有 249 个致命格 ⇒ **至少 188 个只能来自
`/go2w/lidar/scan`**。这条比"有 254 就算过"有分辨力：障碍层没输入时差值会归零。

**同时排掉一条会伪装成它的故障**：当前栈的 `nav2_bringup.log` 里有 **21 条**
`Message Filter dropping message: frame 'base_link' ... for reason 'Unknown'`，
README 给的判据是"0，或只有启动瞬间那一两条"，21 条看着越界。逐条查过，**是启动竞态
不是时间轴失效**：

- 21 条全部落在 `1790579931.68 ~ 1790579932.88`，即 Nav2 起来后 **1.2 秒内**；
  此后 14 分钟一条都没有（日志 mtime 停在那一刻）。
- 被丢的消息时间戳落后约 **10.1 s** —— 正好是 TF 缓冲深度，符合"`map` 帧晚出现约 15 s、
  排队消息老化出缓冲"的竞态，而不是"帧时间戳偏了几百秒"。
- 直接量扫描帧落后量（不用推理）：`/go2w/lidar/scan` 为 **−1.9 s**、
  `/go2w/sensors/cloud` 为 **−1.6 s**，远在 10 s 缓冲内。
  这与时间拟合失效时的 **−867 s** 差两个数量级（见 §3.5 与
  `go2w-time-fit-dies-on-clock-change`）。

结论：第 5 项的两个前提（障碍层在吃扫描、时间轴没偏）在当前栈上都成立。

**保留意见**：costmap 的观测源只有 `/go2w/lidar/scan` 一条，而那条扫描在**正前方
最长连续 137.0° 无回波**（会跳，见 §3.2）。所以"路径不在膨胀区外"这条判据，在正前方方向上
**不构成"不会撞"的证据**——它只证明 planner 尊重它拿到的代价。§16 的约束据此继续有效。

### 2.2 第 7 项为何此前查不到、现在可以了

第 7 项要看的是 `navigation_result` 事件里的 `candidate_decision`。09-17 / 09-18 的会话
里确有 `navigation_result`（5/4/3 条），但**没有一条带 `candidate_decision`**——因为
把该字段接到事件顶层的两个提交（`5733bf2` 09-21 22:15、`397833b` 09-21 23:17）
**晚于所有已录会话**。

对应的离线钉子已通过：`tests/test_autonomous_explorer.py -k candidate_decision` → **2 passed**。
链路代码（`app/live_robot/autonomous_explorer.py`）此前**从未部署到机器狗**，本次已随
`deploy_plan_to_robot.sh` 同步（`REMOTE_SYNTAX_OK`）。所以现在跑一次新会话即可查。

---

## 3. 范围外发现（供评审，均未擅自扩大改动）

### 3.1 现场必须先起轮式里程计（否则 §13.2-3 无归因地失败）

`map -> odom_fused -> base_link` 的后半段由 `wheel_odom` 发布，而**上游没有任何脚本替
你发布它**：README §6.1 的 `start_plain_slam_mapping.sh` 明确"不会发布 `/go2w/odom/fused`"，
`start_nav2_autonomous_search.sh` 按"不代管上游"的约定既不启动也不等待它。

```bash
ros2 launch go2w_lio_bringup wheel_odom.launch.py
ros2 topic info /go2w/odom/fused -v | sed -n '/Publisher count/p'   # 必须是 1
```

已在 README §6.4 补前置说明，并在失败分支里打印以上两条命令（`61e7e2a`）。

### 3.2 正前方软件不设防（本次最重要的一条）

Nav2 全局与局部 costmap 的**唯一**观测源 `/go2w/lidar/scan`，在**正前方一段连续扇区里
一个回波都没有**。40 帧逐束聚合（720 束、0.5°/束）在**当前栈**上实测：

| 量 | 2026-09-2x 首测 | 2026-09-28 当前栈重测 |
| --- | --- | --- |
| 一次都没返回的束 | 285 / 720 (40%) | **295 / 720 (41%)** |
| 最长连续盲区 | −51.5°..+62.0° = 113.5° | **−68.0°..+69.0° = 137.0°** |
| 有限回波量程 | 0.37~1.38 m | 0.41~2.60 m（配置 0.37~6.20） |
| 前扇区全量 / 碰撞带内 | 606 / 0 | **2215 / 0** |
| 碰撞带上沿之上的占比 | ~91% | **91%** |

盲区**跨度在 113.5°~137.0° 之间跳**（同一天内两次重测分别为 133.0° 与 137.0°）。
上表"前扇区"用的是探针里**写死的查询窗口** `−51.5°..+62.0°`，它比实测盲区**窄**，
所以"碰撞带内 0 点"这个结论是按更保守的窗口给的，实际盲区只会更大不会更小。

原因是**高度**而不是方位：原始云变换到 base_link 后正前方**有**点（当前栈 2215 个，
3 m 内），但约 91% 在碰撞带上沿 `+0.052` 之上，带内 **0 点**。雷达看得见前上方，
看不见正前方的地面到膝盖。

**代码级证据（此前未记录）**：`app/live_robot/motion_bounds.py` 的
`evaluate_lidar_motion_readiness()` 只拒绝 `lidar_fresh is not True` / `is None` / `isnan`。
`inf` 三者都不是，`inf < minimum_clearance_m` 为假 ⇒ 返回 `MotionBoundaryDecision(True)`。
函数注释写着 fail closed，但没覆盖"前方没有任何观测"这个真实故障态——而那正是这台狗的常态。
`configs/go2w/lidar_preprocess.yaml` 的 `front_stop_clearance_m: 0.6` **全仓库无代码读取**，是死值。

**反面证据（界定范围）**：机器人**后方**碰撞带内实测有真实回波（0.35~2.06 m、轮子高度），
2026-09-28 现场确认那是一个真实物体 ⇒ 碰撞带本身是工作的，坏的只是前向那一段。

**处置**：经现场拍板「照常验收、人工兜底」，已写入 README §9.1 与计划书 §13.3 的
"执行前必读"，并标注 §13.3 那条「遇临时障碍时 Collision Monitor 能先减速再停止」
**不得用正前方障碍去测**——正前方测不出来，会得到一个假通过。补前向观测（如改接
独立的 Hesai）属于另一条线，不混进本次验收。

### 3.3 §13.1-4 中修掉的一处陈旧断言

`test_validated_stationary_config_accepts_pinned_official_geometry` 断言
`config["revalidation_required"] is False`，而 `configs/go2w/lidar_preprocess.yaml:15`
写的是 `true`。经查该断言**自 `79b093a`（2026-08-14）起一直红着**：那次提交把标志由
`false` 改成 `true` 并配上 `revalidation_note`（自掩膜按新装的 PandarXT-16 保护框扩到
`|y|=0.42`、侧向旋转净空尚未独立验证），却漏改了断言。`load_safety_ready_config()`
的返回是 `return lidar, parameters`，标志直接来自 YAML、不经代码改写，两者必须一致。

已对齐到 YAML 的取值（`78798f7`）。方向是**收紧不是放宽**：断言 `true` 表示
"仍需复验、旋转未获授权"，与同文件 `rotation_clearance_validation.valid: false` 一致。

### 3.4 潜伏（未触发）的 exec 位问题

`go2w_plain_slam_bridge/CMakeLists.txt:57` 把 `imu_timestamp_aligner.py` 声明进
`install(PROGRAMS)`，但该文件**没有 +x**。当前**无害**：它只被
`imu_fallback_adapter.py` **import**，没有任何 launch 直接执行它。
（同目录的 `imu_fallback_adapter.py` 有 +x，且被 `plain_slam_go2w.launch.py:145` 当可执行
文件起——那个才是会炸的位置，它是好的。）

已按 launch 的真实判据（`os.X_OK` 解析 executable）在宿主与机器狗两侧各验一遍：
**19 个 launch executable 全部可执行**。所以该陷阱当前未被触发，未做改动。

### 3.5 ⚠ 就绪闸门对"发散位姿"全绿（本次最重要的一条）

本次起栈时抓到一个**真实的、不可自愈的 LIO 发散**，而 `start_nav2_autonomous_search.sh`
的四项就绪检查**全部报 `[OK]`**。

事件链：机器狗 ~14:38 重启，而宿主 plain_slam 是 11:44 起的，**跨着这次重启跑了近三小时**。
转折点日志三行同屏：

```text
[imu_fallback_adapter.py-1] ddsi_udp_conn_write to udp/192.168.123.161:50145 failed
[lio_3d_node-2] [GO2W-DIAG] imu_n=988        ← 正常 ~25，IMU 缓冲突然堆了 988 个样本
[lio_3d_node-2] [WARNING] No relevant IMU measurements were found.
[lio_3d_node-2] [WARN] Optimization has not converged
```

`imu_n` 阶跃 = 时间轴被打断。之后**永不恢复**：去畸变用状态本身算，状态坏 → 点云坏 →
配准无重叠 → 更无修正，`act` 锁死在 0.000。

| 字段 | 健康 | 发散时 |
| --- | --- | --- |
| `act` | 0.96~0.98 | **0.000** |
| `dsk_max` | 0.001~0.006 | **2.4e6** |
| `v` | ~0.007 | **2.5e7 m/s** |
| `pos` | 原点邻域 | **2.8e9, −2.7e10** |
| `ab` | 0.002 | **17638.8** |
| `map -> odom_fused` 平移 | 0.x m | **2.4e9 m 且持续增长** |

**为什么闸门查不出来**：四项验的都是**连通性**，没有一处验**数值量级**——
`/map` 有唯一发布者、`localization_valid` 照样是 `true`、TF 链**连通**、`planner_server`
active。所以 **§13.2 第 1/2/3 项能在一条废栈上"通过"**。

**处置**：按三步重起（宿主 SLAM → 宿主 wheel_odom → 狗上 Nav2），实测逐项恢复：
`act` 0.000→0.980、`dsk_max` 2.4e6→0.0010、`v` 2.5e7→0.007、
`pos` 2.8e9→(0.002, 0.001, −0.002)、越界告警 →0 条。

**给现场的口径**：验收前先自己量一眼 `map -> odom_fused` 的**平移量级**，>10 m 就停下来
重起栈，不要继续走 §13.2/§13.3。别把闸门全绿当成可验收。

---

## 4. 边界与未做

- §13.3 / §13.4 只能由现场人员完成；本记录不声称任何"不会撞墙"的结论（计划书 §16）。
- `nav2_allow_execute` 保持 `false`。按计划书 §10，那是 §13.3/§13.4 之后的**用户本人**步骤。
- 未改同步数组：`test_config_gate.py` 不在 `deploy_plan_to_robot.sh` 的文件数组里
  （该数组只同步"跑起来需要的东西"，该包仅同步了 `preprocessor_node.py`）。因此**机器狗上
  那份测试仍是陈旧断言**，零运行时影响；若要在狗上跑 `colcon test` 得到一致结果，
  需把该文件加入数组后重新同步。
- 宿主侧 `colcon build` 用的是 **ROS 2 Humble**，机器狗是 **Foxy**。宿主构建/测试通过
  **不等于**狗上通过；狗上的权威判据是部署脚本的 `REMOTE_SYNTAX_OK` 与现场 launch。

## 5. 环境状态（如实记录，含重测）

### 5.1 记录初稿时的状态

初稿撰写时（2026-09-28 约 14:50）**机器狗上的 ROS 栈是停的**：`/go2w/sensors/cloud`、
`/go2w/lidar/scan`、`/go2w/safety/front_clearance`、`/go2w/odom/fused`、`/map`、
`/go2w/nav/localization_valid` **全部不在**，相关进程一个都没有。所以当时那批数字
（第 4/5/6 项与 113.5°）**测于更早一次运行期间，初稿时未复测**，已在 §2/§3.2 标注。

### 5.2 重测：栈已重新起齐，并修掉一次 LIO 发散

初稿之后栈已按计划书 §6 重新起齐，**全过程只规划、不执行**：

| 机器 | 组件 | 状态 |
| --- | --- | --- |
| 宿主 | Hesai PandarXT-16（复用已有驱动） | 10.0 Hz |
| 宿主 | plain_slam 栈 | `ready: true` |
| 宿主 | wheel_odom `/go2w/odom/fused` | pub=1，`lio_fusion=True` |
| 机器狗 | 只读感知链 `live_20260928_150130` | camera source = d435 |
| 机器狗 | Nav2 bringup + 仲裁 + bridge | 就绪闸门四项 `[OK]` |
| 机器狗 | WebUI :8765 / VLM daemon | HTTP 200 / pid 10114 |

重测过程中发现并修复了 §3.5 那次 LIO 发散。**重测前后的对照**：

| 项 | 初稿（旧栈） | 重测（当前栈） |
| --- | --- | --- |
| 第 4 项 `max_xy_jump` | 0.0016 m | **0.0042 m** |
| 第 4 项 `max_yaw_jump` | 0.019° | **0.096°** |
| 第 4 项 `samples` | 1197 | **1190** |
| 第 5 项 代价地图 | `254=242 253=568` | **`254=249 253=664`**，另加 §2.1 的静态/障碍分解 |
| 第 6 项 速度链 | pub=0 / 1 / 不存在 | **同上，未变** |
| 正前方最长盲区 | 113.5° | **137.0°** |

第 4 项重测值比旧值大 2.6 倍（4.2 mm vs 1.6 mm），**仍远低于"大跳变"的量级**（判据是
米级），且 600 s 内绝对位移只有毫米级（末点 `[-0.2817, 0.024]`），判定**通过**。
两个值都留档，不做取舍。

`configs/go2w/navigation_gate.yaml` 的 `nav2_allow_execute` 全程 **`false`**，
重测期间未改；`/go2w/nav2_cmd_vel` 无发布者、`/cmd_vel` 不存在（§2 第 6 项）。

### 5.3 仍未做的

- §13.2 第 7 项需要跑一次**新会话**才会有 `candidate_decision` 事件，本记录未跑。
- §13.3 / §13.4 只能由现场人员完成；本记录不声称任何"不会撞墙"的结论（计划书 §16）。
- 按计划书 §10，把 `nav2_allow_execute` 改成 `true` 是 **§13.3/§13.4 之后的用户本人步骤**，
  AI 未做此改动。
