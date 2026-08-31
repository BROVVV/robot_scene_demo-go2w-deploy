# Go2-W SemanticNavigation-style Semantic Search Integration Handoff

## 基线

- 日期：2026-08-13
- 仓库：`/home/brov/robot/robot_scene_demo`
- 分支 / 开工 HEAD：`main@e861b95`
- 计划基线记录：`main@dd17e34`；`e861b95` 是其后的 2026-08-13 交接书提交
- 当前改动：工作树内，尚未提交

开工前已读：当前 README、`go2w_codex_handoff_20260813.md`、
`go2w_codex_handoff_20260807.md`、`go2w_codex_continuation_status_20260806.md` 和
用户指定 SemanticNavigation 详细计划。没有重复实现已经通过的相机、LiDAR、wheel/fused odom、
LLM quick/verify 或既有状态机闭环。

## 已实现

- 基于现有 `TargetProfile` 构建 GoalGraph：target / explicit anchor / inferred
  context 分级，关系规范化，不增加开发者固定 room prior 表。
- 基于现有 `SceneGraph` / `ObservedSceneGraphBuilder` 的可解释 matcher：exact / alias /
  lexical / attribute / relation / weak context，输出 zero / partial / strong、匹配节点、
  缺失节点、关系、属性、证据和 warning。
- `strong_match` 不包含 confirmed 状态，也不能改变目标确认结果。
- `LiveSemanticObserver`：稳定观察事件触发、TTL/heading/profile cache，复用现有
  object tracker 和 observed graph builder；节点只写 `observation_pose_only`、heading、
  bbox/provenance，不产生 3D/map 坐标。
- `SemanticSearchMemory`：session negative evidence、sector penalty、TTL、解封、任务
  reset；可只读复用 `ObservationMemoryStore`，不自动持久化失败 scan。
- reasoner：legacy / semantic_navigation / hybrid，输出可审计 `SearchDirective`；zero 优先未观察
  sector，partial 检查 anchor，strong 换角度重观测。
- `directive_to_step_plan`：转向 clamp ≤30°，低置信度/异常回退 legacy；semantic
  forward 默认关闭，显式开启后仍受 radius 和既有运动安全链约束。
- `SearchStateMachine.next_view_selected()`：在原 `SELECT_NEXT_VIEW → CHECK_SAFETY`
  迁移中记录 backend/directive/confidence，保留旧 `next_view_unavailable()`。
- `StepSearchRunner`：只在 detection recoverable error / `best is None` 分支调用语义层；
  target candidate 的 approach → verify 链未改变。
- shadow：生成并记录 semantic directive，实际执行严格为原 legacy scan step。
- active：置信度达标并可转换才执行 semantic step，否则自动 legacy fallback。
- full-scene worker 保留 legacy target-only `objects`，额外提供 `scene_objects` /
  `scene_relations` 给 semantic observer。
- CLI / config / UI status：增加 semantic enable、backend、shadow/active、forward 等配置；
  仓库默认 disabled + legacy + shadow + no-forward。
- observe-only pipeline 输出 `goal_graph.json`、`semantic_navigation_match.json`、
  `search_directive.json`。
- 新增离线 `scripts/evaluate_live_search_reasoners.py`，从已有 session scene graph 比较
  legacy/semantic_navigation/hybrid，输出 JSON/Markdown；不可用的真实运动指标明确为 null。
- README 与 `docs/SEMANTIC_NAVIGATION_SEMANTIC_SEARCH_INTEGRATION.md` 已补充架构、边界、命令、
  回滚和分级验收说明。

## 安全状态

- `configs/go2w/navigation_gate.yaml`：无 diff，继续 fail-closed。
- semantic forward 默认：`false`。
- 单步转向：adapter 硬 clamp 30°，未扩大现有真机边界。
- 低层控制：无新增；未修改 ROS2 包或 `unitree_go2w_control`。
- target confirmation：仍由现有 visual evidence + quick/crop verify/evidence gate 完成。
- stale camera、clearance、odom/radius、mode/error、STOP/disarm：原链未删除或弱化。
- 受保护记忆：全量测试曾追加一条临时 observation；已精确移除。最终两个 memory
  文件相对 Git baseline 均无 diff（当前行数 24 / 28）。
- API key / token：无新增或写入。
- 第三方源码：没有复制 SemanticNavigation 文件，不需要新增 vendored MIT notice；文档保留了
  算法思想来源和未来复制源码时的 notice 要求。

## 测试

### 新增与受影响回归

```text
python -m unittest <计划指定核心、新增、PSG/topology/observer 用例>
Ran 53 tests: OK

python -m pytest -q <新增与相关 pipeline/topology 用例>
25 passed in 1.55s
```

核心覆盖：GoalGraph、zero/partial/strong、alias、attribute mismatch、relation mismatch、
context 分级、memory TTL/release/read-only、reasoner、directive sign/clamp/radius、shadow
行为等价、active semantic turn、candidate 分支优先、observer cache/no fake metric pose、
state-machine trace。

### 全量 unittest

```text
python -m unittest discover -s tests
Ran 217 tests in 913.854s
FAILED (failures=2, errors=2)
```

213 项通过；4 项均是 README 在改造前已经记录的基线问题：

1. `test_go2w_live_ui_status...`：Streamlit AppTest 30 s timeout；
2. `test_streamlit_natural_language_task_ui...`：Streamlit AppTest 30 s timeout；
3. `test_task_examples_evaluator...`：legacy `count_objects` /
   `navigate_to_location` 与当前 LLM-first task schema 断言不一致；
4. `test_task_planner...count_objects...`：同一 legacy task-type 断言不一致。

没有新增 SemanticNavigation/live_robot 用例失败。测试期间没有 GPU、API 或真机调用。

### 其它检查

- Conda Python 3.11：相关文件 `py_compile` PASS；
- ROS `/usr/bin/python3` 3.10：纯 reasoning/adapter/observer/Runner 和 autonomous script
  `py_compile` PASS；
- `bash -n scripts/go2w/start_search_session.sh` PASS；
- offline reasoner evaluator smoke PASS；
- 既有真实 session `live_20260806_204728/scene_graph.json` 离线 policy replay PASS：
  2 nodes / 1 edge，legacy=`legacy_scan`，semantic_navigation/hybrid=`inspect_anchor(0.69)`；
- `git diff --check` PASS。

## 如何启用

### Shadow（推荐第一步）

```bash
/usr/bin/python3 scripts/go2w/run_autonomous_loop.py \
  --mode state_machine_search \
  --target "饮水机旁边的蓝色垃圾桶" \
  --detector llm \
  --semantic-reasoning \
  --search-reasoner hybrid \
  --search-reasoner-mode shadow \
  --semantic-no-forward \
  --max-radius 1.0 \
  --odom-topic /go2w/odom/fused \
  --output outputs/live_sessions/semantic_navigation_shadow.jsonl
```

此模式会调用 semantic observer / matcher / reasoner，但 execute_step 仍收到 legacy
plan；用 `reasoner_shadow_compare` 事件做 A/B。

### Active turn-only

只有 shadow 证据经操作者审查后，把 mode 改为 `active`，继续保留
`--semantic-no-forward`。仍需既有 `GO2W_MOTION_READY` 授权和全部现场安全条件。

### 回滚 legacy

去掉 `--semantic-reasoning`，或传：

```text
--search-reasoner legacy
```

仓库默认配置本身就是 legacy，因此不需要回滚代码或配置文件。

## 尚未验证

- 完整录像/event-stream shadow A/B（已有一个真实 session scene-graph policy replay，
  但没有 time-to-candidate / motion trace 指标）；
- 真机 shadow；
- 真机 active turn-only；
- active short-forward（默认关闭）；
- external LLM full-scene semantic observer 的现场延迟/稳定性；
- CUDA/GPU 路线。

禁止把上述事项写成 PASS。

## 原交接书仍未完成基础项

- Point-LIO translation：BLOCKED；
- USLAM：BLOCKED；
- wheel radius / 4WS 标定：未完成；
- RGB–LiDAR extrinsic：EXPERIMENTAL；
- camera TF：BLOCKED；
- base→LiDAR yaw：仍需复核；
- map / Level D / Nav2：BLOCKED / fail-closed；
- 自由探索、录像硬件编码/精确时间戳：未完成。

本语义层没有绕过或解决这些基础设施阻塞项。

## 下一步

1. 用已有 live session / 录像生成完整 scene graph，运行 offline evaluator；
2. 真机 observe-only，检查 GoalGraph、anchor match、directive 与 LLM 延迟；
3. 在明确运动授权下运行 shadow，确认实际 step 与 legacy 完全一致；
4. 通过审查后再运行 active turn-only；
5. 只有前四步有证据后，才讨论显式 short-forward；
6. metric graph / map goal / Nav2 必须等待原基础门禁真实通过。

## 2026-08-13 真机续测

用户开启 Go2-W 后已完成下一阶段的非运动验证：

- `enp6s0=192.168.123.99/24`，`192.168.123.18` ping 正常；
- 内置 RGB 约 18.4 Hz，`/go2w/lidar/scan` 约 15.4 Hz；
- 启动只读 `wheel_odom.launch.py` 后 `/go2w/odom/fused` 为 20 Hz；当前是
  `Sport heading only` fallback，`lio_valid=false`、`lio_used=false`，不能写成
  Point-LIO translation PASS；
- `/lf/sportmodestate`：`mode=1`、`error_code=0`、线速度和角速度均为 0；
- 当时 Bundle health 曾把 camera / camera_info / RGB-LiDAR extrinsics / fusion / LiDAR
  都写为 true；其中两个 RGB-LiDAR true 后续审计确认是实验候选状态误标，已由本报告
  末尾“RGB-LiDAR 状态分级修复”取代；LIO / TF 为 false；
- 未启动 control lease，未调用运动接口，`motion_commands.jsonl` 和
  `nav2_requests.jsonl` 均为空。

真机执行：`observe_only + detector=llm + semantic_reasoning + hybrid + shadow`，目标为
“饮水机旁边的蓝色垃圾桶”。输出目录：

```text
outputs/live_runs/semantic_navigation_observe_20260813_01/live_20260807_163307
```

视觉模型观察到蓝色垃圾桶和饮水机，并给出二者相邻关系，置信度 0.9。首次运行暴露
ROS bridge 只保留最近 Bundle、远程推理期间输入图被清理的竞态；现已在任何 LLM API
调用前把已通过传感器门的帧原子快照到 `input_frames/`，并增加“删除原 spool 图后快照
仍可读”的回归测试。第二次运行完整产出了输入快照、标注图、SceneGraph、GoalGraph、
match、directive 和安全日志，未再出现竞态。

现场结果还暴露并修复了三处数据链问题：

1. LLM 偶尔输出 0-based index；现在只要整批关系/target index 出现 0，就对整批统一按
   0-based 解释，避免后续不含 0 的关系发生 off-by-one；
2. `source_object_id` / frame object id 原先未映射，导致观察关系无法进入 tracked
   SceneGraph；现在关系端点映射到 frame object id 后再构图；
3. `color` 原先未进入 frame/object-track attributes；现在颜色随视觉对象进入观察图。

用现场保存的同一帧对修复后链路离线复放：

```text
scene relation: obj_trash_bin_001 --near--> obj_water_dispenser_001
attribute support: goal_target:blue
match: strong_match, score=0.7875
directive: reobserve_sector, allow_forward=false
```

`target_found=false` 仍是正确的 fail-closed 结果：本次仅一帧且显式关闭 crop verify，
现有 confirmed-track / visual evidence gate 不允许仅凭 GoalGraph strong match 确认目标。
语义层只建议换角度重观测，没有把 2D bbox 或模型 route plan 变成运动/导航目标。

本轮新增/受影响回归：

```text
51 passed in 0.83s
```

其中新增覆盖稳定帧快照、全局 0-based index、关系端点映射、颜色属性保留和
`next_to`/`near` 等价匹配。`git diff --check` PASS。

另跑了排除交接书已知 Streamlit 长超时、旧 task example / `count_objects` 基线断言的
广泛 pytest 回归：

```text
231 passed, 1 deselected in 229.24s
```

这不是对已排除基线问题的修复声明，但受影响的语义、视觉、live robot 与安全链未出现
新增失败。

下一项仍是真机 shadow 运动 A/B；必须先获得操作者明确的 `GO2W_MOTION_READY` 授权，
并在开阔现场确认急停条件。仅“机器狗已开机”不等于运动授权，因此本轮没有转向或前进。

### 三帧视觉复核续测

随后又采集 3 张真实静止帧并启用原 crop verify。初次结果为：

- 蓝色垃圾桶连续 3 帧，track=`confirmed`；
- GoalGraph=`strong_match(0.7875)`，属性 `blue` 和 `near 饮水机` 均有支持；
- crop gate 仍 fail-closed，原因是原 1.35 倍紧裁剪只包含垃圾桶，无法观察“旁边的
  饮水机”；最终为 `target_candidate`，不是误确认；
- 3 张输入快照和标注图完整，运动 / Nav2 日志仍为 0 字节，机器狗仍为零速。

该续测又定位并修复两处通用问题：

1. `VideoObjectTracker` 原先可能只凭重叠 bbox、位置和共享属性，把标签完全不同的邻近
   物体合并，现场表现为饮水机 track 混入主机、垃圾桶混入饮水机属性。现在不同标签
   相似度低于 0.55 时直接禁止合并；离线复放后垃圾桶 / 饮水机 / 主机轨迹已分离，
   directive anchor id 与 label 一致为 `obj_water_cooler_001 / 饮水机`。
2. 对带显式关系约束的目标，crop verifier 现在使用“候选 bbox ∪ 最近的、与
   `TargetProfile.context_terms()` 匹配的视觉锚点 bbox”作为复核区域；候选本身的 bbox
   和原 evidence gate 阈值均未改变。无关系约束的普通目标继续使用原紧裁剪。

复用上述真实保存帧，只重跑新的上下文 crop verify 和原 tracking/search/evidence gate：

```text
frame 7752: crop confirmed, fused score=0.9225
frame 7754: crop confirmed, fused score=0.9175
trash-bin track: 3 frames, 2 positive crop verifications, final_score=1.0
evidence gate: visual_confirmed, target_found=true, blocking_rules=[]
```

复核 crop 保存在：

```text
outputs/live_runs/semantic_navigation_context_verify_replay_20260813/video_crops/
```

这证明真实输入上的多帧视觉确认链已可通过，但没有产生 3D/map pose，也没有触发运动。
新增测试覆盖“重叠但不同标签不能串轨”和“关系目标裁剪包含显式锚点”；相关测试
`23 passed`。最终以同样的已知基线排除清单重跑广泛回归：

```text
234 passed, 1 deselected in 284.03s
```

## 2026-08-13 厂商避障开关实测

用户明确授权“允许切换避障模式”后，仅执行了一次官方高层 Sport
`SwitchAvoidMode()`（API 2058），没有发送 Move、姿态或低层命令。

执行前：`mode=1`、`error_code=0`、三轴速度和 yaw 均为 0，机器人网络正常。调用结果：

```text
operation: SportClient.SwitchAvoidMode
api_id: 2058
call_count: 1
status_code: 3203 (server API not implemented)
```

收到 3203 后未重复 toggle。调用后复核：

- `ObstaclesAvoidClient.SwitchGet()` 仍返回 3102，状态不可读；
- `/api/obstacles_avoid/request` 没有机器人端订阅者，response 没有发布者；
- RobotState 服务列表中没有 `obstacles_avoid`，只有正常的 `unitree_lidar` 和
  `voxel_height_mapping`；
- Sport server API version 为 `1.0.0.1`；
- 机器人继续 `mode=1`、`error_code=0`、速度和 yaw 均为 0；没有运动或模式副作用。

结论：当前 Go2-W 固件/已安装服务不实现 Sport API 2058，也没有独立避障 RPC 服务，
所以不能用当前 DDS SDK 手动打开。项目中的
`PLATFORM_OBSTACLE_AVOIDANCE_ASSUMED=false` 只是高层规划安全假设，不是关闭厂商功能的
开关。下一步需在 Unitree App 检查 OTA、机器人功能开关和授权包，或向宇树提供序列号/
主固件版本确认 Go2-W 对应避障模块；在厂商功能真实可读并通过实测前继续保持项目侧
`platform_obstacle_avoidance_assumed=false`。

## 2026-08-13 狭窄现场运动授权与门禁结果

操作者明确发送 `GO2W_MOTION_READY`，并把授权范围限定为：以授权时机器狗的位置与朝向
为原点，只允许初始正前方 180° 半平面内、半径 1.5 m 的区域。为精确表达该约束，新增：

- `--front-half-plane-only`：半平面固定到授权时朝向，转向后不能把“当前前方”当成新的
  可行半平面；
- `--turn-only`：最终动作执行器拒绝所有 `f`，不只约束 semantic directive；
- `--max-motion-steps 1`：一次成功动作后由 `StepSearchRunner` 正常结束；
- `--min-rotation-clearance 0.511`：左右 clearance 任一缺失或低于已验证的全机身转向
  包络即拒绝转向；
- 每次动作前预测 forward endpoint，每次动作后复核实际 odom；越过固定前向半圆时
  emergency STOP。

新增纯几何门与 Runner step-limit 测试；最终相关回归 `23 passed`，ROS Python 3.10
`py_compile` 与 `git diff --check` PASS。

授权后只读实测：

```text
front clearance: 约 0.58–0.61 m
left clearance:  约 0.37–0.41 m
right clearance: 约 0.37–0.41 m
required full-body rotation envelope: 0.511 m
```

约 90 帧连续 clearance 采样均显示左右障碍在转向包络内部；最新前视图也显示机器狗位于
办公椅、隔板和桶之间的狭窄工位。故安全门关闭：没有启动 Sport lease，没有 arm，
没有发送 Action goal，实际运动为 0 步。此结果不能写成真机 shadow PASS。

继续条件：先移走两侧椅子/隔板/桶等障碍，使左右 0.511 m 全转向包络内无点（topic
通常为 `inf`，或至少两侧读数均不低于 0.511 m）。若只是清理障碍而不改变机器狗初始
姿态，可重新做预检；若人工搬动机器狗，则必须在新位置与新朝向重新授权并重新记录原点。

### LiDAR 假障碍复核与更正

操作者现场指出左右实际为空、前方实体障碍在 1 m 外。随后暂停全部运动并对原始点云按
高度和自过滤边界做 30–60 帧静止复核，确认上一节把 clearance 数值解释成实体障碍是
错误的；上一节的“移走两侧障碍”结论由本节取代。

根因有两部分：

1. `/go2w/lidar/obstacles` 原本把最高到 `z=0.972 m` 的全高度语义点全部送进 2D
   `/scan` 和 clearance。前方约 0.58 m 的连续点主要位于 `z≈0.6 m`，高于 Go2-W
   站立碰撞体，并非前进时会碰撞的地面障碍；
2. 左右 0.38–0.50 m 点落在 L2 近场/机身自反射区。简单扩大矩形 self mask 时最近点
   会跟随 mask 边界移动，说明不能靠继续“擦掉点”来证明侧面安全。

已把两类用途拆开：

- `/go2w/lidar/obstacles` 保留全高度点，供语义和诊断；
- 新增 `/go2w/lidar/collision_obstacles`，只保留
  `-0.448 < z <= 0.052 m`，供 `/scan` 与前向 clearance；上界由 URDF 地面
  `-0.528 m`、厂家站立高度 `0.50 m` 和 `0.08 m` 顶部余量推导；
- 新增 `/go2w/safety/rotation_clearance_valid`。由于 L2 近场尚不能覆盖 0.511 m
  全机身旋转包络，当前固定发布 `false`，左右 clearance 发布 `NaN`（unknown）；
- 运动执行器对任意转向都先要求上述有效性为 true，不能再把 `inf`、缺 topic 或默认
  `--min-rotation-clearance 0` 当成已确认空旷。

重新编译并替换只读预处理节点后，60 帧现场验收结果：

```text
full-height obstacles z: -0.448 .. 0.972 m
collision obstacles z:  -0.448 .. 0.052 m
front: 60/60 no collision-height return（原 0.58 m 假障碍消失）
left:  60/60 unknown
right: 60/60 unknown
rotation_clearance_valid: 60/60 false
validation: PASS（只读、未下发运动）
```

证据：

```text
outputs/go2w_acceptance/lidar_collision_height_live_20260813/result.json
```

验证时 `/go2w/odom/wheel` 线速度三轴为 0，未运行 `/go2w/motion` Action 或 arm 服务。
因此修正后的准确结论是：前方没有检测到碰撞高度内的近障碍；左右现场情况与操作者观察
一致，但传感器自身仍无法独立验证旋转包络，故真机 active turn-only 继续 BLOCKED，不能
写成运动 PASS。

## 2026-08-13 完整离线 Shadow A/B 指标补验

计划书要求的 `scripts/evaluate_live_search_reasoners.py` 原实现只对单个最终 SceneGraph
调用三种 backend，大部分要求指标仍为 `None`，且只要 semantic directive 带 heading
就计为与 legacy 不一致，没有比较适配后的 `StepPlan`。现已补为无 API、无运动的可审计
replay：

- 复用 session 保存的精确 `target_profile.json`，不在 replay 中重新调用 LLM；
- 分别记录 directive、适配后的 `proposed_step` 和 shadow 真正执行的
  `shadow_executed_step`；
- 从 `frame_observations`、crop report、evidence gate 和可选 runner JSONL 统计 detector、
  推导的历史 LLM 调用、候选/确认时间、复核拒绝、步数、转角与距离；
- `success_proxy` 明确定义为“存在持久化候选视觉证据”，绝不等同 target confirmed；
- 生成 `reasoner_comparison.json/.md`、`reasoner_metrics.json` 和
  `shadow_comparison.json`。

已对三份真实保存 session 运行：

```text
phone_live_20260806:
  match partial；semantic proposed l30；shadow executed r30
semantic_navigation_live_20260813_01:
  match partial；semantic proposed l25；shadow executed r30
semantic_navigation_live_20260813_02:
  match strong；semantic proposed l25；shadow executed r30
  detector_calls=3；recorded_llm_calls_inferred=6
  time_to_candidate=0.0s；verify_reject_count=3
  success_proxy=true；target_confirmed=false
```

三份 replay 均满足：`actual_shadow_behavior_matches_legacy=true`、
`dangerous_forward_request_count=0`。这完成的是 recorded-session 离线 A/B，不是带运动的
真机 shadow；后者仍受旋转近场门禁阻断。

### Runner 实时 LiDAR freshness 门修复

继续安全审计发现 `run_autonomous_loop.py` 原先只用“曾收到过 front clearance”近似
`lidar_fresh`，`_safety_ok()` 在 clearance 缺失时也可能继续通过。已改为显式订阅
`/go2w/safety/lidar_fresh`，并且：

- 启动前等待 fresh clearance，并在 arm 前执行完整 mode/error/LiDAR 预检；
- 每一步拒绝 freshness=false/缺 topic、front 缺失或 NaN；
- `inf` 仅在 freshness=true 时作为“当前无前向碰撞高度回波”接受；
- `StepSearchRunner` 的 `SensorSnapshot.lidar_fresh` 使用真实 Bool，不再由非空历史值推断。

新增纯函数测试覆盖 stale、missing、NaN 和 fresh+inf；相关集中回归 `16 passed`。

为避免下游再次混淆非有限值，`robot_scene_live_bridge` 的 Frame Bundle 新增：

```text
front_status: no_return
left_status: unknown
right_status: unknown
rotation_clearance_valid: false
```

真机重启只读 bridge 后已在新 session `live_20260813_clearance_status` 观察到上述字段，
且 `lidar_fresh=true`。此外，默认关闭的 `go2w_cmd_vel_bridge` 也加入
`rotation_clearance_valid` gate：任何非零 yaw 请求在旋转近场未验证时均返回
`rotation_clearance_unvalidated`；直行请求仍由其余既有门禁处理。ROS 工作区累计测试结果
后续累计更新为 `62 tests, 0 failures`。

### 修正后 Bundle 的三帧真机 Observe-only

使用新 Bundle schema 重新运行三帧
`observe_only + detector=llm + semantic_reasoning + hybrid + shadow`，输出：

```text
outputs/live_runs/semantic_navigation_observe_20260813_03/live_20260813_clearance_status
```

结果：

- 3/3 全景观察均检测到蓝色垃圾桶和饮水机及 `next_to` 关系；
- 2 次 crop verify 均成功，`final_score=0.9125`；
- Scene/Goal graph 为 `strong_match=0.7875`，匹配关系
  `goal_target:near:goal_anchor_001`，颜色支持 `goal_target:blue`；
- target evidence gate 为 `visual_confirmed`，空间状态仍严格为 `target_2d_only`；
- 模型输出中的 1 m route plan 未被执行，`navigation_goal_block_reason` 仍是
  `target_has_no_validated_3d_pose`；
- motion / Nav2 / safety JSONL 均为 0 字节。

本次又发现 crop 模型虽然确认主体和颜色，却在字段中声明缺少“位于饮水机旁边”的关系；
完整 SceneGraph 同时有该关系，所以结果本身有关系证据，但原 gate 没把组合条件显式化。
现已增加 `TARGET_CONFIRMATION_REQUIRE_RELATION_EVIDENCE`：凡 TargetProfile 带显式关系，
必须同时满足原 crop/track 视觉门与 Goal/Observed SceneGraph 的 strong relation match。
用上述真实保存 graph 复放时关系边存在则通过；删除该边后同一主体 crop 被 fail-closed。

最终广泛离线回归（排除交接书已知 Streamlit 长超时、旧 task examples 和
`count_objects` 基线断言）：

```text
244 passed, 1 deselected in 373.74s
```

这仍是 Stage 1 observe-only 的通过证据，不是带运动的真机 shadow PASS。

## 2026-08-13 RGB-LiDAR 状态分级修复

继续完成度审计时发现，`sensor_extrinsics.yaml` 同时声明“实验候选、不可用于导航”和
`confirmed: true`，`rgb_lidar_fusion.yaml` 也把实验接受写成
`validation_status: validated`。融合节点据此把
`rgb_lidar_extrinsics_validated` / `fusion_ready` 发布为 true，与计划书 V1 的硬边界
矛盾，存在未来误开 3D 分支的风险。

现已拆成两个不可混淆的等级：

```text
stationary diagnostic overlay:
  /perception/rgb_lidar_overlay_ready = true
navigation-grade metric geometry:
  /perception/rgb_lidar_extrinsics_validated = false
  /perception/fusion_ready = false
  authorizes_3d_output = false
```

配置已恢复真实语义：外参 `calibration_status=experimental`、`confirmed=false`、
`moved_position_recheck_passed=false`；保留 7 场景、33.6 px 和候选变换作为诊断证据。
节点在实验等级可以生成调试图，但会在发布 Detection3D / relative pose / odom pose 前
强制返回。严格 gate 还要求 `navigation_geometry_validated=true`、真实移动位置复核和
`authorizes_3d_localization=true`，单纯修改一个 `validated` 字符串不能再解锁。

真机只读复验连续 5 组结果：overlay 全 true，extrinsics/fusion 全 false，三个 metric
3D 话题合计 0 消息；新 Bundle 为：

```text
camera=true, camera_info_calibrated=true
rgb_lidar_overlay=true
rgb_lidar_extrinsics=false, rgb_lidar_fusion=false
lidar=true, lio=false, tf=false
```

证据：

```text
outputs/go2w_acceptance/rgb_lidar_geometry_tier_20260813/result.json
outputs/go2w_acceptance/frame_bundle_geometry_tier_20260813/result.json
```

本项全程未启动运动/lease/Nav2，机器狗未移动。

## 2026-08-13 LiDAR 轮组自回波修复与现场复核

用户指出左右现场空、正前障碍超过 1 m 后，继续对碰撞高度点云做 80 帧逐象限审计。
此前已修掉的 0.58 m 正前假障碍来自机器人高度以上；本次又确认 0.39–0.51 m 左侧
近点中心约为 `x=-0.21, y=0.42, z=-0.02 m`，并有左前对应小簇，处于 7 英寸轮胎
高度范围。旧 `left_body_side` 自滤只覆盖 `y<=0.37, z>=0`，漏掉轮胎外缘和
base_link 以下部分。

使用 80 帧 collision cloud 先做内存 A/B 后，仅增加两个当前姿态实测小掩码：
`left_rear_wheel_live_cluster` 与 `left_front_wheel_live_cluster`。没有把范围扩成整个
左侧，也没有对当前不可见的右轮做镜像猜测。A/B 删除 1,412 个轮组高度点，正前结果
与右侧 0.95–0.97 m 环境回波不变。

重启只读预处理后的新 80 帧验收：

```text
front collision corridor: 80/80 no_return
front-left raw diagnostic: minimum 0.608 m, median 0.620 m
front-right raw diagnostic: minimum 0.955 m, median 0.978 m
inside 0.511 m rotation envelope: 1 point in 1/80 frames
published left/right: 80/80 unknown
rotation_clearance_valid: false
validation: PASS（只读、未下发运动）
```

相机广角画面左缘可见白色办公椅的低位轮/支腿，方向与保留下来的左前约 0.62 m 簇
一致，因此没有继续扩大自滤。准确语义是：旧自车近障碍误报已消除，正前走廊无近障碍；
左右仍因旋转包络可观测性不足为 unknown，而不是“检测到左右有障碍”。证据：

```text
outputs/go2w_acceptance/lidar_wheel_self_filter_live_20260813/result.json
```

旋转、active turn-only 和 short-forward 仍未开放，机器狗未移动。

## 2026-08-13 真机 Shadow 决策等价与 arm-before-gate 修复

审计 `run_autonomous_loop.py` 时发现，`state_machine_search` 原来会在视觉检测/semantic
推理前全局 arm，而逐步旋转有效性门只在 `_execute_step()` 内检查。虽然当前转向仍会
被拒绝，但提前 arm 没有必要。现已改成状态机模式仅在某一步依次通过固定初始半平面/
1.5 m 边界、旋转有效性、LiDAR freshness 后，才在发送 Action 前即时 arm；如果步骤
提前被门禁拒绝，则清理路径也不调用急停或 disarm 服务，并记录
`control_cleanup_skipped`。

第一次真机 shadow 暴露 full-scene semantic observer 对同一稳定帧的第二次 API 调用
超时 120 s。quick 检测其实已明确返回 `target_decision.is_present=false` 和场景摘要。
现增加严格受限复用：只有 quick 明确判断目标不存在时，构造不含任何伪造物体/关系的
zero-match 语义观察；正例或不确定结果仍走 full-scene observer。新增测试覆盖负例复用
和正例/不确定拒绝复用，避免重复 API 延迟。

在 `/go2w/odom/fused=20 Hz` 且线/角速度为 0 后，用当前画面不存在的“紫色三角锥”
完成真实传感器 shadow：

```text
graph match: zero_match
semantic directive/step: explore_unseen, r30
legacy step: r30
actual_shadow_behavior_matches_legacy: true
dangerous_forward_request: false
rotation_clearance_valid: false
successful motion steps: 0
start/final odom delta: [0, 0, 0]
initial arm: deferred
arm/Action/motion-control graph: not started
```

严格 JSONL 与自动汇总的 10 项检查全部通过：

```text
outputs/live_sessions/semantic_navigation_shadow_fail_closed_20260813_03.jsonl
outputs/go2w_acceptance/semantic_navigation_shadow_fail_closed_20260813/result.json
```

这证明 Stage 1 在当前真机传感器上保持 legacy 决策且安全 fail-closed；不是“成功转动
后的运动 A/B”。Stage 2 active turn-only 仍因旋转近场不可验证而 BLOCKED，Stage 3
short-forward 也按顺序保持未开放。机器狗全程未移动。

## 2026-08-13 长期 Observation Memory 真机接线审计

计划第 11/22 节要求区分已有长期 observation memory 与 session negative memory。
审计发现 observe-only 已读取长期库，但 `run_autonomous_loop.py` 的真机状态机原先仅创建
短期 memory，且 controller 仍使用硬编码默认图匹配阈值。现已修复：

- 真机与 observe-only controller 都读取 `Settings` 中的 partial/strong 阈值；
- `LIVE_SEARCH_REASONER_USE_OBSERVATION_MEMORY=true` 时，以只读方式检索现有
  `ObservationMemoryStore`，并把记录传入每步 `SearchReasoningContext`；
- artifact 记录 retrieved count、`memory_id` 和
  `persistent_write_attempted=false`；
- `LIVE_SEARCH_NEGATIVE_MEMORY_ENABLED` 与 TTL 现在实际控制真机 session memory；
- 长期记录没有明确观测 heading 时，不把历史 bbox 猜成真机转向；
- 修复 `datetime.UTC` 和运行期 schema import，使该读取路径兼容 ROS 2 Humble
  `/usr/bin/python3`（Python 3.10），同时保持 Conda 3.11 路径。

当前长期库共 24 行；用真实“手机”查询读取最新 10 条。接线前后 SHA-256 均为：

```text
2aba27a0aad27efd39a7b0e867fd930f90e49983cf965ec159a5c963fe4706ad
```

相关定向回归 16 项全部通过。本项没有启动任何运动节点或命令，机器狗未移动。

## 2026-08-13 当前现场 LiDAR 再复核

针对操作者指出“左右为空、正前障碍超过 1 m”，再次采集 60 帧只读数据。正式安全输出
为 `front=no_return`、`left=unknown`、`right=unknown`、
`rotation_clearance_valid=false`；因此当前系统已不再声称左右存在近障碍，也没有把原
0.58 m 高处回波当作正前碰撞。全高度点云仍保留给语义观察。

诊断点云中左右/斜后仍存在办公椅支腿等回波，但这些 raw sector 数值不具安全权威性；
相机广角画面也能看到边缘办公椅。当前 `base_link -> LiDAR` 的俯仰/地面方向已验收，
但水平朝向与稀疏低位障碍覆盖没有独立物理标定，故不能据单帧静止数据把左右从
`unknown` 升为 clear，也不能开放旋转。最新证据：

```text
outputs/go2w_acceptance/lidar_current_scene_recheck_20260813/result.json
```

该复核连续 60 帧通过处理链一致性检查，全程只读、机器狗未移动。

随后又对实时 Frame Bundle 做了独立复核：最新 1920×1080 图像与 CameraInfo 一致，
LiDAR fresh；Bundle 明确写出 `front_status=no_return`、
`left_status=unknown`、`right_status=unknown`，未把非有限数值伪装成距离。证据为：

```text
outputs/go2w_acceptance/live_bundle_current_scene_20260813/result.json
```

## 2026-08-13 旋转扫掠包络可观测性硬门

进一步把“近场不可验证”从文字说明变成确定性几何审计。新逻辑按 720 个方位计算厂家
矩形 footprint 到 `0.511 m` 旋转包络之间的自由环带，并求该环带与 LiDAR
`0.37 m` 最小有效距离、base self mask、各轮组/机身局部 self region 的交集。当前结果：

```text
angles with unobservable free space: 720 / 720
front / rear axis blind radial length: 0.040 m
left / right axis blind radial length: 0.155 m
maximum sampled blind radial length: 0.290780 m
minimum-range affected bearings: 584 / 720
named self-region affected bearings: 482 / 720
sensor-only rotation observability complete: false
```

因此 raw collision cloud 即使当前几乎没有 `r<=0.511 m` 点，也不能证明原地转向安全。
配置加载器同时增加防误解锁合同：`valid=true` 还必须具备物理 360°+LiDAR 交叉验证
方法、操作者/时间/站姿、至少 0.511 m 的验证包络、证据路径，以及水平 yaw、近场盲区
缓解、自滤物理复核、全扇区检测、站姿五项 true；仅修改布尔值会使安全配置加载失败。

当时重编译并增加证据文件内容校验后包级 15 项测试通过；加入下述短时许可工作流并
修正过期相机测试夹具后，最新结果更新为 LiDAR 包 26 项、ROS 工作区累计 86 tests，
0 failure/error/skip。
新节点连续 60 帧只读验收通过，正式输出仍为 front `no_return`、左右 `unknown`、
`rotation_clearance_valid=false`：

```text
outputs/go2w_acceptance/lidar_rotation_observability_20260813/result.json
```

本项没有启动 Action、lease、`/cmd_vel` 或运动控制节点，机器狗未移动。Stage 2 需要
补齐上述独立物理证据，不能由当前点云结果自动解锁。

## 2026-08-13 Replan、PSG 与 Situated Prior 配置接线

继续审计计划第 18/21 节发现，`.env` 中的 `MIN_REPLAN_SECONDS`、`USE_PSG` 和
`USE_LLM_SITUATED_PRIOR` 原先只有配置字段，真机 StepSearchRunner 未完整消费。现已补齐：

- 同一 semantic frame、observer 明确 cache-hit、x/y/yaw 与错误状态不变且仍在最小间隔
  内时，复用上次 directive，并记录 `semantic_replan_throttled`；
- `LiveSemanticObserver` 增加 0.05 m translation refresh，运动后不能复用旧场景；
- 复用现有 `VideoPSGPredictor`，只把真实 observed node 支撑的 left/right next-best-view
  转成 turn-only 辅助 hint；
- situated prior 只消费上游已经产生、明确 `can_confirm_target=false` 的 payload；没有安全
  payload 时状态为 unavailable，不在真机循环发起重复 LLM API；
- 只有 Hybrid 的 `zero_match` 在 visited/negative-memory 成本同分时才使用辅助 hint；
  negative memory 可压过置信度为 1.0 的 PSG hint；纯 SemanticNavigation backend 不消费辅助层；
- artifact 与 observe-only 输出新增 source status、hints、used refs、
  `can_confirm_target=false` 和 replan 状态。

新增/受影响定向测试 28 项全部通过；ROS 系统 Python 3.10 可导入完整 runner 与辅助层。
这项改动只改变下一视角建议和审计信息，不新增低层控制、不解锁 forward/Nav2，也没有让
机器狗移动。

计划点名的离线 `unittest` 回归共 63 项通过。全目录回归没有出现断言失败，但在旧
`tests/test_task_planner.py` 调用真实 SiliconFlow/OpenAI 兼容端点时长期停在 TLS
握手，13 分钟后人工中止；该项属于外部服务可用性，不能记作完整全量通过，也与本次
LiDAR/SemanticNavigation 改动无关。

## 2026-08-13 原位旋转短时许可与四向标定工具

为避免把人工“当前周围为空”错误升级成永久传感器能力，现将两类证据分开：

- 预处理器永久 `rotation_clearance_valid=true` 只接受完整 sensor-only 近场覆盖；人工
  原位检查产生的证据会被配置加载器明确拒绝；
- 新只读工具先采 baseline，再要求同一低矮宽目标依次出现在 front/right/rear/left，
  用前后点数增量验证水平 yaw、四扇区和 self filter；每次非 baseline 采集都会即时
  对照并在目标缺失/方向距离错误/增量不足时返回失败；
- 证据绑定 `odom_wheel` 初始位置、最大平移 0.03 m、最长有效 900 s，并哈希链接五个
  capture 文件；五次采集之间的 yaw 变化也不得超过 1°；
- 新 `/go2w/diagnostics/lidar_clearance_raw` 永远标为诊断值。Runner 仅在短时证据有效、
  当前位置未漂移、LiDAR fresh 且诊断样本年龄不超过 0.3 s 时临时采用；否则仍使用正式
  `NaN/unknown` 并在 arm 前拒绝转向；
- 许可只覆盖原位转向；前进后自动因位置变化失效，不能变成移动后的通用旋转许可。
- 许可入口硬限制为 `state_machine_search + active/turn-only + wheel odom + 单步 + ≤30° +
  固定前半平面 + radius≤1.5 m`，不能用于 pattern、wander、多步或 short-forward。

当前已在机器狗静止时完成 30 帧 baseline，里程计最大平移约 4 微米、最大 yaw 变化
约 0.0003 rad，所有只读健康检查通过：

```text
outputs/go2w_acceptance/rotation_physical_crosscheck_20260813/baseline.json
```

baseline 表明 0.70 m 的 rear/left ROI 已有环境回波，因此工具支持 0.55–1.50 m 多距离
画像和受控 before/after 增量，不会把既有椅腿直接当标定物。建议的低占用起始距离为
front 0.55–0.65 m、right 0.55–0.65 m、rear 0.55–0.65 m、left 约 0.85–0.90 m；仍需人工
放置四向标定物和实测完整 0.511 m 扫掠圆后才能生成最终许可。此阶段没有 Action、arm、
lease 或运动节点，机器狗未移动。

在目标尚未放置时还执行了 front 0.60 m 的负向采集：健康、静止和 baseline ROI 均
通过，但 repeat detection、point contrast、bearing、range 四项均按预期失败，证明空场
不会被误签成 front 标定成功。证据为：

```text
outputs/go2w_acceptance/rotation_physical_crosscheck_20260813/front_no_target_negative.json
```

负向真机门禁也已验证：故意把 `baseline.json` 当最终许可传给 Runner，进程以 rc=2
在 arm 前拒绝，理由为 validation contract mismatch；ROS 图中没有 Action 或运动节点，
轮式里程计线速度为 0。审计日志：

```text
outputs/live_sessions/rotation_lease_invalid_fail_closed_20260813.jsonl
```

最新相关应用回归 37 项通过；完整 ROS/Unitree 环境下 10 个包累计 86 项全部通过。

## 2026-08-13 外接 Hesai PandarXT-16 识别审计与标定重置

操作者在上述四向标定过程中给机器狗加装了一台外接激光雷达。只读设备发现与 Web
状态查询确认该设备为 Hesai `PandarXT-16`（16 线），控制地址
`192.168.123.20`，序列号 `XT38BA50E238BA26`，固件 `1.2.8`。实时监控返回
`11.60 V`、约 `672--683 mA`、温度 `33.2--34.4 C`、非 standby、360 度 FOV；
点云初始配置为 600 rpm、目的地址 `192.168.123.18:2368`，GPS 端口 `10110`。
操作者随后给出 `PANDAR_REDIRECT_READY`，允许把点云接收端切换到工作站。

因此当前结论必须分层：

- 雷达本体已上电、网络可达且可被准确识别；
- 写入前后逐项读取，仅把 destination IP 从 `192.168.123.18` 改为
  `192.168.123.99`；UDP `2368`、GPS `10110`、600 rpm 和其余配置保持不变；
- `tcpdump` 已确认 `192.168.123.20:10000 -> 192.168.123.99:2368` 连续 UDP
  数据，修改可通过同一配置接口回滚为 `.18`；
- 官方 `HesaiLidar_ROS_2.0` 已固定在提交
  `e7e112f0809f0eed5e3c81c55a1a0376474db234`，SDK 子模块固定
  `9d5dc4fc4ade5be5f6a6ca00e71dd4050b054168`，ROS 2 Humble 构建通过；
- 独立话题 `/hesai/pandarxt16/points_raw` 已稳定发布，frame 固定为
  `pandarxt16_link_unvalidated`，不会冒充 `base_link`；原 `/utlidar/cloud` 仍是
  内置 Unitree LiDAR，两套来源保持隔离；
- 20 帧只读验收通过：约 10.00 Hz、每帧 64,000 点、ring 0--15 齐全、header
  时间戳严格递增、每圈点时间跨度约 0.10 s；有效回波比例约 95.0%，累计
  853,181 包、报告丢包 0；
- PTC 已从雷达成功读取角度修正。firetime 文件当前为空；厂家文档明确其为可选且
  缺失不阻止运行，但在精确几何标定完成前仍记录为精度限制；
- 新流没有接入 `/go2w/lidar/*`、TF、安全门或运动链；ROS Action 列表为空，
  `/go2w/safety/rotation_clearance_valid=false`，机器狗未移动。

可复现入口和证据为：

```text
configs/go2w/hesai_pandarxt16.yaml
scripts/go2w/start_hesai_pandarxt16.sh
scripts/go2w/validate_hesai_pandarxt16_ros.py
outputs/go2w_acceptance/hesai_pandarxt16_20260813/result.json
```

外接雷达改变了机体质量、外形、自遮挡与潜在旋转扫掠半径。它是在 `front_v2.json`
目标采集之后、空场基线之前安装的，因此所有此前 baseline/front 候选均不得用于运动
授权；四向标定必须在最终硬件固定后从空场 baseline 重新开始。配置中的厂家原始
`0.511 m` 包络也不再足以代表加装后的整机，必须实测外接雷达最远水平外廓，并重新
计算旋转包络和 self-filter/外参后，才可考虑接入安全链。永久
`rotation_clearance_valid` 继续为 `false`。

本轮还把受控方向标定的 A/B 判据补为两种独立对照：明显的点数增量，或目标遮挡背景
造成至少 `0.04 m` 的稳定中位距离替换。原失败样本在新判据下仍失败；相关
rotation-crosscheck/lease 回归共 21 项通过。

### 外接雷达照片推断候选（同日补充）

操作者提供了加装后照片。结合 Go2-W 官方/固定 URDF 几何、照片中安装脚位置、Pandar
静态地面拟合，以及与内置点云的低置信度配准，记录了不发布 TF 的候选：

```text
base_link -> pandarxt16_link_unvalidated
xyz = (+0.130, +0.015, +0.014) m
rpy = (+0.385, +0.905, +11.357) deg
```

外接雷达拟合地面高度 `0.541868 m`、RMSE `0.005402 m`、倾斜约 `0.984°`，因此
z/roll/pitch 的证据强于照片。x/y 来自照片尺度；8+8 帧双雷达配准中位误差仍有
`0.3516 m`、10 cm 内比例仅 `13.54%`，yaw 只作初值。候选平移不确定度为
`(0.060, 0.035, 0.040) m`，yaw 不确定度 `15°`。

照片同时显示松弛电缆环是最高点。水平投影看起来仍落在原站立 footprint 内，故
`0.511 m` 仅作为未复核候选保留；整机高度候选提高为 `0.65 ±0.05 m`。完整记录见：

```text
configs/go2w/hesai_pandarxt16_mount_candidate.yaml
reports/go2w_pandarxt16_mount_candidate_20260813.md
```

正式驱动继续保持 `transform_flag=false`，未发布 `base_link -> Pandar` TF，未修改
安全碰撞高度和旋转门禁，也未启动任何运动。
