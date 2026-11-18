# §8.1 离线回放记录（2026-09-17）

对应计划书：`docs/GO2W_WebUI三维物体框中文统一与低置信过滤_整改实施计划_20260917.md` §8.1。

回放对象：`outputs/live_runs/search_20260917_190603_df66d115/events.jsonl`
（19:06 那轮真机会话，**整改之前**的代码写出来的，因此里面没有 `label_zh` /
`confidence_max` / `status` —— 正好用来验证"历史数据兼容"这条要求）。

## 1. 怎么跑的

| 工具 | 作用 |
| --- | --- |
| `scripts/go2w/replay_object_display_gate.py` | 读 events.jsonl，跑**入口门**与**显示门**，打审计表 |
| 写端离线干跑（rclpy 打桩，喂真机快照的真实包围盒） | 跑**真的** `WebuiSnapshotWriter._load_objects()` |

两者都不是另写一份判定逻辑：入口门/显示门/去重/补历史字段都在
`app/spatial/semantic_object_gate.py`，写端和回放器调的是同一批函数。**回放出来的
结论因此对写端有约束力** —— 这一点是这次专门重构出来的，早先写端里另有一份
"读事件 + 去重"的私有实现，那样回放只能证明"另一份代码会怎么判"。

原始输出留档：

```text
outputs/object_gate_replay/replay_20260917_190603_df66d115.txt
outputs/object_gate_replay/writer_dryrun_20260917_190603_df66d115.txt
```

## 2. 结论

```text
持久 ID 41 个 → 显示 4 个（过滤 37）
  丢弃原因：below_display_threshold=29, non_object_label=8

显示的 4 个：
  obj_001  办公椅      0.822  TENTATIVE  box ✓
  obj_002  bin→垃圾桶  0.694  TENTATIVE  box ✓
  obj_003  barrel→桶   0.673  TENTATIVE  box ✓
  obj_022  黑色底座    0.530  CONFIRMED  box ✓
```

写端干跑独立跑出来**同样的 4 个、同样的 stats**（`input=41 displayed=4
filtered=37`），且 `stats["source"]` 指向这份最新的 events.jsonl —— 两条路径互证。

### 与 §5.3 预估的对照

计划书 §5.3 预估"约 4 个：办公椅、垃圾桶/容器、桶、黑色底座"，实测**逐个命中**。
这不是巧合而是判据按预期生效的证据，但也说明一件事：预估和实测都建立在
**旧的 `max()` 置信度**上（历史事件里只有那么一个数），所以这一轮还验证不了
"平均聚合"带来的差异 —— 那条要等 §8.2 的新数据。

## 3. §8.1 五条验收逐条

| 验收标准 | 结果 | 依据 |
| --- | --- | --- |
| 可见名称全部为中文 | ✅ | 回放器自检输出"显示名含英文的: 无"；4 个可见名分别是 办公椅/垃圾桶/桶/黑色底座。`bin`、`barrel` 只留在 `raw_label` 里 |
| `purple/sparkle/corner/floor` 不进入三维物体框层 | ✅ | 四类全部被拦：`purple`(0.754, n=3)、`sparkle`(0.347)、`corner`×3、`地面`×2，原因均为 `non_object_label`。注意 `purple` 是 **0.754 高分**被拦的 —— 拦它的是标签判据不是分数判据 |
| `speaker=0.395` 不显示 | ✅ | `obj_025 speaker→音箱 0.395 CONFIRMED n=5` → `below_display_threshold`。5 次观测 + CONFIRMED 身份都齐了，仍卡在确认档 0.45 上 |
| 低于 `0.35` 的观测不进入新的持久实体图 | ✅ | 逐帧原始观测 49 条 → 入口门留 10 条（`low_confidence=29`、`non_object_label=10`）。41 个持久 ID 里 **30 个**的全部观测都低于 0.35，新门下不会存在。注：回放读的是检测端视角的 49 条（入口门入参的**上界**，见回放器 `iter_frame_observations` 的口径说明），线上 `semantic_object_ingest_filter` 的精确数要以真机为准 |
| 原始事件数量不因过滤而减少 | ✅ | 入口门只吃 `localized`，逐帧事件写的是 `observation.scene_objects`，是两张不同的表；新增单测 `test_gate_does_not_touch_the_raw_observation_log` 钉住"不原地改元素、不改入参列表" |

### 关于第 4 条的一处口径说明

老事件里的 `confidence` 就是当时的 `max()`（整改前 `_merge` 取最大值），所以
`confidence_max < 0.35` ⟺ **这个实体的每一次观测**都低于门限 —— 上面说的 30 个
是严格成立的。

反过来**不成立**：`max ≥ 0.35` 只说明"后来见过一次好的"，不排除它是靠一次低分
观测先建出来的。新入口门会挡住那次建实体，等高分观测到达时实体才诞生，所以
新口径下实体的 `observation_count` 会比老口径小。这一点在 §8.2 的新数据里应当
能直接看到，本轮历史回放看不出来。

## 4. 顺带确认的几件事

- **显示路径没有被入口门碰到。** `enrich_scene_objects_with_spatial(scene_objects,
  localized)` 跑在入口门**之前**（`run_semantic_exploration.py:2117` vs `:2198`），
  所以 WebUI 的"当前检测"仍带全部高召回候选和坐标；被拦的只是"写进持久实体图"
  这条路。这是计划书 §3"目标搜索仍需要高召回候选"能同时成立的原因。
- **下标没有错位。** 过滤后 `localized` 被 `update_with_associations()`、
  `association_by_index`、`semantic_object_localized` 的 `enumerate` 共用同一张表，
  三处都在过滤之后取。另确认 `state["localized"]` 只写不读，没有第二处消费者。
- **实体关系图不受下标耦合影响。** `sync_from_observation()` 的 `labels` /
  `spatial_objects` 两个入参在函数体里根本没被使用，关系边只通过
  `update_result.associations` 解析 —— 过滤后不会串到别的物体上。
- **`source_prompt_term` 已在链路上。** `DetectedObject` → `_gsam_to_dict()` →
  `dict(obj)` 透传，逐帧事件里能看到（计划书 §4.1 要求保留）。

### 与计划书字面的一处偏离（按"最小改动"处理，供评审）

§6 阶段 D.2 写的是"新建明确命名的 `persistent_candidates`"，实际做法是把
`localized` **重新绑定**到过滤后的表（`localized, dropped = filter_...`），
没有引入第二个变量名。

理由：D.4 自己警告的正是"改完 `localized` 又用旧索引表"。下游引用 `localized`
的地方有十几处（`update_with_associations`、`association_by_index`、
`semantic_object_localized` 的 `enumerate`、`state["localized"]`…），引一个新名字
就得同时改所有引用点，**每漏一处就是一次 persistent ID 串物体**；而就地重绑定
在类型上不可能漏（旧名从此不存在）。这与既有的 `localized = drop_dynamic_objects(
localized)` 也是同一个写法。D.3 要的"三处消费同一份表"因此是**结构性**成立的，
不是靠人记着。

## 5. 撤回：`start_manual_web_demo.sh:156` 的归档默认值 **不是**隐患

我一度把这里记成"定时炸弹"（归档 mtime 一刷新就会永久压住活会话），**读完
`tests/test_webui_snapshot_object_source.py` 的头部说明后收回这个判断**。

`start_manual_web_demo.sh:156` 在 `GO2W_WEBUI_SNAPSHOT_EVENTS` 未设时把
`objects_events` 默认指向最新一份 Dataset D 归档。这**不是**遗留疏忽，而是仓库
已经明确选定的语义：

- 规则是「候选 = glob 命中 ∪ 显式路径，取 mtime 最新的那一份」；
- `test_live_session_beats_a_newer_archive` **专门**钉住"活会话没动过时，更新的
  归档应当胜出"，文件头的注释写明它挡的是另一种改错方向 ——「图省事写成'一律以
  实时会话为准'」。所以那不是我以为的坑，是被人有意固定下来的行为。

真正残留的场景很窄，而且会自愈：**正在搜索时**若恰好重跑一次 Dataset D 抽取，
归档 mtime 变成"此刻"，可能短暂压过活会话；但活会话每追加一条事件就刷新 mtime，
下一轮比较（写端 `period_s` 默认 5 s）就抢回来了。搜索结束后活会话不再追加，
此时显示更新的归档**正是**上面那条用例规定的行为。

结论：**不改**。

## 6. 还没做的

- §8.2 真机短程验收（计划书点名要安全监督在场，等人回来）。
- **写端必须重启**：PID 705521（18:41 启动）跑的还是整改前的代码 —— 线上快照里
  11 个物体仍是老口径（只有 `label`，没有 `label_zh`/`raw_label`/`decision`/
  `object_filter_stats`）。重启走 `start_manual_web_demo.sh`（别单起；它同时管着
  map topic 与 pid 文件，且这次新增的三个 ROS 参数就在它里面）。
  重启后现场应能直接看到：物体框从 11 个降到 4 个、名字全是中文、状态栏多出
  "物体框 4 个 / 已过滤 37 个"。
- §8.2 采集清单：新会话 `events.jsonl` / `semantic_map.json` / `slam_map_3d.json`
  / WebUI 三维地图截图 / `semantic_object_ingest_filter` 统计（写端重启后才能
  在快照里看到 `object_filter_stats`，入口门的统计则在 `run_semantic_exploration`
  这一步，重启的是**搜索进程**不是写端）。

## 7. 测试

### 本整改相关

```text
pytest -q  §7.5 清单（9 个文件）+ test_vocabulary_label_zh.py   135 passed in 3.74s
node tests/test_slam_map_3d_frame.js                            ALL SLAM3D FRAME TESTS PASSED
```

### 仓库全量（含既有失败，逐条定性）

```text
.venv/bin/python -m pytest -q \
  --ignore=tests/test_map_quality_evaluator.py \
  --ignore=tests/test_semantic_frame_object_observations.py \
  --ignore=tests/test_streamlit_video_mode.py
→ 6 failed, 1515 passed, 1 skipped in 464.47s
```

**这 6 条一条都不是本整改引起的**，逐条依据如下（判据是"在 `HEAD` 上是否已经红"，
不是"看着像不像我改的"）：

| 失败用例 | 真因 | 依据 |
| --- | --- | --- |
| `test_lio_motion_alignment_analysis.py::test_icp_recovers_a_known_ten_degree_rotation` | `.venv` 里没有 scipy | 报错就是 `scripts/go2w/analyze_lio_motion_alignment.py:392: ModuleNotFoundError: No module named 'scipy'`；与它同因的还有两个 collection error |
| `test_live_exploration_graph.py` ×3 | `/api/search/*` 路由在 **HEAD 就不存在** | `git cat-file -p HEAD:app/manual_web_demo/web_server.py \| grep -c api/search` → **0**，而同在 HEAD 的该测试文件里出现 **9** 次。这三条是上一代架构留下的僵尸用例，超时只是表象（`_wait_finished` 等一个永远不会出现的状态） |
| `test_manual_web_demo_config.py::test_hold_mode_params` | 默认值在 HEAD 就是 1.0，测试仍期望 0.15 | `git diff HEAD -- app/manual_web_demo/config.py` 为空（该文件根本没被改过），而 `config.py:96` 是 `turn_yaw_rate: float = 1.0` |
| `test_evaluate_live_search_reasoners.py::...tracked_recording` | 脏工作区把**被跟踪的**录制目录删了 | `git status` 里 ` D outputs/live_runs/search_20260901_120501_96392a00/*`，用例正是读这个目录 |

两个 collection error（`test_map_quality_evaluator.py`、
`test_semantic_frame_object_observations.py`）同为缺 scipy，**会中断整个 collection**，
所以上面那条命令必须显式 ignore 掉它们。

`tests/test_streamlit_video_mode.py` 是**主动排除**的，不是漏跑：它会写真 `outputs/`
并往被跟踪的 `data/memory/observational_memory.jsonl` 追加 mock 行，而那份文件本来就
已经处于修改状态 —— 跑它等于往真观测记忆里掺假数据，且会让后续 diff 更难读。
