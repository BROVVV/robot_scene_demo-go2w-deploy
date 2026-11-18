# §8.2 真机短程验收记录（2026-09-18）

对应计划书：`docs/GO2W_WebUI三维物体框中文统一与低置信过滤_整改实施计划_20260917.md` §8.2。

会话：`outputs/live_runs/search_20260918_125223_d46440a8`（目标「黑色沙发」，12:52:23 起）

七条通过条件**全部通过**。过程中发现三件超出计划书范围的事，两件已在本次修掉，
一件留待评审（见 §5）。

## 1. 前置检查（跑之前逐条验过）

| 项 | 结果 | 判据 |
| --- | --- | --- |
| 运动栈真的活着（不是假绿） | ✅ | `/tmp/go2w_network_low_state.json` 落后墙钟 0.03~0.05 s；重启后 lease id 换新为 `1789705721972768` |
| 就绪闸门 | ✅ | `ready=True`、`blocking=[]` |
| SLAM 已重置 | ✅ | `mapping_session_id=1 mapping_health=HEALTHY revision=5 web_points=1405`，退出码 0 |
| 快照写端跑的是整改后代码 | ✅ | 重启为 PID 843416，三个新门限参数在命令行里 |

判据取 `network_low_state.json` 的 mtime 而不是 `lease_status/alive` —— 后者在机器狗重启后是假绿
（见 [[go2w-motion-stack-stale-after-robot-reboot]]）。

**预算**：请求体带 `max_seconds=300`。会话事件 `session_start.budget.max_search_seconds`
逐字为 `300.0`，即预算确实进了 `policy.budget`。实际跑了 **476 s**，原因见 §5.3。

**终止方式**：到 441 s（超预算 141 s）仍未停，我主动 POST `/api/search/stop`。
机器狗停稳已单独验证：5 s 内 `slam_map_trajectory.json` 点数与末点均不变、文件 mtime 距今 80 s、
`sdk_sport_state.velocity=[0,0,0]` / `yaw_speed=0.0`。

## 2. 七条通过条件逐条

判定脚本 `/tmp/verify_82.py`（只读），判据号与计划书一致。

```text
[PASS] §8.2-1 可见名称 100% 为中文
[PASS] §8.2-2 每个显示对象可反查原始标签与来源事件
[PASS] §8.2-3 所有显示对象满足组合门限
[PASS] §8.2-4 连续三次轮询物体数量与名称稳定
[PASS] §8.2-5 不同位置同类物体保持不同 persistent ID
[PASS] §8.2-6 同一物体重复观测不重复画框
[PASS] §8.2-7 召回/复核链路仍工作，低分候选仍在日志中
```

现场显示的 5 个框（快照 `input=12 displayed=5 filtered=7`）：

| object_id | label_zh | raw_label | conf(均值) | conf_max | n | status | box |
| --- | --- | --- | --- | --- | --- | --- | --- |
| obj_001 | 办公椅 | office chair | 0.887 | 0.887 | 1 | TENTATIVE | ✓ |
| obj_002 | 垃圾箱 | bin | 0.769 | 0.850 | 5 | CONFIRMED | ✓ |
| obj_003 | 计算机显示器 | computer monitor | 0.635 | 0.643 | 3 | CONFIRMED | ✓ |
| obj_004 | 垃圾箱 | bin | 0.573 | 0.625 | 3 | CONFIRMED | ✓ |
| obj_056 | 电脑椅 | computer chair | 0.695 | 0.695 | 1 | TENTATIVE | ✓ |

第 4 条是三次间隔 3 s 的轮询，`(object_id, label_zh)` 签名三次完全一致。
第 6 条：5 个框 5 个唯一 ID，无重复画框，其中 3 个是 n>1 的重复观测。

### 第 2 条一度是 FAIL —— 已修（见 §5.1）

修之前，快照里的 `raw_label` 与 `label_zh` 同为中文（`办公椅`），
与计划书 §6 F.4 写死的 `"raw_label": "speaker"` 不符，"反查原始英文标签"因此落空。
修完重跑，同一条判据从 FAIL 翻成 PASS，且逐帧事件里确实能查到原话：

```text
obj_001 办公椅     ← raw_label='office chair'     (逐帧事件中出现 2 次)
obj_002 垃圾箱     ← raw_label='bin'              (逐帧事件中出现 5 次)
obj_003 计算机显示器 ← raw_label='computer monitor' (逐帧事件中出现 2 次)
obj_004 垃圾箱     ← raw_label='bin'              (逐帧事件中出现 5 次)
obj_056 电脑椅     ← raw_label='computer chair'    (逐帧事件中出现 1 次)
```

### 第 7 条的两个数

`semantic_object_localized=16`、含 verif 的事件 23 条、逐帧候选 **145 条其中 118 条低于 0.35**
（最低 0.120）—— 计划书 §3「不提高底层召回门限」这条硬约束在数据上是成立的：
低分候选原样留在日志里可供取证，被拦的只是"写进持久实体图"这条路。

入口门（`semantic_object_ingest_filter`）本次记了 6 次，例如：

```text
in=24 kept=5 drop={'low_confidence': 15, 'non_object_label': 4, 'invalid_score': 0}
in=23 kept=2 drop={'low_confidence': 21, 'non_object_label': 0, 'invalid_score': 0}
```

## 3. 只有 §8.2 数据能证的两件事

### 3.1 平均聚合 ≠ 最大值（§8.1 明确说这条要等新数据）

| object_id | conf(均值) | confidence_max | 差 |
| --- | --- | --- | --- |
| obj_002 | 0.7692 | 0.8498 | +0.0806 |
| obj_003 | 0.6346 | 0.6434 | +0.0088 |
| obj_004 | 0.5734 | 0.6246 | +0.0512 |

`obj_002` 在旧口径下会显示 0.850，新口径 0.769 —— **差 0.08，是屏幕上看得见的差别**。
§8.1 的历史回放证不了这一点（老事件里只有一个数），本轮证到了。

### 3.2 §8.1 的预测命中：新口径下不再有"整条实体都低于入口门"

离线回放同一份新会话：`全部观测都低于入口门的持久 ID: 0 个 / 共 12 个`。
对照老归档（§8.1）是 **30 个 / 41 个**。这正是 §8.1 §3 末尾预测的方向 ——
新入口门下实体只由高分观测催生，所以不会再出现"靠一次低分观测先建出来"的实体。

## 4. 交付物

| 计划书要求的 | 落在哪 | 状态 |
| --- | --- | --- |
| 新会话 `events.jsonl` | `outputs/live_runs/search_20260918_125223_d46440a8/events.jsonl` | ✅ 19 MB |
| `semantic_map.json` | 同目录 `semantic_map.recovered.json` | ⚠️ **替代品**，见下 |
| `slam_map_3d.json` | `outputs/autonomous_search/runtime/slam_map_3d.json` | ✅ |
| WebUI 三维地图截图 | 同目录 `webui_map3d_payload.json` | ⚠️ **载荷而非像素**，见下 |
| `semantic_object_ingest_filter` 统计 | `events.jsonl` 内 6 条 | ✅ |

两处替代的理由，不掩饰：

- **`semantic_map.json` 没写成，因为会话是被强停的。** 该文件由
  `_write_session_artifacts()` 在**收工那一刻**写（`run_semantic_exploration.py`），
  而 worker 在 `EXECUTE` 相位被终止，收工路径没跑到。替代品是从
  `webui_state.json` 的 `spatial.semantic_graph` 抽出来的 —— 它与
  `explorer.semantic_graph.to_dict()` 的顶层键逐字相同
  （`schema_version/revision/current_place_id/nodes/edges/places/objects/frontiers/observations/object_topology`），
  `revision=6 place=P1 nodes=134 objects=122`。**故意不叫 `semantic_map.json`**：
  它是飞行中抓的同一份结构，不是 worker 自己落的盘，改名以免被当成原生产物。
- **截图没拍成。** 无头 Firefox 与本机已在跑的 Firefox 抢同一个 profile，
  换独立 profile 又有"WebGL 渲染出空白却看着像证据"的风险（见
  [[go2w-webui-map-blur-is-rendering]]：渲染层的问题最容易伪装成数据问题）。
  改为归档**三维视图实际收到的那份载荷**（`curl /api/slam/map3d`），
  它逐字带着 5 个框的 `label_zh`/`raw_label`/`status`/`confidence` 与 `object_filter_stats`。
  截图需要人在 GUI 前点一下，见 §6。

## 5. 本次发现的、计划书范围外的四件事

### 5.1 `raw_label` 退化成中文（已修，就是第 2 条 FAIL 的真因）

真因：`semantic_object_gate.py` 的 `normalize_snapshot_object()` 取
`record["label"]`，而整改后实体事件的 `label` 已经是**中文关联键**，不再是检测器原话。
老归档里 `label` 是英文（`bin`/`barrel`），所以 §8.1 那轮看不出来 —— 是整改自己引入的。

修法（只动 `app/spatial/semantic_object_gate.py` 一个文件，写端与回放器共用）：
`load_object_records()` 顺带扫 `semantic_frame_objects_localized`，用
**`provenance.frame_id` + `provenance.source_object_id`** 把创建那一帧的原话捞回来，
捞不到才退回 `record["label"]`（老归档行为不变）。

一处必须记下来的坑：**不能用 `source_observation_ids` 配对。** 它会被合并累积成一串，
而各帧的 `gsam_obj_00N` 编号是各自独立的。实测 `obj_002` 的
`source_object_id=gsam_obj_001` 在创建帧 69870 里是 `office chair`，在合并进来的
72243 帧里却是 `bin` —— 串用必然张冠李戴。我第一版就是那么配的，
发现后才改成 `provenance.frame_id`。

**回归验证**：老归档（§8.1 那份）重跑，显示的 4 个与分数逐位不变
（办公椅 0.822 / bin→垃圾桶 0.694 / barrel→桶 0.673 / 黑色底座 0.530），
只是 `raw_label` 从中文归位成英文 —— 正是 §8.1 审计表本来的列样子。

### 5.2 非物体过滤的中文那条臂其实没抓住（顺带证出来的）

本轮 `obj_005`（`raw_label='floor'`、`label_zh='地板/地面'`、conf 0.603）的丢弃原因
**从 `below_display_threshold` 变成了 `non_object_label`**。

原因是 `label_zh` 是词典拼出来的**合注形式**（`地板/地面`、`桌子/表格`），
而 `NON_OBJECT_LABELS` 里没有这种带斜杠的写法 —— 中文那条臂匹配不上，英文那条臂才匹配得上。
显示结果不变（两种原因都是丢），但这说明：**§5.1 那个退化不只是"取证字段不好看"，
它同时让非物体过滤器少了一条腿。** 若当时修反方向（干脆不要英文），这道门会静默变松。

### 5.3 `max_seconds` / `--max-seconds` 完全无法终止搜索

`AutonomousExplorer.run()` 的外层循环：

```python
started = float(self._now())
elapsed = float(self._now()) - started        # 行 503，循环外
while budget.remaining(elapsed_sec=elapsed, planning_cycles=..., motion_steps=...):
    ...                                       # 循环体 508..1249
elapsed = float(self._now()) - started        # 行 1251，在 while 的 orelse 里
```

AST 核对：**循环体内唯一一处 `elapsed` 赋值在 1251 行，而它在 `while` 的 `orelse` 子句里，
不在循环体内（体止于 1249）**。于是条件每次都用冻结的初值（≈0.0）重算，
时间那一维永远不触发。`max_search_seconds` 只在循环**已经因为别的原因退出之后**，
才在行 1253 用来把 `finish_reason` 标成 `TIMEOUT` —— 即"事后追认"，不是"到点叫停"。

实证：`session_start.budget.max_search_seconds = 300.0`，实际跑到 476 s 才由外部停掉。

这条**不在本次整改范围内**，属既有缺陷，我不擅自改（改它会动搜索主循环的终止条件）。

### 5.4 超过 120 s 的搜索被正常停止时，会被报成"启动失败"

我 POST stop 后，WebUI 的终态是：

```text
status=FAILED  finish_reason=WORKER_INTERRUPTED
error: 搜索启动超过等待时限，worker 未进入运行状态；已保留启动诊断。
```

这句话是假的：搜索好端端跑了 476 s，是我主动停的。真因在
`search_session_service.py` 的 `_reap_stale_locked()`：`STOPPING` 分支里
`timed_out = (time.time() - self._started_at) > STARTING_TIMEOUT_SEC(120)`，
而 `_started_at` 是**搜索开始**的时刻，不是进入 STOPPING 的时刻。所以
**任何跑够 120 s 的搜索，被正常停止时都会立刻命中这个分支**，报出一条指向"启动"的错。
进程也确实是在这条分支里被 `_executor.shutdown()` 收掉的（这就是 §4 里
`semantic_map.json` 没写成的原因）。

同样不在本次整改范围内。

## 6. 需要人确认的两件事

1. **补一张 WebUI 三维地图截图**：浏览器开 `http://127.0.0.1:8765/`，
   三维视图应显示 5 个中文框（办公椅 / 垃圾箱 / 计算机显示器 / 垃圾箱 / 电脑椅），
   状态栏过滤统计为「物体框 5 个 / 已过滤 7 个」。存到会话目录即可。
2. ~~看一眼现场有没有两个垃圾桶~~ **已确认：是两个垃圾桶**（2026-09-18 用户答复）。
   `obj_002` 与 `obj_004` 都是 `垃圾箱`，三维距离 **0.371 m**，只比合并门限
   `entity_map.base_merge_distance_m: 0.35` 高 **2.1 cm**；两者的创建观测在
   **同一帧 69870** 里是两个不同的检测框（`gsam_obj_002` 与 `gsam_obj_004`，都标 `bin`）。
   现场确为两只独立的桶，所以**不合并是正确行为**，2.1 cm 的余量这次是有效的、
   不是侥幸。反过来说，这个余量也确实很薄 —— 若两只桶再靠近 2 cm，
   它们就会并成一个实体。**这条只记录，不改门限**（改 `base_merge_distance_m`
   会动全部物体的合并行为，超出本次整改范围）。

### 6.1 用户贴来的截图里出现的英文（`bin` / `barrel`）—— 查证结论

截图里能认出 `bin` 与 `barrel` 两个英文字样，与 §2 的「5 个中文框」矛盾，
因此单独查了一遍。**结论：当前代码 + 当前数据画不出这张图，该页面是整改前的。**

三条证据（都是实测，不是推断）：

1. 用**真的**前端模块 `app/manual_web_demo/static/slam_map_3d.js`（WebUI 加载的
   就是这一份，sha256 与工作区逐位相同）跑**真的**载荷
   `webui_map3d_payload.json`，逐个物体调它自己的 `objectLabel()`：

   ```text
   obj_001  屏幕上写: "办公椅 0.89"        (raw_label="office chair")
   obj_002  屏幕上写: "垃圾箱 0.77"        (raw_label="bin")
   obj_003  屏幕上写: "计算机显示器 0.63"  (raw_label="computer monitor")
   obj_004  屏幕上写: "垃圾箱 0.57"        (raw_label="bin")
   obj_056  屏幕上写: "电脑椅 0.70"        (raw_label="computer chair")

   全部为中文，无英文可见文字
   ```

   `objectLabel()` 的返回值只有两种可能：`label_zh` 或兜底 `未分类物体`。
   英文根本没有出口。
2. 服务端回应头带 `cache-control: no-store, no-cache, must-revalidate, max-age=0`，
   模板引用带 `?v=go2w-fix-20260911`；8765 端口上只有一个监听进程。
3. **`git show HEAD:app/manual_web_demo/static/slam_map_3d.js` 证明整改前的 JS
   画的就是 `item.label`**：第 124 行 `context.fillText(item.label, ...)`、
   第 166 行 `context.fillText(item.label + conf, ...)`。整改后这两处才改成
   `objectLabel(...)`（`git diff --stat` 该文件 60 insertions / 18 deletions）。

即：整改前的 JS + 整改前的快照（那时 `label` 对部分实体还是英文 `bin`）
恰好能画出截图那个样子。**请刷新页面（F5 或 Ctrl+Shift+R）后复核**，
应看到 5 个中文框。刷新后若仍是英文，那才是真问题，届时再看。

## 7. 测试

```text
§7.5 清单（8 个文件）+ test_vocabulary_label_zh.py
  → 135 passed in 4.79s            （与 §8.1 那次同数）

node tests/test_slam_map_3d_frame.js
  → ALL SLAM3D FRAME TESTS PASSED
```

回放与干跑：

```text
scripts/go2w/replay_object_display_gate.py outputs/live_runs/search_20260918_125223_d46440a8/events.jsonl
写端离线干跑（rclpy 打桩，跑真的 _load_objects()）:
  老归档 → 41 输入 / 4 显示（与 §8.1 逐位一致）
  新会话 → 12 输入 / 5 显示
```

### 方法学留痕（判定脚本自己的两个坑）

`/tmp/verify_82.py` 第一版把逐帧候选的路径写成了 `event["observation"]["scene_objects"]`，
而真机事件里 `scene_objects` 在**顶层**。后果是第 2 条的"原话出现次数"与
第 7 条的"低分候选数"双双恒为 0 —— 看起来像"证据不存在"，其实是**读错了地方**，
而且它会伪造出一个 FAIL。改正后才拿到上面那些真数字。

这正是 [[calibrate-before-trusting-a-negative]] 说的那类陷阱：判据没有分辨力时，
"没检出"与"不存在"长得一模一样。**报阴性结论之前，先证明判据本身能出阳性。**

## 8. 当时没做、现在仍没做的

- 三笔提交（计划书 §9）**未执行**，等工作区评审。
- 计划书要求的两项真机验收只剩 §8.2 的补图（§6 第 1 条）。
- §5.3 / §5.4 两个既有缺陷**未改**，只记录。
