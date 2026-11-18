# 任务书：在不连机器狗的前提下改造自主搜索的寻路思维链

> 面向对象：一个接手本仓库的 AI 编码助手。
> 本文件由 2026-09-16 的实际排查结果写成，其中的文件行号与数字都经过复现。
> **所有思考与交付物用中文。**

---

## 0. 一句话

改 `app/reasoning/semantic_navigation/` 下的寻路思维链（以及它的离线判据与测试），
**全程不连机器狗、不发运动指令、不调付费 API**，用仓库里已有的离线 A/B 回放器
验证改动。

---

## 1. 硬约束（先看这一节，违反其中任何一条的改动一律不接受）

### 1.1 禁止改三维建图（计划书 §10.2，权威原文在 `docs/GO2W_合并计划书_20260911_RAMPlus双通路_含交接书_20260914.md:4467`）

> **禁止改三维建图。**

涉及：`min_local_map_matching_rate`、LIO 滤波器、`app/perception/depth_object_localizer.py`、
`app/spatial/semantic_object_map.py`、`app/slam/slam_bridge.py`、
`app/spatial/slam_web_map_state.py`、`ros2_ws/src/plain_slam_ros2`、
`ros2_ws/src/go2w_plain_slam_bridge`、**`app/navigation`**。

⚠ **最容易踩的坑**：`app/navigation/candidate_goal_generator.py` 与
`app/navigation/local_scan.py` 就在冻结清单里的 `app/navigation` 下。
你在读思维链链路时一定会读到它们、也一定会觉得"改这里最直接" —— **不许改**。
思维链本体的可改范围是 `app/reasoning/`（不在冻结清单内）。

为冻结文件**写测试**是允许的；**修改**它们不允许。

### 1.2 Git（计划书 §10.3）

禁止 `git add .` / `git reset` / `git clean` / `git checkout`。提交必须**逐条点名文件路径**。
测试注入一律用 `/tmp` 备份 + `cp` 还原 + `sha256sum` 逐字节校验。

### 1.3 风格

**禁止过度设计、禁止防御性编程、禁止工程化编程。改动落最小处。**
不要为了可测试性去拆函数、抽接口、加配置项。

### 1.4 判据纪律（本仓库最硬的一条规矩）

**没证明判据有分辨力的"没检出"不算数。**
- 加任何阈值之前必须实测标定；没有标定的阈值不许进门。
- 加任何测试之前先做**注入校准**：故意把被测逻辑改坏，确认测试真的转红。
  没转红的测试等于没写。
- 阴性结论（"没发现问题"）必须配对照。
- 校准之前先跑**对照组**（未注入必须绿）。对照组不绿，说明这次的"红"来自别处。
  同理：**用例名打错时 pytest 也返回非零**，看起来像"转红"，其实是"没找到用例"。
- 注入/还原之后**必须清字节码缓存**（见 §6.3）—— 否则你测的是上一次的 `.pyc`，
  源文件 `sha256` 却是对的。

### 1.5 安全

- **不要 `ping` / `ssh` / `curl` 机器狗**（`192.168.123.18`）—— 本任务全程离线，不需要它。
- **不要 dump `.env` 的内容**（只查 key 名，值打码）。

---

## 2. 环境（不需要 ROS）

离线工作**不需要 source 任何 ROS 环境**，直接：

```bash
cd /home/mxt/robotscene
PY=/home/mxt/anaconda3/envs/go2_robot_scene_demo/bin/python
```

- 全量 pytest **只有这个解释器**能跑（`.venv` 缺 scipy，会在 collection 阶段整个中断）。
- 已实测：不 source 任何东西即可 `from app.reasoning.semantic_navigation.search_reasoner import ...`。
- ⚠ **不要跑全量 `pytest tests/`**：会卡在 `test_streamlit_video_mode.py`（约 130 s，99% 在等网络）。
  按 `-k` 选子集，或只跑下面第 3 节点名的文件。

---

## 3. 验证工具（现成的，别重造）

### 3.1 离线 A/B 回放器 —— 主要仪器

`scripts/evaluate_live_search_reasoners.py`：对同一个真机录制，让
`legacy` / `semantic_navigation` / `hybrid` 三个后端各提议一次，出报告。
**不调 API、不发运动指令。**

```bash
$PY scripts/evaluate_live_search_reasoners.py \
  --session outputs/live_runs/explore_go2w_20260916_200905 \
  --target "白色垃圾桶" --output-dir /tmp/ab_demo
cat /tmp/ab_demo/reasoner_comparison.md
```

- `--node-index` 选第几次观测（默认 `-1` = 最后一次）。同一会话换 index 会得到不同的图，
  这是看"思维链在哪一帧开始改道"的入口。
- **报告里的 `replay_context` 必须看**：它写着喂给推理器的 `robot_yaw_deg` 与
  `observed_heading_sectors`。这两个量是空的还是真的，决定结论可不可信 —— 见 §5.3。

**语料**：`outputs/live_runs/` 下 309 个真机会话，其中 **102 个可回放**（57 个内嵌图完整，
45 个是 mock/早期会话、字段不全会被拒绝 —— 那是预期行为，不是 bug）。

### 3.3 决策比对器 —— 判断"改动到底改变了什么"

```bash
$PY scripts/compare_reasoner_decisions.py /tmp/baseline /tmp/after
```

参数是两个回放目录（§6.1 那种）。它只比会变的那些字段，绕开
`directive_id` 是 `uuid4()` 这个噪音源。**不要用 `diff -r` 代替**，理由见 §6.2。
它的分辨力是实测标定过的：同一份基线重跑两次 → 0；注入候选方向倒置 → 78；
还原后再跑 → 0。

### 3.2 与思维链相关的离线测试

242 条，全部不需要 ROS：

```bash
$PY -m pytest tests/ -q -k "reason or semantic or navigation or goal_graph or graph_match or exploration or shadow or replay or local_scan"
```

单独点名常用文件：
`tests/test_evaluate_live_search_reasoners.py`、
`tests/test_local_scan.py`、`tests/test_exploration_replay.py`、
`tests/test_live_candidate_goal_generator.py`（**只读不改**，它守的是冻结文件）、
`tests/test_autonomous_explorer.py`、`tests/test_exploration_graph.py`。

---

## 4. 思维链的完整链路（改之前先把这张图看明白）

```
run_semantic_exploration.py:2324   controller.propose(context)
  └─ router.py:48  SemanticSearchController.propose
       ├─ router.py:28  matcher.match(goal_graph, scene_graph, target_profile)
       │    └─ graph_matcher.py  →  GraphMatchResult
       │         产出 state ∈ {ZERO, PARTIAL, STRONG}  +  supporting_anchor_scene_node_ids
       └─ search_reasoner.py:34  SemanticNavigationReasoner.propose
            ├─ ZERO    → explore_unseen    ← 走 _best_unseen_heading
            ├─ PARTIAL → inspect_anchor    ← 走 _anchor_heading
            └─ STRONG  → reobserve_sector  ← 走 _anchor_heading
  ⇒ SearchDirective
       ↓
  两条消费路径，产出不一样：
  (A) app/live_robot/search_directive_adapter.py  （run_autonomous_loop 那条）
  (B) app/navigation/candidate_goal_generator.py:125-171  （★ §22 真机走的是这条）
```

配套的输入侧：

| 文件 | 作用 |
|---|---|
| `app/reasoning/target_profile.py` | 目标短语 → `TargetProfile` |
| `app/reasoning/semantic_navigation/goal_graph_builder.py` | `TargetProfile` → `GoalGraph`（目标节点 + 锚点节点 + 情境节点） |
| `app/reasoning/semantic_navigation/graph_matcher.py` | `GoalGraph` × 观测到的 `SceneGraph` → match |
| `app/reasoning/semantic_navigation/semantic_memory.py` | 负记忆（哪些扇区白跑过） |

打分与选向：`app/navigation/search_policy_config.py`（**冻结目录，只读**）、
`app/navigation/local_scan.py`（**冻结目录，只读**）。

---

## 5. 已知病根（第一个任务，可直接开工）

### 5.1 先分清两条候选路径（**本节最容易读错，先读这里**）

自主搜索有**两套**候选生成器，由 `--spatial-v2` 切换
（`run_semantic_exploration.py:3161-3163`）：

| | V1（默认，不加开关） | V2（`--spatial-v2`；WebUI 起搜索与 **§22 走的都是这条**） |
|---|---|---|
| 候选生成器 | `generate_live_exploration_candidates`（`app/navigation/candidate_goal_generator.py`） | `spatial_candidate_generator`（`run_semantic_exploration.py:2417`） |
| 读不读语义指令 | **读** —— `autonomous_explorer.py:897-899` 用 `directive=` 传进去，`candidate_goal_generator.py:126` 起消费它的 kind / heading / anchor | **完全不读** —— 整个函数体（2417–2870）里 `directive` 出现 **0** 次 |
| 拿它干什么 | 建 `GOAL_INSPECT_ANCHOR` / `GOAL_ROTATE_VIEW` 候选 | 只取 `state["match"].graph_match.state`（zero/partial/strong），决定"要不要做一轮 bounded local scan" |

真机实证（`outputs/live_runs/*/decisions.jsonl`，106 个会话 / 932 条决策）：

| 候选的 `provenance.source` | 条数 | 出自 |
|---|---|---|
| `semantic_navigation_directive`（**新语义思维链**） | **8**，全是 `ROTATE_VIEW` | 只有 2026-09-14 的 2 个会话（V1） |
| `semantic_anchor`（**老的视频管线**，锚点名是英文如 `water dispenser`） | 90 | `search_20260820_*` 那批 |
| `local_scan` | —— | **§22 那次 6 条决策，100% 是这个** |

⚠ **本文件上一版把这两件事混成了一件**，写成"92 会话 / 932 决策里语义候选 98 条、
全是 `INSPECT_ANCHOR`"。复核后：98 条里 90 条是**老的视频管线**，
其 `semantic_reason` 里根本不含 `semantic_navigation`；真正来自新语义思维链的只有 8 条。
两套的 `goal_type` 同名，**只看类型分不出来，必须看 `provenance.source`**。

### 5.2 在 §22 这条路上，思维链的"往哪看"从未被消费

`run_semantic_exploration.py` 全文里 `directive` 只出现两次：
`:2324` 由 `controller.propose(context)` 产出、`:2341` 塞进 `SemanticMatch`。
**此后没有任何一行读它。**

真正决定狗往哪转的是 `select_local_scan_goal`（`app/navigation/local_scan.py:45`，
**在 §10.2 冻结清单内**）：

- 候选只有四个：当前朝向 `+30 / −30 / +60 / −60`（`deltas = (1, -1, 2, -2)`）；
- 打分只看"这个扇区看过没有" + "上一步是不是同方向"（防单向打转）；
- **完全不看场景里有什么** —— `scene_graph`、锚点、目标可见性都进不了这个函数。

### 5.3 裸目标的空指令缺陷（真缺陷，但后果要按路径分开说）

对**不含关系的裸目标**（如「白色垃圾桶」）：

1. 锚点节点来自 `goal_graph_builder.py:54` 的 `_explicit_anchors(profile)` ——
   即**目标短语里写出来的关系**。裸目标 ⇒ `GoalGraph` 里**只有 `goal_target` 一个节点**。
   已构造对比验证：

   | 目标短语 | 锚点/情境节点数 |
   |---|---|
   | `白色垃圾桶`（§22 用的就是它） | **0** |
   | `饮水机旁边的蓝色垃圾桶` | 1 |
   | `桌子上的手机` | 4 |

2. ⇒ `graph_matcher.py:79` 的 `anchors` 恒空（只从 `role ∈ {anchor, context}` 取），
   而 `:72` 的 `elif score >= self.partial_threshold or matched_ids:`
   让**只匹配到 target 也能判 `PARTIAL`**。
3. ⇒ `search_reasoner.py:69-80` 的 `PARTIAL` 分支**无条件**发 `inspect_anchor`，
   此时 `anchor=None`、`anchor_label=None`、heading 走
   `_anchor_heading(None, …)` 硬编码 `30.0`，而 `reason_zh` 却写
   「**已匹配到任务锚点**」—— 自相矛盾。
4. 后果**分路径**：
   - **V1 路**：`candidate_goal_generator.py:133` 的
     `if kind_value in {"inspect_anchor"} and anchor_label:` 把它整条丢掉 ——
     **这一层是正确防御**（没有锚点就不该建"检查锚点"的目标），**错在上游第 3 步**
     ⇒ 语义候选消失、搜索退回 `local_scan`。
   - **V2 路（§22 走的就是它）**：状态被判成 `partial_match` 而不是 `zero_match`，
     但这两者在 V2 里**都**进同一个 bounded local scan 分支
     （`run_semantic_exploration.py:2460` 的 `if match_state in {"zero_match", "partial_match"}`）
     ⇒ **走法一模一样**，受损的只是报告的可信度（`reason_zh` 说"已匹配到锚点"，
     而根本没有锚点）。

**这不是回归。** 09-15 之后没再跑过带关系的目标而已，是设计的必然后果。

### 5.4 ⚠ 上一轮踩过的坑（不要再踩）

排查时第一版回放**漏喂** `robot_yaw_deg` 与 `observed_heading_sectors`
（真机调用点 `run_semantic_exploration.py:2318-2322` 是喂的）。后果不是"略有偏差"：

`search_reasoner.py:_best_unseen_heading` 在**固定六个**候选
`[-30, 30, -60, 60, -90, 90]` 里按 `visited_cost` 挑，而 `visited_cost` 依赖
`observed_heading_sectors`。空集 ⇒ 六个候选**全部平局** ⇒
`min(key=(cost, -aux, abs(heading)))` 退化成**恒选 −30**。

于是得到"49 例全是 −30"、据此写下「语义侧一次方向判断都没有发生过」——
**那是坏仪器测出来的假结论**。修好后同一批数据的分布是
`−30×39 / +30×15 / −60×3`，legacy-vs-semantic 分歧从 7 处变成 14 处。

**纪律**：判断"某侧输出是常量"之前，先确认喂给它的输入不是空集。
常量输出最常见的成因是空输入平局，不是代码里写死了常量。

### 5.5 待拍板：三个层次完全不同的选择（**需要人拍板，不要自行决定**）

先说清一件最容易搞错的事：**改 `search_reasoner.py` / `graph_matcher.py`
（换 kind、拆 PARTIAL 判定、让 `_best_unseen_heading` 用上场景图）在 §22 这条路上
对狗没有任何影响** —— 因为 §5.2，那条路根本不读 directive。
改完只会让报告自洽，狗的走法一模一样。

于是真正要选的是：

| | 做什么 | 改哪里 | 对狗的走法 | 代价 |
|---|---|---|---|---|
| **丙 只收拾思维链** | 修 §5.3 的空指令（`anchor is None` 时不发 `inspect_anchor`），让报告不再自相矛盾；不接线 | `app/reasoning/`（**可改**） | **无影响** | 零风险 |
| **甲 接线** | 让 V2 的候选生成器也读语义指令（把 directive 的 kind / heading 变成真候选） | `scripts/go2w/run_semantic_exploration.py`（**不在冻结清单**） | **有影响** | 真机行为会变，要重跑验收 |
| **乙 改手脚** | 让 `select_local_scan_goal` 用上场景信息（现在只会在 ±30/±60 里挑"没看过的"） | `app/navigation/local_scan.py`（**在冻结清单内，需解除**） | **有影响** | 要动 §10.2 冻结层 |

推荐顺序 **丙 → 甲 → 乙**：丙是把话说对；甲才是"让思维链真的有用"的前提
——**不接线的话，把 `app/reasoning/` 改得再好，狗一步都不会变**；乙要动冻结层，
留到最后。

⚠ 无论选哪个，**离线只能证明"决策变了、且变得有依据"，不能证明"狗走得更好了"**（见 §7）。

---

## 6. 工作方法（照着做）

### 6.1 先打基线

改任何东西之前，对全部可回放会话跑一次回放，每个会话的报告落到独立目录。
把下面这段存成 `/tmp/ab_run.py`（它接受一个参数：输出目录）：

```bash
cd /home/mxt/robotscene
PY=/home/mxt/anaconda3/envs/go2_robot_scene_demo/bin/python

cat > /tmp/ab_run.py <<'PY'
import json, pathlib, sys
sys.path.insert(0, ".")
from scripts.evaluate_live_search_reasoners import evaluate
out = pathlib.Path(sys.argv[1]); out.mkdir(parents=True, exist_ok=True)
for d in sorted(pathlib.Path("outputs/live_runs").glob("*/")):
    f = d / "exploration_graph.json"
    if not f.is_file(): continue
    p = json.loads(f.read_text(encoding="utf-8"))
    if not [x for x in (p.get("nodes") or []) if (x.get("scene_graph") or {}).get("nodes")]:
        continue
    tf = d / "task.json"
    t = json.loads(tf.read_text(encoding="utf-8")) if tf.is_file() else {}
    try:
        evaluate(session=str(d), target=str(t.get("canonical_target") or "目标"),
                 output_dir=str(out / d.name))
    except ValueError as e:
        print("拒绝:", d.name, e)
PY

PYTHONPYCACHEPREFIX=/tmp/pyc_ab $PY /tmp/ab_run.py /tmp/baseline
```

跑完应有 **57 份报告、拒绝 45 个**。

### 6.2 改完再跑一次，用比对器比**决策**

```bash
PYTHONPYCACHEPREFIX=/tmp/pyc_ab $PY /tmp/ab_run.py /tmp/after
$PY scripts/compare_reasoner_decisions.py /tmp/baseline /tmp/after
```

**不要用 `diff -r`**：`search_reasoner.py:_id()` 的 `directive_id` 是 `uuid4()`，
**每个**会话都会报差异 —— 一个什么都没改的改动也能 diff 出 114 个文件的差异，
"有差异"这个信号就此失效。（另外 `diff -rq` 的输出在中文 locale 下是中文，
`grep '^Files'` 一条都匹配不到；真要用就加 `LC_ALL=C`。）

**也不要只看汇总数字**：汇总项会互相抵消。要看的是"哪些会话的提议步变了、
变成了什么、变之后有没有更贴观测到的图"。

### 6.3 ⚠ 陈旧 `.pyc` —— 会让注入校准和 A/B 一起说谎（本轮实测踩到）

Python 用源文件的 `(mtime, size)` 判断缓存的 `.pyc` 还有没有效。如果注入是
**同长度**改写（例如把 `[-30, 30, -60, 60, -90, 90]` 倒成
`[30, -30, 60, -60, 90, -90]`，字节数一模一样），并且还原时保住了 `mtime`，
那么还原之后源文件的 `(mtime, size)` 与旧 pyc 头里记的**完全相等** ——
`sha256sum` 逐字节校验全对，解释器却继续执行**上一次注入的逻辑**。

受控复现（会话 `explore_go2w_20260819_132459`，yaw=93.52、已观测扇区={2,3,4}，
该会话正确答案是 `−60`）：

| 步骤 | 源文件 sha256 | 实际执行的逻辑 | 报告值 |
|---|---|---|---|
| 注入后首跑 | 注入版 | 注入版 ✓ | +60 |
| 还原后（不清 pyc） | **已是原版、校验一致** | **仍是注入版** ✗ | **+60（错）** |
| 清 `__pycache__` 后 | 原版 | 原版 ✓ | −60 |

两个方向都会骗人：

- 注入**没生效** ⇒ 测试保持绿 ⇒ 你会得出"这个判据没有分辨力"的错误结论；
- 还原**没生效** ⇒ A/B 报一堆差异 ⇒ 你会把它算成自己改动的功劳。

**纪律**：每轮注入/还原之后都要清字节码，最省事的写法是给每次运行一个
**全新的** `PYTHONPYCACHEPREFIX`：

```bash
PYTHONPYCACHEPREFIX=$(mktemp -d) $PY ...
```

它让字节码写到临时目录、仓库里的 pyc 一概不读（已实测：仓库内不再生成
`__pycache__`，模块照常导入）。

⚠ **`python -B` 不够** —— 它只禁止*写* pyc，已经存在的 pyc 照样会被读。

### 6.4 每条新测试都做注入校准，并且先跑对照组

1. **先跑对照组**：未注入时该测试必须是绿的。对照组不绿 ⇒ 这次的"红"不是注入的功劳。
2. 注入后必须转红；**没转红就是判据没有分辨力**，要么补测、要么改判据。
3. 确认用例名真的解析到了：**用例名打错时 pytest 也返回非零**，那看起来像
   "转红"，其实是"没找到用例"。校准脚本里显式检查输出里的
   `not found` / `no tests ran`。（本仓库的测试是类方法，
   node id 形如 `文件::类名::方法名`。）
4. 还原用 `git show HEAD:<路径> > <路径>`，比 `/tmp` 备份干净 —— 不会拿错版本。

### 6.5 改一条、验一条

不要一次改三处 —— 那样比出来的差异分不清是谁的功劳。

### 6.6 提交逐条点名路径

message 结尾加：

```
Co-Authored-By: Claude Code <noreply@anthropic.com>
```

---

## 7. 离线**测不了**的三类（不要过度承诺）

1. **阈值类改动**：任何数值门限都需要真机实测标定才敢写进门。
2. **运动执行层的脾气**：亚 3° 旋转会被拒并中断整条腿、`f0.15` 实际只走约 0.10 m、
   `b0.12` 会自动倒车吃掉进度。这些只在真机上体现。
3. **语义帧的真实延迟与退化形态**：离线用的是已落盘的 `SceneGraph`，
   拿不到 VLM 的响应时间、超时、降级路径。

所以：**离线能证明"思维链的决策变了、且变得有依据"，不能证明"狗走得更好了"。**
交付时把这句话写清楚。

---

## 8. 上下文文档（需要细节时按顺序读）

1. 本文件。
2. `docs/GO2W_断电中断与验收A完成交接_20260916.md` **§19** ——
   离线 A/B 回放器的补缺口过程、全库回放结果、上面 §5 的完整版。
   其中 **§19.6** 记着"仪器自己会说谎"的三个实例（`diff -r` 噪音、
   陈旧 `.pyc`、用例名打错造成的假红），§6.2–6.4 那几条纪律就是从那里来的。
3. 同文件 §18 —— 判据挂账与工具归档位置。
4. `docs/GO2W_真机验收_交接书.md` §6.5 —— §22 验收配方与判据（**只读**，
   本任务不发运动指令）。
5. 需要查计划书时：`docs/GO2W_合并计划书_20260911_RAMPlus双通路_含交接书_20260914.md`。
