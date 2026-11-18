# GO2W 合并计划书（2026-09-11 晚）

> **本文档 = 上一份计划书的「剩余部分」与《RAM++ 双通路语义感知详细实施计划》的合并。**
>
> - 上一份计划书：`docs/GO2W_总计划书_合并版_20260910.md`（1106 行，含 §9–§14 六份交接书）
> - 新计划书：`GO2W_RAMPlus_GroundingDINO_SAM2_VLM_双通路语义感知_详细实施计划_20260911.md`（3364 行，66 章）
> - 本合并版：**能让下一个 AI 无缝接手已完成的工作，并按新计划书正确执行下一轮改造。**
>
> 目标分支：`fix/go2w-autonomous-search-lio-topology-target`
> 本地 HEAD：`b1e929a`　远端同名分支：`b1beba2`（**内容逐字节一致，仅 SHA 不同**，见 §1.7）

---

# 0. 怎么用这份文件（给接手 AI，先读这一节）

## 0.1 文件的构成

| 部分 | 行数 | 作用 | 权威性 |
|---|---|---|---|
| **第 0 部分**（本节） | ~60 | 怎么读 | — |
| **第 1 部分**：交接 | ~300 | 现状、已做完、剩余、错误假设、坑 | **最高**（均为实测） |
| **第 2 部分**：合并分析 | ~200 | 表 A 已完成 / 表 B 新增 / 表 C 冲突勘误 | **最高** |
| **第 3 部分**：执行顺序 | ~80 | 合并后的唯一执行顺序 | **最高** |
| **第 4 部分**：新计划书正文 | 3364 | 原文逐字，未改一字 | 次高 |

> **冲突优先级：第 1/2/3 部分 > 第 4 部分（新计划书原文）。**
> 第 4 部分是**别人写的**计划，它的前提假设里有几处与本仓库实际不符（见 §2.3），
> 我没有改动原文，而是把勘误集中放在 §2.3。**读到第 4 部分时，先扫一遍 §2.3 的对照表。**

## 0.2 上手 30 分钟（不改任何代码）

```bash
cd /home/mxt/robotscene
git log --oneline -8                 # 看最近 8 笔提交，对应本文 §1.3
git status --porcelain               # 看本地未提交改动（有，但与本计划无关）
sed -n '1,120p' docs/GO2W_合并计划书_20260911_RAMPlus双通路.md   # 就是本文
```

然后**按 §2.3 的顺序读**：先读 §2.3 勘误表，再读新计划书原文。

## 0.3 三条不可违反的规则（两边计划书都同意）

1. **禁止改三维建图。**（新计划书 §0 第 2 条、§65 第 3 条）—— 详见 §2.3 的 C1/C2/C5。
2. **禁止过度设计**：不建 gRPC/HTTP 服务、不建第二套 semantic map、不建数据库、不建 worker pool、
   不建多级 scheduler、不引入 embedding ontology、不重写 association、不加复杂 fallback。
3. **禁止 `git add .` / `git reset` / `git clean` / `git checkout`**（主仓库有大量未提交改动，
   本轮之前就存在，与本计划无关，不许被覆盖）。

## 0.4 机器狗的当前状态（2026-09-11 晚）

**机器狗没电，正在充电，不在线。** 因此：

- **不需要机器狗就能做的**：新计划书 Phase 0–7 的**全部**（视觉链路在本机 GPU 上跑）+ 装 RAM++。
- **需要机器狗才能做的**：§14.3 漂移复查、真机端到端验收、遥控建图。
- 所以顺序天然是：**先把本机的 RAM++ 双通路做完，机器狗回来了再做真机验收。**（见第 3 部分）

---

# 1. 交接：当前系统真实状态

> 本节是 `docs/GO2W_总计划书_合并版_20260910.md` §1 与 §9–§14 的浓缩。
> **需要细节时读原文件的那几节**，这里只留"接手必须知道"的部分。

## 1.1 拓扑与环境（不搞清楚会白跑一整天）

| 项 | 值 |
|---|---|
| 控制主机 | **192.168.123.99 = 本机**（`enp3s0`），项目根 `/home/mxt/robotscene` |
| 机器狗 | `192.168.123.18`，用户 `unitree` / 密码 `123`，sudo `123`，项目在 `robotscene/` |
| 雷达 | Hesai PandarXT-16（`enp3s0`，192.168.123.201 网段） |
| SLAM | `plain_slam_ros2`（ROS 2 Humble），**不用 rko**（rko 的地图是圆的，见 §1.5） |
| WebUI | `http://127.0.0.1:8765`（手动模式） |
| 视觉推理 | **本机** GPU：RTX 3070 Laptop，**8.22 GB VRAM**，torch 2.5.1+cu121 |
| Grounded-SAM | `external/Grounded-SAM-2`（已装，`GROUNDED_SAM_ROOT` 在 `.env` 里） |
| Python | `/home/mxt/anaconda3/envs/go2_robot_scene_demo/bin/python` |

**本机网络**（2026-09-11 实测，各端点分别验过，不要用 `curl -I` 的
`200 Connection established` 当"通"的证据 —— 那只是代理隧道建立，不代表站点可用）：

| 端点 | 可用 | 实测 |
|---|---|---|
| `huggingface.co` | ✅ | `HTTP/2 200`，且文件 range 下载成功（`206`，0.39 s） |
| `pypi.org` | ✅ | `200` |
| `raw.githubusercontent.com` | ✅ | `200` |
| `api.github.com` | ✅ | `200`（这是**推送**要走的） |
| `git clone https://github.com/...` | ✅ | `git ls-remote` 与 `clone --depth 1` **都成功** |
| `codeload.github.com` | ✅ | `200`（tarball 备用路径） |
| **`git push`** | ❌ | 必挂 → 走 GitHub Git Data API（见 §1.7 第 8 条） |

## 1.1b 接手时的现场进程状态（2026-09-11 15:58 实测）

```text
PID     运行时长    进程
22494   19:29:45   ros2 run（hesai 驱动的外层）
22495   19:29:45   hesai_ros_driver_node                ← 雷达在跑
228499  01:59:35   plain_slam_ros2/slam_3d_node         ← SLAM 在跑
228501  01:59:35   go2w_plain_slam_bridge/pandar_slam_adapter
228588  01:59:34   scripts/go2w/lidar_delay_buffer.py   ← IMU 覆盖修复，在跑
178089  05:51:08   uvicorn（WebUI）                     ← HTTP 200
239014  01:19:59   scripts/go2w/webui_map_snapshot_writer.py
```

**网卡**：只有 `lo` 和 `wlo1 192.168.3.129/24`；`enp3s0` 的 `carrier = 0`
→ **`192.168.123.99` 不存在，机器狗确实不在线。**

> **重要判断：SLAM 栈不依赖机器狗，所以它还活着，地图还能继续长 —— 但 odom 随机器狗一起没了。**
> 接手时**不要**以为"SLAM 死了要重启"，也**不要**在机器狗回来之前重启 SLAM 栈
> （重启 = 语义坐标作废，见 §1.5 的 D4）。

## 1.2 一句话现状

**三维建图、WebUI 显示、语义标注链路都已经跑通并真机验证；语义感知的精度/覆盖率是本轮要治的病，
新计划书的 RAM++ 双通路正是来治它的。**

## 1.3 已做完（带 commit，逐条可查证）

| commit | 做了什么 | 验收证据 |
|---|---|---|
| `740cd23` | **IMU 覆盖修复**（`scripts/go2w/lidar_delay_buffer.py` 缓存点云 0.12 s） | `imu_n` 12–14 → **24–26**；`dsk_max` 0.026 → **0.0015–0.0023 m** |
| `740cd23` | WebUI 地图快照写入器 `scripts/go2w/webui_map_snapshot_writer.py` | 替换了 8 天前的坏快照（内含 `pose=2.28e11`） |
| `740cd23` | `app/manual_web_demo/static/slam_map_3d.js` 加语义标注层 | 用户截图确认 |
| `740cd23` | 地图形状自检 `scripts/go2w/map_watch.py`（`aspect`/`fill`） | 能自动发现"被叠成圆"的坏地图 |
| `740cd23` | **SAM2 真实 mask → depth → 3D Box**（`app/perception/depth_object_localizer.py` +105 行） | 治 D2/D3 |
| `740cd23` | **`app/slam/slam_bridge.py` +206 行**（复用 reproject，治 D4 跨会话语义坐标） | — |
| `740cd23` | **`app/spatial/semantic_object_map.py` +104 行**（association / persistent ID / reproject） | `duplicate_rate=false_split=false_merge=0` |
| `f3b493e` | **IMU 时间轴时钟阶跃重锁**（治地图重影的真因） | 见 §1.5 |
| `bd6d7d0` | WebUI 地图年龄字段变真（原先恒空白） | — |
| `c776eae` | WebUI 按**轨迹 bbox** 定画幅 + 默认正俯视 | 房间占画幅 26% → **58%** |
| `0f0f2b7` `19117c3` `b1e929a` | 三份交接文档 | 就是 §13/§14 |

**另外两处计数/显示口径修复**（在 `740cd23` 之后、`f3b493e` 之前）：

- `semantic_goal_selection_count` **恒为 0 → 实现为 1**（口径没跟上 `--spatial-v2` 执行路径，语义其实一直在生效）；
- WebUI 垃圾桶标签**置信度过滤** `MIN_OBJECT_CONFIDENCE = 0.35`（只改**显示层**，
  不动 `events.jsonl` 原始记录）。

## 1.4 ★ 上一份计划书尚未完成的部分（本文档必须承接的）

| # | 事项 | 需要机器狗？ | 优先级 | 备注 |
|---|---|---|---|---|
| **R1** | **用户肉眼验收 WebUI「地图发糊」修复** —— 刷新 `http://127.0.0.1:8765` 即可（静态资源 `no-store`，普通刷新就够）。地图应占满画幅、墙是细线。 | **否** | **最高（唯一未闭环交付）** | 见 §1.6 |
| **R2** | **§14.3 的 2.5 m 位姿漂移复查** | 是 | 高 | 见 §2.3 的 C1 |
| **R3** | 深层 SLAM 调优（`acc_scale: 0.954`、腰部附近表面厚度 ~2 cm、`pt` 残差 7–10 cm）—— 需一次**全新**建图行走 | 是 | **挂起** | **与新计划书冲突，见 §2.3 的 C2** |
| **R4** | 旧计划书 §10.7 第 3 项：llm 路由基线 D1/D2 | 否（可离线） | 低（延缓项） | 与新计划书无关，独立 |

## 1.5 必须记住的错误假设（**先读，省一天**）

| 曾经以为 | 实测真相 |
|---|---|
| 换 rko 后端能修好地图 | **错**。rko 的地图是**圆的**；plain_slam 才是正确长方形。**切换前先看地图几何**，端点位姿好 ≠ 地图正确 |
| WebUI 地图重叠是 SLAM 造成的 | 第一轮是**8 天前的坏快照**；第二轮（用户第二次反馈"发糊"）是**渲染层** —— 数据里没有重影（分层渲染证明：0–5 m 层是轮廓清晰的房间图，5–12 m 层是**空间上完全分离**的走廊结构） |
| 语义坐标可跨会话复用 | **不能**。`pslam_map` 原点 = 会话启动位置，重启即作废（D4） |
| `mapping_health` 为 null 是本轮回归 | **不是**。手动模式的 writer（`webui_map_snapshot_writer.py`）本来就不写这几个键；**但副作用是 `web_server.py:377` 的 `slam_map_not_drifting` 门在手动模式下恒真 = 这层防护一直失效**（已知项，本轮不动） |
| `${GO2W_WEBUI_SNAPSHOT_MIN_Z_M:-0.15}` 是负数 | **错**。`:-` 是分隔符，展开就是正的 `0.15`。脚本原本就对 |
| odom 链路断了 | **错**。是 **RMW 不匹配** —— 见 §1.7 |

## 1.6 本轮（2026-09-11）已修并且**本轮不要回退**的东西

**「WebUI 地图发糊」= 渲染层问题，不是数据问题。** 两条真因、改了四处：

| # | 文件 | 改动 |
|---|---|---|
| 1 | `scripts/go2w/webui_map_snapshot_writer.py` | 新增 `focus_margin_m` 参数（默认 3.0）+ `_focus_bounds()` + payload 新增 `focus_bounds` 字段 |
| 2 | `app/manual_web_demo/static/slam_map_3d.js` | `centerAndSpan()` 优先用 `focusBounds`；`applySnapshot()` 读 `snapshot.focus_bounds` |
| 3 | 同上 | 默认 `pitch` **0.63 → 1.35**（原 36° 斜视把"远处低点"和"近处高点"压进同一片屏幕区域） |
| 4 | `scripts/go2w/start_manual_web_demo.sh` | 传 `-p focus_margin_m:=${GO2W_WEBUI_SNAPSHOT_FOCUS_MARGIN_M:-3.0}` |

**效果**：画幅 30.9 m → **13.8 m**；房间占画幅 26% → **58%**；端到端（原始地图 + 旧视角）9% → **58%**。
`focus_margin_m` 给 `0` 表示关闭（退回用点云 bbox 定画幅）。

> **判据（重要）**：以后判断"地图糊不糊"，**必须先把 WebUI 的画面复刻出来**，不要看数值。
> 复刻要点：`span=max(2,dx,dy,dz*1.8)`；`scale=min(W,H)*0.78/span`；
> `size=max(1.1,min(3.2,W/720))` 像素/点；6 个 depth 桶按 `depth=sin(pitch)*dz+cos(pitch)*ry` 分，
> `alpha=0.48+0.08*i`。注意 `size` 换算成地面距离（`size/scale`）要跟体素比 ——
> 实测 1.94 px = **10.1 cm** 地面距离 > 5 cm 体素，点会互相盖住。

## 1.7 坑（全部，都是踩过的）

1. **连 SLAM 话题必须用对的 RMW，否则一个话题都收不到。**
   默认 RMW 是 fastrtps，SLAM 栈跑的是 **cyclonedds** 且绑定专用网卡：

   ```bash
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   export CYCLONEDDS_URI=file:///home/mxt/robotscene/runtime/go2w/cyclonedds_enp3s0.xml
   ```

   准确值从活着的进程里抄，比翻脚本可靠：
   ```bash
   tr '\0' '\n' < /proc/$(pgrep -f slam_3d_node | head -1)/environ | grep -iE "RMW|CYCLONE|ROS_DOMAIN"
   ```
   XML 里 `<NetworkInterface address="192.168.123.99">` 是**写死**的 → 机器狗掉线时新建节点会报
   `does not match an available interface` + `rmw_create_node: failed to create domain`，
   **这个报错本身就是"网卡掉了"的哨兵**。

2. **`pkill -f` / `ps|grep` 会杀掉调用者自己**（bash 工具的 cmdline 含目标文件名时），**退出码 144**。
   安全做法：字符拼接 `PAT='webui_map_''snapshot_writer'` 让模式不出现在自己 cmdline，
   且**查杀与启动分成两条命令**。

3. **`source scripts/go2w/setup_environment.sh` 之后还要再清一次 noetic**：
   `/opt/ros/noetic` 存在，污染来自 shell profile；清理必须在 source **之后**做：
   ```bash
   source /opt/ros/humble/setup.bash
   export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v noetic | grep -v '^$' | paste -sd:)
   export PYTHONPATH="$HOME/.local/lib/python3.10/site-packages${PYTHONPATH:+:$PYTHONPATH}"
   ```

4. **禁止直接 `kill` launch 子进程**（会拖垮整个 plain_slam 栈）—— 用 `stop_plain_slam_mapping.sh`。

5. **WebUI 的 `/api/slam/map3d` 不订阅实时话题**，只读快照文件
   `outputs/autonomous_search/runtime/slam_map_3d.json`。地图不刷新先查 writer 是否在跑。

6. **三个 writer 会打架**：曾同时跑三个 `webui_map_snapshot_writer.py`（参数不同），
   后写的覆盖先写的。查 `ps` 确认只有一个。

7. **`slam_map_snapshot.py:55` 是 `result = dict(value)` 原样透传**，不补默认值 ——
   writer 没写的键在 WebUI 上就是 `null`，不是 bug（见 §1.5）。

8. **推送走 Git Data API**：`git push` 必挂。脚本 `/tmp/gh_push_api.py`，
   用法 `GH_TOKEN=xxx /usr/bin/python3 /tmp/gh_push_api.py <repo> <branch> <base_sha> <sha...>`。
   **base 必须用远端 HEAD**（本地没有远端 commit 对象，`git diff <远端sha>` 会报 128）。
   > 副作用：API 重建的 commit **SHA 与本地不同**（当前远端 `b1beba2` ↔ 本地 `b1e929a`，
   > 两边 tree **逐字节一致**）。**下次推送前先 `git fetch` 确认，不要以为是分叉。**

9. **测试环境**（跑 §61 Phase 0 的 baseline 前必读）：
   - `.venv` 里有 streamlit 但**缺 scipy**；
   - **任务解析每次打真实 LLM**，所以**测试失败集合不稳定** —— 基线要连续跑两次再下结论；
   - 跑测试用 conda 环境：`/home/mxt/anaconda3/envs/go2_robot_scene_demo/bin/python -m pytest`。

10. **`external/`、`.env`、`artifacts/` 都在 `.gitignore` 里**（`artifacts/` 有 ~73 GB 实验数据）。
    **`external/` 里装的东西不会被提交**，所以 bootstrap 脚本必须能被重跑。

---

# 2. 合并分析：已做完 / 新增需求 / 冲突

## 2.1 表 A：新计划书 §1 声称「当前代码已经具备」的能力 —— **我逐条核对过，8 条全部为真**

> **价值：防止下一个 AI 因为不敢相信而重做。** 每一条我都指出了实际的实现位置。
> 新计划书 §65 说这些"**绝对不要重复实现**"，是对的。

| § | 断言 | 核对结果 | 实现位置（实测） |
|---|---|---|---|
| 1.1 | Grounded-SAM 已是常驻 worker | ✅ | `app/detectors/grounded_sam_subprocess.py:50` `self._worker: subprocess.Popen \| None`；`:124` `_ensure_worker()`；`:158` `subprocess.Popen(...)` |
| 1.2 | GroundingDINO / SAM2 只加载一次 | ✅ | `app/detectors/grounded_sam_worker.py:23` `--serve`；`:70` `_build_sam2_predictor(...)`；`:191` 定义 |
| 1.3 | SAM2 mask 已进三维定位链 | ✅ | `DetectedObject.mask_area_ratio / mask_rle / mask_size` |
| 1.4 | `DepthObjectLocalizer` 支持 mask depth 和 3D Box | ✅ | `app/perception/depth_object_localizer.py:318-330` 解码 `mask_rle`；`:367` p05/p95 AABB；输出 `box_center_map` / `box_size_map` |
| 1.5 | GroundingDINO 支持 dynamic terms | ✅ | `app/detectors/grounded_sam_subprocess.py:56` `detect_with_dynamic_terms`；`:220` `_build_prompts` |
| 1.6 | `TargetProfile` 支持复杂目标 | ✅ | `app/reasoning/target_profile.py:93` `detector_terms()`；字段 `primary_labels_en` / `aliases_en` / `possible_confusions` / `context_labels_en` 齐 |
| 1.7 | `CropVerifier` 能查属性与关系 | ✅ | `app/detectors/crop_verifier.py:43-46` 输出 `target_match_score` / `category_score` / `attribute_score` / `context_score` |
| 1.8 | `AsyncSemanticObserver` 会丢弃过期帧 | ✅ | `app/live_robot/async_semantic_observer.py:80` `self._pending`；`:137` 只保留最新 pending |

**额外核实**（新计划书没提但相关）：
- `GroundingPromptPlanner` 在 `app/perception/grounding_prompt_planner.py:27`（新计划书 §13 要"最小调整"它）；
- `app/spatial/semantic_object_map.py` **已存在**，`SemanticObjectMap` 在 `:153`，带 `_association_score` / `_hard_gate` / `_merge` / `reproject_observations`；
- 现有相关测试：`tests/test_grounded_sam_prompt_integration.py`、`tests/test_grounded_sam_runtime.py`、`tests/test_target_profile.py`、`tests/test_crop_utils.py`、`tests/test_async_semantic_observer.py`、`tests/test_live_semantic_observer.py`、`tests/test_semantic_entity_association.py` 等。

## 2.2 表 B：新计划书**新增**的需求（相对上一份计划书）

新计划书 66 章的主体就是"RAM++ 双通路"，相对上一份计划书是真新增。归成 8 类：

| # | 新增需求 | 落在哪 |
|---|---|---|
| B1 | **RAM++ 常驻加载** + JSONL `operation=tag` | §6、§7、§30 |
| B2 | **`discover_scene_terms()`** 低频自动发现场景词（≈2 s 一次，不每帧跑） | §9、§10、§11、§31 |
| B3 | **三类词分层与优先级**：`target_mandatory_terms` > `base_terms` > `ram_scene_terms`（目标词**永不**依赖 RAM++） | §4、§12、§13 |
| B4 | **Grounded-SAM 模式下取消 full-scene VLM 枚举**（普通物体改由 RAM++→DINO→SAM2 建图） | §16、§17、§41 |
| B5 | **低频 VLM target sweep 兜底**（连续 4 次无候选 且 距上次 ≥10 s） | §18–§23、§37–§40 |
| B6 | **coverage 与 recall 分离测试**（不能只看"识别了多少物体"） | §42、§47、§48 |
| B7 | **部署面**：`bootstrap_fresh_machine.sh` / `verify_fresh_deployment.sh` 加 RAM++；README 说明双通路 | §5.2、§58 |
| B8 | **硬验收标准 + 强制报告** | §64、§63 |

## 2.3 ★ 表 C：冲突与勘误（**最重要，读第 4 部分前先读这里**）

### C1 ── 新计划书的前提「当前三维建图已经稳定」有一个**未结论的反例**

- **新计划书说**（§0 第 2 条 / §65 第 3 条）：以"当前三维建图已经稳定"为前提，禁止动 SLAM。
- **实际**：`§14.3` 记录了一次**未结论**的异常 —— **14:29–14:41 轨迹点从 109 涨到 119**，
  最后 5 点 `(-2.32,3.08) → (-2.52,3.39) → (-2.72,3.71) → (-2.93,4.04) → (-3.13,4.37)`，
  **近似匀速直线**，每步约 `(-0.20, +0.33)`、间距 **0.38 m**（都超过 0.25 m 去重门限），
  同期地图点数 61959 → 67680（**+5721**）。**而那段时间没有下发过任何运动指令。**
  两种解释**尚未区分**：机器狗掉线前兆（电量低 → 轮速计/IMU 抖动）vs 真实 LIO 漂移。
- **影响**：**不阻塞 RAM++ 改造**（视觉链路与建图解耦）。但 §64 那条
  「三维地图/SLAM 行为没有被修改」要成立，前提是建图本身没坏。
- **处置**：**不改 SLAM**（尊重新计划书）。但机器狗恢复后，**第一件事**是花 5 分钟做 §14.6 第 2 步的复查。
  若复现 → **单独立项**，不要混进本轮 RAM++ 改造。

### C2 ── ⚠️ 直接冲突：上一份计划的「深层 SLAM 调优」vs 新计划书「禁止改三维建图」

| | 说 |
|---|---|
| 上一份计划书 `§14.5 第 3 项` | **要做**深层 SLAM 调优：`acc_scale: 0.954`、腰部附近表面厚度 ~2 cm、`pt` 残差 7–10 cm |
| 新计划书 `§0 第 2 条` | **禁止**改当前已经稳定的三维建图 |

**裁定：以新计划书为准，深层 SLAM 调优挂起（R3）。**
理由：新计划书是**后出的、且是当前要执行的那一份**；且该调优需要一次全新建图行走，成本高、风险大。
**要做也必须等 RAM++ 双通路做完，并由用户单独点头** —— 因为它直接违反本轮的硬性规则。

### C3 ── 上一份计划书 §7 的文件清单有**两个错路径**，新计划书用的是对的

| 上一份计划书 §7 写 | 实际 | 结论 |
|---|---|---|
| 「**新增** `app/mapping/semantic_object_map.py`（当前不存在）」 | 实际是 **`app/spatial/semantic_object_map.py`**，**已存在**（29 KB，`740cd23` 已扩到带 association/reproject） | **按新计划书走**（§0 第 4 条 / §24 / §60 用的都是 `app/spatial/...`）。**不要**去 `app/mapping/` 下新建 |
| 「**必改** `app/slam/slam_bridge.py`（仅复用 reproject，不改写）」 | 已在 `740cd23` 改完（**+206 行**） | 已完成 ✅。新计划书把它列为**只读**，正确 |

### C4 ── 新计划书只读清单里有**三个路径在本仓库根目录不存在**

新计划书 §0 第 2 条与 §60 点名的只读清单：

```text
plain_slam_ros2/              ← 实际在 ros2_ws/src/plain_slam_ros2
go2w_plain_slam_bridge/       ← 实际在 ros2_ws/src/go2w_plain_slam_bridge
navigation/                   ← 本仓库没有；导航代码在 app/navigation/
```

（`github_artifacts/robot_project_snapshot_20260901/` 里另有一份**更早**的快照，含这三个目录，**别用它当基线**。）

**处置：只读约束照旧执行**，只是路径按上面理解。**不要因为"找不到目录"就以为约束不适用。**

### C5 ── 我上一轮改过的文件，有没有落进新计划书的只读清单？

| 我改的 | 新计划书是否只读 | 结论 |
|---|---|---|
| `scripts/go2w/webui_map_snapshot_writer.py` | 不在清单 | ✅ 可继续改 |
| `app/manual_web_demo/static/slam_map_3d.js` | 不在清单 | ✅ 可继续改 |
| `scripts/go2w/start_manual_web_demo.sh` | 不在清单 | ✅ 可继续改 |
| `scripts/go2w/lidar_delay_buffer.py` | 不在清单，**但属于建图链路** | 🔒 **视为冻结** —— 回退它 = 建图回到 `dsk_max 0.026` |
| `configs/go2w/plain_slam.yaml` | 同上 | 🔒 **视为冻结**（`pandar_adapted → buffered`） |
| `scripts/go2w/start_plain_slam_mapping.sh` | 同上 | 🔒 **视为冻结** |
| `app/slam/slam_bridge.py` | **§0 第 2 条只读** | 🔒 已改完（`740cd23`），以后不动 |
| `app/perception/depth_object_localizer.py` | **§60 只读**（§1.4 说能力已有） | 🔒 已改完，以后不动 |
| `app/spatial/semantic_object_map.py` | **§60 只读**（§24 说不重写） | 🔒 已存在，以后不动 |

> **没有真冲突。** 唯一需要"主动加锁"的是中间那三行 —— 它们是**建图链路**的改动，
> 新计划书没列到（因为它只列了 ROS 包和几个 app 文件），但**精神上属于"已稳定的三维建图"**。

### C6 ── 边界提醒：改的是 `grounded_sam` 路线，**不要顺手砍 `llm` 路线**

新计划书 §17 / §41 要求"取消 full-scene VLM 枚举"，这两节都**明确限定在
「Grounded-SAM（+ RAM++）模式」**。而上一份计划书 §8 第 2 条明确：

> `llm` 路线**留作基线与回退**。

**执行时不要越界**：只改 grounded-sam 路径下"因 scene object list 为空而 fall through 到
full-scene VLM"这一处；`--detector llm` 的整图枚举**保持原样**。新计划书 §59 的文件表里
也没有任何 llm 路线文件 —— 一致。

### C7 ── RAM++ 在本机**没有任何安装痕迹** —— Phase 1 之前有一串前置（**已实测出可执行路径**）

| 检查 | 结果 |
|---|---|
| `import ram` | ❌ `ModuleNotFoundError: No module named 'ram'` |
| `external/` 下有 recognize-anything | ❌ 没有（只有 `Grounded-SAM-2`、`bert-base-uncased`、`cyclonedds_0.10.2` 等） |
| `ram_plus_swin_large_14m.pth` | ❌ 全盘没找到 |
| **`external/bert-base-uncased`** | ✅ **已存在** —— RAM++ 的 `ram/models/bert.py` 要用它，**省一步** |
| GPU | RTX 3070 Laptop，**8.22 GB VRAM**，torch 2.5.1+cu121，CUDA 可用 |
| 磁盘 `/` | 剩 **49 GB** |
| `recognize-anything` 在 PyPI 上 | ❌ **404，不在 PyPI**（`pip install recognize-anything` 必失败） |

**权重**：官方在 HF `xinyu1205/recognize-anything-plus-model`，
文件 `ram_plus_swin_large_14m.pth`，**`content-length: 3010210801` ≈ 3.01 GB**
（**注意：不是 5.6 GB**，别按错的数量估磁盘）。HF 可下，已验 range 请求返回 `206`。

**源码**：`git clone --depth 1 https://github.com/xinyu1205/recognize-anything.git` **实测成功**
（≈24 MB）。备用路径（都已验 `200`）：`api.github.com/repos/xinyu1205/recognize-anything/tarball/main`、
`codeload.github.com/xinyu1205/recognize-anything/tar.gz/refs/heads/main`。

**依赖（实测）**：本机 conda 环境已有 `torch / torchvision / numpy / transformers 4.40.2 /
scipy 1.17.1 / regex`；**缺** `fairscale`、`clip`、`pycocoevalcap`、`ruamel`、`open_clip`。
但 `ram/models/ram_plus.py` 的**推理路径只 import** `numpy / torch / 自带的 .bert / .swin_transformer / .utils`，
`get_transform` 走 torchvision —— 所以**真正的推理依赖只有 `timm`**（`fairscale`/`clip`/`pycocoevalcap`
是训练和其它脚本用的，推理不需要）。

> ⚠️ **`timm` 版本冲突（要留意但已实测不致命）**：repo 的 `requirements.txt` 钉的是
> `timm==0.4.12`，本机是 **1.0.28**。已实测 `from timm.models.layers import DropPath, trunc_normal_`
> 在 1.0.28 上**仍能用**（只打 `FutureWarning`），所以**先不要降级 timm**（降级会连带影响
> Grounded-SAM-2）。真出 `ImportError` 再按需处理。

**结论：Phase 1 的第一件事是"装依赖 + 下权重"，不是写代码。先做完这个再动手，否则后面全是空转。**

另外 **8 GB VRAM 让 §55 的 OOM 场景成为真实风险**（不是理论风险）：
RAM++ swin-large + GroundingDINO SwinT + SAM2 同进程。
按 §55 的规则 —— **只有真发生 OOM 才允许拆 `ram_plus_worker.py`，不许预先做双架构。**

### C8 ── §43/§46 要的"真实模型离线验证"用图，仓库里现成就有

| 素材 | 路径 | 数量 |
|---|---|---|
| 真机 D435 帧缓存 | `runtime/go2w/semantic_observe_cache/bundle_*.jpg` | **60 张** |
| 上一轮语义采集 | `artifacts/slam_validation/semanticD_20260910_*/` | 3 组 |
| CropVerifier 的 crop 结果 | `outputs/crops/image_candidate_obj_*.jpg` | ~17 张 |

**不需要新建 `tests/fixtures/ram_semantic/` 也能开工。**
但开工前要先挑出覆盖 §43 各 Scene（普通办公室 / 属性混杂 / 黄色椅子 / 挂书包的椅子 / 稀有设备）的帧 ——
**这些帧是上一个房间的，先确认里面真的有对应物体**（用户原话："这房间遍地都是椅子"）。

### C9 ── 目标 B（稀有目标兜底）**完全押在 VLM 上**，而 VLM 外网不可靠（D7）

- §64「稀有目标」要求：RAM++ 漏词不宣告目标不存在 → target profile 强制词独立运行 →
  连续无 candidate 后**低频 VLM target sweep 会触发**。
- 但上一份计划书 **D7** 实测：**VLM 单次调用 26 s，且 `APIConnectionError` 曾导致整轮实验失败**。
- **不冲突，是风险。** 处置：按 §0 第 6 条，**VLM 不可用时要"明确报错或按配置关闭"**，
  **不要**写"偷偷切换到十种方案"的 fallback。§63 的报告里要如实记 VLM 可用率。

### C10 ── `mapping_health` 恒 null → 手动模式的漂移防护一直失效（**已知项，本轮不动**）

见 §1.5。**不要在本轮顺手修它**（它属于独立立项），但要知道：
真机跑端到端验收时，**`slam_map_not_drifting` 这道门在手动模式下不起作用**，
漂移要靠**人工盯** `runtime/go2w/sessions/plain_slam_go2w.launch.log` 里的 `imu_n`
（判据：`imu_n=0` / `dsk_zero≥1000` / `act<0.7` / 时钟重锁 → 退化）。
**不要拿 `dsk_max` 的绝对值当退化判据** —— 运动时 `dsk_max` 大恰恰是去畸变**在工作**；
真退化时 `dsk` 反而趋近 **0**。

## 2.4 需要用户裁定的事项（**不要自己决定**）

| # | 事项 | 为什么需要裁定 |
|---|---|---|
| **D1** | 深层 SLAM 调优（R3）**做还是不做** | 它**直接违反**新计划书 §0 第 2 条。见 C2 |
| **D2** | §14.3 的漂移若复现，**是否单独立项** | 会影响"建图已稳定"这个前提，进而影响 §64 的验收口径 |
| **D3** | RAM++ 权重下载（~5.6 GB）+ 装依赖，**是否现在就做** | 会改 `external/`（gitignored，不进仓库），但耗时且要占带宽 |
| **D4** | GitHub 令牌轮换 | 用户此前明文贴过 `ghp_...`，**已存在于对话记录中**。建议轮换（已多次提出，用户尚未确认） |

---

# 3. 合并后的执行顺序（**唯一顺序**）

> 把上一份计划的剩余项（R1–R4）与新计划书的 Phase 0–7 排成一条线。
> **分界线是"需不需要机器狗"** —— 机器狗现在在充电。

## A. 不需要机器狗（**现在就能做，建议今天就做完**）

| 步 | 做什么 | 来源 |
|---|---|---|
| **A0** | 让用户刷新 `http://127.0.0.1:8765` 肉眼验收地图（**唯一未闭环交付**） | R1 |
| **A1** | 装 RAM++ 依赖 + 下 `ram_plus_swin_large_14m.pth`（≈3.01 GB，**不在 PyPI，要 clone**）—— 具体命令见 §2.3 C7 | C7 |
| **A2** | Phase 0 Baseline：`git status` / `git diff --stat` / `pytest` 现有相关测试（**跑两次**，见 §1.7 第 9 条） | 新 §61 Phase 0 |
| **A3** | 从 `runtime/go2w/semantic_observe_cache/` 挑出覆盖 §43 各 Scene 的验证帧 | C8 |
| **A4** | Phase 1 RAM++ runtime（worker 加载、`operation=tag`、`discover_scene_terms`、tag filter） | 新 §61 Phase 1 |
| **A5** | Phase 2 RAM → GroundingDINO（验证 **target mandatory 不会消失**） | 新 §61 Phase 2 |
| **A6** | Phase 3 普通场景 Semantic Map（`RAM tag → DINO bbox → SAM2 mask → 现有 DepthObjectLocalizer → 现有 SemanticObjectMap`） | 新 §61 Phase 3 |
| **A7** | Phase 4 复杂目标 VLM verify（"黄色椅子"、"挂书包的椅子"） | 新 §61 Phase 4 |
| **A8** | Phase 5 VLM target sweep（低频漏检兜底） | 新 §61 Phase 5 |
| **A9** | Phase 6 去掉 Grounded-SAM 模式下的普通 full-scene VLM 枚举（**不要越界动 llm 路线**，见 C6） | 新 §61 Phase 6 |
| **A10** | Phase 7 完整回归（`python -m compileall app tests` + `python -m pytest -q`） | 新 §61 Phase 7 |
| **A11** | 写 `reports/RAM_PLUS_SEMANTIC_PERCEPTION_VALIDATION.md`（§63 强制） | 新 §63 |

## B. 需要机器狗（**机器狗恢复后按此顺序**）

| 步 | 做什么 | 来源 |
|---|---|---|
| **B1** | 先确认恢复：`ip -4 -o address show \| grep 192.168.123.99` 且 `ping -c2 192.168.123.18` 通 | 旧 §14.6 |
| **B2** | **§14.3 漂移复查**（优先级最高，它决定 R3 要不要做） | R2 |
| **B3** | 用户遥控走一圈**重新建图**，按旧 §13.9 验收画幅 | 旧 §14.6 |
| **B4** | 真机端到端：RAM++ 双通路 + 用户目标搜索，按 §64 全组验收 | 新 §64 |
| **B5** | 挂起项决策：深层 SLAM 调优（**问用户**，见 D1） | R3 |

## C. 独立延缓项（与上面无关）

| 项 | 来源 |
|---|---|
| llm 路由基线 D1/D2 | R4 |

---

# 4. 新计划书正文（原文逐字，**未改动一字**）

> **从下面这一行开始，是《GO2W RAM++ + GroundingDINO + SAM2 + VLM 双通路语义感知详细实施计划》的原文。**
>
> 它的章节号（§0–§66）**独立编号**，与本文档第 0–3 部分的编号**无关**。
>
> ⚠️ **凡原文与本文档第 1/2/3 部分冲突处，以第 1/2/3 部分为准。** 已知需要修正的地方集中在 §2.3
> （C1 建图稳定性前提、C2 与深层 SLAM 调优的冲突、C3 两处错路径、C4 三个不存在的目录、
> C6 llm 路线边界、C7 RAM++ 前置缺失、C9 VLM 兜底风险）。
>
> 原文中"当前三维建图已经稳定""不要重复实现 §1 的能力"这两类断言，**我已逐条核对属实**（见 §2.1）。

---

# GO2W RAM++ + GroundingDINO + SAM2 + VLM 双通路语义感知详细实施计划

> **项目**：`BROVVV/robot_scene_demo`  
> **目标分支**：`fix/go2w-autonomous-search-lio-topology-target`  
> **计划日期**：2026-09-11  
> **用途**：直接交给 Coding Agent / AI 执行。  
>
> **本计划只改视觉语义感知链。**  
> 当前三维建图已经稳定，**不允许更换或重构 SLAM，不允许引入 LIO-SAM / FAST-LIO，不允许重写现有三维地图链路。**
>
> 最终目标是：
>
> 1. RAM++ 低频自动发现当前画面里的大量常见物体类别；
> 2. GroundingDINO 使用“RAM++ 场景词 + 当前任务强制目标词 + 上下文物体词”定位物体；
> 3. SAM2 继续负责 mask；
> 4. 现有 D435 Depth + `DepthObjectLocalizer` 继续负责三维物体定位和 3D Box；
> 5. 现有 `SemanticObjectMap` 继续负责多帧融合、持久 object ID 和去重；
> 6. VLM 不再承担每帧场景枚举，但保留：
>    - 复杂属性判断；
>    - 关系判断；
>    - 稀有目标复核；
>    - GroundingDINO/RAM++ 长时间无候选时的低频整图漏检兜底；
> 7. “黄色椅子”“挂着书包的椅子”“特殊设备盒”等复杂目标，不能因为 RAM++ 没识别到完整语义就被漏掉。

---

# 0. 给 Coding Agent 的硬性执行规则

在修改代码前先完整阅读本计划和被点名的现有文件。

必须遵守：

1. **以当前本地工作区为真实基线。**
   - 用户可能有尚未提交的修改。
   - 开始前记录：
     ```bash
     git status
     git diff --stat
     git branch --show-current
     git rev-parse HEAD
     ```
   - 不覆盖、不回滚与本计划无关的本地修改。

2. **禁止改当前已经稳定的三维建图。**
   以下模块原则上只读：
   ```text
   plain_slam_ros2/
   go2w_plain_slam_bridge/
   app/spatial/slam_web_map_state.py
   app/slam/slam_bridge.py
   app/spatial/plain_slam_spatial_provider.py
   ```
   除非只是为了读取现有输出做极小接口接线，否则不要修改。

3. **禁止重写当前 Grounded-SAM 基础设施。**
   当前分支已经有：
   - persistent `Popen` worker；
   - `--serve` JSONL 模式；
   - GroundingDINO 一次加载；
   - SAM2 一次加载；
   - `mask_rle`；
   - D435 mask depth；
   - 3D box；
   - timestamp-aware map transform。

   **不要重复实现这些能力。**

4. **禁止新建第二套 Semantic Map。**
   必须继续使用：
   ```text
   app/spatial/semantic_object_map.py
   ```

5. **禁止工业化过度设计。**
   不要引入：
   - gRPC；
   - Redis；
   - Kafka；
   - Celery；
   - 新 HTTP inference service；
   - 新数据库；
   - worker pool；
   - 多级 scheduler；
   - heartbeat/watchdog；
   - 复杂 retry framework；
   - 新的 world model 框架。

6. **禁止防御性编程泛滥。**
   - 不为理论上每种异常写一层 fallback；
   - 不加大量 try/except；
   - 不增加复杂状态机；
   - 使用现有错误处理风格；
   - RAM++ 不可用时明确报错或按配置关闭，不要偷偷切换到十种方案。

7. **新增文件越少越好。**
   推荐最多新增：
   ```text
   app/detectors/ram_plus.py
   ```
   如果直接在现有 Grounded-SAM worker 中实现更清楚，也可以不新增该文件。

8. **修改后必须自己跑测试。**
   不能只给代码不验证。

---

# 1. 当前代码已经完成的能力：不要重复造轮子

Coding Agent 必须先确认以下事实。

---

## 1.1 Grounded-SAM 已经是常驻 worker

现有：

```text
app/detectors/grounded_sam_subprocess.py
```

已经维护：

```python
self._worker: subprocess.Popen | None
```

并通过：

```python
_ensure_worker()
```

只在 worker 不存在时：

```python
subprocess.Popen(...)
```

启动：

```text
grounded_sam_worker.py --serve
```

因此：

> **本次绝对不要再写“把 subprocess.run 改成 Popen”的代码。已经改好了。**

---

## 1.2 GroundingDINO / SAM2 已经只加载一次

现有：

```text
app/detectors/grounded_sam_worker.py
```

已经明确支持：

```text
--serve
```

启动时：

```python
grounding_model = load_model(...)
sam2_predictor = _build_sam2_predictor(...)
```

之后循环读 stdin JSON request。

所以：

> **本次只需要把 RAM++ 也加载到这个长期存活的视觉 worker 生命周期里，或者做一个同样简单的常驻 RAM++ wrapper。**

优先方案见第 5 节。

---

## 1.3 SAM2 mask 已经真正进入三维定位链

当前：

```python
DetectedObject
```

已经有：

```python
mask_area_ratio
mask_rle
mask_size
```

`grounded_sam_worker.py` 已经输出：

```text
mask_rle
mask_size
refined bbox
```

---

## 1.4 `DepthObjectLocalizer` 已经支持 mask depth 和 3D Box

现有：

```text
app/perception/depth_object_localizer.py
```

已经：

- 解码 `mask_rle`；
- 从 SAM2 mask 内抽取 D435 depth；
- 过滤背景深度；
- 得到 mask foreground 3D points；
- 输出：
  ```text
  camera_xyz
  map_xyz
  box_center_camera
  box_size_camera
  box_center_map
  box_size_map
  ```
- 使用 p05/p95 等方式估计简单 3D extent。

所以：

> **不要重新写 SAM2→Depth→3D Box。**

---

## 1.5 GroundingDINO 已经支持 dynamic terms

现有：

```python
GroundedSAMSubprocessDetector.detect_with_dynamic_terms(
    image_path,
    target_text,
    dynamic_terms,
)
```

而 `_build_prompts()` 已经会合并：

```text
TargetProfile.detector_terms()
+
dynamic_terms
+
TargetProfile.context_labels_en
```

因此 RAM++ 最适合的接入方式就是：

```text
RAM++ tags
→ dynamic_terms
→ 现有 detect_with_dynamic_terms()
```

不需要新写 Grounding prompt pipeline。

---

## 1.6 TargetProfile 已经支持复杂目标

现有：

```text
app/reasoning/target_profile.py
```

已经有：

```text
primary_labels_en
aliases_en
attributes
colors
materials
affordances
relation_constraints
context_labels_en
negative_terms
possible_confusions
```

而：

```python
detector_terms()
```

已经会把：

```text
yellow + chair
```

组合成：

```text
yellow chair
```

同时仍保留：

```text
chair
```

因此：

> **“黄色椅子”无需新做任务解析器。**

---

## 1.7 CropVerifier 已经能检查属性与关系

现有：

```text
app/detectors/crop_verifier.py
```

已经把：

```text
目标描述
核心实体
英文词
属性
颜色
空间关系
混淆物
上下文
```

全部给 VLM。

输出已经有：

```text
target_match_score
category_score
attribute_score
context_score
observed_attributes
missing_required_attributes
```

所以：

> **VLM 复杂目标复核能力已经存在，本次只需要调整“什么时候调用”和“给它什么候选上下文”。**

---

## 1.8 AsyncSemanticObserver 已经会丢弃过期帧

现有：

```text
app/live_robot/async_semantic_observer.py
```

已经：

```text
inference 正忙
→ 新 frame 进来
→ 只保留最新 pending frame
```

不要新建 queue/scheduler。

---

# 2. 本次真正要解决的问题

现在系统有两个感知目标，但不能混为一条链。

---

## 2.1 目标 A：给三维地图尽可能多地贴普通物体标签

例如：

```text
chair
table
monitor
keyboard
plant
backpack
printer
trash can
lamp
cabinet
...
```

这件事要求：

```text
低成本
自动发现
不需要用户提前给 vocabulary
```

由：

```text
RAM++
→ GroundingDINO
→ SAM2
→ D435
→ SemanticObjectMap
```

负责。

---

## 2.2 目标 B：用户指定目标绝对不能只依赖 RAM++

例如：

```text
黄色椅子
挂着书包的椅子
带两个红色接口的设备盒
饮水机旁边的蓝色垃圾桶
```

这件事必须独立于 RAM++。

目标通路：

```text
用户任务
→ 现有 TargetProfileResolver
→ TargetProfile.detector_terms()
→ 强制送进 GroundingDINO
→ 候选 crop
→ VLM CropVerifier
```

因此：

> **RAM++ 漏掉目标类别，不代表目标搜索失败。**

---

# 3. 最终架构：双通路，不是单链

实现后必须是：

```text
                         用户任务
                            │
                            ▼
                 现有 TargetProfileResolver
                            │
              ┌─────────────┼─────────────┐
              │             │             │
          主类别词        属性词       上下文物体
          chair          yellow        backpack
              │             │             │
              └─────────────┬─────────────┘
                            │
                    mandatory target terms
                            │
                            │
RGB ─────────────► RAM++    │
                   │        │
                   ▼        │
            scene tags      │
                   │        │
                   └───┬────┘
                       ▼
             final Grounding terms
                       │
                       ▼
                GroundingDINO
                       │
                      boxes
                       │
                       ▼
                     SAM2
                       │
                     masks
                       │
                       ▼
                  D435 Depth
                       │
                       ▼
                3D objects / boxes
                       │
                       ▼
              SemanticObjectMap
                       │
                       ▼
                  3D Web Map

                      同时

             target-like candidates
                       │
                       ▼
               existing CropVerifier
                       │
                 attributes / relation
                       │
                       ▼
                    target?

如果长期没有 target candidate：
                       │
                       ▼
              low-frequency full-frame
                   VLM target sweep
```

---

# 4. 三类词必须明确分开

不要把所有 prompt term 混成同一种优先级。

---

## 4.1 `target_mandatory_terms`

来源：

```python
TargetProfile.detector_terms()
```

以及必要的：

```python
TargetProfile.context_labels_en
GroundingPromptPlan.direct_terms_en
GroundingPromptPlan.proxy_object_terms_en
GroundingPromptPlan.context_anchor_terms_en
```

这些词：

> **RAM++ 无论有没有输出，都必须进入 GroundingDINO。**

例如：

用户：

```text
找黄色椅子
```

mandatory：

```text
chair
yellow chair
seat
```

用户：

```text
找挂着书包的椅子
```

mandatory：

```text
chair
seat
backpack
bag
```

---

## 4.2 `ram_scene_terms`

来源：

```text
RAM++
```

用于建立丰富场景语义地图。

例如：

```text
monitor
keyboard
lamp
plant
picture frame
backpack
chair
table
printer
```

这些词：

- 可以被 RAM++ 漏掉；
- 不影响 target mandatory；
- 每隔一段时间更新。

---

## 4.3 `base_terms`

现有项目的固定 vocabulary 不必删除。

但不要把一大堆固定词永远塞进去。

只保留现有逻辑作为：

```text
RAM++ disabled / first frame before RAM tags ready
```

或少量高价值基础对象。

不要重新维护巨大 ontology。

---

# 5. RAM++ 的最小接入方式

推荐方案：

> **把 RAM++ 加到现有 `grounded_sam_worker.py --serve` 进程中，和 GroundingDINO/SAM2 共用一个长期 worker。**

原因：

- 当前 worker 已经是 persistent；
- 已有 stdin/stdout JSONL；
- 已经在 GPU perception 环境；
- 不需要增加第二个 daemon；
- 不需要新增 IPC；
- 最少代码。

---

## 5.1 新依赖

在现有 `full` profile 中增加：

```text
recognize-anything
RAM++ checkpoint
```

优先使用官方：

```text
xinyu1205/recognize-anything
```

模型：

```text
ram_plus_swin_large_14m.pth
```

不要训练。

---

## 5.2 bootstrap 修改

检查：

```text
scripts/bootstrap_fresh_machine.sh
scripts/verify_fresh_deployment.sh
```

现有 `--profile=full` 已经安装：

```text
Grounded-SAM-2
GroundingDINO
SAM2
```

在 **full profile** 里再增加：

```text
clone/install recognize-anything
download/check RAM++ checkpoint
```

不要影响：

```text
core
go2w
```

profile。

---

## 5.3 不把模型权重提交到 Git

只保存：

```text
路径配置
下载逻辑
校验逻辑
```

权重继续本地。

---

# 6. `grounded_sam_worker.py` 的具体修改

---

## 6.1 新增参数

建议只加：

```text
--ram-root
--ram-checkpoint
--disable-ram-plus
```

不要再加十几个参数。

RAM++ image size 使用：

```text
384
```

直接常量或一个现有配置字段即可。

---

## 6.2 worker 启动时加载 RAM++

伪代码：

```python
ram_model = None
ram_transform = None

if not args.disable_ram_plus:
    ram_model, ram_transform = _build_ram_plus(
        args.ram_root,
        args.ram_checkpoint,
        device,
    )
```

使用官方 API：

```python
from ram.models import ram_plus
from ram import inference_ram, get_transform

transform = get_transform(image_size=384)

model = ram_plus(
    pretrained=checkpoint,
    image_size=384,
    vit="swin_l",
)
model.eval()
model = model.to(device)
```

---

## 6.3 不要每帧重新加载

硬性测试：

```text
worker 生命周期：
RAM++ model init = 1
GroundingDINO init = 1
SAM2 init = 1
```

---

# 7. 扩展现有 JSONL worker protocol

不要建立新 protocol 文件。

当前 detect request：

```json
{
  "image": "...",
  "text_prompt": "...",
  "box_threshold": 0.12,
  "text_threshold": 0.10,
  "max_objects": 20
}
```

增加一个可选字段：

```json
{
  "operation": "detect"
}
```

默认缺省仍视为：

```text
detect
```

保持旧测试兼容。

---

## 7.1 RAM request

新增：

```json
{
  "operation": "tag",
  "image": "/path/frame.jpg"
}
```

返回：

```json
{
  "tags_en": [
    "chair",
    "table",
    "monitor",
    "backpack",
    "plant"
  ],
  "tags_zh": [
    "椅子",
    "桌子",
    "显示器",
    "书包",
    "植物"
  ]
}
```

不要返回 embedding。

不要存 feature vector。

---

# 8. RAM++ 输出必须做轻量过滤

RAM++ 会输出：

```text
chair
lamp
gray
green
living room
sit
wood floor
...
```

不是每个 tag 都应该进入 GroundingDINO。

---

## 8.1 不新增 NLP 模型

只做简单规则。

可以放在：

```text
app/detectors/vocabulary.py
```

新增：

```python
filter_ram_scene_terms(tags: list[str], max_terms: int) -> list[str]
```

---

## 8.2 小型 stoplist

删除明显不是独立物体的词：

```text
indoor
outdoor
room
home
background
image
photo

sit
stand
lay
hang

gray
green
yellow
black
white
red
blue

wood
metal
plastic
```

注意：

> 颜色不能从 TargetProfile 中删，只删除 **RAM scene tags 中单独出现的纯属性词**。

---

## 8.3 不要删除这些

即使不在现有 vocabulary，也保留：

```text
1~4 个英文单词
不是 stoplist
```

因为 RAM++ 的意义就是扩大开放物体覆盖。

例如：

```text
picture frame
wall lamp
fire cabinet
network switch
tool box
```

不要因为“不在 COMMON_OBJECT_VOCAB”就删除。

否则又退回封闭词表。

---

## 8.4 去重

统一：

```text
lowercase
strip
空白归一
完全字符串去重
```

即可。

不要做 embedding similarity。

---

# 9. `GroundedSAMSubprocessDetector` 增加一个方法

在：

```text
app/detectors/grounded_sam_subprocess.py
```

增加：

```python
def discover_scene_terms(self, image_path: str) -> list[str]:
    ...
```

逻辑：

```text
ensure worker
→ operation=tag
→ tags_en
→ filter_ram_scene_terms()
→ return
```

不要新增新的 detector class hierarchy。

---

# 10. RAM++ 不能每帧跑

RAM++ 的职责是：

> “当前视野里大概出现了哪些类别？”

不是 tracking。

第一版推荐：

```text
每 1.5~2.0 秒最多跑一次
```

即：

```text
0.5~0.7 Hz
```

如果实测 GPU 余量充足，可以调到：

```text
1 Hz
```

但不要每 camera frame 运行。

---

# 11. 缓存 RAM scene tags

不要创建 scheduler。

直接在现有搜索/感知 orchestrator 保存：

```python
self._cached_ram_terms: list[str] = []
self._last_ram_tag_time: float = 0.0
```

位置优先选择：

```text
app/live_robot/live_search_pipeline.py
```

如果实际 detector 调用发生在其他现有函数，就在那个最小作用域保存。

---

## 11.1 刷新逻辑

```python
if now - self._last_ram_tag_time >= ram_scene_tag_interval_seconds:
    self._cached_ram_terms = detector.discover_scene_terms(image_path)
    self._last_ram_tag_time = now
```

否则：

```text
复用 cached terms
```

不要建后台 RAM queue。

---

# 12. Grounding prompt 合并规则

现有 `GroundedSAMSubprocessDetector._build_prompts()` 已经会自动加入 TargetProfile。

因此调用时只需要：

```python
detector.detect_with_dynamic_terms(
    image_path,
    target_text,
    dynamic_terms=self._cached_ram_terms,
)
```

不要在外面再拼一次 `target_profile.detector_terms()`，避免重复。

---

## 12.1 目标词优先级必须高于 RAM tags

如果 `grounding_prompt_max_terms` 有上限：

排序必须：

```text
1. target direct terms
2. target context/proxy terms
3. RAM scene terms
4. static baseline terms
```

**绝不能因为 RAM++ 返回很多 tag，把用户目标词截掉。**

---

# 13. 修改 `GroundingPromptPlanner` 时只做最小调整

现有：

```text
app/perception/grounding_prompt_planner.py
```

已经负责任务相关 prompt。

如果其 `max_terms` 截断发生在 RAM terms merge 之前：

```text
不改。
```

如果 RAM terms merge 后总 prompt 会超过合理长度：

只在：

```text
grounded_sam_subprocess.py
```

最终 terms 组装处做：

```text
mandatory target terms 永不截断
RAM terms 截到 N 个
```

不要重写 `GroundingPromptPlanner`。

---

# 14. “黄色椅子”完整执行逻辑

必须通过该测试。

用户：

```text
找黄色椅子
```

现有 TargetProfile 应得到类似：

```text
primary = chair
colors = yellow
detector_terms:
  chair
  yellow chair
```

假设 RAM++ 当前只输出：

```text
table
monitor
plant
```

最终 GroundingDINO prompt 仍必须包含：

```text
chair
yellow chair
table
monitor
plant
```

GroundingDINO 检出：

```text
chair_1
chair_2
```

然后：

```text
CropVerifier
```

判断：

```text
chair_1:
attribute_score=0.2
is_target=false

chair_2:
attribute_score=0.95
is_target=true
```

RAM++ 是否知道“黄色椅子”不影响目标搜索。

---

# 15. “挂着书包的椅子”完整执行逻辑

必须通过该测试。

用户：

```text
找挂着书包的椅子
```

TargetProfile：

```text
base:
  chair

context:
  backpack

relation:
  backpack hanging on chair
```

最终 GroundingDINO 必须至少搜索：

```text
chair
backpack
```

---

## 15.1 不要求 GroundingDINO 判断 relation

GroundingDINO 只负责：

```text
chair bbox
backpack bbox
```

---

## 15.2 先做非常便宜的几何 context

对于每个 chair candidate：

检查同帧 backpack：

```text
bbox overlap
或
center distance
或
expanded chair bbox 是否包含 backpack center
```

只需简单 2D 几何。

---

## 15.3 VLM crop 必须包含关系上下文

当前 `crop_verify_expand_ratio` 已存在。

对于 relation target：

不要只裁 chair 紧边界。

使用：

```text
chair bbox 与附近 backpack bbox 的 union
```

再加少量 margin。

把 context 传给现有：

```python
CropVerifier.verify_crop(..., context=...)
```

context：

```json
{
  "nearby_objects": [
    {
      "label": "backpack",
      "bbox": [...]
    }
  ],
  "required_relations": [
    "backpack hanging on chair"
  ]
}
```

不要新建 relation reasoner。

---

# 16. 普通场景物体不要进 VLM

对于：

```text
chair
table
monitor
lamp
plant
```

只要不是当前 target candidate：

```text
RAM++
→ GroundingDINO
→ SAM2
→ Depth
→ SemanticObjectMap
```

结束。

不要调用 VLM。

---

# 17. 当前 full-scene VLM scene enumeration 要降级

现有：

```text
semantic_observer.py
```

逻辑中，如果 quick target 判断没有 object list：

```text
会 fall through 到 full-scene VLM
去列出所有可见物体
```

加入 RAM++ 后，Grounded-SAM 模式应改为：

```text
RAM++ + GroundingDINO 的 scene_objects
=
普通场景语义来源
```

因此：

> **Grounded-SAM + RAM++ 模式下，不再为了“给地图列普通物体”调用 full-scene VLM。**

---

# 18. 但是 VLM 不能彻底放到最后

为了防止：

```text
RAM++ 漏掉稀有目标
+
GroundingDINO 目标词仍没产生 candidate
```

保留一个：

> **低频 full-frame target sweep**

注意：

它不是：

```text
“请列出画面所有东西”
```

而是：

```text
“当前整张图里是否存在符合 TargetProfile 的目标，
包括稀有类别、颜色、属性、关系？
如果可能存在，指出候选区域和可见证据。”
```

---

# 19. 复用现有 VLM daemon，不新增服务

现有：

```text
siliconflow_vision_protocol.py
```

已经有：

```text
MODE_QUICK
MODE_SEMANTIC
MODE_VERIFY
```

本次不要再加一个 daemon。

优先复用：

```text
MODE_QUICK
```

配：

```text
target_profile.prompt_context()
+
额外 target-sweep 指令
```

如果现有 quick 模式无法返回可用候选区域：

才在同一个 protocol 中增加：

```text
MODE_TARGET_SWEEP
```

但优先避免。

---

# 20. VLM 漏检兜底触发条件

保持简单。

满足任一即可：

```text
1. 连续 4 次 GroundingDINO 都没有 target-like candidate
且
2. 距离上一次 full-frame target sweep >= 10 秒
```

或：

```text
TargetProfile.requires_scene_confirmation == True
且
当前是新的明显视角
```

第一版建议：

```yaml
vlm_target_sweep_interval_seconds: 10.0
vlm_target_sweep_empty_cycles: 4
```

只需两个参数。

---

# 21. VLM sweep 不应该阻塞普通语义地图

继续使用现有异步设施。

```text
RAM/GDINO/SAM2 scene mapping
继续

VLM target sweep
后台低频
```

如果当前 quick VLM 已经在实时链路中：

直接复用已有调用，不新建线程。

---

# 22. VLM sweep 发现疑似稀有目标后怎么办

VLM 不直接宣布任务完成。

它只产生：

```text
target_sweep_candidate
```

例如：

```json
{
  "is_present": true,
  "confidence": 0.72,
  "candidate_description": "桌面右侧黑色设备盒",
  "bbox_2d": [0.62, 0.31, 0.84, 0.68],
  "evidence": [
    "黑色矩形盒体",
    "可见红色接口"
  ]
}
```

然后：

```text
bbox
→ 现有 crop verification
或
→ SAM2 box prompt
→ D435 depth
→ SemanticObjectMap
```

---

## 22.1 如果 quick VLM 没有 bbox 输出

不要大改 protocol。

可先：

```text
VLM sweep 只返回“可能存在”
↓
触发当前帧 GroundingDINO high-recall retry
并把 target/profile terms 再搜索一次
```

若仍无 candidate：

保持：

```text
possible but unlocalized
```

不能伪造 3D object。

---

# 23. RAM++ 与 VLM 的职责边界

最终必须严格：

| 能力 | RAM++ | GroundingDINO | SAM2 | VLM |
|---|---:|---:|---:|---:|
| 自动发现常见场景类别 | ✅ 主力 | ❌ | ❌ | 低频兜底 |
| 找 bbox | ❌ | ✅ | ❌ | 只做稀有候选 |
| 精确 mask | ❌ | ❌ | ✅ | ❌ |
| 普通 3D 地图标签 | 提供词 | ✅ | ✅ | ❌ |
| 判断黄色/材质/状态 | 不作为最终判断 | 候选级 | ❌ | ✅ |
| 判断“挂在/旁边/属于” | ❌ | 只找组成物体 | ❌ | ✅ |
| 稀有目标漏检兜底 | ❌ | 目标词强制搜索 | ❌ | ✅ |
| 最终目标确认 | ❌ | ❌ | ❌ | ✅ |

---

# 24. SemanticObjectMap 不重写

当前现有：

```text
app/spatial/semantic_object_map.py
```

已经有：

- persistent object IDs；
- observation_count；
- one-to-one association；
- reproject observations；
- multi-view merge。

本次：

> **只验证 RAM++ 增加大量 scene objects 后，去重是否仍然正常。**

如果正常：

```text
不改。
```

只有确实出现：

```text
chair_001
chair_002
chair_003
都是同一张椅子
```

且 map_xyz 本身稳定时：

才小幅调整现有 association threshold。

---

# 25. RAM++ tag 不能直接变成地图 object

关键原则：

```text
RAM++ tag = “画面里可能有这个类别”
```

不是：

```text
SemanticObject
```

必须经过：

```text
RAM tag
→ GroundingDINO bbox
→ SAM2 mask
→ D435 depth
→ valid 3D localization
→ SemanticObjectMap
```

RAM++ 输出：

```text
monitor
```

但 GroundingDINO 找不到 monitor bbox：

```text
不创建 monitor object。
```

---

# 26. 普通物体标签质量

GroundingDINO 返回：

```text
source_prompt_term
```

继续使用。

对于 RAM++ term：

```text
label = GroundingDINO matched label
source_prompt_term = RAM scene term
```

不要让 RAM++ 中文标签直接覆盖 map canonical ID。

---

# 27. 不做复杂 label normalization

只用现有：

```text
label_zh()
category_for_label()
COMMON_OBJECT_VOCAB
```

以及 lowercase cleanup。

不要新增：

- WordNet；
- sentence embedding；
- LLM label merge；
- ontology graph。

---

# 28. 配置新增项：严格控制数量

在：

```text
app/config.py
.env.example
```

只增加必要字段。

建议：

```text
ENABLE_RAM_PLUS=true
RAM_PLUS_ROOT=<recognize-anything path>
RAM_PLUS_CHECKPOINT=<ram_plus_swin_large_14m.pth>
RAM_SCENE_TAG_INTERVAL_SECONDS=2.0
RAM_SCENE_TAG_MAX_TERMS=24

VLM_TARGET_SWEEP_INTERVAL_SECONDS=10.0
VLM_TARGET_SWEEP_EMPTY_CYCLES=4
```

共 7 个。

不要再做几十个 threshold。

RAM++ 使用官方默认 tagging threshold。

---

# 29. `app/config.py` 具体修改

增加 defaults：

```python
DEFAULT_ENABLE_RAM_PLUS = True
DEFAULT_RAM_PLUS_ROOT = ...
DEFAULT_RAM_PLUS_CHECKPOINT = ...
DEFAULT_RAM_SCENE_TAG_INTERVAL_SECONDS = 2.0
DEFAULT_RAM_SCENE_TAG_MAX_TERMS = 24

DEFAULT_VLM_TARGET_SWEEP_INTERVAL_SECONDS = 10.0
DEFAULT_VLM_TARGET_SWEEP_EMPTY_CYCLES = 4
```

加入 `Settings`。

加入 env 解析。

不要新增独立 config class。

---

# 30. `grounded_sam_worker.py` 修改清单

必须：

- [ ] 新增 RAM++ args；
- [ ] startup 加载 RAM++；
- [ ] serve 支持 `operation=tag`；
- [ ] 旧 detect request 不受影响；
- [ ] RAM inference 返回英文/中文 tag；
- [ ] 模型只加载一次；
- [ ] 不改变现有 GroundingDINO/SAM2 detect 结果 schema。

不要动：

```text
mask_rle
SAM2 refine box
NMS
现有 detection threshold
```

除非测试证明有冲突。

---

# 31. `grounded_sam_subprocess.py` 修改清单

必须：

- [ ] `discover_scene_terms(image_path)`；
- [ ] 使用同一个 `_worker`；
- [ ] operation=tag；
- [ ] filter RAM terms；
- [ ] 继续 `detect_with_dynamic_terms()`；
- [ ] target profile 的词仍然自动进入 prompt；
- [ ] target terms 不被 RAM terms 挤掉。

不要：

- [ ] 新启 RAM subprocess；
- [ ] 新增第二个 worker 管理器。

---

# 32. `vocabulary.py` 修改清单

只加：

```python
RAM_SCENE_TAG_STOPLIST = {...}

def filter_ram_scene_terms(
    terms: list[str],
    max_terms: int = 24,
) -> list[str]:
    ...
```

规则：

```text
normalize
drop empty
drop pure attribute/action/scene terms
keep unknown object nouns/phrases
dedupe
truncate
```

不要做 POS tagging。

---

# 33. `live_search_pipeline.py` 修改清单

找到当前 Grounded-SAM inference 入口。

增加：

```python
_cached_ram_terms
_last_ram_tag_time
_target_empty_candidate_cycles
_last_vlm_target_sweep_time
```

这是足够的全部状态。

不要新建 class。

---

## 33.1 每个语义 inference cycle

伪代码：

```python
if ram_plus_enabled and ram_refresh_due:
    self._cached_ram_terms = detector.discover_scene_terms(image_path)
    self._last_ram_tag_time = now

detections = detector.detect_with_dynamic_terms(
    image_path,
    target_text,
    dynamic_terms=self._cached_ram_terms,
)
```

后面继续走现有：

```text
SAM2
DepthObjectLocalizer
Semantic graph
SemanticObjectMap
```

---

# 34. candidate classification

当前检测结果需要分：

```text
scene object
target-like object
context object
```

不要新增新 schema。

可以通过现有：

```text
label
source_prompt_term
TargetProfile.direct_terms()
TargetProfile.context_terms()
```

判断。

---

## 34.1 普通 scene object

直接进入地图。

---

## 34.2 direct target candidate

进入：

```text
CropVerifier
```

---

## 34.3 context object

例如：

```text
backpack
```

对于：

```text
挂着书包的椅子
```

它既：

```text
进入 SemanticObjectMap
```

又：

```text
提供给 chair candidate 的 VLM context
```

---

# 35. relation candidate 的简单几何组装

只需要新增一个小 helper。

推荐放：

```text
live_search_pipeline.py
```

而不是新建 relation module。

例如：

```python
def nearby_context_objects(
    candidate,
    detections,
    target_profile,
) -> list[DetectedObject]:
    ...
```

判断：

```text
same frame
label matches context terms
expanded bbox overlap / center near
```

---

# 36. CropVerifier 修改原则

优先不改它的 JSON schema。

现有已经能判断：

```text
attribute_score
context_score
```

可能只需：

- 给 relation target 扩大 crop；
- 把 nearby objects 放入 context；
- prompt 增加一句：
  ```text
  若目标包含关系约束，必须同时看到主体和关联物体证据，
  不能只看到其中一个就判定 true。
  ```

这是小改。

---

# 37. VLM target sweep 的实现位置

优先复用：

```text
app/detectors/siliconflow_vision_protocol.py
app/detectors/siliconflow_vision_worker.py
```

现有：

```text
MODE_QUICK
```

已经是低 token target 判断。

先尝试：

```text
MODE_QUICK
+
extra_instructions
```

完成。

不要先创建 `MODE_TARGET_SWEEP`。

---

# 38. target sweep prompt

可以直接使用：

```text
TargetProfile.prompt_context()
```

再追加：

```text
这是漏检兜底检查，不需要列出全部场景物体。
只回答当前整图是否存在符合目标画像的可见候选。
尤其检查 GroundingDINO 可能漏掉的稀有类别、颜色、部件、状态和物体关系。
不确定则 is_present=false 或降低 confidence。
```

---

# 39. 什么时候触发 VLM sweep

在 `live_search_pipeline.py` 使用：

```python
if not target_like_candidates:
    self._target_empty_candidate_cycles += 1
else:
    self._target_empty_candidate_cycles = 0
```

当：

```python
empty_cycles >= configured_limit
and now - last_sweep >= configured_interval
```

触发一次。

---

# 40. VLM sweep 命中后的动作

优先使用现有 quick payload 的：

```text
matched_indices
scene_objects
target_decision
```

如果 payload 已有 bbox object：

```text
把 bbox 作为 VLM-generated target candidate
→ crop verifier / depth localization
```

如果没有 bbox：

```text
只触发 GroundingDINO high-recall target retry
```

不要把没有空间定位的 VLM 文本直接写成 3D map object。

---

# 41. `semantic_observer.py` 修改

当前 quick target absence 没 object list 时会：

```text
fall through
→ full scene analysis
→ VLM 枚举所有可见 object
```

在：

```text
Grounded-SAM + RAM++ enabled
```

时改成：

> **不要因为 scene object list 为空而自动让 VLM 全场景枚举。**

普通 object map 的职责已转给：

```text
RAM++ → GroundingDINO → SAM2
```

VLM 只走 target sweep。

---

# 42. 语义地图 coverage 和目标 recall 必须分开测试

这次不能只看“识别多少 object”。

必须两个指标：

```text
scene semantic coverage
target recall
```

---

# 43. 测试数据集

准备最少 6 类静态图片/录制帧。

可以放：

```text
tests/fixtures/ram_semantic/
```

如果仓库不适合提交图片，就用本地 `data/eval` 并写脚本。

---

## Scene A：普通办公室

真实有：

```text
chair
table
monitor
keyboard
mouse
plant
backpack
cup
```

目标：

```text
RAM++ 至少给出大部分物体词
GroundingDINO 能定位主要实例
```

---

## Scene B：RAM++ 属性混杂

RAM++ 可能输出：

```text
chair
gray
green
room
sit
wood floor
lamp
```

过滤后：

```text
chair
lamp
...
```

属性/动作不能污染 DINO prompt 太严重。

---

## Scene C：黄色椅子

RAM++ 故意 mock 为：

```text
table
monitor
plant
```

**不包含 chair。**

TargetProfile：

```text
yellow chair
```

要求：

```text
最终 Grounding prompt 仍包含 chair / yellow chair
```

并能走到 VLM attribute verify。

---

## Scene D：挂书包的椅子

图中：

```text
chair A：没有书包
chair B：挂着 backpack
```

要求：

```text
GroundingDINO 找到 chair + backpack
VLM 最终只确认 chair B
```

---

## Scene E：稀有设备

RAM++ mock 不包含目标。

GroundingDINO 第一轮也无 candidate。

连续空 candidate 达阈值：

```text
VLM full-frame target sweep 触发
```

必须证明 VLM 旁路存在。

---

## Scene F：普通搜索，无目标

持续 30~60 秒。

要求：

```text
大量普通 objects 进入地图
VLM target sweep 调用低频
VLM 不每帧调用
```

---

# 44. 单元测试清单

---

## 44.1 RAM tag filtering

新增：

```text
tests/test_ram_scene_terms.py
```

输入：

```text
armchair
gray
green
living room
sit
wood floor
picture frame
lamp
backpack
```

要求：

```text
保留：
armchair
picture frame
lamp
backpack
wood floor

删除：
gray
green
room
sit
```

不要要求 NLP 完美。

> **2026-09-16 勘误：本节原文把 `living room` 列进「删除」，实际做不到，而且是有意为之。**
> `filter_ram_scene_terms`（`app/detectors/vocabulary.py:254`）判停用词是
> `normalized in RAM_SCENE_STOPLIST` —— **整条 tag 精确相等**，所以 stoplist 里出现
> 多词条目会**静默失效**；`tests/test_ram_scene_terms.py:73-75`
> （`test_stoplist_has_no_multi_word_entries`）正是把这条约束钉住的测试。
> 而多词短语往往是真实的开放物体（`picture frame` / `glass door` / `wall lamp`），
> 按子串删除会误伤 RAM++ 的开放词表覆盖 —— 那是装 RAM++ 的全部意义。
> **因此停用词只收单词：`room` 在表里，`living room` 不在、也不该在。**
> 上面两个多词项已按实际行为改正：`wood floor` 的实际结果是**保留**
> （`tests/test_ram_scene_terms.py:27` 就是这么断言的）。
> **代码与测试一致，不一致的只有本节文字；本次改的是文字，代码零改动。**

---

## 44.2 RAM worker operation

扩展：

```text
tests/test_grounded_sam_persistent_worker.py
```

或新增：

```text
tests/test_grounded_sam_ram_operation.py
```

fake worker 连续：

```text
tag
detect
tag
detect
```

要求：

```text
Popen 仍然只有 1 个
```

---

## 44.3 Target mandatory terms independent of RAM

新增：

```text
tests/test_grounding_prompt_ram_target_merge.py
```

RAM：

```text
table
monitor
plant
```

目标：

```text
黄色椅子
```

最终 prompt 必须含：

```text
chair
yellow chair
```

---

## 44.4 RAM terms 被截断不能影响 target

RAM 产生 100 个 terms。

`max_terms` 限制后：

目标词仍全部存在。

---

## 44.5 Yellow chair VLM verify

mock：

```text
2 chair candidates
```

VLM：

```text
chair A attribute false
chair B attribute true
```

最终只 B 为目标。

---

## 44.6 Relation target

mock：

```text
chair A
chair B
backpack near chair B
```

要求传给 chair B verifier 的 context 包含：

```text
backpack
required relation
```

chair A 不应该有该 context。

> **2026-09-16 勘误：这一条不可满足，而且是有意为之。**
> `_verification_region`（`app/video/semantic_verifier.py:151-225`）**没有距离门**：
> 它只排除候选自己（`:183`），然后在整帧里挑**一个** anchor 返回 ——
> `:211` `min(matches, key=lambda item: (item[0], item[1]))`，先按「是不是
> relation anchor」再按距离。所以帧里只要存在 anchor，**每一个**候选都会被挂上离
> 它最近的那个 ⇒ chair A 也会拿到 `backpack` context。
> 设计意图写在函数 docstring：「*Include the nearest explicit relation anchor
> without weakening the gate*」：几何层只保证**证据落在裁剪框内**，
> 「书包是否真在**这把**椅子上」这个空间判断**交给 VLM**。这是分工，不是遗漏。
> 所以本节的测试断言只能是「**chair B 的 context 含 backpack**」（可满足），
> 不能是「**chair A 不含 backpack**」（不可满足，别去写它）。
> **本次改的是文字，代码零改动。**

---

## 44.7 VLM target sweep trigger

连续：

```text
4 cycles no target candidate
```

时间达到 interval：

```text
VLM quick target sweep call = 1
```

下一周期：

```text
不能马上再次 call
```

---

## 44.8 有 candidate 时不 sweep

只要正常 Grounding target candidate 存在：

```text
empty counter reset
```

---

## 44.9 RAM ordinary objects 不进 VLM

RAM+DINO 返回：

```text
chair
table
monitor
plant
```

目标：

```text
fire extinguisher
```

VLM crop verify：

```text
0
```

普通 object 仍然进入 semantic map。

---

# 45. 必须跑的现有测试

不要猜文件名，先：

```bash
find tests -type f | grep -E \
'grounded|sam|depth|semantic|target_profile|crop|vision|live_search|prompt'
```

至少覆盖：

```text
GroundedSAMSubprocessDetector
grounded_sam_worker
DepthObjectLocalizer
TargetProfile
GroundingPromptPlanner
CropVerifier
SemanticObjectMap
AsyncSemanticObserver
LiveSemanticObserver
LiveSearchPipeline
VLM protocol
```

最后：

```bash
python -m pytest -q
```

---

# 46. 真实模型离线验证

不能只 mock。

准备 20~30 张室内图片。

每张人工记录主要物体。

运行：

```text
RAM++
→ DINO
→ SAM2
```

输出：

```text
ram_tags.json
detections.json
```

统计：

```text
RAM tag recall
DINO localized-object recall
```

---

# 47. 场景语义覆盖验收

不要要求“识别所有东西”。

MVP 目标：

对普通室内明显可见、尺寸合理的主要物体：

```text
scene object category recall >= 70%
```

建议按图片级：

```text
人工标 8 个主要物体
识别/定位 >= 6 个
```

重点是比固定 vocabulary 更丰富。

---

# 48. Target recall 验收必须更高

因为有：

```text
TargetProfile mandatory terms
+
GroundingDINO
+
VLM sweep
```

对测试目标集：

```text
黄色椅子
挂书包的椅子
蓝色垃圾桶
灭火器
打印机
特殊设备盒
```

目标：

```text
target candidate recall >= 90%
```

这里：

```text
candidate recall
```

不是最终确认准确率。

---

# 49. 属性/关系最终确认验收

对复杂目标：

```text
黄色椅子
挂书包的椅子
桌上的红色水杯
饮水机旁边垃圾桶
```

至少设计正负样本。

要求：

```text
VLM 不应只因为 base category 正确就确认。
```

例如：

```text
蓝椅子 ≠ 黄色椅子
普通椅子 ≠ 挂书包椅子
```

---

# 50. VLM 调用效率验收

运行 60 秒正常场景。

统计：

```text
semantic frames
RAM calls
GroundingDINO calls
crop verify calls
full-frame VLM target sweep calls
full-scene enumeration calls
```

目标：

```text
full-scene VLM enumeration calls = 0
```

普通无目标情况下：

```text
VLM full-frame target sweep
<= 6 次 / 60 秒
```

因为 interval 默认 10 秒。

如果普通 scene object 没有目标属性需求：

```text
crop verify = 0
```

---

# 51. RAM++ 调用频率验收

默认：

```text
RAM interval = 2 sec
```

60 秒：

理论最多：

```text
约 30 次
```

不应该等于 camera frame count。

---

# 52. 3D Semantic Map 验收

RAM++ 新发现的普通对象：

```text
lamp
picture frame
backpack
printer
```

经过：

```text
DINO + SAM2 + D435
```

后：

必须能够：

```text
进入 SemanticObjectMap
获得 map_xyz
持续 observation merge
```

---

# 53. 同一物体不能因为 RAM tags 刷新而重复创建

同一 chair：

```text
RAM frame1: chair
RAM frame2: armchair
RAM frame3: chair
```

如果现有 `SemanticObjectMap` 能正常 merge：

保持。

若由于 label synonym 导致：

```text
chair_001
armchair_001
```

不要引入 embedding。

只在：

```text
vocabulary.py
```

增加极少数已知 canonical aliases：

```text
armchair -> chair
cell phone -> phone
television -> tv
```

只对实测重复进行补充。

禁止做通用 ontology。

---

# 54. 性能验收

RAM++ 会增加 GPU 负载，所以必须测。

记录：

```text
RAM inference median ms
GroundingDINO median ms
SAM2 median ms
overall semantic cycle ms
GPU memory before/after RAM load
```

最低要求：

```text
机器狗导航主循环不被阻塞
AsyncSemanticObserver 仍正常 coalesce
```

场景语义不要求实时 30 FPS。

建议目标：

```text
RAM 0.5 Hz
Grounded-SAM semantic updates 1~3 Hz
```

---

# 55. 显存要求的处理原则

不要写复杂自动显存降级。

如果 RAM++ + GroundingDINO + SAM2 同时加载确实 OOM：

Coding Agent：

1. 记录实际 GPU / VRAM；
2. 先确认没有重复模型实例；
3. 确认一个 session 只有一个 worker；
4. 若确实装不下，再将 RAM++ 放到**一个单独常驻进程**。

只有发生真实 OOM 才允许做这个调整。

不要预先做双架构。

---

# 56. 如果必须分 RAM worker

仅 OOM/依赖冲突时：

最多新增：

```text
app/detectors/ram_plus_worker.py
```

使用同样：

```text
stdin/stdout JSONL
persistent Popen
```

不创建网络服务。

但默认计划仍是：

```text
same Grounded-SAM worker
```

---

# 57. 部署位置

RAM++ 应跟：

```text
GroundingDINO
SAM2
```

运行在同一台具备 NVIDIA GPU 的现有视觉计算环境。

不要放到算力弱的机器狗 Python 环境，除非当前 Grounded-SAM 本来就在那里且实测满足性能。

---

# 58. README / 文档改动

只更新必要部分。

说明新的视觉职责：

```text
RAM++:
自动场景 tags

GroundingDINO:
开放词汇定位

SAM2:
实例 mask

D435:
3D localization

VLM:
复杂属性/关系与漏检兜底
```

以及：

```text
RAM++ 不决定任务目标是否存在
```

这句话必须写清楚。

---

# 59. 最终文件修改表

| 文件 | 修改内容 |
|---|---|
| `app/detectors/grounded_sam_worker.py` | 加载 RAM++；JSONL `operation=tag`；保持 detect 不变 |
| `app/detectors/grounded_sam_subprocess.py` | 新增 `discover_scene_terms()`；RAM tags 作为 dynamic terms |
| `app/detectors/vocabulary.py` | RAM tag 轻量过滤、少量 canonical aliases |
| `app/config.py` | 增加 RAM 与 VLM sweep 的少量配置 |
| `.env.example` | 增加对应配置示例 |
| `app/live_robot/live_search_pipeline.py` | 缓存 RAM tags、刷新间隔、empty candidate counter、低频 VLM sweep |
| `app/live_robot/semantic_observer.py` | RAM 模式不再靠 full-scene VLM 枚举普通 object |
| `app/detectors/crop_verifier.py` | 关系目标 prompt/context 小幅增强，如现有能力足够则不改 |
| `scripts/bootstrap_fresh_machine.sh` | full profile 安装 RAM++ repo/依赖/权重 |
| `scripts/verify_fresh_deployment.sh` | full profile 检查 RAM++ 安装和 checkpoint |
| README / 对应部署文档 | 简要说明视觉双通路 |
| tests | 增加本文列出的 targeted tests |

---

# 60. 明确不应该修改的核心模块

除非因为字段传递缺失做极小修复，否则不要动：

```text
app/perception/depth_object_localizer.py
app/spatial/semantic_object_map.py
app/slam/slam_bridge.py
app/spatial/slam_web_map_state.py
plain_slam_ros2/
go2w_plain_slam_bridge/
navigation/
```

原因：

这些能力当前已经存在并且不是本次问题源头。

---

# 61. 实施顺序

Coding Agent 必须按以下顺序。

### Phase 0：Baseline

```bash
git status
git diff --stat
pytest targeted existing tests
```

---

### Phase 1：RAM++ runtime

完成：

```text
bootstrap
worker load
operation=tag
discover_scene_terms
tag filter
```

先测试 RAM 单独工作。

---

### Phase 2：RAM → GroundingDINO

把：

```text
cached RAM scene terms
```

接到现有：

```python
detect_with_dynamic_terms()
```

验证 target mandatory 不会消失。

---

### Phase 3：普通场景 Semantic Map

验证：

```text
RAM tag
→ DINO bbox
→ SAM2 mask
→ existing DepthObjectLocalizer
→ existing SemanticObjectMap
```

完整通过。

---

### Phase 4：复杂目标 VLM verify

测试：

```text
黄色椅子
挂书包的椅子
```

---

### Phase 5：VLM target sweep

加低频漏检旁路。

---

### Phase 6：去掉 Grounded-SAM 模式下普通 full-scene VLM 枚举

确保地图仍由 RAM pipeline 构建。

---

### Phase 7：完整回归

```bash
python -m compileall app tests
python -m pytest -q
```

---

# 62. Coding Agent 自检逻辑

出现问题时只能修对应层。

```text
RAM 漏普通物体
→ 看 RAM tags / filtering

RAM 有 tag 但无 bbox
→ 看 GroundingDINO

有 bbox 但 3D 不对
→ 看现有 SAM2 mask / Depth / timestamp

3D 正确但重复 ID
→ 看现有 SemanticObjectMap association

黄色椅子误判
→ 看 CropVerifier attribute

书包关系误判
→ 看 relation crop/context

稀有目标全漏
→ 看 target mandatory terms + VLM sweep
```

禁止一个问题就重构整个 pipeline。

---

# 63. 最终验证报告

Coding Agent 完成后生成：

```text
reports/RAM_PLUS_SEMANTIC_PERCEPTION_VALIDATION.md
```

必须写真实结果。

格式：

```text
Baseline branch:
Baseline HEAD:
Local dirty files preserved:

RAM++:
  model loaded:
  worker load count:
  median inference ms:

Scene dataset:
  annotated major categories:
  RAM discovered:
  DINO localized:
  scene category recall:

Target tests:
  yellow chair:
  backpack-on-chair:
  rare object:
  target candidate recall:

VLM:
  60s frames:
  crop verify calls:
  full-frame sweep calls:
  full-scene enumeration calls:

Semantic map:
  persistent objects:
  duplicate objects:
  same-class separate objects test:

Regression:
  pytest passed:
  pytest failed:
  compileall:

FINAL:
  PASS / FAIL
```

---

# 64. 硬验收标准

全部满足才能写 PASS。

## RAM++

- [ ] 预训练 RAM++ 成功加载
- [ ] 一个 worker 生命周期只加载一次
- [ ] 每约 2 秒最多刷新一次 scene tags
- [ ] RAM tag filter 不只允许旧 vocabulary
- [ ] 普通 scene object coverage 相比固定词表明显增加

## GroundingDINO + SAM2

- [ ] RAM scene terms 进入 dynamic terms
- [ ] target mandatory terms 无论 RAM 是否输出都保留
- [ ] SAM2 现有 mask pipeline 未被破坏
- [ ] D435 现有 3D localization 未被破坏

## 复杂目标

- [ ] RAM 不含 chair 时，“黄色椅子”仍能搜索 chair
- [ ] 颜色由 VLM 最终复核
- [ ] “挂着书包的椅子”同时搜索 chair + backpack
- [ ] relation target 的 crop/context 包含关联物体
- [ ] 只看到普通 chair 不能确认 relation target

## 稀有目标

- [ ] RAM++ 漏词不会直接宣告目标不存在
- [ ] target profile 强制词独立运行
- [ ] 连续无 candidate 后低频 VLM target sweep 会触发
- [ ] VLM sweep 不会每帧调用
- [ ] 无 bbox/空间证据时不能伪造 3D object

## 场景地图

- [ ] 普通物体无需 VLM 即可进入 3D SemanticObjectMap
- [ ] RAM++ tag 自己不能直接创建 3D object
- [ ] 同一物体多帧仍复用现有 persistent object
- [ ] 三维地图/SLAM 行为没有被修改

## 效率

- [ ] full-scene VLM ordinary object enumeration = 0
- [ ] VLM 只用于 candidate verify / low-frequency target sweep
- [ ] 60 秒内 full-frame VLM sweep 不超过 interval 所允许次数
- [ ] AsyncSemanticObserver 主链不阻塞导航

## 回归

- [ ]现有 Grounded-SAM tests 通过
- [ ]现有 SAM2/depth tests 通过
- [ ] TargetProfile tests 通过
- [ ] CropVerifier tests 通过
- [ ] SemanticObjectMap tests 通过
- [ ] LiveSearch tests 通过
- [ ] SLAM/map tests 通过
- [ ] `python -m pytest -q` 无本次引入的新失败

---

# 65. 给 Coding Agent 的最终执行提示词

把下面文本和本计划书一起交给 Coding Agent：

```text
你正在修改 BROVVV/robot_scene_demo 的
fix/go2w-autonomous-search-lio-topology-target 分支。

严格按《GO2W RAM++ + GroundingDINO + SAM2 + VLM 双通路语义感知详细实施计划》执行。

重要：当前三维建图已经稳定，本次禁止换 SLAM、禁止改 plain_slam、禁止引入 LIO-SAM/FAST-LIO、禁止重写三维地图。

先完整阅读当前代码再修改。当前分支已经具备：
- Grounded-SAM persistent Popen worker；
- grounded_sam_worker.py --serve；
- GroundingDINO/SAM2 一次加载；
- DetectedObject.mask_rle/mask_size；
- DepthObjectLocalizer 的 SAM2 mask depth 与 3D box；
- TargetProfile 的颜色/属性/关系/context；
- GroundingPromptPlanner；
- CropVerifier；
- AsyncSemanticObserver；
- SemanticObjectMap 的 persistent ID / association / reprojection。

因此绝对不要重复实现这些东西。

本次真正要做的是：
1. 在现有 Grounded-SAM persistent worker 中加载 RAM++；
2. 增加 JSONL operation=tag；
3. 新增 discover_scene_terms(image_path)；
4. RAM++ 每约 2 秒自动发现场景 tags；
5. RAM tags 通过现有 detect_with_dynamic_terms 进入 GroundingDINO；
6. TargetProfile.detector_terms/context terms 永远优先，不能依赖 RAM++；
7. 普通物体由 RAM++→GroundingDINO→SAM2→Depth→SemanticObjectMap 建图；
8. VLM 不再每帧列出所有物体；
9. 复杂属性/关系目标继续由 CropVerifier 判断；
10. 连续多次没有 target candidate 时，每约 10 秒最多做一次 full-frame VLM target-only sweep，用于稀有目标漏检兜底。

关键例子必须通过：
A. RAM++ 没输出 chair，但任务是“黄色椅子”：
   Grounding prompt 仍必须有 chair/yellow chair，最后 VLM 判断颜色。
B. “挂着书包的椅子”：
   DINO 搜 chair + backpack，VLM 必须看包含二者的 context 后才确认关系。
C. 稀有设备：
   RAM++ 和第一轮 DINO 都没找到时，低频 full-frame VLM target sweep 必须触发，但不能每帧调用。
D. 普通 chair/table/monitor/plant：
   不得调用 full-scene VLM，直接进入现有 SemanticObjectMap。

禁止过度编程：
- 不建 gRPC/HTTP 服务；
- 不建第二套 semantic map；
- 不建数据库；
- 不建 worker pool；
- 不建复杂 scheduler；
- 不引入 embedding ontology；
- 不重写现有 association；
- 不增加复杂 fallback；
- 不修改导航/SLAM。

修改前记录 git status/diff，保留用户本地未提交代码。

修改后必须：
- 跑新增 targeted tests；
- 跑现有相关 tests；
- python -m compileall app tests；
- python -m pytest -q；
- 做真实 RAM++ 室内图片测试；
- 测黄色椅子、挂书包椅子、稀有物体旁路；
- 输出 reports/RAM_PLUS_SEMANTIC_PERCEPTION_VALIDATION.md。

有测试失败就修对应层并重跑，不要靠扩展架构解决局部问题。
```

---

# 66. 最终成功后的行为

正常场景：

```text
机器狗移动
↓
稳定 SLAM 正常建 3D 地图（完全不改）
↓
RGB 最新帧
↓
RAM++ 低频：
chair / table / monitor / plant / backpack
↓
GroundingDINO：
给这些物体定位 bbox
同时强制搜索用户目标词
↓
SAM2：
mask
↓
D435：
3D box
↓
SemanticObjectMap：
多帧融合 / 去重
↓
3D map：
显示普通物体语义
```

复杂任务：

```text
“找挂着黑色书包的黄色椅子”
↓
TargetProfile:
chair + yellow + backpack + hanging relation
↓
GroundingDINO:
chair + backpack
↓
几何找相邻候选
↓
VLM CropVerifier:
黄色？
黑色书包？
真的是挂在椅子上？
↓
确认 / 拒绝
```

稀有目标：

```text
RAM++ 没发现
↓
没关系
↓
TargetProfile 强制 GroundingDINO 搜索
↓
仍连续无 candidate
↓
10 秒级低频 full-frame VLM target sweep
↓
发现疑似区域
↓
重新 localization / crop verification
```

这就是本次改造的最终目标：

> **RAM++ 负责扩大日常场景覆盖；GroundingDINO/SAM2 负责快速定位分割；现有 Depth/SLAM/SemanticObjectMap 负责三维世界落点；VLM 不做每帧主检测，但仍保留复杂语义判断和稀有目标漏检兜底。**

---

# 5. 执行交接：2026-09-11 这一轮干完之后的状态

> §1 是**接手前**的状态，本节是**本轮（A0–A11）执行完**之后的状态。
> 完整证据在 `reports/RAM_PLUS_SEMANTIC_PERCEPTION_VALIDATION.md`（下称"验证报告"）——
> 每条结论都带文件行号或日志路径。本节只写**接手要拿主意的地方**。

## 5.1 一句话

**不需要机器狗的部分（A0–A11）已全部执行完**：RAM++ 双通路在离线全链路和
真模型（RAM++ / GroundingDINO / SAM2 / SiliconFlow VLM）上验证通过，
A7（§61 Phase 4 复杂目标复核）连跑四次结论一致。期间修掉 6 个问题，
其中 2 个会让真机链路**直接崩**。需要机器狗的部分只做完 B1/B2，
**B3/B4 仍等一次新的运动授权**。

## 5.2 本轮修掉的问题（**不要回退**）

| # | 问题 | 不修的后果 |
|---|---|---|
| 1 | `_build_prompts` 收到 `ram_scene_terms=None` 崩 | RAM++ 关掉时整条检测链崩 |
| 2 | `_verification_region` 把同帧物体全跳过（两边 `object_id` 都是 `None`，`None == None` 成立） | 关系裁剪永远不生效 |
| 3 | 目标画像的 context 词被 `STATIC_OBJECT_PROMPTS_ENABLED=false` 挡掉 | 复杂目标的上下文词进不了 DINO |
| 4 | 关系锚点被上游 LLM 时给时不给（实测 3 次错 2 次） | §15 的裁剪框拿不到锚点 |
| 5 | **两份 `TargetProfile` 分叉** → 真机 `AttributeError` | **真机上任何 relation 目标必崩** |
| 6 | §15 判据「框变大了就算过」 | 并进一扇玻璃门也算通过（假通过） |

第 5 条最要命：`scripts/go2w/run_autonomous_loop.py:2036` 用 `app.reasoning`
的 resolver 建画像，`:2170` 却把画像交给 `app.video` 的 verifier 裁图，
而两个模块各有一份**逐字节相同**的 `TargetProfile`。本轮只补字段、**没有合并**
（合并是重构），但加了跨模块回归测试
`tests/test_video_semantic_verifier.py::VideoSemanticVerifierTests::test_reasoning_profile_drives_the_video_crop_region`
锁住这个组合。再动这两份文件之前先跑它。

⚠️ **它锁的范围比看起来窄**：用例构造的是 `app.reasoning` 那份、传给 `app.video`
的函数，而那个函数只读传入对象的属性（类型标注 Python 运行时不校验）——
所以它只能在**分叉发生在 reasoning 侧**时报错，`app/video` 那份自己少字段它抓不到。
别把它当合并的替代品。详见验证报告「诚实记录」第 9 条。

## 5.3 磁盘写满事故（本轮新增处理，与 RAM++ 无关）

建图时 `slam_3d_node` 每加一个关键帧就往 `/tmp/go2w_plain_slam` 写两个 `.bin`，
**运行期从不回收**（回环检测和出图都要读回它们，删不得），只有建图收尾的
`WriteMapCloudPCD()` 才清；而 start 脚本只在启动时 `rm -rf` 一次 →
2026-09-11 把根文件系统写到 100%（约 47G）。

已加**运行期守护** `scripts/go2w/guard_plain_slam_disk.sh`：缓存 >20G 或磁盘
剩余 <20G 时有序停掉建图栈（复用 `stop_plain_slam_mapping.sh`，不碰运动链），
start 脚本另加启动前磁盘预检。验证用例、阈值调法、以及**没动的那半个问题**
（上万关键帧本身就不正常，属三维建图参数，本轮按规则没碰）见验证报告文末「附」。

## 5.3.1 WebUI 地图不显示 + 网页卡死（2026-09-11 用户报告，已修，与 RAM++ 无关）

用户反馈两句话（「打开 webui 里面没有地图」「网页卡死了没反应」）**是同一个 bug**：
`odom_base` 发散 → 写入器把发散的位姿收进持久化轨迹（9332 点里 9206 个涨到 1e20）
→ 据此算出的 `focus_bounds` 到 ±1e19 → 前端 `span≈1.7e20`，于是
**scale≈0 地图缩成一个像素（看不见）**，且 `drawGrid` 的网格循环要迭代
**3.5e19 次把浏览器主线程永久卡死**。

诱因是 **c776eae（当天 14:41）把 `centerAndSpan()` 从「只用点云 bounds」改成
「优先用 focusBounds」** —— 脏数据一直都在，只是之前没人拿它定画幅。

已修三处（提交 `022e2cc`）：写入器丢掉 >1000 m 的发散点、前端加 `saneFrame()`
退回点云 bounds、`index.html` 的 `?v=` 从 `go2w-fix-20260901` 提到 `20260911`。
数据侧把轨迹从 9332 点清回 126 点并重算了快照的 `focus_bounds`。
**没有碰三维建图**；`odom_base` 为何发散属三维建图范畴，仍未查。
细节与验证用例见验证报告文末「附」。

⚠️ **一条被我自己写错又更正的结论，留在这里防风：**
一度写成"`?v=` 没 bump → 用户拿到缓存里的旧 JS → **修复等于没生效**"，
还据此改了 `web_server.py`。**这条是错的，改动已撤回。**
实测 `/static/` 的响应头本来就带 `no-store`（提交 `8c85c70` 引入的
`_no_store_static` 中间件，**就是专门用来取代手工 `?v=` 的**），
所以那个版本号是冗余装饰，没 bump 不影响拿到新 JS。
用户当时的真实情况是**标签页已卡死、没重新加载**。
A0 复看只需**关掉卡死的标签页重开**，**不需要 `Ctrl+Shift+R`**。
自查命令：`curl -s -D - -o /dev/null http://127.0.0.1:8765/static/slam_map_3d.js`

## 5.4 需要拿主意的事（按优先级）

1. **B3 已完成（2026-09-11）；B4 于 2026-09-14 获得用户批次授权，执行中。**
   既有授权不跨批次、不跨重启、不跨参数变更，每批之间要回到人。
   开跑前三件事见验证报告「下一步」（`.env` 超时值 65、同查询跑 3 次记命中率、
   每次移动后回胶带原点）。

   **B4 开跑前已排除的阻塞（2026-09-14）**：
   机器狗 09-11 17:49 没电关机后，**宿主运动栈不会自愈** ——
   进程全部存活、`ros2 topic info` 仍报 `pub=1`、`/go2w/odom/fused` 照常 20 Hz，
   全是假绿，实际 SDK 链接已冻死 2.7 天。判据是
   `/tmp/go2w_network_low_state.json` 与 `/tmp/go2w_sdk_sport_state.json` 的 mtime
   是否推进；修法是 `kill -TERM -<launch_pid>` 后重跑 `start_motion_control.sh`。
   已重启并复检通过（`ros2 topic echo --once /go2w/motion/local_lowstate` 实收消息）。
   详见 memory `go2w-motion-stack-stale-after-robot-reboot`。
2. **关键帧爆炸 —— 2026-09-11 B3 真机批次已实测。**
   机理（读代码，未改一行）：加帧判据（`slam_3d_interface.cpp:151-157`）里
   `min_local_map_matching_rate: 0.95` 这道"扫描对不上就加帧"的安全网，
   比 `distance_th: 10.0`（米）那条"每 10 m 一帧"的正常规则**更先触发**，
   于是 10 m 规则形同虚设。
   **实测（B3：59.89 m 一圈，54 个关键帧 / 101 MB）= 1.11 m 一帧。**
   反推 47 GB（按实测 1.87 MB/帧）≈ **25,700 帧**：
   - 按 1.11 m/帧 → 要走 **28.5 km**，一次会话不可能；
   - 按"每帧都被判成关键帧"（雷达 10 Hz）→ 只要 **43 分钟**，正好是一次会话长度。

   **∴ 爆炸不是稳态行走累出来的，是某一段时间里每一帧都被判成了关键帧。**
   什么会让每一帧都对不上局部图？位姿跳变。**∴ 与 odom 发散同源 —— 不再是推测。**

   **B3 同时抓到了 odom 发散的活体前兆**（狗静止 63 s 期间的 LIO 内部状态，
   数据来自 `[GO2W-DIAG]` 行，源码 `lio_3d_interface.cpp:236-264`）：

   | 量 | 实测 | 该有的样子 |
   |---|---|---|
   | `pos` | 钉死 (0.97, 5.71) 不动 | 不动 ✅ |
   | `v` = `imu_state_.v.norm()` | 恒 **~1.0 m/s**，93 s 只从 1.50 衰到 0.97 | 停下 1–2 s 内归零 |
   | `ab` 加计零偏 | 从 0 **单调涨到 1.305 m/s²**（≈13% g） | ~0.01–0.05 |
   | `pre_t` / `fix_t` | 每帧 ≈0.11 m / ≈0.11 m | 都该 ≈0 |

   即：预积分每帧预测走 0.11 m，配准又把它拉回 0.11 m，**两股力每帧对拉**。
   姿态/重力估计在走动中退化，误差被加计零偏吃掉，速度状态因此不肯归零
   —— **滤波器不自洽**。位置这一圈还靠扫描匹配钉住了；一旦场景失去约束
   （空旷、快速运动），位置就会跑掉，那就是 1e20。
   旁证：同一次会话 z 漂了 **0.965 m**（狗不可能下沉 96 cm），姿态估计确实有病。

   改 `min_local_map_matching_rate` 或 LIO 滤波器都属三维建图，**须单独授权**。
3. **两份 `TargetProfile` 要不要合并** —— 目前靠人工同步 + 一条回归测试兜着，
   合并方法照 `app/vision/target_profile.py` 那个 shim 的既有做法（把
   `app/video/target_profile.py` 也改成 5 行转发壳）。爆炸半径：`app/video` 那份
   被 4 个生产模块 + 3 个测试文件引用，见「诚实记录」第 9 条。
   **本轮建议：不急着合** —— 合并没有行为变化，不会让 B4 更容易过，
   而真机测试前夜动 import 结构不值当。
4. **A0 肉眼确认** —— `http://127.0.0.1:8765` 的 WebUI 地图去糊效果，
   本轮唯一没关闭的无机器人交付项。用户首次复看时地图**根本看不见**，
   那是画幅被污染数据撑爆（见 §5.3.1），**不是去糊效果本身** —— A0 仍未验收。
5. **`/api/slam/map3d` 是 850 ms 轮询、每次回 16610 个点** —— 本轮卡死的主因
   不是它（是画幅），但它是前端负载的大头。降频或减点会影响观感，
   **没有动**：A0 就是"肉眼看去糊效果"，在 A0 关闭前改渲染节奏会污染这次验收。
   （原先这里还列了一条"`?v=` 该改成按 mtime 自动生成"—— 那是基于错误诊断的
   建议，已作废，理由见 §5.3.1。）

## 5.5 本轮新踩的坑（省时间用）

- **全量测试只有唯一一个解释器能跑**：
  `/home/mxt/anaconda3/envs/go2_robot_scene_demo/bin/python`。
  `.venv` 缺 scipy（2 个 collection error），anaconda base 缺 openai（37 个）。
  公平对比（两边都带 `.env`）：
  **干净 HEAD 12 failed / 1020 passed / 5 skipped / 1069 s**；
  **当前树 14 failed / 1048 passed / 1 skipped / 1052 s**。
  逐条 diff 后**变绿 0 项、新挂 2 项、稳定失败 12 项**，
  2 项新挂都已用实验排除（见验证报告「最终回归」）。
- **⚠️ 基线树必须自带 `.env`，否则跑出来的绿是假的**：
  `git archive`/`worktree`/`clone` 出来的干净树**不含 `.env`**（被 gitignore），
  而 `app/config.py:708` 是 `load_dotenv(_project_root() / ".env")` ——
  按**文件位置**定位项目根，跟 cwd 无关。没有 `.env` → LLM key 为空 →
  `parse_robot_task` 走**规则降级路径**，不发网络请求：
  3 个 LLM 用例会在 **2.54 s** 里"全绿"，全量只要 **486 s**；
  带 `.env` 的真实环境是 **142–162 s** 和 **1069 s**。
  **判据：跑基线时先 `ls <基线树>/.env`，并拿耗时对一眼**（差一倍就是中招了）。
  少了它，LLM 相关用例的「基线通过」全是幻觉，据此做 diff 会把环境差异
  误判成代码回归 —— 本轮就踩了这个坑（详见验证报告「最终回归」）。
  **当前 `/tmp/rs_baseline/.env` 是本轮补的**（权限 600），留着是为了让那棵树
  继续可用；`/tmp` 重启即清，要重跑基线记得重新 `cp` 一份。
- **`setsid cmd &` 的 `$!` 不保证是那个进程**：setsid 当上进程组组长时会 fork，
  `$!` 拿到的是 fork 完就退出的父进程。带 job control 的 shell 里必然如此
  （实测 `$!`=276219、真实进程=276221）。新写的守护自己写 `$$`；
  既有的 `plain_slam_go2w.pid` 有 pgrep 兜底，**没动**。
- **`.env.example` 的 `VLM_RUNTIME_QUICK_TIMEOUT_SECONDS` 曾是过时残留**
  （写 20，代码默认 65，verify/semantic 两项两边本来就一致），本轮对齐成 65。
  注意 `bootstrap_fresh_machine.sh:110` 会 `cp .env.example .env`，
  所以**新部署**机器的 quick 超时会从 20 变 65。本机 `.env` 三项都没设，不受影响。
- **`tests/test_ram_scene_payload.py` 用的是 `_ram_scene_payload` 的镜像副本**，
  不是 import 生产代码（其余新测试都是直接 import）。改生产那份时记得同步它。
