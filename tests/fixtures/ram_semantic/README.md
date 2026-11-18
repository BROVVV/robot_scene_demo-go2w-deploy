# §43 六类场景数据集：哪几个真的需要图片

RAM++ 计划 §43 要求"准备最少 6 类静态图片/录制帧"。逐类核对之后的结论是：
**六类里只有一类真的需要图片。** 写清楚这件事，比凑齐六个目录有用。

| 场景 | §43 要求 | 实际覆盖 | 需要图片吗 |
|---|---|---|---|
| **A 普通办公室** | RAM++ 给出大部分物体词、DINO 定位主要实例 | `scene_a_office.json` + `scripts/go2w/eval_ram_semantic_coverage.py` | **要**（本目录） |
| **B RAM++ 属性混杂** | `gray`/`green`/`room`/`sit`/`wood floor` 等属性动作词不能污染 DINO prompt | `tests/test_ram_scene_terms.py`（8 tests，在 `unittest.TestCase` 里） | **不要** —— 这是**过滤**行为，纯函数、无图 |
| **C 黄色椅子** | RAM++ 故意 mock 成不含 `chair`，最终 prompt 仍须含 `chair`/`yellow chair`，并能走到 VLM attribute verify | `tests/test_ram_target_priority.py` + `tests/test_yellow_chair_vlm_verify.py` | **不要** —— §43 原文就是 "RAM++ 故意 mock" |
| **D 挂书包的椅子** | DINO 找到 chair + backpack，VLM 最终只确认 chair B | `tests/test_yellow_chair_vlm_verify.py`（同帧 A/B 对照） | **不要** —— 是注入场景 |
| **E 稀有设备** | 连续空 candidate 达阈值 ⇒ VLM full-frame sweep 触发，证明旁路存在 | `tests/test_target_sweep_trigger.py`（6 tests） | **不要** —— 判据是计数器，无图 |
| **F 普通搜索无目标** | 大量普通 object 进地图、VLM 不每帧调用 | `tests/test_target_sweep_trigger.py` + `tests/test_vlm_quick_contract.py` | **不要** —— 是调用频率契约 |

## 为什么 A 只有标注、没有配好的 run

`scene_a_office.json` 里 `expected_scene_terms` 是**人工看图标的**
（依据 `runtime/go2w/rgbd_validate/color_8451.jpg`，肉眼可见办公椅×2、办公桌、隔断、垃圾桶）。
它是 `scene semantic coverage` 的**分母**，与任何模型输出无关，所以标了就是标的。

但**分子**必须来自一次真的观测过这个场景的 run。本仓库现有两次 run
（`search_20260916_110826_3f7ac850` 09-16 11:08、`explore_go2w_20260916_155531` 15:55）
都在**别的位姿**上，而 `color_8451.jpg` 拍于 09-15 16:56 —— **没有证据说明它们看过同一场景**。

所以 `frames` 与 `--run` 的配对**故意留空**。随便配一个 run 上去会得到一个
**看起来有数、其实是错的**覆盖率，那比没有数更糟。第一次真实测量要等一次
"自己标注自己那张帧"的 run。

## ⚠ 干跑记录：`4/4 = 1.000` 看着完美，但它**不是配对证据**

2026-09-16 拿 15:55 那次 run 干跑过（只为验证 CLI 不崩）：

```
== Scene A ==  run=outputs/live_runs/explore_go2w_20260916_155531
   run 语义地图物体数：18
   §42 指标一 scene semantic coverage : 4/4 = 1.000
   §42 指标二 target recall           : 不适用（目标不在场，召回率无定义）
```

该 run 的 18 个 label 是：
`人、办公桌×2、办公椅、垃圾桶×2、墙壁×3、显示器、桌子、椅子×4、纸箱、蓝色垃圾桶、隔断`。

**别把这个 4/4 读成"验证过了"。** 四个标注词是 `办公椅 / 办公桌 / 垃圾桶 / 隔断`
——**任何一间办公室都会有这四样**。一个对场景身份**没有分辨力**的标注，
在这间办公室和隔壁那间上都会给出 4/4。所以它既不能证明这次 run 看过 `color_8451.jpg`
那个位姿，也不能证伪。

这恰好暴露了标注的粗糙：**要真做配对，标注必须落在有区分度的词上**
（比如「蓝色网格垃圾桶」这种，或画面里特有的布置），而不是"办公椅、桌子"。
在那之前，`--run` 一栏**留空是对的**。

（顺带：这次 run 同时出现了 `办公椅` 与 `椅子×4`、`办公桌` 与 `桌子`
——检测器对同一批物体给了两种粒度。标注用哪一种，分母就会跟着变，
这也是标注必须先定的原因。）

## 怎么用

```bash
/usr/bin/python3 scripts/go2w/eval_ram_semantic_coverage.py \
    --run outputs/live_runs/<一次真的看过该场景、且标注有区分度的 run> \
    --manifest tests/fixtures/ram_semantic/scene_a_office.json
```

脚本打印 §42 的两项指标 —— `scene semantic coverage` 与 `target recall` ——
**分开打印，永不合并**。合并的理由不成立：09-16 11:08 那次 run 进了 57 个物体、
`navigation_failures=0`，却一次都没确认目标「红色雨伞」（`verify_attempts=0`）。
合并成一个数，它会被评成"中等偏上"，而它其实是彻底失败。

## 关于提交图片

本仓库**有**提交小体积 fixture 图片的先例（`tests/fixtures/depth_object_localizer/*.png`，
8 个）。本目录**没有**复制任何图片进来，理由是没必要：
`runtime/go2w/rgbd_validate/`、`runtime/go2w/rgbd_cache/` 里已经有几百张真实帧，
复制一份只会制造两份会各自变旧的副本。清单按**路径**引用它们。
