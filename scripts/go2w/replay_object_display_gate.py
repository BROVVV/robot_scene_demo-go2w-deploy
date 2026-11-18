#!/usr/bin/env python3
"""拿一份历史 events.jsonl 回放 WebUI 三维物体框的显示门，打出一张审计表。

用途（计划书 §8.1）：改门限之前先看清"哪些物体会被滤掉、按什么理由"。审计表
把显示和被丢的**都**列出来 —— 只记被丢的，事后答不了"某个东西为什么没上屏、
换个门限会不会上屏"。

判定走的和快照写端是同一套代码（`app/spatial/semantic_object_gate.py`），
所以这里的结论对线上有约束力；唯一的差别是写端还会叠一层"落在当前地图包围盒
之外就丢"（那挡的是另一个 SLAM 会话的过期坐标，与分数无关，回放没有地图可比）。

用法：

    /usr/bin/python3 scripts/go2w/replay_object_display_gate.py \
        outputs/live_runs/search_20260917_190603_df66d115/events.jsonl

    # 试别的门限，不动代码：
    ... --confirmed 0.45 --tentative 0.65 --min-observations 2
"""

from __future__ import annotations

import argparse
import os
import sys

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from app.spatial.semantic_object_gate import (  # noqa: E402
    DEFAULT_CONFIRMED_MIN_CONFIDENCE,
    DEFAULT_CONFIRM_MIN_OBSERVATIONS,
    DEFAULT_INGEST_MIN_CONFIDENCE,
    DEFAULT_TENTATIVE_MIN_CONFIDENCE,
    audit_rows,
    filter_persistent_observations,
    load_object_records,
    select_display_objects,
)


class _Observation:
    """入口门的入参形状：它只读 `label` / `label_zh` / `confidence` 三个属性。

    生产里这是 `ObjectSpatialObservation`；回放时没必要为了调用一个纯函数去
    造一整套定位流水线的对象。
    """

    __slots__ = ("label", "label_zh", "confidence")

    def __init__(self, record: dict) -> None:
        self.label = record.get("label")
        self.label_zh = record.get("label_zh")
        self.confidence = record.get("confidence")


def iter_frame_observations(events_path: str):
    """逐帧原始观测 —— 入口门的输入。

    取 `semantic_frame_objects_localized` 而不是 `semantic_object_localized`：
    后者是**过了入口门、且已经合并成实体**之后写的，拿它回放入口门等于拿结论
    去验证前提。前者写的是 `observation.scene_objects`，即检测端交给下游的
    候选表，整改前后一条不少（这正是"低分候选仍可在日志里取证"的落点）。

    **口径说明**：这是检测端的视角，不是入口门入参 `localized` 的逐字复制
    （`localized` 是同一帧的定位结果，两者对已定位的物体一一对应，但定位失败
    的候选只会出现在这边）。所以这里算出的条数是入口门输入的**上界**，用来
    判断"门会拦掉哪一类"，不能拿来当线上 `semantic_object_ingest_filter`
    事件的精确值 —— 那个数以真机跑出来的事件为准。
    """
    import json

    with open(events_path, encoding="utf-8") as handle:
        for line in handle:
            try:
                event = json.loads(line)
            except json.JSONDecodeError:
                continue
            if event.get("event") != "semantic_frame_objects_localized":
                continue
            for record in event.get("objects") or []:
                yield record


def _score(value: object) -> str:
    return "--" if value is None else f"{float(value):.3f}"


def _width(text: object) -> int:
    """终端里的显示宽度：中日韩字符占两格，`ljust` 按字符数补不对齐。"""
    return sum(2 if "ᄀ" <= char <= "鿿" else 1 for char in str(text))


def _pad(text: object, width: int) -> str:
    value = str(text)
    return value + " " * max(0, width - _width(value))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("events", help="要回放的 events.jsonl")
    parser.add_argument("--confirmed", type=float,
                        default=DEFAULT_CONFIRMED_MIN_CONFIDENCE)
    parser.add_argument("--tentative", type=float,
                        default=DEFAULT_TENTATIVE_MIN_CONFIDENCE)
    parser.add_argument("--min-observations", type=int,
                        default=DEFAULT_CONFIRM_MIN_OBSERVATIONS)
    parser.add_argument("--ingest", type=float,
                        default=DEFAULT_INGEST_MIN_CONFIDENCE,
                        help="持久地图入口门的门限（§5.1）")
    args = parser.parse_args()
    if not os.path.isfile(args.events):
        print(f"no such events file: {args.events}", file=sys.stderr)
        return 2

    records = load_object_records(
        args.events, confirm_min_observations=args.min_observations)
    rows = audit_rows(
        records,
        confirmed_min_confidence=args.confirmed,
        tentative_min_confidence=args.tentative,
        confirm_min_observations=args.min_observations,
    )
    visible, stats = select_display_objects(
        records,
        confirmed_min_confidence=args.confirmed,
        tentative_min_confidence=args.tentative,
        confirm_min_observations=args.min_observations,
    )

    print(f"events: {args.events}")
    print(f"门限: confirmed>={args.confirmed} tentative>={args.tentative} "
          f"confirm_min_observations={args.min_observations}")
    print(f"持久 ID: {len(records)} 个，显示 {stats['displayed']} 个，"
          f"过滤 {stats['filtered']} 个")
    print()

    # ---- 入口门回放（§5.1）-----------------------------------------------
    # 这段答的是"低于门限的观测还会不会建出持久实体"，不是"还会不会上屏"。
    # 两者是两道门，混在一起报会看不清究竟是哪一道在挡。
    frame_records = list(iter_frame_observations(args.events))
    kept, ingest_dropped = filter_persistent_observations(
        [_Observation(record) for record in frame_records],
        min_confidence=args.ingest,
    )
    print(f"入口门（>={args.ingest}）: 逐帧观测 {len(frame_records)} 条，"
          f"保留 {len(kept)} 条，丢弃 " + (", ".join(
              f"{reason}={count}"
              for reason, count in sorted(ingest_dropped.items()) if count
          ) or "0 条"))
    kept_scores = [
        float(item.confidence) for item in kept
        if isinstance(item.confidence, (int, float))
    ]
    if kept_scores:
        print(f"  保留观测分数区间 {min(kept_scores):.3f} ~ {max(kept_scores):.3f}")
    # 老事件里的 `confidence` 就是当时的 max()（整改前 `_merge` 取最大值），
    # 所以 confidence_max < 门限 ⟺ 这个实体的**每一次**观测都低于门限 ——
    # 新入口门下它压根不会被建出来。反过来不成立：max 达标只说明"后来见过
    # 一次好的"，不排除它是靠一次低分观测先建出来的。
    never_created = [
        row for row in rows
        if float(row["confidence_max"] or 0.0) < args.ingest
    ]
    print(f"  全部观测都低于入口门的持久 ID（新门下不会存在）: "
          f"{len(never_created)} 个 / 共 {len(rows)} 个")
    if never_created:
        print("    " + ", ".join(
            f"{row['object_id']}({row['raw_label']},{row['confidence_max']:.3f})"
            for row in never_created
        ))
    print()

    columns = [
        ("object_id", 9), ("raw_label", 16), ("label_zh", 12),
        ("conf", 6), ("max", 6), ("n", 3), ("status", 10),
        ("decision", 15), ("drop_reason", 24), ("box", 4),
    ]
    print("  ".join(_pad(name, width) for name, width in columns))
    print("  ".join("-" * width for _name, width in columns))
    for row in rows:
        print("  ".join(_pad(value, width) for (name, width), value in zip(
            columns,
            (
                row["object_id"], row["raw_label"], row["label_zh"],
                _score(row["confidence"]), _score(row["confidence_max"]),
                row["observation_count"], row["status"], row["decision"],
                row["drop_reason"] or "", "Y" if row["box_valid"] else "n",
            ),
        )))

    print()
    print("丢弃原因统计: " + (", ".join(
        f"{reason}={count}" for reason, count in sorted(stats["dropped"].items())
    ) or "无"))
    # 验收要点：可见名称必须 100% 是中文。
    english = [
        item["label_zh"] for item in visible
        if not any("一" <= char <= "鿿" for char in str(item["label_zh"]))
    ]
    print("显示名含英文的: " + (", ".join(english) if english else "无"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
