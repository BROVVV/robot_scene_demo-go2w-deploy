#!/usr/bin/env python3
"""把 v1 的 WebUI 搜索归档一次性迁到 §2.2 的「小状态 + 图分文件」形状。

计划书 docs/GO2W_搜索状态与地图链路_一次性架构整改计划_20261008.md §2.2：

    outputs/live_runs/<session_id>/
      webui_session.json   # 会话索引
      webui_state.json     # 不含 graph 的小状态
      webui_graph.json     # 唯一图快照
      webui_events.jsonl   # 只追加小事件
      events.jsonl         # worker 原始运行证据

v1 的 `webui_state.json` 里同一张语义图被存了 **三份**（`map.semantic_navigation_graph`、
`spatial.semantic_graph`、`spatial.spatial_map`，实测三份逐字节相同），另有
`map.nodes/edges/revision` 与 `spatial.{place_graph,semantic_objects,frontiers}`
等从同一张图再投影一遍的字段。本脚本把图抽到 `webui_graph.json`，并把状态里
那些重复字段删掉。

**默认只报告，不改任何文件**；确认报告无误后加 `--apply`。运行时不做静默兼容，
所以这份迁移必须显式跑一次 —— 计划书 §2.2 的原话是"不要在每次启动时静默兼容
或自动迁移"。

    # 1. 先看一遍（不写任何东西）
    .venv/bin/python scripts/go2w/migrate_webui_archive_v1.py

    # 2. 确认后执行（默认备份到 outputs/live_runs_v1_backup/<时间戳>/）
    .venv/bin/python scripts/go2w/migrate_webui_archive_v1.py --apply

几个刻意的取舍：

* **图文件写成 `web_snapshot()` 的形状**（只留 schema_version / revision /
  current_place_id / places / objects / frontiers / edges / object_topology）。
  旧图里的 `nodes` 就是 places+objects+frontiers 的并集（实测 134 = 122+1+11，
  逐条重复），`observations` 计划书 §2.1 明说是离线证据、不进 Web 图，前端
  没有任何一处读 `state.graph.nodes` / `.observations`（已 rg 核对）。丢掉这两
  个键，迁移后的文件才和在线写出来的是**同一个形状** —— "运行时只有一种读法"
  说的就是这个。原文件完整留在备份里。
* **逐会话整目录备份**再改。`events.jsonl` 是原始运行证据，本脚本从不读写它。
* 幂等：已经有 `webui_graph.json` 的会话直接跳过，反复跑不会二次投影。
* 没有图可提取的会话（只有 `webui_session.json` 的启动即失败）**不造空图**，
  只报告。计划书 §6：不虚构默认值。
* 三个 v1 会话的 `webui_state.json` 在本仓库里是 **git 跟踪文件**
  （search_20260901_120501_96392a00 / _121219_0a3965d1 / _122525_25291a89），
  改它们会产生 diff；脚本会在报告里点名，是否提交由人决定。
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
import time
from pathlib import Path
from typing import Any

_PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_PROJECT_ROOT))

from app.manual_web_demo.search_session_archive import (  # noqa: E402
    GRAPH_FILE,
    MARKER_FILE,
    STATE_FILE,
)

# 旧状态里的重复图字段：图副本本身，和从同一张图再投影一遍的字段。
LEGACY_STATE_KEYS = ("map",)
LEGACY_SPATIAL_KEYS = (
    "semantic_graph", "spatial_map", "place_graph", "semantic_objects", "frontiers",
)
# 图的三份副本，按优先级取第一份是 dict 的。实测三份相同时取谁都一样（脚本会核对）。
GRAPH_SOURCES = (
    ("map", "semantic_navigation_graph"),
    ("spatial", "semantic_graph"),
    ("spatial", "spatial_map"),
)
# §2.1 的 Web 图只保留这些键。
WEB_GRAPH_KEYS = (
    "schema_version", "revision", "current_place_id",
    "places", "objects", "frontiers", "edges", "object_topology",
)


def _read_json(path: Path) -> Any:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def _write_json_atomic(path: Path, value: Any) -> None:
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        json.dump(value, handle, ensure_ascii=False, separators=(",", ":"))
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)


def _find_graph(state: dict[str, Any]) -> tuple[dict[str, Any], str] | tuple[None, str]:
    """返回 (图, 来源描述)。三份副本都在时核对是否相同 —— 不同就报出来。"""
    found: list[tuple[str, dict[str, Any]]] = []
    for section, key in GRAPH_SOURCES:
        block = state.get(section)
        if isinstance(block, dict) and isinstance(block.get(key), dict):
            found.append((f"{section}.{key}", block[key]))
    if not found:
        return None, "无"
    if len(found) > 1 and any(graph != found[0][1] for _, graph in found[1:]):
        names = ", ".join(name for name, _ in found)
        raise SystemExit(f"图的三份副本不一致（{names}）；不猜，请人工确认。")
    return found[0][1], ", ".join(name for name, _ in found)


def _project(graph: dict[str, Any]) -> dict[str, Any]:
    return {key: graph[key] for key in WEB_GRAPH_KEYS if key in graph}


def _plan_session(directory: Path, *, tracked: set[str]) -> dict[str, Any]:
    report: dict[str, Any] = {"session": directory.name, "action": "", "notes": []}
    marker = directory / MARKER_FILE
    state_path = directory / STATE_FILE
    graph_path = directory / GRAPH_FILE
    report["tracked"] = any(
        str(path).startswith(f"{directory.name}/") for path in tracked
    )
    if not marker.is_file():
        report["action"] = "skip"
        report["notes"].append(f"没有 {MARKER_FILE}（不是 WebUI 会话）")
        return report
    if graph_path.is_file():
        report["action"] = "skip"
        report["notes"].append(f"已有 {GRAPH_FILE}，已迁移过")
        return report
    if not state_path.is_file():
        report["action"] = "skip"
        report["notes"].append(f"没有 {STATE_FILE}")
        return report
    try:
        state = _read_json(state_path)
    except (OSError, json.JSONDecodeError) as exc:
        report["action"] = "skip"
        report["notes"].append(f"{STATE_FILE} 读不出来：{exc}")
        return report
    if not isinstance(state, dict):
        report["action"] = "skip"
        report["notes"].append(f"{STATE_FILE} 顶层不是对象")
        return report

    graph, source = _find_graph(state)
    report["graph_source"] = source
    report["notes"].append(f"图来源：{source}")
    if graph is None:
        report["action"] = "no-graph"
        report["notes"].append("没有可提取的图 —— 不造空图，只清重复字段")
    else:
        report["action"] = "migrate"
        report["graph_revision"] = graph.get("revision")
        report["graph_keys_dropped"] = sorted(set(graph) - set(WEB_GRAPH_KEYS))
        report["graph_counts"] = {
            key: len(graph[key])
            for key in ("places", "objects", "frontiers", "edges", "nodes", "observations")
            if isinstance(graph.get(key), list)
        }
    report["state_bytes"] = state_path.stat().st_size
    spatial = state.get("spatial") if isinstance(state.get("spatial"), dict) else {}
    report["drop_top_level"] = [key for key in LEGACY_STATE_KEYS if key in state]
    report["drop_spatial"] = [key for key in LEGACY_SPATIAL_KEYS if key in spatial]
    return report


def _apply_session(
    directory: Path,
    *,
    backup_dir: Path,
    stats: dict[str, Any],
) -> None:
    backup_dir.mkdir(parents=True, exist_ok=True)
    # 整目录备份：events.jsonl / webui_events.jsonl 是原始证据，不碰它们。
    shutil.copytree(directory, backup_dir / directory.name, dirs_exist_ok=True)

    state_path = directory / STATE_FILE
    state = _read_json(state_path)
    graph, source = _find_graph(state)
    if graph is not None:
        _write_json_atomic(directory / GRAPH_FILE, _project(graph))

    state.pop("map", None)
    spatial = state.get("spatial")
    if isinstance(spatial, dict):
        for key in LEGACY_SPATIAL_KEYS:
            spatial.pop(key, None)
    _write_json_atomic(state_path, state)

    # 写完立刻按运行时的读法复核一次：图能解析、状态里没有重复图字段。
    if graph is not None:
        written = _read_json(directory / GRAPH_FILE)
        if written.get("revision") != graph.get("revision"):
            raise SystemExit(f"{directory.name}: graph revision 复核不一致，已中止")
        if set(written) - set(WEB_GRAPH_KEYS):
            raise SystemExit(f"{directory.name}: 图文件仍有 Web 图之外的键，已中止")
    reread = _read_json(state_path)
    if "map" in reread or any(
        key in (reread.get("spatial") or {}) for key in LEGACY_SPATIAL_KEYS
    ):
        raise SystemExit(f"{directory.name}: 重复图字段没删干净，已中止")
    stats["after_bytes"] = state_path.stat().st_size


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", default=str(_PROJECT_ROOT / "outputs" / "live_runs"),
                        help="WebUI 归档根目录（默认 outputs/live_runs）")
    parser.add_argument("--apply", action="store_true",
                        help="真的写盘；不加则只报告（默认）")
    parser.add_argument("--backup-root", default="",
                        help="备份根目录（默认 outputs/live_runs_v1_backup/<时间戳>）")
    parser.add_argument("--session", default="", help="只处理一个 session_id")
    args = parser.parse_args()

    root = Path(args.root).resolve()
    if not root.is_dir():
        print(f"ERROR: 归档根目录不存在: {root}", file=sys.stderr)
        return 2

    # 判断"这个会话有没有 git 跟踪文件"：一个会话目录只有几个文件，不必为它
    # 去跑 git；--session 模式或没有 .git 时直接跳过这项提示。
    tracked: set[str] = set()
    git_dir = _PROJECT_ROOT / ".git"
    if git_dir.exists():
        import subprocess
        try:
            out = subprocess.run(
                ["git", "ls-files", "--", str(root)],
                cwd=_PROJECT_ROOT, capture_output=True, text=True, timeout=60,
            )
            prefix = f"{root.relative_to(_PROJECT_ROOT)}/" if root.is_relative_to(
                _PROJECT_ROOT) else ""
            tracked = {
                line[len(prefix):] if prefix and line.startswith(prefix) else line
                for line in out.stdout.splitlines()
            }
        except (OSError, subprocess.SubprocessError, ValueError):
            tracked = set()

    directories = sorted(
        (item for item in root.iterdir() if item.is_dir()),
        key=lambda item: item.name,
    )
    if args.session:
        directories = [item for item in directories if item.name == args.session]
        if not directories:
            print(f"ERROR: 找不到会话 {args.session}", file=sys.stderr)
            return 2

    stamp = time.strftime("%Y%m%d_%H%M%S")
    backup_root = Path(args.backup_root).resolve() if args.backup_root else (
        root.parent / f"{root.name}_v1_backup" / stamp)

    reports = [_plan_session(item, tracked=tracked) for item in directories]
    to_migrate = [report for report in reports if report["action"] in ("migrate", "no-graph")]
    # 归档根目录下绝大多数目录是历史的 CLI/实验运行（没有 webui_session.json）。
    # 逐个打印会把真正要迁的十几个埋在 300 行里，所以那些只计数、不逐条列。
    foreign = [report for report in reports
               if report["action"] == "skip" and report["notes"]
               and report["notes"][0].startswith(f"没有 {MARKER_FILE}")]

    print(f"归档根目录 : {root}")
    print(f"模式       : {'APPLY（会写盘）' if args.apply else 'DRY-RUN（不写任何文件）'}")
    print(f"备份目录   : {backup_root}")
    print(f"会话       : {len(reports)} 个（待迁移 {len(to_migrate)}，"
          f"非 WebUI 会话 {len(foreign)}，其他跳过 {len(reports) - len(to_migrate) - len(foreign)}）\n")
    for report in reports:
        if report in foreign:
            continue
        flag = " " if not report.get("tracked") else "G"
        print(f"[{flag}] {report['session']}  -> {report['action']}")
        for note in report["notes"]:
            print(f"        {note}")
        if report["action"] == "migrate":
            print(f"        丢掉的图键: {report.get('graph_keys_dropped')}")
            print(f"        图规模    : {report.get('graph_counts')}")
        if report.get("drop_top_level") or report.get("drop_spatial"):
            print(f"        状态删除  : map={report.get('drop_top_level')} "
                  f"spatial={report.get('drop_spatial')}")
    print(f"\n（另有 {len(foreign)} 个目录没有 {MARKER_FILE}，是历史的 CLI/实验运行，不动）")
    print("G = 该会话有 git 跟踪文件（迁移会产生 diff；提交与否由人决定）")

    if not args.apply:
        print("\nDRY-RUN 结束：没有写任何文件。加 --apply 执行。")
        return 0

    migrated = 0
    for report, directory in zip(reports, directories):
        if report["action"] not in ("migrate", "no-graph"):
            continue
        stats: dict[str, Any] = {}
        _apply_session(directory, backup_dir=backup_root, stats=stats)
        migrated += 1
        print(f"[OK] {directory.name}: state {report.get('state_bytes', 0)} B"
              f" -> {stats.get('after_bytes', 0)} B; 备份在 {backup_root / directory.name}")
    print(f"\n迁移完成：{migrated} 个会话。原文件在 {backup_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
