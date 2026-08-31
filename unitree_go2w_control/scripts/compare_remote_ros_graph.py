#!/usr/bin/env python3
"""Compare passive ROS graph snapshots captured before and after each round."""

from __future__ import annotations

import argparse
from pathlib import Path


def lines(path: Path) -> set[str]:
    if not path.exists():
        return set()
    return {line.strip() for line in path.read_text(encoding="utf-8").splitlines() if line.strip()}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-root", required=True, type=Path)
    args = parser.parse_args()
    root = args.capture_root.resolve()
    before_root = root / "metadata" / "ros_graph_before"
    after_root = root / "metadata" / "ros_graph_after"
    output = root / "analysis" / "ros_graph_diff.md"
    output.parent.mkdir(parents=True, exist_ok=True)
    rounds = sorted(
        {path.name for path in before_root.iterdir() if path.is_dir()}
        if before_root.exists()
        else set()
    )
    with output.open("w", encoding="utf-8") as handle:
        handle.write("# ROS 图变化\n\n")
        if not rounds:
            handle.write("没有可比较的分轮 ROS 图快照。\n")
        for round_name in rounds:
            before = before_root / round_name
            after = after_root / round_name
            handle.write(f"## {round_name}\n\n")
            for filename, label in (("topics.txt", "topic"), ("nodes.txt", "node")):
                old = lines(before / filename)
                new = lines(after / filename)
                added = sorted(new - old)
                removed = sorted(old - new)
                handle.write(f"- 新增 {label}: {added or '无'}\n")
                handle.write(f"- 消失 {label}: {removed or '无'}\n")
            for topic_file in (
                "topic_api_sport_request.txt",
                "topic_api_sport_response.txt",
                "topic_wirelesscontroller.txt",
                "topic_wirelesscontroller_unprocessed.txt",
            ):
                old = lines(before / topic_file)
                new = lines(after / topic_file)
                if old != new:
                    handle.write(f"- `{topic_file}` endpoint 文本发生变化。\n")
            handle.write("\n")
    print(f"ROS_GRAPH_DIFF=PASS rounds={len(rounds)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
