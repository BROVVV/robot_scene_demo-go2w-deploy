"""比对两次离线 A/B 回放的**决策**差异。

用法：
    python scripts/compare_reasoner_decisions.py <基线目录> <对照目录>

两个目录都是 `scripts/evaluate_live_search_reasoners.py --output-dir` 的产物
（每个会话一个子目录，里面有 reasoner_comparison.json）。

**为什么不能直接 `diff -r`**：`search_reasoner.py:_id()` 用
`uuid4().hex[:12]` 生成 `directive_id`，每次调用都不同。文件级 diff 会把
**每一个**会话都报成"有差异"—— 一个什么都没改的改动也会 diff 出满屏噪音，
于是"有差异"这个信号就彻底失效了。这个脚本只比会变的那几个字段。

**为什么必须比对决策而不是比对报告里的汇总数字**：汇总数字会互相抵消
（比如两处改动方向相反），而"哪些会话的提议步变了"才是改动真正的作用面。
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

# directive 里参与比对的字段；directive_id 故意不在其中。
DIRECTIVE_FIELDS = ("kind", "preferred_heading_delta_deg", "confidence", "anchor_label")


def _signature(report: dict) -> dict[str, tuple]:
    """一个会话的决策指纹：每个后端一条。

    `proposed_step` / `legacy_step` 在 results[i] 这一层，**不在 directive 里** ——
    从 directive 取会恒为 None，于是最该比的那个字段反而永远不比。
    """
    signature = {}
    for row in report["results"]:
        directive = row["directive"]
        match = row["graph_match"] or {}
        signature[row["backend"]] = (
            row["proposed_step"],
            tuple(directive.get(field) for field in DIRECTIVE_FIELDS),
            row["legacy_step"],
            match.get("state"),
            match.get("score"),
        )
    return signature


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", help="基线目录（改动之前跑的那次）")
    parser.add_argument("candidate", help="对照目录（改动之后跑的那次）")
    args = parser.parse_args()

    baseline = pathlib.Path(args.baseline)
    candidate = pathlib.Path(args.candidate)
    changed = 0
    missing = 0
    for base_report in sorted(baseline.glob("*/reasoner_comparison.json")):
        session = base_report.parent.name
        other_report = candidate / session / "reasoner_comparison.json"
        if not other_report.is_file():
            print(f"[缺席] {session}")
            missing += 1
            continue
        before = _signature(json.loads(base_report.read_text(encoding="utf-8")))
        after = _signature(json.loads(other_report.read_text(encoding="utf-8")))
        for backend in before:
            if before[backend] == after[backend]:
                continue
            changed += 1
            print(f"[变] {session}")
            print(f"     {backend}: {before[backend]}")
            print(f"     {' ' * len(backend)}  →  {after[backend]}")
    print(f"\n决策发生变化的 (会话,后端) 数: {changed}")
    if missing:
        print(f"对照目录里缺席的会话数: {missing}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
