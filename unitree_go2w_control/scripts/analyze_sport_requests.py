#!/usr/bin/env python3
"""Correlate captured Sport requests/responses and remove baseline traffic."""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path
from typing import Any


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    with path.open(encoding="utf-8") as handle:
        return [json.loads(line) for line in handle if line.strip()]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--capture-root", required=True, type=Path)
    args = parser.parse_args()
    analysis = args.capture_root.resolve() / "analysis"
    requests = read_jsonl(analysis / "sport_requests.jsonl")
    responses = read_jsonl(analysis / "sport_responses.jsonl")
    response_index = {
        (row["round"], row["request_id"], row["api_id"]): row for row in responses
    }
    baseline_signatures = Counter(
        (row["api_id"], row["parameter_raw"])
        for row in requests
        if row["round"] == "01_idle_baseline"
    )
    pairs = []
    classifications = Counter()
    for request in requests:
        signature = (request["api_id"], request["parameter_raw"])
        if request["round"] == "01_idle_baseline" or signature in baseline_signatures:
            classification = "baseline periodic"
        elif request["round"].startswith(("D", "U")):
            classification = "action-correlated candidate"
        else:
            classification = "one-off unrelated or unknown"
        classifications[classification] += 1
        response = response_index.get(
            (request["round"], request["request_id"], request["api_id"])
        )
        pairs.append(
            {
                **request,
                "classification": classification,
                "response": response,
                "response_delay_s": None
                if response is None
                else (response["bag_timestamp_ns"] - request["bag_timestamp_ns"]) / 1e9,
            }
        )
    with (analysis / "sport_request_response_pairs.jsonl").open(
        "w", encoding="utf-8"
    ) as handle:
        for row in pairs:
            handle.write(json.dumps(row, ensure_ascii=False) + "\n")
    with (analysis / "background_vs_action_requests.md").open(
        "w", encoding="utf-8"
    ) as handle:
        handle.write("# 背景请求与动作窗口对比\n\n")
        if not pairs:
            handle.write("未观察到公开 `/api/sport/request`。\n")
        for name, count in classifications.items():
            handle.write(f"- {name}: {count}\n")
        handle.write("\n## 动作窗口候选\n\n")
        candidates = [row for row in pairs if row["classification"].startswith("action")]
        if not candidates:
            handle.write("无。\n")
        for row in candidates:
            response = row["response"]
            handle.write(
                f"- {row['round']}: api={row['api_id']} request={row['request_id']} "
                f"lease={row['lease_id']} parameter=`{row['parameter_raw']}` "
                f"response={None if response is None else response['status_code']}\n"
            )
    print(f"SPORT_REQUEST_ANALYSIS=PASS pairs={len(pairs)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
