#!/usr/bin/env python3
"""Write an auditable, fail-closed Go2-W Nav2 capability gate result."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from app.live_robot.navigation_gate import evaluate_navigation_gate


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("nav2_plan_only", "nav2_execute"), required=True)
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "configs/go2w/navigation_gate.yaml",
    )
    parser.add_argument("--runtime-json", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    config = yaml.safe_load(args.config.read_text(encoding="utf-8")) or {}
    evidence = dict(config.get("current_evidence") or {})
    if args.runtime_json:
        runtime = json.loads(args.runtime_json.read_text(encoding="utf-8"))
        evidence.update(runtime.get("evidence", runtime))
    result = evaluate_navigation_gate(args.mode, evidence).to_dict()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["allowed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())

