"""Query the local scene knowledge base from the command line."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from app.knowledge.retrieval import retrieve_relevant_knowledge
from app.knowledge.scene_kb import SceneKnowledgeBase


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="查询本地场景知识库。")
    parser.add_argument("--target", help="目标物体或任务文本，例如：手机")
    parser.add_argument("--room_type", help="房间类型，例如：office")
    parser.add_argument("--location", help="位置线索，例如：floor_5 或 502")
    parser.add_argument("--floor_id", help="直接读取指定楼层布局，例如：floor_5")
    parser.add_argument(
        "--kb-dir",
        default=str(ROOT / "data" / "scene_kb"),
        help="知识库目录，默认使用 data/scene_kb",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    kb_dir = Path(args.kb_dir)

    if args.floor_id:
        layout = SceneKnowledgeBase(kb_dir).get_floor_layout(args.floor_id)
        payload = layout.model_dump(mode="json") if layout else None
    else:
        items = retrieve_relevant_knowledge(
            target_text=args.target,
            room_type=args.room_type,
            location_hint=args.location,
            kb_dir=kb_dir,
        )
        payload = [item.model_dump(mode="json") for item in items]

    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
