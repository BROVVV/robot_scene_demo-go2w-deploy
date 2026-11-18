#!/usr/bin/env python3
"""从持久录制包离线重建一次自主搜索的回放视频（不依赖机器狗与 Web）。

录制包是搜索时由只读采集器写下的 ``<session-dir>/replay/`` 目录；本脚本
只读它，把成品写回同一目录的 ``replay/artifacts/``：

* ``replay.mp4``      1920x1080 / 30 fps / H.264 yuv420p / faststart
* ``final_map.png``   最终二维占据地图
* ``cover.jpg``       封面
* ``chapters.json``   只有时长跨章节时才生成

与 Web 端生成走同一条代码路径（全局编码锁 + 会话锁 + 原子改名 + 解码
校验），所以可以在 Web 正跑着的时候手工重建某个历史会话。

用法::

    .venv/bin/python scripts/go2w/render_search_replay.py \\
        --session-dir outputs/live_runs/<session_id> --speed 10

本脚本不导入 rclpy，不连接机器狗，不发布任何 ROS 消息。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from app.search_replay.manager import ReplayManager  # noqa: E402
from app.search_replay.models import load_replay_config  # noqa: E402

PLAYABLE = {"READY", "PARTIAL"}


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "从持久录制包重新生成自主搜索回放视频。只读录制包，不需要"
            "机器狗在线，也不需要 Web 正在运行。"
        ),
        epilog=(
            "示例：.venv/bin/python scripts/go2w/render_search_replay.py "
            "--session-dir outputs/live_runs/search_20261021_121136_readonly "
            "--speed 10"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--session-dir", required=True, type=Path,
        help="会话目录（其下应有 replay/manifest.json 与采集数据）",
    )
    parser.add_argument(
        "--speed", type=int, default=None,
        help="回放倍速；默认取 configs/go2w/search_replay.yaml 的 default_speed",
    )
    parser.add_argument(
        "--force", action="store_true",
        help="即使已有同倍速成品也重新编码（旧成品会归档到 artifacts/versions/）",
    )
    parser.add_argument(
        "--config", type=Path, default=None,
        help="回放配置文件路径（默认 configs/go2w/search_replay.yaml）",
    )
    parser.add_argument(
        "--json", action="store_true", help="输出机器可读的结果 JSON",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    session_dir = args.session_dir.expanduser()
    if not session_dir.is_absolute():
        session_dir = (Path.cwd() / session_dir).resolve()
    if not (session_dir / "replay" / "manifest.json").is_file():
        print(f"错误：{session_dir} 下没有 replay/manifest.json", file=sys.stderr)
        return 2

    config = load_replay_config(PROJECT_ROOT, args.config)
    manager = ReplayManager(
        config,
        sessions_root=session_dir.parent,
        project_root=PROJECT_ROOT,
        source="offline_cli",
        config_path=args.config,
    )
    result = manager.render_offline(
        session_dir.name, speed=args.speed, force=args.force,
    )
    status = str(result.get("status") or "")
    payload = {
        **result,
        "session_id": session_dir.name,
        "speed": result.get("speed"),
        "playable": status in PLAYABLE,
        "video": str(session_dir / "replay" / "artifacts" / "replay.mp4"),
    }
    if args.json:
        print(json.dumps(payload, ensure_ascii=False, indent=1))
    elif result.get("ok") and status in PLAYABLE:
        print(f"{status} {payload['video']}"
              + (f"  reasons={result.get('reasons')}" if result.get("reasons")
                 else ""))
    else:
        print(f"失败：{result.get('error') or status or result}", file=sys.stderr)
    return 0 if result.get("ok") and status in PLAYABLE else 1


if __name__ == "__main__":
    raise SystemExit(main())
