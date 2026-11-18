#!/usr/bin/env python3
"""量 `apply` 先 / `publish` 先 这个次序对 `TARGET_CONFIRMED` 可见性的影响。

背景（详见计划进度书 §2.9-2）：`ExplorerSearchAdapter.on_explorer_event` 先
`store.apply` 再 `bus.publish`。`apply` 内部就把快照翻成 `TARGET_FOUND`，
之后才把 `TARGET_CONFIRMED` 送上总线 ⇒ 存在一个「状态已翻、事件未入列」的窗口。
`tests/test_search_session_service.py` 里那句 `_wait_status({"TARGET_FOUND"})`
紧接着读总线，撞进这个窗口就断在 `:121`。

**这条脚本是那条结论的分辨力证明**（见 memory: calibrate-before-trusting-a-negative）：
  - `new` = 当前 HEAD 的次序（apply 先）⇒ 应挂
  - `old` = 2026-10-12 之前 / `main` 的次序（publish 先）⇒ 应全绿
两者只差一行次序，别的什么都不动；若 `old` 也挂，说明窗口另有来源、结论不成立。

**不改仓库文件**：`old` 那一臂在进程内 monkeypatch，跑完即弃。

用法：
  env -u PYTHONNOUSERSITE .venv/bin/python3 scripts/go2w/probe_store_bus_ordering.py new 300
  env -u PYTHONNOUSERSITE .venv/bin/python3 scripts/go2w/probe_store_bus_ordering.py old 300
退出码：0 = 跑完（挂不挂看打印的那一行）
"""
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import time

REPO = subprocess.run(
    ["git", "rev-parse", "--show-toplevel"],
    cwd=os.path.dirname(os.path.abspath(__file__)),
    capture_output=True, text=True, check=True,
).stdout.strip()
sys.path.insert(0, REPO)

from app.live_robot.explorer_search_adapter import ExplorerSearchAdapter  # noqa: E402
from app.manual_web_demo.control_ownership import ControlOwner  # noqa: E402
from app.manual_web_demo.search_models import SearchStartRequest  # noqa: E402
from app.manual_web_demo.search_session_service import (  # noqa: E402
    SearchSessionService,
    make_mock_executor_factory,
)

ORDER = (sys.argv[1] if len(sys.argv) > 1 else "new").lower()
N = int(sys.argv[2]) if len(sys.argv) > 2 else 300
if ORDER not in ("new", "old"):
    print("用法: probe_store_bus_ordering.py {new|old} [次数]", file=sys.stderr)
    raise SystemExit(2)


def _on_explorer_event_old(self, event):
    """publish 在 apply 之前 —— 2026-10-12 之前的次序（`main` 仍是这个）。"""
    for search_event in self._convert(event):
        self._bus.publish(search_event)
        self._store.apply(search_event)


if ORDER == "old":
    ExplorerSearchAdapter.on_explorer_event = _on_explorer_event_old


def _wait_status(service, statuses, timeout=60.0):
    """照抄测试自己的读法：等状态（不是等事件）。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        session = service.current_session()
        if (session.status if session else "IDLE") in statuses:
            return
        time.sleep(0.05)
    raise AssertionError("等状态超时")


def one_run() -> bool:
    service = SearchSessionService(
        owner=ControlOwner(),
        executor_factory=make_mock_executor_factory(
            scenario="anchor_then_target", mock_target_after=3, confirm_after_seen=1),
        session_dir=tempfile.mkdtemp(),
    )
    result = service.start_search(SearchStartRequest.from_dict({
        "target": "饮水机旁边的蓝色垃圾桶", "backend": "mock",
        "finish_on_visual_confirmation": True,
    }))
    assert result["ok"] is True
    _wait_status(service, {"TARGET_FOUND"})
    types = [item["event_type"] for item in service.recent_events(1000)]
    seen = "TARGET_CONFIRMED" in types
    service.shutdown()
    return seen


def main() -> int:
    fails = sum(0 if one_run() else 1 for _ in range(N))
    print(f"order={ORDER} : {fails}/{N} failed "
          f"({'MISS TARGET_CONFIRMED' if fails else 'all ok'})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
