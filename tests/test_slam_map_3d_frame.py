"""WebUI 三维地图画幅纯函数测试（同 test_search_map_viewport.py 的做法）。

回归背景（2026-09-11）：odom 发散把持久化轨迹污染成 ±1e20，写入器据此算出的
focus_bounds 撑到 1e19，前端 span≈1.7e20 —— 地图 scale≈0 缩成一个像素（用户报
"webui 里面没有地图"），且 drawGrid 的网格循环要迭代 3.5e19 次把浏览器主线程
永久卡死。这条用例锁住 saneFrame 这道闸。
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

NODE = shutil.which("node")
JS_TEST = Path(__file__).with_name("test_slam_map_3d_frame.js")


@pytest.mark.skipif(NODE is None, reason="node.js not available")
def test_slam3d_frame_js_helpers():
    result = subprocess.run(
        [NODE, str(JS_TEST)],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, f"frame test failed:\n{result.stdout}\n{result.stderr}"
    assert "ALL SLAM3D FRAME TESTS PASSED" in result.stdout
