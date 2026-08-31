"""Plan §36-style: WebUI 地图滚轮缩放/拖拽平移纯函数测试。

加载 search_map.js 暴露的 window.SvgViewport（zoom/pan/clamp），验证：
  * 滚轮以光标为锚缩放（世界坐标不变）
  * 拖拽平移换算正确
  * 缩放范围 clamp
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

NODE = shutil.which("node")
JS_TEST = Path(__file__).with_name("test_search_map_viewport.js")


@pytest.mark.skipif(NODE is None, reason="node.js not available")
def test_map_viewport_js_helpers():
    result = subprocess.run(
        [NODE, str(JS_TEST)],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, f"viewport test failed:\n{result.stdout}\n{result.stderr}"
    assert "ALL VIEWPORT TESTS PASSED" in result.stdout
