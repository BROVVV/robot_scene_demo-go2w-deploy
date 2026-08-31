"""Plan §36: Web topology renderer layout test.

Runs a headless Node unit test against the pure ``computeTopologyLayout`` /
``topologyFingerprint`` helpers exported by search_map.js (no DOM, no browser).
These helpers must be deterministic and must never place two nodes on the same
spot.
"""

from __future__ import annotations

import shutil
import subprocess
from pathlib import Path

import pytest

NODE = shutil.which("node")
JS_TEST = Path(__file__).with_name("test_search_topology_layout.js")


@pytest.mark.skipif(NODE is None, reason="node.js not available")
def test_topology_layout_js_helpers():
    result = subprocess.run(
        [NODE, str(JS_TEST)],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert result.returncode == 0, (
        f"node layout test failed:\n{result.stdout}\n{result.stderr}"
    )
    assert "ALL JS LAYOUT TESTS PASSED" in result.stdout
