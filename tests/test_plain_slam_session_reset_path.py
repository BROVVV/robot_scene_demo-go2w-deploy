"""§9.4 恢复路径回归：mapping session 复位不许在第一次就被吞掉。

两个缺陷都是在离线全链路（假 plain_slam 源 + 真 relay + 真桥）里实测出来的：

1) `setup_environment.sh` 在网卡不存在时被 `pipefail` 静默打断，
   `reset_plain_slam_session.sh` 一行输出都没有，操作员看不到任何原因；
2) 桥第一次看到 reset marker 只记 mtime、不复位，
   于是机器狗上「第一次执行 reset 脚本」永远无效，地图冻结解不掉。
"""

from __future__ import annotations

import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BRIDGE = ROOT / "scripts/go2w/plain_slam_web_bridge.py"


def test_setup_environment_reports_error_instead_of_silent_abort(tmp_path: Path) -> None:
    stub = tmp_path / "bin"
    stub.mkdir()
    (stub / "ip").write_text("#!/bin/sh\nexit 1\n", encoding="utf-8")
    (stub / "ip").chmod(0o755)
    env = dict(os.environ)
    env["PATH"] = f"{stub}:{env['PATH']}"
    env.pop("GO2W_HOST_IP", None)
    env["GO2W_INTERFACE"] = "nic_that_does_not_exist"
    result = subprocess.run(
        ["bash", "-c", "set -euo pipefail; source scripts/go2w/setup_environment.sh"],
        cwd=ROOT, env=env, capture_output=True, text=True, timeout=60,
    )
    # 必须是那条明确的 ERROR + 退出码 2，而不是无输出的 rc=1。
    assert result.returncode == 2, result.stderr
    assert "cannot resolve a 192.168.123.x host address" in result.stderr


def test_reset_marker_first_touch_starts_a_new_session() -> None:
    text = BRIDGE.read_text(encoding="utf-8")
    # 基线在启动时就取（文件不存在算 0.0），第一次 touch 就大于基线。
    assert "self._reset_marker_mtime = self._reset_marker_stamp()" in text
    assert "if mtime > self._reset_marker_mtime:" in text
    # 旧的 “第一次只记 mtime” 写法必须消失。
    assert "if self._reset_marker_mtime is not None and mtime >" not in text
