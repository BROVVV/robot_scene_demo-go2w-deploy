# Copyright 2026 robot_scene_demo maintainers

"""端到端闸门契约：快照里的 mapping_health 必须真的拦住 /api/search/start。

计划书 §13.4 的健康门在接线前是**假绿**：``web_server.py:377`` 判
``slam.get("mapping_health") != "DEGRADED_LIO_DRIFT"``，而真正在写快照的
``scripts/go2w/webui_map_snapshot_writer.py`` 根本不写这个键 —— 于是
``None != "DEGRADED_LIO_DRIFT"`` 恒成立，``slam_map_not_drifting`` 恒真，
``search_routes.py:118`` 那条 409 ``MAPPING_FROZEN`` **一次都没响过**。

本文件盯的不是判据本身（那在 ``test_lio_nofix_gate_calibration.py``），而是
**判据写出来的那个键，能不能走到闸门上**。中间隔着三层：写端 payload →
``slam_map_snapshot.load_slam_map_snapshot`` → ``web_server.search_readiness``
→ ``search_routes`` 的 409。任何一层把键吃掉，这里必须响。

第 3 条（409）是这个文件存在的理由：没有它，验收就退化成"没检出 = 没问题"。

**注入校准（2026-09-16 实做）**：把 ``web_server.py:377`` 改回
``"slam_map_not_drifting": True`` 后，``test_degraded_...`` 与
``test_degraded_payload_blocks_search_start_with_409`` 立刻失败；还原后全绿。
"""

from __future__ import annotations

import json
import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable

from fastapi.testclient import TestClient

from app.manual_web_demo.config import ManualDemoSettings
from app.manual_web_demo.search_session_service import make_mock_executor_factory
from app.manual_web_demo.web_server import DemoRuntime, create_app

DEGRADED = "DEGRADED_LIO_DRIFT"
# 逐字取自 webui_map_snapshot_writer.py 的 health_reason（判据模块 _describe 的产出）
REASON = "激光修正停摆 10 帧（iter=10 fix_t=0，阈值 10 帧） v=0.00 act=0.00 pos=(-0.35,0.62,0.02)"


class FakeWorker:
    """Same shape as tests/test_autonomous_search_web_routes.py::FakeWorker."""

    def __init__(self) -> None:
        self._cb: Callable[[str, dict[str, Any]], None] | None = None
        self._status = {
            "state": "ready", "motion_available": True, "robot_mode": 1,
            "robot_error_code": 0, "state_fresh": True, "lease_alive": True,
            "lidar_fresh": True, "front_clearance_m": 1.0,
            "left_clearance_m": 1.0, "right_clearance_m": 1.0,
            "rotation_clearance_valid": True, "last_error": None,
            "odom_frame": "odom", "odom_pose": [0.0, 0.0, 0.0],
        }

    def set_on_message(self, cb): self._cb = cb
    def status(self): return dict(self._status)
    def alive(self): return True
    def start(self): pass
    def stop(self): pass
    def request_pulse(self, direction): pass
    def request_stop(self): pass
    def request_estop(self): pass
    def request_status(self):
        # camera_available 是 blocking 项，只能由这条回调喂出来
        if self._cb is not None:
            self._cb("camera_status", {"available": True, "width": 1920, "height": 1080})
    def shutdown(self): pass


def _snapshot_path(tmp_path: Path) -> Path:
    return tmp_path / "slam_map_3d.json"


def _write_snapshot(tmp_path: Path, **overrides: Any) -> Path:
    """写一份**瘦写端能写出的**快照：健康字段是叠加进去的，不是必有。

    刻意不带 mapping_health / health_reason / lio_pose_valid 三个键的默认值 ——
    "第三态"就是这么来的（判据没在跑时写端**不写**这些键）。
    """
    payload: dict[str, Any] = {
        "schema_version": "go2w_slam_web_cloud_v2",
        "available": True,
        "points": [[0.0, 0.0, 0.0]],
        "mapping_mode": "mapping_assist",
        "motion_authorized": False,
        "safety_authorized": False,
        "canonical_frame": "pslam_map",
        "permanent_source": "/go2w/slam/map_3d",
        "mapping_session_id": 0,
        "generated_at": time.time(),
    }
    payload.update(overrides)
    path = _snapshot_path(tmp_path)
    path.write_text(json.dumps(payload), encoding="utf-8")
    return path


def make_app(tmp_path: Path) -> TestClient:
    config = replace(
        ManualDemoSettings(),
        runtime_dir=str(tmp_path / "runtime"),
        logs_dir=str(tmp_path / "logs"),
        analysis_frames_dir=str(tmp_path / "frames"),
        # 绝对路径：config.slam_map_snapshot_path 对相对路径会拼 _PROJECT_ROOT，
        # 那会把测试指到真机的 outputs/ 上。
        slam_map_snapshot=str(_snapshot_path(tmp_path)),
    )
    runtime = DemoRuntime(
        config,
        worker=FakeWorker(),
        camera_fresh=lambda: True,
        search_executor_factory=make_mock_executor_factory(),
        search_session_dir=str(tmp_path / "sessions"),
    )
    return TestClient(create_app(config=config, runtime=runtime))


# --------------------------------------------------------------------------- #
# 1-2. 就绪页必须说真话
# --------------------------------------------------------------------------- #

def test_healthy_payload_reports_healthy_and_does_not_block(tmp_path) -> None:
    _write_snapshot(tmp_path, mapping_health="HEALTHY", lio_pose_valid=True)
    with make_app(tmp_path) as client:
        readiness = client.get("/api/search/readiness").json()

    assert readiness["checks"]["slam_map_not_drifting"] is True
    assert readiness["mapping_health"] == "HEALTHY"
    assert readiness["mapping_health_reason"] == ""
    assert "slam_map_not_drifting" not in readiness["blocking"]
    assert readiness["ready"] is True


def test_degraded_payload_blocks_and_carries_the_reason(tmp_path) -> None:
    _write_snapshot(
        tmp_path,
        mapping_health=DEGRADED,
        lio_pose_valid=False,
        health_reason=REASON,
    )
    with make_app(tmp_path) as client:
        readiness = client.get("/api/search/readiness").json()

    assert readiness["checks"]["slam_map_not_drifting"] is False
    assert readiness["mapping_health"] == DEGRADED
    assert readiness["mapping_health_reason"] == REASON
    assert readiness["ready"] is False
    # 只该拦这一项：地图冻结不该顺带把相机/worker 也报成故障
    assert readiness["blocking"] == ["slam_map_not_drifting"]
    # 操作员要在就绪页上看见**为什么**被拦，不能只有一个 False
    assert REASON in readiness["reason"]


# --------------------------------------------------------------------------- #
# 3. 409 —— 这个文件存在的理由
# --------------------------------------------------------------------------- #

def test_degraded_payload_blocks_search_start_with_409(tmp_path) -> None:
    """发散时 /api/search/start 必须 409 且带中文原因。

    没有这一条，整件事的验收就是"没检出 = 没问题"：判据、键、就绪页全对，
    但真正会动的那个接口照样放行。
    """
    _write_snapshot(
        tmp_path,
        mapping_health=DEGRADED,
        lio_pose_valid=False,
        health_reason=REASON,
    )
    with make_app(tmp_path) as client:
        response = client.post(
            "/api/search/start", json={"target": "蓝色垃圾桶", "backend": "mock"}
        )
        # 拦在 search_routes.py:115-129，service.start_search 根本没被调用
        status = client.get("/api/search/state").json()["status"]

    assert response.status_code == 409
    body = response.json()
    assert body["ok"] is False
    detail = body["error_detail"]
    assert detail["code"] == "MAPPING_FROZEN"
    assert detail["category"] == "spatial"
    assert REASON in detail["message"], "409 报文里必须带上写端给的原因"
    assert detail["suggestion"], "报错要给恢复路径（reset_plain_slam_session.sh）"
    assert status == "IDLE", "被 409 拦下时不得留下半个搜索会话"


# --------------------------------------------------------------------------- #
# 4-5. 第三态：判据没在跑 ≠ 发散，也 ≠ 绿
# --------------------------------------------------------------------------- #

def test_absent_health_keys_are_unavailable_and_do_not_block(tmp_path) -> None:
    """判据没跑时写端**不写**这三个键 —— 必须表现为 UNAVAILABLE 且放行。

    这是设计上的第三态：挡住它的代价是"日志路径一变就再也开不了搜索"，
    而假的 HEALTHY 会让绿灯说谎。所以只允许"不绿 + 不拦"。
    """
    _write_snapshot(tmp_path)  # 不带任何健康字段
    with make_app(tmp_path) as client:
        readiness = client.get("/api/search/readiness").json()

    assert readiness["mapping_health"] == "UNAVAILABLE"
    assert readiness["mapping_health_reason"] == ""
    assert readiness["checks"]["slam_map_not_drifting"] is True
    assert "slam_map_not_drifting" not in readiness["blocking"]
    assert readiness["ready"] is True


def test_missing_snapshot_does_not_block(tmp_path) -> None:
    """快照还没生成也不拦：遥控建图必须始终可用（§10.2 的原话）。"""
    with make_app(tmp_path) as client:
        readiness = client.get("/api/search/readiness").json()

    assert readiness["mapping_health"] == "UNAVAILABLE"
    assert readiness["checks"]["slam_map_not_drifting"] is True
    assert readiness["checks"]["slam_map_live"] is False, "地图没来该降级，但不该拦"
    assert "slam_map_not_drifting" not in readiness["blocking"]
    assert "slam_map_live" in readiness["degraded"]
    assert readiness["ready"] is True
