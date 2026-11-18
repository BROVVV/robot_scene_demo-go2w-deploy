"""The gateway's emergency latch must mirror the real action server after re-arm."""

from types import SimpleNamespace
from pathlib import Path
import sys

import pytest

# ROS Humble uses system Python 3.10; the application venv uses Python 3.13.
# Execute this module separately in the sourced ROS environment as well.
pytest.importorskip('rclpy', reason='requires ROS Python; run with sourced /usr/bin/python3')

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/go2w'))
from motion_local_gateway import MotionLocalGateway


class Future:
    def __init__(self, response):
        self.response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self.response


class ArmClient:
    def __init__(self, success):
        self.success = success

    def service_is_ready(self):
        return True

    def call_async(self, _request):
        return Future(SimpleNamespace(success=self.success, message="arm result"))


class Request:
    def __init__(self, armed):
        self.payload = {"request": {"data": armed}}
        self.response = None

    def finish(self, response):
        self.response = response


def arm_with_latched_estop(*, success, armed):
    gateway = MotionLocalGateway.__new__(MotionLocalGateway)
    gateway._arm_client = ArmClient(success)
    gateway._arm_timeout_sec = 60.0
    gateway._armed_until_monotonic = 0.0
    gateway._estop_latched = True
    request = Request(armed)
    gateway._start_arm(request)
    return gateway, request


def test_successful_rearm_clears_gateway_estop_latch():
    gateway, request = arm_with_latched_estop(success=True, armed=True)
    assert request.response["success"] is True
    assert gateway._armed_until_monotonic > 0
    assert gateway._estop_latched is False


def test_rejected_rearm_does_not_clear_gateway_estop_latch():
    gateway, request = arm_with_latched_estop(success=False, armed=True)
    assert request.response["success"] is False
    assert gateway._armed_until_monotonic == 0.0
    assert gateway._estop_latched is True


def test_disarm_does_not_clear_gateway_estop_latch():
    gateway, request = arm_with_latched_estop(success=True, armed=False)
    assert request.response["success"] is True
    assert gateway._estop_latched is True
