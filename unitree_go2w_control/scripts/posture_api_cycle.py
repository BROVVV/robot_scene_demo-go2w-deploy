#!/usr/bin/env python3
"""Response-checked Go2/Go2-W high-level posture cycle; never publishes LowCmd."""

import signal
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from unitree_api.msg import Request, Response
from unitree_go.msg import SportModeState


API_STOP = 1003
API_STAND_UP = 1004
API_STAND_DOWN = 1005


class PostureClient(Node):
    def __init__(self) -> None:
        super().__init__("go2w_response_checked_posture_client")
        self.publisher = self.create_publisher(Request, "/api/sport/request", 10)
        self.response_sub = self.create_subscription(
            Response, "/api/sport/response", self._on_response, 10
        )
        self.state_sub = self.create_subscription(
            SportModeState, "/lf/sportmodestate", self._on_state, 10
        )
        self.state: Optional[SportModeState] = None
        self.responses: dict[int, Response] = {}

    def _on_response(self, response: Response) -> None:
        self.responses[response.header.identity.id] = response

    def _on_state(self, state: SportModeState) -> None:
        self.state = state

    def spin_until(self, predicate, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def require_stationary_state(self, timeout: float = 5.0) -> SportModeState:
        if not self.spin_until(lambda: self.state is not None, timeout):
            raise RuntimeError("no SportModeState received")
        assert self.state is not None
        vx, vy, vz = (float(v) for v in self.state.velocity)
        yaw = float(self.state.yaw_speed)
        print(
            f"STATE mode={self.state.mode} velocity=[{vx:.4f},{vy:.4f},{vz:.4f}] "
            f"yaw_speed={yaw:.4f}",
            flush=True,
        )
        if max(abs(vx), abs(vy), abs(vz), abs(yaw)) > 0.05:
            raise RuntimeError("robot is not stationary")
        return self.state

    def call(self, api_id: int, name: str, timeout: float = 5.0) -> None:
        request_id = time.time_ns()
        request = Request()
        request.header.identity.id = request_id
        request.header.identity.api_id = api_id
        self.publisher.publish(request)
        print(f"REQUEST name={name} id={request_id} api_id={api_id}", flush=True)
        if not self.spin_until(lambda: request_id in self.responses, timeout):
            raise RuntimeError(f"{name} response timeout")
        response = self.responses.pop(request_id)
        code = int(response.header.status.code)
        print(f"RESPONSE name={name} status_code={code}", flush=True)
        if code != 0:
            raise RuntimeError(f"{name} rejected with status {code}")

    def wait_mode(self, expected: int, name: str, timeout: float = 15.0) -> None:
        if not self.spin_until(
            lambda: self.state is not None and int(self.state.mode) == expected,
            timeout,
        ):
            actual = "none" if self.state is None else str(int(self.state.mode))
            raise RuntimeError(f"{name} mode timeout: expected {expected}, actual {actual}")
        print(f"MODE name={name} reached={expected}", flush=True)
        self.require_stationary_state()


def main() -> int:
    rclpy.init()
    node = PostureClient()
    stand_down_sent = False
    stand_up_reached = False
    stopping = False

    def request_shutdown(_signum, _frame) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_shutdown)
    signal.signal(signal.SIGTERM, request_shutdown)

    try:
        initial = node.require_stationary_state()
        if int(initial.mode) != 1:
            raise RuntimeError(f"expected initial stand mode 1, got {initial.mode}")
        node.call(API_STAND_DOWN, "STAND_DOWN")
        stand_down_sent = True
        node.wait_mode(5, "LIE_DOWN")
        time.sleep(2.0)
        if stopping:
            raise RuntimeError("interrupted after lie down")
        node.call(API_STAND_UP, "STAND_UP")
        node.wait_mode(1, "STAND_UP")
        stand_up_reached = True
        node.call(API_STOP, "STOP_MOVE")
        node.require_stationary_state()
        print("RESULT posture_cycle=PASS final_stop=PASS", flush=True)
        return 0
    except Exception as exc:  # safety path must still run
        print(f"ERROR {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if stand_down_sent and not stand_up_reached:
            try:
                node.call(API_STAND_UP, "SAFETY_STAND_UP")
                node.wait_mode(1, "SAFETY_STAND_UP", timeout=15.0)
            except Exception as exc:
                print(f"SAFETY_ERROR stand_up: {exc}", file=sys.stderr, flush=True)
        try:
            node.call(API_STOP, "FINAL_STOP")
        except Exception as exc:
            print(f"SAFETY_ERROR final_stop: {exc}", file=sys.stderr, flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
