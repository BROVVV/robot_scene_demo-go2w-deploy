#!/usr/bin/env python3
"""Read-only SportModeState observer. This module never creates a publisher."""

import json
import sys
import time

import rclpy
from rclpy.node import Node
from unitree_go.msg import SportModeState


class StateObserver(Node):
    def __init__(self, topic: str, output_path: str) -> None:
        super().__init__("go2w_state_observer")
        self.last_message = time.monotonic()
        self.output = open(output_path, "a", encoding="utf-8")
        self.subscription = self.create_subscription(
            SportModeState, topic, self.on_state, 10
        )

    def on_state(self, msg: SportModeState) -> None:
        self.last_message = time.monotonic()
        record = {
            "time": time.time(),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "yaw_speed": float(msg.yaw_speed),
        }
        self.output.write(json.dumps(record, ensure_ascii=False) + "\n")
        self.output.flush()

    def close(self) -> None:
        self.output.close()


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: verify_motion.py TOPIC OUTPUT.jsonl", file=sys.stderr)
        return 2
    rclpy.init()
    node = StateObserver(sys.argv[1], sys.argv[2])
    deadline = time.monotonic() + 5.0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.last_message > deadline - 5.0:
                deadline = time.monotonic() + 5.0
            if time.monotonic() > deadline:
                print("no SportModeState message for 5 seconds", file=sys.stderr)
                return 1
    except KeyboardInterrupt:
        return 0
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
