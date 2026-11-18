#!/usr/bin/env python3
"""Write map identity for semantic objects without a Web point-cloud snapshot."""

import argparse
import json
import os
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class MapEpochMetaBridge(Node):
    def __init__(self, output: Path):
        super().__init__("go2w_map_epoch_meta_bridge")
        self.output = output
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/go2w/slam/keyframe_graph", self.on_graph, qos
        )

    def on_graph(self, msg: String) -> None:
        try:
            graph = json.loads(msg.data)
            epoch = str(graph.get("map_epoch") or "")
            frame = str(graph.get("frame_id") or "")
            if not epoch or not frame:
                return
            payload = {
                "map_epoch": epoch,
                "frame_id": frame,
                "map_revision": int(graph.get("graph_revision") or 0),
                "generated_at": self.get_clock().now().nanoseconds / 1e9,
            }
            self.output.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.output.with_suffix(self.output.suffix + ".tmp")
            temporary.write_text(json.dumps(payload) + "\n", encoding="utf-8")
            os.replace(temporary, self.output)
        except (ValueError, TypeError, OSError) as exc:
            self.get_logger().warning("map metadata update rejected: %s" % exc)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--meta-output", type=Path, required=True)
    args = parser.parse_args()
    rclpy.init()
    node = MapEpochMetaBridge(args.meta_output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
