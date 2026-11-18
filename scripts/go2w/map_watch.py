#!/usr/bin/env python3
"""真机建图实时监视：周期性抓取两条链路的 map_3d，渲染并记录形状指标。

用途：大场地/走出房间的真机测试中，随时判断"地图有没有变成圆形/变形"，
不必等测试结束才发现问题。输出 /tmp/map_watch_latest.png 与 /tmp/map_watch.log。
"""

from __future__ import annotations

import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402

TOPICS = ["/go2w/slam/map_3d", "/go2w/rko_slam/map_3d"]
PNG = "/tmp/map_watch_latest.png"
LOG = "/tmp/map_watch.log"


def latched(depth: int = 1) -> QoSProfile:
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def best_effort(depth: int = 1) -> QoSProfile:
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.VOLATILE)


class MapWatch(Node):
    def __init__(self) -> None:
        super().__init__("map_watch")
        self.frames: dict = {}
        self.counts: dict = {}
        for topic in TOPICS:
            self.create_subscription(PointCloud2, topic,
                                     lambda msg, t=topic: self._on(msg, t), latched(1))
            self.create_subscription(PointCloud2, topic,
                                     lambda msg, t=topic: self._on(msg, t), best_effort(1))
        self.create_timer(30.0, self._render)
        self.get_logger().info("map_watch up (every 30 s)")

    def _on(self, message: PointCloud2, topic: str) -> None:
        self.frames[topic] = message
        self.counts[topic] = self.counts.get(topic, 0) + 1

    @staticmethod
    def _shape(points: np.ndarray) -> dict:
        span = points.max(axis=0) - points.min(axis=0)
        aspect = float(span[0] / span[1]) if span[1] > 1e-6 else float("inf")
        # 用 2D 占据栅格估"填充率"：圆形 ≈0.79，细长房间明显更低
        cells = 60
        histogram, _, _ = np.histogram2d(points[:, 0], points[:, 1],
                                         bins=cells)
        fill = float(np.count_nonzero(histogram)) / cells ** 2
        return {"span_x": float(span[0]), "span_y": float(span[1]),
                "aspect": aspect, "fill_ratio": fill}

    def _render(self) -> None:
        rows = []
        figure, axes = plt.subplots(1, len(TOPICS), figsize=(7 * len(TOPICS), 6))
        for axis, topic in zip(np.atleast_1d(axes), TOPICS):
            message = self.frames.get(topic)
            if message is None:
                axis.set_title(f"{topic}\n(no data yet)")
                rows.append(f"{topic}: no data")
                continue
            count = int(message.width) * int(message.height)
            raw = np.frombuffer(message.data, dtype=np.uint8).reshape(
                count, int(message.point_step))
            points = np.ascontiguousarray(raw[:, 0:12]).view(np.float32).reshape(-1, 3)
            shape = self._shape(points)
            axis.scatter(points[:, 0], points[:, 1], s=0.3, c=points[:, 2],
                         cmap="viridis", linewidths=0)
            axis.set_title(f"{topic}\nn={len(points)} aspect={shape['aspect']:.2f} "
                           f"fill={shape['fill_ratio']:.2f}")
            axis.set_xlabel("x [m]")
            axis.set_ylabel("y [m]")
            axis.grid(alpha=0.3)
            axis.set_aspect("equal", adjustable="datalim")
            rows.append(f"{topic}: n={len(points)} xspan={shape['span_x']:.1f} "
                        f"yspan={shape['span_y']:.1f} aspect={shape['aspect']:.2f} "
                        f"fill={shape['fill_ratio']:.2f}")
        figure.suptitle(time.strftime("%H:%M:%S"))
        figure.tight_layout()
        figure.savefig(PNG, dpi=110)
        plt.close(figure)
        with open(LOG, "a", encoding="utf-8") as handle:
            handle.write(f"--- {time.strftime('%H:%M:%S')}\n")
            for row in rows:
                handle.write(f"{row}\n")


def main() -> int:
    rclpy.init()
    node = MapWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
