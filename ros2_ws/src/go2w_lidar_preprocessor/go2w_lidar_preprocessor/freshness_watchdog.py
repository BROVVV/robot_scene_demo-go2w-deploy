"""Independent scan heartbeat watchdog; a hung preprocessor cannot retain true."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from .preprocess_core import sensor_header_fresh

SCAN_RECEIVE_TIMEOUT_SECONDS = .25
SCAN_STAMP_TIMEOUT_SECONDS = .32


class LidarFreshnessWatchdog(Node):
    def __init__(self):
        super().__init__('go2w_lidar_freshness_watchdog')
        self.last = None
        self.received = 0.
        self.publisher = self.create_publisher(Bool, '/go2w/safety/lidar_fresh', 10)
        self.create_subscription(LaserScan, '/go2w/lidar/scan', self.scan,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_timer(.05, self.tick)

    def scan(self, message):
        self.last = message
        self.received = time.monotonic()

    def tick(self):
        message = self.last
        valid = bool(message and message.header.frame_id == 'base_link' and len(message.ranges)
                     and time.monotonic()-self.received <= SCAN_RECEIVE_TIMEOUT_SECONDS
                     and sensor_header_fresh(now_ns=self.get_clock().now().nanoseconds,
                         stamp_sec=message.header.stamp.sec, stamp_nanosec=message.header.stamp.nanosec,
                         timeout_seconds=SCAN_STAMP_TIMEOUT_SECONDS,
                         future_tolerance_seconds=.1))
        self.publisher.publish(Bool(data=valid))


def main(args=None):
    rclpy.init(args=args);node=LidarFreshnessWatchdog()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
