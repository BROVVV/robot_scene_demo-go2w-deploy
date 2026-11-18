"""Gate final collision output independently of blocking sensor/TF processing."""
import json
import math
import time
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Align the final guard with the measured 15 W acquisition/receive latency.
# Collision Monitor independently remains fail-closed at its 0.30 s source
# timeout, and the fault latch below is unchanged.
SCAN_RECEIVE_TIMEOUT_SECONDS = .30
SCAN_STAMP_TIMEOUT_SECONDS = .40


def guard_reason(*, raw_age, command_age, scan_age, scan_receive_age, localization_ok,
                 linear, angular, raw_stopped=False):
    if not all(math.isfinite(v) for v in (linear, angular)):
        return 'nonfinite_command'
    if not 0 <= raw_age <= .3:
        return 'controller_command_stale'
    if raw_stopped:
        # An explicit controller stop must overrule the smoother's old tail.
        # Otherwise cancel can retain nonzero velocity until its .3s timeout
        # plus downstream scheduling, even after Nav2 acknowledged cancel.
        return 'controller_stop'
    if not 0 <= command_age <= .3:
        return 'smoothed_command_stale'
    if (not -.1 <= scan_age <= SCAN_STAMP_TIMEOUT_SECONDS
            or not 0 <= scan_receive_age <= SCAN_RECEIVE_TIMEOUT_SECONDS):
        return 'scan_stale'
    if not localization_ok:
        return 'localization_stale'
    return ''


class FaultStopLatch:
    """A running command cannot resume merely because a failed input recovers.

    A controller zero ends the old velocity intent. The next nonzero command
    can then start a new attempt through all the usual freshness checks.
    Before the first allowed motion, missing startup inputs simply block.
    """
    def __init__(self):
        self.moved = False
        self.fault = ''

    def controller_stop(self):
        self.moved = False
        self.fault = ''

    def apply(self, reason):
        if self.fault:
            return self.fault + '_waiting_controller_stop'
        if reason and reason != 'controller_stop' and self.moved:
            self.fault = reason
        return reason

    def allowed_output(self, linear, angular):
        if abs(linear) + abs(angular) >= 1e-9:
            self.moved = True


class NavigationVelocityGuard(Node):
    def __init__(self):
        super().__init__('go2w_navigation_velocity_guard')
        for name, value in [('input_topic','/cmd_vel_nav_collision_checked'),('raw_topic','/cmd_vel_nav_raw'),
                            ('output_topic','/go2w/nav2_cmd_vel'),('scan_topic','/go2w/lidar/scan'),
                            ('require_localization', True)]:
            self.declare_parameter(name,value)
        self.command=Twist();self.raw_at=self.command_at=self.scan_at=0.;self.scan_stamp=0.
        self.raw_stopped=True
        self.stop_latch=FaultStopLatch()
        self.status_valid=False;self.status_stamp=0.
        self.pub=self.create_publisher(Twist,self.get_parameter('output_topic').value,10)
        self.status=self.create_publisher(String,'~/status',10)
        self.create_subscription(Twist,self.get_parameter('input_topic').value,self.on_command,1)
        self.create_subscription(Twist,self.get_parameter('raw_topic').value,self.on_raw,1)
        self.create_subscription(LaserScan,self.get_parameter('scan_topic').value,self.on_scan,
                                 QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String,'/go2w/nav/localization_status',self.on_status,1)
        self.create_timer(.05,self.tick)

    def on_command(self,message):
        self.command,self.command_at=message,time.monotonic()

    def on_raw(self,message):
        self.raw_at=time.monotonic()
        self.raw_stopped=abs(message.linear.x)+abs(message.angular.z)<1e-9
        if self.raw_stopped:
            self.stop_latch.controller_stop()

    def on_scan(self,message):
        if message.header.frame_id=='base_link' and len(message.ranges):
            self.scan_at=time.monotonic()
            self.scan_stamp=message.header.stamp.sec+message.header.stamp.nanosec*1e-9

    def on_status(self,message):
        try:
            payload=json.loads(message.data)
            self.status_valid=payload.get('valid') is True
            self.status_stamp=float(payload['wall_time'])
        except (ValueError,KeyError,TypeError):
            self.status_valid=False

    def tick(self):
        now=time.monotonic()
        localization_ok=(self.status_valid and -.1 <= time.time()-self.status_stamp <= .3
                         or not self.get_parameter('require_localization').value)
        reason=guard_reason(raw_age=now-self.raw_at,command_age=now-self.command_at,
                            scan_age=self.get_clock().now().nanoseconds/1e9-self.scan_stamp,
                            scan_receive_age=now-self.scan_at,localization_ok=localization_ok,
                            linear=self.command.linear.x,angular=self.command.angular.z,
                            raw_stopped=self.raw_stopped)
        reason=self.stop_latch.apply(reason)
        output=Twist()
        if not reason:
            output.linear.x=self.command.linear.x
            output.angular.z=self.command.angular.z
            self.stop_latch.allowed_output(output.linear.x,output.angular.z)
        self.pub.publish(output)
        self.status.publish(String(data=reason or 'allowed'))


def main(args=None):
    rclpy.init(args=args);node=NavigationVelocityGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # ros2 launch can invalidate the context before the Python process
        # reaches finally.  Publishing through that dead context raises and
        # used to turn every clean isolated-chain shutdown into exit code 1.
        if rclpy.ok():
            node.pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
