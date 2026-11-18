#!/usr/bin/python3
# Copyright 2026 robot_scene_demo maintainers

"""计划书 §6.2：把 plain_slam 地图与两路里程计接成 Nav2 要的唯一坐标链。

订阅：

* ``/go2w/slam/map_2d``（plain_slam 的占据栅格，坐标系是 ``pslam_odom``）；
* ``/go2w/slam/odom_base``（plain_slam 的 ``pslam_odom -> base_link`` 位姿）；
* ``/go2w/odom/fused``（运动栈的 ``odom_fused -> base_link`` 位姿）。

发布：

* ``/map``：内容沿用 plain_slam 地图，只把 ``header.frame_id`` 改成 ``map``；
* TF ``map -> odom_fused``；
* TF ``odom_fused -> base_link``；
* ``/go2w/nav/localization_valid``（``std_msgs/Bool``）。

二维变换固定按下式计算，只用 ``x``/``y``/``yaw``：

    T_map_odom = T_map_base_from_plain_slam * inverse(T_odom_base_from_fused_odom)

公式成立的前提是「两路里程计报的是同一个 ``base_link``」。plain_slam 那边报的是
``base_link_mapping_assist``，两者是同一台车的同一处机体点，所以这里直接相消。

本节点是 ``/map`` 与 ``odom_fused -> base_link`` 的**唯一**发布者。旧脚本
``plain_slam_map_relay.py`` 不得再与本节点同时启动（会让 ``/map`` 出现两个发布者）；
``sport_odom_bridge.py`` 不发布 TF，也不需要改。

纯几何与闸门逻辑放在模块顶层且**不导入 rclpy**，这样离线测试可以直接 import 本文件
（宿主机的测试环境没有 rclpy）。rclpy 只在 ``main()`` 里导入。
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Any, Iterable

# 计划书 §6.2 固定的闸门阈值。
DEFAULT_MAX_ODOM_AGE_S = 0.5
DEFAULT_MAX_FRAME_SKEW_S = 0.1

MAP_FRAME = "map"
ODOM_FRAME = "odom_fused"
BASE_FRAME = "base_link"

MAP_2D_TOPIC = "/go2w/slam/map_2d"
PLAIN_SLAM_ODOM_TOPIC = "/go2w/slam/odom_base"
FUSED_ODOM_TOPIC = "/go2w/odom/fused"
MAP_TOPIC = "/map"
LOCALIZATION_VALID_TOPIC = "/go2w/nav/localization_valid"

# 拒绝原因，全部是固定的短字符串，便于日志比对。
REASON_OK = "OK"
REASON_NO_MAP = "NO_MAP"
REASON_NO_PLAIN_SLAM_ODOM = "NO_PLAIN_SLAM_ODOM"
REASON_NO_FUSED_ODOM = "NO_FUSED_ODOM"
REASON_STALE_PLAIN_SLAM_ODOM = "STALE_PLAIN_SLAM_ODOM"
REASON_STALE_FUSED_ODOM = "STALE_FUSED_ODOM"
REASON_FRAME_SKEW = "FRAME_SKEW"
REASON_NONFINITE = "NONFINITE"
REASON_TF_NOT_PUBLISHED = "TF_NOT_PUBLISHED"


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose2D:
    """二维位姿；``yaw`` 用弧度。"""

    x: float
    y: float
    yaw: float

    def is_finite(self) -> bool:
        return math.isfinite(self.x) and math.isfinite(self.y) and math.isfinite(self.yaw)


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    """从四元数取 yaw。四元数只用于 ROS 消息转换，位姿本身仍走二维。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    """返回 ``(x, y, z, w)``。"""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def invert_pose2d(pose: Pose2D) -> Pose2D:
    cos_yaw = math.cos(pose.yaw)
    sin_yaw = math.sin(pose.yaw)
    return Pose2D(
        x=-(cos_yaw * pose.x + sin_yaw * pose.y),
        y=-(-sin_yaw * pose.x + cos_yaw * pose.y),
        yaw=normalize_angle(-pose.yaw),
    )


def compose_pose2d(first: Pose2D, second: Pose2D) -> Pose2D:
    """``first * second``，即 ``first`` 系到 ``second`` 子系的复合。"""
    cos_yaw = math.cos(first.yaw)
    sin_yaw = math.sin(first.yaw)
    return Pose2D(
        x=first.x + cos_yaw * second.x - sin_yaw * second.y,
        y=first.y + sin_yaw * second.x + cos_yaw * second.y,
        yaw=normalize_angle(first.yaw + second.yaw),
    )


def map_to_odom_2d(map_to_base: Pose2D, odom_to_base: Pose2D) -> Pose2D:
    """计划书 §6.2 的固定公式。"""
    return compose_pose2d(map_to_base, invert_pose2d(odom_to_base))


@dataclass(frozen=True)
class LocalizationVerdict:
    valid: bool
    reason: str

    def __bool__(self) -> bool:  # pragma: no cover - 方便断言
        return self.valid


@dataclass(frozen=True)
class BridgeComputation:
    """发布 TF 之前能确定的一切。``ok`` 为假时 ``map_to_odom`` 为 ``None``。"""

    ok: bool
    reason: str
    map_to_odom: Pose2D | None
    odom_to_base: Pose2D | None
    frame_skew_s: float | None
    plain_slam_age_s: float | None
    fused_age_s: float | None


def evaluate_localization(
    *,
    map_received: bool,
    plain_slam_pose: Pose2D | None,
    fused_pose: Pose2D | None,
    plain_slam_age_s: float | None,
    fused_age_s: float | None,
    frame_skew_s: float | None,
    tf_published: bool,
    max_odom_age_s: float = DEFAULT_MAX_ODOM_AGE_S,
    max_frame_skew_s: float = DEFAULT_MAX_FRAME_SKEW_S,
) -> LocalizationVerdict:
    """计划书 §6.2 的固定判定顺序；任一条件不满足就返回 false。

    绝不沿用上一次的 ``true``：本函数是纯函数，没有任何记忆。
    """
    if not map_received:
        return LocalizationVerdict(False, REASON_NO_MAP)
    if plain_slam_pose is None:
        return LocalizationVerdict(False, REASON_NO_PLAIN_SLAM_ODOM)
    if fused_pose is None:
        return LocalizationVerdict(False, REASON_NO_FUSED_ODOM)
    if plain_slam_age_s is None or plain_slam_age_s > max_odom_age_s:
        return LocalizationVerdict(False, REASON_STALE_PLAIN_SLAM_ODOM)
    if fused_age_s is None or fused_age_s > max_odom_age_s:
        return LocalizationVerdict(False, REASON_STALE_FUSED_ODOM)
    if frame_skew_s is None or abs(frame_skew_s) > max_frame_skew_s:
        return LocalizationVerdict(False, REASON_FRAME_SKEW)
    if not plain_slam_pose.is_finite() or not fused_pose.is_finite():
        return LocalizationVerdict(False, REASON_NONFINITE)
    if not tf_published:
        return LocalizationVerdict(False, REASON_TF_NOT_PUBLISHED)
    return LocalizationVerdict(True, REASON_OK)


class FrameBridgeCore:
    """两路里程计 + 地图的状态机，以及闸门判定。不含任何 ROS 调用。"""

    def __init__(
        self,
        *,
        max_odom_age_s: float = DEFAULT_MAX_ODOM_AGE_S,
        max_frame_skew_s: float = DEFAULT_MAX_FRAME_SKEW_S,
        buffer_size: int = 200,
    ) -> None:
        self._max_odom_age_s = float(max_odom_age_s)
        self._max_frame_skew_s = float(max_frame_skew_s)
        self._plain_slam: deque[tuple[float, Pose2D]] = deque(maxlen=buffer_size)
        self._fused: deque[tuple[float, Pose2D]] = deque(maxlen=buffer_size)
        self._map_received = False

    # ------------------------------------------------------------- 输入
    def note_map(self) -> None:
        self._map_received = True

    def note_plain_slam_odom(self, stamp_s: float, pose: Pose2D) -> None:
        self._plain_slam.append((float(stamp_s), pose))

    def note_fused_odom(self, stamp_s: float, pose: Pose2D) -> None:
        self._fused.append((float(stamp_s), pose))

    @property
    def map_received(self) -> bool:
        return self._map_received

    # ------------------------------------------------------------- 计算
    def compute(self, now_s: float) -> BridgeComputation:
        """算 ``map -> odom_fused``。

        「数据不超过 0.5 秒」看的是两路**各自最新**样本的新鲜度；「两帧时间差不
        超过 0.1 秒」看的是**实际参与计算的那两帧**。两者是两件事：如果拿配对帧
        的年龄去判新鲜度，一路断流时会把锅算到另一路头上。
        """
        if not self._map_received:
            return self._empty(REASON_NO_MAP)
        if not self._plain_slam:
            return self._empty(REASON_NO_PLAIN_SLAM_ODOM)
        if not self._fused:
            return self._empty(REASON_NO_FUSED_ODOM)

        plain_latest_s = self._plain_slam[-1][0]
        fused_latest_s = self._fused[-1][0]
        plain_age = float(now_s) - plain_latest_s
        fused_age = float(now_s) - fused_latest_s

        # 配对：以大滞后那一路的最新时刻为基准，两边各取时间上最近的样本。
        reference_s = min(plain_latest_s, fused_latest_s)
        plain_stamp, plain_pose = _nearest(self._plain_slam, reference_s)
        fused_stamp, fused_pose = _nearest(self._fused, reference_s)
        skew = plain_stamp - fused_stamp

        verdict = evaluate_localization(
            map_received=True,
            plain_slam_pose=plain_pose,
            fused_pose=fused_pose,
            plain_slam_age_s=plain_age,
            fused_age_s=fused_age,
            frame_skew_s=skew,
            tf_published=True,
            max_odom_age_s=self._max_odom_age_s,
            max_frame_skew_s=self._max_frame_skew_s,
        )
        if not verdict.valid:
            return BridgeComputation(
                ok=False,
                reason=verdict.reason,
                map_to_odom=None,
                odom_to_base=fused_pose,
                frame_skew_s=skew,
                plain_slam_age_s=plain_age,
                fused_age_s=fused_age,
            )
        return BridgeComputation(
            ok=True,
            reason=REASON_OK,
            map_to_odom=map_to_odom_2d(plain_pose, fused_pose),
            odom_to_base=fused_pose,
            frame_skew_s=skew,
            plain_slam_age_s=plain_age,
            fused_age_s=fused_age,
        )

    def verdict(
        self, computation: BridgeComputation, *, tf_published: bool
    ) -> LocalizationVerdict:
        """TF 广播之后补上最后一项条件。"""
        if not computation.ok:
            return LocalizationVerdict(False, computation.reason)
        if not tf_published:
            return LocalizationVerdict(False, REASON_TF_NOT_PUBLISHED)
        return LocalizationVerdict(True, REASON_OK)

    def _empty(self, reason: str) -> BridgeComputation:
        return BridgeComputation(
            ok=False,
            reason=reason,
            map_to_odom=None,
            odom_to_base=None,
            frame_skew_s=None,
            plain_slam_age_s=None,
            fused_age_s=None,
        )


def _nearest(
    buffer: Iterable[tuple[float, Pose2D]], stamp_s: float
) -> tuple[float, Pose2D]:
    return min(buffer, key=lambda item: abs(item[0] - stamp_s))


def retarget_map_frame(grid: Any, frame_id: str = MAP_FRAME) -> Any:
    """就地改写 ``header.frame_id``（``grid`` 是 ``nav_msgs/OccupancyGrid``）。"""
    grid.header.frame_id = frame_id
    return grid


# ---------------------------------------------------------------------------
# ROS 层
# ---------------------------------------------------------------------------


def main() -> None:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from std_msgs.msg import Bool
    from tf2_ros import TransformBroadcaster

    class PlainSlamNavFrameBridge(Node):
        def __init__(self) -> None:
            super().__init__("plain_slam_nav_frame_bridge")
            self.declare_parameter("map_frame", MAP_FRAME)
            self.declare_parameter("odom_frame", ODOM_FRAME)
            self.declare_parameter("base_frame", BASE_FRAME)
            self.declare_parameter("max_odom_age_s", DEFAULT_MAX_ODOM_AGE_S)
            self.declare_parameter("max_frame_skew_s", DEFAULT_MAX_FRAME_SKEW_S)
            self.declare_parameter("publish_rate_hz", 10.0)
            self.declare_parameter("map_2d_topic", MAP_2D_TOPIC)
            self.declare_parameter("plain_slam_odom_topic", PLAIN_SLAM_ODOM_TOPIC)
            self.declare_parameter("fused_odom_topic", FUSED_ODOM_TOPIC)

            self._map_frame = str(self.get_parameter("map_frame").value)
            self._odom_frame = str(self.get_parameter("odom_frame").value)
            self._base_frame = str(self.get_parameter("base_frame").value)
            self._core = FrameBridgeCore(
                max_odom_age_s=float(self.get_parameter("max_odom_age_s").value),
                max_frame_skew_s=float(self.get_parameter("max_frame_skew_s").value),
            )
            rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))

            # /map 走 transient_local：Nav2 的 static_layer 晚启动也要能拿到地图。
            map_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            # 两路 odom 用 BEST_EFFORT：与 RELIABLE 发布者兼容，也与 BEST_EFFORT 发布者兼容。
            odom_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=20,
            )
            source_map_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            valid_qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )

            self._map_pub = self.create_publisher(OccupancyGrid, MAP_TOPIC, map_qos)
            self._valid_pub = self.create_publisher(
                Bool, LOCALIZATION_VALID_TOPIC, valid_qos
            )
            self._tf_broadcaster = TransformBroadcaster(self)

            self.create_subscription(
                OccupancyGrid,
                str(self.get_parameter("map_2d_topic").value),
                self._on_map,
                source_map_qos,
            )
            self.create_subscription(
                Odometry,
                str(self.get_parameter("plain_slam_odom_topic").value),
                self._on_plain_slam_odom,
                odom_qos,
            )
            self.create_subscription(
                Odometry,
                str(self.get_parameter("fused_odom_topic").value),
                self._on_fused_odom,
                odom_qos,
            )
            self.create_timer(1.0 / rate, self._on_timer)
            self._last_log_reason = ""
            self.get_logger().info(
                "plain_slam_nav_frame_bridge up: "
                f"{self._map_frame} -> {self._odom_frame} -> {self._base_frame}"
            )

        # ------------------------------------------------------- callbacks
        def _on_map(self, message: OccupancyGrid) -> None:
            self._core.note_map()
            self._map_pub.publish(retarget_map_frame(message, self._map_frame))

        def _on_plain_slam_odom(self, message: Odometry) -> None:
            self._core.note_plain_slam_odom(
                stamp_to_seconds(message.header.stamp), _pose_of(message)
            )

        def _on_fused_odom(self, message: Odometry) -> None:
            self._core.note_fused_odom(
                stamp_to_seconds(message.header.stamp), _pose_of(message)
            )

        def _on_timer(self) -> None:
            now_s = stamp_to_seconds(self.get_clock().now().to_msg())
            computation = self._core.compute(now_s)
            tf_published = False
            if computation.ok and computation.map_to_odom and computation.odom_to_base:
                stamp = self.get_clock().now().to_msg()
                transforms = [
                    self._transform(
                        stamp,
                        self._map_frame,
                        self._odom_frame,
                        computation.map_to_odom,
                    ),
                    self._transform(
                        stamp,
                        self._odom_frame,
                        self._base_frame,
                        computation.odom_to_base,
                    ),
                ]
                self._tf_broadcaster.sendTransform(transforms)
                tf_published = True
            verdict = self._core.verdict(computation, tf_published=tf_published)
            self._valid_pub.publish(Bool(data=bool(verdict.valid)))
            self._maybe_log(verdict, computation)

        def _transform(self, stamp, parent: str, child: str, pose: Pose2D):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = parent
            transform.child_frame_id = child
            transform.transform.translation.x = float(pose.x)
            transform.transform.translation.y = float(pose.y)
            transform.transform.translation.z = 0.0
            qx, qy, qz, qw = quaternion_from_yaw(pose.yaw)
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            return transform

        def _maybe_log(self, verdict, computation: BridgeComputation) -> None:
            """只在判定翻转时打日志；行内带上 skew，现场可直接看出是不是时钟差。"""
            if verdict.reason == self._last_log_reason:
                return
            self._last_log_reason = verdict.reason
            skew = computation.frame_skew_s
            detail = (
                f"frame_skew_ms={round(float(skew) * 1000.0, 1)}"
                if skew is not None
                else "frame_skew_ms=unknown"
            )
            if verdict.valid:
                self.get_logger().info(f"localization_valid=true {detail}")
            else:
                self.get_logger().warn(
                    f"localization_valid=false reason={verdict.reason} {detail}"
                )

    def stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _pose_of(message: Odometry) -> Pose2D:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        return Pose2D(
            x=float(position.x),
            y=float(position.y),
            yaw=yaw_from_quaternion(
                float(orientation.x),
                float(orientation.y),
                float(orientation.z),
                float(orientation.w),
            ),
        )

    rclpy.init()
    node = PlainSlamNavFrameBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
