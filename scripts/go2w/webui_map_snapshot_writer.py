#!/usr/bin/env python3
"""把当前权威 SLAM 地图写成 WebUI 的地图快照（go2w_slam_web_cloud_v2）。

背景：manual_web_demo 的 ``/api/slam/map3d`` **不订阅实时话题**，而是直接读快照文件
``outputs/autonomous_search/runtime/slam_map_3d.json``。该文件在本机已停止更新
（现存副本的时间戳是 2026-09-02，且其中 pose 是异常值 2.28e11）——这就是 WebUI 上
"地图重叠"的直接来源。本节点周期性地把**当前选择的地图话题**写成该快照格式。

用法（先 source scripts/go2w/setup_environment.sh）：

    /usr/bin/python3 scripts/go2w/webui_map_snapshot_writer.py --ros-args \
      -p map_topic:=/go2w/rko_slam/map_3d -p max_points:=60000
"""

from __future__ import annotations

import json
import os
import sys
import time
import uuid

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import PointCloud2

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
# 判据住在 app/ 下，理由见那个模块的 docstring：它必须能被没有 rclpy 的 pytest
# 直接 import。本节点的启动环境只保证 scripts/go2w 在 sys.path 上。
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from app.spatial.lio_nofix_judge import (  # noqa: E402
    DEGRADED_LIO_DRIFT,
    HEALTHY,
    LioNofixJudge,
)
from app.spatial.object_display_snapshot import meta_payload  # noqa: E402

DEFAULT_SNAPSHOT = os.path.join(
    _PROJECT_ROOT, "outputs/autonomous_search/runtime/slam_map_3d.json")

DEFAULT_LIO_DIAG_LOG = os.path.join(
    _PROJECT_ROOT, "runtime/go2w/sessions/plain_slam_go2w.launch.log")

# 回填窗口。DIAG 行实测均长 265 字节、速率 10 Hz ⇒ 1 MB ≈ 6 分钟历史。
# 必须回填：LIO 先发散、writer 后启动（launcher 每次都会重启 writer）时，
# 只 tail 新内容必然假绿。
LIO_DIAG_BACKFILL_BYTES = 1 << 20

# 轨迹点的绝对量级上限（米）。odom 发散时 x/y 会指数涨到 1e20 —— 2026-09-11 实测
# 持久化轨迹 9332 个点里 9206 个是这种值。轨迹决定 WebUI 的画幅 focus_bounds，
# 画幅被撑到 1e19 后前端 scale≈0（地图缩成一个像素 = 看不见），同时 drawGrid 的
# 网格循环要从 -8.7e19 跑到 +8.7e19、步长 5，浏览器主线程永久卡死。
MAX_TRAJECTORY_ABS_M = 1000.0


def best_effort(depth: int = 5) -> QoSProfile:
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.BEST_EFFORT,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.VOLATILE)


def reliable(depth: int = 5) -> QoSProfile:
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.VOLATILE)


def latched(depth: int = 1) -> QoSProfile:
    """plain_slam 的 map_3d 是 transient_local 发布：新订阅者要收到"最后一帧"必须匹配。"""
    return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE,
                      history=HistoryPolicy.KEEP_LAST,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class WebuiSnapshotWriter(Node):
    def __init__(self) -> None:
        super().__init__("webui_map_snapshot_writer")
        self.declare_parameter("map_topic", "/go2w/rko_slam/map_3d")
        self.declare_parameter("snapshot_path", DEFAULT_SNAPSHOT)
        self.declare_parameter("max_points", 60000)
        self.declare_parameter("period_s", 5.0)
        # 显示层清理（对应用户反馈的 B/C 类问题）：
        #   max_range_m      —— 去掉远处噪声/窗外散点
        #   min_range_m      —— 去掉机身自遮挡的近点
        #   voxel_size_m     —— 体素降采样，压掉重复叠加的冗余点
        #   min_z_m/max_z_m  —— 高度过滤。原点在雷达。2026-09-11 实测：
        #     地面在 z≈-0.23（低层点拟合平面 z@原点=-0.234，倾角仅 0.08°，
        #     说明不是整体倾斜；但它不是一张干净平面，而是 -0.336 / -0.097
        #     两张相距 0.24 m 的平行面，残差 std 0.13 m）。
        #     俯视投影时地面回波把房间内部填成一整团，z<0 的点占 30%，
        #     完全不提供结构信息，只会糊住画面。默认取 0.15（在两张地面
        #     之上、家具与墙之下）；要连地面一起看就设回 -0.45。
        #     上限 2.80 去掉天花板与灯。
        self.declare_parameter("max_range_m", 25.0)
        self.declare_parameter("min_range_m", 1.0)
        self.declare_parameter("voxel_size_m", 0.05)
        self.declare_parameter("min_z_m", 0.15)
        self.declare_parameter("max_z_m", 2.80)
        # 轨迹距离过滤 —— 这是唯一有物理依据的幽灵点判据。
        # 2026-09-11 实测（用户遥控走完房间后）：
        #   单帧 /go2w/slam/deskewed_scan 最远只到 9.9 m（半径 p50=1.9 p99=6.4），
        #   而累积 /go2w/slam/map_3d 铺到 x[-60.9,27.6] y[-59.2,34.4]。
        #   地图 = 45 个关键帧点云按优化位姿摆放的并集，其中的误回环会把整帧
        #   点云扔到几十米外 —— 密度过滤分不掉它们（它们自己就是密云，
        #   0.5 m 邻域 p50 有 68 个邻居），但"被观测到的点必在某个位姿的
        #   雷达量程内"是硬约束。实测距轨迹 >12 m 的点占 3.9%（5803 个），
        #   却把 WebUI 的自动缩放宽到 60 m，房间只占画幅 7%，这才是"发糊"。
        # 不给 0 表示不启用；轨迹点不足 20 个时也不启用（避免半截轨迹误删真点）。
        self.declare_parameter("odom_topic", "/go2w/slam/odom_base")
        self.declare_parameter("trajectory_radius_m", 12.0)
        # 2026-09-11 二次核验（分层渲染）修正了上面的判断：>12 m 的点只占
        # 3.9%，砍掉后画幅 60 m→27 m，但房间仍只占 26%。分层画出来才看清：
        # 5-12 m 那层是**空间上完全分离的走廊结构**（矩形边界、栏杆条纹），
        # 不是与房间错位叠加的整帧幽灵；0-5 m 那层本身就是一张轮廓清晰的
        # 房间图。所以"发糊"的主因不在数据，在渲染（见下面的 focus_bounds）。
        # 过滤器保留：它挡的是真实的误回环离群帧，只是量没当初以为的大。
        #
        # 画幅聚焦边距（focus_bounds）。WebUI 用点的 bbox 自动定缩放，而地图
        # 含 12 m 内的走廊，bbox 有 27×31 m，房间（8 m）只占画幅 26%，细节
        # 全挤在一起。轨迹是"狗真正建过图的区域"，拿轨迹 bbox 加这个边距当
        # 画幅，房间能占约 60%。给 0 表示不提供（前端退回用点云 bbox）；
        # 轨迹不足 20 点时同样不提供。
        self.declare_parameter("focus_margin_m", 3.0)
        # 轨迹落盘：writer 重启后不至于丢掉整条轨迹（否则要么过滤器失效，
        # 要么拿半截轨迹误删真点）。SLAM 会话重启后地图换系，旧轨迹会
        # 让过滤比例飙到接近 100%，由下面的 50% 护栏自动停用。
        self.declare_parameter("trajectory_path", os.path.join(
            os.path.dirname(DEFAULT_SNAPSHOT), "slam_map_trajectory.json"))
        # §2.3：本写端**不再**从 events.jsonl 里挑物体。在线显示的唯一对象源是
        # 语义建图进程写的 semantic_objects.json，它按 map_epoch 与这张地图配对
        # （见 app/manual_web_demo/slam_map_snapshot.py 的合成逻辑）。以前这里
        # 用 glob 猜"哪份 events 属于当前地图"，2026-09-17 实测挑中的是 09-10 的
        # Dataset D 归档，当天新定位的 20 个物体一个都没上屏 —— 猜来源这条路已经
        # 被证伪，不要加回来。原始逐帧检测仍留在 events.jsonl 里供离线评测使用。
        # 点云之外另写一份轻量元数据（map_epoch/frame_id/map_revision），对象
        # 生产者只读它，不去解析这份几万点的大 JSON。
        self.declare_parameter("meta_path", os.path.join(
            os.path.dirname(DEFAULT_SNAPSHOT), "slam_map_meta.json"))
        # LIO 的 launch 日志：§13.4 健康门的判据源。`[GO2W-DIAG]` 那些量
        # （ros2_ws/src/plain_slam_ros2/src/lio_3d_interface.cpp:250-263）没有任何
        # 非日志出口，而那个文件在 §10.2 禁改清单里、且重启 LIO 栈会作废当前建图
        # 会话 —— 所以判据从日志读。判据本身已在该日志的逐帧转录上做过注入校准，
        # 见 tests/test_lio_nofix_gate_calibration.py。
        self.declare_parameter("lio_diag_log", DEFAULT_LIO_DIAG_LOG)
        self._latest_bounds = None
        self._max_range = float(self.get_parameter("max_range_m").value)
        self._min_range = float(self.get_parameter("min_range_m").value)
        self._voxel = float(self.get_parameter("voxel_size_m").value)
        self._min_z = float(self.get_parameter("min_z_m").value)
        self._max_z = float(self.get_parameter("max_z_m").value)
        self._traj_radius = float(self.get_parameter("trajectory_radius_m").value)
        self._trajectory_path = str(self.get_parameter("trajectory_path").value)
        self._focus_margin = float(self.get_parameter("focus_margin_m").value)
        self._trajectory_saved = 0
        self._trajectory: list = self._load_trajectory()
        self._snapshot_path = str(self.get_parameter("snapshot_path").value)
        self._meta_path = str(self.get_parameter("meta_path").value)
        self._max_points = int(self.get_parameter("max_points").value)
        self._latest = None
        self._received = 0
        # 地图**数据**最后一次变化的墙钟时间（不是文件写入时间）。_flush 每
        # period_s 秒重写一次快照，generated_at 会跟着刷新；拿它当地图年龄等于
        # 恒报新鲜。前端"slam_map_3d.js"的"地图 X 前"读的是读取端算出的
        # map_age_seconds，而读取端优先取 ros_stamp（跟着数据走，写端重启后
        # 重投 latched 那一帧也不会变），只在没有 ros_stamp 时才退回这个键。
        self._map_updated_at = None
        self._last_ros_stamp = None
        # §13.4 健康门。两条独立的降级路径，复位方式不同：
        #   发散 latch —— 复位只有一条路：观察到 LIO 栈重启（launch 日志被截断）；
        #   饿死       —— 自清：DIAG 行重新出现即解除。理由见判据模块的 docstring。
        self._lio_diag_log = str(self.get_parameter("lio_diag_log").value)
        self._period_s = float(self.get_parameter("period_s").value)
        # 饿死阈值取 2 × poll 周期，语义是「至少连续 3 轮 poll 没读到新 DIAG 行」
        # ⇒ 至少 10 秒没有 DIAG 行。**不能取 1 × 周期**：_poll_lio_diag 是每
        # period_s 秒批量跑一次，这个周期本身就把 (now - last_feed) 量化了，
        # 取 1 × 周期时单轮 timer 抖动就能误报。标定数字见
        # app/spatial/lio_nofix_judge.py 的模块 docstring。
        self._judge = LioNofixJudge(starve_timeout_s=2.0 * self._period_s)
        self._lio_fh = None
        self._session_id = 0
        # §2.3：进程启动随机前缀 + 地图会话计数。重启即换前缀，LIO 日志截断
        # （= SLAM 换了会话）即换计数，两种"这不是同一张图了"都因此改变 epoch，
        # 旧对象立刻不上屏，不需要再造第二套 reset 状态。
        self._epoch_prefix = uuid.uuid4().hex[:8]
        # §13.5 的 last-good 地图本体，元素见 _flush 里 body 那个 dict。
        # None = 还没攒到过一帧健康地图。
        self._last_good_map = None
        topic = str(self.get_parameter("map_topic").value)
        # map_3d 多为 RELIABLE 发布（transient_local），两个 QoS 都订阅以兼容。
        self.create_subscription(PointCloud2, topic, self._on_cloud, reliable(3))
        self.create_subscription(PointCloud2, topic, self._on_cloud, best_effort(3))
        self.create_subscription(PointCloud2, topic, self._on_cloud, latched(1))
        odom_topic = str(self.get_parameter("odom_topic").value)
        if odom_topic and self._traj_radius > 0.0:
            # plain_slam_odom_adapter 以 BEST_EFFORT 发布；再挂一个 RELIABLE 订阅
            # 以兼容换成其它发布端的情况（重复帧由 _on_odom 的距离去重吸收）。
            self.create_subscription(Odometry, odom_topic, self._on_odom, best_effort(10))
            self.create_subscription(Odometry, odom_topic, self._on_odom, reliable(10))
        else:
            odom_topic = ""
        self.create_timer(self._period_s, self._flush)
        self.get_logger().info(f"snapshot writer: {topic} -> {self._snapshot_path}")
        if odom_topic:
            self.get_logger().info(
                f"trajectory filter: {odom_topic} (radius {self._traj_radius} m)")

    def _focus_bounds(self):
        """给 WebUI 定画幅的聚焦框（轨迹 bbox + 边距，见参数处说明）。"""
        if self._focus_margin <= 0.0 or len(self._trajectory) < 20:
            return None
        trajectory = np.asarray(self._trajectory, dtype=np.float64)
        low, high = trajectory.min(axis=0), trajectory.max(axis=0)
        # 画幅不许超出当前地图（+边距）。odom 发散时轨迹尾段会甩到几百米外
        # （2026-09-15 实测 313.7 / 996.8 m，09-17 实测 978.3 m），三个值全都刚好
        # 绕过 MAX_TRAJECTORY_ABS_M=1000 这道绝对上限，画幅被撑到千米量级、房间压进
        # 角落的一个像素 —— 就是"地图全缩在边边一点、其余全空"。改用地图当归一化基准
        # 是相对判据，发散到多远都拦得住，不像常量要靠刚好卡在中间（那条老路三次全没碰到）。
        if self._latest_bounds is not None:
            map_low = np.asarray(self._latest_bounds[0], dtype=np.float64)[:2]
            map_high = np.asarray(self._latest_bounds[1], dtype=np.float64)[:2]
            low = np.maximum(low, map_low)
            high = np.minimum(high, map_high)
            if (low > high).any():
                # 整条轨迹都被夹到了地图外（SLAM 重启换了原点，轨迹与当前地图不同系）。
                # 此时画幅取地图范围 —— 绝不能退回上面那个 bbox，它会是个千米级空框。
                low, high = map_low, map_high
        return {
            "min": [float(low[0] - self._focus_margin),
                    float(low[1] - self._focus_margin), self._min_z],
            "max": [float(high[0] + self._focus_margin),
                    float(high[1] + self._focus_margin), self._max_z],
        }

    def _load_trajectory(self) -> list:
        try:
            with open(self._trajectory_path, encoding="utf-8") as handle:
                return [(float(x), float(y)) for x, y in json.load(handle)]
        except (OSError, ValueError, TypeError):
            return []

    def _save_trajectory(self) -> None:
        if len(self._trajectory) == self._trajectory_saved:
            return
        try:
            os.makedirs(os.path.dirname(self._trajectory_path), exist_ok=True)
            temporary = f"{self._trajectory_path}.tmp"
            with open(temporary, "w", encoding="utf-8") as handle:
                json.dump([[round(x, 3), round(y, 3)] for x, y in self._trajectory], handle)
            os.replace(temporary, self._trajectory_path)
            self._trajectory_saved = len(self._trajectory)
        except OSError as error:
            self.get_logger().warn(f"cannot persist trajectory: {error}")

    def _on_odom(self, message: Odometry) -> None:
        """记录轨迹（同 pslam_map 系）。0.25 m 去重，避免静止时堆积。"""
        position = message.pose.pose.position
        x, y = float(position.x), float(position.y)
        # 发散点在这里就丢（NaN/Inf 也走这条路：abs(nan) < 上限 恒为假）。
        if not (abs(x) < MAX_TRAJECTORY_ABS_M and abs(y) < MAX_TRAJECTORY_ABS_M):
            return
        if self._trajectory:
            last_x, last_y = self._trajectory[-1]
            if (x - last_x) ** 2 + (y - last_y) ** 2 < 0.0625:
                return
        self._trajectory.append((x, y))

    def _drop_far_from_trajectory(self, points):
        """丢掉离轨迹超过 _traj_radius 的点（幽灵点判据，见参数处说明）。"""
        if self._traj_radius <= 0.0 or len(self._trajectory) < 20 or not len(points):
            return points
        trajectory = np.asarray(self._trajectory, dtype=np.float64)
        keep = np.zeros(len(points), dtype=bool)
        radius2 = self._traj_radius ** 2
        # 双向分块，内存有界（2000×256 点对）
        for start in range(0, len(points), 2000):
            block = points[start:start + 2000, :2]
            best = np.full(len(block), np.inf)
            for t0 in range(0, len(trajectory), 256):
                chunk = trajectory[t0:t0 + 256]
                delta = block[:, None, :] - chunk[None, :, :]
                best = np.minimum(best, (delta ** 2).sum(axis=-1).min(axis=1))
            keep[start:start + 2000] = best <= radius2
        if not keep.any():
            return points
        # 护栏：正常只滤掉个位数百分比；一旦超过一半，说明轨迹与当前地图不同系
        # （SLAM 会话重启后地图换原点），此时过滤会删掉真地图，直接停用。
        if (~keep).mean() > 0.5:
            self.get_logger().warn(
                f"trajectory filter would drop {100.0 * (~keep).mean():.0f}% of the map "
                f"— trajectory likely from another SLAM session, filter disabled")
            return points
        self.get_logger().info(
            f"trajectory filter dropped {int((~keep).sum())} ghost point(s) "
            f"(>{self._traj_radius} m from any pose)")
        return points[keep]

    def _on_cloud(self, message: PointCloud2) -> None:
        self._latest = message
        self._received += 1
        stamp = (float(message.header.stamp.sec)
                 + float(message.header.stamp.nanosec) * 1e-9)
        if stamp != self._last_ros_stamp:
            # 只有地图数据真的换了一帧才更新，重复投递同一帧（latched/多 QoS）
            # 不算"地图在长"。
            self._last_ros_stamp = stamp
            self._map_updated_at = time.time()

    @staticmethod
    def _write_atomic(path: str, payload: dict) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        temporary = f"{path}.tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump(payload, handle)
        os.replace(temporary, path)

    def _open_lio_diag(self, start: int):
        try:
            handle = open(self._lio_diag_log, "r", errors="replace")
        except OSError:
            return None
        handle.seek(start)
        if start > 0:
            handle.readline()   # seek 到中间时丢掉半行
        return handle

    def _poll_lio_diag(self) -> None:
        """tail LIO 的 launch 日志，喂判据，维护 mapping_health。

        这里是**发散锁存**的唯一复位路径：日志变短 == 被截断 == LIO 换会话
        （start_plain_slam_mapping.sh:201 用 ``>`` 重定向）。不许改成"前兆消失
        即复位" —— 判据触发后会自己闭嘴（实测其后几千帧 run 跌回 0 且只再触发
        1 次），那样做会把闸门重新打开，制造出比改动前更糟的假绿。

        饿死（DIAG 行停够 starve_timeout_s）走的是另一条路、且是自清的：
        这里读不到新行时 ``feed()`` 不被调用，判据自己按注入的时钟发现饿死；
        新行一出现即自解。所以本方法**不需要**为饿死做任何事。

        文件打不开（路径不对 / LIO 没起）时直接返回，判据保持不表态 ——
        调用方因此**不写** mapping_health 字段。这是有意为之的第三态，
        不是 bug：见 app/spatial/lio_nofix_judge.py 的 docstring。
        注意它和饿死是两件事：``_last_feed is None``（从没喂过）不表态，
        ``_last_feed`` 过期（喂过又停了）才报 DEGRADED。
        """
        try:
            size = os.path.getsize(self._lio_diag_log)
        except OSError:
            return
        if self._lio_fh is None:
            self._lio_fh = self._open_lio_diag(
                max(0, size - LIO_DIAG_BACKFILL_BYTES))
        elif self._lio_fh.tell() > size:
            self._session_id += 1
            self._judge.note_session_restart()
            self._lio_fh.close()
            self._lio_fh = self._open_lio_diag(0)   # 文件可能被删了重建
        if self._lio_fh is None:
            return
        while True:
            line = self._lio_fh.readline()
            if not line:
                break
            if "GO2W-DIAG" in line:
                self._judge.feed(line)

    def _flush(self) -> None:
        # 放在 early return 之前：地图还没来时判据也要继续跟，否则状态会冻结。
        self._poll_lio_diag()
        if self._latest is None:
            return
        message = self._latest
        count = int(message.width) * int(message.height)
        if count == 0:
            return
        raw = np.frombuffer(message.data, dtype=np.uint8).reshape(count, int(message.point_step))
        points = np.ascontiguousarray(raw[:, 0:12]).view(np.float32).reshape(-1, 3)
        points = points[np.isfinite(points).all(axis=1)]
        # range filter（去掉远处噪声与近处自遮挡）
        radius = np.linalg.norm(points, axis=1)
        points = points[(radius >= self._min_range) & (radius <= self._max_range)]
        # 高度过滤（去掉地面回波层与天花板/灯，见参数处说明）
        if len(points):
            points = points[(points[:, 2] >= self._min_z)
                            & (points[:, 2] <= self._max_z)]
        # 轨迹距离过滤（去掉误回环扔到远处的整帧点云，见参数处说明）
        points = self._drop_far_from_trajectory(points)
        # voxel 降采样（压掉"同一面墙重复叠加"的冗余点，让墙看着更薄更干净）
        if self._voxel > 0.0 and len(points):
            keys = np.floor(points / self._voxel).astype(np.int64)
            _, unique = np.unique(keys, axis=0, return_index=True)
            points = points[np.sort(unique)]
        # 三级点数必须是真的三级：SLAM 源点(count) > 体素后(voxels) > 网页显示(step 后)。
        # 2026-09-03 的 v2 门限明写"三个数不许相等冒充"，所以这里留中间量。
        voxels = int(len(points))
        step = max(1, len(points) // self._max_points)
        points = points[::step]
        stamp = float(message.header.stamp.sec) + float(message.header.stamp.nanosec) * 1e-9
        body = {
            "points": points,
            "voxels": voxels,
            "low": [float(v) for v in points.min(axis=0)] if len(points) else None,
            "high": [float(v) for v in points.max(axis=0)] if len(points) else None,
            "stamp": stamp,
            "frame": message.header.frame_id or "pslam_map",
            "revision": self._received,
            "source_points": count,
        }
        # §13.4/§13.5 健康门：判据已锁存 DEGRADED 时，这一帧点云大概率已被发散
        # 的 LIO 污染，不许它顶掉 last-good。冻的是**地图本体**（点云 / 包围盒 /
        # 体素数 / revision / ros_stamp 一律沿用 last-good 那一帧），而
        # generated_at 照常前进 —— 所以前端算出的 fresh 仍为真，"故意冻结"与
        # "写端死了"不会在页面上塌成同一个显示（这正是当初不做冻结的理由）。
        # 判据不表态（health is None）时照常更新，第三态不冻结。
        health = self._judge.health
        frozen = health == DEGRADED_LIO_DRIFT and self._last_good_map is not None
        if frozen:
            body = self._last_good_map
        elif health != DEGRADED_LIO_DRIFT:
            self._last_good_map = body
        points, voxels = body["points"], body["voxels"]
        low, high, stamp, frame = body["low"], body["high"], body["stamp"], body["frame"]
        self._latest_bounds = (low, high) if low is not None else None
        # 字段名必须是前端 static/slam_map_3d.js 读的那套 v2 词表（§9/§10 的
        # app/spatial/slam_web_map_state.py 定义），否则页脚全取到 undefined 显示 0。
        # 本写端只订阅地图话题，§10 那些概念拿不到的一律不编 —— 缺字段时前端显示
        # "--"，好过一个假的 0 或者假的 HEALTHY。
        # 例外是 mapping_health / lio_pose_valid / mapping_session_id 三个：
        # 2026-09-16 起它们有了真信号源（LIO 的 launch 日志判据 + 会话重启计数），
        # 见 _poll_lio_diag。判据不表态时仍然一个都不写。
        payload = {
            "schema_version": "go2w_slam_web_cloud_v2",
            "available": bool(len(points)),
            "source": f"map_3d:{self.get_parameter('map_topic').value}",
            "permanent_source": str(self.get_parameter("map_topic").value),
            "canonical_frame": frame,
            "frame_id": frame,
            "map_epoch": f"{self._epoch_prefix}-{self._session_id}",
            "target_map_frame": frame,
            "ros_stamp": stamp,
            "last_good_map_stamp": stamp,
            "map_updated_at": self._map_updated_at,
            "generated_at": time.time(),
            "point_count": int(len(points)),
            "source_map_points": body["source_points"],
            "global_cached_voxels": voxels,
            "web_display_points": int(len(points)),
            "max_web_points": self._max_points,
            "map_revision": body["revision"],
            "global_voxel_size_m": self._voxel,
            "voxel_size_m": self._voxel,
            "map_extent_m": ([high[i] - low[i] for i in range(3)]
                             if low is not None else [0.0, 0.0, 0.0]),
            "bounds": {"min": low, "max": high},
            # 前端 centerAndSpan() 优先用这个定缩放；null 时退回 bounds。
            "focus_bounds": self._focus_bounds(),
            "points": [[round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3)]
                       for p in points],
            # objects / object_filter_stats 不在这里写：本写端只负责地图。
            # /api/slam/map3d 会把语义建图进程写的对象快照按 map_epoch 合成进来。
            # 本写端不订阅 scan 话题，给一个干净的空 preview；前端显示"未接入"。
            "preview": {"frame_id": "", "ros_stamp": 0.0, "wall_time": 0.0,
                        "point_count": 0, "points": []},
            "mapping_mode": "mapping_assist",
            "motion_authorized": False,
            "safety_authorized": False,
            # 这两个是占位：读取端 load_slam_map_snapshot() 会用 generated_at
            # 重算并覆盖它们，改这里不影响前端。真正的地图年龄看 map_updated_at。
            "age_seconds": 0.0,
            "fresh": True,
            "map_age_seconds": None,
            "preview_age_seconds": None,
            # 这个计数是上面唯一复位路径（日志截断）的仪表，也是
            # reset_plain_slam_session.sh 第 [2/6] 步要读的字段。
            "mapping_session_id": self._session_id,
        }
        # §13.4 健康门（衔接 web_server.py:377 的 slam_map_not_drifting 与
        # :405 的 mapping_health）。严格三态，只有前两态写字段：
        #   DEGRADED_LIO_DRIFT -> 就绪页拦下 /api/search/start，3D 页转黄
        #   HEALTHY            -> 3D 页转绿。这个绿是**有边界的**：它只断言
        #                         "LIO 的 iter/fix_t 判据未触发"，不断言地图质量、
        #                         不断言 act、不断言 ab。
        #   键不写（判据不表态） -> 判据没在跑，前端显示 "--"、闸门放行
        if health is not None:
            payload["mapping_health"] = health
            payload["lio_pose_valid"] = health != DEGRADED_LIO_DRIFT
            # §16.3 状态栏要的 LIVE / FROZEN_LAST_GOOD。第三个值 SUSPECT 不在
            # 计划书名单里，但那种情形是真实存在的：writer 起来时 backfill 就
            # 已经判了 DEGRADED，此时 _last_good_map 是 None，冻不了、只能照发
            # 这一帧可疑点云。此时报 LIVE 是假绿，报 FROZEN_LAST_GOOD 是撒谎。
            payload["map_status"] = (
                "FROZEN_LAST_GOOD" if frozen
                else "LIVE" if health == HEALTHY
                else "SUSPECT")
            if health == DEGRADED_LIO_DRIFT:
                payload["health_reason"] = self._judge.reason
        self._save_trajectory()
        self._write_atomic(self._snapshot_path, payload)
        # 元数据在地图之后落盘：先写 meta 会让对象生产者读到一张还没上屏的地图的
        # epoch，于是它投影出的对象要等下一次轮询才对得上。
        self._write_atomic(self._meta_path, meta_payload(payload))
        self.get_logger().info(
            f"snapshot written: {len(points)} pts frame={payload['frame_id']} "
            f"({os.path.getsize(self._snapshot_path)} bytes)")


def main() -> int:
    rclpy.init()
    node = WebuiSnapshotWriter()
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
