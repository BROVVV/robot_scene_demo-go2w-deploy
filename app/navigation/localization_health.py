"""Pure checks shared by the 2D localization monitor and runtime audit."""
from __future__ import annotations

import math
from typing import Any
from app.navigation.occupancy_values import int8_grid_view


class StructuralScanFreshnessLatch:
    """Debounce only the structural mapping scan, never the safety scan.

    ``/go2w/lidar/mapping_scan`` is assembled after the raw collision scan and
    may miss one monitor tick while the Jetson is busy.  A single late
    structural frame must not cancel an already-safe Nav2 goal when the raw
    scan is still fresh.  Continuous structural loss still fails closed after
    a small, explicit grace window; loss of the raw scan fails immediately.
    """

    def __init__(self, grace_seconds: float = 0.25):
        if not math.isfinite(grace_seconds) or not 0.0 < grace_seconds <= 0.5:
            raise ValueError("structural scan grace must be in (0, 0.5] seconds")
        self.grace_seconds = float(grace_seconds)
        self._invalid_since: float | None = None

    def update(self, *, structural_fresh: bool, safety_scan_fresh: bool,
               now: float) -> tuple[bool, bool]:
        """Return ``(effective_fresh, grace_active)`` for this observation."""
        if not math.isfinite(now):
            raise ValueError("monotonic time must be finite")
        if structural_fresh:
            self._invalid_since = None
            return True, False
        if not safety_scan_fresh:
            self._invalid_since = None
            return False, False
        if self._invalid_since is None or now < self._invalid_since:
            self._invalid_since = now
        if now - self._invalid_since < self.grace_seconds:
            return True, True
        return False, False


def canonical_node_name(name: str, namespace: str) -> str:
    """Return the ROS graph spelling used by publisher/service contracts."""
    return ('/' + '/'.join(part.strip('/') for part in (namespace, name)
                           if part.strip('/'))).replace('//', '/')


def authoritative_map_authorization(status: dict, publishers: list[str], *, mode: str) -> bool:
    """Accept map authorization only from the live localization health owner.

    A read-only audit creates its own sampler node, whose local
    ``map_authorized`` parameter is intentionally false.  In localization mode
    the authority is the already-running health monitor launched only after
    strict saved-map verification.  Require both its public check and raw
    evidence plus the exact unique publisher identity; a user-supplied audit
    flag or an arbitrary String publisher cannot grant this condition.
    """
    return bool(
        mode == "localization"
        and publishers == ["/go2w_localization_health"]
        and isinstance(status, dict)
        and status.get("mode") == "localization"
        and status.get("valid") is True
        and (status.get("checks") or {}).get("map_authorized") is True
        and (status.get("evidence") or {}).get("map_authorized") is True
    )


def map_statistics(width: int, height: int, resolution: float, data) -> dict:
    counts = {"free": 0, "occupied": 0, "unknown": 0, "invalid": 0}
    values = int8_grid_view(data)
    if values is not None:
        counts['unknown'] = int((values == -1).sum())
        counts['free'] = int((values == 0).sum())
        counts['occupied'] = int(((values > 0) & (values <= 100)).sum())
        counts['invalid'] = len(values) - sum(counts.values())
    else:
        for value in data:
            if value == -1:
                counts["unknown"] += 1
            elif value == 0:
                counts["free"] += 1
            elif 0 < value <= 100:
                counts["occupied"] += 1
            else:
                counts["invalid"] += 1
    total = width * height
    return {"width": width, "height": height, "resolution": resolution,
            "cells": len(data), **counts,
            "valid": (width > 0 and height > 0 and math.isfinite(resolution)
                      and resolution > 0 and len(data) == total
                      and not counts["invalid"] and counts["free"] > 0
                      and counts["occupied"] > 0)}


def sanitize_non_finite(value, *, path: str = "$") -> tuple[Any, list[str]]:
    """把结构里的非有限浮点换成 ``None``，并返回它们出现的位置（计划书 §P3-6）。

    返回 ``(cleaned, offenders)``；``offenders`` 形如
    ``$['evidence']['pose']['x']``。它存在的唯一理由是让
    ``json.dumps(..., allow_nan=False)`` 编得出来：

    * ``allow_nan=True`` 会写出 ``NaN`` / ``Infinity`` —— 那不是合法 JSON，
      Python 自己读得回来，JS 的 ``JSON.parse`` 读不了，而这份 payload 的消费者
      正是 JS；
    * ``allow_nan=False`` 遇到非有限值**抛 ValueError**。调用点在健康监视器的
      定时器回调里，抛出去等于把整个监视器打死，而它正是
      ``/go2w/nav/localization_valid`` 的发布者 —— 之后后端看到的是"没人发布"，
      归因指向定位，真相却是"监视器自己崩了"。

    所以既不能糊过去，也不能抛：换掉，并把**换掉了哪些**记下来当证据。
    """
    offenders: list[str] = []

    def walk(node, where: str):
        if isinstance(node, float):
            if math.isfinite(node):
                return node
            offenders.append(where)
            return None
        if isinstance(node, dict):
            return {key: walk(item, f"{where}[{key!r}]") for key, item in node.items()}
        if isinstance(node, (list, tuple)):
            return [walk(item, f"{where}[{index}]") for index, item in enumerate(node)]
        return node

    return walk(value, path), offenders


def fresh(
    sample: dict,
    limit: float,
    *,
    stamp_limit: float | None = None,
    future_tolerance: float = 0.1,
) -> bool:
    """Require both a recent arrival and a plausible source timestamp.

    ``limit`` remains the fail-fast receive timeout.  Some sensors timestamp a
    scan at the beginning of acquisition, so their header can legitimately be
    older than the packet's local receive age.  ``stamp_limit`` lets callers
    account for that acquisition/transport latency without hiding an actual
    topic dropout.  By default the old, strict one-limit behaviour is kept.
    """
    age = sample.get("age_s")
    receive_age = sample.get("receive_age_s")
    maximum_stamp_age = limit if stamp_limit is None else stamp_limit
    return (isinstance(age, (int, float)) and math.isfinite(age)
            and -future_tolerance <= age <= maximum_stamp_age
            and isinstance(receive_age, (int, float)) and math.isfinite(receive_age)
            and 0 <= receive_age <= limit)


TF_FRESHNESS_SECONDS = 0.3
TF_STAMP_FRESHNESS_SECONDS = 0.4
TF_FUTURE_TOLERANCE_SECONDS = 0.3
TF_HEALTH_EDGES = ('map->odom_fused', 'odom_fused->base_link')
# Release18's bounded field trace measured a 0.254 s receive age and 0.361 s
# acquisition-stamp age during one 15 W scheduler stall.  Collision Monitor
# still owns the independent 0.30 s receive timeout; this health verdict uses
# the same receive boundary and accounts for the measured acquisition delay.
SAFETY_SCAN_RECEIVE_TIMEOUT_SECONDS = 0.30
SAFETY_SCAN_STAMP_TIMEOUT_SECONDS = 0.40
ODOM_RECEIVE_TIMEOUT_SECONDS = 0.2
ODOM_STAMP_TIMEOUT_SECONDS = 0.3


def tf_fresh(sample: dict) -> bool:
    """Use one TF time policy for health evidence and dynamic buffer input.

    The fused transform inherits the wheel-state acquisition timestamp.  The
    15 W field trace can therefore receive a new TF packet within 0.30 s while
    its source stamp is 0.30--0.35 s old.  Keep the receive timeout unchanged
    (a real publisher/transport dropout still fails closed at 0.30 s), but
    account for that measured acquisition latency in the stamp bound.
    """
    return fresh(
        sample,
        TF_FRESHNESS_SECONDS,
        stamp_limit=TF_STAMP_FRESHNESS_SECONDS,
        future_tolerance=TF_FUTURE_TOLERANCE_SECONDS,
    )


def _finite_triple(value) -> bool:
    return (isinstance(value, (list, tuple)) and len(value) == 3
            and all(isinstance(item, (int, float)) and math.isfinite(item) for item in value))


def observe_motion_since(sample: dict, odom: dict, *, continuous: bool) -> dict:
    """Keep maximum excursion since AMCL, including an out-and-back motion.

    Missing odometry observations invalidate the exemption until another AMCL
    sample establishes a new baseline. Endpoint equality alone is not history.
    """
    result = dict(sample)
    arrived = sample.get('odom_at_arrival')
    now = [odom.get(k) for k in ('x', 'y', 'yaw')]
    if not continuous or not _finite_triple(arrived) or not _finite_triple(now):
        result['motion_observation_complete'] = False
        return result
    result['max_position_excursion_m'] = max(sample.get('max_position_excursion_m', 0),
                                             math.hypot(now[0]-arrived[0], now[1]-arrived[1]))
    result['max_yaw_excursion_rad'] = max(sample.get('max_yaw_excursion_rad', 0),
        abs(math.atan2(math.sin(now[2]-arrived[2]), math.cos(now[2]-arrived[2]))))
    return result


def stationary_since(sample: dict, odom: dict, *,
                     position_tolerance: float = 0.02,
                     yaw_tolerance: float = 0.02) -> bool:
    """样本到达之后车体没动过（位置与偏航都在容差内）。

    这是 ``amcl_fresh`` 唯一允许陈旧的正当理由，理由来自 amcl 源码而不是猜测：
    ``nav2_amcl`` 1.1.20 的 ``laserReceived()`` 只在
    ``resampled || force_publication || !first_pose_sent_`` 时调
    ``publishAmclPose()``；车一停，``update_min_d/a``（本仓 0.01 m / 0.01 rad）就
    不再触发滤波更新，于是 **/amcl_pose 不再重发**，而同一函数里的
    ``else if (latest_tf_valid_)`` 分支仍在重发 ``map->odom_fused``。
    所以"静止 ⇒ 没有 /amcl_pose"是正常行为，拿 1 秒新鲜度去卡它等于给定位门
    判死：车一动门绿、一停门红。

    （顺带：``save_pose_rate`` 在 nav2 里是**死参数** —— 只在 1094/1254 行被赋值，
    再没有读取点。别指望它能让静止时周期性重发。）

    故意不用"没有 /amcl_pose 就放行"：那样 ``amcl`` 从未到达时也会绿。这里要求
    **到达时刻的 odom 位姿、此后完整的观测和最大偏移**都必须存在且在容差内。
    绕一圈回到原点、或中间丢过里程计，不能用端点相同冒充一直静止。
    """
    arrived = sample.get("odom_at_arrival")
    now = [odom.get("x"), odom.get("y"), odom.get("yaw")]
    for field, lower in (('age_s', -.1), ('receive_age_s', 0.)):
        value = sample.get(field, 0.)
        if type(value) not in (int, float) or not math.isfinite(value) or value < lower:
            return False
    excursions = [sample.get('max_position_excursion_m'), sample.get('max_yaw_excursion_rad')]
    if (sample.get('motion_observation_complete') is not True
            or any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in excursions)
            or excursions[0] > position_tolerance or excursions[1] > yaw_tolerance):
        return False
    if not _finite_triple(arrived) or not _finite_triple(now):
        return False
    moved = math.hypot(now[0] - arrived[0], now[1] - arrived[1])
    turned = abs(math.atan2(math.sin(now[2] - arrived[2]), math.cos(now[2] - arrived[2])))
    return moved <= position_tolerance and turned <= yaw_tolerance


#: 审计在不同模式下该查的生命周期节点。写死在这里而不是审计脚本里，是为了让这套
#: 口径能被离线测到 —— 审计脚本 import 时就 import rclpy，没起 ROS 的机器上根本
#: 进不去，而"模式口径搞错"恰恰是计划书 §P4-5 点的那个缺陷。
_AUDIT_LIFECYCLE_NODES = {
    "mapping": ("planner_server", "bt_navigator"),
    "localization": ("map_server", "amcl"),
}
_AUDIT_EXECUTION_NODES = ("controller_server", "velocity_smoother",
                          "collision_monitor", "behavior_server")


def lifecycle_manager_owners(node_names):
    """Ignore Foxy's companion client only when its owning manager exists."""
    names=set(node_names)
    return sorted(name for name in node_names
        if name.rsplit('/',1)[-1].startswith('lifecycle_manager')
        and not (name.endswith('_service_client')
                 and name[:-len('_service_client')] in names))


def audit_expectations(mode: str, *, require_execution: bool = False) -> dict:
    """运行时审计在 ``mode`` 下该期待什么。

    * ``lifecycle_nodes``：必须处于 active 的生命周期节点。
    * ``required_managers``：必须出现的 lifecycle_manager（全路径）。
    * ``max_lifecycle_managers``：允许多少个。mapping 下正常就是 1 个，多一个
      就是"两套并存而按名字查不出来"那类坏法；localization 下正常是 2 个
      （localization_2d 自己的 + 导航栈的 plan_only），所以按模式分开。
    """
    if mode not in _AUDIT_LIFECYCLE_NODES:
        # 不退回 mapping 口径：mode 拼错时"按 mapping 查、按 localization 判"会
        # 静默错配。调用方的 argparse choices 已经挡住，这里是第二道。
        raise ValueError("unknown audit mode: %r" % (mode,))
    nodes = list(_AUDIT_LIFECYCLE_NODES[mode])
    if require_execution:
        nodes += list(_AUDIT_EXECUTION_NODES)
        if mode == "localization":
            # 定位模式默认不查导航栈；一旦要求执行链，规划那两个也要一起查，
            # 否则会出现"控制链齐了但没人能规划"却被判健康。
            nodes += list(_AUDIT_LIFECYCLE_NODES["mapping"])
    if mode == "localization":
        return {"lifecycle_nodes": nodes,
                "required_managers": ["/lifecycle_manager_localization_2d"],
                "max_lifecycle_managers": 2}
    return {"lifecycle_nodes": nodes, "required_managers": [],
            "max_lifecycle_managers": 1}


def evaluate_localization(snapshot: dict[str, Any], *, mode: str = "mapping") -> dict:
    grid = snapshot.get("map") or {}
    scan = snapshot.get("scan") or {}
    odom = snapshot.get("odom") or {}
    transforms = snapshot.get("tf") or {}
    checks = {
        "map_unique": grid.get("publishers") == 1,
        "map_valid": grid.get("valid") is True and grid.get("frame") == "map",
        "map_current": mode == "localization" or fresh(grid, 3.0),
        "scan_unique": scan.get("publishers") == 1,
        # Pandar scans are timestamped near acquisition start.  On the live
        # Go2-W path their header age is normally 0.16--0.23 s even though the
        # packet usually arrives only 0--0.06 s later.  The release08 bounded
        # trace saw one 0.221 s arrival / 0.30005 s header interval while the
        # independent collision monitor was still inside its 0.30 s source
        # timeout.  Keep this upstream gate tighter than that final stop layer
        # while allowing the measured scheduler jitter.
        "scan_fresh": fresh(
            scan,
            SAFETY_SCAN_RECEIVE_TIMEOUT_SECONDS,
            stamp_limit=SAFETY_SCAN_STAMP_TIMEOUT_SECONDS,
        ),
        "scan_returns": int(scan.get("valid_ranges", 0)) >= 10,
        "scan_monotonic": scan.get("stamp_regressions", 0) == 0,
        "odom_unique": odom.get("publishers") == 1,
        # The release14 trace received fused odometry only 0.004--0.033 s ago
        # while the inherited source stamp was 0.221--0.227 s old.  Separate
        # acquisition latency from topic loss: arrival remains fail-fast at
        # 0.20 s, whereas the source stamp may be up to 0.30 s old.
        "odom_fresh": fresh(
            odom,
            ODOM_RECEIVE_TIMEOUT_SECONDS,
            stamp_limit=ODOM_STAMP_TIMEOUT_SECONDS,
        ),
        "odom_frames": odom.get("frame") == "odom_fused" and odom.get("child") == "base_link",
        "pose_finite": odom.get("finite") is True,
        "localizer_active": snapshot.get("localizer_active") is True,
        "nodes_unique": not snapshot.get("duplicate_nodes"),
        "no_pose_jump": snapshot.get("pose_jump") is False,
        "tf_future_cache_recovered": not snapshot.get('tf_future_recovery_pending', []),
    }
    for edge in TF_HEALTH_EDGES:
        transform = transforms.get(edge) or {}
        checks[edge + ":unique"] = transform.get("authorities") == 1
        checks[edge + ":fresh"] = tf_fresh(transform)
        checks[edge + ":finite"] = transform.get("finite") is True
    structural_scan = snapshot.get("mapping_scan") or {}
    checks["mapping_scan_unique"] = structural_scan.get("publishers") == 1
    checks["mapping_scan_fresh"] = fresh(
        structural_scan,
        SAFETY_SCAN_RECEIVE_TIMEOUT_SECONDS,
        stamp_limit=SAFETY_SCAN_STAMP_TIMEOUT_SECONDS,
    )
    checks["mapping_scan_returns"] = structural_scan.get("valid_ranges", 0) >= 10
    checks["mapping_scan_monotonic"] = structural_scan.get("stamp_regressions", 0) == 0
    basis = None
    if mode == "localization":
        amcl = snapshot.get("amcl") or {}
        if fresh(amcl, 1.0):
            basis = "fresh"
        elif stationary_since(amcl, odom):
            basis = "stationary_exempt"
        else:
            basis = "stale"
        checks["amcl_fresh"] = basis != "stale"
        covariance = amcl.get("covariance", [])
        checks["amcl_covariance"] = (
            len(covariance) == 3 and all(math.isfinite(x) and 0 <= x <= limit
                                       for x, limit in zip(covariance, (0.25, 0.25, 0.12))))
        checks["map_authorized"] = snapshot.get("map_authorized") is True
    blocking = [name for name, ok in checks.items() if not ok]
    result = {"valid": not blocking, "mode": mode, "checks": checks, "blocking": blocking}
    if basis is not None:
        # 这是证据不是判据：现场要能看出这扇门是"刚收到位姿"绿的，还是"车没动
        # 所以没有新位姿"绿的 —— 后者一旦被读成前者，就会把一台其实没在定位的
        # 车当成定位正常。它不进 checks，因为豁免被用上时它不是"错"。
        result["amcl_fresh_basis"] = basis
    return result
