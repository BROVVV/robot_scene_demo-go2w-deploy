#!/usr/bin/env python3
"""离线验证 §13.5 的 last-good 冻结 + §16.3 的 map_status 三值（不移动狗）。

计划书 §29 的硬停止条件里有一条「``mapping_health = DEGRADED`` 但 permanent map
仍继续写」。本脚本证明它不再成立：判据一锁存，写端就停止用新点云顶掉 last-good。

**为什么要单独一个脚本而不是 pytest**：被验证的逻辑住在
``webui_map_snapshot_writer.py`` 的 ``_flush`` 里，那是个 rclpy 节点，而离线测试
环境（``.venv``）没有 rclpy。判据本身的分辨力在
``tests/test_lio_nofix_gate_calibration.py``，判据→闸门的契约在
``tests/test_slam_map_snapshot_health_contract.py``，**冻结这一段只能在这里验**。

跑法（必须 source 环境，节点要建 DDS 参与者）：

    source scripts/go2w/setup_environment.sh
    /usr/bin/python3 scripts/go2w/check_snapshot_freeze.py

覆盖六种情形，全绿才打印 ``ALL FREEZE CHECKS PASSED``、否则非零退出：

    ① HEALTHY 帧          -> 攒下 last-good、map_status=LIVE
    ② 转 DEGRADED         -> 点云/revision/ros_stamp/source_map_points 逐位等于
                             last-good，而 generated_at 照常前进（所以前端算出的
                             fresh 仍为真：故意冻结 != 写端死了）
    ②b 锁存               -> 触发后再来 300 帧好帧，必须仍冻结（判据会自己闭嘴）
    ②c 唯一复位路径       -> 日志被截断（LIO 重启）才回 HEALTHY、session_id 递增
    ③ 无 last-good 的 DEGRADED -> SUSPECT，**不是** LIVE（报 LIVE 是假绿）
    ④ 判据不表态          -> 四个键一个都不写（判据没在跑时不许显示成绿）
"""

from __future__ import annotations

import json
import os
import shutil
import sys
import tempfile
import time

_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, _PROJECT_ROOT)
sys.path.insert(0, os.path.join(_PROJECT_ROOT, "scripts", "go2w"))

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from builtin_interfaces.msg import Time as RosTime  # noqa: E402
from sensor_msgs.msg import PointCloud2, PointField  # noqa: E402

from webui_map_snapshot_writer import WebuiSnapshotWriter  # noqa: E402

GOOD = ("[lio_3d_node-2] [GO2W-DIAG] imu_n= 25 pt=0.1012 w=0.023 act=0.904 "
        "iter=3 eps=0.0050 dsk_max=0.1391 dsk_mean=0.0682 dsk_zero=0 "
        "pre_t=0.1428 pre_r=0.088 fix_t=0.0390 fix_r=0.212 act_t=0.1049 act_r=0.282")
BAD = ("[lio_3d_node-2] [GO2W-DIAG] imu_n= 25 pt=0.1012 w=0.023 act=0.104 "
       "iter=10 eps=0.0050 dsk_max=0.1391 dsk_mean=0.0682 dsk_zero=0 "
       "pre_t=0.1428 pre_r=0.088 fix_t=0.0000 fix_r=0.212 act_t=0.1049 act_r=0.282 "
       "pos=(12.34,-5.67,0.89)")

FAILURES: list[str] = []


def check(name: str, got, want) -> None:
    ok = got == want
    print(f"  [{'OK  ' if ok else 'FAIL'}] {name}  got={got!r} want={want!r}")
    if not ok:
        FAILURES.append(name)


def cloud(arr: np.ndarray, stamp: float) -> PointCloud2:
    msg = PointCloud2()
    msg.height, msg.width = 1, int(len(arr))
    msg.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(("x", "y", "z"))]
    msg.is_bigendian = False
    msg.point_step, msg.row_step = 16, 16 * int(len(arr))
    buf = np.zeros((len(arr), 4), dtype=np.float32)
    buf[:, :3] = arr
    msg.data = buf.tobytes()
    msg.is_dense = True
    msg.header.frame_id = "pslam_map"
    msg.header.stamp = RosTime(sec=int(stamp), nanosec=int(round((stamp % 1) * 1e9)))
    return msg


def make_writer(tmp: str, log_path: str) -> WebuiSnapshotWriter:
    """每个用例一个干净节点。参数走 ROS 覆盖，不改任何生产代码。"""
    rclpy.init(args=[
        "--ros-args",
        "-p", f"snapshot_path:={os.path.join(tmp, 'slam_map_3d.json')}",
        "-p", f"lio_diag_log:={log_path}",
        "-p", f"trajectory_path:={os.path.join(tmp, 'traj.json')}",
        "-p", "trajectory_radius_m:=0.0",   # 空值的 -p 解析不了，用半径 0 关掉轨迹层
        "-p", f"meta_path:={os.path.join(tmp, 'slam_map_meta.json')}",
    ])
    return WebuiSnapshotWriter()


def read(tmp: str) -> dict:
    with open(os.path.join(tmp, "slam_map_3d.json"), encoding="utf-8") as handle:
        return json.load(handle)


def append(log_path: str, line: str, times: int) -> None:
    with open(log_path, "a", encoding="utf-8") as handle:
        for _ in range(times):
            handle.write(line + "\n")


# 三朵点云都必须落在 writer 的量程/高度过滤器内（1~25 m、z 0.15~2.80），
# 否则会被滤成 0 点，测的就不是冻结而是过滤器了。
A = np.array([[1.0, 2.0, 0.5], [2.0, 1.0, 0.7], [3.0, 3.0, 1.1]], dtype=np.float32)
B = A + np.array([5.0, 5.0, 0.0], dtype=np.float32)  # 发散后的点云：水平平移 5 m
# 高度不跟着平移：writer 还有一道 z≤2.80 的上限，抬高了会被滤成 0 点，
# 那测的就成了过滤器而不是冻结。
C = A + 0.5         # 复位后的新一帧

# ---------------------------------------------------------------- ①② 正路 + 冻结
print("\n== ①② HEALTHY 攒 last-good → DEGRADED 冻结 ==")
tmp1 = tempfile.mkdtemp(prefix="freeze_")
log1 = os.path.join(tmp1, "launch.log")
open(log1, "w").close()
append(log1, GOOD, 12)
node1 = make_writer(tmp1, log1)
node1._on_cloud(cloud(A, 100.5))
node1._flush()
snap = read(tmp1)
print("  ① 健康帧")
check("mapping_health", snap.get("mapping_health"), "HEALTHY")
check("lio_pose_valid", snap.get("lio_pose_valid"), True)
check("map_status", snap.get("map_status"), "LIVE")
check("health_reason 不写", "health_reason" in snap, False)
check("mapping_session_id", snap.get("mapping_session_id"), 0)
check("last-good 已攒下", node1._last_good_map is not None, True)
good_points, good_rev = snap["points"], snap["map_revision"]
good_stamp, good_gen = snap["ros_stamp"], snap["generated_at"]
good_src = snap["source_map_points"]
check("点云是这一帧的", len(good_points), 3)

append(log1, BAD, 10)
node1._on_cloud(cloud(B, 200.5))     # revision 递增、点云换了
time.sleep(0.05)
node1._flush()
snap2 = read(tmp1)
print("  ② 转 DEGRADED")
check("mapping_health", snap2.get("mapping_health"), "DEGRADED_LIO_DRIFT")
check("lio_pose_valid", snap2.get("lio_pose_valid"), False)
check("map_status", snap2.get("map_status"), "FROZEN_LAST_GOOD")
check("点云逐位等于 last-good", snap2["points"], good_points)
check("map_revision 冻住", snap2.get("map_revision"), good_rev)
check("ros_stamp 冻住", snap2.get("ros_stamp"), good_stamp)
check("source_map_points 冻住", snap2.get("source_map_points"), good_src)
check("generated_at 照常前进", snap2["generated_at"] > good_gen, True)
check("health_reason 非空", bool(snap2.get("health_reason")), True)
print("     reason =", snap2.get("health_reason"))

# ------------------------------------------------- ②b 好帧不许把闸门骗开（核心）
print("\n== ②b 锁存：触发之后再来 300 帧好帧，必须仍然冻结 ==")
append(log1, GOOD, 300)
time.sleep(0.05)
node1._flush()
snap3 = read(tmp1)
check("mapping_health 仍是 DEGRADED", snap3.get("mapping_health"), "DEGRADED_LIO_DRIFT")
check("map_status 仍是 FROZEN_LAST_GOOD", snap3.get("map_status"), "FROZEN_LAST_GOOD")
check("点云仍是 last-good", snap3["points"], good_points)

# --------------------------------------------- ②c 唯一复位路径：日志截断 == 换会话
print("\n== ②c 唯一复位路径：日志被截断（LIO 重启）→ 回 HEALTHY ==")
with open(log1, "w", encoding="utf-8") as handle:   # 截断；比上次短才判得出
    handle.write(GOOD + "\n")
append(log1, GOOD, 12)
node1._on_cloud(cloud(C, 300.5))
node1._flush()
snap4 = read(tmp1)
check("mapping_session_id 递增", snap4.get("mapping_session_id"), 1)
check("mapping_health 复位", snap4.get("mapping_health"), "HEALTHY")
check("map_status 回 LIVE", snap4.get("map_status"), "LIVE")
check("点云重新跟着新帧走", snap4["points"],
      [[round(float(v), 3) for v in row] for row in C])
node1.destroy_node()
rclpy.shutdown()
shutil.rmtree(tmp1, ignore_errors=True)

# ------------------------------------------ ③ 一起床就是 DEGRADED 且没有 last-good
print("\n== ③ 无 last-good 的 DEGRADED：必须 SUSPECT，不许报 LIVE ==")
tmp2 = tempfile.mkdtemp(prefix="suspect_")
log2 = os.path.join(tmp2, "launch.log")
open(log2, "w").close()
append(log2, BAD, 12)
node2 = make_writer(tmp2, log2)
node2._on_cloud(cloud(B, 400.5))
node2._flush()
snap5 = read(tmp2)
check("mapping_health", snap5.get("mapping_health"), "DEGRADED_LIO_DRIFT")
check("map_status", snap5.get("map_status"), "SUSPECT")
check("没冻（无 last-good 可冻），发的是本帧", len(snap5["points"]), 3)
check("第一点确实是本帧的", snap5["points"][0][0], round(float(B[0][0]), 3))
node2.destroy_node()
rclpy.shutdown()
shutil.rmtree(tmp2, ignore_errors=True)

# ------------------------------------------------------ ④ 判据不表态（日志不存在）
print("\n== ④ 判据没在跑（日志路径不对）：三个键一个都不许写 ==")
tmp3 = tempfile.mkdtemp(prefix="silent_")
node3 = make_writer(tmp3, os.path.join(tmp3, "does_not_exist.log"))
node3._on_cloud(cloud(C, 500.5))
node3._flush()
snap6 = read(tmp3)
for key in ("mapping_health", "lio_pose_valid", "map_status", "health_reason"):
    check(f"{key} 不写", key in snap6, False)
check("地图照常写", len(snap6["points"]), 3)
node3.destroy_node()
rclpy.shutdown()
shutil.rmtree(tmp3, ignore_errors=True)

print("\n" + ("ALL FREEZE CHECKS PASSED" if not FAILURES
              else f"{len(FAILURES)} CHECK(S) FAILED: {FAILURES}"))
sys.exit(1 if FAILURES else 0)
