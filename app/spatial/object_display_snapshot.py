"""§2.3 的文件层契约：轻量地图元数据 与 在线对象快照。

一份地图由两个进程协作显示，它们之间**只通过这两个文件**说话：

* 地图桥写 ``slam_map_meta.json``（本模块的 :func:`meta_payload` /
  ``load_map_meta``）—— 只有 ``map_epoch`` / ``frame_id`` / ``map_revision`` /
  ``generated_at`` 四个键，**不含点云**。对象生产者读它来知道自己该声明属于
  哪张地图；否则每写一次快照都要解析几万点的大地图 JSON。
* 对象生产者写 ``semantic_objects.json``（:func:`build_object_snapshot` 造形状、
  :func:`write_object_snapshot` 落盘）—— 显示对象 + 它声明的 ``map_epoch``。
* Web 端（``app/manual_web_demo/slam_map_snapshot.py``）读这两份，只有两个标识
  相等时才把对象叠到地图上。

两个 ``load_*`` 都返回 ``None`` 表示"这次没得用"，而不是抛错或造一个空快照：
调用方对"文件不存在"和"内容坏了"的处置完全相同（对象不上屏），多分一档没有
任何行为差异。吸收的异常刻意只有 ``OSError`` / ``ValueError`` / ``TypeError``
这一组明确的输入错误，不是宽泛吞没。
"""

from __future__ import annotations

import json
import os
import threading
import time
from pathlib import Path
from typing import Any

from app.spatial.semantic_object_gate import (
    OBJECT_SNAPSHOT_SCHEMA,
    build_object_snapshot,
)

#: 轻量元数据文件里允许出现的全部字段。刻意只有这四个：地图健康、点云统计等
#: 仍留在大地图快照里，避免元数据长成第二份地图模型。
META_KEYS = ("map_epoch", "frame_id", "map_revision", "generated_at")

#: 地图快照与对象快照的默认文件名，两个模式共用同一套名字、靠目录区分。
MAP_META_NAME = "slam_map_meta.json"
OBJECT_SNAPSHOT_NAME = "semantic_objects.json"


def meta_payload(payload: dict[str, Any]) -> dict[str, Any]:
    """从任意一份 v2 地图快照里取出轻量元数据（两个地图桥共用）。"""
    return {key: payload.get(key) for key in META_KEYS}


def load_map_meta(path: str | os.PathLike[str]) -> dict[str, Any] | None:
    """读地图桥写的轻量元数据；拿不到可用的 ``map_epoch`` 就返回 None。"""
    data = _load_json_object(path)
    if data is None:
        return None
    epoch = str(data.get("map_epoch") or "")
    frame = str(data.get("frame_id") or "")
    if not epoch or not frame:
        return None
    return data


def load_object_snapshot(path: str | os.PathLike[str]) -> dict[str, Any] | None:
    """读对象生产者写的显示快照；形状不对就当作"没有对象快照"。"""
    data = _load_json_object(path)
    if data is None or data.get("schema_version") != OBJECT_SNAPSHOT_SCHEMA:
        return None
    if not str(data.get("map_epoch") or ""):
        return None
    return data


class ObjectDisplayPublisher:
    """§2.3：把内存里的持久实体表投影成上屏对象并原子覆写。

    自主搜索（``AutonomousExplorer.semantic_graph.object_map``）与遥控语义建图
    （``remote_control_semantic_mapping_worker`` 的 ``self._semantic_map``）各持
    一个实例，**调的是同一个投影函数**，显示门因此只有一份实现。

    epoch 每次发布都从 meta 现读：地图桥换了 epoch，这里就跟着换。**不锁存**。

    锁存看起来更安全（"旧坐标不许冒用新地图"），实际会造出一个更糟的失败模式：
    SLAM reset 后生产者若在桥写出新 epoch 之前发布过一次，就会永久声明旧 epoch，
    此后整场会话一个物体都不上屏，而页面上看不出任何原因。真正防鬼影的是
    **读取端的配对判据**：一场已结束的搜索留下的对象快照带着它那时的 epoch，
    新地图对不上，就不画 —— 这才是"旧对象立即不上屏"要挡的常见情形。
    未覆盖的只有"搜索进行中 SLAM 被 reset"：那种情形下搜索本身已经不可用
    （恢复要重起 SLAM + wheel_odom + Nav2），不属于受支持的运行状态。
    """

    def __init__(self, *, meta_path: str | os.PathLike[str],
                 snapshot_path: str | os.PathLike[str],
                 producer_session_id: str = "",
                 confirmed_min_confidence: float | None = None,
                 confirm_min_observations: int | None = None,
                 topology_accepted_only: bool = False) -> None:
        self.publisher_meta_path = Path(meta_path)
        self.snapshot_path = Path(snapshot_path)
        self.producer_session_id = str(producer_session_id)
        self.confirmed_min_confidence = confirmed_min_confidence
        self.confirm_min_observations = confirm_min_observations
        self.topology_accepted_only = bool(topology_accepted_only)
        self.epoch = ""
        self.frame_id = ""
        self.last_revision: int | None = None

    def publish(self, records: Any, *, graph_revision: int) -> bool:
        """实体表版本变化时覆写对象快照；没有合法 map epoch 时什么都不写。"""
        revision = int(graph_revision)
        if revision == self.last_revision:
            return False
        meta = load_map_meta(self.publisher_meta_path)
        if meta is None:
            # 地图桥还没写出元数据（或写坏了）——没有 epoch 就不上屏，不猜。
            return False
        self.epoch = str(meta["map_epoch"])
        self.frame_id = str(meta.get("frame_id") or "")
        options: dict[str, Any] = {
            "topology_accepted_only": self.topology_accepted_only,
        }
        if self.confirmed_min_confidence is not None:
            options["confirmed_min_confidence"] = self.confirmed_min_confidence
        if self.confirm_min_observations is not None:
            options["confirm_min_observations"] = self.confirm_min_observations
        snapshot = build_object_snapshot(
            records,
            map_epoch=self.epoch,
            frame_id=self.frame_id,
            producer_session_id=self.producer_session_id,
            graph_revision=revision,
            **options,
        )
        write_object_snapshot(self.snapshot_path, snapshot)
        self.last_revision = revision
        return True


def write_object_snapshot(path: str | os.PathLike[str],
                          snapshot: dict[str, Any]) -> None:
    """原子覆写对象快照（临时文件 + ``os.replace``）。

    Web 端可能正好在这一刻读这个文件；非原子写会让它读到半截 JSON，而那看起来
    与"没有对象快照"一模一样 —— 页面上的表现是物体闪一下消失，取证时无从分辨。
    """
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + f".{os.getpid()}.{threading.get_ident()}.{time.time_ns()}.tmp")
    temporary.write_text(
        json.dumps(snapshot, ensure_ascii=False, separators=(",", ":")),
        encoding="utf-8",
    )
    os.replace(temporary, target)


def _load_json_object(path: str | os.PathLike[str]) -> dict[str, Any] | None:
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None
    except (OSError, ValueError, TypeError):
        return None
    return data if isinstance(data, dict) else None
