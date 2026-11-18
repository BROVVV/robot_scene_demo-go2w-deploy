"""三维对象的来源与配对：**只有 map_epoch 和 frame_id 同时相等才叠画**。

回归背景（2026-09-17，用户反馈"地图上有很久之前扫描过的几个物体的点"）：三维视图
画的是 09-10 归档里的 7 个物体，当天搜索刚定位到的 20 个一个都没上屏。当时写端的
选源办法是"glob 一遍 events.jsonl ∪ 写死的归档，取 mtime 最新的那一份" —— 而这个
办法**永远猜不准**：坐标属于哪张地图是数据里的事实，不是文件时间能推断的。

2026-10-08（计划书 §2.3）改成：地图桥写一份轻量 ``slam_map_meta.json`` 声明
``map_epoch``/``frame_id``，语义进程读它、把同一对标识写进 ``semantic_objects.json``，
读取端只认两个都相等的一对。**文件时间、坐标包围盒不再参与任何判断。**

本文件测三件事，每一件都要能区分"改对了"和"看起来改对了"：

1. 写端不再有从 events 选源的本事（那个接口已删，回来就是又一个会猜错的来源）。
2. 错配**绝不**叠画 —— 这是本文件的判别性用例：两个快照的点云、物体、统计全都
   齐全、看着完全正常，唯一的区别是 epoch 不同；只有真按 epoch 判的实现才会返回空。
3. 配对成功时保留原有显示契约（``label_zh``、合法 box、过滤统计），别为了配对把
   显示门绕过去。
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from app.manual_web_demo.slam_map_snapshot import (
    OVERLAY_EPOCH_MISMATCH,
    OVERLAY_FRAME_MISMATCH,
    OVERLAY_NO_SNAPSHOT,
    OVERLAY_OK,
    load_slam_map_snapshot,
)
from app.spatial.object_display_snapshot import (
    ObjectDisplayPublisher,
    load_object_snapshot,
    write_object_snapshot,
)
from app.spatial.semantic_object_gate import build_object_snapshot

WRITER = (Path(__file__).resolve().parents[1]
          / "scripts" / "go2w" / "webui_map_snapshot_writer.py")

EPOCH = "abcdef12-3"
FRAME = "pslam_map"


def _map_snapshot(path: Path, *, epoch: str = EPOCH, frame: str = FRAME) -> Path:
    path.write_text(json.dumps({
        "schema_version": "go2w_slam_web_cloud_v2",
        "map_epoch": epoch,
        "frame_id": frame,
        "points": [[1.0, 2.0, 3.0]],
        "point_count": 1,
        "available": True,
        "generated_at": 0.0,
    }), encoding="utf-8")
    return path


def _entity(label: str, *, x: float, confidence: float = 0.9,
            observations: int = 5) -> dict:
    """一条 ``SemanticObjectEntry.to_dict()`` 形状的记录。"""
    return {
        "object_id": "obj_001", "label": label, "label_zh": label,
        "map_xyz": [x, 0.5, 0.09], "box_size": [0.3, 0.3, 0.6],
        "provenance": {"map_frame": FRAME},
        "confidence": confidence, "confidence_max": confidence,
        "observation_count": observations, "status": "CONFIRMED",
    }


def _object_snapshot(path: Path, *, epoch: str = EPOCH, frame: str = FRAME,
                     label: str = "垃圾桶") -> Path:
    write_object_snapshot(path, build_object_snapshot(
        [_entity(label, x=2.0)],
        map_epoch=epoch, frame_id=frame,
        producer_session_id="search_20261008_120000_deadbeef", graph_revision=12,
    ))
    return path


# --------------------------------------------------------------------------- #
# 1. 从 events 选源的能力已经删掉
# --------------------------------------------------------------------------- #
def test_the_writer_no_longer_picks_objects_from_events() -> None:
    """写端只写地图。glob/选最新文件这条路已被证伪两次，不要加回来。

    断言的名字都是**仍然存在**的在线/离线接口（`load_object_records`、
    `select_display_objects` 都还在仓库里），所以"写端不引用它们"是一条
    真会变红的判据 —— 断言一个已经被删掉的名字则是恒真的空话。
    """
    source = WRITER.read_text(encoding="utf-8")
    for gone in ("_load_objects", "objects_glob", "objects_events",
                 "load_object_records", "select_display_objects"):
        assert gone not in source, f"{gone} 仍然出现在快照写入器里"
    # 地图 payload 里也不再构造 objects —— 对象由读取端按 epoch 合成。
    assert '"objects":' not in source


# --------------------------------------------------------------------------- #
# 2. 配对判据（本文件的判别性用例）
# --------------------------------------------------------------------------- #
def test_same_epoch_and_frame_overlays(tmp_path: Path) -> None:
    result = load_slam_map_snapshot(
        _map_snapshot(tmp_path / "map.json"),
        _object_snapshot(tmp_path / "objects.json"),
    )
    assert result["object_overlay_status"] == OVERLAY_OK
    assert [item["object_id"] for item in result["objects"]] == ["obj_001"]
    assert result["object_filter_stats"]["displayed"] == 1


def test_a_different_epoch_never_overlays(tmp_path: Path) -> None:
    """同一个 frame_id、同一份点云，只有 epoch 不同 —— 必须一个都不画。

    这是"靠 mtime/包围盒猜来源"与"按 epoch 判"唯一能分开的地方：两种实现的
    点云、物体、统计全都一样，只有这一个断言会变红。
    """
    result = load_slam_map_snapshot(
        _map_snapshot(tmp_path / "map.json", epoch="abcdef12-3"),
        _object_snapshot(tmp_path / "objects.json", epoch="abcdef12-4"),
    )
    assert result["object_overlay_status"] == OVERLAY_EPOCH_MISMATCH
    assert result["objects"] == []
    assert result["object_filter_stats"]["displayed"] == 0
    # 地图本身照常可见：没配上对象不等于这张地图不能看。
    assert result["available"] is True


def test_a_different_frame_never_overlays(tmp_path: Path) -> None:
    result = load_slam_map_snapshot(
        _map_snapshot(tmp_path / "map.json"),
        _object_snapshot(tmp_path / "objects.json", frame="odom"),
    )
    assert result["object_overlay_status"] == OVERLAY_FRAME_MISMATCH
    assert result["objects"] == []


def test_no_object_source_still_shows_the_map(tmp_path: Path) -> None:
    """手动演示模式没有语义建图进程：地图照看，不显示历史归档物体。"""
    map_path = _map_snapshot(tmp_path / "map.json")
    result = load_slam_map_snapshot(map_path, tmp_path / "none" / "objects.json")
    assert result["object_overlay_status"] == OVERLAY_NO_SNAPSHOT
    assert result["objects"] == []
    assert result["available"] is True
    # 不传对象路径（就绪页那条调用）也必须是同一套形状，前端不会读到 undefined。
    assert load_slam_map_snapshot(map_path)["object_overlay_status"] == OVERLAY_NO_SNAPSHOT


def test_a_snapshot_without_epoch_is_not_usable(tmp_path: Path) -> None:
    """没有 epoch 的对象快照不算"当前地图的对象" —— 宁可当作没有。"""
    path = tmp_path / "objects.json"
    path.write_text(json.dumps({
        "schema_version": "semantic_object_display_v1",
        "objects": [{"object_id": "obj_001"}],
    }), encoding="utf-8")
    assert load_object_snapshot(path) is None
    result = load_slam_map_snapshot(_map_snapshot(tmp_path / "map.json"), path)
    assert result["object_overlay_status"] == OVERLAY_NO_SNAPSHOT


# --------------------------------------------------------------------------- #
# 3. 配对成功时保留原有显示契约
# --------------------------------------------------------------------------- #
def test_overlaid_objects_keep_the_display_contract(tmp_path: Path) -> None:
    path = tmp_path / "objects.json"
    write_object_snapshot(path, build_object_snapshot(
        [
            _entity("垃圾桶", x=2.0),
            # 碎片框（只见过一次、分数一般）仍被显示门挡下 —— 配对不能把门绕过去。
            _entity("紫色", x=3.0, confidence=0.5, observations=1),
            # 没有合法坐标的实体画不出来。
            {**_entity("办公椅", x=0.0), "map_xyz": None},
        ],
        map_epoch=EPOCH, frame_id=FRAME, graph_revision=3,
    ))
    result = load_slam_map_snapshot(_map_snapshot(tmp_path / "map.json"), path)
    assert result["object_overlay_status"] == OVERLAY_OK
    displayed = result["objects"]
    assert [item["label_zh"] for item in displayed] == ["垃圾桶"]
    item = displayed[0]
    assert item["box_valid"] is True and item["size"] == [0.3, 0.3, 0.6]
    assert item["decision"] == "DISPLAY"
    assert result["object_filter_stats"]["input"] == 3
    assert result["object_filter_stats"]["filtered"] == 2


def test_entity_coordinates_must_really_belong_to_the_declared_frame(tmp_path: Path) -> None:
    wrong = {**_entity("垃圾桶", x=2.0),
             "provenance": {"map_frame": "pslam_odom"}}
    snapshot = build_object_snapshot(
        [wrong, _entity("办公椅", x=3.0)],
        map_epoch=EPOCH, frame_id=FRAME,
    )
    assert [item["label_zh"] for item in snapshot["objects"]] == ["办公椅"]
    assert snapshot["object_filter_stats"]["dropped"]["map_frame_mismatch"] == 1


# --------------------------------------------------------------------------- #
# 4. 生产者：没有 epoch 就不写，同版本不重写
# --------------------------------------------------------------------------- #
def test_the_publisher_writes_nothing_without_a_map_epoch(tmp_path: Path) -> None:
    publisher = ObjectDisplayPublisher(
        meta_path=tmp_path / "none" / "slam_map_meta.json",
        snapshot_path=tmp_path / "objects.json",
    )
    assert publisher.publish([_entity("垃圾桶", x=1.0)], graph_revision=1) is False
    assert not (tmp_path / "objects.json").exists()


def test_the_publisher_skips_an_unchanged_object_revision(tmp_path: Path) -> None:
    meta = tmp_path / "meta.json"
    meta.write_text(json.dumps({
        "map_epoch": EPOCH, "frame_id": FRAME,
        "map_revision": 7, "generated_at": 0.0,
    }), encoding="utf-8")
    snapshot_path = tmp_path / "objects.json"
    publisher = ObjectDisplayPublisher(
        meta_path=meta, snapshot_path=snapshot_path,
        producer_session_id="search_x",
    )
    assert publisher.publish([_entity("垃圾桶", x=1.0)], graph_revision=4) is True
    written = snapshot_path.read_text(encoding="utf-8")
    assert publisher.publish([_entity("垃圾桶", x=9.0)], graph_revision=4) is False
    assert snapshot_path.read_text(encoding="utf-8") == written
    # epoch 换了（桥重启 / SLAM reset）就跟着换：这是"现在写的是哪张地图"。
    meta.write_text(json.dumps({
        "map_epoch": "ffffffff-9", "frame_id": FRAME,
        "map_revision": 1, "generated_at": 0.0,
    }), encoding="utf-8")
    assert publisher.publish([_entity("垃圾桶", x=1.0)], graph_revision=5) is True
    assert load_object_snapshot(snapshot_path)["map_epoch"] == "ffffffff-9"


@pytest.mark.parametrize("meta_body", ["not json at all", "[]", '{"frame_id":"pslam_map"}'])
def test_a_broken_meta_is_treated_as_no_epoch(tmp_path: Path, meta_body: str) -> None:
    """写坏的元数据 → 不写对象快照，而不是编一个 epoch 出来。"""
    meta = tmp_path / "meta.json"
    meta.write_text(meta_body, encoding="utf-8")
    publisher = ObjectDisplayPublisher(
        meta_path=meta, snapshot_path=tmp_path / "objects.json",
    )
    assert publisher.publish([_entity("垃圾桶", x=1.0)], graph_revision=1) is False
