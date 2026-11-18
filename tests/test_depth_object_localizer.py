"""Tests for DepthObjectLocalizer using synthetic depth images."""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import pytest

from app.perception.depth_object_localizer import (
    SPATIAL_QUALITY_CAMERA_LOCAL,
    SPATIAL_QUALITY_RGB_ONLY,
    DepthObjectLocalizer,
)
from app.perception.rgbd_source import RGBDFrame


def _write_depth(path: Path, depth_mm: np.ndarray) -> None:
    cv2.imwrite(str(path), depth_mm.astype(np.uint16))


def _rect_mask_rle(row0: int, col0: int, rows: int, cols: int) -> list[list[int]]:
    """[row, start_x, length] 行游程编码一个实心矩形（计划书 §5）。"""
    return [[row0 + i, col0, cols] for i in range(rows)]


def _frame(tmp_path: Path, depth_mm: np.ndarray) -> RGBDFrame:
    depth_path = tmp_path / "depth.png"
    _write_depth(depth_path, depth_mm)
    height, width = depth_mm.shape
    return RGBDFrame(
        frame_id="f1",
        timestamp=0.0,
        color_ref=str(tmp_path / "color.jpg"),
        depth_ref=str(depth_path),
        width=width,
        height=height,
        fx=50.0,
        fy=50.0,
        cx=width / 2.0,
        cy=height / 2.0,
        depth_unit_m=0.001,
    )


def test_localize_constant_depth_box(tmp_path: Path):
    depth_mm = np.full((64, 64), 0, dtype=np.uint16)
    depth_mm[20:44, 20:44] = 2000
    frame = _frame(tmp_path, depth_mm)
    localizer = DepthObjectLocalizer()
    objects = [{"label": "box", "label_zh": "箱子", "bbox_2d": [0.3, 0.3, 0.7, 0.7], "confidence": 0.9}]
    out = localizer.localize(objects, frame)
    assert len(out) == 1
    assert out[0].spatial_quality == SPATIAL_QUALITY_CAMERA_LOCAL
    assert out[0].depth_m == pytest.approx(2.0, abs=0.02)
    assert out[0].camera_xyz is not None
    assert out[0].bearing_deg is not None


def test_localize_mixed_invalid_depth_uses_median(tmp_path: Path):
    depth_mm = np.full((64, 64), 0, dtype=np.uint16)
    depth_mm[20:44, 20:44] = 1500
    depth_mm[22:24, 22:24] = 0
    depth_mm[26:28, 26:28] = 9999
    frame = _frame(tmp_path, depth_mm)
    localizer = DepthObjectLocalizer()
    objects = [{"label": "box", "bbox_2d": [0.3, 0.3, 0.7, 0.7]}]
    out = localizer.localize(objects, frame)
    assert out[0].depth_m == pytest.approx(1.5, abs=0.02)


def test_localize_rgb_only_when_depth_missing(tmp_path: Path):
    frame = RGBDFrame(
        frame_id="f2", timestamp=0.0, color_ref="", depth_ref="", width=64, height=64,
        fx=50.0, fy=50.0, cx=32.0, cy=32.0,
    )
    localizer = DepthObjectLocalizer()
    out = localizer.localize([{"label": "box", "bbox_2d": [0.3, 0.3, 0.7, 0.7]}], frame)
    assert out[0].spatial_quality == SPATIAL_QUALITY_RGB_ONLY
    assert out[0].depth_m is None


def test_real_desk_frame_uses_stable_dominant_depth(tmp_path: Path):
    fixture = (
        Path(__file__).parent
        / "fixtures/depth_object_localizer/desk_404975_depth.png"
    )
    depth_mm = cv2.imread(str(fixture), cv2.IMREAD_UNCHANGED)
    frame = _frame(tmp_path, depth_mm)

    out = DepthObjectLocalizer().localize(
        [{"label": "办公桌", "bbox_2d": [0.26, 0.13, 0.80, 0.27]}],
        frame,
    )

    assert out[0].depth_m == pytest.approx(1.775, abs=0.04)


def test_real_desk_thin_bbox_uses_complete_bbox_dominant_depth(tmp_path: Path):
    fixture = (
        Path(__file__).parent
        / "fixtures/depth_object_localizer/desk_54198_depth.png"
    )
    depth_mm = cv2.imread(str(fixture), cv2.IMREAD_UNCHANGED)
    frame = _frame(tmp_path, depth_mm)

    out = DepthObjectLocalizer().localize(
        [{"label": "办公桌", "bbox_2d": [0.19, 0.07, 0.82, 0.25]}],
        frame,
    )

    assert out[0].depth_m == pytest.approx(1.67, abs=0.04)


def test_real_desk_wide_bbox_uses_dominant_depth_without_inner_support(tmp_path: Path):
    fixture = (
        Path(__file__).parent
        / "fixtures/depth_object_localizer/desk_118180_depth.png"
    )
    depth_mm = cv2.imread(str(fixture), cv2.IMREAD_UNCHANGED)
    frame = _frame(tmp_path, depth_mm)

    out = DepthObjectLocalizer().localize(
        [{"label": "桌子", "bbox_2d": [0.19, 0.07, 0.83, 0.25]}],
        frame,
    )

    assert out[0].depth_m == pytest.approx(1.67, abs=0.04)


def test_dominant_far_background_does_not_override_inner_foreground(tmp_path: Path):
    depth_mm = np.full((100, 100), 4000, dtype=np.uint16)
    # The foreground occupies the bbox center but less area than the far
    # background in the complete detection box.
    depth_mm[35:65, 35:65] = 1200
    frame = _frame(tmp_path, depth_mm)

    out = DepthObjectLocalizer().localize(
        [{"label": "small object", "bbox_2d": [0.10, 0.10, 0.90, 0.90]}],
        frame,
    )

    assert out[0].depth_m == pytest.approx(1.2, abs=0.02)


# --- 计划书 §37：mask 深度采样 + 3D box + bbox fallback -----------------------


def test_mask_depth_isolates_foreground_from_background(tmp_path: Path):
    """§30 场景：1.5m 目标从 3.5m 背景中提取，mask 只采目标深度。"""
    depth_mm = np.full((100, 100), 3500, dtype=np.uint16)
    depth_mm[40:60, 40:60] = 1500
    frame = _frame(tmp_path, depth_mm)
    objects = [{
        "label": "red fire extinguisher",
        "label_zh": "灭火器",
        # bbox 故意覆盖背景，验证 mask 能纠正 bbox 的背景污染。
        "bbox_2d": [0.05, 0.05, 0.95, 0.95],
        "mask_rle": _rect_mask_rle(40, 40, 20, 20),
        "mask_size": [100, 100],
        "confidence": 0.9,
    }]
    out = DepthObjectLocalizer().localize(objects, frame)
    assert len(out) == 1
    assert out[0].spatial_quality == SPATIAL_QUALITY_CAMERA_LOCAL
    assert out[0].depth_m == pytest.approx(1.5, abs=0.02)
    assert out[0].provenance["depth_source"] == "sam2_mask"


def test_mask_missing_falls_back_to_bbox_and_contaminates(tmp_path: Path):
    """无 mask 时退回 bbox 分支：dominant depth 被背景污染为 3.5m（对比基线）。"""
    depth_mm = np.full((100, 100), 3500, dtype=np.uint16)
    depth_mm[40:60, 40:60] = 1500
    frame = _frame(tmp_path, depth_mm)
    out = DepthObjectLocalizer().localize(
        [{"label": "obj", "bbox_2d": [0.05, 0.05, 0.95, 0.95]}],
        frame,
    )
    assert out[0].provenance["depth_source"] == "bbox"
    assert out[0].box_center_camera is None
    assert out[0].box_size_camera is None
    assert out[0].depth_m == pytest.approx(3.5, abs=0.02)


def test_mask_produces_3d_box_center_and_size(tmp_path: Path):
    """mask 定位输出 box_center_camera / box_size_camera（§8.3 AABB）。"""
    depth_mm = np.full((100, 100), 0, dtype=np.uint16)
    depth_mm[40:60, 40:60] = 2000
    frame = _frame(tmp_path, depth_mm)
    objects = [{
        "label": "box",
        "bbox_2d": [0.35, 0.35, 0.65, 0.65],
        "mask_rle": _rect_mask_rle(40, 40, 20, 20),
        "mask_size": [100, 100],
    }]
    out = DepthObjectLocalizer().localize(objects, frame)
    obs = out[0]
    assert obs.box_center_camera is not None
    assert obs.box_size_camera is not None
    # fx=fy=50, cx=cy=50：中心像素 (49.5, 49.5) -> 3D ≈ (0, 0, 2.0)。
    assert obs.box_center_camera[2] == pytest.approx(2.0, abs=0.02)
    # 20px 宽 @ 2m 深 / 50 焦距 = 0.8m（p05/p95 略缩，给容差）。
    assert obs.box_size_camera[0] == pytest.approx(0.76, abs=0.08)
    assert obs.box_size_camera[1] == pytest.approx(0.76, abs=0.08)
    # 深度均匀 → 高度接近 0。
    assert obs.box_size_camera[2] == pytest.approx(0.0, abs=0.1)


# --------------------------------------------------------------------------- #
# §7.2 数据链路契约：label / label_zh / category 三段不丢
#
# `label` 是实体关联键，`label_zh` 是显示名。已知词优先中文；未知英文直接显示
# 原始类别，避免不同类别全部塌成「未分类物体」。
# --------------------------------------------------------------------------- #
def test_chinese_display_name_is_forwarded(tmp_path: Path):
    depth_mm = np.full((64, 64), 0, dtype=np.uint16)
    depth_mm[20:44, 20:44] = 2000
    frame = _frame(tmp_path, depth_mm)
    out = DepthObjectLocalizer().localize(
        [{"label": "speaker", "label_zh": "音箱", "category": "electronics",
          "bbox_2d": [0.3, 0.3, 0.7, 0.7], "confidence": 0.9}],
        frame,
    )
    # label 是**关联键**：知道中文时就用中文（两只 speaker 才并得成一个实体），
    # label_zh 是显示名。知道名字的物体上两者相等；未知英文才分叉。
    assert out[0].label == "音箱"
    assert out[0].label_zh == "音箱"
    assert out[0].category == "electronics"
    assert out[0].to_dict()["label_zh"] == "音箱"
    assert out[0].to_dict()["category"] == "electronics"


def test_unknown_english_keeps_the_original_as_the_association_key(tmp_path: Path):
    """拿不到中文时：关联键和显示名都保留英文，身份仍不依赖显示兜底。

    若这里把 label 也写成「未分类物体」，两个互不相干的未知类别会在
    `SemanticObjectMap` 里并成同一个实体（计划书 §3）。
    """
    depth_mm = np.full((64, 64), 0, dtype=np.uint16)
    depth_mm[20:44, 20:44] = 2000
    frame = _frame(tmp_path, depth_mm)
    out = DepthObjectLocalizer().localize(
        [{"label": "gizmo", "bbox_2d": [0.3, 0.3, 0.7, 0.7]}], frame)
    assert out[0].label == "gizmo"
    assert out[0].label_zh == "未分类物体"


def test_yoloe_keeps_raw_english_for_map_filtering(tmp_path: Path):
    depth_mm = np.full((64, 64), 0, dtype=np.uint16)
    depth_mm[20:44, 20:44] = 2000
    frame = _frame(tmp_path, depth_mm)
    out = DepthObjectLocalizer().localize(
        [{
            "label": "door handle",
            "label_zh": "门",
            "source": "yoloe_seg",
            "attributes": ["detected_by_yoloe"],
            "bbox_2d": [0.3, 0.3, 0.7, 0.7],
            "confidence": 0.9,
        }],
        frame,
    )
    assert out[0].label == "door handle"
    assert out[0].label_zh == "门"


def test_display_name_is_resolved_when_absent(tmp_path: Path):
    """上游只给英文 label 时，localizer 这一层就把它解析成中文。"""
    depth_mm = np.full((64, 64), 0, dtype=np.uint16)
    depth_mm[20:44, 20:44] = 2000
    frame = _frame(tmp_path, depth_mm)
    out = DepthObjectLocalizer().localize(
        [{"label": "barrel", "bbox_2d": [0.3, 0.3, 0.7, 0.7]}], frame)
    assert out[0].label_zh == "桶"


def test_category_falls_back_to_the_vocabulary(tmp_path: Path):
    depth_mm = np.full((64, 64), 0, dtype=np.uint16)
    depth_mm[20:44, 20:44] = 2000
    frame = _frame(tmp_path, depth_mm)
    out = DepthObjectLocalizer().localize(
        [{"label": "chair", "bbox_2d": [0.3, 0.3, 0.7, 0.7]}], frame)
    assert out[0].category == "furniture"


def test_three_fields_survive_the_mask_path(tmp_path: Path):
    """mask 分支和 bbox 分支必须走同一套取值，不能有一条漏掉。"""
    depth_mm = np.full((64, 64), 0, dtype=np.uint16)
    depth_mm[20:44, 20:44] = 2000
    frame = _frame(tmp_path, depth_mm)
    mask = np.zeros((64, 64), dtype=np.uint8)
    mask[20:44, 20:44] = 1
    out = DepthObjectLocalizer().localize(
        [{"label": "speaker", "label_zh": "音箱", "category": "electronics",
          "mask": mask, "bbox_2d": [0.3, 0.3, 0.7, 0.7], "confidence": 0.9}],
        frame,
    )
    assert (out[0].label, out[0].label_zh, out[0].category) == (
        "音箱", "音箱", "electronics")


def test_three_fields_survive_the_rgb_only_path():
    frame = RGBDFrame(
        frame_id="f_rgb2", timestamp=0.0, color_ref="", depth_ref="",
        width=64, height=64, fx=50.0, fy=50.0, cx=32.0, cy=32.0,
    )
    out = DepthObjectLocalizer().localize(
        [{"label": "bin", "label_zh": "垃圾桶", "category": "container",
          "bbox_2d": [0.3, 0.3, 0.7, 0.7]}],
        frame,
    )
    assert (out[0].label, out[0].label_zh, out[0].category) == (
        "垃圾桶", "垃圾桶", "container")
