"""A6 离线验证：RAM tag → DINO bbox → SAM2 mask → DepthObjectLocalizer → SemanticObjectMap。

用仓库里现成的真机 D435 帧（`runtime/go2w/semantic_observe_cache/`）跑真实模型，
只有深度图是合成的 —— 缓存帧里没有配套深度，而这一跑要验的是**链路是否通**，
不是测深度精度（深度精度属 §14.5，已挂起）。

    python scripts/go2w/verify_ram_semantic_chain.py [帧路径]
"""

from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from app.config import get_settings  # noqa: E402
from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector  # noqa: E402
from app.perception.depth_object_localizer import DepthObjectLocalizer  # noqa: E402
from app.perception.rgbd_source import RGBDFrame  # noqa: E402
from app.spatial.semantic_object_map import SemanticObjectMap  # noqa: E402
from app.video.target_profile import TargetProfileResolver  # noqa: E402

DEFAULT_FRAME = (
    PROJECT_ROOT / "runtime/go2w/semantic_observe_cache/bundle_137555.jpg"
)


def synth_depth(width: int, height: int, depth_path: Path) -> None:
    """一堵 2 m 的墙，底部留一块 1.2 m 的近景，让不同 bbox 拿到不同深度。"""
    depth_mm = np.full((height, width), 2000, dtype=np.uint16)
    depth_mm[int(height * 0.6) :, :] = 1200
    cv2.imwrite(str(depth_path), depth_mm)


def main() -> int:
    frame_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_FRAME
    settings = get_settings()

    print(f"帧: {frame_path}")

    detector = GroundedSAMSubprocessDetector(settings=settings)
    ram_terms = detector.discover_scene_terms(str(frame_path))
    print(f"\n[1] RAM++ 场景词 ({len(ram_terms)}): {ram_terms}")

    profile = TargetProfileResolver(settings=settings).resolve(
        "找挂着书包的椅子", use_llm=True
    )
    print(f"[2] 目标词: {profile.detector_terms()}")

    detected = detector.detect_with_dynamic_terms(
        str(frame_path), profile.raw_query, [], ram_terms
    )
    with_mask = [item for item in detected if getattr(item, "mask_rle", None)]
    print(f"[3] GroundingDINO+SAM2: {len(detected)} 个候选，{len(with_mask)} 个带 mask")
    for item in detected[:8]:
        print(
            f"      {item.label:<20} score={item.score:.3f} "
            f"mask={'Y' if getattr(item, 'mask_rle', None) else 'N'}"
        )

    width, height = 1280, 720
    depth_path = PROJECT_ROOT / "outputs/ram_chain_depth.png"
    depth_path.parent.mkdir(parents=True, exist_ok=True)
    synth_depth(width, height, depth_path)
    frame = RGBDFrame(
        frame_id="ram-chain",
        timestamp=0.0,
        color_ref=str(frame_path),
        depth_ref=str(depth_path),
        width=width,
        height=height,
        fx=1484.6,
        fy=1491.0,
        cx=893.6,
        cy=417.1,
        depth_unit_m=0.001,
    )
    localizer = DepthObjectLocalizer()
    spatial = localizer.localize(
        [
            {
                "label": item.label,
                "bbox_2d": list(item.bbox_2d),
                "mask_rle": getattr(item, "mask_rle", None),
                "mask_size": getattr(item, "mask_size", None),
            }
            for item in detected
        ],
        frame,
    )
    located = [item for item in spatial if item.depth_m is not None]
    print(f"[4] DepthObjectLocalizer: {len(spatial)} 条，其中 {len(located)} 条有深度")
    for item in spatial[:8]:
        print(
            f"      {item.label:<20} depth={item.depth_m} "
            f"xyz={tuple(round(v, 2) for v in item.camera_xyz)}"
            if item.depth_m is not None
            else f"      {item.label:<20} depth=None (rgb_only)"
        )

    object_map = SemanticObjectMap()
    object_map.update(spatial, now=0.0)
    object_map.update(spatial, now=1.0)  # 第二次观测才够 confirm_min_observations
    print(f"[5] SemanticObjectMap: {len(object_map.objects)} 个物体")
    for entry in list(object_map.objects.values())[:10]:
        print(f"      {entry.label:<20} obs={entry.observation_count}")

    detector.close()
    ok = bool(ram_terms) and bool(with_mask) and bool(located) and bool(object_map.objects)
    print(f"\n链路{'通过' if ok else '不通过'}: "
          f"RAM={bool(ram_terms)} DINO+SAM2={bool(with_mask)} "
          f"定位={bool(located)} 地图={bool(object_map.objects)}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
