"""A7 / §61 Phase 4 离线验证：复杂目标的 VLM 复核。

两个必须通过的用例：

* §14 「找黄色椅子」—— 画像里必须有颜色组合词（`yellow chair`），
  且复核模块能把不是黄色的椅子判掉（attribute 低、is_target=false）。
* §15 「找挂着书包的椅子」—— GroundingDINO 必须同时搜 chair 和 backpack，
  §15.2 的 2D 几何要挑出同帧邻近的背包，§15.3 的裁剪框必须是
  chair ∪ backpack 的并集，再把 context 交给复核。

跑的是真模型（RAM++ / GroundingDINO / SAM2 / SiliconFlow VLM）。
VLM 不可用时不算通过，如实打印失败原因（D7）。

    python scripts/go2w/verify_ram_target_verify.py [帧路径]
"""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PROJECT_ROOT))

from app.config import get_settings  # noqa: E402
from app.detectors.crop_verifier import CropVerifier  # noqa: E402
from app.detectors.grounded_sam_subprocess import GroundedSAMSubprocessDetector  # noqa: E402
from app.reasoning.target_profile import TargetProfile, TargetProfileResolver  # noqa: E402
from app.video.semantic_verifier import _verification_region  # noqa: E402
from app.vision.crop_utils import save_candidate_crop  # noqa: E402

DEFAULT_FRAME = PROJECT_ROOT / "runtime/go2w/semantic_observe_cache/bundle_137555.jpg"
OUTPUT_DIR = PROJECT_ROOT / "outputs/ram_target_verify"


def _frame_objects(objects: list) -> list[dict]:
    """把检测结果转成 _verification_region 认识的形状（它要 bbox / object_id）。

    object_id 不能省：_verification_region 用
    `obj.object_id == candidate.object_id` 跳过候选自己，两边都缺这个键时
    None == None 成立，同帧物体会被整体跳过，关系裁剪永远不生效。
    """
    return [
        {
            "object_id": f"det_{index:03d}",
            "label": item.label,
            "label_zh": getattr(item, "label_zh", ""),
            "bbox": list(item.bbox_2d),
        }
        for index, item in enumerate(objects)
    ]


def _detect(
    detector: GroundedSAMSubprocessDetector, frame: Path, profile: TargetProfile
) -> list:
    ram_terms = detector.discover_scene_terms(str(frame))
    return ram_terms, detector.detect_with_dynamic_terms(
        str(frame), profile.raw_query, [], ram_terms
    )


def _verify(
    verifier: CropVerifier,
    frame: Path,
    profile: TargetProfile,
    objects: list,
    candidate,
    name: str,
) -> dict:
    """走 §15.2/§15.3：算关系裁剪框 → 存裁剪 → 带 context 交给复核。"""
    frame_objects = _frame_objects(objects)
    candidate_id = next(
        (
            item["object_id"]
            for item in frame_objects
            if item["bbox"] == list(candidate.bbox_2d)
        ),
        "",
    )
    verify_bbox, context_objects = _verification_region(
        {"object_id": candidate_id, "bbox": list(candidate.bbox_2d)},
        frame_objects,
        profile,
    )
    crop_path = OUTPUT_DIR / f"{name}_crop.jpg"
    saved, expanded = save_candidate_crop(
        frame, verify_bbox, crop_path, get_settings().crop_verify_expand_ratio
    )
    result = verifier.verify_crop(
        saved,
        profile,
        candidate.label,
        context={
            "candidate_bbox": list(candidate.bbox_2d),
            "verification_bbox": verify_bbox,
            "nearby_objects": context_objects,
            "required_relations": list(profile.relation_constraints),
        },
    )
    print(f"    candidate={candidate.label} score={candidate.score:.3f}")
    print(f"    裁剪框 candidate={_fmt(candidate.bbox_2d)} -> verify={_fmt(verify_bbox)}")
    print(f"    邻近 context 物体: {[c['label'] for c in context_objects]}")
    print(f"    type={profile.target_type} 关系约束={list(profile.relation_constraints)}")
    print(
        f"    复核: is_target={result.get('is_target')} "
        f"match={result.get('target_match_score')} "
        f"attr={result.get('attribute_score')} "
        f"context={result.get('context_score')}"
    )
    print(f"    理由: {result.get('reason_zh') or result.get('rejection_reason') or ''}")
    if result.get("verification_failed"):
        print(f"    复核失败: {result.get('failure_reason')}")
    return {**result, "verification_bbox": verify_bbox, "crop": str(saved),
            "context_objects": context_objects, "expanded": expanded}


def _fmt(bbox) -> str:
    return "[" + ", ".join(f"{float(v):.3f}" for v in bbox) + "]"


def _yellow_chair_case(resolver, detector, frames: list[Path], verifier) -> bool:
    print("\n=== §14 找黄色椅子 ===")
    profile = resolver.resolve("找黄色椅子", use_llm=True)
    terms = profile.detector_terms()
    print(f"  primary={profile.primary_labels_en} colors={profile.colors}")
    print(f"  detector_terms={terms}")
    prompt_ok = "chair" in terms and any("yellow" in term for term in terms)
    print(f"  [{'OK' if prompt_ok else 'FAIL'}] 颜色组合词进了目标词表")

    judged_false = 0
    for index, frame in enumerate(frames):
        ram_terms, objects = _detect(detector, frame, profile)
        chairs = [item for item in objects if "chair" in item.label][:3]
        print(f"  {frame.name}: RAM={len(ram_terms)} 检出={len(objects)} 椅子候选={len(chairs)}")
        for chair_index, chair in enumerate(chairs):
            result = _verify(
                verifier, frame, profile, objects, chair,
                f"yellow_f{index}_c{chair_index}",
            )
            if not result.get("verification_failed") and not result.get("is_target"):
                judged_false += 1
    # §14 的验收点：非黄色的椅子必须被复核判掉（chair_1 attribute 低、is_target=false）。
    rejected_ok = judged_false > 0
    print(f"  [{'OK' if rejected_ok else 'FAIL'}] 被判掉的椅子数={judged_false}")
    return prompt_ok and rejected_ok


def _bag_on_chair_case(resolver, detector, frame: Path, verifier) -> bool:
    print("\n=== §15 找挂着书包的椅子 ===")
    profile = resolver.resolve("找挂着书包的椅子", use_llm=True)
    print(f"  detector_terms={profile.detector_terms()}")
    print(f"  context={profile.context_labels_en + profile.context_labels_zh} "
          f"关系={list(profile.relation_constraints)}")
    print(f"  关系锚点="
          f"{profile.relation_anchor_labels_en + profile.relation_anchor_labels_zh}")

    ram_terms, objects = _detect(detector, frame, profile)
    # §15：GroundingDINO 必须至少同时搜 chair 和 backpack。
    # 判据取自**真正送给 DINO 的那串 prompt**，不是 profile.detector_terms() ——
    # 关系锚点按设计不进 detector_terms（那是「目标本身」的词表），
    # 只查它会永远 FAIL，而实际 prompt 里其实有 backpack。
    prompts = detector._build_prompts(profile.raw_query, [], ram_terms)
    prompt_text = " ".join(prompts).lower()
    print(f"  DINO prompt={prompts}")
    search_ok = "chair" in prompt_text and any(
        word in prompt_text for word in ("backpack", "bag", "schoolbag")
    )
    print(f"  [{'OK' if search_ok else 'FAIL'}] chair 与 backpack 同时进搜索词")

    print(f"  帧 {frame.name}: RAM={len(ram_terms)} 检出={len(objects)}")
    labels = sorted({item.label for item in objects})
    print(f"  检出类别: {labels}")
    found_chair = any("chair" in label for label in labels)
    found_bag = any(
        word in label for label in labels for word in ("backpack", "bag", "schoolbag")
    )
    print(f"  [{'OK' if found_chair and found_bag else 'FAIL'}] "
          f"同帧检出 chair={found_chair} backpack={found_bag}")

    chairs = [item for item in objects if "chair" in item.label]
    if not chairs:
        print("  没有椅子候选，跳过关系裁剪检查")
        return search_ok and found_chair and found_bag
    chair = max(chairs, key=lambda item: item.score)
    result = _verify(verifier, frame, profile, objects, chair, "bag_chair")
    # §15.3：裁剪框必须并进**关系锚点物体**（背包），不是随便并进一个 context。
    # 之前只判「框变大了没」，结果并进一扇玻璃门也算过 —— 那是假通过。
    union_ok = list(result["verification_bbox"]) != list(chair.bbox_2d)
    anchor_ok = any(
        word in str(item["label"]).lower()
        for item in result["context_objects"]
        for word in ("backpack", "bag", "schoolbag")
    )
    context_ok = bool(result["context_objects"]) or not profile.relation_constraints
    print(f"  [{'OK' if union_ok else 'FAIL'}] 裁剪框并进了邻近 context 物体")
    print(f"  [{'OK' if anchor_ok else 'FAIL'}] 并进来的那个是关系锚点（背包）")
    print(f"  [{'OK' if context_ok else 'FAIL'}] context 里有邻近物体可传给复核")
    return (
        search_ok and found_chair and found_bag and union_ok and anchor_ok
        and context_ok
    )


def main() -> int:
    frame = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_FRAME
    second = PROJECT_ROOT / "runtime/go2w/semantic_observe_cache/bundle_106952.jpg"
    frames = [path for path in (frame, second) if path.is_file()]
    if not frames:
        print("找不到验证帧")
        return 1
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    settings = get_settings()
    verifier = CropVerifier(settings=settings)
    print(f"VLM 复核可用={verifier.available} 帧={[path.name for path in frames]}")
    if not verifier.available:
        print("没有 API key，Phase 4 无法验证")
        return 1

    resolver = TargetProfileResolver(settings=settings)
    detector = GroundedSAMSubprocessDetector(settings=settings)
    try:
        yellow_ok = _yellow_chair_case(resolver, detector, frames, verifier)
        bag_ok = _bag_on_chair_case(resolver, detector, frames[0], verifier)
    finally:
        detector.close()

    print(f"\nPhase 4 {'通过' if yellow_ok and bag_ok else '不通过'}: "
          f"黄色椅子={yellow_ok} 挂书包的椅子={bag_ok}")
    return 0 if yellow_ok and bag_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
