"""Scripted observation scenes for offline E2E (mock backend + mock vision).

The AutonomousExplorer's observer/verifier are injected callables; this module
provides deterministic fakes so the full loop can run without a robot or LLM:
target appears after N nodes, target never appears, anchor appears then
target, operator stop, etc.  Also used by ``scripts/go2w/run_semantic_exploration.py
--backend mock`` for offline dry-runs.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Any, Callable

from app.navigation.models import LiveObservation
from app.live_robot.autonomous_explorer import SemanticMatch, VerificationOutcome


@dataclass
class MockSceneStep:
    objects: list[str]
    relations: list[dict[str, Any]] | None = None
    target_present: bool = False
    bundle_id: str | None = None
    anchor_labels: list[str] = field(default_factory=list)
    target_score: float = 0.0


class MockObservationScene:
    """Deterministic observer + verifier pair for offline E2E scenarios."""

    def __init__(
        self,
        *,
        scenes: list[MockSceneStep],
        confirm_after_seen: int = 1,
        yaw_deg: float = 0.0,
        scene_graph: bool = True,
    ) -> None:
        self.scenes = list(scenes)
        self.index = 0
        self.yaw_deg = float(yaw_deg)
        self.confirm_after_seen = max(1, int(confirm_after_seen))
        self.scene_graph = bool(scene_graph)
        self.target_seen_count = 0
        self.observations_made = 0
        self.last_observation: LiveObservation | None = None

    # ---- observer ---------------------------------------------------------

    def observer(self) -> Callable[[], LiveObservation]:
        def observe() -> LiveObservation:
            step = self.scenes[min(self.index, len(self.scenes) - 1)]
            self.index += 1
            self.observations_made += 1
            bundle_id = step.bundle_id or f"mock_{self.observations_made:03d}"
            objects = []
            for i, raw in enumerate(step.objects):
                if isinstance(raw, dict):
                    obj = dict(raw)
                    obj.setdefault("label", obj.get("label_zh") or obj.get("name") or "object")
                    obj.setdefault("label_zh", obj.get("label_zh") or obj.get("label") or "object")
                    obj.setdefault("name", obj.get("name") or obj["label"])
                    obj.setdefault("confidence", 0.9)
                    obj.setdefault("bbox_2d", [0.4, 0.3, 0.6, 0.7])
                    obj.setdefault("id", obj.get("frame_object_id") or f"mock_obj_{self.observations_made:03d}_{i:02d}")
                    obj["id"] = str(obj["id"])
                else:
                    label = str(raw)
                    obj = {
                        "label": label,
                        "label_zh": label,
                        "name": label,
                        "position_2d": "center",
                        "confidence": 0.9,
                        "bbox_2d": [0.4, 0.3, 0.6, 0.7],
                        "id": f"mock_obj_{self.observations_made:03d}_{i:02d}",
                    }
                objects.append(obj)
            observation = LiveObservation(
                bundle_id=bundle_id,
                timestamp=time.time(),
                image_ref=f"mock://{bundle_id}",
                detections=[
                    {
                        "label": label,
                        "score": 0.9,
                        "bbox_2d": [0.4, 0.3, 0.6, 0.7],
                    }
                    for label in step.objects
                ],
                scene_graph=(
                    {
                        "nodes": [
                            {"node_id": f"mock_{label}", "label": label,
                             "label_zh": label, "attributes": {}}
                            for label in step.objects
                        ],
                        "edges": [],
                    }
                    if self.scene_graph else None
                ),
                scene_objects=objects,
                scene_relations=list(step.relations or []),
                target_match={
                    "target_present": bool(step.target_present),
                    "score": step.target_score,
                },
                pose={
                    "x": 0.0, "y": 0.0,
                    "yaw_deg": self.yaw_deg,
                },
                sensor_health={"camera": True, "lidar": True},
                provenance={"source": "mock_scene", "step": self.index - 1},
            )
            self.last_observation = observation
            return observation

        return observe

    # ---- matcher ----------------------------------------------------------

    def matcher(self) -> Callable[[LiveObservation], SemanticMatch]:
        def match(observation: LiveObservation) -> SemanticMatch:
            step = self.scenes[min(max(0, self.index - 1), len(self.scenes) - 1)]
            return SemanticMatch(
                has_candidate=bool(observation.target_present),
                target_match=observation.target_match,
                target_profile=None,
                anchor_labels=list(step.anchor_labels),
                target_score=float((observation.target_match or {}).get("score", 0.0)),
                target_match_level=(
                    "candidate" if observation.target_present else "none"
                ),
                provenance={"source": "mock_matcher"},
            )

        return match

    # ---- verifier ---------------------------------------------------------

    def verifier(self) -> Callable[[LiveObservation, SemanticMatch], VerificationOutcome]:
        def verify(observation: LiveObservation,
                   match: SemanticMatch) -> VerificationOutcome:
            if observation.target_present:
                self.target_seen_count += 1
            confirmed = (
                observation.target_present
                and self.target_seen_count >= self.confirm_after_seen
            )
            return VerificationOutcome(
                confirmed=confirmed,
                attempts=1,
                reason_zh=(
                    "mock verify confirmed target"
                    if confirmed else "mock verify: target not confirmed"
                ),
                details={"seen_count": self.target_seen_count},
            )

        return verify


def scenario_target_appears_after(n: int, *, target: str = "blue trash bin",
                                  anchor: str = "water dispenser") -> MockObservationScene:
    """Target appears after ``n`` observations (anchor on observation 1)."""
    scenes: list[MockSceneStep] = []
    for index in range(n):
        if index == 0:
            scenes.append(
                MockSceneStep(
                    objects=[anchor], anchor_labels=[anchor],
                    bundle_id=f"obs_{index + 1:03d}",
                )
            )
        else:
            scenes.append(
                MockSceneStep(
                    objects=["desk", "chair"], bundle_id=f"obs_{index + 1:03d}",
                )
            )
    scenes.append(
        MockSceneStep(
            objects=[anchor, target],
            relations=[
                {"subject_label": target, "object_label": anchor,
                 "relation": "near", "confidence": 0.9},
            ],
            target_present=True,
            anchor_labels=[anchor],
            target_score=0.95,
            bundle_id="obs_final",
        )
    )
    return MockObservationScene(scenes=scenes, confirm_after_seen=1)


def scenario_no_target(*, empty_scenes: int = 6) -> MockObservationScene:
    scenes = [
        MockSceneStep(objects=["desk"], bundle_id=f"obs_{index + 1:03d}")
        for index in range(empty_scenes)
    ]
    return MockObservationScene(scenes=scenes)


def scenario_anchor_then_target() -> MockObservationScene:
    return scenario_target_appears_after(3)


def scenario_semantic_topology() -> MockObservationScene:
    """确定性场景：两帧 3 物体、2 条关系（办公桌 near 垃圾桶、办公椅 left_of 办公桌）。

    第二帧使用全新 frame id 但相同 map_xyz，验证 persistent merge：最终仍是
    3 个 persistent 节点、2 条关系边，near 被证据融合为 CONFIRMED ×2。
    用于 WebUI mock 演示「语义拓扑」视图。
    """

    def item(oid: str, label: str, xyz: tuple) -> dict:
        return {
            "id": oid,
            "label": label,
            "label_zh": label,
            "name": label,
            "map_xyz": list(xyz),
            "confidence": 0.9,
            "bbox_2d": [0.35, 0.3, 0.65, 0.7],
        }

    scenes = [
        MockSceneStep(
            objects=[
                item("topo_obj_001", "办公桌", (1.0, 0.0, 0.0)),
                item("topo_obj_002", "绿色垃圾桶", (1.3, 0.0, 0.0)),
                item("topo_obj_003", "办公椅", (0.3, 0.2, 0.0)),
            ],
            relations=[
                {"subject_id": "topo_obj_001", "object_id": "topo_obj_002",
                 "relation": "near", "confidence": 0.72, "description_zh": "办公桌靠近绿色垃圾桶"},
                {"subject_id": "topo_obj_003", "object_id": "topo_obj_001",
                 "relation": "left_of", "confidence": 0.80, "description_zh": "办公椅在办公桌左侧"},
            ],
            bundle_id="obs_topo_1",
        ),
        MockSceneStep(
            objects=[
                item("topo_obj_101", "办公桌", (1.02, 0.0, 0.0)),
                item("topo_obj_102", "绿色垃圾桶", (1.28, 0.0, 0.0)),
                item("topo_obj_103", "办公椅", (0.32, 0.2, 0.0)),
            ],
            relations=[
                {"subject_id": "topo_obj_101", "object_id": "topo_obj_102",
                 "relation": "near", "confidence": 0.80, "description_zh": "办公桌靠近绿色垃圾桶"},
            ],
            bundle_id="obs_topo_2",
        ),
    ]
    return MockObservationScene(scenes=scenes, confirm_after_seen=1)


def scenario_green_bin():
    """确定性场景：搜「绿色垃圾桶」。
    前两帧办公室有「办公桌/办公椅/纸箱」（无目标），最后一帧出现「绿色垃圾桶」
    且 near 办公桌 -> 目标出现并可确认。用于 WebUI mock 演示 + 端到端自测。
    """

    def item(oid: str, label: str, xyz: tuple) -> dict:
        return {
            "id": oid,
            "label": label,
            "label_zh": label,
            "name": label,
            "map_xyz": list(xyz),
            "confidence": 0.9,
            "bbox_2d": [0.35, 0.3, 0.65, 0.7],
        }

    scenes = [
        MockSceneStep(
            objects=[
                item("gb_001", "办公桌", (1.0, 0.0, 0.0)),
                item("gb_002", "办公椅", (0.3, 0.2, 0.0)),
                item("gb_003", "纸箱", (2.0, 0.0, 0.0)),
            ],
            relations=[
                {"subject_id": "gb_002", "object_id": "gb_001", "relation": "left_of", "confidence": 0.8},
            ],
            bundle_id="obs_gb_1",
        ),
        MockSceneStep(
            objects=[
                item("gb_101", "办公桌", (1.02, 0.0, 0.0)),
                item("gb_102", "办公椅", (0.32, 0.2, 0.0)),
            ],
            relations=[],
            bundle_id="obs_gb_2",
        ),
        MockSceneStep(
            objects=[
                item("gb_201", "办公桌", (1.0, 0.0, 0.0)),
                item("gb_202", "绿色垃圾桶", (1.4, 0.0, 0.0)),
            ],
            relations=[
                {"subject_id": "gb_201", "object_id": "gb_202", "relation": "near", "confidence": 0.92,
                 "description_zh": "办公桌靠近绿色垃圾桶"},
            ],
            target_present=True,
            target_score=0.99,
            bundle_id="obs_gb_final",
        ),
    ]
    return MockObservationScene(scenes=scenes, confirm_after_seen=1)


def _spatial_provider_for_scene(scene: "MockObservationScene") -> Any:
    """A minimal fake SpatialProvider that emits map_xyz for scripted objects.

    It ties the object labels observed in the mock scene to concrete map
    positions (plan §20 deterministic spatial mock).
    """
    map_positions = {
        "办公桌": (1.0, 0.5),
        "desk": (1.0, 0.5),
        "垃圾桶": (2.0, 0.5),
        "green trash bin": (2.0, 0.5),
        "蓝色垃圾桶": (2.0, 0.5),
        "饮水机": (0.8, 2.0),
        "water dispenser": (0.8, 2.0),
    }
    class _MockSpatialProvider:
        def quality(self) -> str:
            return "RELATIVE_RGBD"
        def get_pose(self):
            return None
        def get_map(self):
            return None
        def get_frontiers(self):
            return []
        def camera_point_to_spatial(
            self, xyz_camera, pose=None,
        ):
            # Provide a default mild forward projection so camera-local gets a
            # (relative) map coordinate.
            if xyz_camera is None:
                return None
            return (round(float(xyz_camera[0]), 3), round(float(xyz_camera[1]), 3), 0.0)
    return _MockSpatialProvider()


def scenario_spatial_semantic_search(
    *,
    target: str = "绿色垃圾桶",
    anchor: str = "办公桌",
) -> MockObservationScene:
    """Deterministic spatial mock scene for E2E (plan §20).

    Provides a small metric-ish world with scripted map_xyz per object so the
    whole online spatial stack (map_xyz -> entity association -> route ->
    decision) can be exercised without a robot.  The returned scene's observer
    attaches map_xyz to each object's camera_xyz, and exposes a
    ``spatial_provider`` attribute callers can wire into the pipeline.
    """
    obj_map = {
        anchor: (1.0, 0.5),
        "办公桌": (1.0, 0.5),
        target: (2.0, 0.5),
        "饮水机": (0.8, 2.0),
        "凳子": (0.2, 1.5),
    }
    scenes: list[MockSceneStep] = [
        MockSceneStep(
            objects=[anchor],
            bundle_id="exp_obs_001",
            anchor_labels=[anchor],
        ),
        MockSceneStep(
            objects=["饮水机", "凳子"],
            bundle_id="exp_obs_002",
        ),
        MockSceneStep(
            objects=[anchor, target],
            relations=[
                {"subject_label": target, "object_label": anchor,
                 "relation": "near", "confidence": 0.9},
            ],
            target_present=True,
            anchor_labels=[anchor],
            target_score=0.92,
            bundle_id="exp_obs_target",
        ),
    ]
    scene = MockObservationScene(scenes=scenes, confirm_after_seen=1)

    base_observe = scene.observer()

    def spatialized_observe() -> LiveObservation:
        observation = base_observe()
        for obj in observation.scene_objects:
            label = str(obj.get("label") or obj.get("label_zh") or obj.get("name") or "")
            map_xyz = obj_map.get(label) or obj_map.get(_canon(label))
            if map_xyz is not None:
                obj["map_xyz"] = [map_xyz[0], map_xyz[1], 0.0]
                obj["depth_m"] = 1.0
                obj["bearing_deg"] = 0.0
                obj["camera_xyz"] = [0.0, 0.0, 1.0]
                obj["spatial_quality"] = "RELATIVE_RGBD"
                obj["confidence"] = 0.9
        scene.last_observation = observation
        return observation

    # Make scene.observer() return the decorated observer.
    scene.observer = (lambda: spatialized_observe)  # type: ignore[assignment]
    scene.spatial_provider = _spatial_provider_for_scene(scene)  # type: ignore[attr-defined]
    return scene


def _canon(label: str) -> str:
    return label.strip()
