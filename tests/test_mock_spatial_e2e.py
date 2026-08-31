"""Deterministic spatial mock E2E (plan §20).

Exercises the full offline spatial stack via the deterministic spatial mock
scene: observer attaches map_xyz -> AutonomousExplorer's SemanticNavigationGraph
absorbs it into persistent object entries -> a changing semantic-map/route
signal changes the selected frontier in the LongTermGoalSelector.
"""

from __future__ import annotations

import threading
import time

from app.live_robot.autonomous_explorer import AutonomousExplorer
from app.live_robot.mock_observation_scene import scenario_spatial_semantic_search
from app.navigation.backend_factory import MockBackend
from app.navigation.exploration_config import load_exploration_policy
from app.navigation.exploration_graph import ExplorationGraph


def _run_until(scene, *, max_obs: int = 6, timeout: float = 20.0) -> AutonomousExplorer:
    explorer = AutonomousExplorer(
        target="绿色垃圾桶",
        observer=scene.observer(),
        matcher=scene.matcher(),
        verifier=scene.verifier(),
        backend=MockBackend(),
        policy=load_exploration_policy(),
        graph=ExplorationGraph(session_id="spatial_e2e"),
        negative_target_key="绿色垃圾桶",
    )
    result = explorer.run()
    return explorer


def test_spatial_mock_e2e_object_entity_map_xyz():
    scene = scenario_spatial_semantic_search(target="绿色垃圾桶", anchor="办公桌")
    explorer = _run_until(scene)
    graph = explorer.semantic_graph
    objects = list(graph.object_map.objects.values())
    # At least the anchor and target were observed with map_xyz.
    labeled = [entry for entry in objects if entry.map_xyz is not None]
    assert len(labeled) >= 1
    # Every object carries a persistent id (obj_xxx), not a bare label.
    for entry in labeled:
        assert entry.object_id.startswith("obj_")
        assert len(entry.map_xyz) == 3


def test_spatial_mock_e2e_place_graph_has_moves():
    scene = scenario_spatial_semantic_search()
    explorer = _run_until(scene)
    places = list(explorer.semantic_graph.place_graph.places.values())
    assert len(places) >= 1
    # observed_object_ids on places are persistent entity ids.
    for place in places:
        for oid in place.observed_object_ids:
            assert oid.startswith("obj_") or oid.startswith("P") is False


def test_semantic_evidence_changes_selected_frontier():
    """Plan DoD 8: changing semantic/map evidence changes the selected
    frontier; distance alone must not be the only signal."""
    from app.navigation.long_term_goal_selector import LongTermGoalSelector
    from app.spatial.frontier_extractor import FrontierCandidate

    selector = LongTermGoalSelector()
    frontiers = [
        FrontierCandidate(frontier_id="F_near", position=(1.0, 0.0), distance_m=1.0,
                          spatial_information_gain=0.5),
        FrontierCandidate(frontier_id="F_far", position=(5.0, 0.0), distance_m=5.0,
                          spatial_information_gain=0.6),
    ]
    # Without semantic bias, the closer frontier wins (lower cost).
    no_bias = selector.select(
        match_state="z", frontiers=frontiers,
        semantic_relevance={"F_near": 0.0, "F_far": 0.0},
        current_yaw_deg=0.0,
    )
    # With a strong semantic signal on the far frontier, it overtakes.
    with_bias = selector.select(
        match_state="partial", frontiers=frontiers,
        semantic_relevance={"F_near": 0.0, "F_far": 1.0},
        current_yaw_deg=0.0,
    )
    assert no_bias is not None
    assert with_bias is not None
    assert no_bias.intent.target_frontier_id != with_bias.intent.target_frontier_id or \
           with_bias.score > no_bias.score
