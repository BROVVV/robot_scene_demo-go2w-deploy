from __future__ import annotations

import unittest

from app.navigation.long_term_goal_selector import LongTermGoalSelector, ScoredIntent
from app.spatial.models import FrontierCandidate


def _frontier(fid: str, bearing: float) -> FrontierCandidate:
    return FrontierCandidate(
        frontier_id=fid,
        bearing_deg=bearing,
        distance_m=1.0,
        spatial_information_gain=0.5,
    )


class TestLongTermGoalMemoryScoring(unittest.TestCase):
    def test_memory_prior_changes_ranking(self) -> None:
        selector = LongTermGoalSelector()
        frontiers = [_frontier("F1", 0.0), _frontier("F2", 0.0)]
        result = selector.select(
            match_state="ZERO",
            frontiers=frontiers,
            memory_context={
                "frontier_priors": {"F1": 0.9, "F2": 0.0},
            },
        )
        self.assertIsNotNone(result)
        self.assertEqual(result.intent.target_frontier_id, "F1")
        self.assertGreater(
            result.components.get("experience_memory_prior", 0.0), 0.0
        )

    def test_common_sense_prior_influences_selection(self) -> None:
        selector = LongTermGoalSelector()
        frontiers = [
            _frontier("F1", 0.0),
            _frontier("F2", 100.0),
        ]
        result = selector.select(
            match_state="ZERO",
            frontiers=frontiers,
            common_sense={
                "frontier_hints": [{"bearing_deg": 0.0, "score": 0.8}]
            },
        )
        self.assertEqual(result.intent.target_frontier_id, "F1")

    def test_negative_evidence_lowers_revisit(self) -> None:
        selector = LongTermGoalSelector(negative_evidence_weight=0.5)
        frontiers = [_frontier("F1", 0.0), _frontier("F2", 0.0)]
        result = selector.select(
            match_state="ZERO",
            frontiers=frontiers,
            frontier_memory={
                "F1": {"negative_evidence": 2},
                "F2": {"negative_evidence": 0},
            },
        )
        self.assertEqual(result.intent.target_frontier_id, "F2")


if __name__ == "__main__":
    unittest.main()