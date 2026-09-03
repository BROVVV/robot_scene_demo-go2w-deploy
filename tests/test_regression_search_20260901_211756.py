"""Regression sample for live session ``search_20260901_211756_3460e15a``.

计划书 §4.2 把这次失败的真机会话固化为回归样本。该会话同时暴露四个缺陷：

1. 规划 ``-60°`` / ``+60°`` 只执行了一个 ``r30`` / ``l30`` 就宣告成功；
2. ROS 2 Foxy logger 的 printf 风格调用抛 ``TypeError``；
3. candidate generator 异常被吞成空候选，再被误报成 ``SEARCH_EXHAUSTED``；
4. 原地转向时 LIO 产生数米假平移，地图健康状态仍是 ``HEALTHY``。

``TestArchivedEvidence`` 断言归档证据里确实存在这些症状（样本不许被悄悄改写），
``TestFixesPreventRegression`` 断言当前代码不会再复现它们。
"""

from __future__ import annotations

import ast
import json
import math
import pathlib
import unittest

from app.live_robot.autonomous_explorer import AutonomousExplorer, ExplorerState
from app.live_robot.mock_observation_scene import MockObservationScene, MockSceneStep
from app.navigation.backend_factory import MockBackend
from app.navigation.exploration_config import load_exploration_policy
from app.navigation.exploration_graph import ExplorationGraph
from app.navigation.go2w_experimental_backend import (
    Go2WBackendConfig,
    Go2WExperimentalBackend,
)
from app.navigation.models import GOAL_ROTATE_VIEW, ExplorationGoal
from app.navigation.robot_backend import NavigationStatus
from app.spatial.models import SpatialPose
from app.spatial.semantic_navigation_graph import SemanticNavigationGraph
from app.spatial.spatial_pose_validator import MotionEvidence, SpatialPoseValidator

REPO = pathlib.Path(__file__).resolve().parents[1]
SESSION = REPO / "outputs" / "regressions" / "search_20260901_211756_3460e15a"


def _events() -> list[dict]:
    rows: list[dict] = []
    path = SESSION / "events.jsonl"
    if not path.exists():
        return rows
    for line in path.read_text(errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except ValueError:
            continue
    return rows


class _FakeMotion:
    """Reproduces the measured Go2-W turn gain (~29° for a commanded 30°)."""

    def __init__(self, gain: float = 28.9 / 30.0) -> None:
        self.steps: list[str] = []
        self.odom = [0.0, 0.0, 0.0]
        self.gain = gain

    def execute(self, step: str) -> tuple[bool, str, dict]:
        self.steps.append(step)
        degrees = float(step[1:]) * self.gain
        if step.startswith("r"):
            degrees = -degrees
        self.odom[2] += math.radians(degrees)
        return True, "motion completed and robot stationary", {"step": step}

    def odometry(self) -> tuple[float, float, float]:
        return (self.odom[0], self.odom[1], self.odom[2])


class TestArchivedEvidence(unittest.TestCase):
    """归档样本必须继续展示原始症状，否则回归基线就被抹掉了。"""

    @classmethod
    def setUpClass(cls) -> None:
        if not SESSION.exists():
            raise unittest.SkipTest(f"regression sample missing: {SESSION}")
        cls.events = _events()

    def test_session_finished_far_inside_its_budget(self) -> None:
        summary = json.loads((SESSION / "summary.json").read_text())
        self.assertEqual(summary["result"], "SEARCH_EXHAUSTED")
        self.assertEqual(summary["finish_reason"], "SEARCH_EXHAUSTED")
        # 预算是 600 s / 100 规划周期 / 50 步；这次只用了 4 周期 3 步。
        self.assertLessEqual(summary["planning_cycles"], 5)
        self.assertLessEqual(summary["motion_steps"], 5)
        self.assertLess(summary["duration_s"], 600.0)

    def test_sixty_degree_goals_were_executed_as_one_thirty_degree_step(self) -> None:
        goals = {
            row["goal"]["goal_id"]: row["goal"]
            for row in self.events
            if row.get("event") == "selected_goal" and row.get("goal")
        }
        results = {
            row["goal_id"]: row
            for row in self.events
            if row.get("event") == "navigation_result" and row.get("goal_id")
        }
        sixty = [
            goal_id for goal_id, goal in goals.items()
            if abs(float(goal.get("relative_dyaw") or 0.0)) >= 55.0
        ]
        self.assertTrue(sixty, "archive should contain +-60 deg rotate goals")
        for goal_id in sixty:
            result = results[goal_id]
            requested = float(result["requested_motion"]["relative_yaw_deg"])
            observed = float(result["observed_motion"]["yaw_delta_deg"])
            self.assertEqual(result["status"], "succeeded")
            # 缺陷：60° 的逻辑目标被静默截断成单个 30° 原语并判成功。
            self.assertLessEqual(abs(requested), 31.0)
            self.assertLess(abs(observed), 40.0)
            self.assertNotIn("segment_count", result["requested_motion"])

    def test_candidate_generator_typeerror_preceded_the_exhaustion(self) -> None:
        names = [str(row.get("event") or "") for row in self.events]
        self.assertIn("candidate_generator_error", names)
        error = next(
            row for row in self.events
            if row.get("event") == "candidate_generator_error"
        )
        self.assertIn("TypeError", str(error.get("error")))
        self.assertIn("warn() takes 2 positional arguments", str(error.get("error")))
        self.assertLess(
            names.index("candidate_generator_error"),
            names.index("search_exhausted"),
        )
        exhausted = next(
            row for row in self.events if row.get("event") == "search_exhausted"
        )
        self.assertEqual(exhausted.get("reason"), "no exploration candidates")

    def test_lio_reported_meters_of_translation_for_in_place_turns(self) -> None:
        snapshot = json.loads((SESSION / "slam_map_3d_robot.json").read_text())
        pose = snapshot["pose"]
        drift = math.hypot(float(pose["x"]), float(pose["y"]))
        # 整场会话只有三次原地转向，轮式里程计一直在原点附近。
        self.assertGreater(drift, 2.0)
        self.assertEqual(snapshot["mapping_health"], "HEALTHY")
        self.assertTrue(snapshot["lio_pose_valid"])
        # 永久地图来自 aligned_scan 累积，map_3d 被当成 frame 不匹配丢掉。
        self.assertEqual(snapshot["source"], "aligned_scan_accumulated")
        self.assertEqual(snapshot["frame_id"], "pslam_odom")
        self.assertGreater(snapshot["dropped_reason_counts"]["map_frame_mismatch"], 0)
        # 40000 体素上限被 FIFO 淘汰卡死。
        self.assertEqual(snapshot["accumulated_voxels"], 40000)


class TestFixesPreventRegression(unittest.TestCase):
    """当前代码不得再复现上面四个缺陷。"""

    def _backend(self, motion: _FakeMotion) -> Go2WExperimentalBackend:
        return Go2WExperimentalBackend(
            execute_step=motion.execute,
            odometry=motion.odometry,
            config=Go2WBackendConfig(max_turn_deg_per_action=30.0),
        )

    def test_minus_sixty_needs_two_primitives_and_odometry_closure(self) -> None:
        motion = _FakeMotion()
        result = self._backend(motion).execute_goal(ExplorationGoal(
            goal_id="local_scan_002", goal_type=GOAL_ROTATE_VIEW, relative_dyaw=-60.0,
        )).result
        self.assertNotEqual(motion.steps, ["r30"])
        self.assertEqual(motion.steps[:2], ["r30", "r30"])
        self.assertEqual(result.status, NavigationStatus.SUCCEEDED)
        self.assertEqual(result.requested_motion["requested_total_deg"], -60.0)
        observed = result.observed_motion["observed_total_deg"]
        self.assertLess(observed, -50.0)
        self.assertLessEqual(abs(-60.0 - observed), 8.0)

    def test_plus_sixty_needs_two_primitives(self) -> None:
        motion = _FakeMotion()
        result = self._backend(motion).execute_goal(ExplorationGoal(
            goal_id="local_scan_003", goal_type=GOAL_ROTATE_VIEW, relative_dyaw=60.0,
        )).result
        self.assertNotEqual(motion.steps, ["l30"])
        self.assertEqual(motion.steps[:2], ["l30", "l30"])
        self.assertEqual(result.status, NavigationStatus.SUCCEEDED)
        self.assertGreater(result.observed_motion["observed_total_deg"], 50.0)
        self.assertGreaterEqual(len(result.provenance["segments"]), 2)

    def test_no_printf_style_ros_logger_calls_remain(self) -> None:
        """ROS 2 Foxy loggers accept exactly one message argument."""
        methods = {"debug", "info", "warn", "warning", "error", "fatal"}
        offenders: list[str] = []
        for root in (REPO / "app", REPO / "scripts", REPO / "ros2_ws"):
            for path in root.rglob("*.py"):
                try:
                    tree = ast.parse(path.read_text(errors="replace"))
                except SyntaxError:
                    continue
                for node in ast.walk(tree):
                    if not isinstance(node, ast.Call):
                        continue
                    func = node.func
                    if not isinstance(func, ast.Attribute) or func.attr not in methods:
                        continue
                    owner = ast.unparse(func.value)
                    if "get_logger()" not in owner and not owner.endswith("logger"):
                        continue
                    if len(node.args) > 1:
                        offenders.append(f"{path.relative_to(REPO)}:{node.lineno}")
        self.assertEqual(offenders, [])

    def test_candidate_generator_exception_is_planning_error_not_exhausted(self) -> None:
        calls = {"n": 0}

        def exploding(**_: object) -> list:
            calls["n"] += 1
            raise TypeError("warn() takes 2 positional arguments but 4 were given")

        scene = MockObservationScene(scenes=[
            MockSceneStep(objects=["desk"], bundle_id="obs_001"),
            MockSceneStep(objects=["chair"], bundle_id="obs_002"),
            MockSceneStep(objects=["shelf"], bundle_id="obs_003"),
        ])
        explorer = AutonomousExplorer(
            target="白色垃圾桶",
            observer=scene.observer(),
            matcher=scene.matcher(),
            verifier=scene.verifier(),
            backend=MockBackend(),
            policy=load_exploration_policy(),
            graph=ExplorationGraph(session_id="regression"),
            negative_target_key="白色垃圾桶",
            candidate_generator=exploding,
        )
        result = explorer.run()
        self.assertEqual(result.result, "PLANNING_ERROR")
        self.assertEqual(explorer.state, ExplorerState.FAILED.value)
        # 第一次异常允许重新观测后再规划，同一异常复现才判失败。
        self.assertGreaterEqual(calls["n"], 2)
        names = [str(item.get("event") or "") for item in explorer.events]
        self.assertIn("candidate_generator_error", names)
        self.assertNotIn("search_exhausted", names)

    def test_object_topology_projection_excludes_place_and_frontier(self) -> None:
        graph = SemanticNavigationGraph()
        graph.update_observation(
            observation_id="bundle_1", heading_sector=0,
            scene_objects=[
                {"id": "f_1", "label": "办公桌", "map_xyz": [1.0, 0.0, 0.0],
                 "confidence": 0.9},
                {"id": "f_2", "label": "白色垃圾桶", "map_xyz": [1.2, 0.4, 0.0],
                 "confidence": 0.9},
            ],
            scene_relations=[{"subject_id": "f_1", "object_id": "f_2",
                              "relation": "near", "confidence": 0.8}],
            pose={"x": 0.0, "y": 0.0, "yaw_rad": 0.0}, timestamp=1.0,
        )
        snapshot = graph.object_topology_snapshot()
        self.assertEqual(snapshot["schema_version"], "semantic_object_topology_v1")
        self.assertTrue(snapshot["nodes"])
        object_ids = {node["node_id"] for node in snapshot["nodes"]}
        for node in snapshot["nodes"]:
            self.assertEqual(node["node_type"], "OBJECT")
            self.assertTrue(node["node_id"].startswith("obj_"))
        for edge in snapshot["edges"]:
            self.assertIn(edge["from"], object_ids)
            self.assertIn(edge["to"], object_ids)
            self.assertNotIn(edge["relation"],
                             {"OBSERVED_FROM", "FRONTIER_TO", "MOVED_TO"})
        # 计划书 §14：内部 Place / Frontier 不许删除，只是不参与语义拓扑投影。
        internal_types = {node["node_type"] for node in graph.to_dict()["nodes"]}
        self.assertIn("PLACE", internal_types)

    def test_rotation_with_meters_of_lio_translation_is_not_healthy(self) -> None:
        validator = SpatialPoseValidator()
        first = validator.validate(SpatialPose(x=0.0, y=0.0, yaw=0.0), timestamp=0.0)
        self.assertTrue(first.accepted)
        self.assertEqual(first.health, "HEALTHY")
        rotation = MotionEvidence(
            command_type="ROTATE",
            requested_turn_deg=-30.0,
            requested_forward_m=0.0,
            wheel_delta_xy_m=0.02,
            wheel_delta_yaw_deg=-28.5,
            motion_completed_at=1.0,
        )
        drifted = validator.validate(
            SpatialPose(x=2.9, y=0.4, yaw=math.radians(-28.0)),
            rotation,
            timestamp=2.0,
        )
        self.assertFalse(drifted.accepted)
        self.assertNotEqual(drifted.health, "HEALTHY")
        self.assertEqual(drifted.health, "DEGRADED")
        self.assertEqual(drifted.reason_code, "LIO_DRIFT_DURING_ROTATION")
        # 拒绝后仍然保留最后一个可信位姿，供地图冻结使用。
        self.assertEqual(validator.last_good_pose, first.accepted_pose)


if __name__ == "__main__":
    unittest.main()
