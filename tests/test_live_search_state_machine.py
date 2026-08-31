import unittest

from app.live_robot.search_state_machine import (
    SearchState,
    SearchStateMachine,
    SensorSnapshot,
    VisualEvidence,
)


class LiveSearchStateMachineTests(unittest.TestCase):
    def test_fail_closed_sensors_do_not_enter_observe(self):
        machine = SearchStateMachine()
        machine.start()
        state = machine.sensors(
            SensorSnapshot(camera_fresh=True, lidar_fresh=False, robot_stationary=True)
        )
        self.assertEqual(state, SearchState.WAIT_FOR_SENSORS)

    def test_llm_context_and_incomplete_visual_evidence_cannot_confirm(self):
        evidence = VisualEvidence(
            bbox=True,
            mask=True,
            crop_verify=True,
            track_vote=True,
            evidence_gate=True,
            source="llm_context",
        )
        self.assertFalse(evidence.confirmed)
        self.assertFalse(
            VisualEvidence(True, False, True, True, True).confirmed
        )

    def test_confirmed_2d_target_never_becomes_metric_pose(self):
        machine = SearchStateMachine(stop_settle_seconds=0.5)
        machine.start()
        machine.sensors(SensorSnapshot(True, True, True))
        machine.observation_elapsed(0.5)
        machine.scene_understood()
        machine.detection(True)
        machine.verify(VisualEvidence(True, True, True, True, True))
        machine.localization(False)
        self.assertEqual(machine.state, SearchState.TARGET_CONFIRMED)
        self.assertEqual(machine.spatial_state, "target_2d_only")
        machine.finish_confirmed()
        self.assertEqual(machine.state, SearchState.FINISH)

    def test_motion_mode_cannot_reach_plan_step_without_authorization(self):
        machine = SearchStateMachine(mode="step_search", motion_allowed=False)
        machine.start()
        machine.sensors(SensorSnapshot(True, True, True))
        machine.observation_elapsed(0.8)
        machine.scene_understood()
        machine.detection(False)
        machine.next_view_unavailable()
        self.assertEqual(machine.safety_checked(True), SearchState.FAILED)

    def test_step_search_plan_move_wait_reobserve_cycle(self):
        machine = SearchStateMachine(
            mode="step_search", motion_allowed=True
        )
        machine.start()
        machine.sensors(SensorSnapshot(True, True, True))
        machine.observation_elapsed(0.8)
        machine.scene_understood()
        machine.detection(False)
        machine.next_view_unavailable()
        self.assertEqual(machine.safety_checked(True), SearchState.PLAN_STEP)
        self.assertEqual(machine.plan_step("r30"), SearchState.MOVE)
        self.assertEqual(
            machine.motion_started("r30"), SearchState.WAIT_FOR_STOP
        )
        self.assertEqual(
            machine.motion_completed("r30", verified=True),
            SearchState.REOBSERVE,
        )
        self.assertEqual(
            machine.sensors(SensorSnapshot(True, True, True)),
            SearchState.OBSERVE,
        )

    def test_semantic_next_view_selection_is_traced_and_keeps_safety_gate(self):
        machine = SearchStateMachine(mode="step_search", motion_allowed=True)
        machine.start()
        machine.sensors(SensorSnapshot(True, True, True))
        machine.observation_elapsed(0.8)
        machine.scene_understood()
        machine.detection(False)
        self.assertEqual(
            machine.next_view_selected(
                source="hybrid", directive_id="directive_1", confidence=0.8
            ),
            SearchState.CHECK_SAFETY,
        )
        self.assertEqual(machine.trace[-1]["source"], "hybrid")
        self.assertEqual(machine.safety_checked(True), SearchState.PLAN_STEP)

    def test_unverified_motion_still_enters_reobserve(self):
        machine = SearchStateMachine(
            mode="step_search", motion_allowed=True
        )
        machine.start()
        machine.sensors(SensorSnapshot(True, True, True))
        machine.observation_elapsed(0.8)
        machine.scene_understood()
        machine.detection(False)
        machine.next_view_unavailable()
        machine.safety_checked(True)
        machine.plan_step("f")
        machine.motion_started("f")
        self.assertEqual(
            machine.motion_completed("f", verified=False),
            SearchState.REOBSERVE,
        )

    def test_llm_line_can_confirm_without_mask_and_track_vote(self):
        evidence = VisualEvidence(
            bbox=True,
            mask=False,
            crop_verify=True,
            track_vote=False,
            evidence_gate=True,
            source="llm_quick",
            require_mask=False,
            require_track_vote=False,
        )
        self.assertTrue(evidence.confirmed)
        self.assertFalse(
            VisualEvidence(
                bbox=True,
                mask=False,
                crop_verify=False,
                track_vote=False,
                evidence_gate=True,
                source="llm_quick",
                require_mask=False,
                require_track_vote=False,
            ).confirmed
        )


if __name__ == "__main__":
    unittest.main()
