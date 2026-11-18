from __future__ import annotations

from app.live_robot.autonomous_explorer import AutonomousExplorer
from app.live_robot.mock_observation_scene import scenario_no_target
from app.navigation.backend_factory import MockBackend
from app.navigation.exploration_config import load_exploration_policy
import pytest


class RecordingBackend(MockBackend):
    def __init__(self):
        super().__init__()
        self.dispatched = []

    def execute_goal(self, goal):
        self.dispatched.append(goal)
        return super().execute_goal(goal)


def make_explorer(backend, observer=None, on_event=None):
    scene = scenario_no_target(empty_scenes=2)
    policy = load_exploration_policy()
    policy.budget.max_planning_cycles = 4
    policy.budget.max_motion_steps = 4
    return AutonomousExplorer(target='蓝色垃圾桶', backend=backend,
        observer=observer or scene.observer(), matcher=scene.matcher(),
        verifier=scene.verifier(), policy=policy, on_event=on_event)


def test_pause_at_dispatch_boundary_does_not_send_a_new_goal():
    backend = RecordingBackend()
    holder = {}
    def event(row):
        if row['event'] == 'action_start': holder['explorer'].request_pause()
        if row['event'] == 'paused': holder['explorer'].request_stop()
    holder['explorer'] = make_explorer(backend, on_event=event)
    holder['explorer'].run()
    assert not backend.dispatched


@pytest.mark.parametrize('command', ['stop','pause_resume'])
def test_operator_command_at_action_event_invalidates_the_selected_goal(command):
    backend = RecordingBackend(); holder = {}; captures = []
    scene = scenario_no_target(empty_scenes=2); observe = scene.observer()
    def observer():
        captures.append(1)
        if len(captures) > 1: holder['explorer'].request_stop()
        return observe()
    def event(row):
        if row['event'] == 'action_start':
            if command == 'stop': holder['explorer'].request_stop()
            else:
                holder['explorer'].request_pause()
                holder['explorer'].request_resume()
    explorer = holder['explorer'] = make_explorer(backend,observer,event)
    result = explorer.run()
    assert result.result == 'OPERATOR_STOP'
    assert not backend.dispatched
    assert explorer._navigation_goal_attempts == 0
    assert len(captures) == (1 if command == 'stop' else 2)


def test_pause_after_action_event_before_backend_dispatch_is_still_effective():
    class PausingPoseBackend(RecordingBackend):
        armed = False
        def get_pose(self):
            if self.armed:
                self.armed = False
                holder['explorer'].request_pause()
            return super().get_pose()
    backend = PausingPoseBackend(); holder = {}
    def event(row):
        if row['event'] == 'action_start': backend.armed = True
        if row['event'] == 'paused': holder['explorer'].request_stop()
    holder['explorer'] = make_explorer(backend,on_event=event)
    assert holder['explorer'].run().result == 'OPERATOR_STOP'
    assert not backend.dispatched


def test_stop_from_paused_state_does_not_capture_an_extra_frame():
    backend = RecordingBackend(); holder = {}; captures = []
    scene = scenario_no_target(empty_scenes=2); observe = scene.observer()
    def observer(): captures.append(1); return observe()
    def event(row):
        if row['event'] == 'action_start': holder['explorer'].request_pause()
        if row['event'] == 'paused': holder['explorer'].request_stop()
    holder['explorer'] = make_explorer(backend,observer,event)
    assert holder['explorer'].run().result == 'OPERATOR_STOP'
    assert len(captures) == 1
    assert 'resumed' not in [row['event'] for row in holder['explorer'].events]


def test_rapid_pause_resume_discards_the_inflight_observation():
    backend = RecordingBackend()
    scene = scenario_no_target(empty_scenes=2)
    observe = scene.observer()
    holder = {}; captures = []
    def observer():
        captures.append(len(captures)+1)
        if len(captures) == 1:
            holder['explorer'].request_pause()
            holder['explorer'].request_resume()
        else:
            holder['explorer'].request_stop()
        return observe()
    holder['explorer'] = make_explorer(backend, observer=observer)
    holder['explorer'].run()
    assert len(captures) == 2
    assert not backend.dispatched
