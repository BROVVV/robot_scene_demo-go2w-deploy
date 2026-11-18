import math
from types import SimpleNamespace
from scripts.go2w.run_semantic_exploration import _Nav2MotionEvidenceRecorder
from app.navigation.models import ExplorationGoal
from app.navigation.robot_backend import RobotPose,PoseQuality


def test_requested_motion_uses_map_pose_and_observed_motion_uses_wheel_deltas():
    before=[1.,2.,0.]
    inner=SimpleNamespace(get_pose=lambda:RobotPose(10.,20.,math.pi/2,frame_id='map',quality=PoseQuality.METRIC))
    state={}
    recorder=_Nav2MotionEvidenceRecorder(inner,node=SimpleNamespace(_odom_snapshot=lambda:before),motion_state=state)
    recorder._odom_before=tuple(before)
    recorder._note_requested(ExplorationGoal('turn','NAVIGATE_POSE',position=(10.,20.),yaw=120.,frame='map'))
    assert state['pending_motion_evidence'].requested_forward_m==0.
    assert abs(state['pending_motion_evidence'].requested_turn_deg-30.)<1e-6
    before[0]+=.01
    recorder._note_completed()
    assert abs(state['pending_motion_evidence'].wheel_delta_xy_m-.01)<1e-6


def test_spin_once_yields_to_the_dedicated_executor(monkeypatch):
    waits=[]
    node=SimpleNamespace(executor=object())
    recorder=_Nav2MotionEvidenceRecorder(SimpleNamespace(),node=node,motion_state={})
    monkeypatch.setattr('scripts.go2w.run_semantic_exploration.time.sleep',waits.append)
    recorder.spin_once(.07)
    assert waits==[.07]
