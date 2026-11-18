"""Synthetic large-map planning only: no ROS imports, publishers or robot IO."""
import math
import time

import pytest

from app.navigation.local_goal_executor import LocalGoalExecutor
from app.navigation.long_term_goal_selector import LongTermGoalSelector
from app.navigation.nav2_candidate_geometry import nearer_frontier_goal, observed_grid
from app.navigation.robot_backend import RobotCapabilities
from app.navigation.semantic_route_planner import SemanticRoutePlanner
from app.spatial.frontier_extractor import FrontierExtractor
from app.spatial.models import SpatialMapSnapshot, SpatialPose
from app.spatial.nav2_spatial_provider import Nav2SpatialProvider
from tests.test_nav2_search_backend import _backend, _pose
from app.navigation.nav2_search_backend import ActionPoll, ACTION_SUCCEEDED


def exercise_large_map(side=1000, cycles=3, resolution_m=.1):
    """Return measurements, never a real navigation acceptance result."""
    started = time.perf_counter()
    grid = SpatialMapSnapshot(revision=1, resolution_m=resolution_m, width=side, height=side,
        free=[(x,y) for y in range(side//10, side*9//10)
                     for x in range(side//10, side*9//10)],
        provenance={'frame_id':'map'}, source='fixture_synthetic_large_map')
    provider = Nav2SpatialProvider(enable_ros=False)
    extractor = provider.frontier_extractor
    position = ((side//2+.5)*resolution_m,) * 2
    pose = SpatialPose(*position, frame_id='map')
    # Reproduce the previous 20m discovery cutoff on this exact input.
    assert FrontierExtractor(min_component_size=1).extract(grid, pose) == []
    occupancy = observed_grid(grid)
    measurements = []
    for cycle in range(cycles):
        t = time.perf_counter()
        frontiers = extractor.extract(grid, pose)
        assert frontiers
        selected = LongTermGoalSelector().select(match_state='ZERO', frontiers=frontiers)
        assert selected is not None
        intent = selected.intent
        route = SemanticRoutePlanner(max_waypoints=32).plan(
            start_pose=pose, target_type='FRONTIER', target_position=intent.preferred_position,
            map_snapshot=grid, frame_id='map')
        assert route and route.reachable and route.planner_source == 'grid_astar'
        assert math.dist(route.waypoints[-1], intent.preferred_position) < .15
        assert len(route.waypoints) <= 32
        executor = LocalGoalExecutor(max_forward_step_m=3.)
        executor.begin(intent)
        goal = executor.next_goal(current_position=(pose.x,pose.y), current_yaw_deg=math.degrees(pose.yaw),
                                 capabilities=RobotCapabilities(supports_metric_navigation=True))
        goal.provenance['intent'] = intent.to_dict()
        goal = nearer_frontier_goal(goal, route.to_dict(), grid, maximum_step_m=3.)
        assert .2 < math.dist(goal.position, (pose.x,pose.y)) <= 3.000001
        # Only Action IO is substituted. Production Nav2 backend still checks
        # the full observed footprint/path before sending to the fake client.
        backend, planner, navigator = _backend(grid=occupancy,
            pose=_pose(x=pose.x,y=pose.y,yaw_deg=math.degrees(pose.yaw)))
        backend._goal_xy_tolerance_m=.03
        backend._goal_yaw_tolerance_rad=.05
        dx,dy = goal.position[0]-pose.x, goal.position[1]-pose.y
        n = math.ceil(math.hypot(dx,dy)/.05)
        path = [(pose.x+dx*i/n, pose.y+dy*i/n) for i in range(n+1)]
        planner.queue(ActionPoll(status=ACTION_SUCCEEDED,path=path))
        handle = backend.execute_goal(goal)
        assert not navigator.sent
        status = backend.get_navigation_status(handle)
        assert status.status.value == 'running', status
        assert len(navigator.sent)==1
        final = _pose(x=goal.position[0],y=goal.position[1],yaw_deg=goal.yaw)
        backend._pose_provider=lambda:final
        navigator.queue(ActionPoll(status=ACTION_SUCCEEDED))
        result = backend.get_navigation_status(handle)
        assert result.status.value=='succeeded', result
        assert result.observed_motion['final_position_error_m']==0.
        measurements.append({'cycle':cycle+1, 'frontier_distance_m':frontiers[0].distance_m,
            'route_length_m':route.path_length_m, 'goal':list(goal.position),
            'goal_step_m':math.hypot(dx,dy), 'software_seconds':time.perf_counter()-t})
        pose = SpatialPose(final.x,final.y,final.yaw,frame_id='map')
    return {'source':'fixture_synthetic_large_map', 'real_robot_motion':False,
            'map_cells':side*side, 'map_size_m':[side*resolution_m]*2,
            'resolution_m':resolution_m,'cycles':measurements,'total_seconds':time.perf_counter()-started}


def test_large_frontier_route_prefix_and_backend_sequence():
    result = exercise_large_map()
    assert len(result['cycles'])==3


def test_long_route_sampling_preserves_both_endpoints():
    grid = SpatialMapSnapshot(revision=1, resolution_m=.1, width=502,height=5,
        free=[(x,y) for x in range(502) for y in range(5)])
    route = SemanticRoutePlanner(max_waypoints=32).plan(start_pose=SpatialPose(.15,.25),
        target_type='FRONTIER',target_position=(49.95,.25),map_snapshot=grid)
    assert route and route.reachable
    assert route.waypoints[0]==pytest.approx((.15,.25))
    assert route.waypoints[-1]==pytest.approx((49.95,.25))
    assert len(route.waypoints)==32
    assert route.path_length_m==pytest.approx(49.8)
    # Counterexample: the old stride on this 499-cell path loses 0.3m.
    full = [(.15+i*.1, .25) for i in range(499)]
    old_sample = full[::len(full)//32]
    assert math.dist(old_sample[-1], (49.95,.25)) > .15
