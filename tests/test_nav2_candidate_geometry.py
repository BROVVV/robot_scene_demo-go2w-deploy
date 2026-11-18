from dataclasses import replace
import math
import pytest
from app.navigation.nav2_candidate_geometry import (observed_grid,scan_is_clear,
    nearer_frontier_goal,filter_frontiers_for_heading,bounded_heading_target)
from app.navigation.local_scan import select_local_scan_goal,LocalScanState
from app.navigation.models import ExplorationGoal
from app.spatial.models import SpatialMapSnapshot
from app.spatial.models import FrontierCandidate
from app.navigation.long_term_goal_selector import LongTermGoalSelector


def grid():
    return SpatialMapSnapshot(revision=1,resolution_m=.05,origin=(-2.,-2.),width=120,height=80,
        free=[(x,y) for y in range(80) for x in range(120)])


def test_bounded_heading_target_uses_relative_bearing_without_changing_heading():
    x,y=bounded_heading_target((1.,2.),math.radians(30.),.2,-15.)
    assert x==pytest.approx(1.+.2*math.cos(math.radians(15.)))
    assert y==pytest.approx(2.+.2*math.sin(math.radians(15.)))


@pytest.mark.parametrize('position,yaw,distance,offset,limit',[
    ((0.,0.),0.,-.1,0.,20.), ((0.,0.),0.,.1,21.,20.),
    ((0.,float('nan')),0.,.1,0.,20.), ((0.,0.),0.,.1,0.,181.),
])
def test_bounded_heading_target_rejects_invalid_or_out_of_scope_geometry(
        position,yaw,distance,offset,limit):
    with pytest.raises(ValueError):
        bounded_heading_target(position,yaw,distance,offset,maximum_offset_deg=limit)


def test_scan_filters_blocked_directions_before_spending_motion_failure_budget():
    g=grid();g.occupied=[(47,45)]
    assert not scan_is_clear(g,(0,0),0,30)
    accepted=[]
    goal,state=select_local_scan_goal(current_yaw_deg=0,max_local_rotations=4,state=LocalScanState(),
        goal_is_allowed=lambda angle:accepted.append(angle) or angle<0)
    assert goal['relative_dyaw']<0 and state.steps==1
    goal,state=select_local_scan_goal(current_yaw_deg=0,max_local_rotations=4,state=LocalScanState(),goal_is_allowed=lambda _:False)
    assert goal is None and state.steps==0


def test_unknown_space_is_not_filled_by_candidate_filter():
    g=grid();g.free=[]
    assert observed_grid(g).blocked(0,0)
    assert not scan_is_clear(g,(0,0),0,30)
    assert not scan_is_clear(None,(0,0),0,30)


def frontier():
    return ExplorationGoal(goal_id='frontier',goal_type='NAVIGATE_POSE',frame='map',position=(2.,0),yaw=0,
        provenance={'intent':{'intent_type':'EXPLORE_FRONTIER'}})


def route():return {'waypoints':[(i*.1,0.) for i in range(21)]}


def test_large_clear_space_uses_long_frontier_goal_and_narrow_end_stays_safe():
    g=grid();goal=frontier()
    open_selected=nearer_frontier_goal(goal,route(),g,maximum_step_m=3)
    assert 1.1 <= open_selected.position[0] <= 1.25
    assert math.dist(open_selected.position, goal.position) >= .75
    assert open_selected.provenance['nearer_frontier_goal']['source']=='observed_map_route_prefix'
    g.occupied=[(x,50) for x in range(65,85)] # x>=1.25, left gap .245<.4
    selected=nearer_frontier_goal(goal,route(),g,maximum_step_m=3)
    assert .2<=selected.position[0]<=1.25
    assert selected.provenance['nearer_frontier_goal']['motion_authorized'] is False
    assert goal.position==(2.,0.)


@pytest.mark.parametrize('case',['approach','frame','different_route','unknown','trial'])
def test_nearer_goal_never_changes_target_semantics_or_short_trial(case):
    goal=frontier();g=grid();r=route();limit=3
    g.occupied=[(x,50) for x in range(40,85)]
    if case=='approach':goal=replace(goal,provenance={'intent':{'intent_type':'APPROACH_TARGET'}})
    elif case=='frame':goal=replace(goal,frame='odom')
    elif case=='different_route':r['waypoints'][-1]=(5.,0.)
    elif case=='unknown':g.free=[]
    else:limit=.12
    assert nearer_frontier_goal(goal,r,g,maximum_step_m=limit) is goal


def test_distant_frontier_uses_long_prefix_within_the_hard_goal_limit():
    goal=replace(frontier(),position=(3.,0.),provenance={
        'intent':{'intent_type':'EXPLORE_FRONTIER','preferred_position':[6.,0.]}})
    plan={'waypoints':[(i*.1,0.) for i in range(61)]}
    selected=nearer_frontier_goal(goal,plan,grid(),maximum_step_m=3.)
    assert 1.5<=selected.position[0]<=3.
    assert selected.provenance['nearer_frontier_goal']['selected_path_length_m']<=3.


def test_same_place_topological_hint_can_propose_a_checked_nearer_frontier():
    g=grid();g.occupied=[(x,50) for x in range(65,85)]
    plan={'waypoints':[(0.,0.)],'planner_source':'place_graph_topological'}
    selected=nearer_frontier_goal(frontier(),plan,g,maximum_step_m=3,current_position=(0,0))
    assert .2<=selected.position[0]<2
    assert selected.provenance['nearer_frontier_goal']['source']=='observed_map_direct_prefix'
    assert selected.provenance['nearer_frontier_goal']['motion_authorized'] is False


def test_missing_route_hint_proposes_observed_long_goal_with_frontier_standoff():
    selected=nearer_frontier_goal(frontier(),None,grid(),maximum_step_m=3.,
        current_position=(0.,0.),current_yaw_deg=0.,maximum_turn_deg=20.)
    proof=selected.provenance['nearer_frontier_goal']
    assert proof['source']=='safe_local_detour'
    assert 1.1<=math.dist((0.,0.),selected.position)<=1.25
    assert math.dist(selected.position, frontier().position)>=.75
    assert proof['motion_authorized'] is False


def test_unreachable_empty_route_hint_uses_the_same_observed_local_fallback():
    unavailable={'planner_source':'unavailable','reachable':False,'waypoints':[]}
    selected=nearer_frontier_goal(frontier(),unavailable,grid(),maximum_step_m=3.,
        current_position=(0.,0.),current_yaw_deg=0.,maximum_turn_deg=20.)
    proof=selected.provenance['nearer_frontier_goal']
    assert proof['source']=='safe_local_detour'
    assert 1.1<=math.dist((0.,0.),selected.position)<=1.25
    assert proof['motion_authorized'] is False


def test_local_fallback_reconciles_only_stale_cells_under_current_body():
    g=grid()
    for y in range(g.height):
        for x in range(g.width):
            wx=g.origin[0]+(x+.5)*g.resolution_m
            wy=g.origin[1]+(y+.5)*g.resolution_m
            if abs(wx)<=.35 and abs(wy)<=.215:
                g.occupied.append((x,y))
    selected=nearer_frontier_goal(frontier(),None,g,maximum_step_m=3.,
        current_position=(0.,0.),current_yaw_deg=0.,maximum_turn_deg=20.)
    proof=selected.provenance['nearer_frontier_goal']
    assert proof['source']=='safe_local_detour'
    assert proof['current_footprint_overlay']['policy']=='current_physical_body_only'
    assert proof['current_footprint_overlay']['masked_occupied_cells']>0
    assert 1.1<=math.dist((0.,0.),selected.position)<=1.25


def test_current_body_overlay_never_clears_the_future_sweep():
    g=grid()
    for y in range(g.height):
        for x in range(g.width):
            wx=g.origin[0]+(x+.5)*g.resolution_m
            wy=g.origin[1]+(y+.5)*g.resolution_m
            if (abs(wx)<=.35 and abs(wy)<=.215) or (
                    .62<=math.hypot(wx,wy)<=.67):
                g.occupied.append((x,y))
    goal=frontier()
    assert nearer_frontier_goal(goal,None,g,maximum_step_m=3.,
        current_position=(0.,0.),current_yaw_deg=0.,maximum_turn_deg=20.) is goal


def test_local_fallback_can_only_leave_a_known_rear_padding_cell_forward():
    g=grid()
    # Cell centre (-0.375, 0.025): behind the 0.35 m physical body but inside
    # the 0.39 m padded footprint.
    g.occupied=[(32,40)]
    selected=nearer_frontier_goal(frontier(),None,g,maximum_step_m=3.,
        current_position=(0.,0.),current_yaw_deg=0.,maximum_turn_deg=20.)
    egress=selected.provenance['nearer_frontier_goal']['initial_footprint_egress']
    assert egress['allowed'] is True
    assert egress['policy']=='known_rear_corner_padding_forward_egress_only'
    assert .2<=selected.position[0]<=.6


def test_local_fallback_never_egresses_a_side_padding_obstacle():
    g=grid()
    # This cell is outside the physical width but inside the padded side edge.
    g.occupied=[(40,44)]
    goal=frontier()
    assert nearer_frontier_goal(goal,None,g,maximum_step_m=3.,
        current_position=(0.,0.),current_yaw_deg=0.,maximum_turn_deg=20.) is goal


def test_missing_route_hint_does_not_invent_free_space():
    unknown=grid();unknown.free=[]
    goal=frontier()
    assert nearer_frontier_goal(goal,None,unknown,maximum_step_m=3.,
        current_position=(0.,0.),current_yaw_deg=0.,maximum_turn_deg=20.) is goal


def test_nearer_goal_uses_safe_local_detour_when_direct_prefix_has_no_reserve():
    g=grid()
    # Direct travel has 0.445 m left body clearance: above the backend's
    # 0.40 m hard threshold, but below the candidate prefilter's 0.50 m reserve.
    row=int((.70-g.origin[1])/g.resolution_m)
    g.occupied=[(x,row) for x in range(36,57)]
    selected=nearer_frontier_goal(frontier(),route(),g,maximum_step_m=.45,
        current_position=(0.,0.),current_yaw_deg=0.,maximum_turn_deg=20.)
    proof=selected.provenance['nearer_frontier_goal']
    assert proof['source']=='safe_local_detour'
    assert -.45<=selected.position[1]<0.
    assert .2<=math.dist((0.,0.),selected.position)<=.45+1e-9
    assert abs(proof['heading_delta_deg'])<=20.
    assert proof['selected_min_side_gap_m']>=proof['minimum_prefilter_side_gap_m']
    assert proof['motion_authorized'] is False


def test_nearer_goal_does_not_invent_detour_when_all_headings_are_too_narrow():
    g=grid()
    for y in range(g.height):
        for x in range(g.width):
            wx=g.origin[0]+(x+.5)*g.resolution_m
            wy=g.origin[1]+(y+.5)*g.resolution_m
            if .62<=math.hypot(wx,wy)<=.67:
                g.occupied.append((x,y))
    goal=frontier()
    assert nearer_frontier_goal(goal,route(),g,maximum_step_m=.45,
        current_position=(0.,0.),current_yaw_deg=0.,maximum_turn_deg=20.) is goal


@pytest.mark.parametrize('yaw,turn',[(None,20.),(float('nan'),20.),(0.,None),(0.,181.)])
def test_nearer_goal_needs_valid_heading_scope_before_inventing_a_detour(yaw,turn):
    g=grid()
    row=int((.70-g.origin[1])/g.resolution_m)
    g.occupied=[(x,row) for x in range(36,57)]
    goal=frontier()
    assert nearer_frontier_goal(goal,route(),g,maximum_step_m=.45,
        current_position=(0.,0.),current_yaw_deg=yaw,maximum_turn_deg=turn) is goal


@pytest.mark.parametrize('position',[None,(1.,0.),(float('nan'),0.)])
def test_degenerate_topological_route_needs_a_fresh_matching_start(position):
    goal=frontier();g=grid();g.occupied=[(x,50) for x in range(65,85)]
    plan={'waypoints':[(0.,0.)],'planner_source':'place_graph_topological'}
    assert nearer_frontier_goal(goal,plan,g,maximum_step_m=3,current_position=position) is goal


def test_trial_selects_admissible_frontier_instead_of_high_gain_side_target():
    side=FrontierCandidate('side',position=(.2,-1.84),frame='map',
        bearing_deg=-86.,spatial_information_gain=1.)
    ahead=FrontierCandidate('ahead',position=(1.7,-.04),frame='map',
        bearing_deg=0.,spatial_information_gain=.4)
    selector=LongTermGoalSelector()
    assert selector.select(match_state='ZERO',frontiers=[side,ahead]).intent.target_frontier_id=='side'
    allowed,rejected=filter_frontiers_for_heading([side,ahead],(.09,0.),0.,maximum_turn_deg=10.)
    assert allowed==[ahead] and rejected[0]['reason']=='outside_trial_heading'
    assert selector.select(match_state='ZERO',frontiers=allowed).intent.target_frontier_id=='ahead'
    # Discovery and map reachability are unchanged by this temporary scope.
    assert side.reachable and ahead.reachable and side.position==(.2,-1.84)


def test_trial_heading_uses_current_coordinates_and_handles_yaw_wrap():
    stale=FrontierCandidate('stale',position=(0.,1.),frame='map',bearing_deg=0.)
    rear=FrontierCandidate('wrap',position=(-1.,-.01),frame='map',bearing_deg=0.)
    allowed,rejected=filter_frontiers_for_heading([stale,rear],(0.,0.),179.,maximum_turn_deg=3.)
    assert allowed==[rear] and rejected[0]['frontier_id']=='stale'


@pytest.mark.parametrize('position,yaw',[(None,0.),((0.,float('nan')),0.),((0.,0.),float('nan'))])
def test_missing_trial_pose_never_admits_a_frontier(position,yaw):
    allowed,rejected=filter_frontiers_for_heading(
        [FrontierCandidate('f',position=(1.,0.),frame='map')],position,yaw,maximum_turn_deg=10.)
    assert allowed==[] and rejected[0]['reason']=='current_pose_unavailable'


@pytest.mark.parametrize('limit',[-1.,181.,float('nan')])
def test_invalid_trial_heading_limit_fails_closed(limit):
    with pytest.raises(ValueError):filter_frontiers_for_heading([], (0.,0.),0.,maximum_turn_deg=limit)


def test_trial_rejects_foreign_frame_zero_distance_and_invalid_position():
    candidates=[FrontierCandidate('foreign',position=(1.,0.),frame='odom'),
        FrontierCandidate('same',position=(0.,0.),frame='map'),
        FrontierCandidate('missing',position=None,frame='map')]
    allowed,rejected=filter_frontiers_for_heading(candidates,(0.,0.),0.,maximum_turn_deg=10.)
    assert not allowed and len(rejected)==3
