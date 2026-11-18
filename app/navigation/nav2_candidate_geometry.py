"""Filter search candidates against the observed map before final Nav2 planning.

This never authorizes motion. The production backend still computes a fresh
Nav2 path, checks its clearance, then arms and sends NavigateToPose.
"""
from dataclasses import replace
import math

from .corridor_analyzer import (CorridorGrid, DECISION_OK, HARD_SIDE_GAP_M,
                               classify_forward_padding_egress,
                               classify_rotation_sweep, classify_path_clearance,
                               current_footprint_observed_grid,
                               measure_path_clearance, ROTATION_TRANSLATION_TOL_M)


# The final Nav2 plan is computed from a newer costmap than this candidate
# snapshot.  A prefix exactly on the hard 0.40 m boundary can therefore pass
# here and fail a moment later in the authoritative backend.  Keep 10 cm for
# map/cell/path differences; the backend still owns the actual motion decision.
LOCAL_PREFIX_CLEARANCE_RESERVE_M = 0.10
LOCAL_PREFIX_MIN_SIDE_GAP_M = HARD_SIDE_GAP_M + LOCAL_PREFIX_CLEARANCE_RESERVE_M

# Strategic goals may use the profile's full mapped-space allowance.  Unknown
# frontier boundaries keep an observation standoff; only genuinely short known
# paths degrade to a local hop.
AUTONOMOUS_STRATEGIC_GOAL_MAX_M = 3.0
AUTONOMOUS_FRONTIER_STANDOFF_M = 0.8


def bounded_heading_target(position, yaw_rad, distance_m, heading_offset_deg,
                           *, maximum_offset_deg=20.0):
    """Return a short target in a bounded bearing sector.

    This is geometry only: it does not plan, reserve a trial slot, arm, or
    authorize motion.  Callers must still run the normal Nav2/corridor gates.
    """
    try:
        x, y = (float(value) for value in position)
        yaw = float(yaw_rad)
        distance = float(distance_m)
        offset = float(heading_offset_deg)
        limit = float(maximum_offset_deg)
    except (TypeError, ValueError):
        raise ValueError('invalid bounded target geometry') from None
    if (not all(math.isfinite(value) for value in (x, y, yaw, distance, offset, limit))
            or distance < 0.0 or not 0.0 <= limit <= 180.0 or abs(offset) > limit):
        raise ValueError('invalid bounded target geometry')
    bearing = yaw + math.radians(offset)
    return x + distance * math.cos(bearing), y + distance * math.sin(bearing)


def observed_grid(snapshot):
    if snapshot is None or snapshot.width <= 0 or snapshot.height <= 0:
        return None
    if not math.isfinite(snapshot.resolution_m) or snapshot.resolution_m <= 0:
        return None
    data=[-1]*(snapshot.width*snapshot.height)
    for cells,value in ((snapshot.free,0),(snapshot.unknown,-1),(snapshot.occupied,100)):
        for x,y in cells:
            if 0<=x<snapshot.width and 0<=y<snapshot.height:
                data[y*snapshot.width+x]=value
    return CorridorGrid(snapshot.resolution_m,*snapshot.origin,snapshot.width,snapshot.height,data)


def scan_is_clear(snapshot, position, yaw_deg, delta_deg):
    grid=observed_grid(snapshot)
    if grid is None or position is None:
        return False
    decision=classify_rotation_sweep(grid,*position,math.radians(yaw_deg),
        math.radians(yaw_deg+delta_deg),minimum_sweep_rad=math.radians(3.),
        translation_radius_m=ROTATION_TRANSLATION_TOL_M)
    return decision.applicable and decision.decision==DECISION_OK


def filter_frontiers_for_heading(frontiers, position, yaw_deg, *, maximum_turn_deg):
    """Constrain a bounded trial's selection without changing map reachability.

    Recompute bearings from the current map pose: an old/rounded frontier
    bearing is not evidence that its next local goal fits the current scope.
    This is only candidate admission; Nav2 and the trial guard still validate
    the final path and goal before any reservation or arm.
    """
    if not math.isfinite(maximum_turn_deg) or not 0 <= maximum_turn_deg <= 180:
        raise ValueError('invalid candidate heading limit')
    valid_pose = (position is not None and len(position) == 2
                  and all(math.isfinite(v) for v in (*position, yaw_deg)))
    allowed, rejected = [], []
    for frontier in frontiers:
        point = frontier.position
        reason, delta = '', None
        if not valid_pose:
            reason = 'current_pose_unavailable'
        elif (frontier.frame != 'map' or point is None or len(point) != 2
              or not all(math.isfinite(v) for v in point)):
            reason = 'invalid_map_frontier'
        else:
            dx, dy = point[0]-position[0], point[1]-position[1]
            if math.hypot(dx, dy) < 1e-6:
                reason = 'frontier_at_current_pose'
            else:
                bearing = math.degrees(math.atan2(dy, dx))
                delta = math.remainder(bearing-yaw_deg, 360.)
                if abs(delta) > maximum_turn_deg+1e-9:
                    reason = 'outside_trial_heading'
        if reason:
            rejected.append({'frontier_id': frontier.frontier_id, 'reason': reason,
                             'heading_delta_deg': delta})
        else:
            allowed.append(frontier)
    return allowed, rejected


def _clear_with_prefix_reserve(path, grid):
    decision = classify_path_clearance(measure_path_clearance(path, grid))
    return (decision.decision == DECISION_OK
            and decision.min_side_gap_m is not None
            and decision.min_side_gap_m >= LOCAL_PREFIX_MIN_SIDE_GAP_M), decision


def _safe_local_detour(goal, endpoint, grid, *, maximum_step_m, current_position,
                       current_yaw_deg, maximum_turn_deg,
                       current_footprint_overlay=None):
    """Propose a short, observed-map detour inside the current trial scope.

    This is deliberately a candidate generator.  Nav2 replans it against its
    current costmap and the production backend repeats footprint, corridor and
    budget checks before it may reserve or arm a motion step.
    """
    try:
        start = tuple(float(v) for v in current_position)
        yaw = float(current_yaw_deg)
        turn_limit = float(maximum_turn_deg)
        if (len(start) != 2 or not all(math.isfinite(v) for v in (*start, yaw, turn_limit))
                or not 0.0 <= turn_limit <= 180.0):
            return None
    except (TypeError, ValueError):
        return None
    # Match the backend's independent starting-footprint gate.  A known cell in
    # the narrow rear padding shell is handled per candidate below: a short,
    # monotonic forward path may leave it without shrinking the footprint for
    # side/front obstacles, UNKNOWN cells, turns, or arbitrary goals.
    footprint = classify_rotation_sweep(
        grid, *start, math.radians(yaw), math.radians(yaw),
        minimum_sweep_rad=0.0, margin_rad=0.0,
    )
    footprint_blocked = not footprint.passable
    distance_limit = min(3.0, float(maximum_step_m))
    if distance_limit < 0.2:
        return None
    # Five-degree samples keep the search bounded and include both trial edges.
    offsets = [0.0]
    step = 5.0
    sample = step
    while sample < turn_limit - 1e-9:
        offsets.extend((sample, -sample))
        sample += step
    if turn_limit > 1e-9:
        offsets.extend((turn_limit, -turn_limit))
    offsets = list(dict.fromkeys(round(v, 9) for v in offsets))
    distances = []
    distance = distance_limit
    while distance >= 0.2 - 1e-9:
        distances.append(round(distance, 9))
        distance -= 0.05

    target_distance = math.dist(start, endpoint)
    candidates = []
    for offset in offsets:
        heading = yaw + offset
        heading_rad = math.radians(heading)
        for travel in distances:
            end = (start[0] + travel * math.cos(heading_rad),
                   start[1] + travel * math.sin(heading_rad))
            progress = target_distance - math.dist(end, endpoint)
            if progress < max(0.02, 0.10 * travel):
                continue
            count = max(1, math.ceil(travel / 0.05))
            path = [
                (start[0] + (end[0] - start[0]) * i / count,
                 start[1] + (end[1] - start[1]) * i / count)
                for i in range(count + 1)
            ]
            footprint_egress = None
            if footprint_blocked:
                footprint_egress = classify_forward_padding_egress(
                    grid, *start, math.radians(yaw), path,
                    max_distance_m=0.60, max_lateral_m=0.05,
                    allow_unknown_rear=False,
                    allow_known_rear_corner_padding=True,
                    max_segment_angle_rad=math.radians(5.0),
                )
                if not footprint_egress.allowed:
                    continue
            clear, decision = _clear_with_prefix_reserve(path, grid)
            if not clear:
                continue
            # Every admitted path already meets the hard clearance reserve.
            # Prefer the longest useful strategic advance, then extra margin;
            # the smaller-turn tie-break avoids gratuitous steering.
            candidates.append(((travel, progress, float(decision.min_side_gap_m),
                                -abs(offset)), end, heading, offset, progress,
                               float(decision.min_side_gap_m), footprint_egress))
    if not candidates:
        return None
    _, end, heading, offset, progress, min_gap, footprint_egress = max(
        candidates, key=lambda item: item[0]
    )
    return replace(goal, position=end, yaw=heading, provenance={**goal.provenance,
        'nearer_frontier_goal': dict(original_position=list(goal.position),
            original_yaw=goal.yaw, selected_path_length_m=math.dist(start, end),
            source='safe_local_detour', heading_delta_deg=offset,
            target_progress_m=progress, selected_min_side_gap_m=min_gap,
            minimum_prefilter_side_gap_m=LOCAL_PREFIX_MIN_SIDE_GAP_M,
            current_footprint_overlay=current_footprint_overlay,
            initial_footprint_egress=(
                footprint_egress.to_dict() if footprint_egress is not None else None
            ),
            motion_authorized=False)})


def _overlay_current_physical_footprint(grid, current_position, current_yaw_deg):
    """Use the same bounded stale-self-cell policy as the execution backend.

    The semantic map snapshot and Nav2's published map can be sampled at
    slightly different instants.  Nav2 already clears cells whose centres are
    physically underneath the robot, while the candidate snapshot may retain
    those stale self/UNKNOWN cells and reject every otherwise valid local hop.
    Reconcile only the proven current body volume here.  Missing pose evidence
    keeps the unmodified fail-closed grid.
    """
    try:
        position = tuple(float(value) for value in current_position)
        yaw_deg = float(current_yaw_deg)
        if len(position) != 2 or not all(math.isfinite(value)
                                         for value in (*position, yaw_deg)):
            return grid, None
    except (TypeError, ValueError):
        return grid, None
    return current_footprint_observed_grid(
        grid, position[0], position[1], math.radians(yaw_deg)
    )


def nearer_frontier_goal(goal, route_plan, snapshot, *, maximum_step_m, current_position=None,
                         current_yaw_deg=None, maximum_turn_deg=None):
    """Choose a shorter safe prefix when the route to a frontier narrows.

    Explicit target/approach/verification poses are never silently changed.
    No safe prefix leaves the original goal for the final backend to reject.
    """
    intent=goal.provenance.get('intent') or {}
    if (goal.frame!='map' or goal.goal_type!='NAVIGATE_POSE'
            or not isinstance(intent,dict) or intent.get('intent_type')!='EXPLORE_FRONTIER'
            or goal.position is None):
        return goal
    grid=observed_grid(snapshot)
    if grid is None or not math.isfinite(maximum_step_m) or maximum_step_m<.2:
        return goal
    grid, footprint_overlay = _overlay_current_physical_footprint(
        grid, current_position, current_yaw_deg
    )
    frontier_step_m = min(
        AUTONOMOUS_STRATEGIC_GOAL_MAX_M, float(maximum_step_m)
    )
    intent_source = str(
        (intent.get("provenance") or {}).get("source")
        or intent.get("source")
        or "frontier"
    ).lower()
    keep_frontier_standoff = intent_source not in {
        "junction",
        "semantic_region",
        "known_region",
    }

    def _distance_limit(endpoint):
        limit = frontier_step_m
        if keep_frontier_standoff and current_position is not None:
            try:
                boundary_distance = math.dist(
                    tuple(float(v) for v in current_position),
                    tuple(float(v) for v in endpoint),
                )
                limit = min(
                    limit,
                    max(0.2, boundary_distance - AUTONOMOUS_FRONTIER_STANDOFF_M),
                )
            except (TypeError, ValueError):
                pass
        return limit
    # A frontier can be discovered before the semantic/topological route
    # cache has produced a route hint.  Do not pass that distant boundary
    # point straight to NavFn: it commonly lies in unknown space and makes
    # every cycle fail before a trial is even reserved.  A checked local
    # detour is still conservative here: it uses only observed free cells,
    # advances toward the frontier, stays inside the heading/step envelope,
    # and the backend repeats the authoritative Nav2/corridor checks before
    # any arm or motion action.  If any of that evidence is missing, keep the
    # original goal so the backend rejects it fail-closed.
    if not route_plan:
        endpoint=intent.get('preferred_position') or goal.position
        detour=_safe_local_detour(
            goal, endpoint, grid, maximum_step_m=_distance_limit(endpoint),
            current_position=current_position, current_yaw_deg=current_yaw_deg,
            maximum_turn_deg=maximum_turn_deg,
            current_footprint_overlay=footprint_overlay,
        )
        return detour if detour is not None else goal
    points=route_plan.get('waypoints') or []
    if not points:
        endpoint=intent.get('preferred_position') or goal.position
        detour=_safe_local_detour(
            goal, endpoint, grid, maximum_step_m=_distance_limit(endpoint),
            current_position=current_position, current_yaw_deg=current_yaw_deg,
            maximum_turn_deg=maximum_turn_deg,
            current_footprint_overlay=footprint_overlay,
        )
        return detour if detour is not None else goal
    # A map route is a hint, not an executed path. Reject malformed hints.
    try:
        points=[(float(p[0]),float(p[1])) for p in points]
        if any(not all(math.isfinite(v) for v in p) for p in points):return goal
    except (TypeError,ValueError,IndexError):
        return goal
    endpoint=intent.get('preferred_position') or goal.position
    source='observed_map_route_prefix'
    try:
        if len(endpoint)!=2 or not all(math.isfinite(float(v)) for v in endpoint):return goal
        endpoint=tuple(float(v) for v in endpoint)
        if len(points)==1 and route_plan.get('planner_source')=='place_graph_topological':
            # A same-place topological route contains no geometric path. Use
            # a freshly located start only to propose observed straight
            # prefixes; never call the topological hint a collision proof.
            if (current_position is None or len(current_position)!=2
                    or not all(math.isfinite(float(v)) for v in current_position)):
                return goal
            start=tuple(float(v) for v in current_position)
            if math.dist(start,points[0])>.15:return goal
            distance=math.dist(start,endpoint)
            if not 0<distance<=30:return goal
            steps=max(1,math.ceil(distance/.05))
            points=[tuple(a+(b-a)*i/steps for a,b in zip(start,endpoint)) for i in range(steps+1)]
            source='observed_map_direct_prefix'
        if len(points)<2:return goal
        if math.dist(points[-1],endpoint)>.15:return goal
    except (TypeError,ValueError):
        return goal  # A different endpoint must not inherit this route.
    length=sum(math.dist(a,b) for a,b in zip(points,points[1:]))
    strategic_limit = min(frontier_step_m, _distance_limit(endpoint))
    if keep_frontier_standoff:
        strategic_limit = min(
            strategic_limit,
            max(0.2, length - AUTONOMOUS_FRONTIER_STANDOFF_M),
        )
    if (length <= strategic_limit
            and classify_path_clearance(measure_path_clearance(points,grid)).decision == DECISION_OK):
        return goal
    traveled=0.;prefixes=[]
    for i,(a,b) in enumerate(zip(points,points[1:]),1):
        traveled+=math.dist(a,b)
        if .2<=traveled<=strategic_limit and math.dist(b,goal.position)>.10:
            prefixes.append((i,traveled))
    # Check longest first, so a clear large field retains long goals.
    for i,distance in reversed(prefixes):
        path=points[:i+1]
        clear, decision = _clear_with_prefix_reserve(path, grid)
        if not clear:
            continue
        a,b=path[-2:];yaw=math.degrees(math.atan2(b[1]-a[1],b[0]-a[0]))
        return replace(goal,position=b,yaw=yaw,provenance={**goal.provenance,
            'nearer_frontier_goal':dict(original_position=list(goal.position),
                original_yaw=goal.yaw,selected_path_length_m=distance,
                source=source,selected_min_side_gap_m=decision.min_side_gap_m,
                minimum_prefilter_side_gap_m=LOCAL_PREFIX_MIN_SIDE_GAP_M,
                current_footprint_overlay=footprint_overlay,
                motion_authorized=False)})
    detour = _safe_local_detour(
        goal, endpoint, grid, maximum_step_m=strategic_limit,
        current_position=current_position, current_yaw_deg=current_yaw_deg,
        maximum_turn_deg=maximum_turn_deg,
        current_footprint_overlay=footprint_overlay,
    )
    if detour is not None:
        return detour
    return goal
