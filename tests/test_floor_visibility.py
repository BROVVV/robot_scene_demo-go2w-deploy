import numpy as np

from app.navigation.floor_visibility import FloorRayMemory, add_floor_visibility


def fuse(data,rays):
    return add_floor_visibility(data,width=len(data),height=1,resolution=.1,
        origin=np.array([0.,0.]),rays=rays)


def test_measured_floor_ray_cannot_clear_past_its_endpoint():
    out=fuse([-1]*20,[([.05,.05],[1.05,.05])])
    assert out[0]==0 and out[7]==0
    assert out[10:]==[-1]*10


def test_known_wall_blocks_ground_visibility_and_is_preserved():
    data=[-1]*20;data[5]=100
    out=fuse(data,[([.05,.05],[1.15,.05])])
    assert out[:5]==[0]*5 and out[5]==100 and out[6:]==[-1]*14


def test_no_rays_no_free_cells_and_nonfinite_rays_are_ignored():
    assert fuse([-1]*10,[])==[-1]*10
    assert fuse([-1]*10,[([.05,.05],[float('nan'),.05])])==[-1]*10


def test_existing_nonempty_cell_is_not_overwritten_and_outside_origin_is_rejected():
    data=[-1,20,0,-1,100,-1]
    out=fuse(data,[([.05,.05],[.55,.05])])
    assert out[1]==20 and out[2]==0 and out[4]==100
    assert fuse([-1]*10,[([-1.,.05],[.8,.05])])==[-1]*10


def test_distant_floor_hit_does_not_clear_a_long_projected_corridor():
    assert fuse([-1]*40,[([.05,.05],[3.05,.05])])==[-1]*40


def test_measured_floor_endpoints_use_world_origin_and_preserve_actual_hits():
    from app.navigation.floor_visibility import measured_floor_endpoints
    origin=np.array([11.,-6.,2.])
    delta=np.array([[.30,0.,-.4],[.70,0.,-.4],[0.,.6,-.4],[-.8,0.,-.4]])
    points=origin+delta
    out=measured_floor_endpoints(points,origin)
    assert len(out)==3
    assert all(any(np.array_equal(v,p) for p in points) for v in out)
    assert any(np.array_equal(v,points[1]) for v in out)
    assert not any(np.array_equal(v,points[0]) for v in out)


def test_cell_scale_ray_sampling_keeps_distinct_measured_nearby_directions():
    from app.navigation.floor_visibility import measured_floor_endpoints
    angles=np.radians([.2,2.5,4.8])
    points=np.c_[np.cos(angles),np.sin(angles),np.full(3,-.4)]
    assert len(measured_floor_endpoints(points,[0.,0.,0.]))==3
    assert len(measured_floor_endpoints(points,[0.,0.,0.],angular_bins=72))==1


def test_ray_selection_never_extends_range_or_accepts_nonfinite_points():
    from app.navigation.floor_visibility import measured_floor_endpoints
    points=np.array([[.05,0.,-.4],[1.3,0.,-.4],[.6,0.,float('nan')]])
    assert measured_floor_endpoints(points,[0.,0.,0.]).shape==(0,3)


def test_grazing_ray_visits_briefly_crossed_cell_and_never_skips_its_wall():
    # Cross y=.1 immediately before x=.5; fixed-distance samples can jump
    # straight from (4,0) to (5,1), missing both evidence and an actual wall.
    data=[-1]*36
    kwargs=dict(width=12,height=3,resolution=.1,origin=np.array([0.,0.]),
                rays=[([.05,.05],[1.15,.05+1.1*.05/.447])])
    out=add_floor_visibility(data,**kwargs)
    assert out[12+4]==0
    data[12+4]=100
    out=add_floor_visibility(data,**kwargs)
    assert out[12+4]==100
    assert out[12+5:12+10]==[-1]*5


def test_diagonal_corner_wall_also_blocks_clearing_beyond_it():
    data=[-1]*100;data[1]=100
    out=add_floor_visibility(data,width=10,height=10,resolution=.1,
        origin=np.array([0.,0.]),rays=[([.05,.05],[.75,.75])])
    assert out[1]==100
    assert out[11]==-1 and out[22]==-1


def test_grazing_occupied_cell_blocks_every_later_cell():
    data=[-1]*36;data[12+4]=100
    out=add_floor_visibility(data,width=12,height=3,resolution=.1,
        origin=np.array([0.,0.]),rays=[([.05,.05],[1.15,.05+1.1*.05/.447])])
    assert out[12+4]==100
    assert out[12+5]==-1


def test_floor_ray_memory_keeps_complementary_measured_angles_at_same_origin():
    memory = FloorRayMemory(origin_resolution_m=.1, angular_bins=8)
    memory.add([.01, .01, 0.], [[.71, .01, -.4]])
    memory.add([.02, .02, 0.], [[.02, .72, -.4]])
    rays = memory.rays()
    assert len(rays) == 2
    assert memory.origin_count == 1


def test_floor_ray_memory_keeps_farthest_actual_return_per_angle():
    memory = FloorRayMemory(origin_resolution_m=.1, angular_bins=8)
    memory.add([0., 0., 0.], [[.6, 0., -.4]])
    memory.add([0., 0., 0.], [[.4, 0., -.4]])
    [(start, endpoint)] = memory.rays()
    assert np.array_equal(start, [0., 0., 0.])
    assert np.array_equal(endpoint, [.6, 0., -.4])


def test_floor_ray_memory_is_bounded_by_spatial_origins():
    memory = FloorRayMemory(origin_resolution_m=.1, angular_bins=8, max_origins=2)
    for x in (.01, .11, .21):
        memory.add([x, 0., 0.], [[x + .5, 0., -.4]])
    assert memory.origin_count == 2
    starts = {round(float(start[0]), 2) for start, _ in memory.rays()}
    assert starts == {.11, .21}


def test_measured_ray_selection_matches_farthest_return_and_first_tie():
    from app.navigation.floor_visibility import measured_floor_endpoints
    rng = np.random.default_rng(48)
    origin = np.array([3., -2., .5])
    values = origin + rng.uniform(-1.5, 1.5, (5000, 3))
    # Equal XY distance but distinct height: the earlier return must win.
    values[:3] = origin + [[.8, 0., -.3], [.8, 0., -.8], [.4, 0., -.4]]
    values[3] = [float('nan'), 0., 0.]
    for count in (1, 8, 180, 720):
        selected = measured_floor_endpoints(values, origin, angular_bins=count)
        delta = values[:, :2] - origin[:2]
        distance = np.linalg.norm(delta, axis=1)
        keep = np.isfinite(values).all(axis=1) & (distance > .10) & (distance <= 1.25)
        points, distance, delta = values[keep], distance[keep], delta[keep]
        bins = np.floor((np.arctan2(delta[:, 1], delta[:, 0]) + np.pi)
                        / (2 * np.pi) * count).astype(int) % count
        expected = []
        for bucket in sorted(set(bins)):
            members = np.flatnonzero(bins == bucket)
            expected.append(points[members[np.argmax(distance[members])]])
        np.testing.assert_array_equal(selected, expected)
    ties = origin + np.array([[.8, 0., -.3], [.8, 0., -.8]])
    np.testing.assert_array_equal(measured_floor_endpoints(ties, origin), ties[:1])
