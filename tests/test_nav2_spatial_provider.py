import json
import math
from types import SimpleNamespace
import time

from app.spatial.nav2_spatial_provider import Nav2SpatialProvider
from app.spatial.models import SpatialFrameMismatch, SpatialPose
from app.slam.slam_bridge import SLAMBridge
import pytest


def map_message():
    return SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        info=SimpleNamespace(
            width=2, height=2, resolution=.05,
            origin=SimpleNamespace(position=SimpleNamespace(x=0., y=0.),
                                   orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)),
        ),
        data=[0, 0, 100, -1],
    )


def shifted_map_message():
    message = map_message()
    message.info.origin.position.x = .05
    return message


def test_queued_old_health_does_not_become_fresh_when_delivered_late():
    provider=Nav2SpatialProvider()
    provider._on_valid(SimpleNamespace(data=json.dumps({'valid':True,'wall_time':time.time()-10})))
    assert not provider._healthy()
    provider._on_valid(SimpleNamespace(data=json.dumps({'valid':True,'wall_time':time.time()})))
    assert provider._healthy()
    provider._on_valid(SimpleNamespace(data='invalid'))
    assert not provider._healthy()


def passive_sparse_health(**updates):
    checks = {name: True for name in Nav2SpatialProvider._PASSIVE_SPATIAL_REQUIRED_CHECKS}
    checks.update({'scan_returns': False, 'mapping_scan_returns': False})
    value = {
        'valid': False,
        'mode': 'mapping',
        'wall_time': time.time(),
        'blocking': ['scan_returns', 'mapping_scan_returns'],
        'checks': checks,
    }
    value.update(updates)
    return value


def test_passive_mapping_can_use_pose_when_only_sparse_returns_are_blocking():
    strict = Nav2SpatialProvider()
    message = SimpleNamespace(data=json.dumps(passive_sparse_health()))
    strict._on_valid(message)
    assert not strict._healthy()

    passive = Nav2SpatialProvider(allow_passive_mapping_sparse_scans=True)
    passive._on_valid(message)
    assert passive._healthy()
    assert passive.health()['localization_valid_basis'] == 'passive_mapping_sparse_scan'


def test_passive_sparse_exception_rejects_any_spatial_integrity_failure():
    provider = Nav2SpatialProvider(allow_passive_mapping_sparse_scans=True)
    status = passive_sparse_health(blocking=['scan_returns', 'map_current'])
    status['checks']['map_current'] = False
    provider._on_valid(SimpleNamespace(data=json.dumps(status)))
    assert not provider._healthy()


def test_passive_sparse_exception_rejects_stale_status():
    provider = Nav2SpatialProvider(allow_passive_mapping_sparse_scans=True)
    provider._on_valid(SimpleNamespace(data=json.dumps(
        passive_sparse_health(wall_time=time.time() - 1.)
    )))
    assert not provider._healthy()


def test_wheel_pose_is_never_injected_as_map_pose():
    provider=Nav2SpatialProvider()
    with pytest.raises(SpatialFrameMismatch):
        provider.set_pose(SpatialPose(1.,2.,frame_id='odom_fused'))
    provider.set_pose(SpatialPose(1.,2.,frame_id='map'))
    assert provider.get_pose() is None
    assert provider.get_frontiers()==[]


def test_camera_projection_uses_the_actual_2d_map_frame():
    provider=Nav2SpatialProvider()
    provider._map_bridge=SLAMBridge(map_frame='map',odom_frame='odom_fused',base_frame='base_link')
    provider._map_bridge.note_map_to_odom(10.,(2.,3.,0.))
    provider._map_bridge.note_odom_to_base(10.,(1.,0.,0.))
    provider.get_pose=lambda:SpatialPose(3.,3.,0.,frame_id='map')
    # The historical pose bridge alone has only nominal camera geometry.
    # It cannot certify a camera point without acquisition-time sensor TF.
    point=provider.camera_point_to_map((0.,0.,1.),10.)
    assert point is None
    assert provider._map_bridge.map_frame=='map'


def transform(x=0.,y=0.,z=0.):
    return SimpleNamespace(transform=SimpleNamespace(
        translation=SimpleNamespace(x=x,y=y,z=z),
        rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))


def test_camera_uses_real_extrinsic_and_current_map_correction():
    provider=Nav2SpatialProvider();provider._tf_buffer=object();provider._healthy=lambda:True
    calls=[]
    def lookup(stamp):
        calls.append(stamp)
        return transform(x=10.),transform(x=1.,z=.07)
    provider._camera_transforms=lookup
    assert provider.camera_point_to_map((0.,0.,1.),42.)==pytest.approx((11.,0.,1.07))
    assert calls==[42.] and provider._transform_source=='tf2'
    provider._camera_transforms=lambda stamp:(transform(x=20.),transform(x=1.,z=.07))
    assert provider.reproject_camera_point_to_map((0.,0.,1.),42.)==pytest.approx((21.,0.,1.07))


def test_missing_image_time_tf_does_not_fall_back_to_nominal_camera():
    provider=Nav2SpatialProvider();provider._tf_buffer=object();provider._healthy=lambda:True
    def missing(stamp):raise LookupError('image not in TF history')
    provider._camera_transforms=missing
    assert provider.camera_point_to_map((0.,0.,1.),42.) is None
    assert provider.camera_point_to_spatial((0.,0.,1.)) is None


def test_anchor_uses_acquisition_time_pose_for_stale_keyframe():
    provider = Nav2SpatialProvider()
    provider._on_keyframe_graph(SimpleNamespace(data=json.dumps({
        'schema_version': 'go2w_keyframe_graph_v1',
        'map_id': 'runtime-map', 'map_epoch': 'epoch-1', 'frame_id': 'map',
        'sensor_profile_id': 'sha256:abc', 'graph_revision': 2,
        'nodes': [{
            'anchor_id': 'keyframe_0', 'source_stamp': 1.,
            'corrected_pose': [0., 0., 0.],
        }],
    })))
    calls = []
    provider._base_pose_at = lambda stamp: calls.append(stamp) or (0.1, 0., 0.01)
    anchor = provider.anchor_map_point((1., 0., .2), 100.)
    assert calls == [100.]
    assert anchor['anchor_id'] == 'keyframe_0'
    assert anchor['anchor_association_method'] == 'acquisition_pose_gate'


def test_anchor_refuses_stale_keyframe_without_acquisition_time_tf():
    provider = Nav2SpatialProvider()
    provider._on_keyframe_graph(SimpleNamespace(data=json.dumps({
        'schema_version': 'go2w_keyframe_graph_v1',
        'map_id': 'runtime-map', 'map_epoch': 'epoch-1', 'frame_id': 'map',
        'sensor_profile_id': 'sha256:abc', 'graph_revision': 2,
        'nodes': [{
            'anchor_id': 'keyframe_0', 'source_stamp': 1.,
            'corrected_pose': [0., 0., 0.],
        }],
    })))
    provider._base_pose_at = lambda stamp: (_ for _ in ()).throw(LookupError())
    assert provider.anchor_map_point((1., 0., .2), 100.) is None


def test_periodic_map_update_preserves_keyframe_world_identity():
    provider = Nav2SpatialProvider()
    graph = json.dumps({
        'schema_version': 'go2w_keyframe_graph_v1',
        'map_id': 'runtime-map', 'map_epoch': 'epoch-1', 'frame_id': 'map',
        'sensor_profile_id': 'sha256:abc', 'graph_revision': 7,
        'nodes': [{
            'anchor_id': 'keyframe_0', 'source_stamp': 10.,
            'corrected_pose': [0., 0., 0.],
        }],
    })
    provider._on_keyframe_graph(SimpleNamespace(data=graph))
    provider._on_occupancy_grid(map_message())
    assert provider._map.provenance == {
        'frame_id': 'map', 'mapping_mode': 'navigation_2d',
        'generator': 'slam_toolbox_or_validated_map_server',
        'map_id': 'runtime-map', 'map_epoch': 'epoch-1',
        'graph_revision': 7, 'sensor_profile_id': 'sha256:abc',
    }

    # The next periodic grid must not fall back to an unrelated runtime epoch.
    provider._on_occupancy_grid(map_message())
    assert provider._map.provenance['map_epoch'] == 'epoch-1'
    assert provider._map.provenance['sensor_profile_id'] == 'sha256:abc'


def test_map_quality_is_world_aligned_and_exposed_in_health():
    provider = Nav2SpatialProvider()
    provider._on_keyframe_graph(SimpleNamespace(data=json.dumps({
        'schema_version': 'go2w_keyframe_graph_v1',
        'map_id': 'runtime-map', 'map_epoch': 'epoch-1', 'frame_id': 'map',
        'sensor_profile_id': 'sha256:abc', 'graph_revision': 2, 'nodes': [],
    })))
    provider._on_occupancy_grid(map_message())
    assert provider.health()['mapping_quality']['world_aligned_change'] == {
        'reason': 'first_map_for_epoch'
    }
    provider._on_occupancy_grid(shifted_map_message())
    quality = provider.health()['mapping_quality']
    assert quality['state'] == 'MEASURED'
    assert quality['map_epoch'] == 'epoch-1'
    assert quality['graph_revision'] == 2
    assert quality['counts']['observed_cells'] == 3
    assert quality['world_aligned_change']['comparable_cells'] > 0
