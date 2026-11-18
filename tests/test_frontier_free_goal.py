"""A component centroid is an identity, not necessarily a navigable cell."""
import math

import pytest

from app.spatial.frontier_extractor import FrontierExtractor
from app.spatial.models import SpatialMapSnapshot, SpatialPose


@pytest.mark.parametrize('center', ['unknown', 'occupied'])
def test_ring_goal_is_on_observed_free_cell_not_its_centroid(center):
    free = [(x, y) for y in range(3, 18) for x in range(3, 18)
            if x in (3, 17) or y in (3, 17)]
    grid = SpatialMapSnapshot(revision=1, width=21, height=21, resolution_m=.1,
        origin=(-1.05, -1.05), free=free,
        occupied=[(10, 10)] if center == 'occupied' else [], provenance={'frame_id':'map'})
    robot = SpatialPose(-.5, -.5, frame_id='map')
    extractor = FrontierExtractor(min_component_size=1)
    found = extractor.extract(grid, robot)
    assert len(found) == 1
    candidate = found[0]
    old_centroid = candidate.provenance['identity_centroid']
    assert tuple(math.floor((v-o)/.1) for v,o in zip(old_centroid, grid.origin)) not in free
    cell = tuple(math.floor((v-o)/.1) for v,o in zip(candidate.position, grid.origin))
    assert cell in free
    assert candidate.frontier_id == extractor._quantized_frontier_id(old_centroid)
    assert candidate.distance_m == pytest.approx(math.dist(candidate.position, (-.5,-.5)), abs=.00051)
    assert not candidate.provenance['reachability_checked']
    assert extractor.extract(grid, robot)[0].position == candidate.position


def test_submillimeter_resolution_goal_stays_inside_free_cell():
    grid = SpatialMapSnapshot(revision=1, width=3, height=3, resolution_m=.0001,
                              free=[(1,1)], provenance={'frame_id':'map'})
    candidate = FrontierExtractor(min_component_size=1).extract(grid)[0]
    assert candidate.position == pytest.approx((.00015, .00015))
