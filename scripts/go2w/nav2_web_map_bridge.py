#!/usr/bin/python3
"""Display /map as a labelled 2D occupancy layer in the existing map viewport."""
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from app.navigation.localization_health import map_statistics
from app.navigation.occupancy_values import int8_grid_view
from app.slam.keyframe_graph import KeyframeGraphStore
from app.spatial.slam_web_map_state import SlamWebMapState
from app.spatial.object_display_snapshot import meta_payload


class Nav2WebMapBridge(Node):
    def __init__(self, output, meta):
        super().__init__('go2w_nav2_web_map_bridge')
        self.output, self.meta = Path(output), Path(meta)
        self.state = SlamWebMapState(canonical_frame='map', permanent_source='/map', voxel_size_m=.05)
        self.keyframes = KeyframeGraphStore()
        self.grid = None
        self.grid_revision = 0
        self.grid_wall_time = 0.0
        self.health, self.health_at = {}, 0.
        self.last_valid_health = {}
        self.health_failed_at = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.create_subscription(String, '/go2w/nav/localization_status', self.on_health, 10)
        self.create_subscription(String, '/go2w/slam/keyframe_graph', self.on_graph, qos)
        self.create_timer(.5, self.write)

    def on_health(self, msg):
        now = time.monotonic()
        self.health, self.health_at = json.loads(msg.data), now
        if self.health.get('valid') is True:
            self.last_valid_health = self.health
            self.health_failed_at = None
        elif self.health_failed_at is None:
            self.health_failed_at = now

    def on_graph(self, msg):
        try:
            self.keyframes.update_json(msg.data)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning('invalid keyframe graph: ' + str(exc))

    def on_map(self, msg):
        info = msg.info
        statistics = map_statistics(info.width, info.height, info.resolution, msg.data)
        if msg.header.frame_id != 'map' or not statistics['valid']:
            return
        # Preserve origin orientation when rendering a saved map.
        q = info.origin.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        values = int8_grid_view(msg.data)
        if values is not None:
            cells = values.tolist()
            counts = {
                'unknown': int((values < 0).sum()),
                'free': int(((values >= 0) & (values < 50)).sum()),
                'occupied': int((values >= 50).sum()),
            }
        else:
            cells = [int(value) for value in msg.data]
            counts = {
                'unknown': sum(1 for value in msg.data if value < 0),
                'free': sum(1 for value in msg.data if 0 <= value < 50),
                'occupied': sum(1 for value in msg.data if value >= 50),
            }
        self.grid = {
            'width': int(info.width), 'height': int(info.height),
            'resolution': float(info.resolution),
            'origin': {
                'x': float(info.origin.position.x),
                'y': float(info.origin.position.y),
                'yaw': float(yaw),
            },
            # Preserve every ROS occupancy value: -1 unknown, 0 free, 100
            # occupied, including intermediate probabilities.
            'data': cells,
            'stamp': stamp,
            'counts': counts,
        }
        self.grid_revision += 1
        self.grid_wall_time = time.time()

    def write(self):
        now = time.monotonic()
        status_live = now-self.health_at <= .3
        reported_valid = self.health.get('valid') is True and status_live
        # The authoritative monitor runs at 10 Hz.  Do not turn one marginal
        # false tick into a flashing Web map; a continuous failure still
        # becomes visible after 0.75 s.  A stopped status topic is never
        # debounced and fails immediately via status_live.
        transient_failure = bool(
            status_live
            and self.last_valid_health
            and self.health_failed_at is not None
            and now-self.health_failed_at < .75
        )
        valid = reported_valid or transient_failure
        effective_health = (
            self.health if reported_valid
            else self.last_valid_health if transient_failure
            else self.health
        )
        pose = effective_health.get('evidence', {}).get('pose')
        if valid and pose:
            self.state.note_lio(pose['x'], pose['y'], pose['yaw'])
        payload = self.state.snapshot(now=now, generated_at=time.time())
        graph = self.keyframes.snapshot()
        if graph is not None:
            payload.update(map_id=graph.map_id, map_epoch=graph.map_epoch,
                           frame_id=graph.frame_id, map_revision=graph.graph_revision,
                           sensor_profile_id=graph.sensor_profile_id)
        else:
            # Never mint a bridge-local identity for a real navigation map.
            # The semantic provider consumes the keyframe graph too, so a
            # temporary random epoch here would split one map into two worlds.
            payload.update(map_id='', map_epoch='', frame_id='map')
        cell_count = len(self.grid['data']) if self.grid else 0
        resolution = float(self.grid['resolution']) if self.grid else 0.0
        extent = ([self.grid['width'] * resolution,
                   self.grid['height'] * resolution, 0.0] if self.grid else [0.0, 0.0, 0.0])
        payload.update(mapping_mode='navigation_2d', map_kind='occupancy_grid_2d',
                       grid=self.grid,
                       available=self.grid is not None, points=[], point_count=0,
                       web_display_points=cell_count, source_map_points=cell_count,
                       source='/map', permanent_source='/map',
                       map_revision=(graph.graph_revision if graph is not None else self.grid_revision),
                       ros_stamp=(self.grid.get('stamp', 0.0) if self.grid else 0.0),
                       map_updated_at=self.grid_wall_time, map_extent_m=extent,
                       mapping_health='HEALTHY' if valid else 'LOCALIZATION_UNAVAILABLE',
                       health_reason='' if valid else ', '.join(effective_health.get('blocking', ['等待二维定位'])),
                       localization_valid=valid, pose=pose if valid else None,
                       lio_pose_valid=False, motion_authorized=False, safety_authorized=False)
        for path, content in [(self.output, self.state.serialize(payload)),
                              (self.meta, json.dumps(meta_payload(payload), ensure_ascii=False))]:
            path.parent.mkdir(parents=True, exist_ok=True)
            temporary = path.with_suffix(path.suffix+'.tmp')
            temporary.write_text(content+'\n')
            os.replace(temporary, path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--meta-output', required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Nav2WebMapBridge(args.output, args.meta_output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
