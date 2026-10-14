"""Person detector: lidar clusters that are not on the static map, fused across robots.

For each robot scan: transform the returns to the map frame with the robot's
own localization (TF), drop returns explained by the static map, returns of
own robots and returns beyond the detection radius, cluster the rest and keep
person-sized clusters. A scan whose returns do not line up with the static
map (the robot's localization is wrong) is not used. Detections from all robots are fused in the map frame
and published at a fixed rate.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

from perimeter_defense.core.clustering import (DetectorParams, detect_people, fuse_detections,
                                               map_consistency, scan_to_points, transform_points)
from perimeter_defense.core.static_map import StaticMap
from perimeter_defense.ros_utils import lookup_pose2d, spin_node, to_sec
from perimeter_defense_msgs.msg import Detection, DetectionArray


class PersonDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('person_detector')
        self.declare_parameter('robots', ['lup1'])
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('max_scan_age', 0.3)
        self.declare_parameter('fusion_radius', 0.8)
        defaults = DetectorParams()
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))

        self.params = DetectorParams(**{n: self.get_parameter(n).value
                                        for n in defaults.__dataclass_fields__})
        self.robots: List[str] = list(self.get_parameter('robots').value)
        self.frame: str = self.get_parameter('global_frame').value
        self.max_age = float(self.get_parameter('max_scan_age').value)
        self.fusion_radius = float(self.get_parameter('fusion_radius').value)

        self.static_map: Optional[StaticMap] = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Latest detections per robot: (stamp, [(xy, width, range, points)])
        self.latest: Dict[str, Tuple[float, list]] = {}

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, self.get_parameter('map_topic').value,
                                 self.on_map, map_qos)
        for robot in self.robots:
            self.create_subscription(LaserScan, f'/{robot}/scan',
                                     lambda msg, r=robot: self.on_scan(msg, r),
                                     qos_profile_sensor_data)
        self.pub = self.create_publisher(DetectionArray, '/perimeter/detections', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.publish)

    def on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        self.static_map = StaticMap(msg.data, info.width, info.height, info.resolution,
                                    (info.origin.position.x, info.origin.position.y))
        self.get_logger().info(f'static map {info.width}x{info.height} received')

    def robot_positions(self, exclude: str) -> List[Tuple[float, float]]:
        out = []
        for r in self.robots:
            if r == exclude:
                continue
            pose = lookup_pose2d(self.tf_buffer, self.frame, f'{r}/base_link')
            if pose is not None:
                out.append((pose[0], pose[1]))
        return out

    def on_scan(self, msg: LaserScan, robot: str) -> None:
        if self.static_map is None:
            return
        sensor = lookup_pose2d(self.tf_buffer, self.frame, msg.header.frame_id, msg.header.stamp)
        if sensor is None:
            return
        pts, _ = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                max(msg.range_min, self.params.min_range), msg.range_max)
        pts_map = transform_points(pts, *sensor)
        consistency = map_consistency(pts_map, (sensor[0], sensor[1]), self.params,
                                      self.static_map.is_static)
        if consistency is not None and consistency < self.params.min_map_consistency:
            self.get_logger().warn(f'{robot}: scan does not match the map ({consistency:.0%}), '
                                   'localization lost? scan ignored', throttle_duration_sec=2.0)
            self.latest.pop(robot, None)
            return
        clusters = detect_people(pts_map, (sensor[0], sensor[1]), self.params,
                                 self.static_map.is_static,
                                 self.robot_positions(robot))
        self.latest[robot] = (to_sec(msg.header.stamp),
                              [(c.centroid, c.width, c.range, len(c.points)) for c in clusters])

    def publish(self) -> None:
        if not self.latest:
            return
        newest = max(stamp for stamp, _ in self.latest.values())
        raw = []
        for robot, (stamp, dets) in self.latest.items():
            if newest - stamp > self.max_age:
                continue
            raw += [(xy, width, robot) for xy, width, _, _ in dets]
        out = DetectionArray()
        out.header.frame_id = self.frame
        sec = int(newest)
        out.header.stamp.sec, out.header.stamp.nanosec = sec, int((newest - sec) * 1e9)
        for (x, y), width, sources in fuse_detections(raw, self.fusion_radius):
            d = Detection()
            d.position.x, d.position.y = float(x), float(y)
            d.width = float(width)
            d.source = ','.join(sources)
            out.detections.append(d)
        self.pub.publish(out)
        # Each scan is used once.
        self.latest.clear()


def main(args=None) -> None:
    spin_node(PersonDetectorNode, args)


if __name__ == '__main__':
    main()
