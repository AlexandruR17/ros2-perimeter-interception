"""Multi-target tracker node.

Inputs:  /perimeter/detections (lidar person detections, all robots fused)
         /perimeter/fence_alert (coarse fence sensor alerts)
         /threats/neutralize    (track id reached by a robot)
         /threats/classification(track id + class, from the IFF node)
Outputs: /threats/tracks           TrackArray at 10 Hz
         /threats/predicted_paths  PredictedPathArray (constant velocity, 0-6 s)
         /threats/markers          RViz markers
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import numpy as np
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import UInt32
from visualization_msgs.msg import MarkerArray

from perimeter_defense.core.prediction import predict_path
from perimeter_defense.core.static_map import StaticMap
from perimeter_defense.core.tracker import MultiTargetTracker, Track as CoreTrack, TrackerParams
from perimeter_defense.ros_utils import spin_node, to_sec, to_stamp
from perimeter_defense.threat_markers import all_track_markers
from perimeter_defense_msgs.msg import (Alert, DetectionArray, PredictedPath,
                                        PredictedPathArray, Track, TrackArray)
from perimeter_defense_world.geometry import load_layout


class TrackerNode(Node):
    def __init__(self) -> None:
        super().__init__('tracker')
        self.declare_parameter('zones_file', '')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('prediction_horizon', 6.0)
        self.declare_parameter('prediction_step', 0.2)
        self.declare_parameter('obstacle_margin', 0.2)
        defaults = TrackerParams()
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        params = TrackerParams(**{n: self.get_parameter(n).value
                                  for n in defaults.__dataclass_fields__})

        self.layout = load_layout(self.get_parameter('zones_file').value or None)
        self.frame: str = self.get_parameter('global_frame').value
        self.horizon = float(self.get_parameter('prediction_horizon').value)
        self.step = float(self.get_parameter('prediction_step').value)
        self.margin = float(self.get_parameter('obstacle_margin').value)
        self.tracker = MultiTargetTracker(params)
        self.static_map: Optional[StaticMap] = None
        # Classification is owned by the IFF node; the tracker only stores it.
        self.classes: Dict[int, Tuple[int, str]] = {}

        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        self.create_subscription(DetectionArray, '/perimeter/detections', self.on_detections, 10)
        self.create_subscription(Alert, '/perimeter/fence_alert', self.on_fence_alert, 10)
        self.create_subscription(UInt32, '/threats/neutralize', self.on_neutralize, 10)
        self.create_subscription(TrackArray, '/threats/classification', self.on_classification, 10)
        self.track_pub = self.create_publisher(TrackArray, '/threats/tracks', 10)
        self.path_pub = self.create_publisher(PredictedPathArray, '/threats/predicted_paths', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/threats/markers', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.publish)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        self.static_map = StaticMap(msg.data, info.width, info.height, info.resolution,
                                    (info.origin.position.x, info.origin.position.y))

    def on_detections(self, msg: DetectionArray) -> None:
        dets = [(d.position.x, d.position.y) for d in msg.detections]
        self.tracker.update_detections(dets, to_sec(msg.header.stamp))

    def on_fence_alert(self, msg: Alert) -> None:
        if msg.type != Alert.TYPE_FENCE_BREACH:
            return
        tid = self.tracker.update_fence_alert((msg.position.x, msg.position.y),
                                              to_sec(msg.header.stamp))
        self.get_logger().info(f'fence alert from {msg.source} -> track T{tid}')

    def on_neutralize(self, msg: UInt32) -> None:
        if self.tracker.neutralize(int(msg.data), self.now_s()):
            self.get_logger().info(f'track T{msg.data} neutralized')

    def on_classification(self, msg: TrackArray) -> None:
        for t in msg.tracks:
            self.classes[int(t.id)] = (int(t.classification), t.transponder_id)

    def blocked(self, x: float, y: float) -> bool:
        if self.static_map is None:
            return False
        return bool(self.static_map.distance_to_obstacle(np.array([[x, y]]))[0] < self.margin)

    def to_msg(self, tr: CoreTrack) -> Track:
        m = Track()
        m.id = tr.id
        m.state = max(tr.state, 0)
        cls, tid = self.classes.get(tr.id, (Track.CLASS_UNKNOWN, ''))
        m.classification, m.transponder_id = cls, tid
        m.position.x, m.position.y = tr.kf.position
        m.velocity.x, m.velocity.y = tr.kf.velocity
        m.covariance = [float(v) for v in tr.kf.P.flatten()]
        m.first_seen = to_stamp(tr.first_seen)
        m.last_update = to_stamp(tr.last_update)
        m.hits = tr.hits
        return m

    def publish(self) -> None:
        now = self.now_s()
        self.tracker.predict_to(now)
        stamp = to_stamp(now)
        tracks = TrackArray()
        tracks.header.stamp, tracks.header.frame_id = stamp, self.frame
        paths = PredictedPathArray()
        paths.header = tracks.header
        path_points: Dict[int, List[Tuple[float, float]]] = {}
        for tr in self.tracker.published():
            msg = self.to_msg(tr)
            tracks.tracks.append(msg)
            if tr.state in (Track.STATE_TRACKED, Track.STATE_LOST):
                pts = predict_path(tr.kf.position, tr.kf.velocity, self.horizon, self.step,
                                   self.blocked, self.layout.objective_target,
                                   self.layout.objective_radius)
                path_points[tr.id] = pts
                paths.paths.append(PredictedPath(
                    track_id=tr.id, dt=self.step,
                    points=[Point(x=float(x), y=float(y), z=0.0) for x, y in pts]))
        self.track_pub.publish(tracks)
        self.path_pub.publish(paths)
        self.marker_pub.publish(MarkerArray(markers=all_track_markers(
            tracks.tracks, path_points, stamp, self.frame)))
        # Classifications of deleted tracks are no longer needed.
        live = {t.id for t in tracks.tracks}
        self.classes = {k: v for k, v in self.classes.items() if k in live}


def main(args=None) -> None:
    spin_node(TrackerNode, args)


if __name__ == '__main__':
    main()
