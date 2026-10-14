"""Tactical symbols for the UGV fleet in RViz.

The robots are Jackal-sized (0.5 m), too small to see on a 40 m map. Each
robot gets a friendly unit symbol (blue frame, north-up as on a C2 map), a
heading line, its call sign and the 10 m detection ring, positioned from
the robot's localized pose (TF map -> <robot>/base_link).
"""

from __future__ import annotations

import math
from typing import Dict, List

import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from perimeter_defense_viz.markers import base_marker, circle, color, line_strip, point, text


class FleetMarkersNode(Node):
    def __init__(self) -> None:
        super().__init__('fleet_markers')
        self.declare_parameter('robots', ['lup1', 'lup2', 'lup3'])
        self.declare_parameter('callsigns', ['LUP-1', 'LUP-2', 'LUP-3'])
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('rate', 5.0)
        self.declare_parameter('symbol_width', 3.0)
        self.declare_parameter('symbol_height', 2.0)
        self.declare_parameter('heading_length', 2.5)
        self.declare_parameter('sensor_range', 10.0)
        self.declare_parameter('friendly_color', [0.25, 0.6, 1.0, 1.0])

        robots = list(self.get_parameter('robots').value)
        callsigns = list(self.get_parameter('callsigns').value)
        self.callsign: Dict[str, str] = dict(zip(robots, callsigns))
        self.frame: str = self.get_parameter('global_frame').value
        self.rgba: List[float] = list(self.get_parameter('friendly_color').value)
        self.w = float(self.get_parameter('symbol_width').value)
        self.h = float(self.get_parameter('symbol_height').value)
        self.heading = float(self.get_parameter('heading_length').value)
        self.range = float(self.get_parameter('sensor_range').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(MarkerArray, '/fleet/markers', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.tick)

    def tick(self) -> None:
        markers: List[Marker] = []
        for i, (robot, callsign) in enumerate(self.callsign.items()):
            try:
                tf = self.tf_buffer.lookup_transform(self.frame, f'{robot}/base_link', Time())
            except TransformException:
                continue
            t = tf.transform
            x, y = t.translation.x, t.translation.y
            q = t.rotation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            markers += self.robot_markers(i, x, y, yaw, callsign)
        if markers:
            self.pub.publish(MarkerArray(markers=markers))

    def robot_markers(self, i: int, x: float, y: float, yaw: float, callsign: str) -> List[Marker]:
        rgba = self.rgba
        fill_rgba = [rgba[0], rgba[1], rgba[2], 0.55]
        hw, hh = self.w / 2.0, self.h / 2.0
        lifetime = Duration(seconds=1.0).to_msg()
        out: List[Marker] = []

        # Friendly frame (rectangle), filled and outlined.
        fill = base_marker('fleet_fill', i, Marker.CUBE, self.frame)
        fill.pose.position = point(x, y, 0.12)
        fill.scale.x, fill.scale.y, fill.scale.z = self.w, self.h, 0.02
        fill.color = color(fill_rgba)
        out.append(fill)
        corners = [(x - hw, y - hh), (x + hw, y - hh), (x + hw, y + hh), (x - hw, y + hh)]
        out.append(line_strip('fleet_frame', i, corners, rgba, 0.18, z=0.15, closed=True))

        # Unmanned ground vehicle mark inside the frame, then the call sign above it.
        out.append(text('fleet_type', i, x, y, 'UGV', [1.0, 1.0, 1.0, 1.0], 0.8, z=0.3))
        out.append(text('fleet_callsign', i, x, y + hh + 0.8, callsign, rgba, 1.1, z=0.3))

        # Heading line from the frame edge.
        hx, hy = math.cos(yaw), math.sin(yaw)
        start = (x + hx * min(hw, hh), y + hy * min(hw, hh))
        end = (x + hx * (min(hw, hh) + self.heading), y + hy * (min(hw, hh) + self.heading))
        out.append(line_strip('fleet_heading', i, [start, end], rgba, 0.18, z=0.15))

        # Detection ring.
        out.append(line_strip('fleet_sensor', i, circle(x, y, self.range, 120),
                              [rgba[0], rgba[1], rgba[2], 0.3], 0.04, z=0.1, closed=True))
        for m in out:
            m.lifetime = lifetime
        return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetMarkersNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
