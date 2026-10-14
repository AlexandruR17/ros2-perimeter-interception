"""Exercise truth overlay for RViz (simulation side, common base).

Shows where the scenario actors really are: a person symbol (white for
intruders, light blue for own soldiers), the actor name and state, and a
dotted trail of the last seconds. This is exercise
control information for the operator and the demo; the defense algorithms
never use it (they only see the tracks built from sensor data).
"""

from __future__ import annotations

import math
from collections import deque
from typing import Deque, Dict, List, Tuple

import rclpy
from geometry_msgs.msg import Point, Pose
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray

STATE_TEXT = {'waiting': 'WAITING', 'walking': 'WALKING', 'running': 'RUNNING',
              'captured': 'CAPTURED', 'reached_target': 'AT TARGET', 'escorted': 'ESCORTED',
              'to_cell': 'ESCORTED', 'detained': 'DETAINED', 'patrol': 'PATROL',
              'observing': 'OBSERVING', 'dispatched': 'TO CAPTIVE', 'escorting': 'ESCORTING',
              'handover': 'HANDOVER', 'returning': 'RETURNING'}


class TruthMarkersNode(Node):
    def __init__(self) -> None:
        super().__init__('truth_markers')
        self.declare_parameter('actors', ['intruder1'])
        self.declare_parameter('friendly_actors', [''])
        self.declare_parameter('friendly_labels', [''])
        self.declare_parameter('friendly_color', [0.55, 0.80, 1.0, 0.95])
        self.declare_parameter('rate', 5.0)
        self.declare_parameter('trail_seconds', 20.0)
        self.declare_parameter('color', [1.0, 1.0, 1.0, 0.95])

        friends = [a for a in self.get_parameter('friendly_actors').value if a]
        labels = list(self.get_parameter('friendly_labels').value)
        self.friend_rgba = list(self.get_parameter('friendly_color').value)
        self.labels: Dict[str, str] = {a: (labels[i] if i < len(labels) and labels[i] else a.upper())
                                       for i, a in enumerate(friends)}
        self.actors: List[str] = list(self.get_parameter('actors').value) + friends
        self.trail_len = int(float(self.get_parameter('trail_seconds').value)
                             * float(self.get_parameter('rate').value))
        self.rgba = list(self.get_parameter('color').value)
        self.pose: Dict[str, Tuple[float, float, float]] = {}
        self.state: Dict[str, str] = {}
        self.trail: Dict[str, Deque[Tuple[float, float]]] = {
            a: deque(maxlen=self.trail_len) for a in self.actors}
        for a in self.actors:
            self.create_subscription(Pose, f'/sim/actors/{a}/pose',
                                     lambda m, a=a: self.on_pose(a, m), 10)
            self.create_subscription(String, f'/sim/actors/{a}/state',
                                     lambda m, a=a: self.state.__setitem__(a, m.data), 10)
        self.pub = self.create_publisher(MarkerArray, '/sim/truth_markers', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.tick)

    def on_pose(self, actor: str, msg: Pose) -> None:
        q = msg.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose[actor] = (msg.position.x, msg.position.y, yaw)

    def marker(self, ns: str, mid: int, mtype: int, friend: bool = False) -> Marker:
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        c = self.friend_rgba if friend else self.rgba
        m.color = ColorRGBA(r=c[0], g=c[1], b=c[2], a=c[3])
        m.lifetime.sec = 1
        return m

    def tick(self) -> None:
        out: List[Marker] = []
        for i, actor in enumerate(self.actors):
            if actor not in self.pose:
                continue
            x, y, yaw = self.pose[actor]
            self.trail[actor].append((x, y))
            friend = actor in self.labels
            mk = lambda ns, t: self.marker(ns, i, t, friend)   # noqa: E731

            ring = mk('truth_actor', Marker.LINE_STRIP)
            ring.scale.x = 0.08
            ring.points = [Point(x=x + 0.45 * math.cos(a), y=y + 0.45 * math.sin(a), z=0.2)
                           for a in [2 * math.pi * k / 24 for k in range(25)]]
            out.append(ring)

            body = mk('truth_body', Marker.CYLINDER)
            body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, 0.9
            body.scale.x = body.scale.y = 0.35
            body.scale.z = 1.8
            body.color.a = 0.5
            out.append(body)

            heading = mk('truth_heading', Marker.ARROW)
            heading.points = [Point(x=x, y=y, z=0.2),
                              Point(x=x + 0.9 * math.cos(yaw), y=y + 0.9 * math.sin(yaw), z=0.2)]
            heading.scale.x, heading.scale.y, heading.scale.z = 0.07, 0.2, 0.2
            out.append(heading)

            state = STATE_TEXT.get(self.state.get(actor, ''), '')
            label = mk('truth_label', Marker.TEXT_VIEW_FACING)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y - 1.0, 0.5
            label.scale.z = 0.6
            name = self.labels.get(actor, actor.upper())
            label.text = f'EX {name} {state}'.strip()
            out.append(label)

            if len(self.trail[actor]) > 1:
                trail = mk('truth_trail', Marker.POINTS)
                trail.scale.x = trail.scale.y = 0.1
                trail.color.a = 0.6
                trail.points = [Point(x=px, y=py, z=0.15) for px, py in self.trail[actor]]
                out.append(trail)
        if out:
            self.pub.publish(MarkerArray(markers=out))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TruthMarkersNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
