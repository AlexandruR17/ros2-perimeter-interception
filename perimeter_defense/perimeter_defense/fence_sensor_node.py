"""Fence sensor: raises an alert when something crosses a breach.

Each breach has a beam-break sensor (a narrow lidar fan along the gap). When
enough beams return shorter than the clear gap, something is crossing. The
sensor only tells which breach was crossed, so the alert position is the
breach centre with a coarse uncertainty (1.5 m by default).
"""

from __future__ import annotations

from typing import Dict

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from perimeter_defense.ros_utils import spin_node
from perimeter_defense_msgs.msg import Alert
from perimeter_defense_world.geometry import load_layout


class FenceSensorNode(Node):
    def __init__(self) -> None:
        super().__init__('fence_sensor')
        self.declare_parameter('zones_file', '')
        self.declare_parameter('uncertainty', 1.5)
        self.declare_parameter('min_blocked_beams', 2)
        self.declare_parameter('clearance', 0.4)      # m shorter than the clear gap counts as blocked
        self.declare_parameter('cooldown', 3.0)       # s between alerts of the same breach

        layout = load_layout(self.get_parameter('zones_file').value or None)
        self.uncertainty = float(self.get_parameter('uncertainty').value)
        self.min_blocked = int(self.get_parameter('min_blocked_beams').value)
        self.clearance = float(self.get_parameter('clearance').value)
        self.cooldown = float(self.get_parameter('cooldown').value)

        self.pub = self.create_publisher(Alert, '/perimeter/fence_alert', 10)
        self.last_alert: Dict[str, float] = {}
        self.alert_id = 0
        for br in layout.breaches:
            centre = br.center(layout.fence_min, layout.fence_max)
            gap = br.end - br.start - 0.15   # sensor sits 0.15 m inside the gap
            self.create_subscription(
                LaserScan, f'/perimeter/breach_{br.name}/scan',
                lambda msg, n=br.name, c=centre, g=gap: self.on_scan(msg, n, c, g),
                qos_profile_sensor_data)
            self.get_logger().info(f'breach sensor "{br.name}" at ({centre[0]:.1f}, {centre[1]:.1f})')

    def on_scan(self, msg: LaserScan, name: str, centre, gap: float) -> None:
        blocked = sum(1 for r in msg.ranges if msg.range_min < r < gap - self.clearance)
        if blocked < self.min_blocked:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_alert.get(name, -1e9) < self.cooldown:
            return
        self.last_alert[name] = now
        self.alert_id += 1
        alert = Alert()
        alert.header.stamp = msg.header.stamp
        alert.header.frame_id = 'map'
        alert.id = self.alert_id
        alert.type = Alert.TYPE_FENCE_BREACH
        alert.severity = Alert.SEVERITY_WARNING
        alert.source = f'fence_sensor/{name}'
        alert.position.x, alert.position.y = float(centre[0]), float(centre[1])
        alert.uncertainty = self.uncertainty
        alert.text = f'Breach {name.upper()}: perimeter crossed'
        self.pub.publish(alert)
        self.get_logger().warn(alert.text)


def main(args=None) -> None:
    spin_node(FenceSensorNode, args)


if __name__ == '__main__':
    main()
