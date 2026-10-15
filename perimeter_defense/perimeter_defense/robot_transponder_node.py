"""Robot transponder: every UGV broadcasts its own position (blue force tracking).

The lidars of the other robots see a UGV like any other obstacle that is not
on the map. The person detector removes the returns around the robot
positions it knows, but a fast robot, a scan latency or a localization error
can still leave a cluster that becomes a track. The robots therefore report
their own position, from their localization (TF map -> base_link), on
/fleet/transponders; the IFF associates these fixes with the tracks like the
soldiers' transponders, so a robot is recognised as FRIEND and never
intercepted.

The fixes travel over the radio link, so a jammed robot loses this
recognition (simulated network, milestone C).
"""

from __future__ import annotations

from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from perimeter_defense.ros_utils import lookup_pose2d, spin_node
from perimeter_defense_msgs.msg import Transponder


class RobotTransponderNode(Node):
    def __init__(self) -> None:
        super().__init__('robot_transponder')
        self.declare_parameter('robot', 'lup1')
        self.declare_parameter('callsign', '')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('period', 0.2)          # s
        self.declare_parameter('accuracy', 0.3)        # m, one sigma of the robot localization
        self.robot: str = self.get_parameter('robot').value
        self.callsign: str = self.get_parameter('callsign').value or callsign(self.robot)
        self.frame: str = self.get_parameter('global_frame').value
        self.accuracy = float(self.get_parameter('accuracy').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Transponder, '/fleet/transponders', 10)
        self.create_timer(float(self.get_parameter('period').value), self.tick)

    def tick(self) -> None:
        pose = lookup_pose2d(self.tf_buffer, self.frame, f'{self.robot}/base_link')
        if pose is None:
            return
        msg = Transponder(id=self.callsign, accuracy=self.accuracy)
        msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), self.frame
        msg.position.x, msg.position.y = pose[0], pose[1]
        self.pub.publish(msg)


def callsign(robot: str) -> str:
    """lup2 -> LUP-2"""
    head = robot.rstrip('0123456789')
    return f'{head.upper()}-{robot[len(head):]}' if head != robot else robot.upper()


def main(args=None) -> None:
    spin_node(RobotTransponderNode, args)


if __name__ == '__main__':
    main()
