"""Wheel odometry with realistic drift.

The Gazebo DiffDrive plugin integrates the wheel joints almost perfectly on
flat ground. This node re-integrates its increments with a systematic scale
error, a heading bias and random noise proportional to the motion (the same
model AMCL assumes), then publishes nav_msgs/Odometry and the odom->base TF.
AMCL corrects the resulting drift against the map.
"""

from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class OdomNoiseNode(Node):
    def __init__(self) -> None:
        super().__init__('odom_noise')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        # Random noise, as standard deviation per unit of motion.
        self.declare_parameter('alpha_rot_from_rot', 0.05)
        self.declare_parameter('alpha_rot_from_trans', 0.01)
        self.declare_parameter('alpha_trans_from_trans', 0.02)
        self.declare_parameter('alpha_trans_from_rot', 0.005)
        # Systematic errors, sampled once per run.
        self.declare_parameter('scale_error_std', 0.015)
        self.declare_parameter('yaw_bias_std', 0.004)  # rad per metre travelled
        self.declare_parameter('seed', -1)

        p = self.get_parameter
        self.odom_frame: str = p('odom_frame').value
        self.base_frame: str = p('base_frame').value
        self.publish_tf: bool = p('publish_tf').value
        self.a_rr: float = p('alpha_rot_from_rot').value
        self.a_rt: float = p('alpha_rot_from_trans').value
        self.a_tt: float = p('alpha_trans_from_trans').value
        self.a_tr: float = p('alpha_trans_from_rot').value
        seed: int = p('seed').value
        self.rng = np.random.default_rng(None if seed < 0 else seed)
        self.scale = 1.0 + self.rng.normal(0.0, p('scale_error_std').value)
        self.yaw_bias = self.rng.normal(0.0, p('yaw_bias_std').value)
        self.get_logger().info(
            f'odometry drift: scale={self.scale:.4f}, yaw bias={self.yaw_bias:.5f} rad/m')

        self.prev: Optional[tuple] = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.pub = self.create_publisher(Odometry, 'odom', 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, 'odom_wheel', self.on_wheel_odom, 20)

    def on_wheel_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        cur = (pose.position.x, pose.position.y, yaw_from_quat(pose.orientation))
        if self.prev is None:
            self.prev = cur
            return
        px, py, pyaw = self.prev
        self.prev = cur

        # Increment expressed in the previous body frame.
        dxw, dyw = cur[0] - px, cur[1] - py
        c, s = math.cos(pyaw), math.sin(pyaw)
        dx = c * dxw + s * dyw
        dy = -s * dxw + c * dyw
        dyaw = wrap_angle(cur[2] - pyaw)
        trans = math.hypot(dx, dy)

        trans_std = self.a_tt * trans + self.a_tr * abs(dyaw)
        rot_std = self.a_rr * abs(dyaw) + self.a_rt * trans
        n_trans = trans * self.scale + (self.rng.normal(0.0, trans_std) if trans_std > 0 else 0.0)
        n_rot = dyaw + self.yaw_bias * trans + (self.rng.normal(0.0, rot_std) if rot_std > 0 else 0.0)
        heading = math.atan2(dy, dx) if trans > 1e-6 else 0.0

        self.x += n_trans * math.cos(self.yaw + heading)
        self.y += n_trans * math.sin(self.yaw + heading)
        self.yaw = wrap_angle(self.yaw + n_rot)
        self.publish(msg)

    def publish(self, src: Odometry) -> None:
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        out = Odometry()
        out.header.stamp = src.header.stamp
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.twist.twist.linear.x = src.twist.twist.linear.x * self.scale
        out.twist.twist.angular.z = src.twist.twist.angular.z
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.01
        cov[35] = 0.02
        out.pose.covariance = cov
        out.twist.covariance = cov
        self.pub.publish(out)

        if self.publish_tf:
            t = TransformStamped()
            t.header = out.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomNoiseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
