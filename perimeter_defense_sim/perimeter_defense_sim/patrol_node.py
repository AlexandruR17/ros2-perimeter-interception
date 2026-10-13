"""Baseline patrol behaviour for one UGV (common base, not threat logic).

Loops over the closed patrol route of the robot (from zones.yaml) by sending
NavigateToPose goals, and caps the speed at the patrol speed through the Nav2
controller speed_limit topic. Patrol can be paused at runtime by publishing
False on 'patrol_enable'; the threat response layer uses this to take over
the robot and gives it back by publishing True.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from perimeter_defense_world.geometry import load_layout

Point = Tuple[float, float]


class PatrolNode(Node):
    def __init__(self) -> None:
        super().__init__('patrol')
        self.declare_parameter('route_name', '')
        self.declare_parameter('zones_file', '')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('patrol_speed', 0.8)
        self.declare_parameter('start_index', 1)
        self.declare_parameter('retry_delay', 2.0)
        self.declare_parameter('enabled', True)

        route_name: str = self.get_parameter('route_name').value or self.get_namespace().strip('/')
        zones_file: str = self.get_parameter('zones_file').value
        layout = load_layout(zones_file or None)
        if route_name not in layout.patrol_routes:
            raise RuntimeError(f'no patrol route "{route_name}" in zones.yaml')
        self.route: List[Point] = layout.patrol_routes[route_name]
        self.frame: str = self.get_parameter('global_frame').value
        self.speed: float = self.get_parameter('patrol_speed').value
        self.retry_delay: float = self.get_parameter('retry_delay').value
        self.index: int = int(self.get_parameter('start_index').value) % len(self.route)
        self.enabled: bool = self.get_parameter('enabled').value

        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.speed_pub = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self.create_subscription(Bool, 'patrol_enable', self.on_enable, 10)
        self.goal_handle = None
        self.busy = False
        self.retry_at: Optional[float] = None
        self.create_timer(0.5, self.tick)
        self.get_logger().info(f'patrol route "{route_name}" with {len(self.route)} waypoints')

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_enable(self, msg: Bool) -> None:
        if msg.data == self.enabled:
            return
        self.enabled = msg.data
        self.get_logger().info('patrol resumed' if msg.data else 'patrol paused')
        if not msg.data and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None
            self.busy = False

    def publish_speed_limit(self) -> None:
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = False
        msg.speed_limit = float(self.speed)
        self.speed_pub.publish(msg)

    def tick(self) -> None:
        if not self.enabled:
            return
        self.publish_speed_limit()
        if self.busy:
            return
        if self.retry_at is not None and self.now_s() < self.retry_at:
            return
        if not self.client.server_is_ready():
            self.client.wait_for_server(timeout_sec=0.0)
            return
        self.send_goal()

    def make_pose(self, i: int) -> PoseStamped:
        x, y = self.route[i]
        nx, ny = self.route[(i + 1) % len(self.route)]
        yaw = math.atan2(ny - y, nx - x)
        pose = PoseStamped()
        pose.header.frame_id = self.frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def send_goal(self) -> None:
        goal = NavigateToPose.Goal()
        goal.pose = self.make_pose(self.index)
        self.busy = True
        self.get_logger().info(
            f'patrol waypoint {self.index}: ({goal.pose.pose.position.x:.1f}, '
            f'{goal.pose.pose.position.y:.1f})')
        self.client.send_goal_async(goal).add_done_callback(self.on_goal_response)

    def on_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('patrol goal rejected')
            self.busy = False
            self.retry_at = self.now_s() + self.retry_delay
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future) -> None:
        status = future.result().status
        self.goal_handle = None
        self.busy = False
        if not self.enabled:
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.index = (self.index + 1) % len(self.route)
            self.retry_at = None
        else:
            # Skip a waypoint that cannot be reached instead of blocking the loop.
            self.get_logger().warn(f'patrol goal ended with status {status}, moving on')
            self.index = (self.index + 1) % len(self.route)
            self.retry_at = self.now_s() + self.retry_delay


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatrolNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
