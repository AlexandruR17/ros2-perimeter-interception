"""Interceptor allocation: assigns roles to the UGVs and sends them Nav2 goals.

M2 implements the 'follow' mode: the robot closest to the threat leaves its
patrol; it investigates a fence alert (ALERT track, coarse position) and
follows the track once the lidar confirms it. A robot that reaches the
person (within neutralize_distance) captures him and HOLDs there until a
soldier reports by radio that he took over (/perimeter/custody), then goes
back to patrol. The predictive and naive interception modes and the
multi-robot roles are added in M3/M4.

Engagement uses the IFF: a robot is sent to a lidar track only once the IFF
has found it HOSTILE (tracked for 2 s without a transponder), so own
soldiers walking around do not pull robots off their patrol. Fence and spot
report alerts are investigated at once.

Robot positions come from the robots' own localization (TF), threats from
/threats/tracks; no ground truth is used.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, UInt32
from tf2_ros import Buffer, TransformListener

from perimeter_defense.ros_utils import lookup_pose2d, spin_node
from perimeter_defense_msgs.msg import Custody, RoleAssignment, Track, TrackArray

XY = Tuple[float, float]
THREAT_STATES = (Track.STATE_ALERT, Track.STATE_TRACKED, Track.STATE_LOST)


@dataclass
class RobotState:
    name: str
    client: ActionClient
    patrol_pub: object
    speed_pub: object
    role: int = RoleAssignment.PATROL
    track_id: int = 0
    goal: Optional[XY] = None
    goal_time: float = -1e9
    goal_handle: object = None
    pose: Optional[Tuple[float, float, float]] = None
    hold_since: float = 0.0


class InterceptorAllocatorNode(Node):
    def __init__(self) -> None:
        super().__init__('interceptor_allocator')
        self.declare_parameter('robots', ['lup1'])
        self.declare_parameter('mode', 'follow')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('period', 0.3)
        self.declare_parameter('neutralize_distance', 1.0)
        self.declare_parameter('resend_distance', 0.7)
        self.declare_parameter('resend_period', 0.6)
        self.declare_parameter('standoff', 0.5)
        self.declare_parameter('max_speed', 2.0)
        self.declare_parameter('hold_timeout', 120.0)

        self.frame: str = self.get_parameter('global_frame').value
        self.neutralize_d = float(self.get_parameter('neutralize_distance').value)
        self.resend_d = float(self.get_parameter('resend_distance').value)
        self.resend_t = float(self.get_parameter('resend_period').value)
        self.standoff = float(self.get_parameter('standoff').value)
        self.hold_timeout = float(self.get_parameter('hold_timeout').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.robots: Dict[str, RobotState] = {}
        for r in self.get_parameter('robots').value:
            self.robots[r] = RobotState(
                name=r,
                client=ActionClient(self, NavigateToPose, f'/{r}/navigate_to_pose'),
                patrol_pub=self.create_publisher(Bool, f'/{r}/patrol_enable', 10),
                speed_pub=self.create_publisher(SpeedLimit, f'/{r}/speed_limit', 10))
        self.tracks: Dict[int, Track] = {}
        self.custody: Dict[int, int] = {}
        self.create_subscription(TrackArray, '/threats/tracks', self.on_tracks, 10)
        self.create_subscription(Custody, '/perimeter/custody', self.on_custody, 10)
        self.neutralize_pub = self.create_publisher(UInt32, '/threats/neutralize', 10)
        self.role_pub = self.create_publisher(RoleAssignment, '/fleet/roles', 10)
        self.create_timer(float(self.get_parameter('period').value), self.tick)
        self.get_logger().info(f'mode: {self.get_parameter("mode").value}')

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_tracks(self, msg: TrackArray) -> None:
        self.tracks = {int(t.id): t for t in msg.tracks}

    def on_custody(self, msg: Custody) -> None:
        self.custody[int(msg.track_id)] = int(msg.state)

    def threats(self) -> List[Track]:
        """Tracks a robot should keep after (not friendly, not captured)."""
        return [t for t in self.tracks.values()
                if t.state in THREAT_STATES and t.classification != Track.CLASS_FRIEND]

    @staticmethod
    def engageable(t: Track) -> bool:
        """A new robot is sent to an alert, or to a lidar track the IFF found hostile."""
        return t.state == Track.STATE_ALERT or t.classification == Track.CLASS_HOSTILE

    # -- main loop ----------------------------------------------------------
    def tick(self) -> None:
        for r in self.robots.values():
            r.pose = lookup_pose2d(self.tf_buffer, self.frame, f'{r.name}/base_link')
        threats = self.threats()
        now = self.now_s()

        for r in self.robots.values():
            if r.role == RoleAssignment.HOLD:
                self.check_hold(r, now)
            # Release robots whose target disappeared or turned out friendly.
            elif r.track_id and r.track_id not in {t.id for t in threats}:
                self.release(r)

        if threats:
            self.assign_follow([t for t in threats if self.engageable(t)])
        for r in self.robots.values():
            self.publish_role(r)

    def assign_follow(self, threats: List[Track]) -> None:
        """M2: the closest available robot investigates or follows each threat."""
        busy = {r.track_id for r in self.robots.values() if r.track_id}
        for t in threats:
            if t.id in busy:
                continue
            free = [r for r in self.robots.values() if r.track_id == 0 and r.pose]
            if not free:
                break
            r = min(free, key=lambda r: math.hypot(r.pose[0] - t.position.x,
                                                   r.pose[1] - t.position.y))
            r.track_id = t.id
            r.patrol_pub.publish(Bool(data=False))
        for r in self.robots.values():
            if r.track_id == 0 or r.role == RoleAssignment.HOLD:
                continue
            t = self.tracks.get(r.track_id)
            if t is None or not r.pose:
                continue
            role = RoleAssignment.INVESTIGATE if t.state == Track.STATE_ALERT else RoleAssignment.FOLLOW
            if role != r.role:
                r.role = role
                name = 'INVESTIGATE' if role == RoleAssignment.INVESTIGATE else 'FOLLOW'
                self.get_logger().warn(f'{r.name}: {name} track T{t.id}')
            self.drive_to_track(r, t)

    def drive_to_track(self, r: RobotState, t: Track) -> None:
        tx, ty = t.position.x, t.position.y
        dist = math.hypot(r.pose[0] - tx, r.pose[1] - ty)
        if dist <= self.neutralize_d and t.state == Track.STATE_TRACKED:
            self.neutralize_pub.publish(UInt32(data=int(t.id)))
            self.get_logger().warn(f'{r.name}: track T{t.id} NEUTRALIZED at {dist:.2f} m, '
                                   'holding until a soldier takes over')
            self.cancel_goal(r)
            r.role, r.hold_since = RoleAssignment.HOLD, self.now_s()
            return
        # Stop short of the person, who is an obstacle in the costmap.
        if dist > self.standoff:
            gx = tx + (r.pose[0] - tx) / dist * self.standoff
            gy = ty + (r.pose[1] - ty) / dist * self.standoff
        else:
            gx, gy = tx, ty
        self.send_goal(r, (gx, gy), math.atan2(ty - r.pose[1], tx - r.pose[0]))

    # -- Nav2 ---------------------------------------------------------------
    def send_goal(self, r: RobotState, goal: XY, yaw: float) -> None:
        now = self.now_s()
        if r.goal is not None:
            moved = math.hypot(goal[0] - r.goal[0], goal[1] - r.goal[1])
            if moved <= self.resend_d and now - r.goal_time < self.resend_t:
                return
        if not r.client.server_is_ready():
            return
        # Lift the patrol speed cap (0 means no limit).
        r.speed_pub.publish(SpeedLimit(percentage=False, speed_limit=0.0))
        msg = NavigateToPose.Goal()
        msg.pose = PoseStamped()
        msg.pose.header.frame_id = self.frame
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = goal
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        r.goal, r.goal_time = goal, now
        future = r.client.send_goal_async(msg)
        future.add_done_callback(lambda f, rr=r: setattr(rr, 'goal_handle', f.result()))

    def check_hold(self, r: RobotState, now: float) -> None:
        """Stay with the captured person until a soldier has him in custody."""
        custody = self.custody.get(r.track_id)
        if custody in (Custody.ESCORTING, Custody.DETAINED):
            self.get_logger().info(f'{r.name}: T{r.track_id} handed over to a soldier')
            self.release(r)
        elif r.track_id not in self.tracks or now - r.hold_since > self.hold_timeout:
            self.release(r)

    def cancel_goal(self, r: RobotState) -> None:
        if r.goal_handle is not None and getattr(r.goal_handle, 'status', 0) not in (
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            r.goal_handle.cancel_goal_async()
        r.goal, r.goal_handle = None, None

    def release(self, r: RobotState) -> None:
        self.cancel_goal(r)
        r.role, r.track_id = RoleAssignment.PATROL, 0
        r.patrol_pub.publish(Bool(data=True))
        self.get_logger().info(f'{r.name}: back to PATROL')

    def publish_role(self, r: RobotState) -> None:
        msg = RoleAssignment()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.robot, msg.role, msg.track_id = r.name, r.role, r.track_id
        if r.goal:
            msg.goal.x, msg.goal.y = r.goal
        self.role_pub.publish(msg)


def main(args=None) -> None:
    spin_node(InterceptorAllocatorNode, args)


if __name__ == '__main__':
    main()
