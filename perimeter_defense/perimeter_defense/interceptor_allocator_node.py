"""Interceptor allocation: assigns roles to the UGVs and sends them Nav2 goals.

M2 implements the 'follow' mode: the robot closest to the threat leaves its
patrol; it investigates a fence alert (ALERT track, coarse position) and
follows the track once the lidar confirms it. A robot that reaches the
person (within neutralize_distance) captures him and HOLDs him for a moment.
Fully robotic base (escort_by: robot, default): the robot then ESCORTs him
to the detention building at walking pace, waiting when he falls behind,
stops next to the door, reports the custody (/perimeter/custody) and goes
back to patrol once he is inside (the lidar no longer sees him). With
soldiers (escort_by: soldier) the robot HOLDs until a soldier reports that
he took over. The predictive and naive interception modes and the
multi-robot roles are added in M3/M4.

Engagement uses the IFF: a robot is sent to a lidar track only once the IFF
has found it HOSTILE (tracked for 2 s without a transponder), so own
personnel with transponders do not pull robots off their patrol. Fence and
spot report alerts are investigated at once.

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
from perimeter_defense_world.geometry import load_layout

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
    escort_phase: str = ''          # lead, handover
    phase_since: float = 0.0


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
        self.declare_parameter('zones_file', '')
        self.declare_parameter('escort_by', 'robot')        # robot or soldier
        self.declare_parameter('hold_time', 2.0)            # s securing the person before the escort
        self.declare_parameter('escort_speed', 0.6)         # m/s, walking pace of the detainee
        self.declare_parameter('escort_wait_distance', 3.0)  # m, robot waits when he falls behind
        self.declare_parameter('handover_timeout', 20.0)    # s at the door before giving up

        self.frame: str = self.get_parameter('global_frame').value
        self.neutralize_d = float(self.get_parameter('neutralize_distance').value)
        self.resend_d = float(self.get_parameter('resend_distance').value)
        self.resend_t = float(self.get_parameter('resend_period').value)
        self.standoff = float(self.get_parameter('standoff').value)
        self.hold_timeout = float(self.get_parameter('hold_timeout').value)
        self.escort_by: str = self.get_parameter('escort_by').value
        self.hold_time = float(self.get_parameter('hold_time').value)
        self.escort_speed = float(self.get_parameter('escort_speed').value)
        self.escort_wait = float(self.get_parameter('escort_wait_distance').value)
        self.handover_timeout = float(self.get_parameter('handover_timeout').value)
        layout = load_layout(self.get_parameter('zones_file').value or None)
        self.robot_stop: Optional[XY] = layout.detention.robot_stop if layout.detention else None

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
        self.custody_out: Dict[int, Custody] = {}     # reports of this node, repeated
        self.create_subscription(TrackArray, '/threats/tracks', self.on_tracks, 10)
        self.create_subscription(Custody, '/perimeter/custody', self.on_custody, 10)
        self.custody_pub = self.create_publisher(Custody, '/perimeter/custody', 10)
        self.create_timer(1.0, self.repeat_custody)
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
            elif r.role == RoleAssignment.ESCORT:
                self.check_escort(r, now)
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
            if r.track_id == 0 or r.role in (RoleAssignment.HOLD, RoleAssignment.ESCORT):
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
            self.get_logger().warn(f'{r.name}: track T{t.id} NEUTRALIZED at {dist:.2f} m')
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
    def send_goal(self, r: RobotState, goal: XY, yaw: float, speed: float = 0.0) -> None:
        now = self.now_s()
        if r.goal is not None:
            moved = math.hypot(goal[0] - r.goal[0], goal[1] - r.goal[1])
            if moved <= self.resend_d and now - r.goal_time < self.resend_t:
                return
        if not r.client.server_is_ready():
            return
        # Lift the patrol speed cap (0 means no limit) or set the escort pace.
        r.speed_pub.publish(SpeedLimit(percentage=False, speed_limit=float(speed)))
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
        """Secure the captured person, then escort him (or wait for a soldier)."""
        if r.track_id not in self.tracks or now - r.hold_since > self.hold_timeout:
            self.release(r)
        elif self.escort_by == 'soldier':
            if self.custody.get(r.track_id) in (Custody.ESCORTING, Custody.DETAINED):
                self.get_logger().info(f'{r.name}: T{r.track_id} handed over to a soldier')
                self.release(r)
        elif now - r.hold_since >= self.hold_time and self.robot_stop is not None and r.pose:
            r.role, r.escort_phase, r.phase_since = RoleAssignment.ESCORT, 'lead', now
            self.report_custody(r, Custody.ESCORTING)
            self.get_logger().warn(f'{r.name}: escorting T{r.track_id} to the detention building')

    def check_escort(self, r: RobotState, now: float) -> None:
        """Lead the captured person to the detention building at walking pace."""
        t = self.tracks.get(r.track_id)
        if not r.pose:
            return
        at_door = math.dist(r.pose[:2], self.robot_stop) < 0.7
        if r.escort_phase == 'lead':
            if at_door:
                self.cancel_goal(r)
                r.escort_phase, r.phase_since = 'handover', now
                return
            if t is None:
                # Lost the detainee on the way: wait for the lidar to find him again.
                self.cancel_goal(r)
                if now - r.phase_since > self.handover_timeout:
                    self.get_logger().warn(f'{r.name}: lost T{r.track_id} during the escort')
                    self.release(r)
                return
            r.phase_since = now
            if math.hypot(t.position.x - r.pose[0], t.position.y - r.pose[1]) > self.escort_wait:
                self.cancel_goal(r)        # he fell behind: wait for him
                return
            sx, sy = self.robot_stop
            self.send_goal(r, self.robot_stop, math.atan2(sy - r.pose[1], sx - r.pose[0]),
                           speed=self.escort_speed)
            return
        # Handover at the door: done when he is inside (no longer seen).
        if t is None or t.state == Track.STATE_LOST or now - r.phase_since > self.handover_timeout:
            self.report_custody(r, Custody.DETAINED)
            self.get_logger().warn(f'{r.name}: T{r.track_id} detained')
            self.release(r)

    def report_custody(self, r: RobotState, state: int) -> None:
        c = Custody(track_id=r.track_id, escort=callsign(r.name), state=state)
        c.header.stamp, c.header.frame_id = self.get_clock().now().to_msg(), self.frame
        if r.pose:
            c.position.x, c.position.y = r.pose[0], r.pose[1]
        self.custody_out[r.track_id] = c
        self.custody_pub.publish(c)

    def repeat_custody(self) -> None:
        # Repeated so late listeners (status board, dashboard) know the state.
        for c in self.custody_out.values():
            self.custody_pub.publish(c)

    def cancel_goal(self, r: RobotState) -> None:
        if r.goal_handle is not None and getattr(r.goal_handle, 'status', 0) not in (
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED):
            r.goal_handle.cancel_goal_async()
        r.goal, r.goal_handle = None, None

    def release(self, r: RobotState) -> None:
        self.cancel_goal(r)
        r.role, r.track_id, r.escort_phase = RoleAssignment.PATROL, 0, ''
        r.speed_pub.publish(SpeedLimit(percentage=False, speed_limit=0.0))
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


def callsign(robot: str) -> str:
    """lup2 -> LUP-2"""
    head = robot.rstrip('0123456789')
    return f'{head.upper()}-{robot[len(head):]}' if head != robot else robot.upper()


def main(args=None) -> None:
    spin_node(InterceptorAllocatorNode, args)


if __name__ == '__main__':
    main()
