"""Interceptor allocation: assigns roles to the UGVs and sends them Nav2 goals.

The robot closest to the threat (driving distance) leaves its patrol and
investigates a fence or spot report alert (ALERT track, coarse position).
Once the lidar confirms the track, two modes, switchable at runtime
(parameter 'mode' or topic /c2/mode, current mode on /fleet/mode):
  predictive  INTERCEPT: drive to the interception point, the first point of
              the target's 6 s prediction the robot reaches no later than the
              target (driving distance around buildings at planning_speed,
              plus intercept_margin and turning time); close to the target
              it goes straight for it. An alert without lidar contact is
              investigated the same way, assuming the intruder walks from the
              alert position to the objective along the shortest path since
              the time of the alert.
  naive       FOLLOW: drive to the last measured position of the target
              (no prediction), the baseline of the evaluation.
A robot that reaches the
person (within neutralize_distance) captures him and HOLDs him for a moment.
Fully robotic base (escort_by: robot, default): the robot then ESCORTs him
to the detention building at walking pace, waiting when he falls behind,
stops next to the door, reports the custody (/perimeter/custody) and goes
back to patrol once he is inside (the lidar no longer sees him). With
soldiers (escort_by: soldier) the robot HOLDs until a soldier reports that
he took over. The multi-robot roles (guard, flank) are added in M4.

Robots defend the inside of the base: people outside the fence are tracked
(situational awareness) but not engaged, a robot stops following a person
who leaves the base, and every goal is kept inside the fence, so robots
never drive out through a breach.

Engagement uses the IFF: a robot is sent to a lidar track only once the IFF
has found it HOSTILE (tracked for 2 s without a transponder), so own
personnel with transponders do not pull robots off their patrol. Fence and
spot report alerts are investigated at once, and a robot investigating an
alert takes over at once an unidentified lidar track that appears near the
alert or near itself (almost certainly the intruder; waiting 2 s for the IFF
would let him pass).

Robot positions come from the robots' own localization (TF), threats from
/threats/tracks; no ground truth is used.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import SetParametersResult
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, ColorRGBA, String, UInt32
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from perimeter_defense.core.geodesic import GeodesicGrid
from perimeter_defense.core.interception import Interception, intercept_point, resample_along
from perimeter_defense.ros_utils import lookup_pose2d, spin_node, to_sec
from perimeter_defense_msgs.msg import (Custody, PredictedPathArray, RoleAssignment, Track,
                                        TrackArray)
from perimeter_defense_world.geometry import load_layout

XY = Tuple[float, float]
THREAT_STATES = (Track.STATE_ALERT, Track.STATE_TRACKED, Track.STATE_LOST)
MODES = ('predictive', 'naive')


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
    intercept: Optional[Interception] = None
    target_xy: Optional[XY] = None   # last known position of the robot's target


class InterceptorAllocatorNode(Node):
    def __init__(self) -> None:
        super().__init__('interceptor_allocator')
        self.declare_parameter('robots', ['lup1'])
        self.declare_parameter('mode', 'predictive')         # predictive or naive
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
        self.declare_parameter('inside_margin', 0.6)        # m beyond the fence still counted inside
        self.declare_parameter('goal_clearance', 0.8)       # m, goals kept this far inside the fence
        self.declare_parameter('planning_speed', 1.6)       # m/s, robot average speed for the ETA
        self.declare_parameter('intercept_margin', 0.3)     # s the robot must be early at the point
        self.declare_parameter('direct_distance', 2.0)      # m, closer than this: go for the target
        self.declare_parameter('robot_radius', 0.35)        # m, obstacle growth for driving distances
        self.declare_parameter('turn_rate', 1.2)            # rad/s, time to turn towards a point
        self.declare_parameter('assumed_speed', 1.3)        # m/s of an intruder only known from an alert
        self.declare_parameter('alert_horizon', 15.0)       # s of the assumed walk considered
        self.declare_parameter('alert_link_radius', 8.0)    # m, unknown track near an alert is the intruder
        self.declare_parameter('robot_link_radius', 4.0)    # m, or near the robot investigating the alert

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
        self.fence_min, self.fence_max = layout.fence_min, layout.fence_max
        self.inside_margin = float(self.get_parameter('inside_margin').value)
        self.goal_clearance = float(self.get_parameter('goal_clearance').value)
        self.planning_speed = float(self.get_parameter('planning_speed').value)
        self.intercept_margin = float(self.get_parameter('intercept_margin').value)
        self.direct_d = float(self.get_parameter('direct_distance').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.turn_rate = float(self.get_parameter('turn_rate').value)
        self.assumed_speed = float(self.get_parameter('assumed_speed').value)
        self.alert_horizon = float(self.get_parameter('alert_horizon').value)
        self.alert_link = float(self.get_parameter('alert_link_radius').value)
        self.robot_link = float(self.get_parameter('robot_link_radius').value)
        self.objective: XY = layout.objective_target
        self.objective_tree = None
        self.mode = self.check_mode(self.get_parameter('mode').value)
        self.grid: Optional[GeodesicGrid] = None
        self.fields: Dict[str, np.ndarray] = {}           # driving distances from each robot
        self.paths: Dict[int, Tuple[float, List[XY]]] = {}

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
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, latched)
        self.create_subscription(PredictedPathArray, '/threats/predicted_paths', self.on_paths, 10)
        self.create_subscription(String, '/c2/mode', lambda m: self.set_mode(m.data, 'C2'), 10)
        self.mode_pub = self.create_publisher(String, '/fleet/mode', latched)
        self.marker_pub = self.create_publisher(MarkerArray, '/fleet/intercept_markers', 10)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.create_timer(float(self.get_parameter('period').value), self.tick)
        self.mode_pub.publish(String(data=self.mode))
        self.get_logger().info(f'mode: {self.mode}')

    # -- mode ---------------------------------------------------------------
    @staticmethod
    def check_mode(mode: str) -> str:
        mode = 'naive' if mode == 'follow' else mode     # M2 name of the naive mode
        if mode not in MODES:
            raise ValueError(f'mode must be one of {MODES}, not "{mode}"')
        return mode

    def set_mode(self, mode: str, source: str) -> bool:
        try:
            mode = self.check_mode(mode.strip().lower())
        except ValueError as e:
            self.get_logger().error(str(e))
            return False
        if mode != self.mode:
            self.mode = mode
            for r in self.robots.values():
                r.goal = None                              # resend at once in the new mode
            self.get_logger().warn(f'mode -> {mode} ({source})')
        self.mode_pub.publish(String(data=self.mode))
        return True

    def on_parameters(self, params) -> SetParametersResult:
        for p in params:
            if p.name == 'mode' and not self.set_mode(str(p.value), 'parameter'):
                return SetParametersResult(successful=False, reason='mode: predictive or naive')
        return SetParametersResult(successful=True)

    # -- inputs -------------------------------------------------------------
    def on_map(self, msg: OccupancyGrid) -> None:
        info = msg.info
        occupied = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width) > 50
        c = self.goal_clearance - self.robot_radius
        self.grid = GeodesicGrid(occupied, info.resolution, (info.origin.position.x, info.origin.position.y),
                                 inflation=self.robot_radius,
                                 bounds=((self.fence_min[0] + c, self.fence_min[1] + c),
                                         (self.fence_max[0] - c, self.fence_max[1] - c)))
        self.objective_tree = self.grid.path_tree(self.objective)
        self.get_logger().info(f'driving distance grid {self.grid.cols}x{self.grid.rows} '
                               f'at {self.grid.res:.2f} m')

    def on_paths(self, msg: PredictedPathArray) -> None:
        self.paths = {int(p.track_id): (float(p.dt), [(q.x, q.y) for q in p.points]) for p in msg.paths}

    def driving_distance(self, r: RobotState, p: XY) -> float:
        field = self.fields.get(r.name)
        if self.grid is None or field is None:
            return math.dist(r.pose[:2], p)
        return self.grid.lookup(field, p)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_tracks(self, msg: TrackArray) -> None:
        self.tracks = {int(t.id): t for t in msg.tracks}

    def on_custody(self, msg: Custody) -> None:
        self.custody[int(msg.track_id)] = int(msg.state)

    def inside(self, x: float, y: float) -> bool:
        m = self.inside_margin
        return (self.fence_min[0] - m <= x <= self.fence_max[0] + m
                and self.fence_min[1] - m <= y <= self.fence_max[1] + m)

    def threats(self) -> List[Track]:
        """Tracks a robot should keep after: not friendly, not captured, inside the base."""
        return [t for t in self.tracks.values()
                if t.state in THREAT_STATES and t.classification != Track.CLASS_FRIEND
                and self.inside(t.position.x, t.position.y)]

    def clamp_inside(self, x: float, y: float) -> XY:
        c = self.goal_clearance
        return (min(max(x, self.fence_min[0] + c), self.fence_max[0] - c),
                min(max(y, self.fence_min[1] + c), self.fence_max[1] - c))

    @staticmethod
    def engageable(t: Track) -> bool:
        """A new robot is sent to an alert, or to a lidar track the IFF found hostile."""
        return t.state == Track.STATE_ALERT or t.classification == Track.CLASS_HOSTILE

    # -- main loop ----------------------------------------------------------
    def tick(self) -> None:
        for r in self.robots.values():
            r.pose = lookup_pose2d(self.tf_buffer, self.frame, f'{r.name}/base_link')
            r.intercept = None
            if r.pose and self.grid is not None:
                field = self.grid.distances_from(r.pose[:2])
                if field is not None:
                    self.fields[r.name] = field
        threats = self.threats()
        now = self.now_s()

        for r in self.robots.values():
            if r.role == RoleAssignment.HOLD:
                self.check_hold(r, now)
            elif r.role == RoleAssignment.ESCORT:
                self.check_escort(r, now)
            # Release robots whose target disappeared or turned out friendly,
            # unless the track only changed id (an alert took over the lidar track).
            elif r.track_id and r.track_id not in {t.id for t in threats}:
                if not self.relink(r, threats):
                    self.release(r)

        if threats:
            self.assign_follow([t for t in threats if self.engageable(t)])
        for r in self.robots.values():
            self.publish_role(r)
        self.publish_markers()

    def assign_follow(self, threats: List[Track]) -> None:
        """The closest available robot (driving distance) takes each threat."""
        busy = {r.track_id for r in self.robots.values() if r.track_id}
        for t in threats:
            if t.id in busy:
                continue
            free = [r for r in self.robots.values() if r.track_id == 0 and r.pose]
            if not free:
                break
            r = min(free, key=lambda r: self.driving_distance(r, (t.position.x, t.position.y)))
            r.track_id = t.id
            r.patrol_pub.publish(Bool(data=False))
        for r in self.robots.values():
            if r.track_id == 0 or r.role in (RoleAssignment.HOLD, RoleAssignment.ESCORT):
                continue
            t = self.tracks.get(r.track_id)
            if t is None or not r.pose:
                continue
            if t.state == Track.STATE_ALERT:
                t = self.link_alert(r, t, busy)
            r.target_xy = (t.position.x, t.position.y)
            if self.try_neutralize(r, t):
                continue
            if t.state == Track.STATE_ALERT:
                role = RoleAssignment.INVESTIGATE
            elif self.mode == 'predictive':
                role = RoleAssignment.INTERCEPT
            else:
                role = RoleAssignment.FOLLOW
            if role != r.role:
                r.role = role
                names = {RoleAssignment.INVESTIGATE: 'INVESTIGATE', RoleAssignment.INTERCEPT: 'INTERCEPT',
                         RoleAssignment.FOLLOW: 'FOLLOW'}
                self.get_logger().warn(f'{r.name}: {names[role]} track T{t.id}')
            if role == RoleAssignment.INTERCEPT:
                self.drive_to_intercept(r, t)
            elif role == RoleAssignment.INVESTIGATE and self.mode == 'predictive':
                self.investigate_predictive(r, t)
            else:
                self.drive_to_track(r, t)

    def relink(self, r: RobotState, threats: List[Track]) -> bool:
        if r.target_xy is None or r.role in (RoleAssignment.HOLD, RoleAssignment.ESCORT):
            return False
        taken = {o.track_id for o in self.robots.values() if o is not r}
        near = [t for t in threats if t.id not in taken and t.state != Track.STATE_ALERT
                and math.dist((t.position.x, t.position.y), r.target_xy) <= 1.5]
        if not near:
            return False
        r.track_id = min(near, key=lambda t: math.dist((t.position.x, t.position.y), r.target_xy)).id
        return True

    def link_alert(self, r: RobotState, alert: Track, busy: set) -> Track:
        """An unidentified lidar track near the alert (or near the robot investigating
        it) is taken over at once, before the IFF has decided."""
        ax, ay = alert.position.x, alert.position.y
        cands = [t for t in self.tracks.values()
                 if t.state == Track.STATE_TRACKED and t.classification == Track.CLASS_UNKNOWN
                 and t.id not in busy and self.inside(t.position.x, t.position.y)
                 and (math.hypot(t.position.x - ax, t.position.y - ay) <= self.alert_link
                      or math.hypot(t.position.x - r.pose[0], t.position.y - r.pose[1]) <= self.robot_link)]
        if not cands:
            return alert
        t = min(cands, key=lambda c: math.hypot(c.position.x - r.pose[0], c.position.y - r.pose[1]))
        self.get_logger().warn(f'{r.name}: alert T{alert.id} -> unidentified lidar track T{t.id}')
        r.track_id = t.id
        busy.add(t.id)
        return t

    def try_neutralize(self, r: RobotState, t: Track) -> bool:
        """A robot within neutralize_distance of a confirmed target captures him."""
        dist = math.hypot(r.pose[0] - t.position.x, r.pose[1] - t.position.y)
        if dist > self.neutralize_d or t.state != Track.STATE_TRACKED:
            return False
        self.neutralize_pub.publish(UInt32(data=int(t.id)))
        self.get_logger().warn(f'{r.name}: track T{t.id} NEUTRALIZED at {dist:.2f} m')
        self.cancel_goal(r)
        r.role, r.hold_since = RoleAssignment.HOLD, self.now_s()
        return True

    def drive_to_intercept(self, r: RobotState, t: Track) -> None:
        """Predictive mode: go where the target will be when the robot gets there."""
        tx, ty = t.position.x, t.position.y
        path = self.paths.get(int(t.id))
        if path is None or math.hypot(r.pose[0] - tx, r.pose[1] - ty) < self.direct_d:
            self.drive_to_track(r, t)
            return
        dt, points = path
        self.go_intercept(r, t, points, dt)

    def investigate_predictive(self, r: RobotState, t: Track) -> None:
        """Alert without lidar contact: cut off the assumed walk to the objective."""
        if self.grid is None or self.objective_tree is None:
            self.drive_to_track(r, t)
            return
        age = max(0.0, self.now_s() - to_sec(t.last_update))
        route = self.grid.path_to_goal(self.objective_tree, (t.position.x, t.position.y))
        dt = 0.2
        points = resample_along(route, self.assumed_speed, dt, age, self.alert_horizon)
        if not points:
            self.drive_to_track(r, t)
            return
        self.go_intercept(r, t, points, dt)

    def go_intercept(self, r: RobotState, t: Track, points: List[XY], dt: float) -> None:
        def cost(q: XY) -> float:
            # Driving distance plus the time to turn towards the point, as distance.
            turn = abs(math.remainder(math.atan2(q[1] - r.pose[1], q[0] - r.pose[0]) - r.pose[2], math.tau))
            return self.driving_distance(r, q) + turn / self.turn_rate * self.planning_speed

        res = intercept_point(points, dt, cost, self.planning_speed, self.intercept_margin)
        if res is None:
            self.drive_to_track(r, t)
            return
        r.intercept = res
        gx, gy = self.clamp_inside(*res.point)
        self.send_goal(r, (gx, gy), math.atan2(t.position.y - gy, t.position.x - gx))

    def drive_to_track(self, r: RobotState, t: Track) -> None:
        """Naive mode (and alerts, and the last metres): go to the last measured position."""
        tx, ty = t.position.x, t.position.y
        dist = math.hypot(r.pose[0] - tx, r.pose[1] - ty)
        # Stop short of the person, who is an obstacle in the costmap.
        if dist > self.standoff:
            gx = tx + (r.pose[0] - tx) / dist * self.standoff
            gy = ty + (r.pose[1] - ty) / dist * self.standoff
        else:
            gx, gy = tx, ty
        gx, gy = self.clamp_inside(gx, gy)
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
        r.role, r.track_id, r.escort_phase, r.target_xy = RoleAssignment.PATROL, 0, '', None
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
        if r.intercept is not None:
            msg.eta = float(r.intercept.eta)
        self.role_pub.publish(msg)

    def publish_markers(self) -> None:
        """Interception points in RViz: cross, line from the robot, ETA vs target time."""
        out: List[Marker] = [Marker(action=Marker.DELETEALL)]
        out[0].header.frame_id = self.frame
        for i, r in enumerate(self.robots.values()):
            if r.intercept is None or not r.pose:
                continue
            (x, y), res = r.intercept.point, r.intercept
            color = ColorRGBA(r=0.3, g=0.95, b=1.0, a=1.0) if res.feasible else ColorRGBA(r=1.0, g=0.6, b=0.2, a=1.0)
            cross = self._marker('intercept_point', i, Marker.LINE_LIST, color)
            cross.scale.x = 0.12
            cross.points = [Point(x=x - 0.6, y=y - 0.6, z=0.3), Point(x=x + 0.6, y=y + 0.6, z=0.3),
                            Point(x=x - 0.6, y=y + 0.6, z=0.3), Point(x=x + 0.6, y=y - 0.6, z=0.3)]
            line = self._marker('intercept_line', i, Marker.LINE_STRIP, color)
            line.scale.x = 0.05
            line.points = [Point(x=r.pose[0], y=r.pose[1], z=0.3), Point(x=x, y=y, z=0.3)]
            label = self._marker('intercept_label', i, Marker.TEXT_VIEW_FACING, color)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y + 1.0, 0.5
            label.scale.z = 0.6
            label.text = f'INT {callsign(r.name)}  ETA {res.eta:.1f}s / T {res.t:.1f}s'
            out += [cross, line, label]
        self.marker_pub.publish(MarkerArray(markers=out))

    def _marker(self, ns: str, mid: int, mtype: int, color: ColorRGBA) -> Marker:
        m = Marker()
        m.header.frame_id = self.frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
        m.pose.orientation.w = 1.0
        m.color = color
        return m


def callsign(robot: str) -> str:
    """lup2 -> LUP-2"""
    head = robot.rstrip('0123456789')
    return f'{head.upper()}-{robot[len(head):]}' if head != robot else robot.upper()


def main(args=None) -> None:
    spin_node(InterceptorAllocatorNode, args)


if __name__ == '__main__':
    main()
