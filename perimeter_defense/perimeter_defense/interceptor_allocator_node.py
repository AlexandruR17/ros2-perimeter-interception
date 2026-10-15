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
he took over.

Multi-robot roles (M4, core/roles.py). Every period the robots that are not
holding or escorting a captured person get a role:
  ENGAGE  the behaviour above (INVESTIGATE / INTERCEPT / FOLLOW, and SEARCH
          for a LOST track: its predicted position search_lead s ahead);
  GUARD   wait on the shortest path from the most urgent threat to the
          objective, guard_distance before the objective;
  FLANK   wait just inside the breach the threat came in by (BLOCK when the
          threat is only an alert), to cut the retreat.
Predictive mode keeps one robot on GUARD while two robots are available,
assigns robots to the most urgent threats (time to the objective) with the
minimum total interception time, and gives the rest FLANK / BLOCK, with
hysteresis against role flapping. Naive mode sends every robot after the
threat closest to it. A guard or flanker that comes within
neutralize_distance of a hostile person captures him.

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
from perimeter_defense.core import roles
from perimeter_defense.core.interception import Interception, intercept_point, resample_along
from perimeter_defense.ros_utils import lookup_pose2d, spin_node, to_sec
from perimeter_defense_msgs.msg import (Custody, PredictedPathArray, RoleAssignment, Track,
                                        TrackArray)
from perimeter_defense_world.geometry import load_layout

XY = Tuple[float, float]
THREAT_STATES = (Track.STATE_ALERT, Track.STATE_TRACKED, Track.STATE_LOST)
MODES = ('predictive', 'naive')
ROLE_NAMES = {RoleAssignment.PATROL: 'PATROL', RoleAssignment.INTERCEPT: 'INTERCEPT',
              RoleAssignment.GUARD: 'GUARD', RoleAssignment.FLANK: 'FLANK',
              RoleAssignment.INVESTIGATE: 'INVESTIGATE', RoleAssignment.SEARCH: 'SEARCH',
              RoleAssignment.BLOCK: 'BLOCK', RoleAssignment.FOLLOW: 'FOLLOW',
              RoleAssignment.HOLD: 'HOLD', RoleAssignment.ESCORT: 'ESCORT'}


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
    task: str = ''                   # M4 role: ENGAGE, GUARD, FLANK, BLOCK ('' patrolling)
    ref_id: int = 0                  # threat a GUARD / FLANK / BLOCK robot is placed against
    eta: float = 0.0                 # s to the current goal


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
        self.declare_parameter('guard_distance', 4.0)       # m before the objective on the threat's path
        self.declare_parameter('search_lead', 2.0)          # s ahead on the prediction of a LOST track
        self.declare_parameter('role_switch_abs', 1.0)      # s, hysteresis of the role assignment
        self.declare_parameter('role_switch_rel', 0.15)
        self.declare_parameter('infeasible_penalty', 30.0)  # s added when a robot cannot intercept

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
        self.guard_d = float(self.get_parameter('guard_distance').value)
        self.search_lead = float(self.get_parameter('search_lead').value)
        self.role_params = roles.RoleParams(
            switch_abs=float(self.get_parameter('role_switch_abs').value),
            switch_rel=float(self.get_parameter('role_switch_rel').value),
            infeasible_penalty=float(self.get_parameter('infeasible_penalty').value))
        self.objective: XY = layout.objective_target
        self.objective_tree = None
        self.objective_field: Optional[np.ndarray] = None
        self.origins: Dict[int, XY] = {}      # first position of each track (where it came in)
        self.aliases: Dict[int, int] = {}     # alert id -> lidar track a robot took over for it
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
        if self.objective_tree is not None:
            self.objective_field = self.objective_tree[0].reshape(self.grid.rows, self.grid.cols)
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
        for i, t in self.tracks.items():
            self.origins.setdefault(i, (t.position.x, t.position.y))
        for i in [i for i in self.origins if i not in self.tracks]:
            del self.origins[i]
        self.aliases = {a: b for a, b in self.aliases.items() if a in self.tracks and b in self.tracks}

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

        self.assign_roles(threats)
        for r in self.robots.values():
            self.publish_role(r)
        self.publish_markers()

    # -- roles (M4) -----------------------------------------------------------
    def assign_roles(self, threats: List[Track]) -> None:
        """Give every available robot a role and drive it accordingly."""
        free = [r for r in self.robots.values()
                if r.pose and r.role not in (RoleAssignment.HOLD, RoleAssignment.ESCORT)]
        engaged = {r.track_id for r in free if r.task == roles.ENGAGE and r.track_id}
        # An alert whose intruder a robot already took over on the lidar is the same person.
        targets = [t for t in threats if (self.engageable(t) or t.id in engaged)
                   and self.aliases.get(t.id) not in engaged]
        infos = [roles.ThreatInfo(t.id, t.state != Track.STATE_ALERT, self.time_to_objective(t)) for t in targets]
        lead = min(targets, key=self.time_to_objective, default=None)
        guard_goal = self.guard_point(lead) if lead is not None else None
        by_id = {t.id: t for t in targets}
        table: Dict[Tuple[str, int], float] = {}

        def engage_time(name: str, tid: int) -> float:
            if (name, tid) not in table:
                table[(name, tid)] = self.engage_time(self.robots[name], by_id[tid])
            return table[(name, tid)]

        def guard_time(name: str) -> float:
            return self.driving_distance(self.robots[name], guard_goal) / self.planning_speed

        previous = {r.name: (r.task, r.track_id if r.task == roles.ENGAGE else 0) for r in free if r.task}
        plan = roles.allocate([r.name for r in free], infos, engage_time, guard_time,
                              self.mode, previous, self.role_params)

        busy = set(engaged)
        for r in free:
            task, tid = plan.get(r.name, ('', 0))
            if not task:
                if r.task:
                    self.release(r)
                continue
            if not r.task:
                r.patrol_pub.publish(Bool(data=False))
            if task != r.task or (task == roles.ENGAGE and tid != r.track_id):
                self.get_logger().warn(f'{r.name}: {task} T{tid or lead.id}')
                r.goal = None
            r.task = task
            if task == roles.ENGAGE:
                if tid != r.track_id:
                    busy.discard(r.track_id)
                r.track_id, r.ref_id = tid, 0
                busy.add(tid)
            else:
                busy.discard(r.track_id)
                r.track_id, r.ref_id, r.target_xy = 0, tid or lead.id, None
        for r in free:
            if r.task == roles.ENGAGE:
                self.engage(r, busy)
            elif r.task == roles.GUARD:
                self.guard(r, guard_goal)
            elif r.task in (roles.FLANK, roles.BLOCK):
                self.flank(r)

    def time_to_objective(self, t: Track) -> float:
        """Priority of a threat: time to walk to the objective, at least at assumed_speed."""
        p = (t.position.x, t.position.y)
        d = math.inf
        if self.grid is not None and self.objective_field is not None:
            d = self.grid.lookup(self.objective_field, p)
        if math.isinf(d):
            d = math.dist(p, self.objective)
        return d / max(math.hypot(t.velocity.x, t.velocity.y), self.assumed_speed)

    def engage_time(self, r: RobotState, t: Track) -> float:
        """Estimated time for a robot to intercept a threat, for the role assignment."""
        res = self.plan_intercept(r, t) if self.mode == 'predictive' else None
        if res is not None:
            return res.eta + (0.0 if res.feasible else self.role_params.infeasible_penalty)
        return self.driving_distance(r, (t.position.x, t.position.y)) / self.planning_speed

    def guard_point(self, t: Track) -> XY:
        """On the shortest path from the threat to the objective, guard_distance before the objective."""
        if self.grid is None or self.objective_tree is None:
            return self.clamp_inside(*self.objective)
        path = self.grid.path_to_goal(self.objective_tree, (t.position.x, t.position.y))
        if len(path) < 2:
            return self.clamp_inside(*self.objective)
        length = sum(math.dist(a, b) for a, b in zip(path, path[1:]))
        return self.clamp_inside(*roles.point_along(path, max(0.0, length - self.guard_d)))

    def engage(self, r: RobotState, busy: set) -> None:
        t = self.tracks.get(r.track_id)
        if t is None:
            return
        if t.state == Track.STATE_ALERT:
            alert_id = t.id
            t = self.link_alert(r, t, busy)
            if t.id != alert_id:
                self.aliases[alert_id] = t.id
        r.target_xy = (t.position.x, t.position.y)
        if self.try_neutralize(r, t):
            return
        if t.state == Track.STATE_ALERT:
            role = RoleAssignment.INVESTIGATE
        elif self.mode == 'naive':
            role = RoleAssignment.FOLLOW
        elif t.state == Track.STATE_LOST:
            role = RoleAssignment.SEARCH
        else:
            role = RoleAssignment.INTERCEPT
        self.set_role(r, role, t.id)
        if role == RoleAssignment.INTERCEPT:
            self.drive_to_intercept(r, t)
        elif role == RoleAssignment.INVESTIGATE and self.mode == 'predictive':
            self.investigate_predictive(r, t)
        elif role == RoleAssignment.SEARCH:
            self.search(r, t)
        else:
            self.drive_to_track(r, t)

    def guard(self, r: RobotState, goal: Optional[XY]) -> None:
        self.set_role(r, RoleAssignment.GUARD, r.ref_id)
        t = self.tracks.get(r.ref_id)
        if t is None or goal is None or self.capture_if_close(r, t):
            return
        self.hold_position(r, goal, t)

    def flank(self, r: RobotState) -> None:
        self.set_role(r, RoleAssignment.FLANK if r.task == roles.FLANK else RoleAssignment.BLOCK, r.ref_id)
        t = self.tracks.get(r.ref_id)
        if t is None or self.capture_if_close(r, t):
            return
        origin = self.origins.get(t.id, (t.position.x, t.position.y))
        self.hold_position(r, self.clamp_inside(*origin), t)

    def hold_position(self, r: RobotState, goal: XY, t: Track) -> None:
        """Drive to a waiting position, facing the threat."""
        r.eta = self.driving_distance(r, goal) / self.planning_speed
        self.send_goal(r, goal, math.atan2(t.position.y - goal[1], t.position.x - goal[0]))

    def capture_if_close(self, r: RobotState, t: Track) -> bool:
        """A guard or flanker captures a hostile person who comes within reach."""
        if t.classification != Track.CLASS_HOSTILE:
            return False
        r.track_id = t.id
        if self.try_neutralize(r, t):
            return True
        r.track_id = 0
        return False

    def search(self, r: RobotState, t: Track) -> None:
        """LOST track: go where the prediction puts him search_lead s from now."""
        future = self.paths.get(int(t.id))
        if future is None or not future[1]:
            self.drive_to_track(r, t)
            return
        dt, points = future
        age = max(0.0, self.now_s() - to_sec(t.last_update))
        i = min(len(points) - 1, int(round((age + self.search_lead) / max(dt, 1e-3))))
        goal = self.clamp_inside(*points[i])
        r.eta = self.driving_distance(r, goal) / self.planning_speed
        self.send_goal(r, goal, math.atan2(goal[1] - r.pose[1], goal[0] - r.pose[0]))

    def set_role(self, r: RobotState, role: int, tid: int) -> None:
        if role != r.role:
            r.role = role
            self.get_logger().warn(f'{r.name}: {ROLE_NAMES[role]} track T{tid}')

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
        r.role, r.hold_since, r.task, r.ref_id = RoleAssignment.HOLD, self.now_s(), '', 0
        return True

    def drive_to_intercept(self, r: RobotState, t: Track) -> None:
        """Predictive mode: go where the target will be when the robot gets there."""
        if math.hypot(r.pose[0] - t.position.x, r.pose[1] - t.position.y) < self.direct_d:
            self.drive_to_track(r, t)
            return
        self.go_intercept(r, t)

    def investigate_predictive(self, r: RobotState, t: Track) -> None:
        """Alert without lidar contact: cut off the assumed walk to the objective."""
        self.go_intercept(r, t)

    def target_points(self, t: Track) -> Optional[Tuple[float, List[XY]]]:
        """Future positions of a threat: its predicted path or, for an alert, the
        assumed walk from the alert to the objective since the time of the alert."""
        if t.state != Track.STATE_ALERT:
            return self.paths.get(int(t.id))
        if self.grid is None or self.objective_tree is None:
            return None
        age = max(0.0, self.now_s() - to_sec(t.last_update))
        route = self.grid.path_to_goal(self.objective_tree, (t.position.x, t.position.y))
        dt = 0.2
        points = resample_along(route, self.assumed_speed, dt, age, self.alert_horizon)
        return (dt, points) if points else None

    def plan_intercept(self, r: RobotState, t: Track) -> Optional[Interception]:
        future = self.target_points(t)
        if future is None or not future[1] or not r.pose:
            return None
        dt, points = future

        def cost(q: XY) -> float:
            # Driving distance plus the time to turn towards the point, as distance.
            turn = abs(math.remainder(math.atan2(q[1] - r.pose[1], q[0] - r.pose[0]) - r.pose[2], math.tau))
            return self.driving_distance(r, q) + turn / self.turn_rate * self.planning_speed

        return intercept_point(points, dt, cost, self.planning_speed, self.intercept_margin)

    def go_intercept(self, r: RobotState, t: Track) -> None:
        res = self.plan_intercept(r, t)
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
        r.eta = self.driving_distance(r, (gx, gy)) / self.planning_speed
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
        r.task, r.ref_id, r.eta = '', 0, 0.0
        r.speed_pub.publish(SpeedLimit(percentage=False, speed_limit=0.0))
        r.patrol_pub.publish(Bool(data=True))
        self.get_logger().info(f'{r.name}: back to PATROL')

    def publish_role(self, r: RobotState) -> None:
        msg = RoleAssignment()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        msg.robot, msg.role, msg.track_id = r.name, r.role, r.track_id or r.ref_id
        if r.goal:
            msg.goal.x, msg.goal.y = r.goal
        if r.intercept is not None:
            msg.eta = float(r.intercept.eta)
        elif r.role != RoleAssignment.PATROL:
            msg.eta = float(r.eta)
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
