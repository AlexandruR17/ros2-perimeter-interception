"""Scripted intruder (simulation side, common base).

Walks from its spawn point outside the fence through a breach and along the
west or east route to the ammunition depot entrance, following an A* path on
the base map. It switches to running once the defense has a track on it
(within detect_radius of its true position) and stops when a robot gets
within capture_distance (surrender). At the target he spends dwell_time
(the objective is compromised), then runs back out through the breach he
came in by; outside the fence he has escaped (exfiltrated). A captured
intruder waits. When the UGV that captured him reports the escort
(/perimeter/custody, ESCORTING by LUP-n within 2.5 m of him) he walks behind
it and, once it stops next to the detention door, walks into the cell and
stays there (detained). In the scenario with soldiers the soldier who takes
him over sends his actor name on /sim/actors/<name>/escort and "cell" at the
door.

The exercise outcome (objective compromised, exfiltrated, captured,
detained) is announced on /sim/exercise/events for the exercise control
displays (RViz banner, dashboard umpire banner).

Uses the actor's true pose (/sim/actors/<name>/pose) and the robots' true
poses (/sim/ground_truth/<robot>/pose): this is the scenario script, not part
of the defense algorithms.
"""

from __future__ import annotations

import json
import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from perimeter_defense_msgs.msg import Custody, Track, TrackArray
from perimeter_defense_sim.path_planning import GridPlanner, plan_route
from perimeter_defense_world.geometry import load_layout

XY = Tuple[float, float]


class IntruderControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('intruder_controller')
        self.declare_parameter('actor', 'intruder1')
        self.declare_parameter('zones_file', '')
        self.declare_parameter('spawn', 'north_breach')        # key in intruder_spawns
        self.declare_parameter('route', 'west')                # west or east (north breach)
        self.declare_parameter('start_delay', 5.0)             # s of sim time
        self.declare_parameter('walk_speed', 1.3)
        self.declare_parameter('run_speed', 2.5)
        self.declare_parameter('run_when_detected', True)
        self.declare_parameter('detect_radius', 2.0)
        self.declare_parameter('capture_distance', 1.0)
        self.declare_parameter('target_radius', 0.9)
        self.declare_parameter('robots', ['lup1', 'lup2', 'lup3'])
        self.declare_parameter('lookahead', 0.6)
        self.declare_parameter('follow_distance', 1.2)    # m behind the escort
        self.declare_parameter('dwell_time', 4.0)         # s at the objective before leaving
        self.declare_parameter('exfiltrate', True)        # leave through the entry breach

        g = lambda n: self.get_parameter(n).value   # noqa: E731
        self.actor: str = g('actor')
        layout = load_layout(g('zones_file') or None)
        self.target: XY = layout.objective_target
        self.walk, self.run = float(g('walk_speed')), float(g('run_speed'))
        self.start_delay = float(g('start_delay'))
        self.run_when_detected = bool(g('run_when_detected'))
        self.detect_radius = float(g('detect_radius'))
        self.capture_d = float(g('capture_distance'))
        self.target_r = float(g('target_radius'))
        self.robots: List[str] = list(g('robots'))
        self.lookahead = float(g('lookahead'))
        self.follow_d = float(g('follow_distance'))
        self.cell: Optional[XY] = layout.detention.cell if layout.detention else None
        self.escort: str = ''
        self.escort_pose: Optional[XY] = None
        self.escort_robot: str = ''
        self.robot_stop: Optional[XY] = layout.detention.robot_stop if layout.detention else None
        self.dwell = float(g('dwell_time'))
        self.exfiltrate = bool(g('exfiltrate'))
        self.layout = layout
        self.at_target_since: Optional[float] = None

        spawn = layout.intruder_spawns[g('spawn')]
        self.route_wps = self.route_waypoints(layout, g('spawn'), g('route'))
        self.spawn: XY = spawn
        self.planner = GridPlanner(layout)
        self.path = plan_route(self.planner, [spawn] + self.route_wps + [self.target])
        self.seg = 0
        self.pose: Optional[XY] = None
        self.robot_poses: Dict[str, XY] = {}
        self.state = 'waiting'
        self.running = False
        self.t0: Optional[float] = None
        self.waiting_logged = False

        self.cmd_pub = self.create_publisher(Vector3, f'/actor/{self.actor}/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, f'/sim/actors/{self.actor}/state', 10)
        latched = QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.event_pub = self.create_publisher(String, '/sim/exercise/events', latched)
        self.create_subscription(Pose, f'/sim/actors/{self.actor}/pose', self.on_pose, 10)
        for robot in self.robots:
            self.create_subscription(Pose, f'/sim/ground_truth/{robot}/pose',
                                     lambda m, r=robot: self.on_robot_pose(r, m), 10)
        self.create_subscription(TrackArray, '/threats/tracks', self.on_tracks, 10)
        self.create_subscription(String, f'/sim/actors/{self.actor}/escort', self.on_escort, 10)
        self.create_subscription(Custody, '/perimeter/custody', self.on_custody, 10)
        self.create_timer(0.05, self.tick)
        length = sum(math.dist(a, b) for a, b in zip(self.path, self.path[1:]))
        self.get_logger().info(f'{self.actor}: route {g("route")} via {g("spawn")}, {length:.1f} m')

    @staticmethod
    def route_waypoints(layout, spawn: str, route: str) -> List[XY]:
        s = layout.intruder_spawns
        if spawn == 'north_breach':
            br = layout.breach_by_name('north')
            inside = (br.center(layout.fence_min, layout.fence_max)[0], layout.fence_max[1] - 2.0)
            return [inside, s['north_route_west' if route == 'west' else 'north_route_east']]
        if spawn == 'west_breach':
            br = layout.breach_by_name('west')
            return [(layout.fence_min[0] + 2.0, br.center(layout.fence_min, layout.fence_max)[1])]
        return []

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg: Pose) -> None:
        self.pose = (msg.position.x, msg.position.y)

    def on_robot_pose(self, robot: str, msg: Pose) -> None:
        self.robot_poses[robot] = (msg.position.x, msg.position.y)

    def on_tracks(self, msg: TrackArray) -> None:
        if not self.run_when_detected or self.running or self.pose is None:
            return
        for t in msg.tracks:
            if t.state == Track.STATE_TRACKED and math.dist(
                    self.pose, (t.position.x, t.position.y)) <= self.detect_radius:
                self.running = True
                self.get_logger().warn(f'{self.actor}: detected, running')

    def on_escort(self, msg: String) -> None:
        if self.state not in ('captured', 'escorted'):
            return
        if msg.data == 'cell':
            if self.cell is not None:
                self.set_state('to_cell')
            return
        if msg.data and msg.data != self.escort:
            self.escort = msg.data
            self.create_subscription(Pose, f'/sim/actors/{msg.data}/pose', self.on_escort_pose, 10)
            self.set_state('escorted')

    def on_custody(self, msg: Custody) -> None:
        """Escort by a UGV: LUP-2 -> lup2, the robot must be next to him."""
        if self.state != 'captured' or msg.state != Custody.ESCORTING or self.pose is None:
            return
        robot = msg.escort.replace('-', '').lower()
        p = self.robot_poses.get(robot)
        if p is not None and math.dist(p, self.pose) <= 2.5:
            self.escort_robot = robot
            self.set_state('escorted')

    def on_escort_pose(self, msg: Pose) -> None:
        self.escort_pose = (msg.position.x, msg.position.y)

    EVENTS = {'reached_target': 'OBJECTIVE_COMPROMISED', 'escaped': 'EXFILTRATED',
              'captured': 'CAPTURED', 'detained': 'DETAINED'}

    def set_state(self, state: str) -> None:
        if state != self.state:
            self.state = state
            self.get_logger().info(f'{self.actor}: {state}')
            if state in self.EVENTS:
                self.event_pub.publish(String(data=json.dumps({
                    't': round(self.now_s(), 2), 'actor': self.actor, 'event': self.EVENTS[state],
                    'x': round(self.pose[0], 2) if self.pose else None,
                    'y': round(self.pose[1], 2) if self.pose else None})))
        self.state_pub.publish(String(data=state))

    def send(self, vx: float, vy: float) -> None:
        self.cmd_pub.publish(Vector3(x=float(vx), y=float(vy), z=0.0))

    def tick(self) -> None:
        now = self.now_s()
        if self.t0 is None:
            if now <= 0.0:
                return
            self.t0 = now
        if self.pose is None or now - self.t0 < self.start_delay:
            self.send(0.0, 0.0)
            # Publish the state while waiting too, so a stalled start is visible.
            self.state_pub.publish(String(data='waiting'))
            if not self.waiting_logged and now - self.t0 > 2.0:
                self.waiting_logged = True
                left = max(0.0, self.start_delay - (now - self.t0))
                self.get_logger().info(
                    f'{self.actor}: waiting, actor pose {"ok" if self.pose else "MISSING"}, '
                    f'starts in {left:.1f} s of simulation time')
            return
        if self.state == 'escorted':
            if self.escort_robot:
                self.escort_pose = self.robot_poses.get(self.escort_robot)
                if (self.cell is not None and self.escort_pose is not None
                        and math.dist(self.escort_pose, self.robot_stop) < 1.0):
                    self.set_state('to_cell')
                    return
            self.follow_escort()
            self.set_state(self.state)
            return
        if self.state == 'to_cell':
            d = math.dist(self.pose, self.cell)
            if d < 0.3:
                self.set_state('detained')
                self.send(0.0, 0.0)
            else:
                self.send((self.cell[0] - self.pose[0]) / d * 0.8, (self.cell[1] - self.pose[1]) / d * 0.8)
                self.set_state(self.state)
            return
        if self.state in ('captured', 'detained', 'escaped'):
            self.send(0.0, 0.0)
            self.set_state(self.state)
            return
        if any(math.dist(self.pose, p) <= self.capture_d for p in self.robot_poses.values()):
            # Also after reaching the target or on the way out (he surrenders).
            if self.state in ('reached_target', 'exfiltrating'):
                self.get_logger().warn(f'{self.actor}: captured after reaching the objective')
            self.set_state('captured')
            self.send(0.0, 0.0)
            return
        if self.state in ('waiting', 'walking', 'running') and math.dist(self.pose, self.target) <= self.target_r:
            self.at_target_since = now
            self.get_logger().warn(f'{self.actor}: OBJECTIVE COMPROMISED')
            self.set_state('reached_target')
        if self.state == 'reached_target':
            self.send(0.0, 0.0)
            self.set_state(self.state)
            if self.exfiltrate and now - self.at_target_since >= self.dwell:
                # Back out through the breach he came in by, running.
                self.path = plan_route(self.planner, [self.pose] + self.route_wps[::-1] + [self.spawn])
                self.seg, self.running = 0, True
                self.set_state('exfiltrating')
            return
        if self.state == 'exfiltrating' and math.dist(self.pose, self.spawn) < 0.6:
            self.get_logger().warn(f'{self.actor}: ESCAPED through the breach')
            self.set_state('escaped')
            self.send(0.0, 0.0)
            return

        # Pure pursuit along the path.
        while self.seg < len(self.path) - 2 and math.dist(self.pose, self.path[self.seg + 1]) < self.lookahead:
            self.seg += 1
        goal = self.path[self.seg + 1]
        dx, dy = goal[0] - self.pose[0], goal[1] - self.pose[1]
        d = math.hypot(dx, dy) or 1e-6
        speed = self.run if self.running else self.walk
        self.send(dx / d * speed, dy / d * speed)
        if self.state != 'exfiltrating':
            self.set_state('running' if self.running else 'walking')
        else:
            self.set_state(self.state)

    def follow_escort(self) -> None:
        if self.escort_pose is None:
            self.send(0.0, 0.0)
            return
        dx, dy = self.escort_pose[0] - self.pose[0], self.escort_pose[1] - self.pose[1]
        d = math.hypot(dx, dy)
        if d <= self.follow_d:
            self.send(0.0, 0.0)
            return
        speed = min(1.6, (d - self.follow_d) * 2.0)
        self.send(dx / d * speed, dy / d * speed)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = IntruderControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
