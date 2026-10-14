"""Scripted intruder (simulation side, common base).

Walks from its spawn point outside the fence through a breach and along the
west or east route to the ammunition depot entrance, following an A* path on
the base map. It switches to running once the defense has a track on it
(within detect_radius of its true position), stops when a robot gets within
capture_distance (surrender) or when it reaches the target.

Uses the actor's true pose (/sim/actors/<name>/pose) and the robots' true
poses (/sim/ground_truth/<robot>/pose): this is the scenario script, not part
of the defense algorithms.
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from perimeter_defense_msgs.msg import Track, TrackArray
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

        spawn = layout.intruder_spawns[g('spawn')]
        waypoints = [spawn] + self.route_waypoints(layout, g('spawn'), g('route')) + [self.target]
        self.path = plan_route(GridPlanner(layout), waypoints)
        self.seg = 0
        self.pose: Optional[XY] = None
        self.robot_poses: Dict[str, XY] = {}
        self.state = 'waiting'
        self.running = False
        self.t0: Optional[float] = None

        self.cmd_pub = self.create_publisher(Vector3, f'/actor/{self.actor}/cmd_vel', 10)
        self.state_pub = self.create_publisher(String, f'/sim/actors/{self.actor}/state', 10)
        self.create_subscription(Pose, f'/sim/actors/{self.actor}/pose', self.on_pose, 10)
        for robot in self.robots:
            self.create_subscription(Pose, f'/sim/ground_truth/{robot}/pose',
                                     lambda m, r=robot: self.on_robot_pose(r, m), 10)
        self.create_subscription(TrackArray, '/threats/tracks', self.on_tracks, 10)
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

    def set_state(self, state: str) -> None:
        if state != self.state:
            self.state = state
            self.get_logger().info(f'{self.actor}: {state}')
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
            return
        if self.state in ('captured', 'reached_target'):
            self.send(0.0, 0.0)
            self.set_state(self.state)
            return
        if any(math.dist(self.pose, p) <= self.capture_d for p in self.robot_poses.values()):
            self.set_state('captured')
            self.send(0.0, 0.0)
            return
        if math.dist(self.pose, self.target) <= self.target_r:
            self.set_state('reached_target')
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
        self.set_state('running' if self.running else 'walking')


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
