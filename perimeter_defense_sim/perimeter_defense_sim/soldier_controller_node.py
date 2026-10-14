"""Own soldiers on foot (simulation side, common base).

Each soldier:
  - walks his patrol loop from zones.yaml (A* between the waypoints);
  - carries a transponder: /perimeter/transponders, 1 Hz, ~1 m error;
  - stops and radios spot reports (/perimeter/spot_reports) when he sees an
    intruder inside the fence; he cannot stop the intruder himself;
  - when the defense reports a captured person (track NEUTRALIZED on
    /threats/tracks), the closest free soldier goes there, takes him over
    from the UGV, escorts him to the detention building and reports the
    custody by radio (/perimeter/custody), then returns to his patrol.

Simulated perception and the escort script use the true actor poses; the
defense algorithms only receive the radio reports and transponder fixes.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, Vector3
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from perimeter_defense_msgs.msg import Alert, Custody, Track, TrackArray, Transponder
from perimeter_defense_sim.path_planning import GridPlanner, plan_route
from perimeter_defense_sim.soldiers import (ObserverParams, SightingLog, can_see, loop_waypoints,
                                            nearest_index, pick_captive, pursuit_step)
from perimeter_defense_world.geometry import load_layout

XY = Tuple[float, float]
CAPTURED_STATES = ('captured',)


@dataclass
class SoldierState:
    name: str
    callsign: str
    patrol: List[XY]
    cmd_pub: object
    state_pub: object
    sightings: SightingLog
    mode: str = 'patrol'            # patrol, observing, dispatched, escorting, handover, returning
    path: List[XY] = field(default_factory=list)
    seg: int = 0
    pose: Optional[XY] = None
    track_id: int = 0
    captive: str = ''
    mode_since: float = 0.0


class SoldierControllerNode(Node):
    def __init__(self) -> None:
        super().__init__('soldier_controller')
        self.declare_parameter('zones_file', '')
        self.declare_parameter('soldiers', [''])           # empty: all soldiers in zones.yaml
        self.declare_parameter('intruders', ['intruder1'])
        self.declare_parameter('walk_speed', 1.2)
        self.declare_parameter('hurry_speed', 2.0)
        self.declare_parameter('escort_speed', 1.0)
        self.declare_parameter('lookahead', 0.6)
        self.declare_parameter('observe_range', 12.0)
        self.declare_parameter('observe_time', 1.0)
        self.declare_parameter('report_period', 4.0)
        self.declare_parameter('report_std', 1.5)          # m, position error of a sighting
        self.declare_parameter('report_delay', 1.0)        # s, radio delay
        self.declare_parameter('transponder_period', 1.0)
        self.declare_parameter('transponder_std', 1.0)
        self.declare_parameter('pickup_distance', 1.6)
        self.declare_parameter('captive_search_radius', 4.0)
        self.declare_parameter('seed', 7)

        g = lambda n: self.get_parameter(n).value   # noqa: E731
        self.layout = load_layout(g('zones_file') or None)
        self.planner = GridPlanner(self.layout)
        self.rng = random.Random(int(g('seed')))
        self.walk, self.hurry, self.escort = float(g('walk_speed')), float(g('hurry_speed')), \
            float(g('escort_speed'))
        self.lookahead = float(g('lookahead'))
        self.observe_range = float(g('observe_range'))
        self.report_std, self.report_delay = float(g('report_std')), float(g('report_delay'))
        self.tx_period, self.tx_std = float(g('transponder_period')), float(g('transponder_std'))
        self.pickup_d = float(g('pickup_distance'))
        self.search_r = float(g('captive_search_radius'))
        obs = ObserverParams(self.observe_range, float(g('observe_time')), float(g('report_period')))

        names = [n for n in g('soldiers') if n] or list(self.layout.soldiers)
        self.soldiers: Dict[str, SoldierState] = {}
        for n in names:
            spec = self.layout.soldiers[n]
            self.soldiers[n] = SoldierState(
                name=n, callsign=spec.callsign,
                patrol=plan_route(self.planner, loop_waypoints(spec.route)),
                cmd_pub=self.create_publisher(Vector3, f'/actor/{n}/cmd_vel', 10),
                state_pub=self.create_publisher(String, f'/sim/actors/{n}/state', 10),
                sightings=SightingLog(obs))
            self.soldiers[n].path = self.soldiers[n].patrol
            self.create_subscription(Pose, f'/sim/actors/{n}/pose',
                                     lambda m, n=n: self.on_soldier_pose(n, m), 10)

        self.intruders: List[str] = list(g('intruders'))
        self.intruder_pose: Dict[str, XY] = {}
        self.intruder_state: Dict[str, str] = {}
        self.escort_pub: Dict[str, object] = {}
        for a in self.intruders:
            self.create_subscription(Pose, f'/sim/actors/{a}/pose',
                                     lambda m, a=a: self.intruder_pose.__setitem__(
                                         a, (m.position.x, m.position.y)), 10)
            self.create_subscription(String, f'/sim/actors/{a}/state',
                                     lambda m, a=a: self.intruder_state.__setitem__(a, m.data), 10)
            self.escort_pub[a] = self.create_publisher(String, f'/sim/actors/{a}/escort', 10)

        self.handled: Dict[int, str] = {}           # track id -> soldier
        self.custody: Dict[int, Custody] = {}       # latest custody report per track
        self.radio_queue: List[Tuple[float, Alert]] = []
        self.report_id = 0
        self.spot_pub = self.create_publisher(Alert, '/perimeter/spot_reports', 10)
        self.tx_pub = self.create_publisher(Transponder, '/perimeter/transponders', 20)
        self.custody_pub = self.create_publisher(Custody, '/perimeter/custody', 10)
        self.create_subscription(TrackArray, '/threats/tracks', self.on_tracks, 10)
        self.create_timer(0.1, self.tick)
        self.create_timer(self.tx_period, self.send_transponders)
        self.create_timer(1.0, self.repeat_custody)
        self.get_logger().info(f'soldiers: {", ".join(s.callsign for s in self.soldiers.values())}')

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_soldier_pose(self, name: str, msg: Pose) -> None:
        self.soldiers[name].pose = (msg.position.x, msg.position.y)

    # -- captured persons -----------------------------------------------------
    def on_tracks(self, msg: TrackArray) -> None:
        for t in msg.tracks:
            if t.state != Track.STATE_NEUTRALIZED or int(t.id) in self.handled:
                continue
            free = [s for s in self.soldiers.values()
                    if s.mode in ('patrol', 'observing', 'returning') and s.pose]
            if not free:
                return
            pos = (t.position.x, t.position.y)
            s = min(free, key=lambda s: math.dist(s.pose, pos))
            path = self.planner.plan(s.pose, pos)
            if path is None:
                continue
            self.handled[int(t.id)] = s.name
            s.track_id, s.captive = int(t.id), ''
            self.set_mode(s, 'dispatched', path)
            self.report_custody(s, Custody.DISPATCHED)
            self.get_logger().warn(f'{s.callsign}: on the way to captured T{t.id}')

    def report_custody(self, s: SoldierState, state: int) -> None:
        c = Custody(track_id=s.track_id, escort=s.callsign, state=state)
        c.header.stamp, c.header.frame_id = self.get_clock().now().to_msg(), 'map'
        if s.pose:
            c.position.x, c.position.y = s.pose
        self.custody[s.track_id] = c
        self.custody_pub.publish(c)

    def repeat_custody(self) -> None:
        # Radio reports are repeated so late listeners (dashboard) know the state.
        for c in self.custody.values():
            self.custody_pub.publish(c)

    # -- main loop ------------------------------------------------------------
    def set_mode(self, s: SoldierState, mode: str, path: Optional[List[XY]] = None) -> None:
        if mode != s.mode:
            self.get_logger().info(f'{s.callsign}: {mode}')
        s.mode, s.mode_since = mode, self.now_s()
        if path is not None:
            s.path, s.seg = path, 0

    def tick(self) -> None:
        now = self.now_s()
        if now <= 0.0:
            return
        for s in self.soldiers.values():
            if s.pose is None:
                continue
            self.observe(s, now)
            self.step(s, now)
            s.state_pub.publish(String(data=s.mode))
        self.flush_radio(now)

    def observe(self, s: SoldierState, now: float) -> None:
        visible = {}
        for a in self.intruders:
            p = self.intruder_pose.get(a)
            free = self.intruder_state.get(a, '') in ('walking', 'running')
            visible[a] = bool(p and free and can_see(self.layout, s.pose, p, self.observe_range))
        for a in s.sightings.update(visible, now):
            x, y = self.intruder_pose[a]
            self.report_id += 1
            alert = Alert(id=self.report_id, type=Alert.TYPE_SPOT_REPORT,
                          severity=Alert.SEVERITY_WARNING, source=f'soldier/{s.callsign}',
                          uncertainty=self.report_std * 1.3,
                          text=f'{s.callsign}: intruder sighted')
            alert.header.stamp, alert.header.frame_id = self.get_clock().now().to_msg(), 'map'
            alert.position.x = x + self.rng.gauss(0.0, self.report_std)
            alert.position.y = y + self.rng.gauss(0.0, self.report_std)
            self.radio_queue.append((now + self.report_delay, alert))
        # A patrolling soldier who sees an intruder stops to watch and report.
        if s.mode in ('patrol', 'observing'):
            self.set_mode(s, 'observing' if any(visible.values()) else 'patrol')

    def flush_radio(self, now: float) -> None:
        due = [a for t, a in self.radio_queue if t <= now]
        self.radio_queue = [(t, a) for t, a in self.radio_queue if t > now]
        for a in due:
            self.spot_pub.publish(a)
            self.get_logger().warn(f'{a.text} at ({a.position.x:.1f}, {a.position.y:.1f})')

    def step(self, s: SoldierState, now: float) -> None:
        if s.mode == 'observing':
            self.send(s, (0.0, 0.0))
            return
        if s.mode == 'patrol':
            v, s.seg, done = pursuit_step(s.pose, s.path, s.seg, self.lookahead, self.walk)
            if done:
                s.seg = 0
            self.send(s, v)
            return
        if s.mode == 'returning':
            v, s.seg, done = pursuit_step(s.pose, s.path, s.seg, self.lookahead, self.walk)
            if done:
                i = nearest_index(s.patrol, s.pose)
                self.set_mode(s, 'patrol', s.patrol)
                s.seg = min(i, len(s.patrol) - 2)
            self.send(s, v)
            return
        if s.mode == 'dispatched':
            self.step_dispatched(s, now)
            return
        if s.mode == 'escorting':
            v, s.seg, done = pursuit_step(s.pose, s.path, s.seg, self.lookahead, self.escort)
            self.send(s, v)
            if done:
                self.escort_pub[s.captive].publish(String(data='cell'))
                self.set_mode(s, 'handover')
            return
        if s.mode == 'handover':
            self.send(s, (0.0, 0.0))
            self.escort_pub[s.captive].publish(String(data='cell'))
            if self.intruder_state.get(s.captive) == 'detained' or now - s.mode_since > 10.0:
                self.report_custody(s, Custody.DETAINED)
                self.get_logger().warn(f'{s.callsign}: T{s.track_id} detained')
                self.go_back(s)

    def step_dispatched(self, s: SoldierState, now: float) -> None:
        captured = {a: p for a, p in self.intruder_pose.items()
                    if self.intruder_state.get(a) in CAPTURED_STATES}
        target = s.path[-1]
        captive = pick_captive(captured, target, self.search_r)
        if captive and math.dist(s.pose, captured[captive]) <= self.pickup_d:
            s.captive = captive
            self.escort_pub[captive].publish(String(data=s.name))
            path = self.planner.plan(s.pose, self.layout.detention.entry)
            self.set_mode(s, 'escorting', path or [s.pose, self.layout.detention.entry])
            self.report_custody(s, Custody.ESCORTING)
            self.get_logger().warn(f'{s.callsign}: took over T{s.track_id}, escorting to detention')
            return
        v, s.seg, done = pursuit_step(s.pose, s.path, s.seg, self.lookahead, self.hurry)
        self.send(s, v)
        if done and captive is None:
            self.get_logger().warn(f'{s.callsign}: nobody to take over at T{s.track_id}')
            self.go_back(s)

    def go_back(self, s: SoldierState) -> None:
        i = nearest_index(s.patrol, s.pose)
        path = self.planner.plan(s.pose, s.patrol[i])
        s.track_id, s.captive = 0, ''
        self.set_mode(s, 'returning', path or [s.pose, s.patrol[i]])

    def send(self, s: SoldierState, v: Tuple[float, float]) -> None:
        s.cmd_pub.publish(Vector3(x=float(v[0]), y=float(v[1]), z=0.0))

    def send_transponders(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for s in self.soldiers.values():
            if s.pose is None:
                continue
            m = Transponder(id=s.callsign, accuracy=self.tx_std)
            m.header.stamp, m.header.frame_id = stamp, 'map'
            m.position.x = s.pose[0] + self.rng.gauss(0.0, self.tx_std)
            m.position.y = s.pose[1] + self.rng.gauss(0.0, self.tx_std)
            self.tx_pub.publish(m)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SoldierControllerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
