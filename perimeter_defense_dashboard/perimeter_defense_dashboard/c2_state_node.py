"""C2 state for the web dashboard.

Collects the defense picture and publishes it as one JSON document on
/c2/state (std_msgs/String, 4 Hz), which the browser receives through
rosbridge:
  robots    pose (own localization, TF), speed (odometry), role, track, goal,
            distance to goal, Nav2 plan, simulated battery and radio link
  tracks    state, class, position, velocity, covariance, 6 s prediction
  contacts  register of every contact inside the perimeter (as the RViz board)
  events    last alerts, role changes, custody reports (numbered)
  threat    provisional threat level
No simulator ground truth is used.
"""

from __future__ import annotations

import json
import math
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from perimeter_defense_dashboard.c2_model import (CLASSES, ROLE_NAMES, TRACK_STATES, BatteryModel,
                                                  ChangeWatch, EventLog, covariance_2x2, downsample,
                                                  link_quality, threat_level)
from perimeter_defense_msgs.msg import (Alert, Custody, PredictedPathArray, RoleAssignment,
                                        TrackArray)
from perimeter_defense_viz.board import ContactRegister, TrackInfo
from perimeter_defense_world.geometry import load_layout

XY = Tuple[float, float]


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class C2StateNode(Node):
    def __init__(self) -> None:
        super().__init__('c2_state')
        self.declare_parameter('zones_file', '')
        self.declare_parameter('robots', ['lup1', 'lup2', 'lup3'])
        self.declare_parameter('rate', 4.0)
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('command_post', [8.5, 17.0])   # radio mast at the flagpole, m
        self.declare_parameter('plan_points', 40)
        self.declare_parameter('events_sent', 60)

        g = lambda n: self.get_parameter(n).value   # noqa: E731
        self.layout = load_layout(g('zones_file') or None)
        self.frame = g('global_frame')
        self.robots: List[str] = [r for r in g('robots') if r]
        self.command_post: XY = tuple(float(v) for v in g('command_post'))
        self.plan_points = int(g('plan_points'))
        self.events_sent = int(g('events_sent'))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.register = ContactRegister(self.layout.is_inside_fence, self.layout.objective_target)
        self.log = EventLog()
        self.watch = ChangeWatch()
        self.battery = {r: BatteryModel() for r in self.robots}
        self.speed: Dict[str, float] = {}
        self.plans: Dict[str, List[XY]] = {}
        self.roles: Dict[str, RoleAssignment] = {}
        self.tracks: TrackArray = TrackArray()
        self.paths: Dict[int, List[XY]] = {}
        self.custody: Dict[int, Tuple[str, int]] = {}
        self.last_tick: Optional[float] = None

        for r in self.robots:
            self.create_subscription(Odometry, f'/{r}/odom',
                                     lambda m, r=r: self.speed.__setitem__(
                                         r, math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)), 10)
            self.create_subscription(Path, f'/{r}/plan', lambda m, r=r: self.plans.__setitem__(
                r, [(p.pose.position.x, p.pose.position.y) for p in m.poses]), 5)
        self.mode = ''
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, '/fleet/mode', self.on_mode, latched)
        self.create_subscription(RoleAssignment, '/fleet/roles',
                                 lambda m: self.roles.__setitem__(m.robot, m), 20)
        self.create_subscription(TrackArray, '/threats/tracks', lambda m: setattr(self, 'tracks', m), 10)
        self.create_subscription(PredictedPathArray, '/threats/predicted_paths', self.on_paths, 10)
        self.create_subscription(Custody, '/perimeter/custody', self.on_custody, 10)
        self.create_subscription(Alert, '/perimeter/fence_alert', lambda m: self.on_alert(m, 'FENCE'), 10)
        self.create_subscription(Alert, '/perimeter/spot_reports', lambda m: self.on_alert(m, 'SPOT'), 10)
        self.create_subscription(Alert, '/perimeter/alerts', lambda m: self.on_alert(m, 'IFF'), 10)
        self.pub = self.create_publisher(String, '/c2/state', 10)
        self.create_timer(1.0 / float(g('rate')), self.tick)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- inputs ---------------------------------------------------------------
    def on_paths(self, msg: PredictedPathArray) -> None:
        self.paths = {int(p.track_id): [(q.x, q.y) for q in p.points] for p in msg.paths}

    def on_mode(self, msg: String) -> None:
        if self.mode and msg.data != self.mode:
            self.log.add(self.now_s(), 'MODE', 'INFO',
                         'Mod interceptare: ' + {'predictive': 'PREDICTIV', 'naive': 'NAIV'}.get(msg.data, msg.data))
        self.mode = msg.data

    def on_custody(self, msg: Custody) -> None:
        self.custody[int(msg.track_id)] = (msg.escort, int(msg.state))
        self.watch.custody_state(self.log, self.now_s(), int(msg.track_id), msg.escort, int(msg.state),
                                 (msg.position.x, msg.position.y))

    def on_alert(self, msg: Alert, kind: str) -> None:
        pos = (msg.position.x, msg.position.y)
        severity = {0: 'INFO', 1: 'WARNING', 2: 'CRITICAL'}.get(int(msg.severity), 'INFO')
        if kind == 'FENCE':
            name = msg.source.split('/')[-1].upper()
            ro = {'NORTH': 'NORD', 'WEST': 'VEST', 'EAST': 'EST', 'SOUTH': 'SUD'}.get(name, name)
            self.register.add_report(f'BREACH {name}', pos, self.now_s())
            text = f'Breșa {ro}: perimetru traversat'
        elif kind == 'SPOT':
            who = msg.source.split('/')[-1]
            self.register.add_report(f'SPOT {who}', pos, self.now_s())
            text = f'{who}: intrus semnalat'
        elif msg.type == Alert.TYPE_HOSTILE_CONFIRMED:
            text = f'T{msg.track_id}: fără transponder, OSTIL'
        elif msg.type == Alert.TYPE_UNAUTHORIZED_ZONE:
            text = msg.text.split(' outside')[0] + ': în afara zonei autorizate (fără interceptare)'
        else:
            text = msg.text
        self.log.add(self.now_s(), kind, severity, text, pos, int(msg.track_id))

    # -- output ---------------------------------------------------------------
    def robot_state(self, r: str, now: float, dt: float) -> Optional[dict]:
        try:
            tf = self.tf_buffer.lookup_transform(self.frame, f'{r}/base_link', Time())
        except TransformException:
            return None
        t = tf.transform
        x, y, yaw = t.translation.x, t.translation.y, _yaw(t.rotation)
        callsign = callsign_of(r)
        role_msg = self.roles.get(r)
        role = ROLE_NAMES.get(int(role_msg.role), '?') if role_msg else 'PATROL'
        track = int(role_msg.track_id) if role_msg else 0
        goal = [round(role_msg.goal.x, 2), round(role_msg.goal.y, 2)] if role_msg and (
            role_msg.goal.x or role_msg.goal.y) else None
        plan = self.plans.get(r, [])
        goal_xy = tuple(goal) if goal else (plan[-1] if plan else None)
        self.watch.role(self.log, now, callsign, role, track)
        blocked = self.layout.buildings_between((x, y), self.command_post)
        return {
            'name': r, 'callsign': callsign, 'x': round(x, 2), 'y': round(y, 2),
            'yaw': round(yaw, 3), 'speed': round(self.speed.get(r, 0.0), 2), 'role': role,
            'track': track, 'goal': list(goal_xy) if goal_xy else None,
            'goal_dist': round(math.dist((x, y), goal_xy), 1) if goal_xy else None,
            'eta': round(float(role_msg.eta), 1) if role_msg and role_msg.eta > 0.0 else None,
            'plan': downsample(plan, self.plan_points),
            'battery': round(self.battery[r].update((x, y), dt), 1),
            'link': round(link_quality((x, y), self.command_post, blocked), 0),
        }

    def tick(self) -> None:
        now = self.now_s()
        dt = 0.0 if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        robots = [s for s in (self.robot_state(r, now, dt) for r in self.robots) if s]

        tracks = []
        infos = []
        for t in self.tracks.tracks:
            state, cls = TRACK_STATES.get(int(t.state), '?'), CLASSES.get(int(t.classification), '?')
            pos = (t.position.x, t.position.y)
            self.watch.track(self.log, now, int(t.id), state, cls, pos)
            infos.append(TrackInfo(int(t.id), int(t.state), int(t.classification), pos,
                                   (t.velocity.x, t.velocity.y)))
            tracks.append({
                'id': int(t.id), 'state': state, 'cls': cls,
                'x': round(pos[0], 2), 'y': round(pos[1], 2),
                'vx': round(t.velocity.x, 2), 'vy': round(t.velocity.y, 2),
                'cov': covariance_2x2(t.covariance), 'tp': t.transponder_id,
                'first_seen': round(_sec(t.first_seen), 2),
                'path': downsample(self.paths.get(int(t.id), []), 31),
            })

        roles = {callsign_of(r): (int(m.role), int(m.track_id)) for r, m in self.roles.items()}
        self.register.update(infos, roles, self.custody, now)
        contacts = [{
            'id': e.track_id, 'label': e.label, 'status': e.status,
            'cls': CLASSES.get(e.classification, '?'), 'via': e.via,
            'registered': round(e.registered, 2), 'speed': round(e.speed, 2),
            'x': round(e.position[0], 2), 'y': round(e.position[1], 2),
            'obj': round(self.register.distance_to_objective(e), 1),
            'robot': e.robot, 'custody': list(e.custody) if e.custody else None,
        } for e in self.register.rows(50)]

        state = {
            't': round(now, 2), 'wall': round(time.time(), 2),
            'robots': robots, 'tracks': tracks, 'contacts': contacts,
            'counters': self.register.counters(), 'threat': threat_level(contacts), 'mode': self.mode,
            'events': self.log.recent(self.events_sent),
        }
        self.pub.publish(String(data=json.dumps(state, ensure_ascii=False, separators=(',', ':'))))


def callsign_of(robot: str) -> str:
    """lup2 -> LUP-2"""
    head = robot.rstrip('0123456789')
    return f'{head.upper()}-{robot[len(head):]}' if head != robot else robot.upper()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = C2StateNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
