"""Status board drawn next to the map in RViz (east of the base).

Top: every contact registered inside the perimeter, one card each: id,
class, status (ALERT / TRACKED / LOST / CAPTURED / ESCORTED / DETAINED /
NO CONTACT), how it was first reported (breach sensor, soldier spot report
or lidar), time since registration, speed, distance to the objective, the
robot working on it and the soldier who has it in custody.
Bottom: own forces, the robots' roles and the soldiers' status.

Defense information only: tracks, alerts, robot roles, custody reports and
transponders; no simulator ground truth.
"""

from __future__ import annotations

from typing import Dict, List, Sequence, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from perimeter_defense_msgs.msg import Alert, Custody, RoleAssignment, TrackArray, Transponder
from perimeter_defense_viz.board import (CLASS_NAMES, DETAINED, ESCORTING, HOSTILE, ROLE_NAMES,
                                         UNKNOWN, ContactRegister, Entry, TrackInfo, format_elapsed)
from perimeter_defense_viz.markers import base_marker, color, line_strip, point, text
from perimeter_defense_world.geometry import load_layout

WHITE = [0.92, 0.94, 0.90, 1.0]
DIM = [0.62, 0.66, 0.60, 1.0]
STATUS_COLORS = {
    'ALERT': [1.0, 0.62, 0.15, 1.0], 'TRACKED': [1.0, 0.30, 0.25, 1.0],
    'LOST': [0.85, 0.75, 0.35, 1.0], 'CAPTURED': [0.45, 0.95, 0.45, 1.0],
    'ESCORTED': [0.45, 0.85, 1.0, 1.0], 'DETAINED': [0.60, 0.75, 0.60, 1.0],
    'NO CONTACT': [0.60, 0.60, 0.60, 1.0],
}
CLASS_COLORS = {HOSTILE: [1.0, 0.30, 0.25, 1.0], UNKNOWN: [1.0, 0.85, 0.25, 1.0]}


class StatusBoardNode(Node):
    def __init__(self) -> None:
        super().__init__('status_board')
        self.declare_parameter('zones_file', '')
        self.declare_parameter('robots', ['lup1', 'lup2', 'lup3'])
        self.declare_parameter('callsigns', ['LUP-1', 'LUP-2', 'LUP-3'])
        self.declare_parameter('origin', [40.5, 32.0])      # top-left corner on the map, m
        self.declare_parameter('width', 16.0)
        self.declare_parameter('text_height', 0.7)
        self.declare_parameter('line_spacing', 0.95)
        self.declare_parameter('max_contacts', 6)
        self.declare_parameter('rate', 2.0)
        self.declare_parameter('transponder_timeout', 3.0)

        g = lambda n: self.get_parameter(n).value   # noqa: E731
        layout = load_layout(g('zones_file') or None)
        self.register = ContactRegister(layout.is_inside_fence, layout.objective_target)
        self.callsign = dict(zip(g('robots'), g('callsigns')))
        self.soldiers = [s.callsign for s in layout.soldiers.values()]
        self.x0, self.y0 = (float(v) for v in g('origin'))
        self.w = float(g('width'))
        self.th = float(g('text_height'))
        self.dy = float(g('line_spacing'))
        self.max_contacts = int(g('max_contacts'))
        self.tx_timeout = float(g('transponder_timeout'))

        self.tracks: List[TrackInfo] = []
        self.roles: Dict[str, Tuple[int, int]] = {}
        self.custody: Dict[int, Tuple[str, int]] = {}
        self.soldier_task: Dict[str, Tuple[int, int]] = {}
        self.last_tx: Dict[str, float] = {}
        self.last_spot: Dict[str, float] = {}

        self.create_subscription(TrackArray, '/threats/tracks', self.on_tracks, 10)
        self.create_subscription(Alert, '/perimeter/fence_alert', self.on_fence, 10)
        self.create_subscription(Alert, '/perimeter/spot_reports', self.on_spot, 10)
        self.create_subscription(RoleAssignment, '/fleet/roles', self.on_role, 10)
        self.create_subscription(Custody, '/perimeter/custody', self.on_custody, 10)
        self.create_subscription(Transponder, '/perimeter/transponders', self.on_tx, 20)
        self.pub = self.create_publisher(MarkerArray, '/perimeter/status_board', 10)
        self.create_timer(1.0 / float(g('rate')), self.tick)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # -- inputs ---------------------------------------------------------------
    def on_tracks(self, msg: TrackArray) -> None:
        self.tracks = [TrackInfo(int(t.id), int(t.state), int(t.classification),
                                 (t.position.x, t.position.y), (t.velocity.x, t.velocity.y))
                       for t in msg.tracks]

    def on_fence(self, msg: Alert) -> None:
        name = msg.source.split('/')[-1].upper()
        self.register.add_report(f'BREACH {name}', (msg.position.x, msg.position.y), self.now_s())

    def on_spot(self, msg: Alert) -> None:
        soldier = msg.source.split('/')[-1]
        self.last_spot[soldier] = self.now_s()
        self.register.add_report(f'SPOT {soldier}', (msg.position.x, msg.position.y), self.now_s())

    def on_role(self, msg: RoleAssignment) -> None:
        self.roles[self.callsign.get(msg.robot, msg.robot.upper())] = (int(msg.role), int(msg.track_id))

    def on_custody(self, msg: Custody) -> None:
        self.custody[int(msg.track_id)] = (msg.soldier, int(msg.state))
        self.soldier_task[msg.soldier] = (int(msg.track_id), int(msg.state))

    def on_tx(self, msg: Transponder) -> None:
        self.last_tx[msg.id] = self.now_s()

    # -- drawing --------------------------------------------------------------
    def tick(self) -> None:
        now = self.now_s()
        self.register.update(self.tracks, self.roles, self.custody, now)
        self.mid = 0
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = 'map'
        out: List[Marker] = [clear]
        x0, w = self.x0, self.w
        cols = (x0 + w * 0.2, x0 + w * 0.52, x0 + w * 0.83)
        y = self.y0 - 0.8

        out.append(self.label(x0 + w / 2, y, 'CONTACTS INSIDE PERIMETER', WHITE, 1.3))
        y -= self.dy * 1.1
        c = self.register.counters()
        out.append(self.label(x0 + w / 2, y, '   '.join(f'{k} {v}' for k, v in c.items()), DIM))
        y -= self.dy * 0.7
        out.append(self.rule(y))

        rows = self.register.rows(self.max_contacts)
        if not rows:
            y -= self.dy
            out.append(self.label(x0 + w / 2, y, 'NO CONTACTS', DIM))
            y -= self.dy * 0.7
        for e in rows:
            y = self.card(out, e, cols, y, now)

        y -= self.dy
        out.append(self.label(x0 + w / 2, y, 'OWN FORCES', WHITE, 1.15))
        y -= self.dy * 0.6
        out.append(self.rule(y))
        for callsign in sorted(self.roles):
            role, track = self.roles[callsign]
            y -= self.dy
            detail = f'T{track}' if track else ''
            self.row(out, cols, y, [(callsign, WHITE), (ROLE_NAMES.get(role, '?'), self.role_color(role)),
                                    (detail, DIM)])
        for callsign in self.soldiers:
            y -= self.dy
            status, detail = self.soldier_status(callsign, now)
            self.row(out, cols, y, [(callsign, WHITE), (status, DIM if status == 'PATROL' else
                                                        STATUS_COLORS['ESCORTED']), (detail, DIM)])
        y -= self.dy * 0.8

        bg = base_marker('board_bg', 0, Marker.CUBE)
        bg.pose.position = point(x0 + w / 2, (self.y0 + y) / 2, 0.02)
        bg.scale.x, bg.scale.y, bg.scale.z = w, self.y0 - y, 0.02
        bg.color = color([0.05, 0.06, 0.05, 0.86])
        out.append(bg)
        out.append(line_strip('board_frame', 0, [(x0, self.y0), (x0 + w, self.y0), (x0 + w, y), (x0, y)],
                              [0.55, 0.60, 0.52, 0.9], 0.06, z=0.05, closed=True))
        self.pub.publish(MarkerArray(markers=out))

    def card(self, out: List[Marker], e: Entry, cols: Sequence[float], y: float, now: float) -> float:
        status = e.status
        scol = STATUS_COLORS.get(status, WHITE)
        top = y
        y -= self.dy
        self.row(out, cols, y, [(e.label, WHITE),
                                (CLASS_NAMES.get(e.classification, '?'),
                                 CLASS_COLORS.get(e.classification, WHITE)),
                                (status, scol)], scale=1.2)
        y -= self.dy
        speed = f'{e.speed:.1f} m/s' if status in ('ALERT', 'TRACKED', 'LOST') else '-'
        self.row(out, cols, y, [(e.via, DIM), (f'T+ {format_elapsed(now - e.registered)}', DIM),
                                (speed, DIM)])
        y -= self.dy
        self.row(out, cols, y, [(f'OBJ {self.register.distance_to_objective(e):.0f} m', DIM),
                                (e.robot or '-', DIM), (e.custody_text, DIM)])
        y -= self.dy * 0.5
        # Status bar on the left edge of the card.
        bar = [(self.x0 + 0.25, top - 0.15), (self.x0 + 0.25, y + 0.1)]
        out.append(line_strip('board_bar', self.next_id(), bar, scol, 0.18, z=0.05))
        out.append(self.rule(y))
        return y

    def soldier_status(self, callsign: str, now: float) -> Tuple[str, str]:
        if now - self.last_tx.get(callsign, -1e9) > self.tx_timeout:
            return 'NO SIGNAL', ''
        track, state = self.soldier_task.get(callsign, (0, DETAINED))
        if state != DETAINED:
            return ('ESCORT' if state == ESCORTING else 'TO CAPTIVE'), f'T{track}'
        if now - self.last_spot.get(callsign, -1e9) < 5.0:
            return 'REPORTING', 'SPOT'
        return 'PATROL', 'TX OK'

    @staticmethod
    def role_color(role: int) -> List[float]:
        return DIM if role == 0 else ([0.45, 0.95, 0.45, 1.0] if role == 8 else [1.0, 0.62, 0.15, 1.0])

    def row(self, out: List[Marker], cols: Sequence[float], y: float,
            cells: Sequence[Tuple[str, List[float]]], scale: float = 1.0) -> None:
        for x, (label, rgba) in zip(cols, cells):
            if label:
                out.append(self.label(x, y, label, rgba, scale))

    def label(self, x: float, y: float, s: str, rgba: List[float], scale: float = 1.0) -> Marker:
        return text('board_text', self.next_id(), x, y, s, rgba, self.th * scale, z=0.1)

    def rule(self, y: float) -> Marker:
        return line_strip('board_rule', self.next_id(), [(self.x0 + 0.4, y), (self.x0 + self.w - 0.4, y)],
                          [0.40, 0.44, 0.38, 0.9], 0.03, z=0.05)

    def next_id(self) -> int:
        self.mid += 1
        return self.mid


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StatusBoardNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
