"""IFF node: classifies tracks as FRIEND / HOSTILE / UNKNOWN from transponders.

Inputs:  /threats/tracks          tracks from the tracker
         /perimeter/transponders  own personnel transponder fixes (1 Hz, ~1 m error)
         /fleet/transponders      own UGVs' positions from their localization (5 Hz);
                                  robots are FRIEND and allowed everywhere
         /perimeter/custody       soldiers escorting a detainee (allowed off their zones)
Outputs: /threats/classification  TrackArray with id, classification, transponder_id
         /perimeter/alerts        HOSTILE_CONFIRMED once per track,
                                  UNAUTHORIZED_ZONE for own personnel outside the
                                  authorized zones (alert only, never intercepted)
"""

from __future__ import annotations

import math
from typing import Dict, List

from rclpy.node import Node

from perimeter_defense.core.iff import FRIEND, HOSTILE, Fix, IffClassifier, IffParams, TrackView
from perimeter_defense.ros_utils import spin_node, to_sec, to_stamp
from perimeter_defense_msgs.msg import Alert, Custody, Track, TrackArray, Transponder
from perimeter_defense_world.geometry import load_layout


class IffNode(Node):
    def __init__(self) -> None:
        super().__init__('iff')
        self.declare_parameter('zones_file', '')
        self.declare_parameter('rate', 2.0)
        self.declare_parameter('zone_alert_cooldown', 30.0)
        self.declare_parameter('zone_margin', 1.5)
        self.declare_parameter('return_time', 60.0)
        defaults = IffParams()
        for name in defaults.__dataclass_fields__:
            self.declare_parameter(name, getattr(defaults, name))
        self.iff = IffClassifier(IffParams(**{n: self.get_parameter(n).value
                                              for n in defaults.__dataclass_fields__}))
        self.layout = load_layout(self.get_parameter('zones_file').value or None)
        self.zone_cooldown = float(self.get_parameter('zone_alert_cooldown').value)
        self.zone_margin = float(self.get_parameter('zone_margin').value)
        self.return_time = float(self.get_parameter('return_time').value)

        self.tracks: List[Track] = []
        self.fixes: Dict[str, Fix] = {}
        self.on_duty_until: Dict[str, float] = {}   # soldier -> zone alerts suppressed until
        self.hostile_reported: set = set()
        self.zone_alerted: Dict[str, float] = {}
        self.alert_id = 0

        self.create_subscription(TrackArray, '/threats/tracks', self.on_tracks, 10)
        self.robot_ids: set = set()
        self.create_subscription(Transponder, '/perimeter/transponders', self.on_fix, 20)
        self.create_subscription(Transponder, '/fleet/transponders', self.on_robot_fix, 20)
        self.create_subscription(Custody, '/perimeter/custody', self.on_custody, 10)
        self.class_pub = self.create_publisher(TrackArray, '/threats/classification', 10)
        self.alert_pub = self.create_publisher(Alert, '/perimeter/alerts', 10)
        self.create_timer(1.0 / float(self.get_parameter('rate').value), self.tick)

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_tracks(self, msg: TrackArray) -> None:
        self.tracks = list(msg.tracks)

    def on_fix(self, msg: Transponder) -> None:
        self.fixes[msg.id] = Fix(msg.id, (msg.position.x, msg.position.y), to_sec(msg.header.stamp))

    def on_robot_fix(self, msg: Transponder) -> None:
        self.robot_ids.add(msg.id)
        self.on_fix(msg)

    def on_custody(self, msg: Custody) -> None:
        # A soldier on an escort duty, and on his way back, may leave his zone.
        if msg.state in (Custody.DISPATCHED, Custody.ESCORTING):
            self.on_duty_until[msg.escort] = math.inf
        elif self.on_duty_until.get(msg.escort, math.inf) == math.inf:
            # First DETAINED report (they are repeated): give him time to walk back.
            self.on_duty_until[msg.escort] = self.now_s() + self.return_time

    def tick(self) -> None:
        now = self.now_s()
        views = [TrackView(int(t.id), int(t.state), (t.position.x, t.position.y),
                           (t.velocity.x, t.velocity.y), int(t.classification)) for t in self.tracks]
        result = self.iff.update(views, list(self.fixes.values()), now)
        out = TrackArray()
        out.header.stamp, out.header.frame_id = to_stamp(now), 'map'
        for t in self.tracks:
            info = result.get(int(t.id))
            if info is None:
                continue
            m = Track(id=t.id, state=t.state, classification=info.classification,
                      transponder_id=info.transponder, position=t.position)
            out.tracks.append(m)
            if info.classification == HOSTILE and t.id not in self.hostile_reported:
                self.hostile_reported.add(t.id)
                self.alert(Alert.TYPE_HOSTILE_CONFIRMED, Alert.SEVERITY_CRITICAL, t,
                           f'T{t.id}: no transponder, HOSTILE')
            if info.classification == FRIEND:
                self.check_zone(t, info.transponder, now)
        self.class_pub.publish(out)

    def check_zone(self, t: Track, transponder: str, now: float) -> None:
        if (t.state != Track.STATE_TRACKED or transponder in self.robot_ids
                or self.on_duty_until.get(transponder, -math.inf) > now):
            return
        # The track position is noisy: tolerate zone_margin around the zones.
        x, y, m = t.position.x, t.position.y, self.zone_margin
        if any(self.layout.in_authorized_zone(p)
               for p in ((x, y), (x + m, y), (x - m, y), (x, y + m), (x, y - m))):
            return
        if now - self.zone_alerted.get(transponder, -1e9) < self.zone_cooldown:
            return
        self.zone_alerted[transponder] = now
        self.alert(Alert.TYPE_UNAUTHORIZED_ZONE, Alert.SEVERITY_INFO, t,
                   f'{transponder} outside the authorized zones at ({x:.1f}, {y:.1f}), no intercept')

    def alert(self, kind: int, severity: int, t: Track, text: str) -> None:
        self.alert_id += 1
        a = Alert(id=self.alert_id, type=kind, severity=severity, source='iff',
                  track_id=t.id, text=text)
        a.header.stamp, a.header.frame_id = self.get_clock().now().to_msg(), 'map'
        a.position.x, a.position.y = t.position.x, t.position.y
        self.alert_pub.publish(a)
        self.get_logger().warn(text)


def main(args=None) -> None:
    spin_node(IffNode, args)


if __name__ == '__main__':
    main()
