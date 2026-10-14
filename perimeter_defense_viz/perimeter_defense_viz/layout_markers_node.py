"""Publish the FOB-3 base layout as a tactical RViz overlay.

Aerial background (top-down render of the Gazebo world), map grid with
labels, fence and breaches, the protected objective, authorized zones,
building labels and patrol routes. Everything comes from zones.yaml.
"""

from __future__ import annotations

import unicodedata
from typing import List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

from perimeter_defense_viz.markers import (base_marker, circle, color, dashed, line_list,
                                           line_strip, point, text)
from perimeter_defense_world.geometry import load_layout


def ascii_upper(s: str) -> str:
    """Upper-case text without diacritics, for RViz text markers."""
    return unicodedata.normalize('NFKD', s).encode('ascii', 'ignore').decode().upper()


COLOR_KEYS = ('fence', 'breach', 'objective', 'authorized', 'grid', 'label',
              'route_lup1', 'route_lup2', 'route_lup3')


class LayoutMarkersNode(Node):
    def __init__(self) -> None:
        super().__init__('layout_markers')
        self.declare_parameter('zones_file', '')
        self.declare_parameter('aerial_variant', 'day')
        self.declare_parameter('grid_spacing', 5.0)
        self.declare_parameter('label_height', 0.7)
        for key in COLOR_KEYS:
            self.declare_parameter(f'colors.{key}', [1.0, 1.0, 1.0, 1.0])

        self.layout = load_layout(self.get_parameter('zones_file').value or None)
        self.colors = {k: list(self.get_parameter(f'colors.{k}').value) for k in COLOR_KEYS}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(MarkerArray, '/perimeter/layout_markers', qos)
        self.pub.publish(self.build())
        self.get_logger().info('layout markers published')

    def build(self) -> MarkerArray:
        lay = self.layout
        c = self.colors
        h = float(self.get_parameter('label_height').value)
        markers: List[Marker] = []

        # Aerial background, slightly below the ground plane of the other markers.
        variant = self.get_parameter('aerial_variant').value
        bg = base_marker('aerial', 0, Marker.MESH_RESOURCE)
        bg.mesh_resource = f'package://perimeter_defense_world/maps/fob3_ortho_{variant}.dae'
        bg.mesh_use_embedded_materials = True
        bg.pose.position.z = -0.05
        # All-zero colour: RViz then uses the mesh material as is, without tinting it.
        bg.color = color([0.0, 0.0, 0.0, 0.0])
        markers.append(bg)

        # Base name above the north fence (ASCII only, RViz fonts lack diacritics).
        title = ascii_upper(lay.name)
        markers.append(text('title', 0, lay.width / 2.0, lay.height + 2.6, title, c['label'], h * 1.3))
        markers.append(text('title', 1, lay.width / 2.0, lay.height + 1.4, ascii_upper(lay.unit),
                            c['label'], h * 0.9))

        # Map grid with coordinate labels along the south and west edges.
        step = float(self.get_parameter('grid_spacing').value)
        segs = []
        x = 0.0
        while x <= lay.width + 1e-6:
            segs.append(((x, 0.0), (x, lay.height)))
            markers.append(text('grid_labels', len(markers), x, -0.8, f'{int(x):02d}',
                                c['label'], h * 0.8))
            x += step
        y = 0.0
        while y <= lay.height + 1e-6:
            segs.append(((0.0, y), (lay.width, y)))
            markers.append(text('grid_labels', len(markers), -1.0, y, f'{int(y):02d}',
                                c['label'], h * 0.8))
            y += step
        markers.append(line_list('grid', 0, segs, c['grid'], 0.03, z=0.02))

        # Fence (breaches are cut out) and breaches.
        fence = [(s.a, s.b) for s in lay.fence_segments()]
        markers.append(line_list('fence', 0, fence, c['fence'], 0.15, z=0.06))
        for i, br in enumerate(lay.breaches):
            bx, by = br.center(lay.fence_min, lay.fence_max)
            if br.side in ('north', 'south'):
                seg = ((br.start, by), (br.end, by))
                lx, ly = bx, by + (1.0 if br.side == 'north' else -1.0)
            else:
                seg = ((bx, br.start), (bx, br.end))
                lx, ly = bx - (2.2 if br.side == 'west' else -2.2), by
            markers.append(line_list('breach', i, dashed(list(seg), 0.4, 0.25), c['breach'], 0.2,
                                     z=0.08))
            markers.append(text('breach_labels', i, lx, ly, f'BREACH {br.name[0].upper()}',
                                c['breach'], h))

        # Protected objective.
        tx, ty = lay.objective_target
        markers.append(line_strip('objective', 0, circle(tx, ty, lay.objective_radius),
                                  c['objective'], 0.15, z=0.08, closed=True))
        dot = base_marker('objective', 1, Marker.CYLINDER)
        dot.pose.position.x, dot.pose.position.y, dot.pose.position.z = tx, ty, 0.08
        dot.scale.x = dot.scale.y = 0.5
        dot.scale.z = 0.02
        dot.color = color(c['objective'])
        markers.append(dot)
        markers.append(text('objective', 2, tx, ty + lay.objective_radius + 0.6, 'OBJ',
                            c['objective'], h))

        # Authorized zones for own personnel, translucent fill.
        for i, (name, poly) in enumerate(lay.authorized_zones.items()):
            zone = base_marker('authorized', i, Marker.TRIANGLE_LIST)
            zone.color = color(c['authorized'])
            for k in range(1, len(poly) - 1):  # fan triangulation, the zones are convex
                for vx, vy in (poly[0], poly[k], poly[k + 1]):
                    zone.points.append(point(vx, vy, 0.03))
            markers.append(zone)

        # Building labels.
        for i, b in enumerate(lay.buildings):
            markers.append(text('building_labels', i, b.center[0], b.center[1],
                                b.label.upper(), c['label'], h, z=b.height + 0.5))

        # Patrol routes, dashed in the colour of each robot.
        for i, (robot, route) in enumerate(sorted(lay.patrol_routes.items())):
            rgba = c.get(f'route_{robot}', c['label'])
            markers.append(line_list('routes', i, dashed(route, 0.5, 0.35, closed=True), rgba,
                                     0.07, z=0.07))
        return MarkerArray(markers=markers)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LayoutMarkersNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
