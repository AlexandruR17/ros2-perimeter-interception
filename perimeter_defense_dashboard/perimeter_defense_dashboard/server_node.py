"""Web server of the C2 dashboard.

Serves the static web application, the base layout as /config.json
(generated from zones.yaml, so the dashboard map always matches the world)
and the top-down renders of the world package under /maps/. Live data comes
from rosbridge (websocket, port 9090 by default), not from this server.
"""

from __future__ import annotations

import json
import os
import threading
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from perimeter_defense_world.geometry import BaseLayout, load_layout


def layout_config(layout: BaseLayout, robots: list, rosbridge_port: int) -> dict:
    """Everything the browser needs to draw the base."""
    rect = lambda r: {'name': r.name, 'label': r.label, 'kind': r.kind,   # noqa: E731
                      'min': list(r.min), 'max': list(r.max), 'height': r.height}
    det = layout.detention
    return {
        'name': layout.name, 'unit': layout.unit, 'code': layout.code,
        'mgrs_origin': layout.mgrs_origin,
        'fence': {'min': list(layout.fence_min), 'max': list(layout.fence_max),
                  'segments': [[list(s.a), list(s.b)] for s in layout.fence_segments()]},
        'breaches': [{'name': b.name, 'side': b.side,
                      'center': list(b.center(layout.fence_min, layout.fence_max)),
                      'from': b.start, 'to': b.end} for b in layout.breaches],
        'buildings': [rect(b) for b in layout.buildings],
        'props': [rect(p) for p in layout.props],
        'objective': {'target': list(layout.objective_target), 'radius': layout.objective_radius},
        'zones': {k: [list(p) for p in v] for k, v in layout.authorized_zones.items()},
        'routes': {k: [list(p) for p in v] for k, v in layout.patrol_routes.items()},
        'detention': ({'building': det.building, 'entry': list(det.entry), 'cell': list(det.cell)}
                      if det else None),
        'render': {'min': list(layout.render_min), 'max': list(layout.render_max)},
        'aerial': {'day': 'maps/fob3_ortho_day.png', 'night': 'maps/fob3_ortho_night.png'},
        'robots': robots,
        'rosbridge_port': rosbridge_port,
    }


class _Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, web_dir: str, maps_dir: str, config: bytes, **kwargs) -> None:
        self.maps_dir = maps_dir
        self.config = config
        super().__init__(*args, directory=web_dir, **kwargs)

    def translate_path(self, path: str) -> str:
        clean = path.split('?', 1)[0].split('#', 1)[0]
        if clean.startswith('/maps/'):
            name = os.path.basename(clean)
            return os.path.join(self.maps_dir, name)
        return super().translate_path(path)

    def do_GET(self) -> None:   # noqa: N802 - http.server API
        if self.path.split('?', 1)[0] == '/config.json':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(self.config)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(self.config)
            return
        super().do_GET()

    def log_message(self, fmt: str, *args) -> None:
        pass


class DashboardServer(Node):
    def __init__(self) -> None:
        super().__init__('dashboard_server')
        self.declare_parameter('zones_file', '')
        self.declare_parameter('port', 8080)
        self.declare_parameter('rosbridge_port', 9090)
        self.declare_parameter('robots', ['lup1', 'lup2', 'lup3'])
        self.declare_parameter('routes', '')     # patrol routes drawn, '' = the robots' own
        port = int(self.get_parameter('port').value)
        layout = load_layout(self.get_parameter('zones_file').value or None)
        cfg = layout_config(layout, list(self.get_parameter('robots').value),
                            int(self.get_parameter('rosbridge_port').value))
        shown = [r for r in str(self.get_parameter('routes').value).split(',') if r] or cfg['robots']
        cfg['routes'] = {k: v for k, v in cfg['routes'].items() if k in shown}
        config = json.dumps(cfg, ensure_ascii=False).encode('utf-8')
        web_dir = os.path.join(get_package_share_directory('perimeter_defense_dashboard'), 'web')
        maps_dir = os.path.join(get_package_share_directory('perimeter_defense_world'), 'maps')
        handler = partial(_Handler, web_dir=web_dir, maps_dir=maps_dir, config=config)
        self.httpd: Optional[ThreadingHTTPServer] = ThreadingHTTPServer(('0.0.0.0', port), handler)
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()
        self.get_logger().info(f'C2 dashboard: http://localhost:{port}')

    def destroy_node(self) -> bool:
        if self.httpd is not None:
            self.httpd.shutdown()
            self.httpd = None
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardServer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
