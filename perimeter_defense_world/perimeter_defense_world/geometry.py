"""Base geometry loaded from zones.yaml.

Everything that needs the layout of the base (world generator, map generator,
markers, dashboard export) goes through this module, so there is exactly one
interpretation of the YAML file.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import yaml

Point = Tuple[float, float]


@dataclass(frozen=True)
class Rect:
    """Axis-aligned rectangle with a height (for 3D generation)."""

    name: str
    min: Point
    max: Point
    height: float
    kind: str = ''
    label: str = ''

    @property
    def center(self) -> Point:
        return ((self.min[0] + self.max[0]) / 2.0, (self.min[1] + self.max[1]) / 2.0)

    @property
    def size(self) -> Point:
        return (self.max[0] - self.min[0], self.max[1] - self.min[1])

    def contains(self, p: Point, margin: float = 0.0) -> bool:
        return (self.min[0] - margin <= p[0] <= self.max[0] + margin
                and self.min[1] - margin <= p[1] <= self.max[1] + margin)


@dataclass(frozen=True)
class Segment:
    """Straight wall segment from a to b."""

    a: Point
    b: Point


@dataclass(frozen=True)
class Breach:
    name: str
    side: str
    start: float
    end: float

    def center(self, fence_min: Point, fence_max: Point) -> Point:
        mid = (self.start + self.end) / 2.0
        if self.side == 'north':
            return (mid, fence_max[1])
        if self.side == 'south':
            return (mid, fence_min[1])
        if self.side == 'west':
            return (fence_min[0], mid)
        return (fence_max[0], mid)


@dataclass
class BaseLayout:
    width: float
    height: float
    fence_min: Point
    fence_max: Point
    fence_height: float
    fence_thickness: float
    post_spacing: float
    post_size: float
    breaches: List[Breach]
    buildings: List[Rect]
    props: List[Rect]
    roads: List[List[Point]]
    road_width: float
    objective_target: Point
    objective_radius: float
    authorized_zones: Dict[str, List[Point]]
    patrol_routes: Dict[str, List[Point]]
    intruder_spawns: Dict[str, Point]
    helipad: Optional[Tuple[Point, float]] = None
    name: str = ''
    unit: str = ''
    code: str = ''
    surroundings: dict = field(default_factory=dict, repr=False)
    render_min: Point = (0.0, 0.0)
    render_max: Point = (0.0, 0.0)
    render_px_per_m: float = 30.0
    raw: dict = field(default_factory=dict, repr=False)

    def obstacles(self) -> List[Rect]:
        """All static solid objects (buildings and props)."""
        return list(self.buildings) + list(self.props)

    def fence_segments(self) -> List[Segment]:
        """Fence wall segments with the breaches cut out."""
        (x0, y0), (x1, y1) = self.fence_min, self.fence_max
        sides = {
            'south': ((x0, y0), (x1, y0)),
            'east': ((x1, y0), (x1, y1)),
            'north': ((x0, y1), (x1, y1)),
            'west': ((x0, y0), (x0, y1)),
        }
        segments: List[Segment] = []
        for side, (a, b) in sides.items():
            horizontal = a[1] == b[1]
            lo, hi = (a[0], b[0]) if horizontal else (a[1], b[1])
            gaps = sorted((br.start, br.end) for br in self.breaches if br.side == side)
            cursor = lo
            for g0, g1 in gaps:
                if g0 > cursor:
                    segments.append(_side_segment(horizontal, a, cursor, g0))
                cursor = max(cursor, g1)
            if cursor < hi:
                segments.append(_side_segment(horizontal, a, cursor, hi))
        return segments

    def fence_posts(self) -> List[Point]:
        """Fence post positions: regular spacing along each segment, one at each end."""
        posts: List[Point] = []
        for seg in self.fence_segments():
            (ax, ay), (bx, by) = seg.a, seg.b
            n = max(1, int(math.ceil(math.hypot(bx - ax, by - ay) / self.post_spacing)))
            posts += [(ax + (bx - ax) * k / n, ay + (by - ay) * k / n) for k in range(n + 1)]
        return posts

    def breach_by_name(self, name: str) -> Optional[Breach]:
        for br in self.breaches:
            if br.name == name:
                return br
        return None

    def is_inside_fence(self, p: Point) -> bool:
        return (self.fence_min[0] < p[0] < self.fence_max[0]
                and self.fence_min[1] < p[1] < self.fence_max[1])


def _side_segment(horizontal: bool, anchor: Point, s0: float, s1: float) -> Segment:
    if horizontal:
        return Segment((s0, anchor[1]), (s1, anchor[1]))
    return Segment((anchor[0], s0), (anchor[0], s1))


def _pt(v: Sequence[float]) -> Point:
    return (float(v[0]), float(v[1]))


def _rect(d: dict, kind_key: str) -> Rect:
    return Rect(
        name=str(d['name']),
        min=_pt(d['min']),
        max=_pt(d['max']),
        height=float(d['height']),
        kind=str(d.get(kind_key, '')),
        label=str(d.get('label', d['name'])),
    )


def _helipad(raw: dict) -> Optional[Tuple[Point, float]]:
    pad = raw.get('markings', {}).get('helipad')
    return (_pt(pad['center']), float(pad['radius'])) if pad else None


def package_dir() -> str:
    """Share directory of the installed package, or the source tree outside ROS."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('perimeter_defense_world')
    except Exception:  # noqa: BLE001 - fallback for use outside a ROS install
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def default_zones_path() -> str:
    return os.path.join(package_dir(), 'config', 'zones.yaml')


def load_layout(path: Optional[str] = None) -> BaseLayout:
    """Parse zones.yaml into a BaseLayout."""
    path = path or default_zones_path()
    with open(path, 'r', encoding='utf-8') as f:
        raw = yaml.safe_load(f)

    fence = raw['fence']
    return BaseLayout(
        width=float(raw['area']['width']),
        height=float(raw['area']['height']),
        fence_min=_pt(fence['min']),
        fence_max=_pt(fence['max']),
        fence_height=float(fence['height']),
        fence_thickness=float(fence['thickness']),
        post_spacing=float(fence['post_spacing']),
        post_size=float(fence['post_size']),
        breaches=[Breach(str(b['name']), str(b['side']), float(b['from']), float(b['to']))
                  for b in fence.get('breaches', [])],
        buildings=[_rect(b, 'style') for b in raw.get('buildings', [])],
        props=[_rect(p, 'type') for p in raw.get('props', [])],
        roads=[[_pt(p) for p in line] for line in raw['roads']['polylines']],
        road_width=float(raw['roads']['width']),
        objective_target=_pt(raw['objective']['target']),
        objective_radius=float(raw['objective']['radius']),
        authorized_zones={z['name']: [_pt(p) for p in z['polygon']]
                          for z in raw.get('authorized_zones', [])},
        patrol_routes={k: [_pt(p) for p in v] for k, v in raw['patrol_routes'].items()},
        intruder_spawns={k: _pt(v) for k, v in raw.get('intruder_spawns', {}).items()},
        helipad=_helipad(raw),
        name=str(raw.get('base', {}).get('name', '')),
        unit=str(raw.get('base', {}).get('unit', '')),
        code=str(raw.get('base', {}).get('code', '')),
        surroundings=raw.get('surroundings', {}),
        render_min=_pt(raw.get('render', {}).get('bounds', {}).get('min', [0.0, 0.0])),
        render_max=_pt(raw.get('render', {}).get('bounds', {}).get(
            'max', [raw['area']['width'], raw['area']['height']])),
        render_px_per_m=float(raw.get('render', {}).get('pixels_per_metre', 30.0)),
        raw=raw,
    )


def point_in_polygon(p: Point, polygon: Sequence[Point]) -> bool:
    """Ray casting point-in-polygon test."""
    x, y = p
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_cross = x1 + (y - y1) * (x2 - x1) / (y2 - y1)
            if x < x_cross:
                inside = not inside
    return inside
