"""Small helpers to build visualization_msgs markers."""

from __future__ import annotations

import math
from typing import Iterable, List, Sequence, Tuple

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

XY = Tuple[float, float]


def color(rgba: Sequence[float]) -> ColorRGBA:
    return ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))


def point(x: float, y: float, z: float = 0.0) -> Point:
    return Point(x=float(x), y=float(y), z=float(z))


def base_marker(ns: str, mid: int, mtype: int, frame: str = 'map') -> Marker:
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns
    m.id = mid
    m.type = mtype
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = 1.0
    return m


def line_strip(ns: str, mid: int, pts: Iterable[XY], rgba: Sequence[float], width: float,
               z: float = 0.05, closed: bool = False) -> Marker:
    m = base_marker(ns, mid, Marker.LINE_STRIP)
    m.scale.x = width
    m.color = color(rgba)
    pts = list(pts)
    if closed and pts:
        pts.append(pts[0])
    m.points = [point(x, y, z) for x, y in pts]
    return m


def line_list(ns: str, mid: int, segments: Iterable[Tuple[XY, XY]], rgba: Sequence[float],
              width: float, z: float = 0.05) -> Marker:
    m = base_marker(ns, mid, Marker.LINE_LIST)
    m.scale.x = width
    m.color = color(rgba)
    for a, b in segments:
        m.points += [point(a[0], a[1], z), point(b[0], b[1], z)]
    return m


def text(ns: str, mid: int, x: float, y: float, label: str, rgba: Sequence[float],
         height: float, z: float = 0.5) -> Marker:
    m = base_marker(ns, mid, Marker.TEXT_VIEW_FACING)
    m.pose.position = point(x, y, z)
    m.scale.z = height
    m.color = color(rgba)
    m.text = label
    return m


def circle(cx: float, cy: float, r: float, n: int = 72) -> List[XY]:
    return [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n))
            for k in range(n)]


def dashed(pts: Sequence[XY], dash: float, gap: float, closed: bool = False) -> List[Tuple[XY, XY]]:
    """Split a polyline into dash segments for a LINE_LIST marker."""
    pts = list(pts) + ([pts[0]] if closed and pts else [])
    out: List[Tuple[XY, XY]] = []
    on, left = True, dash
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        s = 0.0
        while s < seg - 1e-9:
            step = min(left, seg - s)
            if on:
                a = (x0 + (x1 - x0) * s / seg, y0 + (y1 - y0) * s / seg)
                b = (x0 + (x1 - x0) * (s + step) / seg, y0 + (y1 - y0) * (s + step) / seg)
                out.append((a, b))
            s += step
            left -= step
            if left <= 1e-9:
                on = not on
                left = dash if on else gap
    return out
