"""RViz markers for tracks: uncertainty ellipse, velocity, predicted path, label."""

from __future__ import annotations

import math
from typing import Dict, List, Sequence, Tuple

import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from perimeter_defense_msgs.msg import Track

# Affiliation colours (C2 convention): hostile red, unknown yellow, friend blue.
CLASS_COLOR = {
    Track.CLASS_HOSTILE: (1.0, 0.2, 0.15),
    Track.CLASS_UNKNOWN: (1.0, 0.85, 0.1),
    Track.CLASS_FRIEND: (0.25, 0.6, 1.0),
}
CLASS_TAG = {Track.CLASS_HOSTILE: 'HOS', Track.CLASS_UNKNOWN: 'UNK', Track.CLASS_FRIEND: 'FRD'}
STATE_TAG = {Track.STATE_ALERT: 'ALERT', Track.STATE_TRACKED: '', Track.STATE_LOST: 'LOST',
             Track.STATE_NEUTRALIZED: 'NEUTR'}


def _rgba(rgb: Tuple[float, float, float], a: float) -> ColorRGBA:
    return ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=a)


def _marker(ns: str, mid: int, mtype: int, stamp, frame: str) -> Marker:
    m = Marker()
    m.header.frame_id = frame
    m.header.stamp = stamp
    m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
    m.pose.orientation.w = 1.0
    m.lifetime.nanosec = 500_000_000
    return m


def covariance_ellipse(cov2: np.ndarray, n_sigma: float = 2.45) -> Tuple[float, float, float]:
    """Axes (full lengths) and orientation of the n-sigma ellipse (2.45 sigma ~ 95 %)."""
    vals, vecs = np.linalg.eigh(cov2)
    vals = np.maximum(vals, 1e-6)
    yaw = math.atan2(vecs[1, 1], vecs[0, 1])
    return 2 * n_sigma * math.sqrt(vals[1]), 2 * n_sigma * math.sqrt(vals[0]), yaw


def track_markers(track: Track, path: Sequence[Tuple[float, float]], stamp, frame: str = 'map'
                  ) -> List[Marker]:
    tid = int(track.id)
    rgb = CLASS_COLOR.get(track.classification, CLASS_COLOR[Track.CLASS_UNKNOWN])
    if track.state in (Track.STATE_LOST, Track.STATE_NEUTRALIZED):
        rgb = (0.6, 0.6, 0.6)
    x, y = track.position.x, track.position.y
    cov = np.array(track.covariance).reshape(4, 4)[:2, :2]
    out: List[Marker] = []

    # Uncertainty ellipse (flat cylinder), dashed look for fence-only alerts.
    a, b, yaw = covariance_ellipse(cov)
    ell = _marker('track_ellipse', tid, Marker.CYLINDER, stamp, frame)
    ell.pose.position.x, ell.pose.position.y, ell.pose.position.z = x, y, 0.05
    ell.pose.orientation.z, ell.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    ell.scale.x, ell.scale.y, ell.scale.z = max(a, 0.3), max(b, 0.3), 0.02
    ell.color = _rgba(rgb, 0.18 if track.state == Track.STATE_ALERT else 0.3)
    out.append(ell)

    # Target symbol.
    dot = _marker('track_dot', tid, Marker.CYLINDER, stamp, frame)
    dot.pose.position.x, dot.pose.position.y, dot.pose.position.z = x, y, 0.1
    dot.scale.x = dot.scale.y = 0.6
    dot.scale.z = 0.05
    dot.color = _rgba(rgb, 0.95)
    out.append(dot)

    # Velocity (1 s ahead).
    vx, vy = track.velocity.x, track.velocity.y
    if track.state != Track.STATE_ALERT and math.hypot(vx, vy) > 0.1:
        arr = _marker('track_velocity', tid, Marker.ARROW, stamp, frame)
        arr.points = [Point(x=x, y=y, z=0.15), Point(x=x + vx, y=y + vy, z=0.15)]
        arr.scale.x, arr.scale.y, arr.scale.z = 0.12, 0.3, 0.3
        arr.color = _rgba(rgb, 1.0)
        out.append(arr)

    # Predicted path, drawn as dots every step.
    if len(path) > 1 and track.state in (Track.STATE_TRACKED, Track.STATE_LOST):
        pts = _marker('track_prediction', tid, Marker.SPHERE_LIST, stamp, frame)
        pts.points = [Point(x=px, y=py, z=0.1) for px, py in path[1:]]
        pts.scale.x = pts.scale.y = pts.scale.z = 0.15
        pts.color = _rgba(rgb, 0.7)
        out.append(pts)

    label = _marker('track_label', tid, Marker.TEXT_VIEW_FACING, stamp, frame)
    label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y + 1.0, 0.5
    label.scale.z = 0.7
    label.color = _rgba(rgb, 1.0)
    tag = ' '.join(t for t in (f'T{tid}', CLASS_TAG.get(track.classification, 'UNK'),
                               STATE_TAG.get(track.state, '')) if t)
    label.text = f'{tag} {math.hypot(vx, vy):.1f} m/s'
    out.append(label)
    return out


def all_track_markers(tracks: Sequence[Track], paths: Dict[int, Sequence[Tuple[float, float]]],
                      stamp, frame: str = 'map') -> List[Marker]:
    out: List[Marker] = []
    for tr in tracks:
        out += track_markers(tr, paths.get(int(tr.id), []), stamp, frame)
    return out
