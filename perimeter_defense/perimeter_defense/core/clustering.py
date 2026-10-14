"""Person detection from a 2D lidar scan: static map filter, clustering, size gate."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

import numpy as np

XY = Tuple[float, float]


@dataclass
class Cluster:
    points: np.ndarray        # N x 2, map frame
    centroid: XY
    width: float              # largest distance between two points of the cluster
    range: float              # distance from the sensor to the centroid


@dataclass
class DetectorParams:
    detection_radius: float = 10.0
    min_range: float = 0.3
    cluster_tolerance: float = 0.35     # max gap between neighbouring returns
    min_points: int = 2
    min_width: float = 0.15             # a person seen side-on is 0.15-0.25 m deep
    max_width: float = 0.9
    static_margin: float = 0.35         # returns this close to the static map are walls ...
    static_margin_per_m: float = 0.035  # ... plus this much per metre of range (heading error)
    robot_exclusion: float = 0.6        # returns this close to an own robot are that robot
    centroid_offset: float = 0.1        # lidar sees the near surface, shift to body centre


def scan_to_points(ranges: Sequence[float], angle_min: float, angle_increment: float,
                   range_min: float, range_max: float) -> Tuple[np.ndarray, np.ndarray]:
    """Valid returns as sensor-frame points, in scan order, plus their ranges."""
    r = np.asarray(ranges, dtype=float)
    ang = angle_min + angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > range_min) & (r < range_max)
    return np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])], axis=1), r[ok]


def transform_points(points: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return points @ rot.T + np.array([x, y])


def cluster_ordered(points: np.ndarray, tolerance: float) -> List[np.ndarray]:
    """Split points ordered by scan angle wherever two neighbours are far apart.

    The first and last clusters are merged when the scan wraps around.
    """
    if len(points) == 0:
        return []
    gaps = np.linalg.norm(np.diff(points, axis=0), axis=1) > tolerance
    splits = np.nonzero(gaps)[0] + 1
    clusters = np.split(points, splits)
    if len(clusters) > 1 and np.linalg.norm(points[0] - points[-1]) <= tolerance:
        clusters[0] = np.vstack([clusters[-1], clusters[0]])
        clusters.pop()
    return clusters


def cluster_width(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    d = points[:, None, :] - points[None, :, :]
    return float(np.sqrt(np.max(np.sum(d * d, axis=2))))


def detect_people(points_map: np.ndarray, sensor_xy: XY, params: DetectorParams,
                  is_static: Callable[[np.ndarray, np.ndarray], np.ndarray],
                  robots_xy: Optional[Sequence[XY]] = None) -> List[Cluster]:
    """Person-sized clusters of lidar returns that are not on the static map.

    points_map: N x 2 returns in scan order, already in the map frame.
    is_static: vectorised test (points, margins) returning True for returns
        explained by the map; the margin grows with range because a small
        heading error of the localization moves far walls more.
    robots_xy: positions of own robots, whose returns are removed.
    """
    if len(points_map) == 0:
        return []
    sx, sy = sensor_xy
    dist = np.hypot(points_map[:, 0] - sx, points_map[:, 1] - sy)
    margins = params.static_margin + params.static_margin_per_m * dist
    keep = (dist <= params.detection_radius) & ~is_static(points_map, margins)
    for rx, ry in robots_xy or []:
        keep &= np.hypot(points_map[:, 0] - rx, points_map[:, 1] - ry) > params.robot_exclusion
    pts = points_map[keep]

    out: List[Cluster] = []
    for cl in cluster_ordered(pts, params.cluster_tolerance):
        if len(cl) < params.min_points:
            continue
        width = cluster_width(cl)
        if not params.min_width <= width <= params.max_width:
            continue
        cx, cy = float(np.mean(cl[:, 0])), float(np.mean(cl[:, 1]))
        rng = math.hypot(cx - sx, cy - sy)
        if rng > 1e-6 and params.centroid_offset > 0.0:
            cx += (cx - sx) / rng * params.centroid_offset
            cy += (cy - sy) / rng * params.centroid_offset
        out.append(Cluster(points=cl, centroid=(cx, cy), width=width, range=rng))
    return out


def fuse_detections(dets: Sequence[Tuple[XY, float, str]], radius: float
                    ) -> List[Tuple[XY, float, List[str]]]:
    """Merge detections of the same person seen by several robots.

    dets: (position, width, source). Detections closer than radius are
    averaged (greedy single linkage, the groups are small).
    """
    groups: List[List[int]] = []
    for i, (p, _, _) in enumerate(dets):
        for g in groups:
            if any(math.hypot(p[0] - dets[j][0][0], p[1] - dets[j][0][1]) <= radius for j in g):
                g.append(i)
                break
        else:
            groups.append([i])
    fused = []
    for g in groups:
        xs = [dets[i][0][0] for i in g]
        ys = [dets[i][0][1] for i in g]
        fused.append(((sum(xs) / len(g), sum(ys) / len(g)), max(dets[i][1] for i in g),
                      sorted({dets[i][2] for i in g})))
    return fused
