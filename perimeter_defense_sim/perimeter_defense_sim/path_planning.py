"""A* on the base occupancy grid, used to script plausible actor walking paths."""

from __future__ import annotations

import heapq
import math
from typing import List, Optional, Sequence, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt

from perimeter_defense_world.generate_map import rasterize
from perimeter_defense_world.geometry import BaseLayout

XY = Tuple[float, float]


class GridPlanner:
    """8-connected A* on the navigation map with an obstacle clearance."""

    def __init__(self, layout: BaseLayout, clearance: float = 0.45, resolution: float = 0.1) -> None:
        grid = rasterize(layout, resolution, solid_fence=True)
        self.res = resolution
        self.free = distance_transform_edt(~grid.occupied) * resolution > clearance
        self.rows, self.cols = self.free.shape

    def _cell(self, p: XY) -> Tuple[int, int]:
        return (min(max(int(p[1] / self.res), 0), self.rows - 1),
                min(max(int(p[0] / self.res), 0), self.cols - 1))

    def _nearest_free(self, cell: Tuple[int, int]) -> Tuple[int, int]:
        if self.free[cell]:
            return cell
        rr, cc = np.nonzero(self.free)
        k = int(np.argmin((rr - cell[0]) ** 2 + (cc - cell[1]) ** 2))
        return int(rr[k]), int(cc[k])

    def plan(self, start: XY, goal: XY) -> Optional[List[XY]]:
        s, g = self._nearest_free(self._cell(start)), self._nearest_free(self._cell(goal))
        steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                 (1, 1, math.sqrt(2))]
        h = lambda c: math.hypot(c[0] - g[0], c[1] - g[1])   # noqa: E731
        open_set = [(h(s), 0.0, s)]
        came = {s: None}
        cost = {s: 0.0}
        while open_set:
            _, gc, cur = heapq.heappop(open_set)
            if cur == g:
                break
            if gc > cost[cur]:
                continue
            for dr, dc, w in steps:
                n = (cur[0] + dr, cur[1] + dc)
                if not (0 <= n[0] < self.rows and 0 <= n[1] < self.cols) or not self.free[n]:
                    continue
                nc = gc + w
                if nc < cost.get(n, math.inf):
                    cost[n], came[n] = nc, cur
                    heapq.heappush(open_set, (nc + h(n), nc, n))
        if g not in came:
            return None
        cells = []
        c = g
        while c is not None:
            cells.append(c)
            c = came[c]
        cells.reverse()
        pts = [((cc + 0.5) * self.res, (rr + 0.5) * self.res) for rr, cc in cells]
        return [start] + simplify(pts[1:-1], 0.15) + [goal]


def simplify(points: Sequence[XY], tolerance: float) -> List[XY]:
    """Ramer-Douglas-Peucker polyline simplification."""
    if len(points) < 3:
        return list(points)
    (x0, y0), (x1, y1) = points[0], points[-1]
    dx, dy = x1 - x0, y1 - y0
    norm = math.hypot(dx, dy) or 1e-9
    dists = [abs(dy * (x - x0) - dx * (y - y0)) / norm for x, y in points[1:-1]]
    i = int(np.argmax(dists)) + 1
    if dists[i - 1] > tolerance:
        return simplify(points[:i + 1], tolerance)[:-1] + simplify(points[i:], tolerance)
    return [points[0], points[-1]]


def plan_route(planner: GridPlanner, waypoints: Sequence[XY]) -> List[XY]:
    """Concatenate A* paths through all waypoints."""
    route: List[XY] = [waypoints[0]]
    for a, b in zip(waypoints, waypoints[1:]):
        leg = planner.plan(a, b)
        if leg is None:
            raise RuntimeError(f'no path from {a} to {b}')
        route += leg[1:]
    return route
