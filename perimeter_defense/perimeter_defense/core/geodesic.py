"""Driving distance of a robot on the base map (geodesic distance).

The interception point needs the distance a robot actually has to drive to
reach a point, around buildings, not the straight line. The occupancy grid
of the navigation map is coarsened, obstacles are grown by the robot
radius, and one Dijkstra run from the robot gives the driving distance to
every cell; querying many points of a predicted path is then a lookup.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

XY = Tuple[float, float]


class GeodesicGrid:
    def __init__(self, occupied: np.ndarray, resolution: float, origin: XY,
                 cell: float = 0.25, inflation: float = 0.35,
                 bounds: Optional[Tuple[XY, XY]] = None) -> None:
        """occupied: rows x cols bool (row 0 at origin y) at the map resolution.

        cell: resolution of the coarse grid; inflation: robot radius;
        bounds: (min, max) corners outside which robots may not drive.
        """
        k = max(1, int(round(cell / resolution)))
        rows, cols = occupied.shape[0] // k, occupied.shape[1] // k
        coarse = occupied[:rows * k, :cols * k].reshape(rows, k, cols, k).any(axis=(1, 3))
        self.res = resolution * k
        self.origin = (float(origin[0]), float(origin[1]))
        self.rows, self.cols = rows, cols
        free = distance_transform_edt(~coarse) * self.res > inflation
        if bounds is not None:
            (x0, y0), (x1, y1) = bounds
            ys = self.origin[1] + (np.arange(rows) + 0.5) * self.res
            xs = self.origin[0] + (np.arange(cols) + 0.5) * self.res
            free &= ((ys >= y0) & (ys <= y1))[:, None] & ((xs >= x0) & (xs <= x1))[None, :]
        self.free = free
        self.graph = self._graph(free)

    def _graph(self, free: np.ndarray) -> csr_matrix:
        """8-connected graph of the free cells, without cutting obstacle corners."""
        nr, nc = free.shape
        idx = np.arange(free.size).reshape(free.shape)
        rows, cols, weights = [], [], []
        for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):
            r_lo, r_hi = max(0, -dr), nr - max(0, dr)
            c_lo, c_hi = max(0, -dc), nc - max(0, dc)
            shifted = lambda a, sr, sc: a[r_lo + sr:r_hi + sr, c_lo + sc:c_hi + sc]   # noqa: E731
            ok = shifted(free, 0, 0) & shifted(free, dr, dc)
            if dr and dc:
                ok &= shifted(free, dr, 0) & shifted(free, 0, dc)
            w = self.res * (math.sqrt(2.0) if dr and dc else 1.0)
            rows.append(shifted(idx, 0, 0)[ok])
            cols.append(shifted(idx, dr, dc)[ok])
            weights.append(np.full(int(ok.sum()), w))
        r, c, w = np.concatenate(rows), np.concatenate(cols), np.concatenate(weights)
        n = free.size
        return csr_matrix((np.concatenate([w, w]), (np.concatenate([r, c]), np.concatenate([c, r]))),
                          shape=(n, n))

    def cell_of(self, p: XY) -> Optional[Tuple[int, int]]:
        c = int(math.floor((p[0] - self.origin[0]) / self.res))
        r = int(math.floor((p[1] - self.origin[1]) / self.res))
        if 0 <= r < self.rows and 0 <= c < self.cols:
            return r, c
        return None

    def _nearest_free(self, p: XY, radius: int = 6) -> Optional[Tuple[int, int]]:
        rc = self.cell_of(p)
        if rc is None:
            return None
        r0, c0 = rc
        best, best_d = None, math.inf
        for r in range(max(0, r0 - radius), min(self.rows, r0 + radius + 1)):
            for c in range(max(0, c0 - radius), min(self.cols, c0 + radius + 1)):
                if self.free[r, c]:
                    d = (r - r0) ** 2 + (c - c0) ** 2
                    if d < best_d:
                        best, best_d = (r, c), d
        return best

    def distances_from(self, start: XY) -> Optional[np.ndarray]:
        """Driving distance (m) from start to every cell (rows x cols), inf where unreachable."""
        rc = self._nearest_free(start)
        if rc is None:
            return None
        d = dijkstra(self.graph, directed=False, indices=rc[0] * self.cols + rc[1])
        return d.reshape(self.rows, self.cols)

    def path_tree(self, goal: XY) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Distances and predecessors of the shortest paths from every cell to goal."""
        rc = self._nearest_free(goal)
        if rc is None:
            return None
        d, pred = dijkstra(self.graph, directed=False, indices=rc[0] * self.cols + rc[1],
                           return_predecessors=True)
        return d, pred

    def path_to_goal(self, tree: Tuple[np.ndarray, np.ndarray], start: XY) -> List[XY]:
        """Shortest path from start to the goal of path_tree, as cell centres."""
        _, pred = tree
        rc = self._nearest_free(start)
        if rc is None:
            return []
        node = rc[0] * self.cols + rc[1]
        out: List[XY] = [start]
        for _ in range(self.rows * self.cols):
            if node < 0:
                break
            r, c = divmod(int(node), self.cols)
            out.append((self.origin[0] + (c + 0.5) * self.res, self.origin[1] + (r + 0.5) * self.res))
            node = pred[node]
        return out

    def lookup(self, field: np.ndarray, p: XY) -> float:
        """Distance of point p in a field from distances_from (nearest free cell if p is blocked)."""
        rc = self.cell_of(p)
        if rc is not None and self.free[rc]:
            return float(field[rc])
        rc = self._nearest_free(p)
        if rc is None:
            return math.inf
        cr = self.origin[0] + (rc[1] + 0.5) * self.res, self.origin[1] + (rc[0] + 0.5) * self.res
        return float(field[rc]) + math.dist(cr, p)
