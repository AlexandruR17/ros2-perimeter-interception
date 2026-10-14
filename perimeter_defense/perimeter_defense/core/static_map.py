"""Static occupancy map queries (walls, buildings, fence) for the detector and the predictor."""

from __future__ import annotations

from typing import Sequence, Tuple

import numpy as np
from scipy.ndimage import distance_transform_edt


class StaticMap:
    """Occupancy grid with a distance-to-obstacle field.

    data: row-major cells as in nav_msgs/OccupancyGrid (row 0 at origin_y),
    values > occupied_threshold are obstacles. Cells outside the grid are free.
    """

    def __init__(self, data: Sequence[int], width: int, height: int, resolution: float,
                 origin: Tuple[float, float], occupied_threshold: int = 50) -> None:
        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        self.occupied = grid > occupied_threshold
        self.resolution = float(resolution)
        self.origin = (float(origin[0]), float(origin[1]))
        self.width, self.height = width, height
        self.distance = distance_transform_edt(~self.occupied) * self.resolution

    def _cells(self, pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        pts = np.atleast_2d(np.asarray(pts, dtype=float))
        c = np.floor((pts[:, 0] - self.origin[0]) / self.resolution).astype(int)
        r = np.floor((pts[:, 1] - self.origin[1]) / self.resolution).astype(int)
        inside = (c >= 0) & (c < self.width) & (r >= 0) & (r < self.height)
        return r, c, inside

    def distance_to_obstacle(self, pts: np.ndarray) -> np.ndarray:
        """Distance (m) from each point to the nearest obstacle; inf outside the map."""
        r, c, inside = self._cells(pts)
        out = np.full(len(r), np.inf)
        out[inside] = self.distance[r[inside], c[inside]]
        return out

    def is_static(self, pts: np.ndarray, margin) -> np.ndarray:
        """True where a point is within margin (scalar or per point) of an obstacle."""
        return self.distance_to_obstacle(pts) < margin

    def is_occupied(self, x: float, y: float) -> bool:
        r, c, inside = self._cells(np.array([[x, y]]))
        return bool(inside[0] and self.occupied[r[0], c[0]])
