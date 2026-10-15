"""Interception point of a moving target (predictive mode).

Given the predicted path of a target, q(t) = p + v t for t = 0, dt, ...,
horizon (the tracker stops it at obstacles and at the objective), the
interception point is the first predicted point the robot can reach no
later than the target: driving_distance(robot, q(t)) / robot_speed + margin
<= t. If no point qualifies within the horizon, the robot is sent to the
last predicted point (the farthest it can still cut off, usually where the
target is heading to), with the time it will take.

For an alert without lidar contact (fence or spot report) there is no
velocity; the target is assumed to walk from the alert position to the
objective along the shortest path, starting at the time of the alert
(resample_along), and the same interception rule applies.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

XY = Tuple[float, float]


@dataclass
class Interception:
    point: XY
    t: float           # s, time at which the target reaches the point
    eta: float         # s, time the robot needs to reach the point
    feasible: bool     # robot arrives first


def intercept_point(path: Sequence[XY], dt: float, distance: Callable[[XY], float],
                    speed: float, margin: float = 0.0) -> Optional[Interception]:
    """path: predicted target positions every dt seconds from now.
    distance(q): driving distance from the robot to q."""
    if not path or speed <= 0.0:
        return None
    last = None
    for k, q in enumerate(path):
        t = k * dt
        eta = distance(q) / speed
        if eta + margin <= t:
            return Interception(q, t, eta, True)
        last = Interception(q, t, eta, False)
    return last


def resample_along(path: Sequence[XY], speed: float, dt: float, start_time: float,
                   horizon: float) -> List[XY]:
    """Positions every dt seconds of someone walking along path at speed, who
    started start_time seconds ago: s(t) = speed * (start_time + t), t = 0..horizon.
    Stops at the end of the path."""
    if len(path) < 2:
        return list(path)
    cum = [0.0]
    for a, b in zip(path, path[1:]):
        cum.append(cum[-1] + math.dist(a, b))
    out: List[XY] = []
    n = int(round(horizon / dt))
    seg = 0
    for k in range(n + 1):
        s = speed * (start_time + k * dt)
        if s >= cum[-1]:
            out.append(path[-1])
            break
        while cum[seg + 1] < s:
            seg += 1
        a, b = path[seg], path[seg + 1]
        f = (s - cum[seg]) / max(cum[seg + 1] - cum[seg], 1e-9)
        out.append((a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f))
    return out
