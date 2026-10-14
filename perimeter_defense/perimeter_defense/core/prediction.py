"""Constant velocity trajectory prediction with stops at obstacles and at the objective."""

from __future__ import annotations

import math
from typing import Callable, List, Optional, Tuple

XY = Tuple[float, float]


def predict_path(position: XY, velocity: XY, horizon: float, step: float,
                 blocked: Optional[Callable[[float, float], bool]] = None,
                 objective: Optional[XY] = None, objective_radius: float = 0.0) -> List[XY]:
    """Positions q(t) = p + v t for t = 0, step, ..., horizon.

    Extrapolation stops before entering an obstacle (blocked(x, y) is True)
    and once the path is inside the objective circle, where the target is
    expected to stop.
    """
    (px, py), (vx, vy) = position, velocity
    out: List[XY] = []
    n = int(round(horizon / step))
    for k in range(n + 1):
        t = k * step
        x, y = px + vx * t, py + vy * t
        if k > 0 and blocked is not None and blocked(x, y):
            break
        out.append((x, y))
        if objective is not None and math.hypot(x - objective[0], y - objective[1]) <= objective_radius:
            break
    return out
