"""Hand-over of a captured person from an interceptor to a transport UGV (M4).

The interceptor that captured a person holds him until a transport UGV comes,
takes him over and escorts him to the detention building; the interceptor
then returns to the defense at once instead of spending minutes on the escort.
"""

from __future__ import annotations

import math
from typing import Callable, Dict, Optional, Sequence, Tuple

XY = Tuple[float, float]


def pickup_point(detainee: XY, destination: XY, standoff: float) -> XY:
    """Where the transport stops: standoff metres from the detainee on the side of the
    destination, so that it can lead him there without turning round him."""
    dx, dy = destination[0] - detainee[0], destination[1] - detainee[1]
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return detainee
    k = min(standoff, d) / d
    return (detainee[0] + dx * k, detainee[1] + dy * k)


def choose_transport(available: Sequence[str], time_to: Callable[[str], float]) -> Optional[str]:
    """The available transport that reaches the detainee first (None if none can)."""
    best: Dict[str, float] = {t: time_to(t) for t in available}
    best = {t: v for t, v in best.items() if math.isfinite(v)}
    return min(best, key=best.get) if best else None
