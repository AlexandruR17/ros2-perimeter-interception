"""Soldier behaviour helpers (simulation side, common base).

A soldier on foot sees a person when the person is inside the fence, within
observe_range and in line of sight (buildings and props block the view, the
chain-link fence does not). After observe_time seconds of continuous sight he
radios a spot report with a coarse position, then repeats it every
report_period seconds while he still sees the person.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from perimeter_defense_world.geometry import BaseLayout

XY = Tuple[float, float]


@dataclass
class ObserverParams:
    observe_range: float = 12.0   # m
    observe_time: float = 1.0     # s of continuous sight before reporting
    report_period: float = 4.0    # s between repeated reports of the same person


@dataclass
class SightingLog:
    """Decides when a soldier radios a spot report about a person."""

    params: ObserverParams = field(default_factory=ObserverParams)
    seen_since: Dict[str, float] = field(default_factory=dict)
    last_report: Dict[str, float] = field(default_factory=dict)

    def update(self, visible: Dict[str, bool], now: float) -> List[str]:
        """visible: person -> in sight now. Returns the persons to report now."""
        out = []
        for person, seen in visible.items():
            if not seen:
                self.seen_since.pop(person, None)
                continue
            since = self.seen_since.setdefault(person, now)
            if (now - since >= self.params.observe_time
                    and now - self.last_report.get(person, -math.inf) >= self.params.report_period):
                self.last_report[person] = now
                out.append(person)
        return out


def can_see(layout: BaseLayout, observer: XY, target: XY, observe_range: float) -> bool:
    """Line of sight from a soldier to a person inside the fence."""
    return (layout.is_inside_fence(target)
            and math.dist(observer, target) <= observe_range
            and layout.line_of_sight(observer, target))


def loop_waypoints(route: Sequence[XY]) -> List[XY]:
    """Waypoints of a closed patrol loop; a two-point route is walked back and forth."""
    pts = list(route)
    return pts + [pts[0]]


def pursuit_step(pose: XY, path: Sequence[XY], seg: int, lookahead: float,
                 speed: float) -> Tuple[XY, int, bool]:
    """Velocity towards the next path point. Returns (velocity, segment, finished)."""
    while seg < len(path) - 2 and math.dist(pose, path[seg + 1]) < lookahead:
        seg += 1
    goal = path[min(seg + 1, len(path) - 1)]
    d = math.dist(pose, goal)
    if seg >= len(path) - 2 and d < 0.3:
        return (0.0, 0.0), seg, True
    v = min(speed, max(d * 2.0, 0.4))
    return ((goal[0] - pose[0]) / d * v, (goal[1] - pose[1]) / d * v), seg, False


def nearest_index(route: Sequence[XY], p: XY) -> int:
    return min(range(len(route)), key=lambda i: math.dist(route[i], p))


def pick_captive(captured: Dict[str, XY], near: XY, radius: float) -> Optional[str]:
    """Captured person closest to a reported position."""
    best = min(captured.items(), key=lambda kv: math.dist(kv[1], near), default=None)
    if best is None or math.dist(best[1], near) > radius:
        return None
    return best[0]
