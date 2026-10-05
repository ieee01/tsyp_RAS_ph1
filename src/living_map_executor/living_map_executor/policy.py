"""How the Executor moves: its own logic, built only from inherited memories.

The command post says WHO goes and WHAT the objective is. Everything here is
the Executor's own HOW:

* every recorded hazard is avoided, whatever its age - a stale fire is still a
  fire - and becomes a hard keep-out disc in the Nav2 planning map;
* beacon-to-beacon links the Writer actually drove (previous/next) are trusted
  more than links inferred from proximity;
* fresher memories are trusted more than ageing ones.
"""
from __future__ import annotations

import math
from typing import Iterable, Sequence

Point = tuple[float, float]

DEFAULT_AVOID_TYPES = frozenset({2, 3, 4})  # GAS_HAZARD, BLOCKED_PATH, FIRE
# Hard keep-out radius stamped into the planning map, per hazard type (metres).
KEEPOUT_M = {2: 2.0, 3: 0.7, 4: 1.8}
FRESHNESS_COST = {"FRESH": 1.0, "AGING": 1.15, "STALE": 1.4}
INFERRED_EDGE_COST = 1.25


def avoid_types(requested: Iterable[int]) -> frozenset[int]:
    """Hazard policy: the robot's own default unless a brief adds more types."""
    return DEFAULT_AVOID_TYPES | frozenset(int(t) for t in requested)


def keepout_radius(event_type: int) -> float:
    return KEEPOUT_M.get(int(event_type), 1.5)


def freshness_cost(freshness: str) -> float:
    return FRESHNESS_COST.get(freshness, 1.0)


def stamp_keepout(data: Sequence[int], width: int, height: int, resolution: float, origin: Point,
                  hazards: Iterable[tuple[float, float, float]],
                  protect: Iterable[tuple[float, float, float]] = ()) -> list[int]:
    """Mark discs around hazard points as lethal (100), except protected discs.

    ``hazards`` and ``protect`` are (x, y, radius) tuples in the map frame.
    Protection keeps the mission target approachable even when it lies near a
    recorded hazard.
    """
    result = list(data)
    protected = list(protect)
    for hx, hy, radius in hazards:
        r0 = max(0, math.floor((hy - radius - origin[1]) / resolution))
        r1 = min(height, math.ceil((hy + radius - origin[1]) / resolution))
        c0 = max(0, math.floor((hx - radius - origin[0]) / resolution))
        c1 = min(width, math.ceil((hx + radius - origin[0]) / resolution))
        for row in range(r0, r1):
            cy = origin[1] + (row + 0.5) * resolution
            for column in range(c0, c1):
                cx = origin[0] + (column + 0.5) * resolution
                if (cx - hx) ** 2 + (cy - hy) ** 2 > radius * radius:
                    continue
                if any((cx - px) ** 2 + (cy - py) ** 2 <= pr * pr for px, py, pr in protected):
                    continue
                result[row * width + column] = 100
    return result


def edge_cost(distance: float, chained: bool, freshness_a: str, freshness_b: str) -> float:
    """Driven chain links cost their length; inferred links cost more; age adds cost."""
    trust = 1.0 if chained else INFERRED_EDGE_COST
    return distance * trust * max(freshness_cost(freshness_a), freshness_cost(freshness_b))
