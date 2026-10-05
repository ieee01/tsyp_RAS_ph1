"""Where the Writer drops beacons, and why.

The Writer carries a limited stock of beacons, so every drop must earn its
place. Event beacons are always dropped (a reserve is kept for them). The rest
of the stock is spent, in priority order, on:

1. ENTRANCE - the first beacon, anchoring the way out;
2. RELAY    - the Writer's link into the gateway-connected mesh is fading, so a
              beacon is dropped while the link still works (the chain never breaks);
3. JUNCTION - the SLAM map shows three or more openings around the Writer;
4. SPACING  - a breadcrumb fallback when no beacon is within ``spacing_m``.

Event beacons are dropped at the Writer's own position - a place a robot has
physically reached - with a guidance vector to the event (the video's option B:
"safe spot before it"). A hazard is described from outside it, never entered.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence

from .protocol import FLAG_ENTRANCE, FLAG_JUNCTION, FLAG_RELAY, FLAG_SPACING

Point = tuple[float, float]


def ring_openings(data: Sequence[int], width: int, height: int, resolution: float,
                  origin: Point, centre: Point, radius_m: float = 1.6, samples: int = 72,
                  occupied_threshold: int = 50, min_gap_m: float = 0.7) -> int:
    """Count wall-bounded openings on a ring around ``centre`` in an occupancy grid.

    Cells are passable when free or unknown (unexplored space is still a way
    on). An opening is a run of passable samples at least ``min_gap_m`` wide,
    bounded by occupied cells. A ring with no occupied sample is open ground,
    not a junction, and returns 0.
    """
    if resolution <= 0 or width <= 0 or height <= 0:
        return 0
    blocked = []
    for index in range(samples):
        angle = 2.0 * math.pi * index / samples
        x = centre[0] + radius_m * math.cos(angle)
        y = centre[1] + radius_m * math.sin(angle)
        column = math.floor((x - origin[0]) / resolution)
        row = math.floor((y - origin[1]) / resolution)
        value = -1
        if 0 <= column < width and 0 <= row < height:
            value = int(data[row * width + column])
        blocked.append(value >= occupied_threshold)
    if not any(blocked):
        return 0
    min_samples = max(1, math.ceil(min_gap_m / (2.0 * math.pi * radius_m / samples)))
    # Rotate so the sequence starts on a wall sample; then runs never wrap.
    start = blocked.index(True)
    ordered = blocked[start:] + blocked[:start]
    openings, run = 0, 0
    for is_blocked in ordered + [True]:
        if is_blocked:
            if run >= min_samples:
                openings += 1
            run = 0
        else:
            run += 1
    return openings


@dataclass(frozen=True)
class PlacementConfig:
    capacity: int = 30
    event_reserve: int = 4
    spacing_m: float = 6.0
    junction_separation_m: float = 2.5
    relay_threshold: float = 0.85
    relay_separation_m: float = 1.0
    relay_cooldown_s: float = 4.0


@dataclass(frozen=True)
class Decision:
    flags: int
    reason: str


class PlacementPolicy:
    def __init__(self, config: PlacementConfig | None = None) -> None:
        self.config = config or PlacementConfig()
        self.last_relay_at = -math.inf

    def remaining(self, deployed: int) -> int:
        return max(0, self.config.capacity - deployed)

    def can_deploy_event(self, deployed: int) -> bool:
        return self.remaining(deployed) > 0

    def decide(self, pose: Point, beacons: Iterable[Point], deployed: int,
               uplink_probability: float | None, openings: int, now: float) -> Decision | None:
        """Decide whether to drop a navigation beacon at ``pose`` right now."""
        positions = list(beacons)
        nearest = min((math.dist(pose, p) for p in positions), default=math.inf)
        remaining = self.remaining(deployed)
        c = self.config

        if not positions:
            return Decision(FLAG_ENTRANCE, "entrance anchor") if remaining > 0 else None

        relay_budget = remaining > c.event_reserve // 2
        if (relay_budget and uplink_probability is not None and uplink_probability < c.relay_threshold
                and nearest >= c.relay_separation_m and now - self.last_relay_at >= c.relay_cooldown_s):
            self.last_relay_at = now
            return Decision(FLAG_RELAY, f"radio link to mesh fading (p={uplink_probability:.2f})")

        if remaining <= c.event_reserve:
            return None
        if openings >= 3 and nearest >= c.junction_separation_m:
            return Decision(FLAG_JUNCTION, f"junction with {openings} openings")
        if nearest >= c.spacing_m:
            return Decision(FLAG_SPACING, f"no beacon within {c.spacing_m:.0f} m")
        return None
